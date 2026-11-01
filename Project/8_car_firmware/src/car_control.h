#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include <stdint.h>
#include "car_kin.h"

/*
 * 厂家（平衡小车之家 MiniBalanceV3.5）那套**串级 PID**的原样移植：
 *
 *   Balance_Pwm  = kp_b·(Angle − 中值) + kd_b·Gyro_Balance      （直立环，PD）
 *   Velocity_Pwm = Encoder_lpf·kp_v + ∫·ki_v                     （速度环，PI，正反馈）
 *   Turn_Pwm     = Turn_Target·kp_t + Gyro_Turn·kd_t             （转向环，PD）
 *   Moto1 = Balance + Velocity − Turn
 *   Moto2 = Balance + Velocity + Turn
 *
 * 厂家默认参数（10 ms 控制周期，PWM 满幅 7200、限幅 6900）：
 *   kp_b = 100, kd_b = 0.40, 中值 ZHONGZHI = 3°
 *   kp_v = 50,  ki_v = kp_v/200 = 0.25, 积分限幅 ±15000
 *   kp_t = 42,  kd_t = 0（直行时 0.6）, Turn_Amplitude = 50
 *   死区 siqu = 400
 *
 * 与厂家的差别：Movement（目标速度）和 Turn_Target 不再来自遥控器按键，
 * 而是由 (v, ω) 目标换算过来，换算系数见 car_ctrl_cfg_t。
 */

typedef struct {
    /* 差速模式：1 = 直立环不参与（moto = velocity ∓ turn），用于"只沿 x 前后/原地转" */
    int   diff_mode;           /* 0 = 自平衡（默认，行为与原版一致）, 1 = 纯差速 */

    /* 直立环 */
    float kp_balance;          /* 100 */
    float kd_balance;          /* 0.40 */
    float angle_mid_deg;       /* 3.0  —— 机械平衡点，装好后要标定 */

    /* 速度环 */
    float kp_speed;            /* 50 */
    float ki_speed;            /* 0.25 */
    float speed_int_limit;     /* 15000 */
    float speed_lpf_alpha;     /* 0.3  —— Encoder = 0.7·old + 0.3·new */

    /* 转向环 */
    float kp_turn;             /* 42 */
    float kd_turn_straight;    /* 0.6  直行时用 */
    float kd_turn_rotate;      /* 0.0  原地转时用 */
    float turn_amplitude;      /* 50 */
    float turn_ramp;           /* 0.7  每个控制周期 Turn_Target 的最大变化 */

    /* 目标换算 */
    float enc_per_mps;         /* 线速度 1 m/s 对应多少编码器计数/控制周期 */
    float turn_per_rads;       /* 角速度 1 rad/s 对应多少 Turn_Target */
    float straight_v_eps;      /* |v| 大于它算"直行"，决定用哪个 kd_turn */

    /* 差速模式专用：v (m/s) -> 两轮共同 PWM 的开环前馈。
     * 差速模式把直立环清零后，厂家那条"正反馈"速度环（int += enc − target）
     * 没有外环兜底，只要目标速度 ≠ 0 就单调发散（见 car_control.c 注释），
     * 所以差速模式不走积分，直接用这个系数把 v 换成 PWM。 */
    float diff_ff_pwm;         /* 10000：0.13 m/s ≈ 1300 PWM */

    /* 输出与保护 */
    float max_pwm;             /* 6900 */
    float deadzone;            /* 400 */
    float angle_limit_deg;     /* 40 —— 超过就关电机（相对 angle_mid） */

    /* ---- 自动找平衡中值（不接 PC，用轮子净位移当判据）---- */
    float mid_learn_k;         /* 0.010：每个 0.5 s 窗口把 angle_mid 挪 k*净位移 */
    float mid_learn_max_deg;   /* 20：angle_mid 相对开机值最多挪这么多 */
    int   auto_mid;            /* 1 = 开机自动找中值 */
} car_ctrl_cfg_t;

typedef struct {
    float speed_int;           /* 速度环积分 */
    float speed_lpf;           /* 一阶低通后的编码器和 */
    float turn_target;         /* 斜坡后的转向目标 */

    /* ---- 自动找中值的运行时状态 ---- */
    float angle_mid;           /* 当前使用的平衡中值（开机测量 + 在线学习） */
    float mid_boot;            /* 开机测到的值，用作学习限幅基准 */
    int   learn_on;            /* 正在学习 */
    int   learn_done;          /* 学习完成 */
    float learn_acc;           /* 窗口内轮子净增量累计 */
    int   learn_n;             /* 窗口内控制周期数 */
    float learn_quiet;         /* 连续“轮子几乎不动”的窗口数 */
} car_ctrl_state_t;

void car_ctrl_defaults(car_ctrl_cfg_t *cfg);
void car_ctrl_reset(car_ctrl_state_t *st);

typedef struct {
    int16_t pwm_l;             /* 已含死区、已限幅，可直接给 motor_set_pwm() */
    int16_t pwm_r;
    float   balance;
    float   velocity;
    float   turn;
    int     cut;               /* 1 = 保护停机（倾角过大或主动停） */
} car_ctrl_out_t;

/*
 * 一次控制周期（应该固定 10 ms 调用一次）：
 *   target     —— 目标线速度/角速度
 *   angle_deg  —— 直立角（IMU 互补/卡尔曼输出）
 *   gyro_balance / gyro_turn —— 对应的角速度（°/s）
 *   enc_l / enc_r —— 本周期左右编码器增量
 *   enable     —— 0 时直接停
 */
car_ctrl_out_t car_ctrl_step(car_ctrl_state_t *st, const car_ctrl_cfg_t *cfg,
                             car_tw_t target,
                             float angle_deg, float gyro_balance, float gyro_turn,
                             int16_t enc_l, int16_t enc_r,
                             int enable);

#endif /* CAR_CONTROL_H */

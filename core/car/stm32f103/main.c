#include "board.h"
#include "bsp_motor.h"
#include "bsp_encoder.h"
#include "bsp_i2c.h"
#include "bsp_mpu6050.h"
#include "bsp_uart.h"
#include "car_kin.h"
#include "car_control.h"
#include "car_proto.h"

#include <math.h>
#include <string.h>

/*
 * 主程序：5 ms 姿态更新，10 ms 控制（和厂家一致），串口协议见 car_proto.h。
 *
 * 控制链路：
 *   @MOVE <act>  --car_act_to_tw-->  (v, ω)  --\
 *   @VEL v w     -------------------> (v, ω) --> car_ctrl_step() --> motor_set_pwm()
 *   (v, ω) 由编码器里程计反推，回传给上位机 @ODOM / @STATE
 */

static volatile uint32_t g_ms;
static uint32_t g_last_cmd_ms;      /* 无线看门狗：最后一条合法指令的时刻 */

void SysTick_Handler(void) { g_ms++; }

static void systick_init(void)
{
    SysTick->LOAD = (BOARD_SYSCLK_HZ / 1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1u << 2) | (1u << 1) | (1u << 0);
}

static uint32_t millis(void) { return g_ms; }

/* ---------------- 参数 ---------------- */

#define CTRL_DT_S        0.010f      /* 控制周期 10 ms（厂家值） */
#define ATT_DT_S         0.005f      /* 姿态更新 5 ms */
#define ENC_PER_MPS      200.0f      /* 编码器计数 / (m/s) / 10ms —— 必须实测标定 */
#define WHEEL_BASE_M     0.150f      /* 轮距，量一下再改 */
#define V_ACT_MPS        0.15f       /* @MOVE 单步的线速度 */
#define K_HEAD           2.0f        /* 朝向误差 -> 角速度 的增益 */
#define V_EPS_MPS        0.02f
#define W_EPS_RADS       0.10f

/* ---------------- 状态 ---------------- */

static float g_angle_deg;            /* 直立角 */
static float g_gyro_bal;             /* 直立角速度 °/s */
static float g_gyro_turn;            /* 转向角速度 °/s */
static float g_gyro_bias[3];         /* 零偏 °/s */

static car_pose_t       g_pose = { 0.0f, 0.0f, 0.0f };
static car_tw_t         g_target = { 0.0f, 0.0f };
static uint8_t          g_act;
static int              g_enable;    /* 电机使能 */
static uint8_t          g_mode_act;  /* 1 = 用 @MOVE，0 = 用 @VEL 直给 */
static car_ctrl_cfg_t   g_cfg;
static car_ctrl_state_t g_ctrl;

/* ---------------- 串口小工具 ---------------- */

static void host_str(const char *s) { uart_host_write((const uint8_t *)s, (uint32_t)strlen(s)); }

static void host_line(char *buf, int n)
{
    if (n > 0) uart_host_write((const uint8_t *)buf, (uint32_t)n);
}

/* ---------------- 状态灯（PC13，低电平点亮） ----------------
 * 无人值守时用它报状态：
 *   开机 1 s 快闪  = 正在测平衡中值（此时请把车扶到能自然立住的位置）
 *   慢闪           = 正在学习平衡中值（轮子会小幅来回找）
 *   常亮           = 已平衡（学习完成）
 *   极快闪         = 保护停机（角度超限/收到 @STOP），需重新上电 */
static void led_init(void)
{
    RCC->APB2ENR |= (1u << 4);                 /* GPIOC 时钟 */
    gpio_cfg(GPIOC, 13, GPIO_CFG_OUT_PP_2);
    gpio_set(GPIOC, 13, 1);                    /* 灭 */
}
static void led(int on) { gpio_set(GPIOC, 13, on ? 0 : 1); }   /* 低电平亮 */

int main(void)
{
    board_clock_init();
    board_dwt_init();
    systick_init();

    uart_host_init(115200);
    uart_dbg_init(115200);

    motor_init();
    motor_stop();

    i2c_init();
    encoder_init();

    car_ctrl_defaults(&g_cfg);
    car_ctrl_reset(&g_ctrl);
    car_proto_reset();

    int mpu_ok = (mpu6050_init() == 0);
    if (mpu_ok) {
        mpu6050_calib_gyro(g_gyro_bias, 200);
    }

    host_str("@READY\n");

    /* ---- 开机自动测"平衡中值"：扶住车 1 s，取角度均值 ----
     * 目的：MPU6050 装在车上不一定水平，机械平衡点对应一个固定读数；
     * 只要模块是刚性固定的，把这个读数当中值就等价于"装正了"。 */
    led_init();
    {
        uint32_t t0 = millis();
        float sum = 0.0f;
        int   n = 0;
        while ((uint32_t)(millis() - t0) < 1000u) {
            mpu_raw_t raw;
            if (mpu_ok && mpu6050_read(&raw) == 0) {
                float ax = mpu_acc_g(raw.ax);
                float az = mpu_acc_g(raw.az);
                sum += atan2f(ax, az) * 57.29578f;
                n++;
            }
            led(((n >> 2) & 1) ? 1 : 0);        /* 快闪：正在测 */
            uint32_t t = millis();
            while ((uint32_t)(millis() - t) < 5u) { }
        }
        if (n > 0) {
            g_ctrl.angle_mid = sum / (float)n;
            g_ctrl.mid_boot  = g_ctrl.angle_mid;
            g_angle_deg      = g_ctrl.angle_mid;   /* 滤波器初值 = 实测值，省去收敛 */
            g_ctrl.learn_on  = g_cfg.auto_mid;     /* 开始在线找平衡中值 */
            g_ctrl.learn_done = 0;
            g_ctrl.learn_acc  = 0.0f;
            g_ctrl.learn_n    = 0;
            g_ctrl.learn_quiet = 0.0f;
        }
        g_enable = 1;                            /* 无人值守：测完直接进平衡 */
    }

    uint32_t next_att = millis();
    int      tick_div = 0;
    int16_t  enc_l = 0, enc_r = 0;
    uint32_t next_report = millis() + 200;
    g_last_cmd_ms = millis();

    for (;;) {
        /* ---- 无线看门狗：1 s 没收到合法指令就停（半双工丢包/断链保护）---- */
        if (g_enable && (millis() - g_last_cmd_ms) > 1000u) {
            g_enable = 0; g_act = 0; g_target.v = 0.0f; g_target.w = 0.0f;
            motor_stop(); car_ctrl_reset(&g_ctrl);
        }
        /* ---- 指令 ---- */
        int c;
        while ((c = uart_host_getc()) >= 0) {
            car_cmd_t cmd;
            if (car_proto_feed((uint8_t)c, &cmd)) {
                /*: 无线链路：收到带 seq 的命令立刻回 @ACK <seq>（半双工，一问一答），
                 *  并刷新车载看门狗（1 s 收不到合法指令就停车）。 */
                int sq = car_proto_last_seq();
                if (sq >= 0) {
                    char ab[24];
                    int an = car_proto_fmt_ackseq(ab, (int)sizeof(ab), sq);
                    host_line(ab, an);
                }
                g_last_cmd_ms = millis();
                switch (cmd.kind) {
                case CAR_CMD_MOVE:
                    g_act = cmd.act;
                    g_mode_act = 1;
                    g_enable = (cmd.act != 0);
                    if (cmd.act == 0) { g_target.v = 0.0f; g_target.w = 0.0f; }
                    break;
                case CAR_CMD_VEL:
                    g_mode_act = 0;
                    g_target.v = cmd.v / 1000.0f;      /* mm/s -> m/s */
                    g_target.w = cmd.w / 1000.0f;      /* mrad/s -> rad/s */
                    g_enable = (fabsf(g_target.v) > 0.0f || fabsf(g_target.w) > 0.0f);
                    break;
                case CAR_CMD_STOP:
                    g_enable = 0;
                    g_act = 0;
                    g_target.v = 0.0f;
                    g_target.w = 0.0f;
                    motor_stop();
                    car_ctrl_reset(&g_ctrl);
                    break;
                case CAR_CMD_CAL:
                    if (mpu_ok) mpu6050_calib_gyro(g_gyro_bias, 200);
                    break;
                default:
                    break;
                }
            }
        }

        /* ---- 5 ms 姿态 ---- */
        if ((int32_t)(millis() - next_att) >= 0) {
            next_att += 5;
            tick_div ^= 1;

            mpu_raw_t raw;
            if (mpu_ok && mpu6050_read(&raw) == 0) {
                float ax = mpu_acc_g(raw.ax);
                float az = mpu_acc_g(raw.az);
                float angle_acc = atan2f(ax, az) * 57.29578f;
                float gyro_y = mpu_gyro_dps(raw.gy) - g_gyro_bias[1];

                g_gyro_bal = -gyro_y;                       /* 厂家同一符号约定 */
                g_gyro_turn = mpu_gyro_dps(raw.gz) - g_gyro_bias[2];

                /* 互补滤波，K1 = 0.02（厂家 filter.c 的值） */
                g_angle_deg = 0.02f * angle_acc +
                              0.98f * (g_angle_deg - g_gyro_bal * ATT_DT_S);
            }

            int16_t dl = encoder_read_left_delta();
            int16_t dr = encoder_read_right_delta();
            enc_l += dl;
            enc_r += dr;

            /* 编码器 -> 车轮线速度，推进里程计（公式 13.2）
             * ENC_PER_MPS 是"10 ms 内的计数"，这里用的是 5 ms 的增量，所以乘 2 */
            const float k = (CTRL_DT_S / ATT_DT_S) / ENC_PER_MPS;
            float vl = (float)dl * k;
            float vr = (float)dr * k;
            car_odom_step(&g_pose, vl, vr, ATT_DT_S, WHEEL_BASE_M);

            /* ---- 10 ms 控制（每两次姿态更新一次）---- */
            if (tick_div == 0) {
                if (g_mode_act && g_act != 0) {
                    g_target = car_act_to_tw(g_act, g_pose.theta, V_ACT_MPS, K_HEAD);
                }

                car_ctrl_out_t o = car_ctrl_step(&g_ctrl, &g_cfg, g_target,
                                                 g_angle_deg, g_gyro_bal, g_gyro_turn,
                                                 enc_l, enc_r, g_enable);
                if (o.cut) {
                    motor_stop();
                    g_enable = 0;                    /* 保护后保持停机，需重新上电 */
                    led(((millis() / 50u) & 1u) ? 1 : 0);   /* 极快闪 */
                } else {
                    motor_set_pwm(o.pwm_l, o.pwm_r);
                    if (g_ctrl.learn_on) led(((millis() / 200u) & 1u) ? 1 : 0);  /* 慢闪 */
                    else                 led(1);                                /* 常亮 */
                }
                enc_l = 0;
                enc_r = 0;
            }
        }

        /* ---- 5 Hz 上报 ---- */
        if ((int32_t)(millis() - next_report) >= 0) {
            next_report += 200;
            char buf[96];

            int n = car_proto_fmt_state(buf, (int)sizeof(buf),
                                        g_angle_deg * 10.0f,      /* 0.1° 单位，输出带 1 位小数 */
                                        g_target.v * 1000.0f,     /* mm/s */
                                        g_target.w * 1000.0f,     /* mrad/s */
                                        g_act);
            host_line(buf, n);

            n = car_proto_fmt_odom(buf, (int)sizeof(buf), g_pose);
            host_line(buf, n);
        }
    }
}

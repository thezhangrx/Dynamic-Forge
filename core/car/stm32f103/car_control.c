#include "car_control.h"

#include <math.h>

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

void car_ctrl_defaults(car_ctrl_cfg_t *cfg)
{
    cfg->kp_balance        = 100.0f;
    cfg->kd_balance        = 0.40f;
    cfg->angle_mid_deg     = 3.0f;

    cfg->kp_speed          = 50.0f;
    cfg->ki_speed          = 0.25f;
    cfg->speed_int_limit   = 15000.0f;
    cfg->speed_lpf_alpha   = 0.3f;

    cfg->kp_turn           = 42.0f;
    cfg->kd_turn_straight  = 0.6f;
    cfg->kd_turn_rotate    = 0.0f;
    cfg->turn_amplitude    = 50.0f;
    cfg->turn_ramp         = 0.7f;

    /* 这两个是"单位换算"，默认值只是量级正确，必须实测标定：
       enc_per_mps：把车以 1 m/s 推着走，看 10 ms 内编码器计数增量。 */
    cfg->enc_per_mps       = 200.0f;
    cfg->turn_per_rads     = 20.0f;
    cfg->straight_v_eps    = 0.02f;
    /* 差速模式：v -> 两轮共同 PWM。取 10000 是为了和厂家速度环的稳态增益一致
     * （厂家稳态 velocity ≈ speed_lpf*kp = v*enc_per_mps*50 ≈ v*10000）。 */
    cfg->diff_ff_pwm       = 10000.0f;

    cfg->max_pwm           = 6900.0f;
    cfg->deadzone          = 400.0f;
    cfg->angle_limit_deg   = 40.0f;

    /* 自动找中值：默认开。k 取小值慢慢逼近平滑，限幅 20° 防止乱跑 */
    cfg->mid_learn_k       = 0.010f;
    cfg->mid_learn_max_deg = 20.0f;
    cfg->auto_mid          = 1;
    cfg->diff_mode         = 0;   /* 默认自平衡；@MODE 可切差速 */
}

void car_ctrl_reset(car_ctrl_state_t *st)
{
    st->speed_int   = 0.0f;
    st->speed_lpf   = 0.0f;
    st->turn_target = 0.0f;
    /* 注意：不动 angle_mid / learn_* —— 它们是“开机自测 + 在线学习”的状态，
     * 收 @STOP 重新复位时不能把辛苦学到的中值丢掉。 */
}

car_ctrl_out_t car_ctrl_step(car_ctrl_state_t *st, const car_ctrl_cfg_t *cfg,
                             car_tw_t target,
                             float angle_deg, float gyro_balance, float gyro_turn,
                             int16_t enc_l, int16_t enc_r,
                             int enable)
{
    car_ctrl_out_t out = { 0, 0, 0.0f, 0.0f, 0.0f, 0 };

    /* ---------- 保护：倾角（相对平衡中值）超过阈值 / 主动停机 ---------- */
    float bias = angle_deg - st->angle_mid;
    if (!enable || fabsf(bias) > cfg->angle_limit_deg) {
        out.cut = 1;
        st->speed_int = 0.0f;          /* 电机关掉后清积分，和厂家一致 */
        return out;
    }

    /* ---------- 自动找平衡中值 ----------
     * 判据：直立环让车"立"住时，轮子的净位移应为 0。
     *   轮子持续往前滚 ⇒ 当前 angle_mid 比真实平衡点小 ⇒ 把中值往上抬；
     * 每 0.5 s 用窗口内 (enc_l+enc_r) 的均值微调一次，连续 3 个窗口"不动"就算收敛。
     * 学习期间把直立环输出限到 25%，防止一开始中值差太多车直接窜出去。 */
    if (st->learn_on) {
        st->learn_acc += 0.5f * ((float)enc_l + (float)enc_r);
        st->learn_n += 1;
        if (st->learn_n >= 50) {
            float drift = st->learn_acc / 50.0f;
            float lo = st->mid_boot - cfg->mid_learn_max_deg;
            float hi = st->mid_boot + cfg->mid_learn_max_deg;
            st->angle_mid += cfg->mid_learn_k * drift;
            if (st->angle_mid < lo) st->angle_mid = lo;
            if (st->angle_mid > hi) st->angle_mid = hi;
            if (fabsf(drift) < 3.0f) st->learn_quiet += 1.0f;
            else                     st->learn_quiet  = 0.0f;
            if (st->learn_quiet >= 3.0f) { st->learn_on = 0; st->learn_done = 1; }
            st->learn_acc = 0.0f;
            st->learn_n = 0;
        }
    }

    /* ---------- 直立环（PD）---------- */
    out.balance = cfg->kp_balance * bias + cfg->kd_balance * gyro_balance;
    if (st->learn_on) {
        float lim = cfg->max_pwm * 0.25f;
        if (out.balance >  lim) out.balance =  lim;
        if (out.balance < -lim) out.balance = -lim;
    }

    /* ---------- 速度环（PI，正反馈）---------- */
    float sum = (float)enc_l + (float)enc_r;
    st->speed_lpf += cfg->speed_lpf_alpha * (sum - st->speed_lpf);

    float movement = target.v * cfg->enc_per_mps;      /* 目标速度换算成编码器计数 */

    if (cfg->diff_mode) {
        /* 差速模式没有直立环兜底：厂家这条速度环是**正反馈**
         * （int += enc − target），只要目标速度 ≠ 0、编码器跟不上，
         * 积分就单调发散：仿真/实测 0.3~0.8 s 顶到 −15000，
         * 输出 velocity = lpf*50 + int*0.25 → −17000 PWM，
         * 两轮一起被拉到反向满转 —— "按哪个键都反转"就是这个。
         * 差速模式改成开环前馈：不积分、不看编码器，v 直接换成共同 PWM。
         * 注意：diff_mode=0（默认/原基线）仍然走下面的厂家原逻辑。 */
        out.velocity = target.v * cfg->diff_ff_pwm;
        st->speed_int = 0.0f;
    } else {
        st->speed_int += st->speed_lpf;
        st->speed_int -= movement;
        st->speed_int = clampf(st->speed_int, -cfg->speed_int_limit, cfg->speed_int_limit);
        out.velocity = st->speed_lpf * cfg->kp_speed + st->speed_int * cfg->ki_speed;
    }

    /* ---------- 转向环（PD）---------- */
    float turn_des = target.w * cfg->turn_per_rads;
    turn_des = clampf(turn_des, -cfg->turn_amplitude, cfg->turn_amplitude);

    /* 厂家那个"起步限速"斜坡：每个周期最多变 turn_ramp */
    if (turn_des > st->turn_target + cfg->turn_ramp)      st->turn_target += cfg->turn_ramp;
    else if (turn_des < st->turn_target - cfg->turn_ramp) st->turn_target -= cfg->turn_ramp;
    else                                                  st->turn_target  = turn_des;

    float kd = (fabsf(target.v) > cfg->straight_v_eps) ? cfg->kd_turn_straight
                                                       : cfg->kd_turn_rotate;
    out.turn = st->turn_target * cfg->kp_turn + gyro_turn * kd;

    /* ---------- 合成两轮 PWM ---------- */
    /* 差速模式：把直立环输出清零，两轮变成 velocity ∓ turn（等速反向可原地转） */
    float bal = cfg->diff_mode ? 0.0f : out.balance;
    float moto1 = bal + out.velocity - out.turn;    /* 左 */
    float moto2 = bal + out.velocity + out.turn;    /* 右 */

    moto1 = clampf(moto1, -cfg->max_pwm, cfg->max_pwm);
    moto2 = clampf(moto2, -cfg->max_pwm, cfg->max_pwm);

    /* 厂家 Set_Pwm 里给占空比加的死区（补偿电机的启动死区） */
    out.pwm_l = (int16_t)(fabsf(moto1) + cfg->deadzone) * (moto1 < 0.0f ? -1 : 1);
    out.pwm_r = (int16_t)(fabsf(moto2) + cfg->deadzone) * (moto2 < 0.0f ? -1 : 1);
    out.cut = 0;
    return out;
}

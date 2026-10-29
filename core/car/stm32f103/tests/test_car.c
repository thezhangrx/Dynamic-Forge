/*
 * 主机上跑的单元测试（不涉及硬件寄存器）：
 *   gcc -std=c11 -I.. -o /tmp/test_car test_car.c ../car_kin.c ../car_control.c ../car_proto.c -lm
 *
 * 覆盖：act<->(v,ω) 互转、轮速换算、公式 13.2 里程计、协议解析与格式化、
 *       串级 PID 的保护与积分限幅。
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "car_kin.h"
#include "car_control.h"
#include "car_proto.h"

static int g_fail;
static int g_total;

#define OK(cond, fmt, ...) do {                                   \
    g_total++;                                                    \
    if (cond) { printf("  ok   " fmt "\n", ##__VA_ARGS__); }      \
    else      { printf("  FAIL " fmt "\n", ##__VA_ARGS__); g_fail++; } \
} while (0)

#define NEAR(a, b, tol) (fabsf((a) - (b)) < (tol))

static void test_act_heading(void)
{
    printf("[act -> 世界方向]\n");
    car_tw_t t;

    /* act 1 = -x -> π；act 2 = +x -> 0；act 3 = +y -> +π/2（页面 y 向下） */
    t = car_act_to_tw(2, 0.0f, 1.0f, 2.0f);
    OK(NEAR(t.v, 1.0f, 1e-3f) && NEAR(t.w, 0.0f, 1e-3f), "act2 从 0 朝上直走");

    t = car_act_to_tw(1, 0.0f, 1.0f, 2.0f);
    OK(NEAR(t.v, 0.0f, 1e-3f) && t.w > 1.0f, "act1 目标在正后方 -> 原地转");

    t = car_act_to_tw(3, 0.0f, 1.0f, 2.0f);
    OK(t.v == 0.0f && t.w > 0.0f, "act3 目标在 90° -> 先原地转（v=%.2f ω=%.2f）", t.v, t.w);

    /* 转过去一点之后应该边走边转 */
    t = car_act_to_tw(3, 0.6f, 1.0f, 2.0f);
    OK(t.v > 0.0f && t.w > 0.0f, "act3 朝向接近后 -> 边走边转（v=%.2f ω=%.2f）", t.v, t.w);

    t = car_act_to_tw(0, 1.0f, 1.0f, 2.0f);
    OK(NEAR(t.v, 0.0f, 1e-6f) && NEAR(t.w, 0.0f, 1e-6f), "act0 停车");
}

static void test_roundtrip(void)
{
    printf("[act -> (v,ω) -> act 往返]\n");
    const float thetas[] = { 0.0f, 0.7f, -0.7f, 1.5f, -1.5f, 3.0f, -3.0f };
    int bad = 0, n = 0;

    for (unsigned i = 0; i < sizeof(thetas) / sizeof(thetas[0]); i++) {
        for (int act = 1; act <= 8; act++) {
            car_tw_t t = car_act_to_tw((uint8_t)act, thetas[i], 0.5f, 3.0f);
            uint8_t back = car_tw_to_act(t, thetas[i], 3.0f, 0.01f, 0.01f);
            n++;
            if (back != act) {
                bad++;
                printf("       theta=%.2f act=%d -> back=%d\n", thetas[i], act, back);
            }
        }
    }
    OK(bad == 0, "%d 个组合往返一致（坏 %d 个）", n, bad);

    uint8_t stop = car_tw_to_act((car_tw_t){ 0.0f, 0.0f }, 1.0f, 3.0f, 0.01f, 0.01f);
    OK(stop == 0, "零速度 -> act0");
}

static void test_wheels(void)
{
    printf("[轮速换算]\n");
    float vl, vr;
    const float L = 0.15f;

    car_tw_to_wheels((car_tw_t){ 1.0f, 0.5f }, L, &vl, &vr);
    OK(NEAR(vl, 1.0f - 0.5f * 0.15f / 2.0f, 1e-4f), "左轮 = v - ωL/2 = %.4f", vl);
    OK(NEAR(vr, 1.0f + 0.5f * 0.15f / 2.0f, 1e-4f), "右轮 = v + ωL/2 = %.4f", vr);

    car_tw_t back = car_wheels_to_tw(vl, vr, L);
    OK(NEAR(back.v, 1.0f, 1e-4f) && NEAR(back.w, 0.5f, 1e-4f),
       "反算回去 v=%.4f ω=%.4f", back.v, back.w);
}

static void test_odom(void)
{
    printf("[公式 13.2 里程计]\n");
    car_pose_t p = { 0.0f, 0.0f, 0.0f };

    /* 直走 1 m：左右轮同速 1 m/s，1 秒 */
    for (int i = 0; i < 100; i++) car_odom_step(&p, 1.0f, 1.0f, 0.01f, 0.15f);
    OK(NEAR(p.x, 1.0f, 2e-3f) && NEAR(p.y, 0.0f, 2e-3f) && NEAR(p.theta, 0.0f, 1e-3f),
       "直行 x=%.4f y=%.4f θ=%.4f", p.x, p.y, p.theta);

    /* 原地转 90°：左 -ωL/2，右 +ωL/2，ω = π/2 rad/s，0.1 s... 用 1 s 转 π rad/s */
    p = (car_pose_t){ 0.0f, 0.0f, 0.0f };
    const float w = (float)M_PI / 2.0f;
    const float L = 0.15f;
    for (int i = 0; i < 100; i++) {
        car_odom_step(&p, -w * L / 2.0f, w * L / 2.0f, 0.01f, L);
    }
    OK(NEAR(p.theta, (float)M_PI / 2.0f, 5e-3f) && NEAR(p.x, 0.0f, 1e-3f) && NEAR(p.y, 0.0f, 1e-3f),
       "原地转 θ=%.4f（应 %.4f）x=%.5f y=%.5f", p.theta, M_PI / 2.0, p.x, p.y);
}

static void test_proto(void)
{
    printf("[协议解析/格式化]\n");
    car_cmd_t c;
    car_proto_reset();

    const char *m = "@MOVE 6\n";
    int got = 0;
    for (const char *p = m; *p; p++) if (car_proto_feed((uint8_t)*p, &c)) got = 1;
    OK(got && c.kind == CAR_CMD_MOVE && c.act == 6, "@MOVE 6 -> act 6");

    const char *v = "@VEL 150 -250\r\n";
    got = 0;
    for (const char *p = v; *p; p++) if (car_proto_feed((uint8_t)*p, &c)) got = 1;
    OK(got && c.kind == CAR_CMD_VEL && NEAR(c.v, 150.0f, 1e-3f) && NEAR(c.w, -250.0f, 1e-3f),
       "@VEL 150 -250 -> v=%.1f w=%.1f", c.v, c.w);

    const char *s = "@STOP\n";
    got = 0;
    for (const char *p = s; *p; p++) if (car_proto_feed((uint8_t)*p, &c)) got = 1;
    OK(got && c.kind == CAR_CMD_STOP, "@STOP");

    /* 格式化 -> 再解析，验证自洽 */
    char buf[96];
    int n = car_proto_fmt_odom(buf, sizeof(buf), (car_pose_t){ 1.234f, -0.5f, 0.785f });
    car_cmd_t dummy;
    car_proto_reset();
    (void)dummy;
    printf("     odom 输出: %s", buf);
    OK(n > 0 && strncmp(buf, "@ODOM 1234 -500 785", 19) == 0, "odom 格式化正确");

    /* 第一个参数是 0.1°，-12.5 就是 -1.25°，四舍五入成 -1.3 */
    n = car_proto_fmt_state(buf, sizeof(buf), -12.5f, 150.0f, -250.0f, 6);
    printf("     state 输出: %s", buf);
    OK(n > 0 && strncmp(buf, "@STATE -1.3 150 -250 6", 22) == 0, "state 格式化正确");

    n = car_proto_fmt_ack(buf, sizeof(buf), "MOVE", 6.0f, 0.0f);
    printf("     ack 输出: %s", buf);
    OK(n > 0, "ack 格式化");
}

/* 夹具：固件开机时会把 angle_mid 设成实测平衡点、learn_on 置位，
 * 这里照着做（car_ctrl_reset 有意不动这几个字段）。不初始化 st 的 learn_* 会让
 * 自动找中值那段用栈垃圾跑，静态就"倾角超限"→ cut=1，原来是假 FAIL。 */
static void fixture(car_ctrl_state_t *st, const car_ctrl_cfg_t *cfg)
{
    car_ctrl_reset(st);
    st->angle_mid    = cfg->angle_mid_deg;
    st->mid_boot     = cfg->angle_mid_deg;
    st->learn_on     = 0;
    st->learn_done   = 1;
    st->learn_acc    = 0.0f;
    st->learn_n      = 0;
    st->learn_quiet  = 0.0f;
}

static void test_control(void)
{
    printf("[串级 PID]\n");
    car_ctrl_cfg_t cfg;
    car_ctrl_state_t st;
    car_ctrl_defaults(&cfg);
    fixture(&st, &cfg);

    /* 倾角超过阈值 -> 保护停机 */
    car_ctrl_out_t o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0, 0 },
                                     45.0f, 0.0f, 0.0f, 0, 0, 1);
    OK(o.cut == 1, "倾角 45° -> 停机");

    /* 静止且角度在平衡点 -> 输出接近 0（只有残余积分） */
    fixture(&st, &cfg);
    o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0, 0 }, cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    OK(!o.cut && abs(o.pwm_l) <= (int)cfg.deadzone + 1 && abs(o.pwm_r) <= (int)cfg.deadzone + 1,
       "平衡点静止 -> 只有死区 pwm_l=%d pwm_r=%d", o.pwm_l, o.pwm_r);

    /* 角度偏 10° -> 直立环输出 ~1000 */
    fixture(&st, &cfg);
    o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0, 0 }, cfg.angle_mid_deg + 10.0f, 0.0f, 0.0f, 0, 0, 1);
    OK(NEAR(o.balance, 1000.0f, 1.0f), "偏 10° 直立环 = %.1f（kp=100）", o.balance);

    /* 目标速度为正 -> 积分项朝负方向（厂家 Movement 是减的） */
    fixture(&st, &cfg);
    for (int i = 0; i < 50; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.5f, 0.0f }, cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(st.speed_int < 0.0f, "目标前进 -> 速度环积分 %.1f 为负（带动车往前）", st.speed_int);

    /* 积分限幅 */
    fixture(&st, &cfg);
    for (int i = 0; i < 5000; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.5f, 0.0f }, cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(fabsf(st.speed_int) <= cfg.speed_int_limit + 1e-3f,
       "积分限幅在 ±%.0f（实际 %.1f）", cfg.speed_int_limit, st.speed_int);

    /* 转向：给正角速度，左右轮 PWM 应该一正一负地差开 */
    fixture(&st, &cfg);
    for (int i = 0; i < 200; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.0f, 1.0f }, cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(o.turn > 0.0f && (int)o.pwm_r != (int)o.pwm_l, "转向 ω=1 -> turn=%.1f 左右差开", o.turn);
}

/* 差速模式（@MODE 1）：两轮同向、靠差速转，并且**不许发散** */
static void test_diff_mode(void)
{
    printf("[差速模式 @MODE 1]\n");
    car_ctrl_cfg_t cfg;
    car_ctrl_state_t st;
    car_ctrl_defaults(&cfg);
    cfg.diff_mode = 1;

    fixture(&st, &cfg);

    /* v=0.13 m/s（前进）+ ω=-0.6 rad/s（右转）：两轮都该正转，左快右慢 */
    car_ctrl_out_t o = { 0, 0, 0, 0, 0, 0 };
    for (int i = 0; i < 200; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.13f, -0.6f },
                          cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(o.pwm_l > 0 && o.pwm_r > 0, "两轮都正转 pwm_l=%d pwm_r=%d", o.pwm_l, o.pwm_r);
    OK(o.pwm_l > o.pwm_r, "右转 -> 左轮更快（%d > %d）", o.pwm_l, o.pwm_r);
    OK(NEAR(st.speed_int, 0.0f, 1e-6f), "差速模式不积分 speed_int=%.1f", st.speed_int);

    /* 轮子被摩擦/纸片卡住（编码器恒 0）跑 5 s：不许像厂家速度环那样发散 */
    fixture(&st, &cfg);
    for (int i = 0; i < 500; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.13f, -0.6f },
                          cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(o.pwm_l > 0 && o.pwm_r > 0, "卡住 5 s 仍两轮正转 pwm_l=%d pwm_r=%d", o.pwm_l, o.pwm_r);

    /* v=0 + ω：保留"两轮反向原地转" */
    fixture(&st, &cfg);
    for (int i = 0; i < 200; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.0f, 0.6f },
                          cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(o.pwm_l < 0 && o.pwm_r > 0, "原地转 pwm_l=%d pwm_r=%d", o.pwm_l, o.pwm_r);

    /* 回归：diff_mode=0 必须还是厂家原逻辑（这条正反馈仍然会发散，别被"顺手修好"） */
    cfg.diff_mode = 0;
    fixture(&st, &cfg);
    for (int i = 0; i < 200; i++) {
        o = car_ctrl_step(&st, &cfg, (car_tw_t){ 0.13f, 0.0f },
                          cfg.angle_mid_deg, 0.0f, 0.0f, 0, 0, 1);
    }
    OK(st.speed_int < -1000.0f, "diff_mode=0 仍走厂家积分（int=%.0f）", st.speed_int);
}

int main(void)
{
    printf("=== car_kin / car_control / car_proto 单元测试 ===\n");
    test_act_heading();
    test_roundtrip();
    test_wheels();
    test_odom();
    test_proto();
    test_control();
    test_diff_mode();

    printf("\n=== %d 项，失败 %d 项 ===\n", g_total, g_fail);
    return g_fail ? 1 : 0;
}

#include "car_kin.h"

#include <math.h>

#define CAR_PI 3.14159265358979323846f

/* 与 core/board/tools/virt_core.py 的 DIRS 一致（页面坐标，y 向下） */
static const float k_dirs[9][2] = {
    { 0.0f,  0.0f},   /* 0 停 */
    {-1.0f,  0.0f},   /* 1 -x */
    { 1.0f,  0.0f},   /* 2 +x */
    { 0.0f,  1.0f},   /* 3 +y */
    { 0.0f, -1.0f},   /* 4 -y */
    {-1.0f,  1.0f},   /* 5 -x +y */
    { 1.0f,  1.0f},   /* 6 +x +y */
    {-1.0f, -1.0f},   /* 7 -x -y */
    { 1.0f, -1.0f},   /* 8 +x -y */
};

void car_act_dir(uint8_t act, float *dx, float *dy)
{
    if (act > 8) act = 0;
    *dx = k_dirs[act][0];
    *dy = k_dirs[act][1];
}

float car_wrap_pi(float a)
{
    while (a >  CAR_PI) a -= 2.0f * CAR_PI;
    while (a <= -CAR_PI) a += 2.0f * CAR_PI;
    return a;
}

float car_act_heading(uint8_t act, float theta)
{
    if (act == 0 || act > 8) return theta;
    float dx, dy;
    car_act_dir(act, &dx, &dy);
    if (dx == 0.0f && dy == 0.0f) return theta;
    return atan2f(dy, dx);          /* 页面上 y 向下，所以正角度是顺时针 */
}

car_tw_t car_act_to_tw(uint8_t act, float theta, float v_nom, float k_head)
{
    car_tw_t out = { 0.0f, 0.0f };

    if (act == 0 || act > 8) return out;

    float psi = car_act_heading(act, theta);
    float e   = car_wrap_pi(psi - theta);

    /* 目标方向和车头夹角超过 60°：先原地转，不往前拱。
     * （用 60° 而不是 90°，是因为 cos≈0 时 float 会正负抖动） */
    float c = cosf(e);
    out.v = (c > 0.5f) ? v_nom * c : 0.0f;
    out.w = k_head * e;
    return out;
}

uint8_t car_tw_to_act(car_tw_t tw, float theta, float k_head,
                      float v_eps, float w_eps)
{
    int stopped = (fabsf(tw.v) < v_eps) && (fabsf(tw.w) < w_eps);
    if (stopped) return 0;

    float e;
    if (k_head > 1e-6f) e = tw.w / k_head;
    else                e = 0.0f;

    float psi = theta + e;
    if (tw.v < 0.0f) psi += CAR_PI;          /* 倒着走 = 反方向 */

    /* 量化到 8 个方向里最近的；注意页面坐标下角度顺时针为正 */
    int   best   = 0;
    float best_d = 1e9f;
    for (int act = 1; act <= 8; act++) {
        float d = car_wrap_pi(psi - car_act_heading((uint8_t)act, 0.0f));
        if (d < 0.0f) d = -d;
        if (d < best_d) { best_d = d; best = act; }
    }
    return (uint8_t)best;
}

void car_tw_to_wheels(car_tw_t tw, float wheel_base, float *v_l, float *v_r)
{
    float half = 0.5f * wheel_base * tw.w;
    *v_l = tw.v - half;
    *v_r = tw.v + half;
}

car_tw_t car_wheels_to_tw(float v_l, float v_r, float wheel_base)
{
    car_tw_t out;
    out.v = 0.5f * (v_l + v_r);
    out.w = (wheel_base > 1e-6f) ? (v_r - v_l) / wheel_base : 0.0f;
    return out;
}

void car_odom_step(car_pose_t *p, float v_l, float v_r, float dt, float wheel_base)
{
    car_tw_t tw = car_wheels_to_tw(v_l, v_r, wheel_base);

    /* 中点法：用积分前后的平均朝向，减小大转弯时的误差 */
    float th_mid = p->theta + 0.5f * tw.w * dt;

    p->x += tw.v * cosf(th_mid) * dt;
    p->y += tw.v * sinf(th_mid) * dt;
    p->theta = car_wrap_pi(p->theta + tw.w * dt);
}

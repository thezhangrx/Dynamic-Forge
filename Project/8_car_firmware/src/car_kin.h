#ifndef CAR_KIN_H
#define CAR_KIN_H

#include <stdint.h>

/*
 * 仿真指令 (@MOVE <act>) 与差速指令 (线速度 v + 角速度 ω) 的互转，
 * 以及差速底盘的运动学。
 *
 * 公式来源：docs/reference/公式.md
 *   公式 13.2（差速驱动机器人模型）
 *       ẋ = r/2 (u_l + u_r) cosθ
 *       ẏ = r/2 (u_l + u_r) sinθ
 *       θ̇ = r/L (u_r - u_l)
 *   其中 u_l/u_r 是左右轮的**角速度**，r 轮半径，L 轮距。
 *   换成轮子线速度 v_l = r·u_l、v_r = r·u_r 后：
 *       v = (v_l + v_r) / 2,   ω = (v_r - v_l) / L
 *
 * 坐标系与仿真一致（页面坐标）：x 向右、y 向下，θ = atan2(ẏ, ẋ)，
 * 因此 θ 增大 = 顺时针（从上往下看）。
 *
 * 动作编号沿用仿真（core/board/tools/virt_core.py 的 DIRS）：
 *   0 停  1 −x  2 +x  3 +y  4 −y  5(−x,+y)  6(+x,+y)  7(−x,−y)  8(+x,−y)
 */

typedef struct {
    float x;        /* 位置，单位与 v 一致（仿真里用像素，实车可用米） */
    float y;
    float theta;    /* 朝向，弧度 */
} car_pose_t;

typedef struct {
    float v;        /* 线速度 */
    float w;        /* 角速度，弧度/秒，正 = 顺时针（页面坐标） */
} car_tw_t;

/* act -> 单位方向向量（页面坐标）。act 非法或 0 时返回 (0,0)。 */
void car_act_dir(uint8_t act, float *dx, float *dy);

/* act -> 世界方向角（弧度），act=0 时返回当前朝向 theta */
float car_act_heading(uint8_t act, float theta);

/* 角度归一化到 (-π, π] */
float car_wrap_pi(float a);

/*
 * act -> (v, ω)：给定当前朝向 theta，把"往某个世界方向走"翻译成
 * 差速底盘的动作。ε = wrap(ψ_act − θ)；v = v_nom·cos ε（ε 大于 90° 时取 0，
 * 原地转），ω = k_head·ε。k_head 越大转得越急。
 */
car_tw_t car_act_to_tw(uint8_t act, float theta, float v_nom, float k_head);

/*
 * (v, ω) -> act：car_act_to_tw 的反变换。ε = ω / k_head，ψ = θ + ε；
 * v < 0 时按"倒着走"理解，方向取反 π。取最接近的 8 个方向之一；
 * 速度与角速度都接近 0 时返回 0。
 */
uint8_t car_tw_to_act(car_tw_t tw, float theta, float k_head,
                      float v_eps, float w_eps);

/* (v, ω) <-> 左右轮线速度 */
void   car_tw_to_wheels(car_tw_t tw, float wheel_base, float *v_l, float *v_r);
car_tw_t car_wheels_to_tw(float v_l, float v_r, float wheel_base);

/* 公式 13.2 的离散积分：给定左右轮线速度和 dt，推进位姿（中点法） */
void car_odom_step(car_pose_t *p, float v_l, float v_r, float dt, float wheel_base);

#endif /* CAR_KIN_H */

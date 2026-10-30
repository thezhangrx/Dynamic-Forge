#ifndef CAR_BSP_MOTOR_H
#define CAR_BSP_MOTOR_H

#include <stdint.h>
#include "board.h"

/* PWM 分辨率：TIM1 ARR=7199，72 MHz / 7200 = 10 kHz（与厂家一致） */
#define MOTOR_PWM_ARR   7199u
#define MOTOR_OUT_MAX   1000   /* 逻辑满量程，便于和 PID 输出对接 */

void motor_init(void);

/* a = 左轮, b = 右轮；范围 -1000..+1000，正数表示"向前"（电机 B 视装配可能相反） */
void motor_set(int16_t a, int16_t b);

/* 直接给 PWM 计数（和厂家 PID 输出的量纲一致，范围约 -7200..+7200，
 * 死区由控制层自己加）。正数 = AIN1/BIN1 为高 */
void motor_set_pwm(int16_t a, int16_t b);

/* 立即停：方向脚全 0 + 占空比 0（TB6612 的滑行停） */
void motor_stop(void);

/* 刹车：两个方向脚同时拉高（TB6612 的短接刹车） */
void motor_brake(void);

#endif /* CAR_BSP_MOTOR_H */

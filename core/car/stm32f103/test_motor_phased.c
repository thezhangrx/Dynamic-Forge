/*
 * 分相诊断固件：把 A / B 两路**分开**驱动，每相位 5 秒
 *
 *   相位 1: 只有 A 正转   (TIM3_CH4 / PB1, AIN1=PB14=0, AIN2=PB15=1)
 *   相位 2: 只有 B 正转   (TIM3_CH3 / PB0, BIN1=PB13=1, BIN2=PB12=0)
 *   相位 3: 只有 A 反转
 *   相位 4: 只有 B 反转
 *   相位 5: 全停 2 秒，然后循环
 *
 * 配合 check_hw.sh：每相位跑一次，看 CCR3/CCR4 到底哪个是 2000、
 * ODR 到底哪两位是 1 —— 就能确认 MCU 是否真的两路都在输出。
 */
#include "board.h"
#include "bsp_motor.h"

#define DUTY 2000

static volatile uint32_t g_ms;

void SysTick_Handler(void) { g_ms++; }

static void delay_ms(uint32_t n)
{
    uint32_t t = g_ms;
    while ((uint32_t)(g_ms - t) < n) { }
}

int main(void)
{
    board_clock_init();
    board_dwt_init();

    SysTick->LOAD = (BOARD_SYSCLK_HZ / 1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1u << 2) | (1u << 1) | (1u << 0);

    motor_init();
    motor_stop();
    delay_ms(1000);

    for (;;) {
        /* 相位 1：只有 A 正转 */
        motor_set_pwm(DUTY, 0);
        delay_ms(5000);
        motor_stop();
        delay_ms(500);

        /* 相位 2：只有 B 正转 */
        motor_set_pwm(0, DUTY);
        delay_ms(5000);
        motor_stop();
        delay_ms(500);

        /* 相位 3：只有 A 反转 */
        motor_set_pwm(-DUTY, 0);
        delay_ms(5000);
        motor_stop();
        delay_ms(500);

        /* 相位 4：只有 B 反转 */
        motor_set_pwm(0, -DUTY);
        delay_ms(5000);
        motor_stop();
        delay_ms(2000);
    }
}

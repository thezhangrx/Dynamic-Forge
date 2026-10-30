/*
 * 电机"持续输出"测试固件（用于配合 check_hw.sh 做分段定位）
 *
 * 行为：正转 DUTY 保持 5 秒 -> 反转 DUTY 保持 5 秒 -> 循环往复。
 * 目的：固件运行期间，用 check_hw.sh 读到
 *         TIM3->CCR3/CCR4 = DUTY
 *         GPIOB->ODR      = 方向脚电平
 *       就能证明"STM32 端已经把信号发出去了"；
 *       若此时电机仍不转，问题在 D153C 侧（STBY/VM/电机线）。
 *
 * D153C 必查：STBY 接 3V3；VIN 接电池并且 SW1 打开；电机插在 Motor_A/Motor_B。
 */
#include "board.h"
#include "bsp_motor.h"

#define DUTY  2000       /* 7200 满量程，2000 ≈ 28%，足够让空载轮子明显转 */

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
    delay_ms(500);

    for (;;) {
        motor_set_pwm(DUTY, DUTY);       /* 正转 */
        delay_ms(5000);
        motor_stop();
        delay_ms(300);
        motor_set_pwm(-DUTY, -DUTY);     /* 反转 */
        delay_ms(5000);
        motor_stop();
        delay_ms(300);
    }
}

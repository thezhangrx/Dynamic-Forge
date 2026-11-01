/*
 * 单路 100% 满占空比测试：用于**在驱动板输出端**量电压，判定哪一路没有输出
 *
 *   相位 1（10 秒）：A 路 PWMA=100%，正转   -> AO1 ≈ VM, AO2 ≈ 0
 *   停 2 秒
 *   相位 2（10 秒）：B 路 PWMB=100%，正转   -> BO1 ≈ VM, BO2 ≈ 0
 *   停 2 秒
 *   相位 3（10 秒）：A 路反转               -> AO1 ≈ 0,  AO2 ≈ VM
 *   停 2 秒
 *   相位 4（10 秒）：B 路反转               -> BO1 ≈ 0,  BO2 ≈ VM
 *   停 2 秒 -> 循环
 *
 * 100% 占空比 = 直流，万用表直接读数（预期 ≈ 电池电压）。
 */
#include "board.h"
#include "bsp_motor.h"

#define DUTY_FULL 7200      /* ARR=7199，写满 = 100% */

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
    delay_ms(1500);

    for (;;) {
        motor_set_pwm(DUTY_FULL, 0);        /* A 正转 100% */
        delay_ms(10000);
        motor_stop();
        delay_ms(2000);

        motor_set_pwm(0, DUTY_FULL);        /* B 正转 100% */
        delay_ms(10000);
        motor_stop();
        delay_ms(2000);

        motor_set_pwm(-DUTY_FULL, 0);       /* A 反转 100% */
        delay_ms(10000);
        motor_stop();
        delay_ms(2000);

        motor_set_pwm(0, -DUTY_FULL);       /* B 反转 100% */
        delay_ms(10000);
        motor_stop();
        delay_ms(2000);
    }
}

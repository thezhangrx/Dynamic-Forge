/*
 * 恒定输出固件：两路都正转，100% 占空比，一直保持
 *
 * 用途：每个信号脚都是稳定的直流电平，用万用表随时量都一样，
 *       不受相位/时机影响。
 *
 * 上电后应该量到（都相对 GND）：
 *   PWMA(PB1)  = 3.3 V      AIN1(PB14) = 0 V      AIN2(PB15) = 3.3 V
 *   PWMB(PB0)  = 3.3 V      BIN1(PB13) = 3.3 V    BIN2(PB12) = 0 V
 *   STBY       = 3.3 V（板上接的，不由 MCU 驱动）
 *
 * 上电 1.5 秒后开始输出，两路同时正转，轮子会一直转。
 */
#include "board.h"
#include "bsp_motor.h"

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

    /* 恒定正转，100%：A 与 B 同时 */
    motor_set_pwm(7200, 7200);

    for (;;) {
        /* 不做任何事，保持输出。若想让它停，拔 SW1 或断电。 */
        delay_ms(1000);
    }
}

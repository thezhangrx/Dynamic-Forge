/*
 * 电机点动测试固件（不联网、不用串口）
 *
 * 烧进去后自动执行：
 *   停 1 s -> 正转慢慢加速到 DUTY_TEST -> 保持 -> 减速停 -> 停 0.6 s
 *   -> 反转慢慢加速 -> 保持 -> 减速停 -> 之后一直保持停机
 *
 * 用途：验证 STM32(TIM3/PB0/PB1) -> D153C(TB6612) -> 电机 这条链路。
 * 前提：D153C 的 STBY 接了高电平、电机电源已上，轮子架空。
 */
#include "board.h"
#include "bsp_motor.h"

#define DUTY_TEST   1000        /* 7200 满量程，1000 ≈ 14%，能明显看到轮子慢慢转 */
#define RAMP_STEP   25
#define RAMP_MS     40

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

    delay_ms(1000);                 /* 上电先静 1 秒，方便拔手 */

    /* ---- 正转：慢慢加速 ---- */
    for (int d = 0; d <= DUTY_TEST; d += RAMP_STEP) {
        motor_set_pwm((int16_t)d, (int16_t)d);
        delay_ms(RAMP_MS);
    }
    delay_ms(700);                  /* 保持速度转一会儿 */
    for (int d = DUTY_TEST; d >= 0; d -= RAMP_STEP) {
        motor_set_pwm((int16_t)d, (int16_t)d);
        delay_ms(RAMP_MS);
    }
    motor_stop();
    delay_ms(600);

    /* ---- 反转：慢慢加速 ---- */
    for (int d = 0; d <= DUTY_TEST; d += RAMP_STEP) {
        motor_set_pwm((int16_t)-d, (int16_t)-d);
        delay_ms(RAMP_MS);
    }
    delay_ms(700);
    for (int d = DUTY_TEST; d >= 0; d -= RAMP_STEP) {
        motor_set_pwm((int16_t)-d, (int16_t)-d);
        delay_ms(RAMP_MS);
    }

    /* ---- 结束：永久停机 ---- */
    motor_stop();
    for (;;) {
        motor_stop();
        delay_ms(200);
    }
}

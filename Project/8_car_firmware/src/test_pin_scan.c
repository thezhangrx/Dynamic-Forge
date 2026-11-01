/*
 * 引脚扫描固件：把 6 个信号脚当普通 GPIO，逐个拉高 5 秒
 *
 * 用途：不动一根线，用万用表在 D153C 的 Input_IO 排针上量，
 *       就能测出"STM32 的哪个脚 → 驱动板的哪个输入"。
 *
 * 顺序（每步 5 秒高，1 秒全低）：
 *   1) PB0   2) PB1   3) PB12   4) PB13   5) PB14   6) PB15
 *
 * 判定：观察到某个 D153C 输入（PWMA/PWMB/AIN1/AIN2/BIN1/BIN2/STBY）
 *       变 3.3 V，就记下它和当前那个 MCU 脚的对应关系。
 *
 * 注意：不涉及 TIM，纯 GPIO，所以和 PWM 配置无关，纯查线。
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

typedef struct {
    GPIO_TypeDef *port;
    int           pin;
    const char   *name;
} pin_t;

static const pin_t k_pins[] = {
    { GPIOB,  0, "PB0"  },
    { GPIOB,  1, "PB1"  },
    { GPIOB, 12, "PB12" },
    { GPIOB, 13, "PB13" },
    { GPIOB, 14, "PB14" },
    { GPIOB, 15, "PB15" },
};

#define NPINS ((int)(sizeof(k_pins) / sizeof(k_pins[0])))

static void all_low(void)
{
    for (int i = 0; i < NPINS; i++) gpio_set(k_pins[i].port, k_pins[i].pin, 0);
}

int main(void)
{
    board_clock_init();
    board_dwt_init();

    SysTick->LOAD = (BOARD_SYSCLK_HZ / 1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1u << 2) | (1u << 1) | (1u << 0);

    /* 6 个脚全部配成普通推挽输出 */
    RCC->APB2ENR |= (1u << 3);                 /* IOPBEN */
    for (int i = 0; i < NPINS; i++) {
        gpio_cfg(k_pins[i].port, k_pins[i].pin, GPIO_CFG_OUT_PP_50);
    }
    all_low();
    delay_ms(1500);

    for (;;) {
        for (int i = 0; i < NPINS; i++) {
            all_low();
            gpio_set(k_pins[i].port, k_pins[i].pin, 1);   /* 只拉高这一个脚 */
            delay_ms(5000);                               /* 保持 5 秒，够量 */
        }
        all_low();
        delay_ms(1000);
    }
}

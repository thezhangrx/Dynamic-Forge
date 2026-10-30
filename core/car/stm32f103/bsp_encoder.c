#include "bsp_encoder.h"

/* 编码器模式：SMCR.SMS=011（TI1 和 TI2 都计数），CCMR1 的 CC1S/CC2S = 01（映射到 TI1/TI2），
 * CCER 里 CC1P/CC2P = 0（上升沿）。定时器按 A/B 相序自动加减。
 * 位号见 RM0008 第 14 章（TIM2/TIM4 通用定时器）。 */

static uint16_t last_left;
static uint16_t last_right;

static void encoder_one(TIM_TypeDef *tim, GPIO_TypeDef *port_a, int pin_a,
                        GPIO_TypeDef *port_b, int pin_b)
{
    board_gpio_enable(port_a);
    board_gpio_enable(port_b);

    gpio_cfg(port_a, pin_a, GPIO_CFG_IN_FLOAT);
    gpio_cfg(port_b, pin_b, GPIO_CFG_IN_FLOAT);

    tim->PSC  = 0;
    tim->ARR  = 0xFFFF;
    tim->CNT  = 0;
    tim->SMCR = 0x3u;                          /* SMS = 011：编码器模式 3 */
    tim->CCMR1 = (1u << 0) | (1u << 8);        /* CC1S=01, CC2S=01 */
    tim->CCER &= ~((1u << 1) | (1u << 5));     /* 上升沿，不反相 */
    tim->CR1  |= (1u << 0);                    /* CEN */
}

void encoder_init(void)
{
    RCC->APB1ENR |= (1u << 0) | (1u << 2);     /* TIM2EN | TIM4EN */

    encoder_one(TIM2, ENC_A_PORT, 0, ENC_A_PORT, 1);   /* PA0 / PA1 */
    encoder_one(TIM4, ENC_B_PORT, 6, ENC_B_PORT, 7);   /* PB6 / PB7 */

    last_left  = (uint16_t)TIM2->CNT;
    last_right = (uint16_t)TIM4->CNT;
}

int16_t encoder_read_left_delta(void)
{
    uint16_t now = (uint16_t)TIM2->CNT;
    int16_t d = (int16_t)(now - last_left);
    last_left = now;
    return d;
}

int16_t encoder_read_right_delta(void)
{
    uint16_t now = (uint16_t)TIM4->CNT;
    int16_t d = (int16_t)(now - last_right);
    last_right = now;
    return d;
}

uint16_t encoder_raw_left(void)  { return (uint16_t)TIM2->CNT; }
uint16_t encoder_raw_right(void) { return (uint16_t)TIM4->CNT; }

void encoder_reset(void)
{
    TIM2->CNT = 0;
    TIM4->CNT = 0;
    last_left = 0;
    last_right = 0;
}

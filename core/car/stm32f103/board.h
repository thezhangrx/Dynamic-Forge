#ifndef CAR_BOARD_H
#define CAR_BOARD_H

#include <stdint.h>
#include "stm32f10x.h"

/*
 * WHEELTEC R1 / MiniBalanceV3.5 一体 C8T6 控制板
 * 时钟：HSE 8 MHz -> PLL x9 -> SYSCLK 72 MHz, AHB 72, APB1 36, APB2 72
 *
 * 引脚来源见 README.md 的引脚表（厂家源码 motor.h / encoder.c / IOI2C.h / usart3.c）。
 */

#define BOARD_HSE_HZ      8000000u
#define BOARD_SYSCLK_HZ   72000000u

/* ---------------- 电机（D153C 双路 TB6612 驱动板） ----------------
 * 引脚取自厂家 D153B/D153C 的 STM32F1 例程（USER/main.c + HAREWER/PWM/pwm.c）：
 *     PWM_Int(7199, 0) -> 10 kHz
 *     A 电机：PWM = TIM3_CH4 = PB1   方向 AIN1=PB14, AIN2=PB15
 *     B 电机：PWM = TIM3_CH3 = PB0   方向 BIN1=PB13, BIN2=PB12
 *     A 电机编码器 = TIM4 (PB6/PB7)   B 电机编码器 = TIM2 (PA0/PA1)
 *     A 电机正转：PB14=0, PB15=1      B 电机正转：PB13=1, PB12=0（两电机安装方向相反）
 * 注意：D153C 的 STBY 必须为高电平，否则驱动芯片输出全关。 */
#define MOT_A_PWM_PORT    GPIOB
#define MOT_A_PWM_PIN     1          /* TIM3_CH4 */
#define MOT_B_PWM_PORT    GPIOB
#define MOT_B_PWM_PIN     0          /* TIM3_CH3 */

/* 如果 STBY 接到了某个 GPIO（而不是直接接 3V3），在这里打开并填引脚 */
/* #define MOT_STBY_PORT   GPIOB */
/* #define MOT_STBY_PIN    5  */

#define MOT_AIN1_PORT     GPIOB
#define MOT_AIN1_PIN      14
#define MOT_AIN2_PORT     GPIOB
#define MOT_AIN2_PIN      15
#define MOT_BIN1_PORT     GPIOB
#define MOT_BIN1_PIN      13
#define MOT_BIN2_PORT     GPIOB
#define MOT_BIN2_PIN      12

/* ---------------- 编码器 ---------------- */
#define ENC_A_PORT        GPIOA      /* TIM2: PA0(CH1) PA1(CH2) */
#define ENC_B_PORT        GPIOB      /* TIM4: PB6(CH1) PB7(CH2) */

/* ---------------- MPU6050（软件 I2C） ---------------- */
#define MPU_SCL_PORT      GPIOB
#define MPU_SCL_PIN       8
#define MPU_SDA_PORT      GPIOB
#define MPU_SDA_PIN       9

/* ---------------- 串口 ---------------- */
#define UART_HOST         USART1     /* PA9 TX / PA10 RX，接上位机 */
#define UART_DBG          USART3     /* PB10 TX / PB11 RX */

/* GPIO 4 bit 配置值（CRL/CRH 里的 CNF[1:0] MODE[1:0]） */
#define GPIO_CFG_OUT_PP_2   0x2u
#define GPIO_CFG_OUT_PP_50  0x3u
#define GPIO_CFG_OUT_OD_50  0x7u
#define GPIO_CFG_AF_PP_50   0xBu
#define GPIO_CFG_IN_FLOAT   0x4u
#define GPIO_CFG_IN_PU      0x8u
#define GPIO_CFG_IN_PD      0x4u   /* 配合 ODR=0 才是下拉，用 gpio_pull 助手 */

/* 配置单个引脚（pin 0..15） */
static inline void gpio_cfg(GPIO_TypeDef *port, int pin, uint32_t cfg4)
{
    volatile uint32_t *cr = (pin < 8) ? &port->CRL : &port->CRH;
    uint32_t pos = (uint32_t)(pin & 7) * 4u;
    *cr = (*cr & ~(0xFu << pos)) | (cfg4 << pos);
}

static inline void gpio_pull_up(GPIO_TypeDef *port, int pin)
{
    gpio_cfg(port, pin, GPIO_CFG_IN_PU);
    port->BSRR = (1u << pin);          /* ODR=1 -> 上拉 */
}

static inline void gpio_pull_down(GPIO_TypeDef *port, int pin)
{
    gpio_cfg(port, pin, GPIO_CFG_IN_PU); /* CNF=10 输入；ODR=0 即下拉 */
    port->BRR = (1u << pin);
}

static inline void gpio_set(GPIO_TypeDef *port, int pin, int v)
{
    if (v) port->BSRR = (1u << pin);
    else   port->BRR  = (1u << pin);
}

/* 系统时钟：HSE 8 MHz -> 72 MHz */
void board_clock_init(void);

/* 打开 DWT 周期计数（微秒延时用） */
void board_dwt_init(void);
void board_delay_us(uint32_t us);
void board_delay_ms(uint32_t ms);

/* 打开外设时钟的快捷封装（APB2ENR 位号：IOPA=2, IOPB=3, IOPC=4, IOPD=5） */
static inline void board_gpio_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) RCC->APB2ENR |= (1u << 2);
    else if (port == GPIOB) RCC->APB2ENR |= (1u << 3);
    else if (port == GPIOC) RCC->APB2ENR |= (1u << 4);
    else if (port == GPIOD) RCC->APB2ENR |= (1u << 5);
}

#endif /* CAR_BOARD_H */

#include <stdint.h>

/* 最小启动文件（gcc 用）。如果改用 Keil，直接用厂家工程里的 startup_stm32f10x_md.s，
 * 本文件不必参与编译。 */

extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

int  main(void);
void Reset_Handler(void);

void Default_Handler(void)
{
    for (;;) { }
}

/* 内核异常 + 外设中断都用弱符号，需要时在自己的 .c 里重定义即可 */
#define WEAK_ALIAS(name) \
    void name(void) __attribute__((weak, alias("Default_Handler")))

WEAK_ALIAS(NMI_Handler);
WEAK_ALIAS(HardFault_Handler);
WEAK_ALIAS(MemManage_Handler);
WEAK_ALIAS(BusFault_Handler);
WEAK_ALIAS(UsageFault_Handler);
WEAK_ALIAS(SVC_Handler);
WEAK_ALIAS(DebugMon_Handler);
WEAK_ALIAS(PendSV_Handler);
WEAK_ALIAS(SysTick_Handler);
WEAK_ALIAS(USART1_IRQHandler);

__attribute__((section(".isr_vector"), used))
void (*const g_vectors[])(void) = {
    (void (*)(void))(&_estack),
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,
    /* 外设中断从 IRQ0 开始，这里只放用到的几个，位置必须和 IRQn 对齐 */
    Default_Handler,                    /* WWDG           0 */
    Default_Handler,                    /* PVD            1 */
    Default_Handler,                    /* TAMPER         2 */
    Default_Handler,                    /* RTC            3 */
    Default_Handler,                    /* FLASH          4 */
    Default_Handler,                    /* RCC            5 */
    Default_Handler,                    /* EXTI0          6 */
    Default_Handler,                    /* EXTI1          7 */
    Default_Handler,                    /* EXTI2          8 */
    Default_Handler,                    /* EXTI3          9 */
    Default_Handler,                    /* EXTI4         10 */
    Default_Handler,                    /* DMA1_CH1      11 */
    Default_Handler,                    /* DMA1_CH2      12 */
    Default_Handler,                    /* DMA1_CH3      13 */
    Default_Handler,                    /* DMA1_CH4      14 */
    Default_Handler,                    /* DMA1_CH5      15 */
    Default_Handler,                    /* DMA1_CH6      16 */
    Default_Handler,                    /* DMA1_CH7      17 */
    Default_Handler,                    /* ADC1_2        18 */
    Default_Handler,                    /* USB_HP_CAN1_TX 19 */
    Default_Handler,                    /* USB_LP_CAN1_RX0 20 */
    Default_Handler,                    /* CAN1_RX1      21 */
    Default_Handler,                    /* CAN1_SCE      22 */
    Default_Handler,                    /* EXTI9_5       23 */
    Default_Handler,                    /* TIM1_BRK      24 */
    Default_Handler,                    /* TIM1_UP       25 */
    Default_Handler,                    /* TIM1_TRG_COM  26 */
    Default_Handler,                    /* TIM1_CC       27 */
    Default_Handler,                    /* TIM2          28 */
    Default_Handler,                    /* TIM3          29 */
    Default_Handler,                    /* TIM4          30 */
    Default_Handler,                    /* I2C1_EV       31 */
    Default_Handler,                    /* I2C1_ER       32 */
    Default_Handler,                    /* I2C2_EV       33 */
    Default_Handler,                    /* I2C2_ER       34 */
    Default_Handler,                    /* SPI1          35 */
    Default_Handler,                    /* SPI2          36 */
    USART1_IRQHandler,                  /* USART1        37 */
    Default_Handler,                    /* USART2        38 */
    Default_Handler,                    /* USART3        39 */
    Default_Handler,                    /* EXTI15_10     40 */
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata) {
        *dst++ = *src++;
    }
    for (dst = &_sbss; dst < &_ebss; ) {
        *dst++ = 0;
    }

    main();
    for (;;) { }
}

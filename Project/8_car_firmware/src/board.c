#include "board.h"

/* 寄存器位定义直接写字面量，避免依赖 StdPeriph 的 stm32f10x_rcc.h / stm32f10x_flash.h。
 * 位号见 RM0008：RCC_CR / RCC_CFGR / FLASH_ACR。 */

#define CAR_CR_HSEON        (1u << 16)
#define CAR_CR_HSERDY       (1u << 17)
#define CAR_CR_PLLON        (1u << 24)
#define CAR_CR_PLLRDY       (1u << 25)

#define CAR_CFGR_SW_MASK        0x3u
#define CAR_CFGR_SW_PLL         0x2u
#define CAR_CFGR_SWS_MASK       0xCu
#define CAR_CFGR_SWS_PLL        0x8u
#define CAR_CFGR_HPRE_DIV1      (0x0u << 4)
#define CAR_CFGR_PPRE1_DIV2     (0x4u << 8)   /* 0b100 = /2 */
#define CAR_CFGR_PPRE2_DIV1     (0x0u << 11)
#define CAR_CFGR_PLLSRC_HSE     (1u << 16)
#define CAR_CFGR_PLLMULL9       (0x7u << 18)  /* 0b0111 = x9 */

void board_clock_init(void)
{
    /* 1. 打开 HSE 并等待稳定 */
    RCC->CR |= CAR_CR_HSEON;
    while ((RCC->CR & CAR_CR_HSERDY) == 0) { }

    /* 2. 切换前先把 Flash 延时设好：72 MHz 需要 2 个等待周期 + 预取 */
    FLASH->ACR = (1u << 4) | 0x2u;      /* PRFTBE | LATENCY=2 */

    /* 3. 预分频 + PLL 源 = HSE，倍频 x9 */
    RCC->CFGR &= ~(0xFu << 4 | 0x7u << 8 | 0x7u << 11 | 1u << 16 | 0xFu << 18 | 0x3u);
    RCC->CFGR |= CAR_CFGR_HPRE_DIV1;
    RCC->CFGR |= CAR_CFGR_PPRE1_DIV2;       /* APB1 = 36 MHz（上限 36） */
    RCC->CFGR |= CAR_CFGR_PPRE2_DIV1;       /* APB2 = 72 MHz */
    RCC->CFGR |= CAR_CFGR_PLLSRC_HSE;       /* PLL 源 = HSE 不分频 */
    RCC->CFGR |= CAR_CFGR_PLLMULL9;         /* 8 MHz x9 = 72 MHz */

    /* 4. 开 PLL 并等待锁定 */
    RCC->CR |= CAR_CR_PLLON;
    while ((RCC->CR & CAR_CR_PLLRDY) == 0) { }

    /* 5. 切 SYSCLK 到 PLL */
    RCC->CFGR = (RCC->CFGR & ~CAR_CFGR_SW_MASK) | CAR_CFGR_SW_PLL;
    while ((RCC->CFGR & CAR_CFGR_SWS_MASK) != CAR_CFGR_SWS_PLL) { }
}

/* 厂家带的 core_cm3.h 是很老的版本，没有 DWT / CoreDebug 这两个指针，
 * 直接按内核手册的地址定义（DWT @0xE0001000，CoreDebug.DEMCR @0xE000EDFC） */
typedef struct {
    volatile uint32_t CTRL;      /* 0x00 */
    volatile uint32_t CYCCNT;    /* 0x04 */
} car_dwt_t;

#define CAR_DWT          ((car_dwt_t *)0xE0001000u)
#define CAR_DEMCR        (*(volatile uint32_t *)0xE000EDFCu)
#define CAR_DEMCR_TRCENA (1u << 24)
#define CAR_DWT_CYCCNTENA (1u << 0)

void board_dwt_init(void)
{
    CAR_DEMCR |= CAR_DEMCR_TRCENA;
    CAR_DWT->CYCCNT = 0;
    CAR_DWT->CTRL |= CAR_DWT_CYCCNTENA;
}

void board_delay_us(uint32_t us)
{
    /* 72 个周期 ≈ 1 us（含循环开销，够用于 I2C 和上电等待） */
    uint32_t cycles = us * (BOARD_SYSCLK_HZ / 1000000u);
    uint32_t start  = CAR_DWT->CYCCNT;
    while ((CAR_DWT->CYCCNT - start) < cycles) { }
}

void board_delay_ms(uint32_t ms)
{
    while (ms--) {
        board_delay_us(1000);
    }
}

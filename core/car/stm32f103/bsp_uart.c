#include "bsp_uart.h"

/* USART 位定义（RM0008 第 27 章）：
 *   SR   : RXNE(5) TXE(7)
 *   CR1  : RE(2) TE(3) RXNEIE(5) UE(13)
 * 中断号：USART1 = 37, USART3 = 39
 */

#define UART_RX_BUF_SIZE 256

static volatile uint8_t  rx_buf[UART_RX_BUF_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static uint32_t brr_for(uint32_t pclk, uint32_t baud)
{
    /* BRR = DIV_Mantissa[15:4] | DIV_Fraction[3:0]，整数除法四舍五入即可 */
    return (pclk + baud / 2u) / baud;
}

void uart_host_init(uint32_t baud)
{
    RCC->APB2ENR |= (1u << 2);                    /* IOPAEN */
    gpio_cfg(GPIOA, 9,  GPIO_CFG_AF_PP_50);       /* TX */
    gpio_cfg(GPIOA, 10, GPIO_CFG_IN_FLOAT);       /* RX */

    RCC->APB2ENR |= (1u << 14);                   /* USART1EN */
    USART1->BRR = brr_for(72000000u, baud);
    USART1->CR1 = (1u << 13) | (1u << 3) | (1u << 2) | (1u << 5);

    rx_head = rx_tail = 0;
    NVIC->ISER[1] = (1u << (37 - 32));            /* USART1_IRQn = 37 */
}

void uart_dbg_init(uint32_t baud)
{
    RCC->APB2ENR |= (1u << 3);                    /* IOPBEN */
    gpio_cfg(GPIOB, 10, GPIO_CFG_AF_PP_50);       /* TX */
    gpio_cfg(GPIOB, 11, GPIO_CFG_IN_FLOAT);       /* RX */

    RCC->APB1ENR |= (1u << 18);                   /* USART3EN */
    USART3->BRR = brr_for(36000000u, baud);
    USART3->CR1 = (1u << 13) | (1u << 3) | (1u << 2);   /* 只发不收 */
}

static void putc_raw(USART_TypeDef *u, uint8_t c)
{
    while ((u->SR & (1u << 7)) == 0) { }          /* 等 TXE */
    u->DR = c;
}

void uart_host_putc(uint8_t c) { putc_raw(USART1, c); }
void uart_dbg_putc(uint8_t c)  { putc_raw(USART3, c); }

void uart_host_write(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) putc_raw(USART1, buf[i]);
}

void uart_dbg_write(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) putc_raw(USART3, buf[i]);
}

void USART1_IRQHandler(void)
{
    if (USART1->SR & (1u << 5)) {                 /* RXNE */
        uint8_t c = (uint8_t)USART1->DR;          /* 读 DR 清标志 */
        uint16_t next = (uint16_t)((rx_head + 1u) % UART_RX_BUF_SIZE);
        if (next != rx_tail) {                    /* 满了就丢新字节 */
            rx_buf[rx_head] = c;
            rx_head = next;
        }
    }
}

int uart_host_available(void)
{
    return (int)((rx_head + UART_RX_BUF_SIZE - rx_tail) % UART_RX_BUF_SIZE);
}

int uart_host_getc(void)
{
    if (rx_head == rx_tail) return -1;
    uint8_t c = rx_buf[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1u) % UART_RX_BUF_SIZE);
    return (int)c;
}

void uart_host_flush_rx(void)
{
    rx_tail = rx_head;
}

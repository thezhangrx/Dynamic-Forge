#ifndef CAR_BSP_UART_H
#define CAR_BSP_UART_H

#include <stdint.h>
#include "board.h"

/* USART1 (PA9 TX / PA10 RX) 接上位机，中断接收 + 环形缓冲
 * USART3 (PB10 TX / PB11 RX) 作调试口，暂时只发送 */

void uart_host_init(uint32_t baud);
void uart_dbg_init(uint32_t baud);

void uart_host_putc(uint8_t c);
void uart_host_write(const uint8_t *buf, uint32_t len);
void uart_dbg_putc(uint8_t c);
void uart_dbg_write(const uint8_t *buf, uint32_t len);

int  uart_host_available(void);      /* 缓冲区里有多少字节 */
int  uart_host_getc(void);           /* 取一个字节，没有返回 -1 */

void uart_host_flush_rx(void);

#endif /* CAR_BSP_UART_H */

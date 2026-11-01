/* prober_main.c —— E30 配置口探针（USART1@9600 ↔ E30，USART3@115200 → CH340 → PC）
 *
 * 用途：把模块的 TXD 通过 C8T6 的 3.3V 电平转发给 PC，判定"模块到底回不回话"。
 * 每 200 ms 发一条 "C0\r\n"，收到的每个字节都按 hex 打给 PC。
 */
#include "board.h"
#include "bsp_uart.h"
#include <string.h>

static volatile uint32_t g_ms;
void SysTick_Handler(void) { g_ms++; }
static void systick_init(void)
{
    SysTick->LOAD = (BOARD_SYSCLK_HZ / 1000u) - 1u;
    SysTick->VAL  = 0;
    SysTick->CTRL = (1u << 2) | (1u << 1) | (1u << 0);
}
static uint32_t millis(void) { return g_ms; }

static const char HEX[] = "0123456789abcdef";
static void log_hex(const uint8_t *b, int n)
{
    char out[8];
    for (int i = 0; i < n; ++i) {
        out[0] = HEX[b[i] >> 4]; out[1] = HEX[b[i] & 15]; out[2] = ' ';
        uart_dbg_write((const uint8_t *)out, 3);
    }
}

int main(void)
{
    board_clock_init();
    uart_host_init(9600);      /* E30 侧 */
    uart_dbg_init(115200);     /* CH340 侧 */
    systick_init();

    const char *banner = "P0-PROBER OK  9600->E30 115200->PC\r\n";
    uart_dbg_write((const uint8_t *)banner, (uint32_t)strlen(banner));

    uint32_t t_beat = millis(), t_tx = millis();
    uint8_t got[64];
    int n = 0;
    uint32_t n_tx = 0;

    for (;;) {
        /* 发指令 */
        if (millis() - t_tx >= 200u) {
            static const uint8_t cmd[] = "C0\r\n";
            uart_host_write(cmd, 4);
            uart_host_write(cmd, 4);        /* 6 字节总量 = C0\r\nC0\r\n 里取 6 */
            t_tx = millis(); n_tx++;
        }
        /* 收字节 */
        int c;
        while ((c = uart_host_getc()) >= 0) {
            if (n < (int)sizeof(got)) got[n++] = (uint8_t)c;
        }
        /* 每秒汇报一次 */
        if (millis() - t_beat >= 1000u) {
            char hdr[64];
            int m = 0;
            hdr[m++] = 'T'; hdr[m++] = 'X'; hdr[m++] = ' ';
            hdr[m++] = (char)('0' + (n_tx / 100) % 10);
            hdr[m++] = (char)('0' + (n_tx / 10) % 10);
            hdr[m++] = (char)('0' + n_tx % 10);
            hdr[m++] = ' '; hdr[m++] = 'R'; hdr[m++] = 'X'; hdr[m++] = ' ';
            hdr[m++] = (char)('0' + (n / 10) % 10);
            hdr[m++] = (char)('0' + n % 10);
            hdr[m++] = ':'; hdr[m++] = ' ';
            uart_dbg_write((const uint8_t *)hdr, (uint32_t)m);
            log_hex(got, n);
            uart_dbg_write((const uint8_t *)"\r\n", 2);
            n = 0; n_tx = 0; t_beat = millis();
        }
    }
}

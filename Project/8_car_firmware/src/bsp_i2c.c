#include "bsp_i2c.h"

/* 软件 I²C：PB8=SCL, PB9=SDA，开漏 + 外部上拉。
 * 半周期约 2 us（≈250 kHz），MPU6050 支持 400 kHz，留了余量。 */

#define I2C_DELAY()  board_delay_us(2)

static inline void scl(int v) { gpio_set(MPU_SCL_PORT, MPU_SCL_PIN, v); }
static inline void sda(int v) { gpio_set(MPU_SDA_PORT, MPU_SDA_PIN, v); }
static inline int  sda_read(void)
{
    return (MPU_SDA_PORT->IDR & (1u << MPU_SDA_PIN)) ? 1 : 0;
}

void i2c_init(void)
{
    board_gpio_enable(MPU_SCL_PORT);
    board_gpio_enable(MPU_SDA_PORT);

    gpio_cfg(MPU_SCL_PORT, MPU_SCL_PIN, GPIO_CFG_OUT_OD_50);
    gpio_cfg(MPU_SDA_PORT, MPU_SDA_PIN, GPIO_CFG_OUT_OD_50);

    scl(1);
    sda(1);
    I2C_DELAY();
}

void i2c_start(void)
{
    sda(1);
    scl(1);
    I2C_DELAY();
    sda(0);                 /* SCL 高时 SDA 下降 = START */
    I2C_DELAY();
    scl(0);
}

void i2c_stop(void)
{
    sda(0);
    scl(1);
    I2C_DELAY();
    sda(1);                 /* SCL 高时 SDA 上升 = STOP */
    I2C_DELAY();
}

int i2c_write_byte(uint8_t byte)
{
    for (int i = 0; i < 8; i++) {
        sda((byte & 0x80u) ? 1 : 0);
        byte <<= 1;
        I2C_DELAY();
        scl(1);
        I2C_DELAY();
        scl(0);
    }

    sda(1);                 /* 释放 SDA 让从机应答 */
    I2C_DELAY();
    scl(1);
    I2C_DELAY();
    int nack = sda_read();  /* 0 = ACK */
    scl(0);
    I2C_DELAY();
    return nack;
}

uint8_t i2c_read_byte(int ack)
{
    uint8_t byte = 0;

    sda(1);                 /* 释放 SDA，交给从机驱动 */
    for (int i = 0; i < 8; i++) {
        I2C_DELAY();
        scl(1);
        I2C_DELAY();
        byte = (uint8_t)((byte << 1) | (sda_read() ? 1 : 0));
        scl(0);
    }

    /* 第 9 个时钟：主机应答 */
    sda(ack ? 0 : 1);
    I2C_DELAY();
    scl(1);
    I2C_DELAY();
    scl(0);
    sda(1);

    return byte;
}

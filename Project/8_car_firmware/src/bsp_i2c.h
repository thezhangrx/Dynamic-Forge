#ifndef CAR_BSP_I2C_H
#define CAR_BSP_I2C_H

#include <stdint.h>
#include "board.h"

/* PB8=SCL, PB9=SDA 的软件 I²C（与厂家 IOI2C 一致）。开漏输出，靠外部上拉。 */
void i2c_init(void);

void i2c_start(void);
void i2c_stop(void);

/* 返回 0 表示收到 ACK，1 表示 NACK */
int  i2c_write_byte(uint8_t byte);

/* ack=1：读完发 ACK（还要继续读）；ack=0：发 NACK（最后一个字节） */
uint8_t i2c_read_byte(int ack);

#endif /* CAR_BSP_I2C_H */

#ifndef CAR_BSP_ENCODER_H
#define CAR_BSP_ENCODER_H

#include <stdint.h>
#include "board.h"

/* 左轮 TIM2（PA0/PA1），右轮 TIM4（PB6/PB7），都是硬件正交解码。
 * 计数器 ARR=65535，读增量时用 int16_t 差值，天然处理 16 位回绕。 */
void encoder_init(void);

/* 自上次调用以来的计数增量（正负代表方向） */
int16_t encoder_read_left_delta(void);
int16_t encoder_read_right_delta(void);

/* 直接读 16 位计数值（调试用） */
uint16_t encoder_raw_left(void);
uint16_t encoder_raw_right(void);

void encoder_reset(void);

#endif /* CAR_BSP_ENCODER_H */

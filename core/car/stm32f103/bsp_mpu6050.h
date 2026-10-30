#ifndef CAR_BSP_MPU6050_H
#define CAR_BSP_MPU6050_H

#include <stdint.h>

#define MPU6050_ADDR        0x68

/* ±4 g  -> 8192 LSB/g ；±2000 dps -> 16.4 LSB/(°/s) */
#define MPU6050_LSB_PER_G     8192.0f
#define MPU6050_LSB_PER_DPS   16.4f

typedef struct {
    int16_t ax, ay, az;      /* 原始值 */
    int16_t temp;
    int16_t gx, gy, gz;
} mpu_raw_t;

/* 返回 0 成功；-1 = WHO_AM_I 不对，-2 = 写寄存器无应答 */
int  mpu6050_init(void);

/* 读一次 14 字节原始数据；返回 0 成功，-1 失败 */
int  mpu6050_read(mpu_raw_t *out);

/* 静止标定陀螺零偏：连续采 n 次求平均 */
void mpu6050_calib_gyro(float bias_dps[3], uint16_t n);

/* 换算成物理量 */
static inline float mpu_acc_g(int16_t raw)        { return raw / MPU6050_LSB_PER_G; }
static inline float mpu_gyro_dps(int16_t raw)     { return raw / MPU6050_LSB_PER_DPS; }

#endif /* CAR_BSP_MPU6050_H */

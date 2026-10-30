#include "bsp_mpu6050.h"
#include "bsp_i2c.h"

/* MPU6050 寄存器地址（见厂家附带的 MPU-60X0 寄存器手册） */
#define REG_SMPLRT_DIV     0x19
#define REG_CONFIG         0x1A
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define REG_PWR_MGMT_1     0x6B
#define REG_WHO_AM_I       0x75

static int write_reg(uint8_t reg, uint8_t val)
{
    i2c_start();
    int err = i2c_write_byte((uint8_t)(MPU6050_ADDR << 1));
    if (err) { i2c_stop(); return -1; }
    i2c_write_byte(reg);
    i2c_write_byte(val);
    i2c_stop();
    return 0;
}

static int read_regs(uint8_t reg, uint8_t *buf, int len)
{
    i2c_start();
    if (i2c_write_byte((uint8_t)(MPU6050_ADDR << 1))) { i2c_stop(); return -1; }
    if (i2c_write_byte(reg))                          { i2c_stop(); return -1; }

    i2c_start();                                        /* 重复起始 */
    if (i2c_write_byte((uint8_t)((MPU6050_ADDR << 1) | 1u))) { i2c_stop(); return -1; }

    for (int i = 0; i < len; i++) {
        buf[i] = i2c_read_byte(i + 1 < len);            /* 最后一字节回 NACK */
    }
    i2c_stop();
    return 0;
}

int mpu6050_init(void)
{
    uint8_t who = 0;

    if (read_regs(REG_WHO_AM_I, &who, 1) != 0) return -2;
    if (who != 0x68) return -1;

    write_reg(REG_PWR_MGMT_1, 0x80);     /* 复位 */
    board_delay_ms(100);
    write_reg(REG_PWR_MGMT_1, 0x01);     /* 退出睡眠，时钟源 = PLL X 轴陀螺 */
    board_delay_ms(10);

    /* 采样率 1 kHz / (1 + SMPLRT_DIV) */
    write_reg(REG_SMPLRT_DIV, 0x04);     /* 200 Hz */
    write_reg(REG_CONFIG, 0x03);         /* DLPF 44 Hz */
    write_reg(REG_GYRO_CONFIG, 0x18);    /* FS_SEL = 3：±2000 °/s */
    write_reg(REG_ACCEL_CONFIG, 0x08);   /* AFS_SEL = 1：±4 g */

    return 0;
}

int mpu6050_read(mpu_raw_t *out)
{
    uint8_t b[14];

    if (read_regs(REG_ACCEL_XOUT_H, b, 14) != 0) return -1;

    out->ax   = (int16_t)((b[0]  << 8) | b[1]);
    out->ay   = (int16_t)((b[2]  << 8) | b[3]);
    out->az   = (int16_t)((b[4]  << 8) | b[5]);
    out->temp = (int16_t)((b[6]  << 8) | b[7]);
    out->gx   = (int16_t)((b[8]  << 8) | b[9]);
    out->gy   = (int16_t)((b[10] << 8) | b[11]);
    out->gz   = (int16_t)((b[12] << 8) | b[13]);
    return 0;
}

void mpu6050_calib_gyro(float bias_dps[3], uint16_t n)
{
    mpu_raw_t r;
    int32_t sx = 0, sy = 0, sz = 0;
    uint16_t got = 0;

    for (uint16_t i = 0; i < n; i++) {
        if (mpu6050_read(&r) == 0) {
            sx += r.gx; sy += r.gy; sz += r.gz;
            got++;
        }
        board_delay_ms(5);
    }

    if (got == 0) got = 1;
    bias_dps[0] = (float)sx / got / MPU6050_LSB_PER_DPS;
    bias_dps[1] = (float)sy / got / MPU6050_LSB_PER_DPS;
    bias_dps[2] = (float)sz / got / MPU6050_LSB_PER_DPS;
}

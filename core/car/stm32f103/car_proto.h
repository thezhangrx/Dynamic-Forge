#ifndef CAR_PROTO_H
#define CAR_PROTO_H

#include <stdint.h>
#include "car_kin.h"

/*
 * 上位机（FPGA 板）与 STM32 之间的行文本协议，一行一条，'\n' 结尾。
 *
 * 下行（上位机 -> STM32）
 *   @MOVE <act>              仿真那套动作编号 0..8
 *   @VEL <v_mm_s> <w_mrad_s> 直接给差速指令：线速度 mm/s、角速度 mrad/s
 *   @STOP                    立即停（保持停车状态直到下一条 MOVE/VEL）
 *   @CAL                     重新标定陀螺零偏（车必须静止）
 *
 * 上行（STM32 -> 上位机）
 *   @ODOM <x_mm> <y_mm> <theta_mrad>              里程计位姿（页面坐标）
 *   @STATE <angle_cdeg> <v_mm_s> <w_mrad_s> <act>  姿态/速度/当前动作回显
 *   @ACK <cmd>                                    指令已接受
 *
 * 用文本而不是二进制，是因为仿真链路本来就是文本（@MOVE/@POSE），
 * 方便直接用串口助手对着调。
 */

typedef enum {
    CAR_CMD_NONE = 0,
    CAR_CMD_MOVE,
    CAR_CMD_VEL,
    CAR_CMD_STOP,
    CAR_CMD_CAL,
} car_cmd_kind_t;

typedef struct {
    car_cmd_kind_t kind;
    uint8_t act;        /* CMD_MOVE 时有效 */
    float   v;          /* CMD_VEL：mm/s */
    float   w;          /* CMD_VEL：mrad/s */
} car_cmd_t;

/* 无线链路（E30-433T20D 透明传输）用：行尾可带 "#<seq>"，回 "@ACK <seq>"。
 * 半双工 + 会丢包，所以必须有 seq/ACK + 车载看门狗；不带 seq 时行为与原来完全一致。 */
int  car_proto_last_seq(void);              /* 刚解析成功那条的 seq；没有 = -1 */
int  car_proto_fmt_ackseq(char *buf, int size, int seq);
void car_proto_reset(void);

/* 每收到一个字节喂进来；解析到完整指令返回 1 并填 out */
int car_proto_feed(uint8_t byte, car_cmd_t *out);

/* 输出格式化：返回写入的长度（不含结尾 0） */
int car_proto_fmt_odom(char *buf, int size, car_pose_t p);
int car_proto_fmt_state(char *buf, int size, float angle_cdeg,
                        float v_mm_s, float w_mrad_s, uint8_t act);
int car_proto_fmt_ack(char *buf, int size, const char *name, float v, float w);

#endif /* CAR_PROTO_H */

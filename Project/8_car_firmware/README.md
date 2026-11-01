# 模块 8：小车固件

## 职责

STM32F103 底盘控制器固件：陀螺仪姿态解算、串级 PID（直立/速度/转向）、
差速模式、无线串口协议、编码器里程计。

## 目录结构

| 目录 | 内容 |
|---|---|
| `src/` | `main.c`（主循环与协议分发）、`board.c`（时钟/GPIO/中断）、`bsp_motor.c`（TB6612 PWM 与方向）、`bsp_encoder.c`（TIM2/TIM4 编码器）、`bsp_i2c.c`、`bsp_mpu6050.c`、`bsp_uart.c`、`car_control.c`（级联 PID 与差速）、`car_proto.c`（`@VEL`/`@MOVE`/`@STOP`/`@MODE`/`@ODOM`/`@STATE` 解析与格式化）、`car_kin.c`（运动学与里程计）、`*.h`、链接脚本 `*.ld`、`Makefile` |
| `cmsis/` | 厂商 CMSIS 设备头文件（`stm32f10x.h`、`core_cm3.h`、`system_stm32f10x.*`） |
| `tools/` | `flash.sh`（st-flash 烧写）、`check_alive.sh`、`check_hw.sh`、`isp_flash.py` |
| `tests/` | 主机可编译的 C 单元测试（运动学、里程计、协议、控制器） |

## 接口

- 串口（115200 8N1，经无线模块透传）：
  - 下行：`@VEL <v_mm_s> <w_mrad_s>`、`@MOVE <act>`、`@STOP`、`@MODE <0|1>`、`@CAL`
  - 上行：`@READY`、`@ODOM <x_mm> <y_mm> <theta_mrad>`、`@STATE <angle_cdeg> <v> <w> <act>`、`@ACK <seq>`
- 控制周期 10 ms；通信看门狗 1 s。

## 构建与烧写

```bash
cd 8_car_firmware/src
make                      # 产出 build/car.elf、build/car.bin
st-flash write build/car.bin 0x8000000 && st-flash reset
```

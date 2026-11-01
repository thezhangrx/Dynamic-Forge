# STM32F103C8T6 驱动层 + 平衡控制（WHEELTEC R1 底盘）

面向 `docs/WHEELTEC  R1平衡车资料` 里那套 **MiniBalanceV3.5 一体 C8T6 控制板 + R1 底盘**。
全部直接写寄存器（CMSIS `stm32f10x.h`），**不依赖厂商 StdPeriph 库**，方便和 FPGA 上位机对接。

## 一、引脚表

来源：厂家源码 `MOTOR/motor.h`、`ENCODER/encoder.c`、`IIC/IOI2C.h`、`USART3/usart3.c`、
`USART/usart.c`；Mini 版与轻装版 V5.0 **完全一致**。

| 功能 | 引脚 | 备注 |
|---|---|---|
| 电机 A PWM | **PB1** | TIM3_CH4，10 kHz（ARR=7199, PSC=0, 72 MHz） |
| 电机 A 方向 AIN1 / AIN2 | **PB14 / PB15** | TB6612，正转 PB14=0 / PB15=1 |
| 电机 B PWM | **PB0** | TIM3_CH3 |
| 电机 B 方向 BIN1 / BIN2 | **PB13 / PB12** | 正转 PB13=1 / PB12=0（两电机反向安装） |
| 编码器 A（电机 A） | **PB6 / PB7** | TIM4 正交解码 |
| 编码器 B（电机 B） | **PA0 / PA1** | TIM2 正交解码 |
| MPU6050（若接） | 待定 | 例程未用；软件 I²C 可配任意两脚 |
| 调试/上位机串口 USART1 | **PA9 / PA10** | 115200；**同时是 BOOT0=1 时的串口 ISP 口** |
| 无线模块串口 USART3 | **PB10 / PB11** | 预留给透传模块 |
| 电池电压 ADC | **PA6** | D153C 稳压版才有 |

> **依据**：厂家 `D153B_霍尔编码器标准库例程` 的 `USER/main.c` + `HAREWER/PWM/pwm.c` + `HAREWER/ENCODER/encoder.c`。
> **STBY 必须为高电平**（接 3V3），否则 TB6612 输出全关。

PB12–PB15 不是 JTAG 脚（JTAG 在 PA13/PA14/PA15/PB3/PB4），不用关 JTAG。

## 二、文件

| 文件 | 内容 |
|---|---|
| `board.h/.c` | 时钟 HSE 8 M→72 M、GPIO 配置助手、DWT 微秒延时 |
| `bsp_motor.h/.c` | TIM1 双路 PWM + TB6612 方向；`motor_set_pwm()` 直接吃 PID 输出（±7200 计数） |
| `bsp_encoder.h/.c` | TIM2/TIM4 正交解码，`int16_t` 差值自动处理 16 位回绕 |
| `bsp_i2c.h/.c` | PB8/PB9 软件 I²C |
| `bsp_mpu6050.h/.c` | MPU6050 初始化（±4 g / ±2000 dps / 200 Hz）、14 字节连读、零偏标定 |
| `bsp_uart.h/.c` | USART1 中断接收 + 256 B 环形缓冲；USART3 调试口 |
| **`car_kin.h/.c`** | **`@MOVE <act>` ↔ (v, ω) 互转**、轮速换算、公式 13.2 里程计 |
| **`car_control.h/.c`** | **厂家串级 PID**（直立 PD + 速度 PI + 转向 PD）原样移植 |
| **`car_proto.h/.c`** | 行文本协议解析与格式化 |
| `main.c` | 5 ms 姿态 / 10 ms 控制 / 5 Hz 上报 |
| `tests/test_car.c` | 主机单元测试（不碰硬件寄存器） |
| `startup_stm32f103.c`、`stm32f103c8.ld`、`Makefile` | arm-none-eabi-gcc 构建 |

## 三、指令与 (v, ω) 的互转

依据 `docs/reference/公式.md` **公式 13.2（差速驱动机器人模型）**：

```
ẋ = r/2 (u_l + u_r) cosθ
ẏ = r/2 (u_l + u_r) sinθ
θ̇ = r/L (u_r − u_l)
```

换成轮子线速度 `v = (v_l + v_r)/2`、`ω = (v_r − v_l)/L`。

**动作编号沿用仿真**（`core/board/tools/virt_core.py` 的 DIRS，页面坐标 x 右 y 下）：

| act | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
|---|---|---|---|---|---|---|---|---|---|
| 方向 | 停 | −x | +x | +y | −y | −x+y | +x+y | −x−y | +x−y |

**正变换**（`car_act_to_tw`）：目标方向 `ψ`；误差 `ε = wrap(ψ − θ)`；
`|ε| > 60°` 就原地转（v=0），否则 `v = v_nom·cos ε`；`ω = k_head·ε`。

**反变换**（`car_tw_to_act`）：`ε = ω/k_head`，`ψ = θ + ε`（v<0 时再加 π），
量化到最近的 8 个方向；`|v|` 和 `|ω|` 都小于阈值时给 0。

单元测试里 **56 个 (θ, act) 组合往返全部一致**。

## 四、串口协议（行文本，`\n` 结尾）

下行（上位机 → STM32）：

```
@MOVE <act>               动作 0..8（仿真语义）
@VEL <v_mm_s> <w_mrad_s>  直接给差速指令（mm/s、mrad/s）
@STOP                     立即停并保持停车
@CAL                      重新标定陀螺零偏（车要静止）
```

上行（STM32 → 上位机）：

```
@READY                                          上电就绪
@STATE <角度0.1°> <v_mm_s> <w_mrad_s> <act>     对齐目标
@ODOM  <x_mm> <y_mm> <theta_mrad>               编码器里程计（页面坐标）
@ACK   <名字> <v> <w>                           指令回执
```

## 五、控制律（厂家原样）

```
Balance  = kp_b·(Angle − 中值) + kd_b·Gyro_balance      kp_b=100, kd_b=0.40, 中值 3°
Velocity = lpf(enc_l+enc_r)·kp_v + ∫·ki_v               kp_v=50, ki_v=0.25, 积分限幅 ±15000
Turn     = Turn_Target·kp_t + Gyro_turn·kd_t            kp_t=42, kd_t=0.6(直行)/0(原地)
Moto_L   = Balance + Velocity − Turn
Moto_R   = Balance + Velocity + Turn        限幅 ±6900，占空比再加 400 死区
```

保护：`|Angle| > 40°` 或 `@STOP` → 立即停机、清速度环积分。

## 六、上车前必须标定的 5 个量

| 量 | 在哪 | 怎么标 |
|---|---|---|
| 机械平衡点 `angle_mid_deg` | `car_ctrl_defaults()` | 车能站住时的实际倾角（厂家给 3°） |
| 编码器方向 | `bsp_encoder.c` 符号 | 手往前推，两个 delta 应同号；不对就取反一个 |
| `ENC_PER_MPS` | `main.c` | 手推车 1 m/s，看 10 ms 编码器计数 |
| `WHEEL_BASE_M` | `main.c` | 卷尺量两轮中心距（厂家底盘约 0.15 m） |
| 陀螺零偏 | 上电自动 `mpu6050_calib_gyro()` | 放平静止 1 s；漂移大就重跑 `@CAL` |

## 七、构建与测试

```bash
# 单元测试（主机，不需要工具链）
cd core/car/stm32f103/tests
gcc -std=c11 -Wall -Wextra -I.. -o /tmp/test_car test_car.c \
    ../car_kin.c ../car_control.c ../car_proto.c -lm && /tmp/test_car

# 固件
cd core/car/stm32f103
make PREFIX=<toolchain>/bin/arm-none-eabi-    # 产出 build/car.elf / car.bin
```

`cmsis/` 里已放好厂家那套 `stm32f10x.h`、`core_cm3.h`、`system_stm32f10x.*`。

### 当前构建状态（已实测）

WSL 没网也没 sudo，工具链是直接下载解压到 `Env/toolchain/`（`Env/` 在 .gitignore 里）的：

```bash
cd core/car/stm32f103
make PREFIX=/home/zhang/FPGA/Env/toolchain/arm-gnu-toolchain-13.2.Rel1-x86_64-arm-none-eabi/bin/arm-none-eabi-
```

结果：**0 error、0 warning**，`text 13996 / data 0 / bss 476` 字节（64 KB Flash、20 KB RAM 够用）。
向量表首字 `0x20005000`（栈顶 = RAM 末端），`SysTick_Handler` 挂在 0x080014bd，`USART1_IRQHandler` 在 IRQ37 位置。

单元测试：`tests/` 下 **24 项全过**（act↔(v,ω) 往返 56 组、公式 13.2 里程计、协议解析/格式化、串级 PID 保护与积分限幅）。

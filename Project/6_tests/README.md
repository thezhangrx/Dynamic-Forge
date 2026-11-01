# 模块 6：单元测试

## 职责

在不依赖硬件的前提下验证各层关键逻辑：板端协议与视觉后处理、CPU 侧规划与势场、
小车固件中的运动学与控制器。

## 目录结构

| 目录 | 内容 |
|---|---|
| `board/` | 板端 Python 测试（寄存器映射、键盘驱动解析与去抖、协议等） |
| `cpu/` | 规划与势场的 Python 测试 |
| `car/` | 小车固件的 C 单元测试（运动学、里程计、协议、控制器），可在主机编译运行 |

## 运行

```bash
# 板端 / CPU
/tmp/vpy/bin/python -m pytest 6_tests/board -q
/tmp/vpy/bin/python -m pytest 6_tests/cpu -q

# 小车固件（主机编译运行）
cd 8_car_firmware/tests
gcc -std=c11 -Wall -Wextra -I../src -o /tmp/test_car test_car.c \
    ../src/car_kin.c ../src/car_control.c ../src/car_proto.c -lm && /tmp/test_car
```

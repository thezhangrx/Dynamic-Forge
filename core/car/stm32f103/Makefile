# STM32F103C8T6 驱动层构建（arm-none-eabi-gcc）
#
# 依赖 CMSIS 设备头（stm32f10x.h + core_cm3.h）。它们来自厂家资料里的
#   Libraries.zip / FWLIB.zip -> Libraries/CMSIS/CM3/{CoreSupport,DeviceSupport/ST/STM32F10x}
# 把这两个目录拷到 cmsis/ 下即可：
#   cmsis/core/core_cm3.h
#   cmsis/device/stm32f10x.h  system_stm32f10x.h
#
#   make          生成 build/car.elf build/car.bin
#   make flash    用 st-flash 烧录（需要 stlink 工具）
#   make clean

PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

CMSIS    ?= cmsis
LDSCRIPT ?= stm32f103c8.ld

CPU     := -mcpu=cortex-m3 -mthumb
CFLAGS  := $(CPU) -O2 -g -std=c11 -Wall -Wextra -DSTM32F10X_MD \
           -ffunction-sections -fdata-sections \
           -I. -I$(CMSIS)/core -I$(CMSIS)/device
LDFLAGS := $(CPU) -T $(LDSCRIPT) -nostartfiles -Wl,--gc-sections -Wl,-Map=build/car.map

SRCS := board.c bsp_motor.c bsp_encoder.c bsp_i2c.c bsp_mpu6050.c bsp_uart.c \
        car_kin.c car_control.c car_proto.c \
        main.c startup_stm32f103.c
OBJS := $(SRCS:%.c=build/%.o)

all: build/car.elf build/car.bin

build:
	@mkdir -p build

build/%.o: %.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/car.elf: $(OBJS) $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) $(OBJS) -lm -o $@
	$(SIZE) $@

build/car.bin: build/car.elf
	$(OBJCOPY) -O binary $< $@

# ---- 电机点动测试固件（不含主程序） ----
TEST_OBJS := build/board.o build/bsp_motor.o build/test_motor.o build/startup_stm32f103.o

build/test_motor.o: test_motor.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/motor_test.elf: $(TEST_OBJS) $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) $(TEST_OBJS) -lm -o $@
	$(SIZE) $@

build/motor_test.bin: build/motor_test.elf
	$(OBJCOPY) -O binary $< $@

build/test_motor_hold.o: test_motor_hold.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/motor_hold.elf: build/board.o build/bsp_motor.o build/test_motor_hold.o build/startup_stm32f103.o $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) build/board.o build/bsp_motor.o build/test_motor_hold.o build/startup_stm32f103.o -lm -o $@
	$(SIZE) $@

build/motor_hold.bin: build/motor_hold.elf
	$(OBJCOPY) -O binary $< $@

build/test_motor_phased.o: test_motor_phased.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/motor_phased.elf: build/board.o build/bsp_motor.o build/test_motor_phased.o build/startup_stm32f103.o $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) build/board.o build/bsp_motor.o build/test_motor_phased.o build/startup_stm32f103.o -lm -o $@
	$(SIZE) $@

build/motor_phased.bin: build/motor_phased.elf
	$(OBJCOPY) -O binary $< $@

build/test_motor_full.o: test_motor_full.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/motor_full.elf: build/board.o build/bsp_motor.o build/test_motor_full.o build/startup_stm32f103.o $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) build/board.o build/bsp_motor.o build/test_motor_full.o build/startup_stm32f103.o -lm -o $@
	$(SIZE) $@

build/motor_full.bin: build/motor_full.elf
	$(OBJCOPY) -O binary $< $@

build/test_pin_scan.o: test_pin_scan.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/pin_scan.elf: build/board.o build/bsp_motor.o build/test_pin_scan.o build/startup_stm32f103.o $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) build/board.o build/bsp_motor.o build/test_pin_scan.o build/startup_stm32f103.o -lm -o $@
	$(SIZE) $@

build/pin_scan.bin: build/pin_scan.elf
	$(OBJCOPY) -O binary $< $@

build/test_motor_const.o: test_motor_const.c | build
	$(CC) $(CFLAGS) -c $< -o $@

build/motor_const.elf: build/board.o build/bsp_motor.o build/test_motor_const.o build/startup_stm32f103.o $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) build/board.o build/bsp_motor.o build/test_motor_const.o build/startup_stm32f103.o -lm -o $@
	$(SIZE) $@

build/motor_const.bin: build/motor_const.elf
	$(OBJCOPY) -O binary $< $@

motor-const: build/motor_const.bin

pin-scan: build/pin_scan.bin

motor-full: build/motor_full.bin

motor-phased: build/motor_phased.bin

motor-hold: build/motor_hold.bin

motor-test: build/motor_test.bin

flash: build/car.bin
	st-flash write build/car.bin 0x8000000

clean:
	rm -rf build

.PHONY: all flash clean motor-test motor-hold motor-phased motor-full pin-scan motor-const

# ---- E30 探针固件（USART1↔E30, USART3→CH340）----
PROBER_OBJS := build/board.o build/bsp_uart.o build/prober_main.o build/startup_stm32f103.o
build/prober_main.o: prober_main.c | build
	$(CC) $(CFLAGS) -c $< -o $@
build/prober.elf: $(PROBER_OBJS) $(LDSCRIPT) | build
	$(CC) $(LDFLAGS) $(PROBER_OBJS) -lm -o $@
	$(SIZE) $@
build/prober.bin: build/prober.elf
	$(OBJCOPY) -O binary $< $@
prober: build/prober.bin

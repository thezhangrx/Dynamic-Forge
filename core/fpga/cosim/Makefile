# ===========================================================================
# core/fpga/cosim/Makefile —— Verilator 协同仿真（RTL ↔ C 参考模型）
#
#   make           构建 obj_dir/Vvision_top
#   make run       跑一小批（默认 2 帧，先打通）
#   make clean
#
# Verilator 免 root 装在 $HOME/.local/verilator-5.020（见 ref/README.md），
# 所以这里显式带上 VERILATOR_ROOT / PATH。
# ===========================================================================

VERILATOR_ROOT ?= $(HOME)/.local/verilator-5.020
VERILATOR      ?= $(VERILATOR_ROOT)/bin/verilator
# Verilator 靠 VERILATOR_ROOT 找 verilated_std.sv / include/；从解包安装里跑
# 时必须显式导出，否则会去 /usr/share/verilator 找。
export VERILATOR_ROOT
export PATH := $(VERILATOR_ROOT)/bin:$(PATH)

RTL   = ../rtl
REF   = ../ref
TOP   = vision_top
Mdir  = obj_dir

REF_OBJ = $(abspath $(REF))/vision_ref.o
RTL_SRC = $(RTL)/vision_top.v $(RTL)/vision_ccl.v
AXI_SRC = $(RTL)/vision_axi.v $(RTL)/vision_top.v $(RTL)/vision_ccl.v
ST_SRC  = $(RTL)/vision_selftest.v $(RTL)/vision_tpg.v $(RTL)/vision_top.v $(RTL)/vision_ccl.v
HDR_SRC = $(RTL)/vision_pkg.vh $(RTL)/vision_ops.vh
REF_SRC = $(REF)/vision_ref.c $(REF)/ref_api.h

STIM  ?= /tmp/stim_color
FRAMES?= 2

all: $(Mdir)/V$(TOP) $(Mdir)/Vvision_axi $(Mdir)/Vvision_selftest

$(REF_OBJ): $(REF_SRC)
	gcc -O2 -Wall -Wextra -std=c11 -I$(REF) -DVR_NO_MAIN -c $(REF)/vision_ref.c -o $@

$(Mdir)/V$(TOP): $(RTL_SRC) $(HDR_SRC) tb_vision.cpp $(REF_OBJ)
	$(VERILATOR) --cc --exe --build -j 4 -O2 \
		--top-module $(TOP) -I$(RTL) \
		--Mdir $(Mdir) \
		-CFLAGS "-I$(abspath $(REF))" \
		$(RTL_SRC) tb_vision.cpp $(REF_OBJ)

# AXI 路径（PS 面向接口）：走配置寄存器 → 启动 → 读窗口
$(Mdir)/Vvision_axi: $(AXI_SRC) $(HDR_SRC) tb_axi.cpp $(REF_OBJ)
	$(VERILATOR) --cc --exe --build -j 4 -O2 \
		--top-module vision_axi -I$(RTL) \
		--Mdir $(Mdir) \
		-CFLAGS "-I$(abspath $(REF))" \
		$(AXI_SRC) tb_axi.cpp $(REF_OBJ)

# 自检图案（不用相机、不用 C 模型：和手算的几何比）
$(Mdir)/Vvision_selftest: $(ST_SRC) $(HDR_SRC) tb_tpg.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 \
		--top-module vision_selftest -I$(RTL) \
		--Mdir $(Mdir) \
		$(ST_SRC) tb_tpg.cpp

run: $(Mdir)/V$(TOP)
	./$(Mdir)/V$(TOP) $(STIM) $(FRAMES)

run-axi: $(Mdir)/Vvision_axi
	./$(Mdir)/Vvision_axi $(STIM) $(FRAMES)

run-selftest: $(Mdir)/Vvision_selftest
	./$(Mdir)/Vvision_selftest 8

clean:
	rm -rf $(Mdir) $(REF_OBJ)

.PHONY: all run run-axi run-selftest clean

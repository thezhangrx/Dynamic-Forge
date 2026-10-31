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
RTL_SRC = $(RTL)/vision_top.v $(RTL)/vision_ccl.v $(RTL)/vision_div.v
#: P0 起 vision_axi 内部例化了正射子系统，所以整个正射树都要进这个模型
AXI_SRC = $(RTL)/vision_axi.v $(RTL)/vision_pixfeed.v $(RTL)/vision_top.v $(RTL)/vision_ccl.v \
          $(RTL)/vision_div.v $(RTL)/vision_ortho.v $(RTL)/vision_linecache.v \
          $(RTL)/vision_linefill.v $(RTL)/vision_axi_master.v $(RTL)/vision_ortho_sys.v \
          $(RTL)/vision_div_pipe.v
ST_SRC  = $(RTL)/vision_selftest.v $(RTL)/vision_tpg.v $(RTL)/vision_top.v $(RTL)/vision_ccl.v $(RTL)/vision_div.v
HDR_SRC = $(RTL)/vision_pkg.vh $(RTL)/vision_ops.vh
REF_SRC = $(REF)/vision_ref.c $(REF)/ref_api.h

STIM  ?= /tmp/stim_color
FRAMES?= 2

# ---- P0 正射链路（2026-10-25）：每个模型一个 --Mdir，互不干扰 ----
ORTHO_DIR = obj_ortho
LC_DIR    = obj_lc
AXIM_DIR  = obj_axim
SYS_DIR   = obj_sys
DIVP_DIR  = obj_divp
ORTHO_SRC = $(RTL)/vision_ortho.v $(RTL)/vision_div_pipe.v
LC_SRC    = $(RTL)/vision_linecache.v
AXIM_SRC  = $(RTL)/vision_axi_master.v
DIVP_SRC  = $(RTL)/vision_div_pipe.v
SYS_SRC   = $(RTL)/vision_ortho_sys.v $(RTL)/vision_ortho.v $(RTL)/vision_linecache.v \
            $(RTL)/vision_linefill.v $(RTL)/vision_axi_master.v $(RTL)/vision_div_pipe.v
#: 定点 oracle 的 VR_DBG_HQ 系数（data/fpga/quad.json 那一组；**tb 里不许手抄**）
ORTHO_COEF = 53183 2476 1584660 -1926 -48241 27506115 -211 3684 16777216 \
             31413589 87381 8041878869 22369621
CAM ?= ../../../data/fpga/ortho_stim/cam_00.ppm

all: $(Mdir)/V$(TOP) $(Mdir)/Vvision_axi $(Mdir)/Vvision_selftest \
     $(ORTHO_DIR)/Vvision_ortho $(LC_DIR)/Vvision_linecache \
     $(AXIM_DIR)/Vvision_axi_master $(SYS_DIR)/Vvision_ortho_sys $(DIVP_DIR)/Vvision_div_pipe

# ---- P0：正射主体（直供源像素，验算术）----
$(ORTHO_DIR)/Vvision_ortho: $(ORTHO_SRC) tb_ortho.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 -Wno-fatal \
		--top-module vision_ortho -I$(RTL) --Mdir $(ORTHO_DIR) \
		$(ORTHO_SRC) tb_ortho.cpp -o Vvision_ortho

# ---- P0：行缓存（直接灌 fill 流）----
$(LC_DIR)/Vvision_linecache: $(LC_SRC) tb_linecache.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 -Wno-fatal \
		--top-module vision_linecache -I$(RTL) --Mdir $(LC_DIR) \
		$(LC_SRC) tb_linecache.cpp -o Vvision_linecache

# ---- P0：流水线除法器（200k 对随机数 vs C 的截断整除）----
$(DIVP_DIR)/Vvision_div_pipe: $(DIVP_SRC) tb_divpipe.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 -Wno-fatal \
		--top-module vision_div_pipe -I$(RTL) --Mdir $(DIVP_DIR) \
		$(DIVP_SRC) tb_divpipe.cpp -o Vvision_div_pipe

# ---- P0：AXI 读主口（DDR 模型）----
$(AXIM_DIR)/Vvision_axi_master: $(AXIM_SRC) tb_axim.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 -Wno-fatal \
		--top-module vision_axi_master -I$(RTL) --Mdir $(AXIM_DIR) \
		$(AXIM_SRC) tb_axim.cpp -o Vvision_axi_master

# ---- P0：全链路（DDR → AXI → 行缓存 → 正射），与 oracle 逐字节比 ----
$(SYS_DIR)/Vvision_ortho_sys: $(SYS_SRC) tb_ortho_sys.cpp
	$(VERILATOR) --cc --exe --build -j 4 -O2 -Wno-fatal \
		--top-module vision_ortho_sys -I$(RTL) --Mdir $(SYS_DIR) \
		$(SYS_SRC) tb_ortho_sys.cpp -o Vvision_ortho_sys

run-ortho: $(ORTHO_DIR)/Vvision_ortho
	./$(ORTHO_DIR)/Vvision_ortho ../../../data/fpga/ortho_stim/cam_00.ppm /tmp/exec_00001.ppm $(ORTHO_COEF)

run-sys: $(SYS_DIR)/Vvision_ortho_sys
	./$(SYS_DIR)/Vvision_ortho_sys $(CAM) /tmp/sys_exp_00001.ppm $(ORTHO_COEF)

run-lc: $(LC_DIR)/Vvision_linecache
	./$(LC_DIR)/Vvision_linecache

run-divp: $(DIVP_DIR)/Vvision_div_pipe
	./$(DIVP_DIR)/Vvision_div_pipe 200000

run-axim: $(AXIM_DIR)/Vvision_axi_master
	./$(AXIM_DIR)/Vvision_axi_master 240
	./$(AXIM_DIR)/Vvision_axi_master 240 80

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
	rm -rf $(Mdir) $(ORTHO_DIR) $(LC_DIR) $(AXIM_DIR) $(SYS_DIR) $(DIVP_DIR) $(REF_OBJ)

.PHONY: all run run-axi run-selftest run-ortho run-sys run-lc run-axim run-divp clean

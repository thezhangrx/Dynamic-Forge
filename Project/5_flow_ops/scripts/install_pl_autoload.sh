#!/bin/bash
# ===========================================================================
# install_pl_autoload.sh —— 让板子**上电自动加载我们的位流**（在板子上跑）
#
#     bash install_pl_autoload.sh [install|uninstall|status]
#
# 为什么需要：启动分区（/dev/mmcblk0p1）里放的位流是**厂商的**
# `ai7030_top.bit` —— 也就是说每次上电，PL 里起来的是厂商的设计，不是我们的
# `vision_axi`。于是我们那个程序一访问 0x43C00000 就"地址没被 PL 译码"
# （devmem 那条路会给出很明确的提示，见 vision_driver_devmem.cpp）。
#
# 现在的做法是每次重启后手动 `echo bd_vision.bin > .../fpga0/firmware`。
# 这个脚本把它做成开机自动执行。
#
# ## 为什么用 systemd unit 而不是改 BOOT.bin
#
# 改 BOOT.bin 要动启动镜像（bootgen 重新打包 + 把 SD 卡插到读卡器上），
# 一旦打包错了板子**起不来**，而且不好回退。
# 开机加载只是一个 oneshot 服务：**失败也不影响启动**（PL 里留着厂商设计，
# 系统照常起来），卸载就是删掉两个文件。风险差一个量级。
#
# ## 幂等
# 重复执行 install 不会叠加 unit；uninstall 会把改动清干净。
# ===========================================================================
set -u

BIN_NAME=${BIN_NAME:-bd_vision.bin}      # /lib/firmware 下的文件名
FW_DIR=/lib/firmware
MGR=/sys/class/fpga_manager/fpga0
UNIT=/etc/systemd/system/pl-autoload.service

usage() { echo "用法: $0 [install|uninstall|status]"; exit 2; }
[ $# -ge 1 ] || usage
cmd=$1

case "$cmd" in
install)
    if [ ! -f "$FW_DIR/$BIN_NAME" ]; then
        echo "找不到 $FW_DIR/$BIN_NAME —— 先把位流传上去（scp bd_vision.bit.bin ...）" >&2
        exit 1
    fi
    if [ ! -d "$MGR" ]; then
        echo "找不到 $MGR —— 这板子没有 fpga_manager？" >&2
        exit 1
    fi
    echo "写入 $UNIT ..."
    cat > "$UNIT" <<EOF
# 上电自动把我们的位流加载进 PL。**失败不影响启动**（PL 里就留着厂商的设计）。
# 由 core/board/tools/install_pl_autoload.sh 安装，卸载：同脚本 uninstall。
[Unit]
Description=Load PL bitstream ($BIN_NAME) via fpga_manager（见 core/board/README.md）
# 等 /lib/firmware 和 sysfs 就绪；比 multi-user 早，好让依赖它的服务能直接用
After=local-fs.target
Before=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
# - 失败**不算错**（厂商位流还在），但要把状态打出来，方便查
ExecStart=/bin/sh -c 'echo $BIN_NAME > $MGR/firmware'
ExecStartPost=/bin/sh -c 'sleep 1; echo "  PL state: \$(cat $MGR/state)"; echo "  (若这里是 operating 就是加载成功了)"'

[Install]
WantedBy=multi-user.target
EOF
    systemctl daemon-reload
    systemctl enable pl-autoload.service 2>&1 | tail -2
    echo "试跑一次（不用重启）："
    systemctl start pl-autoload.service 2>&1 | tail -3
    systemctl --no-pager status pl-autoload.service 2>&1 | head -8
    ;;
uninstall)
    systemctl disable --now pl-autoload.service 2>/dev/null
    rm -f "$UNIT"
    systemctl daemon-reload
    echo "已卸载（位流文件 $FW_DIR/$BIN_NAME 保留，可手动加载）"
    ;;
status)
    echo "unit 文件: $([ -f "$UNIT" ] && echo 存在 || echo 不存在)"
    echo "是否使能: $(systemctl is-enabled pl-autoload.service 2>&1)"
    echo "是否成功: $(systemctl is-active pl-autoload.service 2>&1)"
    echo "PL state: $(cat $MGR/state 2>/dev/null)"
    echo "位流文件: $FW_DIR/$BIN_NAME $([ -f "$FW_DIR/$BIN_NAME" ] && echo "($(stat -c %s "$FW_DIR/$BIN_NAME") 字节)" || echo 缺失)"
    ;;
*) usage ;;
esac

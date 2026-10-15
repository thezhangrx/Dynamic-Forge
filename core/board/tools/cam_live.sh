#!/bin/bash
# ===========================================================================
# cam_live.sh —— 实时采集 → PL 视觉 → 决策 → 键盘（**在板子上跑**）
#
#     bash cam_live.sh [--frames 0] [--echo-action] [--no-keyboard] ...
#
# 为什么要这么绕：原来每帧都是
#     ffmpeg -frames:v 1 ... out.ppm && ./vision_board_devmem --ppm out.ppm
# 也就是**每帧重开一次摄像头设备 + 落一次盘**，实测 1.0～1.25 秒/帧（≈1 帧/秒）。
# 这个脚本改成"**一次开设备、连续出帧、原始 RGB 直接走管道**"，
# 上限就回到摄像头本身（实测 25.3 帧/秒）。
#
# ---------------------------------------------------------------------------
# 三个坑（都是实测踩出来的，不是理论）
# ---------------------------------------------------------------------------
# 1) `exposure_auto_priority=1`（默认）允许驱动**为了长曝光而降低帧率** ——
#    实测采集只有 14.6 帧/秒；关掉它 → **25.3 帧/秒**。
# 2) **自动曝光要好几帧才收敛**（实测第 5~9 帧才稳定，早期抓到过全黑的第 1 帧），
#    所以必须丢开头几帧 —— 见 `--skip-frames`。不丢就是拿黑图去识别。
# 3) **别硬杀流式进程**（另外：节点号会随重启变，所以别写死 /dev/videoN）（`kill -9` 一个正在 stream 的 ffmpeg/v4l2-ctl）：
#    设备会卡在 "Device or resource busy"，之后所有采集都失败。
#    真卡住了用 USB 层复位（等价拔插一次）：
#        echo -n 1-1.3 > /sys/bus/usb/drivers/usb/unbind; sleep 2
#        echo -n 1-1.3 > /sys/bus/usb/drivers/usb/bind;   sleep 4
#    本脚本的 `CAM_RESET=1` 就是干这个的（设备路径用 lsusb 确认）。
# ===========================================================================
set -u
set -o pipefail

# 摄像头节点**不要写死**：实测重启后编号会变（视频 capture 从 video1 变成 video0，
# 而 video1 变成了它的 **metadata 节点** —— 那个节点打开就报
# `ioctl(VIDIOC_G_INPUT): Inappropriate ioctl for device`，只能用 0 字节。
DEV=${DEV:-}                     # 留空 = 自动识别（推荐）；需要时用 DEV=/dev/videoN 覆盖
# 采集：**默认 MJPG 640x480 + 中心裁剪到 640x360**。
# 选它的依据是实测的"摄像头原始交付率"（这一环的硬上限）：
#     YUYV 640x480 → 25.2 帧/秒（未压缩，吃 USB 带宽）
#     MJPG 640x480 → **48.4 帧/秒**
#     MJPG 1280x720 → 60 帧/秒（但解码+缩放更贵）
# 另外屏幕是 16:9、相机是 4:3，**裁掉上下各 60 行**比缩放几何上更正确。
# 要换：CAM_MODE=yuyv，或 CAM_W/CAM_H 覆盖尺寸。
CAM_MODE=${CAM_MODE:-mjpg}       # mjpg 或 yuyv（见下面"实测交付率"的说明）
CAM_W=${CAM_W:-640}
CAM_H=${CAM_H:-480}
PIPE_W=${PIPE_W:-640}            # PL 的流水线固定 640x360（编译期常量）
PIPE_H=${PIPE_H:-360}
SKIP=${SKIP:-10}                 # 丢弃的热身帧（自动曝光收敛）
APP=${APP:-/root/board_src/build/vision_board_devmem}
CAM_RESET=${CAM_RESET:-0}        # 1 = 先做一次 USB 层复位
USB_DEV=${USB_DEV:-1-1.3}

# ---- 0) 可选：USB 层复位（设备卡在 busy 时用）-----------------------------
if [ "$CAM_RESET" = "1" ]; then
    echo "[cam_live] USB 复位 $USB_DEV ..."
    echo -n "$USB_DEV" > /sys/bus/usb/drivers/usb/unbind 2>/dev/null
    sleep 2
    echo -n "$USB_DEV" > /sys/bus/usb/drivers/usb/bind 2>/dev/null
    sleep 4
fi

# ---- 1) 自动识别摄像头节点 ------------------------------------------------
#
# 判据用"支持 MJPG"：这块板子上还有
#   * UVC 的 **metadata 节点**（video1）—— 能打开但一 stream 就
#     "Inappropriate ioctl for device"；
#   * Linlon 的 **M2M 编解码器**（video2）—— 它**也**列 YUYV，是个诱饵。
# 只有真摄像头会列 MJPG，所以拿它当判据最稳。
detect_dev() {
    for d in /dev/video*; do
        [ -e "$d" ] || continue
        if timeout 10 v4l2-ctl -d "$d" --list-formats-ext 2>/dev/null | grep -q MJPG; then
            echo "$d"; return 0
        fi
    done
    return 1
}
if [ -z "$DEV" ]; then
    if ! DEV=$(detect_dev); then
        echo "[cam_live] 找不到支持 MJPG 的摄像头 —— 插了吗？以下是现有节点：" >&2
        v4l2-ctl --list-devices >&2 2>&1 || true
        exit 1
    fi
    echo "[cam_live] 自动识别到摄像头: $DEV"
fi
if [ ! -e "$DEV" ]; then
    echo "[cam_live] 找不到 $DEV —— 摄像头插了吗？v4l2-ctl --list-devices 看一眼" >&2
    exit 1
fi
if [ ! -x "$APP" ]; then
    echo "[cam_live] 找不到可执行文件 $APP —— 先在板上 cmake --build build 编一次" >&2
    exit 1
fi

# ---- 1.5) 放开管道上限 -----------------------------------------------------
# 板子程序会把 stdin 管道放大到 3MB（一帧 691200B，默认 64KB 根本装不下一帧，
# 于是 ffmpeg 写满就阻塞、"采集/处理"被迫串行）。上限受这个 sysctl 限制。
MAXP=/proc/sys/fs/pipe-max-size
if [ -w "$MAXP" ]; then
    cur=$(cat "$MAXP" 2>/dev/null || echo 0)
    if [ "$cur" -lt 4194304 ] 2>/dev/null; then
        echo 4194304 > "$MAXP" 2>/dev/null && echo "[cam_live] pipe-max-size: $cur → $(cat $MAXP)"
    fi
fi

# ---- 2) 关掉"为长曝光降帧率"（14.6 → 25.3 帧/秒）--------------------------
v4l2-ctl -d "$DEV" -c exposure_auto_priority=0 >/dev/null 2>&1 \
    || echo "[cam_live] 警告：设 exposure_auto_priority 失败（帧率可能只有 ~14）" >&2

# ---- 3) 一次开设备、连续出帧、原始 RGB 走管道 → 板子程序 -------------------
if [ "$CAM_MODE" = "mjpg" ]; then
    IN_FMT=mjpeg
else
    IN_FMT=yuyv422
fi
# 尺寸适配：**能裁就裁**（屏幕是 16:9，相机是 4:3；裁掉上下各 60 行几何不失真），
# 尺寸不同才缩放。
#
# 实测三种组合的**摄像头原始交付率**（决定了这一环的上限）：
#     YUYV 640x480 → 25.2 帧/秒      ← 未压缩，USB 带宽受限
#     MJPG 640x480 → 48.4 帧/秒
#     MJPG 1280x720 → 60 帧/秒（但解码/缩放更贵，且和我们的程序争 CPU）
# 所以默认用 **MJPG 640x480 + 裁剪**：交付率翻倍，解码量却比 720p 小。
if [ "$CAM_H" -gt "$PIPE_H" ]; then
    CROP_Y=$(( (CAM_H - PIPE_H) / 2 ))
    VF="crop=${PIPE_W}:${PIPE_H}:0:${CROP_Y}"
else
    VF="scale=${PIPE_W}:${PIPE_H}"
fi
echo "[cam_live] $DEV ${CAM_W}x${CAM_H} $IN_FMT -vf $VF → RGB → $APP $*"
ffmpeg -hide_banner -loglevel error \
       -f v4l2 -input_format "$IN_FMT" -video_size "${CAM_W}x${CAM_H}" -i "$DEV" \
       -vf "$VF" -pix_fmt rgb24 -f rawvideo - \
  | "$APP" --stdin-rgb --skip-frames "$SKIP" "$@"
rc=$?
echo "[cam_live] 退出码 $rc（$([ $rc -eq 0 ] && echo 正常 || echo 失败/被打断)）"
exit $rc

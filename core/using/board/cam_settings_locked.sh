#!/bin/bash
# ===========================================================================
# cam_settings_locked.sh —— **acc9r16 这一轮锁死的相机参数**（2026-10-24）
# 来源：板上 /root/vision_state/cam_settings.sh（开机由 restore_vision_state.sh 调用）
# 这一轮（run_20261024_181610_acc9r16）实测：0 碰撞 / 24-24 / 最小间隙 +9.92
# 其余控件保持驱动默认：gamma=120 contrast=32 白平衡=自动 曝光=自动(exposure_auto=3)
# 用法（板子上）：bash cam_settings_locked.sh
# ===========================================================================
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=16 2>/dev/null
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=70 2>/dev/null
v4l2-ctl -d /dev/video0 --set-ctrl=gain=4 2>/dev/null
v4l2-ctl -d /dev/video0 --set-ctrl=power_line_frequency=1 2>/dev/null

#!/bin/bash
# ===========================================================================
# fix_boot_stall.sh —— 修掉"上电要干等 90 秒才进得去系统"
#
# 在**板子上**以 root 跑（幂等，可重复执行）：
#     scp core/board/tools/fix_boot_stall.sh root@192.168.137.242:/root/
#     ssh root@192.168.137.242 bash /root/fix_boot_stall.sh
#
# ---------------------------------------------------------------------------
# 症状
# ---------------------------------------------------------------------------
#   上电后长时间连不上：SSH 要 90 秒以上才通，串口那边看着像卡住了。
#   `systemd-analyze` 实测（修复前）：
#       Startup finished in 7.246s (kernel) + 1min 30.813s (userspace) = 1min 38.059s
#   也就是说内核只花 7 秒，**用户态却要 1 分 30 秒**。
#
# ---------------------------------------------------------------------------
# 根因：厂商 rootfs 里没有 udev，但内核 cmdline 让 generator 生成了一个
#       带 BindsTo=dev-*.device 的 serial getty
# ---------------------------------------------------------------------------
#   1) 这个 rootfs **根本没装 systemd-udevd**（不是没启动，是连 unit 文件都没有）：
#          # systemctl is-enabled systemd-udevd.service
#          Failed to get unit file state ...: No such file or directory
#   2) 没有 udev，systemd 就永远收不到"设备已就绪"的通知，`dev-*.device`
#      这个 unit 永远不会变成 active —— 而它又不会立刻失败，只会**干等到
#      默认的 90 秒 JobTimeout 才超时**。
#   3) 内核 cmdline 是 `console=ttyPS0,115200n8`，于是 systemd-getty-generator
#      自动生成了 `serial-getty@ttyPS0.service`，而
#      `/lib/systemd/system/serial-getty@.service` 里写着：
#          BindsTo=dev-%i.device
#          After=dev-%i.device ...
#      → 这个 getty 的 job 会去等 dev-ttyPS0.device → 等满 90 秒 → 超时。
#   4) 厂商自己又启了一个 `getty@ttyPS0.service`
#      （/etc/systemd/system/getty.target.wants/ 里的软链），
#      `systemctl show -p BoundBy dev-ttyPS0.device` 确认**只有 serial 那个**绑它。
#
#   结果就是**两个 getty 抢同一个串口**，其中一个注定失败，还顺手把
#   `Reached target Multi-User System` 卡在超时那一秒。失败那次的时间线：
#       03:28:20 Started Getty on ttyPS0.          ← 厂商那个，正常
#       03:29:41 dev-ttyPS0.device: timed out      ← 干等 88 秒
#       03:29:41 Dependency failed for Serial Getty on ttyPS0.
#       03:29:41 Reached target Multi-User System. ← 超时同一秒才到 multi-user
#
#   为什么以前没注意：这一条**每次上电都会发生**（不是偶发），但因为最后
#   总能起来，就被当成"板子慢"而不是"配错了"。它也确实会偶发地更糟：
#   驱动探测快于 systemd 时设备已存在，就碰巧不等 —— 竞态。
#
# ---------------------------------------------------------------------------
# 修法：屏蔽掉那个注定失败的重复 getty（不动提供登录的那个）
# ---------------------------------------------------------------------------
#   屏蔽后 `BoundBy` 为空 → 没人再去拉 dev-ttyPS0.device 的 job → 90 秒消失。
#   串口登录不受影响，因为提供 `U login:` 的一直是厂商的 getty@ttyPS0.service。
#
#   实测（修复后，同一条命令）：
#       Startup finished in 3.186s (kernel) + 5.655s (userspace) = 8.842s
#   设备超时次数 0，SSH 20 秒内就恢复，串口 `U login:` 依旧正常。
#
#   注意：**不要**用改内核 cmdline（去掉 console=）的办法 —— 那会连串口
#   控制台一起弄丢，而控制台是这块板子唯一可靠的观测手段（JTAG 因为国产
#   克隆片的 DAP 限制，枚举不出 Cortex-A9，不能当判据）。
# ===========================================================================
set -u

if [ "$(id -u)" != "0" ]; then
    echo "需要 root（/etc/systemd 和 /var/log）" >&2
    exit 1
fi

echo "=== ① 确认这个 rootfs 确实没有 udev ==="
if systemctl is-enabled systemd-udevd.service >/dev/null 2>&1; then
    echo "  警告：systemd-udevd 存在，根因可能不同 —— 本脚本可能不适用"
else
    echo "  systemd-udevd 不存在（符合本 rootfs 的已知情况）"
fi

echo
echo "=== ② 屏蔽重复的 serial getty（幂等）==="
if [ -L /etc/systemd/system/serial-getty@ttyPS0.service ]; then
    echo "  已经屏蔽过，跳过"
else
    systemctl mask serial-getty@ttyPS0.service
fi
systemctl daemon-reload
echo "  dev-ttyPS0.device 的 BoundBy 现在为: [$(systemctl show -p BoundBy --value dev-ttyPS0.device)]"
echo "  （这一项为空才是修好了；不为空说明还有别的 unit 在等它）"

echo
echo "=== ③ 确认提供串口登录的那个 getty 还在 ==="
if systemctl is-enabled getty@ttyPS0.service >/dev/null 2>&1; then
    echo "  getty@ttyPS0.service 仍是 enabled ✓（U login: 由它提供）"
else
    echo "  ✗ getty@ttyPS0.service 不是 enabled —— 先别重启！" >&2
    echo "    否则会失去唯一的串口登录口" >&2
    exit 1
fi

echo
echo "=== ④ 顺手修 rsyslog 的 Permission denied ==="
# 厂商把 /var/log 设成 0755 root:syslog，而 rsyslog 以 syslog 用户跑，
# 建不了 /var/log/syslog、auth.log、kern.log，于是每次启动都报
#   rsyslogd: file '/var/log/syslog': open error: Permission denied
# Ubuntu 上这个目录本来就是 0775 root:syslog。
chmod 775 /var/log
echo "  /var/log 权限: $(stat -c '%a %U:%G' /var/log) （应为 775 root:syslog）"

echo
echo "=== 完成。验证方式（会重启板子）==="
echo "  systemctl reboot"
echo "  # 回来后对比：修复前 userspace 1min30.8s → 修复后应在 10s 以内"
echo "  systemd-analyze"
echo "  journalctl -b | grep -c 'Timed out waiting for device'   # 应为 0"

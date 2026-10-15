#!/usr/bin/env python3
"""PC 侧按键注入的映射必须和 RP2040 那份**同义**，否则"换台机器就不好使"。

`pc_key_driver.py`（软件注入，不用额外硬件）和 `code.py`（RP2040 真 HID）是
同一件事的两条实现。两边各写一张"动作号 → 按键"的表迟早会分家，而且**分家了
不报错，只会朝错方向走**。所以：

1. `pc_key_driver` 的键**从协议的 `DISCRETE_DIRECTIONS` 推导**，不另立一张表；
2. 这里把它和 `code.py` 的 `KEYS` 表（CircuitPython 的 Keycode 名）**逐项对起来** ——
   两份独立实现、同一个协议真值，必须给出同一组方向；
3. 差集逻辑（换向时不松开共用的键）单独测，这决定斜向切换会不会抖。
"""

from __future__ import annotations

import pathlib
import re
import select
import shutil
import socket
import subprocess
import sys
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
RP2040 = ROOT / "core" / "rp2040_keyboard"
CODE_PY = RP2040 / "code.py"

sys.path.insert(0, str(RP2040))
sys.path.insert(0, str(ROOT / "core"))

import pc_key_driver as drv  # noqa: E402
from bullet_sim.core.actions import (  # noqa: E402
    DISCRETE_ACTION_NAMES,
    DISCRETE_DIRECTIONS,
)

#: 键位方案 → (虚拟键码 → 方向名)。**每套各建一份**：arrows 与 wasd 的键码不同，
#: 只建 arrows 那套的话查 wasd 会 KeyError（这条测试最初就是这么错的）。
VK_TO_NAME = {km: {vk: name for name, vk in table.items()}
              for km, table in drv.VK.items()}


def _code_py_keys() -> dict[int, set[str]]:
    """解析 code.py 的 KEYS 表：动作号 → 方向名集合。"""
    src = CODE_PY.read_text(encoding="utf-8")
    m = re.search(r"^KEYS\s*=\s*\{(.*?)^\}", src, re.M | re.S)
    assert m, "code.py 里找不到 KEYS 表"
    out: dict[int, set[str]] = {}
    for idx, body in re.findall(r"(\d+)\s*:\s*\(([^)]*)\)", m.group(1)):
        names = set()
        for kc in re.findall(r"Keycode\.(\w+)_ARROW", body):
            names.add(kc)
        out[int(idx)] = names
    return out


# ---------------------------------------------------------------------------
# 1) 从协议真值推导，且与 code.py 一致
# ---------------------------------------------------------------------------
def test_every_action_matches_protocol_direction() -> None:
    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        x, y = float(DISCRETE_DIRECTIONS[i][0]), float(DISCRETE_DIRECTIONS[i][1])
        want = set()
        if x < 0: want.add("LEFT")
        if x > 0: want.add("RIGHT")
        if y > 0: want.add("UP")
        if y < 0: want.add("DOWN")
        got = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i)}
        assert got == want, (
            f"动作 {i}（{name}）推导出的方向不对：拿到 {sorted(got)}，"
            f"协议方向 {DISCRETE_DIRECTIONS[i]} 应为 {sorted(want)}")


def test_pc_driver_and_rp2040_agree_on_every_action() -> None:
    """PC 软件注入与 RP2040 真 HID 必须按同一组键 —— 否则换个方案就朝错方向走。"""
    code_keys = _code_py_keys()
    assert set(code_keys) == set(range(len(DISCRETE_ACTION_NAMES))), \
        f"code.py 的 KEYS 覆盖不全：{sorted(code_keys)}"
    for i, name in enumerate(DISCRETE_ACTION_NAMES):
        pc = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i)}
        rp = code_keys[i]
        assert pc == rp, (
            f"动作 {i}（{name}）两边按的键不一致：\n"
            f"  pc_key_driver（软件注入）: {sorted(pc)}\n"
            f"  code.py（RP2040 HID）    : {sorted(rp)}\n"
            "两者是同一件事的两条实现，必须同义")


def test_wasd_option_maps_same_directions() -> None:
    """WASD 那套只是换键码，方向语义必须完全一样。"""
    for i in range(len(DISCRETE_ACTION_NAMES)):
        a = {VK_TO_NAME["arrows"][k] for k in drv.keys_for_action(i, "arrows")}
        b = {VK_TO_NAME["wasd"][k] for k in drv.keys_for_action(i, "wasd")}
        assert a == b, f"动作 {i} 在 arrows 与 wasd 下方向不一致：{a} vs {b}"
    assert drv.keys_for_action(2, "wasd") == (0x44,)      # right → D
    assert drv.keys_for_action(1, "wasd") == (0x41,)      # left  → A
    assert drv.keys_for_action(3, "wasd") == (0x57,)      # up    → W
    assert drv.keys_for_action(4, "wasd") == (0x53,)      # down  → S


# ---------------------------------------------------------------------------
# 2) 差集逻辑（决定斜向切换抖不抖）
# ---------------------------------------------------------------------------
def test_stay_presses_and_releases_nothing() -> None:
    assert drv.key_diff((), ()) == ([], [])


def test_switching_diagonals_keeps_the_shared_key_down() -> None:
    """右上 → 右下：RIGHT 必须**保持按住**，只换 UP→DOWN。

    若把 RIGHT 也松开再按，平台会把它当成"松了又按"，角色会一顿一顿的。
    """
    up_right = drv.keys_for_action(6)
    down_right = drv.keys_for_action(8)
    press, release = drv.key_diff(up_right, down_right)
    assert drv.VK["arrows"]["RIGHT"] not in release, "RIGHT 不该被松开"
    assert drv.VK["arrows"]["RIGHT"] not in press, "RIGHT 不该被重复按下"
    assert press == [drv.VK["arrows"]["DOWN"]]
    assert release == [drv.VK["arrows"]["UP"]]


def test_going_to_stay_releases_everything() -> None:
    held = drv.keys_for_action(5)                      # up_left
    press, release = drv.key_diff(held, ())
    assert press == []
    assert set(release) == set(held)


def test_out_of_range_action_is_a_no_op() -> None:
    """越界动作号不能按出乱七八糟的键（控制台噪声就可能撞上）。"""
    assert drv.keys_for_action(-1) == ()
    assert drv.keys_for_action(99) == ()


# ---------------------------------------------------------------------------
# 3) Windows 注入器能起来（**只 PING，绝不按键**）
# ---------------------------------------------------------------------------
def test_powershell_injector_loads_and_pings() -> None:
    """注入器的 `Add-Type`（P/Invoke 声明）必须能编译过。

    这是最容易写错、又最难在真按键时发现的一环（写错了只是按不动，
    而"按不动"会被误当成"平台不吃合成输入"）。所以这里只发 `PING`：
    **不注入任何按键**，只验证脚本能加载、命令循环能应答。
    """
    ps = shutil.which("powershell.exe")
    if ps is None:
        pytest.skip("没有 powershell.exe（不在 WSL / 不是 Windows）")
    ps1 = RP2040 / "win_sendinput.ps1"
    assert ps1.exists(), "缺 win_sendinput.ps1"
    r = subprocess.run(
        [ps, "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", str(ps1)],
        input="PING\nQUIT\n", capture_output=True, text=True, timeout=120)
    assert r.returncode == 0, f"注入器退出码 {r.returncode}：\n{r.stdout}\n{r.stderr}"
    assert "PONG" in r.stdout, f"注入器没应答 PONG：\n{r.stdout}\n{r.stderr}"


# ---------------------------------------------------------------------------
# 3) 命令行契约：C 路的入口就这几条命令，行为不能悄悄变
# ---------------------------------------------------------------------------
def _run(*argv: str, timeout: float = 30.0) -> subprocess.CompletedProcess:
    return subprocess.run([sys.executable, str(RP2040 / "pc_key_driver.py"), *argv],
                          capture_output=True, text=True, timeout=timeout)


def test_source_is_required_and_exclusive() -> None:
    """不给来源 / 同时给两个来源，都必须报错退出（而不是默默干等）。"""
    r = _run("--press", "0", "--delay-s", "0")     # 有 --press，不走来源检查
    assert r.returncode == 2, r.stdout + r.stderr
    r = _run("--console", "socket://127.0.0.1:1", "--udp", "0")
    assert r.returncode != 0, "两个来源同时给应当报错"
    assert "只能选一个" in (r.stdout + r.stderr)


def test_press_rejects_out_of_range_and_stay() -> None:
    """`--press` 是判定"平台收不收合成按键"的手段，越界和 stay 都测不出东西。"""
    assert _run("--press", "99", "--delay-s", "0").returncode == 2
    assert _run("--press", "1,99", "--delay-s", "0").returncode == 2, "序列里有一个越界就该整体拒绝"
    r = _run("--press", "0", "--delay-s", "0")
    assert r.returncode == 2 and "不按任何键" in r.stdout
    assert _run("--press", "1,x", "--delay-s", "0").returncode == 2
    assert _run("--press", "", "--delay-s", "0").returncode == 2
    # 逗号和空格都认（命令行里手写更顺）
    assert _run("--press", "1 2", "--delay-s", "0", "--hold-ms", "10").returncode == 0


def test_press_dry_run_lists_every_action_and_defaults_to_dry_run() -> None:
    """干跑要把每个动作和它要按的键列出来；默认绝不能真按键。"""
    r = _run("--press", "3", "--delay-s", "0", "--hold-ms", "50")
    assert r.returncode == 0, r.stdout + r.stderr
    assert "动作 3（up）" in r.stdout and "按住 [38]" in r.stdout, r.stdout
    assert "干跑" in r.stdout, "默认必须是干跑，不能真按键"


def test_press_sequence_covers_four_directions() -> None:
    """一次跑完四个方向：这样才能一次运行同时判定"收不收"和"键位对不对"。"""
    r = _run("--press", "1,2,3,4", "--delay-s", "0", "--hold-ms", "10", "--gap-ms", "10")
    assert r.returncode == 0, r.stdout + r.stderr
    for i, name in enumerate(("left", "right", "up", "down"), 1):
        assert f"[{i}/4] 动作 {i}（{name}）" in r.stdout, r.stdout
    for vk in drv.VK["arrows"].values():
        assert f"按住 [{vk}]" in r.stdout, f"缺了键码 {vk}：\n{r.stdout}"
    # WASD 那套只是换键码，方向语义一样
    r2 = _run("--press", "1", "--keys", "wasd", "--delay-s", "0", "--hold-ms", "10")
    assert "按住 [65]" in r2.stdout, r2.stdout      # A = 左


def test_udp_source_matches_console_source() -> None:
    """UDP 源和串口源必须产生**同一串按键** —— 换传输不该改变行为。"""
    # 探针 socket 必须先关掉再起子进程：否则它自己占着这个端口，
    # 子进程 bind 会 EADDRINUSE（这条测试最初就是这么挂的）。
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()

    p = subprocess.Popen(
        [sys.executable, str(RP2040 / "pc_key_driver.py"), "--udp", str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        # 等它说出"就绪"再发：绑端口之前的报文会被内核丢掉（同 relay 那条测试）
        deadline = time.monotonic() + 10.0
        ready = False
        while time.monotonic() < deadline and not ready:
            r, _, _ = select.select([p.stdout], [], [], deadline - time.monotonic())
            if not r:
                break
            line = p.stdout.readline()
            if not line:
                break
            ready = "UDP :" in line
        assert ready, "驱动没在 10s 内就绪"

        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(b"@MOVE 1\n", ("127.0.0.1", port))       # left
        time.sleep(0.3)
        sk.sendto(b"@MOVE 1\n", ("127.0.0.1", port))       # 同向：不该有任何按键变化
        time.sleep(0.3)
        sk.sendto(b"@MOVE 5\n", ("127.0.0.1", port))       # up_left：LEFT 保持，只加 UP
        time.sleep(0.3)
        sk.sendto(b"@MOVE 0\n", ("127.0.0.1", port))       # stay：全松
        time.sleep(0.3)
        sk.close()
        p.terminate()
        out, _ = p.communicate(timeout=10)
    finally:
        if p.poll() is None:
            p.kill()

    vk_left, vk_up = drv.VK["arrows"]["LEFT"], drv.VK["arrows"]["UP"]
    # 只算"真的按了键"的行：松开-only 的行前缀同样是「→ 按住 」（按住空列表），
    # 直接 count("→ 按住") 会把松开那行也算进去（这条断言最初就是这么错的）。
    presses = [ln for ln in out.splitlines()
               if "→ 按住" in ln and "按住 []" not in ln]
    assert len(presses) == 2, f"同向重复不该再按键，应当只有 2 次按下：\n{out}"
    assert f"按住 [{vk_left}] 松开 []" in out, out
    assert f"按住 [{vk_up}] 松开 []" in out, f"斜向应当只加 UP：\n{out}"
    # 松开顺序 = keys_for_action 的顺序（先 y 后 x），两个键都要松掉
    assert f"按住 [] 松开 [{vk_up}, {vk_left}]" in out, f"stay 应当全松：\n{out}"


# ---------------------------------------------------------------------------
# 4) SSH 管道：板子 `--keyboard-dev stdout` | 驱动 `--stdin`
#
# 这条路是 C 路里最省事的一条：复用已经在用的 SSH 通道，不用接引脚、不用
# usbipd 挂串口、也不用动 Windows 防火墙。它是**成对**的：发射端写 stdout，
# 接收端读 stdin，两边任何一边被改掉，命令就静默不工作了 —— 所以两边一起钉。
# ---------------------------------------------------------------------------
def test_stdin_source_end_to_end() -> None:
    """管道里混着日志也要只挑 `@MOVE` 行，读完 EOF 要干净收工。"""
    stream = (
        '键盘桥: stdout → 标准输出（已打开，发 "@MOVE <n>"）\n'
        "帧    1: seq=1  长度=76 B  障碍=0  缺口=0  尺寸=640x360\n"
        "@MOVE 1\n"           # left
        "@MOVE 1\n"           # 同向重复：不该再按
        "  动作: 1 (left)\n"   # 日志里也带动作号，但不是协议行，必须忽略
        "@MOVE 5\n"           # up_left：只加 UP
        "@MOVE 0\n"           # stay：全松
    ).encode("utf-8")
    r = subprocess.run(
        [sys.executable, str(RP2040 / "pc_key_driver.py"), "--stdin"],
        input=stream, capture_output=True, timeout=30)
    assert r.returncode == 0, r.stdout + r.stderr
    out = r.stdout.decode("utf-8", "replace")
    vk_left, vk_up = drv.VK["arrows"]["LEFT"], drv.VK["arrows"]["UP"]
    # 只算"真的按了键"的行（松开行也以「→ 按住 」开头，见上一条测试的坑）
    presses = [ln for ln in out.splitlines()
               if "→ 按住" in ln and "按住 []" not in ln]
    assert len(presses) == 2, f"应当只按两次（left、left+up）：\n{out}"
    assert f"按住 [{vk_left}] 松开 []" in out, out
    assert f"按住 [{vk_up}] 松开 []" in out, f"斜向只该加 UP：\n{out}"
    assert f"按住 [] 松开 [{vk_up}, {vk_left}]" in out, f"stay 该全松：\n{out}"
    assert "输入结束" in out, f"管道关了应当收工退出（否则会空转）：\n{out}"


def test_stdin_is_mutually_exclusive_with_other_sources() -> None:
    r = _run("--stdin", "--udp", "0")
    assert r.returncode != 0 and "只能选一个" in (r.stdout + r.stderr), r.stdout + r.stderr


def test_emitter_has_matching_stdout_sink() -> None:
    """发射端必须有配对的 `stdout` 目标，而且**必须 flush**。

    stdout 接管道时是块缓冲的 —— 不 flush 的话动作串会卡在用户态缓冲区里，
    直到进程退出才吐出来，那就完全不是实时控制了（这个坑很隐蔽：干跑看着
    "有输出"，因为退出时刷了）。
    """
    src = (ROOT / "core" / "board" / "main.cpp").read_text(encoding="utf-8")
    assert re.search(r'target\s*==\s*"stdout"\s*\|\|\s*target\s*==\s*"-"', src), \
        "main.cpp 的 ActionEmitter 不认 stdout / - 这种目标"
    assert "stdout_ = true" in src, "少了 stdout 目标的置位"
    assert re.search(r"std::fflush\(stdout\)", src), \
        "写 stdout 后必须 fflush，否则管道里拿不到实时动作"
    # 帮助文本是多个相邻字符串字面量拼起来的，所以按**原文位置**截一段来看
    at = src.index("--keyboard-dev T")
    assert "stdout" in src[at:at + 400], \
        "--keyboard-dev 的帮助文本里要写清 stdout 这种写法"


def test_readme_documents_the_ssh_pipeline() -> None:
    """文档里的这条命令两端必须成套出现，否则用户照着敲就是坏的。"""
    for name in ("README.md", "pc_key_driver.py"):
        text = (RP2040 / name).read_text(encoding="utf-8")
        assert "--keyboard-dev stdout" in text, f"{name} 少了发射端写法"
        assert "--stdin" in text, f"{name} 少了接收端写法"

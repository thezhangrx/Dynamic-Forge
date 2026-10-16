"""板子控制平台（`board-control`）：协议解析、传输、看门狗、`--nostop`。

板子那条链路是**跨语言**的：C++（`core/board/action_codec.hpp`）写、Python 读。
这里钉住三件事：

1. **解析规则必须和键盘桥一模一样**（`console_relay.parse_moves`）：同一批字节
   必须解析出同一串动作号、留下同一段残渣。差一条就会"看起来也能跑，只是偶尔不动"；
2. **传输**（stdin / UDP / 串口）能真的把字节变成 `Action`，并且**断流回中**
   （看门狗）——不放的话板子一断，仿真里的角色会一直朝最后那个方向飞；
3. **`--nostop` 真的不自动停**：不设步数上限、也不因回合结束（truncated）停下。

不需要板子、不需要摄像头：stdin/UDP 用管道和本地 socket 就能验。
"""

from __future__ import annotations

import json
import os
import pathlib
import select
import socket
import subprocess
import sys
import time

import pytest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "core"))
# console_relay 不是包（没有 __init__.py），按路径加载它来对账 —— 仓库里的老规矩
sys.path.insert(0, str(ROOT / "core" / "rp2040_keyboard"))

from bullet_sim.hardware_interface.board_link import (  # noqa: E402
    MAX_N,
    PREFIX,
    StdinBoardAdapter,
    UdpBoardAdapter,
    parse_moves,
)


# ---------------------------------------------------------------------------
# 1) 解析：与键盘桥逐条一致
# ---------------------------------------------------------------------------
def test_prefix_and_range_match_the_keyboard_bridge() -> None:
    import console_relay as cr

    assert PREFIX == cr.PREFIX, "前缀必须与 console_relay / action_codec.hpp 一致"
    assert MAX_N == cr.MAX_N, "动作号上界必须一致（0..8）"


_BATCHES = [
    b"@MOVE 3\n@MOVE 0\n",
    b"@MO", b"VE 5\n",                       # 一行被切开
    b"junk\n@MOVE 9\n@MOVE -1\n@MOVE 2.5\n@MOVE 4\n",   # 噪声 + 越界
    b"@MOVE \n@MOVE 8\r\n@MOVE-not\n@MOVE 007\n",
    b"no newline at all", b"@MOVE 1", b"\n2\n", b"@MOVE 0\r\n@MOVE 8\r\n",
]


@pytest.mark.parametrize("batch", _BATCHES)
def test_parse_matches_console_relay_exactly(batch: bytes) -> None:
    import console_relay as cr

    mine, theirs = bytearray(), bytearray()
    a = parse_moves(batch, mine)
    b = cr.parse_moves(batch, theirs)
    assert a == b, f"动作号不一致：{a} vs {b}（输入 {batch!r}）"
    assert bytes(mine) == bytes(theirs), (
        f"半行残渣不一致：{bytes(mine)!r} vs {bytes(theirs)!r}（输入 {batch!r}）")


def test_residue_does_not_grow_forever() -> None:
    buf = bytearray()
    for _ in range(20):
        parse_moves(b"x" * 1000, buf)
    assert len(buf) <= 4096, "没有换行的日志不能把内存吃光"


# ---------------------------------------------------------------------------
# 2) 传输：stdin / UDP + 看门狗
# ---------------------------------------------------------------------------
def _drain(adapter, n: int, timeout: float = 2.0) -> list:
    """轮询到拿到 n 个动作或超时。"""
    out, t0 = [], time.monotonic()
    while len(out) < n and time.monotonic() - t0 < timeout:
        a = adapter.poll(1.0 / 60.0)
        if a is not None:
            out.append(a)
        time.sleep(0.005)
    return out


def test_stdin_transport_maps_move_to_action() -> None:
    r, w = os.pipe()
    os.set_blocking(w, False)
    try:
        ad = StdinBoardAdapter(fd=r, hold_s=0.05)
        ad.open()
        os.write(w, b"noise line\n@MOVE 1\n@MOVE 5\n")
        got = _drain(ad, 2)
        assert len(got) == 2, f"应当拿到 2 个动作，得到 {len(got)}"
        # 1 = 左（-x）；5 = 左上（-x, +y）。⚠ `Action` 把方向**归一化**、幅度另存，
        #   所以斜向是 (-0.707, +0.707) 而不是 (-1, +1) —— 这是它的设计。
        assert got[0].direction[0] == pytest.approx(-1.0)
        assert got[1].direction[0] == pytest.approx(-0.70710678, abs=1e-6)
        assert got[1].direction[1] == pytest.approx(+0.70710678, abs=1e-6)
        assert ad.stats.polls >= 2
    finally:
        os.close(r)
        os.close(w)


def test_udp_transport_maps_move_to_action() -> None:
    ad = UdpBoardAdapter(port=0, hold_s=0.05)
    ad.open()
    try:
        assert ad.port > 0, "port=0 时应当回填真实端口"
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(b"@MOVE 2\n", ("127.0.0.1", ad.port))       # 右
        got = _drain(ad, 1)
        assert len(got) == 1 and got[0].direction[0] == pytest.approx(1.0)
        sk.close()
    finally:
        ad.close()


def test_watchdog_returns_to_stay_when_the_board_goes_quiet() -> None:
    """断流必须回中：否则板子一断，角色会一直朝最后那个方向飞。"""
    ad = UdpBoardAdapter(port=0, hold_s=0.05)
    ad.open()
    try:
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(b"@MOVE 1\n", ("127.0.0.1", ad.port))
        got = _drain(ad, 1)
        assert got and got[0].direction[0] == pytest.approx(-1.0)
        assert ad.poll(0.016) is None, "还没到 hold 时间不该突然回中"
        time.sleep(0.08)
        a = ad.poll(0.016)
        assert a is not None and a.direction == (0.0, 0.0), "断流超时后必须回中"
        assert ad.n_holds == 1
        sk.close()
    finally:
        ad.close()


# ---------------------------------------------------------------------------
# 3) CLI：真的把板子信号喂进仿真，而且 `--nostop` 不自动停
# ---------------------------------------------------------------------------
def _cli(args: list[str], *, stdin_data: bytes = b"", timeout: float = 180.0):
    return subprocess.run(
        [sys.executable, "-m", "bullet_sim", *args],
        input=stdin_data, capture_output=True, cwd=str(ROOT),
        timeout=timeout,
    )


def test_cli_board_control_accepts_stdin_signal() -> None:
    r = _cli(["board-control", "--headless", "--stdin", "--steps", "40",
              "--duration", "1.0", "--quiet", "--json"],
             stdin_data=b"@MOVE 1\n")
    assert r.returncode == 0, r.stdout.decode() + r.stderr.decode()
    payload = json.loads(r.stdout.decode())
    assert payload["last_action"]["direction"][0] == pytest.approx(-1.0), \
        f"板子给的是左，仿真应当执行左：{payload['last_action']}"
    assert "board:stdin" in payload["input_source"], payload["input_source"]


def test_cli_board_control_does_not_stop_by_itself() -> None:
    """`--nostop`：不设步数上限时不因**回合结束**停下。

    构造：`--duration 0.5` 让回合在 total_steps 处 truncated，然后要求跑 3 倍步数。
    默认（会停）应当只跑完一个回合；`--nostop` 应当跑满要求的步数。
    """
    long_steps = "900"
    # 对照组：`run`（没有 --nostop）—— 回合一结束就停
    stopped = _cli(["run", "--input", "board", "--stdin", "--steps", long_steps,
                    "--duration", "0.5", "--json"],
                   stdin_data=b"@MOVE 0\n")
    assert stopped.returncode == 0, stopped.stdout.decode() + stopped.stderr.decode()
    n_stopped = json.loads(stopped.stdout.decode())["steps"]

    kept = _cli(["board-control", "--headless", "--stdin", "--steps", long_steps,
                 "--duration", "0.5", "--quiet", "--json"],
                stdin_data=b"@MOVE 0\n")
    assert kept.returncode == 0, kept.stdout.decode() + kept.stderr.decode()
    n_kept = json.loads(kept.stdout.decode())["steps"]

    assert n_stopped < int(long_steps), f"默认应当在回合结束处停下，却跑了 {n_stopped}"
    assert n_kept == int(long_steps), (
        f"`--nostop` 应当跑满 {long_steps} 步，却只跑了 {n_kept}（默认设置下是 {n_stopped}）")


def test_cli_board_control_needs_a_transport() -> None:
    """一个传输都不给就报错 —— 别默默用占位适配器让人以为"板子没信号"。"""
    r = _cli(["board-control", "--headless", "--steps", "5", "--quiet"])
    assert r.returncode != 0
    msg = (r.stdout + r.stderr).decode()
    assert "传输" in msg and ("--port" in msg or "--udp" in msg or "--stdin" in msg), msg


# ---------------------------------------------------------------------------
# 4) 信号必须**真的让角色动**（不只是"动作对象对了"）
# ---------------------------------------------------------------------------
def test_board_signal_actually_moves_the_player() -> None:
    """`@MOVE 1` 要真的把角色推向左 —— 这一段跨了 `action_to_codec_input` 与
    `World.step`，只断言"解析出的 Action 对"是盖不住它的。"""
    from bullet_sim.action.base import Action, action_to_codec_input
    from bullet_sim.entities.player import P_SPEED, P_X
    from bullet_sim.hardware_interface import HardwareInputSource, UdpBoardAdapter
    from bullet_sim.simulator.env import BulletHellEnv
    from bullet_sim.tests.helpers import blank_spec

    env = BulletHellEnv(blank_spec(), seed=0)
    ad = UdpBoardAdapter(port=0, hold_s=1.0)      # hold 长一点，别在测试中途回中
    src = HardwareInputSource(ad)
    ad.open()
    try:
        env.reset(seed=0)
        x0 = float(env.world.player[P_X])
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(b"@MOVE 1\n", ("127.0.0.1", ad.port))      # 左
        for _ in range(30):
            a = src.poll(env.spec.dt) or Action.zero()
            raw = action_to_codec_input(a, env.world.codec, float(env.world.player[P_SPEED]))
            env.step(raw)
        sk.close()
        x1 = float(env.world.player[P_X])
        assert x1 < x0 - 1.0, f"角色应当被推向左：x {x0} → {x1}"
    finally:
        ad.close()
        env.close()


def test_board_signal_up_moves_the_other_axis() -> None:
    """正交方向也要对：`@MOVE 3` 是上（+y 或 -y，取决于场地约定，只要**换了一个轴**）。"""
    from bullet_sim.action.base import Action, action_to_codec_input
    from bullet_sim.entities.player import P_SPEED, P_X, P_Y
    from bullet_sim.hardware_interface import HardwareInputSource, UdpBoardAdapter
    from bullet_sim.simulator.env import BulletHellEnv
    from bullet_sim.tests.helpers import blank_spec

    env = BulletHellEnv(blank_spec(), seed=0)
    ad = UdpBoardAdapter(port=0, hold_s=1.0)
    src = HardwareInputSource(ad)
    ad.open()
    try:
        env.reset(seed=0)
        p0 = (float(env.world.player[P_X]), float(env.world.player[P_Y]))
        sk = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sk.sendto(b"@MOVE 3\n", ("127.0.0.1", ad.port))      # 上
        for _ in range(30):
            a = src.poll(env.spec.dt) or Action.zero()
            env.step(action_to_codec_input(a, env.world.codec,
                                           float(env.world.player[P_SPEED])))
        sk.close()
        p1 = (float(env.world.player[P_X]), float(env.world.player[P_Y]))
        assert abs(p1[0] - p0[0]) < 1.0, f"上不该动 x：{p0} → {p1}"
        assert abs(p1[1] - p0[1]) > 1.0, f"上应当动 y：{p0} → {p1}"
    finally:
        ad.close()
        env.close()


def test_cli_missing_serial_port_explains_itself() -> None:
    """串口不在时**不能甩 traceback、也不能开一个没输入的空窗**。

    这条是用户实际踩到的：`--port /dev/ttyUSB0` 在没 attach 的 WSL 里直接崩，
    而崩在"开窗之前"，所以看起来像是"平台弹窗出不来"。现在要退 2 并说清楚
    设备在不在、以及两条不需要串口的路（--stdin / --udp）。
    """
    r = _cli(["board-control", "--port", "/dev/definitely-not-a-port"])
    assert r.returncode == 2, f"应当退 2，实际 {r.returncode}"
    msg = (r.stdout + r.stderr).decode()
    assert "Traceback" not in msg, f"不该甩 traceback：\n{msg}"
    assert "打不开串口" in msg, msg
    assert "--stdin" in msg and "--udp" in msg, f"要给两条替代路：\n{msg}"
    assert "usbipd" in msg, f"WSL 里要给 usbipd 的提示：\n{msg}"

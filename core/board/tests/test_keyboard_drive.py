"""keyboard_drive.py 的本地无硬件自测（R1d 第 3 关：接口级自测）。

只测"纯逻辑"三件事，不开口、不碰小车：
  1. 终端字节流 → 按键事件（普通字符 / \\x1b[A / \\x1bOA / 半截序列）
  2. 按下沿判定（终端 raw 模式没有 key-up，只能靠"续期窗口"）
  3. ←/→ → (编码器目标角, 角速度指令) 的符号关系（tsign 对调）

跑：pytest core/board/tests/test_keyboard_drive.py -q
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))

import keyboard_drive as kd  # noqa: E402


# ---------------- 1. 字节流解析 ----------------

def test_parse_arrow_csi():
    ev, rest = kd.parse_events(b"\x1b[A")
    assert ev == [("arrow", b"A")] and rest == b""


def test_parse_arrow_application_mode():
    ev, rest = kd.parse_events(b"\x1bOC")
    assert ev == [("arrow", b"C")] and rest == b""


def test_parse_mixed_and_partial():
    ev, rest = kd.parse_events(b"\x1b[Df\x1b[")
    assert ev == [("arrow", b"D"), ("ch", b"f")]
    assert rest == b"\x1b["          # 半截序列留到下一次


def test_parse_split_across_reads():
    ev1, rest = kd.parse_events(b"\x1b")
    assert ev1 == [] and rest == b"\x1b"
    ev2, rest = kd.parse_events(rest + b"[C")
    assert ev2 == [("arrow", b"C")]


def test_parse_unknown_escape_dropped():
    ev, rest = kd.parse_events(b"\x1b[Zq")
    assert ev == [("ch", b"q")] and rest == b""


def test_parse_parameterized_csi_swallowed():
    ev, rest = kd.parse_events(b"\x1b[1;5Cq")     # Ctrl+→ 之类
    assert ev == [("ch", b"q")] and rest == b""


def test_parse_incomplete_csi_waits():
    ev, rest = kd.parse_events(b"\x1b[1;5")
    assert ev == [] and rest == b"\x1b[1;5"


def test_parse_repeat_burst():
    ev, _ = kd.parse_events(b"\x1b[A" * 5)
    assert ev == [("arrow", b"A")] * 5


# ---------------- 2. 按下沿 ----------------

def test_one_tap_is_one_edge_even_with_autorepeat():
    """按下 → 0.5 s 首延迟 → 每 40 ms 重复：只算一次按下。"""
    e = kd.Edge(0.8)
    times = [100.0] + [100.5 + 0.04 * k for k in range(20)]
    news = [e.feed(t) for t in times]
    assert news[0] is True
    assert not any(news[1:]), "按住期间的自动重复不能触发第二次"


def test_two_taps_close_together_count_once():
    """两次真按下但间隔 < 窗口（终端无法区分）——按一次处理，这是有意的。"""
    e = kd.Edge(0.8)
    assert e.feed(10.0) is True
    assert e.feed(10.3) is False


def test_release_then_press_again_after_window():
    e = kd.Edge(0.8)
    assert e.feed(10.0) is True
    assert e.feed(10.9) is True          # 松开 > 0.8 s 后再按 = 新的一次


def test_rearm_swallows_backlog():
    """转向期间积压的重复事件，在转完之后不应该被当成新按下。"""
    e = kd.Edge(0.8)
    assert e.feed(20.0) is True
    e.rearm(24.0)                        # 转了 4 s，回来续期
    assert e.feed(24.1) is False


# ---------------- 3. 转向指令符号 ----------------

def test_turn_cmd_default_sign():
    delta, w = kd.turn_cmd(b"C", 90.0, 0.75, +1, 600.0)     # →
    assert (delta, w) == (+67.5, -600.0)
    delta, w = kd.turn_cmd(b"D", 90.0, 0.75, +1, 600.0)     # ←
    assert (delta, w) == (-67.5, +600.0)


def test_turn_cmd_flipped_sign():
    assert kd.turn_cmd(b"C", 90.0, 0.75, -1, 600.0) == (-67.5, +600.0)
    assert kd.turn_cmd(b"D", 90.0, 0.75, -1, 600.0) == (+67.5, -600.0)


def test_turn_cmd_angle_scaling():
    """地面 90° + scale 0.75 ⇒ 只让编码器走 67.5°。"""
    delta, _ = kd.turn_cmd(b"C", 120.0, 0.75, +1, 600.0)
    assert abs(delta) == 90.0


# ---------------- 4. 默认左右（2026-10-29 实测锁定） ----------------

def test_default_sign_makes_right_key_turn_right():
    """固件 moto1 物理上接的是右轮：w>0 = 车往右（顺时针）。
    所以按 → 必须发 w>0。这条锁的是实车验证过的默认值，别再翻回去。"""
    assert kd.TURN_SIGN_DEFAULT == -1
    delta_r, w_r = kd.turn_cmd(b"C", 90.0, 0.75, kd.TURN_SIGN_DEFAULT, 600.0)   # →
    delta_l, w_l = kd.turn_cmd(b"D", 90.0, 0.75, kd.TURN_SIGN_DEFAULT, 600.0)   # ←
    assert w_r > 0 and w_l < 0
    assert delta_r < 0 and delta_l > 0     # 编码器 θ：w>0 时 θ 减小（实测）

"""``core/board/geometry_frame.cpp`` ↔ ``core/standard/vision/vision_frame.py`` 对账。

线协议有**三个独立实现**：

======================  ================================  ==========================
角色                     位置                              怎么验
======================  ================================  ==========================
真值（Python）           `standard/vision/vision_frame.py`  —（它就是标准）
PL 生产者（Verilog）     `core/fpga/rtl/vision_top.v`       620 帧逐位协同仿真
PS 消费者（C++）         `core/board/geometry_frame.cpp`    **本文件**
======================  ================================  ==========================

Verilog 那一侧已经有逐位对账守着了；这一侧在此之前**只有读过源码**。
本文件把它变成可执行的检查：Python 的 `encode_vision_frame()` 产出字节，
喂给 C++ 解码器，再把解出来的每个字段和 Python 的 `decode_vision_frame()` 比。

覆盖：
* 各种规模（0/1/多障碍、0/多缺口）的**正常帧**逐字段一致；
* Q6 反定标（少除 64 是最容易犯又最不响的错）；
* **坏帧必须被拒**：magic 错 / 版本错 / 长度不符 / CRC 错 / 被截断。
"""

from __future__ import annotations

import pathlib
import shutil
import struct
import subprocess
import sys

import pytest

_HERE = pathlib.Path(__file__).resolve().parent
_BOARD = _HERE.parent
_ROOT = _BOARD.parents[1]
_BUILD = _ROOT / "data" / "board" / "geometry_frame"


def _gxx() -> str | None:
    return shutil.which("g++") or shutil.which("c++")


@pytest.fixture(scope="module")
def dumper() -> pathlib.Path:
    gxx = _gxx()
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过几何帧解码的对账")
    _BUILD.mkdir(parents=True, exist_ok=True)
    exe = _BUILD / "geometry_frame_dump"
    srcs = [_BOARD / "geometry_frame.cpp", _HERE / "geometry_frame_dump.cpp"]
    if not exe.exists() or any(p.stat().st_mtime > exe.stat().st_mtime
                               for p in srcs + [_BOARD / "geometry_frame.hpp"]):
        r = subprocess.run(
            [gxx, "-std=c++17", "-O2", "-Wall", "-Wextra", "-I", str(_BOARD),
             "-o", str(exe), *[str(p) for p in srcs]],
            capture_output=True, text=True)
        if r.returncode != 0 or "warning:" in r.stderr:
            pytest.fail(f"C++ 几何帧解码编译失败/有警告：\n{r.stdout}\n{r.stderr}")
    return exe


# ---------------------------------------------------------------------------
# 造帧
# ---------------------------------------------------------------------------

def _sys_path() -> None:
    for p in (str(_ROOT / "core"), str(_ROOT / "core" / "cpu")):
        if p not in sys.path:
            sys.path.insert(0, p)


def _make_frame(*, seq=7, stamp_ns=123456789, n_obs=2, n_gaps=1,
                player=True, units_m=True):
    _sys_path()
    from standard.vision.vision_frame import (VisionFrame, encode_vision_frame,
                                              VISION_Q)
    from standard.obstacle.obstacle import Obstacle, ObstacleShape, ObstacleType
    # ⚠ `Units` 在 vision_output 里，**不在** obstacle 里；
    #   `PlayerState` 在 standard.player.player 里。这两处早先都写错了。
    from standard.vision.vision_output import GapObs, Units
    from standard.player.player import PlayerState

    def q(v: float) -> float:
        # 协议是 Q6 定标：编码前先对齐到 1/64 的整数倍，
        # 否则 Python 的 `_q()` 会四舍五入，而 C++ 解出来的值就和这里的输入差半个 LSB。
        return round(v * VISION_Q) / VISION_Q

    obs = []
    for i in range(n_obs):
        obs.append(Obstacle(
            id=i + 1, shape=ObstacleShape.RECT, type=ObstacleType.WALL_WITH_GAP,
            type_id=2,
            x=q(100.0 + 37.5 * i), y=q(200.0 + 11.25 * i),
            half_w=q(50.0 + i), half_h=q(8.0),
            rotation=q(0.125 * i)))
    gaps = []
    for i in range(n_gaps):
        # ⚠ `blockers` 必须指回**本帧真实存在**的障碍 id —— `VisionFrame.validate()`
        #   会逐条核对，指错了直接抛。想当然地写 `(1+2i, 2+2i)` 在 n_obs=5、n_gaps=3
        #   时就会引用到 id=6（不存在）而被拒。
        ba = (2 * i) % n_obs + 1
        bb = (2 * i + 1) % n_obs + 1
        gaps.append(GapObs(id=i + 1, center=(q(320.0 + i), q(200.0 + i)),
                           width=q(60.0), axis=q(0.0),
                           blockers=(ba, bb),
                           blocked_by_type=ObstacleType.WALL_WITH_GAP.value))

    frame = VisionFrame(
        seq=seq, stamp=stamp_ns / 1e9,
        units=Units.M if units_m else Units.PX,
        image_width=640, image_height=480,
        player=PlayerState(x=q(320.0), y=q(72.0), radius=q(10.0),
                           stamp=stamp_ns / 1e9, seq=seq) if player else None,
        obstacles=obs, gaps=gaps, target=None)
    return frame, encode_vision_frame(frame)


def _feed(dumper: pathlib.Path, frames: list[bytes]) -> list[str]:
    """把帧按 `<u32 长度><字节>` 喂进去，返回输出行。"""
    blob = b"".join(struct.pack("<I", len(b)) + b for b in frames)
    r = subprocess.run([str(dumper)], input=blob, capture_output=True)
    assert r.returncode == 0, f"dumper 退出 {r.returncode}: {r.stderr!r}"
    return r.stdout.decode().splitlines()


def _close(a: float, b: float, tag: str) -> None:
    # 两端都是同一次 Q6 量化后的值，应当**逐位**相同；留一点余量只为容错
    assert abs(a - b) <= 1e-12 * max(1.0, abs(a), abs(b)), f"{tag}: {a!r} != {b!r}"


# ---------------------------------------------------------------------------
# 正向：正常帧逐字段一致
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("n_obs,n_gaps", [(0, 0), (1, 0), (2, 1), (5, 3), (32, 16)])
def test_cpp_decoder_matches_python_on_good_frames(dumper, n_obs, n_gaps):
    _sys_path()
    from standard.vision.vision_frame import decode_vision_frame
    from standard.obstacle.obstacle import ObstacleShape

    pyf, raw = _make_frame(n_obs=n_obs, n_gaps=n_gaps)
    lines = _feed(dumper, [raw])
    assert lines and lines[0].startswith("FRAME "), lines

    # ⚠ 列号要按 `geometry_frame_dump.cpp` 的 printf **数一遍**，不要凭印象：
    #   FRAME seq stamp_ns flags img_w img_h player_present px py pr p_occluded n_obs n_gaps
    #   -> 下标 0     1    2        3     4     5     6              7  8  9  10         11    12
    f = lines[0].split()
    assert f[0] == "FRAME"
    assert int(f[1]) == pyf.seq
    assert int(f[2]) == int(round(pyf.stamp * 1e9))
    assert bool(int(f[3]) & 1) == (pyf.units.value == "m"), "units 位（kFrameUnitsM）"
    assert int(f[4]) == pyf.image_width
    assert int(f[5]) == pyf.image_height
    assert int(f[6]) == (1 if pyf.player is not None else 0)
    if pyf.player is not None:
        _close(float(f[7]), pyf.player.x, "player.x")
        _close(float(f[8]), pyf.player.y, "player.y")
        _close(float(f[9]), pyf.player.radius, "player.radius")
    assert int(f[11]) == len(pyf.obstacles)
    assert int(f[12]) == len(pyf.gaps)

    obs_lines = [l for l in lines if l.startswith("OBS ")]
    gap_lines = [l for l in lines if l.startswith("GAP ")]
    assert len(obs_lines) == len(pyf.obstacles)
    assert len(gap_lines) == len(pyf.gaps)

    for line, o in zip(obs_lines, pyf.obstacles):
        v = line.split()
        assert int(v[1]) == o.id
        assert int(v[2]) == (1 if o.shape == ObstacleShape.RECT else 0)
        assert int(v[3]) == o.type_id
        _close(float(v[4]), o.x, "obs.x")
        _close(float(v[5]), o.y, "obs.y")
        _close(float(v[7]), o.half_w, "obs.half_w")
        _close(float(v[8]), o.half_h, "obs.half_h")
        _close(float(v[9]), o.rotation, "obs.rotation")

    for line, g in zip(gap_lines, pyf.gaps):
        v = line.split()
        assert int(v[1]) == g.id
        _close(float(v[2]), g.center[0], "gap.cx")
        _close(float(v[3]), g.center[1], "gap.cy")
        _close(float(v[4]), g.width, "gap.width")
        assert (int(v[6]), int(v[7])) == g.blockers
        assert bool(int(v[9])) == bool(g.reliable)


def _repatch(raw: bytes, off: int, val: int) -> bytes:
    """改一个字节并**重算 CRC**。

    ⚠ 这一步是必须的。直接改字节会把 CRC 也弄坏，于是**CRC 那一关先把它拦下来**，
    版本关/长度关根本没被执行到 —— 我实测过：把版本检查整段删掉，
    "不重算 CRC"的坏帧测试照样全绿。只有重算 CRC，才能让每一关被**单独**考到。
    """
    import zlib
    b = bytearray(raw)
    b[off] = val
    b[-4:] = struct.pack("<I", zlib.crc32(bytes(b[:-4])) & 0xFFFFFFFF)
    return bytes(b)


def test_cpp_decoder_rejects_bad_frames(dumper):
    """**坏帧必须被拒**，而且必须是**对的那一关**拦下来的。

    每条都断言错误说明里的关键词，而不是只看"有没有报错" —— 否则
    "版本检查被删掉、CRC 顺手拦住"会伪装成通过（实测就是这样）。
    """
    raw = _make_frame()[1]

    cases = [
        ("magic 错", _repatch(raw, 0, raw[0] ^ 0xFF), "magic"),
        ("版本错", _repatch(raw, 4, 99), "版本"),
        ("n_obs 与长度不符", _repatch(raw, 12, 0xFF), "长度"),
        ("被截断", raw[:-1], "长度"),
        ("CRC 错", bytes(bytearray(raw[:-1]) + bytes([raw[-1] ^ 0xFF])), "CRC"),
    ]

    lines = _feed(dumper, [b for _, b, _ in cases])
    errs = [l for l in lines if l.startswith("ERR ")]
    assert not any(l.startswith("FRAME ") for l in lines), f"坏帧被放行了：{lines}"
    assert len(errs) == len(cases), f"应当 {len(cases)} 条都被拒，实际 {len(errs)}：{lines}"
    for (name, _, needle), line in zip(cases, errs):
        assert needle in line, f"{name}: 由错的那一关拦下了 —— {line!r} 里没有 {needle!r}"


def test_cpp_decoder_accepts_frames_after_a_bad_one(dumper):
    """坏帧不能把流带偏：后面那帧仍然要解对。"""
    pyf, good = _make_frame(seq=1)
    _, good2 = _make_frame(seq=2)
    bad = bytearray(good)
    bad[-1] ^= 0xFF

    lines = _feed(dumper, [good, bytes(bad), good2])
    frames = [l for l in lines if l.startswith("FRAME ")]
    assert len(frames) == 2, lines
    assert int(frames[0].split()[1]) == 1
    assert int(frames[1].split()[1]) == 2
    assert len([l for l in lines if l.startswith("ERR ")]) == 1


def test_cpp_crc32_matches_python_zlib():
    """CRC32 必须是 zlib 那一套（多项式 0xEDB88320），不是别的变体。

    标准测试向量 ``crc32(b"123456789") == 0xCBF43926``。
    这里不编 C++，直接对 Python 的 zlib 比 —— C++ 侧那条由上面的长帧覆盖
    （长度/CRC 都错时才会走到 CRC 那一关，见 ``bad_crc``）。
    """
    import zlib
    assert zlib.crc32(b"123456789") & 0xFFFFFFFF == 0xCBF43926
    _, raw = _make_frame()
    # 帧自带的 CRC 与 zlib 一致 —— 说明 Python 编码器用的就是这一套
    assert struct.unpack_from("<I", raw, len(raw) - 4)[0] == \
        zlib.crc32(raw[:-4]) & 0xFFFFFFFF

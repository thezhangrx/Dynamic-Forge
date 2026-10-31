"""``core/vision/motion.py`` ↔ ``core/cpu/cpp/motion.cpp`` 的逐帧交叉验证。

这是"板子上那面移动墙到底动没动"的最后一道保险：Python 侧是算法真值，
板子跑的是 C++ 移植。两边**必须**给出同一套关联、同一套速度。

验收口径（同 ``core/cpu/cpp/README.md``）：

==================================  ==========================================
离散量（id / 命中 / 丢失 / 关联下标）  **逐位一致** —— 全是整数逻辑
``+ - * /`` 的浮点量                 **逐位一致**（IEEE-754 要求正确舍入）
``hypot``（只用于门限判据）          相对误差 < 1e-9 —— 两侧算法不同，末位可差 1 ulp
==================================  ==========================================

**唯一**有意的行为差异：``stamp`` 不单调时 Python 抛 ``ValueError``，C++ 返回
``false`` 且不动状态（板子上是实时环，不能因为一个坏时间戳崩掉）。这条也在下面
单独钉住 —— 差异必须**写在测试里**，不能只是"我们知道"。
"""

from __future__ import annotations

import math
import pathlib
import random
import shutil
import subprocess

import pytest

from vision.motion import Tracker

_CPP = pathlib.Path(__file__).resolve().parents[1] / "cpp"
_ROOT = pathlib.Path(__file__).resolve().parents[3]
#: 编译产物丢在 data/（已 gitignore）。**不要**跟源码放一起。
_BUILD = _ROOT / "data" / "cpu" / "cpp"

#: -ffp-contract=off 是**必须**的：α-β 更新里有 `v += (beta/dt) * r` 这种乘加，
#: GCC 默认会收缩成 FMA（只舍入一次），与 Python 分开算的结果不同。
_CXXFLAGS = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-ffp-contract=off"]


@pytest.fixture(scope="module")
def cli() -> pathlib.Path:
    gxx = shutil.which("g++") or shutil.which("c++")
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过 C++ 速度跟踪移植的交叉验证")
    _BUILD.mkdir(parents=True, exist_ok=True)
    exe = _BUILD / "motion_cli"
    srcs = [str(_CPP / "motion.cpp"), str(_CPP / "motion_cli.cpp")]
    deps = srcs + [str(_CPP / "motion.hpp")]
    if not exe.exists() or any(pathlib.Path(s).stat().st_mtime > exe.stat().st_mtime
                               for s in deps):
        r = subprocess.run([gxx, *_CXXFLAGS, "-I", str(_CPP), "-o", str(exe), *srcs],
                           capture_output=True, text=True)
        # 警告也算失败：这一份是要上板的，不是实验代码
        if r.returncode != 0 or "warning:" in r.stderr:
            pytest.fail(f"C++ 速度跟踪移植编译失败/有警告：\n{r.stdout}\n{r.stderr}")
    return exe


# ---------------------------------------------------------------------------
# 跑两侧
# ---------------------------------------------------------------------------
Frame = tuple[float, list[tuple[float, float]]]          # (stamp, 检测点)


def _text(frames: list[Frame]) -> str:
    lines = []
    for stamp, dets in frames:
        pts = " ".join("%.17g %.17g" % p for p in dets)
        lines.append("FRAME %.17g %d %s" % (stamp, len(dets), pts))
    return "\n".join(lines) + "\n"


def _run_cpp(cli: pathlib.Path, frames: list[Frame]) -> list[dict]:
    r = subprocess.run([str(cli)], input=_text(frames),
                       capture_output=True, text=True)
    assert r.returncode == 0, f"C++ 侧退出 {r.returncode}：{r.stderr}"
    out: list[dict] = []
    cur: dict | None = None
    for line in r.stdout.splitlines():
        f = line.split()
        if f[0] == "ERR":
            out.append({"err": float(f[1])})
            continue
        if f[0] == "TRK":
            cur = {"n": int(f[1]), "tracks": [], "att": None}
            out.append(cur)
            continue
        if f[0] == "T":
            assert cur is not None, "T 行出现在 TRK 之前"
            cur["tracks"].append(dict(
                id=int(f[1]), x=float(f[2]), y=float(f[3]),
                vx=float(f[4]), vy=float(f[5]), stamp=float(f[6]),
                hits=int(f[7]), misses=int(f[8]), age=int(f[9]), assign=int(f[10])))
            continue
        if f[0] == "ATT":
            assert cur is not None
            n = int(f[1])
            vals = [float(v) for v in f[2:]]
            assert len(vals) == 2 * n, f"ATT 长度不对：{line!r}"
            cur["att"] = [(vals[2 * i], vals[2 * i + 1]) for i in range(n)]
            continue
        raise AssertionError(f"意外的输出行：{line!r}")
    return out


def _run_python(frames: list[Frame]) -> list[dict]:
    """每局一个新的 Tracker —— 与 C++ 侧"一个进程一局"对齐。"""
    tr = Tracker()
    out: list[dict] = []
    for stamp, dets in frames:
        try:
            tracks = tr.update(dets, stamp)
        except ValueError as e:
            out.append({"err": str(e)})
            continue
        asg = tr.last_assignment
        att = [(0.0, 0.0)] * len(dets)
        for t, j in zip(tracks, asg):
            if j is not None:
                att[j] = (t.vx, t.vy)
        out.append({"n": len(tracks), "att": att, "tracks": [
            dict(id=t.id, x=t.x, y=t.y, vx=t.vx, vy=t.vy, stamp=t.stamp,
                 hits=t.hits, misses=t.misses, age=t.age,
                 assign=(-1 if j is None else j))
            for t, j in zip(tracks, asg)]})
    return out


def _cmp(cpp: list[dict], py: list[dict], *, label: str) -> None:
    assert len(cpp) == len(py), f"{label}：帧数不一致 {len(cpp)} vs {len(py)}"
    for k, (c, p) in enumerate(zip(cpp, py)):
        if "err" in p or "err" in c:
            assert "err" in c and "err" in p, f"{label} 第 {k} 帧：只有一侧报错"
            continue
        assert c["n"] == p["n"], f"{label} 第 {k} 帧轨迹数不同：{c['n']} vs {p['n']}"
        assert len(c["tracks"]) == c["n"], f"{label} 第 {k} 帧 TRK 行数与实际不符"
        for i, (tc, tp) in enumerate(zip(c["tracks"], p["tracks"])):
            # 离散量：必须逐位一致
            for key in ("id", "hits", "misses", "age", "assign"):
                assert tc[key] == tp[key], (
                    f"{label} 第 {k} 帧第 {i} 条轨迹的 {key} 不一致："
                    f"{tc[key]} vs {tp[key]}")
            # 浮点量：相对误差 < 1e-9（hypot 只影响关联，浮点链本身是 +-*/）
            for key in ("x", "y", "vx", "vy", "stamp"):
                a, b = float(tc[key]), float(tp[key])
                scale = max(1.0, abs(a), abs(b))
                assert abs(a - b) / scale < 1e-9, (
                    f"{label} 第 {k} 帧第 {i} 条轨迹的 {key} 不一致：{a!r} vs {b!r}")
        for j, ((vc, wc), (vp, wp)) in enumerate(zip(c["att"], p["att"])):
            for a, b in ((vc, vp), (wc, wp)):
                scale = max(1.0, abs(a), abs(b))
                assert abs(a - b) / scale < 1e-9, (
                    f"{label} 第 {k} 帧第 {j} 个检测贴的速度不一致：{a!r} vs {b!r}")


# ---------------------------------------------------------------------------
# 用例
# ---------------------------------------------------------------------------
def _uniform(vx: float, vy: float, *, fps: float = 30.0, n: int = 40,
             start: tuple[float, float] = (0.0, 0.0)) -> list[Frame]:
    return [(k / fps, [(start[0] + vx * k / fps, start[1] + vy * k / fps)])
            for k in range(n)]


def test_uniform_single_target_matches(cli: pathlib.Path) -> None:
    frames = _uniform(0.30, -0.10)
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="匀速单目标")


def test_two_crossing_targets_match(cli: pathlib.Path) -> None:
    """两个目标对穿：关联一旦配错，速度就会互换 —— 离散量立刻对不上。"""
    frames = []
    for k in range(50):
        t = k / 30.0
        frames.append((t, [(0.30 * t, 0.0), (10.0 - 0.20 * t, 0.0)]))
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="对穿双目标")


def test_detection_order_shuffled_matches(cli: pathlib.Path) -> None:
    """帧内顺序每帧翻：帧里的 id 是当帧序号，关联只能靠位置。"""
    frames = []
    for k in range(40):
        t = k / 30.0
        a = (0.30 * t, 0.0)
        b = (10.0 - 0.20 * t, 0.0)
        frames.append((t, [a, b] if k % 2 == 0 else [b, a]))
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="顺序翻转")


def test_misses_new_targets_and_reappear_match(cli: pathlib.Path) -> None:
    """丢帧、外推、新目标、重新出现 —— 计数与 id 分配必须同步。"""
    frames: list[Frame] = []
    for k in range(30):
        t = k / 30.0
        dets = [(0.30 * t, 0.0)]
        if k % 7 != 3:                       # 每隔 7 帧丢一次
            dets.append((5.0, 0.20 * t))
        if k in (11, 12):                    # 突然多一个目标
            dets.append((8.0, 8.0))
        frames.append((t, dets))
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="丢帧/新目标")


def test_jittery_stamps_match(cli: pathlib.Path) -> None:
    """时间戳抖动（真实采集就是这样）：dt 不同 → 速度估计也不同，两侧要同步。"""
    rng = random.Random(20261015)
    frames: list[Frame] = []
    t = 0.0
    for k in range(60):
        t += rng.uniform(0.02, 0.06)
        frames.append((t, [(0.30 * t, -0.10 * t), (6.0, 3.0 - 0.05 * t)]))
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="时戳抖动")


def test_large_gap_resets_velocity_on_both_sides(cli: pathlib.Path) -> None:
    """间隔 > max_dt：两侧都要"落位 + 速度置零"，不能跨大间隔乱推。"""
    frames = [(0.0, [(0.0, 0.0)]), (0.02, [(0.1, 0.0)]),
              (2.0, [(9.0, 0.0)]),          # 大间隔
              (2.02, [(9.1, 0.0)]), (2.04, [(9.2, 0.0)])]
    cpp, py = _run_cpp(cli, frames), _run_python(frames)
    _cmp(cpp, py, label="大间隔")
    assert cpp[2]["tracks"][0]["vx"] == 0.0, "大间隔后速度必须先归零"


def test_zero_dt_matches(cli: pathlib.Path) -> None:
    """两帧同一个时间戳（dt == 0）：走"落位"分支，不是除以 0。"""
    frames = [(1.0, [(0.0, 0.0)]), (1.0, [(0.5, 0.0)]), (1.02, [(0.6, 0.0)])]
    _cmp(_run_cpp(cli, frames), _run_python(frames), label="dt=0")


def test_assignments_are_a_permutation_into_detections(cli: pathlib.Path) -> None:
    """每条被匹配的轨迹对应**不同**的检测：不该出现两条轨迹抢同一个检测。"""
    frames = [(k / 30.0, [(1.0 * i + 0.01 * k, 0.0) for i in range(4)])
              for k in range(20)]
    out = _run_cpp(cli, frames)
    for k, fr in enumerate(out):
        got = [t["assign"] for t in fr["tracks"] if t["assign"] >= 0]
        assert len(got) == len(set(got)), f"第 {k} 帧有轨迹抢了同一个检测：{got}"


def test_non_monotonic_stamp_differs_on_purpose(cli: pathlib.Path) -> None:
    """**有意的**差异：Python 抛异常（"宁可炸"），C++ 返回 false（板子不能崩）。

    差异本身不是问题，问题是差异没被写下来。所以这里同时钉住两边的行为。
    """
    frames = [(1.0, [(0.0, 0.0)]), (0.5, [(0.1, 0.0)])]
    cpp, py = _run_cpp(cli, frames), _run_python(frames)
    assert "err" in py[1], "Python 侧必须对倒流的 stamp 抛异常"
    assert "err" in cpp[1], "C++ 侧必须报告这一帧没算"
    # 而且 C++ 侧**状态一点没动**：坏帧不改基准，所以基准仍是 1.0 ——
    # 接着喂 0.9 仍然算倒流（继续报错），必须喂 > 1.0 才能接上。
    frames_back = frames + [(0.9, [(0.2, 0.0)])]
    cpp_back = _run_cpp(cli, frames_back)
    assert "err" in cpp_back[2], "坏帧不改基准：0.9 < 1.0，仍然该报错"
    frames_ok = frames + [(1.1, [(0.2, 0.0)])]
    cpp_ok = _run_cpp(cli, frames_ok)
    assert "err" not in cpp_ok[2] and cpp_ok[2]["n"] == 1, \
        "喂回 > 基准的时间戳应当能接上"


def test_position_jump_matches(cli: pathlib.Path) -> None:
    """位置跳变（真实素材里带合并导致的段中心横移）：两侧都要"落位但不给速度"。"""
    frames: list[Frame] = [(k / 30.0, [(100.0, 50.0)]) for k in range(6)]
    frames.append((6 / 30.0, [(76.0, 50.0)]))            # 单帧横移 24
    for k in range(7, 12):
        frames.append((k / 30.0, [(76.0, 50.0)]))
    cpp, py = _run_cpp(cli, frames), _run_python(frames)
    _cmp(cpp, py, label="位置跳变")
    assert cpp[6]["tracks"][0]["vx"] == 0.0, "跳变帧不该产生速度"
    assert cpp[6]["tracks"][0]["x"] == 76.0, "位置仍要落位"
    assert cpp[11]["tracks"][0]["vx"] == 0.0, "后续帧也不该把速度带出来"


def test_realistic_motion_below_threshold_matches(cli: pathlib.Path) -> None:
    """阈值内的真运动两侧都要照常跟踪（守卫别把真运动也挡了）。"""
    frames: list[Frame] = [(k / 30.0, [(2.0 * k, 0.3 * k)]) for k in range(40)]
    cpp, py = _run_cpp(cli, frames), _run_python(frames)
    _cmp(cpp, py, label="阈值内匀速")
    assert cpp[-1]["tracks"][0]["vx"] == pytest.approx(60.0, abs=2.0)


def test_fast_target_and_jump_guard_match(cli: pathlib.Path) -> None:
    """守卫的两条边界都要两侧一致：真运动要拿到速度，单帧跳变不许改写已有速度。"""
    # ① 10 单位/帧 的真运动（残差一直大于 max_residual=8）
    fast = [(k * 0.05, [(10.0 * k, 0.0)]) for k in range(20)]
    cpp, py = _run_cpp(cli, fast), _run_python(fast)
    _cmp(cpp, py, label="快速真运动")
    assert cpp[-1]["tracks"][0]["vx"] > 0, "真运动必须拿到速度"

    # ② 先建立速度，再来一帧 24 的跳变
    frames = [(k * 0.05, [(2.0 * k, 0.0)]) for k in range(20)]
    frames.append((20 * 0.05, [(2.0 * 20 + 24.0, 0.0)]))
    cpp2, py2 = _run_cpp(cli, frames), _run_python(frames)
    _cmp(cpp2, py2, label="已有速度时遇跳变")
    assert cpp2[-1]["tracks"][0]["x"] == 2.0 * 20 + 24.0, "位置要落位"
    assert cpp2[-1]["tracks"][0]["vx"] == pytest.approx(cpp2[-2]["tracks"][0]["vx"]), \
        "单帧跳变不该改写已有速度"


def test_fast_start_matches(cli: pathlib.Path) -> None:
    """**出现时就在动**的目标：两侧的快速起步必须给出同一串速度。"""
    frames = [(k / 30.0, [(60.0 * k / 30.0, 0.0)]) for k in range(40)]
    cpp, py = _run_cpp(cli, frames), _run_python(frames)
    _cmp(cpp, py, label="出现即在动（快速起步）")
    assert cpp[-1]["tracks"][0]["vx"] == pytest.approx(60.0, abs=2.0), \
        "跑道末帧速度应当基本追上真值"

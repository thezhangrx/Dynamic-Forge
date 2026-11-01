"""``gap_avoid.py`` ↔ ``core/cpu/cpp/gap_avoid.cpp`` 的逐帧交叉验证。

验收口径（见 ``core/cpu/docs/CPP_PORT.md`` 开头）：

==============  ==========================================================
``tier``         **逐位一致** —— 全是比较与整数逻辑，没有理由不同
浮点量          相对误差 < 1e-9 —— ``hypot``/``atan2``/``cos``/``sin``
                不要求正确舍入，两侧 glibc/编译器的末位可以差 1 ulp
==============  ==========================================================

``wx/wy`` 为什么不能要求逐位：它们 = ``u_mag*sgn_lat*û + w_frac*fwd``，中间
过了一次 ``sqrt``（``w_cap``）。``sqrt`` 本身是正确舍入的，但 ``atan2`` 才
是分水岭 —— 只要开了 ``max_turn_rate`` 就会走 ``atan2/cos/sin``。

**这个测试是有牙齿的**：C++ 侧那三个陷阱（``wrap_pi`` 的 fmod 符号、
``sort`` 的稳定顺序、``round`` 的五成双）每一个都能让它在某帧上翻脸。
回退成"朴素翻译"当场变红，见 ``test_naive_translations_are_actually_wrong``。
"""

from __future__ import annotations

import math
import pathlib
import shutil
import subprocess
import sys

import pytest

# conftest 已经把 core/ 与 core/cpu/ 放进 sys.path
from cpu.gap_avoid import (EPS, AlgoConfig, Decision, GapMemory, Rect, WorldView,
                           plan, to_command)

_CPP = pathlib.Path(__file__).resolve().parents[1] / "cpp"
_ROOT = pathlib.Path(__file__).resolve().parents[3]

#: 编译产物丢在这里（``data/`` 已 gitignore）。**不要**跟源码放一起。
_BUILD = _ROOT / "data" / "cpu" / "cpp"

#: -ffp-contract=off 是**必须**的：GCC 默认会把 `a*b+c` 收缩成一条 fma，
#: 那会改变舍入，`tier` 就会在边界上分家。见 gap_avoid.cpp 文件头。
_CXXFLAGS = ["-std=c++17", "-O2", "-Wall", "-Wextra", "-ffp-contract=off"]


def _gxx() -> str | None:
    return shutil.which("g++") or shutil.which("c++")


@pytest.fixture(scope="module")
def cli() -> pathlib.Path:
    gxx = _gxx()
    if gxx is None:
        pytest.skip("没有 g++/c++，跳过 C++ 决策移植的交叉验证")
    _BUILD.mkdir(parents=True, exist_ok=True)
    exe = _BUILD / "gap_avoid_cli"
    srcs = [str(_CPP / "gap_avoid.cpp"), str(_CPP / "gap_avoid_cli.cpp")]
    if not exe.exists() or any(pathlib.Path(s).stat().st_mtime > exe.stat().st_mtime
                               for s in srcs + [str(_CPP / "gap_avoid.hpp")]):
        r = subprocess.run([gxx, *_CXXFLAGS, "-I", str(_CPP), "-o", str(exe), *srcs],
                           capture_output=True, text=True)
        # 警告也算失败：这一份是要上板的，不是实验代码
        if r.returncode != 0 or "warning:" in r.stderr:
            pytest.fail(f"C++ 决策移植编译失败/有警告：\n{r.stdout}\n{r.stderr}")
    return exe


# ===========================================================================
# 轨迹格式（与 gap_avoid_cli.cpp 里的注释一一对应）
# ===========================================================================

def _fmt_cfg(cfg: AlgoConfig) -> str:
    return ("CFG %.17g %.17g %.17g %.17g %d %.17g %d %.17g %.17g %d %.17g %.17g %.17g %.17g"
            % (cfg.safety_margin, cfg.max_window, cfg.align_frac, cfg.dead_frac,
               1 if cfg.allow_cross else 0, cfg.aim_lookahead,
               1 if cfg.front_back else 0, cfg.fwd_speed, cfg.hold_clearance,
               1 if cfg.forward is not None else 0,
               *(cfg.forward if cfg.forward is not None else (0.0, 0.0)),
               cfg.max_turn_rate, cfg.edge_margin))


def _fmt_frame(view: WorldView, horizon: float) -> str:
    p = [float(v) for v in view.player]
    assert len(p) == 7, "player 必须是 7 元组"
    lines = ["FRAME %.17g %d %.17g %.17g %.17g %d %s"
             % (view.dt, view.step_index, view.field_w, view.field_h, horizon,
                len(view.rects), " ".join("%.17g" % v for v in p))]
    for r in view.rects:
        lines.append("RECT %d %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g"
                     % (r.ent_id, r.x, r.y, r.vx, r.vy, r.half_w, r.half_h,
                        r.rotation, r.angular_velocity))
    return "\n".join(lines)


def _run_trace(cli: pathlib.Path, cfg: AlgoConfig,
               frames: list[tuple[WorldView, float]]) -> list[tuple]:
    """把轨迹喂给 C++，返回解析后的 DEC 行。"""
    text = _fmt_cfg(cfg) + "\n" + "\n".join(_fmt_frame(v, h) for v, h in frames) + "\n"
    r = subprocess.run([str(cli)], input=text, capture_output=True, text=True)
    assert r.returncode == 0, f"C++ 侧退出 {r.returncode}：{r.stderr}"
    out = []
    for line in r.stdout.splitlines():
        f = line.split()
        assert f[0] == "DEC", f"意外的输出行：{line!r}"
        out.append((float(f[1]), float(f[2]), f[3],
                    *(float(x) for x in f[4:])))
    return out


def _run_python(cfg: AlgoConfig, frames: list[tuple[WorldView, float]]) -> list[Decision]:
    """同一批输入喂给 Python。**每局一个新的 GapMemory** —— 与 C++ 侧一个进程一局对齐。"""
    mem = GapMemory()
    return [plan(v, mem, cfg, h) for v, h in frames]


# ===========================================================================
# 比较
# ===========================================================================

def _close(a: float, b: float, *, tag: str) -> None:
    if math.isnan(a) and math.isnan(b):
        return
    if math.isinf(a) or math.isinf(b):
        assert a == b, f"{tag}: {a} != {b}"
        return
    scale = max(abs(a), abs(b), 1.0)
    assert abs(a - b) <= 1e-9 * scale, f"{tag}: {a!r} != {b!r}（相对差 {abs(a-b)/scale:.3g}）"


def _compare(cfg: AlgoConfig, frames: list[tuple[WorldView, float]], cli: pathlib.Path,
             *, tag: str) -> dict:
    cpp = _run_trace(cli, cfg, frames)
    py = _run_python(cfg, frames)
    assert len(cpp) == len(py), f"{tag}: C++ 给了 {len(cpp)} 帧，Python {len(py)} 帧"

    tiers: dict[str, int] = {}
    # ⚠ 顺序必须与 gap_avoid_cli.cpp 里那条 printf **逐字对应**：
    #   wx wy tier t_arr clearance gap usable_half e_lo e_hi aim_lat s_hat n_rects wall_band
    #   n_rects 在 wall_band **前面**。早先按 Python Decision 的字段顺序排，于是
    #   拿 n_rects 去和 wall_band 比，报出 "2.0 != -476.0" 这种莫名其妙的差异。
    fields = ("wx", "wy", "t_arr", "clearance", "gap", "usable_half",
              "e_lo", "e_hi", "aim_lat", "s_hat", "n_rects", "wall_band")
    for i, (c, p) in enumerate(zip(cpp, py)):
        c_wx, c_wy, c_tier = c[0], c[1], c[2]
        c_vals = (c_wx, c_wy) + c[3:]
        assert c_tier == p.tier, (
            f"{tag} 第 {i} 帧 tier 不同：Python={p.tier} C++={c_tier}\n"
            f"    Python reason={p.reason!r}")
        tiers[p.tier] = tiers.get(p.tier, 0) + 1
        p_vals = (p.wx, p.wy, p.t_arr, p.clearance, p.gap, p.usable_half,
                  p.e_lo, p.e_hi, p.aim_lat, p.s_hat, float(p.n_rects), p.wall_band)
        assert len(c_vals) == len(p_vals), f"{tag}: 字段数 {len(c_vals)} vs {len(p_vals)}"
        for name, cv, pv in zip(fields, c_vals, p_vals):
            _close(cv, pv, tag=f"{tag} 第 {i} 帧 {name}")
    return tiers


# ===========================================================================
# 输入轨迹
# ===========================================================================

def _wall(y: float, gap_x: float, gap_w: float, *, half_h: float = 8.0,
          rot: float = 0.0, vx: float = 0.0, vy: float = 0.0,
          field_w: float = 640.0, ent0: int = 1) -> list[Rect]:
    """一堵"缺口墙"的两个矩形段（与平台 wall_with_gap 的编码方式一致）。"""
    left_w = gap_x - gap_w / 2.0
    right_x0 = gap_x + gap_w / 2.0
    right_w = field_w - right_x0
    out = []
    if left_w > 0:
        out.append(Rect(ent_id=ent0, x=left_w / 2.0, y=y, vx=vx, vy=vy,
                        half_w=left_w / 2.0, half_h=half_h, rotation=rot,
                        angular_velocity=0.0))
    if right_w > 0:
        out.append(Rect(ent_id=ent0 + 1, x=right_x0 + right_w / 2.0, y=y, vx=vx, vy=vy,
                        half_w=right_w / 2.0, half_h=half_h, rotation=rot,
                        angular_velocity=0.0))
    return out


def _player(x: float, y: float, vx: float = 0.0, vy: float = 0.0,
            r: float = 10.0, speed: float = 200.0) -> list[float]:
    return [x, y, vx, vy, r, speed, 1.0]


def _wrap_pi_branch_cut_trace(steps: int = 12) -> list[tuple[WorldView, float]]:
    """专打 ``wrap_pi`` 的**分支切割线**（CPP_PORT.md §2.1 点名的那个陷阱）。

    构造要点：指令方向必须**跨过 ±180°** 来回翻，`a_new - a_old` 才会落进
    ``(-2pi, -pi)`` —— 只有这一段区间里 Python 的 ``%`` 与 C++ 的 ``fmod`` 才不同
    （在 ``(-pi, pi)`` 内两者一致，差一个 2pi 的整数倍时也一致）。

    怎么做到：
      * ``forward=(-1,0)`` —— 车朝 −x 开，于是 ``fwd = -n̂ = (-1,0)``；
      * ``fwd_speed=0.95`` —— 油门吃掉 95% 的速度预算，横向只剩
        ``sqrt(1-0.95²) ≈ 0.312``；
      * 墙是**竖直**的（``rotation=90°``），缺口在 y 方向，``û = (0,1)``；
      * 玩家的横向坐标在缺口两侧来回跳，``sgn_lat`` 于是翻号。

    结果：方向在 ``(-0.95, +0.312)`` 与 ``(-0.95, -0.312)`` 之间跳，也就是
    ``+161.8°`` 与 ``-161.8°`` —— 差值 ``-323.6°``，正在那段区间里。

    实测：正确实现（Python 的 ``%``）把这次跳变判成 +35.4°，限幅后输出 +162.8°；
    朴素 ``fmod`` 判成 -324.6°，限幅后输出 **+160.8°** —— 差 2°。
    """
    gap_y = 230.0
    rects = [
        Rect(ent_id=1, x=320.0, y=(gap_y - 40.0) / 2.0, vx=0.0, vy=0.0,
             half_w=(gap_y - 40.0) / 2.0, half_h=8.0,
             rotation=math.radians(90.0), angular_velocity=0.0),
        Rect(ent_id=2, x=320.0, y=(gap_y + 40.0 + _FIELD_H) / 2.0, vx=0.0, vy=0.0,
             half_w=(_FIELD_H - (gap_y + 40.0)) / 2.0, half_h=8.0,
             rotation=math.radians(90.0), angular_velocity=0.0),
    ]
    frames = []
    for i in range(steps):
        py = 150.0 if i % 2 == 0 else 310.0          # 在缺口两侧来回跳
        frames.append((WorldView(player=_player(400.0, py), rects=list(rects),
                                 dt=_DT, step_index=i, field_w=_FIELD_W,
                                 field_h=_FIELD_H), 2.0))
    return frames


def _round_half_boundary_trace() -> list[tuple[WorldView, float]]:
    """专打分桶键的 **`.5` 边界**：Python ``round`` 五成双，``llround`` 五入。

    造法：让 ``band/2`` 与 ``rotation*1000`` **恰好**落在 x.5 上。实测过的三个值：

    ==================  ==============  ==========  ==========
    输入                量化前的值      Python      llround
    ==================  ==============  ==========  ==========
    ``y = 1.0``         ``band/2=0.5``  0           1
    ``y = 5.0``         ``band/2=2.5``  2           3
    ``rotation=0.0005`` ``rot*1000=0.5`` 0          1
    ==================  ==============  ==========  ==========

    分桶不同 → 墙的**条数**不同 → 选中那堵墙的 ``n_rects`` 也不同。实测本条轨迹
    上 Python 给 ``n_rects=1``、llround 给 ``n_rects=2``。

    ⚠ 为什么必须专门造：真实场景的墙 ``rotation`` 恒为 0、``band`` 是连续量，
    恰好落在 .5 上是**测度为零**的事件 —— 随手造的轨迹永远碰不到，于是
    "C++ 用了 llround"这种错会一路绿到上板。这个用例就是补这个洞。
    """
    frames = [
        # (a) band/2 正好 .5：y=1.0 → 0.5，y=2.0 → 1.0
        WorldView(player=_player(320.0, 72.0),
                  rects=[Rect(1, 320.0, 1.0, 0.0, 0.0, 100.0, 0.5, 0.0, 0.0),
                         Rect(2, 320.0, 2.0, 0.0, 0.0, 100.0, 50.0, 0.0, 0.0)],
                  dt=_DT, step_index=0, field_w=_FIELD_W, field_h=_FIELD_H),
        # (b) rotation*1000 正好 .5：0.0005 → 0.5，0.001 → 1.0
        WorldView(player=_player(320.0, 400.0),
                  rects=[Rect(1, 160.0, 300.0, 0.0, 0.0, 100.0, 8.0, 0.0005, 0.0),
                         Rect(2, 480.0, 300.0, 0.0, 0.0, 100.0, 8.0, 0.001, 0.0)],
                  dt=_DT, step_index=1, field_w=_FIELD_W, field_h=_FIELD_H),
        # (c) 两个都踩：band/2 = 2.5 且 rot*1000 = 0.5
        WorldView(player=_player(200.0, 200.0),
                  rects=[Rect(1, 200.0, 5.0, 0.0, 0.0, 100.0, 1.0, 0.0005, 0.0),
                         Rect(2, 500.0, 5.0, 0.0, 0.0, 100.0, 1.0, 0.0005, 0.0)],
                  dt=_DT, step_index=2, field_w=_FIELD_W, field_h=_FIELD_H),
    ]
    return [(v, 2.0) for v in frames]


def _stable_sort_trace(n: int = 20) -> list[tuple[WorldView, float]]:
    """专打 ``sorted(members, key=lambda r: r.x)`` 的**稳定性**。

    为什么需要：桶里 ``r.x`` 并列时，``members[0].rotation`` 决定 û —— 用了不稳定
    排序就会换人。而 libstdc++ 的 ``std::sort`` 在 **n <= 16** 时走插入排序，
    对并列元素**恰好也是稳定的**（实测 n=2..16 全部不打乱，n>=17 全部打乱）。
    所以桶里必须塞够 17 条以上才照得出来 —— 真实场景每堵墙只有 2 段，永远照不出来。

    造法：``n`` 条矩形**同 x 同 y**，``rotation`` 取 ``0.0001 + 1e-6*i`` ——
    每条**都不一样**。所有 ``round(rot*1000)`` 都是 0、所有 ``round(band/2)``
    都是 150，所以落在同一个桶里，但 ``sin(rotation)`` 逐条相差 1e-6 ——
    远大于 1e-9 的比较口径。

    ⚠ 早先写成"0.0001 / 0.0002 交替"，结果**抓不到**：``std::sort`` 确实打乱了
    顺序，但打乱后落在下标 0 的恰好还是 0.0001 那一批，于是墙一模一样。
    每条的 rotation 必须**互不相同**，任何重排才会改变 ``members[0].rotation``。
    """
    rects = [Rect(ent_id=i + 1, x=320.0, y=300.0, vx=0.0, vy=0.0,
                  half_w=20.0, half_h=8.0,
                  rotation=0.0001 + 1e-6 * i,
                  angular_velocity=0.0)
             for i in range(n)]
    frames = [WorldView(player=_player(320.0, 100.0), rects=rects, dt=_DT,
                        step_index=0, field_w=_FIELD_W, field_h=_FIELD_H)]
    # 再来几帧，让跨帧记忆（环形缓冲）也参与
    for k in range(1, 4):
        frames.append(WorldView(player=_player(320.0, 100.0 + 10.0 * k),
                                rects=list(rects), dt=_DT, step_index=k,
                                field_w=_FIELD_W, field_h=_FIELD_H))
    return [(v, 2.0) for v in frames]


_DT = 1.0 / 120.0
_FIELD_W, _FIELD_H = 640.0, 480.0


def _incoming_trace(steps: int = 240, *, seed: int = 0) -> list[tuple[WorldView, float]]:
    """墙朝静止的车压过来（现有 wall_with_gap 场景）。"""
    frames = []
    gap_x = 180.0
    y = 480.0
    for i in range(steps):
        rects = _wall(y, gap_x, 60.0, vy=-40.0)
        frames.append((WorldView(player=_player(320.0, 72.0), rects=rects,
                                 dt=_DT, step_index=i, field_w=_FIELD_W,
                                 field_h=_FIELD_H), 2.0))
        y -= 40.0 * _DT
        gap_x += 12.0 * _DT
    return frames


def _drive_trace(steps: int = 400, *, seed: int = 0) -> list[tuple[WorldView, float]]:
    """车朝三堵静态墙开过去，缺口横向滑动（真实模型；走 front_back 分支）。"""
    frames = []
    py = 60.0
    ys = (170.0, 280.0, 390.0)
    phases = (0.0, 1.1, 2.2)
    for i in range(steps):
        rects: list[Rect] = []
        for k, y0 in enumerate(ys):
            # 每堵墙的缺口位置按 seed 错开，且在 [160,480] 里来回滑
            gx = 320.0 + 140.0 * math.sin(phases[k] + i * _DT * 1.2 + 0.37 * seed)
            rects.extend(_wall(y0, gx, 70.0, ent0=1 + 2 * k))
        frames.append((WorldView(player=_player(320.0, py, 0.0, 180.0), rects=rects,
                                 dt=_DT, step_index=i, field_w=_FIELD_W,
                                 field_h=_FIELD_H), 3.0))
        py += 180.0 * _DT
    return frames


def _tilted_trace(steps: int = 200) -> list[tuple[WorldView, float]]:
    """**倾斜的墙** —— 专打 round(rotation*1000) 的分桶。

    轴对齐的墙 rotation 恒为 0，分桶键永远相同，`round` 的五成双、
    `stable_sort` 的并列行为**全都照不出来**。这里让角度连续变化，
    让键在整数边界上来回跨。"""
    frames = []
    for i in range(steps):
        rot = 0.004 * math.sin(i * _DT * 2.0)      # ±0.004 rad ≈ ±4 个桶
        gap_x = 320.0
        y = 300.0
        # 手工造两段倾斜的墙：绕整堵墙的中心转 rot
        c, s = math.cos(rot), math.sin(rot)
        rects = []
        for sx, w in ((gap_x / 2.0, gap_x), (gap_x + (640.0 - gap_x) / 2.0,
                                             640.0 - gap_x)):
            dx, dy = sx - 320.0, 0.0
            rects.append(Rect(ent_id=1, x=320.0 + dx * c - dy * s,
                              y=y + dx * s + dy * c, vx=0.0, vy=0.0,
                              half_w=w / 2.0 - 30.0, half_h=8.0, rotation=rot,
                              angular_velocity=0.0))
        frames.append((WorldView(player=_player(300.0, 120.0), rects=rects,
                                 dt=_DT, step_index=i, field_w=_FIELD_W,
                                 field_h=_FIELD_H), 2.0))
    return frames


def _edge_trace() -> list[tuple[WorldView, float]]:
    """把所有分支都踩一遍的边角集：单段墙 / 无墙 / 身后墙 / 场边 / 贴墙。"""
    frames: list[tuple[WorldView, float]] = []
    W, H = WorldView, float
    cases = [
        ([], _player(320.0, 72.0)),                                    # 空场 → IDLE
        ([Rect(1, 320.0, 400.0, 0.0, -40.0, 300.0, 8.0, 0.0, 0.0)],
         _player(320.0, 72.0)),                                        # 单段（整面带缺口？只剩一段）
        (_wall(400.0, 60.0, 60.0), _player(20.0, 72.0)),               # 缺口在场边之外
        (_wall(400.0, 600.0, 60.0), _player(620.0, 72.0)),             # 同上，右侧
        (_wall(100.0, 320.0, 60.0), _player(320.0, 400.0)),            # 墙在身后
        (_wall(72.0, 320.0, 60.0), _player(320.0, 72.0)),              # 玩家正压在墙带上
        (_wall(90.0, 320.0, 200.0), _player(320.0, 72.0)),             # 缺口很宽
        (_wall(90.0, 320.0, 4.0), _player(320.0, 72.0)),               # 缺口几乎放不下
        ([Rect(1, 320.0, 90.0, 0.0, 0.0, 300.0, 8.0, 0.0, 0.0),
          Rect(2, 320.0, 90.0, 0.0, 0.0, 300.0, 8.0, 0.0, 0.0)],
         _player(320.0, 72.0)),                                        # 完全重合（同一个桶）
    ]
    for i, (rects, pl) in enumerate(cases):
        frames.append((W(player=pl, rects=list(rects), dt=_DT, step_index=i,
                         field_w=_FIELD_W, field_h=_FIELD_H), 2.0))
    # 再来一圈连续逼近（学全局缺口宽度以后再走单段分支）
    for i in range(60):
        y = 400.0 - 3.0 * i
        frames.append((W(player=_player(320.0, 72.0),
                         rects=_wall(y, 200.0, 60.0), dt=_DT, step_index=100 + i,
                         field_w=_FIELD_W, field_h=_FIELD_H), 2.0))
    # 只剩右段的单段情形（缺口在左、可见的是右段）+ 缺口滑动
    for i in range(60):
        rects = [Rect(1, 480.0, 300.0, 0.0, 0.0, 160.0, 8.0, 0.0, 0.0)]
        frames.append((W(player=_player(200.0 + i, 120.0), rects=rects, dt=_DT,
                         step_index=200 + i, field_w=_FIELD_W,
                         field_h=_FIELD_H), 2.0))
    return frames


# ===========================================================================
# 测试
# ===========================================================================

def test_cpp_port_is_byte_exact_on_incoming(cli):
    """incoming 场景：墙朝静止的车压过来，只有左右移动（front_back=False）。"""
    tiers = _compare(AlgoConfig(), _incoming_trace(), cli, tag="incoming")
    assert tiers, "轨迹是空的？"
    assert set(tiers) - {"IDLE"}, "整条轨迹一个有效决策都没有，这个测试是空跑的"


def test_cpp_port_is_byte_exact_on_drive(cli):
    """drive 场景：车朝静态墙开过去，允许前后控制（走 SLOW/STOP/CRUISE 分支）。"""
    cfg = AlgoConfig(front_back=True, forward=(0.0, 1.0))
    tiers = _compare(cfg, _drive_trace(), cli, tag="drive")
    assert len(tiers) >= 2, f"只踩到一种分支 {tiers}，覆盖不够"


def test_cpp_port_matches_with_turn_rate_limiting(cli):
    """开 max_turn_rate —— 这条路径会走 ``atan2/cos/sin`` + ``wrap_pi``。

    ``wrap_pi`` 是 CPP_PORT.md §2.1 点名的陷阱：朴素的 ``fmod`` 翻译在这里
    会给出 -2pi 而不是 0，方向限幅整帧错向。
    """
    cfg = AlgoConfig(max_turn_rate=120.0)
    _compare(cfg, _incoming_trace(steps=300), cli, tag="turn-rate")


def test_cpp_port_is_byte_exact_on_tilted_walls(cli):
    """**倾斜的墙** —— 专打分桶键 ``round(rotation*1000)``。

    轴对齐的墙 rotation 恒为 0，五成双 / 稳定排序都照不出来。
    """
    _compare(AlgoConfig(), _tilted_trace(), cli, tag="tilted")


def test_cpp_port_is_byte_exact_on_edge_cases(cli):
    """边角集：空场 / 单段 / 身后 / 贴墙 / 缺口极宽极窄 / 重复桶。"""
    tiers = _compare(AlgoConfig(), _edge_trace(), cli, tag="edge")
    assert "IDLE" in tiers, "空场那一帧应当走 IDLE —— 否则这个用例根本没造出来"


def test_cpp_port_is_byte_exact_on_real_episode(cli):
    """**真场景**：跑一遍平台的 wall_with_gap，把每一步的输入原样喂给两边。

    前面几个用例是手搓的；这一个用的是平台自己的场景与适配层，防的是
    "手搓输入恰好绕开了某个平台才有的形状"。
    """
    bullet = pytest.importorskip("bullet_sim.simulator.env",
                                 reason="没有平台，跳过真场景对账")
    sys.path.insert(0, str(_ROOT / "core" / "cpu"))
    import gap_wall_demo as demo                                    # noqa: E402
    from bullet_sim.entities.player import P_SPEED as _P_SPEED       # noqa: E402
    from cpu.adapters.sim_adapter import view_from_world             # noqa: E402

    spec = demo.build_spec(seed=3, gap_width=60.0, seconds=6.0,
                           obstacle_speed=40.0, player_y=None,
                           player_speed=200.0, interval=1.6)
    env = bullet.BulletHellEnv(spec, seed=3, codec="action", collision=None)
    env.reset(seed=3)

    cfg = AlgoConfig()
    mem = GapMemory()
    frames: list[tuple[WorldView, float]] = []
    total = int(spec.total_steps)
    for i in range(total):
        view = view_from_world(env.world)
        horizon = (total - i) * float(spec.dt)
        frames.append((view, horizon))
        dec = plan(view, mem, cfg, horizon)
        cmd = to_command(dec)
        from bullet_sim.action import Action
        from bullet_sim.action.base import action_to_codec_input
        raw = action_to_codec_input(
            Action.zero() if math.hypot(*cmd) < EPS else Action.from_vector(cmd),
            env.world.codec, float(env.world.player[_P_SPEED]))
        env.step(raw)

    assert len(frames) > 100, f"真场景只跑了 {len(frames)} 步？"
    _compare(cfg, frames, cli, tag="real-episode")


def test_cpp_port_matches_on_the_wrap_pi_branch_cut(cli):
    """``wrap_pi`` 的**分支切割线**：``a_new - a_old`` 落进 ``(-2pi, -pi)``。

    这是 CPP_PORT.md §2.1 点名的陷阱。**反向验证过**：把 C++ 的 ``wrap_pi``
    换成朴素的 ``fmod(a+pi,2pi)-pi``，这一条当场变红，而其余 7 条全绿 ——
    也就是说，没有这个用例，"C++ 抄了个 fmod"能一路混过去。
    """
    cfg = AlgoConfig(front_back=False, fwd_speed=0.95,
                     forward=(-1.0, 0.0), max_turn_rate=120.0)
    _compare(cfg, _wrap_pi_branch_cut_trace(), cli, tag="wrap-pi-cut")


def test_cpp_port_matches_on_round_half_boundaries(cli):
    """分桶键的 **`.5` 边界**：``round`` 五成双 vs ``llround`` 五入。

    **反向验证过**：把 ``py_round_int`` 换成 ``(int)std::llround(x)``，
    只有这一条会红（其余 7 条全绿）。
    """
    _compare(AlgoConfig(), _round_half_boundary_trace(), cli, tag="round-half")


def test_cpp_port_matches_on_stable_bucket_sort(cli):
    """桶内**按 x 排序的稳定性**：``r.x`` 并列时 ``members[0].rotation`` 决定 û。

    **反向验证过**：把 ``std::stable_sort`` 换成 ``std::sort``，只有这一条会红。
    桶里必须 >= 17 条 —— libstdc++ 的 ``std::sort`` 在 n<=16 时恰好也稳定。
    """
    _compare(AlgoConfig(), _stable_sort_trace(), cli, tag="stable-sort")


def test_naive_translations_are_actually_wrong():
    """**反向验证**：证明上面那三个陷阱不是纸上谈兵。

    三条"看起来对"的朴素翻译，各自在一个具体输入上给出不同答案 —— 如果这个
    测试红了，说明 Python 侧的行为变了，CPP_PORT.md §2 需要重新核对。
    """
    # 1) wrap_pi：Python 的 % 对负数非负，fmod 保留符号
    angle = -4.0 * math.pi
    py = (angle + math.pi) % (2.0 * math.pi) - math.pi
    naive = math.fmod(angle + math.pi, 2.0 * math.pi) - math.pi
    assert abs(py) < 1e-12, py
    assert abs(naive - (-2.0 * math.pi)) < 1e-12, naive
    assert abs(py - naive) > 1.0, "朴素 fmod 翻译竟然和 Python 一致？"

    # 2) round：Python 五成双，llround 五入
    assert round(0.5) == 0 and round(1.5) == 2 and round(2.5) == 2
    assert math.floor(0.5 + 0.5) == 1 and math.floor(2.5 + 0.5) == 3

    # 3) sorted 的稳定性：r.x 相等时顺序必须保持
    a = Rect(1, 5.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.1, 0.0)
    b = Rect(2, 5.0, 0.0, 0.0, 0.0, 1.0, 1.0, 0.9, 0.0)
    out = sorted([a, b], key=lambda r: r.x)
    assert out[0] is a and out[1] is b, "sorted 不再稳定，分桶的 û 来源会换人"


# ===========================================================================
# 边界余量（edge_margin）：C++ 与 Python 必须逐位一致
# ===========================================================================
def _edge_margin_trace() -> list[tuple[WorldView, float]]:
    """贴着四条边、朝外推的一串帧 —— 专打 `limit_edges` 的斜坡与归零。

    每帧换一个"距边界的余量"（0 / margin/4 / margin/2 / margin / 很远），
    朝外分量分别应当是 0 / 25% / 50% / 100% / 100%；切向分量一律不变。
    """
    frames: list[tuple[WorldView, float]] = []
    margin = 25.0
    for d in (0.0, margin / 4.0, margin / 2.0, margin, 100.0):
        # 左边界：中心 x = 半径 + d
        frames.append((WorldView(player=_player(10.0 + d, 240.0),
                                 rects=_wall(200.0, 320.0, 120.0),
                                 dt=1.0 / 120.0, step_index=0,
                                 field_w=640.0, field_h=480.0), 2.0))
        # 右边界：中心 x = 场宽 - 半径 - d
        frames.append((WorldView(player=_player(630.0 - d, 240.0),
                                 rects=_wall(200.0, 320.0, 120.0),
                                 dt=1.0 / 120.0, step_index=1,
                                 field_w=640.0, field_h=480.0), 2.0))
        # 上边界（y 小）
        frames.append((WorldView(player=_player(320.0, 10.0 + d),
                                 rects=_wall(200.0, 320.0, 120.0),
                                 dt=1.0 / 120.0, step_index=2,
                                 field_w=640.0, field_h=480.0), 2.0))
        # 下边界（y 大）
        frames.append((WorldView(player=_player(320.0, 470.0 - d),
                                 rects=_wall(200.0, 320.0, 120.0),
                                 dt=1.0 / 120.0, step_index=3,
                                 field_w=640.0, field_h=480.0), 2.0))
    return frames


def test_cpp_port_matches_with_edge_margin(cli) -> None:
    """`edge_margin > 0` 时两边必须逐位一致（斜坡是纯乘除，要求逐位相等）。"""
    cfg = AlgoConfig(edge_margin=25.0)
    frames = _edge_margin_trace()
    tiers = _compare(cfg, frames, cli, tag="edge_margin")
    assert sum(tiers.values()) == len(frames)


def test_edge_margin_default_is_off_and_unchanged(cli) -> None:
    """默认 `edge_margin=0` 必须与旧行为一致（同一条轨迹、两次运行结果相同）。"""
    frames = _edge_margin_trace()
    a = _compare(AlgoConfig(), frames, cli, tag="edge_off_a")
    b = _compare(AlgoConfig(edge_margin=0.0), frames, cli, tag="edge_off_b")
    assert a == b

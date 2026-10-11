"""网格势场 + 波前测地距离的回归测试。

**全部用合成几何**：不 import bullet_sim、不开仿真、不依赖平台。
这一条是刻意的 —— ``potential_field`` 的定位就是"零平台耦合"，
测试必须能独立证明这一点。

锁住的每一条都对应一次实际踩到的错（写这个模块时真踩了）：

1. **``J = U / 距离`` 会奖励"远"** —— 偏导 ``∂(U/d)/∂d < 0``。
2. **只有势 + 直线距离，会把墙后紧贴的空地当成好去处** —— 那里势=0、
   直线又近；实测所有 λ 都选它。
3. **代价函数里没有"靠近目标"就退化成"原地不动"** ——
   玩家所在格的行程代价是 0，它就是全局最小。
4. **缓存不能挂在对调用方填的字段上** —— 一度用 ``view.step_index`` 判缓存，
   调用方忘了填（默认 0）→ 缓存永远命中 → 路点永远不变 →
   角色朝一个固定点走然后原地抖，**外面看不出任何异常**。现在自己计数。
5. **输出方向也要做可达性检查** —— 下降落点是对的，但"从当前位置到落点的
   直线"可能切进膨胀障碍（缺口喉部最容易）。没有这一步，实测 ``cell=4``
   会卡在缺口口上。
"""

from __future__ import annotations

import math

import pytest

from cpu.gap_avoid import Rect
from cpu.potential_field import (BLOCKED, FieldConfig, FieldMemory, FieldView,
                                backward_arrival, build_field, build_grid,
                                build_layers, descend_field, line_clear,
                                plan_field, wavefront, wavefront_mask)

# 平台实测几何（seed=7、难度 0.4 的 wall_with_gap）：
#   墙在 y=266.2，厚 8，缺口 x∈[313.0, 388.6]，缺口中心 350.8
WY, HH = 266.2, 4.0
SEGS = ((0.0, 313.0), (388.6, 640.0))
GAP_C = 0.5 * (SEGS[0][1] + SEGS[1][0])


def _walls() -> tuple[Rect, ...]:
    return tuple(Rect(i, 0.5 * (a + b), WY, 0.0, 0.0, 0.5 * (b - a), HH, 0.0, 0.0)
                 for i, (a, b) in enumerate(SEGS))


def _view(px: float, py: float, goal=(320.0, 240.0), *, circles=(),
          radius: float = 10.0) -> FieldView:
    return FieldView(player=[px, py, 0.0, 0.0, radius, 200.0, 1.0],
                     rects=_walls(), circles=circles,
                     field_w=640.0, field_h=480.0, goal=goal)


# ---------------------------------------------------------------------------
# 1. 网格与势
# ---------------------------------------------------------------------------
def test_grid_covers_field_and_marks_inflated_wall() -> None:
    g = build_grid(_view(320.0, 105.6), FieldConfig(cell=8.0))
    assert g.nx == 80 and g.ny == 60
    # 实体墙段里应当是膨胀障碍（注意别拿 GAP_C —— 那是缺口中心，本来就没墙）
    assert g.blocked[g.index(*g.cell_of(200.0, WY))]
    assert g.blocked[g.index(*g.cell_of(500.0, WY))]
    # 缺口中心、离墙 40 处应当自由（膨胀量 16 < 40）
    assert not g.blocked[g.index(*g.cell_of(GAP_C, WY - 40.0))]


def test_gap_stays_passable() -> None:
    """缺口必须留出可通行走廊 —— 否则膨胀量算错了。"""
    g = build_grid(_view(320.0, 105.6), FieldConfig(cell=8.0, safety_margin=6.0))
    # 沿墙的 y 走一遍，看缺口那一列有多少自由格
    # 只采缺口【内部】：两个端点本身就是墙的角点，必然落在膨胀区内
    col = [g.blocked[g.index(*g.cell_of(x, WY))] for x in
           [332.0, 340.0, 350.8, 364.0, 372.0]]
    assert not any(col), f"缺口被膨胀堵住了：{[bool(c) for c in col]}"
    # 缺口两边的实体段必须是障碍
    assert g.blocked[g.index(*g.cell_of(200.0, WY))]
    assert g.blocked[g.index(*g.cell_of(500.0, WY))]


def test_blocked_cells_get_blocked_risk() -> None:
    g = build_grid(_view(320.0, 105.6), FieldConfig(cell=8.0))
    k = g.index(*g.cell_of(200.0, WY))          # 实体墙段里
    assert g.risk[k] == BLOCKED
    k2 = g.index(*g.cell_of(200.0, 60.0))       # 远处自由空间
    assert g.risk[k2] == 0.0


# ---------------------------------------------------------------------------
# 2. 波前：测地距离不会被墙骗
# ---------------------------------------------------------------------------
def test_wavefront_cannot_cross_solid_wall() -> None:
    """墙后紧贴的格，测地距离必须明显大于直线距离。

    这是整个模块存在的理由：**势是点的性质，可达性是路径的性质。**
    """
    g = build_grid(_view(200.0, 105.6), FieldConfig(cell=8.0))
    start = g.cell_of(200.0, 105.6)
    d = wavefront(g, start)
    behind = g.cell_of(200.0, WY + 26.0)         # 墙后、已出膨胀区（16）
    geo = d[g.index(*behind)]
    straight = math.hypot(200.0 - 200.0, (WY + 10.0) - 105.6)
    assert math.isfinite(geo), "绕行应当可达（缺口在右边）"
    assert geo > 1.5 * straight, (
        f"测地 {geo:.0f} 应当远大于直线 {straight:.0f}；"
        "如果差不多，说明波前从墙里漏过去了")


def test_wavefront_respects_blocked_cells() -> None:
    g = build_grid(_view(320.0, 105.6), FieldConfig(cell=8.0))
    d = wavefront(g, g.cell_of(320.0, 105.6))
    for i in range(g.nx):
        for j in range(g.ny):
            k = i * g.ny + j
            if g.blocked[k]:
                assert not math.isfinite(d[k]), "波前不许进入膨胀障碍"


def test_wavefront_unreachable_when_walled_off() -> None:
    """连通性：把缺口也堵上，墙后应当**不可达**（而不是"绕远"）。"""
    walls = list(_walls())
    walls.append(Rect(99, GAP_C, WY, 0.0, 0.0, 0.5 * (SEGS[1][0] - SEGS[0][1]),
                      HH, 0.0, 0.0))            # 用一段墙把缺口填死
    v = FieldView(player=[320.0, 105.6, 0, 0, 10.0, 200.0, 1.0], rects=tuple(walls),
                  circles=(), field_w=640.0, field_h=480.0, goal=(320.0, 240.0))
    g = build_grid(v, FieldConfig(cell=8.0))
    d = wavefront(g, g.cell_of(320.0, 105.6))
    assert not math.isfinite(d[g.index(*g.cell_of(320.0, WY + 20.0))])


# ---------------------------------------------------------------------------
# 3. 决策函数：两个反例
# ---------------------------------------------------------------------------
def test_field_is_minimal_at_goal() -> None:
    g = build_grid(_view(320.0, 105.6), FieldConfig(cell=8.0))
    F, diag = build_field(g, _view(320.0, 105.6), FieldConfig(cell=8.0))
    gk = g.index(*g.cell_of(320.0, 240.0))
    finite = [v for v in F if math.isfinite(v)]
    # 目标处的 F 不是恰好 0：那里仍有很小的排斥势（缺口内距两段墙各 23.3）。
    # 真正要锁的是"它是全局最小"。
    assert F[gk] == pytest.approx(min(finite), abs=1e-9)
    assert F[gk] < 0.05, f"目标处的场值应当很小，实得 {F[gk]}"


def test_no_goal_term_degenerates_to_stay_put() -> None:
    """**反面对照**：关掉"到目标"这一项，最优就是原地不动。

    这条证明上面那个"最小值在目标处"的断言有分辨力 ——
    不是"怎么设都指向目标"。
    """
    v = _view(320.0, 105.6)
    cfg = FieldConfig(cell=8.0, via_goal_weight=0.0)
    g = build_grid(v, cfg)
    F, diag = build_field(g, v, cfg)
    assert diag.get("fallback") == "no_goal"
    start = g.cell_of(320.0, 105.6)
    reach, _ = descend_field(g, F, start, steps=4)
    assert reach == start, "没有目标项时不该移动（这就是退化）"


def test_line_clear_rejects_segment_through_wall() -> None:
    g = build_grid(_view(200.0, 105.6), FieldConfig(cell=8.0))
    a = g.cell_of(200.0, 105.6)
    b = a
    # 正上方穿墙的那一格
    through = g.cell_of(200.0, WY)
    assert not line_clear(g, a, through)
    # 远处自由方向应当通
    assert line_clear(g, a, g.cell_of(200.0, 150.0))


# ---------------------------------------------------------------------------
# 4. 缓存不能挂在对调用方填的字段上
# ---------------------------------------------------------------------------
def test_cache_advances_without_step_index() -> None:
    """调用方从不设置 ``step_index``，路点也必须逐步前进。

    **这是真踩过的坑**：缓存一度挂在 ``view.step_index`` 上，忘了填
    （默认恒为 0）→ 缓存永远命中 → 路点永远不变 → 角色朝一个固定的点走
    然后原地抖，而外面看不出任何异常。现在用内存里的内部计数。
    """
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    mem = FieldMemory()
    targets = []
    px, py = 200.0, 105.6
    for _ in range(6):
        v = _view(px, py, goal=(200.0, 450.0))       # 注意：step_index 从不设置
        d = plan_field(v, mem, cfg)
        targets.append(d.target)
        px += d.wx * 6.0
        py += d.wy * 6.0
    assert mem.recomputes == 6, "每一步都应当重算"
    assert len(set(targets)) > 1, f"路点一直没变：{targets}"
    assert py > 105.6 + 20.0, f"没有真正前进：终点 y={py:.1f}"


def test_recompute_caches_within_period() -> None:
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=4)
    mem = FieldMemory()
    for _ in range(4):
        plan_field(_view(320.0, 105.6, goal=(320.0, 240.0)), mem, cfg)
    assert mem.recomputes == 1, "recompute=4 时前 4 次调用只该重算 1 次"
    plan_field(_view(320.0, 105.6, goal=(320.0, 240.0)), mem, cfg)
    assert mem.recomputes == 2


# ---------------------------------------------------------------------------
# 5. 端到端：绕过实体墙走到墙后
# ---------------------------------------------------------------------------
def _navigate(start, goal, cfg, *, step=6.0, max_steps=400):
    mem = FieldMemory()
    px, py = start
    traj = [(px, py)]
    for _ in range(max_steps):
        d = plan_field(_view(px, py, goal=goal), mem, cfg)
        if math.hypot(d.wx, d.wy) < 1e-9:
            return traj, "STALL"
        px += d.wx * step
        py += d.wy * step
        traj.append((px, py))
        if math.hypot(px - goal[0], py - goal[1]) < 12.0:
            return traj, "ARRIVED"
    return traj, "TIMEOUT"


def _crosses_wall(traj) -> bool:
    for (x0, y0), (x1, y1) in zip(traj, traj[1:]):
        for s in range(41):
            t = s / 40.0
            x = x0 + (x1 - x0) * t
            y = y0 + (y1 - y0) * t
            if abs(y - WY) <= HH and any(a <= x <= b for a, b in SEGS):
                return True
    return False


@pytest.mark.parametrize("cell,steps", [(8.0, 4), (4.0, 6)])
def test_navigates_around_solid_wall(cell: float, steps: int) -> None:
    """目标在【实体墙】后面 —— 必须从缺口绕过去。

    这就是本方案相对朴素 APF 的价值所在：朴素 APF 只会"直线朝目标 +
    被墙推开"，实测 400 步都卡在墙前（终点 y=237~262，从没越过 266）；
    本方案从缺口绕过去，且**从不穿墙**。
    """
    cfg = FieldConfig(cell=cell, descend_steps=steps, travel_weight=1.0, recompute=1)
    traj, res = _navigate((200.0, 105.6), (200.0, 450.0), cfg)
    assert res == "ARRIVED", f"{res}，终点 {traj[-1]}"
    assert not _crosses_wall(traj), "轨迹穿墙了"
    # 必须真的往缺口（x≈350.8）那边绕过：x 最大值应当超过缺口左沿
    assert max(x for x, _ in traj) > SEGS[0][1], "没有经过缺口"


def test_straight_line_apf_would_fail_here() -> None:
    """反面对照：同样几何下，"直线朝目标 + Khatib 排斥"过不去。

    把这条也写成测试，是为了让"本方案有用"这个结论**可回归** ——
    如果哪天有人把测地距离换回直线距离，上面那条会挂，这条会提醒为什么。
    """
    def naive(px, py, *, rr=150.0, eta=800.0, kp=0.05, step=6.0, steps=400):
        for _ in range(steps):
            gx, gy = 0.0, 0.0
            dx, dy = 200.0 - px, 450.0 - py
            n = math.hypot(dx, dy) or 1.0
            gx += kp * dx / n
            gy += kp * dy / n
            for a, b in SEGS:
                cx = min(max(px, a), b)
                vx, vy = px - cx, py - WY
                dd = math.hypot(vx, vy)
                if 1e-9 < dd < rr:
                    m = eta * (1.0 / dd - 1.0 / rr) / (dd * dd)
                    gx += vx / dd * m
                    gy += vy / dd * m
            n = math.hypot(gx, gy)
            if n < 1e-12:
                return "STALL", (px, py)
            px += gx / n * step
            py += gy / n * step
            if math.hypot(px - 200.0, py - 450.0) < 12.0:
                return "ARRIVED", (px, py)
        return "TIMEOUT", (px, py)

    for kp in (0.05, 0.3, 1.0):
        res, end = naive(200.0, 105.6, kp=kp)
        assert res != "ARRIVED", f"kp={kp} 竟然过了（{end}）—— 反例失效了"


# ---------------------------------------------------------------------------
# 6. 确定性
# ---------------------------------------------------------------------------
def test_same_input_same_output() -> None:
    """同样输入必须给同样输出 —— 并列时取下标最小，不看 dict 迭代顺序。"""
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    runs = []
    for _ in range(3):
        mem = FieldMemory()
        out = []
        px, py = 260.0, 120.0
        for _ in range(10):
            d = plan_field(_view(px, py, goal=(320.0, 240.0)), mem, cfg)
            out.append((round(d.wx, 12), round(d.wy, 12), d.target))
            px += d.wx * 5.0
            py += d.wy * 5.0
        runs.append(out)
    assert runs[0] == runs[1] == runs[2]


def test_no_numpy_or_bullet_sim_imported() -> None:
    """零平台耦合：本模块不许把 numpy / bullet_sim 拉进来。"""
    import ast, pathlib
    src = (pathlib.Path(__file__).resolve().parents[1]
           / "potential_field.py").read_text(encoding="utf-8")
    tree = ast.parse(src)
    mods: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            mods.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            mods.add(node.module.split(".")[0])
    assert "numpy" not in mods, f"算法模块不许依赖 numpy：{sorted(mods)}"
    assert "bullet_sim" not in mods, f"算法模块不许依赖平台：{sorted(mods)}"
    # 平台适配是**一个函数**（field_from_world），且只做鸭子类型取值，
    # 不在模块级 import 平台 —— 这样本模块可以脱离平台单独编译/移植。
    assert "field_from_world" in src


# ---------------------------------------------------------------------------
# 7. ★ 目标被封死时必须降级，不许原地不动当靶子
# ---------------------------------------------------------------------------
def _walled_off_view(px: float, py: float):
    """用一圈墙把玩家和"场中央"这个目标彻底隔开。"""
    walls = list(_walls())
    # 在玩家和目标之间横一道密闭的墙（y=180 处，贯穿整个场地宽度）
    walls.append(Rect(50, 320.0, 180.0, 0.0, 0.0, 400.0, 4.0, 0.0, 0.0))
    return FieldView(player=[px, py, 0.0, 0.0, 10.0, 200.0, 1.0],
                     rects=tuple(walls), circles=(),
                     field_w=640.0, field_h=480.0, goal=(320.0, 240.0))


def test_goal_unreachable_falls_back_instead_of_freezing() -> None:
    """**这是实测踩到的最大失效模式。**

    障碍把场地切开以后，玩家所在连通分量里到不了目标，于是下降场里
    玩家周围全是 ``inf``，找不到更低的邻居 → 返回零速度 → **角色原地不动当靶子**。

    实测三种障碍各 3 个种子，"冻结帧"与"目标不可达帧"**完全相等**：
    wall_with_gap 2658/2658、moving_block 793/793、small_obstacles 1197/1197。

    修法：只看【玩家所在格】能不能到目标；到不了就降级成"只用风险场"。
    """
    v = _walled_off_view(200.0, 100.0)
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    g = build_grid(v, cfg)
    start = g.cell_of(200.0, 100.0)
    F, diag = build_field(g, v, cfg, start)
    assert diag["player_can_reach_goal"] is False
    assert diag["fallback"] == "goal_unreachable_from_player"
    # 玩家那一格必须是有限的，否则下降无处可去
    assert math.isfinite(F[g.index(*start)]), "降级后玩家所在格必须有限"


def test_player_cannot_reach_goal_even_when_others_can() -> None:
    """判据必须是【玩家】的连通分量，不是"全场有几个格能到目标"。

    这里把玩家关在一个小房间里，房间外面照样有大片能到目标的自由格 ——
    只看 ``n_goal_reach`` 会误判成"能到"。
    """
    walls = list(_walls())
    # 给玩家造一个封闭小房间（4 面墙），目标在外面
    for i, (x, y, hw, hh) in enumerate(((200.0, 60.0, 60.0, 4.0),
                                        (200.0, 140.0, 60.0, 4.0),
                                        (140.0, 100.0, 4.0, 44.0),
                                        (260.0, 100.0, 4.0, 44.0))):
        walls.append(Rect(100 + i, x, y, 0.0, 0.0, hw, hh, 0.0, 0.0))
    v = FieldView(player=[200.0, 100.0, 0, 0, 8.0, 200.0, 1.0],
                  rects=tuple(walls), circles=(),
                  field_w=640.0, field_h=480.0, goal=(320.0, 240.0))
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    g = build_grid(v, cfg)
    start = g.cell_of(200.0, 100.0)
    F, diag = build_field(g, v, cfg, start)
    assert diag["n_goal_reach"] > 0, "房间外面应当有能到目标的格子"
    assert diag["player_can_reach_goal"] is False, "但玩家自己出不去"
    assert diag["fallback"] == "goal_unreachable_from_player"


def test_fallback_moves_first_and_only_stops_at_optimum() -> None:
    """目标不可达而降级后：**先动起来**，停下来时必须在最优点。

    "零速度"本身不一定是坏的 —— 到达安全点之后停下是对的。
    坏的是**一开始就不动**（那是退化平衡），或者停在非最优点（那是假收敛）。
    """
    v = _walled_off_view(200.0, 100.0)
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    mem = FieldMemory()
    px, py = 200.0, 100.0
    moved_early = 0
    bad_stops = 0
    for i in range(40):
        d = plan_field(_view_swap(v, px, py), mem, cfg)
        if math.hypot(d.wx, d.wy) < 1e-12:
            if not d.diag.get("at_optimum"):
                bad_stops += 1
        elif i < 5:
            moved_early += 1
        px += d.wx * 5.0
        py += d.wy * 5.0
    assert moved_early >= 3, f"开头 {5 - moved_early}/5 帧没动 —— 退化平衡又回来了"
    assert bad_stops == 0, f"有 {bad_stops} 帧停在非最优点（假收敛）"


def _view_swap(base: FieldView, px: float, py: float) -> FieldView:
    """换掉玩家位置，其余（墙、目标）照抄。"""
    pl = list(base.player)
    pl[0], pl[1] = px, py
    return FieldView(player=pl, rects=base.rects, circles=base.circles,
                     field_w=base.field_w, field_h=base.field_h, goal=base.goal)


# ---------------------------------------------------------------------------
# 8. ★ 路点落到脚下的"退化平衡"
# ---------------------------------------------------------------------------
def test_waypoint_underfoot_forces_recompute() -> None:
    """**这是零速度帧的第二个来源，也是我诊断错两次的那个。**

    机制：玩家正好停在某个**格心**上、而那个格心就是缓存的路点
    ⇒ ``|路点 − 位置| = 0`` ⇒ 输出零速度 ⇒ 位置不变 ⇒ 下一帧还是零，
    一直停到缓存过期（``recompute`` 步）。

    实测 ``wall_with_gap``（``recompute=12``）：28 帧零速度/种子，
    其中 20 帧是这个退化平衡、8 帧是"真的到了最优点"。
    修法：路点在脚下时**不信缓存，强制重算一次**；重算后仍在脚下才是真到了。

    这条测试直接构造"玩家站在格心上"的场景来复现。
    """
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=12)
    mem = FieldMemory()
    g_probe = build_grid(_view(200.0, 105.6, goal=(200.0, 450.0)), cfg)
    # 取一个格心当起点，且它不是目标
    cx, cy = g_probe.centre(*g_probe.cell_of(200.0, 105.6))
    assert (cx, cy) != (200.0, 450.0)

    forwards = 0
    px, py = cx, cy
    for i in range(12):
        d = plan_field(_view(px, py, goal=(200.0, 450.0)), mem, cfg)
        if math.hypot(d.wx, d.wy) > 1e-12:
            forwards += 1
        px += d.wx * 6.0
        py += d.wy * 6.0
    # 第一帧就必须动（强制重算），不能因为"站在格心上"而卡住
    assert forwards >= 6, f"12 帧里只有 {forwards} 帧在动 —— 退化平衡又回来了"


def test_at_optimum_has_zero_cost_and_converges() -> None:
    """到了最优点时，场值必须是 0，而且几步之内速度收敛到 0。

    注意"在最优点"和"速度恰好为 0"**不是**一回事：玩家可能落在目标**格内**
    但不在格心上，那就会有一个不到半个格的小修正量（实测 4.47 单位）。
    这是无害的网格离散化残差，不是退化平衡 —— 判据是**场值**，不是速度。
    """
    cfg = FieldConfig(cell=8.0, descend_steps=4, recompute=1)
    mem = FieldMemory()
    goal = (200.0, 450.0)
    px, py = goal
    speeds = []
    for _ in range(6):
        d = plan_field(_view(px, py, goal=goal), mem, cfg)
        speeds.append(math.hypot(d.wx, d.wy))
        px += d.wx * 4.0
        py += d.wy * 4.0
    assert min(speeds) < 1e-12, f"从没停下过：{speeds}"
    assert speeds[0] < 1.0, f"第一帧就给了满速，不像在最优点附近：{speeds[0]}"
    # 到了最优点，场值应当是 0
    d = plan_field(_view(*goal, goal=goal), FieldMemory(), cfg)
    assert d.cost == pytest.approx(0.0, abs=1e-9)


# ---------------------------------------------------------------------------
# 9. ★ 时间维（时间扩展的倒推 DP）
# ---------------------------------------------------------------------------
def _moving_wall_view(px, py, goal, *, wall_y=100.0, vy=400.0):
    """一面**横贯全场、正在往上走**的墙。玩家在下（y=50）、目标在上（y=200）。

    ``vy=400`` 是挑出来的：一个切片 0.08 s，K=6 走 192 ⇒ 墙最终停在
    y≈292（在目标上方），于是**终点所在的连通分量既包含终点也包含玩家**，
    终端代价才有意义。``vy=200`` 时墙会正好停在终点上（y≈196），
    终点落到膨胀区里、整个 DP 锚在非法位置 —— 我第一版测试就是这么写错的。
    """
    return FieldView(
        player=[px, py, 0.0, 0.0, 10.0, 200.0, 1.0],
        rects=(Rect(1, 320.0, wall_y, 0.0, vy, 320.0, 4.0, 0.0, 0.0),),
        circles=(), field_w=640.0, field_h=480.0, goal=goal)


def test_layers_are_blocked_masks_not_free_masks() -> None:
    """**语义锁定**：``build_layers`` 返回的 ``True`` 必须是"被占"。

    这条是把 `blocked` 和 `free` 搞反的直接守卫。搞反的症状极其隐蔽：
    倒推 DP 只能从"被占的格"往外传，``V[0]`` 几乎全 ``inf``，于是
    ``build_field`` 判成"目标不可达"而**静默降级**成只走风险场 ——
    实测 ``wall_with_gap`` 因此从 0 碰撞变成 48 碰撞，而 diag 里只看到
    ``fallback=goal_unreachable``，完全看不出是掩膜语义反了。
    """
    v = _moving_wall_view(320.0, 50.0, (320.0, 200.0))
    cfg = FieldConfig(cell=16.0)
    layers, cell, nx, ny = build_layers(v, cfg, steps=3, pred_dt=1 / 30)
    g = build_grid(v, cfg)
    # 第 0 层必须和 build_grid 的 blocked 完全一致
    assert list(layers[0]) == list(g.blocked), "第 0 层应当复用 build_grid"
    # 墙所在那一行必须是 True（被占）
    i, j = g.cell_of(320.0, 100.0)
    assert layers[0][i * ny + j] is True, "墙中心的格应当是 blocked"
    # 墙上方/下方必须是 False
    i2, j2 = g.cell_of(320.0, 300.0)
    assert layers[0][i2 * ny + j2] is False
    # 层数 = steps + 1
    assert len(layers) == 4


def test_time_aware_passes_a_wall_that_moves_away_but_static_cannot() -> None:
    """**时间维存在的理由**：一面会移开的墙。

    墙横贯全场（没有缺口），此刻把玩家和目标隔开。静态波前只能说"不可达"；
    时间扩展的倒推 DP 知道"等它走过去那一格就通了"。
    """
    v = _moving_wall_view(320.0, 50.0, (320.0, 200.0))
    cfg = FieldConfig(cell=16.0, time_steps=6, time_dt=1 / 30)
    g = build_grid(v, cfg)
    start = g.cell_of(320.0, 50.0)
    goal_cell = g.cell_of(320.0, 200.0)

    static = wavefront(g, goal_cell)
    assert not math.isfinite(static[g.index(*start)]), (
        "静态波前应当说不可达（墙横贯全场）")

    speed = 200.0
    eff_dt = max(cfg.time_dt, g.cell / speed)
    layers, _cell, nx, ny = build_layers(v, cfg, steps=cfg.time_steps,
                                         pred_dt=eff_dt)
    arrival = backward_arrival(layers, goal_cell, nx, ny, g.cell,
                               speed=speed, dt=eff_dt)
    a = arrival[g.index(*start)]
    assert math.isfinite(a), "时间维应当找到'等墙走过去'的路"
    assert a > 0.0


def test_time_aware_still_blocks_a_permanent_wall() -> None:
    """反面对照：**不会移开**的墙，时间维也过不去。

    没有这条，上面那条可能只是"时间维什么都说可达"。
    """
    v = FieldView(player=[320.0, 50.0, 0.0, 0.0, 10.0, 200.0, 1.0],
                  rects=(Rect(1, 320.0, 100.0, 0.0, 0.0, 320.0, 4.0, 0.0, 0.0),),
                  circles=(), field_w=640.0, field_h=480.0, goal=(320.0, 200.0))
    cfg = FieldConfig(cell=16.0, time_steps=6, time_dt=1 / 30)
    g = build_grid(v, cfg)
    speed = 200.0
    eff_dt = max(cfg.time_dt, g.cell / speed)
    layers, _c, nx, ny = build_layers(v, cfg, steps=cfg.time_steps, pred_dt=eff_dt)
    arrival = backward_arrival(layers, g.cell_of(320.0, 200.0), nx, ny, g.cell,
                               speed=speed, dt=eff_dt)
    assert not math.isfinite(arrival[g.index(*g.cell_of(320.0, 50.0))]), (
        "永久墙不应当被时间维绕过去")


def test_slice_interval_clamped_to_one_cell_of_travel() -> None:
    """**切片间隔必须 ≥ 走一格的时间。**

    一层之内迈不出一格的话，倒推 DP 全程只能"原地不动"，``V[0]`` 全 ``inf``，
    时间维**静默失效**。实测 cell=16 / speed=200 / dt=1/30 正是这种情况
    （一层只能走 6.67，而最近邻格在 16 处），``wall_with_gap`` 因此从
    0 碰撞变 48 碰撞。

    这里断言：引擎把 dt 放大到 ``cell/speed`` 并在 diag 里写出来。
    """
    v = _moving_wall_view(320.0, 50.0, (320.0, 200.0))
    speed = float(v.player[5])
    for cell in (16.0, 8.0):
        cfg = FieldConfig(cell=cell, time_steps=6, time_dt=1.0 / 120.0,
                          descend_steps=2, recompute=1)
        g = build_grid(v, cfg)
        F, diag = build_field(g, v, cfg, g.cell_of(320.0, 50.0))
        need = g.cell / speed
        assert diag.get("time_dt_clamped_to", 0.0) >= need - 1e-9, (
            f"cell={cell}: dt 没有被放大到 {need:.4f}，diag={diag}")
        assert diag["time_vision_s"] >= need - 1e-9


def test_terminal_cost_reaches_the_whole_field() -> None:
    """**终端代价的作用**：空场里**每个**自由格都必须有值。

    如果把终端写成"只有终点 = 0、其余 inf"，K 步倒推只能覆盖 K 步锥内
    （K=6、一步走 16 ⇒ 半径才 96，而场是 640×480），玩家一旦在锥外就被判成
    "目标不可达"而静默降级。用静态测地距离当初始终端就没这个问题。

    空场是**唯一能精确断言**的场景：时间维在空场里不该改变任何东西，
    所以 ``V[0][p]`` 必须恰好等于 ``静态测地距离(p) / 速度``。
    """
    v = FieldView(player=[320.0, 50.0, 0.0, 0.0, 10.0, 200.0, 1.0],
                  rects=(), circles=(), field_w=640.0, field_h=480.0,
                  goal=(320.0, 240.0))
    cfg = FieldConfig(cell=16.0, time_steps=6, time_dt=1.0 / 30.0)
    g = build_grid(v, cfg)
    speed = float(v.player[5])
    eff = max(cfg.time_dt, g.cell / speed)
    layers, _c, nx, ny = build_layers(v, cfg, steps=cfg.time_steps, pred_dt=eff)
    gc = g.cell_of(320.0, 240.0)
    V = backward_arrival(layers, gc, nx, ny, g.cell, speed=speed, dt=eff)

    free = [k for k in range(nx * ny) if not layers[0][k]]
    assert len(free) == nx * ny, "空场不该有被占的格"
    missing = [k for k in free if not math.isfinite(V[k])]
    assert not missing, f"有 {len(missing)} 个自由格没有值 —— 终端代价没生效"

    static = wavefront_mask(layers[0], nx, ny, g.cell, gc)
    for k in free:
        assert V[k] == pytest.approx(static[k] / speed, rel=1e-9)

    # 最远的角：必须有限，而且明显超过一个切片的行程（证明不是"锥内"）
    corner = g.index(*g.cell_of(632.0, 8.0))
    assert math.isfinite(V[corner])
    assert V[corner] > 3.0 * eff, f"角落只用了 {V[corner]:.2f}s，不像'很远'"


def test_time_aware_goal_cell_may_be_inf_when_a_wall_will_run_it_over() -> None:
    """**反直觉但正确**：``V[0][终点]`` 可以是 ``inf``。

    墙在 t=0.24 s 扫过终点。站在终点不动 ⇒ 那一刻被撞 ⇒
    「从终点出发活到终点」不存在，``V[0][终点] = inf`` 是对的。

    （我第一版测试把它当 bug 了，断言"终点格必须有值"，然后查了半天。
    真正该断言的是**玩家格**有值 —— 也就是下一条测试。）
    """
    v = _moving_wall_view(320.0, 50.0, (320.0, 200.0))
    cfg = FieldConfig(cell=16.0, time_steps=6, time_dt=1.0 / 30.0)
    g = build_grid(v, cfg)
    speed = float(v.player[5])
    eff = max(cfg.time_dt, g.cell / speed)
    layers, _c, nx, ny = build_layers(v, cfg, steps=cfg.time_steps, pred_dt=eff)
    gc = g.cell_of(320.0, 200.0)
    V = backward_arrival(layers, gc, nx, ny, g.cell, speed=speed, dt=eff)
    # 墙确实会在某一层扫过终点 —— 否则这条测试就没意义
    assert any(layers[k][gc[0] * ny + gc[1]] for k in range(len(layers))), \
        "这个构造里墙应当扫过终点"
    assert not math.isfinite(V[g.index(*gc)]), "站在会被扫过的终点上，应当是 inf"
    # 但**玩家**格必须有值：等墙过去再走是可行的
    assert math.isfinite(V[g.index(*g.cell_of(320.0, 50.0))])


def test_time_aware_flag_off_by_default() -> None:
    """默认必须是静态（``time_steps=0``）—— 时间维贵 5 倍，要显式打开。"""
    assert FieldConfig().time_steps == 0
    v = _moving_wall_view(320.0, 50.0, (320.0, 200.0))
    cfg = FieldConfig(cell=16.0)
    g = build_grid(v, cfg)
    _F, diag = build_field(g, v, cfg, g.cell_of(320.0, 50.0))
    assert "time_steps" not in diag

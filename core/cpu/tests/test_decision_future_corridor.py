"""Future Safe Corridor (FSC) shadow tests - v0.6.0-SafeCorridor-Shadow.

Covers SafeRect construction/validity, rectangle algebra, reachability, the
persistent mask, the four toy corridors (stable / dead-end / unstable /
reachability failure), target containment, pre-positioning, shadow neutrality,
deterministic replay, serialization and configuration validation.  Nothing here
may change a v0.5.2 action.
"""

from __future__ import annotations

import json
import pathlib

import numpy as np
import pytest

from cpu.decision import ActionPlanner, ViabilityContext
from cpu.decision import VERSION as DECISION_VERSION
from cpu.decision.future_corridor import (
    CORRIDOR_DEAD_END,
    CORRIDOR_NONE,
    CORRIDOR_STABLE,
    CORRIDOR_TYPES,
    CORRIDOR_UNKNOWN,
    CORRIDOR_UNSTABLE,
    RECT_CENTRE_BLOCKED,
    RECT_DEGENERATE,
    RECT_DIAGONAL_OVERLAP,
    RECT_NO_OBSTACLES,
    RECT_OK,
    SAFE_CORRIDOR_SHADOW_ENABLED,
    AnchorSpace,
    FutureCorridorConfig,
    SafeCorridorShadow,
    bool_to_mask,
    circular_sector_distance,
    classify_corridor,
    descriptor_from_spaces,
    expand_rect,
    mask_direction,
    mask_hamming,
    mask_max_run,
    mask_popcount,
    mask_to_bool,
    persistent_mask,
    rect_reachable,
    rects_intersect,
    safe_rect_from_obstacles,
)
from cpu.decision.models import Action, AgentState, Obstacle, SceneState

CTX = ViabilityContext(short_horizon=0.3, scene_scale=2.4)
ANCHORS = np.array([0.30, 0.55, 0.80])
V_MAX = 10.0
NEAR = (0.0, 10.0, 0.0, 10.0)


def _agent(x=0.0, y=0.0, radius=0.5, max_speed=V_MAX) -> AgentState:
    return AgentState(position=(x, y), velocity=(0.0, 0.0), radius=radius, max_speed=max_speed)


def _scene(obstacles=(), goal=(50.0, 0.0), agent=None) -> SceneState:
    return SceneState(
        agent=agent or _agent(),
        obstacles=tuple(obstacles),
        goal=np.asarray(goal, dtype=np.float64),
        dt=1.0 / 120.0,
        horizon=0.3,
    )


def _obstacle(x, y, vx=0.0, vy=0.0, radius=1.0) -> Obstacle:
    return Obstacle(position=(x, y), velocity=(vx, vy), radius=radius)


def _space(
    mask: int,
    max_run: int,
    run_start: int,
    *,
    clearance: float = 5.0,
    corridor_clearance: float | None = None,
    rect=NEAR,
    rect_valid: bool = True,
    reason: str = RECT_OK,
    anchor: float = 0.30,
) -> AnchorSpace:
    return AnchorSpace(
        anchor=anchor,
        safe_mask=mask,
        max_run=max_run,
        run_start=run_start,
        min_clearance=clearance,
        corridor_clearance=clearance if corridor_clearance is None else corridor_clearance,
        safe_rect_valid=rect_valid,
        rect=rect,
        safe_rect_invalid_reason=reason,
    )


# ==========================================================================
# toy scenarios
# ==========================================================================
def _toy_stable():
    """A: R1 large, R2 medium, R3 smaller but still usable; a direction persists."""
    return (
        _space(0b00000111, 3, 0, clearance=6.0, rect=(0.0, 10.0, 0.0, 10.0), anchor=0.30),
        _space(0b00000111, 3, 0, clearance=5.0, rect=(1.0, 9.0, 1.0, 9.0), anchor=0.55),
        _space(0b00000011, 2, 0, clearance=4.0, rect=(2.0, 8.0, 2.0, 8.0), anchor=0.80),
    )


def _toy_dead_end():
    """B: R1 large, R2 small, R3 tiny and unreachable."""
    return (
        _space(0b00001111, 4, 0, clearance=6.0, rect=(0.0, 10.0, 0.0, 10.0), anchor=0.30),
        _space(0b00000011, 2, 0, clearance=2.0, rect=(0.0, 10.0, 0.0, 10.0), anchor=0.55),
        _space(
            0b00000001,
            1,
            0,
            clearance=0.2,
            corridor_clearance=0.2,
            rect=(200.0, 210.0, 0.0, 10.0),
            anchor=0.80,
        ),
    )


def _toy_unstable():
    """C: masks 11110000 -> 00111100 -> 00001111."""
    return (
        _space(0b11110000, 4, 4, clearance=6.0, anchor=0.30),
        _space(0b00111100, 4, 2, clearance=5.0, anchor=0.55),
        _space(0b00001111, 4, 0, clearance=4.0, anchor=0.80),
    )


def _toy_unreachable():
    """D: all three regions exist, but the speed budget cannot connect them."""
    return (
        _space(0b00000111, 3, 0, clearance=6.0, rect=(0.0, 10.0, 0.0, 10.0), anchor=0.30),
        _space(0b00000111, 3, 0, clearance=5.0, rect=(100.0, 110.0, 0.0, 10.0), anchor=0.55),
        _space(0b00000011, 2, 0, clearance=4.0, rect=(200.0, 210.0, 0.0, 10.0), anchor=0.80),
    )


# ==========================================================================
# 1-2. SafeRect construction and invalidity
# ==========================================================================
def test_safe_rect_construction_from_directional_clearance():
    # one obstacle to the right: xmax is pinned to its inflated left edge
    valid, rect, reason = safe_rect_from_obstacles(
        (0.0, 0.0),
        np.array([[10.0, 0.0]]),
        np.array([1.0]),
        agent_radius=0.5,
        buffer=0.0,
        max_half=50.0,
        min_half=0.5,
    )
    assert valid is True and reason == RECT_OK
    xmin, xmax, ymin, ymax = rect
    assert xmax == pytest.approx(8.5)          # 10 - (1.0 + 0.5)
    assert xmin == pytest.approx(-50.0)        # unbounded left -> cap
    assert ymin == pytest.approx(-50.0) and ymax == pytest.approx(50.0)
    assert xmin <= 0.0 <= xmax and ymin <= 0.0 <= ymax   # centre stays inside

    # empty scene -> valid, capped, explicit reason
    ok, rect_e, reason_e = safe_rect_from_obstacles(
        (1.0, 2.0), np.zeros((0, 2)), np.zeros(0), agent_radius=0.5, max_half=50.0
    )
    assert ok is True and reason_e == RECT_NO_OBSTACLES
    assert rect_e == (-49.0, 51.0, -48.0, 52.0)


def test_safe_rect_invalid_reasons():
    # an obstacle covering the centre
    blocked, rect_b, reason_b = safe_rect_from_obstacles(
        (0.0, 0.0), np.array([[0.0, 0.0]]), np.array([1.0]), agent_radius=0.5, max_half=50.0
    )
    assert blocked is False and reason_b == RECT_CENTRE_BLOCKED
    assert rect_b[0] <= 0.0 <= rect_b[1]     # still a rectangle, just not proven safe

    # a purely diagonal obstacle: the row/column rule cannot see it -> verified out
    diag, _rect_d, reason_d = safe_rect_from_obstacles(
        (0.0, 0.0), np.array([[5.0, 5.0]]), np.array([2.0]), agent_radius=0.5, max_half=50.0
    )
    assert diag is False and reason_d == RECT_DIAGONAL_OVERLAP

    # a valid-but-too-small box is rejected as degenerate
    small, _rect_s, reason_s = safe_rect_from_obstacles(
        (0.0, 0.0),
        np.array([[10.0, 0.0]]),
        np.array([1.0]),
        agent_radius=0.5,
        max_half=50.0,
        min_half=30.0,
    )
    assert small is False and reason_s == RECT_DEGENERATE


def test_safe_rect_never_contains_an_inflated_obstacle():
    rng = np.random.default_rng(7)
    for _ in range(30):
        n = int(rng.integers(0, 12))
        positions = rng.uniform(-12.0, 12.0, size=(n, 2))
        radii = rng.uniform(0.3, 1.8, size=n)
        center = rng.uniform(-6.0, 6.0, size=2)
        valid, (xmin, xmax, ymin, ymax), _reason = safe_rect_from_obstacles(
            center, positions, radii, agent_radius=0.5, max_half=30.0
        )
        if not valid:
            continue
        inflate = radii + 0.5
        overlap = (
            (positions[:, 0] - inflate < xmax)
            & (positions[:, 0] + inflate > xmin)
            & (positions[:, 1] - inflate < ymax)
            & (positions[:, 1] + inflate > ymin)
        )
        assert not bool(overlap.any())
        assert xmin <= center[0] <= xmax and ymin <= center[1] <= ymax


# ==========================================================================
# 3-4. rectangle expand / intersect
# ==========================================================================
def test_expand_rect_is_symmetric():
    assert expand_rect(NEAR, 2.5) == (-2.5, 12.5, -2.5, 12.5)
    assert expand_rect((1.0, 2.0, 3.0, 4.0), 0.0) == (1.0, 2.0, 3.0, 4.0)


def test_rects_intersect_cases():
    assert rects_intersect(NEAR, (5.0, 15.0, 5.0, 15.0)) is True
    assert rects_intersect(NEAR, (20.0, 30.0, 0.0, 10.0)) is False
    assert rects_intersect(NEAR, (0.0, 10.0, 20.0, 30.0)) is False
    assert rects_intersect(NEAR, (10.0, 20.0, 10.0, 20.0)) is True      # touching
    assert rects_intersect(NEAR, NEAR) is True


# ==========================================================================
# 5-6. reach12 / reach23
# ==========================================================================
def test_reachability_budget_is_v_max_times_dt():
    assert rect_reachable(NEAR, (12.0, 20.0, 0.0, 10.0), V_MAX, 0.25) is True
    assert rect_reachable(NEAR, (20.0, 30.0, 0.0, 10.0), V_MAX, 0.25) is False
    assert rect_reachable(NEAR, (5.0, 15.0, 0.0, 10.0), 0.0, 0.0) is True


def test_descriptor_reach_flags_follow_the_rects():
    d = descriptor_from_spaces(_toy_stable(), ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.reach_12 is True and d.reach_23 is True

    far = list(_toy_stable())
    far[2] = _space(0b00000011, 2, 0, clearance=4.0, rect=(30.0, 40.0, 30.0, 40.0))
    d_far = descriptor_from_spaces(far, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d_far.reach_12 is True and d_far.reach_23 is False

    broken = list(_toy_stable())
    broken[1] = _space(0b00000111, 3, 0, clearance=5.0, rect=NEAR, rect_valid=False)
    d_broken = descriptor_from_spaces(broken, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d_broken.reach_12 is False and d_broken.reach_23 is False


# ==========================================================================
# 7. persistent mask and overlaps
# ==========================================================================
def test_persistent_mask_and_overlap_popcounts():
    assert persistent_mask(0b11110000, 0b00111100, 0b00001111) == 0b00000000
    assert persistent_mask(0b00000111, 0b00000111, 0b00000011) == 0b00000011
    assert mask_popcount(0b00000000) == 0
    assert mask_popcount(0b10101010) == 4
    assert mask_popcount(0b11111111) == 8

    unstable = descriptor_from_spaces(
        _toy_unstable(), ANCHORS, V_MAX, FutureCorridorConfig()
    )
    assert unstable.persistent_mask == 0
    assert unstable.persistent_sectors == 0
    assert unstable.overlap_12 == mask_popcount(0b11110000 & 0b00111100) == 2
    assert unstable.overlap_23 == mask_popcount(0b00111100 & 0b00001111) == 2

    stable = descriptor_from_spaces(_toy_stable(), ANCHORS, V_MAX, FutureCorridorConfig())
    assert stable.persistent_mask == 0b00000011
    assert stable.persistent_sectors == 2


def test_mask_run_direction_and_hamming_helpers():
    assert mask_max_run(0b00000000) == (0, 0)
    assert mask_max_run(0b11111111) == (8, 0)
    assert mask_max_run(0b00000111) == (3, 0)
    assert mask_max_run(0b10000001) == (2, 7)      # wraps across sector 7 -> 0
    assert mask_direction(0b00000111, 3, 0) == 1
    assert mask_direction(0b11110000, 4, 4) == 5
    assert mask_direction(0b00000000, 0, 0) == -1
    assert mask_hamming(0b00000111, 0b00000011) == 1
    assert circular_sector_distance(0, 7) == 1
    assert circular_sector_distance(1, 6) == 3


# ==========================================================================
# 8. stable corridor toy case
# ==========================================================================
def test_toy_stable_corridor():
    spaces = _toy_stable()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_STABLE
    assert d.reach_12 and d.reach_23
    assert d.persistent_sectors >= 1
    assert d.corridor_match_count == 3
    # narrowing is not itself a failure
    assert spaces[0].max_run > spaces[2].max_run


# ==========================================================================
# 9. dead-end toy case
# ==========================================================================
def test_toy_dead_end_corridor():
    d = descriptor_from_spaces(_toy_dead_end(), ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_DEAD_END
    assert d.per_anchor[2].max_run < FutureCorridorConfig().min_run


def test_dead_end_when_t3_clearance_is_below_the_floor():
    spaces = list(_toy_stable())
    spaces[2] = _space(
        0b00000011, 2, 0, clearance=0.2, corridor_clearance=0.2, rect=(2.0, 8.0, 2.0, 8.0)
    )
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.reach_23 is True                  # geometrically reachable ...
    assert d.corridor_type == CORRIDOR_DEAD_END   # ... but no usable exit


def test_rapid_collapse_without_a_persistent_direction_is_dead_end():
    spaces = (
        _space(0b00001111, 4, 0, clearance=6.0, rect=NEAR),
        _space(0b00001100, 2, 2, clearance=4.0, rect=NEAR),
        _space(0b00000011, 2, 0, clearance=3.0, rect=NEAR),
    )
    cfg = FutureCorridorConfig()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, cfg)
    assert d.persistent_mask == 0
    assert (spaces[0].max_run - spaces[2].max_run) >= cfg.rapid_shrink_run_drop
    assert d.corridor_type == CORRIDOR_DEAD_END


def test_no_space_now_is_none_not_dead_end():
    spaces = list(_toy_stable())
    spaces[0] = _space(0b00000000, 0, 0, clearance=0.0, corridor_clearance=0.0)
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_NONE


# ==========================================================================
# 10. unstable toy case
# ==========================================================================
def test_toy_unstable_corridor():
    spaces = _toy_unstable()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_UNSTABLE
    assert d.reach_12 and d.reach_23          # reachable, yet still unstable
    assert d.persistent_mask == 0


def test_unstable_by_low_overlap():
    spaces = (
        _space(0b00001111, 4, 0),
        _space(0b11110000, 4, 4),
        _space(0b00001111, 4, 0),
    )
    cfg = FutureCorridorConfig()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, cfg)
    assert d.overlap_12 == 0 and d.overlap_23 == 0
    assert d.persistent_mask == 0
    assert d.corridor_type == CORRIDOR_UNSTABLE


def test_classify_requires_three_anchors():
    with pytest.raises(ValueError):
        classify_corridor(_toy_stable()[:2], True, True, FutureCorridorConfig())


# ==========================================================================
# 11. reachability failure toy case (scenario D)
# ==========================================================================
def test_toy_reachability_failure_is_not_a_corridor():
    spaces = _toy_unreachable()
    cfg = FutureCorridorConfig()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, cfg)

    # every region exists and is individually valid ...
    assert all(s.safe_rect_valid for s in spaces)
    assert all(s.max_run >= cfg.min_run for s in spaces)
    assert d.persistent_sectors >= 1
    # ... but the agent cannot arrive in time
    assert d.reach_12 is False and d.reach_23 is False
    assert d.corridor_type in (CORRIDOR_DEAD_END, CORRIDOR_NONE)
    assert d.corridor_type == CORRIDOR_DEAD_END


def test_reachability_failure_only_on_the_second_leg():
    spaces = (
        _space(0b00000111, 3, 0, rect=(0.0, 10.0, 0.0, 10.0)),
        _space(0b00000111, 3, 0, rect=(1.0, 9.0, 1.0, 9.0)),
        _space(0b00000011, 2, 0, rect=(500.0, 510.0, 0.0, 10.0)),
    )
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.reach_12 is True and d.reach_23 is False
    assert d.corridor_type == CORRIDOR_DEAD_END


# ==========================================================================
# 12. future target
# ==========================================================================
def test_future_target_lies_inside_a_valid_r3():
    d = descriptor_from_spaces(_toy_stable(), ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.future_target_valid is True
    assert d.future_target_source == "rect_center"
    xmin, xmax, ymin, ymax = d.per_anchor[2].rect
    assert xmin <= d.future_target_x <= xmax
    assert ymin <= d.future_target_y <= ymax


def test_future_target_invalid_without_a_valid_r3():
    spaces = list(_toy_stable())
    spaces[2] = _space(
        0b00000011, 2, 0, clearance=4.0, rect=(2.0, 8.0, 2.0, 8.0), rect_valid=False
    )
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.future_target_valid is False and d.future_target_source == "none"

    spaces2 = list(_toy_stable())
    spaces2[2] = _space(
        0b00000011, 2, 0, clearance=0.2, corridor_clearance=0.2, rect=(2.0, 8.0, 2.0, 8.0)
    )
    d2 = descriptor_from_spaces(spaces2, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d2.future_target_valid is False


# ==========================================================================
# 13. corridor match (pre-positioning)
# ==========================================================================
def test_corridor_match_checks_containment():
    spaces = _toy_stable()
    inside = ((5.0, 5.0), (5.0, 5.0), (5.0, 5.0))
    d_in = descriptor_from_spaces(
        spaces, ANCHORS, V_MAX, FutureCorridorConfig(), future_positions=inside
    )
    assert d_in.corridor_match == (True, True, True)
    assert d_in.corridor_match_count == 3
    assert d_in.corridor_match_t1 and d_in.corridor_match_t2 and d_in.corridor_match_t3

    outside = ((5.0, 5.0), (50.0, 50.0), (5.0, 5.0))
    d_out = descriptor_from_spaces(
        spaces, ANCHORS, V_MAX, FutureCorridorConfig(), future_positions=outside
    )
    assert d_out.corridor_match == (True, False, True)
    assert d_out.corridor_match_count == 2


def test_real_candidate_prepositioning_counts():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, radius=1.2)])
    d = SafeCorridorShadow(enabled=True).evaluate(scene, Action(V_MAX, 0.0), CTX)
    assert len(d.corridor_match) == 3
    assert d.corridor_match_count == sum(d.corridor_match)
    assert all(isinstance(m, bool) for m in d.corridor_match)


# ==========================================================================
# 14 / 18. shadow never changes the action
# ==========================================================================
def test_shadow_does_not_change_the_action():
    scenes = [
        _scene([_obstacle(6.0, 0.0, vx=-8.0)]),
        _scene([_obstacle(5.0, 1.0), _obstacle(7.0, -1.0)]),
        _scene([_obstacle(4.0, 0.0, vx=-2.0), _obstacle(9.0, 2.0, radius=1.5)]),
        _scene([_obstacle(0.0, 0.0, radius=5.0)]),      # emergency frame
    ]
    plain = ActionPlanner(horizon=0.3)
    plain_actions = [plain.plan(s, context=CTX) for s in scenes]

    shadow = SafeCorridorShadow(enabled=True)
    watched = ActionPlanner(horizon=0.3)
    watched_actions = []
    for scene in scenes:
        action = watched.plan(scene, context=CTX)
        shadow.observe(
            scene,
            watched.candidate_actions(scene),
            CTX,
            chosen_index=watched.last_diagnostics.chosen_index,
            v052_diagnostics=watched.last_diagnostics,
        )
        watched_actions.append(action)

    assert plain_actions == watched_actions
    assert DECISION_VERSION == "v0.6.0-PredictiveCorridorSelector"


def test_shadow_disabled_by_default_and_computes_nothing():
    assert SAFE_CORRIDOR_SHADOW_ENABLED is False
    shadow = SafeCorridorShadow()
    assert shadow.enabled is False
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    assert shadow.observe(scene, (Action(V_MAX, 0.0),), CTX, chosen_index=0) is None
    assert SafeCorridorShadow(enabled=True).observe(
        scene, (Action(V_MAX, 0.0),), CTX, chosen_index=0
    ) is not None


def test_corridor_machinery_is_gated_in_the_planner():
    """v0.6.0: the corridor module is now a real selector input, not shadow-only,
    but the switch is OFF by default so the v0.5.2 action path is untouched."""
    base = pathlib.Path(__file__).resolve().parents[1] / "decision"   # 测试与模块同层：core/cpu/tests -> core/cpu
    planner_src = (base / "planner.py").read_text(encoding="utf-8")
    assert "future_corridor" in planner_src          # real selector now
    assert "FSC_SELECTOR_ENABLED" in planner_src
    assert ActionPlanner(horizon=0.3).fsc_selector_enabled is False
    # core.py must still not pull in the corridor machinery
    assert "future_corridor" not in (base / "core.py").read_text(encoding="utf-8")


# ==========================================================================
# 15. deterministic replay
# ==========================================================================
def test_deterministic_replay_is_identical():
    scene = _scene(
        [
            _obstacle(6.0, 0.0, vx=-8.0),
            _obstacle(9.0, 2.0, vx=-4.0),
            _obstacle(12.0, -2.0, vx=-1.0, radius=1.4),
        ]
    )
    actions = tuple(Action(V_MAX, np.radians(45 * i)) for i in range(8))
    shadow = SafeCorridorShadow(enabled=True)

    first = shadow.observe(scene, actions, CTX, chosen_index=2, k=4)
    second = shadow.observe(scene, actions, CTX, chosen_index=2, k=4)
    assert first.chosen.to_dict() == second.chosen.to_dict()
    assert first.dss.to_dict() == second.dss.to_dict()
    assert first.evaluated_indices == second.evaluated_indices
    assert [d.to_dict() for d in first.per_candidate] == [
        d.to_dict() for d in second.per_candidate
    ]

    third = SafeCorridorShadow(enabled=True).observe(
        scene, actions, CTX, chosen_index=2, k=4
    )
    assert third.chosen.to_dict() == first.chosen.to_dict()


# ==========================================================================
# 16. diagnostics serialization
# ==========================================================================
def test_shadow_frame_and_dss_are_json_serializable():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    actions = tuple(Action(V_MAX, np.radians(45 * i)) for i in range(17))
    frame = SafeCorridorShadow(enabled=True).observe(
        scene, actions, CTX, chosen_index=3, k=4, build_id="cpudec-test"
    )
    payload = frame.to_dict()
    text = json.dumps(payload)                 # strict JSON
    assert json.loads(text)["candidate_count"] == 4

    assert frame.candidate_count == 4          # K <= 4
    assert frame.evaluated_indices[0] == 3
    total = (
        frame.stable_funnel_count
        + frame.dead_end_funnel_count
        + frame.unstable_funnel_count
        + frame.none_funnel_count
        + frame.unknown_funnel_count
    )
    assert total == frame.candidate_count

    dss = frame.dss.to_dict()
    assert set(dss) == {"header", "global_safety", "candidate_risk", "future_space"}
    assert dss["header"]["shadow_only"] is True
    assert dss["header"]["version"] == "v0.6.0-SafeCorridor-Shadow"
    assert "t3_clearance_floor" in dss["header"]["thresholds"]
    assert len(dss["candidate_risk"]) == 17
    assert len(dss["future_space"]) == 17
    assert all(len(row) == 3 for row in dss["future_space"])
    assert "safe_rect_invalid_reason" in dss["future_space"][3][0]
    chosen_risk = dss["candidate_risk"][3]
    assert chosen_risk["fsc_corridor_type"] in CORRIDOR_TYPES
    assert isinstance(chosen_risk["fsc_persistent_mask"], int)

    # FutureSpace rows carry corridor_type as well, so a consumer can tell
    # "algorithm does not know" (UNKNOWN) from "algorithm proved dead end"
    assert chosen_risk["corridor_type"] == chosen_risk["fsc_corridor_type"]
    for k in range(3):
        assert dss["future_space"][3][k]["corridor_type"] == chosen_risk["corridor_type"]
        assert dss["future_space"][3][k]["corridor_type"] in CORRIDOR_TYPES
        assert "safe_rect_invalid_reason" in dss["future_space"][3][k]
    # unevaluated candidates keep an empty row rather than a fake verdict
    assert dss["future_space"][0][0] == {}

    # thresholds are echoed rather than hidden in the code
    assert payload["chosen"]["thresholds"]["min_run"] == 2


def test_frame_timings_are_present_and_non_negative():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])
    frame = SafeCorridorShadow(enabled=True).observe(
        scene, (Action(V_MAX, 0.0),), CTX, chosen_index=0
    )
    for name in ("safe_rect_us", "reachability_us", "corridor_us", "total_shadow_us"):
        assert getattr(frame, name) >= 0.0
    assert frame.fsc_compute_us == frame.safe_rect_us        # legacy alias
    assert frame.diagnostics_us == frame.total_shadow_us     # legacy alias


# ==========================================================================
# 17. configuration validation
# ==========================================================================
def test_config_is_validated_and_echoed():
    cfg = FutureCorridorConfig()
    assert cfg.to_dict()["t3_clearance_floor"] == cfg.t3_clearance_floor
    assert cfg.to_dict()["unstable_min_overlap"] == cfg.unstable_min_overlap

    with pytest.raises(ValueError):
        FutureCorridorConfig(anchors=(0.3, 0.4))
    with pytest.raises(ValueError):
        FutureCorridorConfig(anchors=(0.5, 0.4, 0.8))
    with pytest.raises(ValueError):
        FutureCorridorConfig(min_run=0)
    with pytest.raises(ValueError):
        FutureCorridorConfig(min_rect_half=5.0, max_rect_half=1.0)
    with pytest.raises(ValueError):
        FutureCorridorConfig(rapid_shrink_run_drop=-1)

    # a threshold change must change the classification, proving it is wired
    spaces = list(_toy_stable())
    spaces[2] = _space(
        0b00000011, 2, 0, clearance=1.5, corridor_clearance=1.5, rect=(2.0, 8.0, 2.0, 8.0)
    )
    strict = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    loose = descriptor_from_spaces(
        spaces, ANCHORS, V_MAX, FutureCorridorConfig(clearance_threshold=1.0)
    )
    assert strict.corridor_type == CORRIDOR_NONE
    assert loose.corridor_type == CORRIDOR_STABLE


def test_anchors_match_the_v052_medium_anchor_times():
    assert tuple(np.round(SafeCorridorShadow().anchors(CTX), 2)) == (0.30, 0.55, 0.80)


# ==========================================================================
# v0.6.0 fix 1: "cannot prove safe" is UNKNOWN, never DEAD_END
# ==========================================================================
def _toy_unknown():
    """E: masks show space, but R2 cannot be proven."""
    return (
        _space(0b00000111, 3, 0, clearance=6.0, rect=NEAR, anchor=0.30),
        _space(
            0b00000111,
            3,
            0,
            clearance=5.0,
            rect=NEAR,
            rect_valid=False,
            reason=RECT_CENTRE_BLOCKED,
            anchor=0.55,
        ),
        _space(0b00000011, 2, 0, clearance=4.0, rect=NEAR, anchor=0.80),
    )


def test_toy_unknown_when_safe_rect_is_invalid():
    d = descriptor_from_spaces(_toy_unknown(), ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_UNKNOWN
    assert d.corridor_type != CORRIDOR_DEAD_END
    # the reason is preserved so UNKNOWN and DEAD_END stay distinguishable
    assert d.per_anchor[1].safe_rect_invalid_reason == RECT_CENTRE_BLOCKED
    assert d.per_anchor[1].safe_rect_valid is False


def test_unknown_is_not_dead_end_even_though_reach_flags_are_false():
    spaces = _toy_unknown()
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    # an unprovable rectangle makes the reach flags conservatively False ...
    assert d.reach_12 is False and d.reach_23 is False
    # ... yet the verdict is "we do not know", not "we proved a dead end"
    assert d.corridor_type == CORRIDOR_UNKNOWN


def test_unknown_never_appears_when_all_rects_are_valid():
    for spaces in (_toy_stable(), _toy_dead_end(), _toy_unstable(), _toy_unreachable()):
        d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
        assert d.corridor_type != CORRIDOR_UNKNOWN
    assert CORRIDOR_UNKNOWN in CORRIDOR_TYPES


def test_empty_no_evidence_is_none_not_unknown():
    spaces = (
        _space(0b00000000, 0, 0, clearance=0.0, corridor_clearance=0.0, rect=NEAR),
        _space(0b00000000, 0, 0, clearance=0.0, corridor_clearance=0.0, rect=NEAR),
        _space(0b00000000, 0, 0, clearance=0.0, corridor_clearance=0.0, rect=NEAR),
    )
    d = descriptor_from_spaces(spaces, ANCHORS, V_MAX, FutureCorridorConfig())
    assert d.corridor_type == CORRIDOR_NONE


def test_real_geometry_produces_unknown_for_an_unprovable_rect():
    # a diagonal obstacle makes the conservative rectangle unprovable at some
    # anchor; the verdict must be UNKNOWN rather than a fabricated dead end
    scene = _scene(
        [
            _obstacle(3.0, 3.0, radius=2.0),
            _obstacle(3.0, -3.0, radius=2.0),
            _obstacle(6.0, 0.0, radius=2.0),
        ]
    )
    d = SafeCorridorShadow(enabled=True).evaluate(scene, Action(V_MAX, 0.0), CTX)
    if not all(d.safe_rect_valids):
        assert d.corridor_type == CORRIDOR_UNKNOWN
        assert any(
            not s.safe_rect_valid and s.safe_rect_invalid_reason != RECT_OK
            for s in d.per_anchor
        )


# ==========================================================================
# v0.6.0 fix 2: the planner's REAL Top-K is exposed and used
# ==========================================================================
def test_true_planner_topk_matches_fsc_candidate_list():
    planner = ActionPlanner(horizon=0.3)
    captured: dict = {}
    original = planner.viability.shortlist

    def spy(actions, short_term_cost, critical_band, feasible):
        mask = original(actions, short_term_cost, critical_band, feasible)
        captured["actions"] = tuple(actions)
        captured["mask"] = np.array(mask)
        return mask

    planner.viability.shortlist = spy
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0), _obstacle(9.0, 2.0, vx=-4.0)])
    planner.plan(scene, context=CTX)
    diag = planner.last_diagnostics
    assert captured, "the LTV shortlist must have run on this scene"

    expected = [i for i, m in enumerate(captured["mask"]) if m]
    assert set(diag.fsc_candidate_indices) == set(expected)
    assert sorted(diag.fsc_candidate_indices) == sorted(expected)
    assert len(diag.fsc_candidate_indices) <= 4
    assert diag.fsc_candidate_source == "ltv_shortlist"
    assert tuple(diag.fsc_candidate_actions) == tuple(
        captured["actions"][i] for i in diag.fsc_candidate_indices
    )
    assert len(diag.fsc_candidate_actions) == len(diag.fsc_candidate_indices)
    assert len(diag.fsc_candidate_sources) == len(diag.fsc_candidate_indices)
    assert set(diag.fsc_candidate_sources) <= {"leader", "diversity"}
    assert diag.fsc_candidate_sources.count("leader") >= 1

    # the shadow prefers the planner's list over its own proxy
    frame = SafeCorridorShadow(enabled=True).observe(
        scene,
        captured["actions"],
        CTX,
        chosen_index=diag.chosen_index,
        v052_diagnostics=diag,
        candidate_indices=list(diag.fsc_candidate_indices),
    )
    assert frame.candidate_source == "planner_topk"
    assert frame.topk_indices == tuple(diag.fsc_candidate_indices)
    assert set(diag.fsc_candidate_indices) <= set(frame.evaluated_indices)
    assert len(frame.topk_indices) <= 4


def test_no_planner_shortlist_is_reported_honestly():
    scene = _scene([_obstacle(6.0, 0.0, vx=-8.0)])

    disabled = ActionPlanner(horizon=0.3, enable_ltv=False)
    disabled.plan(scene, context=CTX)
    diag = disabled.last_diagnostics
    assert diag.fsc_candidate_indices == ()
    assert diag.fsc_candidate_actions == ()
    assert diag.fsc_candidate_sources == ()
    assert diag.fsc_candidate_source == "ltv_disabled"

    skipped = ActionPlanner(horizon=0.3)
    skipped._fast_path = lambda *a, **k: (True, "fast_path:forced", "")
    skipped.plan(scene, context=CTX)
    assert skipped.last_diagnostics.fsc_candidate_source == "ltv_skipped"
    assert skipped.last_diagnostics.fsc_candidate_indices == ()

    frame = SafeCorridorShadow(enabled=True).observe(
        scene,
        skipped.candidate_actions(scene),
        CTX,
        chosen_index=0,
        v052_diagnostics=skipped.last_diagnostics,
        candidate_indices=list(skipped.last_diagnostics.fsc_candidate_indices),
    )
    assert frame.candidate_source == "shadow_proxy"      # says so, does not guess
    assert frame.topk_indices != ()


def test_shadow_on_off_episode_is_identical():
    """The hard regression: shadow on vs off must not change a single outcome."""
    import benchmark_decision as bench
    from tools.fsc_shadow_probe import run_episode_shadow

    frames: list[dict] = []
    on = run_episode_shadow(bench, 42, k=4, max_steps=120, frames=frames, enabled=True)
    assert frames, "shadow on must produce per-frame diagnostics"
    off = run_episode_shadow(bench, 42, k=4, max_steps=120, frames=[], enabled=False)

    assert on["survived"] == off["survived"]
    assert on["collision"] == off["collision"]
    assert on["changes"] == off["changes"]
    assert on["failure"] == off["failure"]
    assert off["records"] == 0


def test_shadow_on_off_planner_actions_are_identical():
    scenes = [
        _scene([_obstacle(6.0, 0.0, vx=-8.0)]),
        _scene([_obstacle(5.0, 1.0), _obstacle(7.0, -1.0)]),
        _scene([_obstacle(4.0, 0.0, vx=-2.0), _obstacle(9.0, 2.0, radius=1.5)]),
        _scene([_obstacle(0.0, 0.0, radius=5.0)]),
    ]
    off = ActionPlanner(horizon=0.3)
    on = ActionPlanner(horizon=0.3)
    shadow = SafeCorridorShadow(enabled=True)
    for scene in scenes:
        a_off = off.plan(scene, context=CTX)
        a_on = on.plan(scene, context=CTX)
        assert a_off == a_on
        assert (
            off.last_diagnostics.chosen_index == on.last_diagnostics.chosen_index
        )
        assert (
            off.last_diagnostics.fsc_candidate_indices
            == on.last_diagnostics.fsc_candidate_indices
        )
        shadow.observe(
            scene,
            on.candidate_actions(scene),
            CTX,
            chosen_index=on.last_diagnostics.chosen_index,
            candidate_indices=list(on.last_diagnostics.fsc_candidate_indices),
            v052_diagnostics=on.last_diagnostics,
        )
    # and the shadow run's own actions stayed identical to the unobserved run
    for scene in scenes:
        assert off.plan(scene, context=CTX) == on.plan(scene, context=CTX)


def test_safe_mask_geometry_and_bit_helpers():
    scene = _scene([_obstacle(4.6, 0.0)])
    d = SafeCorridorShadow(enabled=True).evaluate(scene, Action(V_MAX, 0.0), CTX)
    t1 = d.per_anchor[0]
    assert t1.safe_mask == 0b11111110
    assert t1.max_run == 7 and t1.run_start == 1
    assert t1.direction == 4
    bits = mask_to_bool(t1.safe_mask)
    assert bool(bits[0]) is False and bool(bits[4]) is True
    assert bool_to_mask(bits) == t1.safe_mask

    empty = SafeCorridorShadow(enabled=True).evaluate(_scene(), Action(V_MAX, 0.0), CTX)
    assert empty.per_anchor[0].safe_mask == 0xFF
    assert empty.per_anchor[0].max_run == 8
    assert empty.corridor_type == CORRIDOR_STABLE

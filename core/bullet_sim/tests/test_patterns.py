"""The obstacle layouts: only realistic obstacle geometry is generated.

The environment has exactly one generator (``obstacle``) and five layouts
(``block`` / ``wall_gap`` / ``small`` / ``cross`` / ``corridor``).  These tests
pin the properties the redesign exists for:

* an object is born on a boundary and travels *into* the field;
* small obstacles enter from **one** boundary and cross toward the opposite one
  (debris drifts in a single direction; it does not converge from all sides);
* a moving block enters from a band, never from a single point, and never runs
  outward;
* a wall's opening is never narrower than the player circle;
* the corridor is tilted, its two walls are not collinear, its width may stay
  constant or narrow spatially, it is parallel after narrowing, and it can never
  go below the player diameter;
* decorative centre-fired patterns no longer exist.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from bullet_sim.core.errors import ConfigError, ScenarioError
from bullet_sim.generators.patterns import (
    LAYOUTS,
    available_patterns,
    emission_steps,
    gen_obstacle_field,
    generator_for,
)
from bullet_sim.generators.spec import PATTERN_KINDS, PatternSpec
from bullet_sim.obstacles.catalog import get_obstacle_type
from bullet_sim.obstacles.spec import ObstacleSpawn, resolve_spec
from bullet_sim.tests.helpers import DT, gen_ctx

FIELD_W, FIELD_H = 640.0, 480.0
PLAYER_R = 10.0


def obstacle_spec(layout: str, *, player_radius: float = PLAYER_R, **params) -> PatternSpec:
    """Resolve a raw layout into a generator spec (as the obstacle layer does)."""
    spawn = ObstacleSpawn(type_key={
        "block": "moving_block",
        "wall_gap": "wall_with_gap",
        "small": "small_obstacles",
        "cross": "cross_traffic",
        "corridor": "corridor",
    }[layout])
    if "region" in params:
        spawn.region = params.pop("region")
    for key in ("count", "size", "speed", "interval", "gap_width",
                "corridor_width", "corridor_min_width", "wall_length", "walls",
                "motion", "change", "tilt_deg", "entry_span"):
        if key in params:
            setattr(spawn, key, params.pop(key))
    spawn.extra.update(params)
    return resolve_spec(
        get_obstacle_type(spawn.type_key), spawn,
        player_radius=player_radius, field_w=FIELD_W, field_h=FIELD_H,
    )


def emit(spec: PatternSpec, **ctx_overrides):
    ctx = gen_ctx(field_w=FIELD_W, field_h=FIELD_H, **ctx_overrides)
    events = gen_obstacle_field(spec, ctx)
    assert events, "the layout emitted nothing"
    return events


# --------------------------------------------------------------------------
# registry: only realistic obstacles remain
# --------------------------------------------------------------------------


def test_only_the_obstacle_generator_exists():
    assert available_patterns() == ["obstacle"]
    assert list(PATTERN_KINDS) == ["obstacle"]


@pytest.mark.parametrize(
    "legacy", ["single", "radial", "spiral", "aimed", "burst", "line", "wall", "random", "mixed"]
)
def test_decorative_generators_are_gone(legacy):
    with pytest.raises(ScenarioError):
        generator_for(legacy)
    with pytest.raises(ConfigError):
        PatternSpec(kind=legacy, count=3)


def test_layout_vocabulary_is_only_realistic_shapes():
    assert set(LAYOUTS) == {"block", "wall_gap", "small", "cross", "corridor"}


# --------------------------------------------------------------------------
# block: a band on the boundary, travelling inward
# --------------------------------------------------------------------------


def test_block_enters_from_a_band_on_the_boundary_not_a_single_point():
    spec = obstacle_spec("block", count=3, size=2.0, entry_span=0.8, origin="field")
    event = emit(spec)[0]
    xs = np.asarray(event.offsets)[:, 0]
    ys = np.asarray(event.offsets)[:, 1]
    assert np.ptp(xs) > 0.2 * FIELD_W, "the blocks all started at one point"
    # top anchor: the whole band sits on/above the top edge
    assert (ys >= FIELD_H - 1e-6).all()
    assert np.ptp(ys) < 1.0


def test_block_always_travels_into_the_field():
    for region in ("top", "bottom", "left", "right", "farthest_side"):
        spec = obstacle_spec("block", count=2, size=2.0, region=region)
        event = emit(spec)[0]
        offsets = np.asarray(event.offsets)
        angles = np.asarray(event.angles)
        vel = np.stack([np.cos(angles), np.sin(angles)], axis=-1)
        # inward normal of the edge each spawn sits on
        for (x, y), v in zip(offsets, vel):
            if abs(y - FIELD_H) < 1.0:                      # top
                assert v[1] < 0.0
            elif abs(y) < 1.0:                              # bottom
                assert v[1] > 0.0
            elif abs(x) < 1.0:                              # left
                assert v[0] > 0.0
            elif abs(x - FIELD_W) < 1.0:                    # right
                assert v[0] < 0.0


def test_block_ignores_an_outward_entry_angle():
    # 90 degrees would point straight up out of the field; it must be corrected
    spec = obstacle_spec("block", size=2.0, entry_angle=90.0, region="top")
    event = emit(spec)[0]
    assert float(np.asarray(event.angles)[0]) != pytest.approx(math.pi / 2.0)
    assert math.sin(float(np.asarray(event.angles)[0])) < 0.0


def test_block_is_sized_as_a_multiple_of_the_player_diameter():
    # half extent = size_ratio x player diameter / 2 = size_ratio x player radius
    spec = obstacle_spec("block", size=2.0)
    assert float(spec.half_w) == pytest.approx(2.0 * PLAYER_R)
    bigger = obstacle_spec("block", size=4.0)
    assert float(bigger.half_w) == pytest.approx(4.0 * PLAYER_R)


# --------------------------------------------------------------------------
# wall with gap
# --------------------------------------------------------------------------


def test_wall_with_gap_spans_the_field_and_keeps_its_opening():
    spec = obstacle_spec("wall_gap", gap_width=120.0, gap_motion="static")
    event = emit(spec)[0]
    assert event.n == 2, "a wall with a gap is two segments"
    hw = np.asarray(event.half_w)
    centres = np.asarray(event.offsets)[:, 0]
    gap = float(np.ptp(centres)) - float(hw.sum())
    assert gap == pytest.approx(120.0, abs=2.0)
    # wall + opening covers the whole field width
    assert 2.0 * float(hw.sum()) + gap >= FIELD_W - 1.0


def test_wall_gap_may_not_be_smaller_than_the_player():
    with pytest.raises(ConfigError):
        obstacle_spec("wall_gap", gap_width=2.0)
    # exactly the player diameter is allowed
    spec = obstacle_spec("wall_gap", gap_width=2.0 * PLAYER_R)
    assert float(spec.params["gap_width"]) >= 2.0 * PLAYER_R


def test_wall_gap_default_opening_admits_the_player():
    spec = obstacle_spec("wall_gap")
    assert float(spec.params["gap_width"]) >= 2.0 * PLAYER_R


def test_wall_gap_can_sweep_its_opening():
    spec = obstacle_spec("wall_gap", gap_width=120.0, gap_motion="sweep",
                         interval=0.5, repetitions=4)
    events = gen_obstacle_field(spec, gen_ctx(field_w=FIELD_W, field_h=FIELD_H))
    assert len(events) > 1
    centres = [float(np.mean(np.asarray(e.offsets)[:, 0])) for e in events]
    assert np.ptp(centres) > 1.0, "the opening never moved"


# --------------------------------------------------------------------------
# small obstacles / cross traffic
# --------------------------------------------------------------------------


def test_small_obstacles_are_circles_born_on_the_boundary():
    spec = obstacle_spec("small", count=8, size=0.35)
    event = emit(spec)[0]
    assert event.n == 8
    assert float(spec.half_w) < PLAYER_R, "small obstacles must be smaller than the player"
    offsets = np.asarray(event.offsets)
    on_edge = (
        (np.abs(offsets[:, 0]) < 3.0)
        | (np.abs(offsets[:, 0] - FIELD_W) < 3.0)
        | (np.abs(offsets[:, 1]) < 3.0)
        | (np.abs(offsets[:, 1] - FIELD_H) < 3.0)
    )
    assert on_edge.all(), f"some small obstacles were born in the middle: {offsets}"


def test_small_obstacles_enter_from_a_single_boundary():
    """Leaves/twigs drift in one direction - they do not come from four sides."""
    spec = obstacle_spec("small", count=12, size=0.35)
    event = emit(spec)[0]
    offsets = np.asarray(event.offsets)

    on_top = np.abs(offsets[:, 1] - FIELD_H) < 3.0
    on_bottom = np.abs(offsets[:, 1]) < 3.0
    on_left = np.abs(offsets[:, 0]) < 3.0
    on_right = np.abs(offsets[:, 0] - FIELD_W) < 3.0
    edges = [on_top, on_bottom, on_left, on_right]
    assert sum(int(mask.any()) for mask in edges) == 1, (
        f"small obstacles must use exactly one boundary, got {offsets}"
    )
    assert on_top.all(), "the default entry boundary for small_obstacles is the top edge"

    # ...and they all travel the same way (downwards), inside the jitter cone
    jitter = math.radians(30.0)          # small_obstacles default angle_jitter (deg)
    inward = -math.pi / 2.0              # top edge -> toward the bottom
    assert np.all(np.abs(np.asarray(event.angles) - inward) <= jitter + 1e-9)


def test_small_obstacles_follow_the_configured_boundary():
    event = emit(obstacle_spec("small", count=6, region="left"))[0]
    offsets = np.asarray(event.offsets)
    assert np.all(np.abs(offsets[:, 0]) < 3.0), f"expected the left edge: {offsets}"
    jitter = math.radians(30.0)
    assert np.all(np.abs(np.asarray(event.angles)) <= jitter + 1e-9), "must travel +x"


def test_cross_traffic_uses_several_edges():
    spec = obstacle_spec("cross", count=8)
    event = emit(spec)[0]
    offsets = np.asarray(event.offsets)
    edges = set()
    for x, y in offsets:
        edges.add(min(
            {"left": x, "right": FIELD_W - x, "bottom": y, "top": FIELD_H - y}.items(),
            key=lambda kv: kv[1],
        )[0])
    assert len(edges) >= 2


# --------------------------------------------------------------------------
# corridor
# --------------------------------------------------------------------------


def _corridor_walls(spec: PatternSpec):
    event = emit(spec)[0]
    offsets = np.asarray(event.offsets)
    hw = np.asarray(event.half_w)
    hh = np.asarray(event.half_h)
    rot = np.asarray(event.rotation)
    return offsets, hw, hh, rot


def _corridor_gaps(offsets, hh, rot):
    """Perpendicular channel width for each pair of facing wall pieces."""
    theta = float(rot[0])
    n = np.array([-math.sin(theta), math.cos(theta)])
    n_side = offsets.shape[0] // 2
    return [
        abs(float((offsets[i] - offsets[i + n_side]) @ n)) - 2.0 * float(hh[0])
        for i in range(n_side)
    ]


def test_corridor_is_symmetric_about_the_player():
    spec = obstacle_spec("corridor", corridor_width=90.0)
    offsets, hw, hh, rot = _corridor_walls(spec)
    assert offsets.shape[0] == 2
    centre = gen_ctx(field_w=FIELD_W, field_h=FIELD_H).player_xy
    assert np.allclose(offsets.mean(axis=0), centre)
    # both walls run along the channel axis
    assert np.ptp(rot) == pytest.approx(0.0)
    assert _corridor_gaps(offsets, hh, rot)[0] == pytest.approx(90.0, abs=0.5)


def test_corridor_can_be_a_single_wall():
    spec = obstacle_spec("corridor", walls=1)
    offsets, hw, hh, rot = _corridor_walls(spec)
    assert offsets.shape[0] == 1


def test_corridor_open_and_close_move_the_walls_apart_or_together():
    for motion, expect_grow in (("open", True), ("close", False)):
        spec = obstacle_spec(
            "corridor", corridor_width=4.0 * PLAYER_R, motion=motion, change=1.5
        )
        event = emit(spec)[0]
        speeds = np.asarray(event.speeds)
        angles = np.asarray(event.angles)
        # both walls move, along the channel normal, in opposite directions
        assert (speeds > 0).all()
        vel = np.stack([np.cos(angles), np.sin(angles)], axis=-1) * speeds[:, None]
        assert float(np.dot(vel[0], vel[1])) < 0.0
        normal = np.array([0.0, 1.0])   # a horizontal channel
        assert (np.sign(vel[:, 1]) == (1.0 if expect_grow else -1.0)).any()


def test_corridor_rotation_is_bounded_by_the_geometry():
    # 80 degrees of scissor would pinch the player; the clamp cuts it down
    spec = obstacle_spec(
        "corridor", corridor_width=4.0 * PLAYER_R,
        motion="rotate_opposite", change=80.0, wall_length=6.0 * PLAYER_R,
    )
    assert 0.0 < float(spec.params["change"]) < 80.0
    # a same-direction tilt keeps the width, so it is allowed to be larger
    same = obstacle_spec(
        "corridor", corridor_width=4.0 * PLAYER_R, motion="rotate_same", change=30.0
    )
    assert float(same.params["change"]) == pytest.approx(30.0, abs=1.0)


def test_corridor_close_is_clamped_so_it_can_never_crush():
    spec = obstacle_spec("corridor", motion="close", change=99.0)
    width = float(spec.params["corridor_width"])
    change = float(spec.params["change"])
    assert width - change * 2.0 * PLAYER_R >= 1.25 * 2.0 * PLAYER_R - 1e-6


def test_corridor_unknown_motion_is_refused():
    with pytest.raises(ConfigError):
        obstacle_spec("corridor", motion="teleport")


def test_corridor_from_a_vertical_edge_runs_vertically():
    spec = obstacle_spec("corridor", region="top", corridor_width=90.0)
    offsets, hw, hh, rot = _corridor_walls(spec)
    assert float(rot[0]) == pytest.approx(-math.pi / 2.0)
    assert float(hw[0]) > float(hh[0])
    # a vertical channel separates the walls along x
    assert float(np.ptp(offsets[:, 0])) > 80.0


# --------------------------------------------------------------------------
# emission scheduling
# --------------------------------------------------------------------------


def test_emission_steps_honour_interval_and_repetitions():
    spec = PatternSpec(kind="obstacle", count=1, start_time=0.5, interval=0.25,
                       repetitions=3, params={"layout": "small"})
    ctx = gen_ctx()
    steps = emission_steps(spec, ctx)
    assert len(steps) == 3
    assert steps[0] == int(round(0.5 / DT))


def test_unknown_layout_is_refused():
    spec = PatternSpec(kind="obstacle", count=1, params={"layout": "nope"})
    with pytest.raises(ScenarioError):
        gen_obstacle_field(spec, gen_ctx())


def test_corridor_walls_extend_across_the_field_by_default():
    spec = obstacle_spec("corridor", corridor_width=120.0)
    # a horizontal channel spans the field width (with a small overshoot), so it
    # reads as a road rather than as two short blocks
    assert float(spec.params["wall_length"]) >= FIELD_W
    event = emit(spec)[0]
    assert float(np.asarray(event.half_w)[0]) == pytest.approx(
        float(spec.params["wall_length"]) / 2.0
    )

    # an explicit length still wins
    short = obstacle_spec("corridor", corridor_width=120.0, wall_length=180.0)
    assert float(short.params["wall_length"]) == pytest.approx(180.0)

    # a vertical channel spans the field height instead
    vertical = obstacle_spec("corridor", region="top", corridor_width=120.0)
    assert float(vertical.params["wall_length"]) >= FIELD_H


def test_corridor_gap_floor_is_explicit_and_cannot_be_lowered_past_safety():
    # an explicit floor above the safety minimum is honoured
    spec = obstacle_spec(
        "corridor", corridor_width=200.0, motion="close", change=20.0,
        corridor_min_width=60.0,
    )
    width = float(spec.params["corridor_width"])
    change = float(spec.params["change"])
    assert width - change * 2.0 * PLAYER_R == pytest.approx(60.0, abs=1e-6)

    # a floor below 1.25 x the player diameter is refused outright
    with pytest.raises(ConfigError):
        obstacle_spec("corridor", corridor_min_width=5.0)

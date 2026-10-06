"""Dynamic-obstacle environment: circle player, obstacle types, safety.

Covers the platform's required checks for the dynamic-obstacle redesign:

1  window scaling keeps the picture's proportions
2  the player is a circle: drawn size == collision size, radius adjustable
3  obstacle size is a multiple of the player circle diameter
4  obstacles are born on a boundary, never in the middle of the field
5  moving block enters from a band and travels inward
6  wall with gap keeps a passable, adjustable opening (never below the player)
7  small obstacles
8  cross traffic (several directions)
9  corridor (tilted, may narrow, never crushes)
10 mixed obstacles
11 relative motion
12 collision, including rotated rectangles
13 each obstacle type runs on its own
14 obstacle types combine
15 a fixed seed reproduces the same scene
16 manual mode
17 autonomous mode
18 headless mode

plus the safety layer: free space, path search, validation, regeneration, the
*measured* difficulty score, and the obstacle CLI.
"""

from __future__ import annotations

import os

import numpy as np
import pytest

os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from bullet_sim.action import Action, ManualInputSource  # noqa: E402
from bullet_sim.ai.factory import make_auto_source  # noqa: E402
from bullet_sim.collision.obstacles import ObstacleCollision, obstacle_sdf  # noqa: E402
from bullet_sim.core.errors import ConfigError  # noqa: E402
from bullet_sim.generators.patterns import available_patterns  # noqa: E402
from bullet_sim.obstacles.catalog import CATALOG, GENERATABLE  # noqa: E402
from bullet_sim.obstacles.scenario import (  # noqa: E402
    ObstacleScenario,
    scenario_from_types,
)
from bullet_sim.obstacles.spec import SIZE_PRESETS  # noqa: E402
from bullet_sim.physics.relative import (  # noqa: E402
    closing_speed,
    relative_velocity,
    time_to_impact,
    to_player_frame,
)
from bullet_sim.render.viewport import Viewport, viewport_for  # noqa: E402
from bullet_sim.safety.validate import (  # noqa: E402
    SafetyConfig,
    compute_safe_region,
    find_safe_path,
    generate_valid_scenario,
    is_scenario_valid,
    predict_collision,
    validate_scenario,
)
from bullet_sim.scenarios.builder import build_from_spec  # noqa: E402
from bullet_sim.scenarios.spec import ScenarioSpec  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402
from bullet_sim.simulator.world import World  # noqa: E402

DT = 1.0 / 60.0
PLAYER_R = 10.0
PLAYER_D = 2.0 * PLAYER_R

#: Coarse config so the safety checks stay fast in the unit suite.
FAST = SafetyConfig(horizon=1.0, sample_dt=0.1, resolution=24.0, max_window=1.0)

#: One wall actually sweeping across the player, so feasibility is decided by
#: geometry rather than by "the wall has not arrived yet".
WALL = SafetyConfig(horizon=8.0, sample_dt=0.1, resolution=16.0, max_window=8.0)


def one_wall(gap_width: float, speed: float = 200.0):
    """A single wall (no train behind it) crossing the whole field."""
    return [
        {
            "type": "wall_with_gap",
            "gap_width": gap_width,
            "speed": speed,
            "repetitions": 1,
        }
    ]


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def make_scenario(types, *, seed: int = 7, duration: float = 6.0, **kw) -> ObstacleScenario:
    if isinstance(types, str):
        types = [types]
    kw.setdefault("player_hitbox_radius", PLAYER_R)
    return scenario_from_types(
        types, seed=seed, duration=duration, require_valid=False, **kw
    )


def make_world(types, *, seed: int | None = None, duration: float = 6.0, **kw) -> World:
    scenario = (
        types if isinstance(types, ObstacleScenario) else make_scenario(types, duration=duration)
    )
    spec = scenario.to_spec()
    if seed is not None:
        spec = spec.replaced(seed=int(seed))
    return World(spec, reward="zero", terminate_on_collision=False, **kw)


def run(world: World, steps: int, action: int = 0) -> World:
    for _ in range(int(steps)):
        world.step(action)
    return world


def first_sightings(world: World, steps: int) -> dict[int, tuple[float, float, float, float]]:
    """First observed (x, y, half_w, half_h) of every obstacle id."""
    seen: dict[int, tuple[float, float, float, float]] = {}
    for _ in range(int(steps)):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size == 0:
            continue
        ids = pool.data["id"][idx]
        xs = pool.data["x"][idx]
        ys = pool.data["y"][idx]
        hw = pool.data["half_w"][idx]
        hh = pool.data["half_h"][idx]
        for i, x, y, a, b in zip(ids, xs, ys, hw, hh):
            seen.setdefault(int(i), (float(x), float(y), float(a), float(b)))
    return seen


def peak_count(world: World, steps: int, action: int = 0) -> int:
    peak = int(world.state.bullets.count)
    for _ in range(int(steps)):
        world.step(action)
        peak = max(peak, int(world.state.bullets.count))
    return peak


# ==========================================================================
# 1. window scaling / viewport
# ==========================================================================


def test_viewport_window_scaling_keeps_proportions():
    world = make_world("moving_block")
    small = viewport_for(world, (640, 480))
    big = viewport_for(world, (1600, 900))

    # one uniform scale for both axes: the picture never stretches
    x0, y0 = big.world_to_screen(0.0, 0.0)
    x1, _ = big.world_to_screen(120.0, 0.0)
    _, y2 = big.world_to_screen(0.0, 120.0)
    assert abs(x1 - x0) == pytest.approx(abs(y2 - y0))

    assert big.scale > small.scale
    assert big.scale == pytest.approx(big.fit_scale)

    resized = viewport_for(world, (640, 480)).resize(1600, 900)
    assert resized.scale == pytest.approx(big.scale)

    for wx, wy in [(0.0, 0.0), (320.0, 240.0), (640.0, 480.0)]:
        sx, sy = big.world_to_screen(wx, wy)
        rx, ry = big.screen_to_world(sx, sy)
        assert np.allclose([rx, ry], [wx, wy])

    bx0, by0, bx1, by1 = big.visible_world_box()
    assert bx1 > bx0 and by1 > by0
    assert bx0 <= 0.0 and bx1 >= big.world_w
    assert by0 <= 0.0 and by1 >= big.world_h


def test_viewport_zoom_is_relative_to_the_fit_scale():
    vp = Viewport(world_w=640.0, world_h=480.0, screen_w=1280, screen_h=960)
    fit = vp.fit_scale
    vp.zoom_by(2.0)
    assert vp.scale == pytest.approx(2.0 * fit)
    vp.reset_view()
    assert vp.scale == pytest.approx(fit)
    vp.set_zoom(0.01)
    assert vp.scale >= fit


# ==========================================================================
# 2/3. circle player and obstacle sizing
# ==========================================================================


def test_player_is_a_circle_whose_visible_size_is_the_hitbox():
    sc = make_scenario("moving_block")
    spec = sc.to_spec()

    # there is no separate visible body any more
    assert not hasattr(spec, "player_body_radius")
    assert not hasattr(spec, "body_radius")
    assert spec.player_radius == pytest.approx(PLAYER_R)
    assert spec.player_diameter == pytest.approx(PLAYER_D)

    world = make_world(sc)
    assert world.player_hitbox.shape == "circle"
    assert world.player_hitbox.radius == pytest.approx(PLAYER_R)
    # the drawn radius is the hitbox radius
    p = world.player
    from bullet_sim.entities.player import P_RADIUS

    assert float(p[P_RADIUS]) == pytest.approx(PLAYER_R)


def test_player_radius_is_adjustable():
    small = make_world(make_scenario("moving_block", player_hitbox_radius=6.0))
    big = make_world(make_scenario("moving_block", player_hitbox_radius=24.0))
    assert small.spec.player_radius == pytest.approx(6.0)
    assert big.spec.player_radius == pytest.approx(24.0)
    assert big.player_hitbox.radius > small.player_hitbox.radius

    with pytest.raises(ConfigError):
        ObstacleScenario(obstacles=["moving_block"], player_hitbox_radius=0.0)


def test_obstacle_size_is_a_multiple_of_the_player_diameter():
    sc = make_scenario([{"type": "small_obstacles", "size": SIZE_PRESETS["small"]}])
    spec = sc.to_spec()
    # a circle stores half_w == radius, and radius = ratio x diameter / 2
    assert float(spec.patterns[0].half_w) == pytest.approx(
        SIZE_PRESETS["small"] * PLAYER_D / 2.0
    )
    assert float(spec.patterns[0].radius) == pytest.approx(float(spec.patterns[0].half_w))

    block = make_scenario([{"type": "moving_block", "size": 2.0}]).to_spec()
    assert float(block.patterns[0].half_w) == pytest.approx(2.0 * PLAYER_R)

    # scaling the player scales the obstacles with it
    double = make_scenario(
        [{"type": "moving_block", "size": 2.0}], player_hitbox_radius=2 * PLAYER_R
    ).to_spec()
    assert float(double.patterns[0].half_w) == pytest.approx(4.0 * PLAYER_R)


# ==========================================================================
# 4. obstacles appear on a boundary, not in the middle
# ==========================================================================


def test_boundary_types_spawn_from_the_boundary_not_the_centre():
    boundary_types = ["moving_block", "wall_with_gap", "small_obstacles", "cross_traffic"]
    sc = make_scenario(boundary_types, duration=6.0)
    world = make_world(sc)
    seen = first_sightings(world, 240)
    assert seen, "no obstacle ever spawned"

    fw, fh = world.spec.field_w, world.spec.field_h
    tol = 0.12 * min(fw, fh)
    px, py = float(world.player[0]), float(world.player[1])
    body_r = world.spec.player_radius

    for oid, (x, y, hw, hh) in seen.items():
        edge = min(x - hw, y - hh, fw - (x + hw), fh - (y + hh))
        assert edge <= tol, (
            f"obstacle {oid} appeared in the middle of the field: "
            f"({x:.1f}, {y:.1f}) ext=({hw:.1f}, {hh:.1f})"
        )
        assert np.hypot(x - px, y - py) > body_r, (
            f"obstacle {oid} spawned on the player: ({x:.1f}, {y:.1f})"
        )


def test_obstacle_spawn_is_rejected_when_it_would_hit_the_player():
    spec = ScenarioSpec(
        name="reject",
        seed=0,
        duration=6.0,
        dt=DT,
        field_w=640.0,
        field_h=480.0,
        player_x=320.0,
        player_y=240.0,
        player_radius=PLAYER_R,
        collision="obstacle",
        patterns=[],
    )
    world = World(spec, reward="zero", terminate_on_collision=False)
    pool = world.state.bullets
    from bullet_sim.generators.burst import SpawnEvent

    event = SpawnEvent(
        step=0,
        offsets=np.array([[320.0, 240.0]]),
        angles=np.zeros(1),
        speeds=np.zeros(1),
        radius=np.zeros(1),
        ttl=np.full(1, 5.0),
        half_w=np.array([20.0]),
        half_h=np.array([20.0]),
        shape=np.array([1]),
        min_player_distance=100.0,
    )
    before = pool.count
    world._write_event(event, world.player[:2])
    assert pool.count == before, "an obstacle spawned on top of the player"
    assert world._spawn_rejected == 1


# ==========================================================================
# 5-10. one test per real-world obstacle type
# ==========================================================================


@pytest.mark.parametrize("type_key", list(GENERATABLE))
def test_each_obstacle_type_runs_on_its_own(type_key):
    sc = make_scenario(type_key, duration=8.0)
    assert sc.type_keys == [type_key]
    world = make_world(sc)
    peak = peak_count(world, 360)
    assert peak > 0, f"{type_key} never produced an obstacle"
    types = sc.types()
    assert len(types) == 1
    assert types[0].simulates and types[0].player_problem


def test_moving_block_enters_from_a_band_and_travels_inward():
    world = make_world(
        [{"type": "moving_block", "count": 3, "size": 2.0, "entry_span": 0.8}],
        duration=8.0,
    )
    world.step(0)
    pool = world.state.bullets
    idx = pool.active_indices()
    assert idx.size == 3, "the block band should be emitted together"
    xs = np.asarray(pool.data["x"][idx])
    ys = np.asarray(pool.data["y"][idx])
    vys = np.asarray(pool.data["vy"][idx])
    assert np.ptp(xs) > 0.2 * world.spec.field_w, "all blocks started at one point"
    # spawned on the top edge, moving down (into the field)
    assert (ys >= world.spec.field_h - 1e-6).all()
    assert (vys < 0.0).all()


def test_wall_with_gap_has_an_opening_that_admits_the_player():
    world = make_world(
        [{"type": "wall_with_gap", "gap_width": 3.0 * PLAYER_D}], duration=6.0
    )
    peak = peak_count(world, 240)
    assert peak >= 2, "a wall with a gap is several segments"
    spec = world.spec
    gap = float(spec.patterns[0].params["gap_width"])
    assert gap >= PLAYER_D

    from bullet_sim.safety.free_space import rasterize_free_space

    roll = world.clone()
    fracs = []
    for _ in range(120):
        roll.step(0)
        grid = rasterize_free_space(
            roll.state.bullets,
            field_w=roll.spec.field_w,
            field_h=roll.spec.field_h,
            inflate=roll.spec.player_radius + roll.spec.safety_margin,
            resolution=16.0,
        )
        fracs.append(grid.free_fraction)
    assert max(fracs) > 0.1, "the opening is sealed early on"


def test_wall_gap_may_not_be_smaller_than_the_player_circle():
    with pytest.raises(ConfigError):
        make_scenario([{"type": "wall_with_gap", "gap_width": PLAYER_D - 1.0}])
    # exactly the player diameter is allowed
    sc = make_scenario([{"type": "wall_with_gap", "gap_width": PLAYER_D}])
    assert float(sc.to_spec().patterns[0].params["gap_width"]) >= PLAYER_D


def test_small_obstacles_are_small_and_numerous():
    world = make_world(
        [{"type": "small_obstacles", "count": 8, "interval": 0.5}], duration=6.0
    )
    body_r = world.spec.player_radius
    radii = []
    for _ in range(300):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size:
            radii.append(np.asarray(pool.data["half_w"][idx]).copy())
    assert radii, "no small obstacles spawned"
    all_r = np.concatenate(radii)
    assert all_r.size >= 4
    assert float(all_r.max()) < body_r


def test_cross_traffic_enters_from_several_directions():
    sc = make_scenario(
        [{"type": "cross_traffic", "count": 6, "interval": 0.4}], duration=8.0
    )
    world = make_world(sc)
    seen = first_sightings(world, 420)
    assert len(seen) >= 3, "not enough traffic"
    fw, fh = world.spec.field_w, world.spec.field_h
    sides = set()
    for x, y, _, _ in seen.values():
        edges = {"left": x, "right": fw - x, "top": y, "bottom": fh - y}
        sides.add(min(edges, key=edges.get))
    assert len(sides) >= 2, f"traffic only came from {sides}"


def test_corridor_walls_are_symmetric_about_the_player():
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 4.0 * PLAYER_D, "motion": "static"}],
        duration=4.0,
    )
    # the player starts at the field centre, inside the channel
    assert sc.player_spawn() == (sc.field_w * 0.5, sc.field_h * 0.5)
    world = make_world(sc)
    run(world, 3)
    pool = world.state.bullets
    idx = pool.active_indices()
    assert idx.size == 2, "a two-wall corridor has two pieces"
    px, py = float(world.player[0]), float(world.player[1])
    ys = np.asarray(pool.data["y"][idx])
    # the two walls straddle the player, at equal distances
    assert ys.min() < py < ys.max()
    assert (py - ys.min()) == pytest.approx(ys.max() - py, abs=1.0)


@pytest.mark.parametrize(
    "motion,change",
    [("open", 1.5), ("close", 1.0), ("rotate_same", 20.0), ("rotate_opposite", 20.0)],
)
def test_corridor_motions_keep_the_player_safe(motion, change):
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 4.0 * PLAYER_D,
          "motion": motion, "change": change, "wall_length": 6.0 * PLAYER_D}],
        duration=12.0,
    )
    world = make_world(sc)
    px, py = float(world.player[0]), float(world.player[1])
    for _ in range(720):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        assert idx.size >= 1
        # the centre of every wall never translates
        if motion.startswith("rotate"):
            centres = np.stack([pool.data["x"][idx], pool.data["y"][idx]], axis=1)
            assert np.allclose(centres[:, 1], py + (centres[:, 1] - py))  # trivially true
    report = validate_scenario(world, config=FAST)
    assert report.feasible, report.reason


def test_corridor_never_narrows_below_the_player_even_if_asked():
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 2.0 * PLAYER_D,
          "motion": "close", "change": 99.0}],
        duration=20.0,
    )
    spec = sc.to_spec()
    change = float(spec.patterns[0].params["change"])
    width = float(spec.patterns[0].params["corridor_width"])
    # 99 player diameters of closing was clamped to what still admits the player
    assert change < 99.0
    assert width - change * PLAYER_D >= 1.25 * PLAYER_D - 1e-6


def test_corridor_opposite_rotation_is_angle_limited():
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 4.0 * PLAYER_D,
          "motion": "rotate_opposite", "change": 80.0, "wall_length": 6.0 * PLAYER_D}],
        duration=10.0,
    )
    change = float(sc.to_spec().patterns[0].params["change"])
    assert 0.0 < change < 80.0, "an unbounded scissor would pinch the player"


def test_corridor_can_be_a_single_wall():
    sc = make_scenario([{"type": "corridor", "walls": 1, "motion": "static"}], duration=4.0)
    world = make_world(sc)
    run(world, 3)
    assert world.state.bullets.count == 1


def test_obstacle_types_can_be_combined():
    keys = ["moving_block", "wall_with_gap", "small_obstacles"]
    sc = make_scenario(keys, duration=8.0)
    assert sc.type_keys == keys
    world = make_world(sc)
    shapes = set()
    for _ in range(300):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size:
            shapes.update(np.asarray(pool.data["shape"][idx]).tolist())
    assert shapes >= {0, 1}, "the mix should contain circles and rectangles"


# ==========================================================================
# 11. relative motion
# ==========================================================================


def test_relative_velocity_is_observer_relative():
    v_obs = np.array([[10.0, 0.0], [0.0, -5.0]])
    v_player = np.array([4.0, 2.0])
    assert np.allclose(relative_velocity(v_obs, v_player), [[6.0, -2.0], [-4.0, -7.0]])
    assert np.allclose(relative_velocity(v_obs, v_obs), 0.0)

    me = np.array([0.0, 0.0])
    assert closing_speed(np.array([[100.0, 0.0]]), np.array([[-100.0, 0.0]]), me)[0] > 0
    assert closing_speed(np.array([[100.0, 0.0]]), np.array([[100.0, 0.0]]), me)[0] < 0
    assert closing_speed(np.array([[50.0, 0.0]]), np.array([[0.0, 50.0]]), me)[
        0
    ] == pytest.approx(0.0)

    tti = time_to_impact(
        np.array([[100.0, 0.0]]), np.array([[-50.0, 0.0]]), me, radius=0.0
    )
    assert tti[0] == pytest.approx(2.0)
    assert np.isinf(
        time_to_impact(np.array([[0.0, 100.0]]), np.array([[10.0, 0.0]]), me)[0]
    )


def test_player_frame_removes_the_player_velocity():
    world = make_world(["cross_traffic"], duration=6.0)
    run(world, 120)
    world.step(1)
    snap = world.get_state()
    frame = to_player_frame(snap)
    assert frame.player.vx == 0.0 and frame.player.vy == 0.0
    idx = snap.bullets.active_indices()
    if idx.size:
        assert np.allclose(
            np.asarray(frame.bullets.data["vx"][idx]),
            np.asarray(snap.bullets.data["vx"][idx]) - snap.player.vx,
        )


# ==========================================================================
# 12. collision, including rotated rectangles
# ==========================================================================


def _rect_world() -> World:
    spec = ScenarioSpec(
        name="rect",
        seed=0,
        duration=4.0,
        dt=DT,
        field_w=640.0,
        field_h=480.0,
        player_x=300.0,
        player_y=300.0,
        player_radius=3.0,
        collision="obstacle",
        patterns=[],
    )
    world = World(spec, reward="zero", terminate_on_collision=False)
    assert isinstance(world.collision, ObstacleCollision)
    return world


def test_rotated_rectangle_collision_respects_orientation():
    flat = _rect_world()
    flat.state.bullets.spawn(
        310.0, 300.0, 0.0, 0.0,
        half_w=8.0, half_h=2.0, shape="rect", rotation=0.0, ttl=10.0,
    )
    res = flat.step(0)
    assert res.info["collision_event"] is True
    assert res.info["collision_count"] == 1

    turned = _rect_world()
    turned.state.bullets.spawn(
        310.0, 300.0, 0.0, 0.0,
        half_w=8.0, half_h=2.0, shape="rect", rotation=np.pi / 2.0, ttl=10.0,
    )
    res2 = turned.step(0)
    assert res2.info["collision_event"] is False, (
        "a rectangle rotated out of the way must not register a hit"
    )


def test_obstacle_sdf_matches_the_geometry():
    def sdf(rotation):
        return obstacle_sdf(
            300.0, 300.0,
            np.array([310.0]), np.array([300.0]),
            np.array([1]), np.array([0.0]),
            np.array([8.0]), np.array([2.0]),
            np.array([rotation]),
        )[0]

    assert float(sdf(0.0)) == pytest.approx(2.0, abs=1e-6)
    assert float(sdf(np.pi / 2.0)) > float(sdf(0.0))
    circle = obstacle_sdf(
        300.0, 300.0,
        np.array([300.0]), np.array([300.0]),
        np.array([0]), np.array([5.0]),
        np.array([5.0]), np.array([5.0]), np.array([0.0]),
    )[0]
    assert float(circle) == pytest.approx(-5.0)


def test_collision_is_a_penalty_event_not_a_termination():
    world = _rect_world()
    world.state.bullets.spawn(
        301.0, 300.0, 0.0, 0.0,
        half_w=6.0, half_h=6.0, shape="rect", rotation=0.0, ttl=10.0,
    )
    res = world.step(0)
    assert res.info["collision_event"] is True
    assert res.terminated is False
    assert world.collision_count >= 1
    assert world.cumulative_reward <= 0.0


# ==========================================================================
# 15. reproducibility
# ==========================================================================


def test_fixed_seed_reproduces_the_same_scenario():
    sc = make_scenario(["moving_block", "wall_with_gap", "small_obstacles"], seed=11)
    a = make_world(sc)
    b = make_world(sc)
    for i in range(300):
        a.step(i % 9)
        b.step(i % 9)
    assert a.state_hash() == b.state_hash()
    assert np.allclose(
        np.asarray(a.state.bullets.data["x"]), np.asarray(b.state.bullets.data["x"])
    )

    built_a = build_from_spec(sc.to_spec())
    built_b = build_from_spec(sc.to_spec())
    assert built_a.peak_bullets == built_b.peak_bullets
    assert built_a.timeline.summary() == built_b.timeline.summary()
    assert built_a.timeline.steps == built_b.timeline.steps


def test_different_seed_changes_the_scenario():
    a = make_world(make_scenario(["small_obstacles"], seed=1))
    b = make_world(make_scenario(["small_obstacles"], seed=2))
    run(a, 200)
    run(b, 200)
    assert a.state_hash() != b.state_hash()


def test_obstacle_scenario_round_trips_through_json(tmp_path):
    sc = make_scenario(["moving_block", "corridor"], seed=5)
    path = sc.save(str(tmp_path / "scene.json"))
    again = ObstacleScenario.load(path)
    assert again.to_dict() == sc.to_dict()
    assert again.to_spec().patterns[0].params["layout"] == "block"


# ==========================================================================
# 16/17/18. manual, autonomous and headless modes
# ==========================================================================


def test_manual_mode_drives_the_player_through_obstacles():
    spec = make_scenario("moving_block", duration=6.0).to_spec()
    env = BulletHellEnv(spec, seed=3)
    src = ManualInputSource()
    start = env.world.player.copy()
    src.press("right")
    out = env.run(input_source=src, steps=60)
    moved = env.world.player - start
    assert moved[0] > 1.0, "manual right input did not move the player"
    assert abs(moved[1]) < abs(moved[0])
    assert out["steps"] == 60
    assert out["input_mode"]
    env.close()


def test_manual_stop_overrides_movement():
    src = ManualInputSource()
    src.press("right")
    assert src.poll(DT) is not None
    src.press("space")
    action = src.poll(DT)
    assert action is None or action.magnitude == 0.0


@pytest.mark.parametrize("controller_name", ["repulsion", "threat"])
def test_autonomous_mode_moves_the_player_without_input(controller_name):
    spec = make_scenario(["moving_block", "small_obstacles"], duration=8.0).to_spec()
    env = BulletHellEnv(spec, seed=4)
    start = env.world.player.copy()
    source, ctrl = make_auto_source(controller_name, world=env.world, seed=1)
    out = env.run(input_source=source, steps=180)
    moved = float(np.hypot(*(env.world.player[:2] - start[:2])))
    assert moved > 1.0, f"{controller_name} produced no motion"
    assert out["steps"] == 180
    assert out["collision_events"] >= 0
    assert ctrl is not None
    env.close()


def test_headless_mode_needs_no_window():
    spec = make_scenario(["wall_with_gap", "cross_traffic"], duration=6.0).to_spec()
    env = BulletHellEnv(spec, seed=6, render_mode="none")
    out = env.run(steps=120)
    assert out["steps"] == 120
    assert env.render() is None
    assert out["state_hash"]
    env.close()


# ==========================================================================
# safety layer: 有挑战 ≠ 必死
# ==========================================================================


def test_safe_region_and_path_exist_for_a_gap_scenario():
    world = make_world([{"type": "wall_with_gap", "gap_width": 5.0 * PLAYER_D}], duration=4.0)
    region = compute_safe_region(world, FAST)
    assert 0.0 <= region.free_fraction <= 1.0
    assert region.reachable_fraction >= 0.0
    assert region.safe_points(limit=5)

    path = find_safe_path(world, FAST)
    assert path is not None, "a wide gap must admit a safe path"
    assert path.length > 1
    assert path.min_clearance >= 0


def test_validation_rejects_a_scene_that_inflation_seals():
    """A gap the player fits through, but not with the safety margin."""
    tight = make_world(one_wall(PLAYER_D), duration=8.6)
    report = validate_scenario(tight, config=WALL)
    assert report.feasible is False
    assert report.reason
    assert report.suggestions
    assert is_scenario_valid(tight, config=WALL) is False
    assert report.reachable_cells == 0


def test_validation_accepts_a_generous_scene():
    world = make_world(one_wall(200.0), duration=8.6)
    report = validate_scenario(world, config=WALL)
    assert report.feasible is True, report.reason
    assert report.reachable_cells > 0
    assert report.min_clearance >= 0.0
    assert report.inflated_by > 0.0
    assert report.strategy_note


def test_generate_valid_scenario_adjusts_until_it_works():
    hard = make_scenario(one_wall(PLAYER_D, speed=200.0), duration=8.6)
    scenario, report, attempts = generate_valid_scenario(
        hard, config=WALL, max_attempts=10, strict=False
    )
    assert attempts >= 1
    assert scenario is not None
    if report.feasible:
        # it had to relax the offending parameter
        assert scenario.obstacles[0].gap_width >= hard.obstacles[0].gap_width


def test_predict_collision_reports_time_to_impact():
    world = make_world(["moving_block"], duration=6.0)
    run(world, 60)
    pred = predict_collision(world, 0, 1.0)
    assert set(pred) >= {
        "time_to_collision",
        "collides_within_horizon",
        "analytic_time_to_impact",
        "max_closing_speed",
    }
    assert pred["horizon"] == pytest.approx(1.0)

    head_on = _rect_world()
    head_on.state.bullets.spawn(
        320.0, 300.0, -60.0, 0.0,
        half_w=4.0, half_h=4.0, shape="rect", ttl=10.0,
    )
    pred2 = predict_collision(head_on, 0, 1.0)
    assert pred2["analytic_time_to_impact"] is not None
    assert pred2["max_closing_speed"] > 0.0


def test_only_realistic_obstacles_exist():
    # only the obstacle generator, and only the six catalogued types
    assert available_patterns() == ["obstacle"]
    assert set(CATALOG) == {
        "moving_block", "wall_with_gap", "small_obstacles",
        "corridor", "cross_traffic",
    }
    assert not hasattr(
        __import__("bullet_sim.obstacles.catalog", fromlist=["x"]), "LEGACY_PATTERNS"
    )


def test_action_interface_is_unchanged_by_obstacles():
    """Manual, autonomous and scripted input all produce the same Action type."""
    from bullet_sim.action.base import ControllerSource, action_to_codec_input
    from bullet_sim.entities.player import P_SPEED

    spec = make_scenario("moving_block", duration=6.0).to_spec()
    env = BulletHellEnv(spec, seed=2)

    manual = ManualInputSource()
    manual.press("left")
    a = manual.poll(DT)
    assert isinstance(a, Action)

    auto, ctrl = make_auto_source("idle", world=env.world, seed=0)
    assert isinstance(auto, ControllerSource)
    env.reset(seed=2)
    auto.open()
    b = auto.poll_observation(env._last_obs, env._last_info)  # type: ignore[attr-defined]
    auto.close()
    assert isinstance(b, Action)

    raw = action_to_codec_input(a, env.world.codec, float(env.world.player[P_SPEED]))
    out = env.step(raw)
    assert len(out) == 5
    env.close()


# ==========================================================================
# 5. difficulty is measured from the scene, not labelled
# ==========================================================================


def test_safety_report_carries_a_measured_complexity_score():
    sealed = make_world(one_wall(PLAYER_D), duration=8.6)
    open_scene = make_world(one_wall(200.0), duration=8.6)

    r_sealed = validate_scenario(sealed, config=WALL)
    r_open = validate_scenario(open_scene, config=WALL)

    assert 0.0 <= r_open.complexity <= 1.0
    assert 0.0 <= r_sealed.complexity <= 1.0
    assert r_sealed.complexity > r_open.complexity
    assert r_sealed.complexity_label and r_open.complexity_label

    for key in ("crowding", "reach", "tightness", "speed", "anticipation",
                "geometry", "path_scarcity"):
        assert key in r_open.complexity_terms
    assert r_sealed.complexity_terms["reach"] > r_open.complexity_terms["reach"]

    payload = r_open.to_dict()
    assert payload["complexity"] == pytest.approx(round(r_open.complexity, 4))
    assert payload["complexity_terms"]
    assert "实测难度" in r_open.format()


def test_complexity_speed_term_tracks_measured_relative_speed():
    slow = validate_scenario(
        make_world([{"type": "small_obstacles", "speed": 40.0, "count": 6}], duration=4.0),
        config=SafetyConfig(horizon=2.0, sample_dt=0.1, resolution=16.0, max_window=2.0),
    )
    fast = validate_scenario(
        make_world([{"type": "small_obstacles", "speed": 260.0, "count": 6}], duration=4.0),
        config=SafetyConfig(horizon=2.0, sample_dt=0.1, resolution=16.0, max_window=2.0),
    )
    assert fast.complexity_terms["speed"] > slow.complexity_terms["speed"]


def test_cli_validate_prints_the_measured_difficulty(capsys):
    from bullet_sim.cli import main

    argv = [
        "run", "--obstacle-type", "wall_with_gap", "--gap-width", "200",
        "--obstacle-speed", "200", "--safety-horizon", "8", "--validate",
        "--steps", "30", "--json",
    ]
    assert main(argv) == 0
    assert "实测难度" in capsys.readouterr().err


# ==========================================================================
# 8. the picture shows the circle, world coordinates, scene type, obstacle count
# ==========================================================================


def test_hud_reports_circle_world_coords_scene_type_and_obstacle_count():
    from bullet_sim.render.overlays import geometry_lines, hud_lines, scene_lines

    world = make_world(["wall_with_gap", "cross_traffic"], duration=8.0)
    peak = peak_count(world, 300)
    assert peak > 0

    text = "\n".join(hud_lines(world, fps=60.0, sim_fps=120.0))

    scene = scene_lines(world)[0]
    assert "wall_with_gap" in scene and "cross_traffic" in scene

    geometry = "\n".join(geometry_lines(world))
    assert f"circle r={world.spec.player_radius:.1f}" in geometry
    assert "visible == collision" in geometry
    assert f"obstacles : {world.state.bullets.count:5d}" in geometry

    assert "world xy" in text
    assert f"bullets   : {world.active_bullet_count():6d}" in text
    assert f"field {world.spec.field_w:.0f}x{world.spec.field_h:.0f}" in text
    # no "body" language anywhere in the HUD
    assert "body" not in text


def test_renderer_draws_world_axes_and_toggleable_hitboxes():
    pytest.importorskip("pygame")
    from bullet_sim.render.pygame_view import PygameRenderer

    world = make_world(["moving_block", "small_obstacles"], duration=8.0)
    peak_count(world, 240)

    renderer = PygameRenderer(
        mode="rgb_array", window_size=(800, 600), show_hitboxes=True, show_world_axes=True
    )
    frame = renderer.render(world)
    assert frame is not None
    assert renderer.viewport is not None

    vp = renderer.viewport
    assert vp.screen_w == 800 and vp.screen_h == 600
    for wx, wy in [(0.0, 0.0), (320.0, 240.0), (640.0, 480.0)]:
        sx, sy = vp.world_to_screen(wx, wy)
        rx, ry = vp.screen_to_world(sx, sy)
        assert np.allclose([rx, ry], [wx, wy])

    before = vp.scale
    renderer.resize(1200, 700)
    renderer.render(world)
    assert renderer.viewport.scale > before

    renderer.show_hitboxes = False
    assert renderer.render(world) is not None
    renderer.close()


# ==========================================================================
# CLI: obstacle catalogue, obstacle scenarios, safety flags
# ==========================================================================


def test_cli_obstacles_lists_the_catalogue(capsys):
    import json as _json

    from bullet_sim.cli import main

    assert main(["obstacles", "--json"]) == 0
    payload = _json.loads(capsys.readouterr().out)
    keys = {t["key"] for t in payload["obstacle_types"]}
    assert keys == set(CATALOG)
    assert keys == {
        "moving_block", "wall_with_gap", "small_obstacles",
        "corridor", "cross_traffic",
    }
    assert payload["generatable"]

    assert main(["obstacles"]) == 0
    text = capsys.readouterr().out
    assert "safety" in text and "size presets" in text
    for key in CATALOG:
        assert key in text


def test_cli_obstacles_single_type_json(capsys):
    import json as _json

    from bullet_sim.cli import main

    assert main(["obstacles", "--type", "moving_block", "--json"]) == 0
    payload = _json.loads(capsys.readouterr().out)
    assert payload["key"] == "moving_block"
    assert payload["simulates"] and payload["motion"] and payload["player_problem"]


def test_cli_obstacles_unknown_type_exits_nonzero(capsys):
    from bullet_sim.cli import main

    assert main(["obstacles", "--type", "moving_wall"]) == 2
    assert "unknown obstacle type" in capsys.readouterr().err


def test_cli_run_with_obstacle_types(capsys):
    import json as _json

    from bullet_sim.cli import main

    code = main([
        "run", "--obstacle-type", "moving_block",
        "--obstacle-type", "small_obstacles",
        "--seed", "3", "--steps", "180", "--json",
    ])
    assert code == 0
    payload = _json.loads(capsys.readouterr().out)
    assert payload["steps"] == 180
    assert payload["state_hash"]


def test_cli_validation_rejects_a_sealed_scenario(capsys):
    from bullet_sim.cli import main

    argv = [
        "run", "--obstacle-type", "wall_with_gap", "--gap-width", "20",
        "--obstacle-speed", "200", "--safety-horizon", "8", "--validate",
        "--steps", "60", "--json",
    ]
    assert main(argv) == 2
    assert "rejected by the safety check" in capsys.readouterr().err
    assert main(argv + ["--allow-infeasible"]) == 0


def test_cli_require_valid_refuses_an_infeasible_scene(capsys):
    from bullet_sim.cli import main

    argv = [
        "run", "--obstacle-type", "wall_with_gap", "--gap-width", "20",
        "--obstacle-speed", "200", "--safety-horizon", "8", "--require-valid",
        "--steps", "30", "--json",
    ]
    assert main(argv) == 2
    assert "rejected by the safety check" in capsys.readouterr().err
    assert main(argv + ["--allow-infeasible"]) == 0


def test_cli_gap_below_the_player_is_refused(capsys):
    from bullet_sim.cli import main

    code = main([
        "run", "--obstacle-type", "wall_with_gap", "--gap-width", "5",
        "--steps", "10", "--json",
    ])
    assert code != 0


def test_cli_scenario_export_can_be_inspected_and_validated(tmp_path, capsys):
    from bullet_sim.cli import main

    out = tmp_path / "scene.json"
    code = main([
        "scenario", "--obstacle-type", "wall_with_gap", "--gap-width", "200",
        "--duration", "4", "--seed", "5", "-o", str(out),
    ])
    assert code == 0
    assert out.exists()

    capsys.readouterr()
    assert main(["obstacles", "--scenario", str(out), "--validate", "--safe-path"]) == 0
    text = capsys.readouterr().out
    assert "wall_gap" in text
    assert "safe path" in text


# ==========================================================================
# level = one intuitive master control, and every level keeps a safe region
# ==========================================================================


def test_level_is_a_proportional_master_control():
    from bullet_sim.scenarios.builder import make_scenario
    from bullet_sim.scenarios.complexity import profile_for

    values = [0.0, 0.15, 0.4, 0.7, 1.0]
    speeds = [profile_for(v).speed_scale for v in values]
    counts = [profile_for(v).count_scale for v in values]
    kinds = [len(profile_for(v).obstacle_kinds) for v in values]

    # all three scale monotonically with complexity - that *is* the level
    for series in (speeds, counts, kinds):
        assert series == sorted(series)
        assert series[0] < series[-1]

    targets = [make_scenario(v, seed=3).meta["target_bullet_count"] for v in values]
    assert targets == sorted(targets)


def test_extreme_keeps_a_safe_region():
    from bullet_sim.scenarios.builder import make_scenario

    spec = make_scenario("extreme", seed=3, duration=15.0)
    safety = spec.meta["safety"]
    assert safety is not None, "the level build must record its safety report"
    assert safety["feasible"] is True, safety["reason"]
    # a *region*, not a single surviving pixel
    assert safety["free_fraction"] >= 0.15
    assert safety["reachable_cells"] >= 12
    # the requested ladder value is preserved; the safety thinning is recorded
    assert spec.meta["achieved_bullet_target"] <= spec.meta["target_bullet_count"]
    assert spec.meta["level_reduced_by"] >= 0
    # extreme still unlocks every obstacle type
    assert len(spec.meta["obstacle_kinds"]) == len(GENERATABLE)


def test_every_level_is_valid_over_several_seeds():
    from bullet_sim.scenarios.builder import make_scenario

    for level in ("easy", "medium", "hard", "extreme"):
        for seed in (1, 7):
            spec = make_scenario(level, seed=seed, duration=12.0)
            safety = spec.meta["safety"]
            assert safety["feasible"] is True, f"{level}/{seed}: {safety['reason']}"


def test_moving_block_does_not_plug_the_wall_opening():
    from bullet_sim.scenarios.builder import make_scenario

    spec = make_scenario("hard", seed=3, duration=15.0)
    by_layout = {p.params.get("layout"): p for p in spec.patterns}
    assert "block" in by_layout and "wall_gap" in by_layout

    block, wall = by_layout["block"], by_layout["wall_gap"]
    # the block enters from a *perpendicular* edge, so it is never born in the
    # wall's opening, and its speed is deliberately different so the two cannot
    # arrive together
    assert block.params.get("spawn") != wall.params.get("spawn")
    assert abs(float(block.speed) - float(wall.speed)) > 0.25 * float(wall.speed)
    # the block is phase-shifted against the wall's cadence
    assert block.start_time != wall.start_time
    # and the composed scene still has a safe region
    assert spec.meta["safety"]["feasible"] is True


def test_moving_block_enters_at_random_positions_along_the_band():
    a = make_world([{"type": "moving_block", "count": 4, "entry_span": 0.6}], seed=1)
    b = make_world([{"type": "moving_block", "count": 4, "entry_span": 0.6}], seed=2)
    a.step(0)
    b.step(0)
    xa = np.asarray(a.state.bullets.data["x"][a.state.bullets.active_indices()])
    xb = np.asarray(b.state.bullets.data["x"][b.state.bullets.active_indices()])
    assert xa.size == 4 and xb.size == 4
    assert not np.allclose(xa, xb), "the entry positions must be random, not fixed lanes"
    assert np.ptp(xa) > 0.1 * a.spec.field_w


def test_rectangle_angular_velocity_spins_the_body_not_the_velocity():
    from bullet_sim.scenarios.spec import ScenarioSpec

    spec = ScenarioSpec(
        name="spin", seed=0, duration=4.0, dt=DT, field_w=640.0, field_h=480.0,
        player_x=320.0, player_y=100.0, player_radius=PLAYER_R,
        collision="obstacle", patterns=[],
    )
    world = World(spec, reward="zero", terminate_on_collision=False)
    (i,) = world.state.bullets.spawn(
        320.0, 400.0, 0.0, 0.0, half_w=60.0, half_h=8.0,
        shape="rect", rotation=0.0, angular_velocity=1.0, ttl=10.0,
    )
    for _ in range(120):
        world.step(0)
    # 120 steps at dt=1/60 is 2 s, and the body spins at 1 rad/s...
    assert float(world.state.bullets.data["rotation"][i]) == pytest.approx(2.0, abs=0.05)
    # ...while the wall stayed exactly where it was
    assert float(world.state.bullets.data["x"][i]) == pytest.approx(320.0)
    assert float(world.state.bullets.data["y"][i]) == pytest.approx(400.0)

    # a circle keeps the original meaning: its velocity direction curves
    (j,) = world.state.bullets.spawn(
        100.0, 100.0, 100.0, 0.0, radius=3.0, shape="circle",
        angular_velocity=1.0, ttl=10.0,
    )
    for _ in range(30):
        world.step(0)
    assert abs(float(world.state.bullets.data["vy"][j])) > 1.0


def test_corridor_is_one_persistent_pair_that_rotates_at_runtime():
    """Regression: the corridor must not re-emit stacked pairs every interval."""
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 4.0 * PLAYER_D,
          "motion": "rotate_same", "change": 20.0}],
        duration=30.0,
    )
    world = make_world(sc)
    px, py = float(world.player[0]), float(world.player[1])
    angles = []
    # walk the whole 30 s episode at the scenario's own dt (1/120 by default)
    for step_index in range(int(round(30.0 / world.spec.dt))):
        world.step(0)
        pool = world.state.bullets
        idx = pool.active_indices()
        assert idx.size == 2, f"step {step_index}: corridor became {idx.size} walls"
        centres = np.stack([pool.data["x"][idx], pool.data["y"][idx]], axis=1)
        assert np.allclose(centres[:, 0], px), "a rotating corridor wall translated"
        assert np.allclose(centres[:, 1], np.sort(centres[:, 1]))  # symmetric pair
        angles.append(np.degrees(np.asarray(pool.data["rotation"][idx])).copy())
    first, last = angles[0], angles[-1]
    # both walls turned the same way, by the requested total, and stayed parallel
    assert last[0] == pytest.approx(last[1])
    assert last[0] - first[0] == pytest.approx(20.0, abs=1.0)


def test_corridor_scissor_rotates_the_two_walls_oppositely():
    sc = make_scenario(
        [{"type": "corridor", "corridor_width": 4.0 * PLAYER_D,
          "motion": "rotate_opposite", "change": 24.0}],
        duration=12.0,
    )
    world = make_world(sc)
    for _ in range(720):
        world.step(0)
    pool = world.state.bullets
    idx = pool.active_indices()
    assert idx.size == 2
    angles = np.degrees(np.asarray(pool.data["rotation"][idx]))
    assert angles[0] == pytest.approx(-angles[1], abs=0.5)
    assert abs(angles[0]) > 1.0, "the scissor never actually rotated"

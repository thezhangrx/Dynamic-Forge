"""Renderer-agnostic overlay data.

Everything here is plain data (strings, arrays, cell lists) so the same debug
information can be drawn by pygame, printed as ASCII, or streamed to a web UI.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np


@dataclass
class FrameCounter:
    """Smoothed render FPS estimator (decoupled from the simulation rate)."""

    window: int = 30
    _times: list[float] = field(default_factory=list)

    def tick(self) -> float:
        now = time.perf_counter()
        self._times.append(now)
        if len(self._times) > self.window:
            del self._times[: len(self._times) - self.window]
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return 0.0 if span <= 0 else (len(self._times) - 1) / span

    @property
    def fps(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        return 0.0 if span <= 0 else (len(self._times) - 1) / span


def mode_lines(
    source: Any,
    *,
    controller: Any = None,
    extra: Sequence[str] | None = None,
) -> list[str]:
    """Control-mode block: what is driving the avatar right now.

    This is the line a组员 looks at first: manual vs autonomous vs board, and
    which concrete controller/model is active.
    """
    if source is None:
        return []
    lines: list[str] = []
    name = getattr(source, "name", type(source).__name__)
    mode = getattr(source, "mode", None)
    if mode is not None:
        modes = getattr(source, "modes", [])
        lines.append(f"mode      : {mode.upper()}   (TAB cycles: {' -> '.join(modes)})")
        active = getattr(source, "active", None)
        name = getattr(active, "name", name)
    else:
        kind = "manual" if "keyboard" in str(name) else "auto"
        lines.append(f"mode      : {kind.upper()}   (source: {name})")

    spec = getattr(controller, "spec", None)
    if callable(spec):
        try:
            info = spec()
            lines.append(info.hud_line())
            if info.notes:
                lines.append(f"ai note   : {info.notes}")
        except Exception:  # pragma: no cover - HUD must never crash the loop
            pass
    else:
        lines.append(f"ai        : -   (source: {name})")
    if extra:
        lines.extend(str(e) for e in extra)
    return lines


def collision_lines(world: Any) -> list[str]:
    """Collision-as-penalty status: count, reward and "just hit" feedback.

    A collision is deliberately shown as a *counter and a cost*, never as a
    game-over banner: the episode keeps running and so does the window.
    """
    count = int(getattr(world, "collision_count", 0))
    reward = float(getattr(world, "cumulative_reward", 0.0))
    lines = [f"collisions: {count:<4d}  reward: {reward:+.1f}"]
    tracker = getattr(world, "contacts", None)
    if tracker is not None:
        mode = getattr(tracker, "mode", "?")
        active = tracker.active_count
        extra = f"  mode: {mode}"
        if active:
            extra += f"  IN CONTACT ({active})"
        elif getattr(tracker, "last_event_frame", None) is not None:
            step = int(getattr(world.state.env, "step_index", 0))
            gap = step - int(tracker.last_event_frame)
            extra += f"  last hit {gap} steps ago"
        lines.append(f"event     :{extra}")
    return lines


def is_collision_flash(world: Any, *, frames: int = 12) -> bool:
    """True for a few frames after a new collision event (visual feedback)."""
    tracker = getattr(world, "contacts", None)
    last = getattr(tracker, "last_event_frame", None)
    if last is None:
        return False
    step = int(getattr(world.state.env, "step_index", 0))
    return 0 <= step - int(last) < int(frames)


def scene_lines(world: Any) -> list[str]:
    """Scenario / obstacle-type line: what kind of environment this is."""
    spec = world.spec
    types = list(spec.meta.get("obstacle_types", []) or [])
    kind = ", ".join(types) if types else (spec.meta.get("obstacle_scenario") or spec.name)
    shape_mix = getattr(world, "obstacle_shape_counts", None)
    return [f"scene     : {kind}"]


def geometry_lines(world: Any) -> list[str]:
    """Player body vs hitbox vs obstacle sizes - the proportion is the point."""
    spec = world.spec
    pool = world.state.bullets
    idx = pool.active_indices()
    hit = float(spec.player_radius)
    sizes = "n/a"
    if idx.size:
        half_w = pool.data["half_w"][idx]
        half_h = pool.data["half_h"][idx]
        extent = np.hypot(half_w, half_h)
        sizes = f"min {extent.min():.1f} / max {extent.max():.1f} / mean {extent.mean():.1f}"
    return [
        f"player    : circle r={hit:.1f} (visible == collision)",
        f"obstacles : {idx.size:5d}  extent(px) {sizes}",
    ]


def hud_lines(
    world: Any,
    *,
    fps: float | None = None,
    sim_fps: float | None = None,
    source: Any = None,
    controller: Any = None,
    extra: Sequence[str] | None = None,
) -> list[str]:
    """Human-readable status block: mode, FPS, bullet count, player, scenario."""
    p = world.player_state()
    spec = world.spec
    lines = mode_lines(source, controller=controller)
    lines += [
        f"scenario  : {spec.name}  seed={spec.seed}  dt={spec.dt:.5f}s",
        f"step      : {world.state.env.step_index}/{spec.total_steps}"
        f"   t={world.state.env.sim_time:.3f}s",
    ]
    if fps is not None:
        lines.append(f"render fps: {fps:6.1f}")
    if sim_fps is not None:
        lines.append(f"sim fps   : {sim_fps:6.1f}  ({sim_fps * spec.dt:.2f}x realtime)")
    lines += scene_lines(world)
    lines += collision_lines(world)
    lines += [
        f"bullets   : {world.active_bullet_count():6d}  (cap {world.state.bullets.capacity})",
        f"player    : x={p.x:7.2f} y={p.y:7.2f} vx={p.vx:7.2f} vy={p.vy:7.2f}"
        f" r={p.radius:.2f} spd={p.speed:.1f}",
        f"target    : x={spec.target_state().x:7.2f} y={spec.target_state().y:7.2f}"
        f" dist={spec.target_state().distance_to(p.x, p.y):7.2f}",
        f"collision : {world.collision.name}   reward: {getattr(world.reward, 'name', '?')}",
    ]
    # inserted before the final line so the most dynamic numbers sit on top
    lines[-3:-3] = geometry_lines(world) + [
        f"world xy  : ({p.x:7.2f}, {p.y:7.2f})  |v|={p.speed_norm:7.2f}  "
        f"field {spec.field_w:.0f}x{spec.field_h:.0f}"
    ]
    if extra:
        lines.extend(str(e) for e in extra)
    return lines


def hud_text(world: Any, **kwargs: Any) -> str:
    return "\n".join(hud_lines(world, **kwargs))


def danger_cells(danger: Any, step: int = 0, threshold: float = 0.05) -> list[tuple[int, int, float]]:
    """``(row, col, value)`` cells above ``threshold`` for one horizon step."""
    grid = danger.grid[step] if step < danger.horizon else danger.max_over_time()
    rows, cols = np.nonzero(grid > threshold)
    return [(int(r), int(c), float(grid[r, c])) for r, c in zip(rows, cols)]


def prediction_polylines(forecast: Any, *, max_lines: int = 256) -> list[np.ndarray]:
    """Per-bullet ``(H, 2)`` polylines from a trajectory forecast (valid only)."""
    out: list[np.ndarray] = []
    positions = np.asarray(forecast.positions)
    valid = np.asarray(forecast.valid, dtype=bool)
    for i in range(min(positions.shape[0], max_lines)):
        m = valid[i]
        if not m.any():
            continue
        line = positions[i][m]
        if line.shape[0] >= 2:
            out.append(line)
    return out


def player_path(snapshot: Any, action_vector: Sequence[float], horizon: int) -> np.ndarray:
    """Straight-line player prediction for a constant velocity command."""
    dt = float(snapshot.env.dt)
    v = np.asarray(action_vector, dtype=np.float64).reshape(2)
    ts = (np.arange(1, int(horizon) + 1, dtype=np.float64))[:, None] * dt
    return snapshot.player.position[None, :] + ts * v[None, :]


def color_for_type(type_id: int) -> tuple[int, int, int]:
    """Stable, human-readable colour per bullet type id."""
    palette = (
        (235, 90, 90),
        (240, 170, 70),
        (235, 235, 110),
        (120, 220, 130),
        (110, 190, 240),
        (150, 150, 250),
        (230, 130, 220),
        (220, 220, 220),
    )
    return palette[int(type_id) % len(palette)]


__all__ = [
    "FrameCounter",
    "hud_lines",
    "mode_lines",
    "collision_lines",
    "scene_lines",
    "geometry_lines",
    "is_collision_flash",
    "hud_text",
    "danger_cells",
    "prediction_polylines",
    "player_path",
    "color_for_type",
]

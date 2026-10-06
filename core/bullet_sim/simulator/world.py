"""The deterministic simulation kernel.

``World`` owns *all* mutable simulation state and is the single place where the
step ordering is defined.  It has no knowledge of rendering, AI, gym or
datasets - the only outward hooks are:

* an :class:`~bullet_sim.core.actions.ActionCodec` (action -> velocity),
* a :class:`~bullet_sim.collision.base.CollisionModel` (overlap test),
* a :class:`~bullet_sim.simulator.rewards.RewardFunction`,
* an optional list of step listeners (used by the dataset recorder).

Step ordering (fixed, part of the reproducibility contract)
-----------------------------------------------------------
1. encode the action and integrate the player with the fixed ``dt``
2. advance ``step_index`` / ``sim_time``
3. spawn every :class:`SpawnEvent` scheduled for the new step
4. integrate all live bullets
5. apply the boundary policy, then expire bullets past their TTL
6. detect collisions
7. compute reward / terminated / truncated
8. publish the step record

Because step 1 happens before step 3, ``aimed`` emitters track the *post-move*
player position.
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Any, Callable, Mapping, Sequence

import numpy as np

from bullet_sim.collision.base import CollisionModel, CollisionResult
from bullet_sim.collision.events import ContactTracker
from bullet_sim.collision.grid import make_collision_model
from bullet_sim.collision.shapes import Hitbox, make_hitbox
from bullet_sim.core.actions import ActionCodec, codec_from_spec
from bullet_sim.core.errors import ConfigError, StateError
from bullet_sim.core.numerics import INT_DTYPE
from bullet_sim.core.rng import SeedManager
from bullet_sim.core.state import WorldSnapshot
from bullet_sim.entities.bullet import BulletPool
from bullet_sim.entities.player import P_RADIUS, P_X, P_Y, PlayerState
from bullet_sim.generators.burst import (
    ORIGIN_PLAYER,
    GenerationContext,
    SpawnEvent,
    SpawnTimeline,
)
from bullet_sim.physics.bounds import cull_out_of_bounds, expire_bullets
from bullet_sim.physics.kinematics import apply_action
from bullet_sim.physics.motion import advance_bullets_n, integrate_bullets
from bullet_sim.scenarios.builder import BuiltScenario, build_from_spec, build_scenario
from bullet_sim.core.clock import FixedClock  # noqa: F401  (re-exported usage below)
from bullet_sim.generators.patterns import generate_pattern
from bullet_sim.generators.spec import PatternSpec, pattern_from
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.rewards import RewardFunction, make_reward

# --------------------------------------------------------------------------


@dataclass
class StepRecord:
    """Everything that happened during one ``World.step`` call."""

    step_index: int
    sim_time: float
    action: Any
    action_vector: np.ndarray
    state: WorldSnapshot
    collision: CollisionResult
    reward: float
    terminated: bool
    truncated: bool
    info: dict
    state_hash: str | None = None


@dataclass
class StepResult:
    """Public return value of :meth:`World.step`."""

    state: WorldSnapshot
    reward: float
    terminated: bool
    truncated: bool
    info: dict

    @property
    def done(self) -> bool:
        return self.terminated or self.truncated


# --------------------------------------------------------------------------


class World:
    """Fixed-step, deterministic 2D dynamic-obstacle world."""

    def __init__(
        self,
        scenario: ScenarioSpec | BuiltScenario | Mapping[str, Any] | str | int | float,
        *,
        seed: int | None = None,
        codec: ActionCodec | str | Mapping[str, Any] | None = None,
        collision: CollisionModel | str | None = None,
        collision_kwargs: Mapping[str, Any] | None = None,
        reward: RewardFunction | str | None = "survival",
        reward_kwargs: Mapping[str, Any] | None = None,
        player_hitbox: Hitbox | str | Mapping[str, Any] | None = None,
        risk_radius: float = 32.0,
        dtype: Any = None,
        terminate_on_collision: bool | None = None,
        collision_count_mode: str | None = None,
        success_terminates_episode: bool | None = None,
        clamp_player: bool = True,
        hash_steps: bool = False,
        compact_every: int = 0,
        listeners: Sequence[Callable[[str, Any], None]] | None = None,
    ) -> None:
        if isinstance(scenario, BuiltScenario):
            self.built: BuiltScenario = scenario
            self.spec: ScenarioSpec = scenario.spec
            if seed is not None:
                self.spec = self.spec.replaced(seed=int(seed))
                self.built = build_from_spec(self.spec, profile=scenario.profile)
        else:
            self.built = build_scenario(scenario, seed=seed)
            self.spec = self.built.spec

        self.clock = FixedClock.make(dt=self.spec.dt)
        self.timeline: SpawnTimeline = self.built.timeline
        self.dtype = dtype
        # ----------------------------------------------------------------
        # Collision is a *penalty event*, not a process-ending event.
        #
        # ``terminate_on_collision`` used to default to True; it now defaults to
        # the scenario's ``collision_terminates_episode`` (False).  Passing it
        # explicitly still works, which is how "Mode B: survival time" runs are
        # expressed - see docs/COLLISION_AND_EPISODE_END.md.
        # ----------------------------------------------------------------
        self.terminate_on_collision = bool(
            self.spec.collision_terminates_episode
            if terminate_on_collision is None
            else terminate_on_collision
        )
        self.success_terminates_episode = bool(
            self.spec.success_terminates_episode
            if success_terminates_episode is None
            else success_terminates_episode
        )
        self.contacts = ContactTracker(
            collision_count_mode or self.spec.collision_count_mode
        )
        self.cumulative_reward = 0.0
        self.last_collision_frame: int | None = None
        self._terminate_requested = False
        #: obstacles dropped because they would have spawned on the player
        self._spawn_rejected = 0
        self.clamp_player = bool(clamp_player)
        self.hash_steps = bool(hash_steps)
        self.compact_every = int(compact_every)

        self.codec: ActionCodec = codec_from_spec(
            codec if codec is not None else self.spec.action_space
        )
        #: Player hitbox shape (circle by default).  When a non-circle hitbox is
        #: requested the collision backend is upgraded to the shaped backend,
        #: which supports circle / point / rect / custom SDFs.
        self.player_hitbox: Hitbox = make_hitbox(
            player_hitbox, radius=float(self.spec.player_radius)
        )
        ck = dict(collision_kwargs or {})
        if collision is not None and not isinstance(collision, str):
            self.collision: CollisionModel = collision
        else:
            kind = str(collision if collision is not None else self.spec.collision or "auto")
            if kind == "auto":
                # A scenario made of dynamic obstacles needs the obstacle-shaped
                # backend; a classic emitters-only scenario keeps the faster one.
                kind = "obstacle" if _scenario_has_rect_obstacles(self.spec) else "circle"
            if self.player_hitbox.shape != "circle" and kind == "circle":
                kind = "shaped"
            if kind == "shaped":
                ck.setdefault("hitbox", self.player_hitbox)
                ck.setdefault("risk_radius", risk_radius)
            self.collision = make_collision_model(kind, **ck)
        rk = dict(reward_kwargs or {})
        rk.setdefault("collision_penalty", float(self.spec.collision_penalty))
        self.reward: RewardFunction = (
            reward
            if reward is not None and not isinstance(reward, str)
            else make_reward(str(reward or "survival"), **rk)
        )

        self._listeners: list[Callable[[str, Any], None]] = list(listeners or [])
        self._rng = SeedManager(int(self.spec.seed))

        capacity = int(
            self.spec.bullet_capacity
            if self.spec.bullet_capacity is not None
            else max(64, self.built.peak_bullets + 64)
        )
        self.state = WorldSnapshot.initial(
            field_w=self.spec.field_w,
            field_h=self.spec.field_h,
            dt=self.spec.dt,
            player=self.spec.player_state(),
            target=self.spec.target_state(),
            bullet_capacity=capacity,
            wrap=self.spec.wrap,
            bullet_budget=capacity,
            scenario_id=self.spec.name,
            seed=self.spec.seed,
            dtype=dtype,
        )
        self._player = self.state.player.to_array()
        self._action_count = 0
        self._collision_model = self.collision
        self._configure_collision()
        self.reset()

    def _configure_collision(self) -> None:
        """Let a collision backend learn the workspace bounds (grid broadphase)."""
        configure = getattr(self.collision, "configure", None)
        if callable(configure):
            configure(self.spec.field_w, self.spec.field_h, self.spec.cull_margin)

    # ==================================================================
    # lifecycle
    # ==================================================================
    def reset(self, seed: int | None = None) -> WorldSnapshot:
        """Reset to ``S_0``; an explicit seed rebuilds the spawn timeline."""
        if seed is not None and int(seed) != int(self.spec.seed):
            self.spec = self.spec.replaced(seed=int(seed))
            self.built = build_from_spec(self.spec, profile=self.built.profile)
            self.timeline = self.built.timeline
            if self.spec.bullet_capacity is not None:
                cap = int(self.spec.bullet_capacity)
            else:
                cap = max(64, self.built.peak_bullets + 64)
            self.state.bullets = BulletPool(cap, dtype=self.dtype)
            self.state.env = self.state.env.__class__(
                field_w=self.spec.field_w,
                field_h=self.spec.field_h,
                dt=self.spec.dt,
                step_index=0,
                sim_time=0.0,
                wrap=self.spec.wrap,
                bullet_budget=cap,
            )
            self.state.scenario_id = self.spec.name
            self.state.seed = int(self.spec.seed)
            self._rng = SeedManager(int(self.spec.seed))
            self._configure_collision()

        pool = self.state.bullets
        pool.clear()
        self._player = self.spec.player_state().to_array()
        self.state.player = PlayerState.from_array(self._player)
        self.state.target = self.spec.target_state()
        self.state.env = self.state.env.__class__(
            field_w=self.spec.field_w,
            field_h=self.spec.field_h,
            dt=self.spec.dt,
            step_index=0,
            sim_time=0.0,
            wrap=self.spec.wrap,
            bullet_budget=pool.capacity,
        )
        self.state.timestamp = 0.0
        self.state.rng_state = self._rng.snapshot() or None
        self._action_count = 0
        self.contacts.reset()
        self._spawn_rejected = 0
        self.cumulative_reward = 0.0
        self.last_collision_frame = None
        self._terminate_requested = False
        self._collision_model.prepare(pool)
        self.reward.reset(self.state)
        # Step-0 emissions exist before the first action so that S_0 is a
        # faithful "scene already running" state.
        self._spawn_due(0)
        self._emit("reset", self.state)
        return self.state

    def close(self) -> None:
        self._listeners.clear()

    # ==================================================================
    # stepping
    # ==================================================================
    def step(self, action: Any) -> StepResult:
        dt = self.spec.dt
        pool = self.state.bullets

        # 1. player
        action_vec = apply_action(
            self._player,
            action,
            self.codec,
            dt,
            field_w=self.spec.field_w,
            field_h=self.spec.field_h,
            clamp=self.clamp_player,
        )

        # 2. time
        step_index = self.state.env.step_index + 1
        sim_time = self.clock.time_of(step_index)
        self.state.env = self.state.env.__class__(
            field_w=self.spec.field_w,
            field_h=self.spec.field_h,
            dt=dt,
            step_index=step_index,
            sim_time=sim_time,
            wrap=self.spec.wrap,
            bullet_budget=pool.capacity,
        )
        self.state.timestamp = sim_time
        self._action_count += 1

        # 3. spawn
        self._spawn_due(step_index)

        # 4. integrate
        integrate_bullets(pool, dt)

        # 5. lifecycle
        cull_out_of_bounds(
            pool, self.spec.field_w, self.spec.field_h, self.spec.cull_margin, self.spec.wrap
        )
        expire_bullets(pool)

        # 6. collision
        self.collision.prepare(pool)
        px = float(self._player[P_X])
        py = float(self._player[P_Y])
        pr = float(self._player[P_RADIUS])
        result = self._query_collision(pool, px, py, pr, step_index)

        # 7. contact events / reward / termination
        #
        # A collision is a *penalty event*: the episode keeps running, the
        # window keeps rendering and the controller keeps receiving
        # observations.  Only explicitly configured conditions can end it.
        self.state.player = PlayerState.from_array(self._player)
        contact = self.contacts.update(result.hit_ids, step_index)
        if contact.events:
            self.last_collision_frame = step_index
        self.collision_count = self.contacts.collision_count
        terminated = self._episode_terminates(contact)
        truncated = bool(step_index >= self.spec.total_steps)
        info: dict[str, Any] = {
            "dt": dt,
            "step_index": step_index,
            "sim_time": sim_time,
            "bullet_count": int(pool.count),
            "action_vector": action_vec,
            "collision": result.to_dict(),
            # --- penalty-event surface (what the reward and the metrics use) ---
            "collision_event": contact.is_new_event,
            "collision_events": contact.events,
            "collision_count": self.contacts.collision_count,
            "collision_active": contact.hit,
            "collision_ids": list(contact.active),
            "collision_entered": list(contact.entered),
            "collision_exited": list(contact.exited),
            "collision_overlap_frames": self.contacts.overlap_frames,
            "spawn_rejected": int(self._spawn_rejected),
            "time_since_collision": (
                None
                if self.contacts.frames_since_event(step_index) is None
                else self.contacts.frames_since_event(step_index) * dt
            ),
            "scenario_id": self.spec.name,
            "seed": self.spec.seed,
        }
        try:
            info["action_code"] = self.codec.normalize(action)
        except Exception:  # pragma: no cover - codec-specific validation
            info["action_code"] = None
        # ``min_distance`` is a first-class collision output now, so every
        # backend reports clearance uniformly (no optional method probing).
        info["clearance"] = (
            result.min_distance if np.isfinite(result.min_distance) else None
        )
        info["risk"] = result.risk
        self.state.rng_state = self._rng.snapshot() or None
        reward = float(self.reward(self.state, self.state, result, info))
        self.cumulative_reward += reward
        info["reward"] = reward
        info["cumulative_reward"] = self.cumulative_reward

        record = StepRecord(
            step_index=step_index,
            sim_time=sim_time,
            action=action,
            action_vector=np.asarray(action_vec, dtype=np.float64),
            state=self.state,
            collision=result,
            reward=reward,
            terminated=terminated,
            truncated=truncated,
            info=info,
            state_hash=self.state_hash() if self.hash_steps else None,
        )

        if self.compact_every and (step_index % self.compact_every == 0):
            pool.compact()
        self._emit("step", record)
        return StepResult(self.state, reward, terminated, truncated, info)

    # ------------------------------------------------------------------
    # ==================================================================
    # episode end
    # ==================================================================
    def _episode_terminates(self, contact: Any) -> bool:
        """Only *configured* conditions end an episode.

        A plain collision is not one of them unless
        ``collision_terminates_episode`` is set (experiment Mode B).  Time
        limits are reported through ``truncated``, not ``terminated``, so that
        learners can tell "the world ended me" from "the budget ran out".
        """
        if self._terminate_requested:
            return True
        if self.terminate_on_collision and contact.events > 0:
            return True
        if self.success_terminates_episode:
            p = self.state.player
            return bool(self.state.target.contains(p.x, p.y))
        return False

    def request_termination(self) -> None:
        """End the episode at the next step (explicit, e.g. from a controller)."""
        self._terminate_requested = True

    def terminate(self) -> None:
        """Alias of :meth:`request_termination`."""
        self.request_termination()

    def episode_metrics(self) -> dict[str, Any]:
        """The episode-level numbers of the collision-as-penalty design."""
        steps = int(self.state.env.step_index)
        dt = float(self.spec.dt)
        survival = steps * dt
        return {
            "steps": steps,
            "survival_time": survival,
            "total_collision_count": int(self.contacts.collision_count),
            "collision_count_mode": self.contacts.mode,
            "collision_events_observed": int(self.contacts.entered_total),
            "overlap_frames": int(self.contacts.overlap_frames),
            "collision_rate_per_step": (
                self.contacts.collision_count / steps if steps else 0.0
            ),
            "collision_rate_per_second": (
                self.contacts.collision_count / survival if survival > 0 else 0.0
            ),
            "cumulative_reward": float(self.cumulative_reward),
            "first_collision_frame": self.contacts.first_event_frame,
            "last_collision_frame": self.contacts.last_event_frame,
            "in_contact": self.contacts.in_contact,
            "active_collision_ids": list(self.contacts.active_ids),
            "collision_terminates_episode": bool(self.terminate_on_collision),
            "success_terminates_episode": bool(self.success_terminates_episode),
        }

    def _query_collision(
        self, pool: BulletPool, px: float, py: float, pr: float, frame: int
    ) -> CollisionResult:
        """Dispatch to the shaped query when the backend supports hitboxes."""
        query_hitbox = getattr(self.collision, "query_hitbox", None)
        if callable(query_hitbox) and self.player_hitbox.shape != "circle":
            return query_hitbox(pool, self.player_hitbox, px, py, frame=frame)
        return self.collision.query(pool, px, py, pr, frame=frame)

    def spawn_pattern(
        self,
        pattern: PatternSpec | Mapping[str, Any] | str,
        *,
        origin: Any = None,
        aim: bool | None = None,
        count: int | None = None,
        speed: float | None = None,
        group_id: int | None = None,
        params: Mapping[str, Any] | None = None,
        **overrides: Any,
    ) -> np.ndarray:
        """Emit a pattern **now**, into the live world.

        This is the specification's ``spawn_pattern(pattern_config)`` entry point:
        game logic (or a script, or a development board) never has to touch
        individual bullets - it describes a pattern and lets the generator
        resolve it.

        Works for any registered pattern kind (``single``, ``radial``, ``spiral``,
        ``aimed``, ``burst``, ``line``, ``wall``, ``random``).  Aimed patterns use
        the *current* player position, so the result is still a deterministic
        function of ``(S_t, pattern)``.
        """
        spec = pattern_from(pattern)
        changes: dict[str, Any] = dict(overrides)
        if origin is not None:
            changes["origin"] = origin
        if aim is not None:
            changes["aim"] = bool(aim)
        if count is not None:
            changes["count"] = int(count)
        if speed is not None:
            changes["speed"] = float(speed)
        if group_id is not None:
            changes["group_id"] = int(group_id)
        if params:
            changes["params"] = {**spec.params, **dict(params)}
        # A live emission is always a single shot at the current step.
        changes.update(start_time=0.0, interval=0.0, repetitions=1)
        spec = spec.replaced(**changes)

        ctx = GenerationContext(
            field_w=self.spec.field_w,
            field_h=self.spec.field_h,
            dt=self.spec.dt,
            duration=self.spec.dt,
            rng=self._rng.generator(f"runtime:{spec.kind}"),
            player_xy=self._player[:2],
            clock=self.clock,
        )
        events = generate_pattern(spec, ctx)
        if not events:
            return np.empty(0, dtype=INT_DTYPE)
        pool = self.state.bullets
        budget = int(self.state.env.bullet_budget)
        slots: list[np.ndarray] = []
        for ev in events:
            ev = SpawnEvent(
                step=self.state.env.step_index,
                origin=ev.origin,
                origin_mode=ev.origin_mode,
                aim=ev.aim,
                group_id=int(ev.group_id),
                pattern=ev.pattern,
                offsets=ev.offsets,
                angles=ev.angles,
                speeds=ev.speeds,
                accel_mag=ev.accel_mag,
                accel_angle=ev.accel_angle,
                radius=ev.radius,
                ttl=ev.ttl,
                type_id=ev.type_id,
                angular_velocity=ev.angular_velocity,
            )
            if budget > 0 and pool.count + ev.n > budget:
                allowed = max(0, budget - pool.count)
                if allowed == 0:
                    continue
                ev = _truncate_event(ev, allowed)
            slots.append(self._write_event(ev, self._player[:2], pool=pool))
        return np.concatenate(slots) if slots else np.empty(0, dtype=INT_DTYPE)

    def _spawn_due(self, step_index: int) -> None:
        events = self.timeline.events_at(step_index)
        if not events:
            return
        pool = self.state.bullets
        player_xy = self._player[:2]
        budget = int(self.state.env.bullet_budget)
        for ev in events:
            if ev.is_empty():
                continue
            if budget > 0 and pool.count + ev.n > budget:
                # Deterministic truncation: emit what the budget allows, in order.
                allowed = max(0, budget - pool.count)
                if allowed == 0:
                    continue
                ev = _truncate_event(ev, allowed)
            self._write_event(ev, player_xy, pool=pool)

    def _write_event(
        self, ev: SpawnEvent, player_xy: np.ndarray, pool: BulletPool | None = None
    ) -> np.ndarray:
        pool = pool if pool is not None else self.state.bullets
        player_xy = np.asarray(player_xy, dtype=np.float64).reshape(2)
        pos, vel, acc = ev.resolve_velocities(
            player_xy if (ev.aim or ev.origin_mode == ORIGIN_PLAYER) else None
        )

        # "Never materialise on top of the agent": drop any obstacle whose
        # spawn position is closer to the live player than the event allows.
        # Without this, a boundary emitter plus a cornered player would still
        # pop an obstacle into the agent's lap.
        keep = np.ones(ev.n, dtype=bool)
        min_gap = float(getattr(ev, "min_player_distance", 0.0) or 0.0)
        if min_gap > 0.0 and ev.n:
            d = np.hypot(pos[:, 0] - player_xy[0], pos[:, 1] - player_xy[1])
            keep = d >= min_gap
            self._spawn_rejected += int(np.count_nonzero(~keep))
            if not keep.any():
                return np.empty(0, dtype=INT_DTYPE)

        return pool.spawn(
            x=pos[keep, 0],
            y=pos[keep, 1],
            vx=vel[keep, 0],
            vy=vel[keep, 1],
            ax=acc[keep, 0],
            ay=acc[keep, 1],
            radius=np.asarray(ev.radius)[keep] if np.ndim(ev.radius) else ev.radius,
            ttl=np.asarray(ev.ttl)[keep] if np.ndim(ev.ttl) else ev.ttl,
            type_id=np.asarray(ev.type_id)[keep] if np.ndim(ev.type_id) else ev.type_id,
            group_id=int(ev.group_id),
            angular_velocity=ev.angular_velocity[keep] if ev.angular_velocity.size else None,
            shape=ev.shape[keep] if ev.shape.size else 0,
            half_w=ev.half_w[keep] if ev.half_w.size else None,
            half_h=ev.half_h[keep] if ev.half_h.size else None,
            rotation=ev.rotation[keep] if ev.rotation.size else 0.0,
        )

    # ==================================================================
    # external injection
    # ==================================================================
    def spawn(self, event: SpawnEvent) -> np.ndarray:
        """Inject a batch of bullets immediately (debug / external sensor input)."""
        if event.step != self.state.env.step_index:
            event = SpawnEvent(
                step=self.state.env.step_index,
                origin=event.origin,
                origin_mode=event.origin_mode,
                aim=event.aim,
                group_id=event.group_id,
                pattern=event.pattern,
                offsets=event.offsets,
                angles=event.angles,
                speeds=event.speeds,
                accel_mag=event.accel_mag,
                accel_angle=event.accel_angle,
                radius=event.radius,
                ttl=event.ttl,
                type_id=event.type_id,
                angular_velocity=event.angular_velocity,
            )
        return self._write_event(event, self._player[:2])

    def spawn_bullets(
        self,
        x: Any,
        y: Any,
        vx: Any,
        vy: Any,
        *,
        ax: Any = 0.0,
        ay: Any = 0.0,
        radius: Any = 3.0,
        ttl: Any = 20.0,
        type_id: Any = 0,
        group_id: Any = 0,
    ) -> np.ndarray:
        """Convenience bulk spawn (used heavily by unit tests)."""
        return self.state.bullets.spawn(
            x, y, vx, vy, ax=ax, ay=ay, radius=radius, ttl=ttl,
            type_id=type_id, group_id=group_id,
        )

    # ==================================================================
    # state access
    # ==================================================================
    def get_state(self) -> WorldSnapshot:
        """A private copy of ``S_t`` - mutating it cannot corrupt the world."""
        self.state.player = PlayerState.from_array(self._player)
        return self.state.copy()

    def set_state(self, snapshot: WorldSnapshot) -> None:
        """Restore a snapshot (capacity must be compatible)."""
        if snapshot.bullets.capacity > self.state.bullets.capacity:
            raise StateError(
                f"snapshot capacity {snapshot.bullets.capacity} exceeds world capacity "
                f"{self.state.bullets.capacity}"
            )
        self.state = snapshot.copy()
        self._player = self.state.player.to_array()
        self._rng.restore(snapshot.rng_state)

    def player_state(self) -> PlayerState:
        return PlayerState.from_array(self._player)

    @property
    def player(self) -> np.ndarray:
        """Live player vector ``float64[7]`` (read-only by convention)."""
        return self._player

    def active_bullet_count(self) -> int:
        return int(self.state.bullets.count)

    def state_hash(self) -> str:
        self.state.player = PlayerState.from_array(self._player)
        return self.state.state_hash()

    def is_truncated(self) -> bool:
        return bool(self.state.env.step_index >= self.spec.total_steps)

    # ==================================================================
    # clone / future simulation
    # ==================================================================
    def clone(self) -> "World":
        """Deep, independent copy including RNG state and collision scratch."""
        other = World.__new__(World)
        other.spec = self.spec
        other.built = self.built
        other.clock = self.clock
        other.timeline = self.timeline
        other.dtype = self.dtype
        other.terminate_on_collision = self.terminate_on_collision
        other.success_terminates_episode = self.success_terminates_episode
        other.contacts = copy.deepcopy(self.contacts)
        other.cumulative_reward = self.cumulative_reward
        other.last_collision_frame = self.last_collision_frame
        other._terminate_requested = self._terminate_requested
        other._spawn_rejected = self._spawn_rejected
        other.clamp_player = self.clamp_player
        other.hash_steps = self.hash_steps
        other.compact_every = self.compact_every
        other.codec = self.codec
        # clone() must carry everything step() reads, or a cloned world behaves
        # differently from its parent (this bit ``player_hitbox`` before).
        other.player_hitbox = self.player_hitbox
        other.collision = copy.deepcopy(self.collision)
        other._collision_model = other.collision
        other.reward = copy.deepcopy(self.reward)
        other._listeners = []
        other._rng = SeedManager(self.spec.seed)
        other._rng.restore(self._rng.snapshot() or None)
        self.state.player = PlayerState.from_array(self._player)
        other.state = self.state.copy()
        other._player = self._player.copy()
        other._action_count = self._action_count
        other.collision.prepare(other.state.bullets)
        return other

    def clone_state(self) -> WorldSnapshot:
        """Snapshot independent of the world (alias of :meth:`get_state`)."""
        return self.get_state()

    def restore_state(self, snapshot: WorldSnapshot) -> None:
        self.set_state(snapshot)

    def simulate_future(
        self,
        action_sequence: Sequence[Any] | Any,
        horizon: int | None = None,
        *,
        stride: int = 1,
        include_initial: bool = True,
    ) -> list[WorldSnapshot]:
        """Roll ``S_t -> S_{t+H}`` on a clone; the world itself is untouched.

        ``action_sequence`` may be a single action (held constant) or a sequence
        of per-step actions.  Returns snapshots at steps ``t, t+stride, ...``.
        """
        rollout = self.clone()
        h = int(horizon) if horizon is not None else _infer_horizon(action_sequence)
        stride = max(1, int(stride))
        actions = _as_action_stream(action_sequence, h)
        out: list[WorldSnapshot] = []
        if include_initial:
            out.append(rollout.get_state())
        for i in range(h):
            rollout.step(actions[i])
            if (i + 1) % stride == 0:
                out.append(rollout.get_state())
        return out

    def rollout_final(
        self, action_sequence: Sequence[Any] | Any, horizon: int | None = None
    ) -> tuple[WorldSnapshot, list[float]]:
        """Like :meth:`simulate_future` but returns only the final snapshot + rewards."""
        rollout = self.clone()
        h = int(horizon) if horizon is not None else _infer_horizon(action_sequence)
        actions = _as_action_stream(action_sequence, h)
        rewards: list[float] = []
        for i in range(h):
            res = rollout.step(actions[i])
            rewards.append(res.reward)
            if res.terminated:
                break
        return rollout.get_state(), rewards

    def advance(self, steps: int, action: Any = 0) -> WorldSnapshot:
        """Headless fast-forward with a constant action (used by benchmarks)."""
        n = int(steps)
        for _ in range(n):
            self.step(action)
        return self.state

    def fast_forward_bullets(self, steps: int) -> None:
        """Closed-form bullet jump (no collision/expiry) for offline analysis."""
        advance_bullets_n(self.state.bullets, self.spec.dt, int(steps))

    # ==================================================================
    # listeners (dataset recorder hook)
    # ==================================================================
    def add_listener(self, listener: Callable[[str, Any], None]) -> None:
        self._listeners.append(listener)

    def remove_listener(self, listener: Callable[[str, Any], None]) -> None:
        try:
            self._listeners.remove(listener)
        except ValueError:
            pass

    def _emit(self, event: str, payload: Any) -> None:
        for listener in self._listeners:
            listener(event, payload)

    # ==================================================================
    def policy_describe(self) -> dict[str, Any]:
        """Environment policy that must be reproduced for a faithful replay.

        Recorded alongside every episode so that replaying a dataset does not
        silently use different termination/collision/reward settings.
        """
        return {
            # ``terminate_on_collision`` is kept as the serialised name so old
            # datasets keep replaying; the value is the resolved flag.
            "terminate_on_collision": bool(self.terminate_on_collision),
            "collision_terminates_episode": bool(self.terminate_on_collision),
            "collision_penalty": float(self.spec.collision_penalty),
            "collision_count_mode": str(self.contacts.mode),
            "success_terminates_episode": bool(self.success_terminates_episode),
            "clamp_player": bool(self.clamp_player),
            "collision": str(self.collision.name),
            "reward": str(getattr(self.reward, "name", type(self.reward).__name__)),
            "action_space": self.codec.describe(),
        }

    def describe(self) -> dict[str, Any]:
        return {
            "scenario": self.spec.name,
            "seed": self.spec.seed,
            "dt": self.spec.dt,
            "fps": self.clock.fps,
            "steps": self.spec.total_steps,
            "field": [self.spec.field_w, self.spec.field_h],
            "bullet_capacity": self.state.bullets.capacity,
            "estimated_peak_bullets": self.built.peak_bullets,
            "action_space": self.codec.describe(),
            "collision": self.collision.describe(),
            "reward": getattr(self.reward, "name", type(self.reward).__name__),
            "wrap": self.spec.wrap,
            "patterns": self.spec.pattern_summary(),
            "timeline": self.timeline.summary(),
            "policy": self.policy_describe(),
            "collision_events": {
                "count_mode": self.contacts.mode,
                "collision_terminates_episode": bool(self.terminate_on_collision),
                "collision_penalty": float(self.spec.collision_penalty),
            },
        }

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return (
            f"World(scenario={self.spec.name!r}, seed={self.spec.seed}, "
            f"step={self.state.env.step_index}/{self.spec.total_steps}, "
            f"bullets={self.state.bullets.count})"
        )


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _scenario_has_rect_obstacles(spec: ScenarioSpec) -> bool:
    """True when any pattern declares a non-circular DynamicObstacle."""
    for pattern in spec.patterns:
        if getattr(pattern, "shape", "circle") != "circle":
            return True
        if str(getattr(pattern, "kind", "")) == "obstacle":
            return True
    return False


def _truncate_event(ev: SpawnEvent, n: int) -> SpawnEvent:
    return SpawnEvent(
        step=ev.step,
        origin=ev.origin,
        origin_mode=ev.origin_mode,
        aim=ev.aim,
        group_id=ev.group_id,
        pattern=ev.pattern,
        offsets=ev.offsets[:n],
        angles=ev.angles[:n],
        speeds=ev.speeds[:n],
        accel_mag=ev.accel_mag[:n],
        accel_angle=ev.accel_angle[:n],
        radius=ev.radius[:n],
        ttl=ev.ttl[:n],
        type_id=ev.type_id[:n],
        angular_velocity=ev.angular_velocity[:n],
    )


def _infer_horizon(action_sequence: Any) -> int:
    if isinstance(action_sequence, np.ndarray):
        return int(action_sequence.shape[0]) if action_sequence.ndim > 0 else 1
    if isinstance(action_sequence, Sequence) and not isinstance(action_sequence, (str, bytes)):
        return len(action_sequence)
    return 1


def _as_action_stream(action_sequence: Any, horizon: int) -> list[Any]:
    if isinstance(action_sequence, np.ndarray):
        if action_sequence.ndim == 0:
            return [action_sequence.item()] * horizon
        return [action_sequence[i] for i in range(min(horizon, action_sequence.shape[0]))] + [
            action_sequence[-1]
        ] * max(0, horizon - action_sequence.shape[0])
    if isinstance(action_sequence, Sequence) and not isinstance(action_sequence, (str, bytes)):
        seq = list(action_sequence)
        if not seq:
            raise ConfigError("action_sequence must not be empty")
        if len(seq) < horizon:
            seq = seq + [seq[-1]] * (horizon - len(seq))
        return seq
    return [action_sequence] * horizon


__all__ = ["World", "StepRecord", "StepResult"]

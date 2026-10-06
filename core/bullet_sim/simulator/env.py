"""Gymnasium-style environment facade.

Provides the familiar ``reset / step / get_state / render / close`` surface
**without depending on Gymnasium** (an adapter is a five-line subclass).  Every
capability the platform needs - headless batch runs, dataset recording, future
simulation, external controllers, rendering - hangs off this one object.
"""

from __future__ import annotations

from typing import Any, Callable, Mapping, Sequence

import numpy as np

from bullet_sim.collision.base import CollisionModel
from bullet_sim.core.actions import ActionCodec
from bullet_sim.core.state import WorldSnapshot
from bullet_sim.interface.controller import ActionProvider, controller_from
from bullet_sim.interface.space import (
    Box,
    Discrete,
    ObservationEncoder,
    action_space_for,
    observation_space_for,
)
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.rewards import RewardFunction
from bullet_sim.simulator.world import StepResult, World


class BulletHellEnv:
    """The public simulator API."""

    metadata = {"render_modes": ["human", "rgb_array", "ascii", "none"]}

    def __init__(
        self,
        scenario: ScenarioSpec | Mapping[str, Any] | str | int | float,
        *,
        seed: int | None = None,
        codec: ActionCodec | str | Mapping[str, Any] | None = None,
        collision: CollisionModel | str = "circle",
        collision_kwargs: Mapping[str, Any] | None = None,
        reward: RewardFunction | str | None = "survival",
        reward_kwargs: Mapping[str, Any] | None = None,
        render_mode: str | None = None,
        normalize: bool = False,
        max_bullets: int | None = None,
        encoder: ObservationEncoder | None = None,
        record: Any = None,
        dtype: Any = None,
        terminate_on_collision: bool | None = None,
        collision_count_mode: str | None = None,
        success_terminates_episode: bool | None = None,
        hash_steps: bool = False,
        compact_every: int = 0,
        renderer: Any = None,
        **world_kwargs: Any,
    ) -> None:
        self.world = World(
            scenario,
            seed=seed,
            codec=codec,
            collision=collision,
            collision_kwargs=collision_kwargs,
            reward=reward,
            reward_kwargs=reward_kwargs,
            dtype=dtype,
            terminate_on_collision=terminate_on_collision,
            collision_count_mode=collision_count_mode,
            success_terminates_episode=success_terminates_episode,
            hash_steps=hash_steps,
            compact_every=compact_every,
            **world_kwargs,
        )
        self.encoder = encoder or ObservationEncoder(
            normalize=normalize, max_bullets=max_bullets
        )
        self.render_mode = render_mode
        self.action_space = action_space_for(self.world.codec)
        self.observation_space = observation_space_for(self.encoder)

        self.recorder = record
        if record is not None and hasattr(record, "listener"):
            self.world.add_listener(record.listener)

        self._renderer = renderer
        self._last_obs: dict[str, Any] | None = None
        self._last_info: dict[str, Any] = {}
        self._closed = False

    # ==================================================================
    # spaces / description
    # ==================================================================
    @property
    def codec(self) -> ActionCodec:
        return self.world.codec

    @property
    def spec(self) -> ScenarioSpec:
        return self.world.spec

    def describe(self) -> dict[str, Any]:
        return {
            "world": self.world.describe(),
            "observation_space": self.observation_space,
            "action_space": self.action_space.to_dict()
            if hasattr(self.action_space, "to_dict")
            else str(self.action_space),
            "encoder": self.encoder.describe(),
            "render_mode": self.render_mode,
        }

    # ==================================================================
    # core API
    # ==================================================================
    def reset(self, seed: int | None = None, options: Mapping[str, Any] | None = None):
        """Reset to ``S_0``.  ``seed`` deterministically rebuilds the scene."""
        opts = dict(options or {})
        snapshot = self.world.reset(seed=seed)
        obs = self.encoder.encode(snapshot)
        obs["reward"] = np.float32(0.0)
        obs["collision"] = False
        info: dict[str, Any] = {
            "dt": float(snapshot.env.dt),
            "step_index": snapshot.env.step_index,
            "sim_time": snapshot.env.sim_time,
            "bullet_count": snapshot.bullet_count,
            "scenario_id": self.spec.name,
            "seed": self.spec.seed,
            "state_hash": self.world.state_hash(),
        }
        if seed is not None and int(seed) != int(snapshot.seed):
            # ``World.reset`` already rebuilt the timeline; keep the env in sync.
            self.action_space = action_space_for(self.world.codec)
        info.update(opts)
        if self.recorder is not None and hasattr(self.recorder, "note_env"):
            self.recorder.note_env(
                {
                    "policy": self.world.policy_describe(),
                    "scenario": self.spec.to_dict(),
                }
            )
        self._last_obs, self._last_info = obs, info
        return obs, info

    def step(self, action: Any):
        """Advance one fixed timestep.

        Returns ``(observation, reward, terminated, truncated, info)``.
        """
        if self._closed:
            raise RuntimeError("env is closed")
        result: StepResult = self.world.step(action)
        obs = self.encoder.encode(result.state)
        obs["reward"] = np.float32(result.reward)
        obs["collision"] = bool(result.info["collision_active"])
        obs["collision_event"] = bool(result.info["collision_event"])
        obs["collision_count"] = int(result.info["collision_count"])
        self._last_obs, self._last_info = obs, result.info
        return obs, result.reward, result.terminated, result.truncated, result.info

    def get_state(self) -> WorldSnapshot:
        """Full, serializable ``S_t`` (a private copy)."""
        return self.world.get_state()

    def set_state(self, snapshot: WorldSnapshot) -> None:
        self.world.set_state(snapshot)

    def clone_state(self) -> WorldSnapshot:
        return self.world.clone_state()

    def restore_state(self, snapshot: WorldSnapshot) -> None:
        self.world.restore_state(snapshot)

    def terminate(self) -> None:
        """Request an explicit episode end (controller/UI initiated).

        This is one of the *legitimate* ways to end an episode; a collision is
        not, unless the scenario asked for it.
        """
        self.world.request_termination()

    def episode_metrics(self) -> dict[str, Any]:
        """total_collision_count / collision_rate / survival_time / reward ..."""
        return self.world.episode_metrics()

    def state_hash(self) -> str:
        return self.world.state_hash()

    def active_bullet_count(self) -> int:
        return self.world.active_bullet_count()

    def simulate_future(
        self,
        action_sequence: Sequence[Any] | Any,
        horizon: int | None = None,
        *,
        stride: int = 1,
        include_initial: bool = True,
    ) -> list[WorldSnapshot]:
        """Roll the *current* state forward on a clone; this env is unchanged."""
        return self.world.simulate_future(
            action_sequence, horizon, stride=stride, include_initial=include_initial
        )

    def evaluate_actions(
        self, candidate_actions: Sequence[Any], horizon: int
    ) -> list[dict[str, Any]]:
        """Score each candidate action by rolling ``horizon`` steps.

        Returns one dict per candidate (final state, cumulative reward, survival
        steps and minimum clearance) - the raw material for risk assessment,
        planning and MPC.
        """
        out: list[dict[str, Any]] = []
        for action in candidate_actions:
            snapshots = self.world.simulate_future(action, horizon, include_initial=False)
            final = snapshots[-1] if snapshots else self.world.get_state()
            out.append(
                {
                    "action": action,
                    "final_state": final,
                    "steps_simulated": len(snapshots),
                    "final_bullet_count": final.bullet_count,
                    "final_target_distance": final.target.distance_to(
                        final.player.x, final.player.y
                    ),
                }
            )
        return out

    # ==================================================================
    # convenience run loops
    # ==================================================================
    def make_input_source(
        self,
        controller: Any = None,
        input_source: Any = None,
        *,
        seed: int | None = None,
    ):
        """Resolve the ``ActionSource`` that will drive this episode.

        Both control modes converge here: a manual keyboard source, a scripted
        replay, or a development-board adapter are all just ActionSource
        implementations, so the environment logic below is identical for all of
        them.
        """
        from bullet_sim.action.base import ControllerSource, source_from

        if input_source is not None:
            return source_from(input_source, seed=seed if seed is not None else self.spec.seed)
        provider = controller_from(
            controller, space=self.action_space, seed=seed or self.spec.seed
        )
        return ControllerSource(provider)

    def run(
        self,
        controller: ActionProvider | Callable[..., Any] | None = None,
        steps: int | None = None,
        *,
        seed: int | None = None,
        stop_on_done: bool = True,
        input_source: Any = None,
    ) -> dict[str, Any]:
        """Headless episode loop for **any** input source.

        ``controller`` accepts a policy/controller (mode C: external program);
        ``input_source`` accepts an explicit ``ActionSource`` - a manual keyboard
        source (mode A), a scripted replay, or a development-board adapter
        (mode B).  All of them produce the same ``Action`` and go through the
        same ``World.step``.
        """
        from bullet_sim.action.base import (
            Action,
            ControllerSource,
            action_to_codec_input,
        )
        from bullet_sim.entities.player import P_SPEED

        source = self.make_input_source(controller, input_source, seed=seed)
        obs, info = self.reset(seed=seed)
        source.open()
        if isinstance(source, ControllerSource):
            source.reset(obs, info)
        # Any source exposing ``poll_observation`` (controllers, models, or a
        # SwitchableSource wrapping them) is driven by the observation; plain
        # device sources are driven by ``poll(dt)``.
        needs_observation = callable(getattr(source, "poll_observation", None))
        last_action = Action.zero()
        n = int(steps) if steps is not None else self.spec.total_steps
        rewards: list[float] = []
        collision_events = 0
        terminated = False
        truncated = False
        info = self._last_info
        for _ in range(n):
            if needs_observation:
                action = source.poll_observation(obs, info)
            else:
                polled = source.poll(self.spec.dt)
                action = polled if polled is not None else last_action
            last_action = action
            raw = action_to_codec_input(
                action, self.world.codec, float(self.world.player[P_SPEED])
            )
            obs, reward, terminated, truncated, info = self.step(raw)
            rewards.append(float(reward))
            collision_events += int(info["collision_events"])
            if stop_on_done and (terminated or truncated):
                break
        source.close()
        metrics = self.world.episode_metrics()
        return {
            "steps": len(rewards),
            "return": float(np.sum(rewards)),
            # ``collisions`` is now the *event* count (not overlapping frames)
            "collisions": int(metrics["total_collision_count"]),
            "collision_count": int(metrics["total_collision_count"]),
            "collision_events": int(collision_events),
            "collision_rate_per_step": float(metrics["collision_rate_per_step"]),
            "collision_rate_per_second": float(metrics["collision_rate_per_second"]),
            "survival_time": float(metrics["survival_time"]),
            "collision_terminates_episode": bool(metrics["collision_terminates_episode"]),
            "episode_metrics": metrics,
            "bullet_count": self.world.active_bullet_count(),
            "state_hash": self.world.state_hash(),
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "final_info": info,
            "input_source": getattr(source, "name", type(source).__name__),
            "input_mode": getattr(source, "mode", getattr(source, "name", "?")),
            "mode_switches": int(getattr(source, "switch_count", 0)),
            "last_action": last_action.to_dict(),
        }

    # ==================================================================
    # rendering
    # ==================================================================
    def render(self, mode: str | None = None, **kwargs: Any):
        """Draw the *current* state.  Never mutates simulation state."""
        mode = mode or self.render_mode or "none"
        if mode == "none":
            return None
        if mode == "ascii":
            from bullet_sim.render.ascii_view import AsciiRenderer

            renderer = AsciiRenderer(**kwargs)
            text = renderer.render(self.world)
            print(text)
            return text
        if self._renderer is None:
            from bullet_sim.render.base import make_renderer

            self._renderer = make_renderer(mode, **kwargs)
        return self._renderer.render(self.world)

    def close(self) -> None:
        if self._renderer is not None and hasattr(self._renderer, "close"):
            self._renderer.close()
        self._renderer = None
        self.world.close()
        self._closed = True

    # ==================================================================
    def __enter__(self) -> "BulletHellEnv":
        return self

    def __exit__(self, *exc: Any) -> None:
        self.close()

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return f"BulletHellEnv({self.world!r})"


# --------------------------------------------------------------------------
# batch helpers
# --------------------------------------------------------------------------


def make_envs(
    scenarios: Sequence[Any],
    *,
    base_seed: int = 0,
    seed_stride: int = 1000003,
    **kwargs: Any,
) -> list[BulletHellEnv]:
    """Build one environment per scenario with reproducible, distinct seeds."""
    out: list[BulletHellEnv] = []
    for i, sc in enumerate(scenarios):
        seed = base_seed + i * seed_stride
        if isinstance(sc, ScenarioSpec):
            sc = sc.replaced(seed=seed)
        out.append(BulletHellEnv(sc, **kwargs))
    return out


__all__ = ["BulletHellEnv", "make_envs", "Box", "Discrete"]

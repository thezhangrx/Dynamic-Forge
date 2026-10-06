"""Replay: ``scenario + seed + action sequence -> trajectory``.

This is the executable form of the reproducibility contract.  ``Replay`` is
also what the dataset verifier and the regression tests use: recording an
episode and replaying it must produce identical per-step state hashes.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Sequence

import numpy as np

from bullet_sim.core.errors import DatasetError
from bullet_sim.interface.controller import ActionProvider, ScriptedController
from bullet_sim.scenarios.spec import ScenarioSpec, scenario_from
from bullet_sim.simulator.env import BulletHellEnv


@dataclass
class ReplayResult:
    """Full trajectory produced by a replay."""

    actions: list[Any] = field(default_factory=list)
    rewards: list[float] = field(default_factory=list)
    terminated: list[bool] = field(default_factory=list)
    truncated: list[bool] = field(default_factory=list)
    state_hashes: list[str] = field(default_factory=list)
    bullet_counts: list[int] = field(default_factory=list)
    initial_hash: str = ""
    scenario_id: str = ""
    seed: int = 0

    @property
    def steps(self) -> int:
        return len(self.actions)

    @property
    def total_reward(self) -> float:
        return float(np.sum(self.rewards)) if self.rewards else 0.0

    def to_dict(self) -> dict:
        return {
            "scenario_id": self.scenario_id,
            "seed": self.seed,
            "steps": self.steps,
            "total_reward": self.total_reward,
            "initial_hash": self.initial_hash,
            "final_hash": self.state_hashes[-1] if self.state_hashes else self.initial_hash,
            "state_hashes": list(self.state_hashes),
            "bullet_counts": list(self.bullet_counts),
        }


def replay(
    scenario: ScenarioSpec | Mapping[str, Any] | str,
    actions: Sequence[Any],
    *,
    seed: int | None = None,
    env: BulletHellEnv | None = None,
    **env_kwargs: Any,
) -> ReplayResult:
    """Run ``actions`` on ``scenario`` (or a supplied env) and collect hashes."""
    owns_env = env is None
    if env is None:
        env = BulletHellEnv(scenario, seed=seed, **env_kwargs)
    obs, info = env.reset(seed=seed)
    result = ReplayResult(
        initial_hash=env.state_hash(),
        scenario_id=env.spec.name,
        seed=int(env.spec.seed),
    )
    for action in actions:
        obs, reward, terminated, truncated, info = env.step(action)
        result.actions.append(action)
        result.rewards.append(float(reward))
        result.terminated.append(bool(terminated))
        result.truncated.append(bool(truncated))
        result.state_hashes.append(env.state_hash())
        result.bullet_counts.append(env.active_bullet_count())
        if terminated or truncated:
            break
    if owns_env:
        env.close()
    return result


def replay_with_controller(
    scenario: ScenarioSpec | Mapping[str, Any] | str,
    controller: ActionProvider | Any,
    *,
    seed: int | None = None,
    steps: int | None = None,
    **env_kwargs: Any,
) -> tuple[ReplayResult, list[Any]]:
    """Run a controller and return both the result and the emitted actions."""
    env = BulletHellEnv(scenario, seed=seed, **env_kwargs)
    from bullet_sim.interface.controller import controller_from

    ctrl = controller_from(controller, space=env.action_space, seed=seed or 0)
    obs, info = env.reset(seed=seed)
    ctrl.reset(obs, info)
    n = int(steps) if steps is not None else env.spec.total_steps
    actions: list[Any] = []
    result = ReplayResult(
        initial_hash=env.state_hash(), scenario_id=env.spec.name, seed=int(env.spec.seed)
    )
    for _ in range(n):
        action = ctrl.act(obs, info)
        actions.append(action)
        obs, reward, terminated, truncated, info = env.step(action)
        result.actions.append(action)
        result.rewards.append(float(reward))
        result.terminated.append(bool(terminated))
        result.truncated.append(bool(truncated))
        result.state_hashes.append(env.state_hash())
        result.bullet_counts.append(env.active_bullet_count())
        if terminated or truncated:
            break
    env.close()
    return result, actions


@dataclass
class ReplayComparison:
    """Result of re-simulating a recorded episode from its scenario + actions."""

    ok: bool
    first_mismatch: int
    expected_steps: int
    actual_steps: int
    reason: str
    policy: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "first_mismatch": self.first_mismatch,
            "expected_steps": self.expected_steps,
            "actual_steps": self.actual_steps,
            "reason": self.reason,
            "policy": self.policy,
        }


def verify_episode(episode: Any, **policy_overrides: Any) -> ReplayComparison:
    """Reproduce a recorded episode using the *policy stored inside it*.

    A dataset records ``meta['env_policy']`` (termination on collision, collision
    model, reward, action space).  Replaying with different settings would
    silently diverge after the first collision, so the recorded policy is the
    default here; ``policy_overrides`` exist only so callers can *prove* that
    assumption wrong.
    """
    from bullet_sim.scenarios.spec import ScenarioSpec

    policy = dict(episode.meta.get("env_policy", {}))
    policy.update(policy_overrides)
    scenario_payload = episode.meta.get("scenario") or episode.meta.get("env", {}).get(
        "scenario"
    )
    if not scenario_payload:
        raise DatasetError(
            "episode does not embed its scenario; re-record with the current "
            "recorder or pass the scenario explicitly to verify_replay()"
        )
    spec = ScenarioSpec.from_dict(scenario_payload)
    actions = [int(a) for a in episode.arrays["actions"]]
    hashes = [str(h) for h in episode.arrays["state_hashes"]][1:]
    terminated_by_collision = policy.get("terminate_on_collision")
    if terminated_by_collision is None:
        terminated_by_collision = policy.get(
            "collision_terminates_episode", spec.collision_terminates_episode
        )
    env_kwargs = {
        "collision": policy.get("collision", "circle"),
        "reward": policy.get("reward", "survival"),
        "terminate_on_collision": bool(terminated_by_collision),
        "collision_count_mode": policy.get(
            "collision_count_mode", spec.collision_count_mode
        ),
    }
    got = replay(spec, actions, seed=episode.seed, **env_kwargs)
    for i, (a, b) in enumerate(zip(got.state_hashes, hashes)):
        if a != b:
            return ReplayComparison(
                ok=False, first_mismatch=i, expected_steps=len(hashes),
                actual_steps=len(got.state_hashes),
                reason="state hash mismatch", policy=policy,
            )
    if len(got.state_hashes) != len(hashes):
        return ReplayComparison(
            ok=False, first_mismatch=min(len(got.state_hashes), len(hashes)),
            expected_steps=len(hashes), actual_steps=len(got.state_hashes),
            reason=(
                "episode ended early - the replay policy terminated sooner than "
                "the recording (check meta['env_policy'])"
            ),
            policy=policy,
        )
    return ReplayComparison(
        ok=True, first_mismatch=-1, expected_steps=len(hashes),
        actual_steps=len(got.state_hashes), reason="exact match", policy=policy,
    )


def verify_replay(
    scenario: ScenarioSpec | Mapping[str, Any] | str,
    actions: Sequence[Any],
    expected_hashes: Sequence[str],
    *,
    seed: int | None = None,
) -> tuple[bool, int]:
    """Re-run a recorded action sequence and compare per-step state hashes.

    Returns ``(matches, first_mismatch_index)``; ``first_mismatch_index == -1``
    means the whole trajectory matched.
    """
    got = replay(scenario, actions, seed=seed)
    for i, (a, b) in enumerate(zip(got.state_hashes, expected_hashes)):
        if a != b:
            return False, i
    if len(got.state_hashes) != len(expected_hashes):
        return False, min(len(got.state_hashes), len(expected_hashes))
    return True, -1


def controller_for_actions(actions: Sequence[Any]) -> ScriptedController:
    return ScriptedController(actions)


def scenario_from_payload(payload: Mapping[str, Any]) -> ScenarioSpec:
    return scenario_from(payload)


__all__ = [
    "ReplayResult",
    "ReplayComparison",
    "verify_episode",
    "replay",
    "replay_with_controller",
    "verify_replay",
    "controller_for_actions",
]

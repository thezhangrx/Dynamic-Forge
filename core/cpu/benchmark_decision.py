#!/usr/bin/env python3
"""Benchmark: reactive baseline vs predictive planner (v0.4.1-SideHit-ASE).

Runs the *same* 1000 episodes (deterministically derived from seed 42) under two
controllers and compares:

    survival time     - steps survived before the first fatal collision
    collision rate    - fraction of episodes that ended in a collision
    decision latency  - wall-clock time per decide() call
    action smoothness - number of action changes per episode (lower = smoother)
    failure types     - front_hit / side_hit / corner_trap / other

    reactive   : ``bullet_sim.ai.baseline.RepulsionController``
                 (pure reactive reference baseline - kept unchanged; the
                  v0.5.0-ReactiveGap Follow-the-Gap experiment is retained in
                  the codebase but was rejected: collision 94.5%)
    predictive : ``cpu.decision.CpuDecisionLayer``
                 (v0.4.1-SideHit-ASE: FAR + adaptive safety envelope +
                  critical side-hit tier + danger-only refinement +
                  safety-first hysteresis)

Both controllers see exactly the same observation, scenes, seeds and collision
definition; only the algorithm differs.

Run with the project virtualenv::

    .venv/bin/python benchmark_decision.py

Results are printed and written to ``docs/benchmark_results.md``.
"""

from __future__ import annotations

import json
import math
import os
import pathlib
import sys
import time

import numpy as np

# --- 路径引导（core/ 模块化后的目录结构）---------------------------------
# 旧位置：仓库根/benchmark_decision.py     新位置：core/cpu/benchmark_decision.py
#   _CORE = core/     -> 模块源码根（cpu / bullet_sim 都在它下面）
#   _REPO = 仓库根    -> 产物按约定集中到 data/cpu/
_HERE = pathlib.Path(__file__).resolve().parent          # core/cpu/
_CORE = _HERE.parent                                     # core/
_REPO = _CORE.parent                                     # 仓库根
for _p in (str(_CORE), str(_HERE)):
    if _p not in sys.path:
        sys.path.insert(0, _p)

ROOT = _HERE                        # 本模块目录：docs/ 的落点
DATA_DIR = _REPO / "data" / "cpu"    # 产物目录（.gitignore 已排除）

from bullet_sim.action.types import coerce_action
from bullet_sim.ai.base import BaseAutonomousController, ControllerSpec
from bullet_sim.ai.baseline import RepulsionController
from cpu.adapters import observation_to_scene, to_game_action
from cpu.decision import (
    BUILD_ID,
    ENABLE_LTV,
    FSC_SELECTOR_ENABLED,
    LTV_OVERRIDE_GUARD,
    LTV_SHADOW_ONLY,
    PREDICTIVE_VERSION,
    STACK_VERSION,
    CpuDecisionLayer,
    ViabilityConfig,
    ViabilityContext,
)
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


def _short_build(build_id: str = BUILD_ID) -> str:
    return build_id.split("-")[-1]


def _hms(seconds: float) -> str:
    """Format a duration as ``1h02m03s`` / ``2m03s`` / ``7s``."""
    total = int(max(0.0, seconds))
    h, rem = divmod(total, 3600)
    m, s = divmod(rem, 60)
    if h:
        return f"{h}h{m:02d}m{s:02d}s"
    if m:
        return f"{m}m{s:02d}s"
    return f"{s}s"


def _print_startup_banner() -> None:
    line = "=" * 50
    print(line)
    print("Dynamic-Forge Decision Benchmark")
    print(f"Decision Version: {STACK_VERSION}")
    print(f"Build ID: {BUILD_ID}")
    print("Reactive:   RepulsionController (reference baseline)")
    print(f"Predictive: {PREDICTIVE_VERSION}")
    print(f"LTV:        {'enabled' if ENABLE_LTV else 'DISABLED (strict v0.4.1 reference path)'}")
    print(f"LTV shadow: {'ON (diagnostics only, action = v0.4.1)' if LTV_SHADOW_ONLY else 'off'}")
    print("LTV override guard: "
          + (
              f"ON (same risk band, viability gain >= "
              f"{ViabilityConfig().viability_improvement_threshold:.2f})"
              if LTV_OVERRIDE_GUARD
              else "OFF (raw v0.5.1 arbitration)"
          ))
    print(
        "FSC selector: "
        + ("ON (predictive corridor selection over the safe set)"
           if FSC_SELECTOR_ENABLED
           else "off (v0.5.2-LTVGuarded action path)")
    )
    print(line)
    print(line)


_print_startup_banner()

# --------------------------------------------------------------------------
# benchmark configuration (reproducible from BASE_SEED)
# --------------------------------------------------------------------------
BASE_SEED = 42
N_EPISODES = 1000
LEVEL = "medium"
DURATION = 20.0          # seconds of scenario spawn schedule
MAX_STEPS = 1200         # survival cap (10 s @ 120 Hz)
HORIZON = 0.3            # linear-prediction lookahead for the predictive controller

#: Failure classification thresholds (documented heuristics).
CORNER_MARGIN = 0.15     # fraction of the smaller field dimension
FRONT_ANGLE_DEG = 60.0   # impact within this angle of the heading counts as "front"

#: Control frequency of the simulator (Hz).  Diagnostics use it *only* to convert
#: an override-to-collision step distance into seconds; it mirrors the existing
#: ``PredictiveController.dt = 1/120`` and does not feed any decision.
CONTROL_HZ = 120.0


class PredictiveController(BaseAutonomousController):
    """Observation-only predictive controller: v0.4.1 FAR+ASE plus v0.5.1 LTV.

    ``observation -> SceneState -> CpuDecisionLayer.decide -> Action``.
    The controller also derives the LTV scene scale ``T_scene =
    min(field_w, field_h) / max_speed`` from the observation ``env`` block and
    passes it explicitly as a :class:`ViabilityContext`.
    """

    SPEC = ControllerSpec(
        name="predictive",
        kind="baseline",
        description=f"{PREDICTIVE_VERSION}: FAR + ASE + guarded Long-Term Viability (K=4 shortlist, bucketed arbitration, override guard)",
        access="observation",
        cost="moderate",
    )

    def __init__(
        self,
        layer: CpuDecisionLayer | None = None,
        *,
        horizon: float = HORIZON,
        dt: float = 1.0 / 120.0,
    ) -> None:
        super().__init__()
        self.layer = layer or CpuDecisionLayer()
        self.horizon = float(horizon)
        self.dt = float(dt)

    @staticmethod
    def _scene_scale(observation) -> float | None:
        """``T_scene = min(field_w, field_h) / max_speed`` from the obs env block."""
        env = observation.get("env")
        if env is None or len(env) < 2:
            return None
        field_w, field_h = float(env[0]), float(env[1])
        speed = float(observation["player"][5])
        if field_w <= 0.0 or field_h <= 0.0 or speed <= 0.0:
            return None
        return min(field_w, field_h) / speed

    def act(self, observation, info):
        self._calls += 1
        scene = observation_to_scene(
            observation, info, horizon=self.horizon, dt=self.dt, prune=True
        )
        context = ViabilityContext(
            short_horizon=self.horizon,
            scene_scale=self._scene_scale(observation),
        )
        return to_game_action(self.layer.decide(scene, context=context))


def classify_failure(obs: dict, info: dict, field_w: float, field_h: float) -> str:
    """Heuristic failure reason at the fatal step.

    * ``corner_trap`` - player is within a corner region of two field edges;
    * ``front_hit``   - impact direction within ``FRONT_ANGLE_DEG`` of the heading;
    * ``side_hit``    - impact direction outside that cone;
    * ``other``       - stationary / no meaningful heading or impact direction.
    """
    px, py = float(obs["player"][0]), float(obs["player"][1])
    margin = CORNER_MARGIN * min(field_w, field_h)
    near_x = px < margin or px > field_w - margin
    near_y = py < margin or py > field_h - margin
    if near_x and near_y:
        return "corner_trap"

    vx, vy = float(obs["player"][2]), float(obs["player"][3])
    if math.hypot(vx, vy) < 1e-6:
        av = info.get("action_vector")
        if av is not None and math.hypot(float(av[0]), float(av[1])) > 1e-6:
            vx, vy = float(av[0]), float(av[1])
        else:
            return "other"

    col = info.get("collision") or {}
    bx, by = float(col.get("x", float("nan"))), float(col.get("y", float("nan")))
    if not (math.isfinite(bx) and math.isfinite(by)):
        return "other"
    dx, dy = bx - px, by - py
    n_v = math.hypot(vx, vy)
    n_d = math.hypot(dx, dy)
    if n_d < 1e-6:
        return "other"
    cos_a = (vx * dx + vy * dy) / (n_v * n_d)
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cos_a))))
    return "front_hit" if angle <= FRONT_ANGLE_DEG else "side_hit"


def _planner_diagnostics(controller):
    """Planner diagnostics for a predictive controller (``None`` for reactive)."""
    layer = getattr(controller, "layer", None)
    planner = getattr(layer, "planner", None)
    return getattr(planner, "last_diagnostics", None)


def _new_ltv_counts() -> dict:
    return {
        "steps": 0,
        "eval": 0,
        "skip": 0,
        "override": 0,
        "corridor_switch": 0,
        "corridor_hold": 0,
        "corridor_unsafe_switch": 0,
        "emergency": 0,
        # v0.5.2 guard observability
        "guard_candidates": 0,
        "guard_blocked_risk_band": 0,
        "guard_blocked_viability_margin": 0,
        "guard_blocked_long_term_degraded": 0,
        "guard_blocked_critical_band": 0,
        "guard_blocked_no_reference": 0,
        "guard_veto": 0,
        "proposed_override": 0,
        # v0.6.0 Predictive Corridor Selector (default OFF)
        "selector_enabled": 0,
        "selector_applied": 0,
        "selector_changed": 0,
        "selector_hard_survivors": 0,
        "selector_eligible": 0,
        "selector_unknown_candidates": 0,
        "sel_level_reachability": 0,
        "sel_level_persistent": 0,
        "sel_level_max_run": 0,
        "sel_level_clearance": 0,
        "sel_level_existing_tiers": 0,
        "sel_base_reachable": 0,
        "sel_winner_reachable": 0,
        "sel_base_persistent": 0,
        "sel_winner_persistent": 0,
        "sel_base_max_run_t3": 0,
        "sel_winner_max_run_t3": 0,
        "sel_table_us_total": 0.0,
    }


def _accumulate_ltv(counts: dict, diag) -> None:
    """Accumulate LTV observability counters.

    These are *association* statistics only: they never claim that an override
    caused a later collision.
    """
    if diag is None:
        return
    counts["steps"] += 1
    if diag.viability_evaluated > 0:
        counts["eval"] += 1
    if diag.viability_skipped:
        counts["skip"] += 1
    if diag.ltv_override:
        counts["override"] += 1
    if diag.corridor_switched:
        counts["corridor_switch"] += 1
    if diag.corridor_switch_kind == "hold":
        counts["corridor_hold"] += 1
    if diag.corridor_switch_kind == "switch_hard_unsafe":
        counts["corridor_unsafe_switch"] += 1
    if diag.emergency:
        counts["emergency"] += 1
    # -- v0.5.2 override guard -------------------------------------------
    counts["guard_candidates"] += int(getattr(diag, "override_candidate_count", 0) or 0)
    counts["guard_blocked_risk_band"] += int(
        getattr(diag, "override_blocked_by_risk_band", 0) or 0
    )
    counts["guard_blocked_viability_margin"] += int(
        getattr(diag, "override_blocked_by_viability_margin", 0) or 0
    )
    counts["guard_blocked_long_term_degraded"] += int(
        getattr(diag, "override_blocked_by_long_term_degraded", 0) or 0
    )
    counts["guard_blocked_critical_band"] += int(
        getattr(diag, "override_blocked_by_critical_band", 0) or 0
    )
    counts["guard_blocked_no_reference"] += int(
        getattr(diag, "override_blocked_by_no_reference", 0) or 0
    )
    # What the raw LTV pipeline wanted, before the guard / shadow veto.
    proposed = int(getattr(diag, "ltv_proposed_index", -1))
    base = int(getattr(diag, "short_term_best_index", -1))
    if proposed != base:
        counts["proposed_override"] += 1
    if getattr(diag, "override_guard_reason", "") not in ("", "shadow"):
        counts["guard_veto"] += 1
    # -- v0.6.0 Predictive Corridor Selector --------------------------------
    if getattr(diag, "fsc_selector_enabled", False):
        counts["selector_enabled"] += 1
        if getattr(diag, "fsc_selector_applied", False):
            counts["selector_applied"] += 1
            counts["selector_changed"] += 1
        counts["selector_hard_survivors"] += int(
            getattr(diag, "fsc_hard_survivors", 0) or 0
        )
        counts["selector_eligible"] += int(getattr(diag, "fsc_eligible", 0) or 0)
        counts["selector_unknown_candidates"] += int(
            getattr(diag, "fsc_unknown_candidates", 0) or 0
        )
        if getattr(diag, "fsc_triggered_reachability", False):
            counts["sel_level_reachability"] += 1
        if getattr(diag, "fsc_triggered_persistent", False):
            counts["sel_level_persistent"] += 1
        if getattr(diag, "fsc_triggered_max_run", False):
            counts["sel_level_max_run"] += 1
        if getattr(diag, "fsc_triggered_clearance", False):
            counts["sel_level_clearance"] += 1
        if getattr(diag, "fsc_triggered_existing_tiers", False):
            counts["sel_level_existing_tiers"] += 1
        if getattr(diag, "fsc_base_reachable", False):
            counts["sel_base_reachable"] += 1
        if getattr(diag, "fsc_winner_reachable", False):
            counts["sel_winner_reachable"] += 1
        counts["sel_base_persistent"] += int(
            getattr(diag, "fsc_base_persistent", 0) or 0
        )
        counts["sel_winner_persistent"] += int(
            getattr(diag, "fsc_winner_persistent", 0) or 0
        )
        counts["sel_base_max_run_t3"] += int(
            getattr(diag, "fsc_base_max_run_t3", 0) or 0
        )
        counts["sel_winner_max_run_t3"] += int(
            getattr(diag, "fsc_winner_max_run_t3", 0) or 0
        )
        counts["sel_table_us_total"] += float(
            getattr(diag, "fsc_table_us", 0.0) or 0.0
        )


def run_episode(seed: int, controller, max_steps: int = MAX_STEPS) -> dict:
    """Run one episode to a fatal collision (or the cap) and collect metrics."""
    env = BulletHellEnv(
        scenario_for_level(LEVEL, seed=seed, duration=DURATION),
        codec="discrete",
        terminate_on_collision=True,
    )
    field_w, field_h = env.spec.field_w, env.spec.field_h
    obs, info = env.reset(seed=seed)
    controller.reset()

    prev_action: int | None = None
    changes = 0
    survived = 0
    collision = False
    collision_step: int | None = None
    failure: str | None = None
    decision_times: list[float] = []
    ltv_counts = _new_ltv_counts()
    override_seen = False
    #: Decision steps where an LTV override *actually passed the guard*.  The raw
    #: proposal (``ltv_proposed_index``) and a guard veto are deliberately NOT
    #: recorded here -- only applied overrides count.
    override_steps: list[int] = []
    try:
        for i in range(max_steps):
            t0 = time.perf_counter()
            raw = controller.act(obs, info)
            decision_times.append(time.perf_counter() - t0)

            diag = _planner_diagnostics(controller)
            if diag is not None and diag.ltv_override:
                override_seen = True
                override_steps.append(i)
            _accumulate_ltv(ltv_counts, diag)

            action_int = coerce_action(raw).as_discrete()
            if prev_action is not None and action_int != prev_action:
                changes += 1
            prev_action = action_int

            obs, _reward, terminated, truncated, info = env.step(action_int)
            if terminated:                      # fatal collision at step i
                collision = True
                collision_step = i
                failure = classify_failure(obs, info, field_w, field_h)
                survived = i
                break
            survived = i + 1
            if truncated:                       # scenario horizon reached
                break
    finally:
        env.close()

    # ---- override timing relative to the fatal collision (diagnostics only) ---
    last_override_step = override_steps[-1] if override_steps else None
    if collision_step is not None:
        prior = [s for s in override_steps if s < collision_step]
    else:
        prior = []
    last_override_step_before_collision = prior[-1] if prior else None
    steps_since_last_override_before_collision = (
        collision_step - last_override_step_before_collision
        if (collision_step is not None and last_override_step_before_collision is not None)
        else None
    )
    seconds_since_last_override_before_collision = (
        steps_since_last_override_before_collision / CONTROL_HZ
        if steps_since_last_override_before_collision is not None
        else None
    )

    return {
        "seed": int(seed),
        "survived": survived,
        "collision": collision,
        "collision_step": collision_step,
        "failure": failure,
        "changes": changes,
        "decision_times": decision_times,
        "ltv": ltv_counts,
        "override_seen": override_seen,
        "override_steps": override_steps,
        "last_override_step": last_override_step,
        "last_override_step_before_collision": last_override_step_before_collision,
        "steps_since_last_override_before_collision": (
            steps_since_last_override_before_collision
        ),
        "seconds_since_last_override_before_collision": (
            seconds_since_last_override_before_collision
        ),
    }


#: Per-episode JSONL schema (stable order, JSON-serializable, no per-frame data).
EPISODE_JSONL_FIELDS: tuple[str, ...] = (
    "seed",
    "collision",
    "collision_step",
    "failure",
    "override_seen",
    "override_steps",
    "last_override_step",
    "last_override_step_before_collision",
    "steps_since_last_override_before_collision",
    "seconds_since_last_override_before_collision",
    "survived",
    "changes",
    "override_count",
    "decisions",
)


def episode_record(row: dict) -> dict:
    """Project a per-episode row onto the JSONL schema (pure; no mutation).

    Only plain ``int`` / ``float`` / ``bool`` / ``str`` / ``None`` / ``list[int]``
    values are emitted, so the result is always ``json.dumps``-able (no numpy
    scalars, no trajectory arrays).
    """
    collision_step = row["collision_step"]
    last_before = row["last_override_step_before_collision"]
    steps_since = row["steps_since_last_override_before_collision"]
    seconds_since = row["seconds_since_last_override_before_collision"]
    return {
        "seed": int(row["seed"]),
        "collision": bool(row["collision"]),
        "collision_step": None if collision_step is None else int(collision_step),
        "failure": None if row["failure"] is None else str(row["failure"]),
        "override_seen": bool(row["override_seen"]),
        "override_steps": [int(s) for s in row["override_steps"]],
        "last_override_step": (
            None if row["last_override_step"] is None else int(row["last_override_step"])
        ),
        "last_override_step_before_collision": (
            None if last_before is None else int(last_before)
        ),
        "steps_since_last_override_before_collision": (
            None if steps_since is None else int(steps_since)
        ),
        "seconds_since_last_override_before_collision": (
            None if seconds_since is None else float(seconds_since)
        ),
        "survived": int(row["survived"]),
        "changes": int(row["changes"]),
        "override_count": len(row["override_steps"]),
        "decisions": len(row["decision_times"]),
    }


def write_episode_jsonl(path: pathlib.Path | str, rows: list[dict]) -> pathlib.Path:
    """Write one JSON object per episode, in the given (stable) order."""
    out = pathlib.Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(episode_record(row), ensure_ascii=False))
            handle.write("\n")
    return out


def summarize(name: str, rows: list[dict]) -> dict:
    survived = np.array([r["survived"] for r in rows], dtype=np.float64)
    changes = np.array([r["changes"] for r in rows], dtype=np.float64)
    collisions = sum(1 for r in rows if r["collision"])
    times = np.array([t for r in rows for t in r["decision_times"]], dtype=np.float64)

    failures: dict[str, int] = {"front_hit": 0, "side_hit": 0, "corner_trap": 0, "other": 0}
    for r in rows:
        if r["collision"]:
            failures[r["failure"] or "other"] = failures.get(r["failure"] or "other", 0) + 1

    ltv_totals = {
        key: int(sum(r["ltv"].get(key, 0) for r in rows)) for key in _new_ltv_counts()
    }
    steps = max(ltv_totals["steps"], 1)
    override_then = {"front_hit": 0, "side_hit": 0, "corner_trap": 0, "other": 0}
    for r in rows:
        if r["collision"] and r.get("override_seen"):
            key = r["failure"] or "other"
            override_then[key] = override_then.get(key, 0) + 1
    ltv_stats = {
        **ltv_totals,
        "override_rate": ltv_totals["override"] / steps,
        "eval_rate": ltv_totals["eval"] / steps,
        "skip_rate": ltv_totals["skip"] / steps,
        "proposed_override_rate": ltv_totals["proposed_override"] / steps,
        "guard_veto_rate": ltv_totals["guard_veto"] / steps,
        # Candidate-level blocks only ("no reference" is a per-frame flag, so it
        # is reported separately and must not inflate this rate).
        "guard_block_rate": (
            ltv_totals["guard_blocked_risk_band"]
            + ltv_totals["guard_blocked_viability_margin"]
            + ltv_totals["guard_blocked_long_term_degraded"]
            + ltv_totals["guard_blocked_critical_band"]
        )
        / max(ltv_totals["guard_candidates"], 1),
        "corridor_switch_mean": float(
            np.mean([r["ltv"]["corridor_switch"] for r in rows])
        ),
        "override_episodes": sum(1 for r in rows if r.get("override_seen")),
        # Association only: episodes with >=1 override that later collided.
        "override_then_collision": sum(
            1 for r in rows if r["collision"] and r.get("override_seen")
        ),
        # Precise final outcome split of the override episodes (sums to
        # override_episodes).  Still association, never "override caused this".
        "override_episode_collided": sum(
            1 for r in rows if r["collision"] and r.get("override_seen")
        ),
        "override_episode_survived": sum(
            1 for r in rows if (not r["collision"]) and r.get("override_seen")
        ),
        "override_then": override_then,
        "selector_change_rate": ltv_totals["selector_changed"] / steps,
        "selector_applied_rate": ltv_totals["selector_applied"] / steps,
        "selector_mean_hard_survivors": (
            ltv_totals["selector_hard_survivors"] / max(ltv_totals["selector_enabled"], 1)
        ),
        "selector_mean_eligible": (
            ltv_totals["selector_eligible"] / max(ltv_totals["selector_enabled"], 1)
        ),
        "selector_mean_table_us": (
            ltv_totals["sel_table_us_total"] / max(ltv_totals["selector_enabled"], 1)
        ),
        "selector_mean_base_max_run_t3": (
            ltv_totals["sel_base_max_run_t3"] / max(ltv_totals["selector_enabled"], 1)
        ),
        "selector_mean_winner_max_run_t3": (
            ltv_totals["sel_winner_max_run_t3"] / max(ltv_totals["selector_enabled"], 1)
        ),
    }

    return {
        "name": name,
        "survival_mean": float(survived.mean()),
        "survival_std": float(survived.std(ddof=1)),
        "collision_rate": collisions / len(rows),
        "collisions": collisions,
        "episodes": len(rows),
        "decision_us_mean": float(times.mean() * 1e6),
        "decision_us_std": float(times.std(ddof=1) * 1e6),
        "decision_ms_mean": float(times.mean() * 1e3),
        "changes_mean": float(changes.mean()),
        "changes_std": float(changes.std(ddof=1)),
        "changes_total": int(changes.sum()),
        "total_decisions": int(times.size),
        "failures": failures,
        "ltv": ltv_stats,
    }


def main() -> int:
    seeds = [BASE_SEED + i for i in range(N_EPISODES)]
    factories = {
        "reactive (纯反应式 baseline)": lambda: RepulsionController(),
        "predictive (PredictiveASE 预测式)": lambda: PredictiveController(),
    }

    print(f"config: seed={BASE_SEED}, episodes={N_EPISODES}, level={LEVEL}, "
          f"max_steps={MAX_STEPS}, horizon={HORIZON}s")
    results: dict[str, dict] = {}
    for name, factory in factories.items():
        rows: list[dict] = []
        print(f"\n== {name} ==")
        print(f"[{STACK_VERSION} | build {_short_build()}]")
        print(f"[PROGRESS]    0/{N_EPISODES} (  0.0%) | starting"
              f" -- a progress notice prints every 100 episodes", flush=True)
        t_start = time.perf_counter()
        for idx, seed in enumerate(seeds):
            rows.append(run_episode(seed, factory()))
            done = idx + 1
            if done % 100 == 0 or done == N_EPISODES:
                elapsed = time.perf_counter() - t_start
                eta = (elapsed / done) * (N_EPISODES - done)
                coll = sum(1 for r in rows if r["collision"])
                print(
                    f"[PROGRESS] {done:>4}/{N_EPISODES} ({done / N_EPISODES * 100:5.1f}%)"
                    f" | elapsed {_hms(elapsed):>9} | ETA {_hms(eta):>9}"
                    f" | collisions {coll}/{done} ({coll / done * 100:5.1f}%)",
                    flush=True,
                )
        results[name] = summarize(name, rows)
        # Optional per-episode JSONL (diagnostics infrastructure only; the
        # default benchmark behaviour is unchanged).  Set ``DF_EPISODES_JSONL``
        # to a path prefix, e.g. ``DF_EPISODES_JSONL=/tmp/run1``.
        prefix = os.environ.get("DF_EPISODES_JSONL")
        if prefix:
            slug = "reactive" if "reactive" in name else "predictive"
            path = write_episode_jsonl(f"{prefix}.{slug}.episodes.jsonl", rows)
            print(f"[episodes] wrote {path}", flush=True)

    reactive = results["reactive (纯反应式 baseline)"]
    predictive = results["predictive (PredictiveASE 预测式)"]

    markdown = _render_markdown(reactive, predictive)
    out = ROOT / "docs" / "benchmark_results.md"
    out.write_text(markdown, encoding="utf-8")

    print("\n" + "=" * 72)
    print("results (also written to docs/benchmark_results.md)")
    print("=" * 72)
    print(markdown)
    return 0


def _failure_row(name: str, reactive: dict, predictive: dict) -> str:
    rf = reactive["failures"]
    pf = predictive["failures"]
    return (
        f"| {name} | {rf.get(name, 0)} | {pf.get(name, 0)} |"
    )


def _render_markdown(reactive: dict, predictive: dict) -> str:
    rf, pf = reactive["failures"], predictive["failures"]
    ltv = predictive["ltv"]
    ot = ltv["override_then"]
    ltv_flag = "enabled" if ENABLE_LTV else "disabled (strict v0.4.1 reference)"
    return f"""# 决策基准测试结果：RepulsionController vs PredictiveASE ({PREDICTIVE_VERSION})

> Stack Version:      `{STACK_VERSION}`
> Reactive baseline:  `RepulsionController`（参考基线，保持不变）
> Predictive Version: `{PREDICTIVE_VERSION}`
> Build ID:           `{BUILD_ID}`
> Random Seed:        `{BASE_SEED}`
> Episode Count:      `{N_EPISODES}`

> 注：`v0.5.0-ReactiveGap`（Follow-the-Gap 实验）已归档保留，但**不参与本基准**
> （实测 collision 94.5%）。本表始终对照原始 `RepulsionController`。

> 本文件由 `benchmark_decision.py` 自动生成。复现命令：
> `.venv/bin/python benchmark_decision.py`

## 1. 实验配置

| 项 | 值 |
|---|---|
| 随机种子 | `{BASE_SEED}`（1000 局场景由 `{BASE_SEED} + i` 确定性派生，两控制器共用同一批场景） |
| 场景 | `{LEVEL}`（`scenario_for_level`），时长 {DURATION}s，`terminate_on_collision=True` |
| 生存上限 | {MAX_STEPS} 步（10 s @ 120 Hz） |
| 预测 horizon | {HORIZON}s（仅预测式决策使用；线性外推 lookahead） |
| 局数 | {N_EPISODES} |

## 2. 对比对象

| 策略 | 实现 | 说明 |
|---|---|---|
| **纯反应式 baseline** | `bullet_sim.ai.baseline.RepulsionController` | 只看**当前帧**附近弹幕密度，沿反方向排斥 + 向目标吸引；O(N)，observation-only。`v0.5.0-ReactiveGap` 实验已归档，不参与本基准 |
| **PredictiveASE ({PREDICTIVE_VERSION})** | `cpu.decision.CpuDecisionLayer` | **v0.4.1 FAR + ASE 全部保留**（候选条件化 `v_rel`、`t_ca`/`d_ca`、`closing_rate`、`min_margin` LEVEL 0/1/2、`lateral_critical_risk`、≤8 局部细化、SafetyGate、FES、dwell、自适应 horizon/smooth、EmergencyFallback），并新增 **v0.5.1 Long-Term Viability**：Top-K=4 短名单（3 短期 leader + 1 个不偷看 viability 的 diversity）→ 3 个 anchor（scene scale 标定，≤0.8s）上用 **future agent × future obstacle** 计算 8 扇区 mobility（`safe_sector_count / max_continuous_safe_sector / corridor_min_clearance`，`mobility∈[0,1]`）→ `viability=min(t1,t2,t3)` → 分档仲裁（critical band > risk band > viability > FES > behaviour > task，`risk_band_width=0.05`）→ corridor hysteresis + action hysteresis。低威胁 5 条件全满足时跳过 LTV（复用 v0.4.1 排序）。**v0.5.2-LTVGuarded** 保留以上全部机制，但把 LTV 降级为**顾问式**：LTV 的提案只有在 *critical band 不劣、risk band 完全相同、viability 增益 ≥ 0.10、且未 long-term degraded* 时才被采纳，否则回退 v0.4.1 short-term best（`short-term safety first, LTV advisory second`）。|

## 3. 结果

| 指标 | 纯反应式 baseline | PredictiveASE |
|---|---|---|
| 生存时间 (步, 均值±标准差) | {reactive['survival_mean']:.1f} ± {reactive['survival_std']:.1f} | {predictive['survival_mean']:.1f} ± {predictive['survival_std']:.1f} |
| 碰撞率 ({N_EPISODES} 局) | {reactive['collision_rate'] * 100:.1f}% ({reactive['collisions']}/{reactive['episodes']}) | {predictive['collision_rate'] * 100:.1f}% ({predictive['collisions']}/{predictive['episodes']}) |
| 决策耗时 (均值±标准差) | {reactive['decision_us_mean']:.1f} ± {reactive['decision_us_std']:.1f} µs | {predictive['decision_us_mean']:.1f} ± {predictive['decision_us_std']:.1f} µs |
| 动作平滑度 (动作变化次数, 均值±标准差) | {reactive['changes_mean']:.1f} ± {reactive['changes_std']:.1f} (总 {reactive['changes_total']} 次 / {reactive['total_decisions']} 步) | {predictive['changes_mean']:.1f} ± {predictive['changes_std']:.1f} (总 {predictive['changes_total']} 次 / {predictive['total_decisions']} 步) |

### 失败类型统计

| 失败原因 | 纯反应式 baseline | PredictiveASE |
|---|---|---|
{_failure_row('front_hit', reactive, predictive)}
{_failure_row('side_hit', reactive, predictive)}
{_failure_row('corner_trap', reactive, predictive)}
{_failure_row('other', reactive, predictive)}

### LTV 可观测性（本轮只做观测；override↔碰撞 仅为统计关联，不表示因果）

| 指标 | 值 |
|---|---|
| ENABLE_LTV | `{ltv_flag}` |
| LTV 决策步数 | {ltv['steps']} |
| LTV 实际评估 (eval) | {ltv['eval']} ({ltv['eval_rate'] * 100:.1f}%) |
| LTV 跳过 (skip) | {ltv['skip']} ({ltv['skip_rate'] * 100:.1f}%) |
| **ltv_override 步数** | **{ltv['override']} ({ltv['override_rate'] * 100:.1f}%)** |
| override 发生的 episode 数 | {ltv['override_episodes']} |
| corridor switch 总数 / 每局均值 | {ltv['corridor_switch']} / {ltv['corridor_switch_mean']:.2f} |
| corridor hysteresis hold | {ltv['corridor_hold']} |
| corridor hard-unsafe 立即切换 | {ltv['corridor_unsafe_switch']} |
| emergency 步数 | {ltv['emergency']} |
| override 后发生碰撞的 episode 数 | {ltv['override_then_collision']} |
| ↳ front_hit / side_hit / corner_trap / other | {ot['front_hit']} / {ot['side_hit']} / {ot['corner_trap']} / {ot['other']} |

### v0.5.2 LTV override guard（short-term safety first；override↔碰撞 仅为统计关联）

| 指标 | 值 |
|---|---|
| LTV shadow 模式 | `{'ON' if LTV_SHADOW_ONLY else 'off'}` |
| override guard | `{'ON' if LTV_OVERRIDE_GUARD else 'OFF'}`（阈值 viability gain ≥ {ViabilityConfig().viability_improvement_threshold:.2f}） |
| LTV 想改而未被 guard 放行的步数 (proposed_override) | {ltv['proposed_override']} ({ltv['proposed_override_rate'] * 100:.1f}%) |
| **实际放行的 ltv_override 步数** | **{ltv['override']} ({ltv['override_rate'] * 100:.1f}%)** |
| guard 否决步数 (veto) | {ltv['guard_veto']} ({ltv['guard_veto_rate'] * 100:.1f}%) |
| guard 考察的候选总数 | {ltv['guard_candidates']}（候选级拦截率 {ltv['guard_block_rate'] * 100:.1f}%） |
| ↳ blocked by risk band（跨风险档） | {ltv['guard_blocked_risk_band']} |
| ↳ blocked by viability margin（优势不足） | {ltv['guard_blocked_viability_margin']} |
| ↳ blocked by long-term degraded | {ltv['guard_blocked_long_term_degraded']} |
| ↳ blocked by critical band | {ltv['guard_blocked_critical_band']} |
| ↳ blocked by no LTV reference | {ltv['guard_blocked_no_reference']} |

### v0.6.0 Predictive Corridor Selector（在已有 hard safety 之上做分层选择；开/关由 `DF_ENABLE_FSC_SELECTOR` 控制）

| 指标 | 值 |
|---|---|
| FSC selector | `{'ON' if FSC_SELECTOR_ENABLED else 'off (v0.5.2-LTVGuarded action path)'}` |
| 启用帧数 | {ltv['selector_enabled']} / {ltv['steps']} |
| hard safety survivors（进入 selector 的候选数，均值/帧） | {ltv['selector_mean_hard_survivors']:.2f} |
| eligible（通过 critical floor 的候选数，均值/帧） | {ltv['selector_mean_eligible']:.2f} |
| UNKNOWN-at-t3 候选（仍参与竞争，均值/帧） | {ltv['selector_unknown_candidates'] / max(ltv['selector_enabled'], 1):.2f} |
| selector applied（改变了 v0.5.2 的选择） | {ltv['selector_applied']} ({ltv['selector_applied_rate'] * 100:.2f}%) |
| **final selector changes / action changes** | **{ltv['selector_changed']} ({ltv['selector_change_rate'] * 100:.2f}%)** |
| ↳ Level 2 reachability preference | {ltv['sel_level_reachability']} |
| ↳ Level 3 persistent corridor preference | {ltv['sel_level_persistent']} |
| ↳ Level 4 max_run preference | {ltv['sel_level_max_run']} |
| ↳ Level 5 clearance preference | {ltv['sel_level_clearance']} |
| ↳ Level 6 existing tiers preference | {ltv['sel_level_existing_tiers']} |
| v0.5.2 base reachable (frames) | {ltv['sel_base_reachable']} |
| selector winner reachable (frames) | {ltv['sel_winner_reachable']} |
| mean base persistent sectors | {ltv['sel_base_persistent'] / max(ltv['selector_enabled'], 1):.2f} |
| mean winner persistent sectors | {ltv['sel_winner_persistent'] / max(ltv['selector_enabled'], 1):.2f} |
| mean base max_run_t3 | {ltv['selector_mean_base_max_run_t3']:.2f} |
| mean winner max_run_t3 | {ltv['selector_mean_winner_max_run_t3']:.2f} |
| corridor table cost (mean µs/frame) | {ltv['selector_mean_table_us']:.1f} |

## 4. 结论

- （结果由脚本实测填写，见上表。）
- 纯反应式 baseline 决策耗时只含「当前帧 O(N) 排斥 + 目标吸引」；PredictiveASE 见下。
- PredictiveASE 决策耗时包含「观测 → 抽象场景 → 17（危险时 ≤25）条候选 × horizon 步线性外推 → FAR 安全/风险 + 自适应安全包络 → v0.5.2 guarded LTV（仅 K≤4 短名单 × 3 anchor）→ 分档 argmin → override guard → corridor/action hysteresis」的完整链路；内部另记 `short_term_us / viability_us / total_us`。
- 失败类型为启发式分类：`corner_trap` = 碰撞时处于场地角落区（{CORNER_MARGIN:.0%}×短边）；`front_hit` = 撞击方向与航向夹角 ≤ {FRONT_ANGLE_DEG:.0f}°；`side_hit` = 夹角更大；`other` = 静止或无有效撞击方向。

_生成时间：由脚本运行生成；配置见 `benchmark_decision.py`。_
"""


if __name__ == "__main__":
    raise SystemExit(main())

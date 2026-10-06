#!/usr/bin/env python3
"""FSC shadow probe - v0.6.0-SafeCorridor-Shadow (diagnostics only).

Runs the real v0.5.2-LTVGuarded controller and, *alongside* each decision,
computes Future Safe Corridor diagnostics.  The controller's action is produced
first and is never modified; this tool additionally re-runs every episode without
the observer and asserts the two runs are byte-identical (same survival, same
collision, same action changes), which is the neutrality proof for this phase.

Usage::

    .venv/bin/python tools/fsc_shadow_probe.py --episodes 3
    .venv/bin/python tools/fsc_shadow_probe.py --episodes 5 --jsonl /tmp/fsc_frames.jsonl

This is a SHADOW prototype: it is not wired into the planner, it does not change
any v0.5.2 parameter, and nothing here can emit an action.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import sys
import time

import numpy as np

# core/cpu/tools/<script>.py -> parents[2] = core/（cpu / bullet_sim 都在这下面）
_HERE = pathlib.Path(__file__).resolve().parent
ROOT = pathlib.Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
if str(_HERE.parent) not in sys.path:
    sys.path.insert(0, str(_HERE.parent))

from cpu.adapters import observation_to_scene  # noqa: E402
from cpu.decision.future_corridor import (  # noqa: E402
    CORRIDOR_DEAD_END,
    CORRIDOR_NONE,
    CORRIDOR_STABLE,
    CORRIDOR_UNSTABLE,
    CORRIDOR_TYPES,
    SafeCorridorShadow,
)
from cpu.decision.viability import ViabilityContext  # noqa: E402
from bullet_sim.simulator.env import BulletHellEnv  # noqa: E402


def _scene_scale(observation) -> float | None:
    """``T_scene = min(field_w, field_h) / max_speed`` (mirrors the controller)."""
    env = observation.get("env")
    if env is None or len(env) < 2:
        return None
    field_w, field_h = float(env[0]), float(env[1])
    speed = float(observation["player"][5])
    if field_w <= 0.0 or field_h <= 0.0 or speed <= 0.0:
        return None
    return min(field_w, field_h) / speed


def _resolve_chosen(actions, diag):
    """Index of the emitted action inside the base candidate list.

    The planner may have used a temporary refinement action; if so it is appended
    to the list used for FSC so the chosen candidate is always representable.
    """
    chosen_action = getattr(diag, "chosen_action", None)
    if chosen_action is not None:
        for i, candidate in enumerate(actions):
            if candidate == chosen_action:
                return actions, i
        return actions + (chosen_action,), len(actions)
    idx = int(getattr(diag, "chosen_index", 0) or 0)
    return actions, max(0, min(idx, len(actions) - 1))


def _resolve_shortlist(actions, diag):
    """Positions, inside ``actions``, of the planner's REAL Top-K shortlist.

    Uses ``PlanDiagnostics.fsc_candidate_indices`` / ``fsc_candidate_actions``
    (3 leaders + 1 diversity, planner order) instead of guessing a shortlist.
    Returns ``(actions, ())`` when the planner produced none.
    """
    shortlist = tuple(getattr(diag, "fsc_candidate_actions", ()) or ())
    if not shortlist:
        return actions, ()
    out: list[int] = []
    for action in shortlist:
        pos = None
        for i, candidate in enumerate(actions):
            if candidate == action:
                pos = i
                break
        if pos is None:                      # e.g. a refinement action
            actions = actions + (action,)
            pos = len(actions) - 1
        out.append(pos)
    return actions, tuple(dict.fromkeys(out))


def run_episode_shadow(
    bench, seed: int, *, k: int, max_steps: int, frames: list, enabled: bool = True
):
    """One episode with FSC recorded per frame.  Returns the episode record.

    The v0.5.2 action is produced first and is identical whether ``enabled`` is
    True or False -- the observer only reads.
    """
    from bullet_sim.scenarios.presets import scenario_for_level

    env = BulletHellEnv(
        scenario_for_level(bench.LEVEL, seed=seed, duration=bench.DURATION),
        codec="discrete",
        terminate_on_collision=True,
    )
    field_w, field_h = env.spec.field_w, env.spec.field_h
    obs, info = env.reset(seed=seed)
    controller = bench.PredictiveController()
    controller.reset()
    shadow = SafeCorridorShadow(enabled=enabled)

    prev_action = None
    changes = 0
    collision = False
    collision_step = None
    failure = None
    survive = 0
    rect_us: list[float] = []
    reach_us: list[float] = []
    corr_us: list[float] = []
    total_us: list[float] = []
    try:
        for i in range(max_steps):
            action = controller.act(obs, info)              # the v0.5.2 action
            diag = controller.layer.planner.last_diagnostics
            chosen = int(getattr(diag, "chosen_index", 0) or 0)
            if enabled:
                scene = observation_to_scene(
                    obs, info, horizon=controller.horizon, dt=controller.dt, prune=True
                )
                context = ViabilityContext(
                    short_horizon=controller.horizon, scene_scale=_scene_scale(obs)
                )
                actions, chosen = _resolve_chosen(
                    controller.layer.planner.candidate_actions(scene), diag
                )
                actions, topk = _resolve_shortlist(actions, diag)
                anchors = getattr(diag, "long_anchors", None)
                frame = shadow.observe(
                    scene,
                    actions,
                    context,
                    chosen_index=chosen,
                    k=k,
                    anchors=anchors,
                    candidate_indices=topk,
                    v052_diagnostics=diag,
                    build_id=bench.BUILD_ID,
                )
                rect_us.append(frame.safe_rect_us)
                reach_us.append(frame.reachability_us)
                corr_us.append(frame.corridor_us)
                total_us.append(frame.total_shadow_us)
                record_frame(frame, frames, seed=seed, step=i, dt=controller.dt, chosen=chosen, diag=diag)

            action_int = int(action.as_discrete())
            if prev_action is not None and action_int != prev_action:
                changes += 1
            prev_action = action_int

            obs, _reward, terminated, truncated, info = env.step(action_int)
            if terminated:
                collision = True
                collision_step = i
                failure = bench.classify_failure(obs, info, field_w, field_h)
                survive = i
                break
            survive = i + 1
            if truncated:
                break
    finally:
        env.close()

    return {
        "seed": int(seed),
        "collision": bool(collision),
        "collision_step": collision_step,
        "failure": failure,
        "survived": int(survive),
        "changes": int(changes),
        "decisions": len(rect_us),
        "safe_rect_us_mean": float(np.mean(rect_us)) if rect_us else 0.0,
        "reachability_us_mean": float(np.mean(reach_us)) if reach_us else 0.0,
        "corridor_us_mean": float(np.mean(corr_us)) if corr_us else 0.0,
        "total_shadow_us_mean": float(np.mean(total_us)) if total_us else 0.0,
        "records": len(rect_us),
    }


def record_frame(frame, frames, *, seed, step, dt, chosen, diag) -> None:
    """Append one per-frame FSC record (JSON-serializable, no per-frame arrays)."""
    frames.append(
        {
            "seed": int(seed),
            "step": int(step),
            "t": float(step) * float(dt),
            "chosen_index": int(chosen),
            "corridor_type": frame.chosen.corridor_type,
            "reach_12": bool(frame.chosen.reach_12),
            "reach_23": bool(frame.chosen.reach_23),
            "persistent_mask": int(frame.chosen.persistent_mask),
            "overlap_12": int(frame.chosen.overlap_12),
            "overlap_23": int(frame.chosen.overlap_23),
            "widths": [int(w) for w in frame.chosen.widths],
            "masks": [int(m) for m in frame.chosen.safe_masks],
            "rect_valid": [bool(v) for v in frame.chosen.safe_rect_valids],
            "rect_reason": [s.safe_rect_invalid_reason for s in frame.chosen.per_anchor],
            "target_valid": bool(frame.chosen.future_target_valid),
            "target_x": float(frame.chosen.future_target_x),
            "target_y": float(frame.chosen.future_target_y),
            "match_count": int(frame.chosen.corridor_match_count),
            "ltv_viability": float(getattr(diag, "chosen_viability", 0.0) or 0.0),
            "ltv_override": bool(getattr(diag, "ltv_override", False)),
            "emergency": bool(getattr(diag, "emergency", False)),
            "stable": int(frame.stable_funnel_count),
            "dead_end": int(frame.dead_end_funnel_count),
            "unstable": int(frame.unstable_funnel_count),
            "none": int(frame.none_funnel_count),
            "unknown": int(frame.unknown_funnel_count),
            "planner_topk": [
                int(i) for i in (getattr(diag, "fsc_candidate_indices", ()) or ())
            ],
            "candidate_source": frame.candidate_source,
            "safe_rect_us": float(frame.safe_rect_us),
            "reachability_us": float(frame.reachability_us),
            "corridor_us": float(frame.corridor_us),
            "total_shadow_us": float(frame.total_shadow_us),
        }
    )


def _lead_windows(frames: list[dict], collision_step: int | None) -> dict:
    """Was a DEAD_END observed in the last 0.30/0.50/0.80 s before the collision?"""
    out = {"dead_end_seen_before_collision": False}
    for window in (0.30, 0.50, 0.80):
        out[f"dead_end_in_last_{window:.2f}s"] = False
    out["first_dead_end_lead_s"] = None
    if collision_step is None:
        return out
    lead = None
    for rec in frames:
        if rec["step"] >= collision_step:
            break
        if rec["corridor_type"] == CORRIDOR_DEAD_END:
            out["dead_end_seen_before_collision"] = True
            step_lead = (collision_step - rec["step"]) / 120.0
            lead = step_lead if lead is None else max(lead, step_lead)
    out["first_dead_end_lead_s"] = lead
    for window in (0.30, 0.50, 0.80):
        out[f"dead_end_in_last_{window:.2f}s"] = bool(
            lead is not None and lead >= window
        )
    return out


def _summarize(records: list[dict], frames: list[dict]) -> dict:
    by_seed: dict[int, list[dict]] = {}
    for rec in frames:
        by_seed.setdefault(rec["seed"], []).append(rec)

    per_episode = []
    for rec in records:
        episode_frames = by_seed.get(rec["seed"], [])
        entry = dict(rec)
        entry.update(_lead_windows(episode_frames, rec["collision_step"]))
        per_episode.append(entry)

    def _rate(key: str, subset=None) -> float:
        rows = [e for e in per_episode if subset is None or subset(e)]
        if not rows:
            return 0.0
        return sum(1 for e in rows if e[key]) / len(rows)

    collapsed = [e for e in per_episode if e["collision"]]
    survived = [e for e in per_episode if not e["collision"]]

    # corridor type of the chosen candidate, per frame, split by episode outcome
    def _type_counts(rows):
        counts = {name: 0 for name in CORRIDOR_TYPES}
        seeds = {e["seed"] for e in rows}
        for rec in frames:
            if rec["seed"] in seeds:
                counts[rec["corridor_type"]] = counts.get(rec["corridor_type"], 0) + 1
        return counts

    target_before = 0
    target_eps = 0
    for rec in records:
        if rec["collision_step"] is None:
            continue
        target_eps += 1
        if any(
            f["seed"] == rec["seed"]
            and f["step"] < rec["collision_step"]
            and f["target_valid"]
            for f in frames
        ):
            target_before += 1

    return {
        "episodes": len(per_episode),
        "collisions": len(collapsed),
        "survived": len(survived),
        "dead_end_rate_collision_episodes": _rate(
            "dead_end_seen_before_collision", lambda e: e["collision"]
        ),
        "dead_end_in_last_0.30s": _rate(
            "dead_end_in_last_0.30s", lambda e: e["collision"]
        ),
        "dead_end_in_last_0.50s": _rate(
            "dead_end_in_last_0.50s", lambda e: e["collision"]
        ),
        "dead_end_in_last_0.80s": _rate(
            "dead_end_in_last_0.80s", lambda e: e["collision"]
        ),
        "collapse_types": _type_counts(collapsed),
        "survive_types": _type_counts(survived),
        "future_target_before_collision": target_before,
        "future_target_collision_episodes": target_eps,
        "per_episode": per_episode,
        "safe_rect_us_mean": float(
            np.mean([r["safe_rect_us"] for r in frames])
        )
        if frames
        else 0.0,
        "reachability_us_mean": float(
            np.mean([r["reachability_us"] for r in frames])
        )
        if frames
        else 0.0,
        "corridor_us_mean": float(
            np.mean([r["corridor_us"] for r in frames])
        )
        if frames
        else 0.0,
        "total_shadow_us_mean": float(
            np.mean([r["total_shadow_us"] for r in frames])
        )
        if frames
        else 0.0,
        "frames": len(frames),
        "topk_frames": sum(
            1 for r in frames if r.get("candidate_source") == "planner_topk"
        ),
        "proxy_frames": sum(
            1 for r in frames if r.get("candidate_source") == "shadow_proxy"
        ),
        "topk_size_mean": float(
            np.mean([len(r.get("planner_topk", ())) for r in frames])
        )
        if frames
        else 0.0,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--k", type=int, default=4, help="FSC candidates per frame (<=4)")
    parser.add_argument("--max-steps", type=int, default=None)
    parser.add_argument(
        "--mode",
        choices=("shadow", "off"),
        default="shadow",
        help="shadow = compute FSC diagnostics; off = v0.5.2 action only (default: shadow)",
    )
    parser.add_argument(
        "--jsonl",
        "--output",
        dest="jsonl_path",
        default=None,
        help="per-frame FSC JSONL output path",
    )
    args = parser.parse_args()

    if args.episodes < 1:
        print("--episodes must be >= 1", file=sys.stderr)
        return 2
    if args.episodes > 5:
        print(
            "[warn] this is a shadow prototype phase; capping episodes at 5.",
            file=sys.stderr,
        )
        args.episodes = 5
    if args.k > 4:
        print("[warn] FSC first version targets K <= 4; capping.", file=sys.stderr)
        args.k = 4

    import benchmark_decision as bench  # prints the runtime banner

    shadow_enabled = args.mode == "shadow"
    max_steps = args.max_steps if args.max_steps is not None else bench.MAX_STEPS
    seeds = [args.seed + i for i in range(args.episodes)]

    print(f"\n== FSC shadow probe | mode={args.mode} episodes={args.episodes} "
          f"seed={args.seed} k={args.k} max_steps={max_steps} ==", flush=True)
    if shadow_enabled:
        print("shadow only: the v0.5.2 action is produced first and never modified",
              flush=True)
    else:
        print("mode=off: no FSC is computed at all (pure v0.5.2 run)",
              flush=True)

    frames: list[dict] = []
    records: list[dict] = []
    plain: list[dict] = []
    for idx, seed in enumerate(seeds):
        t0 = time.monotonic()
        records.append(
            run_episode_shadow(
                bench,
                seed,
                k=args.k,
                max_steps=max_steps,
                frames=frames,
                enabled=shadow_enabled,
            )
        )
        elapsed = time.monotonic() - t0
        rec = records[-1]
        print(
            f"[{idx + 1}/{args.episodes}] seed={seed} survived={rec['survived']} "
            f"collision={rec['collision']} failure={rec['failure']} "
            f"frames={rec['records']} ({elapsed:.1f}s)",
            flush=True,
        )

    # neutrality proof: the identical episode without the observer
    print("\n[NEUTRALITY] re-running every episode without the observer ...", flush=True)
    for seed in seeds:
        plain.append(bench.run_episode(seed, bench.PredictiveController()))
    for rec, base in zip(records, plain):
        same = (
            rec["survived"] == base["survived"]
            and rec["collision"] == base["collision"]
            and rec["changes"] == base["changes"]
        )
        print(
            f"  seed={rec['seed']}: shadow(survived={rec['survived']}, "
            f"collision={rec['collision']}, changes={rec['changes']}) vs "
            f"plain(survived={base['survived']}, collision={base['collision']}, "
            f"changes={base['changes']}) -> {'IDENTICAL' if same else 'DIFFERENT'}",
            flush=True,
        )
        if not same:
            print("[FATAL] the shadow observer changed the action.", file=sys.stderr)
            return 3

    summary = _summarize(records, frames)

    if args.jsonl_path:
        out = pathlib.Path(args.jsonl_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        with out.open("w", encoding="utf-8") as handle:
            for rec in frames:
                handle.write(json.dumps(rec))
                handle.write("\n")
        print(f"\nwrote {out} ({len(frames)} frames)", flush=True)

    print("\n" + "=" * 72)
    print("FSC SHADOW SUMMARY (association only -- no causal claim)")
    print("=" * 72)
    print(f"episodes={summary['episodes']} collisions={summary['collisions']} "
          f"survived={summary['survived']} frames={summary['frames']}")
    print("\ncorridor type of the chosen candidate, per frame:")
    print(f"  {'type':<22}{'collision eps':>15}{'survive eps':>15}")
    for name in CORRIDOR_TYPES:
        print(f"  {name:<22}{summary['collapse_types'][name]:>15}"
              f"{summary['survive_types'][name]:>15}")
    print("\nFSC candidate list origin:")
    print(f"  planner real Top-K frames   = {summary['topk_frames']}")
    print(f"  shadow proxy fallback frames= {summary['proxy_frames']}")
    print(f"  mean planner|Top-K|         = {summary['topk_size_mean']:.2f}")
    print("\nDEAD_END before a collision:")
    print(f"  any DEAD_END before collision      = "
          f"{summary['dead_end_rate_collision_episodes'] * 100:.0f}%")
    print(f"  DEAD_END within last 0.30 s        = "
          f"{summary['dead_end_in_last_0.30s'] * 100:.0f}%")
    print(f"  DEAD_END within last 0.50 s        = "
          f"{summary['dead_end_in_last_0.50s'] * 100:.0f}%")
    print(f"  DEAD_END within last 0.80 s        = "
          f"{summary['dead_end_in_last_0.80s'] * 100:.0f}%")
    print(f"  future_target before collision     = "
          f"{summary['future_target_before_collision']}/"
          f"{summary['future_target_collision_episodes']} episodes")
    print("\nprototype cost (mean over frames):")
    print(f"  safe_rect_us     = {summary['safe_rect_us_mean']:.1f}")
    print(f"  reachability_us  = {summary['reachability_us_mean']:.1f}")
    print(f"  corridor_us      = {summary['corridor_us_mean']:.1f}")
    print(f"  total_shadow_us  = {summary['total_shadow_us_mean']:.1f}")
    print("\nno 1000-episode run; no planner change; no action change.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

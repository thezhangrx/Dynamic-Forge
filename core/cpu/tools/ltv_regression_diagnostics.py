#!/usr/bin/env python3
"""Long-Term Viability regression diagnostics (observability only).

Runs the *same* predictive controller over an *identical* set of episodes twice --
once with LTV enabled (guarded) and once with LTV disabled (the strict v0.4.1
reference path) -- prints the LTV observability counters, and (in ``both`` mode)
joins the two per-episode logs by ``seed`` for a real paired analysis.

It only *reads* decision state. It never changes an algorithm parameter, it never
changes the emitted action, and it never writes ``docs/benchmark_results.md``
(unlike ``benchmark_decision.py``).

Usage (always with the project virtualenv)::

    .venv/bin/python tools/ltv_regression_diagnostics.py --episodes 80 --mode both
    .venv/bin/python tools/ltv_regression_diagnostics.py --episodes 5 --mode on --debug
    .venv/bin/python tools/ltv_regression_diagnostics.py --episodes 1000 --mode both \
        --episodes-jsonl /tmp/ltv1000 --json /tmp/ltv1000.json

Per-episode JSONL (one JSON object per line, stable seed order)::

    <prefix>.off.episodes.jsonl       LTV OFF / strict v0.4.1 reference
    <prefix>.guarded.episodes.jsonl   LTV guarded (current build)

Reading the output:

    override_rate   fraction of decision steps where an applied LTV override moved
                    the choice away from v0.4.1  <-- the regression answer
    paired analysis off x guarded joined by seed; 4-cell table, per-failure
                    transitions, and override-to-collision timing
    override_*      association statistics only, NOT causation
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import subprocess
import sys
import tempfile
import time

# core/cpu/tools/<script>.py -> parents[2] = core/（cpu / bullet_sim 都在这下面）
_HERE = pathlib.Path(__file__).resolve().parent
ROOT = pathlib.Path(__file__).resolve().parents[2]

#: Counters printed for each mode, in order.
_COUNTERS = (
    "steps",
    "eval",
    "eval_rate",
    "skip",
    "skip_rate",
    "proposed_override",
    "proposed_override_rate",
    "override",
    "override_rate",
    "guard_veto",
    "guard_veto_rate",
    "guard_candidates",
    "guard_block_rate",
    "guard_blocked_risk_band",
    "guard_blocked_viability_margin",
    "guard_blocked_long_term_degraded",
    "guard_blocked_critical_band",
    "guard_blocked_no_reference",
    "override_episodes",
    "override_episode_survived",
    "override_episode_collided",
    "corridor_switch",
    "corridor_switch_mean",
    "corridor_hold",
    "corridor_unsafe_switch",
    "emergency",
    "override_then_collision",
)
_SUMMARY = ("survival_mean", "survival_std", "collision_rate", "decision_us_mean", "changes_mean")

#: Failure categories produced by ``benchmark_decision.classify_failure``.
FAILURE_TYPES: tuple[str, ...] = ("front_hit", "side_hit", "corner_trap", "other")

#: Override-to-collision time buckets, in seconds.  Boundaries are half-open:
#: ``[lo, hi)`` except the last finite bucket which includes 2.00 exactly.
TEMPORAL_BINS: tuple[str, ...] = (
    "<0.10",
    "0.10-0.25",
    "0.25-0.50",
    "0.50-1.00",
    "1.00-2.00",
    ">2.00",
    "none",
)


def _parse() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--episodes", type=int, default=80, help="number of episodes (default: 80)"
    )
    parser.add_argument(
        "--seed", type=int, default=42, help="first episode seed (default: 42)"
    )
    parser.add_argument(
        "--mode",
        choices=("on", "off", "both"),
        default="both",
        help="LTV guarded / disabled / side-by-side (default: both)",
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="enable per-frame [LTV] trace (DF_LTV_DEBUG=1); use with few episodes",
    )
    parser.add_argument(
        "--max-steps",
        type=int,
        default=None,
        help="survival cap override (default: benchmark default, 1200)",
    )
    parser.add_argument(
        "--json", dest="json_path", default=None, help="also write the results as JSON"
    )
    parser.add_argument(
        "--episodes-jsonl",
        dest="episodes_jsonl",
        default=None,
        help="path prefix for per-episode JSONL logs "
        "(<prefix>.off.episodes.jsonl / <prefix>.guarded.episodes.jsonl)",
    )
    return parser.parse_args()


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


def _episode_paths(prefix: str) -> tuple[pathlib.Path, pathlib.Path]:
    """``(off_path, guarded_path)`` for a JSONL prefix."""
    return (
        pathlib.Path(f"{prefix}.off.episodes.jsonl"),
        pathlib.Path(f"{prefix}.guarded.episodes.jsonl"),
    )


def _mode_label(stack_version: str, *, ltv_on: bool) -> str:
    """Run label derived from the *runtime* build, never a hard-coded version."""
    if ltv_on:
        return f"LTV ON  ({stack_version})"
    return f"LTV OFF (strict v0.4.1 reference path; binary {stack_version})"


# ==========================================================================
# per-episode run (single mode)
# ==========================================================================
def _run_single(args: argparse.Namespace, *, ltv_on: bool) -> dict:
    """Run one mode in this process and return its summary dict."""
    # The planner reads these at import time, so set them *before* importing.
    os.environ["DF_ENABLE_LTV"] = "1" if ltv_on else "0"
    if args.debug:
        os.environ["DF_LTV_DEBUG"] = "1"

    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    if str(_HERE.parent) not in sys.path:
        sys.path.insert(0, str(_HERE.parent))

    import benchmark_decision as bench  # noqa: E402  (import after env setup)

    max_steps = b_max = args.max_steps if args.max_steps is not None else bench.MAX_STEPS
    seeds = [args.seed + i for i in range(args.episodes)]
    # Labels come from the runtime build, never from a hard-coded version string.
    label = _mode_label(bench.STACK_VERSION, ltv_on=ltv_on)

    print(f"\n== {label} | episodes={args.episodes} seed={args.seed} "
          f"max_steps={b_max} ==", flush=True)
    print(f"[PROGRESS]    0/{args.episodes} (  0.0%) | starting"
          f" -- a progress notice prints every 100 episodes", flush=True)
    rows: list[dict] = []
    t_start = time.monotonic()
    done = 0
    for idx, seed in enumerate(seeds):
        rows.append(bench.run_episode(seed, bench.PredictiveController(), max_steps=max_steps))
        done = idx + 1
        if done % 100 == 0 or done == args.episodes:
            elapsed = time.monotonic() - t_start
            eta = (elapsed / done) * (args.episodes - done)
            coll = sum(1 for r in rows if r["collision"])
            print(
                f"[PROGRESS] {done:>4}/{args.episodes} ({done / args.episodes * 100:5.1f}%)"
                f" | elapsed {_hms(elapsed):>9} | ETA {_hms(eta):>9}"
                f" | collisions {coll}/{done} ({coll / done * 100:5.1f}%)",
                flush=True,
            )

    summary = bench.summarize(label, rows)
    summary["mode"] = "on" if ltv_on else "off"
    summary["build_id"] = bench.BUILD_ID
    summary["version"] = bench.STACK_VERSION
    if args.episodes_jsonl:
        off_path, guarded_path = _episode_paths(args.episodes_jsonl)
        target = guarded_path if ltv_on else off_path
        bench.write_episode_jsonl(target, rows)
        summary["episode_file"] = str(target)
        if not os.environ.get("LTVDIAG_CHILD"):
            print(f"\nwrote {target}", flush=True)
    return summary


# ==========================================================================
# paired episode analysis
# ==========================================================================
def load_episodes(path: pathlib.Path | str) -> list[dict]:
    """Read a per-episode JSONL file (one JSON object per line)."""
    out: list[dict] = []
    with pathlib.Path(path).open("r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError as exc:  # pragma: no cover - defensive
                raise ValueError(f"{path}:{lineno}: invalid JSON line: {exc}") from exc
    return out


def _index_by_seed(episodes: list[dict], path: pathlib.Path | str) -> dict[int, dict]:
    """Index episodes by seed, rejecting duplicates (never silently overwrite)."""
    index: dict[int, dict] = {}
    for record in episodes:
        seed = int(record["seed"])
        if seed in index:
            raise ValueError(f"{path}: duplicate seed {seed}")
        index[seed] = record
    return index


def temporal_bin(seconds: float | None) -> str:
    """Bucket an override-to-collision distance in seconds (see TEMPORAL_BINS)."""
    if seconds is None:
        return "none"
    if seconds < 0.10:
        return "<0.10"
    if seconds < 0.25:
        return "0.10-0.25"
    if seconds < 0.50:
        return "0.25-0.50"
    if seconds < 1.00:
        return "0.50-1.00"
    if seconds <= 2.00:
        return "1.00-2.00"
    return ">2.00"


def _empty_bins() -> dict[str, int]:
    return {name: 0 for name in TEMPORAL_BINS}


def paired_episode_analysis(
    off_path: pathlib.Path | str, guarded_path: pathlib.Path | str
) -> dict:
    """Inner-join the OFF and GUARDED per-episode logs by ``seed``.

    Raises ``ValueError`` on a duplicate seed, a differing episode count, or any
    unmatched seed -- nothing is silently dropped.
    """
    off = _index_by_seed(load_episodes(off_path), off_path)
    guarded = _index_by_seed(load_episodes(guarded_path), guarded_path)

    if len(off) != len(guarded):
        raise ValueError(
            f"episode count mismatch: off={len(off)} guarded={len(guarded)}"
        )
    off_seeds, guarded_seeds = set(off), set(guarded)
    if off_seeds != guarded_seeds:
        off_only = sorted(off_seeds - guarded_seeds)[:5]
        guarded_only = sorted(guarded_seeds - off_seeds)[:5]
        raise ValueError(
            "unmatched seeds between off and guarded logs: "
            f"off_only={off_only} guarded_only={guarded_only}"
        )
    seeds = sorted(off_seeds)

    both_collision = 0
    both_survive = 0
    off_survive_on_collision = 0
    off_collision_on_survive = 0
    failure_pairs = {
        name: {"off_failure_on_survive": 0, "off_survive_on_failure": 0}
        for name in FAILURE_TYPES
    }

    def _pair(name: str) -> dict:
        return failure_pairs.setdefault(
            name, {"off_failure_on_survive": 0, "off_survive_on_failure": 0}
        )

    for seed in seeds:
        o, g = off[seed], guarded[seed]
        off_coll, guarded_coll = bool(o["collision"]), bool(g["collision"])
        if off_coll and guarded_coll:
            both_collision += 1
        elif not off_coll and not guarded_coll:
            both_survive += 1
        elif not off_coll and guarded_coll:
            off_survive_on_collision += 1
        else:
            off_collision_on_survive += 1
        # Final episode-level transitions only (never "some type occurred").
        if off_coll and not guarded_coll and o.get("failure") is not None:
            _pair(str(o["failure"]))["off_failure_on_survive"] += 1
        if (not off_coll) and guarded_coll and g.get("failure") is not None:
            _pair(str(g["failure"]))["off_survive_on_failure"] += 1

    cell_total = (
        both_collision
        + both_survive
        + off_survive_on_collision
        + off_collision_on_survive
    )
    if cell_total != len(seeds):
        raise ValueError(
            f"4-cell total {cell_total} != episode count {len(seeds)}"
        )

    off_collision_total = sum(1 for s in seeds if off[s]["collision"])
    guarded_collision_total = sum(1 for s in seeds if guarded[s]["collision"])
    on_gain = off_collision_on_survive
    on_loss = off_survive_on_collision
    net_episode_delta = on_gain - on_loss
    identity = off_collision_total - guarded_collision_total
    if net_episode_delta != identity:
        raise ValueError(
            f"net_episode_delta {net_episode_delta} != "
            f"off_collision_total - guarded_collision_total = {identity}"
        )

    # ---- override bookkeeping on the guarded side --------------------------
    override_episodes = sum(1 for s in seeds if guarded[s]["override_seen"])
    override_collided = sum(
        1 for s in seeds if guarded[s]["override_seen"] and guarded[s]["collision"]
    )
    override_survived = override_episodes - override_collided
    overrides_total = sum(len(guarded[s]["override_steps"]) for s in seeds)
    total_decisions = sum(int(guarded[s].get("decisions", 0)) for s in seeds)

    # ---- override -> collision timing (guarded collision episodes) ---------
    bins = _empty_bins()
    with_prior = 0
    without_prior = 0
    per_failure: dict[str, dict] = {}
    for name in FAILURE_TYPES:
        per_failure[name] = {
            "collision_episodes": 0,
            "prior_override_count": 0,
            "no_prior_override_count": 0,
            "bins": _empty_bins(),
        }
    for seed in seeds:
        g = guarded[seed]
        if not g["collision"]:
            continue
        seconds = g.get("seconds_since_last_override_before_collision")
        bins[temporal_bin(seconds)] += 1
        if seconds is None:
            without_prior += 1
        else:
            with_prior += 1
        key = str(g.get("failure") or "other")
        entry = per_failure.setdefault(
            key,
            {
                "collision_episodes": 0,
                "prior_override_count": 0,
                "no_prior_override_count": 0,
                "bins": _empty_bins(),
            },
        )
        entry["collision_episodes"] += 1
        entry["bins"][temporal_bin(seconds)] += 1
        if seconds is None:
            entry["no_prior_override_count"] += 1
        else:
            entry["prior_override_count"] += 1

    return {
        "episodes": len(seeds),
        "seed_first": seeds[0],
        "seed_last": seeds[-1],
        "both_collision": both_collision,
        "both_survive": both_survive,
        "off_survive_on_collision": off_survive_on_collision,
        "off_collision_on_survive": off_collision_on_survive,
        "off_collision_total": off_collision_total,
        "guarded_collision_total": guarded_collision_total,
        "on_gain": on_gain,
        "on_loss": on_loss,
        "net_episode_delta": net_episode_delta,
        "failure_pairs": failure_pairs,
        "override_episodes": override_episodes,
        "override_episode_survived": override_survived,
        "override_episode_collided": override_collided,
        "overrides_total": overrides_total,
        "total_decisions": total_decisions,
        "override_rate_per_decision": (
            overrides_total / total_decisions if total_decisions else 0.0
        ),
        "guarded_collision_episodes_with_prior_override": with_prior,
        "guarded_collision_episodes_without_prior_override": without_prior,
        "temporal_bins": bins,
        "temporal_by_failure": per_failure,
    }


# ==========================================================================
# reporting
# ==========================================================================
def print_paired_analysis(a: dict) -> None:
    print("\n[PAIRED EPISODE ANALYSIS]")
    print(f"episodes (seed join off x guarded) = {a['episodes']} "
          f"(seed {a['seed_first']}..{a['seed_last']})")
    print(f"both_collision              = {a['both_collision']}")
    print(f"both_survive                = {a['both_survive']}")
    print(f"off_survive_on_collision    = {a['off_survive_on_collision']}")
    print(f"off_collision_on_survive    = {a['off_collision_on_survive']}")
    total = (
        a["both_collision"]
        + a["both_survive"]
        + a["off_survive_on_collision"]
        + a["off_collision_on_survive"]
    )
    print(f"sum_check                   = {total} == {a['episodes']} "
          f"{'OK' if total == a['episodes'] else 'MISMATCH'}")
    print(f"on_gain                     = {a['on_gain']}")
    print(f"on_loss                     = {a['on_loss']}")
    print(f"net_episode_delta           = {a['net_episode_delta']}")
    identity = a["off_collision_total"] - a["guarded_collision_total"]
    print(f"identity_check              = net {a['net_episode_delta']} == "
          f"off_collision {a['off_collision_total']} - guarded_collision "
          f"{a['guarded_collision_total']} = {identity} "
          f"{'OK' if a['net_episode_delta'] == identity else 'MISMATCH'}")


def print_failure_paired_analysis(a: dict) -> None:
    print("\n[FAILURE-TYPE PAIRED ANALYSIS]")
    print("(episode-level final failure only)")
    # side_hit / corner_trap first -- those are the regression of interest.
    order = ["side_hit", "corner_trap", "front_hit", "other"]
    for name in order + [k for k in a["failure_pairs"] if k not in order]:
        pair = a["failure_pairs"][name]
        print(f"{name}:")
        print(f"    off_failure_on_survive  = {pair['off_failure_on_survive']}")
        print(f"    off_survive_on_failure  = {pair['off_survive_on_failure']}")
        print(f"    net                     = "
              f"{pair['off_failure_on_survive'] - pair['off_survive_on_failure']:+d}")


def print_temporal_analysis(a: dict) -> None:
    print("\n[OVERRIDE TEMPORAL ANALYSIS]")
    print(f"episodes_with_override      = {a['override_episodes']}")
    print(f"overrides_total             = {a['overrides_total']}")
    print(f"override_rate_per_decision  = {a['override_rate_per_decision'] * 100:.3f}%"
          f" ({a['overrides_total']}/{a['total_decisions']})")
    print(f"override_episode_survived   = {a['override_episode_survived']}")
    print(f"override_episode_collided   = {a['override_episode_collided']}")
    check = a["override_episode_survived"] + a["override_episode_collided"]
    print(f"outcome_sum_check           = {check} == {a['override_episodes']} "
          f"{'OK' if check == a['override_episodes'] else 'MISMATCH'}")
    print("")
    print("guarded collision episodes:")
    print(f"    with prior override     = "
          f"{a['guarded_collision_episodes_with_prior_override']}")
    print(f"    without prior override  = "
          f"{a['guarded_collision_episodes_without_prior_override']}")
    print("    seconds since last override before collision:")
    for name in TEMPORAL_BINS:
        print(f"        {name:<10} = {a['temporal_bins'][name]}")
    print("")
    print("[OVERRIDE TEMPORAL BY FAILURE TYPE]")
    for name in FAILURE_TYPES:
        entry = a["temporal_by_failure"][name]
        print(f"{name}: collisions={entry['collision_episodes']} "
              f"prior_override={entry['prior_override_count']} "
              f"no_prior_override={entry['no_prior_override_count']}")
        parts = ", ".join(f"{b}={entry['bins'][b]}" for b in TEMPORAL_BINS)
        print(f"    {parts}")


def _fmt(value) -> str:
    if isinstance(value, float):
        return f"{value:.4f}"
    return str(value)


def _print_block(summary: dict) -> None:
    print(f"\n--- {summary['name']} ---")
    for key in _SUMMARY:
        val = summary[key]
        if key == "collision_rate":
            print(f"  {key:<22} = {val * 100:.1f}%")
        elif isinstance(val, float):
            print(f"  {key:<22} = {val:.1f}")
        else:
            print(f"  {key:<22} = {val}")
    ltv = summary["ltv"]
    for key in _COUNTERS:
        val = ltv[key]
        if key.endswith("_rate"):
            print(f"  {key:<22} = {val * 100:.2f}%")
        else:
            print(f"  {key:<22} = {_fmt(val)}")
    ot = ltv["override_then"]
    print(f"  {'override_then':<22} = "
          f"front {ot['front_hit']} / side {ot['side_hit']} / "
          f"corner {ot['corner_trap']} / other {ot['other']}")


def _print_comparison(on: dict, off: dict) -> None:
    l_on, l_off = on["ltv"], off["ltv"]
    print("\n" + "=" * 72)
    print("SIDE-BY-SIDE  (identical episodes; only LTV differs)")
    print("=" * 72)
    print(f"{'metric':<26}{'LTV ON':>16}{'LTV OFF':>16}")
    for key in ("survival_mean", "collision_rate", "decision_us_mean", "changes_mean"):
        a, b = on[key], off[key]
        if key == "collision_rate":
            print(f"{key:<26}{a * 100:>15.1f}%{b * 100:>15.1f}%")
        else:
            print(f"{key:<26}{a:>16.1f}{b:>16.1f}")
    for key in _COUNTERS:
        a, b = l_on[key], l_off[key]
        if key.endswith("_rate"):
            print(f"{key:<26}{a * 100:>15.2f}%{b * 100:>15.2f}%")
        else:
            print(f"{key:<26}{_fmt(a):>16}{_fmt(b):>16}")
    print("\nnote: override_* are association statistics, not causation.")


def _run_both(args: argparse.Namespace) -> int:
    """Re-exec this script once per mode so each gets a clean import env."""
    prefix = args.episodes_jsonl
    if prefix is None:
        # Always produce the paired analysis in both-mode; keep the files in a
        # temp dir unless the caller asked for a persistent prefix.
        prefix = str(pathlib.Path(tempfile.mkdtemp(prefix="ltvdiag_")) / "ltv")

    results: dict[str, dict] = {}
    for mode in ("on", "off"):
        with tempfile.NamedTemporaryFile(suffix=f".{mode}.json", delete=False) as tmp:
            tmp_path = tmp.name
        cmd = [
            sys.executable, str(pathlib.Path(__file__).resolve()),
            "--episodes", str(args.episodes),
            "--seed", str(args.seed),
            "--mode", mode,
            "--json", tmp_path,
            "--episodes-jsonl", prefix,
        ]
        if args.debug:
            cmd.append("--debug")
        if args.max_steps is not None:
            cmd += ["--max-steps", str(args.max_steps)]
        proc = subprocess.run(
            cmd, cwd=str(ROOT), env={**os.environ, "LTVDIAG_CHILD": "1"}
        )
        if proc.returncode != 0:
            print(f"[error] mode={mode} exited with {proc.returncode}", file=sys.stderr)
            return proc.returncode
        results[mode] = json.loads(pathlib.Path(tmp_path).read_text(encoding="utf-8"))
        pathlib.Path(tmp_path).unlink(missing_ok=True)

    _print_comparison(results["on"], results["off"])

    off_path, guarded_path = _episode_paths(prefix)
    analysis = paired_episode_analysis(off_path, guarded_path)
    print_paired_analysis(analysis)
    print_failure_paired_analysis(analysis)
    print_temporal_analysis(analysis)

    results["episode_files"] = {"off": str(off_path), "guarded": str(guarded_path)}
    results["paired_analysis"] = analysis
    if args.json_path:
        out = pathlib.Path(args.json_path)
        out.write_text(json.dumps(results, indent=2), encoding="utf-8")
        print(f"\nwrote {out}")
    return 0


def main() -> int:
    args = _parse()
    if args.episodes < 1:
        print("--episodes must be >= 1", file=sys.stderr)
        return 2
    if args.mode == "both":
        if args.debug:
            print("[warn] --debug with --mode both prints a lot; prefer --mode on.",
                  file=sys.stderr)
        return _run_both(args)

    summary = _run_single(args, ltv_on=args.mode == "on")
    _print_block(summary)
    if args.json_path:
        out = pathlib.Path(args.json_path)
        out.write_text(json.dumps(summary, indent=2), encoding="utf-8")
        if not os.environ.get("LTVDIAG_CHILD"):
            print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

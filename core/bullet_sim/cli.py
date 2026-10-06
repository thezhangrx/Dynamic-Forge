"""Command line interface.

    python -m bullet_sim scenarios                 # list patterns and difficulty levels
    python -m bullet_sim scenario --level hard --seed 7 -o configs/hard.json
    python -m bullet_sim run --level medium --seed 7 --controller random --steps 1200
    python -m bullet_sim record --level extreme --count 4 --out data/ --format npz
    python -m bullet_sim benchmark --ladder 50 200 1000 --steps 200
    python -m bullet_sim replay --dataset data/ep_00000.npz
    python -m bullet_sim play --level hard --seed 3
    python -m bullet_sim info
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Sequence

import numpy as np

from bullet_sim.ai.policy import MODEL_NOT_AVAILABLE_YET
from bullet_sim.core.errors import ConfigError, ScenarioError
from bullet_sim.debug_log import start as start_debug_log


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="bullet_sim",
        description="2D bullet-hell / dynamic-obstacle simulation platform",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    # -- scenarios -------------------------------------------------------
    p = sub.add_parser("scenarios", help="list available patterns, levels and protocols")
    p.add_argument("--json", action="store_true", help="emit machine-readable JSON")

    # -- obstacles -------------------------------------------------------
    p = sub.add_parser(
        "obstacles",
        help="list the dynamic-obstacle catalogue and its real-world semantics",
    )
    p.add_argument("--json", action="store_true", help="emit machine-readable JSON")
    p.add_argument("--type", dest="type_key", default=None,
                   help="describe one obstacle type in full")
    p.add_argument("--scenario", dest="scenario_json", default=None,
                   help="inspect an ObstacleScenario .json instead of the catalogue")
    p.add_argument("--validate", action="store_true",
                   help="run the safety check on the scenario and print the report")
    p.add_argument("--safe-path", action="store_true",
                   help="also print one concrete collision-free path")

    # -- scenario --------------------------------------------------------
    p = sub.add_parser("scenario", help="build and export a scenario specification")
    _add_scenario_args(p)
    p.add_argument("-o", "--out", type=str, default=None, help="output .json path")
    p.add_argument("--built", action="store_true", help="also dump the materialised timeline")
    p.add_argument("--print", dest="show", action="store_true", help="print the spec to stdout")

    # -- run -------------------------------------------------------------
    p = sub.add_parser("run", help="headless single-episode run")
    _add_scenario_args(p)
    p.add_argument("--controller", default="stay", help="stay|random|constant:<n>|<module:callable>")
    p.add_argument("--input", default=None,
                   help="explicit control mode "
                        "(manual|auto|model|random|scripted|null|board|controller); "
                        "overrides --controller")
    p.add_argument("--auto", default="threat",
                   help="autonomous controller for --input auto: "
                        "idle|random|repulsion|threat|planner")
    p.add_argument("--model", default=None, help="trained policy (MODEL_NOT_AVAILABLE_YET)")
    p.add_argument("--collision-terminates", action="store_true",
                   help="Mode B: end the episode on a collision "
                        "(default: collision is a penalty event, episode continues)")
    p.add_argument("--collision-penalty", type=float, default=None,
                   help="reward subtracted per collision event (default: scenario value)")
    p.add_argument("--collision-count-mode", default=None,
                   choices=["per_contact", "per_step", "per_frame"],
                   help="how overlapping frames become collision events "
                        "(default: per_contact)")
    p.add_argument("--keep-alive", action="store_true",
                   help="deprecated alias: collisions never stop the run by default now")
    p.add_argument("--steps", type=int, default=None)
    p.add_argument("--json", action="store_true")

    # -- record ----------------------------------------------------------
    p = sub.add_parser("record", help="headless batch dataset generation")
    _add_scenario_args(p)
    p.add_argument("--count", type=int, default=4, help="number of episodes")
    p.add_argument("--out", type=str, default="data", help="output directory")
    p.add_argument("--format", default="npz", choices=["json", "npz", "csv", "bin"])
    p.add_argument("--controller", default="random")
    p.add_argument("--steps", type=int, default=None)
    p.add_argument("--bullet-stride", type=int, default=1)
    p.add_argument("--hashes", action="store_true", help="store per-step state hashes")

    # -- replay ----------------------------------------------------------
    p = sub.add_parser("replay", help="re-verify a recorded dataset from its seed+actions")
    p.add_argument("--dataset", required=True, type=str)
    p.add_argument("--json", action="store_true")

    # -- benchmark -------------------------------------------------------
    p = sub.add_parser("benchmark", help="run performance suites")
    p.add_argument("--ladder", type=int, nargs="*", default=None, help="bullet counts")
    p.add_argument("--levels", nargs="*", default=None)
    p.add_argument("--steps", type=int, default=200)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--collision", default="circle")
    p.add_argument("--out", type=str, default=None, help="write the JSON report here")
    p.add_argument("--full", action="store_true", help="also raw motion / generation suites")
    p.add_argument("--prediction", action="store_true",
                   help="include the prediction-latency suite")
    p.add_argument("--memory", action="store_true",
                   help="include the memory / allocation suite")

    # -- play ------------------------------------------------------------
    p = sub.add_parser("play", help="interactive / visual run (requires pygame)")
    _add_scenario_args(p)
    p.add_argument("--controller", default="stay")
    p.add_argument(
        "--input",
        default="manual",
        help="control mode: manual|auto|model|random|scripted|null|board|controller",
    )
    p.add_argument("--auto", default="threat",
                   help="autonomous controller used by --input auto and by TAB: "
                        "idle|random|repulsion|threat|planner")
    p.add_argument("--model", default=None,
                   help=f"trained policy path or registered name {MODEL_NOT_AVAILABLE_YET}")
    p.add_argument("--interactive", action="store_true",
                   help="legacy alias for --input manual")
    p.add_argument("--render-fps", type=float, default=60.0)
    p.add_argument("--sim-speed", type=float, default=1.0, help="sim seconds per real second")
    p.add_argument("--prediction", action="store_true", help="draw ballistic future trajectories")
    p.add_argument("--danger", action="store_true",
                   help="overlay the future danger field (red grid)")
    p.add_argument("--danger-every", type=int, default=4,
                   help="recompute the danger field every N rendered frames")
    p.add_argument("--horizon", type=int, default=90)
    p.add_argument("--scale", type=float, default=1.0)
    p.add_argument("--no-switch", action="store_true",
                   help="disable runtime MANUAL<->AUTO switching (no TAB binding)")
    p.add_argument("--max-seconds", type=float, default=None,
                   help="stop after N seconds of *simulation* time")
    p.add_argument("--max-steps", type=int, default=None, help="stop after N steps")
    p.add_argument("--collision-terminates", action="store_true",
                   help="Mode B: end the episode on a collision "
                        "(default: collision is a penalty event, episode continues)")
    p.add_argument("--collision-penalty", type=float, default=None,
                   help="reward subtracted per collision event (default: scenario value)")
    p.add_argument("--collision-count-mode", default=None,
                   choices=["per_contact", "per_step", "per_frame"],
                   help="how overlapping frames become collision events "
                        "(default: per_contact)")
    p.add_argument("--keep-alive", action="store_true",
                   help="deprecated alias: collisions never stop the run by default now")
    p.add_argument("--quiet", action="store_true", help="do not print the start-up banner")

    # -- autoplay --------------------------------------------------------
    p = sub.add_parser(
        "autoplay",
        help="headless evaluation of the autonomous controllers (no window)",
    )
    _add_scenario_args(p)
    p.add_argument("--controllers", nargs="*", default=None,
                   help="subset of idle|random|repulsion|threat|planner")
    p.add_argument("--steps", type=int, default=1200)
    p.add_argument("--seeds", nargs="*", type=int, default=None,
                   help="seeds to average over (default: the scenario seed only)")
    p.add_argument("--json", action="store_true")

    # -- info ------------------------------------------------------------
    sub.add_parser("info", help="print platform, protocol and dataset schema information")
    return parser


def _add_scenario_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--scenario", type=str, default=None, help="path to a scenario .json")
    p.add_argument(
        "--level",
        type=str,
        default="medium",
        help="easy|medium|hard|extreme|trivial or a complexity float in [0,1]",
    )
    p.add_argument("--bullets", type=int, default=None, help="target live bullets (overrides level)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--duration", type=float, default=None)
    p.add_argument("--dt", type=float, default=1.0 / 120.0)
    p.add_argument("--field", type=float, nargs=2, default=(640.0, 480.0))
    p.add_argument(
        "--action-space",
        default=None,
        help="discrete|velocity|acceleration|action "
             "(default: 'action' for run/play, 'discrete' for scenarios)",
    )
    p.add_argument("--player-radius", type=float, default=None,
                   help="radius of the player circle in world units (visible == "
                        "collision); obstacle sizes, gaps and corridor widths are "
                        "all quoted in multiples of its diameter")
    p.add_argument("--collision", default=None,
                   help="circle|grid|null|circle_distance|shaped|obstacle "
                        "(default: the scenario's own choice)")
    _add_obstacle_args(p)
    _add_safety_args(p)


def _add_obstacle_args(p: argparse.ArgumentParser) -> None:
    """Dynamic-obstacle scenario type selection (composable, repeatable)."""
    p.add_argument(
        "--obstacle-type", action="append", default=None, metavar="KEY",
        help="dynamic obstacle type, repeatable - list several to compose a "
             "scene: moving_block|wall_with_gap|small_obstacles|corridor|"
             "cross_traffic (see `python -m bullet_sim obstacles`)",
    )
    p.add_argument("--obstacle-count", type=int, default=None,
                   help="obstacles emitted per interval by the chosen type(s)")
    p.add_argument("--obstacle-speed", type=float, default=None,
                   help="obstacle speed in world units/second")
    p.add_argument("--obstacle-size", type=float, default=None,
                   help="obstacle size as a multiple of the player circle diameter "
                        "(0.35 small ... 4.5 huge)")
    p.add_argument("--obstacle-interval", type=float, default=None,
                   help="seconds between obstacle emissions")
    p.add_argument("--gap-width", type=float, default=None,
                   help="opening width for wall_with_gap in world units; may not be "
                        "smaller than the player circle diameter")
    p.add_argument("--corridor-width", type=float, default=None,
                   help="channel width for the corridor type in world units; may not "
                        "be smaller than the player circle diameter")
    p.add_argument("--corridor-min-width", type=float, default=None,
                   help="hard floor for the corridor gap in world units; it may not "
                        "go below 1.25 x the player diameter")
    p.add_argument("--corridor-walls", type=int, default=None, choices=[1, 2],
                   help="2 = two walls symmetric about the player (default), "
                        "1 = a single barrier")
    p.add_argument("--corridor-motion", default=None,
                   choices=["static", "open", "close", "rotate_same", "rotate_opposite"],
                   help="how the corridor changes: open/close (width), "
                        "rotate_same (tilt, constant width), rotate_opposite (scissor)")
    p.add_argument("--corridor-change", type=float, default=None,
                   help="total change over the episode: width delta in player "
                        "diameters (open/close) or degrees (rotate_*); clamped so the "
                        "channel can never crush the player")
    p.add_argument("--wall-length", type=float, default=None,
                   help="corridor wall length in world units")
    p.add_argument("--tilt-deg", type=float, default=None,
                   help="initial inclination of the whole corridor in degrees")
    p.add_argument("--entry-span", type=float, default=None,
                   help="fraction of the boundary edge a moving block may enter along")
    p.add_argument("--entry-angle", type=float, default=None,
                   help="entry direction in degrees (overrides the region default; "
                        "obstacles are always forced to travel into the field)")


def _add_safety_args(p: argparse.ArgumentParser) -> None:
    """Feasibility check: 有挑战 ≠ 必死."""
    p.add_argument("--safety-margin", type=float, default=None,
                   help="extra clearance beyond the player hitbox (world units)")
    p.add_argument("--safety-horizon", type=float, default=None,
                   help="how many seconds ahead feasibility must hold")
    p.add_argument("--require-valid", dest="require_valid", action="store_true", default=None,
                   help="refuse to run a scenario with no feasible safe path")
    p.add_argument("--no-require-valid", dest="require_valid", action="store_false",
                   help="build/run even if the safety check fails (debugging)")
    p.add_argument("--validate", action="store_true",
                   help="run the free-space / path feasibility check and print the report")
    p.add_argument("--safe-path", action="store_true",
                   help="also print one concrete collision-free path (implies --validate)")
    p.add_argument("--allow-infeasible", action="store_true",
                   help="with --validate, run anyway when the scenario is infeasible")


# --------------------------------------------------------------------------
# helpers
# --------------------------------------------------------------------------


def _level_value(level: str) -> Any:
    try:
        return float(level)
    except ValueError:
        return level


def _spec_from_args(args: argparse.Namespace):
    from bullet_sim.scenarios.builder import build_scenario
    from bullet_sim.scenarios.presets import stress
    from bullet_sim.scenarios.spec import ScenarioSpec

    if getattr(args, "scenario", None):
        return ScenarioSpec.load(args.scenario)
    if getattr(args, "obstacle_type", None):
        return _obstacle_scenario_from_args(args).to_spec()
    level_kwargs: dict[str, Any] = {}
    if getattr(args, "player_radius", None) is not None:
        level_kwargs["player_radius"] = float(args.player_radius)
    if getattr(args, "bullets", None):
        return stress(int(args.bullets), seed=args.seed, dt=args.dt, **level_kwargs)
    spec = build_scenario(
        _level_value(args.level),
        seed=args.seed,
        dt=args.dt,
        duration=getattr(args, "duration", None),
        field_w=args.field[0],
        field_h=args.field[1],
        **level_kwargs,
    ).spec
    if getattr(args, "action_space", None):
        spec = spec.replaced(action_space=args.action_space)
    return spec


def _obstacle_scenario_from_args(args: argparse.Namespace):
    """Build an :class:`ObstacleScenario` from ``--obstacle-*`` flags.

    Repeated ``--obstacle-type`` flags compose: each entry keeps its own
    parameters, and ``mixed_obstacles`` expands into its constituents.
    """
    from bullet_sim.obstacles.scenario import ObstacleScenario

    spawns: list[dict[str, Any]] = []
    for key in args.obstacle_type:
        entry: dict[str, Any] = {"type": key}
        for flag, field in (
            ("obstacle_count", "count"),
            ("obstacle_speed", "speed"),
            ("obstacle_size", "size"),
            ("obstacle_interval", "interval"),
            ("gap_width", "gap_width"),
            ("corridor_width", "corridor_width"),
            ("corridor_min_width", "corridor_min_width"),
            ("corridor_walls", "walls"),
            ("corridor_motion", "motion"),
            ("corridor_change", "change"),
            ("wall_length", "wall_length"),
            ("tilt_deg", "tilt_deg"),
            ("entry_span", "entry_span"),
            ("entry_angle", "entry_angle"),
        ):
            value = getattr(args, flag, None)
            if value is not None:
                entry[field] = value
        spawns.append(entry)

    kwargs: dict[str, Any] = {}
    if getattr(args, "duration", None) is not None:
        kwargs["duration"] = float(args.duration)
    if getattr(args, "safety_margin", None) is not None:
        kwargs["safety_margin"] = float(args.safety_margin)
    if getattr(args, "safety_horizon", None) is not None:
        kwargs["safety_horizon"] = float(args.safety_horizon)
    if getattr(args, "player_radius", None) is not None:
        kwargs["player_hitbox_radius"] = float(args.player_radius)
    if getattr(args, "require_valid", None) is not None:
        kwargs["require_valid"] = bool(args.require_valid)
    else:
        kwargs["require_valid"] = False
    return ObstacleScenario(
        name="+".join(args.obstacle_type),
        seed=int(args.seed),
        dt=float(args.dt),
        field_w=float(args.field[0]),
        field_h=float(args.field[1]),
        obstacles=spawns,
        **kwargs,
    )


def _run_safety_check(args: argparse.Namespace, spec: Any) -> tuple[Any, bool]:
    """Run the feasibility check when asked; return ``(report_or_None, ok)``.

    Triggered by ``--validate`` / ``--safe-path``, or by ``--require-valid``
    (which refuses to run an infeasible scene even without printing a report).
    ``--safety-margin`` / ``--safety-horizon`` override the scenario's own
    values, so the flags mean the same thing for every scenario source.
    """
    require = bool(getattr(args, "require_valid", False))
    explicit = bool(getattr(args, "validate", False) or getattr(args, "safe_path", False))
    if not (require or explicit):
        return None, True
    from bullet_sim.safety.validate import SafetyConfig, find_safe_path, validate_scenario

    margin = getattr(args, "safety_margin", None)
    horizon = getattr(args, "safety_horizon", None)
    config = SafetyConfig(
        margin=float(margin if margin is not None else getattr(spec, "safety_margin", 6.0)),
        horizon=float(horizon if horizon is not None else getattr(spec, "safety_horizon", 2.0)),
    )
    report = validate_scenario(spec, config=config)
    if explicit or not report.feasible:
        print(report.format(), file=sys.stderr)
    if report.feasible and getattr(args, "safe_path", False):
        path = find_safe_path(spec, config)
        if path is not None:
            print(f"safe path       : {path.length} waypoints, "
                  f"min clearance {path.min_clearance:.2f}, "
                  f"displacement {path.displacement():.2f}", file=sys.stderr)
    ok = bool(report.feasible) or bool(getattr(args, "allow_infeasible", False))
    return report, ok


def _collision_kwargs(args: argparse.Namespace, spec: Any) -> dict[str, Any]:
    """Collision-as-penalty configuration, all of it from flags/config.

    ``collision_terminates_episode`` defaults to False (the scenario value), so
    a collision is a penalty event unless someone explicitly asks for Mode B.
    """
    out: dict[str, Any] = {
        "terminate_on_collision": bool(getattr(args, "collision_terminates", False)),
    }
    mode = getattr(args, "collision_count_mode", None)
    if mode:
        out["collision_count_mode"] = mode
    penalty = getattr(args, "collision_penalty", None)
    if penalty is not None:
        spec.collision_penalty = float(penalty)  # type: ignore[attr-defined]
    return out


def _controller(spec_arg: str, space: Any, seed: int):
    if spec_arg.startswith("constant:"):
        return int(spec_arg.split(":", 1)[1])
    if ":" in spec_arg:
        module_name, fn_name = spec_arg.split(":", 1)
        import importlib

        module = importlib.import_module(module_name)
        return getattr(module, fn_name)
    return spec_arg


def _load_episode(path: str):
    from bullet_sim.dataset.readers import load_episode

    return load_episode(path)


# --------------------------------------------------------------------------
# commands
# --------------------------------------------------------------------------


def cmd_obstacles(args: argparse.Namespace) -> int:
    """The dynamic-obstacle catalogue: what each type simulates and why."""
    from bullet_sim.obstacles.catalog import CATALOG, GENERATABLE
    from bullet_sim.obstacles.spec import SAFETY_STRATEGIES, SIZE_PRESETS
    from bullet_sim.safety.validate import STRATEGY_NOTES

    if args.scenario_json:
        from bullet_sim.obstacles.scenario import ObstacleScenario
        from bullet_sim.safety.validate import SafetyConfig, find_safe_path, validate_scenario
        from bullet_sim.scenarios.spec import ScenarioSpec

        path = args.scenario_json
        try:
            scenario = ObstacleScenario.load(path)
            spec = scenario.to_spec()
            describe = scenario.describe()
            margin, horizon = scenario.safety_margin, scenario.safety_horizon
        except Exception:
            # also accept a plain ScenarioSpec export (`scenario -o ...json`)
            spec = ScenarioSpec.load(path)
            describe = {
                "name": spec.name,
                "seed": spec.seed,
                "duration": spec.duration,
                "field": [spec.field_w, spec.field_h],
                "obstacle_types": list(spec.meta.get("obstacle_types", [])),
                "patterns": [p.params.get("layout") for p in spec.patterns],
                "safety": {"margin": spec.safety_margin, "horizon": spec.safety_horizon},
            }
            margin, horizon = spec.safety_margin, spec.safety_horizon
        print(json.dumps(describe, indent=2, ensure_ascii=False))
        if args.validate or args.safe_path:
            config = SafetyConfig(margin=margin, horizon=horizon)
            report = validate_scenario(spec, config=config)
            print(report.format())
            if report.feasible and args.safe_path:
                safe = find_safe_path(spec, config)
                if safe is not None:
                    print(f"safe path: {safe.length} waypoints, "
                          f"min clearance {safe.min_clearance:.2f}")
        return 0

    if args.type_key:
        try:
            entry = CATALOG[args.type_key]
        except KeyError:
            print(f"unknown obstacle type {args.type_key!r}; available: {sorted(CATALOG)}",
                  file=sys.stderr)
            return 2
        payload = entry.to_dict()
        payload["safety_note"] = STRATEGY_NOTES.get(entry.safety_strategy, "")
        payload["generatable"] = entry.key in GENERATABLE
        if args.json:
            print(json.dumps(payload, indent=2, ensure_ascii=False))
        else:
            print(entry.describe())
            print(f"  safety check : {entry.safety_strategy} - {payload['safety_note']}")
            print(f"  defaults     : {json.dumps(dict(entry.defaults), ensure_ascii=False)}")
            print(f"  size presets : {json.dumps(SIZE_PRESETS)}")
        return 0

    payload = {
        "obstacle_types": [
            {**t.to_dict(), "safety_note": STRATEGY_NOTES.get(t.safety_strategy, "")}
            for t in CATALOG.values()
        ],
        "generatable": list(GENERATABLE),
        "size_presets": dict(SIZE_PRESETS),
        "safety_strategies": list(SAFETY_STRATEGIES),
    }
    if args.json:
        print(json.dumps(payload, indent=2, ensure_ascii=False))
        return 0
    for t in CATALOG.values():
        mark = "" if t.key in GENERATABLE else "  (composition)"
        print(f"{t.key:<24} {t.label_zh} / {t.label}{mark}")
        print(f"    simulates : {t.simulates}")
        print(f"    motion    : {t.motion}")
        print(f"    challenge : {t.player_problem}")
        print(f"    safety    : {t.safety_strategy} - "
              f"{STRATEGY_NOTES.get(t.safety_strategy, '')}")
    print()
    print(f"size presets (x player circle diameter): {json.dumps(SIZE_PRESETS)}")
    return 0


def cmd_scenarios(args: argparse.Namespace) -> int:
    from bullet_sim.core.actions import DISCRETE_ACTION_NAMES
    from bullet_sim.generators.patterns import available_patterns
    from bullet_sim.interface.protocol import describe_protocol
    from bullet_sim.scenarios.complexity import BULLET_COUNT_LADDER, LEVEL_VALUES
    from bullet_sim.scenarios.complexity import profile_for

    payload = {
        "patterns": available_patterns(),
        "difficulty_levels": LEVEL_VALUES,
        "bullet_count_ladder": list(BULLET_COUNT_LADDER),
        "discrete_actions": list(DISCRETE_ACTION_NAMES),
        "protocol": describe_protocol(),
        "complexity_profiles": {
            name: profile_for(value).to_dict() for name, value in LEVEL_VALUES.items()
        },
    }
    if args.json:
        print(json.dumps(payload, indent=2))
        return 0
    print("patterns        :", ", ".join(payload["patterns"]))
    print("difficulty      :", ", ".join(LEVEL_VALUES))
    print("bullet ladder   :", ", ".join(str(c) for c in BULLET_COUNT_LADDER))
    print("actions         :", ", ".join(DISCRETE_ACTION_NAMES))
    print("protocol        :", describe_protocol()["frame_size"])
    for name, value in LEVEL_VALUES.items():
        prof = profile_for(value)
        print(
            f"  {name:<8} complexity={value:.2f} target_bullets~{prof.bullet_count:<5}"
            f" patterns={len(prof.pattern_kinds)} duration={prof.duration:.1f}s"
        )
    return 0


def cmd_scenario(args: argparse.Namespace) -> int:
    from bullet_sim.scenarios.builder import build_from_spec

    spec = _spec_from_args(args)
    _report, ok = _run_safety_check(args, spec)
    if not ok:
        print("scenario rejected by the safety check; use --allow-infeasible to export anyway",
              file=sys.stderr)
        return 2
    built = build_from_spec(spec)
    if args.show or not args.out:
        print(spec.to_json())
    if args.out:
        path = spec.save(args.out)
        print(f"wrote {path}", file=sys.stderr)
        if args.built:
            bpath = built.save(str(Path(args.out).with_suffix(".built.json")))
            print(f"wrote {bpath}", file=sys.stderr)
    print(json.dumps(built.summary(), indent=2), file=sys.stderr)
    return 0


def cmd_run(args: argparse.Namespace) -> int:
    from bullet_sim.interface.controller import controller_from
    from bullet_sim.simulator.env import BulletHellEnv

    spec = _spec_from_args(args)
    _report, ok = _run_safety_check(args, spec)
    if not ok:
        print("scenario rejected by the safety check; use --allow-infeasible to run anyway",
              file=sys.stderr)
        return 2
    # ``action`` is the unified Action{direction, magnitude} codec and the right
    # default for a run: it is the only one that can express a magnitude, so
    # focus/slow and analog board input are not silently quantised to "stay".
    env = BulletHellEnv(
        spec, seed=args.seed, collision=args.collision,
        codec=args.action_space or "action",
        **_collision_kwargs(args, spec),
    )
    if getattr(args, "input", None):
        from bullet_sim.ai import make_play_source

        try:
            play = make_play_source(
                args.input, env=env, auto=args.auto, model=args.model,
                seed=args.seed, switchable=False,
            )
        except Exception as exc:
            print(f"cannot start input mode {args.input!r}: {exc}", file=sys.stderr)
            env.close()
            return 2
        result = env.run(input_source=play.source, steps=args.steps, seed=args.seed)
    else:
        ctrl = controller_from(_controller(args.controller, env.action_space, args.seed),
                               space=env.action_space, seed=args.seed)
        result = env.run(ctrl, steps=args.steps, seed=args.seed)
    env.close()
    if args.json:
        payload = {k: v for k, v in result.items() if k != "final_info"}
        payload["final_bullet_count"] = result["bullet_count"]
        print(json.dumps(payload, indent=2))
    else:
        print(f"scenario        : {spec.name} (seed={spec.seed})")
        print(f"steps survived  : {result['steps']}")
        print(f"return          : {result['return']:.2f}")
        print(f"collisions      : {result['collisions']}")
        print(f"bullets alive   : {result['bullet_count']}")
        print(f"terminated      : {result['terminated']}  truncated: {result['truncated']}")
        print(f"collision events: {result['collision_count']} "
              f"(mode {result['episode_metrics']['collision_count_mode']}, "
              f"{result['episode_metrics']['overlap_frames']} overlap frames)")
        print(f"collision rate  : {result['collision_rate_per_step']:.4f} /step  "
              f"({result['collision_rate_per_second']:.3f} /s)")
        print(f"survival time   : {result['survival_time']:.3f} s")
        print(f"final state hash: {result['state_hash']}")
    return 0


def cmd_record(args: argparse.Namespace) -> int:
    from bullet_sim.dataset.recorder import TrajectoryRecorder
    from bullet_sim.dataset.writers import save_episode
    from bullet_sim.interface.controller import controller_from
    from bullet_sim.simulator.env import BulletHellEnv

    outdir = Path(args.out)
    outdir.mkdir(parents=True, exist_ok=True)
    base_spec = _spec_from_args(args)
    stride = 1000003
    manifest: list[dict[str, Any]] = []
    for i in range(int(args.count)):
        spec = base_spec.replaced(seed=args.seed + i * stride, name=f"{base_spec.name}_{i:05d}")
        rec = TrajectoryRecorder(
            bullet_stride=args.bullet_stride,
            store_hashes=args.hashes,
            # Embedding the scenario makes the dataset self-reproducing.
            extra_meta={"scenario": spec.to_dict()},
        )
        env = BulletHellEnv(spec, seed=spec.seed, collision=args.collision, record=rec)
        ctrl = controller_from(_controller(args.controller, env.action_space, spec.seed),
                               space=env.action_space, seed=spec.seed)
        env.run(ctrl, steps=args.steps, seed=spec.seed)
        episode = rec.finish()
        path = save_episode(episode, outdir / f"ep_{i:05d}.{args.format}")
        spec.save(outdir / f"scenario_{i:05d}.json")
        summary = episode.summary()
        summary["path"] = str(path)
        manifest.append(summary)
        env.close()
        print(f"[{i + 1}/{args.count}] {path}  steps={summary['steps']} "
              f"max_bullets={summary['max_bullets']} collisions={summary['collisions']}")
    (outdir / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"wrote {outdir / 'manifest.json'}")
    return 0


def cmd_replay(args: argparse.Namespace) -> int:
    from bullet_sim.core.errors import DatasetError
    from bullet_sim.simulator.replay import verify_episode

    episode = _load_episode(args.dataset)
    try:
        comparison = verify_episode(episode)
    except DatasetError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    payload = {
        "dataset": args.dataset,
        "scenario_id": episode.scenario_id,
        "seed": episode.seed,
        "steps": episode.steps,
        "hashes_checked": comparison.expected_steps,
        "reproduced": comparison.ok,
        "first_mismatch": comparison.first_mismatch,
        "reason": comparison.reason,
        "policy": comparison.policy,
    }
    if args.json:
        print(json.dumps(payload, indent=2))
    else:
        for k, v in payload.items():
            print(f"{k:<16}: {v}")
    return 0 if comparison.ok else 1


def cmd_benchmark(args: argparse.Namespace) -> int:
    from bullet_sim.benchmark.suites import (
        DEFAULT_LADDER,
        bullet_ladder_suite,
        collision_backend_suite,
        difficulty_suite,
        format_report,
        memory_suite,
        prediction_suite,
    )
    from bullet_sim.benchmark.runner import benchmark_raw_motion, benchmark_scenario_generation

    counts = tuple(args.ladder) if args.ladder else DEFAULT_LADDER
    report: dict[str, Any] = {"seed": args.seed, "steps": args.steps}
    report["bullet_ladder"] = bullet_ladder_suite(
        counts, steps=args.steps, seed=args.seed, collision=args.collision, verbose=True
    )
    if args.levels:
        report["difficulty"] = difficulty_suite(
            tuple(args.levels), steps=args.steps, seed=args.seed, collision=args.collision
        )
    report["collision_backends"] = collision_backend_suite(
        bullet_count=min(max(counts), 2000), seed=args.seed
    )
    if args.prediction or args.full:
        report["prediction"] = prediction_suite(seed=args.seed)
    if args.memory or args.full:
        report["memory"] = memory_suite(counts, seed=args.seed)
    if args.full:
        report["raw_motion"] = benchmark_raw_motion(max(counts)).to_dict()
        report["scenario_generation"] = benchmark_scenario_generation("hard", seed=args.seed).to_dict()
    print()
    print(format_report(report))
    if args.out:
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"\nwrote {path}")
    return 0


def _print_play_banner(args, spec, play, env) -> None:
    """Tell the player, in the console, everything the HUD shows."""
    keys = (
        "arrows / WASD = move   SPACE = stop   SHIFT = focus (0.4x)   "
        "CTRL = slow (0.25x)"
    )
    print("=" * 74)
    print(f"  bullet_sim  -  scenario '{spec.name}'  (seed={spec.seed}, dt={spec.dt:.5f}s)")
    print("=" * 74)
    print(f"  control mode : {play.mode.upper()}")
    print(f"  source       : {play.name}")
    if play.note:
        print(f"  note         : {play.note}")
    if play.controller is not None and hasattr(play.controller, "spec"):
        try:
            info = play.controller.spec()
            print(f"  controller   : {info.name}  [{info.kind}/{info.access}/{info.cost}]")
            print(f"                 {info.description}")
        except Exception:  # pragma: no cover - never block start-up
            pass
    print(f"  bullets      : ~{spec.meta.get('target_bullet_count', '?')} target, "
          f"capacity {env.world.state.bullets.capacity}")
    print(f"  collisions   : penalty event (penalty={spec.collision_penalty}, "
          f"mode={spec.collision_count_mode}) - episode continues")
    if env.world.terminate_on_collision:
        print("                 MODE B: --collision-terminates is ON, "
              "a collision ends the episode")
    print("  keys         : " + keys)
    print("                 TAB = switch MANUAL <-> AUTONOMOUS   H = HUD   "
          "ESC/Q = quit")
    print("=" * 74)


def cmd_play(args: argparse.Namespace) -> int:
    import time

    from bullet_sim.action.base import action_to_codec_input
    from bullet_sim.ai import make_play_source
    from bullet_sim.entities.player import P_SPEED
    from bullet_sim.render.base import make_renderer
    from bullet_sim.simulator.env import BulletHellEnv

    spec = _spec_from_args(args)
    _report, ok = _run_safety_check(args, spec)
    if not ok:
        print("scenario rejected by the safety check; use --allow-infeasible to run anyway",
              file=sys.stderr)
        return 2
    env = BulletHellEnv(
        spec, seed=args.seed, collision=args.collision,
        codec=args.action_space or "action",
        **_collision_kwargs(args, spec),
    )

    mode = "manual" if args.interactive else args.input
    switchable = not args.no_switch
    try:
        play = make_play_source(
            mode,
            env=env,
            auto=args.auto,
            model=args.model,
            controller=_controller(args.controller, env.action_space, args.seed),
            seed=args.seed,
            switchable=switchable,
        )
    except Exception as exc:
        print(f"cannot start input mode {mode!r}: {exc}", file=sys.stderr)
        env.close()
        return 2

    if not args.quiet:
        _print_play_banner(args, spec, play, env)

    predictor = None
    if args.prediction:
        from bullet_sim.prediction.ballistic import BallisticPredictor

        predictor = BallisticPredictor()
    try:
        renderer = make_renderer(
            "human",
            scale=args.scale,
            show_prediction=args.prediction,
            predictor=predictor,
            prediction_horizon=args.horizon,
            show_danger=args.danger,
            interactive=False,
            input_source=play.source,
            controller=play.controller,
        )
    except Exception as exc:
        print(f"cannot open a window ({exc}).", file=sys.stderr)
        print("  headless instead:  python -m bullet_sim run "
              f"--input {mode} --steps {args.max_steps or 1200}", file=sys.stderr)
        env.close()
        return 2

    source = play.source
    obs, info = env.reset(seed=args.seed)
    source.open()
    from bullet_sim.action import Action

    last_action = Action.zero()
    sim_per_frame = max(1, int(round(args.sim_speed / args.render_fps / spec.dt)))
    frame_dt = 1.0 / args.render_fps
    step_limit = args.max_steps if args.max_steps else spec.total_steps
    if args.max_seconds is not None:
        step_limit = min(step_limit, int(round(args.max_seconds / spec.dt)))
    steps_done = 0
    frame_index = 0
    terminated = truncated = False
    from bullet_sim.prediction.danger_field import compute_danger_field

    try:
        while steps_done < step_limit:
            if callable(getattr(source, "poll_observation", None)):
                last_action = source.poll_observation(obs, info)
            else:
                polled = source.poll(frame_dt)
                if polled is not None:
                    last_action = polled
            raw = action_to_codec_input(
                last_action, env.world.codec, float(env.world.player[P_SPEED])
            )
            for _ in range(sim_per_frame):
                if steps_done >= step_limit:
                    break
                obs, _r, terminated, truncated, info = env.step(raw)
                steps_done += 1
                if terminated or truncated:
                    break
            if args.danger and frame_index % max(1, args.danger_every) == 0:
                try:
                    renderer.danger = compute_danger_field(
                        env.simulate_future(0, horizon=8, include_initial=True),
                        resolution=16.0,
                    )
                except Exception:  # pragma: no cover - overlay must never break the loop
                    renderer.danger = None
            frame_index += 1
            renderer.render(env.world)
            if renderer.closed or terminated or truncated:
                break
            time.sleep(frame_dt)
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        renderer.close()
        env.close()
    metrics = env.episode_metrics()
    print(
        f"ran {steps_done} steps ({steps_done * spec.dt:.2f}s of simulation)"
        f"  terminated={terminated} truncated={truncated}"
    )
    print(f"collisions   : {metrics['total_collision_count']} event(s) "
          f"across {metrics['overlap_frames']} overlap frame(s)"
          f"  ({metrics['collision_count_mode']})")
    print(f"reward       : {metrics['cumulative_reward']:+.2f}")
    print(f"input source : {getattr(source, 'name', type(source).__name__)}")
    if hasattr(source, "switch_count"):
        print(f"mode switches: {source.switch_count}  history="
              f"{[(e.from_mode, e.to_mode) for e in source.history]}")
    return 0


def cmd_autoplay(args: argparse.Namespace) -> int:
    """Headless evaluation of the autonomous controllers - no window needed."""
    from bullet_sim.ai import make_controller
    from bullet_sim.simulator.env import BulletHellEnv

    spec = _spec_from_args(args)
    names = args.controllers or ["idle", "random", "repulsion", "threat", "planner"]
    seeds = list(args.seeds) if args.seeds else [args.seed]
    rows: list[dict[str, Any]] = []
    for name in names:
        totals = {
            "controller": name,
            "steps": 0,
            "return": 0.0,
            "collisions": 0,
        }
        for sd in seeds:
            env = BulletHellEnv(
                spec.replaced(seed=sd), seed=sd, terminate_on_collision=True
            )
            kwargs: dict[str, Any] = {"world": env.world}
            if name == "planner":
                kwargs["horizon"] = 18
            if name == "random":
                kwargs["seed"] = sd
            result = env.run(make_controller(name, **kwargs), steps=args.steps, seed=sd)
            env.close()
            totals["steps"] += result["steps"]
            totals["return"] += result["return"]
            totals["collisions"] += result["collisions"]
        n = max(len(seeds), 1)
        row = {
            "controller": name,
            "mean_steps": round(totals["steps"] / n, 1),
            "mean_return": round(totals["return"] / n, 1),
            "collisions": totals["collisions"],
        }
        spec_controller = None
        try:
            spec_controller = make_controller(name, world=None).spec() if name != "planner" else None
        except Exception:
            pass
        if spec_controller is not None:
            row["access"] = spec_controller.access
            row["cost"] = spec_controller.cost
        rows.append(row)

    if args.json:
        print(json.dumps({"scenario": spec.name, "seeds": seeds,
                          "steps": args.steps, "rows": rows}, indent=2))
        return 0
    try:
        from bullet_sim.benchmark.metrics import format_table

        print(format_table(rows))
    except Exception:  # pragma: no cover
        for r in rows:
            print(r)
    print()
    print(f"scenario '{spec.name}' (seed {spec.seed}), {len(seeds)} seed(s), "
          f"cap {args.steps} steps")
    print("note: 'planner' has world access (it clones the world), the others are "
          "observation-only.")
    return 0


def cmd_info(args: argparse.Namespace) -> int:
    from bullet_sim.core.version import DATASET_SCHEMA_VERSION, STATE_PROTOCOL_VERSION, __version__
    from bullet_sim.dataset.schema import ARRAY_KEYS
    from bullet_sim.dataset.writers import supported_formats
    from bullet_sim.interface.protocol import describe_protocol

    print(f"bullet_sim version   : {__version__}")
    print(f"state protocol       : v{STATE_PROTOCOL_VERSION}")
    print(f"dataset schema       : v{DATASET_SCHEMA_VERSION}")
    print(f"dataset formats      : {', '.join(supported_formats())}")
    print(f"dataset array keys   : {', '.join(ARRAY_KEYS)}")
    print("hardware protocol    :")
    for k, v in describe_protocol().items():
        print(f"  {k:<20}: {v}")
    try:
        import os

        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame  # noqa: F401

        print("pygame               : available")
    except Exception:
        print("pygame               : NOT available (ascii/none render only)")
    print(f"numpy                : {np.__version__}")
    return 0


_COMMANDS = {
    "scenarios": cmd_scenarios,
    "obstacles": cmd_obstacles,
    "scenario": cmd_scenario,
    "run": cmd_run,
    "record": cmd_record,
    "replay": cmd_replay,
    "benchmark": cmd_benchmark,
    "play": cmd_play,
    "autoplay": cmd_autoplay,
    "info": cmd_info,
}


def _run_command(args: argparse.Namespace) -> int:
    try:
        return int(_COMMANDS[args.command](args))
    except (ConfigError, ScenarioError) as exc:
        # A bad configuration is a user error, not a crash: explain and exit 2.
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    # One debug log per launch: data/bullet_sim/debug_<date stamp>.log
    log = start_debug_log(args.command, argv)
    if log is None:
        return _run_command(args)
    with log:
        log.exit_code = _run_command(args)
    return log.exit_code


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

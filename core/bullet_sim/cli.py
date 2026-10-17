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

#: `play` / `board-control` 的默认障碍分批进场间隔（秒）。
#:
#: 这两条命令是给人看、给摄像头看的：开场把 3 种障碍一次全放上去，视觉链路
#: 根本来不及分辨"哪个是墙、哪个是目标"。默认每 8 秒再解锁一种，第一段只有
#: 墙+缺口（也正是板子侧 `gap_avoid` 唯一能可靠读出的几何）。
DEFAULT_STAGE_SECONDS: float = 8.0

#: **板子在环里时**（`board-control` / `play --input board`）默认只放的障碍类型。
#:
#: 板子的决策（`core/board/main.cpp` → `gap_avoid`）只对"墙 + 一个缺口"有明确
#: 语义：墙在哪、缺口在哪、来不来得及。移动块/小障碍没有这个概念，场上多一种
#: 障碍只会让摄像头画面更难读（实测过：三种一起上时板子的指令会恒定向一边）。
#: 所以板子控制平台时**只放墙+缺口这一种**；难度（更快、更窄、更密）照旧由
#: `--level` 控制，类型不会跟着变多。
BOARD_KINDS: tuple[str, ...] = ("wall_with_gap",)


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
    p.add_argument("--nostop", action="store_true",
                   help="**不许自己停**：不看回合步数、也不因 terminated/truncated 结束"
                        "（不设 --steps 时一直跑，Ctrl-C 停）")
    p.add_argument("--port", type=str, default=None, help="板子信号来自串口（--input board）")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--udp", type=int, default=None, metavar="PORT",
                   help="板子信号来自 UDP（--input board）")
    p.add_argument("--stdin", dest="use_stdin", action="store_true",
                   help="板子信号来自标准输入（--input board）")
    p.add_argument("--hold-ms", type=int, default=500, help="断流多久回中（默认 500ms）")
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
    _add_play_args(p)

        # -- board-control ---------------------------------------------------
    p = sub.add_parser(
        "board-control",
        help="板子控制平台：板子的 @MOVE n → 仿真动作，**默认不自动停**"
             "（传输：--port 串口 / --udp 端口 / --stdin 管道）",
    )
    _add_play_args(p, input_default="board")
    p.set_defaults(nostop=True)     # 这个子命令的默认就是"一直跑"

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


def _add_play_args(p: argparse.ArgumentParser, *, input_default: str = "manual") -> None:
    """`play` 与 `board-control` **共用**的一组参数。

    共用是必须的：`board-control` 就是"输入源换成板子、默认不自动停"的 `play`；
    两边各写一份参数迟早会分家（比如新加一个 `--danger` 只加在一边）。

    这两条命令都是**给人看/给摄像头看**的，所以障碍默认分批进场（`--stage-seconds`）：
    开场只有一种障碍，视觉链路先跑通再接难度。
    """
    _add_scenario_args(p, stage_default=DEFAULT_STAGE_SECONDS)
    p.add_argument("--controller", default="stay")
    p.add_argument(
        "--input",
        default=input_default,
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
    p.add_argument("--potential", action="store_true",
                   help="画**决策势场**（移植自 core/cpu/field_demo.py：分层等值带 + "
                        "膨胀障碍 + 缺口目标点）。与 --danger（未来子弹威胁）不同")
    p.add_argument("--danger", action="store_true",
                   help="overlay the future danger field (red grid)")
    p.add_argument("--danger-every", type=int, default=4,
                   help="recompute the danger field every N rendered frames")
    p.add_argument("--horizon", type=int, default=90)
    p.add_argument(
        "--state-log",
        default=None,
        metavar="FILE",
        help="**仿真真值日志**（每帧一行 JSON：角色与所有障碍的世界坐标）。"
             "配合 core/board/tools/score_vs_sim.py，把板子报的几何与真值按场地单位对账",
    )
    p.add_argument("--fullscreen", action="store_true",
                   help="全屏显示场地（调试视觉时建议开：场地就是整个屏幕，"
                        "窗口外的桌面内容不会再被当成目标）")
    p.add_argument("--gap-position", type=float, default=None,
                   help="wall_with_gap 缺口在场地宽度上的位置 0~1（默认 0.5 = 正中）")
    p.add_argument("--gap-motion", default=None, choices=("static", "sweep"),
                   help="缺口动不动（默认 sweep；录调试视频时建议 static，"
                        "否则缺口会自己游走到角色身上，测不出控制有没有生效）")
    p.add_argument("--calib-marker", action="store_true",
                   help="draw the calibration reference at the field origin "
                        "(for a camera filming the screen)")
    p.add_argument("--calib-marker-w", type=float, default=0.90,
                   help="reference width as a fraction of field_w (default 0.90)")
    p.add_argument("--calib-marker-h", type=float, default=0.90,
                   help="reference height as a fraction of field_h (default 0.90)")
    p.add_argument("--calib-marker-color", default="800080",
                   help="reference colour as RRGGBB hex (default 800080 half-intensity magenta; "
                        "magenta is immune to the room's white glare)")
    p.add_argument("--wall-color", default="F0A030",
                   help="墙（矩形障碍）的颜色 RRGGBB。**这是和视觉链路的契约**：PL 判墙的"
                        "条件是 r-b>40 且 r>110 且 g>60，而摄像头会把屏幕上**又亮又橙**的墙"
                        "洗成白色（实测：屏幕 (240,160,48) → 相机里 r-b 只剩 20~60，"
                        "半边被判成'亮'，墙就消失、缺口无从谈起）。把墙调得**更深更饱和**"
                        "（例如 C83C00 = (200,60,0)）反而更耐洗。默认 F0A030")
    p.add_argument("--no-fiducials", action="store_true",
                   help="不画四角 ArUco 标定标记。板子在环里时建议加：标记**画在所有东西之上**，"
                        "墙/角色经过角落会被它盖住；它本身也会被相机判成矩形/亮块，"
                        "实测让几何帧里多出 1~7 个假矩形，决策跟着乱")
    p.add_argument("--plain-field", action="store_true",
                   help="**场地只留墙和角色**：不画网格、不画场地外框。相机拍屏幕时那一圈"
                        "外框是一条横贯画面的长线，视觉会把它判成矩形（实测几何帧里多出"
                        "一条贴底边的「墙」）；网格线同理会变成细长假矩形。板子在环里时加")
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

    # -- 「平台不许自己停」与板子输入 ------------------------------------
    p.add_argument("--nostop", action="store_true",
                   help="**跑起来就不自动停**：不看回合步数上限、也不因 terminated/truncated "
                        "结束（只由 Ctrl-C / 关窗口停）。板子控制平台时用这个")
    p.add_argument("--port", type=str, default=None,
                   help="板子信号来自串口（如 /dev/ttyUSB0 / COM3）")
    p.add_argument("--baud", type=int, default=115200, help="串口波特率（默认 115200）")
    p.add_argument("--udp", type=int, default=None, metavar="PORT",
                   help="板子信号来自 UDP（板子侧 --keyboard-dev udp://本机IP:PORT）")
    p.add_argument("--stdin", dest="use_stdin", action="store_true",
                   help="板子信号来自标准输入（ssh 板子 '... --keyboard-dev stdout' | ...）")
    p.add_argument("--hold-ms", type=int, default=500,
                   help="断流多久回中（默认 500ms，与键盘桥/看门狗一致）")
    p.add_argument("--headless", action="store_true",
                   help="不开窗口、只跑（给无显示的机器/自测用）")
    # 这两个是 `--headless` 时才用得上的（headless 走的是 run 那条路）：
    # `--steps` 只是 `--max-steps` 更顺手的写法（同一个 dest），`--json` 出报告。
    p.add_argument("--steps", type=int, default=None, dest="max_steps",
                   help="同 --max-steps（--headless 时更顺手）")
    p.add_argument("--json", action="store_true", help="--headless 时输出 JSON 报告")


    p.add_argument("--quiet", action="store_true", help="do not print the start-up banner")


def _add_scenario_args(
    p: argparse.ArgumentParser, *, stage_default: float | None = None
) -> None:
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
        "--stage-seconds",
        type=float,
        default=stage_default,
        metavar="S",
        help="**开场不要一次全放**：第 1 种障碍从 0 秒开始，之后每 S 秒再多放一种"
             "（顺序：墙缺口 → 移动块 → 小障碍 → 交叉 → 走廊）。0 = 关闭（所有类型"
             "开场就到齐，历史行为）；不填 = 按命令的默认值"
             + (f"（本命令默认 {stage_default:g} 秒）" if stage_default else "（默认关闭）"),
    )
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
             "cross_traffic (see `python -m bullet_sim obstacles`)。"
             "只给一种 = 只看这一种；配合 `--obstacle-cycle S` = 逐个类型分开展示",
    )
    p.add_argument(
        "--obstacle-cycle", type=float, default=0.0, metavar="S",
        help="**把障碍分开展示**：每个类型独占 S 秒档期，整轮（类型数 × S 秒）循环，"
             "同一时刻场上只有一种 —— 想逐个类型验视觉/决策时用这个。"
             "顺序：`--obstacle-type` 就按你写的先后，难度场景按"
             "墙缺口 → 移动块 → 小障碍 → 交叉 → 走廊。0 = 关（默认）。"
             "给了它就接管 `--stage-seconds`（分批是「叠加」，轮播是「独占」）",
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
    p.add_argument("--region", default=None,
                   choices=["left", "right", "top", "bottom"],
                   help="which boundary boundary-spawned obstacles enter from "
                        "(small_obstacles / moving_block); they travel toward the "
                        "opposite boundary. Default comes from the obstacle type.")


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


def parse_hex_color(text: str) -> tuple[int, int, int]:
    """``"FF00FF"`` / ``"#ff00ff"`` → ``(255, 0, 255)``。

    只接受 6 位十六进制；解析不出来时抛 ``ValueError``（由命令行报错），
    不做静默回退 —— 颜色写错时画出个别的颜色比直接报错更难查。
    """
    s = str(text).strip().lstrip("#")
    if len(s) != 6:
        raise ValueError(f"colour must be 6 hex digits (RRGGBB), got {text!r}")
    try:
        v = int(s, 16)
    except ValueError:
        raise ValueError(f"colour must be 6 hex digits (RRGGBB), got {text!r}") from None
    return ((v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)


def _spec_from_args(args: argparse.Namespace):
    """构造本次运行要用的 ``ScenarioSpec``。

    三条来源（``--scenario`` 文件 / ``--obstacle-type`` 组合 / ``--level`` 难度）
    都会应用"按类型分时间"的两个开关：

    * ``--obstacle-cycle S``（**分开展示**）：每个类型独占 S 秒，整轮循环；
    * ``--stage-seconds S``（分批进场）：开场一种，之后每 S 秒**再多**一种。

    后者给 `--obstacle-cycle` 让路（轮播更彻底，两个一起给没有意义）。难度那条走
    ``make_scenario(...)``，因为分时间必须在"标定数量 + 安全校验"之前发生，
    否则 meta 里记的峰值描述的不是真正会跑的场景。

    ``--obstacle-type`` 的书写顺序就是展示顺序（用户一个个列出来，那就是他要的顺序）。

    **板子在环里**（`board-control` / `play --input board`）时，场景**只能有一种**
    障碍类型，由 `--obstacle-type` 指定（不给就是 :data:`BOARD_KINDS` = 墙+缺口）：
    板子的决策只对那种几何有语义，场上多一种只会让画面更难读、指令更没意义。
    给多种类型 / `--obstacle-cycle` / 多类型场景文件都**直接报错**，不静默放宽。
    难度（更快、更窄、更密）照旧由 `--level` 控制，只是类型不会跟着变多。
    """
    from bullet_sim.scenarios.builder import (
        build_scenario,
        cycle_spec,
        kind_order,
        stage_spec,
    )
    from bullet_sim.scenarios.presets import stress
    from bullet_sim.scenarios.spec import ScenarioSpec

    cycle = float(getattr(args, "obstacle_cycle", 0.0) or 0.0)
    stage = 0.0 if cycle > 0.0 else float(getattr(args, "stage_seconds", None) or 0.0)
    order = list(getattr(args, "obstacle_type", None) or []) or None
    board_mode = _is_board_mode(args)
    if board_mode:
        if cycle > 0.0:
            raise ConfigError(
                "板子控制平台时只能有一种障碍类型：`--obstacle-cycle` 会让场上换类型。"
                "要看逐个类型用 `play --obstacle-cycle S`（不带板子）。")
        if order and len(order) > 1:
            raise ConfigError(
                f"板子控制平台时只能有一种障碍类型：`--obstacle-type` 只给一个，"
                f"收到 {len(order)} 个（{', '.join(order)}）。"
                "要看多种障碍用 `play`。")
    kinds: tuple[str, ...] | None = None
    if (board_mode and not order
            and not getattr(args, "scenario", None) and not getattr(args, "bullets", None)):
        kinds = BOARD_KINDS
    level_kwargs: dict[str, Any] = {}
    if getattr(args, "player_radius", None) is not None:
        level_kwargs["player_radius"] = float(args.player_radius)
    if kinds is not None:
        level_kwargs["kinds"] = kinds
    if getattr(args, "scenario", None):
        spec = ScenarioSpec.load(args.scenario)
        spec = cycle_spec(spec, cycle) if cycle > 0.0 else stage_spec(spec, stage)
    elif getattr(args, "obstacle_type", None):
        spec = _obstacle_scenario_from_args(args).to_spec()
        spec = (cycle_spec(spec, cycle, order=order) if cycle > 0.0
                else stage_spec(spec, stage, order=order))
    elif getattr(args, "bullets", None):
        spec = stress(
            int(args.bullets), seed=args.seed, dt=args.dt,
            stage_seconds=stage, cycle_seconds=cycle, **level_kwargs,
        )
    else:
        spec = build_scenario(
            _level_value(args.level),
            seed=args.seed,
            dt=args.dt,
            duration=getattr(args, "duration", None),
            field_w=args.field[0],
            field_h=args.field[1],
            stage_seconds=stage,
            cycle_seconds=cycle,
            **level_kwargs,
        ).spec
    if getattr(args, "action_space", None):
        spec = spec.replaced(action_space=args.action_space)
    if board_mode:
        present = kind_order(spec.patterns)
        if len(present) != 1:
            raise ConfigError(
                f"板子控制平台时只能有一种障碍类型，这个场景有 {len(present)} 种"
                f"（{', '.join(present) or '无'}）：换个场景文件，或显式"
                " `--obstacle-type <一种>`；要看多种障碍用 `play`。")
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
            ("gap_position", "gap_position"),
            ("gap_motion", "gap_motion"),
            ("corridor_width", "corridor_width"),
            ("corridor_min_width", "corridor_min_width"),
            ("corridor_walls", "walls"),
            ("corridor_motion", "motion"),
            ("corridor_change", "change"),
            ("wall_length", "wall_length"),
            ("tilt_deg", "tilt_deg"),
            ("entry_span", "entry_span"),
            ("entry_angle", "entry_angle"),
            ("region", "region"),
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


def _probe_display(timeout_s: float = 6.0) -> tuple[bool, str]:
    """**开窗口之前**先在一个子进程里试一次开窗（能不能开、会不会卡）。

    为什么必须放在子进程里：`pygame.display.set_mode()` 卡在 X/WSLg 里时是**握着 GIL
    的 C 调用** —— 同进程里的看门狗线程等不到 GIL、信号处理器也递不进去（实测发
    SIGTERM 后进程状态一直是 R）。只有**子进程**能被 `timeout` 杀掉，于是"静默卡死"
    才能变成"6 秒后一句能照做的诊断"。

    返回 ``(ok, 说明)``。
    """
    import subprocess
    code = (
        "import pygame;"
        "pygame.display.init();"
        "pygame.display.set_mode((64, 64));"
        "print('ok')"
    )
    try:
        r = subprocess.run([sys.executable, "-c", code], capture_output=True,
                           text=True, timeout=timeout_s)
    except subprocess.TimeoutExpired:
        return False, "超时（显示子系统没响应）"
    if r.returncode != 0:
        tail = (r.stderr or r.stdout or "").strip().splitlines()
        return False, (tail[-1] if tail else f"退出码 {r.returncode}")
    return "ok" in r.stdout, "ok"


def _run_steps(args: argparse.Namespace, spec: Any) -> int | None:
    """无窗口跑的步数；`None` = 一直跑（配合 `stop_on_done=False`）。

    参数名要兼容两条调用路：`run` 用 `--steps`，`board-control`（复用 play 的参数）
    用 `--max-steps`/`--max-seconds`。显式给的步数**永远优先于** `--nostop`。
    """
    steps = getattr(args, "steps", None)
    if steps is None:
        steps = getattr(args, "max_steps", None)
    secs = getattr(args, "max_seconds", None)
    if secs is not None:
        by_time = int(round(float(secs) / spec.dt))
        steps = by_time if steps is None else min(int(steps), by_time)
    return None if steps is None else int(steps)


def _board_source_kwargs(args: argparse.Namespace) -> dict[str, Any]:
    """把 CLI 上的传输参数收成给 `make_play_source(board=...)` 的字典。

    一个传输都不给就直接报错：让它默默用占位适配器"拒绝产出"的话，用户只会看到
    "角色一动不动"，然后去怀疑板子 —— 这是最浪费时间的一种失败。
    """
    hold_s = max(0.0, float(getattr(args, "hold_ms", 500)) / 1000.0)
    if getattr(args, "port", None):
        return {"port": args.port, "baud": int(getattr(args, "baud", 115200)),
                "hold_s": hold_s}
    if getattr(args, "udp", None) is not None:
        return {"udp": int(args.udp), "hold_s": hold_s}
    if getattr(args, "use_stdin", False):
        return {"stdin": True, "hold_s": hold_s}
    raise ValueError("板子输入要指定传输：--port <串口> / --udp <端口> / --stdin")


def cmd_board_control(args: argparse.Namespace) -> int:
    """板子控制平台：板子的 `@MOVE n` → 仿真里的动作，**默认不自动停**。

    实现上就是 `play`（`--headless` 时是 `run`）换个输入源 —— **刻意不做第三个主循环**：
    "什么时候停、怎么收尾"的规矩只有一份，两处各写一份迟早分家。
    """
    args.input = "board"
    args.nostop = True              # 这个子命令的默认；显式 --max-steps 仍然优先
    if getattr(args, "headless", False):
        return cmd_run(args)
    return cmd_play(args)


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
    # 「一直跑」的无窗口路径也要能被干净停掉：不装处理函数的话 SIGTERM 是硬杀，
    # 既不收尾（source.close()）也不出汇总。装成 KeyboardInterrupt，`env.run` 里
    # 已经有对应的 except（它会收尾并返回已有统计）。
    import signal as _signal
    _prev = {}
    def _on_signal(signum, frame):          # pragma: no cover - 真信号路径
        raise KeyboardInterrupt
    for _sig in (_signal.SIGINT, _signal.SIGTERM):
        try:
            _prev[_sig] = _signal.signal(_sig, _on_signal)
        except (ValueError, OSError):
            pass

    if getattr(args, "input", None):
        from bullet_sim.ai import make_play_source

        try:
            play = make_play_source(
                args.input, env=env, auto=args.auto, model=args.model,
                seed=args.seed, switchable=False,
                board=(_board_source_kwargs(args) if args.input == "board" else None),
            )
        except Exception as exc:
            print(f"cannot start input mode {args.input!r}: {exc}", file=sys.stderr)
            env.close()
            return 2
        result = env.run(input_source=play.source, steps=_run_steps(args, spec),
                         seed=args.seed,
                         stop_on_done=not bool(getattr(args, "nostop", False)))
    else:
        ctrl = controller_from(_controller(args.controller, env.action_space, args.seed),
                               space=env.action_space, seed=args.seed)
        result = env.run(ctrl, steps=_run_steps(args, spec), seed=args.seed,
                         stop_on_done=not bool(getattr(args, "nostop", False)))
    env.close()
    for _sig, _h in _prev.items():
        try:
            _signal.signal(_sig, _h)
        except (ValueError, OSError):
            pass
    if args.json:
        payload = {k: v for k, v in result.items() if k != "final_info"}
        payload["final_bullet_count"] = result["bullet_count"]
        print(json.dumps(payload, indent=2))
    else:
        print(f"scenario        : {spec.name} (seed={spec.seed})")
        print(f"obstacles       : {', '.join(_spec_kinds(spec)) or '?'}"
              + ("  ← 板子在环里：只放这一种" if _is_board_mode(args) else "")
              + (f"  ← 轮播：每种 {spec.meta['cycle_seconds']:g}s，整轮 "
                 f"{spec.meta['cycle_period_s']:g}s" if spec.meta.get("cycle_seconds") else ""))
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


def _is_board_mode(args) -> bool:
    """板子在环里吗（`board-control` 或 `play --input board`）—— 决定场景类型数上限。"""
    return str(getattr(args, "input", "") or "").strip().lower() == "board"


def _spec_kinds(spec) -> list[str]:
    """场景里**实际**有哪几种障碍类型（从 patterns 推，不看 meta）。

    为什么以 patterns 为准：`--obstacle-type` 那条路（`ObstacleScenario.to_spec()`）
    写的是 `meta['obstacle_types']`，难度那条写的是 `meta['obstacle_kinds']` ——
    两条路的 meta 名字不一样，而"场上到底有哪几种"只有一个答案（patterns）。
    """
    from bullet_sim.scenarios.builder import kind_order

    return kind_order(getattr(spec, "patterns", []) or [])


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
    tgt = spec.meta.get("target_bullet_count")
    peak = spec.meta.get("measured_peak_bullets")
    if tgt is None and peak is None:
        print(f"  bullets      : 场景自带数量（不按难度标定）, "
              f"capacity {env.world.state.bullets.capacity}")
    else:
        print(f"  bullets      : ~{tgt if tgt is not None else '?'} target, "
              f"实测峰值 {peak if peak is not None else '?'}, "
              f"capacity {env.world.state.bullets.capacity}")
    kinds = ", ".join(_spec_kinds(spec)) or "?"
    print(f"  obstacles    : {kinds}")
    if _is_board_mode(args):
        why = ("板子在环里 → **只放这一种**（板子的 gap_avoid 只对墙+缺口有语义；"
               "场上多一种只会让画面更难读）")
        print(f"                 {why}")
        if spec.meta.get("kinds_override"):
            print("                 换类型用 --obstacle-type <一种>（如 small_obstacles）；"
                  "--level 只改难度（更快更窄），不会让类型变多")
        else:
            print("                 这一种由 --obstacle-type 指定；"
                  "参数用 --obstacle-count/--obstacle-speed/--gap-width 等覆盖")
    if spec.meta.get("cycle_seconds"):
        print(f"  cycle        : 每种独占 {spec.meta['cycle_seconds']:g}s，整轮 "
              f"{spec.meta['cycle_period_s']:g}s（同一时刻场上只有一种）")
    elif spec.meta.get("obstacle_stages"):
        print(f"  staged       : 开场只有第一种，之后每 "
              f"{spec.meta.get('stage_seconds', 0):g}s 多解锁一种"
              f"（{len(spec.meta['obstacle_stages'])} 种）")
    print(f"  collisions   : penalty event (penalty={spec.collision_penalty}, "
          f"mode={spec.collision_count_mode}) - episode continues")
    if env.world.terminate_on_collision:
        print("                 MODE B: --collision-terminates is ON, "
              "a collision ends the episode")
    print("  keys         : " + keys)
    print("                 TAB = switch MANUAL <-> AUTONOMOUS   H = HUD   "
          "ESC/Q = quit")
    print("=" * 74)


#: `play` 不给 `--duration` 时的默认场景时长（秒）。见 `cmd_play` 里的说明。
PLAY_DEFAULT_SECONDS = 1800.0


def cmd_play(args: argparse.Namespace) -> int:
    import time

    from bullet_sim.action.base import action_to_codec_input
    from bullet_sim.ai import make_play_source
    from bullet_sim.entities.player import P_SPEED
    from bullet_sim.render.base import make_renderer
    from bullet_sim.simulator.env import BulletHellEnv

    # 交互窗口默认只跑 30 秒（场景默认 duration）就自己关了 —— 对看窗口的人太短，
    # 而且 `--max-steps` **单独加长不了**：env 会在 spec.total_steps 处截断
    # （实测 `--max-steps 300000` 仍然只跑 3600 步）。所以给 play 一个更长的默认，
    # 按 ESC/Q 随时退。
    if args.duration is None and args.max_seconds is None:
        args.duration = PLAY_DEFAULT_SECONDS
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

    _state_log = None
    if getattr(args, "state_log", None):
        _state_log = open(args.state_log, "w", encoding="utf-8")

    mode = "manual" if args.interactive else args.input
    switchable = not args.no_switch
    try:
        board_kw = _board_source_kwargs(args) if mode == "board" else None
        play = make_play_source(
            mode,
            env=env,
            auto=args.auto,
            model=args.model,
            controller=_controller(args.controller, env.action_space, args.seed),
            seed=args.seed,
            switchable=switchable,
            board=board_kw,
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
            show_potential=getattr(args, "potential", False),
            interactive=False,
            input_source=play.source,
            controller=play.controller,
            # 标定参考物：相机拍屏幕时用它建场地坐标系
            fullscreen=args.fullscreen,
            show_calibration_marker=args.calib_marker,
            calibration_marker=(args.calib_marker_w, args.calib_marker_h),
            calibration_marker_color=parse_hex_color(args.calib_marker_color),
            wall_color=parse_hex_color(args.wall_color),
            show_fiducials=not args.no_fiducials,
            show_world_axes=not args.plain_field,
            show_field_border=not args.plain_field,
        )
    except Exception as exc:
        print(f"cannot open a window ({exc}).", file=sys.stderr)
        print("  headless instead:  python -m bullet_sim run "
              f"--input {mode} --steps {args.max_steps or 1200}", file=sys.stderr)
        env.close()
        return 2

    source = play.source
    obs, info = env.reset(seed=args.seed)
    try:
        source.open()
    except Exception as exc:
        # 输入源打不开（串口不在 / 被占 / UDP 端口占用）就**别开窗**、也别甩 traceback：
        # 说清楚怎么修，然后退出。以前这里直接崩，用户会以为是"窗口出不来"。
        print(f"输入源打不开，比赛场没起来：\n{exc}", file=sys.stderr)
        try:
            renderer.close()
        except Exception:
            pass
        env.close()
        return 2

    # ⚠ 开窗口之前先探一次（**放在输入源打开之后**：传输错误优先报，那更常见也更该先修）。
    #   卡在 `set_mode` 的 C 调用里时握着 GIL —— 同进程的看门狗线程等不到 GIL、信号也
    #   递不进去（实测发 SIGTERM 后进程状态一直是 R），只有子进程能被 timeout 杀掉。
    #   见 `_probe_display`。
    if not getattr(args, "headless", False):
        ok, why = _probe_display()
        if not ok:
            print(
                f"✗ 开不了窗口：{why}\n"
                "  窗口是仿真器当「平台」用的必需品（相机要拍它），但显示子系统现在不响应。\n"
                "  这不是程序算不出来 —— 实测同一个 pygame：offscreen 驱动 0.9s 就能开、\n"
                "  x11（WSLg）卡住，说明是 WSLg 那边的问题。按顺序试：\n"
                "    ① 清掉残留窗口/进程：pgrep -af bullet_sim，然后 kill（退不掉就 kill -9）\n"
                "    ② Windows 侧执行 `wsl --shutdown`，再重新打开 WSL（WSLg 卡住的正规解法）\n"
                "    ③ 先不折腾窗口：加 --headless（板子照旧驱动仿真，只是没有画面）\n"
                "    ④ 或者在 Windows 上直接跑（原生窗口，不经过 WSLg）",
                file=sys.stderr)
            env.close()
            return 4

    from bullet_sim.action import Action

    last_action = Action.zero()
    sim_per_frame = max(1, int(round(args.sim_speed / args.render_fps / spec.dt)))
    frame_dt = 1.0 / args.render_fps
    nostop = bool(getattr(args, "nostop", False))
    # 「不许自己停」= ① 不设回合步数上限（除非显式给了 --max-steps/--max-seconds，
    # 显式永远优先）② 不因 terminated/truncated 结束。见 README 的 board-control 一节。
    if args.max_steps:
        step_limit = args.max_steps
    elif nostop:
        step_limit = None
    else:
        step_limit = spec.total_steps
    if args.max_seconds is not None:
        limit_s = int(round(args.max_seconds / spec.dt))
        step_limit = limit_s if step_limit is None else min(step_limit, limit_s)
    steps_done = 0
    frame_index = 0
    terminated = truncated = False
    from bullet_sim.prediction.danger_field import compute_danger_field

    # ⚠ **必须自己装信号处理，而且要装在 pygame 初始化之后**：SDL2 会接管
    #   SIGINT/SIGTERM 并把它翻译成 SDL_QUIT 事件，而在 WSLg/某些驱动下那个事件
    #   到不了 `pygame.event.get()` —— 实测发 SIGINT 后进程状态一直是 R、8 秒都不退。
    #   最后一个 `signal.signal()` 生效，所以这里装在 renderer 之后就能赢过 SDL。
    import signal as _signal

    _stop: dict[str, Any] = {"flag": False, "n": 0}
    _last_link_print = 0.0

    def _on_signal(signum, frame):          # pragma: no cover - 真信号路径
        _stop["n"] = _stop.get("n", 0) + 1
        if _stop["n"] >= 2:
            # 连按两次 = 强制退出。卡在 C 调用（如 set_mode/event.get）里时，
            # 第一次信号的异常要等调用返回才递得进去 —— 那就再给一次机会直接走。
            import os as _os
            _os._exit(130)
        _stop["flag"] = True
        raise KeyboardInterrupt             # 让 sleep/阻塞调用立刻中断

    _prev_handlers = {}
    for _sig in (_signal.SIGINT, _signal.SIGTERM):
        try:
            _prev_handlers[_sig] = _signal.signal(_sig, _on_signal)
        except (ValueError, OSError):       # 非主线程等场合
            pass

    # 退出方式要在一开始就说清楚：`--nostop` 不设步数上限，出口只有这几个。
    if not args.quiet:
        print("  退出: 窗口里按 ESC，或关窗口；终端里 Ctrl-C。"
              "（--nostop 不设回合上限，所以自己不会停）", file=sys.stderr)
        if mode == "board":
            print("  等待板子信号 @MOVE n …（窗口标题栏不要最小化；"
                  "HUD 里 board 那行会显示收到多少行）", file=sys.stderr)

    try:
        while (_stop["flag"] is False
               and (step_limit is None or steps_done < step_limit)):
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
                if step_limit is not None and steps_done >= step_limit:
                    break
                obs, _r, terminated, truncated, info = env.step(raw)
                steps_done += 1
                if (terminated or truncated) and not nostop:
                    break
            if args.danger and frame_index % max(1, args.danger_every) == 0:
                try:
                    renderer.danger = compute_danger_field(
                        env.simulate_future(0, horizon=8, include_initial=True),
                        resolution=16.0,
                    )
                except Exception as exc:  # pragma: no cover - overlay must never break the loop
                    # 以前这里是静默吞掉：势图不显示时完全看不出原因（用户会以为"没改"）。
                    # 现在第一次失败就报出来（只报一次，避免刷屏）。
                    if not getattr(args, "_danger_warned", False):
                        args._danger_warned = True
                        print(f"⚠ 势图计算失败（--danger 不会显示）：{exc!r}", file=sys.stderr)
                    renderer.danger = None
            frame_index += 1
            if _state_log is not None:
                # 真值：角色 + 所有障碍（矩形带 half_w/half_h/rotation，圆带半径）
                _pl = env.world.player
                _obs = []
                _pool = getattr(getattr(env.world, "state", None), "bullets", None)
                if _pool is not None:
                    try:
                        _n = int(getattr(_pool, "count", len(_pool.alive)))
                    except Exception:
                        _n = 0
                    for _i in range(min(_n, int(getattr(_pool, "capacity", _n)))):
                        try:
                            if not bool(_pool.alive[_i]):
                                continue
                            _obs.append([float(_pool.x[_i]), float(_pool.y[_i]),
                                         float(_pool.half_w[_i]), float(_pool.half_h[_i]),
                                         float(getattr(_pool, "rotation", [0.0] * _n)[_i])])
                        except Exception:
                            continue
                _na = -1
                for _attr in ("n_actions", "board", "link", "_board", "_link", "adapter"):
                    try:
                        _obj = source if _attr == "n_actions" else getattr(source, _attr, None)
                        if _obj is None:
                            continue
                        if _attr == "n_actions":
                            _na = int(_obj)
                            break
                        _v = getattr(_obj, "n_actions", None)
                        if _v is not None:
                            _na = int(_v)
                            break
                    except Exception:
                        continue
                _state_log.write(json.dumps({
                    "frame": frame_index,
                    "board_actions": _na,
                    "t": steps_done * float(env.spec.dt) if hasattr(env, "spec") else steps_done,
                    "player": [float(_pl[0]), float(_pl[1]), float(_pl[4])],
                    # 仿真**实际执行**的动作（离散码）——用来区分"指令没到"和
                    # "指令到了但没执行"，这是判断延迟/断流的关键字段
                    "applied": [float(last_action.direction[0]) * float(last_action.magnitude),
                                float(last_action.direction[1]) * float(last_action.magnitude)]
                    if hasattr(last_action, "direction") else None,
                    "obstacles": _obs,
                }) + "\n")
                _state_log.flush()
            # ⚠ 窗口被关掉时 SDL 不一定投递 pygame.QUIT（WSLg 上实测过），
            #   而 `--nostop` 不设步数上限 —— 那就变成"窗口没了、进程还在跑"（卡死）。
            #   所以每一帧都判一次显示是否还活着，渲染失败也直接退。
            # 板子模式：每 2 秒往**终端**报一次链路状态。
            # 为什么打终端而不是画在屏幕上：屏幕上的文字会被相机拍进去，而视觉判据里
            # "亮块"正是找角色的判据 —— 那等于自己给视觉制造假目标（`show_text` 默认
            # 关掉就是这个道理）。终端里看得到，且不污染相机画面。
            if mode == "board" and not args.quiet:
                now_mono = time.monotonic()
                if now_mono - _last_link_print >= 2.0:
                    _last_link_print = now_mono
                    _fn = getattr(source, "hud_line", None)
                    _line = _fn() if callable(_fn) else None
                    if _line:
                        print(f"  [{_line.strip()}]", flush=True)

            alive = getattr(renderer, "alive", None)
            if callable(alive) and not alive():
                print("窗口已关闭（显示子系统已退出），收工", file=sys.stderr)
                break
            try:
                renderer.render(env.world)
            except Exception as exc:
                print(f"渲染失败（窗口多半已经关掉）：{exc}", file=sys.stderr)
                break
            if renderer.closed or ((terminated or truncated) and not nostop):
                break
            time.sleep(frame_dt)
    except KeyboardInterrupt:
        pass
    finally:
        for _sig, _h in _prev_handlers.items():
            try:
                _signal.signal(_sig, _h)
            except (ValueError, OSError):
                pass
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
    fn = getattr(source, "hud_line", None)
    link = fn() if callable(fn) else None
    if link:
        print(f"link status  : {link}")
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
    "board-control": cmd_board_control,
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

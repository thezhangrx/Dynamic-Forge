"""开平台 + 录制 + 出对账报告，一条命令。

    python vision_capture_rig.py --out data/vision/run1              # 用摄像头拍
    python vision_capture_rig.py --out data/vision/run1 --no-camera  # 你自己录屏

平台用**默认设置**（默认配色、默认生成间隔），和平时跑一样；
录完直接打印 `vision_reconcile.py` 的报告。

**角色可以用键盘开**（方向键 / WASD；不按键就等于平台默认的静止）。
这不是为了好玩：墙是**周期性**生成的，只看缺口的话画面每隔一个生成
周期就长得一样，时间对齐会落到隔壁周期上；角色是非周期的，动起来
才能把时间偏移钉死。全程不动的录也能出报告，但会带一条歧义告警。

两个实现要点
------------
* **摄像头在后台线程里抓帧**。``cap.read()`` 在这个摄像头上一帧要 ~120 ms，
  放在主循环里会把渲染拖到 8 fps —— 平台看起来"卡卡的"。
  放进线程，主循环只管步进+渲染（~60 fps），有新的帧就取走存一张。
* **平台可见性只在失败时出声**。相机拍到别的窗口时报告毫无意义，
  所以在第一帧上检查一次：不像平台就**一行话说清并退出**，像就一声不吭。
"""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import subprocess
import sys
import time

_ROOT = pathlib.Path(__file__).resolve().parent
_CORE = _ROOT / "core"
if str(_CORE) not in sys.path:
    sys.path.insert(0, str(_CORE))

from bullet_sim.action.types import Action                      # noqa: E402
from bullet_sim.simulator.truth import truth_gaps, truth_player  # noqa: E402
from harness.loop import FIELD, SIM_DT, WIN, build_env           # noqa: E402
from standard.fiducial import FiducialLayout                     # noqa: E402
from vision.capture import Camera, looks_like_platform           # noqa: E402

FIDUCIAL_LAYOUT = FiducialLayout(field_w=FIELD[0], field_h=FIELD[1],
                                 marker_side=48.0, margin=8.0)


def auto_action(px: float, py: float, sim_time: float):
    """**确定性、非周期**的巡航动作 —— 让时间对齐有非周期信息可用。

    为什么需要它
    ------------
    墙是每隔 1.6s 生成一堵的，于是**只看缺口**时画面每个生成周期就长得
    一样，内容对齐会出现等间距的多个极小值（歧义比 ≈1，工具会如实告警）。
    角色是**非周期**的，把它计进评分就能把隔壁周期拉开 —— 但前提是角色真的动。

    为什么不用"让人按键"
    -------------------
    两个理由：

    1. **键盘焦点**。平台窗口拿不到焦点时按键根本不进窗口（WSLg 上实测过），
       现象是"车完全不动"，而这时录出来的一整段素材都带歧义告警。自动巡航
       不依赖任何输入设备。
    2. **可复现**。轨迹是 ``sim_time`` 的确定函数，所以同一个 ``--seed`` /
       ``--seconds`` 跑两遍，角色的每一帧位置都一致 —— 出问题能复跑对齐。

    轨迹：两个**不可通约**的频率（周期约 8.6s 与 5.6s），并且都与墙的
    1.6s 生成周期不可通约，所以不会跟着墙一起重复。振幅留在场地内部，
    不会贴着边界跑。
    """
    from bullet_sim.action.types import Action

    tx = 320.0 + 140.0 * math.sin(0.73 * sim_time)
    ty = 240.0 + 90.0 * math.sin(1.13 * sim_time)
    dx, dy = tx - float(px), ty - float(py)
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return Action.zero()
    return Action.from_vector((dx / n, dy / n))

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="开平台 + 录制 + 出对账报告")
    ap.add_argument("--out", type=pathlib.Path, required=True)
    ap.add_argument("--seconds", type=float, default=25.0)
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--warmup", type=int, default=25)
    ap.add_argument("--fullscreen", action="store_true",
                    help="平台窗口全屏（相机拍屏幕时建议开）")
    ap.add_argument("--window-pos", default=None, metavar="X,Y",
                    help="把窗口钉到指定屏幕坐标（多屏时用）")
    ap.add_argument("--scale", type=float, default=1.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--gap-width", type=float, default=60.0)
    ap.add_argument("--gap-position", type=float, default=0.30,
                    help="缺口位置 0~1。别用 0.5：角色恰好生在中线，原地不动就能活")
    ap.add_argument("--no-camera", action="store_true",
                    help="不开摄像头：只开平台 + 记状态日志。"
                         "给\"在 Windows 自己录像\"用 —— 录完把视频和这份日志一起给我")
    ap.add_argument("--state-hz", type=float, default=30.0,
                    help="--no-camera 时状态日志的写入频率")
    ap.add_argument("--drive", choices=("auto", "keys", "none"), default="auto",
                    help="角色怎么动。auto=确定性自动巡航（默认，可复现、不依赖键盘焦点）；"
                         "keys=方向键/WASD 手操；none=不动（时间对齐会有歧义告警）")
    ap.add_argument("--no-reconcile", action="store_true", help="录完不自动对账")
    args = ap.parse_args(argv)

    import cv2
    import pygame
    if args.window_pos:
        # SDL 只在**创建窗口之前**读这个环境变量
        __import__("os").environ["SDL_VIDEO_WINDOW_POS"] = str(args.window_pos)
    from bullet_sim.action.manual import ManualInputSource
    from bullet_sim.render.pygame_view import PygameRenderer, pygame_key_names

    args.out.mkdir(parents=True, exist_ok=True)

    # -- 平台：默认设置 ---------------------------------------------------
    env = build_env(args.seed, args.gap_width, args.seconds * 4, 40.0,
                    args.gap_position, "static")
    env.reset(seed=args.seed)
    win = (int(WIN[0] * args.scale), int(WIN[1] * args.scale))
    # 角色动作见 --drive。这里只在 keys 模式挂输入源（auto/none 下按键无意义，
    # 挂上去只会让人以为"按了应该动"）。ESC/Q 关窗口由渲染器自己处理，与它无关。
    src = ManualInputSource()
    drive_banner = {
        "auto": "角色**自动巡航**（确定性轨迹，可复现，不需要键盘）",
        "keys": "角色**手操**：方向键 / WASD（先把窗口点一下给它键盘焦点）",
        "none": "角色**不动** —— 时间对齐会有歧义告警（不建议）",
    }[args.drive]
    r = PygameRenderer(mode="human", window_size=win, show_hitboxes=False,
                       show_world_axes=True, fullscreen=bool(args.fullscreen),
                       show_fiducials=True,
                       input_source=src if args.drive == "keys" else None,
                       fiducial_side=FIDUCIAL_LAYOUT.marker_side)
    if args.no_camera:
        cam = None
        print(f"平台 start: {args.seconds:.0f}s，状态日志 -> {args.out}/truth.jsonl\n"
              f"  {drive_banner}\n"
              f"  **现在开始录你的屏幕**；录完把视频和这份日志一起给我。",
              flush=True)
    else:
        cam = Camera(args.device, args.width, args.height, args.warmup)
        cam.start()
        print(f"录制 start: {args.seconds:.0f}s -> {args.out}  "
              f"({args.width}x{args.height} /dev/video{args.device})\n"
              f"  {drive_banner}", flush=True)

    def grab_camera(timeout: float = 3.0):
        """抓一张（用于参考物）。返回 bgr 或 None（用户关了窗口）。"""
        t_end = time.monotonic() + timeout
        while time.monotonic() < t_end:
            r.render(env.world)
            for e in pygame.event.get():
                if e.type == pygame.QUIT:
                    return None
            item = cam.take()
            if item is not None:
                return item[1]
            time.sleep(0.01)
        return None

    # 标定标记（四角 ArUco）**全程可见**：它们只占场地面积约 5%，
    # 不像原来那块洋红矩形（盖住 90% 场地，开着就没法测墙和缺口）。
    # 于是对账时随便挑一帧就能标定，而且每一帧都能反解一次当哨兵。
    (args.out / "fiducials.json").write_text(
        json.dumps(FIDUCIAL_LAYOUT.to_dict(), indent=2, ensure_ascii=False),
        encoding="utf-8")

    # -- 主循环：步进 + 渲染（~60fps）；有新的相机帧就存一张 --------------
    t0 = time.monotonic()
    deadline = t0 + args.seconds
    stamps: list[float] = []
    truth_lines: list[str] = []
    checked = False
    last_state = 0.0

    def _log_state(now: float) -> None:
        px, py, pr = truth_player(env.world)
        gaps = truth_gaps(env.world) or ()
        truth_lines.append(json.dumps({
            "stamp": float(env.world.state.timestamp), "wall": now - t0,
            "player": {"x": px, "y": py, "radius": pr},
            "gaps": [{"cx": g[0], "width": g[1], "y": g[2]} for g in gaps],
        }, ensure_ascii=False))

    #: pygame 键码 -> 设备无关键名。渲染器内部那张表是**唯一**的映射出处，
    #: 这里复用它，免得两处各写一份、改一处漏一处。
    keymap = pygame_key_names()

    def sync_keys() -> bool | None:
        """把"此刻按住的键"同步进 ``src``；返回窗口是否有键盘焦点。

        **为什么不只靠 KEYDOWN/KEYUP 事件**：这个循环自己也取事件
        （为了看 QUIT/ESC），而 ``pygame.event.get()`` 是**谁先取谁拿到**、
        取走就没了 —— 渲染器的 ``_pump_events()`` 和这里抢同一个队列，
        事件驱动在这种"两个消费者"的结构里天生脆。

        ``pygame.key.get_pressed()`` 读的是**设备当前状态**，不经过事件队列，
        所以：不会因为队列被谁先取走而丢键，也不会漏掉"窗口还没拿到焦点
        就已经按住"的那个键（那时 KEYDOWN 早就丢了，只靠事件会永远松开）。

        返回 ``None`` 表示视频系统还没初始化（渲染器第一次 ``render()``
        之前）—— 那时读键会抛 "video system not initialized"，直接跳过，
        而且**不能**把它当成"没焦点"去告警，否则第一帧就误报一次。
        """
        if not pygame.display.get_init():
            return None
        pressed = pygame.key.get_pressed()
        held = {name for code, name in keymap.items()
                if 0 <= code < len(pressed) and pressed[code]}
        for name in held - src.pressed_keys:
            src.press(name)
        for name in src.pressed_keys - held:
            src.release(name)
        return bool(pygame.key.get_focused())

    focused_warned = False
    while time.monotonic() < deadline:
        now = time.monotonic()
        # 仿真按**墙上时钟**推进：摄像头是真的、时间是真的，仿真跟着走，
        # 屏幕上墙的速度才等于标称的 40 单位/秒。
        want = now - t0
        sim_now = float(env.world.state.timestamp)
        # 角色动作来自键盘（不按键 = 零动作）。**每帧同步按键状态**，
        # 而不是等事件 —— 见 sync_keys 的说明。
        # 角色动作：auto=确定性巡航 / keys=手操 / none=不动。
        # 注意 keys 模式下**不拿焦点当开关**：焦点只用来决定要不要提醒；
        # 万一某个平台/驱动把 get_focused() 报错（WSLg 上见过不可靠的窗口状态），
        # 拿它当开关会直接把输入全部掐死 —— 那是把一个"不动"换成另一个"不动"。
        if args.drive == "auto":
            px, py, _pr = truth_player(env.world)
            action = auto_action(px, py, float(env.world.state.timestamp))
        elif args.drive == "none":
            action = Action.zero()
        else:
            focused = sync_keys()
            if focused:
                focused_warned = False
            elif focused is False and not focused_warned:
                # 键盘焦点在终端/别的窗口上时，按键**根本不会进这个窗口**，
                # 现象就是"车完全不动"。这不是 bug，必须一次性说清楚。
                focused_warned = True
                print("⚠ 平台窗口现在没有键盘焦点 —— 车不会动。"
                      "用鼠标点一下窗口（或 Alt-Tab 切过去），"
                      "或者改用 --drive auto（自动巡航，不需要键盘）。",
                      file=sys.stderr)
            action = src.current_action()
        for _ in range(max(0, int((want - sim_now) / SIM_DT))):
            env.world.step(action)
        r.render(env.world)
        # 不再自己 pygame.event.get()：渲染器已经在 _pump_events() 里处理了
        # QUIT/ESC，这里只看它的状态位，避免两个消费者抢同一个队列。
        if r.closed:
            print("（窗口被关闭）")
            deadline = 0.0
            break

        if cam is None:
            # 无相机：只按 state_hz 记状态，不存图
            if now - last_state >= 1.0 / max(0.1, args.state_hz):
                last_state = now
                _log_state(now)
            continue
        item = cam.take()
        if item is None:
            continue
        t_cap, bgr = item

        # 第一帧检查一次"相机是不是在拍平台"。只在失败时出声。
        if not checked:
            checked = True
            ok, why = looks_like_platform(bgr)
            if not ok:
                cam.close()
                cv2.imwrite(str(args.out / "probe.jpg"), bgr)
                print(f"\n相机拍到的不是平台（{why}）—— 这次录制没有意义，已停止。\n"
                      f"  这一帧存在 {args.out}/probe.jpg，看一眼就知道拍到了什么。\n"
                      "  平台窗口要铺满相机画面，并且全程不能被别的窗口挡住。",
                      file=sys.stderr)
                return 2

        i = len(stamps)
        cv2.imwrite(str(args.out / f"{i:04d}.jpg"), bgr)
        stamps.append(t_cap - t0)
        px, py, pr = truth_player(env.world)
        gaps = truth_gaps(env.world) or ()
        truth_lines.append(json.dumps({
            "stamp": float(env.world.state.timestamp),
            "wall": t_cap - t0,
            "player": {"x": px, "y": py, "radius": pr},
            "gaps": [{"cx": g[0], "width": g[1], "y": g[2]} for g in gaps],
        }, ensure_ascii=False))

    if cam is not None:
        cam.close()
    pygame.quit()

    if cam is None:
        (args.out / "truth.jsonl").write_text("\n".join(truth_lines) + "\n",
                                              encoding="utf-8")
        print(f"平台 done : 状态日志 {len(truth_lines)} 条")
        return 0
    if not stamps:
        print("一帧都没录到", file=sys.stderr)
        return 2
    (args.out / "truth.jsonl").write_text("\n".join(truth_lines) + "\n", encoding="utf-8")
    (args.out / "stamps.json").write_text(json.dumps(stamps), encoding="utf-8")
    (args.out / "frames.json").write_text(
        json.dumps([f"{i:04d}.jpg" for i in range(len(stamps))]), encoding="utf-8")
    print(f"录制 done : {len(stamps)} 帧 / {args.seconds:.0f}s "
          f"({len(stamps) / max(1e-6, args.seconds):.1f} fps)")

    if args.no_reconcile:
        return 0

    # -- 对账：直接调 vision_reconcile.py，它的输出就是报告 ---------------
    # 标定走**四角 ArUco 标记**（fiducials.json 已经在 out/ 里），
    # 所以不再传 --marker-image/--marker（那是已删掉的洋红矩形方案，留着会直接报错）。
    # --estimate-offset 在这里**成立**：stamps.json 与 truth.jsonl 是同一次循环里
    # 成对写出的，天然锁步，可以估出相机采集时延。
    print()
    return subprocess.call([sys.executable, str(_ROOT / "vision_reconcile.py"),
                            "--images", str(args.out),
                            "--log", str(args.out / "truth.jsonl"),
                            "--stamps", str(args.out / "stamps.json"),
                            "--estimate-offset"], cwd=str(_ROOT))

if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\n已中断")
        raise SystemExit(130) from None


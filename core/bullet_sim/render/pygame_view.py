"""pygame renderer (optional dependency).

Decoupled display loop
----------------------
The renderer never advances the simulation.  A caller decides how many
simulation steps to run between frames, so the *same* scene can be watched in
real time or stepped far slower/faster without changing physics.

Overlays
--------
* player (with hitbox radius), target zone, bullets coloured by type
* optional future danger field and predicted bullet trajectories

**No text is drawn on the window by default** (``show_text=False``): the HUD,
the world-coordinate tick labels, the control-help panel, the mode banner and
the collision caption are all suppressed, so the popup shows the scene only.
Pass ``show_text=True`` to bring them back.  The text itself is still produced
by :mod:`bullet_sim.render.overlays`, which remains the single source of truth
for the HUD content (and is what the tests assert on).

Press ``ESC`` (or close the window) to request shutdown; ``q`` also quits and
``h`` toggles the HUD (visible only with ``show_text=True``).  When
``mode='rgb_array'`` nothing is shown on screen.
"""

from __future__ import annotations

import os
import time
from typing import Any

import numpy as np

# Keep pygame's banner off stdout: the platform's stdout is data (JSON/CSV).
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")

import pygame  # noqa: E402

from bullet_sim.render.viewport import Viewport
from bullet_sim.render.overlays import (
    FrameCounter,
    color_for_type,
    danger_cells,
    hud_lines,
    is_collision_flash,
    prediction_polylines,
)


#: pygame key constant -> device-neutral key name understood by
#: :class:`bullet_sim.action.manual.ManualInputSource`.  This table is the ONLY
#: place where pygame and the input layer meet, which is what keeps the manual
#: input module free of any rendering dependency.
def pygame_key_names() -> "dict[int, str]":
    return {
        pygame.K_UP: "up",
        pygame.K_DOWN: "down",
        pygame.K_LEFT: "left",
        pygame.K_RIGHT: "right",
        pygame.K_w: "w",
        pygame.K_s: "s",
        pygame.K_a: "a",
        pygame.K_d: "d",
        pygame.K_SPACE: "space",
        pygame.K_0: "0",
        pygame.K_LSHIFT: "shift",
        pygame.K_RSHIFT: "shift",
        pygame.K_LCTRL: "ctrl",
        pygame.K_RCTRL: "ctrl",
    }


def key_name_for(key: int) -> str | None:
    """Translate a pygame key constant to a device-neutral key name."""
    return pygame_key_names().get(int(key))


#: 墙（矩形障碍）统一画成这个颜色，**不按 type_id 走子弹调色板**。
#: 理由：调色板会让**同一堵** wall_with_gap 的两段拿到不同颜色
#: （实测录像里一段白一段橙），按颜色找墙的 detect_walls 就只看得见一段，
#: 缺口无从谈起。这个值就是 wall_with_gap 测试与文档里一直假定的橙色。
#:
#: **2026-10-18：提高对比度（为了在识别侧只加一个"颜色门"）**
#:   旧值 (240,160,48)：相机里 r−b 只有 60~118，和角色（偏粉，r−b≈57）挨得近，
#:   而亮度判据又会被网格线/外框骗（空场地 100% 的帧报出两道假墙）。
#:   新值 (255,120,0)：**色相还是橙**（RTL 的"暖色"判据不受影响），但 r−b 拉到
#:   255（相机里实测 ~150+），与所有中性物体（网格线/边框/标记/角色）差一个量级。
#:   于是识别侧只要 `r - b >= 60` 这一道门，不必再叠亮度/运动/结构门。
WALL_COLOR = (255, 120, 0)


class PygameRenderer:
    name = "pygame"

    def __init__(
        self,
        mode: str = "human",
        *,
        scale: float = 1.0,
        show_hud: bool = True,
        show_text: bool = False,
        show_prediction: bool = False,
        show_danger: bool = False,
        show_potential: bool = False,
        field_bg: tuple = (34, 40, 55),
        potential_every: int = 30,
        show_target: bool = False,
        predictor: Any = None,
        prediction_horizon: int = 90,
        danger: Any = None,
        background: tuple[int, int, int] = (12, 14, 22),
        interactive: bool = False,
        input_source: Any = None,
        controller: Any = None,
        switch_key: int | None = None,
        help_seconds: float = 8.0,
        max_prediction_lines: int = 200,
        window_size: tuple[int, int] | None = None,
        resizable: bool = True,
        fullscreen: bool = False,
        show_hitboxes: bool = True,
        show_world_axes: bool = True,
        show_calibration_marker: bool = False,
        calibration_marker: tuple[float, float] = (0.90, 0.90),
        calibration_marker_color: tuple[int, int, int] = (128, 0, 128),
        wall_color: tuple[int, int, int] = WALL_COLOR,
        show_fiducials: bool = True,
        fiducial_side: float = 28.0,   # 28 场地单位：够定边界，又不会挡住宅的墙/缺口
        show_field_border: bool = True,
        window_timeout_s: float = 20.0,
    ) -> None:
        self.mode = mode
        self.scale = float(scale)
        self.show_hud = bool(show_hud)
        #: Master switch for every piece of text drawn on the window.  Off by
        #: default: the popup is meant to show the scene, not labels.
        self.show_text = bool(show_text)
        self.show_prediction = bool(show_prediction)
        self.show_danger = bool(show_danger)
        #: **决策势场**（移植自 core/cpu/field_demo.py）：分层等值带 + 膨胀障碍 + 目标点。
        #: 与 `show_danger` 不同：那个是"未来子弹威胁"，这个是"朝缺口走的代价场"。
        self.show_potential = bool(show_potential)
        #: 场地底色。**相机对着屏幕时，底色也是视觉输入的一部分**：深蓝底会把
        #: 墙/角色的对比压小（相机端还有增益/偏置），实测改成纯黑后板子的
        #: 识别明显更稳。默认值保持平台原样，只有"给板子看"的演示才覆盖。
        self.field_bg = tuple(field_bg)
        #: 势场**每 N 帧**重算一次（实测每帧重算是 14 帧/秒，把整个仿真拖慢 4 倍 ——
        #: 用户看到的就是"墙怎么变慢了"。势场是慢变量，没必要每帧算）。
        self.potential_every = max(1, int(potential_every))
        self._pot_frame = -1
        self._pot_ov = None
        self._pot_surf = None
        self._field_mem = None
        self._field_cfg = None
        self._gap_mem = None
        #: Draw the target zone marker.  Off by default: in an obstacle field the
        #: green ring reads as "something to reach / protect" and is mistaken for
        #: an obstacle, while it neither collides nor affects the reward unless a
        #: scenario explicitly configures ``target_bonus``.
        self.show_target = bool(show_target)
        self.predictor = predictor
        self.prediction_horizon = int(prediction_horizon)
        self.danger = danger
        self.background = background
        self.interactive = bool(interactive)
        #: When set, keyboard events are forwarded to it as press/release; the
        #: renderer never produces an Action itself.
        self.input_source = input_source
        #: Object whose ``spec()`` describes the active controller (HUD line).
        self.controller = controller
        self.switch_key = pygame.K_TAB if switch_key is None else switch_key
        #: How long the big control-help panel stays on screen at start-up.
        self.help_seconds = float(help_seconds)
        self.max_prediction_lines = int(max_prediction_lines)

        self.frames = FrameCounter()
        self._initialized = False
        self._screen = None
        self._font = None
        self._size = (0, 0)
        self.closed = False
        #: 开窗最多等多久（秒）。见 `_set_mode_with_watchdog`。
        self.window_timeout_s = float(window_timeout_s)
        self.pending_action: Any = None
        self.started_at = time.monotonic()
        self.mode_banner = ""
        self.mode_banner_until = 0.0
        #: how many frames the red collision flash stays on screen
        self.collision_flash_frames = 12
        #: All world<->screen conversion goes through this one object, so a
        #: window resize changes the observation scale and nothing else.
        self._window_size = window_size
        self.resizable = bool(resizable)
        #: 全屏。**调试时建议开**：窗口化时"平台窗口占屏幕一小块"会给视觉侧带来
        #: 一堆与算法无关的麻烦（窗口外的桌面内容会被当成目标、"场地在哪"要靠
        #: 标定去猜）。全屏之后场地就是整个屏幕，路径短得多。
        #: 只对 ``mode="human"`` 有效；``rgb_array`` 是离屏的，不存在全屏。
        self.fullscreen = bool(fullscreen)
        self.show_hitboxes = bool(show_hitboxes)
        self.show_world_axes = bool(show_world_axes)
        #: 标定参考物：**画在场地左下角**的一块纯白实心矩形，边长按场地比例给。
        #: 相机拍屏幕时用它建立场地坐标系（见 core/vision/calibration.py）。
        #: 默认关闭，免得污染正常回放。
        self.show_calibration_marker = bool(show_calibration_marker)
        self.calibration_marker = (float(calibration_marker[0]),
                                   float(calibration_marker[1]))
        #: 参考物的**颜色**。默认洋红 (255,0,255)：它是整个场景里唯一一个
        #: "G 远小于 R 和 B"的饱和色，而相机拍屏幕时最麻烦的**灯光反光**是
        #: 无彩色（R≈G≈B），于是判据 ``2G < R and 2G < B`` 天然免疫反光。
        #: 白色参考物则会被反光淹没 —— 实测一帧里反光块的面积是白块的 5.5 倍。
        self.calibration_marker_color = (int(calibration_marker_color[0]),
                                         int(calibration_marker_color[1]),
                                         int(calibration_marker_color[2]))
        #: 四角标定标记（ArUco）。**默认开**：它们占场地面积约 4%，
        #: 可以全程可见，于是每一帧都能反解单次标定（视觉侧据此判断"标定还成立吗"）。
        #: 位图与布局在 ``standard.fiducial``（平台画、视觉认，共享契约）。
        self.show_fiducials = bool(show_fiducials)
        self.fiducial_side = float(fiducial_side)
        #: 场地外框那 1 像素的线。相机拍屏幕时它是一条**横贯画面的长线**，会被
        #: 视觉判成矩形（实测：几何帧里多出一条贴底边的"墙"），板子在环里时关掉。
        self.show_field_border = bool(show_field_border)
        #: 墙（矩形障碍）的颜色。**和视觉链路的契约**，见 ``WALL_COLOR``。
        self.wall_color = (int(wall_color[0]), int(wall_color[1]), int(wall_color[2]))
        self.viewport: Viewport | None = None

    # ------------------------------------------------------------------
    def calibration_marker_world_rect(self, world: Any) -> tuple[float, float, float, float]:
        """参考物在**场地世界坐标**里的矩形 ``(x0, y0, x1, y1)``。

        原点就是左下角 ``(0, 0)`` —— 这样参考物的世界四角恰好是
        ``(0,0) (w,0) (w,h) (0,h)``，与 ``vision.calibration.Marker.world_corners()``
        的顺序（左下 → 右下 → 右上 → 左上）一致，标定时不需要再做任何换算。
        """
        fw = float(world.state.env.field_w)
        fh = float(world.state.env.field_h)
        return 0.0, 0.0, self.calibration_marker[0] * fw, self.calibration_marker[1] * fh

    # ------------------------------------------------------------------
    def _ensure_init(self, world: Any) -> None:
        if self._initialized:
            return
        # ⚠ 整个初始化（display.init / set_mode / 字体）都可能卡在 X 里 ——
        #   所以看门狗要罩住**整段**，不能只罩 set_mode。
        _done = self._start_window_watchdog()
        try:
            self._ensure_init_body(world)
        finally:
            _done.set()

    def _ensure_init_body(self, world: Any) -> None:
        # ⚠ 只初始化**要用的**子系统：`pygame.init()` 会连 **mixer（音频）**一起初始化，
        #   而跑这个仿真根本不需要声音 —— 在没有声卡的机器/容器里它只会刷一屏 ALSA 报错
        #   （实测把真正的失败信息埋掉了），还多一个卡住的机会。
        pygame.display.init()
        pygame.font.init()
        pygame.display.set_caption(f"bullet_sim - {world.spec.name}")
        if self._window_size is not None:
            self._size = (max(64, int(self._window_size[0])), max(64, int(self._window_size[1])))
        else:
            w = int(round(world.state.env.field_w * self.scale))
            h = int(round(world.state.env.field_h * self.scale))
            self._size = (max(64, w), max(64, h))
        self.viewport = Viewport(
            world_w=float(world.state.env.field_w),
            world_h=float(world.state.env.field_h),
            screen_w=self._size[0],
            screen_h=self._size[1],
        )
        if self.mode == "rgb_array":
            self._screen = pygame.Surface(self._size)
        else:
            flags = 0
            if self.fullscreen:
                # 用桌面的实际分辨率，而不是 field*scale —— 否则全屏会拉伸，
                # 场地被拉变形，视觉侧量出来的宽高比就不对了。
                info = pygame.display.Info()
                if info.current_w > 0 and info.current_h > 0:
                    self._size = (int(info.current_w), int(info.current_h))
                self.viewport = Viewport(
                    world_w=float(world.state.env.field_w),
                    world_h=float(world.state.env.field_h),
                    screen_w=self._size[0], screen_h=self._size[1])
                flags = pygame.FULLSCREEN
            elif self.resizable:
                flags = pygame.RESIZABLE
            self._screen = pygame.display.set_mode(self._size, flags)
        try:
            self._font = pygame.font.SysFont("monospace", 13)
        except Exception:  # pragma: no cover - font backend dependent
            self._font = pygame.font.Font(None, 15)
        self._initialized = True

    # ------------------------------------------------------------------
    def _start_window_watchdog(self) -> Any:
        """起一个守望线程：初始化卡住就说清怎么办，并让进程真的退出。

        为什么需要：`pygame.display.init()/set_mode()` 是 C 调用，卡在 X/WSLg 里时
        Python 的异常递不进去（**实测：发信号也进不来**，进程状态一直是 R），用户看到
        的就是"弹窗出不来、Ctrl-C 也没用"。守望线程到点直接打印诊断并 `os._exit`。
        """
        import os as _os
        import threading as _threading

        done = _threading.Event()

        def _watchdog() -> None:
            if done.wait(self.window_timeout_s):
                return
            import sys as _sys
            print(
                f"✗ 等了 {self.window_timeout_s:.0f} 秒还没能把窗口初始化出来 —— "
                "显示子系统卡住了\n"
                "  这不是程序算不出来，是 X/WSLg 那边没响应。按顺序试：\n"
                "    ① 关掉所有残留的窗口/进程：pgrep -af bullet_sim，然后 kill\n"
                "    ② Windows 侧执行 `wsl --shutdown`，再重新打开 WSL\n"
                "       （WSLg 卡住的正规解法；实测 offscreen 驱动 0.9s 就能开，只有 x11 卡）\n"
                "    ③ 先不折腾窗口：加 --headless（无窗口跑）\n"
                "  注意：卡在 C 调用里时 Ctrl-C 可能也进不来，退不掉就 `kill -9`。",
                file=_sys.stderr, flush=True,
            )
            _os._exit(4)

        th = _threading.Thread(target=_watchdog, name="window-watchdog", daemon=True)
        th.start()
        return done

    def render(self, world: Any) -> Any:
        self._ensure_init(world)
        self._pump_events()
        if self.closed:
            return None
        fps = self.frames.tick()
        screen = self._screen
        screen.fill(self.background)

        vp = self.viewport
        assert vp is not None
        vp.resize(*self._size)          # keep in sync even without an event
        fh = float(world.state.env.field_h)

        # --- world frame / letterbox -----------------------------------
        x0, y0 = vp.world_to_screen(0.0, 0.0)
        x1, y1 = vp.world_to_screen(world.state.env.field_w, fh)
        field_rect = pygame.Rect(
            int(min(x0, x1)), int(min(y0, y1)), int(abs(x1 - x0)), int(abs(y1 - y0))
        )
        pygame.draw.rect(screen, self.field_bg, field_rect)
        if self.show_field_border:
            pygame.draw.rect(screen, (60, 70, 95), field_rect, 1)
        if self.show_world_axes:
            self._draw_world_axes(screen, world, vp)

        # 标定参考物**最后画**（在障碍物和玩家之后，见 render() 末尾），
        # 这样它永远不会被墙压住 —— 实测障碍物画在它上面时，一条墙线横穿参考物，
        # 把它切成上下两块，连通域检测直接失败。参考物是标定用的，必须完整可见。

        # Collision feedback: a short red edge flash, never a game-over screen.
        if is_collision_flash(world, frames=self.collision_flash_frames):
            flash = pygame.Surface(self._size, pygame.SRCALPHA)
            pygame.draw.rect(flash, (255, 60, 60, 70), flash.get_rect(),
                             max(6, int(10 * self.scale)))
            screen.blit(flash, (0, 0))
            if self.show_text:
                label = self._font.render("COLLISION  (penalty, episode continues)",
                                          True, (255, 220, 220))
                screen.blit(label, (8, self._size[1] - label.get_height() - 8))

        # --- target zone (off by default, see show_target) --------------
        if self.show_target:
            t = world.state.target
            if t.shape == "rect":
                pts = vp.rect_to_screen(t.x, t.y, t.half_w, t.half_h)
                pygame.draw.polygon(screen, (90, 200, 140), pts, 2)
            else:
                pygame.draw.circle(
                    screen, (90, 200, 140), vp.world_to_screen(t.x, t.y),
                    max(2, int(vp.len_to_screen(t.radius))), 2,
                )

        # --- 板子决策的目标点（`@TGT`）：画成准星 ---
        # 开了决策势场时**不画**这个：`field_demo.draw_marks` 已经画了场的瞄准点，
        # 两个准星各自算、会互相打架（用户看到的就是"目标点乱跳"）。
        _skip_aim = getattr(self, "show_potential", False)
        # 用户要"看得见决策打的是哪个缺口"。这条只依赖输入源给的目标点，
        # 不改任何决策/物理逻辑。
        for _cand in (() if _skip_aim else (self.input_source,
                                            getattr(self.input_source, "adapter", None))):
            _fn = getattr(_cand, "debug_target", None)
            if not callable(_fn):
                continue
            try:
                _tgt = _fn()
            except Exception:
                _tgt = None
            if _tgt is not None:
                sx, sy = vp.world_to_screen(float(_tgt[0]), float(_tgt[1]))
                pygame.draw.circle(screen, (255, 220, 60), (int(sx), int(sy)), 9, 2)
                pygame.draw.line(screen, (255, 220, 60), (int(sx) - 13, int(sy)),
                                 (int(sx) + 13, int(sy)), 1)
                pygame.draw.line(screen, (255, 220, 60), (int(sx), int(sy) - 13),
                                 (int(sx), int(sy) + 13), 1)
            break

        if self.show_potential:
            try:
                self._draw_potential(screen, world)
            except Exception as exc:      # 可视化绝不能把主循环带崩
                if not getattr(self, "_pot_warned", False):
                    self._pot_warned = True
                    print(f"⚠ 势场绘制失败：{exc!r}", file=sys.stderr)

        if self.show_danger and self.danger is not None:
            self._draw_danger(screen, world)

        if self.show_prediction and self.predictor is not None:
            self._draw_prediction(screen, world)

        # --- dynamic obstacles (circle or rotated rectangle) ------------
        self._draw_obstacles(screen, world, vp)

        # --- player: visible body + real hitbox -------------------------
        self._draw_player(screen, world, vp)

        # --- 四角标定标记（ArUco）---------------------------------------
        # 画在目标之上：标记要**完整可见**才能被解码，被一条墙线切过就废了。
        import os as _os2
        if self.show_fiducials and _os2.environ.get("BULLET_NO_FIDUCIALS", "") in ("", "0"):
            self._draw_fiducials(screen, world, vp)

        # **场地白框**：平台边界在相机画面里必须是**可见的线**。
        # 之前没有边界线，场外黑区会被当成"墙上的缺口"（实测缺口中心被拉到
        # 场地左边缘），角色于是朝平台外走。白框同时给视觉一个硬边界。
        # ⚠ 这段**不能**放在 `show_fiducials` 的条件里 —— 之前放进去过，
        #   结果 `BULLET_NO_FIDUCIALS=1` 时连白框一起消失（实测：录像里平台四周
        #   没有任何边框）。白框只受它自己的开关 `BULLET_NO_FRAME` 控制。
        if _os2.environ.get("BULLET_NO_FRAME", "") in ("", "0"):
            fw = float(world.state.env.field_w)
            fh = float(world.state.env.field_h)
            a = vp.world_to_screen(0.0, fh)
            b = vp.world_to_screen(fw, 0.0)
            pygame.draw.rect(
                screen, (255, 255, 255),
                pygame.Rect(int(min(a[0], b[0])), int(min(a[1], b[1])),
                            max(1, int(abs(b[0] - a[0]))), max(1, int(abs(b[1] - a[1])))),
                2)

        # --- 标定参考物：画在**所有东西之上** ---------------------------
        # 位置在这里是有意的：障碍物/玩家先画，参考物最后覆盖上去，
        # 于是它绝不会被一条墙线切开（见 _draw_calibration_marker 的说明）。
        if self.show_calibration_marker:
            self._draw_calibration_marker(screen, world, vp)

        if self.show_hud and self.show_text:
            self._draw_hud(screen, world, fps)

        if self.mode == "rgb_array":
            arr = pygame.surfarray.array3d(screen)
            return np.transpose(arr, (1, 0, 2))
        pygame.display.flip()
        return None

    # ------------------------------------------------------------------
    def _draw_fiducials(self, screen: Any, world: Any, vp: Viewport) -> None:
        """四角 ArUco 标定标记。

        位图与几何来自 ``standard.fiducial``（共享契约）。这里只做两件事：
        按当前缩放把边长**吸附到整数模块像素**（否则模块间会有半像素缝、
        相机一模糊就连片），然后逐块画矩形。

        画法刻意不依赖 cv2：位图是硬编码的 4x4 位（apriltag 也是把家族编码
        硬编码成 ``uint64_t *codes``），所以平台侧仍然只依赖 numpy + pygame。
        """
        from standard.fiducial import (FIDUCIAL_LEVEL, FiducialLayout,
                                       snap_marker_side)

        layout = FiducialLayout(field_w=float(world.state.env.field_w),
                                field_h=float(world.state.env.field_h),
                                marker_side=self.fiducial_side)
        layout = snap_marker_side(layout, float(vp.scale))
        # 亮块用 FIDUCIAL_LEVEL（中灰）而**不是纯白**：纯白会满足视觉链的
        # "亮块"判据（min(R,G,B) > 195，见 vision.primitives），四角就出现
        # 四块比角色还大的亮块，把角色检测挤掉 —— 实测角色被钉死在标记上、
        # 逐帧不动。量化依据见 standard.fiducial.FIDUCIAL_LEVEL 的注释。
        light = (FIDUCIAL_LEVEL, FIDUCIAL_LEVEL, FIDUCIAL_LEVEL)
        for slot in sorted(layout.to_dict()["ids"]):
            for (x, y, side, white) in layout.rects(slot):
                # 场地 y **向上**为正（Viewport 把 y 翻了），所以矩形的 (x,y) 是
                # **下边缘**：把两个对角都映射过去再取 min/max，就与 y 方向无关了。
                ax, ay = vp.world_to_screen(x, y)
                bx, by = vp.world_to_screen(x + side, y + side)
                sx0, sy0 = min(ax, bx), min(ay, by)
                px = max(1, int(round(vp.len_to_screen(side))))
                rect = pygame.Rect(int(round(sx0)), int(round(sy0)), px, px)
                pygame.draw.rect(screen, light if white else (0, 0, 0), rect)

    def _draw_calibration_marker(self, screen: Any, world: Any, vp: Viewport) -> None:
        """画出标定参考物（实心矩形，锚在场地左下角）。

        为什么锚在 (0,0)：参考物的世界四角就是 (0,0)(w,0)(w,h)(0,h)，
        与 `Marker.world_corners()` 的顺序完全对齐，标定时零换算。注意世界
        y 向上、屏幕 y 向下，所以世界原点落在**屏幕左下角**。

        为什么默认是洋红而不是白色：相机拍的是显示器，房间里任何反光都会在
        画面里形成大块高亮。白色参考物的判据（"三通道都亮且接近"）恰好把反光
        一起选中；实测一帧 640×480 里反光连通域 36065 px，白块只有 6468 px，
        "取最大亮块"必然选错。洋红的判据 ``2G < R and 2G < B`` 对无彩色的
        反光恒为假，而场景调色板（overlays.color_for_type）里最接近的粉
        (230,130,220) 也满足 2·130 > 220 而被排除。

        唯一的"竞争者"是玩家圆（白色 (220,220,220)），既不满足洋红判据，
        面积也远小于参考物。
        """
        mx0, my0, mx1, my1 = self.calibration_marker_world_rect(world)
        p0 = vp.world_to_screen(mx0, my1)          # 屏幕 y 向下，所以上边用 y1
        p1 = vp.world_to_screen(mx1, my0)
        rect = pygame.Rect(int(min(p0[0], p1[0])), int(min(p0[1], p1[1])),
                           max(1, int(abs(p1[0] - p0[0]))),
                           max(1, int(abs(p1[1] - p0[1]))))
        pygame.draw.rect(screen, self.calibration_marker_color, rect)

    def _draw_obstacles(self, screen: Any, world: Any, vp: Viewport) -> None:
        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size == 0:
            return
        d = pool.data
        shapes = d["shape"][idx]
        types = d["type_id"][idx]
        # 墙（矩形）统一用 self.wall_color，不用子弹调色板 —— 见 WALL_COLOR。
        # 圆形障碍保留 type_id 上色（那里颜色是有效的区分信息）。
        xs = d["x"][idx]
        ys = d["y"][idx]
        hw = d["half_w"][idx]
        hh = d["half_h"][idx]
        rot = d["rotation"][idx]
        rad = d["radius"][idx]

        # rectangles first (walls), circles on top
        rects = shapes != 0
        for i in np.nonzero(rects)[0]:
            pts = vp.rect_to_screen(
                float(xs[i]), float(ys[i]), float(hw[i]), float(hh[i]), float(rot[i])
            )
            colour = self.wall_color
            pygame.draw.polygon(screen, colour, pts)
            # 黑边：墙的轮廓（纯视觉，帮助看清"墙是一条带"）
            pygame.draw.polygon(screen, (0, 0, 0), pts, 2)
            if self.show_hitboxes:
                pygame.draw.polygon(screen, (255, 255, 255), pts, 1)
        circles = ~rects
        if circles.any():
            cxs = xs[circles]
            cys = ys[circles]
            crs = rad[circles]
            cts = types[circles]
            for tid in np.unique(cts):
                sel = cts == tid
                colour = color_for_type(int(tid))
                for x, y, r in zip(cxs[sel], cys[sel], crs[sel]):
                    px, py = vp.world_to_screen(float(x), float(y))
                    pygame.draw.circle(
                        screen, colour, (int(px), int(py)),
                        max(1, int(vp.len_to_screen(float(r)))),
                    )
                    if self.show_hitboxes:
                        pygame.draw.circle(
                            screen, (255, 255, 255), (int(px), int(py)),
                            max(1, int(vp.len_to_screen(float(r)))), 1,
                        )

    def _draw_player(self, screen: Any, world: Any, vp: Viewport) -> None:
        """Draw the player as one circle: the drawn size *is* the hitbox.

        There is no separate visible body any more, so the picture can never
        claim a size the physics does not have.  A short heading tick inside the
        circle keeps the movement direction readable.
        """
        p = world.player
        from bullet_sim.entities.player import P_RADIUS, P_VX, P_VY, P_X, P_Y

        px, py = vp.world_to_screen(float(p[P_X]), float(p[P_Y]))
        radius = float(p[P_RADIUS])
        r = max(1, int(round(vp.len_to_screen(radius))))

        pygame.draw.circle(screen, (235, 240, 250), (int(px), int(py)), r)
        # 描边必须是**亮色**：可见的玩家圆就是碰撞圆，所以从中心到边缘都应该是
        # "亮且接近无彩色"的。原来这里画的是深蓝 (90,110,150)，于是视觉模块用
        # "亮块"去找玩家时只量到里面那圈白芯，半径系统性偏小 —— 实测干净渲染下
        # 半径 10 的玩家，白芯只有约 6.8 px，填充率 0.56，恰好卡在检测阈值下方。
        # 真机上因为有镜头模糊把描边糊进了白芯才勉强能过，这属于"靠模糊兜底"，
        # 不该依赖。改成亮色之后，"相机看到的圆"与"物理碰撞圆"是同一个。
        pygame.draw.circle(
            screen, (255, 255, 255), (int(px), int(py)), r, max(1, int(2 * vp.scale))
        )
        if p[P_VX] or p[P_VY]:
            heading = float(np.arctan2(float(p[P_VY]), float(p[P_VX])))
            tip = vp.world_to_screen(
                float(p[P_X]) + np.cos(heading) * radius,
                float(p[P_Y]) + np.sin(heading) * radius,
            )
            pygame.draw.line(screen, (90, 110, 150), (int(px), int(py)),
                             (int(tip[0]), int(tip[1])), max(1, int(2 * vp.scale)))
        # the collision outline is the same circle
        if self.show_hitboxes:
            pygame.draw.circle(screen, (255, 80, 80), (int(px), int(py)), r, 2)
            pygame.draw.circle(screen, (255, 255, 255), (int(px), int(py)), 1)

    def _draw_world_axes(self, screen: Any, world: Any, vp: Viewport) -> None:
        """World-coordinate grid; the numeric tick labels need ``show_text``.

        ⚠ ``BULLET_NO_GRID=1`` 时**不画网格**。为什么需要这个开关：网格线和墙在
        相机里只差 ~10 个灰阶（墙 L≈128、场地 L≈118、网格 L≈40~60 但会被自动
        曝光拉近），缺口定位会被这些横线带偏；实测把网格去掉后缺口中心才稳。
        """
        import os as _os
        if _os.environ.get("BULLET_NO_GRID", "") not in ("", "0"):
            return
        w = float(world.state.env.field_w)
        h = float(world.state.env.field_h)
        step = 100.0
        colour = (70, 82, 105)
        x = 0.0
        while x <= w + 1e-6:
            p0 = vp.world_to_screen(x, 0.0)
            p1 = vp.world_to_screen(x, h)
            pygame.draw.line(screen, colour, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), 1)
            if self.show_text and x % (step * 2) == 0:
                lab = self._font.render(f"{x:.0f}", True, (95, 110, 140))
                screen.blit(lab, (int(p0[0]) + 2, int(p0[1]) - lab.get_height() - 2))
            x += step
        y = 0.0
        while y <= h + 1e-6:
            p0 = vp.world_to_screen(0.0, y)
            p1 = vp.world_to_screen(w, y)
            pygame.draw.line(screen, colour, (int(p0[0]), int(p0[1])), (int(p1[0]), int(p1[1])), 1)
            if self.show_text and y % (step * 2) == 0:
                lab = self._font.render(f"{y:.0f}", True, (95, 110, 140))
                screen.blit(lab, (int(p0[0]) + 2, int(p0[1]) + 2))
            y += step

    # ------------------------------------------------------------------
    def resize(self, width: int, height: int) -> "PygameRenderer":
        """React to a new window/surface size.  The physical world is unchanged."""
        if not self._initialized:
            self.fullscreen = False          # 手动 resize 就退出全屏
            self._window_size = (int(width), int(height))
            return self
        self._size = (max(64, int(width)), max(64, int(height)))
        if self.mode == "rgb_array":
            self._screen = pygame.Surface(self._size)
        else:
            self._screen = pygame.display.set_mode(self._size, pygame.RESIZABLE)
        if self.viewport is not None:
            self.viewport.resize(*self._size)
        return self

    @property
    def scale(self) -> float:  # type: ignore[override]
        """Current world->screen scale (from the viewport when initialised)."""
        if self.viewport is not None:
            return self.viewport.scale
        return self._scale_hint

    @scale.setter
    def scale(self, value: float) -> None:
        self._scale_hint = float(value)

    def _step_hint(self) -> int:
        return int(getattr(self, "_step_index", -1))

    def _active_controller(self) -> Any:
        """The controller driving the avatar right now (through a switch if any)."""
        source = self.input_source
        active = getattr(source, "active", source)
        return getattr(active, "provider", active) or self.controller

    def _draw_hud(self, screen: Any, world: Any, fps: float) -> None:
        self._step_index = int(getattr(world.state.env, "step_index", -1))
        source = self.input_source
        held = getattr(source, "pressed_keys", None)
        if held is None:
            active = getattr(source, "active", None)
            held = getattr(active, "pressed_keys", None)
        help_text = "[TAB] manual/auto   [H] hud   [ESC] quit"
        extra = [help_text + (f"   held={sorted(held)}" if held else "")]
        # 任何输入源都能补一行自己的状态（板子链路用它显示"有没有收到信号"）。
        # 渲染器只认 `hud_line()` 这个方法名，不认识"开发板"这个概念。
        for cand in (source, getattr(source, "adapter", None)):
            fn = getattr(cand, "hud_line", None)
            if callable(fn):
                try:
                    line = fn()
                except Exception:
                    line = None
                if line:
                    extra.insert(0, str(line))
                # 决策过程（如果这条链路有的话）：一行一条，插在状态行下面
                dbg = getattr(cand, "debug_hud", None)
                if callable(dbg):
                    try:
                        for d in list(dbg())[::-1]:
                            extra.insert(1, str(d))
                    except Exception:
                        pass
                break
        lines = hud_lines(
            world,
            fps=fps,
            source=source,
            controller=self._active_controller(),
            extra=extra,
        )
        y = 4
        for line in lines:
            surf = self._font.render(line, True, (225, 230, 240))
            bg = pygame.Surface((surf.get_width() + 6, surf.get_height() + 2), pygame.SRCALPHA)
            bg.fill((0, 0, 0, 130))
            screen.blit(bg, (4, y))
            screen.blit(surf, (7, y + 1))
            y += surf.get_height() + 2

        # control guide, shown for the first seconds of a session
        if self.help_seconds > 0 and (time.monotonic() - self.started_at) < self.help_seconds:
            self._draw_help_panel(screen)

        if self.mode_banner and time.monotonic() < self.mode_banner_until:
            surf = self._font.render(self.mode_banner, True, (20, 20, 20))
            pad = 8
            box = pygame.Surface(
                (surf.get_width() + 2 * pad, surf.get_height() + 2 * pad), pygame.SRCALPHA
            )
            box.fill((250, 230, 120, 230))
            x = (self._size[0] - box.get_width()) // 2
            y = int(self._size[1] * 0.18)
            screen.blit(box, (x, y))
            screen.blit(surf, (x + pad, y + pad))

    def _draw_help_panel(self, screen: Any) -> None:
        """Loud, temporary on-screen control guide (also printed to the console)."""
        rows = [
            "CONTROLS",
            "move ....... arrow keys  or  W A S D",
            "diagonals .. hold two directions",
            "focus/slow . hold SHIFT (0.4x) or CTRL (0.25x)",
            "stop ....... SPACE",
            "TAB ........ switch MANUAL <-> AUTONOMOUS",
            "H .......... toggle this HUD",
            "ESC / Q .... quit",
        ]
        rendered = [self._font.render(r, True, (240, 244, 255)) for r in rows]
        w = max(s.get_width() for s in rendered) + 20
        h = sum(s.get_height() + 3 for s in rendered) + 16
        panel = pygame.Surface((w, h), pygame.SRCALPHA)
        panel.fill((10, 14, 24, 220))
        pygame.draw.rect(panel, (90, 120, 170), panel.get_rect(), 1)
        y = 8
        for surf in rendered:
            panel.blit(surf, (10, y))
            y += surf.get_height() + 3
        x = (self._size[0] - w) // 2
        y0 = int(self._size[1] * 0.55)
        screen.blit(panel, (max(4, x), max(4, y0)))

    def _draw_prediction(self, screen: Any, world: Any) -> None:
        try:
            forecast = self.predictor.predict(
                world.get_state(), self.prediction_horizon, world.spec.dt
            )
        except Exception:  # pragma: no cover - predictor errors must not break the view
            return
        vp = self.viewport
        for line in prediction_polylines(forecast, max_lines=self.max_prediction_lines):
            pts = [vp.world_to_screen(float(x), float(y)) for x, y in line]
            if len(pts) >= 2:
                pygame.draw.lines(screen, (70, 110, 170), False, pts, 1)

    def _draw_potential(self, screen: Any, world: Any) -> None:
        """把 `core/cpu/field_demo.py` 的**决策势场**画到同一块画布上。

        为什么用它的实现而不是自己再画一套：那是用户看的基准可视化
        （`python cpu_field_demo.py --goal auto`），分层色、等值带、膨胀障碍、
        目标点都在里面；移植它才能保证"平台上看到的"和"那个 demo 里看到的"是
        同一个东西，而不是两份近似。

        目标点**只有一个来源**：`gap_avoid.plan()` 的 `aim_world`（与板子同一套
        决策代码）。图上的准星、场的目标、平台执行的方向都从这里来。
        """
        from cpu.adapters.sim_adapter import observe_rects, view_from_world
        from cpu.field_demo import draw_field, draw_marks, field_overlay
        from cpu.gap_avoid import AlgoConfig, GapMemory, plan
        from cpu.potential_field import FieldConfig, FieldMemory, FieldView

        if self._field_mem is None:
            self._field_mem = FieldMemory()
            self._field_cfg = FieldConfig()
            self._gap_mem = GapMemory()
        import pygame

        env = world.state.env
        frame_no = int(env.step_index)
        # 一切（决策瞄准点、场、绘制）都放进节流块：实测每帧做这些会把仿真从
        # 60 帧/秒拖到 14~16 帧/秒，仿真时间轴随之变慢（用户感觉"墙变慢"）。
        if self._pot_surf is None or (frame_no - self._pot_frame) >= self.potential_every:
            self._pot_frame = frame_no
            dec = plan(view_from_world(world), self._gap_mem, AlgoConfig(), 1e9)
            aim = None
            ax, ay = float(dec.aim_world[0]), float(dec.aim_world[1])
            if ax == ax and ay == ay and abs(ax) < 1e6 and abs(ay) < 1e6:
                aim = (ax, ay)
            view = FieldView(
                player=[float(v) for v in world.player],
                rects=observe_rects(world.state.bullets),
                field_w=float(env.field_w), field_h=float(env.field_h),
                goal=aim, step_index=frame_no, dt=float(env.dt),
            )
            ov = field_overlay(view, self._field_mem, self._field_cfg, aim=aim)
            # **半分辨率**画势场再放大贴回：势场本身是成块的，半分辨率几乎看不出差别，
            # 但每帧的贴图量降到 1/4（实测整幅 SRCALPHA 贴图把 60 帧/秒压到 35）。
            if self._pot_surf is None:
                self._pot_scale = 0.5
                self._pot_surf = pygame.Surface(
                    (max(1, int(self._size[0] * self._pot_scale)),
                     max(1, int(self._size[1] * self._pot_scale))), pygame.SRCALPHA)
            else:
                self._pot_surf.fill((0, 0, 0, 0))
            from dataclasses import replace as _dc_replace
            sub_vp = _dc_replace(self.viewport,
                                 screen_w=self._pot_surf.get_width(),
                                 screen_h=self._pot_surf.get_height(),
                                 margin=max(0, int(self.viewport.margin * self._pot_scale)))
            draw_field(self._pot_surf, sub_vp, ov)
            draw_marks(self._pot_surf, sub_vp, ov)
        if self._pot_surf.get_size() != self._size:
            screen.blit(pygame.transform.smoothscale(self._pot_surf, self._size), (0, 0))
        else:
            screen.blit(self._pot_surf, (0, 0))

    def _draw_danger(self, screen: Any, world: Any) -> None:
        danger = self.danger
        if danger is None:
            return
        vp = self.viewport
        # **分层色**：把连续的危险值量化成 4 档，每档一个颜色（用户要的"分层"观感）。
        # 只是显示方式，不改变势图数值本身。
        layer = pygame.Surface(self._size, pygame.SRCALPHA)
        levels = ((0.15, (60, 200, 120, 40)),     # 低：青绿
                  (0.35, (230, 220, 70, 55)),     # 中：黄
                  (0.55, (240, 150, 50, 70)),     # 高：橙
                  (0.75, (230, 60, 60, 95)))      # 极高：红
        for r, c, v in danger_cells(danger, step=0, threshold=0.15):
            colour = None
            for thr, col in levels:
                if v >= thr:
                    colour = col
            if colour is None:
                continue
            wx = (c + 0.5) * danger.field_w / danger.cols
            wy = (r + 0.5) * danger.field_h / danger.rows
            px, py = vp.world_to_screen(wx, wy)
            half_w = max(1, int(vp.len_to_screen(danger.field_w / danger.cols) / 2))
            half_h = max(1, int(vp.len_to_screen(danger.field_h / danger.rows) / 2))
            pygame.draw.rect(
                layer, colour,
                pygame.Rect(int(px - half_w), int(py - half_h), 2 * half_w, 2 * half_h),
            )
        screen.blit(layer, (0, 0))

    # ------------------------------------------------------------------
    def _pump_events(self) -> None:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.closed = True
            elif event.type == pygame.VIDEORESIZE:
                # Only the observation scale changes; the world is untouched.
                self.resize(int(event.w), int(event.h))
            elif event.type == pygame.MOUSEWHEEL:
                if self.viewport is not None:
                    self.viewport.zoom_by(1.1 if event.y > 0 else 1 / 1.1)
                    self.mode_banner = f"zoom {self.viewport.zoom:.2f}x"
                    self.mode_banner_until = time.monotonic() + 0.8
            elif event.type == pygame.KEYDOWN:
                if event.key in (pygame.K_ESCAPE, pygame.K_q):
                    self.closed = True
                elif event.key == pygame.K_h:
                    self.show_hud = not self.show_hud
                elif event.key == pygame.K_b:
                    self.show_hitboxes = not self.show_hitboxes
                elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                    self.viewport and self.viewport.zoom_by(1.15)
                elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                    self.viewport and self.viewport.zoom_by(1 / 1.15)
                elif event.key == pygame.K_0:
                    self.viewport and self.viewport.reset_view()
                elif event.key == self.switch_key and hasattr(self.input_source, "toggle"):
                    new_mode = self.input_source.toggle(step=self._step_hint())
                    self.mode_banner = f"mode -> {str(new_mode).upper()}"
                    self.mode_banner_until = time.monotonic() + 1.5
                elif self.input_source is not None:
                    name = key_name_for(event.key)
                    if name is not None:
                        self.input_source.press(name)
                elif self.interactive:
                    self.pending_action = self._key_to_action(event.key)
            elif event.type == pygame.KEYUP and self.input_source is not None:
                name = key_name_for(event.key)
                if name is not None:
                    self.input_source.release(name)

    @staticmethod
    def _key_to_action(key: int) -> Any:
        from bullet_sim.core.actions import DISCRETE_ACTION_INDEX

        mapping = {
            pygame.K_LEFT: "left",
            pygame.K_RIGHT: "right",
            pygame.K_UP: "up",
            pygame.K_DOWN: "down",
            pygame.K_a: "left",
            pygame.K_d: "right",
            pygame.K_w: "up",
            pygame.K_s: "down",
            pygame.K_SPACE: "stay",
        }
        name = mapping.get(key)
        return DISCRETE_ACTION_INDEX.get(name, 0) if name else None

    def alive(self) -> bool:
        """窗口还活着吗？

        ⚠ 必须单独有这个方法：**点窗口右上角的 × 时 SDL 不一定投递 `pygame.QUIT`**
        （WSLg 上实测过），而 `--nostop` 不设步数上限 —— 那就变成"窗口没了、进程还在
        跑"（用户看到的就是卡死）。每帧问一次这个，就没有那条路。

        ⚠⚠ **还没初始化时必须是 True**：窗口是在第一次 `render()` 里才创建的
        （`_ensure_init` → `pygame.display.set_mode`）。主循环在第一次 render **之前**
        就问这一句，那时 `pygame.display.get_init()` 还是 False —— 判成"窗口已关闭"
        就会**当场退出、窗口根本不开**（这个回归真发生过：用户看到的就是"弹窗不出来"）。
        """
        if self.closed or not self._initialized:
            return not self.closed
        try:
            return bool(pygame.display.get_init())
        except Exception:
            return False

    def close(self) -> None:
        if self._initialized:
            pygame.quit()
        self._initialized = False
        self.closed = True


__all__ = ["PygameRenderer"]

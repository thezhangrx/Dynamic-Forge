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
WALL_COLOR = (240, 160, 48)


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
        fiducial_side: float = 48.0,
    ) -> None:
        self.mode = mode
        self.scale = float(scale)
        self.show_hud = bool(show_hud)
        #: Master switch for every piece of text drawn on the window.  Off by
        #: default: the popup is meant to show the scene, not labels.
        self.show_text = bool(show_text)
        self.show_prediction = bool(show_prediction)
        self.show_danger = bool(show_danger)
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
        pygame.init()
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
        pygame.draw.rect(screen, (34, 40, 55), field_rect)
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
        if self.show_fiducials:
            self._draw_fiducials(screen, world, vp)

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
        """World-coordinate grid; the numeric tick labels need ``show_text``."""
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
        lines = hud_lines(
            world,
            fps=fps,
            source=source,
            controller=self._active_controller(),
            extra=[help_text + (f"   held={sorted(held)}" if held else "")],
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

    def _draw_danger(self, screen: Any, world: Any) -> None:
        danger = self.danger
        if danger is None:
            return
        vp = self.viewport
        layer = pygame.Surface(self._size, pygame.SRCALPHA)
        for r, c, v in danger_cells(danger, step=0, threshold=0.15):
            alpha = int(min(1.0, v) * 90)
            wx = (c + 0.5) * danger.field_w / danger.cols
            wy = (r + 0.5) * danger.field_h / danger.rows
            px, py = vp.world_to_screen(wx, wy)
            half_w = max(1, int(vp.len_to_screen(danger.field_w / danger.cols) / 2))
            half_h = max(1, int(vp.len_to_screen(danger.field_h / danger.rows) / 2))
            pygame.draw.rect(
                layer, (255, 80, 80, alpha),
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

    def close(self) -> None:
        if self._initialized:
            pygame.quit()
        self._initialized = False
        self.closed = True


__all__ = ["PygameRenderer"]

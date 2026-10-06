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
        show_hitboxes: bool = True,
        show_world_axes: bool = True,
    ) -> None:
        self.mode = mode
        self.scale = float(scale)
        self.show_hud = bool(show_hud)
        #: Master switch for every piece of text drawn on the window.  Off by
        #: default: the popup is meant to show the scene, not labels.
        self.show_text = bool(show_text)
        self.show_prediction = bool(show_prediction)
        self.show_danger = bool(show_danger)
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
        self.show_hitboxes = bool(show_hitboxes)
        self.show_world_axes = bool(show_world_axes)
        self.viewport: Viewport | None = None

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
            flags = pygame.RESIZABLE if self.resizable else 0
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

        # --- target zone ------------------------------------------------
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

        if self.show_hud and self.show_text:
            self._draw_hud(screen, world, fps)

        if self.mode == "rgb_array":
            arr = pygame.surfarray.array3d(screen)
            return np.transpose(arr, (1, 0, 2))
        pygame.display.flip()
        return None

    # ------------------------------------------------------------------
    def _draw_obstacles(self, screen: Any, world: Any, vp: Viewport) -> None:
        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size == 0:
            return
        d = pool.data
        shapes = d["shape"][idx]
        types = d["type_id"][idx]
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
            colour = color_for_type(int(types[i]))
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
        pygame.draw.circle(
            screen, (90, 110, 150), (int(px), int(py)), r, max(1, int(2 * vp.scale))
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

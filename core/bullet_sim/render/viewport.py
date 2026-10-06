"""Viewport: the single place where world coordinates become screen pixels.

The simulator works in **world units** (metres / pixels of the abstract field);
the window works in **screen pixels**.  Nothing else in the code base is allowed
to convert between them, which is what makes the two requirements hold at once:

* resizing the window changes only the *observation scale*;
* no physical quantity (position, velocity, radius, hitbox) ever changes.

Design rules
------------
1. **Uniform scale.**  One ``scale`` for both axes, so circles stay circles and
   rectangles keep their aspect ratio - a window resize can never stretch the
   world.
2. **Letterbox, never crop.**  ``fit`` mode picks ``min(screen/world)`` and
   centres the result, leaving bars instead of hiding part of the field.
3. **World-locked.**  ``world_w``/``world_h`` come from the scenario and are
   fixed for the whole episode; the viewport is derived from them.
4. **Zoom is a view parameter.**  ``zoom`` multiplies the fit scale and is
   clamped so the world always covers the window; it is *not* a physics knob.

Everything the renderer needs goes through :meth:`world_to_screen`,
:meth:`screen_to_world`, :meth:`len_to_screen` and :meth:`rect_to_screen`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

#: Zoom bounds: below 1.0 the world would not cover the window.
MIN_ZOOM = 1.0
MAX_ZOOM = 12.0


@dataclass
class Viewport:
    """Uniform, aspect-preserving world <-> screen mapping."""

    world_w: float
    world_h: float
    screen_w: int
    screen_h: int
    #: Pixels kept free on every side (reserved for HUD/borders).
    margin: int = 0
    #: View zoom relative to the fit scale; 1.0 == exactly fit.
    zoom: float = 1.0
    #: World point that should sit at the screen centre (``None`` = world centre).
    center: tuple[float, float] | None = None
    #: Extra padding around the world in world units (keeps edge bullets visible).
    world_pad: float = 0.0

    # -- derived -----------------------------------------------------------
    @property
    def usable_w(self) -> int:
        return max(1, int(self.screen_w) - 2 * int(self.margin))

    @property
    def usable_h(self) -> int:
        return max(1, int(self.screen_h) - 2 * int(self.margin))

    @property
    def world_extent_w(self) -> float:
        return max(1e-9, float(self.world_w) + 2.0 * float(self.world_pad))

    @property
    def world_extent_h(self) -> float:
        return max(1e-9, float(self.world_h) + 2.0 * float(self.world_pad))

    @property
    def fit_scale(self) -> float:
        """Largest uniform scale that fits the whole world in the window."""
        return min(self.usable_w / self.world_extent_w, self.usable_h / self.world_extent_h)

    @property
    def scale(self) -> float:
        return self.fit_scale * max(MIN_ZOOM, min(MAX_ZOOM, float(self.zoom)))

    @property
    def origin_x(self) -> float:
        """Screen x of world x = 0 (letterbox offset, zoom-aware)."""
        return (self.screen_w - self.world_extent_w * self.scale) * 0.5 + self.world_pad * self.scale

    @property
    def origin_y(self) -> float:
        """Screen y of world y = world_h (screen y grows downward)."""
        return (self.screen_h - self.world_extent_h * self.scale) * 0.5 + self.world_pad * self.scale

    @property
    def center_world(self) -> tuple[float, float]:
        if self.center is not None:
            return (float(self.center[0]), float(self.center[1]))
        return (self.world_w * 0.5, self.world_h * 0.5)

    @property
    def center_px(self) -> tuple[float, float]:
        cx, cy = self.center_world
        return self.world_to_screen(cx, cy)

    # -- mapping -----------------------------------------------------------
    def world_to_screen(self, x: float, y: float) -> tuple[float, float]:
        """World point -> screen pixel (uniform scale, y flipped, zoom centred)."""
        cx, cy = self.center_world
        sx = self.screen_w * 0.5 + (float(x) - cx) * self.scale
        sy = self.screen_h * 0.5 - (float(y) - cy) * self.scale
        return sx, sy

    def screen_to_world(self, sx: float, sy: float) -> tuple[float, float]:
        cx, cy = self.center_world
        x = cx + (float(sx) - self.screen_w * 0.5) / self.scale
        y = cy - (float(sy) - self.screen_h * 0.5) / self.scale
        return x, y

    def len_to_screen(self, length: float) -> float:
        """World length -> screen length (always uniform, never axis dependent)."""
        return float(length) * self.scale

    def vec_to_screen(self, vx: float, vy: float, *, flip_y: bool = True) -> tuple[float, float]:
        """World direction -> screen direction (for drawing arrows/axes)."""
        return float(vx) * self.scale, (-float(vy) if flip_y else float(vy)) * self.scale

    # -- shapes ------------------------------------------------------------
    def rect_to_screen(
        self, cx: float, cy: float, half_w: float, half_h: float, rotation: float = 0.0
    ) -> list[tuple[float, float]]:
        """Corners of a (possibly rotated) rectangle, in screen pixels.

        Returned as a polygon so the renderer can use ``pygame.draw.polygon`` and
        a rotated obstacle is exact instead of approximated by an axis-aligned
        bounding box.
        """
        import math

        cos_r, sin_r = math.cos(rotation), math.sin(rotation)
        pts: list[tuple[float, float]] = []
        for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            wx = cx + (sx * half_w) * cos_r - (sy * half_h) * sin_r
            wy = cy + (sx * half_w) * sin_r + (sy * half_h) * cos_r
            pts.append(self.world_to_screen(wx, wy))
        return pts

    def visible_world_box(self) -> tuple[float, float, float, float]:
        """``(x0, y0, x1, y1)`` world rect currently visible (for culling)."""
        x0, y1 = self.screen_to_world(0.0, 0.0)
        x1, y0 = self.screen_to_world(self.screen_w, self.screen_h)
        return x0, y0, x1, y1

    # -- control -----------------------------------------------------------
    def resize(self, screen_w: int, screen_h: int) -> "Viewport":
        """React to a window resize.  Only the observation scale changes."""
        self.screen_w = max(1, int(screen_w))
        self.screen_h = max(1, int(screen_h))
        return self

    def set_zoom(self, zoom: float) -> "Viewport":
        self.zoom = max(MIN_ZOOM, min(MAX_ZOOM, float(zoom)))
        return self

    def zoom_by(self, factor: float) -> "Viewport":
        return self.set_zoom(self.zoom * float(factor))

    def pan(self, dx_world: float, dy_world: float) -> "Viewport":
        cx, cy = self.center_world
        self.center = (cx + float(dx_world), cy + float(dy_world))
        return self

    def reset_view(self) -> "Viewport":
        self.zoom = 1.0
        self.center = None
        return self

    # -- diagnostics -------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        x0, y0, x1, y1 = self.visible_world_box()
        return {
            "world": [self.world_w, self.world_h],
            "screen": [self.screen_w, self.screen_h],
            "scale": self.scale,
            "fit_scale": self.fit_scale,
            "zoom": self.zoom,
            "uniform": True,
            "letterbox_px": [
                round(self.origin_x, 2),
                round(self.origin_y, 2),
            ],
            "visible_world": [round(v, 2) for v in (x0, y0, x1, y1)],
        }

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return (
            f"Viewport(world={self.world_w:g}x{self.world_h:g}, "
            f"screen={self.screen_w}x{self.screen_h}, scale={self.scale:.4f})"
        )


def viewport_for(world: Any, screen_size: Sequence[int] | None = None, **kw: Any) -> Viewport:
    """Build a viewport that fits ``world``'s field, optionally at a given size."""
    field_w = float(world.state.env.field_w)
    field_h = float(world.state.env.field_h)
    if screen_size is None:
        screen = (int(round(field_w)), int(round(field_h)))
    else:
        screen = (int(screen_size[0]), int(screen_size[1]))
    return Viewport(world_w=field_w, world_h=field_h, screen_w=screen[0], screen_h=screen[1], **kw)


__all__ = ["Viewport", "viewport_for", "MIN_ZOOM", "MAX_ZOOM"]

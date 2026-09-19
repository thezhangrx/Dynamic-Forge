"""Dependency-free ASCII renderer.

Useful over SSH, in CI logs and inside tests, where opening a window is not
possible.  It renders the same information as the pygame view: player, bullets,
target zone, HUD (FPS, bullet count, player state) and optional danger overlay.
"""

from __future__ import annotations

from typing import Any, Sequence

import numpy as np

from bullet_sim.render.overlays import FrameCounter, hud_lines


class AsciiRenderer:
    name = "ascii"

    def __init__(
        self,
        width: int = 96,
        height: int = 32,
        *,
        show_hud: bool = True,
        show_danger: bool = False,
        danger: Any = None,
        danger_step: int = 0,
        bullet_char: str = "*",
        target_char: str = "O",
        player_char: str = "@",
        extra_hud: Sequence[str] | None = None,
        source: Any = None,
        controller: Any = None,
    ) -> None:
        self.width = max(8, int(width))
        self.height = max(4, int(height))
        self.show_hud = bool(show_hud)
        self.show_danger = bool(show_danger)
        self.danger = danger
        self.danger_step = int(danger_step)
        self.bullet_char = bullet_char
        self.target_char = target_char
        self.player_char = player_char
        self.extra_hud = list(extra_hud or [])
        self.source = source
        self.controller = controller
        self.frames = FrameCounter()

    # ------------------------------------------------------------------
    def _to_cell(self, world: Any, x: float, y: float) -> tuple[int, int]:
        w = float(world.state.env.field_w) or 1.0
        h = float(world.state.env.field_h) or 1.0
        col = int(np.clip(x / w * (self.width - 1), 0, self.width - 1))
        # ASCII rows grow downward; the field's +y is up.
        row = int(np.clip((1.0 - y / h) * (self.height - 1), 0, self.height - 1))
        return row, col

    def render(self, world: Any) -> str:
        fps = self.frames.tick()
        canvas = np.full((self.height, self.width), " ", dtype="<U1")

        if self.show_danger and self.danger is not None:
            grid = self.danger.grid[min(self.danger_step, self.danger.horizon - 1)]
            rows, cols = grid.shape
            for r in range(rows):
                for c in range(cols):
                    if grid[r, c] > 0.05:
                        rr = int(r / max(rows - 1, 1) * (self.height - 1))
                        cc = int(c / max(cols - 1, 1) * (self.width - 1))
                        canvas[rr, cc] = "."

        t = world.state.target
        row, col = self._to_cell(world, t.x, t.y)
        canvas[row, col] = self.target_char

        pool = world.state.bullets
        idx = pool.active_indices()
        if idx.size:
            w = float(world.state.env.field_w) or 1.0
            h = float(world.state.env.field_h) or 1.0
            xs = pool.data["x"][idx]
            ys = pool.data["y"][idx]
            cc = np.clip(xs / w * (self.width - 1), 0, self.width - 1).astype(np.int64)
            rr = np.clip((1.0 - ys / h) * (self.height - 1), 0, self.height - 1).astype(np.int64)
            canvas.ravel()[rr * self.width + cc] = self.bullet_char

        p = world.player
        from bullet_sim.entities.player import P_X, P_Y

        row, col = self._to_cell(world, float(p[P_X]), float(p[P_Y]))
        canvas[row, col] = self.player_char

        art = "\n".join("".join(line) for line in canvas)
        if not self.show_hud:
            return art
        hud = "\n".join(
            hud_lines(
                world,
                fps=fps,
                source=self.source,
                controller=self.controller,
                extra=self.extra_hud,
            )
        )
        return f"{art}\n{'-' * self.width}\n{hud}"

    def close(self) -> None:
        return None


__all__ = ["AsciiRenderer"]

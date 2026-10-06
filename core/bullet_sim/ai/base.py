"""Autonomous controller contract.

A controller is an :class:`~bullet_sim.interface.controller.ActionProvider`
plus a small amount of self-description, so the UI and the CLI can tell the
player *what* is driving the avatar and whether it needs privileged access.

Two access levels are explicitly distinguished, because they are not equivalent
and the difference matters for deployment:

``observation-only``
    The controller sees exactly what a real robot would see through the
    observation vector (player, target, live bullets).  This is what a
    deployable policy must be limited to.

``world-access``
    The controller is handed the live ``World`` so it can clone it and roll the
    future forward (MPC-style planning).  Legitimate for research and for a
    CPU/FPGA planner, but it is *not* deployable as-is; anything that uses it
    must say so.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Mapping, Protocol, runtime_checkable

import numpy as np


@dataclass(frozen=True)
class ControllerSpec:
    """Self-description used by the HUD, the CLI and the README."""

    name: str
    kind: str  # "baseline" | "model" | "manual" | "hardware"
    description: str = ""
    #: "observation" (deployable) or "world" (planning / research)
    access: str = "observation"
    deterministic: bool = True
    #: Rough cost hint, shown in the HUD: "cheap" | "moderate" | "expensive"
    cost: str = "cheap"
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "kind": self.kind,
            "description": self.description,
            "access": self.access,
            "deterministic": self.deterministic,
            "cost": self.cost,
            "notes": self.notes,
        }

    def hud_line(self) -> str:
        return f"ai        : {self.name} [{self.kind}/{self.access}/{self.cost}]"


@runtime_checkable
class AutonomousController(Protocol):
    """``act(observation, info) -> action`` plus ``spec()``."""

    def reset(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> None: ...

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Any: ...

    def spec(self) -> ControllerSpec: ...


class BaseAutonomousController:
    """Convenience base: sane defaults for ``reset`` / ``close`` / ``spec``."""

    #: Overridden by subclasses.
    SPEC = ControllerSpec(name="unnamed", kind="baseline")

    def __init__(self) -> None:
        self._last_action: Any = 0
        self._calls = 0

    # -- ActionProvider ----------------------------------------------------
    def reset(self, observation: Mapping[str, Any] | None = None,
              info: Mapping[str, Any] | None = None) -> None:
        self._last_action = 0
        self._calls = 0

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Any:
        raise NotImplementedError

    def close(self) -> None:
        return None

    # -- self description --------------------------------------------------
    def spec(self) -> ControllerSpec:
        return self.SPEC

    @property
    def calls(self) -> int:
        return self._calls

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return f"{type(self).__name__}({self.SPEC.name})"


# --------------------------------------------------------------------------
# shared geometry helpers (observation-only, no world access)
# --------------------------------------------------------------------------


def bullet_matrix(observation: Mapping[str, Any]) -> np.ndarray:
    """``(n, 7)`` live bullets from an observation: x, y, vx, vy, r, ttl, type."""
    bullets = observation.get("bullets")
    if bullets is None:
        return np.zeros((0, 7), dtype=np.float32)
    arr = np.asarray(bullets, dtype=np.float32)
    return arr.reshape(-1, 7) if arr.size else np.zeros((0, 7), dtype=np.float32)


def direction_to_action(dx: float, dy: float) -> int:
    """Map a world-frame direction onto the canonical 9-way discrete action."""
    from bullet_sim.core.actions import DISCRETE_ACTION_INDEX, DISCRETE_DIRECTIONS

    if not np.isfinite(dx) or not np.isfinite(dy) or (abs(dx) < 1e-9 and abs(dy) < 1e-9):
        return DISCRETE_ACTION_INDEX["stay"]
    unit = np.array([dx, dy], dtype=np.float64)
    unit /= float(np.hypot(unit[0], unit[1]))
    sims = DISCRETE_DIRECTIONS @ unit
    return int(np.argmax(sims))


__all__ = [
    "AutonomousController",
    "BaseAutonomousController",
    "ControllerSpec",
    "bullet_matrix",
    "direction_to_action",
]

"""CPU side of the CPU/FPGA split: turn an FPGA result into an Action.

    state frame (bytes)  --+
                           |     (FPGA, see bullet_sim.fpga)
    prediction frame ----+ |
                           v v
                     CpuDecisionLayer  ->  Action  ->  Simulator

Why this exists
---------------
The eventual hardware loop is

    Simulator State -> FPGA input -> FPGA result -> CPU -> Action -> Simulator

and the *only* thing that changes when the FPGA becomes real is where the
prediction frame comes from.  So the CPU decision layer is written against
**frames**, not against the ``World``: it decodes bytes, picks an action, and
hands back an :class:`~bullet_sim.action.types.Action`.  Everything above it
(simulator, dataset, UI) is untouched.

The transport between CPU and FPGA is ``[HARDWARE_INTERFACE_TBD]`` - see
``bullet_sim/hardware_interface/tbd.py``.  :class:`CpuFpgaPipeline` runs the loop
in-process so the interfaces can be developed and measured before the board
exists.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping

import numpy as np

from bullet_sim.action.types import Action
from bullet_sim.core.actions import DISCRETE_DIRECTIONS
from bullet_sim.fpga.prediction_block import (
    HARDWARE_INTERFACE_TBD,
    FpgaPredictionReference,
    decode_prediction_frame,
)
from bullet_sim.interface.protocol import decode_state


@dataclass
class PipelineLatency:
    """Wall-clock breakdown of one hardware-loop iteration (seconds)."""

    encode_state: float = 0.0
    fpga_predict: float = 0.0
    cpu_decide: float = 0.0
    apply_action: float = 0.0
    iterations: int = 0

    @property
    def total(self) -> float:
        return self.encode_state + self.fpga_predict + self.cpu_decide + self.apply_action

    def summary(self) -> dict[str, float]:
        n = max(self.iterations, 1)
        return {
            "iterations": self.iterations,
            "total_ms": self.total * 1e3,
            "encode_state_ms": self.encode_state / n * 1e3,
            "fpga_predict_ms": self.fpga_predict / n * 1e3,
            "cpu_decide_ms": self.cpu_decide / n * 1e3,
            "apply_action_ms": self.apply_action / n * 1e3,
            "fpga_share_pct": (self.fpga_predict / self.total * 100.0) if self.total else 0.0,
        }


class CpuDecisionLayer:
    """Pick an action from a state frame plus an FPGA risk-grid prediction.

    The rule is the same two-stage, safety-first rule the ``threat`` baseline
    uses, but it reads a *rasterised risk grid* instead of raw bullets - i.e. it
    is exactly the function the CPU will run when the grid arrives over a real
    bus.

    * **stage 1** - among the 9 candidate directions, keep those whose worst
      sampled risk is within ``tie_tolerance`` of the best;
    * **stage 2** - among those, prefer the one that reduces target distance,
      then continuity, then the lowest index (determinism).
    """

    name = "cpu-decision"

    def __init__(
        self,
        *,
        tie_tolerance: float = 0.10,
        samples: int = 12,
        progress_weight: float = 0.05,
    ) -> None:
        self.tie_tolerance = float(tie_tolerance)
        self.samples = max(2, int(samples))
        self.progress_weight = float(progress_weight)
        self._last_index = 0
        self.decisions = 0

    # ------------------------------------------------------------------
    def decide(
        self,
        state_frame: bytes,
        prediction_frame: bytes,
        *,
        player_speed: float | None = None,
    ) -> tuple[Action, dict[str, Any]]:
        """Return ``(Action, diagnostics)``; ``diagnostics`` is HUD/log material."""
        snapshot = decode_state(state_frame)
        grid, header = decode_prediction_frame(prediction_frame)
        self.decisions += 1

        player = np.array([snapshot.player.x, snapshot.player.y], dtype=np.float64)
        target = np.array([snapshot.target.x, snapshot.target.y], dtype=np.float64)
        speed = float(player_speed if player_speed is not None else snapshot.player.speed)

        horizon = int(header.horizon)
        if horizon <= 0 or grid.size == 0:
            return Action.zero(), {"reason": "empty prediction", "risk": 0.0}

        # sample the grid along each candidate straight path
        travel = np.linspace(0.0, 1.0, self.samples) * (speed * header.dt * horizon)
        risk = np.full(9, -1.0, dtype=np.float64)
        for idx in range(9):
            dx, dy = DISCRETE_DIRECTIONS[idx]
            px = player[0] + dx * travel
            py = player[1] + dy * travel
            risk[idx] = self._sample_path(grid, header, px, py)

        finite = risk >= 0.0
        if not finite.any():
            return Action.zero(), {"reason": "no samples in grid", "risk": 0.0}
        best = float(risk[finite].min())
        safe = np.flatnonzero(finite & (risk <= best + self.tie_tolerance))

        gain = np.array(
            [
                float(np.hypot(*(target - player)))
                - float(np.hypot(*(target - (player + np.asarray(DISCRETE_DIRECTIONS[i])))))
                for i in range(9)
            ]
        )
        order = sorted(safe.tolist(), key=lambda i: (-gain[i], i != self._last_index, i))
        chosen = int(order[0])
        self._last_index = chosen
        dx, dy = DISCRETE_DIRECTIONS[chosen]
        norm = float(np.hypot(dx, dy)) or 1.0
        action = Action((float(dx) / norm, float(dy) / norm), 1.0) if norm > 0 else Action.zero()
        return action, {
            "risk": best,
            "risk_vector": risk.tolist(),
            "chosen": chosen,
            "grid": {"horizon": horizon, "rows": header.rows, "cols": header.cols},
            "source_step": header.source_step,
            "frame_bytes": len(prediction_frame),
        }

    # ------------------------------------------------------------------
    @staticmethod
    def _sample_path(
        grid: np.ndarray, header: Any, px: np.ndarray, py: np.ndarray
    ) -> float:
        """Worst (maximum) risk along a path, or -1.0 if it leaves the grid."""
        cols = int(header.cols)
        rows = int(header.rows)
        cx = np.floor((px - header.origin_x) / header.cell_w).astype(np.int64)
        cy = np.floor((py - header.origin_y) / header.cell_h).astype(np.int64)
        inside = (cx >= 0) & (cx < cols) & (cy >= 0) & (cy < rows)
        if not inside.any():
            return -1.0
        cx, cy = cx[inside], cy[inside]
        step_index = np.linspace(
            0, int(header.horizon) - 1, inside.sum()
        ).astype(np.int64)
        values = grid[step_index, cy, cx]
        return float(values.max())


class CpuFpgaPipeline:
    """Run the full ``State -> FPGA -> CPU -> Action`` loop over byte frames.

    ``step(world)`` performs one iteration and returns the chosen action, so the
    pipeline can drive the simulator through the ordinary ``ActionProvider``
    path (see :class:`CpuFpgaController`).
    """

    def __init__(
        self,
        fpga: FpgaPredictionReference | None = None,
        cpu: CpuDecisionLayer | None = None,
        *,
        transport: str = HARDWARE_INTERFACE_TBD,
    ) -> None:
        from bullet_sim.interface.protocol import encode_state

        self._encode_state = encode_state
        self.fpga = fpga or FpgaPredictionReference()
        self.cpu = cpu or CpuDecisionLayer()
        self.transport = transport
        self.latency = PipelineLatency()
        self.last_diagnostics: dict[str, Any] = {}

    # ------------------------------------------------------------------
    def iterate(self, world: Any, *, apply: bool = True) -> Action:
        """One hardware-loop iteration against a live ``World``."""
        t0 = time.perf_counter()
        state_frame = self._encode_state(world.get_state())
        t1 = time.perf_counter()
        prediction_frame = self.fpga.predict(state_frame)
        t2 = time.perf_counter()
        action, diag = self.cpu.decide(state_frame, prediction_frame)
        t3 = time.perf_counter()
        if apply:
            # The CPU emits the unified Action; converting it to whatever the
            # world's codec expects is the documented seam (see action/base.py).
            from bullet_sim.action.base import action_to_codec_input
            from bullet_sim.entities.player import P_SPEED

            world.step(
                action_to_codec_input(
                    action, world.codec, float(world.player[P_SPEED])
                )
            )
        t4 = time.perf_counter()

        self.latency.encode_state += t1 - t0
        self.latency.fpga_predict += t2 - t1
        self.latency.cpu_decide += t3 - t2
        self.latency.apply_action += t4 - t3
        self.latency.iterations += 1
        self.last_diagnostics = diag
        return action

    # ------------------------------------------------------------------
    def describe(self) -> dict[str, Any]:
        return {
            "simulator -> fpga": "state frame (interface.protocol v2)",
            "fpga -> cpu": "prediction frame (BHP1)",
            "cpu -> simulator": "Action{direction, magnitude}",
            "transport": self.transport,
            "fpga": self.fpga.describe(),
            "latency": self.latency.summary(),
            "interface_status": {
                "state_frame": "implemented",
                "prediction_frame": "implemented",
                "fpga_prediction_block": "python reference implemented; HDL [TBD]",
                "cpu_decision_layer": "implemented",
                "cpu_fpga_transport": HARDWARE_INTERFACE_TBD,
            },
        }


class CpuFpgaController:
    """Adapt :class:`CpuFpgaPipeline` to the ``ActionProvider`` interface.

    Access level is ``world``: the pipeline reads the live state to build a state
    frame.  On real hardware that frame comes from sensors instead - the
    *interface* is identical, which is the whole point.
    """

    def __init__(
        self,
        world: Any,
        fpga: FpgaPredictionReference | None = None,
        cpu: CpuDecisionLayer | None = None,
    ) -> None:
        self.world = world
        self.pipeline = CpuFpgaPipeline(fpga, cpu)
        self.calls = 0
        self.last_diagnostics: dict[str, Any] = {}

    # -- ActionProvider ----------------------------------------------------
    def reset(self, observation: Mapping[str, Any] | None = None,
              info: Mapping[str, Any] | None = None) -> None:
        self.calls = 0

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> Any:
        self.calls += 1
        action = self.pipeline.iterate(self.world, apply=False)
        self.last_diagnostics = self.pipeline.last_diagnostics
        return action

    def close(self) -> None:
        return None

    def spec(self) -> Any:
        from bullet_sim.ai.base import ControllerSpec

        return ControllerSpec(
            name="cpu-fpga",
            kind="baseline",
            description="state frame -> FPGA reference -> prediction frame -> CPU decision",
            access="world",
            cost="moderate",
            notes=f"transport {HARDWARE_INTERFACE_TBD}",
        )

    def describe(self) -> dict[str, Any]:
        return self.pipeline.describe()


__all__ = [
    "PipelineLatency",
    "CpuDecisionLayer",
    "CpuFpgaPipeline",
    "CpuFpgaController",
]

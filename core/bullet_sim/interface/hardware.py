"""Hardware interface (HAL) - deliberately separate from the Simulator API.

The simulator's job is to produce ``S_t`` and consume actions.  *How* those
bytes travel to a CPU/FPGA co-processor, a serial MCU, or a real 2D cart is a
different concern, expressed here.  Both deployment paths share one contract:

    Virtual Simulator -> CPU -> FPGA
    Physical Sensors  -> CPU -> FPGA

so switching from simulation to hardware means swapping the ``HardwareLink``
implementation, nothing else.

Concrete links
--------------
``LoopbackLink`` in-process echo (self-test, latency baseline)
``FileLink``     frame exchange through files (offline / FPGA batch replay)
``NullLink``     no-op (pure CPU simulation)
"""

from __future__ import annotations

import struct
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from bullet_sim.core.errors import ProtocolError
from bullet_sim.interface.protocol import (
    decode_action_velocity,
    encode_action_velocity,
)


@dataclass
class LinkStats:
    uploads: int = 0
    downloads: int = 0
    actions: int = 0
    bytes_up: int = 0
    bytes_down: int = 0
    upload_ns: list[float] = field(default_factory=list)
    download_ns: list[float] = field(default_factory=list)
    action_ns: list[float] = field(default_factory=list)

    def record_upload(self, nbytes: int, ns: float) -> None:
        self.uploads += 1
        self.bytes_up += nbytes
        self.upload_ns.append(ns)

    def record_download(self, nbytes: int, ns: float) -> None:
        self.downloads += 1
        self.bytes_down += nbytes
        self.download_ns.append(ns)

    def record_action(self, ns: float) -> None:
        self.actions += 1
        self.action_ns.append(ns)

    def summary(self) -> dict[str, Any]:
        def stats(xs: list[float]) -> dict[str, float]:
            if not xs:
                return {"count": 0}
            a = np.asarray(xs, dtype=np.float64)
            return {
                "count": int(a.size),
                "mean_us": float(a.mean() / 1000.0),
                "p50_us": float(np.percentile(a, 50) / 1000.0),
                "p95_us": float(np.percentile(a, 95) / 1000.0),
                "max_us": float(a.max() / 1000.0),
            }

        return {
            "uploads": self.uploads,
            "downloads": self.downloads,
            "actions": self.actions,
            "bytes_up": self.bytes_up,
            "bytes_down": self.bytes_down,
            "upload_latency": stats(self.upload_ns),
            "download_latency": stats(self.download_ns),
            "action_latency": stats(self.action_ns),
        }

    def reset(self) -> None:
        self.__init__()  # type: ignore[misc]


class HardwareLink(ABC):
    """Abstract CPU <-> accelerator transport."""

    name = "abstract"

    def __init__(self) -> None:
        self.stats = LinkStats()

    # -- lifecycle ---------------------------------------------------------
    def open(self) -> None:  # pragma: no cover - trivial
        return None

    def close(self) -> None:  # pragma: no cover - trivial
        return None

    # -- data path ---------------------------------------------------------
    @abstractmethod
    def upload_state(self, frame: bytes) -> None:
        """Push ``S_t`` (one protocol frame) to the accelerator input buffer."""

    @abstractmethod
    def fetch_prediction(self) -> bytes:
        """Read the accelerator's prediction output buffer (protocol frame)."""

    @abstractmethod
    def send_action(self, action: np.ndarray) -> None:
        """Send a decision back to the accelerator / actuator."""

    # -- convenience -------------------------------------------------------
    def roundtrip(self, frame: bytes) -> bytes:
        """upload -> fetch; the default is a synchronous request/response cycle."""
        self.upload_state(frame)
        return self.fetch_prediction()

    def describe(self) -> dict[str, Any]:
        return {"name": self.name, "stats": self.stats.summary()}


class NullLink(HardwareLink):
    """Drop everything: CPU-only simulation."""

    name = "null"

    def upload_state(self, frame: bytes) -> None:
        t0 = time.perf_counter_ns()
        self.stats.record_upload(len(frame), time.perf_counter_ns() - t0)

    def fetch_prediction(self) -> bytes:
        t0 = time.perf_counter_ns()
        self.stats.record_download(0, time.perf_counter_ns() - t0)
        return b""

    def send_action(self, action: np.ndarray) -> None:
        t0 = time.perf_counter_ns()
        self.stats.record_action(time.perf_counter_ns() - t0)


class LoopbackLink(HardwareLink):
    """In-process link with an optional pluggable accelerator function.

    ``accelerator(frame) -> frame`` models the FPGA: it receives the state frame
    and returns a prediction frame.  The default echo returns the input.
    """

    name = "loopback"

    def __init__(self, accelerator=None, prediction_factory=None) -> None:
        super().__init__()
        self.accelerator = accelerator
        self.prediction_factory = prediction_factory
        self.last_state: bytes | None = None
        self.last_action: np.ndarray | None = None
        self._pending: bytes = b""

    def upload_state(self, frame: bytes) -> None:
        t0 = time.perf_counter_ns()
        self.last_state = frame
        if self.accelerator is not None:
            self._pending = self.accelerator(frame)
        self.stats.record_upload(len(frame), time.perf_counter_ns() - t0)

    def fetch_prediction(self) -> bytes:
        t0 = time.perf_counter_ns()
        if self.prediction_factory is not None:
            self._pending = self.prediction_factory(self.last_state)
        out = self._pending
        self.stats.record_download(len(out), time.perf_counter_ns() - t0)
        return out

    def send_action(self, action: np.ndarray) -> None:
        t0 = time.perf_counter_ns()
        self.last_action = np.asarray(action, dtype=np.float64).reshape(-1)
        self.stats.record_action(time.perf_counter_ns() - t0)


class FileLink(HardwareLink):
    """Frame exchange through files - models an offline FPGA batch pipeline.

    ``upload_state`` appends a length-prefixed frame to ``tx_path``; the
    accelerator (or a human with an FPGA) writes replies to ``rx_path`` in the
    same format, and ``fetch_prediction`` pops the oldest reply.
    """

    name = "file"

    def __init__(self, tx_path: str | Path, rx_path: str | Path) -> None:
        super().__init__()
        self.tx_path = Path(tx_path)
        self.rx_path = Path(rx_path)
        self.tx_path.parent.mkdir(parents=True, exist_ok=True)
        self.rx_path.parent.mkdir(parents=True, exist_ok=True)
        self.tx_path.write_bytes(b"")
        self.rx_path.write_bytes(b"")
        self._rx_offset = 0

    @staticmethod
    def _frame(payload: bytes) -> bytes:
        return struct.pack("<I", len(payload)) + payload

    def upload_state(self, frame: bytes) -> None:
        t0 = time.perf_counter_ns()
        with self.tx_path.open("ab") as fh:
            fh.write(self._frame(frame))
        self.stats.record_upload(len(frame), time.perf_counter_ns() - t0)

    def fetch_prediction(self) -> bytes:
        t0 = time.perf_counter_ns()
        data = self.rx_path.read_bytes()
        if self._rx_offset + 4 > len(data):
            self.stats.record_download(0, time.perf_counter_ns() - t0)
            return b""
        (size,) = struct.unpack_from("<I", data, self._rx_offset)
        start = self._rx_offset + 4
        payload = data[start : start + size]
        if len(payload) != size:
            raise ProtocolError("truncated reply frame in FileLink rx buffer")
        self._rx_offset = start + size
        self.stats.record_download(size, time.perf_counter_ns() - t0)
        return payload

    def send_action(self, action: np.ndarray) -> None:
        t0 = time.perf_counter_ns()
        self.stats.record_action(time.perf_counter_ns() - t0)


class SerialLinkStub(HardwareLink):
    """Placeholder for the real MCU/FPGA serial transport (byte protocol ready)."""

    name = "serial-stub"

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 921600) -> None:
        super().__init__()
        self.port = port
        self.baud = int(baud)
        self._out: list[np.ndarray] = []

    def open(self) -> None:
        raise NotImplementedError(
            "SerialLinkStub documents the transport contract only; implement "
            "with pyserial once the FPGA board is wired up"
        )

    def upload_state(self, frame: bytes) -> None:
        t0 = time.perf_counter_ns()
        self.stats.record_upload(len(frame), time.perf_counter_ns() - t0)

    def fetch_prediction(self) -> bytes:
        return b""

    def send_action(self, action: np.ndarray) -> None:
        t0 = time.perf_counter_ns()
        self._out.append(np.asarray(action, dtype=np.float64).reshape(-1))
        self.stats.record_action(time.perf_counter_ns() - t0)


class ActionCodecLink:
    """Wrap a link so actions are exchanged as protocol action frames."""

    def __init__(self, link: HardwareLink, discrete: bool = False) -> None:
        self.link = link
        self.discrete = bool(discrete)

    def send(self, action: Any) -> None:
        if self.discrete:
            self.link.send_action(np.array([float(action)]))
        else:
            v = np.asarray(action, dtype=np.float64).reshape(-1)
            self.link.send_action(v)

    def decode(self, frame: bytes) -> np.ndarray:
        return decode_action_velocity(frame)

    @staticmethod
    def encode(vx: float, vy: float) -> bytes:
        return encode_action_velocity(vx, vy)


def make_link(kind: str = "null", **kwargs: Any) -> HardwareLink:
    table = {
        "null": NullLink,
        "loopback": LoopbackLink,
        "file": FileLink,
        "serial": SerialLinkStub,
    }
    try:
        cls = table[kind]
    except KeyError as exc:
        raise ValueError(f"unknown hardware link {kind!r}; available: {sorted(table)}") from exc
    return cls(**kwargs)  # type: ignore[arg-type]


__all__ = [
    "HardwareLink",
    "LinkStats",
    "NullLink",
    "LoopbackLink",
    "FileLink",
    "SerialLinkStub",
    "ActionCodecLink",
    "make_link",
]

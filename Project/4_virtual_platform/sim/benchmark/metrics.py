"""Timing primitives for benchmarks."""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable, Iterable

import numpy as np


@dataclass
class Stats:
    """Summary of a set of per-call durations (seconds)."""

    count: int = 0
    total: float = 0.0
    mean: float = 0.0
    std: float = 0.0
    minimum: float = 0.0
    p50: float = 0.0
    p90: float = 0.0
    p99: float = 0.0
    maximum: float = 0.0

    @classmethod
    def from_samples(cls, samples: Iterable[float]) -> "Stats":
        a = np.asarray(list(samples), dtype=np.float64)
        if a.size == 0:
            return cls()
        return cls(
            count=int(a.size),
            total=float(a.sum()),
            mean=float(a.mean()),
            std=float(a.std()),
            minimum=float(a.min()),
            p50=float(np.percentile(a, 50)),
            p90=float(np.percentile(a, 90)),
            p99=float(np.percentile(a, 99)),
            maximum=float(a.max()),
        )

    def to_dict(self, unit: str = "ms") -> dict[str, float]:
        scale = {"s": 1.0, "ms": 1e3, "us": 1e6}[unit]
        return {
            "count": self.count,
            f"total_{unit}": self.total * scale,
            f"mean_{unit}": self.mean * scale,
            f"std_{unit}": self.std * scale,
            f"min_{unit}": self.minimum * scale,
            f"p50_{unit}": self.p50 * scale,
            f"p90_{unit}": self.p90 * scale,
            f"p99_{unit}": self.p99 * scale,
            f"max_{unit}": self.maximum * scale,
        }

    def rate(self, per: float = 1.0) -> float:
        """Throughput in units per second (``1/mean`` scaled by ``per``)."""
        return float(per) / self.mean if self.mean > 0 else float("inf")


def timeit(fn: Callable[[], Any], *, repeat: int = 30, warmup: int = 3) -> Stats:
    """Measure per-call wall time of ``fn`` over ``repeat`` timed runs."""
    for _ in range(max(0, int(warmup))):
        fn()
    samples = []
    for _ in range(max(1, int(repeat))):
        t0 = time.perf_counter()
        fn()
        samples.append(time.perf_counter() - t0)
    return Stats.from_samples(samples)


class Stopwatch:
    """Context manager accumulating elapsed time."""

    __slots__ = ("elapsed", "_t0")

    def __init__(self) -> None:
        self.elapsed = 0.0
        self._t0 = 0.0

    def __enter__(self) -> "Stopwatch":
        self._t0 = time.perf_counter()
        return self

    def __exit__(self, *exc: Any) -> None:
        self.elapsed += time.perf_counter() - self._t0


@dataclass
class BenchmarkResult:
    name: str
    metrics: dict[str, Any] = field(default_factory=dict)
    notes: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {"name": self.name, "metrics": self.metrics, "notes": self.notes}

    def format(self) -> str:
        pairs = ", ".join(
            f"{k}={v:.4g}" if isinstance(v, float) else f"{k}={v}"
            for k, v in self.metrics.items()
        )
        return f"{self.name}: {pairs}"


def format_table(rows: list[dict[str, Any]], columns: list[str] | None = None) -> str:
    """Render a list of flat dicts as an aligned text table."""
    if not rows:
        return "(no rows)"
    cols = columns or list(rows[0].keys())
    widths = {c: max(len(str(c)), *(len(_fmt(r.get(c))) for r in rows)) for c in cols}
    header = "  ".join(str(c).rjust(widths[c]) for c in cols)
    sep = "  ".join("-" * widths[c] for c in cols)
    body = "\n".join("  ".join(_fmt(r.get(c)).rjust(widths[c]) for c in cols) for r in rows)
    return f"{header}\n{sep}\n{body}"


def _fmt(value: Any) -> str:
    if value is None:
        return "-"
    if isinstance(value, float):
        return f"{value:.4g}"
    return str(value)


__all__ = ["Stats", "timeit", "Stopwatch", "BenchmarkResult", "format_table"]

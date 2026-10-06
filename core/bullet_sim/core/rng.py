"""Deterministic seed management.

Every source of randomness in the platform derives from an explicit master
seed.  Named sub-streams are produced by *hashing* (seed, stream-name) so that:

* adding a new stream never perturbs existing streams, and
* the same stream name always yields the same generator for a given seed.

The global ``numpy.random`` API is never used anywhere in this package.
"""

from __future__ import annotations

import hashlib
from typing import Dict

import numpy as np

_MASK64 = 0xFFFFFFFFFFFFFFFF


def derive_seed(master_seed: int, stream: str) -> int:
    """Deterministically derive a 64-bit sub-seed from ``master_seed`` + name."""
    h = hashlib.blake2b(digest_size=8)
    h.update(int(master_seed).to_bytes(8, "little", signed=False))
    h.update(b"\x00")
    h.update(stream.encode("utf-8"))
    return int.from_bytes(h.digest(), "little")


class SeedManager:
    """Owns the master seed and hands out named, reproducible generators."""

    __slots__ = ("_master", "_streams")

    def __init__(self, master_seed: int) -> None:
        self._master = int(master_seed) & _MASK64
        self._streams: Dict[str, np.random.Generator] = {}

    @property
    def master_seed(self) -> int:
        return self._master

    def generator(self, stream: str) -> np.random.Generator:
        """Return (and cache) the generator for a named stream."""
        gen = self._streams.get(stream)
        if gen is None:
            gen = np.random.Generator(np.random.PCG64(derive_seed(self._master, stream)))
            self._streams[stream] = gen
        return gen

    def reset_stream(self, stream: str) -> None:
        """Drop a cached stream so the next access restarts it deterministically."""
        self._streams.pop(stream, None)

    def spawn(self, child: str) -> "SeedManager":
        """Derive an independent child manager (for parallel scenarios)."""
        return SeedManager(derive_seed(self._master, f"child:{child}"))

    def snapshot(self) -> Dict[str, object]:
        """Serializable state of every *used* stream (for exact clone/resume)."""
        return {
            name: gen.bit_generator.state for name, gen in self._streams.items()
        }

    def restore(self, state: Dict[str, object] | None) -> None:
        if not state:
            return
        for name, st in state.items():
            gen = self.generator(name)
            gen.bit_generator.state = st

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return f"SeedManager(master_seed={self._master}, streams={sorted(self._streams)})"

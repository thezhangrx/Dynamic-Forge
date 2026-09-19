"""Dataset readers.

``load_episode`` dispatches on file suffix; every reader returns the same
:class:`~bullet_sim.dataset.schema.Episode` container so downstream code never
branches on the on-disk format.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Iterable, Iterator, Sequence

import numpy as np

from bullet_sim.core.errors import DatasetError
from bullet_sim.dataset.schema import Episode
from bullet_sim.dataset.writers import (
    load_binary,
    load_csv,
    load_json,
    load_npz,
    supported_formats,
)

_READERS = {
    ".json": load_json,
    ".npz": load_npz,
    ".csv": load_csv,
    ".bin": load_binary,
    ".dat": load_binary,
}


def load_episode(path: str | Path) -> Episode:
    p = Path(path)
    if not p.exists():
        raise DatasetError(f"dataset not found: {p}")
    reader = _READERS.get(p.suffix.lower())
    if reader is None:
        raise DatasetError(
            f"unsupported dataset suffix {p.suffix!r}; supported: {supported_formats()}"
        )
    return reader(p)


def load_many(paths: Iterable[str | Path]) -> list[Episode]:
    return [load_episode(p) for p in paths]


def iter_transitions(episodes: Sequence[Episode]) -> Iterator[dict[str, Any]]:
    """Stream every transition of every episode (memory-friendly for big sets)."""
    for ep in episodes:
        for t in range(ep.steps):
            yield ep.transition(t)


def stack_states(episodes: Sequence[Episode], t: int = 0) -> np.ndarray:
    """Stack ``S_t`` player vectors across episodes - a quick ML feature matrix."""
    return np.stack([ep.arrays["player"][t] for ep in episodes]) if episodes else np.zeros((0, 7))


def verify_roundtrip(episode: Episode, path: str | Path, *, atol: float = 0.0) -> bool:
    """True when the on-disk file reproduces every array of ``episode``.

    Not applicable to CSV (a deliberately lossy scalar view).
    """
    p = Path(path)
    loaded = load_episode(p)
    if p.suffix.lower() == ".csv":
        return bool(
            np.allclose(
                loaded.arrays["rewards"], episode.arrays["rewards"], atol=max(atol, 1e-6)
            )
        )
    for key, arr in episode.arrays.items():
        if key == "state_hashes" and arr.size == 0:
            continue
        other = loaded.arrays.get(key)
        if other is None:
            raise DatasetError(f"round-trip lost array {key!r}")
        if arr.dtype.kind == "f":
            if not np.allclose(arr, other, atol=atol, rtol=0.0):
                return False
        elif arr.dtype.kind in "iub":
            if not np.array_equal(np.asarray(arr, dtype=other.dtype), other):
                return False
        else:
            if not np.array_equal(arr, other):
                return False
    return True


__all__ = [
    "load_episode",
    "load_many",
    "iter_transitions",
    "stack_states",
    "verify_roundtrip",
    "supported_formats",
]

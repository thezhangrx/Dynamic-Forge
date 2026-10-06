"""Numerical policy: dtypes, canonical hashing, deterministic helpers.

The simulator defaults to ``float64`` because cross-run, cross-platform bit
exactness is a hard requirement.  ``float32`` is supported for FPGA-aligned
data paths (see :func:`float_dtype`) and is documented as *not* bit comparable
with float64 runs.
"""

from __future__ import annotations

import hashlib
from typing import Any, Iterable

import numpy as np

# --------------------------------------------------------------------------
# dtype policy
# --------------------------------------------------------------------------

FLOAT64 = np.dtype(np.float64)
FLOAT32 = np.dtype(np.float32)

#: Default float dtype for simulator state.
DEFAULT_FLOAT_DTYPE = FLOAT64

_FLOAT_DTYPES = {"float64": FLOAT64, "float32": FLOAT32, "f8": FLOAT64, "f4": FLOAT32}

#: Integer dtype used for ids, counters and step indices (kept stable for the
#: flat hardware protocol).
INT_DTYPE = np.dtype(np.int32)
#: Small integer dtype for bullet type / group identifiers.
TYPE_DTYPE = np.dtype(np.int16)


def float_dtype(value: Any = None) -> np.dtype:
    """Resolve a user supplied float dtype specifier (``None`` -> default)."""
    if value is None:
        return DEFAULT_FLOAT_DTYPE
    if isinstance(value, np.dtype):
        if value not in (FLOAT64, FLOAT32):
            raise ValueError(f"unsupported float dtype: {value!r}")
        return value
    if value is np.float64:
        return FLOAT64
    if value is np.float32:
        return FLOAT32
    if isinstance(value, type) and issubclass(value, np.floating):
        raise ValueError(f"unsupported float dtype: {value!r}")
    key = str(value).lower()
    if key not in _FLOAT_DTYPES:
        raise ValueError(f"unsupported float dtype: {value!r}")
    return _FLOAT_DTYPES[key]


# --------------------------------------------------------------------------
# canonical hashing
# --------------------------------------------------------------------------

_CHUNK = 1 << 20


def _feed(hasher: "hashlib._Hash", arr: np.ndarray, dtype: np.dtype) -> None:
    """Feed an array to the hasher in a canonical (C-contiguous, fixed dtype) form."""
    a = np.ascontiguousarray(arr, dtype=dtype)
    flat = a.reshape(-1)
    # Chunked to bound peak memory for very large arrays.
    if flat.nbytes <= _CHUNK:
        hasher.update(flat.tobytes())
        return
    for start in range(0, flat.size, _CHUNK // flat.dtype.itemsize):
        hasher.update(flat[start : start + _CHUNK // flat.dtype.itemsize].tobytes())


def canonical_hash(parts: Iterable[tuple[np.ndarray, np.dtype]]) -> str:
    """Order-sensitive hash over ``(array, dtype)`` pairs -> hex digest.

    Arrays are cast to an explicit dtype first so that the digest does not
    depend on memory layout or endianness of the caller's arrays.
    """
    hasher = hashlib.blake2b(digest_size=16)
    for arr, dtype in parts:
        _feed(hasher, arr, dtype)
    return hasher.hexdigest()


# --------------------------------------------------------------------------
# misc
# --------------------------------------------------------------------------


def rng_generator(seed: int | None) -> np.random.Generator:
    """Create an explicitly seeded PCG64 generator (never the global RNG)."""
    if seed is None:
        raise ValueError("seed must be an explicit integer for reproducibility")
    return np.random.Generator(np.random.PCG64(int(seed) & 0xFFFFFFFFFFFFFFFF))


def as_c_contiguous(arr: np.ndarray, dtype: np.dtype | None = None) -> np.ndarray:
    """Return a C-contiguous array, copying only when required."""
    if dtype is not None:
        arr = arr.astype(dtype, copy=False)
    if not arr.flags["C_CONTIGUOUS"]:
        arr = np.ascontiguousarray(arr)
    return arr


def clip_norm(vec: np.ndarray, max_norm: float) -> np.ndarray:
    """Scale ``vec`` so its norm never exceeds ``max_norm`` (no-op if smaller)."""
    n = float(np.linalg.norm(vec))
    if n > max_norm and n > 0.0:
        return vec * (max_norm / n)
    return vec

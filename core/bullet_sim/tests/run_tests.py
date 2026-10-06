"""Dependency-free fallback test runner.

    python -m bullet_sim.tests.run_tests          # run everything
    python -m bullet_sim.tests.run_tests motion   # only tests/test_motion.py

If :mod:`pytest` is installed it is used directly (``pytest -q``).  Otherwise a
minimal shim provides the three pytest features the suite relies on
(``approx``, ``raises``, ``mark.parametrize``), so the platform has **no
mandatory test dependency** beyond NumPy.
"""

from __future__ import annotations

import importlib
import inspect
import math
import os
import pkgutil
import sys
import traceback
import types
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np


# --------------------------------------------------------------------------
# minimal pytest shim
# --------------------------------------------------------------------------


class _Approx:
    # Tell NumPy to hand elementwise comparisons back to us instead of
    # broadcasting an object array (mirrors pytest.approx's behaviour).
    __array_ufunc__ = None
    __array_priority__ = 1000.0

    def __init__(self, expected: Any, rel: float | None = None, abs: float | None = None) -> None:
        self.expected = expected
        self.rel = rel
        self.abs = abs

    def _cmp(self, other: Any) -> bool:
        return bool(
            np.allclose(
                np.asarray(self.expected, dtype=np.float64),
                np.asarray(other, dtype=np.float64),
                rtol=self.rel if self.rel is not None else 1e-7,
                atol=self.abs if self.abs is not None else 0.0,
                equal_nan=False,
            )
        )

    def __eq__(self, other: Any) -> bool:  # type: ignore[override]
        return self._cmp(other)

    def __req__(self, other: Any) -> bool:
        return self._cmp(other)

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return f"approx({self.expected!r}, rel={self.rel}, abs={self.abs})"


class _Raises:
    def __init__(self, exc: Any, match: str | None = None) -> None:
        self.exc = exc
        self.match = match
        self.value: BaseException | None = None

    def __enter__(self) -> "_Raises":
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if exc_type is None:
            raise AssertionError(f"DID NOT RAISE {self.exc}")
        if not issubclass(exc_type, self.exc):
            return False
        self.value = exc
        if self.match is not None:
            import re

            if not re.search(self.match, str(exc)):
                raise AssertionError(f"pattern {self.match!r} not found in {exc!r}")
        return True


class _Mark:
    def parametrize(self, argnames: Any, argvalues: Any, **_: Any):
        names = [a.strip() for a in argnames.split(",")] if isinstance(argnames, str) else list(argnames)

        def deco(fn: Callable[..., Any]) -> Callable[..., Any]:
            existing = getattr(fn, "_parametrize", [])
            fn._parametrize = list(existing) + [(names, list(argvalues))]  # type: ignore[attr-defined]
            return fn

        return deco

    def __getattr__(self, item: str) -> Callable[..., Any]:
        def deco(*_a: Any, **_kw: Any):
            def inner(fn: Callable[..., Any]) -> Callable[..., Any]:
                return fn

            return inner

        return deco


def _install_shim() -> None:
    mod = types.ModuleType("pytest")
    mod.approx = lambda expected, rel=None, abs=None: _Approx(expected, rel, abs)  # type: ignore[attr-defined]
    mod.raises = lambda exc, match=None: _Raises(exc, match)  # type: ignore[attr-defined]
    mod.mark = _Mark()  # type: ignore[attr-defined]
    mod.skip = lambda *a, **k: None  # type: ignore[attr-defined]
    mod.fail = lambda msg="": (_ for _ in ()).throw(AssertionError(msg))  # type: ignore[attr-defined]
    mod.fixture = lambda *a, **k: (lambda fn: fn)  # type: ignore[attr-defined]
    sys.modules.setdefault("pytest", mod)


def _expand(fn: Callable[..., Any]) -> list[tuple[str, dict[str, Any]]]:
    """Expand ``mark.parametrize`` decorators into ``{argname: value}`` cases."""
    specs = getattr(fn, "_parametrize", None)
    if not specs:
        return [("", {})]
    cases: list[tuple[str, dict[str, Any]]] = [("", {})]
    for names, values in specs:
        new: list[tuple[str, dict[str, Any]]] = []
        for label, mapping in cases:
            for entry in values:
                vals = tuple(entry) if isinstance(entry, (tuple, list)) else (entry,)
                merged = dict(mapping)
                for name, value in zip(names, vals):
                    merged[name] = value
                ident = "-".join(repr(v) for v in vals)
                new.append((f"{label}[{ident}]", merged))
        cases = new
    return cases


# --------------------------------------------------------------------------
# runner
# --------------------------------------------------------------------------


def discover(selectors: Iterable[str] = ()) -> list[str]:
    here = Path(__file__).parent
    names = []
    for mod in pkgutil.iter_modules([str(here)]):
        if not mod.name.startswith("test_"):
            continue
        if selectors and not any(s in mod.name for s in selectors):
            continue
        names.append(f"bullet_sim.tests.{mod.name}")
    return sorted(names)


def run(selectors: Iterable[str] = ()) -> int:
    import tempfile

    modules = discover(selectors)
    passed = failed = 0
    failures: list[tuple[str, str]] = []
    tmp_root = Path(tempfile.mkdtemp(prefix="bullet_sim_tests_"))
    for modname in modules:
        module = importlib.import_module(modname)
        for fname in sorted(vars(module)):
            if not fname.startswith("test_"):
                continue
            fn = getattr(module, fname)
            if not callable(fn):
                continue
            params = list(inspect.signature(fn).parameters)
            for index, (label, mapping) in enumerate(_expand(fn)):
                display = f"{modname}::{fname}{label}"
                call_args: list[Any] = []
                # minimal fixture support: `tmp_path` (a fresh directory)
                for pname in params:
                    if pname in mapping:
                        call_args.append(mapping[pname])
                    elif pname == "tmp_path":
                        d = tmp_root / f"{modname.split('.')[-1]}_{fname}_{index}"
                        d.mkdir(parents=True, exist_ok=True)
                        call_args.append(d)
                    else:
                        call_args.append(None)
                try:
                    fn(*call_args)
                    passed += 1
                except Exception:
                    failed += 1
                    failures.append((display, traceback.format_exc()))
        print(f"  {modname}  ok")
    print()
    for name, tb in failures:
        print("=" * 72)
        print("FAILED:", name)
        print(tb)
    print("=" * 72)
    print(f"{passed} passed, {failed} failed")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    force_fallback = bool(os.environ.get("BULLET_SIM_NO_PYTEST"))
    try:
        if force_fallback:
            raise ImportError("forced fallback")
        import pytest

        args = ["-q", str(Path(__file__).parent)]
        if argv:
            args += ["-k", " or ".join(argv)]
        return int(pytest.main(args))
    except ImportError:
        _install_shim()
        importlib.invalidate_caches()
        return run(argv)


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())

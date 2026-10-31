"""One debug log file per platform launch.

Every time the CLI is started it writes a single file into the module's data
folder, named with a date stamp:

    data/bullet_sim/debug_YYYYmmdd_HHMMSS.log

The file contains

1. a **header** - when, which command, the full argv, the working directory,
   the interpreter, and the versions of numpy / pygame / bullet_sim;
2. a **verbatim copy** of everything the run prints to stdout/stderr (stdout
   itself is untouched, so machine-readable output stays machine-readable);
3. a **footer** - end time, wall-clock duration and the exit status, plus the
   traceback if the run died.

Controls
--------
* ``BULLET_SIM_DEBUG_LOG=0`` disables the log entirely.
* ``BULLET_SIM_DATA_DIR=<path>`` overrides where the log is written.
* Under pytest the log is skipped automatically (a test run is not a platform
  launch); ``BULLET_SIM_DEBUG_LOG=1`` forces it on anyway.
"""

from __future__ import annotations

import importlib.util
import io
import os
import platform
import sys
import time
import traceback
from pathlib import Path
from typing import Any, Sequence

#: ``0`` / ``false`` / ``off`` disables the per-launch debug log.
DEBUG_LOG_ENV = "BULLET_SIM_DEBUG_LOG"
#: Optional override of the output directory.
DATA_DIR_ENV = "BULLET_SIM_DATA_DIR"
#: Date stamp embedded in the file name.
STAMP_FORMAT = "%Y%m%d_%H%M%S"

_OFF = {"0", "false", "no", "off", ""}


def repo_root() -> Path | None:
    """The source checkout root, or ``None`` when running from an install."""
    # core/bullet_sim/debug_log.py -> parents[2] == <repo root>
    root = Path(__file__).resolve().parents[2]
    return root if (root / "core" / "bullet_sim").is_dir() else None


def data_dir() -> Path:
    """Directory the log belongs to: ``<repo>/data/bullet_sim``."""
    override = os.environ.get(DATA_DIR_ENV)
    if override:
        return Path(override).expanduser()
    root = repo_root()
    if root is not None:
        return root / "data" / "bullet_sim"
    return Path.cwd() / "data" / "bullet_sim"


def enabled() -> bool:
    """Whether a launch should write a debug log."""
    flag = os.environ.get(DEBUG_LOG_ENV)
    if flag is not None:
        return flag.strip().lower() not in _OFF
    # A test run is not a platform launch: keep the data folder clean.
    return "PYTEST_CURRENT_TEST" not in os.environ


class _Tee(io.TextIOBase):
    """A write-through copy of a text stream (stdout/stderr keep working)."""

    def __init__(self, stream: Any, mirror: Any) -> None:
        self._stream = stream
        self._mirror = mirror

    def write(self, data: str) -> int:  # type: ignore[override]
        self._mirror.write(data)
        self._mirror.flush()
        return self._stream.write(data)

    def flush(self) -> None:  # type: ignore[override]
        try:
            self._stream.flush()
        finally:
            self._mirror.flush()

    def writable(self) -> bool:  # type: ignore[override]
        return True

    def isatty(self) -> bool:  # type: ignore[override]
        return False


class DebugLog:
    """Context manager that mirrors one launch's console output to a file."""

    def __init__(self, path: Path, command: str, argv: Sequence[str]) -> None:
        self.path = path
        self.command = command
        self.argv = list(argv)
        #: Set by the caller before leaving the ``with`` block.
        self.exit_code: int | None = None
        self.started_at = time.time()
        self._fh = None
        self._saved: tuple[Any, Any] | None = None

    # -- context manager -------------------------------------------------
    def __enter__(self) -> "DebugLog":
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fh = self.path.open("w", encoding="utf-8")
        self._write_header()
        self._saved = (sys.stdout, sys.stderr)
        sys.stdout = _Tee(self._saved[0], self._fh)
        sys.stderr = _Tee(self._saved[1], self._fh)
        return self

    def __exit__(self, exc_type, exc, tb) -> bool:
        if self._saved is not None:
            sys.stdout, sys.stderr = self._saved
        try:
            if exc_type is not None and not issubclass(exc_type, KeyboardInterrupt):
                self._fh.write("\n--- traceback ---\n")
                traceback.print_exception(exc_type, exc, tb, file=self._fh)
            self._write_footer(exc_type)
        finally:
            self._fh.flush()
            self._fh.close()
        return False  # never swallow the exception

    # -- content ---------------------------------------------------------
    def _write_header(self) -> None:
        from bullet_sim.core.version import STATE_PROTOCOL_VERSION, __version__

        root = repo_root()
        pygame_state = "not installed"
        if importlib.util.find_spec("pygame") is not None:
            try:
                import pygame  # noqa: PLC0415  (optional dependency)

                pygame_state = f"{pygame.version.ver} (available)"
            except Exception:  # pragma: no cover - environment dependent
                pygame_state = "installed but not importable"
        try:
            import numpy  # noqa: PLC0415

            numpy_version = numpy.__version__
        except Exception:  # pragma: no cover - numpy is a hard dependency
            numpy_version = "missing"

        fh = self._fh
        fh.write("bullet_sim debug log\n")
        fh.write("===================\n")
        fh.write(f"started     : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        fh.write(f"command     : {self.command}\n")
        fh.write(f"argv        : {' '.join(self.argv)}\n")
        fh.write(f"cwd         : {os.getcwd()}\n")
        fh.write(f"repo root   : {root if root is not None else '(not a source checkout)'}\n")
        fh.write(f"data dir    : {data_dir()}\n")
        fh.write(f"python      : {platform.python_version()} ({platform.python_implementation()}) "
                 f"at {sys.executable}\n")
        fh.write(f"platform    : {platform.platform()}\n")
        fh.write(f"numpy       : {numpy_version}\n")
        fh.write(f"pygame      : {pygame_state}\n")
        fh.write(f"bullet_sim  : {__version__} (state protocol v{STATE_PROTOCOL_VERSION})\n")
        fh.write("-" * 60 + " output " + "-" * 60 + "\n")
        fh.flush()

    def _write_footer(self, exc_type: Any) -> None:
        elapsed = time.time() - self.started_at
        fh = self._fh
        fh.write("\n" + "-" * 60 + " end " + "-" * 61 + "\n")
        fh.write(f"finished    : {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        fh.write(f"duration    : {elapsed:.2f} s\n")
        if exc_type is not None:
            fh.write(f"exit status : raised {exc_type.__name__}\n")
        elif self.exit_code is None:
            fh.write("exit status : unknown\n")
        else:
            fh.write(f"exit status : {self.exit_code}\n")


def start(command: str, argv: Sequence[str] | None = None) -> DebugLog | None:
    """Open the debug log for one launch, or ``None`` when logging is off.

    The file name carries a date stamp (``debug_YYYYmmdd_HHMMSS.log``); if that
    name is already taken - two launches inside the same second - a numeric
    suffix is appended instead of overwriting the earlier log.
    """
    if not enabled():
        return None
    directory = data_dir()
    directory.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime(STAMP_FORMAT)
    suffix = 0
    while True:
        name = f"debug_{stamp}.log" if suffix == 0 else f"debug_{stamp}_{suffix}.log"
        path = directory / name
        try:
            # Reserve the name immediately, so two launches inside the same
            # second can never end up pointing at the same file.
            path.touch(exist_ok=False)
        except FileExistsError:
            suffix += 1
            continue
        break
    return DebugLog(path, command, list(argv) if argv is not None else sys.argv[1:])


__all__ = [
    "DEBUG_LOG_ENV",
    "DATA_DIR_ENV",
    "STAMP_FORMAT",
    "DebugLog",
    "data_dir",
    "enabled",
    "repo_root",
    "start",
]

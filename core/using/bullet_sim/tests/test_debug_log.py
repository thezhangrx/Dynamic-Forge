"""The per-launch debug log (``core/bullet_sim/debug_log.py``).

Covers the contract the CLI relies on: one date-stamped file per launch, the
run's console output mirrored into it, a footer with the exit status, and the
env-var switches (including the automatic skip under pytest).
"""

from __future__ import annotations

import re
from pathlib import Path

from bullet_sim import debug_log

_NAME_RE = re.compile(r"^debug_\d{8}_\d{6}(_\d+)?\.log$")


def test_switched_off_by_env_var(monkeypatch):
    monkeypatch.setenv(debug_log.DEBUG_LOG_ENV, "0")
    assert debug_log.enabled() is False
    assert debug_log.start("info") is None


def test_env_var_overrides_the_pytest_skip(monkeypatch):
    monkeypatch.setenv("PYTEST_CURRENT_TEST", "test_x")
    monkeypatch.delenv(debug_log.DEBUG_LOG_ENV, raising=False)
    # a test run is not a platform launch
    assert debug_log.enabled() is False
    # ... unless explicitly forced
    monkeypatch.setenv(debug_log.DEBUG_LOG_ENV, "1")
    assert debug_log.enabled() is True


def test_log_has_a_date_stamp_and_captures_the_run(monkeypatch, tmp_path: Path):
    monkeypatch.setenv(debug_log.DEBUG_LOG_ENV, "1")
    monkeypatch.setenv(debug_log.DATA_DIR_ENV, str(tmp_path))

    log = debug_log.start("info", ["info", "--x"])
    assert log is not None
    with log:
        print("hello from the run")
        log.exit_code = 0

    assert log.path.parent == tmp_path
    assert _NAME_RE.match(log.path.name), log.path.name

    text = log.path.read_text(encoding="utf-8")
    assert "bullet_sim debug log" in text          # header
    assert "command     : info" in text
    assert "hello from the run" in text            # stdout mirrored verbatim
    assert "exit status : 0" in text               # footer


def test_a_second_launch_in_the_same_second_does_not_overwrite(monkeypatch, tmp_path: Path):
    monkeypatch.setenv(debug_log.DEBUG_LOG_ENV, "1")
    monkeypatch.setenv(debug_log.DATA_DIR_ENV, str(tmp_path))

    first = debug_log.start("info")
    second = debug_log.start("info")
    assert first is not None and second is not None
    assert first.path != second.path
    assert _NAME_RE.match(second.path.name), second.path.name


def test_raised_error_is_recorded_as_the_exit_status(monkeypatch, tmp_path: Path):
    monkeypatch.setenv(debug_log.DEBUG_LOG_ENV, "1")
    monkeypatch.setenv(debug_log.DATA_DIR_ENV, str(tmp_path))

    log = debug_log.start("run", ["run"])
    assert log is not None
    try:
        with log:
            raise ValueError("boom")
    except ValueError:
        pass
    text = log.path.read_text(encoding="utf-8")
    assert "ValueError" in text
    assert "boom" in text
    assert "raised ValueError" in text

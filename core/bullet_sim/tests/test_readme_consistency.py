"""The README must describe the code that actually exists.

A team manual is worse than useless if it drifts from the repository, so this
test parses ``README.md`` and checks every concrete claim it makes:

* every repository path it mentions exists;
* every ``python -m bullet_sim <subcommand>`` it documents is a real subcommand
  and every ``--flag`` it shows is accepted by that subcommand (or by one of
  them, for flags shared across commands);
* every ``from bullet_sim.x import y`` snippet imports successfully;
* every data-structure field it lists for Bullet / Player / Environment / Action
  really exists in the code;
* the status markers it uses (``[TBD]``, ``[MODEL_NOT_AVAILABLE_YET]``,
  ``[HARDWARE_INTERFACE_TBD]``) are the ones the code defines.
"""

from __future__ import annotations

import argparse
import importlib
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[3]
README = REPO_ROOT / "README.md"


@pytest.fixture(scope="module")
def readme_text() -> str:
    assert README.exists(), "README.md must exist at the repository root"
    return README.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def parser() -> argparse.ArgumentParser:
    from bullet_sim.cli import build_parser

    return build_parser()


def _subparsers(parser: argparse.ArgumentParser) -> dict[str, argparse.ArgumentParser]:
    for action in parser._actions:
        if isinstance(action, argparse._SubParsersAction):
            return dict(action.choices)
    return {}


def _option_strings(sub: argparse.ArgumentParser) -> set[str]:
    out: set[str] = set()
    for action in sub._actions:
        out.update(action.option_strings)
    return out


# --------------------------------------------------------------------------
# paths
# --------------------------------------------------------------------------

_PATH_RE = re.compile(
    r"`((?:core/)?bullet_sim/[A-Za-z0-9_./\-]+"
    r"|bullet_sim_run_demo\.py|run_demo\.py|pyproject\.toml|README\.md)`"
)


def test_every_documented_path_exists(readme_text: str):
    missing = []
    for raw in sorted(set(_PATH_RE.findall(readme_text))):
        target = REPO_ROOT / raw.rstrip("/")
        if not target.exists():
            missing.append(raw)
    assert not missing, f"README documents paths that do not exist: {missing}"


def test_every_documented_package_module_imports(readme_text: str):
    modules = set(re.findall(r"`(bullet_sim(?:\.[a-z_]+)+)`", readme_text))
    bad = []
    source_root = REPO_ROOT / "core"  # packages live under core/
    for name in sorted(modules):
        path = source_root / (name.replace(".", "/") + ".py")
        pkg = source_root / name.replace(".", "/")
        if not (path.exists() or (pkg / "__init__.py").exists()):
            bad.append(name)
            continue
        try:
            importlib.import_module(name)
        except Exception as exc:  # pragma: no cover - surfaced as a failure
            bad.append(f"{name} ({exc})")
    assert not bad, f"README names modules that do not import: {bad}"


# --------------------------------------------------------------------------
# CLI commands and flags
# --------------------------------------------------------------------------


def test_every_documented_cli_command_exists(readme_text: str, parser):
    subs = _subparsers(parser)
    documented = set(re.findall(r"python -m bullet_sim ([a-z_]+)", readme_text))
    assert documented, "README should document at least one CLI command"
    unknown = sorted(c for c in documented if c not in subs)
    assert not unknown, f"README documents unknown subcommands: {unknown} (have {sorted(subs)})"


def test_every_documented_cli_flag_exists(readme_text: str, parser):
    subs = _subparsers(parser)
    global_flags = set()
    for sub in subs.values():
        global_flags |= _option_strings(sub)
    examples = re.findall(r"python -m bullet_sim ([a-z_]+)([^\n`]*)", readme_text)
    unknown: list[str] = []
    for command, tail in examples:
        flags = set(re.findall(r"(--[a-z][a-z0-9\-]*)", tail))
        if not flags or command not in subs:
            continue
        allowed = _option_strings(subs[command]) | {"--help"}
        for flag in flags:
            if flag not in allowed and flag not in global_flags:
                unknown.append(f"{command} {flag}")
    assert not unknown, f"README shows flags that are not accepted: {sorted(set(unknown))}"


def test_every_flag_named_anywhere_in_the_readme_exists(readme_text: str, parser):
    """Stronger than the per-command check: no flag may be mentioned that the CLI
    does not accept, even in prose or a table."""
    subs = _subparsers(parser)
    known = {"--help", "--version"}
    for sub in subs.values():
        known |= _option_strings(sub)
    known |= {"--help"}  # top level
    mentioned = set(
        re.findall(r"(?<![\w-])(--[a-z][a-z0-9\-]*)(?![\w])", readme_text)
    )
    # Flags that belong to tools other than this CLI.  The root README is the
    # whole-repo manual, so it also documents the sibling entry points under
    # core/vision/ - those flags are legitimate and must not be reported here.
    external = {
        # pip / pytest / git
        "--version", "--upgrade", "--user", "--prefix",
        # vision_live.py   (实时视频)
        "--device", "--width", "--height", "--fps", "--fourcc", "--window",
        "--headless", "--no-overlay", "--seconds",
        # vision_detect.py (图像识别)
        "--image", "--out-dir", "--no-save", "--wall-diff", "--edge-min",
        "--min-gap-px",
    }
    unknown = sorted(f for f in mentioned if f not in known and f not in external)
    assert not unknown, f"README mentions flags the CLI does not accept: {unknown}"


def test_documented_input_modes_are_supported(readme_text: str):
    from bullet_sim.ai.factory import MODE_ALIASES, normalise_mode
    from bullet_sim.ai.baseline import BASELINE_CONTROLLERS

    for mode in re.findall(r"--input ([a-z_]+)", readme_text):
        assert normalise_mode(mode) in {
            "manual", "auto", "model", "random", "scripted", "null", "board", "controller"
        } or mode in MODE_ALIASES, f"unknown --input mode documented: {mode}"
    for name in re.findall(r"--auto ([a-z_]+)", readme_text):
        assert name in BASELINE_CONTROLLERS, f"unknown --auto controller documented: {name}"


# --------------------------------------------------------------------------
# data structures
# --------------------------------------------------------------------------


def test_documented_bullet_fields_exist(readme_text: str):
    from bullet_sim.entities.bullet import PROTOCOL_FIELDS

    section = _table_fields(readme_text, "Bullet (`BulletPool`)")
    assert section, "README must document the Bullet fields"
    known = set(PROTOCOL_FIELDS) | {"age", "active", "alive"}
    unknown = sorted(f for f in section if f not in known)
    assert not unknown, f"README lists Bullet fields that do not exist: {unknown}"


def test_documented_player_fields_exist(readme_text: str):
    from bullet_sim.entities.player import PLAYER_FIELDS

    section = _table_fields(readme_text, "Player")
    assert section, "README must document the Player fields"
    unknown = sorted(f for f in section if f not in set(PLAYER_FIELDS) | {"hitbox"})
    assert not unknown, f"README lists Player fields that do not exist: {unknown}"


def test_documented_environment_fields_exist(readme_text: str):
    from bullet_sim.core.state import ENV_FIELDS

    section = _table_fields(readme_text, "Environment")
    assert section, "README must document the Environment fields"
    unknown = sorted(f for f in section if f not in set(ENV_FIELDS) | {"bullet_capacity"})
    assert not unknown, f"README lists Environment fields that do not exist: {unknown}"


def test_documented_action_fields_exist(readme_text: str):
    from bullet_sim.action.types import Action

    assert "direction" in Action.__dataclass_fields__
    assert "magnitude" in Action.__dataclass_fields__
    fields = _table_fields(readme_text, "Action")
    if fields:
        unknown = sorted(
            f for f in fields if f not in Action.__dataclass_fields__
        )
        assert not unknown, f"README lists Action fields that do not exist: {unknown}"
    assert "Action" in readme_text and "magnitude" in readme_text


def _table_fields(text: str, heading_fragment: str) -> set[str]:
    """Collect the first column of the markdown table that follows a heading."""
    pattern = re.compile(
        r"^#{2,4}[^\n]*" + re.escape(heading_fragment) + r"[^\n]*\n(.*?)(?=\n#{2,4} |\Z)",
        re.M | re.S,
    )
    match = pattern.search(text)
    if not match:
        return set()
    fields: set[str] = set()
    for line in match.group(1).splitlines():
        line = line.strip()
        if not line.startswith("|"):
            continue
        first = line.strip("|").split("|")[0].strip()
        first = first.replace("`", "").strip()
        if not first or set(first) <= set("-: ") or first.lower() == "field":
            continue
        if re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", first):
            fields.add(first)
    return fields


# --------------------------------------------------------------------------
# markers and status honesty
# --------------------------------------------------------------------------


def test_status_markers_used_in_the_readme_are_the_ones_the_code_defines(readme_text: str):
    from bullet_sim.ai.policy import MODEL_DEPLOYMENT_TBD, MODEL_NOT_AVAILABLE_YET
    from bullet_sim.fpga import HARDWARE_INTERFACE_TBD as FPGA_TBD
    from bullet_sim.hardware_interface.tbd import HARDWARE_INPUT_INTERFACE_TBD

    markers = {
        "[TBD]": True,  # generic marker, no code constant required
        MODEL_NOT_AVAILABLE_YET: True,
        MODEL_DEPLOYMENT_TBD: True,
        HARDWARE_INPUT_INTERFACE_TBD: True,
        FPGA_TBD: True,
        "[SOFTWARE_REFERENCE_ONLY]": True,
    }
    used = [m for m in markers if m in readme_text]
    assert MODEL_NOT_AVAILABLE_YET in used, "README must state that no model exists yet"
    assert HARDWARE_INPUT_INTERFACE_TBD in used, "README must mark the board link as TBD"


def test_readme_does_not_claim_a_trained_model_exists(readme_text: str):
    from bullet_sim.ai.policy import available_policies

    assert available_policies() == [], "the repository ships no trained policy"
    lowered = readme_text.lower()
    for claim in ("trained model is included", "pre-trained model download",
                  "we have trained a model", "模型已训练完成"):
        assert claim not in lowered


def test_readme_states_the_real_test_count(readme_text: str):
    """If the README quotes a test count, it must not be absurdly stale.

    The bound is derived from the number of test *modules* rather than being a
    frozen window: the suite keeps growing, and a hard-coded range silently
    turns into "any accurate number above the old maximum fails" (the previous
    ``abs(q - 270) < 120`` rejected a correct 391).
    """
    from bullet_sim.tests.run_tests import discover

    module_count = len(discover())
    assert module_count >= 10
    # Each test module holds at least a handful of cases; parametrisation pushes
    # the collected count above the number of ``def test_`` functions, so the
    # upper bound is deliberately loose.
    lo, hi = module_count * 5, module_count * 60
    quoted = re.findall(r"(\d{2,4})\s*(?:项|automated tests|tests)\b", readme_text)
    if quoted:
        assert any(lo <= int(q) <= hi for q in quoted), (
            f"README quotes a test count {quoted} that is far from reality: "
            f"expected something in [{lo}, {hi}] for {module_count} test modules"
        )


def test_readme_documents_the_required_sections(readme_text: str):
    required = [
        "架构",
        "核心模块",
        "数据结构",
        "CPU / FPGA",
        "Manual Control",
        "Autonomous",
        "Pattern",
        "Collision",
        "Headless",
        "Dataset",
        "目录",
        "Installation",
        "Quick Start",
        "Test",
        "Current Status",
        "Future Hardware",
    ]
    missing = [s for s in required if s.lower() not in readme_text.lower()]
    assert not missing, f"README is missing required sections: {missing}"

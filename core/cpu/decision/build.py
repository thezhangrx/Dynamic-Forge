"""Single source of truth for the dual-planner versions and build fingerprint.

# [v0.5.0-DualPlanner]
# PURPOSE:
#   Give every benchmark run a verifiable identity so the terminal output can
#   never be mistaken for a different revision, and a source edit can never
#   leave BUILD_ID stale.  v0.5.0 also carries one version per planner so the
#   reactive and predictive halves are never confused.
#
# OPEN-SOURCE REFERENCE:
#   Build-stamped embedded control images (TinyMPC-style resource-constrained
#   deployment practice): the artifact reports exactly which sources produced it.
#
# ALGORITHM:
#   SHA-256 over the exact bytes of the decision sources, in a fixed order,
#   truncated to 12 hex chars -> ``cpudec-xxxxxxxxxxxx``.  Any byte change in any
#   listed source file changes the digest.  Paths are relative to ``core/``
#   (``core/cpu/...`` -> ``cpu/...``) so the reactive baseline is covered too.
#
# FPGA MAPPING:
#   None at runtime; this is a host-side integrity stamp.  It guarantees the
#   deployed bitstream/image and the CPU planner weights came from one source tree.
#
# CPU ROLE:
#   Identity only (never part of the decision math).
#
# COMPLEXITY:
#   O(total source bytes), once per process import.
"""

from __future__ import annotations

import hashlib
from pathlib import Path
from typing import Sequence

__all__ = [
    "STACK_VERSION",
    "REACTIVE_VERSION",
    "PREDICTIVE_VERSION",
    "VERSION",
    "BUILD_ID",
    "FINGERPRINT_FILES",
    "fingerprint",
]

#: Overall revision.  v0.5.0 built a second reactive controller; that experiment
#: FAILED (collision 94.5%) and is retained for the record only.  v0.5.1 added
#: Long-Term Viability, v0.5.2 demoted it to a guarded advisory tie-break, v0.5.3
#: tried a post-safety tie-break (accepted only 0.106%, retired).  v0.6.0 upgrades
#: the Future Safe Corridor into a real *predictive selector* over the already-safe
#: candidate set (reachability > persistent sectors > t3 width > clearance >
#: existing tiers).  OFF by default (``DF_ENABLE_FSC_SELECTOR``); with it off the
#: action path is v0.5.2-LTVGuarded exactly.
STACK_VERSION = "v0.6.0-PredictiveCorridorSelector"
#: ReactiveGap controller revision (current frame only).  Kept as a documented
#: failed experiment; the benchmark continues to use ``RepulsionController``.
REACTIVE_VERSION = "v0.5.0-ReactiveGap"
#: Predictive planner revision (v0.4.1 FAR + ASE, guarded LTV in v0.5.2,
#: predictive corridor selection in v0.6.0).
PREDICTIVE_VERSION = "v0.6.0-PredictiveCorridorSelector"
#: Backward-compatible alias: the predictive decision-stack version.
VERSION = PREDICTIVE_VERSION

#: Source files whose content defines the build (repo-relative).  Order is part
#: of the contract; adding/removing a file changes every future BUILD_ID.
#: ``build.py`` itself is included, so even a version-string edit changes it, and
#: the reactive controller is covered too.
FINGERPRINT_FILES: tuple[str, ...] = (
    "cpu/decision/models.py",
    "cpu/decision/predictor.py",
    "cpu/decision/risk.py",
    "cpu/decision/cost.py",
    "cpu/decision/far.py",
    "cpu/decision/immediate.py",
    "cpu/decision/reactive.py",
    "cpu/decision/viability.py",
    "cpu/decision/future_corridor.py",
    "cpu/decision/corridor_selector.py",
    "cpu/decision/planner.py",
    "cpu/decision/core.py",
    "cpu/decision/build.py",
    "cpu/baseline_reactive_gap.py",
)

#: 源码根：build.py -> decision -> cpu -> core/（新结构下 ``cpu`` 与
#: ``bullet_sim`` 都挂在 ``core/`` 下面，所以指纹名都是 ``cpu/...``）。
_REPO_ROOT = Path(__file__).resolve().parents[2]


def fingerprint(
    root: Path | str | None = None,
    files: Sequence[str] = FINGERPRINT_FILES,
) -> str:
    """Stable ``cpudec-xxxxxxxxxxxx`` digest of ``files`` under ``root``.

    Deterministic: identical source bytes always produce the identical id, and a
    one-byte change produces a different one.  Missing files hash as empty so the
    function never raises during packaging.
    """
    base = Path(root) if root is not None else _REPO_ROOT
    digest = hashlib.sha256()
    for name in files:
        digest.update(str(name).encode("utf-8"))
        digest.update(b"\0")
        path = base / name
        digest.update(path.read_bytes() if path.is_file() else b"")
        digest.update(b"\0")
    return "cpudec-" + digest.hexdigest()[:12]


#: Real code fingerprint of this checkout (not a version-string hash).
BUILD_ID = fingerprint()


if __name__ == "__main__":  # pragma: no cover - tiny version script
    print("==================================================")
    print("Dynamic-Forge Decision")
    print(f"Stack Version:      {STACK_VERSION}")
    print(f"Reactive Version:   {REACTIVE_VERSION}")
    print(f"Predictive Version: {PREDICTIVE_VERSION}")
    print(f"Build ID:           {BUILD_ID}")
    print("==================================================")

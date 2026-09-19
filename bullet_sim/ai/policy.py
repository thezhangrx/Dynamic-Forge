"""Trained-model interface (currently ``[MODEL_NOT_AVAILABLE_YET]``).

**This repository does not contain a trained model, and nothing here pretends
otherwise.**  What it does contain is the contract a model must satisfy once
someone trains one, so that plugging it in is a one-line change:

    Observation (float32 vector)
        -> policy(obs) -> Action index (0..8)
        -> Action -> Simulator

Three loaders are supported, in increasing order of "real deployment":

``callable``
    any object with ``predict(obs, deterministic=True)`` / ``act(obs)`` /
    ``__call__(obs)`` - this is what a PyTorch / NumPy model wrapper looks like.

``.npz``
    a linear / small MLP policy stored as plain arrays (``W1,b1,W2,b2,...``).
    This is the *quantisation-friendly* form that maps onto the 30TAI / FPGA
    path, and the format the project should export to before board bring-up.

``.onnx`` / vendor runtime
    ``[TBD]`` - the exact deployment toolchain (Icraft / 30TAI) is not chosen
    yet, so no loader is guessed.  ``load_policy`` raises a message that says
    exactly which decision is still open.

Until a model exists, ``TrainedPolicyController`` refuses to be constructed and
:data:`MODEL_NOT_AVAILABLE_YET` is what the CLI and the README report.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence

import numpy as np

from bullet_sim.ai.base import BaseAutonomousController, ControllerSpec

#: Marker used everywhere the project must say "no trained model exists yet".
MODEL_NOT_AVAILABLE_YET = "[MODEL_NOT_AVAILABLE_YET]"

#: Marker for the still-undecided deployment toolchain.
MODEL_DEPLOYMENT_TBD = "[MODEL_DEPLOYMENT_TBD]"

#: Default directory a future model should be dropped into.
MODEL_DIR = Path(__file__).resolve().parents[2] / "models"


class ModelNotAvailable(RuntimeError):
    """Raised when a trained policy is requested but none exists."""


# --------------------------------------------------------------------------
# registry
# --------------------------------------------------------------------------


@dataclass
class PolicyRecord:
    """A pluggable policy: name -> loader."""

    name: str
    loader: Callable[[str | os.PathLike | None], Any]
    description: str = ""
    kind: str = "callable"
    notes: str = MODEL_NOT_AVAILABLE_YET

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "kind": self.kind,
            "notes": self.notes,
        }


class PolicyRegistry:
    """Name -> policy loader.  Empty by default, and honest about it."""

    def __init__(self) -> None:
        self._records: dict[str, PolicyRecord] = {}

    def register(self, name: str, loader: Callable[..., Any], **kwargs: Any) -> None:
        self._records[str(name)] = PolicyRecord(str(name), loader, **kwargs)

    def unregister(self, name: str) -> None:
        self._records.pop(str(name), None)

    def names(self) -> list[str]:
        return sorted(self._records)

    def get(self, name: str) -> PolicyRecord:
        try:
            return self._records[str(name)]
        except KeyError as exc:
            raise ModelNotAvailable(
                f"{MODEL_NOT_AVAILABLE_YET} no policy named {name!r} is registered. "
                f"registered: {self.names() or 'none'}. "
                "Train a model and call register_policy(...) (see "
                "bullet_sim/ai/policy.py) before selecting --input model."
            ) from exc

    def describe(self) -> dict[str, Any]:
        return {
            "marker": MODEL_NOT_AVAILABLE_YET,
            "registered": {k: v.to_dict() for k, v in self._records.items()},
            "model_dir": str(MODEL_DIR),
            "model_dir_exists": MODEL_DIR.exists(),
            "deployment": MODEL_DEPLOYMENT_TBD,
        }


_REGISTRY = PolicyRegistry()


def register_policy(name: str, loader: Callable[..., Any], **kwargs: Any) -> None:
    """Make a trained policy selectable by name (``--input model --model NAME``)."""
    _REGISTRY.register(name, loader, **kwargs)


def available_policies() -> list[str]:
    return _REGISTRY.names()


def policy_registry() -> PolicyRegistry:
    return _REGISTRY


# --------------------------------------------------------------------------
# built-in loaders
# --------------------------------------------------------------------------


def load_npz_policy(path: str | os.PathLike) -> Any:
    """Load a small MLP policy from ``.npz`` (the FPGA-friendly export form).

    Expected keys: ``W1, b1[, W2, b2, ...]`` with ``W`` shaped ``(out, in)``.
    Activation is ReLU on hidden layers and argmax on the output logits, which
    is exactly what a fixed-point pipeline implements.
    """
    p = Path(path)
    if not p.exists():
        raise ModelNotAvailable(
            f"{MODEL_NOT_AVAILABLE_YET} no model file at {p}. "
            f"Drop a trained .npz policy into {MODEL_DIR} or pass --model PATH."
        )
    with np.load(p, allow_pickle=False) as data:
        arrays = {k: np.asarray(data[k]) for k in data.files}

    def policy(observation: Any, deterministic: bool = True) -> int:
        x = np.asarray(observation, dtype=np.float64).reshape(-1)
        layer = 1
        while f"W{layer}" in arrays:
            x = arrays[f"W{layer}"] @ x + arrays.get(f"b{layer}", 0.0)
            if f"W{layer + 1}" in arrays:
                x = np.maximum(x, 0.0)  # ReLU on hidden layers only
            layer += 1
        return int(np.argmax(x))

    return policy


def load_callable_policy(obj: Any) -> Any:
    """Adapt a duck-typed policy object (``predict`` / ``act`` / ``__call__``)."""
    if hasattr(obj, "predict"):
        return lambda obs, deterministic=True: obj.predict(obs, deterministic=deterministic)
    if hasattr(obj, "act"):
        return lambda obs, deterministic=True: obj.act(obs)
    if callable(obj):
        return obj
    raise ModelNotAvailable(
        f"{MODEL_NOT_AVAILABLE_YET} {obj!r} is not a usable policy "
        "(expected predict()/act()/__call__)"
    )


def load_policy(path_or_name: str | os.PathLike | None = None, *,
                policy: Any = None, obs_dim: int | None = None) -> Any:
    """Resolve a trained policy.

    ``path_or_name`` may be a ``.npz`` file, a registered policy name, or
    ``None`` (in which case the single registered policy is used).  The
    ``.onnx`` / vendor-runtime path is deliberately **not** implemented:

    ``[MODEL_DEPLOYMENT_TBD]`` the 30TAI / Icraft toolchain has not been chosen,
    so this function will not guess a loader.
    """
    if policy is not None:
        return load_callable_policy(policy)

    if path_or_name is None:
        names = _REGISTRY.names()
        if not names:
            raise ModelNotAvailable(
                f"{MODEL_NOT_AVAILABLE_YET} no trained policy is available. "
                "This repository ships baseline controllers only; see "
                "'Baseline Autonomous Mode' in the README."
            )
        return _REGISTRY.get(names[0]).loader(None)

    text = str(path_or_name)
    if text in _REGISTRY.names():
        return _REGISTRY.get(text).loader(None)

    suffix = Path(text).suffix.lower()
    if suffix == ".npz":
        return load_npz_policy(text)
    if suffix in (".onnx", ".pt", ".pth", ".tflite", ".kmodel"):
        raise ModelNotAvailable(
            f"{MODEL_DEPLOYMENT_TBD} cannot load {suffix} yet: the deployment "
            "toolchain for the 30TAI board is not decided, so no runtime is "
            "guessed. Export to .npz (see load_npz_policy) or register a "
            "callable with register_policy()."
        )
    if Path(text).exists():
        return load_npz_policy(text)
    raise ModelNotAvailable(
        f"{MODEL_NOT_AVAILABLE_YET} no policy file at {text} and no policy "
        f"registered under that name (registered: {_REGISTRY.names() or 'none'})"
    )


# --------------------------------------------------------------------------
# controller wrapper
# --------------------------------------------------------------------------


def _flatten_observation(observation: Mapping[str, Any], obs_dim: int | None) -> np.ndarray:
    """Deterministic observation vector for a model: player + target + env + bullets."""
    head = [
        np.asarray(observation["player"], dtype=np.float32).reshape(-1),
        np.asarray(observation["target"], dtype=np.float32).reshape(-1),
        np.asarray(observation["env"], dtype=np.float32).reshape(-1),
    ]
    bullets = np.asarray(observation.get("bullets", np.zeros((0, 7))), dtype=np.float32)
    vec = np.concatenate(head + [bullets.reshape(-1)])
    if obs_dim is not None:
        if vec.size < obs_dim:
            vec = np.concatenate([vec, np.zeros(obs_dim - vec.size, dtype=np.float32)])
        vec = vec[:obs_dim]
    return vec.astype(np.float32)


class TrainedPolicyController(BaseAutonomousController):
    """Wrap a trained policy so it drives the simulator like any other source.

    Input : flat ``float32`` observation vector (player 6 + target 4 + env 6 +
            bullets ``n x 7``, optionally padded/truncated to ``obs_dim``)
    Output: discrete action index ``0..8``
    """

    SPEC = ControllerSpec(
        name="model",
        kind="model",
        description="trained policy (none available yet)",
        access="observation",
        cost="cheap",
        notes=MODEL_NOT_AVAILABLE_YET,
    )

    def __init__(self, policy: Any, *, name: str = "model", obs_dim: int | None = None) -> None:
        super().__init__()
        self.policy = load_callable_policy(policy)
        self.obs_dim = obs_dim
        self.SPEC = ControllerSpec(
            name=name,
            kind="model",
            description="trained policy wrapped as an ActionProvider",
            access="observation",
            cost="cheap",
            notes=MODEL_NOT_AVAILABLE_YET,
        )

    def act(self, observation: Mapping[str, Any], info: Mapping[str, Any]) -> int:
        self._calls += 1
        vec = _flatten_observation(observation, self.obs_dim)
        out = self.policy(vec, deterministic=True)
        if isinstance(out, tuple):
            out = out[0]
        return int(np.asarray(out).reshape(-1)[0])


__all__ = [
    "MODEL_NOT_AVAILABLE_YET",
    "MODEL_DEPLOYMENT_TBD",
    "MODEL_DIR",
    "ModelNotAvailable",
    "PolicyRecord",
    "PolicyRegistry",
    "TrainedPolicyController",
    "available_policies",
    "load_callable_policy",
    "load_npz_policy",
    "load_policy",
    "policy_registry",
    "register_policy",
]

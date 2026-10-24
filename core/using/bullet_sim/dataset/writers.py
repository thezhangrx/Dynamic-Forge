"""Dataset writers: JSON (debug), NPZ (training), CSV (experiments), binary (fast).

All four formats round-trip through :mod:`bullet_sim.dataset.readers` and are
verified by the test suite.
"""

from __future__ import annotations

import csv
import json
import struct
from pathlib import Path
from typing import Any, Mapping

import numpy as np

from bullet_sim.core.errors import DatasetError
from bullet_sim.dataset.schema import Episode
from bullet_sim.entities.bullet import PROTOCOL_FIELDS

_BIN_MAGIC = 0x444C4842  # b'BHLD'
_BIN_VERSION = 1

#: Field order of a recorded bullet row / of the WorldSnapshot bullet dict.
_BULLET_KEYS = tuple(PROTOCOL_FIELDS)
_PLAYER_DICT_KEYS = ("x", "y", "vx", "vy", "radius", "speed", "alive")
_TARGET_DICT_KEYS = ("x", "y", "radius", "shape", "half_w", "half_h")
_ENV_DICT_KEYS = (
    "field_w", "field_h", "dt", "step_index", "sim_time", "wrap", "bullet_budget",
)
_WRAP_CODES = {"cull": 0.0, "wrap": 1.0, "bounce": 2.0}


def _vec(value: Any, dim: int, dict_keys: tuple[str, ...]) -> np.ndarray:
    """Coerce a list or a dataclass-dict into a fixed-length float32 vector."""
    if isinstance(value, Mapping):
        out = np.zeros(dim, dtype=np.float32)
        for i, key in enumerate(dict_keys[:dim]):
            raw = value.get(key, 0.0)
            if key == "shape":
                raw = 1.0 if raw == "rect" else 0.0
            elif key == "wrap":
                raw = _WRAP_CODES.get(str(raw), 0.0)
            elif key == "alive":
                raw = 1.0 if raw else 0.0
            out[i] = float(raw)
        return out
    if value is None:
        return np.zeros(dim, dtype=np.float32)
    return np.asarray(value, dtype=np.float32).reshape(-1)[:dim]

_CSV_COLUMNS = (
    "step_index",
    "timestamp",
    "action",
    "action_vx",
    "action_vy",
    "player_x",
    "player_y",
    "player_vx",
    "player_vy",
    "target_x",
    "target_y",
    "bullet_count",
    "collision",
    "collision_count",
    "min_dist2",
    "reward",
    "terminated",
    "truncated",
    "done",
    "state_hash",
)


# --------------------------------------------------------------------------
# JSON
# --------------------------------------------------------------------------


def save_json(
    episode: Episode, path: str | Path, *, max_transitions: int | None = None, indent: int = 2
) -> Path:
    """Full-fidelity JSON dump - intended for debugging and inspection."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    payload = episode.to_dict(max_transitions=max_transitions)
    payload["meta"] = {
        k: (v if isinstance(v, (str, int, float, bool, list, dict, type(None))) else str(v))
        for k, v in payload["meta"].items()
    }
    p.write_text(json.dumps(payload, indent=indent), encoding="utf-8")
    return p


def load_json(path: str | Path) -> Episode:
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    return episode_from_transitions(payload)


def episode_from_transitions(payload: Mapping[str, Any]) -> Episode:
    """Rebuild an :class:`Episode` from the JSON transition representation."""
    transitions = list(payload.get("transitions", []))
    initial = dict(payload.get("initial_state", {}))
    meta = dict(payload.get("meta", {}))
    n = len(transitions)

    player, target, env, counts, offsets, sampled = [], [], [], [], [], []
    bullet_rows: list[np.ndarray] = []
    total = 0

    def add_state(state: Mapping[str, Any]) -> None:
        nonlocal total
        player.append(_vec(state.get("player"), 7, _PLAYER_DICT_KEYS))
        target.append(_vec(state.get("target"), 6, _TARGET_DICT_KEYS))
        envs = _vec(state.get("env"), 7, _ENV_DICT_KEYS).tolist()
        envs[3] = float(state.get("step_index", envs[3]))
        env.append(np.asarray(envs, dtype=np.float32))
        rows = state.get("bullets", [])
        if isinstance(rows, Mapping):  # WorldSnapshot.to_dict() bullet form
            fields = rows.get("fields", {})
            n = int(rows.get("count", len(next(iter(fields.values()), []))))
            mat = np.stack(
                [np.asarray(fields.get(name, [0.0] * n), dtype=np.float32) for name in _BULLET_KEYS],
                axis=-1,
            ) if n else np.zeros((0, 10), dtype=np.float32)
        else:
            mat = np.asarray(rows, dtype=np.float32) if rows else np.zeros((0, 10), dtype=np.float32)
        counts.append(int(state.get("bullet_count", mat.shape[0])))
        offsets.append(total)
        sampled.append(bool(state.get("bullets_recorded", True)))
        bullet_rows.append(mat)
        total += mat.shape[0]

    add_state(initial)
    actions, action_vectors = [], []
    collisions, collision_counts, collision_events, min_d2 = [], [], [], []
    rewards, terminated, truncated, done, timestamps, step_indices, hashes = (
        [], [], [], [], [], [], []
    )
    for t in transitions:
        actions.append(t.get("action_t", 0))
        action_vectors.append(np.asarray(t.get("action_vector", [0.0, 0.0]), dtype=np.float32))
        collisions.append(bool(t.get("collision", False)))
        collision_counts.append(int(t.get("collision_count", 0)))
        collision_events.append(int(t.get("collision_events", 0)))
        min_d2.append(float(t.get("min_dist2", np.inf)))
        rewards.append(float(t.get("reward", 0.0)))
        terminated.append(bool(t.get("terminated", False)))
        truncated.append(bool(t.get("truncated", False)))
        done.append(bool(t.get("done", False)))
        timestamps.append(float(t.get("timestamp", 0.0)))
        step_indices.append(int(t.get("step_index", 0)))
        hashes.append(str(t.get("state_hash", "")))
        add_state(t.get("next_state", {}))

    actions_arr = _uniform_actions(actions)
    # ``state_hashes`` is per *state* (T+1); the JSON form only carries hashes
    # for S_1..S_T, so pad the initial slot.
    hashes_arr = np.asarray([str(initial.get("state_hash", ""))] + hashes, dtype="U32")
    arrays: dict[str, np.ndarray] = {
        "player": np.asarray(player, dtype=np.float32),
        "target": np.asarray(target, dtype=np.float32),
        "env": np.asarray(env, dtype=np.float32),
        "bullets": (
            np.concatenate(bullet_rows, axis=0) if bullet_rows and total else np.zeros((0, 10), np.float32)
        ),
        "bullet_counts": np.asarray(counts, dtype=np.int32),
        "bullets_offsets": np.asarray(offsets, dtype=np.int32),
        "bullet_sampled": np.asarray(sampled, dtype=bool),
        "actions": actions_arr,
        "action_vectors": np.asarray(action_vectors, dtype=np.float32).reshape(n, 2),
        "collisions": np.asarray(collisions, dtype=bool),
        "collision_counts": np.asarray(collision_counts, dtype=np.int32),
        "collision_events": np.asarray(collision_events, dtype=np.int32),
        "min_dist2": np.asarray(min_d2, dtype=np.float32),
        "rewards": np.asarray(rewards, dtype=np.float32),
        "terminated": np.asarray(terminated, dtype=bool),
        "truncated": np.asarray(truncated, dtype=bool),
        "done": np.asarray(done, dtype=bool),
        "timestamps": np.asarray(timestamps, dtype=np.float64),
        "step_indices": np.asarray(step_indices, dtype=np.int32),
        "state_hashes": hashes_arr,
    }
    return Episode(meta=meta, initial_state=initial, arrays=arrays)


def _uniform_actions(actions: list) -> np.ndarray:
    if not actions:
        return np.zeros((0,), dtype=np.int32)
    if isinstance(actions[0], (list, tuple)):
        return np.asarray(actions, dtype=np.float32)
    try:
        return np.asarray(actions, dtype=np.int32)
    except (TypeError, ValueError):
        return np.asarray(actions, dtype=np.float32)


# --------------------------------------------------------------------------
# NPZ
# --------------------------------------------------------------------------


def save_npz(episode: Episode, path: str | Path) -> Path:
    """Compressed NumPy archive: arrays + JSON metadata blobs."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    payload: dict[str, Any] = {k: v for k, v in episode.arrays.items()}
    payload["__meta__"] = np.asarray(json.dumps(_jsonable(episode.meta)))
    payload["__initial__"] = np.asarray(json.dumps(_jsonable(episode.initial_state)))
    np.savez_compressed(p, **payload)
    return p


def load_npz(path: str | Path) -> Episode:
    with np.load(Path(path), allow_pickle=False) as data:
        keys = set(data.files)
        meta = json.loads(str(data["__meta__"])) if "__meta__" in keys else {}
        initial = json.loads(str(data["__initial__"])) if "__initial__" in keys else {}
        arrays = {k: data[k] for k in data.files if not k.startswith("__")}
    return Episode(meta=meta, initial_state=initial, arrays=arrays)


# --------------------------------------------------------------------------
# CSV
# --------------------------------------------------------------------------


def save_csv(episode: Episode, path: str | Path) -> Path:
    """Scalar per-step table - handy for spreadsheets and quick plots."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    a = episode.arrays
    n = episode.steps
    with p.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(_CSV_COLUMNS)
        for t in range(n):
            action = a["actions"][t]
            action_str = (
                int(action) if np.asarray(action).ndim == 0 else json.dumps(np.asarray(action).tolist())
            )
            hashes = a.get("state_hashes")
            writer.writerow(
                [
                    int(a["step_indices"][t]),
                    float(a["timestamps"][t]),
                    action_str,
                    float(a["action_vectors"][t, 0]),
                    float(a["action_vectors"][t, 1]),
                    float(a["player"][t + 1, 0]),
                    float(a["player"][t + 1, 1]),
                    float(a["player"][t + 1, 2]),
                    float(a["player"][t + 1, 3]),
                    float(a["target"][t + 1, 0]),
                    float(a["target"][t + 1, 1]),
                    int(a["bullet_counts"][t + 1]),
                    bool(a["collisions"][t]),
                    int(a["collision_counts"][t]),
                    float(a["min_dist2"][t]),
                    float(a["rewards"][t]),
                    bool(a["terminated"][t]),
                    bool(a["truncated"][t]),
                    bool(a["done"][t]),
                    str(hashes[t + 1]) if hashes is not None and hashes.size > t + 1 else "",
                ]
            )
    return p


def load_csv(path: str | Path) -> Episode:
    rows = list(csv.DictReader(Path(path).open("r", encoding="utf-8")))
    transitions = []
    for r in rows:
        action_raw = r["action"]
        try:
            action = int(action_raw)
        except ValueError:
            action = json.loads(action_raw)
        # CSV has no full state blocks; reconstruct what is representable.
        player = [float(r["player_x"]), float(r["player_y"]), float(r["player_vx"]),
                  float(r["player_vy"]), 0.0, 0.0, 1.0]
        target = [float(r["target_x"]), float(r["target_y"]), 0.0, 0.0, 0.0, 0.0]
        env = [0.0, 0.0, 0.0, float(r["step_index"]), float(r["timestamp"]), 0.0, 0.0]
        state = {
            "step_index": int(r["step_index"]),
            "timestamp": float(r["timestamp"]),
            "player": player,
            "target": target,
            "env": env,
            "bullets": [],
            "bullet_count": int(r["bullet_count"]),
            "bullets_recorded": False,
        }
        transitions.append(
            {
                "state_t": {},
                "action_t": action,
                "action_vector": [float(r["action_vx"]), float(r["action_vy"])],
                "next_state": state,
                "collision": r["collision"] == "True",
                "collision_count": int(r["collision_count"]),
                "min_dist2": float(r["min_dist2"]),
                "reward": float(r["reward"]),
                "terminated": r["terminated"] == "True",
                "truncated": r["truncated"] == "True",
                "done": r["done"] == "True",
                "timestamp": float(r["timestamp"]),
                "step_index": int(r["step_index"]),
                "state_hash": r.get("state_hash", ""),
            }
        )
    payload = {
        "meta": {"format": "csv", "note": "scalar fields only; bullet blocks unavailable"},
        "initial_state": transitions[0]["state_t"] if transitions else {},
        "transitions": transitions,
    }
    return episode_from_transitions(payload)


# --------------------------------------------------------------------------
# binary
# --------------------------------------------------------------------------


def save_binary(episode: Episode, path: str | Path) -> Path:
    """Self-describing raw array container for high-throughput downstream use."""
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    meta_bytes = json.dumps(_jsonable(episode.meta)).encode("utf-8")
    init_bytes = json.dumps(_jsonable(episode.initial_state)).encode("utf-8")
    items = [(k, v) for k, v in episode.arrays.items() if isinstance(v, np.ndarray)]
    with p.open("wb") as fh:
        fh.write(struct.pack("<III", _BIN_MAGIC, _BIN_VERSION, len(meta_bytes)))
        fh.write(meta_bytes)
        fh.write(struct.pack("<I", len(init_bytes)))
        fh.write(init_bytes)
        fh.write(struct.pack("<I", len(items)))
        for name, arr in items:
            a = np.ascontiguousarray(arr)
            nb = name.encode("utf-8")
            dt = a.dtype.str.encode("utf-8")
            fh.write(struct.pack("<I", len(nb)))
            fh.write(nb)
            fh.write(struct.pack("<I", len(dt)))
            fh.write(dt)
            fh.write(struct.pack("<I", a.ndim))
            for dim in a.shape:
                fh.write(struct.pack("<Q", int(dim)))
            raw = a.tobytes()
            fh.write(struct.pack("<Q", len(raw)))
            fh.write(raw)
    return p


def load_binary(path: str | Path) -> Episode:
    with Path(path).open("rb") as fh:
        magic, version, meta_len = struct.unpack("<III", fh.read(12))
        if magic != _BIN_MAGIC:
            raise DatasetError(f"bad dataset magic 0x{magic:08X}")
        if version != _BIN_VERSION:
            raise DatasetError(f"unsupported binary dataset version {version}")
        meta = json.loads(fh.read(meta_len).decode("utf-8"))
        (init_len,) = struct.unpack("<I", fh.read(4))
        initial = json.loads(fh.read(init_len).decode("utf-8"))
        (n_arrays,) = struct.unpack("<I", fh.read(4))
        arrays: dict[str, np.ndarray] = {}
        for _ in range(n_arrays):
            (nlen,) = struct.unpack("<I", fh.read(4))
            name = fh.read(nlen).decode("utf-8")
            (dlen,) = struct.unpack("<I", fh.read(4))
            dtype = np.dtype(fh.read(dlen).decode("utf-8"))
            (ndim,) = struct.unpack("<I", fh.read(4))
            shape = tuple(struct.unpack("<Q", fh.read(8))[0] for _ in range(ndim))
            (nbytes,) = struct.unpack("<Q", fh.read(8))
            raw = fh.read(nbytes)
            if len(raw) != nbytes:
                raise DatasetError(f"truncated array {name!r}")
            arrays[name] = np.frombuffer(raw, dtype=dtype).reshape(shape).copy()
    return Episode(meta=meta, initial_state=initial, arrays=arrays)


# --------------------------------------------------------------------------
# dispatch
# --------------------------------------------------------------------------

_WRITERS = {
    ".json": save_json,
    ".npz": save_npz,
    ".csv": save_csv,
    ".bin": save_binary,
    ".dat": save_binary,
}
_READERS = {
    ".json": load_json,
    ".npz": load_npz,
    ".csv": load_csv,
    ".bin": load_binary,
    ".dat": load_binary,
}


def save_episode(episode: Episode, path: str | Path, fmt: str | None = None) -> str:
    """Save ``episode``; the format is chosen by suffix unless ``fmt`` is given."""
    p = Path(path)
    suffix = f".{fmt.lstrip('.')}" if fmt else p.suffix.lower()
    if suffix not in _WRITERS:
        suffix = ".npz"
        p = p.with_suffix(".npz")
    _WRITERS[suffix](episode, p)
    return str(p)


def supported_formats() -> list[str]:
    return sorted({s.lstrip(".") for s in _WRITERS})


def _jsonable(obj: Any) -> Any:
    if isinstance(obj, dict):
        return {str(k): _jsonable(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_jsonable(v) for v in obj]
    if isinstance(obj, np.generic):
        return obj.item()
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, (str, int, float, bool)) or obj is None:
        return obj
    return str(obj)


__all__ = [
    "save_json",
    "load_json",
    "save_npz",
    "load_npz",
    "save_csv",
    "load_csv",
    "save_binary",
    "load_binary",
    "save_episode",
    "supported_formats",
    "episode_from_transitions",
]

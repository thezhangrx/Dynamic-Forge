"""Example 3 - headless batch simulation and training-dataset generation.

    python bullet_sim/examples/03_headless_dataset.py [out_dir]

Produces one episode per seed in four formats and verifies every one of them by
re-reading and re-simulating from the recorded action sequence.
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python core/bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import sys
from pathlib import Path

import numpy as np

from bullet_sim.dataset.readers import load_episode, verify_roundtrip
from bullet_sim.dataset.recorder import TrajectoryRecorder
from bullet_sim.dataset.writers import save_episode
from bullet_sim.scenarios.builder import build_many
from bullet_sim.simulator.env import BulletHellEnv
from bullet_sim.simulator.replay import replay, verify_replay


def main() -> None:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else "data/example03")
    out.mkdir(parents=True, exist_ok=True)

    scenarios = build_many(3, "medium", base_seed=1000)
    print(f"generating {len(scenarios)} episodes into {out}/")

    for i, built in enumerate(scenarios):
        rec = TrajectoryRecorder(store_hashes=True, extra_meta={"scenario": built.spec.to_dict()})
        env = BulletHellEnv(built.spec, seed=built.spec.seed, record=rec)
        env.run("random", steps=600, seed=built.spec.seed)
        episode = rec.finish()
        env.close()

        fmt = ["npz", "json", "bin", "csv"][i % 4]
        path = save_episode(episode, out / f"ep_{i:05d}.{fmt}")
        ok = verify_roundtrip(episode, path)

        # reproducibility: re-run the recorded actions and compare state hashes
        actions = [int(a) for a in episode.arrays["actions"]]
        hashes = [str(h) for h in episode.arrays["state_hashes"]][1:]
        reproduced, mismatch = verify_replay(built.spec, actions, hashes, seed=built.spec.seed)

        s = episode.summary()
        print(
            f"  ep_{i:05d} [{fmt:>4}] steps={s['steps']:<5} max_bullets={s['max_bullets']:<5} "
            f"mean_bullets={s['mean_bullets']:.1f} collisions={s['collisions']} "
            f"| roundtrip={'OK' if ok else 'FAIL'} replay={'OK' if reproduced else f'FAIL@{mismatch}'}"
        )

    # Demonstrate loading and consuming the dataset as ML arrays.
    first = sorted(out.glob("ep_*.npz"))[0]
    ep = load_episode(first)
    a = ep.arrays
    print(f"\nloaded {first.name}: transitions={ep.steps}")
    print(f"  player   {a['player'].shape} {a['player'].dtype}")
    print(f"  bullets  {a['bullets'].shape} {a['bullets'].dtype}")
    print(f"  actions  {a['actions'].shape} {a['actions'].dtype}")
    print(f"  rewards  {a['rewards'].shape}  total={float(a['rewards'].sum()):.1f}")
    print(f"  one transition:")
    tr = ep.transition(0)
    print(f"    S_0 step={tr['state_t']['step_index']} bullets={tr['state_t']['bullet_count']}")
    print(f"    A_0={tr['action_t']} reward={tr['reward']} done={tr['done']}")
    print(f"    S_1 step={tr['next_state']['step_index']} bullets={tr['next_state']['bullet_count']}")

    # A tiny supervised target: predict the player position 10 steps ahead.
    horizon = 10
    if ep.steps > horizon:
        x = a["player"][:-horizon]
        y = a["player"][horizon:]
        print(f"  (supervised) X={x.shape} Y={y.shape} ready for a model")


if __name__ == "__main__":
    main()

"""Example 8 - visual debugging.

Three views, in increasing fidelity:

* ASCII  - dependency-free, works over SSH / in CI logs
* rgb_array - offscreen pygame surface as a NumPy array (no window)
* human - a real window with HUD, danger overlay and prediction overlay

    python bullet_sim/examples/08_render_debug.py ascii
    python bullet_sim/examples/08_render_debug.py rgb
    python bullet_sim/examples/08_render_debug.py human     # needs a display
"""

from __future__ import annotations

import pathlib
import sys

if __package__ in (None, ""):  # allow `python core/bullet_sim/examples/<name>.py`
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

import sys
import time

from bullet_sim.prediction.ballistic import BallisticPredictor
from bullet_sim.prediction.danger_field import compute_danger_field
from bullet_sim.scenarios.presets import scenario_for_level
from bullet_sim.simulator.env import BulletHellEnv


def main() -> None:
    mode = sys.argv[1] if len(sys.argv) > 1 else "ascii"
    spec = scenario_for_level("medium", seed=31, duration=30.0)
    env = BulletHellEnv(spec, seed=31, render_mode=None)
    env.reset(seed=31)

    predictor = BallisticPredictor()
    danger = None

    if mode == "ascii":
        for i in range(400):
            env.step(0)
            if i % 40 == 0:
                print(env.render("ascii", width=78, height=24))
                print()
        env.close()
        return

    if mode == "rgb":
        frame = None
        for i in range(200):
            env.step(0)
            if i % 20 == 0:
                danger = compute_danger_field(
                    env.simulate_future(0, horizon=10), resolution=16.0
                )
                frame = env.render(
                    "rgb_array",
                    predictor=predictor,
                    show_prediction=True,
                    show_danger=True,
                    danger=danger,
                    show_hud=True,
                )
        print(f"rgb_array frame: shape={None if frame is None else frame.shape}")
        env.close()
        return

    if mode == "human":
        from bullet_sim.render.pygame_view import PygameRenderer

        renderer = PygameRenderer(
            mode="human",
            show_prediction=True,
            predictor=predictor,
            prediction_horizon=90,
            interactive=True,
        )
        sim_per_frame = 2  # 60 render FPS x 2 steps of 120 Hz = 1x real time
        try:
            while not renderer.closed:
                danger = compute_danger_field(
                    env.simulate_future(0, horizon=8), resolution=16.0
                )
                renderer.danger = danger
                renderer.show_danger = True
                action = renderer.pending_action if renderer.pending_action is not None else 0
                renderer.pending_action = None
                for _ in range(sim_per_frame):
                    env.step(action)
                renderer.render(env.world)
                time.sleep(1.0 / 60.0)
        except KeyboardInterrupt:
            pass
        finally:
            renderer.close()
            env.close()
        return

    print(f"unknown mode {mode!r}; expected ascii|rgb|human")


if __name__ == "__main__":
    main()

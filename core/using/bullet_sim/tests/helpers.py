"""Shared test helpers.

Deliberately fixture-free so the suite runs both under ``pytest`` and under the
bundled fallback runner (``python -m bullet_sim.tests.run_tests``).
"""

from __future__ import annotations

import numpy as np

from bullet_sim.core.actions import DiscreteActionCodec
from bullet_sim.generators.burst import GenerationContext
from bullet_sim.core.clock import FixedClock
from bullet_sim.scenarios.spec import ScenarioSpec
from bullet_sim.simulator.world import World

DT = 1.0 / 60.0


def blank_spec(**overrides) -> ScenarioSpec:
    """A scenario with no emitters - tests inject bullets manually."""
    params = dict(
        name="blank",
        seed=0,
        duration=10.0,
        dt=DT,
        field_w=10000.0,
        field_h=10000.0,
        player_x=5000.0,
        player_y=5000.0,
        patterns=[],
        cull_margin=100000.0,
    )
    params.update(overrides)
    return ScenarioSpec(**params)


def make_world(**overrides) -> World:
    return World(
        blank_spec(**overrides),
        collision="null",
        reward="zero",
        terminate_on_collision=False,
    )


def gen_ctx(**overrides) -> GenerationContext:
    params = dict(
        field_w=640.0,
        field_h=480.0,
        dt=DT,
        duration=5.0,
        rng=np.random.Generator(np.random.PCG64(1234)),
        player_xy=np.array([320.0, 240.0]),
        clock=FixedClock.make(dt=DT),
    )
    params.update(overrides)
    return GenerationContext(**params)


def discrete_codec() -> DiscreteActionCodec:
    return DiscreteActionCodec()


def step_many(world: World, action, n: int):
    result = None
    for _ in range(n):
        result = world.step(action)
    return result


__all__ = ["DT", "blank_spec", "make_world", "gen_ctx", "discrete_codec", "step_many"]

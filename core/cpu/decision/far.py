"""Future Action Risk (FAR): candidate-conditioned predictive risk (v0.4.1-SideHit-ASE).

v0.3-5 asked *"is the current position dangerous?"*  v0.4.0 answers *"if the agent
executes candidate action ``a``, does the near future become safer or more
dangerous?"*.  Everything here is a pure function of numpy arrays and the two
motion models already used by the project - no new dynamics are introduced::

    p_agent(t)    = p_a + v_candidate(a) * t          (first-order omnidirectional)
    p_obstacle(t) = p_o + v_o * t                     (existing linear model)

For every candidate and obstacle::

    r0    = p_o - p_a
    v_rel = v_o - v_candidate(a)          # the key: LEFT and RIGHT differ here
    t_ca  = clamp(-dot(r0, v_rel) / |v_rel|^2, 0, horizon)   # closest approach
    d_ca^2 = |r0|^2 + 2 dot(r0, v_rel) t_ca + |v_rel|^2 t_ca^2

IMPORTANT (naming): ``t_ca`` is the *constant-velocity closest-approach time*,
not a collision time.  A strict time-to-collision only exists when the obstacle
is approaching **and** the closest approach penetrates the inflated radius, in
which case the first-contact time is
``t_contact = t_ca - sqrt(r_sum^2 - d_ca^2) / |v_rel|``.  Only that quantity is
called ``min_ttc``; ``t_ca``/``d_ca`` keep their own names throughout.

v0.4.1-SideHit-ASE adds a **candidate-specific lateral closing rate** and an
**adaptive safety envelope** on top of the FAR profile:

    closing_rate(a) = max over obstacles of max(0, -dot(r0, v_rel(a)) / |r0|)
    safety_margin(t) = clearance(t) - (base_margin + closing_rate * margin_time)
    min_margin       = min over samples/obstacles of safety_margin(t)

    LEVEL 0 normal | LEVEL 1 critical (penalty) | LEVEL 2 unsafe (infeasible)

This attacks the v0.4.0 ``side_hit`` failure mode: a fast *lateral* approach can
be critical even while the geometric clearance is still positive, and LEFT /
RIGHT / DIAGONAL genuinely differ because ``v_rel`` depends on the candidate.

This module is deliberately geometry-only (CommonRoad Drivability Checker split):
feasibility is decided here, utility is decided by the planner's critics.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

from cpu.decision.models import Action, SceneState

__all__ = [
    "CandidateSafety",
    "CandidateRiskProfile",
    "SafetyEnvelope",
    "candidate_safety_batch",
    "future_action_risk_batch",
    "future_risk_critic",
    "critical_risk_critic",
    "adaptive_safety_envelope",
    "sample_times",
    "FAR_SAMPLE_NEAR",
    "FAR_SAMPLE_MID",
    "ASE_BASE_MARGIN",
    "ASE_MARGIN_TIME",
    "ASE_CRITICAL_MARGIN",
]

#: Fixed future sample times (seconds).  Kept tiny on purpose: FPGA-friendly.
FAR_SAMPLE_NEAR = 0.08
FAR_SAMPLE_MID = 0.16

#: Adaptive safety envelope defaults (world units / seconds; configurable).
ASE_BASE_MARGIN = 0.0          # constant buffer added on top of surface contact
ASE_MARGIN_TIME = 0.04         # seconds of closing speed converted into buffer
ASE_CRITICAL_MARGIN = 4.0      # margin band that counts as LEVEL 1 (critical)
#: Lateral closing gates, expressed as a fraction of the agent's max speed so the
#: envelope stays scale-invariant (like RepulsionController's reaction radius).
ASE_LATERAL_RATE_THRESHOLD_FRAC = 0.05
ASE_LATERAL_RATE_SCALE_FRAC = 0.5

_EPS = 1e-9


@dataclass(frozen=True)
class CandidateSafety:
    """Continuous candidate x obstacle safety features (v0.4.0-FAR).

    All arrays are ``(B, N)`` unless noted; ``B`` candidates, ``N`` obstacles.
    """

    t_ca: np.ndarray          # (B, N) closest-approach time, clamped to [0, horizon]
    d_ca: np.ndarray          # (B, N) distance at closest approach (>= 0)
    approaching: np.ndarray   # (B, N) bool: dot(r0, v_rel) < 0
    collides: np.ndarray      # (B,)   bool: min distance over the horizon penetrates
    min_ttc: np.ndarray       # (B,)   strict first-contact time (inf = no contact)


@dataclass(frozen=True)
class CandidateRiskProfile:
    """Per-candidate FAR + adaptive-envelope profile (one array entry per candidate).

    ``risk_trend`` is **signed** (``risk_far - risk_near``): ``> 0`` means the
    future is getting worse, ``< 0`` means it is improving.  The critic applies
    ``max(0, risk_trend)``, so the signed form is preserved for diagnostics.

    ``closing_rate`` is the candidate-specific lateral/radial closing rate at
    t = 0 (max over obstacles).  ``min_margin`` is the minimum adaptive safety
    margin over the future samples.  ``envelope_level`` is 0 normal / 1 critical /
    2 unsafe.  ``lateral_critical_risk`` is the side-hit dedicated critical score.
    """

    current_risk: np.ndarray   # (B,)  risk at t = 0, candidate-conditioned
    risk_near: np.ndarray      # (B,)  risk at t_near
    risk_mid: np.ndarray       # (B,)  risk at t_mid
    risk_far: np.ndarray       # (B,)  risk at t_far
    risk_peak: np.ndarray      # (B,)  max(risk_near, risk_mid, risk_far)
    risk_trend: np.ndarray     # (B,)  risk_far - risk_near (signed)
    min_ttc: np.ndarray        # (B,)  strict first-contact time
    min_clearance: np.ndarray  # (B,)  min geometric clearance over the samples
    closing_rate: np.ndarray   # (B,)  max candidate-specific closing rate (>= 0)
    min_margin: np.ndarray     # (B,)  min adaptive safety margin over the samples
    envelope_level: np.ndarray  # (B,) int: 0 normal / 1 critical / 2 unsafe
    lateral_critical_risk: np.ndarray  # (B,) side-hit critical score in [0, 1]
    feasible: np.ndarray       # (B,)  not unsafe (envelope level < 2)
    sample_times: np.ndarray   # (S,)  the sample times used


@dataclass(frozen=True)
class SafetyEnvelope:
    """Adaptive Safety Envelope view over a :class:`CandidateRiskProfile`.

    LEVEL 0 normal, LEVEL 1 critical (feasible, penalised), LEVEL 2 unsafe
    (infeasible).  Reference: Nav2 MPPI true-collision vs near-collision critics.
    """

    level: np.ndarray                 # (B,) int 0/1/2
    min_margin: np.ndarray            # (B,)
    closing_rate: np.ndarray          # (B,)
    critical: np.ndarray              # (B,) bool (level == 1)
    unsafe: np.ndarray                # (B,) bool (level >= 2)
    lateral_critical_risk: np.ndarray  # (B,)


# --------------------------------------------------------------------------
# geometry helpers
# --------------------------------------------------------------------------
def _obstacle_arrays(
    scene: SceneState,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Obstacle SoA arrays ``(positions (N,2), velocities (N,2), radii (N,))``."""
    obstacles = scene.obstacles
    if not obstacles:
        return (
            np.zeros((0, 2), dtype=np.float64),
            np.zeros((0, 2), dtype=np.float64),
            np.zeros((0,), dtype=np.float64),
        )
    positions = np.stack([o.position for o in obstacles]).astype(np.float64, copy=False)
    velocities = np.stack([o.velocity for o in obstacles]).astype(np.float64, copy=False)
    radii = np.asarray([o.radius for o in obstacles], dtype=np.float64)
    return positions, velocities, radii


def _candidate_velocities(actions: Sequence[Action]) -> np.ndarray:
    return np.stack([a.velocity_vector for a in actions]).astype(np.float64, copy=False)


def sample_times(horizon: float) -> np.ndarray:
    """Fixed future sample times, adaptively capped by ``horizon`` (3 samples)."""
    h = max(float(horizon), 0.0)
    return np.array(
        [min(FAR_SAMPLE_NEAR, h), min(FAR_SAMPLE_MID, h), h], dtype=np.float64
    )


# --------------------------------------------------------------------------
# continuous candidate safety (closest approach)
# --------------------------------------------------------------------------
def candidate_safety_batch(
    scene: SceneState,
    actions: Sequence[Action],
    horizon: float,
) -> CandidateSafety:
    """Velocity-aware continuous safety filter for every candidate.

    # [v0.4.0-FAR]
    # PURPOSE:
    #   Decide feasibility from the *continuous* constant-velocity closest
    #   approach instead of a few sampled frames, so a trajectory that only
    #   clips an obstacle between samples is still rejected.  This is the filter
    #   that removes the side-hit candidates v0.3-5 kept ranking.
    #
    # OPEN-SOURCE REFERENCE:
    #   F1TENTH iTTC / instantaneous collision risk (distance + closing speed),
    #   structured like a DWA candidate rollout and CommonRoad's separated
    #   collision checker.  Structure only - no code is copied.
    #
    # ALGORITHM:
    #   r0 = p_o - p_a, v_rel = v_o - v_candidate
    #   t_ca  = clip(-dot(r0,v_rel)/|v_rel|^2, 0, horizon)   (0 if |v_rel|^2 ~ 0)
    #   d_ca^2 = |r0|^2 + 2 dot(r0,v_rel) t_ca + |v_rel|^2 t_ca^2
    #   collides = any(d_ca <= r_agent + r_obstacle)
    #   min_ttc  = min first-contact time where approaching and d_ca < r_sum
    #
    # FPGA MAPPING:
    #   candidate x obstacle fixed-size dot/square/compare kernel; one sqrt and
    #   one reciprocal per (candidate, obstacle).  No atan2, no branches except
    #   guarded divides.
    #
    # CPU ROLE:
    #   Produces the feasibility features consumed by the planner; it never picks
    #   an action itself.
    #
    # COMPLEXITY:
    #   O(B * N) with B = 17 candidates.
    # """
    actions = tuple(actions)
    b = len(actions)
    h = max(float(horizon), 0.0)
    positions, velocities, radii = _obstacle_arrays(scene)
    n = radii.shape[0]

    if b == 0:
        return CandidateSafety(
            t_ca=np.zeros((0, n)),
            d_ca=np.zeros((0, n)),
            approaching=np.zeros((0, n), dtype=bool),
            collides=np.zeros(0, dtype=bool),
            min_ttc=np.zeros(0, dtype=np.float64),
        )
    if n == 0:
        return CandidateSafety(
            t_ca=np.zeros((b, 0)),
            d_ca=np.zeros((b, 0)),
            approaching=np.zeros((b, 0), dtype=bool),
            collides=np.zeros(b, dtype=bool),
            min_ttc=np.full(b, np.inf, dtype=np.float64),
        )

    agent = scene.agent
    v_cand = _candidate_velocities(actions)                      # (B, 2)
    r0 = positions[None, :, :] - agent.position[None, None, :]   # (1, N, 2)
    v_rel = velocities[None, :, :] - v_cand[:, None, :]          # (B, N, 2)

    r0_sq = (r0 * r0).sum(axis=-1)                # (1, N)
    r0_dot = (r0 * v_rel).sum(axis=-1)            # (B, N)
    v_sq = (v_rel * v_rel).sum(axis=-1)           # (B, N)

    # Guard |v_rel|^2 == 0 (candidate matching the obstacle velocity exactly):
    # the separation is then constant, closest approach is t = 0.
    moving = v_sq > _EPS
    t_ca = np.where(moving, -r0_dot / np.where(moving, v_sq, 1.0), 0.0)
    t_ca = np.clip(t_ca, 0.0, h)

    d2_ca = r0_sq + 2.0 * r0_dot * t_ca + v_sq * t_ca * t_ca    # (B, N)
    d2_ca = np.maximum(d2_ca, 0.0)                               # kill float noise
    d_ca = np.sqrt(d2_ca)

    r_sum = agent.radius + radii                                 # (N,)
    r_sum_sq = r_sum * r_sum
    approaching = r0_dot < 0.0

    collides = (d_ca <= r_sum[None, :]).any(axis=1)              # (B,)

    # Strict first-contact time (only when the closest approach penetrates).
    speed = np.sqrt(v_sq)                                        # (B, N)
    disc = r_sum_sq[None, :] - d2_ca                             # (B, N)
    t_contact = t_ca - np.sqrt(np.maximum(disc, 0.0)) / np.maximum(speed, _EPS)
    already_inside = r0_sq <= r_sum_sq[None, :]                  # (1, N)
    potential = (disc > 0.0) & approaching
    ttc = np.where(
        already_inside,
        0.0,
        np.where(potential, np.maximum(t_contact, 0.0), np.inf),
    )
    min_ttc = ttc.min(axis=1)                                    # (B,)

    return CandidateSafety(
        t_ca=t_ca,
        d_ca=d_ca,
        approaching=approaching,
        collides=collides,
        min_ttc=min_ttc,
    )


# --------------------------------------------------------------------------
# future action risk profile + adaptive safety envelope
# --------------------------------------------------------------------------
def future_action_risk_batch(
    scene: SceneState,
    actions: Sequence[Action],
    horizon: float,
    *,
    ttc_reference: float = 1.0,
    safety: CandidateSafety | None = None,
    base_margin: float = ASE_BASE_MARGIN,
    margin_time: float = ASE_MARGIN_TIME,
    critical_margin: float = ASE_CRITICAL_MARGIN,
    lateral_rate_threshold_frac: float = ASE_LATERAL_RATE_THRESHOLD_FRAC,
    lateral_rate_scale_frac: float = ASE_LATERAL_RATE_SCALE_FRAC,
    trend_gain: float = 1.0,
) -> CandidateRiskProfile:
    """Candidate-conditioned risk profile + adaptive safety envelope.

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Per candidate, quantify (a) how fast it is *closing* on each obstacle and
    #   (b) whether its future clearance stays inside an adaptive buffer.  This is
    #   what turns "no geometric collision yet" into "critical, because a fast
    #   lateral approach will arrive soon" - the v0.4.0 side_hit mode.
    #
    # OPEN-SOURCE REFERENCE:
    #   PythonRobotics DWA (candidate rollout + trajectory evaluation), Nav2 MPPI
    #   (true-collision vs near-collision critics), F1TENTH iTTC (distance +
    #   closing speed as the safety primitive), RVO2/ORCA velocity-obstacle
    #   *geometry idea* only (relative velocity), CommonRoad (feasibility split
    #   from utility).  Structure only - no implementation is copied.
    #
    # ALGORITHM:
    #   closing_rate(a)  = max_o max(0, -dot(r0_o, v_rel(a)_o) / |r0_o|)
    #   dynamic_margin   = base_margin + closing_rate * margin_time
    #   min_margin       = min_o,t ( clearance_o(t) - dynamic_margin_o )
    #   unsafe   (L2)    = collision  OR  min_margin < 0
    #   critical (L1)    = not unsafe AND min_margin < critical_margin
    #   lateral_critical = rate_gate * max(margin_term, ttc_term, risk_peak, trend+)
    #     with rate_gate = clip((closing_rate/max_speed - thr)/scale, 0, 1), so it
    #     is exactly 0 for a purely lateral move with no radial closing.
    #
    # FPGA MAPPING:
    #   Adds a candidate x obstacle closing-rate kernel and a candidate x sample x
    #   obstacle margin-min-reduce: add/multiply/square/compare/min/max only.
    #
    # CPU ROLE:
    #   Emits the fixed-size feature vector the CPU critics rank; selection stays
    #   on the CPU.
    #
    # COMPLEXITY:
    #   O(B * S * N + B * N), S = 3.
    # """
    actions = tuple(actions)
    b = len(actions)
    times = sample_times(horizon)
    if safety is None:
        safety = candidate_safety_batch(scene, actions, horizon)

    positions, velocities, radii = _obstacle_arrays(scene)
    n = radii.shape[0]
    if b == 0 or n == 0:
        zeros = np.zeros(b, dtype=np.float64)
        return CandidateRiskProfile(
            current_risk=zeros,
            risk_near=zeros.copy(),
            risk_mid=zeros.copy(),
            risk_far=zeros.copy(),
            risk_peak=zeros.copy(),
            risk_trend=zeros.copy(),
            min_ttc=safety.min_ttc,
            min_clearance=np.full(b, np.inf, dtype=np.float64),
            closing_rate=zeros.copy(),
            min_margin=np.full(b, np.inf, dtype=np.float64),
            envelope_level=np.zeros(b, dtype=np.int64),
            lateral_critical_risk=zeros.copy(),
            feasible=np.ones(b, dtype=bool),
            sample_times=times,
        )

    agent = scene.agent
    v_cand = _candidate_velocities(actions)                      # (B, 2)
    r0 = positions[None, :, :] - agent.position[None, None, :]   # (1, N, 2)
    v_rel = velocities[None, :, :] - v_cand[:, None, :]          # (B, N, 2)
    r_sum = agent.radius + radii                                 # (N,)
    t_ref = max(float(ttc_reference), _EPS)

    def _risk_at(t: float) -> tuple[np.ndarray, np.ndarray]:
        r = r0 + v_rel * float(t)                                # (B, N, 2)
        d_sq = (r * r).sum(axis=-1)                              # (B, N)
        d = np.sqrt(np.maximum(d_sq, _EPS))
        clearance = d - r_sum[None, :]                           # (B, N)
        closing = np.maximum(-(r * v_rel).sum(axis=-1), 0.0) / d  # (B, N) >= 0
        ttc = np.where(
            closing > _EPS,
            np.maximum(clearance, 0.0) / np.maximum(closing, _EPS),
            np.inf,
        )
        # Linear iTTC risk: 0 once contact is further away than ttc_reference,
        # 1 at contact.  Clipping at a physical time keeps the risk horizon-aware
        # (a distant obstacle must not dominate a 0.3-0.6 s lookahead).
        risk = np.clip(1.0 - ttc / t_ref, 0.0, 1.0)              # (B, N)
        risk = np.where(clearance <= 0.0, 1.0, risk)             # penetrating -> 1
        return risk.max(axis=1), clearance

    risks = []
    clearances = []
    for t in times:
        risk_t, clearance_t = _risk_at(t)
        risks.append(risk_t)
        clearances.append(clearance_t)
    current_risk, _ = _risk_at(0.0)

    risk_stack = np.stack(risks)                 # (S, B)
    clearance_stack = np.stack(clearances)       # (S, B, N)
    risk_near, risk_mid, risk_far = risk_stack[0], risk_stack[1], risk_stack[2]
    risk_peak = risk_stack.max(axis=0)
    risk_trend = risk_far - risk_near

    # ---- candidate-specific lateral closing rate -------------------------
    # [v0.4.1-SideHit-ASE] closing_rate(a) = max_o max(0, -dot(r0,v_rel)/|r0|).
    # This is the ONLY place a "how fast is this candidate closing" scalar is
    # produced; because v_rel depends on the candidate, LEFT/RIGHT/DIAGONAL
    # genuinely differ here (unlike a state-level closing rate).
    r0_sq = (r0 * r0).sum(axis=-1)                               # (1, N)
    d0 = np.sqrt(np.maximum(r0_sq, _EPS))                        # (1, N)
    closing_obs = np.maximum(-(r0 * v_rel).sum(axis=-1), 0.0) / d0  # (B, N)
    closing_rate = closing_obs.max(axis=1)                       # (B,)
    dynamic_margin = float(base_margin) + closing_obs * float(margin_time)  # (B,N)

    margin_stack = clearance_stack - dynamic_margin[None, :, :]  # (S, B, N)
    min_margin = margin_stack.min(axis=(0, 2))                   # (B,)
    min_clearance = clearance_stack.min(axis=(0, 2))             # (B,)

    # ---- three-level adaptive safety envelope ----------------------------
    unsafe = safety.collides | (min_margin < 0.0)
    critical = (~unsafe) & (min_margin < float(critical_margin))
    envelope_level = np.where(unsafe, 2, np.where(critical, 1, 0)).astype(np.int64)

    # ---- side-hit dedicated critical risk --------------------------------
    max_speed = max(float(agent.max_speed), _EPS)
    rate_frac = closing_rate / max_speed
    rate_gate = np.clip(
        (rate_frac - float(lateral_rate_threshold_frac))
        / max(float(lateral_rate_scale_frac), _EPS),
        0.0,
        1.0,
    )
    margin_term = np.clip(
        (float(critical_margin) - min_margin) / max(float(critical_margin), _EPS),
        0.0,
        1.0,
    )
    ttc_term = np.where(
        np.isfinite(safety.min_ttc),
        np.clip(1.0 - safety.min_ttc / t_ref, 0.0, 1.0),
        0.0,
    )
    trend_term = np.clip(risk_trend * float(trend_gain), 0.0, 1.0)
    lateral_critical_risk = np.clip(
        rate_gate * np.maximum.reduce([margin_term, ttc_term, risk_peak, trend_term]),
        0.0,
        1.0,
    )

    return CandidateRiskProfile(
        current_risk=current_risk,
        risk_near=risk_near,
        risk_mid=risk_mid,
        risk_far=risk_far,
        risk_peak=risk_peak,
        risk_trend=risk_trend,
        min_ttc=safety.min_ttc,
        min_clearance=min_clearance,
        closing_rate=closing_rate,
        min_margin=min_margin,
        envelope_level=envelope_level,
        lateral_critical_risk=lateral_critical_risk,
        feasible=~unsafe,
        sample_times=times,
    )


def adaptive_safety_envelope(profile: CandidateRiskProfile) -> SafetyEnvelope:
    """Extract the three-level envelope from a risk profile (no recomputation)."""
    level = profile.envelope_level
    return SafetyEnvelope(
        level=level,
        min_margin=profile.min_margin,
        closing_rate=profile.closing_rate,
        critical=level == 1,
        unsafe=level >= 2,
        lateral_critical_risk=profile.lateral_critical_risk,
    )


# --------------------------------------------------------------------------
# critics
# --------------------------------------------------------------------------
def critical_risk_critic(
    profile: CandidateRiskProfile,
    envelope: SafetyEnvelope | None = None,
    *,
    critical_penalty: np.ndarray | None = None,
) -> np.ndarray:
    """LEVEL 2 critical dynamic safety score in ``[0, 1]``.

    # [v0.4.1-SideHit-ASE]
    # PURPOSE:
    #   Give fast lateral approaches (and the configurable near-collision penalty)
    #   their own hierarchy tier, *above* ordinary future risk, without turning
    #   them into hard infeasibility.  This is the "critical near-collision"
    #   layer that Nav2 separates from a true collision.
    #
    # OPEN-SOURCE REFERENCE:
    #   Nav2 MPPI critic layering (critical/near-collision critic); CommonRoad
    #   (criticality distinct from collision).
    #
    # ALGORITHM:
    #   score = max(lateral_critical_risk, level == 1, normalised critical penalty)
    #
    # FPGA MAPPING:
    #   Candidate-wise max-reduce over 2-3 fixed features.
    #
    # CPU ROLE:
    #   Feeds the second hierarchy tier; the CPU still owns argmin.
    #
    # COMPLEXITY:
    #   O(B).
    # """
    if envelope is None:
        envelope = adaptive_safety_envelope(profile)
    level = envelope.level
    if level.shape[0] == 0:
        return np.zeros(0, dtype=np.float64)
    critical_flag = (level == 1).astype(np.float64)
    components = [envelope.lateral_critical_risk, critical_flag]
    if critical_penalty is not None:
        components.append(np.asarray(critical_penalty, dtype=np.float64))
    return np.clip(np.maximum.reduce(components), 0.0, 1.0)


def future_risk_critic(
    profile: CandidateRiskProfile,
    actions: Sequence[Action],
    max_speed: float,
    *,
    ttc_reference: float = 1.0,
    near_miss_threshold: float = 10.0,
    critical_penalty: np.ndarray | None = None,
    trend_gain: float = 1.0,
    speed_risk_gain: float = 1.0,
) -> np.ndarray:
    """Per-candidate future-risk score in ``[0, 1]`` (LEVEL 3 critic).

    # [v0.4.0-FAR] with the v0.4.1 threat-aware speed coupling.
    # PURPOSE:
    #   Collapse min_ttc / min_clearance / risk_peak / risk_trend and a
    #   threat-aware speed coupling into one bounded score, so the planner ranks
    #   by *future* danger instead of only the current risk.
    #
    # OPEN-SOURCE REFERENCE:
    #   Nav2 MPPI critic layering and F1TENTH iTTC; max-aggregation keeps it a
    #   single compare/reduce stage.  Structure only.
    #
    # ALGORITHM:
    #   threat = max(risk_peak, clip(1 - min_ttc/ttc_ref,0,1),
    #                clip(1 - min_clearance/near_miss,0,1))
    #   score  = max(current_risk, risk_peak, ttc, clearance, trend+,
    #                clip(speed_fraction * threat * speed_risk_gain, 0, 1))
    #
    #   The speed coupling is a continuous penalty: it is exactly 0 in a
    #   low-threat scene (so FAST is never banned for its own sake) and grows
    #   with threat, so FAST costs more only when the future is dangerous.
    #
    # FPGA MAPPING:
    #   Candidate-wise max-reduce over a fixed feature set; one multiply for the
    #   speed coupling.
    #
    # CPU ROLE:
    #   Produces the future-risk tier; the CPU still owns final selection.
    #
    # COMPLEXITY:
    #   O(B).
    # """
    b = profile.risk_peak.shape[0]
    if b == 0:
        return np.zeros(0, dtype=np.float64)

    t_ref = max(float(ttc_reference), _EPS)
    ttc_component = np.clip(1.0 - profile.min_ttc / t_ref, 0.0, 1.0)
    ttc_component = np.where(np.isfinite(profile.min_ttc), ttc_component, 0.0)
    clearance_component = np.clip(
        1.0 - profile.min_clearance / max(float(near_miss_threshold), _EPS), 0.0, 1.0
    )
    trend_component = np.clip(profile.risk_trend * float(trend_gain), 0.0, 1.0)

    threat = np.maximum.reduce(
        [profile.risk_peak, ttc_component, clearance_component]
    )
    speeds = np.asarray([a.speed for a in actions], dtype=np.float64)
    speed_fraction = speeds / max(float(max_speed), _EPS)
    speed_component = np.clip(
        speed_fraction * threat * float(speed_risk_gain), 0.0, 1.0
    )

    components = [
        profile.current_risk,
        profile.risk_peak,
        ttc_component,
        clearance_component,
        trend_component,
        speed_component,
    ]
    if critical_penalty is not None:
        components.append(np.asarray(critical_penalty, dtype=np.float64))
    return np.clip(np.maximum.reduce(components), 0.0, 1.0)

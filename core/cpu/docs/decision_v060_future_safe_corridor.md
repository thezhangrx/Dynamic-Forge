# v0.6.0-SafeCorridor-Shadow — SafeRect + Reachability + Future Safe Corridor

## 0. Status

**Shadow prototype, default OFF.** `core/cpu/decision/future_corridor.py`
is not imported by `planner.py` / `core.py`, is not in `FINGERPRINT_FILES`, and
cannot emit an `Action`. The official controller stays `v0.5.2-LTVGuarded`.

Master switch: `SAFE_CORRIDOR_SHADOW_ENABLED` (env
`DF_ENABLE_SAFE_CORRIDOR_SHADOW`, default `false`). While disabled `observe()`
returns `None` and computes nothing, so the module can be wired in
unconditionally and still produce the exact v0.5.2 action.

## 1. Hypothesis

v0.5.1/v0.5.2 summarise the future with one scalar (`viability`). FSC asks whether
a **time-indexed spatial structure** — three future safe regions plus the
reachability relation between them — carries more usable decision information,
especially for `side_hit`, `corner_trap` and dead-end convergence.

This round studies **option 3 (SafeRect + Reachability)** and **option 4 (Future
Safe Corridor)** only. It does not replace v0.5.2.

## 2. Open-source references (ideas only, no planner copied)

| project | borrowed idea |
|---|---|
| Nav2 MPPI | layered collision / critical / preference critics; a hard safety constraint is never overridden by an ordinary preference |
| F1TENTH Follow-the-Gap | continuous free-space gap; a safe bubble, not just the nearest obstacle |
| CommonRoad-Reach | future reachable space / driving corridor as first-class information (no reachability solver) |
| TEB | finite route/corridor, route persistence, hysteresis — no route switch on a momentary change |
| Open-SPITE | box-shaped geometric approximation for fast safety screening |

Explicitly not implemented: full MPPI / TEB / CommonRoad reachability, A*, BFS, or
any occupancy-grid global search.

## 3. Time anchors

Fixed and unchanged: `t1 = 0.30 s`, `t2 = 0.55 s`, `t3 = 0.80 s`. The probe passes
the planner's own `long_anchors` when present so the comparison is exact. No extra
anchors, no change to the v0.5.2 anchor computation.

## 4. SafeMask (8-sector, reused geometry)

The 8-sector reduction reuses the **exact** v0.5.2 kernels (`_sector_summary_batch`,
`_circular_max_run`) and the identical dynamic-buffer formula, so FSC can never
call a sector free that LTV mobility would call blocked:

```
dynamic[d] = min(base_buffer + closing[d]*margin_time, max_dynamic_buffer)
free[d]    = (nearest_clearance[d] - dynamic[d]) >= 0
safe_mask  = pack(free)                        # bit d = sector d
max_run, run_start = circular longest run of free
persistent_mask = mask_t1 & mask_t2 & mask_t3  # bitwise AND
overlap_12 = popcount(mask_t1 & mask_t2); overlap_23 = popcount(mask_t2 & mask_t3)
```

## 5. SafeRect

`R = (xmin, xmax, ymin, ymax)` around the candidate's future position `(px, py)`.
Obstacles are inflated by `radius + agent_radius + base_buffer` (the box bounds the
agent **centre**). The rule is deliberately simple — no polygon clipping, no convex
hull, no optimiser:

```
xmin = px - left_clearance      xmax = px + right_clearance
ymin = py - down_clearance      ymax = py + up_clearance
```

where each clearance is the distance to the nearest inflated box that blocks that
axis line (capped by `max_rect_half`). Because the row/column rule cannot see a
purely diagonal obstacle, the box is then **verified** against every inflated box.
If the check fails, the box is rejected rather than silently including obstacle
area. `safe_rect_invalid_reason` records why:

| reason | meaning |
|---|---|
| `ok` | proven obstacle-free |
| `no_obstacles` | empty scene, capped box |
| `centre_blocked` | an inflated box contains the future position |
| `diagonal_overlap` | row/column rule insufficient; verification rejected it |
| `degenerate` | smaller than `2*min_rect_half` on an axis |

This is a conservative approximation, **not** the maximal safe region.

## 6. Reachability

```
D12 = v_max*(t2-t1);  D23 = v_max*(t3-t2)
reach_12 = intersect(Expand(R1, D12), R2)
reach_23 = intersect(Expand(R2, D23), R3)
```

Add / subtract / compare only — no sqrt, no atan2, no A*, no BFS, no optimiser.
An invalid rect can never be reachable. Touching counts as intersecting.
This is what distinguishes *"safe space exists"* from *"the agent can get there in
time"* (toy scenario D).

## 7. Corridor classification

Enum: `NONE`, `UNKNOWN`, `STABLE`, `DEAD_END`, `UNSTABLE` (previous `*_FUNNEL`
names remain as aliases). Order: no evidence beats unknown beats dead end beats
unstable beats stable.

| class | condition |
|---|---|
| `NONE` | no evidence to classify at all: `max_run(t1) < min_run` |
| `UNKNOWN` | future information exists, but at least one conservative SafeRect could not be **proven** — so neither safe nor dead end is established |
| `DEAD_END` | all rectangles proven **and** a proof exists: `reach_12`/`reach_23` fails, or t3 has no continuous exit, or t3 corridor clearance `< t3_clearance_floor`, or rapid collapse (`max_run(t1) - max_run(t3) >= rapid_shrink_run_drop`) with no persistent direction |
| `UNSTABLE` | rectangles valid, but the corridor jumps: direction jump `>= unstable_direction_jump`, or SafeMask Hamming jump `>= unstable_mask_jump`, or `overlap_12/23 < unstable_min_overlap` |
| `STABLE` | `persistent_mask` has `>= min_persistent_sectors`, reaches hold, every anchor keeps `>= min_run` sectors and `>= clearance_threshold` clearance |

**"Cannot prove safe" is not "proved dead end."** `safe_rect_invalid_reason` can
only ever produce `UNKNOWN`; it can never produce `DEAD_END`. This is enforced by
the ordering above (the rectangle-validity test runs before the reachability test)
and pinned by tests.

A shrinking region is never by itself a failure — that is the *stable narrowing*
vs *dead-end narrowing* distinction.

## 7b. The planner's real Top-K (read-only observation)

`PlanDiagnostics` now carries four observation-only fields, populated in
`plan()` after the LTV evaluation and read by nothing in the control path:

| field | meaning |
|---|---|
| `fsc_candidate_indices` | the real shortlist the planner used (3 leaders + 1 diversity, planner order), `<= 4` entries |
| `fsc_candidate_actions` | those candidates as `Action`s (indices refer to the planner's own array, which may include the `<=8` refinement actions) |
| `fsc_candidate_sources` | `"leader"` / `"diversity"` per entry |
| `fsc_candidate_source` | `"ltv_shortlist"` / `"ltv_skipped"` / `"ltv_disabled"` / `"no_feasible_candidate"` |

The ordering reproduces `LongTermViabilityEvaluator.shortlist`'s traversal key
(`critical_band`, `risk_band`, index) and only *reads* the mask the evaluator
returned; a test spies on the evaluator and asserts the exposed set equals the
real shortlist exactly. `viability.py` is **not** modified.

The FSC shadow prefers `candidate_indices` (`candidate_source="planner_topk"`);
only when the planner produced none (LTV fast-path skip or `ENABLE_LTV=False`) does
it fall back to its own proxy and say so (`candidate_source="shadow_proxy"`). The
chosen candidate is always evaluated as well, so `chosen` is never a stand-in, but
it is not part of the Top-K list.

## 8. Future target and pre-positioning

* `future_target` = centre of `R3`, only when `R3` is valid **and** usable;
  otherwise `future_target_valid = false`. Diagnostics only.
* `corridor_match_t{1,2,3}` = the candidate's own `p(t_k)` inside a **valid** `R_k`;
  `corridor_match_count` is their sum. No score, no weight, not fed to the planner.

## 8b. DSS v6.0 shadow — where `corridor_type` lives

`DSSv6Shadow.to_dict()` = `header` + `global_safety` + `candidate_risk[17]` +
`future_space[17][3]`.

* `candidate_risk[i].corridor_type` (and its `fsc_corridor_type` twin) is the
  candidate's corridor class; it is one of `NONE / UNKNOWN / STABLE / DEAD_END /
  UNSTABLE`.
* **every** `future_space[i][k]` anchor row also carries `corridor_type`, so a
  consumer never has to join back to `candidate_risk` to know the verdict.
* each anchor row keeps `safe_rect_valid` **and** `safe_rect_invalid_reason`, which
  is what makes `UNKNOWN` ("the algorithm does not know") distinguishable from
  `DEAD_END` ("the algorithm proved a dead end").
* a candidate that was not evaluated keeps an empty `{}` row rather than a fake
  verdict.

## 9. Toy scenarios (pytest)

| scenario | construction | expected |
|---|---|---|
| A stable | R1 large → R2 medium → R3 smaller, both reaches true, persistent mask 0b00000011 | `STABLE` |
| B dead end | R1 large → R2 small → R3 tiny/unreachable | `DEAD_END` |
| C unstable | masks `11110000` → `00111100` → `00001111` | `UNSTABLE` |
| D reachability failure | all three regions valid and non-empty, but the speed budget cannot connect them | `DEAD_END` |
| E safe rect invalid | masks show space, but one SafeRect cannot be proven | `UNKNOWN` |

## 10. Probe evidence — 3 episodes, seed 42–44 (association only)

After the `UNKNOWN` fix the picture is materially different from the previous
round, and the previous round's headline was an artifact of the bug:

```
frames 2850        STABLE 864 (30.3%)   UNKNOWN 1986 (69.7%)
                   DEAD_END 0            UNSTABLE 0            NONE 0
frames with all 3 rects provable: 864 -> corridor_type = STABLE 864 / 864
invalid-rect reasons: diagonal_overlap t1 1552, t2 1421, t3 1215; centre_blocked 53
candidate_source: planner_topk 2481 (87.1%), shadow_proxy 369 (12.9%)
planner Top-K sizes: 4 -> 2418, 0 -> 369, 3 -> 27, 2 -> 26, 1 -> 10   (all <= 4)
cost: safe_rect 242 us | reachability 8 us | corridor 9 us | total 1401 us
```

**Honest reading.** In the previous round "DEAD_END before collision = 100%" was
produced by counting *unprovable* rectangles as dead ends. Once `UNKNOWN` is
separated out, this sample contains **zero** provable dead ends: whenever the
conservative SafeRect could be proven at all three anchors, the corridor was
`STABLE`, and in ~70% of frames the rectangle was not provable (overwhelmingly
`diagonal_overlap`), which is now reported as `UNKNOWN`. So the current classifier
does **not** yet identify dead ends in real scenes — the bottleneck is the
conservative rectangle rule, not the corridor logic. Three episodes is far too
small to generalise; this is a diagnostic finding, not a result.

## 11. Historical data

No saved artifact contains per-frame FSC data (the episode JSONL schema from the
previous round carries no FSC fields), so the requested 1000-game frame-level
analysis **cannot be computed from existing data** and is not fabricated. The probe
produces fresh per-frame JSONL instead.

## 12. How to run

```bash
cd /home/wei/Dynamic-Forge
.venv/bin/python tools/fsc_shadow_probe.py --episodes 5 --seed 42 --mode shadow \
    --output /tmp/fsc_frames.jsonl
# env-var form of the shadow switch:
DF_ENABLE_SAFE_CORRIDOR_SHADOW=1 .venv/bin/python tools/fsc_shadow_probe.py \
    --episodes 5 --seed 42 --mode shadow
```

## 13. Not changed

No edit to `viability.py`, `build.py`, `core.py`, FAR, ASE, SafetyGate, FES,
dwell, adaptive horizon/smooth, the 17 candidates, Top-K=4, 3 leaders + 1 diversity,
LTV, the LTV override guard, `risk_band_width`, the 3 anchors, corridor hysteresis,
action hysteresis, EmergencyFallback or `max_dynamic_buffer`. No threshold default
was changed. No 1000-episode run, no push.

`planner.py` received exactly one **observation-only** addition — the four
`fsc_candidate_*` diagnostics fields and the block that fills them. It reads
already-computed arrays and writes only to `PlanDiagnostics`; no control variable
is assigned and no control expression reads these fields. Because `planner.py` is
in `FINGERPRINT_FILES`, the control BUILD_ID legitimately changed to
`cpudec-1b04f4b8c1f5`; `fingerprint() == BUILD_ID` still holds.


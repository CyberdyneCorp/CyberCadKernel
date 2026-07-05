## Why

`SSI-ROADMAP.md` S4 is **the moat** — tangent / degeneracy robustness. S4-a/S4-b landed the
CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`); S4-c MARCHES THROUGH a
`NearTangentTransversal` single-branch **graze** (the SURFACE PAIR grazes, `n₁×n₂ → 0`, but
the curve continues); S4-d LOCALIZES a **branch point** (the intersection LOCUS self-crosses —
the Steinmetz saddle where four arms meet) and routes the arms. **S4-e is a DIFFERENT
degeneracy: a single SURFACE's own PARAMETRIZATION is singular** — a sphere PARAMETRIC POLE
(`v = ±π/2`, where `Sphere::dU` collapses to zero because of the `cos v` factor) or a cone
APEX (where the signed radius `R₀ + v·sin α = 0`, so `Cone::dU` collapses), OR the intersection
curve itself has a CUSP (its own velocity `→ 0`). **The 3D point and the surface NORMAL are
perfectly well-defined at all of these** — only the `(u,v)` CHART degenerates (`‖dU‖ → 0` while
the normal stays finite). The intersection can be perfectly TRANSVERSAL through a pole
(`‖n₁ × n₂‖` need NOT collapse), which is exactly why S4-e is NOT S4-c and NOT S4-d.

The marcher advances via each surface's own `(u,v)` tangent plane (`advanceParams` solves the
2×2 `[dU dV]ᵀ[dU dV]` normal equations). At a pole/apex that single-surface Jacobian is
**rank-deficient** (its `dU` row vanishes), so the param-space update is ill-conditioned even
though the 3D corrector residual is fine. Worse, the pole/apex sits on a NON-PERIODIC `v`
edge (sphere `v ∈ [−π/2, π/2]`, cone `v` linear), so the current marcher treats a
curve that should CROSS the pole as a `BoundaryExit` and TRUNCATES there.

Diagnosed on the CURRENT marcher (host, `CYBERCAD_HAS_NUMSCI` ON, `trace_from_seeds` forcing
MARCHING — analytic pairs skip marching via S1, so the fixtures are seeded directly):

| Fixture (marched) | S3+S4-c result today | Verdict |
|---|---|---|
| **SPHERE POLE** — unit sphere ∩ plane `y = 0` (great circle through BOTH poles `v = ±π/2`) | one WLine, `status = BoundaryExit`, **189 pts**, **arcLen ≈ 3.1415 (= π, HALF the loop)**, `u1 ∈ [0,0]` (never reaches the `u = π` meridian), `nearTangentGaps = 0`, worst on-surf residual `2.95e-12` | **TRUNCATES at the pole** — the full closed great circle (circumference `2π ≈ 6.283`) is cut to a single pole-to-pole meridian; the curve is transversal + on-surface throughout, only the chart broke |
| **CONE APEX** — double cone (`R₀ = 0` at apex, α = 45°) ∩ plane `y = 0` (apex-crossing line spanning `v ∈ [−2, +2]`) | one WLine, `status = BoundaryExit`, **20042 pts (hit maxPoints)**, `v1 ∈ [−0.036, 2.0]` (stalls just short of the apex `v = 0`, never reaches the `v < 0` nappe), worst residual `1.62e-10` | **STEP COLLAPSE at the apex** — `dU → 0` makes the param step crawl, burning the whole node budget crossing `v ≈ 0`, then stops short of the apex; the far nappe is never traced |
| 5 transversal S3 pairs | `nt = 0`, bit-identical | **must stay UNTOUCHED** |
| S4-c crossable graze (sphere / offset cylinder) | `nearTangentCrossed ≥ 1`, `nearTangentGaps = 0`, full loop | **must stay CROSSED** |
| S4-d Steinmetz bicylinder | `branchPoints = 2`, four arms assembled | **must stay TRACED** |

Both failures are the SAME root cause — the single-surface chart Jacobian goes rank-deficient
at the pole/apex — and both are honestly recoverable BECAUSE the surface point + normal stay
finite: the intersection tangent `t = normalize(n₁ × n₂)` is still well-defined, and a
POINT-BASED corrector that does not depend on the degenerate `dU` can step across the singular
band. This change adds the FIRST HONEST S4-e SLICE: DETECT the single-surface chart singularity
(`‖dU‖` collapse vs `‖dV‖·scale`; a curve cusp is the CURVE velocity collapsing while both
surfaces are regular), STEP across the singular band with a point-based / chart-clamped
corrector that uses the finite point + normal, then RESUME the normal `(u,v)` march once `‖dU‖`
recovers. Anything not robustly steppable + verified across the singularity STILL DEFERS
(honest `NearTangent`/truncated stop + typed reason → OCCT). It NEVER fabricates a point across
a singularity and NEVER weakens a tolerance.

## What Changes

- **(S4-e-1) SINGLE-SURFACE CHART-SINGULARITY DETECTION.** Along the march, on EACH surface,
  watch the single-surface chart conditioning: finite-difference `‖dU‖` relative to `‖dV‖`
  and the model scale. A collapse (`‖dU‖ < chartCollapseFrac · ‖dV‖`, with `‖dU‖` small
  vs scale) while the surface NORMAL stays finite flags a POLE/APEX approach on THAT surface —
  distinct from the PAIR transversality sine `‖n₁ × n₂‖` (which need NOT collapse at a pole).
  A CURVE CUSP is flagged separately: the intersection tangent's raw velocity collapses (the
  predicted chord shrinks toward zero) while BOTH surfaces' charts are regular. The detector
  reuses `advanceParams`' own finite-difference `dU`/`dV`, so it adds no new surface query
  contract. This is a NEW, INDEPENDENT witness — it does NOT reuse the S4-c sine-collapse or the
  S4-d tangent-flip seam (those fire on pair/locus degeneracies, not chart degeneracies).
- **(S4-e-2) POINT-BASED / CHART-CLAMPED CORRECTOR ACROSS THE SINGULAR BAND.** Because the
  surface point + normal stay finite, correct the node with a constraint that does NOT rely on
  the degenerate `dU`: reuse the S4-c FIXED-PLANE cut (the `branch_point.h`-style point-based
  reproject with an along-`t★` hyperplane residual) — the least-squares solve drives the 3D
  residual `A.point − B.point → 0` and the along-plane advance, neither of which needs a
  full-rank single-surface Jacobian. Map back to `(u,v)` only LOOSELY: at a SPHERE POLE the
  longitude `u` is ARBITRARY (the whole `u` circle maps to one point), so pin the outgoing `u`
  from CONTINUITY of the incoming arc (the great circle continues on the `u_in + π` meridian);
  the pole `v = ±π/2` is clamped. At a CONE APEX treat the apex as a SINGLE 3D point the curve
  may pass through, re-seeding the far-side `(u,v)` from the continued 3D tangent. The
  crossing is a bounded fine-step walk exactly like the S4-c band, but triggered by the CHART
  witness and using the point-based corrector so the degenerate `dU` never enters the solve.
- **(S4-e-3) STEP CONTROL ACROSS THE BAND.** Enter the singular band with a FINE step (the same
  discipline as S4-c) so the pole/apex is RESOLVED rather than leapt; do NOT let the ordinary
  `advanceParams` step-shrink loop crawl (the 20042-point apex pathology) — the chart witness
  short-circuits into the point-based crossing, which advances by a fixed fine step off the
  singular point along the continued tangent, then hands back to the normal S3 march once
  `‖dU‖` recovers above the collapse threshold on both surfaces. A `v`-edge that is a genuine
  DOMAIN boundary (not a parametric pole/apex — e.g. a finite cylinder's cap) still exits as
  `BoundaryExit`, unchanged: the pole/apex is distinguished by the `‖dU‖` collapse WITH a
  finite normal, a boundary is not.
- **(S4-e-4) HONEST GUARD.** The crossing is emitted ONLY if every node across the singular band
  verifies on BOTH surfaces `≤ onSurfTol` AND the march makes real progress off the singular
  point on the far side. If the point-based corrector cannot land a verified far-side node
  (the crossing is not robust — e.g. a cusp where the curve genuinely ends, or a degeneracy the
  point-based cut cannot resolve), the arc is DISCARDED and the march STILL STOPS + defers
  (truncated `NearTangent`/`BoundaryExit` + typed reason → OCCT), reporting the MEASURED gap.
  A curve CUSP (curve velocity → 0 with both charts regular) that is a genuine curve endpoint
  ENDS; only a cusp the point-based step can continue through is crossed, and only if verified.
- **Marching result carries the singularity-crossing outcome (additive).** `marching.h` gains a
  per-`TraceSet` `singularitiesCrossed` count (chart poles/apexes the marcher STEPPED ACROSS
  and verified) and a per-`WLine` `chartSingularCrossed` count; a pole/apex the marcher
  crosses no longer truncates as a spurious `BoundaryExit`. No `cc_*` ABI change.

## Capabilities

### New Capabilities
<!-- none — this change EXTENDS the living native-ssi capability with the FIRST S4-e
chart-singularity slice (single-surface chart-collapse detection + point-based crossing of a
sphere pole / cone apex + honest defer). It adds no new capability spec and no cc_* ABI. -->

### Modified Capabilities
- `native-ssi`: add (S4-e) a native, OCCT-free **chart-singularity** capability that, when a
  marched intersection curve crosses a SINGLE SURFACE's own PARAMETRIC SINGULARITY (a sphere
  parametric pole `v = ±π/2` where `‖dU‖ → 0`, or a cone apex where the signed radius `→ 0`)
  — where the 3D point and surface normal are well-defined but the `(u,v)` chart degenerates
  and the current marcher TRUNCATES (a half-traced great circle, an apex step-collapse) —
  DETECTS the chart collapse via a SINGLE-surface Jacobian rank-drop (`‖dU‖` collapse vs
  `‖dV‖·scale`, NOT the pair transversality sine), STEPS across the singular band with a
  point-based / chart-clamped corrector (using the finite point + normal, pinning the arbitrary
  pole longitude from arc continuity / treating the apex as one 3D point), and RESUMES the
  normal `(u,v)` march on the far side — reporting a `singularitiesCrossed` count. A crossing
  that cannot be robustly verified on both surfaces STILL STOPS + defers → OCCT (never a
  fabricated pole-crossing point); a genuine curve CUSP endpoint ENDS. The transversal S3
  trace, the S4-c crossable graze, and the S4-d branch-point trace are UNTOUCHED. The tracer
  NEVER fabricates a point across a singularity and NEVER weakens a tolerance; a singularity it
  cannot cross is an honestly reported gap. No `cc_*` ABI change; `src/native/**` stays
  OCCT-free; the S4-e machinery is compiled under `CYBERCAD_HAS_NUMSCI` like S2/S3/S4-c/S4-d.

## Impact

- **ABI**: none. SSI is INTERNAL — no `cc_*` entry point, signature, or POD struct change.
  Additive only; the tessellator (`src/native/tessellate`) and the CyberCad app are untouched.
- **Build**: extends `src/native/ssi/marching.{h,cpp}` (a new `chartCondition` single-surface
  witness, a `crossChartSingularity` point-based crossing driver modelled on the S4-c
  `crossNearTangent`, its band-entry hook in `marchDir`, and the additive
  `TraceSet.singularitiesCrossed` / `WLine.chartSingularCrossed`), plus a new OCCT-free
  `src/native/ssi/chart_singularity.h` isolating the detection + pole-longitude-continuity /
  apex-single-point mapping math (like `branch_point.h` isolates the S4-d branch math), under
  `CYBERCAD_HAS_NUMSCI` like the S3/S4-c/S4-d marcher. It reuses the S4-c fixed-plane
  point-based reproject (`branchpt::reproject` / the `correct` along-`t★` cut) unchanged. No
  change to `src/native/tessellate`; no new substrate routine; no new tolerance beyond the
  chart discriminators (`chartCollapseFrac`, band step fractions — documented, sentinel-resolved,
  never weakening `onSurfTol` / `tangentSinTol` / `minStep` / `maxDeflection`).
- **Verification**: two gates. **Host (no OCCT)** — extend
  `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4e_singularities.cpp`), all under `CYBERCAD_HAS_NUMSCI`: the
  SPHERE-POLE great circle that S3 today truncates (arcLen ≈ π, one meridian) is now FULLY
  traced — the full closed loop (arcLen ≈ `2π`, both `u = 0` and `u = π` meridians visited,
  crossing both poles), `singularitiesCrossed ≥ 2`, every node on BOTH surfaces ≤ `onSurfTol`;
  the CONE-APEX line that S3 today stalls on (20042 pts, stops at `v ≈ 0`) is now traced across
  the apex (spanning both nappes `v ∈ [−2, +2]` in a bounded node count), `singularitiesCrossed
  ≥ 1`, every node ≤ `onSurfTol`; a genuine DOMAIN-boundary `v`-edge (finite cylinder cap) STILL
  exits as `BoundaryExit` (`singularitiesCrossed = 0`); the 5 transversal pairs trace
  bit-identically (`nt = 0`), the S4-c graze STILL crosses, the S4-d Steinmetz STILL traces
  (`branchPoints = 2`). Full CTest green NUMSCI ON and OFF (the S4-e assertions absent with
  NUMSCI off, like S2/S3/S4-c/S4-d). No OCCT linked; no tolerance weakened. **Sim
  native-vs-OCCT (booted sim)** — extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm` (or a new `scripts/run-sim-native-ssi-s4e.sh`):
  assert the sphere-pole great circle and the cone-apex line are now FULLY traced natively —
  crossing the pole/apex — matching OCCT `IntPatch` / `GeomAPI_IntSS`: every sampled native node
  on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both surfaces ≤
  `onSurfTol`; the native arc length / loop closure reconciles with the OCCT curve within tol;
  the native curve passes through the OCCT pole/apex point to `tol`. Report per-pair the
  crossed-vs-still-deferred count. `xcrun simctl list devices booted`.
- **Roadmap**: advances `SSI-ROADMAP.md` S4 from the S4-d branch-point slice to the FIRST S4-e
  SINGULARITY slice — the sphere-pole / cone-apex crossing S3 truncated is now fully traced vs
  OCCT. **Explicitly a first slice:** general / freeform parametric singularities, edge
  singularities, higher-order cusps, and self-intersection completeness (S4-f) remain OUT OF
  SCOPE, and any singularity not robustly crossable + verifiable still defers → OCCT.
- **Risk (honest)**: (a) the chart witness could FALSE-FIRE at a genuine domain boundary (a
  finite cap's `v`-edge) and try to "cross" past the surface — mitigated by requiring the
  `‖dU‖` collapse WITH a finite normal AND a re-projectable far-side node; a true boundary has
  no far side to land on, so the crossing fails its verification and the march exits
  `BoundaryExit` as today; the finite-cylinder-cap control is the "must-still-exit" pin. (b) The
  pole longitude is ARBITRARY, so a wrong continuity pick sends the far arc down the wrong
  meridian — mitigated by pinning `u_out` from the incoming arc's great-circle continuity
  (`u_in + π` for a pole) and VERIFYING the first far-side node on both surfaces ≤ `onSurfTol`
  before accepting; a mis-pinned node does not verify and the crossing defers. (c) The
  point-based cut could DRIFT along the arbitrary-`u` null direction at the pole — mitigated by
  the fixed-plane along-`t★` residual (the same well-posed-as-sine→0 cut S4-c uses) plus the
  clamp of the pole `v` coordinate. (d) Crossing must not perturb the transversal S3 trace, the
  S4-c graze, or the S4-d branch trace — mitigated by leaving the S3 corrector / deflection
  controller, the S4-c crossing driver, and the S4-d branch machinery bit-identical and
  engaging the chart machinery ONLY at a detected single-surface chart collapse (a NEW,
  independent witness), pinned by the 5 transversal pairs + the S4-c graze + the S4-d Steinmetz
  staying green. Whatever does not resolve robustly still stops + defers → OCCT and is reported
  with the measured gap; no point is faked, hand-tuned, or weakened to pass.

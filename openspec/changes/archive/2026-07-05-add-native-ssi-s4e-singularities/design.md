# Design — add-native-ssi-s4e-singularities (SSI Stage S4-e, first slice)

## Context

The native SSI stack is S1 analytic → S2 seeding → S3 marching, hardened by the S4-a/S4-b
CLASSIFICATION layers (typed `CoincidentRegion` + `TangentContact`), the S4-c near-tangent
MARCHING-THROUGH slice (a `NearTangentTransversal` single-branch graze marched through a
fixed-plane-cut corrector), and the S4-d BRANCH-POINT slice (the Steinmetz self-crossing locus
localized + arm-routed). **S4-e is a DISTINCT degeneracy from all three: a single SURFACE's own
PARAMETRIZATION is singular.**

The three prior failures and why S4-e is none of them:

- **S4-c near-tangent** = the SURFACE PAIR grazes (`‖n₁ × n₂‖ → 0`), the curve may continue.
  Witnessed by the transversality SINE collapse.
- **S4-d branch point** = the intersection LOCUS self-crosses (multiple arms at one point).
  Witnessed by the sine collapse PLUS a raw-tangent FLIP.
- **S4-e chart singularity** = ONE surface's `(u,v)` chart degenerates (`‖dU‖ → 0`) at a sphere
  pole (`v = ±π/2`, factor `cos v`) or a cone apex (signed radius `R₀ + v·sin α = 0`), OR the
  CURVE itself has a cusp (curve velocity `→ 0`). **The surface point + normal are FINE**; the
  intersection can be perfectly TRANSVERSAL through the pole (`‖n₁ × n₂‖` need NOT collapse).
  Witnessed by a SINGLE-surface Jacobian rank-drop — NOT the pair sine, NOT a locus flip.

Confirm from `src/native/math/elementary.h`:

- `Sphere::dU(u,v) = pos.x·(−R cos v·sin u) + pos.y·(R cos v·cos u)` → `‖dU‖ = R·cos v` →
  **ZERO at `v = ±π/2`** (the poles). `Sphere::normal(u,v) = (S − center)/R` is radial and
  perfectly defined at the pole (points along ±axis). `Sphere::dV` has `‖dV‖ = R` everywhere.
- `Cone::dU(u,v) = ` (tangential, magnitude `|R₀ + v·sin α|`) → **ZERO at the apex** where
  `R₀ + v·sin α = 0`. `Cone::normal = normalize(dU × dV)` degenerates AT the apex point itself
  but is well-defined arbitrarily close to it; the apex is a single 3D point. `‖dV‖ = 1`.

So on BOTH surfaces the chart singularity is a `cos v`-type / signed-radius collapse of `‖dU‖`
while `‖dV‖` and the normal stay `O(scale)` — a REMOVABLE coordinate singularity.

### Diagnosis (the "before", confirmed on the current marcher)

Host build, `CYBERCAD_HAS_NUMSCI` ON, forcing MARCHING via `trace_from_seeds` with a hand seed
(analytic pairs skip marching via S1's closed-form dispatch, so a marched fixture is required):

| Fixture (marched) | S3+S4-c result today | Verdict |
|---|---|---|
| **SPHERE POLE** — unit sphere ∩ plane `y = 0` (great circle through both poles) | one WLine, `BoundaryExit`, **189 pts**, **arcLen ≈ 3.1415 = π (HALF the closed loop)**, `u1 ∈ [0, 0]` (only the `u = 0` meridian), `v1 ∈ [−π/2, +π/2]`, worst residual `2.95e-12` | **TRUNCATES at the pole** — the full closed great circle (`2π ≈ 6.283`) is cut to a pole-to-pole meridian; the curve is transversal + on-surface throughout, only the chart broke |
| **CONE APEX** — double cone (`R₀ = 0`, α = 45°) ∩ plane `y = 0` (apex-crossing line) | one WLine, `BoundaryExit`, **20042 pts (hit `maxPoints`)**, `v1 ∈ [−0.036, 2.0]` (stalls just short of the apex `v = 0`, never the `v < 0` nappe), worst residual `1.62e-10` | **STEP COLLAPSE at the apex** — `dU → 0` makes `advanceParams` crawl, exhausting the node budget crossing `v ≈ 0`, then stops short; the far nappe is never traced |
| 5 transversal S3 pairs | `nt = 0`, bit-identical | **must stay UNTOUCHED** |
| S4-c crossable graze | `nearTangentCrossed ≥ 1`, full loop | **must stay CROSSED** |
| S4-d Steinmetz bicylinder | `branchPoints = 2`, arms assembled | **must stay TRACED** |

The two failures share the ROOT CAUSE: `advanceParams` solves the single-surface 2×2 normal
equations `[dU dV]ᵀ[dU dV] (Δu,Δv) = [dU dV]ᵀ (h·t)`; at the pole/apex the `dU` row vanishes so
the 2×2 is rank-1 and the `(u,v)` update is ill-conditioned. Because the pole/apex also lies on
a NON-PERIODIC `v` edge (sphere `v ∈ [−π/2, π/2]`, cone `v` linear), `onBoundary` fires and the
march reports a spurious `BoundaryExit` (sphere), or the deflection/step-shrink loop crawls
against the ill-conditioning until the node budget is spent (cone). BUT the 3D corrector
residual is fine throughout (`2.95e-12` / `1.62e-10`), and the intersection tangent
`t = normalize(n₁ × n₂)` stays well-defined — so the crossing is honestly recoverable with a
corrector that does NOT touch the degenerate `dU`.

The method is clean-room; OCCT (`IntPatch` / `IntWalk` / `GeomAPI_IntSS`) is the verification
ORACLE only — the chart-collapse detection, the pole-longitude continuity map, and the apex
single-point handling are re-derived, never copied.

**Honest nuance (scope-setting).** S4-e's FIRST slice targets the ELEMENTARY REMOVABLE chart
singularities: a SPHERE parametric pole (`v = ±π/2`) and a CONE apex (signed radius `= 0`) on a
marched intersection curve, plus a curve CUSP only when the point-based step can continue
through it and every node verifies. General / freeform parametric singularities (NURBS
degenerate edges, seam singularities), edge / higher-order singularities, higher-order cusps,
and S4-f self-intersection completeness are OUT OF SCOPE and DEFER. A genuine DOMAIN boundary
(a finite cap's `v`-edge — `‖dU‖` does NOT collapse there) still exits `BoundaryExit`, unchanged.

## Goals / Non-Goals

**Goals**
- (S4-e-1) DETECT a single-surface chart singularity along the march: `‖dU‖` collapse relative
  to `‖dV‖·scale` on either surface with a FINITE normal (pole/apex), or a CURVE-velocity
  collapse with both charts regular (cusp). A NEW, independent witness — not the S4-c sine
  collapse or the S4-d tangent flip.
- (S4-e-2) STEP across the singular band with a POINT-BASED / chart-clamped corrector (the S4-c
  fixed-plane cut, which needs only the finite point + normal), pinning the arbitrary pole
  longitude from arc continuity and treating the cone apex as one 3D point.
- (S4-e-3) STEP CONTROL: enter the band FINE (resolve, don't leap), short-circuit the
  `advanceParams` crawl, resume the normal S3 march once `‖dU‖` recovers on both surfaces.
- (S4-e-4) HONEST GUARD: emit the crossing only if every node verifies on both surfaces
  `≤ onSurfTol` AND the far side makes real progress; otherwise DISCARD + STOP + defer → OCCT.
  A genuine cusp endpoint ENDS. Never fabricate a pole/apex-crossing point; never weaken a tol.
- Emit the FULL sphere-pole great circle (closed loop through both poles) and the FULL
  cone-apex line (both nappes), verified vs OCCT (on-surface, on-locus, arc/closure, pole/apex
  point match).

**Non-Goals (deferred — never faked here)**
- **General / freeform parametric singularities** — NURBS degenerate edges, collapsed control
  rows, seam singularities on freeform surfaces. Only the elementary sphere pole + cone apex are
  in scope.
- **Higher-order cusps / edge singularities** — a curve cusp the point-based step cannot
  continue through (genuine curve endpoint) ENDS; a ridge / edge singularity of a surface DEFERS.
- **S4-f: self-intersection completeness / global topology repair.**
- **Any change to `src/native/tessellate`, the `cc_*` ABI, the S3 transversal trace, the S4-c
  crossable-graze crossing, or the S4-d branch machinery.** The S3 corrector / deflection
  controller, the S4-c crossing driver, and the S4-d branch handler are bit-identical; the chart
  machinery engages ONLY at a detected single-surface chart collapse.
- **Weakening `onSurfTol`, `tangentSinTol`, `minStep`, `maxDeflection`, or any tolerance to
  "pass".** A singularity that still truncates is an honestly reported gap.

## Module shape

```
src/native/ssi/marching.h           [extend — additive result fields]
  struct WLine {
    // ... existing fields unchanged ...
    int chartSingularCrossed = 0;    // S4-e: sphere poles / cone apexes this branch STEPPED ACROSS
                                     // (verified on both surfaces). 0 for a pure S3 / S4-c / S4-d trace.
  }
  struct TraceSet {
    // ... existing fields unchanged ...
    int singularitiesCrossed = 0;    // S4-e: chart poles/apexes crossed + verified across all branches
    // nearTangentGaps keeps counting ONLY singularities that could NOT be crossed (deferred)
  }
  struct MarchOptions {
    // ... existing fields unchanged ...
    bool   enableChartSingularities = false; // S4-e master switch (off → S3/S4-c behaviour, byte-identical)
    double chartCollapseFrac = -1.0; // ‖dU‖ < chartCollapseFrac·‖dV‖ ⇒ chart collapse (≤0 → 1e-3)
    double chartStepFrac = -1.0;     // fine step off the singular point when crossing (≤0 → h0/16)
  }

src/native/ssi/chart_singularity.h  [NEW — OCCT-free, the S4-e detection + mapping math]
  namespace chartsing {
    struct ChartCond { double dU, dV; bool collapsed; };  // single-surface chart conditioning
    // chartConditionAt(S, u, v, scale) → ChartCond
    //   central finite-difference ‖dU‖, ‖dV‖ (reusing advanceParams' scheme); collapsed iff
    //   ‖dU‖ < chartCollapseFrac·‖dV‖ AND ‖dU‖ < chartCollapseFrac·scale AND the normal is finite.
    // poleContinuationU(uIn, uPeriod) → uOut
    //   the arbitrary-longitude pin: a great arc through a sphere pole continues on the uIn+π
    //   meridian (mod the U period). The pole v is clamped by the caller.
    // isApexApproach / isPoleApproach helpers keyed on which surface's dU collapsed.
    // (No corrector here — the crossing/reproject lives in marching.cpp, which owns the corrector
    //  and the WLine assembly, exactly as branch_point.h leaves routing to marching.cpp.)
  }

src/native/ssi/marching.cpp         [extend — the S4-e crossing driver, CYBERCAD_HAS_NUMSCI]
  // chartCondition(A,B, State, scale) → which surface (if any) collapsed + the ChartCond
  //   the per-step chart witness, evaluated alongside the transversality tangent in marchDir.
  // crossChartSingularity(A,B, stall, tStarFwd, whichSurf, t, scale, out) → CrossOut
  //   modelled on crossNearTangent: enter FINE along t★, point-based fixed-plane correct
  //   (reuse the S4-c along-t★ cut — no degenerate dU in the solve) across the singular band,
  //   pin the far-side chart coords (pole: v clamped + u = poleContinuationU; apex: re-seed
  //   (u,v) from the continued 3D tangent), verify every node on both surfaces ≤ onSurfTol,
  //   resume when ‖dU‖ recovers on both surfaces. DISCARD + defer on any failure.
  // tryChartBand(...) — the marchDir hook (parallels tryBandEntry): at a detected chart
  //   collapse (enableChartSingularities on), call crossChartSingularity; on success resume the
  //   walk on the far side (++chartSingularCrossed / singularitiesCrossed); on failure STOP +
  //   defer EXACTLY as the current boundary/near-tangent stop.
  // marchDir: before the boundary/near-tangent branches, check chartCondition; a chart collapse
  //   routes to tryChartBand instead of the spurious BoundaryExit / step-crawl.
```

`src/native/**` stays OCCT-free. The new machinery lives in the new header `chart_singularity.h`
(detection + mapping math, header-only) and in `marching.cpp` under `CYBERCAD_HAS_NUMSCI` (the
crossing calls the existing `correct` / `branchpt::reproject`). It reuses the S4-c fixed-plane
point-based corrector unchanged. No new substrate routine; no new hand-tuned constant beyond the
chart discriminators (`chartCollapseFrac`, `chartStepFrac` — documented, sentinel-resolved, never
weakening a tolerance).

## S4-e-1 — Single-surface chart-singularity detection

Along the march, at each accepted node, evaluate the SINGLE-surface chart conditioning on BOTH
surfaces (the same central finite-difference `dU`, `dV` `advanceParams` already computes):

```
‖dU‖_A, ‖dV‖_A  at (u1,v1);   ‖dU‖_B, ‖dV‖_B  at (u2,v2)
chart collapse on S  ⇔  ‖dU‖_S < chartCollapseFrac · ‖dV‖_S
                        AND ‖dU‖_S < chartCollapseFrac · scale
                        AND ‖normal_S‖ is finite (a pole/apex, not a NaN)
```

This is DISTINCT from the S4-c witness (the PAIR sine `‖n₁ × n₂‖`) and the S4-d witness (sine
collapse + raw-tangent flip). A pole crossing can have a HEALTHY transversality sine (the great
circle ∩ plane is transversal at the pole), so the S4-c/S4-d seams never fire there — the chart
witness is the only thing that sees it. A CURVE CUSP is the complementary case: the intersection
tangent's predicted chord shrinks toward zero (the corrector's advance `dot(A.point−prev, t)`
cannot reach `h`) while BOTH charts are regular — flagged separately, and only crossed if the
point-based step continues through it and verifies (else it is a genuine endpoint → END).

The witness feeds the step controller: at a detected collapse the marcher SHORT-CIRCUITS the
ordinary `advanceParams` step-shrink loop (the apex crawl) and routes into the point-based
crossing. This also GUARDS the boundary test: a `v`-edge is a genuine `BoundaryExit` ONLY when
there is NO chart collapse there (a finite cap); a pole/apex `v`-edge collapses `‖dU‖` and is a
CROSSING candidate, not a boundary.

## S4-e-2 — Point-based / chart-clamped corrector across the singular band

The surface point + normal stay finite, so the crossing reuses the S4-c FIXED-PLANE cut (the
`branch_point.h`-style point-based reproject): the least-squares corrector drives

```
r₀..₂ = A.point(u1,v1) − B.point(u2,v2)                (land on the intersection — needs only point)
r₃    = dot(A.point − Panchor, t★) − d                 (advance d along the last-good tangent t★)
```

NEITHER residual needs a full-rank single-surface `dU`, so the solve stays well-posed exactly
where `advanceParams` failed. The FAR-SIDE `(u,v)` are mapped back LOOSELY:

- **SPHERE POLE.** The longitude `u` is ARBITRARY at the pole (the whole `u` circle collapses to
  one point). A great arc through the pole continues on the OPPOSITE meridian: `u_out =
  poleContinuationU(u_in, uPeriod) = u_in + π (mod 2π)`. The pole `v` is CLAMPED to `±π/2`. So
  after the point-based cut lands the far-side 3D point, its sphere `(u,v)` are RE-SEEDED as
  `(u_in + π, v)` (v now decreasing away from the pole) and the corrector confirms them.
- **CONE APEX.** The apex is a SINGLE 3D point (the origin of the cone). The curve may pass
  THROUGH it; the far-side nappe has `v` of the opposite sign. After the point-based cut steps
  just past the apex, the far-side `(u,v)` are re-seeded from the CONTINUED 3D tangent
  (`u` from the projected direction, `v` sign flipped), and the corrector confirms them.

The pole/apex point itself (where `‖dU‖ = 0` exactly and, for the apex, the normal degenerates)
is NOT emitted as a march node from the degenerate side; the crossing steps FROM a last-good
pre-singular node TO a verified far-side node, sampling the singular point only through the
point-based fixed-plane cut (which is well-posed there). The emitted nodes are all on both
surfaces `≤ onSurfTol`.

## S4-e-3 — Step control across the band

Mirror the S4-c band discipline: enter with a FINE step (`chartStepFrac·h0`, default `h0/16`) so
the pole/apex is RESOLVED, not leapt; the fine steps sample the collapse so the guard can verify
each. Do NOT let `advanceParams`' shrink loop crawl against the ill-conditioning (the 20042-point
apex pathology): the chart witness short-circuits into the crossing BEFORE the shrink loop
engages. Once the far-side node verifies AND `‖dU‖` has RECOVERED above the collapse threshold on
BOTH surfaces, hand back to the normal S3 `marchDir` walk (the ordinary transversal step resumes,
byte-identical to S3). The whole crossing is a bounded fine-step walk (a `chartMaxSteps` budget
like `crossMaxSteps`) so a pathological case cannot spin — it defers instead.

## S4-e-4 — Honest guard

A chart-singularity crossing is EMITTED only if:

1. every node across the singular band verifies on BOTH surfaces `≤ onSurfTol` (no fabricated
   point), AND
2. the far-side node makes REAL progress off the singular point (`‖dU‖` recovers and the march
   advances more than a fine step), AND
3. for a POLE, the continuity-pinned `u_out` node verifies (a wrong meridian pick does not land
   on both surfaces ≤ `onSurfTol`).

If ANY fails, the crossing arc is DISCARDED (rolled back like S4-c) and the march STILL STOPS +
defers — a truncated `NearTangent` / `BoundaryExit` WLine counted in `nearTangentGaps` with the
typed stop reason (→ OCCT), reporting the MEASURED gap (e.g. the sphere-pole arc still ≈ π). A
genuine curve CUSP endpoint (curve velocity → 0 that the point-based step cannot continue) ENDS
as a normal curve termination. No pole/apex-crossing point is ever fabricated; no tolerance is
weakened.

## Crossed-vs-deferred scope (honest)

| Configuration at the node | S4-e action | counted as |
|---|---|---|
| Sphere pole / cone apex (`‖dU‖` collapse, finite normal), point-based cut verifies both surfaces + far-side progress | **CROSS the singular band, resume S3** | `singularitiesCrossed`, `chartSingularCrossed` |
| Chart collapse but the far-side node will NOT re-project on both surfaces ≤ `onSurfTol` | STOP + defer → OCCT | `nearTangentGaps` |
| Genuine DOMAIN boundary `v`-edge (`‖dU‖` does NOT collapse — a finite cap) | EXIT `BoundaryExit` (unchanged) | (open curve end) |
| Curve cusp (curve velocity → 0, both charts regular) the point-based step cannot continue | END (genuine curve endpoint) | (curve termination) |
| Chart collapse the point-based cut cannot resolve (unverifiable across the band) | STOP + defer → OCCT | `nearTangentGaps` |
| PAIR near-tangent graze (`‖n₁×n₂‖ → 0`, single branch) — S4-c | S4-c MARCH THROUGH (unchanged) | `nearTangentCrossed` |
| Locus self-crossing (branch point) — S4-d | S4-d localize + route (unchanged) | `branchPoints` |
| Transversal region, healthy chart | normal S3 march (unchanged) | traced |

## Verification model (two gates)

- **Host (no OCCT).** Extend `tests/native/test_native_ssi_marching.cpp` (or a new
  `tests/native/test_native_ssi_s4e_singularities.cpp`), all under `CYBERCAD_HAS_NUMSCI`:
  - **Sphere pole now fully traced.** Unit sphere ∩ plane `y = 0` (the fixture S3 today
    truncates to arcLen ≈ π, one meridian) is now the FULL closed great circle: arcLen ≈ `2π`
    within the deflection tolerance, both `u = 0` and `u = π` meridians visited (the loop
    crosses BOTH poles), `singularitiesCrossed ≥ 2`, `status = Closed`, every node on BOTH
    surfaces ≤ `onSurfTol`.
  - **Cone apex now crossed.** Double cone (`R₀ = 0`, α = 45°) ∩ plane `y = 0` (the fixture S3
    today stalls at `v ≈ 0` burning 20042 pts) is now traced ACROSS the apex — the line spans
    both nappes (`v` from ≈ +2 through 0 to ≈ −2) in a BOUNDED node count,
    `singularitiesCrossed ≥ 1`, every node ≤ `onSurfTol`.
  - **Genuine boundary still exits.** A finite cylinder ∩ plane whose curve runs to a real
    `v`-cap edge (no `‖dU‖` collapse) STILL exits `BoundaryExit` with `singularitiesCrossed = 0`
    — the chart machinery does NOT fire at a true boundary.
  - **Regression.** The 5 transversal S3 fixtures trace bit-identically (`nt = 0`); the S4-c
    graze STILL crosses (`nearTangentCrossed ≥ 1`, `singularitiesCrossed = 0`); the S4-d
    Steinmetz STILL traces (`branchPoints = 2`, `singularitiesCrossed = 0`).
  - Full CTest green NUMSCI ON and OFF (S4-e assertions absent with NUMSCI off, like
    S2/S3/S4-c/S4-d). No OCCT linked; no tolerance weakened.
- **Sim native-vs-OCCT (booted simulator).** Extend `scripts/run-sim-native-ssi-marching.sh` +
  `tests/sim/native_ssi_marching_parity.mm` (or a new `scripts/run-sim-native-ssi-s4e.sh`): add
  the sphere-pole great circle and the cone-apex line and assert they are now FULLY traced
  natively — crossing the pole/apex — matching OCCT `IntPatch` / `GeomAPI_IntSS`: every sampled
  native node on the OCCT locus ≤ `onCurveTol` (`GeomAPI_ProjectPointOnCurve`) AND on both
  surfaces ≤ `onSurfTol`; the native arc length / loop closure reconciles with the OCCT curve
  within tol; the native curve passes through the OCCT pole/apex point to `tol`. Also assert the
  genuine-boundary control STILL exits and the transversal + S4-c + S4-d cases are UNCHANGED.
  Report per-pair crossed vs still-deferred. Run via `xcrun simctl spawn <booted udid>`;
  `xcrun simctl list devices booted`.

## Decisions

- **A NEW, INDEPENDENT single-surface `‖dU‖`-collapse witness — NOT the S4-c/S4-d seam.** A pole
  crossing can be perfectly transversal (`‖n₁ × n₂‖` healthy), so the S4-c sine collapse and the
  S4-d tangent flip DO NOT fire there. The chart singularity is a property of ONE surface's
  Jacobian, so it needs its own witness (`‖dU‖` vs `‖dV‖·scale`). This is the crux distinction
  the roadmap draws between S4-c/d (pair/locus) and S4-e (chart), enforced by construction: the
  two witnesses are computed from different quantities and gate different code paths.
- **Reuse the S4-c fixed-plane point-based corrector for the crossing.** The corrector's 3D
  residual + along-`t★` cut need only the finite point + normal, so it is well-posed exactly
  where `advanceParams`' `dU`-based solve fails. No new corrector; the crossing driver
  `crossChartSingularity` parallels `crossNearTangent` structurally.
- **Pin the arbitrary pole longitude from arc continuity; treat the apex as one 3D point.** At a
  pole the whole `u` circle maps to one point, so `u` is a free coordinate — pinning it from the
  incoming arc's great-circle continuation (`u_in + π`) and VERIFYING the far node on both
  surfaces is the honest map back; a wrong pick simply fails verification and defers. The apex is
  a single 3D point the curve passes through, re-seeded from the continued 3D tangent.
- **Guard the boundary test with the chart witness.** A pole/apex `v`-edge and a genuine cap
  `v`-edge are both non-periodic `v` edges; the `‖dU‖` collapse distinguishes them, so a real
  boundary still exits `BoundaryExit` and only a chart singularity is crossed. The
  finite-cylinder-cap control is the "must-still-exit" pin.
- **Emit ONLY verified crossings; otherwise DISCARD + DEFER.** A crossing whose nodes will not
  verify on both surfaces, or whose far side makes no progress, is rolled back and the march
  defers — never a fabricated pole/apex point. This closes the cardinal S4-e risk (a faked
  crossing) by construction, exactly as S4-c/S4-d close theirs.
- **`enableChartSingularities` master switch (default OFF).** The whole S4-e path is behind a
  default-off switch (off → the current S3/S4-c behaviour, byte-identical) plus a
  `chartMaxSteps` crossing budget, so a caller opts in and no existing trace changes. Both
  additive, neither weakens a tolerance.
- **Additive, ABI-stable, tessellator-untouched.** New `TraceSet.singularitiesCrossed` /
  `WLine.chartSingularCrossed` + `MarchOptions` chart knobs (sentinel-resolved) + the new
  `chart_singularity.h`; no `cc_*` change; `src/native/tessellate` untouched; the S4-e code under
  `CYBERCAD_HAS_NUMSCI` like S2/S3/S4-c/S4-d.

## Risks / Trade-offs

- **False chart trigger at a genuine boundary (crossing past the surface).** Mitigation: require
  the `‖dU‖` collapse WITH a finite normal AND a re-projectable far-side node on both surfaces ≤
  `onSurfTol`; a true boundary has no far side, so the crossing fails verification and the march
  exits `BoundaryExit` as today. The finite-cap control pins it. Accepted.
- **Wrong pole-longitude continuation (far arc down the wrong meridian).** Mitigation: pin
  `u_out = u_in + π` from great-circle continuity and VERIFY the first far-side node on both
  surfaces ≤ `onSurfTol` before accepting; a mis-pinned node does not verify → defer. Accepted.
- **Drift along the arbitrary-`u` null direction at the pole.** Mitigation: the fixed-plane
  along-`t★` residual (the well-posed-as-sine→0 cut) plus clamping the pole `v` coordinate pin
  the crossing; the `u` circle's freedom is removed by the continuity pin. Accepted.
- **Transversal / S4-c / S4-d regression.** Any change near the marcher risks perturbing the
  passing S3 traces, the S4-c crossing, or the S4-d branch trace. Mitigation: the chart machinery
  engages ONLY at a detected single-surface chart collapse (`enableChartSingularities` + the NEW
  `‖dU‖` witness, disjoint from the S4-c/S4-d seams); the S3 corrector / deflection controller,
  the S4-c crossing driver, and the S4-d branch handler are bit-identical; the 5 transversal
  pairs + the S4-c graze + the S4-d Steinmetz are pinned green. Accepted.
- **Cusp misread as a crossable singularity.** Mitigation: a curve cusp is crossed ONLY if the
  point-based step continues through it AND every node verifies + the far side progresses; a
  genuine endpoint cusp fails the progress check and ENDS as a normal curve termination — never a
  fabricated continuation. Accepted. Whatever does not resolve robustly still stops + defers →
  OCCT and is reported with the measured gap; no point is faked, hand-tuned, or weakened to pass.

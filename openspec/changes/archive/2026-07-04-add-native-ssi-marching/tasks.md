# Tasks — add-native-ssi-marching (SSI Stage S3)

Verification levels: **host** = OCCT-free host CTest (known-shape native pairs:
every sampled WLine / B-spline point on both surfaces ≤ tol; loop branches close;
open branches exit at a boundary; near-tangent fixtures trace up to the tangent and
report `nearTangentGaps`, never a point past it; duplicate seeds dedup to one WLine);
**sim** = native-vs-OCCT **curve parity** vs `GeomAPI_IntSS` / `IntPatch` on the
simulator (branch count + per-branch length + sampled point-to-curve distance +
on-both-surfaces residual + near-tangent gap count). SSI is INTERNAL — no `cc_*`
entry point is added or exercised; the tracer is asserted at the
`cybercad::native::ssi` C++ boundary, exactly like S1 / S2 / native-math. The
corrector (re-projection) and B-spline fit — and thus a useful tracer — are under
**`CYBERCAD_HAS_NUMSCI`**.

> Implementation note: the result types live in `src/native/ssi/marching.h`
> (alongside the tracer) rather than a separate `wline.h` — same OCCT-free,
> substrate-free data-only header, one fewer file. Names match the design.md
> contract: `TraceStatus { Closed, BoundaryExit, NearTangent, Failed }`,
> `WLine { points; curve; onSurfResidual; status; branchId }`,
> `TraceSet { lines; tracedBranches; nearTangentGaps; dedupedRetraces; … }`. The
> WLine node is `WLinePoint` (3D point **plus** its (u1,v1,u2,v2) on both surfaces)
> rather than a bare `Point3` — the params are needed by the corrector and by S5.

## 1. WLine result types + surface adapter reuse
- [x] 1.1 Result types in `marching.h` (see note): `WLine`, `TraceStatus`,
  `FittedBSpline`, `TraceSet` + `WLinePoint`. OCCT-free, substrate-free (data only,
  always visible even with NUMSCI off). (**host** ✓ `test_native_ssi_marching`)
- [x] 1.2 Reuses the S2 `SurfaceAdapter` (point / normal / `[u0,u1]×[v0,v1]` domain +
  param periods) so elementary / torus / Bézier / B-spline / NURBS all march through
  one path; consumes the S2 `SeedSet` as the input contract (`trace_intersection`
  seeds via `seed_intersection`; `trace_from_seeds` takes a `SeedSet` directly).
  (**host** ✓ sphere/plane/cylinder + Bézier + B-spline fixtures)

## 2. Corrector — substrate re-projection onto both surfaces  [CYBERCAD_HAS_NUMSCI]
- [x] 2.1 `correct()` drives `r = A.point − B.point` **plus** an along-tangent advance
  residual (m=n=4, well-posed — pins the along-curve DOF) with `least_squares`, seeded
  at the tangent-plane-predicted params, **clamped** to each range (angular directions
  left unclamped to wrap the seam); accepts iff `‖A−B‖ ≤ onSurfTol`. (**host** ✓ every
  node on both surfaces < 1e-6)
- [x] 2.2 The corrector **fails** (stops the march) when it does not converge on both
  surfaces, or `‖nA × nB‖ < tangentSinTol` (near-tangent) — never a fabricated /
  uncorrected point. (**host** ✓ tangent-spheres emits no curve)

## 3. Predictor — tangent step
- [x] 3.1 `intersectionTangent()` computes `t = normalize(nA × nB)` from the two
  surface normals (sign-continued by the walk direction) and predicts `P + h·t`;
  `advanceParams()` moves each surface's (u,v) through its tangent-plane 2×2.
  (**host** ✓)

## 4. Adaptive step control  [CYBERCAD_HAS_NUMSCI]
- [x] 4.1 `tryStep()` bounds `h` in `[minStep, maxStep]`: **shrinks** (halves + retries)
  on corrector failure, chord/arc deflection over `maxDeflection`, or a slid-back step;
  **grows** (×1.5) on a smooth, cheap step. Documented as the fidelity/cost knob.
  (**host** ✓)
- [x] 4.2 A step that must shrink below `minStep` to converge is the **near-tangent**
  signal → the direction stops with `NearTangent`, never forcing the point. (**host** ✓)

## 5. March loop + termination
- [x] 5.1 `marchDir()` walks predict → correct → adapt, appending nodes; `march_branch()`
  marches **both directions** and stitches (reversed backward + seed + forward), bounded
  by `maxPoints`. (**host** ✓)
- [x] 5.2 **Loop closure**: within `loopCloseFrac·h` of the seed after a min step count
  → `TraceStatus::Closed`. (**host** ✓ crossing spheres / plane∩sphere / skew cyl loops)
- [x] 5.3 **Boundary exit**: a corrected param reaches a non-periodic domain edge (within
  a `1e-4·domain` band, above the corrector's along-step short-fall) → stop that
  direction; branch is `TraceStatus::BoundaryExit`. (**host** ✓ ramp∩plane open segment)
- [x] 5.4 Continuity guard: a step that barely advances (`< 0.25·h` from the previous
  point — an off-branch/slid-back correction) is rejected → shrink `h` and retry;
  persistent failure stops the march. (**host** ✓ folded into `tryStep`)

## 6. Dedup retraced branches
- [x] 6.1 `retraces()` — after marching, a WLine whose sampled nodes mostly sit within a
  `dedupFrac·scale` radius of an already-kept line is dropped (`dedupedRetraces++`) → one
  WLine per distinct branch. (**host** ✓ duplicate seed → 1 WLine)

## 7. B-spline fit + self-verify  [CYBERCAD_HAS_NUMSCI]
- [x] 7.1 `fitBSpline()` — chord-length-parametrized least-squares fit (clamped uniform
  knots, native-math `basisFuns` + numerics `lstsq`) → the WLine's `FittedBSpline`
  (evaluate with `math::curvePoint`). (**host** ✓)
- [~] 7.2 **Self-verify**: `maxFitError` (max polyline-to-curve deviation) is reported and
  host-asserted < 1e-3 on the closed-circle fixture. Automatic densify-and-refit on a
  too-loose fit is NOT yet wired (the polyline is always the on-surface ground truth and
  is retained; the fit is a convenience curve). Remaining follow-up.

## 8. Transversal-only scope + honest deferral (S4 gaps, never faked)
- [x] 8.1 Transversal branches get a full WLine (Closed / BoundaryExit); a near-tangent
  march is traced **up to** the tangent, marked `NearTangent`, counted in
  `nearTangentGaps` — never a point past it. (**host** ✓ tangent spheres deferred by S2,
  no curve fabricated by S3)
- [x] 8.2 Coincident / branch-point / self-intersection **deferred to S4** — documented
  in `marching.h` header + `native_ssi.h` namespace doc; `TraceSet` documented as the S5
  contract and `nearTangentGaps > 0` as the honest S4 signal. (**host** ✓ docs)

## 9. Verification (two gates)
- [x] 9.1 Host known-shape suite `test_native_ssi_marching` (**7 cases, 0 failed**):
  crossing spheres → closed circle; plane∩sphere → closed circle; skew cylinders → two
  closed loops (+ seam wrap); sphere∩Bézier bump → loop on both freeform+sphere; ramp
  B-spline∩plane → open segment exiting the boundary; tangent spheres → no curve
  (deferred); duplicate seed → 1 WLine. Every node on both surfaces < 1e-6; fit error
  < 1e-3. No OCCT. Full CTest **26/26** (NUMSCI on), **23/23** (NUMSCI off, S3 tests
  correctly absent).
- [x] 9.2 Sim native-vs-OCCT curve parity (`GeomAPI_IntSS` / `IntPatch`: branch count +
  per-branch length + sampled point-to-curve distance + on-both residual + gap count),
  via `native_ssi_marching_parity.mm`. **PASS** — 5 transversal pairs, 9 branches, all
  TRANSVERSAL/fully-traced, 0 near-tangent-truncated (deferred to S4). Branch counts match
  OCCT on every pair (1/1, 4/4, 2/2, 1/1, 1/1); 5/5 OCCT closed loops reproduced as Closed
  native WLines (bspline×plane 0-closed/4-open correctly matched). Worst deltas: max
  on-OCCT-curve 1.60e-06, max on-surface 6.81e-07 (both skew-cyl-unequal), max length
  delta 2.28e-03 abs / ~0.33% rel (bspline×plane, within deflection/step tol). `run-sim-suite.sh`
  **221 passed, 0 failed**.
- [x] 9.3 `openspec validate add-native-ssi-marching --strict` ✓ green. S3 marked done-at-bar
  with measured deltas in `SSI-ROADMAP.md` / `ROADMAP.md` / `NATIVE-REWRITE.md` /
  `docs/STATUS-phase-4.md` / `docs/ROADMAP.md` / `README.md`; S4 robustness (moat) + S5 curved
  booleans (payoff) remain.

## Deferred to S4 / S5 / OCCT (NOT in S3 marching scope — honest)

- [ ] **Near-tangent** marching (`n₁ × n₂ → 0`: higher-order predictor / step control
  to cross a tangent) → **S4**; S3 traces up to the tangent and reports
  `nearTangentGaps`, never faked.
- [ ] **Branch-point splitting** (a singular crossing of two intersection branches) →
  **S4**; S3 traces up to it and flags, never attempts the split.
- [ ] **Self-intersection resolution** (a curve crossing itself) → **S4**.
- [ ] **Coincident / overlapping-surface** curve extraction (no discrete branch to
  march) → **S4** + OCCT fallback.
- [ ] **S5 curved booleans** — using the traced WLines to split curved faces,
  classify fragments, and assemble the watertight shell → **S5** (this change only
  produces the WLines that S5 consumes).

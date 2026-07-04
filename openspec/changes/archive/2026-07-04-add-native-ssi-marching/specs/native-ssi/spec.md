# native-ssi

Native, OCCT-free surface-surface intersection. This change extends the S1
closed-form / S2 seeding module (`openspec/SSI-ROADMAP.md`) with **SSI Stage S3 —
the marching-line tracer (WLine)**: from each S2 seed, **trace the full intersection
curve** of a **transversal** branch (predictor `t = normalize(n₁ × n₂)` +
`native-numerics` re-projection corrector + adaptive step + loop / boundary
termination + B-spline fit), producing one WLine per seed whose sampled points lie
on both surfaces within tol AND coincide with OCCT `IntPatch` / `GeomAPI_IntSS`
(branch count / length / sampled distance) within tolerance. Near-tangent /
coincident / degenerate marching, branch-point splitting and self-intersection are
deferred to S4 (traced up to the tangent, flagged, never faked). The traced WLines
are the input contract for S5 curved booleans. Internal capability: **no `cc_*`
ABI**.

## MODIFIED Requirements

### Requirement: SSI is an internal capability verified against OCCT with no ABI change

SSI SHALL be an **internal** native capability consumed by native booleans/blends
(the S5 on-ramp) and by the S3 marching tracer (which consumes S2 seeds), and SHALL
NOT be exposed through the `cc_*` C ABI: no `cc_*` entry point, signature, or POD
struct SHALL be added or changed by this capability. It SHALL be verified at the
SSI-function level (per `SSI-ROADMAP.md` §Verification model) by two gates: a **host**
gate (no OCCT) and a **sim native-vs-OCCT** gate (`GeomAPI_IntSS` / `IntPatch` parity
/ branch-recall) — the same internal parity discipline used for `native-math` and
`native-topology`. For the S1 closed-form pairs the host gate asserts known conics
with every sampled point on both surfaces; for the S2 seeding pairs the host gate
asserts known transversal branch counts with every seed on both surfaces, and the
sim gate reports the measured **branch recall** vs OCCT; for the S3 marching pairs
the host gate asserts known curve shapes (loop closure / boundary exit) with every
sampled WLine point on both surfaces, and the sim gate reports **curve parity**
(branch count / length / sampled point-to-curve distance) vs OCCT.

#### Scenario: native SSI matches OCCT GeomAPI_IntSS on a supported pair
- GIVEN a supported elementary pair built both as native `math` surfaces and as
  OCCT `Geom_*Surface`
- WHEN the native SSI module and OCCT `GeomAPI_IntSS` each intersect the pair on
  the simulator
- THEN the native curve(s) SHALL match OCCT's in kind, branch count, and sampled
  point-to-curve distance within tolerance, compared at the SSI C++ boundary
- AND no `cc_*` entry point SHALL have been called or added

#### Scenario: host analytic gate runs without OCCT
- GIVEN the native SSI module built for the OCCT-free host
- WHEN the analytic test suite runs the supported pairs and the out-of-scope pairs
- THEN each supported pair SHALL match its closed-form curve with all samples on
  both surfaces, and each out-of-scope pair SHALL return `analytic == false` with
  the correct reason — with no OCCT linked

#### Scenario: S2 seed set is verified against OCCT branch recall on the simulator
- GIVEN a transversal freeform or non-closed-form quadric pair built both as native
  `math` surfaces and as OCCT `Geom_*Surface`
- WHEN the native S2 seeder produces its seed set and OCCT `GeomAPI_IntSS` computes
  the same pair on the simulator
- THEN the harness SHALL report the native **branch recall** (native transversal
  branches carrying ≥1 seed ÷ OCCT transversal branches) and each native seed's
  on-both-surfaces residual, compared at the SSI C++ boundary
- AND no `cc_*` entry point SHALL have been called or added
- AND recall SHALL be **reported** (with the deferred-tangent branch count called
  out), not asserted to be 1.0 blindly

#### Scenario: S3 traced curves are verified against OCCT IntPatch curve parity on the simulator
- GIVEN a transversal freeform or non-closed-form quadric pair built both as native
  `math` surfaces and as OCCT `Geom_*Surface`, with its S2 seeds
- WHEN the native S3 tracer walks one WLine per seed and OCCT `GeomAPI_IntSS` /
  `IntPatch` computes the same pair on the simulator
- THEN the harness SHALL report **curve parity** — native branch count vs OCCT, and
  for each matched branch the curve length and the sampled point-to-curve distance
  (native samples projected onto OCCT's curve and vice-versa) within tolerance —
  plus each WLine's on-both-surfaces residual, compared at the SSI C++ boundary
- AND no `cc_*` entry point SHALL have been called or added
- AND the near-tangent gap count SHALL be **reported** (the S4 seam) rather than
  hidden, and whatever S3 cannot trace SHALL fall back to OCCT with the measured gap

## ADDED Requirements

### Requirement: Native predictor-corrector marching traces the full transversal intersection curve

The module SHALL provide a native, **OCCT-free** SSI Stage-S3 **marching-line
tracer** (`cybercad::native::ssi`) that, given the S2 `SeedSet` (≥1 seed per distinct
**transversal** branch, each with its `(u1,v1,u2,v2)` on both surfaces) and the two
native surfaces from `src/native/math/` (elementary + torus + freeform Bézier /
B-spline / NURBS, each via its `point(u,v)` / `dU` / `dV` / `normal(u,v)` evaluator),
**traces the full intersection curve** of each transversal branch and returns **one
WLine per seed** — a corrected polyline fitted to a B-spline — whose sampled points
provably lie on **both** surfaces within a scale-derived tolerance. The tracer SHALL:

- **Predict.** At the current corrected point, compute the intersection tangent
  `t = normalize(n₁ × n₂)` from the two surface unit normals
  (`normalize(dU × dV)` per surface), sign-continued with the current walk direction,
  and step to a predicted next point `Pₚ = P + h·t`.
- **Correct.** Re-project `Pₚ` back onto **both** surfaces with the `native-numerics`
  substrate — driving `A.point(u1,v1) − B.point(u2,v2) = 0` with `least_squares`,
  seeded by `closest_point_on_surface` per operand — landing the point on the
  intersection curve and advancing `(u1,v1,u2,v2)`, **clamped** to each surface's
  range. A corrected point SHALL be accepted only when its on-both-surfaces residual
  is ≤ tol; a predicted point SHALL NEVER be accepted uncorrected.
- **Adapt the step.** The step `h` SHALL be bounded by a scale/tol-derived
  `[h_min, h_max]` and SHALL **shrink** on high curvature / corrector strain
  (iteration count, correction distance, turn angle over a threshold) and **grow** on
  repeated clean steps — the recall/fidelity vs cost knob.
- **Terminate.** The walk SHALL stop on **loop closure** (the corrected point returns
  within a tol-scaled radius of the walk start after a minimum step count → a closed
  loop) or on a **boundary exit** (a corrected param leaves `[u0,u1]×[v0,v1]` on
  either surface → an open branch, marched in both directions from the seed), bounded
  by a max-step count so it always terminates.
- **Dedup.** A seed whose point already lies within a tol radius of an
  already-traced WLine SHALL be skipped (retrace), so the output is **one WLine per
  distinct transversal branch**.
- **Fit.** Fit a B-spline through the ordered corrected polyline via `native-math`,
  exposed as the WLine's `Geom`-quality curve, and **self-verify** it lies on both
  surfaces before it is returned.

Each returned WLine SHALL carry its polyline, its fitted B-spline, its
on-both-surfaces residual, and a termination status. The tracer SHALL be OCCT-free
and consume only the S2 `SeedSet`, `native-math` (surface evaluators + B-spline fit)
and `native-numerics` (`least_squares`, closest-point); the corrector and fit path
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` signature or POD struct
SHALL change. The returned WLines SHALL be the input contract for S5 curved booleans
(one WLine splits a curved face).

#### Scenario: a transversal closed-loop branch is traced and closes
- GIVEN a transversal pair whose intersection is a closed loop (e.g. a sphere
  piercing a freeform bump) and an S2 seed on that loop
- WHEN the S3 tracer marches from the seed (predict `n₁ × n₂` step → correct onto
  both surfaces → adaptive step)
- THEN the walk SHALL return within a tol-scaled radius of its start and terminate as
  a **closed loop**, and the fitted B-spline SHALL close (its endpoints coincide)
- AND every sampled WLine and B-spline point SHALL lie on both surfaces within tol

#### Scenario: a transversal open branch is traced to both boundary exits
- GIVEN a transversal pair whose intersection crosses the patch (e.g. two crossing
  cylinders) and an S2 seed on that branch
- WHEN the S3 tracer marches from the seed in both directions
- THEN each direction SHALL terminate at a **boundary exit** (a param leaving a
  surface's range), the branch SHALL be marked open, and its endpoints SHALL lie on
  the patch boundary
- AND every sampled WLine point SHALL lie on both surfaces within tol

#### Scenario: the corrector re-projects each predicted point onto both surfaces
- GIVEN a predicted point `Pₚ = P + h·t` and its parameter guess `(u1,v1,u2,v2)`
- WHEN the corrector drives `A.point(u1,v1) − B.point(u2,v2)` with `least_squares`
  seeded by closest-point and clamps the params to each range
- THEN the accepted point SHALL have on-both-surfaces residual ≤ tol before it is
  appended to the polyline
- AND a predicted point SHALL NEVER be appended uncorrected

#### Scenario: duplicate seeds on one branch dedup to a single WLine
- GIVEN two S2 seeds that both land on the same transversal branch
- WHEN the S3 tracer processes the second seed and finds its point already within a
  tol radius of the WLine traced from the first
- THEN it SHALL skip the retrace and increment the deduped-retrace diagnostic, so the
  branch yields exactly one WLine

#### Scenario: the fitted B-spline is self-verified on both surfaces before return
- GIVEN a traced corrected polyline for a transversal branch
- WHEN the tracer fits a B-spline through it and samples the fitted curve
- THEN it SHALL confirm every sample lies on both surfaces within tol before
  returning the WLine; if the fit residual exceeds tol it SHALL densify the polyline
  and refit rather than return a leaky fitted curve

### Requirement: Near-tangent, coincident, and degenerate marching is deferred to S4, never faked

The S3 tracer SHALL trace **transversal** branches only. When a march enters a
**near-tangent** region — the surface normals become near-parallel
(`‖n₁ × n₂‖ → 0`, so the predictor tangent degenerates and the corrector
ill-conditions), or the required step must shrink below `h_min` to converge — the
tracer SHALL **STOP** the march at that point, tracing the branch only **up to** the
tangent, and SHALL mark that WLine `NearTangent` and increment a reported
`nearTangentGaps` count (the S4 seam). It SHALL **NOT** force, extrapolate, or
fabricate any curve point past the tangent, and SHALL **NOT** claim a full trace that
stopped short. Likewise **coincident / overlapping** surfaces (no discrete branch to
march), **branch-point splitting** (a singular crossing of two branches), and
**self-intersection** resolution SHALL be **deferred to S4** — reported, not resolved
or faked. `nearTangentGaps > 0` SHALL be a normal, first-class outcome (the S4 seam),
not an error, mirroring S1's `NotAnalytic` and S2's `deferredTangent` stance.

#### Scenario: a branch running into a near-tangent region is traced up to it and flagged
- GIVEN a transversal branch that runs into a near-tangent region (`‖n₁ × n₂‖ → 0`)
- WHEN the S3 tracer marches into that region and the corrector fails / the step must
  shrink below `h_min`
- THEN the tracer SHALL STOP at the last transversal point, trace the branch only up
  to there, mark the WLine `NearTangent`, and increment the reported
  `nearTangentGaps` count
- AND it SHALL NOT emit any curve point past the tangent nor claim a full trace

#### Scenario: coincident / overlapping surfaces are deferred, not marched
- GIVEN two surfaces that are coincident or overlap over a region (no discrete
  transversal branch to walk)
- WHEN the S3 tracer is handed such a region
- THEN it SHALL defer it to S4 (reported), emitting no fabricated WLine

#### Scenario: a branch point / self-intersection is deferred, not split
- GIVEN a march that reaches a branch point (two branches crossing at a singularity)
  or a self-intersection of the curve
- WHEN the S3 tracer detects the degeneracy
- THEN it SHALL trace up to it and defer the split / resolution to S4 (reported),
  never fabricating the continuation

### Requirement: Traced curve fidelity is a measured native-vs-OCCT parity figure, not a blind claim

The S3 tracer's fidelity SHALL be reported as a **measured native-vs-OCCT curve
parity** — native branch count vs OCCT `GeomAPI_IntSS` / `IntPatch`, and for each
matched branch the curve length and the sampled point-to-curve distance (native
samples projected onto OCCT's curve and vice-versa) within tolerance — rather than
asserted to be a perfect trace. A branch stopped short at a near-tangent region SHALL
reduce the reported parity for that branch and SHALL be called out in the
`nearTangentGaps` count, never hidden or padded with fabricated points. Whatever S3
cannot trace robustly SHALL fall back to OCCT and be reported with the measured gap
(the roadmap's self-verify → OCCT-fallback discipline). The step-size and closure
parameters SHALL be tolerance-scaled knobs that trade trace cost for fidelity.

#### Scenario: native-vs-OCCT curve parity is reported per branch
- GIVEN a transversal pair traced natively and by OCCT on the simulator
- WHEN parity is measured
- THEN the harness SHALL report the native vs OCCT branch count and, per matched
  branch, the length delta and the max sampled point-to-curve distance within tol,
  plus each WLine's on-both-surfaces residual
- AND the near-tangent gap count SHALL be reported, not hidden

#### Scenario: a branch stopped at a tangent lowers reported parity, not faked
- GIVEN a branch that S3 traces only up to a near-tangent region
- WHEN parity is measured against OCCT's full curve for that branch
- THEN the shortened trace SHALL lower the reported parity (length / coverage) for
  that branch and be counted in `nearTangentGaps`
- AND the tracer SHALL NOT fabricate points to inflate the parity figure

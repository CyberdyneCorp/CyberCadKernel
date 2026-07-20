# native-ssi Specification

## Purpose
Native, OCCT-free surface-surface intersection (SSI) — the enabler for general
native curved booleans (SSI-ROADMAP payoff S5). This spec is **Stage S1**: exact
closed-form intersection curves (`Line` / `Circle` / `Ellipse` / `Parabola` /
`Hyperbola`) for the elementary-surface pairs that reduce to a degree-≤2 conic,
built over `src/native/math` only (IntAna-style closed form; no GeomAPI, no
numerics). Any pair outside this family (skew cyl∩cyl, general cone∩cone,
non-coaxial quadrics, oblique plane∩torus, torus∩curved, freeform) returns an
honest `NotAnalytic` status with no curve — the contract with S2 subdivision
seeding / S3 marching / S4 tangent-robustness / OCCT fallback. SSI is INTERNAL: no
`cc_*` entry point; verified at the `cybercad::native::ssi` C++ boundary against
the OCCT `GeomAPI_IntSS` oracle.
## Requirements
### Requirement: Native analytic SSI for the closed-form elementary-pair conic family

The kernel SHALL provide a native, **OCCT-free** surface-surface intersection
module (`cybercad::native::ssi`) that, given two native analytic surfaces from
`src/native/math/` (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`), returns
native intersection curve(s) — `Line` / `Circle` / `Ellipse` / `Parabola` /
`Hyperbola` (or `Point` for a tangent contact), built from native-math primitives
— that provably lie on **both** surfaces, for the following **closed-form** pairs
and placements (SSI Stage S1 scope):

- **plane ∩ plane** → a line (or coincident / parallel-with-no-intersection).
- **plane ∩ sphere** → a circle (or a tangent point / none).
- **plane ∩ cylinder** → parallel lines, a circle, or an ellipse, selected by the
  plane orientation relative to the cylinder axis (perpendicular → circle,
  parallel → 0/1/2 lines, oblique → ellipse).
- **plane ∩ cone** → a circle, ellipse, parabola, hyperbola, or a degenerate case
  (apex point / line pair) by the conic-section rule.
- **plane ∩ torus** → the in-plane quartic, solved with the `native-numerics`
  polynomial-root substrate; the closed-form families (axis-perpendicular
  concentric circles, plane-through-axis two circles, Villarceau two circles) are
  returned as native curves.
- **sphere ∩ sphere** → a circle (or a tangent point / none).
- **sphere ∩ cylinder, COAXIAL** → up to two circles (or a tangent circle / none).
- **sphere ∩ cone, COAXIAL** → up to two circles (or a tangent circle / none).
- **cylinder ∩ cylinder, COAXIAL or PARALLEL** → coincident, parallel lines, a
  tangent line, or none.
- **cylinder ∩ cone, COAXIAL** → circle(s) at the solved height(s) (or tangent /
  none).

Each returned curve SHALL be expressed against a native frame with a `value(t)`
evaluator, and SHALL be self-verified (see the self-verification requirement)
before it is returned. Curve(s) SHALL match OCCT `GeomAPI_IntSS` for the same
operands within tolerance (kind, sampled-point distance, branch count). The module
SHALL be OCCT-free; only the plane∩torus handler MAY use the `native-numerics`
substrate, and the solver-free pairs SHALL build without NumPP/SciPP. No `cc_*`
signature or POD struct SHALL change.

#### Scenario: plane ∩ sphere returns the exact intersection circle
- GIVEN a sphere of radius `R` centered at the origin and a plane at signed
  distance `h` from the center with `|h| < R`
- WHEN the native SSI module intersects them
- THEN it SHALL return a single `Circle` of radius `√(R² − h²)` centered at the
  foot of the sphere center on the plane, lying in the plane
- AND every sampled point on that circle SHALL lie on both the sphere and the
  plane within tolerance

#### Scenario: plane ∩ cylinder selects line-pair / circle / ellipse by orientation
- GIVEN a cylinder of radius `R` and, in turn, (a) a plane perpendicular to the
  axis, (b) a plane parallel to the axis at axis-distance `< R`, and (c) a plane
  oblique to the axis at angle `θ`
- WHEN the native SSI module intersects each
- THEN case (a) SHALL return a `Circle` of radius `R`, case (b) SHALL return two
  `Line`s parallel to the axis, and case (c) SHALL return an `Ellipse` of
  semi-minor `R` and semi-major `R / cos θ`
- AND in every case each sampled curve point SHALL lie on both surfaces within tol

#### Scenario: plane ∩ cone yields the classic conic section
- GIVEN a cone of half-angle `α` and a cutting plane
- WHEN the plane is perpendicular to the axis / cuts all generators / is parallel
  to exactly one generator / cuts both nappes
- THEN the native SSI module SHALL return a `Circle` / `Ellipse` / `Parabola` /
  `Hyperbola` respectively (and a degenerate apex point or line pair when the
  plane passes through the apex)
- AND each sampled curve point SHALL lie on both the cone and the plane within tol

#### Scenario: coaxial quadric pairs return their circle(s)
- GIVEN a coaxial sphere∩cylinder, a coaxial sphere∩cone, and a coaxial
  cylinder∩cone
- WHEN the native SSI module intersects each
- THEN it SHALL return the closed-form circle(s) at the solved height(s) (zero,
  one tangent, or two circles, per the geometry)
- AND every sampled point of every returned circle SHALL lie on both surfaces
  within tolerance

#### Scenario: plane ∩ torus is solved as a planar quartic via the numeric substrate
- GIVEN a torus (major `R`, minor `r`) and a plane whose intersection is one of the
  closed-form families (axis-perpendicular, plane-through-axis, or Villarceau)
- WHEN the native SSI module intersects them, using the `native-numerics`
  polynomial-root substrate for the in-plane quartic
- THEN it SHALL return the corresponding native circle(s)
- AND every sampled point SHALL lie on both the torus and the plane within tol

### Requirement: Pair-dispatch classifies and returns NOT-ANALYTIC for out-of-scope pairs

The module SHALL provide a symmetric pair-dispatch that inspects the two operand
surface types and their relative placement and either routes to a closed-form
handler or returns a typed **NOT-ANALYTIC** result (`analytic == false`) with a
reason and an **empty** curve list. NOT-ANALYTIC pairs SHALL be **deferred** to the
S2 subdivision seeder / S3 marching or OCCT and SHALL NEVER be faked with an
approximate or fabricated curve. The dispatch SHALL return NOT-ANALYTIC for at
least: general/skew cylinder∩cylinder (a quartic space curve), general cone∩cone,
non-coaxial cone∩cylinder, torus∩curved (any torus pair other than the supported
plane∩torus), ANY NURBS/Bézier/B-spline/freeform operand, and any **near-tangent /
coincident** configuration where the closed-form branch is numerically unsafe.
`analytic == false` is a normal outcome (the deferral seam into S2), not an error.
The freeform and non-closed-form-quadric NOT-ANALYTIC pairs are exactly the input
set the S2 subdivision seeder targets.

#### Scenario: skew cylinder ∩ cylinder defers, not faked
- GIVEN two cylinders whose axes are neither coaxial nor parallel (a general
  quartic space-curve intersection)
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with reason "out-of-scope pair" and an
  empty curve list — deferring to the S2 seeder / S3 or OCCT
- AND it SHALL NOT return any approximate or fabricated curve

#### Scenario: any freeform operand defers
- GIVEN a pair where at least one operand is a NURBS / Bézier / B-spline surface
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with the freeform-surface reason and an
  empty curve list, deferring to the S2 seeder / S3

#### Scenario: non-coaxial quadric pair defers rather than approximating
- GIVEN a sphere∩cylinder (or sphere∩cone, or cylinder∩cone) whose axes are NOT
  coaxial (nor, for cyl∩cyl, parallel)
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with the non-coaxial-quadric reason,
  deferring the general case to the S2 seeder / a later SSI stage

#### Scenario: near-tangent / coincident configuration defers
- GIVEN a supported pair in a near-tangent or coincident configuration where the
  closed-form branch is numerically unsafe
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with the near-tangent/coincident reason
  rather than emit a fragile curve

### Requirement: Every analytic curve is self-verified to lie on both surfaces

Before returning any `IntersectionCurve`, the module SHALL sample it densely enough
for its kind and confirm that every sample satisfies **both** operand surfaces'
implicit equations within a tolerance derived from the operands' scale. If any
sample fails, the module SHALL downgrade the entire result to `analytic == false`
(reason: self-verify-failed) rather than return an unverified curve. This is the S1
instance of the roadmap's mandatory self-verify → OCCT-fallback discipline: S1
SHALL never emit a wrong or leaky curved result.

#### Scenario: a returned analytic curve passes the on-both-surfaces check
- GIVEN any supported pair that produces a closed-form curve
- WHEN the module has computed the curve
- THEN before returning it the module SHALL have verified that every sampled point
  lies on both surfaces within tolerance
- AND only then SHALL it return the curve with `analytic == true`

#### Scenario: a closed-form branch that fails self-verify is downgraded, not returned
- GIVEN a configuration whose closed-form branch produces a curve that does not
  satisfy both surfaces within tolerance
- WHEN self-verification runs
- THEN the module SHALL return `analytic == false` (reason: self-verify-failed)
  with an empty curve list — never the failing curve

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

### Requirement: Native subdivision seeding finds a seed per transversal intersection branch

The module SHALL provide a native, **OCCT-free** SSI Stage-S2 **subdivision seeder**
(`cybercad::native::ssi`) that, given two native surfaces from `src/native/math/`
(the elementary + torus surfaces AND the freeform Bézier / B-spline / NURBS surfaces,
each via its `point(u,v)` / `dU` / `dV` / `normal(u,v)` evaluator), returns a set of
**seed points** — **at least one seed on every distinct TRANSVERSAL intersection
branch / loop** — targeting exactly the **freeform** pairs and the **non-closed-form
quadric** pairs that S1's dispatch returns NOT-ANALYTIC for (skew cylinder∩cylinder,
general cone∩cone, non-coaxial cone∩cylinder / sphere∩cylinder / sphere∩cone, oblique
plane∩torus, torus∩curved, and any NURBS/freeform pair). The seeder SHALL:

- **Subdivide + prune.** Recursively subdivide each surface's `[u,v]` parameter
  domain into patches; bound each patch by an AABB — from the **control-net convex
  hull** for B-spline / Bézier / NURBS operands, and an analytic-or-sampled bound for
  the elementary + torus operands; **prune** every patch pair whose AABBs do not
  overlap and recurse on the overlapping pairs down to a size / depth threshold,
  yielding candidate regions that bracket the intersection.
- **Refine.** For each surviving candidate region, refine `(u1,v1,u2,v2)` using the
  `native-numerics` `least_squares` substrate to drive `S1(u1,v1) − S2(u2,v2) = 0` to
  a point that lies on **both** surfaces, **clamping** the parameters to each
  surface's range.
- **Dedup.** Cluster seeds that fall on the same branch (spatial proximity) so the
  output is ~**one seed per branch**.

Every returned seed SHALL lie on **both** surfaces within a scale-derived tolerance
and SHALL carry its parameters on **both** surfaces `(u1,v1,u2,v2)`. The seeder SHALL
be OCCT-free and consume only `native-math` (surface evaluators + control nets) and
`native-numerics` (`least_squares`, closest-point); the refine path SHALL be compiled
under `CYBERCAD_HAS_NUMSCI`. No `cc_*` signature or POD struct SHALL change. The
returned seed set SHALL be the input contract for S3 marching (one traced curve per
seed).

#### Scenario: freeform (NURBS) transversal pair gets a seed on every branch
- GIVEN two native NURBS surfaces that cross transversally in a known number of
  branches/loops
- WHEN the S2 subdivision seeder intersects them (recursive patch-AABB-overlap
  subdivision + `least_squares` refine + dedup)
- THEN it SHALL return at least one seed for each transversal branch, each seed
  lying on both surfaces within tolerance and carrying its `(u1,v1,u2,v2)`
- AND after dedup the seed count SHALL be ~one per distinct branch

#### Scenario: non-closed-form quadric pair (skew cyl∩cyl) is seeded, not deferred by dispatch
- GIVEN two cylinders whose axes are skew (S1 dispatch returns NOT-ANALYTIC)
- WHEN the pair is routed into the S2 subdivision seeder
- THEN the seeder SHALL return ≥1 seed on each transversal branch of the quartic
  intersection, each on both surfaces within tolerance with its `(u1,v1,u2,v2)`

#### Scenario: control-net convex hull bounds a freeform patch soundly
- GIVEN a B-spline / Bézier / NURBS surface patch over a param sub-box
- WHEN the seeder computes the patch AABB from the control-net convex hull
- THEN the AABB SHALL be a conservative (sound) bound of the surface over that
  sub-box, so a disjoint-AABB prune SHALL never discard a region that actually
  contains an intersection

#### Scenario: refine drives the residual onto both surfaces and clamps to range
- GIVEN a surviving candidate region bracketing a transversal branch
- WHEN the seeder refines its center with `least_squares` on
  `A.point(u1,v1) − B.point(u2,v2)`
- THEN the converged `(u1,v1,u2,v2)` SHALL be clamped to each surface's parameter
  range and the point SHALL lie on both surfaces within tolerance before it is
  emitted as a seed

### Requirement: Near-tangent, coincident, and degenerate seeding is deferred to S4, never faked

The S2 seeder SHALL target **transversal** intersections only. For a candidate
region whose refine **ill-conditions** — the surface normals are near-parallel
(`‖n₁ × n₂‖ ≈ 0` at the solution: a near-tangent branch), the surfaces are
coincident / overlapping, or the configuration is otherwise degenerate — the seeder
SHALL **NOT** emit a seed for that region. Instead it SHALL count the region as a
**deferred-to-S4 gap** (a reported diagnostic) and SHALL NEVER assign a fabricated or
mis-refined seed to it. `deferredTangent > 0` SHALL be a normal, first-class outcome
(the S4 seam), not an error.

#### Scenario: a near-tangent branch is reported as an S4 gap, not seeded
- GIVEN a pair with a near-tangent branch where the refine Jacobian ill-conditions
  (`‖n₁ × n₂‖ ≈ 0`)
- WHEN the S2 seeder processes the candidate region for that branch
- THEN it SHALL NOT emit a seed for that branch
- AND it SHALL increment the reported deferred-to-S4 count rather than fabricate a
  seed

#### Scenario: coincident / overlapping surfaces are deferred, not seeded
- GIVEN two surfaces that are coincident or overlap over a region (no discrete
  transversal branch)
- WHEN the S2 seeder processes that region
- THEN it SHALL defer it to S4 (reported), emitting no fabricated seed

### Requirement: Branch completeness is a measured recall figure, not a blind claim

The S2 seeder's completeness SHALL be reported as a **measured branch recall** —
(native transversal branches carrying ≥1 seed) ÷ (true / OCCT transversal branches) —
rather than asserted to be 100%. Missing a small loop because the subdivision was too
shallow SHALL be the acknowledged honest failure mode: it SHALL reduce the reported
recall and SHALL NOT be hidden or compensated with a fabricated seed. The subdivision
depth / minimum patch size SHALL be tolerance-scaled parameters that trade recall for
cost, and deeper subdivision SHALL be able to recover smaller loops.

#### Scenario: a small loop missed by shallow subdivision lowers reported recall
- GIVEN a pair with a small intersection loop and a subdivision depth too shallow to
  bracket it
- WHEN the S2 seeder runs and recall is measured against the true branch count
- THEN the missed loop SHALL lower the reported recall figure
- AND the seeder SHALL NOT fabricate a seed for the missed loop to inflate recall

#### Scenario: deeper subdivision recovers a small loop
- GIVEN the same small-loop pair run at a deeper subdivision depth
- WHEN the S2 seeder brackets the smaller region and refines it
- THEN it SHALL emit a seed for the previously-missed loop, raising the reported
  recall

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

### Requirement: The S3 TraceSet is the consumed input contract for S5-a curved booleans

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the **input contract consumed by the
native S5-a curved boolean** (`src/native/boolean`): for a transversal elementary curved
face pair, the boolean SHALL obtain the `TraceSet` and use each transversal `WLine` — its
per-node `(u1,v1,u2,v2)` on both surfaces (the UV split track) and its fitted B-spline
(the shared seam edge) — to split the curved faces. The S5-a boolean SHALL consume a
`TraceSet` ONLY when it is fully transversal — `nearTangentGaps == 0` and every consumed
`WLine.status` is `Closed` or `BoundaryExit`; a `TraceSet` with `nearTangentGaps > 0`, or
any `NearTangent` / `Failed` WLine, SHALL be treated as the honest **S4 fallback
boundary** and SHALL NOT be consumed (the boolean declines → OCCT). The tracer SHALL NOT
change to serve this consumption — the contract is the already-shipped S3 output; no
`cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI module
SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a fully-transversal TraceSet is consumed to split a curved face
- GIVEN a transversal elementary curved face pair whose S3 `TraceSet` has
  `nearTangentGaps == 0` and every WLine `Closed` or `BoundaryExit`
- WHEN the S5-a curved boolean consumes the `TraceSet`
- THEN it SHALL split each curved face along its WLine's `(u,v)` track and use the WLine's
  fitted B-spline as the shared seam edge, with every seam node on both surfaces ≤ tol
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a non-transversal TraceSet is the S4 fallback boundary, not consumed
- GIVEN an intersecting curved face pair whose S3 `TraceSet` reports
  `nearTangentGaps > 0` (or a consumed WLine is `NearTangent` / `Failed`)
- WHEN the S5-a curved boolean inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never splitting a face on a truncated or fabricated seam

### Requirement: The single closed TraceSet seam is the consumed input contract for the S5-c sphere∩sphere common

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the input contract consumed by ALL THREE
native S5-c sphere∩sphere booleans (`src/native/boolean/ssi_boolean.cpp`): `buildLensCommon`
(COMMON), `buildLensFuse` (FUSE), and `buildLensCut` (CUT). For a transversal sphere∩sphere
pair, each boolean SHALL obtain the `TraceSet` and use the SINGLE `Closed` `WLine` — its
per-node `(u1,v1,u2,v2)` on both spheres (the seam-circle track used to fan each spherical
cap) and its shared 3D nodes (the seam vertices the caps weld on through the shared
`VertexPool`). The SAME seam SHALL split each sphere into an INNER cap (apex nearest the
other centre) and an OUTER cap (apex at the far pole); the ops differ ONLY in cap selection
and orientation: COMMON assembles the two INNER caps (the lens), FUSE the two OUTER caps
(the peanut), and CUT the OUTER cap of the minuend plus the INNER cap of the subtrahend
REVERSED (the scooped cavity). Each boolean SHALL consume the `TraceSet` ONLY when it is
fully transversal — `nearTangentGaps == 0` and the consumed `WLine.status` is `Closed`; a
`TraceSet` with `nearTangentGaps > 0`, a `NearTangent` / `Failed` WLine, or a seam that
spans a full sphere (coincident) SHALL be treated as the honest S4 fallback boundary and
SHALL NOT be consumed by any of the three ops (the boolean declines → OCCT). The tracer
SHALL NOT change to serve this consumption — the contract is the already-shipped S3 output;
no `cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI
module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a single closed TraceSet seam is consumed to weld the lens
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` has `nearTangentGaps == 0`
  and exactly one `Closed` WLine
- WHEN the S5-c curved boolean consumes the `TraceSet`
- THEN it SHALL fan each spherical cap from its pole to the WLine's shared 3D seam nodes,
  with every seam node on both spheres ≤ tol, and the two caps SHALL weld along that single
  seam
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: the one closed seam feeds fuse and cut as it already feeds common
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` is one `Closed` WLine with
  `nearTangentGaps == 0` and `(u1,v1,u2,v2)` per node
- WHEN the native boolean path runs `Op::Fuse` or `Op::Cut`
- THEN the SAME single seam SHALL be decimated once (`decimateSeam` + `seamNodeTarget`) and
  shared by the two caps the op selects (two OUTER caps for FUSE; OUTER + reversed INNER for
  CUT), so the caps weld watertight along the ONE seam
- AND no additional trace, re-trace, or seam SHALL be required beyond the single closed
  circle S3 already produces

#### Scenario: a tangent / coincident sphere TraceSet is the S4 boundary, not consumed
- GIVEN a sphere∩sphere pair whose S3 `TraceSet` reports `nearTangentGaps > 0` (tangent
  spheres) or whose single WLine is `NearTangent` / spans a full sphere (coincident)
- WHEN any of the S5-c curved booleans (COMMON, FUSE, CUT) inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never welding on a truncated or fabricated seam

### Requirement: Native typed coincident / overlapping-surface detection (S4-a)

The kernel SHALL provide a native, **OCCT-free** coincident-surface detector in
`cybercad::native::ssi` that returns a TYPED `CoincidentRegion` describing the shared
locus of two native surfaces, classified as exactly one of `FullSurfaceSame` (the two
surfaces are the same locus), `OverlapSubRegion` (they coincide on a delimited parameter
sub-region, carrying the param bounds on EACH surface), `None` (not coincident), or
`Undecided` (coincidence suspected but the region cannot be robustly delimited).

On the **analytic path**, the detector SHALL decide `FullSurfaceSame` in closed form
from the surface frames and sizes for ALL elementary families — same plane (same normal
up to sign AND same signed offset), coaxial-equal cylinder, coaxial-equal cone (same
apex, collinear axis, equal half-angle), same sphere (same centre, equal radius), same
torus (same centre and axis, equal major and minor radius) — generalising the existing
`IntersectionStatus::Coincident` detection into a complete, consistent family while
keeping the shipped same-sphere and coaxial-equal-cylinder `Coincident` results
bit-identical. On the **seeded path** (`CYBERCAD_HAS_NUMSCI`), the detector SHALL detect
that two general/quadric surfaces coincide over a PATCH by verifying that BOTH the
on-both-surfaces point residual and the surface normals agree over a sampled sub-region
(not merely at isolated seeds), delimit the agreement boundary in parameter space, and
return `OverlapSubRegion` with those bounds — SUPPRESSING spurious seeds and marching
inside it. When the overlap boundary cannot be robustly delimited (partial agreement,
fuzzy boundary, ambiguous domain-edge touch) the detector SHALL return `Undecided` (→
OCCT) and SHALL NOT fabricate a region. `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed.

#### Scenario: analytic identical elementary surfaces classify FullSurfaceSame

- GIVEN two native elementary surfaces of the same kind occupying the same locus — same
  plane, coaxial-equal cylinder, coaxial-equal cone, same sphere, or same torus
- WHEN the S4-a detector classifies the pair
- THEN it SHALL return `CoincidentRegion` of kind `FullSurfaceSame`
- AND `intersect_surfaces` SHALL continue to report `IntersectionStatus::Coincident` for
  those pairs, now backed by the typed region, with the previously-shipped same-sphere
  and coaxial-equal-cylinder results unchanged

#### Scenario: a near-miss of an identical pair classifies None, not a false coincidence

- GIVEN a pair that is close to identical but shifted, rotated, or resized beyond the
  linear/angular tolerance (e.g. two spheres with slightly different radii, or two
  planes with a small offset)
- WHEN the S4-a detector classifies the pair
- THEN it SHALL return `CoincidentRegion` of kind `None` — it SHALL NOT report a
  coincidence for a pair that is not the same locus within tolerance

#### Scenario: two coincident freeform patches yield a delimited OverlapSubRegion

- GIVEN two freeform (Bézier / B-spline / NURBS) surface patches that coincide over a
  sub-rectangle of their parameter domains, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-a seeded detector runs over the candidate region
- THEN it SHALL verify that both the on-both residual and the normals agree across the
  sampled sub-region, delimit the agreement boundary, and return `CoincidentRegion` of
  kind `OverlapSubRegion` carrying the parameter bounds on each surface
- AND it SHALL suppress spurious seeds and marching inside the delimited region

#### Scenario: an undelimitable overlap returns Undecided, not a guessed region

- GIVEN two surfaces whose coincident overlap boundary is partial or fuzzy, or touches a
  domain edge ambiguously, so the region cannot be robustly delimited
- WHEN the S4-a seeded detector attempts to delimit it
- THEN it SHALL return `CoincidentRegion` of kind `Undecided` (the engine falls back to
  OCCT, reported) — it SHALL NOT fabricate a region boundary

### Requirement: Native typed tangent-contact classification (S4-b)

The kernel SHALL provide a native, **OCCT-free** tangent-contact classifier in
`cybercad::native::ssi` that, given a surface pair (analytic) or a seeded solution where
‖n₁ × n₂‖ < `SeedOptions.tangentSinTol`, returns a TYPED `TangentContact` classifying
the contact as exactly one of `TransversalOnly` (no tangency — the normal path handles
it), `TangentPoint` (an isolated 0-dimensional contact — the point SHALL be emitted),
`TangentCurve` (the surfaces are tangent along a whole curve — the curve SHALL be
emitted where closed-form, else its existence flagged with the contact locus),
`NearTangentTransversal` (the surfaces graze but CROSS — an S4-c gap handed on to OCCT,
NOT traced through here), or `Undecided` (the local jet is ambiguous — → OCCT).

On the **analytic path**, tangent configurations SHALL be classified in closed form —
sphere∩sphere at `d = R₁+R₂` (external) or `d = |R₁−R₂|` (internal) → `TangentPoint`
carrying the centre-line contact point; coaxial sphere∩cylinder / sphere∩cone tangent
equator and a plane tangent along a cylinder ruling → `TangentCurve` carrying the tangent
circle / line; a plane tangent to a sphere → `TangentPoint`. Analytic tangency SHALL NOT
return `NearTangentTransversal` or `Undecided`. On the **seeded path**
(`CYBERCAD_HAS_NUMSCI`), the classifier SHALL type the contact by the LOCAL DIFFERENTIAL
GEOMETRY — the sign/rank structure of the relative second fundamental form (relative
normal curvature) in the shared tangent plane: sign-definite → `TangentPoint`; rank-1
(one near-zero eigenvalue) → `TangentCurve`; indefinite (grazes and crosses) →
`NearTangentTransversal`; within the model-scale-derived curvature-noise band →
`Undecided`. The classifier SHALL replace the blunt `SeedSet.deferredTangent`
increment with a typed `TangentContact` recorded per dropped near-tangent region, and
SHALL KEEP the `deferredTangent` integer as a compatibility summary count. It SHALL NOT
fabricate a seed for any tangent contact, SHALL NOT trace through a
`NearTangentTransversal` (that is S4-c → OCCT), and SHALL NOT hand-tune or weaken the
curvature band to force a verdict. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry
point SHALL be added or changed.

#### Scenario: analytic tangent spheres classify TangentPoint and emit the point

- GIVEN two spheres at centre distance `d = R₁ + R₂` (externally tangent) or `d =
  |R₁ − R₂|` (internally tangent)
- WHEN the S4-b analytic classifier classifies the pair
- THEN it SHALL return `TangentContact` of type `TangentPoint` carrying the contact
  point, which SHALL lie on BOTH spheres within tolerance

#### Scenario: analytic surfaces tangent along a curve classify TangentCurve and emit the curve

- GIVEN a coaxial sphere∩cylinder tangent at its equator (cylinder radius equal to the
  sphere radius), or a plane tangent along a cylinder ruling
- WHEN the S4-b analytic classifier classifies the pair
- THEN it SHALL return `TangentContact` of type `TangentCurve` carrying the tangent
  curve (the equator circle / the ruling line), which SHALL lie on BOTH surfaces within
  tolerance

#### Scenario: a seeded near-tangent solution is typed by its relative normal curvature

- GIVEN a seeded refine solution with ‖n₁ × n₂‖ < `tangentSinTol`, with
  `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-b seeded classifier evaluates the relative second fundamental form at the
  contact
- THEN it SHALL return `TangentPoint` when the form is sign-definite, `TangentCurve` when
  it is rank-1, `NearTangentTransversal` when it is indefinite, and `Undecided` when it
  is within the curvature-noise band
- AND the typed `TangentContact` SHALL be recorded on the `SeedSet` while
  `deferredTangent` is kept as a compatibility summary count, with NO seed fabricated

#### Scenario: a near-tangent transversal is classified and handed on, never traced through

- GIVEN a seeded near-tangent contact whose relative second fundamental form is
  indefinite (the surfaces graze but cross)
- WHEN the S4-b classifier types it
- THEN it SHALL return `TangentContact` of type `NearTangentTransversal` and hand it on
  as an S4-c gap (→ OCCT) — it SHALL NOT step through the tangency and SHALL NOT
  fabricate any intersection-curve points across it

#### Scenario: an ambiguous local jet returns Undecided, not a guessed type

- GIVEN a seeded near-tangent contact whose relative second fundamental form is within
  the model-scale-derived curvature-noise band (the type is not robustly decidable)
- WHEN the S4-b classifier types it
- THEN it SHALL return `TangentContact` of type `Undecided` (the engine falls back to
  OCCT, reported) — it SHALL NOT guess `TangentPoint`, `TangentCurve`, or
  `NearTangentTransversal`

### Requirement: The S3 near-tangent hand-off carries the typed contact (additive)

The S3 marching tracer (`src/native/ssi/marching.h`) SHALL carry the typed
`TangentContact` classification of WHY a march stopped at a tangency, additively,
WITHOUT changing the tracer's transversal-only stepping. A `WLine` whose `status` is
`TraceStatus::NearTangent` SHALL carry an optional `TangentContact` stop reason (computed
by the S4-b seeded classifier at the stop point); `TraceSet.nearTangentGaps` SHALL be
unchanged as a count. The tracer SHALL still stop AT the tangency and march only up to it
— it SHALL NOT step through the tangency (that is S4-c, out of scope). No `cc_*` entry
point SHALL be added or changed, and the marching entry points SHALL remain compiled
under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a march that stops at a tangency reports the typed reason

- GIVEN a transversal march that runs into a near-tangent region and stops with `status
  == TraceStatus::NearTangent`
- WHEN the tracer finalises the `WLine`
- THEN the `WLine` SHALL carry a typed `TangentContact` stop reason classifying the
  tangency (point / curve / near-tangent transversal), and `TraceSet.nearTangentGaps`
  SHALL still count that gap
- AND the number of traced nodes SHALL be unchanged from the pre-typing behaviour — no
  points SHALL have been fabricated past the tangency

### Requirement: Native near-tangent marching-through of a continuing curve (S4-c)

The kernel SHALL provide a native, **OCCT-free** near-tangent marching capability in
`cybercad::native::ssi` that, when the S3 marcher reaches a near-tangent region
(‖n₁ × n₂‖ below the near-tangent gate) whose intersection curve GENUINELY CONTINUES on the
same branch, MARCHES THROUGH the region and emits the FULL curve rather than truncating.
The capability SHALL use a robust corrector that stays well-posed as ‖n₁ × n₂‖ → 0 by
pinning the new node to a FIXED PLANE perpendicular to the LAST-GOOD intersection tangent at
arc-distance equal to the step (a constrained residual-minimization over the native-numerics
substrate — minimizing the on-both-surfaces gap subject to the fixed-plane cut — NOT the
along-local-tangent Newton corrector that degenerates as the local tangent ill-conditions).
It SHALL seed that corrector with a curvature-aware predictor that bends the first-order
guess by the discrete curvature of the last two accepted nodes, and SHALL cross the
minimum-clearance region with a reduced step, resuming the normal deflection-driven step
once ‖n₁ × n₂‖ recovers. Outside the low-sine band the transversal corrector, accept test,
and deflection controller SHALL be unchanged, so every transversal march is traced as
before. The near-tangent regions the marcher successfully crosses SHALL be reported
(a per-branch and per-`TraceSet` crossed count) and SHALL NOT be counted in
`nearTangentGaps` (they are completed arcs). `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed; the marching entry points
SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a currently-truncating near-tangent transversal curve is now fully traced

- GIVEN a surface pair whose intersection curve grazes a near-tangent region but genuinely
  continues on the same branch (e.g. two equal cylinders whose axes cross, whose S3 trace
  today stops at the grazing region with `TraceStatus::NearTangent`,
  `stopReason == NearTangentTransversal`, and `nearTangentGaps == 1`), with
  `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-c near-tangent marcher traces the pair
- THEN it SHALL MARCH THROUGH the near-tangent region using the fixed-plane-cut corrector
  and the curvature-aware predictor, emit the FULL intersection curve, report the crossed
  region in the crossed count, and yield `nearTangentGaps == 0` for that curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march outside the low-sine band is unchanged

- GIVEN a transversal surface pair whose intersection never enters the low-sine band
- WHEN the S4-c marcher traces it
- THEN the traced curve SHALL be identical to the S3 result (same nodes, same status, same
  counts) — the corrector, accept test, and deflection controller SHALL be unchanged outside
  the low-sine band

### Requirement: The crossable gate defers genuine tangency and branch crossings (S4-c honesty)

The S4-c marcher SHALL attempt to march through a near-tangent region ONLY when the S4-b
tangent-contact classification at the stall is `NearTangentTransversal` AND the local
configuration is a SINGLE-BRANCH graze (the curve continues on the same branch), and SHALL
VERIFY any crossing before accepting it. When the S4-b classification is `TangentPoint` (an
isolated 0-dimensional genuine tangency where the curve does NOT continue), `TangentCurve`
(the surfaces are tangent along a whole seam), or `Undecided` (the local jet is within the
curvature-noise band), OR when the stall is a BRANCH CROSSING (multiple curve branches
meeting — S4-d), OR when the fixed-plane corrector cannot converge on both surfaces at the
minimum step, the marcher SHALL STOP, record the typed `TangentContact` stop reason, count
the region in `nearTangentGaps`, and DEFER it (→ OCCT) — exactly as the S3/S4-b tracer does
today. A crossing arc SHALL be accepted only if every node on it lies on BOTH surfaces
within `onSurfTol` and advances monotonically along the last-good tangent onto a
far-side tangent consistent with it; a crossing that fails this verification SHALL be
DISCARDED and the march SHALL truncate at the band entry. The marcher SHALL NEVER fabricate
an intersection-curve point past a degeneracy and SHALL NEVER weaken a tolerance to force a
crossing; a region that cannot be robustly crossed SHALL remain an honestly reported gap.

#### Scenario: a genuine tangency still stops and is classified, never crossed

- GIVEN a genuinely tangent surface pair — two spheres at centre distance `d = R₁ + R₂`
  (externally tangent, an isolated `TangentPoint`), or a sphere tangent to a cylinder along a
  circle (a `TangentCurve`)
- WHEN the S4-c marcher encounters the tangency
- THEN it SHALL STOP, classify the contact (`TangentPoint` / `TangentCurve`), count it in
  the deferred / near-tangent-gap tally, and NOT march through it
- AND it SHALL NOT fabricate any intersection-curve point across the tangency

#### Scenario: a branch crossing is deferred to S4-d, not crossed

- GIVEN a near-tangent stall that is a BRANCH CROSSING (multiple intersection-curve branches
  meeting at the point — e.g. the exact saddle where two equal crossing cylinders' branches
  touch), so more than one continuing branch is present
- WHEN the S4-c crossable gate evaluates it
- THEN it SHALL STOP and DEFER the region (S4-d owns branch topology), counting it in
  `nearTangentGaps` — it SHALL NOT march a fabricated single branch through the crossing

#### Scenario: an unverifiable or non-convergent crossing truncates honestly

- GIVEN a near-tangent region classified `NearTangentTransversal` where either the
  fixed-plane corrector cannot converge on both surfaces at the minimum step (a deep,
  near-coincident band) or a tentative crossing arc fails the on-both-surfaces / monotone
  verification
- WHEN the S4-c marcher attempts the crossing
- THEN it SHALL DISCARD any tentative arc, STOP at the band entry, and count the region in
  `nearTangentGaps` (deferred → OCCT) — it SHALL NOT emit a partially fabricated arc and
  SHALL NOT weaken a tolerance to make the crossing pass

### Requirement: Native branch-point localization, arm enumeration, and routing (S4-d)

The kernel SHALL provide a native, **OCCT-free** branch-point capability in
`cybercad::native::ssi` that, when the S3/S4-c marcher reaches a genuine SELF-CROSSING of the
intersection locus (a point where multiple real intersection-curve arms meet — detected by the
existing S4-c transversality-sine COLLAPSE and raw-tangent FLIP that today force a defer),
LOCALIZES the branch point, ENUMERATES the outgoing arms, ROUTES the march down each arm, and
ASSEMBLES the multi-arm curve — rather than truncating at the crossing. The capability SHALL
LOCALIZE the branch point B as the on-both-surfaces point where the transversality sine
‖n₁ × n₂‖ reaches its minimum (≈ 0) along the approach, with B on BOTH surfaces within
`onSurfTol`. It SHALL ENUMERATE the outgoing arm directions as the REAL, distinct roots of the
tangent-cone quadratic formed from the local second-order structure of the two surfaces at B
(the relative second fundamental form restricted to the shared tangent plane) — a transversal
self-crossing (indefinite form) yielding two distinct tangent lines and hence up to four
outgoing rays; it SHALL return ONLY the real distinct directions and, when the quadratic has NO
two distinct real roots, it SHALL enumerate NO arms. It SHALL ROUTE each enumerated arm by
stepping off B a small distance along the ray, re-projecting onto both surfaces with the
fixed-plane corrector (verified within `onSurfTol`), then continuing the normal march to
termination; and it SHALL DEDUP an arm that retraces an already-traced arm and CONNECT the arms
meeting at the same branch point into a branch-point node in the assembled result. The
branch points the marcher localizes and routes SHALL be reported (a per-`TraceSet` branch-point
count and the localized point with its connected arm ids) and SHALL NOT be counted in
`nearTangentGaps`. Outside a detected branch point the transversal S3 trace and the S4-c
crossable-graze crossing SHALL be unchanged. `src/native/**` SHALL NOT link OCCT; no `cc_*`
entry point, signature, or POD struct SHALL be added or changed; the marching entry points
SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the Steinmetz self-crossing is now fully traced

- GIVEN two equal-radius cylinders (R = 1) whose axes cross orthogonally at the origin (the
  Steinmetz bicylinder — its intersection is two ellipses that CROSS each other at two branch
  points), whose S3+S4-c trace today stops at the branch point with `TraceStatus::NearTangent`,
  `stopReason == NearTangentTransversal`, `tracedBranches == 0`, and `nearTangentGaps == 1`,
  with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-d marcher traces the pair
- THEN it SHALL LOCALIZE both branch points (each on BOTH cylinders within `onSurfTol`, at a
  near-zero transversality sine), ENUMERATE the outgoing arms from the tangent cone, ROUTE the
  march down each arm, and ASSEMBLE the two crossing ellipses — reporting `branchPoints == 2`
  and yielding `nearTangentGaps == 0` for the traced structure
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march and the S4-c graze are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a branch point, and a
  surface pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c
  slice marches through
- WHEN the S4-d marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same
  status, same counts) and the graze SHALL still be MARCHED THROUGH by the S4-c crossing
  (reported in the crossed count, `branchPoints == 0` for both) — the branch machinery SHALL
  engage ONLY at a detected branch point

### Requirement: Branch-point honesty — isolated tangents end and unresolved branches defer (S4-d)

The S4-d marcher SHALL route arms out of a stall ONLY when it is a genuine branch point — a
self-crossing of the intersection locus whose tangent-cone quadratic has TWO DISTINCT REAL
roots (an indefinite relative second fundamental form). When the S4-b tangent-contact
classification at the stall is `TangentPoint` (an isolated 0-dimensional genuine tangency where
the curve does NOT continue — a sign-definite relative second form, equivalently no two
distinct real tangent-cone roots), the marcher SHALL let the curve END there and SHALL NOT
fabricate any outgoing arm. When the classification is `TangentCurve` or `Undecided`, when the
tangent-cone quadratic has a double root (a cusp / degenerate branch, out of scope), when the
branch point cannot be localized on both surfaces within `onSurfTol`, or when a would-be arm's
first step cannot be verified on both surfaces within `onSurfTol`, the marcher SHALL STOP,
record the typed `TangentContact` stop reason, count the region in `nearTangentGaps`, and DEFER
it (→ OCCT) — exactly as the S4-c tracer does today — or DROP the unverifiable arm. The marcher
SHALL NEVER fabricate an arm or an intersection-curve point past a degeneracy and SHALL NEVER
weaken a tolerance to force a branch; a branch that cannot be robustly resolved SHALL remain an
honestly reported gap.

#### Scenario: an isolated tangent point still ends, never sprouting arms

- GIVEN a genuinely tangent surface pair — two spheres at centre distance `d = R₁ + R₂`
  (externally tangent, an isolated `TangentPoint` with a sign-definite relative second form)
- WHEN the S4-d marcher encounters the tangency
- THEN it SHALL let the curve END at the isolated contact, classify it (`TangentPoint`), and
  enumerate NO outgoing arms (`branchPoints == 0`)
- AND it SHALL NOT fabricate any intersection-curve arm or point across the tangency

#### Scenario: a branch point that cannot be robustly resolved defers honestly

- GIVEN a suspected branch point where either the localization does not re-project onto both
  surfaces within `onSurfTol`, the tangent-cone quadratic has no two distinct real roots (a
  definite form or a double root), or a would-be arm's first step off the branch point fails
  the on-both-surfaces verification
- WHEN the S4-d marcher attempts to localize / enumerate / route it
- THEN it SHALL DROP any unverifiable arm, STOP at the stall, record the typed
  `TangentContact` stop reason, and count the region in `nearTangentGaps` (deferred → OCCT) —
  it SHALL NOT fabricate an arm or a point and SHALL NOT weaken a tolerance to force the branch

### Requirement: The S4-d branched TraceSet is the consumed input contract for the S5-d Steinmetz common

The S4-d `cybercad::native::ssi` branched `TraceSet` SHALL be the input contract consumed by ALL
THREE native S5-d Steinmetz-family branched curved booleans
(`src/native/boolean/ssi_boolean.cpp`): `buildSteinmetzCommon` (COMMON), `buildSteinmetzFuse`
(FUSE), and `buildSteinmetzCut` (CUT). For an equal-radius orthogonal crossing cylinder pair,
each boolean SHALL obtain the branched `TraceSet` by tracing with
`MarchOptions.enableBranchPoints = true` and use its two `BranchNode`s (the localized branch
points, each on both cylinders within `onSurfTol`) and its four `BranchArc` `WLine`s — each arm's
per-node `(u1,v1,u2,v2)` on both cylinders (the arc track used to split each wall into lune
patches) and its shared 3D nodes (the seam vertices the patches weld on, plus the two shared
branch-point vertices). The SAME four arcs SHALL split each cylinder wall into an INSIDE region
(the lune, inside the other cylinder) and an OUTSIDE region; the ops differ ONLY in fragment
selection, orientation, and cap handling: COMMON assembles the four INSIDE lunes (the
bicylinder), FUSE the four OUTSIDE lune walls plus both cylinders' two original disc end caps
(the outer envelope), and CUT the OUTSIDE walls plus disc caps of the minuend plus the INSIDE
lunes of the subtrahend REVERSED (the carved channel). Each boolean SHALL consume the branched
`TraceSet` ONLY when it is fully resolved — `nearTangentGaps == 0`, `branchPoints == 2` with
`branchNodes.size() == 2`, exactly FOUR `WLine`s all of `status == BranchArc`, every arm on both
cylinders within `onSurfTol`, and the two branch nodes connecting all four arms. A branched
`TraceSet` with `nearTangentGaps > 0`, `branchPoints != 2`, a WLine set that is not four
`BranchArc` arms, or arms that do not meet at the two branch nodes SHALL be treated as the honest
fallback boundary and SHALL NOT be consumed by any of the three ops (the boolean declines →
OCCT). The branched re-trace SHALL be entered ONLY after the DEFAULT (unbranched) trace has
declined AND the Steinmetz pre-gate (both cylinders, equal radii, orthogonal crossing axes)
matches, so the single-seam S3 transversal trace the S5-a/b/c paths consume is UNCHANGED. The
tracer SHALL NOT change to serve this consumption — the contract is the already-shipped S4-d
output; no `cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI
module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like the S3/S4-d tracer).

#### Scenario: the branched Steinmetz TraceSet is consumed to weld the four lunes

- GIVEN an equal-radius orthogonal crossing cylinder pair whose S4-d branched `TraceSet`
  (`enableBranchPoints = true`) has `nearTangentGaps == 0`, `branchPoints == 2`, and exactly
  four `BranchArc` arms meeting at the two branch nodes
- WHEN the S5-d curved boolean consumes the branched `TraceSet`
- THEN it SHALL split each cylinder wall along its two arcs into the inside-the-other lune
  patches, with every arc node on both cylinders within `onSurfTol`, and the four lune patches
  SHALL weld along the four arcs and the two shared branch-point vertices
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: the four arcs feed fuse and cut as they already feed common

- GIVEN an equal-radius orthogonal crossing cylinder pair whose S4-d branched `TraceSet` is
  fully resolved (`branchPoints == 2`, four `BranchArc` arms, `nearTangentGaps == 0`) with
  `(u1,v1,u2,v2)` per node
- WHEN the native boolean path runs `Op::Fuse` or `Op::Cut`
- THEN the SAME four arcs SHALL be oriented and pole-axis-resampled ONCE (the shared prologue) and
  shared by the fragments the op selects (both cylinders' OUTSIDE lunes plus their disc end caps
  for FUSE; the minuend's OUTSIDE lunes plus its caps plus the subtrahend's INSIDE lunes REVERSED
  for CUT), so the fragments weld watertight along the four arcs and the two shared branch-point
  poles
- AND no additional trace, re-trace, or arc SHALL be required beyond the four `BranchArc` arms
  the S4-d re-trace already produces

#### Scenario: a non-Steinmetz or unresolved branched TraceSet is the fallback boundary, not consumed

- GIVEN a branched `TraceSet` that reports `nearTangentGaps > 0` (an arm the S4-d marcher could
  not resolve), or `branchPoints != 2`, or a WLine set that is not exactly four `BranchArc`
  arms, or arms that do not meet at the two branch nodes
- WHEN any of the S5-d curved booleans (COMMON, FUSE, CUT) inspects the branched `TraceSet`
- THEN it SHALL decline to consume the trace (the honest fallback boundary) and the boolean
  SHALL fall back to OCCT, reported — never welding a shell on a truncated, mismatched, or
  fabricated branched structure

#### Scenario: the default single-seam trace is unchanged for non-Steinmetz pairs

- GIVEN a transversal surface pair whose DEFAULT (unbranched) trace is a clean single-seam
  transversal (a through-drill cylinder pair or a sphere-lens pair) OR any pair the Steinmetz
  pre-gate does not match
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume the DEFAULT `TraceSet` (branch points OFF) exactly as the single-seam
  S5-a/b/c paths do, and SHALL NOT enter the branched re-trace — the branch machinery engages
  ONLY on the declined edge when the Steinmetz pre-gate matches

### Requirement: Native chart-singularity detection and point-based crossing (S4-e)

The kernel SHALL provide a native, **OCCT-free** chart-singularity capability in
`cybercad::native::ssi` that, when the S3/S4-c/S4-d marcher reaches a point where a SINGLE
surface's own `(u,v)` PARAMETRIZATION is singular (a sphere parametric pole where `‖dU‖`
collapses to zero, or a cone apex where the signed radius and hence `‖dU‖` collapses), while the
3D point and the surface normal remain well-defined, DETECTS the chart collapse, STEPS across the
singular band, and RESUMES the normal march — rather than truncating at the singularity. The
capability SHALL DETECT the singularity via a SINGLE-surface Jacobian rank-drop — `‖dU‖`
collapsing relative to `‖dV‖` and the model scale on one surface while that surface's normal
stays finite — and this detection SHALL be INDEPENDENT of the S4-c pair transversality sine
`‖n₁ × n₂‖` (which need not collapse at a pole) and the S4-d locus tangent flip. It SHALL STEP
across the singular band with a POINT-BASED corrector that does NOT depend on the degenerate
`dU` (the fixed-plane cut whose residuals use only the surface point and the last-good tangent),
mapping the far-side chart coordinates back LOOSELY — at a sphere pole pinning the arbitrary
longitude from the continuity of the incoming arc and clamping the pole latitude, and at a cone
apex treating the apex as a single 3D point the curve passes through. It SHALL enter the band
with a fine step so the singularity is resolved rather than leapt, and SHALL resume the normal
`(u,v)` march once `‖dU‖` recovers on both surfaces. The chart singularities the marcher steps
across and verifies SHALL be reported (a per-`TraceSet` chart-singularities-crossed count and a
per-`WLine` crossed count) and SHALL NOT be counted in `nearTangentGaps`. Outside a detected
chart singularity the transversal S3 trace, the S4-c crossable-graze crossing, and the S4-d
branch-point trace SHALL be unchanged. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry
point, signature, or POD struct SHALL be added or changed; the marching entry points SHALL remain
compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the sphere-pole great circle is now fully traced

- GIVEN a unit sphere and a plane through the sphere's axis (the plane `y = 0`), whose
  intersection is a great circle passing through BOTH sphere parametric poles (`v = ±π/2`, where
  `‖dU‖` collapses), forced through marching, whose S3 trace today stops at the pole as a
  `BoundaryExit` covering only a single pole-to-pole meridian (arc length ≈ π, half the closed
  loop), with `CYBERCAD_HAS_NUMSCI` built and chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at each pole, STEP across it with the point-based
  corrector (pinning the outgoing longitude from arc continuity), and RESUME the march on the
  opposite meridian — assembling the FULL closed great circle (arc length ≈ `2π` within the
  deflection tolerance, both meridians visited, both poles crossed), reporting at least two chart
  singularities crossed and yielding `nearTangentGaps == 0` for the traced curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the cone-apex line is now crossed

- GIVEN a double cone whose apex is at the origin (reference radius zero at the apex) and a plane
  through the apex (the plane `y = 0`), whose intersection is a line crossing the apex and
  spanning both nappes, forced through marching, whose S3 trace today stalls just short of the
  apex (`‖dU‖ → 0` collapses the parameter step, exhausting the node budget) and never reaches
  the far nappe, with chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at the apex, treat the apex as a single 3D point,
  STEP across it with the point-based corrector, and RESUME the march on the far nappe — tracing
  the full apex-crossing line spanning both nappes in a BOUNDED node count, reporting at least
  one chart singularity crossed
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the transversal march, the S4-c graze, and the S4-d branch trace are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a chart singularity, a
  surface pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c
  slice marches through, and the Steinmetz bicylinder whose intersection self-crosses at two
  branch points the S4-d slice localizes and routes
- WHEN the S4-e marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same status,
  same counts), the graze SHALL still be MARCHED THROUGH by the S4-c crossing, and the Steinmetz
  SHALL still be assembled by the S4-d branch machinery (`branchPoints == 2`) — each with zero
  chart singularities crossed, because the chart machinery SHALL engage ONLY at a detected
  single-surface chart collapse

### Requirement: Chart-singularity honesty — genuine boundaries exit, cusps end, unresolved singularities defer (S4-e)

The S4-e marcher SHALL step across a `v`-edge only when it is a genuine PARAMETRIC SINGULARITY —
a sphere pole or cone apex where `‖dU‖` collapses while the surface normal stays finite. When the
`v`-edge is a genuine DOMAIN BOUNDARY (a finite surface's cap edge, where `‖dU‖` does NOT
collapse), the marcher SHALL let the curve EXIT as a boundary exit and SHALL NOT attempt a
crossing. When the intersection curve has a genuine CUSP endpoint (the curve velocity collapses
while both surfaces' charts are regular and the point-based step cannot continue through it), the
marcher SHALL let the curve END there and SHALL NOT fabricate a continuation. When a chart
singularity's far-side node cannot be re-projected onto both surfaces within `onSurfTol`, when the
crossing makes no real far-side progress, or when a pole's continuity-pinned outgoing node fails
the on-both-surfaces verification, the marcher SHALL DISCARD the crossing arc, STOP, record the
typed stop reason, count the region in `nearTangentGaps`, and DEFER it (→ OCCT) — reporting the
measured gap. The marcher SHALL NEVER fabricate a point across a singularity and SHALL NEVER
weaken a tolerance to force a crossing; a singularity that cannot be robustly crossed SHALL remain
an honestly reported gap.

#### Scenario: a genuine domain boundary still exits, never fabricating a crossing

- GIVEN a finite surface whose intersection curve runs to a real domain-boundary `v`-edge (a
  finite cylinder's cap edge, where `‖dU‖` does NOT collapse and there is no surface beyond the
  edge)
- WHEN the S4-e marcher reaches the edge
- THEN it SHALL let the curve EXIT as a boundary exit, reporting zero chart singularities crossed
- AND it SHALL NOT fabricate any curve node beyond the boundary

#### Scenario: a chart singularity that cannot be robustly crossed defers honestly

- GIVEN a detected chart singularity where either the far-side node does not re-project onto both
  surfaces within `onSurfTol`, the crossing makes no real far-side progress, or a pole's
  continuity-pinned outgoing node fails the on-both-surfaces verification
- WHEN the S4-e marcher attempts to step across it
- THEN it SHALL DISCARD the crossing arc, STOP at the singularity, record the typed stop reason,
  and count the region in `nearTangentGaps` (deferred → OCCT) — it SHALL NOT fabricate a point
  across the singularity and SHALL NOT weaken a tolerance to force the crossing

### Requirement: Native robust loop closure and self-intersection detection (S4-f part 1)

The kernel SHALL provide, in `cybercad::native::ssi`, a native, **OCCT-free** robust-closure and
self-intersection capability. Loop closure SHALL be a TRUE-RETURN test: a march SHALL declare a
closed loop ONLY when it returns within the loop-closure proximity radius of the SEED **AND** its
current forward tangent is CONTINUOUS with the seed's outgoing tangent (heading the way it left).
A march that returns NEAR the seed but heading substantially the OTHER way (a near-antiparallel
pass-through) SHALL NOT be declared closed; it SHALL continue to its true termination. This
true-return test SHALL reduce to the current proximity result for any curve that truly closes (it
tightens a necessary condition — it can only REFUSE a close the proximity test would have made,
never MAKE one), so the transversal S3 traces and every existing loop-closing case SHALL be
unchanged. The capability SHALL additionally DETECT a genuine SELF-INTERSECTION — the traced arm
crossing an EARLIER NON-SEED node of its own history within a tolerance-scaled radius with a
TRANSVERSE (non-continuation, non-retrace) tangent — and SHALL record it as a typed
self-intersection with its point and both surface parameters, WITHOUT declaring a loop close there
and WITHOUT stopping the march (the arm SHALL continue to its true termination). A self-intersection
SHALL be DISTINCT from an S4-d locus branch point: at a curve self-crossing both surfaces are
transversal (the pair transversality sine does not collapse) and no new arms emanate, so the
self-intersection witness (a node-history crossing) SHALL be independent of the S4-c pair sine and
the S4-d locus tangent flip. An arm merely running back over itself (a retrace, tangents nearly
parallel or antiparallel) SHALL NOT be reported as a self-intersection. The self-intersection guard
SHALL be behind a default-off switch; with it off the trace SHALL be byte-identical to the current
S3 result. The self-intersections detected SHALL be reported (a per-`WLine` count and typed list
and a per-`TraceSet` total). `src/native/**` SHALL NOT link OCCT; no `cc_*` entry point, signature,
or POD struct SHALL be added or changed; the marching entry points SHALL remain compiled under
`CYBERCAD_HAS_NUMSCI`.

#### Scenario: a curve passing near its seed the other way is not false-closed

- GIVEN an open or longer intersection curve whose S3 trace passes within the loop-closure
  proximity radius of the seed while heading substantially the OTHER way (a near-antiparallel
  pass-through), which the current pure-proximity test declares `Closed` and truncates at the
  near-pass, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the S4-f marcher traces the pair
- THEN it SHALL NOT declare the curve closed at the near-pass (the return tangent is not continuous
  with the seed's outgoing tangent) and SHALL continue the march to the curve's TRUE termination
- AND the traced curve SHALL match the analytic ground truth end-to-end (arc length / endpoints
  within tolerance), never fabricating a closure

#### Scenario: a genuine self-intersection is detected and traced through

- GIVEN a single-arm intersection curve that genuinely self-crosses (one arm passing through the
  same 3D point twice, headed differently, while BOTH surfaces stay transversal — distinct from an
  S4-d branch where the intersection locus branches into multiple arms), with the self-intersection
  guard enabled
- WHEN the S4-f marcher traces the pair
- THEN it SHALL DETECT the self-crossing as a typed self-intersection at the crossing point (on
  both surfaces within `onSurfTol`), report at least one self-intersection, and continue the arm
  THROUGH the crossing to its true termination — it SHALL NOT declare a loop close at the crossing
  and SHALL NOT step past it unrecorded
- AND the self-intersection SHALL be reported as distinct from an S4-d branch point (the trace of
  this fixture yields zero localized branch points)

#### Scenario: the transversal closure, the S4-c graze, the S4-d branch trace, and the S4-e crossings are unchanged

- GIVEN the 5 transversal surface pairs whose intersection curves close by returning to the seed
  tangent-continuously, the S4-c `NearTangentTransversal` graze, the Steinmetz bicylinder whose
  locus self-crosses at two S4-d branch points, and the sphere-pole / cone-apex pairs the S4-e
  slice crosses
- WHEN the S4-f marcher traces them (with the self-intersection guard off)
- THEN each transversal curve SHALL still close (identical nodes, status, and counts to the S3
  result), the S4-c graze SHALL still be marched through, the S4-d Steinmetz SHALL still be
  assembled (two branch points), and the S4-e pole + apex SHALL still be crossed — each with zero
  self-intersections reported, because the true-return closure reduces to the current result on
  truly-closing curves and the self-intersection guard is off

### Requirement: Native bounded adaptive completeness-critic re-seed with an honestly reported recall floor (S4-f part 2)

The kernel SHALL provide, in `cybercad::native::ssi`, a native, **OCCT-free** completeness-critic
capability that, after the initial fixed-resolution seed and trace, RECOVERS small intersection
loops the fixed subdivision floor (default leaf fraction 1/32) silently missed. The critic SHALL,
when enabled, compute the param regions NOT covered by any traced curve, RE-SUBDIVIDE those
uncovered regions at a FINER resolution than the previous round, refine each new candidate to a
point on BOTH surfaces at the SAME `onSurfTol` (a candidate that does not refine to an
on-both-surfaces point, or that is near-tangent, SHALL be DISCARDED — never a fabricated seed),
DEDUP the new seeds against all kept curves, and TRACE each genuinely new seed. The critic SHALL
REPEAT this re-seed round LOOP-UNTIL-DRY — stopping after a configured number of CONSECUTIVE rounds
that recover NO new branch — OR when a hard cost cap (a maximum number of rounds and a maximum total
number of re-seed candidate regions) is reached, whichever occurs first. Every recovered loop SHALL
be a VERIFIED on-both-surfaces seed that refined to a real curve; the critic SHALL NEVER fabricate a
branch. The critic SHALL be behind a default-off switch; with it off the seeding and trace SHALL be
byte-identical to the current fixed-resolution result. The capability SHALL report the completeness
FLOOR it reached — the finest re-seed leaf fraction, the number of rounds run, and whether it
stopped DRY (the configured dry rounds elapsed with no new branch) or on the COST CAP — together
with the MEASURED recall (native branches carrying at least one seed ÷ the true transversal branch
count), which SHALL NEVER be asserted a blind 1.0. The report SHALL ALWAYS acknowledge the RESIDUAL:
a loop smaller than the finest re-seed round can still exist, so completeness is MEASURED and
ASYMPTOTIC, not a guarantee. `src/native/**` SHALL NOT link OCCT; no `cc_*` entry point, signature,
or POD struct SHALL be added or changed; the critic SHALL be compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a small loop below the fixed floor is recovered

- GIVEN a surface pair whose intersection has a small loop lying entirely inside one default (1/32)
  leaf cell, so the fixed-resolution seeder produces NO seed for it and the measured recall is below
  1 at the default resolution, with `CYBERCAD_HAS_NUMSCI` built
- WHEN the completeness critic is enabled and the pair is traced
- THEN the critic SHALL re-seed the uncovered region at a finer resolution, refine a verified
  on-both-surfaces seed for the small loop, trace it, and recover the loop — raising the measured
  recall to 1 ON THIS FIXTURE at the reached floor, with at least one recovered loop reported and
  the reached leaf fraction finer than 1/32
- AND the report SHALL STILL acknowledge the residual (a loop smaller than the reached floor can
  still exist) — the recall figure is a MEASURED win on this fixture at this floor, not a
  completeness proof

#### Scenario: many small loops measurably raise recall vs OCCT, with the floor and residual reported

- GIVEN an adversarial surface pair with several disjoint small loops (the wrap-emboss / blend seam
  pattern), most of them below the fixed seeding floor, so the default measured recall is well
  below 1
- WHEN the completeness critic is enabled and the pair is traced
- THEN the critic SHALL MEASURABLY raise the recall over the default (recovering loops down to the
  reached floor via the loop-until-dry re-seed), and SHALL report the reached floor, the number of
  rounds, whether it stopped dry or on the cost cap, and the measured recall against the OCCT branch
  count
- AND the report SHALL acknowledge the residual (a loop below the reached floor can still be
  missed) — it SHALL NOT claim total completeness, and no branch SHALL be fabricated

#### Scenario: a re-seed candidate that does not verify is discarded, never fabricated

- GIVEN a completeness-critic re-seed round in an uncovered region where a candidate fails to refine
  to a point on both surfaces within `onSurfTol`, or is near-tangent
- WHEN the critic processes the candidate
- THEN it SHALL DISCARD the candidate and SHALL NOT emit a seed or a branch for it — the recovered
  set contains only verified on-both-surfaces seeds that traced to real curves
- AND a loop the critic cannot recover within its cost cap SHALL be reported as a measured recall
  below 1 with the residual acknowledged, never a faked branch and never a weakened tolerance

### Requirement: The S1 analytic coaxial-cone circle seam is the consumed input contract for the S5 cone common

The S1 `cybercad::native::ssi` analytic layer's coaxial-cone circle seam SHALL be the input
contract consumed by the native S5 cone COMMON booleans (`src/native/boolean/ssi_boolean.cpp`):
`buildConeCylCommon` (coaxial cone∩cylinder) and, optionally, `buildConeSphereCommon` (coaxial
cone∩sphere, single-crossing config). For a COAXIAL frustum cone and cylinder, the boolean SHALL
obtain the `TraceSet` and use the SINGLE closed full-turn `WLine` circle
(`intersectCylinderConeCoaxial`, radius `Rc` at the crossing height `h*` where the cone
cross-section radius `r_c(h*) = Rc`) — its per-node `(u1,v1,u2,v2)` on both walls (the seam-circle
track) and its shared 3D nodes (the seam vertices both bands weld on through the shared
`VertexPool`). The SAME seam SHALL split the axial overlap into a CONE-tighter sub-band (kept as
the cone wall, inside the cylinder) and a CYLINDER-tighter sub-band (kept as the cylinder wall,
inside the cone); the common assembles the frustum band welded along the seam circle to the
cylinder-segment band, closed by two disc caps. The boolean SHALL consume the seam ONLY when it is
a fully transversal single interior circle — `nearTangentGaps == 0`, `branchPoints == 0`, exactly
ONE full-circle `WLine` on BOTH walls, the frustum apex-free over its extent, and `h*` strictly
inside both operand extents. A seam with `nearTangentGaps > 0`, a `NearTangent` / `Failed` /
`BranchArc` WLine, a two-circle coaxial crossing, an apex-crossing circle, or a non-coaxial
(transversal) cone pair whose true seam is a quartic space curve (`intersectCylinderConeCoaxial`
returns `notAnalytic`) SHALL be treated as the honest S4 / OCCT boundary and SHALL NOT be consumed
(the boolean declines → OCCT). The tracer SHALL NOT change to serve this consumption — the
contract is the already-shipped S1 output; no `cc_*` entry point, signature, or POD struct SHALL be
added or changed, and the SSI module SHALL remain OCCT-free and compiled under
`CYBERCAD_HAS_NUMSCI` (like the S1 analytic layer).

#### Scenario: the single coaxial-cone circle seam is consumed to weld the frustum and cylinder bands
- GIVEN a coaxial frustum-cone∩cylinder pair whose S1 seam is exactly one closed full-turn circle
  (radius `Rc` at `h*`), `nearTangentGaps == 0`, `branchPoints == 0`, the frustum apex-free and
  `h*` strictly inside both extents
- WHEN the S5 cone COMMON boolean consumes the seam
- THEN it SHALL resample the circle into one pooled full-turn seam ring, split the axial overlap at
  `h*` into the cone-tighter and cylinder-tighter sub-bands, and weld the frustum band and the
  cylinder-segment band along that single seam ring, with every seam node on BOTH walls ≤ tol
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a transversal, two-circle, or apex-crossing cone seam is the decline boundary, not consumed
- GIVEN a NON-coaxial (transversal) cone∩cylinder pair whose true seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a coaxial cone∩sphere pair whose S1
  seam is TWO circles inside both extents, OR a coaxial pair whose circle passes through the cone
  apex (a frustum extent reaching `r_c → 0`)
- WHEN the S5 cone COMMON boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic, multi-circle, or
  apex-crossing seam

#### Scenario: the default single-seam trace is unchanged for non-cone pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams) or a
  sphere∩sphere lens (one closed circle), OR any pair that is not a coaxial cone pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz paths
  do, and the cone COMMON builders SHALL return NULL for it — the coaxial-cone seam machinery
  engages ONLY when one operand is a `Cone` and the other is a coaxial `Cylinder` (or `Sphere`)

### Requirement: The S1 analytic coaxial cone∩cylinder circle seam is the shared input contract for all three cone∩cylinder ops

The S1 `cybercad::native::ssi` analytic coaxial cone∩cylinder circle seam SHALL be the input
contract consumed by ALL THREE native S5-e coaxial cone∩cylinder booleans (via
`intersectCylinderConeCoaxial`): `buildConeCylCommon` (already native), `buildConeCylFuse`, and
`buildConeCylCut`. For a COAXIAL frustum cone and cylinder, each boolean SHALL obtain the
`TraceSet` and use the SINGLE closed full-turn `WLine` circle (radius `Rc` at the crossing height
`s*` where `r_c(s*) = Rc`) — its shared 3D nodes (the seam vertices all surviving bands weld on
through the shared `VertexPool`, pooled ONCE). The SAME seam SHALL split the axial overlap into a
CONE-tighter sub-band and a CYLINDER-tighter sub-band; COMMON welds the two INSIDE bands (the min-
radius profile), FUSE welds the two OUTER bands (the max-radius profile) plus the beyond-overlap
wall segments and the operand end caps / annular steps, and CUT welds A's OUTER wall to B's INSIDE
band emitted REVERSED (bounding the carved cavity). Each boolean SHALL consume the seam ONLY when it
is a fully transversal single interior circle — `nearTangentGaps == 0`, `branchPoints == 0`,
exactly ONE full-circle `WLine` on BOTH walls, the frustum apex-free over its extent, and `s*`
strictly inside both operand extents. A seam with `nearTangentGaps > 0`, a `NearTangent` / `Failed`
/ `BranchArc` WLine, an apex-crossing circle, a cap-edge-tangent crossing, or a non-coaxial
(transversal) cone pair whose true seam is a quartic space curve (`intersectCylinderConeCoaxial`
returns `notAnalytic`) SHALL be treated as the honest S4 / OCCT boundary and SHALL NOT be consumed
(the boolean declines → OCCT). The tracer SHALL NOT change to serve this consumption — the contract
is the already-shipped S1 output; no `cc_*` entry point, signature, or POD struct SHALL be added or
changed, and the SSI module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like
the S1 analytic layer).

#### Scenario: the single coaxial cone∩cylinder circle seam is consumed by all three ops to weld their bands
- GIVEN a coaxial frustum-cone∩cylinder pair whose S1 seam is exactly one closed full-turn circle
  (radius `Rc` at `s*`), `nearTangentGaps == 0`, `branchPoints == 0`, the frustum apex-free and
  `s*` strictly inside both extents
- WHEN each of the S5-e cone∩cylinder COMMON, FUSE, and CUT booleans consumes the seam
- THEN each SHALL split the axial overlap at `s*` into the cone-tighter and cylinder-tighter sub-
  bands, resample the circle into one pooled full-turn seam ring shared by its surviving bands with
  every seam node on BOTH walls ≤ tol, and weld — COMMON the two inside bands, FUSE the two outer
  bands (plus caps / annular steps), CUT A's outer wall to B's reversed inside band
- AND no `cc_*` entry point SHALL have been added or changed.

#### Scenario: a transversal or apex-crossing cone∩cylinder seam is the decline boundary for all three ops
- GIVEN a NON-coaxial (transversal) cone∩cylinder pair whose true seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a coaxial pair whose circle passes
  through the cone apex (a frustum extent reaching `r_c → 0`), OR a pair whose crossing `s*` sits on
  a cap edge (a tangent, not a strictly-interior transversal circle)
- WHEN the S5-e cone∩cylinder COMMON, FUSE, or CUT boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic or apex-crossing seam.

#### Scenario: the default single-seam trace is unchanged for non-cone∩cylinder pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams) or a
  sphere∩sphere lens (one closed circle), OR any pair that is not a coaxial cone∩cylinder pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz paths
  do, and the cone∩cylinder FUSE / CUT builders SHALL return NULL for it — the coaxial cone∩cylinder
  seam machinery engages ONLY when one operand is a `Cone` and the other is a coaxial `Cylinder`.

### Requirement: The S1 analytic coaxial cone∩sphere circle seam is the shared input contract for all three cone∩sphere ops

The S1 `cybercad::native::ssi` analytic coaxial cone∩sphere circle seam SHALL be the input contract
consumed by ALL THREE native S5-f coaxial cone∩sphere booleans (via `intersectSphereConeCoaxial`):
`buildConeSphereCommon`, `buildConeSphereFuse`, and `buildConeSphereCut`. For a COAXIAL frustum cone
and a sphere whose centre lies on the cone axis, each boolean SHALL obtain the `TraceSet` and use the
SINGLE closed full-turn `WLine` circle (radius `r_c(s*)` at the crossing height `s*` where `r_c(s*)
= r_s(s*)`) — its shared 3D nodes (the seam vertices all surviving bands weld on through the shared
`VertexPool`, pooled ONCE). The SAME seam SHALL split the axial span into a CONE-tighter sub-band and
a SPHERE-tighter sub-band; COMMON welds the cone band inside the sphere to the sphere segment inside
the cone (the min-cross-section profile), FUSE welds the sphere OUTER cap to the cone OUTER band (the
max-cross-section profile), and CUT welds A's OUTER wall to the sphere INNER cap emitted REVERSED
(bounding the carved dimple). Each boolean SHALL consume the seam ONLY when it is a SINGLE fully
transversal interior circle — `nearTangentGaps == 0`, `branchPoints == 0`, exactly ONE full-circle
`WLine` on BOTH walls, the frustum apex-free over its extent, and `s*` strictly inside both operand
extents with the QUADRATIC's other root OUTSIDE the frustum extent. A seam with `nearTangentGaps >
0`, a `NearTangent` / `Failed` / `BranchArc` WLine, a TWO-circle crossing (both roots inside both
extents), an apex-crossing circle, or a non-coaxial (transversal) cone∩sphere pair whose true seam
is a quartic space curve (`intersectSphereConeCoaxial` returns `notAnalytic`) SHALL be treated as
the honest S4 / OCCT boundary and SHALL NOT be consumed (the boolean declines → OCCT). The tracer
SHALL NOT change to serve this consumption — the contract is the already-shipped S1 output; no
`cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI module SHALL
remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the single coaxial cone∩sphere circle seam is consumed by all three ops to weld their bands
- GIVEN a coaxial frustum-cone∩sphere pair (sphere centre on the cone axis) whose S1 seam is exactly
  one closed full-turn circle (radius `r_c(s*)` at `s*`), `nearTangentGaps == 0`, `branchPoints ==
  0`, the frustum apex-free, `s*` strictly inside both extents, and the QUADRATIC's other root
  outside the frustum extent
- WHEN each of the S5-f cone∩sphere COMMON, FUSE, and CUT booleans consumes the seam
- THEN each SHALL split the axial span at `s*` into the cone-tighter and sphere-tighter sub-bands,
  resample the circle into one pooled full-turn seam ring shared by its surviving cone band and
  sphere cap with every seam node on BOTH walls ≤ tol, and weld — COMMON the cone band to the sphere
  segment, FUSE the sphere outer cap to the cone outer band, CUT A's outer wall to the reversed
  sphere inner cap
- AND no `cc_*` entry point SHALL have been added or changed.

#### Scenario: a two-circle, transversal, or apex-crossing cone∩sphere seam is the decline boundary for all three ops
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone / spans the apex), OR a NON-coaxial
  (transversal) cone∩sphere pair whose true seam is a quartic space curve
  (`intersectSphereConeCoaxial` returns `notAnalytic`), OR a coaxial pair whose circle passes
  through the cone apex (a frustum extent reaching `r_c → 0`)
- WHEN the S5-f cone∩sphere COMMON, FUSE, or CUT boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic, two-circle, or apex-
  crossing seam.

#### Scenario: the default single-seam trace is unchanged for non-cone∩sphere pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams), a
  sphere∩sphere lens (one closed circle), or a coaxial cone∩cylinder (one closed circle), OR any
  pair that is not a coaxial cone∩sphere pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz /
  cone∩cylinder paths do, and the cone∩sphere COMMON / FUSE / CUT builders SHALL return NULL for it
  — the coaxial cone∩sphere seam machinery engages ONLY when one operand is a `Cone` and the other
  is a `Sphere` whose centre lies on the cone axis.

### Requirement: Native freeform parametric-pole crossing (S4-e general)

The kernel SHALL extend the native, **OCCT-free** chart-singularity capability in
`cybercad::native::ssi` to cross a FREEFORM parametric pole — a B-spline / NURBS surface with a
DEGENERATE control row (a collapsed U-row / spline cone-tip) where the surface's own `‖∂S/∂u‖`
collapses to zero at a `v` domain edge while the 3D point remains well-defined. When the
S3/S4-c/S4-d/S4-e marcher reaches such a freeform pole, it SHALL DETECT the chart collapse via the
SAME single-surface Jacobian rank-drop witness used for the analytic sphere pole and cone apex —
`‖dU‖` collapsing relative to `‖dV‖` and the model scale on that surface while the surface normal
evaluates FINITE (the degenerate freeform normal being a finite near-zero direction, never a
non-finite value) — and this detection SHALL remain INDEPENDENT of the S4-c pair transversality sine
and the S4-d locus tangent flip. It SHALL STEP across the singular band with the EXISTING
POINT-BASED fixed-plane corrector, whose residuals use only the surface point and the last-good
tangent and therefore do NOT depend on the degenerate `∂S/∂u` or the surface normal. Because a
freeform surface adapter carries no analytic longitude period, the marcher SHALL re-seed the
far-side `(u,v)` for a freeform pole from the CONTINUED 3D TANGENT — keeping the near-pole LATITUDE
`v` fixed (as the analytic reflect does) and recovering the far LONGITUDE `u` by a point-only search
for the `u` at that fixed latitude whose surface point is nearest the continued world point — rather
than the analytic `u_in + π` meridian jump used for the sphere pole; when the surface DOES carry an
analytic longitude period (the sphere pole) the marcher SHALL keep the existing meridian jump
unchanged. The marcher
SHALL resume the normal `(u,v)` march once `‖dU‖` recovers on both surfaces, and SHALL report the
freeform pole it steps across and verifies in the existing chart-singularities-crossed count (a
per-`TraceSet` count and a per-`WLine` count), NOT in `nearTangentGaps`. Outside a detected freeform
pole the analytic sphere-pole and cone-apex crossings, the transversal S3 trace, the S4-c
crossable-graze crossing, and the S4-d branch-point trace SHALL be unchanged. `src/native/**` SHALL
NOT link OCCT; no `cc_*` entry point, signature, or POD struct SHALL be added or changed; the
marching entry points SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the freeform degenerate-pole curve is now crossed

- GIVEN a B-spline surface whose top U-row of control points is COLLAPSED to a single 3D tip point
  (a spline cone-tip, so `‖∂S/∂u‖` collapses to zero at that `v` edge while the tip point is
  well-defined) and a plane through the tip, whose intersection is a curve passing through the tip,
  forced through marching with a hand seed on the curve away from the tip, whose S3 trace today
  truncates at the tip (a boundary exit / one-sided arc that never reaches the far side of the tip),
  with `CYBERCAD_HAS_NUMSCI` built and chart-singularity handling enabled
- WHEN the S4-e marcher traces the pair
- THEN it SHALL DETECT the chart collapse at the tip, STEP across it with the point-based corrector,
  re-seed the far-side `(u,v)` from the continued 3D tangent, and RESUME the march on the far side of
  the tip — reporting at least one chart singularity crossed and yielding `nearTangentGaps == 0` for
  the traced curve
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`

#### Scenario: the analytic sphere-pole and cone-apex crossings are unchanged

- GIVEN the unit-sphere ∩ plane great circle (which the S4-e analytic slice traces as a full closed
  loop crossing both poles) and the double-cone ∩ plane apex-crossing line (which the S4-e analytic
  slice traces across the apex spanning both nappes)
- WHEN the S4-e marcher traces them with chart-singularity handling enabled
- THEN the sphere-pole great circle SHALL still be fully traced (at least two chart singularities
  crossed, closed status) and the cone-apex line SHALL still be crossed (at least one chart
  singularity crossed) — each identical to the analytic S4-e result, because the freeform-pole
  re-seed engages ONLY when the surface carries no analytic longitude period

#### Scenario: the transversal march, the S4-c graze, and the S4-d branch trace are unchanged

- GIVEN a transversal surface pair whose intersection never reaches a chart singularity, a surface
  pair whose intersection is a `NearTangentTransversal` single-branch graze the S4-c slice marches
  through, and the Steinmetz bicylinder whose intersection self-crosses at two branch points the
  S4-d slice localizes and routes
- WHEN the S4-e marcher traces them
- THEN the transversal traced curve SHALL be identical to the S3 result (same nodes, same status,
  same counts), the graze SHALL still be MARCHED THROUGH by the S4-c crossing, and the Steinmetz
  SHALL still be assembled by the S4-d branch machinery (`branchPoints == 2`) — each with zero chart
  singularities crossed, because the chart machinery SHALL engage ONLY at a detected single-surface
  chart collapse

### Requirement: Freeform-pole honesty and curve-cusp decline (S4-e general)

The S4-e marcher SHALL cross a freeform parametric pole only when the crossing is robustly verified,
and SHALL DECLINE the curve cusp without adding an unreachable mechanism. A freeform-pole crossing
SHALL be emitted only if every node across the singular band verifies on BOTH surfaces within
`onSurfTol`, the far side makes real progress off the tip, and the far-side node re-seeded from the
continued 3D tangent verifies on both surfaces. When a freeform pole's far-side node cannot be
re-projected onto both surfaces within `onSurfTol` (an asymmetric freeform pole whose continued-
tangent re-seed misses), when the crossing makes no real far-side progress, or when the point-based
cut cannot resolve the pole, the marcher SHALL DISCARD the crossing arc, STOP, record the typed stop
reason, count the region in `nearTangentGaps`, and DEFER it (→ OCCT) — reporting the measured gap.
For a CURVE CUSP (the intersection curve's own velocity collapsing to zero), the marcher SHALL NOT
introduce a standalone single-surface-chart cusp witness or corrector: because a cusp of the
intersection curve requires the pair transversality sine to collapse (a cusp with regular charts and
a healthy pair sine is impossible by the implicit function theorem), a curve cusp coincides with the
pair-tangency regime and SHALL be handled by the existing S4-c graze crossing (when the branch
continues through), the S4-d branch machinery (when the locus self-crosses), or an honest deferral →
OCCT (a genuine tangential endpoint the point-based step cannot continue). The marcher SHALL NEVER
fabricate a point across a singularity and SHALL NEVER weaken a tolerance to force a crossing; a
singularity that cannot be robustly crossed SHALL remain an honestly reported gap.

#### Scenario: an asymmetric freeform pole that cannot be verified defers honestly

- GIVEN a freeform degenerate pole whose far-side `(u,v)`, re-seeded from the continued 3D tangent,
  does NOT re-project onto both surfaces within `onSurfTol` (an asymmetric tip where the continued
  tangent misses the far side), with chart-singularity handling enabled
- WHEN the S4-e marcher attempts to step across it
- THEN it SHALL DISCARD the crossing arc, STOP at the pole, record the typed stop reason, and count
  the region in `nearTangentGaps` (deferred → OCCT), reporting zero chart singularities crossed for
  that region — it SHALL NOT fabricate a point across the pole and SHALL NOT weaken a tolerance to
  force the crossing

#### Scenario: a curve cusp is declined and routed to the pair-tangency path, never fabricated

- GIVEN an intersection whose curve has a genuine CUSP (the curve velocity collapses to zero), which
  necessarily coincides with the pair transversality sine collapsing
- WHEN the S4-e marcher reaches the cusp
- THEN it SHALL NOT apply a standalone single-surface-chart cusp crossing, but SHALL let the existing
  S4-c graze crossing march through it when the branch continues, the S4-d branch machinery route it
  when the locus self-crosses, or the march STOP and DEFER it → OCCT when it is a genuine tangential
  endpoint the point-based step cannot continue — never fabricating a continuation across the cusp

### Requirement: Native general/freeform open-arm branch-point localization, arm routing, and assembly (S4-d general)

The kernel SHALL extend the native, **OCCT-free** branch-point capability in `cybercad::native::ssi`
to a GENERAL / FREEFORM branch point — a FREEFORM (B-spline / NURBS) surface pair whose intersection
LOCUS self-crosses at a point B where the pair transversality sine `‖n_A × n_B‖` reaches its minimum
(≈ 0) and the relative second fundamental form `H = II_A − II_B` is INDEFINITE (a saddle contact) — and,
on a FINITE freeform patch, whose arms radiate OPEN to the patch boundary (branch-to-boundary) rather
than closing between two branch points. When the S3/S4-c marcher reaches such a freeform self-crossing
(detected by the SAME scale-free transversality-sine COLLAPSE and band-minimum witness that captures the
analytic Steinmetz stall — a `branchSignature` hand-off gated on `enableBranchPoints`, independent of
any quadric assumption), the marcher SHALL LOCALIZE B on BOTH surfaces within `onSurfTol` (the existing
sine-minimization along the approach followed by a full re-projection onto both surfaces), ENUMERATE the
outgoing arm directions as the REAL distinct roots of the tangent-cone quadratic of `H` (computed with
the existing central-difference relative second form, which is already accurate on a freeform patch),
ROUTE each enumerated arm (step off B, re-project with the existing normal-free point-based corrector,
verify on both surfaces within `onSurfTol`, then continue the normal march to termination), and ASSEMBLE
the multi-arm freeform curve into branch-point-connected arcs — rather than truncating at the crossing.
To resolve the OPEN-ARM topology, the marcher SHALL reclassify a near-tangent-terminated arc out of the
`nearTangentGaps` count (as a `BranchArc`) when EVERY end of the arc that stalled at a near-tangency sits
on a LOCALIZED branch point and at least one end does — the remaining end being a clean domain-boundary
exit — thereby covering BOTH the Steinmetz branch-to-branch closed network AND the freeform
branch-to-boundary open arm. The marcher SHALL record, per traced arc, WHICH end (front / back) stalled
at a near-tangency, and SHALL NOT reclassify an arc that has a near-tangent end which is NOT on a
localized branch point (a genuine still-open S4 gap). The freeform branch points the marcher localizes
and routes SHALL be reported in the existing per-`TraceSet` `branchPoints` count (with the localized
point and its connected arm ids), and the assembled arcs SHALL be counted in `tracedBranches` /
`openCurves`. Outside a detected branch point the transversal S3 trace, the S4-c crossable graze, and
the S4-e chart crossings SHALL be unchanged, and the reclassification rule SHALL reduce to the existing
both-ends-on-branch behaviour for the analytic Steinmetz network (bit-identical). `src/native/**` SHALL
NOT link OCCT; no `cc_*` entry point, signature, or POD struct SHALL be added or changed; the marching
entry points SHALL remain compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: a freeform-saddle open X-crossing is now fully traced

- GIVEN a FREEFORM pair — a bicubic B-spline SADDLE patch (height `≈ 0.15·(x² − y²)`, `makeBSplineAdapter`)
  TANGENT to a plane placed THROUGH the B-spline saddle point (the surface value at the patch centre,
  ≈ z 0.2449 — NOT z = 0, where the two hyperbola branches are DISJOINT), whose intersection locus is an
  X-shaped self-crossing (two curves crossing at the saddle point B, `H = diag(κ, −κ)` indefinite) with
  four arms radiating OPEN to the patch boundary, whose S3+S4-c trace today stops at the crossing with
  `TraceStatus::NearTangent`, `branchPoints == 0`, `tracedBranches == 0`, and `nearTangentGaps == 1`,
  with `CYBERCAD_HAS_NUMSCI` built and `enableBranchPoints` on
- WHEN the S4-d marcher traces the pair
- THEN it SHALL LOCALIZE B on BOTH surfaces within `onSurfTol` (`branchPoints == 1`), enumerate the
  outgoing arms from the tangent-cone quadratic (two distinct real tangent lines ⇒ the outgoing rays),
  ROUTE each arm with the normal-free point-based corrector, and RECLASSIFY each open arm (one end on
  the localized branch, the other a domain boundary) as a resolved `BranchArc` — yielding
  `nearTangentGaps == 0`, `tracedBranches == 4`, and each traced arc a `BranchArc` with EXACTLY one end
  on the branch point
- AND every emitted curve node SHALL lie on BOTH surfaces within `onSurfTol`, and the localized branch
  node SHALL lie on both surfaces within `onSurfTol` at the saddle point

#### Scenario: the analytic Steinmetz branch trace and the transversal / S4-c / S4-e cases are unchanged

- GIVEN the analytic Steinmetz bicylinder (two equal orthogonal cylinders whose intersection self-crosses
  at two branch points the S4-d slice localizes and routes as a branch-to-branch closed network), a
  transversal surface pair whose intersection never reaches a branch point, a `NearTangentTransversal`
  single-branch graze the S4-c slice marches through, and the sphere-pole / cone-apex / freeform-pole
  chart crossings the S4-e slice steps across
- WHEN the S4-d marcher traces them with the generalised open-arm reclassification in place
- THEN the Steinmetz SHALL still be assembled with `branchPoints == 2` and `nearTangentGaps == 0` (its
  branch-to-branch arcs still reclassify — the generalised rule reduces to the existing
  both-ends-on-branch rule when both ends are branch nodes), the transversal traced curve SHALL be
  identical to the S3 result (same nodes, same status, same counts — branch points OFF leaves the new
  per-end flags unused and the polyline byte-identical), the graze SHALL still be MARCHED THROUGH by the
  S4-c crossing, and the chart singularities SHALL still be crossed by the S4-e machinery — the freeform
  open-arm reclassification SHALL engage ONLY on a near-tangent arc terminating on a detected, localized
  branch point

### Requirement: Freeform branch-point honesty — definite contacts end and unresolved arcs defer (S4-d general)

The S4-d marcher SHALL route arms out of a freeform stall ONLY when it is a genuine transversal branch
point — a self-crossing whose tangent-cone quadratic has TWO DISTINCT REAL roots (an indefinite relative
second form). When the quadratic has NO two distinct real roots — a DEFINITE relative second form (an
isolated `TangentPoint`, e.g. a B-spline bump tangent to a plane) or a DOUBLE root (a cusp) — the marcher
SHALL let the curve END there and SHALL enumerate NO outgoing arm. When the branch point cannot be
localized on both surfaces within `onSurfTol`, when an enumerated arm's first step cannot be re-projected
on both surfaces within `onSurfTol` (an unverifiable freeform arm), when the junction is
higher-multiplicity (three+ tangent lines) or non-isolated, or when a near-tangent-terminated arc has a
stall end that is NOT on a localized branch point (a genuine still-open tangency), the marcher SHALL
STOP / DROP the unverifiable arm / LEAVE the arc a `NearTangent` gap (counted in `nearTangentGaps`) and
DEFER it (→ OCCT) — reporting the measured gap. The marcher SHALL NEVER fabricate an arm or an
intersection point past a freeform branch and SHALL NEVER weaken a tolerance to force a branch; the
open-arm reclassification SHALL NOT turn a definite contact into a spurious branch (a definite form
yields no arms ⇒ no branch node ⇒ nothing to reclassify) and SHALL NOT clear a genuine gap (an arc with
a near-tangent end not on a branch stays a gap). A freeform branch that cannot be robustly resolved SHALL
remain an honestly reported gap. No standalone freeform-branch mechanism beyond the open-arm branch-arc
reclassification (and its two per-end near-tangent flags) SHALL be added (no unreachable dead code); the
relative-second-form finite difference SHALL NOT be altered (its central difference is already accurate
on freeform, so no bias-cancellation is shipped).

#### Scenario: an isolated freeform tangent point ends, never sprouting arms

- GIVEN a FREEFORM DEFINITE contact — a B-spline BUMP patch (`z = 0.15·(x²+y²)`) tangent to a plane
  through its minimum, where the relative second form `H` is definite (both relative normal curvatures
  the same sign, so the tangent-cone quadratic has no two distinct real roots), with `enableBranchPoints`
  on
- WHEN the S4-d marcher encounters the contact
- THEN it SHALL let the curve END at the isolated contact, enumerate NO outgoing arm
  (`branchPoints == 0`, `routedArms == 0`, `curveCount() == 0`, `deferredTangent ≥ 1`), and NOT fabricate
  any intersection-curve arm or point across the tangency — exactly as for the analytic isolated tangent
  point

#### Scenario: an unverifiable, off-branch, or higher-multiplicity freeform arc defers honestly

- GIVEN a FREEFORM stall that is either a suspected branch whose enumerated arm's first step does NOT
  re-project on both surfaces within `onSurfTol`, a branch that cannot be localized on both surfaces
  within `onSurfTol`, a near-tangent-terminated arc whose stall end is NOT within the branch merge radius
  of any localized branch point, a freeform CUSP (double root), or a higher-multiplicity junction
  (three+ tangent lines), with `enableBranchPoints` on
- WHEN the S4-d marcher attempts to resolve it
- THEN it SHALL DROP the unverifiable arm and/or leave the backbone's truncated `NearTangent` WLine and
  count the region in `nearTangentGaps` (deferred → OCCT), reporting the measured gap — it SHALL NOT
  reclassify the arc as a `BranchArc`, SHALL NOT fabricate an arm or a point across the branch, and SHALL
  NOT weaken a tolerance to force the crossing

### Requirement: S4-c near-tangent crossing breadth is extended into the grazing regime via adaptive re-anchoring, with a measured honest floor

The native SSI marching tracer SHALL cross near-tangent GRAZING intersections deeper into the
transversality-sine regime than the shipped fixed-plane crossing corrector, WITHOUT weakening any
tolerance and WITHOUT fabricating any curve, gated behind a default-off `MarchOptions` flag.

The shipped S4-c crossing corrector freezes one reference tangent as both the crossability anchor
and the fixed-plane advance direction for the whole crossing; its robust-crossing floor is a
minimum transversality sine (the norm of the cross product of the two unit surface normals along
the intersection) of approximately 0.17 — below it the two-surface corrector fails to converge
because the turning curve leaves the frozen advance plane.

- **Adaptive crossing re-anchoring (S3/S4-c, additive, default-off).** A `MarchOptions` flag
  (`adaptiveCrossReanchor`, with a `reanchorBlend` weight) SHALL, when set, let the crossing
  corrector re-anchor its fixed-plane advance direction toward the LOCAL intersection tangent
  (the continuity-oriented normalized cross product of the two surface normals) as it steps, so the
  advance plane follows the curve's turn through a tighter graze. On this path per-step progress
  SHALL be measured along the actual step direction (bounded by an anti-orbit total-arc cap so a
  non-traversing orbit still terminates and defers), and hand-back to the normal transversal march
  SHALL occur once transversality recovers above the band-enter threshold with a two-consecutive-
  node stability requirement. This flag SHALL DEFAULT OFF, so the crossing corrector, its per-node
  crossability guard, its step control, and its hand-back threshold are byte-identical to the
  shipped S4-c crossing for every already-passing case.

- **Honesty anchors preserved.** With the flag on, the crossability decision (the band-minimum
  sine floor, the steep-sine-collapse witness, and the per-step ≥60° branch-flip guard) SHALL
  remain, every crossing node SHALL be verified on BOTH surfaces within the SAME on-surface
  tolerance (never fabricated), and a genuine tangency or branch point (transversality sine → 0)
  SHALL still defer. No closed form SHALL be added; no tolerance SHALL be widened.

- **Measured breadth extension and honest floor.** With re-anchoring on, the robust-crossing floor
  SHALL drop from a minimum transversality sine of approximately 0.17 to approximately 0.14: a
  grazing pose whose sine dips to ≈ 0.14 (which the shipped corrector declines) SHALL be traced to
  a full closed loop with every node on both surfaces within tolerance. A grazing pose whose sine
  dips below approximately 0.12 SHALL remain an HONEST DECLINE even with re-anchoring on (the
  near-tangent band is then too wide to recover to a transversal stretch within budget) — reported
  as a near-tangent gap, never forced to pass.

The behaviour SHALL be verified by BOTH gates of the two-gate model:
- **Gate A (host, OCCT-free)** — self-consistency + closure: for the newly-crossable grazing pose,
  the default path (flag off) defers with a near-tangent gap and no curve, while the re-anchoring
  path (flag on) traces ONE closed loop with `nearTangentGaps == 0`, `nearTangentCrossed ≥ 1`,
  every node on both surfaces within tolerance, and an arc length within a step-bounded window of
  the tolerance-below-dip ground truth; and for the pose below the extended floor, the re-anchoring
  path still declines (no crossing, no closed loop, a reported near-tangent gap) while a
  ground-truth loop still exists.
- **Gate B (sim, native-vs-OCCT via `GeomAPI_IntSS`)** — the re-anchoring crossing SHALL trace a
  single closed loop whose densely-sampled nodes all lie on the OCCT intersection locus AND on both
  surfaces within tolerance (the crossed curve IS the true intersection, not a fabricated path),
  the default path SHALL still decline the same pose, and below the extended floor the native
  marcher SHALL decline while OCCT reports a locus (the honest, measured boundary).

#### Scenario: a tighter graze the shipped corrector declines is crossed with adaptive re-anchoring
- GIVEN an offset cylinder grazing a sphere posed so the intersection is a single closed loop whose
  minimum transversality sine dips to ≈ 0.14 (below the ≈ 0.17 shipped fixed-plane floor)
- WHEN the crossing is attempted with `adaptiveCrossReanchor` OFF (the default)
- THEN the tracer SHALL HONESTLY DEFER — `nearTangentGaps ≥ 1`, `nearTangentCrossed == 0`, no closed
  loop
- AND WHEN the crossing is attempted with `adaptiveCrossReanchor` ON
- THEN the tracer SHALL trace ONE `Closed` WLine with `nearTangentGaps == 0`, `nearTangentCrossed ≥ 1`,
  every node on both surfaces within tolerance, and an arc length within a step-bounded window of the
  tolerance-below-dip ground truth
- AND against `GeomAPI_IntSS` every densely-sampled node SHALL lie on the OCCT locus and on both
  surfaces within tolerance

#### Scenario: below the extended floor the marcher honestly declines while OCCT reports
- GIVEN an offset cylinder grazing a sphere posed so the minimum transversality sine dips below
  ≈ 0.12 (below the extended re-anchoring floor), with a ground-truth loop that still exists when
  traced with the tolerance below the dip
- WHEN the crossing is attempted with `adaptiveCrossReanchor` ON
- THEN the tracer SHALL still HONESTLY DECLINE — `nearTangentCrossed == 0`, no closed loop, a
  reported near-tangent gap — never fabricating a curve across the knife-edge
- AND `GeomAPI_IntSS` SHALL report a locus for the same pose (the honest, measured boundary between
  native and OCCT)

#### Scenario: the re-anchor flag defaults off leave every prior case unchanged
- GIVEN any already-passing SSI case (transversal march, shipped S4-c graze, S4-d branch, S4-e
  chart singularity, or a genuine tangency that must defer)
- WHEN it is traced with `adaptiveCrossReanchor` at its default (off)
- THEN the crossing corrector, its per-node crossability guard, its step control, and its hand-back
  threshold SHALL be byte-identical to the shipped S4-c behaviour, so the result is unchanged

### Requirement: The S4-c re-anchored crossing orients by continuity, so a graze whose tangent turns past 90° is traversed rather than trapped

The native SSI marching tracer SHALL resolve the ORIENTATION of the re-anchored advance direction
against the previously accepted step direction rather than against the frozen band-entry tangent,
gated behind a default-off `MarchOptions` flag, so that a crossable graze whose intersection
tangent accumulates a turn of 90° or more across the near-tangent band is traversed instead of
being trapped in a non-advancing cycle.

The shipped re-anchor path resolves BOTH the local tangent's sign AND its adoption gate against
the frozen band-entry tangent. Both are half-spaces of a single stale direction and both degenerate
at the same point: once the intersection curve's tangent has turned 90° from that direction, the
sign test inverts the true forward tangent into the backward one. The march then retreats, re-enters
the sub-90° half-space, and advances again — a self-sustaining cycle that consumes the whole node
budget without net transport. The resulting decline is honestly reported, but it is a TRAP rather
than a geometric limit: the poses are crossable.

- **Incremental orientation (S4-c, additive, default-off).** A `MarchOptions` flag
  (`reanchorIncrementalOrientation`) SHALL, when set, re-reference BOTH the sign test and the
  adoption gate of the re-anchored tangent to the previously accepted step direction, seeded to the
  frozen band-entry tangent for the first step. Orientation by continuity is monotone across an
  arbitrarily large accumulated turn. The flag SHALL be consulted ONLY when the adaptive re-anchor
  path is itself enabled, and SHALL DEFAULT OFF, so every already-passing case is byte-identical.

- **Both tests SHALL move together.** Re-referencing only the sign test SHALL NOT be considered
  conformant: the adoption gate then rejects the local tangent, the advance direction falls back to
  the frozen tangent, the step lands perpendicular to the curve, and the step size collapses to the
  floor — relocating the decline rather than removing it.

- **Net-transport termination guard.** On this path the tracer SHALL bound the ratio of arc spent to
  net displacement from the band entry, and discard-and-defer when it is exceeded. The existing
  per-step advance test cannot detect a non-advancing cycle, because the corrector pins the measured
  advance to the requested step along the very direction the step was taken in; and the existing
  anti-orbit arc cap is derived from the step budget, so it scales with the budget and cannot bind.
  This guard is TERMINATION SAFETY only: it converts a residual cycle into an immediate honest
  defer, and SHALL NOT be able to cause a curve to be emitted.

- **Honesty anchors preserved.** The frozen band-entry tangent SHALL remain the anchor for the
  crossability decision — the band-minimum sine floor, the steep-sine-collapse witness, and the
  per-step branch-flip guard are UNCHANGED. Every node SHALL still be verified on BOTH surfaces at
  the SAME on-surface tolerance. No tolerance SHALL be widened and no point SHALL be fabricated.

- **The floor SHALL rest on a declared tolerance.** With the trap removed, the limiting condition
  SHALL be the configured minimum crossing sine, reached at the band-minimum gate with ZERO crossing
  nodes emitted — a principled refusal, distinguishable from a budget-exhaustion decline that emits
  and discards a full budget of non-advancing nodes.

#### Scenario: A wide-band graze whose tangent turns past 90° is crossed

- **GIVEN** a unit sphere and a cylinder of radius 0.4 offset along +x by 0.593, 0.595, or 0.597,
  whose intersection loop has a minimum transversality sine of approximately 0.118, 0.100, or 0.077
- **AND** the adaptive re-anchor path enabled
- **WHEN** the intersection is traced with incremental orientation OFF
- **THEN** the tracer declines, reporting a near-tangent gap and no closed curve
- **WHEN** the same trace is run with incremental orientation ON
- **THEN** the tracer SHALL produce exactly ONE closed loop with no near-tangent gap
- **AND** every corrected node SHALL lie on BOTH surfaces within the on-surface tolerance
- **AND** the traced length SHALL be within a step-bounded window of a ground-truth trace taken
  with a tolerance below the dip

#### Scenario: Below the minimum crossing sine the tracer still declines

- **GIVEN** the same family offset by 0.5975 or 0.598, whose band-minimum sine falls below the
  configured minimum crossing sine
- **WHEN** the intersection is traced with incremental orientation ON
- **THEN** the tracer SHALL decline, reporting a near-tangent gap and no closed curve
- **AND** SHALL emit no crossing nodes at all

#### Scenario: A genuine tangency still defers

- **GIVEN** the same family offset so the surfaces are tangent
- **WHEN** the intersection is traced with incremental orientation ON
- **THEN** the tracer SHALL NOT report a crossing and SHALL NOT close a loop

#### Scenario: The flag defaults off and is inert without the re-anchor path

- **GIVEN** default march options
- **THEN** incremental orientation SHALL be off
- **AND** a pose that the shipped re-anchor path declines SHALL still decline
- **WHEN** incremental orientation is set WITHOUT the adaptive re-anchor path
- **THEN** the trace SHALL be identical to the plain frozen-tangent path

### Requirement: The densify refit SHALL stay clear of interpolation and SHALL be judged between nodes

The native SSI marching tracer's convenience-curve densify-and-refit SHALL choose a pole count that
remains a fraction of the node count, and SHALL judge a candidate refit at parameters BETWEEN the
nodes as well as at the nodes, so that a fit which interpolates its data while oscillating between
data points is never accepted.

At a pole count equal to the node count the least-squares system is square and interpolating, and a
clamped-uniform knot vector over a chord-length parametrization degenerates: the curve passes
through every node exactly while deviating from the true curve between them by orders of magnitude
more than the fit it replaced. The at-node error metric is structurally blind to this, because it is
sampled at precisely the parameters such a fit interpolates.

- **Conditioning guard.** The refit pole target SHALL be bounded by a FRACTION of the node count,
  not by the node count itself, keeping the ratio clear of the measured degeneration onset. The
  bound SHALL be RELATIVE: a flat pole ceiling low enough to avoid degeneration on a moderate-node
  loop starves a dense high-curvature loop that legitimately requires the full pole cap.

- **Between-node accept test.** A candidate refit SHALL be accepted only if it does not worsen the
  deviation measured at node-midpoint parameters, in addition to the existing at-node test. The
  polyline remains the ground truth for that comparison.

- **Trigger proportionate to the verification budget.** The densify trigger SHALL be set tighter
  than the on-curve tolerance the native-vs-OCCT parity verification enforces, so a loop whose fit
  misses the true locus by more than that budget actually trips the refit. It SHALL remain loose
  enough that ordinary well-resolved loops do not refit. The multiplier SHALL be exposed as a
  caller option so the previous behaviour can be restored; changing it SHALL affect cost and curve
  quality only, and SHALL NOT move a node, widen a tolerance, or alter the polyline.

#### Scenario: A moderate-node high-curvature loop refits without interpolating

- **GIVEN** a near-tangent intersection loop of at most a few hundred nodes whose initial fit
  exceeds the densify trigger
- **WHEN** the curve is fitted
- **THEN** the refit SHALL fire, raising the pole count above the initial value
- **AND** the resulting pole count SHALL remain strictly below the node count
- **AND** the fitted curve sampled BETWEEN nodes SHALL lie on both surfaces within the on-curve
  budget the parity verification enforces

#### Scenario: A dense high-curvature loop still receives the full pole budget

- **GIVEN** an intersection loop with many hundreds of nodes that genuinely requires the pole cap
- **WHEN** the curve is fitted
- **THEN** the refit SHALL still reach the full pole cap, unchanged by the conditioning guard

#### Scenario: Ordinary loops are unaffected

- **GIVEN** a well-resolved intersection loop whose initial fit already rides its nodes
- **WHEN** the curve is fitted
- **THEN** no refit SHALL fire and the result SHALL be unchanged

### Requirement: Subdivision SHALL prune a box pair proven separated along an oriented direction

The native SSI seeder's recursive subdivision SHALL be able to discard a candidate box pair when the
two surface pieces are PROVEN to come no closer than the subdivision gap, using a witness direction
that need not be axis-aligned.

The existing disjointness test is axis-aligned. On a near-parallel pair separated along no
coordinate axis it can never fire, so the descent enumerates the entire 4D box product and hands
every leaf to the region refiner even though the pair does not intersect anywhere.

- **The witness SHALL be a containment argument.** By the convex-hull property each surface piece
  lies inside the hull of its exact sub-net over that param sub-box. Projection onto a direction is
  linear, so it maps each hull into a closed interval. Two intervals separated by more than the gap
  therefore PROVE no crossing exists in that box pair. Because a descendant's param boxes are
  contained in its parent's, a parent-level proof discards only crossing-free subtrees.

- **Soundness SHALL NOT depend on the direction chosen.** A direction that fails to separate simply
  leaves the descent unchanged, so the direction is a heuristic for REACH and never for
  correctness. The predicate SHALL normalize the direction itself rather than relying on its caller,
  since a non-unit direction would scale the projections and could overstate the separation. A
  degenerate direction SHALL cause the predicate to refuse.

- **Scope SHALL match what the proof requires and nothing looser.** The predicate SHALL apply only
  where both operands expose an exact single-span non-rational control net. It SHALL NOT require the
  two nets to share a degree, because each hull bounds its own surface independently and no
  correspondence between the nets is involved. Operands outside that scope SHALL take the unchanged
  path with byte-identical results.

- **The predicate SHALL NOT be applied where box locality does not hold.** Region refinement clamps
  into the full parameter domain and is effectively a global solve, so a converged solution is not
  confined to the candidate box that produced it. Applying this test there would discard real seeds.
  The justification recorded in the code SHALL be the containment argument, NOT an argument from
  precedent, because a precedent-shaped justification would equally license that unsound use.

#### Scenario: A near-parallel pair with no axis of separation is pruned

- **GIVEN** two freeform surfaces offset along their common normal by more than the subdivision gap,
  positioned so that their axis-aligned bounds overlap on every coordinate axis at every depth
- **WHEN** the seeder subdivides the pair
- **THEN** the descent SHALL be pruned rather than enumerating the full box product
- **AND** the seeder SHALL report no seeds, because the pair does not intersect

#### Scenario: A transversal crossing is never lost

- **GIVEN** two freeform surfaces that genuinely intersect
- **WHEN** the seeder subdivides the pair with the prune active
- **THEN** the seed count, the branch count and any coincidence verdict SHALL be unchanged from the
  same seeder with the prune inactive
- **AND** only the candidate-region count, which is pure cost, may differ

#### Scenario: An operand outside the proof's scope is unaffected

- **GIVEN** a pair in which either operand is elementary, rational, or a multi-span B-spline
- **WHEN** the seeder subdivides the pair
- **THEN** the prune SHALL NOT fire and the emitted candidate regions SHALL be byte-identical to the
  behaviour without it

#### Scenario: A caller-supplied direction cannot overstate the separation

- **GIVEN** a box pair whose surfaces are closer than the gap along every direction
- **WHEN** the predicate is asked with a direction of large magnitude
- **THEN** it SHALL NOT report a separation, having normalized the direction before comparing


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

The S3 `cybercad::native::ssi` `TraceSet` SHALL be the input contract consumed by the
native S5-c sphere∩sphere common (`src/native/boolean/ssi_boolean.cpp buildLensCommon`):
for a transversal sphere∩sphere pair, the boolean SHALL obtain the `TraceSet` and use the
SINGLE `Closed` `WLine` — its per-node `(u1,v1,u2,v2)` on both spheres (the seam-circle
track used to fan each spherical cap) and its shared 3D nodes (the seam vertices both caps
weld on through the shared `VertexPool`) — to assemble the two-cap lens. The S5-c boolean
SHALL consume the `TraceSet` ONLY when it is fully transversal — `nearTangentGaps == 0`
and the consumed `WLine.status` is `Closed`; a `TraceSet` with `nearTangentGaps > 0`, a
`NearTangent` / `Failed` WLine, or a seam that spans a full sphere (coincident) SHALL be
treated as the honest S4 fallback boundary and SHALL NOT be consumed (the boolean declines
→ OCCT). The tracer SHALL NOT change to serve this consumption — the contract is the
already-shipped S3 output; no `cc_*` entry point, signature, or POD struct SHALL be added
or changed, and the SSI module SHALL remain OCCT-free and compiled under
`CYBERCAD_HAS_NUMSCI` (like the S3 tracer).

#### Scenario: a single closed TraceSet seam is consumed to weld the lens
- GIVEN a transversal sphere∩sphere pair whose S3 `TraceSet` has `nearTangentGaps == 0`
  and exactly one `Closed` WLine
- WHEN the S5-c curved boolean consumes the `TraceSet`
- THEN it SHALL fan each spherical cap from its pole to the WLine's shared 3D seam nodes,
  with every seam node on both spheres ≤ tol, and the two caps SHALL weld along that single
  seam
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a tangent / coincident sphere TraceSet is the S4 boundary, not consumed
- GIVEN a sphere∩sphere pair whose S3 `TraceSet` reports `nearTangentGaps > 0` (tangent
  spheres) or whose single WLine is `NearTangent` / spans a full sphere (coincident)
- WHEN the S5-c curved boolean inspects the `TraceSet`
- THEN it SHALL decline to consume the trace (the honest S4 seam) and the boolean SHALL
  fall back to OCCT, reported — never welding a lens on a truncated or fabricated seam

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


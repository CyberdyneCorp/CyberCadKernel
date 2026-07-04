# native-ssi

Native, OCCT-free surface-surface intersection. This change extends the S1
closed-form module (`openspec/SSI-ROADMAP.md`) with **SSI Stage S2 — subdivision
seeding**: for the freeform (NURBS/Bézier/B-spline) and non-closed-form quadric
pairs that S1's dispatch returns `NotAnalytic` for, find ≥1 seed point on every
distinct **transversal** intersection branch/loop (recursive patch-AABB-overlap
subdivision + `native-numerics` `least_squares` refine + branch dedup), each seed on
both surfaces within tol with its `(u1,v1,u2,v2)`. Branch **recall** is measured vs
OCCT `GeomAPI_IntSS`; near-tangent / coincident / degenerate seeding is deferred to
S4 (reported, not faked); small-loop completeness is a measured recall figure. The
seed set is the input contract for S3 marching. Internal capability: **no `cc_*`
ABI**.

## MODIFIED Requirements

### Requirement: SSI is an internal capability verified against OCCT with no ABI change

SSI SHALL be an **internal** native capability consumed by native booleans/blends
(the S5 on-ramp) and by the S3 marching tracer (which consumes S2 seeds), and SHALL
NOT be exposed through the `cc_*` C ABI: no `cc_*` entry point, signature, or POD
struct SHALL be added or changed by this capability. It SHALL be verified at the
SSI-function level (per `SSI-ROADMAP.md` §Verification model) by two gates: a **host**
gate (no OCCT) and a **sim native-vs-OCCT** gate (`GeomAPI_IntSS` parity /
branch-recall) — the same internal parity discipline used for `native-math` and
`native-topology`. For the S1 closed-form pairs the host gate asserts known conics
with every sampled point on both surfaces; for the S2 seeding pairs the host gate
asserts known transversal branch counts with every seed on both surfaces, and the
sim gate reports the measured **branch recall** vs OCCT.

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

## ADDED Requirements

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

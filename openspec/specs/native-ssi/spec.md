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
reason and an **empty** curve list. NOT-ANALYTIC pairs SHALL be **deferred** to
S2/S3 marching or OCCT and SHALL NEVER be faked with an approximate or fabricated
curve. The dispatch SHALL return NOT-ANALYTIC for at least: general/skew
cylinder∩cylinder (a quartic space curve), general cone∩cone, non-coaxial
cone∩cylinder, torus∩curved (any torus pair other than the supported plane∩torus),
ANY NURBS/Bézier/B-spline/freeform operand, and any **near-tangent / coincident**
configuration where the closed-form branch is numerically unsafe. `analytic ==
false` is a normal outcome (the deferral seam), not an error.

#### Scenario: skew cylinder ∩ cylinder defers, not faked
- GIVEN two cylinders whose axes are neither coaxial nor parallel (a general
  quartic space-curve intersection)
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with reason "out-of-scope pair" and an
  empty curve list — deferring to S2/S3 or OCCT
- AND it SHALL NOT return any approximate or fabricated curve

#### Scenario: any freeform operand defers
- GIVEN a pair where at least one operand is a NURBS / Bézier / B-spline surface
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with the freeform-surface reason and an
  empty curve list, deferring to S2/S3

#### Scenario: non-coaxial quadric pair defers rather than approximating
- GIVEN a sphere∩cylinder (or sphere∩cone, or cylinder∩cone) whose axes are NOT
  coaxial (nor, for cyl∩cyl, parallel)
- WHEN the native SSI dispatch classifies the pair
- THEN it SHALL return `analytic == false` with the non-coaxial-quadric reason,
  deferring the general case to a later SSI stage

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
(the S5 on-ramp) and SHALL NOT be exposed through the `cc_*` C ABI: no `cc_*`
entry point, signature, or POD struct SHALL be added or changed by this capability.
It SHALL be verified at the SSI-function level by two gates (per
`SSI-ROADMAP.md` §Verification model): a **host analytic** gate (no OCCT — known
conics and every sampled point on both surfaces) and a **sim native-vs-OCCT** gate
(`GeomAPI_IntSS` parity on the supported elementary pairs) — the same internal
parity discipline used for `native-math` and `native-topology`.

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


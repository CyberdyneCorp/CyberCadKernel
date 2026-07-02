# native-math Specification

## Purpose
TBD - created by archiving change add-native-math-geometry. Update Purpose after archive.
## Requirements
### Requirement: OCCT-free, host-buildable math library
The native math foundation SHALL live under `src/native/math/` and SHALL include NO
OCCT header in any of its translation units, so that it compiles and unit-tests
with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO simulator.
The library SHALL link no OCCT; OCCT SHALL appear ONLY in the simulator
native-vs-OCCT parity test, never in the library itself. This change SHALL make no
`cc_*` signature or POD struct layout change and SHALL NOT wire the library into
the active engine.

#### Scenario: Library builds on the host without OCCT
- GIVEN the sources under `src/native/math/`
- WHEN they are compiled with `clang++ -std=c++20` with no OCCT and no simulator
- THEN the build SHALL succeed AND no compiled translation unit SHALL include any OCCT header

#### Scenario: No ABI change and no engine wiring
- GIVEN this change applied
- WHEN the public headers and the active engine are inspected
- THEN no `cc_*` signature or POD struct layout SHALL have changed AND the native math library SHALL NOT be reachable through the `cc_*` facade

### Requirement: fp64 value types and transforms
The library SHALL provide fp64 value types `vec3`, `point3`, and `dir3` (a
unit-length direction), and a 4×4 affine `transform`. The `transform` SHALL support
`compose` (composition of two transforms), `invert` (the inverse transform), and
`apply` to a `point3` (including translation), to a free `vec3` (excluding
translation), and to a `dir3` (linear part applied then re-normalized). The
point/vector/direction semantics SHALL match the `gp_Trsf` convention. All
operations SHALL be fp64 and deterministic.

#### Scenario: Transform identity, associativity, and inverse round-trip (host)
- GIVEN transforms built on the host with no OCCT
- WHEN the identity transform is applied and when `A`, `B`, `C` are composed and inverted
- THEN applying the identity SHALL leave a point/vector/direction unchanged, `compose` SHALL be associative within the documented fp64 tolerance, AND `compose(T, invert(T))` SHALL equal the identity within the documented fp64 tolerance

#### Scenario: Point vs vector vs direction application semantics (host)
- GIVEN a transform with a nonzero translation and a rotation
- WHEN it is applied to a `point3`, the same coordinates as a free `vec3`, and as a `dir3`
- THEN the `point3` result SHALL include the translation, the `vec3` result SHALL exclude the translation, AND the `dir3` result SHALL be the rotated direction re-normalized to unit length

#### Scenario: Transforms match gp_Trsf (parity)
- GIVEN the same rotation and translation used to build a native `transform` and an OCCT `gp_Trsf`, on a booted iOS simulator
- WHEN `compose`, `invert`, and `apply` to sampled points, vectors, and directions run on both
- THEN the native results SHALL equal the `gp_Trsf` results within the documented tight fp64 tolerance

### Requirement: Bézier curve evaluation
The library SHALL evaluate a Bézier curve's point and derivatives from its control
polygon via the de Casteljau algorithm (derivatives via the hodograph). Rational
(weighted) Bézier curves SHALL be evaluated in homogeneous coordinates with the
quotient rule for derivatives; a curve with all weights equal to 1 SHALL reduce to
the polynomial case. All evaluation SHALL be fp64 and deterministic.

#### Scenario: Bézier endpoints and derivative match closed form (host)
- GIVEN a Bézier curve with a known control polygon, evaluated on the host with no OCCT
- WHEN it is evaluated at `t = 0`, `t = 1`, and `t = 0.5`
- THEN the point at `t = 0` and `t = 1` SHALL equal the first and last control points, the point at `t = 0.5` SHALL equal the known closed-form value, AND the first derivative SHALL equal the closed-form hodograph value within the documented fp64 tolerance

#### Scenario: Rational Bézier arc lies on the exact circle (host)
- GIVEN a rational quadratic Bézier representing a circular arc
- WHEN it is evaluated across `t`
- THEN every evaluated point SHALL lie on the exact circle within the documented fp64 tolerance

#### Scenario: Bézier evaluation matches the OCCT oracle (parity)
- GIVEN random (weighted) control polygons on a booted iOS simulator
- WHEN the native Bézier point and first derivative are compared against the OCCT Bézier oracle (`PLib`) across sampled `t`
- THEN the native results SHALL equal the oracle within the documented tight fp64 tolerance

### Requirement: B-spline and NURBS curve evaluation
The library SHALL evaluate a B-spline / NURBS curve's point and first derivative
from its degree, knot vector, control points, and (for NURBS) weights, using span
location (FindSpan, A2.1), basis functions and their derivatives (BasisFuns, A2.2),
and the curve-point / curve-derivative combinations (A3.1 / A3.2). Rational NURBS
SHALL be evaluated in homogeneous coordinates with the quotient rule for
derivatives; all weights equal to 1 SHALL reduce to the polynomial B-spline case.
All evaluation SHALL be fp64 and deterministic.

#### Scenario: Known B-spline and NURBS values (host)
- GIVEN a degree-1 B-spline, a known cubic B-spline, and a NURBS unit-circle arc, evaluated on the host with no OCCT
- WHEN each is evaluated across its parameter range
- THEN the degree-1 B-spline SHALL reproduce its control polygon, the cubic B-spline SHALL match the known closed-form point, AND the NURBS arc's points SHALL lie on the exact unit circle with the expected tangent direction, all within the documented fp64 tolerance

#### Scenario: B-spline/NURBS evaluation matches the OCCT oracle (parity)
- GIVEN the same degree, knots, control points, and weights used to build a native curve and an OCCT `BSplCLib` curve, on a booted iOS simulator
- WHEN the native point and first derivative are compared against `BSplCLib` across sampled parameters including values near interior knots and at the endpoints
- THEN the native results SHALL equal the oracle within the documented tight fp64 tolerance

### Requirement: Tensor-product surface evaluation with partials and normal
The library SHALL evaluate a tensor-product Bézier / B-spline / NURBS surface's
point, first partial derivatives `dS/du` and `dS/dv`, and unit normal
`normalize(dS/du × dS/dv)`, using the surface-point / surface-derivative
combinations (SurfacePoint A3.5 / SurfaceDerivs A3.6) over the two directions (and
de Casteljau for the Bézier case). Rational surfaces SHALL be evaluated in
homogeneous coordinates with the quotient rule. The normal orientation SHALL follow
a documented convention, and the degenerate case (a pole or parallel partials)
SHALL be handled and documented. All evaluation SHALL be fp64 and deterministic.

#### Scenario: Bilinear and Bézier patch known values (host)
- GIVEN a bilinear (degree 1×1) patch and a known Bézier patch, evaluated on the host with no OCCT
- WHEN the bilinear patch is evaluated at its corners and the Bézier patch at a known `(u,v)`
- THEN the bilinear patch SHALL reproduce its four corner points with the expected flat normal, AND the Bézier patch point SHALL equal the known closed-form value, within the documented fp64 tolerance

#### Scenario: Cylindrical NURBS patch has a radial normal (host)
- GIVEN a NURBS surface patch representing part of a cylinder
- WHEN its normal is evaluated at sampled `(u,v)`
- THEN the normal SHALL be radial (perpendicular to the axis, pointing outward per the documented convention) within the documented fp64 tolerance

#### Scenario: Surface point, partials, and normal match the OCCT oracle (parity)
- GIVEN the same knots, degrees, control net, and weights used to build a native surface and an OCCT `BSplSLib` surface, on a booted iOS simulator
- WHEN the native point, `dS/du`, `dS/dv`, and normal are compared against `BSplSLib` across a sampled `(u,v)` grid
- THEN the native point and partials SHALL equal the oracle within the documented tight fp64 tolerance AND the native normal SHALL match the oracle normal up to the documented orientation convention

### Requirement: Elementary surface point and normal
The library SHALL evaluate the point and unit normal of the elementary surfaces
plane, cylinder, cone, and sphere from their closed-form parametrizations. Parameter
ranges (angular parameters, the axis coordinate, the cone half-angle) and the
outward-normal orientation SHALL follow a documented convention matched to `ElSLib`.
All evaluation SHALL be fp64 and deterministic.

#### Scenario: Elementary-surface normals match closed form (host)
- GIVEN a unit sphere, a cylinder, a cone, and a plane, evaluated on the host with no OCCT
- WHEN each is evaluated at sampled `(u,v)`
- THEN the sphere normal SHALL equal the radial direction, the cylinder normal SHALL be radial and independent of the axis coordinate, the cone normal SHALL tilt by its half-angle, AND the plane normal SHALL be constant, all within the documented fp64 tolerance

#### Scenario: Elementary surfaces match the OCCT oracle (parity)
- GIVEN plane, cylinder, cone, and sphere built with the same parameters natively and via OCCT `ElSLib`, on a booted iOS simulator
- WHEN the native point and normal are compared against `ElSLib` (`Value` / `D1` / normal) across sampled `(u,v)`
- THEN the native results SHALL equal the oracle within the documented tight fp64 tolerance

### Requirement: Deterministic fp64 evaluation
All native math evaluation SHALL be fp64 with a fixed evaluation order (the de Boor
and de Casteljau reductions and the basis accumulation), so that repeated evaluation
of the same input produces bit-identical results.

#### Scenario: Repeated evaluation is bit-identical (host)
- GIVEN any curve, surface, transform, or elementary-surface evaluation and a fixed input
- WHEN the same evaluation is run twice on the host
- THEN the two results SHALL be bit-identical


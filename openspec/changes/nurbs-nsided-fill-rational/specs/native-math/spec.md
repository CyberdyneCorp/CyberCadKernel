# native-math

## ADDED Requirements

### Requirement: Rational N-sided boundary closed-loop consistency verification

The N-sided module SHALL provide, ADDITIVELY to the non-rational `verifyNSidedBoundary`
(`src/native/math/bspline_nsided.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `verifyNSidedBoundaryRational` that verifies a set of N boundary
B-spline / NURBS curves forms a CONSISTENT closed N-gon and ACCEPTS RATIONAL edges. The routine SHALL
require `N ≥ 3`; every edge to be well-formed (clamped flat knot vector, degree ≥ 1, ≥ 2 control
points) and, if rational, to carry exactly one STRICTLY-POSITIVE weight per pole (a zero, negative, or
count-mismatched weight SHALL be declined); non-rational edges (empty `weights`) SHALL be accepted and
treated as weight 1. The loop SHALL close on consecutive shared corners measured with the RATIONAL
evaluator: `edges[i](1)` SHALL equal `edges[(i+1) mod N](0)` to within a caller-supplied tolerance. It
SHALL report the maximum corner mismatch. On any violation the routine SHALL return `ok = false` with a
human-readable reason, never a silently-broken loop and never a crash. The existing non-rational
`verifyNSidedBoundary` SHALL be byte-unchanged.

#### Scenario: A consistent rational arc loop verifies with a machine-precision corner error

- GIVEN N ≥ 3 boundary curves whose consecutive corners coincide, where one or more edges are rational (for example exact circular arcs forming a rounded frame)
- WHEN the boundary is verified with the rational routine
- THEN the routine SHALL return `ok = true` with `maxCornerError` at machine precision and `n` equal to the number of edges

#### Scenario: A non-closed, non-positive-weight, N<3, or malformed boundary declines honestly

- GIVEN a boundary with a displaced corner (open loop), or a rational edge carrying a zero or negative weight, or fewer than three edges, or a malformed edge (degree 0 or a bad knot vector)
- WHEN the boundary is verified with the rational routine
- THEN the routine SHALL return `ok = false` with a reason and (for the displaced case) a large `maxCornerError`, without crashing

### Requirement: Rational N-sided fill by homogeneous midpoint subdivision

The module SHALL construct, ADDITIVELY via `fillNSidedRational` returning `NSidedFillRationalResult`,
from a consistent closed N-gon boundary whose edges MAY be rational, a set of N tensor-product Coons
sub-patches whose UNION reproduces all N input boundary curves — performing the ENTIRE midpoint
subdivision and per-corner Coons boolean sum in HOMOGENEOUS `(w·x, w·y, w·z, w)` space and
de-homogenizing only at the end, so a RATIONAL boundary curve (an exact circular arc) is reproduced
EXACTLY rather than approximated polynomially. It SHALL first verify the boundary with
`verifyNSidedBoundaryRational` (declining honestly on a non-closed / malformed / non-positive-weight
boundary). It SHALL compute the corners `V[i] = edges[i](0)` and midpoints `M[i] = edges[i](0.5)` with
the rational evaluator, the midpoints via the Layer-1 exact `splitCurve` (which preserves the rational
half-arc exactly), and the centroid `C = mean(V[i])`. For each corner it SHALL build the quad from the
two rational boundary half-edges and two straight interior spokes to `C`, where the spokes carry MATCHED
corner weights (the arc midpoint weight at `M[i]`, weight 1 at `C`) so the four homogeneous corners are
consistent; and fill it by the homogeneous Coons boolean sum `L_u ⊕ L_v ⊖ B` on the R⁴ nets, projecting
once. A non-positive projected weight, a degenerate spoke (a midpoint coinciding with the centroid), or
a sub-quad the homogeneous Coons rejects SHALL return `ok = false` with a reason, without crashing. When
every projected weight is 1 the sub-patch `weights` vector SHALL be dropped (non-rational reduction).

#### Scenario: A rational circular-arc boundary is reproduced EXACTLY

- GIVEN a closed N-gon boundary whose edges are exact rational circular arcs (for example a rounded frame of four quarter-circle NURBS arcs)
- WHEN the rational N-sided fill is built
- THEN the result SHALL be N (possibly rational) sub-patches whose outer iso-edges reproduce each rational boundary arc pointwise on a dense sample to within 1e-12 (achieved ~1e-15) — an EXACT rational reproduction, NOT a polynomial approximation — and at least one sub-patch SHALL carry non-unit weights

#### Scenario: A weights-all-1 boundary reduces to the non-rational fillNSided

- GIVEN a non-rational boundary (empty weights, or all weights explicitly 1)
- WHEN both `fillNSided` and `fillNSidedRational` are built
- THEN the rational fill's sub-patches SHALL reproduce the `fillNSided` sub-patches pointwise to within 1e-12 (achieved 0), and each rational sub-patch's `weights` vector SHALL be empty (non-rational reduction)

#### Scenario: A planar rational boundary yields points on the plane

- GIVEN a boundary whose rational edges are coplanar (for example rational arcs in the `z=0` plane)
- WHEN the rational N-sided fill is built
- THEN every point of every sub-patch SHALL lie on that plane to within 1e-10 (achieved 0)

#### Scenario: Non-closed and non-positive-weight boundaries decline honestly

- GIVEN a boundary with a displaced corner (open loop), or a rational edge with a non-positive weight, or fewer than three edges, or a malformed edge
- WHEN the rational N-sided fill is requested
- THEN the routine SHALL return `ok = false` with a reason, without crashing, and SHALL NOT emit patches that silently miss their own boundary curves, and SHALL NOT widen the tolerance to pass

### Requirement: Rational N-sided fill is additive and does not fake continuity

The rational N-sided fill SHALL be purely ADDITIVE: the existing non-rational `verifyNSidedBoundary` /
`fillNSided` behavior and the `bspline_coons` module SHALL be unchanged, and no `cc_*` ABI SHALL be
added or altered. The rational fill SHALL match the boundary POSITION exactly and be C0 at the interior
spokes and centroid; it SHALL NOT claim tangent (G1) or curvature (G2) continuity across the interior
seams, nor an energy-minimized curved interior. Rational-G1 Gregory / plate blends and curved-N-sided
interior fairing remain documented residuals.

#### Scenario: The rational fill neither changes the non-rational path nor fakes G1/G2

- GIVEN the additive rational N-sided fill
- WHEN the non-rational `fillNSided` is exercised and the rational result is inspected
- THEN `fillNSided` SHALL behave exactly as before, no `cc_*` ABI SHALL change, and the rational fill SHALL NOT claim tangent/curvature continuity across the interior spokes nor an energy-minimized curved interior

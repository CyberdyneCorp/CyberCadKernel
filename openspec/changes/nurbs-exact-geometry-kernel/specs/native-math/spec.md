# native-math

## ADDED Requirements

### Requirement: Exact NURBS knot insertion and refinement

The native math library SHALL provide, in an OCCT-free, substrate-free module
(`src/native/math/bspline_ops.{h,cpp}`, always-on, no `CYBERCAD_HAS_NUMSCI` guard), knot
insertion for B-spline and NURBS curves and tensor-product surfaces that **preserves the
represented geometry exactly**. Inserting a knot value `u` with multiplicity `r` (Boehm,
*The NURBS Book* A5.1 for curves, applied per row/column for surfaces) SHALL produce a new
control net, weights, and knot vector that evaluate to the **same point** as the original at
every parameter. Refinement (inserting an entire sorted new-knot vector at once, A5.4) SHALL
be equivalent to the corresponding sequence of single insertions. Rational inputs SHALL be
handled by homogeneous lifting; non-rational inputs (empty weights) SHALL be handled directly.
The knot-vector length invariant `knots.size() == poles.size() + degree + 1` (per direction
for surfaces) SHALL hold on the result.

#### Scenario: Knot insertion preserves the curve pointwise

- GIVEN a B-spline or NURBS curve `C` and a knot value `u` in its domain
- WHEN `u` is inserted with any multiplicity `r` up to `degree - existingMultiplicity`
- THEN the result SHALL have `r` more poles and `r` more knots, satisfy the length invariant, and evaluate to `C(t)` at every sampled `t` to within an exact-arithmetic tolerance (~1e-12 relative to the curve's magnitude)

#### Scenario: Refinement equals repeated single insertion

- GIVEN a curve or surface and a sorted vector of new knot values
- WHEN it is refined by inserting the whole vector at once
- THEN the resulting net, weights, and knots SHALL match those produced by inserting the same values one at a time, and SHALL evaluate to the original geometry pointwise

### Requirement: Exact NURBS degree elevation

The module SHALL raise the degree of a curve or surface (in a chosen parametric direction for
surfaces) by any positive `t` (A5.9 curve / A5.10 surface) such that the elevated
representation evaluates to the **same** curve/surface at every parameter. The result's degree
SHALL be exactly `degree + t`, and interior-knot multiplicities SHALL be raised consistently
so the length invariant holds.

#### Scenario: Degree elevation preserves the geometry pointwise

- GIVEN a B-spline or NURBS curve or surface and a positive integer `t`
- WHEN its degree is elevated by `t`
- THEN the result SHALL have degree raised by exactly `t`, satisfy the knot-length invariant, and evaluate to the original geometry at every sampled parameter to ~1e-12

### Requirement: Tolerance-bounded knot removal and degree reduction with honest reporting

The module SHALL provide knot removal (A5.8) and degree reduction by one (A5.11) for curves
and surfaces. Each SHALL report the achieved result honestly: knot removal SHALL return how
many of the requested removals were achievable within the supplied tolerance and the resulting
maximum deviation; degree reduction SHALL return whether the geometry was reducible within
tolerance and the resulting maximum deviation. Neither SHALL widen its tolerance to claim
success: a knot that cannot be removed within tolerance SHALL NOT be removed, and an
irreducible curve SHALL report `ok = false` with the true error bound rather than returning a
geometry that differs beyond tolerance while claiming exactness.

#### Scenario: Insert-then-remove is an identity

- GIVEN a curve `C` into which a knot `u` has been inserted `r` times
- WHEN `u` is removed up to `r` times with an exact-arithmetic tolerance
- THEN all `r` removals SHALL succeed, and the recovered curve SHALL equal `C` (same degree, poles, weights, knots to within the tolerance)

#### Scenario: An irreducible curve is reported honestly, not falsely reduced

- GIVEN a curve whose degree cannot be reduced within the supplied tolerance
- WHEN degree reduction is attempted
- THEN the operation SHALL return `ok = false` with the true maximum deviation, and SHALL NOT return a lower-degree curve presented as an exact match

#### Scenario: A genuinely reducible curve is recovered exactly

- GIVEN a curve produced by elevating a known lower-degree curve by one
- WHEN degree reduction is applied
- THEN it SHALL return `ok = true` and recover the original lower-degree curve to within the exact-arithmetic tolerance

### Requirement: NURBS splitting, Bézier decomposition, and reparametrization

The module SHALL split a curve at a parameter into two curves, and a surface along a U or V
isoparameter into two patches, via knot insertion to full multiplicity, such that the pieces
**reconstruct the original geometry** on their sub-domains and join continuously at the split.
It SHALL decompose a curve into its constituent Bézier segments (A5.6), each of which
re-evaluates to the source curve on its span. It SHALL reparametrize a curve's knot domain
affinely to a new `[a, b]` interval without changing the poles or weights, so the geometry is
unchanged up to the parameter remap.

#### Scenario: Split pieces reconstruct the original curve

- GIVEN a curve `C` and a split parameter `u` strictly inside its domain
- WHEN `C` is split at `u`
- THEN the two resulting curves SHALL evaluate to `C` on their respective sub-domains and share the point `C(u)` at the join, to within ~1e-12

#### Scenario: Bézier decomposition segments re-evaluate to the source

- GIVEN a B-spline or NURBS curve with interior knots
- WHEN it is decomposed into Bézier segments
- THEN the number of segments SHALL equal the number of distinct knot spans, and each segment SHALL evaluate to the source curve on its corresponding span to within ~1e-12

### Requirement: Rational evaluation robustness under the construction algorithms

The existing NURBS evaluators SHALL remain correct under the degenerate configurations the
construction algorithms produce and consume — interior knots at full multiplicity,
clamped and unclamped ends, endpoint parameters, and high degree — and SHALL guard against
non-positive projected weights rather than dividing by a non-positive denominator. This
hardening SHALL be behaviour-preserving: every parameter/geometry case that evaluates
correctly today SHALL continue to produce a byte-identical result.

#### Scenario: Full-multiplicity interior knots evaluate correctly

- GIVEN a curve or surface whose interior knot has multiplicity equal to the degree (a Bézier-segment boundary produced by splitting or decomposition)
- WHEN it is evaluated at and around that knot
- THEN the evaluation SHALL be finite, continuous, and equal to the un-split geometry at those parameters

#### Scenario: Hardening does not change any currently-correct evaluation

- GIVEN the existing evaluation regression cases
- WHEN the hardened evaluators are run
- THEN every result SHALL be byte-identical to the pre-change output

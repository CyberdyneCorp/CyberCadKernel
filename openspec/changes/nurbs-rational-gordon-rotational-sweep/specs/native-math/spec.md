# native-math

## ADDED Requirements

### Requirement: Rational curve-network verification

The native math library SHALL verify that a WEIGHTED (rational) curve network forms a consistent
grid in the OCCT-free `CYBERCAD_HAS_NUMSCI`-gated Gordon module
(`verifyRationalNetwork(network, tol)` in `src/native/math/bspline_gordon.{h,cpp}`). Every u/v
curve MUST be rational (non-empty `weights`, one weight per pole, every weight strictly positive)
and well-formed (degree ≥ 1, valid flat knot vector). The routine SHALL require at least two
curves in each direction, param arrays matching the family sizes, and strictly-increasing station
parameters, and SHALL check grid consistency with the RATIONAL evaluator: `C_k(uParams[l])` MUST
equal `D_l(vParams[k])` (the Euclidean grid point `Q_{k,l}`) within `tol`. On success it SHALL
report the K×L averaged Euclidean intersection grid; on failure it SHALL return `ok = false` with
a reason, never crashing.

#### Scenario: A consistent rational network passes

- GIVEN a rational curve network extracted from a KNOWN rational surface (both families from the same surface, so their intersections agree)
- WHEN it is verified rationally
- THEN the routine SHALL return `ok = true` with a grid consistency error at ~1e-9 (achieved: machine precision)

#### Scenario: A non-rational or non-positive-weight curve is declined

- GIVEN a network containing a non-rational curve (empty weights) or a non-positive weight
- WHEN rational verification is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never fabricating weights

#### Scenario: An inconsistent rational network is declined

- GIVEN a rational network whose curves do not meet on the grid within tolerance
- WHEN rational verification is attempted
- THEN the routine SHALL return `ok = false` with a large reported grid error

### Requirement: Rational Gordon / network surface

The module SHALL construct a **rational** tensor-product NURBS surface through a WEIGHTED curve
network as the boolean sum `G = S_u ⊕ S_v ⊖ T` done ENTIRELY in HOMOGENEOUS space
(`gordonRationalSurface(network, tol, uInterpDegree, vInterpDegree)`). The routine SHALL verify
the network rationally, make each family rationally compatible, interpolate the two rational skins
`S_u` (u-curves across v) and `S_v` (v-curves across u) and the rational grid interpolant `T` on
the 4-D homogeneous net `(w·P, w)` (the SAME collocation as the non-rational path, solved for all
four coordinates), bring the three summands to a COMMON basis with the exact rational-aware
Layer-1 `elevateDegreeSurface` / `refineKnotSurface`, form `homog(G) = homog(S_u) + homog(S_v) −
homog(T)` per pole, and project back to a rational `BsplineSurfaceData` (poles + one weight per
pole). The rational Gordon surface SHALL CONTAIN every rational network curve — `G(·, v_k)` equals
`C_k` and `G(u_l, ·)` equals `D_l` pointwise as NURBS.

The routine SHALL require the network to be consistent in HOMOGENEOUS (weight) space at the grid
(`C_k^w(u_l)` equals `D_l^w(v_k)` within `tol`) — an honest precondition, since the boolean sum
cancels on the network curves only when the summands agree homogeneously. It SHALL decline (`ok =
false`, no crash) on fewer than two curves in a direction, a non-rational or non-positive-weight
curve, an inconsistent (Euclidean- or homogeneous-inconsistent) or degenerate network, a singular
interpolation, or a projected non-positive control weight (never divide by ≤ 0, never a faked
rational surface).

#### Scenario: The rational Gordon surface contains every rational network curve

- GIVEN a rational u/v curve network extracted from a KNOWN rational surface (rational in both directions), with strictly positive weights
- WHEN the rational Gordon surface is built
- THEN the result SHALL be rational (one positive weight per pole), its rational iso-curve at each `v_k` SHALL reproduce rational u-curve `k` and at each `u_l` reproduce rational v-curve `l` pointwise to ~1e-9 (achieved: machine precision), and the K×L rational grid points SHALL lie on the surface to ~1e-9

#### Scenario: A non-rational, non-positive-weight, or inconsistent network is declined

- GIVEN a network with a non-rational curve, a non-positive weight, an inconsistent grid, or fewer than two curves in a direction
- WHEN a rational Gordon surface is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never emitting a surface with a non-positive control weight

### Requirement: Rotational (revolved) swept surface

The module SHALL construct an EXACT RATIONAL surface of revolution by revolving a profile curve
about an axis (`sweepRotational(section, axisPoint, axisDir, angle)` in
`src/native/math/bspline_sweep.{h,cpp}`, *The NURBS Book* §8.5 / Algorithm A7.1). The U direction
SHALL carry the profile (its degree, knots, poles, and — if the profile is rational — its
weights). The V direction SHALL be a degree-2 rational circular arc split into `narcs =
ceil(|angle| / 90°)` segments each spanning ≤ 90°, giving `2·narcs + 1` V-poles over a clamped
degree-2 knot vector with interior multiplicity 2. Each profile point (radius `r` from the axis)
SHALL contribute per segment an ON-arc pole (arc weight 1) and a BETWEEN pole at radius
`r / cos(Δθ/2)` (arc weight `cos(Δθ/2)`); the surface weight SHALL be the SEPARABLE product
`wProfile · wArc`. An on-axis profile point (`r ≈ 0`) SHALL keep its position but STILL carry the
arc weight pattern (so the separable weight structure — and thus the exact revolve — is
preserved). The finished surface SHALL be rational (`weights` non-empty), SHALL contain the
profile at `V = 0` and the profile rotated by `angle` at `V = 1`, with `vParams = {0, 1}`.

The routine SHALL decline (`ok = false`, no crash) on a malformed profile, a rational profile with
a non-positive or mismatched weight, a null/non-unit axis direction, a (near-)zero `|angle|`, or a
DEGENERATE placement where the entire profile lies on the axis (every `r ≈ 0`).

#### Scenario: A straight offset segment revolved 360° is an EXACT cylinder

- GIVEN a straight segment parallel to the axis at radius `R`, spanning height `H`, revolved through `2π`
- WHEN it is revolved
- THEN every surface point SHALL lie at exactly radius `R` from the axis and at its own height (`z` linear in the profile parameter), matching the analytic cylinder pointwise to ~1e-9 (achieved ~1e-15) — an exact rational surface, not a facet

#### Scenario: A tilted segment revolved is an EXACT cone/frustum

- GIVEN a straight segment from radius `R0` (at `z=0`) to radius `R1` (at `z=H`), revolved 360°
- WHEN it is revolved
- THEN every surface point SHALL lie at radius `R0 + (R1−R0)·u` (profile parameter `u`, height `u·H`), matching the analytic frustum pointwise to ~1e-9 (achieved ~1e-15)

#### Scenario: A rational semicircle revolved 360° is an EXACT sphere

- GIVEN a rational quadratic semicircle of radius `R` whose diameter lies on the axis, revolved through `2π`
- WHEN it is revolved
- THEN the result SHALL be a rational surface every point of which lies at distance `R` from the center, matching the analytic sphere pointwise to ~1e-9 (achieved ~1e-15) — the strongest revolve oracle

#### Scenario: A partial-angle revolve is the correct rational arc sector

- GIVEN a straight offset segment revolved through a partial angle (e.g. 90°, and 270° which forces 3 arc segments → 7 V-poles)
- WHEN it is revolved
- THEN every surface point SHALL be at the exact radius within the swept sector, and `S(·, 1)` SHALL equal the profile rotated by exactly the swept angle to ~1e-9

#### Scenario: Zero angle, on-axis, null-axis, or malformed input is declined

- GIVEN a zero swept angle, a profile lying entirely on the axis, a null axis direction, a malformed profile, or a rational profile with a non-positive weight
- WHEN a rotational sweep is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never emitting a degenerate or faked surface of revolution

### Requirement: Rational Gordon and rotational revolve are the tractable slice; N-sided and exact-BRepFill remain residuals

Rational Gordon SHALL operate only on PRESCRIBED-weight networks that are consistent in
homogeneous space, and the rotational sweep SHALL produce a surface of revolution about an axis;
neither SHALL fabricate a surface it cannot represent exactly. IRREGULAR / N-sided networks
(non-grid topologies, trimmed boundaries, N-sided fill), exact GeomFill/BRepFill continuous
sweeps (an analytically exact swept surface rather than a skinned station approximation, and the
~1e-6 averaging-knot Gordon reconstruction residual), and VARIABLE-section sweeps SHALL remain
documented residuals recorded in `docs/NURBS-SCOPE.md`. The non-rational Gordon/sweep routines
SHALL continue to return an empty `weights` vector; the rational routines SHALL return one solved
weight per surface pole.

#### Scenario: The module never fakes an unsupported rational surfacing result

- GIVEN a network or profile outside the supported rational Gordon / rotational-revolve slice (an inconsistent network, an N-sided network, or a degenerate profile)
- WHEN a rational Gordon surface or rotational sweep is attempted
- THEN the routine SHALL decline (`ok = false`) rather than emit an approximate or faked surface, and the residual SHALL remain documented in `docs/NURBS-SCOPE.md`

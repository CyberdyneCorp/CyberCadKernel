# native-math

## ADDED Requirements

### Requirement: Rational section compatibility

The native math library SHALL make a set of weighted (rational) B-spline section curves
COMPATIBLE in the OCCT-free `CYBERCAD_HAS_NUMSCI`-gated skinning module
(`makeRationalSectionsCompatible(sections)` in `src/native/math/bspline_skin.{h,cpp}`). Every
input section MUST be rational (non-empty `weights`, one weight per pole, every weight strictly
positive). The routine SHALL raise every section to the common maximum degree and merge every
section onto the union of all sections' knot vectors using the rational-aware Layer-1 ops
`elevateDegreeCurve` and `refineKnotCurve` (which run on the homogeneous R⁴ net so the weights
ride through both operations exactly). After compatibilization every returned section SHALL
share the same degree, the same flat knot vector, and the same control-point count, SHALL carry
one weight per pole, and SHALL still equal its ORIGINAL rational curve pointwise.

The routine SHALL decline (`ok = false`, no crash) when a section is non-rational (empty
weights), when a weight count does not match its pole count, when any weight is not strictly
positive, or when a section is empty / degenerate / has a malformed knot vector.

#### Scenario: Mixed-degree rational sections are made compatible without geometry drift

- GIVEN two or more rational sections of different degree and/or different interior knots over a common domain, each with strictly positive weights
- WHEN they are made rationally compatible
- THEN every returned section SHALL share degree + knot vector + control-point count, carry one positive weight per pole, and evaluate (as a rational NURBS) to its original curve pointwise to ~1e-9 (achieved: machine precision)

#### Scenario: A non-rational or non-positive-weight section is declined

- GIVEN a section set containing a non-rational section (empty weights), a weight/pole count mismatch, or a non-positive weight
- WHEN rational compatibility is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never fabricating weights

### Requirement: Rational B-spline surface skinning (lofting)

The module SHALL construct a **rational** tensor-product NURBS surface that CONTAINS every
weighted section curve as an iso-parametric curve (`skinRationalSurface(sections, degreeV)`).
The routine SHALL make the sections rationally compatible, assign section parameters `v_k` by
chord length across the sections' EUCLIDEAN control polygons, and for each control-point index
`i` interpolate a degree-`degreeV` B-spline in V through the K homogeneous poles
`(w·P_i^k, w_i^k) ∈ R⁴` — using the SAME collocation matrix as the non-rational skin, solved
for all FOUR homogeneous coordinates (the fourth interpolating the weights) — then project the
solved control net back to (pole, weight). The returned surface SHALL be rational (one weight
per pole) and its rational iso-curve at `v = v_k` SHALL equal the compatibilized rational
section `k` to solver precision. `degreeV` SHALL be clamped to `K−1`.

The routine SHALL decline (`ok = false`, no crash) with fewer than two sections, on a
non-rational or non-positive-weight section, on an all-coincident set of sections (no V length
to normalize), or when any solved control weight is not strictly positive (a projected
non-positive weight is a documented guard — never divide by ≤ 0, never a faked rational
surface).

#### Scenario: The rational surface contains every rational section

- GIVEN two or more rational sections (e.g. exact rational circles) sharing a common domain, with strictly positive weights
- WHEN the rational surface is skinned
- THEN the result SHALL be rational (one positive weight per pole) and its rational iso-curve at each `v_k` SHALL reproduce the compatibilized rational section `k` pointwise to ~1e-9 (achieved: machine precision)

#### Scenario: Rational skinning of different-radius circles is a rational cone/frustum

- GIVEN 2+ exact rational circles of DIFFERENT radii stacked along an axis, each with its conic weights
- WHEN the rational surface is skinned
- THEN the surface SHALL contain each rational circle pointwise, and every ring iso-curve SHALL be a TRUE circle of its prescribed radius to ~1e-9

#### Scenario: Degenerate or non-rational input is declined

- GIVEN fewer than two sections, a non-rational section, a non-positive/mismatched weight, or an all-coincident set of sections
- WHEN rational skinning is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never emitting a surface with a non-positive control weight

### Requirement: Rational swept surfaces

The module SHALL construct rational swept surfaces from a weighted section curve
(`src/native/math/bspline_sweep.{h,cpp}`).

`sweepRationalTranslational(section, sweep)` SHALL return the EXACT rational tensor-product
surface whose U direction is the rational section (its degree, knots, poles AND weights) and
whose V direction is a degree-1 two-pole line from `0` to `sweep`. The weight net SHALL be
constant in V (`weight(i,0) = weight(i,1) = section.weights[i]`) and the pole net SHALL be
`pole(i,0) = section.poles[i]`, `pole(i,1) = section.poles[i] + sweep`, so the rational
iso-curve `S(·, v)` is EXACTLY the rational section translated by `v·sweep` — machine-exact, no
fitting. The routine SHALL decline on a non-rational section (empty weights), a
non-positive/mismatched weight, an empty/malformed section, or a null sweep vector.

`sweepRationalAlongTrajectory(section, trajectory, sectionNormal, stations, degreeV)` SHALL
place the rational section at `stations` along a NON-rational trajectory using the rotation-
minimizing moving frame (a RIGID transform, which preserves the section's weights exactly), then
RATIONAL-SKIN the placed rational sections via `skinRationalSurface`. The routine SHALL decline
with fewer than two stations, on a non-rational or non-positive-weight section, on a rational
trajectory, on an empty/malformed section or trajectory, on a degenerate (coincident-station)
trajectory, or on a downstream rational-skin failure.

#### Scenario: A rational circle swept translationally is an EXACT rational cylinder

- GIVEN an exact rational quadratic circle of radius `R` in the XY plane and a sweep vector `H·ẑ`
- WHEN it is swept with `sweepRationalTranslational`
- THEN the result SHALL be a rational surface of degree `(2 × 1)` whose every point `S(u,v)` lies at radius `R` from the axis and at height `v·H`, matching the analytic cylinder (the circle profile lifted to `v·H`) pointwise to ~1e-9 (achieved ~1e-15) — proving an exact rational surface, not a faceted approximation

#### Scenario: The general rational sweep contains each transformed rational section

- GIVEN a rational section and a non-rational trajectory
- WHEN it is swept with `sweepRationalAlongTrajectory`
- THEN the swept rational surface SHALL contain each rigidly transformed rational section at its station parameter to skinning tolerance (~1e-8)

#### Scenario: Non-rational, non-positive-weight, or rational-trajectory input is declined

- GIVEN a non-rational section (empty weights), a non-positive/mismatched weight, a null sweep, a rational trajectory, fewer than two stations, or a coincident trajectory
- WHEN a rational sweep is attempted
- THEN the routine SHALL return `ok = false`, never crashing and never emitting a non-rational or non-positive-weight result

### Requirement: Rational surfacing is prescribed-weight; Gordon-rational and irregular networks are residuals

Rational skinning and rational sweeps SHALL operate only on sections carrying PRESCRIBED
weights, and SHALL NOT estimate weights from unweighted geometry. Rational Gordon / network
surfaces, irregular / N-sided networks, rotational (revolved) sweeps, and exact GeomFill/BRepFill
continuous sweeps SHALL remain documented residuals recorded in `docs/NURBS-SCOPE.md`. The
non-rational skin/sweep routines SHALL continue to return an empty `weights` vector; the rational
routines SHALL return one solved weight per surface pole.

#### Scenario: The module never fakes a rational surface

- GIVEN a request for rational surfacing
- WHEN no per-pole weights are prescribed on the sections
- THEN the module SHALL offer only non-rational skinning/sweep (empty weights) — it SHALL NOT invent weights to present a rational surface, and the rational routines require an explicit positive weight per pole

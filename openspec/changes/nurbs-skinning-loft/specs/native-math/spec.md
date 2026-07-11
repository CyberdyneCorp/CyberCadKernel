# native-math

## ADDED Requirements

### Requirement: Section curve compatibility for skinning

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_skin.{h,cpp}`, namespace `cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated),
a routine that makes a set of `K` B-spline **section curves** COMPATIBLE (*The NURBS Book* §10.3):
it SHALL raise every section to the common maximum degree (via the Layer-1 exact `elevateDegreeCurve`)
and merge every section onto the UNION of all sections' knot vectors — the maximum multiplicity per
distinct knot value — (via the Layer-1 exact `refineKnotCurve`), so that afterwards every section
shares the SAME degree, the SAME flat knot vector, and therefore the SAME control-point count
`N = knots.size() − degree − 1`. Because both Layer-1 operations are exact (geometry-preserving),
each compatible section SHALL still represent its ORIGINAL curve exactly. The routine SHALL operate
on **non-rational** sections only: a rational section (non-empty weights) SHALL make the call decline
(`ok = false`), never a silently-wrong result. Fewer than one section, an empty section, a degree
below one, or a malformed knot vector SHALL also decline without crashing.

#### Scenario: Mixed-degree, mixed-knot sections become compatible without geometry drift

- GIVEN a set of at least two non-rational sections of possibly different degree and different interior knots over a common parameter domain
- WHEN the sections are made compatible
- THEN every returned section SHALL share the common maximum degree, the union knot vector, and the same control-point count `N`, and each compatible section SHALL evaluate to the SAME point as its ORIGINAL section at every parameter to within ~1e-11 (exact, no drift from elevation/refinement)

#### Scenario: Rational or malformed sections decline honestly

- GIVEN a set of sections in which at least one is rational (non-empty weights), empty, of degree below one, or has a knot vector inconsistent with its pole count
- WHEN the sections are made compatible
- THEN the routine SHALL return `ok = false` without crashing, and SHALL NOT emit a compatibilized set that drops or fabricates weights

### Requirement: Skinned tensor-product surface containing every section

The module SHALL construct, from `K ≥ 2` B-spline section curves and a chosen V degree, a
**non-rational** tensor-product B-spline surface that CONTAINS every section as an iso-parametric
curve (*The NURBS Book* Algorithm A10.3). It SHALL first make the sections compatible (shared
degree `p`, knots, control-point count `N`), assign section parameters `v_k ∈ [0,1]` by chord length
across the sections' control polygons (averaged over the `N` control-point indices, ends pinned),
build a common averaging V-knot vector of degree `q` (clamped to `[1, K−1]`), and for each
control-point index `i ∈ [0,N)` interpolate a degree-`q` B-spline in V through the `K` points
`{P_i^k}` at parameters `v_k` on the common V-knots (a collocation solve via the numerics facade
`lin_solve`, one solve per coordinate). The result SHALL be a row-major, U-outer
`BsplineSurfaceData` with `nPolesU = N`, `nPolesV = K`, U carrying the section shape (degree `p`,
common section knots) and V the across-sections interpolation (degree `q`, averaging V-knots), and
empty weights. The core guarantee: the surface's iso-curve at `v = v_k` SHALL equal the compatible
section `k` pointwise. Fewer than two sections, coincident sections (no V length to normalize), a
rational section, or a singular V-collocation SHALL return `ok = false` without crashing.

#### Scenario: The skinned surface contains every section pointwise

- GIVEN `K ≥ 2` non-rational sections and a V degree
- WHEN the surface is skinned
- THEN the result SHALL be a non-rational surface with `nPolesU = N` and `nPolesV = K`, and its iso-curve `S(·, v_k)` SHALL equal the compatible section `k` at every parameter `u` to within ~1e-8 (achieved: machine precision), for every section `k` — the surface CONTAINS every input section

#### Scenario: Skinning reconstructs known tensor-product geometry (idempotent round-trip)

- GIVEN a KNOWN tensor-product B-spline surface whose iso-curves at chosen stations are extracted as sections and skinned, and separately a skinned surface `S1` whose own iso-curves at its own section parameters `v_k` are re-extracted and re-skinned into `S2`
- WHEN the results are compared
- THEN the surface skinned from the KNOWN surface's iso-curves SHALL contain each extracted iso-curve to ~1e-8, AND `S1` and `S2` SHALL have identical control nets and evaluate to the same point at every `(u,v)` to within ~1e-8 (achieved: machine precision) — proving the skin reconstructs known geometry pointwise where the V-parametrization is a fixed point

#### Scenario: Degenerate and incompatible inputs are handled honestly

- GIVEN fewer than two sections, or a set of coincident (identical) sections, or a set containing a rational section, or an incompatible-but-recoverable pair (differing only in degree and knots)
- WHEN the surface is skinned
- THEN the fewer-than-two, coincident, and rational cases SHALL return `ok = false` without crashing, and the incompatible-but-recoverable pair SHALL skin successfully (compatibility recovers a common representation) with every section still contained exactly

### Requirement: Non-rational skinning scope with rational and general surfacing as explicit residuals

The skinning module SHALL produce **non-rational** tensor-product B-spline surfaces (all weights
implicitly one; the output `weights` vector is empty) from **non-rational** sections. It SHALL NOT
fabricate weights or claim rational-skinning capability. Rational / weighted skinning (interpolating
the section weights), general Gordon / network / boundary surfacing (multiple transversal curve
families, N-sided filling, plate/energy surfaces), and exact swept surfaces (a section swept along a
spine with orientation frames) are documented residuals for later slices, recorded in
`docs/NURBS-SCOPE.md`.

#### Scenario: Skinned geometry is non-rational, not faked-rational or faked-general

- GIVEN any successful skin
- WHEN the result is inspected
- THEN its `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights, nor emit a Gordon/network/swept surface, nor claim those capabilities

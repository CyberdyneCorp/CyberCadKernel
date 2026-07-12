# native-math

## ADDED Requirements

### Requirement: Four-sided boundary-corner consistency verification for Coons patches

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_coons.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine that verifies a set of FOUR boundary B-spline curves —
`c0(u)` (the edge `v=0`), `c1(u)` (the edge `v=1`), `d0(v)` (the edge `u=0`), `d1(v)` (the edge
`u=1`) — forms a CONSISTENT topological quad on the unit square. The routine SHALL require every
boundary to be **non-rational** and well-formed (clamped flat knot vector, degree ≥ 1, ≥ 2 control
points), and the four shared corners to coincide: `c0(0)` SHALL equal `d0(0)`, `c0(1)` SHALL equal
`d1(0)`, `c1(0)` SHALL equal `d0(1)`, and `c1(1)` SHALL equal `d1(1)`, each to within a
caller-supplied tolerance. It SHALL report the maximum corner mismatch. On any violation the routine
SHALL return `ok = false` with a human-readable reason, never a silently-wrong quad and never a
crash.

#### Scenario: A consistent four-sided boundary verifies with a machine-precision corner error

- GIVEN four boundary curves whose shared corners coincide (for example, boundary iso-curves extracted from a single tensor-product surface, or four curves each pinned to the shared corner points)
- WHEN the boundary is verified
- THEN the routine SHALL return `ok = true` with `maxCornerError` at machine precision (~1e-9 or below)

#### Scenario: A mismatched-corner, rational, or malformed boundary declines honestly

- GIVEN a boundary in which one corner is displaced (so opposing curves no longer meet), or containing a rational curve (non-empty weights), or with a malformed / degenerate curve (degree 0 or a bad knot vector)
- WHEN the boundary is verified
- THEN the routine SHALL return `ok = false` with a reason and (for the displaced case) a large `maxCornerError`, without crashing

### Requirement: Bilinearly-blended Coons patch interpolating all four boundary curves

The module SHALL construct, from a consistent four-sided boundary (four `c0`/`c1`/`d0`/`d1` curves
with matching corners), a **non-rational** tensor-product B-spline surface that INTERPOLATES all four
boundary curves — the boolean sum `Coons = L_u ⊕ L_v ⊖ B` (*The NURBS Book* §10.5 / Coons 1967). It
SHALL first verify the boundary (declining honestly on mismatched-corner / rational / degenerate
input), make each opposing pair compatible in its own direction (shared degree/knots/control-count
via the Layer-1 exact `elevateDegreeCurve`/`refineKnotCurve` — `c0`/`c1` in u, `d0`/`d1` in v), then
build three tensor-product summands: `L_v` the ruled surface between `c0` and `c1` (degree 1 in v,
the c-shape carried in u), `L_u` the ruled surface between `d0` and `d1` (degree 1 in u, the d-shape
carried in v), and `B` the degree-(1,1) bilinear tensor product of the four corner points. It SHALL
raise the three summands to a COMMON degree and merge them onto COMMON knot vectors in each direction
with the exact Layer-1 surface ops (`elevateDegreeSurface` / `refineKnotSurface`,
geometry-preserving), then form the Coons control net pointwise
`poles(Coons) = poles(L_v) + poles(L_u) − poles(B)` with empty weights. The core guarantee: the
surface's edge iso-curve `S(·, 0)` SHALL equal `c0`, `S(·, 1)` SHALL equal `c1`, `S(0, ·)` SHALL
equal `d0`, and `S(1, ·)` SHALL equal `d1`, and the four corners SHALL be interpolated exactly. A
mismatched-corner / rational / degenerate boundary, or a failure to reach a common basis, SHALL
return `ok = false` without crashing.

#### Scenario: The Coons surface contains every boundary curve and corner pointwise

- GIVEN a consistent non-rational four-sided boundary
- WHEN the Coons patch is built
- THEN the result SHALL be a non-rational tensor-product surface whose four edge iso-curves `S(·,0)`, `S(·,1)`, `S(0,·)`, `S(1,·)` equal the four boundary curves `c0`, `c1`, `d0`, `d1` pointwise on a dense sample to within ~1e-9 (achieved: machine precision), and whose four corners equal the boundary corners exactly to within ~1e-12

#### Scenario: A flat boundary yields a flat patch matching the plane

- GIVEN a boundary whose four edges are coplanar (for example the four edges of a rectangle in the `z=0` plane, with in-plane curvature)
- WHEN the Coons patch is built
- THEN every point of the surface SHALL lie on that plane to within ~1e-12, AND for a rectangular straight-edge boundary the surface SHALL equal the exact planar bilinear patch `P00(1-u)(1-v) + P10·u(1-v) + P01(1-u)v + P11·uv`

#### Scenario: Coons reconstructs a bilinearly-blended surface (known-surface round-trip)

- GIVEN a KNOWN RULED / bilinearly-blended tensor-product surface whose four boundary iso-curves are extracted and fed to the Coons builder, and separately a general tensor-product surface whose four boundary iso-curves are extracted
- WHEN the Coons patches are compared to the sources
- THEN the Coons patch of the RULED surface's boundary SHALL recover the source POINTWISE to within ~1e-9 (achieved: machine precision — Coons is exact for bilinearly-blended surfaces), AND the Coons patch of the GENERAL surface's boundary SHALL reproduce the four boundary curves exactly (the interior being the bilinear blend by definition, verified as boundary containment, not an interior match)

#### Scenario: Mismatched-corner and degenerate boundaries decline honestly

- GIVEN a boundary with a displaced corner, or a rational boundary curve, or a malformed / degenerate boundary curve
- WHEN the Coons patch is requested
- THEN the routine SHALL return `ok = false` with a reason, without crashing, and SHALL NOT emit a surface that silently misses its own boundary curves

### Requirement: Non-rational four-sided Coons scope with N-sided, rational, and plate blends as explicit residuals

The Coons module SHALL produce **non-rational** tensor-product B-spline surfaces (all weights
implicitly one; the output `weights` vector is empty) from **non-rational** FOUR-sided boundaries
only. It SHALL NOT fabricate weights or claim rational-Coons capability, nor accept more or fewer
than four boundaries. Rational / weighted Coons patches (interpolating weighted boundaries),
N-SIDED fill (5+ boundaries, degenerate-corner / triangular patches), and the Gregory /
energy-minimizing PLATE blends that achieve tangent (G1) / curvature (G2) continuity to the
surrounding surfaces are documented residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: Coons geometry is non-rational, four-sided, not faked-rational or faked-N-sided

- GIVEN any successful Coons patch
- WHEN the result is inspected
- THEN its `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights, nor emit an N-sided / triangular / G1-G2-continuous plate surface, nor claim those capabilities

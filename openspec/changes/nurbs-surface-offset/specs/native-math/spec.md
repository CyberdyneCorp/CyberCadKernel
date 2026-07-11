# native-math

## ADDED Requirements

### Requirement: NURBS surface offset by normal-locus sampling and fitting

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_offset.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `offsetSurface` that constructs the OFFSET of a
tensor-product NURBS surface `S(u,v)` at signed distance `d` as a fitted **non-rational** B-spline
surface. The true offset locus is `O(u,v) = S(u,v) + d·N(u,v)`, where `N` is the unit surface normal
`(Sᵤ × Sᵥ)/‖Sᵤ × Sᵥ‖`; positive `d` offsets along `+N`, negative along `−N`. Because the exact offset
of a NURBS surface is not in general a NURBS (the unit normal carries a square root), the routine
SHALL sample the offset locus on an adaptive `(u,v)` grid and INTERPOLATE a tensor-product B-spline
through the samples using the Layer-7 fitter (`interpolateSurface`, `numerics::lin_solve`
collocation), REFINING the grid until the fitted surface's maximum deviation from the true offset
locus is within a caller tolerance `tol`, or a refinement budget (`maxGrid`) is spent. The routine
SHALL report the ACHIEVED maximum offset error (the true deviation, never widened). The input surface
MAY be rational (its weights are honoured through `nurbsSurfacePoint` / `surfaceNormal`); the fitted
offset SHALL be non-rational (empty weights).

#### Scenario: Every point of the offset surface lies at distance |d| from S along its normal

- GIVEN a curved (non-degenerate, fold-free) NURBS surface `S` and a signed distance `d`
- WHEN the offset surface is constructed and accepted (`ok = true`)
- THEN for a dense grid of points on the fitted offset surface, the nearest-point distance from each point to `S` SHALL equal `|d|` to within the reported maximum offset error, for both signs of `d`

#### Scenario: Offset error is the achieved deviation from the true locus, honestly reported

- GIVEN any accepted offset
- WHEN the result's reported maximum error is inspected
- THEN it SHALL be the true maximum deviation of the fitted surface from the offset locus `S + d·N` (never a widened or nominal value), and the fitted surface SHALL be non-rational (empty weights)

### Requirement: Analytic offset of a cylinder and a plane

For surfaces whose offset has a closed form, the offset SHALL match it to the fitting tolerance. The
offset of a NURBS-represented CYLINDER of radius `r` by `d` SHALL lie on the coaxial cylinder of
radius `r + d` (outward) or `r − d` (inward). The offset of a PLANE by `d` SHALL be the exact
parallel plane translated by `d` along the plane's unit normal.

#### Scenario: A cylinder offsets to a coaxial cylinder of radius r±d

- GIVEN a NURBS quarter-cylinder of radius `r` about an axis and a signed distance `d` with `r + d > 0`
- WHEN the offset surface is constructed
- THEN every point of the fitted offset SHALL lie at distance `r + d` from the cylinder axis to within the fitting tolerance

#### Scenario: A plane offsets to an exact parallel plane

- GIVEN a planar NURBS patch with unit normal `n̂` and a signed distance `d`
- WHEN the offset surface is constructed
- THEN the fitted offset SHALL be the parallel plane displaced by `d·n̂`, matching it to ~1e-9 (the flat offset is exactly fittable), with a reported error ~0

### Requirement: Offset error converges monotonically under grid refinement

As the offset-locus sample grid is refined, the reported maximum offset error SHALL decrease
monotonically (within a small floating-point slack) and SHALL be the achieved deviation, never
widened. The refinement SHALL be bounded by a caller-supplied maximum grid resolution.

#### Scenario: Refining the sample grid lowers the reported offset error

- GIVEN a fixed curved surface and signed distance, offset at increasing maximum grid resolutions
- WHEN the reported maximum offset errors are compared across resolutions
- THEN each finer resolution's reported error SHALL be no greater than the coarser resolution's (monotone decrease, small fp slack), converging toward zero

### Requirement: Self-intersecting and degenerate offsets are declined honestly

The routine SHALL DETECT an offset that would self-intersect (fold) and DECLINE it rather than
return folded geometry. An offset folds when `|d|` reaches a principal radius of curvature of `S` on
the side the offset bends toward — equivalently, when the offset map's Jacobian factor `(1 + d·κ)`
reaches zero for some principal curvature `κ`. The routine SHALL compute the principal curvatures
from the surface's first and second fundamental forms (via the order-2 surface derivatives) on a
dense analysis grid; if the smallest `(1 + d·κ)` over the grid is `≤ 0`, it SHALL return a
self-intersection status with `ok = false` and NO surface (empty), reporting the minimum curvature
radius on the folding side. A patch with a near-zero (degenerate) surface normal, or a degenerate
tangent-plane metric, SHALL decline as a degenerate-normal case. A malformed or empty input surface
SHALL decline as degenerate input. In no case SHALL the routine return a folded or degenerate offset
as a valid surface, and in no case SHALL it crash.

#### Scenario: An offset past the curvature radius is declined as a fold, not returned folded

- GIVEN a tightly-curved patch and a signed distance `d` whose magnitude exceeds the patch's minimum principal radius of curvature on the concave side
- WHEN the offset surface is requested
- THEN the routine SHALL return `ok = false` with a self-intersection status and no surface, and SHALL report the minimum curvature radius (≈ the patch's radius of curvature) it tripped on — it SHALL NOT return a folded surface

#### Scenario: A safe small offset of the same tight patch succeeds

- GIVEN the same tightly-curved patch and a small signed distance whose magnitude is well within every principal radius of curvature on the offsetting side
- WHEN the offset surface is requested
- THEN the routine SHALL construct a valid offset surface (`ok = true`) — the self-intersection guard is a genuine curvature test, not a blanket rejection

#### Scenario: A degenerate-normal patch declines without crashing

- GIVEN a patch whose surface normal is near-null somewhere on its domain (e.g. a collapsed control net with no extent in one parameter direction)
- WHEN the offset surface is requested
- THEN the routine SHALL return `ok = false` with a degenerate-normal (or degenerate-input) status, without crashing and without returning a surface

### Requirement: Non-rational offset scope with solid thicken/shell and fold-trimming as explicit residuals

The offset module SHALL produce a **non-rational** offset SURFACE (empty weights). It SHALL NOT
fabricate weights, and SHALL NOT claim solid thicken / shell / hollow (offsetting both faces of a
solid and stitching side walls into a closed B-rep solid), robust self-intersecting-offset trimming
(recovering a valid offset from a folded region by trimming rather than declining), or rational
offset fitting. These are documented residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: Offset output is a single non-rational surface, not a solid or a trimmed fold

- GIVEN any accepted offset
- WHEN the result is inspected
- THEN it SHALL be a single non-rational `BsplineSurfaceData` (empty weights), and the module SHALL NOT emit a closed solid shell, a trimmed self-intersecting offset, or fabricated weights, nor claim those capabilities

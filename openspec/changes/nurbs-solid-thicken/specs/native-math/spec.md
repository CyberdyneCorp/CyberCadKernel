# native-math

## ADDED Requirements

### Requirement: NURBS solid thicken into a closed watertight shell

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_thicken.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `thickenSurface` that thickens an OPEN tensor-product NURBS
surface `S(u,v)` by a signed distance `d` into a CLOSED, watertight SOLID. The solid's boundary is
the original surface `S`, its offset surface `O(u,v) = S(u,v) + d·N(u,v)` (from the Layer-5
`offsetSurface`), and four ruled SIDE WALLS joining the boundary loops of `S` and `O`; positive `d`
thickens along `+N`, negative along `−N`. The solid SHALL be returned as a triangle shell
(`tessellate::Mesh`: fp64 `Point3` vertices + indexed triangles) assembled so that the three panel
kinds share EXACT boundary vertices (watertight by construction). The input surface MAY be rational
(its weights are honoured through `nurbsSurfacePoint` / `surfaceNormal`); the offset panel is
non-rational (inherited from `offsetSurface`).

#### Scenario: The thickened solid is watertight and consistently oriented

- GIVEN a well-formed open NURBS surface `S` and a signed thickness `d` above the linear tolerance for which the offset is fold-free
- WHEN the solid is constructed and accepted (`ok = true`)
- THEN the returned shell SHALL be a closed 2-manifold — every undirected edge shared by exactly two triangles (watertight), zero boundary edges, Euler characteristic `χ = 2`, and consistently oriented (every interior directed half-edge used once forward and once reversed) — for both signs of `d`

#### Scenario: A non-closed assembled shell is declined, never returned

- GIVEN any thicken whose assembled shell is not watertight or not consistently oriented
- WHEN closure is verified
- THEN the routine SHALL return `ok = false` with a not-closed status and no solid — it SHALL NOT return an open or leaky shell as a valid solid

### Requirement: Enclosed volume matches the closed form for analytic patches

The enclosed volume of the thickened solid SHALL be the divergence-theorem volume of its watertight
boundary and SHALL match the exact closed form where one exists. For a flat rectangular patch of
extent `Lx × Ly` thickened by `d`, the solid is a box and its volume SHALL equal `Lx·Ly·|d|`
exactly. For a thin slab over a curved patch, the volume SHALL converge to `(mid-surface area)·|d|`
as `|d| → 0`. For a cylindrical wedge shell, the volume SHALL match the annular-wedge closed form.

#### Scenario: A flat rectangle thickened by d is an exact box

- GIVEN a flat rectangular NURBS patch of extent `Lx × Ly` and a signed thickness `d`
- WHEN the solid is constructed
- THEN its enclosed volume SHALL equal `Lx·Ly·|d|` to ~1e-9, and its reported mid-surface area SHALL equal `Lx·Ly`

#### Scenario: A curved patch's volume converges to area·|d| as the slab thins

- GIVEN a fixed curved NURBS patch thickened at decreasing thicknesses `|d|`
- WHEN the enclosed volumes are compared to `(mid-surface area)·|d|` across thicknesses
- THEN the relative discrepancy SHALL shrink monotonically as `|d| → 0` (thin-slab limit)

### Requirement: The offset side of the solid matches the Layer-5 offset at distance |d|

The offset panel of the thickened solid SHALL be consistent with `offsetSurface`: every offset-cap
vertex SHALL lie at distance `|d|` from `S` along its normal, and the reported offset error SHALL be
the same achieved deviation `offsetSurface` reports. The original panel SHALL lie on `S`.

#### Scenario: Every offset-cap vertex is |d| from S and the original cap is on S

- GIVEN an accepted thicken of a curved surface `S` by `d`
- WHEN each offset-cap vertex is projected onto `S` (nearest point) and each original-cap vertex is projected onto `S`
- THEN every offset-cap vertex's nearest distance to `S` SHALL equal `|d|` (to ~1e-9), every original-cap vertex SHALL lie on `S` (distance ~0), and the reported offset error SHALL equal `offsetSurface`'s achieved maximum error

### Requirement: Self-intersecting and degenerate thickens are declined honestly

The routine SHALL DECLINE a thicken that would self-intersect (fold) rather than return folded
geometry. A thicken folds when `|d|` reaches a principal radius of curvature of `S` on the side the
offset bends toward — detected by the Layer-5 offset's second-fundamental-form guard. On such a
thicken the routine SHALL return a self-intersection status with `ok = false` and no solid,
reporting the minimum curvature radius on the folding side. A patch with a near-null (degenerate)
surface normal, a malformed input, or a zero (below-tolerance) thickness SHALL likewise decline with
the corresponding status. In no case SHALL the routine return a folded, degenerate, or non-closed
solid as valid, and in no case SHALL it crash.

#### Scenario: A thicken past the curvature radius is declined as a fold, not returned folded

- GIVEN a tightly-curved patch and a signed thickness `d` whose magnitude exceeds the patch's minimum principal radius of curvature on the concave side
- WHEN the thicken is requested
- THEN the routine SHALL return `ok = false` with a self-intersection status and no solid, and SHALL report the minimum curvature radius (≈ the patch's radius of curvature) it tripped on — it SHALL NOT return a folded solid

#### Scenario: A safe small thicken of the same tight patch succeeds and is closed

- GIVEN the same tightly-curved patch and a small signed thickness on the fold-free side, well within every principal radius of curvature
- WHEN the thicken is requested
- THEN the routine SHALL construct a valid CLOSED watertight solid (`ok = true`) — the fold guard is a genuine curvature test, not a blanket rejection

#### Scenario: A degenerate or zero-thickness request declines without crashing

- GIVEN a patch whose surface normal is near-null somewhere on its domain, OR a thickness `d` at or below the linear tolerance
- WHEN the thicken is requested
- THEN the routine SHALL return `ok = false` with a degenerate-normal / degenerate-input / zero-thickness status and no solid, without crashing

### Requirement: Single-patch non-rational thicken scope with robust/multi-face/rational as residuals

The thicken module SHALL produce a CLOSED shell for a SINGLE NURBS patch (2 caps + 4 walls). It SHALL
NOT claim robust self-intersecting-shell recovery (trimming a folded region rather than declining),
multi-face shells (thickening a whole multi-patch B-rep with mitred/rounded corners), or a rational
offset panel. These are documented residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: Thicken output is a single closed box-topology shell, not a trimmed fold or multi-face solid

- GIVEN any accepted thicken
- WHEN the result is inspected
- THEN it SHALL be a single closed watertight `tessellate::Mesh` of box topology (two caps + four ruled walls) with a non-rational offset panel, and the module SHALL NOT emit a trimmed self-intersecting shell, a multi-face shell, or a rational offset, nor claim those capabilities

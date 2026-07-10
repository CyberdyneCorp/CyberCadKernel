# native-surfacing

## ADDED Requirements

### Requirement: Native bounded N-sided fill (tessellated Coons/Gregory patch over analytic boundaries)

The kernel SHALL provide a native, **OCCT-free** surface-fill capability that fills an
N-sided boundary loop (3 ≤ N ≤ 6) of ANALYTIC / POLYLINE edges (straight segments +
circular arcs) with a smooth interpolating patch, EVALUATED to a TESSELLATED triangle-grid
mesh. The patch SHALL be a Coons / Gregory-style transfinite interpolant of the boundary
curves; it SHALL NOT be stored as a general trimmed-NURBS surface, and the kernel SHALL NOT
add a general NURBS surface representation or a NURBS evaluator beyond what a Coons/Gregory
patch over the given boundary curves needs.

For a boundary loop with N in {3,4} the builder SHALL evaluate a discrete **Coons patch**:
for each of the four logical sides (a triangle folds one corner), sample the two opposing
boundary curves and linearly blend, subtracting the bilinear corner term, producing a
`(gridN+1)×(gridN+1)` grid of interior points; the four boundary rows/columns SHALL equal
the boundary-curve samples BIT-EXACTLY (identical `math::Point3` values), so the patch
shares the boundary points. For N in {5,6} the builder SHALL evaluate a **Gregory-style
convex combination** of the per-side Coons contributions over a mean-value / generalized
barycentric parameterization of the N-gon, again reusing the boundary samples bit-exactly
on the boundary.

The fill SHALL be exposed through two native entry points:
- `surface::fillNGon(boundary, opts, &decline)` returning the tessellated `tess::Mesh`
  patch surface (an open mesh) plus the measured on-boundary residual;
- `surface::fillHoleSolid(openShell, opts, &decline)` returning a welded watertight
  `topo::Shape` solid — it finds the single free-boundary loop of an open shell (the
  used-once-edge boundary graph, re-implemented locally so `heal/` stays untouched),
  builds the fill patch, and welds it to the shell's existing faces on shared boundary
  points via the `boolean::assembleSolid(std::vector<boolean::Polygon>)` substrate.

The solid result SHALL be accepted ONLY when it passes the self-verify (identical semantics
to `heal::self_verify`): watertight closed 2-manifold across the deflection ladder and
positive enclosed volume; and it SHALL be consistently oriented (`tess::isConsistentlyOriented`).
A candidate that fails SHALL be DISCARDED (honest decline). When the boundary loop is planar
within tolerance the patch SHALL reduce to the planar face (fan triangulation) that restores
the hole EXACTLY (patch area = the polygon area; a box's missing face restored ⇒ volume
restored exactly). When the boundary is non-planar the tessellated Coons patch SHALL
interpolate every boundary curve to an on-boundary residual ≤ 1e-9 and its deviation from
the smooth ideal SHALL be bounded and CONVERGE as `gridN` increases.

The native path SHALL REUSE `boolean::Polygon` / `boolean::extractPolygons` /
`boolean::assembleSolid`, `tess::Mesh` / `tess::isWatertight` / `tess::isConsistentlyOriented`
/ `tess::enclosedVolume` / `tess::surfaceArea`, and `math` BYTE-IDENTICALLY, adding no change
to those landed modules, and SHALL keep the tessellator, boolean, construct, blend, analysis,
exchange, sheetmetal, and heal modules UNTOUCHED.

Anything outside this bound SHALL be honest-declined (→ OCCT `BRepFill_Filling` /
`BRepOffsetAPI_MakeFilling`) with a measured `NGonDecline`: a boundary edge that is neither a
straight segment nor a circular arc (`NonAnalyticBoundary`); a loop with fewer than 3 or more
than 6 sides (`TooManySides`); a degenerate / zero-length / duplicate-corner boundary
(`DegenerateBoundary`); a patch whose grid self-intersects or whose weld does not produce a
watertight positive-volume solid (`SelfIntersecting` / self-verify failure); or a requested
higher (G2) continuity the bounded patch cannot honestly provide (`NotConverged`). The engine
SHALL report the decline and SHALL NEVER hand a native void to OCCT as if it were a fill.

#### Scenario: Planar quad hole in a box filled watertight (volume restored exactly)

- **WHEN** `fillHoleSolid` fills the single missing planar quad face of an otherwise closed
  box open shell under the native engine
- **THEN** the result is a watertight, consistently-oriented single lump whose enclosed
  volume equals the original box volume to ≤ 1e-9 relative, and the fill patch shares the
  four hole-boundary corner points bit-exactly

#### Scenario: Planar N-gon patch area equals the polygon area

- **WHEN** `fillNGon` fills a planar N-gon (N in 3..6) boundary loop of straight edges
- **THEN** the tessellated patch's surface area equals the exact polygon area to ≤ 1e-9
  relative, and every boundary sample lies on the boundary polygon (on-boundary residual 0)

#### Scenario: Saddle 4-sided analytic boundary interpolated with bounded, converging deviation

- **WHEN** `fillNGon` fills a non-planar (saddle) 4-sided boundary of straight/arc edges
- **THEN** every boundary sample of the patch lies on its boundary curve to a residual
  ≤ 1e-9, the patch is watertight when welded to a matching cup, and the patch's deviation
  from the analytic Coons ideal decreases monotonically as `gridN` doubles (convergence)

#### Scenario: Non-analytic / >6-sided / self-intersecting boundary honest-declines

- **WHEN** `fillNGon` / `fillHoleSolid` is asked to fill a boundary with a non-analytic
  (spline) edge, more than six sides, a degenerate corner, or a boundary whose patch cannot
  weld watertight
- **THEN** it returns an empty patch / NULL Shape with the corresponding measured
  `NGonDecline`, and the engine reports the decline (→ OCCT), never a wrong or leaky patch

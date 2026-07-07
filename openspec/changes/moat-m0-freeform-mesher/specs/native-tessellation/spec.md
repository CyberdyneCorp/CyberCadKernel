# native-tessellation

## ADDED Requirements

### Requirement: Mesh a genuinely trimmed freeform face watertight via an additive interior-sampled path proven byte-identical for existing kinds

The library SHALL mesh a native `Face` whose surface is a freeform kind
(`Kind::BSpline` — rational `weights` included — or `Kind::Bezier`) and whose outer
bound is a **genuinely trimmed `EDGE_LOOP`** (a real pcurve-bounded loop that is NOT
the full parametric rectangle, i.e. it fails `isFullRectangle`), to a triangle mesh
at a requested deflection, respecting the deflection bound exactly as for the other
curved kinds. The mesh SHALL be produced through a **new, additive** mesh branch that:

- flattens the `EDGE_LOOP` (and any inner hole loops) to a UV boundary polygon at the
  **shared per-edge fractions** (the existing STAGE-2 shared-edge machinery) and
  records the canonical seam anchors, so the two faces of every shared edge place
  BIT-IDENTICAL boundary points and the solid welds watertight;
- samples the surface **INTERIOR** of the trim region on a curvature-driven UV grid
  (the SAME sagitta metric `Δ ≤ √(8·deflection/‖S″‖)` used by the structured-grid
  path), keeping only points inside the outer loop and outside every hole, so every
  interior triangle's chord deviation is within the deflection bound (the ear-clip
  path's missing interior sampling is exactly the gap this closes for a CURVED patch);
- triangulates the boundary and interior together with a **boundary-constrained** UV
  triangulation in which EVERY boundary segment survives as a triangle edge (the
  shared-edge samples are preserved verbatim), with no gaps and no overlaps;
- evaluates every UV vertex to `S(u,v)` on the true surface (rational-aware:
  `math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs` when `weights` is non-empty),
  snapping boundary vertices to their canonical anchors and flipping normals for a
  Reversed face.

The addition SHALL NOT modify the `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`,
bare-periodic `BSpline`, or `Bezier` structured-grid path, NOR the PLANAR trimmed
ear-clip path (`triangulatePolygon`): every existing face SHALL mesh
**byte-identically** — the same triangle counts, the same watertight status, and the
same enclosed volumes — as before this change, PROVEN across the full
tessellation-sensitive suite (`run-sim-suite`, STEP import, curved-fillet,
curved-chamfer, curved-boolean, wrap-emboss, loft, phase3). The new branch SHALL be
reachable ONLY by a curved genuinely-trimmed freeform face (a case that today
produces no valid mesh and declines), so no passing mesh can change. If a clean
additive path that keeps every existing mesh byte-identical AND meshes the trimmed
freeform face watertight cannot be achieved, the mesh branch SHALL be reverted and
the trimmed freeform face SHALL keep the honest OCCT decline (an OCCT-imported patch
loses nothing). The library SHALL remain OCCT-free and host-buildable, and no
tolerance SHALL be weakened.

#### Scenario: A trimmed B-spline face meshes watertight within the deflection bound (host)

- GIVEN a native `Kind::BSpline` face over a curved patch, bounded by a genuinely trimmed `EDGE_LOOP` whose edges carry faithful pcurves, built on the host with no OCCT and a requested deflection `d`
- WHEN it is meshed through the additive trimmed-freeform branch
- THEN every triangle's chord-height deviation from the true surface SHALL be at or below `d`, the interior SHALL be sampled (not a bare boundary web), every boundary segment SHALL be preserved as a triangle edge, the meshed solid SHALL be watertight, AND its enclosed volume SHALL converge to the independent value within the deflection tolerance

#### Scenario: A trimmed rational NURBS face meshes on the true rational surface (host)

- GIVEN a native `Kind::BSpline` face with non-empty `weights` (a rational NURBS patch) bounded by a trimmed `EDGE_LOOP`, built on the host with no OCCT
- WHEN it is meshed through the additive branch
- THEN every emitted vertex SHALL equal `math::nurbsSurfacePoint(u,v)` within `1e-9` (the rational evaluator, not a polynomial approximation) AND the mesh SHALL satisfy the same deflection, boundary-preservation, and watertight guarantees as the non-rational case

#### Scenario: A trimmed freeform face with a hole omits the hole and stays watertight (host)

- GIVEN a native trimmed `Kind::BSpline` face carrying an inner `EDGE_LOOP` hole, built on the host with no OCCT
- WHEN it is meshed through the additive branch
- THEN no triangle SHALL fall inside the hole loop, the hole-boundary vertices SHALL lie on the hole's pcurve within tolerance, every boundary segment (outer and hole) SHALL be preserved as a triangle edge, AND the meshed region SHALL remain watertight against its neighbours

#### Scenario: The additive trimmed-freeform branch leaves every existing kind's mesh byte-identical (host + sim)

- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, bare-periodic `BSpline`, `Bezier`) and the planar trimmed ear-clip path, meshed before and after the trimmed-freeform branch is added, together with the full tessellation-sensitive sim suite (`run-sim-suite`, STEP import, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, loft, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to the baseline (the branch is additive and reachable only by a currently-declining case); if ANY differs, the mesh branch SHALL be reverted and the trimmed freeform face SHALL keep the OCCT decline

#### Scenario: A trimmed freeform face that cannot mesh watertight additively keeps the honest decline (host + sim)

- GIVEN a native trimmed freeform face whose additive mesh path cannot both mesh it watertight (within deflection, matching the OCCT `BRepMesh` volume/area) AND leave every existing mesh byte-identical
- WHEN the engine's watertight + volume/area self-verify (against the OCCT oracle) is evaluated
- THEN the native result SHALL be DISCARDED and the reader SHALL DECLINE the patch (NULL → OCCT), no existing tessellation SHALL have been perturbed, no tolerance weakened, and no non-watertight mesh emitted — the honest deferral is reported, not faked

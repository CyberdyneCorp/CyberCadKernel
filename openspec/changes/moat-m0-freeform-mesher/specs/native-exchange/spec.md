# native-exchange

## ADDED Requirements

### Requirement: Admit a foreign trimmed B-spline surface face whose pcurves reconstruct faithfully, or DECLINE

The STEP reader SHALL admit a `B_SPLINE_SURFACE_WITH_KNOTS` surface (rational
`weights` included) bounded by a real, genuinely trimmed `EDGE_LOOP` — a foreign
trimmed B-spline/NURBS patch — as a native trimmed `Kind::BSpline` face **only when**
a faithful 2D pcurve can be reconstructed for EVERY boundary edge, so the native
trimmed-freeform mesh path (`native-tessellation`) can mesh it watertight. For each
boundary edge the reader SHALL reconstruct the pcurve on the B-spline surface by
inverting the surface at the edge's sampled points (`projectBSplineUV`): a straight-
in-`(u,v)` edge (rim / seam / isoparametric trim) SHALL become a UV `Line` through
the two projected endpoints; a curved boundary edge SHALL become a UV `B_SPLINE`
pcurve (degree and knots preserved) through densified projected samples. The reader
SHALL then run a **faithful-reconstruction guard**: re-evaluate `S_face(pcurve(t)) =
C_edge(t)` at several parameters within a **scale-relative** tolerance (never
weakened). If ANY boundary edge's pcurve fails the guard — the surface inversion did
not converge, the reconstructed pcurve does not lie on the patch within tolerance, or
the boundary gap exceeds tolerance — the reader SHALL `decline()` the face (NULL →
OCCT), exactly as it declines any other non-faithful reduction. A face that IS
admitted SHALL be subject to the engine's mandatory watertight + volume/area
self-verify against the OCCT oracle downstream; a native result that is not watertight
or off-volume SHALL be DISCARDED → OCCT. The reader SHALL remain OCCT-free
(`src/native/**`), no tolerance SHALL be weakened, and the `cc_*` ABI SHALL be
unchanged (additive reader behaviour only).

#### Scenario: A foreign trimmed B-spline face with faithful pcurves imports and meshes watertight (sim, parity)

- GIVEN a foreign STEP file carrying a `B_SPLINE_SURFACE_WITH_KNOTS` face bounded by a genuinely trimmed `EDGE_LOOP` whose boundary pcurves reconstruct faithfully, imported on a booted simulator with OCCT linked
- WHEN the reader admits the face and the native tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, and triangle envelope SHALL match the OCCT `BRepMesh_IncrementalMesh` oracle within tolerance (the foreign trimmed patch that previously declined now meshes watertight)

#### Scenario: A foreign trimmed B-spline face whose pcurve does not reconstruct faithfully declines to OCCT (sim)

- GIVEN a foreign STEP file whose trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face has at least one boundary edge whose pcurve cannot be reconstructed within the scale-relative tolerance (non-converging inversion, off-surface boundary, or beyond-tolerance gap)
- WHEN the reader evaluates the faithful-reconstruction guard
- THEN the reader SHALL `decline()` the face (NULL → OCCT), the file SHALL round-trip through OCCT unchanged, no tolerance SHALL have been weakened, and no approximate/leaky native face SHALL be emitted — the honest decline is reported

#### Scenario: The engine's self-verify discards a non-watertight admitted face (sim)

- GIVEN a foreign trimmed B-spline face that passes the pcurve guard but whose native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the engine runs its mandatory watertight + volume/area self-verify
- THEN the native result SHALL be DISCARDED and the import SHALL fall through to OCCT, so a wrong or leaky mesh is never emitted downstream

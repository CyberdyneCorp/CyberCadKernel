# native-tessellation Specification

## Purpose
TBD - created by archiving change add-native-tessellation. Update Purpose after archive.
## Requirements
### Requirement: OCCT-free, host-buildable tessellation library

The native tessellation mesher SHALL live under `src/native/tessellate/` and
SHALL include NO OCCT header in any of its translation units, so that it compiles
and unit-tests with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT
and NO simulator. The library MAY include `src/native/math` and
`src/native/topology`. The library SHALL link no OCCT; OCCT SHALL appear ONLY in
the simulator native-vs-OCCT parity test, which reuses the TEST-ONLY OCCT→native
bridge from `tests/sim/native_topology_parity.mm` to walk a real `TopoDS_Shape`
into the native model and mesh it with OCCT `BRepMesh_IncrementalMesh` for
comparison — never in the library itself. This change SHALL make no `cc_*`
signature or POD struct layout change and SHALL NOT wire the library into the
active engine.

#### Scenario: Library builds on the host without OCCT
- GIVEN the sources under `src/native/tessellate/`
- WHEN they are compiled with `clang++ -std=c++20` with no OCCT and no simulator (the `src/native/math` and `src/native/topology` headers available)
- THEN the build SHALL succeed AND no compiled translation unit SHALL include any OCCT header

#### Scenario: No ABI change and no engine wiring
- GIVEN this change applied
- WHEN the public headers and the active engine are inspected
- THEN no `cc_*` signature or POD struct layout SHALL have changed AND the native tessellation library SHALL NOT be reachable through the `cc_*` facade

### Requirement: Mesh a native Face to a triangle mesh at a given deflection

The library SHALL mesh a native `Face` (from `native-topology`) to a triangle
mesh at a requested **deflection** (chord-height) tolerance by sampling the
face's surface on a UV grid via `native-math` (`value(u,v)`, `dU`/`dV`,
`normal(u,v)`), respecting the face's parameter box `[u0,u1]×[v0,v1]`. The grid
density SHALL be chosen — from a curvature estimate refined by a sampled
chord-height probe — so that the maximum chord-height deviation of any triangle
from the true surface is at or below the requested deflection. The output SHALL
be an fp64 vertex buffer with a triangle index buffer wound consistently with the
face's outward surface normal (flipped for a `Reversed` face). A planar face
SHALL produce the minimal grid; higher-curvature regions SHALL be sampled more
densely. The mesh SHALL be deterministic for a given face and deflection.

#### Scenario: Deflection bound holds on curved faces (host)
- GIVEN a cylindrical and a spherical face built on the host with no OCCT and a requested deflection `d`
- WHEN each is meshed
- THEN every triangle's chord-height deviation from the true surface SHALL be at or below `d`

#### Scenario: A planar face meshes to the minimal exact grid (host)
- GIVEN a planar rectangular face built on the host with no OCCT
- WHEN it is meshed at any deflection
- THEN the mesh SHALL cover the face's parameter box exactly (its area equals the rectangle area within tolerance) AND every vertex SHALL lie exactly on the plane

#### Scenario: Grid density increases as deflection decreases (host)
- GIVEN a curved face and a decreasing sequence of deflections
- WHEN it is meshed at each deflection
- THEN the triangle count SHALL be non-decreasing as the deflection decreases AND the deflection bound SHALL hold at every level

#### Scenario: Native face mesh matches the OCCT BRepMesh envelope (parity)
- GIVEN a face of a representative OCCT `TopoDS_Shape` walked into the native model on a booted iOS simulator via the reused OCCT→native bridge, and a matched deflection
- WHEN the face is meshed by the native mesher and by OCCT `BRepMesh_IncrementalMesh` at the same deflection
- THEN the two meshes' axis-aligned bounding boxes AND surface areas SHALL agree within the documented tolerance (triangle count and topology are NOT compared)

### Requirement: Trim the UV grid against inner wires (holes)

The library SHALL trim the UV grid against the face's inner wires (holes) in
**parameter space** using the wires' **pcurves** (`pcurveOf(edge, face)` from
`native-topology`). Each inner wire SHALL be reconstructed as a 2D polyline from
its pcurves; grid cells whose UV footprint lies inside a hole SHALL be removed,
and cells straddling a hole boundary SHALL be re-triangulated against the clipped
region so the mesh honours the face's trimmed area. The outer wire SHALL bound
the sampled region. The trimmed mesh's surface area SHALL equal the outer-region
area minus the hole areas within the deflection tolerance, and NO triangle SHALL
lie inside a hole.

#### Scenario: A holed face omits triangles inside the hole (host)
- GIVEN a planar rectangular face with one circular inner wire (hole) built on the host with no OCCT
- WHEN it is meshed
- THEN the mesh SHALL contain NO triangle whose centroid lies inside the hole boundary AND the mesh's surface area SHALL equal `rect_area − π·r²` within the deflection tolerance

#### Scenario: Hole-boundary vertices lie on the hole curve (host)
- GIVEN the holed face meshed on the host
- WHEN the vertices adjacent to the hole are inspected
- THEN each such boundary vertex SHALL lie on the hole's wire curve within tolerance

#### Scenario: Trimmed native mesh matches OCCT on a holed face (parity)
- GIVEN a holed OCCT face walked into the native model on a booted iOS simulator via the reused OCCT→native bridge, and a matched deflection
- WHEN the face is meshed by the native mesher and by OCCT `BRepMesh_IncrementalMesh`
- THEN the two meshes' surface areas AND bounding boxes SHALL agree within the documented tolerance AND neither SHALL cover the hole region

### Requirement: Every mesh vertex lies on the true surface within tolerance

Every vertex the mesher emits SHALL be produced by evaluating the face's surface
at that vertex's `(u,v)` parameter via `native-math` `value(u,v)`, so it lies on
the true surface by construction. For any emitted vertex, its distance to the
exact face surface SHALL be at or below the deflection tolerance, and a vertex on
a face boundary SHALL additionally lie on the corresponding wire curve
(`curveOf(edge)`) within tolerance.

#### Scenario: Vertices lie on the analytic surface (host)
- GIVEN a plane, a cylinder, a sphere, and a holed plane meshed on the host with no OCCT at a deflection `d`
- WHEN each mesh vertex is evaluated against the exact surface at its `(u,v)`
- THEN every vertex's residual distance to the true surface SHALL be at or below `d`

#### Scenario: Boundary vertices lie on their wire curve (host)
- GIVEN a meshed face with an outer wire (and a hole) on the host
- WHEN the boundary vertices are evaluated against the wire's edge curves
- THEN each boundary vertex SHALL lie on its edge curve within tolerance

#### Scenario: Native mesh vertices lie on the OCCT surface (parity)
- GIVEN a face walked into the native model on a booted iOS simulator via the reused OCCT→native bridge
- WHEN the native mesh vertices are evaluated against the OCCT face surface (`BRep_Tool`)
- THEN every native mesh vertex SHALL lie on the OCCT face within the documented tolerance

### Requirement: Mesh a whole Solid by stitching shared edges into a watertight mesh

The library SHALL mesh a whole `Solid` by meshing each `Face` (via the
`native-topology` `Explorer`) and **stitching** the per-face meshes along shared
edges. For each edge shared by two faces (from the edge→face `AncestryMap`), both
adjacent faces SHALL sample the single shared edge curve (`curveOf(edge)` over
`[first, last]`) at the SAME deflection-driven parameter set so their boundary
vertices coincide, and coincident boundary vertices SHALL be welded across faces
into shared mesh vertices. Seam edges (an edge appearing twice on one periodic
face) SHALL be sampled once and referenced from both sides. For a closed solid
the resulting mesh SHALL be **watertight**: every mesh edge SHALL be shared by
exactly two triangles (a 2-manifold edge count with no naked/boundary edges). The
solid mesh SHALL preserve a per-triangle face-id tag and be deterministic.

#### Scenario: A closed solid meshes watertight (host)
- GIVEN a closed box solid and a closed cylinder solid built on the host with no OCCT
- WHEN each is meshed and its per-face meshes are stitched
- THEN every mesh edge SHALL be shared by exactly two triangles (no naked edges) AND the boundary vertices of adjacent faces SHALL be welded so there are no duplicated coincident boundary vertices beyond the tolerance weld

#### Scenario: Shared edges are sampled identically by both faces (host)
- GIVEN two faces of a solid that share one edge, meshed on the host
- WHEN the boundary vertices along the shared edge are compared between the two faces
- THEN they SHALL coincide (sampled from the same edge curve at the same parameter set) within the weld tolerance

#### Scenario: Coincident straight seams built as separate edge nodes weld exactly (host)
- GIVEN two adjacent faces whose shared STRAIGHT seam is built as two SEPARATE edge nodes with opposite vertex order (a per-turn helical-thread ruled band ↔ band or band ↔ V-cap seam), meshed on the host
- WHEN each face places its boundary vertices on that seam
- THEN both faces SHALL emit BIT-IDENTICAL 3D seam points — interpolated at the shared sample indices between the seam's two bounding vertices in a fixed (lexicographic) endpoint order, independent of build order — so the single-cell spatial weld fuses them even when a shared coordinate lands on a weld-grid cell boundary, AND the assembled solid SHALL be watertight (`boundaryEdges == 0`) at EVERY deflection in the self-verify ladder (no deflection-dependent seam sliver)

#### Scenario: Native solid mesh is watertight and matches OCCT (parity)
- GIVEN a closed OCCT solid (box, cylinder, sphere) walked into the native model on a booted iOS simulator via the reused OCCT→native bridge, and a matched deflection
- WHEN it is meshed by the native mesher and by OCCT `BRepMesh_IncrementalMesh`
- THEN the native mesh SHALL be watertight (every edge shared by exactly two triangles) AND the two meshes' bounding boxes, total surface areas, AND enclosed volumes SHALL agree within the documented tolerance

### Requirement: Mesh-derived area and volume converge to the analytic values

The mesher SHALL compute the mesh-derived surface area (the sum of triangle
areas) and, for a watertight closed solid, the enclosed volume (the signed-
tetrahedron / divergence-theorem sum over the consistently outward-wound
triangles). As the requested deflection decreases, the mesh-derived surface area
and enclosed volume SHALL converge toward the analytic (or B-rep) values: the
error SHALL decrease monotonically across a decreasing deflection sequence
(within discretization) and its limit SHALL be within tolerance of the analytic
value.

#### Scenario: Area and volume converge on analytic solids (host)
- GIVEN a plane rectangle (area `w·h`), a cylinder (area `2π·r·h`), a sphere (area `4π·r²`, volume `4/3·π·r³`), and a box solid (volume `w·d·h`) meshed on the host with no OCCT
- WHEN each is meshed across a decreasing sequence of deflections and its mesh-derived area/volume is computed
- THEN the area/volume error SHALL decrease monotonically as the deflection decreases AND its limit SHALL be within the documented tolerance of the analytic value

#### Scenario: Area and volume match the B-rep values (parity)
- GIVEN an OCCT solid walked into the native model on a booted iOS simulator via the reused OCCT→native bridge, and a matched deflection
- WHEN the native mesh-derived surface area and enclosed volume are compared against the OCCT B-rep values (`GProp` / `BRepGProp`) and against the OCCT `BRepMesh` mesh
- THEN the native mesh-derived area and volume SHALL agree with the B-rep values AND with the OCCT mesh within the documented tolerance

### Requirement: Mesh a native Torus face watertight via an additive mesh path proven byte-identical for existing kinds

The library SHALL mesh a native `Face` whose surface is of kind `Torus` (a doubly-periodic ring torus,
`u∈[0,2π]` the major/revolution angle and `v∈[0,2π]` the minor/tube angle, evaluated through the
`native-math` `Torus` `value` / `dU` / `dV` / `normal`) to a triangle mesh at a requested deflection,
respecting the deflection bound exactly as for the other analytic-curved kinds (cylinder / cone / sphere).
The torus SHALL be meshed through a **new, additive** mesh branch that reuses the EXISTING
periodic-analytic grid and canonical-seam-anchor machinery to weld BOTH the `u=0≡2π` seam and the
`v=0≡2π` seam (a ring torus has NO degenerate pole, so the seam weld is strictly simpler than the sphere's
pole-plus-seam case). The addition SHALL NOT modify the `Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`,
or `Bezier` mesh paths: every existing face SHALL mesh **byte-identically** — the same triangle counts, the
same watertight status, and the same enclosed volumes — as before this change, PROVEN across the full
tessellation-sensitive suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss,
phase3). If a clean additive torus path that keeps every existing mesh byte-identical AND meshes the torus
watertight cannot be achieved, the torus mesh path SHALL be reverted and the STEP reader SHALL keep the
honest OCCT torus decline (an OCCT-imported torus loses nothing). The library SHALL remain OCCT-free and
host-buildable, and no tolerance SHALL be weakened.

#### Scenario: A torus face meshes watertight within the deflection bound (host)
- GIVEN a native `Torus` face (major radius `R`, minor radius `r`, full period in both `u` and `v`) built on the host with no OCCT and a requested deflection `d`
- WHEN it is meshed
- THEN every triangle's chord-height deviation from the true torus SHALL be at or below `d`, the mesh SHALL be watertight (both seams welded, no pole), AND the enclosed volume SHALL converge to the analytic torus volume `2·π²·R·r²` within the deflection tolerance

#### Scenario: The additive torus path leaves every existing kind's mesh byte-identical (host + sim)
- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, `Bezier`) meshed before and after the torus mesh branch is added, and the full tessellation-sensitive sim suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to the baseline (the torus branch is additive and touches no existing mesh path); if ANY differs, the torus mesh path SHALL be reverted and the torus SHALL keep the OCCT decline

#### Scenario: A torus that cannot mesh watertight additively keeps the honest OCCT decline (host)
- GIVEN a native `Torus` face whose additive mesh path cannot both weld its seams watertight AND leave every existing mesh byte-identical
- WHEN the tessellation zero-regression proof is evaluated
- THEN the torus mesh path SHALL be reverted, the STEP reader SHALL DECLINE the torus (NULL → OCCT), and no existing tessellation SHALL have been perturbed and no tolerance weakened — the honest deferral is reported, not faked

### Requirement: Mesh a rational revolved B-spline face watertight through the existing path, or keep the honest decline

The library SHALL mesh a native `Face` whose surface is a **rational** `Kind::BSpline` (non-empty
`weights`) — specifically the revolved rational tensor-product B-spline a `SURFACE_OF_REVOLUTION` of an
`ELLIPSE` / non-rational `B_SPLINE_CURVE_WITH_KNOTS` profile reconstructs, `u`-periodic (seam at `u=0≡2π`,
`degreeU = 2`, the standard revolution weights `{1,1/√2,1,1/√2,1,1/√2,1,1/√2,1}` and knots
`{0,0,0,π/2,π/2,π,π,3π/2,3π/2,2π,2π,2π}`) tensored with the profile in `v` — to a triangle mesh at a
requested deflection, evaluating the surface through the EXISTING rational evaluators
(`math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs`), respecting the deflection bound exactly as for a
non-rational B-spline face. The mesh SHALL be produced through the EXISTING freeform B-spline mesh path with
**no modification** to the `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `BSpline`, or `Bezier` mesh
paths. If welding the `u=0≡2π` seam and closing a profile-endpoint axis pole watertight requires more than
the existing freeform path provides, the additional close SHALL be added ONLY as a **new, additive** guarded
branch that leaves every existing face meshing **byte-identically** (the same triangle counts, watertight
status, and enclosed volumes), PROVEN across the full tessellation-sensitive suite (`run-sim-suite`,
curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3). If a watertight mesh of the revolved
rational B-spline face cannot be achieved without either perturbing an existing mesh path or fabricating a
seam/pole, the tessellator SHALL NOT be changed and the STEP reader (`native-exchange`) SHALL keep the
honest OCCT decline for that revolution (an OCCT-imported revolution loses nothing). The library SHALL
remain OCCT-free and host-buildable, and no tolerance SHALL be weakened.

#### Scenario: A rational revolved B-spline face meshes watertight through the existing path (host)
- GIVEN a native rational `Kind::BSpline` face reconstructed from an ellipse-profile `SURFACE_OF_REVOLUTION` (a spheroid of revolution, `u`-periodic, with axis poles at the profile endpoints) built on the host with no OCCT and a requested deflection `d`
- WHEN it is meshed through the existing rational B-spline mesh path
- THEN every triangle's chord-height deviation from the true surface SHALL be at or below `d`, and — when the `u`-seam welds and the axis poles close — the mesh SHALL be watertight and the enclosed volume SHALL converge to the analytic revolved-solid volume within the deflection tolerance; if it does NOT close watertight, the STEP reader SHALL DECLINE (NULL → OCCT) rather than emit a leaky mesh

#### Scenario: The general-revolution mapping leaves every existing kind's mesh byte-identical (host + sim)
- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `BSpline`, `Bezier`) meshed before and after this change, and the full tessellation-sensitive sim suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to the baseline (the tessellator is preferred untouched; any added branch is additive and touches no existing mesh path); if ANY differs, the mesher change SHALL be reverted and the revolution SHALL keep the OCCT decline

#### Scenario: A revolved B-spline that cannot mesh watertight without perturbation keeps the honest decline (host)
- GIVEN a native rational `Kind::BSpline` revolved face whose `u`-seam or axis pole cannot be closed watertight without either modifying an existing mesh path or fabricating a seam/pole
- WHEN the watertight self-verify is evaluated
- THEN the tessellator SHALL NOT be changed, the STEP reader SHALL DECLINE the revolution (NULL → OCCT), and no existing tessellation SHALL have been perturbed and no tolerance weakened — the honest deferral is reported, not faked

### Requirement: Weld a shared CURVED edge watertight at any deflection via ONE canonical per-edge discretization consumed by both incident faces

The library SHALL discretize a genuinely **curved** edge (3‑D curvature above zero —
a circle, ellipse, or free‑form seam such as the degree‑2 Bézier boolean trace or a
bowl‑lid parabola) that is shared by two faces through **separate edge nodes** into ONE
**canonical** per‑edge discretization — a single fraction list AND a single 3‑D polyline
`C_edge(t)` — that BOTH incident faces consume, so both faces place BIT‑IDENTICAL
boundary points at every sample and the seam welds watertight at ANY requested
deflection. The sharing SHALL be keyed by the edge's order‑independent, quantized
**endpoint pair PLUS a curve‑identity discriminator** (a quantized interior sample, e.g.
the 3‑D midpoint), so two GEOMETRICALLY DIFFERENT curves between the same endpoints never
collapse to one canonical record. This generalises the existing straight‑edge
endpoint‑keyed single‑sampling (the segment‑count sharing and canonical straight anchors)
to curved edges.

The addition SHALL be strictly additive and reachable ONLY by a curved edge that is (a)
NOT straight in 3‑D and (b) NOT already shared through a single `TShape` node. An edge
shared through ONE `TShape` node (every analytic primitive — a cylinder cap and side
share the circle node) SHALL keep its existing per‑node discretization, and a straight
separate‑node edge SHALL keep its existing endpoint‑keyed count sharing and canonical
straight anchors, so every existing face SHALL mesh **byte‑identically** — the same
triangle counts, the same watertight status, and the same enclosed volumes — PROVEN
across the full tessellation‑sensitive suite (`run-sim-suite`, STEP import, curved‑fillet,
curved‑chamfer, curved‑boolean, wrap‑emboss, loft, phase3) and a per‑surface‑kind
snapshot diffed against the base. The canonical‑curved path SHALL NOT modify the shared
segment‑count sizing, the curve evaluators, the three face‑mesh arms (`structuredGrid`,
`earClipMesh`, `trimmedFreeformMesh`), the boundary flattener, or the spatial weld. The
library SHALL remain OCCT‑free and host‑buildable, and no tolerance SHALL be weakened. If
a clean additive path that keeps every existing mesh byte‑identical AND welds the curved
seam at every deflection cannot be achieved, the change SHALL be reverted and the freeform
boolean SHALL keep the honest OCCT decline (an OCCT‑handled boolean loses nothing).

#### Scenario: A freeform boolean CUT welds watertight across a deflection sweep at the closed-form volume (host, no OCCT)

- GIVEN the bowl‑lidded convex‑quad operand (a degree‑2 Bézier top genuinely trimmed by
  the quad, four planar side walls, a planar bottom) and the half‑space CUT `x ≤ 0`,
  built on the host with no OCCT
- WHEN it is meshed through the canonical curved‑edge single‑sampling at each deflection
  in the sweep `{0.03, 0.02, 0.01, 0.008, 0.004, 0.002}`
- THEN the CUT solid SHALL be watertight at EVERY deflection (no watertight↔NotWatertight
  oscillation), AND its enclosed volume SHALL lie within the deflection band of the
  closed‑form value `∫∫_{Q∩{x≤0}} (H0 + a·(x²+y²)) dA`, converging as deflection → 0

#### Scenario: A freeform boolean COMMON welds watertight across the same sweep (host, no OCCT)

- GIVEN the same bowl operand and half‑space COMMON, built on the host with no OCCT
- WHEN it is meshed at each deflection in the sweep `{0.03, 0.02, 0.01, 0.008, 0.004, 0.002}`
- THEN the COMMON solid SHALL be watertight at EVERY deflection AND its enclosed volume
  SHALL lie within the deflection band of the closed‑form COMMON value

#### Scenario: The bowl-lid curved seam welds because both faces share ONE canonical polyline (host)

- GIVEN a genuinely curved edge (a bowl‑lid parabola — `(x,y)` linear × `z` quadratic)
  shared by the curved Bézier top face and a planar side‑wall face as SEPARATE edge nodes
- WHEN both faces are meshed with a shared edge cache at any deflection
- THEN both faces SHALL read the SAME canonical fraction list and the SAME canonical 3‑D
  polyline, place BIT‑IDENTICAL boundary points at every sample, and the two faces SHALL
  weld into a watertight seam without relying on the spatial weld tolerance bridging two
  independent samplings

#### Scenario: Two different curved edges between the same endpoints keep separate discretizations (host)

- GIVEN two geometrically DIFFERENT curved edges (for example a minor and a major arc, or
  two distinct blend seams) that happen to share the same two endpoints
- WHEN each is discretized through the canonical curved‑edge cache
- THEN the curve‑identity discriminator (the quantized interior sample) SHALL place them
  in DIFFERENT canonical records, so neither edge's sampling is corrupted by the other

#### Scenario: Every existing surface kind meshes byte-identically after the addition (host + sim)

- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`,
  `Torus`, bare‑periodic `BSpline`, `Bezier`), a planar trim, and a loft side wall,
  meshed before and after the canonical curved‑edge path is added, together with the full
  tessellation‑sensitive suite (`run-sim-suite`, STEP import, curved‑fillet,
  curved‑chamfer, curved‑boolean, wrap‑emboss, loft, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the
  pre‑change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to
  the baseline (the canonical curved path is reachable only by a curved separate‑node
  shared edge that today welds coincidentally); if ANY differs, the change SHALL be
  reverted and the freeform boolean SHALL keep the OCCT path

#### Scenario: The native freeform CUT matches the OCCT oracle at multiple deflections, or declines honestly (sim)

- GIVEN the freeform CUT built natively and its OCCT `BRepAlgoAPI_Cut` + `BRepMesh` oracle,
  on the booted simulator with OCCT linked
- WHEN the native solid is meshed at multiple deflections (at least `0.01` and one finer)
  and compared to the oracle
- THEN at EACH deflection the native volume, area, watertight status, and triangle envelope
  SHALL match OCCT within tolerance — OR the reader SHALL decline and the file SHALL
  round‑trip through OCCT unchanged (both PASS); a non‑watertight native mesh SHALL never be
  emitted and no tolerance SHALL be weakened


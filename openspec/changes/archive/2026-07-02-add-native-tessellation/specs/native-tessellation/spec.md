# native-tessellation

## ADDED Requirements

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

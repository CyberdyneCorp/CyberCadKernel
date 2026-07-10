# native-tessellation

## ADDED Requirements

### Requirement: Render-quality display mesh post-processed from the correctness mesh, OCCT-free

The library SHALL provide a render-quality DISPLAY mesh derived from the existing
correctness tessellation, in a NEW module `src/native/render/display_mesh.h`
(namespace `cybercad::native::render`), header-only, that includes ONLY
`src/native/math` and `src/native/tessellate/mesh.h` and NO OCCT symbol. The module
SHALL be a pure CONSUMER of the correctness `Mesh` (vertices + triangles): it SHALL
NOT mesh a B-rep, SHALL NOT modify the tessellator, and the files under
`src/native/tessellate/**` SHALL remain byte-identical. The display mesh SHALL add
per-vertex smooth normals with crease-angle hard edges, and OPTIONALLY texture
coordinates and a lower level of detail. Every DISPLAY POSITION before decimation
SHALL be a verbatim source vertex (moved by no operation); a decimated vertex SHALL
move only within a Hausdorff budget. An empty input mesh SHALL yield an empty
display mesh (HONEST DECLINE), never a fabricated result.

#### Scenario: Module builds on the host without OCCT
- GIVEN the sources under `src/native/render/`
- WHEN they are compiled with `clang++ -std=c++20` with no OCCT and no simulator
- THEN the build SHALL succeed AND no translation unit SHALL include any OCCT header

#### Scenario: The correctness tessellator is unchanged
- GIVEN this change applied
- WHEN `src/native/tessellate/**` is diffed against the base
- THEN the diff SHALL be empty AND `cc_tessellate` / `cc_face_meshes` behaviour SHALL be unchanged AND the existing mesh-hash suites SHALL stay green

#### Scenario: Empty input declines
- GIVEN an empty correctness mesh (no triangles)
- WHEN `buildDisplayMesh` is called
- THEN the result SHALL have zero vertices and zero triangles

### Requirement: Smooth per-vertex normals with crease-angle hard edges

The display mesh SHALL compute per-vertex normals as the angle-weighted (and
area-weighted) average of the incident triangle face-normals, averaged ONLY across
triangles whose shared-edge dihedral is BELOW a caller-supplied crease angle. A
vertex incident to more than one such smoothing group SHALL be SPLIT (duplicated,
one display vertex per group with its own normal), so an edge whose dihedral
EXCEEDS the crease angle stays a HARD edge. Raising the crease angle SHALL not
increase the number of split vertices.

#### Scenario: Cylinder lateral wall shades smooth and exactly radial (host, no OCCT)
- GIVEN a cylinder of radius `R` about the Z axis, meshed at deflection `d`, with a crease angle of 30°
- WHEN the display mesh is built
- THEN the wall copy of each ring vertex (normal ⊥ Z) SHALL equal the analytic surface normal `(x,y,0)/R` within `1e-6` AND all wall normals SHALL point outward

#### Scenario: Cylinder cap↔wall circle is a crease and splits (host, no OCCT)
- GIVEN the same cylinder with a 30° crease angle (the 90° cap↔wall seam exceeds it)
- WHEN the display mesh is built
- THEN each cap↔wall ring point SHALL have at least two coincident display copies, one with a near-axial normal (`|nz| > 0.9`) and one with a near-radial normal (`|nz| < 0.1`)

#### Scenario: Sphere is all-smooth and converges to the analytic normal (host, no OCCT)
- GIVEN a sphere of radius `R` (no crease anywhere) with a 30° crease angle
- WHEN the display mesh is built
- THEN no vertex SHALL be split (display vertex count equals source vertex count) AND the maximum deviation of a display normal from the analytic radial `P/R` SHALL DECREASE as the deflection is refined AND SHALL be within a small multiple of the deflection

#### Scenario: Box has 24 split corner normals and 6 face normals (host, no OCCT)
- GIVEN an axis-aligned box with a 30° crease angle (every edge is a 90° crease)
- WHEN the display mesh is built
- THEN every display normal SHALL be axis-aligned AND each of the 8 geometric corners SHALL split into exactly 3 coincident display vertices (24 total) AND there SHALL be exactly 6 distinct face normals

#### Scenario: Raising the crease angle removes splits (host, no OCCT)
- GIVEN a cylinder whose only crease is the 90° cap↔wall seam
- WHEN the crease angle is raised above 90° (e.g. 120°)
- THEN the display vertex count SHALL be strictly less than at 30° AND SHALL equal the source vertex count (no split remains)

### Requirement: Optional texture coordinates in the unit range, seam-consistent

When UVs are requested, the display mesh SHALL assign each vertex a `(u,v)` in
`[0,1]` by a dominant-axis planar (box) projection normalised over the mesh extent.
Two vertices with the same position and the same dominant projection axis SHALL
receive the same UV (seam-consistent). When UVs are NOT requested the display mesh
SHALL carry no UVs.

#### Scenario: Box UVs are in range and seam-consistent (host, no OCCT)
- GIVEN a box display mesh built with `wantUVs` set
- WHEN the UVs are inspected
- THEN every `u` and `v` SHALL lie in `[0,1]` AND the UVs SHALL span the box (max u and max v both exceed 0.5) AND any two vertices sharing a position and a dominant axis SHALL have identical UVs

### Requirement: Optional level-of-detail decimation within a Hausdorff bound

When a positive LOD target is requested, the display mesh SHALL be decimated by
quadric-error edge collapse toward that triangle target. Boundary, non-manifold,
and crease (position-shared) vertices SHALL be LOCKED and never removed, preserving
silhouettes and hard edges. A collapse SHALL be rejected if it would flip a
triangle or push the surface beyond a Hausdorff budget derived from the deflection.
The triangle count SHALL be reduced but SHALL never over-collapse below the target;
every surviving vertex SHALL stay within the Hausdorff budget of the source
surface. A non-positive LOD target SHALL leave the triangle count unchanged.

#### Scenario: LOD reduces triangles within the Hausdorff bound (host, no OCCT)
- GIVEN a sphere display mesh with a LOD target of half the source triangle count and a Hausdorff budget of `8·deflection`
- WHEN the display mesh is built
- THEN the triangle count SHALL be strictly less than the source AND SHALL be at least the target AND every surviving vertex SHALL lie within the budget of the analytic sphere

#### Scenario: A tighter budget throttles the collapse (host, no OCCT)
- GIVEN the same sphere with an aggressive target but a tiny Hausdorff budget
- WHEN the display mesh is built
- THEN the collapse SHALL stop early (more triangles survive than the target) AND every survivor SHALL still lie within the tiny budget

#### Scenario: LOD disabled leaves the mesh full-resolution (host, no OCCT)
- GIVEN a display mesh built with `lodTargetTris <= 0`
- WHEN the triangle count is compared to the source
- THEN it SHALL equal the source triangle count

### Requirement: cc_display_mesh facade op (ADDITIVE)

The library SHALL expose an ADDITIVE `cc_display_mesh(body, deflection,
creaseAngleDeg, lodTargetTris, wantUVs, out)` and `cc_display_mesh_free`, plus a
`CCDisplayMesh` POD (positions, unit normals, optional uvs, triangles). These SHALL
NOT be part of the mirrored `KernelBridgeAPI.h` ABI and SHALL make NO change to any
existing `cc_*` signature or POD layout. The op SHALL consume the active engine's
tessellation (serving under both the OCCT and native engines) and post-process it
OCCT-free. It SHALL return the triangle count on success and 0 with `*out` zeroed
and `cc_last_error` set on an empty/unknown body or a mesh that cannot be produced —
never a fabricated mesh.

#### Scenario: ABI is additive-only
- GIVEN this change applied
- WHEN the public header is compared to the mirrored `KernelBridgeAPI.h`
- THEN no existing `cc_*` signature or POD layout SHALL have changed AND `cc_display_mesh` / `cc_display_mesh_free` / `CCDisplayMesh` SHALL be new additive symbols AND `test_abi` SHALL stay green

#### Scenario: Empty / unknown body declines
- GIVEN an invalid or empty `CCShapeId`
- WHEN `cc_display_mesh` is called
- THEN it SHALL return 0 AND `*out` SHALL be zeroed AND `cc_last_error` SHALL be set

#### Scenario: Base display mesh is watertight-consistent with cc_tessellate (host + sim)
- GIVEN a closed solid whose `cc_tessellate` mesh is watertight
- WHEN `cc_display_mesh` is built without LOD and its split vertices are folded back by position
- THEN the folded mesh SHALL be watertight AND its triangle count SHALL equal the `cc_tessellate` triangle count

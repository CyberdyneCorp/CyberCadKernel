# native-meshing

## ADDED Requirements

### Requirement: Tetrahedral volume meshing (C3D4 / C3D10, TetGen-backed, optional)

`cc_tet_mesh(...)` and `cc_tet_mesh_surface(...)` SHALL each produce a tetrahedral
volume mesh of a closed body — the first by tessellating the body's surface
(honouring `deflection`) then filling the
resulting piecewise-linear complex, the second by filling a raw closed triangle
surface directly with no OCCT — both funnelling through one shared surface-fill path.
The mesh SHALL be returned as a `CCTetMesh` POD (flat `nodes` = x,y,z triplets, flat
`elements` = `nodesPerElement` zero-based indices per tetrahedron, `order`), owned by
the caller and freed with `cc_tet_mesh_free`. When `opts.order` selects linear
(`4`) the elements SHALL be **C3D4** with 4 nodes each; when it selects quadratic
(`10`, the default) the elements SHALL be **C3D10** whose six mid-edge nodes are
constructed NATIVELY (the backend produces LINEAR tetrahedra only — no `-o2`),
placed at the exact edge midpoints, in CalculiX `shape10tet` order
(`5=mid(1,2) 6=mid(2,3) 7=mid(3,1) 8=mid(1,4) 9=mid(2,4) 10=mid(3,4)`), and
deduplicated per shared edge so the mesh stays watertight. Every emitted tetrahedron
SHALL be corner-ordered to have a **positive signed volume**
`V = (1/6) det[c2−c1, c3−c1, c4−c1]` so the FE Jacobian is positive. The mesh SHALL be
**watertight**: interior faces SHALL be shared by exactly two tetrahedra and the
boundary faces (shared by one) SHALL form a closed 2-manifold covering the input
surface. When no target element size is requested the input boundary triangulation
SHALL be preserved; when a target element size is requested the backend MAY refine
the boundary to honor the requested size (the max-volume cap cannot be met while the
input boundary is frozen). The backend MAY introduce interior and (on the sized path)
boundary Steiner points, so `nodeCount` MAY exceed the input vertex count. On any failure (open / non-watertight / degenerate surface, empty or
invalid input, or backend error) the call SHALL return an empty `CCTetMesh` (null
buffers, zero counts) with `cc_last_error` set and SHALL NOT crash or return a partial
mesh. The volume mesher is backed by the **optional, external, AGPL-3.0** TetGen
dependency and is available only when the kernel is built with `CYBERCAD_HAS_TETGEN`
ON (see the license-gating requirement).

#### Scenario: A closed surface yields a watertight C3D4 mesh with positive Jacobians (host, TetGen ON)

- GIVEN a closed unit-cube triangle surface (8 vertices, 12 triangles) and a kernel built with `CYBERCAD_HAS_TETGEN=ON`
- WHEN `cc_tet_mesh_surface(...)` is called with `opts.order = 4`
- THEN the call SHALL return a non-empty `CCTetMesh` with `nodesPerElement == 4` AND every tetrahedron SHALL have a positive signed volume and a positive scaled Jacobian AND every interior face SHALL be shared by exactly two tetrahedra while the boundary faces form a closed 2-manifold surface (every boundary edge shared by exactly two boundary faces)

#### Scenario: A C3D10 mesh carries native mid-edge nodes in CalculiX order (host, TetGen ON)

- GIVEN the same closed unit-cube surface and a kernel built with `CYBERCAD_HAS_TETGEN=ON`
- WHEN `cc_tet_mesh_surface(...)` is called with `opts.order = 10`
- THEN the call SHALL return a `CCTetMesh` with `nodesPerElement == 10` AND for every element the six mid-edge nodes SHALL lie at the exact midpoints of edges `(1,2),(2,3),(3,1),(1,4),(2,4),(3,4)` in that CalculiX `shape10tet` order (within `1e-12`) AND every element SHALL retain a positive signed volume of its four corners

#### Scenario: The tetrahedra fill the source volume with no gaps or overlaps (host, TetGen ON)

- GIVEN a closed triangle surface of a solid of known enclosed volume on a kernel built with `CYBERCAD_HAS_TETGEN=ON`
- WHEN it is meshed via `cc_tet_mesh_surface(...)`
- THEN the sum of the absolute signed volumes of all tetrahedra SHALL equal the enclosed volume of the source surface (computed by the divergence theorem) within a small relative tolerance (`1e-9`), demonstrating a gap-free, overlap-free fill

#### Scenario: A smaller target element size refines the interior (host, TetGen ON)

- GIVEN the closed unit-cube triangle surface on a kernel built with `CYBERCAD_HAS_TETGEN=ON`
- WHEN it is meshed twice via `cc_tet_mesh_surface(...)` with a coarse and a fine `target_element_size` (the finer being several times smaller)
- THEN the finer mesh SHALL contain strictly more elements than the coarser mesh AND the mean tetrahedron volume of the finer mesh SHALL not exceed the requested max-volume cap `h³/(6√2)` AND total volume conservation SHALL still hold, demonstrating that `target_element_size` genuinely controls resolution and is never silently ignored

### Requirement: Native mesh quality metrics and reporting

`cc_mesh_quality(mesh, min_scaled_jacobian)` SHALL compute a `CCQualityReport` over a
`CCTetMesh` using **pure geometry** over the four corner nodes of each element,
referencing no OCCT and no TetGen, so that it is available in EVERY build regardless
of `CYBERCAD_HAS_TETGEN` and applies identically to C3D4 and C3D10 meshes (a C3D10
element scores the same as its four corners). For each element it SHALL compute the
signed volume, the six dihedral angles (in degrees), a scaled Jacobian normalized so
a regular tetrahedron scores exactly `1`, a sliver approaches `0`, and an inverted
element is `< 0`, and an aspect ratio normalized so a regular tetrahedron scores `1`
and grows without bound as the element degenerates; each formula SHALL be a
documented, cited, defensible metric. The report SHALL aggregate the minimum and
maximum dihedral angle, the minimum and mean scaled Jacobian, and the maximum aspect
ratio over the whole mesh, and SHALL list in `flagged_elements` (with count
`elements_below_threshold`) the ids of every element whose scaled Jacobian is below
`min_scaled_jacobian`. The report SHALL set `valid = 0` (and `cc_last_error`) on empty
or degenerate input and SHALL be freed with `cc_quality_report_free`. Field names
(`min/max_dihedral_angle`, `min/mean_scaled_jacobian`, `max_aspect_ratio`,
`elements_below_threshold`, `flagged_elements`) SHALL mirror the CalculiX++
`QualityReport` contract.

#### Scenario: A regular tetrahedron scores the analytic baseline (host, any build)

- GIVEN a single regular tetrahedron with corners `(1,1,1),(1,-1,-1),(-1,1,-1),(-1,-1,1)` in a `CCTetMesh`, in any build (TetGen ON or OFF)
- WHEN `cc_mesh_quality(mesh, 0.1)` is called
- THEN `valid` SHALL be non-zero AND all six dihedral angles SHALL be `70.5288°` (within `1e-3`) so `min_dihedral_angle == max_dihedral_angle` AND `min_scaled_jacobian == mean_scaled_jacobian == 1.0` (within `1e-9`) AND `max_aspect_ratio == 1.0` (within tolerance) AND the signed volume SHALL be positive AND `elements_below_threshold` SHALL be `0`

#### Scenario: A sliver element is flagged below the threshold (host, any build)

- GIVEN a two-element `CCTetMesh` containing one well-shaped tetrahedron and one near-coplanar sliver whose scaled Jacobian is near `0`, in any build
- WHEN `cc_mesh_quality(mesh, min_scaled_jacobian)` is called with a threshold above the sliver's scaled Jacobian
- THEN `valid` SHALL be non-zero AND `elements_below_threshold` SHALL be at least `1` AND `flagged_elements` SHALL contain the sliver's element id AND `min_scaled_jacobian` SHALL be near `0`

#### Scenario: An empty or degenerate mesh reports invalid (host, any build)

- GIVEN an empty `CCTetMesh` (zero elements)
- WHEN `cc_mesh_quality(mesh, 0.1)` is called
- THEN the report SHALL set `valid = 0` AND set a non-empty `cc_last_error` AND SHALL NOT crash

### Requirement: License gating (AGPL TetGen optional / external, MIT default unaffected)

The TetGen backend SHALL be an **optional, external, AGPL-3.0** dependency, OFF by
default, integrated so that the default MIT build compiles and links ZERO AGPL code.
TetGen sources SHALL live externally (at `/home/leonardo/work/tetgen`) and SHALL be
referenced by absolute path only — never copied, vendored, or committed into the
repository — and the external build output (`/build-tet/`) SHALL be git-ignored. The
sole AGPL-consuming translation unit (`src/native/mesh/tet_mesher.cpp`) SHALL be
excluded from the default source glob and compiled only under the `CYBERCAD_HAS_TETGEN`
option, with the TetGen include directory and the `TETLIBRARY` define scoped to that
one source so no other translation unit can reach `tetgen.h`. When `CYBERCAD_HAS_TETGEN`
is OFF, `cc_tet_mesh` and `cc_tet_mesh_surface` SHALL return an empty `CCTetMesh` and
set `cc_last_error` to a clear "tet meshing unavailable" message that names the
optional, external AGPL backend — never crashing and never linking AGPL code — while
the always-on native `cc_mesh_quality` continues to work. Documentation SHALL state
honestly that turning the flag ON and shipping a closed-source application that links
TetGen requires a TetGen commercial license, and that wiring CalculiX++'s `CadMesher`
to these entry points is a follow-up outside this change.

#### Scenario: With TetGen OFF the mesher reports unavailable and links no AGPL code (host, default MIT build)

- GIVEN a kernel built in the default configuration with `CYBERCAD_HAS_TETGEN` OFF
- WHEN `cc_tet_mesh_surface(...)` (or `cc_tet_mesh(...)`) is called with valid input
- THEN the call SHALL return an empty `CCTetMesh` (null buffers, zero counts) AND set a non-empty `cc_last_error` naming the optional external AGPL TetGen backend AND SHALL NOT crash AND the build SHALL contain no TetGen (`tetgen.*` / `predicates.*`) object code

#### Scenario: Quality reporting still works with the backend OFF (host, default MIT build)

- GIVEN the same default build with `CYBERCAD_HAS_TETGEN` OFF and a hand-built `CCTetMesh` (e.g. one regular tetrahedron)
- WHEN `cc_mesh_quality(mesh, 0.1)` is called
- THEN the report SHALL be computed normally with `valid` non-zero (quality is TetGen-independent) even though the volume mesher is unavailable in this build

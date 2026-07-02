# gpu-tessellation

## ADDED Requirements

### Requirement: Additive GPU-tessellation toggle
The library SHALL expose two additive C-ABI entry points — `void cc_set_gpu_tessellation(int enabled)` and `int cc_gpu_tessellation_enabled(void)` — that turn the GPU surface-evaluation tessellation path on or off at runtime. GPU tessellation SHALL default to OFF. These are the ONLY additions to the public C ABI; the `cc_tessellate` and `cc_face_meshes` signatures SHALL be unchanged, and no existing `cc_*` signature or POD struct layout SHALL change.

#### Scenario: Toggle defaults off and round-trips
- GIVEN a freshly loaded kernel with no prior GPU-tessellation call
- WHEN `cc_gpu_tessellation_enabled()` is queried
- THEN it SHALL return `0` (OFF by default)
- AND after `cc_set_gpu_tessellation(1)` it SHALL return `1`, and after `cc_set_gpu_tessellation(0)` it SHALL return `0`

#### Scenario: Toggle is a no-op on a non-Metal build
- GIVEN a build compiled WITHOUT `CYBERCAD_HAS_METAL`
- WHEN `cc_set_gpu_tessellation(1)` is called
- THEN `cc_gpu_tessellation_enabled()` SHALL return `0` and `cc_tessellate` SHALL behave exactly as the OCCT-only path

#### Scenario: ABI addition is source-compatible
- GIVEN the host app previously linked the kernel
- WHEN it links the version with the two new entry points
- THEN `cc_tessellate` / `cc_face_meshes` SHALL keep their existing signatures AND the ABI contract test (`tests/test_abi.cpp`) SHALL still pass

### Requirement: GPU tessellation OFF is identical to today
When GPU tessellation is OFF (the default), `cc_tessellate` and `cc_face_meshes` SHALL produce byte-for-byte the same mesh as the current OCCT `BRepMesh_IncrementalMesh` path — same vertex buffer, same triangle buffer, same face traversal and merge order. The GPU code path SHALL NOT run when the toggle is OFF.

#### Scenario: OFF path matches the OCCT baseline suite
- GIVEN GPU tessellation is OFF
- WHEN the full OCCT runtime suite (`scripts/run-sim-suite.sh`) runs
- THEN it SHALL pass 221/221 unchanged, identical to the pre-change baseline

#### Scenario: OFF path never dispatches the GPU
- GIVEN GPU tessellation is OFF
- WHEN `cc_tessellate` runs for any body
- THEN no `ComputeKind::SurfaceEval` dispatch SHALL be issued AND the resulting mesh SHALL equal the OCCT-only mesh exactly

### Requirement: Per-face GPU eligibility with mandatory OCCT fallback
When GPU tessellation is ON, the tessellation path SHALL classify each face independently and route it to the GPU surface-evaluation path ONLY when the face is provably equivalent to a GPU-evaluated regular `(u,v)` grid. A face SHALL be GPU-eligible only when ALL of the following hold: it has a single outer wire, it has NO inner wires (no holes), its 2D boundary equals the `BRepTools::UVBounds` rectangle within tolerance (an untrimmed rectangular patch), and its surface is representable by the `gpu_surface_eval` module (`cyber::metal::SurfaceDef`) after conversion via `Geom_BSplineSurface` / `BRepBuilderAPI_NurbsConvert` / Bézier decomposition, with the surface degree within `cyber::metal::kMaxSurfaceDegree`. EVERY face that is not eligible — holes/inner wires, trimmed (non-rectangular UV boundary), unsupported surface type, or a rational surface the module cannot represent — SHALL fall back to OCCT `BRepMesh_IncrementalMesh` for that face. When eligibility cannot be established, the face SHALL fall back to OCCT.

#### Scenario: Trimmed or holed faces fall back to OCCT
- GIVEN GPU tessellation is ON and a body whose faces include a face with an inner wire (hole) and a face whose UV boundary is not the `BRepTools::UVBounds` rectangle
- WHEN `cc_tessellate` runs
- THEN those faces SHALL be tessellated by OCCT `BRepMesh_IncrementalMesh`, not by the GPU grid path

#### Scenario: Unsupported or unrepresentable surfaces fall back to OCCT
- GIVEN GPU tessellation is ON and a face whose surface cannot be represented by the `gpu_surface_eval` module (unsupported type, degree above `kMaxSurfaceDegree`, or a rational form the module rejects)
- WHEN `cc_tessellate` runs
- THEN that face SHALL fall back to OCCT `BRepMesh_IncrementalMesh`

#### Scenario: Only untrimmed rectangular patches take the GPU path
- GIVEN GPU tessellation is ON and a face with a single outer wire, no holes, a UV boundary equal to its `BRepTools::UVBounds` rectangle within tolerance, and a surface convertible to a supported `SurfaceDef`
- WHEN `cc_tessellate` runs
- THEN that face SHALL be tessellated on the GPU surface-evaluation grid path

### Requirement: GPU-path faces lie on the true face and match the OCCT mesh envelope
For a GPU-eligible face, the mesh produced from the GPU-evaluated grid SHALL have every vertex lying on the true trimmed face within a documented fp32 tolerance, and its axis-aligned bounding box and surface area SHALL match those of the OCCT `BRepMesh_IncrementalMesh` mesh of the same face within the documented fp32 tolerance. Triangle topology need NOT match OCCT (the GPU path emits a regular grid triangulation); equivalence is asserted by the on-face, bbox, and area checks, not by triangle count.

#### Scenario: GPU-path face vertices lie on the true face
- GIVEN GPU tessellation is ON, on a booted iOS simulator whose Metal device is "Apple iOS simulator GPU", and a GPU-eligible face
- WHEN the face is tessellated through the GPU grid path
- THEN every mesh vertex SHALL lie on the exact face surface within the documented fp32 tolerance (evaluated against the fp64 OCCT surface)

#### Scenario: GPU-path and OCCT-path meshes share bbox and area
- GIVEN GPU tessellation is ON and a GPU-eligible face
- WHEN the face is tessellated once via the GPU grid path and once via OCCT `BRepMesh_IncrementalMesh` at the same deflection
- THEN the two meshes' axis-aligned bounding boxes AND surface areas SHALL agree within the documented fp32 tolerance

### Requirement: GPU and OCCT faces stitch into one mesh
When GPU tessellation is ON, `cc_tessellate` SHALL merge the GPU-path faces and the OCCT-fallback faces into a single `CCMesh` (and `cc_face_meshes` SHALL keep one `CCFaceMesh` per face id). Coincident boundary vertices between adjacent faces SHOULD be welded within tolerance for watertightness; if a build does not weld them, the seam handling SHALL be documented in the design. The merged mesh SHALL preserve the existing face-id tagging and traversal order used by the OCCT-only path.

#### Scenario: Mixed GPU/OCCT body yields one coherent mesh
- GIVEN GPU tessellation is ON and a body with both GPU-eligible faces and OCCT-fallback faces
- WHEN `cc_tessellate` runs
- THEN it SHALL return a single `CCMesh` whose bounding box and total surface area match the OCCT-only mesh within the documented fp32 tolerance
- AND `cc_face_meshes` SHALL still return exactly one `CCFaceMesh` per face id in the same order as the OCCT-only path

### Requirement: Metal-guarded, fp32 display-only GPU path
The GPU tessellation path SHALL be compiled only under `#ifdef CYBERCAD_HAS_METAL` so the non-Metal OCCT build still compiles and links. GPU surface evaluation SHALL be fp32 and confined to the display mesh; the exact fp64 modeling core (surface definitions, topology, exact queries) SHALL be untouched by the GPU path.

#### Scenario: Non-Metal build compiles without the GPU path
- GIVEN a build without `CYBERCAD_HAS_METAL`
- WHEN the kernel is compiled and linked
- THEN it SHALL build successfully with the GPU tessellation path excluded AND `cc_tessellate` SHALL run the OCCT-only path

#### Scenario: GPU path never touches the fp64 core
- GIVEN GPU tessellation is ON
- WHEN a GPU-eligible face is tessellated
- THEN only fp32 sample points/normals SHALL come from the GPU AND the fp64 surface definition, topology, and exact queries (`cc_mass_properties`, `cc_bounding_box`) SHALL be unchanged from the OCCT-only path

### Requirement: GPU surface-grid evaluation
The library SHALL evaluate a NURBS/Bezier surface on a parameter grid `(u_i, v_j)`
on the compute backend (as `ComputeKind::SurfaceEval`), producing an fp32 grid of
positions and an fp32 grid of surface normals, with results matching a CPU
reference evaluation within a documented fp32 tolerance.

#### Scenario: GPU surface grid matches the CPU reference
- GIVEN a NURBS/Bezier surface and a parameter grid, on a booted iOS simulator
  whose Metal device is "Apple iOS simulator GPU"
- WHEN the surface is evaluated on the GPU and on the CPU reference for the same grid
- THEN every GPU position and normal SHALL equal the corresponding CPU-reference
  value within the documented fp32 tolerance

### Requirement: Topology stays on the CPU
The GPU SHALL only supply per-sample numeric fields (points and normals); all
topology — face selection/eligibility, trimming, grid-to-triangle connectivity, and
cross-face stitching — SHALL be computed on the CPU (OCCT plus the grid triangulator).
The GPU SHALL never decide connectivity or trimming.

#### Scenario: Topology decisions never run on the GPU
- GIVEN GPU tessellation is ON and a face tessellated through the GPU evaluation path
- WHEN the mesh is produced
- THEN eligibility classification, trimming, grid-to-triangle connectivity, and
  stitching SHALL have been computed on the CPU AND the GPU SHALL have contributed
  only sample points and normals

### Requirement: GPU per-vertex mesh normals
The library SHALL compute smooth per-vertex mesh normals as a GPU post-processing
pass (`ComputeKind::MeshPostProcess`) — weighted accumulation of incident face
normals, then normalization — with results matching a CPU reference within a
documented fp32 tolerance.

#### Scenario: GPU per-vertex normals match the CPU reference
- GIVEN a vertex/index mesh on the iOS simulator GPU
- WHEN per-vertex normals are computed on the GPU and on the CPU reference
- THEN each GPU per-vertex normal SHALL equal the CPU-reference normal within the
  documented fp32 tolerance

### Requirement: Backend routing with CPU fallback
Surface evaluation and mesh-normal passes SHALL run on the Metal backend when it
is active and on the CPU backend otherwise, selected through the compute-backend
registry and precision guard, with no `cc_*` signature change; the exact fp64
surface definitions and topology SHALL always remain on the CPU.

#### Scenario: Same grid with or without a Metal backend
- GIVEN a GPU-eligible face's grid evaluated with a Metal backend active and again
  with only the CPU backend active
- WHEN the surface-evaluation pass runs in each case
- THEN both SHALL produce the same point and normal grid within the documented fp32
  tolerance AND the `cc_tessellate` signature SHALL be unchanged

### Requirement: Deterministic GPU tessellation
GPU tessellation SHALL be reproducible: using a fixed grid decomposition and a
fixed normal-accumulation order, repeated GPU runs on the same input SHALL produce
identical output.

#### Scenario: Repeated GPU tessellation runs are reproducible
- GIVEN the same surface and grid evaluated twice on the GPU
- WHEN both runs complete
- THEN their point grids, normal grids, and per-vertex normals SHALL be identical
  between runs

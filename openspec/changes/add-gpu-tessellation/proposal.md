## Why

Meshing is the single biggest self-contained throughput win for GPU offload
(`ROADMAP.md` Phase 2; `occt-usage` §Meshing): evaluating a parametric surface on
a dense parameter grid is embarrassingly data-parallel and fp32-tolerant, which
is exactly what the `add-metal-compute-backend` GPU path is for. The exact fp64
topology core stays authoritative on the CPU — the GPU only produces
display-tolerant sample points and normals that a CPU triangulator stitches into
a mesh.

Phase 0 gave us the compute-backend seam and the precision guard; this change is
the first algorithm that actually uses the GPU. It keeps topology (which faces,
trim loops, connectivity, stitching) entirely on the CPU and offloads only the
per-sample numeric evaluation, so correctness never depends on the GPU and the
result is a mesh identical (within fp32 tolerance) to the CPU-only path.

## What Changes

- Add **GPU surface-grid evaluation**: for a NURBS/Bezier surface and a
  parameter grid `(u_i, v_j)`, evaluate positions and surface normals in parallel
  on the compute backend (fp32), returning grids of points and normals.
- **Wire the existing `gpu_surface_eval` module into `cc_tessellate` /
  `cc_face_meshes`** behind a new, additive, default-OFF toggle
  (`cc_set_gpu_tessellation` / `cc_gpu_tessellation_enabled`). When ON, each face
  is classified for GPU-eligibility and, if eligible, tessellated from the GPU
  grid; **every other face falls back to OCCT `BRepMesh_IncrementalMesh`**.
- **Per-face GPU-eligibility with mandatory OCCT fallback**: a face is eligible
  ONLY when it is an untrimmed rectangular patch — single outer wire, no holes, 2D
  boundary equal to the `BRepTools::UVBounds` rectangle within tolerance — AND its
  surface converts to a `cyber::metal::SurfaceDef` (via `Geom_BSplineSurface` /
  `BRepBuilderAPI_NurbsConvert` / Bézier decomposition, degree within
  `kMaxSurfaceDegree`). Holed, trimmed, unsupported, or unrepresentable-rational
  faces fall back to OCCT. When in doubt, fall back.
- **Stitch GPU-path and OCCT-fallback faces into one `CCMesh`**, welding
  coincident boundary vertices within tolerance for watertightness (documented if
  not welded), preserving the existing face-id tagging and traversal order.
- Add **GPU per-vertex mesh normals**: a mesh post-processing pass that computes
  smooth per-vertex normals (area/angle-weighted face-normal accumulation) on the
  GPU for an existing vertex/index mesh.
- Route through the Phase-0 compute-backend (`SurfaceEval`, `MeshPostProcess` work
  kinds) so it runs on Metal when available and on the CPU reference otherwise,
  with **identical-within-fp32-tolerance** results either way.
- Provide a **CPU reference** implementation for every GPU kernel so an
  on-simulator GPU-vs-CPU (and GPU-path-vs-OCCT-path) parity test can assert the
  match.

C ABI change is **ADDITIVE only**: two new entry points
(`cc_set_gpu_tessellation(int)` + `cc_gpu_tessellation_enabled(void)`). The
`cc_tessellate` / `cc_face_meshes` signatures are unchanged. GPU tessellation
defaults OFF, so with it off `cc_tessellate` behaves EXACTLY as today. The GPU
path is guarded by `#ifdef CYBERCAD_HAS_METAL` so the non-Metal OCCT build still
compiles; GPU eval is fp32 (display mesh only) and the exact fp64 modeling core is
untouched.

## Capabilities

### New Capabilities
- `gpu-tessellation`: GPU-accelerated parametric surface-grid evaluation (points
  + normals) wired into `cc_tessellate` / `cc_face_meshes` behind a default-OFF
  toggle, with per-face GPU-eligibility classification and mandatory OCCT
  `BRepMesh` fallback, plus a GPU per-vertex mesh-normal post-processing pass, with
  results matching a CPU/OCCT reference within a documented fp32 tolerance. Depends
  on `metal-backend` (the GPU dispatch path) and `compute-backend` (interface +
  precision guard); consumes the OCCT triangulation/topology already behind
  `engine-adapter`.

### Modified Capabilities
<!-- none — gpu-tessellation is additive: the only ABI change is the two new
     cc_set_gpu_tessellation / cc_gpu_tessellation_enabled entry points; the
     cc_tessellate / cc_face_meshes signatures and the OCCT-only mesh (toggle OFF)
     are unchanged. -->

## Impact

- **Contract**: accelerates `cybercad/openspec/specs/occt-usage/spec.md` §Meshing
  without changing its observable result (mesh equal within fp32 tolerance).
- **App**: no code change — `cc_tessellate` returns the same mesh, produced faster
  when a GPU is available and identically on CPU when not.
- **Build**: GPU kernels are self-contained fp32 numeric code compiled at runtime
  as MSL; they do **not** link OCCT, and the GPU test suite is OCCT-free.
- **Determinism / precision**: GPU output is fp32 and confined to display-tolerant
  sampling; the exact fp64 surface definitions and all topology remain on the CPU.
- **Risk**: fp32 sampling error must stay within the meshing deflection tolerance;
  the CPU reference + parity test bound it, and the CPU path is always available.

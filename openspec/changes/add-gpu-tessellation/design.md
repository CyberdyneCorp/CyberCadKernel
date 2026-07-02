## Context

Tessellation dominates the interactive-display cost and is the "biggest
self-contained win" for GPU offload (`ROADMAP.md` Phase 2, `occt-usage`
§Meshing). Evaluating a NURBS/Bezier surface on a dense `(u, v)` grid — position
and normal per sample — is a large batch of independent fp32 evaluations: an
ideal `ComputeKind::SurfaceEval` job for the `add-metal-compute-backend` path.
Computing smooth per-vertex normals on an existing mesh is likewise a data-parallel
`ComputeKind::MeshPostProcess` pass.

The invariant is that the CPU remains the source of truth for geometry and
topology: exact fp64 surface definitions, trim loops, grid->triangle connectivity,
and stitching all stay on the CPU. The GPU only fills sample fields (points,
normals) that the CPU triangulator consumes. So the mesh must be identical —
within a documented fp32 tolerance — whether the samples came from the GPU or a
CPU reference.

Constraints:
- **ABI stability**: `cc_tessellate` / `cc_face_meshes` signatures unchanged; the
  only ABI addition is `cc_set_gpu_tessellation(int)` / `cc_gpu_tessellation_enabled(void)`.
- **Default OFF**: with GPU tessellation off, `cc_tessellate` is byte-identical to
  today's OCCT `BRepMesh_IncrementalMesh` path (the 221/221 `run-sim-suite.sh`
  baseline must be unchanged).
- **Topology on CPU**: the GPU never decides eligibility, connectivity, or trimming.
- **Correctness over coverage**: a face takes the GPU path ONLY when it is provably
  an untrimmed rectangular patch representable by `gpu_surface_eval`; every other
  face falls back to OCCT. When in doubt, fall back.
- **Metal is iOS-only**: the GPU path is guarded by `#ifdef CYBERCAD_HAS_METAL` so
  the non-Metal OCCT build still compiles.
- **fp32-only GPU**: precision guard keeps exact fp64 work on the CPU
  (`compute_backend.h`); GPU output is display-mesh only.
- **Determinism by default**: a fixed grid decomposition and a fixed normal-
  accumulation order so repeated runs are reproducible.

## Goals / Non-Goals

Goals:
- GPU surface-grid evaluation returning point + normal grids for NURBS/Bezier
  surfaces, dispatched through the compute backend.
- A CPU triangulator that consumes those grids and owns all topology.
- A GPU per-vertex mesh-normal post-processing pass.
- A CPU reference for every GPU kernel and an on-simulator GPU-vs-CPU parity test
  asserting a match within fp32 tolerance.
- CPU fallback that produces the same mesh when no GPU is present.

Non-Goals:
- Native replacement of the CPU triangulator/topology (Phase 4
  `add-native-tessellation`).
- LOD / decimation / deformation beyond per-vertex normals (later mesh
  post-processing work).
- BVH / picking (`add-gpu-spatial-acceleration`).
- The Metal backend primitives themselves (`add-metal-compute-backend`).

## Decisions

- **Additive default-OFF toggle.** `cc_set_gpu_tessellation(int)` /
  `cc_gpu_tessellation_enabled(void)` gate the whole GPU path, mirroring the
  existing `cc_set_parallel` pattern in `src/facade/cc_kernel.cpp`. Default OFF, so
  `cc_tessellate` is unchanged until a caller opts in; on a non-Metal build the
  setter is a no-op and the query returns 0.
- **Per-face eligibility, mandatory OCCT fallback.** Inside the OCCT tessellate
  path (`src/engine/occt/occt_tessellate.cpp`) each face is classified before the
  merge. A face is GPU-eligible ONLY when: it has a single outer wire; it has no
  inner wires (no holes); its 2D boundary equals the `BRepTools::UVBounds`
  rectangle within tolerance (an untrimmed rectangular patch); and its surface
  converts to a `cyber::metal::SurfaceDef` via `Geom_BSplineSurface` /
  `BRepBuilderAPI_NurbsConvert` / Bézier decomposition with degree within
  `kMaxSurfaceDegree`. Any other face (holes, trimmed, unsupported surface, or a
  rational form the module cannot represent) is meshed by OCCT
  `BRepMesh_IncrementalMesh` exactly as today. Ambiguous cases fall back.
- **Grid → triangles on the CPU.** For an eligible face, call
  `evaluateSurfaceGrid(backend, surf, req)` (Metal when active, CPU reference
  otherwise) to get the fp32 point/normal grid, then build a regular grid
  triangulation on the CPU (respecting face orientation, matching the OCCT winding
  convention in `appendFaceTriangulation`). Grid density is derived from the
  requested deflection so vertices stay within the display tolerance.
- **Stitch into one CCMesh, per-face vertices (no cross-face weld).** GPU-path
  faces and OCCT-fallback faces are merged into a single vertex/triangle buffer in
  the exact face-traversal order of the OCCT-only path (`TopExp_Explorer` over
  `TopAbs_FACE` for `cc_tessellate`; the `mapFaces` index for `cc_face_meshes`),
  each appended at the running `baseVertex`. **This first cut does NOT weld
  coincident boundary vertices across faces** — deliberately, because the existing
  OCCT-only path also emits per-face vertices (a box already yields 6×4 = 24
  vertices, not 8), so keeping per-face vertices preserves that behaviour and the
  face-id tagging exactly. Watertightness is not degraded relative to today: an
  eligible face's grid boundary is sampled on the shared trimmed edge, so its
  boundary vertices coincide with the neighbour face's within the documented fp32
  tolerance (`kSurfaceEvalAbsTol`/`kSurfaceEvalRelTol`), the same
  coincident-within-tolerance seam the OCCT-only path already produces. The
  divergence-theorem volume check is invariant to vertex duplication, so it still
  holds. A future pass may add tolerance welding at stitch time; until then the
  seam handling is per-face-vertices-with-fp32-coincident-boundaries as documented
  here.
- **Split numeric eval from topology.** The GPU evaluates the surface on a
  regular parameter grid (points + normals); the CPU applies eligibility, trimming,
  builds the triangle connectivity, and stitches adjacent faces. This keeps the
  fp64/topology authority on the CPU and makes the GPU contribution a pure,
  testable numeric field.
- **Normals from analytic partials.** The surface normal at `(u, v)` is
  `normalize(dS/du x dS/dv)` evaluated on the GPU from the basis derivatives, so
  normals come from the same evaluation as the points (no finite differencing).
- **Per-vertex mesh normals by weighted face-normal accumulation.** For an
  existing mesh, accumulate each triangle's face normal (weighted by incident
  angle/area) into its vertices, then normalize — done on the GPU with a fixed
  accumulation order for determinism.
- **Route via `ComputeKind::SurfaceEval` / `MeshPostProcess`.** The active backend
  (Metal or CPU) is chosen by the registry; fp32 tolerance is the same contract
  either way, so callers do not branch on backend.
- **CPU reference is the oracle.** Every kernel has a straightforward CPU
  implementation; the parity test compares GPU vs CPU element-by-element within a
  documented fp32 tolerance (absolute + relative), on the simulator GPU.
- **fp32 sampling error bounded by deflection.** The evaluation tolerance is kept
  well within the meshing deflection so fp32 sampling never coarsens the visible
  mesh; documented alongside the tolerance constant.

## Risks / Trade-offs

- **fp32 vs fp64 sampling drift.** GPU samples differ from an fp64 CPU eval at the
  fp32 level; mitigated by keeping the tolerance below deflection and by the
  parity test. Exact geometry queries never use these samples.
- **Grid vs trimmed boundary.** A regular grid cannot represent a trimmed or holed
  face, so those faces are NOT GPU-eligible and fall back to OCCT. Only untrimmed
  rectangular patches (2D boundary == `BRepTools::UVBounds` within tolerance) take
  the GPU path, where the grid is provably the whole face. This removes GPU-side
  clipping entirely — trimming stays with OCCT.
- **Topology differs from OCCT for eligible faces.** The GPU grid triangulation is
  a regular quad-grid, not OCCT's `BRepMesh` triangulation, so triangle counts
  differ. Equivalence is asserted by vertices-on-face + bbox + area within fp32
  tolerance, not by triangle-topology equality.
- **Normal accumulation order.** Parallel scatter into shared vertices can reorder
  fp32 adds; pinned to a fixed order (or deterministic reduction) so results are
  reproducible.
- **Simulator throughput is not device throughput.** Parity is validated on the
  simulator GPU; real speedup is an on-device concern (deferred, per Phase 0/1
  acceptance).

## Migration Plan

1. Define the surface-grid eval job (inputs: control net/knots + `(u,v)` grid;
   outputs: point grid + normal grid) against the compute backend. **(done)**
2. Implement the CPU reference eval; implement the MSL GPU kernel; add the parity
   test on the simulator GPU. **(done)**
3. Add the additive toggle `cc_set_gpu_tessellation` / `cc_gpu_tessellation_enabled`
   (default OFF), threaded through the facade and engine like `cc_set_parallel`.
4. Add per-face GPU-eligibility classification (single outer wire, no holes, UV ==
   `BRepTools::UVBounds` rectangle, surface convertible to `SurfaceDef`) with OCCT
   `BRepMesh` fallback for every other face, inside `occt_tessellate.cpp` under
   `#ifdef CYBERCAD_HAS_METAL`.
5. Build the CPU grid triangulator for eligible faces; stitch GPU-path + OCCT
   faces into one `CCMesh`, welding coincident boundary vertices.
6. Implement per-vertex mesh normals: CPU reference + GPU kernel + parity test.
   **(done)**
7. Add the on-simulator GPU-path-vs-OCCT-path comparison (vertices-on-face + bbox +
   area within fp32 tolerance) and the toggle-OFF baseline check (221/221
   unchanged).
8. `openspec validate --all --strict`; update `ROADMAP.md` Phase 2 status.

## Open Questions

- The exact fp32 tolerance constant (absolute + relative) tying GPU parity to the
  meshing deflection — reuse `kSurfaceEvalAbsTol` / `kSurfaceEvalRelTol` from
  `gpu_surface_eval.h`, confirmed empirically from the parity suite.
- The `BRepTools::UVBounds` rectangle-equality tolerance for eligibility — set so
  that any real trim (a hole or a non-rectangular boundary) is rejected while a
  genuine untrimmed patch passes.
- Whether boundary-vertex welding is done at stitch time or left to the app.
  RESOLVED for this change: the first cut does NOT weld — it keeps per-face
  vertices (matching the existing OCCT-only path, which is already unwelded), with
  boundary vertices coincident within the fp32 tolerance. Tolerance welding is a
  documented follow-up (see the "Stitch into one CCMesh" decision above).

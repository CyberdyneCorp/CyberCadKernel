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
- **ABI stability**: `cc_tessellate` / `cc_face_meshes` signatures unchanged.
- **Topology on CPU**: the GPU never decides connectivity or trimming.
- **fp32-only GPU**: precision guard keeps exact fp64 work on the CPU
  (`compute_backend.h`).
- **No OCCT in GPU modules**: kernels + GPU test suite are self-contained numeric
  code, runnable on the iOS simulator GPU.
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

- **Split numeric eval from topology.** The GPU evaluates the surface on a
  regular parameter grid (points + normals); the CPU applies trimming, builds the
  triangle connectivity, and stitches adjacent faces. This keeps the fp64/topology
  authority on the CPU and makes the GPU contribution a pure, testable numeric
  field.
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
- **Grid vs trimmed boundary.** A regular grid must be reconciled with trim loops
  on the CPU; the GPU only supplies interior/boundary samples, the CPU clips —
  documented as the topology-on-CPU boundary.
- **Normal accumulation order.** Parallel scatter into shared vertices can reorder
  fp32 adds; pinned to a fixed order (or deterministic reduction) so results are
  reproducible.
- **Simulator throughput is not device throughput.** Parity is validated on the
  simulator GPU; real speedup is an on-device concern (deferred, per Phase 0/1
  acceptance).

## Migration Plan

1. Define the surface-grid eval job (inputs: control net/knots + `(u,v)` grid;
   outputs: point grid + normal grid) against the compute backend.
2. Implement the CPU reference eval; implement the MSL GPU kernel; add the parity
   test on the simulator GPU.
3. Wire the CPU triangulator to consume the grids (trim + connectivity + stitch
   on CPU); confirm the mesh matches the CPU-only path within tolerance.
4. Implement per-vertex mesh normals: CPU reference + GPU kernel + parity test.
5. Route `cc_tessellate` / `cc_face_meshes` through the GPU eval path when a GPU
   backend is active; CPU fallback otherwise.
6. `openspec validate --all --strict`; update `ROADMAP.md` Phase 2 status.

## Open Questions

- The exact fp32 tolerance constant (absolute + relative) tying GPU parity to the
  meshing deflection — set empirically from the parity suite.
- Whether trimmed-surface sampling needs a GPU-side inside/outside test or stays
  fully CPU-clipped for the first cut (CPU-clip chosen initially).

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
- Feed those grids to a **CPU triangulator**: the CPU owns trimming, grid->triangle
  connectivity, and stitching. **Topology stays on the CPU**; the GPU contributes
  only the numeric sample fields.
- Add **GPU per-vertex mesh normals**: a mesh post-processing pass that computes
  smooth per-vertex normals (area/angle-weighted face-normal accumulation) on the
  GPU for an existing vertex/index mesh.
- Route both through the Phase-0 compute-backend (`SurfaceEval`,
  `MeshPostProcess` work kinds) so they run on Metal when available and on the CPU
  otherwise, with **identical-within-fp32-tolerance** results either way.
- Provide a **CPU reference** implementation for every GPU kernel so an
  on-simulator GPU-vs-CPU parity test can assert the match.

No `cc_*` signature change: `cc_tessellate` / `cc_face_meshes` gain a GPU
evaluation path internally; the mesh they return is unchanged within tolerance.

## Capabilities

### New Capabilities
- `gpu-tessellation`: GPU-accelerated parametric surface-grid evaluation (points
  + normals) feeding a CPU triangulator that owns topology, plus a GPU per-vertex
  mesh-normal post-processing pass, with results matching a CPU reference within a
  documented fp32 tolerance. Depends on `metal-backend` (the GPU dispatch path)
  and `compute-backend` (interface + precision guard); consumes the CPU
  triangulation/topology already behind `engine-adapter`.

### Modified Capabilities
<!-- none — gpu-tessellation is additive; the CPU triangulation path and the
     cc_* meshing signatures are unchanged. -->

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

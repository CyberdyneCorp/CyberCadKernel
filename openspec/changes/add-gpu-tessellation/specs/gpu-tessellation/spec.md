# gpu-tessellation

## ADDED Requirements

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

### Requirement: CPU triangulator owns topology
The GPU SHALL only supply per-sample numeric fields (points and normals); the CPU
triangulator SHALL own all topology — trimming, grid-to-triangle connectivity, and
cross-face stitching. The mesh built from GPU-evaluated grids SHALL match the
CPU-only mesh within the documented fp32 tolerance.

#### Scenario: GPU-fed mesh matches the CPU-only mesh
- GIVEN a face whose surface samples are evaluated on the GPU and fed to the CPU
  triangulator
- WHEN the same face is tessellated entirely on the CPU for the same deflection
- THEN the two meshes SHALL have the same triangle topology AND their vertex
  positions SHALL agree within the documented fp32 tolerance

#### Scenario: Topology decisions never run on the GPU
- GIVEN a trimmed face tessellated through the GPU evaluation path
- WHEN the mesh is produced
- THEN trimming, connectivity, and stitching SHALL have been computed on the CPU
  AND the GPU SHALL have contributed only sample points and normals

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

#### Scenario: Same mesh with or without a GPU backend
- GIVEN a shape tessellated with a Metal backend registered and again with no GPU
  backend registered
- WHEN `cc_tessellate` runs in each case
- THEN both SHALL produce the same mesh within the documented fp32 tolerance AND
  the `cc_tessellate` signature SHALL be unchanged

### Requirement: Deterministic GPU tessellation
GPU tessellation SHALL be reproducible: using a fixed grid decomposition and a
fixed normal-accumulation order, repeated GPU runs on the same input SHALL produce
identical output.

#### Scenario: Repeated GPU tessellation runs are reproducible
- GIVEN the same surface and grid evaluated twice on the GPU
- WHEN both runs complete
- THEN their point grids, normal grids, and per-vertex normals SHALL be identical
  between runs

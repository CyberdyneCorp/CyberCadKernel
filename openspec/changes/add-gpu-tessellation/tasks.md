# Tasks — add-gpu-tessellation

Verification levels: **host** = the CPU reference eval + CPU triangulator run in
the no-OCCT / no-Metal host CTest (CPU fallback path, unchanged mesh);
**ios-sim-build** = the GPU kernels + harness compile for
`arm64-apple-ios16.0-simulator` (`-framework Metal -framework Foundation`, no
OCCT); **ios-sim-run** = the GPU kernels run on the booted simulator GPU
("Apple iOS simulator GPU", runtime-compiled MSL) and the GPU-vs-CPU parity
checks pass within the documented fp32 tolerance — this is the acceptance bar for
every parity requirement below.

## 1. Surface-grid evaluation job
- [x] 1.1 Define the eval job: inputs (control net / knots + `(u,v)` grid),
  outputs (point grid + normal grid), routed as `ComputeKind::SurfaceEval`. (**host**)
- [x] 1.2 CPU reference: evaluate NURBS/Bezier position + analytic normal per
  sample (the parity oracle). (**host**)
- [x] 1.3 GPU MSL kernel: same position + `normalize(dS/du x dS/dv)` per sample,
  fp32, into shared buffers. (**ios-sim-run**)
- [x] 1.4 Parity: GPU point grid + normal grid match the CPU reference within the
  documented fp32 tolerance, element-by-element. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS surface-eval: GPU grid matches de Casteljau reference (fp32 rel ~1e-4)` + grid-dimensions check in `run-sim-gpu-suite.sh`.

## 2. CPU triangulator (topology stays on CPU)
- [ ] 2.1 Consume the point/normal grids; apply trimming, grid->triangle
  connectivity, and cross-face stitching on the CPU. (**host**)
  <br>NOT DONE: the design is documented in `gpu_surface_eval.h` (GPU emits the
  point/normal grid a CPU triangulator consumes) but no standalone triangulator
  component that applies trimming / grid->triangle connectivity / cross-face
  stitching exists yet. The GPU surface-eval grid is validated standalone against
  the CPU reference; feeding it through a triangulator is a follow-up.
- [ ] 2.2 The mesh built from GPU-evaluated grids matches the CPU-only mesh within
  fp32 tolerance (same triangle count/topology, positions within tolerance). (**ios-sim-run**)
  <br>NOT DONE: depends on 2.1; there is no GPU-fed-mesh vs CPU-only-mesh check in
  the sim suite (only the grid-level parity of task 1.4).

## 3. GPU per-vertex mesh normals (post-processing)
- [x] 3.1 CPU reference: weighted face-normal accumulation -> normalized per-vertex
  normals, fixed accumulation order. (**host**)
- [x] 3.2 GPU MSL kernel: same computation via `ComputeKind::MeshPostProcess`. (**ios-sim-run**)
- [x] 3.3 Parity: GPU per-vertex normals match the CPU reference within the
  documented fp32 tolerance. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS mesh-post: GPU normals match CPU reference per component (fp32)` and `... dot product ~ 1` in `run-sim-gpu-suite.sh`.

## 4. Backend routing + fallback
- [x] 4.1 Route surface eval + mesh normals through the compute backend (Metal
  when active, CPU otherwise); precision guard keeps the exact fp64 core on CPU. (**host** + **ios-sim-run**)
  <br>Both engines take a `MetalBackend*` (GPU path) and fall back to the CPU
  reference when null; the fp32-only Metal backend + Phase-0 precision guard keep
  fp64 off the GPU. GPU path verified on the simulator.
- [x] 4.2 CPU fallback produces the same mesh (within tolerance) when no GPU
  backend is registered. (**host**)
  <br>Verified at the grid/normal level: with a null backend the surface-eval and
  mesh-normal engines run the identical fp32 CPU reference the GPU is compared
  against (parity checks in 1.4 / 3.3 establish they agree). Full mesh-level
  equivalence depends on the triangulator (2.x).
- [ ] 4.3 `cc_tessellate` / `cc_face_meshes` use the GPU eval path internally with
  no signature change. (**ios-sim-run**)
  <br>NOT DONE: the GPU surface-eval + mesh-normal modules are validated
  standalone but are not yet wired into the OCCT `cc_tessellate` / `cc_face_meshes`
  facade path (no references from `src/facade` or `src/engine`). Integration into
  the tessellate path is the remaining follow-up.

## 5. Determinism
- [ ] 5.1 Fixed grid decomposition + fixed normal-accumulation order: repeated GPU
  runs on the same input are reproducible. (**ios-sim-run**)
  <br>Deterministic by construction (one GPU thread per grid sample; fixed CSR
  accumulation order for per-vertex normals), but the sim suite does not yet run
  an explicit repeat-run reproducibility assertion. Follow-up: add a repeat-run
  check to `run-sim-gpu-suite.sh`.

## 6. Validation
- [ ] 6.1 On-simulator GPU-vs-CPU parity suite (surface grid, GPU-fed mesh,
  per-vertex normals) green within fp32 tolerance. (**ios-sim-run**)
  <br>PARTIAL: surface-grid parity and per-vertex-normal parity are green on the
  simulator (`run-sim-gpu-suite.sh`, all 18 checks pass); the **GPU-fed mesh**
  leg is not covered because the CPU triangulator (2.x) is not built yet.
- [x] 6.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 2 +
  change index for `gpu-tessellation`.

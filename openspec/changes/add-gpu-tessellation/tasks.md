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

## 2. Additive GPU-tessellation toggle
- [x] 2.1 Add `cc_set_gpu_tessellation(int)` + `cc_gpu_tessellation_enabled(void)`
  to `include/cybercadkernel/cc_kernel.h` and `src/facade/cc_kernel.cpp`, mirroring
  the `cc_set_parallel` / `cc_parallel_enabled` pattern; default OFF; no-op / returns
  0 on a non-Metal build. `cc_tessellate` / `cc_face_meshes` signatures unchanged. (**host**)
  <br>Routed through `IEngine::set_gpu_tessellation` / `gpu_tessellation_enabled`
  (default no-op/false in the stub); `OcctEngine` holds the flag as
  `std::atomic<bool>` and only latches it ON under `CYBERCAD_HAS_METAL`.
- [x] 2.2 With the toggle OFF, `cc_tessellate` / `cc_face_meshes` produce the exact
  OCCT-only mesh (no GPU dispatch); `scripts/run-sim-suite.sh` stays 221/221. (**ios-sim-run**)
  <br>Verified: with the toggle OFF (default), `appendFaceMesh` never references
  the GPU path (guarded by `if (gpuOn)` inside `#ifdef CYBERCAD_HAS_METAL`) and
  delegates to the unchanged `appendFaceTriangulation` with identical face
  traversal/merge — the GPU-OFF C++ path is byte-identical. `scripts/run-sim-suite.sh`
  = **221 passed, 0 failed** (exit 0), run against a sim slice rebuilt from CURRENT
  source (OCCT on, Metal off), not the stale prebuilt slice. Host stub build
  (OCCT=OFF, METAL=OFF) also builds clean, CTest 7/7, and
  `cc_set_gpu_tessellation` is a verified safe runtime no-op in the stub
  (`cc_gpu_tessellation_enabled()==0` before and after enable, no crash).
- [x] 2.3 ABI contract test (`tests/test_abi.cpp`) still passes with the two
  additive entry points. (**host**) <br>Verified: host CTest `test_abi` green.

## 3. Per-face eligibility + GPU grid triangulator (topology stays on CPU)
- [x] 3.1 In `occt_gpu_tessellate.cpp` (under `#ifdef CYBERCAD_HAS_METAL`), classify each
  face: GPU-eligible iff single outer wire, no inner wires (holes), 2D boundary ==
  `BRepTools::UVBounds` rectangle within tolerance, and surface convertible to a
  `cyber::metal::SurfaceDef` (via `Geom_RectangularTrimmedSurface` +
  `GeomConvert::SurfaceToBSplineSurface`, degree <= `kMaxSurfaceDegree`). Every other
  face falls back to OCCT `BRepMesh_IncrementalMesh`; ambiguous cases fall back
  (`tryTessellateFaceGPU` returns false and any `Standard_Failure` is caught). (**host**)
- [x] 3.2 For an eligible face, call `evaluateSurfaceGrid(backend, surf, req)` (grid
  density from deflection) and build a regular grid triangulation on the CPU, matching
  the OCCT winding convention in `appendFaceTriangulation` (forward `+u×+v`, flipped
  for a REVERSED face). (**host**)
- [x] 3.3 Stitch GPU-path faces and OCCT-fallback faces into one `CCMesh` in the
  existing face traversal order; per-face vertices with fp32-coincident boundaries
  (NOT welded — documented in design.md). `cc_face_meshes` keeps one slot per face id. (**host**)
- [x] 3.4 GPU-path face vertices lie on the exact fp64 OCCT surface within the
  documented fp32 tolerance; GPU-path vs OCCT-path bbox and area agree within
  tolerance (on-simulator GPU-path-vs-OCCT-path comparison). (**ios-sim-run**)
  <br>Verified via `scripts/run-sim-integ-suite.sh` (`brep_available=1`, kernel
  built from source with OCCT + Metal). **Box** (10×10×10, all 6 faces GPU-eligible):
  routing `gpu=6 fallback=0`, GPU mesh v=600 t=972, watertight, enclosed
  volume=1000.000069, area gpu=600.000004 vs occt=600.000000, bbox match.
  **Slab** (20×20×10 with a round hole, mixed): routing `gpu=4 fallback=3 total=7`
  (4 planar outer walls on GPU, 3 holed/curved faces fall back to OCCT), bbox
  match, area gpu=1732.037370 vs occt=1732.037356, volume gpu=3720.000797 vs
  occt=3720.000673, watertight. Full integ suite = **26 passed, 0 failed**.

## 4. GPU per-vertex mesh normals (post-processing)
- [x] 4.1 CPU reference: weighted face-normal accumulation -> normalized per-vertex
  normals, fixed accumulation order. (**host**)
- [x] 4.2 GPU MSL kernel: same computation via `ComputeKind::MeshPostProcess`. (**ios-sim-run**)
- [x] 4.3 Parity: GPU per-vertex normals match the CPU reference within the
  documented fp32 tolerance. (**ios-sim-run**)
  <br>Verified: `[GPU] PASS mesh-post: GPU normals match CPU reference per component (fp32)` and `... dot product ~ 1` in `run-sim-gpu-suite.sh`.

## 5. Backend routing + fallback
- [x] 5.1 Route surface eval + mesh normals through the compute backend (Metal
  when active, CPU otherwise); precision guard keeps the exact fp64 core on CPU. (**host** + **ios-sim-run**)
  <br>Both engines take a `MetalBackend*` (GPU path) and fall back to the CPU
  reference when null; the fp32-only Metal backend + Phase-0 precision guard keep
  fp64 off the GPU. GPU path verified on the simulator.
- [x] 5.2 CPU fallback produces the same grid/normals (within tolerance) when no GPU
  backend is registered. (**host**)
  <br>Verified at the grid/normal level: with a null backend the surface-eval and
  mesh-normal engines run the identical fp32 CPU reference the GPU is compared
  against (parity checks in 1.4 / 4.3 establish they agree).

## 6. Determinism
- [ ] 6.1 Fixed grid decomposition + fixed normal-accumulation order: repeated GPU
  runs on the same input are reproducible. (**ios-sim-run**)
  <br>Deterministic by construction (one GPU thread per grid sample; fixed CSR
  accumulation order for per-vertex normals), but the sim suite does not yet run
  an explicit repeat-run reproducibility assertion. Follow-up: add a repeat-run
  check to `run-sim-gpu-suite.sh`.

## 7. Validation
- [x] 7.1 On-simulator parity suite green within fp32 tolerance: surface grid
  (GPU vs CPU reference), GPU-path face vs OCCT-path face (vertices-on-face + bbox +
  area), and per-vertex normals. (**ios-sim-run**)
  <br>Verified: surface-grid parity and per-vertex-normal parity are green on the
  simulator (`run-sim-gpu-suite.sh`, all 18 checks pass); the **GPU-path-vs-OCCT-path**
  face comparison is now green via `run-sim-integ-suite.sh` (**26 passed, 0 failed**),
  which builds the kernel from source with the GPU path wired into `cc_tessellate`
  and compares the GPU-fed mesh against the OCCT-only mesh (bbox + area + volume +
  watertightness) on the box and mixed slab fixtures (see 3.4).
- [x] 7.2 Toggle-OFF baseline: `scripts/run-sim-suite.sh` stays 221/221 unchanged
  with GPU tessellation OFF (default). (**ios-sim-run**)
  <br>Verified: **221 passed, 0 failed** (exit 0) against a sim slice rebuilt from
  current source (OCCT on, Metal off). One infra fix: `scripts/run-sim-suite.sh`
  previously skipped only `parity_bench.cpp`, but `tests/sim/` now also holds two
  standalone GPU harnesses with their own `main()` + GPU/Metal includes
  (`metal_selftest.cpp`, `integ_gpu_tess.cpp`) that were being swept into the
  OCCT-only compile; the skip list was extended to exclude all three standalone
  harnesses. No regression from the integration itself.
- [x] 7.3 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 2 +
  change index for `gpu-tessellation`.

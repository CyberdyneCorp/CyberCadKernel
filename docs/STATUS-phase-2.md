# CyberCadKernel — Phase 2 status (GPU / Metal)

Honest, verification-anchored snapshot of Phase 2 (`add-metal-compute-backend`,
`add-gpu-tessellation`, `add-gpu-spatial-acceleration`). Nothing below is claimed
as working unless it was actually built and run in this environment. The
acceptance bar for Phase 2 (decided alongside the change specs) is the
**booted iOS simulator** running runtime-compiled MSL on the real
"Apple iOS simulator GPU", with every GPU result asserted against an independent
CPU reference within an fp32 tolerance. On-device (physical Apple silicon) runs
and OCCT `cc_tessellate` integration are explicit follow-ups.

Date: 2026-07-02 · Branch: `main`.

## TL;DR

- **VERIFIED here (iOS simulator GPU):** the Metal compute backend self-test
  passes, and the integrated GPU-vs-CPU parity suite passes **18 / 18** checks —
  surface-grid evaluation, LBVH build + stackless nearest-hit ray traversal,
  batched ray-picking, and GPU per-vertex mesh normals — each within an fp32
  tolerance against an independent CPU reference. The host (no-Metal) CTest stays
  green (`CYBERCAD_HAS_METAL=OFF`), so the CPU-only path is unaffected.
- **NOT verified / not built here:** the **CPU triangulator** that turns
  GPU-evaluated grids into a stitched/trimmed mesh (so no "GPU-fed mesh vs
  CPU-only mesh" check exists); the **frustum-pick** parity leg (kernels + CPU
  reference are coded but not asserted in the sim suite); **`cc_tessellate` /
  `cc_face_meshes` integration** of the GPU eval path (modules are standalone,
  not wired into the OCCT facade); explicit **repeat-run determinism** assertions;
  and any **on-device** run.

## What was implemented

All Metal code is Objective-C++ (`.mm`), iOS-only, compiled solely when
`CYBERCAD_HAS_METAL=1`. It links only Metal + Foundation and the Phase-0 CPU
backend / `ComputeRegistry` — **no OCCT**, and it is **fp32-only**: no fp64
modeling work is ever routed to the GPU.

### Metal compute backend (`add-metal-compute-backend`)

`src/compute/metal/metal_backend.{h,mm}` — a drop-in Phase-0 `IComputeBackend`:

- CMake option `CYBERCAD_HAS_METAL` (default OFF); the `.mm` target is added
  iOS-only and does not link OCCT. Public header exposes only `IComputeBackend`,
  `makeMetalBackend()`, and `registerMetalBackend()`; no Metal type crosses it.
- Acquires the system default `MTLDevice` + command queue at construction;
  construction fails cleanly (null backend) when no device is available.
- `MTLResourceStorageModeShared` (unified-memory) input/output buffers with a
  CPU-write → GPU-read → CPU-read round-trip and no explicit copy.
- Runtime MSL compilation via `newLibraryWithSource:` into a
  `MTLComputePipelineState`, cached by (source, function) key so a second
  dispatch reuses the pipeline; malformed source returns a `Result` error, not a
  crash.
- Compute dispatch over `[0, count)` with threadgroup sizing from
  `maxTotalThreadsPerThreadgroup`, commit + wait; `parallel_for` for interface
  conformance.
- Registers with `ComputeRegistry` (CPU stays default + fallback);
  `supports_fp64()` is false and a `Precision::Fp64` dispatch is defensively
  refused, so the precision guard always keeps fp64 on the CPU.

### GPU surface evaluation (`add-gpu-tessellation`, module 1)

`src/compute/metal/gpu_surface_eval.{h,mm}` — `ComputeKind::SurfaceEval`. One GPU
thread per `(u,v)` grid sample evaluates position + `normalize(dS/du × dS/dv)` in
fp32 into shared buffers. A CPU de Casteljau / Cox-de Boor reference is both the
parity oracle and the CPU fallback used when no Metal backend is present.
Documented tolerance: `kSurfaceEvalAbsTol = kSurfaceEvalRelTol = 1e-4f`.

### GPU per-vertex mesh normals (`add-gpu-tessellation`, module 3)

`src/compute/metal/gpu_mesh_post.{h,mm}` — `ComputeKind::MeshPostProcess`. Weighted
face-normal accumulation → normalized per-vertex normals, with a fixed CSR
accumulation order matching `computeNormalsCpuReference` exactly.

### GPU LBVH build + ray traversal (`add-gpu-spatial-acceleration`, modules 2–3)

`src/compute/metal/gpu_bvh.{h,mm}` — GPU AABB + Morton-code computation, a
deterministic (stable / index tie-break) sort, and a linear stackless BVH node
array built as `ComputeKind::BvhBuild`. Query side: stackless nearest-hit ray
traversal (documented epsilon + lowest-index tie-break) and an AABB-vs-frustum
pick returning an ascending-sorted index set. `closestHitBruteForce` /
`frustumPickBruteForce` are the CPU parity oracles + null-backend fallback
(`onGpu()`).

### GPU picking (`add-gpu-spatial-acceleration`, module 3)

`src/compute/metal/gpu_pick.{h,mm}` — `ComputeKind::Picking`. Batched ray-pick and
frustum-pick over a triangle scene, with `cpuPickReference` / `cpuFrustumReference`
as the oracle + CPU fallback (`usesGpu()`).

## Integrated GPU-vs-CPU simulator results

Both suites run runtime-compiled MSL on the real "Apple iOS simulator GPU" via
`xcrun simctl spawn`. No timing was emitted (parity only).

**Backend self-test** (`scripts/run-sim-gpu-selftest.sh`): prints
`device: Apple iOS simulator GPU` and `SELFTEST PASS` — device init, shared-buffer
round-trip, runtime MSL compile + pipeline cache, dispatch, and fp32 saxpy parity.

**Integrated parity suite** (`scripts/run-sim-gpu-suite.sh`):
`== 18 passed, 0 failed ==`.

| Module | Checks (all PASS) |
|---|---|
| device | metal device available on simulator GPU |
| surface-eval | GPU Bezier grid evaluates · grid dimensions match request · GPU grid matches de Casteljau reference (fp32 rel ~1e-4) |
| bvh | LBVH builds on GPU · query path bound to the Metal backend · GPU closestHit batch succeeds · GPU nearest hit matches brute force (same id + t, fp32 tol) · nearest-hit selection resolves the stacked/miss scene as designed |
| pick | GpuPick engine compiles pipelines · engine driven by the Metal GPU · GPU batched ray-pick succeeds · GPU ray-pick matches CPU reference (same id + point, tol) · nearest-hit / miss resolved as designed |
| mesh-post | GPU per-vertex normals compute · one normal per input vertex · GPU normals match CPU reference per component (fp32) · GPU/CPU normal dot product ~ 1 (aligned unit vectors) |

The host regression (`CYBERCAD_HAS_METAL=OFF` CTest) stays **PASS**, confirming
the CPU-only path is untouched by the Metal target.

## Reproduce commands

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel

# Metal backend self-test on the iOS simulator GPU
bash scripts/run-sim-gpu-selftest.sh   # expect: "device: Apple iOS simulator GPU", "SELFTEST PASS", exit 0

# Integrated GPU-vs-CPU parity suite on the iOS simulator GPU
bash scripts/run-sim-gpu-suite.sh      # expect: "== 18 passed, 0 failed ==", exit 0

# Host (no-Metal) regression — CPU-only path unaffected
cmake -S . -B build \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF
cmake --build build
cd build && ctest --output-on-failure  # expect: 100% tests passed

# OpenSpec validation
openspec validate --all --strict       # expect: all changes pass
```

## VERIFIED vs FOLLOW-UP

### VERIFIED in this environment (iOS simulator GPU)

| Fact | Evidence |
|---|---|
| Metal device init + command queue on the simulator GPU | `run-sim-gpu-selftest.sh` → `device: Apple iOS simulator GPU` |
| Unified-memory (`StorageModeShared`) round-trip, runtime MSL compile + pipeline cache, dispatch, fp32 saxpy parity | `run-sim-gpu-selftest.sh` → `SELFTEST PASS` |
| CMake `CYBERCAD_HAS_METAL` option (default OFF); host CPU-only CTest stays green | host regression PASS |
| GPU surface-grid eval matches the CPU de Casteljau reference (fp32) | `run-sim-gpu-suite.sh` surface-eval checks |
| GPU LBVH build + stackless nearest-hit ray traversal matches CPU brute force (same id + t) | `run-sim-gpu-suite.sh` bvh checks |
| GPU batched ray-pick matches CPU reference (same id + point) | `run-sim-gpu-suite.sh` pick checks |
| GPU per-vertex normals match CPU reference per component (dot ≈ 1) | `run-sim-gpu-suite.sh` mesh-post checks |
| fp32-only backend refuses fp64; precision guard keeps fp64 on CPU | `metal_backend.mm` fp64 guard + Phase-0 `ComputeRegistry` routing |
| Integrated suite green | `== 18 passed, 0 failed ==` |

### FOLLOW-UP (not verified / not built here)

| Follow-up | Why it is not done here | Task(s) |
|---|---|---|
| **CPU triangulator** consuming GPU grids (trimming, grid→triangle connectivity, cross-face stitching) + a GPU-fed-mesh vs CPU-only-mesh parity check | only the design is documented in the headers; no triangulator component exists, so there is no mesh-level parity check | `gpu-tessellation` 2.1, 2.2, 6.1 |
| **`cc_tessellate` / `cc_face_meshes` integration** of the GPU eval path | the GPU surface-eval + mesh-normal modules are validated standalone; they are not wired into the OCCT facade path (no `src/facade` / `src/engine` references) | `gpu-tessellation` 4.3 |
| **Frustum-pick parity on the sim** | `frustumPick` / `pickFrustum` kernels + CPU references are coded, but the 18-check suite only asserts ray/nearest-hit parity — no frustum assertion is run | `spatial-acceleration` 3.2, 3.4, 6.1 |
| **Facade pick/cull wiring** for the GPU BVH/pick modules | modules gain the GPU option at the module level only; no OCCT-side pick/cull `cc_*` path exists to route through them today | `spatial-acceleration` 4.3 |
| **Explicit repeat-run determinism** assertions (surface eval, mesh normals, nearest-hit, frustum) | deterministic by construction (fixed grid decomposition, fixed CSR accumulation order, epsilon + lowest-index tie-break, sorted sets) but no repeat-run check is in the sim suite | `gpu-tessellation` 5.1, `spatial-acceleration` 5.1 |
| **On-device run** on physical Apple silicon | everything above ran on the booted **simulator** GPU only; nothing was run on hardware. The Phase-2 acceptance bar is the simulator, so this is optional | all Phase-2 changes |

The GPU compute backend and all four GPU modules are verified on the iOS
simulator GPU against independent CPU references (18/18, fp32 tolerance). What
remains before Phase 2 can be flipped to ✅ is the **OCCT `cc_tessellate`
integration** of GPU eval, the **CPU triangulator + GPU-fed-mesh check**, the
**frustum-pick suite leg**, and the **facade pick/cull wiring** — so the three
changes stay ◐ (in progress) in `ROADMAP.md`.

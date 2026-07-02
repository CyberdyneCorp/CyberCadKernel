# CyberCadKernel — Phase 2 status (GPU / Metal)

Honest, verification-anchored snapshot of Phase 2 (`add-metal-compute-backend`,
`add-gpu-tessellation`, `add-gpu-spatial-acceleration`). Nothing below is claimed
as working unless it was actually built and run in this environment. The
acceptance bar for Phase 2 (decided alongside the change specs) is the
**booted iOS simulator** running runtime-compiled MSL on the real
"Apple iOS simulator GPU", with every GPU result asserted against an independent
CPU reference within an fp32 tolerance. On-device (physical Apple silicon) runs
and OCCT `cc_tessellate` integration are explicit follow-ups.

Date: 2026-07-02 · Branch: `main`. Spatial-acceleration frustum-pick + determinism
leg landed (GPU pick suite 18 → 26).

## TL;DR

- **VERIFIED here (iOS simulator GPU):** the Metal compute backend self-test
  passes, and the integrated GPU-vs-CPU parity suite passes **26 / 26** checks —
  surface-grid evaluation, LBVH build + stackless nearest-hit ray traversal,
  batched ray-picking, **frustum-picking** (set parity vs CPU reference + sorted
  ordering), **repeat-run determinism** (ray + frustum byte-identical ×8), and GPU
  per-vertex mesh normals — each within an fp32 tolerance against an independent
  CPU reference. The host (no-Metal) CTest stays green (`CYBERCAD_HAS_METAL=OFF`),
  so the CPU-only path is unaffected.
- **NEW — VERIFIED here (iOS simulator):** the GPU eval path is now wired into
  **`cc_tessellate` / `cc_face_meshes`** behind the `cc_set_gpu_tessellation`
  toggle, with per-face eligibility routing (planar/untrimmed → GPU grid
  triangulator; holed/curved → OCCT fallback). The integration parity suite
  (`scripts/run-sim-integ-suite.sh`) is **26 / 26** PASS, comparing the GPU-fed
  mesh against the OCCT-only mesh (bbox + area + volume + watertightness) on an
  all-GPU box and a mixed GPU+OCCT slab. The GPU-OFF sim suite stays **221 / 221**.
  See "GPU tessellation integration" below.
- **NOT verified / not built here:** the **OPTIONAL additive `cc_*` pick/cull
  facade entry** (app-facing — no OCCT-side pick path exists in the facade today;
  out of scope for `add-gpu-spatial-acceleration`), and any **on-device** run.
  Holed / trimmed / curved
  faces **always fall back to OCCT by design** (not a gap — the GPU grid
  triangulator only handles single-outer-wire, UV-rectangular, low-degree faces).

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
`== 26 passed, 0 failed ==` (baseline 18 + 8 new frustum/determinism checks;
re-run confirmed identical 26 / 0).

| Module | Checks (all PASS) |
|---|---|
| device | metal device available on simulator GPU |
| surface-eval | GPU Bezier grid evaluates · grid dimensions match request · GPU grid matches de Casteljau reference (fp32 rel ~1e-4) |
| bvh | LBVH builds on GPU · query path bound to the Metal backend · GPU closestHit batch succeeds · GPU nearest hit matches brute force (same id + t, fp32 tol) · nearest-hit selection resolves the stacked/miss scene as designed |
| pick (ray) | GpuPick engine compiles pipelines · engine driven by the Metal GPU · GPU batched ray-pick succeeds · GPU ray-pick matches CPU reference (same id + point, tol) · nearest-hit / miss resolved as designed |
| pick (frustum) | GPU frustum-pick succeeds · GPU frustum set == CPU reference set (same ids) · frustum reference selects the known subset {0,1} · GPU frustum set is sorted ascending · empty-selection frustum returns {} (matches CPU) · all-enclosing frustum returns every id |
| determinism | GPU ray-pick byte-identical across 8 runs · GPU frustum-pick byte-identical across 8 runs |
| mesh-post | GPU per-vertex normals compute · one normal per input vertex · GPU normals match CPU reference per component (fp32) · GPU/CPU normal dot product ~ 1 (aligned unit vectors) |

The host regression (`CYBERCAD_HAS_METAL=OFF` CTest) stays **PASS**, confirming
the CPU-only path is untouched by the Metal target.

## GPU tessellation integration

The GPU surface-eval path is now stitched into the OCCT facade
(`cc_tessellate` / `cc_face_meshes`) behind the `cc_set_gpu_tessellation` toggle
(default OFF). Topology stays on the CPU; only per-face surface sampling is
GPU-eligible.

**Eligibility rule (per face).** A face routes to the GPU grid triangulator iff
it has a single outer wire, **no inner wires (no holes)**, its 2D boundary equals
the `BRepTools::UVBounds` rectangle within tolerance, and its surface converts to
a low-degree `SurfaceDef` (`degree <= kMaxSurfaceDegree`). Every other face —
holed, trimmed to a non-rectangular UV region, or high-degree/curved beyond the
converter — falls back to OCCT `BRepMesh_IncrementalMesh`. Ambiguous cases and any
`Standard_Failure` also fall back. Eligible faces are triangulated on a regular
`(u,v)` grid on the CPU, matching OCCT winding (`+u×+v`, flipped for a REVERSED
face), then stitched with the OCCT-fallback faces in the existing face-traversal
order — one `cc_face_meshes` slot per face id.

**Verified on the iOS simulator** (`scripts/run-sim-integ-suite.sh`,
`brep_available=1`, kernel built from source with OCCT + Metal), **26 / 26** PASS:

| Fixture | Routing (GPU / fallback) | GPU mesh | GPU-vs-OCCT parity | Watertight |
|---|---|---|---|---|
| Box 10×10×10 (all 6 faces planar/untrimmed) | **6 / 0** (all GPU) | v=600 t=972 | bbox match · area 600.000004 vs 600.000000 · enclosed vol 1000.000069 | yes (both paths) |
| Slab 20×20×10 w/ round hole (mixed) | **4 / 3** of 7 (4 planar walls GPU; 3 holed/curved → OCCT) | v=674 t=1048 | bbox match · area 1732.037370 vs 1732.037356 · vol 3720.000797 vs 3720.000673 | yes (both paths) |

The box exercises the all-GPU path; the slab proves the mixed router (`fallback>0`,
`gpu<total`) and that a GPU+OCCT stitched mesh is still watertight and matches the
OCCT-only mesh within fp32 tolerance.

**GPU-OFF parity is byte-identical.** With `gpuOn=false` (default) and no
`CYBERCAD_HAS_METAL`, `appendFaceMesh` delegates to `appendFaceTriangulation` with
unchanged face traversal/merge — no GPU dispatch. `scripts/run-sim-suite.sh` stays
**221 / 221** (rebuilt from current source, OCCT on / Metal off), and in the host
stub `cc_set_gpu_tessellation` is a verified safe runtime no-op
(`cc_gpu_tessellation_enabled()==0` before/after enable, no crash; CTest 7/7).

One infra fix landed with the integration: `scripts/run-sim-suite.sh` previously
skipped only `parity_bench.cpp`, but `tests/sim/` now also holds two standalone
GPU harnesses with their own `main()` + GPU/Metal includes (`metal_selftest.cpp`,
`integ_gpu_tess.cpp`) that were being swept into the OCCT-only compile and broke
it; the skip list was extended to exclude all three standalone harnesses.
(`metal_selftest.cpp`'s breakage was pre-existing at HEAD and independently
reproduced; the integration only compounded it.)

## Reproduce commands

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel

# Metal backend self-test on the iOS simulator GPU
bash scripts/run-sim-gpu-selftest.sh   # expect: "device: Apple iOS simulator GPU", "SELFTEST PASS", exit 0

# Integrated GPU-vs-CPU parity suite on the iOS simulator GPU
bash scripts/run-sim-gpu-suite.sh      # expect: "== 26 passed, 0 failed ==", exit 0

# GPU-tessellation INTEGRATION suite (GPU path wired into cc_tessellate vs OCCT)
bash scripts/run-sim-integ-suite.sh    # expect: "[ITEG] == 26 passed, 0 failed ==", exit 0

# GPU-OFF baseline (default) — full cc_* suite unchanged
bash scripts/run-sim-suite.sh          # expect: "== 221 passed, 0 failed ==", exit 0

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
| **GPU frustum-pick set == CPU reference set** (subset {0,1} / empty / all-enclosing, sorted ascending; guarded so equality is not trivially-true) | `run-sim-gpu-suite.sh` frustum checks |
| **GPU pick determinism** — ray-pick + frustum-pick **byte-identical across 8 runs** (memcmp, not within tolerance) | `run-sim-gpu-suite.sh` determinism checks |
| GPU per-vertex normals match CPU reference per component (dot ≈ 1) | `run-sim-gpu-suite.sh` mesh-post checks |
| fp32-only backend refuses fp64; precision guard keeps fp64 on CPU | `metal_backend.mm` fp64 guard + Phase-0 `ComputeRegistry` routing |
| Integrated suite green | `== 26 passed, 0 failed ==` |
| **GPU eval wired into `cc_tessellate` / `cc_face_meshes`** behind the toggle; per-face eligibility routing (planar/untrimmed → GPU, holed/curved → OCCT) | `run-sim-integ-suite.sh` box `gpu=6/0`, slab `gpu=4/3` |
| **GPU-fed mesh vs OCCT-only mesh parity** (bbox + area + volume + watertight) on box + mixed slab | `run-sim-integ-suite.sh` → `[ITEG] == 26 passed, 0 failed ==` |
| **GPU-OFF path byte-identical**; full `cc_*` suite unchanged with toggle OFF | `run-sim-suite.sh` → `== 221 passed, 0 failed ==` |
| `cc_set_gpu_tessellation` is a safe runtime no-op in the non-Metal stub | host stub build CTest 7/7; `cc_gpu_tessellation_enabled()==0` before/after enable |

### FOLLOW-UP (not verified / not built here)

| Follow-up | Why it is not done here | Task(s) |
|---|---|---|
| **Holed / trimmed / curved faces on the GPU** | *by design*, not a gap — the GPU grid triangulator only handles single-outer-wire, UV-rectangular, low-degree faces; everything else falls back to OCCT `BRepMesh` (slab: 3 of 7 faces). Trimmed/curved GPU tessellation is a Phase-4-native concern | `gpu-tessellation` 3.1 (documented) |
| **Explicit repeat-run determinism** assertion for the tessellation path | deterministic by construction (one GPU thread per grid sample, fixed CSR normal accumulation) but no repeat-run assertion runs in the integ suite | `gpu-tessellation` 6.1 |
| **OPTIONAL additive `cc_*` pick/cull facade entry** for the GPU BVH/pick modules | app-facing and **out of scope** for `add-gpu-spatial-acceleration` — the GPU BVH/pick modules are fully verified standalone; no OCCT-side pick/cull `cc_*` path exists in the facade today, so exposing one is a separate app-facing change | — (out of scope) |
| **Explicit repeat-run determinism** assertions (surface eval, mesh normals) | deterministic by construction (fixed grid decomposition, fixed CSR accumulation order) but no repeat-run check is in the sim suite. *(Spatial-acceleration determinism — nearest-hit + frustum, byte-identical ×8 — is now VERIFIED above.)* | `gpu-tessellation` 5.1 |
| **On-device run** on physical Apple silicon | everything above ran on the booted **simulator** GPU only; nothing was run on hardware. The Phase-2 acceptance bar is the simulator, so this is optional | all Phase-2 changes |

The GPU compute backend and all four GPU modules are verified on the iOS
simulator GPU against independent CPU references (18/18, fp32 tolerance), and the
GPU surface-eval path is now **integrated into `cc_tessellate` / `cc_face_meshes`**
behind the toggle, with GPU-fed-vs-OCCT mesh parity verified on the sim (integ
suite 26/26, GPU-OFF suite 221/221). `add-gpu-tessellation` is therefore complete
at the Phase-2 acceptance bar except for an explicit repeat-run determinism
assertion; GPU tessellation of holed/trimmed/curved faces is deferred **by design**
(they fall back to OCCT). `add-gpu-spatial-acceleration` is now **complete at the
Phase-2 acceptance bar** — the GPU pick suite is **26/26** with frustum-pick set
parity (vs CPU reference, subset / empty / all-enclosing, sorted ascending) and
byte-identical ×8 repeat-run determinism for both ray-pick and frustum-pick — so
that change is ✅ in `ROADMAP.md`. The only remaining spatial item is the
**OPTIONAL additive `cc_*` pick/cull facade entry** (app-facing, out of scope for
this change — no OCCT-side pick path exists in the facade today).

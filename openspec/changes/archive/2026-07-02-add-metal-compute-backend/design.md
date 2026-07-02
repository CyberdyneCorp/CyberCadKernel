## Context

Phase 0 defined `IComputeBackend` (`name` / `supports_fp64` / `parallel_for`),
the `ComputeRegistry` (register / set_active / active / cpu / `backend_for`), and
the precision guard that keeps fp64 work off any fp32-only backend
(`src/core/compute_backend.h`). Only the CPU backend exists so far. This change
adds the first GPU backend — Metal — as a drop-in `IComputeBackend`, plus the GPU
primitives (buffers, pipelines, dispatch) the sibling Phase-2 changes build on.

Verified environment facts this design relies on:
- Metal **compute** runs in the iOS simulator: a plain binary spawned via
  `xcrun simctl spawn` gets `MTLCreateSystemDefaultDevice()` = "Apple iOS
  simulator GPU", and `newLibraryWithSource:` compiles MSL at runtime.
- Simulator TUs compile with
  `xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator
  -isysroot "$(xcrun --sdk iphonesimulator --show-sdk-path)" -std=c++20 -fobjc-arc
  -framework Metal -framework Foundation`.
- Apple GPUs are **fp32 only** (no fp64 in hardware/MSL); exact modeling stays on
  the CPU fp64 core.

Constraints:
- **ABI stability**: no `cc_*` change; Metal lives entirely behind the Phase-0
  compute-backend seam.
- **Host build stays CPU-only**: `CYBERCAD_HAS_METAL` defaults OFF; the no-OCCT,
  no-Metal host build and its CTest must keep passing untouched.
- **No OCCT in GPU modules**: the Metal backend and its test suite are
  self-contained numeric code; they must not link OCCT.
- **Determinism by default** (`project.md`): fp32 GPU work is confined to
  display-tolerant paths; fp64 exact work never reaches the GPU.

## Goals / Non-Goals

Goals:
- A working `MetalComputeBackend : IComputeBackend`, iOS-only, that inits a
  device+queue, allocates shared (unified-memory) buffers, compiles MSL kernels
  at runtime into cached pipelines, dispatches compute over an index range, and
  registers itself with `ComputeRegistry`.
- An fp32 precision boundary: `supports_fp64() == false` and defensive refusal of
  fp64 job requests, so the Phase-0 guard keeps exact work on the CPU.
- Clean absence handling: no Metal device -> backend not registered -> callers
  transparently use the CPU backend.
- On-simulator tests that exercise device init, buffer round-trip, runtime
  pipeline compilation, dispatch parity vs a CPU reference, and the precision
  guard.

Non-Goals (delivered by sibling Phase-2 changes):
- GPU surface evaluation / tessellation / mesh normals (`add-gpu-tessellation`).
- GPU LBVH build, ray traversal, picking (`add-gpu-spatial-acceleration`).
- Non-Apple GPU backends (CUDA / OpenCL / Vulkan) — later phases.
- `.metallib` precompilation, on-device benchmarking, or app link-swap.

## Decisions

- **`.mm` Objective-C++, iOS-only, behind `CYBERCAD_HAS_METAL` (default OFF).**
  Metal types stay out of portable C++ headers; the public backend header exposes
  only the `IComputeBackend` surface plus a factory (`makeMetalBackend()` returning
  `nullptr` when unavailable). Metal/Foundation are linked only for iOS targets.
- **Device + command queue owned by the backend instance.** Acquire
  `MTLCreateSystemDefaultDevice()` and one `MTLCommandQueue` at construction; if
  the device is null the factory returns null and nothing registers.
- **Unified-memory shared buffers.** Allocate `MTLResourceStorageModeShared` so
  the CPU can fill/read the same allocation the GPU uses with no blit — the
  natural fit for Apple Silicon and the simulator, and what the tessellation/BVH
  data paths need.
- **Runtime MSL compilation with a pipeline cache.** Kernel source is embedded as
  C++ string literals; `newLibraryWithSource:` + `newComputePipelineStateWithFunction:`
  build a `MTLComputePipelineState`, cached by kernel-function name so repeated
  dispatches don't recompile. Compilation errors surface as a `Result` error, not
  a crash.
- **Dispatch over an index range.** A helper encodes a compute command, binds
  buffers, computes threadgroup sizing from
  `maxTotalThreadsPerThreadgroup`, dispatches `count` threads, commits and waits.
  `parallel_for` is honoured for interface conformance, but the real GPU value is
  the buffer/pipeline/dispatch primitives the sibling changes call.
- **fp32-only guard, enforced twice.** `supports_fp64()` returns false (so
  `ComputeRegistry::backend_for(Fp64)` already diverts to CPU), and the backend
  additionally rejects any explicitly fp64-typed job as a defensive invariant.
- **CPU stays default + fallback.** Registering Metal does not change the active
  backend selection contract; the CPU backend remains registered and is the
  fallback whenever Metal is absent or a job is fp64.

## Risks / Trade-offs

- **GPU availability varies.** Absent/failed device init must be a clean "not
  registered", never a hard failure — otherwise CPU-only builds regress. Covered
  by an explicit requirement + test.
- **Runtime compile cost.** `newLibraryWithSource:` is not free; mitigated by
  caching pipelines per kernel name and compiling lazily on first use.
- **Simulator vs device parity.** The simulator GPU validates the API path and
  fp32 parity, not real device throughput; on-device benchmarking is an explicit
  non-goal here (deferred, mirroring Phase 0/1 acceptance).
- **fp32 precision.** GPU results differ from fp64 CPU at the fp32 ULP level; this
  is acceptable only for display-tolerant work and is enforced by the guard —
  documented, with the tolerance owned by the sibling algorithm changes.

## Migration Plan

1. Add `CYBERCAD_HAS_METAL` CMake option (default OFF) and the `.mm` target,
   linked to Metal + Foundation for iOS only; host build unchanged.
2. Implement device/queue init + `makeMetalBackend()` factory (null when
   unavailable).
3. Implement shared-buffer alloc and a CPU<->GPU round-trip.
4. Implement runtime MSL compile + pipeline cache and a dispatch helper.
5. Implement `IComputeBackend` (name/`supports_fp64`=false/`parallel_for`) and
   fp64 refusal; register with `ComputeRegistry` when available.
6. Add the on-simulator test suite (init, buffer round-trip, compile, dispatch
   parity vs CPU, precision guard); keep it OCCT-free.
7. `openspec validate --all --strict`; update `ROADMAP.md` Phase 2 status.

## Open Questions

- Threadgroup-sizing heuristic (fixed vs pipeline-derived
  `maxTotalThreadsPerThreadgroup`) — to be tuned with the sibling algorithm
  changes on real workloads.
- Whether pipeline caching should persist across backend instances (process-wide
  cache) or stay per-instance; per-instance chosen first for simplicity.

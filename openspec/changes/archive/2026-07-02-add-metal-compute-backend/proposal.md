## Why

Phase 0 (`add-kernel-foundation`) shipped the `IComputeBackend` interface, the
`ComputeRegistry`, and the precision guard (`backend_for(Precision)`) with only a
CPU/multi-core backend behind them (`src/core/compute_backend.h`). The
abstraction was designed "from day one" so a GPU path can be added without
touching callers, the `cc_*` facade, or the engine adapter (`project.md`
§Compute-backend abstraction). Phase 2 begins the GPU work, and Metal is the only
GPU path on iOS — CyberCad's first-class target.

Everything the higher Phase-2 changes need (GPU tessellation, GPU BVH/picking)
depends on a concrete GPU backend existing first: a way to init a device,
allocate buffers, compile kernels, dispatch compute, and register itself so the
precision guard routes fp32-tolerant work to it while keeping exact fp64 modeling
on the CPU. This change delivers exactly that Metal backend and nothing
higher-level — the algorithms sit in the two sibling changes.

Metal compute is verifiable in this environment on the **iOS simulator GPU**:
`MTLCreateSystemDefaultDevice()` returns "Apple iOS simulator GPU" and
`newLibraryWithSource:` (runtime MSL compilation) works, so every requirement
here is testable on-simulator without physical hardware.

## What Changes

- Add a **Metal compute backend** (`MetalComputeBackend`) implementing the
  Phase-0 `IComputeBackend` interface: `name()`, `supports_fp64()` and
  `parallel_for`, plus the GPU primitives the sibling changes call
  (buffer alloc, pipeline get-or-compile, dispatch).
- **Device initialization**: acquire the system default Metal device and a
  command queue at construction; fail cleanly (no registration) when no Metal
  device is available so the CPU-only build path is unaffected.
- **Unified-memory (shared) buffers** on Apple Silicon: allocate
  `MTLResourceStorageModeShared` buffers so CPU and GPU address the same memory
  with no explicit copy on the tessellation/BVH data path.
- **Runtime MSL pipeline compilation**: MSL kernels are embedded as C++ string
  literals and compiled at runtime with `newLibraryWithSource:` into cached
  `MTLComputePipelineState` objects (no `.metallib` precompile step).
- **Compute dispatch**: encode a compute command, bind buffers, dispatch a
  threadgroup grid over an index range, commit, and wait — the GPU realization of
  a data-parallel `parallel_for`.
- **Registry integration**: register the Metal backend with `ComputeRegistry`
  when available; the CPU backend stays the default and fallback.
- **fp32 precision guard**: the backend reports `supports_fp64() == false`, so the
  existing `backend_for(Precision::Fp64)` guard never routes exact work to it; the
  backend also refuses fp64 job requests defensively.
- **Build**: a new CMake option `CYBERCAD_HAS_METAL` (default **OFF**). The
  Metal backend is Objective-C++ (`.mm`), iOS-only, and does **not** link OCCT.
  The host build (`CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) stays
  CPU-only and keeps passing its CTest.

No `cc_*` signature changes: the facade and engine adapter are untouched; the
Metal backend plugs in entirely behind the Phase-0 compute-backend seam.

## Capabilities

### New Capabilities
- `metal-backend`: a Metal-based `IComputeBackend` implementation — device/queue
  init, unified-memory shared buffers, runtime MSL pipeline compilation +
  caching, compute dispatch over an index range, self-registration in
  `ComputeRegistry`, and an fp32-only precision boundary. Depends on
  `compute-backend` (the Phase-0 interface, registry, and precision guard).

### Modified Capabilities
<!-- none — metal-backend is additive; it implements the existing compute-backend
     interface without changing its contract. -->

## Impact

- **Interface**: implements `src/core/compute_backend.h`'s `IComputeBackend` and
  registers via `ComputeRegistry`; no interface change, purely a new backend.
- **App**: no code change — the facade never sees Metal; the backend accelerates
  eligible fp32 work transparently and falls back to CPU when unavailable.
- **Build**: new option `CYBERCAD_HAS_METAL` (default OFF); `.mm` sources built
  only for iOS targets, linked against `Metal` + `Foundation`, never OCCT. Host
  CPU-only build and CTest unchanged.
- **Determinism**: GPU work is fp32 and confined to display-tolerant paths (the
  sibling changes); exact fp64 modeling stays on CPU per the precision guard.
- **Risk**: GPU availability varies; the backend must degrade to "not registered"
  cleanly so no capability breaks when Metal is absent.

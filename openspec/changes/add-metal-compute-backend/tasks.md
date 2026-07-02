# Tasks — add-metal-compute-backend

Verification levels: **host** = no-OCCT / no-Metal host CTest stays green
(`CYBERCAD_HAS_METAL=OFF`, CPU-only, unchanged); **ios-sim-build** = the `.mm`
Metal backend compiles/links for `arm64-apple-ios16.0-simulator` with
`-framework Metal -framework Foundation` (no OCCT); **ios-sim-run** = the backend
runs in a booted simulator via `xcrun simctl spawn`, where
`MTLCreateSystemDefaultDevice()` = "Apple iOS simulator GPU" and
`newLibraryWithSource:` compiles MSL at runtime — this is the acceptance bar for
every GPU-vs-CPU parity check below.

## 1. Build wiring
- [x] 1.1 Add CMake option `CYBERCAD_HAS_METAL` (default OFF); host build
  (`CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) stays CPU-only and its
  CTest keeps passing. (**host**)
- [x] 1.2 Add the `.mm` Objective-C++ target, iOS-only, linked to `Metal` +
  `Foundation`, and assert it does **not** link OCCT. (**ios-sim-build**)
- [x] 1.3 Public header exposes only `IComputeBackend` + a `makeMetalBackend()`
  factory; no Metal type crosses the header. (**ios-sim-build**)

## 2. Device initialization
- [x] 2.1 Acquire the system default Metal device + a command queue at
  construction. (**ios-sim-run**)
- [x] 2.2 `makeMetalBackend()` returns null (backend not registered) when no
  Metal device is available; CPU path unaffected. (**host** + **ios-sim-run**)

## 3. Unified-memory (shared) buffers
- [x] 3.1 Allocate `MTLResourceStorageModeShared` buffers for input/output. (**ios-sim-run**)
- [x] 3.2 CPU-write -> GPU-read -> CPU-read round-trip returns the written bytes
  with no explicit copy. (**ios-sim-run**)

## 4. Runtime MSL pipeline compilation
- [x] 4.1 Compile an embedded MSL kernel string via `newLibraryWithSource:` into
  a `MTLComputePipelineState`. (**ios-sim-run**)
- [x] 4.2 Cache pipelines by kernel-function name; second dispatch reuses the
  cached pipeline (no recompile). (**ios-sim-run**)
- [x] 4.3 A malformed kernel source surfaces a `Result` error, not a crash. (**ios-sim-run**)

## 5. Compute dispatch
- [x] 5.1 Encode/bind/dispatch a compute grid over `[0, count)` with
  threadgroup sizing from `maxTotalThreadsPerThreadgroup`; commit + wait. (**ios-sim-run**)
- [x] 5.2 A trivial elementwise fp32 kernel (e.g. saxpy) matches a CPU reference
  within fp32 tolerance for the full index range. (**ios-sim-run**)
- [x] 5.3 Implement `parallel_for(count, body)` for interface conformance. (**ios-sim-run**)

## 6. Registry integration + precision guard
- [x] 6.1 Register the Metal backend with `ComputeRegistry` when available; CPU
  stays default + fallback. (**ios-sim-run**)
- [x] 6.2 `supports_fp64()` returns false; `backend_for(Fp64)` routes to the CPU
  backend, never Metal. (**host** + **ios-sim-run**)
- [x] 6.3 The backend defensively refuses an explicitly fp64 job request. (**ios-sim-run**)

## 7. Validation
- [x] 7.1 On-simulator suite: device init, buffer round-trip, runtime compile,
  dispatch parity vs CPU, precision guard — all green in a booted sim. (**ios-sim-run**)
- [x] 7.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 2 +
  change index for `metal-backend`.

> **Status (2026-07-02):** all tasks verified at the **ios-sim-run** acceptance
> bar. `scripts/run-sim-gpu-selftest.sh` prints `device: Apple iOS simulator GPU`
> and `SELFTEST PASS` (device init, `MTLResourceStorageModeShared` round-trip,
> `newLibraryWithSource` runtime compile + pipeline cache, dispatch, fp32 saxpy
> parity), and the CMake `CYBERCAD_HAS_METAL` option (default OFF) keeps the host
> CPU-only CTest green (host regression PASS). **On physical Apple hardware** the
> backend is not exercised here — the task acceptance bar is the booted simulator,
> not a device, so no task is blocked; a real-device run remains an optional
> follow-up.

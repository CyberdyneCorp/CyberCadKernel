## Why

CyberCad talks to geometry through a plain-C facade (`cc_*`) with two engines
behind it: a Swift preview kernel and OCCT compiled into the app. To evolve the
kernel — accelerate it (multi-core + GPU), add features OCCT lacks (G2 fillet,
robust thread boolean), and ultimately replace OCCT with a native C++20
implementation — that logic needs to live in a **dedicated, portable library**
with clean seams, not inline in the app's Objective-C++ bridge. This change
stands up that library, `CyberCadKernel`, wrapping OCCT unchanged in behaviour so
the app keeps working while future phases migrate capability-by-capability.

## What Changes

- Introduce the **CyberCadKernel** C++20 library with a stable plain-C ABI
  identical to CyberCad's `KernelBridgeAPI.h` (`cc_*`, integer shape handles, POD
  structs). The app links this library instead of embedding the OCCT bridge.
- Add an **engine-adapter** seam so the geometry engine is pluggable; ship an
  **OCCT adapter** as the first (and initially only) implementation, preserving
  today's behaviour and the guard/`IsDone`/`IsValid` error model.
- Add an **operation scheduler** that runs kernel operations off the UI thread as
  cancellable, progress-reporting coroutine tasks — replacing today's blocking,
  non-cancellable `Build`.
- Add a **compute-backend interface** abstracting GPU/parallel compute
  (Metal/CUDA/OpenCL/Vulkan), with a CPU/no-op backend as the default so nothing
  yet depends on a GPU.
- No behaviour change for the app in this phase: same `cc_*` results, now served
  by the library. Acceleration, new features, and native rewrite are later
  changes (see `ROADMAP.md`).

## Capabilities

### New Capabilities
- `kernel-facade`: the stable plain-C ABI, shape registry (integer handles), and
  exception-to-status guard/error model that the host app depends on.
- `engine-adapter`: pluggable geometry-engine abstraction; the OCCT-backed
  adapter as first implementation, allowing native implementations to coexist and
  be compared behind the same facade call.
- `operation-scheduler`: asynchronous, cancellable, progress-reporting execution
  of kernel operations off the UI thread.
- `compute-backend`: abstraction over GPU/parallel compute backends
  (Metal/CUDA/OpenCL/Vulkan) for fp32-tolerant data-parallel work, with a default
  CPU backend.

### Modified Capabilities
<!-- none — this is a greenfield library; nothing exists yet -->

## Impact

- **New repo**: `CyberdyneCorp/CyberCadKernel` (MIT). Wrapping OCCT (LGPL-2.1 +
  exception) carries the usual static-relink obligation until the native rewrite
  removes OCCT.
- **CyberCad app**: eventually links `CyberCadKernel` in place of the in-app OCCT
  bridge; the `cc_*` call sites and the Swift `PreviewKernel` fallback are
  unaffected (ABI-compatible).
- **Build**: CMake library cross-compiled for arm64 iOS device + simulator now;
  desktop/Android later. OCCT remains an external static dependency for this
  phase.
- **Contract source**: mirrors `cybercad/openspec/specs/occt-usage/spec.md`
  (the exact OCCT surface the app relies on).

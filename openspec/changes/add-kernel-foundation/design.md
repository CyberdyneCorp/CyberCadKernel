## Context

CyberCad currently embeds OCCT behind an Objective-C++ bridge (`KernelBridge.mm`)
exposing a plain-C facade (`KernelBridgeAPI.h`). This change extracts and
re-homes that boundary into a standalone C++20 library so future phases
(multi-core, GPU, native rewrite) have clean seams. This phase is behaviour-
preserving: same `cc_*` results, now served by the library with OCCT wrapped.

Constraints:
- **ABI stability**: `cc_*` must stay binary-compatible with the app's current
  header; the Swift `PreviewKernel` fallback must remain usable.
- **Portability**: CMake, arm64 iOS device + simulator now; desktop/Android on
  the roadmap → backend abstractions must not assume Apple.
- **Precision**: exact modeling is fp64 on CPU; Apple GPUs have no fp64.
- **License**: library MIT; OCCT LGPL-2.1+exception stays behind the adapter.

## Goals / Non-Goals

Goals:
- Establish four seams: `kernel-facade`, `engine-adapter`, `operation-scheduler`,
  `compute-backend`.
- Ship an OCCT adapter at behavioural parity with today's bridge.
- Provide cancellation + progress for long operations.

Non-Goals (later changes):
- Any GPU backend implementation (only the interface + CPU default here).
- Enabling OCCT parallelism / tuning booleans (Phase 1).
- New algorithms OCCT lacks (Phase 3).
- Native replacement of any OCCT capability (Phase 4).

## Decisions

- **Facade stays plain C, mirrored from `occt-usage`.** The contract is the exact
  set of `cc_*` functions the app relies on. Additive-only evolution.
- **Engine interface is internal C++20**, one method group per capability
  (construct / boolean / fillet / tessellate / query / transform / exchange). The
  OCCT adapter implements all of it; a native adapter can implement a subset and
  fall through to OCCT during migration.
- **Scheduler uses C++20 coroutines + `std::stop_token`.** A `Task<T>` type with a
  cancellation token and a progress sink. Where the underlying engine call is not
  interruptible (most OCCT ops), the scheduler runs it on a worker and honours
  cancellation at the boundary (discard result, reclaim resources) — the spec's
  "cancellation-safe engine boundary" requirement.
- **Result type**: `std::expected` is C++23; use an in-house `Result<T,Error>`
  (or `tl::expected`) on C++20. The C facade continues to collapse this to
  0/nil + `cc_last_error`.
- **Compute backend is a capability-registry**: tasks are expressed abstractly;
  a backend (CPU default, Metal/CUDA/OpenCL/Vulkan later) is chosen at build or
  runtime. fp64 work is never dispatched to an fp32-only backend.
- **No engine types in public headers**; OCCT headers are included only in
  adapter `.cpp` translation units.

## Risks / Trade-offs

- **Cancellation of non-interruptible OCCT ops** is cooperative-at-boundary only;
  a running fine-thread boolean still consumes CPU until it returns. Mitigated by
  Phase 1 (parallel booleans) and Phase 3 (thread-specific boolean); documented
  as a known limitation.
- **ABI drift** between the app header and the library. Mitigation: treat
  `occt-usage` as the single source of truth and add an ABI contract test.
- **C++20 toolchain** under Xcode/Clang for iOS: avoid modules initially
  (headers only); verify coroutine + `<stop_token>` availability on the
  deployment target.

## Migration Plan

1. Create the CMake library skeleton (arm64 iOS device + sim) linking the trimmed
   OCCT static libs the app already builds.
2. Port the `cc_*` implementations from `KernelBridge.mm` into the OCCT adapter,
   unchanged in behaviour, behind the facade + guard model.
3. Introduce the scheduler and route long ops through it (cancellation/progress);
   keep synchronous shims where the app still calls synchronously.
4. Add the compute-backend interface + CPU default (no GPU yet).
5. Point the CyberCad app at the library (link swap) behind its existing OCCT
   build flag; verify parity against the in-app bridge; keep PreviewKernel.

## Open Questions

- Exact `Task<T>` shape and how the Swift layer awaits it (callback vs polled
  handle vs Swift concurrency interop).
- Whether STEP/IGES data exchange stays on OCCT longest (likely yes) and how that
  interacts with the eventual "drop OCCT" milestone.

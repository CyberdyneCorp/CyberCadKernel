# CyberCadKernel — Phase 0 + Phase 1 status

Honest, verification-anchored snapshot of the two scaffolded phases
(`add-kernel-foundation`, `accelerate-multicore-occt`). Nothing below is claimed
as working unless it was actually built/run in this environment; runtime OCCT
behaviour, device builds, and app integration are explicitly listed as
follow-ups.

Date: 2026-07-01 · Branch: `main`.

## TL;DR

- **VERIFIED here:** the host (no-OCCT) library builds and all 6 unit tests pass;
  the full OCCT engine adapter **compiles, archives, and link-checks** for the
  iOS **simulator** (arm64) against the real trimmed OCCT static libs.
- **NOT verified here:** OCCT-backed **runtime** behaviour / parity vs the app's
  `KernelBridge.mm`, the **arm64 device** build, and the **app link-swap**. These
  need the iOS simulator/device and the CyberCad app project.

## What was implemented

Static C++20 library behind a stable plain-C ABI (`cc_*`), two build configs:
`CYBERCAD_HAS_OCCT=OFF` (host, stub engine) and `=ON` (iOS, OCCT adapter).

Phase 0 (`add-kernel-foundation`):

- **kernel-facade** — `include/cybercadkernel/cc_kernel.h` (57 `cc_*`, POD
  structs, `CCShapeId`, `0 = invalid`); `src/facade/cc_kernel.cpp` guards and
  delegates every entry point.
- **shape registry** — `src/core/shape_registry.*`, thread-safe `CCShapeId <->`
  opaque `EngineShape`, `cc_shape_release`.
- **guard / error model** — `src/core/guard.*`: `guard()`/`guard_void()` catch
  `std::exception` (and, in OCCT TUs, `Standard_Failure`, which does not derive
  from `std::exception`) → `0/nil` + per-thread `cc_last_error`.
- **engine adapter** — `src/engine/IEngine.h` (capability-grouped interface);
  `src/engine/engine_registry.cpp` (active-engine selector, OCCT/native
  coexist); `src/engine/stub/` (no-op host engine); `src/engine/occt/` (~3.4k LOC
  OCCT adapter: construct / boolean+transform / feature / tessellate / query /
  exchange).
- **operation scheduler** — `src/core/scheduler.*`: coroutine `Task<T>`, a
  `std::thread` worker pool, in-house `StopToken`/`StopSource` cancellation, and
  a staged/0..1 progress sink. `std::jthread`/`std::stop_token` are gated out
  under Apple Clang libc++, so an in-house equivalent is used (documented in the
  header).
- **compute backend** — `src/core/compute_backend.*`: `IComputeBackend`, default
  fp64 CPU backend, registration hook for GPU backends (interface only), and a
  precision guard that refuses to route fp64 work to an fp32-only backend.
- **Result type** — `src/core/result.h` (in-house `Result<T,Error>`;
  `std::expected` is C++23), collapsed to `0/nil + cc_last_error` at the C facade.

Phase 1 (`accelerate-multicore-occt`), all inside the OCCT adapter + policy layer:

- **parallel policy** — `src/engine/occt/parallel_policy.*`: worker-cap over
  `OSD_ThreadPool`, global `parallel` toggle + per-op override, scheduler routing
  (`runScheduled`), and the fine-thread boolean **gate** (`evaluateGate` /
  `checkFineThreadGate`, decision surfaced via `cc_last_error`).
- **parallel booleans** — `occt_booltransform.cpp`: `SetRunParallel(true)` on
  fuse/cut/common, tuned `SetFuzzyValue`, unchanged `IsValid()`/volume gate.
- **parallel meshing** — `occt_tessellate.cpp`: `BRepMesh_IncrementalMesh`
  `InParallel` behind `cc_tessellate` / `cc_face_meshes`.
- **cancellable long ops** — booleans + meshing routed through the Phase-0
  scheduler; non-interruptible `Build` handled cancellation-safely (result
  discarded on cancel).

## Host tests that pass (no-OCCT)

`ctest` → **6/6 pass**: `test_registry`, `test_guard`, `test_scheduler`,
`test_compute_backend`, `test_parallel_policy`, `test_abi`. `test_abi`
`static_assert`s the size/offsets of every POD struct against the app's real
`KernelBridgeAPI.h` (binary ABI compatibility).

## What compiles for iOS-sim (with OCCT)

`scripts/verify-ios-compile.sh` → **15 translation units compiled**
(`arm64-apple-ios-simulator`, `-DCYBERCAD_HAS_OCCT`, facade + core + engine incl.
`engine/occt/*`), archived into `build-ios/libcybercadkernel.a`, and a
**link check** (`cc_brep_available()` main) links against the archive + the TK*
libs and produces an executable. This proves the adapter compiles against real
OCCT headers and the objects archive/link; it does **not** run anything (the OCCT
libs are simulator slices, not host libs).

## Reproduce commands

Host build + test (no OCCT):

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel
cmake -S . -B build \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCYBERCAD_HAS_OCCT=OFF
cmake --build build
cd build && ctest --output-on-failure     # expect: 100% tests passed, 6/6
```

iOS-simulator OCCT compile + archive + link check:

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel
bash scripts/verify-ios-compile.sh        # expect: "RESULT: COMPILE+ARCHIVE OK"
                                          #         "Link check: LINK OK"
```

OpenSpec validation:

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel
openspec validate --all --strict          # expect: 2 passed, 0 failed
```

## VERIFIED vs FOLLOW-UP

### VERIFIED in this environment

| Fact | Evidence |
|---|---|
| Host (no-OCCT) library builds with Homebrew LLVM clang++ | `cmake --build build` clean |
| 6/6 host unit tests pass | `ctest` → 100% passed, 6/6 |
| ABI matches the app's `KernelBridgeAPI.h` (sizes/offsets) | `test_abi` passes with the reference header present |
| OCCT adapter compiles for arm64 iOS-**simulator** | `verify-ios-compile.sh` → 15 TUs compiled |
| Adapter objects archive + link against real trimmed OCCT | same script → archive built, link check `LINK OK` |
| `openspec validate --all --strict` is green | 2 passed, 0 failed |

### FOLLOW-UP (not verified here)

| Follow-up | Why it is not done here | Blocks |
|---|---|---|
| **OCCT runtime parity** — each `cc_*` vs the in-app `KernelBridge.mm` | host build is stub-only; OCCT libs are simulator slices (cannot run on host) | needs the iOS **simulator** | tasks `add-kernel-foundation` 3.4, 6.2; `accelerate` 3.2, 5.1–5.2, 6.2 |
| **Parallel determinism audit + on-device benchmark** | needs to run parallel vs serial on real hardware | needs **device/simulator** | `accelerate` 5.1–5.2, 6.1 |
| **arm64 iOS device build** | script builds the simulator slice only | needs the **device** SDK/toolchain pass | `add-kernel-foundation` 1.1 |
| **App link-swap** — point CyberCad at this library behind its OCCT flag, keep PreviewKernel fallback | needs the CyberCad **app** project | `add-kernel-foundation` 6.1, 6.2 |

Because runtime parity, the device build, and the app link-swap are all open,
**Phase 0 and Phase 1 stay ◐ (in progress)** in `ROADMAP.md` — Phase 0 is NOT
flipped to ✅ (see `add-kernel-foundation` task 6.4).

## Notes / deviations

- Scheduler uses `std::thread` + an in-house `StopToken`/`StopSource` instead of
  `std::jthread`/`std::stop_token`, which Apple Clang's libc++ gates off. The
  capability (off-UI-thread execution, cooperative cancel, progress) is
  unchanged; the substitution is documented in `src/core/scheduler.h`.
- The iOS link check emits `ld` warnings that the OCCT `.o` files were built for
  a newer simulator version (18.0) than the link target (14.0). These are
  warnings only; the link succeeds.

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
  iOS **simulator and device** (arm64) against the real trimmed OCCT static libs
  (packaged as `CyberCadKernel.xcframework`); and the adapter **runs on the iOS
  simulator** — 16 OCCT-backed correctness checks pass, the parallel paths are
  **run-to-run deterministic**, and a boolean/mesh benchmark is captured.
- **NOT verified here:** **full** per-`cc_*` parity across all 57 functions and
  vs the app's `KernelBridge.mm`, **on-device runtime**, serial-vs-parallel A/B
  timing, and the **app link-swap** (xcframework is built; the CyberCad project
  edit + app build still need Xcode).

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

## Packaged xcframework (device + simulator)

`scripts/build-xcframework.sh` compiles the whole WITH-OCCT library for **both**
`arm64-apple-ios` (device) and `arm64-apple-ios-simulator`, archives each slice,
and packages `build-ios/CyberCadKernel.xcframework` (`ios-arm64` +
`ios-arm64-simulator`, headers bundled, `cc_brep_available` exported). This is the
artifact the CyberCad app links in place of its inline `KernelBridge.mm` (the
link-swap). It is a build artifact and is gitignored — rebuild locally.

## Runtime verification (iOS simulator)

`scripts/run-sim-harness.sh` compiles `tests/sim/parity_bench.cpp`, links it
against the simulator slice + the real OCCT libs, and runs it **inside a booted
simulator** (`xcrun simctl spawn`). Result: **16 / 16 checks pass**.

- **Correctness (analytic):** a 10×10×10 box → volume `1000`, area `600`, bbox
  `[0,0,0,10,10,10]`, 12 edges; `fuse`/`cut`/`common` of overlapping boxes →
  `1875` / `875` / `125` exactly; `cc_fillet_edges` → valid; **STEP export +
  re-import** preserves volume `1000`.
- **Determinism (Phase 1 audit, tested cases):** the parallel `fuse`+mesh and the
  parallel box mesh are **byte-identical across 16 runs** (FNV-1a over the mesh) —
  parallel meshing/booleans are run-to-run reproducible for these bodies.
- **Benchmark:** boolean fuse ≈ **5.9 ms/op** (avg of 50); fine mesh
  (deflection 0.01) ≈ **1.1 ms**. Absolute only — serial-vs-parallel A/B needs an
  additive `cc_*` parallel toggle (follow-up).

**Bug found + fixed (regression covered by this harness):** the process
segfaulted on **exit** after running OCCT algorithms. Root cause: the facade's
`ShapeRegistry` and the active-engine `shared_ptr` were static singletons whose
destructors freed `TopoDS_Shape`s during static destruction, racing OCCT's own
static teardown. Fix: both process-wide singletons are now **intentionally
leaked** (`src/facade/cc_kernel.cpp`, `src/engine/engine_registry.cpp`) so no
OCCT object of ours is freed at exit. A residual crash from OCCT's *own* static
teardown after algorithm use remains (isolated: a build that runs no OCCT
algorithm exits cleanly); it is an OCCT-upstream quirk of the trimmed static
build, harmless to the app (post-`main`), and the harness `std::_Exit`s with its
true result. The harness now exits `0` — a re-introduced teardown crash would
resurface as a non-zero/`signal 11` exit.

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

Package the xcframework (device + simulator) and run the OCCT harness in a
booted simulator:

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel
bash scripts/build-xcframework.sh         # -> build-ios/CyberCadKernel.xcframework
bash scripts/run-sim-harness.sh           # expect: "== 16 passed, 0 failed ==", exit 0
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
| OCCT adapter compiles for arm64 iOS-**simulator** + **device** | `verify-ios-compile.sh` (sim) + `build-xcframework.sh` (both slices) |
| Adapter objects archive + link against real trimmed OCCT | archive built, link check `LINK OK`; `CyberCadKernel.xcframework` produced |
| OCCT adapter **runs** on the iOS simulator: 16/16 correctness checks | `run-sim-harness.sh` → exact box/boolean volumes, STEP round-trip |
| Parallel booleans + meshing are **run-to-run deterministic** (tested bodies) | same harness → mesh byte-identical across 16 runs |
| `openspec validate --all --strict` is green | 2 passed, 0 failed |

### FOLLOW-UP (not verified here)

| Follow-up | Why it is not done here | Blocks |
|---|---|---|
| **Full per-`cc_*` parity** — all 57 functions, and vs the in-app `KernelBridge.mm` output | the harness covers core ops (extrude/translate/boolean/fillet/mass-props/bbox/subshape/tessellate/STEP), not every function or a bridge-vs-library diff | needs a broader sim harness / app test target | `add-kernel-foundation` 3.4, 6.2; `accelerate` 6.2 |
| **Comprehensive determinism audit + serial-vs-parallel A/B** | harness proves parallel run-to-run reproducibility on box cases; a serial baseline needs an additive `cc_*` parallel toggle, and more representative bodies | needs the toggle hook + more cases | `accelerate` 5.1–5.2 |
| **On-device runtime + benchmark** | the device **slice compiles/links** (xcframework), but was not **run** on hardware | needs a physical device | `accelerate` 6.1 |
| **App link-swap** — point CyberCad at `CyberCadKernel.xcframework` behind its OCCT flag, keep PreviewKernel fallback | the xcframework is built; the `project.yml` edit + app build need Xcode | needs the CyberCad **app** project | `add-kernel-foundation` 6.1, 6.2 |

Core OCCT runtime is now verified on the simulator, but full parity, the app
link-swap, and on-device runs remain open, so **Phase 0 and Phase 1 stay ◐
(in progress)** in `ROADMAP.md` — Phase 0 is NOT flipped to ✅ (see
`add-kernel-foundation` task 6.4).

## Notes / deviations

- Scheduler uses `std::thread` + an in-house `StopToken`/`StopSource` instead of
  `std::jthread`/`std::stop_token`, which Apple Clang's libc++ gates off. The
  capability (off-UI-thread execution, cooperative cancel, progress) is
  unchanged; the substitution is documented in `src/core/scheduler.h`.
- The iOS link check emits `ld` warnings that the OCCT `.o` files were built for
  a newer simulator version (18.0) than the link target (14.0). These are
  warnings only; the link succeeds.

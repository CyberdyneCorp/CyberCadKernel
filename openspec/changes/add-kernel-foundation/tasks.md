# Tasks — add-kernel-foundation

Verification levels used below: **host** = no-OCCT host build + CTest (6/6 pass);
**ios-sim** = OCCT adapter compiles + archives + link-checks for
`arm64-apple-ios-simulator` (`scripts/verify-ios-compile.sh`, 15 TUs). Items that
need a running simulator/device or the app project are left `[ ]` with a note.

## 1. Library skeleton & build
- [x] 1.1 CMake C++20 static library `CyberCadKernel`; targets arm64 iOS device + simulator. — builds on **host** (no-OCCT) and compiles/archives for **ios-sim**; the arm64-device slice uses the same toolchain path but is NOT built here (follow-up).
- [x] 1.2 Link the trimmed OCCT static libs (as the app builds today); OCCT headers private to adapter TUs only. — **ios-sim** link check links the TK* libs and produces an executable; only `src/engine/occt/*` include OCCT headers.
- [x] 1.3 CI/build script parity with `cybercad/docs/occt-build.md`; document toolchain (Clang C++20, no modules). — `scripts/verify-ios-compile.sh` + `docs/build.md` (toolchain documented; no C++20 modules).

## 2. kernel-facade
- [x] 2.1 Public C header mirroring `cybercad/.../KernelBridgeAPI.h` (`cc_*`, POD structs, `CCShapeId`). — `include/cybercadkernel/cc_kernel.h` (57 `cc_*`); layout checked by `test_abi`.
- [x] 2.2 Shape registry: integer handles, 0 = invalid, `cc_shape_release`. — `shape_registry.*`; covered by `test_registry` (**host**).
- [x] 2.3 Guard model: wrap every entry point; catch `Standard_Failure`/`std::exception` → 0/nil; per-thread `cc_last_error`. — `guard.*` (std::exception path tested on **host**); `Standard_Failure` translation in `occt_engine.h` compiled for **ios-sim**.
- [x] 2.4 `cc_brep_available()` probe. — returns 0 with the stub (**host**); called by the **ios-sim** link check.
- [x] 2.5 ABI contract test: signatures binary-compatible with the app header. — `test_abi` `static_assert`s sizes/offsets vs the app's `KernelBridgeAPI.h` (**host**, passes).

## 3. engine-adapter
- [x] 3.1 Internal C++20 engine interface grouped by capability (construct/boolean/fillet/tessellate/query/transform/exchange). — `src/engine/IEngine.h`; compiles in both builds.
- [x] 3.2 OCCT adapter implementing the full set used by CyberCad (per `occt-usage`), with `IsDone`/`IsValid` validation. — `src/engine/occt/*` (~3.4k LOC) compiles + archives for **ios-sim**; runtime correctness is task 3.4.
- [x] 3.3 Active-engine selection + registry; allow native + OCCT impls of a capability to coexist. — `engine_registry.cpp` (`active_engine`/`set_active_engine`); stub vs OCCT selected at build; compiles in both builds.
- [ ] 3.4 Parity tests: each `cc_*` vs the current in-app bridge behaviour. — DEFERRED: OCCT-backed runtime parity needs the iOS simulator/app; not runnable in the host build (stub returns 0/nil).

## 4. operation-scheduler
- [x] 4.1 `Task<T>` on C++20 coroutines; worker pool (`jthread`) off the UI thread. — coroutine `Task<T>` + `std::thread` pool (`scheduler.*`); `std::jthread` unavailable under Apple Clang libc++, so a plain pool is used. Covered by `test_scheduler` (**host**).
- [x] 4.2 Cancellation via `std::stop_token`; cancelled → cancelled status, resources reclaimed. — in-house `StopToken`/`StopSource` (Apple Clang gates `<stop_token>`); cancel path in `test_scheduler` (**host**).
- [x] 4.3 Progress sink (0..1 / staged) surfaced to the host. — `OperationContext::report`/`checkpoint`; tested (**host**).
- [x] 4.4 Cancellation-safe boundary for non-interruptible engine calls (discard result on cancel). — body runs to completion on a worker, result discarded on cancel; tested (**host**).
- [x] 4.5 Synchronous shims so existing synchronous `cc_*` callers keep working. — facade delegates synchronously; all 57 `cc_*` link (**host**), no async required at the ABI.

## 5. compute-backend
- [x] 5.1 Compute-backend interface for data-parallel tasks (surface eval, BVH, picking, mesh post-proc). — `compute_backend.h`; `test_compute_backend` (**host**).
- [x] 5.2 Default CPU/multi-core backend; correctness independent of backend. — `CpuBackend`; tested (**host**).
- [x] 5.3 Registration hook for platform GPU backends (Metal/CUDA/OpenCL/Vulkan) — interface only, no impl this phase. — registry `register`/active-selector; no GPU impl (by design).
- [x] 5.4 Precision guard: never dispatch fp64 exact-modeling work to an fp32-only backend. — `supports_fp64()` + precision guard; tested (**host**).

## 6. Integration & validation
- [ ] 6.1 Point CyberCad at the library (link swap) behind its OCCT flag; PreviewKernel fallback intact. — DEFERRED: app link-swap needs the CyberCad app project; not done here.
- [ ] 6.2 End-to-end parity: app scenarios produce equivalent results via the library. — DEFERRED: needs the app + simulator runtime (depends on 3.4 and 6.1).
- [x] 6.3 `Result<T,Error>` (C++20; not `std::expected`) internal type; C facade collapses to 0/nil + `cc_last_error`. — `core/result.h`; facade collapses via `guard()`; verified on **host**.
- [ ] 6.4 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 0 to ✅. — `openspec validate --all --strict` is GREEN (2 passed), but Phase 0 is NOT flipped to ✅: it stays ◐ (in progress) because 3.4/6.1/6.2 are deferred.

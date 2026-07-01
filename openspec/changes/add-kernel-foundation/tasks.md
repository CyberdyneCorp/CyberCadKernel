# Tasks — add-kernel-foundation

## 1. Library skeleton & build
- [ ] 1.1 CMake C++20 static library `CyberCadKernel`; targets arm64 iOS device + simulator.
- [ ] 1.2 Link the trimmed OCCT static libs (as the app builds today); OCCT headers private to adapter TUs only.
- [ ] 1.3 CI/build script parity with `cybercad/docs/occt-build.md`; document toolchain (Clang C++20, no modules).

## 2. kernel-facade
- [ ] 2.1 Public C header mirroring `cybercad/.../KernelBridgeAPI.h` (`cc_*`, POD structs, `CCShapeId`).
- [ ] 2.2 Shape registry: integer handles, 0 = invalid, `cc_shape_release`.
- [ ] 2.3 Guard model: wrap every entry point; catch `Standard_Failure`/`std::exception` → 0/nil; per-thread `cc_last_error`.
- [ ] 2.4 `cc_brep_available()` probe.
- [ ] 2.5 ABI contract test: signatures binary-compatible with the app header.

## 3. engine-adapter
- [ ] 3.1 Internal C++20 engine interface grouped by capability (construct/boolean/fillet/tessellate/query/transform/exchange).
- [ ] 3.2 OCCT adapter implementing the full set used by CyberCad (per `occt-usage`), with `IsDone`/`IsValid` validation.
- [ ] 3.3 Active-engine selection + registry; allow native + OCCT impls of a capability to coexist.
- [ ] 3.4 Parity tests: each `cc_*` vs the current in-app bridge behaviour.

## 4. operation-scheduler
- [ ] 4.1 `Task<T>` on C++20 coroutines; worker pool (`jthread`) off the UI thread.
- [ ] 4.2 Cancellation via `std::stop_token`; cancelled → cancelled status, resources reclaimed.
- [ ] 4.3 Progress sink (0..1 / staged) surfaced to the host.
- [ ] 4.4 Cancellation-safe boundary for non-interruptible engine calls (discard result on cancel).
- [ ] 4.5 Synchronous shims so existing synchronous `cc_*` callers keep working.

## 5. compute-backend
- [ ] 5.1 Compute-backend interface for data-parallel tasks (surface eval, BVH, picking, mesh post-proc).
- [ ] 5.2 Default CPU/multi-core backend; correctness independent of backend.
- [ ] 5.3 Registration hook for platform GPU backends (Metal/CUDA/OpenCL/Vulkan) — interface only, no impl this phase.
- [ ] 5.4 Precision guard: never dispatch fp64 exact-modeling work to an fp32-only backend.

## 6. Integration & validation
- [ ] 6.1 Point CyberCad at the library (link swap) behind its OCCT flag; PreviewKernel fallback intact.
- [ ] 6.2 End-to-end parity: app scenarios produce equivalent results via the library.
- [ ] 6.3 `Result<T,Error>` (C++20; not `std::expected`) internal type; C facade collapses to 0/nil + `cc_last_error`.
- [ ] 6.4 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 0 to ✅.

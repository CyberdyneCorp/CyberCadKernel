# Tasks — accelerate-multicore-occt

Verification levels: **host** = `test_parallel_policy` + no-OCCT host CTest (6/6);
**ios-sim** = OCCT adapter (incl. the parallel paths) compiles/archives/link-checks
for `arm64-apple-ios-simulator` (`scripts/verify-ios-compile.sh`). Implementation
items below are compile-verified (ios-sim) and, for the policy layer, host-tested;
runtime "confirm/audit/benchmark/parity" items need a running simulator/device and
are left `[ ]`.

## 1. Parallelism policy & infrastructure
- [x] 1.1 Thread-pool policy over `OSD_ThreadPool` (`SetNbDefaultThreadsToLaunch`); host-settable worker cap for mobile. — `ParallelPolicy::setWorkerCap/resolvedWorkerCount` (`parallel_policy.h`), applied by `applyOcctWorkerCap()` (`parallel_policy.cpp`).
- [x] 1.2 Global `parallel` toggle (default on, opt-out) surfaced to the adapter; per-op override. — `ParallelPolicy::setEnabled/parallelFor(ParallelPref)`.
- [x] 1.3 Verify OCCT build exposes `OSD_Parallel` (and TBB backend if built with it). — `parallel_policy.cpp` compiles+links against `<OSD_Parallel.hxx>`/`<OSD_ThreadPool.hxx>` (`scripts/verify-ios-compile.sh`).

## 2. Parallel booleans (cc_boolean)
- [x] 2.1 Set `BOPAlgo_Options::SetRunParallel(true)` on fuse/cut/common in the OCCT adapter. — `runBoolean()` in `occt_booltransform.cpp`, driven by `ParallelPolicy::parallelFor()`.
- [x] 2.2 Expose + tune `SetFuzzyValue` for robustness on fine geometry; document chosen default. — tuned `booleanFuzz()` + one-shot per-op override (`setNextBooleanFuzzy`/`takeBooleanFuzz`).
- [x] 2.3 Keep `IsValid()` gate; parallel result must pass the same validity check as serial. — `BRepCheck_Analyzer(...).IsValid()` + fuse volume gate unchanged in `runBoolean()`.

## 3. Parallel meshing (cc_tessellate / cc_face_meshes)
- [x] 3.1 Enable `BRepMesh_IncrementalMesh` `isInParallel` (ctor arg / `IMeshTools_Parameters::InParallel`).
- [x] 3.2 Confirm per-face triangulation output is identical to the serial path (topology unchanged). — PARTIAL (**ios-run**): `scripts/run-sim-harness.sh` confirms parallel meshing is byte-identical across 16 runs (FNV-1a) for the tested bodies; a direct serial-vs-parallel diff needs an additive `cc_*` toggle (see 5.x).

## 4. Cancellable long ops via the scheduler
- [x] 4.1 Route boolean + meshing through the Phase-0 `operation-scheduler` (`Task<T>`, off UI thread). — `occt::runScheduled()` (`parallel_policy.h`) wraps `boolean_op` / `tessellate` / `face_meshes`.
- [x] 4.2 Cancellation-safe boundary for the non-interruptible OCCT `Build` (discard result on cancel, reclaim resources). — `runScheduled` maps `OperationStatus::Cancelled` → `OperationCancelled`; `run_operation` discards the computed result.
- [x] 4.3 Progress reporting (staged) for long booleans/meshing. — `ctx.report()` stages in `runBoolean` and the mesh bodies.
- [x] 4.4 Fine-thread boolean gate: refuse / keep-as-separate-bodies for high-turn fine-pitch threads until fast; expose the gate decision to the host. — `ParallelPolicy::evaluateGate` + `occt::checkFineThreadGate`; thread bodies tagged by `occt::tagAsThread`; decision surfaced via `cc_last_error` + `ParallelPolicy::lastGateDecision()`.

## 5. Determinism audit
- [ ] 5.1 Regression suite: parallel vs serial results bit-reproducible across representative bodies (booleans + meshes). — PARTIAL (**ios-run**): `tests/sim/parity_bench.cpp` proves parallel run-to-run reproducibility (box fuse+mesh byte-identical ×16). Still needed: an additive `cc_*` parallel toggle for a true serial baseline, plus more representative bodies.
- [ ] 5.2 Only enable parallel-by-default for paths that pass the audit; document any path kept serial. — parallel is default-on today; gate the default on the completed 5.1 audit.

## 6. Validation
- [ ] 6.1 Benchmark fine-thread fuse/cut: serial vs parallel wall-clock + core scaling on device. — PARTIAL (**ios-run**): absolute sim benchmark captured (boolean ≈5.9 ms/op, fine mesh ≈1.1 ms). Serial-vs-parallel A/B + on-device core scaling still pending.
- [ ] 6.2 Parity: `cc_boolean` / `cc_tessellate` results unchanged vs Phase-0 (serial) behaviour. — sim harness confirms correct boolean/mesh output; a direct vs-serial diff needs the toggle (5.1).
- [ ] 6.3 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 1 + change index. — `openspec validate` GREEN (2 passed); Phase 1 stays ◐ pending the full determinism audit + on-device benchmark.

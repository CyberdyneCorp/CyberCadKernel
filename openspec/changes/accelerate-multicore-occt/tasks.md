# Tasks — accelerate-multicore-occt

Verification levels: **host** = `test_parallel_policy` + no-OCCT host CTest (6/6);
**ios-sim** = OCCT adapter (incl. the parallel paths) compiles/archives/link-checks
for `arm64-apple-ios-simulator` (`scripts/verify-ios-compile.sh`); **ios-run** =
the adapter runs in a booted simulator via the full suite (`scripts/run-sim-suite.sh`,
221 passed / 0 failed), which now includes the serial-vs-parallel determinism A/B +
benchmark driven by the additive `cc_set_parallel(int)` toggle. The determinism
audit (3.2, 5.1, 5.2) and serial-vs-parallel parity/benchmark (6.1 sim A/B, 6.2) are
now **ios-run** verified; the only item still left `[ ]` is **on-device core
scaling** (6.1), which needs physical Apple hardware.

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
- [x] 3.2 Confirm per-face triangulation output is identical to the serial path (topology unchanged). — DONE (**ios-run**): the additive `cc_set_parallel(int)` toggle drives a true serial-vs-parallel A/B in `tests/sim/checks_accel.cpp`. For box-box fuse, box-box cut, revolve tube, and a multi-face fillet solid the parallel mesh is **bit-identical to the serial mesh** (same FNV-1a hash, same triangle count, same exact volume) — `serial == parallel: YES` for all four, and parallel is stable across 8 runs. Run via `scripts/run-sim-suite.sh`.

## 4. Cancellable long ops via the scheduler
- [x] 4.1 Route boolean + meshing through the Phase-0 `operation-scheduler` (`Task<T>`, off UI thread). — `occt::runScheduled()` (`parallel_policy.h`) wraps `boolean_op` / `tessellate` / `face_meshes`.
- [x] 4.2 Cancellation-safe boundary for the non-interruptible OCCT `Build` (discard result on cancel, reclaim resources). — `runScheduled` maps `OperationStatus::Cancelled` → `OperationCancelled`; `run_operation` discards the computed result.
- [x] 4.3 Progress reporting (staged) for long booleans/meshing. — `ctx.report()` stages in `runBoolean` and the mesh bodies.
- [x] 4.4 Fine-thread boolean gate: refuse / keep-as-separate-bodies for high-turn fine-pitch threads until fast; expose the gate decision to the host. — `ParallelPolicy::evaluateGate` + `occt::checkFineThreadGate`; thread bodies tagged by `occt::tagAsThread`; decision surfaced via `cc_last_error` + `ParallelPolicy::lastGateDecision()`.

## 5. Determinism audit
- [x] 5.1 Regression suite: parallel vs serial results bit-reproducible across representative bodies (booleans + meshes). — DONE (**ios-run**): the additive `cc_set_parallel(int)` toggle gives a real serial baseline; `checks_accel.cpp` runs the A/B over four representative bodies (box-box fuse, box-box cut, revolve tube, multi-face fillet solid). **All four report `serial == parallel: YES`** (bit-identical mesh hash + exact volume + tri count) and are stable across 8 parallel runs:
      - box-box fuse  vol=1875.0000 tris=36
      - box-box cut   vol=875.0000  tris=24
      - revolve tube  vol=659.7345  tris=328
      - fillet solid  vol=946.5774  tris=964
- [x] 5.2 Only enable parallel-by-default for paths that pass the audit; document any path kept serial. — DONE: booleans (fuse/cut) and meshing both PASS the 5.1 audit (serial==parallel bit-identical), so parallel-by-default is justified for exactly those paths. No path had to be kept serial. The fine-thread boolean gate (4.4) remains the one intentional throttle.

## 6. Validation
- [ ] 6.1 Benchmark fine-thread fuse/cut: serial vs parallel wall-clock + core scaling on device. — SIM A/B DONE, on-device pending (**ios-run**): `checks_accel.cpp` times serial vs parallel (N=20) per body:
      - box-box fuse  serial=6.608 ms  parallel=7.558 ms  speedup=0.87x  (36 tris)
      - box-box cut   serial=5.560 ms  parallel=6.329 ms  speedup=0.88x  (24 tris)
      - revolve tube  serial=1.870 ms  parallel=1.242 ms  speedup=1.51x  (328 tris)
      - fillet solid  serial=16.574 ms parallel=16.337 ms speedup=1.01x  (964 tris)
      Parallelism helps once tri-count is large enough to amortize thread setup (revolve 1.51x); small booleans are dominated by pool overhead on the simulator. **Still needs a physical device** for true core-scaling (thread-count sweep on Apple hardware); left `[ ]` for that reason.
- [x] 6.2 Parity: `cc_boolean` / `cc_tessellate` results unchanged vs Phase-0 (serial) behaviour. — DONE (**ios-run**): `cc_set_parallel(0)` reproduces the exact Phase-0 serial path; the 5.1 A/B proves `cc_boolean` (fuse/cut) and `cc_tessellate` produce **bit-identical** results under parallel vs serial across all four bodies.
- [ ] 6.3 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 1 + change index. — `openspec validate --all --strict` GREEN (2 passed). The determinism audit (5.1/5.2) + serial-vs-parallel sim benchmark are now complete and reflected in `ROADMAP.md`; Phase 1 stays ◐ only for **on-device core scaling** (6.1). Kept `[ ]` until Phase 1 is fully closed on hardware.

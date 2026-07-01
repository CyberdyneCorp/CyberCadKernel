## Why

Phase 0 (`add-kernel-foundation`) re-homes the OCCT bridge behind the `cc_*`
facade with no behaviour change — so today the kernel still runs OCCT's booleans
and meshing **single-threaded**, exactly as the app does now. OCCT already ships
mature parallel paths for both, chosen to default off for determinism, not
because threading is unsafe (`OCCT/openspec/acceleration.md`). The app's #1
performance wall — a fine multi-turn thread fused/cut into a shaft pegging OCCT
for **minutes** with a non-cancellable `Build` — is squarely a multi-core-CPU
problem, not a GPU one (`cybercad/.../research-modern-kernel.md` TL;DR §5).

This change is the highest-leverage, lowest-risk acceleration: turn on the
existing parallel paths behind the facade, tune the boolean fuzzy tolerance, and
route the long ops through the Phase-0 scheduler so they are cancellable — with
no new geometry code and no ABI change.

## What Changes

- Enable **parallel booleans** in the OCCT adapter: `BOPAlgo_Options::SetRunParallel(true)`
  on the fuse/cut/common path behind `cc_boolean`, with a tuned `SetFuzzyValue`
  for robustness on fine geometry.
- Enable **parallel meshing**: the `isInParallel` path of
  `BRepMesh_IncrementalMesh` behind `cc_tessellate` / `cc_face_meshes`.
- Route long booleans/meshing through the Phase-0 **operation-scheduler** so they
  run off the UI thread and honour cooperative cancellation; **gate the
  fine-thread boolean** (refuse / keep as separate bodies) until it completes
  fast rather than hanging the app.
- Add a **thread-count / parallelism policy** (driven by `OSD_ThreadPool`) so the
  host can bound worker threads on mobile.
- **Determinism audit**: parallel results must be bit-reproducible against the
  serial path before parallel becomes the default; parallel is opt-out.

No `cc_*` signature changes and no new geometry: same results, computed on more
cores, now cancellable.

## Capabilities

### New Capabilities
- `parallel-acceleration`: multi-core execution of the OCCT-backed boolean and
  meshing paths behind the facade, with deterministic results, a bounded worker
  pool, and cancellable long operations. Depends on `engine-adapter` (the OCCT
  adapter) and `operation-scheduler` (cancellation/progress) from Phase 0.

### Modified Capabilities
<!-- none — parallel-acceleration is additive; it consumes engine-adapter and
     operation-scheduler without changing their contracts. -->

## Impact

- **Contract**: satisfies `cybercad/openspec/specs/occt-usage/spec.md`
  §Performance & acceleration targets (parallel pave-filler + tuned FuzzyValue,
  GitHub #286) and §Meshing; behaviour of §Boolean operations and §Meshing is
  preserved (same results, parallelised).
- **App**: no code change — `cc_boolean` / `cc_tessellate` gain speed and
  cancellability transparently. The fine-thread gate replaces today's app-side
  workaround once the boolean is fast enough to un-gate.
- **Build**: no new dependency; uses OCCT's in-tree `OSD_Parallel` /
  `OSD_ThreadPool`. Optional TBB backend only if OCCT was built with it.
- **Risk**: parallelism must preserve determinism — the audit gates the default.

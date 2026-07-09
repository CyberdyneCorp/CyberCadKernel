# Tasks — moat-m5h-collinear-run

## 1. Fixpoint refactor (src/native/heal/collinear_vert.h)

- [x] 1.1 Extract the existing disjoint left-to-right sweep into
  `detail::removeLoopVertsPass(loop, tol, maxDeviation, out) -> int removedThisPass`.
- [x] 1.2 Rewrite `detail::removeLoopVerts` to iterate the pass to a FIXPOINT on the
  previous pass's survivors: accumulate `nRemoved`, `swap` in the survivors, stop when a
  pass removes nothing or the loop reaches a triangle (`< 4` corners).
- [x] 1.3 Update the header block + the `removeLoopVerts` and `removeCollinearVertices`
  comments to document the fixpoint (run collapse, termination, never-over-remove).
- [x] 1.4 No `HealOptions` / `HealMetrics` / `heal.cpp` change (internal to the helper).

## 2. Gate 1 — HOST analytic (tests/native/test_native_heal.cpp)

- [x] 2.1 Add `quadFaceCollinearRunFirstSide` + `cubeTopCollinearRun(ts)` fixtures
  (a straight side over-split by `|ts|` collinear vertices).
- [x] 2.2 `heal_collinear_run_two_verts_heals`: run of 2 → `V=1`, `nRemovedCollinearVerts==2`.
- [x] 2.3 `heal_collinear_run_three_verts_heals`: run of 3 → `V=1`, `nRemovedCollinearVerts==3`.
- [x] 2.4 `heal_collinear_run_default_off_declines`: flag OFF → `GapBeyondTolerance`, unchanged.
- [x] 2.5 `heal_collinear_run_remove_loop_layer`: unit-drive the fixpoint; two-vertex run →
  square; mixed run keeps the off-line real corner.
- [x] 2.6 Build + run: `36 cases, 0 failed`.

## 3. Gate 2 — SIM parity vs OCCT (tests/sim/native_heal_parity.mm)

- [x] 3.1 Add `cubeTopCollinearRunSoup(ts)` + native/OCCT run-soup builders.
- [x] 3.2 `collinear-run fixpoint matches OCCT sew+fix`: native `removed==2`, `V≈1`, matches OCCT.
- [x] 3.3 `collinear-run default-off: native more-conservative than OCCT`.
- [x] 3.4 Build + run in booted sim: `16 passed, 0 failed`.

## 4. Discipline + docs

- [x] 4.1 `src/native/**` OCCT-free; `git diff include/` empty (ABI unchanged).
- [x] 4.2 Update `openspec/MOAT-ROADMAP.md` M5 status (fixpoint run collapse landed).
- [x] 4.3 `openspec validate moat-m5h-collinear-run --strict`.
- [x] 4.4 Commit to `moat-seq` with both gate numbers.

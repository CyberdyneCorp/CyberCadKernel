# Tasks — moat-clashfix-edge-edge

## 1. Reproduce

- [x] 1.1 Build a host driver for the coplanar plus-sign-cross box pair (A
  `[0,3]×[1,2]×[0,1]`, B `[1,2]×[0,3]×[1,2]`, coplanar at `z=1`, no contained vertex)
  and confirm `cc_interference` / `meshInterference` returns CLEAR with
  `minDistance ≈ 1.0` (the bug).

## 2. Fix

- [x] 2.1 Add `idetail::segmentSegmentDistance` — closed-form clamped segment–segment
  distance (Ericson RTCD §5.1.9), handling the parallel and degenerate zero-length
  cases, OCCT-free pure math.
- [x] 2.2 Add `idetail::triEdgeEdgeDistance` — the nine edge–edge sub-tests over a
  triangle pair.
- [x] 2.3 In step 4, fold the edge–edge term into the running per-pair distance so the
  tri–tri minimum is `min(6 vertex–face, 9 edge–edge)`. Keep the AABB-gap prune and
  the existing vertex–face tests.
- [x] 2.4 Update the header docstring (step 4 + the file header) to note the tri–tri
  distance now includes the edge–edge term; remove the stale "tight bound otherwise"
  language.

## 3. Verify (two-gate)

- [x] 3.1 HOST gate: add regression tests to
  `tests/native/test_native_interference.cpp` — coplanar-cross → TOUCHING (was CLEAR);
  a penetrating cross → CLASH with witness; a gapped cross → CLEAR at the exact gap;
  the existing seated / coincident / contained / slid contacts stay TOUCHING. Full
  host `ctest` green (no regression).
- [x] 3.2 SIM gate: extend `tests/sim/native_interference_parity.mm` with the
  coplanar-cross pose (and its gapped variant) and assert the native state matches the
  OCCT `BRepExtrema_DistShapeShape` oracle; no existing parity case regresses.
- [x] 3.3 Confirm no tolerance was widened and no existing clash case regressed (the
  equal-or-more-conservative contract holds).

## 4. Finalize

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` (tri–tri distance now includes edge–edge;
  GS7 clash op correct for arbitrary flush contact) and the interference row in
  `openspec/DROP-OCCT-READINESS.md` if relevant.
- [x] 4.2 `openspec validate moat-clashfix-edge-edge --strict`.
- [x] 4.3 Structural check: `git diff src/native` OCCT-free and additive (only
  `interference.h`), `cc_*` unchanged, other modules untouched. Commit to
  `moat-clashfix`.

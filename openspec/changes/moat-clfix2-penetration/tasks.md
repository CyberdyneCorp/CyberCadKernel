# Tasks — moat-clfix2-penetration

## 1. Reproduce

- [x] 1.1 Build a host driver for the pass-through pose (slab `[0,10]×[0,10]×[0,1]`,
  bar `[4,6]×[4,6]×[-5,20]`, no contained vertex/centroid) and confirm
  `meshInterference` returns TOUCHING (the bug) while the true overlap is
  `[4,6]×[4,6]×[0,1]`, volume 4.

## 2. Fix

- [x] 2.1 Add `idetail::segmentPiercesTriangleInterior` — a transversal
  segment–triangle interior-crossing test over the landed `boolean::mollerTrumbore`
  kernel, with strict-interior gates on the segment parameter and the barycentric hit
  (seam-safe). OCCT-free pure math.
- [x] 2.2 Add step 2c to `meshInterference` — scan each edge of one solid against each
  face of the other; the first interior pierce forces CLASH (`anyCross`) with a witness
  box + seed point. Evaluate only when the enclosure signature (2a/2b) found no
  contained point; try both operand directions.
- [x] 2.3 Update the header docstring (the file header state machine + the step-2
  algorithm note) to document the pass-through signature alongside the enclosure one.

## 3. Verify (two-gate)

- [x] 3.1 HOST gate: add regression tests to
  `tests/native/test_native_interference.cpp` — bar-through-slab → CLASH with a witness
  inside the overlap box (was TOUCHING); order-symmetry; a touching variant (bar bottom
  flush with slab top) → TOUCHING; a gapped variant → CLEAR at the exact gap; the
  facade path asserts the exact overlap volume 4 (native COMMON). Full host `ctest`
  green (no regression, incl. the edge-edge-fix regressions).
- [x] 3.2 SIM gate: extend `tests/sim/native_interference_parity.mm` with the
  pass-through pose (and its touching/gapped variants) and assert the native state
  matches the OCCT `BRepAlgoAPI_Common` (positive volume) + `BRepExtrema_DistShapeShape`
  oracle; no existing parity case regresses.
- [x] 3.3 Confirm no tolerance was widened and no existing CLASH / TOUCHING / CLEAR
  case regressed (the equal-or-more-conservative contract holds).

## 4. Finalize

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` (interference step 2 now detects
  pass-through penetration; the noted CLASH-signature gap resolved).
- [x] 4.2 `openspec validate moat-clfix2-penetration --strict`.
- [x] 4.3 Structural check: `git diff src/native` OCCT-free and additive (only
  `interference.h`), `cc_*` unchanged, other modules untouched. Commit to
  `moat-clfix2`.

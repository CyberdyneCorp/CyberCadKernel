# Tasks — trimmed-nurbs-healing-depth

## Implementation

- [x] Add `MultiSplitReport` + `splitAtPinches()` / `splitTrimLoopAtPinches()` to
      `trimmed_nurbs.h` (declarations + contract docs).
- [x] Implement the N-way / crossing pinch resolver in `trimmed_nurbs.cpp`:
  - [x] `firstPinchCluster()` — cluster coincident vertices at one location.
  - [x] `resolveCluster()` — leftmost-turn (largest signed CCW turn) pairing + re-route + trace.
  - [x] `splitAtPinches()` — fixpoint over clusters with SIGNED-area preservation + simple-loop
        validity + non-progress guards (honest `ambiguous` decline).
- [x] Add `ClassifyOptions::splitNWay` (default OFF) and route `classifyOuter()` to the EVEN-ODD
      (parity/XOR) union of the resolved sub-loops when the 2-way split declines.
- [x] Add `ConstructOptions::rational` (default ON) + `PcurveConstruction::rational`.
- [x] Implement the RATIONAL pcurve path in `constructPcurve()`: `edgeIsRational()`, project the
      edge control net to (u,v), reuse the edge's degree/knots/weights, verify via round-trip
      fidelity, and honestly fall back to the non-rational fit when the affine assumption fails.

## Tests (regression, airtight oracles)

- [x] 3-way split → exactly 3 simple loops, total signed area preserved ≤1e-12.
- [x] 4-way split → exactly 4 simple loops (generalization).
- [x] Crossing pinches → fixpoint (`iterations>1`), signed area preserved ≤1e-12.
- [x] Even-odd containment preservation vs the reference union (no probe flips).
- [x] Idempotence: a clean loop through `splitAtPinches()` is a no-op (area unchanged).
- [x] Honest decline: default (`splitNWay` OFF) still declines the 3-way pinch `Unknown`.
- [x] Rational pcurve exactness on a plane circle (≤1e-9) vs polynomial SAG (≫1e-4).
- [x] Cylinder circular arc → honest non-rational fall-back with reported true deviation.

## Verification

- [x] `src/native` stays OCCT-free (0 OCCT/BRep/Geom/TK refs in changed files).
- [x] `cc_*` ABI byte-unchanged (additive only); healing stays opt-in default-OFF.
- [x] `test_native_trimmed_nurbs` passes (189 checks); full `ctest` green.
- [x] `openspec validate --all` passes.

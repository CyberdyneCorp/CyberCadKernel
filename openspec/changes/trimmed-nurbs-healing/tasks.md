# Tasks — trimmed-nurbs-healing

## 1. Healing primitive (extend trimmed_nurbs, no shape.h change)
- [x] 1.1 Header: `HealOp`, `HealOptions`, `HealReport`, `healLoop()`, `healTrimLoop()` + file-header
      contract (three ops + region-preservation proof + residual note).
- [x] 1.2 `flattenLoopForHeal` — flatten a loop keeping raw inter-segment gaps + record `joinGaps`
      (segment-join distances incl. the closing join).
- [x] 1.3 `healLoop` — pass 0 large-gap triage (decline), pass 1 weld small gaps / near-coincident
      pairs to midpoint (incl. closing edge), pass 2 pinch detection (decline). Scale-relative tol.

## 2. classify() integration
- [x] 2.1 `ClassifyOptions::heal` (default on) + `healGapTol`; `preparedLoop()` helper heals each
      loop before the raycast; a heal failure declines `Unknown` (no regression to former declines).

## 1b. Pinch-splitting primitive (extend trimmed_nurbs, additive)
- [x] 1b.1 Header: `SplitReport`, `splitAtPinch()`, `splitTrimLoopAtPinch()`; file-header +
      classify contract updated (region-preservation proof; 3+-way/crossing residual narrows).
- [x] 1b.2 `splitAtPinch` — find non-adjacent coincident pairs; a single pair (clean 2-way pinch)
      splits into two sub-loops sharing the snapped pinch vertex; 3+-way / invalid-lobe → ambiguous.
- [x] 1b.3 `classify()` OPT-IN `ClassifyOptions::splitPinch` (default OFF); `classifyOuter()` splits a
      pinched outer loop and classifies the union of the two sub-loops (region-preserving).

## 3. HOST-analytic gate (extend test_native_trimmed_nurbs.cpp)
- [x] 3.1 Small INJECTED gap HEALS to closed and classifies IDENTICALLY to the exact loop.
- [x] 3.2 LARGE gap (beyond tol) still DECLINES honestly (`largeGap`, `classify → Unknown`).
- [x] 3.3 PRESERVATION — a sweep of gap sizes up to tol never flips any interior/exterior probe's
      In/Out verdict vs the exact loop (a heal never changes the region).
- [x] 3.4 Near-coincident pcurves SNAP (welded seam OnBoundary); PINCH detected (`pinch`, Unknown).
- [x] 3.5 Healing toggle (`heal=false`) path runs deterministically.
- [x] 3.6 PINCH-SPLIT: a figure-eight SPLITS into two loops classifying IDENTICALLY to the two-loop
      reference (containment-preservation, no probe flips); an un-splittable 3-way pinch declines
      (`ambiguous`, Unknown); a clean loop is unaffected by `splitPinch`; total region preserved.

## 4. No-regression + docs
- [x] 4.1 Full host ctest green (esp. `test_native_step_reader`, `test_native_topology`); `cc_*`
      unchanged; `src/native` OCCT-free; `math`/`ssi`/`blend`/`boolean` untouched (only `#include`d).
- [x] 4.2 Update `docs/NURBS-SCOPE.md` §4 Layer-8 row (model+pcurve+healing partial; general
      non-manifold repair + rational residual).
- [x] 4.3 `openspec validate --all --strict`.

# Tasks — nurbs-nsided-fill-rational

## 1. Homogeneous rational fill (additive to bspline_nsided)

- [x] 1.1 Add `verifyNSidedBoundaryRational(b, tol)` — accepts rational edges (one strictly-positive
      weight per pole), declines non-positive/mismatched weights, non-closed loops (rational corner
      eval), N<3, and malformed edges.
- [x] 1.2 Add rational-aware split/reverse helpers (`firstHalfR`, `secondHalfReversedR`, `reverseCurveR`)
      that carry weights through `splitCurve` / `reparamCurve` (Layer-1 exact, rational-preserving).
- [x] 1.3 Add a homogeneous Coons boolean sum (`coonsPatchRational`): lift the ruled/bilinear summands to
      R⁴, unify to a common basis with the exact Layer-1 surface ops, sum `L_u ⊕ L_v ⊖ B` in R⁴, project
      once (non-positive projected weight is a hard guard).
- [x] 1.4 Build the interior spokes with MATCHED corner weights (the arc midpoint weight → 1 at the
      centroid) so the four homogeneous corners are consistent.
- [x] 1.5 Add `NSidedFillRationalResult` + `fillNSidedRational(b, tol)`; drop the weight vector when every
      projected weight is 1 (non-rational reduction).
- [x] 1.6 Keep the existing `verifyNSidedBoundary` / `fillNSided` byte-unchanged (additive only).

## 2. Tests + wiring

- [x] 2.1 Add `tests/native/test_native_nurbs_nsided_rational.cpp` — rational-arc exactness,
      non-rational reduction, planar rational containment, honest declines.
- [x] 2.2 Wire the test into CMake (`CYBERCAD_TESTS` + `_SRC` map + `CYBERCAD_HAS_NUMSCI` compile def),
      mirroring `test_native_nurbs_nsided`.
- [x] 2.3 Confirm OCCT-free (0 OCCT refs in changed `src/native` files), full ctest green, openspec
      validates.

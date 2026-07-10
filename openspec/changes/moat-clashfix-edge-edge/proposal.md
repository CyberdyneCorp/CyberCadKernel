# Proposal — moat-clashfix-edge-edge (MOAT M-GS, GS7 correctness fix)

## Why

The M6-breadth-17 clash fuzzer (moat-m6q) surfaced a genuine correctness gap in the
native clash classifier `cc_interference` (`src/native/analysis/interference.h`). Its
step-4 minimum triangle–triangle distance is computed from ONLY the six vertex-vs-face
sub-tests and OMITS the edge–edge term (the header documented this as "a tight bound
otherwise").

For two boxes whose faces are EXACTLY COPLANAR and overlap in a plus-sign cross with
NO mutually-contained vertex — a horizontal bar `[0,3]×[1,2]×[0,1]` and a vertical bar
`[1,2]×[0,3]×[1,2]` sharing the plane `z=1` — the true closest approach is EDGE–EDGE
(A's top edges cross B's bottom edges at distance 0). The vertex-face minimum
overshoots to `≈0.53–1.0`, so a real flush TOUCH is mis-reported as CLEAR
(native=CLEAR while OCCT `BRepExtrema_DistShapeShape` and the closed form both say
TOUCHING / distance 0).

This is NOT a case where native is more correct — it is a real approximation gap. The
certified assembly-mate contact envelope (seated / coincident / contained / slid
contacts) already classifies correctly; this fix extends correctness to arbitrary
flush contact.

## What Changes

- Add a closed-form clamped SEGMENT–SEGMENT distance (Ericson RTCD §5.1.9, handling
  the parallel / degenerate zero-length cases) to `interference.h`, and evaluate the
  nine EDGE–EDGE sub-tests per candidate triangle pair. The tri–tri minimum distance
  becomes `min(6 vertex–face tests, 9 edge–edge tests)` — the exact minimum for
  disjoint convex triangles.
- The coplanar plus-sign-cross pose now classifies TOUCHING (distance 0). CLASH
  (positive-volume overlap, via the unchanged B3 penetration signature) and CLEAR
  (positive clearance) are unaffected; the equal-or-more-conservative contract holds.
- Pure math — `src/native/**` stays OCCT-FREE. The `cc_interference` signature and the
  `CCInterference` POD are UNCHANGED (internal correctness fix). Only `interference.h`
  and its tests change.

## Impact

- Affected specs: `native-interference` (the tri–tri distance now includes the
  edge–edge term; the coplanar-cross TOUCH scenario added).
- Affected code: `src/native/analysis/interference.h` (additive edge–edge term),
  `tests/native/test_native_interference.cpp` (regression tests),
  `tests/sim/native_interference_parity.mm` (OCCT parity for the coplanar-cross pose).
- No ABI change, no other module touched.

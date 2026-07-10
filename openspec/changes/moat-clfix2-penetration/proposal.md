# Proposal — moat-clfix2-penetration (MOAT M-GS, GS7 correctness fix)

## Why

The M6-breadth-17 clash fuzzer (moat-m6q) and the edge-edge-fix report noted a second,
separate correctness gap in the native clash classifier `cc_interference`
(`src/native/analysis/interference.h`). Step 2's CLASH detection uses only a
B3-membership signature: a boundary VERTEX or a boundary TRIANGLE CENTROID of one
solid that classifies strictly INSIDE the other (an ENCLOSURE signal).

For a genuine PENETRATION where a bar pokes CLEAN THROUGH a slab (or a slab is pierced
by a thin bar) such that NEITHER solid has a vertex nor a triangle centroid inside the
other — a "pass-through" — the overlap is real (positive volume) but the
enclosure signature MISSES it, so the pose is mis-classified as TOUCHING rather than
CLASH. Concretely, slab `[0,10]×[0,10]×[0,1]` with bar `[4,6]×[4,6]×[-5,20]`: the bar's
ends stick out both faces of the slab (its vertices are OUTSIDE the slab's z-range),
the slab is wider than the bar (its vertices are OUTSIDE the bar), and every triangle
centroid lands outside the other solid — yet the interiors overlap over
`[4,6]×[4,6]×[0,1]`, a positive volume of 4. OCCT `BRepAlgoAPI_Common` reports volume 4
(CLASH); native reported TOUCHING.

This is the CLASH / positive-overlap side of the classifier. The recently-landed
edge-edge fix (moat-clashfix-edge-edge) corrected the tri–tri DISTANCE / TOUCHING side
and explicitly deferred this step-2 penetration gap as out of scope. This change
closes it.

## What Changes

- Add a PASS-THROUGH penetration signature to step 2 of `interference.h`: an EDGE of
  one solid that pierces a FACE of the other TRANSVERSALLY through its interior
  (segment–triangle interior crossing). A single interior pierce means the edge passes
  from outside to inside the other solid's wall, so the interiors overlap → CLASH.
- The signature reuses the landed Möller–Trumbore ray–triangle kernel
  (`boolean::mollerTrumbore`) as a segment test bounded by the edge length, requiring
  the crossing to be STRICTLY interior to BOTH the segment (not at an endpoint) and the
  triangle (not on a triangle edge/vertex). This makes it SEAM-SAFE: a coplanar contact
  (shared face) or an endpoint seat (a shaft flush against a bore wall) never pierces
  an interior, so TOUCHING and CLEAR are UNAFFECTED — the equal-or-more-conservative
  contract holds and no tolerance is widened.
- The pass-through scan runs ONLY when the existing enclosure signature (2a/2b) finds
  no contained point, so the common already-decided cases short-circuit before the
  O(edges·faces) work.
- On the engine side (`native_engine.cpp`, unchanged) a CLASH verdict already forces
  the native boolean COMMON volume with the two-sided self-verify; a positive-volume
  COMMON is thereby reconciled with a CLASH mesh verdict, and an honest decline still
  applies when the COMMON cannot be robustly computed for a pose.
- Pure math — `src/native/**` stays OCCT-FREE. The `cc_interference` signature and the
  `CCInterference` POD are UNCHANGED (internal correctness fix). Only `interference.h`
  and its tests change.

## Impact

- Affected specs: `native-interference` (step 2 now has a pass-through penetration
  signature in addition to the enclosure signature; the bar-through-slab CLASH
  scenario and its touching/gapped variants added).
- Affected code: `src/native/analysis/interference.h` (additive pass-through
  signature + `idetail::segmentPiercesTriangleInterior`),
  `tests/native/test_native_interference.cpp` (regression tests),
  `tests/sim/native_interference_parity.mm` (OCCT parity for the pass-through pose).
- No ABI change, no other module touched.

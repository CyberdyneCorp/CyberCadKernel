# Design — moat-clashfix-edge-edge

## Context

`meshInterference` (`src/native/analysis/interference.h`) decides CLASH / TOUCHING /
CLEAR at the mesh level, OCCT-free. Step 2 (the B3 penetration signature) decides
CLASH; step 4 computes the minimum triangle–triangle distance to split TOUCHING from
CLEAR. Step 4 evaluated ONLY the six vertex-vs-face sub-tests per triangle pair.

For two DISJOINT convex triangles the true minimum distance is attained at either a
vertex–face pair OR an edge–edge pair. The vertex-face-only distance is exact when a
vertex of one triangle projects into the other's face Voronoi region, but it
OVERSHOOTS when the closest approach is edge–edge with no such vertex — precisely the
coplanar plus-sign-cross pose the M6-breadth-17 fuzzer found. There, A's top edges
cross B's bottom edges in the shared plane `z=1`; every vertex of each box is far from
the other's nearest face, so the vertex-face minimum reports `≈1.0` and the flush
TOUCH (true distance 0) is mis-classified CLEAR.

## Decision

Complete the tri–tri distance with the missing EDGE–EDGE term:

1. `segmentSegmentDistance(p1,q1,p2,q2)` — closed-form clamped segment–segment
   distance (Ericson, *Real-Time Collision Detection* §5.1.9). Solves the 2×2 system
   for the closest parameters, clamps `s,t` to `[0,1]` so the closest points stay on
   the segments, and guards the degenerate cases via the squared-length / determinant
   near-zero tests: both-points, one-point, and parallel segments (`denom ≈ 0` ⇒ pin
   `s=0` and clamp `t`). Pure fp64 vector math on `math::Vec3`/`Point3` — no OCCT.
2. `triEdgeEdgeDistance(...)` — the 3×3 = 9 edge-pair minimum over the two triangles'
   edges.
3. Step 4 now folds `triEdgeEdgeDistance` into the running per-pair distance:
   `dd = min(6 vertex–face tests, triEdgeEdgeDistance)`. The AABB-gap prune and the
   `best` running minimum are unchanged, so far-apart bodies still skip the O(n·m)
   inner work.

The result is `min(…)` over a SUPERSET of the previous sub-tests, so the reported
distance can only stay the same or DECREASE. TOUCHING is `best ≤ contactBand`; adding
terms can only move CLEAR→TOUCHING for a genuinely-closer edge–edge contact, never the
reverse. Existing CLEAR cases with a true positive gap keep their (unchanged) gap
distance; existing TOUCHING cases stay TOUCHING. CLASH is decided earlier in step 2
and is untouched.

## Why not touch the CLASH signature

The fuzzer's finding is specifically a TOUCHING-vs-CLEAR distance gap. A separate,
out-of-scope limitation exists in the step-2 penetration signature for a bar poking
CLEAN THROUGH a slab with no contained vertex/centroid; that is not this change and is
explicitly excluded (this change only completes the tri–tri distance and does not
modify step 2). The regression CLASH fixture therefore uses a penetrating cross whose
overlap DOES contain a boundary vertex, exercising the existing (correct) signature.

## Invariants preserved

- `src/native/**` OCCT-free (segment–segment distance is pure math).
- `cc_interference` signature + `CCInterference` POD byte-for-byte unchanged.
- No tolerance widened; the contact band `max(1e-9·scale, 2·deflection)` is unchanged.
- Only `interference.h` (+ its host/sim tests) changes; tessellator / boolean / other
  modules untouched.

# Design — moat-clfix2-penetration

## Context

`meshInterference` (`src/native/analysis/interference.h`) decides CLASH / TOUCHING /
CLEAR at the mesh level, OCCT-free. Step 2 (the penetration signature) decides CLASH;
step 4 computes the minimum triangle–triangle distance to split TOUCHING from CLEAR.

Step 2's CLASH signal was ENCLOSURE only: a boundary VERTEX (2a) or a boundary
TRIANGLE CENTROID (2b) of one solid classified strictly `In` the other by the B3
ray-parity classifier. This is exact for one solid ENCLOSING a point of the other, but
a positive-volume overlap need not enclose any such point.

The counterexample (noted by the M6-breadth-17 fuzzer and the edge-edge-fix report) is
a bar poking CLEAN THROUGH a slab:

    slab B = [0,10]×[0,10]×[0,1]   (wide, thin)
    bar  A = [4,6]×[4,6]×[-5,20]   (thin, long; passes clean through B)

- A's vertices are at z ∈ {−5, 20}, OUTSIDE B's z ∈ [0,1] → no A-vertex In B.
- B's vertices are at the wide corners, OUTSIDE A's x,y ∈ [4,6] → no B-vertex In A.
- A's side-face centroids are at z = 7.5, OUTSIDE B; B's box faces tessellate into two
  triangles whose centroids fall near the wide corners, OUTSIDE A.

So neither enclosure sub-signal fires, yet the interiors overlap over
`[4,6]×[4,6]×[0,1]` (volume 4). Native mis-reported TOUCHING (min tri–tri distance 0,
because A's side faces exactly meet B's top/bottom face rings); OCCT
`BRepAlgoAPI_Common` gives volume 4 → CLASH.

## Decision

Add a second, complementary step-2 signal — a PASS-THROUGH pierce:

1. `idetail::segmentPiercesTriangleInterior(p, q, a, b, c)` — does the segment `[p,q]`
   pierce triangle `(a,b,c)` transversally through its interior? It normalizes the edge
   to a unit direction and calls the landed `boolean::mollerTrumbore` ray–triangle
   kernel, then applies STRICT-interior gates:
   - a coplanar segment makes the ray-plane determinant ≈0 → Möller returns `nullopt`
     (no transversal crossing) — a coplanar overlap is a zero-volume TOUCH, not a
     pass-through;
   - the crossing parameter `t` must be strictly interior to the segment
     (`segEps < t < len − segEps`) — an endpoint touching the face (a flush seat) is
     contact, not a through-crossing;
   - the barycentric hit `(u,v)` must be strictly interior to the triangle
     (`u,v > baryEps`, `u+v < 1 − baryEps`) — a pierce exactly on a triangle edge is a
     seam artifact, not an interior transversal.
   Returns the pierce point (a seed witness) on success, else `nullopt`.
2. Step 2c scans every EDGE of one solid against every FACE of the other; the first
   interior pierce sets `anyCross` (the existing CLASH accumulator) and a witness box +
   seed point. It is evaluated ONLY when 2a/2b found no enclosed point (`!anyInside &&
   !anyCross`), so already-decided common cases short-circuit before this O(edges·faces)
   pass. Both directions are tried (A-edges vs B-faces, then B-edges vs A-faces).

The CLASH decision (`if (anyInside || anyCross)`) and everything downstream — witness
plumbing, the engine's native COMMON volume + two-sided self-verify, the honest decline
when COMMON can't be robustly computed — are unchanged. A positive-volume COMMON is
thereby reconciled with a CLASH mesh verdict.

## Why the pierce test is seam-safe (TOUCHING / CLEAR unaffected)

A pass-through requires an edge to cross the face plane at a point strictly inside both
the edge and the triangle. The three contact families the classifier must NOT promote
to CLASH all fail that requirement:

- **Coplanar / flush face contact** (two boxes sharing a face; the coplanar plus-cross)
  — the crossing edges lie IN the shared plane → Möller's determinant ≈0 → declined.
- **Endpoint seat** (a shaft bottomed against a bore wall; a bar resting on a slab) —
  the edge meets the face AT an endpoint → `t` at the segment boundary → declined.
- **Positive clearance** (the gapped bar) — no crossing at all.

The pierce test only fires when part of an edge is genuinely on the far side of the
other solid's wall — i.e. real interpenetration. It is a SUPERSET-of-signatures
addition: it can only move a pose from TOUCHING/CLEAR to CLASH, and only for a genuine
positive-volume overlap the enclosure signature missed. It never moves a pose the
other way.

## Why not compute the COMMON in interference.h

The header is OCCT-free AND boolean-free (mesh-math only); it cannot call the native
boolean COMMON. The engine already forces the COMMON volume once the mesh verdict is
CLASH and guards it with the two-sided self-verify (an overlap cannot exceed either
operand; a non-watertight/absent COMMON declines to OCCT). So the correct, minimal fix
is to make the MESH verdict recognize the pass-through as CLASH; the engine then
supplies the positive-volume COMMON exactly as it does for the enclosure signature.

## Invariants preserved

- `src/native/**` OCCT-free (the pierce test is pure math over the landed Möller kernel).
- `cc_interference` signature + `CCInterference` POD byte-for-byte unchanged.
- No tolerance widened; the contact band `max(1e-9·scale, 2·deflection)` is unchanged.
  The pierce gates use relative fp64 epsilons (`1e-9·len`, `1e-9` barycentric) that only
  ever REJECT a grazing/endpoint crossing — they never accept a wider contact.
- Only `interference.h` (+ its host/sim tests) changes; tessellator / boolean / other
  modules untouched.

# Design — moat-clash-interference (MOAT M-GS, GS7)

## Context

The clash check answers three questions about two placed solids A and B: *do they
interfere* (CLASH / TOUCHING / CLEAR), *how much* (the overlap volume), and *where*
(a witness). The OCCT oracle answers them with `BRepAlgoAPI_Common` + `BRepGProp`
(volume) and `BRepExtrema_DistShapeShape` (clearance). GS7 reproduces that answer
OCCT-free for native bodies, reusing landed, already-verified parts.

## The clash / touch discriminator (the load-bearing decision)

CLASH must be distinguished from TOUCHING **coplanar-safely** — two boxes sharing a
face must read TOUCHING, not CLASH, even though their boundaries coincide over a
whole face.

**Rejected approach — Möller triangle–triangle crossing.** The obvious signal is "a
triangle of A transversally crosses a triangle of B". The GS6 validity checker
already has a Möller tri–tri predicate with a parallel-plane guard. But across two
*separate* meshes it over-reports at a shared seam: edge-adjacent triangles of two
flush solids (e.g. A's bottom face and B's bottom face, both on z=0, meeting at the
shared edge x=1) are reported as `Cross` because the on-line interval overlap
includes the shared boundary, and there are no shared vertex indices to filter them
(the GS6 `shareVertex` filter works only within one mesh). A midpoint-inside gate
did not save it — the midpoint of two face centroids is naturally pulled into one
solid's interior. Measured: two face-touching unit boxes produced 30 spurious
crossings.

**Chosen approach — B3 point-membership of vertices + triangle centroids.** The
reliable, coplanar-safe signal is: a boundary VERTEX or a boundary TRIANGLE CENTROID
of one solid classifies strictly `In` the other, via the landed B3 multi-ray parity
`classifyPointInMesh`. Its ON-band absorbs a shared seam (a coincident point reads
`On`, never `In`), so a flush TOUCH never fires; a face poking through the other's
wall has an interior centroid → CLASH. This reuses the exact classifier already
verified vs OCCT, adds no new predicate, and is coplanar-safe by construction.

When no penetration signature fires, the minimum triangle–triangle distance (the
landed exact `pointTriangleDistance`, evaluated as the min over the six vertex-face
sub-tests of each AABB-close pair, pruned by the running best) decides TOUCHING
(within the mesh-fidelity contact band `max(1e-9·scale, 2·deflection)`) vs CLEAR (the
min distance is the reported clearance).

## Honest-decline gate (never a wrong clash flag)

`classifyPointInMesh` returns `Unknown` when its rays graze / disagree. A blanket
"any Unknown → decline" is too aggressive: a native-meshed box has face-centroid
points that lie in a shared coordinate plane with the *other* box's faces, so rays
from a point clearly OUTSIDE the target still graze and return `Unknown` (measured:
a B centroid at (1.67, 0.67, 0), 0.67 away from A, declined). That is a spurious
`Unknown` on a far face, not evidence of a hidden overlap.

The gate: an `Unknown` VETOES the verdict (→ UNKNOWN decline) **only when the point
could be masking an interior overlap** — it lies strictly inside the target's AABB
(the only region an interior point can occupy) AND beyond the contact band of the
target boundary. A point outside the target AABB cannot be interior regardless of the
rays; an `Unknown` at the contact seam is an expected touch artifact. This preserves
the honest decline for a genuinely ambiguous enclosable point while not vetoing a
clean touch/clearance. A non-watertight operand mesh is an immediate UNKNOWN (ray
parity is undefined).

## Overlap volume + two-sided self-verify (never a wrong number)

The header stays boolean-free / OCCT-free — it never computes the volume. On CLASH
the ENGINE computes it as `watertightVolume(boolean_solid(A, B, Op::Common))`, the
same native BSP-CSG COMMON + watertight-volume helper `boolean_op` already uses. The
**two-sided self-verify** discards a wrong value:

1. the COMMON solid must be watertight (`watertightVolume >= 0`), and
2. `vc <= min(V(A), V(B)) + tol` — an intersection cannot be larger than either
   operand (an independent set-algebra sanity bound).

A null / non-watertight / out-of-band COMMON (a curved or near-tangent overlap the
native planar boolean cannot build), or a mesh-soup operand with no B-rep to
intersect, DECLINES — the engine returns a clean error and the facade falls through
to the OCCT `BRepAlgoAPI_Common` oracle. A wrong overlap volume is never returned.

The CLASH witness is sharpened from the mesh-classifier's boundary witness to the
COMMON solid's tight AABB + its signed-tetra centroid — a point guaranteed to lie in
the overlap interior, matching the OCCT oracle's witness (COMMON bbox + centre of
mass).

## Two gates

- **GATE A (host, no OCCT):** closed-form fixtures — overlapping boxes (exact
  intersection-box volume + witness), disjoint (exact gap), face-touching (volume 0),
  nested (clash), non-watertight (decline). `test_native_interference.cpp`.
- **GATE B (sim, OCCT oracle):** native `cc_interference` vs `BRepAlgoAPI_Common` +
  `BRepGProp` (volume) and `BRepExtrema_DistShapeShape` (state + clearance) on
  identical geometry at fixed tolerances. `native_interference_parity.mm`.

## Scope / declines (tracked)

- A CLASH whose overlap volume needs a curved / near-tangent COMMON the native
  boolean cannot robustly build → decline → OCCT.
- A freeform-operand overlap → decline (the sharpened next blocker).
- A mesh-soup (imported STL) operand's overlap volume → decline (no B-rep to intersect).

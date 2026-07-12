# Proposal — nurbs-nsided-fill (NURBS roadmap Layer 6)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. The Layer-6 surfacing
slices — **skinning / lofting** (`bspline_skin.{h,cpp}`), **swept surfaces** (`bspline_sweep.{h,cpp}`),
the **Gordon / network surface** (`bspline_gordon.{h,cpp}`), and the boundary-filled **Coons patch**
(`bspline_coons.{h,cpp}`) — are landed. A single bilinearly-blended Coons patch fills exactly FOUR
boundaries (a topological quad). The next Layer-6 capability is the **N-SIDED boundary-filled surface**:
given N ≠ 4 boundary B-spline curves forming a CLOSED loop (a topological N-gon — triangle, pentagon,
hexagon …, with consecutive shared corners), fill it with a smooth multi-patch of tensor-product
B-spline surfaces that together INTERPOLATE all N input boundary curves. This is the core surfacing op
for **filling a non-quad hole / wireframe** and **capping an N-sided opening** — designers routinely
need to cap a triangular or pentagonal boundary that has no natural 4-sided grid.

This slice is worth building **now** because it (a) is small and well-bounded — the classic
Catmull-Clark-style **midpoint subdivision** turns any N-gon into N quads meeting at the polygon
centroid, and each quad is filled with the already-verified Coons patch — (b) is built entirely on
machinery that already exists (the **Layer-1 exact** `splitCurve` / `reparamCurve` to halve and
re-domain the boundary edges, plus `bspline_coons` for the per-quad fill — no new numerical solver),
and (c) has a **uniquely airtight oracle**: the finished multi-patch must contain every input boundary
curve *pointwise* (each boundary edge is the exact outer iso-edge of two sub-patches → containment
residual → 0), a planar N-gon yields N coplanar flat patches, and adjacent sub-patches meet C0 along
the shared interior spokes and the shared centroid. It composes verified lower layers into a new
capability with a machine-precision oracle, and DECLINES honestly on a non-closed / rational /
degenerate boundary.

## What

A new OCCT-free module `src/native/math/bspline_nsided.{h,cpp}` (namespace `cybercad::native::math`,
beside `bspline_coons`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, because it composes the numsci-gated
`bspline_coons`, and to sit uniformly with the rest of the numsci-gated Layer-6 surfacing family) —
though the construction itself uses only the exact Layer-1 ops and **no linear solve**. It reuses the
Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types for its input (the N boundary curves) and
output (the N sub-patches). **Non-rational, N ≥ 3 boundary loop only** (all weights = 1); rational /
Gregory-plate G1/G2 fill is an explicit residual.

**Midpoint subdivision** (Catmull-Clark N-gon → N quads):

1. **Loop consistency** (`verifyNSidedBoundary`) — the N edges only form a valid N-gon when the loop
   closes on consecutive shared corners: `edges[i](1) == edges[(i+1)%N](0)` to within tolerance;
   every edge non-rational, well-formed, and N ≥ 3. `verifyNSidedBoundary` checks this and reports the
   worst corner mismatch; `fillNSided` DECLINES (`ok=false`, with a reason) on a non-closed / rational
   / degenerate boundary — never a surface that silently misses its own boundary.
2. **Subdivide + Coons-fill** (`fillNSided`) — compute the corners `V[i] = edges[i](0)`, the edge
   midpoints `M[i] = edges[i](0.5)` (via `splitCurve` at 0.5), and the centroid `C = mean(V[i])`. For
   each corner `V[i]` build the quad centred on it — outer edges = the first half of `e[i]` and the
   second half of `e[i-1]` reversed (both exact boundary sub-curves via `splitCurve` + `reparamCurve`),
   interior edges = the straight spokes `M[i-1]→C` and `M[i]→C` — and fill it with `coonsPatch`. Each
   boundary edge `e[k]` is covered by exactly two sub-patch outer edges (its first half by sub-patch k,
   its second half by sub-patch k+1), so the UNION reproduces every input boundary curve pointwise;
   adjacent sub-patches meet C0 on the shared spoke `M[i]→C` and all share the centroid `C`.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_nsided.cpp`:
  1. **Boundary containment (the core oracle)** — the N sub-patches together reproduce all N input
     boundary curves POINTWISE along the outer edges to ~1e-9 (achieved ~1e-15), for a planar pentagon,
     a planar triangle, and a mildly-curved (in-plane) hexagon.
  2. **Planar N-gon → flat patches** — a planar pentagon / triangle / curved-hexagon boundary yields N
     coplanar patches; every surface point of every patch lies on the boundary's plane exactly (~1e-12,
     achieved 0).
  3. **C0 interior junctions** — adjacent sub-patches meet C0 along the shared interior spoke `M[i]→C`,
     and all patches pass through the shared centroid `C` (~1e-12, achieved 0).
  4. **N=4 consistency** — for a 4-sided boundary the 4 sub-patches' union reproduces the same four
     boundary curves as the single Coons patch of that boundary (both contain the boundary ~1e-9).
  5. **Honest declines** — a non-closed loop, a rational edge, N<3, and a malformed edge decline
     (`ok=false`, with a reason and — for the broken loop — a large corner error), never a
     silently-wrong surface, never a crash.

## Scope

- Adds `src/native/math/bspline_nsided.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_nsided.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_coons`.
- Only `#include`s `bspline_coons.h` (the per-quad fill), `bspline_ops.h` (Layer 1: `splitCurve` /
  `reparamCurve`), and the evaluators (`bspline.h`) — it does NOT modify them, and does NOT touch
  `bspline_skin` / `bspline_sweep` / `bspline_gordon` / `bspline_fit` (a concurrent rational-Gordon
  track owns gordon/sweep; this change only shares the subdivision *pattern*, not their code).
- Does NOT touch `ssi` / `blend` / `boolean` / `topology`.
- **`cc_*` ABI unchanged.** Layer 6 is an internal geometry-algorithm library; its consumers are later
  surfacing features, not the app today. No ABI is added until a consumer needs it — consistent with
  the demand-driven policy.

## Non-goals

- **No rational / weighted N-sided fill** — this module builds non-rational sub-patches from
  non-rational boundaries only and never fabricates weights.
- **No G1/G2 continuity across the interior spokes** — the midpoint-subdivision multi-patch matches the
  boundary POSITION exactly and is C0 at the interior spokes/centroid, but is NOT tangent (G1) or
  curvature (G2) continuous there. The Gregory / energy-minimizing plate patches that achieve G1/G2 at
  the interior seams (and to adjacent faces) are a distinct, harder residual.
- **No curved-N-sided interior fairing** — the interior spokes are straight lines to the centroid, so a
  strongly-curved 3D boundary is filled position-exactly on the boundary but with a piecewise-planar-ish
  interior (not an energy-minimized surface). Curved-interior fairing is a residual.
- No single-patch N-sided surface (a genuine trimmed/degenerate-corner tensor patch); no error-driven
  adaptive knot refinement; no new `cc_*` ABI; no change to STEP admission, the tessellator, or any
  evaluator signature.

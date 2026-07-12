# Design — nurbs-nsided-fill

## Placement & conventions

New module `src/native/math/bspline_nsided.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline_coons.{h,cpp}`. Reuses `math::Point3` (`native/math/vec.h`), the evaluator `curvePoint`
(`bspline.h`, corner + midpoint evaluation), the **Layer-1 exact** curve ops `splitCurve` /
`reparamCurve` (`bspline_ops.h`) to halve and re-domain the boundary edges, `bspline_coons`
(`coonsPatch`) for the per-quad fill, and the **Layer-1 data types** `BsplineCurveData` (the N
boundaries in) / `BsplineSurfaceData` (the N sub-patches out). OCCT-free, fp64, deterministic. Added
to the `native_math.h` aggregator.

**numsci gate.** The construction uses **no linear solve** — edge halving is exact Layer-1 knot
arithmetic and the per-quad fill is the exact boolean-sum Coons. The `.cpp` is nonetheless placed
under `CYBERCAD_HAS_NUMSCI` because it composes the numsci-gated `bspline_coons`, and to sit uniformly
with the rest of the numsci-gated Layer-6 surfacing family (the header declares everything; with the
guard OFF the implementation TU is inert and the functions are absent). `CYBERCAD_HAS_NUMSCI` is
defined library-wide, so `bspline_nsided.cpp` — in the default `src/native` glob — sees it when the
option is ON.

Conventions match the rest of the kernel: **flat clamped knot vectors**; **row-major, U-outer**
surface poles; **non-rational** (weights empty).

## The N-sided boundary

`NSidedBoundary { std::vector<BsplineCurveData> edges; }` — the N boundary curves in loop order. Edge
`edges[i]` runs on [0,1] from corner `V[i]` to corner `V[i+1]` (indices mod N). The loop is closed iff
`edges[i](1) == edges[(i+1)%N](0)` for every i (consecutive shared corners). N ≥ 3.

`verifyNSidedBoundary` requires N ≥ 3, every edge non-rational and well-formed (clamped flat knot
vector, degree ≥ 1, ≥ 2 poles), and the consecutive-corner closure within `tol`; it reports the worst
mismatch and declines (`ok=false`, reason) on any violation.

## Midpoint subdivision (Catmull-Clark N-gon → N quads)

A single Coons patch fills a topological quad. For N ≠ 4 there is no natural single tensor-product
quad, so the N-gon is subdivided into N quads by the classic **midpoint (Catmull-Clark) subdivision**:

- **Corners** `V[i] = edges[i](0)`.
- **Edge midpoints** `M[i] = edges[i](0.5)` (the split point).
- **Centroid** `C = (1/N)·Σ V[i]` — the single interior hub shared by all sub-patches. For a planar
  polygon centred at the origin, `C` is the origin, and `C` lies in the polygon's plane.

Sub-patch `i` is the quad CENTRED on corner `V[i]`, mapped to the Coons unit square `[0,1]²` with
corners `P00 = V[i]`, `P10 = M[i]`, `P01 = M[i-1]`, `P11 = C`:

| Coons edge | direction              | geometry                                    |
|------------|------------------------|---------------------------------------------|
| `c0` (v=0) | `V[i] → M[i]`          | FIRST half of boundary edge `e[i]`          |
| `d0` (u=0) | `V[i] → M[i-1]`        | SECOND half of `e[i-1]` REVERSED            |
| `c1` (v=1) | `M[i-1] → C`           | straight interior spoke                     |
| `d1` (u=1) | `M[i] → C`             | straight interior spoke                     |

The two outer edges are exact boundary sub-curves: `firstHalf(e)` = `splitCurve(e,0.5).left`
reparametrized to [0,1] (runs V→M); `secondHalfReversed(e)` = `splitCurve(e,0.5).right` reparametrized
to [0,1] then reversed (runs V(next)→M). `reverseCurve` mirrors the poles and complements the flat
knot vector (`k'[i] = a + b − k[n−i]`), geometry-exact. The corners of each quad coincide by
construction, so `coonsPatch` accepts each quad.

## Why the union interpolates all N boundaries

Boundary edge `e[k]` is covered by exactly TWO sub-patch outer edges: its first half by sub-patch `k`
(as `c0 = S_k(·,0)`) and its second half by sub-patch `k+1` (as `d0 = S_{k+1}(0,·)`, the reversal
undone). Each Coons patch interpolates its own boundary exactly (the boundary-containment guarantee of
`bspline_coons`), so the two halves together reproduce the full edge `e[k]` pointwise. The
**containment oracle** samples each edge against its two owning patches' exact iso-edges — because
each half-edge IS an exact iso-curve of one sub-patch, the residual is machine precision (~1e-15).

## C0 at the interior seams

Adjacent sub-patches `i` and `i+1` share the interior spoke `M[i]→C`: it is sub-patch `i`'s `d1`
(`S_i(1,·)`) and sub-patch `i+1`'s `c1` (`S_{i+1}(·,1)`) — the SAME straight degree-1 segment, so they
meet C0 there. All N sub-patches have `P11 = C`, so they all pass through the centroid exactly. This is
**C0, not G1** — tangent/curvature continuity across the spokes is a Gregory/plate residual.

## Degenerate & decline guards

`fillNSided` first calls `verifyNSidedBoundary` (declines on non-closed / rational / malformed / N<3).
It then guards against a degenerate spoke (an edge midpoint coinciding with the centroid, which would
collapse a sub-quad). Any sub-quad the Coons builder itself rejects propagates as an honest decline
(`ok=false`, reason) with the partial patches cleared — never a crash, never a silently-wrong net.

## Alternatives considered

- **A single degenerate-corner / trimmed tensor patch** (collapse one pair of a quad's edges to a
  point for N=3, or trim a big tensor patch to the N-gon). This gives one patch but a degenerate
  parametrization (a singular corner) and does not generalize cleanly to arbitrary N or reproduce a
  curved boundary exactly. The midpoint subdivision is simpler, exact on the boundary for every N, and
  reuses the verified Coons builder unchanged.
- **A Gregory / GeomPlate energy-minimized single patch** (OCCT's `BRepFill_Filling`). This achieves
  G1 continuity and a fair interior but needs a nonlinear plate solve and matches the boundary only
  approximately (a fit residual). It is the harder residual documented in `docs/NURBS-SCOPE.md`; the
  80/20 midpoint-subdivision fill is boundary-exact and solver-free.

## Residuals (honest)

Rational / weighted N-sided fill; **G1/G2** tangent/curvature continuity across the interior spokes
(this is C0 there); **curved-N-sided interior fairing** (the interior spokes are straight lines to the
centroid, so a strongly-curved 3D boundary is boundary-exact but has a piecewise interior); a genuine
single-patch N-sided surface. All recorded in `docs/NURBS-SCOPE.md` Layer-6 row.

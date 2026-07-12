# Proposal — nurbs-surface-join-g1-g2 (NURBS roadmap Layer 6)

## Why

The Layer-6 surfacing layer BUILDS many surfaces (skin, sweep, Coons, Gordon, N-sided), but it has no
operation to ENFORCE cross-boundary continuity between two surfaces that ALREADY EXIST. Two adjacent
NURBS patches produced independently — e.g. two faces of a healed B-rep, or a fill placed next to an
existing panel — commonly touch along a shared edge with POSITION continuity only (C0, a visible
crease), even when the design intent is a smooth (G1) or class-A (G2) transition. Rebuilding either
patch is expensive and destructive; the right tool is a POST-hoc continuity operator that nudges only
the near-boundary control rows to remove the crease while freezing the shared boundary curve.

This is worth building now because (a) it composes machinery that already exists — the Layer-1 exact
`elevateDegreeSurface` / `refineKnotSurface` for edge compatibility and the `bspline.h`
evaluators/derivatives for the residual measurement — with **no new numerical solver** (one scalar
least-squares for the minimal-movement proportionality); and (b) it has a **uniquely airtight,
closed-form oracle**: coplanar patches are a no-op, a C0 crease becomes machine-exact G1 by placing the
second control row on the collinear shared ribbon, G2 follows by matching the second cross-difference,
and two halves of a cylinder split along a knot line are already G2 (no-op). It is additive — every
existing surface-builder API is byte-unchanged.

Crucially it is HONEST about distortion: enforcing G1/G2 costs control-point movement, and beyond a
caller-supplied cap the surface would be visibly distorted. The module DECLINES an over-cap demand with
a clear reason rather than silently distorting the patch or widening a tolerance.

## What

A new OCCT-free module `src/native/math/bspline_join.{h,cpp}` (namespace `cybercad::native::math`,
beside the other Layer-6 surfacing modules), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, for family
uniformity with the rest of Layer-6; the construction itself uses only the exact Layer-1 ops and **no
linear solve** beyond one scalar least-squares). It adds an `EdgeSpec` (which boundary of each patch is
shared, and the along-edge orientation), and three entry points:

- **`makeCompatibleAlongEdge(A, B, edge)`** — knot-merge + degree-match along the shared edge so both
  patches share the same along-edge degree and knot vector (a precondition for the row-wise join). Exact
  — the represented geometry of neither surface changes (reuses Layer-1 `elevateDegreeSurface` /
  `refineKnotSurface`). Declines a non-coincident (not actually C0) shared edge, a malformed patch, or a
  rationality mismatch.
- **`joinG1(A, B, edge, cap)`** — reposition B's SECOND control row (optionally splitting the move onto
  A's second row) so the cross-boundary tangent ribbons are collinear ⇒ the unit normal is continuous
  (G1). The single global proportionality `s > 0` is the closed-form least-squares MINIMISER of the
  total row-1 movement. The boundary row is frozen (C0 + boundary curve preserved).
- **`joinG2(A, B, edge, cap)`** — additionally reposition B's THIRD control row so the cross-boundary
  second derivative matches (normal curvature continuous), G1 preserved, C0 preserved.

**Continuity conditions** (Piegl & Tiller ch.10; Farin), per along-edge control station `i`:

1. **Compatibility** — degree-elevate the lower-degree side and knot-merge (union of interior knots,
   remapped affinely) along the edge; verify the shared boundary rows coincide (C0).
2. **G1 shared ribbon** — with `P0[i]` the frozen boundary pole and `A1off[i] = A_row1[i] − P0[i]`, set
   `B_row1[i] = P0[i] − s·A1off[i]`. B's cross-boundary tangent is then antiparallel-collinear to A's,
   so the two tangent planes coincide and the unit normal is continuous for any `s > 0`. `s` minimises
   `Σ|B_row1_current[i] − (P0[i] − s·A1off[i])|²` in closed form.
3. **G2 shared ribbon** — with `A2off[i] = A_row2[i] − P0[i]`, set
   `B_row2[i] = P0[i] + s²·A2off[i] − 2s(s+1)·A1off[i]`, so B's cross-boundary second difference equals
   `s²·(A's)` ⇒ `∂²S/∂n²` continuous; row 1 and row 0 are untouched so G1 and C0 hold.
4. **No-op guard** — measure the achieved residual FIRST; if already continuous within tolerance, return
   with movement 0 (the coplanar and cylinder-split analytic cases).
5. **Movement cap** — if the required max displacement exceeds the caller cap, decline honestly (would
   distort the surface); never widen a tolerance to pass.

## Verification (HOST-analytic, the airtight oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_join.cpp`:
  1. **Coplanar no-op** — two co-planar bicubic patches already meeting G-infinity → `joinG1` /
     `joinG2` are no-ops (`maxMovement == 0`, residual already below tol).
  2. **G1 enforced** — two bicubic patches meeting only C0 (a genuine crease, start-mismatch > 1e-3) →
     after `joinG1` the unit normal is continuous across the shared edge on a dense sample to ≤ 1e-7 rad
     and the boundary curve is unchanged to ≤ 1e-12.
  3. **G2 enforced** — after `joinG2` the normal curvature is continuous across the edge (relative
     ≤ 1e-5) AND G1 still holds; boundary unchanged ≤ 1e-12.
  4. **Analytic cylinder** — two halves of a cubic-in-u cylinder wall split at its knot line already
     meet G2 → both joins are no-ops (movement 0). An over-cap `joinG1` (`cap = 1e-6`) on the crease
     honest-declines (`ok = false`, with a reason, reporting the required movement).

- `src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files); the `cc_*` facade is
  untouched (ABI byte-unchanged). Wired into `CMakeLists.txt` under `CYBERCAD_HAS_NUMSCI`.

## Impact

- New capability `makeCompatibleAlongEdge` / `joinG1` / `joinG2` + `EdgeSpec` / `JoinResult` in
  `bspline_join.{h,cpp}`; additive to every surface-builder API. Test `test_native_nurbs_join`. The
  Layer-6 surfacing row gains a post-hoc cross-boundary continuity operator (previously only builders
  existed); rational-join and multi-edge chained joins remain residuals.

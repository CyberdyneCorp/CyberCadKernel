# Proposal — nurbs-coons-surface (NURBS roadmap Layer 6)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel*, `src/native/math/bspline_ops.{h,cpp}`), Layer 7 (fitting /
approximation, `src/native/math/bspline_fit.{h,cpp}`), and the Layer-6 slices — **skinning /
lofting** (`bspline_skin.{h,cpp}`), **swept surfaces** (`bspline_sweep.{h,cpp}`), and the
**Gordon / network surface** (`bspline_gordon.{h,cpp}`) — are landed. Skinning fills ONE family of
parallel sections; a Gordon surface fills a REGULAR grid network of two transversal curve families.
The next Layer-6 capability is the **boundary-filled COONS patch**: given four boundary B-spline
curves forming a CLOSED four-sided loop (a topological quad — `c0(u)` at `v=0`, `c1(u)` at `v=1`,
`d0(v)` at `u=0`, `d1(v)` at `u=1`, corners shared), construct a single tensor-product B-spline
**surface that INTERPOLATES all four boundaries** — the surface's four edge iso-curves reproduce the
four input curves exactly. This is the core surfacing op for **filling holes**, **capping openings**
and **lofting a closed boundary** (the way designers fill a wireframe quad or cap an open shell edge).

This slice is worth building **now** because it (a) is small and well-bounded (*The NURBS Book*
§10.5 / the classic Coons construction — the bilinearly-blended patch is the **boolean sum**
`Coons = L_u ⊕ L_v ⊖ B`), (b) is built entirely on machinery that already exists — the **Layer-1
exact** curve and surface ops (`elevateDegreeCurve`/`refineKnotCurve`, `elevateDegreeSurface`/
`refineKnotSurface`) to make the boundary pairs compatible and bring the three summands to one common
basis so their control nets add/subtract, with the ruled and bilinear summands built directly (no
linear solve) — and (c) has a **uniquely airtight oracle**: the finished surface must contain every
input boundary curve *pointwise* (closed-form containment residual → 0), the four corners are
interpolated exactly, a flat boundary yields a flat patch, and a ruled/bilinear surface's own four
boundary iso-curves fed back through the Coons builder reconstruct it (Coons is exact for
bilinearly-blended surfaces). It composes verified lower layers into a new capability with a
machine-precision oracle, and DECLINES honestly on a mismatched-corner / incompatible boundary.

## What

A new OCCT-free module `src/native/math/bspline_coons.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_skin` / `bspline_gordon`), **numsci-gated**
(`CYBERCAD_HAS_NUMSCI`, like `bspline_gordon.cpp`) so it sits uniformly with the rest of the
numsci-gated Layer-6 surfacing family — though the construction itself uses only the exact Layer-1
ops and **no linear solve**. It reuses the Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types
for its input (the four boundary curves) and output (the surface). **Non-rational, four-sided
boundary only** (all weights = 1); rational / N-sided / Gregory-plate patches are explicit residuals.

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 (and Coons 1967 / Farin) — the
bilinearly-blended Coons patch as a BOOLEAN SUM:

    Coons = L_u ⊕ L_v ⊖ B

1. **Corner consistency** (`verifyCoonsBoundary`) — the four boundaries only form a valid quad when
   their shared corners actually coincide: `c0(0)==d0(0)`, `c0(1)==d1(0)`, `c1(0)==d0(1)`,
   `c1(1)==d1(1)` to within tolerance; every boundary non-rational and well-formed. `verifyCoonsBoundary`
   checks this and reports the worst corner mismatch; `coonsPatch` DECLINES (`ok=false`, with a
   reason) on a mismatched-corner / rational / degenerate boundary — never a surface that silently
   misses its own boundary.
2. **Boolean sum** (`coonsPatch`) — build the three summands: `L_v` = the ruled surface between `c0`
   and `c1` (linear blend in v; the c-shape carried in u, c0/c1 first made compatible in u by the
   exact Layer-1 ops); `L_u` = the ruled surface between `d0` and `d1` (linear blend in u; the
   d-shape carried in v, d0/d1 made compatible in v); `B` = the degree-(1,1) bilinear tensor product
   of the four corner points. Raise the three to a COMMON degree and merge them onto COMMON knot
   vectors in each direction with the exact Layer-1 surface ops, then form the Coons net pointwise
   `poles(Coons) = poles(L_v) + poles(L_u) − poles(B)`. The bilinear term `B` is exactly the part
   `L_u` and `L_v` share along the boundary, so the boolean sum interpolates every boundary curve: on
   `v=0`, `L_v` reduces to `c0` while `L_u` and `B` both reduce to the SAME straight line between the
   corners (so `L_u − B` cancels), giving `Coons(u,0) = c0(u)`; symmetrically for the other edges.

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_coons.cpp`:
  1. **Boundary containment (the core oracle)** — the Coons surface evaluated along each of its four
     edges reproduces the corresponding boundary curve POINTWISE on a dense sample to ~1e-9:
     `S(·,0)==c0`, `S(·,1)==c1`, `S(0,·)==d0`, `S(1,·)==d1` (achieved ~1.8e-15).
  2. **Corner interpolation** — the four corners are interpolated exactly (~1e-12; achieved 0).
  3. **Flat patch** — a flat (four coplanar, curved-but-in-plane) boundary of a rectangle yields a
     flat patch matching the plane exactly (every surface point on `z=0`, ~1e-12; achieved 0); a
     rectangular straight-edge boundary yields the exact planar bilinear patch.
  4. **Known-surface round-trip** — extract the four boundary iso-curves of a KNOWN tensor-product
     surface and Coons-fill them. For a RULED / bilinear surface (which IS bilinearly-blended) the
     original is recovered POINTWISE (~1e-9; achieved ~1.8e-15); for a general surface the boundary
     is reproduced exactly (the Coons interior is the bilinear blend by definition — verified as a
     containment, not an interior match, ~9e-16).
  5. **Honest declines** — a mismatched corner (a displaced boundary endpoint), a rational boundary,
     and a malformed boundary are declined (`ok=false`, with a reason and a large corner error),
     never a silently-wrong surface, never a crash.
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `GeomFill_BSplineCurves`
  (Coons style) boundary surfacing (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_coons.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_coons.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_gordon`.
- Only `#include`s `bspline_ops.h` (Layer 1) and the evaluators (`bspline.h`) — it does NOT modify
  them, and does NOT touch `bspline_skin` / `bspline_sweep` / `bspline_fit` (a concurrent rational
  track modifies skin/sweep; this change only shares the boolean-sum *pattern*, not their code).
- **`cc_*` ABI unchanged.** Layer 6 is an internal geometry-algorithm library; its consumers are
  later surfacing features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No rational / weighted Coons patches** — interpolating weighted boundaries is materially harder
  and is an explicit residual. This module builds non-rational patches from non-rational boundaries
  only and never fabricates weights.
- **No N-sided fill** — the bilinear Coons construction requires exactly FOUR boundaries (a
  topological quad). N-sided fill (5+ boundaries, degenerate-corner / triangular patches) remains a
  demand-gated residual.
- **No G1/G2 tangent/curvature-continuous plate blends** — the bilinearly-blended Coons patch matches
  the boundary POSITION exactly but not the surrounding surfaces' tangent/curvature. The Gregory /
  energy-minimizing plate patches that achieve G1/G2 continuity to adjacent faces are a distinct,
  harder residual.
- No error-driven adaptive knot refinement; no automatic degree/knot selection; no new `cc_*` ABI;
  no change to STEP admission, the tessellator, or any evaluator signature.

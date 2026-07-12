# Proposal — nurbs-nsided-fill-g1 (NURBS roadmap Layer 6)

## Why

The landed C0 N-sided fill (`bspline_nsided.{h,cpp}`, change `nurbs-nsided-fill`) fills a closed N-gon
boundary with N Coons sub-patches that meet the boundary and each other with POSITION continuity only —
a visible crease at every internal spoke and along the boundary tangent. `docs/NURBS-SCOPE.md` records
the **Gregory / plate G1 blend** as the explicit residual on the Layer-6 N-sided row. This change lands
that residual: a **G1 (tangent-plane continuous)** N-sided fill for N ≥ 3 that reproduces the boundary
exactly AND meets G1 across every internal spoke and to the boundary cross-tangent field.

This is worth building now because (a) it composes machinery that already exists — the same midpoint /
pie-slice subdivision topology plus the Layer-1 exact `elevateDegreeCurve` and the `bspline.h`
evaluators/derivatives — with **no new numerical solver**; and (b) it has a **uniquely airtight,
closed-form oracle**: boundary interpolation to machine precision, G1 across every spoke via POLE
EQUALITY (the two slices at a seam share the spoke column and a shared cross-spoke rib → identical
tangent plane → machine-exact normal continuity), and a planar boundary yields a planar fill. It is
additive — the C0 `fillNSided` / `verifyNSidedBoundary` API is byte-unchanged.

Crucially it is HONEST about the geometric limit: a tangent-plane-continuous surface cannot cross a
boundary that itself creases at a corner in 3-D (non-collinear edge tangents not coplanar with the
spoke). The module DECLINES that case with a clear reason rather than emitting a residual crease or
widening a tolerance — G1 is guaranteed exactly for tangent-continuous (smooth-corner) boundaries and
for planar N-gons, which are the realistic hole-fill inputs.

## What

A new OCCT-free module `src/native/math/bspline_nsided_g1.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_nsided`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, for family
uniformity with the rest of Layer-6; the construction itself uses only the exact Layer-1 ops and **no
linear solve**). It reuses the shared `NSidedBoundary` type and `verifyNSidedBoundary` from
`bspline_nsided.h`, and adds a `CrossTangentField` input (optional prescribed cross-boundary tangent per
edge) plus the `nSidedFillG1` entry point returning N Gregory bicubic sub-patches.

**Gregory pie-slice construction** (Chiyokura-Kimura; Piegl & Tiller ch.11; Farin):

1. **Loop consistency** — reuse `verifyNSidedBoundary` (non-closed / rational / degenerate / N<3
   decline honestly).
2. **Subdivide** — corners `V[i]=edges[i](0)`, centroid `C=mean(V[i])`. Slice `i` covers the full edge
   `e[i]` at `v=0` and shrinks to `C` at `v=1`; the two spokes `V[i]→C` and `V[i+1]→C` are the `u=0` /
   `u=1` edges, each SHARED with a neighbour.
3. **Gregory bicubic per slice** — u carries the edge's own (elevated-to-cubic, exact) basis so `v=0`
   reproduces `e[i]` pointwise (boundary interpolation exact); v is a cubic Hermite boundary→centre with
   the `v=0` cross-tangent equal to the prescribed / natural field (boundary G1). Adjacent slices share
   each corner spoke column AND a shared cross-spoke rib field (injected into the second / second-to-
   last u-columns) so their `∂S/∂u` along the seam is the identical vector → C1 (⇒ G1) by pole equality.
   The degenerate hub apex (all slices collapse to `C`) is the classic Gregory twist case, kept finite
   by the radial interior/twist row (no blowup).
4. **G1-feasibility guard** — decline a boundary that creases at a corner in 3-D (no tangent plane
   across the incident spokes), and decline a prescribed cross-tangent (anti-)parallel to the boundary
   tangent (no tangent plane at the boundary). Never widen a tolerance to pass.

## Verification (HOST-analytic, the airtight oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_nsided_g1.cpp`:
  1. **Boundary interpolation** — each slice's `v=0` iso reproduces its boundary curve to ≤ 1e-12
     (achieved ~1e-15), for a planar pentagon / triangle / heptagon and a smooth non-planar loop.
  2. **G1 across spokes** — the unit normal is continuous across every internal spoke to ≤ 1e-6 rad
     (achieved 0 for planar; ~2e-16 for a genuinely non-planar smooth loop, |z|≈0.53).
  3. **Planar sanity** — a planar boundary yields a fill with every point on the plane to ≤ 1e-10
     (achieved 0).
  4. **Degenerate hub (Gregory twist)** — the apex is finite and equals the centroid; no blowup.
  5. **Honest declines** — non-closed / rational / N<3 / malformed boundary, wrong tangent-field count,
     a G1-incompatible (parallel) prescribed cross-tangent, and a creased 3-D corner all decline
     (`ok=false`, with a reason). The creased-3-D case is contrasted with the C0 fill, which fills it
     with a measurable crease.

- `src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files); the `cc_*` facade is
  untouched (ABI byte-unchanged). Wired into `CMakeLists.txt` under `CYBERCAD_HAS_NUMSCI`.

## Impact

- New capability `nSidedFillG1` + `CrossTangentField` in `bspline_nsided_g1.{h,cpp}`; additive to the
  C0 API. Test `test_native_nurbs_nsided_g1`. `docs/NURBS-SCOPE.md` Layer-6 N-sided row: the Gregory G1
  blend moves from residual to landed (rational N-sided fill and G2 plate blends remain residuals).

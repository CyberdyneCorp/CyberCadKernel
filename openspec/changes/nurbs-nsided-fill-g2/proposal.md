# Proposal — nurbs-nsided-fill-g2 (NURBS roadmap Layer 6)

## Why

The landed G1 N-sided fill (`bspline_nsided_g1.{h,cpp}`, change `nurbs-nsided-fill-g1`) fills a closed
N-gon boundary with N Gregory bicubic pie-slices that meet the boundary and each other with TANGENT-PLANE
continuity (G1) — the unit normal is continuous across every internal spoke, but the second fundamental
form (the normal curvature) generally JUMPS at the seam: a curvature crease, visible in a reflection /
zebra analysis even when the surface looks smooth. `docs/NURBS-SCOPE.md` records the **G2 plate blend** as
the explicit residual on the Layer-6 N-sided row (left open by the G1 change). This change lands that
residual: a **G2 (curvature continuous)** N-sided fill for N ≥ 3 that reproduces the boundary exactly AND
meets G2 across every internal spoke and to the boundary cross-tangent + cross-curvature fields.

This is worth building now because (a) it composes machinery that already exists — the same midpoint /
pie-slice subdivision topology plus the Layer-1 exact `elevateDegreeCurve` and the `bspline.h`
evaluators/derivatives — with **no new numerical solver**; and (b) it has a **uniquely airtight,
closed-form oracle**: boundary interpolation to machine precision, G2 across every spoke via POLE
EQUALITY of the three seam-adjacent u-columns (the two slices at a seam carry byte-identical position +
1st-inward-rib + 2nd-inward-rib columns → identical `∂S/∂u` AND `∂²S/∂u²` → machine-exact unit-normal AND
normal-curvature continuity), and a sphere / cylinder boundary matching the analytic surface. It is
additive — the C0 `fillNSided` and G1 `nSidedFillG1` APIs are byte-unchanged.

Crucially it is HONEST about the geometric limit: a curvature-continuous surface across the incident
spokes requires the boundary to be G1-feasible (tangent-continuous or planar corners) AND
CURVATURE-continuous at its corners. A boundary that is only G1 (position + tangent matched but not the
second derivative — e.g. cubic Hermite arcs of a smooth loop) is curvature-CREASED at its corners; the
module DECLINES that case with a clear reason rather than emitting a residual curvature crease or widening
a tolerance. Likewise a prescribed cross-curvature whose incident-corner values point in opposing
directions is irreconcilable and declined.

## What

A new OCCT-free module `src/native/math/bspline_nsided_g2.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_nsided_g1`), **numsci-gated** (`CYBERCAD_HAS_NUMSCI`, for family
uniformity with the rest of Layer-6; the construction itself uses only the exact Layer-1 ops and **no
linear solve**). It reuses the shared `NSidedBoundary` type + `verifyNSidedBoundary` and the
`CrossTangentField` type from `bspline_nsided_g1.h`, adds a `CrossCurvatureField` input (optional
prescribed cross-boundary 2nd-derivative per edge) and the `nSidedFillG2` entry point returning N Gregory
quintic-in-v sub-patches.

**G2 Gregory pie-slice construction** (Chiyokura-Kimura; Piegl & Tiller ch.11; Farin):

1. **Loop consistency** — reuse `verifyNSidedBoundary` (non-closed / rational / degenerate / N<3
   decline honestly).
2. **Subdivide** — corners `V[i]=edges[i](0)`, centroid `C=mean(V[i])`. Slice `i` covers the full edge
   `e[i]` at `v=0` and shrinks to `C` at `v=1`; the two spokes `V[i]→C` and `V[i+1]→C` are the `u=0` /
   `u=1` edges, each SHARED with a neighbour. Pre-elevate each edge to degree ≥ 5 (exact Layer-1 A5.9)
   so each slice has three distinct seam-adjacent u-columns per side.
3. **Gregory quintic per slice** — u carries the edge's own (elevated, exact) basis so `v=0` reproduces
   `e[i]` pointwise (boundary interpolation exact); v is a **quintic (degree-5) G2 Hermite** boundary→hub
   carrying position + cross-tangent + cross-2nd-derivative at `v=0` and a radially-tapered hub at `v=1`.
   Adjacent slices share, per spoke, the position column AND a 1st-inward rib AND a 2nd-inward rib
   (injected into the three seam-adjacent u-columns' interior v-levels) so their `∂S/∂u` AND `∂²S/∂u²`
   along the seam are the identical vectors → C0 + G1 + G2 by pole equality. The degenerate hub apex is
   kept finite by radially tapering the interior 1st/2nd rows (the Gregory twist case), with no blowup.
4. **G2-feasibility guards** — decline a G1-infeasible creased corner and a cross-tangent (anti-)parallel
   to the boundary tangent (inherited from G1), decline a boundary that is curvature-DISCONTINUOUS at a
   corner (G1-only arcs), and decline a prescribed cross-curvature whose incident-corner values point in
   opposing directions. Never widen a tolerance to pass.

## Verification (HOST-analytic, the airtight oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_nsided_g2.cpp`:
  1. **Boundary interpolation** — each slice's `v=0` iso reproduces its boundary curve to ≤ 1e-12
     (achieved ~1e-15), for a planar pentagon / triangle / heptagon, a smooth non-planar G2 loop, and a
     spherical N-gon.
  2. **G2 across spokes** — the unit normal (≤ 1e-6 rad) AND the normal curvature (relative ≤ 1e-5) are
     continuous across every internal spoke (achieved 0 for planar; ~5e-7 curvature-rel for a genuinely
     non-planar smooth G2 loop; ~1e-16 for the sphere).
  3. **Analytic sphere sanity** — a G2 boundary on a sphere yields a fill whose boundary iso lies on the
     sphere (to the quintic-arc approximation of the rational circle) and whose seams are G2 (~1e-16).
  4. **Degenerate hub (Gregory twist)** — the apex is finite and equals the centroid; no blowup.
  5. **Honest declines** — non-closed / rational / N<3 / malformed boundary, wrong tangent- or
     curvature-field count, a G1-incompatible (parallel) prescribed cross-tangent, a curvature-creased
     boundary (G1-only arcs), and a corner-incompatible prescribed cross-curvature all decline
     (`ok=false`, with a reason).

- `src/native/**` stays OCCT-free (0 OCCT/Geom/BRep/TK refs in the changed files); the `cc_*` facade is
  untouched (ABI byte-unchanged). Wired into `CMakeLists.txt` under `CYBERCAD_HAS_NUMSCI`.

## Impact

- New capability `nSidedFillG2` + `CrossCurvatureField` in `bspline_nsided_g2.{h,cpp}`; additive to the
  C0 and G1 APIs. Test `test_native_nurbs_nsided_g2`. `docs/NURBS-SCOPE.md` Layer-6 N-sided row: the
  Gregory G2 blend moves from residual to landed (rational N-sided fill remains a residual).

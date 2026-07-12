# Proposal — nurbs-nsided-fill-rational (NURBS roadmap Layer 6)

## Why

The landed N-sided fill (`nurbs-nsided-fill`, `src/native/math/bspline_nsided.{h,cpp}`) fills a closed
N-gon boundary (N ≥ 3) with N Coons sub-patches by Catmull-Clark-style midpoint subdivision. It is
**non-rational**: it accepts non-rational boundaries only and builds non-rational sub-patches. When the
boundary curves are RATIONAL — e.g. exact circular arcs forming a rounded frame — that construction
cannot represent them; the "rational / weighted N-sided fill" was recorded as an explicit residual in
the original change's non-goals.

This slice lifts that residual. Rational boundaries are common (every exact arc, circle, ellipse, or
conic-derived edge is rational), and a designer capping such an opening expects the fill to REPRODUCE
the rational boundary EXACTLY, not to approximate it with a polynomial (a polynomial of any degree
cannot trace a circular arc). It is worth building now because it composes only machinery that already
exists: the Layer-1 exact `splitCurve` / `reparamCurve` already preserve rational curves (they operate
in homogeneous R⁴ internally), and the Coons boolean sum is exact in homogeneous space when the four
corner weights are made consistent. The oracle is airtight and closed-form: a boundary of exact
rational quarter-circle arcs is reproduced to machine precision, a weights-all-1 boundary reduces to the
existing non-rational fill exactly, and a planar rational boundary yields points on the plane.

## What

An **additive** extension of `src/native/math/bspline_nsided.{h,cpp}` (namespace
`cybercad::native::math`, `CYBERCAD_HAS_NUMSCI`-gated). The existing non-rational `verifyNSidedBoundary`
/ `fillNSided` API is **byte-unchanged**. New symbols:

- `verifyNSidedBoundaryRational(b, tol)` — like `verifyNSidedBoundary` but ACCEPTS rational edges,
  requiring exactly one STRICTLY-POSITIVE weight per pole (a zero / negative / mismatched weight is
  dishonest and declined); non-rational edges (empty weights) are accepted as w = 1.
- `NSidedFillRationalResult` + `fillNSidedRational(b, tol)` — perform the ENTIRE midpoint subdivision +
  per-corner Coons boolean sum in HOMOGENEOUS (wx, wy, wz, w) space, de-homogenizing only at the end.

**Homogeneous construction:**

1. **Loop consistency** (`verifyNSidedBoundaryRational`) — corners measured with the RATIONAL evaluator;
   `fillNSidedRational` DECLINES on a non-closed / malformed / non-positive-weight boundary.
2. **Subdivide** — corners `V[i] = edges[i](0)`, midpoints `M[i] = edges[i](0.5)` via `splitCurve`
   (which preserves the RATIONAL half-arc exactly), centroid `C = mean(V[i])`.
3. **Homogeneous Coons per corner** — the two boundary half-edges are rational-exact; the two interior
   spokes are straight lines carrying MATCHED corner weights (a rational-linear segment traces the same
   straight geometry for any positive endpoint weights, so the spoke inherits the arc's midpoint weight,
   making the four homogeneous corners consistent — Piegl & Tiller §10.5, rational Coons). The ruled and
   bilinear summands are lifted to R⁴, unified to a common basis by the exact Layer-1 surface ops, summed
   `L_u ⊕ L_v ⊖ B` in R⁴, and projected once (a non-positive projected weight is a hard guard).
4. **Non-rational reduction** — when every projected weight is 1 the weight vector is dropped, so a
   non-rational boundary yields sub-patches byte-identical in shape to `fillNSided`.

## Verification (HOST-analytic, closed-form oracles)

`tests/native/test_native_nurbs_nsided_rational.cpp` (host, numsci-gated):

1. **Rational-arc exactness (the core oracle)** — a rounded frame of 4 exact rational quarter-circle
   NURBS arcs; each rational boundary edge is reproduced by the sub-patch outer iso-curves to ≤1e-12
   (achieved ~1.2e-15), measured against the TRUE rational arc. A polynomial cannot trace a circle, so a
   near-machine residual proves the fill stayed rational throughout.
2. **Non-rational reduction** — a weights-all-1 (and empty-weights) boundary reproduces the existing
   `fillNSided` result POINTWISE to ≤1e-12 (achieved 0).
3. **Planar rational containment** — rational arcs in the `z=0` plane yield sub-patch points on that
   plane to ≤1e-10 (achieved 0).
4. **Honest declines** — non-closed loop, non-positive weight, N<3, malformed edge decline (`ok=false`
   with a reason), never a silently-wrong surface, never a crash.

## Scope

- Extends `src/native/math/bspline_nsided.{h,cpp}` — OCCT-free, numsci-gated, compiled via the existing
  `src/native` glob. No new module.
- Adds `tests/native/test_native_nurbs_nsided_rational.cpp` (host, numsci-gated) wired into CMake
  mirroring `test_native_nurbs_nsided`.
- The existing `verifyNSidedBoundary` / `fillNSided` and `bspline_coons` are UNCHANGED.
- **`cc_*` ABI unchanged** — additive internal geometry-algorithm capability only.

## Non-goals

- **No G1/G2 continuity across the interior spokes** — the rational fill matches the boundary POSITION
  exactly and is C0 at the interior spokes/centroid, exactly like the non-rational fill. Rational-G1
  Gregory/plate blends remain a residual.
- **No curved-N-sided interior fairing** — the interior spokes are straight lines to the centroid.
- No new `cc_*` ABI; no change to any evaluator signature or to the non-rational API.

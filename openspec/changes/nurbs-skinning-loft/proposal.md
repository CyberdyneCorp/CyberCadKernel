# Proposal — nurbs-skinning-loft (NURBS roadmap Layer 6)

## Why

`docs/NURBS-SCOPE.md` records the NURBS roadmap as a dependency-ordered stack. Layer 1 (the
exact-NURBS *geometry kernel* — knot/degree/split ops, `src/native/math/bspline_ops.{h,cpp}`)
and Layer 7 (fitting / approximation, `src/native/math/bspline_fit.{h,cpp}`) are landed.
Layer 6 is **skinning / lofting**: given a set of B-spline **section curves**, construct a single
tensor-product B-spline **surface** that *contains every section* as an iso-parametric curve — the
"cross-sections → smooth skin" direction of freeform surfacing (the Shapr3D loft workflow). The
sections define the shape along one parameter direction (U); the surface interpolates smoothly
*across* them in the other (V).

This layer is worth building **now** because it (a) is small and well-bounded (*The NURBS Book*
§10.3, Algorithm A10.3 — one compatibility pass + one V-interpolation), (b) is built entirely on
machinery that already exists — the Layer-1 data types and exact ops (`elevateDegreeCurve` /
`refineKnotCurve`) to make the sections compatible, plus the Layer-7 curve-interpolation path
(collocation + `numerics::lin_solve`) to interpolate across the sections — and (c) is **uniquely
airtight to verify**: after compatibility the surface's iso-curve at each section parameter must
equal that section *pointwise* (closed-form containment residual → 0), and a known tensor-product
surface's iso-curves, re-skinned, must reconstruct it. It composes verified lower layers into a
new capability with a machine-precision oracle.

## What

A new OCCT-free module `src/native/math/bspline_skin.{h,cpp}` (namespace
`cybercad::native::math`, beside `bspline_ops` / `bspline_fit`), **numsci-gated**
(`CYBERCAD_HAS_NUMSCI`, like `bspline_fit.cpp`) because the V-interpolation solves linear systems
through the numsci facade. It reuses the Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types as
its input (sections) and output (surface). **Non-rational sections only** (all weights = 1);
rational/weighted skinning is an explicit residual.

From *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 10:

1. **Section compatibility** (`makeSectionsCompatible`, §10.3) — given `K` sections of possibly
   different degree and knots, raise every section to the common maximum degree with
   `elevateDegreeCurve`, then merge every section onto the UNION knot vector (max multiplicity per
   distinct value) with `refineKnotCurve`. Afterwards every section shares degree `p`, knot vector,
   and control-point count `N`. Both Layer-1 ops are exact, so each compatible section still
   represents its ORIGINAL curve exactly (no geometry drift).
2. **Skinning** (`skinSurface`, Algorithm A10.3) — assign section parameters `v_k ∈ [0,1]` by chord
   length across the sections' control polygons (averaged over the `N` control-point indices); build
   a common averaging V-knot vector of degree `q = degreeV` (clamped to `K−1`). For each control-point
   index `i ∈ [0,N)`, interpolate a degree-`q` B-spline in V through the `K` points `{P_i^k}` at
   parameters `v_k` on the common V-knots (a single collocation matrix reused across control indices,
   one `lin_solve` per coordinate). The interpolated V-curves' control points assemble the
   `N × K` tensor-product surface net (row-major, U-outer), with U carrying the section shape
   (degree `p`, common section knots) and V the across-sections interpolation (degree `q`).

## Verification (HOST-analytic, the airtight-oracle is the whole point)

- **HOST (no OCCT), the primary gate** — `tests/native/test_native_nurbs_skin.cpp`:
  1. **Section containment (the core oracle)** — the skinned surface evaluated at `v = v_k`
     reproduces the compatible section `k` POINTWISE on a dense u-sample to ~1e-8 (achieved ~1e-15).
     The surface CONTAINS every input section exactly.
  2. **Compatibility correctness** — after raising/merging, all sections share degree + knots + `N`,
     and each compatible section still equals its ORIGINAL curve pointwise (achieved ~1e-15) — no
     geometry drift from elevation/refinement.
  3. **Known-surface round-trip** — extract `K` iso-curves from a KNOWN tensor-product surface as
     sections → skin → the surface contains every extracted iso-curve to ~1e-8 (achieved ~1e-15);
     and IDEMPOTENCE (skin → re-extract iso-curves at the same `v_k` → re-skin) reconstructs the
     surface POINTWISE to machine precision (achieved ~1e-15) — the strongest full-surface oracle,
     valid where the V-parametrization is a fixed point.
  4. **Degenerate guards** — fewer than two sections, coincident sections, rational sections, and
     incompatible-but-recoverable inputs are handled honestly (`ok=false` or graceful recovery, no
     crash).
- **SIM native-vs-OCCT parity** — OPTIONAL cross-check against OCCT `BRepOffsetAPI_ThruSections`
  (a separate track; HOST is primary and sufficient).

## Scope

- Adds `src/native/math/bspline_skin.{h,cpp}` — OCCT-free, numsci-gated (`CYBERCAD_HAS_NUMSCI`),
  compiled into the core lib via the existing `src/native` glob. Added to `native_math.h`.
- Adds `tests/native/test_native_nurbs_skin.cpp` (host, numsci-gated) wired into CMake mirroring
  `test_native_nurbs_fit`.
- Only `#include`s `bspline_ops.h` (Layer 1) and, transitively, the evaluators / numerics facade —
  it does NOT modify them.
- **`cc_*` ABI unchanged.** Layer 6 is an internal geometry-algorithm library; its consumers are
  later surfacing features, not the app today. No ABI is added until a consumer needs it —
  consistent with the demand-driven policy.

## Non-goals

- **No rational / weighted skinning** — interpolating the section weights is materially harder and
  is an explicit residual for a later slice. This module skins non-rational sections only and never
  fabricates weights.
- **No general Gordon / network / boundary surfacing** — skinning interpolates a single family of
  parallel sections; Gordon surfaces (two transversal curve families), N-sided / boundary-curve
  filling, and plate/energy surfaces remain demand-gated residuals.
- **No exact swept surfaces** — a NURBS sweep (section swept along a spine with orientation frames,
  GeomFill/BRepFill-class) is a distinct construction and remains a residual.
- No error-driven adaptive knot refinement; no automatic degree/knot selection; no new `cc_*` ABI;
  no change to STEP admission, the tessellator, or any evaluator signature.

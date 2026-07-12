# Proposal — nurbs-rational-surfacing (NURBS roadmap Layer 6, rational skinning + swept surfaces)

## Why

Layer 6 landed the **non-rational** side of surfacing: exact skinning / lofting
(`skinSurface`, change `nurbs-skinning-loft`), swept surfaces (`sweepTranslational` /
`sweepAlongTrajectory`, change `nurbs-swept-surface`), and Gordon / network surfaces
(`nurbs-gordon-surface`). All three explicitly deferred **rational (weighted) surfacing** as a
residual. Meanwhile Layer 7 landed **rational INTERPOLATION with prescribed weights**
(`nurbs-rational-fitting`) — the homogeneous-lift pattern: lift each datum to `(w·P, w) ∈ R⁴`,
run the SAME collocation solve on the 4-D net, project back. The Layer-1 ops
(`elevateDegreeCurve` / `refineKnotCurve`) are already rational-aware (they run on the
homogeneous net so weights ride through exactly).

This change lands the **tractable exact case** of rational surfacing on top of that machinery:
**rational skinning** and **rational swept surfaces**. Sections carry prescribed weights; the
surface must contain every rational section exactly, and a rational CIRCLE swept translationally
must be an EXACT rational CYLINDER — the strongest possible oracle (it proves an exact rational
surface, not a faceted approximation).

## What

Extend `src/native/math/bspline_skin.{h,cpp}` and `bspline_sweep.{h,cpp}` **additively** (no
existing signature changes) with:

- `makeRationalSectionsCompatible(sections)` — the rational analogue of
  `makeSectionsCompatible`: every section must be rational (non-empty, strictly-positive
  weights); raise to the common degree + merge to the union knots with the rational-aware
  Layer-1 `elevateDegreeCurve` / `refineKnotCurve` (homogeneous — weights preserved exactly), so
  each compatible section still equals its original rational curve pointwise.
- `skinRationalSurface(sections, degreeV)` — rational skin: compatibilize, assign section
  parameters `v_k` by chord length across the EUCLIDEAN control polygons, then for each control
  index interpolate a V-curve through the K homogeneous poles `(w·P_i^k, w_i^k)` (the SAME
  collocation matrix as the non-rational skin, solved for all FOUR homogeneous coordinates),
  and project back to a rational `BsplineSurfaceData` (poles + one weight per pole). The
  surface's iso-curve at `v = v_k` is exactly the rational section `k`.
- `sweepRationalTranslational(section, sweep)` — EXACT rational tensor product of a rational
  section (U) with a degree-1 two-pole path (V). The weight net is CONSTANT in V, so every
  iso-curve is the rational section translated by `v·sweep`, machine-exact.
- `sweepRationalAlongTrajectory(section, trajectory, sectionNormal, stations, degreeV)` — place
  the rational section at K stations by the SAME rotation-minimizing moving frame (a RIGID
  transform, which preserves the weights exactly), then RATIONAL-SKIN via `skinRationalSurface`.

The rational-lift convention is identical to `bspline_ops.h` (Layer 1) and `bspline_fit.h`
(Layer 7), so the non-rational and rational surfacing paths never diverge. A projected
**non-positive control weight** is a documented guard (decline rather than divide by ≤ 0).

## Verification (HOST-analytic, airtight rational oracles)

`tests/native/test_native_nurbs_skin.cpp` and `test_native_nurbs_sweep.cpp` (extended,
numsci-gated):

1. **Rational section containment** — the rational skinned/swept surface, evaluated as a
   rational NURBS, contains every rational input section pointwise to ~1e-9 (achieved ~1e-15).
2. **Exact rational cylinder** — a rational quadratic CIRCLE (cos-half-angle weights) swept
   translationally along an axis is an EXACT rational CYLINDER matching the analytic cylinder
   (circle profile × height) pointwise to ~1e-9 (achieved ~1e-15), every point at the exact
   radius (a true circle, not faceted) — the strongest oracle.
3. **Rational cone / frustum** — rational skinning of 2+ rational circles of DIFFERENT radii
   contains each rational circle, and every ring iso is a true circle of its prescribed radius.
4. **Guards** — a non-rational section (wrong path), a non-positive or mismatched weight, a
   rational trajectory, `<2` sections/stations, and coincident sections/trajectory all DECLINE
   honestly (`ok = false`, no crash, no faked rational surface).

## Scope

- Extends `src/native/math/bspline_skin.{h,cpp}` and `bspline_sweep.{h,cpp}` — additive only;
  existing `makeSectionsCompatible`, `skinSurface`, `sweepTranslational`,
  `sweepAlongTrajectory` signatures and behavior are byte-unchanged.
- Extends `tests/native/test_native_nurbs_skin.cpp` and `test_native_nurbs_sweep.cpp` (already
  CMake-wired, numsci-gated).
- Updates `docs/NURBS-SCOPE.md` §4 Layer-6 rows (non-rational + rational skinning/sweep landed;
  Gordon-rational + irregular / N-sided networks residual).
- **`cc_*` ABI unchanged.** `src/native` stays OCCT-free. No change to `bspline_fit`,
  `bspline_ops`, `bspline_gordon`, ssi, blend, boolean, or topology.

## Non-goals

- **No rational Gordon / network surface** — the boolean-sum Gordon construction in homogeneous
  space is a distinct slice and remains a residual.
- **No weight ESTIMATION** — sections carry PRESCRIBED weights; recovering weights from
  unweighted geometry is not attempted and never faked.
- **No irregular / N-sided networks, rotational (revolved) sweeps, or exact GeomFill/BRepFill
  continuous sweeps** — documented residuals.
- No new `cc_*` ABI.

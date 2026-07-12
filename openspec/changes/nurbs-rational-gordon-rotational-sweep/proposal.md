# Proposal — nurbs-rational-gordon-rotational-sweep (NURBS roadmap Layer 6, rational Gordon + rotational sweep)

## Why

Layer 6 has landed non-rational skinning/sweep/Gordon (`nurbs-skinning-loft`,
`nurbs-swept-surface`, `nurbs-gordon-surface`) and the rational skin + rational translational /
general sweep (`nurbs-rational-surfacing`). Two rational-surfacing capabilities were explicitly
left as residuals by those changes:

1. **Rational Gordon / network surface** — the boolean-sum Gordon construction
   `G = S_u ⊕ S_v ⊖ T` done in HOMOGENEOUS space for a WEIGHTED curve network.
2. **Rotational (revolved) sweep** — revolving a profile about an axis, a distinct
   exact-rational construction (a surface of revolution, not a skinned station approximation).

Both are the natural next slice: the rational-skin / rational-fit homogeneous-lift machinery is
already in place, and the rotational sweep is the exact rational-arc construction (*The NURBS
Book* §8.5 / A7.1) that turns a straight/tilted/semicircular profile into an EXACT
cylinder/cone/sphere — the strongest possible oracle (it matches the analytic surface of
revolution pointwise, proving an exact rational surface, not a facet).

## What

Extend `src/native/math/bspline_gordon.{h,cpp}` and `bspline_sweep.{h,cpp}` **additively** (no
existing signature changes) with:

- `verifyRationalNetwork(network, tol)` — the rational analogue of `verifyNetwork`: every u/v
  curve MUST be rational (non-empty, strictly-positive weights); grid consistency is checked with
  the RATIONAL evaluator (`nurbsCurvePoint`), so `C_k(u_l)` must equal `D_l(v_k)` (the Euclidean
  grid point) within tol. Same monotone-station / size preconditions.
- `gordonRationalSurface(network, tol, uInterpDegree, vInterpDegree)` — the rational boolean sum
  `G = S_u ⊕ S_v ⊖ T` done ENTIRELY in HOMOGENEOUS (wx,wy,wz,w) space: the two rational skins and
  the rational grid interpolant each run on the 4-D homogeneous net (the same collocation as the
  non-rational path, solved for all four coordinates), are brought to a common basis with the
  exact rational-aware Layer-1 `elevateDegreeSurface`/`refineKnotSurface` (weights ride through),
  and combined `homog(G) = homog(S_u) + homog(S_v) − homog(T)` then projected back. The network
  must be consistent in HOMOGENEOUS (weight) space at the grid — an honest precondition checked
  against tol. The rational Gordon surface CONTAINS every rational network curve pointwise.
- `sweepRotational(section, axisPoint, axisDir, angle)` — revolve a profile about an axis through
  a signed angle → an EXACT RATIONAL surface of revolution (§8.5 / A7.1). U carries the profile
  (its degree/knots/poles and, if rational, its WEIGHTS); V is a degree-2 rational arc split into
  `ceil(|angle|/90°)` ≤90° segments (`2·narcs+1` V-poles, interior multiplicity 2) with
  cos-half-angle middle weights. Each profile point contributes an ON-arc pole (arc weight 1) and
  a BETWEEN pole at radius `r/cos(Δθ/2)` (arc weight `cos(Δθ/2)`), radius `r` = the point's
  distance from the axis; the surface weight is the SEPARABLE product `wProfile·wArc`. An on-axis
  profile point keeps position `P` but STILL carries the arc weight pattern (forcing weight 1
  there would break separability and warp the revolve).

A projected **non-positive control weight** (rational Gordon) is a documented guard (decline
rather than divide by ≤ 0). The rational-lift convention is identical to `bspline_ops.h`,
`bspline_fit.h`, and `bspline_skin.h`.

## Verification (HOST-analytic, airtight rational oracles)

`tests/native/test_native_nurbs_gordon.cpp` and `test_native_nurbs_sweep.cpp` (extended,
numsci-gated):

1. **Rational Gordon containment** — extract a u/v network of RATIONAL iso-curves from a KNOWN
   rational surface (a rational revolved patch), build the rational Gordon surface, and confirm
   it CONTAINS every rational network curve pointwise to ~1e-9 (achieved ~7e-16), with the K×L
   rational grid points on the surface (~1e-9).
2. **EXACT rotational** — a straight segment offset from and parallel to the axis revolved 360°
   is an EXACT CYLINDER (radius/height exact ~1e-9, achieved ~1e-15); a tilted segment an EXACT
   CONE/frustum (~1e-15); a rational semicircle an EXACT SPHERE (every point at distance R
   ~1e-15) — each matching the analytic surface of revolution, the strongest oracle (proves an
   exact rational revolve, not a faceted approximation).
3. **Partial-angle revolve** — a 90° revolve is the correct rational quarter-cylinder sector
   (radius R exact, every point within the [0,90°] sector, `S(·,1)` = the profile rotated by the
   full angle); a 270° revolve forces 3 arc segments (7 V-poles) and stays analytic.
4. **Honest declines** — rational Gordon declines a non-rational / non-positive-weight /
   inconsistent (Euclidean- or homogeneous-inconsistent) / too-few-curve network; the rotational
   sweep declines a zero angle, a profile entirely on the axis, a null axis, a malformed profile,
   and a rational profile with a non-positive weight. Never a crash, never a faked surface.

## Scope

- Extends `src/native/math/bspline_gordon.{h,cpp}` and `bspline_sweep.{h,cpp}` — additive only;
  existing `verifyNetwork`, `gordonSurface`, `sweepTranslational`, `sweepAlongTrajectory`,
  `sweepRationalTranslational`, `sweepRationalAlongTrajectory` signatures and behavior are
  byte-unchanged.
- Extends `tests/native/test_native_nurbs_gordon.cpp` and `test_native_nurbs_sweep.cpp` (already
  CMake-wired, numsci-gated). CMake comment blocks only (no new source files — the library globs
  the module `.cpp`; the tests already link it).
- Updates `docs/NURBS-SCOPE.md` §4 Layer-6 row (rational Gordon + rotational revolve landed;
  irregular / N-sided networks + exact GeomFill/BRepFill + variable-section sweeps residual).
- **`cc_*` ABI unchanged.** `src/native` stays OCCT-free. No change to `bspline_fit`,
  `bspline_ops`, `bspline_skin`, `bspline.h`, `transform.h` (only `#include`d), ssi, blend,
  boolean, topology, or the concurrent `bspline_nsided` N-sided track.

## Non-goals

- **No irregular / N-sided networks** — non-grid topologies, trimmed boundaries, and N-sided
  fill remain residuals (a separate track adds `bspline_nsided`).
- **No exact GeomFill/BRepFill continuous sweeps** — an analytically exact swept surface rather
  than a skinned station approximation, and the ~1e-6 averaging-knot Gordon reconstruction
  residual, remain documented residuals.
- **No variable-section sweeps** — a section that morphs along the spine is not attempted.
- **No weight ESTIMATION** — rational Gordon operates on PRESCRIBED-weight networks.
- No new `cc_*` ABI.

# Tasks — nurbs-rational-gordon-rotational-sweep

## 1. Rational network verification
- [x] 1.1 `verifyRationalNetwork(network, tol)` in `bspline_gordon.{h,cpp}` — rational scope guard
      (every u/v curve rational, one strictly-positive weight per pole, well-formed knots, ≥2 per
      direction, matching param sizes, strictly-increasing stations); grid consistency checked
      with the RATIONAL evaluator `nurbsCurvePoint` (`C_k(u_l)` == `D_l(v_k)` within tol); grid
      holds the averaged Euclidean intersection points.

## 2. Rational Gordon boolean sum (§10.5, homogeneous)
- [x] 2.1 `homogCurvePoint` / `interpFamilyAcrossRational` / `interpGridRational` helpers — the
      rational analogues of the non-rational family/grid interpolation, all solving in homogeneous
      R⁴ `(w·P, w)` (four coordinates through the SAME collocation matrix); project back to
      (pole, weight); a projected non-positive weight clears `ok` (documented guard).
- [x] 2.2 `gordonRationalSurface(network, tol, uInterpDegree, vInterpDegree)` — verify (rational),
      rationally compatibilize both families (`makeRationalSectionsCompatible`), build the
      homogeneous grid `Qh_{k,l} = C_k^w(u_l)` and DECLINE if the two families disagree in
      homogeneous (weight) space at the grid (honest precondition), interpolate the three
      homogeneous summands `S_u`/`S_v`/`T`, bring to a common basis with the rational-aware
      `unifyDirection`, form `homog(G) = homog(S_u) + homog(S_v) − homog(T)`, project back. Guards:
      inconsistent/degenerate/non-rational network, singular interpolation, projected non-positive
      weight → `ok=false`.

## 3. Rotational (revolved) sweep (§8.5 / A7.1)
- [x] 3.1 `sweepRotational(section, axisPoint, axisDir, angle)` — degree-2 rational arc in V split
      into `ceil(|angle|/90°)` ≤90° segments (`2·narcs+1` V-poles, interior mult 2, cos-half-angle
      middle weights); each profile point revolved to its ON-arc / BETWEEN poles about the axis;
      surface weight = separable `wProfile·wArc` (the on-axis point keeps position P but STILL
      carries the arc weight pattern — forcing weight 1 there breaks separability and warps the
      revolve). U carries the profile (its degree/knots/poles + weights if rational). `vParams =
      {0,1}`. Guards: malformed profile, rational profile with a bad/non-positive weight, null
      axis, zero angle, whole-profile-on-axis (radius ≈ 0 everywhere) → `ok=false`.

## 4. HOST-analytic gate (no OCCT — airtight rational oracles)
- [x] 4.1 Extend `tests/native/test_native_nurbs_gordon.cpp`: rational containment (surface iso at
      v_k / u_l == rational network curve pointwise ~1e-9, achieved ~7e-16, extracted from a KNOWN
      rational revolved surface), rational grid on-surface (~1e-9),
      non-rational/non-positive-weight/inconsistent/<2-curve declines.
- [x] 4.2 Extend `tests/native/test_native_nurbs_sweep.cpp`: EXACT cylinder (straight offset
      segment revolved 360° — radius/height exact ~1e-9, achieved ~1e-15), EXACT cone/frustum
      (tilted segment ~1e-15), EXACT sphere (rational semicircle — every point at distance R
      ~1e-15), partial 90° revolve (correct arc sector + endpoint rotation), 270° revolve (3 arc
      segments, 7 V-poles), zero-angle/on-axis/null-axis/malformed/non-positive-weight declines.
- [x] 4.3 Update the CMake comment blocks for the gordon/sweep gates to record the rational-Gordon
      + rotational oracles (tests already wired; comments only). CMake `if/endif` balanced.

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: rational Gordon + rotational (revolved)
      sweep landed; irregular / N-sided networks + exact GeomFill/BRepFill + variable-section
      sweeps residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (zero regression; the
      skin/sweep/gordon consumers still pass). `cc_*` ABI byte-unchanged; `src/native` stays
      OCCT-free; `bspline_fit.h`/`bspline_ops.h`/`bspline.h`/`transform.h`/`bspline_skin.h` only
      `#include`d (not modified); `bspline_nsided` / ssi / blend / boolean / topology untouched.

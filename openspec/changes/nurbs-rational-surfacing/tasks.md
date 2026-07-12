# Tasks — nurbs-rational-surfacing

## 1. Rational section compatibility
- [x] 1.1 `makeRationalSectionsCompatible(sections)` in `bspline_skin.{h,cpp}` — rational scope
      guard (every section rational, one strictly-positive weight per pole, well-formed knots);
      raise to common degree + merge to union knots via the rational-aware Layer-1
      `elevateDegreeCurve` / `refineKnotCurve` (homogeneous — weights preserved); verify the
      post-condition includes matching weight counts.

## 2. Rational skinning (§10.3, homogeneous)
- [x] 2.1 `interpolateAcrossVRational` helper — same collocation matrix as the non-rational skin,
      solved for all FOUR homogeneous coordinates `(w·P, w)`; project each control point back to
      (pole, weight); a projected non-positive weight clears `ok` (documented guard).
- [x] 2.2 `skinRationalSurface(sections, degreeV)` — compatibilize (rational), section params by
      chord length across the Euclidean control polygons, homogeneous V-interpolation, assemble a
      rational `BsplineSurfaceData` (poles + weights). `degreeV` clamped to `K−1`. Guards: `<2`
      sections, non-rational/non-positive-weight section, coincident sections, projected
      non-positive weight → `ok=false`.

## 3. Rational swept surfaces (§10.4)
- [x] 3.1 `sweepRationalTranslational(section, sweep)` — EXACT rational tensor product: U = the
      rational section (poles + WEIGHTS), V = degree-1 two-pole path; weight net CONSTANT in V so
      every iso is the rational section translated. Guards: non-rational/non-positive/mismatched
      weight, malformed section, null sweep → `ok=false`.
- [x] 3.2 Factor the station-placement (sampling + RMF + rigid transform) into a shared helper
      `placeSectionsAlongTrajectory` reused by the non-rational and rational general sweeps (the
      rigid transform preserves weights exactly).
- [x] 3.3 `sweepRationalAlongTrajectory(...)` — place the rational section at K stations, then
      RATIONAL-SKIN via `skinRationalSurface`. Guards: `<2` stations, non-rational/non-positive-
      weight section, rational trajectory, malformed inputs, degenerate trajectory, skin failure.

## 4. HOST-analytic gate (no OCCT — airtight rational oracles)
- [x] 4.1 Extend `tests/native/test_native_nurbs_skin.cpp`: rational containment (surface iso at
      v_k == rational section pointwise ~1e-9), iso-curves are true circles, rational cone/frustum
      (different-radius circles, each ring a true circle), non-positive/mismatched-weight +
      non-rational/coincident/<2 declines, mixed-degree rational loft.
- [x] 4.2 Extend `tests/native/test_native_nurbs_sweep.cpp`: EXACT rational cylinder (rational
      circle swept translationally == analytic cylinder pointwise ~1e-15, every point at radius R),
      each iso == rational circle + v·axis; general rational sweep station containment (~1e-8);
      non-rational/non-positive/mismatched-weight + rational-trajectory + <2-station + coincident
      declines.
- [x] 4.3 Update the CMake comment blocks for the skin/sweep gates to record the rational oracles
      (tests already wired; comments only). CMake `if/endif` balanced.

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 rows: non-rational + rational skinning/sweep
      landed; Gordon-rational + irregular / N-sided networks + rotational/exact-BRepFill/variable-
      section sweeps residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (zero regression; the
      Gordon gate, which composes the skin, still passes). `cc_*` ABI byte-unchanged; `src/native`
      stays OCCT-free; `bspline_fit.h` / `bspline_ops.h` / `bspline.h` / `transform.h` only
      `#include`d (not modified); `bspline_gordon` / ssi / blend / boolean / topology untouched.

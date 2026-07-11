# Tasks — nurbs-fitting-approximation

## 1. Foundation — module + parametrization + knots
- [x] 1.1 Create `src/native/math/bspline_fit.h` — `ParamMethod`, `CurveFitResult`,
      `SurfaceFitResult`, `PointGrid`, and the parametrization / knot / fit declarations. Reuses
      the Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types. Add to `native_math.h`.
- [x] 1.2 `assignParams` (uniform / chord-length / centripetal, §9.2.1) — monotone, `[0,1]`,
      duplicate points share a parameter, all-coincident input returns empty (honest guard).
- [x] 1.3 `averagingKnots` (Eq 9.8) + `approxKnots` (Eq 9.68/9.69).

## 2. Curve fitting
- [x] 2.1 `interpolateCurve` (A9.1) — collocation matrix + `numerics::lin_solve` per coordinate;
      passes through every point; degenerate input declines.
- [x] 2.2 `approximateCurve` (A9.4/9.6) — `H < N` control points, endpoints pinned, free interior
      via `numerics::lstsq`; reports achieved max / RMS error.

## 3. Surface fitting (tensor-product, reusing the curve core per row/column)
- [x] 3.1 Shared `fitLine` (interpolation via `lin_solve` / approximation via `lstsq`) + grid
      parameter averaging (§9.2.5).
- [x] 3.2 `interpolateSurface` (fit each row in V, then each column in U).
- [x] 3.3 `approximateSurface` (least-squares in each direction; endpoints pinned).

## 4. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 4.1 `tests/native/test_native_nurbs_fit.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_ssi_seeding`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)` for the TU).
- [x] 4.2 Interpolation exactness: curve + surface pass through EVERY input point to ~1e-9
      (achieved ~1e-15).
- [x] 4.3 Round-trip recovery (idempotence): interpolate → resample at nodes → interpolate ≡
      original pointwise to ~1e-9 (achieved ~1e-14).
- [x] 4.4 Approximation error: reported error is the achieved error (no widening) and decreases
      monotonically as `H` grows toward `N`; endpoints pinned exactly.
- [x] 4.5 Parametrization sanity: chord-length + centripetal in `[0,1]`, monotone; duplicate /
      all-coincident handled; honest declines (too-few control points, degenerate input).

## 5. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 5.1 `tests/sim/native_nurbs_fit_parity.mm` cross-checking `GeomAPI_Interpolate` /
      `GeomAPI_PointsToBSpline` for a couple of curves. HOST is primary and sufficient; this is a
      separate track (simulator shared with concurrent tracks).

## 6. Docs & close-out
- [x] 6.1 Update `docs/NURBS-SCOPE.md` §2/§4 Layer-7 rows: ❌ → 🟡 partial (non-rational curve +
      surface interp/approx landed; rational + advanced surfacing residual).
- [x] 6.2 Ran `openspec validate --all --strict` (pass), full host ctest (82/82 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline_ops.h` only `#include`d (not modified); `ssi/blend/boolean` untouched.

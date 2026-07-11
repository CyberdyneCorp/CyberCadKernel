# Tasks — nurbs-rational-fitting

## 1. Rational curve interpolation (prescribed weights)
- [x] 1.1 `interpolateRationalCurve(points, weights, degree, method)` in `bspline_fit.{h,cpp}` —
      lift each datum to `Qʷₖ = (wₖQₖ, wₖ)`, solve the SAME averaging-knot collocation matrix as
      `interpolateCurve` for all four homogeneous coordinates (`lin_solve`), project back to
      (pole, weight). Additive — no existing signature changed.
- [x] 1.2 Guards: `weights.size() == points.size()`, every input weight strictly positive,
      `points.size() ≥ degree+1`, non-degenerate params, and every SOLVED control weight strictly
      positive (a projected non-positive weight → decline, never divide by ≤ 0).

## 2. Rational surface interpolation (prescribed weights)
- [x] 2.1 `WeightGrid` struct (parallel to `PointGrid`) + `interpolateRationalSurface(grid, wg,
      degreeU, degreeV, method)` — lift the grid to R⁴, tensor-interpolate each row in V then each
      column in U on the 4-D net, project back to (pole, weight).
- [x] 2.2 Guards: weight-grid dims match the point grid, every input weight positive, grid large
      enough for the degrees, every solved control weight positive.

## 3. HOST-analytic gate (extend the existing airtight gate)
- [x] 3.1 Rational interpolation exactness — curve + surface pass through every EUCLIDEAN datum to
      ~1e-9 (achieved ~1e-15) for arbitrary positive prescribed weights.
- [x] 3.2 Known-rational round-trip — sample a KNOWN rational unit CIRCLE (and half-CYLINDER) with
      the exact rational denominators as prescribed weights; recover the conic at every node to
      ~1e-15 and reconstruct it POINTWISE by rational idempotence to ~1e-15.
- [x] 3.3 Recovered weights match the input (idempotent round-trip) to ~1e-10 (up to the invariant
      common weight scale); projected poles match to ~1e-10.
- [x] 3.4 Guards: mismatched weight count/dims, zero/negative weight, all-coincident / too-few
      points, and a wild weight set that drives a control weight non-positive — all decline
      honestly (`ok = false`, no crash, no faked curve).

## 4. Docs & close-out
- [x] 4.1 Update `docs/NURBS-SCOPE.md` §4 Layer-7 rows: non-rational + rational-with-prescribed-
      weights landed; **weight ESTIMATION (Ma–Kruth)** called out as the residual.
- [x] 4.2 `openspec validate --all --strict` passes; full host ctest green (89/89, zero
      regression — skin/sweep/offset/thicken/gordon consumers still pass, additive change only);
      `cc_*` ABI byte-unchanged; `src/native` stays OCCT-free; `bspline_ops`/ssi/blend/boolean/
      topology untouched.

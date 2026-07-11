# Tasks — nurbs-surface-offset

## 1. Foundation — module + guards
- [x] 1.1 Create `src/native/math/bspline_offset.h` — `OffsetStatus`, `OffsetResult`, and the
      `offsetSurface(S, d, tol, startGrid, maxGrid)` declaration. Reuses the Layer-1
      `BsplineSurfaceData` type. Add to `native_math.h`.
- [x] 1.2 `principalCurvatures(u,v)` — first + second fundamental forms from `surfaceDerivs` /
      `nurbsSurfaceDerivs` (order 2); Gaussian `K`, mean `H`, `κ = H ± √(max(0, H²−K))`.
      Near-null-normal / degenerate-metric guard.
- [x] 1.3 Guard pass — dense analysis grid (`≥ 11×11`): degenerate normal → `DegenerateNormal`;
      `min(1 + d·κ) ≤ 0` over the grid → `SelfIntersection` (offset would fold), reporting the
      minimum curvature radius on the folding side. NEVER return a folded surface.

## 2. Sample → fit → refine (Ch. 10 offset approximation)
- [x] 2.1 Sample the true offset locus `O = S + d·N` on a `g × g` uniform parameter grid
      (rational-aware `evalS` + `surfaceNormal`).
- [x] 2.2 Fit through the samples with the Layer-7 `interpolateSurface` (degree `min(3,·)`,
      `lin_solve` collocation).
- [x] 2.3 GEOMETRIC deviation metric — a fixed cell-centred `7×7` check grid over the fitted
      surface, projected onto `S` via `numerics::closest_point_on_surface`; error =
      `max |dist − |d||` (parametrization-independent). Refine (`g ← 2g−1`, cap `maxGrid`) until
      error ≤ `tol` or budget spent; keep the best fit; report the TRUE achieved error (flag
      `ToleranceNotMet` if short — never widened).

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_offset.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_fit`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Offset distance (core oracle): every point of the fitted offset of a curved bicubic bump,
      projected onto `S`, is at distance `|d|` within the reported tol (both signs).
- [x] 3.3 Analytic cross-check: NURBS cylinder radius `r` → offset on radius `r±d` (to fit tol);
      plane → exact parallel plane (~1e-9).
- [x] 3.4 Error convergence: reported max offset error decreases monotonically as the grid refines
      (`~O(h⁴)`), honest / never widened.
- [x] 3.5 Self-intersection guard: over-radius offset of a tight dome is DECLINED
      (`SelfIntersection`, `ok=false`, empty surface), reported curvature radius ≈ dome radius; a
      safe small offset succeeds; a near-null-normal patch declines. No crash.

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_offset_parity.mm` cross-checking OCCT
      `BRepOffsetAPI_MakeOffsetShape` / `BRepOffset` for a couple of offsets. HOST is primary and
      sufficient; this is a separate track (simulator shared with concurrent tracks).

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-5 row (NURBS offset / thicken): analytic-walls-only
      → 🟡 partial (surface offset landed; solid thicken/shell + robust self-intersection trimming +
      rational residual).
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (84/84 pass, zero
      regression — `test_native_ssi_curved_boolean` is a pre-existing slow SSI test, not touched).
      `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline.h` / `bspline_ops.h` / `bspline_fit.h` only `#include`d (not modified);
      `ssi/blend/boolean/topology` untouched.

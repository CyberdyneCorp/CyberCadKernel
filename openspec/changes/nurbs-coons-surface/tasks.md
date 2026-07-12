# Tasks — nurbs-coons-surface

## 1. Foundation — module + four-sided boundary + corner check
- [x] 1.1 Create `src/native/math/bspline_coons.h` — `CoonsBoundary`, `CoonsCornerCheck`,
      `CoonsResult`, and the `verifyCoonsBoundary` / `coonsPatch` declarations. Reuses the Layer-1
      `BsplineCurveData` / `BsplineSurfaceData` types. Add to `native_math.h`.
- [x] 1.2 `verifyCoonsBoundary` — non-rational + well-formed guard for all four boundaries (degree
      ≥ 1, clamped knot vector, ≥ 2 poles); corner coincidence `c0(0)==d0(0)`, `c0(1)==d1(0)`,
      `c1(0)==d0(1)`, `c1(1)==d1(1)` within `tol`; report `maxCornerError`; decline honestly with a
      reason on any violation (`ok=false`).

## 2. Boolean sum (§10.5 / Coons)
- [x] 2.1 `unifyCurves` — raise two curves to the common max degree (`elevateDegreeCurve`) then merge
      to the union knot vector (`refineKnotCurve`), both exact Layer-1 ops (no geometry drift), so an
      opposing pair shares degree/knots/N. Used for `c0`/`c1` (in u) and `d0`/`d1` (in v).
- [x] 2.2 The three summands — `ruledInV(c0,c1)` = `(1−v)c0 + v·c1` (degree 1 in v, c-shape in u);
      `ruledInU(d0,d1)` = `(1−u)d0 + u·d1` (degree 1 in u, d-shape in v); `bilinearCorners` = the
      degree-(1,1) bilinear tensor of the four corner points. Row-major U-outer nets, weights empty.
- [x] 2.3 `unifyDirection` (per direction) — raise two surfaces to the common max degree
      (`elevateDegreeSurface`) then merge to the union knot vector (`refineKnotSurface`), both exact
      Layer-1 ops. Fold `L_v`/`L_u`/`B` pairwise (two passes) until all three share one basis;
      `sameBasis` verifies the common shape.
- [x] 2.4 `coonsPatch` — verify → compatibilize the opposing pairs → build `L_v`, `L_u`, `B` →
      unify to a common basis → form the Coons net `poles(L_v) + poles(L_u) − poles(B)`.
      Non-rational; exactly four boundaries; decline honestly on mismatched-corner / degenerate /
      rational input or a failed common-basis merge.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_coons.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_gordon`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Boundary containment (the core oracle): the surface's four edges `S(·,0)==c0`, `S(·,1)==c1`,
      `S(0,·)==d0`, `S(1,·)==d1` pointwise on a dense sample to ~1e-9 (achieved ~1.8e-15), for a
      general curved (non-planar) boundary.
- [x] 3.3 Corner interpolation: the four surface corners equal the boundary corners exactly (~1e-12,
      achieved 0).
- [x] 3.4 Flat patch: a coplanar (curved-but-in-plane) rectangle boundary → every surface point on
      the `z=0` plane (~1e-12, achieved 0); a rectangular straight-edge boundary → the exact planar
      bilinear patch.
- [x] 3.5 Known-surface round-trip: a RULED / bilinearly-blended surface recovered POINTWISE from its
      four boundary iso-curves (~1e-9, achieved ~1.8e-15 — Coons exact for bilinearly-blended
      surfaces); a general tensor surface's four boundaries contained pointwise (~9e-16).
- [x] 3.6 Honest declines: mismatched corner (a displaced boundary endpoint, large `maxCornerError`),
      rational boundary, malformed boundary — all `ok=false` with a reason, no crash; a consistent
      boundary still succeeds (the guard is not over-eager).

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_coons_parity.mm` cross-checking OCCT `GeomFill_BSplineCurves`
      (Coons style) for a couple of boundaries. HOST is primary and sufficient; this is a separate
      track (simulator shared with concurrent tracks).

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: skinning+sweep+Gordon+Coons now partial;
      N-sided + rational boundaries + Gregory/plate G1/G2 blends residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (92/92 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline_ops.h` only `#include`d (not modified); `bspline_skin`/`bspline_sweep`/`bspline_fit`
      untouched (concurrent rational track owns them); `ssi/blend/boolean/topology` untouched.

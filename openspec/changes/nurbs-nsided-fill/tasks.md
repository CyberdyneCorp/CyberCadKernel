# Tasks — nurbs-nsided-fill

## 1. Foundation — module + N-sided boundary + loop check
- [x] 1.1 Create `src/native/math/bspline_nsided.h` — `NSidedBoundary`, `NSidedBoundaryCheck`,
      `NSidedFillResult`, and the `verifyNSidedBoundary` / `fillNSided` declarations. Reuses the
      Layer-1 `BsplineCurveData` / `BsplineSurfaceData` types. Add to `native_math.h`.
- [x] 1.2 `verifyNSidedBoundary` — N ≥ 3 guard; non-rational + well-formed guard for every edge
      (degree ≥ 1, clamped knot vector, ≥ 2 poles); closed-loop corner coincidence
      `edges[i](1)==edges[(i+1)%N](0)` within `tol`; report `maxCornerError`; decline honestly with a
      reason on any violation (`ok=false`).

## 2. Midpoint subdivision → N Coons sub-patches
- [x] 2.1 Edge-halving helpers — `firstHalf(e)` = `splitCurve(e,0.5).left` reparametrized to [0,1]
      (runs V→M); `secondHalfReversed(e)` = `splitCurve(e,0.5).right` reparametrized to [0,1] then
      reversed (runs V(next)→M); `reverseCurve` (mirror poles + complement the flat knot vector,
      geometry-exact); `lineEdge` (straight degree-1 spoke). All exact Layer-1 ops.
- [x] 2.2 `fillNSided` — verify → compute corners `V[i]`, midpoints `M[i]`, centroid `C=mean(V[i])` →
      per corner build the quad (c0=first half of e[i], d0=second half of e[i-1] reversed, c1=spoke
      M[i-1]→C, d1=spoke M[i]→C) → `coonsPatch` each. Degenerate-spoke guard (a midpoint coinciding
      with the centroid). Decline honestly on a sub-quad the Coons builder rejects; return the N
      sub-patches + the centroid.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_nsided.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_coons`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Boundary containment (the core oracle): the N sub-patches together contain all N boundary
      curves pointwise via the exact outer iso-edges (~1e-9), for a planar pentagon (~9e-16), a planar
      triangle (~9e-16), and a mildly-curved in-plane hexagon (~1.4e-15).
- [x] 3.3 Planar N-gon → flat: every sub-patch point on the `z=0` plane for the pentagon / triangle /
      curved hexagon (~1e-12, achieved 0); the pentagon centroid == origin.
- [x] 3.4 C0 interior junctions: adjacent sub-patches share the interior spoke `M[i]→C` (S_i(1,·) ==
      S_{i+1}(·,1)) and the centroid (S(1,1)==C) exactly (~1e-12, achieved 0).
- [x] 3.5 N=4 consistency: the 4-sub-patch union contains the four boundary edges (~1e-9) and the
      single Coons patch of the same boundary reproduces them exactly — the two constructions agree on
      the boundary.
- [x] 3.6 Honest declines: non-closed loop (a displaced corner, large `maxCornerError`), rational edge,
      N<3, malformed edge — all `ok=false` with a reason, no crash; a consistent pentagon still
      succeeds (the guard is not over-eager).

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_nsided_parity.mm` cross-checking OCCT N-sided filling
      (`BRepFill_Filling` / `GeomPlate`) for a couple of boundaries. HOST is primary and sufficient;
      this is a separate track (simulator shared with concurrent tracks). OCCT's GeomPlate fills with a
      G1 energy-minimized single patch, so the parity is boundary-containment, not interior-match.

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: N-sided fill (midpoint subdivision → N Coons
      sub-patches) now partial; rational N-sided + Gregory/plate G1/G2 across the spokes +
      curved-N-sided interior fairing residual.
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (94/94 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline_ops.h` / `bspline_coons.h` only `#include`d (not modified); `bspline_skin` /
      `bspline_sweep` / `bspline_gordon` / `bspline_fit` untouched (concurrent rational track owns
      gordon/sweep); `ssi` / `blend` / `boolean` / `topology` untouched.

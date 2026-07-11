# Tasks — nurbs-skinning-loft

## 1. Foundation — module + section compatibility
- [x] 1.1 Create `src/native/math/bspline_skin.h` — `SectionCompatibility`, `SkinResult`, and the
      `makeSectionsCompatible` / `skinSurface` declarations. Reuses the Layer-1
      `BsplineCurveData` / `BsplineSurfaceData` types. Add to `native_math.h`.
- [x] 1.2 `makeSectionsCompatible` (§10.3) — raise all sections to the common max degree
      (`elevateDegreeCurve`), merge to the UNION knot vector (max multiplicity per distinct value,
      `refineKnotCurve`); verify identical degree + knots + control-point count post-condition.
      Non-rational guard (rational section → `ok=false`); malformed/degenerate guards.

## 2. Skinning (A10.3)
- [x] 2.1 Section parameters `v_k` — chord length across the sections' control polygons, averaged
      over the `N` control-point indices; all-coincident → empty (honest guard). Averaging V-knots
      (Eq 9.8); `q = clamp(degreeV, 1, K−1)`.
- [x] 2.2 `interpolateAcrossV` — one `K×K` collocation matrix reused across control indices;
      per control index, three RHS (x/y/z) solved via `numerics::lin_solve`; singular → decline.
- [x] 2.3 `skinSurface` — compose compatibility + V-interpolation into the `N×K` row-major U-outer
      `BsplineSurfaceData` (U = section shape, V = across-sections). ≥2 sections; non-rational.

## 3. HOST-analytic gate (no OCCT — the airtight-oracle primary gate)
- [x] 3.1 `tests/native/test_native_nurbs_skin.cpp` + CMake wiring (numsci-gated, mirroring
      `test_native_nurbs_fit`: `_SRC` + `CYBERCAD_TESTS` under `CYBERCAD_HAS_NUMSCI`, plus the
      per-target `target_compile_definitions(... CYBERCAD_HAS_NUMSCI=1)`).
- [x] 3.2 Section containment: surface iso-curve at `v_k` == section `k` pointwise on a dense
      u-sample to ~1e-8 (achieved ~1e-15) — the core oracle.
- [x] 3.3 Compatibility correctness: mixed degree/knot sections → shared degree+knots+N, each
      compatible section still == its original pointwise (achieved ~1e-15).
- [x] 3.4 Known-surface round-trip: extract `K` iso-curves from a KNOWN surface → skin → contains
      each iso-curve (~1e-15); IDEMPOTENCE full-surface pointwise identity (~1e-15).
- [x] 3.5 Degenerate guards: <2 sections, coincident sections, rational sections, and
      incompatible-but-recoverable pairs handled honestly (no crash).

## 4. SIM native-vs-OCCT parity gate — OPTIONAL FOLLOW-UP (not this pass)
- [ ] 4.1 `tests/sim/native_nurbs_skin_parity.mm` cross-checking OCCT
      `BRepOffsetAPI_ThruSections` for a couple of lofts. HOST is primary and sufficient; this is a
      separate track (simulator shared with concurrent tracks).

## 5. Docs & close-out
- [x] 5.1 Update `docs/NURBS-SCOPE.md` §4 Layer-6 row: ❌ → 🟡 partial (non-rational skinning
      landed; general Gordon/network/rational/exact-sweep residual).
- [x] 5.2 Run `openspec validate --all --strict` (pass), full host ctest (83/83 pass, zero
      regression). `cc_*` ABI byte-unchanged (no ABI file touched); `src/native` stays OCCT-free;
      `bspline_ops.h` / `bspline_fit.h` only `#include`d (not modified); `ssi/blend/boolean`
      untouched.

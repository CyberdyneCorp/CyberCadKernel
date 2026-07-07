# Tasks — moat-m4r-rational-bspline-step (MOAT M4-rational, first slice)

Order: baseline capture → shared grid parse factor (byte-identical) → rational-surface
read arm → host analytic gate → sim native-vs-OCCT parity gate → zero-regression proof →
docs, or HONEST DECLINE. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::exchange`. No `cc_*` ABI change. The
tessellator is NOT modified (the M0 rational mesh path already consumes `weights`). No
tolerance is weakened; a correct decline (rational patch stays OCCT) is a first-class
outcome.

## 0. Substrate + baseline (capture BEFORE touching the reader)

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host` (both
  exit 0; `libnumsci_iossim_arm64.a` + `libnumsci_host.a` produced); `CYBERCAD_NUMSCI_DIR`
  exported to `build-numsci/host` (host CTest) / `build-numsci/iossim` (sim).
- [x] 0.2 GREEN baseline captured pre-change: host reader test 41/41 (incl. the M4 non-
  rational `foreign_trimmed_bspline_*` cases). Full host CTest and the sim STEP-import
  harness re-run post-change (see §5) for the byte-identical comparison.

## 1. Shared grid parse factor (must be byte-identical)

- [x] 1.1 Extracted `fillBsplineGrid(degU, degV, polesArg, uMultsArg, vMultsArg,
  uKnotsArg, vKnotsArg, FaceSurface& out)` from `bsplineSurface()` — pole-grid read +
  `expandKnots` + `nPolesU/nPolesV` + knot-length validation
  (`step_reader.cpp:1362`).
- [x] 1.2 Re-pointed `bsplineSurface()` at the helper (`step_reader.cpp:1391`). The 41
  existing reader cases (incl. both M4 non-rational fixtures) pass byte-identical; full
  host CTest 36/36 and the sim STEP-import harness (box/cyl/torus/sphere/ellipsoid/
  bspline-revolution/bumpcap/assembly/multisolid) unchanged (§5).

## 2. Rational-surface read arm

- [x] 2.1 Added `hasSub` / `findSub` (`step_reader.cpp:1350`) — sub-record scan mirroring
  `relationshipAndTransform()`.
- [x] 2.2 `surface()` now routes a `r->combined` record carrying a
  `RATIONAL_B_SPLINE_SURFACE` sub to `rationalBsplineSurface(*r)`; every OTHER combined
  surface keeps the `nullopt` decline (`step_reader.cpp:918`). Regression-guarded by the
  host `combined_bspline_surface_without_rational_sub_declines` case.
- [x] 2.3 `rationalBsplineSurface()` (`step_reader.cpp:1409`): reads degrees + poles from
  the `B_SPLINE_SURFACE` sub, `(uMults,vMults,uKnots,vKnots)` from the
  `B_SPLINE_SURFACE_WITH_KNOTS` sub via `fillBsplineGrid`, then the `((weights))` grid
  from `RATIONAL_B_SPLINE_SURFACE` into `FaceSurface::weights` (row-major, U outer).
- [x] 2.4 Weight-grid validation: outer length `== nPolesU`, every inner row `== nPolesV`,
  total `== poles.size()`, every weight finite and strictly positive; else `nullopt`
  (decline). No clamping. Proven by `foreign_rational_bspline_surface_malformed_weights_
  decline` (ragged / wrong-count / zero / negative all → NULL).
- [x] 2.5 `weights` flow unmodified: the sim-admitted sphere passes the bare-periodic path
  and the trimmed bump-cap passes the rational-aware `S_face(pcurve(t))=C_edge(t)` guard
  (`bsplineSurfaceValue` → `math::nurbsSurfacePoint`). No edit to the guard, mesher, or
  `shape.h` (git shows only `step_reader.cpp` changed under `src/`).

## 3. HOST ANALYTIC gate (no OCCT linked) — GREEN (46/46 reader cases)

- [x] 3.1/3.3 Closed-form rational patch: `foreign_rational_bspline_sphere_combined_record_
  imports_watertight` authors the EXACT-sphere combined `RATIONAL_B_SPLINE_SURFACE` record
  (9×5 rational-quadratic tensor grid, weights `{1,1/√2,…}` in u and v — the byte-identical
  grid the reader's proven `revolvedProfile` builds) and asserts the meshed solid is
  watertight with `V = 4/3·πR³` within deflection. The `foreign_rational_bspline_surface_
  unit_weights_matches_nonrational` case proves the split-record parse + row-major weight
  read reproduce the SAME solid as the non-rational keyword import (independent oracle).
- [x] 3.2 `foreign_rational_bspline_surface_unfaithful_edge_declines`: the perturbed cap,
  delivered as a unit-weight combined rational record, is REJECTED by the rational-aware
  guard (`decline()` → NULL). Malformed weight grids also decline (§2.4).

## 4. SIM native-vs-OCCT parity gate (booted iOS simulator, OCCT linked) — GREEN (83/0)

- [x] 4.1/4.2 `runRationalBsplineSphere` (G3): authors the combined
  `RATIONAL_B_SPLINE_SURFACE` sphere in OCCT's exact Part-21 dialect (with the product /
  `SHAPE_DEFINITION_REPRESENTATION` chain so OCCT `STEPControl_Reader` finds the root), then
  imports it under BOTH engines. Native `cc_set_engine(1)` admits + meshes watertight to
  `nativeVol=112.925` vs the closed form `113.097` (**0.15%**); OCCT re-reads the SAME file
  (`occtVol=114.145`). Structural parity is exact (1 face, 0 native boundary edges for the
  bare-periodic surface, bboxΔ=1.6e-3). NOTE (honest, measured): OCCT's OWN volume for this
  degenerate-pole bare-periodic rational bspline is **0.93% ABOVE** the analytic sphere, so
  NATIVE is the closer of the two to ground truth — the mass gate is therefore anchored on
  the CLOSED FORM (native 0.15%, strict), not on OCCT; native↔OCCT agree to 1.07%.
- [x] 4.3 Decline case, TWO forms: (a) `runNurbsConvertRationalDecline` (G4) — a GENUINE
  OCCT-authored rational surface (`BRepBuilderAPI_NurbsConvert` of a sphere →
  `STEPControl_Writer` emits real `RATIONAL_B_SPLINE_SURFACE` records) DECLINES natively
  (parsed=0, because its seam boundary is a rational B-spline CURVE, still out of scope) and
  the shipping path falls through to OCCT identically (`nat=oracle=267.965`). (b) malformed
  weights decline host-side (§2.4). No fabricated / leaky rational face is ever emitted.

  MEASURED GAP (honest): OCCT `STEPControl_Writer` does NOT emit a rational B-spline surface
  in the bare-periodic VERTEX_LOOP form the reader admits — it always writes a seamed
  `EDGE_LOOP` whose seam is a rational B-spline CURVE (out of this slice's scope). So a
  genuinely-`STEPControl_Writer`-AUTHORED rational surface that native ADMITS is not
  reachable in this slice; the native-admit parity is proven against (i) the closed form on
  the simulator and (ii) OCCT reading the identical combined-record file. Rational B-spline
  CURVE admission (which would unlock NurbsConvert bodies) is the next slice.

## 5. Zero-regression proof (mandatory) — GREEN

- [x] 5.1 Non-rational keyword path + every analytic arm byte-identical: `surface()`'s
  keyword dispatch is unchanged; the rational arm is reachable ONLY by a combined
  `RATIONAL_B_SPLINE_SURFACE` record (which declined before). `git diff` touches ONLY
  `src/native/exchange/step_reader.cpp`; the refactor is proven behaviour-preserving by
  41/41 existing reader cases + 36/36 host CTest + the sim STEP-import harness.
- [x] 5.2 Host CTest **36/36**; host reader test **46/46** (41 existing + 5 new rational);
  sim STEP-import native-vs-OCCT harness **83 passed / 0 failed** (all prior box/cyl/torus/
  sphere/ellipsoid/bspline-revolution/bumpcap/assembly/multisolid cases unchanged + the 4
  new rational cases). Full `run-sim-suite.sh` re-run after rebuilding the xcframework.
- [x] 5.3 `src/native/**` has **0** OCCT `#include`s (only OCCT-as-oracle doc comments);
  `cc_*` ABI unchanged (no `include/` diff); tessellator (`tessellate/`), writer, and
  `topology/shape.h` untouched (git-verified); no tolerance weakened — the mass oracle for
  the new gate is the STRICT closed form.

## 6. Docs / spec

- [x] 6.1 Tasks recorded with results + the measured OCCT-authoring gap (bare-periodic
  rational surface / rational-curve seam). On landing, sync the `native-exchange` delta into
  the main spec and archive; the honest gap (rational-curve seam) keeps NurbsConvert bodies
  on OCCT until the rational-curve slice.

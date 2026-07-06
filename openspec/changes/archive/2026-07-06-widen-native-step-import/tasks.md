# Tasks — widen-native-step-import (Phase 4 #7 — widen the native STEP reader)

Three independent, honestly-gated breadth tracks on the WORKING reader
(`src/native/exchange/step_reader.{h,cpp}`): **T1** ELLIPSE map + TOROIDAL_SURFACE documented
DECLINE; **T2** multi-solid Compound import; **T3** bspline-face round-trip IF constructible.
Each maps ONLY onto native geometry that genuinely exists; otherwise DECLINE (NULL → OCCT).
Native code stays OCCT-free + host-buildable (`/opt/homebrew/opt/llvm/bin/clang++
-std=c++20`). No `cc_*` ABI change. Default engine stays OCCT. `step_writer.cpp` (default) and
the tessellator are NOT modified. `iges_*` / `step_export` stay unchanged.

## 1. T1a — ELLIPSE curve mapping (`step_reader.cpp`, Pass A `curve()`)

- [x] 1.1 Add an `ELLIPSE('',#axis2placement,semiAxis1,semiAxis2)` arm in `curve()` (mirror of
      the `CIRCLE` arm): resolve the `AXIS2_PLACEMENT_3D` frame; set
      `EdgeCurve::Kind::Ellipse` with `radius = semiAxis1` (major, frame X),
      `minorRadius = semiAxis2` (minor, frame Y). Arity/type-checked (4 args, ref + two
      numbers).
- [x] 1.2 Degenerate guard: either semi-axis ≤ 0 (or non-finite) → `std::nullopt` (DECLINE);
      never a wrong/zero-radius ellipse.
- [x] 1.3 Confirm the existing `EDGE_CURVE` vertex-projection param-range path works for the
      ellipse unchanged (the tessellator's `edge_mesher.h`/`trim.h` `K::Ellipse` arms already
      evaluate `origin + X·major·cos t + Y·minor·sin t`).

## 2. T1b — TOROIDAL_SURFACE stays a documented DECLINE (`step_reader.cpp`, Pass A `surface()`)

- [x] 2.1 Keep `surface()` returning `std::nullopt` for `TOROIDAL_SURFACE`; add a comment
      naming the reason: NO native `FaceSurface::Kind::Torus` (kinds are Plane/Cylinder/Cone/
      Sphere/BSpline/Bezier) AND the tessellator (`surface_eval.h`) must not be modified, so a
      torus face cannot self-verify natively. `math::Torus` is untouched. Falls through → OCCT.
- [x] 2.2 Do NOT add a Torus surface kind, a `surface_eval.h` arm, or a rational-patch fake —
      any of those would modify the tessellator or force a wrong/rational map (prohibited).

## 3. T2 — multi-solid Compound import (`step_reader.cpp`, `build()` + root discovery)

- [x] 3.1 Replace `findSingleManifoldBrep` (which DECLINEs on >1 root) with
      `findManifoldBreps()` → sorted list of ALL root `MANIFOLD_SOLID_BREP` ids (#id order =
      author order).
- [x] 3.2 Add `hasNestedAssembly()`: DECLINE if any of `NEXT_ASSEMBLY_USAGE_OCCURRENCE`,
      `MAPPED_ITEM`, `ITEM_DEFINED_TRANSFORMATION`, `REPRESENTATION_RELATIONSHIP[_WITH_TRANSFORMATION]`,
      `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` appears (a transform tree the flat-compound path
      would mis-place). Nested assemblies stay out of scope → OCCT.
- [x] 3.3 In `build()`: map each root via the existing `mapManifoldBrep`; any member NULL/fail
      → whole import DECLINEs (NULL, never partial). One solid → return the Solid unchanged
      (byte-for-byte prior behaviour). ≥2 → `ShapeBuilder::makeCompound(solids)`.

## 4. T2 — engine self-verify for a Compound (`src/engine/native/native_engine.cpp`)

- [x] 4.1 Add `robustlyWatertightMulti(shape)`: for a `Compound`, explore its `Solid`s and
      require EACH `robustlyWatertight` (any leaky member → false; empty → false); for a
      `Solid`, the existing `robustlyWatertight` (identical prior branch).
- [x] 4.2 `step_import` calls `robustlyWatertightMulti`; on NULL parse OR any failure, fall
      through to OCCT `STEPControl_Reader` (labelled; re-reads the FILE, no native void).
      `iges_*` / `step_export` untouched.

## 5. T3 — B-spline-FACE round-trip IF constructible (task 7.4 from the prior slice)

- [x] 5.1 Investigate: does any EXISTING native construct op (loft / sweep / revolve +
      residual builders under `src/native/construct/**`) emit a WATERTIGHT solid with at least
      one `FaceSurface::Kind::BSpline` face + `EdgeCurve::Kind::BSpline` rim edges that
      `step_can_export_native` accepts (non-rational, all kinds in scope)? Record the finding.
- [x] 5.2 IF constructible (native-built, not fabricated): add a host round-trip in
      `test_native_step_reader.cpp` — build → `step_export_native` → `step_import_native` →
      tessellate → valid + watertight AND the B-spline face reconstructs the same degrees /
      row-major control grid / RLE-expanded knots AND volume within analytic tolerance. Close
      prior task 7.4.
- [x] 5.3 IF NO non-fabricated fixture exists: record the honest skip here + in the spec note.
      Do NOT fabricate a bspline-face solid and do NOT modify `step_writer.cpp` to synthesize
      one. The B_SPLINE mapping stays reviewed; the round-trip stays deferred, honestly.

## 6. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [x] 6.1 `step_import_native` signature unchanged (returns one `topo::Shape`, now possibly a
      `Compound`). Update the doc-comment to mention ellipse edges + multi-solid compounds +
      the still-declined set (torus, nested assembly, rational, non-mm).
- [x] 6.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO
      simulator (clean). Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 7. Gate 1 — host unit / round-trip (no OCCT) `tests/native/test_native_step_reader.cpp`

- [x] 7.1 T1a: an in-scope buffer with an `ELLIPSE('',#plc,a,b)` edge → `EdgeCurve::Kind::Ellipse`
      with `radius==a`, `minorRadius==b`, frame from `#plc`; degenerate ellipse → NULL.
- [x] 7.2 T1b: a buffer with a `TOROIDAL_SURFACE` face → NULL (DECLINE), documented.
- [x] 7.3 T2: a two-root buffer → `Compound` of two `Solid`s (count 2, each valid); a
      nested-assembly buffer (`NEXT_ASSEMBLY_USAGE_OCCURRENCE`) → NULL; the existing
      single-solid box/cylinder buffers STILL return a Solid (no regression).
- [x] 7.4 T3: bspline-face round-trip EXACT IF constructible (task 5.2), else documented skip.
- [x] 7.5 Wire into host CTest; all existing native suites green.

## 8. Gate 2 — sim vs OCCT + foreign STEP `tests/sim/native_step_import_parity.mm`

- [x] 8.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list
      devices booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine
      restored in teardown (suite assertion count unchanged).
- [x] 8.2 T1 foreign: OCCT `STEPControl_Writer` authors a solid with an `ELLIPSE` edge; native
      import vs OCCT `STEPControl_Reader` re-import agree (volume / watertight / valid in tol).
- [x] 8.3 T1 fall-through: OCCT authors a `TOROIDAL_SURFACE` solid; native `cc_step_import`
      DECLINEs → OCCT and matches `cc_set_engine(0)`.
- [x] 8.4 T2 foreign: OCCT authors a 2-solid `TopoDS_Compound`; native import returns a
      compound whose per-solid centroid / volume / bbox / count match the OCCT re-import.

## 9. No-regression + NUMSCI + complexity + docs + validation

- [x] 9.1 No regression: prior import slice (host round-trip + sim `[NIMPORT]` parity), STEP
      export, healing, SSI S1–S4, S5 native-pass, native blends + #6/#7, marching, boolean,
      construct, tessellation, phase3 — all green (host CTest + `run-sim-suite.sh`).
- [x] 9.2 NUMSCI ON build proves no interaction: `bash scripts/build-numsci.sh host`; configure
      `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` + the NUMSCI/NUMPP/SCIPP dirs; build + ctest.
- [x] 9.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`curve` ELLIPSE arm, `findManifoldBreps`, `hasNestedAssembly`, `build`,
      `robustlyWatertightMulti`) all 🟢/acceptable; none pushed to a higher band.
- [x] 9.4 `openspec validate widen-native-step-import --strict` green.
- [x] 9.5 Update `openspec/NATIVE-REWRITE.md` #7 + `docs/STATUS-phase-4.md`: native STEP import
      now covers ELLIPSE edges + multi-solid compounds; torus / nested assemblies / rational /
      AP242 / IGES stay OCCT; T3 bspline-face round-trip closed IF constructible else skipped;
      #8 `drop-occt` stays blocked. Living-spec sync/archive when the gates are green.

## Completion status (honest per-track gate — verified host 29/29 + sim [NIMPORT] 28/28)

- **T2 multi-solid → LANDED (genuine native).** Flat multi-root file imports as a native
  `Compound` of watertight Solids; sim vs OCCT re-import `nativeVol=1064 occtVol=1064
  rel=2.14e-16`, per-solid watertight, count/bbox match. Host `multi_solid_flat_file_imports_as_compound`
  (2 solids, exact vol) + `decline_transformed_assembly_returns_null` green.
- **T3 bspline-face round-trip → LANDED (genuine native, non-fabricated fixture).** The
  existing native `build_prism_profile_spline` op emits a watertight `B_SPLINE_SURFACE`-face
  solid; host + sim round-trip native-export→native-import EXACT (`vol nat=304.38 orig=304.38`,
  watertight, face-count match, `B_SPLINE_SURFACE` present). Prior task 7.4 closed.
- **T1a ELLIPSE curve mapping → PARTIAL (curve kind mapped; solid import still DECLINES → OCCT).**
  The `ELLIPSE` arm in `curve()` maps to the genuine `EdgeCurve::Kind::Ellipse` and the host
  test confirms the edge kind + major/minor + degenerate-decline. But end-to-end there is NO
  watertight NATIVE ellipse-solid import: the foreign OCCT-authored ellipse-cut solid parses
  (`parsed=1`) yet the ellipse-on-quadric pcurve is out of this slice, so it fails the
  watertight self-verify (`watertight=0 nativeVol=0`) and the whole solid FALLS BACK to OCCT
  (`ellipse_cut vol nat=942.478 oracle=942.478` is the OCCT fallback matching the oracle).
  So T1a is claimed ONLY as "the reader now recognises + maps the ELLIPSE curve entity"; it is
  NOT claimed as native ellipse-bearing-solid import.
- **T1b TOROIDAL_SURFACE → NOT LANDED (documented DECLINE → OCCT).** No native
  `FaceSurface::Kind::Torus` and the tessellator must not be modified, so the reader returns
  NULL and the engine falls back (`torus native parsed=0`, `fallback torus vol rel=0.00e+00`).
  This is a documented honest decline, not a native import.

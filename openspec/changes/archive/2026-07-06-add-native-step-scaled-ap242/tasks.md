# Tasks — add-native-step-scaled-ap242 (Phase 4 #7 — scaled/mirrored placements + AP242 tolerance)

Widen the WORKING native STEP reader (`src/native/exchange/step_reader.{h,cpp}`) from RIGID-only
placed assemblies to **uniform-scale + mirror** placed assemblies (T1), and teach it to import
the geometry of an **AP242** file while SKIPPING its PMI / annotation entities (T2). Map ONLY onto
placements/geometry the file carries and the un-modified tessellator renders correctly; otherwise
DECLINE (NULL → OCCT). Native code stays OCCT-free + host-buildable
(`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`). No `cc_*` ABI change. Default engine stays OCCT.
`step_writer.cpp` + the tessellator are NOT modified. `iges_*` / `step_export` stay unchanged.
NEVER invent a placement, a scale, a reflection, or a solid; NEVER import PMI as geometry.

## 1. Confirm the exact OCCT-emitted scaled/mirrored + AP242 entities (grounding, before coding)

- [x] 1.1 Author a 2-box assembly with one component at 2× UNIFORM SCALE and a second variant with
      a MIRRORED (reflected) component (OCCT `STEPControl_Writer` on a `TopoDS_Compound` of
      `BRepBuilderAPI_Transform`/`BRepBuilderAPI_GTransform`-placed boxes, and/or
      `STEPCAFControl_Writer` on an XCAF assembly doc). Diagnose the emitted DATA: does OCCT carry
      the scale/mirror INSIDE the `AXIS2_PLACEMENT_3D` frames of the `ITEM_DEFINED_TRANSFORMATION`
      (§2.1) or as a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` (§2.2)? Record the EXACT keyword +
      arg order for whichever form appears (and the non-uniform `_NON_UNIFORM` form for the
      decline case). Confirm the mirror's handedness encoding (left-handed axis triad vs det<0
      operator).
- [x] 1.2 Author an AP242 file with a single in-slice solid + PMI (a datum + a geometric tolerance
      + an annotation) via `STEPCAFControl_Writer` with PMI, or a hand-authored AP242 header + PMI
      entities. Diagnose the emitted DATA: record the EXACT annotation/PMI keyword set
      (`DRAUGHTING_MODEL`, `ANNOTATION_*`, `*_GEOMETRIC_TOLERANCE`, `DATUM*`, `DIMENSIONAL_*`,
      `GEOMETRIC_ITEM_SPECIFIC_USAGE`, `PRESENTATION_*`, …) and the extra unit contexts
      (`PLANE_ANGLE_UNIT`, `SOLID_ANGLE_UNIT`, PMI contexts) so the skip set (§5) + the unit-gate
      refinement (§4) are grounded against what OCCT actually writes.

## 2. Placement classifier (`step_reader.cpp` + `math`)

- [x] 2.1 `classifyPlacement(const math::Transform&) → optional<Placement{cls,scale}>`: Gram-matrix
      conformality test `MᵀM ≈ k²·I` (scale-relative tol) with a det-sign branch →
      `Rigid`(k≈1,det>0) | `UniformScale`(k>0,det>0) | `Mirror`(det<0); a non-conformal `MᵀM`
      (non-uniform/shear) or degenerate linear part → `nullopt` (DECLINE). Read-only predicate; NO
      change to existing `math`/`Transform` behaviour. `Rigid` reproduces the old `isRigid==true`
      exactly (byte-unchanged rigid path).
- [x] 2.2 Remove/retire the boolean `isRigid`; every composed component placement routes through
      `classifyPlacement`. A `nullopt` DECLINES the whole file (unchanged honesty).

## 3. Mirror orientation compensation (`step_reader.cpp`, topology — existing algebra only)

- [x] 3.1 For a `Mirror`-class component, complement the component solid's face orientation using
      the EXISTING `topo::Orientation` `reversed`/`complemented` algebra (`shape.h`) BEFORE applying
      the mirror `Location`, so the tessellator's tangent-derived normal (`cross(place(∂u),
      place(∂v))`, which FLIPS under det<0 — see `surface_eval.h`) points OUTWARD again and the
      mirrored solid self-verifies watertight with POSITIVE volume. NO tessellator change, NO new
      topology primitive.
- [x] 3.2 Pick the exact mechanism (complement the top Solid orientation vs. reverse the shell's
      faces) against the mirror fixture (task 8.3) so the engine self-verify volume is positive;
      both use only the existing algebra. A mirrored solid that still fails the self-verify after
      compensation → DECLINE → OCCT (never a fabricated flip).

## 4. AP242 unit-context tolerance (`step_reader.cpp`)

- [x] 4.1 Refine `validateUnitContext()` to answer exactly "is the LENGTH unit millimetre?": a
      length `SI_UNIT` (`.METRE.`) MUST be `.MILLI.` (unchanged non-mm decline); a non-length
      `SI_UNIT` (`.RADIAN.`/`.STERADIAN.`/PMI angle contexts) is SKIPPED, not read as non-mm. The
      mm gate is UNCHANGED — no tolerance weakened; only additive PMI unit contexts ignored.

## 5. AP242 PMI / annotation skip policy (`step_reader.cpp`)

- [x] 5.1 Recognise a curated AP242 annotation/PMI keyword set (grounded by 1.2) and SKIP it: such
      entities never force the assembly path and never fail a scan. Document the set as additive.
- [x] 5.2 Scope `hasNestedAssembly()` so it returns true only for a transform relationship that
      reaches a `MANIFOLD_SOLID_BREP` root (a PRODUCT placement), not for a
      `REPRESENTATION_RELATIONSHIP`/`MAPPED_ITEM`/`CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` in the
      annotation/draughting graph. A plain AP242 solid + PMI takes the single-solid/flat path.
- [x] 5.3 In `assembly()`, SKIP an annotation-graph relationship whose child representation reaches
      no geometric root brep (rather than declining), and compute the completeness gate over the
      GEOMETRIC root breps only. A genuinely uncomposable PRODUCT transform still DECLINES.

## 6. Scale/mirror transform sources (`step_reader.cpp`)

- [x] 6.1 `itemDefinedTransform` (§2.1): replace `isRigid` with `classifyPlacement`; the composed
      `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹` naturally carries a frame-encoded scale/mirror;
      classify the result. No new parse for the frame-encoded case.
- [x] 6.2 `cartesianOperator(id) → optional<Transform>` (§2.2, add IFF 1.1 shows OCCT emits it):
      read `CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#axis1,#axis2,#origin,scale[,#axis3])` into an
      affine `Transform`; a `_NON_UNIFORM` form or a `scale1/2/3` triple with unequal factors →
      `nullopt` (DECLINE). Route through `classifyPlacement`. If 1.1 shows OCCT encodes scale/mirror
      only in the frames, add this defensively but the frame path (6.1) carries the fixtures.

## 7. Placed compound in `assembly()` (`step_reader.cpp`)

- [x] 7.1 Remember each placed brep's `PlacementClass`; for `Mirror` apply the §3 orientation
      complement, then `solid.located(Location{T})`. `UniformScale`/`Rigid` ride the located node
      directly. The single-solid + flat multi-solid paths in `build()` are UNCHANGED (byte-for-byte)
      for rigid/flat/single files. NEVER default a component to identity; NEVER partial-import.

## 8. Native reader API + OCCT-free build (`step_reader.h` / `native_exchange.h`)

- [x] 8.1 `step_import_native` signature unchanged (returns one `topo::Shape`, now possibly a
      PLACED `Compound` with uniformly-scaled / mirrored members). Update the doc-comment:
      uniform-scale + mirror single-level assemblies import; AP242 files import geometry with PMI
      skipped; non-uniform/shear + PMI semantics + deep-nested + out-of-slice still decline.
- [x] 8.2 Confirm `src/native/exchange/` still compiles with
      `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20 -Wall -Wextra -Wpedantic`, NO OCCT, NO
      simulator. Grep-gate: zero OCCT includes/symbols in `src/native/**`.

## 9. Engine hook + OCCT fallback (`src/engine/native/native_engine.cpp`) — logic unchanged

- [x] 9.1 Confirm `step_import` still calls `step_import_native` then `robustlyWatertightImport`
      (per-member, volume>0). A uniform-scale member self-verifies with volume `k³·V₀>0`; a
      mirror-compensated member self-verifies with the correct positive volume; a wrongly-mirrored
      (uncompensated / non-watertight) member fails → OCCT. No new engine gate. `iges_*` /
      `step_export` untouched.

## 10. Gate 1 — host unit / decline `tests/native/test_native_step_reader.cpp`

- [x] 10.1 Classifier: uniform-scale (`k=2`) → `UniformScale(2)`; reflection (det<0) → `Mirror`;
      rigid → `Rigid(1)`; non-uniform (`diag(2,1,1)`) + shear → DECLINE (nullopt).
- [x] 10.2 Scaled placed compound: a two-component transform-tree buffer with a 2× uniform-scale
      component → `Compound` of two `Solid`s; the scaled solid's volume is `8×` the unit + centroid
      at the scaled world placement; both valid + watertight.
- [x] 10.3 Mirrored placed compound: a reflected-component buffer → `Compound` whose mirrored solid
      is valid + watertight with POSITIVE volume + reflected centroid (proves the §3 orientation
      compensation).
- [x] 10.4 AP242 tolerance: a buffer whose `FILE_SCHEMA` is AP242 with a solid + PMI/annotation
      entities + PLANE_ANGLE/PMI unit contexts imports the SOLID + drops the PMI (same solid as the
      AP203 equivalent).
- [x] 10.5 Non-uniform decline: a non-uniform-scale / shear component placement DECLINES (NULL).
      Split `decline_non_rigid_assembly_returns_null`: uniform-scale + mirror now IMPORT,
      non-uniform/shear RETAINS the decline.
- [x] 10.6 No regression: the RIGID assembly, FLAT multi-solid, single-solid, quadric, and
      bspline-face round-trip cases STILL pass. Wire into host CTest; all existing native suites
      green.

## 11. Gate 2 — sim vs OCCT + foreign fixtures `tests/sim/native_step_import_parity.mm`

- [x] 11.1 Extend the harness + `scripts/run-sim-native-step-import.sh`; `xcrun simctl list devices
      booted` first; own `main()`, on the `run-sim-suite.sh` SKIP list; default engine restored in
      teardown (suite assertion count unchanged).
- [x] 11.2 (A) OCCT authors a 2-component assembly with one component at 2× uniform scale; native
      `cc_step_import` (engine 1) → placed `Compound`; OCCT `STEPControl_Reader` re-imports; assert
      same solid COUNT, same TOTAL volume (scaled component contributing `k³·V₀`, rel tol),
      per-solid bbox + centroid/placement within tol.
- [x] 11.3 (B) OCCT authors a reflected (mirrored) component; native import → placed `Compound`;
      assert same count / total volume / mirrored per-solid bbox + centroid AND the native mirrored
      member is watertight with POSITIVE volume (correct outward orientation).
- [x] 11.4 (C) OCCT authors an AP242 file with a solid + PMI annotations; native `cc_step_import`
      imports the SOLID; OCCT `STEPControl_Reader` re-imports; assert identical solid (count /
      volume / bbox) with PMI ignored on both sides.
- [x] 11.5 (D) OCCT authors an assembly with a NON-uniform-scaled / sheared component; native
      `cc_step_import` DECLINES → OCCT and matches `cc_set_engine(0)`.

## 12. No-regression + NUMSCI + complexity + docs + validation

- [x] 12.1 No regression: prior import slices (flat multi-solid + bspline-face + RIGID assemblies,
      sim `[NIMPORT]` 33/33), STEP export, healing, SSI S1–S5, native blends + #6/#7, marching,
      boolean, construct, tessellation, phase3 — all green (host CTest + `run-sim-suite.sh`).
- [x] 12.2 NUMSCI ON build proves no interaction: `bash scripts/build-numsci.sh host`; configure
      `build-ns` with `-DCYBERCAD_HAS_NUMSCI=ON` + the NUMSCI/NUMPP/SCIPP dirs
      (`-DCYBERCAD_NUMSCI_DIR=.../build-numsci/host`); build + ctest.
- [x] 12.3 Cognitive complexity (`cognitive-complexity` skill) of the touched functions
      (`classifyPlacement`, `cartesianOperator`, `validateUnitContext`, `hasNestedAssembly`,
      `assembly`, the mirror-complement helper) all acceptable for the parser/systems band; none
      pushed to a higher band.
- [x] 12.4 `openspec validate add-native-step-scaled-ap242 --strict` green.
- [x] 12.5 Update `openspec/NATIVE-REWRITE.md` #7 + `docs/STATUS-phase-4.md`: native STEP import
      now covers uniform-scale + mirror single-level PLACED assemblies + AP242 geometry import with
      PMI skipped; non-uniform/shear / deep-nested / out-of-slice transforms + PMI semantics stay
      OCCT; #8 `drop-occt` stays blocked. Living-spec sync/archive when the gates are green.

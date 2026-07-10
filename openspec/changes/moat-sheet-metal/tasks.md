# Tasks — moat-sheet-metal (MOAT M-SM first slice, one PR)

Order: native module → facade + engine wiring → host test → sim test + runner → docs.
All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::sheetmetal`. No `cc_*` ABI change (additive entry points only).

## 1. Native sheet-metal module (`src/native/sheetmetal/`, OCCT-free, header-only)

- [x] 1.1 `common.h`: the `SheetMetalDecline` measured-reason enum, tolerances
      (`kTol`/`kMinThick`/`kAngleFloor`/`kPi`), `signedArea`, the `FoldRecord` POD, and
      the ONE `verifySolid(shape, expectedVol, defl)` gate — watertight +
      consistently-oriented + χ=2 + positive volume within a deflection-set closed-form
      band (reuses `tess::isWatertight`/`isConsistentlyOriented`/`enclosedVolume` +
      `dm::rfdetail::eulerChar`, the same vocabulary draft_faces uses).
- [x] 1.2 `base_flange.h`: `baseFlange(profileXY, count, thickness, why)` — extrude the
      closed profile by `thickness` via the landed `construct::build_prism`; self-verify
      at `|profileArea|·thickness`; decline a degenerate profile / thickness ≤ 0.
- [x] 1.3 `edge_flange.h`: recognise the L×W×t base plate (`readBasePlate`) and the
      straight +X/+Z rim (`isFlangeableRim`); build base + true partial-cylinder bend
      (fan of strips sized from the deflection) + planar flange wall as ONE watertight
      solid, all faces auto-oriented; self-verify at `base + ½θ((r+t)²−r²)·W + h·t·W`;
      emit a `FoldRecord`; decline bad param / non-straight bend / non-recognised base /
      self-collision (measured).
- [x] 1.4 `unfold.h`: `flatPattern` closed form (BA = θ(r + k·t), devLength =
      baseRun + BA + h, area = devLength·W); `unfold(...)` emits the developed rectangle
      blank via `build_prism`, self-verify at `area·thickness`; a `FoldRecord` overload
      for the engine path; decline an invalid record / kFactor outside [0,1].
- [x] 1.5 `sheetmetal.h` umbrella header including the three ops.

## 2. Facade + engine wiring (ADDITIVE only)

- [x] 2.1 `cc_kernel.h`: declare `cc_sheet_base_flange` / `cc_sheet_edge_flange` /
      `cc_sheet_unfold` with the honest engine-note (native-only, no OCCT oracle).
- [x] 2.2 `cc_kernel.cpp`: three `cyber::guard` facade wrappers.
- [x] 2.3 `IEngine.h`: three virtuals defaulting to `engine_unsupported`.
- [x] 2.4 `native_engine.{h,cpp}`: three overrides; `FoldRecord` field on `NativeShape` +
      `wrapNativeFold`; a `smDeclineText` helper; every op self-verifies (the module) and
      the engine re-audits `watertightVolume > 0`; a non-native / mesh body or an
      unbuildable case returns a clean `make_error` — NEVER an OCCT forward.

## 3. Host test — Gate (a), closed-form (no OCCT)

- [x] 3.1 `tests/native/test_native_sheetmetal.cpp` (CTest `test_native_sheetmetal`):
      base flange volume; L-profile base; edge flange 90° closed-form volume watertight /
      χ=2 / oriented; r=0 / r=1 / 120° variants build; the honest declines; unfold BA +
      area; fold→unfold area invariant; unfold declines.
- [x] 3.2 Register in `CMakeLists.txt` (`CYBERCAD_TESTS` + `_SRC` path).

## 4. Sim test — Gate (b), native under its own engine (NO OCCT compared)

- [x] 4.1 `tests/sim/native_sheetmetal_selftest.mm` (own `main()`): under
      `cc_set_engine(1)`, base/edge/unfold pass `cc_check_solid` + `cc_mass_properties`
      volume = closed form; determinism; honest decline. NO OCCT oracle (stated).
- [x] 4.2 `scripts/run-sim-native-sheetmetal.sh` runner (OCCT-free harness; links the
      OCCT adapter TUs only to satisfy the facade's create_default_engine).
- [x] 4.3 SKIP entry in `scripts/run-sim-suite.sh`.

## 5. Docs + validation

- [x] 5.1 `openspec/MOAT-ROADMAP.md` — new `M-SM` stage (first slice), sharpened next
      blocker = multi-bend / miter / corner-relief.
- [x] 5.2 `openspec validate moat-sheet-metal --strict` green.
- [x] 5.3 Structural discipline: `git diff src/native` OCCT-free + additive; only the new
      `sheetmetal/` module + engine/facade wiring; other modules untouched; `cc_*`
      additions only.

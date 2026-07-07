# Tasks — moat-m7a-construction-breadth (MOAT M7a, first slice)

Landed slice: expose the already-proven OCCT-free N-section ruled loft
(`build_loft_sections`) through a new ADDITIVE facade entry `cc_solid_loft_sections`,
backed by a real OCCT `ThruSections` oracle, engine self-verify → OCCT fallback, and a sim
parity gate. The `cc_*` change is ADDITIVE-ONLY (a new function; no existing
signature/POD/enum changes). OCCT stays confined to `src/engine/occt`; `src/native/**`
keeps 0 OCCT includes and no native source is changed. The tessellator is untouched. A
correct decline (out-of-slice input → OCCT) is a first-class outcome.

DEFERRED to a later M7 slice (honest decline, recorded in `MOAT-ROADMAP.md` M7): the
orientation-constraining guided sweep (`cc_guided_orient_sweep`) — it needs a NEW OCCT
`MakePipeShell(guideWire)` oracle + a guide-aimed native frame not landed here.

## 0. Substrate + baseline

- [x] 0.1 Build the numsci substrate and record the GREEN baseline:
      `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`, then
      `export CYBERCAD_NUMSCI_DIR=…/build-numsci/{iossim|host}`. (host OK=77 FAIL=0.)
- [x] 0.2 Capture the pre-change baseline: host construction suite + the loft/sweep parity
      sim gates GREEN, so the additive change is PROVEN not to perturb them.

## 1. Additive facade entry (`cc_solid_loft_sections`)

- [x] 1.1 Declare `cc_solid_loft_sections(sectionsXYZ, counts, sectionCount)` in
      `include/cybercadkernel/cc_kernel.h` with a doc comment stating the ≥3-section ruled
      semantics, mismatched-count resample, and degenerate-input return; leave
      `cc_solid_loft` / `cc_solid_loft_wires` untouched.
- [x] 1.2 Wire the C ABI shim in `src/facade/cc_kernel.cpp` to the active
      `IEngine::solid_loft_sections`, mirroring `cc_solid_loft_wires`. Additive-only
      (no existing symbol changes; `cc_kernel.h` has no deletions).

## 2. Real OCCT oracle (`src/engine/occt`)

- [x] 2.1 Add `OcctEngine::solid_loft_sections` using `BRepOffsetAPI_ThruSections`
      (solid + ruled), one `BRepBuilderAPI_MakePolygon` wire per section, all added in
      order; validate + `addIfValid`, error on degenerate input (< 2 sections, a section
      < 3 points).
- [x] 2.2 Add the `IEngine::solid_loft_sections` default (returns `engine_unsupported`)
      and the `OcctEngine` / `NativeEngine` overrides; confirm OCCT stays confined to
      `src/engine/occt` (no OCCT include leaks into `src/native`).

## 3. Native builder (`src/native/construct/loft.h`, OCCT-free — pre-existing)

- [x] 3.1 Reuse the already-landed `ncst::build_loft_sections` (equal- and
      mismatched-count planar N-section ruled loft with T1 arc-length resample); no native
      source change. `src/native/**` stays at 0 OCCT includes (grep-verified).

## 4. Engine dispatch + mandatory self-verify

- [x] 4.1 `NativeEngine::solid_loft_sections`: call `build_loft_sections`; if the result
      is null, not `robustlyWatertight`, or has non-positive `watertightVolume`, DISCARD →
      `fallback().solid_loft_sections(same args)` (OCCT). Never keep an unverified body.
- [x] 4.2 Confirm every out-of-slice input (non-planar / point-collapsed section,
      mismatched caps that can't close, asymmetric expand-then-contract spool whose mesh
      T-junctions the shared ring) forwards the SAME arguments to OCCT.

## 5. Host analytic gate (Gate 1 — no OCCT linked)

- [x] 5.1 `tests/native/test_native_loft.cpp` (pre-existing, unchanged) asserts, with
      `CYBERCAD_USE_OCCT=OFF`, a watertight `Solid` and the closed-form enclosed volume
      (prismatoid per band / `A·H` for a stack) for stacked-box, square/octagon spools,
      mismatched 4→8 frustums. **Re-run independently: 21/21, 0 failed.**
- [x] 5.2 Decline coverage: non-planar-middle, degenerate, and single-section inputs each
      yield a NULL native Shape (host).

## 6. Sim parity gate (Gate 2 — OCCT oracle) — OCCT-linked simulator

- [x] 6.1 `tests/sim/native_loft_parity.mm`: on a booted iOS simulator,
      `cc_solid_loft_sections` with the native engine active vs
      `OcctEngine::solid_loft_sections` (`ThruSections`, ruled) — native fixtures (square
      spool 10→4→10 vol 624, triangle stack 30, stacked box 144, octagon spool 18 faces)
      match volume/area/watertight/topology to fp precision.
- [x] 6.2 Deferred fixtures (asymmetric spool, non-planar middle, mismatched-4to3) must
      delegate to OCCT (`native active=1`, delegated volume equal to the oracle to fp
      precision); the measured gap is RECORDED — never shipped mismatched. **39 passed, 0
      failed.**

## 7. Zero-regression proof

- [x] 7.1 `cc_solid_loft` / `cc_solid_loft_wires` / `cc_solid_sweep` and every other
      shipped op remain byte-identical to the baseline (host suite + loft/sweep parity
      gates GREEN); the change reaches new code only via the new entry. `git diff main --
      cc_kernel.h` has NO deletions.
- [x] 7.2 Tessellator unmodified (no `tessellat|mesh|triangul` file in the diff).

## 8. Docs

- [x] 8.1 Update the `native-construction` delta spec with the `cc_solid_loft_sections`
      requirement (this change) and record the still-deferred guided-sweep /
      non-planar-cap loft / fine-pitch residuals in `MOAT-ROADMAP.md` M7.

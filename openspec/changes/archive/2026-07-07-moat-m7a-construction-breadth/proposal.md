# Proposal — moat-m7a-construction-breadth (MOAT M7a, first slice)

## Why

The native construction family (`src/native/construct`) already lands the tractable
sweep/loft breadth: straight + smooth-curved-planar + rotation-minimizing-frame sweep,
twisted/scale sweep, straight-rail loft, and — in `loft.h` `build_loft_sections`, with a
green host suite (`tests/native/test_native_loft.cpp`) — the equal- **and**
mismatched-count planar **N-section (≥3) ruled loft**. That N-section builder was,
however, NOT reachable through the `cc_*` facade: only `cc_solid_loft` (single-profile
extrude-loft) and `cc_solid_loft_wires` (2 wires) were wired. A kernel consumer could not
build a 3-or-more-section loft at all.

The remaining Tier-4 breadth is three genuinely harder residuals
(`openspec/MOAT-ROADMAP.md` M7): the **orientation guided/rail sweep**, the **general
non-planar-cap loft**, and the **fine-pitch self-intersecting thread** (needs M2 — OUT OF
SCOPE here).

### Diagnosis — what the first slice actually lands

The most tractable, immediately verifiable increment turned out to be neither of the hard
frame/closure residuals but the **facade wiring of the already-proven N-section loft**:
exposing `build_loft_sections` through a new additive facade entry
`cc_solid_loft_sections`, backed by a real OCCT `BRepOffsetAPI_ThruSections` oracle for
self-verify and fallback. This is a genuine new construction capability at the ABI
(3-or-more-section lofts, equal or mismatched counts) that the underlying native geometry
already computes correctly and that both verification gates confirm, at low risk and with
no tessellator changes.

The **orientation-constraining guided sweep** (`cc_guided_orient_sweep`) — the residual
this change originally scoped — is **DEFERRED / honest decline** for this iteration: it
requires a NEW OCCT orientation oracle (`BRepOffsetAPI_MakePipeShell` + `SetMode(guideWire)`)
and a guide-aimed native frame producer that were not landed here. The ABI-additive path
to it remains open and is recorded in `MOAT-ROADMAP.md` M7 as the next slice; nothing about
this change forecloses it. A correct decline is a first-class outcome.

## What Changes

1. **A new ADDITIVE facade function `cc_solid_loft_sections`** in
   `include/cybercadkernel/cc_kernel.h` + the C ABI:
   `cc_solid_loft_sections(sectionsXYZ, counts, sectionCount)` — the ≥3-section
   generalisation of `cc_solid_loft_wires`. `cc_solid_loft` / `cc_solid_loft_wires` and
   every other shipped op are UNCHANGED. Purely additive — no existing signature, POD
   layout, or enum value changes; `cc_kernel.h` has NO deletions.

2. **A real OCCT oracle** `OcctEngine::solid_loft_sections` in `src/engine/occt`
   (`BRepOffsetAPI_ThruSections`, solid + ruled, one wire per section), the oracle the
   native N-section builder self-verifies against and the target the engine forwards to on
   a native decline. OCCT stays confined to `src/engine/occt`.

3. **Engine dispatch + self-verify** in `NativeEngine::solid_loft_sections`: it calls the
   pre-existing OCCT-free `ncst::build_loft_sections`, runs the MANDATORY self-verify
   (`robustlyWatertight` + positive `watertightVolume`), and DISCARDS → OCCT on any
   failure. The `IEngine` base gets a default `solid_loft_sections` returning
   `engine_unsupported`. `src/native/**` keeps 0 OCCT includes; no native source changed.

4. **Two verification gates** (the standing two-gate model): a HOST analytic gate
   (`tests/native/test_native_loft.cpp`, native closed-form volume with NO OCCT linked)
   and a SIM native-vs-OCCT parity gate (`tests/sim/native_loft_parity.mm`, the N-section
   cases through `cc_solid_loft_sections` vs `BRepOffsetAPI_ThruSections`, including the
   honest-decline fixtures that must delegate to OCCT and return the oracle solid).

5. **Docs**: this delta updates the `native-construction` spec with the
   `cc_solid_loft_sections` requirement; `MOAT-ROADMAP.md` M7 records the landed slice and
   the still-deferred guided-sweep / non-planar-cap loft / fine-pitch residuals.

## Impact

- Affected specs: `native-construction` (this delta — one ADDED requirement).
- Affected code (all additive; no behavior change to any existing op):
  `include/cybercadkernel/cc_kernel.h` (+ C ABI wiring), `src/facade/cc_kernel.cpp`,
  `src/engine/IEngine.h` (default), `src/engine/native/*` (dispatch + self-verify),
  `src/engine/occt/*` (new oracle), `tests/sim/native_loft_parity.mm` (N-section + decline
  fixtures). No `src/native/**` and no tessellator source changed.
- OCCT remains the oracle and the fallback; it is never removed. Every shipped op stays
  byte-identical.

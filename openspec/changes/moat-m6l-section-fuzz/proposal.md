# Proposal — moat-m6l-section-fuzz (MOAT M6-breadth-12, the TWELFTH domain)

## Why

The MOAT completeness bar (dropping OCCT is gated by *proven* native correctness) has
eleven landed differential-fuzzing domains — curved booleans (`native_boolean_fuzz.mm`),
STEP round-trip (`native_step_import_fuzz.mm`), construction/loft/sweep
(`native_construct_fuzz.mm`), blends (`native_blend_fuzz.mm`), wrap/emboss
(`native_wrap_emboss_fuzz.mm`), mesh mass-properties (`native_mass_props_fuzz.mm`),
geometry services (`native_geometry_services_fuzz.mm`), rigid/similarity transforms
(`native_transform_fuzz.mm`), reference/datum geometry
(`native_reference_geometry_fuzz.mm`), direct-modeling (`native_directmodel_fuzz.mm`), and
transformed-boolean (`native_transformed_boolean_fuzz.mm`).

This change adds the **TWELFTH** native domain — a **section-curve** differential fuzzer of
the native planar SECTION-CURVE service (`cybercad::native::section::sectionByPlane`,
`src/native/section/section.h`) the app's drawing / section-view path reads through the
additive `cc_section_plane` facade. It is the highest-value remaining pick, for three
reasons:

- **It is a DISTINCT, un-fuzzed domain with a PRISTINE closed-form arbiter.** There is a
  `native_section_parity.mm`, but it is a small FIXED-fixture gate (a handful of hand-chosen
  box / cylinder / sphere cuts). This is the FIRST SEEDED, N≥60/seed DIFFERENTIAL FUZZER of
  the section service: random primitive dimensions AND random cut planes (axis-aligned AND
  OBLIQUE) per family. The section of an elementary solid is an EXACT elementary conic —
  rectangle, circle, or ellipse — whose perimeter and enclosed area have a closed form in
  plain fp64. Because the native loop's `length()` / `area()` ARE those closed forms
  (Circle/Ellipse analytic fields; Polygon shoelace), native-vs-analytic is EXACT to
  machine epsilon, making the analytic conic a large-margin PRIMARY correctness oracle that
  no meshing-tolerance fuzzer enjoys.

- **The section service is STABLE and NOT touched by the concurrent M3 workflow.** M3 is
  actively modifying `src/native/{blend,feature,boolean}`. The section service lives in a
  SEPARATE directory, `src/native/section/`, and reads `src/native/{math,topology,ssi}`
  READ-ONLY — none of which M3 is changing. The domain is therefore stable and
  non-overlapping (fillet / shell / offset / wrap-emboss are all AVOIDED here).

- **It is OCCT-FREE and needs NO numsci substrate.** Like `native_section_parity`, the
  native section service is the OCCT-free SSI Stage-S1 header path + header-only topology,
  so the harness compiles only the OCCT-free native math TUs (bezier/bspline) alongside the
  header-only section / topology / ssi-S1 headers, and links ONLY the OCCT oracle toolkits
  (`BRepPrimAPI` + `BRepAlgoAPI_Section` + `ShapeAnalysis_FreeBounds` + `BRepGProp`). The
  system under test is exercised at the `cybercad::native::section` C++ boundary.

## What Changes

1. **A new section-curve differential fuzzer** `tests/sim/native_section_fuzz.mm`
   (iOS-simulator, OCCT-oracle slice, no numsci), reusing the landed harness machinery
   (splitmix64/xoshiro256** RNG, coverage tally, five-way verdict). Per trial it:
   - **deterministically generates** a random-but-VALID primitive AND cut plane across the
     section AGREE families — `BOX` (axis-aligned interior cut → rectangle), `CYL_PERP`
     (perpendicular cut → circle), `CYL_AXIAL` (plane through the axis → rectangle),
     `CYL_OBL` (OBLIQUE cut fully inside the axial band → ellipse), `SPHERE` (interior cut
     → circle) — plus a `DECLINE` exerciser (plane clearly MISSING the solid / plane
     COINCIDENT with a planar face), via an RNG keyed ONLY by an explicit `FUZZ_SEED`
     (argv/env, fixed default) — NO clock, NO `rand()`; same seed → byte-identical batch.
     The first `F_COUNT` trials force each family for coverage.
   - **sections the native solid** at the `cybercad::native::section` C++ boundary and the
     matched OCCT solid (built independently with `BRepPrimAPI` to the same dimensions) with
     `BRepAlgoAPI_Section` — loop count via `ShapeAnalysis_FreeBounds` wire recovery, edge
     length via `GCPnts_AbscissaPoint::Length` (the adaptive arc-length integrator
     `native_section_parity` proved converges to the true perimeter), capped area via
     `BRepGProp::SurfaceProperties`.
   - **arbitrates** against a THIRD engine-independent CLOSED-FORM conic (rectangle /
     circle / ellipse perimeter + area) as PRIMARY, with native-vs-OCCT as SECONDARY, and
     **classifies** each trial into EXACTLY ONE of AGREED / HONESTLY-DECLINED (native
     Empty/Declined → OCCT ships) / DISAGREED (a real finding) / ORACLE_UNRELIABLE (native
     matches exact math while OCCT does not — native vindicated, gated off). Prints a
     per-family coverage summary; exits 0 IFF the bar holds. Any DISAGREED /
     ORACLE_UNRELIABLE prints seed + case index + full descriptor.
2. **A runner** `scripts/run-sim-native-section-fuzz.sh` mirroring
   `run-sim-native-section.sh` (OCCT-free native math TUs + section/topology/ssi-S1
   header-only + the OCCT oracle toolkits; NO numsci). It runs ≥2 seeds by default and
   fails if any seed fails; the new `.mm` is added to the `run-sim-suite.sh` SKIP list.
3. **Nothing in `src/native/**` or `src/engine/**` changes.** The native section service +
   the `cc_section_plane` facade path is the SYSTEM UNDER TEST and stays byte-unchanged; the
   `cc_*` ABI is unchanged. If the fuzzer surfaces a real native disagreement it is reported
   with its seed — not silenced.

## Capabilities

### Modified Capabilities

- `native-verification`: ADDS the twelfth differential-fuzzing domain — a section-curve
  harness that cuts random primitives (box / cylinder / sphere) with random axis-aligned
  AND OBLIQUE planes and verifies the native planar section service against a THIRD
  engine-independent CLOSED-FORM conic (rectangle / circle / ellipse perimeter + area,
  EXACT for the elementary targets) plus native-vs-OCCT `BRepAlgoAPI_Section` agreement at
  a deflection-matched tolerance, with the native service's coincident / tangent / missing
  declines as first-class honest scope.

## Impact

- `tests/sim/native_section_fuzz.mm` — NEW test harness (infrastructure).
- `scripts/run-sim-native-section-fuzz.sh` — NEW runner.
- `scripts/run-sim-suite.sh` — the new `.mm` added to the SKIP list (one line).
- `openspec/MOAT-ROADMAP.md` — M6 breadth row updated (×11 → ×12).
- **Zero production-code change.** `src/native/**`, `src/engine/**`, `include/**`, and the
  `cc_*` ABI stay BYTE-UNCHANGED. No tolerance is weakened; no result is silently capped or
  dropped. Straight / circular sections are held TIGHT (native-vs-analytic 1e-9); the only
  approximated quantity — the ellipse perimeter (Ramanujan-II + OCCT arc-length integrator)
  — reuses `native_section_parity`'s proven 1e-4 bound and is NEVER widened.
- **Verdict:** across ≥2 seeds at N=96 (>60), DISAGREED = 0 and ORACLE_UNRELIABLE = 0, with
  full family coverage (BOX / CYL_PERP / CYL_AXIAL / CYL_OBL / SPHERE each ≥1 AGREED; the
  decline exerciser ≥1 HONESTLY-DECLINED). Every AGREE trial matches the closed-form conic
  AND OCCT to machine epsilon (dLen = 0 for straight/circular; ellipse dLen ≤ ~1e-15).
- **Out of scope / declined (documented, not faked):** an OBLIQUE cut of a BOX (a triangle /
  pentagon polygon that is still exact but whose closed form is not a simple rectangle) is
  deferred as future breadth — the box AGREE family is a clean axis-aligned rectangle. An
  EXACT-tangency plane grazing a sphere pole (`d == R`) is deliberately NOT forced: it is a
  measure-zero knife-edge where floating-point rounding lands the plane a sub-nanometre
  INSIDE the sphere and native then CORRECTLY returns the true sub-micron circle (verified:
  at `d == R` exactly it declines; at `d = R − 1e-9` it returns the mathematically-correct
  tiny loop while OCCT rounds that to empty). Asserting a decline on that knife-edge would
  flag a CORRECT native section as wrong — the opposite of the bar's intent — so the decline
  exerciser uses only UNAMBIGUOUS declines (plane clearly missing / coincident with a face).
  Freeform / torus faces and arc-trimmed curved-face conics are honestly DECLINED by the
  native service and route to HONESTLY-DECLINED, NEVER a DISAGREE.

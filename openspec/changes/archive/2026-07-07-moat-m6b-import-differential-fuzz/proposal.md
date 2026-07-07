# Proposal — moat-m6b-import-differential-fuzz (MOAT M6b, the completeness bar's SECOND domain)

## Why

MOAT M6 landed the FIRST differential-fuzzing harness — a seeded batch that drives
random-but-valid curved-boolean operands through BOTH the native path and the OCCT
oracle and asserts ZERO SILENT WRONG RESULTS (`tests/sim/native_boolean_fuzz.mm`,
capability `native-verification`). One fuzzed capability is not a completeness bar:
drop-occt needs the discipline proven across MORE THAN ONE native domain, so a silent
wrong result in any migrated capability is caught by a seeded batch, not only by the
handful of hand-picked fixtures the curated parity harnesses carry.

The native ISO-10303-21 (Part-21) **STEP reader** (`src/native/exchange/step_reader`)
is the natural second domain. It already has a curated round-trip parity harness
(`tests/sim/native_step_import_parity.mm`) that proves a *handful* of hand-picked
solids (box, cylinder, holed plate, a few foreign fixtures) re-import correctly. That
is exactly the M6-shaped gap: a curated harness proves a few fixtures; a fuzzer proves
a *seeded batch*. This change turns the STEP round-trip into a differential fuzzer so
the reader is held to the same zero-silent-wrong bar as the curved boolean.

This is INFRASTRUCTURE — a test harness, not a new geometry capability. `src/native/**`
is untouched and stays OCCT-free; the `cc_*` ABI is unchanged. The harness is additive
test/sim code only.

## What Changes

1. **A new differential-fuzzing harness** `tests/sim/native_step_import_fuzz.mm`
   (own `main()`, seed-driven, OCCT oracle) that, for a seeded batch of N random-but-
   valid native solids:
   - DETERMINISTICALLY generates a solid from the families the native STEP **writer**
     serialises AND whose native-write → OCCT-read round-trip is a CLEAN oracle: box /
     n-gon prism (planar), cylinder (revolved rectangle), frustum (revolved trapezoid),
     holed box (circular through-hole → CIRCLE edges + CYLINDRICAL wall). Built through
     the SAME native construct entry points the `cc_solid_extrude` / `cc_solid_revolve`
     / `cc_solid_extrude_holes` facade uses. The RNG is splitmix64-seeded xoshiro256**
     keyed ONLY by an explicit seed (argv/env) — no clock, no `rand()`.
   - EXPORTS each solid to ONE on-disk STEP file via the native writer, then IMPORTS
     THAT SAME FILE two ways: the native OCCT-free Part-21 reader (`readStepFile`) and
     the OCCT `STEPControl_Reader` oracle (measured exactly by `BRepGProp`).
   - CLASSIFIES each trial: **AGREED** (native watertight + vol/area/solid-count match
     OCCT within a FIXED relTol) / **HONESTLY-DECLINED** (native NULL / non-watertight
     → OCCT fallback, oracle valid) / **DISAGREED** (native watertight but wrong vs the
     ground truth — a silent wrong import). A closed-form **analytic arbiter** (every
     family has an exact volume/area) attributes any native-vs-OCCT disagreement so a
     native result vindicated by exact math is logged as **ORACLE-INACCURATE**, not a
     false native fault (see design.md — this is a strengthening, not a weakening).
   - Prints a per-family coverage summary; exits 0 IFF `DISAGREED == 0` and no
     unreliable-oracle. Any DISAGREE prints the seed + case index + param tuple as a
     reproducible regression find.

2. **A run script** `scripts/run-sim-native-step-import-fuzz.sh` — compiles the harness
   for the iOS simulator against the native exchange + math TUs (OCCT-free, **no
   numsci**) and the OCCT oracle toolkits, then runs it in a booted simulator. Seed + N
   are argv/env overridable with fixed deterministic defaults.

3. **The new `.mm` is added to `scripts/run-sim-suite.sh`'s SKIP list** (own `main()`,
   OCCT-oracle slice), matching the sibling `native_boolean_fuzz.mm`.

4. **Delta to the `native-verification` capability spec** adding requirements for the
   STEP-round-trip fuzz domain (generator, dual import, analytic-arbitrated classifier,
   coverage bar with logged honest exclusions).

## Honest scope (logged, not silently dropped)

Two writer-producible families are DELIBERATELY EXCLUDED because their native-write →
OCCT-read round-trip is not a clean oracle (verified empirically, seed 0x5744EE9911):

- **Sphere** (bare-periodic `SPHERICAL_SURFACE` bounded by a `VERTEX_LOOP`): OCCT's
  `STEPControl_Reader` re-imports the native VERTEX_LOOP sphere INCONSISTENTLY
  (sometimes spurious sub-solids / near-zero or negative area), so OCCT is not a
  trustworthy oracle here. Sphere STEP import is still covered by the curated
  foreign-authored round-trip in `native_step_import_parity` (`runRevolvedSphere`).
- **Ruled loft** (bilinear `B_SPLINE_SURFACE` side faces): the native writer HONESTLY
  DECLINES to serialise it (`canSerialize` = false) — a writer-scope limit, not a
  reader gap.

These are a first-class honest DECLINE at the DOMAIN level, recorded here and in the
spec delta. The **frustum** family exercises the honest-decline branch for real: the
native reader reconstructs a revolved cone watertight only occasionally and otherwise
falls back to OCCT (logged), so all of AGREE / DECLINE / (arbitrated) disagreement are
covered.

## Impact

- Affected specs: `native-verification` (ADDED requirements only).
- Affected code: additive test/sim only — `tests/sim/native_step_import_fuzz.mm`,
  `scripts/run-sim-native-step-import-fuzz.sh`, one line in `scripts/run-sim-suite.sh`.
- `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.
- Proven: `DISAGREED == 0` across seeds 0x5744EE9911, 0xB16B00B5, 0x1234 (N=96 and
  N=128) with real per-family coverage; byte-identical determinism verified across two
  runs of the same seed. Seed 0x1234 index=10 surfaced a genuine oracle-side inaccuracy
  (OCCT re-imports a shallow native cone ~2.7% too large while the native reader matches
  the analytic frustum to 9e-4) — correctly attributed ORACLE-INACCURATE, native
  vindicated by exact math, bar NOT failed.

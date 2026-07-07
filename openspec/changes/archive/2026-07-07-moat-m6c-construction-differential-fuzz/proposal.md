# Proposal — moat-m6c-construction-differential-fuzz (MOAT M6c, the completeness bar's THIRD domain)

## Why

MOAT M6 landed the FIRST differential-fuzzing harness (curved boolean,
`tests/sim/native_boolean_fuzz.mm`) and M6b the SECOND (STEP round-trip import,
`tests/sim/native_step_import_fuzz.mm`), each a seeded batch that drives random-but-valid
inputs through BOTH the native path and the OCCT oracle and asserts ZERO SILENT WRONG
RESULTS (capability `native-verification`). Two fuzzed capabilities are not yet a
completeness bar: drop-occt needs the discipline proven across a THIRD independent native
domain, so a silent wrong result in any migrated capability is caught by a seeded batch,
not only by the handful of hand-picked fixtures the curated parity harnesses carry.

The native swept-solid **construction** library (`src/native/construct` — `loft.h`,
`sweep.h`) is the natural third domain. It already has curated parity harnesses
(`tests/sim/native_loft_parity.mm`, `native_sweep_parity.mm`) that prove a *handful* of
hand-picked lofts and sweeps match OCCT. That is exactly the M6-shaped gap: a curated
harness proves a few fixtures; a fuzzer proves a *seeded batch*. This change turns
construction into a differential fuzzer so the native loft/sweep builders are held to the
same zero-silent-wrong bar as the curved boolean and the STEP reader.

This is INFRASTRUCTURE — a test harness, not a new geometry capability. `src/native/**`
is untouched and stays OCCT-free; the `cc_*` ABI is unchanged. The harness is additive
test/sim code only.

## What Changes

1. **A new differential-fuzzing harness** `tests/sim/native_construct_fuzz.mm`
   (own `main()`, seed-driven, OCCT oracle) that, for a seeded batch of N random-but-
   valid construction inputs:
   - DETERMINISTICALLY generates an input from the families the native path CLAIMS
     (native_construct.h): equal- AND mismatched-count **PLANAR N-section ruled loft**
     (coaxial regular-n-gon frustums, prismatoid stacks, and a T1 collinear-resampled
     mismatched-count loft) and **STRAIGHT-path constant-frame sweep** (prisms), plus two
     SPARSE out-of-scope inputs (a non-planar loft section; a non-planar sweep spine) to
     exercise the native DECLINE branch. The RNG is splitmix64-seeded xoshiro256** keyed
     ONLY by an explicit seed (argv/env) — no clock, no `rand()`.
   - Builds each input BOTH ways: DIRECTLY via the OCCT-free native builders
     (`ncst::build_loft_sections` / `ncst::build_sweep`, measured by the native
     tessellator) AND via the OCCT oracle (`BRepOffsetAPI_ThruSections` ruled solid /
     `BRepOffsetAPI_MakePipe` — the SAME construction the `cc_*` OCCT engine uses,
     measured exactly by `BRepGProp`).
   - CLASSIFIES each trial: **AGREED** (native watertight + vol/area/solid-count match
     OCCT within a FIXED relTol) / **HONESTLY-DECLINED** (native NULL / non-watertight,
     the engine self-verify would discard → OCCT, logged; OCCT valid) / **DISAGREED**
     (native watertight but wrong vs the ground truth — a silent wrong build) /
     **ORACLE-INACCURATE** (native matches the closed-form ground truth while OCCT does
     not — native vindicated, logged, not a bar failure) / **BOTH-DECLINED** (a
     DECLINE-exerciser both engines refuse — no wrong result). A closed-form **analytic
     arbiter** (prismatoid loft = pyramidal-frustum stack; prism sweep = area·length)
     attributes any native-vs-OCCT disagreement so a native result vindicated by exact
     math is never a false native fault (see design.md — a strengthening, not a weakening).
   - Prints a per-family coverage summary; exits 0 IFF `DISAGREED == 0` and no
     core-family unreliable oracle. Any DISAGREE / ORACLE-INACCURATE prints the seed +
     case index + family/param tuple + all measurements as a reproducible regression find.

2. **A run script** `scripts/run-sim-native-construct-fuzz.sh` — compiles the harness for
   the iOS simulator against the native math TUs (construct/tessellate/topology are
   header-only; OCCT-free, **no numsci**) and the OCCT oracle toolkits, then runs it in a
   booted simulator. Seed + N are argv/env overridable with fixed deterministic defaults.

3. **The new `.mm` is added to `scripts/run-sim-suite.sh`'s SKIP list** (own `main()`,
   OCCT-oracle slice), matching the siblings `native_boolean_fuzz.mm` /
   `native_step_import_fuzz.mm`.

4. **Delta to the `native-verification` capability spec** adding requirements for the
   construction (loft/sweep) fuzz domain (generator, dual build, analytic-arbitrated
   classifier, coverage bar with logged honest exclusions).

## Honest scope (logged, not silently dropped)

The native builders' claimed scope is broader than the fuzzer's arbitrated AGREE families.
Two claimed sub-families are DELIBERATELY EXCLUDED from the seeded comparison because
their native-mesh-vs-OCCT-exact match is only DEFLECTION-BOUNDED, not exact — so at a
fixed 2e-2 relTol they would blur the AGREE/DISAGREE line — and they are covered instead
by the curated parity harnesses:

- **Twisted / rotated-section loft** (truly bilinear hyperbolic-paraboloid side faces):
  the native mesh is inscribed, OCCT measures the exact ruled surface — covered by
  `native_loft_parity` (`buildRotatedSquareTwist`).
- **Smooth-curved planar sweep** (constant-frame ruled tube): native constant-frame vs
  OCCT `MakePipe` agree only deflection-bounded — covered by `native_sweep_parity`
  (`buildSmoothArcSweep`).

The fuzzer's arbitrated AGREE families are the ones where the native mesh reproduces the
EXACT solid (planar parallel-plane frustums; straight prisms — no tessellation-vs-exact
bias), keeping the differential razor-sharp. The **non-planar loft section** and
**non-planar sweep spine** ARE generated (sparingly) so the native DECLINE branch is
exercised for real. The N-section stack family additionally surfaces the documented
native tessellator seam limit: a stack whose consecutive bands taper at different ratios
T-junctions the shared ring → the engine self-verify discards it → OCCT (an honest
DECLINE), so prisms and symmetric spools AGREE while free random stacks decline.

These are a first-class honest DECLINE at the DOMAIN level, recorded here and in the spec
delta so no coverage is silently dropped.

## Impact

- Affected specs: `native-verification` (ADDED requirements only).
- Affected code: additive test/sim only — `tests/sim/native_construct_fuzz.mm`,
  `scripts/run-sim-native-construct-fuzz.sh`, one line in `scripts/run-sim-suite.sh`.
- `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.
- Proven: `DISAGREED == 0` across seeds 0x5744EE9911 (N=96) and 0xABCDEF12345 / 0x99 /
  0xDEADBEEFCAFE (N=128) with real per-family coverage — AGREED in loft2-frustum,
  loftN-prismatoid-stack, loft2-mismatched-count, and sweep-straight-prism; HONESTLY-
  DECLINED in the two non-planar exercisers AND in free N-section stacks; every AGREE
  exact to ~1e-15 (planar/prism families reproduce the exact solid on both sides).
  Byte-identical determinism verified across two runs of the same seed.

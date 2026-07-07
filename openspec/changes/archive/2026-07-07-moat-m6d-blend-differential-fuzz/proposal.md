# Proposal — moat-m6d-blend-differential-fuzz (MOAT M6d, the completeness bar's FOURTH domain)

## Why

MOAT M6 landed the FIRST differential-fuzzing harness — a seeded batch driving random-
but-valid curved-boolean operands through BOTH the native path and the OCCT oracle,
asserting ZERO SILENT WRONG RESULTS (`tests/sim/native_boolean_fuzz.mm`). M6b extended
it to the native STEP reader (`native_step_import_fuzz.mm`) and M6c to the native
swept-solid construction library (`native_construct_fuzz.mm`). Three fuzzed domains are
not a completeness bar: drop-occt needs the discipline proven across MORE native domains,
so a silent wrong result in ANY migrated capability is caught by a seeded batch, not only
by the handful of hand-picked fixtures the curated parity harnesses carry.

The native **blend** library (`src/native/blend` — fillet / chamfer / offset / shell) is
the natural fourth domain. It already has curated round-trip parity harnesses
(`native_blend_parity.mm`, `native_curved_fillet_parity.mm`, `native_curved_chamfer_parity.mm`)
that prove a *handful* of hand-picked fillets/chamfers agree with OCCT. That is exactly
the M6-shaped gap: a curated harness proves a few fixtures; a fuzzer proves a *seeded
batch*. This change turns the native blend path into a differential fuzzer so it is held
to the same zero-silent-wrong bar as the boolean, import, and construction domains.

This is INFRASTRUCTURE — a test harness, not a new geometry capability. `src/native/**`
is untouched and stays OCCT-free; the `cc_*` ABI is unchanged. The harness is additive
test/sim code only.

## What Changes

1. **A new differential-fuzzing harness** `tests/sim/native_blend_fuzz.mm` (own `main()`,
   seed-driven, OCCT oracle) that, for a seeded batch of N random-but-valid blend inputs:
   - DETERMINISTICALLY generates a blend input from the families the native blend path
     CLAIMS: a **planar-dihedral chamfer** (symmetric distance) and **planar-dihedral
     fillet** (constant radius) of ONE convex box edge; and, on a convex cylinder↔cap
     **circular rim**, a **constant-radius fillet**, a **variable-linear-radius fillet**
     (r1→r2), a **symmetric cone-frustum chamfer**, and an **asymmetric cone-frustum
     chamfer** (d1 axial / d2 radial). The bodies are built through the SAME native
     construct entry points the `cc_solid_extrude_profile` / `cc_solid_extrude` facade
     uses (`build_prism_profile` full-circle → capped cylinder; `build_prism` →
     axis-aligned box). Plus ONE SPARSE out-of-scope input — a fillet radius with
     `Rc/2 < r < Rc`, OUTSIDE the native ring-torus scope (`Rc ≥ 2r`) — to exercise the
     native DECLINE branch. The RNG is splitmix64-seeded xoshiro256** keyed ONLY by an
     explicit seed (argv/env) — no clock, no `rand()`.
   - BUILDS each input two ways on the SAME geometric edge/rim: the OCCT-free native
     builder (native primitive → `blend::chamfer_edges` / `fillet_edges` /
     `curved_fillet_edge` / `variable_fillet_edge` / `curved_chamfer_edge` /
     `curved_chamfer_edge_asym`, measured by the native tessellator), called DIRECTLY so
     a NULL / non-watertight result is an UNAMBIGUOUS native DECLINE (not a silent facade
     forward); and the OCCT oracle (`BRepPrimAPI_MakeBox` / `MakeCylinder` →
     `BRepFilletAPI_MakeFillet` / `MakeChamfer`, measured exactly by `BRepGProp`).
   - CLASSIFIES each trial: **AGREED** / **HONESTLY-DECLINED** / **DISAGREED** /
     **ORACLE-INACCURATE** / **BOTH-DECLINED**. A closed-form **analytic arbiter** (every
     AGREE family has an exact removed volume — a torus-canal fillet's Pappus removed
     volume, a cone-frustum chamfer's `π·d1·d2·(Rc − d2/3)`, a box-edge prism/groove) is
     the load-bearing correctness oracle: because OCCT's own variable-radius fillet is an
     APPROXIMATE evolved surface, native-vs-OCCT there is a two-approximation comparison,
     so a native result that matches EXACT MATH while OCCT strays is logged
     ORACLE-INACCURATE (native vindicated), and only a native result that FAILS exact math
     is DISAGREED (see design.md — this is a strengthening, not a weakening).
   - Prints a per-family coverage summary; exits 0 IFF `DISAGREED == 0` and no
     core-family unreliable oracle. Any DISAGREE / ORACLE-INACCURATE prints the seed +
     case index + family/param tuple + all measurements as a reproducible record.

2. **A run script** `scripts/run-sim-native-blend-fuzz.sh` — compiles the harness for the
   iOS simulator against the native blend + construct + math TUs (OCCT-free, **no
   numsci**) and the OCCT oracle toolkits, then runs it in a booted simulator. Seed + N
   are argv/env overridable with fixed deterministic defaults.

3. **The new `.mm` is added to `scripts/run-sim-suite.sh`'s SKIP list** (own `main()`,
   OCCT-oracle slice), matching the siblings `native_boolean_fuzz.mm`,
   `native_step_import_fuzz.mm`, `native_construct_fuzz.mm`.

4. **Delta to the `native-verification` capability spec** adding requirements for the
   blend fuzz domain (generator, dual build on the same edge/rim, analytic-arbitrated
   classifier, coverage bar with logged honest exclusions).

## Honest scope (logged, not silently dropped)

The native blend path's claimed scope (`native_blend.h`) also includes the CONCAVE
stepped-shaft fillet (`concave_fillet_edge`) and `offset_face` / `shell`. Those are
DELIBERATELY left to the curated parity harnesses for this FIRST blend-fuzz slice — a
concave stepped-shaft input and a shell cavity are not yet cleanly generatable as a
seeded random family with a matching OCCT oracle. That is a first-class honest DECLINE at
the DOMAIN level, recorded here and in the spec delta so no coverage is silently dropped.
The big-radius fillet (`Rc/2 < r < Rc`) is generated SPARINGLY to exercise the native
DECLINE branch for real (native returns NULL, OCCT still fillets → HONESTLY-DECLINED),
not to manufacture DISAGREE.

## Impact

- Affected specs: `native-verification` (ADDED requirements only).
- Affected code: additive test/sim only — `tests/sim/native_blend_fuzz.mm`,
  `scripts/run-sim-native-blend-fuzz.sh`, one line in `scripts/run-sim-suite.sh`.
- `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.
- Proven: `DISAGREED == 0` across seeds 0x5744EE9911 (N=96), 0xC0FFEE1234 and
  0xDEADBEEF99 (N=160) with real per-family coverage (all six core families ≥15 AGREE
  each); native-vs-exact-math error ≤ ~1.6e-3 for every family (a ~12× margin under the
  fixed `2e-2` tolerance); the DECLINE branch exercised (4–10 declines per seed). Seeds
  0xC0FFEE1234 (index 37, 134) and 0xDEADBEEF99 (index 38, 57, 76) surfaced genuine
  oracle-side inaccuracies: OCCT's variable-radius fillet re-evaluates ~2–2.6% off the
  exact analytic while the native fillet matches it to ~2–3e-4 — correctly attributed
  ORACLE-INACCURATE, native vindicated by exact math, bar NOT failed.

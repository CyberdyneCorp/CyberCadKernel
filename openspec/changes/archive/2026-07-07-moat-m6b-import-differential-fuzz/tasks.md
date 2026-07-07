# Tasks — moat-m6b-import-differential-fuzz

## 1. Deterministic seeded generator (writer-serialisable families)
- [x] 1.1 splitmix64 → xoshiro256** RNG keyed ONLY by an explicit uint64 seed (argv/env),
      no clock / `rand()`; fixed deterministic default seed + N.
- [x] 1.2 Generate solids from box / n-gon prism / cylinder / frustum / holed-box, built
      via the SAME construct entry points the `cc_*` facade uses (`build_prism`,
      `build_revolution` raw-polygon, `build_prism_with_holes`); every param constrained
      valid / non-degenerate (frustum radii both strictly positive).
- [x] 1.3 Compute the closed-form volume + area per family (the analytic arbiter input).

## 2. STEP round-trip dual import (native reader vs OCCT oracle on identical bytes)
- [x] 2.1 Export each solid to ONE on-disk STEP file via the native writer
      (`step_can_export_native` / `writeStepFile`); a decline is logged (WRITER_DECLINE),
      never faked.
- [x] 2.2 Import that SAME file via the native OCCT-free reader (`step_import_native`) and
      via OCCT `STEPControl_Reader`; measure native by the native tessellator (mesh
      vol/area, watertight, solid count) and OCCT exactly by `BRepGProp`.
- [x] 2.3 ORACLE validity gate: the OCCT re-import must be a valid closed solid, else
      ORACLE_UNRELIABLE (excluded, fails the bar — never laundered).

## 3. Analytic-arbitrated three-way classifier
- [x] 3.1 AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE at a FIXED relTol
      (2e-2), never widened per-trial.
- [x] 3.2 Attribute a native-vs-OCCT disagreement with the closed-form ground truth:
      native matches analytic AND OCCT does not → ORACLE-INACCURATE (native vindicated,
      not a bar failure); native fails analytic → DISAGREED (silent wrong import).
- [x] 3.3 Print seed + case index + family/param tuple + all three measurements on any
      DISAGREE / ORACLE-INACCURATE (a reproducible regression / limitation record).

## 4. Coverage summary + zero-silent-wrong-import bar
- [x] 4.1 Per-family summary [agreed/declined/DISAGREED/oracle-inaccurate (writer-declined)];
      exit 0 IFF `DISAGREED == 0` AND `ORACLE_UNRELIABLE == 0`.
- [x] 4.2 Log the domain-level honest exclusions (bare-periodic sphere: unreliable OCCT
      re-import; ruled loft: writer-declined) in the harness header + spec.

## 5. Build + wiring (additive test/sim only)
- [x] 5.1 `tests/sim/native_step_import_fuzz.mm` (own `main()`, `std::_Exit`, OCCT-free
      native TUs + OCCT oracle, no numsci).
- [x] 5.2 `scripts/run-sim-native-step-import-fuzz.sh` (compile + run in booted simulator,
      seed + N argv/env).
- [x] 5.3 Add `native_step_import_fuzz.mm` to `scripts/run-sim-suite.sh`'s SKIP list.
- [x] 5.4 Confirm `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.

## 6. Proof
- [x] 6.1 `DISAGREED == 0` across ≥2 seeds (0x5744EE9911, 0xB16B00B5, 0x1234; N=96 and
      N=128) with real per-family coverage.
- [x] 6.2 Byte-identical determinism across two runs of the same seed.
- [x] 6.3 Record the seed 0x1234 index=10 ORACLE-INACCURATE find (native matches the
      analytic frustum; OCCT re-imports the shallow cone ~2.7% too large).

## 7. Archive (post-merge)
- [x] 7.1 `openspec archive moat-m6b-import-differential-fuzz` after the change lands.

# Tasks — moat-m6q-clash-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_interference_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), the primitive builders (box /
      regular n-gon prism / faceted cylinder / UV-sphere) that emit BOTH a watertight
      outward-CCW native mesh AND the matching OCCT `BRepPrimAPI` solid, and the shared
      rigid transform (Rodrigues rotation + translation) applied identically to both.
- [x] 1.2 Implement the round-robin generator over {box / ngon / cyl / sphere} ×
      {CLEAR / TOUCHING / CLASH}, drawing a relative placement that lands each pair in
      its target regime — with a flush knife-edge jitter (exact-flush / slight-penetrate
      / slight-gap) on TOUCHING and a minority non-watertight soup probe — keeping the
      TOUCHING contact within the certified assembly-mate envelope (B's contact footprint
      ⊆ A's).
- [x] 1.3 Implement the closed-form arbiters — box∩box axis-aligned intersection-box
      volume + axis gap, and sphere∩sphere lens volume + centre-distance regime — and the
      OCCT oracle (`BRepAlgoAPI_Common` volume + `BRepExtrema_DistShapeShape` distance),
      declining (Unknown) if the OCCT boolean itself fails.
- [x] 1.4 Implement the six-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / FACET-CONVERGENT / BOTH-DECLINED): states agree ⇒ AGREED; native
      Unknown ⇒ HONESTLY-DECLINED; a soup probe MUST decline (else DISAGREE); a curved
      TOUCH↔CLEAR straddle ⇒ FACET-CONVERGENT; a hard split arbitrated by the closed form
      (sides with native ⇒ ORACLE-INACCURATE, sides with OCCT ⇒ DISAGREED).
- [x] 1.5 Print a per-[family × regime] coverage table; `std::_Exit(0)` IFF
      `DISAGREED == 0` with every populated cell truly exercised; report any DISAGREE /
      ORACLE-INACCURATE with seed + case index + family/regime/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-interference-fuzz.sh` cloned from
      `run-sim-native-interference.sh` (native math TUs only — `interference.h` is
      header-only — plus the OCCT oracle toolkits `TKBO`/`TKShHealing`/`TKHLR`/`TKPrim`/…),
      seeded ONLY by `FUZZ_SEED`/argv (default N=72).
- [x] 2.2 Add `native_interference_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
      `main()`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across ≥2 seeds, N ≥ 60/seed; capture
      the coverage table; verify `DISAGREED == 0`.
- [x] 3.3 Re-run one seed twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×16 → ×17 (add the
      interference/clash domain entry; reconcile with the concurrent draft-fuzzer bump so
      the final count is ×17).
- [x] 4.2 `openspec validate moat-m6q-clash-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6q` (concise technical message, no Claude/AI mention).

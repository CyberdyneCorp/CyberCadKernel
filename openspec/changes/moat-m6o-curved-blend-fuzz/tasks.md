# Tasks — moat-m6o-curved-blend-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_curved_blend_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), the position-welded
      watertight / Euler-χ / mesh-volume diagnostics, and the analytic-revolve base
      solids (capped cylinder / cone frustum / sphere-cap dome) built through the ACTIVE
      engine's public facade.
- [x] 1.2 Implement the nine curved-blend families {FILLET, SHELL, OFFSET} × {cyl, cone,
      sphere} driven through `cc_fillet_edges` / `cc_shell` / `cc_offset_face` under BOTH
      engines (`cc_set_engine`), with engine-independent sub-shape pickers
      (`findRimEdge` / `capFaceIds` / `cyl|cone|sphereWallFaceId` resolved per engine).
- [x] 1.3 Implement the closed-form volume arbiters (fillet toroidal canal; shell wall;
      offset re-radius) — including the cone-offset `d/cosσ` perpendicular-shift, matching
      `curved_offset.h` — and the OCCT oracles (facade for fillet/shell, direct
      `BRepPrimAPI` for the planar-only offset facade).
- [x] 1.4 Implement the six-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED) arbitrated by the closed-form
      truth with FIXED never-widened deflection-convergence bands.
- [x] 1.5 Print a per-family coverage table; `std::_Exit(0)` IFF `DISAGREED == 0 &&
      ORACLE_UNRELIABLE == 0` with each of the nine families ≥1 AGREED; report any
      DISAGREE / ORACLE-INACCURATE with seed + case index + family/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-curved-blend-fuzz.sh` cloned from
      `run-sim-native-curved-offset.sh` (whole kernel + OCCT for the facade path,
      `TKHLR`/`TKShHealing` retained), seeded ONLY by `FUZZ_SEED`/argv (default N=72).
- [x] 2.2 Add `native_curved_blend_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
      `main()`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across ≥2 seeds, N ≥ 60/seed; capture
      the coverage table; verify `DISAGREED == 0`.
- [x] 3.3 Re-run one seed twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×14 → ×15 (add the
      curved-blend domain entry).
- [x] 4.2 `openspec validate moat-m6o-curved-blend-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6o` (concise technical message, no Claude/AI mention).

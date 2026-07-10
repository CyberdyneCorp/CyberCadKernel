# Tasks — moat-m6s-variable-sweep-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_vsweep_fuzz.mm` with the shared splitmix64 → xoshiro256**
      `Rng` (keyed ONLY by `FUZZ_SEED`), the position-welded watertight / Euler-χ /
      mesh-volume diagnostics, and the profile builders (circle 64/48-gon, regular n-gon,
      per-vertex radial polygon) + straight/arc spine + splaying guide generators.
- [x] 1.2 Implement the five families — circle-morph straight, polygon-morph straight,
      section-A→B straight, guided straight, circle-morph curved-arc — each swept through
      the public `cc_variable_sweep` facade under BOTH engines (`cc_set_engine`) on
      IDENTICAL inputs.
- [x] 1.3 Implement the CLOSED-FORM straight-spine volume arbiter: `∫₀¹ A(f)·H df` with
      `A(f) = polyArea((A_k + (B_k−A_k)f)·s(f))` and `s(f)` the guide-splay scale,
      evaluated by an exact composite Simpson over 4 sub-intervals (A(f) is degree ≤4 in f);
      the circle families use the smooth truncated-cone `πH/3(r0²+r0r1+r1²)`. The OCCT
      oracle is `cc_set_engine(0)` → `BRepOffsetAPI_MakePipeShell` multi-section via the
      facade, measured by `cc_mass_properties`.
- [x] 1.4 Implement the six-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED) arbitrated by the closed-form
      truth (straight families) or OCCT alone (curved family) with FIXED never-widened
      bands (native-vs-closed-form volume ≤ 2e-3 polygon / ≤ 1.2e-2 circle, native-vs-OCCT
      volume ≤ 5e-2, area ≤ 8e-2), gating on watertight + χ=2 + positive volume. Read the
      native BUILDER decline signal (`build_variable_sweep` NULL, a read-only probe) so a
      native forward-to-OCCT lands DECLINED, keeping AGREED strictly native-produced.
- [x] 1.5 Interleave out-of-envelope DECLINE probes (mismatched vertex counts, coincident
      guide start, collapsing guide scale, non-planar guided helix) at a fixed cadence so
      the honest-decline branch is covered every run.
- [x] 1.6 Print a per-family coverage table; `std::_Exit(0)` IFF `DISAGREED == 0 &&
      ORACLE_UNRELIABLE == 0` with each of the five families ≥1 AGREED; report any
      DISAGREE / ORACLE-INACCURATE with seed + case index + family/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-vsweep-fuzz.sh` cloned from
      `run-sim-native-vsweep.sh` (whole kernel + broad OCCT toolkit set,
      `TKHLR`/`TKShHealing` retained, NO numsci — the native variable-sweep path is
      OCCT-free and numsci-free), seeded ONLY by `FUZZ_SEED`/argv (default N=72), runs TWO
      default seeds and fails if either fails.
- [x] 2.2 Add `native_vsweep_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own `main()`,
      `std::_Exit`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across 2 seeds, N = 72/seed; capture the
      coverage table; verify `DISAGREED == 0` and `ORACLE_UNRELIABLE == 0` on both.
- [x] 3.3 Re-run one seed twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×17 → ×19 (add the
      variable-section-sweep domain entry; the concurrent tracks bump the same row —
      reconciled at merge to the final ×20).
- [x] 4.2 `openspec validate moat-m6s-variable-sweep-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6s` (concise technical message, no Claude/AI mention).

# Tasks — moat-m6p-draft-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_draft_faces_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), the position-welded watertight /
      Euler-χ / mesh-volume diagnostics, and the prismatic base solids (box / regular
      n-gon prism, n∈[3,8]) built through the ACTIVE engine's public `cc_solid_extrude`.
- [x] 1.2 Implement the four families {BOX, NGON} × {SINGLE-face, MULTI-face}: select a
      random subset of planar side faces (single = one; multi = ≥2), resolve the SAME
      subset per engine via `cc_subshape_ids` + `cc_project_point_on_face` (foot-on-plane
      + outward-nudge normal discriminator), and draft them about the base plane
      (origin (0,0,0), pull +Z) at a random valid angle through `cc_draft_faces` under
      BOTH engines (`cc_set_engine`).
- [x] 1.3 Implement the CLOSED-FORM drafted-volume arbiter: `V = ∫₀^h A(z) dz` with A(z)
      the footprint polygon clipped (Sutherland–Hodgman) by each drafted edge's
      inward-shifted (`z·tanθ`) half-plane, integrated with an exact 3-point Simpson
      (A(z) is degree ≤2 in z) — handling adjacent-face corner interactions exactly. The
      OCCT oracle is `cc_set_engine(0)` → `BRepOffsetAPI_DraftAngle` via the facade,
      measured by `cc_mass_properties`.
- [x] 1.4 Implement the six-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED) arbitrated by the closed-form
      truth with FIXED never-widened bands (native-vs-closed-form volume ≤ 1e-3, native-
      vs-OCCT volume ≤ 2e-2, area ≤ 3e-2), gating on watertight + χ=2 + strict shrink.
- [x] 1.5 Print a per-family coverage table; `std::_Exit(0)` IFF `DISAGREED == 0 &&
      ORACLE_UNRELIABLE == 0` with each of the four families ≥1 AGREED; report any
      DISAGREE / ORACLE-INACCURATE with seed + case index + family/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-draft-faces-fuzz.sh` cloned from
      `run-sim-native-directmodel-fuzz.sh` (whole kernel + OCCT + numsci iossim substrate
      for the native draft's inward split-plane cut, `TKHLR`/`TKShHealing` retained),
      seeded ONLY by `FUZZ_SEED`/argv (default N=72), runs TWO default seeds and fails if
      either fails.
- [x] 2.2 Add `native_draft_faces_fuzz.mm` to the `run-sim-suite.sh` SKIP list (own
      `main()`, `std::_Exit`).

## 3. Build & gate

- [x] 3.1 `scripts/build-numsci.sh` host + iossim both exit 0 (product unchanged).
- [x] 3.2 Run the harness on the booted simulator across 2 seeds, N = 72/seed; capture
      the coverage table; verify `DISAGREED == 0` and `ORACLE_UNRELIABLE == 0` on both.
      (0xD4AF7A11EE → 60 AGREED / 12 HONESTLY-DECLINED / 0 DISAGREED;
      0x5EEDDA7A16 → 64 / 8 / 0 — every AGREE native==OCCT==closed-form to ~1e-16.)
- [x] 3.3 Re-run seed 0xD4AF7A11EE twice → byte-identical batch (determinism proof).

## 4. Docs & structural discipline

- [x] 4.1 Update `openspec/MOAT-ROADMAP.md` M6 row: breadth ×15 → ×16 (add the
      draft-angle domain entry).
- [x] 4.2 `openspec validate moat-m6p-draft-fuzz --strict` passes.
- [x] 4.3 Structural check: `git diff` touches ONLY `tests/sim` + `scripts` + `openspec`
      (NOT `src/native`, `src/engine`, `include`).
- [x] 4.4 Commit to branch `moat-m6p` (concise technical message, no Claude/AI mention).

# Tasks — moat-m6v-wrap-emboss-fuzz

## 1. Harness

- [x] 1.1 Add `tests/sim/native_wrap_emboss_freeform_fuzz.mm` with the shared splitmix64 →
      xoshiro256** `Rng` (keyed ONLY by `FUZZ_SEED`), the exact closed-form arbiters
      (cylinder `A·|Rout²−R²|/(2R)`; sphere-cap shell-sector `2π(1−cosφ0)·((R+h)³−R³)/3`),
      a shoelace area helper, a position-welded watertight + Euler-χ mesh check, and a mesh
      volume helper.
- [x] 1.2 Implement the base × mode families: `{cylinder, sphere-cap} × {raised(boss=1),
      recessed(boss=0)}`. Build the base solid IDENTICALLY under both engines
      (`cc_solid_extrude_profile` kind-2 full circle for the cylinder;
      `cc_solid_revolve_profile` base+arc+axis segments for the sphere-cap dome). Resolve
      the wrap face id GEOMETRICALLY on the body being embossed (all mesh vertices ≈ R from
      the axis for the cylinder wall; ≈ R from the dome centre for the sphere wall). Add the
      out-of-envelope decline exercisers (general non-cylindrical developable base,
      self-intersecting footprint, boss reaching the dome rim, >2π footprint, deboss ≥ R).
- [x] 1.3 Drive the SAME wrap input through the public `cc_wrap_emboss` facade under BOTH
      engines (`cc_set_engine(1)`=NativeEngine, `cc_set_engine(0)`=OCCT); measure volume +
      area via `cc_mass_properties` and watertight/χ/mesh-volume via `cc_tessellate`. For
      the sphere base ASSERT the OCCT wrap DECLINES (returns 0 on the sphere wall) and use
      the base-dome OCCT volume + closed-form delta as the reference.
- [x] 1.4 Implement the five-way classifier (AGREED / HONESTLY-DECLINED / DISAGREED /
      ORACLE_UNRELIABLE / BOTH-DECLINED): cylinder arbitrated by the closed-form delta
      (PRIMARY) cross-checked against OCCT volume+area (bands vol ≤ 2e-2, area ≤ 3e-2);
      sphere arbitrated by the shell-sector closed form on the OCCT base-dome volume (band
      vol ≤ 1.5e-2, mesh-vs-brep ≤ 2e-2). FIXED, never widened. The `sphere+recessed` cell
      is a first-class DOMAIN-level decline (native has no sphere-deboss path).
- [x] 1.5 Print a per-cell coverage table (`{cylinder,sphere}×{raised,recessed}`);
      `std::_Exit(0)` IFF `DISAGREED == 0 && ORACLE_UNRELIABLE == 0` with each IN-SCOPE cell
      ≥1 AGREED and no guard-leak SURPRISE; report any DISAGREE / ORACLE_UNRELIABLE with
      seed + case index + base/mode/param tuple.

## 2. Runner + suite wiring

- [x] 2.1 Add `scripts/run-sim-native-wrap-emboss-freeform-fuzz.sh` cloned from
      `run-sim-native-wrap-emboss.sh` (whole-kernel link — facade + core + engine[native+
      occt] + native math — full OCCT toolkit set incl. `TKHLR`/`TKShHealing`). Seeded ONLY
      by `FUZZ_SEED`/argv; default N=64/seed; run TWO default seeds; fail if either fails.
- [x] 2.2 Add `native_wrap_emboss_freeform_fuzz.mm` to the `run-sim-suite.sh` SKIP list
      (own `main()`, `std::_Exit`).

## 3. Gate + docs

- [x] 3.1 `scripts/build-numsci.sh host` + `iossim` both exit 0.
- [x] 3.2 Run on the booted simulator, ≥2 seeds, N≥60/seed: DISAGREED == 0,
      ORACLE_UNRELIABLE == 0, each in-scope cell ≥1 AGREED, determinism re-verified.
- [x] 3.3 Update `openspec/MOAT-ROADMAP.md` M6 row (breadth ×20 → ×22; reconcile with the
      concurrent sheet-metal fuzzer at merge — final ×22).
- [x] 3.4 `openspec validate --strict moat-m6v-wrap-emboss-fuzz`; structural check
      (`git diff` touches only `tests/sim` + `scripts` + `openspec`); `src/native`,
      `src/engine`, `include`, `cc_*` ABI byte-unchanged.

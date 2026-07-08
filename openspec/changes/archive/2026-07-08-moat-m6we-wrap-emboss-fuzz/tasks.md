# Tasks — moat-m6we-wrap-emboss-fuzz

## 1. Deterministic seeded generator (native-claimed wrap-emboss families)
- [x] 1.1 splitmix64 → xoshiro256** RNG keyed ONLY by an explicit uint64 seed (argv/env),
      no clock / `rand()`; fixed deterministic default seed + N.
- [x] 1.2 Generate wrap-emboss inputs from the four core families — rectangular pad
      (emboss), rectangular deboss (pocket), convex N-gon (n=3..7) emboss, convex N-gon
      deboss — plus SPARSE out-of-scope DECLINE-exercisers: non-cylindrical base, >2π
      footprint, deboss depth ≥ R, self-intersecting (pentagram) loop.
- [x] 1.3 Build every base body through the SAME construct entry point the facade uses
      (`build_prism_profile` full-circle → capped cylinder, ONE Cylinder wall face);
      constrain every core param valid / in-scope (arc span < 2π, axial span strictly
      inside the wall, deboss depth < R, positive height); BOUND the polygon circumradius /
      amount so the inscribed-cap facet floor stays under the fixed tolerance.
- [x] 1.4 Compute the closed-form curvature-corrected changed volume per core case
      (`ΔV = A·|Rout²−R²|/(2R)`, `A` = shoelace footprint area) — the primary arbiter input.

## 2. Dual build + measure (native builder called directly; closed-form + OCCT-recon oracle)
- [x] 2.1 Call the OCCT-free native builder DIRECTLY (`feature::wrap_emboss`) so a NULL /
      non-watertight result is an UNAMBIGUOUS native DECLINE, not a silent facade forward;
      pick the Cylinder wall face by `surfaceOf(...).kind == Cylinder`; measure by the
      native tessellator (mesh vol/area, watertight, solid count).
- [x] 2.2 PRIMARY oracle: the closed-form curvature-corrected expected total volume
      (`πR²H ± ΔV`), universal across rectangle and polygon footprints.
- [x] 2.3 SECONDARY oracle (rectangle families, where clean): reconstruct the SAME solid via
      OCCT boolean — `Fuse(base, outer pie wedge)` for a pad, `Cut(base, shell wedge)` for a
      pocket (inner core grown so faces are non-coincident) — measured exactly by
      `BRepGProp` + a `BRepCheck` validity check; best-effort, guarded by `IsDone()`, falling
      back to the authoritative closed form on any boolean failure.
- [x] 2.4 Honest ORACLE-level decline: NO OCCT reconstruction for the polygon families (a
      wrapped-polygon pad would re-implement the feature) — closed-form volume only; recorded
      in the harness header + spec.

## 3. Five-way classifier (closed form is the primary correctness oracle)
- [x] 3.1 AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED at a
      FIXED tolerance (`kVolRelTol = 2e-2`, `kAreaRelTol = 3e-2`), never widened per-trial.
- [x] 3.2 nativeUsable & !analyticMatch → DISAGREED; nativeUsable & analyticMatch with a
      valid OCCT reconstruction that mismatches VOLUME → ORACLE-INACCURATE, that mismatches
      AREA → DISAGREED; else AGREED. Correctness gated on the exact-math volume so an
      area-only excursion can never falsely DISAGREE a polygon family.
- [x] 3.3 Out-of-scope DECLINE-exerciser: native MUST return NULL / non-watertight →
      BOTH-DECLINED; a watertight native solid for an out-of-scope input is a guard-leak
      SURPRISE that FAILS the bar (never laundered).
- [x] 3.4 Print seed + case index + family/param tuple + all measurements (native, closed
      form, OCCT recon) on any DISAGREE / ORACLE-INACCURATE / SURPRISE.

## 4. Coverage summary + zero-silent-wrong-wrap-emboss bar
- [x] 4.1 Per-family summary [agreed/declined/DISAGREED/oracle-inaccurate/both-declined] +
      the count of OCCT rectangle reconstructions checked + the measured max native-vs-oracle
      bias vs the fixed tol; exit 0 IFF `DISAGREED == 0` AND no guard-leak SURPRISE.
- [x] 4.2 Log the ORACLE-level honest exclusion (no clean OCCT reconstruction for polygon
      footprints — closed-form volume only) in the harness header + spec.

## 5. Build + wiring (additive test/sim only)
- [x] 5.1 `tests/sim/native_wrap_emboss_fuzz.mm` (own `main()`, `std::_Exit`, OCCT-free
      native TUs + OCCT oracle, no numsci).
- [x] 5.2 `scripts/run-sim-native-wrap-emboss-fuzz.sh` (compile + run in booted simulator,
      seed + N argv/env, links `TKPrim` + `TKBO` + `TKGeomAlgo` + `TKTopAlgo` + geometry base).
- [x] 5.3 Add `native_wrap_emboss_fuzz.mm` to `scripts/run-sim-suite.sh`'s SKIP list.
- [x] 5.4 Confirm `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.

## 6. Proof
- [x] 6.1 `DISAGREED == 0` across ≥2 seeds — proven across 6 (0x5745E6B055 N=120;
      0xC0FFEE01, 0xABCDEF99, 0x1337BEEF, 0x2, 0xDEADBEEF N=240–300; ~1500 trials) with real
      per-family coverage (all four core families AGREE in the hundreds each) and all four
      DECLINE-exercisers hitting BOTH-DECLINED.
- [x] 6.2 Max native-vs-oracle bias on AGREE ≤ 9.5e-3 (a ~2× margin under the fixed 2e-2
      tolerance); 100+ OCCT rectangle reconstructions validating the closed form per seed;
      byte-identical determinism across two runs of the same seed.
- [x] 6.3 Zero DISAGREED, zero ORACLE-INACCURATE, zero guard-leak SURPRISE across the batch.

## 7. Archive (post-merge)
- [x] 7.1 `openspec archive moat-m6we-wrap-emboss-fuzz` after the change lands.

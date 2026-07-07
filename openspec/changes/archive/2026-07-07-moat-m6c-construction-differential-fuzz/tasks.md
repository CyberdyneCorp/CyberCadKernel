# Tasks — moat-m6c-construction-differential-fuzz

## 1. Deterministic seeded generator (native-claimed construction families)
- [x] 1.1 splitmix64 → xoshiro256** RNG keyed ONLY by an explicit uint64 seed (argv/env),
      no clock / `rand()`; fixed deterministic default seed + N.
- [x] 1.2 Generate LOFT inputs: 2-section coaxial regular-n-gon frustum (equal count);
      N-section (3..5) prismatoid stack drawn mostly from the welding sub-families (prism /
      symmetric 3-spool) with a free-random fraction; a T1 mismatched-count loft (n → 2n via
      collinear midpoint insertion — same regular-n-gon outline). Generate SWEEP inputs:
      a closed planar profile (jittered n-gon) swept along a STRAIGHT 3D path.
- [x] 1.3 Generate the two SPARSE out-of-scope DECLINE-exercisers: a non-planar loft section
      (a lifted middle-section vertex) and a non-planar sweep spine (a 3D polyline).
- [x] 1.4 Compute the closed-form volume + area for every arbitrated family (prismatoid
      pyramidal-frustum stack; straight prism) — the analytic arbiter input.

## 2. Native-vs-OCCT dual build on identical inputs
- [x] 2.1 Build each input DIRECTLY via the OCCT-free native builder
      (`ncst::build_loft_sections` / `ncst::build_sweep`); a NULL / non-watertight result is
      an unambiguous native DECLINE (the engine self-verify would discard it → OCCT).
- [x] 2.2 Build the SAME input via the OCCT oracle — `BRepOffsetAPI_ThruSections`(solid,
      ruled) over the section polygons and `BRepOffsetAPI_MakePipe` of the centroid-centred
      profile face along the spine polyline (the exact idiom `occt_construct.cpp` uses);
      measure native by the native tessellator (mesh vol/area, watertight, solid count) and
      OCCT exactly by `BRepGProp` + a `BRepCheck` validity + closed-shell check.
- [x] 2.3 ORACLE validity gate: for a CORE family the OCCT build must be a valid closed
      solid, else ORACLE_UNRELIABLE (excluded, fails the bar — never laundered). A
      DECLINE-exerciser both engines refuse is BOTH-DECLINED (logged, not a bar failure).

## 3. Analytic-arbitrated classifier
- [x] 3.1 AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED at a
      FIXED relTol (2e-2), never widened per-trial.
- [x] 3.2 Attribute a native-vs-OCCT disagreement with the closed-form ground truth: native
      matches analytic AND OCCT does not → ORACLE-INACCURATE (native vindicated, not a bar
      failure); native fails analytic → DISAGREED (silent wrong build).
- [x] 3.3 Print seed + case index + family/param tuple + all measurements on any DISAGREE /
      ORACLE-INACCURATE (a reproducible regression / limitation record).

## 4. Coverage summary + zero-silent-wrong-build bar
- [x] 4.1 Per-family summary [agreed/declined/DISAGREED/oracle-inaccurate/both-declined];
      exit 0 IFF `DISAGREED == 0` AND core-family `ORACLE_UNRELIABLE == 0`.
- [x] 4.2 Log the domain-level honest exclusions (twisted/rotated-section loft and
      smooth-curved planar sweep — deflection-bounded, covered by the curated parity
      harnesses; N-section mismatched-taper stacks T-junction → honest self-verify DECLINE)
      in the harness header + spec.

## 5. Build + wiring (additive test/sim only)
- [x] 5.1 `tests/sim/native_construct_fuzz.mm` (own `main()`, `std::_Exit`, OCCT-free
      native TUs + OCCT oracle, no numsci).
- [x] 5.2 `scripts/run-sim-native-construct-fuzz.sh` (compile + run in booted simulator,
      seed + N argv/env).
- [x] 5.3 Add `native_construct_fuzz.mm` to `scripts/run-sim-suite.sh`'s SKIP list.
- [x] 5.4 Confirm `src/native/**` UNTOUCHED and OCCT-free; `cc_*` ABI unchanged.

## 6. Proof
- [x] 6.1 `DISAGREED == 0` across ≥2 seeds (0x5744EE9911 N=96; 0xABCDEF12345, 0x99,
      0xDEADBEEFCAFE N=128) with real per-family coverage — AGREED in loft2-frustum,
      loftN-prismatoid-stack, loft2-mismatched-count, sweep-straight-prism; HONESTLY-
      DECLINED in the two non-planar exercisers and free N-section stacks.
- [x] 6.2 Byte-identical determinism across two runs of the same seed (0x99 N=64).
- [x] 6.3 Record that every AGREE is exact to ~1e-15 (planar/prism families reproduce the
      exact solid on both sides), so no ORACLE-INACCURATE was needed — the arbiter stands
      as a ready strengthening, not a tolerance dodge.

## 7. Archive (post-merge)
- [x] 7.1 `openspec archive moat-m6c-construction-differential-fuzz` after the change lands.

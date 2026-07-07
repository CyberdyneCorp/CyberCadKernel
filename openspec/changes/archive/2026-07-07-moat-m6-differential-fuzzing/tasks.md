# Tasks — moat-m6-differential-fuzzing

Order: seeded generator → dual builder → three-way classifier → coverage summary →
sim runner script → verify a batch on a booted simulator. All NEW sim/test code;
`src/native` stays OCCT-free and is NOT modified; OCCT appears only on the oracle
side of the `.mm`. Deterministic only (seeded RNG, no clock). No `cc_*` ABI change.
Do NOT commit. Model the harness and script on the curated sibling
(`tests/sim/native_ssi_curved_boolean_parity.mm` +
`scripts/run-sim-native-ssi-curved-boolean.sh`).

## 1. Deterministic seeded input generator (`tests/sim/native_boolean_fuzz.mm`)

- [x] 1.1 Inline `splitmix64` → `xoshiro256**` PRNG seeded ONLY by an explicit
      64-bit seed; helpers `next_double(lo,hi)`, `next_int(lo,hi)`, `next_choice`.
      NO clock / `rand()` / `Date` / pid / address. Read `FUZZ_SEED` / `FUZZ_COUNT`
      from `getenv` with fixed defaults.
- [x] 1.2 `struct OperandSpec` POD (family enum {Cyl,Sphere,Cone}, axis, centre[3],
      radius, r1/halfAngle, lo, hi) and `struct PairSpec` (two OperandSpec +
      placement enum {Coaxial, OrthogonalThrough, OffsetParallel}). No OCCT/native
      type in the POD.
- [x] 1.3 `sampleValidPair(rng) -> PairSpec`: radii ∈ [0.4,2.5], extent half-length
      ∈ [1.5,4.0], cones non-degenerate (`r0 ≠ r1`, positive span), placement/family
      drawn so operands OVERLAP and the native recogniser accepts them (exercise, not
      trivial decline). Family pairs from {cyl×cyl, sphere×sphere, cone×cyl,
      cone×sphere, sphere×cyl}.
- [x] 1.4 Determinism check inline: same seed + count ⇒ identical tuple sequence
      (assert by re-generating a short prefix twice).

## 2. Dual native + OCCT-oracle builder

- [x] 2.1 `buildNative(OperandSpec) -> ntopo::Shape` via the OCCT-free builders
      (`makeCyl`/`makeSphere`/`makeCone` — the curated harness's constructors:
      `nb::curved::buildCommonSegment` + `AxisCylinder`, etc.). No OCCT type.
- [x] 2.2 `buildOcct(OperandSpec) -> TopoDS_Shape` via `BRepPrimAPI_MakeCylinder`/
      `MakeSphere`/`MakeCone` transformed to the same axis/centre/extent.
- [x] 2.3 Pre-boolean operand self-check: native operand volume/area ≈ OCCT operand
      volume/area within tol (guarantees the two builders produce the SAME solid, so
      a downstream DISAGREE is attributable to the boolean, not the input).

## 3. Three-way classifier vs OCCT

- [x] 3.1 Reuse `nativeMeshVerify` (tessellate → watertight + enclosed volume + area)
      and `occtVerify` (valid + closed shell + `BRepGProp` volume/area) helper shapes
      from the curated harness.
- [x] 3.2 `occtBoolean(a,b,op)` (`BRepAlgoAPI_{Fuse,Cut,Common}`) and native
      `nb::boolean_solid(a,b,op)` for each op ∈ {Fuse,Cut,Common}.
- [x] 3.3 Classify EXACTLY one of AGREED / HONESTLY_DECLINED / DISAGREED /
      FALLBACK_ORACLE_INVALID per the design decision-5 logic: watertight+match →
      AGREED; watertight+outside-tol → DISAGREED (fail); NULL/non-watertight +
      valid OCCT → HONESTLY_DECLINED; NULL + invalid OCCT → FALLBACK_ORACLE_INVALID
      (fail). FIXED `VOL_TOL`/`AREA_TOL` sized to tessellation deflection — NEVER
      widened to hide a gap.
- [x] 3.4 On DISAGREE, print `DISAGREE seed=… idx=… op=… family=… spec={…}
      volNat/volOcct/relVol …` — a self-contained regression find.

## 4. Coverage summary + exit code

- [x] 4.1 Accumulate counts overall and per (family-pair, op).
- [x] 4.2 Print summary: seed, `N` = pairs × 3 ops, agreed / honestly-declined /
      DISAGREED / oracle-invalid, per-op and per-family breakdown.
- [x] 4.3 Exit `0` iff `DISAGREED == 0 && FALLBACK_ORACLE_INVALID == 0`; non-zero
      otherwise. Print `BAR: zero silent wrong results — PASS/FAIL`.

## 5. Sim runner script (`scripts/run-sim-native-boolean-fuzz.sh`)

- [x] 5.1 Copy `run-sim-native-ssi-curved-boolean.sh`; swap harness path →
      `tests/sim/native_boolean_fuzz.mm` and output binary →
      `native_boolean_fuzz`. Keep the numsci iossim substrate build, native TU list,
      and OCCT toolkit link line unchanged.
- [x] 5.2 Pass `FUZZ_SEED` / `FUZZ_COUNT` through to the `simctl spawn` environment
      (default seed fixed, default count e.g. 128 pairs → 384 classified inputs).
- [x] 5.3 Add the harness to the SKIP list of `run-sim-suite.sh` (own `main()`,
      OCCT + numsci slice) so the aggregate runner does not try to link it.

## 6. Verify (deterministic batch on a booted simulator)

- [x] 6.1 Boot an iOS simulator (`xcrun simctl list devices booted`; boot iPhone 16
      if none) and run `scripts/run-sim-native-boolean-fuzz.sh` with the default seed.
- [x] 6.2 Confirm the coverage summary reports `DISAGREED == 0` and the process
      exits zero — the M6 bar. Confirm a non-trivial `agreed` count (native exercised,
      not all declines).
- [x] 6.3 Re-run with the same seed; confirm the batch is byte-identical
      (determinism). Try a second seed for breadth.
- [x] 6.4 If ANY DISAGREE surfaces, do NOT hide it: record the printed seed + index +
      tuple as a regression find and report it (the harness catching a real native bug
      is the intended outcome, not a harness failure).
- [x] 6.5 Confirm no existing suite regressed (the fuzz runner is standalone; nothing
      in `src/native` or the curated harnesses changed).

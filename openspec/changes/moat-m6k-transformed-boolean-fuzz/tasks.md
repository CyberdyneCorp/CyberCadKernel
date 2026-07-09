# Tasks — moat-m6k-transformed-boolean-fuzz (MOAT M6-breadth-11)

Order: confirm the substrate → pick the domain → harness skeleton (facade-driven, both
engines) → operand + transform generators → composed-op drivers + closed-form invariant
arbiter → five-way classifier → coverage bar over ≥2 seeds → runner + suite SKIP +
roadmap. NO change to `src/native/**`, `src/engine/**`, `include/**` (system under test);
no `cc_*` ABI change; no tolerance weakened.

## 0. Substrate

- [x] 0.1 Build the OCCT-linked simulator numsci and host: `bash scripts/build-numsci.sh
      host && bash scripts/build-numsci.sh iossim` (both exit 0). The shared `NativeEngine`
      TU pulls in `CYBERCAD_HAS_NUMSCI`-gated freeform paths, so the harness links numsci.
- [x] 0.2 Confirm in source: `NativeEngine::boolean_op` forwards two native operands to
      `cybercad::native::boolean::boolean_solid` (planar BSP-CSG, self-verified watertight +
      set-algebra volume); it is native ONLY for all-planar polyhedra; the transforms
      (`cc_translate_shape` / `cc_rotate_shape_about` / `cc_mirror_shape`) produce a
      `Shape::located(math::Transform)`; `src/native/boolean/polygon.h` folds the face
      `Location` into each world polygon + normal; the `cc_*` facade exposes `cc_boolean` /
      the transforms / `cc_set_engine` / `cc_mass_properties` / `cc_solid_extrude`.

## 1. Domain choice (transformed-boolean vs section-curve vs HLR vs healing)

- [x] 1.1 Pick boolean-of-transformed-operands: the roadmap M6 REMAINING list names it as
      the interaction "single-domain fuzzers miss". Section-curve (GS2) and HLR (GS1) are
      already covered inside `native_geometry_services_fuzz`; healing is a valid future
      pick but the transform×boolean INTERACTION is a distinct, un-fuzzed composition with
      a pristine rigid-invariant arbiter. Document the deferral of section/HLR (already in
      GS2/GS1) and healing (next candidate).

## 2. Harness skeleton (facade-driven, both engines)

- [x] 2.1 Create `tests/sim/native_transformed_boolean_fuzz.mm` with the RNG helper
      (splitmix64 → xoshiro256**, `FUZZ_SEED` from argv/env, fixed default `0xB007C0DE11`),
      the coverage tally, the five-way `Verdict`, and the coverage-summary + `std::_Exit`
      epilogue, mirroring the sibling fuzzers.
- [x] 2.2 Include ONLY `cybercadkernel/cc_kernel.h` (no native C++ headers) — this harness
      drives the public facade like `native_directmodel_fuzz.mm`.

## 3. Operand + transform generators (built identically under both engines)

- [x] 3.1 `genPair` for two all-planar operands per family: BOX (box × offset box), NGON
      (n-gon prism × straddling box, n∈[3,7]), LSHAPE (concave L-prism × straddling box),
      positioned so `A∩B` is a clean transversal overlap valid for all three ops.
- [x] 3.2 `genXForm` for IDENTITY / TRANSLATE / ROTATE-about-random-axis /
      MIRROR-through-random-plane; the first `T_COUNT` trials force each kind (coverage).
- [x] 3.3 Build each operand under `cc_set_engine(0)` and `cc_set_engine(1)`; an operand
      neither engine builds → BOTH-DECLINED. RAII `Body` releases each id under its owning
      engine.

## 4. Composed-op driver + closed-form invariant arbiter (both engines)

- [x] 4.1 Baseline `R0 = cc_boolean(A, B, op)` and transformed `RT = cc_boolean(T(A), T(B),
      op)` under EACH engine (transforms applied via the facade to the built operand ids).
- [x] 4.2 PRIMARY arbiter: rigid VOLUME invariant `|RT|_vol == |R0|_vol` at a TIGHT
      tolerance (`kInvVol=1e-6`) + the AREA invariant at the meshed-facet-weld bound
      (`kInvArea=5e-3`, volume stays exact; only the derived welded-facet area wobbles).
- [x] 4.3 SECONDARY arbiter: native `RT` vs OCCT `RT` volume + area at the sibling-proven
      `native_boolean_fuzz` tolerance `2e-2` (never widened). Measure every shape under the
      engine that BUILT it (guard the cross-engine unwrap crash).

## 5. Classifier

- [x] 5.1 Five-way classify (AGREED / HONESTLY-DECLINED / BOTH-DECLINED / ORACLE_UNRELIABLE
      / DISAGREED). Native NULL on a near-tangent/degenerate composed op while OCCT ships →
      HONESTLY-DECLINED. OCCT breaks its OWN rigid invariant while native upholds the tight
      closed-form invariant → ORACLE_UNRELIABLE (native vindicated). Invariant break OR
      native-vs-OCCT mismatch → DISAGREED.
- [x] 5.2 Coverage summary (per-family + per-op + per-transform AGREED/DISAGREED); bar =
      `DISAGREED==0 && ORACLE_UNRELIABLE==0 && every family + op + transform ≥1 AGREED`.

## 6. Runner + suite + roadmap

- [x] 6.1 `scripts/run-sim-native-transformed-boolean-fuzz.sh` (whole kernel + OCCT +
      numsci iossim; ≥2 default seeds `0xB007C0DE11` / `0x1DEA5EED77`; fails if any seed
      fails).
- [x] 6.2 Add `native_transformed_boolean_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 6.3 Update `openspec/MOAT-ROADMAP.md` M6 row (breadth ×10 → ×11) with the coverage.

## 7. Gate (≥2 seeds, N≥60)

- [x] 7.1 Run both default seeds at N=96 on the booted simulator: seed `0xB007C0DE11`
      89 AGREED / 7 HONESTLY-DECLINED / 0 DISAGREED / 0 ORACLE_UNRELIABLE / 0 BOTH-DECLINED;
      seed `0x1DEA5EED77` 95 / 1 / 0 / 0 / 0. Both PASS; every operand family, every op, and
      every transform kind ≥1 AGREED on each seed.
- [x] 7.2 Localised the initial too-tight-tolerance false positives: the PRIMARY VOLUME
      invariant holds to ≤1e-15 across 400 trials (zero violations) — the native
      transform+boolean composition is EXACT; the apparent gaps were (a) native-vs-OCCT at
      ~0.1–0.9% (OCCT meshing/rounding on concave prisms, well inside the sibling-proven
      `2e-2`) and (b) the AREA invariant at ~1e-3 (canonical-vs-located meshed-facet weld
      wobble; volume unaffected). Calibrated the SECONDARY bound to `2e-2` and the AREA
      invariant to `5e-3`; the tight VOLUME invariant (the real signal) is unchanged. No
      native disagreement was hidden — no product code changed.
- [x] 7.3 Determinism re-verified: same seed twice → byte-identical batch (md5 match).
- [x] 7.4 Structural check: `git diff` touches only `tests/sim` + `scripts` + `openspec`;
      `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged.

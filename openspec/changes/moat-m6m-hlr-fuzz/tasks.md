# Tasks — moat-m6m-hlr-fuzz (MOAT M6-breadth-13)

Order: confirm the HLR service + facade → harness skeleton (facade-driven, both engines)
→ base-solid + rigid-pose + view generator → differential compare + closed-form arbiter →
classifier → coverage bar over ≥2 seeds → runner + suite SKIP + roadmap. NO change to
`src/native/**`, `src/engine/**`, `include/**` (system under test); no `cc_*` ABI change;
no tolerance weakened.

## 0. Service + facade

- [x] 0.1 Build the numsci substrate as a build sanity gate: `bash scripts/build-numsci.sh
      host && bash scripts/build-numsci.sh iossim` (both exit 0). This domain does NOT link
      numsci — the HLR path is not `CYBERCAD_HAS_NUMSCI`-gated — but the substrate build is
      the standard MOAT build check.
- [x] 0.2 Confirm in source: `src/native/drafting/orthographic_hlr.h` projects the straight
      edge set over the M0 occluder and classifies visible/hidden; `silhouette.h` traces the
      closed-form silhouette (`n·view = 0`) of cylinder/sphere/cone/frustum/`Kind::Torus`
      and declines freeform; the `cc_*` facade exposes `cc_hlr_project` / `cc_set_engine` /
      `cc_solid_extrude` / `cc_solid_revolve` / `cc_solid_revolve_profile` /
      `cc_rotate_shape_about` / `cc_translate_shape` / `cc_drawing_free`; both engines return
      `CCDrawing` in the same drawing-plane basis.

## 1. Harness skeleton (facade-driven, both engines)

- [x] 1.1 Create `tests/sim/native_hlr_fuzz.mm` with the RNG helper (splitmix64 →
      xoshiro256**, `FUZZ_SEED` from argv/env, fixed default `0x4D6F617436`), a self-contained
      fp64 vec3 + Rodrigues rotate, the coverage tally, the `Verdict`, and the coverage-summary
      + `std::_Exit` epilogue, mirroring the sibling fuzzers.
- [x] 1.2 Include ONLY `cybercadkernel/cc_kernel.h` (no native C++ headers) — this harness
      drives the public facade like `native_hlr_parity.mm`.

## 2. Base-solid + rigid-pose + view generator (built identically under both engines)

- [x] 2.1 `genCase` for six families — BOX / NGON prism (`cc_solid_extrude`), CYLINDER /
      CONE-or-frustum / SPHERE (`cc_solid_revolve` / `cc_solid_revolve_profile`), and FREEFORM
      (B-spline-meridian revolve → `Kind::BSpline` bands, the decline probe).
- [x] 2.2 Random rigid pose (`cc_rotate_shape_about` about a random unit axis + random angle,
      then `cc_translate_shape`; NO scale/mirror → exact isometry) applied IDENTICALLY under
      both engines.
- [x] 2.3 Random VIEW direction with a non-parallel up hint (least-aligned world axis), binned
      into shallow / oblique / grazing view regimes.

## 3. Differential compare + closed-form arbiter (both engines)

- [x] 3.1 Project the posed solid via `cc_hlr_project` under `cc_set_engine(0)` (OCCT HLRBRep
      oracle) and `cc_set_engine(1)` (native); compare visible/hidden 2D segment sets: exact
      counts + tight `1e-4` partition for polyhedral; bidirectional partition coverage at
      `0.08` curve tol for quadrics (the authoritative same-locus check); total length as a
      corroborating proxy (tight for polyhedral, 3% band corroborating for curved).
- [x] 3.2 Closed-form silhouette-tangency arbiter for CYLINDER (two generator lines θ* =
      `atan2(−X·d, Z·d)`) + SPHERE (great circle ⟂ view), posed by the same rigid transform
      and projected into the drawing plane; native-on-arbiter ∧ ¬oracle-on-arbiter →
      ORACLE_UNRELIABLE (native vindicated).

## 4. Classifier

- [x] 4.1 Classify AGREED / HONESTLY-DECLINED (native EMPTY or native could-not-pose → OCCT
      draws) / DISAGREED / ORACLE_UNRELIABLE / BOTH-DECLINED at FIXED tolerances (never
      widened). A grazing-cylinder length-proxy miss under a holding bidirectional partition
      → AGREED (same locus), NOT a fault.
- [x] 4.2 Coverage summary (per-family + per-view-regime); bar = `DISAGREED == 0`.

## 5. Runner + suite + roadmap

- [x] 5.1 `scripts/run-sim-native-hlr-fuzz.sh` (whole kernel + full OCCT incl. TKHLR, NO
      numsci; ≥2 default seeds `0x4D6F617436` / `0x171313C0FFEE`; fails if any seed fails).
- [x] 5.2 Add `native_hlr_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 5.3 Update `openspec/MOAT-ROADMAP.md` M6 row (breadth ×12 → ×13) with the coverage.

## 6. Gate (≥2 seeds, N≥60)

- [x] 6.1 Run both default seeds at N=60 on the booted simulator: seed `0x4D6F617436`
      60 trials → 53 AGREED / 7 HONESTLY-DECLINED / 0 DISAGREED / 0 ORACLE_UNRELIABLE;
      seed `0x171313C0FFEE` 60 trials → 50 / 10 / 0 / 0. Both PASS; every family (box/ngon/
      cylinder/cone/sphere) with ≥1 AGREED and freeform ≥1 DECLINED, all three view regimes
      exercised on each seed.
- [x] 6.2 Determinism re-verified: same seed twice → byte-identical batch (diff empty).
- [x] 6.3 A grazing-cylinder length-proxy over-fire (seed `0x171313C0FFEE` cases 45/47, visLen
      rel ≈ 3.2%) was LOCALIZED via bidirectional partition (`bi=1 v⊆v=1 o⊆v=1`) to a pure
      discretization artifact — the two engines trace the identical silhouette locus; native
      complete. Fixed by making bidirectional partition the authoritative curved gate (the
      length band is corroborating), NOT by widening any tolerance.
- [x] 6.4 Structural check: `git diff` touches only `tests/sim` + `scripts` + `openspec`;
      `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged.

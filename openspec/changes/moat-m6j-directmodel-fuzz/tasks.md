# Tasks — moat-m6j-directmodel-fuzz (MOAT M6-breadth-10)

Order: confirm the substrate → pick the domain → harness skeleton (facade-driven, both
engines) → per-op drivers + closed-form arbiters → six-way classifier → coverage bar over
≥2 seeds → runner + suite SKIP + roadmap. NO change to `src/native/**`, `src/engine/**`,
`include/**` (system under test); no `cc_*` ABI change; no tolerance weakened.

## 0. Substrate

- [x] 0.1 Build the OCCT-linked simulator numsci and host: `bash scripts/build-numsci.sh
      host && bash scripts/build-numsci.sh iossim` (both exit 0). This domain DOES link
      numsci (the native split/offset seam trace is `CYBERCAD_HAS_NUMSCI`-gated).
- [x] 0.2 Confirm in source: `NativeEngine::split_plane` / `replace_face` compose
      `freeformHalfSpaceCut` guarded by `CYBERCAD_HAS_NUMSCI`; `directmodel::projectPointOnFace`
      serves plane/cylinder/sphere and declines cone/freeform/ambiguous; the `cc_*` facade
      exposes `cc_split_plane` / `cc_replace_face` / `cc_project_point_on_face` /
      `cc_set_engine` / `cc_mass_properties` / `cc_bounding_box` / `cc_subshape_ids` /
      `cc_face_axis`.

## 1. Domain choice (direct-modeling vs section-curve vs HLR)

- [x] 1.1 Pick direct-modeling: three landed per-op parity harnesses but no fuzz domain;
      drives the real `cc_*` + `cc_set_engine` shipping path; each op has a pristine closed
      form. Document the deferral of section-curve (native side already the closed-form
      arbiter with a close parity harness) and HLR (already facade-driven).

## 2. Harness skeleton (facade-driven, both engines)

- [x] 2.1 Create `tests/sim/native_directmodel_fuzz.mm` with the RNG helper (splitmix64 →
      xoshiro256**, `FUZZ_SEED` from argv/env, fixed default `0xD3ADBEE710`), the coverage
      tally, the six-way `Verdict`, and the coverage-summary + `std::_Exit` epilogue,
      mirroring the sibling fuzzers.
- [x] 2.2 Include ONLY `cybercadkernel/cc_kernel.h` (no native C++ headers) — this harness
      drives the public facade like `native_hlr_parity.mm`.

## 3. Base-solid generator (built identically under both engines)

- [x] 3.1 `genBase` for BOX / NGON prism (`cc_solid_extrude`) and CYLINDER / CONE frustum
      (`cc_solid_revolve`), with exact closed-form geometry (base area, whole volume, bbox,
      a lateral-face probe + outward normal) recorded per case.
- [x] 3.2 Build each operand under `cc_set_engine(0)` and `cc_set_engine(1)`; a base neither
      engine builds → BOTH-DECLINED.

## 4. Per-op drivers + closed-form arbiters (both engines)

- [x] 4.1 SPLIT: `cc_split_plane` axis-aligned + oblique, random keep side; native-vs-OCCT
      keep volume/area/bbox; exact half-space keep-volume (axis-aligned box/prism) +
      partition closure `V(keep+)+V(keep−)==V(whole)` (all families).
- [x] 4.2 OFFSET: `cc_replace_face` planar cap by a signed grow/trim offset; native-vs-OCCT
      volume/area/bbox; exact `ΔV==capArea·offset` for constant-section families; cone
      offset arbitrated by native==OCCT only (no false closed form).
- [x] 4.3 PROJECT: `cc_project_point_on_face` of a random exterior point; native-vs-OCCT
      foot + distance; exact planar / cylinder-radial foot; cone-lateral → native decline.
- [x] 4.4 Measure every shape under the engine that BUILT it (guard the cross-engine
      unwrap crash); select target faces GEOMETRICALLY so both engines resolve the same
      face; accept the best servable curved wall (native seam-split cylinder/cone).

## 5. Classifier

- [x] 5.1 Six-way classify (AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE /
      BOTH-DECLINED / ORACLE_UNRELIABLE) at the FIXED tolerances; TRIM offset → AGREE,
      GROW offset (OCCT half-space-cut cannot add) → ORACLE-INACCURATE (native vindicated).
- [x] 5.2 Coverage summary (per-family + per-op AGREED/DISAGREED); bar =
      `DISAGREED==0 && ORACLE_UNRELIABLE==0 && every family + op ≥1 AGREED`.

## 6. Runner + suite + roadmap

- [x] 6.1 `scripts/run-sim-native-directmodel-fuzz.sh` (whole kernel + OCCT + numsci
      iossim; ≥2 default seeds `0xD3ADBEE710` / `0x5EC0FFEE42`; fails if any seed fails).
- [x] 6.2 Add `native_directmodel_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 6.3 Update `openspec/MOAT-ROADMAP.md` M6 row (breadth ×9 → ×10) with the coverage.

## 7. Gate (≥2 seeds, N≥60)

- [x] 7.1 Run both default seeds at N=80 on the booted simulator: seed `0xD3ADBEE710`
      39 AGREED / 29 HONESTLY-DECLINED / 0 DISAGREED / 11 ORACLE-INACCURATE / 0
      ORACLE_UNRELIABLE / 1 BOTH-DECLINED; seed `0x5EC0FFEE42` 48 / 26 / 0 / 5 / 0 / 1.
      Both PASS; every base family and every op ≥1 AGREED on each seed.
- [x] 7.2 Determinism re-verified: same seed twice → byte-identical batch (md5 match).
- [x] 7.3 Structural check: `git diff` touches only `tests/sim` + `scripts` + `openspec`;
      `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged.

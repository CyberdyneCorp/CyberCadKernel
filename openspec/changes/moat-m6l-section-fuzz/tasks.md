# Tasks — moat-m6l-section-fuzz (MOAT M6-breadth-12)

Order: confirm the substrate → pick the domain → harness skeleton (C++-boundary,
OCCT-oracle slice) → primitive + cut-plane generators → native + OCCT section drivers +
closed-form conic arbiter → four-way classifier → coverage bar over ≥2 seeds → runner +
suite SKIP + roadmap. NO change to `src/native/**`, `src/engine/**`, `include/**` (system
under test); no `cc_*` ABI change; no tolerance weakened.

## 0. Substrate

- [x] 0.1 Build host + iossim numsci: `bash scripts/build-numsci.sh host && bash
      scripts/build-numsci.sh iossim` (both exit 0). (The section slice itself needs NO
      numsci — like `native_section_parity` — but the build gate is run for the campaign.)
- [x] 0.2 Confirm in source: the native section service `cybercad::native::section::
      sectionByPlane` (`src/native/section/section.h`) is OCCT-FREE (SSI Stage-S1 header
      path + header-only topology, read-only over `src/native/{math,topology,ssi}`); it
      returns EXACT analytic loops (Circle/Ellipse analytic fields, Polygon shoelace) with
      closed-form `length()`/`area()`, and HONESTLY DECLINES (Empty/Declined) a plane
      coincident/tangent to a face, an open section, or a freeform/torus face. It is exposed
      through the additive `cc_section_plane` facade.

## 1. Domain choice (section-curve vs HLR vs STEP-export vs validity/inertia)

- [x] 1.1 Pick the section-curve fuzzer: the roadmap M6 candidate list names it; it is a
      DISTINCT, un-fuzzed domain (only a fixed-fixture parity gate exists) with a pristine
      EXACT closed-form conic arbiter, and it is STABLE — `src/native/section/` is NOT
      touched by the concurrent M3 workflow (blend/feature/boolean). Document the deferral
      of HLR (already covered by `native_hlr_parity` + inside `native_geometry_services_
      fuzz`), STEP round-trip (already `native_step_import_fuzz`), and validity/inertia
      (viable future pick; inertia is a documented native decline).

## 2. Harness skeleton (C++-boundary, OCCT-oracle slice)

- [x] 2.1 Create `tests/sim/native_section_fuzz.mm` with the RNG helper (splitmix64 →
      xoshiro256**, `FUZZ_SEED` from argv/env, fixed default `0x5EC7104FEED`), the coverage
      tally, the four-way `Verdict` (+ ORACLE_BAD), and the coverage-summary + `std::_Exit`
      epilogue, mirroring the sibling fuzzers.
- [x] 2.2 Include `native/section/native_section.h` + the OCCT oracle toolkits (like
      `native_section_parity.mm`) — this harness drives the native C++ boundary, not the
      `cc_*` facade (the facade accessor is covered by the host suite + shipping build).

## 3. Primitive + cut-plane generators (built identically native + OCCT)

- [x] 3.1 Native fixture builders (`makeBox` / `makeCylinder` / `makeSphere`) mirroring the
      parity suite; matched OCCT solids via `BRepPrimAPI_Make{Box,Cylinder,Sphere}` to the
      SAME dimensions.
- [x] 3.2 `genCase` per family: BOX (axis-aligned interior cut → rectangle), CYL_PERP
      (interior z=const → circle), CYL_AXIAL (plane through axis at random azimuth →
      rectangle), CYL_OBL (oblique cut θ∈~7°..36° with H tall enough that the ellipse fits
      the finite axial band, no arc-trim → ellipse), SPHERE (random unit normal, |d|<R →
      circle r=√(R²−d²)), DECLINE exerciser (MISS box / COINCIDENT box face / MISS sphere
      beyond pole). First `F_COUNT` trials force each family.
- [x] 3.3 Compute the closed-form analytic ground truth per case: rectangle 2(w+h)/w·h;
      circle 2πR/πR²; axial rectangle 2(2R+H)/2R·H; ellipse Ramanujan-II perimeter / πab
      (a=R/|cosθ|, b=R); sphere circle 2πr/πr².

## 4. Native + OCCT section drivers + closed-form conic arbiter

- [x] 4.1 Native: `sec::sectionByPlane(nativeSolid, cutPlane(origin, normal))` → loopCount /
      totalLength / totalArea / status.
- [x] 4.2 OCCT: `BRepAlgoAPI_Section` → edge length via `GCPnts_AbscissaPoint::Length`, loop
      count via `ShapeAnalysis_FreeBounds::ConnectEdgesToWires`, capped area via
      `BRepGProp::SurfaceProperties` on the section face(s) (verbatim `native_section_parity`
      oracle discipline).
- [x] 4.3 PRIMARY arbiter: native-vs-analytic closed-form conic — tight `1e-9` for
      straight/circular length + area; `1e-4` for the ellipse PERIMETER only (Ramanujan-II +
      OCCT integrator, the parity-proven bound, NEVER widened); area exact `πab` held to
      `1e-6`. SECONDARY: native-vs-OCCT length + area at the same family tol.

## 5. Classifier

- [x] 5.1 Four-way classify (AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE, +
      ORACLE_BAD for OCCT emitting no usable section where native+analytic cover it). Native
      Empty/Declined → HONESTLY-DECLINED. Native Ok matching analytic while OCCT does not →
      ORACLE_UNRELIABLE (native vindicated). Native Ok OUTSIDE the analytic truth, OR native
      Ok on a config it should have declined → DISAGREED.
- [x] 5.2 Coverage summary (per-family AGREED/DECLINED/DISAGREED/…); bar =
      `DISAGREED==0 && ORACLE_UNRELIABLE==0 && ORACLE_BAD==0 && every AGREE family ≥1 AGREED
      && decline exerciser ≥1 HONESTLY-DECLINED`.

## 6. Runner + suite + roadmap

- [x] 6.1 `scripts/run-sim-native-section-fuzz.sh` (OCCT-free native math TUs +
      section/topology/ssi-S1 header-only + OCCT oracle toolkits; NO numsci; ≥2 default
      seeds `0x5EC7104FEED` / `0x1CE5EC12ABC`; fails if any seed fails).
- [x] 6.2 Add `native_section_fuzz.mm` to the `run-sim-suite.sh` SKIP list.
- [x] 6.3 Update `openspec/MOAT-ROADMAP.md` M6 row (breadth ×11 → ×12) with the coverage.

## 7. Gate (≥2 seeds, N≥60)

- [x] 7.1 Run both default seeds at N=96 on the booted simulator: seed `0x5EC7104FEED`
      77 AGREED / 19 HONESTLY-DECLINED / 0 DISAGREED / 0 ORACLE_UNRELIABLE; seed
      `0x1CE5EC12ABC` 81 / 15 / 0 / 0. Both PASS; every AGREE family (BOX / CYL_PERP /
      CYL_AXIAL / CYL_OBL / SPHERE) ≥1 AGREED and the decline exerciser ≥1 HONESTLY-DECLINED
      on each seed. A third seed at N=120 also PASS (101 / 19 / 0 / 0).
- [x] 7.2 Localised the initial 2–5 apparent DISAGREEs: they were the EXACT-tangency
      sphere-pole decline case (`d == R`), where floating-point rounding lands the plane a
      sub-nanometre INSIDE the sphere and native CORRECTLY returns the true sub-micron circle
      (verified at the C++ level: `d==R` → decline; `d=R−1e-9` → correct tiny loop; OCCT
      rounds to empty). This is native being RIGHT, not wrong; the decline exerciser was
      re-scoped to only UNAMBIGUOUS declines (plane clearly missing / coincident). No
      tolerance was touched; no native disagreement was hidden; no product code changed.
- [x] 7.3 Determinism re-verified: same seed twice → byte-identical `[FUZZ]` batch (md5
      match).
- [x] 7.4 Structural check: `git diff` touches only `tests/sim` + `scripts` + `openspec`;
      `src/native`, `src/engine`, `include`, and the `cc_*` ABI are byte-unchanged.

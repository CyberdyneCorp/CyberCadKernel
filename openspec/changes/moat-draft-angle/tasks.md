# Tasks — moat-draft-angle (MOAT feature — draft angle)

Order: substrate → draft header → engine dispatch + additive facade → OCCT oracle →
host analytic gate (a) → sim native-vs-OCCT gate (b) → docs. All new native code is
header-only, OCCT-free, host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::feature`. DM1/DM2 + M0/M1/M2 byte-frozen; `cc_*` additive-only. No
tolerance weakened; a measured decline (curved base / non-planar neutral / cap face /
degenerate angle / self-intersecting) is a first-class outcome.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh host && bash scripts/build-numsci.sh iossim` (both exit 0).

## 1. Native draft verb

- [x] 1.1 `src/native/feature/draft_faces.h` — `draftFaces` derives each drafted target
      plane (pivot on the trace `L = F ∩ N`) from the ORIGINAL face geometry and applies
      it as an inward `nb::splitByPlane` trim, then re-audits the composite (watertight /
      χ=2 / oriented / shrink). `DraftFacesDecline` (NonPrismaticOrForeign /
      NonPlanarNeutral / FaceParallelToPull / DegenerateAngle / ResolveFailed).

## 2. ABI + engine dispatch

- [x] 2.1 Additive `cc_draft_faces` in `include/cybercadkernel/cc_kernel.h`;
      `IEngine::draft_faces` default `engine_unsupported`; facade wiring in
      `src/facade/cc_kernel.cpp`.
- [x] 2.2 `NativeEngine::draft_faces` → native for an all-planar body (self-verified),
      honest decline otherwise (never → OCCT); OCCT body forwards.
- [x] 2.3 `OcctEngine::draft_faces` = the `BRepOffsetAPI_DraftAngle` oracle
      (iOS syntax-checked against the OCCT sim install).

## 3. Gate (a) — host analytic (no OCCT)

- [x] 3.1 `tests/native/test_native_draft_faces.cpp` — single-side wedge
      `V₀ − ½·H·(H·tanθ)·D` fp-exact, four-side frustum, off-axis wedge invariance, +
      declines (curved base / non-planar neutral / cap face / degenerate angle). 7/7.

## 4. Gate (b) — sim native-vs-OCCT parity

- [x] 4.1 `tests/sim/native_draft_faces_parity.mm` (own `main()`) +
      `scripts/run-sim-native-draft-faces.sh` + `run-sim-suite.sh` SKIP entry: native
      draft vs OCCT `BRepOffsetAPI_DraftAngle` + `BRepGProp`
      (volume/area/watertight/topology/bbox/Hausdorff) for the wedge, four-side frustum,
      and off-axis fixtures, + the cap-face honest decline. 31/31 on the booted simulator.

## 5. Docs + structural discipline

- [x] 5.1 Update `MOAT-ROADMAP.md` (new "feature — draft angle" entry).
- [x] 5.2 `git diff src/native` OCCT-free & additive; `cc_*` additive-only; tessellator +
      boolean + analysis + exchange UNTOUCHED; DM1/DM2/M0/M1/M2 byte-frozen.

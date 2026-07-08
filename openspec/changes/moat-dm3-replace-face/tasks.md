# Tasks — moat-dm3-replace-face (MOAT M-DM, DM3 + DM4)

Order: substrate → DM3 general re-solve header → DM4 projection header → engine
dispatch + additive facade → host analytic gate (A) → sim native-vs-OCCT gate (B)
→ docs. All new native code is header-only, OCCT-free, host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::directmodel`. DM1/DM2 +
M0/M1/M2 byte-frozen; `cc_*` additive-only. No tolerance weakened; a measured
decline (tilt / non-planar / non-analytic / ambiguous) is a first-class outcome.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh host && bash scripts/build-numsci.sh iossim` (both exit 0).

## 1. DM3 general replace_face (offset/tilt)

- [x] 1.1 `src/native/directmodel/replace_face_general.h` — `replaceFaceOffsetTilt`
      derives `(o + n̂_F·offset, n̂_F)` and re-solves via DM2 `replaceFaceToPlane`;
      `ReplaceFaceGeneralDecline` (NonPlanarOrForeign / TiltNotReproduced / ResolveFailed).
- [x] 1.2 `NativeEngine::replace_face` → native for a native body (pure offset),
      honest decline otherwise; OCCT body forwards. `replace_face.h` byte-frozen.

## 2. DM4 point projection

- [x] 2.1 `src/native/directmodel/project.h` — `projectPointOnFace` (plane / cylinder /
      sphere closed-form foot + distance); `ProjectDecline` (ForeignBody / NonAnalyticFace
      / DegenerateSurface / Ambiguous).
- [x] 2.2 Additive `CCProjection` + `cc_project_point_on_face` (ABI); `ProjectionData`
      + `IEngine::project_point_on_face`; `NativeEngine` override (native / decline);
      `OcctEngine::project_point_on_face` = `GeomAPI_ProjectPointOnSurf` oracle; facade wiring.

## 3. Gate (a) — host analytic (no OCCT)

- [x] 3.1 `tests/native/test_native_replace_face_general.cpp` — pure push/pull `V₀+A·offset`
      fp-exact, off-axis face offset, + declines (tilt / curved neighbour / no-op). 6/6.
- [x] 3.2 `tests/native/test_native_project.cpp` — plane / cylinder / sphere closed-form
      feet + declines (cone / axis / centre / foreign). 9/9.

## 4. Gate (b) — sim native-vs-OCCT parity

- [x] 4.1 `tests/sim/native_dm3_dm4_parity.mm` (own `main()`) + `scripts/run-sim-native-dm3-dm4.sh`
      + `run-sim-suite.sh` SKIP entry: DM3 vs OCCT move-face oracle (volume/area/watertight/
      topology/bbox); DM4 vs `GeomAPI_ProjectPointOnSurf` (foot coords + distance). 36/36.

## 5. Docs + structural discipline

- [x] 5.1 Update `MOAT-ROADMAP.md` (DM3/DM4) + `DROP-OCCT-READINESS.md` (row 81 B→A, new project row).
- [x] 5.2 `git diff src/native` OCCT-free & additive; `cc_*` additive-only; DM1/DM2/M0/M1/M2 byte-frozen.

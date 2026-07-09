# Tasks — moat-m3af-analytic-fillet (MOAT M3, analytic-solid face fillets)

Order: substrate → fillet_face header → full_round header → engine dispatch → host
analytic gate (A) → sim native-vs-OCCT gate (B) → docs. All new native code is
header-only, OCCT-free, host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::blend`. The landed blend substrate (`fillet_edges.h`,
`curved_fillet.h`, `blend_geom.h`) + M0/M1/M2 byte-frozen; the tessellator UNTOUCHED;
`cc_*` additive-only. No tolerance weakened; a measured decline (dihedral / curved /
closed-seam / non-analytic) is a first-class outcome.

## 0. Substrate

- [x] 0.1 `bash scripts/build-numsci.sh host && bash scripts/build-numsci.sh iossim` (both exit 0).

## 1. fillet_face (round every bounding edge)

- [x] 1.1 `src/native/blend/fillet_face.h` — `fillet_face(solid, faceId, radius)`
      collects the convex planar-dihedral bounding edges of the picked planar face
      (via a `filletArc` probe) and applies `blend::fillet_edges`; `FilletFaceDecline`
      (BadInput / NonPlanarSolid / NonPlanarFace / NoConvexEdges / WeldGatesM2).
- [x] 1.2 `NativeEngine::fillet_face` → native for a native body (verified SHRINK),
      honest decline otherwise; OCCT body forwards. `fillet_edges.h` byte-frozen.

## 2. full_round_fillet (prismatic tangent-cylinder cap)

- [x] 2.1 `src/native/blend/full_round.h` — `full_round_fillet(solid, faceId)` (auto-
      detect the two longest opposite edges + across-neighbours) and
      `full_round_fillet_faces(solid, left, middle, right)` (shared seam edges); build
      the r = w/2 cap via `blend::fillet_edges` on the two seams for PARALLEL planar
      walls; `FullRoundDecline` (NonPlanar / NotParallel / Dihedral / ClosedSeam /
      NoSeams).
- [x] 2.2 `NativeEngine::full_round_fillet` / `full_round_fillet_faces` → native for a
      native body (verified SHRINK + middle-face-consumed), honest decline otherwise;
      OCCT body forwards.

## 3. Gate (a) — host analytic (no OCCT)

- [x] 3.1 `tests/native/test_native_analytic_fillet.cpp` (one suite) — full_round
      prismatic rib `(w²/2)(1−π/4)L` cap volume, middle-face consumed, auto == explicit,
      + declines (dihedral / curved). fillet_face MEASURED decline: the full-face fillet
      returns NULL (`WeldGatesM2`) because the landed multi-edge fillet welds only NON-
      adjacent edges (asserted: adjacent box-face edges never weld → corner weld gates M2)
      + declines (curved / non-planar / bad input). 5/5.
- [x] 3.2 (folded into 3.1 — single always-on native suite.)
- [x] 3.3 Register in `CMakeLists.txt` (always-on `test_native_analytic_fillet`); `ctest` green.

## 4. Gate (b) — sim native-vs-OCCT parity

- [x] 4.1 `tests/sim/native_analytic_fillet_parity.mm` (own `main()`) — cc_fillet_face
      + cc_full_round_fillet[_faces] under both engines; compare volume / area /
      watertight / Euler χ=2 / bbox / Hausdorff; assert the freeform / dihedral /
      closed-seam cases DECLINE to OCCT (id != 0 but forwarded).
- [x] 4.2 `scripts/run-sim-native-analytic-fillet.sh` (models run-sim-native-curved-fillet.sh)
      + SKIP entry in `scripts/run-sim-suite.sh`.
- [x] 4.3 Run on the booted iOS simulator; capture PASS numbers.

## 5. Docs + structural check

- [x] 5.1 Readiness rows 84–86 (`fillet_face`, `full_round_fillet[_faces]`) move the
      analytic families toward A; freeform / dihedral / closed-seam residual noted.
- [x] 5.2 `openspec/MOAT-ROADMAP.md` M3 status: analytic face-fillet slice landed.
- [x] 5.3 `git diff src/native` OCCT-free + additive; `git diff src/native/tessellate`
      EMPTY (tessellator untouched); `cc_*` unchanged. `openspec validate
      moat-m3af-analytic-fillet --strict`.

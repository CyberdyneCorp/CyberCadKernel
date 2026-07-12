# Tasks — nurbs-boolean-l3-s2 (NURBS roadmap Layer-3, slice 2)

Order: native verb → host gate (a) → sim gate (b) → docs. All new native code stays
OCCT-free and host-buildable under the substrate (`clang++ -std=c++20`,
`CYBERCAD_HAS_NUMSCI`), namespace `cybercad::native::boolean`. No `cc_*` ABI change
(internal geometry). `nurbs_plane_split.h` / `ssi_boolean.{h,cpp}` / `assemble.h` /
`face_split.h` untouched; `ssi` / `topology` / `math` READ-ONLY.

## 1. Native verb (`src/native/boolean/nurbs_curved_split.h`)

- [x] 1.1 `NurbsCurvedSplitDecline` enum (Ok / WallNotNurbs / BaseNotPlanar /
      CutterNotCurved / SeamUnusable / SeamOffSurface / SmoothSplitFailed /
      KeepFaceUnusable / CapDegenerate / WeldOpen / NotWatertight / VolumeInconsistent)
      + name fn.
- [x] 1.2 `NurbsCurvedSplitResult` — the honest witnesses (seamFidelityF, seamFidelityG,
      seamOnSurf, seamNodes, areaInside/Outside, tilingGap, watertight, enclosedVolume)
      + `ok()`.
- [x] 1.3 stage 1 TRACE — `npsdetail::makeWallAdapter(fs)` (reused) ∩ the cutter's
      `CurvedSolid::adapter()`; pick the closed / longest WLine (≥ 3 nodes).
- [x] 1.4 stage 2 FIDELITY — `npsdetail::seamFidelity` on F (reused) +
      `ncsdetail::seamFidelityOnCutter(G, seam)` on G; both round-trip below tol AND
      on-both-surfaces below tol, else `SeamOffSurface`.
- [x] 1.5 stage 3 SPLIT — `splitFaceSmoothTrim` (reused) → disk + annulus.
- [x] 1.6 stage 4 KEEP — `ncsdetail::pickKeepByMembership` via
      `ssidetail::classifyPoint(G, centroid)`; Above→inside, Below→outside; ON → decline.
- [x] 1.7 stage 5 SEW — `ncsdetail::synthCurvedCap`: a deflection-bounded planar-triangle
      fan (the S5-a `appendMouthCap` idiom) on the cutter G, outer ring = EXACT seam
      nodes, interior points ON G, oriented outward by keep side; via `assemble.h`
      `VertexPool` + `detail::triangleFace`. Plus `ncsdetail::appendKeptBase`.
- [x] 1.8 driver `nurbsFaceCurvedSplit(wall, base, cutter, side, deflection)` — admit →
      trace → fidelity gate → split → keep → sew → M0 watertight + positive-volume
      self-verify; NULL + measured decline on any failure.

## 2. Host gate (a) — closed-form (`tests/native/test_native_nurbs_curved_split.cpp` + fixture)

- [x] 2.1 `nurbs_curved_split_fixture.h` — the `Kind::BSpline` paraboloid bowl (patch
      half-width H=0.35 so only the inner seam circle is on-patch) + a native analytic
      SPHERE cutter (`build_revolution_profile` arc, translated onto the axis) + the
      closed-form lens oracle.
- [x] 2.2 GATE 0 — the NURBS surface reproduces the paraboloid to machine eps.
- [x] 2.3 GATE 1 — CUT (Below) lens volume converges monotonely to the closed form as
      the mesh refines, watertight χ=2, seam on BOTH curved surfaces (DISAGREED=0),
      tiling gap ~0; COMMON (Above) watertight + positive.
- [x] 2.4 GATE 2 — honest declines: non-curved cutter → CutterNotCurved, non-NURBS wall
      → WallNotNurbs, far sphere → a measured no-seam reason (NULL, never wrong).

## 3. Sim gate (b) — OCCT parity (`tests/sim/native_nurbs_curved_split_parity.mm` + runner)

- [x] 3.1 Reconstruct the SAME cup in OCCT (`Geom_BSplineSurface` bowl + planar lid, sewn)
      + the SAME sphere as `BRepPrimAPI_MakeSphere`.
- [x] 3.2 CUT-LENS leg vs `BRepAlgoAPI_Cut(cup, ball)` (+ closed-form cross-check);
      COMMON leg vs `BRepAlgoAPI_Common(cup, ball)`; volume/watertight/χ parity within
      the tessellation band.
- [x] 3.3 `scripts/run-sim-native-nurbs-curved-split.sh` runner (numsci iossim +
      ssi/{seeding,marching} + boolean/ssi_boolean + OCCT oracle toolkits).

## 4. CMake + docs

- [x] 4.1 Register `test_native_nurbs_curved_split` + its `CYBERCAD_HAS_NUMSCI`
      compile-definition (additive; if/endif balance preserved).
- [x] 4.2 Update `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` (§3 L3-S2 landed; stage 5
      curved↔curved sew resolved for the analytic cutter).
- [x] 4.3 Update `docs/NURBS-SCOPE.md` §4 Layer-3 row (L3-S2 landed).
- [x] 4.4 `openspec validate --all --strict` green.

## 5. Verification

- [x] 5.1 Host gate green (38/38 checks: lens closed-form + convergence + χ=2 + DISAGREED=0
      + declines).
- [x] 5.2 Sim parity green (14/14: CUT-LENS + COMMON vs OCCT `Cut`/`Common`, χ=2).
- [x] 5.3 Full host ctest green (92/92, no regression).
- [x] 5.4 `cc_*` unchanged, `src/native` OCCT-free, `nurbs_plane_split.h` /
      `ssi_boolean.{h,cpp}` / `assemble.h` / `face_split.h` / `ssi` / `topology` / `math`
      NOT modified.

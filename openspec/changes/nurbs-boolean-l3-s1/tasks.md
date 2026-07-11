# Tasks — nurbs-boolean-l3-s1 (NURBS roadmap Layer-3, slice 1)

Order: native verb → host gate (a) → sim gate (b) → docs. All new native code stays
OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::boolean`. No `cc_*` ABI change (internal geometry). `assemble.h` /
`face_split.h` untouched; `ssi` / `topology/trimmed_nurbs` / `math` READ-ONLY.

## 1. Native verb (`src/native/boolean/nurbs_plane_split.h`)

- [x] 1.1 `NurbsPlaneSplitDecline` enum (Ok / WallNotNurbs / BaseNotPlanar /
      SeamUnusable / SeamOffSurface / SmoothSplitFailed / KeepFaceUnusable /
      CapDegenerate / WeldOpen / NotWatertight / VolumeInconsistent) + name fn.
- [x] 1.2 `NurbsPlaneSplitResult` — the honest witnesses (seamFidelity, seamOnSurf,
      seamNodes, areaInside/Outside, tilingGap, watertight, enclosedVolume) + `ok()`.
- [x] 1.3 `npsdetail::makeWallAdapter(fs)` — non-rational → `makeBSplineAdapter`,
      rational → `makeNurbsAdapter` (the L3 NURBS operand front-end).
- [x] 1.4 `npsdetail::traceNurbsWallSeam(fs, P)` — NURBS analog of
      `hscdetail::traceWallSeam`; plane ParamBox sized from the wall's control-net AABB
      projected onto P; return the closed / longest WLine (≥ 3 nodes).
- [x] 1.5 `npsdetail::seamFidelity(fs, loc, seam, ...)` — the S(pcurve)==C gate:
      `S(u1,v1) == node.point` + echo the WLine on-both-surfaces residual.
- [x] 1.6 `npsdetail::pickKeepNurbs(...)` — half-space keep-side sub-face by trim
      centroid (analog of `cwcdetail::pickKeepFreeform`, on a NURBS surface).
- [x] 1.7 `nurbsFacePlaneSplit(wall, base, P, side, meshDeflection)` — the driver:
      admit NURBS wall + flat base → trace → fidelity gate → smooth-trim split → pick
      keep → keep the flat base if on the keep side → synth flat cap + weld →
      mandatory M0 watertight + positive-volume self-verify. NULL + measured reason on
      any decline; NEVER a leaky solid. Reuses `cwcdetail` / `hscdetail` helpers, no
      modification.

## 2. Host GATE (a) (`tests/native/`)

- [x] 2.1 `nurbs_plane_split_fixture.h` — a genuine NURBS-walled bowl-cup: a
      `Kind::BSpline` degree-2 bowl (clamped knots {0,0,0,1,1,1}) reproducing
      `z = a·(x²+y²)` exactly + a flat top-lid; the closed-form oracles fullVolume /
      cutVolume / commonVolume; the horizontal cut plane z=c.
- [x] 2.2 `test_native_nurbs_plane_split.cpp` GATE 0 — the NURBS surface reproduces the
      paraboloid to machine ε (the oracle is exact on THIS surface, not a fit).
- [x] 2.3 GATE 1 — CUT (Below) + COMMON (Above): each returns a verified watertight
      keep-side solid whose enclosed volume matches the closed form; DISAGREED=0 (seam
      fidelity S(u,v)==C < tol AND on-both-surfaces < tol); smooth-trim tiling < tol;
      the partition identity V(below)+V(above)=V(full).
- [x] 2.4 GATE 2 — honest declines: an above-cup plane and a non-NURBS wall each
      return NULL with a measured `NurbsPlaneSplitDecline` (no silent-wrong).
- [x] 2.5 CMake: register `test_native_nurbs_plane_split` under CYBERCAD_HAS_NUMSCI
      (macro only, no substrate include tree — like the L3 readiness gate); if/endif
      balance verified.

## 3. Sim GATE (b) (`tests/sim/`, `scripts/`)

- [x] 3.1 `native_nurbs_plane_split_parity.mm` — reconstruct the SAME operand in OCCT
      (a `Geom_BSplineSurface` degree-2 bowl trimmed by the rim + a planar lid, sewn
      watertight), cut it by `BRepAlgoAPI_Cut` for both keep sides, and assert
      native-vs-OCCT parity on volume + watertight + Euler χ within a tessellation band;
      cross-check OCCT vs the closed form.
- [x] 3.2 `scripts/run-sim-native-nurbs-plane-split.sh` — build the numsci iossim
      substrate + native math/ssi TUs + the OCCT oracle slice and run in a booted
      simulator (SIM GATE b), mirroring the walled-bowl-midwall runner.

## 4. Docs

- [x] 4.1 `docs/NURBS-SCOPE.md` §4 Layer-3 row: ❌ → first slice landed (NURBS face ∩
      plane), with the file + gate pointers.
- [x] 4.2 `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` — mark L3-S1 landed (the recipe
      is now a shipped verb + two-gate proof).
- [x] 4.3 `openspec validate nurbs-boolean-l3-s1 --strict` green; full host ctest green
      (no regression — the verb is additive and composes existing pieces).

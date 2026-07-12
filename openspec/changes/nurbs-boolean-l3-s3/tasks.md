# Tasks — nurbs-boolean-l3-s3 (NURBS roadmap Layer-3, slice 3)

Order: native verb → host gate (a) → sim gate (b) → docs. All new native code stays
OCCT-free and host-buildable under the substrate (`clang++ -std=c++20`,
`CYBERCAD_HAS_NUMSCI`), namespace `cybercad::native::boolean`. No `cc_*` ABI change
(internal geometry). `nurbs_plane_split.h` / `nurbs_curved_split.h` /
`freeform_freeform_cut.h` / `freeform_operand.h` / `freeform_membership.h` /
`smooth_trim_split.h` / `ssi_boolean.{h,cpp}` / `assemble.h` / `face_split.h` untouched;
`ssi` / `topology` / `math` READ-ONLY.

## 1. Native verb (`src/native/boolean/nurbs_freeform_split.h`)

- [x] 1.1 `NurbsFreeformSplitDecline` enum (Ok / NotAdmittedF / NotAdmittedG /
      WallFNotNurbs / WallGNotNurbs / SeamUnusable / SeamOffSurface / SmoothSplitFailedF /
      SmoothSplitFailedG / ClassifyAmbiguous / WeldOpen / NotWatertight /
      VolumeInconsistent) + name fn.
- [x] 1.2 `NurbsFreeformSplitResult` — the honest witnesses (seamFidelityF, seamFidelityG,
      seamOnSurf, seamNodes, areaInsideF/G, tilingGapF/G, watertight, enclosedVolume)
      + `ok()`.
- [x] 1.3 stage 0/1 RECOGNISE — `recogniseFreeformSolid` on both operands (reused) +
      `nfsdetail::nurbsWall` requiring EXACTLY one `Kind::BSpline` freeform wall per
      operand (else `WallFNotNurbs` / `WallGNotNurbs`).
- [x] 1.4 stage 1 TRACE — `npsdetail::makeWallAdapter(fsF)` ∩ `makeWallAdapter(fsG)` (the
      L3 NURBS front-end on BOTH walls); pick the CLOSED WLine (≥ 3 nodes).
- [x] 1.5 stage 2 FIDELITY — `npsdetail::seamFidelity` on F (reused) +
      `nfsdetail::seamFidelityOnG(fsG, locG, seam)` on G; both round-trip below tol AND
      on-both-surfaces below tol, else `SeamOffSurface`.
- [x] 1.6 stage 3 SPLIT — `splitFaceSmoothTrim` (reused) on F by `(u1,v1)` and G by
      `ffcdetail::rekeyToB(seam)` → disk + annulus on BOTH walls.
- [x] 1.7 stage 4 SELECT — `ffcdetail::pickByMembership` via `classifyPointInMesh` in the
      OTHER operand's pre-cut mesh (COMMON = both disks INSIDE the other); On/Unknown/
      wrong-side → `ClassifyAmbiguous`.
- [x] 1.8 stage 5 SEW — `ffcdetail::weldOrientationCoherent` (reused): the two NURBS caps
      share the EXACT seam nodes (bit-identical outer ring), M0 position-welds them; the
      directed-edge invariant repairs orientation (exactly one cap reversed).
- [x] 1.9 driver `nurbsFaceFreeformSplit(F, G, op, meshDeflection, analyticOpVolume)` —
      recognise → NURBS-wall gate → trace → fidelity gate → split both → select → weld →
      M0 watertight/coherent + positive-volume + UPPER-bound + TWO-SIDED self-verify; NULL
      + measured decline on any failure. (CUT leg reached-not-faked → honest decline.)

## 2. Host gate (a) — closed-form (`tests/native/test_native_nurbs_freeform_split.cpp` + fixture)

- [x] 2.1 `nurbs_freeform_split_fixture.h` — two `Kind::BSpline` degree-2 paraboloid
      bowl-cups (an UP bowl F reproducing `z = a·(x²+y²)`, a DOWN dome G reproducing
      `z = H − a·(x²+y²)`) meeting in ONE closed circle `ρ = √(H/2a)` at `z = H/2`, both
      NURBS walls trimmed by the SAME rim; the closed-form lens oracle `V = π·H²/(4a)`.
- [x] 2.2 GATE 0 — the shared seam is a CLOSED circle on BOTH NURBS walls (radius ρ on
      both `(u,v)` to ~1e-14, on both surfaces, at `z = H/2`, interior to both rims).
- [x] 2.3 GATE 0b — the closed-form oracles tile exactly (`V(F−G)+V(F∩G)==V(F)`) and the
      lens is a substantial discriminating fraction.
- [x] 2.4 GATE 1 — COMMON lens volume converges monotonely to the closed form as the mesh
      refines, watertight χ=2, consistently oriented, seam on BOTH NURBS walls
      (DISAGREED=0), tiling gaps ~0.
- [x] 2.5 GATE 2 — honest declines: null F → NotAdmittedF, non-intersecting pair →
      SeamUnusable, apex-ambiguous CUT → ClassifyAmbiguous (NULL, never wrong).

## 3. Sim gate (b) — OCCT parity (`tests/sim/native_nurbs_freeform_split_parity.mm` + runner)

- [x] 3.1 Reconstruct the SAME two cups in OCCT (each a `Geom_BSplineSurface` bowl + planar
      lid, sewn into one solid).
- [x] 3.2 COMMON leg vs `BRepAlgoAPI_Common(F, G)` (+ closed-form cross-check);
      volume/watertight/orientation/χ parity within the tessellation band.
- [x] 3.3 `scripts/run-sim-native-nurbs-freeform-split.sh` runner (numsci iossim +
      ssi/{seeding,marching} + boolean/ssi_boolean + OCCT oracle toolkits).

## 4. CMake + docs

- [x] 4.1 Register `test_native_nurbs_freeform_split` + its `CYBERCAD_HAS_NUMSCI`
      compile-definition (additive; if/endif balance preserved).
- [x] 4.2 Update `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` (§3 L3-S3 landed; stage 5
      general freeform↔freeform sew resolved for the tractable COMMON single-seam pose).
- [x] 4.3 Update `docs/NURBS-SCOPE.md` §4 Layer-3 row (L3-S3 landed).
- [x] 4.4 `openspec validate --all --strict` green.

## 5. Verification

- [x] 5.1 Host gate green (48/48 checks: seam-on-both + lens closed-form + monotone
      convergence + χ=2 + consistently-oriented + DISAGREED=0 + declines).
- [x] 5.2 Sim parity green (9/9: COMMON vs OCCT `BRepAlgoAPI_Common`, volume/watertight/
      orientation/χ, OCCT vs closed-form cross-check).
- [x] 5.3 Full host ctest green (94/94, no regression — freeform_freeform_cut + L3-S1/S2
      still pass).
- [x] 5.4 `cc_*` unchanged, `src/native` OCCT-free, `nurbs_plane_split.h` /
      `nurbs_curved_split.h` / `freeform_freeform_cut.h` / `freeform_operand.h` /
      `freeform_membership.h` / `smooth_trim_split.h` / `ssi_boolean.{h,cpp}` /
      `assemble.h` / `face_split.h` / `ssi` / `topology` / `math` NOT modified.

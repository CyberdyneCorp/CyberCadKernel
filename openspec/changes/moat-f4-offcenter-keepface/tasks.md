# Tasks — moat-f4-offcenter-keepface

## 1. Diagnose the off-center keep-face volume defect
- [x] 1.1 Reproduce the measured error (0.5% @ x=0, 7% @ ±0.03, 29% @ ±0.10) with a host probe.
- [x] 1.2 Confirm the seam trace + B2 split + kept freeform/analytic sub-faces are geometrically correct off-center.
- [x] 1.3 Isolate the defect to the cross-section CAP: watertight but `isConsistentlyOriented`=false → untrustworthy signed volume.
- [x] 1.4 Root-cause: `planarFaceFromLoop` orients by 3-D loop signed area, but the mesher forces UV-outer CCW so Forward normal = +fr.z always → the cap flips wrong.

## 2. Fix (additive, src/native/boolean, OCCT-free)
- [x] 2.1 Add `hscdetail::planarFaceFromLoopByNormal` (orient by `dot(fr.z, wantOutward)` only); leave the frozen `planarFaceFromLoop` untouched + document the NOTE.
- [x] 2.2 `halfSpaceCut` builds the cap via `planarFaceFromLoopByNormal`.
- [x] 2.3 `freeformHalfSpaceCut` self-verify: require `isConsistentlyOriented` (not just watertight).
- [x] 2.4 `slab_disjoint_cut.h` `assembleLump` caps via `planarFaceFromLoopByNormal`; per-lump + compound self-verify → `isConsistentlyOriented`.

## 3. Regression tests + two gates
- [x] 3.1 Host gate (a): off-center CUT at x∈{±0.03,±0.10}, both keep-sides, matches closed form < 1% and is consistently oriented.
- [x] 3.2 Host gate (a): disjoint slab CUT welds at the closed-form two-body volume (relerr < 1%), two-solid compound.
- [x] 3.3 Sim gate (b): off-center split-plane fixture (x=0.10) vs OCCT `MakeHalfSpace`+`Cut`+`BRepGProp`.
- [x] 3.4 Sim gate (b): disjoint slab CUT welds a two-body compound matching OCCT `BRepAlgoAPI_Cut`+`BRepGProp`.
- [x] 3.5 Full host ctest stays all-green (67/67).

## 4. ff↔ff FUSE
- [x] 4.1 Assess tractability without a tessellator weld → HONEST-DECLINE (needs a curved-annulus-to-curved-annulus outer weld; a NEW seam topology beyond the M0w closed-inner-seam weld). Not landed; documented as the sharpened next blocker.

## 5. Spec + commit
- [x] 5.1 OpenSpec change dir + `openspec validate --strict`.
- [x] 5.2 Commit to moat-feat5 (gate numbers in the message).

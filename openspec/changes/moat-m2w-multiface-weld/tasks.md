# Tasks — moat-m2w-multiface-weld

## 1. The multi-face corner-clip weld verb (additive, OCCT-free)

- [x] 1.1 Add `src/native/boolean/multi_face_weld.h` — `multiFaceCornerClip(A, seamGraph, junctionSplit, op, deflection, report)` returning the welded solid or a measured `MultiFaceDecline` (NULL).
- [x] 1.2 Classify each analytic face against the two cutting planes (keep-whole / single-plane clip via byte-frozen `hscdetail::cutAnalyticFace` / two-plane corner-clip of the straddling bottom).
- [x] 1.3 Bottom corner-clip: L-survivor (reroute the boundary around the removed quadrant through the corner `J'`) for CUT/FUSE, convex two-plane clip for COMMON.
- [x] 1.4 Synthesize the two box CAP faces (bowl seam arc half + vertical `J→J'` + bottom chord + outer vertical), sharing the arc with the wall sub-face and `J→J'` between the two caps.
- [x] 1.5 FUSE: `B`'s four non-cutting faces WHOLE + the two cutting faces NOTCHED by the cap region (rectangle-minus-notch, curved boundary = the shared arc).
- [x] 1.6 Mandatory self-verify: M0 watertight + a consistent op-volume bound (`0 ≤ V(A−B) ≤ V(A)`; `0 ≤ V(A∩B) ≤ min`; `max ≤ V(A∪B) ≤ V(A)+V(B)`); NULL on any failure.

## 2. Wire the entry point

- [x] 2.1 `freeformBooleanMultiSeam` consumes `multiFaceCornerClip` after the junction split (additive report fields: `weldOk`, `weldDecline`, `resultVolume`, `volA`, `volB`); returns the welded solid or NULL → OCCT.
- [x] 2.2 Keep the whole substrate byte-frozen (only additive edits to `multi_seam.h`).

## 3. GATE (a) — host analytic (no OCCT)

- [x] 3.1 `test_native_multi_seam.cpp`: all three ops land watertight at the closed-form corner oracle volumes (rel ≤ 2e-2 at d=0.01).
- [x] 3.2 Volume monotone-converges across a deflection sweep to ≤ 0.5% at the tightest deflection.
- [x] 3.3 The honest-decline envelope (single-cut box, non-freeform operand) still declines to NULL.

## 4. GATE (b) — sim native-vs-OCCT

- [x] 4.1 `tests/sim/native_multi_seam_freeform_boolean_parity.mm` (own `main()`): reconstruct A+B in OCCT, run `BRepAlgoAPI_Cut/Common/Fuse`, compare native on volume/area/watertight/Euler/bbox/Hausdorff + a classify batch, at d=0.01 and 0.005.
- [x] 4.2 Runner `scripts/run-sim-native-multi-seam-freeform-boolean.sh` + SKIP entry in `scripts/run-sim-suite.sh`.

## 5. Zero-regression + finalize

- [x] 5.1 `git diff src/native` OCCT-free + additive; `cc_*` unchanged; byte-frozen substrate unchanged.
- [x] 5.2 Existing host suites green (face_split, first_freeform_boolean, freeform_boolean_breadth, two_operand, multi_seam).
- [x] 5.3 Update `openspec/MOAT-ROADMAP.md` M2 status; `openspec validate moat-m2w-multiface-weld --strict`.

# Proposal — moat-f4-offcenter-keepface (F4, off-center-accurate freeform keep-face / cross-section cap)

## Why

The app's `cc_boolean` @13 freeform/mixed usage routinely cuts a curved body along a plane
that does NOT pass through the operand's symmetric center. The landed freeform half-space
CUT (`src/native/boolean/half_space_cut.h`) synthesises the keep-side solid by welding the
kept freeform sub-face, the kept analytic sub-faces, and a **cross-section cap** on the cut
plane. The cap was built with `hscdetail::planarFaceFromLoop`, which picks the cap face's
`Forward`/`Reversed` orientation from the **3-D loop signed-area sign**.

That rule is WRONG for a freshly-chained cap loop. The M0 mesher forces the UV OUTER loop
CCW regardless of the incoming edge order, so a `Forward` planar face ALWAYS meshes with
normal `+fr.z` and a `Reversed` one with `−fr.z` — independent of `loopSignedArea`. For the
cap (whose chained-loop winding is arbitrary) this flips the cap the wrong way: the welded
solid is watertight but NOT consistently oriented, so its signed `enclosedVolume` is
untrustworthy. Measured on the bowl-lidded prism fixture the keep-side volume error was
0.5% at a center cut (x=0), **7% at x=±0.03, and 29% at x=±0.10** — three oracles (host
closed-form integrator, OCCT `BRepGProp`, and the meshed enclosed volume) agreed the true
value; the native cap was the outlier. Because the self-verify only checked `isWatertight`
(not `isConsistentlyOriented`), the wrong-volume solid passed at the center by symmetric
coincidence.

This also blocked the landed disjoint/multi-lump CUT (`slab_disjoint_cut.h`), which reused
the same cap machinery: with the closed-form volume supplied its TWO-SIDED self-verify
correctly HONEST-DECLINED `VolumeInconsistent` (the ~29% over-estimate), so the disjoint cut
could only be exercised in upper-bound mode and never welded at the true two-body volume.

## What changes

- **`src/native/boolean/half_space_cut.h` (additive):** add `hscdetail::planarFaceFromLoopByNormal`,
  which orients a synthesised planar face by the mesher's ACTUAL convention — `Forward` iff
  `dot(fr.z, wantOutward) ≥ 0`, else `Reversed` — dropping the spurious `sign(area)` factor.
  The frozen `planarFaceFromLoop` is UNTOUCHED (its analytic-keep-face callers rely on its
  behavior); a NOTE documents why the cap must use the by-normal variant. `halfSpaceCut`
  builds the cross-section cap via `planarFaceFromLoopByNormal`. `freeformHalfSpaceCut`'s
  mandatory self-verify is strengthened from `isWatertight` to `isConsistentlyOriented` so a
  watertight-but-mis-wound shell (untrustworthy signed volume) is NEVER emitted.
- **`src/native/boolean/slab_disjoint_cut.h` (additive):** `assembleLump` builds the
  cross-section cap via `planarFaceFromLoopByNormal`; the per-lump and combined-compound
  self-verify are strengthened to `isConsistentlyOriented`. With each lump now consistently
  oriented and off-center-accurate, the TWO-SIDED band ACCEPTS the weld — the former
  `VolumeInconsistent` honest-decline becomes a full, volume-accurate two-body WELD.
- **ff↔ff FUSE: honest-declined (measured).** The two landed ff↔ff verbs
  (`freeform_freeform_cut.h`) weld a disk cap against an annulus along a CLOSED curved inner
  seam. A ff↔ff FUSE (`A ∪ B` of two coaxial curved cups) would instead weld the two curved
  ANNULUS regions into one outer shell across the shared closed curved seam — a genuine
  curved-annulus-to-curved-annulus outer weld, a NEW seam topology the M0w closed-inner-seam
  weld does not cover. Per the no-tessellator-change discipline this is HONEST-DECLINED with
  the measured blocker (needs a curved-annulus weld), not forced.
- Regression tests + two-gate proof (below); no `cc_*` ABI change; `src/native/**` stays
  OCCT-free.

## Two-gate proof

- **Gate (a) — host, no OCCT:**
  - `tests/native/test_native_freeform_boolean_breadth.cpp::offcentre_half_space_cut_matches_closed_form`
    — off-center CUT at x∈{±0.03,±0.10}, BOTH keep-sides, is consistently oriented and matches
    the closed-form integrator to < 1% (was up to 29%).
  - `tests/native/test_native_slab_disjoint_cut.cpp::slab_two_sided_verify_welds_at_closed_form`
    — the disjoint slab CUT WELDS at the closed-form two-body volume (relerr < 1%), a
    consistently-oriented two-solid compound.
- **Gate (b) — sim native-vs-OCCT on the booted iOS simulator:**
  - `tests/sim/native_split_plane_parity.mm` — new off-center fixture (plane x=0.10, both keep
    sides) matches OCCT `BRepPrimAPI_MakeHalfSpace`+`BRepAlgoAPI_Cut`+`BRepGProp` at rel 0.4–0.7%.
  - `tests/sim/native_slab_disjoint_cut_parity.mm` — the native disjoint CUT welds a two-body
    compound whose volume matches OCCT `BRepAlgoAPI_Cut`+`BRepGProp` at rel 0.4–0.7%.

## Impact

- Affected specs: `native-booleans` (off-center-accurate freeform half-space keep-face;
  disjoint slab CUT now welds at closed form).
- Affected code: `src/native/boolean/half_space_cut.h`, `src/native/boolean/slab_disjoint_cut.h`
  (additive only). Tessellator UNTOUCHED. No `cc_*` ABI change.

# Tasks — moat-m2b-boolean-breadth (MOAT M2b freeform↔analytic DISJOINT / multi-lump CUT)

Order: baseline capture → diagnose the tractable family → slab fixture + closed-form oracle
→ the composed verb → host analytic gate → sim native-vs-OCCT gate → complexity + byte-freeze
+ zero-regression proof → docs. All new native code stays OCCT-free and host-buildable
(`clang++ -std=c++20`), namespace `cybercad::native::boolean`. No `cc_*` ABI change. Strictly
ADDITIVE: `half_space_cut.h`, `inter_solid_seam.h`, `two_operand.h`, B1/B2/B3, M0/M1 and the
whole tessellator stay BYTE-IDENTICAL. No tolerance weakened; a correct decline is first-class.

## STATUS

LANDED (BOTH gates). `freeformSlabDisjointCut` parts a freeform-walled solid `A` with a
central all-planar SLAB `B` into TWO lumps, composing recognise[B1] → slab-pair-find →
per-lump inter-solid-seam weld → disjoint-check → TWO-SIDED self-verify. The DISJOINT
MECHANISM lands (a `Compound` of two watertight `Solid`s, matching OCCT `BRepAlgoAPI_Cut`'s
two-body topology); the frozen keep-face machinery's OFF-CENTRE volume over-estimate
(measured 29.2% over OCCT) is HONEST-DECLINED by the two-sided gate → OCCT owns the
correct-volume result. Never a wrong/leaky solid; no tolerance widened.

## Tasks

- [x] Capture baseline: existing freeform-boolean spine green; `test_native_freeform_freeform_cut` 8/8.
- [x] Diagnose the tractable family (proposal §Diagnosis): FUSE not tractable (curved-annulus
      closed-seam weld opens, measured); disjoint mechanism tractable + verifiable; picked disjoint CUT.
- [x] `tests/native/slab_disjoint_cut_fixture.h` — the bowl-lidded prism `A` + central slab
      `B` (x∈[−0.10,0.10]) + closed-form CUT-volume oracle (lumpLo+lumpHi = A−B, slab = A∩B).
- [x] `src/native/boolean/slab_disjoint_cut.h` — the additive verb: `SlabCutDecline` enum +
      `slabCutDeclineName`; `findSlabPair` (opposite-parallel straddle + containment);
      `containmentBox` (per-face half-space box for the landed seam machinery); `assembleLump`
      (aKeepFaces + `planarFaceFromLoop` cap); `buildLump` / `verifyVolume` / `overlapsAlongAxis`
      helpers; `freeformSlabDisjointCut` entry (recognise → pair → two lumps → disjoint-check →
      compound → two-sided self-verify → NULL/OCCT). OCCT-FREE, header-only.
- [x] Host GATE (a) — `tests/native/test_native_slab_disjoint_cut.cpp` (6/6): admit + pair;
      closed-form partition; DISJOINT mechanism (watertight two-solid compound); TWO-SIDED
      honest decline (`VolumeInconsistent`, measured >10% over cf); decline battery.
- [x] Sim GATE (b) — `tests/sim/native_slab_disjoint_cut_parity.mm` +
      `scripts/run-sim-native-slab-disjoint-cut.sh` (15/15): OCCT `BRepAlgoAPI_Cut` yields
      TWO solids at closed-form volume; native mechanism matches the two-body topology;
      native two-sided self-verify honest-declines (29.2% over-estimate vs OCCT); OCCT owns
      the correct result.
- [x] Cognitive complexity within backend target: entry `freeformSlabDisjointCut` = 13
      (≤ 15), all functions ≤ 13.
- [x] Structural discipline: `git diff src/native/tessellate` empty; 0 OCCT symbols in
      `src/native/**`; `cc_*` ABI unchanged; CMake change is an additive test registration only.
- [x] Zero regression: full `ctest` 66/66 pass (incl. the new host gate).
- [x] Docs: MOAT-ROADMAP.md M2 status + DROP-OCCT-READINESS.md M2 row updated.

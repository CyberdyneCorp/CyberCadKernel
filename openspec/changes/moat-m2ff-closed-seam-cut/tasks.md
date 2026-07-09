# Tasks — moat-m2ff-closed-seam-cut (MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON)

Order: baseline capture → two-bowl-cup fixture + closed-form oracle → the composed verb
→ host analytic gate → sim native-vs-OCCT gate → byte-freeze + zero-regression proof →
docs. All new native code stays OCCT-free and host-buildable (`clang++ -std=c++20`),
namespace `cybercad::native::boolean`. No `cc_*` ABI change. Strictly ADDITIVE:
`smooth_trim_split.h`, B2 `splitFace`, `freeform_operand.h`, `freeform_membership.h`,
M0/M1 and every landed boolean header stay BYTE-IDENTICAL. No tolerance weakened; a
correct decline is first-class.

## STATUS

LANDED (BOTH gates). The composed verb `freeformFreeformClosedSeamCut` runs recognise[B1]
→ trace[M1] → split[B2 smooth-trim] → classify[B3] → weld[M0] for two coaxial freeform
bowl-cups over a shared CLOSED curved seam. With the M0-rim tessellator weld in place the
two-CURVED-side closed seam welds watertight at the working deflections; the remaining
defect was an ORIENTATION bug — both survivor disk caps inherit their parent wall's
orientation (`A` opens UP, `B` opens DOWN), so the watertight shell was orientation-
INCONSISTENT (1104 same-direction duplicate seam half-edges) and its signed volume was a
locked 33.3% too small, NOT converging. FIXED in the additive boolean layer: the COMMON
assembly enforces the DIRECTED-edge invariant (`tess::isConsistentlyOriented`) and flips
exactly one cap for a coherent outward-normal boundary; the self-verify was hardened to a
TWO-SIDED volume band vs the analytic op-volume (new `VolumeInconsistent` reason). RESULT:
COMMON now WELDS a coherently-oriented lens at the closed-form volume `π·H²/(4a)=0.010053`
and CONVERGES (rel err 12.4%→6.0%→3.4% across d 0.01/0.005/0.0025); CUT honest-declines
(`ClassifyAmbiguous`) → OCCT. NO tessellator change. Host gate 8/8; sim native-vs-OCCT
gate 14/14 (native COMMON matches OCCT `BRepAlgoAPI_Common`, converging).

## 1. Baseline + fixture

- [x] 1.1 Capture the byte-frozen substrate baseline: `test_native_curved_wall_cut`
  (13/13), `test_native_two_operand_freeform_boolean` (6/6) GREEN before the change.
- [x] 1.2 Add `tests/native/freeform_freeform_cut_fixture.h`: two coaxial paraboloid
  bowl-cups (UP bowl-cup `A` + DOWN dome-cup `B`, degree-2 separable Bézier walls + flat
  lids), the real M1 seam (`ssi::trace_intersection` of the two Béziers → a CLOSED circle
  ρ = √(H/2a) at z = H/2), and closed-form oracles `volA = π·a·R⁴/2`,
  `volCommon = π·H²/(4a)`, `volCut = volA − volCommon`.

## 2. The composed verb

- [x] 2.1 `freeform_freeform_cut.h`: `FfOp`, `FfCutDecline` + `ffCutDeclineName`, and
  `freeformFreeformClosedSeamCut`, composing B1 → M1 trace (two `makeBezierAdapter`) → B2
  smooth-trim split of BOTH walls (B's seam re-keyed `(u2,v2)→(u1,v1)`) → B3 membership
  survivor selection → M0 weld + mandatory watertight + op-volume self-verify → NULL on
  any decline.
- [x] 2.2 Keep the driver + helpers within the backend cognitive-complexity band.
- [x] 2.3 ORIENTATION-COHERENCE REPAIR (`ffcdetail::weldOrientationCoherent`): the COMMON
  assembly enforces the directed-edge invariant `tess::isConsistentlyOriented` (additive
  `sameDirectionEdgeCount`/`isConsistentlyOriented` in `tessellate/mesh.h`), flipping one
  cap so the lens is a coherent outward-normal boundary; declines `NotWatertight` if no
  single flip yields a consistent 2-manifold.
- [x] 2.4 TWO-SIDED self-verify: new `FfCutDecline::VolumeInconsistent`; the welded volume
  is checked against the analytic op-volume (optional `analyticOpVolume` param) within a
  deflection-bounded band, in addition to the operand upper bound.

## 3. Host GATE (a) — analytic, no OCCT

- [x] 3.1 Both operands B1-admit with one freeform wall + one analytic lid.
- [x] 3.2 The shared seam is a CLOSED curved circle radius ρ on BOTH walls' (u,v) to
  ~1e-13, on both surfaces to the trace residual, at z*.
- [x] 3.3 The closed-form partition is consistent (`volCut + volCommon = volA`), the lens
  is substantial.
- [x] 3.4 The pipeline B2-splits BOTH curved walls (each disk area = π·ρ²).
- [x] 3.5 COMMON WELDS non-null + watertight + consistently-oriented at the closed-form
  lens volume within the deflection band, converging as deflection refines; CUT
  honest-declines to NULL (`ClassifyAmbiguous`) — never a leaky/wrong solid
  (`ff_common_welds_watertight_at_closed_form`, `ff_cut_honest_declines_never_leaky`).
- [x] 3.6 A non-operand declines `NotAdmittedA`; a non-intersecting operand declines
  `SeamUnusable`.

## 4. Sim GATE (b) — native-vs-OCCT

- [x] 4.1 `tests/sim/native_freeform_freeform_cut_parity.mm` + run script
  `scripts/run-sim-native-freeform-freeform-cut.sh`.
- [x] 4.2 OCCT reconstructs the SAME two bowl-cups (Geom_BezierSurface bowl + a planar lid
  on the shared rim edges, sewn, interior-classified outward).
- [x] 4.3 Each native seam node lies ON BOTH OCCT Bézier surfaces (BRepExtrema ≤ 1e-4,
  measured ~1e-14); OCCT `Cut` / `Common` match the closed-form volumes to ≤ 2% (Common
  EXACT); native COMMON WELDS and matches OCCT `BRepAlgoAPI_Common`+`BRepGProp` within the
  deflection band (relErr 0.124→0.060→0.034, converging) while native CUT DECLINES to NULL;
  a non-intersecting operand → native `SeamUnusable` + OCCT no-op. 14/14 PASS.

## 5. Byte-freeze + zero-regression proof

- [x] 5.1 `include/` diff EMPTY (no `cc_*` change); in `src/native` only
  `boolean/freeform_freeform_cut.h` + the additive directed-edge checks in
  `tessellate/mesh.h` changed; `smooth_trim_split.h` / `face_split.h` /
  `freeform_operand.h` / `freeform_membership.h` / the tessellator mesher/CDT/rim weld /
  M1 BYTE-IDENTICAL; existing `enclosedVolume`/`isWatertight`/`isTwoManifold` untouched.
- [x] 5.2 `src/native` has 0 new OCCT includes.
- [x] 5.3 Full host `ctest` 62/62 GREEN with zero regression.

## 6. Docs

- [x] 6.1 Update `openspec/MOAT-ROADMAP.md` M2 status: COMMON welds-at-closed-form via the
  orientation-coherence fix + two-sided volume self-verify; CUT honest-declines; both gate
  numbers (host 8/8, sim 14/14); prior "two-curved-side weld" blocker marked RESOLVED.

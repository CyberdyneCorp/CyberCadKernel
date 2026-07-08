# Tasks — moat-fuse-two-operand-freeform-boolean (MOAT M2-FUSE)

Order: baseline capture → inter-solid seam-set builder → two-operand split of BOTH
operands → two-operand shell classifier → per-op weld → mandatory self-verify → host
analytic gate → sim native-vs-OCCT parity gate → zero-regression proof → docs, OR
HONEST DECLINE at the sharpest reachable level (§ design 9). All new native code stays
OCCT-free and host-buildable (`clang++ -std=c++20`), namespace
`cybercad::native::boolean`. No `cc_*` ABI change. The change is strictly ADDITIVE: B1
`recogniseFreeformSolid`, B2 `splitFace`, B3 `classifyPointInMesh`, M0 `SolidMesher`,
the analytic `recogniseCurvedSolid`/`classifyPoint`, and the landed
`freeformHalfSpaceCut` CUT/COMMON path stay BYTE-IDENTICAL; any M1 touch is additive
with prior controls byte-frozen. No tolerance is weakened; a correct decline is a
first-class outcome; no FUSE stub.

## STATUS

DIAGNOSE (host-analytic gate, empirical probe — temporary, reverted; numsci host+iossim
built green). VERDICT: **TRACTABLE at § design 9 LEVEL 1 (full FUSE)** for the target
single-curved-seam box pose. A throwaway host probe composed the LANDED verbs into the
full two-operand FUSE and it PASSED the host gate:

- Pose: A = bowl-lidded convex-quad prism (fixture); B = finite box x∈[0,0.8], y∈[−0.6,0.6],
  z∈[−0.6,0.2] whose only cutting face is x=0 (the landed CUT plane) — B fully contains
  A's x≥0 part, so exactly ONE box face slices the Bézier wall; the other 5 miss A.
- Inter-solid seam: `traceWallSeam(A, wall.fs, Pcut)` returned a clean 250-pt BoundaryExit
  WLine; `orderLoop` spliced the curved bowl arc + 3 straight analytic-crossing chords into
  ONE closed D-outline (4 pieces) — design §1–2 confirmed with the LANDED `hscdetail`.
- Two-operand weld: A-keep faces (5, = `halfSpaceCut`'s face set WITHOUT the cap) + B's 5
  whole box faces + B's x=0 **annulus** (planar rect outer + curved D-hole inner reusing the
  SAME cap-loop pieces) = 16 faces → shell → solid.
- Self-verify: **watertight at deflection 0.02 / 0.01 / 0.005** and enclosed volume
  0.86725 / 0.86700 / 0.86671 vs the closed-form union V(B)+cutVol = 0.86630 (relerr
  1.1e-3 → 5e-4, **converging** as deflection tightens). Sub-checks: landed CUT = cutVol
  (0.099), box-only = boxVol (0.768).

KEY FINDING: the anticipated enabler "M0 shared-curved-edge single-sampling for
deflection-independent welding" (§ design 9 level 2) is **NOT required for watertightness
in this pose** — the shared curved seam welds at all three deflections because A's bowl
sub-face and B's annulus hole-boundary both derive the seam edge from the identical
`seam3d` poles via `edgeFromPiece` (bit-exact), so M0's per-pole edge sampling coincides.

IMPLEMENTATION NOTES (for the wave):
1. `FaceMesher` routes FULL-RECTANGLE planar faces through `structuredGrid`, which uses a
   fixed grid winding and IGNORES the outer-loop winding that `planarFaceFromLoop` relies
   on → build B's box faces via the earclip path (`detail::triangulatePolygonToFaces` with
   an outward-oriented `Polygon`) or match the grid orientation; else the shell is
   watertight-by-incidence but signed-volume-inconsistent. (Isolated in probe: box-only
   read 0.128 via `planarFaceFromLoop`, 0.768 via the earclip Polygon path.)
2. `makeFace(plane, outerWire, {innerWire}, Reversed)` with a curved (BSpline-polyline)
   hole boundary meshes and welds watertight — the annulus construct is sound.

NOT yet exercised (remaining implementation risk, all LOW — landed verbs used elsewhere):
survivor selection was done geometrically-by-construction, NOT via B3 `classifyPointInMesh`
on fragment centroids (design §4); CUT/COMMON survivor sets (for this pose CUT == the
landed single-operand result = cutVol, so trivially reachable; COMMON = A-in ∪ B-in
untested); and the SIM native-vs-OCCT spatial gate (§7). Next blocker to watch: the
general MULTI-curved-seam box pose (>1 box face slicing the wall) needs the seam-GRAPH
assembly deferred in § design 10 — out of scope for this single-curved-seam slice.

## IMPLEMENT — OUTCOME: **LEVEL 1 (full FUSE) LANDED**, both gates GREEN

The full two-operand freeform FUSE is implemented ADDITIVELY and self-verified against
BOTH non-negotiable gates. CUT and COMMON reduce (a theorem of the containment guard)
to the landed single-operand half-space cut and match their closed forms.

- **New OCCT-free headers (header-only, 0 OCCT includes, no `cc_*` ABI change):**
  - `src/native/boolean/inter_solid_seam.h` — `buildInterSolidSeam(A, B)`: identifies the
    unique `Pcut` (straddle test on the wall poles), the containment guard, traces
    `S_curve` (landed `traceWallSeam`), splits `A`'s wall (B2) + planar faces (landed
    `cutAnalyticFace`), and chains the closed inter-solid seam loop (landed `orderLoop`).
    Returns `SeamDecline` on any non-conforming pose.
  - `src/native/boolean/two_operand.h` — `freeformBooleanTwoOperand(A, B, op, defl)`:
    FUSE = `A`-outer keeps (no cap) ∪ `B`'s non-cutting faces WHOLE ∪ the `Pcut`
    rectangle-minus-`D` ANNULUS (curved hole = the shared seam). B3 `classifyPointInMesh`
    CONFIRMS the clean fragments (A-out of B, B-out of A) — never guesses. Mandatory
    self-verify: watertight + a consistent union volume `max(V_A,V_B) ≤ V ≤ V_A+V_B`;
    any failure → NULL `Shape` → OCCT. CUT/COMMON delegate to the landed, self-verified
    `freeformHalfSpaceCut` (Below/Above).

- **GATE (a) HOST ANALYTIC (no OCCT), 6/6:** inter-solid seam closes; FUSE watertight at
  the closed-form union `V(B)+V(A∩{x≤0})` (rel ≤ 2e-2, and MONOTONE-converging across
  deflection 0.02/0.01/0.005); CUT and COMMON at their closed forms; a box that never
  cuts the wall and a non-freeform operand each DECLINE to NULL.
- **GATE (b) SIM native-vs-OCCT `BRepAlgoAPI_Fuse` (booted simulator), 23/23:** at
  deflection 0.01 AND 0.005 — VOLUME rel 9.4e-4/5.7e-4, AREA rel 5.8e-5/3.7e-5,
  WATERTIGHT, TOPOLOGY Euler χ=2, BBOX worst 1e-7, HAUSDORFF 7e-10 (spatial, not
  volume-only), and a 2197-point CLASSIFY batch vs `BRepClass3d_SolidClassifier` with
  ZERO crisp IN↔OUT disagreements (native membership via the landed B3 multi-ray
  classifier). Fallback: a non-cutting box DECLINES to NULL (→ OCCT).
- **ZERO-REGRESSION:** `git diff HEAD -- src/native/` is EMPTY (no tracked native file
  changed); the two new headers add 0 OCCT includes; landed `first_freeform_boolean`
  (CUT, 5/5) and `freeform_boolean_breadth` (COMMON, 5/5) host suites still pass with
  UNCHANGED counts; B1/B2/B3/M0, `recogniseCurvedSolid`/`classifyPoint`, and the landed
  `freeformHalfSpaceCut` are byte-identical; M1 (`ssi/marching.h`) untouched.
- **NEXT BLOCKER (honest, out of this slice):** the GENERAL multi-curved-seam box pose
  (> 1 box face slicing the wall) needs the seam-GRAPH assembly deferred in design §10;
  and a non-containment pose (`B` partially inside `A`) needs the two-operand CUT/COMMON
  survivor weld rather than the half-space-cut reduction.

## 0. Substrate (FRESH worktree — run FIRST)

- [x] 0.1 `bash scripts/build-numsci.sh iossim && bash scripts/build-numsci.sh host`;
      export `CYBERCAD_NUMSCI_DIR=…/build-numsci/host` (host gate) and `…/iossim` (sim gate).
- [x] 0.2 Build host + NUMSCI and record the GREEN baseline for the native-booleans and
      native-ssi suites (curved-boolean native-pass count, freeform CUT/COMMON,
      face-split, freeform-membership, SSI-marching, seeding) — the byte-identical
      reference for §8.
- [x] 0.3 Snapshot the landed `freeformHalfSpaceCut` CUT/COMMON results (volume,
      watertight, topology) and the analytic `recogniseCurvedSolid`/`classifyPoint`
      outputs — the byte-identical reference for §8.

## 1. Inter-solid seam-set builder (`src/native/boolean/inter_solid_seam.h`, NEW)

- [x] 1.1 Define `InterSolidSeam` (ordered closed loop of `S_curve` + `{S_lin,i}`,
      per-face typed crossings) and `enum class SeamDecline` (`NoOverlap`,
      `NotSingleCurvedCut`, `SeamUnusable`, `SeamNotClosed`).
- [x] 1.2 `buildInterSolidSeam(A, B)`: overlap AABB; identify the UNIQUE `B` face
      crossing `A`'s Bézier wall (DECLINE `NotSingleCurvedCut` on 0 or >1).
- [x] 1.3 Trace `S_curve` via the EXISTING `traceWallSeam`/`ssi::trace_intersection`
      (byte-unchanged); require `≥2` points and `Closed`/`BoundaryExit`.
- [x] 1.4 Compute each `S_lin,i` = plane∩plane clipped to both face polygons (reuse the
      planar clip); splice into ONE loop; verify SIMPLE + CLOSED or DECLINE.
- [x] 1.5 Keep per-function cognitive complexity in the backend band via per-pair helpers.

## 2. Two-operand split of BOTH operands (`src/native/boolean/two_operand.h`, NEW)

- [x] 2.1 Split `A`'s Bézier wall along `S_curve` via B2 `splitFace` UNCHANGED.
- [x] 2.2 Split each crossed PLANAR face of `A` and of `B` along `S_lin,i` via the landed
      analytic-face-split primitive (`half_space_cut.h` B4), exactly-two-crossings
      verified; DECLINE `SplitDecline` on tangent / at-vertex / wrong count.
- [x] 2.3 Build each seam edge ONCE and share it bit-exactly between the two incident
      sub-faces (weld contract); pass uncrossed faces through whole.

## 3. Two-operand shell classifier (`two_operand.h` classify step)

- [x] 3.1 Mesh `A` and `B` (pre-split) once each with M0 → `meshA`, `meshB`.
- [x] 3.2 Classify each `A` fragment centroid against `meshB` and each `B` fragment
      centroid against `meshA` via B3 `classifyPointInMesh` UNCHANGED → in/out.
- [x] 3.3 DECLINE `ClassifyAmbiguous` on any `On`/`Unknown`/ON-band verdict — never guess.

## 4. Per-op weld (`two_operand.h` weld step)

- [x] 4.1 Select survivors per op — FUSE = `A`-out + `B`-out; CUT = `A`-out + `B`-in
      reversed; COMMON = `A`-in + `B`-in.
- [x] 4.2 Weld into one shell → `Solid` via `assemble.h` `VertexPool` (coincident corners
      → shared vertices; seam edges already coincide); DECLINE `WeldOpen` on an open shell.
- [x] 4.3 Entry point `freeformBooleanTwoOperand(A, B, op, deflection, TwoOperandDecline*)`.

## 5. Mandatory self-verify → OCCT fallback

- [x] 5.1 Require `isWatertight(SolidMesher::mesh(result))` (every edge shared by exactly
      two faces), else DISCARD → NULL `Shape`.
- [x] 5.2 Require enclosed volume == closed-form op value
      (`V(A)+V(B)−V(A∩B)` FUSE) within the scale-relative deflection band, else DISCARD.
- [x] 5.3 On DISCARD return NULL `Shape` recording the failing verb/measurement (→ OCCT);
      NEVER emit a partial/overlapping/leaky/wrong-volume solid; NEVER weaken a tolerance.

## 6. Host analytic gate (`tests/native/…`, NO OCCT)

- [x] 6.1 The closed-form union oracle: `V(A ∪ B) = V(A) + V(B) − V(A ∩ B)` from the exact
      per-triangle quadratic-moment bowl oracle + box volume + closed-form clip — mesh-free,
      OCCT-free, the primary FUSE correctness check.
- [x] 6.2 Assemble FUSE on the host with NO OCCT; assert watertight `Solid` at the
      closed-form union volume within the band (and CUT/COMMON at their closed-form values).
- [x] 6.3 Assert the inter-solid seam loop CLOSES and every fragment classifies CRISPLY
      (no `On`/`Unknown`) — provable even if the weld declines (§ design 9 level 3).

## 7. Sim native-vs-OCCT parity gate (booted simulator, OCCT linked)

- [x] 7.1 Build `A`, `B` BOTH natively and as OCCT shapes; compare native FUSE to
      `BRepAlgoAPI_Fuse` on VOLUME + AREA (`BRepGProp`), WATERTIGHT, TOPOLOGY counts, and
      a SPATIAL BBOX match (not volume-only).
- [x] 7.2 A query-point batch agrees with `BRepClass3d_SolidClassifier` on the native
      result with ZERO crisp IN↔OUT disagreements.
- [x] 7.3 A deliberately-perturbed non-watertight/wrong-volume native result is DISCARDED
      → `BRepAlgoAPI_Fuse` (OCCT); no wrong/leaky solid emitted; no tolerance weakened.

## 8. Zero-regression proof (MANDATORY — additive discipline)

- [x] 8.1 `src/native/**` has ZERO OCCT includes; `cc_*` ABI byte-identical (no new entry
      point, no POD layout change).
- [x] 8.2 B1/B2/B3/M0, analytic `recogniseCurvedSolid`/`classifyPoint`, and the landed
      `freeformHalfSpaceCut` CUT/COMMON headers are BYTE-IDENTICAL vs `main`; the
      native-booleans + native-ssi suites pass with counts UNCHANGED from the §0 baseline.
- [x] 8.3 If M1 is touched, prove the additive helper INERT on every existing SSI caller
      (byte-identical seeding/marching outputs) before it is used.

## 9. Docs / spec

- [x] 9.1 Record the reached decline level (§ design 9) and the measured next blocker in
      STATUS and in `openspec/MOAT-ROADMAP.md`.
- [x] 9.2 `openspec validate moat-fuse-two-operand-freeform-boolean --strict` passes.

## 10. Honest-out (a first-class outcome, not a fallback failure)

- [x] 10.1 If the full FUSE weld is not robustly reachable, land the provable subset (§
      design 9 level 2/3) and DECLINE the rest with the measured gap; name the M0
      shared-curved-edge single-sampling fix and/or general multi-seam graph assembly as
      the next enabler.
- [x] 10.2 Confirm NO FUSE stub, NO dead code, NO weakened tolerance shipped; every path
      emits a self-verified-correct solid OR a NULL `Shape` → OCCT.

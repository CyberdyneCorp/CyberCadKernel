# Proposal — moat-m2ms-multi-seam-boolean (MOAT M2-multiseam)

## Why

The landed M2-FUSE two-operand freeform boolean
(`src/native/boolean/inter_solid_seam.h` + `two_operand.h`) handles exactly ONE
pose: a finite box `B` positioned so that **exactly ONE** of its planar faces
(`Pcut`) slices operand `A`'s Bézier bowl wall in a **single closed seam loop**,
with every OTHER `B` face fully containing `A`'s interior-side material. Under that
containment guard `buildInterSolidSeam` identifies the unique cutting face by a
pole-straddle test (`findCuttingFace` DECLINES `NotSingleCurvedCut` on 0 or `> 1`
crossing faces), traces the single curved arc (`traceWallSeam`), and chains ONE
`orderLoop` D-outline; FUSE welds the `Pcut` rectangle-minus-`D` annulus, while
CUT/COMMON reduce — a **theorem of the single-cutting-face containment guard** — to
the landed single-operand `freeformHalfSpaceCut` (`A ∩ {x ≤ 0}` / `A ∩ {x ≥ 0}`).

That reduction is the ceiling of the single-seam slice. The landed spec names the
exact next blocker:

> "NEXT BLOCKER (honest, out of this slice): the GENERAL multi-curved-seam box pose
> (> 1 box face slicing the wall) needs the seam-GRAPH assembly deferred in design
> §10."

This change lands that seam-GRAPH slice: a finite box `B` positioned so **TWO** of
its planar faces slice `A`'s freeform wall in **two distinct inter-solid seam arcs
that MEET at a shared junction vertex** on the wall. That single generalization
breaks four assumptions the single-seam path is built on:

1. **One cutting face → a set of cutting faces.** `findCuttingFace` requires
   `nCut == 1`; the seam-GRAPH pose has `nCut == 2` whose two supporting planes share
   the box's vertical edge, and that edge pierces the bowl wall at a **junction
   vertex** `J`.
2. **One full boundary-to-boundary arc → junction-bounded partial arcs.**
   `traceWallSeam` traces each plane∩bowl chord across the WHOLE operand param box; in
   the seam-GRAPH pose each arc is only the PORTION from its Q-boundary crossing to the
   interior junction `J` — the two full chords must be clipped at `J` and consistently
   joined, or the split double-counts the corner.
3. **One `orderLoop` D-outline → a seam graph ordered at junctions.** The removed
   corner's boundary on the wall is `arcA(boundary→J)` bent to `arcB(J→boundary)`; its
   weld faces (the two box cutting sub-faces) share `J` as a **valence-≥3 vertex** —
   the arcs must be ordered consistently AROUND `J` into closed sub-face loops.
4. **CUT/COMMON no longer reduce to one half-space.** `B` now removes a **quadrant
   CORNER** of `A`, so `A − B = A ∩ (x ≤ 0 ∨ y ≤ 0)` is an **L-shaped** bowl-topped
   solid with a re-entrant corner at `J`. It is NOT a half-space of `A` and CANNOT
   delegate to `freeformHalfSpaceCut` — CUT and COMMON become genuine multi-seam welds.

The reachability was DIAGNOSED against the real fixture geometry (grounding below), NOT
assumed. **OCCT remains the oracle and the fallback**: the assembled result is admitted
ONLY if it self-verifies WATERTIGHT and matches the INDEPENDENT closed-form corner
volume; otherwise the boolean DECLINES → OCCT. If the full seam-GRAPH weld is not
robustly weldable this wave (the junction-vertex coincidence is the sharpened
fragility), the change LANDS the provable piece — the seam-graph builder + junction
computation + junction-joined B2 split proven in isolation — and returns the next
SHARPENED blocker honestly. A correct decline carrying the measured next enabler is a
first-class outcome; no partial, overlapping, leaky, or wrong-volume solid is ever
emitted, and no seam-graph stub is written.

### Grounding (DIAGNOSED on the fixture, no OCCT, no fabrication)

Operand `A` = the landed bowl-lidded convex-quad prism (`first_freeform_boolean_fixture`;
`V(A) = 0.196310`). Operand `B` = the finite box `x ∈ [0, 0.8]`, `y ∈ [0, 0.6]`,
`z ∈ [−0.6, 0.2]` (`V(B) = 0.384000`) straddling `A`'s `(+x, +y)` corner. Against the
bowl-pole hull (`x, y ∈ {−0.5, 0, 0.5}`), the pole-straddle predicate reports EXACTLY
two cutting faces — `x = 0` and `y = 0` — and the four remaining faces contain `A`.
The box's vertical edge `x = 0, y = 0` pierces the bowl wall at the junction
`J = (0, 0, 0)`. The closed-form corner clip gives `V(A ∩ B) = 0.051275`, hence
`V(A − B) = 0.145035` and `V(A ∪ B) = 0.529035`, and the removed footprint
`{(0.328, 0), (0.30, 0.32), (0, 0.30), (0, 0)}` has its re-entrant corner exactly at
`J` — confirming a genuine two-arc, one-junction seam graph.

## What Changes

1. **An additive seam-GRAPH builder** — a new OCCT-free header
   `src/native/boolean/seam_graph.h` that, GIVEN the recognised freeform operand `A`
   and the finite all-planar box `B`, identifies the **set** of cutting faces (this
   slice: EXACTLY two whose planes share a box edge), traces each curved arc via the
   EXISTING `traceWallSeam` machinery UNCHANGED, computes the shared **junction
   vertex** `J` where the two cutting planes and the wall meet, CLIPS each full chord
   at `J`, and orders the arcs at `J` into the closed corner boundary. It returns a
   typed DECLINE (no partial graph) on any non-conforming pose. It reuses the landed
   `inter_solid_seam.h` primitives (`wallWorldPoles`, `planeStraddlesWall`,
   `aabbInsidePlane`, `tracePlaneOf`) and every `hscdetail::` primitive UNCHANGED.
2. **A junction-joined freeform split** — the two junction-bounded arcs
   `arcA(boundary→J)` and `arcB(J→boundary)` concatenate into ONE boundary-to-boundary
   `WLine` bent at `J`, which B2 `splitFace` (`face_split.h`) splits in a SINGLE call
   UNCHANGED into the corner sub-face and the L-shaped survivor sub-face. `J` is an
   interior vertex of that polyline — the seam GRAPH's freeform split reduces to the
   landed one-seam B2 verb with a bent seam, no new split code.
3. **A per-op seam-graph weld** behind a new entry point
   `freeformBooleanMultiSeam(A, B, op)` in `src/native/boolean/multi_seam.h`. FUSE =
   `A`-outer L-survivor (no cap) ∪ `B`'s non-cutting faces WHOLE ∪ the two cutting
   faces' **corner-notched** sub-faces sharing the junction edge to `J` bit-exactly.
   The two cutting sub-faces and the wall's corner sub-face meet at `J` as a
   valence-≥3 vertex welded by the `assemble.h` `VertexPool` at a SINGLE shared 3-D
   junction position (built once from `J`).
4. **Genuine multi-seam CUT and COMMON** — since `B` removes a quadrant corner, CUT
   (`A − B`, L-shaped) and COMMON (`A ∩ B`, corner) are welded by the SAME seam-graph
   machinery with flipped survivor sets — they do NOT reduce to `freeformHalfSpaceCut`.
   Each is gated on its OWN closed-form volume.
5. **A MANDATORY self-verify → OCCT fallback** — before returning, the welded result
   MUST be WATERTIGHT (every edge shared by exactly two faces) AND its enclosed volume
   MUST match the INDEPENDENT closed-form value for the op within a scale-relative
   deflection band. A result that FAILS is DISCARDED (NULL `Shape` → OCCT). No
   tolerance is weakened; a leak is never emitted.
6. **The M1 SSI touch, if any, is strictly additive.** The junction-bounded arc reuses
   `traceWallSeam` and `trace_intersection` UNCHANGED; if junction-aware trimming needs
   a helper it is added ADDITIVELY to `native-ssi` behind a defaulted option with every
   prior seeding/marching control BYTE-FROZEN and proven inert on all existing callers
   before use. The landed `inter_solid_seam.h`/`two_operand.h` single-seam path,
   `recogniseCurvedSolid`/`classifyPoint`, and B1/B2/B3/M0 are consumed byte-identical.
7. **The honest-out is preserved end-to-end.** The assembler DECLINES (NULL `Shape` →
   OCCT) whenever any verb declines OR the self-verify fails, records WHICH verb
   declined and the measured gap, and — if the full seam-graph weld is not robustly
   reachable this wave — LANDS the seam-graph builder + junction computation +
   junction-joined split proven in isolation and returns the next sharpened blocker
   (the junction-vertex weld coincidence, or the general `≥ 3`-seam / branch-point
   graph). No seam-graph stub is shipped.

## Capabilities

### Modified Capabilities

- `native-booleans`: ADDS the FIRST **multi-seam (seam-graph)** two-operand freeform
  boolean — a recognised freeform operand FUSED / CUT / COMMONED with a finite box that
  slices its wall along TWO inter-solid seam arcs meeting at a junction vertex — via a
  seam-graph builder (cutting-face SET + junction computation + arc ordering), a
  junction-joined B2 split, and a per-op seam-graph weld, gated by the mandatory
  watertight + closed-form-corner-volume self-verify and an OCCT parity gate, with the
  honest decline retained and the landed single-seam `inter_solid_seam.h`/`two_operand.h`
  path byte-identical.
- `native-ssi`: ADDS (only if required for junction-aware arc trimming) a strictly
  ADDITIVE helper, proven to leave every prior seeding/marching control and every
  existing SSI parity figure byte-identical.

## Impact

- `src/native/boolean/seam_graph.h` — NEW OCCT-free header: the seam-graph builder
  (cutting-face set, junction-vertex computation, per-arc `traceWallSeam`, junction
  clip + ordering). Reuses `inter_solid_seam.h` and `hscdetail::` primitives unchanged;
  cognitive complexity kept in the backend band via per-arc / per-junction helpers.
- `src/native/boolean/multi_seam.h` — NEW OCCT-free header:
  `freeformBooleanMultiSeam(A, B, op)` composing seam-graph → junction-joined split →
  classify (B3) → per-op weld → self-verify, plus a typed `MultiSeamDecline`. Reuses
  B1/B2/B3/M0, the landed annulus/weld primitives, and `assemble.h` UNCHANGED.
- `src/native/boolean/inter_solid_seam.h`, `two_operand.h` — CONSUMED unchanged; the
  landed single-cutting-face path stays byte-identical.
- `src/native/boolean/freeform_operand.h`, `face_split.h`, `freeform_membership.h`,
  `half_space_cut.h`, `assemble.h`, `tessellate/solid_mesher.h`, `ssi/marching.h` —
  CONSUMED unchanged (B1/B2/B3/M0/M1 + `hscdetail`); `ssi/marching.h` touched ONLY if a
  strictly additive junction-trim helper is proven necessary, prior controls byte-frozen.
- `src/engine/occt` + simulator proof harness — OCCT `BRepAlgoAPI_Fuse`/`_Cut`/`_Common`
  oracle for the SIM parity gate (volume / area / watertight / topology / spatial BBOX /
  point classification). OCCT is referenced ONLY here; `src/native/**` stays OCCT-free.
- **Zero-regression discipline (mandatory).** The landed `inter_solid_seam.h`,
  `two_operand.h`, `freeformHalfSpaceCut`, `recogniseCurvedSolid`/`classifyPoint`, and
  B1/B2/B3/M0/M1 MUST be byte-identical; the native-booleans and native-ssi suites MUST
  pass with counts unchanged from the pre-change baseline; `src/native/**` MUST keep ZERO
  OCCT includes; the `cc_*` ABI MUST be unchanged (internal boolean behaviour).
- **Out of scope (declines, documented not faked):** cutting-face sets of size `≥ 3`;
  junction vertices of valence `> 3` or multiple junctions; branch-point / non-transversal
  / tangent inter-solid contact; a non-box finite operand with curved faces; a
  freeform↔freeform multi-branch SSI seam. No `cc_*` ABI change; no CyberCad app change;
  no OCCT linked into `src/native/**`; no seam-graph stub.

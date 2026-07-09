# Design — moat-m2b-boolean-breadth (MOAT M2b freeform↔analytic DISJOINT / multi-lump CUT)

## 1. The reachable pose

`A` = a recognised freeform-walled solid with EXACTLY one freeform (Bézier) wall (the B1
domain shared with `freeformHalfSpaceCut` / `inter_solid_seam`) — the bowl-lidded convex-
quad prism. `B` = a finite all-planar SLAB (axis-aligned box) positioned so a PAIR of its
OPPOSITE parallel faces (`P_lo`, `P_hi`) each slice fully across `A`'s freeform wall, and
its four other faces contain `A`. Then

    A − B  =  (A ∩ {beyond P_lo, −axis})  ⊎  (A ∩ {beyond P_hi, +axis})

is TWO disconnected lumps — the disjoint result. This is a THEOREM of the pose (the four
non-cutting faces contain `A`, so `B` removes only the band between `P_lo` and `P_hi`).

## 2. Why the inter-solid-seam weld, not the half-space cut

Two enablers can build "A restricted to one plane's outer side, capped":
- `freeformHalfSpaceCut` (standalone) — MEASURED off-centre-INACCURATE: its meshed volume
  matches the closed form only for a cut through the operand's symmetric centre (relerr
  0.5% at x=0), degrading to 29% at x=±0.10 and even declining `NotWatertight` at some
  positions. Three independent oracles (closed-form integrator, dense grid, and the
  two-operand FUSE union) agree the true value; the standalone lump does not.
- `buildInterSolidSeam` + `hscdetail::planarFaceFromLoop` — the SAME machinery the landed
  two-operand FUSE uses, MEASURED off-centre-reliable (the FUSE union matches its closed
  form to 0.0–0.1% at box faces x∈{−0.10,−0.05,0,+0.10} and for oblique faces 0–30°).

The verb therefore builds each lump via `buildInterSolidSeam`: for each slab face it
synthesises a large half-space CONTAINMENT box sharing that face's plane (interior toward
the slab), so the seam builder sees EXACTLY one cutting face + containment, returns
`aKeepFaces` (A restricted to the outer side, no cap) + the closed `capLoop`, and
`assembleLump` closes it with `planarFaceFromLoop(capLoop)` on the trace plane.

## 3. The disjoint compound + self-verify

The two lumps are `ShapeBuilder::makeCompound({lumpLo, lumpHi})`. The frozen M0
`SolidMesher` meshes every Face of a Compound and `isWatertight`/`enclosedVolume` operate
on the combined mesh, so two disjoint closed 2-manifolds satisfy the every-edge-used-twice
invariant and their signed-tetrahedra volumes sum. The self-verify:

1. each lump WATERTIGHT (else `NotWatertight`);
2. the two lumps DISJOINT — their world AABBs must NOT overlap along the slab axis (else
   `NotDisjoint`: the slab did not fully separate `A`, so the result is not two bodies);
3. the combined compound WATERTIGHT;
4. `0 < V ≤ V(A)` (upper bound), and — when `analyticCutVolume` is supplied — within a
   deflection-bounded TWO-SIDED band `min(0.5, 0.02 + 3·d)·V_cf` of it (else
   `VolumeInconsistent`).

## 4. The measured honest decline

On the reachable operand the two slab faces are necessarily OFF-CENTRE, so the frozen
keep-face machinery over-estimates each lump's volume; the combined compound measures 0.177
vs the closed-form / OCCT 0.137 (**+29.2%**), well beyond the band. The TWO-SIDED gate
returns NULL → OCCT `BRepAlgoAPI_Cut`. This is the load-bearing no-silent-wrong invariant:
the disjoint compound is topologically real, but its off-centre VOLUME is not trusted, so
it is HONEST-DECLINED rather than emitted. The upper-bound-only mode (no closed form) ships
the compound for the engine's own volume self-verify (the `native_boolean.h` architecture:
"self-verify is the engine's job").

## 5. Orientation-coherence note

The consumed `hscdetail::planarFaceFromLoop`/aKeepFaces weld a WATERTIGHT solid that is not
always `tess::isConsistentlyOriented` (a latent property of the frozen verb, which self-
verifies on watertight + volume, not directed-edge coherence). The disjoint verb therefore
gates on WATERTIGHT + the two-sided VOLUME band + the DISJOINT-component invariant — the
achievable, honest self-verify for this composition — rather than re-imposing a coherence
gate the enabler does not meet. It never fabricates a pass.

## 6. Two-gate verification

- Host GATE (a), OCCT-free: `test_native_slab_disjoint_cut.cpp` — admit+pair; closed-form
  partition tiles exactly; DISJOINT mechanism welds a watertight two-solid compound; the
  two-sided self-verify honest-declines the off-centre over-estimate (measured >10% over);
  decline battery (non-operand, non-slab box).
- Sim GATE (b), OCCT oracle: `native_slab_disjoint_cut_parity.mm` — OCCT `BRepAlgoAPI_Cut`
  parts `A` into TWO solids at the closed-form volume; native mechanism matches the
  two-body topology + watertight; native two-sided self-verify honest-declines with the
  measured 29.2% over-estimate vs OCCT; OCCT owns the correct-volume result.

## 7. Additivity

New: `src/native/boolean/slab_disjoint_cut.h`, `tests/native/slab_disjoint_cut_fixture.h`,
`tests/native/test_native_slab_disjoint_cut.cpp`, `tests/sim/native_slab_disjoint_cut_parity.mm`,
`scripts/run-sim-native-slab-disjoint-cut.sh`, one additive CMake test registration.
`half_space_cut.h`, `inter_solid_seam.h`, `two_operand.h`, B1/B2/B3, M0/M1 and the whole
tessellator are BYTE-IDENTICAL. No `cc_*` ABI change; 0 OCCT includes under `src/native/**`.

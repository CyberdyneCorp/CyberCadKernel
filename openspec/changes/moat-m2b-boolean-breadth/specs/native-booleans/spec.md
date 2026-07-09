# native-booleans

## ADDED Requirements

### Requirement: Freeform↔analytic DISJOINT (multi-lump) CUT welds a two-body compound and honest-declines an off-centre wrong volume

The native boolean layer SHALL provide an additive OCCT-free header-only verb
`freeformSlabDisjointCut(A, B, deflection, why, analyticCutVolume)`
(`src/native/boolean/slab_disjoint_cut.h`, namespace `cybercad::native::boolean`) that
parts a recognised freeform-walled solid `A` with a finite all-planar SLAB `B` (an
axis-aligned box) into TWO disconnected bodies — the FIRST native freeform boolean whose
result is a disjoint multi-lump solid (the outcome OCCT returns as a compound of two
solids, and the landed planar/curved verbs treat as a degenerate NULL). The verb SHALL:

1. recognise `A` via B1 `recogniseFreeformSolid` requiring EXACTLY one freeform wall (else
   `NotAdmittedA`), and require `B` to be an all-planar solid with ≥ 4 faces (else
   `NotPlanarB`);
2. locate the UNIQUE pair of OPPOSITE parallel `B` faces that BOTH straddle `A`'s freeform
   wall pole hull, with every OTHER `B` face containing `A` (the slab passes fully across),
   declining `NoSlabPair` / `NotContained` otherwise;
3. build EACH lump — `A` restricted to one slab face's OUTER side, closed by the
   cross-section cap on that face — through the LANDED, off-centre-reliable inter-solid-seam
   machinery (`buildInterSolidSeam` + `hscdetail::planarFaceFromLoop`), NOT the off-centre-
   inaccurate standalone `freeformHalfSpaceCut` cap, declining `SeamDeclined*` / `LumpOpen*`
   on failure;
4. assemble the two lumps into a `Compound` of two `Solid`s and run a MANDATORY self-verify:
   both lumps AND the combined compound mesh WATERTIGHT (else `NotWatertight`); the two
   lumps genuinely DISJOINT — their world AABBs separated along the slab axis, so the result
   has two connected components (else `NotDisjoint`); the enclosed volume positive and
   `≤ V(A)` (else `VolumeInconsistent`); and — when `analyticCutVolume` is finite and
   positive — within a deflection-bounded TWO-SIDED band of it (else `VolumeInconsistent`).

On ANY failure the verb SHALL return a NULL `Shape` (→ OCCT `BRepAlgoAPI_Cut`) with a typed
`SlabCutDecline`, SHALL NEVER emit a leaky / overlapping / single-lump / wrong-volume
result, and SHALL NOT weaken any tolerance to force a pass. The verb SHALL be strictly
ADDITIVE: it SHALL NOT modify `half_space_cut.h`, `inter_solid_seam.h`, `two_operand.h`,
B1/B2/B3, M0/M1 or the tessellator (all BYTE-IDENTICAL), and SHALL NOT change the `cc_*`
ABI. On the reachable bowl-lidded-prism operand the DISJOINT MECHANISM SHALL weld a
watertight two-solid compound, while the two-sided self-verify SHALL HONEST-DECLINE the
byte-frozen keep-face machinery's OFF-CENTRE volume over-estimate (measured materially above
the OCCT / closed-form value) → NULL → OCCT, so the oracle owns the correct-volume result.

#### Scenario: DISJOINT mechanism welds a watertight two-body compound; two-sided verify honest-declines the off-centre wrong volume (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism `A` and a central axis-aligned slab `B`
  (x∈[−0.10,0.10]) whose two opposite faces slice fully across `A`'s Bézier wall, plus the
  closed-form CUT-volume oracle `V(A∩{x≤−0.10}) + V(A∩{x≥+0.10})`
- WHEN `freeformSlabDisjointCut(A, B, d)` runs WITHOUT the closed form across a deflection
  sweep, and `freeformSlabDisjointCut(A, B, d, why, V_cf)` runs WITH it
- THEN the no-closed-form call returns a NON-NULL `Shape` with `why == Ok` that is a
  `Compound` of EXACTLY TWO `Solid`s, meshes WATERTIGHT, and has `0 < V ≤ V(A)`; the
  with-closed-form call returns a NULL `Shape` with `why == VolumeInconsistent` (the
  off-centre keep-face over-estimate, measured > 10% above `V_cf`) → OCCT, NEVER a
  wrong/leaky solid; and a non-operand `A` declines `NotAdmittedA` while a box that does not
  straddle `A`'s wall declines `NoSlabPair`

#### Scenario: OCCT parts A into two solids and the native mechanism matches while its off-centre volume honest-declines (sim, native-vs-OCCT)

- GIVEN the SAME `A` reconstructed in OCCT (sewn 6-face Bézier-bowl prism) and the SAME slab
  `B` as a `BRepPrimAPI_MakeBox`, on a booted iOS simulator
- WHEN OCCT `BRepAlgoAPI_Cut(A, B)` (the ORACLE) is computed and the native
  `freeformSlabDisjointCut` is run in upper-bound mode and in two-sided mode
- THEN OCCT `Cut` yields a compound of EXACTLY TWO solids whose total volume equals the
  closed form to ≤ 2%; the native upper-bound mechanism returns a `Compound` of TWO
  watertight `Solid`s (matching OCCT's two-body topology); and the native two-sided
  self-verify HONEST-DECLINES `VolumeInconsistent` because its meshed volume exceeds OCCT's
  correct value beyond the deflection band (measured ~29% over) → OCCT owns the
  correct-volume result; the native path NEVER emits a wrong/leaky solid

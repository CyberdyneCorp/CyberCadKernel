# Proposal — moat-m2b-boolean-breadth (MOAT M2b freeform↔analytic DISJOINT / multi-lump CUT)

## Why

The app's `cc_boolean` @13 forwards freeform/mixed booleans to OCCT (the A-envelope M2
residual, docs/APP-MIGRATION-READINESS.md rows 94/147). Every landed M2 freeform boolean
returns exactly ONE connected solid — `half_space_cut.h` (one lump), `two_operand.h` /
`multi_seam.h` (a curved prism welded against a box, one lump), `curved_wall_cut.h` /
`freeform_freeform_cut.h` (curved caps, one lump). But a real curved-part edit routinely
PARTS the body — a slot / channel / cut-through that SEPARATES the solid into TWO pieces,
which OCCT returns as a compound of two solids. No native verb produces that outcome; the
landed planar BSP explicitly treats a disjoint result as a degenerate NULL
(`native_boolean.h`: "a cut that removes everything"). This change lands the first native
freeform boolean whose RESULT is TWO disconnected bodies, and — where the frozen enabler
cannot deliver a correct off-centre volume — HONEST-DECLINES with a sharply measured
blocker.

## Diagnosis (why THIS pose family)

Three candidates were diagnosed against the frozen M0 tessellator + landed enablers:

- **freeform↔freeform FUSE** (CUT+COMMON landed): MEASURED NOT tractable. The union of the
  two bowl-cups needs two CURVED ANNULI to weld across the shared closed curved seam; that
  curved-annulus↔curved-annulus closed-seam weld does NOT close (watertight=0 in every
  cap-flip combination at d∈{0.01,0.005,0.0025}) — the same tessellator-weld blocker the
  landed ff-CUT declines on. Needs a new tessellator weld (owned by a concurrent track).
- **disjoint / multi-lump result** (this change): tractable — the DISJOINT MECHANISM
  (a `Compound` of two watertight `Solid`s) welds and matches OCCT's two-body topology; the
  tail is the frozen keep-face machinery's off-centre volume accuracy (declined, measured).
- **curved∪/∩analytic beyond single-seam/corner/curved-wall**: the landed two-operand FUSE
  already welds off-centre and oblique box faces (measured exact); a through-box FUSE needs
  new opposite-face multi-seam machinery (unlanded).

The disjoint result is the most app-relevant tractable new TOPOLOGICAL outcome and is
verifiable vs OCCT (`BRepAlgoAPI_Cut` returns a two-solid compound; `BRepGProp` the volume).

## What

An ADDITIVE OCCT-free header-only verb `freeformSlabDisjointCut(A, B, deflection, why,
analyticCutVolume)` (`src/native/boolean/slab_disjoint_cut.h`, namespace
`cybercad::native::boolean`) that parts a recognised freeform-walled solid `A` with a
finite all-planar SLAB `B` (an axis-aligned box) whose TWO opposite parallel faces both
slice fully across `A`'s freeform wall (the slab strictly interior to `A` along its axis):

```
A − B  =  (A ∩ {beyond the low slab face})  ⊎  (A ∩ {beyond the high slab face})
```

Each lump is `A` restricted to one slab face's OUTER side, closed by the cross-section cap
on that face, assembled through the LANDED, off-centre-RELIABLE inter-solid-seam machinery
(`buildInterSolidSeam` + `hscdetail::planarFaceFromLoop` — the SAME weld the landed
two-operand FUSE uses; the standalone `freeformHalfSpaceCut` cap is deliberately NOT used,
being off-centre-inaccurate). The two lumps become a `Compound` of two `Solid`s.

Mandatory self-verify → OCCT fallback (never a wrong/leaky result): (a) both lumps + the
combined compound WATERTIGHT; (b) the two lumps genuinely DISJOINT (AABBs separated along
the slab axis — two connected components); (c) `0 < V ≤ V(A)`, and — when the closed-form
op-volume is supplied — within a TWO-SIDED deflection-bounded band of it. Any failure →
NULL `Shape` → OCCT `BRepAlgoAPI_Cut`, with a typed `SlabCutDecline`. No tolerance widened.

## Result (both gates green)

- **Mechanism LANDED**: the verb welds a WATERTIGHT `Compound` of EXACTLY TWO `Solid`s
  (host + sim: `solids=2`, matches OCCT `BRepAlgoAPI_Cut`'s two-body topology).
- **Off-centre volume HONEST-DECLINED (measured)**: the byte-frozen keep-face machinery
  over-estimates an OFF-CENTRE cross-section's volume (measured **29.2%** over OCCT's
  correct value: native 0.177 vs OCCT/closed-form 0.137 at the ±0.10 slab), so the
  TWO-SIDED self-verify returns NULL → OCCT (the oracle owns the correct-volume result).
  The upper-bound-only path ships the topologically-correct two-body compound for the
  engine's own volume self-verify (`native_boolean.h` architecture).

Strictly ADDITIVE: `half_space_cut.h`, `inter_solid_seam.h`, `two_operand.h`, B1/B2/B3,
M0/M1 and the whole tessellator stay BYTE-IDENTICAL; no `cc_*` ABI change; 0 OCCT includes
under `src/native/**`.

## Sharpened next blocker

Closing the disjoint CUT to a WELD (not a decline) on this operand needs an off-centre-
accurate freeform keep-face/cap synthesis — the frozen `hscdetail` keep-face is volume-
exact only for a cut through the operand's symmetric centre (measured relerr 0.5% at x=0,
7% at ±0.03, 29% at ±0.10). That is a `boolean/` enabler upgrade (an off-centre cap +
keep-face reflatten), independent of the tessellator; it also unblocks a correct-volume
single off-centre `freeformHalfSpaceCut`.

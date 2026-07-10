# native-booleans

## ADDED Requirements

### Requirement: Off-center-accurate freeform half-space keep-face / cross-section cap

The native boolean layer SHALL synthesise the cross-section CAP of a freeform half-space CUT
so that the welded keep-side solid is CONSISTENTLY ORIENTED (a coherently-wound closed
2-manifold) at ANY cut-plane offset, not only for a cut through the operand's symmetric
center. The cap SHALL be oriented by the tessellator's actual convention — a `Forward` planar
face meshes with normal `+fr.z` and a `Reversed` face with `−fr.z` (the mesher forces the UV
outer loop CCW independent of the incoming edge order) — via an additive helper
`hscdetail::planarFaceFromLoopByNormal` in `src/native/boolean/half_space_cut.h` that chooses
`Forward` iff `dot(fr.z, wantOutward) ≥ 0` else `Reversed`. The frozen `planarFaceFromLoop`
(consumed byte-identically by the analytic-keep-face callers) SHALL remain unchanged.

`freeformHalfSpaceCut`'s mandatory self-verify SHALL require `tess::isConsistentlyOriented`
(not merely `tess::isWatertight`), so a watertight-but-mis-wound shell — whose signed
`enclosedVolume` is untrustworthy — is NEVER emitted; any failure returns a NULL Shape
(→ OCCT fall-through). This path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

#### Scenario: Off-center half-space cut matches the closed-form integrator (host)

- GIVEN the bowl-lidded convex-quad prism operand and a cut plane x = c with c ∈ {+0.03, +0.10, −0.03, −0.10}, native engine active and no OCCT
- WHEN `freeformHalfSpaceCut` is invoked for BOTH keep-sides (Above and Below)
- THEN each result SHALL be a consistently-oriented watertight solid whose meshed enclosed volume matches the closed-form ∫∫ (H0 + a(x²+y²)) dA over the clipped footprint to within the deflection band (< 1%, down from the frozen cap's up to 29% off-center over-estimate)

#### Scenario: Off-center half-space cut matches OCCT (simulator)

- GIVEN the same prism operand reconstructed in OCCT (sewn 6-face solid) and the cut plane x = 0.10, on a booted iOS simulator
- WHEN the native `splitByPlane` (over `freeformHalfSpaceCut`) is run for both keep-sides and compared to OCCT `BRepPrimAPI_MakeHalfSpace` + `BRepAlgoAPI_Cut` + `BRepGProp`
- THEN the native piece SHALL be watertight with Euler characteristic 2 and its volume and area SHALL match OCCT within the fixed 2% deflection-bounded band (measured rel 0.4–0.7%), and the two keep-sides SHALL partition the operand

### Requirement: Freeform↔analytic DISJOINT (multi-lump) CUT welds a two-body compound at the closed-form volume

The native `freeformSlabDisjointCut` verb (`src/native/boolean/slab_disjoint_cut.h`) SHALL
WELD a recognised freeform-walled solid `A` parted by a finite all-planar SLAB `B` into a
`Compound` of EXACTLY TWO disconnected `Solid`s at the closed-form two-body CUT volume, using
the off-center-accurate cross-section cap (`hscdetail::planarFaceFromLoopByNormal`) and a
per-lump / combined-compound self-verify that requires `tess::isConsistentlyOriented`. When
the closed-form volume is supplied, the mandatory TWO-SIDED band SHALL ACCEPT the weld iff the
combined enclosed volume lies within the deflection-bounded band of it (the former off-center
over-estimate that forced a `VolumeInconsistent` decline is eliminated). Any failure
(non-consistent lump, non-disjoint, off-band volume) SHALL return a NULL Shape → OCCT
`BRepAlgoAPI_Cut`. No leaky / overlapping / single-lump / wrong-volume result is EVER emitted;
no tolerance is weakened. This path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

#### Scenario: Disjoint slab CUT welds at the closed-form two-body volume (host)

- GIVEN the bowl-lidded prism `A` parted by a central axis-aligned slab `B` (x∈[−0.10,+0.10]) and the closed-form CUT volume V(A∩{x≤−0.10}) + V(A∩{x≥+0.10}), native engine active and no OCCT
- WHEN `freeformSlabDisjointCut(A, B, deflection, why, closedFormVolume)` is invoked at deflection ∈ {0.006, 0.008, 0.010}
- THEN the verb SHALL return a consistently-oriented watertight `Compound` of EXACTLY TWO `Solid`s, disjoint along the slab axis, whose combined enclosed volume matches the closed form to within the deflection band (< 1%), with decline `Ok`

#### Scenario: Disjoint slab CUT matches OCCT BRepAlgoAPI_Cut (simulator)

- GIVEN the same `A` and slab `B` reconstructed in OCCT on a booted iOS simulator, where `BRepAlgoAPI_Cut(A, B)` yields a compound of two solids at the closed-form volume
- WHEN the native `freeformSlabDisjointCut` is run with the closed-form volume supplied
- THEN the native result SHALL be a consistently-oriented two-solid compound whose meshed volume matches OCCT `BRepGProp` within the fixed 2% band (measured rel 0.4–0.7%)

### Requirement: Freeform↔freeform FUSE falls through to OCCT

The native boolean layer SHALL NOT natively FUSE (`A ∪ B`) two coaxial curved-cup operands
in this slice. A ff↔ff FUSE requires welding the two curved ANNULUS regions into one outer
shell across the shared CLOSED curved seam — a curved-annulus-to-curved-annulus outer weld, a
seam topology beyond the landed M0w closed-inner-seam (disk↔annulus / disk↔disk) weld and
requiring a new tessellator weld the drop-OCCT discipline forbids touching here. The FUSE case
SHALL therefore fall through to OCCT `BRepAlgoAPI_Fuse` and SHALL NEVER emit an unverified or
partial native solid.

#### Scenario: Two coaxial curved cups fall through to OCCT for FUSE

- GIVEN two coaxial curved bowl-cup operands whose walls meet in one closed curved seam, native engine active
- WHEN a FUSE (`A ∪ B`) is requested
- THEN no native ff↔ff FUSE verb SHALL produce a result; the operation SHALL fall through to OCCT `BRepAlgoAPI_Fuse`, and no native void SHALL be handed to OCCT

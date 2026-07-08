# Design — moat-m2w-multiface-weld

## The pose (measured)

`A` = the bowl-lidded convex-quad prism (one Bézier wall over footprint quad `Q`, four
planar side walls, planar bottom at `z=−H0=−0.5`). `B` = the corner box
`x∈[0,0.8], y∈[0,0.6], z∈[−0.6,0.2]`. `B` removes exactly `A ∩ {x≥0, y≥0}`. Probed values:

- The seam graph: two orthogonal iso-parametric arcs `arc0` (`u=½`, i.e. `x=0`) and `arc1`
  (`v=½`, i.e. `y=0`), junction `J=(0,0,0)` on both planes (`junctionPlaneResidual=0`).
- The junction split: seam chord `E=(0,0.30,0.036) … J=(0,0,0) … X=(0.328,0,0.043)` (183
  nodes, `jIdx=90`); corner UV area `0.097491 = uvCornerArea()`.
- Footprint straddles both planes (`footprintStraddlesBothPlanes()==true`) — genuinely
  multi-face: `x=0` also clips the bottom + walls Q2Q3 (top) and Q1Q2 (right); `y=0` clips
  the bottom + walls Q1Q2 and Q3Q0-family. Bottom `J' = (0,0,−0.5)`.

## Why the weld is reachable (and needs no B2/M0 change)

The three ops are each a bag of faces whose coincident boundaries share BIT-EXACT 3-D
geometry, so the M0 mesher position-welds the shell watertight (weld tol
`max(0.5·deflection, 1e-7)` — generous — and shared curved edges fuse via the M0 EdgeCache
keyed by endpoints + midpoint). The load-bearing shared geometry:

- The bowl seam arc halves `E→J` (`arc0`, on `x=0`) and `J→X` (`arc1`, on `y=0`) are the
  SAME degree-1 B-spline poles the wall sub-face (`splitFaceJunction`) uses — reconstructed
  identically (surface eval at the seam UV nodes with the analytic `J` override) — so the
  cap / notched-box faces weld to the wall survivor along the arc.
- The vertical `J→J'` edge is shared by the two caps (CUT/COMMON) or the two notched box
  faces (FUSE). The bottom chords `J'→E'`, `J'→X'` are shared with the bottom sub-face. The
  outer verticals `E'→E`, `X'→X` are shared with the clipped side walls' cut edges.

Every genuinely-new face is planar and built from `hscdetail::Piece`s via the byte-frozen
`orderLoop` + `planarFaceFromLoop`; the only curved edges are the seam arc halves, reused
verbatim. Nothing in B2/M0/M1 changes.

## Face inventory per op

- **CUT** (8 faces): bowl L-survivor · bottom L-survivor · walls Q0Q1, Q3Q0 whole · walls
  Q1Q2 (`y≤0`), Q2Q3 (`x≤0`) clipped · x-cap (outward `+x`) · y-cap (outward `+y`).
- **COMMON** (6 faces): bowl corner · bottom corner · walls Q1Q2 (`y≥0`), Q2Q3 (`x≥0`)
  clipped · x-cap (outward `−x`) · y-cap (outward `−y`).
- **FUSE** (≈16 faces): CUT's A-faces (no caps) · `B`'s four non-cutting faces WHOLE · `B`'s
  two cutting faces NOTCHED by the cap region.

## Self-verify (the honest gate)

Native has no closed-form oracle, so the verb self-verifies by (i) M0 watertight and (ii) a
consistent op-volume bound from the operand + box meshes. The TIGHT closed-form corner
oracle and the partition identities live in the host gate; the sim gate is native vs OCCT.
Measured results (enclosed volume, rel to closed form): CUT 4.6e-3, COMMON 3.1e-3, FUSE
1.3e-3 at d=0.005; all watertight, all monotone-converging.

## Honest declines

`NoStraddlingBottom` (no analytic face straddles both planes — pose guard), `LoopOpen` (a
synthesized boundary does not chain closed), `WallClipFailed`, `NotWatertight`,
`VolumeInconsistent`. Any decline → NULL → OCCT `BRepAlgoAPI_*`. Never a wrong solid.

## Alternatives considered

- Sequential half-space cuts + native solid union: rejected — the union of two solids sharing
  a PARTIAL face is itself a general boolean the kernel does not have natively; it trades the
  (solved) hand-weld for an unsolved union.
- Feeding the bent joined seam to byte-frozen B2 `splitFace`: refuted in the prior wave
  (`RebuildMismatch`); the junction-aware split is the reason this weld is reachable.

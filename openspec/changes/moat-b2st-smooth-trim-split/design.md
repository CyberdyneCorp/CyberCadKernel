# Design — moat-b2st-smooth-trim-split (MOAT M2b / B2 smooth-trim)

## 1. Problem

B2 `splitFace` (`face_split.h`) requires the seam to CROSS the trimmed outer loop
EXACTLY twice (`crossings == 2`) — an open chord partitioning a convex straight-edged
loop. A **closed smooth seam interior to the face** (a horizontal plane ∩ a bowl/dome
cap → a circle; a plane ∩ a curved wall → a ring) has ZERO boundary crossings, so B2
declines `CrossingsNot2`. This is the roadmap's named "B2 smooth-trim (closed/circular
wall) generalisation."

## 2. Approach — an additive sibling verb

Mirror `splitFaceJunction` (`junction_split.h`): a NEW header
`smooth_trim_split.h::splitFaceSmoothTrim`, leaving B2 `splitFace` BYTE-FROZEN. A
closed interior seam partitions the face into:

- `faceInside` — the disk the seam ENCLOSES (outer wire = the seam loop);
- `faceOutside` — the parent MINUS the disk (parent outer wire + the seam as a HOLE).

`areaInside + areaOutside == parentArea` exactly, and the M0 mesher's existing keep
rule (inside outer AND outside every hole, `trim.h` UVRegion) meshes both with NO
tessellator change.

## 3. Why one straight edge per polyline segment (the load-bearing decision)

The traced seam is a fine polyline (241 nodes for the fixture circle). Representing it
as a single degree-1 B-spline edge (or two half-arc edges) DEFEATS the M0 edge
discretizer (`edge_mesher.h`): that discretizer is curvature-driven and treats a
degree-1 B-spline as "straight in 3D" (`edgeStraight3d`/`edgeSegments` → `minSegs`),
so it samples the whole arc at only a few chords — the MEASURED failure was the disk
meshing to ~half its area (0.065 vs the true 0.126). The FAITHFUL representation is one
short STRAIGHT edge per polyline segment: each segment is genuinely straight in 3D, so
the discretizer samples it exactly at its two endpoints, and the concatenation
reproduces the full polygon. Each segment edge is built ONCE and laid on both sub-faces
with opposite orientation → the seam is their bit-exact shared boundary and welds
watertight. This lives ENTIRELY in the new verb; the tessellator is untouched.

## 4. Self-verify (SAME strict `SplitOptions` tolerances as B2, never weakened)

1. Closed interior loop: ≥ 3 distinct nodes; 0 seam×boundary crossings over the closed
   polygon; every node strictly inside the outer loop; a simple polygon.
2. Non-degenerate: enclosed disk AND annulus both above the scale-relative area floor.
3. Rebuild tiling: reflatten both rebuilt sub-faces; `faceOutside` has EXACTLY one hole;
   its OUTER reflattens to the parent (the boundary wire is REUSED verbatim → straight
   pcurves reflatten exactly); the seam reflattens BIT-IDENTICALLY as the disk's outer
   and the annulus's hole (the watertight-share invariant); the independent tiling
   residual `|parent − (disk + (annulusOuter − hole))|` collapses to machine ε. For a
   genuinely CURVED seam the coarse reflattened chord area legitimately differs from the
   fine combinatorial area, so the gate is the SHARE invariant (disk == hole cancels),
   NOT a fixed-density area match — comparing coarse reflatten to the fine area would be
   the wrong test. Any failure → typed DECLINE (`RebuildMismatch` / `TilingGap`).

## 5. Verification (two-gate discipline, OCCT the oracle)

- **Gate A (HOST, no OCCT).** Fixture: quad-trimmed Bézier bowl `z = a·r²` sliced by the
  horizontal plane `z = c` → the CLOSED CIRCLE `r = ρ = √(c/a)` (the real S3 trace,
  interior to the quad). Asserted: seam-is-closed-circle-on-surface; B2 still declines
  the same seam (contrast); tiling to machine ε (3.3e-16); disk area = closed-form
  `π·ρ² = 0.125664` (measured 0.125648, rel 1.3e-4); sub-faces mesh and their areas
  converge MONOTONICALLY to the true curved area (rel 6.2e-3 → 4.2e-3 → 1.07e-3 at
  d = {0.02, 0.01, 0.005}); honest-decline battery.
- **Gate B (SIM native-vs-OCCT) — honest scope.** A closed interior trim has no clean
  `BRepAlgoAPI` face-split oracle (it needs a splitter / section-wire reconstruction,
  not a solid boolean). The closed-form partition grounds Gate A; the sim `BRepAlgoAPI`
  parity lands with the downstream weld verb that CONSUMES `splitFaceSmoothTrim` (the
  freeform half-space cut of a dome by a horizontal plane), where the whole solid is
  reconstructed in OCCT and cut by `BRepAlgoAPI_Cut`.

## 6. Honest declines (measured, first-class)

`SeamTooShort` (< 3 nodes), `SeamNotClosed` (endpoints far apart), `SeamNotInterior`
(the seam crosses/touches the boundary — an open chord is B2's job), `SelfIntersecting`
(a non-simple loop), `DegenerateSubRegion`, `TilingGap`, `RebuildMismatch`,
`NoOuterLoop`. No partial/leaky split is ever emitted.

## 7. Sharpened next blocker

The verb hands `faceInside`/`faceOutside` to the M0 weld. The remaining work for the
END-TO-END curved-wall freeform boolean whose seam is a closed/circular smooth curve is
the WELD verb that consumes this split (a horizontal-plane half-space cut of a
dome-capped solid: split the freeform cap by `splitFaceSmoothTrim`, split the analytic
side walls, synthesize the flat circular cross-section cap, weld + watertight
self-verify) and its sim `BRepAlgoAPI_Cut` parity gate.

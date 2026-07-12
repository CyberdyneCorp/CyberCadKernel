# Design — nurbs-thicken-shell-trim

## Single-face trim (`thickenTrimmed`)

The interpenetration of a single thickened face is exactly the offset FOLD locus: the offset
map `O(u,v) = S + d·N` becomes singular where a principal Jacobian factor `1 + d·κᵢ` goes
non-positive (the offset crosses a centre of curvature and folds back through `S`). Wave-E's
`offsetSurfaceTrimmed` already computes this on a dense analysis grid (first/second
fundamental forms → `κ₁,κ₂`) and returns the maximal fold-free axis-aligned parameter
rectangle `[keptU0,keptU1]×[keptV0,keptV1]`.

`thickenTrimmed` composes it:

1. `offsetSurfaceTrimmed(S, d, tol)` → `{trimmed, keptU0..keptV1, status}`.
2. Fold-free everywhere (`trimmed == false`) → delegate to `thickenSurface` verbatim, so the
   result is BYTE-IDENTICAL (same offset call, same domain, same assembly). This satisfies
   the passthrough oracle.
3. Partially folded → assemble the closed shell over the KEPT rectangle only (`assembleShell`
   with `dom = kept`). The folded strip is cut at the fold locus; the shell re-closes over the
   valid region, watertight and self-intersection-free. Volume is the true trimmed volume.
4. Degenerate normal / whole-domain fold → honest-decline (empty solid).

`assembleShell` is the existing thicken body, factored to take an explicit parameter
rectangle. The full-domain caller (`thickenSurface`) passes the whole knot domain, reproducing
the historical output exactly.

## Multi-face slab-overlap trim (`shellTrimmed`)

At a dihedral seam the two offset caps diverge and are bridged by a mitre. The historical
`thickenPatches` uses the extend-to-meet apex `X = S + (nA+nB)·d/(1 + nA·nB)` (the point on
both offset planes). For a SHARP corner this apex spikes far past the faces
(`α = |d|/(1 + nA·nB)` grows as the interior angle shrinks), and the mitre panel then
interpenetrates the offset caps.

`shellTrimmed` (`buildShell(trimOverlap = true)`):

- **Detect** — per dihedral seam, `slabsOverlap(nA, nB, d, extentA, extentB)` compares the
  apex extension `α·‖nA+nB‖` against the per-node face extent (distance from the seam node to
  the opposite parametric boundary). Overshoot ⇒ the extend-to-meet corner cannot fit between
  the caps ⇒ the seam is re-closed with a BISECTOR CHAMFER (direct `oA → oB` bridge along the
  offset-edge midpoint) whose ridge lies strictly between the two offset caps and cannot spike
  out. Apex vertices are only welded for non-overlapping seams (so an unused apex chain never
  orphans a vertex and breaks χ).
- **Verify** — after closure, a robust non-adjacent triangle-pair PIERCING scan
  (`hasSelfIntersection`) confirms no two non-adjacent triangles cross. A DIHEDRAL seam's
  outer-boundary wall grazes the adjacent cap near the corner by construction (a shallow,
  boundary-hugging contact that is not a solid defect); a genuine slab interpenetration
  pierces DEEP into a cap interior. The scan uses a 5%-of-edge parametric slack so it ignores
  the former and catches the latter, and a scale-relative coplanar skip so two coplanar skin
  triangles never read as "piercing".
- **Decline** — if a residual DEEP interpenetration remains (the chamfer cannot resolve a
  cap-through-cap crossing), `SelfIntersecting` honest-decline with an empty solid.

`buildShell(trimOverlap = false)` gates every trim branch off, reproducing the historical
`thickenPatches` output bit-for-bit (the passthrough regression asserts byte-identity).

## Why not gate on a generic self-intersection scan

A full Separating-Axis triangle-overlap scan false-positives on the coplanar cap-to-cap and
wall-to-cap adjacency inherent to any welded seam. The edge-vs-triangle PIERCING test with a
scale-relative coplanar skip and a parametric slack is the robust primitive: it reads a clean
welded shell as self-intersection-free while catching a true transverse cap penetration.

## Honest residual

Cap-through-cap interpenetration at a sharp concave wedge needs the offset caps themselves
trimmed at their mutual intersection curve (a CSG-scale boolean), out of scope here.
`shellTrimmed` honest-declines those inputs rather than emitting a self-intersecting solid.

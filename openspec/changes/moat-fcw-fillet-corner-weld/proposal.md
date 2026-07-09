# Proposal — moat-fcw-fillet-corner-weld (spherical fillet-corner weld → full-face fillets)

## Why

`cc_fillet_face(body, faceId, radius)` — round EVERY edge bounding a picked planar face
— was the last app-facing M3 blend that hard-declined every native body and kept the
LGPL engine on its critical path. Its native path was wired + self-verified
(`fillet_face.h`), but the full-face fillet on a planar solid HONESTLY DECLINED
(`WeldGatesM2`): rounding a CLOSED corner-sharing edge loop needs a SPHERICAL corner
patch where two edge-fillet cylinder strips meet at a solid corner, and the sequential
`fillet_edges` welds only NON-adjacent edge sets — its per-edge arc facets share no
seam vertices at a shared corner, so the corner region opens / self-intersects
(MEASURED: a box top-face loop → V=1010/1011 vs 1000, not watertight).

This is a curved↔curved seam reconciliation — distinct from the landed M0 welds
(curved↔flat closed-inner-seam pin; curved↔analytic-flat rim). At a corner where 2+
mutually-adjacent convex edges are filleted at the same radius r, the corner needs a
SPHERE of radius r centred at the trihedral offset point, trimmed to the triangle
bounded by the tangent great-circle arcs where it meets the incident cylinder strips.

## What changes

- **New OCCT-free header `src/native/blend/fillet_corner.h`** — `fillet_corner(solid,
  faceId, radius, deflection)` rebuilds the filleted solid as ONE planar-facet soup:
  the trimmed face F (inset to the tangent lines), the side faces set back to the
  side-tangent lines, per-edge tangent-cylinder strips, and — the crux — a SPHERICAL
  corner patch + flat corner ledge at every shared corner. The weld is EXACT because
  the sphere centre S_i lies on BOTH incident cylinder axes (it is the point at
  distance −r from F and the two side planes, which is exactly the two constraints
  defining each cylinder axis), so the cylinder strip's end arc and the sphere patch's
  leg are the SAME quarter great-circle. ONE canonical `arcSample` (great-circle slerp)
  is consumed bit-identically by both incident faces of every shared arc, so the seam
  vertices coincide exactly and `assembleSolid` welds them watertight at ANY deflection
  — PURELY in the assembly/blend layer, NO tessellator change. A mandatory self-verify
  (consistently-oriented watertight closed 2-manifold + a two-sided SHRINK volume
  bound, via `tess::isConsistentlyOriented` / `enclosedVolume`) gates it; declines
  (`FilletCornerDecline`) for a curved solid, non-planar face, non-convex edge, a
  NON-perpendicular wall (the ledge is non-planar → out of this weld's exact scope), an
  oversized radius (adjacent corner spheres overlap), or a self-verify miss.
- **`fillet_face.h`** — now tries `fillet_corner` FIRST (the closed corner-sharing
  loop) and falls back to the sequential `fillet_edges` for a non-corner-sharing
  subset; only if BOTH return NULL does it decline `WeldGatesM2` → OCCT.
- **`NativeEngine::fillet_face`** is UNCHANGED (it already routes native bodies to
  `nblend::fillet_face` behind the existing facade seam, gated by the SAME
  `blendResultVerified` SHRINK self-verify) — the corner weld lands automatically.

## Impact

- The landed blend substrate (`fillet_edges.h`, `curved_fillet.h`, `blend_geom.h`,
  `corner_chamfer_weld.h`) and the M0/M1/M2 tessellator remain **byte-frozen**;
  `fillet_corner.h` is an additive sibling and `fillet_face.h` gains an additive
  first-choice branch.
- `src/native/**` stays OCCT-free (descriptive comments only).
- The tessellator (`src/native/tessellate/`) is consumed as-is, UNTOUCHED — the weld
  needs no mesher change (assembly-layer canonical-arc sharing), so the byte-identical
  gate is trivially satisfied (zero tessellator diff).
- `cc_*` ABI unchanged (native path only, behind the existing facade seam).
- Readiness: `cc_fillet_face` moves to A for the prism-cap analytic family; the
  freeform / non-perpendicular-wall / dihedral full-round residual stays OCCT.

## Scope (honest, MEASURED)

Native only when the solid is all-planar, the picked face is planar, every bounding
edge is a convex planar dihedral fitting radius r, AND every incident side wall is
PERPENDICULAR to the face (a prism cap — the case where the corner ledge is planar and
the weld is exact). Verified vs the OCCT oracle on the booted iOS simulator: watertight,
χ=2, volume matches OCCT to <5e-3 (and the exact rolling-ball closed form to <1e-3 —
the native volume is CLOSER to the ideal than OCCT's setback-corner shape), area <2e-2,
bbox exact. OCCT's corner-blend shape differs from the pure sphere-octant by ~r locally
(a modeling-convention gap, like the landed chamfer triple-corner), so a vertex
Hausdorff sees that O(r) corner-region difference — reported for transparency, NOT a
geometry error in the mass/topology/bbox oracle properties.

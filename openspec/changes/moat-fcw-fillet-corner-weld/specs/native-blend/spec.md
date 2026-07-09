# native-blend

## ADDED Requirements

### Requirement: Native spherical fillet-corner weld (full-face fillets land)

The engine SHALL provide a NATIVE, OCCT-free spherical fillet-corner weld
(`blend::fillet_corner(solid, faceId, radius, deflection)`) that rounds EVERY convex
planar-dihedral edge bounding a picked planar face, welding the per-edge tangent-
cylinder strips together with a SPHERICAL corner patch (a sphere of radius `radius`
centred at the trihedral offset point, distance −r from the face and both incident side
planes) at every shared corner. `blend::fillet_face` SHALL consume this weld (trying it
before the sequential `fillet_edges` fall-back), so `cc_fillet_face` on a native prism
cap LANDS natively.

The weld SHALL be EXACT along every shared corner arc: because the sphere centre lies on
BOTH incident cylinder axes, the cylinder strip end arc and the sphere patch leg are the
SAME great-circle arc; the engine SHALL sample both with ONE canonical routine so the
seam vertices coincide bit-identically and the result welds watertight at ANY deflection
through the SAME `assembleSolid` path a native prism / boolean uses. The weld SHALL be
built PURELY in the assembly/blend layer — the tessellator SHALL NOT be changed.

The engine SHALL restrict the native weld to the EXACT scope: an all-planar solid, a
planar picked face, every bounding edge a convex planar dihedral fitting `radius`, AND
every incident side wall PERPENDICULAR to the face (a prism cap, where the corner ledge
is planar). A candidate SHALL be accepted ONLY under a mandatory self-verify: a
CONSISTENTLY-ORIENTED watertight closed 2-manifold (directed-edge orientation coherence,
not merely undirected watertightness) AND a two-sided SHRINK volume bound
(`0 < V < V(original)` — a convex fillet only removes material). This path SHALL remain
OCCT-free and SHALL NOT change the `cc_*` ABI.

A curved solid, a non-planar picked face, a non-convex / curved / ≠2-face bounding edge,
a NON-perpendicular wall, an oversized radius (adjacent corner spheres overlap along an
edge), or any self-verify miss SHALL DECLINE to a NULL result with a MEASURED reason
(`FilletCornerDecline`) and the engine SHALL fall through to OCCT
`BRepFilletAPI_MakeFillet`. The engine SHALL NEVER hand a native void to OCCT, SHALL
NEVER emit an inverted / leaky / partial solid, and SHALL NEVER widen a tolerance.

#### Scenario: A full-face fillet on a box prism cap lands watertight at the closed form (host)

- GIVEN a native box `[0,L]³` and its top face, with the native engine active and no OCCT
- WHEN `cc_fillet_face(B, id(topFace), r)` is invoked
- THEN the native op SHALL return a CONSISTENTLY-ORIENTED watertight solid whose enclosed volume converges (as the deflection refines) to the closed form `L³ − [r²L(4−π) − 4r³ + (4/3)π r³]` AND SHALL be strictly less than `L³` (material removed)

#### Scenario: Every prism face (top / bottom / side) and a non-rectangular cap weld (host)

- GIVEN a native prism (rectangular or triangular cap), with the native engine active and no OCCT
- WHEN `blend::fillet_corner` is invoked on the +Z, −Z, a side, or the non-rectangular cap face at a fitting radius
- THEN each SHALL return a CONSISTENTLY-ORIENTED watertight solid with material removed (`FilletCornerDecline::Ok`)

#### Scenario: Native full-face fillet matches the OCCT oracle on the simulator (sim parity)

- GIVEN a native box built through the `cc_*` facade under the native engine, and the SAME box + `cc_fillet_face` under the OCCT oracle engine
- WHEN both round the top face at radius r and are tessellated
- THEN the native result SHALL be watertight with Euler χ=2, its volume SHALL match the OCCT oracle to a relative 5e-3 (and the closed form to 1e-3), its area to 2e-2, and its bounding box to the OCCT oracle exactly (the O(r) corner-blend-convention Hausdorff difference is reported for transparency, not gated)

#### Scenario: Out-of-scope inputs honestly decline to OCCT (host)

- GIVEN a native solid with a curved face, or a non-planar picked face, or a zero/oversized radius, or a NON-perpendicular wall, with the native engine active and no OCCT
- WHEN `blend::fillet_corner` is invoked
- THEN it SHALL return a NULL result with a measured reason (`NonPlanarSolid` / `NonPlanarFace` / `BadInput` / `RadiusTooLarge` / `NotPerpWall` / `NotConvexEdge` / `VolumeInconsistent`) AND SHALL NOT emit a solid (the engine falls through to OCCT)

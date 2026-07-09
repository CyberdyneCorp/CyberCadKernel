# native-blends

## ADDED Requirements

### Requirement: Full-face chamfer weld of a picked planar face, oracle-matched or DECLINE

The native blend library SHALL provide an OCCT-free, header-only full-face chamfer weld
verb (`src/native/blend/chamfer_face.h`, `chamfer_face`) — the planar sibling of
`fillet_face` — that, GIVEN an all-planar `Solid`, a 1-based `mapShapes(Face)` face id,
and a symmetric setback `distance`, chamfers EVERY convex planar-dihedral edge bounding
the picked planar face into ONE watertight solid. It SHALL do so by collecting those
bounding edges — keeping an edge only when the SAME `detail::filletArc` convex-fit guard
`fillet_face` uses accepts it on the two faces meeting at it (so a concave /
curved-neighbour / ≠2-face / oversized edge is silently skipped) — and assembling the
landed convex-corner weld `chamfer_corner` over that edge loop. Because a single face's
edge loop is a set of 2-edge DIHEDRAL corners (never a TRIPLE — the third edge through
each corner vertex runs off the OTHER, unpicked faces), the loop welds watertight and the
`chamfer_corner` triple-corner oracle-gap decline is unreachable here.

`chamfer_face` SHALL be a strictly ADDITIVE SIBLING: it SHALL NOT modify `chamfer_edges`,
`corner_chamfer_weld` (`chamfer_corner`), `fillet_edges`, `fillet_face`, `full_round`,
the M0 tessellator, or any landed weld path, and it SHALL consume `blend_geom.h`'s
`PlanarModel` / `facePlane` / `edgeEnds` / `facesOnEdgeInSoup`, `fillet_edges.h`'s
`detail::filletArc`, and `corner_chamfer_weld.h`'s `chamfer_corner` BYTE-IDENTICAL. It
SHALL introduce no `cc_*` ABI surface and no engine-glue change (there is no OCCT
`chamfer_face` engine method to mirror; the verb is arbitrated at the native-blend layer
by the two gates directly). It SHALL remain OCCT-free and keep its per-function cognitive
complexity within the backend band.

`chamfer_face` SHALL return the result of its assembled `chamfer_corner` — which runs the
mandatory self-verify (the welded result meshes WATERTIGHT and its volume is
`0 < V < V(original)`) — and SHALL return a NULL Shape carrying a typed measured DECLINE,
and NO solid, when: the solid is null or the distance is non-positive (`BadInput`); the
solid carries a curved face (`NonPlanarSolid`); the picked face is not planar
(`NonPlanarFace`); no bounding edge is a convex planar dihedral fitting the setback
(`NoConvexEdges`); or the corner weld's watertight / shrink self-verify fails
(`WeldFailed`).

Because the chamfer is EXACT planar geometry, `chamfer_face` SHALL match the OCCT oracle
(`BRepFilletAPI_MakeChamfer` adding every edge of the picked face) to machine ε on a
box/prism face — the removed volume equals the inclusion-exclusion closed form
`V_removed = 2·d²·L − 4·d³/3` for a cube face of side `L` and setback `d`. No tolerance
SHALL be weakened to force a match.

#### Scenario: Every face of a cube chamfers watertight at the exact closed-form volume (host, no OCCT)

- GIVEN a 10×10×10 box built on the host with NO OCCT linked
- AND a picked planar face and a symmetric setback `d`
- WHEN `chamfer_face(box, faceId, d)` is called on each of the six faces
- THEN each returns a NON-null solid with decline `Ok`
- AND the solid meshes WATERTIGHT under the native `SolidMesher`
- AND its enclosed volume equals `L³ − (2·d²·L − 4·d³/3)` to `1e-3` (959.5 at d=1.5,
  981.333 at d=1), matched across a setback sweep {0.5, 1.0, 2.0, 3.0}

#### Scenario: A full-face chamfer matches OCCT MakeChamfer to fp64 on the simulator (native-vs-OCCT)

- GIVEN a 10×10×10 box built both natively and under OCCT on a booted iOS simulator
- WHEN `chamfer_face` chamfers each cube face natively AND
  `BRepFilletAPI_MakeChamfer` chamfers every edge of the corresponding OCCT face at the
  same setback
- THEN the native enclosed volume equals the OCCT `BRepGProp` volume equals the closed
  form to `1e-6` (fp64) for every face and every swept setback
- AND the native result is watertight

#### Scenario: An out-of-domain input honestly declines to OCCT (never a wrong solid)

- GIVEN a picked planar face and a setback at least half the face extent (the chamfer
  would consume the whole face), or a curved solid, or a non-planar picked face, or a
  non-positive distance
- WHEN `chamfer_face` is called
- THEN it returns a NULL Shape with the matching typed decline (`NoConvexEdges` or
  `WeldFailed` for the oversized setback; `NonPlanarSolid`; `NonPlanarFace`; `BadInput`)
- AND emits no solid, so the engine falls through to the OCCT `BRepFilletAPI_MakeChamfer`
  oracle

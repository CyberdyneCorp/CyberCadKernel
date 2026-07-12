# native-blend

## ADDED Requirements

### Requirement: NURBS chamfer generator (setback curves + ruled bevel face) on analytic substrates

The native blend library SHALL provide, in an OCCT-free header
(`src/native/blend/chamfer_edge_nurbs.h`, namespace `cybercad::native::blend::chamfer_nurbs`),
a chamfer GENERATOR that replaces a dihedral edge between two adjacent faces with a FLAT BEVEL face.
Given the two faces as analytic substrates (plane, cylinder, or cone) and the shared edge as a
polyline of stations (each with a point, unit tangent, and the outward normals of both faces), it
SHALL trace the two SETBACK CURVES — one per face, at the prescribed setback measured ALONG that
face's surface (the geodesic/normal offset), perpendicular to the edge, into the material — and loft a
ruled (Piegl & Tiller) chamfer face `R(t,τ) = (1−τ)·cA(t) + τ·cB(t)` between them. It SHALL expose
three entry points: `chamfer_edge_symmetric(faceA, faceB, edge, d)` (each face set back `d`),
`chamfer_edge_asymmetric(faceA, faceB, edge, d1, d2)` (faceA set back `d1`, faceB set back `d2`), and
`chamfer_edge_distance_angle(faceA, faceB, edge, d, angleDeg)` (faceA set back `d`; the faceB leg
`d·tan(angleDeg)` so the bevel makes `angleDeg` with faceA). Each SHALL return a `ChamferResult` with
the two setback curves, the ruled chamfer face triangles, a planarity witness, and a decline reason.
The routine SHALL be OCCT-free (0 OCCT/Geom/BRep/TK references), SHALL NOT modify the byte-frozen
`chamfer_edges.h`, and SHALL NOT change the `cc_*` ABI.

#### Scenario: A planar dihedral yields an exactly-planar bevel with setback lines at distance d

- GIVEN two PLANAR faces meeting at a convex 90° dihedral edge and a symmetric setback `d`
- WHEN `chamfer_edge_symmetric` traces the chamfer
- THEN each setback curve SHALL lie ON its base plane (C0) at in-plane distance `d` from the edge to within 1e-12, AND the ruled chamfer face SHALL be EXACTLY planar (four-corner best-fit-plane residual ≤1e-12), AND the chamfer face normal SHALL make 45° with each base face (|cos| = 1/√2 to within 1e-12)

#### Scenario: The three chamfer modes are mutually consistent

- GIVEN a planar dihedral and setbacks `d`, `d1`, `d2`
- WHEN `chamfer_edge_asymmetric(faceA, faceB, edge, d, d)` and `chamfer_edge_symmetric(faceA, faceB, edge, d)` are compared, AND `chamfer_edge_distance_angle(faceA, faceB, edge, d1, atan(d2/d1)·180/π)` and `chamfer_edge_asymmetric(faceA, faceB, edge, d1, d2)` are compared
- THEN the asymmetric-equal-leg result SHALL reproduce the symmetric result rail-for-rail to within 1e-12, AND the distance-angle result SHALL reproduce the corresponding asymmetric result rail-for-rail to within 1e-12

#### Scenario: A cylinder substrate setback is the exact geodesic/normal offset

- GIVEN a planar cap and a coaxial cylinder wall of radius `R` meeting at the convex circular rim, and a symmetric setback `d`
- WHEN the chamfer is traced on the rim
- THEN the cap setback curve SHALL be the coaxial circle of radius `R−d` at the cap height to within 1e-9, AND the wall setback curve SHALL STAY on the cylinder (radius `R`) at axial offset `d` to within 1e-9 (the correct along-surface geodesic offset), AND the ruled chamfer face SHALL contain both setback curves as its rails

### Requirement: Honest decline on over-large setback, degenerate dihedral, or unsupported substrate

The chamfer generator SHALL return an HONEST DECLINE — an empty triangle band plus a measured decline
reason (`ChamferDecline`) — and SHALL NOT emit a self-intersecting chamfer face, whenever a setback
exceeds the face extent (the two setback rails cross, coincide, or a rail sweeps backward relative to
the edge), the dihedral is degenerate (the two face normals are parallel so there is no wedge to
bevel, or the edge tangent is null), a setback argument is non-positive, or a face substrate is
FREEFORM (no closed-form along-surface offset). No tolerance SHALL be widened to force a pass; a
decline with a measured reason is a first-class outcome.

#### Scenario: An over-large setback declines rather than emitting a folded face

- GIVEN a plane cap ↔ cylinder wall rim and a cap setback larger than the cylinder radius (so the cap rail laps past the axis)
- WHEN `chamfer_edge_asymmetric` is invoked
- THEN it SHALL return a result whose triangle band is EMPTY and whose decline reason is `OverLargeSetback`, and SHALL NOT return any triangles (no self-intersecting face)

#### Scenario: A degenerate dihedral or unsupported substrate declines with a measured reason

- GIVEN two faces whose outward normals are parallel (no wedge), OR a face whose substrate kind is Freeform, OR a non-positive setback
- WHEN any chamfer entry point is invoked
- THEN it SHALL return an empty band with decline reason `DegenerateDihedral`, `UnsupportedSubstrate`, or `BadArguments` respectively, never a chamfer face

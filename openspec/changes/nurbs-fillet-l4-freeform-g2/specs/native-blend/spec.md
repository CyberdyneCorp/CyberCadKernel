# native-blend

## ADDED Requirements

### Requirement: G2 rolling-ball fillet between two general freeform NURBS faces

The native blend library SHALL provide, in an OCCT-free header
(`src/native/blend/fillet_edge_g2_freeform.h`, namespace `cybercad::native::blend`), a routine
`fillet_edge_g2_freeform(faceA, faceB, radius, seed, nSectionSamples)` that constructs a G2
(curvature-continuous) rolling-ball fillet where BOTH base faces are GENERAL freeform NURBS surfaces
(tensor-product B-spline or rational NURBS, not restricted to analytic primitives). The routine SHALL
march the rolling-ball CENTRE LOCUS (the set of centres equidistant `radius` from both faces): at
each station it SHALL drop the footpoint (nearest point) of the current centre onto each face and
Newton-adjust the centre so both footpoint distances equal `radius`, then step the centre along the
local spine tangent `n̂A × n̂B`. At each seated station it SHALL build a quintic-Bézier cross-section
whose endpoints are the two contact points, whose end tangents lie in each face's tangent plane (G1),
and whose end curvatures equal each face's NORMAL CURVATURE in the section plane, read from that
face's LOCAL second fundamental form (`κ_n = II(d)/I(d)` with the first/second fundamental forms
evaluated via the Layer-1 `nurbsSurfaceDerivs` order-2 evaluator) and placed by the pole rule
`q = (5/4)·κ·h²`. Consecutive sections SHALL be skinned into a triangle band. The routine SHALL be
OCCT-free (0 OCCT/Geom/BRep/TK references) and SHALL NOT change the `cc_*` ABI.

#### Scenario: A NURBS plane pair reduces to the analytic zero-end-curvature quintic

- GIVEN faceA and faceB that are PLANAR faces represented as NURBS (bilinear patches)
- WHEN the freeform fillet seats a station on that concave dihedral
- THEN the freeform normal-curvature read for each face SHALL be 0 to within 1e-9, AND the built quintic section's end curvatures SHALL be 0 to within 1e-9 (the collinear-triple, reproducing the analytic planar G2 fillet)

#### Scenario: A rational-NURBS sphere reads its umbilic normal curvature 1/R

- GIVEN a sphere of radius `R` represented as a rational NURBS surface
- WHEN the surface's normal curvature is read in any tangent direction at an interior parameter
- THEN the reading SHALL equal `1/R` to within 1e-9 (a sphere is umbilic), matching the analytic sphere G2 builder's end-curvature value

#### Scenario: The fillet is G2 (tangent and curvature continuous) to each freeform face

- GIVEN two genuinely freeform (bicubic bump) faces on which the ball seats at multiple stations
- WHEN each seated station's cross-section is inspected at its two contact ends
- THEN the section tangent SHALL lie in the corresponding face's tangent plane to within 1e-6 rad (G1), AND the section's end curvature SHALL equal that face's normal curvature in the section plane to a relative tolerance of 1e-4 (G2)

### Requirement: Honest decline when the freeform ball will not fit or the section folds

The freeform fillet routine SHALL return an HONEST DECLINE — an empty triangle band plus a measured
decline reason (`FreeformFilletDecline`) — and SHALL NOT emit a self-intersecting fillet, whenever
any requested station fails to seat a consistent rolling ball (the centre Newton does not converge —
e.g. `radius` exceeds the local concave curvature limit so the ball will not fit, or the two contact
normals are anti-parallel), the cross-section folds (its poles cross), or the second-fundamental-form
curvature read is non-finite. No tolerance SHALL be widened to force a pass; a decline with a
measured reason is a first-class outcome.

#### Scenario: An over-radius ball declines rather than emitting a folded fillet

- GIVEN two freeform faces separated by a crease and a `radius` far larger than the local crease can seat
- WHEN `fillet_edge_g2_freeform` is invoked
- THEN it SHALL return a result whose triangle band is EMPTY and whose decline reason is set (not `None`), and SHALL NOT return any triangles (no self-intersecting fillet)

#### Scenario: A single failed station declines the whole fillet, never a partial leaky band

- GIVEN a spine march in which at least one requested station cannot seat or its section folds
- WHEN the routine processes the stations
- THEN the whole result SHALL be a decline (empty triangles + a decline reason), never a partially-skinned band that stops at the failed station

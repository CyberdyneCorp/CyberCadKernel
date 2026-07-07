# native-ssi

Affirm the S1 analytic coaxial-cone circle seam (`intersectCylinderConeCoaxial` /
`intersectSphereConeCoaxial`, `src/native/ssi/quadric_pairs.h`) as the consumed input contract for
the new S5 cone COMMON assembler. The coaxial cone∩cylinder seam is a SINGLE closed full-turn
circle (radius `Rc`, at the height where the cone cross-section radius equals `Rc`),
`nearTangentGaps == 0`, `branchPoints == 0`, full-circle on both walls, NOT passing through the
cone apex — the cleanest possible seam. The cone COMMON builders (`buildConeCylCommon`,
optionally `buildConeSphereCommon`) consume that seam to weld the frustum band to the
cylinder-segment / spherical-segment band. The tracer does not change; the transversal
(non-coaxial) cone pair (a quartic space curve, `notAnalytic` here), the two-circle coaxial
crossing, and the apex-crossing seam remain the honest decline boundary → OCCT.

## ADDED Requirements

### Requirement: The S1 analytic coaxial-cone circle seam is the consumed input contract for the S5 cone common

The S1 `cybercad::native::ssi` analytic layer's coaxial-cone circle seam SHALL be the input
contract consumed by the native S5 cone COMMON booleans (`src/native/boolean/ssi_boolean.cpp`):
`buildConeCylCommon` (coaxial cone∩cylinder) and, optionally, `buildConeSphereCommon` (coaxial
cone∩sphere, single-crossing config). For a COAXIAL frustum cone and cylinder, the boolean SHALL
obtain the `TraceSet` and use the SINGLE closed full-turn `WLine` circle
(`intersectCylinderConeCoaxial`, radius `Rc` at the crossing height `h*` where the cone
cross-section radius `r_c(h*) = Rc`) — its per-node `(u1,v1,u2,v2)` on both walls (the seam-circle
track) and its shared 3D nodes (the seam vertices both bands weld on through the shared
`VertexPool`). The SAME seam SHALL split the axial overlap into a CONE-tighter sub-band (kept as
the cone wall, inside the cylinder) and a CYLINDER-tighter sub-band (kept as the cylinder wall,
inside the cone); the common assembles the frustum band welded along the seam circle to the
cylinder-segment band, closed by two disc caps. The boolean SHALL consume the seam ONLY when it is
a fully transversal single interior circle — `nearTangentGaps == 0`, `branchPoints == 0`, exactly
ONE full-circle `WLine` on BOTH walls, the frustum apex-free over its extent, and `h*` strictly
inside both operand extents. A seam with `nearTangentGaps > 0`, a `NearTangent` / `Failed` /
`BranchArc` WLine, a two-circle coaxial crossing, an apex-crossing circle, or a non-coaxial
(transversal) cone pair whose true seam is a quartic space curve (`intersectCylinderConeCoaxial`
returns `notAnalytic`) SHALL be treated as the honest S4 / OCCT boundary and SHALL NOT be consumed
(the boolean declines → OCCT). The tracer SHALL NOT change to serve this consumption — the
contract is the already-shipped S1 output; no `cc_*` entry point, signature, or POD struct SHALL be
added or changed, and the SSI module SHALL remain OCCT-free and compiled under
`CYBERCAD_HAS_NUMSCI` (like the S1 analytic layer).

#### Scenario: the single coaxial-cone circle seam is consumed to weld the frustum and cylinder bands
- GIVEN a coaxial frustum-cone∩cylinder pair whose S1 seam is exactly one closed full-turn circle
  (radius `Rc` at `h*`), `nearTangentGaps == 0`, `branchPoints == 0`, the frustum apex-free and
  `h*` strictly inside both extents
- WHEN the S5 cone COMMON boolean consumes the seam
- THEN it SHALL resample the circle into one pooled full-turn seam ring, split the axial overlap at
  `h*` into the cone-tighter and cylinder-tighter sub-bands, and weld the frustum band and the
  cylinder-segment band along that single seam ring, with every seam node on BOTH walls ≤ tol
- AND no `cc_*` entry point SHALL have been added or changed

#### Scenario: a transversal, two-circle, or apex-crossing cone seam is the decline boundary, not consumed
- GIVEN a NON-coaxial (transversal) cone∩cylinder pair whose true seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a coaxial cone∩sphere pair whose S1
  seam is TWO circles inside both extents, OR a coaxial pair whose circle passes through the cone
  apex (a frustum extent reaching `r_c → 0`)
- WHEN the S5 cone COMMON boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic, multi-circle, or
  apex-crossing seam

#### Scenario: the default single-seam trace is unchanged for non-cone pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams) or a
  sphere∩sphere lens (one closed circle), OR any pair that is not a coaxial cone pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz paths
  do, and the cone COMMON builders SHALL return NULL for it — the coaxial-cone seam machinery
  engages ONLY when one operand is a `Cone` and the other is a coaxial `Cylinder` (or `Sphere`)

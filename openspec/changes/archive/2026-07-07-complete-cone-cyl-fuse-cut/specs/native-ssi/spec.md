# native-ssi

Re-affirm the S1 analytic coaxial cone∩cylinder circle seam (`intersectCylinderConeCoaxial`,
`src/native/ssi/quadric_pairs.h`) as the consumed input contract for ALL THREE coaxial cone∩cylinder
booleans (`src/native/boolean/ssi_boolean.cpp`) — the already-native COMMON (`buildConeCylCommon`)
AND the newly-native FUSE (`buildConeCylFuse`) and CUT (`buildConeCylCut`). The seam is a SINGLE
closed full-turn circle (radius `Rc`, at the height `s*` where the cone cross-section radius
`r_c(s*)` equals `Rc`), `nearTangentGaps == 0`, `branchPoints == 0`, full-circle on both walls, NOT
passing through the cone apex — the cleanest possible seam. The SAME seam splits each coaxial wall
into an inside-the-other axial band and an outside band; COMMON welds the two inside bands, FUSE the
two outer bands (plus the beyond-overlap segments and the operand end caps / annular steps), and CUT
A's outer wall to B's inside band emitted REVERSED — differing only in WHICH bands survive, their
orientation, and the caps. The tracer does not change; the transversal (non-coaxial) cone pair (a
quartic space curve, `notAnalytic` here) and the apex-crossing seam remain the honest decline
boundary → OCCT.

## ADDED Requirements

### Requirement: The S1 analytic coaxial cone∩cylinder circle seam is the shared input contract for all three cone∩cylinder ops

The S1 `cybercad::native::ssi` analytic coaxial cone∩cylinder circle seam SHALL be the input
contract consumed by ALL THREE native S5-e coaxial cone∩cylinder booleans (via
`intersectCylinderConeCoaxial`): `buildConeCylCommon` (already native), `buildConeCylFuse`, and
`buildConeCylCut`. For a COAXIAL frustum cone and cylinder, each boolean SHALL obtain the
`TraceSet` and use the SINGLE closed full-turn `WLine` circle (radius `Rc` at the crossing height
`s*` where `r_c(s*) = Rc`) — its shared 3D nodes (the seam vertices all surviving bands weld on
through the shared `VertexPool`, pooled ONCE). The SAME seam SHALL split the axial overlap into a
CONE-tighter sub-band and a CYLINDER-tighter sub-band; COMMON welds the two INSIDE bands (the min-
radius profile), FUSE welds the two OUTER bands (the max-radius profile) plus the beyond-overlap
wall segments and the operand end caps / annular steps, and CUT welds A's OUTER wall to B's INSIDE
band emitted REVERSED (bounding the carved cavity). Each boolean SHALL consume the seam ONLY when it
is a fully transversal single interior circle — `nearTangentGaps == 0`, `branchPoints == 0`,
exactly ONE full-circle `WLine` on BOTH walls, the frustum apex-free over its extent, and `s*`
strictly inside both operand extents. A seam with `nearTangentGaps > 0`, a `NearTangent` / `Failed`
/ `BranchArc` WLine, an apex-crossing circle, a cap-edge-tangent crossing, or a non-coaxial
(transversal) cone pair whose true seam is a quartic space curve (`intersectCylinderConeCoaxial`
returns `notAnalytic`) SHALL be treated as the honest S4 / OCCT boundary and SHALL NOT be consumed
(the boolean declines → OCCT). The tracer SHALL NOT change to serve this consumption — the contract
is the already-shipped S1 output; no `cc_*` entry point, signature, or POD struct SHALL be added or
changed, and the SSI module SHALL remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI` (like
the S1 analytic layer).

#### Scenario: the single coaxial cone∩cylinder circle seam is consumed by all three ops to weld their bands
- GIVEN a coaxial frustum-cone∩cylinder pair whose S1 seam is exactly one closed full-turn circle
  (radius `Rc` at `s*`), `nearTangentGaps == 0`, `branchPoints == 0`, the frustum apex-free and
  `s*` strictly inside both extents
- WHEN each of the S5-e cone∩cylinder COMMON, FUSE, and CUT booleans consumes the seam
- THEN each SHALL split the axial overlap at `s*` into the cone-tighter and cylinder-tighter sub-
  bands, resample the circle into one pooled full-turn seam ring shared by its surviving bands with
  every seam node on BOTH walls ≤ tol, and weld — COMMON the two inside bands, FUSE the two outer
  bands (plus caps / annular steps), CUT A's outer wall to B's reversed inside band
- AND no `cc_*` entry point SHALL have been added or changed.

#### Scenario: a transversal or apex-crossing cone∩cylinder seam is the decline boundary for all three ops
- GIVEN a NON-coaxial (transversal) cone∩cylinder pair whose true seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a coaxial pair whose circle passes
  through the cone apex (a frustum extent reaching `r_c → 0`), OR a pair whose crossing `s*` sits on
  a cap edge (a tangent, not a strictly-interior transversal circle)
- WHEN the S5-e cone∩cylinder COMMON, FUSE, or CUT boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic or apex-crossing seam.

#### Scenario: the default single-seam trace is unchanged for non-cone∩cylinder pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams) or a
  sphere∩sphere lens (one closed circle), OR any pair that is not a coaxial cone∩cylinder pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz paths
  do, and the cone∩cylinder FUSE / CUT builders SHALL return NULL for it — the coaxial cone∩cylinder
  seam machinery engages ONLY when one operand is a `Cone` and the other is a coaxial `Cylinder`.

# native-ssi

Consume the S1 analytic coaxial cone∩sphere circle seam (`intersectSphereConeCoaxial`,
`src/native/ssi/quadric_pairs.h`) as the input contract for ALL THREE coaxial cone∩sphere booleans
(`src/native/boolean/ssi_boolean.cpp`) — the newly-native COMMON (`buildConeSphereCommon`), FUSE
(`buildConeSphereFuse`), and CUT (`buildConeSphereCut`). `intersectSphereConeCoaxial` is a QUADRATIC
in the cone parameter (up to TWO circles); the single-crossing configuration this change consumes is
a SINGLE closed full-turn circle (radius `r_c(s*)`, at the height `s*` where the cone cross-section
radius `r_c(s*)` equals the sphere cross-section radius `r_s(s*)`), `nearTangentGaps == 0`,
`branchPoints == 0`, full-circle on both walls, NOT passing through the cone apex — the QUADRATIC's
OTHER root falling OUTSIDE the frustum extent (the sphere sits on the frustum side). The SAME seam
splits each coaxial wall into an inside-the-other band and an outside band; COMMON welds the cone
band inside the sphere to the sphere segment inside the cone, FUSE welds the sphere outer cap to the
cone outer band, and CUT welds A's outer wall to the sphere inner cap emitted REVERSED — differing
only in WHICH bands survive, their orientation, and the cone caps. The tracer does not change; a
TWO-circle crossing (the sphere passes fully through the cone / spans the apex), the transversal
(non-coaxial) cone∩sphere pair (a quartic space curve, `notAnalytic` here), and the apex-crossing
seam remain the honest decline boundary → OCCT.

## ADDED Requirements

### Requirement: The S1 analytic coaxial cone∩sphere circle seam is the shared input contract for all three cone∩sphere ops

The S1 `cybercad::native::ssi` analytic coaxial cone∩sphere circle seam SHALL be the input contract
consumed by ALL THREE native S5-f coaxial cone∩sphere booleans (via `intersectSphereConeCoaxial`):
`buildConeSphereCommon`, `buildConeSphereFuse`, and `buildConeSphereCut`. For a COAXIAL frustum cone
and a sphere whose centre lies on the cone axis, each boolean SHALL obtain the `TraceSet` and use the
SINGLE closed full-turn `WLine` circle (radius `r_c(s*)` at the crossing height `s*` where `r_c(s*)
= r_s(s*)`) — its shared 3D nodes (the seam vertices all surviving bands weld on through the shared
`VertexPool`, pooled ONCE). The SAME seam SHALL split the axial span into a CONE-tighter sub-band and
a SPHERE-tighter sub-band; COMMON welds the cone band inside the sphere to the sphere segment inside
the cone (the min-cross-section profile), FUSE welds the sphere OUTER cap to the cone OUTER band (the
max-cross-section profile), and CUT welds A's OUTER wall to the sphere INNER cap emitted REVERSED
(bounding the carved dimple). Each boolean SHALL consume the seam ONLY when it is a SINGLE fully
transversal interior circle — `nearTangentGaps == 0`, `branchPoints == 0`, exactly ONE full-circle
`WLine` on BOTH walls, the frustum apex-free over its extent, and `s*` strictly inside both operand
extents with the QUADRATIC's other root OUTSIDE the frustum extent. A seam with `nearTangentGaps >
0`, a `NearTangent` / `Failed` / `BranchArc` WLine, a TWO-circle crossing (both roots inside both
extents), an apex-crossing circle, or a non-coaxial (transversal) cone∩sphere pair whose true seam
is a quartic space curve (`intersectSphereConeCoaxial` returns `notAnalytic`) SHALL be treated as
the honest S4 / OCCT boundary and SHALL NOT be consumed (the boolean declines → OCCT). The tracer
SHALL NOT change to serve this consumption — the contract is the already-shipped S1 output; no
`cc_*` entry point, signature, or POD struct SHALL be added or changed, and the SSI module SHALL
remain OCCT-free and compiled under `CYBERCAD_HAS_NUMSCI`.

#### Scenario: the single coaxial cone∩sphere circle seam is consumed by all three ops to weld their bands
- GIVEN a coaxial frustum-cone∩sphere pair (sphere centre on the cone axis) whose S1 seam is exactly
  one closed full-turn circle (radius `r_c(s*)` at `s*`), `nearTangentGaps == 0`, `branchPoints ==
  0`, the frustum apex-free, `s*` strictly inside both extents, and the QUADRATIC's other root
  outside the frustum extent
- WHEN each of the S5-f cone∩sphere COMMON, FUSE, and CUT booleans consumes the seam
- THEN each SHALL split the axial span at `s*` into the cone-tighter and sphere-tighter sub-bands,
  resample the circle into one pooled full-turn seam ring shared by its surviving cone band and
  sphere cap with every seam node on BOTH walls ≤ tol, and weld — COMMON the cone band to the sphere
  segment, FUSE the sphere outer cap to the cone outer band, CUT A's outer wall to the reversed
  sphere inner cap
- AND no `cc_*` entry point SHALL have been added or changed.

#### Scenario: a two-circle, transversal, or apex-crossing cone∩sphere seam is the decline boundary for all three ops
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone / spans the apex), OR a NON-coaxial
  (transversal) cone∩sphere pair whose true seam is a quartic space curve
  (`intersectSphereConeCoaxial` returns `notAnalytic`), OR a coaxial pair whose circle passes
  through the cone apex (a frustum extent reaching `r_c → 0`)
- WHEN the S5-f cone∩sphere COMMON, FUSE, or CUT boolean inspects the seam
- THEN it SHALL decline to consume the seam (the honest S4 / OCCT boundary) and the boolean SHALL
  fall back to OCCT, reported — never welding a shell on a non-analytic, two-circle, or apex-
  crossing seam.

#### Scenario: the default single-seam trace is unchanged for non-cone∩sphere pairs
- GIVEN a transversal surface pair whose seam is a through-drill cyl∩cyl (two rim seams), a
  sphere∩sphere lens (one closed circle), or a coaxial cone∩cylinder (one closed circle), OR any
  pair that is not a coaxial cone∩sphere pair
- WHEN the S5 curved boolean traces it
- THEN it SHALL consume its existing S3/S1 `TraceSet` exactly as the cyl / sphere / Steinmetz /
  cone∩cylinder paths do, and the cone∩sphere COMMON / FUSE / CUT builders SHALL return NULL for it
  — the coaxial cone∩sphere seam machinery engages ONLY when one operand is a `Cone` and the other
  is a `Sphere` whose centre lies on the cone axis.

# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random base-solid and rigid-pose generator for reference-geometry fuzzing

The reference-geometry differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of base-solid
parameters and rigid-pose operations on any machine.

Each generated trial SHALL parameterise (a) a VALID, non-degenerate base solid drawn
from the analytic families the native construction path builds — a `BOX` prism, an
`NGON` prism, a `CYLINDER`, and a `CONE` frustum — and (b) a RIGID pose composed of a
rotation about a random unit axis followed by a random translation (NO scale, NO
mirror), so that every reference datum of the base solid transforms EXACTLY and has an
exact closed form. The generator SHALL build the rigid pose three independent ways in
lock-step (the native `math::Transform`, the OCCT `gp_Trsf`, and an engine-independent
fp64 affine) and SHALL be OCCT-free in the sense that it produces plain parameter POD
consumed identically by the native builder, the OCCT oracle builder, and the analytic
arbiter.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the reference-geometry fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` base-solid + rigid-pose trials
- THEN the two sequences SHALL be byte-identical (same base families, same pose operations and numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Only rigid poses are generated (honest scope)

- GIVEN a generated pose
- WHEN its operations are inspected
- THEN the pose SHALL be a rotation followed by a translation only (an orthonormal linear part, no scale and no mirror), so every datum image (axis, plane normal, edge line, offset-polygon area) is an exact transform of the base solid's closed form

### Requirement: Three independent drivings of each reference query with a closed-form datum arbiter

For each trial the harness SHALL drive every reference query on the posed native solid,
the posed OCCT analog, and a closed-form analytic datum image. The **native** driving
SHALL call the OCCT-free reference services (`faceAxis` / `refAxisFromFace` /
`refPlaneFromFace` / `refAxisFromEdge` / `tangentChain` / `outerRimChain` /
`offsetFaceBoundary`) on the solid returned by `Shape::located(Location{...})`. The
**oracle** driving SHALL compute the corresponding OCCT topology query
(`gp_Cylinder`/`gp_Cone::Axis`, `gp_Pln`, `gp_Lin`, `BRepOffsetAPI_MakeOffset`,
`BRepTools::OuterWire`, `BRepAdaptor_Curve::D1`) on the OCCT analog base transformed by
the SAME `gp_Trsf`. The **analytic arbiter** SHALL be a THIRD computation using a plain
fp64 affine (NOT the native `math::Transform`, NOT the OCCT `gp_Trsf`) that applies the
identical rigid pose to the base solid's KNOWN construction datum: an axis/line
direction is `P.linear·d₀` through `P·o₀`, a planar cap normal is `P.linear·n₀`, and an
inward polygon-offset area is the rigid-invariant closed-form miter offset of the base
polygon.

The native tolerance SHALL be a FIXED tight rigid tolerance (direction dot to `1e-9`,
on-line/on-plane residual to `1e-7`, offset area to `1e-6`) and SHALL NOT be widened to
force a pass.

#### Scenario: Native, OCCT, and the analytic image all query the identical posed solid

- GIVEN one generated trial
- WHEN the native builder builds the base and the native reference services query the located solid, the OCCT builder builds the analog and `BRepBuilderAPI_Transform` applies the same `gp_Trsf`, and the independent fp64 affine applies the same rigid pose to the exact base construction datum
- THEN all three SHALL have operated on the SAME base solid and the SAME rigid pose, and the analytic datum image SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: A cylinder/cone axis matches the analytic posed axis and the OCCT axis

- GIVEN a CYLINDER or CONE base at a rigid pose
- WHEN the native lateral-face axis (`faceAxis`) is compared to the analytic posed axis (the construction +Y axis transformed by the pose) and the OCCT `gp_Cylinder`/`gp_Cone::Axis`
- THEN the native axis direction SHALL be parallel to the analytic axis within `1e-9`, its origin SHALL lie on the analytic axis line within `1e-7`, and OCCT SHALL concur; `refAxisFromFace` SHALL equal `faceAxis` bit-for-bit

#### Scenario: A planar-face datum plane and a straight-edge axis match the OCCT oracle and the analytic image

- GIVEN a BOX or NGON prism base at a rigid pose
- WHEN every OCCT planar face's posed outward normal is matched against a native `refPlaneFromFace` datum, and every OCCT line edge's posed `gp_Lin` is matched against a native `refAxisFromEdge`
- THEN each OCCT planar face SHALL be matched by a native datum whose normal is parallel and whose origin is coplanar, and each OCCT line edge SHALL be matched by a native axis whose direction is parallel and whose line passes through the OCCT edge midpoint

#### Scenario: A scoped-out reference case declines and is matched by the closed form

- GIVEN a circular cap (CYLINDER/CONE) offered to `offsetFaceBoundary`, or a freeform edge offered to `tangentChain`
- WHEN the native reference service is called
- THEN it SHALL return an honest decline (`nullopt`/empty), the closed form SHALL confirm no closed-form datum exists for that case, and the trial SHALL be classified HONESTLY-DECLINED — a native service that instead returned a datum there (e.g. a polygon for a circular cap) SHALL be a DISAGREE

### Requirement: Agree / honestly-declined / DISAGREE reference-datum classifier arbitrated by the closed-form ground truth

The harness SHALL classify each reference-op trial into EXACTLY ONE bucket at the fixed
rigid tolerance, with the closed-form datum image as the PRIMARY correctness oracle:

- **AGREED** — the native datum matches the analytic datum image within tolerance AND
  the OCCT oracle concurs.
- **HONESTLY-DECLINED** — the native service returned an honest decline on a case the
  reference services scope out (a circular-cap offset, a freeform edge in a tangent
  walk) and the closed form agrees no closed-form datum exists. First-class, logged,
  NOT a bar failure.
- **DISAGREED** — the native service returned a datum that does NOT match the closed-form
  image (a wrong axis/plane/line/offset, a wrong rim id set, a non-collinear tangent
  grow). A genuine SILENT WRONG DATUM — the failure this harness exists to catch. FAILS
  the bar.
- **ORACLE-INACCURATE** — the native datum matches the closed-form image but OCCT does
  NOT. The native datum is CORRECT and vindicated by exact math; OCCT is the outlier.
  Logged in full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a scoped-out exerciser where neither the native service nor the
  oracle produced a datum.
- **ORACLE_UNRELIABLE** — a core case whose OCCT oracle could not be obtained or does not
  match the closed form (e.g. no matching OCCT cap face). FAILS the bar (investigate;
  never laundered into a pass).

For the `outerRimChain` of a CIRCULAR cap, whose native periodic revolution face stores
its rim as arc edges with periodic seam vertices (a legitimate representational
difference from OCCT's single seam edge), the arbiter SHALL be STRUCTURAL: the rim id
set MUST equal the native cap face's own outer-wire edge ids, confirmed as a circular
boundary by the OCCT circle oracle. A per-vertex geometric comparison against the OCCT
circle SHALL NOT be used as the oracle there, because it does not reflect the faithful
"which edges bound this cap" datum the query returns.

#### Scenario: A native datum matching OCCT and the closed form is AGREED

- GIVEN an in-scope reference-op trial
- WHEN the native datum matches both the OCCT oracle and the closed-form analytic image within the fixed rigid tolerance
- THEN the trial SHALL be classified AGREED and SHALL contribute to that base family's and that op's coverage count

#### Scenario: A valid native datum that fails the closed form is a silent-wrong-result

- GIVEN a reference-op trial whose native service returned a datum
- WHEN that datum disagrees with the closed-form analytic image beyond the fixed tolerance (a wrong direction, off-line/off-plane origin, wrong offset area, wrong rim id set, or a non-collinear tangent grow)
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and base descriptor so the native fault is reproducible

#### Scenario: A native datum vindicated by exact math is not a false native fault

- GIVEN a trial where the native datum differs from the OCCT oracle but matches the closed-form analytic image while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-datum bar, and logged honest scope

The harness SHALL print a coverage summary — the per-base-family and per-op counts of
AGREED trials and the per-family DISAGREED count — and SHALL exit 0 IF AND ONLY IF the
bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real
coverage (each of BOX, NGON, CYLINDER, CONE and each of PLANE, FAXIS, EAXIS, OFFSET,
RIM, TANGENT exercised with at least one AGREED trial) proven across **at least two
distinct seeds**. Any DISAGREED (and any ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL
be reported with its seed so it is reproducible. The harness SHALL NOT weaken a
tolerance, silently cap the batch, or drop trials to make the bar pass. The honest scope
SHALL be logged explicitly — the rigid-pose-only restriction and the scoped declines
(circular-cap offset, freeform edge in a tangent walk) SHALL appear in the summary.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every base family and every reference op
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each base family and each op has at least one AGREED trial
- THEN the harness SHALL print the per-family / per-op coverage summary, log the honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and base descriptor, so the silent-wrong-datum is reproducible and not laundered into a pass

#### Scenario: Honest scope and any dropped trials are logged, not silently omitted

- GIVEN a run in which some trials are scoped-out declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the rigid-pose-only restriction, the circular-cap-offset decline, and the freeform-tangent decline SHALL be logged explicitly in the summary, so no honest exclusion is hidden

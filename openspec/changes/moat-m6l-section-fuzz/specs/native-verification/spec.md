# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random primitive-and-cut-plane generator for section-curve fuzzing

The section-curve differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL NOT
read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64 seeding a
xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED` (argv/env-overridable,
with a fixed default). Re-running the harness with the same seed and batch size SHALL
produce a **byte-identical** sequence of primitive parameters and cut planes on any machine.

Each generated trial SHALL parameterise (a) one VALID elementary primitive drawn from the
families the native planar section service serves — a `BOX`, a `CYLINDER` (perpendicular,
axial, and oblique cuts), and a `SPHERE` — built at the native `cybercad::native::section`
C++ boundary with the same `ShapeBuilder` fixtures the parity gate uses AND independently as
a matched OCCT `BRepPrimAPI` solid to the same dimensions; and (b) one cut plane in that
family's AGREE envelope: a `BOX` axis-aligned interior cut (→ rectangle), a `CYL_PERP`
interior perpendicular cut (→ circle), a `CYL_AXIAL` cut through the axis (→ rectangle), a
`CYL_OBL` OBLIQUE cut whose ellipse fits inside the finite axial band with no arc-trim (→
ellipse), a `SPHERE` interior cut at signed offset `|d| < R` (→ circle of radius
`√(R²−d²)`), or a `DECLINE`-exerciser plane that UNAMBIGUOUSLY yields no section (clearly
missing the solid, or coincident with a planar face). The first `F_COUNT` trials SHALL force
each family so every family is exercised.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the section-curve fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` primitive + cut-plane trials
- THEN the two sequences SHALL be byte-identical (same families, same primitive dimensions, same cut-plane origins and normals in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Matched primitives are built identically at the native boundary and as an OCCT solid

- GIVEN a generated trial's primitive family and dimensions
- WHEN the native solid is built with the `ShapeBuilder` fixtures and the OCCT solid with the matching `BRepPrimAPI_Make{Box,Cylinder,Sphere}` to the SAME dimensions
- THEN both SHALL represent the same nominal elementary solid so the native section service is sectioned against a geometrically identical OCCT oracle solid

### Requirement: Native section driven at its C++ boundary against a closed-form conic and an OCCT section oracle

For each trial the harness SHALL section the native solid at the `cybercad::native::section::
sectionByPlane` C++ boundary (the OCCT-FREE system under test) and the matched OCCT solid
with `BRepAlgoAPI_Section`, and SHALL arbitrate each trial against a THIRD engine-independent
CLOSED-FORM conic computed in plain fp64:

- **PRIMARY — the closed-form conic.** The section of an elementary solid is an exact
  elementary conic whose perimeter and enclosed area have a closed form: a rectangle
  (`2(w+h)` / `w·h`), a circle (`2πR` / `πR²`), an axial rectangle (`2(2R+H)` / `2R·H`), an
  ellipse (Ramanujan-II perimeter / `πab`, `a=R/|cosθ|`, `b=R`), or a sphere circle (`2πr` /
  `πr²`). Because the native loop's `length()`/`area()` ARE these closed forms, the native
  section SHALL match the analytic value to a TIGHT tolerance — `1e-9` relative for the
  straight/circular length and for every family's area, and `1e-4` relative for the ELLIPSE
  PERIMETER only (the Ramanujan-II + OCCT arc-length integrator bound), which SHALL NOT be
  widened. This closed-form value is the certifying correctness signal.
- **SECONDARY — the OCCT section oracle.** The native loop count SHALL equal the OCCT wire
  count recovered by `ShapeAnalysis_FreeBounds`, the native total edge length SHALL match
  the summed OCCT arc length measured by `GCPnts_AbscissaPoint::Length` within the family
  tolerance, and the native capped area SHALL match the OCCT `BRepGProp::SurfaceProperties`
  section-face mass within `1e-6` relative.

#### Scenario: The closed-form conic is the primary arbiter

- GIVEN a section trial whose native result is a valid `Ok` section
- WHEN the harness compares the native loop count, total edge length, and capped area to the closed-form conic
- THEN it SHALL require the native values to match the analytic conic within the tight family tolerance (`1e-9` straight/circular, `1e-4` ellipse perimeter, `1e-6` area), independent of any OCCT result

#### Scenario: The OCCT section is the secondary oracle at a deflection-matched tolerance

- GIVEN a section trial whose native result is a valid `Ok` section
- WHEN the harness compares the native loop count / edge length / capped area to the OCCT `BRepAlgoAPI_Section` wire count / `GCPnts` arc length / `BRepGProp` face area
- THEN it SHALL require agreement within the same deflection-matched family tolerance (never widened), so a native-vs-OCCT gap can be attributed with the analytic truth

### Requirement: Agree / honestly-declined / DISAGREE section classifier arbitrated by the closed-form conic

The harness SHALL classify each section-curve trial into EXACTLY ONE bucket at the fixed
tolerances, with the closed-form conic as the primary correctness oracle:

- **AGREED** — the native section is a valid `Ok` section whose loop count, length, and area
  match the closed-form conic AND the OCCT section within the family tolerance.
- **HONESTLY-DECLINED** — the native service reported `Empty`/`Declined` (a plane
  missing/coincident/tangent to a face, an open section, a freeform/torus face, or a
  numerically marginal cut its self-verify rejects) while OCCT shipped or also declined.
  First-class, logged, NOT a bar failure.
- **DISAGREED** — the native section is a valid `Ok` section OUTSIDE the closed-form analytic
  truth, OR a valid `Ok` section on a config the harness expects the native service to
  decline. A genuine SILENT WRONG SECTION — the failure this harness exists to catch. FAILS
  the bar.
- **ORACLE_UNRELIABLE** — the native section matches the closed-form analytic truth while
  OCCT does not, or OCCT produced no usable section where the native service and the closed
  form both cover it (OCCT the outlier). Native vindicated; logged in full, gated off, NOT a
  native fault.

#### Scenario: A native section matching the conic and OCCT is AGREED

- GIVEN a section trial whose native result is a valid `Ok` section
- WHEN its loop count, length, and area match the closed-form conic and the OCCT section within the family tolerance
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A native section outside the analytic truth is a silent-wrong-section

- GIVEN a section trial whose native result is a valid `Ok` section
- WHEN its length or area differs from the closed-form conic beyond the family tolerance, or the native service produced a section on a config the harness expects it to decline
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and full primitive + cut-plane descriptor so the native fault is reproducible

#### Scenario: A native section vindicated by the conic is not a false native fault

- GIVEN a trial where the native section matches the closed-form conic but OCCT does not (or OCCT emits no usable section) — for example an exact-tangency plane a sub-nanometre inside a sphere where native returns the true sub-micron circle and OCCT rounds it to empty
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE_UNRELIABLE (or the harness SHALL not force such a measure-zero knife-edge as a decline), logged in full with its seed, and SHALL NOT be counted as a native fault

### Requirement: Coverage summary, zero-silent-wrong-section bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED,
HONESTLY-DECLINED, DISAGREED, and ORACLE_UNRELIABLE trials — and SHALL exit 0 IF AND ONLY IF
the bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real
coverage (each AGREE family — BOX, CYL_PERP, CYL_AXIAL, CYL_OBL, SPHERE — having at least one
AGREED trial, and the decline exerciser at least one HONESTLY-DECLINED trial) proven across
**at least two distinct seeds**. Any DISAGREED (and any ORACLE_UNRELIABLE) SHALL be reported
with its seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently cap
the batch, or drop trials to make the bar pass; the tight closed-form conic SHALL remain the
certifying arbiter. The honest scope SHALL be logged explicitly — the missing / coincident /
tangent native declines SHALL appear in the summary.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every AGREE family and the decline exerciser
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each AGREE family has at least one AGREED trial and the decline exerciser at least one HONESTLY-DECLINED trial
- THEN the harness SHALL print the per-family coverage summary, log the honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and full primitive + cut-plane descriptor, so the silent-wrong-section is reproducible and not laundered into a pass

#### Scenario: Honest scope is logged, not silently omitted

- GIVEN a run in which some trials are scoped-out declines (HONESTLY-DECLINED)
- WHEN the harness summarises
- THEN the missing / coincident / tangent native declines SHALL be counted and reported in the summary, so no honest exclusion is hidden

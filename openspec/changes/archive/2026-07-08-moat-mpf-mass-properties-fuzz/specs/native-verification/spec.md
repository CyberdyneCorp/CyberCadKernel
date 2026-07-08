# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random-valid solid generator for the native mass-properties families

The mass-properties differential-fuzzing harness SHALL generate its batch of solids
from a **deterministic, explicitly-seeded** pseudo-random number generator. The
harness SHALL NOT read any wall clock, `rand()`, `Date`, process id, address, or any
other non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of solid
parameter tuples on any machine.

Each generated tuple SHALL parameterise a VALID, non-degenerate solid drawn from the
**native mass-properties families** the native path actually builds and meshes: a
`BOX` (rectangular prism), an `NGON_PRISM` (regular-n-gon extrude), a `CYLINDER`, a
`CONE`/`FRUSTUM`, a `SPHERE` (revolves of a rectangle / trapezoid / half-disk
profile), a `LOFT` (coaxial regular-n-gon prismatoid stack), and a `REVOLVE` (an
arbitrary axial polygon profile revolved through 2π). All sampled parameters SHALL be
constrained to produce a **valid, watertight, positive-volume** solid (positive
radii/extents, non-self-intersecting profile, finite dimensions) so the native mass
path is genuinely EXERCISED rather than trivially declined. The generator MAY also
emit **sparse, explicitly-labelled out-of-scope DECLINE-exercisers** (e.g. a
degenerate zero-height or self-touching profile) whose only purpose is to reach the
no-valid-mass path. The generator SHALL be OCCT-free (it produces plain parameter
POD consumed identically by the native builder and the OCCT oracle builder).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the mass-properties fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` solid parameter tuples
- THEN the two sequences SHALL be byte-identical (same family tags and same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Generated solids are valid and within the native mass path's meshable scope

- GIVEN a generated in-scope tuple (BOX / NGON_PRISM / CYLINDER / CONE / SPHERE / LOFT / REVOLVE)
- WHEN the native builder builds it
- THEN the solid SHALL be a single valid closed body with positive volume, watertight when meshed at the property deflection, so the native mass computation is genuinely exercised rather than declined on malformed input

### Requirement: Dual native mass measurement vs OCCT BRepGProp from one solid, with a closed-form analytic arbiter

For each generated tuple the harness SHALL measure the SAME solid two independent
ways and SHALL additionally compute a **closed-form analytic ground truth** for every
family that has one. The **native** measurement SHALL be the native mesh-based mass
path called DIRECTLY under the native engine — enclosed volume, surface area, and
centroid via the tessellator's signed-tetra decomposition at the fixed property
deflection, with `valid = watertight ∧ volume > 0`. The **oracle** measurement SHALL
be OCCT `BRepGProp::VolumeProperties` + `BRepGProp::SurfaceProperties` on the exact
B-rep (plus `GProp_PrincipalProps` for inertia). The **analytic arbiter** SHALL be
the exact closed form for the family: box/prism/cylinder/cone/sphere exact volume,
area, centroid (and principal moments); a coaxial-n-gon LOFT via the prismatoid-band
volume `Σ (Δz/3)(A_k + √(A_k A_{k+1}) + A_{k+1})` with planar-trapezoid lateral area;
a REVOLVE via Pappus (`V = 2π·r̄·A_profile`, lateral `A = 2π·r̄·P_profile`).

Because the native answer is a mesh discretisation of an exact solid, the native
tolerance SHALL be **matched to the tessellation deflection bound** at
`kPropertyDeflection` (tight for a planar exact-meshing family such as BOX / NGON_PRISM
/ straight LOFT; the deflection-derived convergence bound for a curved family such as
CYLINDER / SPHERE / CONE). This tolerance SHALL NOT be widened beyond that bound. The
harness SHALL NOT invent a native principal-moments value: because the native path
delegates inertia to OCCT (`CC_NATIVE_BODY_UNSUPPORTED`), the inertia dimension has
no native answer and is recorded as an honest native decline (the analytic-vs-OCCT
inertia comparison MAY still be logged as an oracle-trust check, never as a native
differential).

#### Scenario: Native and OCCT measure the identical solid, native called directly

- GIVEN one generated parameter tuple
- WHEN the native builder builds it and the native mass path measures it, and the OCCT builder builds the same tuple and `BRepGProp` measures it
- THEN both engines SHALL have operated on the SAME geometric solid (same family and parameters), the native mass path SHALL have been invoked directly (a native decline is observed as `valid == 0`, not silently forwarded to OCCT), and the closed-form analytic volume/area/centroid SHALL have been computed for every family that has one

#### Scenario: The native inertia dimension is an honest decline, never a fabricated number

- GIVEN a valid native body in any family
- WHEN the harness requests native principal moments
- THEN the native path SHALL delegate to OCCT (no independent native inertia), and the harness SHALL record the inertia dimension as an HONEST NATIVE DECLINE — it SHALL NOT synthesise a native inertia value nor count inertia as a native-vs-native agreement; the closed-form-vs-OCCT inertia check, if run, SHALL be logged only as oracle validation

#### Scenario: A curved family is held to the deflection-matched bound, not a widened tolerance

- GIVEN a CYLINDER / SPHERE / CONE trial whose native mesh under-approximates the curved surface at `kPropertyDeflection`
- WHEN the native mesh-based volume/area is compared to the exact closed form
- THEN the tolerance SHALL be the tessellation convergence bound derived from the deflection and feature size (never an arbitrary widened value), and a planar family (BOX / NGON_PRISM / straight LOFT) SHALL be held to the tight exact-meshing tolerance

### Requirement: Agree / honestly-declined / DISAGREE mass-properties classifier arbitrated by the closed-form ground truth

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed,
deflection-matched tolerance, with the closed-form analytic ground truth as the
PRIMARY correctness oracle (used to ATTRIBUTE a native-vs-OCCT gap rather than
reflexively blame the native path):

- **AGREED** — native `valid == 1` (watertight, positive volume) AND native
  volume/area/centroid match BOTH the OCCT `BRepGProp` measurement AND the
  closed-form analytic ground truth within tolerance.
- **HONESTLY-DECLINED** — native returns no valid mass (`valid == 0`: the native
  mesh is not watertight, so the mandatory self-verify yields no mass), while the
  OCCT build of the same tuple IS a valid measurable solid. First-class, logged, NOT
  a bar failure. (The inertia dimension is always recorded here for native bodies.)
- **DISAGREED** — native `valid == 1` but native volume/area/centroid does NOT match
  the CLOSED-FORM analytic ground truth beyond the deflection bound. A genuine SILENT
  WRONG MASS — the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native `valid == 1`, native DIFFERS from OCCT `BRepGProp`,
  but native MATCHES the closed-form analytic ground truth while OCCT does NOT. The
  native mass is CORRECT and vindicated by exact math; OCCT is the outlier. Logged in
  full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a DECLINE-exerciser where native produced no valid mass AND
  OCCT also could not measure a valid solid: neither engine produced a wrong number,
  so there is nothing to compare. Logged, NOT a bar failure.

An unreliable oracle SHALL NOT be laundered into a pass: when the closed-form arbiter
exists it is authoritative over OCCT; a native result is exonerated only when it
POSITIVELY matches exact math while OCCT does not.

#### Scenario: A native mass matching OCCT and the closed form is AGREED

- GIVEN an in-scope trial whose native solid is watertight with positive volume
- WHEN native volume/area/centroid match both the OCCT `BRepGProp` measurement and the closed-form analytic ground truth within the deflection-matched tolerance
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A non-watertight native mesh with a valid OCCT solid is HONESTLY-DECLINED, not a failure

- GIVEN a trial (in-scope or a decline-exerciser) whose native mesh is not watertight, so the native mass path returns `valid == 0`
- WHEN the OCCT build of the same tuple is a valid measurable solid
- THEN the trial SHALL be classified HONESTLY-DECLINED, logged with its seed and tuple, and SHALL NOT fail the bar — a missing native mass is never counted as a wrong mass

#### Scenario: A watertight native mass that fails the closed form is a silent-wrong-result

- GIVEN a trial whose native solid is watertight with positive volume (`valid == 1`)
- WHEN native volume, area, or centroid disagrees with the closed-form analytic ground truth beyond the deflection bound
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, family/parameter tuple, and all three measurements (native / OCCT / analytic) so the native fault is reproducible

#### Scenario: A native mass vindicated by exact math is not a false native fault

- GIVEN a trial where native differs from OCCT `BRepGProp` but native matches the closed-form analytic ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-mass bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED — and SHALL exit 0
IF AND ONLY IF the bar holds: **DISAGREED == 0** across the batch, with real
per-family coverage (at least the BOX, NGON_PRISM, CYLINDER, CONE, SPHERE, LOFT, and
REVOLVE families each exercised with at least one AGREED trial) proven across **at
least two distinct seeds**. Any DISAGREED (and any ORACLE-INACCURATE) SHALL be
reported with its seed so it is reproducible. The harness SHALL NOT weaken a
tolerance, silently cap the batch, or drop trials to make the bar pass: any capped or
skipped trial SHALL be logged. The honest scope SHALL be logged explicitly — the
native inertia decline (native principal moments delegate to OCCT) and the
mesh-vs-exact deflection boundary (why a curved family uses the deflection-matched
tolerance) SHALL appear in the summary, not be silently omitted.

#### Scenario: Zero disagreements across multiple seeds with real family coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every in-scope family
- WHEN no trial is classified DISAGREED and each family has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary, log the native inertia decline and the deflection boundary as honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, family/parameter tuple, and the native/OCCT/analytic measurements, so the silent-wrong-mass is reproducible and not laundered into a pass

#### Scenario: Honest scope and any dropped trials are logged, not silently omitted

- GIVEN a run in which some trials are decline-exercisers (BOTH-DECLINED) or the batch is capped for time
- WHEN the harness summarises
- THEN the native inertia decline, the mesh-vs-exact deflection boundary, and any capped or skipped trial SHALL be logged explicitly in the summary, so no honest exclusion is hidden

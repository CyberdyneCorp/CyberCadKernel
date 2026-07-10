# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random defected-solid generator for healing fuzzing

The healing differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of base-solid
parameters, defect families, and defect magnitudes on any machine.

Each generated trial SHALL parameterise (a) a VALID base solid whose exact geometry is
KNOWN by construction — a unit-scale cube (V=1), a random axis-aligned box, or a random
convex N-gon prism — so its enclosed volume and surface area have a closed form, and
(b) exactly one injected DEFECT drawn from the healer's defect families: sew-jitter
(coincident-within-tolerance soup), a flipped face, a near-miss seam gap (in-band and
out-of-band relative to the bridge budget), a redundant short-edge split, a redundant
collinear boundary vertex, a single missing planar face, two opposite missing faces,
two adjacent missing faces, and a beyond-tolerance gap. Every defect family SHALL be
**shape-preserving**: a correct heal reconstructs the ORIGINAL known solid, so the
closed-form volume+area of the undamaged solid is an engine-independent ground-truth
arbiter. The generator SHALL emit plain parameter POD consumed identically by the
native face-soup builder (`heal::healShell` input) and the OCCT compound builder
(`sewAndFix` input), and SHALL be OCCT-free in that sense.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the healing fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` base-solid + defect trials
- THEN the two sequences SHALL be byte-identical (same base families, same defect families and numeric magnitudes in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Every defect is shape-preserving so the closed form is a valid arbiter

- GIVEN a generated defect
- WHEN it is inspected
- THEN it SHALL perturb only the REPRESENTATION of the known base solid (jitter, orientation, a redundant vertex/edge, a near-miss gap, a missing face) and SHALL NOT change the intended solid's true geometry, so a correct heal reproduces the base solid's exact closed-form volume and area

### Requirement: Two independent healings of each defect with a closed-form ground-truth arbiter

For each trial the harness SHALL heal the SAME injected defect two independent ways and
arbitrate the result against the closed-form ground truth of the undamaged base solid.
The **native** healing SHALL call the OCCT-free `heal::healShell(shape, opts)` on the
native `topology::Shape` face soup, with the family-appropriate opt-in flag enabled
(gap-bridge budget for a seam gap, `capPlanarHoles`/`capMultiplePlanarHoles` for a
missing face, `shortEdgeMergeLen` for a short-edge split, `removeCollinearVerts` for a
collinear vertex). The **oracle** healing SHALL call `cyber::occt::sewAndFix(compound,
tolerance)` (`BRepBuilderAPI_Sewing → ShapeFix_Shell → ShapeFix_Solid`, measured by
`BRepGProp`) on the SAME defect built as a TopoDS compound at the SAME tolerance. The
**ground-truth arbiter** SHALL be the closed-form volume+area of the undamaged base
solid (a THIRD, engine-independent value): a native `Healed` result's enclosed volume
(from the native tessellator at a fixed deflection) SHALL be compared to that closed
form, NOT merely to OCCT. Where a defect family requires reconstructing a specific face
(a synthesized planar cap), the harness MAY provide OCCT the analogous reference cap so
the two engines are compared apples-to-apples.

The native tolerances SHALL be FIXED (the sew/weld `tolerance`, the bridge budget, the
short-edge merge length, and the mesh-deflection volume band) and SHALL NOT be widened
to force a pass.

#### Scenario: Native and OCCT heal the identical defect at the identical tolerance

- GIVEN one generated trial
- WHEN the native builder assembles the defect as a native face soup and `heal::healShell` heals it, and the OCCT builder assembles the SAME defect as a TopoDS compound and `sewAndFix` heals it at the same tolerance
- THEN both SHALL have operated on the SAME defect of the SAME known base solid, and the base solid's closed-form volume+area SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An in-scope defect heals to the known solid under both engines

- GIVEN a sew-jitter, flipped-face, in-band seam-gap, short-edge, collinear-vertex, single-hole, or two-opposite-hole defect of a known base solid
- WHEN native `healShell` returns `Healed` and OCCT `sewAndFix` returns a valid solid
- THEN the native repaired solid's enclosed volume SHALL match the base solid's closed-form volume within the fixed deflection band, the OCCT solid SHALL concur, and the trial SHALL be classified AGREED

#### Scenario: An out-of-scope defect declines under native and is not a wrong repair

- GIVEN a beyond-tolerance gap, a two-adjacent-missing-face defect (non-planar wrap), or a seam gap beyond the bridge budget
- WHEN native `healShell` returns `Unhealed` with the measured residual and the input unchanged
- THEN the trial SHALL be classified HONESTLY-DECLINED (native defers to OCCT), and this SHALL be a first-class outcome, never a bar failure

### Requirement: Equal-or-more-conservative classifier arbitrated by the closed-form ground truth

The harness SHALL classify each healing trial into EXACTLY ONE bucket at the fixed
tolerances, with the undamaged base solid's closed-form volume+area as the PRIMARY
correctness oracle, enforcing the **equal-or-more-conservative** contract (native must
never emit a watertight solid that differs from the known truth; an honest decline is
always safe):

- **AGREED** — native returned `Healed` with a watertight solid whose volume+area match
  the closed-form ground truth within the fixed band AND OCCT also formed a valid solid
  (agreement), OR native returned `Unhealed` where the defect is genuinely out of scope
  and OCCT also failed to form a valid closed solid at the same tolerance (parity of
  decline), OR native honestly declined a defect that OCCT aggressively repaired to the
  SAME honest ground-truth solid (native MORE conservative — a safe deferral that the
  opt-in flag is separately proven to recover).
- **HONESTLY-DECLINED** — native returned `Unhealed` (input unchanged, measured
  residual) and defers to OCCT. First-class, logged, NOT a bar failure.
- **DISAGREED** — native returned a watertight `Healed` solid whose volume or area does
  NOT match the closed-form ground truth beyond the fixed band. A genuine SILENT WRONG
  REPAIR — the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native `Healed` matches the closed-form ground truth exactly
  while OCCT's repair does NOT. The native repair is CORRECT and vindicated by exact
  math; OCCT is the outlier. Logged in full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a defect that both engines refuse to close.
- **ORACLE_UNRELIABLE** — an in-scope core case whose OCCT oracle could not be obtained
  or does not match the closed form. FAILS the bar (investigate; never laundered).

#### Scenario: A native repair matching OCCT and the closed form is AGREED

- GIVEN an in-scope defect trial
- WHEN the native repaired solid matches both OCCT and the closed-form ground truth within the fixed band
- THEN the trial SHALL be classified AGREED and SHALL contribute to that base family's and that defect family's coverage count

#### Scenario: A watertight native repair that fails the closed form is a silent-wrong-result

- GIVEN a healing trial whose native `healShell` returned a watertight `Healed` solid
- WHEN that solid's volume or area disagrees with the closed-form ground truth beyond the fixed band
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and defect descriptor so the native fault is reproducible

#### Scenario: Native declining where OCCT aggressively repairs is more-conservative, not a fault

- GIVEN a short-edge or collinear-vertex defect with the corresponding opt-in flag OFF
- WHEN native returns `Unhealed` (nothing collapsed/removed, input unchanged) while OCCT's sewer/ShapeFix aggressively closes the same soup
- THEN provided OCCT's closure is the honest ground-truth solid (or OCCT also declined), the trial SHALL be classified AGREED (native more conservative — never a wrong repair), and native SHALL NEVER be penalised for the safe deferral

#### Scenario: A native repair vindicated by exact math is not a false native fault

- GIVEN a trial where the native `Healed` solid differs from OCCT's repair but matches the closed-form ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-repair bar, and logged honest scope

The harness SHALL print a coverage summary — the per-base-family and per-defect-family
counts of AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED
trials — and SHALL exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0 AND
ORACLE_UNRELIABLE == 0** across the batch, with real coverage (each base family and
each of the exercised defect families with at least one non-error trial) proven across
**at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED (and any
ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL be reported with its seed so it is
reproducible. The harness SHALL NOT weaken a tolerance, silently cap the batch, or drop
trials to make the bar pass. The honest scope SHALL be logged explicitly — the
shape-preserving-defect restriction and the out-of-scope declines (beyond-tolerance
gap, two-adjacent-missing-face non-planar wrap, out-of-budget seam gap) SHALL appear in
the summary.

#### Scenario: Zero silent-wrong repairs across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every base family and every exercised defect family
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each base family and each defect family has at least one non-error trial
- THEN the harness SHALL print the per-family coverage summary, log the honest scope, and exit 0

#### Scenario: Any silent-wrong repair fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and defect descriptor, so the silent-wrong-repair is reproducible and not laundered into a pass

#### Scenario: Honest scope and declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-scope declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the shape-preserving-defect restriction and the out-of-scope defect declines SHALL be logged explicitly in the summary, so no honest exclusion is hidden

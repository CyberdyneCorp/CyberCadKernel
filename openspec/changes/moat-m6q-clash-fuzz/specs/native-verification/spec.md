# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random interference/clash pair generator

The interference/clash differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of families, regimes, solid
dimensions, and relative placements on any machine.

Each generated trial SHALL parameterise (a) a solid FAMILY — box, regular n-gon prism,
cylinder, or sphere — built at random valid dimensions as BOTH a watertight
outward-oriented native boundary mesh AND the matching OCCT primitive solid; (b) a target
interference REGIME — CLEAR (positive clearance gap), TOUCHING (zero-volume boundary
contact), or CLASH (positive-volume interior overlap); and (c) a relative rigid placement
(rotation + translation) applied IDENTICALLY to the native mesh and the OCCT solid that
lands the pair in the target regime.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the interference fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` family + regime + dimension + placement trials
- THEN the two sequences SHALL be byte-identical (same families, regimes, numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Both operands are the same body in the same pose on both sides

- GIVEN a generated trial
- WHEN the native mesh operand and the OCCT solid operand are built and body B is placed by the shared rigid transform
- THEN both sides SHALL represent the SAME two solids at the SAME dimensions in the SAME relative pose, so any state or measurement difference is attributable to the classifier, not to divergent inputs

### Requirement: Three-regime coverage with a stressed TOUCHING knife-edge

The generator SHALL cover all three interference regimes for every family, and SHALL
stress the TOUCHING regime — the boundary-coincident knife-edge — under randomised
jitter, not only a single hand-built flush pose. For the TOUCHING regime the generator
SHALL draw the exact flush pose and a signed jitter about it: an exact flush contact
(expected TOUCHING), a slight penetration deeper than the contact band (expected CLASH),
and a slight separation (expected CLEAR). The flush contact SHALL be kept within the
op's certified assembly-mate contact envelope — the contact footprint of one operand
contained within the other's face — so at least one operand has a boundary vertex on the
other's boundary (seated / coincident / contained / slid contact).

A boundary-coincident flush pose SHALL be classified TOUCHING (a shared face reads `On`),
NEVER a CLASH — the coplanar-safe property the landed op guarantees via its B3 ON-band
membership.

#### Scenario: An exact flush contact reads TOUCHING, never a clash

- GIVEN two solids placed so their boundaries are exactly coincident over a positive-area contact region within the certified contact envelope
- WHEN the native classifier and the OCCT oracle evaluate the pair
- THEN both SHALL report TOUCHING (zero-volume boundary contact), the native classifier SHALL NOT report CLASH, and the trial SHALL be classified AGREED

#### Scenario: A slight penetration past the contact band reads CLASH on both sides

- GIVEN a flush pose nudged into interpenetration by more than the classifier's contact band
- WHEN the native classifier and the OCCT oracle evaluate the pair
- THEN both SHALL report CLASH, and the trial SHALL be classified AGREED

### Requirement: OCCT and closed-form oracles with a faceting-aware convergence rule

For each trial the harness SHALL arbitrate the native classifier against an OCCT oracle
AND, where the pair has one, a closed-form ground truth. The **native** verdict SHALL be
the CLASH / TOUCHING / CLEAR state (and min clearance) from the header-only classifier on
the native mesh. The **OCCT oracle** SHALL combine `BRepAlgoAPI_Common` volume (positive
⇒ interior overlap ⇒ CLASH) with `BRepExtrema_DistShapeShape` (min boundary distance ~0
with no overlap ⇒ TOUCHING; positive ⇒ CLEAR), and SHALL decline if the OCCT boolean
itself fails. The **closed-form arbiter** SHALL be, where present, the exact box∩box
axis-aligned intersection-box volume + axis gap, or the exact sphere∩sphere lens volume +
centre-distance regime.

Because the native classifier consumes a deflection-bounded planar-facet mesh while OCCT
keeps a true analytic B-rep, a curved-pair (cylinder / sphere) TOUCHING↔CLEAR straddle
within the deflection facet band SHALL be treated as a CONVERGENT match, NOT a
disagreement. The classifier's contact band SHALL be its own value
(`max(1e-9·scale, 2·deflection)`) and SHALL NOT be widened to force a pass.

#### Scenario: A closed-form arbiter overrides a sub-tolerance OCCT rounding

- GIVEN a pair with a closed-form arbiter (box∩box or sphere∩sphere) whose native state matches the exact closed form while OCCT reports a different state (e.g. a sub-tolerance overlap OCCT rounds away)
- WHEN the classifier attributes the state difference
- THEN the trial SHALL be classified ORACLE-INACCURATE (native vindicated by exact math), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

#### Scenario: A curved-pair facet straddle is a convergent match

- GIVEN a cylinder or sphere pair at a near-flush contact where the native faceted mesh reads CLEAR (inset by the facet chord error) while OCCT's exact B-rep reads TOUCHING
- WHEN the classifier compares the two states
- THEN the trial SHALL be classified FACET-CONVERGENT (a convergent soft-state match), logged, and SHALL NOT fail the bar, with no tolerance widened

### Requirement: Six-way classifier and honest-decline probe

The harness SHALL classify each interference trial into EXACTLY ONE bucket:

- **AGREED** — native state == OCCT state (and, where present, the closed-form regime).
- **HONESTLY-DECLINED** — native `meshInterference` returned `Unknown` (a non-watertight
  or ambiguous operand) while OCCT reported a crisp verdict; native falls through to
  OCCT. First-class, logged, NOT a bar failure.
- **DISAGREED** — native reported a crisp state differing from OCCT on a HARD boundary
  (a CLASH that OCCT calls CLEAR/TOUCHING, or vice-versa) and the closed-form arbiter (if
  any) sides with OCCT. A genuine silent-wrong clash — the failure this harness exists to
  catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form ground truth while OCCT does
  NOT. Native vindicated; logged; NOT a bar failure.
- **FACET-CONVERGENT** — a curved-pair TOUCHING↔CLEAR straddle within the deflection facet
  band. Logged; NOT a failure.
- **BOTH-DECLINED** — both engines declined.

A minority of trials SHALL be an HONEST-DECLINE PROBE that hands the native classifier a
deliberately NON-WATERTIGHT operand. The classifier's watertight precondition SHALL make
it DECLINE (`Unknown`) rather than emit a verdict; if it instead emits a crisp state the
trial SHALL be classified DISAGREED (the "never guess on ambiguous mesh evidence /
a wrong overlap is NEVER returned" contract was violated).

#### Scenario: A non-watertight operand declines, never a guessed clash

- GIVEN a trial whose native operand mesh has been made non-watertight (an open shell)
- WHEN the native classifier evaluates it
- THEN it SHALL return `Unknown` (an honest decline), the trial SHALL be classified HONESTLY-DECLINED, and the harness SHALL NOT count a guessed CLASH / TOUCHING / CLEAR state

#### Scenario: A hard state split backed by the closed form is a silent-wrong clash

- GIVEN a trial where native reports a crisp state that differs from OCCT on a hard boundary and the closed-form arbiter agrees with OCCT
- WHEN the harness classifies it
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/regime/parameter tuple so the fault is reproducible

### Requirement: Coverage summary and zero-silent-wrong-clash bar

The harness SHALL print a coverage summary — the per-[family × regime] counts of
AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / FACET-CONVERGENT /
BOTH-DECLINED trials for each of the four families × three regimes — and SHALL exit 0 IF
AND ONLY IF the bar holds: **DISAGREED == 0** across the batch, with every populated
[family × regime] cell truly exercised (at least one AGREED / ORACLE-INACCURATE /
FACET-CONVERGENT — never an all-decline cell), proven across **at least two distinct
seeds**, N ≥ 60 per seed. Any DISAGREED (and any ORACLE-INACCURATE) SHALL be reported
with its seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently
cap the batch, or drop trials to make the bar pass.

#### Scenario: Zero silent-wrong clashes across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every family × regime cell
- WHEN no trial is classified DISAGREED and each populated cell has at least one truly-exercised (AGREED / ORACLE-INACCURATE / FACET-CONVERGENT) trial
- THEN the harness SHALL print the per-cell coverage summary and exit 0

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are honest declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-cell decline counts SHALL appear in the summary, so no honest exclusion is hidden

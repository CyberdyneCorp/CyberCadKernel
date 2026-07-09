# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random operand-pair, boolean-op, and rigid-transform generator for transformed-boolean fuzzing

The transformed-boolean differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of operand-pair parameters,
boolean ops, and rigid transforms on any machine.

Each generated trial SHALL parameterise (a) a VALID pair of ALL-PLANAR operand solids
drawn from the families the native planar boolean serves — a `BOX` pair, an `NGON` prism
(n∈[3,7]) with a straddling box, and a concave `LSHAPE` prism with a straddling box —
built IDENTICALLY under both engines via the public `cc_solid_extrude` facade and
positioned so `A ∩ B` is a clean transversal overlap valid for all three ops; (b) exactly
one boolean op drawn from `FUSE`, `CUT`, `COMMON`; and (c) exactly one rigid transform
drawn from `IDENTITY`, `TRANSLATE`, `ROTATE` (about a random axis through a random
centre), and `MIRROR` (through a random plane). The first `T_COUNT` trials SHALL force
each transform kind so every kind is exercised.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the transformed-boolean fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` operand-pair + boolean-op + transform trials
- THEN the two sequences SHALL be byte-identical (same operand families, same op kinds, same transform kinds and numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Operands are built identically under both engines and kept in the native planar domain

- GIVEN a generated operand pair
- WHEN it is built under `cc_set_engine(0)` (OCCT) and under `cc_set_engine(1)` (NativeEngine) from the same `cc_solid_extrude` parameters
- THEN both engines SHALL produce the same nominal all-planar solids (so the native planar BSP-CSG boolean is EXERCISED, not declined for a curved face), and the composed op under each engine acts on a topologically-identical operand pair

### Requirement: Composed transform-then-boolean driven under both engines with a rigid-invariant closed-form arbiter

For each trial the harness SHALL run, under EACH engine (`cc_set_engine(0)` OCCT oracle
and `cc_set_engine(1)` NativeEngine, through the public `cc_boolean` + transform facade),
BOTH the baseline boolean `R0 = boolean(A, B, op)` and the transformed boolean
`RT = boolean(T(A), T(B), op)`, where `T` is applied to each operand via
`cc_translate_shape` / `cc_rotate_shape_about` / `cc_mirror_shape`. Every resulting shape
SHALL be measured (`cc_mass_properties`) under the SAME engine that built it — a shape
built under one engine MUST NOT be measured under the other (the OCCT mass-property
adapter unwraps the shared registry entry as an OCCT shape and would dereference a native
body as garbage).

The harness SHALL arbitrate each trial against a THIRD engine-independent CLOSED-FORM
ground truth computed in plain fp64:

- **PRIMARY — the rigid invariant.** A rigid transform `T` is an isometry that preserves
  volume and area, and a boolean commutes with it (`T(A) ∘ T(B) == T(A ∘ B)`); therefore
  the native transformed-boolean's enclosed VOLUME SHALL equal the native untransformed
  boolean's volume, `|RT|_vol == |R0|_vol`, to a TIGHT tolerance. Because the operands are
  all-planar prisms the native mesh is exact, so this invariant is the certifying
  correctness signal. The AREA invariant `|RT|_area == |R0|_area` SHALL be held to a
  meshed-facet-weld bound (the canonical and located native meshes can weld a marginally
  different facet set while the volume stays exact).
- **SECONDARY — engine agreement.** The native `RT` volume and area SHALL match the OCCT
  `RT` volume and area within the FIXED relative tolerance the sibling
  `native_boolean_fuzz` harness proved (`2e-2`), which SHALL NOT be widened to launder a
  disagreement.

#### Scenario: The rigid invariant is checked without an oracle

- GIVEN a composed trial whose native baseline `R0` and transformed `RT` are both valid solids
- WHEN the harness compares their enclosed volumes
- THEN it SHALL require `|RT|_vol == |R0|_vol` to the tight volume tolerance (a rigid transform preserves volume and commutes with the boolean), independent of any OCCT result

#### Scenario: A shape is always measured under the engine that built it

- GIVEN a shape id produced under one engine (native or OCCT)
- WHEN the harness measures its mass properties
- THEN it SHALL activate the owning engine before the measurement and restore the default engine afterwards, so a native body is never unwrapped by the OCCT mass-property adapter

### Requirement: Agree / honestly-declined / DISAGREE transformed-boolean classifier arbitrated by the rigid invariant

The harness SHALL classify each transformed-boolean trial into EXACTLY ONE bucket at the
fixed tolerances, with the rigid invariant as the primary correctness oracle:

- **AGREED** — the native `RT` upholds the rigid VOLUME + AREA invariant against native
  `R0` AND matches the OCCT `RT` within the secondary tolerance.
- **HONESTLY-DECLINED** — the native planar boolean returned NULL on a composed op it
  scopes out (a near-tangent / degenerate pose its self-verify rejects) while OCCT shipped
  a valid solid. First-class, logged, NOT a bar failure.
- **BOTH-DECLINED** — neither engine produced a usable result (an operand one engine could
  not build, or a genuinely empty `COMMON`).
- **ORACLE_UNRELIABLE** — the native `RT` upheld the tight rigid invariant while OCCT
  broke its OWN rigid invariant (OCCT the pathological outlier). Native is vindicated;
  logged in full, gated off, NOT a native fault.
- **DISAGREED** — the native `RT` broke the rigid invariant, or the native `RT` did not
  match the OCCT `RT` within the secondary tolerance while OCCT upheld its own invariant.
  A genuine SILENT WRONG RESULT — the failure this harness exists to catch. FAILS the bar.

#### Scenario: A native result upholding the invariant and matching OCCT is AGREED

- GIVEN a composed trial whose native `RT` is a valid solid
- WHEN it upholds the rigid volume + area invariant against native `R0` and matches the OCCT `RT` within the fixed tolerances
- THEN the trial SHALL be classified AGREED and SHALL contribute to that operand family's, that op's, and that transform kind's coverage count

#### Scenario: A native result that breaks the rigid invariant is a silent-wrong-result

- GIVEN a composed trial whose native `RT` is a valid solid
- WHEN its enclosed volume differs from the native `R0` volume beyond the tight invariant tolerance (a dropped / mis-composed / mis-oriented operand Location), or the native `RT` disagrees with the OCCT `RT` beyond the secondary tolerance while OCCT upholds its own invariant
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and full operand + transform descriptor so the native fault is reproducible

#### Scenario: A native result vindicated by the invariant is not a false native fault

- GIVEN a trial where the native `RT` upholds the tight rigid invariant but OCCT's own `RT` breaks the invariant (OCCT the outlier)
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE_UNRELIABLE, logged in full with its seed, and SHALL NOT be counted as a native fault

### Requirement: Coverage summary, zero-silent-wrong-result bar, and logged honest scope

The harness SHALL print a coverage summary — the per-operand-family, per-op, and
per-transform-kind counts of AGREED and DISAGREED trials — and SHALL exit 0 IF AND ONLY IF
the bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real
coverage (each of BOX, NGON, LSHAPE; each of FUSE, CUT, COMMON; and each of IDENTITY,
TRANSLATE, ROTATE, MIRROR exercised with at least one AGREED trial) proven across **at
least two distinct seeds**. Any DISAGREED (and any ORACLE_UNRELIABLE) SHALL be reported
with its seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently
cap the batch, or drop trials to make the bar pass; the tight rigid VOLUME invariant SHALL
remain the certifying arbiter. The honest scope SHALL be logged explicitly — the
near-tangent / degenerate native declines and any ORACLE_UNRELIABLE outlier SHALL appear
in the summary.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every operand family, every boolean op, and every transform kind
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each family, each op, and each transform kind has at least one AGREED trial
- THEN the harness SHALL print the per-family / per-op / per-transform coverage summary, log the honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and full operand + transform descriptor, so the silent-wrong-result is reproducible and not laundered into a pass

#### Scenario: Honest scope is logged, not silently omitted

- GIVEN a run in which some trials are scoped-out declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the near-tangent / degenerate native declines SHALL be counted and reported in the summary, so no honest exclusion is hidden

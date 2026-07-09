# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random base-solid and direct-model-op generator for direct-modeling fuzzing

The direct-modeling differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of base-solid parameters and
direct-model operations on any machine.

Each generated trial SHALL parameterise (a) a VALID, non-degenerate base solid drawn
from the analytic families the native construction path builds — a `BOX` prism, an
`NGON` prism, a `CYLINDER`, and a `CONE` frustum — built IDENTICALLY under both engines
via the public `cc_solid_extrude` / `cc_solid_revolve` facade so the direct-model op
operates on topologically-identical operands; and (b) exactly one direct-model op drawn
from `SPLIT` (`cc_split_plane`), `OFFSET` (`cc_replace_face` parallel cap offset), and
`PROJECT` (`cc_project_point_on_face`), with random op parameters (an axis-aligned or
oblique cut plane and keep side; a top/bottom cap and a signed grow/trim offset; a target
face and an exterior source point).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the direct-modeling fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` base-solid + direct-model-op trials
- THEN the two sequences SHALL be byte-identical (same base families, same op kinds and numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Operands are built identically under both engines

- GIVEN a generated base solid
- WHEN it is built under `cc_set_engine(0)` (OCCT) and under `cc_set_engine(1)` (NativeEngine) from the same `cc_solid_extrude` / `cc_solid_revolve` parameters
- THEN both engines SHALL produce the same nominal solid, so the direct-model op under each engine acts on a topologically-identical operand and the results are directly comparable

### Requirement: Each direct-model op driven under both engines with a closed-form arbiter

For each trial the harness SHALL run the generated direct-model op ONCE under the OCCT
engine (`cc_set_engine(0)`, the oracle) and ONCE under the NativeEngine
(`cc_set_engine(1)`, the system under test) through the public `cc_*` facade, and SHALL
compare the two results against a THIRD engine-independent CLOSED-FORM arbiter computed
in plain fp64. Every resulting shape SHALL be measured (`cc_mass_properties` /
`cc_bounding_box`) under the SAME engine that built it — a shape built under one engine
MUST NOT be measured under the other (the OCCT mass-property adapter unwraps the shared
registry entry as an OCCT shape and would dereference a native body as garbage).

The closed-form arbiters SHALL be:

- **SPLIT** — for an axis-aligned plane through a `BOX` or `NGON` prism, the exact
  half-space keep-volume (base area × clipped height, or a rectangular-slab clip); and for
  EVERY family a PARTITION-CLOSURE identity `V(keep+) + V(keep−) == V(whole)` obtained by
  cutting the native operand both ways.
- **OFFSET** — for the CONSTANT-cross-section families (`BOX` / `NGON` / `CYLINDER`), the
  exact volume change `ΔV == capArea · offset` of moving a planar cap plane by `offset`
  along its outward normal. (A `CONE` frustum's conical wall changes radius with height,
  so `capArea·offset` is NOT exact there — the cone offset is arbitrated by native==OCCT
  alone, with no false closed-form claim.)
- **PROJECT** — for a PLANAR face the foot `p − ((p−o)·n̂)·n̂` and distance `|(p−o)·n̂|`;
  for a `CYLINDER` lateral face the axis-radial foot at radius `R`.

The tolerances SHALL be FIXED and tight (native-vs-OCCT volume relative `2e-2`, area
relative `3e-2`, bbox absolute `1.5e-2`; result-vs-exact-math relative `5e-3`; projection
foot + distance absolute `1e-6`) and SHALL NOT be widened to force a pass.

#### Scenario: Native, OCCT, and the closed form all evaluate the identical operand + op

- GIVEN one generated trial
- WHEN the op runs under the OCCT engine on the OCCT-built operand, under the NativeEngine on the native-built operand, and the fp64 closed form is applied to the operand's known analytic geometry
- THEN all three SHALL have operated on the SAME nominal operand and the SAME op parameters, and the closed-form result SHALL be the correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An axis-aligned split keeps the exact half-space volume and both sides close

- GIVEN a `BOX` or `NGON` prism cut by an axis-aligned plane at a random interior coordinate, keeping a random side
- WHEN the native keep-side volume is compared to the exact half-space keep-volume and to the OCCT keep-side, and the native keep+ and keep− volumes are summed
- THEN the native keep-side volume SHALL match the exact half-space volume and the OCCT keep-side within the fixed tolerances, and `V(keep+) + V(keep−)` SHALL equal `V(whole)` within the volume tolerance

#### Scenario: A cap trim matches OCCT and the exact volume; a cap grow surfaces the OCCT-facade limitation

- GIVEN a constant-cross-section base whose planar cap is moved by a signed `offset` via `cc_replace_face`
- WHEN `offset < 0` (a TRIM) the native, OCCT, and `capArea·offset` volumes are compared, AND when `offset > 0` (a GROW) the same comparison is made
- THEN a TRIM SHALL be AGREED (native == OCCT == exact math within tolerance), and a GROW SHALL be ORACLE-INACCURATE — the native result matches the exact `capArea·offset` math while the OCCT half-space-cut adapter (which can only remove material) does not — logged in full, NOT a bar failure

#### Scenario: A point projects to the exact planar or cylindrical foot; a cone lateral honestly declines

- GIVEN an exterior source point and a target face
- WHEN the target is a planar cap/side face or a `CYLINDER` lateral face, the native foot + distance are compared to the OCCT projection and the closed-form foot; and when the target is a `CONE` lateral face, the native projection is requested
- THEN a planar or cylindrical target SHALL be AGREED (native == OCCT == closed form within `1e-6`), and a cone lateral target SHALL cause the native service to honestly decline (`valid == 0`) → the trial SHALL be HONESTLY-DECLINED with OCCT as the fallback oracle, NEVER a DISAGREE

### Requirement: Agree / honestly-declined / DISAGREE direct-model classifier arbitrated by the closed-form ground truth

The harness SHALL classify each direct-model-op trial into EXACTLY ONE bucket at the
fixed tolerance, with the closed-form result as the correctness oracle where one exists:

- **AGREED** — the native result matches the OCCT result within tolerance AND, where a
  closed form exists, matches the exact math.
- **HONESTLY-DECLINED** — the native service returned NULL/invalid on a case the native
  direct-model slice scopes out (a cone-lateral projection, a cone cap offset, an
  out-of-scope split configuration) and OCCT served as the fallback oracle. First-class,
  logged, NOT a bar failure.
- **DISAGREED** — the native service returned a result that does NOT match OCCT / the
  exact math (a wrong-volume split, a wrong keep side, a failed partition closure, a
  wrong-amount cap offset, a wrong foot, or a decline of a face it is scoped to serve).
  A genuine SILENT WRONG EDIT — the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — the native result matches the exact closed form but OCCT does
  NOT (the `cc_replace_face` grow limitation). The native result is CORRECT and
  vindicated by exact math; OCCT is the outlier. Logged in full, NOT a native fault, NOT
  a bar failure.
- **BOTH-DECLINED** — a case where neither engine produced a usable result (e.g. an
  operand one engine could not build, or a face id neither engine could resolve).
- **ORACLE_UNRELIABLE** — a core case whose OCCT result itself disagrees with the closed
  form (e.g. OCCT served a projection the native slice matched exactly while OCCT was the
  outlier with no vindicating decline). FAILS the bar (investigate; never laundered into
  a pass).

#### Scenario: A native result matching OCCT and the closed form is AGREED

- GIVEN an in-scope direct-model-op trial
- WHEN the native result matches both the OCCT result and (where one exists) the closed-form value within the fixed tolerance
- THEN the trial SHALL be classified AGREED and SHALL contribute to that base family's and that op's coverage count

#### Scenario: A native result that fails the closed form is a silent-wrong-result

- GIVEN a direct-model-op trial whose native service returned a result
- WHEN that result disagrees with the closed-form value beyond the fixed tolerance (a wrong split volume/side, a failed partition closure, a wrong cap-offset volume, or a wrong projection foot/distance), or the native service declined a face it is scoped to serve
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and base descriptor so the native fault is reproducible

#### Scenario: A native result vindicated by exact math is not a false native fault

- GIVEN a trial where the native result differs from OCCT but matches the closed-form value while OCCT does not (the `cc_replace_face` grow limitation)
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-result bar, and logged honest scope

The harness SHALL print a coverage summary — the per-base-family and per-op counts of
AGREED and DISAGREED trials — and SHALL exit 0 IF AND ONLY IF the bar holds:
**DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real coverage (each
of BOX, NGON, CYLINDER, CONE and each of SPLIT, OFFSET, PROJECT exercised with at least
one AGREED trial) proven across **at least two distinct seeds**. Any DISAGREED (and any
ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL be reported with its seed so it is
reproducible. The harness SHALL NOT weaken a tolerance, silently cap the batch, or drop
trials to make the bar pass. The honest scope SHALL be logged explicitly — the
cone-lateral-projection decline and the `cc_replace_face` grow ORACLE-INACCURATE
signature SHALL appear in the summary.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every base family and every direct-model op
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each base family and each op has at least one AGREED trial
- THEN the harness SHALL print the per-family / per-op coverage summary, log the honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and base descriptor, so the silent-wrong-result is reproducible and not laundered into a pass

#### Scenario: Honest scope and the surfaced OCCT-facade limitation are logged, not silently omitted

- GIVEN a run in which some trials are scoped-out declines (HONESTLY-DECLINED / BOTH-DECLINED) and some are `cc_replace_face` grows (ORACLE-INACCURATE)
- WHEN the harness summarises
- THEN the cone-lateral-projection decline and the ORACLE-INACCURATE grow limitation SHALL be logged explicitly in the summary, so no honest exclusion or surfaced oracle limitation is hidden

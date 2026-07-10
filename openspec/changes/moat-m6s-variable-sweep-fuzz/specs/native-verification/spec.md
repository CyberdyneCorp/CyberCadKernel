# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random variable-section-sweep generator

The variable-section-sweep differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL NOT
read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64 seeding a
xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED` (argv/env-overridable,
with a fixed default). Re-running the harness with the same seed and batch size SHALL
produce a **byte-identical** sequence of families, profiles, spines, and guide rails on any
machine.

Each generated trial SHALL parameterise (a) a morph section pair — profile A (spine start)
and profile B (spine end) with the SAME vertex count — for one of the certified families
{circle→circle morph, regular-polygon→polygon morph, section-A→section-B morph with
per-vertex radii}, (b) a spine — a straight `+Z` polyline or a smooth planar quarter-arc,
(c) an OPTIONAL guide rail whose splay scales every section uniformly, and (d) their
dimensions, all so the swept solid's volume is either exactly known (straight families) or
OCCT-arbitrable (curved family). The sweep SHALL be applied through the SAME public
`cc_variable_sweep` facade the application calls, under BOTH engines — not by shipping a
native builder result directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the variable-sweep fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` family + profile + spine + guide trials
- THEN the two sequences SHALL be byte-identical (same families, same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each straight-spine sweep has a closed-form volume

- GIVEN a generated straight-spine trial (circle / polygon / section-A→B morph, with or without a guide)
- WHEN its profiles, spine length, and guide-scale law are inspected
- THEN the swept solid's volume SHALL be given by an exact closed form — the cross-section polygon area `A(f) = polyArea((A_k + (B_k−A_k)f)·s(f))` integrated over the spine as `∫₀¹ A(f)·H df` by an exact composite Simpson (A(f) is degree ≤4 in f), or the smooth truncated-cone `πH/3(r0²+r0r1+r1²)` for the circle family — so that closed form is an engine-independent ground-truth arbiter

### Requirement: Both-engine facade drive with closed-form and OCCT oracles

For each trial the harness SHALL sweep the SAME inputs two ways through the public
`cc_variable_sweep` facade and arbitrate against the ground truth. The **native** candidate
SHALL be produced under `cc_set_engine(1)` (the NativeEngine RMF/perp-framed guide-scaled
morph tube with its honest OCCT fallback), measured by `cc_mass_properties` and
`cc_tessellate`. The **OCCT oracle** SHALL be produced under `cc_set_engine(0)` through the
SAME facade (OCCT `BRepOffsetAPI_MakePipeShell` multi-section on the SAME profiles, spine,
and guide), measured by `cc_mass_properties`. The **ground-truth arbiter** for a
straight-spine family SHALL be the closed-form volume, compared to the native volume, NOT
merely to OCCT; the curved family SHALL be arbitrated against OCCT alone (a
deflection-bounded band) plus the engine-independent watertight / Euler χ=2 invariants.

Because the `cc_variable_sweep` facade under the native engine forwards to OCCT on an
out-of-envelope pose (never returning 0 to signal a decline), the harness SHALL read the
native BUILDER's decline signal directly (a read-only `build_variable_sweep` probe on the
same arguments; a NULL result means the native arm declined and the facade shipped the OCCT
delegate). A native decline SHALL be classified HONESTLY-DECLINED, never counted as an
AGREED native result. The comparison tolerances SHALL be FIXED — native-vs-closed-form
volume ≤ 2e-3 (polygon families, exact planar sweep) / ≤ 1.2e-2 (circle families, 64-gon
inscription of the smooth cone), native-vs-OCCT volume ≤ 5e-2, area ≤ 8e-2 — and SHALL NOT
be widened to force a pass.

#### Scenario: Native and OCCT sweep the identical inputs through the facade

- GIVEN one generated trial
- WHEN the native candidate is produced under `cc_set_engine(1)` and the OCCT oracle under `cc_set_engine(0)`, both sweeping the SAME profiles, spine, and guide
- THEN both SHALL have operated on the SAME variable sweep, and the closed-form volume (straight families) SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An in-envelope sweep agrees with OCCT and the closed form

- GIVEN a straight-spine circle / polygon / section-A→B morph inside the native envelope
- WHEN native produces a valid body and OCCT produces a valid measurement
- THEN the native body SHALL be watertight with Euler χ=2 and positive volume, its volume SHALL match the closed form within the fixed band, OCCT SHALL concur, and the trial SHALL be classified AGREED

#### Scenario: An out-of-envelope pose declines under native and is not a wrong sweep

- GIVEN a pose outside the native envelope (mismatched section vertex counts, a coincident guide start, or a non-planar guided spine)
- WHEN the native `build_variable_sweep` probe returns NULL and the facade forwards to a valid OCCT result
- THEN the trial SHALL be classified HONESTLY-DECLINED (native defers to OCCT), and this SHALL be a first-class outcome, never a bar failure

### Requirement: Guided family certified only within the native exact envelope

The harness SHALL certify the guided family ONLY within the native builder's two EXACT
sub-regimes and SHALL NOT widen a tolerance to admit the coupled morph×scale regime. The
native straight-spine guided sweep uses a two-station linear ruling that reproduces the
continuous guide-scale law exactly when EITHER the section is constant OR the guide scale is
constant, but drops a morph×scale cross-term when BOTH the section morphs AND the guide
scale varies (a native limitation confirmed by first-principles analysis and by two
independent oracles agreeing where native diverges). The two certified sub-regimes are
{constant section + splaying guide → a similar-polygon frustum} and {morphing section +
guide parallel to the spine so the scale is ≡1}. The coupled morph×scale regime SHALL be
REPORTED as a product limitation, with `src/native/**`, `src/engine/**`, `include/**`, and
the `cc_*` ABI byte-unchanged.

#### Scenario: A guided sweep in the exact envelope agrees with both oracles

- GIVEN a guided straight-spine trial drawn in an exact sub-regime (constant section + splaying guide, or morphing section + a spine-parallel guide)
- WHEN native produces a valid body and OCCT produces a valid measurement
- THEN the native volume SHALL match both the closed form and OCCT within the fixed bands and the trial SHALL be classified AGREED

#### Scenario: The coupled morph×scale regime is a reported limitation, not a widened tolerance

- GIVEN the observation that a coupled morph×scale guided sweep makes native diverge from two agreeing oracles
- WHEN the harness is designed
- THEN it SHALL exclude the coupled regime from the certified guided family and document it as a REPORTED native limitation, and SHALL NOT widen any tolerance to make such a case pass

### Requirement: Six-way classifier arbitrated by the ground truth

The harness SHALL classify each sweep trial into EXACTLY ONE bucket at the fixed
tolerances, with the closed-form volume (straight families) or OCCT (curved family) as the
correctness oracle:

- **AGREED** — native returned a watertight, χ=2, positive-volume body whose volume matches
  the ground truth within the fixed band, AND the oracle concurs.
- **HONESTLY-DECLINED** — the native builder returned NULL (an out-of-envelope pose) and the
  facade forwarded to a valid OCCT result. First-class, logged, NOT a bar failure.
- **DISAGREED** — native returned a valid body whose volume does NOT match the ground truth
  beyond the fixed band while the oracle matches it. A genuine SILENT WRONG sweep — the
  failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form ground truth while OCCT does NOT.
  The native sweep is CORRECT and vindicated by exact math; OCCT is the outlier. Logged, NOT
  a native fault, NOT a bar failure.
- **BOTH-DECLINED** — an out-of-envelope pose both engines refuse.
- **ORACLE_UNRELIABLE** — a trial whose native result misses the ground truth AND the oracle
  also does not match it (or is absent). FAILS the bar (investigate; never laundered).

#### Scenario: A native sweep matching OCCT and the closed form is AGREED

- GIVEN an in-envelope straight-spine trial
- WHEN the native body matches both OCCT and the closed-form ground truth within the fixed band with a watertight χ=2 positive-volume mesh
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A watertight native sweep that fails the closed form is a silent-wrong-result

- GIVEN a straight-spine trial whose native `cc_variable_sweep` returned a valid, watertight body
- WHEN that body's volume disagrees with the closed-form ground truth beyond the fixed band while OCCT matches it
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/parameter tuple so the native fault is reproducible

#### Scenario: A native sweep vindicated by exact math is not a false native fault

- GIVEN a trial where the native body differs from OCCT but matches the closed-form ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-sweep bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED trials
for each of the five families (circle-morph / polygon-morph / section-A→B straight; guided
straight; circle-morph curved) — and SHALL exit 0 IF AND ONLY IF the bar holds:
**DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real coverage (each of
the five families with at least one AGREED trial) proven across **at least two distinct
seeds**, N ≥ 60 per seed. Any DISAGREED (and any ORACLE-INACCURATE or ORACLE_UNRELIABLE)
SHALL be reported with its seed so it is reproducible. The harness SHALL NOT weaken a
tolerance, silently cap the batch, or drop trials to make the bar pass.

#### Scenario: Zero silent-wrong sweeps across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every one of the five families
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each of the five families has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary and exit 0

#### Scenario: Any silent-wrong sweep fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and family/parameter tuple, so the silent-wrong-sweep is reproducible and not laundered into a pass

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-envelope declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-family decline counts SHALL appear in the summary, so no honest exclusion is hidden

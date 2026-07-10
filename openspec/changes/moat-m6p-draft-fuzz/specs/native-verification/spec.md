# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random draft-angle generator

The draft-angle differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of families, base-solid
parameters, drafted-face subsets, and draft angles on any machine.

Each generated trial SHALL parameterise (a) a VALID prismatic base solid — a box or a
regular n-gon prism (n∈[3,8]) — at random valid dimensions, extruded +Z from the z=0
base plane, so its footprint and enclosed volume are exactly known, (b) one of the four
families {BOX, NGON} × {SINGLE-face, MULTI-face}, (c) a random subset of the solid's
planar side faces to draft (SINGLE = exactly one; MULTI = at least two), and (d) a random
valid draft angle about the base plane (pull +Z) bounded so the tapered top cross-section
stays non-degenerate. The base solid SHALL be built through the SAME public `cc_*` facade
the application calls, under the ACTIVE engine, and the draft SHALL be applied through the
public `cc_draft_faces` facade — not by calling a native builder directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the draft-angle fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` family + base-solid + drafted-subset + angle trials
- THEN the two sequences SHALL be byte-identical (same families, same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each drafted solid has a closed-form volume

- GIVEN a generated trial
- WHEN its base solid, drafted-face subset, and draft angle are inspected
- THEN the drafted solid's volume SHALL be given by an exact closed form — the base footprint polygon clipped by each drafted edge's inward-shifted (`z·tanθ`) supporting half-plane, integrated over the extrude height — so that closed form is an engine-independent ground-truth arbiter, exact for the ideal prismatic draft including adjacent-face corner interactions

### Requirement: Both-engine facade drive with closed-form and OCCT oracles

For each trial the harness SHALL apply the SAME draft two ways through the public
`cc_draft_faces` facade and arbitrate against a closed-form ground truth. The **native**
candidate SHALL be produced under `cc_set_engine(1)` (the NativeEngine prismatic-draft
path with its honest OCCT fallback), measured by `cc_mass_properties` and `cc_tessellate`.
The **OCCT oracle** SHALL be produced under `cc_set_engine(0)` through the SAME facade
(OCCT `BRepOffsetAPI_DraftAngle` with the SAME face ids, neutral origin, pull direction,
and angle), measured by `cc_mass_properties`. The **ground-truth arbiter** SHALL be the
closed-form drafted volume, compared to the native volume, NOT merely to OCCT.

Because sub-shape ids are engine-local, the drafted-face subset SHALL be resolved
SEPARATELY under each engine's body from geometry (a side face's mid-edge probe point
projected onto candidate faces via `cc_project_point_on_face`, selecting the face the
point lies on with a matching outward normal), never a stored id shared across engines.
If either engine cannot resolve the requested subset the trial SHALL be counted as an
unposable BOTH-DECLINED, never laundered into a pass.

The drafted AREA has no simple closed form, so AREA SHALL be cross-checked against OCCT
only, never against a fabricated analytic area. The comparison tolerances SHALL be FIXED
— native-vs-closed-form volume ≤ 1e-3 (planar draft volume is exact), native-vs-OCCT
volume ≤ 2e-2, area ≤ 3e-2 — and SHALL NOT be widened to force a pass.

#### Scenario: Native and OCCT apply the identical draft through the facade

- GIVEN one generated trial
- WHEN the native candidate is produced under `cc_set_engine(1)` and the OCCT oracle under `cc_set_engine(0)`, both applying the SAME draft (same face subset, neutral plane, pull, angle) to the SAME base solid
- THEN both SHALL have operated on the SAME draft of the SAME known base solid, and the closed-form volume SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An in-envelope draft agrees with OCCT and the closed form

- GIVEN a draft of one or more planar side faces of a box or n-gon prism inside the native envelope
- WHEN native produces a valid body and OCCT produces a valid measurement
- THEN the native body SHALL be watertight with Euler χ=2, its volume SHALL match the closed form within the fixed band, its volume SHALL be STRICTLY smaller than the base solid (a draft only removes stock), OCCT SHALL concur, and the trial SHALL be classified AGREED

#### Scenario: An out-of-envelope pose declines under native and is not a wrong draft

- GIVEN a draft pose outside the native envelope (e.g. adjacent box side faces whose tilted half-space cuts fail the composite self-verify, or a near-collapsing top)
- WHEN native `cc_draft_faces` returns 0 / an invalid body and OCCT ships a valid result
- THEN the trial SHALL be classified HONESTLY-DECLINED (native defers to OCCT), and this SHALL be a first-class outcome, never a bar failure

### Requirement: Six-way classifier arbitrated by the closed-form ground truth

The harness SHALL classify each draft trial into EXACTLY ONE bucket at the fixed
tolerances, with the ideal drafted solid's closed-form volume as the PRIMARY correctness
oracle:

- **AGREED** — native returned a watertight, χ=2 body whose volume matches the closed
  form within the fixed band and is strictly smaller than the base, AND OCCT also matches
  the closed form (agreement).
- **HONESTLY-DECLINED** — native `cc_draft_faces` returned 0 / an invalid body (an
  out-of-envelope pose) while OCCT formed a valid result. First-class, logged, NOT a bar
  failure.
- **DISAGREED** — native returned a valid body whose volume does NOT match the closed
  form beyond the fixed band while OCCT matches it. A genuine SILENT WRONG draft — the
  failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form ground truth while OCCT does
  NOT. The native draft is CORRECT and vindicated by exact math; OCCT is the outlier.
  Logged in full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — an out-of-envelope or unresolvable pose both engines refuse.
- **ORACLE_UNRELIABLE** — a trial whose native result misses the closed form AND OCCT
  also does not match it. FAILS the bar (investigate; never laundered).

#### Scenario: A native draft matching OCCT and the closed form is AGREED

- GIVEN an in-envelope draft trial
- WHEN the native body matches both OCCT and the closed-form ground truth within the fixed band with a strict-shrink, watertight χ=2 mesh
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A watertight native draft that fails the closed form is a silent-wrong-result

- GIVEN a draft trial whose native `cc_draft_faces` returned a valid, watertight body
- WHEN that body's volume disagrees with the closed-form ground truth beyond the fixed band while OCCT matches it
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/parameter tuple so the native fault is reproducible

#### Scenario: A native draft vindicated by exact math is not a false native fault

- GIVEN a trial where the native body differs from OCCT but matches the closed-form ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-draft bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED
trials for each of the four {BOX, NGON} × {SINGLE-face, MULTI-face} families — and SHALL
exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across
the batch, with real coverage (each of the four families with at least one AGREED trial)
proven across **at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED (and any
ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL be reported with its seed so it is
reproducible. The harness SHALL NOT weaken a tolerance, silently cap the batch, or drop
trials to make the bar pass.

#### Scenario: Zero silent-wrong drafts across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every one of the four families
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each of the four families has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary and exit 0

#### Scenario: Any silent-wrong draft fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and family/parameter tuple, so the silent-wrong-draft is reproducible and not laundered into a pass

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-envelope declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-family decline counts SHALL appear in the summary, so no honest exclusion is hidden

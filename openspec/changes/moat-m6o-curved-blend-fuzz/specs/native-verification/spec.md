# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random curved-blend generator

The curved-blend differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of families,
base-solid parameters, and blend magnitudes on any machine.

Each generated trial SHALL parameterise (a) a VALID analytic-revolve base solid — a
capped cylinder, a cone frustum, or a sphere-cap dome — at random valid dimensions, so
its enclosed volume after the blend has an exact closed form, and (b) exactly one of the
nine curved-blend families {FILLET, SHELL, OFFSET} × {cylinder-wall, cone-wall,
sphere-wall} with a random valid blend magnitude (fillet radius, shell wall thickness,
or signed offset distance) drawn inside the family's valid range. The base solid SHALL
be built through the SAME public `cc_*` facade the application calls, under the ACTIVE
engine, and the blend SHALL be applied through the public `cc_fillet_edges` /
`cc_shell` / `cc_offset_face` facade — not by calling a native builder directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the curved-blend fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` family + base-solid + blend-magnitude trials
- THEN the two sequences SHALL be byte-identical (same families, same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each base solid has a closed-form post-blend volume

- GIVEN a generated trial
- WHEN its base solid and blend magnitude are inspected
- THEN the base solid SHALL be an analytic revolve (capped cylinder / cone frustum / sphere-cap dome) at valid dimensions whose filleted, shelled, or offset volume is given by an exact closed form, so that closed form is an engine-independent ground-truth arbiter

### Requirement: Both-engine facade drive with OCCT and closed-form oracles

For each trial the harness SHALL apply the SAME curved blend two ways through the public
`cc_*` facade and arbitrate against a closed-form ground truth. The **native** candidate
SHALL be produced under `cc_set_engine(1)` (the NativeEngine analytic curved-blend
path), measured by `cc_mass_properties` and `cc_tessellate`. The **OCCT oracle** SHALL
be produced under `cc_set_engine(0)` through the facade for the FILLET and SHELL families
(`BRepFilletAPI` and `BRepOffsetAPI_MakeThickSolid`), and SHALL be built DIRECTLY with
OCCT primitives (`BRepPrimAPI_MakeCylinder` / `MakeCone` / `MakeSphere` at the offset
radius, measured by `BRepGProp`) for the OFFSET families — because the shipped OCCT
`cc_offset_face` handles PLANAR faces only and honestly declines a curved wall, so
OCCT-through-facade cannot be the offset oracle. The **ground-truth arbiter** SHALL be
the closed-form volume of the ideal blended solid, compared to the native volume, NOT
merely to OCCT.

The offset of a cone wall SHALL be treated as a PERPENDICULAR offset: both cap radii
shift by `d/cosσ` (σ the cone semi-angle), NOT by `d`, in BOTH the closed form and the
direct-OCCT oracle — matching the native `curved_offset.h` `Rref → Rref + d/cosσ`.

The blend magnitude bands and comparison tolerances SHALL be FIXED — the
deflection-convergence bands the per-op curved parity harnesses validated (native-vs-OCCT
volume ≤ 2e-2, native-vs-closed-form volume ≤ 2e-2, area ≤ 4e-2) — and SHALL NOT be
widened to force a pass.

#### Scenario: Native and OCCT apply the identical blend through the facade

- GIVEN one generated trial
- WHEN the native candidate is produced under `cc_set_engine(1)` and the OCCT oracle under `cc_set_engine(0)` (or built directly for the planar-only offset facade), both applying the SAME blend to the SAME base solid
- THEN both SHALL have operated on the SAME curved blend of the SAME known base solid, and the closed-form volume SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An in-envelope curved blend agrees with OCCT and the closed form

- GIVEN a fillet / shell / offset of a cylinder, cone, or sphere wall inside the native envelope
- WHEN native produces a valid body and OCCT (facade or direct) produces a valid measurement
- THEN the native body SHALL be watertight with Euler χ=2, its volume SHALL match the closed form within the fixed deflection band, its volume-change direction SHALL be correct (fillet/inward-shell REMOVE material; a grow/shrink offset moves volume the expected way), OCCT SHALL concur, and the trial SHALL be classified AGREED

#### Scenario: An out-of-envelope pose declines under native and is not a wrong blend

- GIVEN a curved-blend pose outside the native envelope (e.g. a shallow spherical-cap rim fillet the native arm does not seat)
- WHEN native `cc_*` returns 0 / an invalid body and OCCT ships a valid result
- THEN the trial SHALL be classified HONESTLY-DECLINED (native defers to OCCT), and this SHALL be a first-class outcome, never a bar failure

### Requirement: Six-way classifier arbitrated by the closed-form ground truth

The harness SHALL classify each curved-blend trial into EXACTLY ONE bucket at the fixed
tolerances, with the ideal blended solid's closed-form volume as the PRIMARY correctness
oracle:

- **AGREED** — native returned a watertight, χ=2 body whose volume matches the closed
  form within the fixed band with the correct grow/shrink direction, AND OCCT (facade or
  direct) also matches the closed form (agreement).
- **HONESTLY-DECLINED** — native `cc_*` returned 0 / an invalid body (an out-of-envelope
  pose) while OCCT formed a valid result. First-class, logged, NOT a bar failure.
- **DISAGREED** — native returned a valid body whose volume does NOT match the closed
  form beyond the fixed band while OCCT matches it. A genuine SILENT WRONG curved blend —
  the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form ground truth while OCCT does
  NOT. The native blend is CORRECT and vindicated by exact math; OCCT is the outlier.
  Logged in full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — an out-of-envelope pose both engines refuse.
- **ORACLE_UNRELIABLE** — a core family whose OCCT oracle could not be obtained or does
  not match the closed form AND native also missed. FAILS the bar (investigate; never
  laundered).

#### Scenario: A native blend matching OCCT and the closed form is AGREED

- GIVEN an in-envelope curved-blend trial
- WHEN the native body matches both OCCT and the closed-form ground truth within the fixed band with the correct direction and a watertight χ=2 mesh
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A watertight native blend that fails the closed form is a silent-wrong-result

- GIVEN a curved-blend trial whose native `cc_*` returned a valid, watertight body
- WHEN that body's volume disagrees with the closed-form ground truth beyond the fixed band while OCCT matches it
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/parameter tuple so the native fault is reproducible

#### Scenario: A native blend vindicated by exact math is not a false native fault

- GIVEN a trial where the native body differs from OCCT but matches the closed-form ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-blend bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED
trials for each of the nine {FILLET, SHELL, OFFSET} × {cyl, cone, sphere} families — and
SHALL exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0**
across the batch, with real coverage (each of the nine families with at least one AGREED
trial) proven across **at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED (and
any ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL be reported with its seed so it is
reproducible. The harness SHALL NOT weaken a tolerance, silently cap the batch, or drop
trials to make the bar pass.

#### Scenario: Zero silent-wrong blends across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every one of the nine families
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each of the nine families has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary and exit 0

#### Scenario: Any silent-wrong blend fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and family/parameter tuple, so the silent-wrong-blend is reproducible and not laundered into a pass

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-envelope declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-family decline counts SHALL appear in the summary, so no honest exclusion is hidden

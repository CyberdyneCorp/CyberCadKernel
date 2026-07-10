# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random wrap-emboss freeform-base generator

The wrap-emboss freeform-base differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL NOT
read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64 seeding a
xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED` (argv/env-overridable,
with a fixed default). Re-running the harness with the same seed and batch size SHALL
produce a **byte-identical** sequence of bases, modes, footprints, and depths on any
machine.

Each generated trial SHALL parameterise (a) a base ∈ {cylinder lateral face, sphere-cap
dome wall}, (b) a mode ∈ {raised (boss=1), recessed (boss=0)}, and (c) a random VALID pose
— for a cylinder, a footprint (rectangle or polygon) that fits on the wall (arc span < 2π,
axial span strictly inside the ends) at a random emboss height / deboss depth < R; for a
sphere-cap dome, a random radius, cap offset, boss half-angle strictly inside the dome's
polar opening, and height. The pose SHALL be embossed through the SAME public
`cc_wrap_emboss` facade the application calls, under the ACTIVE engine — not by calling a
native builder directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the wrap-emboss freeform fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` base + mode + footprint + depth trials
- THEN the two sequences SHALL be byte-identical (same bases, same modes, same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each in-scope emboss has a closed-form volume delta

- GIVEN a generated cylinder or sphere-cap raised/recessed trial in scope
- WHEN its pose is inspected
- THEN the embossed/debossed volume delta SHALL be given by an exact closed form — `A·|Rout²−R²|/(2R)` for a wrapped footprint of flat area `A` on a cylinder, and the spherical-shell sector `2π(1−cosφ0)·((R+h)³−R³)/3` for a sphere-cap pole boss — so that closed form is an engine-independent ground-truth arbiter, exact for the ideal feature

### Requirement: Both-engine facade drive with a closed-form primary arbiter and OCCT only where developable

For each trial the harness SHALL emboss the SAME pose through the public `cc_wrap_emboss`
facade and arbitrate. The **native** candidate SHALL be produced under `cc_set_engine(1)`
(the NativeEngine facet-soup builder with its honest OCCT fallback), and — for a CYLINDER
base — the **OCCT oracle** under `cc_set_engine(0)` (the OCCT wrap-emboss on the SAME
cylinder wall). Volume and area SHALL be read through the public `cc_mass_properties`, and
watertightness / Euler characteristic / mesh volume through the public `cc_tessellate`.

Because a sphere-cap face is NON-developable, OCCT's own `cc_wrap_emboss` SHALL decline it
(return 0); the harness SHALL ASSERT that decline as a first-class negative reference and
SHALL use the CLOSED-FORM shell-sector delta as the SOLE boss arbiter, added to the base
dome volume that OCCT measures exactly (`cc_mass_properties` on the revolved dome). For a
cylinder base the PRIMARY arbiter SHALL be the closed-form changed volume, with OCCT
cross-checked on volume AND area. The comparison tolerances SHALL be FIXED — cylinder
native/OCCT/closed-form volume ≤ 2e-2 and area ≤ 3e-2; sphere native-vs-closed-form volume
≤ 1.5e-2 and mesh-vs-brep ≤ 2e-2 — and SHALL NOT be widened to force a pass.

#### Scenario: Native and OCCT emboss the identical cylinder pose through the facade

- GIVEN one generated cylinder trial
- WHEN the native candidate is produced under `cc_set_engine(1)` and the OCCT oracle under `cc_set_engine(0)`, both embossing the SAME footprint on the SAME cylinder wall
- THEN both SHALL have embossed the SAME known pose, and the closed-form changed volume SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: OCCT declines the non-developable sphere base and the closed form is the sole arbiter

- GIVEN one generated sphere-cap raised trial
- WHEN `cc_wrap_emboss` is invoked on the sphere wall under `cc_set_engine(0)`
- THEN OCCT SHALL return 0 (it cannot wrap a non-cylindrical face), the harness SHALL record that decline as expected, and the native embossed volume SHALL be arbitrated SOLELY by the closed-form shell-sector delta added to the OCCT-measured base-dome volume

#### Scenario: An in-scope cylinder emboss agrees with OCCT and the closed form

- GIVEN a cylinder raised or recessed pose in scope
- WHEN native produces a watertight solid and OCCT produces a valid solid
- THEN the native volume SHALL match the closed-form delta within the fixed band, OCCT SHALL concur on volume and area, the native solid SHALL be watertight with Euler χ = 2, and the trial SHALL be classified AGREED

#### Scenario: An in-scope sphere-cap emboss agrees with the closed form

- GIVEN a sphere-cap raised pole boss with a half-angle strictly inside the dome's polar opening
- WHEN native produces a watertight solid
- THEN the native volume SHALL equal the OCCT base-dome volume plus the closed-form shell-sector delta within the fixed band, the native solid SHALL be watertight with Euler χ = 2 and a strictly GROWN volume, and the trial SHALL be classified AGREED

#### Scenario: An out-of-envelope pose declines under native and is not a wrong solid

- GIVEN an out-of-envelope pose (a general non-cylindrical developable base, a self-intersecting footprint, a boss half-angle reaching the dome rim, a >2π footprint, a deboss depth ≥ R, or a sphere-cap RECESSED pose the native arm does not implement)
- WHEN native `cc_wrap_emboss` returns 0 / a non-watertight candidate
- THEN the trial SHALL be classified HONESTLY-DECLINED (OCCT shipped, cylinder) or BOTH-DECLINED (no engine produced a solid), a first-class outcome, never a bar failure

### Requirement: Five-way classifier arbitrated by the closed-form or OCCT oracle

The harness SHALL classify each wrap-emboss trial into EXACTLY ONE bucket at the fixed
tolerances:

- **AGREED** — native returned a watertight solid (Euler χ = 2, volume in the correct
  direction) whose volume matches the arbiter within the fixed band (the closed-form delta
  for both bases, cross-checked against OCCT volume + area for the cylinder).
- **HONESTLY-DECLINED** — native `cc_wrap_emboss` returned 0 / a non-watertight candidate on
  an in-scope pose while OCCT (cylinder) shipped a valid solid. First-class, logged, NOT a
  bar failure.
- **DISAGREED** — native returned a watertight solid whose volume does NOT match the
  closed-form delta beyond the fixed band (and, for the cylinder, the OCCT reconstruction),
  or whose area is wrong (cylinder), or whose Euler χ ≠ 2. A genuine SILENT WRONG solid —
  the failure this harness exists to catch. FAILS the bar.
- **ORACLE_UNRELIABLE** (a.k.a. ORACLE-INACCURATE) — native matches the closed-form delta
  while OCCT (cylinder) does NOT. The native solid is CORRECT and vindicated by exact math;
  OCCT is the outlier. Logged in full, NOT a native fault, NOT a bar failure — but a
  non-zero count is reported so it is never laundered.
- **BOTH-DECLINED** — an out-of-envelope pose both engines refuse (including the
  sphere-recessed domain decline and the decline exercisers).

#### Scenario: A native solid matching the arbiter is AGREED

- GIVEN an in-scope emboss trial
- WHEN the native solid is watertight with Euler χ = 2 and its volume matches the closed-form delta (cross-checked against OCCT for the cylinder) within the fixed band
- THEN the trial SHALL be classified AGREED and SHALL contribute to that base×mode cell's coverage count

#### Scenario: A watertight native solid with the wrong volume is a silent-wrong-result

- GIVEN a trial whose native `cc_wrap_emboss` returned a watertight solid
- WHEN that solid's volume disagrees with the closed-form delta beyond the fixed band
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and base/mode/parameter tuple so the native fault is reproducible

#### Scenario: A native solid vindicated by exact math is not a false native fault

- GIVEN a cylinder trial where the native volume matches the closed-form delta while the OCCT reconstruction does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE_UNRELIABLE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-emboss bar, and logged honest scope

The harness SHALL print a coverage summary — the per-cell counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE / BOTH-DECLINED trials for each of the
four {cylinder, sphere-cap} × {raised, recessed} cells — and SHALL exit 0 IF AND ONLY IF
the bar holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real
coverage (each IN-SCOPE cell with at least one AGREED trial; the sphere-recessed cell is an
honest DOMAIN-level decline and is exempt from the AGREED requirement but SHALL still appear
in the table) proven across **at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED
(and any ORACLE_UNRELIABLE) SHALL be reported with its seed so it is reproducible. The
harness SHALL NOT weaken a tolerance, silently cap the batch, or drop trials to make the bar
pass.

#### Scenario: Zero silent-wrong embosses across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering all four base×mode cells
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each in-scope cell has at least one AGREED trial
- THEN the harness SHALL print the per-cell coverage summary and exit 0

#### Scenario: Any silent-wrong emboss fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and base/mode/parameter tuple, so the silent-wrong-emboss is reproducible and not laundered into a pass

#### Scenario: The non-developable sphere base and its OCCT decline are documented in the summary

- GIVEN a run whose batch includes sphere-cap trials
- WHEN the harness summarises
- THEN the summary SHALL record that OCCT's `cc_wrap_emboss` declined every sphere wall (the closed-form shell-sector delta was the sole arbiter) and SHALL show the sphere-recessed domain-decline count, so no honest exclusion is hidden

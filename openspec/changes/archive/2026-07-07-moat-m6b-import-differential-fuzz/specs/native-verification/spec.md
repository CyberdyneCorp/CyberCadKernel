# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random-valid solid generator for writer-serialisable STEP families

The STEP round-trip differential-fuzzing harness SHALL generate its batch of source
solids from a **deterministic, explicitly-seeded** pseudo-random number generator. The
harness SHALL NOT read any wall clock, `rand()`, `Date`, process id, address, or any
other non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64-seeded xoshiro256**) keyed ONLY by an explicit integer seed (argv/env-
overridable, with a fixed default). Re-running the harness with the same seed and batch
size SHALL produce a **byte-identical** batch on any machine.

Each generated case SHALL build a native B-rep `Solid` drawn from the families the native
STEP **writer** can serialise AND whose native-write → OCCT-read round-trip is a clean
oracle: a **box** and a **regular n-gon prism** (planar faces), a **cylinder** (a
rectangle revolved 360°), a **frustum** (a trapezoid revolved 360°, both radii strictly
positive — no degenerate apex), and a **holed box** (a circular through-hole → true
CIRCLE edges + a CYLINDRICAL wall). The solids SHALL be built through the SAME native
construct entry points the `cc_solid_extrude` / `cc_solid_revolve` /
`cc_solid_extrude_holes` facade uses (`build_prism`, `build_revolution` on a raw polygon,
`build_prism_with_holes`). All sampled parameters SHALL be constrained to produce a
valid, non-degenerate solid, so the writer + reader are genuinely EXERCISED rather than
trivially declined. The generator SHALL be OCCT-free.

For every family the harness SHALL also compute the **closed-form** volume and area of
the generated solid (the exact analytic ground truth consumed by the classifier).

#### Scenario: Same seed reproduces the identical batch (determinism)
- GIVEN the fuzz harness run twice with the same explicit seed `S` and batch size `N`
- WHEN each run generates its sequence of `N` source solids and drives them through the round-trip
- THEN the two runs SHALL produce byte-identical trial output (same family, params, and per-trial classification) with NO dependence on wall-clock time, `rand()`, or any other non-deterministic source

#### Scenario: Generated solids are valid and within the writer's clean-oracle scope
- GIVEN a generated case from any of the five families
- WHEN the harness builds the native solid and computes its closed-form volume/area
- THEN the solid SHALL be a non-degenerate native B-rep the native writer can serialise, and a family whose native-write → OCCT-read round-trip yields a valid closed solid (bare-periodic sphere and ruled loft are deliberately EXCLUDED and logged, per the scope-and-bar requirement)

### Requirement: STEP round-trip dual import — native Part-21 reader vs OCCT STEPControl_Reader on identical bytes

For each generated solid the harness SHALL export it to **ONE on-disk STEP file** via the
native OCCT-free writer, then import THAT SAME FILE two ways: via the native OCCT-free
Part-21 reader (`step_import_native` / `readStepFile`) AND via the OCCT
`STEPControl_Reader` oracle. The native reconstruction SHALL be measured by the native
tessellator (mesh volume, mesh area, watertight flag, solid count); the OCCT re-import
SHALL be measured EXACTLY by `BRepGProp` (volume, area, solid count) plus a `BRepCheck`
validity + closed-shell check. `src/native/**` SHALL remain OCCT-free — only the harness
links OCCT, and only as the oracle.

Before any reader verdict the harness SHALL apply an **oracle validity gate**: the OCCT
re-import of the file MUST be a valid closed solid with positive volume/area and at least
one solid. A file whose OCCT re-import is NOT a valid closed solid SHALL be classified
ORACLE_UNRELIABLE — excluded from the reader verdict and counted against the bar
(investigate the writer / OCCT; never laundered into a pass). A source the native writer
cannot serialise SHALL be classified WRITER_DECLINE — logged as a coverage drop (so a
silently-uncovered family cannot hide), and SHALL NOT be faked as a reader trial.

#### Scenario: Both readers consume the identical written file
- GIVEN a generated solid exported once to a STEP file `F` by the native writer
- WHEN the native reader and the OCCT `STEPControl_Reader` each import `F`
- THEN both SHALL read the SAME bytes, and the harness SHALL measure the native result by the native tessellator and the OCCT result exactly by `BRepGProp`, with `src/native/**` untouched and OCCT-free

#### Scenario: An unreliable oracle is never laundered into a pass
- GIVEN a written file whose OCCT re-import is not a valid closed solid
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE_UNRELIABLE, exclude it from the reader verdict, and FAIL the zero-silent-wrong bar (never silently accept it)

### Requirement: Agree / honestly-declined / DISAGREE classifier arbitrated by closed-form ground truth

The harness SHALL classify each reader trial (past the oracle validity gate) into EXACTLY
one of four buckets at a **FIXED** relative tolerance that is NEVER widened per-trial:

- **AGREED** — the native reader returns a watertight solid whose volume, area, AND solid
  count match the OCCT re-import within the fixed tolerance.
- **HONESTLY-DECLINED** — the native reader returns NULL or a non-watertight result, so
  the engine falls back to OCCT (logged); the OCCT re-import of the same file is a valid
  closed solid (a real, ship-able oracle result). An honest decline is a first-class
  outcome.
- **ORACLE-INACCURATE** — the native reader returns a watertight solid that DIFFERS from
  the OCCT re-import but MATCHES the closed-form analytic ground truth while OCCT does
  NOT. The native import is CORRECT and is VINDICATED by exact math; OCCT's re-import is
  the inaccurate one. This SHALL be logged in full and SHALL NOT count as a native fault
  or fail the bar.
- **DISAGREED** — the native reader returns a watertight solid that does NOT match the
  analytic ground truth (a genuine SILENT WRONG native import). This is the failure the
  harness exists to catch.

The analytic arbiter SHALL exonerate a native result ONLY when it POSITIVELY matches the
independent closed-form truth while OCCT does not; a native result that fails the analytic
truth SHALL remain DISAGREED. The tolerance SHALL NOT be widened to reclassify a
disagreement. Any DISAGREE or ORACLE-INACCURATE SHALL print the seed, the case index, the
family/param tuple, and all three measurements (native, OCCT, analytic) as a reproducible
regression / limitation record.

#### Scenario: A native import vindicated by exact math is not a false native fault
- GIVEN a shallow-cone frustum whose native re-import matches the closed-form frustum volume/area within tolerance while the OCCT re-import is measurably outside tolerance in the SAME direction (e.g. seed 0x1234 index=10: native within ~9e-4/5e-4, OCCT ~2.7e-2/1.2e-2 high)
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE-INACCURATE (native vindicated by exact math), log it with the seed + repro tuple + all three measurements, and NOT fail the bar

#### Scenario: A watertight native import that fails the ground truth is a silent-wrong-result
- GIVEN a native reader that returns a watertight solid whose volume/area/solid-count does NOT match the closed-form ground truth
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial DISAGREED and print the seed + case index + full param tuple + native/OCCT/analytic measurements as a reproducible regression find

### Requirement: Coverage summary, zero-silent-wrong-import bar, and logged honest scope

The harness SHALL print a coverage summary: the seed, the batch size, and per-family
counts of agreed / honestly-declined / DISAGREED / oracle-inaccurate plus writer-declined.
The process SHALL exit 0 IF AND ONLY IF `DISAGREED == 0` AND `ORACLE_UNRELIABLE == 0`.
The harness SHALL carry its own `main()`, be on `scripts/run-sim-suite.sh`'s SKIP list,
compile the native writer + reader + tessellator TUs (OCCT-free, no numsci) plus the OCCT
oracle toolkits, and leave `src/native/**` untouched and OCCT-free.

The harness SHALL record, in its header and in this spec, the domain-level honest
exclusions so no coverage is silently dropped: the **bare-periodic sphere**
(`SPHERICAL_SURFACE` / `VERTEX_LOOP`) is excluded because OCCT's re-import of the native
sphere is inconsistent (not a clean oracle in the write→read direction); the **ruled
loft** (bilinear `B_SPLINE_SURFACE`) is excluded because the native writer honestly
declines to serialise it. The **frustum** family SHALL exercise the honest-decline branch
for real (the native reader reconstructs a revolved cone watertight only in a minority of
cases and otherwise falls back to OCCT).

#### Scenario: Zero DISAGREED across multiple seeds with real family coverage
- GIVEN the harness run across at least two explicit seeds with a batch that covers all five families
- WHEN every trial is classified
- THEN `DISAGREED` SHALL be 0 and `ORACLE_UNRELIABLE` SHALL be 0 (the process exits 0), with AGREED trials in the planar / cylinder / holed families and HONESTLY-DECLINED trials in the frustum family

#### Scenario: Honest domain-level exclusions are logged, not silently dropped
- GIVEN the bare-periodic sphere and ruled-loft families are out of the clean-oracle scope
- WHEN the harness documents its coverage
- THEN the exclusions SHALL be recorded (in the harness header and this spec) with their reason, and a writer decline encountered at run time SHALL be counted and printed rather than silently skipped

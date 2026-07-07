# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random-valid construction-input generator

The construction (loft/sweep) differential-fuzzing harness SHALL generate its batch of
source inputs from a **deterministic, explicitly-seeded** pseudo-random number generator.
The harness SHALL NOT read any wall clock, `rand()`, `Date`, process id, address, or any
other non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64-seeded xoshiro256**) keyed ONLY by an explicit integer seed (argv/env-
overridable, with a fixed default). Re-running the harness with the same seed and batch
size SHALL produce a **byte-identical** batch on any machine.

Each generated case SHALL be a construction input drawn from the families the native
swept-solid builders CLAIM (`src/native/construct` — `loft.h`, `sweep.h`): an equal-count
PLANAR **2-section loft** (a coaxial regular-n-gon frustum), a PLANAR **N-section loft**
(a prismatoid stack of coaxial regular n-gons), a **mismatched-count loft** (a T1
collinear-resampled `n → 2n` loft whose two loops describe the same regular-n-gon
outline), and a **straight-path constant-frame sweep** (a closed planar profile swept
along a straight 3D path). The generator SHALL additionally emit, SPARINGLY, two
out-of-scope inputs that the native builder honestly returns NULL for — a **non-planar
loft section** and a **non-planar sweep spine** — to exercise the native DECLINE branch.
All sampled parameters SHALL be constrained to produce a valid, non-degenerate input, so
the native builder and the OCCT oracle are genuinely EXERCISED. The generator SHALL be
OCCT-free.

For every arbitrated family the harness SHALL also compute the **closed-form** volume and
area of the input (the exact analytic ground truth consumed by the classifier): a
prismatoid loft as a stack of pyramidal frustums, and a straight prism sweep as
`profileArea·pathLength`.

#### Scenario: Same seed reproduces the identical batch (determinism)
- GIVEN the fuzz harness run twice with the same explicit seed `S` and batch size `N`
- WHEN each run generates its sequence of `N` construction inputs and builds them both ways
- THEN the two runs SHALL produce byte-identical trial output (same family, params, and per-trial classification) with NO dependence on wall-clock time, `rand()`, or any other non-deterministic source

#### Scenario: Generated inputs are valid and within the native builder's claimed scope
- GIVEN a generated case from any core family (2-section frustum, N-section prismatoid stack, mismatched-count loft, straight-prism sweep)
- WHEN the harness builds the native solid and computes its closed-form volume/area
- THEN the input SHALL be a non-degenerate construction the native builder claims, and the two out-of-scope DECLINE-exercisers (non-planar loft section, non-planar sweep spine) SHALL be generated only sparingly to hit the native NULL branch, not to manufacture a disagreement

### Requirement: Construction dual build — native builder vs OCCT ThruSections/MakePipe on identical inputs

For each generated input the harness SHALL build it two ways from the SAME parameters:
DIRECTLY via the native OCCT-free builders (`ncst::build_loft_sections` /
`ncst::build_sweep`) AND via the OCCT oracle (`BRepOffsetAPI_ThruSections` as a ruled solid
over the section polygons; `BRepOffsetAPI_MakePipe` of the centroid-centred profile face
along the spine polyline — the SAME construction the `cc_*` OCCT engine uses). The native
result SHALL be measured by the native tessellator (mesh volume, mesh area, watertight
flag, solid count); the OCCT build SHALL be measured EXACTLY by `BRepGProp` (volume, area,
solid count) plus a `BRepCheck` validity + closed-shell check. The harness SHALL call the
native builders DIRECTLY (not through the `cc_*` facade) so that a NULL or non-watertight
native result is an UNAMBIGUOUS native DECLINE rather than a silent OCCT fall-through.
`src/native/**` SHALL remain OCCT-free — only the harness links OCCT, and only as the
oracle.

Before any AGREE/DISAGREE verdict the harness SHALL apply an **oracle validity gate**: for
a CORE family the OCCT build MUST be a valid closed solid with positive volume/area and at
least one solid. A core-family input whose OCCT build is NOT a valid closed solid SHALL be
classified ORACLE_UNRELIABLE — excluded from the verdict and counted against the bar
(investigate; never laundered into a pass). A DECLINE-exerciser (out-of-scope input) that
the native builder declined AND OCCT also did not build into a valid solid SHALL be
classified BOTH-DECLINED — logged, and NOT a bar failure (neither engine produced a wrong
result).

#### Scenario: Both engines build the identical input, native called directly
- GIVEN a generated construction input
- WHEN the native builder and the OCCT `ThruSections`/`MakePipe` oracle each build it from the same parameters
- THEN the harness SHALL measure the native result by the native tessellator and the OCCT result exactly by `BRepGProp`, with the native builder invoked DIRECTLY (a NULL/non-watertight result read as a native DECLINE), and with `src/native/**` untouched and OCCT-free

#### Scenario: A core-family unreliable oracle is never laundered into a pass
- GIVEN a core-family input whose OCCT build is not a valid closed solid
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE_UNRELIABLE, exclude it from the verdict, and FAIL the zero-silent-wrong bar (never silently accept it)

### Requirement: Construction agree / honestly-declined / DISAGREE classifier arbitrated by prismatoid/prism closed-form ground truth

The harness SHALL classify each construction trial (past the oracle validity gate) into
EXACTLY one of five buckets at a **FIXED** relative tolerance that is NEVER widened
per-trial:

- **AGREED** — the native builder returns a watertight solid whose volume, area, AND solid
  count match the OCCT build within the fixed tolerance.
- **HONESTLY-DECLINED** — the native builder returns NULL or a non-watertight candidate (the
  engine's mandatory self-verify would discard it → OCCT, logged); the OCCT build of the
  same input is a valid closed solid (a real, ship-able oracle result). An honest decline is
  a first-class outcome.
- **ORACLE-INACCURATE** — the native builder returns a watertight solid that DIFFERS from the
  OCCT build but MATCHES the closed-form analytic ground truth while OCCT does NOT. The
  native build is CORRECT and is VINDICATED by exact math; OCCT is the outlier. This SHALL be
  logged in full and SHALL NOT count as a native fault or fail the bar.
- **DISAGREED** — the native builder returns a watertight solid that does NOT match the
  analytic ground truth (a genuine SILENT WRONG native build). This is the failure the
  harness exists to catch.
- **BOTH-DECLINED** — a DECLINE-exerciser both engines refuse (no wrong result); logged, not
  a bar failure.

The analytic arbiter SHALL exonerate a native result ONLY when it POSITIVELY matches the
independent closed-form truth while OCCT does not; a native result that fails the analytic
truth SHALL remain DISAGREED. The tolerance SHALL NOT be widened to reclassify a
disagreement. Any DISAGREE or ORACLE-INACCURATE SHALL print the seed, the case index, the
family/param tuple, and all measurements (native, OCCT, analytic) as a reproducible
regression / limitation record.

#### Scenario: A native build vindicated by exact math is not a false native fault
- GIVEN a native construction whose result matches the closed-form ground truth (prismatoid or prism volume/area) within tolerance while the OCCT build is measurably outside tolerance
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE-INACCURATE (native vindicated by exact math), log it with the seed + repro tuple + all measurements, and NOT fail the bar

#### Scenario: A watertight native build that fails the ground truth is a silent-wrong-result
- GIVEN a native builder that returns a watertight solid whose volume/area/solid-count does NOT match the closed-form ground truth
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial DISAGREED and print the seed + case index + full param tuple + native/OCCT/analytic measurements as a reproducible regression find

### Requirement: Coverage summary, zero-silent-wrong-build bar, and logged honest scope

The harness SHALL print a coverage summary: the seed, the batch size, and per-family counts
of agreed / honestly-declined / DISAGREED / oracle-inaccurate / both-declined. The process
SHALL exit 0 IF AND ONLY IF `DISAGREED == 0` AND core-family `ORACLE_UNRELIABLE == 0`. The
harness SHALL carry its own `main()`, be on `scripts/run-sim-suite.sh`'s SKIP list, compile
the native construct / tessellate / topology / math sources (OCCT-free, no numsci) plus the
OCCT oracle toolkits, and leave `src/native/**` untouched and OCCT-free.

The harness SHALL record, in its header and in this spec, the domain-level honest exclusions
so no coverage is silently dropped: the **twisted / rotated-section loft** (bilinear
hyperbolic-paraboloid side faces) and the **smooth-curved planar sweep** (constant-frame
ruled tube) are EXCLUDED from the seeded comparison because their native-mesh-vs-OCCT-exact
match is only DEFLECTION-BOUNDED, not exact, and are covered instead by the curated parity
harnesses `native_loft_parity` / `native_sweep_parity`; the **N-section stack** family
surfaces the native tessellator seam limit (a stack whose consecutive bands taper at
different ratios T-junctions the shared ring → the engine self-verify discards it → OCCT),
so prisms and symmetric spools AGREE while free random stacks honestly DECLINE.

#### Scenario: Zero DISAGREED across multiple seeds with real family coverage
- GIVEN the harness run across at least two explicit seeds with a batch that covers all families
- WHEN every trial is classified
- THEN `DISAGREED` SHALL be 0 and core-family `ORACLE_UNRELIABLE` SHALL be 0 (the process exits 0), with AGREED trials in the 2-section frustum, N-section prismatoid stack, mismatched-count loft, and straight-prism sweep families, and HONESTLY-DECLINED trials in the non-planar exercisers and the free N-section stacks

#### Scenario: Honest domain-level exclusions are logged, not silently dropped
- GIVEN the twisted-section loft and smooth-curved planar sweep are out of the exact-comparison scope, and mismatched-taper N-section stacks T-junction the native mesh
- WHEN the harness documents its coverage
- THEN the exclusions SHALL be recorded (in the harness header and this spec) with their reason and their curated-parity coverage, and an N-section self-verify decline encountered at run time SHALL be counted and printed rather than silently skipped

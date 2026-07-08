# native-verification Specification

## Purpose
TBD - created by archiving change moat-m6-differential-fuzzing. Update Purpose after archive.
## Requirements
### Requirement: Deterministic seeded random-valid input generator for recognised boolean families

The differential-fuzzing harness SHALL generate its batch of boolean operand pairs
from a **deterministic, explicitly-seeded** pseudo-random number generator. The
harness SHALL NOT read any wall clock, `rand()`, `Date`, process id, address, or any
other non-deterministic source; the RNG SHALL be a self-contained integer generator
(e.g. splitmix64 / xoshiro) seeded ONLY by an explicit integer seed (env-overridable,
with a fixed default). Re-running the harness with the same seed and batch size SHALL
produce a **byte-identical** sequence of operand parameter tuples on any machine.

Each generated tuple SHALL parameterise an operand drawn from the **recognised
curved-solid families** the native path actually supports — a finite `AxisCylinder`
(axis ∈ {X,Y,Z}, radius > 0, `lo < hi`), a `Sphere` (radius > 0), or a `Cone`
(non-degenerate: `r0 ≠ r1` or a valid apex, positive extent) — placed in one of the
**recognised placements** (coaxial, orthogonal-through, or offset-parallel). All
sampled parameters SHALL be constrained to produce a **valid, non-degenerate,
overlapping** operand pair (positive radii, finite extents, an actual intersection),
so the native boolean path is genuinely EXERCISED rather than trivially declined on
malformed input. The generator SHALL be OCCT-free (it produces plain parameter POD).

#### Scenario: Same seed reproduces the identical batch (determinism)
- GIVEN the fuzz harness run twice with the same explicit seed `S` and batch size `N`
- WHEN each run generates its sequence of `N` operand parameter tuples
- THEN the two sequences SHALL be identical tuple-for-tuple (same axis, centre, radius, extents, half-angle, placement, family, and op ordering) with NO dependence on wall-clock time, `rand()`, or any other non-deterministic source

#### Scenario: Generated operands are valid, non-degenerate, and from recognised families
- GIVEN a batch generated from any seed
- WHEN each operand tuple is inspected
- THEN every operand SHALL be a recognised family (cylinder / sphere / cone) with a strictly positive radius and finite `lo < hi` extent (and, for a cone, a non-degenerate profile), and each pair SHALL be placed to actually overlap — so the native path is exercised, not declined merely because an input was malformed

### Requirement: Dual native + OCCT-oracle builder from one parameter tuple

The harness SHALL turn each generated parameter tuple into BOTH (a) a native operand
built through the OCCT-free native constructors (`nb::curved` builders /
`makeCyl` / `makeSphere` / `makeCone`) AND (b) the geometrically IDENTICAL OCCT
primitive (`BRepPrimAPI_MakeCylinder` / `MakeSphere` / `MakeCone` with the same axis,
centre, radius, and extents). The native operand and the OCCT operand SHALL be the
same solid so that any downstream volume/area difference is attributable to the
boolean, not to a mismatched input. OCCT SHALL appear ONLY on the oracle side of the
harness; the native operand builders SHALL reference no OCCT type, and `src/native`
SHALL remain OCCT-free.

#### Scenario: Native and OCCT operands built from one tuple are the same solid
- GIVEN a single generated parameter tuple for a recognised family
- WHEN the harness builds the native operand and the OCCT operand from that one tuple
- THEN the two operands SHALL have matching volume and surface area within the harness tolerance (they are the same solid), and the native operand SHALL have been constructed with no OCCT type

### Requirement: Agree / honestly-declined / DISAGREE classifier against the OCCT oracle

For each generated operand pair and each op `{Fuse, Cut, Common}`, the harness SHALL
run the native boolean path (`nb::boolean_solid`, the `cc_set_engine(1)` native
engine at its C++ boundary) AND the OCCT oracle (`BRepAlgoAPI_{Fuse,Cut,Common}`),
then classify the result as EXACTLY ONE of three outcomes:

- **AGREED** — the native result is non-null AND watertight (closed 2-manifold,
  positive enclosed volume) AND its enclosed volume AND surface area are BOTH within
  tolerance of the OCCT oracle's volume and area.
- **HONESTLY-DECLINED** — the native path returned NULL (the honest decline → OCCT
  fallback), AND the OCCT oracle is itself a valid watertight solid (so the shipped
  fallback result is correct).
- **DISAGREED** — the native result is non-null AND watertight BUT its volume OR area
  is OUTSIDE tolerance of the OCCT oracle.

A native result that is non-null but NOT watertight SHALL be treated as an honest
decline (the engine self-verify would discard it → fallback), NOT as an agreement.
The classifier SHALL NEVER weaken or widen the volume/area tolerance to convert a
DISAGREE into an AGREE; the tolerance SHALL be fixed and sized only to the
curved-face tessellation deflection, never to the observed discrepancy.

#### Scenario: A watertight native result matching OCCT volume and area is AGREED
- GIVEN a generated pair whose native boolean returns a non-null watertight solid
- WHEN its enclosed volume and surface area are compared to the OCCT oracle within the fixed tolerance
- THEN the case SHALL be classified AGREED and counted as a native agreement

#### Scenario: A native NULL with a valid OCCT oracle is HONESTLY-DECLINED, not a failure
- GIVEN a generated pair whose native boolean returns NULL (or a candidate the watertight self-verify rejects)
- WHEN the OCCT oracle for that pair and op is a valid watertight solid
- THEN the case SHALL be classified HONESTLY-DECLINED and SHALL NOT count as a failure (the OCCT fallback is the correct shipped result for an unsupported input)

#### Scenario: A watertight native result whose volume disagrees with OCCT is a DISAGREE (the M6 failure)
- GIVEN a generated pair whose native boolean returns a non-null WATERTIGHT solid
- WHEN its enclosed volume or surface area falls OUTSIDE the fixed tolerance of the OCCT oracle
- THEN the case SHALL be classified DISAGREED (a silent wrong result), the harness SHALL fail, and it SHALL print the reproducing seed, case index, op, family, and full parameter tuple as a regression find — the tolerance SHALL NOT be relaxed to hide it

### Requirement: Coverage summary and zero-silent-wrong-result bar

At the end of a batch the harness SHALL print a **coverage summary**: the batch seed,
the total number `N` of classified `{pair, op}` inputs, and the counts of AGREED,
HONESTLY-DECLINED, and DISAGREED, with a per-family and per-op breakdown. The
**completeness bar** is `DISAGREED == 0`: the harness process SHALL exit zero if and
only if no input DISAGREED (and every honestly-declined pair had a valid OCCT
oracle). Any DISAGREE — a native watertight result that disagrees with OCCT — SHALL
make the process exit non-zero. The summary SHALL make the honest breakdown explicit
so that "N random valid inputs, zero silent wrong results" is a measured, reproducible
statement rather than a claim over hand-picked fixtures.

#### Scenario: Zero disagreements passes the bar with a reproducible summary
- GIVEN a full seeded batch of `N` valid operand pairs classified across all three ops
- WHEN every input is either AGREED or HONESTLY-DECLINED (none DISAGREED)
- THEN the harness SHALL print the coverage summary (seed, `N`, agreed / declined / DISAGREED counts, per-family and per-op breakdown) and exit zero — meeting the M6 bar of zero silent wrong results

#### Scenario: Any disagreement fails the bar and is reported with its seed
- GIVEN a seeded batch in which at least one input is classified DISAGREED
- WHEN the batch completes
- THEN the coverage summary SHALL report `DISAGREED > 0`, the harness SHALL exit non-zero, and each disagreement SHALL be printed with the seed, case index, op, family, and parameter tuple needed to reproduce it as a regression find

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

### Requirement: Deterministic seeded random-valid blend-input generator for native-claimed fillet/chamfer families

The blend differential-fuzzing harness SHALL generate its batch of blend inputs from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL NOT
read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64-seeded
xoshiro256**) keyed ONLY by an explicit integer seed (argv/env-overridable, with a fixed
default). Re-running the harness with the same seed and batch size SHALL produce a
**byte-identical** batch on any machine.

Each generated case SHALL build a native B-rep body and pick a blend edge/rim from the
families the native blend path CLAIMS: a **planar-dihedral chamfer** (symmetric distance)
and a **planar-dihedral fillet** (constant radius) of ONE convex box edge; and, on a convex
cylinder↔cap **circular rim**, a **constant-radius fillet**, a **variable-linear-radius
fillet** (r1→r2), a **symmetric cone-frustum chamfer**, and an **asymmetric cone-frustum
chamfer** (d1 axial / d2 radial). The bodies SHALL be built through the SAME native
construct entry points the `cc_solid_extrude_profile` / `cc_solid_extrude` facade uses
(`build_prism_profile` on a full-circle profile → a capped cylinder; `build_prism` → an
axis-aligned box). All sampled parameters SHALL be constrained to produce a valid,
non-degenerate blend within the native path's scope (a ring-torus fillet `Rc ≥ 2·max(r)`, a
frustum chamfer `Rc − d2 > 0`, a box setback well inside the adjacent faces), so the native
builder is genuinely EXERCISED rather than trivially declined. The generator SHALL also emit
a SPARSE out-of-scope DECLINE-exerciser (a fillet radius with `Rc/2 < r < Rc`, outside the
native ring-torus scope) to exercise the native NULL branch. The generator SHALL be
OCCT-free.

For every AGREE family the harness SHALL compute the **closed-form** removed volume of the
blend (the exact analytic ground truth consumed by the classifier): the box-edge prism/groove
(`L·d²/2`, `L·r²(1−π/4)`), the torus-canal Pappus fillet removed volume (constant and linear
law), and the cone-frustum chamfer removed volume `π·d1·d2·(Rc − d2/3)`.

#### Scenario: Same seed reproduces the identical batch (determinism)
- GIVEN the fuzz harness run twice with the same explicit seed `S` and batch size `N`
- WHEN each run generates its sequence of `N` blend inputs and drives them through both builders
- THEN the two runs SHALL produce byte-identical trial output (same family, params, and per-trial classification) with NO dependence on wall-clock time, `rand()`, or any other non-deterministic source

#### Scenario: Generated inputs are valid and within the native path's claimed scope
- GIVEN a generated case from any of the six core families
- WHEN the harness builds the native body, picks the blend edge/rim, and computes the closed-form removed volume
- THEN the body SHALL be a non-degenerate native B-rep with the picked convex edge/rim in the native builder's scope, and the concave stepped-shaft fillet and offset/shell families SHALL be deliberately EXCLUDED and logged (per the scope-and-bar requirement)

### Requirement: Dual blend build on the SAME geometric edge — native builder called directly vs OCCT BRepFilletAPI oracle

For each generated case the harness SHALL build the blend two ways on the SAME geometric
edge/rim. The native side SHALL call the OCCT-free native blend builder DIRECTLY
(`blend::chamfer_edges` / `fillet_edges` / `curved_fillet_edge` / `variable_fillet_edge` /
`curved_chamfer_edge` / `curved_chamfer_edge_asym`) — NOT through the `cc_*` facade — so a
NULL Shape or a non-watertight candidate (which the engine's mandatory self-verify would
DISCARD) is an UNAMBIGUOUS native DECLINE rather than a silent forward to OCCT. The native
result SHALL be measured by the native tessellator (mesh volume, mesh area, watertight flag,
solid count). The OCCT side SHALL build the SAME body via `BRepPrimAPI_MakeBox` /
`BRepPrimAPI_MakeCylinder` and blend the SAME geometric edge/rim via `BRepFilletAPI_MakeFillet`
/ `BRepFilletAPI_MakeChamfer` (including `Add(d1,d2,edge,face)` for the asymmetric chamfer),
measured EXACTLY by `BRepGProp` plus a `BRepCheck` validity check. The box edge SHALL be
matched between the two bodies by vertex coincidence; the cylinder rim SHALL be picked in
both by geometry (the Circle edge at the top cap). `src/native/**` SHALL remain OCCT-free —
only the harness links OCCT, and only as the oracle.

Before any AGREE/DISAGREE verdict on a CORE family the harness SHALL apply an **oracle
validity gate**: the OCCT build MUST be a valid solid with positive volume/area and at least
one solid. A core-family input whose OCCT build is NOT a valid solid SHALL be classified
ORACLE_UNRELIABLE — excluded from the verdict and counted against the bar (investigate;
never laundered into a pass).

#### Scenario: Both builders act on the identical geometric edge
- GIVEN a generated body and a picked convex edge/rim
- WHEN the native builder blends it directly and the OCCT oracle blends the matched edge/rim of the identically-dimensioned OCCT body
- THEN both SHALL blend the SAME geometry, and the harness SHALL measure the native result by the native tessellator and the OCCT result exactly by `BRepGProp`, with `src/native/**` untouched and OCCT-free

#### Scenario: A native NULL / non-watertight result is an unambiguous decline
- GIVEN a native blend builder that returns a NULL Shape or a non-watertight candidate (e.g. the out-of-scope big-radius fillet)
- WHEN the harness measures the native result
- THEN it SHALL treat it as a native DECLINE (never a silent facade forward to OCCT), and — if the OCCT build of the same input is a valid solid — classify the trial HONESTLY-DECLINED

#### Scenario: An unreliable core-family oracle is never laundered into a pass
- GIVEN a core-family input whose OCCT build is not a valid solid
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE_UNRELIABLE, exclude it from the verdict, and FAIL the zero-silent-wrong bar (never silently accept it)

### Requirement: Blend classifier with the closed-form analytic ground truth as the primary correctness oracle

The harness SHALL classify each core-family trial (past the oracle validity gate) into
EXACTLY one bucket at a **FIXED** relative tolerance that is NEVER widened per-trial. The
clean **AGREED** verdict is native-vs-OCCT volume, area, AND solid-count all within the fixed
tolerance. When native-vs-OCCT EXCEEDS the tolerance, the harness SHALL NOT reflexively blame
the native builder: it SHALL arbitrate with the closed-form analytic ground truth (exact
math), which is the strongest oracle here because OCCT's own variable-radius fillet is an
APPROXIMATE evolved surface (a native-vs-OCCT gap for that family is a comparison of two
approximations). The arbitration SHALL be:

- **AGREED (via exact math)** — native matches the analytic ground truth AND OCCT matches
  it: the native result is VINDICATED by exact math and the native-vs-OCCT gap is merely two
  deflection-bounded approximations of the same exact solid (counted separately for audit).
- **ORACLE-INACCURATE** — native matches the analytic ground truth and OCCT does NOT: the
  native blend is CORRECT and OCCT's evaluation is the outlier. Logged in full; NOT a native
  fault and NOT a bar failure.
- **DISAGREED** — native does NOT match the analytic ground truth (a genuine SILENT WRONG
  native blend). This is the failure the harness exists to catch.
- **HONESTLY-DECLINED** — native returned NULL / non-watertight while the OCCT build is a
  valid solid.
- **BOTH-DECLINED** — an out-of-scope (DECLINE-exerciser) input that both engines refuse
  (no wrong result to compare).

Correctness SHALL be gated on the exact-math volume, so an area-only excursion can NEVER
produce a false DISAGREED. A native result SHALL be exonerated ONLY when it POSITIVELY
matches the independent closed-form truth; a native result that fails the analytic truth
SHALL remain DISAGREED. The tolerance SHALL NOT be widened to reclassify a disagreement. Any
DISAGREE or ORACLE-INACCURATE SHALL print the seed, the case index, the family/param tuple,
and all measurements (native, OCCT, analytic) as a reproducible regression / limitation
record.

#### Scenario: A native blend vindicated by exact math is not a false native fault
- GIVEN a variable-linear-radius fillet whose native re-evaluation matches the closed-form removed volume within tolerance while OCCT's evolved-surface fillet is measurably outside tolerance vs the same exact math (e.g. seed 0xC0FFEE1234 index=37: native within ~2e-4, OCCT ~2.6e-2 off)
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE-INACCURATE (native vindicated by exact math), log it with the seed + repro tuple + all measurements, and NOT fail the bar

#### Scenario: A watertight native blend that fails the ground truth is a silent-wrong-result
- GIVEN a native builder that returns a watertight solid whose volume does NOT match the closed-form removed-volume ground truth
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial DISAGREED and print the seed + case index + full param tuple + native/OCCT/analytic measurements as a reproducible regression find

### Requirement: Coverage summary, zero-silent-wrong-blend bar, and logged honest scope

The harness SHALL print a coverage summary: the seed, the batch size, per-family counts of
agreed / honestly-declined / DISAGREED / oracle-inaccurate / both-declined, the count of
AGREE trials resolved via the exact-math arbiter, and the measured max native-vs-OCCT bias
against the fixed tolerance. The process SHALL exit 0 IF AND ONLY IF `DISAGREED == 0` AND
core-family `ORACLE_UNRELIABLE == 0`. The harness SHALL carry its own `main()`, be on
`scripts/run-sim-suite.sh`'s SKIP list, compile the native blend + construct + tessellator +
math TUs (OCCT-free, no numsci) plus the OCCT oracle toolkits, and leave `src/native/**`
untouched and OCCT-free.

The harness SHALL record, in its header and in this spec, the domain-level honest exclusions
so no coverage is silently dropped: the **concave stepped-shaft fillet**
(`concave_fillet_edge`) and **offset_face / shell** are part of the native blend path's
claimed scope but are left to the curated parity harnesses for this first blend-fuzz slice
(not yet cleanly generatable as a seeded random family with a matching OCCT oracle). The
big-radius fillet DECLINE-exerciser SHALL exercise the native NULL branch for real (native
returns NULL, OCCT still fillets → HONESTLY-DECLINED).

#### Scenario: Zero DISAGREED across multiple seeds with real family coverage
- GIVEN the harness run across at least two explicit seeds with a batch that covers all six core families
- WHEN every trial is classified
- THEN `DISAGREED` SHALL be 0 and core-family `ORACLE_UNRELIABLE` SHALL be 0 (the process exits 0), with AGREED trials in every core family and HONESTLY-DECLINED trials in the big-radius-fillet DECLINE-exerciser

#### Scenario: Honest domain-level exclusions are logged, not silently dropped
- GIVEN the concave stepped-shaft fillet and offset/shell families are out of this slice's scope
- WHEN the harness documents its coverage
- THEN the exclusions SHALL be recorded (in the harness header and this spec) with their reason, and a native decline encountered at run time SHALL be counted and printed rather than silently skipped

### Requirement: Deterministic seeded random-valid wrap-emboss-input generator for native-claimed pad/pocket families

The wrap-emboss differential-fuzzing harness SHALL generate its batch of wrap-emboss inputs
from a **deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other non-deterministic
source; the RNG SHALL be a self-contained integer generator (splitmix64-seeded xoshiro256**)
keyed ONLY by an explicit integer seed (argv/env-overridable, with a fixed default).
Re-running the harness with the same seed and batch size SHALL produce a **byte-identical**
batch on any machine.

Each generated core case SHALL build a native capped-cylinder body and a footprint from the
families the native wrap-emboss path CLAIMS: a **rectangular pad** (emboss, material added), a
**rectangular deboss** pocket (material removed), a **convex N-gon** (n = 3..7) **emboss**, and
a **convex N-gon deboss** — every footprint wrapped onto the cylinder's lateral face. The base
body SHALL be built through the SAME native construct entry point the `cc_solid_extrude_profile`
facade uses (`build_prism_profile` on a full-circle profile → a capped cylinder with one
Cylinder wall face). All sampled parameters SHALL be constrained to a valid, in-scope input
(arc span < 2π, axial span strictly inside the wall ends, deboss depth < R, positive height),
so the native builder is genuinely EXERCISED rather than trivially declined; the polygon
footprint circumradius and amount SHALL be BOUNDED so the polygon cap's deflection-independent
inscribed-facet floor stays under the fixed tolerance. The generator SHALL also emit SPARSE
out-of-scope DECLINE-exercisers — a **non-cylindrical base**, a **>2π footprint**, a **deboss
depth ≥ R**, and a **self-intersecting** (pentagram) loop — to exercise the native NULL branch.
The generator SHALL be OCCT-free.

For every core case the harness SHALL compute the **closed-form curvature-corrected** changed
volume `ΔV = A·|Rout² − R²|/(2R)` (the exact analytic ground truth consumed by the classifier),
where `A` is the flat shoelace footprint area and `Rout = R + height` (emboss) or `R − depth`
(deboss). This closed form is universal across the rectangle and polygon footprints (it depends
only on the wrapped `(u, v)` measure `A/R`).

#### Scenario: Same seed reproduces the identical batch (determinism)
- GIVEN the fuzz harness run twice with the same explicit seed `S` and batch size `N`
- WHEN each run generates its sequence of `N` wrap-emboss inputs and drives them through the native builder and the oracle
- THEN the two runs SHALL produce byte-identical trial output (same family, params, and per-trial classification) with NO dependence on wall-clock time, `rand()`, or any other non-deterministic source

#### Scenario: Generated inputs are valid and within the native path's claimed scope
- GIVEN a generated case from any of the four core families
- WHEN the harness builds the native cylinder, wraps the footprint, and computes the closed-form changed volume
- THEN the body SHALL be a non-degenerate native capped cylinder, the footprint SHALL be in the native builder's scope (arc span < 2π, axial span inside the wall, deboss depth < R), and the out-of-scope inputs SHALL be confined to the sparse DECLINE-exercisers

### Requirement: Wrap-emboss dual build — native builder called directly vs closed-form + OCCT-reconstruction oracle

For each generated case the harness SHALL build the wrap-emboss two ways. The native side
SHALL call the OCCT-free native builder DIRECTLY (`feature::wrap_emboss`) — NOT through the
`cc_*` facade — so a NULL Shape or a non-watertight candidate (which the engine's mandatory
self-verify would DISCARD) is an UNAMBIGUOUS native DECLINE rather than a silent forward to
OCCT. The Cylinder wall face SHALL be picked by inspecting each face's surface kind. The native
result SHALL be measured by the native tessellator (mesh volume, mesh area, watertight flag,
solid count).

Because OCCT exposes NO single wrap-emboss API, the **PRIMARY** correctness oracle SHALL be
the closed-form curvature-corrected changed volume (universal across rectangle and polygon
footprints). The **SECONDARY** oracle SHALL, for the RECTANGLE families ONLY (where clean),
reconstruct the SAME solid via an OCCT boolean of the base cylinder with a wrapped shell wedge
— `BRepAlgoAPI_Fuse` of the base with an outer pie wedge for a pad, `BRepAlgoAPI_Cut` of the
base with a shell wedge (inner core grown so its faces are non-coincident) for a pocket, built
from `BRepPrimAPI_MakeCylinder` sectors and measured EXACTLY by `BRepGProp` plus a `BRepCheck`
validity check. The OCCT reconstruction SHALL be best-effort (guarded by `IsDone()` / validity)
and, on any boolean failure, SHALL be marked unavailable and the trial SHALL fall back to the
authoritative closed form (never a bar failure — the closed form is primary). `src/native/**`
SHALL remain OCCT-free — only the harness links OCCT, and only as the secondary oracle.

The harness SHALL NOT attempt an OCCT reconstruction of the POLYGON families: a wrapped
polygon pad would re-implement the feature (its arcs need their own faceting), so it is NOT
clean. This ORACLE-level decline SHALL be recorded (harness header + this spec); the polygon
families are arbitrated by the exact closed form, which the rectangle OCCT reconstruction
transitively validates (it is the SAME `A·|Rout²−R²|/(2R)` formula).

#### Scenario: The native builder is called directly and the rectangle solid is reconstructed by OCCT
- GIVEN a generated rectangle pad / pocket case
- WHEN the native builder wraps the footprint directly and the OCCT oracle reconstructs the same solid by fusing / cutting the base cylinder with a wrapped shell wedge
- THEN the harness SHALL measure the native result by the native tessellator and the OCCT reconstruction exactly by `BRepGProp`, and the OCCT reconstruction's volume SHALL equal the closed-form ground truth (with `src/native/**` untouched and OCCT-free)

#### Scenario: A native NULL / non-watertight result is an unambiguous decline
- GIVEN a native wrap-emboss builder that returns a NULL Shape or a non-watertight candidate
- WHEN the harness measures the native result
- THEN it SHALL treat it as a native DECLINE (never a silent facade forward to OCCT) — an in-scope decline classified HONESTLY-DECLINED and an out-of-scope decline classified BOTH-DECLINED

#### Scenario: The polygon families have no clean OCCT reconstruction and rely on the closed form
- GIVEN a polygon emboss / deboss case
- WHEN the harness selects an oracle
- THEN it SHALL NOT attempt an OCCT reconstruction (which would re-implement the wrap), SHALL arbitrate the trial by the exact closed-form volume alone, and SHALL record this ORACLE-level exclusion rather than silently dropping it

### Requirement: Wrap-emboss five-way classifier with the closed-form ground truth as the primary correctness oracle

The harness SHALL classify each core-family trial into EXACTLY one bucket at a **FIXED**
relative tolerance that is NEVER widened per-trial (`kVolRelTol = 2e-2`, `kAreaRelTol =
3e-2`). With `nativeUsable` meaning the native result is present, watertight, and of positive
volume, and `analyticMatch` meaning the native volume is within tolerance of the closed-form
expected total volume `πR²H ± ΔV`, the classification SHALL be:

- **AGREED** — `nativeUsable` and `analyticMatch` (and, for a rectangle family with a valid
  OCCT reconstruction, the native volume AND area also within tolerance of the reconstruction).
- **HONESTLY-DECLINED** — the native result is NULL / non-watertight on an in-scope input (the
  engine's self-verify would discard it → OCCT ships): a first-class outcome.
- **DISAGREED** — `nativeUsable` but NOT `analyticMatch` (a watertight native result whose
  volume violates exact math), OR a rectangle family whose volume matches but whose AREA
  disagrees with the OCCT reconstruction. This is the SILENT WRONG result the harness exists
  to catch.
- **ORACLE-INACCURATE** — `nativeUsable` and `analyticMatch` but the valid OCCT reconstruction
  disagrees on VOLUME: the native result is VINDICATED by exact math and the OCCT
  reconstruction is the outlier. Logged in full; NOT a native fault and NOT a bar failure.
- **BOTH-DECLINED** — an out-of-scope DECLINE-exerciser that the native builder refuses (no
  oracle, no wrong result to compare).

Correctness SHALL be gated on the exact-math VOLUME, so an area-only excursion can never
produce a false DISAGREED on a polygon family (which has no area oracle). A native result SHALL
be exonerated ONLY when it POSITIVELY matches the independent closed-form truth; the tolerance
SHALL NOT be widened to reclassify a disagreement. If the native builder returns a watertight
solid for an OUT-OF-SCOPE input (a guard leak), the harness SHALL flag it as a SURPRISE and
FAIL the bar (never laundered). Any DISAGREE / ORACLE-INACCURATE / SURPRISE SHALL print the
seed, the case index, the family/param tuple, and all measurements (native, closed form, OCCT
reconstruction) as a reproducible regression / limitation record.

#### Scenario: A watertight native wrap-emboss that fails the ground truth is a silent-wrong-result
- GIVEN a native builder that returns a watertight solid whose volume does NOT match the closed-form changed-volume ground truth (or, for a rectangle, whose area disagrees with the OCCT reconstruction while the volume matches)
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial DISAGREED and print the seed + case index + full param tuple + native/closed-form/OCCT measurements as a reproducible regression find

#### Scenario: A native result vindicated by exact math is not a false native fault
- GIVEN a rectangle case whose native volume matches the closed-form ground truth within tolerance while the OCCT boolean reconstruction is measurably outside tolerance vs the same exact math
- WHEN the harness classifies the trial
- THEN it SHALL bin the trial ORACLE-INACCURATE (native vindicated by exact math), log it with the seed + repro tuple + all measurements, and NOT fail the bar

#### Scenario: A guard leak on an out-of-scope input fails the bar
- GIVEN an out-of-scope DECLINE-exerciser (non-cylindrical base, >2π footprint, deboss depth ≥ R, or self-intersecting loop)
- WHEN the native builder returns a watertight solid instead of declining
- THEN the harness SHALL flag it as a SURPRISE (a native-guard leak), print its seed + repro tuple, and FAIL the zero-silent-wrong bar rather than silently accepting it

### Requirement: Wrap-emboss coverage summary, zero-silent-wrong bar, and logged honest scope

The harness SHALL print a coverage summary: the seed, the batch size, per-family counts of
agreed / honestly-declined / DISAGREED / oracle-inaccurate / both-declined, the count of OCCT
rectangle reconstructions checked, and the measured max native-vs-oracle bias against the fixed
tolerance. The process SHALL exit 0 IF AND ONLY IF `DISAGREED == 0` AND there is no out-of-scope
guard-leak SURPRISE. The harness SHALL carry its own `main()`, be on `scripts/run-sim-suite.sh`'s
SKIP list, compile the native feature + construct + tessellator + math TUs (OCCT-free, no
numsci) plus the OCCT oracle toolkits, and leave `src/native/**` untouched and OCCT-free.

The harness SHALL record, in its header and in this spec, the honest scope so no coverage is
silently dropped: the SECONDARY OCCT reconstruction is RECTANGLE-only (a wrapped polygon pad is
not cleanly reconstructable in OCCT), so the polygon families are arbitrated by the exact
closed form alone; and the differential's sensitivity to the pad is bounded by the native
builder's deflection-bounded faceting, which the FIXED tolerance and the bounded polygon
footprint hold auditably wide of a false positive. The out-of-scope inputs SHALL exercise the
native DECLINE branch for real (native returns NULL → BOTH-DECLINED), not manufacture DISAGREE.

#### Scenario: Zero DISAGREED across multiple seeds with real family coverage
- GIVEN the harness run across at least two explicit seeds with a batch that covers all four core families
- WHEN every trial is classified
- THEN `DISAGREED` SHALL be 0 and there SHALL be no guard-leak SURPRISE (the process exits 0), with AGREED trials in every core family and BOTH-DECLINED trials in the out-of-scope DECLINE-exercisers

#### Scenario: Honest oracle-level and sensitivity limitations are logged, not silently dropped
- GIVEN the polygon families have no clean OCCT reconstruction and the pad-volume sensitivity is bounded by deflection-bounded faceting
- WHEN the harness documents its coverage
- THEN the RECTANGLE-only OCCT reconstruction, the closed-form-only polygon arbitration, the bounded polygon footprint, and the measured max faceting bias vs the fixed tolerance SHALL all be recorded (harness header + this spec) rather than silently assumed
### Requirement: Deterministic seeded random-valid solid generator for the native mass-properties families

The mass-properties differential-fuzzing harness SHALL generate its batch of solids
from a **deterministic, explicitly-seeded** pseudo-random number generator. The
harness SHALL NOT read any wall clock, `rand()`, `Date`, process id, address, or any
other non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of solid
parameter tuples on any machine.

Each generated tuple SHALL parameterise a VALID, non-degenerate solid drawn from the
**native mass-properties families** the native path actually builds and meshes: a
`BOX` (rectangular prism), an `NGON_PRISM` (regular-n-gon extrude), a `CYLINDER`, a
`CONE`/`FRUSTUM`, a `SPHERE` (revolves of a rectangle / trapezoid / half-disk
profile), a `LOFT` (coaxial regular-n-gon prismatoid stack), and a `REVOLVE` (an
arbitrary axial polygon profile revolved through 2π). All sampled parameters SHALL be
constrained to produce a **valid, watertight, positive-volume** solid (positive
radii/extents, non-self-intersecting profile, finite dimensions) so the native mass
path is genuinely EXERCISED rather than trivially declined. The generator MAY also
emit **sparse, explicitly-labelled out-of-scope DECLINE-exercisers** (e.g. a
degenerate zero-height or self-touching profile) whose only purpose is to reach the
no-valid-mass path. The generator SHALL be OCCT-free (it produces plain parameter
POD consumed identically by the native builder and the OCCT oracle builder).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the mass-properties fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` solid parameter tuples
- THEN the two sequences SHALL be byte-identical (same family tags and same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Generated solids are valid and within the native mass path's meshable scope

- GIVEN a generated in-scope tuple (BOX / NGON_PRISM / CYLINDER / CONE / SPHERE / LOFT / REVOLVE)
- WHEN the native builder builds it
- THEN the solid SHALL be a single valid closed body with positive volume, watertight when meshed at the property deflection, so the native mass computation is genuinely exercised rather than declined on malformed input

### Requirement: Dual native mass measurement vs OCCT BRepGProp from one solid, with a closed-form analytic arbiter

For each generated tuple the harness SHALL measure the SAME solid two independent
ways and SHALL additionally compute a **closed-form analytic ground truth** for every
family that has one. The **native** measurement SHALL be the native mesh-based mass
path called DIRECTLY under the native engine — enclosed volume, surface area, and
centroid via the tessellator's signed-tetra decomposition at the fixed property
deflection, with `valid = watertight ∧ volume > 0`. The **oracle** measurement SHALL
be OCCT `BRepGProp::VolumeProperties` + `BRepGProp::SurfaceProperties` on the exact
B-rep (plus `GProp_PrincipalProps` for inertia). The **analytic arbiter** SHALL be
the exact closed form for the family: box/prism/cylinder/cone/sphere exact volume,
area, centroid (and principal moments); a coaxial-n-gon LOFT via the prismatoid-band
volume `Σ (Δz/3)(A_k + √(A_k A_{k+1}) + A_{k+1})` with planar-trapezoid lateral area;
a REVOLVE via Pappus (`V = 2π·r̄·A_profile`, lateral `A = 2π·r̄·P_profile`).

Because the native answer is a mesh discretisation of an exact solid, the native
tolerance SHALL be **matched to the tessellation deflection bound** at
`kPropertyDeflection` (tight for a planar exact-meshing family such as BOX / NGON_PRISM
/ straight LOFT; the deflection-derived convergence bound for a curved family such as
CYLINDER / SPHERE / CONE). This tolerance SHALL NOT be widened beyond that bound. The
harness SHALL NOT invent a native principal-moments value: because the native path
delegates inertia to OCCT (`CC_NATIVE_BODY_UNSUPPORTED`), the inertia dimension has
no native answer and is recorded as an honest native decline (the analytic-vs-OCCT
inertia comparison MAY still be logged as an oracle-trust check, never as a native
differential).

#### Scenario: Native and OCCT measure the identical solid, native called directly

- GIVEN one generated parameter tuple
- WHEN the native builder builds it and the native mass path measures it, and the OCCT builder builds the same tuple and `BRepGProp` measures it
- THEN both engines SHALL have operated on the SAME geometric solid (same family and parameters), the native mass path SHALL have been invoked directly (a native decline is observed as `valid == 0`, not silently forwarded to OCCT), and the closed-form analytic volume/area/centroid SHALL have been computed for every family that has one

#### Scenario: The native inertia dimension is an honest decline, never a fabricated number

- GIVEN a valid native body in any family
- WHEN the harness requests native principal moments
- THEN the native path SHALL delegate to OCCT (no independent native inertia), and the harness SHALL record the inertia dimension as an HONEST NATIVE DECLINE — it SHALL NOT synthesise a native inertia value nor count inertia as a native-vs-native agreement; the closed-form-vs-OCCT inertia check, if run, SHALL be logged only as oracle validation

#### Scenario: A curved family is held to the deflection-matched bound, not a widened tolerance

- GIVEN a CYLINDER / SPHERE / CONE trial whose native mesh under-approximates the curved surface at `kPropertyDeflection`
- WHEN the native mesh-based volume/area is compared to the exact closed form
- THEN the tolerance SHALL be the tessellation convergence bound derived from the deflection and feature size (never an arbitrary widened value), and a planar family (BOX / NGON_PRISM / straight LOFT) SHALL be held to the tight exact-meshing tolerance

### Requirement: Agree / honestly-declined / DISAGREE mass-properties classifier arbitrated by the closed-form ground truth

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed,
deflection-matched tolerance, with the closed-form analytic ground truth as the
PRIMARY correctness oracle (used to ATTRIBUTE a native-vs-OCCT gap rather than
reflexively blame the native path):

- **AGREED** — native `valid == 1` (watertight, positive volume) AND native
  volume/area/centroid match BOTH the OCCT `BRepGProp` measurement AND the
  closed-form analytic ground truth within tolerance.
- **HONESTLY-DECLINED** — native returns no valid mass (`valid == 0`: the native
  mesh is not watertight, so the mandatory self-verify yields no mass), while the
  OCCT build of the same tuple IS a valid measurable solid. First-class, logged, NOT
  a bar failure. (The inertia dimension is always recorded here for native bodies.)
- **DISAGREED** — native `valid == 1` but native volume/area/centroid does NOT match
  the CLOSED-FORM analytic ground truth beyond the deflection bound. A genuine SILENT
  WRONG MASS — the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native `valid == 1`, native DIFFERS from OCCT `BRepGProp`,
  but native MATCHES the closed-form analytic ground truth while OCCT does NOT. The
  native mass is CORRECT and vindicated by exact math; OCCT is the outlier. Logged in
  full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a DECLINE-exerciser where native produced no valid mass AND
  OCCT also could not measure a valid solid: neither engine produced a wrong number,
  so there is nothing to compare. Logged, NOT a bar failure.

An unreliable oracle SHALL NOT be laundered into a pass: when the closed-form arbiter
exists it is authoritative over OCCT; a native result is exonerated only when it
POSITIVELY matches exact math while OCCT does not.

#### Scenario: A native mass matching OCCT and the closed form is AGREED

- GIVEN an in-scope trial whose native solid is watertight with positive volume
- WHEN native volume/area/centroid match both the OCCT `BRepGProp` measurement and the closed-form analytic ground truth within the deflection-matched tolerance
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A non-watertight native mesh with a valid OCCT solid is HONESTLY-DECLINED, not a failure

- GIVEN a trial (in-scope or a decline-exerciser) whose native mesh is not watertight, so the native mass path returns `valid == 0`
- WHEN the OCCT build of the same tuple is a valid measurable solid
- THEN the trial SHALL be classified HONESTLY-DECLINED, logged with its seed and tuple, and SHALL NOT fail the bar — a missing native mass is never counted as a wrong mass

#### Scenario: A watertight native mass that fails the closed form is a silent-wrong-result

- GIVEN a trial whose native solid is watertight with positive volume (`valid == 1`)
- WHEN native volume, area, or centroid disagrees with the closed-form analytic ground truth beyond the deflection bound
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, family/parameter tuple, and all three measurements (native / OCCT / analytic) so the native fault is reproducible

#### Scenario: A native mass vindicated by exact math is not a false native fault

- GIVEN a trial where native differs from OCCT `BRepGProp` but native matches the closed-form analytic ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-mass bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / BOTH-DECLINED — and SHALL exit 0
IF AND ONLY IF the bar holds: **DISAGREED == 0** across the batch, with real
per-family coverage (at least the BOX, NGON_PRISM, CYLINDER, CONE, SPHERE, LOFT, and
REVOLVE families each exercised with at least one AGREED trial) proven across **at
least two distinct seeds**. Any DISAGREED (and any ORACLE-INACCURATE) SHALL be
reported with its seed so it is reproducible. The harness SHALL NOT weaken a
tolerance, silently cap the batch, or drop trials to make the bar pass: any capped or
skipped trial SHALL be logged. The honest scope SHALL be logged explicitly — the
native inertia decline (native principal moments delegate to OCCT) and the
mesh-vs-exact deflection boundary (why a curved family uses the deflection-matched
tolerance) SHALL appear in the summary, not be silently omitted.

#### Scenario: Zero disagreements across multiple seeds with real family coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every in-scope family
- WHEN no trial is classified DISAGREED and each family has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary, log the native inertia decline and the deflection boundary as honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, family/parameter tuple, and the native/OCCT/analytic measurements, so the silent-wrong-mass is reproducible and not laundered into a pass

#### Scenario: Honest scope and any dropped trials are logged, not silently omitted

- GIVEN a run in which some trials are decline-exercisers (BOTH-DECLINED) or the batch is capped for time
- WHEN the harness summarises
- THEN the native inertia decline, the mesh-vs-exact deflection boundary, and any capped or skipped trial SHALL be logged explicitly in the summary, so no honest exclusion is hidden


# native-verification

## ADDED Requirements

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

# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random-valid input generator for the geometry-services families, including oblique/tilted regimes

The geometry-services (GS) differential-fuzzing harness SHALL generate its batch of
per-service inputs from a **deterministic, explicitly-seeded** pseudo-random number
generator. The harness SHALL NOT read any wall clock, `rand()`, `Date`, process id,
address, or any other non-deterministic source; the RNG SHALL be a self-contained
integer generator (splitmix64 seeding a xoshiro256** stream) keyed ONLY by an
explicit integer `FUZZ_SEED` (argv/env-overridable, with a fixed default).
Re-running the harness with the same seed and batch size SHALL produce a
**byte-identical** sequence of per-service input tuples on any machine.

Each generated tuple SHALL parameterise a VALID, non-degenerate input drawn from the
families the covered GS services actually accept: for **GS3 distance** entity pairs
(vertex / edge / face) in general position INCLUDING skew (non-coplanar) segment
pairs and offset/tilted line-plane pairs; for **GS4 curvature** analytic faces
(plane / sphere / cylinder / cone / torus) and NURBS faces sampled at interior
`(u,v)`, and edges at parameter `t`; for **GS2 section** cut planes through solids
INCLUDING OBLIQUE planes; for **GS1 HLR** polyhedral/quadric solids with random view
directions INCLUDING oblique/isometric (not only axis-on); for **GS5 inertia** solids
under random rotation so principal axes are off the world axes; for **GS6 validity**
valid solids AND deterministically-broken variants with a known ground-truth verdict.
The generator SHALL deliberately reach the **oblique / tilted** regime for every
covered service — axis-aligned-only sampling is out of scope — and MAY additionally
emit sparse, explicitly-labelled out-of-scope DECLINE-exerciser tuples (a singular
curvature chart, the oblique cylinder cut, a curved silhouette, a non-watertight
solid, a coplanar-overlap pair) whose only purpose is to reach the documented
honest-decline path. The generator SHALL be OCCT-free (plain parameter POD consumed
identically by the native evaluator and the OCCT oracle builder).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the GS-services fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` per-service input tuples
- THEN the two sequences SHALL be byte-identical (same service tags, same family tags, and same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: The oblique/tilted regime is exercised for every covered service

- GIVEN a batch generated from any seed for the covered GS services
- WHEN the generated tuples are inspected
- THEN each covered service SHALL have at least one input in a genuinely OBLIQUE / TILTED configuration (a skew GS3 pair, a tilted GS4 face, an oblique GS2 cut plane, an oblique/isometric GS1 view, a rotated GS5 solid, a tilted broken GS6 solid) — the axis-aligned-only regime that hid the `ssi/plane_conics` oblique-cylinder bug SHALL NOT be the only regime sampled

#### Scenario: Generated inputs are valid and within the covered services' scope

- GIVEN a generated in-scope tuple for any covered service
- WHEN the native service evaluates it
- THEN the input SHALL be a valid, non-degenerate instance the service genuinely exercises (a real entity pair with a defined minimum distance, a face point off any singular chart, a plane that actually cuts the solid, a solid with a defined occluder / inertia / validity verdict) rather than one trivially declined on malformed input

### Requirement: Dual native GS-service evaluation vs its OCCT oracle from one input, with a closed-form analytic arbiter

For each generated tuple the harness SHALL evaluate the SAME input two independent
ways and SHALL additionally compute a **closed-form analytic ground truth** for every
family that has one. The **native** evaluation SHALL call the OCCT-free GS service
DIRECTLY (`analysis/distance.h`, `analysis/curvature.h`, `analysis/inertia.h`,
`analysis/validity.h`, `section/section.h`, `drafting/orthographic_hlr.h` via their
`NativeEngine` entry points) — a typed native decline SHALL be observed as a decline,
NOT silently forwarded to OCCT. The **oracle** evaluation SHALL build the
geometrically IDENTICAL shape and run the mapped OCCT oracle: `BRepExtrema_DistShapeShape`
for GS3, `GeomLProp_SLProps` / `BRepLProp_SLProps` for GS4, `BRepAlgoAPI_Section`
(edge length + loop count + closed-ness) plus `BRepGProp` (cap area) for GS2,
`HLRBRep_Algo` for GS1, `GProp_PrincipalProps` for GS5, and `BRepCheck_Analyzer::IsValid`
for GS6. The **analytic arbiter** SHALL be the exact closed form where one exists
(exact minimum distance for GS3 analytic cells; sphere/cylinder/cone/torus/circle
curvature for GS4; box/cylinder/sphere section area for GS2; the box-from-isometric
9-visible/3-hidden count for GS1; the exact inertia tensor and principal moments for
GS5; the construction-time valid/broken verdict for GS6).

Because several GS answers are mesh-derived (GS5 inertia and GS2 cap area from the M0
mesh, GS4 on a meshed NURBS, GS6 over the M0 mesh), the tolerance for those services
SHALL be **matched to the tessellation deflection-convergence bound** at the property
deflection — tight for a planar exact-meshing family, the deflection-derived bound for
a curved family — and SHALL NOT be widened beyond that bound. The exact-analytic
services (GS3 closed-form cells, GS4 analytic charts) SHALL be held to a tight
tolerance. OCCT SHALL appear ONLY on the oracle side; the native evaluation path SHALL
reference NO OCCT type, and `src/native/**` SHALL remain OCCT-FREE.

#### Scenario: Native and OCCT evaluate the identical input, native called directly

- GIVEN one generated parameter tuple for a covered service
- WHEN the native service evaluates it and the OCCT oracle evaluates the geometrically identical shape
- THEN both SHALL have operated on the SAME geometry (same family and parameters), the native service SHALL have been invoked directly (a native decline is observed as a decline, not forwarded to OCCT), and the closed-form analytic ground truth SHALL have been computed for every family that has one

#### Scenario: A mesh-derived service is held to the deflection-matched bound, not a widened tolerance

- GIVEN a GS5 inertia / GS2 cap-area / GS4-on-NURBS / GS6 trial whose native answer derives from the M0 mesh under-approximating a curved surface at the property deflection
- WHEN the native answer is compared to the exact closed form
- THEN the tolerance SHALL be the tessellation convergence bound derived from the deflection and feature size (never an arbitrary widened value), and a planar exact-meshing family SHALL be held to the tight tolerance

#### Scenario: The native evaluation path references no OCCT type

- GIVEN any trial for any covered service
- WHEN the native evaluation runs
- THEN it SHALL call only OCCT-free `src/native/**` code, OCCT SHALL appear only on the oracle side, and `src/native/**` SHALL remain OCCT-free

### Requirement: Agree / honest-native-decline / DISAGREE / oracle-inaccurate / both-decline classifier for the GS services

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed,
per-service tolerance, with the closed-form analytic ground truth as the PRIMARY
correctness oracle used to ATTRIBUTE a native-vs-OCCT gap rather than reflexively
blame the native path:

- **AGREE** — the native service returns a confident answer that matches BOTH the
  OCCT oracle AND the closed-form analytic ground truth (where one exists) within the
  fixed tolerance.
- **HONEST-NATIVE-DECLINE** — the native service returns its documented typed decline
  (a non-convergent GS3 freeform witness pair, a GS4 parametric-singularity chart, the
  GS2 oblique cylinder cut, a GS1 curved silhouette, a GS5 non-watertight mesh, a GS6
  coplanar-overlap out-of-scope verdict) while OCCT does answer. First-class, logged,
  NOT a bar failure. The harness SHALL assert the decline is HONEST — the native
  service returned its typed decline AND the trial genuinely lies in that documented
  out-of-scope sub-domain — so a decline can never mask a wrong answer.
- **DISAGREE** — the native service returns a CONFIDENT answer that does NOT match the
  closed-form analytic ground truth (or, absent a closed form, the OCCT oracle) beyond
  the fixed tolerance. A genuine SILENT WRONG ANSWER — the fault this harness exists to
  catch. FAILS the bar.
- **ORACLE-INACCURATE** — the native answer DIFFERS from the OCCT oracle but MATCHES
  the closed-form analytic ground truth while OCCT does NOT. The native answer is
  CORRECT and vindicated by exact math; OCCT is the outlier. Logged in full, NOT a
  native fault, NOT a bar failure.
- **BOTH-DECLINE** — a decline-exerciser where the native service produced no
  comparable answer AND OCCT also could not produce one: neither engine emitted a wrong
  answer, so there is nothing to compare. Logged, NOT a bar failure.

An unreliable oracle SHALL NOT be laundered into a pass: when the closed-form arbiter
exists it is authoritative over OCCT; a native result is exonerated only when it
POSITIVELY matches exact math while OCCT does not.

#### Scenario: A native answer matching OCCT and the closed form is AGREE

- GIVEN an in-scope trial whose native service returns a confident answer
- WHEN the native answer matches both the OCCT oracle and the closed-form analytic ground truth within the fixed per-service tolerance
- THEN the trial SHALL be classified AGREE and SHALL contribute to that service's coverage count

#### Scenario: The GS2 oblique cylinder cut is an honest decline, not a disagreement

- GIVEN an OBLIQUE cut plane through a cylinder — the regime the `ssi/plane_conics` `intersectPlaneCylinder` semi-major bug (`R/|sinθ|` instead of `R/|cosθ|`) makes unreliable
- WHEN GS2 `sectionByPlane` returns its documented typed decline for that oblique cylinder cut while OCCT `BRepAlgoAPI_Section` answers
- THEN the trial SHALL be classified HONEST-NATIVE-DECLINE (the harness asserting the decline is honest and the trial genuinely oblique-cylinder), logged with its seed, and SHALL NOT be classified DISAGREE nor skipped — and if a future `ssi/` fix makes GS2 answer, the same trial SHALL re-classify AGREE with no harness change

#### Scenario: A confident native answer that fails the closed form is a silent-wrong-answer

- GIVEN a trial whose native service returns a confident answer
- WHEN that answer disagrees with the closed-form analytic ground truth (or, absent a closed form, the OCCT oracle) beyond the fixed tolerance
- THEN the trial SHALL be classified DISAGREE, the harness SHALL FAIL the bar, and it SHALL print the seed, service, case index, input tuple, and the native/OCCT/analytic values so the native fault is reproducible and not papered over

#### Scenario: A native answer vindicated by exact math is not a false native fault

- GIVEN a trial where the native answer differs from the OCCT oracle but matches the closed-form analytic ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: GS6 validity is fuzzed against deterministically-broken solids with a known ground truth

The harness SHALL fuzz the GS6 validity checker against broken solids, because the
checker's interesting signal is on INVALID inputs. It SHALL generate —
deterministically from the same seed — both valid solids AND
broken variants with a KNOWN construction-time ground-truth verdict: a **hole** (a
dropped face → open shell), an **orientation defect** (one face's winding flipped),
and a **self-intersection** (a translated sub-mesh crossing the body). The harness
SHALL classify the native `validity` report against BOTH the OCCT
`BRepCheck_Analyzer::IsValid` oracle AND the construction-time ground truth. A native
verdict that matches both is AGREE; a native confident verdict that contradicts the
construction-time ground truth is DISAGREE. A coplanar-overlap self-intersection that
GS6 marks `selfIntersectionCertified == false` SHALL be classified HONEST-NATIVE-DECLINE
(a documented out-of-scope verdict), NOT a DISAGREE.

#### Scenario: A broken solid is flagged invalid in agreement with the ground truth

- GIVEN a deterministically-broken solid (hole / flipped face / self-intersection) with a known-invalid construction-time ground truth
- WHEN GS6 native validity reports it invalid AND OCCT `BRepCheck_Analyzer::IsValid` also reports it invalid
- THEN the trial SHALL be classified AGREE and contribute to GS6 coverage

#### Scenario: A coplanar-overlap pair GS6 cannot certify is an honest decline

- GIVEN a self-intersection reachable only as a coplanar-overlap pair that GS6's transversal predicate cannot decide (`selfIntersectionCertified == false`)
- WHEN GS6 declares the self-intersection check out of scope rather than assert a clean verdict
- THEN the trial SHALL be classified HONEST-NATIVE-DECLINE, logged, and SHALL NOT be classified DISAGREE

### Requirement: GS-services coverage summary, zero-silent-wrong-answer bar, and logged honest scope

The harness SHALL print a coverage summary — the per-service counts of AGREE /
HONEST-NATIVE-DECLINE / DISAGREE / ORACLE-INACCURATE / BOTH-DECLINE — and SHALL exit 0
IF AND ONLY IF the bar holds: **DISAGREE == 0** across the batch, with real per-service
coverage (each COVERED GS service exercised with at least one AGREE trial, including an
oblique/tilted trial where the service accepts it) proven across **at least two
distinct seeds**. Any DISAGREE (and any ORACLE-INACCURATE) SHALL be reported with its
seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently cap the
batch, or drop trials to make the bar pass: any capped or skipped trial SHALL be
logged. The honest scope SHALL be logged explicitly — each covered service's documented
decline sub-domain (notably the GS2 oblique-cylinder `plane_conics` decline), the
mesh-vs-exact deflection boundary for the mesh-derived services, and any GS service
that is not cleanly fuzzable within this slice (documented and reduced to a coarse
invariant or deferred, with the cleanly-fuzzable subset covered) SHALL appear in the
summary, not be silently omitted.

#### Scenario: Zero disagreements across multiple seeds with real per-service coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering the covered GS services incl. their oblique/tilted regimes
- WHEN no trial is classified DISAGREE and each covered service has at least one AGREE trial
- THEN the harness SHALL print the per-service coverage summary, log the honest scope (each decline sub-domain, the GS2 oblique-cylinder decline, and the deflection boundary), and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREE trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREE trial SHALL be reported with its seed, service, case index, input tuple, and the native/OCCT/analytic values, so the silent-wrong-answer is reproducible and not laundered into a pass

#### Scenario: Honest scope, dropped trials, and any non-fuzzable service are logged, not silently omitted

- GIVEN a run in which some trials are decline-exercisers (BOTH-DECLINE), the batch is capped for time, or a GS service proves not cleanly fuzzable within this slice
- WHEN the harness summarises
- THEN each covered service's decline sub-domain, the GS2 oblique-cylinder `plane_conics` decline, the mesh-vs-exact deflection boundary, any capped or skipped trial, and any service reduced to a coarse invariant or deferred SHALL be logged explicitly in the summary, so no honest exclusion is hidden

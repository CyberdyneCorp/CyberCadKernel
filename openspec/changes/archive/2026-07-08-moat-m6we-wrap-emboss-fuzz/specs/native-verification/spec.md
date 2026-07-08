# native-verification

## ADDED Requirements

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

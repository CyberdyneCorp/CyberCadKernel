# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random freeform-boolean pose generator

The freeform-boolean differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of pose-families and pose
parameters.

Each generated trial SHALL parameterise (a) a pose-family drawn from {off-centre
half-space, disjoint slab, curved-wall CUT, curved-wall COMMON, bicylinder Steinmetz
COMMON}, and (b) a random VALID cut POSE for that family — a half-space plane offset,
a slab half-width, a curved-wall cut height, or an equal cylinder radius — drawn in the
RELIABLE interior band of the family's landed per-op fixture so the native verb is
EXERCISED (not merely declined), with a minority of deliberate out-of-envelope DECLINE
probes. The operand geometry SHALL be the SAME landed fixture operand the per-op parity
harness builds; the RANDOM degree of freedom SHALL be the cut pose.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the freeform-boolean fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` pose-family + pose-parameter trials
- THEN the two sequences SHALL be byte-identical, with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each pose has a closed-form volume

- GIVEN a generated trial
- WHEN its pose-family and pose parameter are inspected
- THEN the boolean result's volume SHALL be given by an engine-independent closed form — a polynomial footprint integral for the half-space and slab families, a paraboloid-segment cap `π·ρ²·c/2` (and its complement) for the curved-wall families, and `16 R³/3` for the Steinmetz family — so that closed form is a ground-truth arbiter for the well-conditioned interior poses

### Requirement: Shipping native verb, OCCT reconstruction, and closed-form arbiter

For each trial the harness SHALL drive the SHIPPING native freeform-boolean verb, the OCCT
reference boolean on a reconstruction of the SAME operand, and the closed-form arbiter. The
**native** candidate SHALL be the OCCT-FREE `freeformHalfSpaceCut` /
`freeformSlabDisjointCut` / `curvedWallHalfSpaceCut` / `nb::boolean_solid`-Steinmetz verb,
measured by the native M0 tessellator (watertight + enclosedVolume + surfaceArea). The
**OCCT oracle** SHALL reconstruct the SAME operand (a `Geom_BezierSurface` prism / bowl-cup
sewn into a solid, or `BRepPrimAPI_MakeCylinder` cylinders) and apply the reference boolean
via `BRepAlgoAPI_{Cut,Common,Fuse}`, measured by exact deflection-independent `BRepGProp`.
The **ground-truth arbiter** SHALL be the closed-form volume, compared to the native
volume, in the well-conditioned interior.

A pre-boolean OPERAND SELF-CHECK SHALL confirm the native and OCCT operand volumes agree
with each other AND with the closed-form full volume, so any downstream disagreement is
attributable to the boolean, never to a mismatched input.

The comparison tolerances SHALL be FIXED and applied PER FAMILY — 2e-2 (native-vs-OCCT and
native-vs-closed-form volume/area) for the near-exact polynomial prism / slab / Steinmetz
families measured at deflection 0.004, and 3e-2 for the STEEP paraboloid bowl-cup measured
at the fine deflection 0.001. The curved-cup 3e-2 band is the deflection-convergence
tolerance the landed `native_curved_wall_cut_parity` harness already validated for this
exact cup (>3× tighter than its 0.10 band), NOT a widening. The tolerances SHALL NOT be
widened to force a pass.

#### Scenario: Native and OCCT operate on the same reconstructed operand

- GIVEN one generated trial
- WHEN the native verb cuts the fixture operand and the OCCT oracle cuts a reconstruction of the SAME operand by the SAME pose
- THEN the operand self-check SHALL confirm the two operands agree in volume/area and with the closed-form full volume, and the closed-form volume SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap in the well-conditioned interior

#### Scenario: An in-envelope pose agrees with OCCT and the closed form

- GIVEN an in-envelope cut pose of one of the five families
- WHEN native produces a watertight solid and OCCT produces a valid measurement
- THEN the native solid's volume SHALL match OCCT within the fixed band, its volume SHALL match the closed form within the fixed band (where well-conditioned), and the trial SHALL be classified AGREED

#### Scenario: An out-of-envelope pose declines under native and is not a wrong boolean

- GIVEN a cut pose outside the native envelope (e.g. a near-rim curved-wall cut, a slab too wide to part cleanly, or a freeform FUSE with no landed verb)
- WHEN the native verb returns a NULL / non-watertight result and OCCT ships a valid result
- THEN the trial SHALL be classified HONESTLY-DECLINED (native defers to OCCT), a first-class outcome, never a bar failure

### Requirement: Classifier arbitrated by the closed form with native-vindication

The harness SHALL classify each freeform-boolean trial into EXACTLY ONE bucket at the
fixed per-family tolerances:

- **AGREED** — native returned a watertight solid whose volume/area match OCCT within the
  band, and match the closed form within the band where the closed form is well-conditioned.
- **HONESTLY-DECLINED** — native returned NULL / non-watertight (a measured decline) while
  OCCT formed a valid closed solid. First-class, logged, NOT a bar failure.
- **DISAGREED** — native returned a watertight solid whose volume/area disagree with a
  trustworthy oracle beyond the fixed band. A genuine SILENT WRONG RESULT — the failure
  this harness exists to catch. FAILS the bar.
- **ORACLE_UNRELIABLE** — native matches the closed-form ground truth while OCCT does NOT.
  The native result is CORRECT and vindicated by exact math; OCCT is the outlier. Logged in
  full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — an out-of-scope probe both engines refuse.
- **FALLBACK_ORACLE_INVALID** — native declined but the shipped OCCT result is itself
  invalid (a broken oracle laundered as a decline). Investigate; FAILS the bar.

Where the closed form is ill-conditioned (the curved-wall COMMON identity near the
paraboloid rim, where `V(full)−V(cut)` suffers catastrophic cancellation), the harness
SHALL fall back to the native-vs-OCCT cross-check and SHALL NOT treat a closed-form
mismatch as a native DISAGREE when the two independent engines mutually agree.

#### Scenario: A native boolean matching OCCT and the closed form is AGREED

- GIVEN an in-envelope trial
- WHEN the native solid matches both OCCT and the well-conditioned closed form within the fixed band with a watertight mesh
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A watertight native boolean that fails a trustworthy oracle is a silent-wrong-result

- GIVEN a trial whose native verb returned a valid, watertight solid
- WHEN that solid's volume/area disagree with a trustworthy oracle beyond the fixed band
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/op/pose tuple so the native fault is reproducible

#### Scenario: A native boolean vindicated by exact math is not a false native fault

- GIVEN a trial where the native solid differs from OCCT but matches the closed-form ground truth while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE_UNRELIABLE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage table, zero-silent-wrong-result bar, and logged honest scope

The harness SHALL print a coverage table — the per-pose-family × op counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE trials for each of the five families
{off-centre-halfspace, disjoint-slab, curved-wall-CUT, curved-wall-COMMON, bicyl-COMMON}
— and SHALL exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0** (with no
OPERAND_MISMATCH and no FALLBACK_ORACLE_INVALID) across the batch, with real coverage
(each of the five families with at least one AGREED trial) proven across **at least two
distinct seeds**, N ≥ 60 per seed. Any DISAGREED SHALL be reported with its seed so it is
reproducible. The harness SHALL NOT weaken a tolerance, silently cap the batch, or drop
trials to make the bar pass.

#### Scenario: Zero silent-wrong results across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every one of the five pose-families
- WHEN no trial is classified DISAGREED (and no OPERAND_MISMATCH / FALLBACK_ORACLE_INVALID), and each of the five families has at least one AGREED trial
- THEN the harness SHALL print the per-pose-family coverage table and exit 0

#### Scenario: Any silent-wrong result fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and family/op/pose tuple, so the silent-wrong-result is reproducible and not laundered into a pass

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-envelope declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-pose-family decline counts SHALL appear in the coverage table, so no honest exclusion is hidden

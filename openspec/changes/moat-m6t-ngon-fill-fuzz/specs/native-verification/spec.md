# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random N-sided-fill boundary generator

The N-sided-fill differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of families, N values,
boundary loops, and per-side kinds on any machine.

Each generated trial SHALL parameterise (a) one of the four families {planar-Ngon,
planar-hole-completion, saddle-4sided, arc-boundary}, (b) a side count N ∈ {3,4,5,6}, and
(c) a random VALID 3–6-sided ANALYTIC boundary loop — random corner positions with per-side
kind straight-segment or circular-arc, planar (in a random plane for planar-Ngon; in a
coordinate plane for planar-hole-completion) or non-planar (a bounded-amplitude saddle for
saddle-4sided; an in-plane arc bulge for arc-boundary). The loop SHALL be filled through the
SAME public `cc_fill_ngon` facade the application calls, under the ACTIVE engine — not by
calling a native builder directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the N-sided-fill fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` family + side-count + boundary-loop trials
- THEN the two sequences SHALL be byte-identical (same families, same N, same numeric loop parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each planar fill has a closed-form area

- GIVEN a generated planar-Ngon or planar-hole-completion trial
- WHEN its boundary loop is inspected
- THEN the filled patch's area SHALL be given by an exact closed form — the Newell 3-D polygon area of the loop corners — so that closed form is an engine-independent ground-truth arbiter, exact for the ideal planar fill (and, for hole-completion, exactly the missing planar face area a hole-completing weld restores)

### Requirement: Both-engine facade drive with closed-form, OCCT, and boundary-residual oracles

For each trial the harness SHALL fill the SAME boundary loop two ways through the public
`cc_fill_ngon` facade and arbitrate. The **native** candidate SHALL be produced under
`cc_set_engine(1)` (the NativeEngine Coons/Gregory tessellated patch with its honest OCCT
fallback), and the **OCCT oracle** under `cc_set_engine(0)` (OCCT `BRepFill_Filling` on the
SAME analytic boundary). Both patches' surface AREA SHALL be read through the public
`cc_mass_properties` (reading the `area` field directly, since a fill patch is an OPEN
surface for which the native `valid` flag — which requires a watertight positive-volume
solid — is 0 by design), and both bounding boxes through `cc_bounding_box`.

Because OCCT's fill is energy-minimizing and the native fill is a transfinite interpolant,
the two INTERIOR patches legitimately differ; the harness SHALL NOT compare interior-vertex
identity. For **planar** families the PRIMARY arbiter SHALL be the exact 3-D polygon area,
compared to the native area, with OCCT cross-checked within a tight band. For **non-planar**
families the arbiter SHALL be the OCCT area within a fixed band PLUS bbox-containment (the
native patch bbox contained in OCCT's fill bbox grown by a slack) PLUS an OCCT-INDEPENDENT
analytic-boundary residual (every native boundary sample lies on its straight/circular-arc
curve). The comparison tolerances SHALL be FIXED — planar native-vs-closed-form area ≤ 1e-4,
planar OCCT-vs-closed-form ≤ 1e-3, non-planar native-vs-OCCT area ≤ 1.2e-1, bbox-containment
slack ≤ 8e-2, boundary residual ≤ 1e-6 — and SHALL NOT be widened to force a pass. The
non-planar saddle amplitude SHALL be bounded to the small-warp regime in which the transfinite
and energy-minimizing surfaces co-area within the fixed band (a generator scope bound, not a
tolerance widening).

#### Scenario: Native and OCCT fill the identical boundary through the facade

- GIVEN one generated trial
- WHEN the native candidate is produced under `cc_set_engine(1)` and the OCCT oracle under `cc_set_engine(0)`, both filling the SAME analytic boundary loop
- THEN both SHALL have filled the SAME known loop, and — for a planar family — the closed-form polygon area SHALL be the PRIMARY correctness oracle used to attribute a native-vs-OCCT gap

#### Scenario: An in-scope planar fill agrees with OCCT and the closed form

- GIVEN a planar-Ngon or planar-hole-completion loop
- WHEN native produces a valid patch and OCCT produces a valid patch
- THEN the native patch area SHALL match the exact 3-D polygon area within the fixed band, its boundary samples SHALL lie on their analytic curves, OCCT SHALL concur, and the trial SHALL be classified AGREED

#### Scenario: An in-scope non-planar fill agrees within the deflection band

- GIVEN a saddle-4sided or arc-boundary loop in the small-warp/arc scope
- WHEN native produces a valid patch and OCCT produces a valid patch
- THEN the native area SHALL match OCCT within the fixed non-planar band, the native patch bbox SHALL be contained in OCCT's fill bbox grown by the slack, every native boundary sample SHALL lie on its analytic curve, and the trial SHALL be classified AGREED

#### Scenario: An out-of-bound loop declines under native and is not a wrong patch

- GIVEN an out-of-bound loop (e.g. a self-intersecting bowtie) outside the native scope
- WHEN native `cc_fill_ngon` returns 0 / an invalid patch
- THEN the trial SHALL be classified HONESTLY-DECLINED (OCCT filled it) or BOTH-DECLINED (OCCT refused it too), and this SHALL be a first-class outcome, never a bar failure

### Requirement: Six-way classifier arbitrated by the closed-form or OCCT oracle

The harness SHALL classify each fill trial into EXACTLY ONE bucket at the fixed
tolerances:

- **AGREED** — native returned a valid patch (positive area, boundary samples on their
  analytic curves) whose area matches the arbiter within the fixed band (the exact polygon
  area for planar; OCCT for non-planar, with bbox-containment), AND — for planar — OCCT also
  matches the closed form.
- **HONESTLY-DECLINED** — native `cc_fill_ngon` returned 0 / an invalid patch (an
  out-of-bound loop) while OCCT filled a valid patch. First-class, logged, NOT a bar
  failure.
- **DISAGREED** — native returned a valid patch whose area does NOT match the exact polygon
  area (planar) beyond the fixed band while OCCT matches it, OR a non-planar patch whose
  area/bbox/residual violates the fixed non-planar arbiter. A genuine SILENT WRONG patch —
  the failure this harness exists to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form polygon area while OCCT does NOT.
  The native patch is CORRECT and vindicated by exact math; OCCT is the outlier. Logged in
  full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — an out-of-bound loop both engines refuse (including the bowtie
  decline probe).
- **ORACLE_UNRELIABLE** — a non-planar trial whose native patch is valid but no trustworthy
  oracle exists to arbitrate its area. FAILS the bar (investigate; never laundered).

#### Scenario: A native patch matching OCCT and the closed form is AGREED

- GIVEN an in-scope fill trial
- WHEN the native patch matches its arbiter (exact polygon area for planar; OCCT area + bbox-containment for non-planar) within the fixed band with boundary samples on their analytic curves
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's coverage count

#### Scenario: A valid native patch that fails the closed form is a silent-wrong-result

- GIVEN a planar fill trial whose native `cc_fill_ngon` returned a valid patch
- WHEN that patch's area disagrees with the exact polygon area beyond the fixed band while OCCT matches it
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and family/N/parameter tuple so the native fault is reproducible

#### Scenario: A native patch vindicated by exact math is not a false native fault

- GIVEN a planar trial where the native area matches the closed-form polygon area while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-patch bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED / ORACLE-INACCURATE / ORACLE_UNRELIABLE / BOTH-DECLINED
trials for each of the four {planar-Ngon, planar-hole-completion, saddle-4sided,
arc-boundary} families — and SHALL exit 0 IF AND ONLY IF the bar holds: **DISAGREED == 0
AND ORACLE_UNRELIABLE == 0** across the batch, with real coverage (each of the four
families with at least one AGREED trial) proven across **at least two distinct seeds**,
N ≥ 60 per seed. Any DISAGREED (and any ORACLE-INACCURATE or ORACLE_UNRELIABLE) SHALL be
reported with its seed so it is reproducible. The harness SHALL NOT weaken a tolerance,
silently cap the batch, or drop trials to make the bar pass.

#### Scenario: Zero silent-wrong patches across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every one of the four families across N ∈ {3,4,5,6}
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, and each of the four families has at least one AGREED trial
- THEN the harness SHALL print the per-family coverage summary and exit 0

#### Scenario: Any silent-wrong patch fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and family/N/parameter tuple, so the silent-wrong-patch is reproducible and not laundered into a pass

#### Scenario: Honest declines are logged, not silently omitted

- GIVEN a run in which some trials are out-of-bound declines (HONESTLY-DECLINED / BOTH-DECLINED)
- WHEN the harness summarises
- THEN the per-family decline counts SHALL appear in the summary, so no honest exclusion is hidden

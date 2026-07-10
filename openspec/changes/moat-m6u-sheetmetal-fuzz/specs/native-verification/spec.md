# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random sheet-metal input generator

The sheet-metal differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed
and batch size SHALL produce a **byte-identical** sequence of ops, profiles, thicknesses,
edge selections, bend parameters, and k-factors on any machine.

Each generated trial SHALL parameterise ONE of the three sheet-metal ops: (a) a BASE flange
from a random simple polygon profile (rectangle / regular n-gon / convex-jittered n-gon) ×
a random thickness; (b) an EDGE flange off the +X straight rim of a random `L×W×t` base,
with a random inner bend radius, bend angle in `(0°,180°)`, and wall height; (c) an UNFOLD
of that folded part at a random k-factor ∈ [0,1]. Every op SHALL be exercised through the
SAME public `cc_sheet_base_flange` / `cc_sheet_edge_flange` / `cc_sheet_unfold` facade the
application calls under the NATIVE engine — not by calling a native builder directly.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the sheet-metal fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` op + profile/parameter trials
- THEN the two sequences SHALL be byte-identical (same ops, same numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Each op has a closed-form arbiter

- GIVEN a generated base-flange, edge-flange-fold, or unfold trial
- WHEN its parameters are inspected
- THEN the correct result SHALL be given by an exact closed form — base volume `|profileArea|·thickness`; folded volume `L·W·t + ½·θ·((r+t)²−r²)·W + height·t·W`; unfold developed area `baseArea + θ·(r+k·t)·W + flangeArea` — so that closed form is an engine-independent ground-truth arbiter, with NO dependence on any OCCT sheet-metal oracle (OCCT has none)

### Requirement: Native-only facade drive arbitrated by closed form, the fold-to-unfold invariant, and validity

The harness SHALL NOT use any OCCT sheet-metal oracle (OCCT core has none) and SHALL NOT drive `cc_set_engine(0)` for these ops nor compare against any OCCT sheet-metal result. For each trial the harness SHALL build the part through the public `cc_sheet_*`
facade under `cc_set_engine(1)` (the NativeEngine, native-only), read its volume through
`cc_mass_properties`, and read its geometric validity through `cc_check_solid`'s per-check
breakdown. It SHALL arbitrate:

- a BASE flange or UNFOLD blank (a PLANAR prism, mesh volume exact) by a HARD volume equality
  to the closed form within a FIXED band (`kExactAbs ≤ 1e-6`), and — for unfold — by the
  fold→unfold AREA INVARIANT residual (`kInvarAbs ≤ 1e-6`);
- a folded EDGE flange (a TRUE cylindrical bend meshed to a deflection, converging FROM
  BELOW) by the closed-form volume within a FIXED band (`kBendBand ≤ 1.5%`, the SAME band the
  product's own `common.h::verifySolid` uses) AND the native volume NOT exceeding the closed
  form.

Every built part SHALL be a valid CLOSED 2-MANIFOLD — watertight, consistently oriented,
non-degenerate, finite, Euler χ=2 (a single genus-0 lump), positive volume — read from
`cc_check_solid`'s per-check flags (`closed_manifold ∧ consistent_orientation ∧
no_degenerate ∧ finite`) and `cc_mass_properties` (`watertight ∧ vol>0`), the SAME contract
the product's own `verifySolid` and the host Gate (a) enforce. The comparison tolerances
SHALL be FIXED and SHALL NOT be widened to force a pass.

#### Scenario: The unfold develops a real folded part (round-trip)

- GIVEN a folded edge flange built in a trial
- WHEN it is unfolded at a random k-factor through `cc_sheet_unfold`
- THEN the developed flat-blank area SHALL be arbitrated by the fold→unfold AREA INVARIANT (`baseArea + θ·(r+k·t)·W + flangeArea` equals the direct `devLength·W`), so the developed footprint area is proven invariant under fold→unfold, not merely re-derived

#### Scenario: An in-scope base flange or unfold blank matches the exact closed form

- GIVEN a base-flange or unfold trial with a valid profile / recognised fold
- WHEN native builds a valid closed 2-manifold
- THEN its volume SHALL equal the exact planar-prism closed form within `kExactAbs`, the unfold area invariant residual SHALL be within `kInvarAbs`, and the trial SHALL be classified AGREED

#### Scenario: An in-scope folded part matches the closed form within the deflection band

- GIVEN an in-slice edge-flange fold
- WHEN native builds a valid closed 2-manifold
- THEN its meshed volume SHALL match the closed-form base+bend+wall volume within `kBendBand` WITHOUT exceeding it (an inscribed convergent mesh only under-fills), and the trial SHALL be classified AGREED (or NATIVE-CHECK-INACCURATE if only the separate GS6 self-X sub-check misreports it — see below)

#### Scenario: An out-of-slice pose declines and is not a wrong solid

- GIVEN an out-of-slice pose (a wrong/non-straight edge id, a degenerate zero-area profile, a bend angle outside (0°,180°), or an unfold of a non-fold body)
- WHEN native `cc_sheet_*` returns 0 with `cc_last_error` set
- THEN the trial SHALL be classified HONESTLY-DECLINED, a first-class outcome, never a bar failure, and the native arm SHALL NEVER emit a wrong or self-intersecting solid in its place

### Requirement: Classifier arbitrated by closed form and the round-trip invariant

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed tolerances:

- **AGREED** — native returned a valid closed 2-manifold (watertight, oriented, χ=2,
  positive volume) whose volume matches the closed form within the op's fixed band (and, for
  unfold, whose area-invariant residual is within `kInvarAbs`).
- **HONESTLY-DECLINED** — native `cc_sheet_*` returned 0 on an out-of-slice pose. First-class,
  logged, NOT a bar failure.
- **DISAGREED** — native returned a valid solid whose volume does NOT match the closed form
  beyond the fixed band, OR the fold→unfold area invariant is violated, OR the built part is
  NOT a valid closed 2-manifold, OR native returned 0 on an IN-slice pose, OR native built a
  solid on a pose it must refuse. A genuine wrong result — the failure this harness exists to
  catch. FAILS the bar. Reported with seed + case index + parameter tuple.
- **NATIVE-CHECK-INACCURATE** — the built part is CORRECT by every geometric arbiter this
  harness owns (watertight / oriented / χ=2 / volume==closed-form / invariant), but a SEPARATE
  native CHECK misreports it — specifically `cc_check_solid`'s GS6 `no_self_intersection`
  sub-check false-positives on the tessellated-cylinder bend. This is the no-OCCT-oracle
  analogue of the sibling fuzzers' ORACLE-INACCURATE: logged in full, measured, NOT a native
  geometry fault, NOT a bar failure. It still counts toward per-op coverage.

#### Scenario: A native part matching the closed form and valid as a closed 2-manifold is AGREED

- GIVEN an in-scope trial
- WHEN the native part matches its closed-form arbiter within the fixed band and is a valid closed 2-manifold (and, for unfold, satisfies the area invariant)
- THEN the trial SHALL be classified AGREED and SHALL contribute to that op's coverage count

#### Scenario: A valid part whose volume or invariant misses the closed form is a wrong result

- GIVEN a trial whose native `cc_sheet_*` returned a built solid
- WHEN that solid's volume disagrees with the closed form beyond the fixed band, or the fold→unfold area invariant is violated, or it is not a valid closed 2-manifold
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and parameter tuple so the native fault is reproducible

#### Scenario: A part correct by every geometric arbiter but flagged by a separate native check is not a false native fault

- GIVEN a folded part that is watertight, consistently oriented, χ=2, and whose volume equals the closed form
- WHEN `cc_check_solid`'s GS6 `no_self_intersection` sub-check nonetheless reports self-intersection (a pre-existing false positive on the tessellated-cylinder bend, reproducible on the base commit)
- THEN the trial SHALL be classified NATIVE-CHECK-INACCURATE, logged in full with a note that it is a reported product-check finding (not a sheet-metal fold-geometry fault), and SHALL NOT be counted as a native geometry fault nor fail the bar

### Requirement: Coverage summary, zero-disagree bar, and logged honest scope

The harness SHALL print a coverage summary — the per-op counts of AGREED /
HONESTLY-DECLINED / DISAGREED / NATIVE-CHECK-INACCURATE trials for each of the three
{base-flange, edge-flange-fold, unfold} ops — and SHALL exit 0 IF AND ONLY IF the bar
holds: **DISAGREED == 0** across the batch, with real coverage (each of the three ops with
at least one geometrically-correct trial — AGREED or NATIVE-CHECK-INACCURATE) proven across
**at least two distinct seeds**, N ≥ 60 per seed. Any DISAGREED SHALL be reported with its
seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently cap the
batch, or drop trials to make the bar pass.

#### Scenario: Zero wrong results across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering all three ops
- WHEN no trial is classified DISAGREED, and each of the three ops has at least one geometrically-correct trial
- THEN the harness SHALL print the per-op coverage summary and exit 0

#### Scenario: Any wrong result fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and parameter tuple, so the wrong result is reproducible and not laundered into a pass

#### Scenario: Honest declines and native-check-inaccurate notes are logged, not silently omitted

- GIVEN a run in which some trials are out-of-slice declines or GS6 self-X false positives
- WHEN the harness summarises
- THEN the per-op HONESTLY-DECLINED and NATIVE-CHECK-INACCURATE counts SHALL appear in the summary, so no honest exclusion or reported check-finding is hidden

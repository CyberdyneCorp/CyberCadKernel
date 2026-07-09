# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random solid, rigid-pose, and view generator for HLR fuzzing

The orthographic-HLR / drafting differential-fuzzing harness SHALL generate its batch from
a **deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator (splitmix64
seeding a xoshiro256** stream) keyed ONLY by an explicit integer `FUZZ_SEED`
(argv/env-overridable, with a fixed default). Re-running the harness with the same seed and
batch size SHALL produce a **byte-identical** sequence of solid parameters, rigid poses, and
view directions on any machine.

Each generated trial SHALL parameterise (a) a VALID, non-degenerate base solid drawn from
six families — a `BOX`, an `NGON` prism (3..8 sides), a `CYLINDER`, a `CONE`/frustum, a
`SPHERE`, and a `FREEFORM` (B-spline-meridian revolve producing `Kind::BSpline` bands) —
built IDENTICALLY under both engines via the public `cc_solid_extrude` / `cc_solid_revolve`
/ `cc_solid_revolve_profile` facade; (b) a random RIGID pose (a rotation about a random unit
axis by a random angle via `cc_rotate_shape_about`, then a random translation via
`cc_translate_shape`) with NO scale and NO mirror, applied IDENTICALLY under both engines so
the projected silhouette is an exact isometry of the base; and (c) a random VIEW direction
with a non-parallel up hint yielding a well-conditioned drawing-plane basis.

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the HLR fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` solid + pose + view trials
- THEN the two sequences SHALL be byte-identical (same families, same numeric solid/pose/view parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Solids are built and posed identically under both engines

- GIVEN a generated solid + rigid pose
- WHEN it is built under `cc_set_engine(0)` (OCCT) and under `cc_set_engine(1)` (NativeEngine) from the same construction parameters and posed by the same `cc_rotate_shape_about` + `cc_translate_shape`
- THEN both engines SHALL produce the same nominal posed solid, so the HLR projection under each engine acts on a geometrically-identical body and the two `CCDrawing`s are directly comparable in the same drawing-plane basis

### Requirement: Each solid projected under both engines and compared as a labelled segment set with a closed-form silhouette arbiter

For each trial the harness SHALL project the posed solid ONCE under the OCCT engine
(`cc_set_engine(0)`, the `HLRBRep_Algo` oracle) and ONCE under the NativeEngine
(`cc_set_engine(1)`, the OCCT-free orthographic-HLR + silhouette core) through the public
`cc_hlr_project` facade, and SHALL compare the two visible/hidden 2D drawing-plane segment
sets. Both engines return `CCDrawing` in the SAME drawing-plane basis
(`right = normalize(view × up)`, `trueUp = right × view`).

The comparison SHALL treat the visible/hidden classification as a LABELLED POINT SET:

- **PARTITION (authoritative)** — every native VISIBLE segment's endpoints SHALL lie on an
  oracle VISIBLE segment, and every native HIDDEN segment's endpoints on the oracle OUTLINE
  (visible∪hidden), within the tolerance (native ⊆ oracle: no misclassified or fabricated
  segment); AND the REVERSE — every oracle VISIBLE segment on a native VISIBLE segment and
  every oracle HIDDEN segment on the native OUTLINE (oracle ⊆ native: no missing arc). This
  BIDIRECTIONAL coverage proves the two engines trace the IDENTICAL visible/hidden locus.
- **COUNTS** — for the POLYHEDRAL convex families (`BOX` / `NGON` prism) the visible and
  hidden segment counts are deterministic and SHALL equal the oracle. For QUADRIC
  silhouettes the two discretizers differ, so counts SHALL NOT be required equal.
- **TOTAL LENGTH** — Σ visible and Σ hidden projected length. For polyhedral (exact
  projection) this SHALL match within a tight relative band. For QUADRIC silhouettes total
  length is a discretization-sensitive CORROBORATING proxy (a foreshortened grazing
  silhouette is sampled at different chord densities by two independent discretizers), NOT
  the agreement gate — when only the length band trips but the bidirectional partition
  holds, the outlines are the same locus and the trial AGREES.

For the `CYLINDER` and `SPHERE` families the harness SHALL additionally compute a CLOSED-FORM
silhouette-tangency arbiter (`n·view = 0`) in plain fp64 — the cylinder's two generator lines
at `θ* = atan2(−X·d, Z·d)` and the sphere's great circle in the plane ⟂ view through the
centre — posed by the same rigid transform and projected into the drawing plane, and SHALL
use it to attribute a native-vs-OCCT quadric gap that survives the bidirectional partition.

The tolerances SHALL be FIXED and SHALL NOT be widened to force a pass: polyhedral partition
`1e-4 mm` and total-length relative `5e-4`; quadric partition `0.08 mm`, total-length
relative band `3e-2` (corroborating only), and silhouette-arbiter `0.08 mm`.

#### Scenario: Native, OCCT, and the closed form all evaluate the identical posed solid + view

- GIVEN one generated trial
- WHEN the projection runs under the OCCT engine on the OCCT-built posed solid, under the NativeEngine on the native-built posed solid, and (for cylinder/sphere) the fp64 closed-form silhouette is posed and projected from the same view
- THEN all three SHALL have operated on the SAME nominal posed solid and the SAME view, in the SAME drawing-plane basis, and the closed form SHALL be the correctness oracle used to attribute a native-vs-OCCT quadric gap

#### Scenario: A polyhedral solid matches the oracle exactly as a labelled segment set

- GIVEN a `BOX` or `NGON` prism at a random rigid pose projected from a random view under both engines
- WHEN the native and oracle visible/hidden segment sets are compared
- THEN the visible and hidden counts SHALL be equal, the total visible and hidden lengths SHALL match within the tight relative band, and the bidirectional partition SHALL hold within `1e-4 mm` (identical labelled point set)

#### Scenario: A quadric silhouette matches the oracle locus regardless of chord sampling

- GIVEN a `CYLINDER`, `CONE`/frustum, or `SPHERE` at a random rigid pose projected from a random view under both engines
- WHEN the native and oracle visible/hidden segment sets are compared by bidirectional partition coverage at the curve tolerance
- THEN a trial whose native and oracle outlines cover each other in both directions (the same visible/hidden locus) SHALL be AGREED even if the total-length proxy differs (a grazing-view discretization artifact), and a native segment drawn in the wrong class or off the oracle outline, or a missing native arc, SHALL fail the bidirectional partition

#### Scenario: A freeform B-spline-band silhouette is honestly declined

- GIVEN a `FREEFORM` (B-spline-meridian revolve → `Kind::BSpline` bands) solid whose silhouette has no closed form
- WHEN it is projected under the NativeEngine
- THEN the native core SHALL return an EMPTY drawing (visible = hidden = 0, arrays null, `cc_last_error` set) and the OCCT oracle SHALL serve the projection — the trial SHALL be HONESTLY-DECLINED, NEVER a DISAGREE

### Requirement: Agree / honestly-declined / DISAGREE HLR classifier arbitrated by the oracle and the closed form

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed tolerances, with
the OCCT oracle as the differential reference and the closed-form silhouette as the arbiter
where one exists:

- **AGREED** — both engines drew and the native visible/hidden segment set matches the
  oracle (exact counts + tight length + bidirectional partition for polyhedral;
  bidirectional partition for quadrics, with length corroborating).
- **HONESTLY-DECLINED** — the native service returned an EMPTY drawing (a freeform B-spline
  silhouette) OR the native engine could not build/pose the solid (a native placement scope
  limit, e.g. `cc_rotate_shape_about` declining a revolve-built-frustum rigid placement),
  and the OCCT oracle served a non-empty drawing as the fallback. First-class, logged, NOT a
  bar failure.
- **DISAGREED** — the native engine drew a NON-EMPTY drawing that does NOT match the oracle
  (wrong counts, a segment misclassified visible↔hidden, a fabricated segment off the
  oracle outline, or a missing arc failing the bidirectional partition) and, where a closed
  form exists, is NOT vindicated by it. A genuine SILENT WRONG DRAWING — the failure this
  harness exists to catch. FAILS the bar.
- **ORACLE_UNRELIABLE** — a quadric mismatch where the native outline matches the closed-form
  silhouette while the OCCT oracle is the outlier (OCCT-inaccurate). The native result is
  CORRECT and vindicated by the closed form; OCCT is the outlier. Logged in full, NOT a
  native fault, NOT a bar failure.
- **BOTH-DECLINED** — a degenerate view (e.g. exactly down a quadric axis, where the whole
  side is silhouette) that BOTH engines decline (both drawings empty). Not a native fault.

#### Scenario: A native drawing matching the oracle as a labelled set is AGREED

- GIVEN an in-scope HLR trial where both engines drew
- WHEN the native visible/hidden segment set matches the oracle within the fixed tolerances (counts + length + bidirectional partition for polyhedral; bidirectional partition for quadrics)
- THEN the trial SHALL be classified AGREED and SHALL contribute to that family's and that view regime's coverage count

#### Scenario: A native drawing that fails the labelled-set match is a silent-wrong-result

- GIVEN an HLR trial whose native engine drew a non-empty drawing
- WHEN that drawing fails the bidirectional partition against the oracle (a misclassified segment, a fabricated segment off the outline, or a missing arc) and, for a quadric family, is not vindicated by the closed-form silhouette
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, and full solid/pose/view tuple so the native fault is reproducible

#### Scenario: A native quadric silhouette vindicated by the closed form is not a false native fault

- GIVEN a quadric trial where the native outline differs from OCCT beyond tolerance but matches the closed-form silhouette while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE_UNRELIABLE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-result bar, and logged honest scope

The harness SHALL print a coverage summary — the per-family and per-view-regime counts of
AGREED / HONESTLY-DECLINED / DISAGREED / ORACLE_UNRELIABLE trials — and SHALL exit 0 IF AND
ONLY IF the bar holds: **DISAGREED == 0** across the batch, with real coverage (each of
`BOX`, `NGON`, `CYLINDER`, `CONE`, `SPHERE` exercised with at least one AGREED trial and the
`FREEFORM` family exercised as an honest decline) proven across **at least two distinct
seeds** at **N ≥ 60 per seed**. Any DISAGREED (and any ORACLE_UNRELIABLE) SHALL be reported
with its seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently cap
the batch, or drop trials to make the bar pass.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds at N ≥ 60 with a batch covering every solid family and all three view regimes
- WHEN no trial is classified DISAGREED, and each of BOX/NGON/CYLINDER/CONE/SPHERE has at least one AGREED trial and FREEFORM is exercised as a decline
- THEN the harness SHALL print the per-family / per-view-regime coverage summary and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, and full solid/pose/view tuple, so the silent-wrong-result is reproducible and not laundered into a pass

#### Scenario: A grazing-view length-proxy over-fire under a holding partition is not a false failure

- GIVEN a grazing-view quadric trial whose total-projected-length proxy exceeds its band while the bidirectional partition holds (the two engines trace the same locus at different chord densities)
- WHEN the classifier decides the verdict
- THEN the trial SHALL be classified AGREED (same locus), the length-proxy trip SHALL be logged, and NO tolerance SHALL be widened to reach that verdict

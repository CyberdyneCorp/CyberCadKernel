# native-verification

## ADDED Requirements

### Requirement: Deterministic seeded random base-solid and transform-chain generator

The transform-chain differential-fuzzing harness SHALL generate its batch from a
**deterministic, explicitly-seeded** pseudo-random number generator. The harness SHALL
NOT read any wall clock, `rand()`, `Date`, process id, address, or any other
non-deterministic source; the RNG SHALL be a self-contained integer generator
(splitmix64 seeding a xoshiro256** stream) keyed ONLY by an explicit integer
`FUZZ_SEED` (argv/env-overridable, with a fixed default). Re-running the harness with
the same seed and batch size SHALL produce a **byte-identical** sequence of base-solid
parameters and transform-chain operations on any machine.

Each generated trial SHALL parameterise (a) a VALID, non-degenerate base solid drawn
from the families the native path builds and meshes watertight — a `BOX`, an
`NGON_PRISM`, a `CYLINDER`, a `SPHERE`, and a coaxial `LOFT` — and (b) a CHAIN of one
to four transform operations, each drawn from `TRANSLATE`, `ROTATE` (about a random
unit axis through a random centre by a random angle), `USCALE` (a positive uniform
scale factor about a random centre), and `MIRROR` (reflection across a random plane).
The chain SHALL be a SIMILARITY (rotation, translation, uniform scale, mirror only) so
that the transformed volume, area, centroid, topology, and handedness have an exact
closed form. The generator SHALL guarantee that each transform KIND is exercised (by
forcing each kind singly in an initial block) and MAY emit **sparse,
explicitly-labelled DECLINE-exercisers** that append a singular (zero-scale) operation
whose only purpose is to reach the collapsed-solid / no-valid-mesh path. The generator
SHALL be OCCT-free (it produces plain parameter POD consumed identically by the native
builder, the OCCT oracle builder, and the analytic arbiter).

#### Scenario: Same seed reproduces the identical batch (determinism)

- GIVEN the transform fuzz harness run twice with the same explicit `FUZZ_SEED` `S` and batch size `N`
- WHEN each run generates its sequence of `N` base-solid + transform-chain trials
- THEN the two sequences SHALL be byte-identical (same base families, same chain operations and numeric parameters in the same order), with no dependence on wall clock, `rand()`, address, or thread scheduling

#### Scenario: Only uniform-scale similarities are generated (honest scope)

- GIVEN a generated transform chain
- WHEN its operations are inspected
- THEN every scale operation SHALL be UNIFORM (a single factor), so the composed map is a similarity `L = S·Q` with `Q` orthonormal, keeping the area closed form exact; an anisotropic scale (which has no closed-form area) SHALL NOT be generated

### Requirement: Three independent drivings of one transform chain with a closed-form similarity arbiter

For each trial the harness SHALL apply the SAME transform chain three independent ways
and SHALL compute a **closed-form analytic ground truth** for the transformed solid.
The **native** driving SHALL compose the chain as one `math::Transform`, apply it via
`topology::Shape::located(Location{...})`, mesh the located solid at the fixed property
deflection, and measure signed + absolute enclosed volume, surface area, centroid, and
the face / edge / vertex counts. The **oracle** driving SHALL compose the SAME chain as
one OCCT `gp_Trsf`, apply it via `BRepBuilderAPI_Transform`, and measure by
`BRepGProp::VolumeProperties` + `SurfaceProperties` + `CentreOfMass`. The **analytic
arbiter** SHALL be a THIRD computation using a plain fp64 affine (NOT the native
`math::Transform`, NOT the OCCT `gp_Trsf`) that composes the identical chain and applies
the exact similarity image to the base solid's exact closed form: `volume' = S³·V₀`,
`area' = S²·A₀`, `centroid' = L·C₀ + t`, with topology counts INVARIANT and the signed
enclosed volume's sign equal to `sign(base)·(−1)^(#mirrors)`.

Because the native answer is a mesh discretisation, the native tolerance SHALL be
**matched to the tessellation deflection bound** (tight for a planar exact-meshing base
such as BOX / NGON_PRISM / LOFT; the deflection-derived convergence bound at the scaled
world feature size for a curved base such as CYLINDER / SPHERE). This tolerance SHALL
NOT be widened beyond that bound.

#### Scenario: Native, OCCT, and the analytic image all transform the identical base solid

- GIVEN one generated trial
- WHEN the native builder builds the base and the native transform path meshes the located solid, the OCCT builder builds the same base and `BRepBuilderAPI_Transform` applies the same `gp_Trsf`, and the independent fp64 affine applies the same chain to the exact base closed form
- THEN all three SHALL have operated on the SAME base solid and the SAME composed similarity, and the analytic volume / area / centroid, the expected topology counts, and the expected handedness parity SHALL have been computed as the PRIMARY correctness oracle

#### Scenario: Topology count is invariant under a transform

- GIVEN a valid base solid and any similarity chain
- WHEN the native located solid is meshed and its face / edge / vertex counts are compared to the base solid's counts
- THEN the counts SHALL be unchanged (a placement adds or drops no sub-shape); a native transform that changes the topology count is a DISAGREE

#### Scenario: A mirror flips handedness yet leaves a valid positive-volume solid

- GIVEN a chain containing an odd number of mirror operations applied to a valid base solid
- WHEN the native located solid is meshed
- THEN the solid SHALL remain watertight with positive absolute volume AND its signed enclosed volume's sign SHALL be flipped relative to the base (handedness reversed); a native mirror that fails to flip the sign, or that collapses / invalidates the solid, is a DISAGREE

#### Scenario: A curved base is held to the deflection-matched bound, not a widened tolerance

- GIVEN a CYLINDER or SPHERE base whose native mesh under-approximates the curved surface at the property deflection
- WHEN the transformed native volume / area is compared to the analytic similarity image
- THEN the tolerance SHALL be the tessellation convergence bound derived from the deflection and the scaled feature size (never an arbitrary widened value), and a planar base (BOX / NGON_PRISM / LOFT) SHALL be held to the tight exact-meshing tolerance

### Requirement: Agree / honestly-declined / DISAGREE transform classifier arbitrated by the closed-form ground truth

The harness SHALL classify each trial into EXACTLY ONE bucket at the fixed,
deflection-matched tolerance, with the closed-form similarity image as the PRIMARY
correctness oracle (used to ATTRIBUTE a native-vs-OCCT gap rather than reflexively
blame the native path):

- **AGREED** — native produced a watertight, positive-|volume| solid whose
  volume / area / centroid match the analytic similarity image within tolerance AND
  whose topology counts are invariant AND whose signed-volume sign matches the
  mirror-parity expectation, AND OCCT concurs.
- **HONESTLY-DECLINED** — native produced no valid mesh (a singular transform
  collapsed the solid → non-watertight / zero volume) while the OCCT build of the same
  trial IS a valid measurable solid. First-class, logged, NOT a bar failure.
- **DISAGREED** — native produced a valid solid but its transformed volume / area /
  centroid, its topology count, or its handedness does NOT match the closed-form
  similarity image. A genuine SILENT WRONG TRANSFORM — the failure this harness exists
  to catch. FAILS the bar.
- **ORACLE-INACCURATE** — native matches the closed-form similarity image but OCCT does
  NOT. The native transform is CORRECT and vindicated by exact math; OCCT is the
  outlier. Logged in full, NOT a native fault, NOT a bar failure.
- **BOTH-DECLINED** — a singular-transform DECLINE-exerciser where native produced no
  valid solid AND OCCT also could not build a valid solid: neither engine produced a
  wrong result. Logged, NOT a bar failure.

An unreliable oracle SHALL NOT be laundered into a pass: when the closed-form arbiter
exists it is authoritative over OCCT; a native result is exonerated only when it
POSITIVELY matches exact math while OCCT does not, and a core-case OCCT that does not
match the closed form is recorded as ORACLE_UNRELIABLE (investigate), never as a pass.

#### Scenario: A native transform matching OCCT and the closed form is AGREED

- GIVEN an in-scope trial whose native located solid is watertight with positive volume
- WHEN native volume / area / centroid match both the OCCT `BRepGProp` measurement and the closed-form similarity image within the deflection-matched tolerance, topology is invariant, and the signed-volume sign matches the mirror parity
- THEN the trial SHALL be classified AGREED and SHALL contribute to that base family's and each involved op kind's coverage count

#### Scenario: A singular transform that collapses the native solid is HONESTLY-DECLINED or BOTH-DECLINED, not a failure

- GIVEN a DECLINE-exerciser trial whose chain appends a singular (zero-scale) operation, collapsing the native solid so the mesh is not watertight
- WHEN the OCCT build of the same trial is either a valid solid or likewise degenerate
- THEN the trial SHALL be classified HONESTLY-DECLINED (OCCT valid) or BOTH-DECLINED (OCCT also degenerate), logged with its seed and descriptor, and SHALL NOT fail the bar — a collapsed transform is never counted as a wrong transform

#### Scenario: A valid native transform that fails the closed form is a silent-wrong-result

- GIVEN a trial whose native located solid is watertight with positive volume
- WHEN native volume, area, or centroid disagrees with the closed-form similarity image beyond the deflection bound, or the topology count changed, or a mirror failed to flip handedness
- THEN the trial SHALL be classified DISAGREED, the harness SHALL FAIL the bar, and it SHALL print the seed, case index, base family / chain descriptor, and the native / OCCT / analytic measurements so the native fault is reproducible

#### Scenario: A native transform vindicated by exact math is not a false native fault

- GIVEN a trial where native differs from OCCT `BRepGProp` but native matches the closed-form similarity image while OCCT does not
- WHEN the classifier attributes the gap
- THEN the trial SHALL be classified ORACLE-INACCURATE (OCCT is the outlier), logged in full, and SHALL NOT be counted as a native fault nor fail the bar

### Requirement: Coverage summary, zero-silent-wrong-transform bar, and logged honest scope

The harness SHALL print a coverage summary — the per-base-family counts of AGREED /
HONESTLY-DECLINED / DISAGREED, the per-op-kind AGREED coverage, and the count of
positively-confirmed mirror handedness flips — and SHALL exit 0 IF AND ONLY IF the bar
holds: **DISAGREED == 0 AND ORACLE_UNRELIABLE == 0** across the batch, with real
coverage (each of BOX, NGON_PRISM, CYLINDER, SPHERE, LOFT and each of
TRANSLATE / ROTATE / USCALE / MIRROR exercised with at least one AGREED trial, and the
mirror handedness flip positively confirmed at least once) proven across **at least two
distinct seeds**. Any DISAGREED (and any ORACLE-INACCURATE) SHALL be reported with its
seed so it is reproducible. The harness SHALL NOT weaken a tolerance, silently cap the
batch, or drop trials to make the bar pass: any capped or skipped trial SHALL be logged.
The honest scope SHALL be logged explicitly — the uniform-scale-only restriction, the
planar-vs-curved deflection tolerance rationale, and the singular-transform decline
SHALL appear in the summary, not be silently omitted.

#### Scenario: Zero disagreements across multiple seeds with real coverage passes the bar

- GIVEN the harness run over at least two distinct seeds with a batch covering every base family and every op kind
- WHEN no trial is classified DISAGREED or ORACLE_UNRELIABLE, each base family and each op kind has at least one AGREED trial, and the mirror handedness flip is confirmed at least once
- THEN the harness SHALL print the per-family / per-op-kind coverage summary, log the honest scope, and exit 0

#### Scenario: Any disagreement fails the bar and is reported with its seed

- GIVEN a batch containing at least one DISAGREED trial
- WHEN the harness prints its summary
- THEN it SHALL exit non-zero, and the DISAGREED trial SHALL be reported with its seed, case index, base family / chain descriptor, and the native / OCCT / analytic measurements, so the silent-wrong-transform is reproducible and not laundered into a pass

#### Scenario: Honest scope and any dropped trials are logged, not silently omitted

- GIVEN a run in which some trials are singular-transform decline-exercisers (BOTH-DECLINED) or the batch is capped for time
- WHEN the harness summarises
- THEN the uniform-scale-only restriction, the planar-vs-curved deflection tolerance rationale, the singular-transform decline, and any capped or skipped trial SHALL be logged explicitly in the summary, so no honest exclusion is hidden

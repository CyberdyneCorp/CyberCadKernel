# native-verification

## ADDED Requirements

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

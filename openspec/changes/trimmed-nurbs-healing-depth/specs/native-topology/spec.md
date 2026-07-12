# native-topology

## ADDED Requirements

### Requirement: N-way / crossing pinch splitting

The trimmed-NURBS module (`src/native/topology/trimmed_nurbs.{h,cpp}`) SHALL provide a GENERAL
N-way / crossing pinch resolver, exposed as `splitAtPinches()` (over a welded loop polyline) and
`splitTrimLoopAtPinches()` (flatten + weld + split a `TrimLoop`), reporting via `MultiSplitReport`
(whether the loop was split into a set of valid simple sub-loops, whether any pinch was detected,
whether it was ambiguous/declined, the number of distinct pinch vertices resolved, the largest
strand-count N at any vertex, the fixpoint iteration count, the scale-relative tolerance applied,
and the resulting sub-loops).

At a pinch vertex where N≥2 strands meet, each INCOMING strand SHALL be paired with an OUTGOING
strand by the **leftmost-turn** rule (the outgoing whose signed CCW turn from the incoming travel
direction is largest) — the orientation-preserving planar-subdivision face-tracing rule — and the
loop's successor links re-routed so tracing them yields SIMPLE sub-loops that share only the pinch
point. Two pinch points that CROSS (a figure-8-of-figure-8) SHALL be resolved by ITERATING the
single-vertex resolution to a FIXPOINT.

The decomposition SHALL be **REGION- and AREA-PRESERVING**: because the re-routing only
re-partitions the SAME directed edges into cycles, the sub-loops' total SIGNED area SHALL equal the
original loop's signed area (verified to ≤1e-12 relative), and the EVEN-ODD region SHALL be
preserved. A pinch whose incoming/outgoing strands do not alternate around the vertex, a sub-loop
that is still self-touching after the fixpoint, a non-convergent fixpoint, or a signed-area
mismatch, SHALL be DECLINED honestly (`ambiguous`), never force-split. The resolver SHALL make no
`shape.h` / `cc_*` change and SHALL keep `src/native` OCCT-free.

`classify()` SHALL expose the resolver as an OPT-IN option `ClassifyOptions::splitNWay` (default
OFF; requires `splitPinch`). With it OFF the classification SHALL be byte-unchanged from the prior
2-way-only behaviour (a 3+-way / crossing pinch still declines `Unknown`). With it ON, when the
clean 2-way split declines, `classify()` SHALL resolve the outer loop via `splitAtPinches()` and
return the EVEN-ODD (parity/XOR) verdict of the resolved simple sub-loops — `In` iff the point is
inside an ODD number of sub-loops, `OnBoundary` on any seam — and SHALL decline `Unknown` for an
`ambiguous` resolution.

#### Scenario: A 3-way pinch splits into three simple loops, area preserved

- GIVEN a synthetic loop that self-touches at ONE vertex where THREE strands meet (three CCW petals
  sharing a centre)
- WHEN `splitTrimLoopAtPinches()` resolves it
- THEN it SHALL report `ok`, `pinch`, not `ambiguous`, exactly THREE sub-loops and `maxWays == 3`
- AND each sub-loop SHALL be simple (≥3 distinct points, no residual self-touch)
- AND the sum of the sub-loops' SIGNED areas SHALL equal the original loop's signed area to ≤1e-12.

#### Scenario: Crossing pinches converge to a fixpoint, area preserved

- GIVEN a loop with TWO distinct pinch vertices that cross (a figure-8-of-figure-8)
- WHEN `splitAtPinches()` resolves it
- THEN it SHALL report `ok` with `iterations > 1` (a fixpoint was needed) and ≥2 pinch vertices
- AND the resolved sub-loops SHALL each be simple and their total SIGNED area SHALL equal the
  original's to ≤1e-12.

#### Scenario: Even-odd containment is preserved by the N-way split

- GIVEN a face whose outer loop is an N-way pinched figure and a set of interior/exterior probes
- WHEN classified with `splitPinch` and `splitNWay` ON
- THEN every probe SHALL classify IDENTICALLY to the reference region (the even-odd union of the
  intended simple sub-loops) — no probe flips.

#### Scenario: A genuinely-ambiguous pinch declines honestly

- GIVEN a pinch whose strands do not alternate cleanly, or a degenerate/non-convergent
  configuration
- WHEN resolved
- THEN `splitAtPinches()` SHALL report `ambiguous` (not `ok`) and `classify()` with `splitNWay` ON
  SHALL return `Unknown` — never a fabricated region.

#### Scenario: Default behaviour is byte-unchanged

- GIVEN any loop
- WHEN classified with `splitNWay` OFF (the default)
- THEN the verdict SHALL be identical to the prior 2-way-only healing behaviour (a 3+-way / crossing
  pinch declines `Unknown`), and a CLEAN loop through `splitAtPinches()` SHALL be a NO-OP (`ok`, one
  sub-loop, no `pinch`, area unchanged — idempotent).

### Requirement: Rational pcurve construction

`constructPcurve()` (numsci-gated) SHALL build a RATIONAL pcurve when the 3-D edge is rational (a
NURBS/Bezier with a parallel weight vector) and `ConstructOptions::rational` is set (default ON),
reporting `PcurveConstruction::rational`. The rational pcurve SHALL inherit the edge's EXACT degree,
knot vector and weights, with its 2-D control net obtained by projecting the edge's control poles to
(u,v) (a homogeneous / Piegl-&-Tiller construction) — so that for a surface whose parametrization is
affine in the curve's (u,v) region (a plane, exactly), the pcurve reproduces the 3-D rational curve
through the surface EXACTLY.

The round-trip fidelity `S(pcurve(t)) == C(t)` SHALL be the acceptance gate: when the rational
pcurve does not meet the fidelity tolerance (the surface parametrization is NOT affine in the region
— e.g. a cylinder's transcendental angle coordinate), `constructPcurve()` SHALL fall back to the
non-rational fit and report its TRUE deviation — never a widened or faked tolerance. A NON-rational
(polynomial) edge SHALL be unaffected by the option and fitted non-rationally (exact for polynomial
edges). The construction SHALL keep `src/native` OCCT-free and make no `shape.h` / `cc_*` change.

#### Scenario: A circular trim edge on a plane has an EXACT rational pcurve

- GIVEN a rational-quadratic full-circle NURBS edge lying on a plane
- WHEN `constructPcurve()` runs with `rational` ON
- THEN it SHALL report `ok` and `rational`, the constructed pcurve SHALL carry weights, and the
  round-trip `S(pcurve(t)) == C(t)` deviation SHALL be ≤1e-9
- AND the SAME edge fitted with `rational` OFF (a polynomial B-spline) SHALL have a measurable SAG
  (deviation ≫ 1e-4, orders of magnitude larger than the rational pcurve).

#### Scenario: A cylinder circular arc honestly declines to non-rational

- GIVEN a rational circular ARC that genuinely lies ON a cylinder (projection residual ~0)
- WHEN `constructPcurve()` runs with `rational` ON
- THEN because the cylinder's angle coordinate is transcendental in the arc parameter, the result
  SHALL be non-rational (honest fall-back) and its reported fidelity deviation SHALL exceed the
  tolerance — the seam / transcendental residual, never a widened tolerance.

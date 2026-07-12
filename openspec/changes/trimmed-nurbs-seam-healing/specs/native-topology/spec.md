# native-topology

## ADDED Requirements

### Requirement: Surface parametric periodicity

The trimmed-NURBS module (`src/native/topology/trimmed_nurbs.{h,cpp}`) SHALL report a
`FaceSurface`'s parametric periodicity via `surfacePeriod()`, returning a `SurfacePeriod`
(whether the u- and v-directions are periodic and, if so, their periods). A Cylinder, Cone, or
Sphere SHALL be reported periodic in u with period 2π (the angular sweep); a Torus SHALL be
reported periodic in BOTH u and v (period 2π each). A Plane, and — in this slice — a free-form
BSpline or Bezier surface, SHALL be reported NON-periodic (a closed free-form surface's seam is an
explicit residual, declined rather than guessed). The function SHALL make no `shape.h` / `cc_*`
change and SHALL keep `src/native` OCCT-free.

#### Scenario: Analytic quadrics are periodic in the angular sweep

- GIVEN a Cylinder, Cone, or Sphere `FaceSurface`
- WHEN `surfacePeriod()` is queried
- THEN it SHALL report `periodicU == true` and `uPeriod == 2π` (and `periodicV == false`), and a
  Torus SHALL report `periodicU` AND `periodicV`.

#### Scenario: A non-periodic surface is a seam-healing no-op

- GIVEN a Plane or a free-form BSpline / Bezier `FaceSurface`
- WHEN `surfacePeriod()` is queried
- THEN it SHALL report `periodicU == false` and `periodicV == false`, so the seam-healing path is a
  strict NO-OP (an honest decline, not a fabricated seam).

### Requirement: Seam-crossing trim-loop healing

The module SHALL provide seam-crossing loop healing, exposed as `loopCrossesSeam()` (detect that a
flattened loop crosses the u-seam of a period-`period` surface), `healSeamLoop()` (unwrap a
flattened loop across the seam), and `healTrimLoopSeam()` (flatten + seam-heal a `TrimLoop` on a
`FaceSurface`), reporting via `SeamHealReport` (whether the loop was healed into one seam loop,
whether it crosses / is tangent to the seam, whether it is ambiguous, whether it is a full wrap, the
net winding, the u-period applied, the unwrapped u-span, the number of seam crossings, the
scale-relative tolerance, and the unwrapped loop polyline).

A trim loop that CROSSES the seam (a consecutive u-jump within tolerance of ±one full period, or a
loop whose unwrapped u-span reaches a full period) SHALL be UNWRAPPED — ∓period is added to u at
each seam jump so u stays continuous — producing ONE closed seam-crossing loop (a wrapped
cross-section circle becomes one loop, not two open arcs). The report SHALL distinguish a **FULL
WRAP** (unwrapped u-span ≥ ~one full period — the whole u-band is enclosed) from a **FINITE
STRADDLING region** (unwrapped u-span < one period — a proper u-arc straddling the seam). The unwrap
SHALL be REGION-PRESERVING: it only ADDS an exact multiple of the period (u and u+period are the
SAME physical point on the periodic surface), so no interior/exterior verdict is flipped, and a loop
that does NOT cross the seam SHALL be echoed BYTE-IDENTICALLY.

A genuinely AMBIGUOUS seam topology SHALL be DECLINED honestly (`ambiguous`, `healed == false`): a
**seam-tangent** loop (a vertex ON the seam line whose two neighbours lie on the SAME side — a touch
without a crossing), a finite straddling region whose unwrapped polyline is NOT a simple loop, or a
closing residual that is not an integer number of periods. A tolerance SHALL NEVER be widened.

#### Scenario: A full cross-section wrap heals into one seam loop

- GIVEN a band on a periodic cylinder bounded by two cross-section circles that crosses the u-seam
- WHEN `healSeamLoop()` unwraps it
- THEN it SHALL report `healed`, `crossesSeam`, `fullWrap`, not `ambiguous`, and an unwrapped u-span
  of approximately the full period 2π.

#### Scenario: A finite region straddling the seam heals into one simple loop

- GIVEN a finite region whose u-arc straddles the seam (crossing u=0 once each way)
- WHEN `healSeamLoop()` unwraps it
- THEN it SHALL report `healed`, `crossesSeam`, not `fullWrap`, not `ambiguous`, net winding 0, and
  the unwrapped polyline SHALL be a simple (non-self-touching) loop.

#### Scenario: A non-crossing loop is echoed byte-identically

- GIVEN a trim loop entirely inside one period (not touching the seam)
- WHEN seam-healed
- THEN `crossesSeam` SHALL be false and the healed polyline SHALL equal the plain flattened loop
  (a strict NO-OP).

#### Scenario: A seam-tangent loop is declined honestly

- GIVEN a loop that TOUCHES the seam line at a vertex without crossing it (both neighbours on the
  same side — a bounce)
- WHEN seam-healed
- THEN `healSeamLoop()` SHALL report `ambiguous` (not `healed`) — never a fabricated full band.

### Requirement: Seam-aware point classification

The module SHALL provide `classifySeam()` — point-in-trimmed-region WITH seam identification.
For a NON-periodic surface, or a periodic surface whose outer loop does not touch the seam,
`classifySeam()` SHALL return a verdict IDENTICAL to `classify()` (a strict NO-OP superset,
byte-for-byte). For a periodic surface whose outer loop CROSSES the seam, `classifySeam()` SHALL
unwrap the loop (`healSeamLoop()`), reduce the query point's u modulo the period into the loop's
unwrapped window, and classify: a **FULL wrap** SHALL classify a point `In` iff its v lies strictly
inside the loop's v-band (the whole u-band is enclosed, for EVERY u), `OnBoundary` within tolerance
of the v-band edges; a **FINITE straddling region** SHALL classify by the ordinary even-odd raycast
in the unwrapped window (enclosing exactly the loop's u-arc). Holes on a periodic surface SHALL be
seam-healed the same way. An AMBIGUOUS outer seam loop SHALL decline `Unknown`.

#### Scenario: A full wrap encloses the whole u-band

- GIVEN a `TrimmedNurbsFace` on a cylinder whose outer loop is a full-wrap band between v0 and v1
- WHEN `classifySeam()` is queried
- THEN a point with v ∈ (v0, v1) SHALL classify `In` for EVERY u, a point with v outside [v0, v1]
  SHALL classify `Out`, and a point on v0 or v1 SHALL classify `OnBoundary`.

#### Scenario: A half wrap encloses exactly its u-arc

- GIVEN a `TrimmedNurbsFace` whose outer loop is a finite region straddling the seam
- WHEN `classifySeam()` is queried
- THEN every interior/exterior probe SHALL classify IDENTICALLY to the reference region computed
  with the seam identification (a point inside the wrapped arc `In`, a point on the far side of the
  cylinder `Out`).

#### Scenario: classifySeam is a strict no-op off the seam

- GIVEN a non-periodic surface, or a periodic surface with a loop wholly inside one period
- WHEN `classifySeam()` and `classify()` are both queried for the same probes
- THEN `classifySeam()` SHALL return the SAME verdict as `classify()` for every probe.

#### Scenario: An ambiguous outer seam loop declines Unknown

- GIVEN a `TrimmedNurbsFace` whose outer loop is seam-tangent (touch without crossing)
- WHEN `classifySeam()` is queried
- THEN it SHALL return `Unknown` — never a fabricated full band.

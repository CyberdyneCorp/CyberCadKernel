# native-topology

## ADDED Requirements

### Requirement: Tolerant-topology healing of trim loops

The trimmed-NURBS module (`src/native/topology/trimmed_nurbs.{h,cpp}`) SHALL provide a bounded,
**region-preserving** tolerant-topology healing pass over a trim loop, exposed as `healLoop()` (over
a flattened loop polyline plus its segment-join gaps) and `healTrimLoop()` (flatten + heal a
`TrimLoop`), reporting via `HealReport` (whether the loop is healed/valid, whether any vertex was
moved, the number and largest magnitude of welded gaps, whether a large gap or a pinch was detected,
the residual gap, and the scale-relative tolerance applied). Every tolerance SHALL be
**scale-relative** (`tol = gapTol · max(loop-UV-extent, 1)`). The healing SHALL:

- **Close small gaps / snap near-coincident pcurves** — two consecutive pcurve endpoints (including
  the closing join) within `tol` but not coincident SHALL be welded to their midpoint, so the loop
  is closed and adjacent pcurves share a single boundary vertex.
- **Decline a genuine large gap** — a segment-join gap exceeding `tol` SHALL make the loop NOT
  healed (`largeGap` reported), never force-welded into a fabricated region.
- **Detect a pinch** — a self-touching loop (a repeated non-adjacent vertex within `tol`) SHALL be
  DETECTED (`pinch` reported) and declined honestly (split is a documented residual, not fabricated).

Healing SHALL be **REGION-PRESERVING**: because a weld moves any vertex by at most `tol/2`, no
interior/exterior point farther than `tol/2` from the boundary SHALL change its `In`/`Out`
classification — a heal SHALL NEVER flip a point's containment verdict versus the intended loop. The
module SHALL make no `shape.h` / `cc_*` / POD change and SHALL keep `src/native` OCCT-free.

#### Scenario: A small injected gap heals to closed and classifies identically

- GIVEN a loop whose consecutive pcurve endpoints are a small distance ε apart (ε below the
  scale-relative gap tolerance) but not coincident
- WHEN the loop is healed and points are classified
- THEN the loop SHALL heal to closed (report `healed`, `gapsClosed ≥ 1`), AND every interior and
  exterior test point SHALL classify identically to the exact (gap-free) loop

#### Scenario: A large gap declines honestly

- GIVEN a loop with a segment-join gap larger than the scale-relative gap tolerance
- WHEN the loop is healed and a point is classified
- THEN the loop SHALL NOT be force-healed (report `largeGap`, `healed=false`), and classification
  SHALL return `Unknown` (an honest decline), never a fabricated verdict

#### Scenario: Healing preserves containment

- GIVEN a loop with a healable injected gap and a set of interior/exterior probe points
- WHEN the gap size is swept across the band up to the tolerance and each probe is classified
- THEN NO probe's `In`/`Out` verdict SHALL ever flip versus the exact loop — a heal SHALL never
  change the region

#### Scenario: Near-coincident pcurves snap; a pinch is detected

- GIVEN (a) two pcurve endpoints within tolerance but not coincident, and (b) a loop that
  self-touches at an interior point
- WHEN each is healed
- THEN (a) the endpoints SHALL be snapped to a shared boundary vertex (welded seam), and (b) the
  pinch SHALL be DETECTED (`pinch` reported) and declined (`Unknown`), not silently repaired into a
  different region

## MODIFIED Requirements

### Requirement: Point-in-trimmed-region classification with honest declines

The module SHALL classify a parameter point `(u,v)` against a `TrimmedNurbsFace` as one of
`In`, `Out`, `OnBoundary`, or `Unknown`, by an even-odd ray-cast in the surface's `(u,v)` plane
over the trim loops. A point strictly inside the outer loop and outside every hole loop SHALL be
`In`; a point outside the outer loop, or inside any hole loop, SHALL be `Out`; a point within a
scale-relative tolerance of any loop edge SHALL be `OnBoundary`. Degenerate configurations — an
empty or open loop (fewer than three distinct flattened points), or a self-touching / pinched loop
(a repeated non-adjacent vertex) — SHALL return `Unknown` (an honest decline) rather than a
fabricated `In`/`Out` verdict, mirroring the kernel's honest-decline discipline. The parity test
SHALL treat shared loop vertices consistently so a ray grazing a vertex (as for a uniformly
sampled axis-aligned rectangle) is classified correctly.

Classification SHALL run the tolerant-topology healing pass (`ClassifyOptions::heal`, default on)
on each loop before the ray-cast: a healable loop (small gaps / near-coincident pcurves) SHALL be
welded closed and classified, while a loop that heals to a large gap, a pinch, or a degeneracy SHALL
still decline `Unknown`. Healing SHALL be region-preserving, so a loop that was already valid (its
segment joins already coincident) SHALL classify unchanged, and no previously-declined broken loop
SHALL be force-accepted. With healing disabled (`heal=false`) the classification SHALL use the
former flatten + well-formedness path unchanged.

#### Scenario: Rectangular sub-region with a circular hole classifies correctly

- GIVEN a rectangular outer loop and a circular hole loop on a bicubic patch
- WHEN points provably inside the rectangle and outside the hole, provably outside the rectangle, and provably inside the hole are classified
- THEN the interior points SHALL be `In`, the outside points SHALL be `Out`, and a point in the hole SHALL be `Out`

#### Scenario: Boundary points classify as OnBoundary

- GIVEN a point on a rectangle edge and a point on the hole circle
- WHEN they are classified
- THEN each SHALL be `OnBoundary` (not silently bucketed as `In` or `Out`)

#### Scenario: Degenerate loops decline honestly

- GIVEN an empty outer loop, an open/degenerate single-point loop, a self-touching (pinched) loop, or a face whose hole loop is degenerate
- WHEN a point is classified
- THEN the result SHALL be `Unknown` (an honest decline), never a fabricated verdict

#### Scenario: A healable loop is classified, a broken loop still declines

- GIVEN a loop with a small injected gap and a loop with a large gap or a pinch
- WHEN each is classified with healing on (the default)
- THEN the small-gap loop SHALL be welded closed and classified identically to the exact loop, while
  the large-gap / pinch loop SHALL still return `Unknown`

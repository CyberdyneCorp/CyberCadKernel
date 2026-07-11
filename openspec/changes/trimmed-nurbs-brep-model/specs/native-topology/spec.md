# native-topology

## ADDED Requirements

### Requirement: Trimmed-NURBS face data model

The native topology library SHALL provide, in an OCCT-free module
(`src/native/topology/trimmed_nurbs.{h,cpp}`, namespace `cybercad::native::topology`), a
`TrimmedNurbsFace` aggregate = a surface (`FaceSurface`, including free-form `Kind::BSpline` /
`Kind::Bezier`) + a `Location` + an **outer trim loop** + zero or more **inner (hole) loops**, each
loop an ordered list of pcurve segments (a `PCurve` in the surface's `(u,v)` parameter plane with a
`[first,last]` parameter range and a `reversed` flag). The module SHALL reuse the existing
`shape.h` storage (`FaceSurface`, `PCurve`, `EdgeCurve`, `TShape::EdgePCurve`) and SHALL make NO
change to `shape.h` and NO change to any existing `FaceSurface` / `PCurve` / `step_reader`
consumer. A `TrimmedNurbsFace` SHALL be buildable from an existing topology face `Shape` (child 0
= outer wire, the rest = holes; each edge's pcurve resolved via `pcurveOf` with the single-pcurve
fallback), returning an honest absence when the shape is not a face, carries no surface, or has no
usable outer loop. The module SHALL make no `cc_*` signature or POD layout change and SHALL keep
`src/native` OCCT-free.

#### Scenario: A face's trimmed region is represented from surface + loops

- GIVEN a surface `S(u,v)` and an outer loop plus hole loops expressed as pcurve segments in `(u,v)`
- WHEN a `TrimmedNurbsFace` is assembled
- THEN it SHALL retain the surface, its placement, the outer loop and every hole loop, and SHALL store enough to answer "is `(u,v)` inside the trimmed region?"

#### Scenario: Building from an existing topology face changes no existing consumer

- GIVEN a topology face `Shape` with a surface and stored pcurves (as produced by the STEP reader)
- WHEN a `TrimmedNurbsFace` is built from it
- THEN the build SHALL reuse the stored pcurves without modifying `shape.h` or the face, AND every existing `FaceSurface` / `PCurve` / `step_reader` consumer SHALL be unchanged

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

### Requirement: Pcurve fidelity guard

The module SHALL verify, for a surface `S`, a 3-D edge curve `C(t)`, and a pcurve `p(t)`, that
`S(p(t))` equals `C(t)` on a dense sample of the shared parameter range within a **scale-relative**
tolerance `tol = absTol + relTol · L` (where `L` is the length scale of the sampled 3-D edge), and
SHALL report the maximum deviation, the mean deviation, the applied tolerance, and the parameter
achieving the worst deviation. A faithful pcurve (the true iso/edge pcurve) SHALL pass; a
deliberately-wrong pcurve that does not lie on `S` SHALL be DETECTED via a large deviation and
SHALL NOT pass. Placements (`Location`) for the edge and surface SHALL be applied so the comparison
is in world coordinates.

#### Scenario: A faithful pcurve satisfies S(p(t)) == C(t)

- GIVEN a surface, its 3-D edge curve equal to a known iso-curve, and the correct pcurve
- WHEN fidelity is checked on a dense sample
- THEN `S(p(t))` SHALL match `C(t)` within the scale-relative tolerance (achieved ~1e-9), and the report SHALL flag success

#### Scenario: A wrong pcurve is detected, never passed

- GIVEN the same surface and edge but a pcurve that drifts off the true edge
- WHEN fidelity is checked
- THEN the reported maximum deviation SHALL be large (flagged), and the guard SHALL report failure — never a fabricated pass

### Requirement: Pcurve construction by projection and fit

The module SHALL construct, for a 3-D edge curve that lies on a surface `S`, a pcurve in `S`'s
`(u,v)` plane by sampling the edge, projecting each sampled point to `(u,v)` via
`numerics::closest_point_on_surface`, and fitting a 2-D **non-rational** B-spline through the
projected feet via `bspline_fit`; it SHALL then round-trip-verify pcurve fidelity
(`S(pcurve(t)) == C(t)`). The fitted pcurve's knots SHALL be reparametrized onto the edge's
`[first,last]` range so `pcurve(t)` is evaluated at the same parameter as `C(t)`. The construction
SHALL decline honestly (report not-ok) when the edge does not lie on `S` (projection residual too
large relative to the edge's world extent), the fit fails, or the round-trip fidelity is not met.
This routine, and only this routine, SHALL be compiled under `CYBERCAD_HAS_NUMSCI` (it depends on
the numerics facade + `bspline_fit`); with the guard off it SHALL be absent while the data model,
classification and fidelity guard remain available.

#### Scenario: Constructed pcurve round-trips a known (u,v) path

- GIVEN a 3-D edge curve built to lie on `S` (S evaluated along a known `(u,v)` path)
- WHEN its pcurve is constructed by projection + fit and round-trip-verified
- THEN the reconstructed pcurve SHALL reproduce the `(u,v)` path and `S(pcurve)` SHALL reproduce the 3-D curve, both within the fit tolerance, and the construction SHALL report success

#### Scenario: An off-surface edge declines honestly

- GIVEN a 3-D edge curve that does NOT lie on `S` (e.g. lifted off the surface)
- WHEN pcurve construction is attempted
- THEN the projection residual SHALL be large and the construction SHALL report not-ok (an honest decline), never a fabricated pcurve

#### Scenario: Non-rational construction reports the true deviation

- GIVEN a rational 3-D edge fitted as a non-rational approximation
- WHEN fidelity is round-trip-verified
- THEN the reported deviation SHALL be the achieved (true) deviation, never a widened or faked tolerance

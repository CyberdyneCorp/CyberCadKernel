# native-exchange

## ADDED Requirements

### Requirement: Native STEP admission of a foreign RATIONAL B-spline CURVE as edge/trim geometry via the combined RATIONAL_B_SPLINE_CURVE record, else DECLINE

The STEP reader SHALL admit ONE foreign **rational** B-spline **curve** — emitted by
OCCT `STEPControl_Writer` as a **combined** Part-21 instance
`( BOUNDED_CURVE() B_SPLINE_CURVE(degree, (#pole…), form, closed, selfInt)
B_SPLINE_CURVE_WITH_KNOTS((mults), (knots), spec) RATIONAL_B_SPLINE_CURVE((weight…))
CURVE() REPRESENTATION_ITEM('') GEOMETRIC_REPRESENTATION_ITEM() )` — used as an
`EDGE_CURVE`'s 3D geometry (directly or through a `SURFACE_CURVE` / `SEAM_CURVE` /
`INTERSECTION_CURVE` unwrap) or as a `TRIMMED_CURVE` basis, as a native
`EdgeCurve` of `Kind::BSpline` carrying curve `weights`, **only when** the combined
record is well-formed AND the per-edge faithful-reconstruction guard verifies the edge,
so the landed native mesh path (`native-tessellation`, whose edge evaluator already
consumes `weights`) — which SHALL NOT be modified — can mesh the solid watertight. This
extends the M4-rational `RATIONAL_B_SPLINE_SURFACE` admission one dimension down to
rational NURBS **curves** by reading the curve weights the reader does not read today.

The reader SHALL dispatch the combined record using the existing combined-instance
machinery it already uses for the rational surface and assembly relationships
(`Record::subs` sub-record scan, `hasSub` / `findSub`) — no new tokenizer. It SHALL read
`degree` and the pole list from the `B_SPLINE_CURVE` sub-record, the RLE-expanded `knots`
(via the existing `expandKnots`) from the `B_SPLINE_CURVE_WITH_KNOTS` sub-record, and the
flat weight list from the `RATIONAL_B_SPLINE_CURVE` sub-record into `EdgeCurve::weights`
in the SAME order as the poles. The shared degree/pole/knot parse SHALL be factored so the
non-rational `B_SPLINE_CURVE_WITH_KNOTS` keyword path produces a byte-identical
`EdgeCurve`.

The reader SHALL `decline()` the curve (NULL → OCCT) when the combined record is
malformed: a required sibling sub-record (`B_SPLINE_CURVE`, `B_SPLINE_CURVE_WITH_KNOTS`,
`RATIONAL_B_SPLINE_CURVE`) is missing or mistyped, the knot vector length does not equal
`poles + degree + 1`, the weight count does not equal the pole count, or any weight is
non-finite or not strictly positive. A weight is NEVER clamped and no tolerance is
introduced by this well-formedness check.

The reader's faithful-guard curve evaluator SHALL be rational-aware: for a `Kind::BSpline`
edge it SHALL evaluate with `math::nurbsCurvePoint` over the curve poles / knots /
`weights` when `weights` is non-empty and with the existing non-rational `math::curvePoint`
otherwise, so the per-edge `pcurve(t) = C_edge(t)` reconstruction guard evaluates a
rational edge correctly. The `TRIMMED_CURVE` sub-domain machinery (the two
`PARAMETER_VALUE` trims selecting the covered knot span) SHALL be reused UNCHANGED for a
rational basis. An admitted rational edge SHALL be subject to the engine's mandatory
watertight + volume/area self-verify against the OCCT oracle downstream; a native result
that is not watertight or off-volume SHALL be DISCARDED → OCCT, so a wrong or leaky solid
is never emitted.

The reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` /
`EngineShape` type), the native tessellator SHALL NOT be modified, `EdgeCurve` and
`PCurve` SHALL be unchanged (`weights` already exists), no tolerance SHALL be weakened, and
the `cc_*` ABI SHALL be unchanged (additive read-side behaviour only). The non-rational
`B_SPLINE_CURVE_WITH_KNOTS` keyword path, the analytic Line / Circle / Ellipse curve arms,
the M4 non-rational surface path, and the M4-rational `RATIONAL_B_SPLINE_SURFACE` surface
path SHALL remain byte-identical. A closed/periodic rational curve needing a seam close
beyond the knot-span clamp, and a rational 2D PCURVE authored in the file, remain out of
scope (unchanged decline / ignored, as the reader synthesises its own analytic pcurves).

#### Scenario: A foreign solid with a rational B-spline edge/trim curve imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a rational B-spline curve as an `EDGE_CURVE`'s 3D geometry or a `TRIMMED_CURVE` basis, expressed as the combined `RATIONAL_B_SPLINE_CURVE` record, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader parses the combined record, populates `EdgeCurve::weights`, recovers the trimmed knot sub-domain, passes the rational-aware per-edge faithful guard, admits the edge, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (a foreign rational NURBS edge that previously declined for lack of a curve-weights read now meshes watertight)

#### Scenario: The parsed rational curve reproduces a closed-form value and the guard accepts it, rejects an off-curve weight (host analytic, no OCCT)

- GIVEN a native-built `Kind::BSpline` `EdgeCurve` whose rational-quadratic weights (`{1, cos(Δ/2), 1}` over the arc's control triangle) make it reproduce an EXACT circular arc of known closed-form geometry, built on the host with NO OCCT linked, plus a deliberately perturbed off-curve weight variant
- WHEN the reader evaluates the curve at a grid of parameters `t` and runs the per-edge faithful-reconstruction guard (rational eval via `math::nurbsCurvePoint`)
- THEN `evalEdge(t)` SHALL equal the closed-form circle point `O + R(cos θ, sin θ)` within `1e-9`, the guard SHALL ACCEPT the faithful edge, AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against an independent closed-form oracle with no OCCT symbol linked

#### Scenario: A malformed or non-positive-weight rational curve record declines to OCCT (host + sim)

- GIVEN a combined rational B-spline curve record whose weight count does not equal the pole count, whose knot vector length does not equal `poles + degree + 1`, that is missing a required sibling sub-record, or that contains a non-finite or non-positive weight
- WHEN the reader parses the `RATIONAL_B_SPLINE_CURVE` sub-record
- THEN the reader SHALL `decline()` the curve (NULL → OCCT), no weight SHALL be clamped, no tolerance SHALL be weakened, the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, and no approximate native edge SHALL be emitted — the honest decline is reported

#### Scenario: A rational edge whose faithful guard fails, or whose native mesh is not watertight, declines to OCCT (sim)

- GIVEN a foreign STEP file whose rational B-spline edge either cannot be reconstructed within the scale-relative faithful-guard tolerance, or whose admitted native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the reader runs the rational-aware per-edge faithful guard, or the engine runs its mandatory watertight + volume/area self-verify
- THEN the reader SHALL `decline()` (or the engine SHALL DISCARD) the native result and the import SHALL fall through to OCCT identical to `cc_set_engine(0)`, so a wrong or leaky native solid is never emitted downstream

#### Scenario: The non-rational and rational-surface paths stay byte-identical (zero-regression)

- GIVEN the existing STEP round-trip suite and the full tessellation-sensitive sim suite, with the rational-curve arm reachable ONLY by a combined `RATIONAL_B_SPLINE_CURVE` record (which declines today at `curve()`'s `r->combined` guard)
- WHEN the suite runs with this change applied
- THEN the non-rational `B_SPLINE_CURVE_WITH_KNOTS` keyword path, the analytic Line / Circle / Ellipse arms, the M4 non-rational surface path, and the M4-rational `RATIONAL_B_SPLINE_SURFACE` surface path SHALL produce byte-identical native results, `src/native/**` SHALL contain 0 OCCT includes, the tessellator SHALL be unmodified, and the `cc_*` ABI SHALL be unchanged — otherwise the rational-curve arm is reverted and the rational curve keeps the OCCT decline

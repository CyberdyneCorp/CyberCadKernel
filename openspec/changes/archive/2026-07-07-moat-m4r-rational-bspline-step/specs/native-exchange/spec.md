# native-exchange

## ADDED Requirements

### Requirement: Native STEP admission of a foreign RATIONAL B-spline surface face via the combined RATIONAL_B_SPLINE_SURFACE record, else DECLINE

The STEP reader SHALL admit ONE foreign **rational** B-spline surface — emitted by
OCCT `STEPControl_Writer` as a **combined** Part-21 instance
`( BOUNDED_SURFACE() B_SPLINE_SURFACE(degU, degV, ((#pole)…), form, uClosed, vClosed,
selfInt) B_SPLINE_SURFACE_WITH_KNOTS((uMults), (vMults), (uKnots), (vKnots), spec)
GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(((weight)…)) REPRESENTATION_ITEM('')
SURFACE() )` — bounded by a real, genuinely trimmed `EDGE_LOOP`, as a native trimmed
`Kind::BSpline` face carrying surface `weights`, **only when** the rational record is
well-formed AND a faithful 2D pcurve is reconstructed and verified for EVERY boundary
edge, so the landed native trimmed-freeform mesh path (`native-tessellation`,
`trimmedFreeformMesh`) — which SHALL NOT be modified — can mesh it watertight. This
extends the M4 non-rational `B_SPLINE_SURFACE_WITH_KNOTS` admission to rational NURBS
surfaces by reading the surface weights the reader does not read today.

The reader SHALL dispatch the combined record using the existing combined-instance
machinery it already uses for rational curves / assembly relationships (`Record::subs`
sub-record scan) — no new tokenizer. It SHALL read `degU, degV, uClosed, vClosed` and the
row-major (U-outer / V-inner) pole grid from the `B_SPLINE_SURFACE` sub-record, the
RLE-expanded `(uMults, vMults, uKnots, vKnots)` knot vectors from the
`B_SPLINE_SURFACE_WITH_KNOTS` sub-record, and the weight grid from the
`RATIONAL_B_SPLINE_SURFACE` sub-record into `FaceSurface::weights` in the SAME row-major
order as the poles. The shared pole/knot parse SHALL be factored so the non-rational
`B_SPLINE_SURFACE_WITH_KNOTS` keyword path produces a byte-identical `FaceSurface`.

The reader SHALL `decline()` the face (NULL → OCCT) when the rational record is malformed:
the weight grid is ragged, its cardinality does not equal the pole-grid cardinality
(`nPolesU × nPolesV`), or any weight is non-finite or not strictly positive. A weight is
NEVER clamped and no tolerance is introduced by this well-formedness check.

An admitted rational face SHALL flow UNMODIFIED through the existing per-edge pcurve arm
and the **faithful-reconstruction guard**, which SHALL be rational-aware: the guard's
surface evaluation SHALL use `math::nurbsSurfacePoint` over the surface poles / knots /
`weights` (the reader's `bsplineSurfaceValue` already routes to the rational evaluator
when `weights` is non-empty), re-evaluate `S_face(pcurve(t)) = C_edge(t)` at several
parameters within the SAME scale-relative tolerance as the non-rational path, and
`decline()` the face if any boundary edge fails. An admitted face SHALL be subject to the
engine's mandatory watertight + volume/area self-verify against the OCCT oracle
downstream; a native result that is not watertight or off-volume SHALL be DISCARDED →
OCCT, so a wrong or leaky solid is never emitted.

The reader SHALL remain OCCT-free (`src/native/**` links no OCCT / `IEngine` /
`EngineShape` type), the native tessellator SHALL NOT be modified, `FaceSurface` SHALL be
unchanged (`weights` already exists), no tolerance SHALL be weakened, and the `cc_*` ABI
SHALL be unchanged (additive read-side behaviour only). The non-rational
`B_SPLINE_SURFACE_WITH_KNOTS` keyword path, the analytic Plane / Cylinder / Cone / Sphere
/ Torus arms, and the bare-periodic full-sphere / full-torus / full-revolution B-spline
admission paths SHALL remain byte-identical. A rational **curve** read remains out of
scope (unchanged decline).

#### Scenario: A foreign rational B-spline surface face imports and meshes watertight matching BRepMesh (sim, parity)

- GIVEN a foreign STEP file carrying a rational B-spline surface face as the combined `RATIONAL_B_SPLINE_SURFACE` record bounded by a genuinely trimmed `EDGE_LOOP` whose boundary pcurves reconstruct faithfully, imported on a booted iOS simulator with OCCT linked and the native engine active (`cc_set_engine(1)`)
- WHEN the reader parses the combined record, populates `FaceSurface::weights`, reconstructs each boundary pcurve, passes the rational-aware `S_face(pcurve(t)) = C_edge(t)` guard, admits the face, and the landed M0 tessellator meshes the resulting solid
- THEN the native solid's enclosed volume, surface area, watertight status, triangle envelope, and sub-shape topology SHALL match the OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` oracle within tolerance (a foreign rational NURBS patch that previously declined for lack of a weights read now meshes watertight)

#### Scenario: The parsed rational surface reproduces a closed-form value and the guard accepts it, rejects an off-surface edge (host analytic, no OCCT)

- GIVEN a native-built trimmed `Kind::BSpline` face whose rational-quadratic weights (`{1, 1/√2, 1, …}`) make it reproduce an EXACT cylindrical/spherical section of known closed-form geometry, built on the host with NO OCCT linked, plus a deliberately perturbed off-surface variant of one boundary edge
- WHEN the reader evaluates the surface at a grid of `(u,v)` and runs the faithful-reconstruction guard (rational eval via `math::nurbsSurfacePoint`)
- THEN `surfaceValue(u,v)` SHALL equal the closed-form surface point within `1e-9`, the guard SHALL ACCEPT the faithful face (every sampled `t` within the scale-relative tolerance and the meshed solid watertight with the closed-form enclosed volume within tolerance), AND SHALL REJECT the perturbed variant (`decline()` fires) — proven against an independent closed-form oracle with no OCCT symbol linked

#### Scenario: A malformed or non-positive-weight rational record declines to OCCT (host + sim)

- GIVEN a rational B-spline surface record whose weight grid is ragged, whose weight count does not equal the pole-grid cardinality, or that contains a non-finite or non-positive weight
- WHEN the reader parses the `RATIONAL_B_SPLINE_SURFACE` sub-record
- THEN the reader SHALL `decline()` the face (NULL → OCCT), no weight SHALL be clamped, no tolerance SHALL be weakened, the file SHALL round-trip through OCCT `STEPControl_Reader` unchanged, and no approximate or leaky native face SHALL be emitted — the honest decline is reported

#### Scenario: A rational face whose boundary pcurve does not reconstruct faithfully declines to OCCT (sim)

- GIVEN a foreign STEP file whose rational B-spline surface face has at least one boundary edge whose pcurve cannot be reconstructed within the scale-relative tolerance (non-converging inversion, off-surface reconstructed pcurve, or beyond-tolerance boundary gap)
- WHEN the reader evaluates the rational-aware faithful-reconstruction guard for that edge
- THEN the reader SHALL `decline()` the face (NULL → OCCT), the file SHALL import via OCCT identical to `cc_set_engine(0)`, and no wrong or leaky native rational face SHALL be emitted

#### Scenario: The engine self-verify discards a non-watertight admitted rational face (sim)

- GIVEN a foreign rational B-spline face that passes the per-edge pcurve guard but whose native mesh does not close watertight (or whose volume/area does not match the OCCT oracle)
- WHEN the engine runs its mandatory watertight + volume/area self-verify
- THEN the native result SHALL be DISCARDED and the import SHALL fall through to OCCT, so a wrong or leaky mesh is never emitted downstream

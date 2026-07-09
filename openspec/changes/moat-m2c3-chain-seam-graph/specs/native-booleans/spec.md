# native-booleans

## ADDED Requirements

### Requirement: Chain seam-GRAPH builder for the freeform↔finite-box three-cutting-face two-junction pose, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only chain seam-graph
builder (`src/native/boolean/seam_graph_chain.h`) that, GIVEN operand `A` (a
`recogniseFreeformSolid`-admitted bowl-lidded convex-quad prism with exactly one freeform
wall) and operand `B` (a FINITE axis-aligned analytic box whose every face is a `Plane`),
assembles the inter-solid intersection as a **three-arc, two-junction seam CHAIN** for the
reachable ≥3-seam pose: one in which EXACTLY THREE faces of `B` slice `A`'s Bézier wall —
two PARALLEL end faces (each an iso-`u` arc) and one ORTHOGONAL middle face (an iso-`v`
arc) — and `B`'s remaining faces contain `A`. The builder SHALL (1) identify the
cutting-face SET via the pole-hull straddle predicate (`isdetail::planeStraddlesWall`)
reused BYTE-IDENTICAL, requiring exactly three cutting faces and every other face
containing `A` (`isdetail::aabbInsidePlane`); (2) trace EACH arc by driving the EXISTING
`hscdetail::traceWallSeam` machinery UNCHANGED and iso-classify it via
`sgdetail::arcIsoParam` UNCHANGED; (3) resolve the chain order — the two ENDS share their
iso-orientation, the MIDDLE is orthogonal, the ends have distinct iso values — into
`{end0, end1, middle}`; (4) compute BOTH analytic junctions `J1 = wall(u(end0), v(mid))`
and `J2 = wall(u(end1), v(mid))` (the points where an end plane and the middle plane meet
the surface), verifying each lies INSIDE the trimmed wall (`tess::pointInPolygon`) and ON
BOTH of its adjacent cutting planes (`|signedDist| < weldTol`); and (5) CLIP each arc at
its junction(s) and JOIN the three sub-arcs into ONE ordered boundary→`J1`→`J2`→boundary
`chainSeam` bent at `J1` and `J2`, which appear as EXACT shared interior vertices (their
world positions carried verbatim, not resampled).

`buildChainSeamGraph(A, B)` SHALL return a typed DECLINE — carrying the measured blocker —
and NO partial graph when: `B` is not all-planar (`NotPlanarB`); the number of cutting
faces is not exactly three (`NotThreeCuttingFaces` — the honest scope boundary of this
slice, e.g. the two-face corner box); a non-cutting face does not contain `A`
(`NotContained`); the three arcs are not two-parallel + one-orthogonal with distinct end
iso values (`NotChainTopology`); an arc is missing / not one well-formed transversal chord
(`SeamUnusable`); an arc is not iso-parametric (`ArcNotIsoParam`); a junction is outside
the trimmed wall (`JunctionUnusable`); a junction is not on both its adjacent planes
(`JunctionOffPlane`); or the clipped arcs do not join coincident at a junction within the
weld tolerance (`JunctionNotJoined`). The builder SHALL remain OCCT-free, SHALL introduce
no `cc_*` ABI surface, SHALL NOT weaken any tolerance to force a junction or a join, SHALL
consume the landed `inter_solid_seam.h` / `seam_graph.h` primitives BYTE-IDENTICAL, and
SHALL keep its per-function cognitive complexity within the backend band via per-arc /
per-junction helpers.

#### Scenario: Three box faces cut the wall and the three arcs join at two junctions into one bent chain (host, no OCCT)

- **WHEN** `A` is the bowl-lidded convex-quad prism and `B` is the axis-aligned box
  `x ∈ [−0.15, 0.15], y ∈ [0.0, 0.8], z ∈ [−0.6, 0.2]` straddling one edge of `A`
- **THEN** `buildChainSeamGraph(A, B)` returns `ChainSeamDecline::Ok` with three distinct
  cutting faces; the three arcs are two PARALLEL iso-`u` ends (canonically ordered by
  ascending iso value) + one ORTHOGONAL iso-`v` middle; both junctions have plane residual
  below `weldTol` (measured ~5e-13) and are distinct; and the `chainSeam` is a bent
  boundary→`J1`→`J2`→boundary polyline in which `J1` and `J2` appear as exact interior
  vertices (world position within 1e-12) with join gap below 0.02·diagonal.

#### Scenario: The middle arc is bounded by both junctions; the ends each by one junction plus the wall boundary (host, no OCCT)

- **WHEN** the chain graph is built for the edge-straddling box
- **THEN** the two junctions share the middle arc's iso-`v` value (they lie on the same
  middle plane), and each junction's iso-`u` equals its own end arc's iso value.

#### Scenario: The strip-clip closed-form volume oracle is self-consistent (host, no OCCT)

- **WHEN** the strip removal `A ∩ {−0.15 ≤ x ≤ 0.15, y ≥ 0}` is integrated in closed form
- **THEN** `V(A∩B)` is a strictly-interior discriminating fraction of `V(A)`
  (`0.05·V(A) < V(A∩B) < 0.6·V(A)`), and the partition identity `V(A∩B) + V(A−B) = V(A)`
  and union identity `V(A∪B) = V(A) + V(B) − V(A∩B)` hold to 1e-12.

#### Scenario: A two-face corner box is not the chain pose and DECLINES (host, no OCCT)

- **WHEN** `B` is the landed two-face corner box (`x ∈ [0, 0.8], y ∈ [0, 0.6]`, only its
  `x = 0` and `y = 0` faces cut the wall)
- **THEN** `buildChainSeamGraph(A, B)` returns `nullopt` with
  `ChainSeamDecline::NotThreeCuttingFaces` — no fabricated third arc, no partial graph.

#### Scenario: OCCT grounds every native arc node and both junctions (sim, native-vs-OCCT)

- **WHEN** the same `A`, `B` are reconstructed in OCCT (bowl `Geom_BezierSurface` face +
  the three cutting planes + the box), and the native chain graph is grounded against them
- **THEN** every native arc node lies on OCCT's bowl surface (`BRepExtrema_DistShapeShape`
  ≤ 1e-4, measured ~1e-15) and on its own cutting plane (≤ 1e-4, measured ~1e-12); each
  junction lies on the bowl surface AND on BOTH its adjacent planes (measured ~1e-13);
  `BRepAlgoAPI_Section(B, bowlFace)` yields a connected section (three edges) with both
  native junctions on it (≤ 1e-4); and the two-face corner box DECLINES
  `NotThreeCuttingFaces` (no fabrication).

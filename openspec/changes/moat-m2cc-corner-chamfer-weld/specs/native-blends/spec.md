# native-blends

## ADDED Requirements

### Requirement: Convex-corner chamfer weld of mutually-adjacent edges, oracle-matched or DECLINE

The native blend library SHALL provide an OCCT-free, header-only convex-corner chamfer
weld verb (`src/native/blend/corner_chamfer_weld.h`, `chamfer_corner`) that, GIVEN an
all-planar `Solid`, a set of 1-based `mapShapes(Edge)` edge ids, and a symmetric setback
`distance`, chamfers the picked CONVEX planar-dihedral edges — INCLUDING a set that
shares a common CORNER, which the byte-frozen sequential `chamfer_edges` DECLINES (its
first cut removes the shared corner, so the next edge is lost from the soup) — into ONE
watertight solid. The verb SHALL resolve EVERY picked edge and its outward chamfer plane
UP FRONT against the ORIGINAL un-clipped polygon soup (where the shared corner still
exists), then apply all clips, so the corner facet is synthesised from the exposed
cross-section rings by the boolean's `chamfer_edges` `detail::applyCut` +
`assembleSolid` T-junction repair with NO extra geometry.

`chamfer_corner` SHALL be a strictly ADDITIVE SIBLING of `chamfer_edges`: it SHALL NOT
modify `chamfer_edges`, `fillet_edges`, `full_round`, `fillet_face`, the M0 tessellator,
or any landed weld path, and it SHALL consume `chamfer_edges`'s `detail::chamferPlane` /
`detail::applyCut` and `blend_geom.h`'s `PlanarModel` / `edgeEnds` / `facesOnEdgeInSoup`
BYTE-IDENTICAL. It SHALL introduce no `cc_*` ABI surface (it is reached through the
existing `cc_chamfer_edges`, tried as an additive candidate after the sequential planar
chamfer and gated by the same shrink self-verify). It SHALL remain OCCT-free and keep
its per-function cognitive complexity within the backend band.

`chamfer_corner` SHALL run a mandatory self-verify — the welded result meshes WATERTIGHT
and its volume is `0 < V < V(original)` (a convex chamfer only REMOVES material) — and
SHALL return a NULL Shape carrying a typed measured DECLINE, and NO solid, when: the
solid or edge list is degenerate or the distance is non-positive (`BadInput`); the solid
carries a curved face (`NonPlanarSolid`); a picked edge id is out of range or not a
straight two-vertex edge (`EdgeNotFound`); an edge is not a convex dihedral between two
planar faces (`NotConvexEdge`); a chamfer plane's clip exposes no usable cross-section
ring (`CutFailed`); fewer than four faces survive (`AssembleFailed`); the welded result
is not watertight (`NotWatertight`); or the volume is non-positive or not below the
original (`VolumeInconsistent`).

The verb SHALL match the OCCT oracle (`BRepFilletAPI_MakeChamfer`) where its geometry
provably equals OCCT's — the DIHEDRAL corner where at most TWO picked edges meet at any
one vertex (a union of setback half-space prisms, EXACT vs OCCT). Where OCCT's chamfer
diverges — a TRIPLE corner at which ≥3 picked edges share ONE vertex, which OCCT breaks
into chamfer-chamfer facets that trim MORE material than the plain half-space
intersection (measured: OCCT 985.667 vs the half-space 985.75 on a 10³ box, d=1) — the
verb SHALL DECLINE with `TripleCornerOracleGap` → OCCT, rather than emit a solid the
oracle disagrees with. No tolerance SHALL be weakened to force a match.

#### Scenario: A 2-adjacent-edge convex corner welds watertight at the exact closed-form volume (host, no OCCT)

- GIVEN a 10×10×10 box and the two convex edges meeting at the corner `(0,0,0)` (each a 90° planar dihedral), built on the host with NO OCCT
- WHEN `chamfer_corner(box, {ex,ey}, distance)` runs
- THEN it SHALL return a watertight solid whose enclosed volume equals `L³ − d²(L − d/3)` (the inclusion-exclusion of the two per-edge corner prisms) to machine precision across a setback sweep, while the byte-frozen `chamfer_edges` DECLINES the SAME adjacent pair to NULL

#### Scenario: A single convex edge and a non-orthogonal corner weld watertight (host, no OCCT)

- GIVEN a 10³ box (a single convex edge, `V_removed = d²L/2`) and an equilateral-triangle prism (a 60° vertical corner edge), built on the host with NO OCCT
- WHEN `chamfer_corner` runs on each
- THEN each SHALL return a watertight solid with material removed (`0 < V < V(original)`), the single-edge case at the exact closed-form volume, proving the weld is not orthogonal-corner-specific

#### Scenario: A triple corner declines to OCCT rather than mismatch the oracle (host + sim)

- GIVEN the three convex edges meeting at ONE box vertex, built on the host with NO OCCT and, on the booted simulator, a native body through `cc_chamfer_edges`
- WHEN `chamfer_corner` (host) / `cc_chamfer_edges` under the native engine (sim) runs on the 3-edge set
- THEN the host verb SHALL return NULL with `TripleCornerOracleGap`, and the native engine SHALL return id 0 (a clean decline — a native body cannot forward to OCCT), because OCCT's `MakeChamfer` trims MORE at the triple corner (985.667) than the half-space corner (985.75) and the weld MUST NOT emit a solid the oracle disagrees with

#### Scenario: native == OCCT MakeChamfer to fp64 on the 2-adjacent corner (sim native-vs-OCCT)

- GIVEN a 10³ box built and chamfered on the two corner edges through `cc_chamfer_edges` once under the OCCT engine (the oracle = `BRepFilletAPI_MakeChamfer`) and once under the native engine, on the booted iOS simulator
- WHEN the two results' volume / area / centroid / bounding box are compared
- THEN they SHALL agree to fp64 (relative volume 0) and the native result SHALL be watertight, with the native engine active (a genuine native intercept, not a forward)

#### Scenario: Out-of-domain inputs decline with a measured reason (host, no OCCT)

- GIVEN a capped cylinder (curved), an out-of-range edge id, a zero setback, an empty edge list, and an oversized setback on a 2-edge corner, built on the host with NO OCCT
- WHEN `chamfer_corner` runs on each
- THEN it SHALL return NULL with the specific measured reason (`NonPlanarSolid`, `EdgeNotFound`, `BadInput`, `BadInput`, and a self-verify decline respectively), emitting NO solid and weakening NO tolerance

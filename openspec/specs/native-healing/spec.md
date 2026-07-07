# native-healing Specification

## Purpose
TBD - created by archiving change add-native-shape-healing. Update Purpose after archive.
## Requirements
### Requirement: Native tolerant sewing of a face soup into a watertight shell

The kernel SHALL provide a native, **OCCT-free** healing module
(`cybercad::native::heal`) that, given a `topology::Shape` (a face soup, or an open
/ malformed shell) and a `HealOptions{ tolerance }`, stitches faces whose boundary
edges are **coincident within `tolerance`** — but not topologically shared — into a
connected shell by merging each such edge pair into ONE shared edge node referenced
by both faces. Two edges SHALL be treated as one shared edge ONLY when (i) their
endpoint vertices have unified to the same two shared vertices AND (ii) their curves
agree along the span (midpoint proximity + curve kind/geometry) within `tolerance`.
An edge with no within-tolerance partner SHALL remain a boundary edge — a candidate
hole — and SHALL NOT be stitched. The module SHALL be OCCT-free (it consumes
`native-topology` and `native-tessellation` only) and SHALL NOT add or change any
`cc_*` signature or POD struct.

#### Scenario: a coincident-within-tolerance face soup sews into a watertight solid
- GIVEN six planar faces of a unit cube, each carrying its OWN copies of the shared
  edges and corners (topologically independent), all coincident within `tolerance`
- WHEN the native healer sews the soup
- THEN it SHALL merge each coincident edge pair into one shared edge and each set of
  coincident corners into one shared vertex, producing a connected shell
- AND the resulting solid SHALL self-verify watertight with enclosed volume 1
- AND `HealResult.status` SHALL be `Healed` with `nMergedEdges == 12`

#### Scenario: a sub-tolerance gap between faces is closed
- GIVEN a face soup whose adjacent faces are pulled apart by a gap strictly less than
  `tolerance`
- WHEN the native healer sews it
- THEN it SHALL unify the near-coincident boundary vertices and stitch the shared
  edges, closing the shell
- AND the result SHALL self-verify watertight with `maxResidualGap ≤ tolerance`

#### Scenario: an edge with no within-tolerance partner is left as a boundary edge
- GIVEN a face soup with one boundary whose nearest counterpart is farther than
  `tolerance`
- WHEN the native healer sews it
- THEN that edge SHALL remain an unstitched boundary edge (no fabricated closure)
- AND the surviving gap SHALL be recorded in `maxResidualGap`

### Requirement: Vertex / tolerance unification merges near-coincident vertices

The module SHALL merge near-coincident boundary vertices (within `tolerance`) onto a
single shared `topology::Vertex` node using a quantized spatial hash at the weld
tolerance (the `native-booleans` `VertexPool` primitive, generalized to arbitrary
B-rep input). The count of unified vertices SHALL be reported in
`HealMetrics.nMergedVerts`. Vertex unification SHALL be a prerequisite for edge
merging: two edges are candidates for sewing only after their endpoints unify.

#### Scenario: scattered near-coincident corners collapse to the true corner set
- GIVEN a face soup whose shared corners appear as multiple copies scattered within
  `tolerance` of each other
- WHEN the native healer unifies vertices
- THEN each cluster within `tolerance` SHALL collapse to ONE shared vertex node
- AND `nMergedVerts` SHALL reflect the reduction to the true corner count
- AND no two vertices farther apart than `tolerance` SHALL be merged

### Requirement: Degenerate-edge and sliver-face removal

The module SHALL drop **zero-length edges** (endpoint separation `< tolerance`, i.e.
both endpoints unified to one vertex) and **sliver / zero-area faces** (parametric or
planar area `< tolerance²`, or a face whose wire collapses to fewer than three
distinct vertices after unification), rebuilding the affected wires and faces without
the removed elements and removing a face that degenerates entirely. The count of
removed degenerate elements SHALL be reported in `HealMetrics.nDroppedDegenerate`.

#### Scenario: a zero-length edge is dropped and its face rebuilt valid
- GIVEN a face whose boundary wire contains a zero-length edge (two coincident
  corners)
- WHEN the native healer removes degenerate elements
- THEN the zero-length edge SHALL be dropped and the face's wire rebuilt without it
- AND `nDroppedDegenerate` SHALL be at least 1
- AND the healed solid SHALL self-verify watertight

#### Scenario: a sliver zero-area face is removed without changing the volume
- GIVEN a face soup with an extra near-zero-area sliver face inserted
- WHEN the native healer removes degenerate elements
- THEN the sliver face SHALL be removed from the shell
- AND the healed solid SHALL self-verify watertight with the correct enclosed volume

### Requirement: Consistent outward orientation via shared-edge flood-fill

The module SHALL make every face normal point consistently outward by building face
adjacency across the shared edges and flood-filling orientation: across each shared
edge the two incident faces SHALL traverse it in opposite directions (the manifold
consistency rule), flipping a neighbour that agrees. The global sign SHALL be
confirmed by the sign of the enclosed volume of the resulting closed mesh — if it is
negative the entire shell SHALL be flipped so the enclosed volume is positive
(outward). The count of re-oriented faces SHALL be reported in
`HealMetrics.nFlipped`.

#### Scenario: an inward-wound face is flipped to match the shell
- GIVEN a face soup that sews watertight but with exactly one face wound inward
- WHEN the native healer fixes orientation
- THEN the flood-fill SHALL re-orient that face consistently with its neighbours
- AND `nFlipped` SHALL be 1
- AND the healed solid's enclosed volume SHALL be positive (outward)

### Requirement: Mandatory self-verify before a heal is reported healed

Before reporting `HealStatus::Healed`, the module SHALL tessellate the candidate
shell/solid and confirm ALL of: (a) `tessellate::isWatertight` (every undirected mesh
edge used by exactly two triangles — closed 2-manifold, no boundary, no T-junction),
(b) `tessellate::enclosedVolume > 0` (consistent outward orientation), and (c) every
recorded merge was within `tolerance`. If any check fails, the module SHALL NOT report
`Healed`; it SHALL return `Unhealed` (reason `SelfVerifyFailed` or `OpenShell`) with
the input shape unchanged. The module SHALL NEVER report `watertight == true` unless
the tessellated mesh actually closed. This is the healing instance of the mandatory
self-verify → OCCT-fallback discipline: the slice SHALL never claim a false closure.

#### Scenario: a candidate that does not tessellate watertight is not reported healed
- GIVEN a candidate shell that fails `isWatertight` after the four sub-operations
- WHEN self-verification runs
- THEN the module SHALL return `Unhealed` with the input shape unchanged
- AND it SHALL NOT report `watertight == true`

#### Scenario: a healed result is confirmed watertight and outward before return
- GIVEN a face soup healed into a candidate solid
- WHEN self-verification runs
- THEN the module SHALL confirm `isWatertight` AND `enclosedVolume > 0` AND all
  merges within `tolerance` before returning
- AND only then SHALL it return `HealStatus::Healed` with `watertight == true` and
  `valid == true`

### Requirement: Honest UNHEALED report for out-of-scope defects, never faked

The module SHALL return a typed **UNHEALED** result (`HealStatus::Unhealed`) with a
reason and the measured residual whenever the defect is out of scope, and SHALL NEVER
fabricate a closure or weaken the tolerance to pass. UNHEALED SHALL be returned for at
least: a gap **beyond `tolerance`** (`GapBeyondTolerance`), a genuinely open shell
that cannot close within tolerance (`OpenShell`), a non-manifold input edge shared by
3+ faces (`NonManifold`), a candidate that fails self-verify (`SelfVerifyFailed`), and
an out-of-scope defect such as a missing pcurve, a self-intersecting wire, or a
freeform re-approximation (`OutOfScope`). On UNHEALED the input shape SHALL be returned
UNCHANGED and `maxResidualGap` SHALL carry the measured largest surviving gap.
`Unhealed` SHALL be a normal outcome (the deferral seam to OCCT `ShapeFix`), not an
error.

#### Scenario: a gap beyond tolerance is reported UNHEALED with the measured residual
- GIVEN a face soup with one face pulled away by several times `tolerance` (a real
  hole beyond tolerance)
- WHEN the native healer runs
- THEN it SHALL return `Unhealed` with reason `GapBeyondTolerance` (or `OpenShell`)
- AND `maxResidualGap` SHALL be approximately the measured gap (well above `tolerance`)
- AND the returned shape SHALL be the input UNCHANGED (no fabricated closure)

#### Scenario: the tolerance is never weakened to force a pass
- GIVEN a defect that would only close if `tolerance` were increased
- WHEN the native healer runs at the caller's `tolerance`
- THEN it SHALL NOT auto-relax `tolerance`
- AND it SHALL return `Unhealed` with the measured residual rather than a heal

### Requirement: Healing is internal, engine-hooked with OCCT fallback, and asymptotic

Healing SHALL be an **internal** native capability invoked by the engine (e.g. after a
future native import, or to repair a native result) and SHALL NOT be exposed through
the `cc_*` C ABI: no `cc_*` entry point, signature, or POD struct SHALL be added or
changed, and the tessellator SHALL NOT be modified. The engine SHALL wire a native
heal hook with the try-native → self-verify → **OCCT fallback** ladder: keep the
native result only when it self-verifies `Healed`, otherwise fall through to OCCT
`BRepBuilderAPI_Sewing` + `ShapeFix_Shell` / `ShapeFix_Solid` (OCCT confined to
`src/engine/occt/`, never included by `src/native/**`). Completeness SHALL be framed
as **asymptotic** (per `NATIVE-REWRITE.md` / `MOAT-ROADMAP.md`): the slice heals the
coincident-within-tolerance / degenerate / orientation family — **plus the opt-in,
bounded near-miss band** `(tolerance, min(gapBridgeBudget, 0.25 · localEdgeLen)]`
defined by the bounded gap-bridging requirement, **plus the opt-in synthesis of ONE cap
face for a single simple planar hole** (`HealOptions.capPlanarHoles == true`) defined by
the bounded single planar-hole capping requirement — and reports **measured wins vs
OCCT, not a guarantee** to heal arbitrary broken industrial B-rep; pcurve reconstruction
and self-intersecting-wire repair are out of scope, beyond-tolerance repair is out of
scope EXCEPT that opt-in bounded near-miss band (a gap `>` the budget or `>` the
local-feature cap still defers), and a genuinely open shell / missing face is out of
scope EXCEPT the opt-in single simple planar hole (two or more missing faces, a
non-planar hole, or a self-intersecting boundary still defer). The capability SHALL be
verified at the healing-function level by two gates: a **host** gate (no OCCT —
deliberately-broken fixtures healed watertight + valid with the expected
merge/drop/flip/**bridge**/**cap** counts, and the un-healable fixture reporting UNHEALED
with a measured residual — including the capped-cube fixture reaching the analytic
enclosed volume `1.0`) and a **sim native-vs-OCCT** gate (`BRepBuilderAPI_Sewing` /
`BRepBuilderAPI_MakeFace` / `ShapeFix_Shell` / `ShapeFix_Solid` parity: same
watertight/closed shell, same valid solid, same volume within tol — including the
in-band bridged fixture matching OCCT `BRepBuilderAPI_Sewing` at sewing tolerance
`≈ gapBridgeBudget`, and the single-planar-hole capped fixture matching an OCCT reference
cap; un-healable / beyond-budget / out-of-scope-hole fixture UNHEALED matching OCCT
leaving it open).

#### Scenario: native heal matches OCCT sewing / ShapeFix on a broken fixture
- GIVEN a deliberately-broken face soup built both natively and as an OCCT
  `TopoDS_Compound` of faces
- WHEN the native healer and OCCT `BRepBuilderAPI_Sewing` + `ShapeFix_Shell` /
  `ShapeFix_Solid` each heal it on the simulator
- THEN the native result SHALL match OCCT in watertight/closed shell, valid solid,
  and volume within tolerance, compared at the `cybercad::native::heal` C++ boundary
- AND no `cc_*` entry point SHALL have been called or added

#### Scenario: an in-band bridged heal matches OCCT sewing at the budget tolerance
- GIVEN a near-miss face soup whose seam gap `g` satisfies `tolerance < g ≤
  gapBridgeBudget` and `g < 0.25 · edgeLength`, built both natively and for OCCT
- WHEN the native healer runs with that `gapBridgeBudget` and OCCT
  `BRepBuilderAPI_Sewing` runs at sewing tolerance `≈ gapBridgeBudget`
- THEN both SHALL produce a watertight closed shell with matching enclosed volume within
  tolerance at the `cybercad::native::heal` C++ boundary
- AND the native result SHALL be `Healed` with `nBridgedGaps > 0` and no `cc_*` call

#### Scenario: a single-planar-hole cap matches an OCCT reference cap
- GIVEN a unit cube missing one planar face, built both natively and as an OCCT open
  shell, with an OCCT reference cap built by `BRepBuilderAPI_MakeFace(gp_Pln,
  freeBoundaryWire)` + `ShapeFix_Solid`
- WHEN the native healer runs with `capPlanarHoles == true`
- THEN both SHALL produce a watertight closed 6-face solid with matching enclosed volume
  within tolerance at the `cybercad::native::heal` C++ boundary
- AND the native result SHALL be `Healed` with `nCappedFaces == 1` and no `cc_*` call

#### Scenario: the un-healable fixture defers, and native UNHEALED matches OCCT
- GIVEN the un-healable fixture (a gap beyond `tolerance`, or beyond `gapBridgeBudget`
  when bridging is enabled, or a two-hole / ≥ 2-missing-face open shell that OCCT sewing
  cannot close either when capping is enabled) built both natively and for OCCT
- WHEN the native healer and the matching OCCT operation each run at the same tolerance
- THEN the native healer SHALL return `Unhealed` AND OCCT SHALL also leave the shell
  open / needing more at that tolerance
- AND the engine SHALL fall through to OCCT `ShapeFix` for that shape

#### Scenario: host gate runs without OCCT
- GIVEN the native healing module built for the OCCT-free host
- WHEN the host test suite runs the deliberately-broken fixtures, the in-band bridging
  fixture, the single-planar-hole cap fixture, and the un-healable / beyond-budget /
  out-of-scope-hole fixtures
- THEN each in-scope fixture SHALL heal watertight + valid with the expected metrics
  (including the capped cube reaching enclosed volume `1.0`), and each un-healable /
  out-of-scope fixture SHALL return `Unhealed` with a measured residual — with no OCCT
  linked

### Requirement: Bounded, opt-in beyond-tolerance gap bridging

The healer SHALL provide an **opt-in, budget-bounded** pass that closes a boundary
**near-miss** — a shared seam whose corners were left unpaired by the primary weld
because their separation is greater than `tolerance` — WITHOUT weakening the primary
weld tolerance and WITHOUT fabricating a closure. The pass SHALL be controlled by
`HealOptions.gapBridgeBudget` (model units): when `gapBridgeBudget == 0.0` the pass
SHALL be a **no-op** and `healShell` SHALL behave identically to the landed slice
(the primary weld is never widened by default); when `gapBridgeBudget > 0.0` it is the
maximum gap, **beyond `tolerance`**, that the healer MAY bridge.

A pair of unpaired boundary corners SHALL be bridged (welded onto one shared vertex
node) ONLY when ALL of the following hold — a bridge failing any condition SHALL NOT be
applied:

1. **Effective bound.** Their separation `g` satisfies `tolerance < g ≤
   min(gapBridgeBudget, kLocalFeatureFraction · localEdgeLen)`, where `localEdgeLen` is
   the shortest edge incident to the corners and `kLocalFeatureFraction = 0.25` is a
   fixed documented constant. This **local-feature-size cap** SHALL hold regardless of
   how large `gapBridgeBudget` is set, so a gap comparable to a real edge is NEVER
   bridged.
2. **Mutual nearest.** Each corner is the OTHER's nearest eligible unpaired corner (a
   symmetric, deterministic pairing); no corner appears in two bridges.
3. **Cross-face.** The two corners lie on DIFFERENT faces (bridging a seam between
   faces, never folding one face onto itself).
4. **Manifold-safe.** Merging them introduces NO edge referenced by 3+ faces.

After bridging, the candidate SHALL pass the UNCHANGED mandatory self-verify
(`isWatertight` AND `enclosedVolume > 0` across the deflection ladder) before it is
reported `Healed`; a bridged candidate that fails self-verify SHALL be `Unhealed`
(`SelfVerifyFailed`) with the input unchanged. The tolerance SHALL NEVER be auto-relaxed
beyond `gapBridgeBudget`, and a surviving gap past the effective bound SHALL be reported,
not closed. Completeness beyond this bounded near-miss band SHALL be framed as
**asymptotic** (per `MOAT-ROADMAP.md` M5): arbitrary beyond-tolerance repair (a real
hole / missing face, a gap at geometry scale, a non-seam defect) remains out of scope
and defers to OCCT `ShapeFix`.

#### Scenario: an in-band near-miss seam is bridged into a watertight solid

- GIVEN a cube face soup with one shared seam pulled apart by a gap `g` with
  `tolerance < g ≤ gapBridgeBudget` and `g < 0.25 · edgeLength`
- WHEN `healShell` runs with `gapBridgeBudget > 0.0`
- THEN the pass SHALL weld the mutual-nearest cross-face corner pair onto a shared
  vertex, close the seam, and the candidate SHALL self-verify watertight with the
  correct positive enclosed volume
- AND `HealResult.status` SHALL be `Healed` with `nBridgedGaps > 0` and
  `maxBridgedGap ≈ g`

#### Scenario: a gap beyond the budget is declined honestly

- GIVEN the same soup with the seam gap `g > gapBridgeBudget`
- WHEN `healShell` runs with that `gapBridgeBudget`
- THEN no bridge SHALL be applied and the healer SHALL return `Unhealed` with reason
  `GapBeyondBudget`, `maxResidualGap ≈ g`, and the input shape UNCHANGED

#### Scenario: default budget leaves the landed slice unchanged (no-op)

- GIVEN the in-band soup whose gap `g > tolerance`
- WHEN `healShell` runs with `gapBridgeBudget == 0.0`
- THEN the bridging pass SHALL NOT run, the primary weld tolerance SHALL NOT be widened,
  and the healer SHALL return `Unhealed` exactly as the landed slice does

#### Scenario: the local-feature cap refuses a bridge the budget alone would allow

- GIVEN a soup and a `gapBridgeBudget` large enough that a candidate bridge's gap
  exceeds `0.25 · localEdgeLen` (it would collapse a real edge)
- WHEN `healShell` runs
- THEN that bridge SHALL be refused (the cap, not the caller's budget, governs) and the
  heal SHALL decline rather than weld distinct geometry

### Requirement: Bridging metrics and the GapBeyondBudget honest-out

The healer SHALL report the bounded bridging pass as measured data on `HealMetrics` and
SHALL distinguish a beyond-budget residual from an in-tolerance open shell. `HealMetrics`
SHALL carry `nBridgedGaps` (the count of unpaired boundary corners closed by the bridging
pass) and `maxBridgedGap` (the largest gap actually bridged, honestly `≤` the effective
bound). `UnhealedReason` SHALL provide `GapBeyondBudget`, returned when
`gapBridgeBudget > 0.0` and a boundary gap survives past the budget / local-feature cap,
distinct from `OpenShell` (an in-`tolerance` open shell) and `GapBeyondTolerance` (a gap
beyond `tolerance` with bridging disabled). On any `Unhealed` outcome `maxResidualGap`
SHALL carry the largest surviving gap and the input shape SHALL be returned UNCHANGED.
These additions SHALL be internal C++ result-type fields / enum values only — no `cc_*`
entry point, signature, or POD SHALL be added or changed, and the tessellator SHALL NOT
be modified.

#### Scenario: a bridged heal reports the count and the largest bridged gap

- GIVEN an in-band near-miss soup healed with `gapBridgeBudget > 0.0`
- WHEN the heal succeeds
- THEN `nBridgedGaps` SHALL equal the number of corner pairs the pass welded AND
  `maxBridgedGap` SHALL be the largest bridged gap AND `maxBridgedGap ≤
  min(gapBridgeBudget, 0.25 · localEdgeLen)`

#### Scenario: a beyond-budget decline is typed GapBeyondBudget with the measured residual

- GIVEN a soup whose surviving boundary gap exceeds the budget after the bridging pass
- WHEN `healShell` runs with `gapBridgeBudget > 0.0`
- THEN the healer SHALL return `Unhealed` with reason `GapBeyondBudget` and
  `maxResidualGap` equal to the largest surviving gap, and SHALL NOT report
  `watertight == true`

### Requirement: Bounded, opt-in single planar-hole capping

The healer SHALL provide an **opt-in** pass that closes a **simple `OpenShell`** — a
shell that sews cleanly (every corner paired within `tolerance`) but is missing exactly
one face, leaving a single ring of boundary edges — by synthesizing ONE cap face,
WITHOUT weakening `tolerance` and WITHOUT fabricating a closure. The pass SHALL be
controlled by `HealOptions.capPlanarHoles`: when `capPlanarHoles == false` (the default)
the pass SHALL be a **no-op** and `healShell` SHALL behave identically to the landed
sew / unify / orientation + gap-bridging slices (a missing face SHALL still return
`Unhealed` with reason `OpenShell`); when `capPlanarHoles == true` a single simple
planar hole MAY be capped.

A cap face SHALL be synthesized ONLY when ALL of the following hold — a hole failing any
condition SHALL NOT be capped and SHALL be returned `Unhealed{OpenShell}` with the input
UNCHANGED:

1. **Single simple boundary cycle.** The surviving boundary edges (each referenced by
   exactly one face) SHALL form EXACTLY ONE closed cycle in which every boundary vertex
   has exactly two incident boundary edges. A branching boundary, or TWO OR MORE disjoint
   boundary loops (two or more missing faces), SHALL be declined.
2. **Planarity within tolerance.** Every corner of the boundary loop SHALL lie within
   `tolerance` of the loop's best-fit plane (Newell normal + centroid). A non-planar hole
   SHALL be declined.
3. **Simple polygon.** The boundary loop, projected onto its best-fit plane, SHALL be a
   non-self-intersecting polygon. A self-intersecting boundary SHALL be declined.

The cap SHALL be built from the boundary loop's EXISTING shared vertex / edge nodes (the
same nodes the sew produced), so the two faces meeting on each capped side place
identical boundary points and the result welds watertight. After capping, the candidate
SHALL be re-sewn and re-oriented by the UNCHANGED pipeline and SHALL pass the UNCHANGED
mandatory self-verify (`isWatertight` AND `enclosedVolume > 0` across the deflection
ladder) before it is reported `Healed`; a capped candidate that fails self-verify SHALL
be `Unhealed` (`SelfVerifyFailed`) with the input unchanged. The tolerance SHALL NEVER be
weakened to force a cap, and a hole outside the bound SHALL be reported, not filled.
Completeness beyond a single simple planar hole SHALL be framed as **asymptotic** (per
`MOAT-ROADMAP.md` M5): two or more missing faces, a non-planar / curved hole, a
self-intersecting boundary, pcurve reconstruction, and self-intersecting-wire repair
remain out of scope and defer to OCCT `ShapeFix`.

The healer SHALL report the cap pass as measured data on `HealMetrics`: `nCappedFaces`
(the count of cap faces synthesized — `≤ 1` in this slice) and `maxCapPlanarityDev` (the
largest coplanarity deviation of a capped loop, honestly `≤ tolerance`). These additions
SHALL be internal C++ result-type fields / options only — no `cc_*` entry point,
signature, or POD SHALL be added or changed, no new `UnhealedReason` value SHALL be
added, and the tessellator SHALL NOT be modified.

#### Scenario: a single missing planar face is capped into a watertight solid

- GIVEN a unit-cube face soup with its `+Z` face removed (a single planar square hole),
  built natively, all remaining corners coincident within `tolerance`
- WHEN `healShell` runs with `capPlanarHoles == true`
- THEN the pass SHALL synthesize ONE planar cap face on the boundary loop's shared nodes,
  close the shell, and the candidate SHALL self-verify watertight with positive enclosed
  volume
- AND `HealResult.status` SHALL be `Healed` with `nCappedFaces == 1`,
  `maxCapPlanarityDev ≤ tolerance`, and enclosed volume `= 1.0` (the analytic value, no
  OCCT linked)

#### Scenario: default-off leaves a missing face declined (no-op)

- GIVEN the same missing-`+Z` cube soup
- WHEN `healShell` runs with `capPlanarHoles == false` (the default)
- THEN the cap pass SHALL NOT run and the healer SHALL return `Unhealed` with reason
  `OpenShell` and the input shape UNCHANGED, exactly as the landed slices do

#### Scenario: two missing faces (two boundary loops) are declined honestly

- GIVEN a unit-cube face soup with two OPPOSITE faces removed (two disjoint planar
  boundary loops)
- WHEN `healShell` runs with `capPlanarHoles == true`
- THEN no cap SHALL be applied (this slice caps exactly one simple hole) and the healer
  SHALL return `Unhealed` with reason `OpenShell`, `nCappedFaces == 0`, and the input
  shape UNCHANGED

#### Scenario: a non-planar hole is declined by the planarity layer

- GIVEN a unit-cube face soup with its `+Z` face removed and one top boundary corner
  lifted out of the z=1 plane (its two incident faces are the axis-aligned +X and +Y
  planes, which stay planar and keep that corner paired within `tolerance`, so the single
  top boundary loop is a simple cycle with `maxResidualGap == 0` but is NON-PLANAR)
- WHEN `healShell` runs with `capPlanarHoles == true`
- THEN the planarity layer SHALL refuse the cap (a corner lies farther than `tolerance`
  from the loop's best-fit plane) and the healer SHALL return `Unhealed` with reason
  `OpenShell` and the input shape UNCHANGED
- NOTE: removing two ADJACENT faces instead orphans their shared corner, which declines
  EARLIER as `GapBeyondTolerance` (a beyond-tolerance gap); the lifted-corner fixture
  isolates the planarity layer as the decisive refusal.

#### Scenario: a native cap matches an OCCT reference cap on the simulator

- GIVEN the missing-`+Z` cube built both natively and as an OCCT open shell, and an OCCT
  reference cap constructed by `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)` added
  to the shell + `ShapeFix_Shell` / `ShapeFix_Solid`
- WHEN the native healer runs with `capPlanarHoles == true` and the OCCT reference cap is
  built on the simulator
- THEN both SHALL produce a watertight closed 6-face solid with matching enclosed volume
  within tolerance, compared at the `cybercad::native::heal` C++ boundary
- AND no `cc_*` entry point SHALL have been called or added

#### Scenario: a two-hole open shell defers, and native UNHEALED matches OCCT

- GIVEN a unit cube missing TWO opposite faces (two disjoint boundary loops) built both
  natively and as an OCCT open shell
- WHEN the native healer runs with `capPlanarHoles == true` and OCCT
  `BRepBuilderAPI_Sewing` + `ShapeFix` runs at the same tolerance
- THEN the native healer SHALL return `Unhealed{OpenShell}` with `nCappedFaces == 0` (this
  slice caps exactly one simple hole) AND OCCT SHALL also leave the shell open, since
  sewing / `ShapeFix` never synthesizes an absent face (parity of decline)
- AND the engine SHALL fall through to OCCT `ShapeFix` for that shape
- NOTE: for a single MILDLY-NON-PLANAR hole the parity is asymmetric and not asserted on
  the simulator — OCCT `BRepBuilderAPI_MakeFace(gp_Pln, wire)` tolerates a near-planar wire
  (it keeps the wire's 3D vertices) and caps it, so the native planarity-layer decline
  there is native being MORE conservative and DEFERRING to OCCT, not a shared decline. The
  native planarity decline itself is covered by the host gate below.


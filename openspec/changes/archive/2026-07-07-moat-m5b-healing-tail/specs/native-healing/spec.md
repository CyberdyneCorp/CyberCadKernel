# native-healing

## ADDED Requirements

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

## MODIFIED Requirements

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

# native-healing

## ADDED Requirements

### Requirement: Bounded, opt-in multi-hole planar capping

The healer SHALL provide an **opt-in** pass that closes a shell that sews cleanly
(every corner paired within `tolerance`) but is missing **two or more** faces —
leaving **two or more DISJOINT** rings of boundary edges — by synthesizing ONE
planar cap face per hole, WITHOUT weakening `tolerance` and WITHOUT fabricating a
closure. The pass SHALL be controlled by `HealOptions.capMultiplePlanarHoles`: when
`capMultiplePlanarHoles == false` (the default) the pass SHALL be a **no-op** and
`healShell` SHALL behave identically to the landed sew / unify / orientation +
gap-bridging + single planar-hole slices (a shell missing two or more faces SHALL
still return `Unhealed` with reason `OpenShell`, exactly as the landed
single-hole-cap slice does); when `capMultiplePlanarHoles == true` a set of two or
more disjoint simple planar holes MAY be capped. The flag is a **superset** of the
single planar-hole cap (it also caps a single simple planar hole) and SHALL NOT
alter the behavior of `HealOptions.capPlanarHoles`.

Cap faces SHALL be synthesized ONLY when ALL of the following hold — a hole set
failing any condition SHALL NOT be capped (no partial closure) and SHALL be returned
`Unhealed{OpenShell}` with the input UNCHANGED:

1. **All boundary loops are disjoint simple cycles.** The surviving boundary edges
   (each referenced by exactly one face) SHALL partition into disjoint closed cycles
   in which EVERY boundary vertex has exactly two incident boundary edges. A
   **branching** boundary — the degree-4 corner shared by two ADJACENT missing faces
   — or a non-closing boundary SHALL decline the WHOLE set.
2. **Planarity within tolerance, per loop.** EVERY corner of EACH boundary loop SHALL
   lie within `tolerance` of THAT loop's best-fit plane (Newell normal + centroid). If
   ANY loop is non-planar the WHOLE set SHALL be declined.
3. **Simple polygon, per loop.** EACH boundary loop, projected onto its own best-fit
   plane, SHALL be a non-self-intersecting polygon. If ANY loop self-intersects the
   WHOLE set SHALL be declined.

Each cap SHALL be built from its boundary loop's EXISTING shared vertex / edge nodes
(the same nodes the sew produced), so the two faces meeting on each capped side place
identical boundary points and the result welds watertight. After capping, the
candidate SHALL be re-sewn and re-oriented by the UNCHANGED pipeline and SHALL pass
the UNCHANGED mandatory self-verify (`isWatertight` AND `enclosedVolume > 0` across
the deflection ladder) before it is reported `Healed`; a capped candidate that fails
self-verify SHALL be `Unhealed` (`SelfVerifyFailed`) with the input unchanged. The
tolerance SHALL NEVER be weakened to force a cap, and a hole set outside the bound
SHALL be reported, not filled. The healer SHALL NEVER cap only a SUBSET of the holes
and emit a shell still carrying boundary edges.

The healer SHALL report the pass as measured data on `HealMetrics`: `nCappedFaces`
(the count of cap faces synthesized — `≥ 2` for a multi-hole heal, and equal to the
number of holes) and `maxCapPlanarityDev` (the largest coplanarity deviation across
all accepted loops, honestly `≤ tolerance`). These additions SHALL be internal C++
result-type fields / options only — no `cc_*` entry point, signature, or POD SHALL be
added or changed, NO new `UnhealedReason` value SHALL be added (a declined set stays
`OpenShell`), and the tessellator SHALL NOT be modified.

Completeness beyond a set of disjoint simple **planar** holes SHALL be framed as
**asymptotic** (per `MOAT-ROADMAP.md` M5 tail): a NON-PLANAR / curved hole, a
SELF-INTERSECTING boundary, a BRANCHING boundary (two adjacent missing faces), pcurve
reconstruction, and self-intersecting-wire repair remain out of scope and defer to
OCCT `ShapeFix`.

#### Scenario: two opposite missing planar faces are capped into a watertight solid

- GIVEN a unit-cube face soup with its `+Z` AND `−Z` faces removed (two DISJOINT
  planar square holes at `z = 1` and `z = 0`), built natively, all remaining corners
  coincident within `tolerance`
- WHEN `healShell` runs with `capMultiplePlanarHoles == true`
- THEN the pass SHALL synthesize ONE planar cap face per hole on the two boundary
  loops' shared nodes, close the shell, and the candidate SHALL self-verify watertight
  with positive enclosed volume
- AND `HealResult.status` SHALL be `Healed` with `nCappedFaces == 2`,
  `maxCapPlanarityDev ≤ tolerance`, `maxResidualGap == 0`, and enclosed volume `= 1.0`
  (the analytic value, no OCCT linked)

#### Scenario: default-off leaves two missing faces declined (no-op, byte-identical)

- GIVEN the same missing-`±Z` cube soup
- WHEN `healShell` runs with `capMultiplePlanarHoles == false` (the default)
- THEN the multi-hole pass SHALL NOT run and the healer SHALL return `Unhealed` with
  reason `OpenShell`, `nCappedFaces == 0`, and the input shape UNCHANGED — exactly as
  the landed single-hole-cap slice does for two holes

#### Scenario: two ADJACENT missing faces (branching boundary) are declined honestly

- GIVEN a unit-cube face soup with two ADJACENT faces removed, so their shared corner
  is incident to FOUR boundary edges (a branching boundary, not two disjoint cycles)
- WHEN `healShell` runs with `capMultiplePlanarHoles == true`
- THEN no cap SHALL be applied (the boundary is not a set of disjoint simple cycles)
  and the healer SHALL return `Unhealed` with reason `OpenShell`, `nCappedFaces == 0`,
  and the input shape UNCHANGED

#### Scenario: a two-hole set with one non-planar loop is declined as a whole

- GIVEN a face soup missing two disjoint faces where ONE boundary loop has a corner
  lifted farther than `tolerance` from that loop's best-fit plane (non-planar) while
  the other loop is planar
- WHEN `healShell` runs with `capMultiplePlanarHoles == true`
- THEN the planarity layer SHALL refuse the WHOLE set (no partial closure) and the
  healer SHALL return `Unhealed` with reason `OpenShell`, `nCappedFaces == 0`, and the
  input shape UNCHANGED

#### Scenario: a native multi-hole cap matches an OCCT per-hole reference cap on the simulator

- GIVEN the missing-`±Z` cube built both natively and as an OCCT open shell, and an
  OCCT reference constructed by ONE `BRepBuilderAPI_MakeFace(gp_Pln, freeBoundaryWire)`
  per hole added to the shell + `ShapeFix_Shell` / `ShapeFix_Solid`
- WHEN the native healer runs with `capMultiplePlanarHoles == true` and the OCCT
  reference is built on the simulator
- THEN both SHALL produce a watertight closed 6-face solid with matching enclosed
  volume within tolerance, compared at the `cybercad::native::heal` C++ boundary
- AND no `cc_*` entry point SHALL have been called or added

#### Scenario: a branching two-adjacent-hole open shell defers to OCCT

- GIVEN a unit cube missing TWO ADJACENT faces (a branching boundary) built both
  natively and as an OCCT open shell
- WHEN the native healer runs with `capMultiplePlanarHoles == true` and OCCT
  `BRepBuilderAPI_Sewing` + `ShapeFix` runs at the same tolerance
- THEN the native healer SHALL return `Unhealed{OpenShell}` with `nCappedFaces == 0`
  AND the engine SHALL fall through to OCCT `ShapeFix` for that shape

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
the bounded single planar-hole capping requirement, **plus the opt-in synthesis of one
planar cap per hole for a set of two or more DISJOINT simple planar holes**
(`HealOptions.capMultiplePlanarHoles == true`) defined by the bounded multi-hole planar
capping requirement — and reports **measured wins vs OCCT, not a guarantee** to heal
arbitrary broken industrial B-rep; pcurve reconstruction and self-intersecting-wire
repair are out of scope, beyond-tolerance repair is out of scope EXCEPT that opt-in
bounded near-miss band (a gap `>` the budget or `>` the local-feature cap still
defers), and a genuinely open shell / missing face is out of scope EXCEPT the opt-in
single simple planar hole and the opt-in set of two or more disjoint simple planar
holes (a NON-PLANAR / curved hole, a SELF-INTERSECTING boundary, or a BRANCHING
boundary of two adjacent missing faces still defer). The capability SHALL be verified
at the healing-function level by two gates: a **host** gate (no OCCT — deliberately-
broken fixtures healed watertight + valid with the expected merge/drop/flip/**bridge**/
**cap** counts, and the un-healable fixtures reporting UNHEALED with a measured residual
— including the single-cap cube AND the two-opposite-hole cube reaching the analytic
enclosed volume `1.0` with `nCappedFaces == 2`) and a **sim native-vs-OCCT** gate
(`BRepBuilderAPI_Sewing` / `BRepBuilderAPI_MakeFace` / `ShapeFix_Shell` /
`ShapeFix_Solid` parity: same watertight/closed shell, same valid solid, same volume
within tol — including the single-planar-hole cap AND the two-opposite-hole multi-cap
matching an OCCT per-hole reference cap; un-healable / beyond-budget / branching /
non-planar-hole fixtures UNHEALED matching OCCT leaving them open / deferring).

#### Scenario: native heal matches OCCT sewing / ShapeFix on a broken fixture

- GIVEN a deliberately-broken face soup built both natively and as an OCCT
  `TopoDS_Compound` of faces
- WHEN the native healer and OCCT `BRepBuilderAPI_Sewing` + `ShapeFix_Shell` /
  `ShapeFix_Solid` each heal it on the simulator
- THEN the native result SHALL match OCCT in watertight/closed shell, valid solid,
  and volume within tolerance, compared at the `cybercad::native::heal` C++ boundary
- AND no `cc_*` entry point SHALL have been called or added

#### Scenario: a two-opposite-hole multi-cap matches an OCCT per-hole reference cap

- GIVEN a unit cube missing its two Z faces, built both natively and as an OCCT open
  shell, with an OCCT reference cap built by ONE `BRepBuilderAPI_MakeFace(gp_Pln,
  freeBoundaryWire)` per hole + `ShapeFix_Solid`
- WHEN the native healer runs with `capMultiplePlanarHoles == true`
- THEN both SHALL produce a watertight closed 6-face solid with matching enclosed
  volume within tolerance at the `cybercad::native::heal` C++ boundary
- AND the native result SHALL be `Healed` with `nCappedFaces == 2` and no `cc_*` call

#### Scenario: host gate runs without OCCT

- GIVEN the native healing module built for the OCCT-free host
- WHEN the host test suite runs the deliberately-broken fixtures, the in-band bridging
  fixture, the single-planar-hole cap fixture, the two-opposite-hole multi-cap fixture,
  and the un-healable / beyond-budget / branching / non-planar-hole fixtures
- THEN each in-scope fixture SHALL heal watertight + valid with the expected metrics
  (including the two-opposite-hole cube reaching enclosed volume `1.0` with
  `nCappedFaces == 2`), and each un-healable / out-of-scope fixture SHALL return
  `Unhealed` with a measured residual — with no OCCT linked

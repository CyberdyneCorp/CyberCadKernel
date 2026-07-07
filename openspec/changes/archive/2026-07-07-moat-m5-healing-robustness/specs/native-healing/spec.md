# native-healing

## ADDED Requirements

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
defined by the bounded gap-bridging requirement — and reports **measured wins vs OCCT,
not a guarantee** to heal arbitrary broken industrial B-rep; pcurve reconstruction and
self-intersecting-wire repair are out of scope, and beyond-tolerance repair is out of
scope EXCEPT that opt-in bounded near-miss band (a gap `>` the budget or `>` the
local-feature cap still defers). The capability SHALL be verified at the
healing-function level by two gates: a **host** gate (no OCCT — deliberately-broken
fixtures healed watertight + valid with the expected merge/drop/flip/**bridge** counts,
and the un-healable fixture reporting UNHEALED with a measured residual) and a **sim
native-vs-OCCT** gate (`BRepBuilderAPI_Sewing` / `ShapeFix_Shell` / `ShapeFix_Solid`
parity: same watertight/closed shell, same valid solid, same volume within tol —
including the in-band bridged fixture matching OCCT `BRepBuilderAPI_Sewing` at sewing
tolerance `≈ gapBridgeBudget`; un-healable / beyond-budget fixture UNHEALED matching
OCCT leaving it open).

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

#### Scenario: the un-healable fixture defers, and native UNHEALED matches OCCT
- GIVEN the un-healable fixture (a gap beyond `tolerance`, or beyond `gapBridgeBudget`
  when bridging is enabled) built both natively and for OCCT
- WHEN the native healer and OCCT sewing each run at the same tolerance
- THEN the native healer SHALL return `Unhealed` AND OCCT sewing SHALL also leave the
  shell open / needing more at that tolerance
- AND the engine SHALL fall through to OCCT `ShapeFix` for that shape

#### Scenario: host gate runs without OCCT
- GIVEN the native healing module built for the OCCT-free host
- WHEN the host test suite runs the deliberately-broken fixtures, the in-band bridging
  fixture, and the un-healable / beyond-budget fixture
- THEN each in-scope fixture SHALL heal watertight + valid with the expected metrics,
  and the un-healable / beyond-budget fixture SHALL return `Unhealed` with a measured
  residual — with no OCCT linked

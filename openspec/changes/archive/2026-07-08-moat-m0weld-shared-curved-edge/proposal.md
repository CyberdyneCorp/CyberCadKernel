# Proposal — moat-m0weld-shared-curved-edge (M0 weld robustness)

## Why

The first native freeform boolean (`freeformHalfSpaceCut` / the COMMON) meshes the
bowl‑lidded convex‑quad operand and produces a solid that welds **watertight only at
ALIGNED deflections** (≈ 0.008–0.01). A deflection sweep `{0.03, 0.02, 0.01, 0.008,
0.004, 0.002}` shows the result **oscillate** watertight ↔ NotWatertight in a
declining pattern instead of staying watertight as it refines. The current
`test_native_first_freeform_boolean.cpp` only exercises the one lucky deflection
`0.01`, so the flakiness is invisible to it (`meshVolume(cut, 0.01, wt)`).

Root cause (pinned by M2‑breadth and verified in source): the two‑stage mesher welds
a **shared CURVED edge** by giving both incident faces the SAME per‑edge sample set —
but the sharing is keyed by **edge `TShape` identity** (`EdgeCache::discretize`, keyed
`edge.tshape().get()`). A boolean seam (the degree‑2 Bézier boolean trace) and the
**bowl‑lid quad edges** are represented as **SEPARATE edge nodes per incident face**:
the curved bowl top and its planar side wall each carry their OWN node over the SAME
parabolic edge (a bowl edge over a straight UV segment is `(x,y)` linear × `z`
quadratic → a vertical‑plane PARABOLA — genuinely curved). Two separate nodes hash to
two different cache slots, so each face discretizes that curved edge **independently**
on its own grid. The canonical‑anchor safety net (`recordEdgeAnchors` → `d.points`)
records the discretization's 3‑D points, but the `BoundaryAnchors` store is **local to
one `FaceMesher::mesh()` call** (`face_mesher.h` L220), so face A snaps to A's points
and face B to B's points — different sets. Cross‑face agreement then falls back to the
final **spatial weld** (`VertexWelder`, `tol = 0.5·deflection`), which fuses the two
independent samplings only when `0.5·deflection` happens to bridge their mismatch —
hence watertight *only at aligned deflections*, oscillating as deflection changes.

This is exactly the mechanism M2‑breadth flagged, and it is the M0 weld‑robustness
keystone: the freeform boolean, freeform blends, and wrap/emboss all emit the same
kind of shared **curved** seam and inherit the same coincidence‑dependent weld.

For **straight** shared edges this gap is already solved: `EdgeCache` shares the
segment COUNT by an order‑independent, quantized **endpoint key** (`segsByEndpoints_`
+ `requireMinSegs`) and `recordEdgeAnchors` reconstructs bit‑identical samples from
canonical endpoints (`canonicalLineEndpoints`), so two separate straight nodes weld
exactly. The fix generalises that SAME endpoint‑keyed single‑sampling to **curved**
edges.

## What Changes

1. **Endpoint‑keyed canonical CURVED‑edge discretization** in `src/native/tessellate/
   edge_mesher.h`. `EdgeCache` gains a second cache — keyed by the edge's
   order‑independent, quantized **endpoint pair PLUS a curve‑identity discriminator**
   (a quantized sampled midpoint, so two *different* arcs between the same endpoints
   never merge) — that stores ONE canonical `EdgeDiscretization` (fractions + the 3‑D
   polyline `C_edge(t)`) for a curved edge. `discretize(edge)` returns that canonical
   record for BOTH separate nodes of a genuinely‑curved shared edge, so both incident
   faces read the **identical** fraction list AND the identical canonical 3‑D points.
   The existing `TShape`‑identity fast path is unchanged: an edge shared through ONE
   node (every primitive — a cylinder cap↔side share the circle node) still hits the
   `TShape` cache first and is byte‑identical.

2. **Both incident faces consume the ONE canonical polyline as anchors** in
   `src/native/tessellate/face_mesher.h`. `recordEdgeAnchors` already snaps a face's
   curved‑seam vertices to `d.points`; because (1) now hands BOTH faces the SAME
   canonical `d.points` for a separate‑node curved edge, each face independently snaps
   its seam vertices to the identical canonical polyline. The two faces therefore place
   BIT‑IDENTICAL boundary points at every sample and the spatial weld fuses them at ANY
   deflection — no reliance on `0.5·deflection` bridging a mismatch. No cross‑face
   anchor state is added; the fix lives in the shared discretization, so the per‑face
   anchor path stays as‑is.

3. **Gate discipline (additive, reachable only for the flaky case).** The canonical
   curved path is taken ONLY when an edge is (a) genuinely CURVED in 3‑D
   (`edgeCurvature > 0`, i.e. not the straight path already covered by
   `canonicalLineEndpoints`) AND (b) NOT already shared through a single `TShape` node.
   Every existing surface kind and every existing mesh — where curved edges are either
   `TShape`‑shared (primitives) or straight (planar trims, loft side walls) — takes the
   unchanged path and meshes **byte‑identically**.

4. **Honest‑out preserved.** The engine's mandatory watertight + volume self‑verify
   against the OCCT oracle stays the final arbiter; a non‑watertight native mesh is
   never emitted and no tolerance is weakened. `src/native/**` stays OCCT‑free and the
   `cc_*` ABI is unchanged (internal mesher behaviour only). If a clean additive path
   that keeps every existing mesh byte‑identical AND welds the curved seam at every
   deflection cannot be achieved, the change is REVERTED and the honest OCCT decline is
   kept — a first‑class outcome, documented with the specific tessellator obstacle.

## Capabilities

### Modified Capabilities

- `native-tessellation`: ADDS shared‑curved‑edge **single‑sampling** — a genuinely
  curved edge shared by two faces through SEPARATE nodes (a freeform boolean seam, the
  bowl‑lid quad edges) gets ONE canonical per‑edge discretization that BOTH incident
  faces consume, so the seam welds watertight at ANY deflection, proven byte‑identical
  for every existing surface kind and mesh path, with the honest decline retained when
  single‑sampling cannot be added additively.

## Impact

- `src/native/tessellate/edge_mesher.h` — `EdgeCache` gains an endpoint+curve‑keyed
  canonical curved‑discretization cache alongside the existing `TShape` cache and the
  existing straight‑edge `segsByEndpoints_` count sharing; `discretize` consults it
  only for a genuinely‑curved, non‑`TShape`‑shared edge. Curve eval, sizing
  (`edgeSegments`), and the straight path are untouched.
- `src/native/tessellate/face_mesher.h` — no new state; `recordEdgeAnchors` already
  consumes `d.points`, which is now the shared canonical polyline. (A distance‑guard /
  gate helper may be added; the three existing mesh arms are untouched.)
- **Zero‑regression discipline (mandatory).** The new path is reachable ONLY by a
  curved separate‑node shared edge that today welds coincidentally; every existing mesh
  MUST stay byte‑identical (triangle counts, watertight status, enclosed volume) — the
  full tessellation‑sensitive suite (`run-sim-suite`, STEP import, curved‑fillet,
  curved‑chamfer, curved‑boolean, wrap‑emboss, loft, phase3) and a per‑surface‑kind
  snapshot diffed against `main`. If ANY differs, the change is reverted.
- **Two oracle gates.** (a) HOST ANALYTIC: the freeform CUT and COMMON weld watertight
  across the full sweep `{0.03,0.02,0.01,0.008,0.004,0.002}` at the closed‑form volume
  (no OCCT). (b) SIM native‑vs‑OCCT: the freeform CUT parity (`BRepAlgoAPI_Cut`) still
  passes AND now at multiple deflections.
- **Out of scope (declines, documented not faked):** shared curved edges whose two
  faces carry geometrically DIFFERENT seam curves beyond weld tolerance (a healing
  problem, M5); self‑intersecting seams; any non‑additive tessellator restructuring.
  No `cc_*` ABI change; no CyberCad app change; no OCCT linked into `src/native/**`.

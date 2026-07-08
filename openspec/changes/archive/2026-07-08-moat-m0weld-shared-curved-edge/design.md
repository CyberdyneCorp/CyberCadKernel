# Design — moat-m0weld-shared-curved-edge (M0 weld robustness)

Make a shared **curved** edge weld watertight at **any** deflection by giving each
such edge ONE canonical per‑edge discretization that BOTH incident faces consume —
the direct generalisation of the existing straight‑edge endpoint‑keyed single‑sampling
(`segsByEndpoints_` / `canonicalLineEndpoints` / `recordEdgeAnchors`). Strictly
additive, PROVEN zero‑regression, OCCT‑free. If a clean additive path is not reachable,
keep the honest OCCT decline and document the specific obstacle. This is the identified
critical M0 weld fix; it is attempted seriously.

OCCT (`BRepMesh` volume/area/watertight and `BRepAlgoAPI_Cut`) is the **oracle only**.
The closed‑form bowl‑operand volume (`first_freeform_boolean_fixture.h`) is the no‑OCCT
analytic gate.

## 0-bis. Implementation reality (MEASURED — corrects the mechanism below)

The sections below were authored from the hypothesis that the two incident faces
receive DIVERGENT `d.points` (separate `TShape` nodes → different cache slots →
different curved samplings). **Direct measurement on the fixture refined this**, and the
LANDED fix follows the measurement, not the original hypothesis:

- For the bowl‑lid quad edges, both incident faces' `d.points` already agree to machine
  ε (`max |S_face(pcurve) − C_edge| ≈ 1e‑16`) and their segment counts match (endpoint
  sharing via `segsByEndpoints_` already equalises them). The operand leak was **not**
  divergent `d.points`. It was a **straight‑edge canonical anchor** bug: `recordEdgeAnchors`
  placed its FAR endpoint at the interpolated `ce.a + dir·1`, which rounds ~1 ULP off the
  true endpoint `ce.b` (e.g. `−0.5 + 0.585 ≠ 0.085`). Where a curved seam terminates
  against straight walls, that rounded anchor competes (same weld/hash slot, first‑add‑wins,
  wire‑order‑dependent) with the exact vertex the curved edge and the vertex itself place —
  so a shared corner gets ~1‑ULP‑apart copies on the two faces, which split when the corner
  lands on a weld‑cell boundary (z=0.085 at tol 0.01). **Fix (a):** pin the straight‑arm
  ENDPOINT anchors to the exact `ce.a`/`ce.b` (interior samples keep `ce.a + dir·(i/n)`).
- For the **freeform BOOLEAN seam** (the M1 cut trace) the two faces (the planar cap and
  the trimmed freeform sub‑face) DO carry SEPARATE edge nodes over the same 3‑D curve, and
  their independent discretizations diverge ~1 ULP (different eval paths). **This** is the
  genuine shared‑curved‑edge‑via‑separate‑nodes case §1 targets. **Fix (b):** the canonical
  curved‑edge single‑sampling below — with one correction: the midpoint discriminator is
  **distance‑matched inside an endpoint‑keyed bucket**, NOT a quantized 9‑tuple key. A
  quantized midpoint has the SAME cell‑boundary fragility as the weld (a midpoint at
  0.0453125 straddles the 1e‑6 quantum and splits the two representations of ONE arc into
  different keys). Bucketing by the (robust) endpoint key and matching the midpoint by
  real distance (`kCurveMidTol = 1e‑6`) is boundary‑free: one arc's two reps agree to ~1 ULP
  and merge; two different arcs differ macroscopically and never merge.

Both fixes are strictly additive and PROVEN byte‑identical for every existing mesh (only
the previously‑leaking freeform operand deflections change — to become watertight).

## 0. What the mesher already does (verified in source)

Two cooperating mechanisms make a shared edge weld (`solid_mesher.h` header):

1. **Shared per‑edge 1‑D discretization.** `EdgeCache::discretize(edge)` builds ONE
   deflection‑sized fraction list per edge, **keyed by `edge.tshape().get()`** (the
   edge's `TShape` node). Both faces that share that node read the same fractions and,
   via `S_face(pcurve(f)) = C_edge(f)`, place the same 3‑D point per fraction.
2. **Spatial weld.** `VertexWelder` (`tol = max(0.5·deflection, 1e‑7)`) merges
   near‑coincident vertices on a hash grid — a safety net, not the primary contract.

Two refinements already close the STRAIGHT separate‑node case:

- **Count sharing by endpoint.** `requireMinSegs(edge, segs)` records a minimum segment
  count keyed by an order‑independent, quantized **endpoint key** (`segsByEndpoints_`,
  `quant()` at `1e‑6`), so two SEPARATE straight nodes with the same endpoints
  discretize to the SAME uniform samples (`build()` reads `segsByEndpoints_`).
- **Canonical straight anchors.** `recordEdgeAnchors` → `canonicalLineEndpoints`
  reconstructs the samples at canonical indices `i/n` between lexicographically ordered
  endpoints read from the SHARED bounding vertices, so two opposite‑order straight nodes
  yield bit‑identical anchor SETS; `BoundaryAnchors::find` snaps the face's boundary
  vertices onto them.

**Facts this design leans on (checked in source):**

- `EdgeDiscretization` (`edge_mesher.h` L71) already carries both `fracs` and the
  canonical 3‑D polyline `points` (`C_edge(t(f))`, world‑placed) — the curved analogue
  of canonical endpoints already EXISTS; it is just not shared across separate nodes.
- `recordEdgeAnchors` (`face_mesher.h` L353) already has a **curved arm**: for a
  non‑line edge it adds every `d.points[i]` as an anchor and later snaps the face's
  seam vertices to them (`face_mesher.h` L365‑366). The missing piece is that the two
  separate nodes hand the two faces DIFFERENT `d.points`.
- `BoundaryAnchors anchors` is **local to `mesh()`** (`face_mesher.h` L220): anchors are
  never shared between faces, so cross‑face agreement must come from the *shared
  discretization returning identical points*, not from a cross‑face anchor store.
- The oscillation is therefore explained entirely by: separate `TShape` nodes → two
  cache slots → two independent curved samplings → weld only when `0.5·deflection`
  bridges them. Confirmed by the fixture: the bowl‑lid quad edge is a parabola shared
  by the curved Bézier top and a planar side wall as separate nodes.

## 1. The single change: canonical curved‑edge sharing (`edge_mesher.h`)

Add, alongside `cache_` (`TShape`→discretization) and `segsByEndpoints_`
(endpoints→count), a third store:

```
std::unordered_map<CurveKey, EdgeDiscretization, CurveKeyHash> curvedByEndpoints_;
```

`CurveKey` = the existing order‑independent quantized endpoint pair **plus a
curve‑identity discriminator**: the quantized 3‑D **midpoint** `C_edge((first+last)/2)`
(and, defensively, the quantized 3‑D arc‑chord ratio or a second interior sample). The
midpoint discriminator guarantees two DIFFERENT arcs sharing the same endpoints (e.g. a
minor vs major arc, or two blend seams) never collapse to one key — the additive‑safety
invariant.

`discretize(edge)` decision order (each step falls through only if the prior misses):

1. **`TShape` hit** → return the existing per‑node record. *(Unchanged: primitives and
   any genuinely edge‑shared curved seam keep today's exact behaviour.)*
2. Build the record once (`build(edge)` as today). If the edge is **straight in 3‑D**
   (`edgeCurvature ≈ 0` across the range — the same test `edgeSegments` uses) → store in
   `cache_` and return. *(Unchanged: the straight separate‑node path stays on
   `segsByEndpoints_` + `canonicalLineEndpoints`.)*
3. Edge is **genuinely curved** and reached here via a **separate node** → compute
   `CurveKey`. If `curvedByEndpoints_` already holds a record for that key, **return the
   canonical one** (both faces now share fracs AND `points`); else insert this record as
   the canonical one. Also store in `cache_` under the `TShape` so a re‑query of the
   SAME node is O(1) and returns the canonical record.

Because the canonical record's `points` is the ONE `C_edge(t)` polyline, both faces'
`recordEdgeAnchors` add the identical anchors, and each face's seam vertices
(`S_face(pcurve(f))`, faithful within evaluation round‑off ≪ `kSnapEps = 1e‑6`) snap
onto them — identical boundary points on both faces → exact weld at ANY deflection.

**Why this is additive.** Step 3 is reachable only by a curved edge that (i) is NOT
`TShape`‑shared and (ii) is NOT straight — precisely the freeform‑boolean seam / bowl‑lid
quad case that today welds coincidentally. Every existing mesh's curved edges are either
`TShape`‑shared (primitives) or straight; both keep their current code path. The
midpoint discriminator prevents any accidental merge. No existing signature changes; the
new store is empty and inert for every current fixture.

**Determinism.** Within one build the canonical record is the first inserted for a key;
both faces see it regardless of face order. Across builds the choice is deterministic for
the same input. Watertightness needs only that BOTH faces use the SAME record, which the
key guarantees.

## 2. Face side (`face_mesher.h`) — no new state

`recordEdgeAnchors`'s curved arm already consumes `d.points`. With §1 handing both faces
the SAME `d.points`, the face code is unchanged in behaviour. The only possible edit is a
small **gate/guard helper** (e.g. reuse the curvature test) if needed to keep the
dispatch obvious; the three mesh arms (`structuredGrid`, `earClipMesh`,
`trimmedFreeformMesh`) and `flattenWireShared`/`appendEdgeSamplesAtFracs` are untouched.

## 3. Complexity budget

`discretize` gains one branch (curved + non‑`TShape` → curved cache); the driver stays a
short dispatch (backend band ≤ 15, target ≤ ~8). `CurveKey`/hash mirror the existing
`EndpointKey`/`EndpointKeyHash`. The irreducible geometry (midpoint discriminator) is one
helper with a comment, like `canonicalLineEndpoints`.

## 4. Gate A — host analytic (no OCCT)

Extend the freeform‑boolean host test to a **deflection sweep**. For each `d ∈ {0.03,
0.02, 0.01, 0.008, 0.004, 0.002}`:

- Mesh the bowl operand (full) → **watertight** at every `d`; enclosed volume within the
  `d`‑band of the closed‑form `∫∫_Q (H0 + a(x²+y²))`.
- `freeformHalfSpaceCut` (KeepSide::Below) and the COMMON → **watertight** at every `d`;
  enclosed volume within the `d`‑band of the closed‑form CUT/COMMON value.
- Assert monotone‑ish convergence (no watertight↔open oscillation) across the sweep.

A pre‑change baseline run of this sweep records the current oscillation (the regression
witness); the post‑change run must be watertight at all six deflections.

## 5. Gate B — sim native‑vs‑OCCT (booted simulator, OCCT linked)

The freeform CUT parity test (`BRepAlgoAPI_Cut` oracle) still passes, now asserted at
**multiple deflections** (at least `0.01` and one finer, e.g. `0.004`): native volume /
area / watertight / triangle envelope match OCCT within tolerance at each — OR the reader
declines and OCCT handles it (both PASS). The engine's watertight + volume self‑verify
still DISCARDS any non‑watertight native result → OCCT.

## 6. Zero‑regression proof (mandatory — the tessellator is touched)

- Per‑surface‑kind snapshot (triangle count, watertight flag, enclosed volume) for
  `Plane, Cylinder, Cone, Sphere, Torus`, bare‑periodic `BSpline`, `Bezier`, planar
  trim, loft side wall — **byte‑identical** vs `main`.
- Full tessellation‑sensitive suite unchanged: `run-sim-suite`, STEP import,
  curved‑fillet, curved‑chamfer, curved‑boolean (native‑pass count), wrap‑emboss, loft,
  phase3. Any diff ⇒ revert (see §7).

## 7. Honest‑out (first‑class outcome)

If no additive path keeps every existing mesh byte‑identical AND welds the curved seam at
every deflection — e.g. the two incident faces carry geometrically DIFFERENT seam curves
(midpoint keys diverge beyond weld tol, a healing problem), or the boolean does not expose
recoverable per‑edge endpoints — REVERT `edge_mesher.h`/`face_mesher.h`, keep the OCCT
decline for the freeform boolean, and DOCUMENT the specific obstacle + the measured sweep.
No fabrication, no dead code, no weakened tolerance.

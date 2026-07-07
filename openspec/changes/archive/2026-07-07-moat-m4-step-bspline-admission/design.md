# Design — moat-m4-step-bspline-admission (MOAT M4, first slice)

Admit ONE foreign trimmed `B_SPLINE_SURFACE_WITH_KNOTS` face from a STEP file into a
native `Kind::BSpline` face and mesh it **watertight** through the **landed M0
tessellator**, verified native-vs-OCCT on the simulator — or keep the **honest
decline** with a measured gap. This is a **reader-only** change
(`src/native/exchange/step_reader.cpp`); the tessellator is consumed as shipped and
NOT edited. Clean-room from Piegl & Tiller (NURBS eval/inversion already native in
`math/bspline.h`); OCCT (`STEPControl_Reader` + `BRepMesh_IncrementalMesh` + the
foreign-rational-B-spline STEP fixture) is the **oracle + fixture-author + fallback
only**, linked ONLY in `src/engine/occt`.

## 0. The substrate (verified in source, this worktree)

- `advancedFace(id)` (step_reader.cpp ~1676): resolves the `ADVANCED_FACE` surface via
  `surface()` → `bsplineSurface()` for `B_SPLINE_SURFACE_WITH_KNOTS` (→ `Kind::BSpline`,
  ~923/1340). A childless bound (VERTEX_LOOP / fully-seamed) routes to the bare-periodic
  full-sphere/torus/`isFullRevolutionBSpline` path (untouched). A **real trimmed
  `EDGE_LOOP`** on a `Kind::BSpline` surface falls through to
  `buildFaceWithPCurves(*srf, outer, holes, orient)` (~1746).
- `buildFaceWithPCurves` (~1752): builds the face node, then re-attaches a pcurve per
  edge via `pcurveFor(srf, e, uRef)` and rebuilds the wires. This is where the guard
  hooks in.
- `pcurveFor(srf, edge, uRef)` (~1813): analytic pcurve per (surface, curve) kind. It
  has explicit arms for Plane (Line / Circle / Ellipse / B-spline-on-plane) and
  Cylinder/Cone/Sphere (Circle rim, straight meridian). For a `Kind::BSpline` surface it
  reaches the **"Generic (should not reach here …)"** linear fall-through (~1912-1916):
  `projectUV(e0)`, `projectUV(e1)`, join by a UV Line. `projectUV` for `Kind::BSpline`
  dispatches to `projectBSplineUV` (~1951, grid-seed + damped-Newton surface inversion
  on the analytic derivatives — no OCCT, no NUMSCI). **Correct only when the edge is
  straight in `(u,v)`; blind for a curved edge; unguarded either way.**
- `surfaceValue(s,u,v)` (~2020): the guard's forward evaluator. Has Plane / Cylinder /
  Cone / Sphere / Torus cases; `default` (incl. `Kind::BSpline`) returns `frame.origin`
  — so the guard cannot evaluate a B-spline face today. `revolutionValue` (~1213)
  already shows the rational eval: `math::nurbsSurfacePoint(degreeU, degreeV, grid,
  weights, knotsU, knotsV, u, v)` over `s.poles/nPolesU/nPolesV`.
- `topo::PCurve` (topology/shape.h:197): `{ kind, origin2d, dir2d, degree, knots,
  poles2d }`. `trim.h::pcurveValue(c, t, frac)` (~72) already evaluates `K::BSpline`
  from `poles2d`+`knots`+`degree` — so a curved UV B-spline pcurve is meshed with NO
  tessellator change.
- **M0 mesher (landed, consumed unchanged):** `face_mesher.h::trimmedFreeformMesh` (~498)
  flattens the boundary loops, folds a curvature-driven interior grid via
  `ConstrainedDelaunay`/`triangulateConstrained`, refines to the deflection bound, and
  evaluates every vertex on the true surface. Dispatched for a curved genuinely-trimmed
  freeform face. A faithfully-admitted trimmed `Kind::BSpline` face flows straight into
  it.

## 1. The `pcurveFor` B-spline-surface arm (the reconstruction)

Insert a `Kind::BSpline` surface arm in `pcurveFor` BEFORE the generic fall-through.
Sample the 3D edge `C_edge(t)` at `first` and `last`, invert to `(u,v)` via
`projectBSplineUV`. Decide straight-vs-curved by measuring the midpoint deviation from
the straight UV chord (in the surface's `(u,v)` domain, normalized by the domain span):

- **Straight-in-`(u,v)`** (deviation ≤ a small UV-relative eps): emit a UV `Line`
  `origin2d = (u0,v0)`, `dir2d = ((u1-u0)/len,(v1-v0)/len)` — reproducing today's
  generic arm exactly (zero-regression for existing straight B-spline walls).
- **Curved**: densify N ≥ `degree·k + 1` samples `t_i` across `[first,last]`, invert
  each 3D sample `C_edge(t_i)` to `(u_i,v_i)` via `projectBSplineUV`, and build a UV
  `B_SPLINE` pcurve carrying the **3D edge curve's own `degree` and `knots`** with
  `poles2d = {(u_i,v_i,0)}` (the projected image is the pcurve — the exact inverse of
  the writer's plane B-spline-on-plane arm, generalised to a curved patch). The landed
  `trim.h::pcurveValue` case `K::BSpline` evaluates it; the mesher flattens it.

A hole (`FACE_BOUND` inner loop) uses the same arm — its edges reconstruct and are
guarded identically; the M0 mesher already omits triangles inside a hole.

## 2. The faithful-reconstruction guard (`S_face(pcurve(t)) = C_edge(t)`)

Add a `Kind::BSpline` case to `surfaceValue` (rational-aware, mirroring
`revolutionValue`): `math::nurbsSurfacePoint(s.degreeU, s.degreeV, grid, weights,
knotsU, knotsV, u, v)`. Then, in `buildFaceWithPCurves`, after computing each edge's
pcurve, run a per-edge guard `pcurveFaithful(srf, edge, pc)`:

```
scale = extent of the surface control net (max |pole - frame.origin|), ≥ 1
for t_k in a fixed set across [first,last] (endpoints + interior, ~5–9 samples):
    uv   = pcurveValue(pc, t_k, frac_k)          // trim.h, the SAME evaluator the mesher uses
    Sp   = surfaceValue(srf, uv.u, uv.v)          // rational-aware B-spline eval
    Ce   = evalEdge(edge.curve, t_k)              // the 3D edge, existing helper
    if distance(Sp, Ce) > 1e-6 * max(1, scale):   // scale-relative, NEVER weakened
        return false                               // this edge is unfaithful
return true
```

Any boundary edge for which `pcurveFaithful` is false ⇒ `decline()` (sets `fail_`), so
`advancedFace`/`closedShell` abort the whole import → OCCT. This catches: a
non-converging `projectBSplineUV` inversion (its output does not re-evaluate to the
edge), a curved fit that does not lie on the patch, and any beyond-tolerance boundary
gap — exactly the M0-delta contract, now actually evaluable because `surfaceValue`
handles `Kind::BSpline`. Using `trim.h::pcurveValue` as the guard's UV evaluator makes
the guard check the SAME pcurve the mesher will flatten (no evaluator drift).

Complexity: `pcurveFor`'s new arm delegates the curved fit to a `bsplinePCurveUV(srf,
edge)` helper and the straight/curved decision to a `uvStraight(...)` predicate; the
guard is one small free function. Each stays ≤ ~10 cognitive; the reader's per-function
band (parser/geometry, ≤ 25-35) is not approached.

## 3. Admission + honest-out (unchanged arbiter)

No new admission branch is needed in `advancedFace`: a trimmed `Kind::BSpline` face
already reaches `buildFaceWithPCurves`. The guard turns today's blind build into an
**explicit faithful-or-decline** build. Downstream, the engine's mandatory watertight +
volume/area self-verify (`src/engine`) is the final arbiter: an admitted patch whose
native mesh is not watertight or off-volume is DISCARDED → OCCT. So there are two
independent honest-outs — the reader's per-edge pcurve guard and the engine's mesh
self-verify — and a wrong/leaky solid is never emitted.

## 4. Verification — two gates

- **HOST ANALYTIC (no OCCT linked).** Build a native trimmed `Kind::BSpline` face whose
  outer `EDGE_LOOP` has a closed-form curved boundary (e.g. an edge whose `(u,v)` image
  is a known curve on a known patch — a bump/saddle patch reused from the M0 host gate).
  Assert: (i) the guard ACCEPTS the reconstructed pcurve — `S(pcurve(t)) = C_edge(t)`
  within `1e-6·max(1,scale)` at many `t`; (ii) the guard REJECTS a deliberately
  perturbed off-surface edge (`decline()` fires); (iii) the meshed solid (via the M0
  mesher) is watertight and its enclosed volume matches the independent closed-form
  value within tol; (iv) a rational (`weights`) variant passes the same. No OCCT
  symbol linked — the guard's correctness is proven against a closed-form oracle.
- **SIM native-vs-OCCT (booted iOS simulator, OCCT linked).** Import the foreign
  trimmed-B-spline STEP fixture with the native engine active. Assert the native solid's
  volume / surface area / watertight status / triangle envelope / sub-shape topology
  match OCCT `STEPControl_Reader` re-import + `BRepMesh_IncrementalMesh` within tol; OR
  the reader declines and the file round-trips through OCCT unchanged (both PASS). A
  second fixture with one unfaithful boundary edge MUST decline → OCCT. Confirm the
  engine self-verify DISCARDS any non-watertight admitted result → OCCT.

## 5. Zero-regression (mandatory, proven)

The straight-UV arm is byte-identical to today's generic-linear pcurve for a
straight-in-`(u,v)` edge, so every existing STEP round-trip is unchanged
(`run-sim-suite` 221/221, STEP import 77/77). The tessellator is not touched, so all
tessellation-sensitive suites are unaffected by construction. If a curved-edge pcurve
cannot be made faithful robustly on the fixture, REVERT the curved arm, keep the
straight arm + guard (which only ever tightens behaviour by declining), and DOCUMENT the
measured gap — the reader stays honest, no fabrication, no dead code, no weakened
tolerance.

## 6. Alternatives considered

- **Fit the UV pcurve by global least-squares on many samples** rather than projecting
  a densified sample set into `poles2d` directly. Deferred: the guard is the arbiter of
  faithfulness regardless of fit method, and the direct projected-poles form is the
  exact inverse of the writer arm and simplest to prove. A tighter fit is a later M4
  slice if the guard rejects otherwise-admissible patches.
- **Admit without a guard and rely solely on the engine self-verify.** Rejected: that is
  today's blind behaviour; an explicit per-edge measured decline is the
  never-fabricate-geometry contract and localises the blocker to the exact edge.
- **Edit the tessellator to special-case B-spline trims.** Rejected: M0 already ships a
  general trimmed-freeform mesher; this slice must CONSUME it, not fork it.

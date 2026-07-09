# Design — moat-fcw-fillet-corner-weld

## The crux: a curved↔curved shared arc

At a solid corner V_i where two convex face-bounding edges E_{i−1}, E_i are filleted at
radius r, three surfaces meet: the two tangent CYLINDERS (one per edge) and a SPHERE
corner patch. The weld is watertight iff all three share EXACT seam vertices along each
tangent arc — the classic "one canonical discretization per shared curved arc" the M0
welds established.

### Why the sphere centre lands on both cylinder axes (the exactness)

For edge E_i between face F (normal nF) and side face i (normal s_i), the rolling-ball
cylinder axis is the locus at signed distance −r from BOTH planes. The corner sphere
centre S_i is the point at distance −r from F, side i−1 AND side i. Those first two
constraints (−r from F and side i) are exactly the two that define cylinder E_i's axis
line ⇒ **S_i lies on cylinder E_i's axis** (and, symmetrically, on E_{i−1}'s). Hence the
cylinder's cross-section circle in the plane through S_i ⟂ crease is a GREAT CIRCLE of
the sphere: the cylinder strip's end arc and the sphere patch's leg arc are the SAME
quarter circle. Sampling both with one routine ⇒ coincident vertices ⇒ watertight weld.

### Canonical arc sample

`arcSample(S, r, dFrom, dTo, k, steps)` returns `S + r·slerp(nrm(dFrom), nrm(dTo), k/steps)`
(great-circle slerp, robust where a rotation-matrix axis `cross(dFrom,dTo)` would
degenerate for near-parallel directions). The cylinder strip end rim at corner S_i uses
`arcSample(S_i, r, nF, s_i, ·, M)`; the sphere patch leg for edge i uses the identical
call — bit-identical points. A single GLOBAL step count M (max over edges from the
deflection sagitta bound) makes every shared arc use the same M.

## Assembly (one planar-facet soup → assembleSolid)

1. **Trimmed face F** — inset polygon, corner j at the sphere F-tangent point
   `S_j + r·nF` (two adjacent edges' F-tangent lines cross there). Planar.
2. **Side faces** — clip each to the F-offset-by-r plane (drop the top strip of height
   r), then FAN-triangulate from the vertex farthest off the cut line with the tangent
   / corner points inserted on the cut edge. The fan avoids the 4-collinear-point
   ear-clip degeneracy that otherwise drops a sliver and opens the seam for some face
   orientations. Outward normal centroid-verified (robust to a reversed base face).
3. **Cylinder strips** — per edge, quads between the two corner cross-sections (S_i,
   S_{i+1}) sampling `arcSample(·, nF, s_i, ·, M)`; the F-side rim (k=0) meets the
   trimmed cap, the side rim (k=M) meets the side-face cut edge.
4. **Sphere octant + flat corner ledge** — the geodesic triangle (nF, s_{i−1}, s_i):
   its two legs are the incident cylinder end arcs (shared, bit-identical); its third
   arc (row M, in the F-offset plane — the sphere equator at centre height, planar for
   perpendicular walls) bounds a FLAT ledge fanned to the corner point `cp` (the two
   side planes ∩ the F-offset plane — the top of the un-rounded sharp vertical corner).
5. **Weld** — `assembleSolid` (VertexPool 1e-7 weld + T-junction repair + triangulate).

## Scope guard (perpendicular walls)

The corner ledge is planar (arc C lies in the F-offset plane) iff every incident side
face is perpendicular to F: `|nF·s_i| ≤ 1e-6`. That is exactly a prism cap. A
non-perpendicular wall ⇒ arc C is a non-planar sphere arc and the ledge is non-planar
⇒ DECLINE (`NotPerpWall`) → OCCT. This keeps the weld EXACT within a verifiable scope.

## Self-verify (never a wrong solid)

- `tess::isConsistentlyOriented` — directed-edge orientation coherence AND watertight
  (a same-direction duplicate half-edge, the inconsistent-shell signature, fails it).
- Two-sided SHRINK volume bound: `0 < V < V(original) − tol` (a convex fillet only
  removes material). NaN / non-positive / grow ⇒ decline.
- Any miss ⇒ NULL → OCCT (`BRepFilletAPI_MakeFillet`). Never a leaky / inverted / wrong
  solid; never a widened tolerance; never a native void forwarded to OCCT.

## Two-gate result

- **Gate (a) HOST-analytic:** the removed volume of a cube side L filleting its 4 top
  edges is `V_removed = r²L(4−π) − 4r³ + (4/3)π r³` (top-slab decomposition: flat core
  column + 4 edge quarter-cylinders + 4 corner eighth-balls). Native converges to it
  monotonically as deflection → 0 (M=48: within 1.3e-5 relative), watertight +
  consistently oriented at every deflection.
- **Gate (b) SIM parity:** vs OCCT `BRepFilletAPI_MakeFillet` — volume rel <5e-3
  (analytic <1e-3), area <2e-2, watertight, χ=2, bbox exact. The vertex Hausdorff sees
  OCCT's O(r) setback-corner-shape convention difference (documented, not gated — the
  meaningful mass/topology/bbox properties all match, and the native volume is closer
  to the ideal closed form than OCCT's).

## Alternatives considered

- **Generalize `fillet_edges` to corner-sharing sets** — rejected: its per-edge
  sequential clip + arc strips fundamentally can't coordinate a shared corner
  discretization; a dedicated from-scratch soup builder is clearer and lower-risk.
- **Tessellator change (topology-guarded sphere↔cylinder seam single-sampling)** —
  UNNECESSARY: the shared-arc identity closes the weld purely in the assembly layer, so
  the mesher is untouched (the preferred, lowest-risk path per the campaign guidance).

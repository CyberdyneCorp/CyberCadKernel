# Design — moat-m3af-analytic-fillet (MOAT M3, analytic-solid face fillets)

## fillet_face = round-every-bounding-edge, reusing the landed dihedral fillet

The OCCT adapter it mirrors (`OcctEngine::fillet_face`) rounds EVERY edge of the
picked face at radius r:

    for each edge e of face F:  BRepFilletAPI_MakeFillet::Add(r, e)

For a native all-planar solid this is exactly the landed multi-edge
`nblend::fillet_edges` applied to the edge set bounding `F`. `fillet_face(solid,
faceId, r)`:

1. resolve `faceId` (`blend_geom.h` `facePlane` guards it is planar);
2. collect the 1-based ids (mapShapes(Edge) order) of every edge of `F` whose two
   incident faces are planar and meet CONVEXLY (a `nblend::detail::filletArc`
   probe with radius r must succeed — the same convexity + r-fits-the-faces guard
   the dihedral fillet uses);
3. call `nblend::fillet_edges(solid, edgeIds, n, r)` — the byte-frozen tangent-
   cylinder blend with the open-seam weld — and return its result.

The engine then runs the SAME `blendResultVerified(result, original, wantGrow=false)`
guard the landed `fillet_edges` uses: a face fillet REMOVES material (0 < Vr < Vo).
A non-planar picked face, a non-all-planar solid, any concave / curved / ≠2-face
bounding edge, or an interfering radius (a bounding edge whose `filletArc` fails)
DECLINES — the multi-edge builder returns NULL / self-verify discards, and the
engine reports the SAME honest decline the prior hard-decline produced. A native
void is NEVER handed to OCCT.

**Why it declines this wave (MEASURED).** The landed multi-edge `fillet_edges` welds
NON-adjacent edge sets (an opposite pair welds; the full-round below relies on exactly
this), but two edges meeting at a CORNER leave a gap between their two cylinder blends
that needs a SPHERICAL corner patch — the corner weld that gates on M2. The host gate
MEASURES this: `fillet_edges` on any two ADJACENT box-face edges returns NULL at every
radius, so a full-face fillet (a closed loop of corner-sharing edges) DECLINES with
`WeldGatesM2`. The native path is fully wired + self-verified — it accepts any face
whose convex edges weld as a group — so it lands automatically once M2 supplies the
corner weld, with NO engine change. The host gate asserts the measured decline; the
SIM gate asserts the native op returns id 0 (honest error, never a void handed to
OCCT) while the OCCT engine owns the `BRepFilletAPI_MakeFillet` reference.

## full_round_fillet = the r = w/2 tangent-cylinder cap on a prismatic rib

A full round replaces a narrow middle face `M` between two side walls `L`, `R` with a
blend tangent to both, consuming `M`. For the ANALYTIC PRISMATIC case — `L` and `R`
PARALLEL planar walls a distance `w` apart, `M` the planar strip capping the rib —
the rolling ball has radius `r = w/2`, its axis runs along the strip mid-line
(direction = the seam-edge direction), sitting on the strip mid-plane at distance r
from each wall, and its crest lands where `M` was. This is the r = w/2 special case of
the same rolling-ball construction `fillet_edges` builds, applied to the TWO seam
edges `M↔L` and `M↔R` at radius w/2: rounding both seam edges at r = w/2 makes the two
arcs meet tangentially on the mid-plane and the middle face vanishes — a single
half-cylinder cap tangent (G1) to both walls at the two seams.

Implementation reuses `nblend::fillet_edges(solid, {eL, eR}, r = w/2)`:

- `full_round_fillet_faces(solid, L, M, R)`: seam edges = the longest shared edge of
  `M∩L` and `M∩R` (mirrors OCCT `resolveFromFaces`); `w` = perpendicular gap between
  them; verify `L ∥ R` (`|nL·nR| ≈ 1`) and that both seams are straight and equal-
  length (prismatic), else DECLINE.
- `full_round_fillet(solid, M)`: the two LONGEST edges of `M` are the seams; their
  across-neighbours are `L`, `R` (mirrors OCCT `resolveAuto`); then as above.

The engine gates the result with `blendResultVerified(wantGrow=false)` (a round
consuming a convex rib REMOVES material) AND checks the middle face is gone (the
result has fewer distinct planes than the input). 

**Why the dihedral / curved / closed-seam cases decline.** A NON-parallel (dihedral)
middle needs the `nL×n R` valley-solve and a variable seam sweep the landed
constant-radius planar-dihedral fillet does not build for a face pair (it fillets a
single crease, not a two-seam cap where the arcs must MERGE across the consumed face);
landing it robustly needs the M2 curved-wall / closed-seam weld. A CURVED wall has no
constant-radius rolling ball. A CLOSED-SEAM annulus (a full round on a circular boss
top) needs the closed-seam weld that gates on M2. All three DECLINE with a measured
reason → OCCT `BRepFilletAPI` full-round path.

**Host-analytic oracle.** For a plain box of top width `w` and length `L`, a full
round rounds the top strip end-to-end: the cap replaces the top slab of cross-section
`w × (w/2)` (a rectangle of width `w`, height = the ball radius `r = w/2`) with a
half-disk of radius `w/2`. The removed cross-section is the rectangle minus the
half-disk, `w·(w/2) − π(w/2)²/2 = (w²/2)(1 − π/4)`, so `V_removed = (w²/2)(1 − π/4)·L`.
The host gate asserts this volume (single prismatic rib), tangency of the cap cylinder
to both walls, the middle face consumed, and watertight; the SIM gate asserts parity
vs the OCCT full-round oracle.

## Self-verify sign, honest decline

Both ops are SHRINK (`wantGrow=false`): a convex face fillet and a convex full round
both remove material. Because the landed guard rejects a wrong-signed volume and the
multi-edge builder returns NULL for anything outside the planar-convex domain, neither
result can be spoofed. A native body the builder cannot serve returns a clean error
with a measured reason (never forwarded to OCCT); an OCCT body forwards to the OCCT
oracle unchanged.

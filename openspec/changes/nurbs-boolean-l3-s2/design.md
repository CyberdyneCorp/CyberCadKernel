# Design — nurbs-boolean-l3-s2

## Context

L3-S1 (`nurbs_plane_split.h`) cut a NURBS face by a **plane**: the seam is NURBS∩plane,
the keep test is a half-space side, the sew is curved-NURBS↔FLAT (the M0w pin). L3-S2 is
the SAME verb with the cutter left as a genuine analytic **curved** surface G, so three
things change — each routed to a measured-WORKS piece so the slice composes existing code
and routes around the general freeform↔freeform sew.

## Key decisions

1. **Scope = ANALYTIC curved cutter (Cylinder/Sphere/Cone), not freeform.** An analytic
   cutter is recognised by S5-a `recogniseCurvedSolid` (reused), its adapter is the
   transversal-WORKS `CurvedSolid::adapter()`, and its membership is the closed-form
   `classifyPoint`. This gives an exact closed-form oracle and a curved cap that fans over
   the true analytic surface. A freeform (both-NURBS) cutter needs the general
   freeform↔freeform sew — the DEFERRED deep tail (declared, not attempted).

2. **Stage 5 sew = deflection-bounded planar-triangle FAN, not a trimmed curved face.** A
   trimmed curved-G face, meshed, produces boundary points ON G that BULGE off the disk's
   straight seam chords → a T-junction (the measured `appendMouthCap` failure). Instead the
   cap is a fan (the S5-a idiom, reused via `assemble.h::{VertexPool, triangleFace}`): the
   OUTER ring is the EXACT traced seam nodes (bit-identical to the disk chords, so the M0
   mesher position-welds them watertight), interior ring/centre points are ON G (so the cap
   follows G's curvature to O(1/rings²)). This is the curved-NURBS↔analytic-CURVED weld.

3. **Stage 4 keep = curved membership, not half-space.** `classifyPoint(G, centroid)` gives
   inside/outside/ON. `KeepSide::Above` keeps the sub-face INSIDE G (COMMON); `Below` keeps
   OUTSIDE (CUT). An ON centroid is ambiguous → decline.

4. **Fidelity on BOTH operands.** The seam pcurve is read from the WLine on both F (`u1,v1`)
   and G (`u2,v2`); BOTH must round-trip (`S_F==node`, `S_G==node`). A drift on either side
   rejects — the S(pcurve)==C invariant, now two-sided.

5. **Oracle = a clean single-seam LENS.** The paraboloid∩sphere has TWO coaxial circles; the
   fixture sizes the NURBS patch (half-width H=0.35) so the OUTER circle (r≈0.598) is OFF the
   patch and only the INNER circle (ρ=0.25) is traced — a clean CLOSED interior loop (the
   L3-S1 trace path). The CUT/Below keep side is the closed-form lens
   `2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}]` = cup − ball.

## Honesty / self-verify

The mandatory M0 watertight + positive-volume self-verify is the gate; ANY failure →
measured `NurbsCurvedSplitDecline` + NULL. No tolerance is weakened. The two-gate proof is
host closed-form (LENS volume converging, χ=2, DISAGREED=0) + sim vs OCCT
`BRepAlgoAPI_Cut/Common`.

## Deferred (the L3 deep tail — declared, not attempted)

- General NURBS↔NURBS where the cutter is FREEFORM (both operands arbitrary NURBS) — the
  general freeform↔freeform sew (stage 5).
- Closed-interior-loop seams the SSI seeder misses (stage 1 recall gap).
- Multi-crossing / re-entrant / multi-seam splits (stage 3).

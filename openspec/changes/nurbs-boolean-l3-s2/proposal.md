# Proposal — nurbs-boolean-l3-s2

## Why

`openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` maps the NURBS-roadmap **Layer 3** (the
exact-NURBS B-rep boolean) into five stages. Slice **L3-S1** (`nurbs-boolean-l3-s1`,
`nurbs_plane_split.h`) landed the first exact-NURBS boolean — a genuine NURBS face cut by
a **PLANE**, welded curved-NURBS↔FLAT. The readiness doc named the next wall as **stage 5,
the curved↔CURVED sew**: cutting a NURBS face by a **CURVED** face makes the seam a curve
on BOTH curved surfaces, and the cap that closes the kept piece is a patch of the curved
cutter — a curved↔curved weld, not curved↔flat. This change LANDS the tractable slice of
that wall.

**L3-S2: a genuine NURBS face SPLIT BY an ANALYTIC CURVED face (Cylinder/Sphere/Cone),
welded watertight.** It extends L3-S1 from a planar cutter to an analytic curved cutter,
scoped to the ANALYTIC-curved cutter (the transversal-WORKS trace + the S5-a curved
membership + a deflection-bounded curved cap) so it composes ONLY measured-WORKS pieces
and routes around the general freeform↔freeform sew (the deep-tail residual). It is gated
by the two-gate discipline (host closed-form volume + SIM vs OCCT `BRepAlgoAPI_Cut`), so
it is a proven milestone, never a shaky general boolean.

## What Changes

1. **A new native boolean verb** `src/native/boolean/nurbs_curved_split.h`
   (`nurbsFaceCurvedSplit`, namespace `cybercad::native::boolean`, OCCT-free,
   header-only, `clang++ -std=c++20`, substrate-gated `CYBERCAD_HAS_NUMSCI`). Given a
   trimmed NURBS wall FACE (`Kind::BSpline`), its flat closing base (a Plane face), an
   ANALYTIC CURVED cutter solid (Cylinder/Sphere/Cone, recognised by S5-a
   `recogniseCurvedSolid`) whose `wall ∩ G` is a CLOSED interior seam, and a keep `side`,
   it composes the L3-S2 recipe:
   - **stage 1 TRACE** — `wall ∩ G` → the closed interior seam WLine, via the NURBS
     operand adapter (`npsdetail::makeWallAdapter`, reused from L3-S1) ∩ the curved
     cutter's own `ssidetail::CurvedSolid::adapter()` (the transversal-WORKS path). The
     WLine node carries `(u1,v1)` on the NURBS wall AND `(u2,v2)` on the curved cutter.
   - **stage 2 FIDELITY (routed around `constructPcurve`)** — the seam pcurve is READ
     from the WLine on BOTH operands, gated by `S_F(u1,v1)==node` AND `S_G(u2,v2)==node`
     (the seam lies on BOTH surfaces); a drifted seam on EITHER operand is REJECTED
     (`SeamOffSurface`), never welded.
   - **stage 3 SPLIT** — `splitFaceSmoothTrim` (reused byte-identical) partitions the
     NURBS wall into disk + annulus along the seam.
   - **stage 4 KEEP** — a CURVED-solid membership test `ssidetail::classifyPoint(G, ·)`
     (inside/outside the cylinder/sphere/cone) at the kept sub-face's trim centroid — NOT
     a plane half-space. An ON (ambiguous) membership declines (`KeepFaceUnusable`).
   - **stage 5 SEW (curved↔CURVED)** — the cap that closes the kept piece is a patch of
     the curved cutter G bounded by the SAME seam, synthesized as a deflection-bounded
     PLANAR-TRIANGLE FAN (the S5-a `appendMouthCap` idiom): the OUTER ring is the EXACT
     traced seam nodes (bit-identical to the NURBS disk's straight seam chords, so the M0
     mesher position-welds the two watertight), every interior ring/centre point is
     evaluated ON G (the cap follows G's curvature to O(1/rings²)). Plus the kept flat
     base when its centroid is on the keep side. → Shell → Solid.

2. **A mandatory self-verify** — mesh (M0), require watertight (closed 2-manifold) AND a
   positive finite enclosed volume; ANY decline returns a NULL Shape with a measured
   `NurbsCurvedSplitDecline` reason. NEVER a leaky / partial solid; NO tolerance widened.

3. **The two-gate proof.** Host gate (a) `tests/native/test_native_nurbs_curved_split.cpp`
   (a `Kind::BSpline` paraboloid bowl cut by a genuine analytic SPHERE: the CUT/Below keep
   side is the closed-form LENS `V = 2π[zc·ρ²/2 − a·ρ⁴/4] − (2π/3)[Rs³ − (Rs²−ρ²)^{3/2}]`,
   the meshed volume converging to it as the mesh refines; watertight χ=2; seam on BOTH
   curved surfaces; honest NULL declines for a non-curved cutter / non-NURBS wall / far
   sphere) + sim gate (b) `tests/sim/native_nurbs_curved_split_parity.mm` (native vs OCCT
   `BRepAlgoAPI_Cut(cup, ball)` for the lens + `BRepAlgoAPI_Common(cup, ball)` for the
   inside piece — volume/watertight/χ parity within the tessellation band).

## Impact

- **New files:** `src/native/boolean/nurbs_curved_split.h`,
  `tests/native/nurbs_curved_split_fixture.h`,
  `tests/native/test_native_nurbs_curved_split.cpp`,
  `tests/sim/native_nurbs_curved_split_parity.mm`,
  `scripts/run-sim-native-nurbs-curved-split.sh`.
- **CMake:** register `test_native_nurbs_curved_split` + its `CYBERCAD_HAS_NUMSCI`
  compile-definition (additive; if/endif balance preserved).
- **Docs:** update `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` (§3 records L3-S2 landed,
  stage 5 curved↔curved sew resolved for the analytic cutter) + `docs/NURBS-SCOPE.md` §4
  Layer-3 row.
- **Unchanged:** `cc_*` ABI byte-identical; `src/native` OCCT-free; `nurbs_plane_split.h`,
  `ssi_boolean.{h,cpp}`, `assemble.h`, `face_split.h`, `src/native/ssi`,
  `src/native/topology`, `src/native/math` NOT modified (all composed byte-identically).
- **Honest residual (DEFERRED, the L3 deep tail):** the general NURBS↔NURBS split where the
  cutter is itself FREEFORM (both operands arbitrary NURBS — needs the general
  freeform↔freeform sew, stage 5), closed-interior-loop seams the SSI seeder misses
  (stage 1 recall), and multi-crossing / re-entrant / multi-seam splits (stage 3).

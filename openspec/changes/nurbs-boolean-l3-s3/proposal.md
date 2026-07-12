# Proposal — nurbs-boolean-l3-s3

## Why

`openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` maps the NURBS-roadmap **Layer 3** (the
exact-NURBS B-rep boolean) into five stages. Slice **L3-S1** (`nurbs-boolean-l3-s1`,
`nurbs_plane_split.h`) cut a NURBS face by a **PLANE** (sew = curved-NURBS↔FLAT); slice
**L3-S2** (`nurbs-boolean-l3-s2`, `nurbs_curved_split.h`) cut a NURBS face by an **ANALYTIC
CURVED** face (Cylinder/Sphere/Cone — sew = curved-NURBS↔analytic-CURVED, a closed-form fan
on the true cutter surface). Both routed around the last named wall: **the general
freeform↔freeform sew** (stage 5, the deep-tail), where BOTH operands are arbitrary NURBS
so the kept-cutter cap is itself a curved NURBS sub-face (no closed-form fan) and the sew is
NURBS-disk↔NURBS-disk along a shared curved seam, both sides welding bit-for-bit watertight.

**L3-S3: a genuine NURBS face SPLIT BY ANOTHER FREEFORM NURBS face (BOTH operands arbitrary
NURBS), welded watertight.** This is the deepest boolean rung — the person-decade moat core.
The observation that makes the tractable slice reachable: `boolean/freeform_freeform_cut.h`
(`freeformFreeformClosedSeamCut`) ALREADY performs the freeform↔freeform curved↔curved
closed-seam weld (split BOTH walls along the shared 3-D seam, select survivors by mesh
membership, repair orientation coherence) — proven watertight at the closed-form lens volume
for the **COMMON** leg — but ONLY for `Kind::Bezier` walls (its wall gate rejects any
non-Bézier surface and its trace hardcodes `makeBezierAdapter`). L3-S3 is that SAME verb with
BOTH walls left as genuine `Kind::BSpline`/NURBS surfaces and the seam traced through the L3
NURBS operand front-end (`makeBSplineAdapter`/`makeNurbsAdapter`). It is gated by the two-gate
discipline (host closed-form lens volume + SIM vs OCCT `BRepAlgoAPI_Common`), so it is a proven
milestone of the general freeform↔freeform sew, never a shaky general boolean.

## What Changes

1. **A new native boolean verb** `src/native/boolean/nurbs_freeform_split.h`
   (`nurbsFaceFreeformSplit`, namespace `cybercad::native::boolean`, OCCT-free, header-only,
   `clang++ -std=c++20`, substrate-gated `CYBERCAD_HAS_NUMSCI`). Given TWO freeform bowl-cup
   operands `F`, `G` whose SINGLE freeform walls are BOTH genuine NURBS (`Kind::BSpline`)
   surfaces meeting in ONE CLOSED interior seam, and an `FfOp`, it composes the L3-S3 recipe:
   - **stage 0/1 RECOGNISE** — B1 `recogniseFreeformSolid` admits both operands (it ALREADY
     tags `Kind::BSpline` faces as `FaceRole::Freeform`); each operand SHALL present EXACTLY
     one freeform wall, and that wall SHALL be `Kind::BSpline` (NURBS, not Bézier — the
     Bézier↔Bézier case is `freeform_freeform_cut.h`'s; the analytic case is L3-S2's). Else
     `WallFNotNurbs` / `WallGNotNurbs`.
   - **stage 1 TRACE** — `F.wall ∩ G.wall` → the closed interior seam WLine, via the L3 NURBS
     operand adapter (`npsdetail::makeWallAdapter`, reused from L3-S1) on BOTH walls. The WLine
     node carries `(u1,v1)` on F's wall AND `(u2,v2)` on G's wall.
   - **stage 2 FIDELITY (routed around `constructPcurve`)** — the seam pcurve is READ from the
     WLine on BOTH operands, gated by `S_F(u1,v1)==node` AND `S_G(u2,v2)==node` (the seam lies
     on BOTH freeform surfaces, evaluated via `tess::SurfaceEvaluator`); a drifted seam on
     EITHER operand is REJECTED (`SeamOffSurface`), never welded.
   - **stage 3 SPLIT** — `splitFaceSmoothTrim` (reused byte-identical) partitions F's NURBS
     wall by `(u1,v1)` and G's NURBS wall by a `(u2,v2)`-re-keyed copy of the SAME seam
     (`ffcdetail::rekeyToB`), each into disk + annulus.
   - **stage 4 SELECT** — survivors by B3 `classifyPointInMesh` membership of each sub-face
     centroid in the OTHER operand's pre-cut mesh (`ffcdetail::pickByMembership`, reused): for
     COMMON, F's disk INSIDE G + G's disk INSIDE F (the lens). An On/Unknown/wrong-side
     membership declines (`ClassifyAmbiguous`).
   - **stage 5 SEW (curved-NURBS↔curved-NURBS)** — the two NURBS caps share the EXACT seam
     nodes (`splitFaceSmoothTrim`'s bit-identical outer ring on both sides), so the M0 mesher
     position-welds NURBS-disk↔NURBS-disk watertight. `ffcdetail::weldOrientationCoherent`
     (reused) repairs orientation coherence (the directed-edge invariant — exactly one cap
     reversed) so the two curved NURBS caps form a coherent outward-normal boundary.

2. **A mandatory self-verify** — the reused weld helper returns only a watertight + coherently
   oriented mesh; the verb then requires a positive finite enclosed volume, an op-volume UPPER
   bound (COMMON ⊂ F and ⊂ G), and — when the closed-form op-volume is supplied — a TWO-SIDED
   band (so a too-small orientation-collapsed volume is rejected). ANY decline returns a NULL
   Shape with a measured `NurbsFreeformSplitDecline` reason. NEVER a leaky / partial / wrong
   solid; NO tolerance widened.

3. **The two-gate proof.** Host gate (a) `tests/native/test_native_nurbs_freeform_split.cpp`
   (two `Kind::BSpline` paraboloid bowl-cups — an UP bowl F + a DOWN dome G — meeting in ONE
   closed circular seam: the COMMON lens enclosed volume converging to the closed form
   `V = π·H²/(4a)` as the mesh refines; watertight χ=2, consistently oriented; seam on BOTH
   NURBS walls (DISAGREED=0); honest NULL declines for a null operand / non-intersecting pair /
   the apex-ambiguous CUT leg) + sim gate (b)
   `tests/sim/native_nurbs_freeform_split_parity.mm` (native vs OCCT
   `BRepAlgoAPI_Common(F, G)` on two reconstructed `Geom_BSplineSurface` cups — volume /
   watertight / orientation / χ parity within the tessellation band; OCCT cross-checked
   against the closed form).

## Impact

- **New files:** `src/native/boolean/nurbs_freeform_split.h`,
  `tests/native/nurbs_freeform_split_fixture.h`,
  `tests/native/test_native_nurbs_freeform_split.cpp`,
  `tests/sim/native_nurbs_freeform_split_parity.mm`,
  `scripts/run-sim-native-nurbs-freeform-split.sh`.
- **CMake:** register `test_native_nurbs_freeform_split` + its `CYBERCAD_HAS_NUMSCI`
  compile-definition (additive; if/endif balance preserved).
- **Docs:** update `openspec/L3-EXACT-NURBS-BOOLEAN-READINESS.md` (§3 records L3-S3 landed,
  stage 5 general freeform↔freeform sew resolved for the tractable COMMON single-seam pose) +
  `docs/NURBS-SCOPE.md` §4 Layer-3 row.
- **Unchanged:** `cc_*` ABI byte-identical; `src/native` OCCT-free; `nurbs_plane_split.h`,
  `nurbs_curved_split.h`, `freeform_freeform_cut.h`, `freeform_operand.h`,
  `freeform_membership.h`, `smooth_trim_split.h`, `ssi_boolean.{h,cpp}`, `assemble.h`,
  `face_split.h`, `src/native/ssi`, `src/native/topology`, `src/native/math` NOT modified
  (all composed byte-identically).
- **Honest residual (DEFERRED, the L3 deep tail):** the **CUT** (`F − G`) leg — whose
  apex-adjacent survivor membership is ambiguous even in the Bézier case (it honest-declines
  in L3-S3 too, reached-not-faked); **multi-crossing / re-entrant / multi-seam** NURBS↔NURBS
  splits; closed-interior-loop seams the SSI seeder misses (stage 1 recall); and a
  boolean-grade general `constructPcurve` (stage 2). L3-S3 lands the tractable
  single-transversal-seam COMMON lens; the general BOPAlgo-class NURBS↔NURBS boolean remains
  the multi-py program the slices decompose.

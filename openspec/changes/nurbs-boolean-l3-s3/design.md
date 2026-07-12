# Design — nurbs-boolean-l3-s3

## Context

L3-S1 (`nurbs_plane_split.h`) cut a NURBS face by a **plane** (sew = curved-NURBS↔FLAT, the
M0w pin); L3-S2 (`nurbs_curved_split.h`) cut a NURBS face by an **analytic curved** face
(sew = curved-NURBS↔analytic-CURVED, a closed-form fan on the true cutter). Both routed
around the last wall the readiness doc named: the **general freeform↔freeform sew** — BOTH
operands arbitrary NURBS, so the kept-cutter cap is itself a curved NURBS sub-face (no
closed-form fan) and the sew is NURBS-disk↔NURBS-disk along a shared curved seam. L3-S3 lands
the tractable slice of that wall.

## Key decisions

1. **The freeform↔freeform curved↔curved weld already EXISTS — for Bézier.**
   `boolean/freeform_freeform_cut.h` (`freeformFreeformClosedSeamCut`) already splits BOTH
   curved walls along the shared 3-D seam (`splitFaceSmoothTrim`, outer ring = the EXACT
   shared seam nodes on both sides ⇒ bit-identical chords ⇒ the M0 mesher position-welds
   them), selects survivors by mesh membership, and repairs orientation coherence (the
   directed-edge invariant — exactly one cap reversed) so the two curved caps form a coherent
   outward-normal boundary. Its **COMMON** leg is PROVEN watertight at the closed-form lens
   volume (`test_native_freeform_freeform_cut`). The ONLY thing gating it to Bézier is its
   wall admission (`ffcdetail::freeformWall` requires `Kind::Bezier`) and its trace
   (`ffcdetail::traceSharedSeam` hardcodes `makeBezierAdapter`). **L3-S3 = that same weld with
   both walls left as genuine `Kind::BSpline`/NURBS and the seam traced through the L3 NURBS
   operand front-end.** So the deep-tail wall is reached by composition, not a re-implemented
   watertight sew.

2. **Additive discipline: a NEW verb, not an edit of `freeform_freeform_cut.h`.** The scope
   forbids modifying `freeform_freeform_cut.h`. `nurbs_freeform_split.h` therefore REUSES the
   surface-kind-agnostic `ffcdetail::` helpers (`rekeyToB`, `pickByMembership`,
   `collectAnalyticByMembership`, `weldOrientationCoherent`) byte-identically, and adds only
   the two NURBS-specific pieces: `nfsdetail::nurbsWall` (requires `Kind::BSpline`, the analog
   of `freeformWall`'s Bézier gate) and `nfsdetail::traceNurbsSharedSeam` (uses
   `npsdetail::makeWallAdapter` on both walls, the analog of `traceSharedSeam`'s
   `makeBezierAdapter`). The B1 recogniser `recogniseFreeformSolid` ALREADY admits
   `Kind::BSpline` walls (`fodetail::classifyFaceRole` tags them `Freeform`), so no operand
   plumbing changes.

3. **Scope = COMMON single-transversal-seam lens, not CUT.** The COMMON leg (both disks INSIDE
   the other operand — the lens) is the one whose membership is unambiguous and whose weld the
   Bézier case proves watertight+convergent. The **CUT** leg (`F − G`) has an apex-adjacent
   survivor membership that is ambiguous even in the Bézier case (it honest-declines there);
   L3-S3 REACHES the CUT membership and honest-declines it too (`ClassifyAmbiguous`) — a
   measured decline, never a faked solid — leaving it a declared DEFERRED residual.

4. **Fidelity on BOTH freeform operands.** The seam pcurve is read from the WLine on both F
   (`u1,v1`) and G (`u2,v2`); BOTH must round-trip against a `tess::SurfaceEvaluator` on the
   respective NURBS wall (`S_F==node`, `S_G==node`), and the node's on-both-surfaces residual
   must be within tol. A drift on either side rejects — the S(pcurve)==C invariant, two-sided
   over two freeform surfaces (the readiness doc's DISAGREED=0 discipline).

5. **Oracle = a clean single-seam LENS between two mirrored NURBS bowls.** Two coaxial
   degree-2 clamped-B-spline bowl-cups — an UP bowl `z_F = a·(x²+y²)` and a DOWN dome
   `z_G = H − a·(x²+y²)` (F mirrored in z about `z = H/2`), each trimmed by the same rim R —
   meet where `a·r² = H − a·r²`, ONE closed circle at `r = ρ = √(H/2a)`, `z = H/2`. A clamped
   degree-2 B-spline with 3 poles reproduces the quadratic EXACTLY, so each wall represents its
   paraboloid to machine eps and the lens `V(F∩G) = ∫₀^ρ ((H−a·r²) − a·r²)·2πr dr = π·H²/(4a)`
   is exact on THESE surfaces. This is the Bézier fixture (`freeform_freeform_cut_fixture.h`)
   with the two walls upgraded from `Kind::Bezier` to `Kind::BSpline`.

## Measured result

The bounded slice LANDS: the COMMON lens sews watertight for two genuine-NURBS walls.
- Seam on BOTH NURBS walls: `S_F==C` 0, `S_G==C` 2.8e-14, on-both-surfaces 2.8e-14
  (DISAGREED=0); 366-node closed loop, radius ρ on both `(u,v)` to ~1.7e-14.
- Both walls tile under the split (tiling gaps 2.8e-16 / 7.2e-16).
- The lens volume converges monotonely 12.97% → 6.08% → 3.38% → 1.87% as the deflection
  halves (0.01 → 0.00125), toward the closed form `π·H²/(4a) = 0.01005`; finest within the 4%
  band. Watertight, Euler χ = 2, consistently oriented.
- SIM parity vs OCCT `BRepAlgoAPI_Common(F, G)`: native-vs-OCCT volume rel 1.87% (within
  band), OCCT vs closed-form rel 3.5e-7, watertight + oriented + χ=2 (9/9).

## Residual (the L3 deep tail, DEFERRED — declared, not attempted)

- **The CUT (`F − G`) leg** — apex-adjacent survivor membership ambiguous (reached and
  honest-declined, exactly as in the Bézier case).
- **Multi-crossing / re-entrant / multi-seam NURBS↔NURBS splits** (stage 3 general split).
- **Closed-interior-loop seams the SSI seeder misses** (stage 1 recall).
- **A boolean-grade general `constructPcurve`** (stage 2) — routed around here by the
  WLine-(u,v) read.

These are the remaining rungs of the general BOPAlgo-class NURBS↔NURBS boolean; L3-S3 lands
the tractable single-transversal-seam COMMON lens — the general freeform↔freeform sew, proven
for BOTH operands arbitrary NURBS.

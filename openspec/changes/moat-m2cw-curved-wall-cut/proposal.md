# Proposal — moat-m2cw-curved-wall-cut (MOAT M2 curved-wall freeform CUT/COMMON)

## Why

The landed freeform half-space CUT verb `freeformHalfSpaceCut`
(`src/native/boolean/half_space_cut.h`) handles the pose where the cut plane produces
an OPEN seam CHORD on the freeform wall — a bowl-lidded prism cut by a VERTICAL plane,
split by byte-frozen B2 `splitFace` (`crossings == 2`). The M2 roadmap named the DUAL
pose as the sharpened next blocker:

> "the curved-wall freeform half-space CUT that CONSUMES `splitFaceSmoothTrim` (dome cut
> by a horizontal plane: split the freeform cap by the closed seam, split the analytic
> walls, synthesize the flat circular cross-section cap, weld + watertight self-verify)
> + its sim parity gate."

Here the cut plane produces a CLOSED CIRCULAR seam INTERIOR to the freeform wall
(`crossings == 0`) — the case byte-frozen B2 `splitFace` DECLINES and the landed B2
smooth-trim sibling `splitFaceSmoothTrim` (`smooth_trim_split.h`) resolves into a disk +
an annulus-with-hole. No landed weld verb composes the smooth-trim split into a
watertight result solid. This change lands it.

## What

One additive OCCT-free header-only verb `src/native/boolean/curved_wall_cut.h`
(`curvedWallHalfSpaceCut`) — an ADDITIVE SIBLING of `freeformHalfSpaceCut` (which stays
BYTE-FROZEN). Given an admitted freeform operand whose single freeform wall is cut by a
plane along a CLOSED interior seam, it composes:

1. **recognise[B1]** `recogniseFreeformSolid` — exactly one freeform wall.
2. **trace[M1]** the byte-unchanged `hscdetail::traceWallSeam` — the closed circular
   `wall ∩ P` seam WLine (it already accepts a `Closed` WLine).
3. **split[B2 smooth-trim]** `splitFaceSmoothTrim(wall, seam)` — the disk `faceInside`
   the seam encloses + the annulus `faceOutside` (the seam as a hole). CONSUMED
   byte-identical.
4. **wall split[B4]** byte-frozen `hscdetail::cutAnalyticFace` splits every planar
   analytic face the plane crosses; faces on one side are kept / dropped whole.
5. **circular cap synth** — ONE flat planar cap on `P` from the seam polyline (the SAME
   straight chords `splitFaceSmoothTrim` laid on the freeform sub-face), oriented so its
   normal faces the discard side.
6. **weld + self-verify** — kept freeform sub-face + kept analytic faces + the flat cap
   → Shell → Solid; mesh (M0) and require WATERTIGHT AND a positive enclosed volume.

`KeepSide::Below` = CUT (remove the cap above the seam); `KeepSide::Above` = COMMON
(keep the sliced-off cap). ANY decline → NULL Shape (→ OCCT fall-through); NEVER a
leaky/partial solid; NO tolerance widened.

## Impact

- **Additive only.** New file `curved_wall_cut.h`; `smooth_trim_split.h`,
  `face_split.h`, `half_space_cut.h`, `two_operand.h`, `multi_face_weld.h`,
  `seam_graph.h`, `junction_split.h`, `freeform_operand.h`, M0/M1 and the whole
  tessellator are BYTE-IDENTICAL. `src/native/**` stays OCCT-free; no `cc_*` ABI change.
  The verb's per-function cognitive complexity is in the backend band (driver 10).
- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_curved_wall_cut.cpp`
  (8/8). A steep Bézier bowl-cup (bowl trimmed by a rim circle + a flat top-lid disk)
  cut by the horizontal plane `z = c` yields a CLOSED CIRCULAR seam (the real S3 trace)
  interior to the wall. The CUT (Below) welds WATERTIGHT (Euler χ = 2) at the closed
  form `V(z≤c) = π·ρ²·c/2` and CONVERGES monotonically to it across a resonance-free
  deflection sweep (rel 9.4% → 2.7%). COMMON (Above) welds watertight (χ = 2) at
  `V(z≥c) = V(full) − V(z≤c)` (rel 2.0%) at its robust deflection; the closed-form
  partition identity `V(z≤c) + V(z≥c) = V(full)` is exact. The COMMON annulus↔lid rim
  weld is deflection-fragile and honestly DECLINES to NULL away from the robust band
  (asserted as a MEASURED decline — the sharpened next blocker). Non-cutting plane +
  non-operand DECLINE to NULL.
- **Gate B (SIM native-vs-OCCT, booted iOS-17 sim)** —
  `tests/sim/native_curved_wall_cut_parity.mm` (37/37). The SAME bowl-cup operand is
  reconstructed in OCCT (Geom_BezierSurface bowl + planar lid, sewn) and cut by
  `BRepAlgoAPI_Common` against the keep-half box. CUT (Below) at three deflections and
  COMMON (Above) at its robust deflection match OCCT on VOLUME (rel ≤ 9.4%, converging;
  each cross-checked to the closed form to ≤6e-7 / 2.9e-3), AREA (rel ≤ 1.2%),
  WATERTIGHT, Euler χ = 2, BBOX (≤ 1.9e-5), HAUSDORFF (native→OCCT ≤ 3.2e-8 for CUT,
  1.4e-5 for COMMON — the native surface lies ON the OCCT cut). The COMMON rim-weld
  fragility declines to NULL (never a leak).
- Lands the curved-wall freeform CUT/COMMON the M2 roadmap named — the freeform boolean
  whose seam is a closed/circular smooth curve, consuming the B2 smooth-trim enabler.

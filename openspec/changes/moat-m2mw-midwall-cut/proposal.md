# Proposal — moat-m2mw-midwall-cut (MOAT M2 walled-bowl MID-WALL freeform CUT)

## Why

The landed curved-wall freeform CUT verb `curvedWallHalfSpaceCut`
(`src/native/boolean/curved_wall_cut.h`) welds the DOME pose: a bowl-cup whose SOLE
freeform wall is cut by a horizontal plane along a CLOSED interior circular seam, and
whose only other face (the flat top lid) is entirely on the keep side, so NO analytic
face is split and the cross-section cap is a simple DISK bounded only by the seam.

The M2 roadmap named the next blocker as the "walled bowl / dome cut MID-WALL" pose,
where the cut plane crosses the freeform wall (closed interior circle) AND a PLANAR
analytic SIDE WALL (the analytic Split fires). In that pose the cross-section cap is
NOT a disk but an ANNULUS: its OUTER boundary is the wall-section polygon (the split
walls' `Face ∩ P` chords) and its inner boundary is the freeform seam circle as a HOLE.
The landed disk-cap synth (`synthCircularCap`) discards the analytic-face crossings and
builds only a disk, so it cannot close this pose (the wall cut edges are left unbridged
→ NotWatertight). This change lands the annular-cap enabler that closes it.

## What

An ADDITIVE enhancement to `curvedWallHalfSpaceCut` (still OCCT-free, header-only,
`cybercad::native::boolean`). The DOME (disk-cap) path stays behaviourally identical.
When one or more planar analytic faces are SPLIT by the cut plane, the verb now:

1. **records** each split wall's `Face ∩ P` chord (from the byte-frozen
   `hscdetail::cutAnalyticFace`, whose keep sub-face already closes on those crossings);
2. **chains** those chords into the OUTER loop of the flat cap on `P`
   (`hscdetail::orderLoop` + `loopSimple`);
3. **synthesizes an ANNULAR cap** — a planar face whose outer wire is that wall-section
   polygon and whose HOLE wire is the closed freeform seam (the SAME seam nodes the B2
   smooth-trim split laid on the freeform disk sub-face). The cap orientation is pinned
   off the plane's `frame.z` (Forward ⇒ +frame.z, Reversed ⇒ −frame.z) so its outward
   normal faces the DISCARD side DETERMINISTICALLY — independent of the wall-chord loop
   winding — then M0-welded to both the kept wall sub-faces (shared crossing endpoints)
   and the freeform disk sub-face (shared seam nodes).

When no analytic face is split (the dome pose), the byte-identical single-disk cap path
is kept. Everything else — B1 recognise, M1 `traceWallSeam`, B2 `splitFaceSmoothTrim`,
B4 `cutAnalyticFace`, M0 weld + mandatory watertight/positive-volume self-verify, the
typed `CurvedWallCutDecline` honest-decline contract — is unchanged. `KeepSide::Below`
= CUT. ANY decline → NULL Shape (→ OCCT fall-through); NEVER a leaky/partial solid; NO
tolerance widened.

## Impact

- **Additive only.** Only `curved_wall_cut.h` changes (an additive annular-cap branch +
  helper); `smooth_trim_split.h`, `face_split.h`, `half_space_cut.h`, `two_operand.h`,
  `multi_face_weld.h`, `seam_graph.h`, `junction_split.h`, `freeform_operand.h`, M0/M1
  and the whole tessellator are BYTE-IDENTICAL. `include/` diff is EMPTY (no `cc_*` ABI
  change); `src/native/**` stays OCCT-free. The driver's per-function cognitive
  complexity stays in the backend band.
- **Gate A (HOST ANALYTIC, no OCCT)** — `tests/native/test_native_curved_wall_cut.cpp`
  (12/12, the 8 landed dome cases unchanged + 4 new mid-wall cases). On a STEEP (a=2.0)
  Bézier bowl over a convex quad + 4 PLANAR side walls + a flat base, cut by the
  horizontal plane `z = c` (c strictly below the interior-circle bound `a·d_e²`, so the
  seam is a closed circle interior to the quad AND each wall is genuinely split): the
  operand B1-admits (1 freeform + 5 analytic); the CUT (Below) yields a 7-face solid
  (disk + 4 wall trapezoids + base + ANNULAR cap) that welds WATERTIGHT (Euler χ = 2)
  and CONVERGES monotonically to the closed form `(H0+c)·A_Q − c·π·ρ²/2` across a
  deflection sweep (rel 0.62% → 0.10%); the removed cap volume matches the bowl-cap
  closed form to < 5%; a non-cutting plane DECLINES to NULL.
- **Gate B (SIM native-vs-OCCT, booted iOS sim)** —
  `tests/sim/native_walled_bowl_midwall_parity.mm` + `scripts/run-sim-native-walled-bowl-midwall.sh`
  (28/28). The SAME operand is reconstructed in OCCT (Geom_BezierSurface bowl over the
  convex quad + 4 planar walls + planar base, sewn, outward-oriented) and cut by
  `BRepAlgoAPI_Cut` against the keep-half box. The OCCT cut matches the closed form to
  rel 1.3e-07. The native CUT at three deflections matches OCCT on VOLUME (rel ≤ 6.2e-3,
  converging), AREA (rel ≤ 2.8e-3), WATERTIGHT, Euler χ = 2, annular 7-face topology,
  BBOX (≤ 1.0e-7), HAUSDORFF (native→OCCT ≤ 6.0e-8 — the native surface lies ON the OCCT
  cut).
- Lands the walled-bowl MID-WALL freeform CUT the M2 roadmap named — the annular-cap
  weld that composes the analytic-wall split with the closed-circular smooth-trim split.

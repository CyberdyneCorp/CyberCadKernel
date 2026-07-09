# Design — moat-m2cw-curved-wall-cut

## Context

`freeformHalfSpaceCut` (B4, `half_space_cut.h`) welds a freeform↔analytic half-space CUT
when `wall ∩ P` is an OPEN chord (B2 `splitFace`, `crossings == 2`). The curved-wall pose
— a dome/bowl cut by a HORIZONTAL plane — has a CLOSED CIRCULAR `wall ∩ P` seam interior
to the freeform wall (`crossings == 0`), which B2 `splitFace` DECLINES and the landed B2
smooth-trim sibling `splitFaceSmoothTrim` resolves into a disk `faceInside` + an annulus
`faceOutside` (the seam as a hole). This design composes that split into a watertight
result.

## The reachable operand: a bowl-cup

`{ (x,y,z) : x²+y² ≤ R², a·(x²+y²) ≤ z ≤ a·R² }`:

- **bowl (freeform)** — the separable-quadratic degree-2 Bézier `z = a·(x²+y²)`
  (`x = u−½`, `y = v−½`), the family the B2 / smooth-trim fixtures use, at a STEEP
  amplitude `a = 2.0` so the cut leaves two well-conditioned pieces (not a sliver).
  Genuinely trimmed by the rim CIRCLE (radius R in the bowl's (u,v)).
- **top lid (analytic Plane)** — the flat disk on `z = a·R²`, bounded by the same rim.

The bowl and lid SHARE the rim edges (built once, laid on both via `addPCurve` — the
substrate's watertight-share idiom), so the operand is a watertight closed shell. Its
sole freeform wall is cut by `z = c` (`0 < c < a·R²`) along a CLOSED interior circle of
radius `ρ = √(c/a)`.

Closed forms (no OCCT): `V(full) = π·a·R⁴/2`, `V(z≤c) = π·ρ²·c/2`, `V(z≥c) = V(full) −
V(z≤c)` — with `V(z≤c) + V(z≥c) = V(full)` exact.

## The weld

`curvedWallHalfSpaceCut(operand, P, side, deflection)`:

1. recognise[B1] — exactly one freeform Bézier wall; else typed decline → NULL.
2. trace[M1] — the byte-unchanged `traceWallSeam` returns the `Closed` circular WLine.
3. split[B2 smooth-trim] — `splitFaceSmoothTrim` → `faceInside` (disk) + `faceOutside`
   (annulus, seam-hole). A non-closed / boundary-crossing seam is not the curved-wall
   pose → `SeamUnusable`.
4. keep-side pick — the freeform sub-face whose trim centroid is on `side` of `P`. For
   Below (CUT) the disk; for Above (COMMON) the annulus.
5. wall split[B4] — `cutAnalyticFace` splits every crossed planar analytic face; whole
   faces on one side are kept/dropped. (For the reachable dome pose the seam is interior
   to the freeform wall and the flat lid is entirely on one side, so no Split fires; the
   machinery is kept so a mid-wall cut of a walled bowl still routes here.)
6. circular cap synth — ONE flat planar cap on `P` from the seam polyline (the SAME
   straight chords `splitFaceSmoothTrim` laid on the freeform sub-face), normal to the
   discard side. Straight cap chords + straight disk-seam chords between the SAME 3-D
   nodes → the M0 EdgeCache endpoint-keyed path pins both to identical samples.
7. weld + self-verify — Shell → Solid; M0 mesh must be WATERTIGHT with a positive
   enclosed volume, else NULL.

## The watertight-weld tension (the measured next blocker)

The flat cap and the freeform sub-face share the CLOSED seam bit-for-bit (straight
chords, endpoint-keyed). The remaining hazard is INTERIOR mesh vertices of a curved
sub-face landing within the M0 weld tolerance (`deflection/2`) of a shared boundary
vertex, producing a non-manifold edge at certain deflections — the same weld-cell-
boundary resonance the roadmap's M0-weld fix removed for the OPEN seam.

Measured outcome:

- **CUT (Below) — LANDED, robust.** The disk+cap keep side (a simple cup) welds
  watertight and CONVERGES monotonically to `π·ρ²·c/2` across a resonance-free deflection
  sweep {0.0102, 0.00737, 0.00532, 0.00385, 0.00278} (rel 9.4% → 2.7%; the coarse band is
  the curved cup, not a leak). Sim native-vs-OCCT parity at three deflections.
- **COMMON (Above) — landed at its robust deflection, fragile elsewhere.** The annulus
  reuses the parent rim wire AND carries the seam hole; its interior mesh resonates with
  the lid's rim samples, so COMMON welds watertight only at isolated deflections (robust
  at 0.0102, rel 2.0%). Away from the robust band the verb honestly DECLINES to NULL
  (`NotWatertight`) — NEVER a leaky/partial solid.

The self-verify is the arbiter: it returns a valid solid ONLY when the M0 mesh is
watertight at the requested deflection, else NULL → OCCT. No tolerance is weakened to
force a pass; the fragility is a MEASURED, asserted decline.

## The sharpened next blocker

Give the flat cap's boundary and the reused annulus/lid rim ONE canonical shared
discretization that also FENCES interior mesh vertices away from the shared seam within
the weld tolerance (the closed-seam analogue of the M0-weld open-seam fix), so COMMON
welds watertight at EVERY deflection like CUT does — then a walled bowl/dome cut mid-wall
(analytic Split fires), and a freeform↔freeform closed-seam weld.

## Alternatives considered

- **Share the disk's seam edge OBJECTS with the cap (add a plane pcurve).** `addPCurve`
  clones the edge node (new TShape identity), and the reused edges' parameter ranges do
  not line up with a fresh plane pcurve — the cap meshed as a unit square. Rejected;
  the straight-chord endpoint-keyed path is the robust sharing.
- **A shallow bowl (a = 0.4).** The CUT-below piece is a sliver (height 0.016) tangent to
  the cut plane over a band → severe weld resonance. The steep a = 2.0 cup fixes it.

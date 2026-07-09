# Proposal — moat-m0w-closed-seam-weld (MOAT M0 tessellator weld for the closed-inner-seam annulus)

## Why

The byte-frozen M0 tessellator welds a shared CLOSED seam between two sub-faces —
a curved annulus-with-hole ∪ a flat disk/cap on the cut plane, the pair produced by
`curved_wall_cut.h` + `smooth_trim_split.h` — watertight only at ISOLATED deflections.
The curved-wall COMMON (`KeepSide::Above`) case therefore honestly DECLINES elsewhere
(returns NULL → OCCT) via the asserted-decline test
`curved_wall_common_rim_weld_fragility_is_measured_decline`. This is the exact
"sharpened next blocker" the landed `moat-m2cw-curved-wall-cut` design named: give the
closed seam ONE canonical shared discretization that fences interior mesh vertices away
from the shared seam, so COMMON welds watertight at EVERY deflection like CUT does.

Two regimes both fail, for the same root cause:

- **COARSE** — the solid-mesher weld tolerance (`0.5·deflection`) merges adjacent
  same-loop seam vertices on the DENSE seam (399 nodes around a small circle), collapsing
  the seam into a non-manifold fold (an edge used by more than two triangles).
- **FINE** — the seam edges are degree-1 straight 3D chords carrying a pcurve on the
  CURVED bowl surface. When the shared per-edge discretization subdivides a chord
  (`n > 1` samples), the annulus places its interior seam-boundary vertices by evaluating
  the BOWL SURFACE at the pcurve UV — which BULGES off the straight chord — while the flat
  cap places them exactly on the chord. The two diverge (measured: only 399 of ~4788
  near-seam vertices coincide at deflection `0.004`), so the shared closed boundary does
  not weld watertight (thousands of open edges each used by one triangle).

ROOT CAUSE: the seam edge's canonical geometry is the STRAIGHT degree-1 chord (its
`EdgeDiscretization` `d.points`), but the mesher re-evaluates boundary vertices through
each face's OWN surface (`S_face(pcurve(t))`) rather than pinning them to the shared
edge's canonical 3D points. The existing `BoundaryAnchors` snap fires only within
`kSnapEps = 1e-6`, far tighter than the bowl bulge, so it never snaps the subdivided
interior samples.

## What

A single guarded, additive, OCCT-free path in the M0 tessellator
(`src/native/tessellate/face_mesher.h`, and if needed `edge_mesher.h`): a shared CLOSED
seam SHALL carry ONE canonical shared discretization (the edge's `d.points`), consumed by
BOTH sub-faces with OPPOSITE orientation, and boundary vertices that come from such a seam
edge SHALL be PINNED EXACTLY to the seam edge's canonical 3D sample points by
parametric/UV correspondence (NOT spatial-proximity snap), so the two sub-faces are
bit-identical along the seam and the CDT does not densify across or fuse through it.

The new path SHALL fire ONLY for the closed-seam topology — a genuinely-curved-surface
face whose boundary edge is a straight degree-1 chord seam (a 2-pole degree-1 curve, the
exact shape `smooth_trim_split` / `curved_wall_cut` lay as one segment of the closed
interior seam) — a guarded, opt-in-by-topology branch. Every other surface kind and edge
(analytic primitives, genuinely-curved shared edges like a cylinder cap↔side circle, and
straight `Line` edges) is untouched, so every existing mesh is BYTE-IDENTICAL.

With the seam pinned, the closed-inner-seam weld welds WATERTIGHT across the full
deflection ladder for both cap poses that carry it: the dome CUT (disk ∪ flat disk-cap)
and the mid-wall CUT (disk ∪ ANNULAR cap whose inner hole is the seam). Honest decline is
preserved: a case that STILL cannot weld returns non-watertight → NULL → OCCT, never a
leaky solid.

SCOPE (honest): the curved-wall COMMON (`KeepSide::Above`) additionally reuses the
freeform bowl's OUTER curved RIM welded to the flat top lid. That rim weld is a SEPARATE,
pre-existing curved-edge fragility (a free-form face subdivides the shared curved rim
beyond a flat neighbour's need; plus a coarse-regime near-degenerate triangulation sliver)
— NOT the closed inner seam this change fixes. COMMON therefore still honestly declines at
the fine end of its ladder; that decline is now attributable SOLELY to the curved-rim
blocker (the closed seam no longer contributes), and remains a first-class NULL → OCCT
outcome (never a leak). The asserted-decline test
`curved_wall_common_rim_weld_fragility_is_measured_decline` is kept.

## Impact

- **Additive, topology-guarded, byte-identical.** The pinning fires ONLY for the
  closed-seam topology; `face_mesher.h`'s existing three mesh arms, the boundary
  flattener, the segment-count sizing, the curve evaluators, the spatial weld, and every
  other surface kind's path are BYTE-IDENTICAL. `curved_wall_cut.h`,
  `smooth_trim_split.h`, `half_space_cut.h`, and the whole boolean substrate are unchanged.
  `src/native/**` stays OCCT-free; the `cc_*` ABI is additive-only (no signature or POD
  layout change).
- **Acceptance = a byte-identical battery.** A FNV hash over `{vertices, triangles,
  watertight, area, volume}` SHALL be IDENTICAL before vs after for EVERY existing surface
  kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Bezier`, `BSpline`, curved seams,
  box / holed / loft / sweep / thread / step / revolve). The ONLY meshes allowed to change
  are the previously-failing closed-seam annulus cases (non-watertight → watertight).
- **Gate A (HOST ANALYTIC, no OCCT)** — the mid-wall ANNULAR-cap CUT (disk ∪ annulus whose
  inner hole is the closed seam) welds watertight (Euler `χ = 2`) and converges to the
  closed-form cap volume across its full ladder `{0.02, 0.012, 0.008, 0.005, 0.0025}`
  (0.62% → 0.10%); the dome CUT closed seam welds watertight at the fine `d = 0.004` that
  DECLINED before the pin — no OCCT. Full host suite `ctest`: 58/58. A FNV mesh-hash battery
  over box / bump-capped cylinder (curved seam) / rational-BSpline cap / bowl operand /
  mid-wall operand × 6 deflections is BYTE-IDENTICAL (36/36) before vs after.
- **Gate B (SIM native-vs-OCCT parity, booted iOS simulator)** — the curved-wall
  CUT/COMMON parity harness matches OCCT `BRepAlgoAPI_Common` + `BRepGProp` at every
  asserted deflection (45/45), INCLUDING the newly-added fine `CUT d = 0.004` (native
  watertight, volume rel 4.0%, area 0.5%, one-sided Hausdorff 3.4e-8) that declined
  pre-pin; the COMMON fine-deflection decline remains an honest NULL → OCCT.
- **Honest decline preserved; no global tolerance widened.** A case that still can't weld
  returns non-watertight → NULL → OCCT. No global weld/snap tolerance is widened; the fence
  is topology-scoped canonical pinning, not a loosened threshold.
- Lands the closed-inner-seam half of the sharpened next blocker of
  `moat-m2cw-curved-wall-cut`: the seam is no longer a weld blocker at any deflection; the
  remaining COMMON fine-deflection decline is now isolated to the pre-existing curved-RIM
  weld (a first-class documented blocker), not the seam.

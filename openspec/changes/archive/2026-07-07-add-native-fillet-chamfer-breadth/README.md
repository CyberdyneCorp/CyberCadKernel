# add-native-fillet-chamfer-breadth

Feature #6 CURVED BLENDS — the **last blend-breadth batch**: widen native FILLET and
CHAMFER off the CIRCULAR crease onto the ASYMMETRIC and NON-CIRCULAR creases, honest
per-track. Today native does planar fillet/chamfer/offset/shell; the CONVEX/CONCAVE +
variable-radius circular FILLET (a G1-tangent torus canal); and the SYMMETRIC circular
CHAMFER (a C0 cone-frustum bevel). This change adds three tracks, each with its own
honest gate (build watertight with the right continuity and correct volume, or return
NULL → OCCT and REPORT the measured gap):

- **T1 — ASYMMETRIC two-distance chamfer** (highest confidence). Extends the symmetric
  cone-frustum chamfer on a convex circular cylinder↔cap rim to `d1 ≠ d2`: the wall is
  set back AXIALLY by `d1`, the cap RADIALLY by `d2`, so the two setback circles sit at
  DIFFERENT setbacks and the bevel is an **OBLIQUE CONE FRUSTUM**. Still C0 (a straight
  bevel) but the two seam bevel-angles differ (`cos∠_cyl = d1/√(d1²+d2²)`,
  `cos∠_cap = d2/√(d1²+d2²)`, neither `1`). The removed corner is the right triangle
  legs `d1×d2`, exact removed volume `π·d1·d2·(Rc − d2/3)` (Pappus). `cc_chamfer_edges`
  carries only ONE distance, so this lands behind a NEW ADDITIVE facade entry
  `cc_chamfer_edges_asym(body, edgeIds, edgeCount, distance1, distance2)` — the existing
  `cc_chamfer_edges` is byte-unchanged.

- **T2 — NON-CIRCULAR-CREASE (elliptical) fillet** (medium confidence, narrow slice). A
  CONSTANT-radius rolling-ball fillet on a NON-circular curved edge: the ELLIPTICAL rim
  where a cylinder meets an OBLIQUE plane. The crease, the ball-centre spine, and both
  ball-contact curves are all ELLIPSES (plane∩cylinder conics the native SSI S1
  `plane_conics` handler already computes in closed form: semi-minor `Rc`, semi-major
  `Rc/sinθ`). The rolling-ball canal is a GENERAL (non-torus) canal surface — the
  envelope of constant-`r` spheres centred on the spine ellipse, swept `r`-circles in the
  planes normal to the spine tangent, G1 to the cylinder and the plane at the two contact
  ellipses. Behind the UNCHANGED `cc_fillet_edges`. Declines (NULL → OCCT) when the plane
  is not oblique-through-the-cylinder, when `r` reaches the crease's tightest curvature
  radius (`≈ Rc·sinθ`, the ellipse self-intersects the canal), or on any non-elliptical
  crease.

- **T3 — CYL↔CYL-CANAL fillet** (lowest confidence — a NARROW slice OR an honest
  decline). A constant-radius fillet on the CURVED↔CURVED crease between two intersecting
  cylinders: the crease is a GENERAL space curve (the SSI marching curve; e.g. two equal
  orthogonal cylinders → the Steinmetz curve), the spine is another general SSI curve
  (offset-cyl ∩ offset-cyl), and the canal is swept `r`-circles along it. This is the
  hardest track. It is attempted ONLY for the narrowest robust slice (equal-radius
  PERPENDICULAR cylinders, `r` safely below the crease curvature) and RETAINED only if it
  self-verifies watertight + G1 + correct SHRINK volume on its fixture; otherwise the
  track is an HONEST DECLINE — cyl↔cyl fillet stays OCCT-fallthrough, documented, the
  measured gap REPORTED, with NO always-NULL dead builder retained. Uses the SSI
  substrate (`CYBERCAD_HAS_NUMSCI`-gated).

The native paths land behind the unchanged `cc_fillet_edges` / `cc_chamfer_edges` ABI
plus the new additive `cc_chamfer_edges_asym`. Every candidate is accepted ONLY through
the engine's mandatory watertight + sane-volume self-verify
(`blendResultVerified(..., wantGrow=false)` — all three tracks REMOVE material), so a
NULL builder result or a failed self-verify DISCARDS the candidate and the engine reports
honestly for the OCCT `BRepFilletAPI` oracle. `src/native/**` stays OCCT-FREE; OCCT is the
verification ORACLE only. Nothing outside the named slices is faked — the honest-out
returns NULL → OCCT and REPORTS the gap, and where a track (T3) is not robustly tractable
it declines WITHOUT dead code.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#6 curved blends),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 plane∩cylinder ellipse seams +
S3 marching cyl∩cyl → #6 blends),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md),
[../archive/2026-07-06-add-native-curved-chamfer](../archive/2026-07-06-add-native-curved-chamfer)
(the SYMMETRIC circular chamfer this change extends to `d1 ≠ d2` for T1, and whose change
shape this one mirrors),
[../archive/2026-07-05-add-native-curved-fillet](../archive/2026-07-05-add-native-curved-fillet)
+ [../archive/2026-07-06-add-native-variable-fillet](../archive/2026-07-06-add-native-variable-fillet)
(the circular / variable FILLET canal this change generalizes to non-circular creases for
T2/T3), and the SSI `src/native/ssi/plane_conics.h` (the plane∩cylinder ellipse crease
for T2) + `src/native/ssi/marching.{h,cpp}` (the cyl∩cyl crease curve for T3).

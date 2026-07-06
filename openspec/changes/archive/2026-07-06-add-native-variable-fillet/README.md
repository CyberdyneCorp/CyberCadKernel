# add-native-variable-fillet

Feature #6 CURVED BLENDS — **variable-radius circular-rim slice**. Extends the native
CURVED-fillet family from the CONSTANT-radius rolling-ball fillet on a CONVEX circular
crease (a cylinder LATERAL face meeting a coaxial PLANAR cap → a coaxial TORUS canal
that REMOVES material) to the first **VARIABLE-radius** blend: a rolling-ball fillet on
the SAME convex circular cylinder↔cap rim whose radius varies LINEARLY around the rim
from `r1` at one angular end to `r2` at the other
(`r(θ) = r1 + (r2 − r1)·θ/2π`, `θ ∈ [0, 2π)`).

Because the ball radius now depends on the rim angle, the rolling-ball CENTRE is no
longer a circle at a fixed offset — it is a **SWEPT curve** (radius `Rc − r(θ)`, axial
`H − s·r(θ)`), and the blend surface is a **variable-radius canal** (a swept-circle
patch: at each angular STATION `θ` a meridian arc of the local radius `r(θ)` tangent to
both faces). The two trim seams are NO LONGER circles: the cylinder seam is a curve at
radius `Rc` with VARYING axial height `H − s·r(θ)`, and the cap seam is a curve at
VARYING radius `Rc − r(θ)` in the plane `z = H`. The blend is **G1-tangent to both
faces at every station** — a property that holds EXACTLY for any linear radius law,
because at the cylinder seam `∂radius/∂v = 0` (the station tangent is axial) and at the
cap seam `∂axial/∂v = 0` (the station tangent is radial), so the swept-surface normal
is radial at the wall and axial at the cap regardless of `r'(θ)`. A convex variable
fillet REMOVES material, so the solid's volume DECREASES (`0 < Vr < Vo`).

The native path lands behind the unchanged `cc_fillet_edges_variable` ABI. The engine
runs its mandatory watertight + correctly-signed-volume self-verify
(`blendResultVerified(..., wantGrow=false)`); a NULL builder result or a failed
self-verify DISCARDS the native candidate and the engine reports honestly so the call
is served by the OCCT `BRepFilletAPI_MakeFillet` (variable / evolved-law) oracle.
Everything outside the named slice — a NON-linear radius law, a NON-circular crease, a
cylinder↔cylinder canal, a CONCAVE variable rim, a tilted/non-coaxial plane, a freeform
face, or a gradient so steep the swept patch leaves the curved-parity tolerance —
returns NULL → OCCT, never faked; the measured OCCT-fallback gap is REPORTED.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#6 curved blends),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 circle seams → #6 blends),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md),
[../archive/2026-07-05-add-native-curved-fillet](../archive/2026-07-05-add-native-curved-fillet)
(the constant-radius convex sibling this change generalises),
[../archive/2026-07-06-add-native-concave-fillet](../archive/2026-07-06-add-native-concave-fillet)
(the concave sibling — the per-crease self-verify sign pattern this change reuses).

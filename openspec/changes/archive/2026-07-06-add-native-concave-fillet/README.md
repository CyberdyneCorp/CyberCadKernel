# add-native-concave-fillet

Feature #6 CURVED BLENDS — **concave circular-rim slice**. Extends the native
`fillet_edges` CURVED path from the CONVEX circular crease (a cylinder LATERAL face
meeting a coaxial PLANAR cap in a CONVEX dihedral → torus canal that REMOVES material)
to the everyday INSIDE-fillet case: a **constant-radius rolling-ball fillet on a
CONCAVE circular edge** — where a cylinder (a boss) meets a LARGER planar face in a
CONCAVE dihedral (the base rim of a boss standing on a plate; the bottom rim of a
blind hole).

For a concave circular crease the rolling ball seats on the MATERIAL side of the
dihedral, so its centre locus is a CIRCLE of radius `Rc + r` (the offset sign FLIPS vs
the convex `Rc − r`) at axial `H + r` (into the material). Revolving the ball
cross-section about the axis still sweeps a coaxial **TORUS** (major radius `Rc + r`,
minor radius `r`); the two trim seams where the torus meets the cylinder and the
larger plane are still **CIRCLES** (torus∩cylinder at radius `Rc`, torus∩plane at
radius `Rc + r` — analytic, native SSI S1). The fillet ADDS material, so the solid's
volume INCREASES (Vr > Vo), G1-tangent to both faces.

The native path lands behind the unchanged `cc_fillet_edges` ABI. Because a concave
fillet GROWS volume while a convex one SHRINKS it, the engine self-verify picks the
sign per case: convex → `0 < Vr < Vo`, concave → `Vr > Vo` (mirroring the offset-face
grow guard). A NULL builder result or a failed self-verify DISCARDS the native
candidate and falls through to the OCCT `BRepFilletAPI_MakeFillet` oracle. Everything
outside the named slice (non-circular / variable / cyl↔cyl-canal / tilted / freeform
creases) returns NULL → OCCT — never faked.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#6 curved blends),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 circle seams → #6 blends),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md),
[../archive/2026-07-05-add-native-curved-fillet](../archive/2026-07-05-add-native-curved-fillet)
(the convex sibling this change extends).

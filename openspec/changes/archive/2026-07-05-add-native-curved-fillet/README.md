# add-native-curved-fillet

Feature #6 CURVED BLENDS — **first native slice**. Extends the planar native
`fillet_edges` (rolling-ball tangent-CYLINDER on a STRAIGHT convex dihedral) to the
FIRST curved crease: a **constant-radius rolling-ball fillet on a CIRCULAR edge** —
the rim where a cylinder LATERAL face meets a PLANAR CAP.

For a circular crease the rolling-ball canal surface is a **TORUS** (major radius =
the crease-circle radius offset to the ball-centre locus, minor radius = the fillet
radius `r`); the two trim seams where the torus meets the cylinder and the cap are
**CIRCLES** (torus∩cylinder and torus∩plane — analytic, native SSI S1). The builder
trims both faces back to their tangent circles, inserts the torus blend patch,
G1-tangent to both, and welds watertight via the boolean `assembleSolid` path — the
same discipline the planar fillet uses.

The native path lands behind the unchanged `cc_fillet_edges` ABI; the engine runs
the MANDATORY watertight + volume-reducing self-verify and DISCARDS a bad native
result, falling through to the OCCT `BRepFilletAPI_MakeFillet` oracle. Everything
outside the named slice (concave rims, variable radius, cyl↔cyl canal fillets,
non-circular curved creases, freeform) returns NULL → OCCT — never faked.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#6 curved blends),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 circle seams → #6 blends),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md).

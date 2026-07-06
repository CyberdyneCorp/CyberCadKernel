# add-native-curved-chamfer

Feature #6 CURVED BLENDS — **curved (circular-rim) chamfer slice**. Extends the native
CHAMFER family from the PLANAR-dihedral corner (a convex edge between two PLANAR faces,
sliced by the single plane through the two setback lines) to the first CURVED chamfer:
a symmetric-distance chamfer on a CONVEX CIRCULAR rim — the crease where a cylinder
LATERAL face meets a coaxial PLANAR cap.

A chamfer is NOT a fillet. Where the constant-radius curved FILLET rolls a ball into the
corner and inserts a G1-tangent quarter-TORUS arc (curved, tangent to both faces), a
CHAMFER cuts a FLAT BEVEL: for a circular rim it is a **CONE-FRUSTUM band** between the
two setback circles —

- the **cylinder seam** circle at radius `Rc`, axial `H − s·d` (the wall set back
  AXIALLY by the chamfer distance `d`); and
- the **cap seam** circle at radius `Rc − d`, axial `H` (the cap set back RADIALLY by
  `d`).

The bevel meets each face at the setback line at the chamfer angle and is **C0, NOT G1**
— a straight bevel, not tangent. For a SYMMETRIC distance the frustum slant makes 45°
with the cylinder wall and 45° with the cap, so the frustum outward normal is the exact
bisector of the two face normals (`cos = 1/√2 ≈ 0.7071` to each, NOT `cos = 1`). The
chamfer REMOVES material, so the solid's volume DECREASES; the removed region is the
sharp-corner ring above the bevel — the right-triangle corner (legs `d × d`) revolved
about the axis — of exact closed-form volume `V_removed = π·d²·(Rc − d/3)` (Pappus).

The native path lands behind the unchanged `cc_chamfer_edges` ABI. The engine dispatch
becomes planar-chamfer → curved-circular-chamfer → OCCT, gated by the SAME watertight +
volume-SHRINK self-verify (`blendResultVerified(..., wantGrow=false)`) the planar
chamfer already uses. A NULL builder result or a failed self-verify DISCARDS the native
candidate and the engine reports honestly so the call is served by the OCCT
`BRepFilletAPI_MakeChamfer` (`Add(distance, edge)`) oracle. Everything outside the named
slice — an ASYMMETRIC two-distance chamfer, a NON-circular curved crease (cone↔plane /
sphere / ellipse / spline), a CONCAVE rim, a curved↔curved (cylinder↔cylinder) rim, a
tilted / non-coaxial plane, a freeform face, `Rc ≤ d` (the cap circle collapses), or a
multi-edge selection — returns NULL → OCCT, never faked; the measured OCCT-fallback gap
is REPORTED.

Because a symmetric chamfer on a circular rim IS EXACTLY a cone frustum (the bevel is
the exact chamfer surface, not an approximation of one), the native↔OCCT parity is TIGHT
(bounded only by the angular tessellation deflection) — unlike the variable fillet's
`O(r')` interior gap. This is an honest strength of the slice, not a loosened tolerance.

Reference: [../../ROADMAP.md](../../ROADMAP.md) (#6 curved blends),
[../../SSI-ROADMAP.md](../../SSI-ROADMAP.md) (S1 circle seams → #6 blends),
[../../NATIVE-REWRITE.md](../../NATIVE-REWRITE.md),
[../archive/2026-07-05-add-native-curved-fillet](../archive/2026-07-05-add-native-curved-fillet)
(the constant-radius curved FILLET on the SAME rim — this change reuses its rim
recognition + planar-facet weld, but replaces the torus canal with a flat cone frustum),
[../archive/2026-07-06-add-native-variable-fillet](../archive/2026-07-06-add-native-variable-fillet)
(the change shape this one mirrors), and the PLANAR chamfer in
`src/native/blend/chamfer_edges.h` (the setback-plane corner slice this change is the
curved-rim sibling of).

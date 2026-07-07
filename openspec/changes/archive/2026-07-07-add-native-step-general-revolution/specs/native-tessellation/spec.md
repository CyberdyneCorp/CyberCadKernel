# native-tessellation

This change meshes a native **rational** `FaceSurface::Kind::BSpline` face that a STEP
`SURFACE_OF_REVOLUTION` (ellipse / non-rational B-spline profile) reconstructs — the exact revolved
rational tensor-product B-spline (see the `native-exchange` delta). The pivotal substrate already exists:
`surface_eval.h` dispatches `Kind::BSpline` with non-empty `weights` to the RATIONAL evaluators
(`math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs`), so a rational B-spline surface EVALUATES through
the current, unmodified mesh path. This delta therefore imposes a **no-perturbation guarantee**: a revolved
rational B-spline face SHALL mesh watertight through the EXISTING freeform path with NO tessellator change,
OR — only if a `u`-periodic-seam / axis-pole close is genuinely required — through a STRICTLY ADDITIVE
branch PROVEN byte-identical for every existing mesh; otherwise the STEP reader keeps the honest OCCT
decline. The tessellator has been kept byte-stable because a mesher change is high-blast-radius; this delta
preserves that discipline.

## ADDED Requirements

### Requirement: Mesh a rational revolved B-spline face watertight through the existing path, or keep the honest decline

The library SHALL mesh a native `Face` whose surface is a **rational** `Kind::BSpline` (non-empty
`weights`) — specifically the revolved rational tensor-product B-spline a `SURFACE_OF_REVOLUTION` of an
`ELLIPSE` / non-rational `B_SPLINE_CURVE_WITH_KNOTS` profile reconstructs, `u`-periodic (seam at `u=0≡2π`,
`degreeU = 2`, the standard revolution weights `{1,1/√2,1,1/√2,1,1/√2,1,1/√2,1}` and knots
`{0,0,0,π/2,π/2,π,π,3π/2,3π/2,2π,2π,2π}`) tensored with the profile in `v` — to a triangle mesh at a
requested deflection, evaluating the surface through the EXISTING rational evaluators
(`math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs`), respecting the deflection bound exactly as for a
non-rational B-spline face. The mesh SHALL be produced through the EXISTING freeform B-spline mesh path with
**no modification** to the `Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `BSpline`, or `Bezier` mesh
paths. If welding the `u=0≡2π` seam and closing a profile-endpoint axis pole watertight requires more than
the existing freeform path provides, the additional close SHALL be added ONLY as a **new, additive** guarded
branch that leaves every existing face meshing **byte-identically** (the same triangle counts, watertight
status, and enclosed volumes), PROVEN across the full tessellation-sensitive suite (`run-sim-suite`,
curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3). If a watertight mesh of the revolved
rational B-spline face cannot be achieved without either perturbing an existing mesh path or fabricating a
seam/pole, the tessellator SHALL NOT be changed and the STEP reader (`native-exchange`) SHALL keep the
honest OCCT decline for that revolution (an OCCT-imported revolution loses nothing). The library SHALL
remain OCCT-free and host-buildable, and no tolerance SHALL be weakened.

#### Scenario: A rational revolved B-spline face meshes watertight through the existing path (host)
- GIVEN a native rational `Kind::BSpline` face reconstructed from an ellipse-profile `SURFACE_OF_REVOLUTION` (a spheroid of revolution, `u`-periodic, with axis poles at the profile endpoints) built on the host with no OCCT and a requested deflection `d`
- WHEN it is meshed through the existing rational B-spline mesh path
- THEN every triangle's chord-height deviation from the true surface SHALL be at or below `d`, and — when the `u`-seam welds and the axis poles close — the mesh SHALL be watertight and the enclosed volume SHALL converge to the analytic revolved-solid volume within the deflection tolerance; if it does NOT close watertight, the STEP reader SHALL DECLINE (NULL → OCCT) rather than emit a leaky mesh

#### Scenario: The general-revolution mapping leaves every existing kind's mesh byte-identical (host + sim)
- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Torus`, `BSpline`, `Bezier`) meshed before and after this change, and the full tessellation-sensitive sim suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to the baseline (the tessellator is preferred untouched; any added branch is additive and touches no existing mesh path); if ANY differs, the mesher change SHALL be reverted and the revolution SHALL keep the OCCT decline

#### Scenario: A revolved B-spline that cannot mesh watertight without perturbation keeps the honest decline (host)
- GIVEN a native rational `Kind::BSpline` revolved face whose `u`-seam or axis pole cannot be closed watertight without either modifying an existing mesh path or fabricating a seam/pole
- WHEN the watertight self-verify is evaluated
- THEN the tessellator SHALL NOT be changed, the STEP reader SHALL DECLINE the revolution (NULL → OCCT), and no existing tessellation SHALL have been perturbed and no tolerance weakened — the honest deferral is reported, not faked

# native-tessellation

This change adds an **additive** torus mesh path to the face mesher so a `FaceSurface` of the new
`Kind::Torus` (see the `native-topology` delta) can be tessellated watertight, WITHOUT perturbing any
existing surface-kind mesh path. The tessellator has been kept byte-stable because a mesher change is
high-blast-radius; therefore this delta requires the torus path to be a NEW branch proven byte-identical
for every existing mesh, and mandates the honest-out (keep the OCCT torus decline) if that cannot be met.

## ADDED Requirements

### Requirement: Mesh a native Torus face watertight via an additive mesh path proven byte-identical for existing kinds

The library SHALL mesh a native `Face` whose surface is of kind `Torus` (a doubly-periodic ring torus,
`u∈[0,2π]` the major/revolution angle and `v∈[0,2π]` the minor/tube angle, evaluated through the
`native-math` `Torus` `value` / `dU` / `dV` / `normal`) to a triangle mesh at a requested deflection,
respecting the deflection bound exactly as for the other analytic-curved kinds (cylinder / cone / sphere).
The torus SHALL be meshed through a **new, additive** mesh branch that reuses the EXISTING
periodic-analytic grid and canonical-seam-anchor machinery to weld BOTH the `u=0≡2π` seam and the
`v=0≡2π` seam (a ring torus has NO degenerate pole, so the seam weld is strictly simpler than the sphere's
pole-plus-seam case). The addition SHALL NOT modify the `Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`,
or `Bezier` mesh paths: every existing face SHALL mesh **byte-identically** — the same triangle counts, the
same watertight status, and the same enclosed volumes — as before this change, PROVEN across the full
tessellation-sensitive suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss,
phase3). If a clean additive torus path that keeps every existing mesh byte-identical AND meshes the torus
watertight cannot be achieved, the torus mesh path SHALL be reverted and the STEP reader SHALL keep the
honest OCCT torus decline (an OCCT-imported torus loses nothing). The library SHALL remain OCCT-free and
host-buildable, and no tolerance SHALL be weakened.

#### Scenario: A torus face meshes watertight within the deflection bound (host)
- GIVEN a native `Torus` face (major radius `R`, minor radius `r`, full period in both `u` and `v`) built on the host with no OCCT and a requested deflection `d`
- WHEN it is meshed
- THEN every triangle's chord-height deviation from the true torus SHALL be at or below `d`, the mesh SHALL be watertight (both seams welded, no pole), AND the enclosed volume SHALL converge to the analytic torus volume `2·π²·R·r²` within the deflection tolerance

#### Scenario: The additive torus path leaves every existing kind's mesh byte-identical (host + sim)
- GIVEN faces of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `BSpline`, `Bezier`) meshed before and after the torus mesh branch is added, and the full tessellation-sensitive sim suite (`run-sim-suite`, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, phase3)
- WHEN each existing face / suite is meshed at the same deflection and compared against the pre-change baseline
- THEN the triangle counts, watertight status, and enclosed volumes SHALL be IDENTICAL to the baseline (the torus branch is additive and touches no existing mesh path); if ANY differs, the torus mesh path SHALL be reverted and the torus SHALL keep the OCCT decline

#### Scenario: A torus that cannot mesh watertight additively keeps the honest OCCT decline (host)
- GIVEN a native `Torus` face whose additive mesh path cannot both weld its seams watertight AND leave every existing mesh byte-identical
- WHEN the tessellation zero-regression proof is evaluated
- THEN the torus mesh path SHALL be reverted, the STEP reader SHALL DECLINE the torus (NULL → OCCT), and no existing tessellation SHALL have been perturbed and no tolerance weakened — the honest deferral is reported, not faked

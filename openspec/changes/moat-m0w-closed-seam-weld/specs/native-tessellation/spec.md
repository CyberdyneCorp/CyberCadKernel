# native-tessellation

## ADDED Requirements

### Requirement: Weld a shared CLOSED inner seam between a curved sub-face and a flat cap watertight at any deflection via topology-guarded canonical seam pinning

The M0 tessellator SHALL weld a shared CLOSED seam between two sub-faces — a
genuinely-curved annulus-with-hole on a curved surface (the `faceOutside` a smooth-trim
split lays on a bowl / dome wall) UNION a flat disk / cap on the cut plane (the
`faceInside` disk and the synthesized planar cap of `curvedWallHalfSpaceCut`) — WATERTIGHT
at ANY requested deflection. The seam edges are straight degree-1 3D chords that each carry
a pcurve on the CURVED incident surface; because the mesher today re-evaluates the curved
sub-face's seam-boundary vertices through THAT surface (`S_face(pcurve(t))`, which bulges
off the straight chord) while the flat sub-face places them on the chord, the two sub-faces
diverge and the closed seam welds watertight only at ISOLATED deflections.

To fix this, a shared CLOSED seam SHALL carry ONE **canonical** shared discretization — the
seam edge's straight-chord 3D sample points (`d.points`) — consumed by BOTH sub-faces with
OPPOSITE orientation, and every boundary vertex that comes from such a seam edge SHALL be
PINNED EXACTLY to the seam edge's canonical 3D sample point by PARAMETRIC / UV
correspondence (the sample index along the shared edge), NOT by spatial-proximity snap. Both
sub-faces SHALL then emit BIT-IDENTICAL 3D seam points at every sample, so the seam welds
without relying on the spatial weld tolerance bridging two independent samplings (removing
the FINE bowl-bulge divergence) and the CDT SHALL NOT densify across or fuse through the
seam (removing the COARSE weld-tolerance fold into a non-manifold edge used by more than two
triangles).

The addition SHALL be strictly ADDITIVE and reachable ONLY by the CLOSED-SEAM TOPOLOGY: a
face whose surface is genuinely curved (not a `Plane`) AND whose boundary edge is a straight
degree-1 chord seam AND that seam is shared as a CLOSED loop. An analytic-primitive edge
shared through ONE `TShape` node SHALL keep its per-node discretization; a straight
separate-node edge SHALL keep its endpoint-keyed count sharing and canonical straight
anchors; a curved separate-node edge SHALL keep the landed canonical curved-polyline path;
an OPEN seam SHALL keep its landed path. The canonical-closed-seam path SHALL NOT modify the
shared segment-count sizing, the curve evaluators, the three face-mesh arms
(`structuredGrid`, `earClipMesh`, `trimmedFreeformMesh`), the boundary flattener, or the
spatial weld, so every existing face SHALL mesh BYTE-IDENTICALLY — the same vertices,
triangles, watertight status, surface area, and enclosed volume (a single FNV hash over
`{vertices, triangles, watertight, area, volume}` IDENTICAL before vs after) — for `Plane`,
`Cylinder`, `Cone`, `Sphere`, `Bezier`, `BSpline`, curved seams, and the
box / holed / loft / sweep / thread / step / revolve solids. The ONLY meshes allowed to
change SHALL be the previously-failing closed-seam annulus cases (non-watertight →
watertight). The library SHALL remain OCCT-free and host-buildable, the `cc_*` ABI SHALL
stay additive-only, and NO global weld or snap tolerance SHALL be widened. A closed-seam
case that STILL cannot weld watertight SHALL keep the honest decline (non-watertight → NULL
→ OCCT) — NEVER a leaky or partial solid.

#### Scenario: The closed-seam annulus CUT welds watertight across the full deflection ladder (host, no OCCT)

- GIVEN the steep degree-2 Bézier bowl over a convex-quad prism and its CLOSED CIRCULAR interior seam, split by `splitFaceSmoothTrim` into a curved disk plus the annulus, with `curvedWallHalfSpaceCut` synthesizing the flat ANNULAR cap on the cut plane whose inner HOLE is the seam (the mid-wall pose — the canonical annulus case), built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut(operand, P, KeepSide::Below, d)` (CUT) runs at each deflection in the ladder `{0.02, 0.012, 0.008, 0.005, 0.0025}`
- THEN the welded solid SHALL be WATERTIGHT with Euler characteristic `χ = 2` at EVERY deflection (no watertight↔NotWatertight oscillation across the closed seam), AND its enclosed volume SHALL equal the closed form `(H0+c)·A_Q − c·π·ρ²/2` within the curved-tessellation band and CONVERGE monotonically as the deflection tightens (measured 0.62% → 0.10%); AND the dome pose (disk ∪ flat disk-cap sharing the seam) SHALL weld watertight at the fine `d = 0.004` that DECLINED before the seam pin

#### Scenario: Both sub-faces place bit-identical seam points because they share ONE canonical chord discretization (host)

- GIVEN a straight degree-1 chord seam shared, as a CLOSED loop, by a genuinely-curved annulus sub-face (evaluated through the bowl surface via its pcurve) and a flat cap sub-face on the cut plane, meshed on the host with a shared edge cache
- WHEN both sub-faces place their boundary vertices on that seam at any deflection
- THEN both sub-faces SHALL read the SAME canonical straight-chord 3D sample points and pin their seam-boundary vertices EXACTLY to them by sample-index correspondence, emitting BIT-IDENTICAL 3D seam points — the curved sub-face's seam boundary SHALL NOT bulge off the chord through its own surface evaluation — so the seam welds without relying on the spatial weld tolerance and the CDT does not densify across it

#### Scenario: Every existing surface kind meshes byte-identically after the addition (host + sim)

- GIVEN faces and solids of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Bezier`, `BSpline`, curved seams) and the box / holed / loft / sweep / thread / step / revolve solids, meshed before and after the canonical closed-seam path is added, together with the full tessellation-sensitive suite (`run-sim-suite`, STEP import, curved-fillet, curved-chamfer, curved-boolean, wrap-emboss, loft, phase3)
- WHEN each existing face / solid / suite is meshed at the same deflection and its FNV hash over `{vertices, triangles, watertight, area, volume}` is compared against the pre-change baseline
- THEN the hash SHALL be IDENTICAL to the baseline for every one of them (the guarded closed-seam branch is unreachable for them), AND the ONLY meshes whose hash changes SHALL be the previously-failing closed-seam annulus cases (non-watertight → watertight); if ANY existing hash differs, the change SHALL be reverted and the closed-seam weld SHALL keep the honest OCCT decline

#### Scenario: A closed-seam case that still cannot weld declines to NULL, and no tolerance is widened (host, no OCCT)

- GIVEN a closed-seam operand whose curved annulus ∪ flat cap does NOT mesh watertight at a given deflection even with canonical seam pinning, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut` runs at that deflection and its mandatory self-verify evaluates the M0 mesh
- THEN it SHALL return a NULL Shape with `NotWatertight` (→ OCCT fall-through) and SHALL NEVER return a leaky or partial solid, AND NO global weld or snap tolerance SHALL have been widened to force a pass

#### Scenario: Native-vs-OCCT parity of the closed-seam CUT weld across the ladder, including the fine deflection that declined before the pin (sim, OCCT oracle)

- GIVEN the SAME bowl-cup operand reconstructed in OCCT (a `Geom_BezierSurface` bowl trimmed by the rim circle plus a planar lid disk, sewn into an outward-oriented solid) on a booted iOS simulator
- WHEN OCCT cuts it by `BRepAlgoAPI_Common` against the keep-half box and the native `curvedWallHalfSpaceCut` CUT (`KeepSide::Below`) result is measured by the native M0 tessellator at each deflection in the ladder `{0.0102, 0.0053, 0.004, 0.0028}` — INCLUDING the fine `d = 0.004` that declined before the closed-seam pin — and the COMMON at its robust `d = 0.0102`
- THEN the native result SHALL match the OCCT result on VOLUME (`BRepGProp`, relative, within the curved band, cross-checked to the closed form), AREA (relative), WATERTIGHTNESS (closed 2-manifold), TOPOLOGY (Euler `χ = 2`), BBOX (per-axis, spatial band), and one-sided HAUSDORFF (native→OCCT, spatial band) at ALL asserted deflections, with FIXED tolerances that are never widened; AND the COMMON at a fine deflection SHALL remain an HONEST DECLINE (NULL → OCCT, `NotWatertight`) — never a leaky solid — because its remaining blocker is the separate curved-RIM weld, not the closed seam

# native-math

## ADDED Requirements

### Requirement: G1 N-sided boundary fill by Gregory pie-slice subdivision

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_nsided_g1.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `nSidedFillG1` that fills a consistent closed N-gon boundary
(`N ≥ 3`, the shared `NSidedBoundary` type, verified by the existing `verifyNSidedBoundary`) with **N
non-rational** tensor-product B-spline sub-patches that meet **G1 (tangent-plane continuous)** across
every internal spoke and to the boundary cross-tangent field. The routine SHALL be **additive**: the C0
`fillNSided` / `verifyNSidedBoundary` API SHALL remain byte-unchanged.

The construction SHALL subdivide the N-gon into N "pie slices", one per boundary edge `e[i]`, each on
`(u,v) ∈ [0,1]²`: `v=0` is the boundary edge `e[i]`, `v=1` collapses to the centroid `C = mean(V[i])`,
and the `u=0` / `u=1` edges are the spokes `V[i]→C` / `V[i+1]→C` (each SHARED with a neighbouring
slice). The u-direction SHALL carry the edge's own basis (degree-elevated to cubic by the exact Layer-1
`elevateDegreeCurve` so each slice has two distinct interior u-columns), so `v=0` reproduces `e[i]`
pointwise. The v-direction SHALL be a cubic Hermite from boundary to centre whose `v=0` cross-tangent
equals the prescribed or natural cross-boundary tangent field.

Adjacent slices SHALL share, at each corner, the identical spoke column AND a shared cross-spoke rib
field (injected into the second / second-to-last u-columns' interior v-levels) so their surface
cross-spoke derivative `∂S/∂u` along the seam is the identical vector — i.e. the two slices meet C1
(hence G1: the unit normal is continuous) across the seam BY POLE EQUALITY. The degenerate hub apex
(all slices collapse to `C`) SHALL be kept finite by the radial interior/twist row (the Gregory
twist case), with no blowup.

The routine SHALL accept an optional per-edge cross-boundary tangent field (`CrossTangentField`, empty
⇒ a natural in-surface field synthesised from the boundary + centroid geometry). It SHALL emit **N
non-rational** sub-patches (empty `weights`) plus the centroid, and SHALL report the achieved boundary
deviation and the worst G1 normal-mismatch angle.

#### Scenario: Boundary interpolation is machine-exact

- GIVEN a consistent non-rational closed N-gon (for example a planar pentagon, triangle, or heptagon, or a smooth non-planar closed loop)
- WHEN the G1 fill is built
- THEN each slice's `v=0` iso-curve SHALL reproduce its boundary curve on a dense sample to within 1e-12 (achieved ~1e-15)

#### Scenario: The fill is G1 across every internal spoke

- GIVEN a successful G1 fill of a tangent-continuous (smooth-corner) or planar boundary
- WHEN the shared internal spokes are sampled (excluding the degenerate hub apex)
- THEN the two slices meeting at each spoke SHALL share the spoke curve pointwise (C0 ≤ 1e-12) AND their unit surface normals SHALL agree across the seam to within 1e-6 rad (achieved 0 for planar boundaries; ~1e-16 for a genuinely non-planar smooth loop)

#### Scenario: A planar boundary yields a planar G1 fill

- GIVEN a boundary whose N edges are coplanar (for example a regular polygon in the `z=0` plane)
- WHEN the G1 fill is built
- THEN every point of every sub-patch SHALL lie on that plane to within 1e-10 (achieved 0)

#### Scenario: The degenerate hub apex is finite (Gregory twist, no blowup)

- GIVEN a successful G1 fill
- WHEN every sub-patch is evaluated up to the hub apex `v→1`
- THEN every evaluated point SHALL be finite, and each sub-patch's `v=1` iso SHALL equal the centroid `C`

### Requirement: G1 feasibility and honest declines

`nSidedFillG1` SHALL decline honestly (`ok = false` with a human-readable reason, never a wrong surface
and never a crash) on every configuration for which a G1 fill is impossible or the input is malformed,
and SHALL NEVER widen a tolerance to pass an oracle.

It SHALL reuse `verifyNSidedBoundary` to decline a non-closed loop, a rational or malformed edge, and
`N < 3`. It SHALL decline a boundary that CREASES at a corner in 3-D — non-collinear incident edge
tangents that are not coplanar with the spoke, so no tangent plane exists across the incident spokes —
because a tangent-plane-continuous surface cannot cross a boundary that is itself tangent-discontinuous
there. It SHALL decline a prescribed cross-boundary tangent that is (anti-)parallel to the boundary
tangent (no tangent plane at the boundary). It SHALL decline a cross-tangent field count that is neither
empty nor exactly `N`, and a per-edge field whose pole count does not match the (elevated) edge.

Machine-exact G1 across the incident spokes SHALL hold when the boundary is tangent-continuous at its
corners (a smooth closed loop) OR the boundary is planar. Rational (weighted) N-sided G1 fill and G2
(curvature-continuous) plate blends remain documented residuals in `docs/NURBS-SCOPE.md`.

#### Scenario: A creased 3-D corner is declined, not silently creased

- GIVEN a non-planar boundary of straight chords whose corners crease in 3-D (for example a pentagon with alternately lifted corners)
- WHEN the G1 fill is requested
- THEN it SHALL return `ok = false` with a reason naming the creased corner, and SHALL NOT emit a surface with a residual crease; the C0 `fillNSided` of the same boundary SHALL by contrast fill it with a measurable normal mismatch at the spokes

#### Scenario: A G1-incompatible prescribed cross-tangent is declined

- GIVEN a prescribed cross-boundary tangent field that is (anti-)parallel to the boundary tangent on some edge
- WHEN the G1 fill is requested
- THEN it SHALL return `ok = false` with a reason (no tangent plane at the boundary → G1 impossible), without widening any tolerance

#### Scenario: Non-closed / rational / N<3 / malformed / wrong-field-count declines

- GIVEN a boundary with a displaced corner (open loop), or a rational edge, or fewer than three edges, or a malformed edge, or a cross-tangent field whose count is neither empty nor N
- WHEN the G1 fill is requested
- THEN it SHALL return `ok = false` with a reason, without crashing

### Requirement: Non-rational G1 N-sided scope with rational and G2 blends as explicit residuals

The G1 N-sided module SHALL produce **non-rational** tensor-product B-spline sub-patches (empty
`weights`) from **non-rational** boundaries only, and SHALL NOT fabricate weights or claim
rational-N-sided capability. It SHALL claim tangent-plane (G1) continuity only within the feasibility
scope above (tangent-continuous or planar boundaries), and SHALL NOT claim curvature (G2) continuity
across the interior spokes. Rational / weighted G1 N-sided fill and the G2 plate blends are documented
residuals for later slices, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: G1 N-sided geometry is non-rational and not faked-G2 or faked-rational

- GIVEN any successful G1 N-sided fill
- WHEN the result is inspected
- THEN every sub-patch's `weights` vector SHALL be empty (non-rational), and the module SHALL NOT attach fabricated weights nor claim G2 continuity across the interior spokes

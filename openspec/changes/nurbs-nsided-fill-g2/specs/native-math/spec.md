# native-math

## ADDED Requirements

### Requirement: G2 N-sided boundary fill by Gregory quintic pie-slice subdivision

The native math library SHALL provide, in an OCCT-free module
(`src/native/math/bspline_nsided_g2.{h,cpp}`, namespace `cybercad::native::math`,
`CYBERCAD_HAS_NUMSCI`-gated), a routine `nSidedFillG2` that fills a consistent closed N-gon boundary
(`N ≥ 3`, the shared `NSidedBoundary` type, verified by the existing `verifyNSidedBoundary`) with **N
non-rational** tensor-product B-spline sub-patches that meet **G2 (curvature continuous)** across every
internal spoke and to the boundary cross-tangent + cross-curvature fields. The routine SHALL be
**additive**: the C0 `fillNSided` / `verifyNSidedBoundary` API and the G1 `nSidedFillG1` /
`CrossTangentField` API SHALL remain byte-unchanged.

The construction SHALL subdivide the N-gon into N "pie slices", one per boundary edge `e[i]`, each on
`(u,v) ∈ [0,1]²`: `v=0` is the boundary edge `e[i]`, `v=1` collapses to the centroid `C = mean(V[i])`,
and the `u=0` / `u=1` edges are the spokes `V[i]→C` / `V[i+1]→C` (each SHARED with a neighbouring
slice). The u-direction SHALL carry the edge's own basis (degree-elevated to at least 5 by the exact
Layer-1 `elevateDegreeCurve` so each slice has three distinct seam-adjacent u-columns per side), so `v=0`
reproduces `e[i]` pointwise. The v-direction SHALL be a **quintic (degree-5) G2 Hermite** from boundary
to hub whose `v=0` position, first derivative (cross-tangent) and second derivative (cross-curvature)
equal the prescribed or natural cross-boundary fields, and whose hub side is radially tapered.

Adjacent slices SHALL share, at each corner spoke, byte-identical position + 1st-inward-rib + 2nd-inward-
rib data across the three seam-adjacent u-columns, so their surface cross-spoke first derivative `∂S/∂u`
AND second derivative `∂²S/∂u²` along the seam are the identical vectors — i.e. the two slices meet C0,
G1 (unit normal continuous) AND G2 (normal curvature continuous) across the seam BY POLE EQUALITY. The
degenerate hub apex (all slices collapse to `C`) SHALL be kept finite by radially tapering the interior
1st/2nd derivative rows (the Gregory twist case), with no blowup.

The routine SHALL accept an optional per-edge cross-boundary tangent field (`CrossTangentField`) and an
optional per-edge cross-boundary curvature field (`CrossCurvatureField`); an empty field ⇒ a natural
minimal-energy field synthesised from the boundary + centroid geometry. It SHALL emit **N non-rational**
sub-patches (empty `weights`, quintic in v) plus the centroid, and SHALL report the achieved boundary
deviation, the worst G1 normal-mismatch angle, and the worst relative normal-curvature mismatch.

#### Scenario: Boundary interpolation is machine-exact

- GIVEN a consistent non-rational closed N-gon (for example a planar pentagon, triangle, or heptagon, a smooth non-planar G2-continuous loop, or a spherical N-gon)
- WHEN the G2 fill is built
- THEN each slice's `v=0` iso-curve SHALL reproduce its boundary curve on a dense sample to within 1e-12 (achieved ~1e-15)

#### Scenario: The fill is G2 across every internal spoke

- GIVEN a successful G2 fill of a curvature-continuous or planar boundary
- WHEN the shared internal spokes are sampled (excluding the degenerate hub apex)
- THEN the two slices meeting at each spoke SHALL share the spoke curve pointwise (C0 ≤ 1e-12), their unit surface normals SHALL agree across the seam to within 1e-6 rad, AND their principal (normal) curvatures SHALL agree to a relative tolerance of 1e-5 (achieved 0 for planar; ~5e-7 curvature-rel for a genuinely non-planar smooth G2 loop; ~1e-16 for a sphere)

#### Scenario: A G2 boundary on a sphere matches the analytic surface

- GIVEN a boundary of G2-continuous arcs lying on a sphere (matching the sphere's latitude-circle position + tangent + curvature at each corner)
- WHEN the G2 fill is built
- THEN the boundary iso-curve SHALL lie on the sphere to the quintic-arc approximation of the (rational) circle, AND the seams SHALL be G2 (normal curvature continuous to relative 1e-5, achieved ~1e-16)

#### Scenario: A planar boundary yields a planar G2 fill

- GIVEN a boundary whose N edges are coplanar (for example a regular polygon in the `z=0` plane)
- WHEN the G2 fill is built
- THEN every point of every sub-patch SHALL lie on that plane to within 1e-10 (achieved 0), and the seams SHALL be trivially G2 (zero curvature)

#### Scenario: The degenerate hub apex is finite (Gregory twist, no blowup)

- GIVEN a successful G2 fill
- WHEN every sub-patch is evaluated up to the hub apex `v→1`
- THEN every evaluated point SHALL be finite, and each sub-patch's `v=1` iso SHALL equal the centroid `C`

### Requirement: G2 feasibility and honest declines

`nSidedFillG2` SHALL decline honestly (`ok = false` with a human-readable reason, never a wrong surface
and never a crash) on every configuration for which a G2 fill is impossible or the input is malformed,
and SHALL NEVER widen a tolerance to pass an oracle.

It SHALL reuse `verifyNSidedBoundary` to decline a non-closed loop, a rational or malformed edge, and
`N < 3`. It SHALL decline (inherited from G1 feasibility) a boundary that CREASES at a corner in 3-D and a
prescribed cross-boundary tangent that is (anti-)parallel to the boundary tangent. Additionally it SHALL
decline a boundary that is CURVATURE-DISCONTINUOUS at a corner — the two incident edges' curvature vectors
`κ⃗ = (r'' − (r''·T̂)T̂)/|r'|²` disagree there (the arcs are G1 but not G2, a curvature crease) — because a
curvature-continuous surface cannot cross a boundary that is itself curvature-discontinuous there. It SHALL
decline a prescribed cross-curvature whose two incident-corner values point in substantially OPPOSING
directions (irreconcilable second-order data). It SHALL decline a cross-tangent or cross-curvature field
count that is neither empty nor exactly `N`, and a per-edge field whose pole count does not match the
(elevated) edge.

Machine-exact G2 across the incident spokes SHALL hold when the boundary is curvature-continuous at its
corners OR the boundary is planar. Rational (weighted) N-sided fill remains a documented residual in
`docs/NURBS-SCOPE.md`.

#### Scenario: A curvature-creased boundary is declined, not silently curvature-creased

- GIVEN a non-planar smooth loop split into G1-only arcs (position + tangent matched at corners but not the second derivative), so the boundary is curvature-discontinuous at its corners
- WHEN the G2 fill is requested
- THEN it SHALL return `ok = false` with a reason naming the curvature-discontinuous corner, and SHALL NOT emit a surface with a residual curvature crease

#### Scenario: A corner-incompatible prescribed cross-curvature is declined

- GIVEN a prescribed cross-boundary curvature field whose two incident values at some corner point in opposing directions
- WHEN the G2 fill is requested
- THEN it SHALL return `ok = false` with a reason (irreconcilable second-order data → G2 impossible), without widening any tolerance

#### Scenario: Non-closed / rational / N<3 / malformed / wrong-field-count / parallel-tangent declines

- GIVEN a boundary with a displaced corner (open loop), or a rational edge, or fewer than three edges, or a malformed edge, or a tangent- or curvature-field whose count is neither empty nor N, or a prescribed cross-tangent (anti-)parallel to the boundary tangent
- WHEN the G2 fill is requested
- THEN it SHALL return `ok = false` with a reason, without crashing

### Requirement: Non-rational G2 N-sided scope with rational blend as an explicit residual

The G2 N-sided module SHALL produce **non-rational** tensor-product B-spline sub-patches (empty
`weights`) from **non-rational** boundaries only, and SHALL NOT fabricate weights or claim
rational-N-sided capability. It SHALL claim curvature (G2) continuity only within the feasibility scope
above (curvature-continuous or planar boundaries). Rational / weighted G2 N-sided fill is a documented
residual for a later slice, recorded in `docs/NURBS-SCOPE.md`.

#### Scenario: G2 N-sided geometry is non-rational and not faked-rational

- GIVEN any successful G2 N-sided fill
- WHEN the result is inspected
- THEN every sub-patch's `weights` vector SHALL be empty (non-rational), each sub-patch SHALL be quintic in the v-direction, and the module SHALL NOT attach fabricated weights nor claim rational-N-sided capability

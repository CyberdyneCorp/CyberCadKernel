# native-tessellation

## ADDED Requirements

### Requirement: Weld a shared CURVED outer rim between a free-form face and an adjacent analytic face watertight at any deflection via topology-guarded canonical rim pinning

The M0 tessellator SHALL weld a genuinely-CURVED outer RIM shared between a FREE-FORM face
(a Bézier / B-spline bowl annulus, the `faceOutside` a smooth-trim split lays on a bowl /
dome wall) and an ADJACENT ANALYTIC face (the flat top lid `Plane` kept whole by
`curvedWallHalfSpaceCut`) WATERTIGHT at ANY requested deflection. The rim is a per-segment
degree-≥2 free-form arc carried on SEPARATE edge nodes (the free-form face's node with a
`Line` pcurve; the analytic face's node with a free-form pcurve); the solid mesher's twist
pre-pass subdivides it to the SAME shared fraction list for both faces, but because the
analytic neighbour's pcurve does NOT reproduce the 3-D rim curve — `S_analytic(pcurve(f)) ≠
C_edge(f)`, a flat lid's rim pcurve stays IN the plane while the true rim arc dips off it —
the two faces' subdivided rim samples diverge (measured up to ~6e-4, far beyond the
mesher's `kSnapEps = 1e-6` anchor-snap radius) and the rim opens once subdivided, AND a
coarse-regime near-degenerate COINCIDENT triangulation sliver (a rim edge used by more than
two triangles) survives.

To fix this, a shared CURVED rim SHALL carry ONE **canonical** shared discretization — the
rim edge's 3-D sample points (`d.points == C_edge`) — and every boundary sample of an
incident face that genuinely DIVERGES from that canonical discretization
(`‖S_face(pcurve(f)) − C_edge(f)‖ > kSnapEps`) SHALL be PINNED EXACTLY to the canonical
point by UV correspondence, NOT by spatial-proximity snap. The free-form face (which
reproduces `C_edge`) SHALL record no pin (byte-identical there); the diverging analytic
neighbour SHALL be pinned to the shared rim, so both faces emit BIT-IDENTICAL 3-D rim points
and the rim welds without relying on the spatial weld tolerance bridging two independent
samplings. In addition, the spatial weld SHALL drop every copy of a COINCIDENT-DUPLICATE
triangle (two triangles on the same three merged vertices — the coarse-regime sliver) and
compact any vertex the drop orphans, so the welded rim is a single closed 2-manifold with
Euler characteristic `χ = 2`.

The addition SHALL be strictly ADDITIVE and reachable ONLY by the FREEFORM↔ANALYTIC CURVED
RIM topology: a shared edge that is a genuinely-curved degree-≥2 free-form (Bézier /
B-spline) arc — NOT an analytic `Circle` / `Ellipse` (excluded BY KIND, so every analytic
primitive's cap↔side / latitude / torus-rim seam is ineligible), NOT a `Line`, NOT a
degree-1 polyline, and NOT the 2-pole degree-1 seam chord (the landed M0w path owns it) —
AND on which an incident face's surface evaluation genuinely diverges from the canonical
discretization. An analytic-primitive edge shared through ONE `TShape` node SHALL keep its
per-node discretization; a curved edge shared through one node (a whole Bézier / B-spline
primitive) reproduces `C_edge` on both faces and SHALL never be pinned; a twisted-loft /
twisted-sweep free-form seam SHALL keep its unchanged twist segment sizing and, its two
free-form faces both reproducing `C_edge`, SHALL never be pinned; the closed-seam chord path
and the straight / curved canonical-anchor paths SHALL be unchanged. The coincident-sliver
drop and orphan compaction SHALL fire ONLY when a coincident-duplicate triangle / orphan
vertex EXISTS. The curved-rim path SHALL NOT modify the twist pre-pass
(`requireEdgeSegments`), the shared segment-count sizing, the curve evaluators, the three
face-mesh arms (`structuredGrid`, `earClipMesh`, `trimmedFreeformMesh`), or the boundary
flattener, so every existing face SHALL mesh BYTE-IDENTICALLY — the same vertex count,
triangle count, vertices, triangles, watertight status, surface area, and enclosed volume (a
single FNV hash over `{vertexCount, triangleCount, vertices, triangles, watertight, area,
volume}` IDENTICAL before vs after) — for `Plane`, `Cylinder`, `Cone`, `Sphere`, `Bezier`,
`BSpline`, curved seams, and the box / triangle-prism / cylinder / cone / sphere-revolve /
thread / sweep / loft (straight, frustum, TWISTED ruled) / twisted-sweep / mid-wall /
first-freeform solids. The ONLY meshes allowed to change SHALL be the previously-failing
curved-wall COMMON rim cases (non-watertight → watertight). The library SHALL remain
OCCT-free and host-buildable, the `cc_*` ABI SHALL stay additive-only, and NO global weld or
snap tolerance SHALL be widened. A rim case that STILL cannot weld watertight SHALL keep the
honest decline (non-watertight → NULL → OCCT) — NEVER a leaky or partial solid.

#### Scenario: The curved-wall COMMON rim welds watertight across the full deflection ladder (host, no OCCT)

- GIVEN the steep degree-2 Bézier bowl trimmed by a rim circle plus a flat top-lid disk sharing that rim, split by `splitFaceSmoothTrim` and reassembled by `curvedWallHalfSpaceCut(KeepSide::Above)` (COMMON keeps the free-form annulus + the flat lid + the cap), built on the host with NO OCCT
- WHEN COMMON runs at each deflection in the full ladder `{0.012, 0.0102, 0.008, 0.006, 0.004, 0.002, 0.001}` — the fine range that DECLINED before the curved-rim weld
- THEN it SHALL NOT decline (no `NotWatertight` NULL) at any deflection, the welded solid SHALL be WATERTIGHT with Euler characteristic `χ = 2` at EVERY deflection, AND its enclosed volume SHALL equal the closed-form COMMON volume `V(z≥c) = V(full) − V(z≤c)` within 3% at every step and CONVERGE toward it as the deflection tightens (the finest strictly closer than the coarsest, and within 1%)

#### Scenario: The free-form face and the analytic neighbour place bit-identical rim points because the diverging samples are pinned to ONE canonical rim curve (host)

- GIVEN a genuinely-curved degree-2 Bézier rim shared, on SEPARATE edge nodes, by a free-form bowl annulus (whose `Line` pcurve reproduces the rim curve `C_edge` on the bowl surface) and a flat lid `Plane` (whose free-form pcurve stays in the plane and diverges from `C_edge`), meshed on the host with a shared edge cache
- WHEN both faces subdivide that rim to the same shared fraction list and place their boundary vertices
- THEN the free-form face SHALL reproduce `C_edge` and record NO pin, the diverging analytic neighbour's samples SHALL be pinned EXACTLY to the same canonical `C_edge` points, both faces SHALL emit BIT-IDENTICAL 3-D rim points, and the coarse-regime coincident-duplicate sliver SHALL be dropped and its orphan vertex compacted, so the rim welds as a single closed 2-manifold

#### Scenario: Every existing surface kind meshes byte-identically after the addition, including twisted free-form (host + sim)

- GIVEN faces and solids of every existing surface kind (`Plane`, `Cylinder`, `Cone`, `Sphere`, `Bezier`, `BSpline`, curved seams) and the box / triangle-prism / cylinder / cone / sphere-revolve / thread / sweep / loft (straight, frustum, TWISTED ruled) / twisted-sweep / mid-wall / first-freeform solids — INCLUDING the twisted free-form cases whose curved seams legitimately need the twist subdivision and the analytic revolves whose shared `Circle` seams could be mistaken for a rim — meshed before and after the curved-rim path is added, together with the full tessellation-sensitive suite
- WHEN each existing face / solid / suite is meshed at the same deflection and its FNV hash over `{vertexCount, triangleCount, vertices, triangles, watertight, area, volume}` is compared against the pre-change baseline
- THEN the hash SHALL be IDENTICAL to the baseline for every one of them (the KIND guard excludes analytic `Circle`/`Ellipse` seams, the divergence gate excludes free-form edges that reproduce `C_edge`, the twist pre-pass is untouched, and the sliver drop / orphan compaction are unreachable without a duplicate / orphan), AND the ONLY meshes whose hash changes SHALL be the previously-failing curved-wall COMMON rim cases (non-watertight → watertight); if ANY existing hash differs, the change SHALL be reverted and the rim weld SHALL keep the honest OCCT decline

#### Scenario: A curved-rim case that still cannot weld declines to NULL, and no tolerance is widened (host, no OCCT)

- GIVEN a curved-wall operand whose free-form annulus ∪ flat lid rim does NOT mesh watertight at a given deflection even with canonical rim pinning and the coincident-sliver drop, built on the host with NO OCCT
- WHEN `curvedWallHalfSpaceCut` runs at that deflection and its mandatory self-verify evaluates the M0 mesh
- THEN it SHALL return a NULL Shape with `NotWatertight` (→ OCCT fall-through) and SHALL NEVER return a leaky or partial solid, AND NO global weld or snap tolerance SHALL have been widened to force a pass

#### Scenario: Native-vs-OCCT parity of the curved-wall COMMON rim weld (sim, OCCT oracle)

- GIVEN the SAME bowl-cup operand reconstructed in OCCT (a `Geom_BezierSurface` bowl trimmed by the rim circle plus a planar lid disk, sewn into an outward-oriented solid) on a booted iOS simulator
- WHEN OCCT cuts it by `BRepAlgoAPI_Common` against the keep-half box and the native `curvedWallHalfSpaceCut` COMMON (`KeepSide::Above`) result is measured by the native M0 tessellator at its asserted deflection ladder
- THEN the native result SHALL match the OCCT result on VOLUME (`BRepGProp`, relative, within the curved band, cross-checked to the closed form), AREA (relative), WATERTIGHTNESS (closed 2-manifold), TOPOLOGY (Euler `χ = 2`), BBOX (per-axis, spatial band), and one-sided HAUSDORFF (native→OCCT, spatial band), with FIXED tolerances that are never widened; COMMON SHALL now be a watertight MATCH (no longer an honest decline) at its ladder

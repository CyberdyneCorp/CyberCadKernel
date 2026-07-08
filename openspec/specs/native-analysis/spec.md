# native-analysis Specification

## Purpose
TBD - created by archiving change moat-gs-measure-curvature. Update Purpose after archive.
## Requirements
### Requirement: Minimum distance between a B-rep entity pair, OCCT-free, matching the closed form and OCCT DistShapeShape

The library SHALL compute the minimum Euclidean distance between two resolved B-rep
entities — each a vertex point, an `EdgeCurve` (over its parameter range), or a
`FaceSurface` (over its trim region) — and return BOTH the distance and the two
witness points (one on each entity), computed with **no OCCT** on the native
analytic / NURBS evaluators (`math/elementary.h`, `math/torus.h`, `math/bspline.h`)
and the OCCT-`Extrema`-verified `numerics/closest_point.h` projection layer.

For an **analytic·analytic** pair (point·point, point·line/segment, point·circle,
point·plane, point·cylinder, point·sphere, line·line including the parallel and skew
cases with feet clamped to their segment ranges) the result SHALL be the exact
closed form. For a pair involving a **NURBS** curve or surface the result SHALL be
obtained by seeding on a deterministic coarse parameter sample and refining each seed
with the `numerics/closest_point` Newton polish, returning the GLOBAL minimum; a
witness on a face SHALL lie inside the face's trim region and a witness on an edge
SHALL lie in its parameter range (a constrained boundary restart is used when the
unconstrained optimum falls outside the trim).

The library SHALL NOT return a guessed minimum: when the pair involves a
genuinely-trimmed freeform patch whose global optimum the seeded minimizer cannot
certify (multiple comparably-deep basins within the seed resolution, or a
non-converging boundary-constrained restart), the service SHALL DECLINE. No tolerance
SHALL be weakened, and `src/native/**` SHALL remain OCCT-free and host-buildable.

#### Scenario: Point-to-line and point-to-plane distance match the closed form (host, no OCCT)

- GIVEN a point and, separately, an infinite line and a plane with known analytic placement, built on the host with no OCCT
- WHEN the minimum distance to each is computed
- THEN the distance SHALL equal the hand-derived closed form (point-to-line perpendicular; point-to-plane signed magnitude) within `1e-9`, AND the returned witness point on the line/plane SHALL be the exact foot of the perpendicular

#### Scenario: Line-to-line distance handles parallel and skew with clamped feet (host, no OCCT)

- GIVEN two segments that are (a) parallel and (b) skew, with known endpoints, built on the host with no OCCT
- WHEN the minimum distance between each pair is computed
- THEN the parallel case SHALL return the constant point-to-line distance and the skew case SHALL return the common-perpendicular distance, each within `1e-9`, with both witness feet clamped to their respective segment ranges (matching the bounded distance OCCT `BRepExtrema_DistShapeShape` reports)

#### Scenario: Distance to a simple NURBS entity matches OCCT DistShapeShape (sim)

- GIVEN a point (or line) and a simple, smooth NURBS edge or untrimmed NURBS face, on a booted iOS simulator
- WHEN the native minimum distance is computed and compared to `BRepExtrema_DistShapeShape` on the same shapes
- THEN the native distance AND both witness points SHALL match the OCCT result within a scale-relative tolerance

#### Scenario: Distance to a non-certifiable trimmed freeform patch declines honestly (host + sim)

- GIVEN an entity pair in which one side is a genuinely-trimmed freeform patch whose seeded minimizer cannot certify the global optimum (comparably-deep basins / non-converging boundary restart)
- WHEN the minimum distance is requested
- THEN the service SHALL DECLINE (return the decline outcome, `cc_last_error` set at the facade), emit NO distance value, weaken no tolerance, and leave `src/native/**` OCCT-free — the honest deferral is reported, not a guessed minimum

### Requirement: Angle between two linear or planar entities, OCCT-free closed form

The library SHALL compute the angle between two entities when BOTH are linear or
planar — line·line, plane·plane, or line·plane — from the entities' direction /
normal vectors, with no OCCT: line·line SHALL return `acos(|d_a·d_b|)` in `[0, π/2]`
(parallel → 0), plane·plane SHALL return `acos(clamp(n_a·n_b, −1, 1))` in `[0, π]`
(oriented normals), and line·plane SHALL return `asin(|d·n|)` in `[0, π/2]` (0 when
the line lies in the plane). The library SHALL DECLINE for any entity that is not a
line or a plane (a general curve or curved surface has no single angle) and for a
degenerate direction (`‖d‖ ≈ 0`); it SHALL NOT return a meaningless angle.

#### Scenario: Line-line, plane-plane, and line-plane angles match the closed form (host, no OCCT)

- GIVEN two lines at a known angle, two planes at a known dihedral, and a line at a known inclination to a plane, built on the host with no OCCT
- WHEN each angle is computed
- THEN line·line SHALL equal the acute angle in `[0, π/2]`, plane·plane SHALL equal the oriented-normal angle in `[0, π]`, and line·plane SHALL equal the line-to-plane angle in `[0, π/2]`, each within `1e-9`

#### Scenario: Angle to a non-linear/non-planar entity declines (host)

- GIVEN a request for the angle between an entity that is neither a line nor a plane (e.g. a circle or a curved surface) and any second entity, on the host with no OCCT
- WHEN the angle is requested
- THEN the service SHALL DECLINE and return no angle, rather than fabricate a value from a tangent or an averaged normal

### Requirement: Surface curvature at a parameter point — Gaussian, mean, and principal — OCCT-free, matching closed form and OCCT SLProps

The library SHALL compute, at a surface parameter `(u, v)`, the Gaussian curvature
`K`, the mean curvature `H`, and the ordered principal curvatures `k1 ≥ k2`, with no
OCCT. For an analytic `FaceSurface` the result SHALL be the exact closed form: a
plane gives `K=0, H=0, k1=k2=0`; a sphere of radius `R` gives `K=1/R²`, `H=1/R`,
`k1=k2=1/R`; a cylinder of radius `R` gives `K=0`, principal `{1/R, 0}`, `H=1/(2R)`;
a torus of major `R` / minor `r` gives `K = cos v /(r (R + r cos v))` and
`H = (R + 2 r cos v)/(2 r (R + r cos v))`. For a NURBS (`Kind::BSpline`/`Bezier`)
surface the result SHALL be computed from the first fundamental form
`E,F,G` (from `S_u, S_v`) and the second fundamental form `L,M,N` (from
`S_uu, S_uv, S_vv` projected on the unit normal), obtained from `math::surfaceDerivs`
/ `nurbsSurfaceDerivs` at `maxDeriv = 2`, as `K = (LN − M²)/(EG − F²)`,
`H = (EN − 2FM + GL)/(2(EG − F²))`, and `k1, k2 = H ± √(max(0, H² − K))`.

The `k1 ≥ k2` ordering and the sign of `H`, `k1`, `k2` SHALL follow the outward /
face-normal convention so the result matches OCCT `BRepLProp_SLProps` (a `Reversed`
face flips the sign of `H`, `k1`, `k2`, and leaves `K` unchanged). When the first
fundamental form is degenerate (`EG − F² ≤ ε·max(E,G)²`, a parametric singularity
such as a sphere pole or a cone apex) the library SHALL DECLINE rather than emit a
blown-up value.

#### Scenario: Analytic surface curvatures match the closed form (host, no OCCT)

- GIVEN a plane, a sphere of radius `R`, a cylinder of radius `R`, and a torus of major `R` / minor `r`, built on the host with no OCCT
- WHEN Gaussian, mean, and principal curvatures are computed at interior parameter points
- THEN the plane SHALL give all zeros, the sphere SHALL give `K=1/R²` and `H=1/R`, the cylinder SHALL give `K=0` and `H=1/(2R)` with principal `{1/R, 0}`, and the torus SHALL give `K = cos v /(r(R+r cos v))`, each within `1e-9`

#### Scenario: NURBS surface curvature matches OCCT BRepLProp_SLProps (sim)

- GIVEN a simple, smooth NURBS face (rational or non-rational) and an interior `(u, v)`, on a booted iOS simulator
- WHEN the native Gaussian, mean, and principal curvatures are computed from the first/second fundamental forms and compared to `BRepLProp_SLProps` at the same `(u, v)`
- THEN the native `K`, `H`, `k1`, and `k2` SHALL match the OCCT values within a scale-relative tolerance, with the sign convention matching the face normal (flipped for a `Reversed` face)

#### Scenario: Curvature at a parametric singularity declines honestly (host + sim)

- GIVEN a surface point where the first fundamental form is degenerate (`EG − F² ≤ ε·max(E,G)²`) — e.g. a sphere pole or within `ε` of a cone apex
- WHEN the curvature is requested
- THEN the service SHALL DECLINE and emit no curvature, rather than return a blown-up or arbitrary value; no tolerance SHALL be weakened

### Requirement: Edge curvature at a parameter point, OCCT-free

The library SHALL compute the curvature `κ` of an `EdgeCurve` at a parameter `t` with
no OCCT: a line gives `0`, a circle of radius `R` gives `1/R`, an ellipse gives the
closed form `‖C′×C″‖/‖C′‖³`, and a NURBS (`BSpline`/`Bezier`) edge gives
`κ = ‖C′×C″‖ / ‖C′‖³` from `math::curveDerivs` / `nurbsCurveDerivs` at
`maxDeriv = 2`. When the first derivative is degenerate (`‖C′‖ ≤ ε`, a stationary
point or cusp) the library SHALL DECLINE rather than divide by a near-zero speed.

#### Scenario: Analytic edge curvature matches the closed form (host, no OCCT)

- GIVEN a straight edge and a circular edge of radius `R`, built on the host with no OCCT
- WHEN the curvature is computed at an interior parameter
- THEN the line SHALL give `κ = 0` and the circle SHALL give `κ = 1/R` within `1e-9`

#### Scenario: NURBS edge curvature matches OCCT GeomLProp (sim)

- GIVEN a simple, smooth NURBS edge and an interior parameter `t`, on a booted iOS simulator
- WHEN the native curvature is computed and compared to `GeomLProp` at the same `t`
- THEN the native `κ` SHALL match the OCCT value within a scale-relative tolerance

#### Scenario: Edge curvature at a cusp declines (host)

- GIVEN a NURBS edge parameter where `‖C′‖ ≤ ε` (a stationary/cusp point), on the host with no OCCT
- WHEN the curvature is requested
- THEN the service SHALL DECLINE rather than return a value divided by a near-zero speed

### Requirement: Additive cc_* measurement and curvature facade, ABI-compatible

The library SHALL expose the GS3 measurement and GS4 curvature services through
ADDITIVE `cc_*` facade functions only, leaving every pre-existing `cc_*` signature
and POD struct byte-for-byte unchanged: `cc_measure_distance` (returning
`out7 = [distance, p1x, p1y, p1z, p2x, p2y, p2z]`), `cc_measure_angle` (returning the
angle in radians), `cc_surface_curvature` (returning `out4 = [K, H, k1, k2]`), and
`cc_edge_curvature` (returning `κ`). Each function SHALL resolve the caller's
`(subKind, subId)` sub-shape selectors to the native leaf geometry, return `1` on
success, and return `0` on an honest decline with `cc_last_error` set to a
descriptive message. No engine type SHALL cross the facade, and the OCCT oracle SHALL
exist only in the sim verification harness, never in `src/native/**`.

#### Scenario: New facade functions are additive and the ABI contract is unchanged

- GIVEN the `cc_kernel.h` header compiled with `CC_KERNEL_NO_PROTOTYPES` for the ABI contract test
- WHEN the pre-existing structs and function signatures are compared against the baseline
- THEN every pre-existing struct and signature SHALL be byte-identical, and only the four new prototypes (`cc_measure_distance`, `cc_measure_angle`, `cc_surface_curvature`, `cc_edge_curvature`) SHALL be added

#### Scenario: The facade reports an honest decline through the return code (host + sim)

- GIVEN a facade call for an entity pair or configuration this slice does not robustly handle (trimmed-freeform distance, non-line/plane angle, or singular curvature)
- WHEN the corresponding `cc_measure_*` / `cc_*_curvature` function is invoked
- THEN it SHALL return `0`, set `cc_last_error` to a descriptive message, write no result into the out-array, and never return a wrong measurement

### Requirement: Native inertia tensor and principal moments/axes from the triangulation, OCCT-free, matching the closed form and OCCT GProp_PrincipalProps

The library SHALL compute, for a watertight native solid and with **no OCCT**, the
mass-inertia tensor about the solid's centroid and its ordered principal moments
`I₁ ≤ I₂ ≤ I₃` (unit density → volume inertia, matching OCCT's convention) together
with the corresponding orthonormal principal axes. The tensor SHALL be accumulated as
the **second moments of the signed-tetra fan** over the M0 triangulation from the
origin — the SAME fan whose first-order sum `⅙ Σ aᵢ·(bᵢ×cᵢ)` the library already uses
for the enclosed volume (`tessellate/mesh.h::enclosedVolume`, the divergence theorem)
— shifted to the centroid by the parallel-axis (Huygens–Steiner) correction, formed as
`I = tr(Cov)·Id − Cov`, and diagonalized by a deterministic symmetric-3×3 **Jacobi**
eigensolver whose eigenvalues sorted ascending are the principal moments and whose
eigenvectors are the principal axes.

`NativeEngine::principal_moments` SHALL be wired from its `CC_NATIVE_BODY_UNSUPPORTED`
stub to this native path, guarded by the existing **watertight precondition**
(`robustlyWatertight` — closed at every deflection in the ladder, with a positive net
signed volume): a body that is not robustly watertight (open shell, non-manifold, or a
net signed volume `≤ 0`) has no meaningful inertia, so the engine SHALL **DECLINE**
(fall back to OCCT) rather than emit a wrong tensor. Planar polyhedra mesh exactly, so
their inertia SHALL be exact; a curved solid's inertia SHALL be within the same
deflection bound the native `mass_properties` already ships under. No tolerance SHALL
be weakened and `src/native/**` SHALL remain OCCT-free and host-buildable. The existing
`cc_principal_moments(body, out3 = [I₁,I₂,I₃])` ABI SHALL be unchanged (no new facade
symbol); the principal axes SHALL be returned by the native service and asserted
against the OCCT axes in the sim harness only.

#### Scenario: Box, cylinder, and sphere principal moments match the closed form (host, no OCCT)

- GIVEN a box `a×b×c`, a cylinder of radius `r` / height `h`, and a sphere of radius `r`, built as native solids on the host with no OCCT
- WHEN the native principal moments about the centroid are computed
- THEN the box SHALL give `I = (V/12)·{b²+c², a²+c², a²+b²}` EXACTLY (planar mesh), the cylinder SHALL give `{(V/12)(3r²+h²), (V/12)(3r²+h²), V·r²/2}`, and the sphere SHALL give `{2/5·V·r², 2/5·V·r², 2/5·V·r²}`, each within the tolerance (exact for the box, deflection-bounded for the curved solids), AND the returned principal axes SHALL be an orthonormal frame that diagonalizes the tensor

#### Scenario: Native inertia matches OCCT GProp_PrincipalProps moments and axes (sim)

- GIVEN an analytic solid and a simple NURBS solid, on a booted iOS simulator
- WHEN the native principal moments and axes are computed and compared to OCCT `GProp_PrincipalProps` (`Moments` + principal axes) on the same shapes
- THEN the native moment triple SHALL match the OCCT moments within a scale-relative tolerance (order-insensitive, both sorted), AND each native principal axis SHALL match the OCCT axis up to sign — except a degenerate frame (`I₁≈I₂≈I₃`, e.g. a sphere or cube where any orthonormal frame is principal), which SHALL be compared on the moments only with the axis frame accepted as long as it is orthonormal and diagonalizes the tensor

#### Scenario: Inertia of a non-watertight body declines honestly (host + sim)

- GIVEN a body that is not robustly watertight (an open shell, a non-manifold solid, or one whose net signed volume is `≤ 0`)
- WHEN the principal moments are requested
- THEN `NativeEngine::principal_moments` SHALL DECLINE (fall back to OCCT), emit NO native inertia value, weaken no tolerance, and leave `src/native/**` OCCT-free — a wrong mass property is NEVER emitted for a body whose triangulation the volume self-verify cannot bless

### Requirement: Standalone native B-rep validity checker, OCCT-free, matching OCCT BRepCheck_Analyzer on valid and broken fixtures

The library SHALL expose an additive `cc_check_solid(body, CCValidityReport*)` facade
that produces, with **no OCCT**, a structured validity report over a native solid with
independent, individually-decidable verdicts for: **finite coordinates** (every
topology leaf point/pole is `std::isfinite`); **closed 2-manifold** (every undirected
mesh edge is used by EXACTLY two triangles — `tessellate/mesh.h::isWatertight` /
`edgeUseCounts`, rejecting both boundary and non-manifold edges); **consistent outward
orientation** (a uniform signed-tetra winding giving a positive enclosed volume — a
flipped face breaks the exactly-twice pairing or flips a fan sign); **no degenerate
face or edge** (no zero-area triangle `‖(b−a)×(c−a)‖ ≈ 0`, no zero-length edge
`‖b−a‖ ≈ 0`); and **no self-intersection** (GS3 `analysis/distance.h` minimum distance
over non-adjacent triangle pairs with an AABB broad-phase — an intersecting pair has
distance 0 with interior-crossing witnesses). The report SHALL carry an overall `valid`
flag (the conjunction of all checks) and a `first_failure` code naming the first
failing check, reusing the landed tessellator + topology + GS3 distance machinery.

The checker SHALL **DECLINE honestly** where a robust check is not reachable: when the
body carries a general trimmed freeform patch whose no-self-intersection the mesh test
cannot certify, `cc_check_solid` SHALL return `0` with the report marked
`decided = 0`, `first_failure` = the self-intersection-undecidable code, and
`cc_last_error` set — it SHALL NEVER report `valid = 1` for a body it cannot verify.
`cc_check_solid` SHALL return `1` only when a full report was produced (`decided = 1`),
with `valid` set to the conjunction of the per-check verdicts. `CCValidityReport` and
`cc_check_solid` SHALL be the ONLY additions to the ABI (every pre-existing struct and
signature byte-identical); `src/native/**` SHALL remain OCCT-free; the OCCT
`BRepCheck_Analyzer` oracle SHALL exist only in the sim verification harness.

#### Scenario: A valid solid and each deliberately-broken fixture report the correct verdict (host, no OCCT)

- GIVEN hand-built native fixtures of KNOWN state — a valid closed box/tetra, a non-closed shell (one face removed), a flipped-face solid, a zero-area/zero-length degenerate, and a self-intersecting polyhedron — built on the host with no OCCT
- WHEN `cc_check_solid` is called on each
- THEN the valid solid SHALL report `valid = 1` (`decided = 1`, all checks pass); the non-closed shell SHALL report `closed_manifold = 0`; the flipped-face solid SHALL report `consistent_orientation = 0`; the degenerate SHALL report `no_degenerate = 0`; the self-intersecting polyhedron SHALL report `no_self_intersection = 0`; and each broken fixture's `first_failure` SHALL name its specific invalidity with `valid = 0`

#### Scenario: The native validity verdict matches OCCT BRepCheck_Analyzer on valid and broken shapes (sim)

- GIVEN a valid solid AND deliberately-broken shapes (a non-closed shell, a flipped face, a self-intersecting wire), on a booted iOS simulator
- WHEN the native `cc_check_solid` overall verdict is compared to OCCT `BRepCheck_Analyzer::IsValid` on the same shapes
- THEN the native overall `valid` SHALL equal OCCT's valid/invalid verdict on every fixture (valid → valid; each broken → invalid), with the native report additionally naming the specific invalidity that OCCT flags

#### Scenario: An uncertifiable freeform solid declines rather than reporting a false valid (host + sim)

- GIVEN a body carrying a general trimmed freeform patch whose no-self-intersection the polyhedral mesh test cannot certify
- WHEN `cc_check_solid` is requested
- THEN it SHALL return `0` with `decided = 0`, `first_failure` set to the self-intersection-undecidable code, `cc_last_error` set, and `valid` NEVER `1` — the honest deferral is reported, not a fabricated "valid" verdict, and no tolerance is weakened


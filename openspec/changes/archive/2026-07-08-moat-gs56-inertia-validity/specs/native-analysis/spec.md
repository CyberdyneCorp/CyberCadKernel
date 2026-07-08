# native-analysis

## ADDED Requirements

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

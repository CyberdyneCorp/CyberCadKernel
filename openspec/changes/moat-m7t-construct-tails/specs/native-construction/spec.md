# native-construction

## MODIFIED Requirements

### Requirement: Native twisted sweep reduces to the plain sweep (no-op only)

The native construction library SHALL serve `cc_twisted_sweep` (`twistRadians`,
`scaleEnd`) NATIVELY for the plain-sweep reduction (`twistRadians ≈ 0` AND
`scaleEnd ≈ 1`, forwarding to `build_sweep`) AND for a **genuine twist / scale** on a
straight spine: `build_twisted_sweep` SHALL DENSIFY the spine so each ruled band's
accumulated twist stays under a small per-band bound, place the profile at every
station through the per-station Frenet section frame (`up = (0,1,0)`,
`nrm = normalize(tan × up)` with a `+X` fallback, `u' = u·cosθ − v·sinθ`,
`v' = u·sinθ + v·cosθ` — the literal OCCT `twisted_sweep` section law) after that
station's linear scale (`1 → scaleEnd`) and accumulated twist (`0 → twistRadians`), and
tile the rings into a watertight `Solid` via `assembleRingTube`. The volume SHALL
converge, as the spine is densified, to the area-preserving value `profileArea · L` for
a pure twist (the cross-section area is invariant under an in-plane rotation) and to the
scaled-frustum value for a twist + scale.

`build_twisted_sweep` SHALL return a NULL `Shape` (→ `NativeEngine` falls through to the
fallback OCCT `ThruSections`) when the twisted tube would SELF-FOLD — at any consecutive
station pair the outermost profile point's twist-induced lateral arc exceeds the axial
advance (`detail::sectionSweepUnsafe`) — or on a degenerate profile / fewer than two
path points / a non-positive station scale. A twist COMBINED WITH a simultaneous scale
(the section rotates AND shrinks along the spine) yields a saddle band the native mesher
welds only INTERMITTENTLY across the deflection ladder; that candidate SHALL be built but
SHALL FAIL the engine `robustlyWatertight` self-verify and be DISCARDED → OCCT (a
measured, sharpened decline — the pure twist stays native). There SHALL be no faked
twist path and no widened tolerance. This builder SHALL remain OCCT-free and
host-buildable.

#### Scenario: A zero-twist unit-scale sweep is the plain native prism (host)
- GIVEN a square profile, a straight path of length `L`, `twistRadians = 0`, `scaleEnd = 1`, built on the host with no OCCT
- WHEN the twisted sweep is invoked and tessellated
- THEN it SHALL forward to the native sweep, the mesh SHALL be watertight AND the volume SHALL equal the plain prism value `profileArea · L` exactly

#### Scenario: A real twist builds a watertight tube converging to the area-preserving volume (host)
- GIVEN a square profile of area `A` swept along a straight spine of length `L` with a non-zero `twistRadians` and `scaleEnd = 1`, built on the host with no OCCT
- WHEN `build_twisted_sweep` densifies the spine, builds the twisted ruled tube, and it is tessellated across the deflection ladder
- THEN the mesh SHALL be watertight AND the enclosed volume SHALL converge to `A · L` within the deflection-bounded tolerance (twist preserves the cross-section area)

#### Scenario: A twist combined with a scale is honestly declined (host)
- GIVEN a square profile swept along a straight spine with a non-zero `twistRadians` AND a `scaleEnd ≠ 1`, built on the host with no OCCT
- WHEN the twisted+scaled candidate is built and tessellated across the deflection ladder
- THEN its enclosed volume SHALL converge to the linearly-scaled swept-section estimate (the geometry is correct) BUT the mesh SHALL NOT be watertight at every deflection, so the engine `robustlyWatertight` self-verify SHALL DISCARD it and fall through to the fallback (a measured decline, not a faked pass)

#### Scenario: A self-folding twist falls through (host)
- GIVEN a twisted sweep whose per-band twist arc at the profile rim exceeds the axial advance (a self-folding tube), built on the host
- WHEN `build_twisted_sweep` is invoked
- THEN it SHALL return a NULL `Shape` (no self-intersecting solid) AND `NativeEngine` SHALL fall through to the fallback for that call

#### Scenario: A real-twist sweep matches the OCCT ThruSections oracle (parity)
- GIVEN a square profile swept along a densified straight spine with a real `twistRadians` (+ optional `scaleEnd`), the SAME spine stations fed to both engines, on a booted iOS simulator
- WHEN it is called native (`cc_set_engine(1)`) vs OCCT (`cc_set_engine(0)`)
- THEN the two shapes' volume / area / watertightness / Euler characteristic (χ = 2) / bounding box SHALL agree within the documented tolerance (both build the same ruled ThruSections from identical stations)

### Requirement: Native loft along rail is native for a straight AND a smooth curved rail

The native construction library SHALL serve `cc_loft_along_rail(rail, A, B)` NATIVELY
for a STRAIGHT rail (the perpendicular-framed ruled loft between equal-count sections
`A`, `B` placed in the plane perpendicular to the single rail tangent — EXACT vs OCCT
`MakePipeShell` on a straight rail) AND for a SMOOTH CURVED rail: `build_loft_along_rail`
SHALL DENSIFY the rail so each band's tangent turn stays under a small per-band bound,
transport the section along the rail with a rotation-minimizing frame
(`detail::rmfFrames`, zero spurious twist about the tangent), morph section `A` into
section `B` by linear per-vertex interpolation in the transported frame, and tile the
rings into a watertight `Solid` via `assembleRingTube`. For a circular-arc rail of
radius `R` through angle `φ` with a constant section of area `Aprof`, the volume SHALL
converge, as the rail is densified, to the Pappus torus-sector value `Aprof · R · φ`.

`build_loft_along_rail` SHALL return a NULL `Shape` (→ `NativeEngine` falls through to
OCCT `MakePipeShell`) on mismatched section counts (`< 3` or `A ≠ B`), a degenerate rail
or section, or a degenerate rail tangent. A rail whose curvature is so tight that the
transported ruled tube does not weld watertight at the densification cap — or a
COARSE / SHARP-CORNERED section whose low-vertex-count ruled bands do not weld along a
curved rail — SHALL fail the engine's `robustlyWatertight` self-verify and be DISCARDED →
OCCT (a well-sampled smooth section along a smooth rail welds robustly and stays native).
There SHALL be no faked morph and no widened tolerance. This builder SHALL remain
OCCT-free and host-buildable.

#### Scenario: A straight-rail loft is the exact perpendicular-framed ruled loft (host)
- GIVEN two equal-count planar sections lofted along a straight rail, built on the host with no OCCT
- WHEN `build_loft_along_rail` is invoked and tessellated
- THEN it SHALL build the perpendicular-framed ruled loft, the mesh SHALL be watertight AND its volume SHALL match the analytic ruled-loft value

#### Scenario: A circular-arc rail loft converges to the Pappus torus-sector volume (host)
- GIVEN a constant well-sampled polygonal section of area `Aprof` lofted along a circular-arc rail of radius `R` through angle `φ` (with `R` well above the section extent), built on the host with no OCCT
- WHEN `build_loft_along_rail` densifies the rail, builds the RMF-transported tube, and it is tessellated across the deflection ladder
- THEN the mesh SHALL be watertight AND the enclosed volume SHALL converge to `Aprof · R · φ` within the deflection-bounded tolerance

#### Scenario: A too-tight curved rail falls through (host)
- GIVEN a curved rail whose per-band turn cannot weld watertight within the densification cap, built on the host
- WHEN `build_loft_along_rail` is invoked (and, where it returns a candidate, the engine self-verifies)
- THEN NO native solid SHALL be served (NULL `Shape`, or a candidate DISCARDED by `robustlyWatertight`) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: A curved-rail loft matches the OCCT MakePipeShell oracle (parity)
- GIVEN two equal-count sections lofted along a densified smooth curved rail, the SAME rail stations fed to both engines, on a booted iOS simulator
- WHEN it is called native (`cc_set_engine(1)`) vs OCCT (`cc_set_engine(0)`)
- THEN the two shapes' volume / area / watertightness / Euler characteristic (χ = 2) / bounding box SHALL agree within the documented tolerance

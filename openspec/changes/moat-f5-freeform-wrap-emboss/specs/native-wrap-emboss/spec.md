# native-wrap-emboss

## ADDED Requirements

### Requirement: Native sphere-cap pole-boss (freeform/curved base) emboss

The kernel SHALL provide a native, **OCCT-free** wrap-emboss arm that computes
`cc_wrap_emboss(body, faceId, profileXY, count, height, boss)` NATIVELY for a RAISED pole cap
on a sphere-cap dome when ALL of the following hold: `boss == 1` (emboss / add material),
`body` is a native solid, `faceId` resolves to a `FaceSurface`-kind-`Sphere` LATERAL face,
the body is a PURE sphere-cap dome (recognised WHOLESALE: every face is a coaxial sphere of
the SAME centre and radius `R`, or EXACTLY ONE axis-normal planar cap whose plane actually
cuts the ball), and the pole-cap half-angle `φ0 = ρ / R` — with `ρ` the profile's arc-length
in-radius (half the smaller footprint bounding-box extent) — satisfies `0 < φ0 < φcap` where
`φcap = acos(capA / R)` is the dome's own polar opening (the boss disc sits strictly inside
the dome, away from the rim).

For such an input the builder SHALL:
- **wrap the pattern as an axisymmetric pole DISC** of half-angle `φ0` centred on the dome
  pole (keeping the raised region an exact spherical-shell sector);
- **build the raised boss** as a deflection-bounded planar-facet soup: the base dome wall
  retiled from the cap latitude up to the `φ0` rim latitude at radius `R`, an OUTER spherical
  cap patch over `φ ∈ [0, φ0]` at radius `R + height`, an annular RIM frustum from `R` to
  `R + height` along the `φ0` circle, and the flat disc cap at the cap plane, ALL sharing the
  same `N` longitude samples so the rim seam welds; and
- **WELD** the soup into a watertight solid through the native `src/native/boolean`
  `assembleSolid`, with the tessellator UNTOUCHED.

The builder SHALL expose `spherePoleBossVolumeDelta(...)` returning the EXACT closed-form
embossed-volume delta `ΔV = 2π(1 − cos φ0)·((R+height)³ − R³)/3` (a spherical-shell sector),
or `0` when the picked face is not a recognised sphere-cap dome wall or the boss is out of
scope. The engine self-verify (`wrapEmbossVerified`) SHALL, for a recognised sphere-cap base,
require `tess::isConsistentlyOriented` and gate the result's meshed volume against
`v0 + ΔV` (deflection-bounded band), DISCARDING any candidate that fails.

Anything outside this arm SHALL return a NULL Shape (→ OCCT): a sphere DEBOSS, a `φ0` that
reaches the rim, a spherical ZONE (two caps), an off-centre / multi-radius sphere, a CONE or
general free-form (spline-surface) base. This path SHALL remain OCCT-free and SHALL NOT change
the `cc_*` ABI.

#### Scenario: Sphere-cap pole boss matches the closed-form shell sector (host)

- GIVEN a native hemisphere dome (R=10, cap at the equator) and a square footprint with arc-length in-radius ρ=3 (φ0=0.3), raised by height=2, native engine active and no OCCT
- WHEN `wrap_emboss` builds the pole boss
- THEN the result SHALL be a watertight solid whose meshed enclosed volume matches `v0 + 2π(1 − cos 0.3)·((12)³ − (10)³)/3` to within the deflection band (< 1.5%), and `spherePoleBossVolumeDelta` SHALL equal the independent closed form to 1e-9

#### Scenario: Sphere-cap pole boss vs OCCT reference (simulator)

- GIVEN a sphere-cap dome reconstructed through the `cc_*` facade on a booted iOS simulator, for three (R, capOff, ρ, height) fixtures
- WHEN the native pole boss is built (`cc_set_engine(1)`) and OCCT's `cc_wrap_emboss` is invoked on the SAME sphere wall (`cc_set_engine(0)`)
- THEN OCCT's `cc_wrap_emboss` SHALL DECLINE (return 0) on the non-cylindrical face, AND the native boss volume SHALL match both the closed form and `oBase + ΔV`, where `oBase` is the base dome volume measured by OCCT `BRepGProp` (`cc_mass_properties` under the OCCT engine), within 2%

#### Scenario: Out-of-scope curved base declines to OCCT

- GIVEN a native sphere-cap dome and either a DEBOSS request, a footprint whose pole cap reaches the rim (φ0 ≥ φcap), or a spherical ZONE body (two cap planes)
- WHEN `wrap_emboss` is invoked
- THEN it SHALL return a NULL Shape and `spherePoleBossVolumeDelta` SHALL return 0, so the engine falls through to OCCT rather than emitting a wrong or leaky solid

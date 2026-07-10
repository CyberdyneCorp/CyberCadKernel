# native-blend

## ADDED Requirements

### Requirement: Native `cc_shell` hollows a sphere-cap dome with a concentric curved wall

The engine SHALL provide a NATIVE, OCCT-free path for `cc_shell(body, faceIds, faceCount,
thickness)` on a body that is EXACTLY a SPHERE-CAP dome — one coaxial Sphere wall (single
centre, single radius Ro) closed at the pole and cut by EXACTLY ONE axis-normal planar cap —
with that single cap removed (opened). The engine SHALL hollow the body to a uniform wall
`thickness` by offsetting the sphere wall inward analytically to the CONCENTRIC sphere of
radius Ri = Ro − t (the offset of a sphere is a concentric sphere), running from the pole down
to the SAME cap plane, leaving the removed cap flush (the opening). The hollow bowl (outer
sphere wall, concentric inner sphere wall, open-rim wall-thickness annulus at the cap) SHALL be
welded watertight through the existing planar-facet assembly (the tessellator is NOT
modified). A produced candidate SHALL be accepted ONLY under the engine's SHRINK self-verify
(watertight, consistently oriented, enclosed volume strictly less than the solid dome). This
path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE — every face a coaxial Sphere of the
SAME centre and radius, plus axis-normal planes at EXACTLY ONE height (the cap) — so a rebuilt
solid of a DIFFERENT shape can never pass the volume self-verify.

#### Scenario: Hemisphere bowl shells natively at the closed-form wall volume (host)

- GIVEN a native hemisphere dome (radius Ro, equatorial cap) with the cap open, the native engine active and no OCCT
- WHEN `cc_shell(B, {cap}, 1, t)` is invoked with t < Ro
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly less than the solid hemisphere AND matches the closed-form wall volume `(2π/3)(Ro³ − (Ro−t)³)` to the deflection bound, converging monotonically as the deflection is refined

#### Scenario: Spherical-cap dome (shallow / deep) shells natively (host)

- GIVEN a native spherical-cap dome cut by ONE axis-normal cap plane at axial coord `a` from the centre (a > 0 shallow, a < 0 deep), with the cap open, native engine active
- WHEN `cc_shell` is invoked opening the cap with wall t
- THEN the native op SHALL return a watertight solid whose enclosed volume is strictly less than the solid dome AND matches the closed-form wall volume `seg(Ro,a) − seg(Ro−t,a)` with `seg(R,a)=π(2R³/3 − R²a + a³/3)` to the deflection bound

#### Scenario: Out-of-envelope sphere shell bodies are honestly declined (host)

- GIVEN a native body that is a sphere-cap dome with the SPHERE WALL picked, a pick of zero faces, a thickness that collapses the cavity (t ≥ Ro or the inner sphere no longer crosses the cap), or a spherical ZONE (two axis-normal cap planes), with the native engine active and no OCCT
- WHEN `cc_shell` (sphere arm) is invoked
- THEN the sphere-shell builder SHALL return a NULL result (falls through to OCCT `BRepOffsetAPI_MakeThickSolid`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native sphere-cap shell matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a sphere-cap dome built via `cc_solid_revolve_profile` (an on-axis arc plus an axis-closing edge) with its cap picked
- WHEN `cc_shell` is run once under the OCCT engine (oracle, `BRepOffsetAPI_MakeThickSolid` + `BRepGProp`) and once under the NativeEngine
- THEN the native result SHALL be watertight AND its `cc_mass_properties` volume/area SHALL match OCCT and the exact closed-form wall volume to the deflection bound (volume rel < 2e-2, area rel < 4e-2), with the volume strictly below the solid dome

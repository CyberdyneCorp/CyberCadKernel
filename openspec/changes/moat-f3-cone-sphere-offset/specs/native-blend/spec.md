# native-blend

## ADDED Requirements

### Requirement: Native `cc_offset_face` offsets a cone-frustum wall to a coaxial cone

The engine SHALL provide a NATIVE, OCCT-free path for `cc_offset_face(body, faceId, distance)`
when `faceId` is the CONE lateral wall of a body that is EXACTLY a capped cone frustum — every
face a coaxial Cone of the SAME semi-angle σ and reference radius, plus axis-normal planes at
EXACTLY TWO distinct heights (the two disc caps), with both original cap radii strictly
positive. The engine SHALL offset the wall by `distance` along its outward (radial, tilted-by-σ)
normal, which produces a COAXIAL cone of the SAME σ whose radius at every height shifts by
`distance/cosσ`; it SHALL rebuild the capped frustum at `Rref + distance/cosσ` with the cap
heights fixed (wall band + two disc caps at the shifted radii) as a planar-facet soup welded
watertight through the existing planar-facet assembly (the tessellator is NOT modified). A
produced candidate SHALL be accepted ONLY under the engine's correctly-signed self-verify
(watertight, consistently oriented, enclosed volume strictly greater than the sharp original
for a grow `distance>0` and strictly less but positive for a shrink `distance<0`). This path
SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE, so a rebuilt solid of a DIFFERENT shape
can never pass the volume self-verify.

#### Scenario: Narrowing cone-frustum wall grows to a coaxial fatter frustum (host)

- GIVEN a native capped cone frustum (bottom radius Rb, top radius Rt, height H, Rb≠Rt) with the cone wall picked, the native engine active and no OCCT
- WHEN `cc_offset_face(B, wall, d)` is invoked with d > 0
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly greater than the sharp frustum AND matches the closed-form frustum volume `πH/3·(Rb'²+Rb'·Rt'+Rt'²)` with `Rb'=Rb+d/cosσ`, `Rt'=Rt+d/cosσ`, `tanσ=(Rt−Rb)/H` to the deflection bound

#### Scenario: Widening cone-frustum wall shrinks inward (host)

- GIVEN a native capped cone frustum with the cone wall picked, native engine active
- WHEN `cc_offset_face(B, wall, d)` is invoked with d < 0 leaving both caps positive
- THEN the native op SHALL return a watertight solid whose enclosed volume is strictly less than the sharp frustum AND matches the shifted-radius closed form to the deflection bound

#### Scenario: Out-of-envelope cone offsets are honestly declined (host)

- GIVEN a native body with a picked PLANAR cap of a frustum, a shrink that inverts a cap (`Rt + d/cosσ ≤ 0`), a stepped / multi-cone shaft, or a full cone through the apex, with the native engine active and no OCCT
- WHEN `cc_offset_face` (cone arm) is invoked
- THEN the cone-offset builder SHALL return a NULL result (falls through to OCCT `BRepOffsetAPI`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native cone-frustum offset matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a capped frustum built via `cc_solid_revolve` with its cone wall picked
- WHEN `cc_offset_face` is run under the NativeEngine and the ground-truth oracle is built directly with OCCT `BRepPrimAPI_MakeCone(Rb+d/cosσ, Rt+d/cosσ, H)` + `BRepGProp`
- THEN the native result SHALL be watertight with Euler χ=2 AND its `cc_mass_properties` volume/area SHALL match OCCT and the exact frustum closed form to the deflection bound (volume rel < 1e-2, area rel < 3e-2), with the volume in the correct grow/shrink direction

### Requirement: Native `cc_offset_face` offsets a sphere wall to a concentric sphere

The engine SHALL provide a NATIVE, OCCT-free path for `cc_offset_face(body, faceId, distance)`
when `faceId` is the SPHERE lateral wall of a body that is EXACTLY a sphere-cap dome — every
face a coaxial Sphere of the SAME centre and radius R, plus EXACTLY ONE distinct axis-normal
planar cap that cuts the ball. The engine SHALL offset the wall by `distance` to the CONCENTRIC
sphere of radius `R + distance` (the offset of a sphere is a concentric sphere), keeping the
SAME cap plane fixed (its rim radius follows to `√((R+distance)²−a²)`), and SHALL rebuild the
dome (sphere-wall latitude bands from the pole down to the cap plane + one disc cap) as a
planar-facet soup welded watertight through the existing planar-facet assembly (the tessellator
is NOT modified). A produced candidate SHALL be accepted ONLY under the engine's
correctly-signed self-verify (watertight, consistently oriented, volume strictly greater than
the original dome for a grow and strictly less but positive for a shrink). This path SHALL
remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE (matching the fragmented sphere-wall and
disc-cap sectors of a full revolve by geometry), so a rebuilt solid of a DIFFERENT shape can
never pass the volume self-verify.

#### Scenario: Hemisphere sphere wall grows to a concentric bigger dome (host)

- GIVEN a native hemisphere dome (radius R, equatorial cap) with the sphere wall picked, the native engine active and no OCCT
- WHEN `cc_offset_face(B, wall, d)` is invoked with d > 0
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly greater than the solid hemisphere AND matches the spherical-segment closed form `π(2(R+d)³/3 − (R+d)²·a + a³/3)` at cap axial coord a to the deflection bound

#### Scenario: Shallow-cap / deep sphere-cap dome shrinks inward (host)

- GIVEN a native spherical-cap dome cut by ONE axis-normal cap plane at axial coord `a` from the centre (a > 0 shallow, a < 0 deep), with the sphere wall picked, native engine active
- WHEN `cc_offset_face(B, wall, d)` is invoked with d < 0 leaving |a| < R+d
- THEN the native op SHALL return a watertight solid whose enclosed volume is strictly less than the original dome AND matches `seg(R+d, a)` to the deflection bound, converging monotonically as the builder deflection refines

#### Scenario: Out-of-envelope sphere offsets are honestly declined (host)

- GIVEN a native body with a picked PLANAR cap, a shrink through the cap plane (`|a| ≥ R+d`), `R + d ≤ 0`, a spherical ZONE (two axis-normal caps), or an off-centre / multi-radius sphere, with the native engine active and no OCCT
- WHEN `cc_offset_face` (sphere arm) is invoked
- THEN the sphere-offset builder SHALL return a NULL result (falls through to OCCT) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native sphere-cap-dome offset matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a sphere-cap dome built via `cc_solid_revolve_profile` (an on-axis arc + a base disc) with its sphere wall picked
- WHEN `cc_offset_face` is run under the NativeEngine and the ground-truth oracle is the exact spherical-segment closed form with an OCCT `BRepPrimAPI_MakeSphere(R+d)` cross-check
- THEN the native result SHALL be watertight with Euler χ=2 AND its `cc_mass_properties` volume SHALL match the closed form to the deflection bound (volume rel < 2e-2), with the volume in the correct grow/shrink direction

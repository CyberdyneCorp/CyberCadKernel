# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_edges` rounds a convex sphere ↔ coaxial-cap circular rim

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_edges(body, edgeIds, 1,
radius)` on the CONVEX circular rim where a coaxial SPHERE lateral face meets a coaxial
planar cap (a truncated ball / dome / spherical plug capped by a flat top). On a native body
of revolution whose picked edge is such a rim, the engine SHALL build the blend as a coaxial
TORUS band — major radius equal to the rolling-ball centre radius, minor radius `radius` —
swept over the minor angle from the sphere-wall seam to the cap seam, G1-tangent to both
neighbour faces at the two seams, welded watertight to the rebuilt (faceted) sphere wall and
trimmed cap through the existing planar-facet assembly (the tessellator is NOT modified). A
produced candidate SHALL be accepted ONLY under the engine's SHRINK self-verify (watertight,
consistently oriented, enclosed volume strictly less than the sharp truncated ball). This
path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE — every face a coaxial sphere of the
SAME radius / centre, or an axis-normal plane at exactly ONE height (the cap), with the rim
radius matching the sphere radius at the rim height (Rrim = √(R²−h²)) — so a rebuilt solid of
a DIFFERENT shape can never pass the volume self-verify. The rolling-ball centre SHALL lie on
a coaxial circle (centre-to-sphere-centre distance R−r, axial coord cap−r); the ring-torus
guard (centre-circle radius ≥ r) and the sphere-seam-stays-below-the-cap guard SHALL hold or
the builder SHALL decline.

#### Scenario: Convex sphere cap rim fillets natively and converges to the closed form (host)

- GIVEN a native truncated ball (sphere radius R centred on the axis, a flat cap at axial height h < R so the top is cut off) whose rim is a circle of radius √(R²−h²) shared by the sphere wall and the coaxial cap, with the native engine active and no OCCT
- WHEN `cc_fillet_edges(B, {rim}, 1, r)` is invoked with the ball-centre radius ≥ r
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly less than the sharp truncated ball AND matches the closed-form removed volume (Pappus of the corner-minus-arc region) to the deflection bound, converging monotonically as the deflection is refined

#### Scenario: Out-of-envelope sphere rims are honestly declined (host)

- GIVEN a native body that is a pure cylinder or cone frustum (no sphere face), a truncated ball whose rolling-ball centre circle radius is < r, a body mixing a sphere with a cylinder/cone, a planar box, a zero/negative radius, or a multi-edge pick, with the native engine active and no OCCT
- WHEN `cc_fillet_edges` (sphere arm) is invoked
- THEN the sphere builder SHALL return a NULL result (the cylinder / cone are served by their own arms; everything else falls through to OCCT `BRepFilletAPI_MakeFillet`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native sphere fillet matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a truncated ball built via `cc_solid_revolve_profile` (an arc segment capped by a line), and its cap rim
- WHEN `cc_fillet_edges` is run once under the OCCT engine (oracle, `BRepFilletAPI_MakeFillet` + `BRepGProp`) and once under the NativeEngine
- THEN the native result SHALL be watertight AND its `cc_mass_properties` volume/area SHALL match OCCT and the exact closed form to the deflection bound (volume rel < 1e-2, area rel < 2e-2), with the volume strictly below the sharp truncated ball

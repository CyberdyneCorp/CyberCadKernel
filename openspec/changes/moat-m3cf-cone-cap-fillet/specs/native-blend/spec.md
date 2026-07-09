# native-blend

## ADDED Requirements

### Requirement: Native `cc_fillet_edges` rounds a convex cone-frustum ↔ coaxial-cap circular rim

The engine SHALL provide a NATIVE, OCCT-free path for `cc_fillet_edges(body, edgeIds, 1,
radius)` on the CONVEX circular rim where a coaxial CONE-FRUSTUM lateral face meets a coaxial
planar cap (a tapered plug / tapered boss / truncated cone capped at one end). On a native
body of revolution whose picked edge is such a rim, the engine SHALL build the blend as a
coaxial TORUS band — major radius equal to the rolling-ball centre radius, minor radius
`radius` — swept over the tilted minor angle from the cone-wall seam to the cap seam,
G1-tangent to both neighbour faces at the two seams, welded watertight to the rebuilt cone
wall and trimmed cap through the existing planar-facet assembly (the tessellator is NOT
modified). A produced candidate SHALL be accepted ONLY under the engine's SHRINK self-verify
(watertight, consistently oriented, enclosed volume strictly less than the sharp frustum).
The cylinder is the σ=0 special case and is handled by the existing cylinder-rim path; this
path SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE — every face a coaxial cone of the SAME
half-angle / reference radius, or an axis-normal plane at one of exactly two distinct heights,
with the rim radius matching the cone radius at the rim height — so a rebuilt solid of a
DIFFERENT shape can never pass the volume self-verify.

#### Scenario: Convex cone-frustum cap rim fillets natively and converges to the closed form (host)

- GIVEN a native capped cone frustum (bottom radius Rb, top radius Rt, height H, Rb≠Rt) whose top rim is a circle of radius Rt shared by the cone wall and the coaxial top cap, with the native engine active and no OCCT
- WHEN `cc_fillet_edges(B, {rim}, 1, r)` is invoked with the ball-centre radius ≥ r
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly less than the sharp frustum AND matches the closed-form removed volume (Pappus of the corner-minus-arc region) to the deflection bound, converging monotonically as the deflection is refined

#### Scenario: Widening and narrowing tapers both land (host)

- GIVEN a narrowing frustum (Rt<Rb) and a widening frustum (Rt>Rb), each with a valid ring-torus ball-centre radius, native engine active
- WHEN `cc_fillet_edges` is invoked on the top rim of each
- THEN both SHALL fillet natively (watertight, volume reduced, closed-form match), the widening case using a negative wall-seam minor angle

#### Scenario: Out-of-envelope cone rims are honestly declined (host)

- GIVEN a native body that is a pure cylinder (σ=0), a steep frustum whose ball-centre radius is < r (spindle), a stepped shaft, a multi-frustum (two cones with different half-angles), a planar box, a zero/negative radius, or a multi-edge pick, with the native engine active and no OCCT
- WHEN `cc_fillet_edges` (cone arm) is invoked
- THEN the cone builder SHALL return a NULL result (the cylinder is served by the cylinder-rim arm; everything else falls through to OCCT `BRepFilletAPI_MakeFillet`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native cone fillet matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a capped cone frustum built via `cc_solid_revolve`, and its top rim
- WHEN `cc_fillet_edges` is run once under the OCCT engine (oracle, `BRepFilletAPI_MakeFillet` + `BRepGProp`) and once under the NativeEngine
- THEN the native result SHALL be watertight AND its `cc_mass_properties` volume/area SHALL match OCCT and the exact closed form to the deflection bound (volume rel < 1e-2, area rel < 2e-2), with the volume strictly below the sharp frustum

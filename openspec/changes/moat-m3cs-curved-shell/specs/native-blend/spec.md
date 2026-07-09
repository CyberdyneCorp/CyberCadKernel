# native-blend

## ADDED Requirements

### Requirement: Native `cc_shell` hollows a capped cylinder / cone frustum with a constant curved wall

The engine SHALL provide a NATIVE, OCCT-free path for `cc_shell(body, faceIds, faceCount,
thickness)` on a body that is EXACTLY a capped CYLINDER or capped CONE FRUSTUM (one coaxial
curved wall plus two axis-normal planar caps) with EXACTLY ONE planar cap removed (opened).
The engine SHALL hollow the body to a uniform wall `thickness` by offsetting the curved wall
inward analytically — a cylinder of radius Rc to a coaxial inner cylinder of radius Rc−t; a
cone of half-angle σ to a coaxial inner cone whose reference radius drops by t/cosσ (the
perpendicular inward offset) — and the kept planar cap inward by `thickness`, leaving the
removed cap flush (the opening). The hollow tube (outer wall, kept-cap outer disk, inner
offset wall, kept-cap inner disk, open-end wall-thickness annulus) SHALL be welded watertight
through the existing planar-facet assembly (the tessellator is NOT modified). A produced
candidate SHALL be accepted ONLY under the engine's SHRINK self-verify (watertight,
consistently oriented, enclosed volume strictly less than the solid body). This path SHALL
remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE — every face a single coaxial cylinder
(one radius) OR a single coaxial cone (one σ / reference radius), plus axis-normal planes at
exactly two distinct heights — so a rebuilt solid of a DIFFERENT shape can never pass the
volume self-verify.

#### Scenario: Capped cylinder shells natively at the closed-form wall volume (host)

- GIVEN a native capped cylinder (radius Rc, height H) with one cap open, the native engine active and no OCCT
- WHEN `cc_shell(B, {open cap}, 1, t)` is invoked with t < Rc and t < H
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume is strictly less than the solid cylinder AND matches the closed-form wall volume `π·Rc²·H − π·(Rc−t)²·(H−t)` to the deflection bound, converging monotonically as the deflection is refined

#### Scenario: Capped cone frustum shells natively (host)

- GIVEN a native capped cone frustum (base radius Rb, top radius Rt, height H, Rb≠Rt) with one cap open, native engine active
- WHEN `cc_shell` is invoked opening the top cap with wall t
- THEN the native op SHALL return a watertight solid whose enclosed volume is strictly less than the solid frustum AND matches the closed-form wall volume (outer frustum minus the inner frustum offset inward by t/cosσ, the cavity running from the kept inner face to the open end) to the deflection bound

#### Scenario: Out-of-envelope shell bodies are honestly declined (host)

- GIVEN a native body that is a stepped shaft (multi-cylinder), a planar box, a capped cylinder with BOTH caps picked, a capped cylinder with the CURVED WALL picked, a pick of zero faces, or a thickness that collapses the cavity (t ≥ radius or t ≥ height), with the native engine active and no OCCT
- WHEN `cc_shell` (curved arm) is invoked
- THEN the curved-shell builder SHALL return a NULL result (the planar box is served by the planar-shell arm; everything else falls through to OCCT `BRepOffsetAPI_MakeThickSolid`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native curved shell matches the OCCT oracle on the simulator (sim)

- GIVEN the cc_* facade on a booted iOS simulator, a capped cylinder built via `cc_solid_extrude_profile` and a capped cone frustum via `cc_solid_revolve`, each with its top cap picked
- WHEN `cc_shell` is run once under the OCCT engine (oracle, `BRepOffsetAPI_MakeThickSolid` + `BRepGProp`) and once under the NativeEngine
- THEN the native result SHALL be watertight AND its `cc_mass_properties` volume/area SHALL match OCCT and the exact closed-form wall volume to the deflection bound (volume rel < 2e-2, area rel < 4e-2), with the volume strictly below the solid body

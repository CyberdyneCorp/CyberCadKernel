# native-blend

## ADDED Requirements

### Requirement: Native `cc_offset_face` offsets a cylinder lateral wall radially

The engine SHALL provide a NATIVE, OCCT-free path for `cc_offset_face(body, faceId, distance)`
when the picked face is the CYLINDER LATERAL wall of a capped cylinder: the wall SHALL move to
radius `Rc + distance` (the coaxial cylinder that is the offset of a cylinder surface) and the
two coaxial planar caps SHALL grow/shrink to match, keeping the axial extent. The result SHALL
be rebuilt as a deflection-bounded planar-facet soup (wall band + two disc caps) welded
watertight through the existing planar-facet assembly (the tessellator is NOT modified), and
accepted ONLY under the engine's correctly-signed volume self-verify (grow `distance>0` →
Vr>Vo; shrink `distance<0` → 0<Vr<Vo, both watertight and consistently oriented). This path
SHALL remain OCCT-free and SHALL NOT change the `cc_*` ABI.

The native builder SHALL recognise the body WHOLESALE — every face a coaxial cylinder of the
SAME axis / radius, or an axis-normal disc plane at exactly two heights — so a rebuilt solid of
a DIFFERENT shape can never pass the volume self-verify. `Rc + distance` SHALL be strictly
positive or the builder SHALL decline. A picked PLANAR face is served by the planar
`offset_face` arm; a cone / sphere / stepped-shaft / multi-radius / tilted-cap body → NULL →
OCCT (`BRepOffsetAPI`).

#### Scenario: Cylinder wall grows/shrinks natively to the closed form (host)

- GIVEN a native capped cylinder (radius Rc, axial length H, two axis-normal caps), the native engine active and no OCCT
- WHEN `cc_offset_face(B, wallFace, d)` is invoked with `Rc + d > 0`
- THEN the native op SHALL return a watertight, consistently-oriented solid whose enclosed volume equals `π(Rc+d)²·H` to the deflection bound, strictly larger than the original for `d>0` and strictly smaller (still positive) for `d<0`

#### Scenario: Out-of-envelope offset faces are honestly declined (host)

- GIVEN a native body and one of: a picked PLANAR cap face, a shrink with `Rc + d ≤ 0`, a zero offset, a cone / sphere / stepped-shaft / multi-radius body, or a non-cylinder solid, with the native engine active and no OCCT
- WHEN `cc_offset_face` (curved arm) is invoked
- THEN the curved builder SHALL return a NULL result (the planar cap is served by the planar arm; everything else falls through to OCCT `BRepOffsetAPI`) AND SHALL NEVER emit an unverified or wrong-shaped solid, and a native void SHALL NEVER be handed to OCCT

#### Scenario: Native curved offset matches the OCCT ground-truth oracle (sim gate)

- GIVEN a capped cylinder built through the `cc_*` facade and the NativeEngine active on a booted iOS simulator (OCCT linked as the oracle only)
- WHEN `cc_offset_face(B, wallFace, d)` is invoked for a grow (`d>0`) and a shrink (`d<0`), `Rc+d>0`, over ≥2 radii / heights
- THEN the native result SHALL match the OCCT ground-truth oracle — the coaxial capped cylinder at radius `Rc+d` (`BRepPrimAPI_MakeCylinder(Rc+d, H)`, measured with `BRepGProp`) — on volume (rel ≤ 1e-2 vs OCCT and vs the exact `π(Rc+d)²·H`), area (rel ≤ 3e-2, facet band), watertightness, Euler χ = 2, per-axis bbox (≤ 2·deflection), and the grow/shrink direction
- AND the shipped OCCT `cc_offset_face` (planar-only) SHALL honestly DECLINE the same curved wall through the facade, and an out-of-slice cone-frustum wall SHALL return native NULL (OCCT owns it)

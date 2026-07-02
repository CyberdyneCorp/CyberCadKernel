# native-construction

This change (Phase 4 #4b Tier A) extends the living `native-construction` capability:
it moves the holed / typed-profile extrudes and the typed-profile (line/arc/circle)
revolve NATIVE in `NativeEngine`, and narrows the "deferred ops fall through" list to
only the sub-cases genuinely still hard (kind-3 SPLINE profile edges, torus-revolve,
any spline-revolve). No `cc_*` ABI change; default engine stays OCCT.

## ADDED Requirements

### Requirement: Native holed prism extrude (circular + polygon holes)

The native construction library SHALL build, from a closed outer polygon profile plus
one or more **holes**, a **holed prism solid** extruded along `+Z` by `depth` with real
**inner-wire (hole) topology**: the bottom and top cap `Plane` faces SHALL each carry
the outer wire PLUS one **reversed inner wire per hole**, and each hole SHALL be swept
along `+Z` into its own inner side sub-shell, so the result is one closed watertight
`Solid` whose material is the annulus between the outer boundary and the holes. A
**circular hole** (given as `cx,cy,r`) SHALL have its bottom and top hole edges built as
**TRUE `Circle` `EdgeCurve`s** (`EdgeCurve::Kind::Circle` — a whole circle is ONE
selectable edge, not sampled line segments) and SHALL sweep to a native `Cylinder` inner
side face. A **polygon hole** (given as a point list) SHALL sweep to N `Plane` inner side
faces on N `Line` edges. Each hole wire SHALL be wound opposite to the outer wire. A hole
lying outside or overlapping the outer polygon, overlapping another hole, or with a
non-positive radius/area SHALL be rejected (no shape produced — the `NativeEngine` then
falls through to the fallback). This builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A rectangle plate with one round hole extrudes to a watertight annular prism with a TRUE circle edge (host)
- GIVEN a `w × h` rectangle outer profile, one circular hole `(cx,cy,r)` inside it, and `depth = d`, built on the host with no OCCT
- WHEN it is extruded and the resulting solid is tessellated by `src/native/tessellate`
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the hole edges SHALL be `Circle` `EdgeCurve`s (not sampled line segments) AND the inner side face SHALL carry a `Cylinder` surface AND the enclosed volume SHALL equal `(w·h − π·r²)·d` within fp64/deflection tolerance

#### Scenario: A rectangle plate with one square (polygon) hole extrudes to a watertight annular prism (host)
- GIVEN a `w × h` rectangle outer profile, one square polygon hole inside it, and `depth = d`, built on the host with no OCCT
- WHEN it is extruded and tessellated
- THEN the mesh SHALL be watertight AND the bottom and top caps SHALL each carry a reversed inner wire of `Line` edges AND the inner side faces SHALL be `Plane` faces AND the enclosed volume SHALL equal `(A_outer − A_hole)·d` within tolerance

#### Scenario: A degenerate or out-of-bounds hole is rejected (host)
- GIVEN a hole with radius/area ≤ 0, OR a hole lying outside the outer polygon, OR two overlapping holes, built on the host
- WHEN the holed extrude is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback engine for that call

### Requirement: Native typed-profile prism extrude (line/arc/full-circle edges + holes)

The native construction library SHALL build a prism from a **typed outer profile** — a
`CCProfileSeg` list mirrored into the engine as `ProfileSeg` — where each outer edge is a
real native-math curved edge: **kind 0 line** `(x0,y0)->(x1,y1)` → a `Line` `EdgeCurve`
that sweeps to a `Plane` side face; **kind 1 arc** (`cx,cy,r` over `[a0,a1]`) → a single
**`Circle` `EdgeCurve`** over that angular span that sweeps to a **`Cylinder`** side
face; **kind 2 full circle** (`cx,cy,r`) → a single closed **`Circle` `EdgeCurve`**
forming a one-edge outer wire that sweeps to a full `Cylinder` lateral face with disc
`Plane` caps. The typed outer segments SHALL chain end-to-end into ONE closed outer wire
(within endpoint tolerance) else the build SHALL be rejected. Circular and polygon holes
MAY be added exactly as for the holed prism. The result SHALL be a watertight native
`Solid` and its curved edges SHALL be TRUE `Circle` edges (a whole arc/circle is ONE
selectable edge). A profile containing a **kind 3 SPLINE** segment SHALL NOT be built
natively — the builder SHALL return a NULL `Shape` so `NativeEngine` falls through to the
fallback (OCCT), and this SHALL NOT be faked or approximated. This builder SHALL remain
OCCT-free and host-buildable.

#### Scenario: An arc-profile prism has a TRUE circle edge and a cylindrical side face (host)
- GIVEN a closed typed outer profile mixing kind-0 line and kind-1 arc segments (e.g. a rounded rectangle / D-shape) and `depth = d`, built on the host with no OCCT
- WHEN it is extruded and tessellated
- THEN the mesh SHALL be watertight AND the arc edge SHALL be a `Circle` `EdgeCurve` (one edge, not sampled) AND the arc SHALL sweep to a `Cylinder` side face AND the enclosed volume SHALL equal `profileArea·d` within tolerance

#### Scenario: A full-circle typed profile extrudes to a native cylinder solid (host)
- GIVEN a single kind-2 full-circle outer profile `(cx,cy,r)` and `depth = d`, built on the host with no OCCT
- WHEN it is extruded and tessellated
- THEN the mesh SHALL be watertight AND the lateral face SHALL carry a `Cylinder` surface AND each cap SHALL carry one `Circle` edge AND the enclosed volume SHALL equal `π·r²·d` and the surface area SHALL equal `2πr·d + 2πr²` within tolerance

#### Scenario: A kind-3 SPLINE profile edge falls through (host)
- GIVEN a typed outer profile containing at least one kind-3 SPLINE segment, built on the host
- WHEN the typed-profile prism builder is invoked
- THEN it SHALL return a NULL `Shape` (no faked/approximate spline edge) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: Native typed-profile revolve (line/arc/circle → plane/cyl/cone/sphere)

The native construction library SHALL build a **solid of revolution** from a typed
`ProfileSeg` profile revolved by an angle about an **arbitrary in-plane axis**
`(ax,ay,adx,ady)`. Per segment it SHALL build the exact analytic face of revolution:
a **kind 0 line** SHALL produce — per its relation to the axis — a `Cylinder` (parallel),
`Plane` (perpendicular), or `Cone` (oblique) face, or no face (on-axis); a **kind 1 arc**
or **kind 2 circle whose supporting-circle centre lies ON the axis** SHALL produce a
**`Sphere`** face of revolution (degenerating to `Plane`/`Cone` at the limits). Circular
edges of revolution SHALL be TRUE `Circle` `EdgeCurve`s about the axis. A **full 360°**
revolution SHALL close the shell into a watertight `Solid`; a **partial** angle
`0 < θ < 2π` SHALL add two `Plane` meridian cap faces so the solid stays watertight.
A **kind 1 arc whose supporting-circle centre is OFF the axis** sweeps a **torus**, for
which there is no native analytic surface type — the builder SHALL return a NULL `Shape`
(fall through, not faked). A **kind 3 SPLINE** segment, and any B-spline surface of
revolution, SHALL likewise return a NULL `Shape` (fall through, not faked). This builder
SHALL remain OCCT-free and host-buildable.

#### Scenario: A line profile about an arbitrary in-plane axis revolves 360° to the analytic solid (host)
- GIVEN a line-segment typed profile and an arbitrary in-plane axis `(ax,ay,adx,ady)`, revolved a full 360°, built on the host with no OCCT
- WHEN it is revolved and tessellated
- THEN the solid SHALL be watertight AND SHALL carry `Cylinder` / `Plane` / `Cone` faces per segment AND its enclosed volume SHALL equal the analytic value within tolerance

#### Scenario: An on-axis semicircle arc revolves 360° to a native sphere solid (host)
- GIVEN a semicircle arc profile whose supporting-circle centre lies ON the axis, of radius `r`, revolved a full 360°, built on the host with no OCCT
- WHEN it is revolved and tessellated
- THEN the solid SHALL be watertight AND the lateral face SHALL carry a `Sphere` surface AND the meridian edge SHALL be a `Circle` AND the enclosed volume SHALL equal `4/3·π·r³` within tolerance

#### Scenario: A partial-angle revolve adds two planar meridian caps and stays watertight (host)
- GIVEN a line-or-arc typed profile revolved by `θ = π/2` about the axis, built on the host with no OCCT
- WHEN it is revolved and tessellated
- THEN the solid SHALL include exactly two additional `Plane` meridian cap faces AND the mesh SHALL be watertight AND its enclosed volume SHALL equal the full-360° volume scaled by `θ / 2π` within tolerance

#### Scenario: An off-axis arc (torus) or a spline segment falls through (host)
- GIVEN a typed revolve profile containing a kind-1 arc whose centre is OFF the axis (a torus), OR a kind-3 SPLINE segment, built on the host
- WHEN the typed-profile revolve builder is invoked
- THEN it SHALL return a NULL `Shape` (no faked torus / spline surface) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: NativeEngine builds the Tier-A profile ops natively, else falls through

`NativeEngine` SHALL override `solid_extrude_holes`, `solid_extrude_polyholes`,
`solid_extrude_profile`, `solid_extrude_profile_polyholes`, and `solid_revolve_profile`
to call the native builders in `src/native/construct/` and type-erase the resulting
native `topology::Shape` into a tracked native `EngineShape`. When a native builder
returns a NULL `Shape` — a degenerate input, or a deferred sub-case (kind-3 SPLINE
profile edge, off-axis arc/torus revolve, or any spline-revolve) — the override SHALL
fall through to the held fallback engine with **no native interception**, producing
exactly the fallback's result. A native holed / typed-profile / revolve body SHALL be
read back by the existing native body-consuming paths (tessellate / face_meshes /
mass_properties / bounding_box / subshape_ids). OCCT SHALL be referenced ONLY under
`CYBERCAD_HAS_OCCT` (in the fallback wiring); the native builders SHALL reference no
OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature, POD layout, or `CCProfileSeg`
field SHALL change.

#### Scenario: A Tier-A profile op is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a supported input (a circular-holed plate, a polygon-holed plate, a line/arc/circle typed profile, or a line/on-axis-arc typed revolve)
- WHEN the corresponding `cc_*` op is invoked
- THEN the shape SHALL be built by `src/native/construct/` with no call to the fallback AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: A deferred profile sub-case falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred sub-case (kind-3 SPLINE profile edge, off-axis arc/torus revolve, or a spline-revolve)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Tier-A profile ops parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME
`cc_solid_extrude_holes`, `cc_solid_extrude_polyholes`, `cc_solid_extrude_profile`,
`cc_solid_extrude_profile_polyholes`, and `cc_solid_revolve_profile` calls SHALL be
issued once with the native engine active (`cc_set_engine(1)`) and once with the OCCT
default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the
`cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight
tessellation — within a documented tight fp64/deflection tolerance. The deferred
sub-cases (kind-3 SPLINE profile edge, off-axis arc/torus revolve, spline-revolve) SHALL
be asserted identical under both engines (fall-through proof). The parity test SHALL
restore the OCCT default in teardown.

#### Scenario: Native Tier-A profile ops match the OCCT oracle (parity)
- GIVEN supported holed / typed-profile / typed-revolve inputs on a booted iOS simulator
- WHEN each of the five `cc_*` ops is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented tolerance

#### Scenario: Deferred sub-cases are identical under both engines (parity)
- GIVEN a kind-3 SPLINE profile edge, an off-axis arc/torus revolve, and a spline-revolve on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL NOT be implemented natively and SHALL fall
through the `NativeEngine` to the fallback (OCCT) engine: `solid_loft`,
`solid_loft_wires`, `solid_sweep`, `twisted_sweep`, `loft_along_rail`, `guided_sweep`,
`wrap_emboss`, `helical_thread`, `tapered_thread`, `tapered_shank`. In addition, these
**sub-cases** of the otherwise-native Tier-A profile ops SHALL fall through (the native
builder returns a NULL `Shape` and the engine delegates): a **kind 3 SPLINE** profile
edge in `solid_extrude_profile` / `solid_extrude_profile_polyholes`; a **kind 1 arc
whose supporting-circle centre is OFF the axis** (a torus surface of revolution) in
`solid_revolve_profile`; and any **spline-revolve** (a kind 3 segment in
`solid_revolve_profile`, or a B-spline surface of revolution). Likewise every feature /
boolean / tessellate-of-a-foreign-body / query / transform / exchange op SHALL fall
through. The change SHALL NOT fake, stub-out, or partially implement any deferred op or
sub-case; each SHALL produce exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_solid_loft`, `cc_helical_thread`, `cc_solid_sweep`, or `cc_twisted_sweep`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred sub-case (a kind-3 SPLINE profile edge, an off-axis arc/torus revolve, or a spline-revolve)
- WHEN the corresponding `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` / `cc_solid_revolve_profile` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

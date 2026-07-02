# native-construction Specification

## Purpose
TBD - created by archiving change add-native-construction. Update Purpose after archive.
## Requirements
### Requirement: OCCT-free, host-buildable native construction library

The native construction library SHALL live under `src/native/construct/` and
SHALL include NO OCCT header in any of its translation units, so that it compiles
and unit-tests with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT
and NO simulator. It MAY include `src/native/math`, `src/native/topology`, and
`src/native/tessellate`. It SHALL expose plain functions returning a native
`topology::Shape` and SHALL NOT reference `IEngine`, `EngineShape`, or any OCCT
type. OCCT SHALL appear ONLY behind `CYBERCAD_HAS_OCCT` in the engine glue and in
the simulator parity test — never in this library.

#### Scenario: Library builds on the host without OCCT
- GIVEN the sources under `src/native/construct/`
- WHEN they are compiled with `clang++ -std=c++20` with no OCCT and no simulator (the `src/native/math`, `src/native/topology`, `src/native/tessellate` headers available)
- THEN the build SHALL succeed AND no compiled translation unit SHALL include any OCCT header

### Requirement: Native prism extrude of a closed polygon profile

The library SHALL build, from a closed 2D polygon profile (`profileXY` as x,y
pairs) and a `depth`, a **prism solid** extruded along `+Z`: exactly one bottom
planar face at `z = 0`, one top planar face at `z = depth`, and one **planar quad
side face per profile edge**. The result SHALL be a native `topology::Shape` of
type `Solid` containing one closed `Shell` whose faces are oriented outward, with
every edge shared by exactly two faces (manifold) and every vertex shared by its
incident edges. Bottom/top faces SHALL carry a `Plane` surface; side faces SHALL
carry a `Plane` surface; side edges SHALL be `Line` curves. A profile with fewer
than 3 distinct points, a zero/degenerate `depth`, or a self-intersecting
polygon SHALL be rejected (no shape produced) rather than yielding an invalid
solid. Profile winding SHALL be normalised so the bottom-face normal points `−Z`
and the top-face normal points `+Z`.

#### Scenario: A rectangle extrudes to a watertight box with exact volume (host)
- GIVEN an axis-aligned `w × h` rectangle profile and `depth = d` built on the host with no OCCT
- WHEN it is extruded and the resulting solid is tessellated by `src/native/tessellate`
- THEN the solid SHALL have exactly 6 faces, 12 edges, 8 vertices AND the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume SHALL equal `w·h·d` and the surface area SHALL equal `2(w·h + w·d + h·d)` within fp64 tolerance

#### Scenario: A triangle extrudes to a watertight prism with N+2 faces (host)
- GIVEN a triangular profile (3 points) and `depth = d` built on the host with no OCCT
- WHEN it is extruded
- THEN the solid SHALL have exactly 5 faces (2 caps + 3 sides), the mesh SHALL be watertight AND the enclosed volume SHALL equal `triangle_area·d` within fp64 tolerance

#### Scenario: A degenerate profile or depth is rejected (host)
- GIVEN a profile with fewer than 3 distinct points, OR `depth == 0`, OR a self-intersecting polygon, built on the host
- WHEN extrude is invoked
- THEN NO solid SHALL be produced AND the failure SHALL be reported (the facade yields shape id 0)

### Requirement: Native solid of revolution for line-segment profiles

The library SHALL build, from a profile composed of **straight line segments**
and a revolution angle, a **solid of revolution** about the axis. Each segment
SHALL become the exact analytic face of revolution determined by its relation to
the axis: a segment **parallel** to the axis SHALL produce a `Cylinder` face; a
segment **perpendicular** to the axis SHALL produce a `Plane` (disc/annulus)
face; an **oblique** segment SHALL produce a `Cone` face; a segment lying **on**
the axis SHALL contribute no face. Circular edges of revolution SHALL be `Circle`
curves about the axis. A **full 360°** revolution SHALL close the shell on itself
(no cap faces) into a watertight `Solid`; a **partial** angle `0 < θ < 2π` SHALL
add the two **planar cap faces** (start and end meridian planes) so the solid
remains closed and watertight. The result SHALL be a native `topology::Shape` of
type `Solid`. Profiles containing arc or spline segments are OUT OF SCOPE for
this requirement (see the deferred-scope requirement).

#### Scenario: A rectangle offset from the axis revolves 360° to a watertight cylindrical shell (host)
- GIVEN a rectangular line-segment profile at radius range `[r0, r1]` and height `h`, revolved a full 360° about the axis, built on the host with no OCCT
- WHEN it is revolved and tessellated
- THEN the solid SHALL be watertight (`boundaryEdgeCount == 0`), SHALL contain `Cylinder` and `Plane` faces AND its enclosed volume SHALL equal `π·h·(r1² − r0²)` within tolerance

#### Scenario: An oblique segment revolves to a conical face (host)
- GIVEN a profile whose segment is oblique to the axis, revolved 360°, built on the host with no OCCT
- WHEN it is revolved
- THEN the swept face for that segment SHALL be a `Cone` surface AND the solid SHALL be watertight AND its enclosed volume SHALL equal the analytic frustum/cone volume within tolerance

#### Scenario: A partial-angle revolution adds planar cap faces and stays watertight (host)
- GIVEN a line-segment profile revolved by `θ = π/2` (90°) about the axis, built on the host with no OCCT
- WHEN it is revolved and tessellated
- THEN the solid SHALL include exactly two additional `Plane` cap faces (start and end meridian) AND the mesh SHALL be watertight AND its enclosed volume SHALL equal the full-360° volume scaled by `θ / 2π` within tolerance

### Requirement: NativeEngine implements IEngine and falls through to OCCT

A `NativeEngine : IEngine` SHALL live under `src/engine/native/`. It SHALL
override `solid_extrude` (polygon extrude) and `solid_revolve` (line-segment
revolve) to build the shape natively via `src/native/construct/`, type-erasing
the native `topology::Shape` into an `EngineShape`. For **every other** `IEngine`
method it SHALL delegate to a held fallback `std::shared_ptr<IEngine>`. It SHALL
reference the OCCT engine ONLY inside `CYBERCAD_HAS_OCCT`: in an OCCT build the
fallback is the OCCT engine; in the host build the fallback is the stub. `name()`
SHALL return `"native"`. Feeding a native-built shape into an OCCT-only op
(feature/boolean/etc.) is NOT supported in this change; such paths remain the
fallback's responsibility and native shapes are exercised via native tessellation
and the parity harness.

#### Scenario: Native engine builds extrude/revolve natively and delegates the rest
- GIVEN a `NativeEngine` wrapping a fallback engine
- WHEN `solid_extrude` (polygon) or `solid_revolve` (line segments) is called
- THEN the shape SHALL be built by `src/native/construct/` with no call to the fallback
- AND WHEN any other `IEngine` method is called (e.g. `boolean_op`, `fillet_edges`, `tessellate`, `mass_properties`, `solid_loft`)
- THEN the call SHALL be forwarded to the fallback engine unchanged

#### Scenario: NativeEngine references OCCT only under the build guard
- GIVEN the sources under `src/engine/native/`
- WHEN they are compiled with no OCCT (host build)
- THEN they SHALL compile with the fallback bound to the stub AND every OCCT reference SHALL be inside `#ifdef CYBERCAD_HAS_OCCT`

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

### Requirement: Additive cc_set_engine / cc_active_engine facade toggle, default OCCT

The public C ABI SHALL gain two ADDITIVE functions —
`void cc_set_engine(int native)` and `int cc_active_engine(void)` — declared in
`include/cybercadkernel/cc_kernel.h` and defined in `src/facade/cc_kernel.cpp`,
modelled on `cc_set_parallel` / `cc_parallel_enabled`. `cc_set_engine(1)` SHALL
install the `NativeEngine` as the active engine (via `set_active_engine`);
`cc_set_engine(0)` SHALL restore the default engine. `cc_active_engine()` SHALL
return `1` iff the native engine is active, else `0`. The DEFAULT active engine
at process start SHALL remain OCCT (the default engine) so that every existing
suite is unchanged unless a caller explicitly opts in. On a build with no native
engine wired (host stub), `cc_set_engine` SHALL be a safe no-op and
`cc_active_engine()` SHALL report `0`. No existing `cc_*` signature or POD struct
layout SHALL change.

#### Scenario: Default engine is OCCT; existing suites unchanged
- GIVEN a fresh process on an OCCT build with no call to `cc_set_engine`
- WHEN any `cc_*` op is invoked
- THEN it SHALL be answered by the OCCT engine exactly as before this change AND `cc_active_engine()` SHALL return `0`

#### Scenario: Toggling native swaps the active engine and back
- GIVEN an OCCT build
- WHEN `cc_set_engine(1)` is called
- THEN `cc_active_engine()` SHALL return `1` AND subsequent `cc_solid_extrude` / `cc_solid_revolve` calls SHALL be built natively
- AND WHEN `cc_set_engine(0)` is called
- THEN `cc_active_engine()` SHALL return `0` AND the active engine SHALL be the OCCT default again

#### Scenario: No ABI break
- GIVEN this change applied
- WHEN the public header and POD structs are inspected
- THEN only the two new additive functions SHALL have been added AND no existing `cc_*` signature or struct layout SHALL have changed

### Requirement: Native-vs-OCCT parity through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME
`cc_solid_extrude` and `cc_solid_revolve` (line-segment) calls SHALL be issued once with the native
engine active (`cc_set_engine(1)`) and once with the OCCT default
(`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the
`cc_*` facade — mass properties (volume / area / centroid), axis-aligned bounding
box, sub-shape counts, and watertight tessellation — within a documented tight
fp64 tolerance. The parity test SHALL restore the OCCT default in teardown so the
process-wide toggle does not perturb other suites. This is the second of the two
verification gates required by `NATIVE-REWRITE.md`.

#### Scenario: Native extrude matches the OCCT extrude oracle (parity)
- GIVEN a closed polygon profile and depth on a booted iOS simulator
- WHEN `cc_solid_extrude` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented tolerance

#### Scenario: Native line-segment revolve matches the OCCT revolve oracle (parity)
- GIVEN a line-segment profile and a revolution angle (both a full 360° and a partial angle) on a booted iOS simulator
- WHEN `cc_solid_revolve` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented tolerance

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

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


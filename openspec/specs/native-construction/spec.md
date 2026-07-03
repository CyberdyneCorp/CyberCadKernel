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

The following construction operations SHALL fall through the `NativeEngine` to the fallback
(OCCT) engine: `wrap_emboss`. (`solid_sweep` — for a straight / smooth-planar / non-planar
(RMF) spine — `solid_loft` / `solid_loft_wires` — for a 2- or N-section equal-count planar
chain — `tapered_shank`, and the well-formed `helical_thread` / `tapered_thread` are NOW
native where their result self-verifies as a watertight, oracle-correct solid; see the native
residual requirements. `twisted_sweep` with a REAL twist/scale, `guided_sweep`, and
`loft_along_rail` were ATTEMPTED but did NOT self-verify oracle-correct and therefore fall
through per the sub-cases below.) These construction ops are **attempted natively** and are native ONLY
where the result is verified a valid watertight solid with the correct volume/geometry on
BOTH gates; OTHERWISE they SHALL fall through to OCCT (labelled, verified, never faked). In
addition, these **sub-cases** SHALL fall through (the native builder returns a NULL `Shape`,
or the `NativeEngine` self-verify DISCARDS the candidate, and the engine delegates):

- **Surface–surface intersection (SSI) — Tier 4, NOT attempted here.** A **self-intersecting
  sweep** (the swept surface folds through itself), a **tight-curvature spine** whose section
  folds (curvature radius below the section's radial extent × the applied scale), a **hard
  pipe-shell rail** that cannot close without trimming two swept surfaces, and a **genuinely
  self-intersecting thread** (flanks truly cross even after root clamping — the crest of one
  turn passes the next) SHALL fall through. This change does NOT attempt SSI.
- **Sweeps:** a `twisted_sweep` beyond the twist/scale envelope (accumulated twist × section
  radius exceeding the station spacing so adjacent rings interpenetrate) and a self-crossing
  spine SHALL fall through.
- **Loft:** a chain with **mismatched vertex counts** across any pair (ambiguous
  resample — OCCT `ThruSections` re-parametrizes) and a **non-planar end section** SHALL fall
  through.
- **Profile ops:** a **spline-revolve** (a kind-3 segment in `solid_revolve_profile`, or a
  general B-spline surface of revolution — NOT attempted) and a **non-planar / self-
  intersecting spline** loop in `solid_extrude_profile` /
  `solid_extrude_profile_polyholes` SHALL fall through. (A kind-3 spline edge in a PLANAR
  extrude and a kind-1 off-axis-arc → TORUS revolve are NOW native.)
- **Threads:** a thread requiring the **round-profile fallback** (the native builder returns
  NULL and OCCT applies that fallback — the native path does not fake it) SHALL fall through;
  the tapered tip-crossing and degenerate guards are unchanged.

Likewise every feature / boolean / tessellate-of-a-foreign-body / query / transform /
exchange op SHALL fall through. The change SHALL NOT fake, stub-out, or partially implement
any deferred op or sub-case, and SHALL NOT attempt SSI; each deferred call SHALL produce
exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_wrap_emboss`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: An SSI sub-case falls through (not attempted here)
- GIVEN the native engine active and an input whose valid B-rep would require surface–surface intersection (a self-intersecting sweep, a tight-curvature fold, a hard pipe-shell rail, or a truly self-intersecting thread)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the self-verify DISCARDS the candidate) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`), with no SSI attempted

#### Scenario: A deferred loft sub-case falls through
- GIVEN the native engine active and a loft chain with mismatched vertex counts OR a non-planar end section
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred profile sub-case (a spline-revolve, or a non-planar / self-intersecting spline extrude loop)
- WHEN the corresponding `cc_solid_revolve_profile` / `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred thread sub-case falls through
- GIVEN the native engine active and a thread requiring the round-profile fallback OR a truly self-intersecting thread (flanks cross even after root clamping)
- WHEN `cc_helical_thread` / `cc_tapered_thread` is invoked
- THEN the native builder SHALL return NULL (no faked round profile, no SSI) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

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

### Requirement: Native two-section ruled loft (equal point counts)

The native construction library SHALL build, from **exactly two** section wires with
the **same** vertex count `n ≥ 3`, a **ruled loft solid**: corresponding vertices
`a[i] ↔ b[i]` and corresponding edges `a[i]→a[i+1]` ↔ `b[i]→b[i+1]` SHALL be paired,
and for each of the `n` corresponding edge pairs the library SHALL build **one ruled
side face** — a **degree-1 skin** surface `S(u,v) = (1−v)·A(u) + v·B(u)` expressed as
a native-math `Bezier`/`BSpline` `FaceSurface`, degenerating to an exact `Plane`
`FaceSurface` when the four corners are coplanar within tolerance. The `n` ruled side
faces SHALL be capped by the **bottom** section face (section A) and the **top**
section face (section B), each a `Plane` face for a planar section, and assembled into
ONE closed `Shell` whose faces are oriented outward → a native `topology::Shape` of
type `Solid` that is manifold and watertight (every edge shared by exactly two faces).
The `n` connecting edges `a[i]→b[i]` SHALL be `Line` curves, each shared by its two
adjacent side faces. This construction SHALL match OCCT
`BRepOffsetAPI_ThruSections(isSolid=true, ruled=true)` over the two sections within a
deflection bound. This builder SHALL remain OCCT-free and host-buildable
(`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator) and SHALL
reference no `IEngine` / `EngineShape` / OCCT type.

The following loft configurations SHALL NOT be built natively — the builder SHALL
return a NULL `Shape` (so `NativeEngine` falls through to the fallback engine), and
this SHALL NOT be faked, stubbed, or partially built: **mismatched vertex counts**
(`nA ≠ nB`); a section with **fewer than 3 distinct points**; a section **degenerating
to a point** (all points coincident — a punctual section); a **non-planar** section
whose cap cannot be built as a single planar face; a **self-intersecting** section or
a corresponding-vertex **ruling that self-intersects** or otherwise fails to close
into a valid watertight solid; and any loft over **3 or more** sections (Tier C).

#### Scenario: An identical square at two z-levels lofts to a watertight box (host)
- GIVEN a square section A at `z = 0` and the identical square section B at `z = d`, equal point count, built on the host with no OCCT
- WHEN it is lofted and the resulting solid is tessellated by `src/native/tessellate`
- THEN the solid SHALL have exactly 6 faces (4 ruled sides + 2 caps), 12 edges, 8 vertices AND every ruled side face SHALL carry a `Plane` surface (the coplanar limit) AND the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume SHALL equal `area·d` within fp64 tolerance

#### Scenario: A square-to-smaller-square loft is a watertight prismatoid with ruled sides (host)
- GIVEN a square section A and a concentric smaller square section B in a parallel plane a gap `d` away, equal point count `n = 4`, built on the host with no OCCT
- WHEN it is lofted and tessellated
- THEN the solid SHALL have exactly 4 ruled side faces (each a degree-1 skin `Bezier`/`BSpline`, or `Plane` when its four corners are coplanar) + 2 `Plane` caps AND the mesh SHALL be watertight AND the enclosed volume SHALL equal the prismatoid value `d/6·(A_bottom + 4·A_mid + A_top)` within tolerance

#### Scenario: Two 3D wires in parallel planes loft via cc_solid_loft_wires (host)
- GIVEN two planar square wires given as 3D point lists in parallel planes, equal point count, built on the host with no OCCT
- WHEN `build_loft_wires` builds the solid and it is tessellated
- THEN the solid SHALL be watertight AND SHALL have `n` ruled side faces + 2 `Plane` caps AND its enclosed volume SHALL equal the analytic value within tolerance

#### Scenario: A mismatched, punctual, non-planar, or non-closeable loft is rejected (host)
- GIVEN two sections with unequal point counts, OR a section that is punctual (all points coincident), OR (for `build_loft_wires`) a non-planar section, OR a section with fewer than 3 distinct points / a self-intersecting section or ruling, built on the host
- WHEN the native loft builder is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback engine for that call

### Requirement: NativeEngine builds the two-section ruled loft natively, else falls through

`NativeEngine` SHALL override `solid_loft` (bottom XY at `z = 0` + top XY at
`z = depth`, equal point count) and `solid_loft_wires` (two arbitrary 3D wires, equal
point count) to call the native ruled-loft builder in `src/native/construct/` and
type-erase the resulting native `topology::Shape` into a tracked native `EngineShape`.
When the native builder returns a NULL `Shape` — a mismatched count, punctual or
non-planar section, a non-closeable ruling, or any 3+-section / guided / rail loft
that never reaches this builder — the override SHALL fall through to the held fallback
engine with **no native interception**, producing exactly the fallback's result. A
native loft body SHALL be read back by the existing native body-consuming paths
(tessellate / face_meshes / mass_properties / bounding_box / subshape_ids). OCCT SHALL
be referenced ONLY under `CYBERCAD_HAS_OCCT` (in the fallback wiring); the native
builder SHALL reference no OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature
or POD layout SHALL change; `native_engine.h` SHALL be unchanged (both signatures
already exist).

#### Scenario: A supported two-section loft is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a supported input (two equal-point-count planar sections)
- WHEN `cc_solid_loft` or `cc_solid_loft_wires` is invoked
- THEN the shape SHALL be built by `src/native/construct/` with no call to the fallback AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: A deferred loft configuration falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred configuration (mismatched point counts, a punctual section, a non-planar `cc_solid_loft_wires` section, or a 3+-section loft)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Two-section ruled loft parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME
`cc_solid_loft` and `cc_solid_loft_wires` calls SHALL be issued once with the native
engine active (`cc_set_engine(1)`) and once with the OCCT default (`cc_set_engine(0)`),
and the two resulting shapes SHALL be compared through the `cc_*` facade — mass
properties (volume / area / centroid), axis-aligned bounding box, sub-shape counts,
and watertight tessellation — against `BRepOffsetAPI_ThruSections(isSolid=true,
ruled=true)` within a documented tight fp64/deflection tolerance. The deferred
configurations (mismatched count, punctual section, non-planar wire) SHALL be asserted
identical under both engines (fall-through proof). The parity test SHALL restore the
OCCT default in teardown so the process-wide toggle does not perturb other suites.
This is the second of the two verification gates required by `NATIVE-REWRITE.md`.

#### Scenario: Native ruled loft matches the OCCT ThruSections oracle (parity)
- GIVEN supported two-section loft inputs (an identical-square box loft, a square→smaller-square frustum-like loft, and two parallel-plane 3D square wires) on a booted iOS simulator
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented tolerance

#### Scenario: Deferred loft configurations are identical under both engines (parity)
- GIVEN a mismatched-count loft, a punctual-section loft, and a non-planar `cc_solid_loft_wires` loft on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

### Requirement: Native straight-path sweep (directional extrude, exact)

The native construction library SHALL build, from a **closed 2D profile** (`profileXY`
as x,y pairs in the profile's local plane) and a **straight 3D path** (`pathXYZ` as
x,y,z triples that are all collinear within tolerance, including the two-point case), a
**swept solid** equal to the profile directionally extruded along the path vector
`d = path[last] − path[0]`. The profile SHALL be placed **perpendicular to the path at
its start** — the profile's local +Z aligned with the unit path tangent and its local
X/Y spanning the plane ⟂ the tangent, with the profile centroid on `path[0]` — and
translated by `d`. The result SHALL be a native `topology::Shape` of type `Solid`: one
**`Plane` side face per profile edge** plus two **`Plane` end caps** (start and far
section), oriented outward, watertight, every edge shared by exactly two faces. Because
the caps are ⟂ the sweep direction, the enclosed volume SHALL equal `profileArea · |d|`
**exactly** (fp64). A profile with fewer than 3 distinct points, a zero-area profile, or
a zero-length path SHALL be rejected (the builder returns a NULL `Shape`). This builder
SHALL remain OCCT-free and host-buildable.

#### Scenario: A square profile swept along a straight path is a watertight prism with exact volume (host)
- GIVEN a `w × w` closed profile and a straight path of length `L` (two collinear points, in any 3D direction), built on the host with no OCCT
- WHEN it is swept and the resulting solid is tessellated by `src/native/tessellate`
- THEN the solid SHALL have exactly `4 + 2` faces (4 side + 2 caps) AND the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume SHALL equal `w²·L` exactly within fp64 tolerance AND the two cap faces SHALL be perpendicular to the path direction

#### Scenario: A degenerate profile or zero-length path is rejected (host)
- GIVEN a profile with fewer than 3 distinct points OR a zero-area profile OR a zero-length path, built on the host
- WHEN the sweep is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback engine for that call

### Requirement: Native smooth-curved planar-path sweep (constant frame, oracle-matched)

The native construction library SHALL build, from a closed 2D profile and a **smooth
curved but PLANAR 3D path** (all spine points coplanar within tolerance) that PASSES the
curvature guard, a watertight swept `Solid` by carrying the profile along the spine with
a **constant frame** — the SAME sweep law OCCT `BRepOffsetAPI_MakePipe` applies on a
planar spine (its default `GeomFill_CorrectedFrenet` collapses to a constant rotation
there). The builder SHALL: (1) form the start frame at the path start exactly as the
oracle does (`x = normalize(cross(t0, ref))`, `y = normalize(cross(x, t0))`, with
`ref = +Y` unless the tangent is near-vertical, and the profile centroid on the spine);
(2) hold that frame's `x`/`y` axes **CONSTANT** at every station — the section is
translated, not rotated to stay perpendicular; (3) build **one bilinear ruled side face
per profile edge per spine segment** with shared per-station vertex rings so patches weld
watertight; (4) cap both ends with the fixed-orientation profile `Plane` face. The result
SHALL be one closed watertight `Solid` oriented outward. Because the section normal
`n = x×y` is fixed, the enclosed volume SHALL equal `profileArea · |Δspine · n̂|` (the
spine displacement projected onto the fixed normal) — which is what the oracle reports on
a planar spine (within the shared polyline discretization), NOT the Pappus arc-length
volume. A **non-planar** curved spine SHALL be rejected (NULL `Shape`) because OCCT then
uses its genuine non-constant law which the constant frame does not reproduce. This
builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A profile swept along a gentle planar arc is watertight and oracle-matched (host)
- GIVEN a closed profile of area `A` swept along a smooth PLANAR circular-arc spine whose curvature radius everywhere exceeds the profile's radial extent, built on the host with no OCCT
- WHEN it is swept and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the solid SHALL have `edges·segments + 2` faces AND the enclosed volume SHALL equal `A · |Δspine · n̂|` (the constant-frame swept volume) within the mesh's linear tolerance

#### Scenario: The transported frame is constant over a planar arc (host)
- GIVEN a profile swept along a planar circular arc, built on the host with no OCCT
- WHEN the frame is transported along the discretized spine
- THEN every station's section axes SHALL equal the start frame's axes (the section is translated, not rotated), reproducing the oracle's planar constant-rotation law

#### Scenario: A non-planar curved spine falls through (host)
- GIVEN a profile swept along a non-planar (fully 3D) curved spine, built on the host with no OCCT
- WHEN the sweep is invoked
- THEN the builder SHALL return a NULL `Shape` (OCCT's genuine corrected-Frenet law is required) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: Native twisted sweep reduces to the plain sweep (no-op only)

The native construction library SHALL serve `cc_twisted_sweep` (`twistRadians`,
`scaleEnd`) NATIVELY **only when it reduces to the plain sweep** — `twistRadians ≈ 0`
AND `scaleEnd ≈ 1` — by forwarding to the native `build_sweep` (itself native for a
straight or smooth-curved planar spine). A **genuine** twist or scale accumulates an
extra per-section rotation/scale the constant-frame sweep does not model, so
`build_twisted_sweep` SHALL return a NULL `Shape` and `NativeEngine` SHALL fall through
to the fallback (OCCT `ThruSections`) — there SHALL be no faked twist path. This builder
SHALL remain OCCT-free and host-buildable.

#### Scenario: A zero-twist unit-scale sweep is the plain native prism (host)
- GIVEN a square profile, a straight path of length `L`, `twistRadians = 0`, `scaleEnd = 1`, built on the host with no OCCT
- WHEN the twisted sweep is invoked and tessellated
- THEN it SHALL forward to the native sweep, the mesh SHALL be watertight AND the volume SHALL equal the plain prism value `profileArea · L` exactly

#### Scenario: A real twist or scale falls through (host)
- GIVEN a twisted sweep with a non-zero `twistRadians` or a `scaleEnd ≠ 1`, built on the host
- WHEN `build_twisted_sweep` is invoked
- THEN it SHALL return a NULL `Shape` AND `NativeEngine` SHALL fall through to the fallback for that call (no separate faked twist path)

### Requirement: Sweep tight-curvature and self-intersection guard (honest fall-through)

The native sweep builder SHALL detect and REJECT (return a NULL `Shape`, never a wrong
solid) any sweep whose swept surface would **self-intersect**: specifically when, at any
spine station, the curvature radius `1/κ` is below the profile's **maximum radial
extent** (with a safety margin) — the inner wall of the bend would fold — or when the
spine self-crosses or a transported section self-intersects. On rejection the
`NativeEngine` SHALL fall through to the fallback (OCCT), whose trimmed pipe-shell
handles the self-intersection. This guard SHALL NOT be faked or approximated into an
invalid solid.

#### Scenario: A tight-curvature sweep is rejected and falls through (host)
- GIVEN a profile of radial extent `R` swept along a spine whose minimum curvature radius is less than `R`, built on the host
- WHEN the sweep is invoked
- THEN the builder SHALL return a NULL `Shape` (no self-intersecting solid) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: NativeEngine builds the sweep ops natively, else falls through

`NativeEngine` SHALL override `solid_sweep` and `twisted_sweep` to call the native
builders in `src/native/construct/` and type-erase the resulting native
`topology::Shape` into a tracked native `EngineShape`. When a native builder returns a
NULL `Shape` — a degenerate input, a tight-curvature / self-intersection guard trip, or
a path shape the builder declines — the override SHALL fall through to the held fallback
engine with **no native interception**, producing exactly the fallback's result. A
native swept body SHALL be read back by the existing native body-consuming paths
(tessellate / face_meshes / mass_properties / bounding_box / subshape_ids). OCCT SHALL
be referenced ONLY under `CYBERCAD_HAS_OCCT` (in the fallback wiring); the native
builder SHALL reference no OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature or
POD layout SHALL change.

#### Scenario: A supported sweep is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a supported input (a straight-path sweep, a gentle-curve sweep, or a twisted sweep on such a path)
- WHEN `cc_solid_sweep` / `cc_twisted_sweep` is invoked
- THEN the shape SHALL be built by `src/native/construct/` with no call to the fallback AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported sweep case falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (tight curvature / self-intersecting sweep, or `cc_guided_sweep` / `cc_loft_along_rail`)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the op is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Sweep ops parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME
`cc_solid_sweep` and `cc_twisted_sweep` calls SHALL be issued once with the native
engine active (`cc_set_engine(1)`) and once with the OCCT default (`cc_set_engine(0)`),
and the two resulting shapes SHALL be compared through the `cc_*` facade — mass
properties, bounding box, sub-shape counts, and watertight tessellation — against the
OCCT `BRepOffsetAPI_MakePipe` oracle within a documented tolerance. Because the native
builder uses the SAME constant-frame law OCCT applies on a planar spine, BOTH the
**straight** and the **smooth-planar-curved** sweep match the oracle EXACTLY (volume /
bbox / centroid / face-count relative error ~0, fp precision), not merely within a
deflection band; the tolerance is set generously (curved ≤ 5e-2) but the measured error
is ~1.7e-16. The fall-through cases (a non-planar / tight-curvature / self-intersecting
sweep, a real-twist `cc_twisted_sweep`, `cc_guided_sweep`, and
`cc_loft_along_rail`) SHALL be asserted identical under both engines (fall-through
proof). The parity test SHALL restore the OCCT default in teardown and SHALL carry its
own `main()` (on the `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is
unchanged.

#### Scenario: Native sweep matches the OCCT MakePipe oracle (parity)
- GIVEN a supported straight-path and gentle-curve sweep (and a twisted sweep) on a booted iOS simulator
- WHEN each `cc_*` op is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented tolerance (straight EXACT, curved deflection-bounded), and the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

#### Scenario: Fall-through sweep cases are identical under both engines (parity)
- GIVEN a tight-curvature / self-intersecting sweep, a `cc_guided_sweep`, and a `cc_loft_along_rail` on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

### Requirement: Native tapered-shank revolve (pointed shank, silhouette)

The native construction library SHALL build, from `radiusMM`, `fullHeightMM`,
`taperHeightMM` (and a `pointsPerMM` density hint), a **solid of revolution** equal to
the `cc_tapered_shank` silhouette revolved 360° about the Z axis: a loop in the `(r, h)`
half-plane that holds `r = radiusMM` for `h ∈ [0, fullHeightMM]` then tapers **linearly
to a point on the axis** `(0, fullHeightMM + taperHeightMM)` over `taperHeightMM`. The
builder SHALL assemble this silhouette as line segments (bottom disk radius, full-radius
cylinder wall, taper cone, on-axis return) and revolve it with the existing native
`build_revolution`, whose per-segment classifier yields a planar bottom **disk**, a
**`Cylinder`** wall, and a **`Cone`** taper (the on-axis return contributes no swept
face); a full 2π turn closes the shell → a watertight native `Solid` oriented outward.
The enclosed volume SHALL equal `π·radiusMM²·fullHeightMM + (1/3)·π·radiusMM²·taperHeightMM`
within the tessellation deflection bound, and the bounding box SHALL be
`[−r, r]×[−r, r]×[0, fullHeightMM + taperHeightMM]`. Degenerate input (`radiusMM ≤ 0`, or
`fullHeightMM + taperHeightMM ≤ 0`) SHALL be rejected (the builder returns a NULL `Shape`,
and `NativeEngine` falls through). This builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A tapered shank is a watertight solid of revolution with the analytic volume (host)
- GIVEN `radiusMM = r`, `fullHeightMM = h`, `taperHeightMM = t` (all positive), built on the host with no OCCT
- WHEN the shank is built and tessellated by `src/native/tessellate`
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume SHALL equal `π·r²·h + π·r²·t/3` within the deflection bound AND the bounding box SHALL equal `[−r, r]×[−r, r]×[0, h + t]`

#### Scenario: A degenerate shank is rejected (host)
- GIVEN `radiusMM ≤ 0` OR `fullHeightMM + taperHeightMM ≤ 0`, built on the host
- WHEN the shank builder is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: Native helical thread — radial V-section on an axis-aux-spine helix

The native construction library SHALL, where achievable (both verification gates green;
otherwise this op falls through per the MODIFIED deferred-ops requirement), build from
`majorRadiusMM`, `pitchMM`, `turns`, `depthMM`, `flankAngleDeg`, `pointsPerMM`,
`samplesPerTurn` a watertight thread `Solid`: a **V/triangular section** (apex outward,
base along the axis, flank half-angle `flankAngleDeg`, radial `depthMM`) swept along a
**constant-radius helix** at the pitch-line radius (from `majorRadiusMM`), `turns`
revolutions at axial `pitchMM` per turn. The builder SHALL keep the section **RADIAL** at
every station using an **axis auxiliary-spine law**: at helix angle θ the section's
outward axis SHALL be the radial unit vector `r̂(θ) = (cosθ, sinθ, 0)` (the apex direction)
and its along-axis direction SHALL be `+Z` — the frame is defined by the Z-axis auxiliary
spine, NOT the Frenet frame of the helix, so the V does NOT rotate/tilt with the helix
tangent (mirroring OCCT `BRepOffsetAPI_MakePipeShell::SetMode(AuxiliarySpine, …)`). The
builder SHALL tile one **bilinear ruled band** per (section edge × spine segment) with
SHARED per-station vertex rings and cap both helix ends with the planar V-section face →
a watertight `Solid` oriented outward. `samplesPerTurn` SHALL be **capped** (both
per-turn and in total station count) so the work is bounded; the mesh SHALL remain
deflection-bounded vs the oracle. When native, the solid SHALL match the OCCT
`BRepOffsetAPI_MakePipeShell` (auxiliary-spine mode) oracle in volume / bbox / centroid /
watertightness within a documented deflection tolerance, the native face count being an
integer multiple `k ≥ 1` of the OCCT face count. The builder SHALL reject
self-intersecting / degenerate cases (see the guard requirement) by returning a NULL
`Shape`. This builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A coarse-pitch shallow-depth thread is watertight with a radial section (host)
- GIVEN a thread whose `depthMM` is safely below the pitch room (the self-intersection guard passes), built on the host with no OCCT
- WHEN it is built and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND at every station the section apex direction SHALL equal the station's radial unit vector `r̂(θ)` (the axis-aux-spine radial invariant — the section does NOT Frenet-rotate) AND the enclosed volume SHALL be within the deflection band of the swept-V estimate

#### Scenario: A native helical thread matches the OCCT MakePipeShell oracle (parity)
- GIVEN a coarse-pitch shallow-depth `cc_helical_thread` on a booted iOS simulator, where the native builder produces a solid
- WHEN it is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the documented deflection tolerance AND the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

### Requirement: Native tapered thread — radial V-section on a tapering helix

The native construction library SHALL, where achievable (both gates green; otherwise this
op falls through per the MODIFIED deferred-ops requirement), build a watertight
tapered-thread `Solid` from `topRadiusMM`, `tipRadiusMM`, `pitchMM`, `turns`, `depthMM`,
`flankAngleDeg`, `pointsPerMM`, `samplesPerTurn` exactly as the native helical thread but
with the helix radius **tapering linearly** from `topRadiusMM` to `tipRadiusMM` over the
`turns` (a conical thread). The section SHALL remain RADIAL at every station via the same
axis auxiliary-spine law (`r̂(θ)` apex, `+Z` base). The builder SHALL apply the same
`samplesPerTurn` cap, the same shared-ring ruled-band tiling + end caps, and the same
self-intersection guard (which additionally rejects a `depthMM` that would cross the axis
at the tip radius). When native, the solid SHALL match the OCCT
`BRepOffsetAPI_MakePipeShell` oracle within a documented deflection tolerance. This
builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A coarse tapered thread is watertight with a radial section (host)
- GIVEN a tapered thread (`topRadiusMM > tipRadiusMM > 0`) whose `depthMM` passes the self-intersection guard at the tip radius, built on the host with no OCCT
- WHEN it is built and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the section apex direction at every station SHALL equal that station's radial unit vector `r̂(θ)` AND the enclosed volume SHALL be within the deflection band of the swept-V estimate

### Requirement: Thread self-intersection / fine-pitch guard (honest fall-through)

The native thread builder SHALL detect and REJECT (return a NULL `Shape`, never a wrong
or self-overlapping solid) any thread whose radial V-section would self-intersect:
specifically when the two base half-widths of one turn's V consume the pitch room
(`2·depthMM·tan(flankAngleDeg) ≥ pitchMM · kPitchSafety`), when a tapered thread's
`depthMM` would cross the axis at the tip radius, or when the transported section of one
turn crosses its neighbour in Z. On rejection the `NativeEngine` SHALL fall through to the
fallback (OCCT), whose auxiliary-spine pipe-shell (and documented round-profile fallback)
handles the case. The native builder SHALL NOT itself emit the round-profile fallback and
SHALL NOT be faked or approximated into an invalid solid. Degenerate input (`turns ≤ 0`,
`pitchMM ≤ 0`, `depthMM ≤ 0`, `flankAngleDeg ∉ (0, 90)`, or a non-positive helix radius)
SHALL likewise return a NULL `Shape`.

#### Scenario: A fine-pitch / large-depth thread is rejected and falls through (host)
- GIVEN a thread where `2·depthMM·tan(flankAngleDeg) ≥ pitchMM` (the flanks of adjacent turns would cross), built on the host
- WHEN the thread builder is invoked
- THEN the builder SHALL return a NULL `Shape` (no self-intersecting solid, no faked round profile) AND `NativeEngine` SHALL fall through to the fallback for that call

#### Scenario: A degenerate thread input is rejected (host)
- GIVEN `turns ≤ 0` OR `pitchMM ≤ 0` OR `depthMM ≤ 0` OR `flankAngleDeg ∉ (0, 90)` OR a non-positive helix radius, built on the host
- WHEN the thread builder is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: NativeEngine builds the tapered shank natively (and threads where achievable), else falls through

`NativeEngine` SHALL override `tapered_shank` to call the native `build_tapered_shank` in
`src/native/construct/` and type-erase the resulting native `topology::Shape` into a
tracked native `EngineShape`; on a NULL native result (degenerate input) it SHALL fall
through to the held fallback engine with no native interception. Where the thread ops pass
both verification gates, `NativeEngine` SHALL likewise override `helical_thread` /
`tapered_thread` to call `build_helical_thread` / `build_tapered_thread`, falling through
on a NULL result (a guard trip or a declined case); where a thread op does NOT pass the
gates, its override SHALL remain a pure labelled fall-through (see the MODIFIED
deferred-ops requirement). A native shank/thread body SHALL be read back by the existing
native body-consuming paths (tessellate / face_meshes / mass_properties / bounding_box /
subshape_ids). OCCT SHALL be referenced ONLY under `CYBERCAD_HAS_OCCT` (in the fallback
wiring); the native builder SHALL reference no OCCT / `IEngine` / `EngineShape` type. No
`cc_*` signature or POD layout SHALL change.

#### Scenario: A tapered shank is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a valid `cc_tapered_shank` input
- WHEN `cc_tapered_shank` is invoked
- THEN the shape SHALL be built by `src/native/construct/` with no call to the fallback AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported/guarded case falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (a degenerate shank, a fine-pitch / large-depth thread, or a thread op that did not pass the gates)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native builder SHALL return NULL (or the op is a pure fall-through) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Tier-D ops parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME
`cc_tapered_shank` (and, where native, `cc_helical_thread` / `cc_tapered_thread`) calls
SHALL be issued once with the native engine active (`cc_set_engine(1)`) and once with the
OCCT default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through
the `cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight
tessellation — against the OCCT `BRepPrimAPI_MakeRevol` (shank) /
`BRepOffsetAPI_MakePipeShell` (thread, auxiliary-spine mode) oracle within a documented
deflection tolerance. The fall-through cases (a degenerate shank, a fine-pitch /
large-depth thread, and any thread op that could not be made native) SHALL be asserted
identical under both engines (fall-through proof). The parity test SHALL restore the OCCT
default in teardown and SHALL carry its own `main()` (on the `run-sim-suite.sh` SKIP list)
so the 221-assertion suite count is unchanged.

#### Scenario: Native tapered shank matches the OCCT MakeRevol oracle (parity)
- GIVEN a valid `cc_tapered_shank` on a booted iOS simulator
- WHEN it is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented deflection tolerance AND the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

#### Scenario: Fall-through Tier-D cases are identical under both engines (parity)
- GIVEN a degenerate shank, a fine-pitch / large-depth thread, and any thread op not made native, on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

### Requirement: Native Torus surface of revolution (off-axis-arc revolve)

The native math library SHALL provide a `Torus` analytic surface (a placement frame `Ax3`
with the revolve axis as its Z, a `major` radius, and a `minor` radius) whose point and
outward normal are `P(u,v) = C + (major + minor·cos v)·(cos u·X + sin u·Y) + minor·sin v·Z`
and `N(u,v) = cos v·(cos u·X + sin u·Y) + sin v·Z`, where `u` is the revolve angle about
the axis and `v` the angle around the tube (added in `src/native/math/torus.h`). The native
construction library SHALL, in `cc_solid_revolve_profile`, recognise a kind-1 arc segment
whose supporting-circle centre lies OFF the revolve axis and build the swept face as a
**torus band** — `major` = the signed distance of the arc-circle centre from the axis,
`minor` = the arc's circle radius, trimmed to `u ∈ [0, angle]` (the revolve angle) and `v`
spanning the arc's own angular range — with its rim edges as shared `Circle` arcs (the
revolved arc endpoints). AS BUILT, the band is emitted as EXACT rational-quadratic B-spline
patches (the angular circle is an exact rational NURBS; the existing tessellator meshes the
`BSpline` `FaceSurface` kind), so NO new `FaceSurface::Kind::Torus` was required — the
observable geometry is identical to an analytic `Torus` band. A full 2π revolve SHALL close
the u-loop (tiled in < π patches, as the other full-turn surfaces of revolution are); a
partial angle SHALL add two planar meridian caps → a watertight native `Solid` oriented
outward. The enclosed volume of a full-tube solid torus (a full-circle
profile revolved 2π) SHALL equal `2π²·major·minor²` within the tessellation deflection
bound; a partial arc/angle SHALL scale by the swept fractions. This surface and builder
SHALL remain OCCT-free and host-buildable.

#### Scenario: An off-axis-arc revolve is a watertight torus band with the analytic volume (host)
- GIVEN a kind-1 arc whose supporting-circle centre is OFF the revolve axis (major radius `R`, minor radius `r`), revolved through an angle, built on the host with no OCCT
- WHEN the profile is revolved and tessellated by `src/native/tessellate`
- THEN the swept face SHALL be a `Torus` band AND the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume of a full-tube 2π solid torus SHALL equal `2π²·R·r²` within the deflection bound

#### Scenario: A native off-axis-arc torus revolve matches the OCCT MakeRevol oracle (parity)
- GIVEN an off-axis-arc `cc_solid_revolve_profile` on a booted iOS simulator, where the native builder produces a watertight solid
- WHEN it is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the documented deflection tolerance AND the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

### Requirement: Native kind-3 spline profile edge extrude

The native construction library SHALL, in `cc_solid_extrude_profile` /
`cc_solid_extrude_profile_polyholes`, build a watertight prism from a typed profile whose
outer loop includes a **kind-3 SPLINE** segment: it SHALL resolve the segment's control
points (`splineXY`, whose `splineXYCount` is the number of DOUBLES = 2× the control-point
count) into a native `BSpline` edge curve (clamped uniform knot vector, degree
`min(3, n−1)`), assemble the closed PLANAR profile loop from the ordered typed segments
(lines / arcs / the spline edge), and EXTRUDE it by the straight translation `depth·ẑ` into
a swept **B-spline side wall** (a `FaceSurface::Kind::BSpline` surface ruling the spline
edge along the extrude direction) capped with the two planar end faces. The spline rim
edges SHALL be shared `BSpline` curves so the wall and caps weld watertight via the
two-stage tessellator. The enclosed volume SHALL equal the extruded planar area × `depth`
within the deflection bound (the planar area computed by the native cap triangulator over
the densely-sampled spline loop). A NON-planar spline loop, or a spline profile that
self-intersects (not a simple closed curve), SHALL be rejected (NULL `Shape` →
`NativeEngine` falls through). This builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A kind-3 spline-edge extrude is a watertight prism with the extruded-area volume (host)
- GIVEN a typed profile whose outer loop includes a kind-3 spline segment forming a simple closed PLANAR curve, extruded by `depth`, built on the host with no OCCT
- WHEN the prism is built and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the swept wall over the spline edge SHALL be a `BSpline` surface AND the enclosed volume SHALL equal the profile's planar area × `depth` within the deflection bound

#### Scenario: A non-planar or self-intersecting spline profile is rejected (host)
- GIVEN a kind-3 spline profile that is non-planar OR self-intersects, built on the host
- WHEN the extrude builder is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: Native N-section ruled loft chain (3+ sections)

The native construction library SHALL build, in `cc_solid_loft` / `cc_solid_loft_wires`, a
watertight ruled `Solid` from a chain of **N ≥ 2 sections** (the facade's multi-section
loft) where every section is a closed polygon with the SAME vertex count `n ≥ 3` and the
two END sections are PLANAR. For each consecutive section pair `(k, k+1)` and each edge `i`
the builder SHALL span one **bilinear ruled band** (`detail::ruledSideFace`, the Tier-B
band builder) with corresponding vertices paired 1:1, SHARING each interior section's
vertex ring between the band below `(k−1, k)` and the band above `(k, k+1)` (a C0 ruled
skin, C1 where consecutive section directions agree), and SHALL cap ONLY the first and last
sections with planar faces → a watertight `Solid` oriented outward. The enclosed volume
SHALL equal the sum over each consecutive slab of the between-two-parallel-sections
(prismatoid/frustum) volume, within the deflection bound. Mismatched vertex counts across
any pair, or a non-planar END section, SHALL be rejected (NULL `Shape` → `NativeEngine`
falls through). This builder SHALL remain OCCT-free and host-buildable.

#### Scenario: A 3+-section loft chain is a watertight solid with the summed-frustum volume (host)
- GIVEN N ≥ 3 aligned planar sections with EQUAL vertex counts (e.g. square → square → smaller-square), built on the host with no OCCT
- WHEN the chain is lofted and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the enclosed volume SHALL equal the sum of the consecutive between-section frustum volumes within the deflection bound

#### Scenario: A native N-section loft matches the OCCT ThruSections oracle (parity)
- GIVEN an N-section (N ≥ 3) equal-count planar loft chain on a booted iOS simulator
- WHEN it is called with the native engine active (`cc_set_engine(1)`) and with the OCCT default (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the documented deflection tolerance AND the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

#### Scenario: A mismatched-count or non-planar-cap loft chain falls through (host)
- GIVEN a loft chain with mismatched vertex counts across a pair OR a non-planar end section, built on the host
- WHEN the loft builder is invoked
- THEN NO solid SHALL be produced (the builder returns a NULL `Shape`) AND `NativeEngine` SHALL fall through to the fallback for that call

### Requirement: Native non-planar sweep — rotation-minimizing frame

The native construction library SHALL sweep a closed profile along a **NON-PLANAR** 3D
spine (previously OCCT-only) using a **rotation-minimizing frame (RMF)** computed by the
double-reflection method (Wang, Jüttler, Zheng, Liu 2008): the frame's up-vector SHALL be
transported segment-to-segment with minimal rotation about the tangent (two reflections per
step), so the section follows the spine WITHOUT accumulating spurious twist. For a PLANAR
spine the RMF SHALL reduce to the Tier-C constant frame (preserving OCCT parity as a special
case). The transported station rings SHALL be tiled with bilinear ruled bands
(`detail::ruledSideFace`) + planar end caps → a watertight `Solid` oriented outward, matching
the OCCT `BRepOffsetAPI_MakePipe` oracle in volume / bbox / centroid / watertightness within
a documented deflection tolerance. This builder SHALL remain OCCT-free and host-buildable.

> HONEST OUTCOME. The RMF non-planar sweep landed NATIVE (verified: the smooth-arc sweep is
> EXACT vs OCCT, vol rel 3.44e-16, watertight, 98→98 faces; the RMF collapsing to the
> constant frame on a planar spine preserves Tier-C parity). The **accumulating
> twist/scale** `cc_twisted_sweep` and the **guided/rail** cases were ATTEMPTED but did NOT
> self-verify as watertight, oracle-correct solids within the bound and therefore stay
> labelled OCCT fall-through (see the MODIFIED deferred-ops requirement): a real-twist sweep
> delegates to OCCT `ThruSections`, a self-intersecting sweep to OCCT `MakePipe`, and a hard
> curved rail to OCCT `MakePipeShell` — each with native active (rel 0.00e+00, no
> interception), never faked. These remaining fall-throughs specifically need SSI / Tier-4
> (surface–surface intersection + trimming).

#### Scenario: A non-planar-spine sweep is watertight with a rotation-minimizing frame (host)
- GIVEN a closed profile swept along a NON-PLANAR spine that does not self-intersect, built on the host with no OCCT
- WHEN it is swept and tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) AND the transported frame SHALL be rotation-minimizing (zero incremental twist about the tangent between consecutive stations) AND the enclosed volume SHALL be within the deflection band of the swept-section estimate

#### Scenario: A planar-spine sweep still matches the OCCT MakePipe oracle (parity)
- GIVEN a closed profile swept along a smooth PLANAR spine on a booted iOS simulator
- WHEN it is called native (`cc_set_engine(1)`) vs OCCT (`cc_set_engine(0)`)
- THEN the two shapes' mass properties, bounding box, and watertightness SHALL agree within the documented deflection tolerance (the RMF collapses to the constant frame — verified EXACT, vol rel ≤ 1e-15)

#### Scenario: A real-twist / real-scale twisted sweep or a guided/rail case falls through (parity)
- GIVEN a `cc_twisted_sweep` with a non-zero `twistRadians` / `scaleEnd ≠ 1`, a self-intersecting sweep, or a `cc_guided_sweep` / `cc_loft_along_rail` hard-rail case, on a booted iOS simulator
- WHEN each is called native (`cc_set_engine(1)`) vs OCCT (`cc_set_engine(0)`)
- THEN the native result SHALL be identical to the OCCT result (`cc_active_engine()==1`, rel 0.00e+00 — the native builder returned NULL or the self-verify DISCARDED the candidate, so the call fell through to OCCT with no interception; SSI is Tier 4, not attempted)

### Requirement: Thread self-intersection resolver (root clamp; truly self-intersecting still falls through)

The native thread builder SHALL carry a root-clamp resolver for the NEAR-self-intersecting
fine-pitch band: where a radial V-section thread's flanks approach but do not truly cross,
the builder MAY CLAMP each flank's axial half-width to at most `pitch/2 − ε` (truncated-root
geometry, preserving the crest/apex depth) so the swept ruled bands weld watertight. The
well-formed `cc_helical_thread` / `cc_tapered_thread` that were native from Tier D SHALL stay
native (`boundaryEdgeCount == 0` across the deflection ladder). A **genuinely
self-intersecting** thread — one whose flanks TRULY cross (the crest of one turn would pass
the crest of the next), a non-manifold swept surface no matter how the vertices weld — SHALL
fail the watertight self-verify and be rejected (NULL `Shape` → `NativeEngine` falls through
to OCCT `MakePipeShell`). Resolving that genuinely self-intersecting case requires
surface–surface intersection (trimming the overlapping flanks) and is Tier 4, NOT attempted
here. The tapered tip-crossing and degenerate guards are unchanged. This builder SHALL remain
OCCT-free and host-buildable.

> HONEST OUTCOME. The root-clamp resolver did NOT extend the native set beyond the
> well-formed threads already native from Tier D — the tested self-intersecting thread still
> falls through to OCCT `MakePipeShell` (vol 1446.76, native active=1, rel 0.00e+00, never
> faked). Widening the native set into truly-crossing geometry needs Tier-4 SSI.

#### Scenario: The well-formed threads stay native and watertight (host)
- GIVEN a well-formed `cc_helical_thread` / `cc_tapered_thread` (the Tier-D native set), built on the host with no OCCT
- WHEN the thread is built + tessellated
- THEN the mesh SHALL be watertight (`boundaryEdgeCount == 0`) across the deflection ladder with a positive volume

#### Scenario: A genuinely self-intersecting thread still falls through (host + parity)
- GIVEN a thread whose flanks TRULY cross (the crest of one turn passes the next), built on the host and on a booted iOS simulator
- WHEN the thread builder is invoked
- THEN NO native solid SHALL be produced (the builder returns a NULL `Shape`, no self-intersecting solid, no SSI attempted) AND `NativeEngine` SHALL fall through to OCCT (identical to `cc_set_engine(0)`, rel 0.00e+00)

### Requirement: NativeEngine builds the completed residuals natively (with self-verify), else falls through

`NativeEngine` SHALL route each affected op — `solid_extrude_profile`,
`solid_extrude_profile_polyholes`, `solid_revolve_profile`, `solid_loft`,
`solid_loft_wires`, `solid_sweep`, `twisted_sweep`, `guided_sweep`, `loft_along_rail`,
`helical_thread`, `tapered_thread` — through its native builder in `src/native/construct/`
and type-erase a produced native `topology::Shape` into a tracked native `EngineShape`.
Before serving a native result `NativeEngine` SHALL run a MANDATORY self-verify — the mesh
is watertight (`robustlyWatertight`) AND the volume/geometry is correct (positive, matching
the analytic expectation where one exists) — and SHALL DISCARD any candidate that fails,
falling through to the held fallback engine with no native interception (never a
wrong / leaky / faked solid). On a NULL native result (a deferred sub-case or a guard trip)
it SHALL likewise fall through. A native body SHALL be read back by the existing native
body-consuming paths (tessellate / face_meshes / mass_properties / bounding_box /
subshape_ids / edge_polylines). OCCT SHALL be referenced ONLY under `CYBERCAD_HAS_OCCT` (in
the fallback wiring); the native builder SHALL reference no OCCT / `IEngine` / `EngineShape`
type. No `cc_*` signature or POD layout SHALL change.

#### Scenario: A completed-residual op is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and a valid input for a now-native residual (off-axis-arc torus revolve, kind-3 spline extrude, N-section loft, or a non-planar (RMF) sweep)
- WHEN the corresponding `cc_*` op is invoked
- THEN the shape SHALL be built by `src/native/construct/`, pass the self-verify, and be served with no call to the fallback AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: A self-verify failure or deferred sub-case falls through under the native engine
- GIVEN the native engine active and an input whose native result fails the self-verify OR a deferred sub-case (spline-revolve, mismatched loft, self-intersecting / tight-curvature sweep, hard rail, or a truly self-intersecting thread)
- WHEN the corresponding `cc_*` op is invoked
- THEN the native candidate SHALL be DISCARDED (or the builder returns NULL) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Completed-residual ops parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): each now-native
residual op SHALL be issued once with the native engine active (`cc_set_engine(1)`) and once
with the OCCT default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared
through the `cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight
tessellation — against the OCCT oracle (`BRepPrimAPI_MakeRevol` for the torus revolve,
`BRepPrimAPI_MakePrism` for the spline extrude, `BRepOffsetAPI_ThruSections` for the
N-section loft, `BRepOffsetAPI_MakePipe` for the non-planar (RMF) sweep) within a documented
deflection tolerance. Every deferred / SSI / DECLINE sub-case (spline-revolve, spindle torus,
self-crossing spline, mismatched loft, self-intersecting / real-twist sweep, hard rail, truly
self-intersecting thread) SHALL be asserted identical under both engines (fall-through proof)
or DECLINED on both engines (occtId=0 natId=0). The parity test SHALL restore the OCCT default in teardown and SHALL carry its own
`main()` (on the `run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

#### Scenario: Each native residual op matches its OCCT oracle (parity)
- GIVEN a valid input for a now-native residual op on a booted iOS simulator, where the native builder produces a watertight solid
- WHEN it is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree within the documented deflection tolerance AND the native face count SHALL be an integer multiple `k ≥ 1` of the OCCT face count

#### Scenario: Deferred / SSI residual cases are identical under both engines (parity)
- GIVEN a spline-revolve, a mismatched-count loft, a self-intersecting / tight-curvature sweep, a hard pipe-shell rail, and a truly self-intersecting thread, on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change


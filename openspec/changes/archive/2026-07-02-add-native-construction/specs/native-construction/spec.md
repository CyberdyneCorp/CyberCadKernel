# native-construction

## ADDED Requirements

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

The following construction operations SHALL NOT be implemented natively in this
change and SHALL fall through the `NativeEngine` to the fallback (OCCT) engine:
`solid_loft`, `solid_loft_wires`, `solid_sweep`, `twisted_sweep`,
`loft_along_rail`, `guided_sweep`, `wrap_emboss`, `helical_thread`,
`tapered_thread`, `tapered_shank`, `solid_extrude_holes`,
`solid_extrude_polyholes`, `solid_extrude_profile`,
`solid_extrude_profile_polyholes`, `solid_revolve_profile` (typed profiles), and
any revolve whose profile contains **arc or spline** segments. Likewise every
feature / boolean / tessellate / query / transform / exchange op SHALL fall
through. The change SHALL NOT fake, stub-out, or partially implement any deferred
op; each SHALL produce exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_solid_loft`, `cc_solid_extrude_holes`, `cc_helical_thread`, or `cc_solid_revolve_profile`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A curved revolve profile falls through
- GIVEN a revolve whose profile contains arc or spline segments (arriving via `cc_solid_revolve_profile`) with the native engine active
- WHEN it is invoked
- THEN it SHALL fall through to OCCT unchanged (the native engine does not intercept `solid_revolve_profile`)

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

# native-construction — delta for add-native-loft (Phase 4 #4b Tier B)

This change extends the `native-construction` capability with a **native
two-section ruled loft** (`cc_solid_loft`, `cc_solid_loft_wires`) for sections with
**equal point counts**, and narrows the existing deferred-ops requirement so loft is
no longer in the fall-through-only list (mismatched-count / punctual / non-planar /
3+-section loft configurations stay fall-through). No `cc_*` ABI, POD layout, or
default-engine change.

## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL NOT be implemented natively and SHALL fall
through the `NativeEngine` to the fallback (OCCT) engine: `solid_sweep`,
`twisted_sweep`, `loft_along_rail`, `guided_sweep`, `wrap_emboss`, `helical_thread`,
`tapered_thread`, `tapered_shank`. In addition, these **sub-cases** of otherwise-native
ops SHALL fall through (the native builder returns a NULL `Shape` and the engine
delegates): a **kind 3 SPLINE** profile edge in `solid_extrude_profile` /
`solid_extrude_profile_polyholes`; a **kind 1 arc whose supporting-circle centre is OFF
the axis** (a torus surface of revolution) in `solid_revolve_profile`; any
**spline-revolve** (a kind 3 segment in `solid_revolve_profile`, or a B-spline surface
of revolution); and, for the otherwise-native two-section ruled loft (`solid_loft`,
`solid_loft_wires`), the **loft configurations** with **mismatched section point
counts**, a **punctual (point) section**, a **non-planar** section whose cap cannot be
built as a single planar face, a **self-intersecting** section or ruling, or **3 or
more** sections. Likewise every feature / boolean / tessellate-of-a-foreign-body /
query / transform / exchange op SHALL fall through. The change SHALL NOT fake,
stub-out, or partially implement any deferred op, sub-case, or loft configuration; each
SHALL produce exactly the fallback engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_solid_sweep`, `cc_twisted_sweep`, `cc_loft_along_rail`, `cc_guided_sweep`, or `cc_helical_thread`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred sub-case (a kind-3 SPLINE profile edge, an off-axis arc/torus revolve, or a spline-revolve)
- WHEN the corresponding `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` / `cc_solid_revolve_profile` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred loft configuration falls through
- GIVEN the native engine active and a loft configuration outside the native two-section equal-count scope (mismatched point counts, a punctual section, a non-planar `cc_solid_loft_wires` section, or a 3+-section loft)
- WHEN `cc_solid_loft` / `cc_solid_loft_wires` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

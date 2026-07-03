# native-construction

This change (Phase 4 #4b Tier C) extends the living `native-construction` capability:
it moves **`cc_solid_sweep`** NATIVE in `NativeEngine` for the tractable path shapes —
a **straight** path (exact, a directional extrude) and a **smooth curved but PLANAR**
path (a **constant-frame** swept surface that reproduces OCCT `MakePipe`'s planar
corrected-Frenet law, matching the oracle to fp precision) — and serves
`cc_twisted_sweep` natively only when it reduces to the plain sweep (twist ≈ 0, scale
≈ 1). It narrows the "deferred ops fall through" list to remove `solid_sweep` (and the
no-op `twisted_sweep`) while keeping the genuinely hard cases — a **non-planar** curved
spine, tight-curvature / self-intersecting sweeps, a **real** twist/scale,
`cc_guided_sweep`, `cc_loft_along_rail` — as honest, labelled OCCT fall-through. No
`cc_*` ABI change; default engine stays OCCT.

> NOTE (implementation reality, verified against the oracle): an earlier design used a
> rotation-minimizing frame (RMF) that keeps the section perpendicular to the tangent
> throughout, giving the Pappus `profileArea · arcLength` volume. That is geometrically
> "nicer" but does **not** match the `BRepOffsetAPI_MakePipe` oracle: for a **planar**
> spine MakePipe's default `GeomFill_CorrectedFrenet` law collapses to a **constant
> rotation** (OCCT `GeomFill_CorrectedFrenet.cxx`, `isPlanar` → `Law_Constant`), so the
> section is **translated with a fixed orientation**, not rotated. The native builder
> therefore holds the **start frame constant** across all stations; on the simulator the
> native `cc_solid_sweep` then matches the OCCT volume/bbox/centroid/face-count to fp
> precision (vol rel ≈ 1.7e-16) on the parity fixture. A non-planar spine needs OCCT's
> genuine (non-constant) law and is deferred.

## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL NOT be implemented natively and SHALL fall
through the `NativeEngine` to the fallback (OCCT) engine: `solid_loft`,
`solid_loft_wires`, `guided_sweep`, `loft_along_rail`, `wrap_emboss`, `helical_thread`,
`tapered_thread`, `tapered_shank`. (`solid_sweep` and `twisted_sweep` are NOW native for
the tractable path shapes — see the native sweep requirements — and fall through only in
their deferred sub-cases below.) In addition, these **sub-cases** SHALL fall through (the
native builder returns a NULL `Shape` and the engine delegates): for **sweeps**, a
**non-planar curved spine** (OCCT's non-constant corrected-Frenet law is required), a
**tight-curvature or self-intersecting** path (curvature radius below the profile's
radial extent, or a self-crossing spine/section), and a **`twisted_sweep` with a real
twist/scale** (only the no-op twist reduces to the native sweep); for the
otherwise-native **Tier-A profile ops**, a
**kind 3 SPLINE** profile edge in `solid_extrude_profile` /
`solid_extrude_profile_polyholes`, a **kind 1 arc whose supporting-circle centre is OFF
the axis** (a torus surface of revolution) in `solid_revolve_profile`, and any
**spline-revolve** (a kind 3 segment in `solid_revolve_profile`, or a B-spline surface of
revolution). Likewise every feature / boolean / tessellate-of-a-foreign-body / query /
transform / exchange op SHALL fall through. The change SHALL NOT fake, stub-out, or
partially implement any deferred op or sub-case; each SHALL produce exactly the fallback
engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_solid_loft`, `cc_helical_thread`, `cc_guided_sweep`, or `cc_loft_along_rail`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A deferred sweep sub-case falls through
- GIVEN the native engine active and a sweep hitting a deferred sub-case (a non-planar curved spine, a tight-curvature / self-intersecting path, or a twisted sweep with a real twist/scale)
- WHEN `cc_solid_sweep` / `cc_twisted_sweep` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred profile sub-case (a kind-3 SPLINE profile edge, an off-axis arc/torus revolve, or a spline-revolve)
- WHEN the corresponding `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` / `cc_solid_revolve_profile` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

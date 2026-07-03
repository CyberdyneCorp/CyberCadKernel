# native-construction

This change (Phase 4 #4b Tier D) extends the living `native-construction` capability:
it moves **`cc_tapered_shank`** NATIVE in `NativeEngine` (a revolve of the pointed-shank
silhouette — a special case of the already-verified native revolve, exact/deflection-
bounded vs OCCT `BRepPrimAPI_MakeRevol`) and **attempts** **`cc_helical_thread`** /
**`cc_tapered_thread`** natively (a RADIAL V/triangular section swept along a helical
spine under an **axis auxiliary-spine** law so the V does NOT Frenet-rotate, tiled into
ruled bands with shared rings + end caps to a watertight solid, matching OCCT
`BRepOffsetAPI_MakePipeShell` in its auxiliary-spine mode within a deflection bound). It
narrows the "deferred ops fall through" list to remove `tapered_shank` (now native) and
records the thread ops honestly: they are native ONLY where the radial-V helical sweep is
verified watertight + oracle-correct, and OTHERWISE remain labelled OCCT fall-through
(never faked). No `cc_*` ABI change; default engine stays OCCT.

> HONEST SCOPE NOTE. `cc_tapered_shank` is expected to land native (both gates green) —
> it reuses `build_revolution`, which is already parity-verified. The two thread ops are
> genuinely hard: a V-section swept along a helix self-intersects at fine pitch / large
> depth, and keeping the section RADIAL requires an axis auxiliary-spine law (OCCT
> `BRepOffsetAPI_MakePipeShell::SetMode(AuxiliarySpine, …)`, consulted as an oracle only)
> that the native ruled-band assembler must reproduce. If the radial-V helical sweep
> cannot be made watertight + oracle-correct for the test cases, the thread ops REMAIN
> OCCT-fall-through and are covered by the (MODIFIED) deferred-ops requirement — the
> change then lands `cc_tapered_shank` native only, reported honestly. A thread
> requirement below is satisfied natively only when BOTH gates are green for that op.

## ADDED Requirements

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

## MODIFIED Requirements

### Requirement: Deferred construction ops fall through to OCCT (honest scope)

The following construction operations SHALL NOT be implemented natively and SHALL fall
through the `NativeEngine` to the fallback (OCCT) engine: `solid_loft`,
`solid_loft_wires`, `guided_sweep`, `loft_along_rail`, `wrap_emboss`. (`solid_sweep` and
`twisted_sweep` are native for the tractable path shapes; `tapered_shank` is NOW native — a
revolve of the pointed-shank silhouette — and no longer falls through.) The thread ops
`helical_thread` and `tapered_thread` are **attempted natively** (a radial V-section swept
along a helical spine under an axis auxiliary-spine law — see the native thread
requirements); each is native ONLY where its radial-V helical sweep is verified watertight
+ oracle-correct on both gates, and OTHERWISE SHALL fall through to OCCT (labelled,
verified, never faked). In addition, these **sub-cases** SHALL fall through (the native
builder returns a NULL `Shape` and the engine delegates): for **threads**, a **fine-pitch
/ large-depth** thread whose radial V self-intersects (`2·depthMM·tan(flankAngleDeg) ≥
pitchMM`, or a tapered `depthMM` crossing the axis at the tip), and any thread requiring
the **round-profile fallback** (the native builder returns NULL and OCCT applies that
fallback — the native path does not fake it); for **sweeps**, a **non-planar curved spine**
(OCCT's non-constant corrected-Frenet law is required), a **tight-curvature or
self-intersecting** path (curvature radius below the profile's radial extent, or a
self-crossing spine/section), and a **`twisted_sweep` with a real twist/scale** (only the
no-op twist reduces to the native sweep); for the otherwise-native **Tier-A profile ops**,
a **kind 3 SPLINE** profile edge in `solid_extrude_profile` /
`solid_extrude_profile_polyholes`, a **kind 1 arc whose supporting-circle centre is OFF
the axis** (a torus surface of revolution) in `solid_revolve_profile`, and any
**spline-revolve** (a kind 3 segment in `solid_revolve_profile`, or a B-spline surface of
revolution). Likewise every feature / boolean / tessellate-of-a-foreign-body / query /
transform / exchange op SHALL fall through. The change SHALL NOT fake, stub-out, or
partially implement any deferred op or sub-case; each SHALL produce exactly the fallback
engine's result.

#### Scenario: A deferred construction op yields the OCCT result through the native engine
- GIVEN the native engine active (`cc_set_engine(1)`) on an OCCT build
- WHEN a deferred op is invoked (e.g. `cc_solid_loft`, `cc_guided_sweep`, `cc_loft_along_rail`, or `cc_wrap_emboss`)
- THEN the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A deferred thread sub-case (or a non-native thread op) falls through
- GIVEN the native engine active and a thread hitting a deferred case (a fine-pitch / large-depth self-intersecting thread, a round-profile-fallback thread, or a thread op that did not pass both gates)
- WHEN `cc_helical_thread` / `cc_tapered_thread` is invoked
- THEN the native builder SHALL return NULL (or the op is a pure fall-through) AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred sweep sub-case falls through
- GIVEN the native engine active and a sweep hitting a deferred sub-case (a non-planar curved spine, a tight-curvature / self-intersecting path, or a twisted sweep with a real twist/scale)
- WHEN `cc_solid_sweep` / `cc_twisted_sweep` is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

#### Scenario: A deferred profile sub-case falls through
- GIVEN the native engine active and an input hitting a deferred profile sub-case (a kind-3 SPLINE profile edge, an off-axis arc/torus revolve, or a spline-revolve)
- WHEN the corresponding `cc_solid_extrude_profile` / `cc_solid_extrude_profile_polyholes` / `cc_solid_revolve_profile` call is invoked
- THEN the native builder SHALL return NULL AND the call SHALL fall through to OCCT unchanged (identical to `cc_set_engine(0)`)

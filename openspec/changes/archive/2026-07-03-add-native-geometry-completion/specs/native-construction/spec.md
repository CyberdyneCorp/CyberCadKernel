# native-construction

This change (Phase 4 #4b Tier 1 + Tier 2#4) extends the living `native-construction`
capability by ATTEMPTING to move NATIVE — each only where verified a watertight,
oracle-correct solid on BOTH gates — the four remaining construction residuals: **(A)** a
kind-3 SPLINE outer profile edge (extrude) and an OFF-AXIS-ARC revolve (a native `Torus`
surface of revolution — this change ADDS the `Torus` surface); **(B)** a 3+-section ruled
loft chain (`cc_solid_loft` / `_wires` beyond two sections); **(C)** a general sweep with a
NON-PLANAR spine + accumulating twist/scale via a rotation-minimizing frame
(`cc_solid_sweep` / `cc_twisted_sweep`) and best-effort guided/rail
(`cc_guided_sweep` / `cc_loft_along_rail`); and **(D)** a thread self-intersection RESOLVER
so more fine-pitch parameters weld watertight. It NARROWS the deferred-ops requirement to
remove the now-native residuals and records the honest per-area split. No `cc_*` ABI
change; default engine stays OCCT.

> HONEST SCOPE NOTE. Each area is ATTEMPTED and lands native ONLY where its result is a
> valid watertight solid with the correct volume/geometry on BOTH gates (host analytic +
> sim native-vs-OCCT parity); otherwise the native builder returns a NULL `Shape` (or the
> `NativeEngine` self-verify DISCARDS the candidate) and the op falls through to OCCT —
> labelled, verified rel~0, NEVER faked. **Surface–surface intersection (SSI) is NOT
> attempted here (that is Tier 4)**: a self-intersecting sweep, a tight-curvature fold, a
> hard pipe-shell rail, and a genuinely self-intersecting thread MUST fall through. A
> requirement below is satisfied natively only when BOTH gates are green for that op;
> whatever cannot be is covered by the (MODIFIED) deferred-ops requirement.

## ADDED Requirements

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

## MODIFIED Requirements

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

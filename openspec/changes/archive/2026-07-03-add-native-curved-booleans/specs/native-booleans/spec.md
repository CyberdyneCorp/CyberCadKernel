# native-booleans

This change (Phase 4 #5, RESEARCH-GRADE — the CURVED slice) extends the `native-booleans`
capability with ONE analytic curved family: it makes **`cc_boolean`** (`op`: `0` fuse,
`1` cut `a−b`, `2` common) NATIVE in `NativeEngine` for an **AXIS-ALIGNED box ↔ cylinder**
(cylinder axis PARALLEL to a box axis), where plane-cylinder intersection is CLOSED-FORM
(a box face ⟂ the axis cuts the cylinder in a `Circle`; a box face ∥ the axis cuts it in
`Line` rulings). The builder splits the cylindrical lateral face and the box planar faces
along these analytic curves, classifies each fragment inside/outside the other solid by a
point-in-solid test (radial + axial cylinder half-space, six box half-spaces), assembles
the surviving shell (round hole for cut, round boss for fuse, intersection for common),
and heals the shared circle / line seams watertight (reusing the two-stage shared-edge
mesher). The analytic volume (`π·r²·h`) is EXACT, so the **mandatory self-verify guard**
checks it and DISCARDS any bad native result → OCCT. Sphere, cone, NON-axis-aligned
cylinders, cylinder-cylinder, NURBS, near-tangent / coincident-curved, and ALL general
curved cases fall through to OCCT (labelled, verified, never faked). No `cc_*` ABI change;
default engine stays OCCT; the archived PLANAR slice keeps working.

> NOTE (honest scope): this is a **narrow analytic slice** (axis-aligned box-cylinder) of
> the research-grade curved boolean. A general curved B-rep boolean over arbitrary
> analytic + NURBS solids — surface-surface intersection curves, robust near-tangent
> handling, and full shape healing — remains research-grade OCCT-backed future work. It is
> fully acceptable that ONLY axis-aligned box-cylinder cut / fuse / common lands native and
> everything else is OCCT-fallback; this spec states that split truthfully.

## ADDED Requirements

### Requirement: Native axis-aligned box-cylinder boolean (fuse / cut / common)

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse (`A ∪ B`,
round boss), `op = 1` cut (`A − B`, round hole), `op = 2` common (`A ∩ B`) — NATIVELY when
one operand is a native **axis-aligned box** (six planar axis-aligned faces) and the other
is a native **cylinder solid** (one `FaceSurface::Kind::Cylinder` lateral face + two planar
`Plane` caps bounded by `Circle` edges) whose **axis is parallel to a box axis** (and to a
world axis). The builder SHALL use the **analytic plane-cylinder intersection**: a box face
**perpendicular** to the cylinder axis SHALL trace a `Circle` on the cylinder (splitting the
box face into an outer loop with an inner circular hole, and capping the cylinder there with
a `Plane` disk bounded by that `Circle`); a box face **parallel** to the axis SHALL trace
axial `Line` rulings (splitting the cylindrical lateral face into angular fragments and the
box face along the chord). The intersection curves SHALL be kept as TRUE `Circle` / `Line`
`EdgeCurve`s in the B-rep (curved faces chorded only at tessellation, deflection-bounded —
never a chord polyline in the B-rep). The builder SHALL classify each fragment as INSIDE /
OUTSIDE / ON the other solid by a point-in-solid test (`dist_to_axis ≤ r` within the axial
span for the cylinder; six half-spaces for the box), select the **surviving** fragments per
the op (fuse: box outside cyl + cyl outside box; cut: box outside cyl + cyl-lateral inside
box REVERSED + entry/exit cap disks; common: box inside cyl + cyl inside box), orient them
outward, and **heal** the shared `Circle` / `Line` seams into one closed watertight `Solid`.
The result SHALL be a native `topology::Shape` of type `Solid`, watertight (every edge shared
by exactly two faces, `boundaryEdgeCount == 0`), with its curved faces `Cylinder` surfaces
and its curved seam edges `Circle` curves; its enclosed volume SHALL equal the exact analytic
set-algebra value for the op (`boxVol − π·r²·h` cut / `boxVol + π·r²·h_boss − overlap` fuse /
`π·r²·h_overlap` common) within the curved-face deflection bound. On the axis-aligned
box-cylinder case the native result SHALL match the OCCT oracle (analytic volume ~exact,
curved faces deflection-bounded). This builder SHALL remain OCCT-free and host-buildable and
SHALL reference no OCCT / `IEngine` / `EngineShape` type.

#### Scenario: A round hole (cut) through a box is a watertight solid with exact analytic volume (host)
- GIVEN an axis-aligned box `A` and an axis-parallel through-cylinder `B` (radius `r`, height `h` spanning the box), both native, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 1)` (cut, `A − B`) is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) whose lateral tunnel wall is a `Cylinder` face bounded by `Circle` edges AND its enclosed volume SHALL equal `boxVol − π·r²·h` within the curved-face deflection bound

#### Scenario: A round boss (fuse) on a box is a watertight solid with exact analytic volume (host)
- GIVEN an axis-aligned box `A` and an axis-parallel cylinder `B` (a boss standing on a box face), both native, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 0)` (fuse, `A ∪ B`) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal `boxVol + π·r²·h_boss − π·r²·h_overlap` within the curved-face deflection bound

#### Scenario: A box-cylinder common is a watertight solid with exact analytic volume (host)
- GIVEN an axis-aligned box `A` and an axis-parallel cylinder `B`, both native, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 2)` (common, `A ∩ B`) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal `π·r²·h_overlap` within the curved-face deflection bound

#### Scenario: The curved cap↔lateral circle seam welds watertight across the deflection ladder (host)
- GIVEN a native axis-aligned box-cylinder boolean result whose cap `Plane` disk shares a `Circle` seam with the `Cylinder` lateral patch
- WHEN it is tessellated at every deflection in the mesher's deflection ladder
- THEN the shared circle seam SHALL weld (both faces pinned to the ONE shared arc discretization) AND `boundaryEdgeCount == 0` at every deflection

## MODIFIED Requirements

### Requirement: Mandatory boolean self-verify guard (discard and fall through)

The engine SHALL accept a native boolean result as native ONLY when it PASSES a mandatory
self-verify: the candidate SHALL be (a) a **closed watertight 2-manifold** (closed at every
deflection in the mesher's deflection ladder, positive enclosed volume) AND (b) have the
**correct set-algebra volume sign and magnitude** for the op. For a PLANAR-faced boolean the
candidate volume `Vr` SHALL satisfy `Vr ≈ |A| + |B| − |A ∩ B|` (fuse), `Vr ≈ |A| − |A ∩ B|`
(cut), or `Vr ≈ |A ∩ B|` (common) within a fp-exact relative tolerance, using the operands'
native volumes and their native intersection volume. For an AXIS-ALIGNED box-cylinder
boolean the operand volumes are ANALYTIC, so the candidate volume `Vr` SHALL satisfy
`Vr ≈ boxVol − π·r²·h` (cut through-hole), `Vr ≈ boxVol + π·r²·h_boss − π·r²·h_overlap`
(fuse boss), or `Vr ≈ π·r²·h_overlap` (common) within a RELATIVE tolerance sized to the
curved-face tessellation deflection (curved faces are deflection-bounded, not fp-exact). If
EITHER check fails, the engine SHALL **DISCARD** the native result. The engine SHALL NEVER
emit an unverified, leaky, or wrong boolean solid; when both operands are native and the
candidate is discarded (or the builder declines), the engine SHALL report an honest error
rather than hand a native void to OCCT (which cannot read it).

#### Scenario: A bad native boolean result is discarded (host)
- GIVEN a native boolean candidate that is open / non-manifold OR has a wrong set-algebra volume for its op (planar or analytic-curved), built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong solid (a native-native case reports an honest error; a case with an OCCT operand falls through to OCCT)

#### Scenario: A bad analytic-curved candidate is discarded by the analytic-volume oracle (host)
- GIVEN a native axis-aligned box-cylinder boolean candidate whose enclosed volume is outside the analytic band for its op (`boxVol − π·r²·h` cut / `boxVol + π·r²·h_boss − overlap` fuse / `π·r²·h_overlap` common), built on the host
- WHEN the self-verify guard is applied
- THEN the analytic-volume oracle SHALL reject the candidate AND no native curved result SHALL be emitted

#### Scenario: A verified native boolean is read back by the native paths (host)
- GIVEN a native boolean result (planar or analytic-curved box-cylinder) that PASSES the self-verify (watertight 2-manifold with the correct set-algebra / analytic volume)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no fallback call

### Requirement: Curved-face, near-degenerate, and non-native boolean cases fall through to OCCT

The native boolean builder SHALL DECLINE (return a NULL `Shape`) for any case outside the
supported native domain (the planar-polyhedron slice AND the axis-aligned box-cylinder
analytic slice): (1) either operand has a **sphere, cone, or NURBS / free-form face**
(`FaceSurface::kind` `Sphere` / `Cone` / `BSpline` / `Bezier`); (2) a **NON-axis-aligned
cylinder** (its axis not parallel to a box axis — the plane-cylinder trace would be a general
ellipse / conic, not a circle / line); (3) a **cylinder-cylinder** pair (any orientation —
a surface-surface intersection curve); (4) any other **general curved** configuration outside
the axis-aligned box-cylinder family; (5) a **near-tangent / coincident-curved / degenerate**
configuration (a cylinder tangent to a box face, a coincident axis / radius, a cap plane
coincident with a cylinder end, a sliver trace, a fragment interior point near-ON the other
solid, touching-only measure-zero contact, or a self-intersecting / open / zero-volume
operand); (6) either operand is **not a native body** (a foreign / OCCT-built shape id). When
either operand is an OCCT body, each such case SHALL produce EXACTLY the fallback (OCCT)
engine's result; when both operands are native voids OCCT cannot read, the engine SHALL report
an honest error (never a faked or misread result). The change SHALL NOT fake, stub-out, or
partially implement any deferred case; each SHALL be labelled and verified as a fall-through,
never faked. The already-native axis-aligned box-cylinder cut / fuse / common (the new curved
slice) is EXCLUDED from this fall-through — it is computed natively.

#### Scenario: A sphere or cone operand falls through / declines (host + parity)
- GIVEN a boolean where at least one operand has a sphere or cone face, with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_boolean` is invoked
- THEN the native builder SHALL return a NULL `Shape` AND (with an OCCT operand) the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A non-axis-aligned cylinder or cylinder-cylinder pair falls through (host + parity)
- GIVEN a boolean whose cylinder axis is NOT parallel to a box axis, OR two cylinders, with the native engine active
- WHEN `cc_boolean` is invoked
- THEN the native curved builder SHALL return a NULL `Shape` (the trace is not a circle / line, or there is no box operand) AND the result SHALL be the OCCT oracle's under `cc_set_engine(0)`, proving fall-through

#### Scenario: A near-tangent / coincident-curved configuration declines (host)
- GIVEN an axis-aligned box-cylinder configuration in a near-tangent / coincident-curved state (cylinder tangent to a box face, coincident axis / radius, or a sliver trace), with the native engine active
- WHEN `cc_boolean` is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong classification) AND the engine SHALL NOT emit a native result for that call

#### Scenario: A non-native (foreign) operand falls through (host)
- GIVEN a `cc_boolean` where at least one operand is not a native body (a foreign / OCCT-built shape id), with the native engine active
- WHEN `cc_boolean` is invoked
- THEN `NativeEngine` SHALL fall through to the fallback engine for that call (a native boolean requires the native B-rep of both operands), identical to `cc_set_engine(0)`

### Requirement: Boolean parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_boolean`
calls SHALL be issued once with the native engine active (`cc_set_engine(1)`) and once with the
OCCT default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the
`cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight tessellation —
against the OCCT `BRepAlgoAPI_Fuse` / `_Cut` / `_Common` oracle. On **axis-aligned box** fuse /
cut / common (the planar slice) the native result SHALL match the oracle EXACTLY (relative
error ~0, fp precision). On **axis-aligned box-cylinder** fuse / cut / common (the new curved
slice) the native result SHALL match the oracle within the curved-face DEFLECTION bound
(analytic volume ~exact, curved faces `Cylinder` surfaces, watertight). The fall-through cases
(a sphere / cone operand, a non-axis-aligned cylinder, a cylinder-cylinder pair, a NURBS
operand, and a near-tangent / coincident-curved configuration) SHALL be asserted identical
under both engines (fall-through proof, the OCCT path owning both voids). The parity test SHALL
restore the OCCT default in teardown and SHALL carry its own `main()` (on the
`run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

#### Scenario: Native box booleans match the OCCT BRepAlgoAPI oracle exactly (parity)
- GIVEN axis-aligned box fuse / cut / common cases (planar) on a booted iOS simulator
- WHEN each `cc_boolean` op is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT `BRepAlgoAPI` oracle

#### Scenario: Native box-cylinder booleans match the OCCT oracle within the deflection bound (parity)
- GIVEN axis-aligned box-cylinder cut (round hole) / fuse (round boss) / common cases on a booted iOS simulator
- WHEN each `cc_boolean` op is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties (analytic volume ~exact) and watertightness SHALL agree within the curved-face deflection bound with the OCCT `BRepAlgoAPI` oracle

#### Scenario: Curved fall-through boolean cases are identical under both engines (parity)
- GIVEN a sphere / cone boolean, a non-axis-aligned cylinder boolean, a cylinder-cylinder boolean, a NURBS boolean, and a near-tangent / coincident-curved boolean on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest (including the planar `test_native_boolean`), and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change

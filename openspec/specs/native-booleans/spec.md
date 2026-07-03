# native-booleans Specification

## Purpose
TBD - created by archiving change add-native-booleans. Update Purpose after archive.
## Requirements
### Requirement: Native planar-faced boolean (fuse / cut / common)

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse (`A ∪ B`),
`op = 1` cut (`A − B`), `op = 2` common (`A ∩ B`) — NATIVELY when BOTH operands are native
solids whose every face is a `FaceSurface` of kind `Plane` (polyhedra: boxes, prisms, convex
or simple-concave). The builder SHALL: (1) compute the **face–face intersection segments**
between the two solids (each planar face pair's supporting-plane line clipped to both face
polygons); (2) **split** each face along the segments crossing it into face fragments;
(3) **classify** each fragment as INSIDE / OUTSIDE / ON the other solid by a
point-in-polyhedron test at the fragment's interior centroid; (4) select the **surviving**
fragments per the op's face-survival rule (fuse: `A` outside `B` + `B` outside `A`; cut: `A`
outside `B` + `B` inside `A` reversed; common: `A` inside `B` + `B` inside `A`), orient them
outward, and **sew/heal** them into one closed watertight `Solid`. The result SHALL be a
native `topology::Shape` of type `Solid`, watertight (every edge shared by exactly two faces),
and its enclosed volume SHALL equal the exact set-algebra value for the op. On axis-aligned
box cases the native result SHALL match the OCCT oracle EXACTLY (fp64). This builder SHALL
remain OCCT-free and host-buildable and SHALL reference no OCCT / `IEngine` / `EngineShape`
type.

#### Scenario: Axis-aligned box fuse is a watertight solid with exact union volume (host)
- GIVEN two overlapping axis-aligned boxes `A` and `B`, both native planar solids, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 0)` (fuse) is computed and the result is tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) with every edge shared by exactly two faces AND its enclosed volume SHALL equal `|A| + |B| − |A ∩ B|` exactly within fp64 tolerance

#### Scenario: Axis-aligned box cut is a watertight solid with exact difference volume (host)
- GIVEN two overlapping axis-aligned boxes `A` and `B`, both native planar solids, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 1)` (cut, `A − B`) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume SHALL equal `|A| − |A ∩ B|` exactly within fp64 tolerance

#### Scenario: Axis-aligned box common is a watertight solid with exact intersection volume (host)
- GIVEN two overlapping axis-aligned boxes `A` and `B`, both native planar solids, built on the host with no OCCT
- WHEN `cc_boolean(A, B, 2)` (common, `A ∩ B`) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume SHALL equal `|A ∩ B|` exactly within fp64 tolerance

#### Scenario: A prism / simple-concave planar boolean is watertight with exact volume (host)
- GIVEN a planar prism or simple-concave polyhedron combined with another planar solid (e.g. an L-prism cut by a box, or a convex fuse), built on the host with no OCCT
- WHEN the corresponding `cc_boolean` op is computed and tessellated
- THEN the result SHALL be a watertight `Solid` AND its enclosed volume SHALL equal the exact set-algebra value for the op

### Requirement: Mandatory boolean self-verify guard (discard and fall through)

The engine SHALL accept a native boolean result as native ONLY when it PASSES a mandatory
self-verify: the candidate SHALL be (a) a **closed watertight 2-manifold** (closed at every
deflection in the mesher's deflection ladder, positive enclosed volume) AND (b) have the
**correct set-algebra volume sign and magnitude** for the op — the candidate volume `Vr` SHALL
satisfy `Vr ≈ |A| + |B| − |A ∩ B|` (fuse), `Vr ≈ |A| − |A ∩ B|` (cut), or `Vr ≈ |A ∩ B|`
(common) within a relative tolerance, using the operands' native volumes and their native
intersection volume. If EITHER check fails, the engine SHALL **DISCARD** the native result and
fall through to OCCT. The engine SHALL NEVER emit an unverified, leaky, or wrong boolean solid.

#### Scenario: A bad native boolean result is discarded and the call falls through (host)
- GIVEN a native boolean candidate that is open / non-manifold OR has a wrong set-algebra volume for its op, built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND `NativeEngine` SHALL fall through to the fallback engine for that call (no leaky or wrong solid is emitted)

#### Scenario: A verified native boolean is read back by the native paths (host)
- GIVEN a native boolean result that PASSES the self-verify (watertight 2-manifold with the correct set-algebra volume)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no fallback call

### Requirement: Curved-face, near-degenerate, and non-native boolean cases fall through to OCCT

The native boolean builder SHALL DECLINE (return a NULL `Shape`) — and `NativeEngine` SHALL
fall through to OCCT — for any case outside the analytic-planar slice: (1) either operand has
a **curved face** (any `FaceSurface::kind != Plane` — cylinder / sphere / cone / NURBS);
(2) a **near-tangent / coincident / degenerate** configuration (coplanar overlapping faces
within tolerance, a coincident plane pair, a fragment centroid near-ON the other solid,
touching-only measure-zero contact, or a self-intersecting / open / zero-volume operand);
(3) either operand is **not a native body** (a foreign / OCCT-built shape id). Each such case
SHALL produce EXACTLY the fallback (OCCT) engine's result. The change SHALL NOT fake,
stub-out, or partially implement any deferred case; each SHALL be labelled and verified as a
fall-through, never faked.

#### Scenario: A curved-face operand falls through (host + parity)
- GIVEN a boolean where at least one operand has a curved face (e.g. a cylinder unioned with a box), with the native engine active (`cc_set_engine(1)`)
- WHEN `cc_boolean` is invoked
- THEN the native builder SHALL return a NULL `Shape` AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A near-tangent / coincident configuration falls through (host)
- GIVEN two planar solids in a near-tangent / coincident configuration (coplanar overlapping faces, or touching-only contact), with the native engine active
- WHEN `cc_boolean` is invoked
- THEN the native builder SHALL return a NULL `Shape` (rather than emit a wrong classification) AND `NativeEngine` SHALL fall through to OCCT for that call

#### Scenario: A non-native (foreign) operand falls through (host)
- GIVEN a `cc_boolean` where at least one operand is not a native body (a foreign / OCCT-built shape id), with the native engine active
- WHEN `cc_boolean` is invoked
- THEN `NativeEngine` SHALL fall through to the fallback engine for that call (a native boolean requires the native B-rep of both operands), identical to `cc_set_engine(0)`

### Requirement: NativeEngine computes the boolean natively, else falls through

`NativeEngine` SHALL override `boolean_op` to run the native builder in `src/native/boolean/`
when BOTH operands are native bodies and apply the mandatory self-verify guard, type-erasing a
verified native `topology::Shape` into a tracked native `EngineShape`. When either operand is
not native, when the native builder returns a NULL `Shape` (a curved / coincident / degenerate
DECLINE), or when the self-verify guard rejects the candidate, the override SHALL fall through
to the held fallback engine with **no native interception**, producing exactly the fallback's
result. OCCT SHALL be referenced ONLY under `CYBERCAD_HAS_OCCT` (in the fallback wiring); the
native builder SHALL reference no OCCT / `IEngine` / `EngineShape` type. No `cc_*` signature or
POD layout SHALL change and the default engine SHALL remain OCCT (opt-in via `cc_set_engine(1)`).

#### Scenario: A supported planar boolean is built natively and read back natively
- GIVEN a `NativeEngine` active (`cc_set_engine(1)`) and two native planar-faced solids in a supported (non-coincident) configuration
- WHEN `cc_boolean(a, b, op)` is invoked for any op (fuse / cut / common)
- THEN the shape SHALL be built by `src/native/boolean/` and PASS the self-verify with no fallback call AND its mass properties / bbox / sub-shape counts / watertight tessellation SHALL be served by the native paths

#### Scenario: An unsupported or unverified boolean falls through under the native engine
- GIVEN the native engine active and an input hitting a deferred case (curved-face operand, near-tangent / coincident configuration, foreign operand) OR a candidate that fails the self-verify
- WHEN `cc_boolean` is invoked
- THEN the native builder SHALL return NULL (or the guard SHALL reject) AND the result SHALL be identical to invoking the same call with the OCCT engine active (`cc_set_engine(0)`), proving fall-through with no native interception

### Requirement: Boolean parity with OCCT through the facade (simulator gate)

The change SHALL be verified on a booted iOS simulator (OCCT linked): the SAME `cc_boolean`
calls SHALL be issued once with the native engine active (`cc_set_engine(1)`) and once with the
OCCT default (`cc_set_engine(0)`), and the two resulting shapes SHALL be compared through the
`cc_*` facade — mass properties, bounding box, sub-shape counts, and watertight tessellation —
against the OCCT `BRepAlgoAPI_Fuse` / `_Cut` / `_Common` oracle. On **axis-aligned box** fuse /
cut / common the native result SHALL match the oracle EXACTLY (volume / bbox / centroid relative
error ~0, fp precision), not merely within a deflection band. The fall-through cases (a
curved-face operand, a near-tangent / coincident configuration, and a foreign operand) SHALL be
asserted identical under both engines (fall-through proof, `cc_active_engine() == 1`). The
parity test SHALL restore the OCCT default in teardown and SHALL carry its own `main()` (on the
`run-sim-suite.sh` SKIP list) so the 221-assertion suite count is unchanged.

#### Scenario: Native box booleans match the OCCT BRepAlgoAPI oracle (parity)
- GIVEN axis-aligned box fuse / cut / common cases on a booted iOS simulator
- WHEN each `cc_boolean` op is called with the native engine active and with the OCCT default
- THEN the two shapes' mass properties, bounding box, sub-shape counts, and watertightness SHALL agree EXACTLY (relative error ~0) with the OCCT `BRepAlgoAPI` oracle

#### Scenario: Fall-through boolean cases are identical under both engines (parity)
- GIVEN a curved-face boolean, a near-tangent / coincident boolean, and a foreign-operand boolean on a booted iOS simulator
- WHEN each is called with the native engine active and with the OCCT default
- THEN the two results SHALL be identical (the native engine intercepts none of them)

#### Scenario: Existing suites stay green at the OCCT default
- GIVEN this change applied on an OCCT build with the engine left at its default
- WHEN `scripts/run-sim-suite.sh`, host CTest, and the GPU / Phase-3 suites are run
- THEN all SHALL stay green (`run-sim-suite.sh` 221/221) with no behavioural change from before this change


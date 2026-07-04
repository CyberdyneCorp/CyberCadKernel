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

### Requirement: SSI-curve-driven curved boolean for transversal elementary pairs

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse
(`A ∪ B`), `op = 1` cut (`A − B`), `op = 2` common (`A ∩ B`) — NATIVELY for
**transversal elementary curved pairs** (a pair where at least one operand has a
Cylinder / Sphere / Cone face, outside the analytic axis-aligned box∩cylinder family,
whose S3 intersection trace is fully transversal) by consuming the native, OCCT-free S3
`cybercad::native::ssi` `TraceSet`. The builder SHALL be a SIBLING path to the planar
BSP-CSG and the analytic `curved.h`, invoked through the same `boolean_solid` entry
behind the `cc_boolean` op codes, and SHALL:

- **Gate.** For each pair of intersecting curved faces, build the two `SurfaceAdapter`s
  and obtain the S3 `TraceSet`; PROCEED only when the trace is fully transversal
  (`nearTangentGaps == 0` and every consumed `WLine.status` is `Closed` or
  `BoundaryExit`). Otherwise return a NULL `Shape` (→ OCCT fallback).
- **Split.** Cut each curved face along its `WLine` using the WLine's per-node `(u,v)`
  track on that face as the split polyline in the face's UV domain (the curved analogue
  of the planar `splitPolygon`), partitioning the trimmed face into fragments; the shared
  **seam edge** SHALL take its 3D geometry from the WLine's fitted B-spline and its
  pcurve on each face from that face's `(u,v)` track, and each fragment SHALL retain its
  parent face's exact surface kind (Cylinder / Sphere / Cone / Plane) — nothing faceted.
- **Classify.** Tag each fragment INSIDE / OUTSIDE / ON the OTHER solid via a curved
  point-in-solid test at an interior UV sample of the fragment (the planar
  side-of-boundary classification idea generalized to curved faces), and select the
  surviving fragments per the op's face-survival rule — the SAME set algebra as the
  planar path (fuse: `A` outside `B` + `B` outside `A`; cut: `A` outside `B` + `B` inside
  `A` reversed; common: `A` inside `B` + `B` inside `A`), oriented outward. A sample that
  is robustly ON the other solid (coincident / tangent) SHALL abort the native path
  (NULL → OCCT), never a guessed side.
- **Weld.** Sew the surviving curved + planar fragments into one closed `Solid`, welding
  coincident corners to shared vertices and sharing the WLine seam edge between exactly
  the two fragments it split (one from each operand), so the two faces meet watertight
  along the curved seam (the tessellator's curved-seam weld).

The result SHALL be a native `topology::Shape` of type `Solid` carrying true curved face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
the exact set-algebra value for the op within a relative tolerance sized to the
curved-face tessellation deflection. The builder SHALL remain OCCT-free and reference no
OCCT / `IEngine` / `EngineShape` type, and — because it consumes the S3 tracer — its
SSI-driven entry point SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry
point, signature, or POD struct SHALL be added or changed.

#### Scenario: Equal-radius crossing cylinders common is the Steinmetz solid (host)
- GIVEN two equal-radius cylinders of radius `r` crossing at right angles, built as
  native curved solids on the host with no OCCT, and their S3 `TraceSet` (fully
  transversal)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated by
  `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (`boundaryEdgeCount == 0`) with every
  edge shared by exactly two faces AND its enclosed volume SHALL equal the analytic
  Steinmetz value `16 · r³ / 3` within the curved-face deflection tolerance

#### Scenario: A curved cut / fuse is watertight with the correct set-algebra volume (host)
- GIVEN a transversal elementary curved pair (e.g. sphere∩box or cone∩box) built as
  native curved solids on the host with no OCCT, and its fully-transversal S3 `TraceSet`
- WHEN the corresponding `cc_boolean` op (cut or fuse) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` with every seam node on
  BOTH surfaces within tolerance AND its enclosed volume SHALL have the correct
  set-algebra sign and magnitude for the op within the curved-face deflection tolerance

#### Scenario: The split is driven by the WLine, not by per-primitive matching (host)
- GIVEN two distinct transversal elementary curved pairs (e.g. cyl∩cyl and sphere∩box)
  and their S3 `TraceSet`s
- WHEN each pair's curved faces are split along their `WLine` `(u,v)` tracks
- THEN the SAME split → classify → weld driver SHALL handle both pairs by consuming their
  WLines, with no hand-matched per-primitive result builder for either pair

#### Scenario: A verified native curved boolean is read back by the native paths (host)
- GIVEN a native SSI-driven curved boolean result that PASSES the engine self-verify
  (watertight 2-manifold with the correct set-algebra volume)
- WHEN its mass properties, bounding box, sub-shape ids, and tessellation are queried
- THEN they SHALL be served by the native body-consuming paths with no fallback call

### Requirement: SSI curved boolean self-verify guard (discard and fall through)

The engine SHALL accept a native SSI-driven curved boolean result as native ONLY when it
PASSES the mandatory self-verify: the candidate SHALL be (a) a **closed watertight
2-manifold** (closed at every deflection in the mesher's deflection ladder, positive
enclosed volume, every edge shared by exactly two faces) AND (b) have the **correct
set-algebra volume sign and magnitude** for the op — `Vr ≈ |A| + |B| − |A ∩ B|` (fuse),
`Vr ≈ |A| − |A ∩ B|` (cut), or `Vr ≈ |A ∩ B|` (common) — within a RELATIVE tolerance
sized to the curved-face tessellation deflection (curved faces are deflection-bounded,
not fp-exact), using the operands' native volumes and their native intersection volume
(the Steinmetz `16 r³ / 3` for the equal-cyl common serves as the host analytic oracle).
If EITHER check fails, the engine SHALL **DISCARD** the native result. The engine SHALL
NEVER emit an unverified, leaky, or wrong SSI-driven curved boolean; when either operand
is OCCT-built the discarded case SHALL fall through to OCCT `BRepAlgoAPI`, and when both
operands are native voids OCCT cannot read, the engine SHALL report an honest error.
The self-verify guard SHALL live in the engine (next to the OCCT fallback), not in the
OCCT-free builder library.

#### Scenario: A bad native SSI curved boolean result is discarded (host)
- GIVEN a native SSI-driven curved boolean candidate that is open / non-manifold OR whose
  enclosed volume is outside the deflection-sized band for its op, built on the host
- WHEN the self-verify guard is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; a case with an OCCT operand
  falls through to OCCT)

#### Scenario: A verified equal-cyl common passes the Steinmetz analytic oracle (host)
- GIVEN a native equal-radius right-angle cylinder∩cylinder `common` candidate whose
  enclosed volume matches the analytic `16 · r³ / 3` within the deflection band, built on
  the host
- WHEN the self-verify guard is applied
- THEN the analytic-volume oracle SHALL accept the candidate AND it SHALL be served
  natively with no fallback call

### Requirement: Near-tangent, coincident, branch-point, and freeform curved pairs fall through to OCCT

The SSI-driven curved boolean builder SHALL DECLINE (return a NULL `Shape`) for any case
outside the transversal-elementary slice: (1) the S3 `TraceSet` for an intersecting face
pair reports **`nearTangentGaps > 0`**, or any consumed `WLine.status` is `NearTangent`
or `Failed` (the branch was traced only up to a tangent — the S4 seam); (2) a fragment's
interior sample lands **ON** the other solid within tolerance (a **coincident /
tangent-face** configuration — no robust inside/outside); (3) **coincident /
overlapping** curved faces (no discrete transversal seam to split); (4) a **branch-point
/ self-intersecting** seam (a singular multi-branch crossing); (5) a **freeform** (NURBS
/ Bézier) operand face (S5-a is elementary-only); (6) either operand is not a native
body. When either operand is an OCCT body, each such case SHALL produce EXACTLY the
fallback (OCCT `BRepAlgoAPI`) engine's result; when both operands are native voids OCCT
cannot read, the engine SHALL report an honest error. The change SHALL NOT fake,
stub-out, hand-tune, or partially implement any deferred case; each SHALL be labelled and
verified as a fall-through, never faked. `nearTangentGaps > 0` SHALL be treated as the
honest S4 fallback boundary, not consumed and not an error.

#### Scenario: A near-tangent curved pair declines to OCCT (host + parity)
- GIVEN a curved boolean whose S3 `TraceSet` reports `nearTangentGaps > 0` (e.g. tangent
  cylinders / a sphere tangent to a box face), with the native engine active
  (`cc_set_engine(1)`)
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` AND (with an OCCT operand) the
  result SHALL be identical to invoking the same call with the OCCT engine active
  (`cc_set_engine(0)`), proving fall-through with no native interception

#### Scenario: A coincident / branch-point curved pair declines (host)
- GIVEN a curved configuration with coincident / overlapping faces or a branch-point /
  self-intersecting seam (no single transversal seam per face), with the native engine
  active
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` (rather than emit a wrong
  classification) AND the engine SHALL NOT emit a native result for that call

#### Scenario: A freeform or non-native operand falls through (host)
- GIVEN a curved boolean where an operand has a NURBS / Bézier face, OR an operand is not
  a native body, with the native engine active
- WHEN `cc_boolean` is invoked
- THEN the SSI-driven builder SHALL return a NULL `Shape` AND fall through to the fallback
  engine for that call, identical to `cc_set_engine(0)`

### Requirement: SSI curved boolean parity with OCCT through the facade (simulator gate)

The SSI-driven curved boolean's fidelity SHALL be reported as a **measured native-vs-OCCT
parity** against `BRepAlgoAPI_{Fuse,Cut,Common}` on the simulator — **volume**,
**surface area**, **watertightness** (closed shell), and **shape validity** (`BRepCheck`)
— on transversal elementary pairs (cylinder∩cylinder including the Steinmetz common,
sphere∩box, cone∩box), rather than asserted to be a perfect result. The harness SHALL be
modelled on the S3 marching sim harness (`scripts/run-sim-native-ssi-marching.sh` +
`tests/sim/native_ssi_marching_parity.mm`), built against the SDK sysroot, linked with
the numsci IOSSIM substrate and the OCCT-for-iOS libs, and run via `xcrun simctl spawn`
on a booted simulator. The count of pairs deferred to OCCT (near-tangent / coincident /
freeform) SHALL be **reported** (the S4 seam), not hidden or padded, and whatever S5-a
cannot compute SHALL fall back to OCCT and be reported with the measured gap.

#### Scenario: native-vs-OCCT curved boolean parity is reported per pair on the simulator
- GIVEN a transversal elementary curved pair built both as native curved solids and as
  OCCT `BRepPrimAPI` solids
- WHEN the native SSI-driven boolean and OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` each compute
  the op on the simulator
- THEN the harness SHALL report the native vs OCCT volume delta, surface-area delta,
  watertight/closed-shell status, and shape validity within tolerance, compared at the
  `cybercad::native::boolean` C++ boundary
- AND no `cc_*` entry point SHALL have been added, and the count of pairs deferred to OCCT
  SHALL be reported, not hidden

#### Scenario: a deferred pair is reported as an OCCT fall-through, not faked
- GIVEN a curved pair S5-a cannot compute (near-tangent / coincident / freeform)
- WHEN parity is measured
- THEN the harness SHALL report it as deferred to OCCT (counted, with the measured gap)
  AND the native path SHALL have emitted no fabricated or hand-tuned result for it


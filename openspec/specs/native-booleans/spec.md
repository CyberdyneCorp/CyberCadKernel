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

### Requirement: SSI-driven native Fuse and Cut for the through-drill cylinder∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse
(`A ∪ B`), `op = 1` cut (`A − B`) — NATIVELY for the **through-drill
cylinder∩cylinder** topology the S5-a COMMON path (`buildCommon`) already recognises: two
transversal, fully-traced (`nearTangentGaps == 0`) closed rim seams, one operand
full-circle on both (the piercing tube), the other local on both (the pierced mouths). It
SHALL reuse the SAME two rim seams, the SAME shared `VertexPool` weld, and the SAME
planar-triangle facet discipline as the S5-a COMMON, and SHALL select the surviving
fragments per the op's face-survival rule — the SAME set algebra as the planar path:

- **Cut `A − B`** (`A` = the pierced cylinder, `B` = the piercing tube): the pierced wall
  and end caps OUTSIDE the tube (re-trimmed to exclude the drilled region) + the tube band
  INSIDE the pierced solid REVERSED as the inward tunnel wall; the two drill-mouth caps
  (removed material) SHALL be dropped.
- **Fuse `A ∪ B`**: the pierced wall OUTSIDE the tube + the tube wall OUTSIDE the pierced
  solid (each outside stretch with its own end-cap disc) + the operands' end caps; the two
  drill-mouth caps AND the inside tube band (now interior to the union) SHALL be dropped.

Every face that shares a rim seam with a differently tessellated neighbour SHALL be
emitted as PLANAR-TRIANGLE facets through the EXACT traced seam nodes drawn from the shared
`VertexPool`, so the shell welds watertight (the S5-a watertight discipline). Fragment
survival SHALL be decided by the S5-a curved point-in-solid test (`classifyPoint`) at an
interior sample; a sample that is robustly ON the other solid, an unrecognised
through-drill topology, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT).

The result SHALL be a native `topology::Shape` of type `Solid` carrying true curved face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
the exact set-algebra value for the op within a relative tolerance sized to the
curved-face tessellation deflection: `Vr ≈ vol(A) + vol(B) − vol(A ∩ B)` (fuse) or
`Vr ≈ vol(A) − vol(A ∩ B)` (cut), where `vol(A ∩ B)` is the S5-a through-drill COMMON. The
builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, and
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry point, signature, or POD
struct SHALL be added or changed, and the S5-a `buildCommon` COMMON path SHALL be
unchanged.

#### Scenario: The through-drill cut removes the tunnel with the correct volume (host)
- GIVEN a thin cylinder drilled clean through a fat one (the S5-a transversal through-drill
  fixture, `nearTangentGaps == 0`, two closed rim seams), built as native curved solids on
  the host with no OCCT
- WHEN `cc_boolean(fat, thin, 1)` (cut) is computed and tessellated by
  `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces)
  AND its enclosed volume SHALL equal `vol(fat) − vol(fat ∩ thin)` within the curved-face
  deflection tolerance (`vol(fat ∩ thin)` = the S5-a-pinned through-drill COMMON volume)

#### Scenario: The through-drill fuse welds both cylinders with the correct volume (host)
- GIVEN the same transversal through-drill pair built as native curved solids on the host
  with no OCCT
- WHEN `cc_boolean(fat, thin, 0)` (fuse) is computed and tessellated
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume
  SHALL equal `vol(fat) + vol(thin) − vol(fat ∩ thin)` within the curved-face deflection
  tolerance AND SHALL satisfy `fuse ≥ max(vol(fat), vol(thin))`

#### Scenario: Fuse and cut reuse the S5-a seam and weld, not a new trace (host)
- GIVEN the through-drill pair and its shipped S3 `TraceSet` (the SAME two rim seams the
  S5-a COMMON consumes)
- WHEN the native fuse and cut are assembled
- THEN they SHALL be built from the SAME two rim seams and the SAME shared-vertex
  planar-facet weld as the S5-a COMMON, with no new tracing and no hand-matched
  per-primitive result builder

### Requirement: SSI-driven native Common for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, 2)` (common, `A ∩ B`) NATIVELY
for a **transversal sphere∩sphere** pair via a NEW single-seam / two-cap assembler
(`buildLensCommon`), taken when the S3 `TraceSet` is ONE `Closed` seam
(`nearTangentGaps == 0`) and BOTH operands are recognised as `Sphere` curved solids. The
sphere∩sphere COMMON (lens) is bounded by TWO spherical caps — the cap of sphere A inside
sphere B and the cap of sphere B inside sphere A — meeting along the ONE seam circle. The
assembler SHALL:

- **Gate.** Require exactly one closed seam and both operands spheres; a seam that is a
  full sphere (a coincident / degenerate case) SHALL return NULL.
- **Cap survival (COMMON rule).** Take each cap's POLE (sphere A's surface point nearest
  sphere B's centre, and symmetrically), evaluated on the analytic sphere; keep the A-cap
  ONLY IF its pole is INSIDE sphere B and the B-cap ONLY IF its pole is INSIDE sphere A
  (the S5-a `classifyPoint` test). A pole robustly ON the other sphere (tangent /
  coincident) SHALL abort the native path → NULL → OCCT, never a guessed side.
- **Cap weld.** Emit each cap with the SAME radial-ring planar-facet discipline as the
  S5-a drill-mouth cap (fan from the pole out through concentric rings to the OUTER ring =
  the EXACT traced seam nodes drawn from the shared `VertexPool`, every ring node on the
  analytic sphere, facet normals oriented outward). Because BOTH caps' outer rings are the
  SAME pooled seam vertices, the two caps SHALL weld watertight along the single seam.

The result SHALL be a native `topology::Shape` of type `Solid`, watertight (every edge
shared by exactly two faces), with every seam node on BOTH spheres within tolerance, whose
enclosed volume equals the closed-form lens volume within the curved-face deflection
tolerance:
`V = π (rA + rB − d)² (d² + 2 d·rB − 3 rB² + 2 d·rA + 6 rA·rB − 3 rA²) / (12 d)`
for centre distance `d` (equal radii `r`: two caps of height `h = r − d/2`, cap volume
`π h² (3r − h) / 3`, lens `= 2 ×`). The assembler SHALL remain OCCT-free and be compiled
under `CYBERCAD_HAS_NUMSCI`; no `cc_*` entry point SHALL be added or changed, and the S5-a
two-seam `buildCommon` path SHALL be unchanged.

#### Scenario: Equal-radius sphere∩sphere common matches the closed-form lens (host)
- GIVEN two equal-radius spheres (radius `r`, centre distance `0 < d < 2r`) built as native
  curved solids on the host with no OCCT, and their S3 `TraceSet` (one `Closed` seam,
  `nearTangentGaps == 0`)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces)
  with every seam node on both spheres within tolerance AND its enclosed volume SHALL
  equal the lens `2 · π h² (3r − h) / 3`, `h = r − d/2`, within the curved-face deflection
  tolerance

#### Scenario: Unequal-radius sphere∩sphere common matches the closed-form lens (host)
- GIVEN two spheres of distinct radii `rA ≠ rB` with `|rA − rB| < d < rA + rB`, built as
  native curved solids on the host, and their one-seam S3 `TraceSet`
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated
- THEN the result SHALL be a watertight `Solid` whose enclosed volume equals the asymmetric
  lens closed form within the curved-face deflection tolerance

#### Scenario: A single-seam sphere pair is handled by the two-cap path, not the tube path (host)
- GIVEN a transversal sphere∩sphere pair (one closed seam) and a through-drill
  cylinder∩cylinder pair (two rim seams)
- WHEN each is dispatched
- THEN the sphere pair SHALL be assembled by the single-seam two-cap `buildLensCommon` and
  the cylinder pair by the two-seam `buildCommon`, each by its own topology gate, with no
  cross-contamination and no hand-matched per-primitive builder

### Requirement: The wider SSI curved booleans are guarded by the existing engine self-verify

The engine SHALL accept a native S5-b fuse / cut or S5-c sphere∩sphere common result as
native ONLY when it PASSES the EXISTING mandatory self-verify
(`native_engine.cpp booleanResultVerified`): a closed watertight 2-manifold with the
correct set-algebra volume — `Vr ≈ va + vb − vc` (fuse), `Vr ≈ va − vc` (cut), or
`Vr ≈ vc` (common) — within a relative tolerance sized to the curved-face tessellation
deflection, where `vc` is the native COMMON volume (the through-drill COMMON for the
cylinder pair; the lens for the sphere pair). NO new engine oracle SHALL be added: the
generic set-algebra guard already computes these, and the `ssiCurvedBooleanVerified`
Steinmetz special oracle (equal-radius perpendicular cylinders) SHALL remain untouched and
SHALL NOT fire for these cases. If the self-verify fails, the engine SHALL DISCARD the
native result and fall through to OCCT `BRepAlgoAPI` (OCCT operand) or report an honest
error (both operands native voids). The engine SHALL NEVER emit an unverified, leaky, or
wrong wider SSI curved boolean.

#### Scenario: A bad wider SSI curved boolean candidate is discarded (host)
- GIVEN a native S5-b fuse/cut or S5-c sphere-common candidate that is open / non-manifold
  OR whose enclosed volume is outside the deflection-sized band for its op, built on the
  host
- WHEN the existing generic set-algebra self-verify guard is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; an OCCT-operand case falls
  through to OCCT)

#### Scenario: A verified fuse / cut / lens-common passes the existing guard (host)
- GIVEN a native through-drill fuse or cut whose watertight volume matches
  `va + vb − vc` / `va − vc`, OR a native sphere∩sphere common whose volume matches the
  closed-form lens, all within the deflection band
- WHEN the existing generic set-algebra guard is applied
- THEN the guard SHALL accept the candidate AND it SHALL be served natively with no OCCT
  fallback call AND the Steinmetz special oracle SHALL NOT have fired

### Requirement: Out-of-envelope wider curved pairs fall through to OCCT

The wider SSI curved boolean builders SHALL DECLINE (return a NULL `Shape`) for any case
outside the two shipped slices: (1) **sphere∩sphere fuse / cut** (S5-c ships COMMON only);
(2) **tangent / coincident spheres** (`nearTangentGaps > 0`, or a cap pole robustly ON the
other sphere); (3) **equal-radius orthogonal cylinder∩cylinder** (the Steinmetz pair —
`nearTangentGaps > 0`, an S4 case); (4) **oblique / multi-tube cylinder∩cylinder**
piercings whose seams are not two clean full-circle rims; (5) **other curved-curved
families** (cylinder∩cone, cylinder∩sphere, cone∩cone, sphere∩box, freeform). When either
operand is an OCCT body, each such case SHALL produce EXACTLY the OCCT `BRepAlgoAPI`
fallback result; when both operands are native voids OCCT cannot read, the engine SHALL
report an honest error. The change SHALL NOT fake, stub-out, hand-tune, or partially
implement any deferred case; `nearTangentGaps > 0` SHALL remain the honest S4 fallback
boundary, not consumed and not an error.

#### Scenario: A sphere∩sphere fuse or cut declines to OCCT (host)
- GIVEN a transversal sphere∩sphere pair with the native engine active
- WHEN `cc_boolean(A, B, 0)` (fuse) or `cc_boolean(A, B, 1)` (cut) is invoked
- THEN the wider builder SHALL return a NULL `Shape` (S5-c ships COMMON only) AND (with an
  OCCT operand) the result SHALL be identical to invoking the same call with the OCCT
  engine active, proving fall-through with no native interception

#### Scenario: Tangent spheres and the Steinmetz cylinder pair decline (host)
- GIVEN two tangent spheres (`d = rA + rB`) OR two equal-radius perpendicular cylinders
  (the Steinmetz pair), with the native engine active
- WHEN `cc_boolean` (common) is invoked
- THEN the wider builder SHALL return a NULL `Shape` (near-tangent → S4) AND the engine
  SHALL NOT emit a native result for that call

### Requirement: Wider SSI curved boolean parity with OCCT through the facade (simulator gate)

The wider SSI curved booleans' fidelity SHALL be reported as a MEASURED native-vs-OCCT
parity against `BRepAlgoAPI_{Fuse,Cut,Common}` on the simulator — volume, surface area,
watertightness (closed shell), and shape validity (`BRepCheck`) — on the through-drill
cylinder∩cylinder **fuse** and **cut** and the transversal sphere∩sphere **common** (built
as OCCT `BRepPrimAPI_MakeSphere` / cylinder solids), extending the S5-a harness
(`scripts/run-sim-native-ssi-curved-boolean.sh` +
`tests/sim/native_ssi_curved_boolean_parity.mm`) rather than adding a new harness. The
count of pairs still deferred to OCCT (sphere fuse/cut, near-tangent, out-of-family) SHALL
be REPORTED (the S4 / follow-on seam), not hidden or padded, and whatever the wider slices
cannot compute SHALL fall back to OCCT and be reported with the measured gap.

#### Scenario: native-vs-OCCT parity is reported per wider pair on the simulator
- GIVEN the through-drill fuse/cut pair and the transversal sphere∩sphere common pair,
  each built both as native curved solids and as OCCT `BRepPrimAPI` solids
- WHEN the native wider boolean and OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` each compute the
  op on the simulator
- THEN the harness SHALL report the native-vs-OCCT volume delta, surface-area delta,
  watertight/closed-shell status, and shape validity within tolerance, compared at the
  `cybercad::native::boolean` C++ boundary
- AND no `cc_*` entry point SHALL have been added, and the count of pairs deferred to OCCT
  SHALL be reported, not hidden

### Requirement: SSI-driven native Common for the Steinmetz-family branched pair

The native boolean library SHALL compute `cc_boolean(a, b, 2)` (common, `A ∩ B`) NATIVELY for
the **Steinmetz family** — two `Cylinder` curved solids of equal radius (`|rA − rB|` within
tolerance) whose axes are orthogonal (`|â · b̂|` within tolerance) and cross (the axis lines
meet within tolerance) — via a NEW branched-trace assembler (`buildSteinmetzCommon`). The
assembler SHALL consume the S4-d branched `ssi::TraceSet` obtained with
`MarchOptions.enableBranchPoints = true`, and SHALL recognise the Steinmetz structure ONLY
when the trace is fully resolved: `nearTangentGaps == 0`, `branchPoints == 2` with
`branchNodes.size() == 2`, EXACTLY FOUR WLines all of `status == BranchArc`, every arm on both
cylinders within `onSurfTol`, and the two branch nodes connecting all four arms (each arm's two
endpoints coincide with the two branch-node points). The assembler SHALL:

- **Split.** On each cylinder the two arms lying on that cylinder bound the region of its wall
  INSIDE the other cylinder; SPLIT the wall along its two arcs into the candidate lune patches,
  each emitted as a strip of PLANAR triangles between its two arcs (walked branch-to-branch in
  lockstep), every interior sample placed ON the analytic cylinder and its `(u,v)` folded
  contiguous around the patch centroid so no ±2π wrap corrupts it.
- **Select (COMMON rule).** KEEP each lune patch ONLY IF its centroid sample is INSIDE the
  OTHER cylinder (`classifyPoint(other, centroid) == inside`) — the four inside∩inside patches
  ARE the bicylinder boundary. A centroid robustly ON the other cylinder SHALL abort the native
  path → NULL → OCCT, never a guessed side.
- **Weld.** Emit every seam-adjacent patch as PLANAR-TRIANGLE facets through the EXACT traced
  arc nodes drawn from ONE shared `VertexPool`, with the two branch-point vertices pooled ONCE
  so all four arcs meet there with no crack; both sides of every shared arc draw the SAME
  pooled vertices, so the four lune patches weld watertight along the four arcs and the two
  branch points (the S5-a planar-facet weld discipline). Facet normals SHALL be oriented
  outward (the cylinder's outward radial).

The result SHALL be a native `topology::Shape` of type `Solid`, watertight (every edge shared
by exactly two faces), with every arc node on BOTH cylinders within tolerance and the two
branch-point vertices a single shared node, whose enclosed volume equals the EXACT Steinmetz
bicylinder value `16 R³ / 3` within the curved-face tessellation deflection tolerance. The
assembler SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, and
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`. No `cc_*` entry point, signature, or POD struct
SHALL be added or changed, and the single-seam S5-a/b/c paths SHALL be unchanged.

#### Scenario: The Steinmetz common matches the exact analytic bicylinder volume (host)

- GIVEN two equal-radius cylinders (radius `R`) whose axes cross orthogonally at the origin,
  built as native curved solids on the host with no OCCT, and their S4-d branched `TraceSet`
  (obtained with `enableBranchPoints = true`: `branchPoints == 2`, four `BranchArc` arms,
  `nearTangentGaps == 0`)
- WHEN `cc_boolean(A, B, 2)` (common) is computed and tessellated by `src/native/tessellate`
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces) with
  every arc node on both cylinders within tolerance and the two branch-point vertices a single
  shared node AND its enclosed volume SHALL equal the exact bicylinder `16 R³ / 3` within the
  curved-face deflection tolerance

#### Scenario: The four lune patches weld at the shared arcs and the two branch points (host)

- GIVEN the Steinmetz pair and its S4-d branched `TraceSet` (four `BranchArc` arms meeting at
  two branch nodes)
- WHEN `buildSteinmetzCommon` splits each cylinder wall along its arcs, selects the four
  inside-the-other lune patches, and welds them
- THEN the four lune patches SHALL be assembled from the SAME traced arc nodes drawn from ONE
  shared `VertexPool` with the two branch-point vertices pooled ONCE, so all four arcs weld at
  the two branch points and both sides of every arc weld along the arc, with no new tracing and
  no hand-matched per-primitive result builder

### Requirement: SSI-driven native Fuse and Cut for the Steinmetz-family branched pair

The native boolean library SHALL compute `cc_boolean(a, b, op)` — `op = 0` fuse (`A ∪ B`),
`op = 1` cut (`A − B`) — NATIVELY for the Steinmetz family (the SAME recognised branched trace
the COMMON path consumes) WHEN it can assemble a watertight, correct-volume shell, reusing the
SAME four arcs, the SAME shared `VertexPool` weld (arcs + the two branch-point vertices), and
the SAME planar-triangle facet discipline as the COMMON, and selecting the surviving fragments
per the op's face-survival rule — the SAME set algebra as the planar path:

- **Cut `A − B`**: A's wall OUTSIDE B + A's two end caps + B's inside-A lune patches REVERSED
  (the tunnel wall); A's inside-B lune patches SHALL be dropped. The shared arcs weld the
  reversed B patches to A's outside wall.
- **Fuse `A ∪ B`**: each cylinder's OUTSIDE-the-other wall + both cylinders' end caps; both
  cylinders' inside-the-other lune patches SHALL be dropped (now interior to the union). The
  shared arcs weld the two outer walls.

Fragment survival SHALL be decided by the S5-a curved point-in-solid test (`classifyPoint`) at
an interior sample; a sample robustly ON the other solid, a non-Steinmetz branched trace, or a
weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The result SHALL be a native
`topology::Shape` of type `Solid`, watertight (every edge shared by exactly two faces), whose
enclosed volume equals the exact set-algebra value for the op within a relative tolerance sized
to the curved-face tessellation deflection: `Vr ≈ vol(A) + vol(B) − vol(A ∩ B)` (fuse) or
`Vr ≈ vol(A) − vol(A ∩ B)` (cut), where `vol(A ∩ B)` is the native Steinmetz COMMON. A builder
that cannot robustly assemble a watertight, correct-volume shell SHALL return NULL (→ OCCT),
reported — the COMMON is the guaranteed slice. The builder SHALL remain OCCT-free and be
compiled under `CYBERCAD_HAS_NUMSCI`; no `cc_*` entry point SHALL be added or changed, and the
single-seam S5-a/b/c paths SHALL be unchanged.

#### Scenario: The Steinmetz cut removes the bicylinder with the correct volume (host)

- GIVEN the equal-R orthogonal Steinmetz pair built as native curved solids on the host with no
  OCCT, and its S4-d branched `TraceSet`
- WHEN `cc_boolean(A, B, 1)` (cut) is computed and tessellated and the builder assembles a
  watertight shell
- THEN the result SHALL be a watertight `Solid` (every edge shared by exactly two faces) AND its
  enclosed volume SHALL equal `vol(A) − 16 R³ / 3` within the curved-face deflection tolerance;
  a builder that cannot assemble a watertight, correct-volume shell SHALL return NULL (→ OCCT)

#### Scenario: The Steinmetz fuse welds both cylinders with the correct volume (host)

- GIVEN the same equal-R orthogonal Steinmetz pair built as native curved solids on the host
- WHEN `cc_boolean(A, B, 0)` (fuse) is computed and tessellated and the builder assembles a
  watertight shell
- THEN the result SHALL be a watertight closed 2-manifold `Solid` AND its enclosed volume SHALL
  equal `vol(A) + vol(B) − 16 R³ / 3` within the curved-face deflection tolerance AND SHALL
  satisfy `fuse ≥ max(vol(A), vol(B))`; a builder that cannot assemble a watertight,
  correct-volume shell SHALL return NULL (→ OCCT)

### Requirement: The Steinmetz branched curved boolean is guarded by the existing engine self-verify

The engine SHALL accept a native S5-d Steinmetz COMMON / FUSE / CUT result as native ONLY when
it PASSES the EXISTING mandatory self-verify: a closed watertight 2-manifold with the correct
volume. The Steinmetz COMMON SHALL be verified by the engine's EXISTING
`ssiCurvedBooleanVerified` Steinmetz oracle (op == common, equal-radius orthogonal cylinders —
the `16 R³ / 3` closed form), which previously always found a NULL native candidate and fell to
OCCT and now verifies the branched native candidate's watertight volume against `16 R³ / 3`.
The Steinmetz FUSE / CUT SHALL be verified by the EXISTING generic set-algebra guard
(`Vr ≈ va + vb − vc` / `va − vc`, `vc` = the native Steinmetz COMMON volume). NO new engine
oracle SHALL be added, and the single-seam S5-a/b/c guards SHALL remain untouched and SHALL NOT
fire for the branched Steinmetz case. If the self-verify fails, the engine SHALL DISCARD the
native result and fall through to OCCT `BRepAlgoAPI` (OCCT operand) or report an honest error
(both operands native voids). The engine SHALL NEVER emit an unverified, leaky, or wrong
Steinmetz curved boolean.

#### Scenario: A bad Steinmetz branched candidate is discarded (host)

- GIVEN a native S5-d Steinmetz COMMON / FUSE / CUT candidate that is open / non-manifold OR
  whose enclosed volume is outside the deflection-sized band for its op, built on the host
- WHEN the existing engine self-verify (the Steinmetz `16 R³/3` oracle for common; the generic
  set-algebra guard for fuse/cut) is applied
- THEN the guard SHALL reject the candidate AND the engine SHALL NOT emit a leaky or wrong
  curved solid (a native-native case reports an honest error; an OCCT-operand case falls through
  to OCCT)

#### Scenario: A verified Steinmetz common passes the existing oracle natively (host)

- GIVEN a native Steinmetz COMMON whose watertight volume matches the exact `16 R³ / 3` within
  the deflection band
- WHEN the existing `ssiCurvedBooleanVerified` Steinmetz oracle is applied
- THEN the oracle SHALL accept the candidate AND it SHALL be served natively with no OCCT
  fallback call AND no new engine oracle SHALL have been added

### Requirement: Out-of-family branched curved pairs fall through to OCCT

The S5-d branched assembler SHALL DECLINE (return a NULL `Shape`) for any branched curved pair
outside the recognised Steinmetz family: (1) **unequal-radius, non-orthogonal, or non-crossing
cylinder pairs** (the Steinmetz pre-gate rejects); (2) **cylinder∩sphere, cylinder∩cone,
cone∩cone, or freeform self-crossings**; (3) branched traces with `nearTangentGaps > 0` (an arm
the S4-d marcher could not resolve), `branchPoints != 2`, or a WLine count / status that is not
exactly four `BranchArc` arms. The branched re-trace SHALL be entered ONLY when the DEFAULT
(unbranched) trace declined AND the Steinmetz pre-gate matches, so no single-seam S5-a/b/c pass
re-traces or changes its result. When either operand is an OCCT body, each declined case SHALL
produce EXACTLY the OCCT `BRepAlgoAPI` fallback result; when both operands are native voids OCCT
cannot read, the engine SHALL report an honest error. The change SHALL NOT fake, stub-out,
hand-tune, or partially implement any deferred case; a branched trace that is not the exact
resolved Steinmetz structure SHALL remain the honest fallback boundary, not an error.

#### Scenario: An unequal-radius or non-orthogonal branched cylinder pair declines to OCCT (host)

- GIVEN two cylinders whose axes cross but with UNEQUAL radii, OR two equal-radius cylinders
  whose axes are NOT orthogonal, with the native engine active
- WHEN `cc_boolean(A, B, op)` is invoked for any op
- THEN the S5-d branched assembler SHALL return a NULL `Shape` (the Steinmetz pre-gate / the
  recognition gate rejects) AND (with an OCCT operand) the result SHALL be identical to invoking
  the same call with the OCCT engine active, proving fall-through with no native interception

#### Scenario: A single-seam S5-a/b/c pair never enters the branched re-trace (host)

- GIVEN a through-drill cylinder∩cylinder pair (unequal radii, two rim seams) and a transversal
  sphere∩sphere pair (one closed seam), each with the native engine active
- WHEN each is dispatched
- THEN each SHALL be handled by its single-seam S5-a/b/c builder on the DEFAULT trace with no
  branched re-trace, and the S5-d branched path SHALL NOT fire (its pre-gate requires equal-R
  orthogonal cylinders), with no cross-contamination and no hand-matched per-primitive builder

### Requirement: SSI-driven native Fuse for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY
for the transversal sphere∩sphere pair the S5-c COMMON path (`buildLensCommon`) already
recognises: both operands recognised as `Sphere` solids, and the S3 trace a SINGLE closed
seam circle with `nearTangentGaps == 0`. It SHALL reuse the SAME decimated shared seam
(`decimateSeam(seam, seamNodeTarget(seam))`), the SAME shared `VertexPool` weld, and the
SAME radial-ring planar-triangle cap discipline as `buildLensCommon`, and SHALL assemble
the union boundary as the two **OUTER** spherical caps — the cap of A whose apex is A's far
pole (`cA − RA·unit(cA→cB)`) and the cap of B whose apex is B's far pole
(`cB + RB·unit(cA→cB)`) — each oriented with the sphere OUTWARD radial normal, welded along
the ONE shared seam.

The builder SHALL keep each outer cap only if its far-pole apex classifies strictly OUTSIDE
the other solid (the transversal-lens fuse survival rule, via the S5-a curved
point-in-solid test `classifyPoint`); a far pole robustly INSIDE the other sphere
(containment) or ON it (tangent), a non-sphere or multi-seam input, or a weld that cannot
close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a
pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Sphere` face
kinds, watertight (every edge shared by exactly two faces), whose enclosed volume equals
`vol(A) + vol(B) − vol(A ∩ B)` within a relative tolerance sized to the curved-face
tessellation deflection, where `vol(A ∩ B)` is the native lens COMMON (`buildLensCommon`).
The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type,
SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry
point, signature, or POD struct.

#### Scenario: The sphere∩sphere fuse grows to the peanut with the correct volume (host)
- GIVEN two overlapping spheres A (radius `RA`) and B (radius `RB`) whose centres are a
  distance `d` apart with `|RA − RB| < d < RA + RB` (a transversal lens), traced as ONE
  closed seam circle with `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse` declines the
  single seam, `buildLensFuse` takes over)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by
  exactly two faces) bounded by the two OUTER spherical caps sharing the ONE seam
- AND its enclosed volume equals `4/3·π(RA³ + RB³) − lens` (with `lens = V_cap(A) +
  V_cap(B)`, `V_cap = π h² (3R − h)/3`) within the deflection-sized band — a GROW
  (`Vr > max(vol(A), vol(B))`)
- AND every seam node lies on BOTH sphere surfaces within tolerance.

#### Scenario: A far pole inside the other sphere declines to OCCT (host)
- GIVEN two spheres where one is largely contained in the other (a far pole classifies
  INSIDE the other solid), or tangent (a far pole ON the other surface)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `buildLensFuse` returns a NULL `Shape` (the fuse survival rule needs both far poles
  strictly outside) and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) −
  vol(A ∩ B)` (a mis-welded seam or wrong cap selection)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` =
  native lens COMMON)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT;
  the engine never emits an unverified sphere fuse.

### Requirement: SSI-driven native Cut for the transversal sphere∩sphere lens

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the
minuend) NATIVELY for the transversal sphere∩sphere pair the S5-c COMMON path already
recognises (both `Sphere`, one closed seam, `nearTangentGaps == 0`). It SHALL reuse the
SAME decimated shared seam, `VertexPool`, and radial-ring planar-triangle cap discipline as
`buildLensCommon`, and SHALL assemble the difference boundary as the **OUTER** cap of A
(apex = A's far pole, outward radial normal) plus the **INNER** cap of B (apex nearest A)
emitted **REVERSED** (inward radial normal) so it bounds the scooped cavity, both welded
along the ONE shared seam. Operand order SHALL be honoured (CUT is not symmetric),
matching `BRepAlgoAPI_Cut(a, b)`.

The builder SHALL proceed only if A's far-pole apex classifies OUTSIDE B AND B's inner apex
classifies INSIDE A (the transversal-lens cut survival rule, via `classifyPoint`); a
tangent / degenerate / wrong-side apex, a non-sphere or multi-seam input, or a weld that
cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Sphere` face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals
`vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where `vol(A ∩ B)` is
the native lens COMMON. The builder SHALL remain OCCT-free, reference no OCCT / `IEngine` /
`EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*`
entry point, signature, or POD struct.

#### Scenario: The sphere∩sphere cut scoops the lens with the correct volume (host)
- GIVEN two overlapping spheres A (radius `RA`, minuend) and B (radius `RB`), transversal
  lens, ONE closed seam, `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut` declines the
  single seam, `buildLensCut` takes over)
- THEN it returns a watertight `Solid` bounded by A's OUTER cap (outward) plus B's INNER cap
  REVERSED (inward, bounding the cavity), sharing the ONE seam
- AND its enclosed volume equals `4/3·π·RA³ − lens` within the deflection-sized band — a
  SHRINK (`Vr < vol(A)`)
- AND every seam node lies on both sphere surfaces within tolerance.

#### Scenario: The reversed inner cap bounds the cavity, verified against the native lens (host)
- GIVEN the sphere∩sphere cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` =
  native lens COMMON `buildLensCommon`)
- THEN a candidate whose reversed INNER-B cap is mis-oriented (outward, not bounding the
  cavity) yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the
  correct candidate matches `vol(A) − lens` and is accepted native.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME overlapping sphere pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the generalisation of
  `appendSphereCap` (defaulted `outer=false, reversed=false`)
- THEN `buildLensCommon` produces the byte-identical two-inner-cap lens (same volume, area,
  and vertices as before this change) — the COMMON native pass does not regress.

### Requirement: SSI-driven native Fuse for the Steinmetz bicylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for
the Steinmetz bicylinder pair the S5-d COMMON path (`buildSteinmetzCommon`) already recognises:
both operands recognised as `Cylinder` solids of near-equal radius whose axes cross orthogonally
(`steinmetzPreGate`), and the branch-enabled S4-d re-trace a resolved branched `TraceSet` with
`branchPoints == 2`, exactly four `BranchArc` arms on both cylinders, and `nearTangentGaps == 0`
(`recogniseSteinmetzTrace`). It SHALL reuse the SAME oriented + pole-axis-resampled four arcs
(`orientArc`, `resampleArcByAxis`), the SAME shared `VertexPool` weld with the two poles pooled
ONCE, and the SAME radial-ring planar-triangle lune discipline (`appendLunePatch`) as
`buildSteinmetzCommon`, and SHALL assemble the union boundary as the two OUTSIDE lune patches of
EACH cylinder (the wall regions outside the other cylinder, oriented with the cylinder OUTWARD
radial normal) PLUS both cylinders' two original disc end caps (full circles at `v = vLo` and
`v = vHi`, oriented along the outward axial direction), all sharing the four arcs and two poles.

The builder SHALL keep each outside lune only if its centroid classifies strictly OUTSIDE the
other solid (the fuse survival rule, via the S5-a curved point-in-solid test `classifyPoint`); a
centroid robustly ON the other wall (tangent), a short cylinder whose end-cap plane falls within
the intersection seam band, a non-Steinmetz or unresolved branched input, or a weld that cannot
close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cylinder` (wall)
and planar (cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed
volume equals `vol(A) + vol(B) − vol(A ∩ B)` within a relative tolerance sized to the curved-face
tessellation deflection, where `vol(A ∩ B)` is the native bicylinder COMMON (`buildSteinmetzCommon`
= `16 R³/3`). The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` / `EngineShape`
type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry
point, signature, or POD struct.

#### Scenario: The Steinmetz fuse grows to the outer envelope with the correct volume (host)
- GIVEN two equal-radius cylinders A (radius `R`, length `L_A`) and B (radius `R`, length `L_B`)
  whose axes cross orthogonally, whose branch-enabled S4-d `TraceSet` has `branchPoints == 2`,
  four `BranchArc` arms, and `nearTangentGaps == 0`, and whose end caps lie outside the `|y| ≤ R`
  intersection band
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the default trace declines near-tangent, the
  branch-enabled re-trace resolves the four arcs, and `buildSteinmetzFuse` assembles the outer
  shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly
  two faces) bounded by both cylinders' two OUTSIDE lune walls sharing the four arcs, closed by
  all four original disc end caps
- AND its enclosed volume equals `π R²(L_A + L_B) − 16 R³/3` within the deflection-sized band —
  a GROW (`Vr > max(vol(A), vol(B))`)
- AND every arc node lies on BOTH cylinder surfaces within tolerance, and the two branch-point
  poles are pooled ONCE (shared by all surviving lune fragments).

#### Scenario: A short cylinder whose caps clip the seam declines to OCCT (host)
- GIVEN a Steinmetz pair one of whose cylinders is so short that an end-cap plane falls within
  the `|y| ≤ R` intersection seam band (the disjoint-cap assumption fails)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `appendCylinderEndCaps` reports the clip and `buildSteinmetzFuse` returns a NULL `Shape`,
  and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not faked, tolerance not
  weakened.

#### Scenario: The engine discards a wrong-volume Steinmetz fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match
  `vol(A) + vol(B) − vol(A ∩ B)` (a mis-selected lune or a mis-placed cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` =
  native bicylinder COMMON `buildSteinmetzCommon = 16 R³/3`; the `op == 2`-only analytic
  Steinmetz oracle does NOT intercept fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified Steinmetz fuse.

### Requirement: SSI-driven native Cut for the Steinmetz bicylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the minuend)
NATIVELY for the Steinmetz bicylinder pair the S5-d COMMON path already recognises (both near-
equal-radius `Cylinder`, orthogonal crossing axes, a resolved 2-branch-point / four-`BranchArc`
`TraceSet` with `nearTangentGaps == 0`). It SHALL reuse the SAME oriented + pole-axis-resampled
four arcs, `VertexPool` (poles pooled once), and radial-ring planar-triangle lune discipline as
`buildSteinmetzCommon`, and SHALL assemble the difference boundary as A's two OUTSIDE lune walls
(outward radial normal) plus A's two original disc end caps, plus B's two INSIDE lune patches
emitted REVERSED (inward radial normal, `outwardSign = −1`) so they bound the carved channel
through A, all sharing the four arcs and two poles. Operand order SHALL be honoured (CUT is not
symmetric), matching `BRepAlgoAPI_Cut(a, b)`.

The builder SHALL proceed only if A's outside-lune centroids classify OUTSIDE B AND B's inside-
lune centroids classify INSIDE A (the cut survival rule, via `classifyPoint`); a tangent /
degenerate / wrong-side centroid, a short cylinder whose cap clips the seam band, a non-Steinmetz
or unresolved branched input, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT).
The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cylinder` (wall) and planar
(cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed volume
equals `vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where `vol(A ∩ B)`
is the native bicylinder COMMON. The builder SHALL remain OCCT-free, reference no OCCT /
`IEngine` / `EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no
`cc_*` entry point, signature, or POD struct.

#### Scenario: The Steinmetz cut carves the channel with the correct volume (host)
- GIVEN two equal-radius cylinders A (radius `R`, length `L_A`, minuend) and B (radius `R`),
  orthogonal crossing axes, a resolved 2-branch-point / four-arc `TraceSet` with
  `nearTangentGaps == 0`
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (branch-enabled re-trace resolves the arcs and
  `buildSteinmetzCut` assembles the shell)
- THEN it returns a watertight `Solid` bounded by A's two OUTSIDE lune walls (outward) + A's two
  disc caps + B's two INSIDE lune walls REVERSED (inward, bounding the channel), all sharing the
  four arcs
- AND its enclosed volume equals `π R² L_A − 16 R³/3` within the deflection-sized band — a SHRINK
  (`Vr < vol(A)`)
- AND every arc node lies on both cylinder surfaces within tolerance, and the two poles are
  pooled ONCE.

#### Scenario: The reversed inner lunes bound the channel, verified against the native common (host)
- GIVEN the Steinmetz cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` = native
  bicylinder COMMON `buildSteinmetzCommon`; the `op == 2`-only analytic oracle does NOT intercept
  cut)
- THEN a candidate whose reversed INSIDE-B lunes are mis-oriented (outward, not bounding the
  channel) yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the
  correct candidate matches `vol(A) − 16 R³/3` and is accepted native.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME equal-radius orthogonal Steinmetz pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the shared arc/pole prologue is factored
  into `orientResampleArcs`
- THEN `buildSteinmetzCommon` produces the byte-identical four-inside-lune bicylinder (same
  volume `16 R³/3`, area, and vertices as before this change) — the COMMON native pass does not
  regress.

### Requirement: SSI-driven native Common for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common (`A ∩ B`) NATIVELY for a
COAXIAL cone∩cylinder pair: one operand recognised as a `Cone` frustum solid (via
`recogniseCurvedSolid`), the other as a `Cylinder` solid whose axis is COLLINEAR with the cone
axis (`sameAxis`), whose S3/S1 seam trace is EXACTLY ONE closed full-circle seam on BOTH walls
(the S1 analytic circle from `intersectCylinderConeCoaxial`, `nearTangentGaps == 0`,
`branchPoints == 0`), where the frustum is APEX-FREE over its extent (`r_c(v) > margin` for all
`v`, so the S4-e apex chart singularity is never touched) and the seam height `h*` (where the cone
cross-section radius `r_c(h*)` equals the cylinder radius `Rc`) lies STRICTLY inside the axial
overlap `[hBot, hTop] = [max(coneBottom, cylBottom), min(coneTop, cylTop)]`.

The builder SHALL assemble the common boundary as the min-radius-profile solid of revolution: the
CONE wall band on the side of `h*` where the cone radius is the smaller, welded along the single
seam circle to the CYLINDER wall band on the side where `Rc` is the smaller, closed by two planar
disc caps (a bottom cap at `hBot` with outward normal `−ẑ`, a top cap at `hTop` with outward
normal `+ẑ`). It SHALL resample the traced seam circle into one full-turn ring pooled ONCE (the
shared seam ring), emit each band as a planar-facet ring strip drawn through the shared
`VertexPool` so the two bands weld along the seam ring and each band's outer terminal ring welds
to its disc cap, and use the cone's TRUE outward wall normal (`radial·cosα − ẑ·sinα`) for the cone
band and the pure outward radial normal for the cylinder band.

The builder SHALL keep the CONE band only if its interior sample classifies strictly INSIDE the
cylinder (`classifyPoint(cyl, mid) == 1`) AND the CYLINDER band only if its interior sample
classifies strictly INSIDE the cone (`classifyPoint(cone, mid) == 1`); an ON verdict (`== 0`, a
tangent / cap-edge seam), a frustum whose extent reaches the apex, a non-coaxial (transversal)
pair, a seam that is not exactly one interior full circle, or a weld that cannot close SHALL
return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cone` (frustum band)
and `Cylinder` (cylinder band) and planar (cap) face kinds, watertight (every edge shared by
exactly two faces), whose enclosed volume equals
`V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)`, where
`V_frustum(ra → rb over Δh) = (π·Δh/3)(ra² + ra·rb + rb²)`, within a relative tolerance sized to
the curved-face tessellation deflection. The builder SHALL remain OCCT-free and reference no OCCT
/ `IEngine` / `EngineShape` type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or
change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder common is the min-profile solid with the correct volume (host)
- GIVEN a frustum cone A (`r_c(h) = R0 + (h − h0)·tanα`, apex-free over its extent) and a coaxial
  cylinder B (radius `Rc`) whose walls cross at a single circle at height `h*` (`r_c(h*) = Rc`)
  strictly inside the axial overlap `[hBot, hTop]`, with a clean single-circle seam trace
  (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs (`buildCommon` declines the single seam,
  `buildLensCommon` declines the non-sphere operand, and `buildConeCylCommon` assembles the shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the CONE band (`r_c ≤ Rc` side, inside the cylinder) welded along the seam
  circle to the CYLINDER band (`Rc ≤ r_c` side, inside the cone), closed by a bottom disc cap at
  `hBot` and a top disc cap at `hTop`
- AND its enclosed volume equals `V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)` within the
  deflection-sized band (for the reference fixture `r_c(h)=0.4+0.4h`, `Rc=1`, `h∈[1,5]`: `h*=1.5`,
  overlap `[1,4]`, volume ≈ `9.1315`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by both bands).

#### Scenario: An apex-crossing / transversal / cap-tangent cone pair declines to OCCT (host)
- GIVEN a coaxial cone∩cylinder pair whose frustum extent reaches the apex (`r_c → 0`), OR a
  NON-coaxial (transversal) cone∩cylinder pair whose seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a pair whose seam `h*` sits on a cap
  edge (a tangent, not a strictly-interior transversal circle)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeCylCommon` refuses at the gate (apex, non-coaxial, or ON-edge seam) and returns a
  NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Common` — reported, not faked,
  tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone common candidate (host)
- GIVEN a cone-common candidate whose welded shell volume does not match
  `V_frustum(r(hBot) → Rc) + π·Rc²·(hTop − h*)` (a mis-selected band, a mis-placed cap, or a
  hairline seam-ring gap)
- WHEN the engine's `ssiCurvedBooleanVerified` runs the coaxial-cone closed-form oracle (the
  `op == 2` branch, mirroring the Steinmetz `16 R³/3` oracle)
- THEN the candidate FAILS the watertight + closed-form-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified cone common.

#### Scenario: Cone Fuse / Cut and other cone pairs remain the OCCT boundary (host)
- GIVEN a coaxial cone∩cylinder pair with `Op::Fuse` or `Op::Cut`, OR a cone∩cone pair, OR a
  two-circle coaxial cone∩sphere crossing
- WHEN `ssi_boolean_solid` runs
- THEN the cone COMMON path is NOT wired for fuse/cut, cone∩cone, or the two-circle crossing, so it
  returns a NULL `Shape` and the engine ships OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` — the
  analytically-clean coaxial cone∩cylinder (and single-crossing cone∩sphere) COMMON is the only
  new native cone op.

#### Scenario: The cyl / sphere / Steinmetz families are unchanged by the cone addition (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz bicylinder fixtures
  across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Common` dispatch grows the `buildConeCylCommon` arm
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeCylCommon` returns `{}` for every non-(cone+coaxial-cylinder)
  pair, so the existing native passes do not regress.

### Requirement: SSI-driven native Common for the coaxial cone∩sphere pair (optional)

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common NATIVELY for a COAXIAL
cone∩sphere pair in the SINGLE-crossing configuration: one operand a `Cone` frustum, the other a
`Sphere` whose centre lies on the cone axis, whose S1 seam (`intersectSphereConeCoaxial`) is
EXACTLY ONE valid circle inside both operand extents. It SHALL assemble the frustum band welded
along that seam circle to the spherical-segment band (the sphere-latitude strip inside the cone),
closed by the terminal disc caps, selected by the same inside-the-other rule (`classifyPoint`) and
welded watertight through one `VertexPool`. When `intersectSphereConeCoaxial` returns TWO circles
inside both extents, or the frustum reaches the apex, or the pair is non-coaxial, the builder SHALL
return a NULL `Shape` (→ OCCT). The result SHALL be a watertight `Solid` whose enclosed volume
equals `V_frustum + V_spherical-segment` within the deflection-sized tolerance, verified by the
engine self-verify against that closed form. The builder SHALL remain OCCT-free, be compiled under
`CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct. This
requirement is OPTIONAL — shipped only if the coaxial cone∩sphere COMMON lands watertight in the
verified envelope; otherwise it is deferred → OCCT with the measured gap reported.

#### Scenario: The single-crossing coaxial cone∩sphere common has the correct closed-form volume (host)
- GIVEN a frustum cone A and a coaxial sphere B (centre on the cone axis) whose S1 seam is exactly
  ONE valid circle inside both extents
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs and `buildConeSphereCommon` assembles the shell
- THEN it returns a watertight `Solid` bounded by the frustum band welded along the seam circle to
  the spherical-segment band, closed by the terminal disc caps
- AND its enclosed volume equals `V_frustum + V_spherical-segment` within the deflection-sized band
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: A two-circle coaxial cone∩sphere crossing declines to OCCT (host)
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeSphereCommon` declines the two-circle config and returns a NULL `Shape`, and the
  engine ships OCCT `BRepAlgoAPI_Common` — reported, not faked (the first slice handles the
  single-crossing config only).

### Requirement: SSI-driven native Fuse for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for the
COAXIAL cone∩cylinder pair the S5-e COMMON path (`buildConeCylCommon`) already recognises: one
operand recognised as a `Cone` frustum solid (via `recogniseCurvedSolid`), the other as a
`Cylinder` solid whose axis is COLLINEAR with the cone axis (`sameAxis`), whose S3/S1 seam trace is
EXACTLY ONE closed full-circle seam on BOTH walls (the S1 analytic circle from
`intersectCylinderConeCoaxial`, `nearTangentGaps == 0`, `branchPoints == 0`), where the frustum is
APEX-FREE over its extent and the seam height `s*` (where the cone cross-section radius `r_c(s*)`
equals the cylinder radius `Rc`) lies STRICTLY inside the axial overlap `[sLo, sHi]`.

The builder SHALL reuse the SAME shared gate/seam prologue (`coneCylSetup`: the coaxial gate, the
analytic-vs-traced seam cross-check, the axis frame, the crossing `s*`, the azimuth resolution, and
the pooled seam ring at `(Rc, s*)`), the SAME shared `VertexPool` weld with the seam ring pooled
ONCE, and the SAME planar-facet revolve discipline (`appendRevolvedBand`, `appendDiskCap`) as
`buildConeCylCommon`, and SHALL assemble the union boundary as the MAX-radius outer profile
`max(r_c(s), Rc)` over the union extent `[min(coneLo, cylLo), max(coneHi, cylHi)]`: the OUTER wall
of whichever operand is wider on each side of the seam (the cylinder wall on the cone-inner side,
the cone wall on the cyl-inner side, welded along the single seam circle), PLUS the wall segments
beyond the overlap (each operand's wall where the other is absent), closed by the two terminal disc
caps (outward `∓ẑ`) AND the annular step caps (flat washers with axial `±ẑ` normal) where one
operand's end-cap disc protrudes past the other's wall radius.

The builder SHALL keep each outer wall band only if its interior sample classifies strictly OUTSIDE
the other solid (the fuse survival rule, via the S5-a curved point-in-solid test `classifyPoint`);
a sample robustly ON the other wall (tangent, `classifyPoint == 0`), a frustum whose extent reaches
the apex, a non-coaxial (transversal) pair, a seam that is not exactly one strictly-interior full
circle, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be
weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying true `Cone` (frustum band)
and `Cylinder` (cylinder band) wall face kinds and planar (disc + annular cap) face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals `vol(A) + vol(B)
− vol(A ∩ B)` within a relative tolerance sized to the curved-face tessellation deflection, where
`vol(A ∩ B)` is the native cone∩cylinder COMMON (`buildConeCylCommon`). The builder SHALL remain
OCCT-free and reference no OCCT / `IEngine` / `EngineShape` type, SHALL be compiled under
`CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder fuse grows to the outer envelope with the correct volume (host)
- GIVEN a frustum cone A (`r_c(s) = R0 + s·tanα`, apex-free over its extent) and a coaxial cylinder
  B (radius `Rc`) whose walls cross at a single circle at `s*` (`r_c(s*) = Rc`) strictly inside the
  axial overlap, with a clean single-circle seam trace (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse` declines the single
  seam, the sphere `buildLensFuse` declines the non-sphere operand, and `buildConeCylFuse` assembles
  the outer shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the max-radius outer profile — the wider operand's wall on each side of the seam
  circle, the beyond-overlap wall segments, the two terminal disc caps, and the annular step caps
  where an end-cap disc protrudes
- AND its enclosed volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized band (for
  the reference fixture `r_c(y) = 0.5 + 0.5y`, `Rc = 1.5`, cone `[0,4]`, cyl `[1,5]`: `s* = 2`,
  volume ≈ `41.626`) — a GROW (`Vr > max(vol(A), vol(B))`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by the cylinder outer band below `s*` and the cone outer band above `s*`).

#### Scenario: An apex-crossing / transversal / cap-tangent cone pair declines fuse to OCCT (host)
- GIVEN a coaxial cone∩cylinder pair whose frustum extent reaches the apex (`r_c → 0`), OR a NON-
  coaxial (transversal) cone∩cylinder pair whose seam is a quartic space curve
  (`intersectCylinderConeCoaxial` returns `notAnalytic`), OR a pair whose seam `s*` sits on a cap
  edge (a tangent, not a strictly-interior transversal circle)
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs
- THEN `buildConeCylFuse` refuses at the shared gate (apex, non-coaxial, or ON-edge seam) and
  returns a NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Fuse` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone∩cylinder fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) − vol(A ∩ B)` (a
  mis-selected outer band or a mis-placed annular cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` = native
  cone∩cylinder COMMON `buildConeCylCommon`; the `op == 2`-only analytic oracle does NOT intercept
  fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the engine
  never emits an unverified cone∩cylinder fuse.

### Requirement: SSI-driven native Cut for the coaxial cone∩cylinder pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the minuend)
NATIVELY for the coaxial cone∩cylinder pair the S5-e COMMON path already recognises (one `Cone`
frustum + one coaxial `Cylinder`, a single strictly-interior full-circle apex-free seam,
`nearTangentGaps == 0`). It SHALL reuse the SAME shared gate/seam prologue (`coneCylSetup`),
`VertexPool` (seam ring pooled once), and planar-facet revolve discipline as `buildConeCylCommon`,
and SHALL assemble the difference boundary as A's OUTER wall bands (the part of A outside B, outward
radial normal) plus A's terminal disc cap(s) outside B plus A's cap-annulus where A's end-cap disc
extends past B, plus B's INSIDE-A wall band emitted REVERSED (inward radial normal, welded at the
pooled seam circle so it bounds the carved cavity) plus B's end-cap disc inside A emitted REVERSED
(the cavity floor/ceiling). Operand order SHALL be honoured (CUT is not symmetric), matching
`BRepAlgoAPI_Cut(a, b)`. The result MAY be DISCONNECTED (two closed components — e.g. a small end
frustum plus a conical washer); it SHALL be assembled as one shell whose components share the pool
and whose summed mesh volume is verified.

The builder SHALL keep each A outer band only if its interior sample classifies strictly OUTSIDE B
AND each reversed B inside band only if its interior sample classifies strictly INSIDE A (the cut
survival rule, via `classifyPoint`); a tangent / degenerate / wrong-side sample (`classifyPoint ==
0`), a frustum whose extent reaches the apex, a non-coaxial (transversal) pair, a cap-edge-tangent
seam, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be
weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall) and `Cylinder`
(wall) and planar (disc + annular cap) face kinds, watertight per component (every edge shared by
exactly two faces), whose enclosed (summed) volume equals `vol(A) − vol(A ∩ B)` within the
deflection-sized relative tolerance, where `vol(A ∩ B)` is the native cone∩cylinder COMMON. The
builder SHALL remain OCCT-free, reference no OCCT / `IEngine` / `EngineShape` type, be compiled
under `CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩cylinder cut carves the cavity with the correct volume (host)
- GIVEN two coaxial operands A (a frustum cone, minuend) and B (a cylinder) whose walls cross at a
  single circle at `s*` strictly inside the axial overlap, with a clean single-circle seam trace
  (`nearTangentGaps == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut` and sphere
  `buildLensCut` decline, and `buildConeCylCut` assembles the shell)
- THEN it returns a watertight `Solid` bounded by A's OUTER wall bands (outside B, outward) + A's
  disc cap(s) + A's cap-annulus outside B, plus B's INSIDE-A wall band REVERSED (inward, pinching to
  the seam circle) + B's cap disc inside A REVERSED (the cavity floor/ceiling), all sharing the
  single seam circle
- AND its enclosed (summed over components — e.g. an end frustum + a conical washer) volume equals
  `vol(A) − vol(A ∩ B)` within the deflection-sized band (for the reference fixture ≈ `13.352`) — a
  SHRINK (`Vr < vol(A)`)
- AND every seam-ring node lies on both walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: The reversed inner band bounds the cavity, verified against the native common (host)
- GIVEN the coaxial cone∩cylinder cut candidate above
- WHEN the engine's generic set-algebra self-verify runs (`expected = va − vc`, `vc` = native
  cone∩cylinder COMMON `buildConeCylCommon`; the `op == 2`-only analytic oracle does NOT intercept
  cut)
- THEN a candidate whose reversed INSIDE-B band is mis-oriented (outward, not bounding the cavity)
  yields the wrong enclosed volume, FAILS the guard, and is DISCARDED → OCCT — the correct candidate
  matches `vol(A) − vol(A ∩ B)` and is accepted native.

### Requirement: The COMMON path and other pairs are unchanged by the cone∩cylinder fuse/cut completion

The native boolean library SHALL keep the coaxial cone∩cylinder COMMON (`buildConeCylCommon`) and
every other curved-boolean family byte-identical when the fuse/cut completion lands. Factoring the
shared gate/seam prologue into `coneCylSetup` SHALL NOT change the COMMON result (same volume, area,
and vertices), and `buildConeCylFuse` / `buildConeCylCut` SHALL return a NULL `Shape` for every non-
(cone + coaxial-cylinder) pair so the through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz
bicylinder builders and all their ops keep their existing results. The dispatch SHALL grow only the
`Op::Fuse` and `Op::Cut` arms (one final call each after the through-drill and lens builders
decline); recognition, tracing, the transversality gate, and the engine self-verify SHALL NOT
change.

#### Scenario: The COMMON path is unchanged by the fuse/cut completion (host)
- GIVEN the SAME coaxial cone∩cylinder pair
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs after the shared gate/seam prologue is factored
  into `coneCylSetup`
- THEN `buildConeCylCommon` produces the byte-identical min-radius-profile common (same volume
  `vol(A ∩ B)`, area, and vertices as before this change) — the COMMON native pass does not regress.

#### Scenario: The cyl / sphere / Steinmetz families are unchanged by the cone∩cylinder fuse/cut addition (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, and Steinmetz bicylinder fixtures
  across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Fuse` / `Op::Cut` arms grow the `buildConeCylFuse` /
  `buildConeCylCut` calls
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeCylFuse` / `buildConeCylCut` return `{}` for every non-(cone +
  coaxial-cylinder) pair, so the existing native passes do not regress.

#### Scenario: Coaxial cone∩sphere and cone∩cone remain the OCCT boundary (host)
- GIVEN a coaxial cone∩sphere pair OR a cone∩cone pair, with `Op::Fuse` or `Op::Cut`
- WHEN `ssi_boolean_solid` runs
- THEN `buildConeCylFuse` / `buildConeCylCut` decline (the gate requires one `Cone` + one coaxial
  `Cylinder`) and return a NULL `Shape`, and the engine ships OCCT `BRepAlgoAPI_{Fuse,Cut}` — the
  coaxial cone∩cylinder is the only cone family with native fuse/cut.

### Requirement: SSI-driven native Common for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=2)` common (`A ∩ B`) NATIVELY for the
COAXIAL cone∩sphere pair in the SINGLE-crossing configuration: one operand recognised as a `Cone`
frustum solid (via `recogniseCurvedSolid`), the other as a `Sphere` solid whose centre lies ON the
cone axis (`distancePointLine`/`sameAxis`), whose S1 seam trace is EXACTLY ONE closed full-circle
seam on BOTH walls (the S1 analytic circle from `intersectSphereConeCoaxial`, `nearTangentGaps ==
0`, `branchPoints == 0`), where the frustum is APEX-FREE over its extent and the single seam height
`s*` (where the cone cross-section radius `r_c(s*)` equals the sphere cross-section radius `r_s(s*)`)
lies STRICTLY inside both operand extents — the QUADRATIC's other root falling OUTSIDE the frustum
extent.

The builder SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`: the coaxial gate, the
analytic-vs-traced seam cross-check, the axis frame, the crossing `s*`, the azimuth resolution, the
two sphere poles classified against the cone, and ONE canonical pooled seam ring at `(r_c(s*), s*)`),
the SAME shared `VertexPool` weld with the seam ring pooled ONCE, the cone-side planar-facet revolve
discipline (`appendRevolvedBand`, `appendDiskCap`), and the sphere-side cap builder
(`appendSphereCap`), and SHALL assemble the overlap boundary as the min-cross-section profile: the
CONE wall band on the `r_c ≤ r_s` side (inside the sphere) welded along the seam circle to the
SPHERE INNER cap on the `r_s ≤ r_c` side (the spherical segment inside the cone, closing to the
sphere pole that lies inside the cone), closed by the cone terminal disc that bounds the overlap (the
cone end disc inside the sphere).

The builder SHALL keep the cone band only if its interior sample classifies strictly INSIDE the
sphere AND the sphere inner cap only if its pole classifies strictly INSIDE the cone (the common
survival rule, via the curved point-in-solid test `classifyPoint`); a sample robustly ON the other
wall (`classifyPoint == 0`), a TWO-circle crossing (both roots inside both extents), a frustum whose
extent reaches the apex, a non-coaxial (transversal) pair, or a weld that cannot close SHALL return a
NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened to force a pass.

The result SHALL be a native `topology::Shape` of type `Solid` carrying a true `Cone` (frustum band)
wall face kind, a true `Sphere` (spherical cap) wall face kind, and planar (disc cap) face kinds,
watertight (every edge shared by exactly two faces), whose enclosed volume equals the closed form
`V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-tighter sub-band)` within a relative
tolerance sized to the curved-face tessellation deflection — verified by the engine self-verify
against that closed form (a NEW analytic oracle arm mirroring the Steinmetz `16 r³/3` and
cone∩cylinder arms). The builder SHALL remain OCCT-free and reference no OCCT / `IEngine` /
`EngineShape` type, SHALL be compiled under `CYBERCAD_HAS_NUMSCI`, and SHALL add or change no `cc_*`
entry point, signature, or POD struct.

#### Scenario: The single-crossing coaxial cone∩sphere common has the correct closed-form volume (host)
- GIVEN a frustum cone A (`r_c(s) = R0 + s·tanα`, apex-free) and a coaxial sphere B (centre on the
  cone axis) whose walls cross at a single circle at `s*` (`r_c(s*) = r_s(s*)`) strictly inside both
  extents, with a clean single-circle seam trace (`nearTangentGaps == 0`, `branchPoints == 0`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs (the through-drill `buildCommon`, the sphere-lens
  `buildLensCommon`, and the cone∩cylinder `buildConeCylCommon` all decline, and
  `buildConeSphereCommon` assembles the overlap shell)
- THEN it returns a watertight `Solid` (`boundaryEdgeCount == 0`, every edge shared by exactly two
  faces) bounded by the cone wall band inside the sphere welded along the seam circle to the
  spherical segment inside the cone, closed by the cone terminal disc inside the sphere
- AND its enclosed volume equals `V_frustum(cone-tighter sub-band) + V_spherical-segment(sphere-
  tighter sub-band)` within the deflection-sized band (for the reference fixture `r_c(s) = 0.5 +
  0.5s`, sphere centre on-axis at the origin, `Rs = 2`: `s* ≈ 1.5436`, volume ≈ `5.256`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE
  (shared by the cone band and the spherical cap).

#### Scenario: A two-circle / apex / transversal cone∩sphere pair declines common to OCCT (host)
- GIVEN a coaxial cone∩sphere pair whose `intersectSphereConeCoaxial` returns TWO circles inside
  both extents (the sphere passes fully through the cone), OR a frustum whose extent reaches the
  apex (`r_c → 0`), OR a NON-coaxial (transversal) cone∩sphere pair whose seam is a quartic space
  curve (`intersectSphereConeCoaxial` returns `notAnalytic`)
- WHEN `ssi_boolean_solid(A, B, Op::Common)` runs
- THEN `buildConeSphereCommon` refuses at the shared gate (two-circle, apex, or non-coaxial) and
  returns a NULL `Shape`, and the engine falls through to OCCT `BRepAlgoAPI_Common` — reported, not
  faked, tolerance not weakened.

#### Scenario: The engine discards a wrong-volume cone∩sphere common candidate (host)
- GIVEN a common candidate whose welded shell volume does not match `V_frustum + V_spherical-
  segment` (a mis-selected band or a mis-placed cap)
- WHEN the engine's NEW coaxial cone∩sphere COMMON analytic oracle arm in `ssiCurvedBooleanVerified`
  runs (`op == 2`)
- THEN the candidate FAILS the watertight + closed-form-volume guard and is DISCARDED → OCCT; the
  engine never emits an unverified cone∩sphere common.

### Requirement: SSI-driven native Fuse for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=0)` fuse (`A ∪ B`) NATIVELY for the
coaxial cone∩sphere single-crossing pair the common path recognises (one `Cone` frustum + one
coaxial `Sphere` centre-on-axis, a single strictly-interior full-circle apex-free seam,
`nearTangentGaps == 0`). It SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`),
`VertexPool` (seam ring pooled once), cone-side revolve discipline, and sphere-side cap builder as
`buildConeSphereCommon`, and SHALL assemble the union boundary as the max-cross-section outer
profile: the SPHERE OUTER cap (the part of the sphere outside the cone, `appendSphereCap` with the
FAR-pole apex, outward) welded along the SAME seam circle to the CONE OUTER wall band (the part of
the cone outside the sphere, outward radial), closed by the cone terminal disc(s) that bound the
union.

The builder SHALL keep the sphere outer cap only if its far pole classifies strictly OUTSIDE the
cone AND each cone outer band only if its interior sample classifies strictly OUTSIDE the sphere
(the fuse survival rule, via `classifyPoint`); a sample robustly ON the other wall
(`classifyPoint == 0`), a two-circle crossing, an apex-reaching frustum, a non-coaxial pair, or a
weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall), `Sphere` (cap), and
planar (disc cap) face kinds, watertight (every edge shared by exactly two faces), whose enclosed
volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized relative tolerance, where
`vol(A ∩ B)` is the native cone∩sphere COMMON (`buildConeSphereCommon`). The builder SHALL remain
OCCT-free, reference no OCCT / `IEngine` / `EngineShape` type, be compiled under
`CYBERCAD_HAS_NUMSCI`, and add or change no `cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩sphere fuse grows to the outer envelope with the correct volume (host)
- GIVEN a frustum cone A and a coaxial sphere B (centre on the cone axis) whose walls cross at a
  single circle at `s*` strictly inside both extents, with a clean single-circle seam trace
- WHEN `ssi_boolean_solid(A, B, Op::Fuse)` runs (the through-drill `buildFuse`, the sphere-lens
  `buildLensFuse`, and the cone∩cylinder `buildConeCylFuse` all decline, and `buildConeSphereFuse`
  assembles the outer shell)
- THEN it returns a watertight `Solid` bounded by the sphere outer cap outside the cone welded along
  the seam circle to the cone outer wall outside the sphere, closed by the cone terminal disc
- AND its enclosed volume equals `vol(A) + vol(B) − vol(A ∩ B)` within the deflection-sized band
  (for the reference fixture volume ≈ `60.718`) — a GROW (`Vr > max(vol(A), vol(B))`)
- AND every seam-ring node lies on BOTH walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: The engine discards a wrong-volume cone∩sphere fuse candidate (host)
- GIVEN a fuse candidate whose welded shell volume does not match `vol(A) + vol(B) − vol(A ∩ B)` (a
  mis-selected outer band or a mis-oriented sphere cap)
- WHEN the engine's generic set-algebra self-verify runs (`expected = va + vb − vc`, `vc` = native
  cone∩sphere COMMON `buildConeSphereCommon`; the `op == 2`-only analytic oracle does NOT intercept
  fuse)
- THEN the candidate FAILS the watertight + correct-volume guard and is DISCARDED → OCCT; the engine
  never emits an unverified cone∩sphere fuse.

### Requirement: SSI-driven native Cut for the coaxial cone∩sphere pair

The native boolean library SHALL compute `cc_boolean(a, b, op=1)` cut (`A − B`, `A` the CONE
minuend) NATIVELY for the coaxial cone∩sphere single-crossing pair the common path recognises. It
SHALL reuse the SAME shared gate/seam prologue (`coneSphereSetup`), `VertexPool` (seam ring pooled
once), cone-side revolve discipline, and sphere-side cap builder as `buildConeSphereCommon`, and
SHALL assemble the difference boundary as A's OUTER wall band (the part of A outside B, outward
radial) plus A's terminal disc cap(s) outside B, plus the sphere INNER cap emitted REVERSED
(`appendSphereCap` with the near-pole apex and the inward normal — the spherical dimple bounding the
carved cavity, welded at the pooled seam circle so it pinches to the seam ring). Operand order SHALL
be honoured (CUT is not symmetric): `A` SHALL be the `Cone`; a `sphere − cone` (sphere minuend)
SHALL return a NULL `Shape` (→ OCCT). The single-crossing cone∩sphere cut SHALL be a CONNECTED solid
(one closed component — a frustum with a spherical dimple).

The builder SHALL keep each A outer band only if its interior sample classifies strictly OUTSIDE B
AND the reversed sphere inner cap only if its near pole classifies strictly INSIDE A (the cut
survival rule, via `classifyPoint`); a tangent / degenerate / wrong-side sample
(`classifyPoint == 0`), a two-circle crossing, an apex-reaching frustum, a non-coaxial pair, a
sphere minuend, or a weld that cannot close SHALL return a NULL `Shape` (→ OCCT). The tolerance
SHALL NOT be weakened.

The result SHALL be a native `topology::Shape` `Solid` with true `Cone` (wall), `Sphere` (dimple
cap), and planar (disc cap) face kinds, watertight (every edge shared by exactly two faces), whose
enclosed volume equals `vol(A) − vol(A ∩ B)` within the deflection-sized relative tolerance, where
`vol(A ∩ B)` is the native cone∩sphere COMMON. The builder SHALL remain OCCT-free, reference no OCCT
/ `IEngine` / `EngineShape` type, be compiled under `CYBERCAD_HAS_NUMSCI`, and add or change no
`cc_*` entry point, signature, or POD struct.

#### Scenario: The coaxial cone∩sphere cut carves a spherical dimple with the correct volume (host)
- GIVEN two coaxial operands A (a frustum cone, minuend) and B (a sphere centre-on-axis) whose walls
  cross at a single circle at `s*` strictly inside both extents, with a clean single-circle seam
  trace
- WHEN `ssi_boolean_solid(A, B, Op::Cut)` runs (the through-drill `buildCut`, the sphere-lens
  `buildLensCut`, and the cone∩cylinder `buildConeCylCut` all decline, and `buildConeSphereCut`
  assembles the shell)
- THEN it returns a watertight CONNECTED `Solid` bounded by A's OUTER wall band (outside B, outward)
  + A's disc cap(s) outside B + the sphere INNER cap REVERSED (inward, the dimple pinching to the
  seam circle)
- AND its enclosed volume equals `vol(A) − vol(A ∩ B)` within the deflection-sized band (for the
  reference fixture ≈ `27.207`) — a SHRINK (`Vr < vol(A)`)
- AND every seam-ring node lies on both walls within tolerance, and the seam ring is pooled ONCE.

#### Scenario: A sphere-minuend cut declines to OCCT (host)
- GIVEN a coaxial cone∩sphere pair with the SPHERE as the first operand (`sphere − cone`)
- WHEN `ssi_boolean_solid(sphere, cone, Op::Cut)` runs
- THEN `buildConeSphereCut` declines (`A` is not the `Cone` minuend) and returns a NULL `Shape`, and
  the engine ships OCCT `BRepAlgoAPI_Cut` — the sphere-with-a-conical-bite is a deferred follow-on,
  reported, not faked.

### Requirement: The existing curved-boolean families are unchanged by the cone∩sphere addition

The native boolean library SHALL keep every existing curved-boolean family (through-drill cyl∩cyl,
sphere∩sphere lens, Steinmetz bicylinder, coaxial cone∩cylinder) byte-identical when the cone∩sphere
op-set lands. `buildConeSphere{Common,Fuse,Cut}` SHALL return a NULL `Shape` for every non-(cone +
coaxial-sphere) pair so the existing builders and all their ops keep their existing results. The
dispatch SHALL grow only one final call per op arm (after the through-drill, lens, and cone∩cylinder
builders decline); recognition, tracing, the transversality gate, and the generic set-algebra
self-verify SHALL NOT change, and the NEW `ssiCurvedBooleanVerified` cone∩sphere arm SHALL return
not-applicable for every non-cone∩sphere pair and for `op != 2`.

#### Scenario: The cyl / sphere / Steinmetz / cone∩cylinder families are unchanged (host)
- GIVEN the existing through-drill cyl∩cyl, sphere∩sphere lens, Steinmetz bicylinder, and coaxial
  cone∩cylinder fixtures across all three ops
- WHEN `ssi_boolean_solid` runs after the `Op::Common` / `Op::Fuse` / `Op::Cut` arms grow the
  `buildConeSphere{Common,Fuse,Cut}` calls and the engine gains the cone∩sphere COMMON oracle arm
- THEN each existing family produces its byte-identical result (same volume, area, and vertices as
  before this change) — `buildConeSphere{Common,Fuse,Cut}` return `{}` for every non-(cone +
  coaxial-sphere) pair and the cone∩sphere oracle arm returns not-applicable for them, so the
  existing native passes do not regress.

#### Scenario: cone∩cone remains the OCCT boundary (host)
- GIVEN a cone∩cone pair with `Op::Common`, `Op::Fuse`, or `Op::Cut`
- WHEN `ssi_boolean_solid` runs
- THEN `buildConeSphere{Common,Fuse,Cut}` decline (the gate requires one `Cone` + one coaxial
  `Sphere`) and return a NULL `Shape`, and the engine ships OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` —
  the coaxial cone∩cylinder and coaxial cone∩sphere are the only cone families with native booleans.

### Requirement: Native point-in-freeform-solid membership by ray-cast over the M0 boundary mesh

The native boolean library SHALL provide an OCCT-free, header-only classifier
`classifyPointInMesh(boundary, bbox, meshDeflection, p, tol)` that, given the
**watertight boundary triangle mesh** of a solid whose boundary includes a genuinely
trimmed freeform (`Kind::BSpline` / `Kind::Bezier`) face — the mesh produced by the
landed MOAT M0 tessellator (`src/native/tessellate` `SolidMesher::mesh` /
`trimmedFreeformMesh`, CONSUMED read-only, NOT rewritten) — and an arbitrary query
point `p`, returns one of `IN`, `OUT`, `ON`, or `UNKNOWN`. The classifier SHALL:

- decide inside/outside by **odd/even ray crossings** — casting a ray from `p` and
  counting forward triangle intersections (Möller–Trumbore, fp64); an ODD count ⇒
  `IN`, an EVEN count ⇒ `OUT`;
- use **multi-ray consensus**: cast a fixed set of independent, non-axis-aligned,
  mutually non-parallel ray directions, DISCARD any ray with a near-degenerate
  crossing (a hit within an edge/vertex tolerance, or a grazing ray nearly parallel
  to a face), and take the MAJORITY parity of the usable rays; if fewer than a quorum
  of rays are usable, or the usable rays DISAGREE, return `UNKNOWN` (an honest
  decline) rather than a guessed IN/OUT;
- report `ON` when the minimum distance from `p` to any boundary triangle is within a
  **scale-relative ON-band** `band = max(absTol, relTol·diag(bbox)) + meshDeflection`
  — the point is within the mesh's own deflection + tolerance resolution of the
  boundary and cannot be crisply classified;
- treat a **non-watertight** input mesh as out of scope and return `UNKNOWN` (never a
  fabricated verdict).

This classifier SHALL be strictly ADDITIVE: it SHALL NOT modify the existing analytic
`classifyPoint` (`src/native/boolean/ssi_boolean.h`) or `recogniseCurvedSolid`, SHALL
NOT modify `src/native/tessellate/**`, SHALL introduce NO `cc_*` ABI surface, and
SHALL add ZERO OCCT includes under `src/native/**`. It SHALL NEVER emit a wrong
classification silently: near-boundary / near-tangent points resolve to `ON`, and
ambiguous ray consensus resolves to `UNKNOWN`. This first slice targets the SIMPLEST
reachable case — a **single freeform-face solid**, with correctness graded on points
**comfortably away from the ON band**; robust ON-boundary / near-tangent
classification is an explicitly deferred asymptotic tail.

#### Scenario: Points comfortably inside/outside a freeform-walled solid classify IN/OUT against analytic truth (host)

- GIVEN a freeform-walled solid whose curved boundary is a `Kind::BSpline` face whose inside/outside is KNOWN in closed form (e.g. a B-spline wall coincident with a cylinder of radius `r`, or a convex-extruded-B-spline profile), built on the host with NO OCCT and meshed watertight by the M0 `SolidMesher`
- WHEN `classifyPointInMesh` is evaluated at sample points placed COMFORTABLY away from the ON band (well inside and well outside the closed-form region)
- THEN every verdict SHALL equal the closed-form analytic truth (`IN` for interior points, `OUT` for exterior points), with NO crisp verdict emitted for any point inside the band

#### Scenario: A point within the deflection+tolerance band resolves to ON, not a crisp IN/OUT (host)

- GIVEN the same freeform-walled solid meshed at deflection `d`, and a query point whose minimum distance to the boundary mesh is within `band = max(absTol, relTol·diag) + d`
- WHEN `classifyPointInMesh` is evaluated at that point
- THEN the verdict SHALL be `ON` — the point is within the mesh's own resolution of the surface — and SHALL NOT be reported as a fabricated crisp `IN` or `OUT`

#### Scenario: An ambiguous or grazing ray arrangement declines to UNKNOWN rather than guessing (host)

- GIVEN a query point comfortably away from the boundary but positioned so that a candidate ray grazes a shared edge/vertex or runs nearly parallel to a face
- WHEN `classifyPointInMesh` casts its ray set and cannot reach a usable-ray quorum with agreeing parity
- THEN the verdict SHALL be `UNKNOWN` (an honest decline), and NO wrong crisp classification SHALL be emitted (0 silent wrong)

#### Scenario: The native classifier agrees with OCCT BRepClass3d_SolidClassifier on N random points (sim)

- GIVEN a trimmed-freeform-walled solid built BOTH natively and as an OCCT solid on a booted iOS simulator with OCCT linked, the native one meshed watertight by the M0 `SolidMesher`
- WHEN `classifyPointInMesh` is evaluated at N random points in a bounding-box-enlarged sampling box and compared to `BRepClass3d_SolidClassifier(occt).State(p, tol)` (mapping `TopAbs_IN→IN`, `TopAbs_OUT→OUT`, `TopAbs_ON→ON`)
- THEN there SHALL be ZERO crisp `IN`↔`OUT` disagreements; a point one side calls `ON`/`UNKNOWN` and the other places within the ON band SHALL count as agreement (the tolerance band, NOT a weakened tolerance); and the run SHALL be recorded as `N passed / 0 crisp-disagreements / k in-band-or-declined`

### Requirement: Point-in-freeform-solid membership self-verifies or DECLINES with a measured gap

The point-in-freeform-solid classifier SHALL be accepted as a landed capability ONLY
when BOTH mandatory gates pass with zero regression: (a) the HOST-ANALYTIC gate — on
a freeform-walled solid whose membership is analytically known, the native verdicts
match the closed-form truth on points comfortably away from the ON band with NO OCCT
linked; and (b) the SIM native-vs-OCCT gate — the native verdicts agree with
`BRepClass3d_SolidClassifier` on a batch of N random points within the tolerance
band, with ZERO crisp IN↔OUT disagreements. If EITHER gate cannot be met robustly —
the M0 mesh of the chosen fixture is not watertight enough, ray consensus is
unreliable away from the ON band, or crisp disagreements remain — the change SHALL
DECLINE HONESTLY: it SHALL record the MEASURED gap (which gate, how many of N
disagreed and by how much, or the residual open-edge count), SHALL leave the existing
analytic `classifyPoint` path UNCHANGED, and SHALL NOT land the classifier as a
passing capability. No tolerance SHALL be weakened to force a pass, OCCT SHALL remain
the oracle (never removed), and NO wrong classification SHALL be emitted at any point.
An honest decline with a specific measured blocker is a first-class, expected outcome.

#### Scenario: Both gates green with zero regression lands the classifier (success)

- GIVEN the host-analytic gate and the sim OCCT-parity gate for the single-freeform-face fixture
- WHEN both are run and the existing analytic `classifyPoint`/`recogniseCurvedSolid` and the tessellator are proven byte-identical vs `main`, with zero OCCT includes under `src/native/**` and no `cc_*` ABI change
- THEN the classifier SHALL be recorded as landed (first slice of B3), with the fixture, N, and the ON band documented in `openspec/MOAT-ROADMAP.md`

#### Scenario: An unreachable gate is reported as an honest decline, not a weakened pass (decline)

- GIVEN robust membership cannot be achieved for the trimmed-freeform fixture (e.g. the M0 mesh retains open edges, or crisp IN↔OUT disagreements against OCCT remain)
- WHEN the gates are evaluated
- THEN the change SHALL report the MEASURED gap (the failing gate and its quantified shortfall), SHALL leave the analytic `classifyPoint` path as-is, SHALL NOT weaken any tolerance, and SHALL NOT emit any wrong classification — the decline is the recorded outcome

### Requirement: Partition ONE trimmed freeform face along the M1 seam into two tiling sub-faces, or DECLINE

The native boolean library SHALL provide an OCCT-free, header-only face-split
subsystem (in `src/native/boolean/`, with an optional seam-clip helper in
`src/native/ssi/`) that partitions ONE trimmed freeform face along the M1-traced seam
into TWO genuinely-trimmed sub-faces. The subsystem SHALL take (1) a `Face` whose
`FaceSurface` is `Kind::BSpline` or `Kind::Bezier` bounded by a real outer-wire
`EDGE_LOOP`, and (2) the M1 `WLine` (`src/native/ssi/marching.h`) whose per-node
`(u1,v1)` params (surface A = this face) form the seam pcurve on the face's own
`(u,v)` domain. The subsystem SHALL:

- flatten the outer `EDGE_LOOP` to a UV boundary polygon at the SAME shared per-edge
  fractions the mesher uses (`FaceMesher::buildBoundaryLoops`), so a sub-face's
  boundary points are BIT-IDENTICAL to the parent's and to any neighbouring face's;
- clip the seam polyline to the trimmed UV domain, requiring — for this first slice —
  a CONVEX outer loop and exactly ONE seam chord that cleanly ENTERS through one
  boundary edge and EXITS through another (no tangency, no re-entry), with every
  interior seam node strictly inside the outer loop;
- partition the domain into two closed UV sub-loops: L1 = one boundary arc between the
  entry and exit crossings spliced with the interior seam chord; L2 = the
  complementary boundary arc spliced with the SAME seam chord in the opposite
  traversal order, so the seam is the EXACT shared boundary of both;
- rebuild each sub-loop as a genuinely-trimmed sub-`Face` over the SAME `FaceSurface`
  node — parent boundary edges carried verbatim (their 3D `EdgeCurve` and `PCurve`
  preserved; an edge crossed mid-span split at the crossing fraction), plus ONE new
  shared seam edge whose UV `PCurve` is the `(u1,v1)` polyline and whose 3D
  `EdgeCurve` is the `WLine`'s fitted B-spline (or the polyline), added to both wires
  with opposite orientation.

The subsystem SHALL consume the landed M0 mesher and M1 tracer WITHOUT modifying
either (the tessellator and marcher remain byte-identical), SHALL keep
`src/native/**` OCCT-free (0 OCCT includes), and SHALL NOT change any `cc_*`
signature or POD layout (additive internal substrate). For any case outside this
first slice — a non-convex outer loop, zero or two-or-more boundary crossings, a
tangential/grazing seam, seam re-entry, a branch point, or a hole in the trimmed
domain — the subsystem SHALL return a NULL split (DECLINE) with the measured blocker,
and SHALL NEVER emit a partial, overlapping, or leaky split.

#### Scenario: A single seam chord across a convex trimmed freeform face yields two sub-faces that tile the original (host, analytic — no OCCT)

- GIVEN a trimmed freeform `Face` (`Kind::BSpline`/`Bezier`, convex outer `EDGE_LOOP`) and an M1 `WLine` whose `(u1,v1)` nodes form one chord that cleanly enters and exits the trimmed `(u,v)` domain, on a host build with NO OCCT linked
- WHEN the face-split subsystem partitions the domain along the seam
- THEN it SHALL emit exactly two genuinely-trimmed sub-`Face`s whose UV areas SUM to the parent outer loop's UV area within a scale-relative tolerance, whose shared seam boundary is the IDENTICAL `(u1,v1)` node sequence traversed in opposite order (bit-identical, no gap), and whose interiors do not overlap — the two sub-faces TILE the original

#### Scenario: A seam that does not cleanly cross the trimmed domain DECLINES with a measured blocker (host)

- GIVEN a trimmed freeform `Face` and an M1 `WLine` whose seam does NOT cleanly cross the convex trimmed domain — it produces zero or three-or-more boundary crossings, grazes the boundary tangentially, re-enters the domain, or leaves a degenerate sub-region
- WHEN the face-split subsystem evaluates its self-verify gate
- THEN it SHALL return a NULL split (DECLINE), report the measured blocker (crossing count, degenerate UV area, or area-sum gap), and SHALL NOT emit any partial, overlapping, or leaky sub-face — the honest decline is a first-class outcome

### Requirement: Mandatory freeform face-split self-verify gate (tile-or-decline)

Before returning two sub-faces, the freeform face-split subsystem SHALL run a
mandatory, OCCT-free self-verify gate and SHALL accept the split ONLY when EVERY
check passes; on ANY failure it SHALL DISCARD the candidate and return a NULL split.
The gate SHALL verify: (1) the seam CLEANLY crosses — exactly one entry and one exit
boundary crossing, with all interior seam nodes strictly inside the outer loop; (2)
each sub-region is NON-DEGENERATE — a simple UV loop with no self-intersection and
|signed UV area| above a scale-relative floor; (3) the seam is the EXACT shared
boundary — L1 and L2 reference the identical seam UV node sequence in opposite order
with no boundary gap beyond snap tolerance; (4) TILING — the two sub-face UV areas
sum to the parent outer loop's UV area within a scale-relative tolerance, with no
overlap. No tolerance SHALL be weakened to admit a case, and no wrong or leaky split
SHALL ever be emitted in place of an honest decline.

#### Scenario: A candidate whose sub-face areas do not sum to the parent is discarded (host)

- GIVEN a partition candidate whose two sub-face UV areas do NOT sum to the parent outer loop's UV area within the scale-relative tolerance (a gap or overlap)
- WHEN the self-verify gate evaluates the tiling identity
- THEN the gate SHALL reject the candidate AND the subsystem SHALL return a NULL split (no sub-faces emitted), reporting the measured area-sum gap

#### Scenario: A candidate with a degenerate or self-intersecting sub-region is discarded (host)

- GIVEN a partition candidate in which one sub-loop is self-intersecting or has |signed UV area| below the scale-relative floor
- WHEN the self-verify gate evaluates the non-degenerate sub-region check
- THEN the gate SHALL reject the candidate AND the subsystem SHALL return a NULL split, reporting the degenerate sub-region as the blocker

### Requirement: Freeform sub-faces mesh watertight and reproduce the parent's area, cross-checked against OCCT (simulator)

The two sub-faces produced by the face-split subsystem SHALL, on a booted iOS
simulator with OCCT linked, each mesh WATERTIGHT through the LANDED M0
`FaceMesher::trimmedFreeformMesh` path with NO modification to the tessellator, and
their boundary points along the shared seam SHALL be BIT-IDENTICAL across the two
sub-faces (the weld contract). The union of the two sub-face meshes SHALL reproduce
the parent face's surface area and topology within tolerance, cross-checked against
the OCCT oracle: each sub-face's area SHALL match its OCCT `BRepMesh_IncrementalMesh`
area within a scale-relative tolerance, and — where an OCCT freeform face-split
reference is available — the native seam split SHALL agree with the OCCT split. OCCT
SHALL be referenced ONLY in the simulator proof harness (`src/engine/occt`); the
face-split subsystem and its inputs SHALL remain OCCT-free.

#### Scenario: Each freeform sub-face meshes watertight and the union matches the parent area vs OCCT (sim, parity)

- GIVEN the two sub-`Face`s emitted for the clean single-chord split, meshed on a booted simulator with OCCT linked
- WHEN each sub-face is meshed via the M0 `FaceMesher` and its area is compared to its OCCT `BRepMesh` area, and the two sub-face meshes are unioned along the shared seam
- THEN each sub-face mesh SHALL be watertight with shared-seam boundary points bit-identical across the two sub-faces, each sub-face's native area SHALL match its OCCT `BRepMesh` area within a scale-relative tolerance, and the unioned area SHALL reproduce the parent face's area (the tessellator is byte-identical — consumed, not modified)

### Requirement: Freeform operand descriptor + recogniseFreeformSolid admission gate (admit one reachable freeform operand or DECLINE)

The native boolean library SHALL provide an OCCT-free, header-only freeform operand
DESCRIPTOR and admission gate (in `src/native/boolean/freeform_operand.h`) — an
**additive sibling** to the analytic `recogniseCurvedSolid` (`ssi_boolean.h`), which
SHALL remain byte-identical. The descriptor SHALL be a data model of a
freeform-faced solid — a `FreeformOperand` carrying, for each boundary face, its role
(`Freeform` for a `Kind::BSpline`/`Kind::Bezier` wall; `AnalyticHalfSpace` for a
`Plane` — or `Cylinder`/`Sphere`/`Cone` — cap/half-space), its WORLD-PLACED
`FaceSurface` (surface + face location folded, as `worldFrame` does), its genuinely
trimmed outer `EDGE_LOOP`, and its orientation-resolved outward normal; plus the
operand's world-space axis-aligned bounding box, the indices of the freeform walls
and of the analytic half-spaces, a closed-2-manifold/watertight flag, and the operand
`Shape` itself. The descriptor SHALL expose EXACTLY what the M2 assembly reads: the
freeform `Face`(s) for the B2 face-split, the analytic half-spaces, the world AABB for
the B3 membership ON-band, and the operand `Shape` for the M0 `SolidMesher`.

`recogniseFreeformSolid(const topology::Shape&)` SHALL return
`std::optional<FreeformOperand>` and SHALL ADMIT one reachable freeform operand: a
non-null `Solid` with a single shell, at least ONE genuinely-trimmed freeform face (a
real pcurve-bounded `EDGE_LOOP` that is NOT the full parametric rectangle and carries
no inner hole loop), every remaining face an admissible analytic cap/half-space, whose
boundary is CLOSED and WATERTIGHT (every edge shared by EXACTLY two faces) and whose
faces round-trip faithfully (each recorded `surface`/kind/trim equals the face's own).
It SHALL return `std::nullopt` — an HONEST DECLINE carrying the measured blocker — for
a non-`Solid`, an open or leaky boundary, a multi-shell operand, a freeform face that
is bare-periodic (a `VERTEX_LOOP` — the analytic/structured paths own it) or holed, a
non-admissible surface kind (e.g. `Torus`), or an operand with NO freeform face (the
analytic `recogniseCurvedSolid` and planar paths own those).

The descriptor and gate SHALL be strictly ADDITIVE: they SHALL NOT modify
`recogniseCurvedSolid` or `classifyPoint` (`ssi_boolean.h`), SHALL NOT modify the
consumed M0/M1/B2/B3 subsystems, SHALL introduce NO `cc_*` ABI surface or POD-layout
change, and SHALL add ZERO OCCT includes under `src/native/**`. No tolerance SHALL be
weakened to admit an operand, and a non-admissible operand SHALL DECLINE (`nullopt`)
rather than be forced through. This first slice targets the SIMPLEST reachable
operand — a single genuinely-trimmed freeform-wall solid with planar caps; richer
operands (multi-freeform-face, holed, freeform↔freeform) are explicitly deferred.

#### Scenario: A single-freeform-wall solid with planar caps is admitted and round-trips its faces/kinds/trims (host, analytic — no OCCT)

- GIVEN a native `Solid` whose boundary is one genuinely-trimmed `Kind::BSpline` wall (a real pcurve-bounded `EDGE_LOOP`) plus planar cap faces, with a KNOWN closed-form volume/area, built on the host with NO OCCT linked
- WHEN `recogniseFreeformSolid` is evaluated on it
- THEN it SHALL return a `FreeformOperand` in which every `OperandFace`'s world-placed `surface`/kind and trimmed outer loop equal the face's own within tolerance, the freeform wall is tagged `Freeform` and the caps `AnalyticHalfSpace`, each outward normal respects the face `Orientation`, the world AABB is tight, and the watertight flag is true — the descriptor faithfully captures the freeform solid

#### Scenario: A non-admissible operand DECLINES to nullopt with a measured blocker (host)

- GIVEN an operand outside the first-slice envelope — a non-`Solid`, an open/leaky boundary (an edge NOT shared by exactly two faces), a multi-shell solid, a bare-periodic (`VERTEX_LOOP`) or holed freeform face, a `Torus` face, or a purely-analytic solid with no freeform face — built on the host with NO OCCT
- WHEN `recogniseFreeformSolid` is evaluated on it
- THEN it SHALL return `std::nullopt` (an honest decline) identifying the measured blocker (the reason the caller logs before falling through to OCCT), and SHALL NOT fabricate a descriptor for an inadmissible operand

#### Scenario: The descriptor exposes exactly the handles the M2 verbs consume (host)

- GIVEN a `FreeformOperand` admitted by `recogniseFreeformSolid`
- WHEN the M2 assembly reads it
- THEN the freeform-wall `Face` it exposes SHALL be directly acceptable to B2 `splitFace`, the operand `Shape` it exposes SHALL be directly meshable by the M0 `SolidMesher::mesh`, and the world AABB it exposes SHALL be directly usable as the B3 `classifyPointInMesh` bbox — the descriptor is the single place the assembly reads faces/kinds/surfaces/trims/bounds/orientation

### Requirement: The freeform operand descriptor is strictly additive and preserves the analytic recognise path byte-identical

Adding the `FreeformOperand` descriptor and `recogniseFreeformSolid` gate SHALL leave
the analytic curved-boolean substrate byte-identical: the analytic
`recogniseCurvedSolid`, `classifyPoint`, and the S5-a curved-solid assembler
(`ssi_boolean.h`) SHALL be UNCHANGED, and the consumed M0 mesher
(`tessellate/face_mesher.h trimmedFreeformMesh`), M1 tracer (`ssi/marching.h WLine`),
B2 face-split (`boolean/face_split.h splitFace`), and B3 membership
(`boolean/freeform_membership.h classifyPointInMesh`) SHALL be consumed WITHOUT
modification. The addition SHALL introduce no `cc_*` ABI change and SHALL keep
`src/native/**` free of OCCT includes. The existing curved-boolean and freeform
substrate test suites SHALL pass with counts unchanged from the pre-change baseline.

#### Scenario: The analytic recogniser and the M2 substrate are byte-identical after the descriptor is added (host + sim)

- GIVEN the analytic `recogniseCurvedSolid`/`classifyPoint` and the M0/M1/B2/B3 subsystems before and after `freeform_operand.h` is added, together with the curved-boolean, face-split, freeform-membership, and SSI-marching suites
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN `recogniseCurvedSolid`/`classifyPoint` and the four consumed subsystems SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

### Requirement: Minimal end-to-end freeform↔analytic boolean assembly with mandatory self-verify, or HONEST DECLINE

As a stretch beyond B1, the native boolean library SHALL ATTEMPT a minimal end-to-end
freeform boolean for the SIMPLEST reachable case — a single freeform-wall operand
(admitted by `recogniseFreeformSolid`) CUT by, or COMMON with, ONE analytic PLANAR
half-space — composed ENTIRELY from the landed M2 verbs consumed unchanged: recognise
the operand[B1], trace the wall∩plane seam[M1 `WLine`], split the freeform wall along
the seam[B2 `splitFace`], classify each resulting fragment IN/OUT at its interior
point against the other operand's M0 boundary mesh[B3 `classifyPointInMesh`], select
the surviving fragments per the op's face-survival rule, and weld them watertight[M0
shared-edge weld]. Before returning, the assembler SHALL run the MANDATORY,
OCCT-free self-verify gate — the welded result SHALL be WATERTIGHT (every edge shared
by exactly two faces) AND its enclosed volume SHALL match the closed-form/set-algebra
value within a scale-relative tolerance — and SHALL DISCARD a result that fails
(returning a NULL `Shape` → OCCT).

The assembler SHALL DECLINE (return a NULL `Shape` → OCCT) whenever any verb declines
— `recogniseFreeformSolid` returns `nullopt`, the M1 seam is not one clean chord, B2
`splitFace` returns a `SplitDecline`, or a fragment's B3 verdict is `On`/`Unknown`
(near the ON band or an ambiguous ray consensus) — and SHALL NEVER emit a partial,
overlapping, leaky, or wrong-volume solid, nor guess a fragment's membership. If the
minimal assembly is NOT robustly reachable, this change SHALL LAND B1 (the descriptor
+ gate) and HONEST-DECLINE the assembly, recording which verb declined and the
measured remaining gap. No tolerance SHALL be weakened to force an assembly; OCCT
SHALL remain the oracle and the fallback.

#### Scenario: A single-freeform-face plate CUT by a planar half-space assembles watertight with the correct volume (host, analytic — no OCCT)

- GIVEN a single-freeform-wall operand admitted by `recogniseFreeformSolid` and one analytic planar half-space whose plane cleanly crosses the freeform wall, on a host build with NO OCCT, such that the cut region's volume is KNOWN in closed form
- WHEN the minimal assembler traces the seam[M1], splits the wall[B2], classifies the fragments[B3], welds the survivors[M0], and runs the mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces) whose enclosed volume equals the closed-form CUT value within the scale-relative tolerance — the four verbs compose into a correct freeform boolean

#### Scenario: A minimal assembly that cannot be completed robustly DECLINES to OCCT, and B1 still lands (host)

- GIVEN a freeform↔analytic case in which a verb declines (no clean single-chord seam, a B2 `SplitDecline`, or a fragment centroid that B3 resolves to `On`/`Unknown`) or the welded result fails the watertight/volume self-verify
- WHEN the minimal assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave B1 (the descriptor + gate) landed and the consumed subsystems byte-identical — the honest decline is a first-class outcome

### Requirement: Minimal freeform↔analytic boolean parity with OCCT BRepAlgoAPI_{Cut,Common} (simulator)

When the minimal freeform↔analytic assembly assembles, it SHALL, on a booted iOS
simulator with OCCT linked, match the OCCT oracle: the native result's enclosed
volume and surface area SHALL match `BRepAlgoAPI_Cut` / `BRepAlgoAPI_Common` (measured
via `BRepGProp`) within a scale-relative tolerance, the native result SHALL be
watertight, and a batch of query points SHALL agree with `BRepClass3d_SolidClassifier`
on the native result with ZERO crisp IN↔OUT disagreements. The engine's mandatory
watertight + volume self-verify SHALL DISCARD any native result that is not watertight
or whose volume does not match the OCCT oracle, falling through to OCCT so that a
wrong/leaky solid is NEVER emitted. OCCT SHALL be referenced ONLY in the simulator
proof harness (`src/engine/occt`); the assembler and its inputs SHALL remain
OCCT-free.

#### Scenario: The native freeform CUT/COMMON matches BRepAlgoAPI within tolerance (sim, parity)

- GIVEN the freeform↔analytic-plane operands built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked, for `op ∈ {Cut, Common}`
- WHEN the native minimal assembler produces a result and it is compared against `BRepAlgoAPI_{Cut,Common}` via `BRepGProp` (volume/area) and against `BRepClass3d_SolidClassifier` on a point batch
- THEN the native volume/area SHALL match the OCCT oracle within the scale-relative tolerance, the native result SHALL be watertight, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume native result is discarded to OCCT (sim)

- GIVEN a native minimal-assembly result that is not watertight or whose volume does not match the OCCT oracle within tolerance
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_{Cut,Common}` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path

### Requirement: B4 analytic-face split and cross-section cap-synthesis weld verb for a planar half-space cut

The native boolean library SHALL provide an OCCT-free, header-only **B4 verb** (in
`src/native/boolean/half_space_cut.h`) that, given a `FreeformOperand` admitted by B1,
a cutting `Plane` `P`, a keep side, and the B2 seam chord for the operand's freeform
wall, SPLITS each analytic (planar) boundary face `P` crosses, SYNTHESISES one
cross-section cap on `P`, and WELDS the survivors into a watertight `Solid` — or
DECLINES (returns a NULL `Shape`) with a measured blocker. It SHALL be a strictly
ADDITIVE sibling: it SHALL NOT modify B1/B2/B3, the M0 mesher, the M1 tracer, or the
analytic `recogniseCurvedSolid`/`classifyPoint`.

- **Analytic-face split.** For each `AnalyticHalfSpace` face, the verb SHALL keep the
  face WHOLE when its outer loop lies entirely on the keep side, DROP it when entirely
  on the discard side, and otherwise SPLIT it along its exact `Face ∩ P` line: it SHALL
  require EXACTLY TWO boundary-edge crossings, record each crossing as (edge id, curve
  parameter), and rebuild the keep-side sub-face over the parent `Plane` surface with
  the parent's per-edge pcurves carried verbatim. A crossing that is tangent, at a
  vertex, or that does not yield exactly two crossings SHALL DECLINE.
- **Cross-section cap synthesis.** The verb SHALL assemble ONE ordered closed loop on
  `P` from the B2 seam chord spliced to the straight crossing segments of the split
  analytic faces, SHALL verify the loop is SIMPLE (no non-adjacent-edge intersection)
  and CLOSED, and SHALL build ONE `Plane`-surface cap face bounded by that loop
  oriented OUTWARD (cap normal facing the discard side). The seam edge SHALL be built
  ONCE and SHARED bit-exactly with the kept freeform sub-face; each straight cap edge
  SHALL be SHARED with its split analytic face's crossing edge. A non-simple or open
  section loop SHALL DECLINE.
- **Weld.** The verb SHALL weld the kept freeform sub-face, the kept analytic sub-faces
  and whole faces, and the section cap into one shell → `Solid`, welding coincident
  corners to shared vertices (the `assemble.h` `VertexPool` discipline). A shell that
  cannot close SHALL DECLINE.

The verb SHALL remain OCCT-free (zero OCCT includes), SHALL introduce no `cc_*` ABI
surface, and SHALL NOT weaken any tolerance to force a result. Its per-function
cognitive complexity SHALL stay within the backend band (helpers for per-face split,
section-loop assembly, and weld).

#### Scenario: The plane splits the crossed analytic faces and caps the section (host, no OCCT)

- GIVEN the bowl-lidded convex-quad prism operand admitted by `recogniseFreeformSolid`, the cutting plane `P: x = 0` keeping `x ≤ 0`, and the B2 seam chord for the bowl top, built on the host with NO OCCT
- WHEN the B4 verb runs
- THEN it SHALL split exactly the two vertical walls and the bottom face `P` crosses (each into a keep and a discard sub-face along `Face ∩ P`, with the two crossings recorded as (edge, param)), keep the wall entirely on the keep side and drop the wall entirely on the discard side whole, and synthesise ONE outward-oriented `Plane` section cap whose boundary is the B2 seam chord spliced to the three straight crossing segments — a simple, closed loop

#### Scenario: A degenerate cut configuration DECLINES to a NULL Shape (host)

- GIVEN a configuration outside the verb's envelope — a plane that misses the operand, a crossing that is tangent or lands on a vertex, an analytic face with other than two crossings, a section loop that is not simple or not closed, or a weld that cannot close — built on the host with NO OCCT
- WHEN the B4 verb runs
- THEN it SHALL return a NULL `Shape` identifying the measured blocker, SHALL NOT emit a partial, overlapping, or leaky solid, and SHALL NOT weaken a tolerance to force a result

#### Scenario: The section cap and freeform top share the seam edge bit-exactly (host)

- GIVEN a B4 result on the bowl-lidded prism CUT
- WHEN its shell is inspected
- THEN the seam edge SHALL be a SINGLE shared edge between the kept freeform sub-face and the section cap (built once, opposite orientation), each straight cap edge SHALL be shared with its split analytic face's crossing edge, and the shell SHALL be watertight (every edge shared by exactly two faces)

### Requirement: First native freeform↔analytic-half-space CUT composed from B1→M1→B2→B4→B3→M0 with mandatory self-verify

The native boolean library SHALL compute the FIRST end-to-end freeform boolean — a
single-freeform-wall operand (the bowl-lidded convex-quad prism) CUT by ONE analytic
PLANAR half-space — composed ENTIRELY from the landed M2 verbs consumed UNCHANGED,
guarded by a mandatory OCCT-free self-verify. The assembler SHALL: (1) admit the
operand with `recogniseFreeformSolid`[B1]; (2) trace the freeform wall ∩ plane seam as
a single M1 `WLine`[M1] with `points.size() ≥ 2` and status `Closed`/`BoundaryExit`;
(3) split the freeform wall along the seam with `splitFace`[B2]; (4) split the crossed
analytic faces and synthesise the cross-section cap with the B4 verb; (5) CONFIRM the
keep-side selection with `classifyPointInMesh`[B3] against the PRE-cut operand's M0
boundary mesh — every kept sub-face's interior centroid SHALL classify `In` AND lie on
the keep side of the plane; (6) weld the survivors watertight[M0]. Before returning,
the assembler SHALL run the MANDATORY self-verify: the welded result SHALL be
WATERTIGHT (every edge shared by exactly two faces) AND its enclosed volume SHALL match
the closed-form CUT value `∫∫_{Q ∩ keep} (h0 + a·(x² + y²)) dA` within a scale-relative
tolerance sized to the M0 deflection, and a result that FAILS SHALL be DISCARDED
(NULL `Shape` → OCCT).

The assembler SHALL DECLINE (NULL `Shape` → OCCT) whenever any verb declines — B1
`nullopt`, an M1 seam that is not one clean chord, a B2 `SplitDecline`, a B4 decline,
or a B3 `On`/`Unknown` verdict or a wrong-side survivor — and SHALL NEVER emit a
partial, overlapping, leaky, or wrong-volume solid, nor GUESS a fragment's membership.
If the end-to-end CUT is not robustly reachable this wave, the change SHALL LAND the B4
verb proven in isolation and HONEST-DECLINE the assembly, recording which verb declined
and the measured remaining gap. The consumed B1/B2/B3/M0/M1 subsystems and the analytic
`recogniseCurvedSolid`/`classifyPoint` paths SHALL remain byte-identical; no tolerance
SHALL be weakened to force an assembly; OCCT SHALL remain the oracle and the fallback.

#### Scenario: The bowl-lidded quad prism CUT by a plane assembles watertight with the closed-form volume (host, analytic — no OCCT)

- GIVEN the bowl-lidded convex-quad prism operand and the cutting plane `x = 0` keeping `x ≤ 0`, whose clipped-region volume `∫∫_{Q ∩ {x≤0}} (h0 + a·(x² + y²)) dA` is KNOWN in closed form, on a host build with NO OCCT linked
- WHEN the assembler recognises the operand[B1], traces the seam[M1], splits the wall[B2], runs the analytic-face split + section-cap synthesis[B4], confirms the survivors[B3], welds[M0], and runs the mandatory self-verify
- THEN it SHALL return a watertight `Solid` (every edge shared by exactly two faces) whose enclosed volume equals the closed-form CUT value within the scale-relative tolerance — the five verbs compose into the first correct freeform boolean

#### Scenario: A composition that cannot complete robustly DECLINES to OCCT, B4 still landing (host)

- GIVEN a freeform↔analytic CUT case in which a verb declines (no clean single-chord M1 seam, a B2 `SplitDecline`, a B4 decline, or a survivor centroid B3 resolves to `On`/`Unknown` or on the wrong side) OR the welded result fails the watertight/volume self-verify
- WHEN the assembler is evaluated
- THEN it SHALL return a NULL `Shape` (→ OCCT) reporting which verb declined and the measured gap, SHALL NOT emit any partial/overlapping/leaky/wrong-volume solid, SHALL NOT guess any fragment's membership, and SHALL leave the B4 verb landed (proven by its isolated host tests) and the consumed subsystems byte-identical — the honest decline is a first-class outcome

### Requirement: First freeform CUT parity with OCCT BRepAlgoAPI_Cut through the facade (simulator gate)

When the first freeform↔analytic CUT assembles, it SHALL be verified on a booted iOS
simulator (OCCT linked) against the OCCT oracle: the same operand and cutting plane
SHALL be built BOTH natively and as OCCT shapes, and the native
`freeformHalfSpaceCut` result SHALL match `BRepAlgoAPI_Cut` (measured via `BRepGProp`)
on ENCLOSED VOLUME and SURFACE AREA within a scale-relative tolerance, SHALL be
WATERTIGHT (closed shell), SHALL agree on topology counts, and a batch of query points
SHALL agree with `BRepClass3d_SolidClassifier` on the native result with ZERO crisp
IN↔OUT disagreements. The engine's mandatory watertight + volume self-verify SHALL
DISCARD any native result that is not watertight or whose volume does not match the
OCCT oracle, falling through to `BRepAlgoAPI_Cut` so a wrong/leaky solid is NEVER
emitted. OCCT SHALL be referenced ONLY in the simulator proof harness / `src/engine/occt`;
the assembler and its inputs SHALL remain OCCT-free. The change SHALL NOT add any
`cc_*` entry point, and the count of any deferred cases SHALL be reported, not hidden.

#### Scenario: The native freeform CUT matches BRepAlgoAPI_Cut within tolerance (sim, parity)

- GIVEN the bowl-lidded convex-quad prism operand and the cutting plane built BOTH natively and as OCCT shapes on a booted simulator with OCCT linked
- WHEN the native `freeformHalfSpaceCut` produces a result and it is compared against `BRepAlgoAPI_Cut` via `BRepGProp` (volume/area) and topology counts, and against `BRepClass3d_SolidClassifier` on a query-point batch
- THEN the native volume/area SHALL match the OCCT oracle within the scale-relative tolerance, the native result SHALL be watertight with agreeing topology counts, and there SHALL be ZERO crisp IN↔OUT classification disagreements

#### Scenario: A non-watertight or wrong-volume native result is discarded to OCCT (sim)

- GIVEN a native first-CUT result that is deliberately perturbed to be non-watertight or wrong-volume on a booted simulator with OCCT linked
- WHEN the engine's mandatory watertight + volume self-verify is evaluated
- THEN the native result SHALL be DISCARDED and the boolean SHALL fall through to `BRepAlgoAPI_Cut` (OCCT), no wrong/leaky solid SHALL be emitted, and no tolerance SHALL be weakened to force the native path

### Requirement: The first freeform boolean is strictly additive and defers the B2 smooth-trim generalisation

Landing the B4 verb and the first freeform↔analytic CUT SHALL leave the consumed M2
substrate byte-identical: B1 `recogniseFreeformSolid` (`boolean/freeform_operand.h`),
B2 `splitFace` (`boolean/face_split.h`), B3 `classifyPointInMesh`
(`boolean/freeform_membership.h`), the M0 `SolidMesher` (`tessellate/solid_mesher.h`),
the M1 `WLine` tracer (`ssi/marching.h`), and the analytic `recogniseCurvedSolid`/
`classifyPoint` (`ssi_boolean.h`) SHALL be UNCHANGED and consumed WITHOUT modification.
The addition SHALL introduce no `cc_*` ABI change and SHALL keep `src/native/**` free of
OCCT includes. The change SHALL EXPLICITLY DEFER the **B2 smooth-trim (closed / circular
freeform wall) generalisation** (B1's blocker (i)): the bowl-lidded convex-quad prism
operand SIDESTEPS it by presenting a freeform wall whose outer loop is the convex
straight-edged quadrilateral B2 already splits, and NO closed/circular-wall split path
SHALL be written or stubbed in this change. The existing curved-boolean and freeform
substrate test suites SHALL pass with counts unchanged from the pre-change baseline.

#### Scenario: The M2 substrate and analytic paths are byte-identical after B4 is added (host + sim)

- GIVEN the B1/B2/B3/M0/M1 headers and the analytic `recogniseCurvedSolid`/`classifyPoint`, together with the curved-boolean, face-split, freeform-membership, and SSI-marching suites, before and after `half_space_cut.h` is added
- WHEN each is exercised at the same inputs and compared against the pre-change baseline
- THEN the six consumed subsystems SHALL be byte-identical (unchanged source, unchanged suite counts), there SHALL be zero OCCT includes under `src/native/**`, and no `cc_*` signature or POD layout SHALL have changed

#### Scenario: The B2 smooth-trim generalisation is deferred, not stubbed (host)

- GIVEN this change scoped to the bowl-lidded convex-quad prism operand (whose freeform wall has a convex straight-edged outer loop) and the B4 verb
- WHEN the change is inspected for closed/circular-freeform-wall handling
- THEN it SHALL contain NO closed/circular-wall split path (stubbed or otherwise), the operand SHALL sidestep blocker (i) within B2's shipped envelope, and the B2 smooth-trim generalisation SHALL be recorded as the deferred NEXT enabler with its measured blocker — not faked or partially implemented


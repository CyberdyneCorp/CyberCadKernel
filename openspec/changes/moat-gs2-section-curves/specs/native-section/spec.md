# native-section

## ADDED Requirements

### Requirement: Section curves of a plane cutting a solid, OCCT-free, on both the plane and the solid faces

The library SHALL compute the **section curves** where a cut plane intersects a
solid — the ordered set of closed loops that lie on BOTH the cut plane AND the
solid's boundary faces — with **no OCCT** (`src/native/section/**` includes only
`src/native/{math,ssi,topology,boolean}` headers; zero OCCT includes) and
header-only, in namespace `cybercad::native::section`, consuming the landed M1
SSI (`ssi::intersect_surfaces` closed-form plane∩{plane,cylinder,cone,sphere};
`ssi::marching` WLine for plane∩freeform) and the topology graph
(`topology::surfaceOf` / `pcurveOf` / face-loop iteration) READ-ONLY.

For each face of the solid the service SHALL intersect the face's surface with the
cut plane — the **analytic** closed-form conic for a `Plane`/`Cylinder`/`Cone`/
`Sphere` face, the **M1 marched** WLine polyline for a `BSpline`/`Bezier` face —
and CLIP the resulting curve to the face's trim region (inside its outer loop and
outside its holes, via the face pcurves) so that only the on-face arcs survive.
Every emitted section-curve sample SHALL satisfy the correctness invariant: it
lies on the cut plane (`|n·(P−O)| ≤ tol`) AND on the source face's surface
(point-on-surface residual ≤ tol). The service SHALL NOT modify `src/native/
boolean/**` or `src/native/ssi/**`; it consumes them.

#### Scenario: Box planar section is a closed rectangle on the plane and the faces (host, no OCCT)

- GIVEN an axis-aligned box of extents `w × h × d` and a cut plane parallel to a
  face pair (normal `+Z`, through the box interior)
- WHEN `sectionPlane(box, plane, wantCap=false)` is called
- THEN the result SHALL contain EXACTLY 1 closed loop AND every loop point SHALL
  lie on the cut plane within `1e-9` AND on a box face within `1e-9`

#### Scenario: Cylinder cross-section perpendicular to the axis is a closed circle (host, no OCCT)

- GIVEN a cylinder of radius `R` and height `H` and a cut plane perpendicular to
  its axis through the interior
- WHEN `sectionPlane(cylinder, plane, wantCap=false)` is called
- THEN the result SHALL contain EXACTLY 1 closed loop whose points lie on the
  plane AND on the cylinder's lateral surface within `1e-9`

### Requirement: Section edges assemble into closed, correctly oriented loops by shared endpoints

The service SHALL assemble the collected per-face section edges into closed loops
by ordering them on SHARED ENDPOINTS — endpoints deduplicated within the native
linear tolerance, preferring the topology shared-vertex identity when two edges
come from faces meeting at a shared model edge — walking each edge chain until it
returns to its start (a CLOSED loop). Each closed loop SHALL be oriented
consistently about the cut-plane normal (CCW positive) so that an outer loop and
its holes carry opposite signed area. The service SHALL emit a loop ONLY when it
is closed within tolerance; an edge chain that does not close, or a vertex whose
continuation is ambiguous, SHALL cause an HONEST DECLINE rather than an open or
guessed loop.

#### Scenario: A holed section assembles into an outer loop plus an inner hole loop (host, no OCCT)

- GIVEN a hollow square prism (an outer box with a smaller coaxial box cavity) and
  a cut plane perpendicular to the prism axis through both walls
- WHEN `sectionPlane(prism, plane, wantCap=false)` is called
- THEN the result SHALL contain EXACTLY 2 closed loops AND the inner loop's signed
  area (about the plane normal) SHALL have the OPPOSITE sign to the outer loop's

#### Scenario: A section whose edges do not close is declined, not emitted open (host, no OCCT)

- GIVEN a configuration whose collected section edges cannot be walked into a
  closed cycle within tolerance
- WHEN `sectionPlane(...)` is called
- THEN the service SHALL return a typed DECLINE (`NotClosed`) AND SHALL NOT emit
  any loop

### Requirement: Optional capped planar section face with an enclosed area matching the closed form

When a cap is requested the service SHALL synthesize the planar cap face bounded
by the assembled section loops (outer loop minus its hole loops), reusing the
landed B4 cross-section-cap synthesis path, and SHALL return its enclosed AREA
computed by the in-plane polygon (shoelace in the cut-plane basis). For the
closed-form fixtures the returned area SHALL equal the analytic value: a **box**
planar section = the rectangle `w·h`; a **cylinder axial** section (plane
containing the axis) = the rectangle `2R·H`; a **cylinder cross**-section ⟂ axis
= the circle `πR²`; a **sphere great-circle** section = `πR²`. A section requiring
nesting beyond one outer loop and its direct holes, or a self-intersecting
section, SHALL DECLINE rather than emit a mis-nested or wrong-area cap.

#### Scenario: Cylinder cross-section cap area equals πR² (host, no OCCT)

- GIVEN a cylinder of radius `R` and a cut plane perpendicular to its axis through
  the interior
- WHEN `sectionPlane(cylinder, plane, wantCap=true)` is called
- THEN the returned `cappedArea` SHALL equal `π·R²` within `1e-6` relative

#### Scenario: Cylinder axial section cap area equals 2R·H (host, no OCCT)

- GIVEN a cylinder of radius `R` and height `H` and a cut plane CONTAINING the
  cylinder axis
- WHEN `sectionPlane(cylinder, plane, wantCap=true)` is called
- THEN the result SHALL contain 1 closed rectangular loop AND `cappedArea` SHALL
  equal `2·R·H` within `1e-6` relative

### Requirement: Freeform face section via the landed M1 marcher

The service SHALL section a `BSpline`/`Bezier` face by tracing the plane∩face
intersection with the landed M1 SSI marching-line tracer, clipping the resulting
WLine polyline to the face trim, and folding those samples into the loop assembly
exactly like an analytic arc. It SHALL accept the section ONLY when the marcher
returns a `Closed` / `BoundaryExit` walk whose samples all satisfy the on-plane
and on-face invariant; a `NearTangent` or non-converged marcher stop SHALL cause
an HONEST DECLINE of the whole section.

#### Scenario: The landed bowl-lidded convex-quad prism sections into a closed loop on both surfaces (host, no OCCT)

- GIVEN the landed bowl-lidded convex-quad prism (its lid is a freeform Bézier
  face) and a cut plane that crosses the freeform lid and the straight walls
- WHEN `sectionPlane(prism, plane, wantCap=false)` is called
- THEN the result SHALL contain a closed loop AND every loop point SHALL lie on
  the cut plane AND on a face of the prism within the fixed section tolerance

### Requirement: Honest decline for tangent, non-closing, and non-robust-freeform sections

The service SHALL return an HONEST, typed DECLINE — rather than emit a wrong,
open, or self-crossing section — for any configuration it cannot robustly section.
The DECLINED cases SHALL include: a cut plane **TANGENT** to a curved face (the
plane∩face result degenerates to a point / a double line — no proper loop); a
section that **does not close** into a cycle within tolerance; a **freeform** face
whose plane-intersection the M1 marcher cannot trace robustly (`NearTangent` /
non-convergent corrector); and a **torus** face (plane∩torus is a planar quartic
outside this slice). A decline SHALL NEVER be silently converted into a
possibly-wrong section, and NO tolerance SHALL be weakened to force a result.

#### Scenario: A plane tangent to a cylinder is declined, not sectioned (host, no OCCT)

- GIVEN a cylinder of radius `R` and a cut plane at distance EXACTLY `R` from the
  axis and parallel to it (tangent to the lateral surface along one ruling)
- WHEN `sectionPlane(cylinder, plane, ...)` is called
- THEN the service SHALL return a typed DECLINE (tangent) AND SHALL NOT emit a
  loop

#### Scenario: A torus face is declined (host, no OCCT)

- GIVEN a solid one of whose faces is a `Torus` surface and a general cut plane
- WHEN `sectionPlane(...)` is called
- THEN the service SHALL return a typed DECLINE rather than an approximate section
  of the torus face

### Requirement: Additive cc_section_plane facade accessor, ABI-stable, OCCT as the oracle

The library SHALL expose the section-curves service through a NEW additive plain-C
facade accessor `cc_section_plane` (with a `CCSection` / `CCSectionLoop` POD
result and `cc_section_free`), returning the ordered section loops (per-loop 3D
polyline points + a closed flag), the loop count, the capped-section area, and the
total section-edge length. The addition SHALL be ADDITIVE-ONLY: no existing `cc_*`
signature SHALL change (verified by diff — only the new struct + prototypes
appended). Under the OCCT engine build the SAME accessor SHALL return the
`BRepAlgoAPI_Section` result (+ `BRepGProp` for the cap area) so it is the sim
verification oracle; `src/native/**` SHALL remain OCCT-free. On a native DECLINE
the accessor SHALL return an empty `CCSection` (loops null, loopCount 0) with
`cc_last_error` set — never a partial section.

#### Scenario: The new accessor is additive and returns the box section loop (host)

- GIVEN a host app built against the existing `cc_*` facade
- WHEN it is rebuilt against the version adding `cc_section_plane`
- THEN every previously-built caller SHALL compile unchanged AND
  `cc_section_plane` on a box with an interior parallel plane SHALL return a
  `CCSection` with `loopCount == 1` and `loops[0].closed == 1`

#### Scenario: A declined section returns an empty CCSection with an error (host)

- GIVEN a cut plane tangent to a cylinder face of `body`
- WHEN `cc_section_plane(body, ...)` is called
- THEN it SHALL return a `CCSection` with `loopCount == 0` and `loops == NULL` AND
  `cc_last_error()` SHALL be non-empty

### Requirement: Native section matches OCCT BRepAlgoAPI_Section on the simulator

On a booted iOS simulator the native section SHALL match the OCCT oracle on
identical geometry: the TOTAL section-edge length, the LOOP COUNT, and the
CLOSED-NESS of the section SHALL equal `BRepAlgoAPI_Section`, and the capped
section AREA SHALL equal `BRepGProp` on the section face, at FIXED tolerances that
SHALL NOT be widened. Any native-vs-OCCT discrepancy on an accepted section SHALL
be treated as a defect, not absorbed by loosening a tolerance.

#### Scenario: Box, cylinder cross-section, and cylinder axial sections match OCCT (sim)

- GIVEN a box, a cylinder cut ⟂ its axis, and the same cylinder cut through its
  axis, each reconstructed identically in native and OCCT
- WHEN the native `sectionPlane` result is compared with `BRepAlgoAPI_Section` and
  the capped area with `BRepGProp`
- THEN the total section-edge length, loop count, closed-ness, and capped area
  SHALL match within the fixed tolerances for all three fixtures

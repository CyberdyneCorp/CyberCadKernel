# Proposal — moat-gs2-section-curves (MOAT M-GS, GS2)

## Why

The app's `SectionGeometry` / `MeshSection` / `SectionCap` features need the
**section CURVES** where a plane cuts a solid — the ordered, closed loops that
lie on both the cut plane and the solid's boundary, plus an optional planar cap
face bounded by those loops. Today the only source is OCCT (`BRepAlgoAPI_Section`
for the edges, `BRepBuilderAPI_MakeFace` + `BRepGProp` for the cap area), which
keeps the LGPL engine on the critical path of a read-only sectioning feature and
blocks the native-only build.

This is a bounded **assembly-of-landed-parts** slice, not new geometry. Every
primitive the section needs is already landed and OCCT-verified in this worktree:

- `src/native/ssi/plane_conics.h` — closed-form `plane ∩ {plane, sphere,
  cylinder, cone}` intersectors that PROVABLY return native `Line` / `Circle` /
  `Ellipse` / `Parabola` / `Hyperbola` lying on BOTH surfaces (the S1 invariant),
  cross-checked vs OCCT `IntAna_QuadQuadGeo` on the simulator;
- `src/native/ssi/marching.h` (M1 S3 marcher) — the predictor-corrector WLine
  tracer that walks the intersection of a plane with a **freeform** face,
  producing an on-both-surfaces polyline + fitted B-spline, with an honest
  `NearTangent` / `BoundaryExit` stop;
- `src/native/topology/{shape.h,accessors.h,explore.h}` — the B-rep graph
  (READ-ONLY): `surfaceOf(face)`, `pcurveOf(edge, face)`, face trim loops, and
  the shared-edge/vertex identity relations that let section edges be assembled
  into loops by shared endpoints;
- `src/native/boolean/half_space_cut.h` (B4) — the ALREADY-PROVEN
  cross-section-cap synthesis weld verb: it assembles the section edges a cut
  plane produces into a closed planar cap and meshes it, verified watertight at
  its closed-form area at both gates. GS2 reuses the SAME loop-assembly + cap
  path, exposed as a standalone SERVICE that returns the curves (not the cut
  solid).

GS2 therefore does NOT touch `boolean/` or `ssi/` (a sibling M2-FUSE workflow
edits those — stay disjoint). It CONSUMES M1 + topology + the B4 cap machinery
read-only from a NEW module `src/native/section/`.

The hard cases are an **HONEST DECLINE**, never a wrong or open section: a plane
**tangent** to a curved face (the intersection degenerates to a point / a double
line — no proper loop), a section that **does not close** (the collected edges
do not assemble into a closed cycle within tolerance), and a **freeform** face
whose plane-intersection the M1 marcher cannot trace robustly (`NearTangent` /
non-convergent corrector). A measured decline is a first-class outcome; a wrong,
open, or self-crossing section is NEVER emitted, and no tolerance is weakened.

## What Changes

1. **A new header-only, OCCT-free module `src/native/section/`** in namespace
   `cybercad::native::section`, consuming M1 SSI + topology + the B4 cap synthesis
   read-only:
   - **`section_edges.h`** — for each face of the solid, intersect its surface
     with the cut plane. `Plane`/`Cylinder`/`Cone`/`Sphere` faces route to
     `ssi::plane_conics` (closed-form conic), clipped to the face's trim loops
     via `pcurveOf` so only the on-face arcs survive; a `BSpline`/`Bezier` face
     routes to the M1 `ssi::marching` WLine tracer, its polyline clipped to the
     face trim. Each surviving arc becomes a native section EDGE tagged with its
     source face and 3D endpoints.
   - **`loop_assembly.h`** — order the collected section edges into closed loops
     by SHARED ENDPOINTS (a next-edge-by-nearest-matching-endpoint walk within
     `kLinearEps`, reusing the topology shared-vertex identity where available),
     yielding one or more oriented, closed wires. An edge chain that does not
     close, or a vertex where the continuation is ambiguous, is a DECLINE.
   - **`section.h`** — the public `sectionPlane(solid, plane)` entry: run
     `section_edges` over every face, assemble loops, verify each loop lies on
     the cut plane AND on the solid's faces AND is closed, and OPTIONALLY
     synthesize the planar cap face (reusing the B4 cap path) with its enclosed
     area. Returns the ordered loops + per-loop closed-form/closed-ness metadata,
     or a typed DECLINE.

2. **One additive facade accessor `cc_section_plane`** (+ a `CCSection` POD
   result and `cc_section_free`) in `include/cybercadkernel/cc_kernel.h` /
   `src/facade/cc_kernel.cpp`, returning the section loops (polyline points per
   loop + closed flag + loop count) and the capped-section area. ADDITIVE-ONLY:
   no existing `cc_*` signature changes; the engine build returns the
   `BRepAlgoAPI_Section` oracle behind the same accessor. `src/native/**` stays
   OCCT-FREE (zero OCCT includes).

3. **Two verification gates** (the non-negotiable discipline):
   - **GATE A — HOST ANALYTIC (no OCCT):** each section loop's points lie on the
     cut plane (`|n·(P−O)| ≤ tol`) AND on the solid's faces (point-on-surface
     residual ≤ tol) AND the loop is closed; and the enclosed loop area matches a
     CLOSED-FORM value — a **box** planar section = a rectangle `w·h`, a
     **cylinder axial** section = a rectangle `2R·H`, a **cylinder cross**-section
     ⟂ axis = a circle `πR²`, a **sphere great-circle** section = `πR²`.
   - **GATE B — SIM native-vs-OCCT:** on a booted iOS simulator, match OCCT
     `BRepAlgoAPI_Section` on section-edge TOTAL LENGTH + LOOP COUNT +
     CLOSED-NESS, and the capped-section AREA vs `BRepGProp`, on identical
     geometry, at fixed (never-widened) tolerances.

Out of scope (honest declines, tracked): a plane **tangent** to a curved face; a
**non-closing** section; a **hard-freeform** face whose plane∩face SSI is not
robust (M1 `NearTangent`); multi-solid / self-intersecting sections. These
DECLINE at the service boundary (native returns a typed decline, the facade
returns an empty `CCSection` with `cc_last_error` set) and stay on OCCT.

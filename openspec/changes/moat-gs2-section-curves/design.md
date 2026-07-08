# Design — moat-gs2-section-curves (MOAT M-GS, GS2)

Expose one exact, OCCT-free SERVICE — the **section curves** of a plane cutting a
solid, with an optional planar cap — by ASSEMBLING landed, already-verified parts
(M1 SSI plane∩face intersection, the topology graph, the B4 cross-section-cap
weld verb) into a new `src/native/section/` module behind an additive
`cc_section_plane` accessor. No new geometry primitive is invented. OCCT
(`BRepAlgoAPI_Section`, `BRepGProp`) is the **oracle only**, in the sim harness —
never linked into `src/native/**`.

## 0. The substrate this consumes (verified in source, READ-ONLY)

- `ssi/plane_conics.h` — `intersectPlanePlane/Sphere/Cylinder/Cone` returning a
  native `IntersectionCurve` (Line/Circle/Ellipse/Parabola/Hyperbola) with the S1
  invariant "every point lies on BOTH surfaces"; `curve.h::value(t)` evaluator.
- `ssi/marching.h` (M1 S3) — `trace_intersection`-style WLine walk of plane∩face
  for a freeform face, returning an on-both-surfaces polyline + fitted B-spline
  with a typed stop (`Closed` / `BoundaryExit` / `NearTangent`).
- `ssi/dispatch.h` — `SurfaceKind {Plane,Cylinder,Cone,Sphere,Torus}` +
  `intersect_surfaces(A,B)` (the analytic router GS2 hands a `Plane` and each
  face surface to).
- `topology/accessors.h` — `surfaceOf(face)` (→ `FaceSurface`), `pcurveOf(edge,
  face)` (→ the 2D trim pcurve), `ParamRange`; `topology/explore.h` face/edge
  iteration; `shape.h` `isSame`/`isPartner` shared-node identity for loop welding.
- `boolean/half_space_cut.h` (B4) — the cross-section-cap synthesis + weld path
  already proven to assemble a cut plane's section edges into a closed planar cap
  and mesh it watertight at its closed-form area (both gates). GS2 calls the SAME
  loop-assembly + cap logic as a standalone service; it does NOT modify
  `boolean/`.

Nothing here needs a new numeric primitive — GS2 is routing + ordering + on-both
verification over the above.

## 1. Per-face plane∩face → section edges (`section/section_edges.h`)

For each face `F` of the solid, resolve `surfaceOf(F)`:

- **Analytic face** (`Plane`/`Cylinder`/`Cone`/`Sphere`): call
  `ssi::intersect_surfaces(cutPlane, faceSurface)`. On `Ok`, take the native
  conic branch(es). CLIP each conic to `F`'s trim: sample the conic, keep the
  parameter spans whose points are INSIDE `F`'s outer loop and OUTSIDE its holes
  (the pcurve even-odd test via `pcurveOf`), producing bounded on-face arcs.
- **Freeform face** (`BSpline`/`Bezier`): call the M1 marcher on
  `(cutPlane, faceSurface)`. A `Closed` / `BoundaryExit` WLine polyline is
  clipped to `F`'s trim the same way. A `NearTangent` / non-converged stop is a
  DECLINE for the whole section (never a partial arc).
- **Tangent / degenerate** (`intersect_surfaces` returns a `Point` / double-line
  tangency, or the analytic classifier flags tangency): DECLINE — the plane is
  tangent to the curved face and no proper section loop exists there.

Each surviving arc becomes a `SectionEdge { curveKind, samples[], face F,
endpointA, endpointB }`. Every sample is verified on-plane and on-face before it
is emitted (the correctness invariant; a failing sample DECLINES).

## 2. Loop assembly by shared endpoints (`section/loop_assembly.h`)

Build a graph whose nodes are section-edge endpoints deduplicated within
`kLinearEps` (preferring the topology shared-vertex identity when both edges came
from faces meeting at a shared model edge). Walk: start at any unused edge, repeat
"advance to the unique other edge sharing my current free endpoint" until the walk
returns to the start (CLOSED loop) or gets stuck / hits an ambiguous branch
(DECLINE). Orient each closed loop consistently in the cut plane (CCW about the
plane normal) so an outer loop and its holes have opposite signed area. Result: an
ordered list of closed section wires, or a typed `NotClosed` / `AmbiguousBranch`
decline.

## 3. The service + optional cap (`section/section.h`)

`sectionPlane(solid, plane, wantCap)`:

1. `section_edges` over every face → the section-edge set (or DECLINE).
2. `loop_assembly` → ordered closed loops (or DECLINE).
3. VERIFY every loop: all points on the cut plane, all points on some solid face,
   loop closed within tol. Any failure → DECLINE (never emit).
4. If `wantCap`: synthesize the planar cap via the B4 cross-section-cap path
   (outer loop minus holes) and compute its enclosed AREA by the planar polygon
   (shoelace in the plane basis) — this is the value GATE A checks against the
   closed form and GATE B against `BRepGProp`.

Returns `Section { loops[], loopCount, allClosed, cappedArea, totalEdgeLength }`
or `Decline { reason }`.

## 4. The additive facade accessor (`cc_section_plane`)

```
typedef struct { double *pts; int pointCount; int closed; } CCSectionLoop; /* xyz triplets */
typedef struct { CCSectionLoop *loops; int loopCount; double cappedArea; double totalLength; } CCSection;
CCSection cc_section_plane(CCShapeId body, double ox,double oy,double oz,
                                           double nx,double ny,double nz, int wantCap);
void cc_section_free(CCSection s);
```

ADDITIVE-ONLY (proven by `git diff`: 0 removed lines, only the new struct +
prototypes appended, mirroring how `cc_hlr_project` / `cc_split_plane` landed).
The native build routes to `section::sectionPlane`; the engine build routes to
`BRepAlgoAPI_Section` (+ `BRepGProp` for the cap) so the SAME accessor is the
oracle under `-DCYBERCAD_HAS_OCCT`. On a native DECLINE the accessor returns an
empty `CCSection` (loops null, loopCount 0) with `cc_last_error` set — NEVER a
partial section.

## 5. Verification strategy

- **GATE A — HOST ANALYTIC (`test_native_section`, no OCCT):** on-plane +
  on-face + closed invariants asserted per loop; enclosed area vs the closed form
  for box rectangle (`w·h`), cylinder axial rectangle (`2R·H`), cylinder
  cross-section circle (`πR²`), sphere great-circle (`πR²`). The three declines
  (tangent plane, non-closing, hard-freeform) asserted to return a typed decline.
- **GATE B — SIM native-vs-OCCT (`native_section_parity.mm`, booted sim):** same
  geometry reconstructed in OCCT; assert native TOTAL section-edge length, LOOP
  COUNT and CLOSED-NESS match `BRepAlgoAPI_Section`, and native `cappedArea`
  matches `BRepGProp` — at fixed tolerances, never widened.

## 6. Why this is low-risk / bounded

Every geometric step is a landed, OCCT-verified capability; GS2 is the routing +
endpoint-ordering + on-both verification that ties them into a section service.
The freeform bowl-lidded convex-quad prism (the landed B4 operand) is the
freeform acceptance fixture; anything M1 cannot trace robustly DECLINES. The
module is disjoint from `boolean/` and `ssi/` (a sibling M2-FUSE workflow owns
those), so the two workflows do not collide.

## 7. Open decisions / deferrals (honest)

- **Torus faces** are OUT of this slice (plane∩torus is a planar quartic, only
  partly analytic in S1) — a torus face in the solid DECLINES.
- **Cap for a multiply-nested section** (island-in-hole) beyond one outer + its
  direct holes is deferred; such a section DECLINES rather than mis-nesting.
- **Self-intersecting / multi-solid** sections are out; they DECLINE.

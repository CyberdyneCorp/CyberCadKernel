# Design — moat-m0-freeform-mesher (MOAT M0)

Mesh ONE concrete **foreign trimmed B-spline/NURBS face** — a pcurve-bounded
`EDGE_LOOP` over a curved patch — into a **watertight** solid that matches OCCT
`BRepMesh`, through a **strictly additive** tessellator branch with a **proven
zero-regression** on all existing tessellation, or keep the **honest decline**. The
tessellator has been kept pristine all session; this stage necessarily touches it,
so the discipline below is the load-bearing part of the design, not an afterthought.

Clean-room from Piegl & Tiller (NURBS eval, already native in `math/bspline.h`) and
the existing mesher; OCCT (`BRepMesh_IncrementalMesh` + the foreign-rational-B-spline
STEP fixture) is the **oracle + fixture-author + fallback only**.

## 0. What the mesher already does (the substrate — verified in source)

`face_mesher.h::mesh()` classifies a face and routes it to ONE of three paths:

```
                     region.hasOuter()?
        no ───────────────┐            yes
   structuredGrid          │   region.isFullRectangle(tol, requireCorners)?
  (natural UV box)         │      yes ──► structuredGrid (boundary rows = shared samples)
                           │      no  ──► earClipMesh   (boundary polygon ONLY)
```

- **STRUCTURED-GRID** (full-parametric-rectangle faces: whole cylinder / sphere /
  torus / our bare-periodic B-spline). Builds a tensor `(u,v)` grid whose lines
  include every boundary `u`/`v` and are refined between them by the deflection
  metric (`axisSamples` → `worstCurvature` → `divisionsFor`). **Carries interior
  curvature sampling.** This is why our OWN B-spline faces mesh watertight.
- **EAR-CLIP** (`earClipMesh` → `uv_triangulate.h::triangulatePolygon`) — for a
  genuinely trimmed face (a cap disk, a holed plane, a fillet blend patch). It
  triangulates ONLY the flattened boundary polygon. **It adds NO interior points.**
  Its own header comment states faces "that need INTERIOR curvature sampling are
  meshed by the … structured-grid path". Exact and cheap for a PLANAR trimmed face
  (flat → no interior needed). **Wrong for a CURVED trimmed patch** — the boundary
  chords span the patch off-surface, chord deflection is unbounded, volume is wrong.

**Exact facts this design leans on (checked in the source):**

- `SurfaceEvaluator` (`surface_eval.h`) is the ONE place the `FaceSurface` variant is
  switched; it already evaluates **rational** B-spline faces via
  `math::nurbsSurfacePoint` / `math::nurbsSurfaceDerivs` (non-empty `weights`). So
  point / `d1` / curvature at any `(u,v)` on a foreign NURBS patch is already native.
- `trim.h` already flattens an `EDGE_LOOP` to a UV polygon at shared per-edge
  fractions (`appendEdgeSamplesAtFracs`), evaluates analytic **and** B-spline pcurves
  (`pcurveValue`, case `K::BSpline` with a real knot vector so `S(pcurve(t)) =
  C_edge(t)`), and classifies interior points (`UVRegion::inside`, even-odd, outside
  every hole). **The boundary + classification machinery already exists.**
- `face_mesher.h` already records **canonical boundary anchors** (`BoundaryAnchors`)
  and snaps every seam-lying vertex to a build-order-independent point, so two faces
  sharing a straight OR curved seam weld BIT-IDENTICAL — the watertight contract.
- `uv_triangulate.h::triangulatePolygon` ear-clips the boundary with hole bridging;
  it has NO Steiner-point / interior support (line 25 comment is explicit).
- Reader: `pcurveFor` (`step_reader.cpp` ~L1812) has arms for **Plane** and
  **angular** (cylinder/cone/sphere) surfaces; a B-spline SURFACE with trim edges
  falls through to the generic **linear** arm (projects endpoints to a straight UV
  line) — wrong for a curved boundary. `projectBSplineUV` (~L1957) already inverts a
  B-spline surface `(grid-seed + damped Newton on the orthogonality condition)` to
  `(u,v)` in fp64 — the projection the pcurve reconstruction needs.

So the MATH (rational eval + surface inversion), the BOUNDARY flattening, the
INTERIOR classification, and the WELD anchors all already exist. **The one missing
piece is interior curvature sampling of a trimmed curved patch, triangulated
boundary-constrained.** That is the whole of M0's tessellator delta.

## 1. The additive mesh branch (`face_mesher.h::trimmedFreeformMesh`)

Dispatch (a single new guarded arm in `mesh()`, after the `isFullRectangle` test):

```
freeForm = (kind == BSpline || kind == Bezier)
if region.hasOuter() && !isFullRectangle && freeForm:
    return trimmedFreeformMesh(eval, loops, region, flip, anchors)   // NEW
else:
    return earClipMesh(...)                                          // UNCHANGED (planar)
```

`trimmedFreeformMesh` (delegates so per-function cognitive complexity stays in the
systems band):

1. **Boundary** — the flattened `loops` are the outer + hole polygons in `(u,v)`,
   already sampled at the shared per-edge fractions (`flattenWireShared`) so the
   boundary is dense enough for its own chord bound and welds with neighbours. These
   vertices are FIXED (constrained).
2. **Interior samples** — over the trim region's UV bbox, size a curvature-driven
   grid `nu × nv` from `worstCurvature(eval, box, foldTwist=true)` and `divisionsFor`
   (the SAME deflection law the structured grid uses). Emit the grid nodes strictly
   **inside** the outer loop and **outside** every hole (`UVRegion::inside`), and at
   least a minimum well-separated from the boundary (a boundary-distance guard so an
   interior node never lands ON a boundary segment, which would break the constrained
   triangulation). This bounds the interior chord error by the same sagitta metric
   that governs every other curved face.
3. **Constrained triangulation** — `triangulateConstrained(pts, loops, interior)`
   (new, §2) triangulates the region so that (a) EVERY boundary segment of every loop
   is an edge of the output (the shared-edge samples survive verbatim → weld), and
   (b) the interior Steiner points are incorporated. Result is a set of UV triangles.
4. **Evaluate + snap** — each UV vertex → `S(u,v)` via `eval.d1` (rational-aware);
   boundary vertices snapped to their canonical anchor; normals flipped for a
   Reversed face. Identical to the existing `evaluatePoints`.

Every produced vertex lies on the true surface; every boundary segment is shared
verbatim with the neighbour → the solid welds watertight exactly as the structured
grid does.

## 2. The constrained triangulator (`uv_triangulate.h::triangulateConstrained`)

A new entry point that PRESERVES the boundary and inserts interior points. Two
implementation options, chosen by robustness on the fixture (the design fixes the
CONTRACT, not the algorithm):

- **(A) Constrained Delaunay in UV** — Bowyer-Watson over the boundary + interior
  points, then recover/enforce the boundary segments as constraints and delete
  triangles whose centroid falls outside the region (`UVRegion::inside`). Interior
  edges are Delaunay → well-shaped, matches BRepMesh's UV-Delaunay character.
- **(B) Grid-clip + boundary stitch** — keep the interior grid quads fully inside
  the region as two triangles each, and stitch the ragged interior/boundary gap with
  a constrained ear-clip band. Simpler, no in-circle predicate exactness worries.

Either way the CONTRACT is: output triangles (i) cover the trim region with no gaps
or overlaps, (ii) contain every boundary segment as an edge, (iii) place no vertex
off the boundary/interior set. `triangulatePolygon` (the planar ear-clip) is
untouched — it remains the path for planar trimmed faces, byte-identical.

The triangulator is the irreducible-geometry core; its complexity is isolated behind
`triangulateConstrained()` and documented (systems band, like the existing ear-clip).

## 3. Reader admission of ONE foreign trimmed B-spline face (`step_reader.cpp`)

`advancedFace` already builds a face for any non-torus surface with real edges via
`buildFaceWithPCurves` → `pcurveFor`. The additive work is a faithful **B-spline
surface** pcurve arm, gated by a reconstruction guard:

- For each boundary edge, reconstruct its pcurve on the B-spline surface:
  - a **straight-in-`(u,v)`** edge (rim / seam / isoparametric trim) → project the
    two endpoints with `projectBSplineUV` and join by a UV **Line** (existing generic
    arm, now reached deliberately for BSpline);
  - a **curved** boundary edge → sample `C_edge(t)` densely, project each sample with
    `projectBSplineUV`, and store a UV **B-spline** pcurve (degree/knots preserved) so
    `pcurveValue` reproduces it; `trim.h` already evaluates a knotted B-spline pcurve.
- **Faithful-reconstruction guard** — re-evaluate `S_face(pcurve(t)) = C_edge(t)` at
  several `t` within a **scale-relative** tolerance (never weakened). If the residual
  exceeds tolerance for ANY edge — the projection did not converge, the pcurve does
  not lie on the patch, the boundary is beyond gap tolerance — **`decline()` → OCCT**.
- The rational case rides along: `bsplineSurface` already reads `weights` shape; the
  only relaxation is admitting a real `EDGE_LOOP` bound (today only the bare-periodic
  `VERTEX_LOOP` revolved form is admitted). Non-faithful ⇒ decline, unchanged.

The engine's mandatory watertight + volume/area self-verify against the OCCT oracle
is the FINAL arbiter downstream of all of this: a native trimmed-freeform solid that
is not watertight or off-volume is DISCARDED → OCCT. A wrong/leaky mesh is never
emitted.

## 4. Zero-regression discipline (the load-bearing invariant)

The new mesh branch is reachable ONLY by a **curved genuinely-trimmed** face —
`freeForm && region.hasOuter() && !isFullRectangle`. Every face any current suite
meshes is one of: a full-parametric primitive (structured grid), a planar trimmed
face (ear-clip), or a bare-periodic freeform face (structured grid) — NONE of which
satisfies the new guard. So no passing mesh can change. This is **proven, not
assumed**:

1. **Byte-identical gate.** Before/after, mesh a face of every kind (`Plane`,
   `Cylinder`, `Cone`, `Sphere`, `Torus`, `BSpline` bare-periodic, `Bezier`) and the
   FULL tessellation-sensitive suite — `run-sim-suite` (221/221), curved-fillet
   (23/23), curved-chamfer (18/18), curved-boolean (native-pass=18), wrap-emboss
   (14/14), loft, phase3 (70/70), STEP import (77/77). Triangle counts, watertight
   status, and enclosed volumes MUST be **identical**. If ANY differs → revert the
   mesher change; the foreign patch keeps the OCCT decline.
2. **Host analytic gate.** A native trimmed `Kind::BSpline` face (a known trimmed
   patch built on the host with no OCCT) meshes watertight, every triangle within
   deflection, enclosed solid volume within tol of the independent value.
3. **Sim parity gate.** On the booted simulator (OCCT linked) the foreign
   trimmed-B-spline STEP fixture imports to a solid whose volume / area / watertight /
   triangle envelope matches OCCT `BRepMesh` — OR the reader declines and the file
   round-trips through OCCT unchanged.
4. **Honest-out.** If a clean additive path that keeps every existing mesh
   byte-identical AND meshes the foreign patch watertight is not achievable, the
   mesher change is reverted and the reader keeps the decline. No fabrication, no
   dead code, no weakened tolerance.

## 5. Alternatives considered

- **Generalise the structured grid to trimmed faces** (clip the tensor grid by the
  loop). Rejected for the first slice: it perturbs the shared grid-line placement of
  neighbouring untrimmed faces (blast radius across every existing mesh) and cannot
  guarantee the boundary segments survive as edges. The additive constrained branch
  isolates all risk to the new (currently-declining) case.
- **A full general trimmed-NURBS mesher** (arbitrary pcurves, self-intersecting
  wires, gap bridging). That is the ~1.5–3 py capability; explicitly out of this
  slice. This change lands the narrow faithful-pcurve case and DECLINES the rest —
  the honest bounded target the roadmap sets for M0.

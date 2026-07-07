# Design — moat-m2b-freeform-face-split

## Context

M2 (general freeform booleans) is blocked on three isolated substrate subsystems
(`openspec/MOAT-ROADMAP.md`): B1 (freeform recogniser), **B2 (freeform face-split)**,
B3 (freeform point-in-solid). This design covers the FIRST SLICE of B2. The
surrounding pipeline is `recognise → trace [M1] → split [B2] → classify [B3] →
weld [M0]`. B2 is purely the middle step, and its inputs/outputs are already
concrete types produced by landed code:

- **Seam input — M1 `WLine`** (`src/native/ssi/marching.h`): `WLine.points` is a
  `std::vector<WLinePoint>`; each `WLinePoint` carries `u1,v1` (params on surface A),
  `u2,v2` (params on surface B), `point` (3D), `onSurfResidual`. `WLine.curve` is the
  fitted `FittedBSpline`. For a face-split we treat THIS face as surface A and use the
  `(u1,v1)` sequence as the seam pcurve in the face's own parameter plane.
- **Face input — `topo::Shape` face** over a `FaceSurface` (`shape.h`) of
  `Kind::BSpline`/`Kind::Bezier`, with an outer wire (`EDGE_LOOP`) of edges that each
  carry a `PCurve` on the surface. The mesher already flattens this to a UV polygon
  via `FaceMesher::buildBoundaryLoops` (shared per-edge fractions + `BoundaryAnchors`).
- **Mesh consumer — M0 `FaceMesher`** (`src/native/tessellate/face_mesher.h`): a
  genuinely-trimmed freeform `EDGE_LOOP` face routes to `trimmedFreeformMesh`, which
  samples the interior and folds it into a boundary-constrained triangulation
  (`triangulateConstrained` / `detail::ConstrainedDelaunay` in `uv_triangulate.h`).
  Each sub-face this change builds is exactly such a face, so it meshes with NO
  tessellator change.

## Goals / Non-Goals

**Goals (first slice).** Given ONE trimmed freeform face with a CONVEX outer
`EDGE_LOOP` and ONE M1 seam chord that CLEANLY crosses it (enters through one
boundary edge, exits through another, no tangency, no re-entry), partition the
trimmed `(u,v)` domain along the seam into TWO closed sub-loops and emit two
genuinely-trimmed sub-faces that TILE the parent. Prove tiling on host (area sum +
shared seam + no overlap) and watertight-per-sub-face + area parity on sim vs OCCT.

**Non-Goals (decline, do not fake).** Multiple seam arms; a seam that re-enters the
domain; tangential/grazing contact with the boundary; branch points; holes in the
trimmed domain; concave outer loops where the chord exits and re-enters; the
boolean-level wiring. Each is a NULL split with a measured blocker, never a
partial/approximate split.

## Decisions

### D1 — B2 is a pure 2D combinatorial partition in the face's own `(u,v)` plane

The split happens entirely in the parameter plane of THIS face. We reuse the SAME
UV boundary polygon the mesher builds (`buildBoundaryLoops` → `UVPolygon` at shared
per-edge fractions, with `BoundaryAnchors`), so sub-face boundary points are
BIT-IDENTICAL to the parent's and to any neighbouring face's — the M0 weld contract
is preserved for free. The seam is the polyline `[(u1,v1)]` from `WLine.points`.
No 3D geometry is invented in the partition; 3D edges/pcurves are carried over from
the parent and the seam edge is built from the M1 curve.

Rationale: the seam nodes already live on the face's surface (surface A residual
`≤ tol`), and the boundary polygon is already the canonical shared flattening.
Keeping the partition 2D makes tiling a provable planar-area identity and keeps the
subsystem OCCT-free and host-testable.

### D2 — Seam clip: entry/exit crossings against the outer polygon

Walk the seam polyline and the outer UV polygon and find the seam's crossings with
the boundary. For the first slice we REQUIRE exactly two crossings — one ENTRY
(seam going from outside→inside the outer loop) and one EXIT (inside→outside) — with
the interior chord monotonically inside. We compute each crossing as a segment
intersection (`orient2d`/`segmentsCross` primitives already in `uv_triangulate.h`)
and record (a) the boundary edge index + fraction and (b) the seam node index +
fraction. If the seam has fewer/more than two boundary crossings, or any interior
node lies outside the outer loop (`UVRegion::inside` even-odd test), the split
DECLINES — that is a case beyond this slice.

Rationale: a single clean chord is the minimal provable partition; more crossings
imply multiple sub-regions or re-entry, which need the general splitter.

### D3 — Partition: two boundary walks spliced with the shared seam chord

With entry crossing `E` and exit crossing `X` on the outer boundary, form:
- **L1** = boundary arc `E → … → X` (forward around the loop) + seam chord `X → E`
  (the interior `(u1,v1)` nodes in reverse), closed.
- **L2** = boundary arc `X → … → E` (the complementary forward arc) + seam chord
  `E → X` (the interior nodes forward), closed.

Both sub-loops carry the SAME seam vertices in opposite traversal order, so the seam
is their EXACT shared boundary (identical UV nodes, opposite orientation) — the weld
is bit-exact. Each sub-loop is a simple polygon by construction when the outer loop
is convex and the chord is a single interior segment set.

Rationale: this is the standard chord-splits-a-simple-polygon operation; convex
outer + single interior chord guarantees two simple sub-polygons with the right
orientation for a Forward/Reversed face pair.

### D4 — Sub-face rebuild reuses parent edges; the seam becomes a new shared edge

Each sub-`Face` is built over the SAME `FaceSurface` node (shared geometry). Its
outer wire is: the parent boundary edges spanning its boundary arc (kept verbatim,
with their existing 3D `EdgeCurve` and `PCurve`), plus one NEW seam edge. The seam
edge's `PCurve` is the `(u1,v1)` polyline as a UV `B_SPLINE`/polyline pcurve on the
surface; its 3D `EdgeCurve` is the `WLine.curve` fitted B-spline (or the polyline
when the fit is absent). The two sub-faces SHARE that one seam edge node (built once,
added to both wires with opposite orientation via `Builder::makeFace` /
`addPCurve`), so downstream welds see one shared edge, two faces.

Boundary edges that the entry/exit crossing lands MID-edge are SPLIT at the crossing
fraction into two sub-edges (same curve, restricted `[first,last]`), so each sub-loop
gets a clean edge chain. If a crossing lands within snapping tolerance of an existing
vertex, no split is made (the vertex is reused).

### D5 — Self-verify → decline gate (host-checkable, no OCCT)

Before returning, the splitter runs a mandatory gate; ANY failure → return NULL
(decline), never a partial split:
1. **Clean crossing:** exactly one entry + one exit crossing; interior seam nodes all
   strictly inside the outer loop; no seam node within tolerance of leaving/re-entering.
2. **Non-degenerate sub-regions:** each sub-loop is a simple polygon (no
   self-intersection via `segmentsCross`) with |signed UV area| above a
   scale-relative floor.
3. **Exact shared seam:** L1 and L2 reference the identical seam UV node sequence
   (opposite order); no gap between the seam endpoints and the boundary crossings
   beyond snap tolerance.
4. **Tiling identity:** `area(L1) + area(L2) == area(parent outer loop)` within a
   scale-relative tolerance (UV signed-area sum), AND no overlap (the seam is the only
   shared boundary; interiors are disjoint by construction from the two complementary
   arcs).

Rationale: these four are the complete "TILE the original" contract, all provable in
the UV plane with the primitives already in `uv_triangulate.h`, so the host gate needs
no OCCT. The gate is the honest-decline mechanism: the measured blocker (crossing
count, degenerate area, area-sum gap) is what a decline reports.

### D6 — OCCT stays the oracle; the split is cross-checked, never OCCT-derived

`src/native/**` remains OCCT-free (0 OCCT includes). OCCT appears ONLY in the SIM
proof harness (`src/engine/occt`), where each sub-face is meshed with
`BRepMesh_IncrementalMesh` and its area compared to the native M0 mesh, and — where
an OCCT freeform face-split reference is available — the native seam is cross-checked
against the OCCT split. The native partition is computed with no OCCT dependency.

## Risks / Trade-offs

- **Marched seam is discretized and possibly noisy.** The `(u1,v1)` polyline may
  wobble near the boundary. Mitigation: the entry/exit crossings are computed against
  the boundary polygon with the same snap tolerance the mesher uses, and the gate
  rejects any seam whose endpoints do not cleanly reach the boundary. If robust
  crossing detection is not reachable on the chosen case, we DECLINE and report the
  measured gap (this is an accepted first-class outcome, not a failure to ship).
- **Convex-only first slice** limits coverage, by design. The chosen proof case is a
  single chord across a convex trimmed patch; the general splitter is future work.
- **No boolean-level effect yet.** B2 alone changes no `cc_*` result (M2a wires it);
  its value is the proven, self-verified substrate the assembly consumes.

## Migration / Rollout

Purely additive, header-only, behind no `cc_*` change. Nothing calls the splitter in
a `cc_*` path yet, so no existing behaviour moves. The subsystem is exercised only by
its own host + sim tests until M2a integrates it.

## Open Questions

- Exact tolerance for the tiling area-sum identity (scale-relative factor) is pinned
  during host implementation against the concrete proof face; it is never weakened to
  pass a case.
- Whether the seam-clip helper lives in `src/native/ssi/` (next to the `WLine` data
  model) or wholly in `src/native/boolean/` is decided during implementation by which
  keeps the cleaner OCCT-free boundary; both are acceptable.

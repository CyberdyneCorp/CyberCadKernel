# Design — moat-hlr-hidden-line-removal

## Context

Hidden-line removal (HLR) for a 2D drawing takes a 3D solid and a view direction
and produces two disjoint sets of 2D curves on the drawing plane: the **visible**
edges (drawn solid) and the **hidden** edges (drawn dashed). OCCT does this with
`HLRBRep_Algo` (+ `HLRBRep_HLRToShape`), the oracle we must match. This slice
targets **orthographic** (parallel) projection of the **analytic/polyhedral
core** and consumes only landed native machinery: `topology` (edges/faces),
`tessellate` (the M0 boundary mesh used as an occluder), and `math` (elementary
surfaces + linear algebra). `src/native/**` stays OCCT-free.

## Goals / Non-Goals

**Goals (this slice)**
- Orthographic projection of straight (linear) edges onto a drawing plane.
- Exact per-sample visible/hidden classification against a **triangle occluder**
  (the tessellated solid boundary) via ray-casting toward the parallel viewpoint.
- Splitting each edge at every visibility transition into disjoint visible/hidden
  segments; conservation of projected length across the split.
- A host-analytic proof (box iso corner → 9 visible + 3 hidden) with no OCCT.

**Non-Goals (declined / follow-up)**
- **Curved-surface silhouette tracing** — the outline where `n·view = 0` on a
  cylinder/cone/sphere (closed-form lines/ellipses) and its tangency curve. Not
  emitted this slice.
- **Freeform faces** (B-spline/Bézier/NURBS surfaces) — no silhouette solver.
- **Perspective** projection, section views, and dimension/annotation geometry
  (later M-GS items GS2–GS4).
- The `cc_hlr_project` facade **body** + PODs, and the Gate (b) `.mm` OCCT harness
  — specified here, implemented as follow-up.

## Decisions

### D1 — Occlusion by ray-casting against the M0 boundary mesh
For each projected edge sample `P` (a world point on the edge), the classifier
casts a ray from `P` toward the parallel viewpoint (direction `−viewDir`) and
counts a hit on any occluder triangle at a strictly positive distance as
evidence that a nearer surface lies in front → `P` is HIDDEN. This reuses the
watertight boundary tessellation the kernel already produces (`solid_mesher.h`),
so occlusion is defined for **any** tessellated solid, analytic or faceted, with
one Möller–Trumbore test per triangle. Rationale: a mesh occluder is exact enough
for HLR classification (the classification is a boolean, and the mesh bounds the
true surface within the tessellation deflection), and it is trivially OCCT-free.

### D2 — Nudge toward the camera to kill self-hits
An edge lies **on** the solid's surface, so its two adjacent faces are coplanar
occluder triangles that a naive ray would graze. Before casting, the sample is
displaced a tiny `surfaceOffset` toward the camera (`P' = P − viewDir·ε`). The
edge's own adjacent faces then sit at negative ray parameter (behind `P'`) and
are ignored; only a genuinely nearer surface produces a positive-distance hit.
For a convex solid this is exact; for the general case the offset is the single
tolerance and is never silently widened.

### D3 — Classify at cell MIDPOINTS, not at nodes
Each edge is divided into `samplesPerEdge` cells and classified at each cell's
**midpoint** — a strict edge interior point — never at the exact endpoints. Edge
endpoints coincide with vertices shared by other edges and, on a silhouette, sit
on the outline where the visible/hidden status is genuinely ambiguous (in a true
isometric view the near and far cube corners even project to the same point).
Sampling interiors makes the classification well-defined; the emitted segments
still span the full edge to its endpoints. This is why the box gate is a clean
`9 + 3` with no spurious silhouette-corner slivers.

### D4 — Split by bisection to a fixed tolerance
Consecutive same-class cells merge into a run; at a class change between two
adjacent midpoints the transition parameter is refined by **bisection** to
`transitionTol` and the run is cut there. Result: disjoint visible/hidden
segments whose union is the full projected edge (length-conserving). A genuine
mid-edge partial occlusion (e.g. an edge half-behind a nearer face) yields the
correct split, proven by the `edge_splits_at_visibility_transition` test.

### D5 — Drawing-plane basis
Orthonormal basis from the view: `right = normalize(viewDir × up)`,
`trueUp = right × viewDir`; a world point projects to `(P·right, P·trueUp)`.
Depth toward the camera increases along `−viewDir`. The `up` hint must not be
parallel to `viewDir` (caller contract).

### D6 — Additive ABI seam (`cc_hlr_project`), specified now, wired later
The public boundary stays a plain-C additive accessor: `cc_hlr_project(body,
viewDir, up, opts) -> CCDrawing` returning flat visible/hidden 2D segment arrays
freed by the caller, mirroring how the drawings feature consumes OCCT's
visible/hidden compounds. No existing `cc_*` signature changes. This slice
specifies the contract and lands the native core behind it; the facade body is a
follow-up task so the ABI shape is reviewed before it is frozen.

## Verification (two gates)

- **GATE (a) HOST ANALYTIC (no OCCT)** — `tests/native/test_native_drafting.cpp`,
  built with plain `clang++ -std=c++20`:
  - box from an isometric corner → **exactly 9 visible + 3 hidden** segments, the
    3 hidden segments sharing one endpoint (the occluded far corner);
  - the same box with an empty occluder → **12 visible, 0 hidden** (baseline);
  - a straight edge half-covered by a nearer face → **1 visible + 1 hidden**
    split at the coverage boundary (`x = 0` within refinement tolerance);
  - projected length is conserved across the visible/hidden split.
- **GATE (b) SIM native-vs-OCCT (follow-up harness)** — on a booted iOS
  simulator, drive `HLRBRep_Algo` + `HLRBRep_HLRToShape` on the same box /
  cylinder / cone and assert the native visible/hidden sets match the oracle on
  **count**, **total projected length**, and **endpoint positions** within
  tolerance. Only the harness `.mm` links OCCT; the native library never does.

## Risks / Honest blockers (sharpened)

1. **Curved silhouette tracing is the real remaining work.** For a cylinder
   side-on the drawing needs the two outline generators + the visible/hidden
   split of the end-circle ellipses where the silhouette is tangent — closed-form
   but not yet implemented. Until it lands, a solid whose visible outline is a
   **curved** silhouette (not a tessellation edge) is **declined**, because
   approximating the silhouette by mesh-edge segments would emit a subtly wrong
   outline. This is the sharpened GS1 blocker.
2. **Coincident / grazing edges** (an edge exactly along the view direction, or
   two solids sharing a face) can defeat the single `surfaceOffset` tolerance; the
   classifier declines rather than guess when a sample's occlusion is within
   tolerance of ambiguous.
3. **Mesh-occluder deflection** bounds classification accuracy near a silhouette;
   the topology-driven adapter must choose a deflection fine enough that no true
   visible/hidden boundary is mislabelled, or decline. This is why the occluder
   deflection is an explicit input, never a hidden default.

## Migration / Compatibility

Additive only. `src/native/**` gains a header-only subtree that includes solely
`src/native/math`; no existing file, the `cc_*` ABI, or any engine path changes.
The default host build simply gains one always-on CTest.

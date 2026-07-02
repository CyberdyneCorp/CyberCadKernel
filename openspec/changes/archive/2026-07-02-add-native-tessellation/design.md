## Context

Phase 4 (`openspec/NATIVE-REWRITE.md`) replaces the OCCT adapter with native
C++20, one capability at a time, behind the unchanged `cc_*` facade. Capability
#1 (`native-math`, `src/native/math/`) landed the OCCT-free fp64 surface
evaluator: `value(u,v)`, `dU`/`dV`, and `normal` for `Plane`/`Cylinder`/`Cone`/
`Sphere` (elementary) and Bézier/B-spline/NURBS surfaces. Capability #2
(`native-topology`, `src/native/topology/`) landed the OCCT-free B-rep model:
`Face → surface + ordered outer/inner wires + pcurves`, a deterministic
`Explorer`, edge→face `AncestryMap`, and `BRep_Tool`-style accessors
(`surfaceOf`, `curveOf`, `rangeOf`, `pcurveOf`, `pointOf`). The locked dependency
order puts `native-tessellation` third: it is the first native capability that
consumes both foundations and produces a concrete, independently checkable
artifact — a triangle mesh.

Tessellation converts a B-rep into triangles at a bounded **deflection**
(chord-height) tolerance. Unlike math and topology (exact, so parity is
value-identical within fp64), a mesh is an **approximation**: two correct meshers
at the same deflection produce *different* triangles. So the verification is
tolerance-based *properties*, never triangle-identical output. This is exactly
what the two independent gates test:

1. **Host analytic tests** — the mesher compiles and unit-tests with
   `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator,
   against closed-form ground truth (a plane rectangle, a cylinder, a sphere, a
   holed plane): deflection bound, on-surface residual, watertightness, and
   area/volume convergence.
2. **Simulator native-vs-OCCT property parity** — on the simulator (OCCT linked)
   the same shape is meshed by the native mesher and by OCCT
   `BRepMesh_IncrementalMesh` at a matched deflection, and their bbox / total
   area / enclosed volume / watertightness are compared within a documented
   tolerance. The OCCT→native bridge is reused from the topology harness.

Constraints:
- **OCCT-free library.** No file under `src/native/tessellate/` may include any
  OCCT header. It MAY include `src/native/math` and `src/native/topology`. The
  library links no OCCT; OCCT appears ONLY in the simulator parity test.
- **Host-buildable.** The library + host tests build with `clang++ -std=c++20`
  (and the `native-math` + `native-topology` headers) and nothing else.
- **Clean-room.** Implemented from computational-geometry references (grid /
  Delaunay triangulation, polygon-in-boundary trimming) and the `cc_*` contract.
  OCCT source (`BRepMesh`, `IMeshTools`) is a reference oracle — consulted to
  confirm an algorithm matches and compare numerics — never copied.
- **No ABI change, no engine wiring.** The `cc_*` facade and the active engine
  are untouched; this change ships the library + its tests only.
- **fp64 CPU is the source of truth.** The Phase-2 GPU surface evaluator MAY be
  an fp32 sampling backend for eligible faces, but every correctness property is
  asserted on the CPU path; the GPU path is display-only and Metal-guarded.
- **Determinism.** Fixed grid-sampling and stitching order; reproducible mesh run
  to run for a given face/solid + deflection.
- **Maintainability first.** Clear, well-named C++20 (`std::span`/`std::vector`/
  concepts where natural). The grid-refinement / trimming / stitching routines are
  the geometry core that may approach the systems band (≤ 25–35, flagged); the
  surface sampling itself delegates to `native-math`.

## Goals / Non-Goals

Goals:
- Mesh a native `Face` to a triangle mesh at a requested deflection by UV-grid
  sampling of its surface via `native-math`, respecting the face's parameter box,
  with grid density chosen so each triangle's chord-height error ≤ deflection.
- Trim the UV grid against the face's inner wires (holes) using the wires'
  pcurves so triangles inside holes are removed and the trimmed region is honoured.
- Guarantee every emitted vertex lies on the true face surface within the
  deflection tolerance (surface residual + boundary-on-curve residual).
- Mesh a whole `Solid` by meshing each face and stitching shared edges (same
  parameter samples on the shared curve, welded coincident vertices) into a
  watertight mesh — each mesh edge shared by exactly two triangles for a closed
  solid.
- Converge mesh-derived surface area and enclosed volume toward the analytic /
  B-rep values as deflection decreases.
- Two-gate verification: host analytic tests + simulator native-vs-OCCT
  `BRepMesh` property parity (bbox/area/volume/watertight within tolerance).

Non-Goals:
- Triangle-identical output vs OCCT (a mesh is an approximation; only properties
  are compared).
- Any construction / boolean / fillet operation (later capabilities) — this
  meshes an existing B-rep; it does not build one.
- Mesh simplification / decimation, LOD, remeshing, or quad meshing.
- Adaptive anisotropic metrics beyond a deflection-driven grid (chord-height +
  optional angular tolerance); a full BRepMesh-grade curvature-adaptive incremental
  mesher is out of scope — a deflection-bounded grid + trim + stitch is the target.
- Any `cc_*` ABI change, facade exposure, or engine wiring.
- fp64 exactness on the GPU path (it is fp32, display-only; correctness is CPU).

## Decisions

- **Directory + build seam.** All native tessellation lives under
  `src/native/tessellate/` (headers + sources), OCCT-free, may include
  `src/native/math` + `src/native/topology`. A host CTest target compiles it with
  `clang++ -std=c++20` and links only the analytic tests. A separate simulator
  parity target links OCCT and includes the native headers + OCCT to mesh both
  ways and compare — OCCT never enters the library translation units. A grep gate
  asserts no OCCT include under `src/native/tessellate/`.
- **Mesh representation.** A `TriMesh` is an fp64 vertex buffer (position +
  optional normal), a triangle index buffer (`uint32` triples, CCW as seen from
  outside so the face's outward normal orientation is consistent with the surface
  normal and the face orientation from `native-topology`), and a per-vertex
  `(u,v)` parameter tag (used by trimming, stitching, and the on-surface check).
  A `FaceMesh` is one `TriMesh` per face; a `SolidMesh` is the stitched union with
  a face-id tag per triangle.
- **Deflection-driven UV grid (chord-height).** For a face, take the parameter
  box `[u0,u1]×[v0,v1]` from the surface + wire bounds. Choose the grid step in
  each parameter direction so the maximum chord-height deviation of a grid quad
  from the true surface is ≤ the deflection: from `native-math` second-order
  behaviour (curvature via `dU`/`dV` sampling, or a closed-form radius for
  elementary surfaces), the sagitta over a step `h` is ≈ `h²·κ/8`, so
  `h ≤ sqrt(8·deflection/κ)`. A conservative curvature estimate (sampled max over
  the box) picks a per-direction step; the grid is refined until a sampled
  chord-height probe is under tolerance. Planes need one quad; high-curvature
  regions get more. An optional angular tolerance caps the normal deviation per
  step for very flat-but-curved cases.
- **Grid triangulation.** Each interior grid quad splits into two triangles on a
  fixed diagonal (deterministic). Wound CCW w.r.t. the surface outward normal so
  a `Reversed` face flips winding (matching `native-topology` orientation
  semantics). This is the clean-room "regular grid triangulation" path; the
  Delaunay reference is consulted for the trimmed-boundary cells only.
- **Trimming against wires (holes).** The outer wire bounds the sampled UV region;
  each inner wire is a hole. Trimming works in **parameter space** using the
  wires' **pcurves** (`pcurveOf(edge, face)` from `native-topology`): build each
  wire's 2D polyline by sampling its pcurves, classify each grid vertex/cell by a
  point-in-polygon test against the outer boundary minus the holes, drop cells
  fully inside a hole, and re-triangulate boundary-straddling cells against the
  clipped polygon (a small constrained triangulation of the cell ∩ region, the one
  place the Delaunay reference is used). The trimmed region's area equals the
  outer area minus the hole areas.
- **On-surface guarantee.** Every emitted vertex is produced by evaluating the
  surface at its `(u,v)` via `native-math` `value(u,v)`, so it lies on the true
  surface by construction; the host test additionally asserts the sampled
  chord-height residual (midpoint of each triangle vs the surface at its `(u,v)`
  centroid) is ≤ deflection, and boundary vertices lie on the wire curve within
  tolerance (evaluated via `curveOf` on the shared edge).
- **Solid meshing + edge stitching (watertightness).** Mesh each `Face` from the
  `Explorer(solid, Face)`. For each shared edge (from the edge→face `AncestryMap`),
  the two adjacent faces sample the **same** edge curve at the **same** parameter
  set (from `curveOf(edge)`'s `[first,last]`, sampled at the deflection-driven
  count) so their boundary vertices are coincident; a tolerance weld merges
  coincident vertices across faces into shared mesh vertices. The result is
  watertight: for a closed solid every mesh edge is shared by exactly two
  triangles (a manifold-edge count check). Seam edges (an edge appearing twice on
  one periodic face) are handled by sampling the seam once and referencing it from
  both sides.
- **Convergence.** Mesh area = sum of triangle areas; enclosed volume = signed
  tetrahedron sum over the (consistently outward-wound, watertight) triangles
  (divergence-theorem `Σ (v0 · (v1 × v2))/6`). As deflection → 0 both converge to
  the analytic / B-rep value; the host test asserts monotone error decrease across
  a deflection sequence and the parity test asserts agreement with OCCT at a
  matched deflection.
- **CPU source of truth; optional GPU fp32 backend.** The CPU sampler
  (`native-math` fp64) is the correctness path. A face is *GPU-eligible* only when
  it is an untrimmed rectangular patch on a representable surface (mirroring the
  `add-gpu-tessellation` eligibility); for such faces the Phase-2 GPU evaluator
  MAY fill the grid in fp32 under `#ifdef CYBERCAD_HAS_METAL`. All correctness
  assertions run on the CPU path; the GPU path is display-only and never gates
  correctness.
- **Determinism.** Fixed parameter-box ordering, fixed grid step selection given a
  deflection, fixed quad-diagonal, fixed face/edge traversal (topology `Explorer`
  order), and a stable weld — so the same face/solid + deflection yields an
  identical mesh run to run.
- **Complexity is isolated and flagged.** The grid-refinement, the trimming /
  boundary-cell re-triangulation, and the stitch/weld are the systems-band
  routines; each is a single documented function with its reference citation, and
  everything else (mesh assembly, area/volume reduction, on-surface probe) stays
  well under 15.

## Verification

Each requirement is verifiable two ways (the roadmap's two gates):

- **(a) Host analytic unit test** — `clang++ -std=c++20`, no OCCT, no simulator.
  Builds faces/solids directly through `native-topology` `ShapeBuilder` with
  `native-math` surfaces whose area/volume/points are known in closed form:
  - a **planar rectangle** (area exact at any deflection; one quad; on-surface
    residual zero);
  - a **cylinder** side (area `2πrh`, curvature-driven grid; on-surface residual
    ≤ deflection; area converges as deflection → 0);
  - a **sphere** (area `4πr²`, enclosed volume `4/3πr³`; both converge);
  - a **holed plane** (a rectangle with a circular hole: trimmed area = rect −
    `πr²`; triangles inside the hole are absent; boundary vertices on the hole
    curve);
  - a **closed box / cylinder solid** (watertight: every mesh edge shared by
    exactly two triangles; enclosed volume converges to the analytic value).
  Assertions: deflection bound (max chord-height ≤ deflection), on-surface
  residual (each vertex on the surface within tolerance), watertightness
  (2-manifold edge count), and area/volume convergence (monotone error decrease
  across a deflection sequence, limit within tolerance).
- **(b) Simulator native-vs-OCCT property parity** — links the native headers and
  OCCT side by side, **reuses the TEST-ONLY OCCT→native bridge** from
  `tests/sim/native_topology_parity.mm` to walk a real `TopoDS_Shape` (box,
  cylinder, sphere, a holed face) into the native model, meshes it with the native
  mesher, meshes the same OCCT shape with `BRepMesh_IncrementalMesh` at the
  **same** deflection, and asserts: axis-aligned bbox match, total surface area
  match, enclosed volume match, and watertightness — each within a documented
  tolerance. Triangle count / topology are NOT compared (the meshes differ by
  construction); only the tolerance-based properties are.

The host gate pins the mesh to closed-form ground truth so an approximation error
that exceeds the deflection cannot pass; the parity gate confirms the native
mesher agrees with the OCCT oracle's mesh envelope at a matched deflection so the
later engine wiring is a drop-in. Because a mesh is an approximation, **both gates
are tolerance-based** — deflection bound, on-surface residual, watertightness, and
area/volume convergence — not triangle-identical output.

## Risks / Trade-offs

- **Deflection → grid-density mapping.** Too coarse misses the tolerance; too fine
  is wasteful. Mitigation: curvature-based step selection with a sampled
  chord-height probe that refines until under tolerance; the host deflection-bound
  test and the parity area/bbox check bound both directions.
- **Trimming correctness (cracks / leaks at hole boundaries).** A mis-classified
  cell leaves a hole unfilled or a hole triangle leaking. Mitigation: the
  holed-plane host test asserts the exact trimmed area (rect − `πr²`) and the
  absence of in-hole triangles; the watertightness invariant catches cracks.
- **Stitching / welding coincident vertices.** If the two faces sharing an edge do
  not sample it identically, boundary vertices don't coincide and the mesh leaks.
  Mitigation: shared edges are sampled from the single `curveOf(edge)` at one
  parameter set used by both faces; the weld is tolerance-based; watertightness
  (every closed-solid edge shared by exactly two triangles) is asserted on both
  gates.
- **fp32 GPU divergence from fp64.** Mitigation: correctness is asserted only on
  the CPU fp64 path; the GPU path is display-only, Metal-guarded, and used only for
  eligible untrimmed patches.
- **Reference-oracle drift.** Mitigation: clean-room from computational-geometry
  references; OCCT `BRepMesh` is consulted only to confirm the algorithm and
  compare numerics; the parity gate compares properties, not code.
- **Complexity of trimming / stitching.** Mitigation: each systems-band routine is
  a single documented function with its reference citation and stays ≤ 35; the
  surrounding assembly stays under 15.

## Migration Plan

1. Create `src/native/tessellate/` with the `TriMesh`/`FaceMesh`/`SolidMesh`
   representation and a host CTest target building with `clang++ -std=c++20`, no
   OCCT (may include `src/native/math` + `src/native/topology`). Add a grep gate
   for no-OCCT-include. (**host**)
2. Implement deflection-driven UV-grid face meshing (curvature-based step,
   chord-height probe, deterministic quad triangulation) over `native-math`
   surfaces; host tests (plane one-quad; cylinder/sphere deflection bound +
   on-surface residual). (**host**)
3. Implement trimming against inner wires (holes) via pcurves (2D polyline,
   point-in-region classification, boundary-cell re-triangulation); host test
   (holed-plane exact trimmed area, no in-hole triangles, boundary-on-curve).
   (**host**)
4. Implement solid meshing + edge stitching/welding (shared edge sampled once,
   coincident boundary vertices welded); host test (closed box/cylinder solid
   watertight — every mesh edge shared by exactly two triangles). (**host**)
5. Implement area/volume reduction and assert convergence across a deflection
   sequence toward the analytic values (plane/cylinder/sphere/box). (**host**)
6. (Optional) Wire the Phase-2 GPU fp32 evaluator as a sampling backend for
   GPU-eligible untrimmed patches under `#ifdef CYBERCAD_HAS_METAL`; correctness
   still asserted on the CPU path. (**host/sim**)
7. Add the simulator native-vs-OCCT `BRepMesh` property-parity target (OCCT linked
   only there; **reuse the topology harness OCCT→native bridge**) meshing box /
   cylinder / sphere / holed face both ways at a matched deflection and comparing
   bbox / area / volume / watertightness within tolerance; keep every existing
   suite green. (**sim-parity**)
8. `openspec validate add-native-tessellation --strict` green; reflect the
   `native-tessellation` in-progress status in `openspec/NATIVE-REWRITE.md` /
   `ROADMAP.md` Phase 4.

## Open Questions

- Whether the grid step is chosen purely from a curvature estimate or from an
  iterative refine-until-under-tolerance probe — default the probe (robust) with
  the curvature estimate as the starting density.
- Whether boundary-cell trimming re-triangulates with a small constrained Delaunay
  or a simpler ear-clip of the clipped polygon — default ear-clip for cells,
  Delaunay only if a cell's clipped polygon is degenerate; both are clean-room.
- Whether stitching welds by a spatial hash on 3D position or by shared-edge
  parameter identity — default parameter identity on the shared `curveOf(edge)`
  (exact), with a spatial-hash weld as a fallback for edges whose curves differ by
  more than tolerance.
- Whether normals are per-vertex analytic (from `native-math` `normal(u,v)`) or
  face-averaged — default analytic per-vertex (exact on the true surface),
  face-averaged only where an analytic normal is unavailable.
- Whether the optional GPU fp32 backend ships in this change or is deferred to the
  engine-wiring change — default: keep the CPU path complete and verified here; the
  GPU backend is optional and non-gating.

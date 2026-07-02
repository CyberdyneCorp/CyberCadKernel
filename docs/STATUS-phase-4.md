# CyberCadKernel — Phase 4 status (native rewrite, drop OCCT)

Honest, verification-anchored snapshot of Phase 4 — replacing the OCCT adapter
with native C++20, one capability at a time, until OCCT can be unlinked. Method,
verification model, and the full capability sequence live in the sub-roadmap
[`openspec/NATIVE-REWRITE.md`](../openspec/NATIVE-REWRITE.md). Nothing below is
claimed unless it was actually built and run in this environment.

Date: 2026-07-02 · Branch: `main`.

## TL;DR

- **Capability #1 `native-math` — done at the Phase-4 verification bar.** Both
  independent gates are green: host analytic unit tests (no OCCT, no simulator)
  and native-vs-OCCT numeric parity on the booted iOS simulator.
- **Capability #2 `native-topology` — done at the Phase-4 verification bar.**
  B-rep data model + exploration (`TopoDS`/`TopExp`/`BRep_Tool` analogues). Host
  gate green (`test_native_topology`, 13 cases, 0 failed) and native-vs-OCCT
  parity green on the booted iOS simulator (3 shapes × 5 checks = **15 passed,
  0 failed**, max accessor error **0.000e+00**).
- **Capability #3 `native-tessellation` — done at the Phase-4 verification bar.**
  Deflection-driven native mesher (UV-grid face meshing + parameter-space hole
  trimming + solid welding) consuming native-math surface eval and native-topology
  faces. Host gate green (`test_native_tessellate`, 13 cases) and native-vs-OCCT
  `BRepMesh` property-parity green on the booted iOS simulator (**All 20 checks PASS
  across 4 shapes** — box / cylinder / sphere / filleted-box; **ALL four closed solids
  watertight, `boundaryEdges==0`**; area/volume relMesh ≤ **6.0e-3**, relExact ≤
  **1.24e-2**, bbox max corner delta ≤ **4.66e-2**, vertices-on-surface residual ≤
  **5.7e-15**). The curved shared-edge seam (cylinder cap↔side, fillet blends) now
  welds watertight via the two-stage shared-edge mesher.
- **Capability #4 `native-construction` — done at the Phase-4 verification bar
  (core), first engine-wired capability.** Native `cc_solid_extrude` (closed
  polygon → prism) and native `cc_solid_revolve` (LINE-SEGMENT profile → surface of
  revolution) build real native solids and are compared A/B against OCCT through the
  facade. Both gates green: host `test_native_construct` + `test_native_engine`
  (CTest **12/12**) and native-vs-OCCT parity on the iOS sim (**17/17** `[NCONS]`
  checks). Planar prisms are EXACT (vol/area/centroid rel 0.00e+00, identical face
  tiling); curved revolves match within a deflection bound (vol rel ≤ 2.36e-2,
  watertight). Wired behind an ADDITIVE `cc_set_engine` / `cc_active_engine` toggle
  (**default stays OCCT**). **`#4b` Tier A is now also done at the bar** — holed
  (`cc_solid_extrude_holes` / `_polyholes`) and typed-profile
  (`cc_solid_extrude_profile` / `_profile_polyholes` / `cc_solid_revolve_profile`
  for line / arc / on-axis-arc) construction is NATIVE (host CTest 13/13 + sim
  parity 22/22). Still OCCT-fallthrough (not faked): kind-3 SPLINE profile edges,
  off-axis-arc (torus) / spline surface-of-revolution, and the remaining swept
  solids — loft, sweep, twisted/guided sweep, threads, wrap-emboss (Tiers B–E).
- **No regressions.** Host build + CTest **12/12** (10 existing + new
  `test_native_construct` + `test_native_engine`); `scripts/run-sim-suite.sh` stays
  **221 passed, 0 failed** (re-verified against a freshly rebuilt SIMULATORARM64
  slice carrying the facade + NativeEngine changes).
- **Contained blast radius.** Native math lives under `src/native/math/`, native
  topology under `src/native/topology/`, native tessellation under
  `src/native/tessellate/` (all header-only, unreachable from the facade by design).
  Native construction (`src/native/construct/`) + `NativeEngine`
  (`src/engine/native/`) are engine-wired but reachable ONLY after an explicit
  `cc_set_engine(1)`; the default active engine is unchanged, so no ABI change and
  no behavioural change on the default (OCCT) path.

## Method recap — native rewrite (clean-room, OCCT as oracle)

Native code is implemented **clean-room** from first principles and public
references (*The NURBS Book*: FindSpan A2.1, BasisFuns A2.2, CurvePoint A3.1,
CurveDerivs A3.2, SurfacePoint A3.5, SurfaceDerivs A3.6; de Casteljau for
Bézier). OCCT source is consulted only as a numeric/convention **oracle**
(`gp_*`, `BSplCLib`, `BSplSLib`, `PLib`, `ElSLib`), never copied. fp64
throughout, fixed evaluation order for determinism.

## Verification model — two independent gates over the same code

Because native code carries **no OCCT dependency**, every capability is validated
by two gates, and is "done at the bar" only when BOTH pass AND every existing
suite stays green:

1. **Host unit tests** — the native library compiles and unit-tests with
   `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator,
   asserting analytic/known-value results (a known Bézier point, a transform
   identity, an exact elementary-surface normal). First roadmap gate.
2. **Simulator native-vs-OCCT parity** — on a booted iOS simulator (OCCT linked
   ONLY in the parity test), the native result is compared element-by-element
   against the OCCT oracle within a documented tight fp64 tolerance. Second gate.

## native-math result table

**Host analytic gate:** `test_native_math` (compiled with Homebrew clang 22.1.3,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports
**ALL TESTS PASSED** — 55 analytic assertions across value types, Bézier /
B-spline / NURBS curves, tensor-product surfaces, and elementary surfaces. It is
one of **8/8** CTest targets green (with the 7 pre-existing tests: test_registry,
test_guard, test_scheduler, test_compute_backend, test_parallel_policy,
test_parallel_toggle, test_abi).

**Native-vs-OCCT parity gate** (`tests/sim/native_math_parity.mm`, booted iOS
simulator, arm64): **24 groups, 24 passed, 0 failed.** Max native-vs-OCCT numeric
error per group:

| Group | Max error | Tolerance | Samples |
|---|---|---|---|
| transform-point | 3.773e-14 | 1.0e-09 | 1600 |
| transform-vector | 1.956e-14 | 1.0e-09 | 1600 |
| transform-dir | 5.135e-16 | 1.0e-09 | 1600 |
| bspline-curve-point | 3.053e-15 | 1.0e-09 | 1320 |
| bspline-curve-D1 | 1.876e-14 | 1.0e-08 | 1320 |
| bspline-curve-D2 | 1.936e-14 | 1.0e-07 | 1320 |
| nurbs-curve-point | 3.553e-15 | 1.0e-09 | 1320 |
| nurbs-curve-D1 | 3.371e-14 | 1.0e-08 | 1320 |
| nurbs-curve-D2 | 1.486e-13 | 1.0e-07 | 1320 |
| bspline-surface-point | 1.155e-14 | 1.0e-09 | 2880 |
| bspline-surface-dU | 1.776e-14 | 1.0e-08 | 2880 |
| bspline-surface-dV | 1.773e-14 | 1.0e-08 | 2880 |
| bspline-surface-normal | 9.853e-15 | 1.0e-08 | 2880 |
| nurbs-surface-point | 1.219e-14 | 1.0e-09 | 2880 |
| nurbs-surface-dU | 2.964e-14 | 1.0e-08 | 2880 |
| nurbs-surface-dV | 3.142e-14 | 1.0e-08 | 2880 |
| nurbs-surface-normal | 8.438e-15 | 1.0e-08 | 2880 |
| elem-plane | 2.092e-14 | 1.0e-09 | 300 |
| elem-cylinder | 3.995e-15 | 1.0e-09 | 300 |
| elem-cylinder-normal | 2.109e-15 | 1.0e-09 | 300 |
| elem-cone | 3.437e-15 | 1.0e-09 | 300 |
| elem-cone-normal | 2.026e-15 | 1.0e-08 | 300 |
| elem-sphere | 2.445e-15 | 1.0e-09 | 300 |
| elem-sphere-normal | 3.775e-15 | 1.0e-09 | 300 |

**Overall max numeric error across all groups: 1.486e-13** (nurbs-curve-D2),
~10⁶× under its 1.0e-07 tolerance.

### Files

Native library (OCCT-free, `src/native/math/`):

- `vec.h` — `Vec3` / `Point3` / `Dir3` fp64 value types + vector algebra.
- `transform.h` — 4×4 affine transform (compose / invert / apply to
  point / vector / direction).
- `bezier.h` / `bezier.cpp` — Bézier curve + surface via de Casteljau
  (rational via homogeneous coords + quotient rule).
- `bspline.h` / `bspline.cpp` — FindSpan / BasisFuns / CurvePoint / CurveDerivs /
  de Boor + tensor-product surface eval; NURBS via homogeneous coords.
- `elementary.h` — plane / cylinder / cone / sphere point + unit normal.
- `native_math.h` — umbrella header.

Tests:

- `tests/test_native_math.cpp` — host analytic gate (no OCCT).
- `tests/sim/native_math_parity.mm` — simulator native-vs-OCCT parity gate
  (own `main()`/runner; explicitly SKIPped by `run-sim-suite.sh`).

## native-topology result table

**Host invariant gate:** `test_native_topology` (compiled with Homebrew clang,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports
**13 cases, 0 failed** — data-model / orientation-compose / location /
sub-shape-sharing / geometry-attachment / stable-id / deterministic-enumeration /
explorer-order / ancestry-symmetry / `BRep_Tool`-accessor / repeat-run-equality
invariants. It is one of **9/9** CTest targets green (with the 8 pre-existing
tests: test_registry, test_guard, test_scheduler, test_compute_backend,
test_parallel_policy, test_parallel_toggle, test_abi, test_native_math). The
test lives under `tests/native/` and is registered with a basename→source
override (`test_native_topology_SRC` → `tests/native/test_native_topology.cpp`).

**Native-vs-OCCT parity gate** (`tests/sim/native_topology_parity.mm`, booted
iOS simulator, arm64): a test-only importer loads OCCT `TopoDS_Shape`s into the
native model and compares against the OCCT oracle (`TopoDS`, `TopAbs`, `TopExp`,
`TopTools`, `BRep_Tool`, `TopLoc_Location`). **3 shapes × 5 checks = 15 passed,
0 failed.**

| Shape | Sub-shapes | mapshapes-order | ancestry (edge→faces) | accessors maxErr (tol 1.0e-09) | orientation |
|---|---|---|---|---|---|
| box | V8 E12 wire6 F6 shell1 solid1 | PASS | 12 edges match | 0.000e+00, surfType match | 34 sub-shapes match |
| cylinder | V2 E3 wire3 F3 shell1 solid1 | PASS | 3 edges match | 0.000e+00, surfType match | 13 sub-shapes match |
| filleted-box | V24 E56 wire26 F26 shell1 solid1 | PASS | 56 edges match | 0.000e+00, surfType match | 134 sub-shapes match |

**Overall max accessor error across all shapes: 0.000e+00** (world points, curve
ranges, and surface parameters read back bit-identically to the OCCT oracle;
surface-type classification matches on every face).

### Files

Native library (OCCT-free, header-only, `src/native/topology/`):

- `shape.h` — `ShapeType` / `Orientation` enums, underlying/use split (shared
  immutable underlying + cheap `(underlying, orientation, location)` use),
  orientation compose, `Location`, and attached geometry (vertex point+tol,
  edge curve+range+pcurves, face surface+ordered wires+tol).
- `explore.h` — deterministic depth-first walk, stable sub-shape ids
  (`MapShapes` analogue), lazy `Explorer`, and `Ancestors`
  (`MapShapesAndAncestors` analogue).
- `accessors.h` — `BRep_Tool`-style free-function accessors (`pnt`, `tolerance`,
  `curve`, `curve_on_surface`, `surface`) resolving geometry through the use's
  location.
- `native_topology.h` — umbrella header.

Tests:

- `tests/native/test_native_topology.cpp` — host invariant gate (no OCCT).
- `tests/sim/native_topology_parity.mm` — simulator native-vs-OCCT parity gate
  (own runner; explicitly SKIPped by `run-sim-suite.sh`).

### Deferred (recorded, not blocking the bar)

- **Non-manifold / degenerate edges** and **seam edges** (two pcurves on the same
  face) are not yet exercised by a fixture — deferred to native-construction,
  which will generate such edges.
- **`curve_on_surface` pcurve subtleties** beyond face-keyed selection (pcurve
  continuity, degenerate edges with no 3D curve) deferred alongside.
- **`CompSolid` `ShapeType`** and **`Internal`/`External` orientations** are
  reserved in the enums but not exercised by a fixture.
- The parity **face-with-a-hole** fixture is deferred (no OCCT holed-face fixture
  in the importer path yet); inner-wire read-back is covered by a host test.

## native-tessellation result table

**Host invariant gate:** `test_native_tessellate` (compiled with Homebrew clang,
`-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) reports all
deflection-bound / on-surface / trimming / watertightness / area-volume
convergence / determinism invariant tests green. It is one of **10/10** CTest
targets green (with the 9 pre-existing: test_registry, test_guard, test_scheduler,
test_compute_backend, test_parallel_policy, test_parallel_toggle, test_abi,
test_native_math, test_native_topology). The test lives under `tests/native/`.

**Native-vs-OCCT `BRepMesh` property-parity gate**
(`tests/sim/native_tessellation_parity.mm`, booted iOS simulator, arm64): the
same OCCT shape is meshed by the native mesher and by OCCT
`BRepMesh_IncrementalMesh` at a matched deflection, then compared on
watertightness / manifoldness, bbox, total surface area, and enclosed volume
(triangle count/topology NOT compared — tessellation is an approximation). **All
20 checks PASS across 4 shapes.** `[NTESS]` per-shape results:

| Shape | Watertightness | tris | bbox maxCornerΔ (tol) | area native / occtMesh / exact · relMesh / relExact (tol) | volume native / occtMesh / exact · relMesh / relExact (tol) | vertices-on-surface maxDist (defl) |
|---|---|---|---|---|---|---|
| box | watertight, boundaryEdges=0 | 12 | 0.0000e+00 (2.0e-1) | relMesh 0.000e+00 / relExact 0.000e+00 (2.0e-2) | relMesh 0.000e+00 / relExact 0.000e+00 (2.0e-2) | 0.000e+00 |
| cylinder | watertight, boundaryEdges=0 | 88 | 4.657e-02 | relMesh 2.826e-03 / relExact 5.838e-03 (2.0e-2) | relMesh 6.017e-03 / relExact 1.239e-02 (2.0e-2) | 0.000e+00 |
| sphere | watertight, boundaryEdges=0 | 1680 | 2.950e-02 | relMesh 2.429e-03 / relExact 4.656e-03 (2.0e-2) | relMesh 5.212e-03 / relExact 9.290e-03 (2.0e-2) | 5.687e-15 |
| filleted-box | watertight, boundaryEdges=0 | 332 | 4.440e-16 | relMesh 1.790e-03 / relExact 2.748e-03 (5.0e-2) | relMesh 2.004e-03 / relExact 3.012e-03 (5.0e-2) | 8.882e-16 |

Watertightness: **ALL four closed solids now mesh watertight (`boundaryEdges=0`)** —
box (12 tris), cylinder (88 tris), sphere (1680 tris), filleted-box (332 tris), each
2-manifold with **0 open/boundary edges**. The curved shared-edge stitch is
implemented — the mesher is a two-stage pipeline (STAGE 1 `edge_mesher.h` discretizes
each unique topological edge ONCE into a shared deflection-based 1D fraction list,
cached by the edge's `TShape` node; STAGE 2 `face_mesher.h` pins BOTH adjacent faces'
boundary vertices to those SAME fractions mapped through each face's pcurve, so
`S_face(pcurve(f)) == C_edge(f)`), exactly as OCCT `BRepMesh` builds its edge
discretization before meshing faces, so a cylinder's circular cap↔side seam (formerly
`boundaryFrac~0.119`, 2-manifold-bounded-open) and a fillet's blend seams now weld
closed. Gate-2 now REQUIRES `isWatertight()` for every closed solid — there is no
longer a weaker `manifold-bounded-open` pass. The host Gate-1 regressions
(`cylinder_solid_watertight_curved_seam`, `cylinder_solid_watertight_converges`)
confirm the cylinder solid is watertight (`boundaryEdges==0`) at every deflection with
area/volume converging to the closed form. Vertices-on-surface deflection residuals
are at machine epsilon (≤ 5.7e-15) — every emitted vertex is produced by `native-math`
`value(u,v)`, on the surface by construction.

**Spec conformance:** the `native-tessellation` spec's watertight requirement
("Mesh a whole Solid by stitching shared edges into a watertight mesh" — *"For a
closed solid the resulting mesh SHALL be watertight: every mesh edge SHALL be shared
by exactly two triangles… no naked/boundary edges"*) is now **genuinely met for every
closed solid**, including CURVED shared edges. Previously only planar-aligned (box)
and seam/pole (sphere) edges welded and the requirement was met with a documented
carve-out for curved seams; that carve-out is gone. The host regression hard-requires
`isWatertight()` + `boundaryEdgeCount()==0` for closed solids — there is no weaker
bounded-open acceptance path.

### Files

Native library (OCCT-free, header-only, `src/native/tessellate/`):

- `mesh.h` — `TriMesh`/`FaceMesh`/`SolidMesh` representation (fp64 vertex buffer
  with position + optional normal + per-vertex `(u,v)`, `uint32` CCW triangle
  index buffer, per-triangle face-id tag) + mesh-derived area/volume.
- `surface_eval.h` — deflection-driven UV-grid step selection over `native-math`
  `value`/`normal`/derivatives.
- `edge_mesher.h` — **STAGE 1**: `EdgeCache` — shared per-edge 1D discretization.
  Each unique topological edge is discretized ONCE into a deflection-based fraction
  list (3D-curvature sized), cached by edge `TShape` identity; both adjacent faces
  reuse it. This is the seam that makes CURVED shared edges weld watertight.
- `trim.h` — parameter-space wire flattening (pcurves → UV polygons) + even-odd
  point-in-polygon keep test (outer ∧ ¬holes); `appendEdgeSamplesAtFracs` samples
  an edge's pcurve at the shared STAGE-1 fractions.
- `uv_triangulate.h` — robust ear-clipping triangulation of a UV polygon (with
  bridged holes) for genuinely-trimmed faces (degeneracy-free; no incircle predicate).
- `face_mesher.h` — **STAGE 2**: boundary pinned to the shared edge discretization;
  structured-grid path for full-parametric-rectangle faces (boundary rows = shared
  samples) and ear-clip path for trimmed faces; produces a `FaceMesh`.
- `solid_mesher.h` — per-face meshing via `Explorer` sharing ONE `EdgeCache` +
  spatial-hash vertex weld (`VertexWelder`, weld tol = ½·deflection) into a
  `SolidMesh`.
- `gpu_sample.h` — optional `#ifdef CYBERCAD_HAS_METAL` fp32 UV-grid fill for
  GPU-eligible faces (display-only; correctness stays on the fp64 CPU path).
- `native_tessellate.h` — umbrella header.

Tests:

- `tests/native/test_native_tessellate.cpp` — host invariant gate (no OCCT).
- `tests/native/checks_tessellate.cpp` — shared property-check helpers.
- `tests/sim/native_tessellation_parity.mm` — simulator native-vs-OCCT `BRepMesh`
  property-parity gate (own runner; explicitly SKIPped by `run-sim-suite.sh`).
- `tests/sim/native_tessellate_parity.mm` — companion sim parity source (own
  `main()`; SKIPped by `run-sim-suite.sh`).

### Resolved in this iteration

- **Curved shared-edge stitch** — RESOLVED. The two-stage mesher (STAGE 1
  `edge_mesher.h` shared per-edge 1D discretization, STAGE 2 `face_mesher.h` pins both
  adjacent faces' boundaries to it) places coincident vertices on CURVED shared edges
  (cylinder cap↔side circle, fillet blend seams), so those solids now weld fully
  watertight (`boundaryEdges==0`). Was: 2-manifold-bounded-open.

### Deferred (recorded, not blocking the bar — none affect watertightness)

- **Ear-clip constrained re-triangulation** of boundary-straddling trim cells —
  currently kept/dropped by centroid, so the hole silhouette is resolved to the
  grid step (verified within a few-percent area bound) rather than clipped exactly.
- **GPU fp32 sampling backend** — compiled behind `CYBERCAD_HAS_METAL` but
  correctness only CPU-verified in this environment (host gate runs `METAL=OFF`);
  the GPU path is display-only by design.
- **Adaptive refinement quality / seam / degenerate faces** — grid density is
  deflection-driven per direction; adaptive per-cell subdivision and explicit
  degenerate/seam-face hardening are follow-ups.

## native-construction result table

**First engine-wired capability.** A native `NativeEngine : IEngine`
(`src/engine/native/`) implements exactly two construction ops natively and
**falls through to OCCT** (or the host stub) for everything else. The facade gains
an additive opt-in toggle `cc_set_engine(int)` / `cc_active_engine()` (modelled on
`cc_set_parallel`); the **default active engine stays OCCT**, so no existing suite
changes unless a caller explicitly opts in.

### What is native vs what falls through to OCCT

| `cc_*` build op | Engine | Native geometry |
|---|---|---|
| `cc_solid_extrude` (closed polygon profile) | **NATIVE** | bottom+top `Plane` caps + one planar quad `Plane` side face per profile edge; side edges `Line` |
| `cc_solid_revolve` (LINE-SEGMENT profile) | **NATIVE** | per-segment surface of revolution — parallel→`Cylinder`, perpendicular→`Plane`, oblique→`Cone`; circular edges `Circle`; full 360° closes shell, partial angle adds two `Plane` meridian caps |
| `cc_solid_extrude_holes` (outer + CIRCULAR holes) | **NATIVE** (#4b Tier A) | outer prism + per hole a TRUE `Circle` cap edge + one inward `Cylinder` wall, reversed circle as inner cap wire |
| `cc_solid_extrude_polyholes` (outer + POLYGON holes) | **NATIVE** (#4b Tier A) | outer prism + per hole an inner ring of `Line` edges + N inward `Plane` walls, reversed ring as inner cap wire |
| `cc_solid_extrude_profile` / `_profile_polyholes` (TYPED outer: kind 0 line / 1 arc / 2 full-circle) | **NATIVE** (#4b Tier A) | line→`Plane` side, arc→TRUE `Circle` edge + `Cylinder` wall (one bounded patch per ≤180° span), full-circle→`Cylinder` wall + disc caps; + circular/polygon holes |
| `cc_solid_revolve_profile` (TYPED: line, on-axis arc/semicircle) | **NATIVE** (#4b Tier A) | line→`Plane`/`Cylinder`/`Cone`, on-axis arc→`Sphere` band; full 2π closes, partial adds two `Plane` meridian caps |
| `cc_solid_extrude_profile` kind-3 SPLINE outer edge | OCCT-fallthrough (#4b) | native builder returns NULL; fall-through verified (vol rel 0.00e+00) |
| `cc_solid_revolve_profile` off-axis arc (TORUS) / any spline-revolve | OCCT-fallthrough (#4b) | no native `Torus` surface / spline surface-of-revolution yet; fall-through verified (torus vol rel 0.00e+00) |
| `cc_solid_loft`, `cc_solid_loft_wires` | OCCT-fallthrough (#4b Tier B) | deferred (ruled/skinned surfacing) |
| `cc_solid_sweep`, `cc_twisted_sweep`, `cc_guided_sweep`, `cc_loft_along_rail` | OCCT-fallthrough (#4b Tier C) | deferred (pipe/sweep surfacing) |
| `cc_helical_thread`, `cc_tapered_thread`, `cc_tapered_shank` | OCCT-fallthrough (#4b Tier D) | deferred (helical swept solids) |
| `cc_wrap_emboss` | OCCT-fallthrough (#4b Tier E) | deferred |
| every feature / boolean / query / transform / exchange op | OCCT-fallthrough | out of the construction capability; delegated |

The `NativeEngine` additionally serves native `tessellate`, `mass_properties`,
`bounding_box`, and `subshape_ids` on its OWN native bodies (bbox derived from the
tessellated mesh, since a revolved solid's B-rep vertices sit only at angular
stations); every other method forwards to the fallback unchanged. Feeding a
native-built shape into an OCCT-only op is not supported in this change.

**Host gate (Gate 1):** `test_native_construct` + `test_native_engine` (Homebrew
clang, `-std=c++20`, `CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`) assert
native builds with NO OCCT — box (exact vol/area/6-faces/centroid/bbox/watertight),
triangle prism (watertight, exact vol = area×depth), L-prism, full-turn tube (9π),
quarter-turn tube (9π/4), cone (4π), all within the deflection bound; plus engine
delegation + `cc_set_engine` toggle + deferred-op fall-through. CTest **12/12**
green (the 10 pre-existing + these two new targets).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_construct_parity.mm`,
booted iOS simulator, arm64, driven THROUGH the `cc_*` facade under
`cc_set_engine(0/1)` with the OCCT default restored in teardown. **17/17 `[NCONS]`
checks PASS.** Per-shape native-vs-OCCT deltas:

| Shape | Op | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ | faces (o / n) | tessellate |
|---|---|---|---|---|---|---|---|
| box | extrude, planar | 30 / 30 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 1.00e-07 | 6 / 6 identical tiling | watertight, 12 tris, meshVolRel 0.00e+00 |
| triangle-prism | extrude, planar | 30 / 30 · **0.00e+00** | 0.00e+00 | 0.00e+00 | 1.00e-07 | 5 / 5 identical | watertight, 8 tris, meshVolRel 0.00e+00 |
| cylinder-tube | revolve 360°, curved | 28.2743 / 27.6063 · **2.36e-02** | 1.24e-02 | 1.11e-15 | 4.37e-02 | 4 / 12 angular tiling (n=3×o) | watertight, 168 tris, meshVolRel 1.55e-02 |
| partial-revolve-90 | revolve 90°, curved | 7.06858 / 6.9344 · **1.90e-02** | 8.19e-03 | 1.51e-02 | 1.00e-07 | 6 / 6 identical | watertight, 44 tris, meshVolRel 1.25e-02 |

Tolerances: planar prisms are EXACT (vol/area/centroid rel = 0, identical face
tiling); curved revolves match OCCT within a deflection bound (vol rel ≤ 2.36e-2,
tol v=5e-2 c=1e-1; bbox tol 1e-1) and are watertight. **Fall-through parity:** with
native active, `cc_boolean(fuse)` returns id=11 vol=14 (expect 14) — delegated to
OCCT, proving no native interception of deferred ops.

**Documented representational difference (not a geometric mismatch):** the native
builder emits per-face edges / per-patch vertices (proper edge/vertex SHARING
deferred) and tiles a full-turn surface of revolution into < π angular patches
(periodic-face construction deferred), so native V/E and the full-turn face count
differ from OCCT's shared/periodic representation while the SOLID is geometrically
identical (volume/area/bbox/watertight all match). The parity gate asserts face
count where tiling matches (prisms, partial revolve) and an integer-multiple
relation for the full-turn revolve.

**Honest scope split — core done, advanced swept solids are follow-up.** The
core #4 change delivered the CORE construction ops (extrude + line-segment revolve)
at the bar. The advanced swept solids — loft, sweep, twisted/guided sweep, threads,
holed/typed-profile extrude, arc/spline revolve — were EXPLICITLY DEFERRED, fall
through to OCCT (not faked), and are tracked as a follow-up (`#4b`) within the
capability. **`#4b` Tier A (holed + typed-profile extrude + typed-profile revolve)
is now done at the bar** — see below. Tiers B (loft) / C (sweep) / D (threads) /
E (wrap-emboss) remain OCCT-fallthrough.

### `#4b` Tier A result table — holed / typed-profile extrude + typed-profile revolve

**Change:** `add-native-construction-profiles`. Built in `src/native/construct/profile.h`
(OCCT-free, host-buildable; unified `build_prism_with_holes` / `build_prism_profile` /
`build_revolution_profile`) + a robustified multi-hole cap triangulator in
`src/native/tessellate/uv_triangulate.h`. Engine-wired behind the SAME additive
`cc_set_engine(1)` toggle (default stays OCCT).

**Host gate (Gate 1):** `test_native_profile` (12 cases — circular / polygon / multi-hole
/ combined holes watertight with exact-or-convergent volume; full-circle extrude →
cylinder; on-axis arc revolve → sphere 36π; partial-turn revolve; typed line/arc extrude)
+ 5 new `test_native_engine` facade cases. Host CTest **13/13** green; `test_native_tessellate`
stayed green (box / sphere / cylinder / filleted-box watertight, `boundaryEdges==0`).

**Native-vs-OCCT parity gate (Gate 2)** — `tests/sim/native_construct_profiles_parity.mm`
through the `cc_*` facade under `cc_set_engine(0/1)`, OCCT default restored in teardown.
**All 22 `[NCPROF]` checks PASS.** Per-op native (n) vs OCCT (o) deltas:

| `cc_*` op / sub-case | Engine | mass vol (o / n) · relVol | area rel | centroidΔ | bbox maxCornerΔ (tol) | faces (o→n) | tessellate |
|---|---|---|---|---|---|---|---|
| `cc_solid_extrude_holes` circular | **NATIVE** | 349.735 / 351.192 · 4.17e-03 | 9.40e-04 | 2.66e-15 | 1.00e-07 (1e-1) | 7→7 | watertight, 108 tris, meshVolRel 2.38e-03 |
| `cc_solid_extrude_polyholes` square | **NATIVE** | 288 / 288 · **1.97e-16** | 1.69e-16 | 0 (EXACT) | 1.00e-07 (1e-6) | 10→10 | watertight, 32 tris, meshVolRel 0 |
| `cc_solid_extrude_profile` line+arc | **NATIVE** | 18.8496 / 18.3688 · 2.55e-02 | 1.02e-02 | 1.09e-02 | 1.00e-07 (5e-2) | 4→4 | watertight, 64 tris, meshVolRel 1.96e-02 |
| `cc_solid_revolve_profile` line-tube | **NATIVE** | 28.2743 / 27.6063 · 2.36e-02 | 1.24e-02 | 1.11e-15 | 4.37e-02 (1e-1) | 4→12 (k=3 tiling) | watertight, 168 tris, meshVolRel 1.55e-02 |
| `cc_solid_revolve_profile` arc-sphere | **NATIVE** | 113.097 / 107.473 · 4.97e-02 | 2.52e-02 | 2.28e-16 | 9.05e-02 (1e-1) | 1→3 | watertight, 780 tris, meshVolRel 3.16e-02 |
| `cc_solid_extrude_profile` kind-3 SPLINE outer | OCCT-fallthrough | 45.6 / 45.6 · **0.00e+00** | — | — | — | — | delegated to OCCT (NULL native → fallback) |
| `cc_solid_revolve_profile` off-axis arc (TORUS) | OCCT-fallthrough | 98.696 / 98.696 · **0.00e+00** | — | — | — | — | delegated to OCCT (NULL native → fallback) |

Tolerances: polygon-hole extrude is EXACT (vol/area/centroid rel = 0, identical face
tiling); curved ops match OCCT within a deflection bound (largest native mass delta
4.97e-02 on arc-sphere, within its 5e-02 tol; bbox tol 1e-1) and are all watertight.
The two deferred sub-cases (kind-3 spline extrude, off-axis-circle → torus revolve)
transparently delegate to OCCT (vol rel 0.00e+00) — fall-through proof, no native
interception. A pure spline-*revolve* takes the same NULL→fallback path as the torus
and stays OCCT-fallthrough. A kind-1 ARC extrude edge is a TRUE `Circle` cap edge + one
bounded (non-periodic) `Cylinder` patch per ≤180° span (split threshold π for the EXTRUDE
wall vs 120° for the revolve), matching OCCT's single cylindrical face — not a chord
polyline.

**No regressions.** Host CTest **13/13** (incl. `test_native_tessellate`);
`scripts/run-sim-suite.sh` **221 passed, 0 failed** against a freshly rebuilt
SIMULATORARM64 slice (determinism + IGES/STEP round-trips PASS). Zero source fixes
required during verification.

**Where OCCT is STILL required after Tier A (reality):** booleans (fuse/cut/common),
fillets/chamfers/offsets/shell, features, data exchange (STEP/IGES), shape healing, and
the remaining swept solids — loft (Tier B), sweep + twisted/guided/rail variants (Tier C),
threads (Tier D), wrap-emboss (Tier E) — plus kind-3 SPLINE profile edges, off-axis-arc
(torus) revolve, and any spline surface-of-revolution. All of these fall through to OCCT
via `NativeEngine` (native builder returns NULL → OCCT), not faked.

### Files

- `src/native/construct/construct.h` — OCCT-free `extrudePolygon` / `revolveSegments`
  returning native `topology::Shape` (host-buildable, no `IEngine`/OCCT).
- `src/engine/native/native_engine.{h,cpp}` — `NativeEngine : IEngine`; native
  `solid_extrude`/`solid_revolve` + native tessellate/mass/bbox/subshape on native
  bodies; forwards the rest to a held fallback `shared_ptr<IEngine>` (OCCT under
  `CYBERCAD_HAS_OCCT`, stub on host).
- `include/cybercadkernel/cc_kernel.h` + `src/facade/cc_kernel.cpp` — additive
  `cc_set_engine` / `cc_active_engine` (default OCCT; host stub no-op → reports 0).
- `src/native/tessellate/trim.h` — `isFullRectangle(..., requireCorners)`: a PLANAR
  face's fast-path now also requires the loop to hit all four box corners, so a
  convex polygon cap (triangle/hexagon) is ear-clipped instead of filled as its UV
  bbox (one real caller, `face_mesher.h`, updated; OCCT tessellation path untouched).

Tests:

- `tests/native/test_native_construct.cpp` — host construction gate (no OCCT).
- `tests/test_native_engine.cpp` — host engine delegation + toggle gate (stub fallback).
- `tests/native/checks_construct.cpp` — shared parity property-check helpers.
- `tests/sim/native_construct_parity.mm` — simulator native-vs-OCCT parity gate
  through the facade (own runner; SKIPped by `run-sim-suite.sh`).

## Regression evidence

- Host build + CTest with Homebrew clang, `-DCYBERCAD_HAS_OCCT=OFF
  -DCYBERCAD_HAS_METAL=OFF`, fresh build dir: configure OK, build OK (no
  warnings/errors), **CTest 12/12 passed, 0 failed** (10 existing +
  `test_native_construct` + `test_native_engine`).
- `scripts/run-sim-suite.sh` (iphonesimulator arm64): still
  **== 221 passed, 0 failed ==** (verified twice). To confirm HONESTLY against the
  facade+NativeEngine changes (the prebuilt sim lib predated them), the
  SIMULATORARM64 slice was REBUILT from working-tree sources (24 TUs,
  `-DCYBERCAD_HAS_OCCT`, arm64 simulator — `native_engine.cpp` compiles cleanly
  under `CYBERCAD_HAS_OCCT` with `OcctEngine` as the fallthrough target) and the
  suite re-run against the fresh lib. The `.mm` parity tests (`native_math_parity.mm`,
  `native_topology_parity.mm`, `native_tessellate_parity.mm` /
  `native_tessellation_parity.mm`, and the new `native_construct_parity.mm`) are in
  the script's SKIP list and carry their own `main()`, so the OCCT-only
  221-assertion suite count is unchanged. The suite never calls `cc_set_engine`, so
  it exercises the pure OCCT path exactly as before.
- Isolation / blast radius: capabilities #1–#3 (native math `src/native/math/`,
  native topology `src/native/topology/`, native tessellation
  `src/native/tessellate/`) remain unreachable from the `cc_*` facade by design.
  Capability #4 is the **first engine-wired** capability, but the wiring is a safe,
  ADDITIVE opt-in: `NativeEngine` (`src/engine/native/native_engine.cpp`) and the
  native construction library (`src/native/construct/`) are compiled into the
  library via `GLOB_RECURSE src/*.cpp` (OCCT excluded by regex), but they enter a
  `cc_*` call path ONLY after `cc_set_engine(1)`. The default engine is unchanged
  (`cc_set_engine(0)` restores `create_default_engine()` — OCCT where linked, stub
  on host), so every existing suite that never toggles is byte-for-byte unaffected.
  The ONE shared-code behavioural change — `isFullRectangle()` gaining a
  `requireCorners` arg in `src/native/tessellate/trim.h` — has exactly one real
  caller (`face_mesher.h`, updated) and does not touch the OCCT tessellation path;
  it is exercised by `test_native_tessellate` + `test_native_construct` (all green).

## Per-capability status

| # | Capability | Status | Notes |
|---|---|---|---|
| 1 | `native-math` | **done at the bar** | Both gates green (55 host asserts + 24 parity groups, max err 1.486e-13); no regressions; not yet engine-wired (by design). |
| 2 | `native-topology` | **done at the bar** | Both gates green (13 host cases + 3 shapes × 5 parity checks = 15/15, max accessor err 0.000e+00); no regressions (host CTest 9/9, `run-sim-suite.sh` 221/221); header-only, not engine-wired (by design). Deferred: non-manifold/degenerate + seam edges, `CompSolid`/`Internal`/`External`, holed-face parity fixture. |
| 3 | `native-tessellation` | **done at the bar** | Both gates green (host `test_native_tessellate` + sim native-vs-OCCT `BRepMesh` parity, All 20 checks PASS across 4 shapes; ALL four closed solids watertight `boundaryEdges==0`; area/volume relMesh ≤ 6.0e-3, relExact ≤ 1.24e-2, bbox maxCornerΔ ≤ 4.66e-2, on-surface residual ≤ 5.7e-15); no regressions (host CTest 10/10, `run-sim-suite.sh` 221/221); header-only `src/native/tessellate/`, not engine-wired by design. RESOLVED: curved shared-edge stitch (two-stage shared per-edge discretization) — cylinder/filleted-box now watertight. Deferred (genuinely minor, not watertightness): ear-clip trim re-triangulation quality, adaptive per-cell refinement, GPU fp32 path CPU-verified only. |
| 4 | `native-construction` | **done at the bar** | Native `cc_solid_extrude` (closed polygon → prism: bottom/top planar caps + one planar quad per profile edge) and native `cc_solid_revolve` for **LINE-SEGMENT** profiles (segments → plane / cylinder / cone faces of revolution; full 360° closes, partial adds planar caps) — full native topology + geometry under `src/native/construct/construct.h`, OCCT-free/host-buildable. Wired through a new `NativeEngine : IEngine` (`src/engine/native/`) that serves these ops + native tessellate / mass / bbox / **subshape_ids** on its own native bodies and FALLS THROUGH to the OCCT engine (or the stub on host) for every other capability. Facade toggle `cc_set_engine(int)` / `cc_active_engine()` (additive, like `cc_set_parallel`; **default stays OCCT** so existing suites are unchanged). **Both gates green.** Host: `test_native_engine` + `test_native_construct` assert native builds with NO OCCT — boxes (exact vol/area/6-faces/centroid/bbox/watertight), a **triangle prism** (now watertight, exact vol = area×depth, via the tessellator cap-fill fix below), an L-prism, a full-turn tube (9π), a quarter-turn tube (9π/4) and a cone (4π), within the deflection bound; CTest **12/12**. Sim native-vs-OCCT parity (`native_construct_parity.mm`, driven through the `cc_*` facade under `cc_set_engine(0/1)`): **17/17** across box / triangle-prism / cylinder-tube / partial-revolve — mass (vol/area/centroid), bbox, face count, watertight tessellation, plus the fallthrough boolean (native→OCCT) all match. No regressions (`run-sim-suite.sh` **221/221**, `native_tessellation_parity.mm` **20/20**). Three fixes landed here: (a) the tessellator `isFullRectangle` fast-path now, for a PLANAR face, also requires the loop to hit all four box corners, so a convex polygon cap (triangle/hexagon) is ear-clipped instead of filled as its bbox — native extrude of ANY simple polygon now meshes watertight with the exact volume (`trim.h`); (b) `NativeEngine::bounding_box` derives from the tessellated mesh (a revolved solid's B-rep vertices sit only at angular stations, so a vertex-only AABB missed the circular extremes); (c) `NativeEngine::subshape_ids` is native for native bodies (Vertex/Edge/Face counts via the native Explorer). EXPLICITLY DEFERRED to OCCT (not faked, falls through): loft, sweep, twisted/guided sweep, threads, holed/typed-profile extrude variants, revolve of ARC/SPLINE profiles. DOCUMENTED REPRESENTATIONAL DIFFERENCE (not a geometric mismatch): the native builder emits per-face edges / per-patch vertices (proper edge/vertex SHARING deferred) and tiles a full-turn surface of revolution into <π angular patches (periodic-face construction deferred), so native V/E and the full-turn face count differ from OCCT's shared/periodic representation while the SOLID is geometrically identical (volume/area/bbox/watertight all match) — the parity gate asserts face-count where the tiling matches (prisms, partial revolve) and an integer-multiple relation for the full-turn revolve. |
| 4b | `native-construction` (advanced swept solids) | ◐ Tier A done at the bar; B–E follow-up | **Tier A (`add-native-construction-profiles`) done at the verification bar:** `cc_solid_extrude_holes` (circular holes → TRUE `Circle` edge + `Cylinder` wall), `cc_solid_extrude_polyholes` (polygon holes), `cc_solid_extrude_profile` / `_profile_polyholes` (typed line/arc/full-circle outer + holes), `cc_solid_revolve_profile` (line → Plane/Cylinder/Cone, on-axis arc → Sphere) are NATIVE (`src/native/construct/profile.h`). Both gates green: host `test_native_profile` + `test_native_engine` CTest **13/13** (no OCCT); sim native-vs-OCCT parity `native_construct_profiles_parity.mm` **22/22** — 5 native families (polyhole EXACT rel 1.97e-16; curved vol rel ≤ 4.97e-2, all watertight) + 2 fall-through families (kind-3 spline extrude, off-axis-arc torus revolve, vol rel 0.00e+00). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): kind-3 SPLINE edges, off-axis-arc (torus) / spline surface-of-revolution, and Tier B loft / C sweep+variants / D threads / E wrap-emboss. |
| 5 | `native-booleans` | ☐ next (**research-grade**) | Native robust B-rep booleans — the hardest, longest-lived OCCT dependency (surface-surface intersection, robust classification, shape healing). Will land progressively hardened and verified against OCCT (BOPAlgo oracle), not production-robust day one. |
| 6–7 | blends → exchange | ☐ planned | Proposed as each begins. |
| 8 | `drop-occt` | ☐ planned | Unlink OCCT once every capability is native. |

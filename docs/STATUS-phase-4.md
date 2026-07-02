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
- **No regressions.** Host build + CTest **10/10** (9 existing + new
  `test_native_tessellate`); `scripts/run-sim-suite.sh` stays **221 passed, 0
  failed**.
- **Zero blast radius.** Native math lives under `src/native/math/`, native
  topology under `src/native/topology/`, and native tessellation under
  `src/native/tessellate/` (header-only). None is reachable from the `cc_*` facade
  or the engine — no ABI change, no engine wiring in any of the three capabilities.

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

## Regression evidence

- Host build + CTest with Homebrew clang, `-DCYBERCAD_HAS_OCCT=OFF
  -DCYBERCAD_HAS_METAL=OFF`, fresh build dir: configure OK, build OK (no
  warnings/errors), **CTest 10/10 passed, 0 failed** (9 existing +
  `test_native_tessellate`).
- `scripts/run-sim-suite.sh` (iphonesimulator arm64): still
  **== 221 passed, 0 failed ==** (8 sources compiled/linked, ran on a booted
  device). The `.mm` parity tests (`native_math_parity.mm`,
  `native_topology_parity.mm`, and the new `native_tessellate_parity.mm` /
  `native_tessellation_parity.mm`) are in the script's SKIP list and carry their
  own `main()`, so the OCCT-only 221-assertion suite count is unchanged.
- Isolation: native math (`src/native/math/`), native topology
  (`src/native/topology/`), and native tessellation (`src/native/tessellate/`) are
  not referenced from `src/facade/cc_kernel.cpp` or `src/engine/*`. Native
  tessellation is **header-only** (no `.cpp`), so the `src/*.cpp` library GLOB
  picks up no new sources and cannot affect existing TUs — it enters the build only
  via the new `test_native_tessellate` executable. Native topology is likewise
  header-only; native math `.cpp` files compile into the library but are dead code
  for every `cc_*` path.

## Per-capability status

| # | Capability | Status | Notes |
|---|---|---|---|
| 1 | `native-math` | **done at the bar** | Both gates green (55 host asserts + 24 parity groups, max err 1.486e-13); no regressions; not yet engine-wired (by design). |
| 2 | `native-topology` | **done at the bar** | Both gates green (13 host cases + 3 shapes × 5 parity checks = 15/15, max accessor err 0.000e+00); no regressions (host CTest 9/9, `run-sim-suite.sh` 221/221); header-only, not engine-wired (by design). Deferred: non-manifold/degenerate + seam edges, `CompSolid`/`Internal`/`External`, holed-face parity fixture. |
| 3 | `native-tessellation` | **done at the bar** | Both gates green (host `test_native_tessellate` + sim native-vs-OCCT `BRepMesh` parity, All 20 checks PASS across 4 shapes; ALL four closed solids watertight `boundaryEdges==0`; area/volume relMesh ≤ 6.0e-3, relExact ≤ 1.24e-2, bbox maxCornerΔ ≤ 4.66e-2, on-surface residual ≤ 5.7e-15); no regressions (host CTest 10/10, `run-sim-suite.sh` 221/221); header-only `src/native/tessellate/`, not engine-wired by design. RESOLVED: curved shared-edge stitch (two-stage shared per-edge discretization) — cylinder/filleted-box now watertight. Deferred (genuinely minor, not watertightness): ear-clip trim re-triangulation quality, adaptive per-cell refinement, GPU fp32 path CPU-verified only. |
| 4 | `native-construction` | ☐ planned | Next up — primitive + swept-solid construction building native topology + geometry; first capability likely engine-wired for a `cc_*` comparison. |
| 5–7 | booleans → blends → exchange | ☐ planned | Proposed as each begins. |
| 8 | `drop-occt` | ☐ planned | Unlink OCCT once every capability is native. |

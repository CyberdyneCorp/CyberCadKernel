## Why

Phase 4 drops OCCT one capability at a time until it can be unlinked entirely
(`openspec/NATIVE-REWRITE.md`). Capability #1 (`native-math`) delivered the
OCCT-free fp64 surface evaluator (point + `dU`/`dV` + normal for
Bézier/B-spline/NURBS/elementary); capability #2 (`native-topology`) delivered
the OCCT-free B-rep model (Face → surface + ordered outer/inner wires + pcurves,
`Explorer`, ancestry, `BRep_Tool`-style accessors). The next capability every
consumer needs — to display a body, to feed downstream algorithms, and to
verify construction/booleans visually — is **tessellation**: turning a B-rep
into a triangle mesh at a bounded chord-height (deflection) tolerance. This is
capability `#3` (`native-tessellation`) in the locked dependency order; it
consumes `native-math` surface evaluation and `native-topology` faces directly
and is the first native capability whose *output* (a mesh) is a concrete,
independently checkable artifact.

Making the mesher OCCT-free and host-buildable is what unlocks the two-gate
verification model. Because it links no OCCT and needs no simulator, it can be
unit-tested on the host with `clang++ -std=c++20` against **analytic** ground
truth: a plane/cylinder/sphere/holed-plane whose exact area, enclosed volume,
and surface points are known in closed form, so deflection bound, on-surface
residual, watertightness, and area/volume convergence are all asserted without
OCCT. It can *also* be compared against the OCCT `BRepMesh_IncrementalMesh`
oracle on the simulator — but because a mesh is an **approximation**, the parity
gate asserts tolerance-based *properties* (bbox, total area, enclosed volume,
watertightness within tolerance at a matched deflection), never triangle-
identical output. Two independent gates over the same code give high confidence
in the mesher before anything is wired into the engine.

This change delivers the mesher + its verification ONLY. It does NOT touch the
`cc_*` ABI and does NOT wire native code into the active engine — the app keeps
tessellating through the existing OCCT (and optional Phase-2 GPU) path behind
the unchanged facade throughout.

## What Changes

- Add a new **OCCT-free C++20 tessellation library** under
  `src/native/tessellate/` (no OCCT include in any file; it MAY include
  `src/native/math` and `src/native/topology`; compiles with
  `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with no OCCT), implemented
  **clean-room** from computational-geometry references (grid / Delaunay
  triangulation, polygon-in-boundary trimming) and the `cc_*` contract. OCCT
  source (`BRepMesh`, `IMeshTools` in `TKMesh`) is consulted only as a
  **reference oracle** — to confirm an algorithm matches and compare numerics —
  never copied.
- **Face meshing (UV-grid sampling).** Mesh a native `Face` to a triangle mesh
  at a requested deflection: sample the face's surface on a UV grid via
  `native-math` (`value`/`dU`/`dV`/`normal`), respecting the face's parameter
  box, with grid density chosen so the chord-height error of each triangle
  against the true surface is at or below the deflection tolerance. Output is a
  vertex buffer (fp64 positions + normals) and an index buffer (triangles).
- **Trimming against holes.** Trim the UV grid against the face's inner wires
  (holes) using the wires' **pcurves** (from `native-topology`): a grid cell (or
  the triangles from it) whose UV footprint lies inside a hole boundary is
  removed, and cells straddling a boundary are cut so the mesh honours the face's
  trimmed region. The outer wire bounds the sampled region.
- **On-surface guarantee.** Every emitted mesh vertex SHALL lie on the true face
  surface within the deflection tolerance (its position equals the surface
  evaluated at that vertex's `(u,v)` within tolerance; boundary vertices lie on
  the wire curves within tolerance).
- **Solid meshing + edge stitching (watertightness).** Mesh a whole `Solid` by
  meshing each `Face` (via the `Explorer`) and **stitching** the per-face meshes
  along shared edges (edge → adjacent faces from `native-topology` ancestry): the
  two faces sharing an edge sample that edge's curve at the **same** parameter
  set so their boundary vertices coincide and are welded, giving a watertight
  mesh — each interior mesh edge shared by exactly two triangles for a closed
  solid.
- **Convergence.** As the deflection decreases, the mesh-derived surface area and
  enclosed volume SHALL converge monotonically (within the discretization) toward
  the analytic / B-rep values.
- **CPU source of truth; optional GPU fp32 sampling.** The fp64 CPU sampling path
  is the correctness source of truth. The Phase-2 GPU surface evaluator MAY be
  used as an fp32 sampling backend for GPU-eligible faces (untrimmed rectangular
  patches on representable surfaces), but every correctness property is asserted
  on the CPU path; the GPU path is display-only and Metal-guarded.
- **Verification harness.** For every requirement, (a) a host analytic unit test
  (`clang++ -std=c++20`, no OCCT) asserting deflection bound, on-surface
  residual, watertightness, and area/volume convergence against closed-form
  values, AND (b) a native-vs-OCCT `BRepMesh_IncrementalMesh` parity test on the
  simulator that meshes the same shape both ways at a matched deflection and
  compares bbox / total area / enclosed volume / watertightness within a
  documented tolerance — **reusing the TEST-ONLY OCCT→native bridge** from
  `tests/sim/native_topology_parity.mm` (the bridge stays in the harness).

No public C ABI change. No engine wiring. Determinism: fixed grid-sampling and
stitching order, reproducible mesh run to run for a given face/solid + deflection.

## Capabilities

### New Capabilities
- `native-tessellation`: OCCT-free, host-buildable C++20 surface mesher — meshes
  a native `Face` to a triangle mesh at a requested deflection by UV-grid
  sampling of the surface via `native-math` (respecting the face's parameter
  box), trims the grid against the face's inner wires (holes) via `native-topology`
  pcurves, guarantees every vertex lies on the true surface within the deflection
  tolerance, meshes a whole `Solid` by meshing each face and stitching shared
  edges into a watertight mesh (each mesh edge shared by exactly two triangles for
  a closed solid), and converges in mesh-derived surface area and enclosed volume
  to the analytic / B-rep values as deflection decreases. fp64 CPU is the source
  of truth; the Phase-2 GPU surface evaluator MAY serve as an fp32 sampling
  backend for eligible faces (display-only, Metal-guarded). Clean-room from
  computational-geometry references + the `cc_*` contract, with OCCT
  (`BRepMesh`/`IMeshTools`) as a reference oracle only. Verified by host analytic
  tests AND native-vs-OCCT `BRepMesh` property parity (bbox/area/volume/watertight
  within tolerance) on the simulator. Depends on `native-math` (surface eval) and
  `native-topology` (faces, wires, pcurves, explorer, ancestry).

### Modified Capabilities
<!-- none — native-tessellation is purely additive: a new OCCT-free library plus
     its tests. No cc_* signature or POD struct changes, and it is NOT wired into
     the active engine in this change. -->

## Impact

- **Contract / ABI**: none. No `cc_*` signature or POD struct layout changes; the
  library is not yet reachable through the facade (the existing OCCT / optional
  Phase-2 GPU tessellation path is untouched).
- **Engine**: none. `NativeEngine` does not consume this yet; the OCCT engine
  remains the sole active tessellation implementation.
- **Build**: new OCCT-free `src/native/tessellate/` sources (may include
  `src/native/math` + `src/native/topology`); a host CTest target
  (`clang++ -std=c++20`, no OCCT, no simulator) and a simulator native-vs-OCCT
  `BRepMesh` property-parity target (OCCT linked only in the parity test, never
  in the library; reuses the topology harness's OCCT→native bridge).
- **Determinism / precision**: fixed grid-sampling and stitching order, fp64
  geometry from `native-math`; reproducible mesh run to run. The optional GPU
  backend is fp32 and display-only; correctness holds on the CPU path.
- **Risk**: (1) deflection→grid-density mapping too coarse (misses the tolerance)
  or too fine (wasteful) — bounded by the host deflection-bound test and the
  parity area/bbox check; (2) trimming a hole boundary leaving cracks or leaking
  triangles — bounded by the holed-plane host test (exact trimmed area) and the
  watertightness check; (3) stitching not welding coincident boundary vertices —
  bounded by the watertightness invariant (every closed-solid mesh edge shared by
  exactly two triangles) on both gates; (4) fp32 GPU divergence from fp64 — bounded
  by asserting correctness only on the CPU path and treating the GPU path as
  display-only.

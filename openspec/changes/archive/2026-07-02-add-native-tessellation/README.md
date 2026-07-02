# add-native-tessellation

Phase 4 capability #3 (`native-tessellation`): an OCCT-free, host-buildable C++20
**surface mesher** under `src/native/tessellate/`, built on the two verified
foundations — `native-math` (`src/native/math/`: surface point + `dU`/`dV` +
normal for Bézier/B-spline/NURBS/elementary) and `native-topology`
(`src/native/topology/`: Face → surface + ordered outer/inner wires + pcurves,
`Explorer`, `BRep_Tool`-style accessors). It converts a native `Face` to a
triangle mesh at a requested **deflection** (chord-height) tolerance by sampling
the surface on a UV grid (via `native-math`, respecting the face's parameter
box), **trims** the grid against the face's inner wires (holes) using the wires'
pcurves so triangles inside holes are dropped, and asserts every emitted vertex
lies on the true surface within the deflection tolerance. It meshes a whole
`Solid` by meshing each face and **stitching** shared edges so the result is
watertight (each mesh edge shared by exactly two triangles for a closed solid),
and its mesh-derived surface area and enclosed volume **converge** to the
analytic / B-rep values as the deflection decreases.

Tessellation is an **approximation** — verification is tolerance-based, never
triangle-identical to OCCT. Each requirement is verified by (a) a host analytic
unit test compiled with `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` (no
OCCT: deflection bound, on-surface residual, watertightness, area/volume
convergence against closed-form values for planes / cylinders / spheres / a
holed plane) AND (b) a native-vs-OCCT `BRepMesh_IncrementalMesh` parity test on
the iOS simulator (bbox / area / volume / watertightness within tolerance at
matched deflection). The parity harness **reuses the TEST-ONLY OCCT→native
walk-in bridge pattern** from `tests/sim/native_topology_parity.mm` (the bridge
stays in the harness; nothing under `src/native` gains an OCCT dependency).

The fp64 CPU path is the **source of truth**; the Phase-2 GPU surface evaluator
MAY be used as an fp32 sampling backend for eligible faces, but correctness is
asserted on the CPU path. This change delivers the native mesher + its
verification ONLY — no `cc_*` ABI change and no engine wiring (that arrives with
later capabilities). The app keeps running unchanged on OCCT throughout.

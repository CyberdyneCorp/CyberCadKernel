# Tasks — moat-mref-reference-topology

## 1. Native reference module (OCCT-free, header-only)
- [x] 1.1 `src/native/reference/reference.h` in `cybercad::native::reference`,
      including only `src/native/{math,topology}` (zero OCCT symbols).
- [x] 1.2 `faceAxis` / `refAxisFromFace` — cylinder/cone axis; decline otherwise.
- [x] 1.3 `refPlaneFromFace` — planar-face datum plane (outward normal + on-plane
      origin); decline non-planar.
- [x] 1.4 `refAxisFromEdge` — straight-edge axis (line only); decline non-linear.
- [x] 1.5 `tangentChain` — C1 edge-set growth over shared-vertex ancestry
      (Line/Circle/Ellipse tangents); decline on a freeform edge in the walk.
- [x] 1.6 `outerRimChain` — planar-cap outer-wire edge ids (all-seeds-coplanar
      cap filter).
- [x] 1.7 `offsetFaceBoundary` — polygon in-plane miter offset; decline
      non-planar / arc-boundary / growing-convex / self-intersecting.
- [x] 1.8 World-placement helpers (bake sub-shape `Location` into frames/points).

## 2. Native engine dispatch (ADDITIVE, no cc_* signature change)
- [x] 2.1 Wire the seven ref ops in `native_engine.cpp` to resolve the id on the
      native B-rep and call `reference.h`.
- [x] 2.2 Decline → clean `Error` (facade falls through to OCCT); native void is
      NEVER forwarded; mesh body errors cleanly.

## 3. Gate A — host analytic (no OCCT)
- [x] 3.1 `tests/native/test_native_reference.cpp` + CMake registration.
- [x] 3.2 Box face planes / cylinder axis / edge axis / tangent grow+stop /
      outer rim / rectangle offset asserted to machine precision.
- [x] 3.3 Declines: non-planar plane, circular-edge axis, growing convex offset,
      non-planar offset.

## 4. Gate B — SIM native-vs-OCCT parity
- [x] 4.1 `tests/sim/native_reference_parity.mm` + `scripts/run-sim-native-reference.sh`.
- [x] 4.2 Parity vs `gp_Pln` / `gp_Cylinder::Axis` / `gp_Lin` /
      `BRepTools::OuterWire` / `BRepOffsetAPI_MakeOffset` / D1 tangent oracle.
- [x] 4.3 `run-sim-suite.sh` SKIP entry for the `.mm` harness.

## 5. Verification & docs
- [x] 5.1 `build-numsci.sh host` + `iossim` exit 0; kernel host build + full
      `ctest` green (53/53, incl. the new host test); SIM harness 8/8 green.
- [x] 5.2 Structural check: `git diff src/native` OCCT-free & additive; `cc_*` ABI
      unchanged.
- [x] 5.3 Update `openspec/MOAT-ROADMAP.md` + the M-REF row in
      `openspec/DROP-OCCT-READINESS.md`.

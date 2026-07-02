# Tasks — add-reference-geometry

Verification levels: **host** = the point-only constructors + degenerate-input
failures run in the no-OCCT host CTest (they must be correct there — this is the
acceptance bar for the analytic constructors); **ios-sim-build** = the OCCT
adapter overrides for the derived constructors compile for
`arm64-apple-ios16.0-simulator` with OCCT; **ios-sim-run** = the derived
constructors run on the booted simulator and their analytic checks pass.

## 1. Analytic math helper + point-only constructors (host-portable)
- [x] 1.1 Add a shared `ref_geometry` fp64 helper: `cross`, `normalize` (with a
  guarded zero-length → failure), and the degeneracy `eps`. (**host**)
- [x] 1.2 `cc_ref_plane_from_points(p0,p1,p2,out6)`: origin `p0`, unit normal
  `normalize((p1-p0)x(p2-p0))`; `0` on colinear/coincident input. (**host**)
- [x] 1.3 `cc_ref_plane_offset(origin,normal,dist,out6)`: unit normal,
  origin moved by `dist` along it; `0` on zero-length normal. (**host**)
- [x] 1.4 `cc_ref_axis_from_points(a,b,out6)`: origin `a`, unit direction
  `normalize(b-a)`; `0` on coincident points. (**host**)

## 2. Derived constructors via IEngine (OCCT adapter)
- [x] 2.1 Add `IEngine` virtuals `ref_plane_from_face`, `ref_axis_from_edge`,
  `ref_axis_from_face` returning `Result<std::vector<double>>` (6 values), default
  `engine_unsupported`; stub inherits the default. (**host**)
- [x] 2.2 OCCT override `ref_plane_from_face`: `BRepAdaptor_Surface`, require
  `GeomAbs_Plane`, return `gp_Pln` location + axis direction; non-planar → error. (**ios-sim-build**)
- [x] 2.3 OCCT override `ref_axis_from_edge`: `BRepAdaptor_Curve`, require
  `GeomAbs_Line`, return line location + direction; non-linear → error. (**ios-sim-build**)
- [x] 2.4 OCCT override `ref_axis_from_face`: reuse the exact `face_axis` code
  path (cyl/cone) so the datum axis equals `cc_face_axis`. (**ios-sim-build**)

## 3. Facade wiring + ABI
- [x] 3.1 Add the six entry points to `include/cybercadkernel/cc_kernel.h` and
  `src/facade/cc_kernel.cpp`; point-only trio computed facade-side (no engine),
  derived trio routed through `active_engine()`. No existing signature changes. (**host**)
- [x] 3.2 `tests/test_abi.cpp` still matches `KernelBridgeAPI.h` (additive-only). (**host**)

## 4. Analytic verification (REAL geometric properties)
- [x] 4.1 Plane-from-3-points: for a triangle with a KNOWN normal (e.g. points in
  z=5 plane → normal `(0,0,±1)`, origin at `p0`), assert `out6` normal equals the
  known unit normal and origin equals `p0` within `1e-9`. (**host**)
- [x] 4.2 Offset plane: given origin `O`, unit normal `N`, distance `d`, assert
  the returned origin equals `O + d*N` and the normal is unchanged unit `N`
  within `1e-9`. (**host**)
- [x] 4.3 Axis-from-2-points: given `a`,`b`, assert origin `a` and direction
  equals `normalize(b-a)` (unit) within `1e-9`. (**host**)
- [x] 4.4 Degenerate inputs FAIL: colinear 3 points → plane returns `0`;
  coincident 2 points → axis returns `0`; zero-length normal → offset returns
  `0`. (**host**)
- [x] 4.5 Derived (sim): plane-from-planar-face on a box top face equals the known
  face normal + a point on that face; axis-from-linear-edge on a box edge equals
  the known edge direction; axis-from-cyl-face equals `cc_face_axis` on the same
  face (within `1e-9`). (**ios-sim-run**)
- [x] 4.6 Derived in host stub return `0` (unsupported), no crash. (**host**)

## 5. Validation
- [x] 5.1 Host CTest green (point-only + degenerate + stub-unsupported). (**host**)
- [x] 5.2 On-simulator derived-constructor checks green; `run-sim-suite.sh`
  unchanged (additive only). (**ios-sim-run**)
- [x] 5.3 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 3 +
  change index for `reference-geometry`.

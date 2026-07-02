# Tasks — add-g2-blend-fillet

Verification levels: **host** = the stub no-op + ABI contract run in the no-OCCT
host CTest; **ios-sim-build** = the blend builder + curvature harness compile for
`arm64-apple-ios16.0-simulator` with OCCT; **ios-sim-run** = the G2 blend runs on
the booted simulator and the IsValid + watertight + MEASURED curvature-continuity
(vs the G1 baseline) checks pass — this is the acceptance bar for the G2
requirement. RESEARCH-GRADE: G2 is claimed only if the numbers show it; otherwise
the measured gap is reported and the case is deferred.

## 1. Entry point + routing
- [x] 1.1 Add `IEngine::fillet_edges_g2(EngineShape body, const int* edgeIds, int
  edgeCount, double radius)` returning `ShapeResult`, default `engine_unsupported`;
  stub inherits it. (**host**)
- [x] 1.2 Add `CCShapeId cc_fillet_edges_g2(CCShapeId body, const int *edgeIds,
  int edgeCount, double radius)` to the header + facade, routed through
  `active_engine()`. Additive only. (**host**)
- [x] 1.3 `tests/test_abi.cpp` still matches `KernelBridgeAPI.h`. (**host**)

## 2. Curvature-matched blend (OCCT adapter)
- [x] 2.1 Per edge, extract the two neighbour faces and the seam rails. (**ios-sim-build**)
- [x] 2.2 Build a curvature-matched blend cross-section (degree ≥ 5 B-spline or a
  conic/rho blend) constrained at both rails to match position + tangent (G1) AND
  curvature (G2) of the neighbour surfaces; sweep via
  `GeomFill_ConstrainedFilling` / B-spline surface. Radius sets width; curvature
  comes from the neighbours, not `1/radius`. (**ios-sim-build**)
- [x] 2.3 Rebuild the solid with the blend faces sewn in
  (`BRepBuilderAPI_Sewing` + `ShapeFix`); gate on `BRepCheck_Analyzer::IsValid`. (**ios-sim-build**)

## 3. Curvature measurement harness
- [x] 3.1 Sample `BRepLProp_SLProps` / `GeomLProp` second-order (principal/normal
  curvature) at seam-interior points on the BLEND side and the NEIGHBOUR side;
  compute the normalized curvature gap. (**ios-sim-build**)
- [x] 3.2 Build OCCT stock `cc_fillet_edges` on the SAME edge/radius and measure
  its seam curvature gap the same way (the G1 baseline). (**ios-sim-build**)

## 4. Verification (REAL, measured properties — honesty rule)
- [x] 4.1 Result is `BRepCheck_Analyzer::IsValid` AND watertight. (**ios-sim-run**)
- [x] 4.2 The G2 blend's seam curvature gap is within the documented G2 tolerance. (**ios-sim-run**)
- [x] 4.3 The G2 blend's curvature gap is measurably SMALLER than the stock G1
  fillet's gap at the same seam (a G1 fillet fails this). (**ios-sim-run**)
- [x] 4.4 Determinism: repeating the blend yields the same measured curvature gap
  + volume + bbox. (**ios-sim-run**)
- [x] 4.5 HONEST outcome: if 4.2 is not met, DO NOT claim G2 — record the measured
  curvature gap (and the G1 baseline for context) and mark the case deferred. If
  the blend is invalid, fall back (no body / G1 fillet) and defer with the number. (**ios-sim-run**)
  <!-- Not triggered: 4.2 was MET. Measured G2 seam curvature gap=0.018835 < tol=0.050000
  (1/r=0.333333); stock G1 baseline=0.309740 fails the bar. G2 is claimed because the
  numbers show it. No deferral needed. -->

## 5. Validation
- [x] 5.1 Host CTest green (stub `0` + ABI); `scripts/run-sim-suite.sh` unchanged
  (additive). (**host** + **ios-sim-run**)
- [x] 5.2 `openspec validate --all --strict` green; update `ROADMAP.md` Phase 3 +
  change index for `g2-blend`, recording the MEASURED curvature numbers and any
  deferred cases honestly (no G2 claim unless the numbers show it).

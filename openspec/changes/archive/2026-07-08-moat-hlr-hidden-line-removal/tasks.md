# Tasks — moat-hlr-hidden-line-removal

## 1. Native drafting core (OCCT-free, header-only) — LANDED this slice
- [x] 1.1 Create `src/native/drafting/` subtree (namespace
      `cybercad::native::drafting`), OCCT-free, host-buildable, including only
      `src/native/math`.
- [x] 1.2 `orthographic_hlr.h`: drawing-plane basis from `(viewDir, up)`,
      orthographic projection of a world point to `(u, w)`.
- [x] 1.3 `orthographic_hlr.h`: Möller–Trumbore ray/triangle intersection and the
      per-sample occlusion test (nudge toward camera by `surfaceOffset`, HIDDEN iff
      a nearer occluder triangle is hit at positive distance).
- [x] 1.4 `orthographic_hlr.h`: `projectOrthographic(edgeVertices, edges, occluder,
      view, params)` — classify each straight edge at cell MIDPOINTS and SPLIT at
      visibility transitions (bisection to `transitionTol`), returning disjoint
      visible/hidden 2D segment sets.
- [x] 1.5 `native_drafting.h` umbrella header.

## 2. GATE (a) host-analytic verification (no OCCT) — LANDED this slice
- [x] 2.1 `tests/native/test_native_drafting.cpp`: box from an isometric corner →
      exactly 9 visible + 3 hidden segments; the 3 hidden segments share one
      endpoint (the occluded far corner).
- [x] 2.2 Baseline: same box with an empty occluder → 12 visible, 0 hidden.
- [x] 2.3 Edge splitting: a straight edge half-covered by a nearer face → 1 visible
      + 1 hidden, split at the coverage boundary; projected length conserved across
      the visible/hidden split.
- [x] 2.4 Register `test_native_drafting` in `CMakeLists.txt` always-on native suite
      (no NumSci link), with its source mapping.
- [x] 2.5 Confirm the module compiles OCCT-free for host AND arm64 iOS-simulator
      (`-fsyntax-only`), and that `grep` finds 0 OCCT tokens under
      `src/native/drafting/`.

## 3. Topology-driven adapter (LANDED — reuses the landed core)
- [x] 3.1 Walk a `topo::Shape`'s edges via `topology::mapShapes`/`Explorer` + the
      `tessellate::EdgeCache` and sample each STRAIGHT edge into a world polyline
      (`NativeEngine::hlr_project`, src/engine/native/native_engine.cpp). Coincident
      per-face edge duplicates (edge-sharing is deferred in the native B-rep) are
      collapsed by a quantized order-independent endpoint key so each edge is drawn
      ONCE — matching the OCCT edge set exactly.
- [x] 3.2 Build the occluder from `tessellate::SolidMesher` at an explicit, caller-
      chosen deflection (`CCHlrOptions.deflection`; `<= 0` uses the mesher default —
      never a hidden default at the call site).
- [ ] 3.3 Emit closed-form **silhouette** curves for cylinder/cone/sphere (outline
      generators + end-ellipse arcs) and classify them like any other edge. STILL
      DEFERRED — the documented next GS1 slice (the polyhedral slice is complete).
- [x] 3.4 **Honest decline**: `hlr_project` returns an error for any non-planar
      (curved/freeform) face, a degenerate view direction, and an up hint parallel to
      the view direction — never a wrong classification. Verified by the sim harness's
      `cylinder decline` case.

## 4. Additive `cc_*` ABI (LANDED)
- [x] 4.1 Additive PODs (`CCDrawingSegment`, `CCDrawing`, `CCHlrOptions`) and
      `cc_hlr_project(body, viewDir, up, opts) -> CCDrawing` + `cc_drawing_free` in
      `include/cybercadkernel/cc_kernel.h`. No existing signature changes; the mirrored
      `KernelBridgeAPI.h` structs are untouched (test_abi unaffected). `IEngine`
      gains `DrawingSegmentData`/`DrawingData`/`HlrOptionsData` + a default-unsupported
      `hlr_project`.
- [x] 4.2 Facade body in `src/facade/cc_kernel.cpp` using the existing `guard`/`resolve`
      helpers; maps the engine visible/hidden segment sets to the flat malloc'd POD
      arrays freed by `cc_drawing_free`. A decline returns an empty `CCDrawing` with
      `cc_last_error` set.

## 5. GATE (b) SIM native-vs-OCCT parity (LANDED)
- [x] 5.1 `tests/sim/native_hlr_parity.mm` (the ONLY OCCT-linked piece; OCCT HLR lives
      in `src/engine/occt/occt_drafting.cpp`): drives `HLRBRep_Algo` +
      `HLRBRep_HLRToShape` (orthographic projector in the SAME drawing-plane basis as
      native) and compares the native visible/hidden sets on segment COUNT, total
      projected LENGTH, and endpoint PARTITION within tolerance — for POLYHEDRAL solids
      (box iso + oblique, triangle prism, non-convex L-prism). Curved-silhouette solids
      (cylinder) are asserted DECLINED, not compared. Result: 13/13 PASS, native == OCCT
      to machine epsilon (length rel ≤ 2e-16, partition within 1e-5).
- [x] 5.2 `scripts/run-sim-native-hlr.sh` builds + runs the parity harness in a booted
      simulator (adds `TKHLR` to the toolkit link set), mirroring the existing
      native-parity scripts.

## 6. Docs
- [x] 6.1 Note the GS1 polyhedral slice (two-gate complete) + the curved-silhouette
      blocker (task 3.3) as the next GS1 slice in `openspec/MOAT-ROADMAP.md` / STATUS.

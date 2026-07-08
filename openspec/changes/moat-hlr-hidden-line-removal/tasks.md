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

## 3. Topology-driven adapter (follow-up — reuses the landed core)
- [ ] 3.1 Walk a `topo::Shape`'s edges via `topology::Explorer` and sample each edge
      (straight + analytic Circle/Ellipse) into world polylines via the accessors.
- [ ] 3.2 Build the occluder from `tessellate::SolidMesher` at an explicit, caller-
      chosen deflection (never a hidden default).
- [ ] 3.3 Emit closed-form **silhouette** curves for cylinder/cone/sphere (outline
      generators + end-ellipse arcs) and classify them like any other edge.
- [ ] 3.4 **Honest decline**: return an error for freeform faces, curved silhouettes
      not yet traced, view directions along an edge, and any sample whose occlusion
      is within tolerance of ambiguous — never emit a wrong classification.

## 4. Additive `cc_*` ABI (follow-up — contract specified here)
- [ ] 4.1 Add additive PODs (`CCDrawing`, `CCDrawingSegment`, `CCHlrOptions`) and
      `cc_hlr_project(body, viewDir, up, opts) -> CCDrawing` + `cc_drawing_free` in
      `include/cybercadkernel/`. No existing signature changes.
- [ ] 4.2 Facade body in `src/facade/` using the existing `guard`/`resolve` helpers;
      map the native visible/hidden segment sets to the flat POD arrays.

## 5. GATE (b) SIM native-vs-OCCT parity (follow-up)
- [ ] 5.1 `tests/sim/native_drafting_parity.mm` (the ONLY OCCT-linked piece): drive
      `HLRBRep_Algo` + `HLRBRep_HLRToShape` on box / cylinder / cone and compare the
      native visible/hidden sets on segment count, total projected length, and
      endpoint positions within tolerance.
- [ ] 5.2 `scripts/run-sim-native-drafting.sh` to build + run the parity harness in a
      booted simulator, mirroring the existing native-parity scripts.

## 6. Docs
- [ ] 6.1 Note the GS1 first slice + the sharpened curved-silhouette blocker in
      `openspec/MOAT-ROADMAP.md` / STATUS when the adapter lands.

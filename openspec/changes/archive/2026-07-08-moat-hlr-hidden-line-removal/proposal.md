# Proposal — moat-hlr-hidden-line-removal

## Why

The CyberCad app's 2D-drawings feature — `DrawingProjector` / `ProjectEdges` /
`ProjectBody` / `ManufacturingDrawing` — depends on OCCT's `HLRBRep_Algo` to
project a B-rep onto a drawing plane and classify each edge segment **visible**
vs **hidden**. This is the single hard blocker to an OCCT-free drawings feature
(MOAT roadmap **M-GS · GS1**, "the substantial one"). No native path exists
today: the kernel can tessellate and explore topology, but it cannot produce a
hidden-line-removed orthographic drawing.

This change adds a **native, OCCT-free drafting service** that performs
**orthographic (parallel) hidden-line removal** for the **analytic + polyhedral
core** (box / prism / cylinder / cone / sphere and simple combinations), exposed
behind a new additive `cc_*` accessor. It is a research-grade first slice: it
lands the provable core and **declines** — rather than guesses — the cases it
cannot yet handle robustly (curved-surface silhouette tracing, freeform faces).

The discipline is non-negotiable because a drawing is a legal/manufacturing
artifact: **a hidden edge shown solid, or a visible edge shown dashed, is a wrong
drawing.** The service therefore self-verifies against two oracles and returns an
honest decline for any configuration it cannot classify correctly.

## What Changes

1. **A new native drafting subtree** (`src/native/drafting/`, namespace
   `cybercad::native::drafting`, **OCCT-FREE**, header-only, host-buildable),
   consuming the landed native topology (`src/native/topology`), tessellator
   (`src/native/tessellate`) and math (`src/native/math`). This slice lands
   `orthographic_hlr.h` — the **polyhedral occluder** HLR core — plus the
   `native_drafting.h` umbrella.
2. **The orthographic HLR core** projects each straight edge onto the drawing
   plane and classifies every projected sample **visible/hidden** by ray-casting
   along the parallel view direction against a triangle occluder (the M0 boundary
   tessellation from `solid_mesher.h`), **splitting** each edge at every
   visibility transition so the visible and hidden segment sets are disjoint and
   exact. A sample is HIDDEN iff a nearer surface lies between it and the parallel
   viewpoint.
3. **The scoped decline surface.** Curved-face **silhouette** curves
   (cylinder/cone/sphere outline lines/ellipses) and **freeform** faces are NOT
   traced in this slice; the topology-driven adapter that walks `Explorer` edges,
   builds the `SolidMesher` occluder, and would emit analytic silhouettes is
   explicitly scoped as follow-up. The core NEVER emits a wrong classification for
   an input it accepts; unsupported inputs are declined honestly (see the design
   doc's blocker section).
4. **Two-gate verification.** GATE (a) HOST ANALYTIC (no OCCT): a box from an
   isometric corner yields exactly **9 visible + 3 hidden** projected segments,
   the 3 hidden edges meeting at the occluded far corner; a half-occluded edge
   splits into exactly one visible + one hidden segment at the coverage boundary;
   projected length is conserved across the visible/hidden split. GATE (b) SIM
   native-vs-OCCT: match `HLRBRep_Algo` / `HLRBRep_HLRToShape` visible/hidden
   compounds on segment count, total projected length, and endpoint positions
   within tolerance (a separate `.mm` harness, scoped in tasks).
5. **An additive `cc_*` accessor** (`cc_hlr_project` + `cc_drawing_free` and the
   `CCDrawing` / `CCDrawingSegment` / `CCHlrOptions` PODs) returns the
   visible/hidden 2D edge-segment sets; existing `cc_*` signatures are unchanged
   and the drawing PODs are additive-only. The accessor is wired end to end —
   facade → `IEngine::hlr_project` → `NativeEngine::hlr_project` (the OCCT-free
   polyhedral core over the M0 occluder + the deduplicated topology edges) with
   `OcctEngine::hlr_project` (the `HLRBRep_Algo` oracle) as the fallthrough — and
   Gate (b) native-vs-OCCT parity now passes for polyhedral solids.

## Capabilities

### Added Capabilities

- `native-drafting`: adds native **orthographic hidden-line removal** for the
  analytic/polyhedral core — orthographic projection of a solid's edges onto a
  drawing plane, per-sample visible/hidden classification against the solid's own
  occluding faces, and edge splitting at visibility transitions — together with
  the **OCCT-free + additive-ABI** discipline, the **two-gate** (host-analytic +
  OCCT-parity) verification contract, and the **honest-decline** contract for the
  curved-silhouette / freeform cases the native path does not yet trace.

## Impact

- New OCCT-free, host-buildable, header-only files under `src/native/drafting/`
  (`native_drafting.h`, `orthographic_hlr.h`). No `.cpp`, no library-link change;
  they include only `src/native/math`.
- `tests/native/test_native_drafting.cpp` — always-on host CTest (GATE a): the
  box isometric-corner 9-visible/3-hidden proof, the no-occluder all-visible
  baseline, and the edge-splitting transition proof. Registered in the always-on
  native suite (no NumSci link).
- `CMakeLists.txt` — `test_native_drafting` added to the always-on `CYBERCAD_TESTS`
  list with its source mapping, exactly like `test_native_quality`.
- **Landed here (ABI + Gate b wiring):** the topology-driven entry
  `NativeEngine::hlr_project` (walks the edges, dedups coincident per-face edge
  nodes, builds the `SolidMesher` occluder, runs the core); the `cc_hlr_project` /
  `cc_drawing_free` facade + PODs in `include/cybercadkernel/cc_kernel.h`;
  `IEngine::hlr_project`; the `OcctEngine::hlr_project` oracle in
  `src/engine/occt/occt_drafting.cpp` (the only OCCT-linked piece, `TKHLR`); and
  the Gate (b) `tests/sim/native_hlr_parity.mm` + `scripts/run-sim-native-hlr.sh`
  harness (13/13 PASS, native == OCCT for box / prism / non-convex L-prism).
- **Out of scope (documented next GS1 slice):** emitting analytic **curved
  silhouette** curves (cylinder/cone/sphere outline generators + end-ellipse arcs)
  — these solids are honestly DECLINED today; and native OCCT-style edge sharing
  (the drawing dedups coincident edges instead).
- Behaviour otherwise unchanged: `src/native/**` stays OCCT-free (0 OCCT includes),
  the `cc_*` ABI is untouched, and no existing engine path is affected.

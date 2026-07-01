# CyberCadKernel Roadmap — Wrap → Accelerate → Rewrite

The trajectory from "OCCT behind a facade" to "fully native C++20 kernel, no
OCCT". Each phase ships behind the **unchanged `cc_*` C ABI**, so the app keeps
working throughout. Native implementations land capability-by-capability and are
validated against the OCCT-backed path before it is retired.

Legend: ☐ not started · ◐ in progress · ✅ done.

## Phase 0 — Foundation (facade + wrapping OCCT)
Stand up the library and move CyberCad's OCCT bridge into it, unchanged in
behaviour. Establishes the seams everything else plugs into.
- ☐ **Stable C ABI facade** (`cc_*`, shape registry, error/guard model) —
  capability `kernel-facade`.
- ☐ **Engine adapter** abstraction with an **OCCT adapter** as the first
  implementation — capability `engine-adapter`.
- ☐ **Operation scheduler**: coroutine-based, cancellable, progress-reporting
  execution off the UI thread — capability `operation-scheduler`.
- ☐ **Compute-backend interface** (no-op/CPU backend first) — capability
  `compute-backend`.
- First change: `add-kernel-foundation`.

## Phase 1 — Multi-core acceleration (still on OCCT)
Highest leverage, lowest risk — attacks the known bottlenecks with no new
geometry code.
- ☐ Enable OCCT parallel booleans (`SetRunParallel`) + tuned `SetFuzzyValue`
  behind `cc_boolean` — targets the fine-thread fuse/cut that pegs OCCT for
  minutes.
- ☐ Enable parallel meshing (`BRepMesh` `isInParallel`) behind tessellation.
- ☐ Make long ops cancellable via the scheduler (fixes non-cancellable `Build`).

## Phase 2 — GPU acceleration (Metal first)
fp32-tolerant, data-parallel work through the compute backend.
- ☐ **Metal tessellation**: GPU NURBS/Bézier surface evaluation feeding the
  triangulator; topology stays on CPU.
- ☐ **Metal BVH** build/traversal (LBVH/Morton) for culling + selection.
- ☐ **GPU picking** (frustum vs BVH) for large models.
- ☐ Mesh post-processing (normals, LOD, deformation) on GPU.
- ☐ Unified-memory buffer path (Apple Silicon) to avoid copies.

## Phase 3 — Missing features OCCT lacks (native algorithms)
New geometry the app already needs; these are native from the start (OCCT can't
do them).
- ☐ **Curvature-continuous (G2) fillet / blend surfaces** (OCCT is G1-only).
- ☐ **Rolling-ball / full-round fillet.**
- ☐ **Robust thread↔shaft boolean** (feature-based, doesn't hang on fine helices).
- ☐ **Robust wrap-emboss** (cap-and-side / healed sew vs fragile ThruSections).
- ☐ **Reference geometry** primitives (datum planes/axes) if kernel support needed.

## Phase 4 — Native rewrite (retire OCCT, capability by capability)
Replace the OCCT adapter with native C++20 implementations, one capability at a
time, each validated against the OCCT path behind the same facade call, then
OCCT unlinked for that capability. Rough dependency order:
- ☐ Math & geometry primitives (points/vectors/transforms, curves/surfaces eval).
- ☐ B-rep topology data model + exploration.
- ☐ Tessellation / meshing (native, GPU-backed).
- ☐ Primitive & swept-solid construction (extrude/revolve/loft/sweep).
- ☐ Booleans (native robust kernel — the hardest; longest-lived OCCT dependency).
- ☐ Fillets/chamfers/offsets/shell.
- ☐ Data exchange (STEP/IGES) — may remain a thin external dependency longest.
- ☐ **Drop OCCT**: kernel is fully native C++20, MIT, no LGPL obligation.

## Guiding rules
- The `cc_*` ABI never breaks; the app is insulated from every phase.
- Native and OCCT-backed implementations coexist behind the adapter so each
  migration is measured (correctness + performance) before OCCT is retired for
  that capability.
- Booleans are expected to be the last and hardest OCCT dependency to replace —
  plan accordingly.

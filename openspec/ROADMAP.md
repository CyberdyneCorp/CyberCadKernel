# CyberCadKernel Roadmap — Wrap → Accelerate → Rewrite

The trajectory from "OCCT behind a facade" to "fully native C++20 kernel, no
OCCT". Each phase ships behind the **unchanged `cc_*` C ABI**, so the app keeps
working throughout. Native implementations land capability-by-capability and are
validated against the OCCT-backed path before it is retired.

Every phase item names the **OpenSpec change** that delivers it, the
**capability** it adds/modifies, and — where it exists today — the **contract**
it must satisfy (`cybercad/openspec/specs/occt-usage/spec.md`, the exact `cc_*`
surface the app relies on) and the tracked **GitHub issue**. Changes are
proposed (`/opsx:propose`) when a phase is about to start, not front-loaded; only
Phase 0 and Phase 1 are scaffolded so far.

Legend: ☐ not started · ◐ in progress · ✅ done.

> **Acceptance bar (decided 2026-07-01):** the in-repo **simulator** integrated
> suite (`scripts/run-sim-suite.sh` — all 57 `cc_*` + determinism + benchmark,
> 221/221) is the acceptance bar for Phase 0/1. The **app link-swap** and
> **on-device** benchmark are **optional, deferred by decision** (they would
> touch the CyberCad app / need physical hardware) — not correctness gates.

## Phase 0 — Foundation (facade + wrapping OCCT)
Stand up the library and move CyberCad's OCCT bridge into it, unchanged in
behaviour. Establishes the seams everything else plugs into.
Change: **`add-kernel-foundation`** *(in progress — implemented; host build+tests
pass, OCCT adapter runs on the iOS simulator with all 57 `cc_*` runtime-verified
(221-check suite); only the app link-swap remains, deferred to an app follow-up)*.
- ✅ **Stable C ABI facade** (`cc_*`, shape registry, error/guard model) —
  capability `kernel-facade`. Contract: `occt-usage` §Foundation-classes
  exception safety, §Modeling-data types. *(implemented; host CTest green,
  `test_abi` matches the app header.)*
- ◐ **Engine adapter** abstraction with an **OCCT adapter** as the first
  implementation — capability `engine-adapter`. Contract: the full `occt-usage`
  surface (construction, boolean, fillet, tessellate, query, transform,
  exchange). *(implemented; device+sim xcframework built; **all 57 `cc_*` entry
  points now run correctly on the iOS simulator** — full suite 221/221 against
  analytic/round-trip references. Only the app **link-swap** + a direct byte-diff
  vs the app's inline `KernelBridge.mm` remain (need the CyberCad app project).)*
- ✅ **Operation scheduler**: coroutine-based, cancellable, progress-reporting
  execution off the UI thread — capability `operation-scheduler`. Addresses the
  non-cancellable `Build` (`occt-usage` §Performance & acceleration targets).
  *(coroutine `Task<T>` + `std::thread` pool + in-house `StopToken` — Apple Clang
  lacks `<jthread>`/`<stop_token>`; host-tested.)*
- ✅ **Compute-backend interface** (no-op/CPU backend first) — capability
  `compute-backend`. *(CPU backend + precision guard; host-tested.)*

## Phase 1 — Multi-core acceleration (still on OCCT)
Highest leverage, lowest risk — attacks the known bottlenecks with no new
geometry code, by enabling OCCT's *existing* parallel paths behind the facade.
Change: **`accelerate-multicore-occt`** *(in progress — parallel paths implemented,
run on the iOS simulator, and the determinism audit + serial-vs-parallel benchmark
are complete on the sim (221-check suite, `cc_set_parallel` A/B); only on-device
core scaling remains)* — capability `parallel-acceleration`.
- ◐ Enable OCCT parallel booleans (`BOPAlgo_Options::SetRunParallel`) + tuned
  `SetFuzzyValue` behind `cc_boolean` — targets the fine-thread fuse/cut that
  pegs OCCT for minutes. Contract: `occt-usage` §Boolean operations, §Performance
  (GitHub #286). *(implemented; runs on the iOS simulator; **serial-vs-parallel
  A/B verified bit-identical** for box-box fuse (1875) + cut (875) via
  `cc_set_parallel`. Fine-thread + on-device benchmark pending.)*
- ◐ Enable parallel meshing (`BRepMesh_IncrementalMesh` `isInParallel`) behind
  `cc_tessellate` / `cc_face_meshes`. Contract: `occt-usage` §Meshing.
  *(implemented; runs on the simulator; **parallel mesh is bit-identical to the
  serial mesh** on all audited bodies (fuse/cut/revolve/fillet), stable ×8.)*
- ◐ Make long ops cancellable via the scheduler and gate fine-thread booleans
  until accelerated (fixes non-cancellable `Build`; `occt-usage` §Performance
  scenario). *(scheduler routing + fine-thread gate implemented; gate host-tested
  via `test_parallel_policy`.)*
- ◐ Determinism audit: parallel results must be bit-reproducible before parallel
  becomes the default. *(**complete on the simulator** — `cc_set_parallel(0/1)`
  A/B over box-box fuse, box-box cut, revolve tube, and a multi-face fillet solid
  all report `serial == parallel: YES` (bit-identical mesh hash + exact volume +
  tri count), stable across 8 parallel runs; parallel-by-default is justified for
  exactly these paths.)*

## Phase 2 — GPU acceleration (Metal first)
fp32-tolerant, data-parallel work through the compute backend. CPU stays the
source of truth; the GPU never touches the exact fp64 topology core.
Changes: **`add-metal-compute-backend`** (capability `metal-backend`),
**`add-gpu-tessellation`** (capability `gpu-tessellation`),
**`add-gpu-spatial-acceleration`** (capability `spatial-acceleration`).
- ☐ **Metal backend** implementing the Phase-0 `compute-backend` interface;
  unified-memory buffer path on Apple Silicon to avoid copies. → `metal-backend`.
- ☐ **Metal tessellation**: GPU NURBS/Bézier surface evaluation feeding the
  triangulator; topology stays on CPU. → `gpu-tessellation`. Contract:
  `occt-usage` §Meshing (biggest self-contained win).
- ☐ **Metal BVH** build/traversal (LBVH/Morton) for culling + selection. →
  `spatial-acceleration`.
- ☐ **GPU picking** (frustum vs BVH) for large models. → `spatial-acceleration`.
- ☐ Mesh post-processing (normals, LOD, deformation) on GPU. → `gpu-tessellation`.

## Phase 3 — Missing features OCCT lacks (native algorithms)
New geometry the app already needs; these are native from the start (OCCT can't
do them). Each replaces/augments its `cc_*` behind the facade.
- ☐ **Curvature-continuous (G2) fillet / blend surfaces** (OCCT is G1/circular
  only). Change **`add-g2-blend-fillet`** — capability `g2-blend`. Contract:
  `occt-usage` §Fillets & chamfers limitation (GitHub #284); `cc_fillet_edges`.
- ☐ **Rolling-ball / full-round fillet.** Change **`add-full-round-fillet`** —
  capability `full-round-fillet` (GitHub #285).
- ☐ **Robust thread↔shaft boolean** (feature-based, doesn't hang on fine
  helices). Change **`add-robust-thread-boolean`** — capability `thread-boolean`.
  Contract: `occt-usage` §Performance (GitHub #286); `cc_boolean`,
  `cc_helical_thread`.
- ☐ **Robust wrap-emboss** (cap-and-side / healed sew vs fragile ThruSections).
  Change **`add-robust-wrap-emboss`** — capability `wrap-emboss`. Contract:
  `occt-usage` §Offsets/sweeps/lofts (GitHub #290); `cc_wrap_emboss`.
- ☐ **Reference geometry** primitives (datum planes/axes) if kernel support
  needed. Change **`add-reference-geometry`** — capability `reference-geometry`.
  Cross-refs `cybercad` `add-datum-plane-sketching`.

## Phase 4 — Native rewrite (retire OCCT, capability by capability)
Replace the OCCT adapter with native C++20 implementations, one capability at a
time, each validated against the OCCT path behind the same facade call, then
OCCT unlinked for that capability. Dependency order below is the change order.
- ☐ Math & geometry primitives (points/vectors/transforms, curves/surfaces
  eval). Change **`add-native-math-geometry`** — capability `native-math`.
  Contract: `occt-usage` §Modeling-data types (`gp_*`, `Geom_*`).
- ☐ B-rep topology data model + exploration. Change
  **`add-native-brep-topology`** — capability `native-topology`. Contract:
  `occt-usage` §Modeling-data (`TopoDS`, `TopExp`, sub-shape ids).
- ☐ Tessellation / meshing (native, GPU-backed via Phase 2). Change
  **`add-native-tessellation`** — capability `native-tessellation`.
- ☐ Primitive & swept-solid construction (extrude/revolve/loft/sweep). Change
  **`add-native-swept-solids`** — capability `native-construction`. Contract:
  `occt-usage` §Primitive & swept-solid, §B-rep construction, §Offsets/sweeps.
- ☐ Booleans (native robust kernel — the hardest; longest-lived OCCT
  dependency). Change **`add-native-booleans`** — capability `native-booleans`.
  Contract: `occt-usage` §Boolean operations.
- ☐ Fillets/chamfers/offsets/shell. Change **`add-native-fillets-offsets`** —
  capability `native-blends`. Contract: `occt-usage` §Fillets & chamfers,
  §Offsets/sweeps/lofts/shells.
- ☐ Data exchange (STEP/IGES) — may remain a thin external dependency longest.
  Change **`add-native-data-exchange`** — capability `native-exchange`.
  Contract: `occt-usage` §Data exchange.
- ☐ **Drop OCCT**: kernel is fully native C++20, MIT, no LGPL obligation. Change
  **`drop-occt`** — retires the OCCT adapter (no new capability).

## Change index

Phase → change → capability → status. Update the status column and each phase's
checkboxes as changes land; flip to ✅ when a change is validated and archived.

| Phase | Change | Capability(ies) | Status |
|---|---|---|---|
| 0 | `add-kernel-foundation` | kernel-facade, engine-adapter, operation-scheduler, compute-backend | ✅ complete at acceptance bar (host tests + all 57 `cc_*` on iOS-sim, 221/221); app link-swap = optional deferred |
| 1 | `accelerate-multicore-occt` | parallel-acceleration | ✅ complete at acceptance bar (parallel paths on iOS-sim; determinism audit + serial-vs-parallel benchmark done); on-device scaling = optional deferred |
| 2 | `add-metal-compute-backend` | metal-backend | ☐ planned |
| 2 | `add-gpu-tessellation` | gpu-tessellation | ☐ planned |
| 2 | `add-gpu-spatial-acceleration` | spatial-acceleration | ☐ planned |
| 3 | `add-g2-blend-fillet` | g2-blend | ☐ planned (#284) |
| 3 | `add-full-round-fillet` | full-round-fillet | ☐ planned (#285) |
| 3 | `add-robust-thread-boolean` | thread-boolean | ☐ planned (#286) |
| 3 | `add-robust-wrap-emboss` | wrap-emboss | ☐ planned (#290) |
| 3 | `add-reference-geometry` | reference-geometry | ☐ planned |
| 4 | `add-native-math-geometry` | native-math | ☐ planned |
| 4 | `add-native-brep-topology` | native-topology | ☐ planned |
| 4 | `add-native-tessellation` | native-tessellation | ☐ planned |
| 4 | `add-native-swept-solids` | native-construction | ☐ planned |
| 4 | `add-native-booleans` | native-booleans | ☐ planned |
| 4 | `add-native-fillets-offsets` | native-blends | ☐ planned |
| 4 | `add-native-data-exchange` | native-exchange | ☐ planned |
| 4 | `drop-occt` | — (retires OCCT adapter) | ☐ planned |

## Guiding rules
- The `cc_*` ABI never breaks; the app is insulated from every phase.
- Native and OCCT-backed implementations coexist behind the adapter so each
  migration is measured (correctness + performance) before OCCT is retired for
  that capability.
- Booleans are expected to be the last and hardest OCCT dependency to replace —
  plan accordingly.
- Propose each change (`/opsx:propose`) only when its phase is about to start,
  keeping speculative spec work out of the tree; archive its delta specs into
  `openspec/specs/` when the change is validated.

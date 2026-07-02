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

> **Acceptance bar:** the in-repo integrated GPU-vs-CPU parity suite
> (`scripts/run-sim-gpu-suite.sh` — 18/18) + the backend self-test
> (`scripts/run-sim-gpu-selftest.sh`) running runtime-compiled MSL on the real
> "Apple iOS simulator GPU", each result asserted against an independent CPU
> reference within an fp32 tolerance. **On-device** (physical Apple silicon) runs
> are optional/deferred. See `docs/STATUS-phase-2.md`.

- ✅ **Metal backend** implementing the Phase-0 `compute-backend` interface;
  unified-memory (`StorageModeShared`) buffer path to avoid copies. →
  `metal-backend`. *(implemented; **self-test PASS on the iOS-sim GPU** — device
  init, buffer round-trip, runtime MSL compile + pipeline cache, dispatch, fp32
  saxpy parity; fp32-only, refuses fp64; host CTest stays green with
  `CYBERCAD_HAS_METAL=OFF`.)*
- ✅ **Metal tessellation**: GPU NURBS/Bézier surface evaluation + GPU per-vertex
  normals, topology stays on CPU. → `gpu-tessellation`. Contract: `occt-usage`
  §Meshing. *(GPU surface-grid eval + per-vertex normals **verified on the
  iOS-sim GPU** vs CPU reference (fp32), and the GPU eval path is now **wired into
  `cc_tessellate` / `cc_face_meshes`** behind the `cc_set_gpu_tessellation` toggle
  (default OFF): per-face eligibility routing sends single-outer-wire /
  UV-rectangular / low-degree faces to a GPU grid triangulator and falls back to
  OCCT `BRepMesh` for holed/trimmed/curved faces. **Verified on the sim** —
  integ suite (`run-sim-integ-suite.sh`) **26/26**: box routes gpu=6/0, mixed slab
  routes gpu=4/3, GPU-fed mesh matches the OCCT-only mesh on bbox + area + volume +
  watertightness (fp32); GPU-OFF path is byte-identical and `run-sim-suite.sh`
  stays 221/221. Remaining: an explicit **repeat-run determinism** assertion;
  GPU tessellation of **holed/trimmed/curved faces** is deferred **by design**
  (they fall back to OCCT).)*
- ✅ **Metal BVH** build/traversal (LBVH/Morton) for culling + selection. →
  `spatial-acceleration`. *(GPU LBVH build + stackless nearest-hit ray traversal
  **verified on the iOS-sim GPU** vs CPU brute force, same id + t (fp32).)*
- ✅ **GPU picking** (rays + frustum vs BVH) for large models. →
  `spatial-acceleration`. *(GPU batched **ray-pick + frustum-pick both verified on
  the iOS-sim GPU** vs CPU reference — frustum set equals the CPU reference set
  (subset {0,1} / empty / all-enclosing), sorted ascending, and **byte-identical
  across 8 runs** (ray-pick + frustum-pick); GPU pick suite now **26/26**. The
  only remaining spatial item is the **OPTIONAL additive `cc_*` pick/cull facade
  entry** (app-facing, out of scope for this change — no OCCT-side pick path
  exists in the facade today).)*
- ✅ Mesh post-processing (GPU per-vertex normals) → `gpu-tessellation`.
  *(**verified on the iOS-sim GPU** vs CPU reference per component, dot ≈ 1; LOD /
  deformation not in scope for this change.)*

## Phase 3 — Missing features OCCT lacks (native algorithms)
New geometry the app already needs; these are native from the start (OCCT can't
do them). Each replaces/augments its `cc_*` behind the facade.

> **Acceptance bar:** the in-repo Phase-3 property suite
> (`scripts/run-sim-phase3-suite.sh`) on the booted iOS simulator with OCCT
> linked (`cc_brep_available()==1`), each result asserted against a REAL
> geometric property (`IsValid`, watertight, volume sign, `1e-9` normals,
> G1-tangency, or a MEASURED curvature gap). Result: **65 passed, 0 failed, 1
> deferred**. On-device runs + app link-swap are optional/deferred. See
> `docs/STATUS-phase-3.md`.

- ✅ **Curvature-continuous (G2) fillet / blend surfaces** (OCCT is G1/circular
  only). Change **`add-g2-blend-fillet`** — capability `g2-blend`. Contract:
  `occt-usage` §Fillets & chamfers limitation (GitHub #284); `cc_fillet_edges`.
  *(implemented; **verified on the iOS sim** — valid + watertight solid, MEASURED
  seam curvature gap **0.018835 within G2 tol 0.05** (1/r=0.333333) while the stock
  G1 baseline **0.309740 fails** the bar; G2 measurably smaller than G1; bit-exact
  determinism (dV=dBBox=dGap=0). G2 is claimed because the numbers show it.)*
- ◐ **Rolling-ball / full-round fillet.** Change **`add-full-round-fillet`** —
  capability `full-round-fillet` (GitHub #285). *(implemented; **verified on the
  iOS sim** for tangent/parallel-wall strips — 10 checks: middle face consumed,
  cylinder blend, axis equidistant, **G1-tangent both seams dot=1.000000**
  (tol cos1°=0.999848), deterministic, single-arg auto-detect matches. **DEFERRED
  (measured):** non-parallel walls (off-parallel 22.62°, n_L·n_R=-0.9231) fall
  back to a VALID standard edge fillet, middle face NOT consumed (vol=1597.844).)*
- ✅ **Robust thread↔shaft boolean** (feature-based, doesn't hang on fine
  helices). Change **`add-robust-thread-boolean`** — capability `thread-boolean`.
  Contract: `occt-usage` §Performance (GitHub #286); `cc_boolean`,
  `cc_helical_thread`. *(implemented; **verified on the iOS sim** — segmented
  apply of a fine multi-turn thread: FUSE **4.3778 s < 8 s budget**, CUT
  **4.4817 s < 8 s**, both `BRepCheck`-valid + watertight (0 free / 0 non-manifold),
  correct volume sign (fuse +29.80; cut removes ≈V_thread), naive `cc_boolean`
  NOT run. Determinism within tolerance, not bit-exact (\|ΔV\|=0.2004, rel 2e-4 —
  parallel BOPAlgo).)*
- ✅ **Robust wrap-emboss** (cap-and-side / healed sew vs fragile ThruSections).
  Change **`add-robust-wrap-emboss`** — capability `wrap-emboss`. Contract:
  `occt-usage` §Offsets/sweeps/lofts (GitHub #290); `cc_wrap_emboss`.
  *(implemented; **verified on the iOS sim** — emboss + deboss both valid +
  watertight (naked=0), correct volume sign (V_base=12566.37; emboss Δ=+105.60,
  deboss Δ=-86.40), reproducible, wide high-curvature profile valid + watertight
  (Δ=369.60); falls back sewn→coarse ThruSections→`0`.)*
- ✅ **Reference geometry** primitives (datum planes/axes) if kernel support
  needed. Change **`add-reference-geometry`** — capability `reference-geometry`.
  Cross-refs `cybercad` `add-datum-plane-sketching`. *(implemented; **verified on
  the iOS sim** — 21/21: plane/axis from points/offset/face/edge; 6/6 box faces
  resolve planes with unit normals within `1e-9`; 12/12 box edges resolve axes;
  cylinder axis unit ±Z; planar-cap / non-planar / degenerate guards hold.)*

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

## Tooling & bindings

Cross-cutting developer tooling that consumes the `cc_*` ABI without changing it.
These are **desktop, development-only** artifacts (not shipped to iOS); they exist
to drive, test, and *see* the kernel while the phases above evolve the engine
behind the facade.

- ◐ **Python binding** — a desktop (macOS arm64) Python package
  (`cybercadkernel`) over the `cc_*` ABI: a low-level 1:1 `ctypes` binding of
  every `cc_*` fn + POD struct, a pythonic `Kernel`/`Shape` object model
  (context-managed handle lifetime, NumPy meshes, exceptions from
  `cc_last_error`), and `trimesh` visualization (STL/PLY/GLB export + offscreen
  PNG with a headless matplotlib fallback). Backed by a **Homebrew-OCCT**
  desktop build of the kernel (`scripts/build-macos-dylib.sh` → `build-mac/`
  `libcybercadkernel.dylib`, CMake `CYBERCAD_MACOS_OCCT=ON`, Metal excluded) so
  Python drives real geometry (`cc_brep_available() == 1`). Change
  **`add-python-binding`** — capability `python-binding`. Pure consumer of the
  ABI; documented in `docs/python.md`. *(implemented + verified on the desktop —
  `python -m pytest python/tests` = **35 passed, 1 skipped** (offscreen GL
  render, no GL context; matplotlib PNG fallback + STL round-trip assert real
  geometry), asserting REAL geometry through Python: box volume 1000 / area 600
  / centroid (5,5,5), boolean cut 875 / fuse 1875 / common 125, exact bbox,
  revolved cylinder `πr²h`, watertight tessellation, STEP + IGES round-trip
  preserve volume, and context-managed handle lifetime.)* **Deferred:** pybind11
  variant, interactive `pyvista` render, wheel packaging.

## Change index

Phase → change → capability → status. Update the status column and each phase's
checkboxes as changes land; flip to ✅ when a change is validated and archived.

| Phase | Change | Capability(ies) | Status |
|---|---|---|---|
| 0 | `add-kernel-foundation` | kernel-facade, engine-adapter, operation-scheduler, compute-backend | ✅ complete at acceptance bar (host tests + all 57 `cc_*` on iOS-sim, 221/221); app link-swap = optional deferred |
| 1 | `accelerate-multicore-occt` | parallel-acceleration | ✅ complete at acceptance bar (parallel paths on iOS-sim; determinism audit + serial-vs-parallel benchmark done); on-device scaling = optional deferred |
| 2 | `add-metal-compute-backend` | metal-backend | ✅ complete at acceptance bar (backend self-test PASS on iOS-sim GPU; fp32-only + precision guard; host CTest green with METAL=OFF); on-device run = optional deferred |
| 2 | `add-gpu-tessellation` | gpu-tessellation | ✅ complete at acceptance bar (GPU surface-eval + per-vertex normals on iOS-sim GPU 18/18; GPU eval wired into `cc_tessellate` behind the toggle, integ suite 26/26 GPU-fed-vs-OCCT parity, GPU-OFF suite 221/221); repeat-run determinism assertion + GPU tessellation of holed/trimmed faces (falls back to OCCT by design) deferred |
| 2 | `add-gpu-spatial-acceleration` | spatial-acceleration | ✅ complete at acceptance bar (iOS-sim GPU: LBVH nearest-hit + batched ray-pick + **frustum-pick** all vs CPU reference, GPU pick suite **26/26** — frustum set == CPU set {0,1}/empty/all-enclosing, sorted ascending, **byte-identical ×8 runs** for ray + frustum); optional additive `cc_*` pick/cull facade entry = app-facing, out of scope |
| 3 | `add-g2-blend-fillet` | g2-blend | ✅ complete at acceptance bar (#284) (iOS-sim: valid+watertight, MEASURED curvature gap 0.018835 within G2 tol 0.05, G1 baseline 0.309740 fails, bit-exact determinism); on-device + app link-swap = optional deferred |
| 3 | `add-full-round-fillet` | full-round-fillet | ◐ in progress (#285) (iOS-sim: true rolling-ball blend verified for tangent/parallel walls — middle face consumed, G1 dot=1.000000, deterministic; **deferred**: non-parallel walls 22.62° → valid standard edge fillet, face not consumed) |
| 3 | `add-robust-thread-boolean` | thread-boolean | ✅ complete at acceptance bar (#286) (iOS-sim: FUSE 4.38 s / CUT 4.48 s both < 8 s budget, valid+watertight, correct volume sign, naive path not run; determinism within rel 2e-4, not bit-exact — parallel BOPAlgo); on-device = optional deferred |
| 3 | `add-robust-wrap-emboss` | wrap-emboss | ✅ complete at acceptance bar (#290) (iOS-sim: emboss+deboss valid+watertight, correct volume sign, reproducible, high-curvature valid; sewn→coarse fallback); on-device + app link-swap = optional deferred |
| 3 | `add-reference-geometry` | reference-geometry | ✅ complete at acceptance bar (iOS-sim: 21/21 — datum planes/axes within 1e-9, 6/6 faces + 12/12 edges + cyl axis, degenerate guards hold); host stub returns 0 for derived; on-device = optional deferred |
| 4 | `add-native-math-geometry` | native-math | ☐ planned |
| 4 | `add-native-brep-topology` | native-topology | ☐ planned |
| 4 | `add-native-tessellation` | native-tessellation | ☐ planned |
| 4 | `add-native-swept-solids` | native-construction | ☐ planned |
| 4 | `add-native-booleans` | native-booleans | ☐ planned |
| 4 | `add-native-fillets-offsets` | native-blends | ☐ planned |
| 4 | `add-native-data-exchange` | native-exchange | ☐ planned |
| 4 | `drop-occt` | — (retires OCCT adapter) | ☐ planned |
| Tooling | `add-python-binding` | python-binding | ◐ implemented + verified on desktop (`pytest python/tests` 35 passed / 1 skipped — real geometry: box 1000/area 600, cut 875, fuse 1875, common 125, watertight tessellation, STEP+IGES round-trip; Homebrew-OCCT dylib via `scripts/build-macos-dylib.sh`, `cc_brep_available()==1`); pure ABI consumer, not shipped to iOS; deferred: pybind11, pyvista, wheel |

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

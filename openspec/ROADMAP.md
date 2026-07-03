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
> G1-tangency, or a MEASURED curvature gap). Result: **70 passed, 0 failed, 0
> deferred**. On-device runs + app link-swap are optional/deferred. See
> `docs/STATUS-phase-3.md`.

- ✅ **Curvature-continuous (G2) fillet / blend surfaces** (OCCT is G1/circular
  only). Change **`add-g2-blend-fillet`** — capability `g2-blend`. Contract:
  `occt-usage` §Fillets & chamfers limitation (GitHub #284); `cc_fillet_edges`.
  *(implemented; **verified on the iOS sim** — valid + watertight solid, MEASURED
  seam curvature gap **0.018835 within G2 tol 0.05** (1/r=0.333333) while the stock
  G1 baseline **0.309740 fails** the bar; G2 measurably smaller than G1; bit-exact
  determinism (dV=dBBox=dGap=0). G2 is claimed because the numbers show it.)*
- ✅ **Rolling-ball / full-round fillet.** Changes **`add-full-round-fillet`** +
  **`enhance-full-round-nonparallel`** — capability `full-round-fillet`
  (GitHub #285). *(implemented; **verified on the iOS sim** for BOTH planar
  configurations. PARALLEL walls — 10 checks: middle face consumed, cylinder
  blend, axis equidistant, **G1-tangent both seams dot=1.000000** (tol
  cos1°=0.999848), deterministic, single-arg auto-detect matches. NON-PARALLEL
  dihedral — genuinely non-parallel fixture (n_L·n_R=-0.7241, 43.60° off-parallel):
  valid + watertight (vol=628.5665), middle strip consumed, blend is a cylinder
  along the crease with axis equidistant (rL=rR=2.9541), **G1-tangent to BOTH
  non-parallel walls** cos(left)=cos(right)=1.000000. **Residual (by design):**
  truly CURVED (non-planar) neighbours fall back to a VALID standard edge fillet,
  recorded deferred with the measured tangency gap.)*
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
time, each validated against the OCCT path (host unit tests + native-vs-OCCT
parity on the simulator), then OCCT unlinked at the final step. Committed to full
drop-OCCT incl. native booleans (research-grade; hardened progressively).
**Method + verification model + full capability sequence: see the sub-roadmap
[NATIVE-REWRITE.md](NATIVE-REWRITE.md).** Clean-room from references with OCCT as
the numeric oracle; native code is host-buildable (OCCT-free).
- ✅ Math & geometry primitives (points/vectors/transforms, curves/surfaces
  eval). Change **`add-native-math-geometry`** — capability `native-math`.
  Contract: `occt-usage` §Modeling-data types (`gp_*`, `Geom_*`). *(done at the
  verification bar — first native capability. Host analytic tests (55 asserts, no
  OCCT) + native-vs-OCCT parity on iOS sim (24 groups, 0 failed, overall max
  numeric error 1.486e-13, well under tolerance); no regressions (host CTest 8/8,
  `run-sim-suite.sh` 221/221). OCCT-free math foundation only — not yet
  engine-wired by design. Detail: `docs/STATUS-phase-4.md`.)*
- ✅ B-rep topology data model + exploration. Change
  **`add-native-brep-topology`** — capability `native-topology`. Contract:
  `occt-usage` §Modeling-data (`TopoDS`, `TopExp`, sub-shape ids). *(done at the
  verification bar — second native capability. Host invariant tests
  (`test_native_topology`, 13 cases, no OCCT) + native-vs-OCCT parity on iOS sim
  (3 shapes — box/cylinder/filleted-box — × 5 checks = 15 passed, 0 failed;
  sub-shape counts + `MapShapes` order + edge→faces ancestry + orientation flags
  match the oracle, accessor max error 0.000e+00 at tol 1.0e-09, surface types
  match); no regressions (host CTest 9/9, `run-sim-suite.sh` 221/221). Header-only
  `src/native/topology/`, not engine-wired by design. Deferred: non-manifold /
  degenerate + seam edges, `CompSolid`/`Internal`/`External`, holed-face parity
  fixture. Detail: `docs/STATUS-phase-4.md`.)*
- ✅ Tessellation / meshing (native, GPU-backed via Phase 2). Change
  **`add-native-tessellation`** — capability `native-tessellation`. Contract:
  `occt-usage` §Meshing (`BRepMesh`). *(done at the verification bar — third native
  capability. Host invariant tests (`test_native_tessellate`, no OCCT — deflection
  bound / on-surface / trimming / watertightness / area-volume convergence /
  determinism) + native-vs-OCCT `BRepMesh` property-parity on iOS sim (4 shapes —
  box / cylinder / sphere / filleted-box — All 20 checks PASS; ALL four closed solids
  watertight `boundaryEdges==0`; area/volume relMesh ≤ 6.0e-3, relExact ≤ 1.24e-2,
  bbox max corner delta ≤ 4.66e-2, on-surface residual ≤ 5.7e-15; triangle
  count/topology NOT compared). No regressions (host CTest 10/10, `test_native_tessellate`
  13 cases, `run-sim-suite.sh` 221/221). Header-only `src/native/tessellate/`, not
  engine-wired by design. RESOLVED: curved shared-edge stitch — two-stage mesher
  (shared per-edge 1D discretization, STAGE 1 `edge_mesher.h`, consumed by both
  faces in STAGE 2, as OCCT `BRepMesh` does), so ALL closed solids
  (box/cylinder/sphere/filleted-box) mesh WATERTIGHT (`boundaryEdges==0`, now
  required — no weaker bounded-open pass). Deferred (not watertightness): GPU fp32
  path CPU-verified only, ear-clip trim quality, adaptive refinement. Detail:
  `docs/STATUS-phase-4.md`.)*
- ◐ Primitive & swept-solid construction (extrude/revolve/loft/sweep). Change
  **`add-native-construction`** — capability `native-construction`. Contract:
  `occt-usage` §Primitive & swept-solid, §B-rep construction, §Offsets/sweeps.
  *(CORE done at the verification bar — the first ENGINE-WIRED capability. Native
  `cc_solid_extrude` (closed polygon → prism) + native `cc_solid_revolve`
  (LINE-SEGMENT profile → cylinder/plane/cone faces of revolution; full 360° closes,
  partial adds planar caps), OCCT-free under `src/native/construct/`, wired through a
  `NativeEngine : IEngine` (`src/engine/native/`) that falls through to OCCT for
  everything else, behind an additive `cc_set_engine`/`cc_active_engine` toggle
  (**default stays OCCT**). Both gates green: host `test_native_construct` +
  `test_native_engine` (CTest 12/12, no OCCT) + native-vs-OCCT parity on the iOS sim
  through the facade (17/17 — planar prisms EXACT vol/area/centroid rel 0.00e+00,
  curved revolves within a deflection bound vol rel ≤ 2.36e-2 watertight, plus a
  fall-through boolean proving no native interception). No regressions (host CTest
  12/12, `run-sim-suite.sh` 221/221 re-verified against a rebuilt sim slice). Archived
  to `openspec/specs/native-construction`. Follow-up `#4b`: Tier A (holed / typed-profile
  extrude + typed-profile revolve) and Tier B (2-section ruled loft) now DONE at the bar;
  still OCCT-fallthrough (not faked): sweep, twisted/guided/rail sweep, threads,
  wrap-emboss, 3+-section / guided / rail loft, arc/spline revolve. Detail:
  `docs/STATUS-phase-4.md`.)*
- ☐ **NEXT** — Booleans (native robust kernel — the hardest; longest-lived OCCT
  dependency; **research-grade**). Change **`add-native-booleans`** — capability
  `native-booleans`. Contract: `occt-usage` §Boolean operations. Requires
  surface-surface intersection, robust section-edge classification, and shape
  healing at near-tangent/coincident configs (the BOPAlgo wall); lands progressively
  hardened and verified against OCCT, not production-robust day one.
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
| 3 | `add-full-round-fillet` + `enhance-full-round-nonparallel` | full-round-fillet | ✅ complete at acceptance bar (#285) (iOS-sim: true rolling-ball blend verified for ALL planar walls — parallel (middle consumed, G1 dot=1.000000, deterministic) AND non-parallel dihedral (fixture n_L·n_R=-0.7241, 43.60°; valid+watertight vol=628.5665, middle consumed, G1 cos(left)=cos(right)=1.000000, axis equidistant rL=rR=2.9541); residual by design: curved/non-planar neighbours → valid standard edge fillet, deferred with measured gap); on-device + app link-swap = optional deferred |
| 3 | `add-robust-thread-boolean` | thread-boolean | ✅ complete at acceptance bar (#286) (iOS-sim: FUSE 4.38 s / CUT 4.48 s both < 8 s budget, valid+watertight, correct volume sign, naive path not run; determinism within rel 2e-4, not bit-exact — parallel BOPAlgo); on-device = optional deferred |
| 3 | `add-robust-wrap-emboss` | wrap-emboss | ✅ complete at acceptance bar (#290) (iOS-sim: emboss+deboss valid+watertight, correct volume sign, reproducible, high-curvature valid; sewn→coarse fallback); on-device + app link-swap = optional deferred |
| 3 | `add-reference-geometry` | reference-geometry | ✅ complete at acceptance bar (iOS-sim: 21/21 — datum planes/axes within 1e-9, 6/6 faces + 12/12 edges + cyl axis, degenerate guards hold); host stub returns 0 for derived; on-device = optional deferred |
| 4 | `add-native-math-geometry` | native-math | ✅ done at verification bar (host analytic tests 55 asserts no-OCCT + iOS-sim native-vs-OCCT parity 24 groups/0 failed, overall max err 1.486e-13; host CTest 8/8, `run-sim-suite.sh` 221/221; OCCT-free math foundation, not engine-wired by design); archived to `openspec/specs/native-math` |
| 4 | `add-native-brep-topology` | native-topology | ✅ done at verification bar (host invariant tests `test_native_topology` 13 cases no-OCCT + iOS-sim native-vs-OCCT parity 3 shapes × 5 checks = 15/15, accessor max err 0.000e+00; host CTest 9/9, `run-sim-suite.sh` 221/221; header-only, not engine-wired by design; deferred: non-manifold/degenerate+seam edges, `CompSolid`/`Internal`/`External`, holed-face fixture); archived to `openspec/specs/native-topology` |
| 4 | `add-native-tessellation` | native-tessellation | ✅ done at verification bar (host invariant tests `test_native_tessellate` no-OCCT + iOS-sim native-vs-OCCT `BRepMesh` property-parity 4 shapes All 20 checks PASS — ALL four closed solids watertight `boundaryEdges==0`; area/volume relMesh ≤ 6.0e-3, relExact ≤ 1.24e-2, bbox maxCornerΔ ≤ 4.66e-2, on-surface residual ≤ 5.7e-15; host CTest 10/10, `run-sim-suite.sh` 221/221; header-only, not engine-wired by design; RESOLVED curved shared-edge stitch (two-stage shared per-edge discretization); deferred (not watertightness): ear-clip trim re-triangulation quality, adaptive refinement, GPU fp32 CPU-verified only); archived to `openspec/specs/native-tessellation` |
| 4 | `add-native-construction` | native-construction | ◐ CORE done at verification bar — first engine-wired capability. Native `cc_solid_extrude` (polygon prism) + `cc_solid_revolve` (line-segment) via `NativeEngine : IEngine` (`src/engine/native/`) falling through to OCCT for the rest, behind additive `cc_set_engine`/`cc_active_engine` (default OCCT). Host `test_native_construct`+`test_native_engine` CTest 12/12 no-OCCT + iOS-sim native-vs-OCCT parity through facade 17/17 (planar prisms EXACT rel 0.00e+00; curved revolves vol rel ≤ 2.36e-2 watertight; fall-through boolean); no regressions (`run-sim-suite.sh` 221/221 re-verified vs rebuilt sim slice); archived to `openspec/specs/native-construction`. Follow-up `#4b` (OCCT-fallthrough, not faked): loft, sweep, twisted/guided sweep, threads, holed/typed-profile extrude, arc/spline revolve |
| 4b | `add-native-construction-profiles` (Tier A) | native-construction (advanced) | ◐ **Tier A done at verification bar** — `cc_solid_extrude_holes` (circular holes, TRUE Circle edge + Cylinder wall), `cc_solid_extrude_polyholes` (polygon holes), `cc_solid_extrude_profile`/`_profile_polyholes` (typed line/arc/full-circle outer + holes), `cc_solid_revolve_profile` (line → Plane/Cylinder/Cone, on-axis arc → Sphere) NATIVE (`src/native/construct/profile.h`). Host `test_native_profile`+`test_native_engine` CTest 13/13 no-OCCT + iOS-sim native-vs-OCCT parity `native_construct_profiles_parity.mm` 22/22 (polyhole EXACT rel 1.97e-16; curved vol rel ≤ 4.97e-2 watertight; 2 deferred sub-cases fall through to OCCT rel 0.00e+00); no regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): kind-3 SPLINE edges, off-axis-arc (torus)/spline revolve. |
| 4b | `add-native-loft` (Tier B) | native-construction (advanced) | ✅ **Tier B done at verification bar** — `cc_solid_loft` / `cc_solid_loft_wires` for TWO PLANAR sections with EQUAL vertex counts (≥3) NATIVE: one BILINEAR (degree-1 Bézier) ruled side face per corresponding edge pair + two planar caps → watertight solid (mirrors ruled `BRepOffsetAPI_ThruSections`; `src/native/construct/loft.h`, cognitive complexity ≤ 7). Host `test_native_loft` (9 cases) + `test_native_engine` CTest **14/14** no-OCCT + iOS-sim native-vs-OCCT parity `native_loft_parity.mm` **17/17** (square-frustum rel 2.54e-16 / hex-prism rel 0.00e+00 / tri-prism loft_wires rel 0.00e+00 EXACT; rotated-square twist vol rel 5.33e-3 watertight; mismatched-count fall-through to OCCT rel 0.00e+00). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): loft with MISMATCHED counts / NON-PLANAR / point-collapse section / 3+ sections / guided / rail (Tier C). |
| 4b | `add-native-swept-solids` (Tiers C–E) | native-construction (advanced) | ☐ follow-up — C sweep+twisted/guided/rail + 3+-section loft / D threads / E wrap-emboss, currently OCCT-fallthrough |
| 5 | `add-native-booleans` | native-booleans | ☐ NEXT (**research-grade**) — robust B-rep booleans: surface-surface intersection + section-edge classification + shape healing; hardened progressively vs OCCT/BOPAlgo |
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

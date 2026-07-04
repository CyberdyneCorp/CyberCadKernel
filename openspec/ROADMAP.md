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
  extrude + typed-profile revolve), Tier B (2-section ruled loft), Tier C (sweep along
  a straight or smooth-planar spine), Tier D (`cc_tapered_shank` + well-formed
  `cc_helical_thread` / `cc_tapered_thread` — the per-turn seam weld is fixed at the mesher
  level so threads mesh `boundaryEdges==0` at every deflection and run NATIVE), and the
  geometry-completion batch (kind-3 SPLINE profile edge extrude, off-axis-arc TORUS revolve,
  N-section (3+) ruled loft, NON-PLANAR (RMF) sweep) now DONE at the bar; still
  OCCT-fallthrough / DECLINE (not faked, all needing SSI / Tier-4): tight-curvature /
  real-twist / self-intersecting / guided / rail sweep, a fine-pitch / self-intersecting
  thread (non-manifold → self-verify defers to OCCT `MakePipeShell`), wrap-emboss, a
  mismatched-count / non-planar / guided / hard-rail loft, a general SPLINE
  surface-of-revolution, and a spindle torus.
  Detail: `docs/STATUS-phase-4.md`.)*
- ✅ **Numeric foundations (native-rewrite #2 — the substrate under booleans/blends).**
  Change **`add-native-numerics`** — **NumPP + SciPP ADOPTED** as the kernel's OCCT-free
  numeric substrate (the org's C++20, MIT NumPy/SciPy ports; absolute-path, NOT vendored;
  CPU-only; `optimize`/`linalg`(+`spatial`/`integrate`) subset, `special`/`stats` EXCLUDED;
  guarded by `CYBERCAD_HAS_NUMSCI`, default OFF; built via `scripts/build-numsci.sh`). A
  thin OCCT-free facade (`src/native/numerics/`) exposes the generic solvers
  (root / `fsolve` / `minimize`(BFGS) / `least_squares`(LM) / `solve` / `lstsq`) and native
  **closest-point / projection** (the `Extrema` on-ramp — point→curve / point→surface,
  multi-start + SciPP refine, global-best foot). Both gates green: host `test_native_numerics`
  (22 assertions, no OCCT, built under `CYBERCAD_HAS_NUMSCI=ON`) + iOS-sim native-vs-OCCT
  `Extrema` parity `native_numerics_parity.mm` **22/22 `[NNUM]`** — dDist ≤ 1.776e-15
  (analytic plane/cylinder/sphere feet fp-exact dPoint ≤ 1.707e-10; B-spline within tol,
  largest `bspline_surf#3` dPoint 3.946e-08 at corner). Substrate compiles+links 77/77 TUs
  HOST + arm64-iOS-simulator. Realizes the eval's ~60–75% #2 effort saving (→ ~0.15–0.35 py),
  moving numeric foundations OFF the critical path; archived to
  `openspec/specs/native-numerics`. No regressions: host `NUMSCI=OFF` CTest 22/22
  (`test_native_numerics` correctly ABSENT), `NUMSCI=ON` 23/23, `run-sim-suite.sh` 221/221.
  Deferred (not blocking): multiple-extrema enumeration, curve-curve / surface-surface
  distance. **SSI (near-tangent) is NOT bought — it stays the booleans capability below.**
- ◐ Booleans (native robust kernel — the hardest; longest-lived OCCT dependency;
  **research-grade**). Change **`add-native-booleans`** — capability
  `native-booleans`. Contract: `occt-usage` §Boolean operations. **PLANAR-polyhedron
  slice DONE at the verification bar:** native `cc_boolean` (fuse / cut / common) for
  planar-faced solids (axis-aligned boxes, prisms) via a BSP-tree CSG
  (`src/native/boolean/`), guarded by a mandatory self-verify (`robustlyWatertight` +
  set-algebra volume) that discards + falls through to OCCT otherwise. Both gates green:
  host `test_native_boolean`+`test_native_engine` CTest **17/17** no-OCCT + iOS-sim
  native-vs-OCCT parity `native_boolean_parity.mm` **25/25** (box fuse rel 1.27e-16 / cut
  2.96e-16 / common 2.22e-16, contained fuse 0.00e+00 / common 2.22e-16 all EXACT +
  watertight; self-verify rejects native∩native disjoint; curved cyl-box / near-coincident
  / disjoint OCCT-fallthrough rel 0.00e+00, no interception); no regressions
  (`run-sim-suite.sh` 221/221, `test_native_tessellate` green); archived to
  `openspec/specs/native-booleans`. **CURVED analytic slice DONE at the verification bar
  (both gates green), archived:** change **`add-native-curved-booleans`** — `cc_boolean`
  (cut / fuse / common) is native for an **AXIS-ALIGNED box ⟷ axis-parallel cylinder**
  (cylinder radially inside the box), built as a closed-form `Cylinder`+`Circle`+`Plane`
  B-rep (cut → round THROUGH hole `boxVol − πr²·h`, fuse → protruding BOSS, common →
  clipped segment) guarded by an analytic-volume self-verify (`src/native/boolean/curved.h`).
  Both gates green: host CTest **18/18** no-OCCT + iOS-sim `[NCURVBOOL]` **18 checks (6×3),
  0 failed** — 3 NATIVE (through-hole-cut rel 3.19e-04, boss-fuse rel 6.10e-05, common rel
  1.30e-03, all watertight) + 3 OCCT-fallback (blind-hole-cut / oblique-cyl-cut /
  sphere-box-cut, rel 0 forwarded); no regressions (`run-sim-suite.sh` 221/221, host CTest
  19/19). This partially unblocks the boolean step of an axis-aligned-cylinder-target
  wrap-emboss (#4b-E). STILL OCCT (research-grade, not faked): **general curved-face
  booleans** (surface-surface intersection: sphere / cone / NURBS / non-axis-aligned /
  cyl-cyl / blind-hole / non-through cut), near-tangent / coincident, general /
  concave-general, foreign operands, shape healing — booleans remain the longest-lived
  OCCT dependency for general curved.
- ◐ Fillets/chamfers/offsets/shell. Change **`add-native-fillets-offsets`** —
  capability `native-blends`. Contract: `occt-usage` §Fillets & chamfers,
  §Offsets/sweeps/lofts/shells. **TRACTABLE-PLANAR slice done at the verification bar
  (both gates green); curved/concave/variable/fillet_face OCCT-fallthrough.** NATIVE:
  `cc_chamfer_edges` (convex planar-planar edge, EXACT), `cc_offset_face` (planar face,
  EXACT slab), `cc_shell` (box-like solid, EXACT wall), `cc_fillet_edges` (CONSTANT
  radius on a convex planar-DIHEDRAL edge — rolling-ball tangent cylinder,
  deflection-bounded), each guarded by a MANDATORY self-verify (`blendResultVerified` —
  watertight + sane volume sign) that discards + falls through to OCCT (`src/native/blend/`).
  Host `test_native_blend`+`test_native_engine` CTest **18/18** no-OCCT + iOS-sim
  native-vs-OCCT parity `native_blend_parity.mm` **[NBLEND] 16/16** (chamfer rel 2.29e-16 /
  offset rel 4.55e-16 / shell rel 4.02e-16 EXACT + watertight; constant fillet
  deflection-bounded rel 8.96e-05; curved-rim fillet forwarded to OCCT rel 0.00e+00;
  self-verify rejects an oversized shell). No regressions (`test_native_tessellate` green,
  `run-sim-suite.sh` 221/221); archived to `openspec/specs/native-blends`. Native offset
  now UNBLOCKS the planar slice of native wrap-emboss (#4b-E). STILL OCCT-fallthrough
  (not faked): CURVED-face inputs, CONCAVE edges, variable-radius `cc_fillet_edges_variable`,
  `cc_fillet_face`, multi-edge interference, non-convex/oversized shell — general blending
  is future work.
- ◐ Data exchange — **native STEP EXPORT slice done at the verification bar (both
  gates green); STEP import + IGES stay OCCT (honest, out of scope).** Change
  **`add-native-data-exchange`** — capability `native-exchange`. Contract: `occt-usage`
  §Data exchange. NATIVE: `cc_step_export` (engine-wired behind `cc_set_engine(1)`) walks
  an in-scope native B-rep and emits a valid ISO-10303-21 STEP **AP203** file in true
  millimetres, OCCT-free under `src/native/exchange/` (geometric dedup → one shared
  `EDGE_CURVE` per physical edge → a sewn manifold `CLOSED_SHELL`). Both gates green: host
  `test_native_step_writer` + `test_native_step` + `test_native_engine` CTest **21/21**
  no-OCCT + iOS-sim OCCT re-read round-trip (source → native-written STEP → OCCT re-read) —
  box relV 2.27e-16 (6→6 faces, 24→24 edges), cylinder relV 1.27e-03 (9→9 faces),
  holed-plate relV 2.90e-04 (7→7 faces, 28→30 edges within tol); writer parity
  native-vs-OCCT relV ≤ 4.70e-15. Native writer active (box 5363 B / cylinder 6893 B /
  holed-plate 6457 B); a foreign OCCT-built body falls through to OCCT `STEPControl_Writer`
  (re-read relV 0.00e+00). No regressions (`run-sim-suite.sh` 221/221 against a rebuilt sim
  slice, `test_native_tessellate` green); archived to `openspec/specs/native-exchange`.
  STILL OCCT (never faked, out of scope): **STEP import**, **IGES export/import**, and an
  out-of-scope geometry kind (Ellipse/Bezier curve, rational spline, Bezier surface).
- ☐ **Drop OCCT** — **NOT reachable at the native ceiling; BLOCKED.** Change
  **`drop-occt`** would retire the OCCT adapter, but it requires EVERY `cc_*` path to be
  native. Two hard dependencies remain research-grade multi-year efforts: (1) a general
  robust curved boolean / blend kernel (arbitrary surface-surface intersection + shape
  healing) and (2) native STEP/IGES IMPORT (the #7 slice delivered EXPORT only). Until
  both exist, OCCT stays linked and **Phase 4 stands COMPLETE AT ITS ACHIEVABLE NATIVE
  CEILING, not fully drop-OCCT.**

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
| 4 | `add-native-construction` | native-construction | ◐ CORE done at verification bar — first engine-wired capability. Native `cc_solid_extrude` (polygon prism) + `cc_solid_revolve` (line-segment) via `NativeEngine : IEngine` (`src/engine/native/`) falling through to OCCT for the rest, behind additive `cc_set_engine`/`cc_active_engine` (default OCCT). Host `test_native_construct`+`test_native_engine` CTest 12/12 no-OCCT + iOS-sim native-vs-OCCT parity through facade 17/17 (planar prisms EXACT rel 0.00e+00; curved revolves vol rel ≤ 2.36e-2 watertight; fall-through boolean); no regressions (`run-sim-suite.sh` 221/221 re-verified vs rebuilt sim slice); archived to `openspec/specs/native-construction`. Follow-up `#4b` (see the Tier A–E rows below): holed/typed-profile extrude + typed revolve (A), 2-section ruled loft (B), straight/smooth-planar sweep (C), tapered-shank + well-formed helical/tapered thread (D) now NATIVE; still OCCT-fallthrough (not faked): twisted/guided/rail sweep, 3+-section/guided/rail loft, arc/spline revolve, fine-pitch thread, wrap-emboss (E) |
| 4b | `add-native-construction-profiles` (Tier A) | native-construction (advanced) | ◐ **Tier A done at verification bar** — `cc_solid_extrude_holes` (circular holes, TRUE Circle edge + Cylinder wall), `cc_solid_extrude_polyholes` (polygon holes), `cc_solid_extrude_profile`/`_profile_polyholes` (typed line/arc/full-circle outer + holes), `cc_solid_revolve_profile` (line → Plane/Cylinder/Cone, on-axis arc → Sphere) NATIVE (`src/native/construct/profile.h`). Host `test_native_profile`+`test_native_engine` CTest 13/13 no-OCCT + iOS-sim native-vs-OCCT parity `native_construct_profiles_parity.mm` 22/22 (polyhole EXACT rel 1.97e-16; curved vol rel ≤ 4.97e-2 watertight; 2 deferred sub-cases fall through to OCCT rel 0.00e+00); no regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): kind-3 SPLINE edges, off-axis-arc (torus)/spline revolve. |
| 4b | `add-native-loft` (Tier B) | native-construction (advanced) | ✅ **Tier B done at verification bar** — `cc_solid_loft` / `cc_solid_loft_wires` for TWO PLANAR sections with EQUAL vertex counts (≥3) NATIVE: one BILINEAR (degree-1 Bézier) ruled side face per corresponding edge pair + two planar caps → watertight solid (mirrors ruled `BRepOffsetAPI_ThruSections`; `src/native/construct/loft.h`, cognitive complexity ≤ 7). Host `test_native_loft` (9 cases) + `test_native_engine` CTest **14/14** no-OCCT + iOS-sim native-vs-OCCT parity `native_loft_parity.mm` **17/17** (square-frustum rel 2.54e-16 / hex-prism rel 0.00e+00 / tri-prism loft_wires rel 0.00e+00 EXACT; rotated-square twist vol rel 5.33e-3 watertight; mismatched-count fall-through to OCCT rel 0.00e+00). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): loft with MISMATCHED counts / NON-PLANAR / point-collapse section / 3+ sections / guided / rail (Tier C). |
| 4b | `add-native-sweep` (Tier C) | native-construction (advanced) | ✅ **Tier C done at verification bar** — `cc_solid_sweep` for a STRAIGHT spine (EXACT directional prism, vol = profileArea×\|d\|) and a SMOOTH CURVED but PLANAR spine (CONSTANT-frame ruled tube matching OCCT MakePipe's planar `GeomFill_CorrectedFrenet`→`Law_Constant` — section TRANSLATED, not perpendicular; NOT Pappus volume) NATIVE (`src/native/construct/sweep.h`, reuses `loft.h` `ruledSideFace` + `construct.h` `planarFace`; `build_sweep` cognitive complexity 14). `cc_twisted_sweep` native only when twist ≈ 0 AND scale ≈ 1. An earlier RMF/double-reflection revision REMOVED (produced Pappus volume — real oracle mismatch). Host `test_native_sweep` (11 cases) + `test_native_engine` (3 sweep cases) CTest **15/15** no-OCCT + iOS-sim native-vs-OCCT parity `native_sweep_parity.mm` **11/11** (8 native + 3 fallback — straight EXACT vol rel 7.11e-16, smooth-arc EXACT vol o=330.299 n=330.299 rel 1.72e-16 native F=OCCT F=98 watertight; real-twist/guided/loft-rail fall-through to OCCT rel 0.00e+00 native active). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). STILL OCCT-fallthrough (not faked): NON-PLANAR / TIGHT-CURVATURE / self-intersecting sweep spine, REAL twist/scale, `cc_guided_sweep` / `cc_loft_along_rail`. |
| 4b | `add-native-threads` (Tier D) | native-construction (advanced) | ◐ **Tier D done at verification bar — shank + well-formed threads NATIVE; fine-pitch thread OCCT-fallthrough** — `cc_tapered_shank` (pointed-shank silhouette cone tip → full-radius cylinder → head disk revolved 360° about WORLD Z by reusing native `build_revolution`; TRUE on-axis apex, robustly watertight; matches OCCT `BRepPrimAPI_MakeRevol`) NATIVE (`src/native/construct/thread.h`, cognitive complexity ≤ 5). `cc_helical_thread` / `cc_tapered_thread` build the full radial-V axis-aux-spine helical tiling (three bilinear ruled bands per span + two planar V caps); the per-turn seam weld — the last blocker — is now fixed at the mesher level (canonical seam-point snap, `edge_mesher.h` `CanonicalEndpoints` / `face_mesher.h` `BoundaryAnchors`), so a well-formed thread meshes `boundaryEdges==0` at EVERY deflection across the full parameter sweep (432/432 helical + 96/96 tapered → native), passes the engine `robustlyWatertight` self-verify and runs NATIVE. Only a FINE-PITCH / self-intersecting thread (non-manifold regardless of weld) still FALLS THROUGH to OCCT `MakePipeShell` (labelled, verified, never faked). Host `test_native_thread` (9 cases — incl. multi-deflection watertight ladder + fine-pitch guard) + `test_native_engine` (`native_thread_runs_native_watertight` + `native_fine_pitch_thread_falls_through_to_default`) CTest **18/18** no-OCCT + iOS-sim native-vs-OCCT parity `native_thread_parity.mm` — `cc_tapered_shank` NATIVE r5/fh20/th10 vol o=1837.94 n=1830.27 rel 4.17e-03 / watertight 144 tris; `cc_helical_thread` NATIVE mr5/p2/t4/d1 vol o=70.2841 n=68.3767 rel 2.71e-02 / F 5→194 / watertight 1286 tris meshVolRel 1.40e-03; `cc_tapered_thread` NATIVE top6/tip4/p2/t4 vol o=70.2677 n=68.3767 rel 2.69e-02 / watertight 1286 tris (the ~2.7% native-vs-OCCT volume gap is chord-vs-arc at spt=16, native mesh-vs-B-rep meshVolRel ≤ 1.40e-3), plus a fine-pitch thread OCCT fall-through (native active=1, vol rel 0.00e+00). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221). |
| 4b | `add-native-geometry-completion` (Tier 1 + Tier 2#4) | native-construction (advanced) | ◐ **done at verification bar** — kind-3 SPLINE profile edge extrude + off-axis-arc TORUS revolve (native `Torus` in `src/native/math/torus.h`; exact rational-quadratic B-spline patches — `src/native/construct/residuals.h`), N-section (3+) ruled loft chain (`loft.h`), and a NON-PLANAR (double-reflection RMF) sweep (`sweep.h`) NOW NATIVE. Host CTest **22/22** no-OCCT (incl. `test_native_residuals`) + iOS-sim native-vs-OCCT parity `native_geomcompletion_parity.mm`: spline extrude vol rel 9.92e-04, torus revolve rel 2.68e-02, N-section frustum + straight-rail loft rel ≤ 1.4e-14 EXACT, RMF smooth-arc sweep rel 3.44e-16 EXACT — all watertight. Honest fall-through / DECLINE (not faked, all needing SSI / Tier-4): self-crossing spline + spindle torus DECLINE on BOTH engines (occtId=0 natId=0); mismatched-count loft → OCCT `ThruSections`, hard curved rail → OCCT `MakePipeShell`, self-intersecting sweep → OCCT `MakePipe`, real-twist `cc_twisted_sweep` → OCCT `ThruSections`, self-intersecting thread → OCCT `MakePipeShell` (native active=1, rel 0.00e+00). The accumulating-twist/scale sweep, guided/rail cases, and the thread self-intersection resolver did NOT self-verify oracle-correct beyond the well-formed set. No regressions (`run-sim-suite.sh` 221/221). Archived to `openspec/specs/native-construction`. |
| 4b | `add-native-swept-solids` (Tier E) | native-construction (advanced) | ☐ follow-up — E wrap-emboss (+ mismatched-count / guided / hard-rail loft, tight-curvature / guided / rail / real-twist sweep — all SSI / Tier-4), currently OCCT-fallthrough. Robust-watertight `cc_helical_thread` / `cc_tapered_thread` are **NO LONGER deferred** — the mesher shared-edge seam weld is done (Tier D), so well-formed threads run NATIVE; only a fine-pitch / self-intersecting thread stays OCCT-fallthrough. N-section (3+) ruled loft, NON-PLANAR (RMF) sweep, spline extrude, and off-axis-arc TORUS revolve are **NO LONGER deferred** (done by the geometry-completion batch above). **PARTIALLY UNBLOCKED by #5:** wrap-emboss's add/subtract step is a boolean, so a PLANAR-polyhedron emboss/deboss can now use the native BSP-CSG booleans (fuse/cut) instead of OCCT; curved-surface wrap-emboss still needs curved native booleans (#5 curved slice) + a curved-surface native offset. **#6 (native `cc_offset_face`) has now LANDED** the planar offset-along-normal, so a PLANAR-target wrap-emboss is reachable natively (native planar offset + native planar boolean); only the curved-surface slice still waits on curved #5 + curved offset. |
| 5 | `add-native-booleans` | native-booleans | ◐ **PLANAR-polyhedron slice done at verification bar; curved/general OCCT-fallthrough** — native `cc_boolean` (fuse/cut/common) for planar-faced solids (axis-aligned boxes, prisms) via BSP-tree CSG (`src/native/boolean/`), guarded by mandatory self-verify (`robustlyWatertight` + set-algebra volume) that discards + falls through to OCCT otherwise. Host `test_native_boolean`+`test_native_engine` CTest **17/17** no-OCCT + iOS-sim native-vs-OCCT parity `native_boolean_parity.mm` **25/25** (box fuse rel 1.27e-16 / cut 2.96e-16 / common 2.22e-16, contained fuse 0.00e+00 / common 2.22e-16 EXACT + watertight; self-verify rejects native∩native disjoint; curved cyl-box / near-coincident / disjoint OCCT-fallthrough rel 0.00e+00 no interception). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221); archived to `openspec/specs/native-booleans`. STILL OCCT (research-grade, not faked): curved-face booleans (surface-surface intersection), near-tangent/coincident, general/concave-general, foreign operands, shape healing — longest-lived OCCT dependency for curved/general. |
| 6 | `add-native-fillets-offsets` | native-blends | ◐ **tractable-planar slice done at verification bar; curved/concave/variable/fillet_face OCCT-fallthrough** — native `cc_chamfer_edges` (convex planar-planar edge) / `cc_offset_face` (planar face) / `cc_shell` (box-like solid) EXACT + `cc_fillet_edges` (CONSTANT radius, convex planar dihedral — rolling-ball cylinder, deflection-bounded) via `src/native/blend/`, guarded by a MANDATORY `blendResultVerified` self-verify (watertight + sane volume sign) that discards + falls through to OCCT. Host `test_native_blend`+`test_native_engine` CTest **18/18** no-OCCT + iOS-sim native-vs-OCCT parity `native_blend_parity.mm` **16/16** (chamfer rel 2.29e-16 / offset 4.55e-16 / shell 4.02e-16 EXACT + watertight; constant fillet deflection-bounded 8.96e-05; curved-rim fillet forwarded to OCCT 0.00e+00; self-verify rejects oversized shell → id 0). No regressions (`test_native_tessellate` green, `run-sim-suite.sh` 221/221); archived to `openspec/specs/native-blends`. Native offset UNBLOCKS the planar slice of #4b-E wrap-emboss. STILL OCCT (not faked): curved-face, concave edges, variable-radius, `cc_fillet_face`, multi-edge interference, non-convex/oversized shell. |
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

# Phase 4 — Native Rewrite Sub-Roadmap (drop OCCT)

The endgame: replace the OCCT adapter with **native C++20**, one capability at a
time, until OCCT can be unlinked entirely. This sub-roadmap sequences that work
and fixes the rules every capability migration follows.

Committed goal: **full drop-OCCT**, including native booleans. Honest caveat:
robust B-rep booleans and shape healing are research-grade — they will land
*progressively hardened and verified against OCCT*, not production-robust on day
one. Difficulty is flagged per capability below.

## Method (locked)

- **Clean-room from references.** Implement from math/first-principles, public
  algorithm references (e.g. *The NURBS Book* — de Boor, de Casteljau, basis
  functions; standard computational-geometry literature), and the `cc_*`
  contract. **OCCT source is a reference *oracle*** — consulted to confirm an
  algorithm matches and to compare numerics/perf — not copied verbatim. License
  is not a constraint on this project; the driver is modern, maintainable,
  fast C++20.
- **Balance maintainability · readability · performance.** Prefer clear,
  well-named, `constexpr`/`span`/`concepts`-friendly C++20 with documented
  algorithms and low cognitive complexity (systems band ≤ 25–35 for irreducible
  geometry, flagged). Optimise with data (benchmarks), not guesswork.

## Verification model (every capability)

Native code has **no OCCT dependency**, which gives two independent test gates:

1. **Host unit tests** — native code compiles and unit-tests with `clang++`
   `-std=c++20` (no OCCT, no simulator): analytic/known-value assertions
   (a known Bézier point, a transform identity, an exact volume).
2. **Simulator native-vs-OCCT parity** — on the iOS simulator (OCCT linked), the
   native result is compared against the **OCCT oracle** at sampled inputs within
   a tight tolerance (fp64 for exact modelling). OCCT reference source lives at
   `/Users/leonardoaraujo/work/OCCT/src` (V8.0).

A capability is **migrated** only when: host unit tests pass, native-vs-OCCT
parity passes on the simulator, **and every existing suite stays green**
(`run-sim-suite.sh` 221/221, host CTest, GPU/Phase-3 suites). Then the engine's
active implementation for that capability is switched native, OCCT kept as a
fallback/oracle until the whole phase completes, and OCCT is unlinked only at
the final `drop-occt` step.

## Architecture

Native code lands under `src/native/` (host-buildable, OCCT-free). A
`NativeEngine : IEngine` implements each capability as it matures and **falls
through to the OCCT engine** for capabilities not yet native (the coexistence the
engine-adapter was built for) — so the app keeps working throughout, behind the
unchanged `cc_*` facade.

```
cc_* facade → active engine
                ├─ NativeEngine (C++20)  ── implements migrated capabilities
                │        └─ falls through to ↓ for the rest
                └─ OcctEngine            ── oracle + fallback (unlinked at the end)
```

## Capability sequence

Dependency order. Each row is one OpenSpec change (`add-native-*`).

| # | Change | Capability | Difficulty | Native-vs-OCCT oracle |
|---|---|---|---|---|
| 1 | `add-native-math-geometry` | `native-math` | moderate | `gp_*`, `BSplCLib`/`BSplSLib`/`PLib`/`ElSLib` |
| 2 | `add-native-brep-topology` | `native-topology` | moderate–hard | `TopoDS`, `TopExp`, `BRep_Tool` |
| 3 | `add-native-tessellation` | `native-tessellation` | moderate | `BRepMesh` (+ Phase-2 GPU eval) |
| 4 | `add-native-swept-solids` | `native-construction` | hard | `BRepPrimAPI`, `BRepBuilderAPI`, `BRepOffsetAPI` |
| 5 | `add-native-booleans` | `native-booleans` | **research-grade** | `BRepAlgoAPI` (BOPAlgo) |
| 6 | `add-native-fillets-offsets` | `native-blends` | hard | `BRepFilletAPI`, `BRepOffsetAPI` |
| 7 | `add-native-data-exchange` | `native-exchange` | moderate (external?) | `STEPControl`, `IGESControl` |
| 8 | `drop-occt` | — | — | unlink OCCT; kernel fully native |

Booleans (#5) are the hardest and longest-lived OCCT dependency — sequenced late
and expected to iterate. #7 (STEP/IGES) may stay a thin external dependency
longest; a native exchange is lower priority than the modelling core.

## Status

- ✅ **#1 `native-math`** — done at the verification bar (first capability). Both
  gates green: host analytic unit tests (55 asserts, no OCCT) + native-vs-OCCT
  parity on the iOS sim (24 groups, 0 failed, overall max numeric error
  1.486e-13, well under tolerance); no regressions (host CTest 8/8,
  `run-sim-suite.sh` 221/221). Not yet engine-wired — by design, this capability
  ships the OCCT-free math foundation only. See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-math`.
- ✅ **#2 `native-topology`** — done at the verification bar (second capability).
  Both gates green: host invariant unit tests (`test_native_topology`, 13 cases,
  no OCCT — data model, orientation compose, location, sub-shape sharing,
  geometry attachment, stable ids, deterministic enumeration, explorer/ancestry,
  `BRep_Tool` accessors, repeat-run equality) + native-vs-OCCT parity on the iOS
  sim (3 shapes — box / cylinder / filleted-box — × 5 checks = **15 passed, 0
  failed**; sub-shape counts + `MapShapes` order + edge→faces ancestry +
  orientation flags match the oracle, accessor max error **0.000e+00** at tol
  1.0e-09, surface types match). No regressions (host CTest 9/9,
  `run-sim-suite.sh` 221/221). Header-only under `src/native/topology/`
  (`shape.h`, `explore.h`, `accessors.h`, `native_topology.h`); not engine-wired —
  by design. Deferred: non-manifold/degenerate + seam edges, `CompSolid` /
  `Internal`/`External`, holed-face parity fixture. See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-topology`.
- ✅ **#3 `native-tessellation`** — done at the verification bar (third
  capability). Both gates green: host invariant unit tests
  (`test_native_tessellate`, no OCCT — deflection-bound, on-surface, trimming,
  watertightness, area/volume convergence, determinism) + native-vs-OCCT
  `BRepMesh` property-parity on the iOS sim (4 shapes — box / cylinder / sphere /
  filleted-box — **All 20 checks PASS**; **ALL four closed solids watertight
  `boundaryEdges==0`**; area/volume relMesh ≤ **6.0e-3**, relExact ≤ **1.24e-2**,
  bbox max corner delta ≤ **4.66e-2**, on-surface residual ≤ **5.7e-15**; triangle
  count/topology NOT compared — tessellation is an approximation). No regressions
  (host CTest 10/10, `test_native_tessellate` 13 cases, `run-sim-suite.sh` 221/221).
  Header-only under `src/native/tessellate/` (`mesh.h`, `surface_eval.h`,
  `edge_mesher.h`, `trim.h`, `uv_triangulate.h`, `face_mesher.h`,
  `solid_mesher.h`, `gpu_sample.h`, `native_tessellate.h`); not engine-wired — by
  design. RESOLVED: the curved shared-edge stitch — the mesher is now a two-stage
  pipeline (STAGE 1 `edge_mesher.h` discretizes each unique edge ONCE into a shared
  deflection-based 1D fraction list; STAGE 2 `face_mesher.h` pins both adjacent
  faces' boundaries to that shared discretization, structured-grid for full
  parametric-rectangle faces and ear-clip (`uv_triangulate.h`) for trimmed faces),
  so CURVED shared edges (cylinder cap↔side circle, fillet blend seams) weld
  WATERTIGHT — every closed solid (box/cylinder/sphere/filleted-box) is now
  required to mesh `boundaryEdges==0`; the weaker "bounded-open" pass is gone.
  Deferred: GPU fp32 sampling path (compiled behind `CYBERCAD_HAS_METAL`,
  CPU-verified only in this environment). See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-tessellation`.
- ☐ #4–#8 — planned; proposed (`/opsx:propose`) as each is about to start. Next:
  **#4 `native-construction`** (primitive + swept-solid construction —
  extrude/revolve/loft/sweep — building native topology + geometry; the first
  capability that will likely be engine-wired for a `cc_*` comparison).

Progress is reflected in [ROADMAP.md](ROADMAP.md) Phase 4 and per-change
`tasks.md`; living specs are synced/archived per capability as they pass the
verification gates.

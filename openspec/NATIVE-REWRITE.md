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
- ◐ **#4 `native-construction`** — **CORE done at the verification bar; advanced
  swept solids are a follow-up (`#4b`).** The first **engine-wired** capability. Two
  construction ops are native: `cc_solid_extrude` (closed polygon → prism: bottom/top
  `Plane` caps + one planar quad `Plane` side per profile edge) and `cc_solid_revolve`
  for **LINE-SEGMENT** profiles (per-segment surface of revolution — parallel→`Cylinder`,
  perpendicular→`Plane`, oblique→`Cone`; full 360° closes the shell, partial angle adds
  two `Plane` meridian caps). Built on the #1–#3 foundations under
  `src/native/construct/construct.h` (OCCT-free, host-buildable). Wired through
  `NativeEngine : IEngine` (`src/engine/native/`), which serves these ops + native
  tessellate / mass / bbox / subshape on its own native bodies and **falls through to
  OCCT** (or the stub on host) for everything else, behind an ADDITIVE facade toggle
  `cc_set_engine(int)` / `cc_active_engine()` (**default stays OCCT** — existing suites
  unchanged). Both gates green: host `test_native_construct` + `test_native_engine`
  (no OCCT — box exact vol/area/6-faces/centroid/bbox/watertight, triangle prism
  watertight vol=area×depth, L-prism, full-turn tube 9π, quarter-turn tube 9π/4, cone
  4π; CTest **12/12**) + native-vs-OCCT parity on the iOS sim through the facade
  (`native_construct_parity.mm`, **17/17** `[NCONS]`): planar prisms EXACT (vol/area/
  centroid rel 0.00e+00, identical face tiling), curved revolves within a deflection
  bound (vol rel ≤ 2.36e-2, area rel ≤ 1.24e-2, bbox maxCornerΔ ≤ 4.37e-2, all
  watertight), plus a fall-through boolean (native→OCCT, fuse vol=14) proving no native
  interception. No regressions (host CTest 12/12, `run-sim-suite.sh` 221/221 re-verified
  against a freshly rebuilt SIMULATORARM64 slice). Documented representational difference
  (not a geometric mismatch): the native builder emits per-face edges / per-patch vertices
  (edge/vertex SHARING deferred) and tiles a full-turn surface of revolution into < π
  angular patches (periodic-face construction deferred), so native V/E and the full-turn
  face count differ from OCCT's shared/periodic representation while the SOLID is
  geometrically identical. See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md);
  living spec archived to `openspec/specs/native-construction`.
  - **◐ `#4b` follow-up — Tier A + Tier B (2-section ruled loft) DONE at the
    verification bar; the rest still OCCT-fallthrough (not faked).** NOW NATIVE
    (host-verified, engine-wired behind the same `cc_set_engine(1)` toggle):
    `cc_solid_extrude_holes` (outer polygon +
    CIRCULAR through-holes kept as TRUE circle edges + cylinder walls),
    `cc_solid_extrude_polyholes` (outer + POLYGON holes), `cc_solid_extrude_profile` /
    `_profile_polyholes` (TYPED outer profile — kind 0 line / 1 arc / 2 full circle —
    with circular + polygon holes; a whole-circle profile keeps one Circle cap edge +
    one Cylinder wall), and `cc_solid_revolve_profile` (TYPED profile revolve: line →
    Plane/Cylinder/Cone, an arc whose circle centre lies ON the axis → Sphere band;
    full 2π closes, partial adds two planar meridian caps). **Tier B:**
    `cc_solid_loft` / `cc_solid_loft_wires` for TWO sections with EQUAL vertex counts
    (≥3) that are both PLANAR — corresponding edge pairs span one BILINEAR (degree-1
    Bézier) ruled side face + two planar caps → watertight solid (mirrors ruled
    `BRepOffsetAPI_ThruSections`); built in `src/native/construct/loft.h` (OCCT-FREE,
    host-buildable, all functions cognitive complexity ≤ 7). Built in
    `src/native/construct/profile.h` (OCCT-FREE, host-buildable) + a robustified
    multi-hole cap triangulator (visibility-checked, rightmost-first hole bridging in
    `src/native/tessellate/uv_triangulate.h`, replacing the single-hole-only nearest-
    vertex heuristic). Gate 1 green: host `test_native_profile` (12 cases — circular /
    polygon / multi-hole / combined holes watertight with exact-or-convergent volume;
    full-circle extrude → cylinder; on-axis arc revolve → sphere 36π; partial-turn
    revolve; typed line/arc extrude) + `test_native_engine` (5 new facade cases through
    `cc_solid_extrude_holes/_polyholes/_profile` + `cc_solid_revolve_profile`); host
    CTest 13/13, existing suites incl. `test_native_tessellate` unchanged. STILL
    OCCT-fallthrough (the native builder returns a NULL Shape → `NativeEngine` forwards
    to OCCT, never fakes): **kind-3 SPLINE profile edges** (extrude AND revolve),
    **arc-revolve whose circle centre is OFF the axis** (a TORUS surface of revolution
    — no native Torus surface yet), a loft with MISMATCHED section counts / a
    NON-PLANAR section / 3+ sections / guided or rail loft (Tier C), and
    twisted-with-real-twist / guided-sweep / loft-along-rail / threads (Tiers C–E).
    (`cc_solid_sweep` itself is NOW native for straight + smooth-planar spines — see the
    Tier-C entry below.) A
    kind-1 ARC extrude edge is a TRUE `Circle` cap edge + a `Cylinder` side wall — one
    bounded, non-periodic patch per ≤180° span (split threshold is π for the EXTRUDE
    wall, NOT the revolve's 120°: an extrude wall is never periodic, so a semicircle is
    ONE patch matching OCCT's single cylindrical face) — not a chord polyline. Gate 2
    (sim OCCT parity) GREEN: `native_construct_profiles_parity.mm` through the cc_*
    facade, **22 passed / 0 failed** — the 5 native ops (holed / polyhole / typed
    line+arc / line-revolve tube / on-axis-arc-revolve sphere) match the OCCT oracle
    (planar EXACT; curved deflection-bounded vol rel ≤ 5.0e-2, all watertight; native
    FACE count a k≥1 integer multiple of OCCT's), and the 2 deferred sub-cases (kind-3
    spline extrude, off-axis-circle → torus revolve) transparently delegate to OCCT
    (vol rel 0.00e+00). Note: `splineXYCount` on the kind-3 side-channel is the number
    of DOUBLES (2× the point count), matching the OCCT `addSplineEdge` bounds guard —
    now documented in `cc_kernel.h`. **Tier B (2-section ruled loft):** Gate 1 (host,
    no OCCT) GREEN — `test_native_loft` (9 cases: prism / frustum / twisted rotated
    square / two-3D-wire triangle prism / tilted planar section watertight, + deferred
    mismatched-count / non-planar / degenerate / bad-input all NULL) + 2 new
    `test_native_engine` facade cases (native square-frustum loft vol 56 @ 6 faces;
    native `loft_wires` triangle prism vol 18); CTest **14/14**, `loft.h` cognitive
    complexity ≤ 7 across all functions. Gate 2 (sim OCCT parity) GREEN —
    `tests/sim/native_loft_parity.mm` + `scripts/run-sim-native-loft.sh` through the
    `cc_*` facade under `cc_set_engine(0/1)` (OCCT default restored in teardown):
    **`[NLOFT]` 17 passed / 0 failed** — square frustum (vol rel 2.54e-16) / hex
    prism (rel 0.00e+00) / two-wire triangle prism (rel 0.00e+00) EXACT,
    rotated-square TWIST deflection-bounded (vol rel 5.33e-3, watertight, tol 5e-2),
    native F = OCCT F (n=1×o) on all four, plus the mismatched-count deferred case
    delegating to OCCT (vol rel 0.00e+00, native active — fall-through proof). Runs
    on the sim (OCCT linked); on `run-sim-suite.sh`'s SKIP list (own `main()`), so
    the 221-assertion suite count is unchanged. No regressions (`test_native_tessellate`
    green, `run-sim-suite.sh` 221/221).
  - **◐ `#4b` Tier C (sweep / pipe-shell) — `cc_solid_sweep` DONE at the verification
    bar; the guided/rail/real-twist pipe-shell cases stay OCCT-fallthrough (not faked).**
    NOW NATIVE (engine-wired behind the same `cc_set_engine(1)` toggle):
    `cc_solid_sweep` for (a) a **STRAIGHT** spine (exact directional prism, vol =
    profileArea × |d|) and (b) a **SMOOTH CURVED but PLANAR** spine. The crux — and the
    fix that made Gate 2 pass — is the FRAME LAW: the OCCT oracle
    `BRepOffsetAPI_MakePipe` uses `GeomFill_CorrectedFrenet`, which for a **planar** spine
    collapses to a **constant rotation** (`GeomFill_CorrectedFrenet.cxx`, `isPlanar` →
    `Law_Constant`), i.e. it TRANSLATES the section with a FIXED orientation, NOT a
    perpendicular-tracking sweep. So `src/native/construct/sweep.h` holds the start frame
    CONSTANT across stations (`detail::constantFrames`), builds one bilinear ruled band
    per (profile edge × spine segment) with shared per-station rings, and caps both ends
    in the fixed section plane → a watertight solid. (An earlier RMF / parallel-transport
    revision kept the section perpendicular and produced the Pappus arc-length volume —
    geometrically "nicer" but a REAL mismatch vs the oracle, correctly rejected by the
    parity gate; we match the oracle.) `cc_twisted_sweep` is native ONLY when it reduces
    to the plain sweep (twist ≈ 0, scale ≈ 1 → forwards to `build_sweep`). Gate 1 green:
    host `test_native_sweep` (11 cases — straight prism / collinear-collapse / arbitrary-
    3D-direction / pentagon / zero-twist prism / smooth-planar-arc watertight + constant-
    frame volume `A·|Δspine·n̂|` / constant-frame invariance / degenerate + real-twist +
    tight-curvature deferrals) + `test_native_engine` (`native_sweep_smooth_arc` vol
    82.57 = the oracle value, `native_sweep_tight_and_twisted_defer`); host CTest 15/15,
    existing suites unchanged. STILL OCCT-fallthrough (native builder returns NULL →
    `NativeEngine` forwards, never fakes): a **NON-PLANAR** curved spine (OCCT's genuine
    non-constant corrected-Frenet law), a **TIGHT-CURVATURE / self-intersecting** spine
    (guarded by `spineTooSharp`), a **real-twist/scale** `cc_twisted_sweep` (OCCT
    `ThruSections`), and the pipe-shell/guide cases **`cc_guided_sweep`** /
    **`cc_loft_along_rail`** (engine-glue fall-through). Gate 2 (sim OCCT parity) GREEN:
    `tests/sim/native_sweep_parity.mm` + `scripts/run-sim-native-sweep.sh` through the
    `cc_*` facade under `cc_set_engine(0/1)` — **`[NSWEEP]` 11 passed / 0 failed**: the
    straight sweep EXACT (vol rel 7e-16) and — because native and OCCT now share the same
    constant-frame law and polyline — the **smooth-arc sweep EXACT too** (vol o=330.299
    n=330.299 **rel 1.7e-16**, bbox maxCornerΔ 1.0e-7, native F = OCCT F = 98, watertight),
    plus the three deferred cases (real-twist / guided / loft-rail) delegating to OCCT
    (vol rel 0.00e+00, native active — fall-through proof). On `run-sim-suite.sh`'s SKIP
    list (own `main()`), 221-assertion count unchanged. No regressions (host CTest 15/15,
    `run-sim-suite.sh` 221/221). Living change: `openspec/changes/add-native-sweep`.
  - **◐ `#4b` Tier D (threads / tapered shank) — `cc_tapered_shank` DONE at the Gate-1
    bar (NATIVE); `cc_helical_thread` / `cc_tapered_thread` ATTEMPTED but HONESTLY
    OCCT-fallthrough (not faked).** NOW NATIVE (engine-wired behind the same
    `cc_set_engine(1)` toggle): **`cc_tapered_shank`** — a pointed-shank silhouette
    (cone tip → full-radius cylinder → head disk) revolved 360° about Z by REUSING the
    already-parity-verified native revolve (`build_revolution`, construct.h). The tip is
    a TRUE on-axis apex (the revolve collapses its angular copies to one shared vertex, so
    no sliver breaks the weld — a non-zero tip radius does NOT weld, verified), giving a
    ROBUSTLY watertight solid at every deflection {0.05…0.005} with volume
    `⅓π r²·taperHeight + π r²·fullHeight` within the deflection bound (r5/fh20/th10 →
    exact 1832.6, meshed 1828–1832), matching `BRepPrimAPI_MakeRevol`. Built in
    `src/native/construct/thread.h` (OCCT-FREE, host-buildable, all four functions
    cognitive complexity 🟢 Excellent ≤ 5). **ATTEMPTED, deferred (honest):** the
    **helical / tapered thread** ops build the full radial-V helical tiling — a
    V/triangular section transported RADIALLY along the pitch-line helix via the AXIS
    auxiliary-spine law (radial = (cosθ,sinθ,0), axial = +Z, so the V does NOT
    Frenet-rotate — mirroring `BRepOffsetAPI_MakePipeShell::SetMode(axisWire,true)`),
    tiled into three bilinear ruled bands per span (loft.h `ruledSideFace`) with shared
    per-station rings + two planar V end caps, capped at `samplesPerTurn ∈ [8,24]` and
    GUARDED against self-intersection (pitch-line radius must clear the axis at both ends;
    `2·halfBase ≤ pitch`; degenerate params → NULL). The candidate solid has the CORRECT
    volume + V geometry (measured helical major5/p2/t4/d1 → vol ≈ 450), BUT its per-turn
    ruled-band ↔ end-cap seams do NOT weld ROBUSTLY watertight on the current two-stage
    tessellator — watertight at some deflections, open at others (a deflection-dependent
    seam sliver: the straight V-section side edges are not guaranteed identical
    subdivision across the two bands sharing each edge). Rather than ship a leaky solid,
    the engine applies a `robustlyWatertight` SELF-VERIFY (closed at every deflection in a
    ladder) and — since NO tested thread passes it on the current mesher — FALLS THROUGH
    to the OCCT `MakePipeShell` oracle (labelled, verified, never faked; the native
    builder never emits the round-profile fallback). If the mesher is later strengthened
    to share ruled-band side-edge discretization (as the loft/sweep STRAIGHT bands already
    weld watertight), the threads light up native automatically with no engine change.
    Gate 1 (host, no OCCT) GREEN — `test_native_thread` (8 cases: shank watertight+volume,
    shank ppm³ scaling, shank degenerate NULL, helical/tapered candidate V-tiling +
    positive volume, degenerate-params NULL, pitch-radius-below-axis NULL, tapered-tip
    -below-axis NULL) + 3 new `test_native_engine` facade cases (native `cc_tapered_shank`
    watertight vol 1832.6; degenerate shank → fall-through 0; well-formed threads → engine
    self-verify defers → fall-through 0); host CTest **16/16** (was 15), all existing
    suites unchanged (`test_native_construct/profile/loft/sweep/tessellate` green). Gate 2
    (sim OCCT parity) GREEN — `tests/sim/native_thread_parity.mm` +
    `scripts/run-sim-native-thread.sh` through the `cc_*` facade (booted sim `2B90AEDB`):
    **`cc_tapered_shank` runs NATIVE** and matches the OCCT `BRepPrimAPI_MakeRevol` oracle —
    r5/fh20/th10 `[native]` vol o=1837.94 n=1830.27 **rel 4.17e-03**, area rel 3.64e-03,
    centroidΔ 3.85e-02 (tol v=5e-02 c=1e-01), bbox maxCornerΔ 1.00e-07, subshapes F 4→9 /
    E 5→30 / V 3→30 (angular tiling), tessellate watertight tris=144 meshVolRel 3.81e-03.
    The **two thread ops are verified OCCT fall-through** (`[fallback]`, `cc_active_engine()==1`,
    native self-verify defers → delegated to OCCT, so native == OCCT oracle EXACTLY):
    helical mr5/p2/t4/d1 vol o=n=70.2841 rel 0.00e+00; tapered top6/tip4/p2/t4 vol
    o=n=70.2677 rel 0.00e+00 — a fall-through proof, no native interception. On
    `run-sim-suite.sh`'s SKIP list (own `main()`, `.mm`), so the 221-assertion OCCT-only
    count is unchanged; `run-sim-suite.sh` **221/221** re-verified. Living change (archived):
    `openspec/changes/add-native-threads` → `openspec/specs/native-construction`.
- ☐ **#5 `native-booleans` — NEXT (research-grade).** The hardest and longest-lived
  OCCT dependency. Native robust B-rep booleans require surface-surface intersection
  (the intersection curves between arbitrary analytic + NURBS surfaces), robust
  section-edge classification (which pieces of each shell survive under
  fuse/cut/common), and shape healing (sewing/tolerance reconciliation of the result).
  This is fundamentally harder than #1–#4: those build clean topology from parameters,
  whereas booleans must reason about the intersection of two arbitrary existing solids
  with fp64 robustness at near-tangent / coincident configurations — the classic BOPAlgo
  wall. It will land **progressively hardened and verified against OCCT** (`BRepAlgoAPI`
  / BOPAlgo as oracle), starting from analytic-surface cases (box∩box, cylinder∩box)
  and widening, and is NOT expected to be production-robust on day one. Proposed via
  `/opsx:propose` when it begins.
- ☐ #6–#8 — planned; proposed as each is about to start (blends → exchange →
  drop-occt).

Progress is reflected in [ROADMAP.md](ROADMAP.md) Phase 4 and per-change
`tasks.md`; living specs are synced/archived per capability as they pass the
verification gates.

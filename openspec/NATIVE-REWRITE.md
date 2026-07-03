# Phase 4 — Native Rewrite Sub-Roadmap (drop OCCT)

The endgame: replace the OCCT adapter with **native C++20**, one capability at a
time, until OCCT can be unlinked entirely. This sub-roadmap sequences that work
and fixes the rules every capability migration follows.

Committed goal: **full drop-OCCT**, including native booleans. Honest caveat:
robust B-rep booleans and shape healing are research-grade — they will land
*progressively hardened and verified against OCCT*, not production-robust on day
one. Difficulty is flagged per capability below.

**Where this stands now (honest ceiling).** The tractable native slice of every
capability #1–#7 is DONE at the verification bar — including the native STEP EXPORT
slice (#7). Phase 4 is therefore **COMPLETE AT ITS ACHIEVABLE NATIVE CEILING, not
fully drop-OCCT**: #8 `drop-occt` is BLOCKED because two hard dependencies remain
research-grade multi-year efforts — (1) a general robust curved boolean / blend
kernel (arbitrary surface-surface intersection + shape healing) and (2) native
STEP/IGES IMPORT (the #7 slice delivered EXPORT only). Until both exist, OCCT stays
linked.

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
and expected to iterate. The PLANAR-polyhedron slice is now native (BSP-CSG,
self-verified EXACT vs OCCT on axis-aligned boxes); curved / general booleans remain
OCCT-backed. #7 (STEP/IGES) may stay a thin external dependency
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
  - **● `#4b` Tier D (threads / tapered shank) — ALL THREE NATIVE at the Gate-1 bar:
    `cc_tapered_shank`, `cc_helical_thread`, `cc_tapered_thread`.** Engine-wired behind
    the same `cc_set_engine(1)` toggle. **`cc_tapered_shank`** — a pointed-shank silhouette
    (cone tip → full-radius cylinder → head disk) revolved 360° about Z by REUSING the
    already-parity-verified native revolve (`build_revolution`, construct.h). The tip is
    a TRUE on-axis apex (the revolve collapses its angular copies to one shared vertex, so
    no sliver breaks the weld — a non-zero tip radius does NOT weld, verified), giving a
    ROBUSTLY watertight solid at every deflection {0.05…0.005} with volume
    `⅓π r²·taperHeight + π r²·fullHeight` within the deflection bound (r5/fh20/th10 →
    exact 1832.6, meshed 1828–1832), matching `BRepPrimAPI_MakeRevol`. Built in
    `src/native/construct/thread.h` (OCCT-FREE, host-buildable, all four functions
    cognitive complexity 🟢 Excellent ≤ 5). **`cc_helical_thread` / `cc_tapered_thread`
    (NOW NATIVE):** the full radial-V helical tiling — a V/triangular section transported
    RADIALLY along the pitch-line helix via the AXIS auxiliary-spine law
    (radial = (cosθ,sinθ,0), axial = +Z, so the V does NOT Frenet-rotate — mirroring
    `BRepOffsetAPI_MakePipeShell::SetMode(axisWire,true)`), tiled into three bilinear ruled
    bands per span (loft.h `ruledSideFace`) with shared per-station rings + two planar V
    end caps, capped at `samplesPerTurn ∈ [8,24]` and GUARDED against self-intersection
    (pitch-line radius must clear the axis at both ends; `2·halfBase ≤ pitch`; degenerate
    params → NULL). **The per-turn seam WELD was the last blocker and is now fixed at the
    mesher level (topology-preferred, geometry untouched):** the ruled-band ↔ band and
    band ↔ V-cap seams are STRAIGHT edges shared by two faces that each evaluated the seam
    through their OWN bilinear surface, so the two boundary points agreed only to ~1 ULP;
    when a shared coordinate landed on a spatial-weld cell boundary (coord·⅟tol = k+0.5)
    the ULP twins rounded to opposite cells and the weld left the seam OPEN at isolated
    deflections. The mesher now emits, for every straight boundary edge, CANONICAL seam
    points — interpolated at the shared sample indices `i/n` between the edge's two
    BOUNDING VERTICES in a fixed lexicographic order (`edge_mesher.h` `CanonicalEndpoints`
    / `face_mesher.h` `recordEdgeAnchors`), which is BIT-IDENTICAL for the two coincident
    edges regardless of build order — and SNAPS each seam-lying vertex to its canonical
    point (`BoundaryAnchors`). The two faces therefore place the identical 3D point and the
    conservative single-cell weld fuses them, with NO widening of the merge radius (which
    would over-collapse a fine curvature grid). This is exactly the "one shared 1D
    discretization pinned on both faces" contract, completed at the 3D-point level; the V
    volume/geometry are unchanged. Result: helical (major6…20 / pitch2…4 / turns1…5 /
    depth0.5…1.5 / spt8…24) and tapered (top5…8 / tip3…4 / …) are ROBUSTLY watertight
    (`boundaryEdges==0`) at EVERY deflection in the `robustlyWatertight` ladder across the
    full swept parameter space (432/432 helical + 96/96 tapered candidates → native), so
    the engine keeps them NATIVE. **HONESTY (unchanged guard):** a FINE-PITCH /
    self-intersecting thread (turns fold through each other, e.g. major2/pitch0.2/depth3)
    still fails `robustlyWatertight` — a self-overlapping mesh is non-manifold no matter
    how the vertices weld — so it still FALLS THROUGH to the OCCT `MakePipeShell` oracle
    (labelled, verified, never faked; the native builder never emits the round-profile
    fallback). Gate 1 (host, no OCCT) GREEN — `test_native_thread` (9 cases: shank watertight+volume,
    shank ppm³ scaling, shank degenerate NULL, `helical_thread_is_watertight_across_ladder`
    + `tapered_thread_is_watertight_across_ladder` — a HARD requirement asserting
    `boundaryEdges==0` at EVERY deflection in the ladder {0.1,0.05,0.02,0.01} with the right
    V-tiling face count, positive volume sign and turn count, degenerate-params NULL,
    pitch-radius-below-axis NULL, tapered-tip-below-axis NULL — plus the
    `fine_pitch_self_intersecting_thread_not_supported` guard) + `test_native_engine`
    facade cases (native `cc_tapered_shank` watertight vol 1832.6; degenerate shank →
    fall-through 0; **`native_thread_runs_native_watertight`** — the well-formed helical +
    tapered thread now run NATIVE through the facade with valid watertight mass-properties;
    `native_fine_pitch_thread_falls_through_to_default` — the self-intersecting thread still
    defers to the fallback); host CTest 18/18, all native suites green
    (`test_native_construct/profile/loft/sweep/tessellate/boolean/blend/topology/thread`).
    The seam-weld fix is exercised directly by `test_native_thread` (both helical + tapered
    candidates are asserted watertight across the deflection ladder) and by a broad
    parameter sweep (432 helical + 96 tapered configs, all `boundaryEdges==0`). Gate 2
    (sim OCCT parity) — `tests/sim/native_thread_parity.mm` +
    `scripts/run-sim-native-thread.sh` through the `cc_*` facade: **`cc_tapered_shank` runs
    NATIVE** and matches the OCCT `BRepPrimAPI_MakeRevol` oracle — r5/fh20/th10 `[native]`
    vol o=1837.94 n=1830.27 **rel 4.17e-03**, area rel 3.64e-03, centroidΔ 3.85e-02
    (tol v=5e-02 c=1e-01), bbox maxCornerΔ 1.00e-07, subshapes F 4→9 / E 5→30 / V 3→30
    (angular tiling), tessellate watertight tris=144 meshVolRel 3.81e-03. With the seam
    weld fixed the **well-formed helical / tapered thread ops now run NATIVE** (radial-V
    tiling, robustlyWatertight passes) and are cross-checked vs the OCCT `MakePipeShell`
    oracle to the deflection-bounded volume/area/bbox tolerance: `cc_helical_thread`
    mr5/p2/t4/d1 `[native]` vol o=70.2841 n=68.3767 **rel 2.71e-02** / area rel 1.73e-02 /
    centroidΔ 4.83e-05 / bbox maxCornerΔ 1.44e-03 / F 5→194 E 9→774 V 6→195 / tessellate
    watertight tris=1286 meshVolRel 1.40e-03, and `cc_tapered_thread` top6/tip4/p2/t4
    `[native]` vol o=70.2677 n=68.3767 **rel 2.69e-02** / watertight tris=1286 (the ~2.7%
    native-vs-OCCT gap is chord-vs-arc at spt=16, tightening to ~1.3% at spt=24, while the
    native mesh matches its OWN B-rep volume to meshVolRel ≤ 1.40e-3). A FINE-PITCH /
    self-intersecting thread (mr5/p0.3/t8/d1) still verifies as OCCT fall-through
    (`[fallback]`, `cc_active_engine()==1`, vol rel 0.00e+00). Both gates are GREEN and the
    engine enforces the watertight bar at runtime via `robustlyWatertight`. Living change
    (archived): `openspec/changes/add-native-threads` → `openspec/specs/native-construction`.
  - **◐ `#4b` geometry-completion batch (Tier 1 + Tier 2#4) — spline extrude, off-axis-arc
    torus revolve, N-section ruled loft, and a NON-PLANAR (RMF) sweep DONE at the
    verification bar; twist/scale + guided/rail + truly-self-intersecting thread stay
    OCCT-fallthrough (SSI/Tier-4, not faked).** Change `add-native-geometry-completion`,
    engine-wired behind the same `cc_set_engine(1)` toggle. **Tier 1 NATIVE:** (A) a **kind-3
    SPLINE** outer profile edge extrude (a native B-spline edge via native-math NURBS + a
    `BSpline` swept side wall + planar caps → watertight) and an **OFF-AXIS-ARC** revolve → a
    **TORUS** surface of revolution (native `Torus` added in `src/native/math/torus.h`;
    emitted as EXACT rational-quadratic B-spline patches so no new tessellator surface kind
    was needed) — both in `src/native/construct/residuals.h`. **Tier 2#4 NATIVE:** an
    **N-section (3+)** ruled loft chain (`src/native/construct/loft.h`, generalizing the
    Tier-B 2-section builder — shared interior rings, first/last caps) and a **NON-PLANAR
    smooth spine** sweep via the double-reflection RMF (Wang et al. 2008,
    `src/native/construct/sweep.h`; the RMF collapses to the constant frame on a planar spine,
    preserving Tier-C parity). Gate 1 (host, no OCCT) GREEN — host build clean, CTest **22/22**
    (incl. `test_native_residuals`, `test_native_loft`, `test_native_sweep`, `test_native_thread`,
    `test_native_tessellate`, `test_native_step`, `test_native_engine`). Gate 2 (sim OCCT
    parity) GREEN — `tests/sim/native_geomcompletion_parity.mm` +
    `scripts/run-sim-native-geomcompletion.sh` through the `cc_*` facade under
    `cc_set_engine(0/1)`: **spline extrude** vol o=45.6 n=45.5547 **rel 9.92e-04** (watertight,
    132 tris, faces 4→4); **torus revolve** vol o=98.696 n=96.0542 **rel 2.68e-02** (watertight,
    1620 tris, faces 2→6); **ruled frustum** + **straight-rail** N-section loft vol rel
    **1.43e-14 / 5.58e-15 EXACT** (watertight, 432 tris, faces 6→6); **smooth-arc (RMF) sweep**
    vol o=330.299 n=330.299 **rel 3.44e-16 EXACT** (watertight, 196 tris, faces 98→98). **STILL
    OCCT-fallthrough / DECLINE (not faked):** a **self-crossing spline** profile and a **spindle
    torus** (off-axis arc crossing the axis — self-intersecting SoR) DECLINE on BOTH engines
    (unbuildable SSI, Tier 4, occtId=0 natId=0); a **mismatched-count loft** → OCCT
    `ThruSections` (vol 202.185), a **hard curved rail** → OCCT `MakePipeShell` (258.596), a
    **self-intersecting sweep** → OCCT `MakePipe` (17.9515), a **real-twist `cc_twisted_sweep`**
    → OCCT `ThruSections` (320), and a **self-intersecting thread** → OCCT `MakePipeShell`
    (1446.76) all delegate with native active=1 (rel 0.00e+00, no interception). **The
    accumulating-twist/scale sweep, the guided/rail cases, and the thread self-intersection
    resolver did NOT extend the native set beyond what self-verifies watertight + oracle-correct
    — those remaining fall-throughs now specifically need SSI / Tier-4 (surface-surface
    intersection + trimming).** No regressions (`run-sim-suite.sh` 221/221, own-`main()` parity
    harness on the SKIP list). See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living
    change `openspec/changes/add-native-geometry-completion` → archived to
    `openspec/specs/native-construction`.
- ◐ **#5 `native-booleans` — PLANAR-polyhedron slice DONE at the verification bar;
  curved / general still OCCT-fallthrough (not faked).** `cc_boolean` (fuse / cut /
  common) is NATIVE for **PLANAR-faced solids** (polyhedra — axis-aligned boxes, prisms)
  via a **BSP-tree CSG** (Naylor-Amanatides-Thibault 1990) over the solids' planar
  polygons — which IS face-face intersection + fragment split + inside/outside
  classification expressed as recursive plane-clip / invert, and handles
  coplanar-coincident faces (two boxes sharing a wall) robustly, with a B-rep-level
  T-junction repair + triangulation (`assemble.h`) closing the coplanar seams. Guarded by
  a MANDATORY self-verify (`robustlyWatertight` + set-algebra volume `Vr ≈ Va±Vb−Vab`) that
  DISCARDS any candidate that is not a valid watertight solid with the correct volume →
  falls through to OCCT. Built OCCT-free under `src/native/boolean/` (`polygon.h`, `bsp.h`,
  `assemble.h`, `native_boolean.h`, entry `boolean_solid(a, b, op)`), engine-wired behind
  the same `cc_set_engine(1)` toggle (default stays OCCT). Both gates green: host
  `test_native_boolean` + `test_native_engine` (no OCCT — box fuse/cut/common watertight
  EXACT set-algebra volume, prism/simple-concave, self-verify rejecting an open/wrong-volume
  candidate, curved/coincident/foreign fall-through → NULL; CTest **17/17**) + native-vs-OCCT
  parity on the iOS sim through the facade (`native_boolean_parity.mm`, **25/25**): box
  overlap fuse (rel 1.27e-16) / cut (2.96e-16) / common (2.22e-16), contained fuse
  (0.00e+00) / common (2.22e-16) all EXACT + watertight, the self-verify correctly rejecting
  a native∩native DISJOINT out-of-domain result, plus curved (cyl-box fuse rel 0.00e+00),
  near-coincident (rel 0.00e+00) and disjoint (rel 0.00e+00) OCCT-fallthrough — all
  delegated, no native interception. No regressions (host CTest 17/17,
  `run-sim-suite.sh` 221/221 — only change is `native_boolean_parity.mm` on the SKIP list).
  STILL OCCT-fallthrough (native builder returns NULL / self-verify discards → forwarded,
  never faked): **curved-face booleans** (surface-surface intersection of cylinder / sphere
  / cone / NURBS), **near-tangent / coincident / degenerate** configurations, **disjoint**
  operands, **foreign** (OCCT-built) operands, and **general / concave-general / mixed**
  cases. **Booleans remain the longest-lived OCCT dependency for curved / general** —
  surface-surface intersection, robust near-tangent handling, and full shape healing are
  future work. See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living change
  `openspec/changes/add-native-booleans` → archived to `openspec/specs/native-booleans`.
  - **◐ Curved analytic slice (deferred residual #2) — AXIS-ALIGNED box ⟷ axis-parallel
    cylinder NOW NATIVE at BOTH gates (host + sim parity), ARCHIVED; general curved still
    OCCT-fallthrough (honest).** `cc_boolean` cut / fuse / common is NATIVE when one operand is an
    axis-aligned box and the other a cylinder whose axis ∥ a box axis (and a world axis),
    with the cylinder RADIALLY INSIDE the box cross-section. Here plane-cylinder
    intersection is ANALYTIC — a box face ⟂ the axis cuts the cylinder in a CIRCLE — so
    instead of faceting (which would fail the analytic-volume self-verify) or a
    research-grade surface-surface solver, the builder RECOGNISES the (box, cylinder) pair
    and CONSTRUCTS the closed-form result B-rep directly from TRUE `Cylinder` walls +
    `Circle` rim edges + `Plane` caps (the exact watertight face/edge kinds a native
    cylinder-with-holes prism uses): **cut** → box with a round THROUGH hole
    (`boxVol − πr²·h`), **common** → the cylinder segment clipped to the box axial extent
    (`πr²·overlap`), **fuse** → box + a protruding round BOSS (`boxVol + πr²·protrude`).
    Every curved seam is a SHARED `Circle` rim edge, so the mesher's two-stage
    shared-1D-discretization welds it WATERTIGHT across the deflection ladder (verified
    `boundaryEdges==0` at {0.1,0.05,0.02,0.01,0.005} on all three world axes, off-centre).
    Built OCCT-free in `src/native/boolean/curved.h` (recognisers `recogniseBox` /
    `recogniseCylinder`, world-frame axis-aware primitive builders, dispatcher
    `tryBoxCylinder`), wired into `native_boolean.h::boolean_solid` (curved path tried
    FIRST; the planar BSP-CSG path is unchanged) and guarded by an ANALYTIC-volume
    self-verify in `native_engine.cpp` (`curvedBooleanVerified`: `Vr` must match the
    closed-form `boxVol ± πr²·len` to the curved-mesh deflection bound, else DISCARD →
    OCCT). Gate 1 GREEN (host CTest **18/18**): `test_native_boolean` adds box-cylinder
    cut / common / fuse (watertight, analytic volume within ~0.2% deflection bound on all
    three axes) + honest DECLINE cases (wrong-order cyl−box, radial breach, blind hole,
    cone operand → NULL → OCCT); `test_native_engine` asserts the engine's analytic guard
    engages and errs honestly (never faked) when the facade-built config leaves the family.
    Cognitive complexity: worst `tryBoxCylinder` 12 (🟡), no 🟠/🔴. **Gate 2 (sim
    native-vs-OCCT parity) GREEN** — `tests/sim/native_curved_boolean_parity.mm` +
    `scripts/run-sim-curved-boolean.sh` through the `cc_*` facade: `[NCURVBOOL]` **18 checks
    (6 cases × 3), 0 failed** — 3 NATIVE analytic-intercept (through-hole-cut mass rel
    **3.19e-04** / area rel 2.10e-08 / watertight 216 tris; boss-fuse rel **6.10e-05** / area
    rel 2.00e-05 / watertight 212 tris; common rel **1.30e-03** / area rel 5.84e-04 /
    watertight 196 tris) + 3 OCCT-fallback (blind-hole-cut / oblique-cyl-cut / sphere-box-cut,
    rel 0 forwarded, volume-bound tessellation only). No regressions (host CTest 19/19 incl.
    `test_native_boolean` + `test_native_tessellate`; `run-sim-suite.sh` 221/221). STILL
    OCCT-fallthrough (DECLINE → NULL, never faked): **sphere / cone / NURBS**, **NON-axis-aligned
    cylinders**, **cylinder ⟷ cylinder**, **radially-breaching (∥-face LINE-ruling slots)**,
    **blind holes / non-through cuts / cyl−box**, and **near-tangent / coincident-curved**.
    Living change `openspec/changes/add-native-curved-booleans` **archived** to
    `openspec/specs/native-booleans` (validate --strict green). **General curved B-rep
    booleans (surface-surface intersection, robust near-tangent handling, shape healing)
    remain research-grade OCCT-backed — the longest-lived OCCT dependency.**
- ☐ **`#4b` Tier E — native `cc_wrap_emboss` — DEFERRED (FUTURE WORK, not scheduled
  yet).** This is the *native* (OCCT-free) rewrite of wrap-emboss; it is distinct from
  the Phase-3 `add-robust-wrap-emboss` change, which is ✅ done and OCCT-backed (the
  app-facing behaviour already works). Native wrap-emboss needs three pieces:
  (1) native project-a-2D-pattern-onto-a-surface into the target face UV domain,
  (2) native offset-along-normal by the emboss depth, and (3) a boolean merge of the
  raised/recessed region with the base solid. Step (2) is now **unblocked by #6** — the
  native planar `cc_offset_face` (slide a planar face along its normal + drag the side
  faces, EXACT slab, self-verified vs OCCT) is exactly the planar offset-along-normal a
  PLANAR-face emboss/deboss needs; step (3) is **partially unblocked by #5** (a
  planar-polyhedron emboss/deboss can use the native BSP-CSG fuse/cut, and — with the
  curved analytic slice now archived — an AXIS-ALIGNED-CYLINDER target can use the native
  box-cylinder curved fuse/cut). So a **planar-target wrap-emboss is now reachable
  natively** (native offset from #6 + native planar boolean from #5), the **boolean step
  of an axis-aligned-cylinder-target wrap-emboss is now also native** (curved analytic
  slice of #5), and only a **general curved-surface** wrap-emboss still waits on the
  **general curved native-boolean slice of #5** plus a **curved-surface native offset** (the
  planar slice of #6 does not offset curved faces). Native wrap-emboss remains sequenced
  AFTER #6, as its own OpenSpec change (`/opsx:propose`) — the planar slice can be
  proposed now that #6 landed; the curved slice waits on curved #5 + curved offset. Until
  then `cc_wrap_emboss` stays OCCT-fallthrough (labelled, verified). Robust-watertight
  `cc_helical_thread`/`cc_tapered_thread` are **NO LONGER deferred** — the mesher
  shared-edge weld (edge_mesher `CanonicalEndpoints` / face_mesher `BoundaryAnchors`) is
  DONE, so a well-formed helical / tapered thread now meshes `boundaryEdges==0` at EVERY
  deflection in the `robustlyWatertight` ladder and runs NATIVE (see the Tier D entry
  above; `test_native_thread` asserts the hard multi-deflection watertight ladder and
  `test_native_engine::native_thread_runs_native_watertight` asserts the op runs native
  through the facade). Several other residual #4b natives are now DONE by the
  geometry-completion batch above — **kind-3 SPLINE profile edge extrude, off-axis-arc TORUS
  revolve, N-section (3+) ruled loft, and a NON-PLANAR (RMF) sweep are now NATIVE.** What is
  still OCCT-fallthrough are the cases that genuinely need SSI / Tier-4 (surface-surface
  intersection + trimming): the accumulating-twist/scale `cc_twisted_sweep`, the guided/rail
  cases (`cc_guided_sweep` / `cc_loft_along_rail` / hard-rail loft), a mismatched-count /
  non-planar loft, a truly self-intersecting sweep or thread, a general SPLINE
  surface-of-revolution, and a spindle torus.
- ✅ **#6 `native-blends` — tractable PLANAR slice done at the verification bar (both
  gates green); curved / concave / variable / fillet_face OCCT-fallthrough (honest).** Native
  `cc_chamfer_edges` / `cc_fillet_edges` (constant radius) / `cc_offset_face` /
  `cc_shell` for the tractable planar cases, built OCCT-free under
  `src/native/blend/` (`blend_geom.h`, `chamfer_edges.h`, `fillet_edges.h`,
  `offset_face.h`, `shell.h`, aggregate `native_blend.h`). Each op edits the solid's
  oriented-planar-polygon soup (the boolean's `extractPolygons`) and re-welds a
  watertight solid via the boolean's `assembleSolid` (T-junction repair + triangulate
  + weld), so it meshes by the SAME path a native prism / boolean does; the engine
  then runs a MANDATORY self-verify (`blendResultVerified` — watertight + sane volume
  sign: chamfer/fillet/shell REDUCE volume, offset GROWS for +distance / shrinks for
  −distance) and DISCARDS a bad candidate → OCCT (never a wrong/leaky/faked solid).
  Native: **chamfer** slices the convex corner off with the plane through the two
  setback lines (EXACT vs OCCT for a box corner — 10³ edge d=2 → vol 980);
  **fillet** replaces a convex planar-dihedral edge with the rolling-ball tangent
  cylinder (the Phase-3 dihedral construction — axis ∥ crease, radius r, seated
  tangent to both planes: C = E − r/(1+n1·n2)·(n1+n2), tangent lines Ti = C + r·ni),
  tiled into deflection-bounded facets (vol 991.4, BETWEEN the sharp 1000 and the
  chamfer 980, watertight); **offset_face** slides a planar face along its normal
  dragging the side faces (EXACT slab — +5 → 1500, −4 → 600); **shell** insets the
  kept walls inward by thickness and native-BSP-cuts the cavity (open-top box t=1 →
  wall vol 424). Gate 1 GREEN — host `test_native_blend` (10 cases: chamfer box /
  2-edge exact + degenerate/curved fallthrough; fillet watertight-between +
  curved/degenerate fallthrough; offset grow/shrink exact; shell wall exact +
  oversize fallthrough; concave L-prism edge → NULL while a convex edge of the same
  prism still lands native) + 5 new `test_native_engine` facade cases (native
  chamfer/fillet/offset/shell through `cc_set_engine(1)` + a variable-radius
  deferral + a native `cc_edge_polylines` regression case) host CTest **18/18**. STILL
  OCCT-fallthrough (native builder
  returns NULL / self-verify discards → forwarded or honest error, never faked):
  CURVED-face inputs, CONCAVE edges, variable-radius `cc_fillet_edges_variable`,
  `cc_fillet_face`, an edge shared by ≠2 faces, multi-edge fillet interference,
  non-convex shell, oversized thickness. Blend functions are 🟢 Excellent (≤10)
  except the two op drivers `fillet_edges` (13) / `chamfer_edges` (11) in the
  🟡 Acceptable band (systems-band per-edge loop, flagged). Gate 2 (sim native-vs-OCCT
  parity, `native_blend_parity.mm` vs BRepFilletAPI/BRepOffsetAPI) GREEN — **`[NBLEND]`
  16 passed / 0 failed** through the `cc_*` facade under `cc_set_engine(0/1)`: chamfer
  (vol o=995 n=995 **rel 2.29e-16**) / offset (1500, rel 4.55e-16) / shell (424, rel
  4.02e-16) EXACT + watertight, constant-radius fillet deflection-bounded (o=997.854
  n=997.765 rel 8.96e-05, watertight), the curved-rim fillet forwarded to OCCT
  (`[fallback]` rel 0.00e+00), and the self-verify guard rejecting a thickness-6 shell on
  a 10³ box (id 0, honest error). **Root-cause fix:** the NativeEngine had no native
  `edge_polylines` — a native body's edges were unqueryable (the op refused a native
  body), so `findAxisEdge` in the sim harness resolved edge id 0 and `cc_chamfer_edges`
  / `cc_fillet_edges` always returned 0. `NativeEngine::edge_polylines` now discretizes
  each edge (in `mapShapes(Edge)` 1-based order, matching `subshape_ids` / the blend
  ops' edge lookup) via the shared `EdgeCache`, so native-body edges are pickable exactly
  as OCCT-body edges are. On `run-sim-suite.sh`'s SKIP list (own `main()`); 221/221
  re-verified.
- ◐ **#7 `native-exchange` — native STEP EXPORT slice DONE at BOTH gates (host +
  sim OCCT re-read round-trip); STEP import + IGES stay OCCT (honest end state, out
  of scope).** `cc_step_export` is NATIVE
  (engine-wired behind the same `cc_set_engine(1)` toggle) for a native-built solid
  whose every face surface + edge curve is in the writer's scope: it walks the
  native B-rep (src/native/topology) and emits a valid ISO-10303-21 STEP AP203 file
  in true MILLIMETRES — the HEADER (FILE_DESCRIPTION / FILE_NAME /
  FILE_SCHEMA 'CONFIG_CONTROL_DESIGN') + the Part-42 DATA graph (CARTESIAN_POINT /
  DIRECTION / AXIS2_PLACEMENT_3D, VERTEX_POINT, LINE / CIRCLE /
  B_SPLINE_CURVE_WITH_KNOTS + EDGE_CURVE, ORIENTED_EDGE → EDGE_LOOP,
  FACE_OUTER_BOUND / FACE_BOUND, PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE /
  SPHERICAL_SURFACE / B_SPLINE_SURFACE_WITH_KNOTS, ADVANCED_FACE → CLOSED_SHELL →
  MANIFOLD_SOLID_BREP, wrapped in ADVANCED_BREP_SHAPE_REPRESENTATION + the mm
  SI_UNIT geometric context + PRODUCT / PRODUCT_DEFINITION / APPLICATION_CONTEXT
  boilerplate). Built OCCT-FREE under `src/native/exchange/` (`step_writer.h/.cpp`,
  aggregate `native_exchange.h`, entry `step_export_native(solid, path)`) on the
  #1–#6 topology/math foundation. The native builders emit PER-FACE edges (edge-node
  sharing deferred, NATIVE-REWRITE.md #4), so the writer DEDUPLICATES geometrically —
  coincident vertices collapse to one VERTEX_POINT and the two faces meeting at a
  physical edge share ONE EDGE_CURVE (used forward on one face, reversed on the
  other via ORIENTED_EDGE) — producing a properly-sewn manifold CLOSED_SHELL that
  re-reads as a solid, not a heap of coincident faces. **Native-else-OCCT wiring
  (honest):** `NativeEngine::step_export` runs native for a native body IN SCOPE;
  a native body OUT of scope (an unsupported geometry kind) returns a clean error
  (never a native void handed to OCCT); a NON-native (OCCT-built) body forwards to
  `STEPControl_Writer`. **`cc_step_import` STAYS OCCT** (parsing arbitrary STEP is
  the huge part, out of scope) and **`cc_iges_export/import` STAY OCCT** — that is
  the honest end state (#8 drop-occt stays blocked on import + IGES + curved/general
  booleans). No cc_* ABI change; default engine stays OCCT. Entity arg orders were
  cross-checked against the OCCT `RWStep*` writer modules (EDGE_CURVE / ADVANCED_FACE
  / CIRCLE / LINE / VECTOR / ORIENTED_EDGE / B_SPLINE_CURVE_WITH_KNOTS all match) so
  the file parses through `STEPControl_Reader`. Gate 1 (host, no OCCT) GREEN — host
  `test_native_step_writer` (6 cases: canSerialize scope boundary; box → valid
  AP203 header + wrapper + mm SI_UNIT; box geometry 6 PLANE / 12 shared EDGE_CURVE /
  8 VERTEX_POINT; cylinder → CYLINDRICAL_SURFACE + CIRCLE rims; every DATA line a
  well-formed contiguous `#n = ENTITY(...);`; coordinates emitted as STEP REALs) +
  `test_native_engine::native_step_export_writes_valid_ap203_file` (the facade
  `cc_step_export` runs native on a native box, returns 1, writes a file with the
  ISO magic + MANIFOLD_SOLID_BREP + mm SI_UNIT); host CTest **21/21**, all native
  suites green. All writer functions 🟢 Excellent (≤ 7 cognitive complexity), no
  systems-band function. **Gate 2 (sim OCCT re-read parity) GREEN** —
  `tests/sim/native_step_parity.mm` + `scripts/run-sim-native-step.sh` through the
  `cc_*` facade: **`[NSTEP]` 28 passed / 0 failed** — each native STEP file re-reads
  through `STEPControl_Reader` to the SAME solid as its source (box EXACT vol 1000 /
  6 faces / 24 edges; cylinder vol rel 1.27e-3, 9 faces; holed-plate vol rel 2.90e-4,
  7 faces, valid), the native-written and OCCT-written files re-read to EQUIVALENT
  solids (writer-parity rel ≤ 4.7e-15), and a FOREIGN (OCCT-built) body forwards to
  `STEPControl_Writer` (fall-through, active native). **Two writer bugs fixed to reach
  this gate:** (1) EDGE_LOOP / ADVANCED_FACE emitted a stray extra empty string
  (`'',''`), giving EDGE_LOOP 3 args (schema 2) and ADVANCED_FACE 5 (schema 4) — OCCT
  rejected both and transferred an EMPTY solid (0 faces, vol 0); (2) a full-turn
  periodic wall (a cylindrical hole wall) was emitted as a periodic surface trimmed to
  its full period with NO seam edge, which OCCT reads back with zero wall area (a
  leaky, invalid solid) — the writer now synthesises the required SEAM edge (a straight
  LINE used forward at u=period and reversed at u=0, mirroring `STEPControl_Writer`'s
  cylindrical-hole-wall representation) for any Cylinder/Cone/Sphere face whose loop is
  closed full-circle rim edges. Both are pinned by host regression tests
  (`edge_loop_and_advanced_face_have_schema_arg_counts`,
  `cylindrical_hole_wall_emits_seam_edge`). The sewn re-read solid legitimately gains
  one seam edge per periodic wall the native deferred-edge-sharing source omits
  (native src 28 → re-read 30 for the holed plate, matching OCCT's own writer), which
  the harness edge-count check now accepts as a bounded superset. STILL OCCT (never
  faked): STEP IMPORT, IGES import/export, and a
  native solid with an out-of-scope geometry kind (Ellipse/Bezier curve, rational
  spline, Bezier surface) → OCCT fallback for an OCCT body / honest error for a
  native void. Living change `add-native-data-exchange` **archived** (validate
  --strict green). This is the native EXPORT slice only — import + IGES stay OCCT by
  design.
- ☐ #8 `drop-occt` — planned; **NOT reachable** at the current native ceiling. Two
  hard, research-grade multi-year dependencies remain: (1) a general robust curved
  boolean / blend kernel (arbitrary surface-surface intersection + shape healing) and
  (2) native STEP/IGES IMPORT (a full AP203/AP214 + IGES parser and B-rep
  reconstructor — the #7 slice was EXPORT only). Until BOTH exist, OCCT stays linked
  and Phase 4 stands COMPLETE AT ITS ACHIEVABLE NATIVE CEILING, not fully drop-OCCT.

## Remaining work to drop OCCT — ordered by difficulty (+ effort)

As of the geometry-completion batch, **every tractable/analytic capability is
native**; what remains is the SSI-gated hard core + the exchange/healing track.
Ordered easiest → hardest by *marginal* effort (deps flagged; a cheap item gated
on a hard prereq is not reachable early). Effort is **order-of-magnitude,
robustness-dominated, human-expert-equivalent** — `w` = weeks, `py` = person-years.
LOC are OCCT's (the port/reference size).

| # | Remaining item | OCCT LOC | Dep | Analytic slice | Production-robust |
|---|---|---|---|---|---|
| 1 | Twist/scale sweep, guided/rail sweep+loft, mismatched loft (self-verify-clean cases) | ~48k (TKOffset/BRepFill) | frame math | ~1–2 w | 0.5–1.5 py |
| 2 | **Numeric foundations** — `math_` solvers (Newton/FunctionSetRoot/BFGS) + `Extrema` (45k) + `Adaptor3d` (7k) | ~55k | — | n/a | **0.5–1 py** — *highest leverage; on-ramp to everything below* |
| 3 | STEP/IGES **import** (full AP203/214/242 + IGES parse + reconstruct) | ~300–600k | needs #4 | narrow (own export): ~w | 2–4 py |
| 4 | **Shape healing** (`ShapeFix`/`ShapeUpgrade`/`ShapeAnalysis`) | 87,647 | — | n/a | 2–4 py |
| 5 | **SSI + general curved booleans** (`IntPatch`/`IntWalk` 89k + BOPAlgo 76k) | ~165k | #2 | ~w/case | **3–6 py** (clean-room) / ~1.5–3 py (port from OCCT) — *the moat* |
| 6 | Curved / variable-radius / fillet-face / concave **blends** (`ChFi3d`) | 95,710 | #5 | ~w/case | 2–4 py |
| 7 | Curved **wrap-emboss** | (composition) | #5 + curved offset | ~days | 0.2–0.5 py |
| 8 | `drop-occt` — unlink + full regression | — | 1–7 | — | small, last |

Critical path: **#2 numeric foundations → #5 SSI → curved booleans → #6 blends →
#7 wrap-emboss**, with **#3 import ← #4 healing** as a parallel track. Both gate
**#8**. Total to genuinely drop OCCT ≈ **10–20 py** (a small team, several years);
matching OCCT means re-earning its person-decades of hardening on real CAD data.

### Effort banked so far (human-expert-equivalent)

The native rewrite delivered the entire analytic/tractable surface, verified vs
OCCT: math, topology, watertight tessellation, construction (extrude/revolve/
holed+typed profiles/spline+torus/N-loft/planar+non-planar sweep/threads/shank),
planar + axis-aligned box∩cylinder booleans, planar blends, STEP export.
Cumulative ≈ **0.6–0.9 py** of skilled kernel work.

- **Last batch (geometry-completion) gained ≈ 3–5 person-weeks** — spline edges,
  torus revolve (+ native `Torus`), N-section loft, non-planar RMF sweep — and
  proved the boundary (declines/fallbacks correctly identified, not faked).
- It did **not** shorten the ~10–20 py drop-OCCT tail: that tail is the SSI +
  healing + import hard core, orthogonal to this construction work. By *effort*
  we are ~5–8% of the way to full drop-OCCT; by *tractable-capability breadth*,
  ~complete. The remaining 90%+ is the research-grade core.

Progress is reflected in [ROADMAP.md](ROADMAP.md) Phase 4 and per-change
`tasks.md`; living specs are synced/archived per capability as they pass the
verification gates.

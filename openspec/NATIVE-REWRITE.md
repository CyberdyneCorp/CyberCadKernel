# Phase 4 â Native Rewrite Sub-Roadmap (drop OCCT)

The endgame: replace the OCCT adapter with **native C++20**, one capability at a
time, until OCCT can be unlinked entirely. This sub-roadmap sequences that work
and fixes the rules every capability migration follows.

Committed goal: **full drop-OCCT**, including native booleans. Honest caveat:
robust B-rep booleans and shape healing are research-grade â they will land
*progressively hardened and verified against OCCT*, not production-robust on day
one. Difficulty is flagged per capability below.

**Where this stands now (honest ceiling).** The tractable native slice of every
capability #1â#7 is DONE at the verification bar â including the native STEP EXPORT
slice (#7). Phase 4 is therefore **COMPLETE AT ITS ACHIEVABLE NATIVE CEILING, not
fully drop-OCCT**: #8 `drop-occt` is BLOCKED because two hard dependencies remain
research-grade multi-year efforts â (1) a general robust curved boolean / blend
kernel (arbitrary surface-surface intersection + shape healing) and (2) native
STEP/IGES IMPORT (the #7 slice delivered EXPORT only). Until both exist, OCCT stays
linked.

## Method (locked)

- **Clean-room from references.** Implement from math/first-principles, public
  algorithm references (e.g. *The NURBS Book* â de Boor, de Casteljau, basis
  functions; standard computational-geometry literature), and the `cc_*`
  contract. **OCCT source is a reference *oracle*** â consulted to confirm an
  algorithm matches and to compare numerics/perf â not copied verbatim. License
  is not a constraint on this project; the driver is modern, maintainable,
  fast C++20.
- **Balance maintainability Â· readability Â· performance.** Prefer clear,
  well-named, `constexpr`/`span`/`concepts`-friendly C++20 with documented
  algorithms and low cognitive complexity (systems band â¤ 25â35 for irreducible
  geometry, flagged). Optimise with data (benchmarks), not guesswork.

## Verification model (every capability)

Native code has **no OCCT dependency**, which gives two independent test gates:

1. **Host unit tests** â native code compiles and unit-tests with `clang++`
   `-std=c++20` (no OCCT, no simulator): analytic/known-value assertions
   (a known BÃ©zier point, a transform identity, an exact volume).
2. **Simulator native-vs-OCCT parity** â on the iOS simulator (OCCT linked), the
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
engine-adapter was built for) â so the app keeps working throughout, behind the
unchanged `cc_*` facade.

```
cc_* facade â active engine
                ââ NativeEngine (C++20)  ââ implements migrated capabilities
                â        ââ falls through to â for the rest
                ââ OcctEngine            ââ oracle + fallback (unlinked at the end)
```

## Capability sequence

Dependency order. Each row is one OpenSpec change (`add-native-*`).

| # | Change | Capability | Difficulty | Native-vs-OCCT oracle |
|---|---|---|---|---|
| 1 | `add-native-math-geometry` | `native-math` | moderate | `gp_*`, `BSplCLib`/`BSplSLib`/`PLib`/`ElSLib` |
| 2 | `add-native-brep-topology` | `native-topology` | moderateâhard | `TopoDS`, `TopExp`, `BRep_Tool` |
| 3 | `add-native-tessellation` | `native-tessellation` | moderate | `BRepMesh` (+ Phase-2 GPU eval) |
| 4 | `add-native-swept-solids` | `native-construction` | hard | `BRepPrimAPI`, `BRepBuilderAPI`, `BRepOffsetAPI` |
| 5 | `add-native-booleans` | `native-booleans` | **research-grade** | `BRepAlgoAPI` (BOPAlgo) |

> #5 SSI → curved-booleans implementation plan: see [SSI-ROADMAP.md](SSI-ROADMAP.md) (staged S1-S5, substrate #2 done; S1 analytic + S2 seeding + S3 marching done — the SSI curve pipeline is now NATIVE for transversal freeform/quadric pairs; **S5-a/b/c landed five native curved-boolean sub-cases** verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` — the through-drill cyl∩cyl COMMON (S5-a) + FUSE + CUT (S5-b) and the sphere∩sphere COMMON lens (S5-c, equal + unequal radii), all watertight, ΔV ≤ 8e-4 (sim `native-pass=5`); `ssi_boolean.{h,cpp}`, changes `add-native-ssi-curved-boolean` + `add-native-ssi-curved-boolean-wider` archived. Remaining: **S4 near-tangent robustness (the moat)** + wider S5 coverage (sphere fuse/cut, more curved-curved families, lifting the near-tangent gate)).
| 6 | `add-native-fillets-offsets` | `native-blends` | hard | `BRepFilletAPI`, `BRepOffsetAPI` |
| 7 | `add-native-data-exchange` | `native-exchange` | moderate (external?) | `STEPControl`, `IGESControl` |
| 8 | `drop-occt` | â | â | unlink OCCT; kernel fully native |

Booleans (#5) are the hardest and longest-lived OCCT dependency â sequenced late
and expected to iterate. The PLANAR-polyhedron slice is now native (BSP-CSG,
self-verified EXACT vs OCCT on axis-aligned boxes); curved / general booleans remain
OCCT-backed. #7 (STEP/IGES) may stay a thin external dependency
longest; a native exchange is lower priority than the modelling core.

## Status

- â **#1 `native-math`** â done at the verification bar (first capability). Both
  gates green: host analytic unit tests (55 asserts, no OCCT) + native-vs-OCCT
  parity on the iOS sim (24 groups, 0 failed, overall max numeric error
  1.486e-13, well under tolerance); no regressions (host CTest 8/8,
  `run-sim-suite.sh` 221/221). Not yet engine-wired â by design, this capability
  ships the OCCT-free math foundation only. See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-math`.
- â **#2 `native-topology`** â done at the verification bar (second capability).
  Both gates green: host invariant unit tests (`test_native_topology`, 13 cases,
  no OCCT â data model, orientation compose, location, sub-shape sharing,
  geometry attachment, stable ids, deterministic enumeration, explorer/ancestry,
  `BRep_Tool` accessors, repeat-run equality) + native-vs-OCCT parity on the iOS
  sim (3 shapes â box / cylinder / filleted-box â Ã 5 checks = **15 passed, 0
  failed**; sub-shape counts + `MapShapes` order + edgeâfaces ancestry +
  orientation flags match the oracle, accessor max error **0.000e+00** at tol
  1.0e-09, surface types match). No regressions (host CTest 9/9,
  `run-sim-suite.sh` 221/221). Header-only under `src/native/topology/`
  (`shape.h`, `explore.h`, `accessors.h`, `native_topology.h`); not engine-wired â
  by design. Deferred: non-manifold/degenerate + seam edges, `CompSolid` /
  `Internal`/`External`, holed-face parity fixture. See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-topology`.
- â **#3 `native-tessellation`** â done at the verification bar (third
  capability). Both gates green: host invariant unit tests
  (`test_native_tessellate`, no OCCT â deflection-bound, on-surface, trimming,
  watertightness, area/volume convergence, determinism) + native-vs-OCCT
  `BRepMesh` property-parity on the iOS sim (4 shapes â box / cylinder / sphere /
  filleted-box â **All 20 checks PASS**; **ALL four closed solids watertight
  `boundaryEdges==0`**; area/volume relMesh â¤ **6.0e-3**, relExact â¤ **1.24e-2**,
  bbox max corner delta â¤ **4.66e-2**, on-surface residual â¤ **5.7e-15**; triangle
  count/topology NOT compared â tessellation is an approximation). No regressions
  (host CTest 10/10, `test_native_tessellate` 13 cases, `run-sim-suite.sh` 221/221).
  Header-only under `src/native/tessellate/` (`mesh.h`, `surface_eval.h`,
  `edge_mesher.h`, `trim.h`, `uv_triangulate.h`, `face_mesher.h`,
  `solid_mesher.h`, `gpu_sample.h`, `native_tessellate.h`); not engine-wired â by
  design. RESOLVED: the curved shared-edge stitch â the mesher is now a two-stage
  pipeline (STAGE 1 `edge_mesher.h` discretizes each unique edge ONCE into a shared
  deflection-based 1D fraction list; STAGE 2 `face_mesher.h` pins both adjacent
  faces' boundaries to that shared discretization, structured-grid for full
  parametric-rectangle faces and ear-clip (`uv_triangulate.h`) for trimmed faces),
  so CURVED shared edges (cylinder capâside circle, fillet blend seams) weld
  WATERTIGHT â every closed solid (box/cylinder/sphere/filleted-box) is now
  required to mesh `boundaryEdges==0`; the weaker "bounded-open" pass is gone.
  Deferred: GPU fp32 sampling path (compiled behind `CYBERCAD_HAS_METAL`,
  CPU-verified only in this environment). See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living spec archived to
  `openspec/specs/native-tessellation`.
- â **#4 `native-construction`** â **CORE done at the verification bar; advanced
  swept solids are a follow-up (`#4b`).** The first **engine-wired** capability. Two
  construction ops are native: `cc_solid_extrude` (closed polygon â prism: bottom/top
  `Plane` caps + one planar quad `Plane` side per profile edge) and `cc_solid_revolve`
  for **LINE-SEGMENT** profiles (per-segment surface of revolution â parallelâ`Cylinder`,
  perpendicularâ`Plane`, obliqueâ`Cone`; full 360Â° closes the shell, partial angle adds
  two `Plane` meridian caps). Built on the #1â#3 foundations under
  `src/native/construct/construct.h` (OCCT-free, host-buildable). Wired through
  `NativeEngine : IEngine` (`src/engine/native/`), which serves these ops + native
  tessellate / mass / bbox / subshape on its own native bodies and **falls through to
  OCCT** (or the stub on host) for everything else, behind an ADDITIVE facade toggle
  `cc_set_engine(int)` / `cc_active_engine()` (**default stays OCCT** â existing suites
  unchanged). Both gates green: host `test_native_construct` + `test_native_engine`
  (no OCCT â box exact vol/area/6-faces/centroid/bbox/watertight, triangle prism
  watertight vol=areaÃdepth, L-prism, full-turn tube 9Ï, quarter-turn tube 9Ï/4, cone
  4Ï; CTest **12/12**) + native-vs-OCCT parity on the iOS sim through the facade
  (`native_construct_parity.mm`, **17/17** `[NCONS]`): planar prisms EXACT (vol/area/
  centroid rel 0.00e+00, identical face tiling), curved revolves within a deflection
  bound (vol rel â¤ 2.36e-2, area rel â¤ 1.24e-2, bbox maxCornerÎ â¤ 4.37e-2, all
  watertight), plus a fall-through boolean (nativeâOCCT, fuse vol=14) proving no native
  interception. No regressions (host CTest 12/12, `run-sim-suite.sh` 221/221 re-verified
  against a freshly rebuilt SIMULATORARM64 slice). Documented representational difference
  (not a geometric mismatch): the native builder emits per-face edges / per-patch vertices
  (edge/vertex SHARING deferred) and tiles a full-turn surface of revolution into < Ï
  angular patches (periodic-face construction deferred), so native V/E and the full-turn
  face count differ from OCCT's shared/periodic representation while the SOLID is
  geometrically identical. See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md);
  living spec archived to `openspec/specs/native-construction`.
  - **â `#4b` follow-up â Tier A + Tier B (2-section ruled loft) DONE at the
    verification bar; the rest still OCCT-fallthrough (not faked).** NOW NATIVE
    (host-verified, engine-wired behind the same `cc_set_engine(1)` toggle):
    `cc_solid_extrude_holes` (outer polygon +
    CIRCULAR through-holes kept as TRUE circle edges + cylinder walls),
    `cc_solid_extrude_polyholes` (outer + POLYGON holes), `cc_solid_extrude_profile` /
    `_profile_polyholes` (TYPED outer profile â kind 0 line / 1 arc / 2 full circle â
    with circular + polygon holes; a whole-circle profile keeps one Circle cap edge +
    one Cylinder wall), and `cc_solid_revolve_profile` (TYPED profile revolve: line â
    Plane/Cylinder/Cone, an arc whose circle centre lies ON the axis â Sphere band;
    full 2Ï closes, partial adds two planar meridian caps). **Tier B:**
    `cc_solid_loft` / `cc_solid_loft_wires` for TWO sections with EQUAL vertex counts
    (â¥3) that are both PLANAR â corresponding edge pairs span one BILINEAR (degree-1
    BÃ©zier) ruled side face + two planar caps â watertight solid (mirrors ruled
    `BRepOffsetAPI_ThruSections`); built in `src/native/construct/loft.h` (OCCT-FREE,
    host-buildable, all functions cognitive complexity â¤ 7). Built in
    `src/native/construct/profile.h` (OCCT-FREE, host-buildable) + a robustified
    multi-hole cap triangulator (visibility-checked, rightmost-first hole bridging in
    `src/native/tessellate/uv_triangulate.h`, replacing the single-hole-only nearest-
    vertex heuristic). Gate 1 green: host `test_native_profile` (12 cases â circular /
    polygon / multi-hole / combined holes watertight with exact-or-convergent volume;
    full-circle extrude â cylinder; on-axis arc revolve â sphere 36Ï; partial-turn
    revolve; typed line/arc extrude) + `test_native_engine` (5 new facade cases through
    `cc_solid_extrude_holes/_polyholes/_profile` + `cc_solid_revolve_profile`); host
    CTest 13/13, existing suites incl. `test_native_tessellate` unchanged. STILL
    OCCT-fallthrough (the native builder returns a NULL Shape â `NativeEngine` forwards
    to OCCT, never fakes): **kind-3 SPLINE profile edges** (extrude AND revolve),
    **arc-revolve whose circle centre is OFF the axis** (a TORUS surface of revolution
    â no native Torus surface yet), a loft with MISMATCHED section counts / a
    NON-PLANAR section / 3+ sections / guided or rail loft (Tier C), and
    twisted-with-real-twist / guided-sweep / loft-along-rail / threads (Tiers CâE).
    (`cc_solid_sweep` itself is NOW native for straight + smooth-planar spines â see the
    Tier-C entry below.) A
    kind-1 ARC extrude edge is a TRUE `Circle` cap edge + a `Cylinder` side wall â one
    bounded, non-periodic patch per â¤180Â° span (split threshold is Ï for the EXTRUDE
    wall, NOT the revolve's 120Â°: an extrude wall is never periodic, so a semicircle is
    ONE patch matching OCCT's single cylindrical face) â not a chord polyline. Gate 2
    (sim OCCT parity) GREEN: `native_construct_profiles_parity.mm` through the cc_*
    facade, **22 passed / 0 failed** â the 5 native ops (holed / polyhole / typed
    line+arc / line-revolve tube / on-axis-arc-revolve sphere) match the OCCT oracle
    (planar EXACT; curved deflection-bounded vol rel â¤ 5.0e-2, all watertight; native
    FACE count a kâ¥1 integer multiple of OCCT's), and the 2 deferred sub-cases (kind-3
    spline extrude, off-axis-circle â torus revolve) transparently delegate to OCCT
    (vol rel 0.00e+00). Note: `splineXYCount` on the kind-3 side-channel is the number
    of DOUBLES (2Ã the point count), matching the OCCT `addSplineEdge` bounds guard â
    now documented in `cc_kernel.h`. **Tier B (2-section ruled loft):** Gate 1 (host,
    no OCCT) GREEN â `test_native_loft` (9 cases: prism / frustum / twisted rotated
    square / two-3D-wire triangle prism / tilted planar section watertight, + deferred
    mismatched-count / non-planar / degenerate / bad-input all NULL) + 2 new
    `test_native_engine` facade cases (native square-frustum loft vol 56 @ 6 faces;
    native `loft_wires` triangle prism vol 18); CTest **14/14**, `loft.h` cognitive
    complexity â¤ 7 across all functions. Gate 2 (sim OCCT parity) GREEN â
    `tests/sim/native_loft_parity.mm` + `scripts/run-sim-native-loft.sh` through the
    `cc_*` facade under `cc_set_engine(0/1)` (OCCT default restored in teardown):
    **`[NLOFT]` 17 passed / 0 failed** â square frustum (vol rel 2.54e-16) / hex
    prism (rel 0.00e+00) / two-wire triangle prism (rel 0.00e+00) EXACT,
    rotated-square TWIST deflection-bounded (vol rel 5.33e-3, watertight, tol 5e-2),
    native F = OCCT F (n=1Ão) on all four, plus the mismatched-count deferred case
    delegating to OCCT (vol rel 0.00e+00, native active â fall-through proof). Runs
    on the sim (OCCT linked); on `run-sim-suite.sh`'s SKIP list (own `main()`), so
    the 221-assertion suite count is unchanged. No regressions (`test_native_tessellate`
    green, `run-sim-suite.sh` 221/221).
  - **â `#4b` Tier C (sweep / pipe-shell) â `cc_solid_sweep` DONE at the verification
    bar; the guided/rail/real-twist pipe-shell cases stay OCCT-fallthrough (not faked).**
    NOW NATIVE (engine-wired behind the same `cc_set_engine(1)` toggle):
    `cc_solid_sweep` for (a) a **STRAIGHT** spine (exact directional prism, vol =
    profileArea Ã |d|) and (b) a **SMOOTH CURVED but PLANAR** spine. The crux â and the
    fix that made Gate 2 pass â is the FRAME LAW: the OCCT oracle
    `BRepOffsetAPI_MakePipe` uses `GeomFill_CorrectedFrenet`, which for a **planar** spine
    collapses to a **constant rotation** (`GeomFill_CorrectedFrenet.cxx`, `isPlanar` â
    `Law_Constant`), i.e. it TRANSLATES the section with a FIXED orientation, NOT a
    perpendicular-tracking sweep. So `src/native/construct/sweep.h` holds the start frame
    CONSTANT across stations (`detail::constantFrames`), builds one bilinear ruled band
    per (profile edge Ã spine segment) with shared per-station rings, and caps both ends
    in the fixed section plane â a watertight solid. (An earlier RMF / parallel-transport
    revision kept the section perpendicular and produced the Pappus arc-length volume â
    geometrically "nicer" but a REAL mismatch vs the oracle, correctly rejected by the
    parity gate; we match the oracle.) `cc_twisted_sweep` is native ONLY when it reduces
    to the plain sweep (twist â 0, scale â 1 â forwards to `build_sweep`). Gate 1 green:
    host `test_native_sweep` (11 cases â straight prism / collinear-collapse / arbitrary-
    3D-direction / pentagon / zero-twist prism / smooth-planar-arc watertight + constant-
    frame volume `AÂ·|ÎspineÂ·nÌ|` / constant-frame invariance / degenerate + real-twist +
    tight-curvature deferrals) + `test_native_engine` (`native_sweep_smooth_arc` vol
    82.57 = the oracle value, `native_sweep_tight_and_twisted_defer`); host CTest 15/15,
    existing suites unchanged. STILL OCCT-fallthrough (native builder returns NULL â
    `NativeEngine` forwards, never fakes): a **NON-PLANAR** curved spine (OCCT's genuine
    non-constant corrected-Frenet law), a **TIGHT-CURVATURE / self-intersecting** spine
    (guarded by `spineTooSharp`), a **real-twist/scale** `cc_twisted_sweep` (OCCT
    `ThruSections`), and the pipe-shell/guide cases **`cc_guided_sweep`** /
    **`cc_loft_along_rail`** (engine-glue fall-through). Gate 2 (sim OCCT parity) GREEN:
    `tests/sim/native_sweep_parity.mm` + `scripts/run-sim-native-sweep.sh` through the
    `cc_*` facade under `cc_set_engine(0/1)` â **`[NSWEEP]` 11 passed / 0 failed**: the
    straight sweep EXACT (vol rel 7e-16) and â because native and OCCT now share the same
    constant-frame law and polyline â the **smooth-arc sweep EXACT too** (vol o=330.299
    n=330.299 **rel 1.7e-16**, bbox maxCornerÎ 1.0e-7, native F = OCCT F = 98, watertight),
    plus the three deferred cases (real-twist / guided / loft-rail) delegating to OCCT
    (vol rel 0.00e+00, native active â fall-through proof). On `run-sim-suite.sh`'s SKIP
    list (own `main()`), 221-assertion count unchanged. No regressions (host CTest 15/15,
    `run-sim-suite.sh` 221/221). Living change: `openspec/changes/add-native-sweep`.
  - **â `#4b` Tier D (threads / tapered shank) â ALL THREE NATIVE at the Gate-1 bar:
    `cc_tapered_shank`, `cc_helical_thread`, `cc_tapered_thread`.** Engine-wired behind
    the same `cc_set_engine(1)` toggle. **`cc_tapered_shank`** â a pointed-shank silhouette
    (cone tip â full-radius cylinder â head disk) revolved 360Â° about Z by REUSING the
    already-parity-verified native revolve (`build_revolution`, construct.h). The tip is
    a TRUE on-axis apex (the revolve collapses its angular copies to one shared vertex, so
    no sliver breaks the weld â a non-zero tip radius does NOT weld, verified), giving a
    ROBUSTLY watertight solid at every deflection {0.05â¦0.005} with volume
    `âÏ rÂ²Â·taperHeight + Ï rÂ²Â·fullHeight` within the deflection bound (r5/fh20/th10 â
    exact 1832.6, meshed 1828â1832), matching `BRepPrimAPI_MakeRevol`. Built in
    `src/native/construct/thread.h` (OCCT-FREE, host-buildable, all four functions
    cognitive complexity ð¢ Excellent â¤ 5). **`cc_helical_thread` / `cc_tapered_thread`
    (NOW NATIVE):** the full radial-V helical tiling â a V/triangular section transported
    RADIALLY along the pitch-line helix via the AXIS auxiliary-spine law
    (radial = (cosÎ¸,sinÎ¸,0), axial = +Z, so the V does NOT Frenet-rotate â mirroring
    `BRepOffsetAPI_MakePipeShell::SetMode(axisWire,true)`), tiled into three bilinear ruled
    bands per span (loft.h `ruledSideFace`) with shared per-station rings + two planar V
    end caps, capped at `samplesPerTurn â [8,24]` and GUARDED against self-intersection
    (pitch-line radius must clear the axis at both ends; `2Â·halfBase â¤ pitch`; degenerate
    params â NULL). **The per-turn seam WELD was the last blocker and is now fixed at the
    mesher level (topology-preferred, geometry untouched):** the ruled-band â band and
    band â V-cap seams are STRAIGHT edges shared by two faces that each evaluated the seam
    through their OWN bilinear surface, so the two boundary points agreed only to ~1 ULP;
    when a shared coordinate landed on a spatial-weld cell boundary (coordÂ·âtol = k+0.5)
    the ULP twins rounded to opposite cells and the weld left the seam OPEN at isolated
    deflections. The mesher now emits, for every straight boundary edge, CANONICAL seam
    points â interpolated at the shared sample indices `i/n` between the edge's two
    BOUNDING VERTICES in a fixed lexicographic order (`edge_mesher.h` `CanonicalEndpoints`
    / `face_mesher.h` `recordEdgeAnchors`), which is BIT-IDENTICAL for the two coincident
    edges regardless of build order â and SNAPS each seam-lying vertex to its canonical
    point (`BoundaryAnchors`). The two faces therefore place the identical 3D point and the
    conservative single-cell weld fuses them, with NO widening of the merge radius (which
    would over-collapse a fine curvature grid). This is exactly the "one shared 1D
    discretization pinned on both faces" contract, completed at the 3D-point level; the V
    volume/geometry are unchanged. Result: helical (major6â¦20 / pitch2â¦4 / turns1â¦5 /
    depth0.5â¦1.5 / spt8â¦24) and tapered (top5â¦8 / tip3â¦4 / â¦) are ROBUSTLY watertight
    (`boundaryEdges==0`) at EVERY deflection in the `robustlyWatertight` ladder across the
    full swept parameter space (432/432 helical + 96/96 tapered candidates â native), so
    the engine keeps them NATIVE. **HONESTY (unchanged guard):** a FINE-PITCH /
    self-intersecting thread (turns fold through each other, e.g. major2/pitch0.2/depth3)
    still fails `robustlyWatertight` â a self-overlapping mesh is non-manifold no matter
    how the vertices weld â so it still FALLS THROUGH to the OCCT `MakePipeShell` oracle
    (labelled, verified, never faked; the native builder never emits the round-profile
    fallback). Gate 1 (host, no OCCT) GREEN â `test_native_thread` (9 cases: shank watertight+volume,
    shank ppmÂ³ scaling, shank degenerate NULL, `helical_thread_is_watertight_across_ladder`
    + `tapered_thread_is_watertight_across_ladder` â a HARD requirement asserting
    `boundaryEdges==0` at EVERY deflection in the ladder {0.1,0.05,0.02,0.01} with the right
    V-tiling face count, positive volume sign and turn count, degenerate-params NULL,
    pitch-radius-below-axis NULL, tapered-tip-below-axis NULL â plus the
    `fine_pitch_self_intersecting_thread_not_supported` guard) + `test_native_engine`
    facade cases (native `cc_tapered_shank` watertight vol 1832.6; degenerate shank â
    fall-through 0; **`native_thread_runs_native_watertight`** â the well-formed helical +
    tapered thread now run NATIVE through the facade with valid watertight mass-properties;
    `native_fine_pitch_thread_falls_through_to_default` â the self-intersecting thread still
    defers to the fallback); host CTest 18/18, all native suites green
    (`test_native_construct/profile/loft/sweep/tessellate/boolean/blend/topology/thread`).
    The seam-weld fix is exercised directly by `test_native_thread` (both helical + tapered
    candidates are asserted watertight across the deflection ladder) and by a broad
    parameter sweep (432 helical + 96 tapered configs, all `boundaryEdges==0`). Gate 2
    (sim OCCT parity) â `tests/sim/native_thread_parity.mm` +
    `scripts/run-sim-native-thread.sh` through the `cc_*` facade: **`cc_tapered_shank` runs
    NATIVE** and matches the OCCT `BRepPrimAPI_MakeRevol` oracle â r5/fh20/th10 `[native]`
    vol o=1837.94 n=1830.27 **rel 4.17e-03**, area rel 3.64e-03, centroidÎ 3.85e-02
    (tol v=5e-02 c=1e-01), bbox maxCornerÎ 1.00e-07, subshapes F 4â9 / E 5â30 / V 3â30
    (angular tiling), tessellate watertight tris=144 meshVolRel 3.81e-03. With the seam
    weld fixed the **well-formed helical / tapered thread ops now run NATIVE** (radial-V
    tiling, robustlyWatertight passes) and are cross-checked vs the OCCT `MakePipeShell`
    oracle to the deflection-bounded volume/area/bbox tolerance: `cc_helical_thread`
    mr5/p2/t4/d1 `[native]` vol o=70.2841 n=68.3767 **rel 2.71e-02** / area rel 1.73e-02 /
    centroidÎ 4.83e-05 / bbox maxCornerÎ 1.44e-03 / F 5â194 E 9â774 V 6â195 / tessellate
    watertight tris=1286 meshVolRel 1.40e-03, and `cc_tapered_thread` top6/tip4/p2/t4
    `[native]` vol o=70.2677 n=68.3767 **rel 2.69e-02** / watertight tris=1286 (the ~2.7%
    native-vs-OCCT gap is chord-vs-arc at spt=16, tightening to ~1.3% at spt=24, while the
    native mesh matches its OWN B-rep volume to meshVolRel â¤ 1.40e-3). A FINE-PITCH /
    self-intersecting thread (mr5/p0.3/t8/d1) still verifies as OCCT fall-through
    (`[fallback]`, `cc_active_engine()==1`, vol rel 0.00e+00). Both gates are GREEN and the
    engine enforces the watertight bar at runtime via `robustlyWatertight`. Living change
    (archived): `openspec/changes/add-native-threads` â `openspec/specs/native-construction`.
  - **â `#4b` geometry-completion batch (Tier 1 + Tier 2#4) â spline extrude, off-axis-arc
    torus revolve, N-section ruled loft, and a NON-PLANAR (RMF) sweep DONE at the
    verification bar; twist/scale + guided/rail + truly-self-intersecting thread stay
    OCCT-fallthrough (SSI/Tier-4, not faked).** Change `add-native-geometry-completion`,
    engine-wired behind the same `cc_set_engine(1)` toggle. **Tier 1 NATIVE:** (A) a **kind-3
    SPLINE** outer profile edge extrude (a native B-spline edge via native-math NURBS + a
    `BSpline` swept side wall + planar caps â watertight) and an **OFF-AXIS-ARC** revolve â a
    **TORUS** surface of revolution (native `Torus` added in `src/native/math/torus.h`;
    emitted as EXACT rational-quadratic B-spline patches so no new tessellator surface kind
    was needed) â both in `src/native/construct/residuals.h`. **Tier 2#4 NATIVE:** an
    **N-section (3+)** ruled loft chain (`src/native/construct/loft.h`, generalizing the
    Tier-B 2-section builder â shared interior rings, first/last caps) and a **NON-PLANAR
    smooth spine** sweep via the double-reflection RMF (Wang et al. 2008,
    `src/native/construct/sweep.h`; the RMF collapses to the constant frame on a planar spine,
    preserving Tier-C parity). Gate 1 (host, no OCCT) GREEN â host build clean, CTest **22/22**
    (incl. `test_native_residuals`, `test_native_loft`, `test_native_sweep`, `test_native_thread`,
    `test_native_tessellate`, `test_native_step`, `test_native_engine`). Gate 2 (sim OCCT
    parity) GREEN â `tests/sim/native_geomcompletion_parity.mm` +
    `scripts/run-sim-native-geomcompletion.sh` through the `cc_*` facade under
    `cc_set_engine(0/1)`: **spline extrude** vol o=45.6 n=45.5547 **rel 9.92e-04** (watertight,
    132 tris, faces 4â4); **torus revolve** vol o=98.696 n=96.0542 **rel 2.68e-02** (watertight,
    1620 tris, faces 2â6); **ruled frustum** + **straight-rail** N-section loft vol rel
    **1.43e-14 / 5.58e-15 EXACT** (watertight, 432 tris, faces 6â6); **smooth-arc (RMF) sweep**
    vol o=330.299 n=330.299 **rel 3.44e-16 EXACT** (watertight, 196 tris, faces 98â98). **STILL
    OCCT-fallthrough / DECLINE (not faked):** a **self-crossing spline** profile and a **spindle
    torus** (off-axis arc crossing the axis â self-intersecting SoR) DECLINE on BOTH engines
    (unbuildable SSI, Tier 4, occtId=0 natId=0); a **mismatched-count loft** â OCCT
    `ThruSections` (vol 202.185), a **hard curved rail** â OCCT `MakePipeShell` (258.596), a
    **self-intersecting sweep** â OCCT `MakePipe` (17.9515), a **real-twist `cc_twisted_sweep`**
    â OCCT `ThruSections` (320), and a **self-intersecting thread** â OCCT `MakePipeShell`
    (1446.76) all delegate with native active=1 (rel 0.00e+00, no interception). **The
    accumulating-twist/scale sweep, the guided/rail cases, and the thread self-intersection
    resolver did NOT extend the native set beyond what self-verifies watertight + oracle-correct
    â those remaining fall-throughs now specifically need SSI / Tier-4 (surface-surface
    intersection + trimming).** No regressions (`run-sim-suite.sh` 221/221, own-`main()` parity
    harness on the SKIP list). See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living
    change `openspec/changes/add-native-geometry-completion` â archived to
    `openspec/specs/native-construction`.
- â **Numeric foundations (remaining-work #2 â the substrate) â NumPP/SciPP ADOPTED +
  native closest-point DONE at the verification bar.** NumPP + SciPP (the org's C++20, MIT
  NumPy/SciPy ports) are the kernel's OCCT-free numeric substrate â referenced **by
  absolute path exactly like OCCT (NOT vendored)**, CPU-only, consuming the SciPP
  `optimize`/`linalg`(+`spatial`/`integrate`) subset with **`special`+`stats` EXCLUDED**
  (the Homebrew-libc++ ISO-29124 gap), gated by `CYBERCAD_HAS_NUMSCI` (default OFF, so the
  rest of `src/native` builds without them). On top, a thin OCCT-free facade
  (`src/native/numerics/`) exposes the generic solvers (root / `fsolve` / `minimize`(BFGS) /
  `least_squares`(LM) / `solve` / `lstsq`) and native **closest-point / projection** (the
  `Extrema` on-ramp â pointâcurve / pointâsurface, multi-start + SciPP refine, global-best
  foot). Both gates green: host `test_native_numerics` (22 assertions, no OCCT â solver
  known-values + closed-form + brute-force closest-point, built under
  `CYBERCAD_HAS_NUMSCI=ON`) + native-vs-OCCT `Extrema` parity on the iOS sim
  (`native_numerics_parity.mm`, **All 22 `[NNUM]` cases PASS** â dDist â¤ **1.776e-15**,
  analytic plane/cylinder/sphere feet fp-exact dPoint â¤ 1.707e-10, B-spline within tol,
  largest `bspline_surf#3` dPoint **3.946e-08** at corner u=v=0). Substrate compiles+links
  **77/77 TUs** on HOST and arm64-iOS-simulator (`scripts/build-numsci.sh {host|iossim}` â
  `libnumsci_<target>.a`). No regressions: host `NUMSCI=OFF` CTest 22/22
  (`test_native_numerics` correctly ABSENT), `NUMSCI=ON` CTest 23/23, `run-sim-suite.sh`
  221/221 (determinism serial==parallel bit-reproducible). This realizes the eval's
  ~**60â75% effort saving** on #2 (â ~0.15â0.35 py) â flipping #2 from *planned* to
  *done-at-bar* and moving numeric foundations OFF the critical path. Deferred (NOT
  blocking, recorded): multiple-extrema enumeration, curve-curve / surface-surface distance
  (`Extrema_ExtCC` / `Extrema_ExtSS`), the `bspline_surf#3` corner caveat. **SSI is NOT
  fully bought by this adoption â the substrate provides the re-projection corrector
  the S2 seeding + S3 marching layers are built on (both now DONE for transversal pairs), but
  the near-tangent / coincident / branch-point surface-surface-intersection moat stays
  capability #5 as S4 (its DETECTION + CLASSIFICATION layer S4-a/b + the first
  near-tangent MARCH-THROUGH slice S4-c now DONE at the bar; the deeper marching core
  S4-d…f is the remaining tail), written on top of this substrate + the
  S3 tracer.** Change `add-native-numerics` **archived**. See
  [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md) numeric-foundations result table.
- â **#5 `native-booleans` â PLANAR-polyhedron slice DONE at the verification bar;
  curved / general still OCCT-fallthrough (not faked).** `cc_boolean` (fuse / cut /
  common) is NATIVE for **PLANAR-faced solids** (polyhedra â axis-aligned boxes, prisms)
  via a **BSP-tree CSG** (Naylor-Amanatides-Thibault 1990) over the solids' planar
  polygons â which IS face-face intersection + fragment split + inside/outside
  classification expressed as recursive plane-clip / invert, and handles
  coplanar-coincident faces (two boxes sharing a wall) robustly, with a B-rep-level
  T-junction repair + triangulation (`assemble.h`) closing the coplanar seams. Guarded by
  a MANDATORY self-verify (`robustlyWatertight` + set-algebra volume `Vr â VaÂ±VbâVab`) that
  DISCARDS any candidate that is not a valid watertight solid with the correct volume â
  falls through to OCCT. Built OCCT-free under `src/native/boolean/` (`polygon.h`, `bsp.h`,
  `assemble.h`, `native_boolean.h`, entry `boolean_solid(a, b, op)`), engine-wired behind
  the same `cc_set_engine(1)` toggle (default stays OCCT). Both gates green: host
  `test_native_boolean` + `test_native_engine` (no OCCT â box fuse/cut/common watertight
  EXACT set-algebra volume, prism/simple-concave, self-verify rejecting an open/wrong-volume
  candidate, curved/coincident/foreign fall-through â NULL; CTest **17/17**) + native-vs-OCCT
  parity on the iOS sim through the facade (`native_boolean_parity.mm`, **25/25**): box
  overlap fuse (rel 1.27e-16) / cut (2.96e-16) / common (2.22e-16), contained fuse
  (0.00e+00) / common (2.22e-16) all EXACT + watertight, the self-verify correctly rejecting
  a nativeâ©native DISJOINT out-of-domain result, plus curved (cyl-box fuse rel 0.00e+00),
  near-coincident (rel 0.00e+00) and disjoint (rel 0.00e+00) OCCT-fallthrough â all
  delegated, no native interception. No regressions (host CTest 17/17,
  `run-sim-suite.sh` 221/221 â only change is `native_boolean_parity.mm` on the SKIP list).
  STILL OCCT-fallthrough (native builder returns NULL / self-verify discards â forwarded,
  never faked): **curved-face booleans** (surface-surface intersection of cylinder / sphere
  / cone / NURBS), **near-tangent / coincident / degenerate** configurations, **disjoint**
  operands, **foreign** (OCCT-built) operands, and **general / concave-general / mixed**
  cases. **Booleans remain the longest-lived OCCT dependency for curved / general** â
  surface-surface intersection, robust near-tangent handling, and full shape healing are
  future work. See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md); living change
  `openspec/changes/add-native-booleans` â archived to `openspec/specs/native-booleans`.
  - **â Curved analytic slice (deferred residual #2) â AXIS-ALIGNED box â· axis-parallel
    cylinder NOW NATIVE at BOTH gates (host + sim parity), ARCHIVED; general curved still
    OCCT-fallthrough (honest).** `cc_boolean` cut / fuse / common is NATIVE when one operand is an
    axis-aligned box and the other a cylinder whose axis â¥ a box axis (and a world axis),
    with the cylinder RADIALLY INSIDE the box cross-section. Here plane-cylinder
    intersection is ANALYTIC â a box face â the axis cuts the cylinder in a CIRCLE â so
    instead of faceting (which would fail the analytic-volume self-verify) or a
    research-grade surface-surface solver, the builder RECOGNISES the (box, cylinder) pair
    and CONSTRUCTS the closed-form result B-rep directly from TRUE `Cylinder` walls +
    `Circle` rim edges + `Plane` caps (the exact watertight face/edge kinds a native
    cylinder-with-holes prism uses): **cut** â box with a round THROUGH hole
    (`boxVol â ÏrÂ²Â·h`), **common** â the cylinder segment clipped to the box axial extent
    (`ÏrÂ²Â·overlap`), **fuse** â box + a protruding round BOSS (`boxVol + ÏrÂ²Â·protrude`).
    Every curved seam is a SHARED `Circle` rim edge, so the mesher's two-stage
    shared-1D-discretization welds it WATERTIGHT across the deflection ladder (verified
    `boundaryEdges==0` at {0.1,0.05,0.02,0.01,0.005} on all three world axes, off-centre).
    Built OCCT-free in `src/native/boolean/curved.h` (recognisers `recogniseBox` /
    `recogniseCylinder`, world-frame axis-aware primitive builders, dispatcher
    `tryBoxCylinder`), wired into `native_boolean.h::boolean_solid` (curved path tried
    FIRST; the planar BSP-CSG path is unchanged) and guarded by an ANALYTIC-volume
    self-verify in `native_engine.cpp` (`curvedBooleanVerified`: `Vr` must match the
    closed-form `boxVol Â± ÏrÂ²Â·len` to the curved-mesh deflection bound, else DISCARD â
    OCCT). Gate 1 GREEN (host CTest **18/18**): `test_native_boolean` adds box-cylinder
    cut / common / fuse (watertight, analytic volume within ~0.2% deflection bound on all
    three axes) + honest DECLINE cases (wrong-order cylâbox, radial breach, blind hole,
    cone operand â NULL â OCCT); `test_native_engine` asserts the engine's analytic guard
    engages and errs honestly (never faked) when the facade-built config leaves the family.
    Cognitive complexity: worst `tryBoxCylinder` 12 (ð¡), no ð /ð´. **Gate 2 (sim
    native-vs-OCCT parity) GREEN** â `tests/sim/native_curved_boolean_parity.mm` +
    `scripts/run-sim-curved-boolean.sh` through the `cc_*` facade: `[NCURVBOOL]` **18 checks
    (6 cases Ã 3), 0 failed** â 3 NATIVE analytic-intercept (through-hole-cut mass rel
    **3.19e-04** / area rel 2.10e-08 / watertight 216 tris; boss-fuse rel **6.10e-05** / area
    rel 2.00e-05 / watertight 212 tris; common rel **1.30e-03** / area rel 5.84e-04 /
    watertight 196 tris) + 3 OCCT-fallback (blind-hole-cut / oblique-cyl-cut / sphere-box-cut,
    rel 0 forwarded, volume-bound tessellation only). No regressions (host CTest 19/19 incl.
    `test_native_boolean` + `test_native_tessellate`; `run-sim-suite.sh` 221/221). STILL
    OCCT-fallthrough (DECLINE â NULL, never faked): **sphere / cone / NURBS**, **NON-axis-aligned
    cylinders**, **cylinder â· cylinder**, **radially-breaching (â¥-face LINE-ruling slots)**,
    **blind holes / non-through cuts / cylâbox**, and **near-tangent / coincident-curved**.
    Living change `openspec/changes/add-native-curved-booleans` **archived** to
    `openspec/specs/native-booleans` (validate --strict green). **General curved B-rep
    booleans (surface-surface intersection, robust near-tangent handling, shape healing)
    remain research-grade OCCT-backed â the longest-lived OCCT dependency.**
  - **SSI Stage S1 (analytic surface-surface intersection) — DONE at the verification
    bar (both gates), ARCHIVED; general / freeform / near-tangent SSI is S2–S4 (honest).**
    SSI is the enabler for the S5 general curved-boolean payoff (see
    [`SSI-ROADMAP.md`](SSI-ROADMAP.md), staged S1–S5). S1 delivers CLOSED-FORM
    intersection curves for the elementary-surface family, OCCT-free and header-only under
    `src/native/ssi/`, built over `src/native/math` ONLY (IntAna-style closed form; NO
    GeomAPI / NO numsci — the SSI unit test does not require NUMSCI). SSI is INTERNAL: no
    `cc_*` entry point; parity asserted at the `cybercad::native::ssi` C++ boundary, like
    native-math / native-topology. **17 analytic-native pairs** verified vs OCCT
    `GeomAPI_IntSS` (all curve TYPES match; on-surface / coincidence residuals ≤ ~4e-15,
    well inside each pair's tol): plane∩plane (Line), plane∩sphere (Circle), plane∩cyl
    (⟂ Circle / ∥ 2 Lines / ∠ Ellipse), plane∩cone (Circle / Ellipse / Parabola / 2
    Hyperbola branches), plane∩torus (⟂ axis 1–2 circles, ∋ axis 2 meridian circles),
    sphere∩sphere (Circle), coaxial sphere∩cyl / sphere∩cone / cyl∩cone (Circles),
    parallel cyl∩cyl (2 Lines), coaxial cyl∩cyl (coincident). **Honestly DEFERRED →
    `NotAnalytic` (never faked):** skew cyl∩cyl (OCCT emits 7 Ellipse curves — a planar
    quartic, no degree-≤2 reduction) and by the same rule general cone∩cone, non-coaxial
    cone∩cyl / sphere∩cyl / sphere∩cone, oblique plane∩torus (spiric quartic),
    torus∩curved, and all freeform pairs — these route to **S2 subdivision seeding
    (DONE)** / S3 marching (NEXT) / S4 robustness. `NotAnalytic` + empty `curves` IS the
    contract with S2/S3/OCCT. Both gates green: host `test_native_ssi` (**11 cases, 0 failed**;
    NUMSCI OFF CTest **23/23**, NUMSCI ON CTest **24/24**) + sim native-vs-OCCT
    `GeomAPI_IntSS` parity `scripts/run-sim-native-ssi.sh` (**18 pairs, 0 failed**). No
    regressions (`run-sim-suite.sh` **221/221**). Files: `src/native/ssi/{curve,tolerance,
    dispatch,plane_conics,plane_torus,quadric_pairs,native_ssi}.h` +
    `tests/native/test_native_ssi.cpp` + `tests/sim/native_ssi_parity.mm`. Living change
    `openspec/changes/add-native-ssi-analytic` **archived**. See
    [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md) SSI-S1 result table.
  - **SSI Stage S2 (subdivision seeding) — DONE at the verification bar (both gates,
    TRANSVERSAL); near-tangent / coincident / degenerate seeding is S4 (honest).** Finds ≥1
    seed point per **transversal** intersection branch for the **freeform** (NURBS / Bézier /
    B-spline) and **non-closed-form quadric** pairs S1 defers as `NotAnalytic`: recursive
    patch-AABB-overlap subdivision → candidate regions → refine to a point with
    `least_squares(S₁(u₁,v₁) − S₂(u₂,v₂) = 0)` on the numerics substrate → 3D/param dedup to
    ~one seed per branch. OCCT-free in `src/native/ssi/` (`cybercad::native::ssi`); the refine
    is guarded by `CYBERCAD_HAS_NUMSCI`; INTERNAL (no `cc_*`). Per-patch AABB = control-net
    convex hull ∩ sampled-with-Lipschitz-margin for freeform, sampled+margin for
    elementary+torus. Both gates green: host `test_native_ssi_seeding` (**6 cases, 0 failed** —
    skew cyl→2, crossing spheres→1, sphere∩Bézier-bump→1, parallel planes→0, tangent spheres→
    `deferredTangent` (no faked seed), deeper resolution recovers a small loop; NUMSCI OFF CTest
    **23/23** with `test_native_ssi_seeding` + `test_native_numerics` correctly ABSENT, NUMSCI ON
    CTest **25/25**) + sim native-vs-OCCT `GeomAPI_IntSS` **recall** parity
    (`tests/sim/native_ssi_seeding_recall.mm`): **3/3 transversal branches recalled at recall
    1.00**, tangent = 0 everywhere, max seed on-surface residual **3.51e-16** (via
    `GeomAPI_ProjectPointOnSurf::LowerDistance` on both OCCT surfaces, well under 1e-6). OCCT
    NbLines (3/2/2) is its arc-split count, not the analytic branch count the recall denominator
    uses. No regressions (`run-sim-suite.sh` **221/221**, xcframework rebuilt with the new
    `src/native/ssi/seeding.cpp`). **Honest scope:** TRANSVERSAL only — near-tangent / coincident
    / degenerate seeding ill-conditions the refine → deferred to **S4** (`SeedSet.deferredTangent`,
    reported not faked); completeness is a measured recall figure (`minPatchFrac` default 1/32 is
    the recall/cost knob). Files: `src/native/ssi/{seed.h,patch_bounds.h,seeding.h,seeding.cpp}` +
    `tests/native/test_native_ssi_seeding.cpp` + `tests/sim/native_ssi_seeding_{recall,parity}.mm`.
    Living change `openspec/changes/add-native-ssi-seeding`. See
    [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md) SSI-S2 result table and
    [`SSI-ROADMAP.md`](SSI-ROADMAP.md).
  - **SSI Stage S3 (marching-line tracer / WLine) â DONE at the verification bar (both
    gates, TRANSVERSAL); near-tangent / coincident / branch-point marching is S4 (honest).**
    From each S2 seed, walks the intersection curve: predictor `t = normalize(n1 x n2)` â adaptive
    step â **corrector** re-projecting each node onto BOTH surfaces via the numerics substrate
    (`least_squares`, m=n=4 well-posed with an along-tangent advance residual, clamped to range) â
    march both directions + stitch â close (`Closed`) / exit a boundary (`BoundaryExit`) â dedup
    retraced branches â fit a clamped-uniform B-spline. OCCT-free in
    `src/native/ssi/{marching.h,marching.cpp}` (`cybercad::native::ssi`); corrector / adaptive step /
    B-spline fit guarded by `CYBERCAD_HAS_NUMSCI` (`marching.cpp` is an EMPTY TU with NUMSCI off);
    INTERNAL (no `cc_*`). Consumes the S2 `SeedSet`, produces a `TraceSet` of `WLine`s (each node
    carries (u1,v1,u2,v2) on both surfaces) â the S5 input contract. Both gates green: host
    `test_native_ssi_marching` (**7 cases, 0 failed** â crossing spheres / plane∩sphere / skew-cyl â
    Closed; ramp B-spline∩plane â `BoundaryExit`; tangent spheres â no curve (deferred, not faked);
    duplicate seed â 1 WLine; every node on both surfaces < 1e-6, fit error < 1e-3; NUMSCI OFF
    CTest **23/23** with the three NUMSCI-gated tests correctly ABSENT, NUMSCI ON CTest **26/26**
    adding `test_native_numerics` (#24), `test_native_ssi_seeding` (#25), `test_native_ssi_marching`
    (#26)) + sim native-vs-OCCT `IntPatch` / `GeomAPI_IntSS` **curve parity**
    (`tests/sim/native_ssi_marching_parity.mm`): **5 pairs, 9 branches, 0 failed â all TRANSVERSAL
    fully-traced, 0 near-tangent-truncated**; branch counts match OCCT on every pair; **5/5 OCCT
    closed loops reproduced as Closed native WLines** (bspline∩plane correctly 0-closed / 4-open).
    Worst deltas: max on-OCCT-curve **1.60e-06**, max on-surface **6.81e-07** (both skew-cyl-unequal),
    max length delta **2.28e-03** abs / ~0.33% rel (bspline∩plane, within the deflection/step tol).
    No regressions (`run-sim-suite.sh` **221/221**; `marching.cpp` additive/guarded, empty TU in the
    default build, `CMakeLists.txt` only APPENDS the test under the existing `CYBERCAD_HAS_NUMSCI`
    block, `native_ssi_marching_parity.mm` carries its own `main()` on the SKIP list). **Honest scope:**
    TRANSVERSAL only â near-tangent branches are traced *up to* the tangent, marked `NearTangent`,
    counted in `nearTangentGaps` (never a point past it); coincident / branch-point / self-intersection
    marching is deferred to **S4** (the moat). `nearTangentGaps > 0` is the honest S4 hand-off signal.
    (**Update:** S4-c since MARCHES THROUGH a `NearTangentTransversal` single-branch graze — see the
    S4-c bullet above; branch / coincident / singular regions still defer here.)
    Files: `src/native/ssi/{marching.h,marching.cpp}` + `tests/native/test_native_ssi_marching.cpp` +
    `tests/sim/native_ssi_marching_parity.mm`. Living change `openspec/changes/add-native-ssi-marching`
    **archived**. See [`docs/STATUS-phase-4.md`](../docs/STATUS-phase-4.md) SSI-S3 result table and
    [`SSI-ROADMAP.md`](SSI-ROADMAP.md) (S4 robustness + S5 curved booleans remain).
  - **SSI Stage S4-a + S4-b (coincident-region detection + tangent-contact
    CLASSIFICATION) + S4-c FIRST MARCHING-CORE SLICE — DONE at the verification bar; the
    deeper marching core (S4-d…f) is the remaining moat tail (honest).** S4-a/b are
    DETECTION + CLASSIFICATION layers: they TYPE the degeneracy and emit the point/curve/
    region where determinable. S4-c is the first slice that MARCHES THROUGH a tangency when
    the curve genuinely continues (a `NearTangentTransversal` single-branch graze), verified
    node-by-node on both surfaces vs OCCT — see the S4-c bullet below. **S4-a** — robust
    coincidence on both the analytic and seeded paths + a typed
    `CoincidentRegion` (`FullSurfaceSame` closed-form for all elementary families: plane,
    coaxial-equal cyl/cone, same sphere, same torus; seeded `OverlapSubRegion` with
    delimited param bounds via grid-agreement + boundary growth; `Undecided` → OCCT when
    the region cannot be robustly delimited). **S4-b** — a typed `TangentContact` replacing
    the blunt `SeedSet.deferredTangent` counter: `TransversalOnly` / `TangentPoint`
    (isolated, emits the point) / `TangentCurve` (tangent along a curve, emits it) /
    `NearTangentTransversal` (grazes-and-crosses → S4-c gap, handed on) / `Undecided`;
    analytic configs decided in closed form, seeded solutions by the relative second
    fundamental form `H = II_A − II_B` (sign-definite → point, rank-1 → curve, indefinite →
    near-tangent, within the model-scale curvature-noise band → undecided, never hand-tuned).
    Marching (`WLine`) carries an additive typed `stopReason` at a `NearTangent` stop — the
    tracer still STOPS at the tangency, never steps through. OCCT-free in
    `src/native/ssi/{coincidence.h,same_surface.h,tangent_contact.h,tangent_analytic.h,
    tangent_seeded.h}` (`cybercad::native::ssi`); the seeded-path parts guarded by
    `CYBERCAD_HAS_NUMSCI`; INTERNAL (no `cc_*`). Both gates green: host
    `test_native_ssi_s4_classification` (**14 analytic + 8 seeded cases, 0 failed**; NUMSCI
    OFF CTest **26/26** with the 8 seeded cases correctly ABSENT, NUMSCI ON CTest **31/31**)
    + sim native-vs-OCCT classification parity (`tests/sim/native_ssi_s4_classification_parity.mm`,
    `scripts/run-sim-native-ssi-s4.sh`): **8 pairs, 0 failed, 0 deferred** — `FullSurfaceSame`
    ↔ `IntAna_Same`, `TangentPoint` ↔ `IntAna_Point`, `TangentCurve` ↔ tangent Line/Circle,
    `TransversalOnly` ↔ proper section; emitted point/curve on both surfaces ≤ ~1e-16.
    No regressions (`run-sim-suite.sh` **221/221**, all six pre-S4 parity scripts green,
    S5 `native-pass=5` persists; additive/guarded, tessellator byte-identical). **Honest
    scope:** the opposite-saddle patch (indefinite relative II) types `NearTangentTransversal`
    → S4-c → OCCT, and a matched-curvature contact below the curvature-noise floor types
    `Undecided` → OCCT — both asserted in the seeded suite, never faked. Files:
    `src/native/ssi/{coincidence.h,same_surface.h,tangent_contact.h,tangent_analytic.h,
    tangent_seeded.h}` + `tests/native/test_native_ssi_s4_classification.cpp` +
    `tests/sim/native_ssi_s4_classification_parity.mm` + `scripts/run-sim-native-ssi-s4.sh`.
    Living change `openspec/changes/add-native-ssi-s4-classification` **archived**
    (`2026-07-04`). See [`SSI-ROADMAP.md`](SSI-ROADMAP.md) (S4-d…f marching-core remain).
  - **SSI Stage S4-c (near-tangent MARCH-THROUGH) — FIRST HONEST SLICE DONE at the
    verification bar (both gates).** The hard core of the moat: MARCH THROUGH a
    near-tangency **when the curve genuinely continues** instead of truncating, verified vs
    OCCT `GeomAPI_IntSS`. Additive to `marching.cpp`, gated `CYBERCAD_HAS_NUMSCI`; no `cc_*`.
    Four levers: **(1) fixed-plane-cut corrector** — inside the band the S3 along-`t`
    advance residual (`t = normalize(nA×nB)` ill-conditions as `sine → 0`) is replaced by a
    cut on the plane perpendicular to the **last-good forward tangent `t★`**, so the
    `least_squares` solve stays well-posed where the local surface tangent degenerates;
    **(2) curvature-aware predictor** — bends `P + h·t★` by the discrete two-node curvature
    so the corrector starts in-basin across the sharp bend; **(3) fine deflection-bounded
    step** through the low-sine band (capped `h₀/16`, `minStep` floor, `crossMaxSteps`
    budget) so it RESOLVES the region rather than leaping it; **(4) crossable gate (honesty
    core)** — crosses ONLY a `NearTangentTransversal` single-branch graze; a steep-sine-
    collapse witness (stall sine < ¼ last-good) OR a band-minimum-floor scan (fine
    look-ahead min sine < `0.3·tangentSinTol`) forces a DEFER, so a branch saddle / genuine
    `TangentPoint`/`TangentCurve`/`Undecided` STILL stops + classifies + defers → OCCT. No
    point is fabricated past a degeneracy: a crossed arc is emitted only if EVERY node
    verified on both surfaces ≤ `onSurfTol`, else the whole arc is discarded (rollback).
    **At the bar:** a sphere grazed by an offset cylinder that S3 TRUNCATES at
    `tangentSinTol=0.25` now traces the FULL closed loop (`nearTangentGaps → 0`,
    `nearTangentCrossed = 22` nodes, every node on both surfaces ≤ 1e-6, crossed arc on the
    OCCT locus onCurve ≤ 5.6e-5 / onSurf ≤ 1.3e-5 / crossResid ≤ 4.1e-11); the equal-radius
    orthogonal-cylinder **saddle (a branch crossing) STILL DEFERS** (`nearTangentCrossed=0`,
    `nearTangentGaps ≥ 1`), as do genuine `TangentPoint`/`TangentCurve` contacts. All 5 S3
    transversal fixtures trace bit-identically (corrector/step outside the band unchanged).
    Both gates green: host `test_native_ssi_marching` (**10 cases, 0 failed**; NUMSCI OFF
    CTest **26/26**, NUMSCI ON **31/31**) + sim `scripts/run-sim-native-ssi-s4c.sh`
    (**7 passed, 0 failed** — `nt-cross s4c` crossed, `eq-cyl defer` deferred, 5 transversal
    pairs `nt=0`). No regressions (S5 `native-pass=5` persists, tessellator byte-identical).
    **Honest scope — what S4-c does NOT do:** deeper near-coincident bands, branch crossings
    (S4-d), singularities (S4-e), self-intersection repair (S4-f), and any near-tangent
    region not robustly crossable stay an honest `NearTangent` gap deferred to OCCT.
    Files: `src/native/ssi/marching.{h,cpp}` +
    `tests/native/test_native_ssi_marching.cpp` +
    `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-s4c.sh`.
    Living change `openspec/changes/add-native-ssi-s4c-near-tangent-marching` **archived**
    (`2026-07-04`).
- â **`#4b` Tier E â native `cc_wrap_emboss` â DEFERRED (FUTURE WORK, not scheduled
  yet).** This is the *native* (OCCT-free) rewrite of wrap-emboss; it is distinct from
  the Phase-3 `add-robust-wrap-emboss` change, which is â done and OCCT-backed (the
  app-facing behaviour already works). Native wrap-emboss needs three pieces:
  (1) native project-a-2D-pattern-onto-a-surface into the target face UV domain,
  (2) native offset-along-normal by the emboss depth, and (3) a boolean merge of the
  raised/recessed region with the base solid. Step (2) is now **unblocked by #6** â the
  native planar `cc_offset_face` (slide a planar face along its normal + drag the side
  faces, EXACT slab, self-verified vs OCCT) is exactly the planar offset-along-normal a
  PLANAR-face emboss/deboss needs; step (3) is **partially unblocked by #5** (a
  planar-polyhedron emboss/deboss can use the native BSP-CSG fuse/cut, and â with the
  curved analytic slice now archived â an AXIS-ALIGNED-CYLINDER target can use the native
  box-cylinder curved fuse/cut). So a **planar-target wrap-emboss is now reachable
  natively** (native offset from #6 + native planar boolean from #5), the **boolean step
  of an axis-aligned-cylinder-target wrap-emboss is now also native** (curved analytic
  slice of #5), and only a **general curved-surface** wrap-emboss still waits on the
  **general curved native-boolean slice of #5** plus a **curved-surface native offset** (the
  planar slice of #6 does not offset curved faces). Native wrap-emboss remains sequenced
  AFTER #6, as its own OpenSpec change (`/opsx:propose`) â the planar slice can be
  proposed now that #6 landed; the curved slice waits on curved #5 + curved offset. Until
  then `cc_wrap_emboss` stays OCCT-fallthrough (labelled, verified). Robust-watertight
  `cc_helical_thread`/`cc_tapered_thread` are **NO LONGER deferred** â the mesher
  shared-edge weld (edge_mesher `CanonicalEndpoints` / face_mesher `BoundaryAnchors`) is
  DONE, so a well-formed helical / tapered thread now meshes `boundaryEdges==0` at EVERY
  deflection in the `robustlyWatertight` ladder and runs NATIVE (see the Tier D entry
  above; `test_native_thread` asserts the hard multi-deflection watertight ladder and
  `test_native_engine::native_thread_runs_native_watertight` asserts the op runs native
  through the facade). Several other residual #4b natives are now DONE by the
  geometry-completion batch above â **kind-3 SPLINE profile edge extrude, off-axis-arc TORUS
  revolve, N-section (3+) ruled loft, and a NON-PLANAR (RMF) sweep are now NATIVE.** What is
  still OCCT-fallthrough are the cases that genuinely need SSI / Tier-4 (surface-surface
  intersection + trimming): the accumulating-twist/scale `cc_twisted_sweep`, the guided/rail
  cases (`cc_guided_sweep` / `cc_loft_along_rail` / hard-rail loft), a mismatched-count /
  non-planar loft, a truly self-intersecting sweep or thread, a general SPLINE
  surface-of-revolution, and a spindle torus.
- â **#6 `native-blends` â tractable PLANAR slice done at the verification bar (both
  gates green); curved / concave / variable / fillet_face OCCT-fallthrough (honest).** Native
  `cc_chamfer_edges` / `cc_fillet_edges` (constant radius) / `cc_offset_face` /
  `cc_shell` for the tractable planar cases, built OCCT-free under
  `src/native/blend/` (`blend_geom.h`, `chamfer_edges.h`, `fillet_edges.h`,
  `offset_face.h`, `shell.h`, aggregate `native_blend.h`). Each op edits the solid's
  oriented-planar-polygon soup (the boolean's `extractPolygons`) and re-welds a
  watertight solid via the boolean's `assembleSolid` (T-junction repair + triangulate
  + weld), so it meshes by the SAME path a native prism / boolean does; the engine
  then runs a MANDATORY self-verify (`blendResultVerified` â watertight + sane volume
  sign: chamfer/fillet/shell REDUCE volume, offset GROWS for +distance / shrinks for
  âdistance) and DISCARDS a bad candidate â OCCT (never a wrong/leaky/faked solid).
  Native: **chamfer** slices the convex corner off with the plane through the two
  setback lines (EXACT vs OCCT for a box corner â 10Â³ edge d=2 â vol 980);
  **fillet** replaces a convex planar-dihedral edge with the rolling-ball tangent
  cylinder (the Phase-3 dihedral construction â axis â¥ crease, radius r, seated
  tangent to both planes: C = E â r/(1+n1Â·n2)Â·(n1+n2), tangent lines Ti = C + rÂ·ni),
  tiled into deflection-bounded facets (vol 991.4, BETWEEN the sharp 1000 and the
  chamfer 980, watertight); **offset_face** slides a planar face along its normal
  dragging the side faces (EXACT slab â +5 â 1500, â4 â 600); **shell** insets the
  kept walls inward by thickness and native-BSP-cuts the cavity (open-top box t=1 â
  wall vol 424). Gate 1 GREEN â host `test_native_blend` (10 cases: chamfer box /
  2-edge exact + degenerate/curved fallthrough; fillet watertight-between +
  curved/degenerate fallthrough; offset grow/shrink exact; shell wall exact +
  oversize fallthrough; concave L-prism edge â NULL while a convex edge of the same
  prism still lands native) + 5 new `test_native_engine` facade cases (native
  chamfer/fillet/offset/shell through `cc_set_engine(1)` + a variable-radius
  deferral + a native `cc_edge_polylines` regression case) host CTest **18/18**. STILL
  OCCT-fallthrough (native builder
  returns NULL / self-verify discards â forwarded or honest error, never faked):
  CURVED-face inputs, CONCAVE edges, variable-radius `cc_fillet_edges_variable`,
  `cc_fillet_face`, an edge shared by â 2 faces, multi-edge fillet interference,
  non-convex shell, oversized thickness. Blend functions are ð¢ Excellent (â¤10)
  except the two op drivers `fillet_edges` (13) / `chamfer_edges` (11) in the
  ð¡ Acceptable band (systems-band per-edge loop, flagged). Gate 2 (sim native-vs-OCCT
  parity, `native_blend_parity.mm` vs BRepFilletAPI/BRepOffsetAPI) GREEN â **`[NBLEND]`
  16 passed / 0 failed** through the `cc_*` facade under `cc_set_engine(0/1)`: chamfer
  (vol o=995 n=995 **rel 2.29e-16**) / offset (1500, rel 4.55e-16) / shell (424, rel
  4.02e-16) EXACT + watertight, constant-radius fillet deflection-bounded (o=997.854
  n=997.765 rel 8.96e-05, watertight), the curved-rim fillet forwarded to OCCT
  (`[fallback]` rel 0.00e+00), and the self-verify guard rejecting a thickness-6 shell on
  a 10Â³ box (id 0, honest error). **Root-cause fix:** the NativeEngine had no native
  `edge_polylines` â a native body's edges were unqueryable (the op refused a native
  body), so `findAxisEdge` in the sim harness resolved edge id 0 and `cc_chamfer_edges`
  / `cc_fillet_edges` always returned 0. `NativeEngine::edge_polylines` now discretizes
  each edge (in `mapShapes(Edge)` 1-based order, matching `subshape_ids` / the blend
  ops' edge lookup) via the shared `EdgeCache`, so native-body edges are pickable exactly
  as OCCT-body edges are. On `run-sim-suite.sh`'s SKIP list (own `main()`); 221/221
  re-verified.
- â **#7 `native-exchange` â native STEP EXPORT slice DONE at BOTH gates (host +
  sim OCCT re-read round-trip); STEP import + IGES stay OCCT (honest end state, out
  of scope).** `cc_step_export` is NATIVE
  (engine-wired behind the same `cc_set_engine(1)` toggle) for a native-built solid
  whose every face surface + edge curve is in the writer's scope: it walks the
  native B-rep (src/native/topology) and emits a valid ISO-10303-21 STEP AP203 file
  in true MILLIMETRES â the HEADER (FILE_DESCRIPTION / FILE_NAME /
  FILE_SCHEMA 'CONFIG_CONTROL_DESIGN') + the Part-42 DATA graph (CARTESIAN_POINT /
  DIRECTION / AXIS2_PLACEMENT_3D, VERTEX_POINT, LINE / CIRCLE /
  B_SPLINE_CURVE_WITH_KNOTS + EDGE_CURVE, ORIENTED_EDGE â EDGE_LOOP,
  FACE_OUTER_BOUND / FACE_BOUND, PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE /
  SPHERICAL_SURFACE / B_SPLINE_SURFACE_WITH_KNOTS, ADVANCED_FACE â CLOSED_SHELL â
  MANIFOLD_SOLID_BREP, wrapped in ADVANCED_BREP_SHAPE_REPRESENTATION + the mm
  SI_UNIT geometric context + PRODUCT / PRODUCT_DEFINITION / APPLICATION_CONTEXT
  boilerplate). Built OCCT-FREE under `src/native/exchange/` (`step_writer.h/.cpp`,
  aggregate `native_exchange.h`, entry `step_export_native(solid, path)`) on the
  #1â#6 topology/math foundation. The native builders emit PER-FACE edges (edge-node
  sharing deferred, NATIVE-REWRITE.md #4), so the writer DEDUPLICATES geometrically â
  coincident vertices collapse to one VERTEX_POINT and the two faces meeting at a
  physical edge share ONE EDGE_CURVE (used forward on one face, reversed on the
  other via ORIENTED_EDGE) â producing a properly-sewn manifold CLOSED_SHELL that
  re-reads as a solid, not a heap of coincident faces. **Native-else-OCCT wiring
  (honest):** `NativeEngine::step_export` runs native for a native body IN SCOPE;
  a native body OUT of scope (an unsupported geometry kind) returns a clean error
  (never a native void handed to OCCT); a NON-native (OCCT-built) body forwards to
  `STEPControl_Writer`. **`cc_step_import` STAYS OCCT** (parsing arbitrary STEP is
  the huge part, out of scope) and **`cc_iges_export/import` STAY OCCT** â that is
  the honest end state (#8 drop-occt stays blocked on import + IGES + curved/general
  booleans). No cc_* ABI change; default engine stays OCCT. Entity arg orders were
  cross-checked against the OCCT `RWStep*` writer modules (EDGE_CURVE / ADVANCED_FACE
  / CIRCLE / LINE / VECTOR / ORIENTED_EDGE / B_SPLINE_CURVE_WITH_KNOTS all match) so
  the file parses through `STEPControl_Reader`. Gate 1 (host, no OCCT) GREEN â host
  `test_native_step_writer` (6 cases: canSerialize scope boundary; box â valid
  AP203 header + wrapper + mm SI_UNIT; box geometry 6 PLANE / 12 shared EDGE_CURVE /
  8 VERTEX_POINT; cylinder â CYLINDRICAL_SURFACE + CIRCLE rims; every DATA line a
  well-formed contiguous `#n = ENTITY(...);`; coordinates emitted as STEP REALs) +
  `test_native_engine::native_step_export_writes_valid_ap203_file` (the facade
  `cc_step_export` runs native on a native box, returns 1, writes a file with the
  ISO magic + MANIFOLD_SOLID_BREP + mm SI_UNIT); host CTest **21/21**, all native
  suites green. All writer functions ð¢ Excellent (â¤ 7 cognitive complexity), no
  systems-band function. **Gate 2 (sim OCCT re-read parity) GREEN** â
  `tests/sim/native_step_parity.mm` + `scripts/run-sim-native-step.sh` through the
  `cc_*` facade: **`[NSTEP]` 28 passed / 0 failed** â each native STEP file re-reads
  through `STEPControl_Reader` to the SAME solid as its source (box EXACT vol 1000 /
  6 faces / 24 edges; cylinder vol rel 1.27e-3, 9 faces; holed-plate vol rel 2.90e-4,
  7 faces, valid), the native-written and OCCT-written files re-read to EQUIVALENT
  solids (writer-parity rel â¤ 4.7e-15), and a FOREIGN (OCCT-built) body forwards to
  `STEPControl_Writer` (fall-through, active native). **Two writer bugs fixed to reach
  this gate:** (1) EDGE_LOOP / ADVANCED_FACE emitted a stray extra empty string
  (`'',''`), giving EDGE_LOOP 3 args (schema 2) and ADVANCED_FACE 5 (schema 4) â OCCT
  rejected both and transferred an EMPTY solid (0 faces, vol 0); (2) a full-turn
  periodic wall (a cylindrical hole wall) was emitted as a periodic surface trimmed to
  its full period with NO seam edge, which OCCT reads back with zero wall area (a
  leaky, invalid solid) â the writer now synthesises the required SEAM edge (a straight
  LINE used forward at u=period and reversed at u=0, mirroring `STEPControl_Writer`'s
  cylindrical-hole-wall representation) for any Cylinder/Cone/Sphere face whose loop is
  closed full-circle rim edges. Both are pinned by host regression tests
  (`edge_loop_and_advanced_face_have_schema_arg_counts`,
  `cylindrical_hole_wall_emits_seam_edge`). The sewn re-read solid legitimately gains
  one seam edge per periodic wall the native deferred-edge-sharing source omits
  (native src 28 â re-read 30 for the holed plate, matching OCCT's own writer), which
  the harness edge-count check now accepts as a bounded superset. STILL OCCT (never
  faked): STEP IMPORT, IGES import/export, and a
  native solid with an out-of-scope geometry kind (Ellipse/Bezier curve, rational
  spline, Bezier surface) â OCCT fallback for an OCCT body / honest error for a
  native void. Living change `add-native-data-exchange` **archived** (validate
  --strict green). This is the native EXPORT slice only â import + IGES stay OCCT by
  design.
- â #8 `drop-occt` â planned; **NOT reachable** at the current native ceiling. Two
  hard, research-grade multi-year dependencies remain: (1) a general robust curved
  boolean / blend kernel (arbitrary surface-surface intersection + shape healing) and
  (2) native STEP/IGES IMPORT (a full AP203/AP214 + IGES parser and B-rep
  reconstructor â the #7 slice was EXPORT only). Until BOTH exist, OCCT stays linked
  and Phase 4 stands COMPLETE AT ITS ACHIEVABLE NATIVE CEILING, not fully drop-OCCT.

## Remaining work to drop OCCT â ordered by difficulty (+ effort)

As of the geometry-completion batch, **every tractable/analytic capability is
native**; what remains is the SSI-gated hard core + the exchange/healing track.
Ordered easiest â hardest by *marginal* effort (deps flagged; a cheap item gated
on a hard prereq is not reachable early). Effort is **order-of-magnitude,
robustness-dominated, human-expert-equivalent** â `w` = weeks, `py` = person-years.
LOC are OCCT's (the port/reference size).

| # | Remaining item | OCCT LOC | Dep | Analytic slice | Production-robust |
|---|---|---|---|---|---|
| 1 | Twist/scale sweep, guided/rail sweep+loft, mismatched loft (self-verify-clean cases) | ~48k (TKOffset/BRepFill) | frame math | ~1â2 w | 0.5â1.5 py |
| 2 | ~~**Numeric foundations**~~ â **DONE at the bar.** `math_` solvers (Newton/FunctionSetRoot/BFGS) + `Extrema` (45k) + `Adaptor3d` (7k). **NumPP + SciPP ADOPTED** as the OCCT-free substrate (`add-native-numerics`, archived); generic solvers + native closest-point / projection are NATIVE + verified vs OCCT `Extrema` (22/22 `[NNUM]`, dDist â¤ 1.776e-15); SSI stays #5 | ~55k | â | done | **~0.15â0.35 py REALIZED** (was 0.5â1 py) â *~60â75% saving banked; on-ramp to everything below now native* |
| 3 | STEP/IGES **import** (full AP203/214/242 + IGES parse + reconstruct) | ~300â600k | needs #4 | narrow (own export): ~w | 2â4 py |
| 4 | **Shape healing** (`ShapeFix`/`ShapeUpgrade`/`ShapeAnalysis`) | 87,647 | â | n/a | 2â4 py |
| 5 | **SSI + general curved booleans** (`IntPatch`/`IntWalk` 89k + BOPAlgo 76k). **SSI-ROADMAP S1 analytic + S2 subdivision seeding DONE at the bar** (S1: 17 elementary pairs vs OCCT `GeomAPI_IntSS`, `add-native-ssi-analytic` archived; S2: transversal branch recall 1.00 on freeform/skew-quadric pairs, seeds on both surfaces ≤ 3.51e-16); **S3 marching-line tracer is NEXT** | ~165k | #2 | ~w/case | **3â6 py** (clean-room) / ~1.5â3 py (port from OCCT) â *the moat* |
| 6 | Curved / variable-radius / fillet-face / concave **blends** (`ChFi3d`) | 95,710 | #5 | ~w/case | 2â4 py |
| 7 | Curved **wrap-emboss** | (composition) | #5 + curved offset | ~days | 0.2â0.5 py |
| 8 | `drop-occt` â unlink + full regression | â | 1â7 | â | small, last |

Critical path: **#2 numeric foundations (DONE) â #5 SSI (NEXT) â curved booleans â
#6 blends â #7 wrap-emboss**, with **#3 import â #4 healing** as a parallel track.
Both gate **#8**. **#2 is done** â NumPP/SciPP adopted, generic solvers + native
closest-point verified vs OCCT `Extrema` â so the next critical-path item is **#5 SSI**: its **S1 analytic + S2 subdivision seeding are now DONE**
(S1: 17 elementary pairs closed-form + verified vs OCCT `GeomAPI_IntSS`, `add-native-ssi-analytic`
archived; S2: transversal branch recall 1.00 on freeform/skew-quadric pairs, seeds on both
surfaces ≤ 3.51e-16, `add-native-ssi-seeding`), and what remains is
**S3 marching-line tracer (NEXT, consuming the S2 seeds)** → **S4 tangent-robustness (near-tangent
seed) layer** on top of the substrate (the moat NumPP/SciPP does not buy), feeding the S5
curved-boolean payoff. Total to genuinely drop OCCT â **10â20 py** (a small team, several years); matching
OCCT means re-earning its person-decades of hardening on real CAD data.

> **Numeric-substrate decision (NumPP/SciPP): ADOPTED â GO-WITH-HARDENING â DELIVERED at
> the verification bar.** NumPP + SciPP
> (the org's C++20, MIT NumPy/SciPy ports) are **adopted as the kernel's OCCT-free numeric
> substrate** for #2, referenced by absolute path exactly like OCCT (NOT vendored),
> CPU-only, consuming the SciPP `optimize`/`linalg`(+`spatial`/`integrate`) subset with
> `special`+`stats` EXCLUDED (a Homebrew-libc++ ISO-29124 gap, confined to `src/special/`,
> unused by the kernel). This retires ~60â75% of #2 (â ~0.15â0.35 py): the generic solvers
> (root/`fsolve`/BFGS/`least_squares`/`solve`/`lstsq`) plus **native closest-point /
> projection (`Extrema` on-ramp)** land native under `src/native/numerics/`, guarded by
> `CYBERCAD_HAS_NUMSCI` so the rest of `src/native` builds without them. It retires only
> ~25â35% of #5's *numeric* slice; **the SSI moat stays** â near-tangent SSI (EXP2b
> naive-seed 0/7, EXP2c both-solver-fail) is NOT bought by these libraries and remains #5.
> Change: `openspec/changes/add-native-numerics`. Eval:
> [`docs/EVAL-numpp-scipp.md`](../docs/EVAL-numpp-scipp.md).

### Effort banked so far (human-expert-equivalent)

The native rewrite delivered the entire analytic/tractable surface, verified vs
OCCT: math, topology, watertight tessellation, construction (extrude/revolve/
holed+typed profiles/spline+torus/N-loft/planar+non-planar sweep/threads/shank),
planar + axis-aligned boxâ©cylinder booleans, planar blends, STEP export, and the
**numeric foundations (#2) â NumPP/SciPP adopted as the OCCT-free substrate + native
closest-point/projection verified vs OCCT `Extrema`**. Cumulative â **0.6â0.9 py** of
skilled kernel work, plus the ~**0.15â0.35 py** #2 slice bought largely by adopting
NumPP/SciPP rather than hand-writing the solver layer (a ~60â75% saving on #2).

- **Last batch (geometry-completion) gained â 3â5 person-weeks** â spline edges,
  torus revolve (+ native `Torus`), N-section loft, non-planar RMF sweep â and
  proved the boundary (declines/fallbacks correctly identified, not faked).
- It did **not** shorten the ~10â20 py drop-OCCT tail: that tail is the SSI +
  healing + import hard core, orthogonal to this construction work. By *effort*
  we are ~5â8% of the way to full drop-OCCT; by *tractable-capability breadth*,
  ~complete. The remaining 90%+ is the research-grade core.

Progress is reflected in [ROADMAP.md](ROADMAP.md) Phase 4 and per-change
`tasks.md`; living specs are synced/archived per capability as they pass the
verification gates.

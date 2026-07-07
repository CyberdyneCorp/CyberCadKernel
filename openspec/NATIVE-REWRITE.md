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
slice AND the first native STEP IMPORT slice (#7). Phase 4 is therefore **COMPLETE AT ITS ACHIEVABLE NATIVE CEILING, not
fully drop-OCCT**. The COMPLETE remaining path to drop OCCT — every research-grade moat
stage (M0 freeform mesher/trimmer keystone → M2/M3 freeform booleans/blends → M4 general
import → M5 healing → M6 completeness bar → M8 unlink), dependency-ordered and each verified
native-vs-OCCT with OCCT retained as the oracle until proven — is
[MOAT-ROADMAP.md](MOAT-ROADMAP.md). In brief: #8 `drop-occt` is BLOCKED because two hard dependencies remain
research-grade multi-year efforts â (1) a general robust curved boolean / blend
kernel (arbitrary surface-surface intersection + shape healing) and (2) native
a FULL native STEP IMPORT (the native reader covers the elementary/B-spline
subset the writer produces, foreign OCCT-written box/cylinder, flat multi-solid
compounds, single-level RIGID/UNIFORM-SCALE/MIRROR placed assemblies, AP242
geometry with PMI skipped, TRIMMED_CURVE edges over an in-slice basis
(LINE/CIRCLE/ELLIPSE/B-spline — the B-spline basis honoring its knot-span trims),
AND a SURFACE_OF_REVOLUTION reducing to the matching native analytic quadric per its
generatrix — a straight LINE ∥ axis → an exact native CYLINDER, an OBLIQUE line meeting
the axis → a native CONE (apex on axis, half-angle from the line-axis angle), a line ⟂ axis
→ a native PLANE annulus cap (all three import NATIVELY, watertight, at OCCT parity within
faceting tol), plus an on-axis CIRCLE/arc → SPHERE reduction — including a FULL sphere: OCCT
writes a whole sphere as ONE SPHERICAL_SURFACE (or the on-axis-circle SURFACE_OF_REVOLUTION
form) ADVANCED_FACE bounded by a VERTEX_LOOP (a single degenerate pole vertex, NO seam/pole
EDGE_CURVEs — a bare periodic surface); the reader now maps that VERTEX_LOOP bound to a
native Sphere face with a NULL outer wire, which the tessellator meshes over its natural
(u∈[0,2π], v∈[−π/2,π/2]) bounds — welding the longitude seam + both poles → a WATERTIGHT
Sphere solid at OCCT parity (sim: native parsed=1, ΔV≈2.7e-3; the STEP-import sim now `[NIMPORT]` 77/77). A
VERTEX_LOOP bound on any NON-sphere surface, or a partial spherical zone carrying real trim
edges that cannot close, keeps the honest OCCT deferral — schema-independent, AP203/AP214/AP242
headers all accepted. A full **TOROIDAL_SURFACE** (and the off-axis-circle SURFACE_OF_REVOLUTION OCCT
emits as one) now imports NATIVELY watertight as an additive `FaceSurface::Kind::Torus` (bare
doubly-periodic surface, both seams welded, no pole — sim `native torus` ΔV≈2.68e-3 vs OCCT, V=2π²Rr²;
the Torus mesh path reuses the sphere bare-periodic path additively, so `face_mesher.h`/`trim.h` are
untouched and every existing mesh is byte-identical). A GENERAL **SURFACE_OF_REVOLUTION** — an **ELLIPSE**
or a (non-rational) **B_SPLINE_CURVE** generatrix that touches the axis at both ends — now also imports
NATIVELY watertight: the reader revolves the profile meridian into the exact RATIONAL tensor-product
B-spline (Piegl & Tiller A7.1 — u = the standard rational-quadratic full circle, 9 poles/weights
{1,1/√2,…}; v = the ellipse promoted to two exact rational-quadratic 90° arcs, or the B-spline profile
used directly) and stores it as an additive `FaceSurface::Kind::BSpline` face WITH weights, meshed as a
bare periodic surface over its natural (u∈[0,2π], v=profile) bounds (u-seam welded, both axis poles
collapsed) — the UNMODIFIED tessellator's existing rational eval + sphere-style bare-periodic path, so
`face_mesher.h`/`surface_eval.h`/`trim.h` stay untouched and every existing mesh is byte-identical (sim
`revolution→ellipsoid` ΔV≈4.5e-3 vs OCCT, V=4/3·π·b²·a; `revolution→bspline` ΔV≈2.6e-3). Still an honest
DECLINE → OCCT: non-uniform/shear transforms, deep-nested assemblies, PMI SEMANTICS, a PARTIAL/trimmed
torus, and a SURFACE_OF_REVOLUTION with no faithful native kind — a tilted/off-axis ellipse, a rational
STEP profile, a SKEW oblique line (hyperboloid) — plus foreign arbitrary-rational B-spline surfaces and
general swept/bounded/offset surfaces still fall back to OCCT). **IGES import/export are DESCOPED
(STEP-only interchange): no native IGES will be built; the `cc_iges_*` ABI stays
OCCT-backed until `drop-occt`, then removed/stubbed, never reimplemented.** Until both
the curved kernel and general STEP import exist, OCCT stays linked.

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

> #5 SSI → curved-booleans implementation plan: see [SSI-ROADMAP.md](SSI-ROADMAP.md) (staged S1-S5, substrate #2 done; S1 analytic + S2 seeding + S3 marching done — the SSI curve pipeline is now NATIVE for transversal freeform/quadric pairs; **S5-a/b/c/d/e/f landed sixteen native curved-boolean sub-cases** verified vs OCCT `BRepAlgoAPI_{Fuse,Cut,Common}` — the through-drill cyl∩cyl COMMON (S5-a) + FUSE + CUT (S5-b), the sphere∩sphere op-set now COMPLETE 3/3 native COMMON + FUSE + CUT (S5-c, equal + unequal radii), the **branched-trace Steinmetz bicylinder op-set now COMPLETE 3/3 native COMMON + FUSE + CUT (S5-d)**, the **CONE surface family — coaxial cone(frustum)∩cylinder op-set now COMPLETE 3/3 native COMMON + FUSE + CUT (S5-e)**, and the **CONE∩SPHERE family — coaxial cone(frustum)∩sphere op-set now COMPLETE 3/3 native COMMON + FUSE + CUT (S5-f)** — all watertight, ΔV ≤ 9e-4 (sim `native-pass=18`); `ssi_boolean.{h,cpp}`, changes `add-native-ssi-curved-boolean` + `add-native-ssi-curved-boolean-wider` + `add-native-ssi-branched-boolean` (archived `2026-07-05`) + `complete-sphere-sphere-fuse-cut` + `complete-steinmetz-fuse-cut` (archived `2026-07-06`) + `add-native-cone-boolean` + `complete-cone-cyl-fuse-cut` + `add-native-cone-sphere-boolean` (archived `2026-07-07`). **S5-e** opens the CONE family: a shared `coneCylSetup` prologue gates a coaxial cone+cylinder with a SINGLE S1-analytic seam circle (`intersectCylinderConeCoaxial`, apex-free, `nearTangentGaps=0`) and resamples it to ONE pooled ring, from which all three ops build by fragment selection — `buildConeCylCommon` welds a frustum band to a cylinder-segment band (min-radius-profile solid of revolution) closed by two disc caps; `buildConeCylFuse` (A∪B) keeps both operands' OUTER wall regions + union caps + `appendAnnulusCap` step caps (`V=V(A)+V(B)−V(A∩B)`, a GROW); `buildConeCylCut` (A−B, cone minuend, order-sensitive) keeps A's outer wall + A's caps + the cylinder's inside-A band emitted REVERSED (inward normal, a disconnected detached-tip + conical-washer solid, `V=V(A)−V(A∩B)`, a SHRINK) — all `appendRevolvedBand` + `appendDiskCap`/`appendAnnulusCap` + `VertexPool`. Verified vs a DUAL oracle: the analytic inclusion-exclusion closed form (engine `ssiCurvedBooleanVerified` COMMON arm + generic `booleanResultVerified` `V(A)±…` for FUSE/CUT with native `buildConeCylCommon` as `V(A∩B)`, same 1% deflection-bounded tol as the Steinmetz `16 R³/3` oracle) AND OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim): COMMON volN=19.107 vs analytic 19.111355 vs OCCT 19.111 (ΔV=2.03e-04, ΔA=9.89e-05); FUSE volN=41.618 vs analytic 41.62610 vs OCCT 41.626 (ΔV=2.04e-04, ΔA=1.13e-04); CUT volN=13.349 vs analytic 13.35177 vs OCCT 13.352 (ΔV=2.03e-04, ΔA=1.02e-04), watertight/closed/valid, no tolerance weakened. The reversed `cyl−cone` CUT, transversal (non-coaxial) / apex-crossing cone pairs, and cone∩cone all decline → NULL → OCCT. **S5-f** extends the CONE family to the coaxial cone(frustum)∩sphere op-set (COMMON + FUSE + CUT): a shared `coneSphereSetup` prologue gates one `Cone` + one `Sphere` with its centre ON the cone axis on the FRUSTUM side, so `intersectSphereConeCoaxial` (a QUADRATIC) gives EXACTLY ONE interior crossing `s*` that does NOT cross the apex (`nearTangentGaps==0`, `curveCount==1`), resampled to ONE pooled ring; the cone side reuses the S5-e cone-wall split (`appendRevolvedBand` + `appendDiskCap`), the sphere side the sphere-lens cap builder (`appendSphereCap`, inner/outer apex + reversed flags). `buildConeSphereCommon` welds the cone band inside the sphere + the cone disc + the sphere INNER cap (`V=V_frustum+V_spherical-segment`); `buildConeSphereFuse` (A∪B) keeps the sphere OUTER cap + the cone OUTER wall + the cone disc (`V=V(A)+V(B)−V(A∩B)`, a GROW); `buildConeSphereCut` (A−B, cone minuend, order-sensitive) keeps A's outer wall + A's disc(s) + the sphere INNER cap emitted REVERSED — a CONNECTED frustum-with-spherical-dimple (`V=V(A)−V(A∩B)`, a SHRINK). Verified vs a DUAL oracle (engine `ssiCurvedBooleanVerified` COMMON arm + generic `booleanResultVerified` for FUSE/CUT with native `buildConeSphereCommon` as `V(A∩B)`, same 1% tol) AND OCCT `BRepAlgoAPI_{Common,Fuse,Cut}`: COMMON volN=5.2546 vs OCCT 5.2558 (ΔV=2.41e-04, ΔA=1.28e-04); FUSE volN=60.686 vs OCCT 60.718 (ΔV=5.22e-04, ΔA=2.61e-04); CUT (cone−sphere) volN=27.202 vs OCCT 27.207 (ΔV=1.96e-04, ΔA=1.34e-04). The sphere-minuend `sphere−cone` CUT, the TWO-circle crossing (sphere through the cone / spanning the apex), apex-crossing / apex-in-extent frustums, and TRANSVERSAL (non-coaxial) cone∩sphere (a quartic space curve) all decline → NULL → OCCT (`add-native-cone-sphere-boolean` archived `2026-07-07`). **S5-c FUSE/CUT** reuse one generalised `appendSphereCap(outer,reversed)`: FUSE (A∪B) = the two OUTER (far-pole) caps welded on the shared seam (`V=V(A)+V(B)−lens`); CUT (A−B, order-sensitive) = the OUTER cap of A + the INNER cap of B emitted REVERSED (inward normal, bounding the scooped cavity) (`V=V(A)−lens`) — verified vs BOTH the analytic closed forms AND OCCT (FUSE ΔV 6.5e-04 eq / 8.3e-04 uneq; CUT ΔV 7.0e-04 eq / 9.3e-04 uneq); the COMMON path is byte-identical (defaults `outer=false,reversed=false`); tangent/containment/concentric pairs decline → NULL → OCCT. **S5-d** is the branched-trace op-set (COMMON / FUSE / CUT): on the S4 decline edge, a `steinmetzPreGate` (equal-R, orthogonal, crossing cylinders) triggers a branch-enabled re-trace, `recogniseSteinmetzTrace` accepts only the canonical 2-branch-point / 4-`BranchArc` structure, and the shared lune/arc split + `VertexPool` weld drive all three ops: `buildSteinmetzCommon` welds the four inside-the-other lunes (`V = 16 R³/3`); `buildSteinmetzFuse` keeps both cylinders' OUTSIDE walls + all four caps (`V=V(A)+V(B)−V(common)`); `buildSteinmetzCut` keeps A's OUTSIDE wall + A's caps + B's lunes REVERSED (`V=V(A)−V(common)`). Verified vs **BOTH** the exact analytic inclusion-exclusion volumes (host) **and** OCCT `BRepAlgoAPI_{Common,Fuse,Cut}` (sim), all watertight/closed/valid inside the 1% bar, no tolerance weakened: COMMON volN = 5.3287, ΔV = 8.75e-04 (−0.088%); FUSE volN = 32.385 vs OCCT 32.366, ΔV = 5.82e-04; CUT volN = 13.526 vs OCCT 13.516, ΔV = 7.22e-04. A disjoint Steinmetz pair (no seam) declines → NULL → OCCT for all three ops (sphere∩sphere FUSE/CUT are also native — see S5-c above). S4-a/b classification + S4-c near-tangent march-through + **S4-d branch-point slice** (the Steinmetz self-crossing bicylinder localized + routed through both branch points vs OCCT `IntPatch`/`GeomAPI_IntSS` — 2 branch pts, 4 arms → 2 crossing ellipses; isolated tangent point still ends; `add-native-ssi-s4d-branch-points` archived) + **S4-e chart-singularity slice** (a marched curve crossing a sphere parametric pole `v=±π/2` or a cone apex — where ONE surface's own `(u,v)` degenerates while its point+normal stay finite — now FULLY traced vs OCCT `GeomAPI_IntSS`: sphere great circle `singX=2 len 6.2829 vs 6.2832`, cone apex `singX=1` both nappes, on-surface ≤ 1.5e-07; `add-native-ssi-s4e-singularities` archived `2026-07-05`). **S4-f COMPLETENESS + LOOP-ROBUSTNESS slice** (hardens curves already traced, adds no new capability): loop-closure is now a TRUE-RETURN test (return to seed AND tangent-continuous AND arc past the closure window) so a near-pass no longer FALSE-CLOSES; a self-intersection guard detects + traces THROUGH a single-arm figure-eight crossing as typed data (`branchPts=0`, distinct from S4-d); an adaptive completeness-critic re-seed recovers small loops the fixed 1/32 subdivision misses (fixture A recall 0.50→1.00 floor 1/128, adversarial many-loops D 0.25→1.00 floor 1/48, both vs OCCT `GeomAPI_IntSS`, every recovered node on both surfaces ≤ 1e-11) — **completeness is MEASURED per-fixture at the reached floor, NOT a proof (below ANY fixed resolution a smaller loop can still be missed); S4-f RAISES the recall floor and de-risks (does NOT unblock) curved blends #6 + wrap-emboss #7**; `add-native-ssi-s4f-completeness` archived `2026-07-05`. Remaining: **S4 general/freeform branch points + general/freeform + higher-order-cusp singularities + S4-f general small-loop residual (below the reached floor) + self-intersection arc-splitting/topology repair** + wider S5 coverage (general non-Steinmetz branched pairs, transversal/apex cone pairs + cone∩cone + the two-circle / apex-crossing / transversal cone∩sphere crossings, more curved-curved families — the equal-R orthogonal Steinmetz op-set is COMPLETE 3/3, and the CONE family's coaxial cone∩cylinder AND coaxial cone∩sphere single-crossing op-sets are now COMPLETE 3/3 native (COMMON/FUSE/CUT))).
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
  - **✅ `#4b` construction-breadth batch (T1/T2/T3) — MISMATCHED-count ruled loft
    LANDED NATIVE (exact); guided sweep (T2) and fine-pitch self-intersecting thread (T3)
    are HONEST DECLINES (still OCCT, no dead code).** Change
    `add-native-construction-breadth`, engine-wired behind the same `cc_set_engine(1)`
    toggle, behind the UNCHANGED `cc_solid_loft` / `cc_solid_loft_wires` ABI. **T1 NATIVE:**
    the old `size mismatch → NULL` guard in `build_ruled_loft_sections`
    (`src/native/construct/loft.h`) is replaced by `detail::equalizeSectionCounts` — both
    section loops are resampled at the sorted UNION of their normalized arc-length
    parameters (dedup within `kProfileTol`), so an M-gon and an N-gon become two equal-K
    loops (K ≤ M+N) that feed the existing ruled-band + cap builder unchanged; every
    original vertex survives and every inserted point is COLLINEAR (geometry preserved).
    The EQUAL-count path is byte-identical (short-circuited). `NativeEngine::solid_loft` /
    `solid_loft_wires` now self-verify `robustlyWatertight && watertightVolume > 0` and
    forward the SAME arguments to OCCT on any miss. Gate 1 (host, no OCCT): `test_native_loft`
    **21 cases / 0 failed** (4 new exact T1 fixtures — box 48, box 1000, frustum 653.33,
    N-section 4→8→4 spool 784, all < 1e-6 rel — plus a triangle→square correspondence-runs
    case). Gate 2 (sim vs OCCT `BRepOffsetAPI_ThruSections`): `run-sim-native-loft.sh`
    **21 passed / 0 failed** — mismatched **4→8** frustum vol o=56 n=56 **rel 1.61e-14**,
    area rel 7.18e-15, faces 10=10, watertight (meshVolRel 1.24e-14). The genuinely
    asymmetric **4→3** case FAILS the watertight self-verify and delegates to OCCT (vol
    o=40.1311 n=40.1311 **rel 0.00e+00**, native active=1) — the guard demonstrably works,
    no faked tolerance. **T2 (orientation-guided sweep) — HONEST DECLINE:** `sweep.h`
    BYTE-IDENTICAL (`git diff` empty); the shipped `cc_guided_sweep` oracle is the
    SCALE-splay ThruSections (already native), and an orientation-guide frame law has no
    oracle behind that fixed entry without an ABI/semantics break — stays OCCT-fallthrough,
    no builder, no dead code. **T3 (fine-pitch self-intersecting thread) — HONEST DECLINE:**
    `thread.h` BYTE-IDENTICAL (`git diff` empty); the `kMaxLeadRatio` guard stays. Crossing
    radial-V flanks are two intersecting helicoids — a single ruled tiling is non-manifold /
    volume-wrong and trimming needs Tier-4 SSI the watertight-only self-verify can't gate;
    stays OCCT-fallthrough, no dead code. Files changed:
    `src/engine/native/native_engine.cpp`, `src/native/construct/loft.h`,
    `tests/native/test_native_loft.cpp`, `tests/sim/native_loft_parity.mm`; controls
    (`sweep.h`, `thread.h`) unchanged. No regressions (host CTest 29/29 NUMSCI-OFF + 36/36
    NUMSCI-ON, `run-sim-suite.sh` 221/221). Living change
    `openspec/changes/add-native-construction-breadth` — archived to
    `openspec/changes/archive/2026-07-07-add-native-construction-breadth` →
    `openspec/specs/native-construction`. **The mismatched-count planar polygon loft is now
    native; the residual loft fall-throughs are the non-planar / punctual section, guided /
    rail loft, and a genuinely-asymmetric resampled cap that fails self-verify — plus the
    T2 guided sweep and T3 fine-pitch thread, all still OCCT (SSI / Tier-4).**
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
  near-tangent MARCH-THROUGH slice S4-c + the first BRANCH-POINT slice S4-d — the
  Steinmetz self-crossing bicylinder localized + routed — + the CHART-SINGULARITY slices
  S4-e — a curve crossing a sphere parametric pole / cone apex AND a FREEFORM NURBS collapsed-row
  pole (via the point-only `freeformChartInvert` far-side re-seed) now fully traced; the curve
  cusp declined by the IFT argument — + the
  COMPLETENESS + LOOP-ROBUSTNESS slice S4-f — robust TRUE-RETURN loop-closure (no false-close),
  a self-intersection guard (figure-eight crossing detected + traced-through as data), and an
  adaptive completeness-critic re-seed that recovers small loops the fixed subdivision misses
  (MEASURED recall floor per-fixture vs OCCT — RAISES the floor, NOT a proof: below the reached
  floor a smaller loop can still exist) — now DONE at the bar; the deeper marching core,
  general/freeform branch points + general/freeform + higher-order-cusp singularities + the
  S4-f general small-loop residual + self-intersection arc-splitting/topology repair, is the
  remaining tail), written on top of this
  substrate + the S3 tracer.** Change `add-native-numerics` **archived**. See
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
    (**Update:** S4-c since MARCHES THROUGH a `NearTangentTransversal` single-branch graze, S4-d
    routes the Steinmetz self-crossing branch, and S4-e crosses a sphere-pole / cone-apex chart
    singularity — see the S4-c/d/e bullets; general/freeform branch / coincident / singular
    regions + higher-order cusps still defer here.)
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
    (now handled for the transversal self-crossing family by S4-d, below), singularities
    (S4-e), self-intersection repair (S4-f), and any near-tangent region not robustly
    crossable stay an honest `NearTangent` gap deferred to OCCT.
    Files: `src/native/ssi/marching.{h,cpp}` +
    `tests/native/test_native_ssi_marching.cpp` +
    `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-s4c.sh`.
    Living change `openspec/changes/add-native-ssi-s4c-near-tangent-marching` **archived**
    (`2026-07-04`).
  - **SSI Stage S4-d (BRANCH POINTS — self-crossing locus) — FIRST HONEST SLICE DONE at the
    verification bar (both gates).** The hardest SSI piece: where the intersection LOCUS
    itself crosses (multiple curve arms meet at one point), LOCALIZE the branch point,
    ENUMERATE the outgoing arms, ROUTE each and ASSEMBLE the multi-arm curve — verified vs
    OCCT `IntPatch`/`GeomAPI_IntSS`. Additive to `marching.cpp` + new `branch_point.h`, gated
    `CYBERCAD_HAS_NUMSCI`, default-on `enableBranchPoints`; no `cc_*`. Fires **exactly where
    S4-c would have deferred** (the steep-sine-collapse + tangent-flip witness). Four steps:
    **(1) LOCALIZE** — `nn::minimize` the transversality sine `g(s) = ‖nA×nB‖` along the
    bracketed approach (each trial re-projected onto both surfaces with the S4-c fixed-plane
    corrector), then a full `nn::least_squares` re-project of the minimum onto both surfaces;
    accepted only if `‖A−B‖ ≤ onSurfTol` and the sine is at/near the floor, else DEFER (no
    fabricated B). **(2) ENUMERATE ARMS** — build the shared tangent-plane basis at B, form
    the relative second fundamental form `H = II_A − II_B`, solve the tangent-cone quadratic:
    discriminant `Δ > 0` ⇒ TWO distinct real tangent lines ⇒ up to four world-space rays;
    `Δ ≤ 0` ⇒ EMPTY (definite ⇒ isolated `TangentPoint`, END; double root ⇒ cusp, out of
    scope, DEFER). **Never fabricates a ray** — the same discriminant sign as S4-b's
    `TangentPoint` classification enforces "an isolated tangent point still ends". **(3)
    ROUTE** — step `h₀/8` off B along each real ray, S4-c-correct back onto both surfaces,
    then run the normal S3 walk to termination; drop the arm if `S₀` fails on-both-surfaces
    or the march makes no progress. **(4) ASSEMBLE** — dedup arms that retrace a kept arm
    (`retraces`), merge their shared branch-point connectivity into the `BranchNode`
    (`point`, `branchSine`, `armLineIds`), `++branchPoints`. A branch not robustly
    localizable/enumerable/routable STOPS + defers **exactly as S4-c** (a `NearTangent` WLine
    in `nearTangentGaps`). **At the bar:** the **Steinmetz bicylinder** (two equal-R=1
    orthogonal cylinders) that S3+S4-c TRUNCATE at the saddle (one `NearTangent` WLine,
    `branchPoints=0`) is now FULLY traced: `branchPoints=2` localized at `(0,±1,0)` (branch
    sine ≈ 5e-8 / 9e-8, re-projection residual ≈ 5e-13), 4 `BranchArc` arms assembled into
    the two crossing ellipses, `nearTangentGaps=0`, every node on both cylinders ≤ `onSurfTol`;
    sim parity vs OCCT `eq-cyl s4d branchPts=2 traced=4 arms=3 onCurve=1.74e-6 onSurf=1.07e-8`
    (both branch points match the OCCT saddles at `(0,±1,0)`). The isolated `TangentPoint`
    (spheres `d=R₁+R₂`) STILL ENDS with zero arms; the S4-c graze still crosses
    (`crossed=22`); the flag-off eq-cyl control still defers; the 5 transversal pairs stay
    `nt=0` bit-identical. Both gates green: host `test_native_ssi_marching` (**12 cases, 0
    failed**; NUMSCI OFF CTest **26/26**, NUMSCI ON **31/31**) + sim
    `scripts/run-sim-native-ssi-s4d.sh` (**8 passed, 0 failed**). No regressions (S5
    `native-pass=5` persists, tessellator byte-identical, `src/native/**` OCCT-free).
    **Honest scope — what S4-d does NOT do:** only the elementary two-real-distinct-line
    **transversal self-crossing** (Steinmetz family) is traced; general/freeform branch
    points, three-plus tangent lines, cusps (double root) and S4-f self-intersection
    completeness DEFER → OCCT, reported with the measured gap, never faked (the **S4-e chart
    singularities** — sphere pole + cone apex — are now crossed natively; see the S4-e bullet
    below). **Steinmetz is now unblocked** natively. Files: `src/native/ssi/branch_point.h` +
    `src/native/ssi/marching.{h,cpp}` + `tests/native/test_native_ssi_marching.cpp` +
    `tests/sim/native_ssi_marching_parity.mm` + `scripts/run-sim-native-ssi-s4d.sh`. Living
    change `openspec/changes/add-native-ssi-s4d-branch-points` **archived** (`2026-07-04`).
  - **SSI Stage S4-e (CHART SINGULARITIES — analytic sphere pole + cone apex, and the FREEFORM
    NURBS collapsed-row pole) — TWO HONEST SLICES DONE at the verification bar (both gates);
    the curve cusp declined by IFT.** A **chart (removable) singularity** is where
    ONE surface's own `(u,v)` parametrization degenerates (`‖dU‖ → 0`) while its 3D point +
    normal stay finite — a **sphere parametric pole** (`v = ±π/2`) or a **cone apex** (signed
    radius `= 0`). The intersection can be perfectly TRANSVERSAL through it (the pair sine need
    NOT collapse), yet S3 breaks: `advanceParams` solves each surface's single-surface 2×2
    normal equations, and the vanishing `dU` row makes that 2×2 rank-1, so the `(u,v)` update
    is ill-conditioned even though the 3D residual + normal are fine (and the pole sits on a
    non-periodic `v` edge → spurious `BoundaryExit` / apex step-crawl). Additive to
    `marching.cpp` + new OCCT-free `chart_singularity.h`, gated `CYBERCAD_HAS_NUMSCI`, default-
    **off** `enableChartSingularities`; no `cc_*`. **(1) single-surface chart witness** —
    `chartConditionAt` finite-differences each surface's `‖dU‖` vs `‖dV‖·scale`; a collapse
    with a finite normal flags a pole/apex on THAT surface — computed from ONE surface's own
    Jacobian, DISTINCT from the S4-c pair sine and the S4-d locus-tangent flip, and a finite
    cap keeps a genuine domain boundary from being mistaken for a pole. **(2) point-based
    fixed-plane-cut crossing** — at a collapse, `crossChartSingularity` makes bounded fine
    POINT-BASED jumps along the fixed last-good tangent `t★` (the branch_point.h / S4-c cut),
    which never touches the degenerate `dU`, so it stays well-posed where `advanceParams`
    failed. **(3) loose chart map-back** — the sphere pole continues on the opposite meridian
    (`u_out = u_in + π mod 2π`), the cone apex is a single 3D point the curve passes through to
    the far nappe (`v → −v`); the singular point itself is never emitted. **(4) honest guard**
    — a node is emitted ONLY if it verifies on BOTH surfaces ≤ `onSurfTol` with real along-`t★`
    progress; on ANY failure the band is DISCARDED (roll back) and the march STOPS + defers →
    OCCT as a `NearTangent` gap (`nearTangentGaps`). No pole/apex point is ever fabricated.
    Both gates green: host `test_native_ssi_s4e_singularities` (**5 cases, 0 failed**; NUMSCI
    OFF CTest **26/26** with the S4-e suite correctly ABSENT, NUMSCI ON CTest **32/32**) + sim
    native-vs-OCCT `GeomAPI_IntSS` parity (`scripts/run-sim-native-ssi-marching.sh`,
    `tests/sim/native_ssi_marching_parity.mm`): the sphere great circle crossing BOTH poles
    (S3 truncated at half loop `len ≈ 3.1415`) is **FULLY traced** — `sphere-pole s4e singX=2
    NTgaps=0 closed=1`, `len` native 6.2829 vs OCCT 6.2832 (rel Δ 5.0e-05), nodes on the OCCT
    locus + both surfaces ≤ 1.51e-07; the double-cone `∩` plane line through the **apex** (S3
    step-collapsed at `v ≈ −0.04`) is **FULLY traced across both nappes** — `cone-apex s4e
    singX=1 NTgaps=0 nodes=159`, `v ∈ [−2.00, +2.00]`, on-locus 7.11e-16 / on-surface 6.79e-16.
    A genuine finite cylinder `v`-cap still exits as a `BoundaryExit` (chart machinery does NOT
    misfire). No regressions: the 5 transversal pairs stay `nt = 0` bit-identical, the S4-c
    graze still `crossed = 22`, the S4-d Steinmetz still `branchPts = 2 traced = 4`, S5
    `native-pass = 6` persists, tessellator byte-identical. Files:
    `src/native/ssi/chart_singularity.h` + `src/native/ssi/marching.{h,cpp}` +
    `tests/native/test_native_ssi_s4e_singularities.cpp` + `tests/sim/native_ssi_marching_parity.mm`.
    Living change `openspec/changes/add-native-ssi-s4e-singularities` **archived** (`2026-07-05`).
    **SECOND SLICE — FREEFORM parametric pole (DONE at the bar):** the same point-based corrector
    is extended to a NURBS unit sphere — a genuine freeform pole (a `uPeriod == 0`
    collapsed-control-ROW surface of revolution, so NO analytic `u + π` meridian map; the
    degenerate normal `normalize(Sᵤ×Sᵥ)` is a finite near-zero `Dir3`, never NaN). The ONLY new
    code is the far-side re-seed: `chartsing::freeformChartInvert` recovers the far LONGITUDE by a
    POINT-ONLY 1-D search (the `u` at the fixed near-pole latitude minimizing `‖S.point(u,vFix) −
    target‖`, using `S.point` alone — no `dU`, no normal), and `chartFarUV` branches on `uPeriod`
    (the analytic `uPeriod > 0` path stays byte-identical). A NURBS-sphere ∩ plane that S3
    truncated at the first pole (half circle, `len ≈ 3.1415`) now crosses BOTH freeform poles and
    closes the full great circle: host `s4e_freeform_nurbs_pole_full_great_circle` LANDS
    (`singularitiesCrossed ≥ 2`, `nearTangentGaps == 0`, `Closed`, every node on both surfaces ≤
    1e-6) + sim `freeform-pole s4e singX=2 NTgaps=0 closed=1`, `len` native 6.2829 vs OCCT 6.2832,
    on both surfaces ≤ 1.51e-07. A collapsed-row Bézier cone-tip on the v=1 DOMAIN BOUNDARY (a
    genuine surface ENDPOINT, no far side) correctly still DEFERS — control
    `s4e_freeform_tip_endpoint_still_defers` asserts `singularitiesCrossed == 0`, `NearTangent`
    (honest truncation → OCCT), no point fabricated past the tip. **CURVE CUSP — honest DECLINE
    (no dead code):** a cusp (marcher velocity `‖n_A × n_B‖ → 0`) is IDENTICALLY the S4-c/S4-d
    pair-tangency witness, not the single-surface chart witness; by the IFT a cusp with regular
    charts + healthy pair sine is the empty set, so a standalone S4-e cusp witness would be
    unreachable dead code — none added; cusps route to S4-c/S4-d/OCCT, never faked. **Honest tail
    (DEFERRED → OCCT, measured gap, never faked):** **asymmetric** freeform poles whose
    continued-tangent re-seed does not verify on both surfaces, **higher-order / edge / seam**
    degeneracies, a full B-rep degenerate-pole B-spline SOLID through the boolean pipeline (no
    native construct yet feeds a freeform-pole face to the marcher — the fixtures are hand-seeded,
    like the analytic S4-e ones), and **S4-f self-intersection completeness**. Files unchanged from
    the first slice (`chart_singularity.h` gains `freeformChartInvert`, `marching.cpp` branches
    `chartFarUV` on `uPeriod`). Living change `openspec/changes/add-native-ssi-s4e-general`
    **archived** (`2026-07-07`).
  - **SSI Stage S5-d (BRANCHED-TRACE CURVED BOOLEAN — Steinmetz COMMON) — DONE at the
    verification bar (both gates); the FUSE/CUT completion follows immediately below.** The S4-d
    branched trace is now turned into a native BOOLEAN: the Steinmetz bicylinder COMMON. On the S4 decline edge (`nearTangentGaps > 0`,
    no usable single seam) a cheap `steinmetzPreGate` (both `Cylinder`, `|rA−rB| ≤ tol`, axes
    orthogonal + crossing) RE-TRACES with `MarchOptions.enableBranchPoints = true`;
    `recogniseSteinmetzTrace` accepts the branched `TraceSet` ONLY when canonical
    (`nearTangentGaps == 0`, `branchPoints == 2`, exactly four `BranchArc` arms, each arm's
    endpoints coincident with the two branch-node points) — anything else → NULL → OCCT.
    `buildSteinmetzCommon` splits each cylinder wall along its two arcs into the inside-the-other
    lune patches (planar-triangle strips walked branch-to-branch in lockstep, every interior
    sample on the analytic cylinder), keeps the four whose centroid is inside the other cylinder,
    and welds them into ONE watertight shell through a single `VertexPool` — both sides of every
    arc and all four arcs at each branch point draw the SAME pooled nodes (the two branch-point
    vertices pooled once), seam-adjacent facets are planar triangles (S5-a discipline, no
    analytic face on a shared seam). The engine's existing `16 R³/3` oracle self-verifies and
    owns the OCCT fallback. **At the bar:** COMMON of two equal-R=1 orthogonal cylinders is
    non-NULL, watertight, enclosed volume 5.3287 vs the EXACT analytic `16/3 = 5.33333` **and**
    OCCT `BRepAlgoAPI_Common` 5.3333 — ΔV = 8.75e-04 (−0.088%), ΔA = 4.68e-04, inside the 1%
    curved-parity bar; no tolerance weakened. At S5-d landing time Steinmetz FUSE / CUT were
    DEFERRED → OCCT (honest NULL); `ssi_boolean_solid` dispatched only `Op::Common` to the branched
    builder. Both gates were green at landing: host `test_native_ssi_curved_boolean` +
    `test_native_ssi_boolean` (analytic `16 R³/3` + watertight + 2-branch/4-arm assertions; FUSE/CUT
    + disjoint → NULL; NUMSCI OFF CTest **26/26**, NUMSCI ON **31/31**) + sim
    `run-sim-native-ssi-curved-boolean.sh` (**18 passed, 0 failed, native-pass=6**). Additive to
    `src/native/boolean/ssi_boolean.{h,cpp}` (`src/native/**` OCCT-free, `CYBERCAD_HAS_NUMSCI`-gated,
    tessellator + ssi tracer + app byte-identical; no `cc_*` entry point added). No regressions — the
    5 prior native passes (S5-a/b/c) persist; the +1 is exactly the Steinmetz COMMON.
    Living change `openspec/changes/add-native-ssi-branched-boolean` **archived** (`2026-07-05`).
  - **SSI Stage S5-d COMPLETION (Steinmetz FUSE + CUT) — DONE at the verification bar (both
    gates), `2026-07-06`.** The Steinmetz op-set is now COMPLETE 3/3 native. `buildSteinmetzFuse`
    (A∪B) keeps the OUTSIDE wall regions of BOTH cylinders + all four original end caps welded
    along the four arcs (`V=V(A)+V(B)−V(common)`); `buildSteinmetzCut` (A−B) keeps A's OUTSIDE wall
    + A's caps + B's lune patches emitted REVERSED (inward normal, bounding the carved channel,
    `V=V(A)−V(common)`) — both reuse the SAME branched trace + lune/arc split + `VertexPool` weld,
    differing only by fragment selection + cap handling, and both self-verify against the analytic
    inclusion-exclusion volume (engine owns the OCCT fallback). **At the bar** (equal-R=1 orthogonal
    cylinders): FUSE volN **32.385** vs OCCT `BRepAlgoAPI_Fuse` **32.366** (ΔV 5.82e-04); CUT volN
    **13.526** vs OCCT `BRepAlgoAPI_Cut` **13.516** (ΔV 7.22e-04); COMMON byte-identical (volN 5.3287);
    all watertight/closed/valid, inside the 1% bar, no tolerance weakened. A disjoint Steinmetz pair
    declines → NULL → OCCT for all three ops. Both gates green: host `test_native_ssi_curved_boolean`
    (new `branched_fuse_cut_watertight_matches_analytic` + `branched_disjoint_returns_null`) +
    `test_native_ssi_boolean` (FUSE/CUT now `!isNull()`), NUMSCI OFF CTest **29/29**, NUMSCI ON
    **36/36**; sim `run-sim-native-ssi-curved-boolean.sh` **18 passed, 0 failed, native-pass=12**.
    Additive to `src/native/boolean/ssi_boolean.{h,cpp}` (`src/native/**` OCCT-free, tessellator +
    ssi tracer + app byte-identical; no `cc_*` entry point added). No regressions — the 10 prior
    native passes persist, COMMON byte-identical; the +2 are exactly the Steinmetz FUSE + CUT.
    **Honest scope — what remains → OCCT:** any branched pair that is NOT equal-R orthogonal
    Steinmetz (unequal-R / non-orthogonal / ≠ 2-branch / ≠ 4-arm), a disjoint Steinmetz pair,
    cyl∩sphere / cyl∩cone / cone∩cone self-crossings, freeform branched.
    Living change `openspec/changes/complete-steinmetz-fuse-cut` **archived** (`2026-07-06`).
- â **`#4b` Tier E â native `cc_wrap_emboss` â DEFERRED (FUTURE WORK, not scheduled
  yet).** This is the *native* (OCCT-free) rewrite of wrap-emboss; it is distinct from
  the Phase-3 `add-robust-wrap-emboss` change, which is â done and OCCT-backed (the
  app-facing behaviour already works). UPDATE 2026-07-05: the FIRST NATIVE SLICE has
  since LANDED -- emboss a RECTANGULAR pad onto a CYLINDER lateral face is now native +
  verified vs OCCT (see the `#7 native-wrap-emboss` entry below; `add-native-wrap-emboss`
  archived `2026-07-05`). UPDATE 2026-07-07: the BREADTH change `add-native-wrap-emboss-breadth`
  (archived `2026-07-07`) widened the native path with T1 DEBOSS (recessed rectangular
  pocket on a cylinder, `boss=0`) and T2 NON-RECTANGULAR polygon emboss AND deboss on a
  cylinder. Residual that STILL stays OCCT-fallthrough: non-cylindrical (cone / sphere /
  freeform) bases (T3 honest decline — no native builder, since the OCCT oracle is itself
  cylinder-only), self-intersecting / dense / high-curvature profiles, `>2pi` / off-end
  footprints, and `depth >= R` debosses. The plan below is retained for those remaining
  general slices. Native wrap-emboss needs three pieces:
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
  gates green); the curved CIRCULAR cyl<->plane fillet (CONVEX + CONCAVE constant-radius, AND VARIABLE-radius LINEAR-law convex via `cc_fillet_edges_variable`) and the curved CIRCULAR cyl<->cap CHAMFER (CONVEX, CONE-FRUSTUM straight bevel, C0 — SYMMETRIC via `cc_chamfer_edges` AND ASYMMETRIC two-distance `d1!=d2` via `cc_chamfer_edges_asym`, an OBLIQUE cone frustum, C0 at two different angles) landed natively in later slices (see the #6 / #6b / #6c curved-blend entries below); non-linear-law / concave-variable / cyl<->cyl-canal / non-circular-crease fillets, non-circular / concave / cyl<->cyl chamfer, and fillet_face still OCCT-fallthrough (honest).** Native
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
- **#6 `native-blends` -- CURVED-blend slice DONE at all gates for BOTH a CONVEX and a
  CONCAVE constant-radius rolling-ball fillet on a CIRCULAR crease (cylinder lateral <->
  coaxial plane) -> TORUS canal, G1-tangent, verified vs OCCT `BRepFilletAPI`.** Extends the
  planar tangent-cylinder fillet to the curved crease, in two ball-centre-offset signs.
  **CONVEX (cyl <-> coaxial cap rim, REMOVES material):** a ball of radius `r` rolled into
  the convex circular rim (cylinder radius `Rc`, coaxial cap at axial `H`) stays tangent
  to both: its centre traces a circle of radius `Rc - r` at height `H - r` -- the
  tube-centre circle of a TORUS coaxial with the cylinder (major `R = Rc - r`, minor
  `r`). The blend surface is that torus's quarter-tube `v in [0, pi/2]`: at `v = 0` the
  point is at radius `Rc` with a RADIAL normal (tangent circle on the cylinder wall --
  `torus n cylinder`, an analytic SSI-S1 coaxial seam); at `v = pi/2` the point is at
  radius `Rc - r` with an AXIAL normal (tangent circle on the cap -- `torus n plane`). So
  the fillet is G1-tangent to the wall at `v=0` and to the cap at `v=pi/2` by
  construction. Built OCCT-FREE in `src/native/blend/curved_fillet.h` (reuses
  `blend_geom.h` + `math/torus.h` + boolean `assembleSolid`): it recognises the cyl<->cap
  rim by GEOMETRY (sole coaxial Cylinder at the rim radius + sole Plane through the rim
  perpendicular to the axis), resolves the cap + far-end axial heights, and rebuilds the
  whole filleted solid as one deflection-bounded planar-facet soup -- far cap, cylinder
  wall up to the wall seam, the torus quarter-tube (each curved quad SPLIT INTO TWO
  TRIANGLES so every facet is exactly planar and welds watertight), and the trimmed cap --
  all four parts sharing the SAME `N` angular samples so the wall->torus (`H-r`, `Rc`) and
  torus->cap (`H`, `Rc-r`) seams weld with coincident vertices. Requires a RING torus
  `Rc >= 2r` (else the tube self-intersects the axis -> NULL -> OCCT). Dispatched in
  `NativeEngine::fillet_edges` AFTER the planar path declines, then the MANDATORY
  `blendResultVerified` self-verify (watertight + `0 < Vr < Vo`) accepts it or discards ->
  OCCT. Gate 1 GREEN -- host `test_native_blend` adds the closed-form-volume assertion +
  `curved_fillet_g1_tangent_at_both_seams` (a pure-math check that the torus canal normal
  is exactly radial at `v=0` vs the cylinder normal and exactly axial at `v=pi/2` vs the
  cap normal, and the seam radii coincide -- ANALYTIC, no mesh, no OCCT). Gate 2 GREEN --
  `run-sim-native-curved-fillet.sh` **9/9** through the `cc_*` facade under
  `cc_set_engine(1)` (`activeNative=1`), native torus blend vs OCCT `BRepFilletAPI`:
  vol rel <= 3.8e-3, area rel <= 2.1e-3, watertight, mesh-vol == B-rep, across
  `Rc in {5,4,6}` incl. the `Rc=2r` ring-torus boundary; the reported `cos(wall seam)=1.0
  / cos(cap seam)=1.0` is the ANALYTIC G1 the construction guarantees (flagged
  analytic-not-mesh-sampled, honestly). **CONCAVE (boss cylinder <-> larger coaxial plane
  base rim, ADDS material):** the everyday inside fillet -- a boss standing on a larger
  slab, filleting the base rim (a CONCAVE dihedral). The ball seats on the MATERIAL side, so
  the ball-centre offset sign FLIPS: the torus tube-centre circle has major `R = Rc + r`
  (convex was `Rc - r`) at height `H + r`, still a coaxial TORUS quarter-tube, tangent to the
  wall at the `v=0` circle of radius `Rc` and to the plane at the `v=pi/2` circle of radius
  `Rc + r` (the plane is rebuilt as an ANNULUS with inner radius `Rc + r`). Because it fills
  the reflex corner, the fillet ADDS material and the enclosed volume GROWS. Built in the same
  OCCT-free `curved_fillet.h` as `concave_fillet_edge(...)` (additive-only; the convex
  `curved_fillet_edge` is byte-identical), reusing the same trim + weld helpers. Dispatched in
  `NativeEngine::fillet_edges` as the THIRD candidate (planar SHRINK -> convex-curved SHRINK ->
  concave-curved GROW), each gated by its OWN correctly-signed `blendResultVerified`: the
  concave path uses `wantGrow=true` (watertight + `Vr > Vo`, the SAME grow branch offset-face
  uses) -- no new guard, no weakened tolerance; a convex candidate can never pass grow and a
  concave never passes shrink, so the sign cannot be spoofed. Gate 1 GREEN -- host
  `test_native_blend` adds `concave_fillet_boss_on_plate_watertight_volume_grown`,
  `concave_fillet_g1_tangent_at_both_seams` (analytic, seams `Rc` / `Rc+r`), and
  `concave_fillet_scope_defers` (17 cases / 0 failed; host CTest 29/29 OFF, 36/36 ON). Gate 2
  GREEN -- `run-sim-native-curved-fillet.sh` now **15/15** (convex 9/9 + concave 6/6,
  `grew=1`, native `n` != OCCT `o`, e.g. boss `Rc=5` on plate `Rp=12` r=1.5: native 2294.95 vs
  OCCT 2296.98, relO 8.85e-4, watertight). STILL OCCT-fallthrough (NULL / self-verify discards,
  honest error, never faked): the blind-hole bottom rim (deferred this slice), VARIABLE radius,
  cyl<->cyl / cyl<->cone canal fillets, NON-circular curved creases (cone/sphere/ellipse/spline
  rim), freeform neighbours, convex `Rc < 2r` near-degenerate, seam-leaves-face, multi-edge.
  Changes `add-native-curved-fillet` archived `2026-07-05`, `add-native-concave-fillet` archived
  `2026-07-06`.
- **#6b `native-blends` -- VARIABLE-RADIUS curved-fillet slice DONE at all gates: a CONVEX
  circular cylinder<->coaxial-cap rim with a LINEAR radius law `r(theta) = r1 + (r2-r1)*theta/2pi`,
  verified vs OCCT `BRepFilletAPI` (evolved).** Generalises the constant-radius convex #6: the
  rolling-ball radius now VARIES around the rim, so the centre locus is no longer a fixed-offset
  circle but a SWEPT curve and the two trim seams are NON-circular (varying-radius) curves on the
  cylinder + plane. Built OCCT-FREE in `src/native/blend/curved_fillet.h` as `variable_fillet_edge(...)`
  (additive-only; the constant convex `curved_fillet_edge` + concave `concave_fillet_edge` are
  byte-identical), reusing the same trim + planar-facet weld helpers. The blend is a ring of planar
  facets swept along the rim, each station using the local `r(theta)` upright meridian arc; **G1-tangent
  at BOTH varying-radius seams by construction** (canal normal radial at the wall seam `v=0`, axial at
  the plane seam `v=pi/2`, cos=1.0 at every station). Wired into
  `NativeEngine::fillet_edges_variable` behind the `cc_fillet_edges_variable` facade, gated by the
  SAME correctly-signed `blendResultVerified(result, shape, wantGrow=false)` self-verify the constant
  convex path uses (a variable convex fillet REDUCES volume; a candidate that is not watertight or
  does not shrink to a sane volume is discarded -> NULL -> OCCT `BRepFilletAPI` evolved; the native
  OCCT-free shape cannot itself be forwarded to OCCT, same pattern as the constant fillet). Gate 1
  GREEN -- host `test_native_blend` adds `variable_fillet_cylinder_cap_watertight_volume_between`,
  `variable_fillet_second_fixture_and_reversed`, `variable_fillet_reduces_to_constant_when_r1_eq_r2`,
  `variable_fillet_g1_tangent_at_both_seams`, `variable_fillet_scope_defers` (22 cases / 0 failed;
  host CTest 29/29 OFF, 36/36 ON). Gate 2 GREEN -- `run-sim-native-curved-fillet.sh` now **23/23**
  (15 constant convex+concave controls unchanged + 8 variable). The HARD native gate: watertight,
  native volume matches the builder's OWN closed-form SWEPT removed volume -- fixture A (Rc=5,
  r1=1->r2=2) relX 1.08e-3 (769.963 vs 770.796), fixture B (Rc=6, r1=0.75->r2=2.25) relX 5.37e-4
  (1338 vs 1338.72) -- REDUCED vs the sharp cylinder, mesh<->B-rep vol ~1e-16, and DISTINCT from the
  OCCT evolved oracle (769.963 vs 778.957) -- proof the sim exercised native geometry, not an OCCT
  fall-through. The native-vs-OCCT-evolved parity is a SEPARATE, LOOSER line (relO 1.15e-2 / 1.09e-2,
  asserted only against 6e-2): the upright-meridian canal differs from OCCT's tilted evolved envelope
  by O(r') in the INTERIOR, agreeing exactly at both seams and in the `r1=r2` limit -- REPORTED
  honestly, never hidden behind the HARD bound. STILL OCCT-fallthrough (NULL / self-verify discards,
  honest error, never faked): NON-LINEAR radius laws (quadratic/spline/per-vertex), CONCAVE variable
  rim, cyl<->cyl / cyl<->cone canal, NON-circular variable creases (cone/sphere/ellipse/spline rim,
  tilted/non-coaxial plane), freeform neighbours, `Rc < 2*rmax` near-degenerate
  or cap radius `Rc - rmax <= 0`, seam-leaves-face, multi-edge (the convex-circular curved CHAMFER is
  now its own native slice #6c below). Change `add-native-variable-fillet` archived `2026-07-06`.
- **#6c `native-blends` -- CURVED CHAMFER slice DONE at all gates: a CONVEX circular
  cylinder<->coaxial-planar-cap rim chamfered as a CONE FRUSTUM (straight bevel, C0 NOT G1),
  verified vs OCCT `BRepFilletAPI_MakeChamfer`.** Unlike the #6 fillet (a curved TORUS arc, G1-tangent),
  a chamfer cuts a FLAT bevel: for a circular rim it is a CONE-FRUSTUM band between the two SETBACK
  circles -- one on the cylinder wall at axial setback `= d`, one on the cap at radial setback `= d` --
  meeting each face at the chamfer angle (**C0, NOT G1**; asserting tangency would be geometrically
  WRONG for a chamfer). Built OCCT-FREE in a NEW header `src/native/blend/curved_chamfer.h` as
  `curved_chamfer_edge(...)` (additive-only; it `#include`s `curved_fillet.h` to REUSE the rim
  recognition (`detail::facesOnRim`, `cylinderInfo`, `rimGeom`), the `sagittaSteps` angular tiling, and
  the `emit*`/planar-facet weld helpers). It rebuilds the capped-cylinder region as one
  deflection-bounded planar-triangle soup -- far cap, wall up to the cylinder setback circle
  (`Rc @ H-s*d`), the straight cone-FRUSTUM bevel band (`Rc @ H-s*d -> Rc-d @ H`, ONE meridian step,
  no minor subdivision), and the cap trimmed to the cap setback circle (`Rc-d @ H`) -- all sharing the
  same `N` angular samples, welded watertight via the boolean `assembleSolid`. The two seams are
  closed-form CIRCLES (cylinder seam radius `Rc`; cap seam radius `Rc-d`) -- no solver, no NUMSCI.
  `native_blend.h` gains the new `#include`. Wired into `NativeEngine::chamfer_edges` as a THREE-way
  dispatch: planar `nblend::chamfer_edges` (SHRINK) -> `nblend::curved_chamfer_edge` (SHRINK) -> honest
  error/OCCT, each candidate accepted ONLY through the SAME `blendResultVerified(result, body,
  wantGrow=false)` self-verify (watertight + `0 < Vr < Vo`); a chamfer REMOVES material so both native
  slices use the SHRINK branch. The planar builder is tried FIRST and is byte-identical (a circular rim
  declines it), so the planar chamfer path does not regress. Gate 1 GREEN -- host `test_native_blend`
  adds `curved_chamfer_cylinder_cap_watertight_volume_reduced` (Fixture A Rc=5 h=10 d=1: watertight +
  volume matches the EXACT closed-form Pappus removed volume `pi*d^2*(Rc - d/3)`),
  `curved_chamfer_second_fixture_and_removes_more_than_fillet` (Fixture B d=2: `V_chamfer < V_fillet <
  V0`), `curved_chamfer_is_c0_bevel_not_g1` (bevel normal = the `radial + s*axis` bisector, cos=1/sqrt2
  with BOTH faces and explicitly `!= 1` -- C0, not tangent), `curved_chamfer_scope_defers`,
  `curved_chamfer_both_rims_and_planar_declines` (27 cases / 0 failed; host CTest 29/29 OFF, 36/36 ON).
  Gate 2 GREEN -- `run-sim-native-curved-chamfer.sh` **9/9** through `cc_chamfer_edges` under
  `cc_set_engine(1)` (`activeNative=1`), native cone-frustum chamfer vs OCCT `BRepFilletAPI_MakeChamfer`
  `Add(distance, edge)` (symmetric): because a symmetric chamfer IS EXACTLY a cone frustum, the
  native<->OCCT vol parity is TIGHT (rel <= 3.25e-3, angular-deflection-bounded, NOT a loosened band)
  AND matches the exact Pappus removed volume (rel <= 3.25e-3), area rel <= 1.61e-3, watertight,
  mesh-vol == B-rep, reduced vs the sharp cylinder; the C0-bevel line `cos(wall)=cos(cap)=1/sqrt2 (!=1)`
  is analytic (the sim line restates the analytic value rather than re-measuring facet normals -- the
  genuine independent bevel-angle measurement is the host `curved_chamfer_is_c0_bevel_not_g1`).
  Fixtures: Rc=5/d=1, Rc=5/d=2, Rc=4/d=1. STILL OCCT-fallthrough (NULL / self-verify discards, honest
  error, never faked): NON-circular curved creases (cone/sphere/ellipse/spline rim, tilted/non-coaxial
  plane), CONCAVE circular rim (frustum would ADD material), cyl<->cyl (curved<->curved) chamfer,
  freeform neighbours, `Rc <= d` (cap circle collapses) or wall shorter than `d`, multi-edge. Change
  `add-native-curved-chamfer` archived `2026-07-06`. (ASYMMETRIC two-distance on the same convex
  circular rim is now native too -- see #6c.)
- ✅ **#6c `native-blends` — ASYMMETRIC TWO-DISTANCE chamfer LANDED NATIVE on the CONVEX circular
  cylinder<->coaxial-planar-cap rim (`d1 != d2`, OBLIQUE cone frustum, C0 at TWO DIFFERENT angles),
  verified vs OCCT `BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face)`.** The symmetric #6b frustum is the
  exact `d1 == d2` special case; the asymmetric bevel is an OBLIQUE cone frustum between the two setback
  circles `(Rc, H-s*d1)` (wall, axial setback `d1`) and `(Rc-d2, H)` (cap, radial setback `d2`), meeting
  the wall at `cos = d1/sqrt(d1^2+d2^2)` and the cap at `cos = d2/sqrt(d1^2+d2^2)` — two DISTINCT angles,
  both explicitly `!= 1` (C0, never G1). Built OCCT-FREE by generalizing `src/native/blend/curved_chamfer.h`
  `buildChamferedCylinder(g,d,defl)` -> `buildChamferedCylinderAsym(g,d1,d2,defl)` (the symmetric entry now
  wraps it with `d1=d2=d`, byte-identical soup) + a new `curved_chamfer_edge_asym(...)` reusing the same
  rim recognition, welded watertight via `assembleSolid`. ADDITIVE facade `cc_chamfer_edges_asym(body,
  edgeIds, count, d1, d2)` + `IEngine::chamfer_edges_asym` (OCCT override prefers the cylinder WALL face so
  `d1` is the wall setback) + `NativeEngine::chamfer_edges_asym` gated by the SAME SHRINK self-verify
  `blendResultVerified(wantGrow=false)`; `cc_chamfer_edges` is BYTE-UNCHANGED. Gate 1 GREEN — host
  `test_native_blend` adds `asym_chamfer_oblique_frustum_watertight_volume` (watertight + removed volume
  `pi*d1*d2*(Rc-d2/3)`), `asym_chamfer_two_bevel_angles_c0` (the two distinct `!= 1` cosines),
  `asym_chamfer_symmetric_special_case` (`d1=d2` -> the symmetric removed volume), `asym_chamfer_scope_defers`
  (31 cases / 0 failed; host CTest 29/29 OFF, 36/36 ON). Gate 2 GREEN — `run-sim-native-curved-chamfer.sh`
  **18/18** (9 symmetric controls unchanged + 9 new asymmetric) through `cc_chamfer_edges_asym` under
  `cc_set_engine(1)`, native oblique frustum vs OCCT `Add(d1,d2,edge,face)`: native == OCCT to rel <=
  3.25e-3 AND matches the exact Pappus removed volume `pi*d1*d2*(Rc-d2/3)`, watertight, mesh-vol == B-rep,
  the two distinct bevel cosines `!= 1`. Fixtures Rc5/H10 d1=2/d2=1 + swapped (1,2), Rc4/H8 d1=1.5/d2=0.8.
  STILL OCCT-fallthrough (honest, never faked): asymmetric on a NON-circular / concave / tilted / cyl<->cyl
  rim, `Rc <= d2`, wall shorter than `d1`, multi-edge. Change `add-native-fillet-chamfer-breadth` archived
  `2026-07-07`. **The T2 ELLIPTICAL-crease fillet (cylinder ∩ oblique plane) and T3 CYL<->CYL-canal fillet
  from the same change are HONEST DECLINES → OCCT (documented OCCT-fallthrough in `NativeEngine::fillet_edges`,
  NO dead code): T2 has no OCCT-FREE constructor for a native Cylinder+oblique-Plane+Ellipse body (SSI reads
  only quadric↔quadric pairs, oblique cuts are OCCT-built → the path is unreachable natively; OCCT ref Rc5/H10/60°/r1
  → 383.454285); T3's equal-radius perpendicular Steinmetz crease loops cross at the poles so a single swept-r-circle
  canal cannot close the corner blend watertight/G1 — a model gap, not a tolerance (OCCT ref Rc3/L20/r0.5 COMMON
  → 143.179260).**
- **#7 `native-wrap-emboss` -- NATIVE now covers EMBOSS + DEBOSS + NON-RECTANGULAR POLYGON
  on a CYLINDER, verified vs OCCT `cc_wrap_emboss`.** The control (raised RECTANGULAR pad
  on a cylinder) is unchanged; the breadth change `add-native-wrap-emboss-breadth` (archived
  `2026-07-07`) added T1 DEBOSS (recessed rectangular pocket, `boss=0`, volume SHRINKS by
  `footprint area * depth`, boolean-cut mirror of the pad with an inward `R-depth` floor and
  pocket-facing wall normals; guard `depth >= R` -> NULL) and T2 NON-RECTANGULAR: an N-vertex
  (`count>=3`) closed SIMPLE polygon footprint (ear-clipped cap, one ruled side wall per edge,
  bbox-minus-polygon base wall) embossed OR debossed (T1xT2 crossed deboss-polygon also native).
  Self-intersecting / degenerate polygons -> NULL -> OCCT. T3 FREEFORM base (cone / sphere) is an
  HONEST DECLINE — NO native cone/sphere builder exists because the OCCT `cc_wrap_emboss` oracle
  is itself CYLINDER-ONLY (rejects Sphere/Cone faces), so there is no parity oracle to certify
  against; `cylinderWall` returns NULL for any non-cylinder face (no dead never-accepted path).
  Sim `run-sim-native-wrap-emboss.sh` **14/14** (6/6 rect-control byte-stable + deboss-rect x2 +
  emboss/deboss-hex), vol rel <= 8e-3, area rel <= 1.6e-2, all watertight, mesh-vol == B-rep.
  Host `test_native_wrap_emboss` 7/7. FIRST-SLICE detail (unchanged control) follows. The Phase-3
  `cc_wrap_emboss` (#290) stays the ORACLE; this adds a NATIVE path behind the same ABI.
  Built OCCT-FREE in `src/native/feature/wrap_emboss.h`: for `boss=1` (emboss) on a native
  solid whose picked face is a Cylinder wall, it wraps a closed 4-corner rectangular
  footprint by the SAME map the OCCT oracle uses (`u = px/R` arc-length->angle,
  `v = py + vMid` axial, `vMid` = wall's axial middle), then rebuilds the whole embossed
  solid as one deflection-bounded planar-facet soup -- the pad's OUTER CAP (cylinder patch
  at `R+height` over the footprint window), two CIRCUMFERENTIAL walls (planes perp to axis
  at `v=vMin,vMax`), two AXIAL walls (planes through the axis at `u=uMin,uMax`), and the
  base cylinder wall retiled over the FULL turn with the footprint window REMOVED -- every
  part sharing a common `u`-sample sequence (window arc = the first `nUwin` cells) so all
  seams weld watertight via the boolean `assembleSolid` (each curved quad SPLIT INTO TWO
  TRIANGLES, planar-facet discipline). Why a facet soup, not an SSI fuse: the pad carries
  planar walls + a cylindrical cap, so it is NOT a single elementary curved solid and the
  S5-a single-elementary-pair `ssi_boolean_solid` cannot drive it -- mirroring the curved-
  fillet slice is the robust path. Dispatched in `NativeEngine::wrap_emboss` for a native
  body (an OCCT body forwards to the Phase-3 oracle unconditionally), then the MANDATORY
  `wrapEmbossVerified` self-verify (watertight + volume GROWS by ~ `footprint area *
  height`; the wrapped footprint area equals the flat profile area because `px` is already
  arc-length) accepts it or discards -> OCCT; a native body the slice declines returns an
  HONEST ERROR (never forwarded -- OCCT would misread the native void). Gate 1 GREEN --
  host `test_native_wrap_emboss` (footprint/rectangle recovery + facet-soup watertightness
  + volume-growth + decline of deboss/non-rectangular/non-cylindrical). Gate 2 GREEN --
  `run-sim-native-wrap-emboss.sh` **6/6** through the `cc_*` facade under
  `cc_set_engine(1)` (`activeNative=1`), native pad-on-cylinder vs OCCT `cc_wrap_emboss`:
  vol rel <= 2.5e-3, area rel <= 7.3e-4, watertight, mesh-vol == B-rep, across
  `Rc in {10,8,12}`. Native-vs-OCCT gap is deflection-bounded (planar-facet tiling vs
  OCCT's exact cylindrical faces), well inside the 1% bar; nothing faked. STILL OCCT
  (NULL / honest error), AFTER the breadth change: NON-cylindrical (cone / sphere / planar /
  NURBS) base (T3 honest decline — no native builder; OCCT oracle is cylinder-only),
  self-intersecting / degenerate / dense / high-curvature profiles, footprints that wrap
  `>2pi` / self-overlap / run off the axial ends, `depth >= R` debosses, non-positive height.
  Changes `add-native-wrap-emboss` archived `2026-07-05`, `add-native-wrap-emboss-breadth`
  (T1 deboss + T2 non-rectangular) archived `2026-07-07`.
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
  `STEPControl_Writer`. **`cc_step_import` STAYS OCCT at export time** (parsing arbitrary STEP is
  the huge part; a first native IMPORT slice landed later â see the import bullet below) and **`cc_iges_export/import` STAY OCCT** â that is
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
  the harness edge-count check now accepts as a bounded superset. STILL OCCT at export time (never
  faked): STEP IMPORT (a native subset landed later, below), IGES import/export, and a
  native solid with an out-of-scope geometry kind (Ellipse/Bezier curve, rational
  spline, Bezier surface) â OCCT fallback for an OCCT body / honest error for a
  native void. Living change `add-native-data-exchange` **archived** (validate
  --strict green). This is the native EXPORT slice only â import + IGES stay OCCT by
  design.
- ✅ **#7/#3 `native-exchange` — FIRST native STEP IMPORT slice DONE at BOTH gates (host round-trip + sim OCCT parity); sits on the #4 healing slice.** Behind the existing `cc_step_import(path)` ABI (default engine still OCCT), `NativeEngine::step_import` now runs an OCCT-FREE ISO-10303-21 (Part 21) reader (`src/native/exchange/step_reader.{h,cpp}`, entry `step_import_native(path)` / `readStepString`) that tokenizes the DATA section into a `map<int,Record>` (integer refs `#N`, typed reals incl. `1.E2`, strings, enums `.T.`, nested lists, `$`) and runs a TWO-PASS mapper: leaf geometry (CARTESIAN_POINT / DIRECTION / AXIS2_PLACEMENT_3D, LINE / CIRCLE / B_SPLINE_CURVE_WITH_KNOTS, PLANE / CYLINDRICAL_SURFACE / CONICAL_SURFACE / SPHERICAL_SURFACE / B_SPLINE_SURFACE) then topology (VERTEX_POINT → EDGE_CURVE → ORIENTED_EDGE → EDGE_LOOP → FACE_OUTER_BOUND/FACE_BOUND → ADVANCED_FACE → CLOSED_SHELL → MANIFOLD_SOLID_BREP), deduping shared EDGE_CURVE/VERTEX_POINT, building the B-rep through `topology::ShapeBuilder`, then running `heal::healShell` to close the sub-tolerance gaps STEP carries. It is the EXACT inverse of `step_writer.cpp` (writer untouched — additive). **Honest decline → OCCT (never fabricated):** unsupported surface/curve keyword (TOROIDAL / SURFACE_OF_REVOLUTION / offset / trimmed / Bezier surface; ELLIPSE / TRIMMED_CURVE / Bezier / rational-weighted B-spline), assembly / >1 MANIFOLD_SOLID_BREP, non-mm unit, malformed/dangling record, or any reconstruction failing the watertight+vol>0 self-verify → NULL, and the engine falls through to `STEPControl_Reader` (labelled). **Gate 1 (host, NO OCCT) GREEN** — `test_native_step_reader` 9/9: box round-trips EXACT (vol 1000, 6 faces / 8 verts / 12 edges, watertight), cylinder + holed-plate watertight with vol rel<1e-9 (host round-trip is byte-inverse of the writer, un-foolably native since no OCCT is linked), and 5 decline cases (TOROIDAL surface, 2-root assembly, non-mm unit, malformed string, empty) return NULL. Host CTest **29/29**. **Gate 2 (sim vs OCCT `STEPControl_Reader`) GREEN** — `tests/sim/native_step_import_parity.mm` + `scripts/run-sim-native-step-import.sh`: **`[NIMPORT]` 15 passed / 0 failed** — native-written box (vol rel 2.27e-16), cylinder (rel 1.27e-3), holed-plate (rel 2.90e-4) all watertight and matching the OCCT re-import within tol, AND FOREIGN OCCT-`STEPControl_Writer`-written box + cylinder imported NATIVELY match OCCT re-import EXACTLY (rel 0). The native path is provably exercised on gate 2(a): the reader reports per-face-oriented edge counts (box 24=2×12, cyl 30) whereas an OCCT fallback would report OCCT's unique count. **Honest residual / deferred to OCCT:** assemblies, AP242, complex/typed profiles beyond the writer's set, all IGES, and a B-spline-FACE solid round-trip (the reader maps B_SPLINE_SURFACE but no `cc_solid_*` builder emits a watertight bspline-face fixture, so no non-fabricated fixture exists yet — documented gap, not silently skipped). Change `add-native-step-import` archived (validate --strict green).
- ✅ **#7 `native-exchange` — STEP IMPORT WIDENED: multi-solid Compound + B-spline-face round-trip landed; ELLIPSE curve recognised; torus stays OCCT.** Building on the first import slice, three independent honestly-gated breadth tracks (change `widen-native-step-import`, archived `2026-07-06`, validate --strict green; host CTest **29/29**, sim **`[NIMPORT]` 28/28**). **LANDED (genuine native, verified vs OCCT):** (T2) **multi-solid** — a flat file with >1 root `MANIFOLD_SOLID_BREP` (no assembly transform tree) now imports as a native `topology::Compound` of watertight Solids instead of a blanket decline (`findManifoldBreps` collects all roots, `build()` maps each and `ShapeBuilder::makeCompound`s them; one root still returns a bare Solid — byte-identical prior behaviour); engine self-verify requires EVERY member watertight (`robustlyWatertightMulti`); sim vs OCCT re-import `nativeVol=1064 occtVol=1064 rel=2.14e-16`, per-solid watertight + count/bbox match. (T3) **B-spline-FACE round-trip** closed (prior deferred task 7.4) — the EXISTING native `build_prism_profile_spline` op (NOT a fabricated fixture) emits a watertight `B_SPLINE_SURFACE`-face solid that round-trips native-export→native-import EXACT (`vol nat=304.38 orig=304.38`, watertight, face-count + `B_SPLINE_SURFACE` present). **PARTIAL:** (T1a) the reader now RECOGNISES + maps the `ELLIPSE('',#pos,a,b)` curve entity to the genuine `EdgeCurve::Kind::Ellipse` (major=a along frame X, minor=b along Y; degenerate → decline), verified by a host edge-mapping test — **but there is NO watertight NATIVE ellipse-bearing-solid import**: a foreign OCCT-authored ellipse-cut solid parses yet its ellipse-on-quadric pcurve is out of this slice, fails the watertight self-verify (`watertight=0 nativeVol=0`), and the whole solid FALLS BACK to OCCT (`ellipse_cut vol nat=942.478 oracle=942.478` = the OCCT fallback). **NOT LANDED (documented DECLINE → OCCT):** (T1b) `TOROIDAL_SURFACE` — no native `FaceSurface::Kind::Torus` and the tessellator must not be modified, so the reader returns NULL and the engine falls back (`torus native parsed=0`, `fallback torus rel=0.00e+00`). A `hasNestedAssembly()` guard also DECLINEs any transform tree (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` / `MAPPED_ITEM` / `REPRESENTATION_RELATIONSHIP*` / `ITEM_DEFINED_TRANSFORMATION` / `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION`) → OCCT. **Honest residual → OCCT (unchanged):** torus, ellipse-on-quadric solids, nested/transformed assemblies, `SURFACE_OF_REVOLUTION`, `TRIMMED_CURVE`, rational/weighted B-splines, `BEZIER`, AP242 / PMI, non-mm units, all IGES. `step_writer.cpp`, the tessellator, and the `cc_*` ABI are PRISTINE. Does NOT unblock #8 `drop-occt` (a general STEP/AP242 reader + IGES + a general-curved kernel still block it).
- ✅ **#7 `native-exchange` — STEP IMPORT WIDENED to RIGID PLACED ASSEMBLIES + AP214/AP242 header acceptance pinned.** Building on the multi-solid slice, the reader now imports a single-level assembly with a transform tree as a native PLACED `topology::Compound` (change `add-native-step-assemblies`, archived `2026-07-06`, validate --strict green; host CTest **29/29** NUMSCI OFF, **36/36** NUMSCI ON; sim **`[NIMPORT]` 33/33**). **LANDED (genuine native, verified vs OCCT):** the `hasNestedAssembly()` decline is replaced by an `assembly()` builder that parses the OCCT-emitted transform structure — `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` → `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` → `ITEM_DEFINED_TRANSFORMATION` (an `AXIS2_PLACEMENT_3D` from/to pair) — composes each component's RIGID placement `T = frameToWorld(to) ∘ frameToWorld(from)⁻¹`, gates it with `isRigid` (orthonormal `M·Mᵀ≈I` AND det≈+1, tol 1e-9), resolves each component representation's root `MANIFOLD_SOLID_BREP`(s) STRUCTURALLY (by refs, not names), maps them via the UNCHANGED `mapManifoldBrep` in local coords, and pushes each `solid.located(Location{T})` into the Compound (native topology carries the placement on edges/faces — no geometry baked). Requires every root brep placed EXACTLY once (`placed.size()==findManifoldBreps().size()`) else NULL — never partial, never identity-defaulted. The flat multi-solid path and single-solid path are byte-for-byte unchanged; only a present transform tree takes the new branch. Sim vs OCCT `STEPControl_Reader` re-import on an OCCT-`STEPControl_Writer`-authored 2-box assembly (box B carries a non-baked `TopLoc_Location`: rotate 0.5 rad about Z + translate(30,5,0), so the writer emits the CDSR/REP_REL/ITEM_DEFINED chain rather than world-baked coords): **solids 2/2, nativeVol=1216 occtVol=1216 (mass rel 3.74e-16), bbox maxCornerΔ=0.00e+00 (tol 5e-3), topology faces 12/12**. **Schema-independence pinned:** the reader enters at `DATA;` and never gates on `FILE_SCHEMA`, so AP203/AP214/AP242 headers all import — confirmed live on a real OCCT-authored AP214 (`AUTOMOTIVE_DESIGN`) file (`header=AP214(1) native parsed=1 solids=1`). **Honest decline → OCCT (verified, never fabricated):** Form-B `MAPPED_ITEM`/`REPRESENTATION_MAP` (`decline_form_b_mapped_item_returns_null`); any non-rigid (scaled/mirrored/sheared) transform (the det≈+1 orthonormal gate); a transform tree with no composable placement (`placedCount==0`, e.g. lone NAUO — `decline_assembly_without_transform_returns_null`); a child rep resolving to 0 or >1 brep, a brep placed twice, or >1 unplaced root; out-of-slice component geometry (torus → `parsed=0` → OCCT). **Residual → OCCT (narrowed):** PMI/GD&T, non-rigid/scaled/mirrored transforms, deep-nested (multi-level) assemblies, complex/trimmed profiles, `SURFACE_OF_REVOLUTION`, ellipse-on-quadric solids, `TOROIDAL_SURFACE`, rational/weighted B-splines, `BEZIER`, non-mm units, all IGES. Exactly 3 files changed (`step_reader.cpp`, `test_native_step_reader.cpp`, `native_step_import_parity.mm`); `step_writer.cpp`, tessellator, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Does NOT unblock #8 `drop-occt`.
- ✅ **#7 `native-exchange` — STEP IMPORT WIDENED to UNIFORM-SCALE + MIRROR placed assemblies (T1) + AP242 geometry with PMI SKIPPED (T2).** Building on the rigid-assembly slice, the reader now composes two more affine placement classes and tolerates AP242 annotation graphs (change `add-native-step-scaled-ap242`, archived `2026-07-06`, validate --strict green; host CTest **29/29** NUMSCI OFF / **36/36** NUMSCI ON, `test_native_step_reader` **20 cases**; sim **`[NIMPORT]` 41/41**). **LANDED (T1, genuine native):** the boolean `isRigid` gate is replaced by `classifyPlacement(const math::Transform&)` — a Gram-matrix conformality test `MᵀM ≈ k²·I` with a det-sign branch → `Rigid`(k≈1,det>0) | `UniformScale`(k>0,det>0) | `Mirror`(det<0), and `nullopt` (DECLINE) for a non-conformal `MᵀM` (non-uniform/shear). `Rigid` reproduces the old path byte-for-byte. A **UniformScale** component (parsed from a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` scale, or a frame-encoded `k·I`; a `_NON_UNIFORM` / unequal `scale1/2/3` form declines) rides `solid.located(Location{T})` directly and self-verifies with volume `k³·V₀` — k=2 → total vol 2728 = 1000 + 216·8, component bbox [30,5,0]..[42,17,12], watertight. A **Mirror** component is orientation-complemented with the EXISTING `topo::Orientation` `reversed`/`complemented` algebra BEFORE the mirror `Location`, so the tessellator's tangent-derived normal (`cross(place(∂u),place(∂v))`, which flips under det<0) points OUTWARD again — the mirrored solid self-verifies watertight with POSITIVE volume 1216 (not −216) and reflected bbox z∈[−6,0]. **No tessellator change, no new topology primitive.** **Honest caveat (load-bearing):** OCCT's `STEPControl_Writer` **cannot serialize** a scaled/mirror assembly location — a 2× component re-imports at native size (scale silently dropped; the IDT AXIS2 frames stay orthonormal), a `SetMirror` becomes a proper 180° rotation (det +1), and the trimmed iOS OCCT throws "Location with scaling transformation is forbidden" on a scaled `TopLoc_Datum3D`; a `CARTESIAN_TRANSFORMATION_OPERATOR_3D` in the IDT slot is schema-invalid and OCCT's reader ignores it. So there is **no OCCT oracle for genuine k³/reflection** — T1 is verified against an **analytic** expectation via the standard STEP scale/mirror operator, and separately verified native == OCCT on the OCCT-authored fixtures that degrade to rigid. **LANDED (T2, genuine native):** the two GLOBAL record scans are relaxed so an AP242 file is not declined for carrying PMI. `validateUnitContext()` now answers exactly "is the LENGTH unit millimetre?" — a length `SI_UNIT` MUST be `.MILLI.` (mm gate UNCHANGED, no tolerance weakened) while a non-length `SI_UNIT` (`.RADIAN.`/`.STERADIAN.`, PMI angle/plane-angle contexts) is SKIPPED, not read as non-mm. `hasNestedAssembly()`/`assemblyDisposition()` return the assembly path only for a transform relationship that reaches a `MANIFOLD_SOLID_BREP`; a `REPRESENTATION_RELATIONSHIP`/`MAPPED_ITEM`/`CDSR` in the annotation/draughting graph that reaches no geometric root brep is SKIPPED, and the completeness gate is computed over the GEOMETRIC root breps only. An AP242 file (rewritten schema + injected PMI/GD&T/draughting incl. a rep-rel graph) imports the SOLID identically to the OCCT re-import (vol 1000, bbox Δ=0, faces 6/6) with PMI skipped — the previously-fatal rep-rel-PMI case now imports instead of declining. **Honest decline → OCCT (verified, never fabricated):** non-uniform-scale / shear transforms (`decline_non_uniform_shear_assembly_returns_null`, sim `shear` → NULL); a mirrored member that still fails the watertight self-verify after compensation; PMI/GD&T **semantics** (never turned into geometry); Form-B `MAPPED_ITEM`/`REPRESENTATION_MAP`; lone NAUO with no composable placement; deep-nested (multi-level) assemblies; out-of-slice component geometry (`TOROIDAL_SURFACE` etc.); ellipse-on-quadric solids; complex/trimmed profiles; rational/weighted B-splines; non-mm units; all IGES. Exactly 2 native/exchange files changed (`step_reader.{cpp,h}`) + 2 tests; `step_writer.cpp`, tessellator, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Does NOT unblock #8 `drop-occt`.
- ✅ **#7 `native-exchange` — STEP IMPORT WIDENED to TRIMMED_CURVE edges (T1) + a cylinder-reducing SURFACE_OF_REVOLUTION (T2).** Building on the scaled/AP242 slice, the reader accepts two leaf-geometry families it previously declined outright (change `add-native-step-general-surfaces`, archived `2026-07-06`, validate --strict green; host CTest **29/29** NUMSCI OFF / **36/36** NUMSCI ON, `test_native_step_reader` **26 cases**; sim **`[NIMPORT]` 53/53**). **LANDED (T1, genuine native):** `curve()` now ACCEPTS `TRIMMED_CURVE('',#basis,trim_1,trim_2,sense,master)` — `trimmedCurve()` unwraps the basis (`LINE`/`CIRCLE`/`ELLIPSE`/`B_SPLINE_CURVE_WITH_KNOTS`, incl. through the SURFACE_CURVE wrapper) onto the native `EdgeCurve`. For a **B-spline** basis the two cached `PARAMETER_VALUE` trims drive `[first,last]` (the covered knot sub-domain, clamped to the clamped span — info the endpoint vertices cannot recover); for an **analytic** basis the exact vertex-derived range is kept (trims redundant). `sense_agreement`/`master_representation` are not consulted. A foreign OCCT-authored solid whose edge geometry is wrapped in a TRIMMED_CURVE imports NATIVELY watertight == OCCT re-import (vol rel 1.27e-3, bboxΔ=0, faces 9/9). **LANDED (T2, cylinder case ONLY):** `surface()` accepts `SURFACE_OF_REVOLUTION('',#profile,#axis1)` (via `axis1placement` + `surfaceOfRevolution` + `revolvedLine`) ONLY when the profile is a straight `LINE` **parallel** to the axis → an EXACT native `Cylinder` (radius = ⊥-distance from line to axis); verified native == OCCT re-import (watertight, vol rel 1.27e-3). **HONEST DECLINE → OCCT (NOT implemented — the code has NO `reduceToQuadric` and NO `revolveToRationalBSpline`):** a SURFACE_OF_REVOLUTION of an **oblique** line (a cone — the reader's apex-carrying cone does not round-trip watertight, a separate pre-existing gap), a **perpendicular** line (a planar annulus), or **any non-line** profile (a circle/arc → sphere/torus, an ellipse/B-spline → general revolved surface) returns `nullopt` → OCCT, kept consistent with the landed `TOROIDAL_SURFACE` decline — host `surface_of_revolution_oblique_line_declines` + `surface_of_revolution_circle_generatrix_declines`, sim `revolution decline parsed=0` with the OCCT fallback matching `cc_set_engine(0)` (torus vol 523.599 rel 0). Exactly 3 files changed (`step_reader.cpp`, `test_native_step_reader.cpp`, `native_step_import_parity.mm`); `step_writer.cpp`, tessellator, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Does NOT unblock #8 `drop-occt`.
- ✅ **#7 `native-exchange` — SURFACE_OF_REVOLUTION analytic-quadric reductions (cone / plane landed native; sphere reduction host-verified).** Extends the prior slice's cylinder-only revolution arm to classify the generatrix and emit the matching native `FaceSurface` kind (change `add-native-step-revolution-quadrics`, archived `2026-07-07`, validate --strict green; host CTest **29/29** NUMSCI OFF / **36/36** NUMSCI ON, `test_native_step_reader` **31 cases** incl. **8** revolution cases; sim **`[NIMPORT]` 65/65**). **LANDED — oblique `LINE` → native `Cone`:** `revolvedLine` dispatches by the line-axis angle `c=|Â·D̂|`; `lineMeetsAxis` computes the common perpendicular and the apex on the axis (skew → DECLINE, a hyperboloid), emitting `FaceSurface{Cone}` (origin on axis, `Z=+axis`, signed `semiAngle=acos(c)`) matching the direct `CONICAL_SURFACE` convention. This needed a genuine reader **bug fix** first — `pcurveFor`'s constant-`u` meridian took the angle from the endpoint AT the apex (`atan2(0,0)=0` collapsed every apex-touching wall onto `u=0`, tearing the cone into 97 open edges); `radialFromAxis` now takes the constant `u` from the endpoint FARTHER from the axis, so the DIRECT `CONICAL_SURFACE` round-trips watertight too. Native import watertight, vol 522.934 vs OCCT 523.599 rel 1.27e-3, bboxΔ=2.96e-15, faces 6/6. **LANDED — perpendicular `LINE` → native `Plane`:** `c≈0` → a flat `Plane` annulus cap (both endpoints verified to share one axial coordinate). Native import watertight, vol 1568.8 vs OCCT 1570.77 rel 1.25e-3, faces 9/9. **HOST-VERIFIED reduction, end-to-end OCCT fallback — on-axis `CIRCLE`/arc → native `Sphere`:** `revolvedCircle` fires `FaceSurface{Sphere}` (radius = circle radius) iff the centre is ON the axis AND the circle plane contains the axis; asserted at host level equal to the direct `SPHERICAL_SURFACE` import — but watertight end-to-end spheres are NOT yet achievable (the native writer serialises a sphere as three pole-seam lune faces — a WRITER limitation, forbidden to touch — and an OCCT-authored sphere is a single periodic-pole face the reader does not yet reconstruct), so the sim fixture proves the guarantee honestly: the reader DECLINES the OCCT periodic-pole-face sphere and `cc_step_import` falls back to OCCT, vol 904.779 == OCCT rel 0. Flagged, not faked. **HONEST DECLINE → OCCT (no faithful native kind):** an off-axis circle/arc (torus, no `Kind::Torus`; fallback vol 523.599 rel 0), an ellipse / B-spline generatrix (general revolved surface; fallback rel 0), a skew oblique line (hyperboloid), and an on-axis circle whose plane is ⟂ the axis. Each builder re-evaluates the candidate quadric through the generatrix's defining points and DECLINES on a scale-relative mismatch (the "never fabricate geometry" gate; no tolerance weakened). Reader-only change — 4 files (`step_reader.cpp`, `test_native_step_reader.cpp`, `native_step_import_parity.mm`, + the archived change); `step_writer.cpp`, tessellator, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Does NOT unblock #8 `drop-occt`.
- ✅ **#7 `native-exchange` — STEP IMPORT: a FULL TORUS imports NATIVELY watertight (T1); general/ellipse revolution stays an honest DECLINE (T2).** Closes the last `SURFACE_OF_REVOLUTION` surface gap in two honestly-gated tracks (change `add-native-step-torus`, archived `2026-07-07`, validate --strict green; host CTest **29/29** NUMSCI OFF / **36/36** NUMSCI ON, `test_native_step_reader` **36 cases** incl. 2 new torus cases; sim **`[NIMPORT]` 69/69**). **LANDED (T1, genuine native watertight):** a NEW additive `FaceSurface::Kind::Torus` (appended to the enum + a new `minorRadius=0.0` field, so every existing kind is byte-unchanged) now carries a torus. A direct `TOROIDAL_SURFACE` face — and the off-axis-circle `SURFACE_OF_REVOLUTION` OCCT itself emits as a `TOROIDAL_SURFACE` — imports as a native `Kind::Torus` solid. Diagnosis showed OCCT bounds a full torus with a FULLY-SEAMED `EDGE_LOOP` (equator v-seam + tube u-seam, each forward AND reversed — no pole, no real trim), so T1 landed through the ALREADY-PROVEN sphere bare-periodic path: the reader detects the all-seam loop and builds a `Kind::Torus` face with a NULL outer wire; the tessellator meshes its natural `(u,v)∈[0,2π]²` rectangle through the UNCHANGED structured-grid path, welding BOTH seams. Host `toroidal_surface_full_torus_imports_watertight` (V=2π²Rr², watertight, ONE Torus face); sim `native torus` parsed=1 watertight=1 solids=1 nativeVol=1771.77 occtVol=1776.53 (rel 2.68e-3), 0 boundary edges (bare doubly-periodic surface — parity asserts volume/area/centroid/face-count + a curved-tessellation-bounded bbox, NOT edge-count, no tolerance weakened). **THE TESSELLATOR WAS NOT TOUCHED beyond a +9-line additive arm:** the ONLY change under `src/native/tessellate/` is `surface_eval.h` (a new `case K::Torus` in `localValue`/`localD1` + a `bounds → {0,2π,0,2π}` arm); **`face_mesher.h` and `trim.h` are NOT in the diff** (more conservative than the planned doubly-periodic seam-weld branch — the sphere path was reused). Existing tessellation is byte-identical by construction (the Torus arm is never evaluated for an existing kind) AND empirically (sphere/cyl/cone/plane revolution faces import byte-identically — `revolution→sphere` vol 902.31 == `sphere_keyword` 902.31, cyl/plane 1568.8, cone 522.934; curved-fillet tris=3912 unchanged; curved-fillet 23 / curved-chamfer 9 / ssi-curved-boolean 21 native-pass=13 / wrap-emboss 6 / run-sim-suite 221 / phase3 70 all match baseline). **T1 HONEST-OUT — a PARTIAL/trimmed torus DECLINES** (the bare-surface route closes watertight only for a fully-seamed full torus; host `toroidal_surface_partial_torus_declines`). **NOT LANDED (T2, honest DECLINE → OCCT retained):** an ELLIPSE / B-spline generatrix revolution keeps the `default → nullopt` decline (imports fine via OCCT). The exact surface is a rational tensor-product B-spline; that reconstruction + watertight self-verify + a capped-solid fixture is a larger, higher-blast-radius change deferred rather than rushed — the per-track gate resolves to DECLINE (sim `foreign revolution decline` parsed=0; OCCT fallback exact rel 0). Additive-only — `shape.h`, `surface_eval.h`, `step_reader.{cpp,h}` + 2 tests; `step_writer.cpp`, `face_mesher.h`, `trim.h`, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Does NOT unblock #8 `drop-occt`.
- ✅ **#7 `native-exchange` — STEP IMPORT: an ELLIPSE / (non-rational) B-spline generatrix SURFACE_OF_REVOLUTION imports NATIVELY as a RATIONAL tensor B-spline surface (the deferred T2, now LANDED).** Closes the LAST `SURFACE_OF_REVOLUTION` gap — a GENERAL profile curve that has no analytic-quadric reduction (change `add-native-step-general-revolution`, archived `2026-07-07`, validate --strict green; host CTest **29/29** NUMSCI OFF / **36/36** NUMSCI ON, `test_native_step_reader` **39 cases** incl. 3 new revolution cases; sim **`[NIMPORT]` 77/77**). **PIVOTAL CHECK → POSITIVE (no tessellator change).** The decisive question — does the native `FaceSurface::Kind::BSpline` carry WEIGHTS and does the tessellator mesh a rational surface watertight — resolves YES: `shape.h` ALREADY had `std::vector<double> weights;` (`empty ⇒ non-rational`) on `FaceSurface`, `math/bspline.h` already evaluates rational NURBS, and the existing sphere-style bare-periodic (`VERTEX_LOOP`) mesh path meshes the rational revolved surface watertight — so the honest DECLINE was NOT required and **`shape.h`, `math/**`, and the ENTIRE tessellator are BYTE-UNCHANGED.** **LANDED (genuine native watertight):** the reader revolves the generatrix control net around the axis at the standard revolution knot angles (`0, π/2, π, 3π/2, 2π`) with the standard rational weights (`1, 1/√2, 1, 1/√2, 1`), producing the EXACT rational tensor-product B-spline surface (degree-2 in the revolution direction), maps it onto a native `Kind::BSpline` face carrying those weights with a NULL outer wire (the `advancedFace` bare-periodic arm gains a single `isFullRevolutionBSpline` OR-clause), and the UNCHANGED bare-periodic path meshes it over its natural periodic bounds — welding the seam + degenerate axis poles into a watertight solid. Sim vs OCCT `BRepGProp`: `revolution→ellipsoid` nativeVol 6.6721 vs occtVol 6.70206 (rel **4.47e-3**), area rel 2.06e-3, watertight, 0 boundary edges; `revolution→bspline` nativeVol 130.995 vs 131.342 (rel **2.64e-3**), area rel 1.32e-3, watertight. **HONEST-OUT — an OFF-AXIS ELLIPSE revolution that fails the faithful-reconstruction/watertight self-verify DECLINES → OCCT** (host `revolution_off_axis_ellipse_declines`), as does a skew-line hyperboloid. Reader-only, additive change — `step_reader.{cpp,h}` + 2 tests; `step_writer.cpp`, the tessellator, `shape.h`, `math/**`, `src/engine/**`, and the `cc_*` ABI PRISTINE; `src/native/**` OCCT-free. Because the tessellator is byte-identical, existing meshes are unchanged by construction and the full tessellation-sensitive sim set was correctly not required. Does NOT unblock #8 `drop-occt`.
- ✅ **#4 `native-healing` — FIRST shape-healing slice DONE at BOTH gates (host + sim).** An INTERNAL, OCCT-FREE healer (`cybercad::native::heal::healShell`, `src/native/heal/*`) that stitches a face-soup / malformed shell into a connected, consistently-oriented, WATERTIGHT solid — or reports UNHEALED honestly. Four sub-operations, in dependency order: **vertex/tolerance unification** (the `boolean/assemble.h` `VertexPool` spatial hash generalized to arbitrary B-rep vertices), **tolerant sewing** (an edge becomes shared iff its endpoints unified to the same two shared vertices within tolerance — never a fabricated coincidence), **degenerate removal** (zero-length edges + sliver/near-zero-area faces via a min-height test), and **orientation fix** (flood-fill consistent winding across shared edges + a global enclosed-volume-sign tie-break). Every heal is SELF-VERIFIED (`tessellate::isWatertight` + `enclosedVolume > 0` across a deflection ladder) before it is kept; otherwise a typed `Unhealed` result carries the measured `maxResidualGap` and the ORIGINAL shape UNCHANGED. Gate 1 (host, no OCCT): `test_native_heal` — soup-cube heals to V=1 with `nMergedEdges=12` / `nMergedVerts=16`, a degenerate-edge and a sliver-face are dropped and the cube still heals to V=1, a flipped face is re-oriented (`nFlipped=1`), an all-inward cube triggers the global sign flip, near-coincident vertices unify (and beyond-tol ones never do), and both un-healable fixtures (missing face → `OpenShell`; gap 1e-2 → `GapBeyondTolerance`, residual 0.0255) report UNHEALED with the input unchanged — green under NUMSCI OFF **and** ON. Gate 2 (sim, `run-sim-native-heal.sh`): native-vs-OCCT parity on identical soups — `[NHEAL] 4 passed / 0 failed`: the in-scope soup-cube + flipped-face heal to V=1 matching OCCT `BRepBuilderAPI_Sewing` + `ShapeFix_Shell`/`ShapeFix_Solid` (V=1, valid), and on the beyond-tol / missing-face fixtures the native UNHEALED verdict MATCHES OCCT leaving the shell open (no valid closed solid at the same tolerance). The engine-internal `tryNativeHeal` (`src/engine/native/native_heal_hook.h`) wires native → self-verify → OCCT fallback (`src/engine/occt/occt_shapefix.cpp`); NO `cc_*` entry point, no ABI change, and `src/native/**` stays OCCT-free. **Honest scope (asymptotic, like SSI S4-f):** this slice heals the coincident-within-tolerance / degenerate / orientation defect family EXACTLY — it is a **measured win vs OCCT on the in-scope fixtures, not a guarantee** on arbitrary broken industrial B-rep. Beyond-tolerance gap bridging, missing-pcurve reconstruction, self-intersecting-wire repair, and freeform re-approximation stay OUT OF SCOPE — reported UNHEALED, deferred to OCCT `ShapeFix`, never faked. **Why this is the gating foundation for native STEP IMPORT (#3):** imported B-rep almost always arrives with coincident-within-tolerance shared edges/vertices that are NOT topologically shared, plus degenerate/orientation defects — exactly this healer's in-scope family — so a future native STEP reader can reconstruct topology through this slice and route only the residual (real gaps, missing pcurves, self-intersecting wires) to OCCT. Change `add-native-shape-healing` (7 ADDED requirements / 14 scenarios), archived to `openspec/specs/native-healing`.
- â #8 `drop-occt` â planned; **NOT reachable** at the current native ceiling. Two
  hard, research-grade multi-year dependencies remain: (1) a general robust curved
  boolean / blend kernel (arbitrary surface-surface intersection + shape healing) and
  (2) a FULL native STEP IMPORT (a full AP203/AP214/AP242 parser and B-rep
  reconstructor â the native reader today covers the elementary/B-spline subset,
  flat multi-solid compounds, single-level RIGID/UNIFORM-SCALE/MIRROR placed assemblies,
  AP242 geometry with PMI skipped, a full SPHERE + a full TORUS (TOROIDAL_SURFACE / off-axis-circle
  revolution → native Kind::Torus), the analytic-quadric SURFACE_OF_REVOLUTION reductions
  (cyl/cone/plane), and a GENERAL ELLIPSE / non-rational-B-spline generatrix SURFACE_OF_REVOLUTION
  → native rational tensor B-spline Kind::BSpline face (schema-independent,
  AP203/AP214/AP242 headers accepted); non-uniform/shear transforms, deep-nested assemblies,
  PMI semantics, complex/trimmed profiles, a PARTIAL/trimmed torus, and an off-axis-ellipse /
  skew-line-hyperboloid revolution still fall back to OCCT. IGES import/export are DESCOPED — STEP-only). Until BOTH exist, OCCT stays linked
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
| 3 | STEP **import** (AP203/214/242 parse + reconstruct; **IGES DESCOPED — STEP-only**) â **NATIVE SLICE DONE + WIDENED at the bar**: OCCT-free Part-21 reader for the elementary/B-spline AP203 subset the native writer emits + foreign OCCT-written box/cylinder, healed via #4, self-verified watertight else -> OCCT; WIDENED to multi-solid `Compound` import + a native B-spline-FACE round-trip (exact) + ELLIPSE-curve recognition + single-level RIGID/UNIFORM-SCALE/MIRROR PLACED ASSEMBLIES (placed `Compound`, schema-independent AP203/AP214/AP242) + AP242 geometry with PMI skipped + TRIMMED_CURVE edges (LINE/CIRCLE/ELLIPSE/B-spline basis, B-spline honoring its knot-span trims) + a SURFACE_OF_REVOLUTION reducing to the matching native quadric per its generatrix — a straight LINE ∥ axis → exact native cylinder, an OBLIQUE line meeting the axis → native cone, a line ⟂ axis → native plane annulus cap (all watertight == OCCT), a full SPHERE (SPHERICAL_SURFACE and on-axis-CIRCLE revolution, VERTEX_LOOP periodic-pole face → native watertight Sphere), a full TORUS (TOROIDAL_SURFACE and off-axis-circle revolution, fully-seamed EDGE_LOOP doubly-periodic face → native watertight additive Kind::Torus, face_mesher/trim untouched), and a GENERAL ELLIPSE / non-rational-B-spline generatrix revolution → native watertight rational tensor B-spline Kind::BSpline face carrying weights (tessellator/shape.h/math untouched) (`[NIMPORT] 77/77`, host 29/29 exact + `test_native_step_reader` 39 cases; multisolid rel=2.14e-16, splineface exact, 2-box assembly vol rel=3.74e-16 bboxΔ=0, uniform-scale k³ vol=2728 vs analytic (OCCT can't author a scaled location), mirror watertight vol=1216, AP242 solid vol=1000 bboxΔ=0 PMI-skipped, trimmed-curve edge + cylinder-revolution vol rel=1.27e-3 == OCCT, cone-revolution vol=522.934 rel=1.27e-3 bboxΔ=3e-15, plane-revolution vol=1568.8 rel=1.25e-3, sphere-revolution native vol=902.31 rel=2.73e-3 == OCCT, torus vol=1771.77 rel=2.68e-3 == OCCT). Residual -> OCCT: PMI SEMANTICS, non-uniform/shear transforms, deep-nested assemblies, a PARTIAL/trimmed torus, ellipse-on-quadric solids, a SURFACE_OF_REVOLUTION with no faithful native kind (an off-axis-ellipse revolution that fails the watertight self-verify / a skew-line hyperboloid — honest DECLINE; the ellipse & non-rational-B-spline generatrix general revolved surface is now NATIVE), complex/trimmed surfaces, arbitrary directly-authored rational B-spline surfaces. IGES import/export stay OCCT until `drop-occt`, then removed/stubbed (never native) | ~300â600k | uses #4 | done (widened subset) | 2â4 py |
| 4 | **Shape healing** (`ShapeFix`/`ShapeUpgrade`/`ShapeAnalysis`) — **FIRST NATIVE SLICE DONE at the bar**: tolerant sewing + vertex/tolerance unification + degenerate removal + orientation fix, verified vs OCCT `BRepBuilderAPI_Sewing`/`ShapeFix` (`[NHEAL] 4/4`; in-scope soup-cube/flipped-face heal to V=1 watertight matching OCCT; un-healable → honest UNHEALED matching OCCT). **Gates #3 import.** Residual still OCCT: beyond-tol gaps, missing pcurves, self-intersecting wires, arbitrary broken industrial B-rep | 87,647 | — | done (in-scope defect family) | 2–4 py (arbitrary B-rep) |
| 5 | **SSI + general curved booleans** (`IntPatch`/`IntWalk` 89k + BOPAlgo 76k). **SSI-ROADMAP S1 analytic + S2 subdivision seeding DONE at the bar** (S1: 17 elementary pairs vs OCCT `GeomAPI_IntSS`, `add-native-ssi-analytic` archived; S2: transversal branch recall 1.00 on freeform/skew-quadric pairs, seeds on both surfaces ≤ 3.51e-16); **S3 marching-line tracer is NEXT** | ~165k | #2 | ~w/case | **3â6 py** (clean-room) / ~1.5â3 py (port from OCCT) â *the moat* |
| 6 | Curved / variable-radius / fillet-face / concave **blends** (`ChFi3d`) | 95,710 | #5 | ~w/case | 2â4 py |
| 7 | Curved **wrap-emboss** | (composition) | #5 + curved offset | ~days | 0.2â0.5 py |
| 8 | `drop-occt` â unlink + full regression | â | 1â7 | â | small, last |

Critical path: **#2 numeric foundations (DONE) â #5 SSI (NEXT) â curved booleans â
#6 blends â #7 wrap-emboss**, with **#4 healing (FIRST NATIVE SLICE DONE at the bar) gating #3 import** as a parallel track.
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

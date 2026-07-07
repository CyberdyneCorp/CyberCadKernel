# Status

A single, honest snapshot of what is implemented and **verified**, with the
commands to reproduce it. Nothing here is claimed working unless it was actually
built/run in-repo. Per-phase detail lives in the phase status docs linked below.

_Last updated: 2026-07-07._

## Acceptance bar

The **in-repo iOS-simulator suite** is the acceptance bar: correctness is
asserted against analytic references, GPU results against CPU references, and
B-rep results against validity/watertightness/volume. Physical-device runs and
the CyberCad **app link-swap** are optional, deferred follow-ups — not gates.

## Verified in-repo

| Suite | Command | Result |
|---|---|---|
| Host unit tests (CPU-only, stub + native core) | `ctest` (host build) | **29 / 29** (**36** with `CYBERCAD_HAS_NUMSCI=ON`) — incl. `test_native_heal`, `test_native_step_reader` |
| Full `cc_*` runtime + determinism + benchmark | `scripts/run-sim-suite.sh` | **221 / 221** |
| GPU-vs-CPU parity (Metal), ray + frustum pick | `scripts/run-sim-gpu-suite.sh` | **26 / 26** |
| GPU tessellation wired into `cc_tessellate` | `scripts/run-sim-integ-suite.sh` | **26 / 26** |
| Native features (Phase 3) | `scripts/run-sim-phase3-suite.sh` | **70 / 70** (OCCT wrap-emboss #290 oracle unchanged) |
| Phase-4 native-vs-OCCT parity (math/topology/tessellation/construct/loft/sweep/thread/boolean/curved-boolean/geomcompletion) | `scripts/run-sim-native-*.sh` | all green |
| **#6 CURVED fillet** (circular cylinder↔cap rim → rolling-ball TORUS canal; CONSTANT convex+concave **and** VARIABLE-radius LINEAR-law convex via `cc_fillet_edges_variable`) vs OCCT `BRepFilletAPI` | `scripts/run-sim-native-curved-fillet.sh` | **23 / 23** (15 constant convex+concave controls unchanged + 8 variable). REAL native `activeNative=1`: constant vol rel ≤ 3.8e-3, area rel ≤ 2.1e-3; **variable** watertight, native vol matches its closed-form SWEPT removed volume rel ≤ 1.1e-3 and is DISTINCT from the OCCT evolved oracle (native-vs-OCCT-evolved reported SEPARATELY as a looser O(r′) line rel ≤ 1.2e-2, tol 6e-2), reduced vs sharp cylinder, mesh-vol==B-rep. G1 cos=1.0 at both seams **analytic from the closed-form canal normal, not mesh-sampled** |
| **#6b CURVED chamfer** (CONVEX circular cylinder↔cap rim → CONE-FRUSTUM straight bevel, **C0 NOT G1** — a chamfer is a flat bevel, not a tangent arc) vs OCCT `BRepFilletAPI_MakeChamfer` (`Add(distance, edge)`, symmetric) | `scripts/run-sim-native-curved-chamfer.sh` | **9 / 9** (REAL native `activeNative=1`: because a symmetric chamfer IS exactly a cone frustum, native-vs-OCCT vol rel ≤ 3.25e-3 (angular-deflection-bounded, not a loosened band) **and** matches the exact closed-form Pappus removed volume `π·d²·(Rc − d/3)` rel ≤ 3.25e-3, area rel ≤ 1.61e-3, watertight, mesh-vol==B-rep, reduced vs sharp; bevel normal cos=1/√2 (≈0.70710678) to BOTH the wall and the cap, explicitly **≠1** ⇒ C0 not G1. 3 fixtures Rc=5/d=1, Rc=5/d=2, Rc=4/d=1). Deferred → OCCT: non-circular / asymmetric two-distance / concave / cyl↔cyl chamfer |
| **#7 wrap-emboss** (EMBOSS + DEBOSS + NON-RECTANGULAR polygon on a cylinder lateral face) vs OCCT `cc_wrap_emboss` | `scripts/run-sim-native-wrap-emboss.sh` | **14 / 14** (REAL native `activeNative=1`: 6/6 rect-pad control byte-stable + **T1 DEBOSS** rect pocket ×2 (`boss=0`, volume SHRINKS) + **T2** emboss/deboss HEXAGON; vol rel ≤ 8e-3, area rel ≤ 1.6e-2, watertight, mesh-vol==B-rep. **T3 freeform base (cone/sphere) honestly DECLINES → OCCT** — no native builder, OCCT oracle is cylinder-only. Residual → OCCT: non-cylindrical base, self-intersecting/dense profiles, >2π footprints, `depth ≥ R`) |
| **Shape healing FIRST SLICE** (tolerant sew + vertex/tolerance unify + degenerate removal + orientation fix) vs OCCT `BRepBuilderAPI_Sewing`/`ShapeFix` | `scripts/run-sim-native-heal.sh` | **4 / 4** (in-scope soup-cube + flipped-face heal to V=1 watertight matching OCCT V=1 valid; un-healable → honest UNHEALED matching OCCT: gap 1e-2 → `GapBeyondTolerance` residual 0.0255, missing face → `OpenShell`, both OCCT valid=0 watertight=0; host `test_native_heal` 10/10) |
| **STEP import (native slice, WIDENED)** (OCCT-free Part-21 reader for the elementary/B-spline subset + multi-solid `Compound` + B-spline-FACE round-trip + ELLIPSE-curve recognition + RIGID/UNIFORM-SCALE/MIRROR PLACED ASSEMBLIES + AP203/AP214/AP242 geometry (PMI skipped), healed then self-verified watertight else → OCCT) vs OCCT `STEPControl_Reader` | `scripts/run-sim-native-step-import.sh` | **69 / 69** (native-written box vol rel 2.27e-16, cylinder rel 1.27e-3, holed-plate rel 2.90e-4, all watertight matching OCCT re-import; **FOREIGN OCCT-written box + cylinder imported NATIVELY match OCCT re-import EXACTLY, rel 0**; **multi-solid** foreign 2-solid file imports as a native Compound rel 2.14e-16 per-solid watertight; **B-spline-FACE** native solid round-trips EXACT vol 304.38; **rigid PLACED ASSEMBLY** (transform tree) imports as a native placed Compound — 2-box: 2 solids, vol rel 3.74e-16, bbox Δ=0, faces 12/12; **UNIFORM-SCALE component** via `CARTESIAN_TRANSFORMATION_OPERATOR_3D` scales the placed solid by k³ (k=2: total vol 2728 = 1000 + 216·8, component watertight) — verified against an analytic oracle because OCCT's writer cannot serialize a scaled location (it drops the scale / the iOS OCCT throws "Location with scaling transformation is forbidden"); **MIRROR component** (reflection, det<0) is orientation-complemented so it stays watertight with POSITIVE volume 1216 and reflected bbox; **AP242 file with PMI/GD&T/annotation** imports the SOLID identically to OCCT (vol 1000, bbox Δ=0, faces 6/6) with the semantic PMI + its angle/plane-angle unit contexts SKIPPED (previously fatal → now imports); **AP214** header accepted (schema-independent reader); **ELLIPSE** curve recognised but an ellipse-on-quadric solid still declines → OCCT; a `LINE`/on-axis-`CIRCLE` **`SURFACE_OF_REVOLUTION`** reduces to the matching native quadric (cyl/cone/plane/sphere); a full **`TOROIDAL_SURFACE`** (and the off-axis-circle revolution OCCT emits as one) now imports **NATIVELY** as a watertight `Kind::Torus` solid (bare doubly-periodic surface, both seams welded, no pole — sim `native torus` vol rel 2.68e-3 vs OCCT, V=2π²Rr²); a **PARTIAL/trimmed torus** and an **ELLIPSE / B-spline-generatrix** general revolution still decline → OCCT; **NON-UNIFORM/SHEAR transforms** + **PMI semantics** + Form-B `MAPPED_ITEM` + deep-nested assemblies decline → OCCT; native path provably exercised — per-face-oriented edge counts, not OCCT's unique count; host `test_native_step_reader` 36/36) |
| Numeric substrate — closest-point vs OCCT `Extrema` (NumPP/SciPP) | `scripts/run-sim-native-numerics.sh` | **22 / 22** |
| SSI **S1** analytic intersection vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi.sh` | **18 / 18** |
| SSI **S2** subdivision-seeding recall vs OCCT | `scripts/run-sim-native-ssi-seeding.sh` | **100% transversal** |
| SSI **S3** marching tracer vs OCCT `IntPatch` | `scripts/run-sim-native-ssi-marching.sh` | **11 / 11** (5 transversal `nt=0`, 9/9 branches; + S4-c graze crossed=22; + S4-d eq-cyl defer control + eq-cyl s4d fully traced; + S4-e `sphere-pole singX=2`, `cone-apex singX=1`, and `freeform-pole singX=2` fully traced) |
| SSI **S4-a/b** coincident + tangent CLASSIFICATION vs OCCT `IntAna_QuadQuadGeo`/`IntPatch` | `scripts/run-sim-native-ssi-s4.sh` | **8 / 8** (0 deferred; `FullSurfaceSame`/`TangentPoint`/`TangentCurve`/`Transversal`, on-surface ≤ ~1e-16) |
| SSI **S4-c** near-tangent MARCH-THROUGH vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4c.sh` | **7 / 7** (sphere∩offset-cyl graze S3 truncated now FULLY traced: `nearTangentGaps → 0`, 22 nodes crossed, on OCCT locus onCurve ≤ 5.6e-5 / onSurf ≤ 1.3e-5; equal-cyl branch saddle STILL defers, `crossed=0`; 5 transversal pairs `nt=0`) |
| SSI **S4-d** branch points (self-crossing locus) vs OCCT `IntPatch`/`GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4d.sh` | **8 / 8** (Steinmetz bicylinder now FULLY traced: `branchPts=2` localized at `(0,±1,0)`, 4 arms → 2 crossing ellipses, `nearTangentGaps=0`, onCurve ≤ 1.74e-6 / onSurf ≤ 1.07e-8; isolated `TangentPoint` STILL ends, no arms; S4-c graze still `crossed=22`; flag-off eq-cyl still defers) |
| SSI **S4-e** chart singularities (analytic sphere pole / cone apex + FREEFORM NURBS pole) vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-marching.sh` (S4-e cases) | **3 / 3** (sphere great circle crossing both poles S3-truncated at half loop now FULLY traced: `singX=2`, `nearTangentGaps=0`, closed, `len` 6.2829 vs OCCT 6.2832, on locus + both surfaces ≤ 1.51e-7; double-cone∩plane line through apex S3 step-collapsed now FULLY traced both nappes: `singX=1`, 159 nodes, `v∈[−2,+2]`, on-surface ≤ 6.79e-16; **freeform** NURBS-sphere (`uPeriod==0` collapsed-control-row pole, no analytic meridian map) ∩ plane now FULLY traced via the `freeformChartInvert` far-side re-seed: `singX=2`, closed, `len` 6.2829 vs OCCT 6.2832, on both surfaces ≤ 1.51e-7; finite cylinder `v`-cap still exits `BoundaryExit`; a genuine collapsed-row endpoint (Bézier cone-tip on the domain boundary — no far side) still DEFERS `NearTangent`; the **curve cusp** declined by the IFT argument — routed to S4-c/S4-d/OCCT) |
| SSI **S5-a/b/c/d/e/f** curved boolean vs OCCT `BRepAlgoAPI` | `scripts/run-sim-native-ssi-curved-boolean.sh` | **24 / 24** (native-pass=18: drill cyl∩cyl COMMON/FUSE/CUT + **sphere∩sphere COMMON/FUSE/CUT eq+uneq (op-set COMPLETE 3/3, FUSE ΔV ≤ 8.3e-4 / CUT ΔV ≤ 9.3e-4 vs analytic+OCCT)** + **branched Steinmetz COMMON/FUSE/CUT** (`16R³/3`+incl-excl + OCCT, ΔV ≤ 8.75e-4) + **coaxial cone∩cylinder COMMON/FUSE/CUT (op-set COMPLETE 3/3)** (CONE family; dual oracle `V_frustum` inclusion-exclusion + OCCT — COMMON volN=19.107/volO=19.111 ΔV=2.03e-4 ΔA=9.89e-5, FUSE volN=41.618/volO=41.626 (analytic 41.62610) ΔV=2.04e-4 ΔA=1.13e-4, CUT volN=13.349/volO=13.352 (analytic 13.35177) ΔV=2.03e-4 ΔA=1.02e-4) + **coaxial cone∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3)** (S5-f, CONE∩SPHERE family; single S1-analytic circle seam, dual oracle `V_frustum + V_spherical-segment` inclusion-exclusion + OCCT — COMMON volN=5.2546/volO=5.2558 ΔV=2.41e-4 ΔA=1.28e-4, FUSE volN=60.686/volO=60.718 ΔV=5.22e-4 ΔA=2.61e-4, CUT (cone−sphere) volN=27.202/volO=27.207 ΔV=1.96e-4 ΔA=1.34e-4; sphere−cone declines → OCCT), all wt, ΔV ≤ 9e-4; 6 honest fallbacks incl. sphere×box, cone×box) |
| SSI **S4-f** completeness + loop-robustness vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4f.sh` | **3 / 3** (**MEASURED per-fixture, not a completeness proof**: (A) small loop inside one default 1/32 leaf cell recovered by the adaptive critic — recall **0.50→1.00** (traced 1→2, recovered=1, floor **1/128**, dry), on both surfaces ≤ 6.87e-13; (D) many-small-loops adversarial pair recall **0.25→1.00** (traced 1→4, recovered=3, floor **1/48**, dry), ≤ 8.85e-12; (C) Gerono figure-eight self-crossing detected+traced-through — `selfInt=1` transverse near origin, `branchPts=0`, guard-off byte-identical, ≤ 1.97e-11. **Residual acknowledged: below the reported floor a smaller loop can still be missed — recall→1 is scoped to the fixture at that floor.**) |
| Spec validation | `openspec validate --all --strict` | **30 / 30** |

Highlights (measured, not asserted-trivially):

- **Determinism:** parallel booleans + meshing are **bit-identical** to serial
  (mesh hash + exact volume + tri count), stable across 8 runs.
- **Thread boolean (#286):** fine multi-turn fuse/cut completes in **~4.3–4.4 s**
  (< 8 s budget), valid + watertight — no minutes-long OCCT hang.
- **GPU tessellation:** box routes 6/6 faces to the GPU (area 600.000004 vs OCCT
  600.000000); holed slab routes 4 GPU / 3 OCCT-fallback, bbox+area+volume match.
- **G2 fillet (#284):** measured seam curvature gap **0.0188** (≤ 0.05 tol) vs
  stock G1 baseline **0.31** — a real, measured curvature improvement.

## Per-phase status

| Phase | Change(s) | Status |
|---|---|---|
| **0 — Foundation** | `add-kernel-foundation` | ✅ complete at acceptance bar |
| **1 — Multi-core** | `accelerate-multicore-occt` | ✅ complete at acceptance bar |
| **2 — GPU (Metal)** | `add-metal-compute-backend` ✅ · `add-gpu-tessellation` ✅ · `add-gpu-spatial-acceleration` ✅ | ✅ complete at acceptance bar; optional `cc_*` pick/cull facade entry deferred |
| **3 — Missing features** | `add-reference-geometry` ✅ · `add-robust-wrap-emboss` ✅ · `add-robust-thread-boolean` ✅ · `add-g2-blend-fillet` ✅ · `add-full-round-fillet` ✅ | ✅ 5/5 full; full-round covers all planar walls (curved neighbours = documented residual) |
| **4 — Native rewrite** | math · topology · tessellation · construction · planar+box∩cyl booleans · planar blends · STEP export · numeric foundations (NumPP/SciPP) · SSI S1+S2+S3 · SSI S4-a/b (coincident-region + tangent-contact classification) · SSI S4-c (first near-tangent MARCH-THROUGH slice) · SSI S4-d (first branch-point slice: Steinmetz self-crossing localized + routed) · SSI S4-e (chart singularities: analytic sphere parametric pole + cone apex crossed, plus the FREEFORM NURBS collapsed-row pole crossed via `freeformChartInvert`; curve cusp declined by IFT) · SSI S4-f (robust TRUE-RETURN loop-closure + self-intersection guard + adaptive completeness-critic re-seed — MEASURED recall floor, not a proof) · SSI S5-a/b/c/d/e/f (curved booleans: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3) + branched Steinmetz COMMON/FUSE/CUT (3/3) + coaxial cone∩cylinder COMMON/FUSE/CUT (op-set COMPLETE 3/3) + coaxial cone∩sphere COMMON/FUSE/CUT (op-set COMPLETE 3/3), native-pass=18) · **#6 curved fillet (circular cyl↔cap rim, CONVEX + CONCAVE → rolling-ball TORUS canal, G1-tangent; + VARIABLE-radius LINEAR-law convex circular cyl↔cap fillet via `cc_fillet_edges_variable` → swept-radius canal, G1 at both varying-radius seams) · #6b curved chamfer (CONVEX circular cyl↔cap rim → CONE-FRUSTUM straight bevel via `cc_chamfer_edges`, C0 at the chamfer angle NOT G1, symmetric-chamfer-IS-a-frustum so vol rel ≤ 3.25e-3 vs OCCT `BRepFilletAPI_MakeChamfer` + exact Pappus `π·d²·(Rc−d/3)`) · #7 wrap-emboss (rectangular pad on a cylinder lateral face)** · **shape-healing FIRST SLICE (tolerant sew + vertex/tolerance unification + degenerate removal + orientation fix — internal, verified vs OCCT `BRepBuilderAPI_Sewing`/`ShapeFix`)** · **STEP import (native slice, WIDENED) (OCCT-free Part-21 reader for the AP203 elementary/B-spline subset + foreign OCCT-written box/cylinder + multi-solid Compound + B-spline-FACE round-trip + ELLIPSE-curve recognition + TRIMMED_CURVE edges + SURFACE_OF_REVOLUTION→cylinder/cone/plane + a FULL SPHERE (SPHERICAL_SURFACE and on-axis-circle SURFACE_OF_REVOLUTION → native watertight Sphere via the VERTEX_LOOP periodic-pole face) + RIGID/UNIFORM-SCALE/MIRROR PLACED ASSEMBLIES + AP203/AP214/AP242 geometry with PMI skipped, healed + self-verified watertight else → OCCT; host round-trip exact + sim parity 69/69)** | ◐ **substantially native (planar/analytic + SSI-driven curved booleans + S4 degeneracy classification + near-tangent march-through + first branch-point routing + sphere-pole/cone-apex chart-singularity crossing + robust closure/self-intersection + measured completeness floor + first curved-feature slices #6 fillet/#6b chamfer/#7 wrap-emboss + shape-healing first slice + STEP import widened slice)**; curved tail (SSI S4-d general/freeform + S4-e general/freeform + S4-f general small-loop residual below the reached floor + self-intersection arc-splitting/topology repair · wider curved booleans/blends incl. general non-Steinmetz branched + transversal/apex cone pairs + cone∩cone + the TWO-circle / apex-crossing / transversal cone∩sphere crossings (the coaxial cone∩cylinder AND coaxial cone∩sphere single-crossing op-sets COMMON/FUSE/CUT are now COMPLETE 3/3 native) · non-linear-law/concave-variable/non-circular-crease/cyl↔cyl fillets (constant convex + concave circular AND variable-radius linear-law convex circular now native) + curved-edge chamfer (CONVEX circular cyl↔cap → CONE-FRUSTUM straight bevel, C0, now native; non-circular / asymmetric two-distance / concave / cyl↔cyl chamfer stays OCCT) · deboss/non-rectangular/non-cylindrical wrap-emboss · **STEP import beyond the widened subset** (PMI SEMANTICS, non-uniform/shear transforms, deep-nested assemblies, torus, ellipse-on-quadric solids, complex/trimmed profiles, rational B-splines → OCCT; multi-solid Compound + B-spline-FACE round-trip + rigid/uniform-scale/mirror placed assemblies + AP242 geometry with PMI skipped now native; all IGES stays OCCT) · **healing RESIDUAL** (beyond-tol gaps, missing pcurves, self-intersecting wires, arbitrary broken industrial B-rep — the coincident-within-tolerance/degenerate/orientation first slice is native)) keeps OCCT linked; drop-occt (#8) BLOCKED (≈9–18 py) |

Detail: [STATUS-phase-0-1.md](STATUS-phase-0-1.md) ·
[STATUS-phase-2.md](STATUS-phase-2.md) · [STATUS-phase-3.md](STATUS-phase-3.md) ·
[STATUS-phase-4.md](STATUS-phase-4.md) · [../openspec/SSI-ROADMAP.md](../openspec/SSI-ROADMAP.md).

## Open / deferred (honest)

- **Phase 2 spatial tail:** GPU **frustum-pick** parity + **repeat-run
  determinism** now VERIFIED on the sim (GPU pick suite 26/26; frustum set == CPU
  set, sorted ascending, byte-identical ×8 for ray + frustum). The only remaining
  item is the **OPTIONAL additive `cc_*` pick/cull facade entry** (app-facing, out
  of scope — no OCCT-side pick path exists in the facade today).
- **Full-round fillet (#285):** rolling-ball blend proven for ALL planar walls
  (parallel AND non-parallel dihedral — non-parallel fixture n_L·n_R=-0.7241,
  43.60°, G1 cos=1.000000, middle consumed, valid+watertight). Only truly CURVED
  (non-planar) neighbours fall back to a valid standard fillet, by design.
- **G2 fillet (#284):** non-straight seams defer to a standard fillet.
- **Thread boolean determinism:** reproducible within rel 2e-4, not bit-exact
  (parallel `BOPAlgo`).
- **On-device runs** (physical Apple silicon) and the **CyberCad app link-swap**
  — verified/derived on the simulator only; both optional by the acceptance bar.

## Reproduce everything

```sh
# Host CPU-only build + unit tests
cmake -S . -B build -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF
cmake --build build && (cd build && ctest --output-on-failure)

# iOS-simulator integrated suites (a booted simulator is required)
bash scripts/run-sim-suite.sh
bash scripts/run-sim-gpu-suite.sh
bash scripts/run-sim-integ-suite.sh
bash scripts/run-sim-phase3-suite.sh

# Specs
openspec validate --all --strict
```

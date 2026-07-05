# Status

A single, honest snapshot of what is implemented and **verified**, with the
commands to reproduce it. Nothing here is claimed working unless it was actually
built/run in-repo. Per-phase detail lives in the phase status docs linked below.

_Last updated: 2026-07-05._

## Acceptance bar

The **in-repo iOS-simulator suite** is the acceptance bar: correctness is
asserted against analytic references, GPU results against CPU references, and
B-rep results against validity/watertightness/volume. Physical-device runs and
the CyberCad **app link-swap** are optional, deferred follow-ups — not gates.

## Verified in-repo

| Suite | Command | Result |
|---|---|---|
| Host unit tests (CPU-only, stub + native core) | `ctest` (host build) | **26 / 26** (**33** with `CYBERCAD_HAS_NUMSCI=ON`) |
| Full `cc_*` runtime + determinism + benchmark | `scripts/run-sim-suite.sh` | **221 / 221** |
| GPU-vs-CPU parity (Metal), ray + frustum pick | `scripts/run-sim-gpu-suite.sh` | **26 / 26** |
| GPU tessellation wired into `cc_tessellate` | `scripts/run-sim-integ-suite.sh` | **26 / 26** |
| Native features (Phase 3) | `scripts/run-sim-phase3-suite.sh` | **70 / 70** |
| Phase-4 native-vs-OCCT parity (math/topology/tessellation/construct/loft/sweep/thread/boolean/curved-boolean/geomcompletion) | `scripts/run-sim-native-*.sh` | all green |
| Numeric substrate — closest-point vs OCCT `Extrema` (NumPP/SciPP) | `scripts/run-sim-native-numerics.sh` | **22 / 22** |
| SSI **S1** analytic intersection vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi.sh` | **18 / 18** |
| SSI **S2** subdivision-seeding recall vs OCCT | `scripts/run-sim-native-ssi-seeding.sh` | **100% transversal** |
| SSI **S3** marching tracer vs OCCT `IntPatch` | `scripts/run-sim-native-ssi-marching.sh` | **10 / 10** (5 transversal `nt=0`, 9/9 branches; + S4-c graze crossed=22; + S4-d eq-cyl defer control + eq-cyl s4d fully traced; + S4-e `sphere-pole singX=2` and `cone-apex singX=1` fully traced) |
| SSI **S4-a/b** coincident + tangent CLASSIFICATION vs OCCT `IntAna_QuadQuadGeo`/`IntPatch` | `scripts/run-sim-native-ssi-s4.sh` | **8 / 8** (0 deferred; `FullSurfaceSame`/`TangentPoint`/`TangentCurve`/`Transversal`, on-surface ≤ ~1e-16) |
| SSI **S4-c** near-tangent MARCH-THROUGH vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4c.sh` | **7 / 7** (sphere∩offset-cyl graze S3 truncated now FULLY traced: `nearTangentGaps → 0`, 22 nodes crossed, on OCCT locus onCurve ≤ 5.6e-5 / onSurf ≤ 1.3e-5; equal-cyl branch saddle STILL defers, `crossed=0`; 5 transversal pairs `nt=0`) |
| SSI **S4-d** branch points (self-crossing locus) vs OCCT `IntPatch`/`GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4d.sh` | **8 / 8** (Steinmetz bicylinder now FULLY traced: `branchPts=2` localized at `(0,±1,0)`, 4 arms → 2 crossing ellipses, `nearTangentGaps=0`, onCurve ≤ 1.74e-6 / onSurf ≤ 1.07e-8; isolated `TangentPoint` STILL ends, no arms; S4-c graze still `crossed=22`; flag-off eq-cyl still defers) |
| SSI **S4-e** chart singularities (sphere pole / cone apex) vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-marching.sh` (S4-e cases) | **2 / 2** (sphere great circle crossing both poles S3-truncated at half loop now FULLY traced: `singX=2`, `nearTangentGaps=0`, closed, `len` 6.2829 vs OCCT 6.2832, on locus + both surfaces ≤ 1.51e-7; double-cone∩plane line through apex S3 step-collapsed now FULLY traced both nappes: `singX=1`, 159 nodes, `v∈[−2,+2]`, on-surface ≤ 6.79e-16; finite cylinder `v`-cap still exits `BoundaryExit`, unverifiable pole/apex still DEFERS) |
| SSI **S5-a/b/c/d** curved boolean vs OCCT `BRepAlgoAPI` | `scripts/run-sim-native-ssi-curved-boolean.sh` | **18 / 18** (native-pass=6: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON eq/uneq + **branched Steinmetz COMMON** (`16R³/3` + OCCT, ΔV=8.75e-4), all wt, ΔV ≤ 9e-4; 12 honest fallbacks incl. Steinmetz fuse/cut) |
| SSI **S4-f** completeness + loop-robustness vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi-s4f.sh` | **3 / 3** (**MEASURED per-fixture, not a completeness proof**: (A) small loop inside one default 1/32 leaf cell recovered by the adaptive critic — recall **0.50→1.00** (traced 1→2, recovered=1, floor **1/128**, dry), on both surfaces ≤ 6.87e-13; (D) many-small-loops adversarial pair recall **0.25→1.00** (traced 1→4, recovered=3, floor **1/48**, dry), ≤ 8.85e-12; (C) Gerono figure-eight self-crossing detected+traced-through — `selfInt=1` transverse near origin, `branchPts=0`, guard-off byte-identical, ≤ 1.97e-11. **Residual acknowledged: below the reported floor a smaller loop can still be missed — recall→1 is scoped to the fixture at that floor.**) |
| Spec validation | `openspec validate --all --strict` | **29 / 29** |

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
| **4 — Native rewrite** | math · topology · tessellation · construction · planar+box∩cyl booleans · planar blends · STEP export · numeric foundations (NumPP/SciPP) · SSI S1+S2+S3 · SSI S4-a/b (coincident-region + tangent-contact classification) · SSI S4-c (first near-tangent MARCH-THROUGH slice) · SSI S4-d (first branch-point slice: Steinmetz self-crossing localized + routed) · SSI S4-e (first chart-singularity slice: sphere parametric pole + cone apex crossed) · SSI S4-f (robust TRUE-RETURN loop-closure + self-intersection guard + adaptive completeness-critic re-seed — MEASURED recall floor, not a proof) · SSI S5-a/b/c/d (curved booleans: drill cyl∩cyl COMMON/FUSE/CUT + sphere∩sphere COMMON + branched Steinmetz COMMON, native-pass=6) | ◐ **substantially native (planar/analytic + SSI-driven curved booleans + S4 degeneracy classification + near-tangent march-through + first branch-point routing + sphere-pole/cone-apex chart-singularity crossing + robust closure/self-intersection + measured completeness floor)**; curved tail (SSI S4-d general/freeform + S4-e general/freeform + S4-f general small-loop residual below the reached floor + self-intersection arc-splitting/topology repair · wider curved booleans/blends incl. Steinmetz fuse/cut + sphere fuse/cut + general branched · import · healing) keeps OCCT linked; drop-occt (#8) BLOCKED (≈9–18 py) |

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

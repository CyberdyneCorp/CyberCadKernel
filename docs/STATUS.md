# Status

A single, honest snapshot of what is implemented and **verified**, with the
commands to reproduce it. Nothing here is claimed working unless it was actually
built/run in-repo. Per-phase detail lives in the phase status docs linked below.

_Last updated: 2026-07-02._

## Acceptance bar

The **in-repo iOS-simulator suite** is the acceptance bar: correctness is
asserted against analytic references, GPU results against CPU references, and
B-rep results against validity/watertightness/volume. Physical-device runs and
the CyberCad **app link-swap** are optional, deferred follow-ups — not gates.

## Verified in-repo

| Suite | Command | Result |
|---|---|---|
| Host unit tests (CPU-only, stub + native core) | `ctest` (host build) | **23 / 23** (**28** with `CYBERCAD_HAS_NUMSCI=ON`) |
| Full `cc_*` runtime + determinism + benchmark | `scripts/run-sim-suite.sh` | **221 / 221** |
| GPU-vs-CPU parity (Metal), ray + frustum pick | `scripts/run-sim-gpu-suite.sh` | **26 / 26** |
| GPU tessellation wired into `cc_tessellate` | `scripts/run-sim-integ-suite.sh` | **26 / 26** |
| Native features (Phase 3) | `scripts/run-sim-phase3-suite.sh` | **70 / 70** |
| Phase-4 native-vs-OCCT parity (math/topology/tessellation/construct/loft/sweep/thread/boolean/curved-boolean/geomcompletion) | `scripts/run-sim-native-*.sh` | all green |
| Numeric substrate — closest-point vs OCCT `Extrema` (NumPP/SciPP) | `scripts/run-sim-native-numerics.sh` | **22 / 22** |
| SSI **S1** analytic intersection vs OCCT `GeomAPI_IntSS` | `scripts/run-sim-native-ssi.sh` | **18 / 18** |
| SSI **S2** subdivision-seeding recall vs OCCT | `scripts/run-sim-native-ssi-seeding.sh` | **100% transversal** |
| SSI **S3** marching tracer vs OCCT `IntPatch` | `scripts/run-sim-native-ssi-marching.sh` | **5 / 5** (9/9 branches) |
| SSI **S5-a** curved boolean vs OCCT `BRepAlgoAPI` | `scripts/run-sim-native-ssi-curved-boolean.sh` | **12 / 12** (native-pass=1: drill cyl∩cyl COMMON, wt, ΔV 8.1e-4; 11 honest fallbacks) |
| Spec validation | `openspec validate --all --strict` | **26 / 26** |

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
| **4 — Native rewrite** | math · topology · tessellation · construction · planar+box∩cyl booleans · planar blends · STEP export · numeric foundations (NumPP/SciPP) · SSI S1+S2+S3 · SSI S5-a (first curved boolean: drill cyl∩cyl COMMON) | ◐ **substantially native (planar/analytic + first SSI-driven curved boolean)**; curved tail (SSI S4 · wider curved booleans/blends · import · healing) keeps OCCT linked; drop-occt (#8) BLOCKED (≈9–18 py) |

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

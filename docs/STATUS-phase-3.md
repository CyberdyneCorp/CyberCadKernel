# CyberCadKernel — Phase 3 status (native algorithms OCCT lacks)

Honest, verification-anchored snapshot of Phase 3 (`add-reference-geometry`,
`add-robust-wrap-emboss`, `add-robust-thread-boolean`, `add-full-round-fillet`,
`add-g2-blend-fillet`). Nothing below is claimed unless it was actually built and
run in this environment. The acceptance bar for Phase 3 (per each change's tasks)
is the **booted iOS simulator** with OCCT linked (`cc_brep_available()==1`),
every result asserted against a REAL geometric property — `BRepCheck_Analyzer::IsValid`,
watertightness, volume sign, unit-normal/direction within `1e-9`, G1-tangency
(normal dot), or a MEASURED curvature gap vs the G1 baseline. On-device runs and
app integration are explicit follow-ups.

Date: 2026-07-02 · Branch: `main`.

## TL;DR

- **VERIFIED here (iOS simulator, OCCT linked):** the Phase-3 suite
  (`scripts/run-sim-phase3-suite.sh`) is **65 passed, 0 failed, 1 deferred**, exit 0.
- **Full (property proven):** reference-geometry, robust wrap-emboss,
  robust thread-boolean, and G2-blend all hit their real geometric bars.
- **Full with one documented fallback:** full-round-fillet proves the true
  rolling-ball blend for tangent/parallel-wall strips; the **1 DEFER** is the
  non-parallel-wall case (22.62° off-parallel), which honestly falls back to a
  VALID standard edge fillet (middle face NOT consumed).
- **Regressions:** none. `scripts/run-sim-suite.sh` stays **221/221** (Phase-3
  sources are in the script's SKIP list; existing paths unchanged). Host stub
  build (`CYBERCAD_HAS_OCCT=OFF`, `CYBERCAD_HAS_METAL=OFF`): **CTest 7/7**, the
  new `cc_*` entry points compile+link as safe no-ops.

## Per-feature status

| Capability | Status | What was verified (numbers, on iOS sim / OCCT) | What falls back |
|---|---|---|---|
| **reference-geometry** | **full** | 21/21 checks. Plane/axis from points, offset plane, plane-from-face, axis-from-edge/face. 6/6 box faces resolve planes, unit normals within `1e-9`; 12/12 box edges resolve axes; 4 vertical ±Z edges; cylinder axis unit ±Z. Analytic point-only constructors exact within `1e-9`. | Non-planar face / non-linear edge / colinear-or-coincident points / zero-length normal → return `0` (guards hold, no crash). Derived constructors in host stub return `0` (unsupported). |
| **wrap-emboss** | **full** | V_base=12566.370614. Emboss (`boss=1`) valid + watertight (naked=0, tris=156), V_after=12671.970610, Δ=105.599996 (nominal 96.0). Deboss (`boss=0`) valid + watertight, V_after=12479.970612, Δ=86.400002. Reproducible: v1==v2=12671.970610. Wide high-curvature emboss valid + watertight, Δ=369.600009 (nominal 336.0), naked=0. | Sewn pad builder → dense-then-coarse ThruSections pad → return `0` (records `cc_last_error`). Reported numbers confirm the valid-watertight property on a high-curvature profile but do not isolate sewn-vs-coarse path. Host stub: no-op `0`. |
| **thread-boolean** | **full** (determinism within tolerance) | shaft V=1056.0150, thread V=76.8310. FUSE (`op=0`): wall-clock=4.3778 s (<8 s budget), valid, watertight (0 free / 0 non-manifold over 2788 tris), V_after=1085.8188 (Δ=+29.8039, external thread adds → V_after > V_shaft). CUT (`op=1`): wall-clock=4.4817 s (<8 s), valid, watertight (0/0 over 3210 tris), removed Δ=69.6626 ≈ V_thread 76.8310 (internal thread removes → V_after < base). Naive `cc_boolean` NOT run (known to hang). | Budget-exceed or invalid → cancel and return `0` (thread + shaft stay separate). No case deferred here (both finished < 8 s, valid). **Determinism is within tolerance, NOT bit-exact:** repeat FUSE \|ΔV\|=0.2004 mm³ (rel 0.0002) — expected for parallel `BOPAlgo` (`SetRunParallel(true)`). |
| **full-round-fillet** | **full for tangent/parallel walls · deferred fallback for non-parallel walls** | 10 checks PASS. Rolling-ball vol=782.8319 (== expect); middle face CONSUMED; blend is a cylinder; axis equidistant from both walls (dLeft=2.0126, dRight=2.0126, rTop=2.0126, axisX=0.0000); G1-tangent at BOTH seams dot(left)=dot(right)=1.000000 (tol cos1°=0.999848); deterministic v1==v2=782.831853, area a1==a2=619.398224; single-arg auto-detect matches the explicit three-face form. | **1 DEFER (measured):** non-parallel walls (off-parallel 22.62°, n_L·n_R=-0.9231) fall back to a VALID standard edge fillet — the flat middle face is NOT consumed (rebuilt vol=1597.844). True full-round is proven only for tangent/parallel-wall strips. Host stub: no-op `0`. |
| **g2-blend** | **full** (G2 achieved, measured) | Valid + watertight solid, corner removed V=7975.437127 (box=8000.0); watertight mesh vol=7974.944311 vs brep=7975.437127. Seam curvature gap G2=0.018835 within G2 tol=0.050000 (1/r=0.333333); stock G1 baseline=0.309740 FAILS the bar (> 0.05); G2 measurably smaller than G1. Deterministic: dV=0, dBBox=0, dGap=0 (bit-exact). | If G2 gap > tol → do NOT claim G2, report the measured gap and defer (NOT triggered here). If blend invalid → fall back (no body / G1 fillet) and defer with the number. Host stub: no-op `0`. |

## Regression / no-harm checks (VERIFIED here)

- **Existing OCCT-only suite** — `scripts/run-sim-phase3-suite.sh` is separate;
  `scripts/run-sim-suite.sh` (Phase-0/1 `full_suite.cpp` + `checks_*.cpp`) stays
  **221 passed, 0 failed**, with Phase-3 sources explicitly in the script's SKIP
  list. Determinism (serial==parallel, bit-reproducible) and IGES/STEP round-trip
  checks remain green. Phase-3 additions are additive only and do not alter
  existing paths.
- **Host stub build** — configured fresh with `/opt/homebrew/opt/llvm/bin/clang++`
  (Clang 22.1.3), `-DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF`; all 7 test
  targets built, CTest **100% (7/7)** (`test_registry`, `test_guard`,
  `test_scheduler`, `test_compute_backend`, `test_parallel_policy`,
  `test_parallel_toggle`, `test_abi`). The new `cc_*` entry points compile and
  link as safe runtime no-ops; `test_abi` still matches the app header
  (additive-only). No fixes were required.

## Reproduce commands

```sh
cd /Users/leonardoaraujo/work/CyberCadKernel

# Phase-3 property suite on the iOS simulator (OCCT linked)
bash scripts/run-sim-phase3-suite.sh   # expect: "65 passed, 0 failed, 1 deferred", exit 0

# Phase-0/1 baseline — full cc_* suite unchanged (Phase-3 sources skipped)
bash scripts/run-sim-suite.sh          # expect: "== 221 passed, 0 failed ==", exit 0

# Host (no-OCCT) regression — CPU-only path unaffected
cmake -S . -B build \
  -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++ \
  -DCYBERCAD_HAS_OCCT=OFF -DCYBERCAD_HAS_METAL=OFF
cmake --build build
cd build && ctest --output-on-failure  # expect: 100% tests passed, 7/7

# OpenSpec validation
openspec validate --all --strict       # expect: all changes pass
```

## VERIFIED vs FOLLOW-UP

### VERIFIED in this environment (iOS simulator, OCCT linked)

| Fact | Evidence |
|---|---|
| Reference datum planes/axes correct within `1e-9`; guards reject degenerate input | phase3 suite: 21/21 (6/6 faces, 12/12 edges, cyl axis ±Z) |
| Robust wrap-emboss produces valid + watertight boss and deboss solids, correct volume sign, reproducible | phase3 suite: emboss/deboss valid+watertight, V deltas vs nominal, v1==v2 |
| Robust thread↔shaft boolean completes < 8 s, valid + watertight, correct volume sign, naive path not run | phase3 suite: FUSE 4.3778 s, CUT 4.4817 s, 0 free/0 non-manifold |
| Full-round rolling-ball blend consumes the middle face, cylinder blend, G1-tangent (dot=1.000000) both seams, deterministic | phase3 suite: 10 checks + auto-detect |
| G2 blend valid + watertight; MEASURED curvature gap 0.018835 within tol 0.05; G1 baseline 0.309740 fails; G2 < G1; bit-exact determinism | phase3 suite: G2 vs G1 measured |
| No regression to existing paths; host stub compiles/links new `cc_*` as no-ops | `run-sim-suite.sh` 221/221; host CTest 7/7 |

### FOLLOW-UP / documented fallbacks

| Item | Why | Task |
|---|---|---|
| **Full-round for non-parallel walls** | Off-parallel 22.62° (n_L·n_R=-0.9231) cannot be resolved to a tangent rolling-ball blend; falls back to a VALID standard edge fillet (middle face not consumed, vol=1597.844). Documented deferral, not a gap in the proven tangent-wall case | `full-round-fillet` 4.5 |
| **Thread-boolean bit-exact determinism** | Reproducible only within rel ~2e-4 (\|ΔV\|=0.2004 mm³) because parallel `BOPAlgo` is used; volume/bbox are stable to tolerance, not bit-identical | `thread-boolean` 4.5 |
| **Wrap-emboss sewn-vs-coarse path isolation** | The valid-watertight property is proven on a high-curvature profile, but the reported numbers do not isolate whether the sewn pad or the coarse ThruSections fallback produced it | `wrap-emboss` 4.3 |
| **On-device run** on physical Apple silicon | Everything above ran on the booted **simulator** with OCCT; nothing on hardware. The Phase-3 acceptance bar is the simulator, so this is optional | all Phase-3 changes |
| **App link-swap / `cc_*` wiring in the CyberCad app** | Requires the app project; out of scope for the kernel repo | all Phase-3 changes |

Reference-geometry, robust wrap-emboss, robust thread-boolean, and G2-blend are
**complete at the Phase-3 acceptance bar** (real geometric property proven on the
sim). Full-round-fillet is complete for the tangent/parallel-wall case with a
single, honestly-documented fallback (non-parallel walls → valid edge fillet), so
it stays **◐ in progress** in `ROADMAP.md`; the other four flip to **✅**.

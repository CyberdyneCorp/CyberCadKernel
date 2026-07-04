# Evaluation — NumPP / SciPP as the kernel's numeric foundation

**Verdict: GO WITH HARDENING.** Adopt NumPP + SciPP as the point-solver / dense-linalg
substrate for the native geometry kernel. They are mature, self-contained, and
iOS-simulator-clean, and they retire most of the *generic numeric-foundations* line.
They are **not** a drop-in SSI kernel: the robustness-critical parts of surface-surface
intersection (marching, seeding, tangent/coincident degeneracy) are demonstrably still
ours to write.

Every claim below is tied to a probe result. Anything not directly exercised is flagged
**UNVERIFIED**.

Toolchain used for all probes:
- Host: `/opt/homebrew/opt/llvm/bin/clang++` = Homebrew clang 22.1.8 (**libc++**), `-std=c++20`.
- CMake: `/Users/leonardoaraujo/anaconda3/bin/cmake` 3.31.4.
- GCC 16 (**libstdc++**) available at `/opt/homebrew/opt/gcc/bin/g++-16` (used only to isolate a libc++ gap).
- iOS: `xcrun --sdk iphonesimulator clang++ -target arm64-apple-ios16.0-simulator -stdlib=libc++`.

No NumPP / SciPP / kernel source was modified in any probe.

---

## 1. What each library is + verified maturity

### NumPP — NumPy-style N-D array + reference linalg engine (the base layer)
- **Builds clean.** CPU-only (all `NUMPP_WITH_*` OFF), tests ON: configure clean,
  build 100% no errors, `ctest` 1/1 (103.8 s). Installed as **NumPP 1.6.0**.
- **Full suite passes:** direct binary = **959/959 cases, 2607 checks, 0 failures** —
  exactly matches the project README. Zero required deps (its own reference
  `solve`/`lstsq`/`inv`; no external BLAS/LAPACK needed).
- **Maturity: MATURE**, and iOS-portable (see §2).

### SciPP — SciPy-style numerics on top of NumPP (optimize / linalg / spatial / integrate / special / stats …)
- **Full library does NOT build under host clang/libc++.** Hard failure in
  `src/special/{orthogonal,bessel,expint,spherical_bessel}.cpp`: `no member named
  'legendre'/'laguerre'/'cyl_bessel_j'/'cyl_neumann'/'cyl_bessel_i'/'cyl_bessel_k'/'expint'
  in namespace 'std'`.
- **Root cause VERIFIED — a libc++ stdlib gap, not a SciPP bug.** These are C++17
  ISO-29124 special-math functions that libc++ does not implement. All 4 offending TUs
  compile clean under **GCC-16/libstdc++** (compiled successfully to prove it). The 10
  offending `std::` calls are grep-confirmed **entirely confined to `src/special/`**; no
  numeric module touches them.
- **The kernel-relevant numeric surface builds clean under host libc++:** 18/18 TUs —
  `optimize/{roots_scalar,minimize,minimize_scalar,least_squares,nnls,linprog}`,
  `linalg/{basic,decomp,eigen,matfuncs,special}`, `spatial/{distance,kdtree,geometry,rotation}`,
  `integrate/{differentiate,quadrature,solve_ivp}`.
- **SciPP's own SciPy-oracle tests (numeric subset, UNMODIFIED, frozen golden data)
  pass:** main + optimize + optimize_lp + linalg + spatial + integrate =
  **29/29 cases, 841 checks, 0 failures.**
- **UNVERIFIED:** the README headline "7503 oracle checks / all 12 subpackages, 0
  divergences" — `special`/`stats` could not be compiled under libc++ on this toolchain,
  so only the numeric 841-check subset was runnable. It passed; the remaining ~6600 checks
  are unverified here.
- **Maturity: MATURE for the numeric surface** the kernel needs; **`special`/`stats`
  unverified on this toolchain** (and not needed by the kernel).

**Net maturity:** the numeric substrate we would actually consume (root-finding, dense
linalg, closest-point, quadrature/ODE) is oracle-validated and passes 3 independent gates
(SciPP's own tests, a functional probe, iOS link). The only build friction is a
**config-level** libc++ special-math gap, not a source defect.

---

## 2. iOS-sim buildability — can they live in the kernel?

**Verdict: YES.** The numeric foundation compiles, archives, and links clean for
`arm64-apple-ios16.0-simulator` with zero source-level blockers.

- **FULL NumPP (all 74 TUs, CPU-only incl. `backend/*_null.cpp` stubs) + SciPP
  optimize(6) + linalg(5): 77/77 TUs compile OK**, archived → `libnumsci_full_iossim_arm64.a`.
- **Numeric-foundation link probe → `probe_full.dylib` LINKS with 0 undefined symbols.**
  Force-references `scipp::optimize::{brentq,newton,fsolve,minimize("BFGS"),least_squares}`
  and `numpp::linalg::{solve,lstsq,svd,qr,norm}` on real ndarrays.
- `otool -L probe_full.dylib` → links **only** `libc++.1.dylib` + `libSystem.B.dylib`.
  No BLAS / LAPACK / OpenMP / TBB leaked.
- **ZERO compiler warnings** across all 88 iOS compilations (`-Wall -Wextra`).
- Platform confirmed genuine (not accidental host arm64-darwin): `otool -l` / `lipo -info`
  report platform=7 (iOS-Simulator), minos 16.0, arch arm64.
- **Portable by design, not luck** — grep across all headers+src found **no**
  `<execution>`/`std::execution`/TBB, **no** x86 intrinsics (`__m128/256/512`, `_mm_*`,
  `immintrin`), **no** `#pragma omp`, **no** `<filesystem>`, **no** `dlopen`, **no**
  `std::thread`/`async`/`hardware_concurrency`. CPU-only is first-class: the `*_null.cpp`
  backend stubs satisfy the linalg vtable so `solve`/`svd`/`qr`/`lstsq` work with no
  external LAPACK.

**Caveats (none block the numeric core):**
1. `std::getenv` in `backend/backend.cpp` + `umath/ufunc.cpp` (GEMM/GPU dispatch gating) —
   POSIX, exists on iOS, returns null in sandbox → no-op; only gates GPU/BLAS which are OFF.
2. `std::ifstream/ofstream` in NumPP `io/npy.cpp` + `io/npz.cpp` — portable, compiled clean;
   a runtime sandbox-path concern for the **I/O module only**, separate from the numeric core.
3. **UNVERIFIED — not run on a simulator device.** Scope was compile+archive+link.
   On-`arm64` numeric *correctness* was not re-executed on-device (the repos carry their
   own test suites; those ran on host, §1).
4. **UNVERIFIED — CMake `find_package(NumPP CONFIG)` iOS packaging path bypassed.** The iOS
   probe compiled TUs directly with `-I` paths + hand-written `config.hpp`/`version.hpp`
   (backend flags = 0, matching shipped `config.hpp.in` defaults). A proper iOS build needs
   a CMake iOS toolchain file + NumPP install; nothing found suggests it would fail, but
   that plumbing is unexercised.

---

## 3. Geometry probe — do the solvers actually solve closest-point + SSI, incl. near-tangent?

Probe: `eval/geom_probe.cpp` + `eval/run_probe.sh`. Built host `clang++ -std=c++20`,
linked vs CMake-built `libnumpp.dylib` + directly-compiled SciPP optimize TUs.
Also `-fsyntax-only` verified for iOS-sim (probe, `bspline.cpp`, `least_squares.cpp` OK).

| Case | What it tests | Result | Verdict |
|---|---|---|---|
| **EXP1** closest point on bicubic B-spline (Extrema analogue) | BFGS min of distance | converged 6 iters/37 fevals → (u,v)=(0.615002,0.398662); dist 1.9303685 vs 400×400 brute 1.9303759 (**err 7.4e-6**); \|grad\|=3.9e-7. `least_squares` R²→R³ matched to 6 digits. | **USABLE AS-IS** |
| **EXP2a** plane ⟷ cylinder (analytic, true circle) | pinned angle, fsolve 3×3 | **8/8 converged**, max ‖S1−S2‖=0.0, max dist to true circle=0.0 (double-exact). | **ROBUST** |
| **EXP2b** two B-spline patches (freeform SSI, 3 eq / 4 unk, pinned u1) | fsolve from seeds | naive/warm seed **0/7** (Newton clamps to boundary); brute-force-nearby seed **3/7** — exactly where a true crossing exists — residuals 2.6e-14…1.1e-16. | **SEED-LIMITED — no built-in tracing/subdivision/marching** |
| **EXP2c** near-tangent (two spheres, tangency at d=3) | fsolve(Newton) + least_squares(LM) | both OK for d=1.5…2.9 (‖F‖≤~1e-13), then **BOTH FAIL at d≥2.99** (fsolve ‖F‖=2.6 @2.99; LM ‖F‖=5.6e-2). LM damping does **not** rescue near-tangent. | **FAILS — geometry-specific hardening required** |
| **EXP3** scalar root + linalg sanity | newton, solve 3×3, lstsq 4×2 | newton cos(u)=u → 0.739085133215161 (15-digit); solve ‖Ax−b‖=0; lstsq recovered slope 2.0 / intercept 1.0 exactly. | **ALL EXACT** |

**Functional probe** (`eval/numeric_probe.cpp`, host, vs libnumpp + numeric `.o`): **7/7 pass** —
brentq(x²−2)=√2; newton(cos x−x)=0.7390851; fsolve 2×2 nonlinear residual<1e-6; BFGS
Rosenbrock→(1,1) within 1e-3; least_squares line fit slope=2; closest-point-on-parabola
\|grad\|<1e-6; `linalg::solve` 3×3 SPD residual<1e-9. These are exactly the SSI /
curved-boolean primitives (n-D root / fsolve / BFGS / least_squares + closest-point + dense solve).

**Geometry-probe conclusion.** As a *point-solver / linalg substrate* the libraries are
excellent — closest-point, local SSI refinement, scalar roots, and dense solve/lstsq all
converge to 1e-14…1e-6 on native surfaces. As an *SSI kernel* they are not drop-in: from a
naive seed generic fsolve found **0/7** freeform intersection points, and **near-tangent
configs break BOTH** Newton (fsolve) and damped LM (least_squares). The
marching-line / seeding / tangent-degeneracy layer — precisely OCCT's `IntPatch`/`IntWalk`
geometry tuning — still has to be written on top.

---

## 4. Recommendation — GO WITH HARDENING

**Adopt NumPP + SciPP as the numeric substrate.** They are accurate, self-contained
(no external LAPACK), mobile-portable, and oracle-validated on the surface we consume.

Conditions / scope-down tasks (all understood, config-level):
1. **Build-scoping (small, config not source):** SciPP's full lib does not build under the
   mandated host clang/libc++ because of the `src/special/` special-math libc++ gap. Consume
   only the numeric modules (optimize/linalg/spatial/integrate) — which build clean — **or**
   build the full lib under libstdc++/GCC. `special`/`stats` are not needed by the kernel.
2. **Budget SSI as "generic solvers + bespoke curve-tracer":** treat fsolve/least_squares as
   the *local refinement* engine only. Marching, seeding/subdivision, and tangent/coincident
   degeneracy hardening are NOT provided — the EXP2b (0/7 from naive seed) and EXP2c
   (both solvers fail at near-tangent) results are concrete evidence.
3. **Perf caveat:** NumPP `ndarray` is heap-allocated NumPy-style. Benchmark before routing
   SSI hot loops through it — likely want dedicated `Vec3`/small-`Mat` types in inner loops.
4. **UNVERIFIED items to close before shipping on-device:** on-simulator numeric correctness
   (§2 caveat 3) and a real CMake iOS toolchain / NumPP-as-dep packaging path (§2 caveat 4).

**Not NO-GO** — no source-level or portability blocker was found. **Not unconditional GO** —
the near-tangent failures prove the hard SSI robustness is not bought by these libraries.

---

## 5. Revised effort

Prior estimate (NATIVE-REWRITE.md "Remaining work" table, production-robust `py`):

| # | Item | Prior estimate |
|---|---|---|
| **#2** | **Numeric foundations** — `math_` solvers (Newton/FunctionSetRoot/BFGS) + `Extrema` + `Adaptor3d` | **0.5–1 py** |
| **#5** | **SSI + general curved booleans** (`IntPatch`/`IntWalk` + BOPAlgo) | **3–6 py** clean-room (~1.5–3 py port). Prior NATIVE-REWRITE range cited **1.5–6 py**. |

**Revised if we adopt NumPP + SciPP:**

| # | Item | Revised | Saving | Basis (probe) |
|---|---|---|---|---|
| **#2 Numeric foundations** | inherit oracle-validated root-finding / fsolve / BFGS / L-BFGS-B / least_squares / curve_fit, dense linalg (solve/lstsq/lu/qr/svd/cholesky/eig), KD-tree / distances / convex-hull / Delaunay / rotations, quadrature / ODE — **plus a passing SciPy-oracle corpus** | **~0.15–0.35 py** (≈ **60–75% saved**) | §1 959/959 + 29/29 + §3 7/7 functional; §2 iOS link 0-undefined. Residual = build-scoping (§4.1) + thin geometry wrapper + OCCT-oracle validation + perf tuning (§4.3). |
| **#5 SSI — NUMERIC portion only** | local Newton/LM refinement of intersection points is provided and proven (EXP2a exact, EXP2b 3/3 residual ~1e-14 *when seeded near a true crossing*) | **~25–35% saved on the SSI *numeric* portion only** | EXP1/EXP2a/EXP3 pass; local-refinement covered. |

**Explicitly NOT saved by these libraries (the moat stays the moat):**
- **The hard SSI core** — marching-line algorithm, seeding/subdivision, and
  tangent/coincident-surface robustness. Direct evidence: EXP2b **0/7** from a naive seed,
  EXP2c **both** Newton and damped LM **fail at near-tangent (d≥2.99)**. The
  robustness-critical **~65–75%** of SSI numerics must still be written. Full curved
  booleans (BOPAlgo classification/trimming) are entirely on top of that.
- **Shape healing** (#4, 2–4 py) — no overlap; nothing provided.
- **STEP/IGES import** (#3, 2–4 py) — no overlap; nothing provided.
- **Blends** (#6) and **wrap-emboss** (#7) — gated on #5's curved slice; unaffected.

**Net picture:** the generic numeric-library layer (#2) moves **off the critical path** —
from a 0.5–1 py build to ~0.15–0.35 py of integration + wrapper + validation. SSI (#5)
stays the dominant remaining cost: only its local-refinement slice is helped; the
marching/tangent/degeneracy robustness that is *the point of an SSI kernel* is unchanged.

---

## Artifacts (absolute paths)

- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/numeric_probe.cpp` — host functional probe (7/7).
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/geom_probe.cpp` + `run_probe.sh` — geometry probe (EXP1–EXP3).
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/build_ios_sim.sh` — 21-TU iOS-sim subset probe.
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/build_full_ios_sim.sh` — full lib + probe link (reproduce iOS result).
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/link_probe.cpp` — iOS numeric-foundation link test.
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/gen/` — hand-written config/version headers (backends 0).
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/libnumsci_iossim_arm64.a`, `libnumsci_full_iossim_arm64.a`, `probe_full.dylib` — iOS-sim archives + linked dylib.
- `/Users/leonardoaraujo/work/CyberCadKernel/.claude/worktrees/finish-phase-0-1/eval/REPRO.md` — host reproduction notes.

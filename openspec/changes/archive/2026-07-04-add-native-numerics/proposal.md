# Proposal — add-native-numerics

## Why

Phase 4 replaces OCCT capability by capability behind the unchanged `cc_*` facade
(`openspec/NATIVE-REWRITE.md`). Capability **#2 `numeric-foundations`** is flagged there as
the **highest-leverage** item and the **on-ramp to everything below** (SSI → curved
booleans → blends): it is the generic solver layer (`math_*` Newton / `FunctionSetRoot` /
BFGS), the `Extrema` closest-point / projection layer, and `Adaptor3d`. Every downstream
hard capability sits on top of it.

Rather than hand-write those solvers, the evaluation `docs/EVAL-numpp-scipp.md` probed
**NumPP + SciPP** — the org's C++20, MIT NumPy / SciPy ports — against this kernel's exact
toolchain and geometry needs and returned **GO WITH HARDENING**:

- **NumPP** builds + tests clean (959/959 cases, 2607 checks, 0 failures), CPU-only, zero
  external deps, its own reference `solve`/`lstsq`/`inv` (no BLAS/LAPACK).
- **SciPP's kernel-relevant subset** (`optimize` / `linalg` / `spatial` / `integrate`, 18
  TUs) builds clean under host libc++ AND on `arm64-apple-ios16.0-simulator` with zero
  warnings; its SciPy-oracle numeric tests pass (29/29 cases, 841 checks, 0 failures); the
  numeric-foundation force-link on iOS-sim links with **0 undefined symbols** (only
  `libc++` + `libSystem`).
- **Closest-point is proven USABLE AS-IS** — the geometry probe's EXP1 (BFGS closest point
  on a bicubic B-spline) converged to err **7.4e-6** vs a 400×400 brute-force reference;
  EXP3 scalar root / `solve` / `lstsq` were fp-exact.

So adopting them retires **~60–75% of #2** (the generic-solver line) — moving numeric
foundations **off the critical path** — while the SSI moat is explicitly untouched. This
change ADOPTS NumPP/SciPP as the substrate and builds the **first `Extrema` slice
(closest-point / projection)** on top, which is the honest, exactly-verifiable on-ramp to
SSI.

**The hardening the eval requires is respected, not ignored:** (1) SciPP's `src/special` +
`src/stats` FAIL under Homebrew libc++ (a libc++ ISO-29124 stdlib gap — `std::legendre`,
`std::cyl_bessel_j`, … — grep-confirmed confined to `src/special/`, not touched by any
numeric module the kernel uses), so this change **SCOPES the SciPP build to EXCLUDE
`special` + `stats`**; (2) the near-tangent SSI failures the eval flagged (EXP2b naive-seed
0/7, EXP2c both solvers fail at near-tangent d≥2.99) are a property of the **SSI geometry
layer that is the NEXT capability (#5)** — NOT closest-point, which is a well-conditioned
single-target minimization the generic solvers handle cleanly. This proposal scopes the
**substrate + closest-point on-ramp only**; SSI and curved booleans stay OCCT, labelled
honestly.

## What changes

1. **Adopt NumPP + SciPP as the OCCT-free numeric substrate (build plumbing).** Reference
   both libraries **by absolute path, exactly like OCCT** (`/Users/leonardoaraujo/work/NumPP`,
   `/Users/leonardoaraujo/work/SciPP`) — **NOT vendored** into this repo — and make the
   NumPP core + the SciPP `optimize` / `linalg` (+ `spatial` / `integrate`) subset
   compile + link into the kernel for **HOST** (`clang++ -std=c++20`, libc++) and
   **arm64-iOS-simulator**. A new CMake option `CYBERCAD_HAS_NUMSCI` (with
   `CYBERCAD_NUMPP_PREFIX` / `CYBERCAD_SCIPP_PREFIX` absolute-path hints, mirroring
   `CYBERCAD_OCCT_INCLUDE_DIR`) gates the whole numerics module. The SciPP build **EXCLUDES
   `special` + `stats`** (the libc++ gap) and enables no NumPP GPU/BLAS backend (CPU-only,
   as probed). With `CYBERCAD_HAS_NUMSCI=OFF` (default), no NumPP/SciPP header or source is
   compiled and every existing native suite is unaffected.
2. **A thin OCCT-free numerics facade** (`src/native/numerics/`, guarded by
   `CYBERCAD_HAS_NUMSCI`). New headers exposing the kernel-facing numeric API over
   NumPP/SciPP:
   - **scalar root** — `newton(f, x0[, fprime])`, `brentq(f, a, b)` (wrap
     `scipp::optimize::{newton,brentq}`);
   - **nonlinear system solve** — `fsolve(F, x0)` (wrap `scipp::optimize::fsolve`);
   - **minimize** — `minimize(f, x0)` BFGS (wrap `scipp::optimize::minimize("BFGS")`) and
     **least_squares** — `least_squares(residual, x0)` LM (wrap
     `scipp::optimize::least_squares`);
   - **dense linear algebra** — `solve(A, b)` and `lstsq(A, b)` (wrap
     `scipp::linalg::{solve,lstsq}` / NumPP).
   Small native `Vec` / `Mat` adapters marshal between the kernel's `double` /
   `math::Point3` types and NumPP `ndarray`, so callers never touch `ndarray` directly. The
   facade includes only STL + NumPP + SciPP + `src/native/math`; it references no OCCT /
   `IEngine` / `EngineShape` type.
3. **Native closest-point / projection (the Extrema on-ramp).** In the same subtree, an
   OCCT-free projector:
   - **nearest `t` on a native curve** to a 3D point `P` — minimize `‖C(t) − P‖²` over the
     curve's parameter range, using the existing `src/native/math` curve evaluators
     (`Line` / `Circle` / `BSpline`), seeded by a coarse parameter sample (**multi-start**)
     and refined by SciPP `minimize` / `least_squares`; clamps / snaps to the parameter
     bounds; returns the foot parameter, the foot point, and the distance;
   - **nearest `(u, v)` on a native surface** to `P` — minimize `‖S(u,v) − P‖²` over the
     surface domain, using the `src/native/math` surface evaluators (`Plane` / `Cylinder` /
     `Cone` / `Sphere` / `BSpline`), with the same coarse-grid multi-start seeding + SciPP
     refinement;
   - **multi-start seeding for robustness** — a coarse parameter grid is sampled, the best
     few seeds are refined, and the global best foot-point is returned, so a non-convex
     distance field (e.g. a point near a curved surface's medial axis) does not trap the
     solver in a spurious local minimum.
   The formulation matches OCCT `Extrema` (`Extrema_ExtPC` for a curve, `Extrema_ExtPS` for
   a surface, via `Adaptor3d`), which is the sim parity oracle.

## Non-goals (DEFERRED — the moat stays; NOT implemented, NOT faked)

- **SSI (surface-surface intersection) + curved booleans** — the marching-line algorithm,
  seeding / subdivision, and tangent / coincident-surface robustness (OCCT `IntPatch` /
  `IntWalk` + BOPAlgo). This is capability **#5**, the NEXT one, and stays OCCT. The eval's
  EXP2b (naive-seed 0/7) and EXP2c (both Newton and damped LM fail at near-tangent d≥2.99)
  are the concrete evidence that this hardening is still ours to write; it is NOT attempted
  here.
- **Curve-curve / curve-surface intersection** — likewise SSI-family, out of scope.
- **SciPP `special` + `stats`** — excluded from the build (the Homebrew-libc++ ISO-29124
  gap; not needed by the kernel).
- **Constrained optimization** (`linprog` / `nnls`), and `spatial` / `integrate` wrappers
  beyond what closest-point needs — the subset builds/links but no kernel facade is added
  for them here.
- **NumPP GPU / BLAS / LAPACK / CUDA / OpenCL / Metal backends** — CPU-only, as probed.
- **Engine-wiring to a `cc_*` closest-point entry** — this change ships the OCCT-free
  substrate + projector library; a `cc_*` facade call and `NativeEngine` wiring are a
  follow-up (as with the other #1–#3 foundation slices that shipped un-wired by design).
- **Vendoring NumPP/SciPP into this repo** — they are referenced by absolute path like
  OCCT, not copied in.
- **Unblocking #8 `drop-occt`** — this change does NOT remove the OCCT dependency; SSI +
  curved booleans + shape healing + STEP/IGES import still block it. Reported honestly.

## Impact

- **New external numeric dependencies** — NumPP (`/Users/leonardoaraujo/work/NumPP`) +
  SciPP (`/Users/leonardoaraujo/work/SciPP`), C++20 MIT NumPy/SciPy ports, referenced by
  absolute path (NOT vendored), CPU-only, SciPP `optimize`/`linalg`/`spatial`/`integrate`
  subset (`special`/`stats` excluded). Recorded in `openspec/project.md` alongside OCCT.
- **New OCCT-free, guarded subtree** `src/native/numerics/` (the facade + the closest-point
  projector) under a `native_numerics.h` umbrella, compiled only when
  `CYBERCAD_HAS_NUMSCI=ON`.
- **`CMakeLists.txt`** — a `CYBERCAD_HAS_NUMSCI` option + `CYBERCAD_NUMPP_PREFIX` /
  `CYBERCAD_SCIPP_PREFIX` absolute-path hints; when ON, add the NumPP/SciPP includes +
  the SciPP `optimize`/`linalg`(+`spatial`/`integrate`) objects (EXCLUDING `special` +
  `stats`) and compile the `src/native/numerics/*` sources; when OFF, none of the above is
  built (the default, so existing suites are unchanged).
- **New host CTest** `test_native_numerics` (analytic solver + closed-form closest-point
  assertions, NumPP/SciPP linked, no OCCT), added to the test list under
  `CYBERCAD_HAS_NUMSCI`.
- **New sim parity test** `tests/sim/native_numerics_parity.mm` +
  `scripts/run-sim-native-numerics.sh` (native-vs-OCCT `Extrema` parity), own `main()`, on
  the `run-sim-suite.sh` SKIP list so the 221-assertion suite count is unchanged.
- **No** `include/cybercadkernel/cc_kernel.h` change; **no** `src/facade/cc_kernel.cpp`
  change; **no** default-engine change. With `CYBERCAD_HAS_NUMSCI=OFF` behaviour is
  identical to today.
- **Docs** — `openspec/project.md` (external numeric deps) + `openspec/NATIVE-REWRITE.md`
  #2 (adoption, linking the eval) updated.

## Verification

Two independent gates from `NATIVE-REWRITE.md`: (a) **host** analytic unit tests
(`clang++ -std=c++20`, no OCCT, NumPP/SciPP linked) — the facade solves known-value
problems (`brentq(x²−2)=√2`, `newton(cos x − x)`, `fsolve` 2×2, BFGS Rosenbrock → (1,1),
`least_squares` line fit, `solve`/`lstsq`) and closest-point on analytic geometry with a
closed-form answer (nearest `t` on a line / circle; nearest `(u,v)` on a plane / cylinder /
sphere; nearest `(u,v)` on a bicubic B-spline vs a dense brute-force grid) within
tolerance; (b) **sim native-vs-OCCT `Extrema` parity** — on the iOS simulator (OCCT
linked), native closest-point / projection over native curves / surfaces is compared
against the OCCT `Extrema` oracle (`Extrema_ExtPC` / `Extrema_ExtPS` via `Adaptor3d`) at
sampled 3D targets, foot parameter + distance within a tight fp64 tolerance. Done only when
both gates pass and every existing suite stays green with `CYBERCAD_HAS_NUMSCI=OFF`.
Reported honestly as the **numeric substrate + Extrema on-ramp** — NOT SSI, NOT curved
booleans; those stay OCCT and remain the moat blocking #8 `drop-occt`.

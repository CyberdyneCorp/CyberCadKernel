# Design — add-native-numerics

Substrate + Extrema on-ramp slice of Phase 4 #2 (`numeric-foundations`): **ADOPT NumPP +
SciPP** as the OCCT-free numeric substrate (root-find / fsolve / minimize / least_squares /
dense solve+lstsq) and build native **closest-point / projection** (nearest `t` on a curve,
nearest `(u,v)` on a surface) on top — verified by host analytic tests + native-vs-OCCT
`Extrema` parity on the sim. **NOT SSI, NOT curved booleans.** The numeric algorithms are
ADOPTED from NumPP/SciPP (mature, SciPy-oracle-validated, MIT); OCCT `Extrema` /
`Adaptor3d` is the parity ORACLE only. Grounded in the eval `docs/EVAL-numpp-scipp.md`
(verdict **GO WITH HARDENING**).

## 1. Why adopt (and why closest-point is the right first slice)

`docs/EVAL-numpp-scipp.md` probed both libraries against this kernel's toolchain
(Homebrew `clang++ -std=c++20`, libc++; iOS-sim `arm64-apple-ios16.0-simulator`) and
geometry needs. The load-bearing facts this design respects:

- **NumPP is mature + self-contained** — builds clean CPU-only, **959/959 cases / 2607
  checks / 0 failures**, its own reference `solve`/`lstsq`/`inv` with no external
  BLAS/LAPACK (eval §1).
- **SciPP's numeric surface is oracle-validated + iOS-clean** — `optimize` / `linalg` /
  `spatial` / `integrate` (18 TUs) build clean under host libc++, pass SciPP's own SciPy
  oracle (**29/29 cases / 841 checks / 0 failures**), and the numeric-foundation force-link
  on iOS-sim links with **0 undefined symbols** (only `libc++` + `libSystem`; no
  BLAS/LAPACK/OpenMP/TBB leaked) (eval §1–§2).
- **Closest-point is USABLE AS-IS** — EXP1: BFGS closest point on a bicubic B-spline
  converged in 6 iters / 37 fevals to err **7.4e-6** vs a 400×400 brute-force reference,
  `‖grad‖=3.9e-7`; `least_squares` R²→R³ matched to 6 digits. EXP3: `newton(cos u − u)` →
  15-digit, `solve` residual 0, `lstsq` exact (eval §3). The functional probe passed
  **7/7** including closest-point-on-parabola and BFGS Rosenbrock.

So the substrate is ADOPTED, and the first geometry slice is closest-point / projection
because it is a **well-conditioned single-target minimization** the generic solvers handle
cleanly — unlike SSI.

### The two hardening conditions the eval imposes (respected here)

1. **Build-scoping (config, not source).** SciPP's full lib does NOT build under Homebrew
   libc++: `src/special/{orthogonal,bessel,expint,spherical_bessel}.cpp` reference
   `std::legendre` / `std::laguerre` / `std::cyl_bessel_j` / `std::cyl_neumann` /
   `std::cyl_bessel_i` / `std::cyl_bessel_k` / `std::expint`, which libc++ (unlike
   libstdc++) does not implement — an **ISO-29124 stdlib gap, VERIFIED not a SciPP bug**
   (the same TUs compile under GCC-16/libstdc++). The 10 offending `std::` calls are
   grep-confirmed **confined to `src/special/`**; no numeric module the kernel consumes
   touches them. → **This design SCOPES the SciPP build to EXCLUDE `special` + `stats`**
   and consumes only `optimize` / `linalg` (+ `spatial` / `integrate`). (eval §1, §4.1)
2. **SSI is NOT bought by these libraries — it is the NEXT capability.** EXP2b (two
   freeform B-spline patches): naive/warm seed fsolve found **0/7** intersection points
   (Newton clamps to the boundary); only a brute-force-nearby seed found 3/3. EXP2c
   (near-tangent spheres): BOTH fsolve (Newton) and damped LM (`least_squares`) FAIL at
   d≥2.99. The marching / seeding-subdivision / tangent-degeneracy layer — OCCT's
   `IntPatch`/`IntWalk` — must still be written. → **SSI is explicitly OUT OF SCOPE here**;
   this change is the substrate + closest-point on-ramp to it. (eval §3, §4.2, §5)

## 2. Adoption model — referenced by absolute path, like OCCT (NOT vendored)

NumPP and SciPP are treated exactly as OCCT is in this repo: an **external dependency
referenced by absolute path**, never copied into the source tree.

- `CYBERCAD_NUMPP_PREFIX` defaults to `/Users/leonardoaraujo/work/NumPP`, and
  `CYBERCAD_SCIPP_PREFIX` to `/Users/leonardoaraujo/work/SciPP` — the mirror of
  `CYBERCAD_OCCT_INCLUDE_DIR`. Both are `CACHE PATH` overridable.
- Includes come from `${prefix}/include` (`numpp/…`, `scipp/…`, both header trees exist).
- SciPP object code — the `optimize` / `linalg` (+ `spatial` / `integrate`) TUs — is built
  from `${CYBERCAD_SCIPP_PREFIX}/src`, **EXCLUDING** `special/*` and `stats/*` (the libc++
  gap). NumPP is header-first with a CPU-only backend; its `backend/*_null.cpp` stubs
  satisfy the linalg vtable so `solve`/`lstsq`/`svd`/`qr` work with no external LAPACK (eval
  §2). No NumPP `NUMPP_WITH_*` backend is enabled (CPU-only, as probed).
- The whole thing is gated by a new **`CYBERCAD_HAS_NUMSCI`** option. `OFF` (default) ⇒
  none of NumPP/SciPP/`src/native/numerics` is compiled; the existing kernel + every
  existing suite build and pass unchanged. `ON` ⇒ the numerics module + its NumPP/SciPP
  deps are compiled for HOST or iOS-sim.

> The eval's UNVERIFIED caveats (a real CMake `find_package(NumPP CONFIG)` iOS packaging
> path, on-simulator numeric correctness re-run) are called out as tasks (§10, tasks.md):
> the iOS probe compiled TUs directly with `-I` + hand-written `config.hpp` (backends 0),
> so the production iOS plumbing needs a CMake toolchain file + a NumPP install/consume
> path; the sim parity gate closes the on-device correctness caveat for closest-point.

## 3. The numerics facade (`src/native/numerics/`) — kernel-facing API over NumPP/SciPP

A thin OCCT-free facade so kernel code never touches `ndarray` directly. Suggested files:
`marshal.h` (native ↔ `ndarray` adapters), `solve.h` (the numeric entry points),
`project.h` (closest-point), umbrella `native_numerics.h`.

| Kernel-facing facade | Wraps | Notes |
|---|---|---|
| `numerics::newton(f, x0, fprime?)` | `scipp::optimize::newton` | scalar root; optional analytic derivative |
| `numerics::brentq(f, a, b)` | `scipp::optimize::brentq` | bracketed scalar root |
| `numerics::fsolve(F, x0)` → `Vec` | `scipp::optimize::fsolve` | nonlinear system (Powell hybrid) |
| `numerics::minimize(f, x0)` → `MinResult` | `scipp::optimize::minimize(..., "BFGS")` | unconstrained minimize (BFGS) |
| `numerics::least_squares(residual, x0)` → `LsqResult` | `scipp::optimize::least_squares` | Levenberg-Marquardt |
| `numerics::solve(A, b)` → `Vec` | `scipp::linalg::solve` | dense linear solve |
| `numerics::lstsq(A, b)` → `LstsqResult` | `scipp::linalg::lstsq` | dense least-squares |

- **`marshal.h`** — `toNdarray(const Vec&)` / `fromNdarray(const ndarray&)` and small
  `Mat` ⇄ `ndarray` (row-major) helpers. Callers pass `std::vector<double>` / `Vec3` and
  get back native types; the `ndarray` boundary is confined to the facade. `f` / `F` /
  `residual` are `std::function` taking/returning native `Vec`, wrapped into the SciPP
  `ScalarFn` / `VecFn` / `ObjFn` signatures inside the facade.
- The facade surfaces convergence status (converged flag, iterations, final residual /
  `‖grad‖`) from the SciPP `OptimizeResult` / `LeastSquaresResult` so callers (and tests)
  can assert convergence, matching the probe's reported diagnostics.
- Includes only STL + `numpp/…` + `scipp/optimize` + `scipp/linalg`; no OCCT.

## 4. Native closest-point / projection (the Extrema on-ramp)

The geometry payload. For a 3D target `P`:

- **Curve — nearest `t`.** Minimize `d(t) = ‖C(t) − P‖²` over the curve's `[t0, t1]`.
  `C(t)` is evaluated through the existing `src/native/math` curve evaluators
  (`elementary.h` `Line`/`Circle`; `bspline.h` for B-spline curves). Return the foot
  parameter `t*`, the foot point `C(t*)`, and `‖C(t*) − P‖`.
- **Surface — nearest `(u, v)`.** Minimize `d(u,v) = ‖S(u,v) − P‖²` over the surface domain.
  `S(u,v)` from the `src/native/math` surface evaluators (`elementary.h`
  `Plane`/`Cylinder`/`Cone`/`Sphere`; `bspline.h` surface; `torus.h`). Return `(u*, v*)`,
  the foot point, and the distance.

### 4.1 Multi-start seeding (robustness)

The squared-distance field is non-convex for curved geometry (a point near a cylinder's
axis, near a sphere's centre, near a B-spline's medial axis has multiple local minima), so
a single seed can converge to a spurious foot. The projector therefore:

1. samples a **coarse parameter grid** over the range/domain (a handful of `t` values for a
   curve; a small `u×v` grid for a surface) and evaluates `d` at each — this is exactly the
   probe's brute-force-nearby seeding that made EXP1 land at err 7.4e-6 and (for SSI, out of
   scope) turned EXP2b's 0/7 into 3/3;
2. refines the **best few seeds** with SciPP `minimize` (BFGS) — or `least_squares` (LM) on
   the vector residual `S(u,v) − P` / `C(t) − P`, which is the natural least-squares form —
   each clamped to the parameter bounds;
3. returns the **global-best** refined foot (smallest distance), with its convergence
   diagnostics.

The number of seeds is a tunable (coarse grid resolution) chosen so the host closed-form
tests and the sim `Extrema` parity both pass; it is bounded (no unbounded search) so the
op is deterministic and fast for the analytic + single-patch B-spline cases in scope.

### 4.2 Why this matches OCCT `Extrema` (the oracle)

OCCT `Extrema_ExtPC` (point→curve) and `Extrema_ExtPS` (point→surface), reached via
`Adaptor3d` adaptors, compute exactly these extremal-distance feet. For the analytic
primitives the foot is closed-form (the parity is fp-exact); for a B-spline it is the same
distance-minimization OCCT solves numerically, so the native result matches within a tight
tolerance. The sim gate asserts foot parameter + foot distance agree.

## 5. Guarding — the rest of `src/native` builds without NumPP/SciPP

`src/native/{math,topology,tessellate,construct,boolean,blend,exchange}` MUST NOT gain a
NumPP/SciPP dependency. Only `src/native/numerics/*` may include them, and those sources
are compiled ONLY under `CYBERCAD_HAS_NUMSCI=ON`. The top-level `file(GLOB_RECURSE
CYBERCAD_SOURCES … src/*.cpp)` gets a `list(FILTER … EXCLUDE REGEX "/src/native/numerics/")`
so the numerics `.cpp` files are added only inside the `if(CYBERCAD_HAS_NUMSCI)` block
(mirroring how `src/engine/occt/*` is excluded then added under `CYBERCAD_HAS_OCCT`). With
the option OFF, the kernel is byte-for-byte the current build and all existing suites stay
green.

## 6. Architecture / dependency boundaries

```
                         cc_* facade  (UNCHANGED — no ABI change, default engine OCCT)
                                  │
   src/native/numerics/  (OCCT-FREE, guarded by CYBERCAD_HAS_NUMSCI)
        ├─ solve.h        root / fsolve / minimize / least_squares / solve / lstsq
        │        └── SciPP optimize + linalg  ── NumPP ndarray   (absolute-path dep, NOT vendored)
        └─ project.h      closest-point t / (u,v)  (multi-start + SciPP refine)
                 └── src/native/math  (Line/Circle/BSpline curves; Plane/Cyl/Cone/Sphere/BSpline/Torus surfaces)

   OCCT Extrema (Extrema_ExtPC / Extrema_ExtPS via Adaptor3d)  ── PARITY ORACLE only (sim gate)
```

- `src/native/numerics/` is OCCT-FREE (NumPP/SciPP are NOT OCCT) and references no OCCT /
  `IEngine` / `EngineShape` type.
- No `cc_*` ABI change; default engine stays OCCT. This slice ships the substrate +
  projector library; it is NOT yet wired to a `cc_*` entry (a follow-up, like the other
  un-wired #1–#3 foundation slices).

## 7. Cognitive complexity

The facade entry points are flat wrappers (each ≤ ~6 — build the SciPP callable, call,
unpack the result). The projector's multi-start driver (sample grid → pick best seeds →
refine → choose global best) is the one loop-nested function; kept in the systems band
(≤ ~20) by factoring the per-seed refine into its own helper and the grid sample into a
generator. All flagged and measured with the `cognitive-complexity` skill before archive.

## 8. Verification plan

- **Gate 1 (host, no OCCT, NumPP/SciPP linked)** — `tests/native/test_native_numerics.cpp`:
  the facade solves analytic known-value problems — `brentq(x²−2)=√2`, `newton(cos x − x) =
  0.739085…`, `fsolve` on a 2×2 nonlinear residual (< 1e-6), BFGS on Rosenbrock → (1,1)
  within 1e-3, `least_squares` line fit (slope 2 / intercept 1), `solve` on a 3×3 SPD
  system (residual < 1e-9), `lstsq` on a 4×2 system — mirroring the eval's 7/7 functional
  probe; plus **closest-point with a closed-form answer**: nearest `t` on a line / circle,
  nearest `(u,v)` on a plane / cylinder / sphere, nearest `(u,v)` on a bicubic B-spline vs a
  dense brute-force grid (err ≲ 1e-5, matching EXP1). No OCCT.
- **Gate 2 (sim native-vs-OCCT `Extrema` parity)** — `tests/sim/native_numerics_parity.mm`
  + `scripts/run-sim-native-numerics.sh`: on a booted iOS simulator (OCCT linked), native
  closest-point over native curves (line / circle / B-spline) and surfaces (plane /
  cylinder / cone / sphere / B-spline) is compared against OCCT `Extrema_ExtPC` /
  `Extrema_ExtPS` (via `Adaptor3d`) at sampled 3D targets — foot parameter and foot-point
  distance within a tight fp64 tolerance. Own `main()`, on the `run-sim-suite.sh` SKIP list
  so the 221-assertion suite count is unchanged.
- **Done** only when both gates are green AND every existing suite (`run-sim-suite.sh`
  221/221, host CTest, GPU/Phase-3) stays green with `CYBERCAD_HAS_NUMSCI=OFF`. Honestly
  reported as the numeric substrate + Extrema on-ramp — NOT SSI, NOT curved booleans; those
  stay OCCT and remain the moat blocking #8 `drop-occt`.

## 9. Explicit non-goals (the moat stays the moat)

- **SSI + curved booleans** (`IntPatch` / `IntWalk` + BOPAlgo) — the NEXT capability (#5).
  The eval's EXP2b (0/7 naive seed) and EXP2c (both solvers fail at near-tangent) are its
  evidence. NOT here.
- **Curve-curve / curve-surface intersection** — SSI-family, out of scope.
- **SciPP `special` / `stats`** — excluded (libc++ gap).
- **Constrained optimization / KD-tree / Delaunay / quadrature / ODE facades** beyond what
  closest-point needs; **NumPP GPU/BLAS backends**; **vendoring**; **`cc_*` wiring**. All
  deferred / out of scope, stated in the proposal Non-goals.

## 10. UNVERIFIED-caveat closeout (from the eval)

- **On-simulator numeric correctness** (eval §2 caveat 3) — closed by the Gate 2 parity run
  for the closest-point surface; the broader NumPP/SciPP suite runs on host (§1).
- **CMake `find_package(NumPP CONFIG)` iOS packaging** (eval §2 caveat 4) — a task: provide
  the CMake iOS-toolchain + NumPP install/consume path the probe bypassed (it used raw `-I`
  + hand-written `config.hpp`). Until then the iOS numerics build follows the probe's
  direct-`-I` recipe, documented in tasks.md.
- **Perf** (eval §4.3) — NumPP `ndarray` is heap-allocated; the closest-point inner loops
  evaluate through native `src/native/math` `Vec3`/small-`Mat`, routing `ndarray` only at
  the SciPP call boundary. Benchmark before any SSI hot loop (out of scope here) is routed
  through `ndarray`.

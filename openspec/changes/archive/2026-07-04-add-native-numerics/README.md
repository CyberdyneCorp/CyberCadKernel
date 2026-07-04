# add-native-numerics

The **SUBSTRATE + Extrema on-ramp** slice of Phase 4 capability **#2 `numeric-foundations`**
(`openspec/NATIVE-REWRITE.md`). This change lands the OCCT-free numeric layer the rest of
the native rewrite needs, in two honest, exactly-verifiable pieces:

1. **ADOPT NumPP + SciPP** (the org's C++20, MIT NumPy / SciPy ports) as the kernel's
   **OCCT-free numeric substrate** — root-finding, nonlinear system solve, unconstrained
   minimization, least-squares, and dense linear algebra — replacing the "write our own
   `math_` solvers" line of #2. They are referenced **by absolute path like OCCT is** (NOT
   vendored into this repo) and built for **HOST** and **arm64-iOS-simulator**.
2. **Native CLOSEST-POINT / projection (the Extrema on-ramp):** a thin OCCT-free
   `src/native/numerics/` facade over SciPP that computes the nearest parameter `t` on a
   native curve, and the nearest `(u, v)` on a native surface, to a 3D point — matching
   OCCT `Extrema` within tolerance.

This is the **numeric substrate and the closest-point/projection on-ramp to SSI — it is
NOT SSI and NOT curved booleans.** The hard surface-surface-intersection core (marching,
seeding/subdivision, near-tangent / coincident-surface robustness) is the NEXT capability
and is explicitly OUT OF SCOPE here (see the eval below). It does NOT change the `cc_*`
ABI, does NOT change the default engine (stays OCCT), and does NOT let `src/native/*`
outside `numerics/` depend on NumPP/SciPP (the numerics module is guarded so every existing
native suite still builds without them).

## Why adopt NumPP + SciPP (the eval says GO WITH HARDENING)

The evaluation `docs/EVAL-numpp-scipp.md` (verdict **GO WITH HARDENING**) probed both
libraries against this kernel's exact toolchain and geometry needs:

- **NumPP builds + tests clean** — CPU-only, zero external deps, **959/959 cases /
  2607 checks / 0 failures**; its own reference `solve` / `lstsq` / `inv` need no
  BLAS/LAPACK.
- **SciPP's kernel-relevant subset builds clean on host libc++ and iOS-sim** — the
  18 TUs of `optimize` / `linalg` / `spatial` / `integrate` compile with zero warnings and
  its SciPy-oracle tests pass (**29/29 cases / 841 checks / 0 failures**); the full
  numeric-foundation force-link on `arm64-apple-ios16.0-simulator` links with **0
  undefined symbols**, pulling in only `libc++` + `libSystem`.
- **The closest-point use case is proven USABLE AS-IS** — the geometry probe's EXP1
  (BFGS closest point on a bicubic B-spline) converged to **err 7.4e-6** vs a 400×400
  brute-force reference, and EXP3 scalar root / dense solve / lstsq were all fp-exact.

The **hardening condition we respect here** is a build-scoping one, not a source defect:
SciPP's `src/special` + `src/stats` FAIL under Homebrew libc++ because libc++ lacks the
C++17 ISO-29124 special-math functions (`std::legendre`, `std::cyl_bessel_j`, …). The 10
offending `std::` calls are grep-confirmed **entirely confined to `src/special/`** and no
numeric module the kernel consumes touches them. So this change **SCOPES the SciPP build to
EXCLUDE `special` + `stats`** and consumes only `optimize` / `linalg` / `spatial` /
`integrate`. See `docs/EVAL-numpp-scipp.md` §1, §2, §4.

The **near-tangent SSI failures** the eval flagged (EXP2b naive-seed 0/7, EXP2c both
solvers fail at near-tangent) are a property of the **SSI geometry layer we still have to
write** — they are the NEXT capability, NOT this one. Closest-point / projection is a
well-conditioned single-target minimization the generic solvers handle cleanly (EXP1), so
it is squarely in scope. This split is stated truthfully throughout.

## What #2 numeric-foundations is (the on-ramp this change delivers)

Per `openspec/NATIVE-REWRITE.md`, #2 `numeric-foundations` is the **highest-leverage** item
and the **on-ramp to everything below** (SSI → curved booleans → blends). Its OCCT oracles
are `math_*` (Newton / `FunctionSetRoot` / BFGS), `Extrema` (closest point / projection),
and `Adaptor3d`. Adopting NumPP/SciPP retires ~60–75% of the generic-solver portion (the
`math_*` solvers) and this change wraps them behind a kernel-facing facade plus the first
`Extrema` slice (closest-point). It does **NOT** deliver the SSI portion (#5), which is
where the moat stays.

## Scope (SUBSTRATE + Extrema closest-point ONLY)

| Item | In this change | Out of scope (NOT faked, NOT here) |
|---|---|---|
| **Build plumbing** — NumPP + the SciPP `optimize` / `linalg` (+ `spatial` / `integrate`) subset compile + link into the kernel for **HOST** and **arm64-iOS-sim**, referenced by absolute path (like OCCT), NOT vendored; SciPP `special` + `stats` EXCLUDED per the libc++ gap; guarded so the rest of `src/native` builds without them | YES | Vendoring the sources into the repo; enabling BLAS / LAPACK / CUDA / OpenCL / Metal NumPP backends; SciPP `special` / `stats` |
| **`src/native/numerics/` facade** — scalar root (`newton` / `brentq`), nonlinear system solve (`fsolve`), unconstrained minimize (`BFGS`) / `least_squares`, dense `solve` / `lstsq`, over native `Vec`/`Mat` adapters onto NumPP `ndarray` | YES | Constrained optimization (`linprog` / `nnls`), ODE / quadrature wrappers beyond what closest-point needs, KD-tree / Delaunay wrappers |
| **Native closest-point / projection** — nearest `t` on a native curve and nearest `(u, v)` on a native surface to a 3D point, built on SciPP `minimize` / `least_squares` with **multi-start seeding** for robustness; matches OCCT `Extrema` within tolerance | YES | Curve-curve / curve-surface / surface-surface **intersection** (SSI); the marching / seeding-subdivision / tangent-degeneracy layer |
| **SSI + curved booleans** | — | OUT — the NEXT capability (#5). The eval's near-tangent failures (EXP2b / EXP2c) are its evidence; not attempted here |

## Method (locked, per NATIVE-REWRITE.md)

**Clean-room from math / first principles + the eval.** The numeric algorithms
(Newton / Brent / Powell-hybrid `fsolve` / BFGS / Levenberg-Marquardt `least_squares` /
dense `solve`/`lstsq`) are **provided by NumPP/SciPP** — mature, SciPy-oracle-validated MIT
libraries authored by this org — so the substrate is ADOPTED, not re-derived. The
closest-point / projection layer is the standard formulation: minimize the squared distance
`‖S(u,v) − P‖²` (surface) / `‖C(t) − P‖²` (curve), seeded by a coarse parameter-grid sample
(multi-start) and refined by SciPP `minimize` / `least_squares`, evaluated through the
existing native `src/native/math` curve / surface evaluators. **OCCT `Extrema`
(`Extrema_ExtPC` / `Extrema_ExtPS`, `Adaptor3d`) is the reference ORACLE only** — the sim
parity gate compares native results against it — never copied.

## Architecture / OCCT boundary + the NumPP/SciPP boundary

- The numerics facade lands under a new **`src/native/numerics/`** subtree. It stays
  **OCCT-FREE** (NumPP/SciPP are NOT OCCT) and depends on NumPP + SciPP `optimize` /
  `linalg` (+ `spatial` / `integrate`) and on `src/native/math` (to evaluate curves /
  surfaces). It references no OCCT / `IEngine` / `EngineShape` type.
- **Guarded module.** The `numerics/` sources compile only under a new
  `CYBERCAD_HAS_NUMSCI` build option (default matching how the kernel is configured for the
  numeric build); the rest of `src/native/*` (math / topology / tessellate / construct /
  boolean / blend / exchange) MUST still build with `CYBERCAD_HAS_NUMSCI=OFF`, so every
  existing host + sim suite is unaffected. NumPP/SciPP are added exactly like OCCT: by
  absolute-path include + link, NOT vendored into the repo.
- **No `cc_*` ABI change**; the default engine stays OCCT. This change ships the OCCT-free
  numeric substrate + closest-point on-ramp; it is not yet engine-wired to a `cc_*`
  closest-point entry (that, and SSI, are follow-ups).

## Verification (two gates, per NATIVE-REWRITE.md)

1. **Host analytic unit tests** (`clang++ -std=c++20`, no OCCT, NumPP/SciPP linked): the
   facade solves known-value analytic problems — `brentq(x²−2)=√2`, `newton(cos x − x)`,
   `fsolve` on a 2×2 nonlinear residual, BFGS on Rosenbrock → (1,1), `least_squares` line
   fit, `solve` / `lstsq` — and closest-point on analytic geometry with a **closed-form**
   answer (nearest `t` on a line / circle, nearest `(u,v)` on a plane / cylinder / sphere,
   nearest `(u,v)` on a bicubic B-spline vs a dense brute-force grid) within tolerance.
2. **Simulator native-vs-OCCT `Extrema` parity** (OCCT linked): on the iOS simulator, the
   native closest-point / projection result is compared against the OCCT `Extrema` oracle
   (`Extrema_ExtPC` for a curve, `Extrema_ExtPS` for a surface, via `Adaptor3d`) at sampled
   3D targets over native curves / surfaces — the returned parameter and the foot-point
   distance within a tight tolerance (fp64).

A requirement is done only when BOTH gates are green AND every existing suite
(`scripts/run-sim-suite.sh` 221/221, host CTest, GPU / Phase-3) stays green with
`CYBERCAD_HAS_NUMSCI=OFF`. This is honestly reported as the **numeric substrate + Extrema
on-ramp** — NOT SSI, NOT curved booleans; those stay OCCT and remain the moat that blocks
#8 `drop-occt`.

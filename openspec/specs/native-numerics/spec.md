# native-numerics Specification

## Purpose
TBD - created by archiving change add-native-numerics. Update Purpose after archive.
## Requirements
### Requirement: Adopt NumPP + SciPP as the OCCT-free numeric substrate (CPU-only, subset-scoped)

The kernel SHALL adopt **NumPP** (`/Users/leonardoaraujo/work/NumPP`) and **SciPP**
(`/Users/leonardoaraujo/work/SciPP`) — the org's C++20, MIT NumPy / SciPy ports — as its
OCCT-free numeric substrate, replacing the hand-written generic-solver line of Phase 4 #2.
Both libraries SHALL be referenced **by absolute path exactly as OCCT is** (via
`CYBERCAD_NUMPP_PREFIX` / `CYBERCAD_SCIPP_PREFIX` `CACHE PATH` hints mirroring
`CYBERCAD_OCCT_INCLUDE_DIR`) and SHALL NOT be vendored into this repository. The adopted
build SHALL be **CPU-only** (no NumPP `NUMPP_WITH_*` BLAS / LAPACK / CUDA / OpenCL / Metal
backend enabled; NumPP's `backend/*_null.cpp` stubs satisfy the linalg vtable). The consumed
SciPP surface SHALL be the numeric subset — **`optimize` / `linalg` (and, as built/linked,
`spatial` / `integrate`)** — and the build SHALL **EXCLUDE SciPP `src/special` and
`src/stats`**, whose failure under Homebrew libc++ is a verified ISO-29124 stdlib gap
(`std::legendre` / `std::cyl_bessel_j` / … absent from libc++), grep-confirmed confined to
`src/special/` and not touched by any numeric module the kernel consumes. The substrate SHALL
build + link for both **HOST** (`clang++ -std=c++20`, libc++) and
**`arm64-apple-ios16.0-simulator`**, pulling in only `libc++` + `libSystem` (no BLAS / LAPACK
/ OpenMP / TBB). Adoption SHALL be recorded in `openspec/project.md` (external numeric deps,
alongside OCCT) and `openspec/NATIVE-REWRITE.md` #2 SHALL link `docs/EVAL-numpp-scipp.md`.

#### Scenario: The consumed NumPP + SciPP subset builds and links for host and iOS-sim
- GIVEN NumPP and SciPP referenced by absolute path (`CYBERCAD_NUMPP_PREFIX` / `CYBERCAD_SCIPP_PREFIX`), NOT vendored, with `CYBERCAD_HAS_NUMSCI=ON`
- WHEN the kernel is configured + built for the HOST (`clang++ -std=c++20`, libc++) and for `arm64-apple-ios16.0-simulator`, building the SciPP `optimize` / `linalg` (+ `spatial` / `integrate`) TUs but EXCLUDING `src/special` and `src/stats`, with no NumPP GPU/BLAS backend
- THEN both targets SHALL compile + link with zero undefined symbols AND SHALL depend only on `libc++` + `libSystem` (no BLAS / LAPACK / OpenMP / TBB) AND the excluded `special` / `stats` TUs SHALL NOT be compiled

#### Scenario: The libc++ special-math gap is scoped out, not worked around in source
- GIVEN SciPP's `src/special/{orthogonal,bessel,expint,spherical_bessel}.cpp` reference C++17 ISO-29124 functions (`std::legendre` / `std::laguerre` / `std::cyl_bessel_j` / `std::cyl_neumann` / `std::cyl_bessel_i` / `std::cyl_bessel_k` / `std::expint`) that Homebrew libc++ does not implement
- WHEN the SciPP build is scoped for the kernel
- THEN the build SHALL EXCLUDE `src/special` and `src/stats` (the offending calls are confined there and unused by the kernel) AND SHALL NOT patch NumPP / SciPP source AND the consumed `optimize` / `linalg` (+ `spatial` / `integrate`) subset SHALL be unaffected

### Requirement: A thin OCCT-free numerics facade over NumPP/SciPP

The change SHALL add a thin, OCCT-free numerics facade under `src/native/numerics/` exposing
the kernel-facing numeric API over NumPP/SciPP so kernel code never touches NumPP `ndarray`
directly: **scalar root** (`newton(f, x0[, fprime])` over `scipp::optimize::newton`;
`brentq(f, a, b)` over `scipp::optimize::brentq`); **nonlinear system solve** (`fsolve(F, x0)`
over `scipp::optimize::fsolve`); **unconstrained minimize** (`minimize(f, x0)` over
`scipp::optimize::minimize(..., "BFGS")`) and **least-squares** (`least_squares(residual, x0)`
over `scipp::optimize::least_squares`, Levenberg-Marquardt); and **dense linear algebra**
(`solve(A, b)` over `scipp::linalg::solve`; `lstsq(A, b)` over `scipp::linalg::lstsq`). Small
native `Vec` / `Mat` adapters SHALL marshal between the kernel's `double` / `math::Point3`
types and NumPP `ndarray`, confining `ndarray` to the facade boundary; the facade SHALL
surface convergence diagnostics (converged flag, iterations, final residual / `‖grad‖`) from
the SciPP result types. The facade SHALL include only STL + NumPP + SciPP + `src/native/math`
and SHALL reference no OCCT / `IEngine` / `EngineShape` type. It SHALL be compiled ONLY under
`CYBERCAD_HAS_NUMSCI=ON`.

#### Scenario: The facade solves analytic known-value problems on the host (no OCCT)
- GIVEN the numerics facade built on the host with NumPP/SciPP linked and no OCCT
- WHEN it is asked to solve `brentq(x²−2)` on `[0,2]`, `newton(cos x − x)`, an `fsolve` 2×2 nonlinear residual, a BFGS `minimize` of the Rosenbrock function from a non-optimal start, a `least_squares` line fit, a `solve` of a 3×3 SPD system, and a `lstsq` of a 4×2 system
- THEN `brentq` SHALL return `√2`, `newton` SHALL return `0.739085…`, `fsolve` SHALL reach residual `< 1e-6`, `minimize` SHALL converge to `(1,1)` within `1e-3`, `least_squares` SHALL recover slope `2` / intercept `1`, `solve` SHALL give residual `< 1e-9`, and `lstsq` SHALL recover the exact fit — each with a converged status surfaced through the facade

#### Scenario: Callers never touch NumPP ndarray
- GIVEN a caller invoking the numerics facade with native `Vec` / `Mat` / `std::function` arguments
- WHEN a facade entry point is called
- THEN all marshalling to and from NumPP `ndarray` SHALL happen inside the facade AND the caller's inputs and returned values SHALL be native kernel types (no `ndarray` in the facade's public signatures)

### Requirement: Native closest-point / projection onto native curves and surfaces (the Extrema on-ramp)

The numerics facade SHALL compute native **closest-point / projection** for a 3D target point
`P`: the nearest parameter `t` on a native curve (minimizing `‖C(t) − P‖²` over the curve's
range) and the nearest parameter pair `(u, v)` on a native surface (minimizing
`‖S(u,v) − P‖²` over the surface domain). Curves SHALL be evaluated through the existing
`src/native/math` curve evaluators (`Line` / `Circle` / `BSpline`) and surfaces through the
surface evaluators (`Plane` / `Cylinder` / `Cone` / `Sphere` / `BSpline` / `Torus`), so the
projector adds no new geometry. It SHALL use **multi-start seeding** for robustness — sample a
coarse parameter grid over the range / domain, refine the best few seeds with SciPP `minimize`
(BFGS) / `least_squares` (LM) each clamped to the parameter bounds, and return the global-best
foot — so a non-convex distance field does not trap the solver in a spurious local minimum.
Each projection SHALL return the foot parameter (`t*` or `(u*, v*)`), the foot point, and the
foot-point distance, and SHALL match the OCCT `Extrema` result (`Extrema_ExtPC` for a curve,
`Extrema_ExtPS` for a surface, via `Adaptor3d`) within tolerance. This projector SHALL remain
OCCT-free (NumPP/SciPP are NOT OCCT) and SHALL be compiled ONLY under `CYBERCAD_HAS_NUMSCI=ON`.

#### Scenario: Closest-point on analytic geometry matches the closed-form answer (host)
- GIVEN native analytic geometry built on the host with no OCCT — a line, a circle, a plane, a cylinder, and a sphere — and a set of 3D target points
- WHEN the projector computes the nearest `t` on the curves and the nearest `(u, v)` on the surfaces
- THEN each returned foot point SHALL equal the closed-form nearest point within tolerance AND the returned foot parameter SHALL lie within the geometry's parameter bounds

#### Scenario: Closest-point on a B-spline surface matches a dense brute-force reference (host)
- GIVEN a native bicubic B-spline surface built on the host with no OCCT and a 3D target point
- WHEN the projector computes the nearest `(u, v)` with multi-start seeding + SciPP refinement
- THEN the returned foot-point distance SHALL match a dense (e.g. 400×400) brute-force grid minimum to within `~1e-5` AND the returned `(u,v)` SHALL be a converged minimizer (small distance gradient), matching the eval's EXP1 result

#### Scenario: Multi-start seeding avoids a spurious local minimum
- GIVEN a native curved surface and a 3D target point positioned so the squared-distance field has more than one local minimum (e.g. a point near the surface's medial axis)
- WHEN the projector runs its coarse-grid multi-start seeding and refines the best seeds
- THEN the returned foot SHALL be the GLOBAL-best (smallest-distance) foot AND SHALL NOT be a spurious higher-distance local minimum that a single naive seed would have produced

#### Scenario: Native closest-point matches OCCT Extrema on the simulator (parity — the correctness gate)
- GIVEN native curves (line / circle / B-spline) and surfaces (plane / cylinder / cone / sphere / B-spline) on a booted iOS simulator (OCCT linked) and sampled 3D target points
- WHEN the native projector computes nearest `t` / nearest `(u, v)` and OCCT `Extrema_ExtPC` / `Extrema_ExtPS` (via `Adaptor3d`) computes the oracle feet for the same targets
- THEN the native foot parameter and the native foot-point distance SHALL match the OCCT `Extrema` oracle within a tight fp64 tolerance (analytic feet fp-exact; B-spline feet within the tight tolerance)

### Requirement: The numerics module is guarded so the rest of src/native builds without NumPP/SciPP

The dependency on NumPP/SciPP SHALL be confined to `src/native/numerics/`. Every other
`src/native/*` module (`math`, `topology`, `tessellate`, `construct`, `boolean`, `blend`,
`exchange`) SHALL NOT gain a NumPP/SciPP dependency, and the whole kernel — minus the numerics
module — SHALL build and pass its existing suites with `CYBERCAD_HAS_NUMSCI=OFF` (the
default). The `src/native/numerics/*` sources SHALL be excluded from the default source glob
and added ONLY inside the `CYBERCAD_HAS_NUMSCI` build block, mirroring how `src/engine/occt/*`
is gated by `CYBERCAD_HAS_OCCT`. This change SHALL NOT change the `cc_*` ABI and SHALL NOT
change the default engine (stays OCCT); with `CYBERCAD_HAS_NUMSCI=OFF` the kernel behaviour
SHALL be identical to today.

#### Scenario: With CYBERCAD_HAS_NUMSCI=OFF nothing changes
- GIVEN the kernel configured with `CYBERCAD_HAS_NUMSCI=OFF` (the default)
- WHEN it is built and every existing suite is run (host CTest, `scripts/run-sim-suite.sh`, GPU / Phase-3)
- THEN no NumPP/SciPP header or source SHALL be compiled AND `src/native/numerics/*` SHALL NOT be compiled AND every existing suite SHALL pass unchanged (`run-sim-suite.sh` 221/221) AND the `cc_*` ABI and the default engine (OCCT) SHALL be unchanged

### Requirement: SSI and curved booleans stay OCCT (out of scope, honest)

The change SHALL NOT implement surface-surface intersection (SSI), curve-curve /
curve-surface intersection, or curved / general booleans. The marching-line algorithm,
seeding / subdivision, and tangent / coincident-surface robustness (OCCT `IntPatch` /
`IntWalk` + BOPAlgo) are the NEXT capability (#5) and SHALL remain OCCT-backed. The spec SHALL
state this split truthfully: `native-numerics` is the numeric **substrate** plus the
closest-point / projection **on-ramp** to SSI; the hard SSI core is NOT bought by adopting
NumPP/SciPP (the eval's EXP2b naive-seed 0/7 and EXP2c near-tangent both-solver-fail are the
evidence) and remains the moat that, with shape healing and STEP/IGES import, blocks #8
`drop-occt`.

#### Scenario: No SSI / curved-boolean code path is added
- GIVEN the `native-numerics` change
- WHEN its scope is examined
- THEN it SHALL add NO surface-surface / curve-curve / curve-surface intersection and NO curved / general boolean code path AND SHALL document that those stay OCCT (capability #5) AND SHALL NOT claim to unblock #8 `drop-occt`

#### Scenario: The near-tangent SSI failures are attributed to the deferred capability, not to closest-point
- GIVEN the eval's EXP2b (naive-seed `fsolve` finds 0/7 freeform surface-surface intersection points) and EXP2c (both Newton `fsolve` and damped LM `least_squares` fail at near-tangent d≥2.99)
- WHEN the scope boundary is stated
- THEN those failures SHALL be attributed to the SSI geometry layer that is the NEXT capability (#5) — NOT to the closest-point / projection delivered here, which is a well-conditioned single-target minimization proven usable (EXP1 err 7.4e-6)


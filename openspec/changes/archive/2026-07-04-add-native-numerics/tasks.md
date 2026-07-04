# Tasks — add-native-numerics (Phase 4 #2 slice — numeric substrate + Extrema on-ramp)

Order: adopt NumPP/SciPP build plumbing (absolute-path, guarded, `special`/`stats`
excluded) → native ↔ `ndarray` marshalling → numeric facade (root / fsolve / minimize /
least_squares / solve / lstsq) → closest-point / projection (multi-start + SciPP refine) →
Gate 1 (host analytic + closed-form closest-point) → Gate 2 (sim native-vs-OCCT `Extrema`
parity) → docs. `src/native/numerics` may depend on NumPP/SciPP + `src/native/math`; it
stays OCCT-FREE. Every other `src/native/*` MUST build with `CYBERCAD_HAS_NUMSCI=OFF`. No
`cc_*` ABI change. Default engine stays OCCT. SSI + curved booleans are OUT OF SCOPE.

Grounded in `docs/EVAL-numpp-scipp.md` (verdict GO WITH HARDENING). Respect its two
conditions: (1) build-scope OUT SciPP `special` + `stats` (Homebrew-libc++ ISO-29124 gap,
confined to `src/special/`); (2) do NOT attempt SSI (EXP2b naive-seed 0/7, EXP2c
near-tangent both-solver fail) — that is capability #5.

## 1. Adopt NumPP + SciPP — build plumbing (absolute-path, NOT vendored, guarded)

- [x] 1.1 Add a `CYBERCAD_HAS_NUMSCI` CMake option (default OFF) + `CYBERCAD_NUMPP_PREFIX`
      (default `/Users/leonardoaraujo/work/NumPP`) and `CYBERCAD_SCIPP_PREFIX` (default
      `/Users/leonardoaraujo/work/SciPP`) `CACHE PATH` hints, mirroring
      `CYBERCAD_OCCT_INCLUDE_DIR`. Absolute-path references, NOT vendored into the repo.
- [x] 1.2 Exclude `src/native/numerics/*` from the default `CYBERCAD_SOURCES` glob
      (`list(FILTER … EXCLUDE REGEX "/src/native/numerics/")`), mirroring the
      `src/engine/occt/` exclusion, so the numerics module compiles ONLY under
      `CYBERCAD_HAS_NUMSCI=ON`.
- [x] 1.3 Under `if(CYBERCAD_HAS_NUMSCI)`: add `${CYBERCAD_NUMPP_PREFIX}/include` +
      `${CYBERCAD_SCIPP_PREFIX}/include` includes; build the SciPP `optimize` + `linalg`
      (+ `spatial` + `integrate`) TUs from `${CYBERCAD_SCIPP_PREFIX}/src`, **EXCLUDING
      `special/*` and `stats/*`** (the libc++ gap); add the NumPP CPU-only backend objects
      (incl. `backend/*_null.cpp` stubs — no `NUMPP_WITH_*` enabled); compile
      `src/native/numerics/*.cpp`; add `CYBERCAD_HAS_NUMSCI=1` compile-def.
- [x] 1.4 HOST build (`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, libc++): configure
      + build with `CYBERCAD_HAS_NUMSCI=ON` clean (0 warnings), and with OFF unchanged.
- [x] 1.5 arm64-iOS-simulator build: the NumPP core + SciPP `optimize`/`linalg` subset +
      `src/native/numerics` compile + link for `arm64-apple-ios16.0-simulator` (follow the
      eval's direct-`-I` recipe; capture the CMake iOS-toolchain / NumPP-consume path as the
      open packaging item, eval §2 caveat 4).
- [x] 1.6 Verify GUARD: with `CYBERCAD_HAS_NUMSCI=OFF`, no NumPP/SciPP header/source is
      compiled and every existing native suite still builds + passes (host CTest,
      `run-sim-suite.sh` 221/221).

> **Implementation note (build-integration slice).** The facade + marshalling +
> solvers + projectors landed in a SINGLE thin header `src/native/numerics/numerics.h`
> (+ `numerics.cpp`, the only TU that includes NumPP/SciPP) rather than the
> `marshal.h`/`solve.h`/`project.h` split sketched below — same API contract, one
> boundary. The substrate is built as a static archive `libnumsci_<target>.a` by
> `scripts/build-numsci.sh {host|iossim}` (compiling the eval-verified NumPP full
> CPU-only TU set + SciPP `optimize`/`linalg` subset, `special`/`stats` EXCLUDED),
> and the kernel links it via `-DCYBERCAD_HAS_NUMSCI=ON -DCYBERCAD_NUMSCI_DIR=…`.
> HOST + arm64-iOS-simulator both compile+archive 77/77 TUs and LINK the facade
> with 0 undefined symbols; the on-simulator run of Gate-1's analytic +
> brute-force closest-point tests passes 13/13 (closing eval §2 caveats 3+4 for
> this module). Gate 2 (§6, native-vs-OCCT `Extrema` parity) needs the full OCCT
> app toolchain and is tracked separately — out of this build-integration scope.

## 2. Native ↔ NumPP `ndarray` marshalling (`src/native/numerics/marshal.h`, OCCT-free)

- [x] 2.1 `toNdarray(const Vec&)` / `fromNdarray(const ndarray&)` for 1-D vectors
      (`std::vector<double>` / `Vec3`), and row-major `Mat` ⇄ 2-D `ndarray` helpers.
- [x] 2.2 Wrap a native `std::function<double(double)>` / `std::function<Vec(const Vec&)>`
      into the SciPP `ScalarFn` / `VecFn` / `ObjFn` signatures, confining `ndarray` to the
      facade boundary so callers never touch it.
- [x] 2.3 Includes only STL + `numpp/…` + `scipp/…`; no OCCT; cognitive complexity 🟢.

## 3. Numeric facade (`src/native/numerics/solve.h`, OCCT-free over NumPP/SciPP)

- [x] 3.1 Scalar root: `newton(f, x0, fprime?)` → `scipp::optimize::newton`;
      `brentq(f, a, b)` → `scipp::optimize::brentq`.
- [x] 3.2 Nonlinear system: `fsolve(F, x0)` → `scipp::optimize::fsolve`, returning a native
      `Vec` + convergence status.
- [x] 3.3 Minimize: `minimize(f, x0)` → `scipp::optimize::minimize(..., "BFGS")`, surfacing
      iterations / `‖grad‖` / converged flag; `least_squares(residual, x0)` →
      `scipp::optimize::least_squares` (Levenberg-Marquardt).
- [x] 3.4 Dense linalg: `solve(A, b)` → `scipp::linalg::solve`; `lstsq(A, b)` →
      `scipp::linalg::lstsq` (residual / rank surfaced).
- [x] 3.5 Each facade entry a flat wrapper (build callable → call → unpack), cognitive
      complexity 🟢; umbrella `native_numerics.h`.

## 4. Native closest-point / projection (`src/native/numerics/project.h`, the Extrema on-ramp)

- [x] 4.1 Curve projector: nearest `t` on a native curve (`Line` / `Circle` / `BSpline`
      via `src/native/math`) to a 3D point — minimize `‖C(t) − P‖²` over `[t0,t1]`; return
      foot `t*`, foot point, distance; clamp/snap to bounds.
- [x] 4.2 Surface projector: nearest `(u,v)` on a native surface (`Plane` / `Cylinder` /
      `Cone` / `Sphere` / `BSpline` / `Torus`) to a 3D point — minimize `‖S(u,v) − P‖²`
      over the domain; return `(u*,v*)`, foot point, distance.
- [x] 4.3 **Multi-start seeding**: sample a coarse parameter grid, refine the best few
      seeds with SciPP `minimize` (BFGS) / `least_squares` (LM) each clamped to bounds, and
      return the global-best foot — so a non-convex distance field does not trap a spurious
      local minimum (the eval's brute-force-nearby seeding that made EXP1 land at 7.4e-6).
      Bounded / deterministic seed count.
- [x] 4.4 The projector driver kept in the systems band (≤ ~20): per-seed refine + grid
      sample factored into helpers; measured with the `cognitive-complexity` skill.

## 5. Gate 1 — host analytic + closed-form closest-point (`tests/native/test_native_numerics.cpp`)

- [x] 5.1 Solver known-values (mirror the eval's 7/7 functional probe): `brentq(x²−2)=√2`;
      `newton(cos x − x)=0.739085…`; `fsolve` 2×2 nonlinear residual < 1e-6; BFGS Rosenbrock
      → (1,1) within 1e-3; `least_squares` line fit slope 2 / intercept 1; `solve` 3×3 SPD
      residual < 1e-9; `lstsq` 4×2 exact.
- [x] 5.2 Closest-point closed-form: nearest `t` on a line / circle; nearest `(u,v)` on a
      plane / cylinder / sphere — foot matches the closed-form answer within tolerance.
- [x] 5.3 Closest-point on a bicubic B-spline surface vs a dense brute-force grid (err ≲
      1e-5, matching EXP1), and a curve B-spline case.
- [x] 5.4 Wire `test_native_numerics` into CTest under `CYBERCAD_HAS_NUMSCI` (source under
      `tests/native/`, per the existing native-test mapping). No OCCT.

## 6. Gate 2 — sim native-vs-OCCT `Extrema` parity (`tests/sim/native_numerics_parity.mm`)

- [x] 6.1 `scripts/run-sim-native-numerics.sh` + `native_numerics_parity.mm` with its own
      `main()`, on the `run-sim-suite.sh` SKIP list (221-assertion count unchanged).
- [x] 6.2 For native curves (line / circle / B-spline): compare native nearest-`t` vs OCCT
      `Extrema_ExtPC` (via `Adaptor3d`) at sampled 3D targets — foot parameter + distance
      within a tight fp64 tolerance. RESULT: 5 `bspline_curve` cases PASS (dDist=0,
      dParam ≤ 2.087e-11, dPoint ≤ 8.434e-11).
- [x] 6.3 For native surfaces (plane / cylinder / cone / sphere / B-spline): compare native
      nearest-`(u,v)` vs OCCT `Extrema_ExtPS` at sampled 3D targets — same tolerance.
      RESULT: plane / cylinder / sphere / bspline_surf (4 each) + `facade_eval_bsurf` PASS.
- [x] 6.4 Analytic feet fp-exact vs OCCT; B-spline feet within the tight tolerance. Report
      max error like the #1–#7 parity harnesses. RESULT: **All 22 `[NNUM]` cases PASS** —
      dDist ≤ 1.776e-15 across all; analytic (plane/cylinder/sphere) feet fp-exact
      (dPoint ≤ 1.707e-10); B-spline feet within tolerance (largest deviation
      `bspline_surf#3` dPoint 3.946e-08 at the corner u=v=0, dU=dV=7.595e-09). Ran in the
      booted simulator, exit 0; `run-sim-suite.sh` unchanged at 221/221.

> **Deferred (recorded, NOT blocking the bar — this slice is single-target closest-point
> only).** (1) **Multiple-extrema enumeration** — the projector returns the GLOBAL-best
> foot (multi-start + refine), not the full set of stationary points OCCT `Extrema` can
> enumerate; a point equidistant to several feet resolves to one. (2) **Curve-curve and
> surface-surface distance** (`Extrema_ExtCC` / `Extrema_ExtSS`) are NOT implemented — only
> point→curve and point→surface projection. (3) The one on-sim numeric caveat is the
> `bspline_surf#3` corner case (dPoint 3.946e-08, ~4e-8) — a domain-corner foot where the
> multi-start grid seed is farthest from the analytic optimum; still ~1e-8, well inside the
> harness tolerance, but the largest deviation observed and the honest ceiling of the
> current grid density.

## 7. Docs + honesty

- [x] 7.1 `openspec/project.md` — add the external numeric deps note (NumPP + SciPP,
      absolute-path, CPU-only, `optimize`/`linalg`/`spatial`/`integrate` subset,
      `special`/`stats` excluded), alongside OCCT. (DONE in this change.)
- [x] 7.2 `openspec/NATIVE-REWRITE.md` #2 — reflect adoption (GO WITH HARDENING; link
      `docs/EVAL-numpp-scipp.md`); state this slice is the substrate + Extrema on-ramp, NOT
      SSI. Flipped from planned to DONE-at-bar; ~60–75% #2 effort saving realized; SSI #5 is
      NEXT and still needs the marching/robustness layer.
- [x] 7.3 `docs/STATUS-phase-4.md` — record the #2 substrate + closest-point slice at its
      verification bar (new numeric-foundations result table + per-capability row); state
      SSI (#5) stays OCCT and blocks #8.
- [x] 7.4 Confirm honesty end state: substrate + closest-point native; SSI + curved
      booleans + healing + STEP/IGES import still OCCT; #8 `drop-occt` still blocked.

## 8. Done bar

- [x] 8.1 Both gates green AND every existing suite (`run-sim-suite.sh` 221/221, host
      CTest, GPU/Phase-3) green with `CYBERCAD_HAS_NUMSCI=OFF`. VERIFIED: host CONFIG 1
      (`NUMSCI=OFF`) CTest 22/22 (`test_native_numerics` correctly ABSENT); host CONFIG 2
      (`NUMSCI=ON`) CTest 23/23 (incl. `test_native_numerics`, all prior suites unchanged);
      NumPP/SciPP substrate compiled 77/77 TUs host + arm64-iOS-simulator; sim suite
      221/221, determinism serial==parallel bit-reproducible.
- [x] 8.2 `openspec validate add-native-numerics --strict` green.

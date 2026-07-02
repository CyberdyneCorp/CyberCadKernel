# Tasks — add-native-math-geometry

Verification levels: **host** = the native library compiles and unit-tests with
`/opt/homebrew/opt/llvm/bin/clang++ -std=c++20` with NO OCCT and NO simulator,
asserting analytic/known-value results (the first roadmap gate); **sim-parity** =
on a booted iOS simulator (OCCT linked ONLY in the test), the native result is
compared element-by-element against the OCCT oracle (`gp_*`, `BSplCLib`,
`BSplSLib`, `PLib`, `ElSLib`) within the documented tight fp64 tolerance (the
second roadmap gate). A requirement is done only when BOTH gates are green AND
every existing suite (`scripts/run-sim-suite.sh` 221/221, host CTest, GPU/Phase-3
suites) stays green. No `cc_*` ABI change; no engine wiring.

## 0. OCCT-free library seam
- [x] 0.1 Create `src/native/math/` (headers + sources) with NO OCCT include in any
  file; add a host CTest target that builds it with `clang++ -std=c++20` (no OCCT,
  no simulator) and a separate simulator parity target that links OCCT only in the
  test. (**host**)
- [x] 0.2 Add a guard/check that the library translation units include no OCCT
  header (grep gate in the host build). (**host**)

## 1. Value types (vec3 / point3 / dir3 / 4×4 transform)
- [x] 1.1 Implement `vec3` / `point3` / `dir3` (fp64; `dir3` unit-normalized with a
  documented degenerate-input precondition) and vector algebra (add/sub/scale/dot/
  cross/length/normalize). (**host**)
- [x] 1.2 Implement the 4×4 affine `transform`: `compose`, `invert`, and `apply` to
  a point (translated), a free vector (not translated), and a direction (linear
  part + re-normalize), with the point/vector convention documented and matched to
  `gp_Trsf`. (**host**)
- [x] 1.3 Host analytic tests: identity is neutral, `compose` associativity,
  `invert` round-trip (T·T⁻¹ = I within fp64 tol), point vs vector translation
  semantics, direction stays unit. (**host**)
- [x] 1.4 Parity: native `transform` compose/invert/apply match `gp_Trsf`
  (built from the same rotation/translation) within the tight fp64 tolerance over
  sampled transforms and points/vectors/dirs. (**sim-parity**)

## 2. Curve evaluation — Bézier (de Casteljau)
- [x] 2.1 Implement Bézier curve point + derivatives via de Casteljau (hodograph
  for derivatives); rational (weighted) Bézier via homogeneous coordinates +
  quotient rule. (**host**)
- [x] 2.2 Host analytic tests: endpoints interpolate control points (`t=0`,`t=1`),
  a known mid value (`t=0.5`), first derivative matches the closed-form hodograph,
  a rational quarter-circle lies on the exact circle. (**host**)
- [x] 2.3 Parity: native Bézier point + derivative match the OCCT oracle
  (`PLib` / Bézier eval) across sampled `t` and random (weighted) control polygons
  within the tight fp64 tolerance. (**sim-parity**)

## 3. Curve evaluation — B-spline / NURBS (basis funcs / de Boor)
- [x] 3.1 Implement `FindSpan` (A2.1), `BasisFuns` (A2.2) + derivative basis,
  `CurvePoint` (A3.1), `CurveDerivs` (A3.2); rational NURBS via homogeneous
  coordinates + quotient rule for derivatives (weights all 1 ⇒ polynomial path). (**host**)
- [x] 3.2 Host analytic tests: a degree-1 B-spline reproduces the control polygon;
  a known cubic B-spline point; a NURBS circle arc lies on the exact unit circle
  (point + tangent direction). (**host**)
- [x] 3.3 Parity: native B-spline/NURBS curve point + first derivative match
  `BSplCLib` (built from the same knots/degree/poles/weights) across sampled
  parameters (including near knots and endpoints) within the tight fp64 tolerance. (**sim-parity**)

## 4. Surface evaluation — tensor-product Bézier / B-spline / NURBS
- [x] 4.1 Implement tensor-product surface point + first partials (`dS/du`,`dS/dv`)
  via `SurfacePoint` (A3.5) / `SurfaceDerivs` (A3.6) (and de Casteljau for the
  Bézier case); rational via homogeneous coordinates + quotient rule; unit normal
  = `normalize(dS/du × dS/dv)` with a documented orientation and degenerate (pole /
  parallel partials) handling. (**host**)
- [x] 4.2 Host analytic tests: a bilinear (degree-1×1) patch reproduces its corner
  points and has the expected flat normal; a known Bézier patch mid value; a
  cylindrical NURBS patch normal is radial. (**host**)
- [x] 4.3 Parity: native surface point, `dS/du`, `dS/dv`, and normal match
  `BSplSLib` (same knots/degrees/control net/weights) across a sampled `(u,v)` grid
  within the tight fp64 tolerance (normal compared up to the documented
  orientation). (**sim-parity**)

## 5. Elementary surfaces (plane / cylinder / cone / sphere)
- [x] 5.1 Implement plane, cylinder, cone, sphere point + unit normal from their
  closed-form parametrizations, parameter/normal conventions documented and matched
  to `ElSLib`. (**host**)
- [x] 5.2 Host analytic tests: unit-sphere point at `(u,v)` has normal equal to the
  radial direction; cylinder normal is radial and independent of the axis
  coordinate; cone normal tilts by the half-angle; plane normal is constant. (**host**)
- [x] 5.3 Parity: native elementary-surface point + normal match `ElSLib`
  (`Value` / `D1` / normal) for plane/cylinder/cone/sphere across sampled `(u,v)`
  within the tight fp64 tolerance. (**sim-parity**)

## 6. Determinism
- [x] 6.1 Fixed evaluation order (de Boor / de Casteljau reduction, basis
  accumulation) so repeated evaluations of the same input are bit-identical;
  add a repeat-run equality assertion in the host suite. (**host**)

## 7. Validation
- [x] 7.1 Host CTest target green: all analytic value-type / curve / surface /
  elementary-surface tests pass under `clang++ -std=c++20` with no OCCT. (**host**)
- [x] 7.2 Simulator native-vs-OCCT parity target green within the documented tight
  fp64 tolerance for transforms, curves, surfaces, and elementary surfaces; every
  existing suite stays green (`scripts/run-sim-suite.sh` 221/221, host CTest,
  GPU/Phase-3 suites). (**sim-parity**)
- [x] 7.3 Confirm no `cc_*` signature / POD struct change and no engine wiring
  (the library is not reachable through the facade in this change). (**host**)
- [x] 7.4 `openspec validate add-native-math-geometry --strict` green; mark
  `native-math` in-progress status in `openspec/NATIVE-REWRITE.md` / `ROADMAP.md`
  Phase 4.

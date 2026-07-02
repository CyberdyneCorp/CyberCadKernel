## Context

Phase 4 (`openspec/NATIVE-REWRITE.md`) replaces the OCCT adapter with native
C++20, one capability at a time, behind the unchanged `cc_*` facade. The locked
dependency order puts `native-math` first: every later capability (topology,
tessellation, swept solids, booleans, blends) is ultimately expressed in terms of
evaluating curves and surfaces in a small fp64 algebra. So the math layer must
land first, and it must be **OCCT-free and host-buildable** — that property is
what enables the two independent verification gates the roadmap requires:

1. **Host unit tests** — the library compiles and unit-tests with
   `/opt/homebrew/opt/llvm/bin/clang++ -std=c++20`, no OCCT, no simulator, against
   analytic known values.
2. **Simulator native-vs-OCCT parity** — on the simulator (OCCT linked) the native
   result is compared against the OCCT oracle at sampled inputs within a tight
   fp64 tolerance.

Constraints:
- **OCCT-free library.** No file under `src/native/math/` may include any OCCT
  header. The library links no OCCT. OCCT appears ONLY in the simulator parity
  test, which includes the native headers and OCCT side by side to compare.
- **Host-buildable.** The library + host tests build with `clang++ -std=c++20`
  and nothing else.
- **Clean-room.** Implemented from math first principles and public references
  (*The NURBS Book*, de Casteljau). OCCT source is a numeric/convention oracle,
  consulted to confirm the algorithm and compare numbers — never copied verbatim.
- **No ABI change, no engine wiring.** The `cc_*` facade and the active engine are
  untouched; this change ships the library + its tests only.
- **Determinism.** fp64 throughout, a fixed evaluation order (e.g. left-to-right de
  Boor, fixed de Casteljau reduction), reproducible run to run.
- **Maintainability first.** Clear, well-named C++20 (`constexpr`/`span`/concepts
  where natural), documented algorithms with their reference citation, cognitive
  complexity in the systems band (≤ 25–35 for the irreducible evaluators, flagged
  where it is inherent to the recurrence).

## Goals / Non-Goals

Goals:
- fp64 value types: `vec3`, `point3`, `dir3`, 4×4 affine `transform` (compose,
  invert, apply-to-point / -vector / -direction).
- Curve evaluation: Bézier (de Casteljau) and B-spline/NURBS (basis funcs / de
  Boor) point + derivatives, rational weights via homogeneous coordinates.
- Surface evaluation: tensor-product Bézier/B-spline/NURBS point + `dS/du`, `dS/dv`
  + unit normal, rational weights in homogeneous form.
- Elementary surfaces: plane/cylinder/cone/sphere point + unit normal.
- Two-gate verification: host analytic tests + simulator native-vs-OCCT parity.

Non-Goals:
- Any topology, B-rep, tessellation, or engine wiring (later capabilities).
- Any `cc_*` ABI change or facade exposure.
- Curve/surface *interpolation/approximation/fitting*, intersection, or projection
  (only forward evaluation here).
- Second derivatives beyond what point + first partials + normal require (higher
  derivatives can be added when a consumer needs them).
- GPU / fp32 paths (this is the exact fp64 core; GPU eval is `gpu-tessellation`).

## Decisions

- **Directory + build seam.** All native math lives under `src/native/math/`
  (headers + sources), OCCT-free. A host CTest target compiles it with
  `clang++ -std=c++20` and links only the analytic tests. A separate simulator
  parity target links OCCT and includes the native headers to compare — OCCT never
  enters the library translation units.
- **Value types are plain fp64 structs.** `vec3`/`point3` are 3×`double`;
  `dir3` is a `vec3` maintained unit-length (normalized on construction, guarding
  the near-zero case). The 4×4 `transform` is a row/column-fixed `double[4][4]`
  (convention documented once and matched to `gp_Trsf`'s point/vector semantics):
  `apply` to a `point3` includes translation, to a `vec3` (free vector) excludes
  it, to a `dir3` applies the linear part and re-normalizes. `compose` is matrix
  product in a fixed order; `invert` uses the affine inverse (inverse of the linear
  block, negated-and-transformed translation) with a general fallback. These are
  small, `constexpr`-friendly, and independently unit-testable (identity,
  round-trip, associativity).
- **Bézier by de Casteljau.** Curve and (tensor-product) surface Bézier use the de
  Casteljau recurrence for the point; derivatives from the standard hodograph /
  the difference of the last-stage intermediate points. Chosen over explicit
  Bernstein sums for numerical stability and because it is the clean-room canonical
  method. Oracle: `PLib` / OCCT Bézier eval.
- **B-spline/NURBS by basis functions + de Boor.** `FindSpan` (A2.1) locates the
  knot span; `BasisFuns` (A2.2) / derivative basis compute the nonzero basis
  values; `CurvePoint` (A3.1) / `CurveDerivs` (A3.2) and `SurfacePoint` (A3.5) /
  `SurfaceDerivs` (A3.6) combine them. Knot vector, degree, and control points are
  passed as spans; the same span machinery serves curves and both surface
  directions. Oracle: `BSplCLib` / `BSplSLib`.
- **Rational (NURBS) via homogeneous coordinates.** Rational curves/surfaces are
  evaluated by lifting control points to homogeneous `(w·P, w)`, evaluating the
  (non-rational) B-spline/Bézier there, then projecting. First derivatives use the
  quotient rule (Aw, w and their derivatives → the rational derivative), the
  standard NURBS-book approach. This keeps one evaluator for both polynomial and
  rational cases (weights all 1 ⇒ polynomial), reducing complexity and matching
  OCCT's rational handling.
- **Surface normal from partials.** The unit normal is
  `normalize(dS/du × dS/dv)`, computed from the same evaluation that produces the
  partials (no finite differencing), with a documented orientation convention
  matched to the oracle; the degenerate (parallel partials / pole) case is flagged.
- **Elementary surfaces closed-form.** Plane/cylinder/cone/sphere use their direct
  parametrizations and analytic normals (oracle: `ElSLib`), not a NURBS
  conversion — cheaper, exact, and independently checkable against closed-form
  values. Parameter conventions (angular ranges, `v` axis, cone half-angle,
  outward normal) documented and matched to `ElSLib`.
- **Complexity is isolated and flagged.** The de Boor / basis-derivative and
  `SurfaceDerivs` routines are the irreducible cores; they are kept in small
  single-purpose functions, documented with their A-number reference, and allowed
  into the systems band (≤ 25–35), flagged where the recurrence inherently drives
  it. Everything else stays well under 15.

## Verification

Each requirement is verifiable two ways (the roadmap's two gates):

- **(a) Host analytic unit test** — `clang++ -std=c++20`, no OCCT, no simulator.
  Asserts known closed-form values: a Bézier point at `t=0/0.5/1`, a de-Boor point
  on a known B-spline, a NURBS circle arc lying on the exact circle, transform
  identity/round-trip/associativity, and exact elementary-surface points/normals
  (e.g. unit-sphere normal equals the radial direction).
- **(b) Simulator native-vs-OCCT parity** — links the native headers and the OCCT
  oracle (`gp_Trsf`, `BSplCLib`/`BSplSLib`, `PLib`, `ElSLib`) side by side and
  samples inputs (parameters across the domain, random control nets/weights),
  asserting the native result matches OCCT within a tight fp64 tolerance
  (absolute + relative), including derivatives and normals.

The host gate pins the math to ground truth so a convention drift cannot pass; the
parity gate pins conventions (knot multiplicity, weight normalization, parameter
ranges, normal orientation) to OCCT so the later engine wiring is a drop-in.

## Risks / Trade-offs

- **Convention mismatch with OCCT** (knot handling, weight normalization, parameter
  ranges, normal orientation). Mitigation: the parity gate compares directly
  against OCCT; the host gate independently pins values, so a mismatch surfaces in
  at least one gate.
- **Numerical divergence in derivatives / near degeneracies** (poles, parallel
  partials, near-zero direction). Mitigation: documented degenerate-case handling
  and parity sampling that includes near-boundary parameters; tolerance chosen tight
  but above fp64 rounding of the differing-but-equivalent algorithms.
- **Cognitive complexity of the evaluators.** Mitigation: isolate the irreducible
  recurrences in small documented functions, flag them per the systems band, and
  keep the surrounding code simple.
- **Duplication of eval logic in the parity test.** Mitigation: the parity test
  calls the same native functions the host test does; only the OCCT comparison is
  test-local.

## Migration Plan

1. Create `src/native/math/` with the value types (`vec3`/`point3`/`dir3`/4×4
   `transform`) and a host CTest target building with `clang++ -std=c++20`, no
   OCCT. Add analytic value-type tests (identity, round-trip, associativity).
2. Add Bézier curve/surface (de Casteljau) point + derivatives; host analytic
   tests (known control-point values, endpoint interpolation).
3. Add B-spline/NURBS curve eval (`FindSpan`/`BasisFuns`/`CurvePoint`/`CurveDerivs`,
   rational via homogeneous); host tests (NURBS circle arc on the exact circle).
4. Add tensor-product B-spline/NURBS surface eval
   (`SurfacePoint`/`SurfaceDerivs`, partials + normal); host tests.
5. Add elementary surfaces (plane/cylinder/cone/sphere) point + normal; host tests
   against closed-form normals.
6. Add the simulator native-vs-OCCT parity target (OCCT linked only there) covering
   transforms, curves, surfaces, and elementary surfaces within the tight fp64
   tolerance; keep every existing suite green.
7. `openspec validate add-native-math-geometry --strict` green; reflect the
   `native-math` in-progress status in `openspec/NATIVE-REWRITE.md` / `ROADMAP.md`.

## Open Questions

- The exact fp64 parity tolerance constant (absolute + relative) — set from the
  parity suite so equivalent-but-different algorithms agree while a real bug fails.
- Whether `dir3` normalization guards against a caller-supplied zero vector by
  contract (assert) or by returning a sentinel — resolve at implementation, default
  to a documented precondition + debug assert.
- Whether to expose second derivatives now or defer until a consumer
  (`native-blends` curvature) needs them — deferred here; the derivative machinery
  is written to extend to order `k` so adding them later is additive.

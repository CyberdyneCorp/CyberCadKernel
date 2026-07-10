# Design — nurbs-exact-geometry-kernel

## Placement & conventions

New module `src/native/math/bspline_ops.{h,cpp}`, namespace `cybercad::native::math`, beside
`bspline.{h,cpp}` / `bezier.{h,cpp}`. Reuses `math::Point3` / `Vec3` (`native/math/vec.h`)
and the existing basis routines (`findSpan`, `basisFuns`) where useful. OCCT-free,
NumPP/SciPP-free, fp64, deterministic. Compiled into the core lib by the `src/native` glob;
no CMake guard. Added to `native_math.h` aggregator.

Conventions match the rest of the kernel and OCCT exactly (so SIM parity is a direct
compare): **flat knot vectors** with expanded multiplicities, length `nPoles + degree + 1`;
**row-major, U-outer** surface poles `pole(i,j) = poles[i*nPolesV + j]`; weights stored
**separately** (empty ⇒ non-rational), one per pole.

## Data types (mirror the free-form fields of `EdgeCurve` / `FaceSurface`)

```cpp
struct BsplineCurveData {
  int degree = 0;
  std::vector<Point3> poles;
  std::vector<double> weights;   // empty => non-rational
  std::vector<double> knots;     // flat, length = poles.size() + degree + 1
};

struct BsplineSurfaceData {
  int degreeU = 0, degreeV = 0;
  int nPolesU = 0, nPolesV = 0;
  std::vector<Point3> poles;     // row-major, U outer: pole(i,j)=poles[i*nPolesV+j]
  std::vector<double> weights;   // empty => non-rational
  std::vector<double> knotsU, knotsV;
};

enum class ParamDir { U, V };    // surface direction selector
```

Trivial adapters convert to/from `topology::EdgeCurve` (Kind::BSpline) and
`topology::FaceSurface` (Kind::BSpline); the ops themselves are `shape.h`-independent so they
unit-test in isolation.

## Rational handling (uniform)

A rational op lifts each pole `P=(x,y,z)` with weight `w` to the homogeneous point
`Pʷ=(w·x, w·y, w·z, w)` in R⁴, runs the **non-rational** algorithm on the R⁴ control net
(knots/degree unchanged by the lift), then projects back: `P' = (x'/w', y'/w', z'/w')`,
`w' = w'`. Non-rational inputs (`weights` empty) skip the lift and treat R³ directly. A
single internal `Homog4{x,y,z,w}` helper + one templated core per algorithm keeps curve and
homogeneous paths from duplicating. Non-positive projected weights are a documented guard
(return failure, never divide-by-≤0).

## API surface (curves)

```cpp
// A5.1 — insert knot u into the curve r times (Boehm). Exact: curve unchanged.
BsplineCurveData insertKnotCurve(const BsplineCurveData& c, double u, int r = 1);

// A5.4 — refine by inserting an entire sorted new-knot vector at once (Oslo-class).
BsplineCurveData refineKnotCurve(const BsplineCurveData& c,
                                 std::span<const double> newKnots);

// A5.8 — remove knot u up to `num` times within tol. Reports how many succeeded.
struct KnotRemovalResult { int removed = 0; double maxError = 0.0; BsplineCurveData curve; };
KnotRemovalResult removeKnotCurve(const BsplineCurveData& c, double u, int num, double tol);

// A5.9 — raise degree by t. Exact: curve unchanged.
BsplineCurveData elevateDegreeCurve(const BsplineCurveData& c, int t);

// A5.11 — reduce degree by 1; exact when reducible, else honest bounded error.
struct DegreeReduceResult { bool ok = false; double maxError = 0.0; BsplineCurveData curve; };
DegreeReduceResult reduceDegreeCurve(const BsplineCurveData& c, double tol);

// Split at parameter u (insert to multiplicity = degree, partition). Pieces reconstruct c.
struct CurveSplit { BsplineCurveData left, right; };
CurveSplit splitCurve(const BsplineCurveData& c, double u);

// A5.6 — decompose into Bézier segments (full-multiplicity knot insertion).
std::vector<BsplineCurveData> decomposeCurveToBezier(const BsplineCurveData& c);

// Affine reparametrization of the knot domain to [a,b]; poles/weights unchanged.
BsplineCurveData reparamCurve(const BsplineCurveData& c, double a, double b);
```

## API surface (surfaces — tensor-product, direction-selected)

Each surface op applies the corresponding curve op along every row (V-dir) or column (U-dir)
of the control net, reusing the curve core so the two never diverge:

```cpp
BsplineSurfaceData  insertKnotSurface (const BsplineSurfaceData& s, ParamDir d, double val, int r = 1);
BsplineSurfaceData  refineKnotSurface (const BsplineSurfaceData& s, ParamDir d, std::span<const double> newKnots);
KnotRemovalResultS  removeKnotSurface (const BsplineSurfaceData& s, ParamDir d, double val, int num, double tol);
BsplineSurfaceData  elevateDegreeSurface(const BsplineSurfaceData& s, ParamDir d, int t);
DegreeReduceResultS reduceDegreeSurface(const BsplineSurfaceData& s, ParamDir d, double tol);
struct SurfaceSplit { BsplineSurfaceData low, high; };
SurfaceSplit        splitSurface(const BsplineSurfaceData& s, ParamDir d, double val);
```

## Oracle strategy (why this layer is uniquely airtight)

| Op | Exact invariant (HOST, no OCCT) |
|---|---|
| insert / refine | `C'(t)==C(t)` (and `S'==S`) for a dense `t` sample, to ~1e-12 |
| remove | round-trip: `insert(u)` then `remove(u)` == original net (byte/eps); reported `maxError ≤ tol` |
| elevate | `C'(t)==C(t)`; degree raised by exactly `t`; knot multiplicities raised correctly |
| reduce | on a curve that IS reducible (built by elevating a lower-degree curve), `reduce` recovers it exactly; else `maxError` is the honest bound |
| split | `left`+`right` re-evaluate to `C` on their sub-domains; C⁰ join at `u` |
| decompose | each Bézier segment re-evaluates to `C` on its span; segment count == distinct interior spans |
| reparam | `C'(map(t))==C(t)`; knot domain becomes `[a,b]` |

Rational cases run the same invariants on `nurbsCurvePoint`/`nurbsSurfacePoint`. SIM parity
diffs the resulting `(poles,weights,knots,degree)` and sampled points against OCCT
`BSplCLib`/`Geom_BSpline*` for insert/elevate/remove/segment.

## Complexity & structure

*The NURBS Book* Ch. 5 algorithms are index-dense; per the cognitive-complexity policy the
compilers/parsers target (25–35) applies to this math. Each algorithm is one focused
function with the book's variable names in comments (`A5.1` etc.); the homogeneous lift and
the "apply curve-op per row/col" surface driver are shared helpers, not copy-paste, to keep
each function within target and the curve/rational/surface paths single-sourced.

## Risks

- **Knot-removal & degree-reduction are the only non-exact ops** — they carry tolerances.
  Mitigation: the tolerance is the *reported* error bound, and the pass criterion is the
  round-trip identity on genuinely-reducible inputs (exact), never a widened compare. An
  irreducible input returns `ok=false` / `removed<num` honestly.
- **Floating-point at high degree / near-coincident knots.** Mitigation: dense-sample
  invariants at fixed 1e-12-class tolerances chosen from the evaluated-magnitude scale;
  SIM parity against OCCT catches any convention mismatch.

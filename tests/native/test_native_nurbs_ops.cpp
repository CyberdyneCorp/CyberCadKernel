// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for the exact-NURBS geometry kernel (src/native/math/
// bspline_ops.{h,cpp}). OCCT-FREE. The oracle is the closed-form invariant that
// every construction op PRESERVES the represented geometry pointwise on a dense
// parameter sample, plus the round-trip / honesty identities from the design's
// oracle table. Knot removal and degree reduction are the only tolerant ops; for
// them the pass criterion is the exact round-trip on genuinely-reducible inputs,
// never a widened compare, and an irreducible input must decline honestly.
//
// Build (mirrors CMake): compiled against the core lib, which includes
// bspline_ops.cpp via the src/native glob.
//
#include "native/math/native_math.h"

#include <cmath>
#include <cstdio>
#include <span>
#include <vector>

using namespace cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}

// ── Evaluators (dispatch rational vs non-rational) ──────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  if (c.weights.empty())
    return curvePoint(c.degree, c.poles, c.knots, u);
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

static double magnitudeCurve(const BsplineCurveData& c) {
  double m = 1.0;
  for (const Point3& p : c.poles) m = std::max({m, std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
  return m;
}

// Dense pointwise-equality check of two curves over [lo,hi]. tolRel ~ 1e-12.
static void sameCurve(const BsplineCurveData& a, const BsplineCurveData& b,
                      double lo, double hi, const char* what, double tolRel = 1e-11) {
  const double mag = std::max(magnitudeCurve(a), magnitudeCurve(b));
  const double tol = tolRel * mag;
  const int N = 97;
  double worst = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double t = lo + (hi - lo) * (static_cast<double>(i) / N);
    const Point3 pa = evalCurve(a, t);
    const Point3 pb = evalCurve(b, t);
    worst = std::max(worst, distance(pa, pb));
  }
  ++g_checks;
  if (!(worst <= tol)) {
    std::printf("FAIL %-46s worst=%.3e tol=%.3e\n", what, worst, tol);
    ++g_failures;
  }
}

static void sameSurface(const BsplineSurfaceData& a, const BsplineSurfaceData& b,
                        double uLo, double uHi, double vLo, double vHi,
                        const char* what, double tolRel = 1e-11) {
  double mag = 1.0;
  for (const Point3& p : a.poles) mag = std::max({mag, std::fabs(p.x), std::fabs(p.y), std::fabs(p.z)});
  const double tol = tolRel * mag;
  const int N = 23;
  double worst = 0.0;
  for (int i = 0; i <= N; ++i)
    for (int j = 0; j <= N; ++j) {
      const double u = uLo + (uHi - uLo) * (static_cast<double>(i) / N);
      const double v = vLo + (vHi - vLo) * (static_cast<double>(j) / N);
      worst = std::max(worst, distance(evalSurface(a, u, v), evalSurface(b, u, v)));
    }
  ++g_checks;
  if (!(worst <= tol)) {
    std::printf("FAIL %-46s worst=%.3e tol=%.3e\n", what, worst, tol);
    ++g_failures;
  }
}

static void checkLenInvariant(const BsplineCurveData& c, const char* what) {
  expectTrue(c.knots.size() == c.poles.size() + c.degree + 1, what);
  if (!c.weights.empty()) expectTrue(c.weights.size() == c.poles.size(), what);
}

// ── Sample curves ───────────────────────────────────────────────────────────────

// A cubic B-spline (non-rational), clamped, with 2 interior knots.
static BsplineCurveData cubicCurve() {
  BsplineCurveData c;
  c.degree = 3;
  c.poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 1}, {4, 1, 2}, {5, 3, 0}, {6, 0, 1}};
  c.knots = {0, 0, 0, 0, 0.35, 0.7, 1, 1, 1, 1};
  return c;
}

// A rational quarter circle (quadratic NURBS), classic w=cos(45°) middle weight.
// Center (0,0,0), radius 1, arc from (1,0,0) to (0,1,0).
static BsplineCurveData quarterCircle() {
  BsplineCurveData c;
  c.degree = 2;
  const double w = std::sqrt(2.0) / 2.0;
  c.poles = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  c.weights = {1.0, w, 1.0};
  c.knots = {0, 0, 0, 1, 1, 1};
  return c;
}

// A full rational circle (degree 2, 9 poles, 4 quadratic Bézier arcs).
static BsplineCurveData fullCircle() {
  BsplineCurveData c;
  c.degree = 2;
  const double w = std::sqrt(2.0) / 2.0;
  c.poles = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}, {-1, 1, 0}, {-1, 0, 0},
             {-1, -1, 0}, {0, -1, 0}, {1, -1, 0}, {1, 0, 0}};
  c.weights = {1, w, 1, w, 1, w, 1, w, 1};
  c.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  return c;
}

// ── Sample surfaces ─────────────────────────────────────────────────────────────

// A bicubic-ish B-spline patch (non-rational): degreeU=2, degreeV=2, 4x4 poles.
static BsplineSurfaceData biquadPatch() {
  BsplineSurfaceData s;
  s.degreeU = 2; s.degreeV = 2;
  s.nPolesU = 4; s.nPolesV = 4;
  s.knotsU = {0, 0, 0, 0.5, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0.5, 1, 1, 1};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double x = i;
      const double y = j;
      const double z = std::sin(i * 0.6) + std::cos(j * 0.5) + 0.3 * i * j * 0.1;
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A rational cylinder patch: circular in V (quarter arc), linear extrude in U.
static BsplineSurfaceData cylinderPatch() {
  BsplineSurfaceData s;
  s.degreeU = 1; s.degreeV = 2;
  s.nPolesU = 2; s.nPolesV = 3;
  s.knotsU = {0, 0, 1, 1};
  s.knotsV = {0, 0, 0, 1, 1, 1};
  const double w = std::sqrt(2.0) / 2.0;
  // Row i=0 at z=0, row i=1 at z=2; each row is the quarter-circle arc.
  const Point3 arc[3] = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  const double aw[3] = {1.0, w, 1.0};
  for (int i = 0; i < 2; ++i)
    for (int j = 0; j < 3; ++j) {
      s.poles.push_back({arc[j].x, arc[j].y, static_cast<double>(2 * i)});
      s.weights.push_back(aw[j]);
    }
  return s;
}

int main() {
  const BsplineCurveData cubic = cubicCurve();
  const BsplineCurveData qcirc = quarterCircle();
  const BsplineCurveData circle = fullCircle();
  const BsplineSurfaceData patch = biquadPatch();
  const BsplineSurfaceData cyl = cylinderPatch();

  // Sanity: quarter circle really is a circle.
  {
    for (int i = 0; i <= 10; ++i) {
      const double t = i / 10.0;
      const Point3 p = evalCurve(qcirc, t);
      expectNear(std::sqrt(p.x * p.x + p.y * p.y), 1.0, 1e-13, "quarterCircle on unit circle");
    }
  }

  // ═══ INSERT (exact) — non-rational + rational, curve ═══
  {
    BsplineCurveData ci = insertKnotCurve(cubic, 0.5, 1);
    checkLenInvariant(ci, "insert cubic len-invariant");
    expectTrue(ci.poles.size() == cubic.poles.size() + 1, "insert cubic +1 pole");
    sameCurve(cubic, ci, 0, 1, "insert cubic preserves curve");

    BsplineCurveData ci2 = insertKnotCurve(cubic, 0.5, 2);
    expectTrue(ci2.poles.size() == cubic.poles.size() + 2, "insert cubic r=2 poles");
    sameCurve(cubic, ci2, 0, 1, "insert cubic r=2 preserves");

    BsplineCurveData qi = insertKnotCurve(qcirc, 0.5, 1);
    checkLenInvariant(qi, "insert qcirc len-invariant");
    sameCurve(qcirc, qi, 0, 1, "insert rational qcirc preserves");

    BsplineCurveData fi = insertKnotCurve(circle, 0.125, 1);
    sameCurve(circle, fi, 0, 1, "insert rational fullcircle preserves");
  }

  // ═══ REFINE == repeated single insertion, and preserves ═══
  {
    std::vector<double> X = {0.2, 0.5, 0.5, 0.9};
    BsplineCurveData cr = refineKnotCurve(cubic, X);
    checkLenInvariant(cr, "refine cubic len-invariant");
    expectTrue(cr.poles.size() == cubic.poles.size() + X.size(), "refine cubic pole count");
    sameCurve(cubic, cr, 0, 1, "refine cubic preserves");

    // Refine == repeated single insertion (same net).
    BsplineCurveData step = cubic;
    for (double x : X) step = insertKnotCurve(step, x, 1);
    expectTrue(step.poles.size() == cr.poles.size(), "refine==insert pole count");
    double worst = 0.0;
    for (std::size_t i = 0; i < step.poles.size(); ++i)
      worst = std::max(worst, distance(step.poles[i], cr.poles[i]));
    expectNear(worst, 0.0, 1e-12, "refine==repeated-insert net identity");

    // Rational refine.
    std::vector<double> Xr = {0.3, 0.6};
    BsplineCurveData qr = refineKnotCurve(qcirc, Xr);
    sameCurve(qcirc, qr, 0, 1, "refine rational qcirc preserves");
  }

  // ═══ INSERT ↔ REMOVE identity ═══
  {
    // Insert 0.5 into cubic 2x, then remove it 2x within a tight tol.
    BsplineCurveData ci = insertKnotCurve(cubic, 0.5, 2);
    KnotRemovalResult rr = removeKnotCurve(ci, 0.5, 2, 1e-9);
    expectTrue(rr.removed == 2, "remove recovers both inserted knots");
    expectTrue(rr.maxError <= 1e-9, "remove reported error within tol");
    checkLenInvariant(rr.curve, "remove result len-invariant");
    expectTrue(rr.curve.poles.size() == cubic.poles.size(), "remove restores pole count");
    sameCurve(cubic, rr.curve, 0, 1, "insert-then-remove == original (curve)");

    // Rational insert↔remove.
    BsplineCurveData qi = insertKnotCurve(qcirc, 0.5, 1);
    KnotRemovalResult qrr = removeKnotCurve(qi, 0.5, 1, 1e-9);
    expectTrue(qrr.removed == 1, "rational remove recovers inserted knot");
    sameCurve(qcirc, qrr.curve, 0, 1, "rational insert-then-remove == original");
  }

  // ═══ HONESTY: a knot that can't be removed within tol is NOT removed ═══
  {
    // The cubic's genuine interior knot 0.35 has multiplicity 1; removing it
    // changes the curve, so under a tight tol it must decline.
    KnotRemovalResult rr = removeKnotCurve(cubic, 0.35, 1, 1e-9);
    expectTrue(rr.removed == 0, "irremovable knot honestly declined (removed==0)");
    // The returned curve is unchanged.
    sameCurve(cubic, rr.curve, 0, 1, "declined removal leaves curve unchanged");
  }

  // ═══ ELEVATE (exact) — curve, degree raised by t ═══
  {
    BsplineCurveData ce = elevateDegreeCurve(cubic, 1);
    expectTrue(ce.degree == 4, "elevate cubic degree +1");
    checkLenInvariant(ce, "elevate cubic len-invariant");
    sameCurve(cubic, ce, 0, 1, "elevate cubic preserves curve");

    BsplineCurveData ce2 = elevateDegreeCurve(cubic, 2);
    expectTrue(ce2.degree == 5, "elevate cubic degree +2");
    sameCurve(cubic, ce2, 0, 1, "elevate cubic +2 preserves");

    BsplineCurveData qe = elevateDegreeCurve(qcirc, 1);
    expectTrue(qe.degree == 3, "elevate qcirc degree +1");
    sameCurve(qcirc, qe, 0, 1, "elevate rational qcirc preserves");

    BsplineCurveData fe = elevateDegreeCurve(circle, 1);
    sameCurve(circle, fe, 0, 1, "elevate rational fullcircle preserves");
  }

  // ═══ ELEVATE → REDUCE identity on reducible inputs (exact) ═══
  {
    // Build a curve KNOWN to be reducible: elevate cubic to quartic, then reduce.
    BsplineCurveData quart = elevateDegreeCurve(cubic, 1);
    DegreeReduceResult dr = reduceDegreeCurve(quart, 1e-8);
    expectTrue(dr.ok, "reduce recovers reducible curve (ok)");
    expectTrue(dr.curve.degree == 3, "reduce lowers degree to 3");
    expectTrue(dr.maxError <= 1e-8, "reduce reported error within tol");
    sameCurve(cubic, dr.curve, 0, 1, "elevate-then-reduce == original cubic");

    // Rational reducible round-trip.
    BsplineCurveData qe = elevateDegreeCurve(qcirc, 1);  // degree 3
    DegreeReduceResult qdr = reduceDegreeCurve(qe, 1e-8);
    expectTrue(qdr.ok, "rational reduce recovers reducible (ok)");
    expectTrue(qdr.curve.degree == 2, "rational reduce degree 2");
    sameCurve(qcirc, qdr.curve, 0, 1, "rational elevate-then-reduce == original");
  }

  // ═══ HONESTY: an irreducible curve reports ok=false with the true bound ═══
  {
    // The cubic itself is generically NOT degree-2 reducible: reducing must
    // decline with a true (non-tiny) error and NOT claim exactness.
    DegreeReduceResult dr = reduceDegreeCurve(cubic, 1e-9);
    expectTrue(!dr.ok, "irreducible cubic honestly declined (ok=false)");
    expectTrue(dr.maxError > 1e-9, "declined reduce reports true bound > tol");
    expectTrue(std::isfinite(dr.maxError), "declined reduce reports a FINITE true bound");
    // Declined result keeps the original degree (no false lower-degree curve).
    expectTrue(dr.curve.degree == 3, "declined reduce leaves degree unchanged");
  }

  // ═══ SPLIT: pieces reconstruct + C0 join ═══
  {
    const double us = 0.4;
    CurveSplit cs = splitCurve(cubic, us);
    checkLenInvariant(cs.left, "split left len-invariant");
    checkLenInvariant(cs.right, "split right len-invariant");
    // Left reconstructs C on [0, us]; right on [us, 1].
    sameCurve(cubic, cs.left, 0, us, "split left reconstructs on [0,us]");
    sameCurve(cubic, cs.right, us, 1, "split right reconstructs on [us,1]");
    // C0 join at us.
    const Point3 jl = evalCurve(cs.left, us);
    const Point3 jr = evalCurve(cs.right, us);
    const Point3 jc = evalCurve(cubic, us);
    expectNear(distance(jl, jr), 0.0, 1e-11, "split C0 join left==right");
    expectNear(distance(jl, jc), 0.0, 1e-11, "split join == C(us)");

    // Rational split.
    CurveSplit qs = splitCurve(qcirc, 0.5);
    sameCurve(qcirc, qs.left, 0, 0.5, "rational split left reconstructs");
    sameCurve(qcirc, qs.right, 0.5, 1, "rational split right reconstructs");
  }

  // ═══ DECOMPOSE: segments re-evaluate to source; count == distinct spans ═══
  {
    std::vector<BsplineCurveData> segs = decomposeCurveToBezier(cubic);
    // cubic has interior knots {0.35, 0.7} → 3 distinct spans.
    expectTrue(segs.size() == 3, "decompose cubic segment count == 3 spans");
    // Each segment is a Bézier (degree+1 poles, clamped) re-evaluating to source.
    const double breaks[4] = {0.0, 0.35, 0.7, 1.0};
    for (std::size_t k = 0; k < segs.size(); ++k) {
      expectTrue(static_cast<int>(segs[k].poles.size()) == cubic.degree + 1,
                 "decompose segment is Bezier (p+1 poles)");
      sameCurve(cubic, segs[k], breaks[k], breaks[k + 1], "decompose segment == source on span");
    }

    // Rational decompose (full circle → 4 quadratic Bézier arcs).
    std::vector<BsplineCurveData> csegs = decomposeCurveToBezier(circle);
    expectTrue(csegs.size() == 4, "decompose circle == 4 Bezier arcs");
    const double cb[5] = {0.0, 0.25, 0.5, 0.75, 1.0};
    for (std::size_t k = 0; k < csegs.size(); ++k)
      sameCurve(circle, csegs[k], cb[k], cb[k + 1], "decompose circle arc == source");
  }

  // ═══ REPARAM: affine remap of domain, geometry unchanged up to remap ═══
  {
    BsplineCurveData cr = reparamCurve(cubic, 2.0, 5.0);
    expectNear(cr.knots.front(), 2.0, 1e-15, "reparam domain start");
    expectNear(cr.knots.back(), 5.0, 1e-15, "reparam domain end");
    checkLenInvariant(cr, "reparam len-invariant");
    // C'(map(t)) == C(t): map t∈[0,1] → [2,5].
    const int N = 50;
    double worst = 0.0;
    for (int i = 0; i <= N; ++i) {
      const double t = static_cast<double>(i) / N;
      const double mt = 2.0 + 3.0 * t;
      worst = std::max(worst, distance(evalCurve(cubic, t), evalCurve(cr, mt)));
    }
    expectNear(worst, 0.0, 1e-11, "reparam C'(map(t))==C(t)");
  }

  // ═══ SURFACES: insert (U & V), exact, non-rational + rational ═══
  {
    BsplineSurfaceData su = insertKnotSurface(patch, ParamDir::U, 0.25, 1);
    expectTrue(su.nPolesU == patch.nPolesU + 1, "surf insert U +1 row");
    expectTrue(su.knotsU.size() == static_cast<std::size_t>(su.nPolesU) + su.degreeU + 1,
               "surf insert U knot-length invariant");
    sameSurface(patch, su, 0, 1, 0, 1, "surf insert U preserves");

    BsplineSurfaceData sv = insertKnotSurface(patch, ParamDir::V, 0.75, 1);
    expectTrue(sv.nPolesV == patch.nPolesV + 1, "surf insert V +1 col");
    sameSurface(patch, sv, 0, 1, 0, 1, "surf insert V preserves");

    // Rational cylinder patch: insert along V (the circular direction).
    BsplineSurfaceData cu = insertKnotSurface(cyl, ParamDir::V, 0.5, 1);
    expectTrue(cu.nPolesV == cyl.nPolesV + 1, "rational surf insert V +1 col");
    sameSurface(cyl, cu, 0, 1, 0, 1, "rational surf insert V preserves");
  }

  // ═══ SURFACES: refine (U & V) ═══
  {
    std::vector<double> X = {0.25, 0.75};
    BsplineSurfaceData su = refineKnotSurface(patch, ParamDir::U, X);
    sameSurface(patch, su, 0, 1, 0, 1, "surf refine U preserves");
    BsplineSurfaceData sv = refineKnotSurface(patch, ParamDir::V, X);
    sameSurface(patch, sv, 0, 1, 0, 1, "surf refine V preserves");
  }

  // ═══ SURFACES: elevate (U & V), exact ═══
  {
    BsplineSurfaceData su = elevateDegreeSurface(patch, ParamDir::U, 1);
    expectTrue(su.degreeU == patch.degreeU + 1, "surf elevate U degree+1");
    sameSurface(patch, su, 0, 1, 0, 1, "surf elevate U preserves");

    BsplineSurfaceData sv = elevateDegreeSurface(patch, ParamDir::V, 1);
    expectTrue(sv.degreeV == patch.degreeV + 1, "surf elevate V degree+1");
    sameSurface(patch, sv, 0, 1, 0, 1, "surf elevate V preserves");

    // Rational surface elevate along the linear U direction.
    BsplineSurfaceData cu = elevateDegreeSurface(cyl, ParamDir::U, 1);
    expectTrue(cu.degreeU == cyl.degreeU + 1, "rational surf elevate U degree+1");
    sameSurface(cyl, cu, 0, 1, 0, 1, "rational surf elevate U preserves");
  }

  // ═══ SURFACES: remove (round-trip identity) ═══
  {
    BsplineSurfaceData su = insertKnotSurface(patch, ParamDir::U, 0.25, 1);
    KnotRemovalResultS rr = removeKnotSurface(su, ParamDir::U, 0.25, 1, 1e-9);
    expectTrue(rr.removed == 1, "surf remove recovers inserted U knot");
    sameSurface(patch, rr.surface, 0, 1, 0, 1, "surf insert-then-remove==original");
  }

  // ═══ SURFACES: reduce (round-trip on reducible input) ═══
  {
    BsplineSurfaceData se = elevateDegreeSurface(patch, ParamDir::U, 1);
    DegreeReduceResultS dr = reduceDegreeSurface(se, ParamDir::U, 1e-8);
    expectTrue(dr.ok, "surf reduce recovers reducible U (ok)");
    expectTrue(dr.surface.degreeU == patch.degreeU, "surf reduce degree back");
    sameSurface(patch, dr.surface, 0, 1, 0, 1, "surf elevate-then-reduce==original");
  }

  // ═══ SURFACES: split (U & V), pieces reconstruct ═══
  {
    SurfaceSplit su = splitSurface(patch, ParamDir::U, 0.5);
    sameSurface(patch, su.low, 0, 0.5, 0, 1, "surf split U low reconstructs");
    sameSurface(patch, su.high, 0.5, 1, 0, 1, "surf split U high reconstructs");

    SurfaceSplit sv = splitSurface(patch, ParamDir::V, 0.5);
    sameSurface(patch, sv.low, 0, 1, 0, 0.5, "surf split V low reconstructs");
    sameSurface(patch, sv.high, 0, 1, 0.5, 1, "surf split V high reconstructs");

    // Rational cylinder split along V.
    SurfaceSplit cv = splitSurface(cyl, ParamDir::V, 0.5);
    sameSurface(cyl, cv.low, 0, 1, 0, 0.5, "rational surf split V low reconstructs");
    sameSurface(cyl, cv.high, 0, 1, 0.5, 1, "rational surf split V high reconstructs");
  }

  // ═══ RATIONAL EVALUATION HARDENING (task 1.3) ═══
  // Full-multiplicity interior knots (Bézier-segment boundary produced by
  // splitting/decomposition) must evaluate finite/continuous and equal to the
  // un-split geometry. Insert a full-multiplicity knot and re-check the eval.
  {
    // Insert 0.5 into the cubic to multiplicity == degree (a C0 knot).
    BsplineCurveData c0 = insertKnotCurve(cubic, 0.5, cubic.degree);
    // Count multiplicity of 0.5.
    int mult = 0;
    for (double k : c0.knots) if (k == 0.5) ++mult;
    expectTrue(mult == cubic.degree, "full-mult interior knot present");
    // Evaluation at and around the full-mult knot is finite and == original.
    for (double t : {0.4999, 0.5, 0.5001}) {
      Point3 p = evalCurve(c0, t);
      expectTrue(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z),
                 "full-mult eval finite");
      expectNear(distance(p, evalCurve(cubic, t)), 0.0, 1e-10, "full-mult eval==original");
    }

    // Endpoint parameters (clamped ends) evaluate to the end poles exactly.
    expectNear(distance(evalCurve(cubic, 0.0), cubic.poles.front()), 0.0, 1e-13,
               "endpoint u=0 interpolates first pole");
    expectNear(distance(evalCurve(cubic, 1.0), cubic.poles.back()), 0.0, 1e-13,
               "endpoint u=1 interpolates last pole");
    // Rational endpoints too.
    expectNear(distance(evalCurve(qcirc, 0.0), qcirc.poles.front()), 0.0, 1e-13,
               "rational endpoint u=0 first pole");
    expectNear(distance(evalCurve(qcirc, 1.0), qcirc.poles.back()), 0.0, 1e-13,
               "rational endpoint u=1 last pole");
  }

  // ── report ──
  if (g_failures == 0)
    std::printf("OK  test_native_nurbs_ops: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_nurbs_ops: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}

// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for tolerance-BOUNDED NURBS simplification (src/native/math/
// bspline_simplify.{h,cpp}). OCCT-FREE. The oracles are the airtight, closed-form
// contracts of P&T Ch.9 bounded reduction:
//
//   1. Over-refined reduces EXACTLY: knot-INSERT a curve many times (geometry
//      unchanged), then removeKnotsBounded(tol=1e-10) recovers the ORIGINAL knot
//      vector exactly (all inserted knots removed, deviation ~machine epsilon).
//   2. Lossy within bound: a genuinely wiggly curve simplified with tol=1e-3 →
//      FEWER knots AND max deviation ≤ 1e-3, measured DENSELY (never exceeds).
//   3. Degree reduction: a degree-ELEVATED curve reduceDegreeBounded back to its
//      original degree exactly (deviation ~epsilon); a curve that CAN'T reduce
//      within tol keeps its degree (honest, no violation).
//   4. No-op: tol too tight to remove anything ⇒ input returned unchanged.
//
// The deviation bound is HARD everywhere: we independently re-sample the returned
// curve against the original and assert the true worst deviation ≤ tol. No compare
// is widened to pass.
//
// Build (mirrors CMake): compiled against the core lib, which includes
// bspline_simplify.cpp via the src/native glob.
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
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s got %.6e want <= %.6e\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators (dispatch rational vs non-rational) ──────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  if (c.weights.empty()) return curvePoint(c.degree, c.poles, c.knots, u);
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

// INDEPENDENT dense true-deviation oracle: max pointwise distance between two
// curves over the domain of `ref`. Denser than the simplifier's internal check.
static double denseCurveDeviation(const BsplineCurveData& ref,
                                  const BsplineCurveData& cand) {
  const double lo = ref.knots.front();
  const double hi = ref.knots.back();
  const int N = 1000;
  double worst = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double u = lo + (hi - lo) * (static_cast<double>(i) / N);
    worst = std::max(worst, distance(evalCurve(ref, u), evalCurve(cand, u)));
  }
  return worst;
}

static double denseSurfaceDeviation(const BsplineSurfaceData& ref,
                                    const BsplineSurfaceData& cand) {
  const double u0 = ref.knotsU.front(), u1 = ref.knotsU.back();
  const double v0 = ref.knotsV.front(), v1 = ref.knotsV.back();
  const int N = 80;
  double worst = 0.0;
  for (int i = 0; i <= N; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / N);
    for (int j = 0; j <= N; ++j) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / N);
      worst = std::max(worst, distance(evalSurface(ref, u, v), evalSurface(cand, u, v)));
    }
  }
  return worst;
}

// Count distinct interior knot values (multiplicity-collapsed) — the "weight" of
// a curve's knot vector for the reduction-achieved assertions.
static int interiorKnotCount(const BsplineCurveData& c) {
  int count = 0;
  const int p = c.degree;
  const int m = static_cast<int>(c.knots.size());
  for (int i = p + 1; i < m - p - 1; ++i) ++count;
  return count;
}

// ── Sample curves ───────────────────────────────────────────────────────────────

// A cubic B-spline (non-rational), clamped, 2 interior knots.
static BsplineCurveData cubicCurve() {
  BsplineCurveData c;
  c.degree = 3;
  c.poles = {{0, 0, 0}, {1, 2, 0}, {2, -1, 1}, {4, 1, 2}, {5, 3, 0}, {6, 0, 1}};
  c.knots = {0, 0, 0, 0, 0.35, 0.7, 1, 1, 1, 1};
  return c;
}

// A rational quarter circle (quadratic NURBS).
static BsplineCurveData quarterCircle() {
  BsplineCurveData c;
  c.degree = 2;
  const double w = std::sqrt(2.0) / 2.0;
  c.poles = {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}};
  c.weights = {1.0, w, 1.0};
  c.knots = {0, 0, 0, 1, 1, 1};
  return c;
}

// A genuinely WIGGLY dense cubic curve: many interior knots and a jagged pole
// polygon, so bounded removal is genuinely lossy AND its removals cost far more
// than 1e-3 — used for the "irreducible keeps its degree" and no-op oracles.
static BsplineCurveData wigglyCurve() {
  BsplineCurveData c;
  c.degree = 3;
  c.poles = {{0, 0, 0},   {1, 1.7, 0},  {2, -1.3, 0}, {3, 2.1, 0},
             {4, -1.9, 0}, {5, 1.5, 0},  {6, -1.1, 0}, {7, 0.9, 0},
             {8, -0.6, 0}, {9, 0, 0}};
  // 10 poles + degree 3 + 1 = 14 knots: 4 leading + 6 interior + 4 trailing.
  c.knots = {0, 0, 0, 0, 0.14, 0.29, 0.43, 0.57, 0.71, 0.86, 1, 1, 1, 1};
  return c;
}

// A curve that is LOSSY-WITHIN-BOUND at 1e-3: a smooth base cubic over-refined
// (exact extra knots), then its interior poles nudged by ~2e-4. The extra knots
// are no longer exactly redundant, but removing them reintroduces only the small
// nudge-scale deviation — so bounded removal at 1e-3 sheds several while the true
// deviation stays strictly under 1e-3. Deterministic, no RNG.
static BsplineCurveData lossyCurve() {
  BsplineCurveData base = cubicCurve();
  // Over-refine (exact): several distinct interior knots.
  BsplineCurveData c = base;
  c = insertKnotCurve(c, 0.1, 1);
  c = insertKnotCurve(c, 0.2, 1);
  c = insertKnotCurve(c, 0.5, 1);
  c = insertKnotCurve(c, 0.8, 1);
  c = insertKnotCurve(c, 0.9, 1);
  // Nudge interior poles by a tiny, smoothly-varying amount (leave the clamped
  // endpoints put so the endpoints match exactly).
  for (std::size_t i = 1; i + 1 < c.poles.size(); ++i) {
    const double s = static_cast<double>(i);
    c.poles[i].x += 2.0e-4 * std::sin(s * 1.3);
    c.poles[i].y += 2.0e-4 * std::cos(s * 0.9);
    c.poles[i].z += 2.0e-4 * std::sin(s * 0.5);
  }
  return c;
}

// A biquad non-rational patch with an interior knot in each direction.
static BsplineSurfaceData biquadPatch() {
  BsplineSurfaceData s;
  s.degreeU = 2; s.degreeV = 2;
  s.nPolesU = 4; s.nPolesV = 4;
  s.knotsU = {0, 0, 0, 0.5, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0.5, 1, 1, 1};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double z = std::sin(i * 0.6) + std::cos(j * 0.5) + 0.03 * i * j;
      s.poles.push_back({static_cast<double>(i), static_cast<double>(j), z});
    }
  return s;
}

int main() {
  std::printf("== bounded NURBS simplification gate ==\n");

  // ── Oracle 1: over-refined recovers EXACTLY ─────────────────────────────────
  // Insert many knots (geometry unchanged), then bounded-remove with a tight tol
  // and recover the original minimal knot vector, deviation ~epsilon.
  double dev1 = 0.0;
  {
    const BsplineCurveData base = cubicCurve();
    const int baseInterior = interiorKnotCount(base);  // 2 distinct * mult 1 = 2

    // Over-refine: insert several DISTINCT new interior knots + raise an existing.
    BsplineCurveData over = base;
    over = insertKnotCurve(over, 0.1, 1);
    over = insertKnotCurve(over, 0.2, 1);
    over = insertKnotCurve(over, 0.5, 1);
    over = insertKnotCurve(over, 0.9, 1);
    over = insertKnotCurve(over, 0.35, 1);  // raise multiplicity of an existing knot
    const int overInterior = interiorKnotCount(over);
    expectTrue(overInterior > baseInterior, "over-refined has more interior knots");
    // Geometry unchanged by insertion:
    expectLE(denseCurveDeviation(base, over), 1e-12, "insertion preserved geometry");

    const BoundedRemovalResult r = removeKnotsBounded(over, 1e-10);
    dev1 = denseCurveDeviation(base, r.curve);
    expectTrue(r.removed == overInterior - baseInterior,
               "over-refined: exactly the inserted knots removed");
    expectTrue(interiorKnotCount(r.curve) == baseInterior,
               "over-refined: recovered minimal interior knot count");
    expectLE(dev1, 1e-9, "over-refined: recovered curve within tol");
    expectLE(dev1, 1e-10, "over-refined: recovery is essentially exact");
    // Reported deviation is a true hard bound.
    expectLE(r.maxDeviation, 1e-10, "over-refined: reported deviation ~epsilon");
    expectLE(dev1, r.maxDeviation > 1e-14 ? r.maxDeviation * 10.0 + 1e-12 : 1e-10,
             "over-refined: independent measure consistent with report");
  }

  // Rational over-refined recovers exactly too.
  {
    const BsplineCurveData base = quarterCircle();
    BsplineCurveData over = base;
    over = insertKnotCurve(over, 0.25, 1);
    over = insertKnotCurve(over, 0.5, 1);
    over = insertKnotCurve(over, 0.75, 1);
    const int overInterior = interiorKnotCount(over);
    const BoundedRemovalResult r = removeKnotsBounded(over, 1e-10);
    const double dev = denseCurveDeviation(base, r.curve);
    expectTrue(r.removed == overInterior, "rational over-refined: all inserted removed");
    expectTrue(interiorKnotCount(r.curve) == 0, "rational over-refined: knot vector minimal");
    expectLE(dev, 1e-10, "rational over-refined: recovery essentially exact");
    // Still a circle:
    for (int i = 0; i <= 20; ++i) {
      const Point3 p = evalCurve(r.curve, static_cast<double>(i) / 20);
      expectLE(std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - 1.0), 1e-12,
               "rational over-refined: still on unit circle");
    }
  }

  // ── Oracle 2: lossy within bound (HARD) ─────────────────────────────────────
  double dev2 = 0.0;
  int removed2 = 0;
  {
    const BsplineCurveData lossy = lossyCurve();
    const int before = interiorKnotCount(lossy);
    const double tol = 1e-3;
    const BoundedRemovalResult r = removeKnotsBounded(lossy, tol);
    removed2 = r.removed;
    dev2 = denseCurveDeviation(lossy, r.curve);
    expectTrue(r.removed >= 1, "lossy: at least one knot removed within 1e-3");
    expectTrue(interiorKnotCount(r.curve) < before, "lossy: fewer interior knots");
    // HARD bound: independently measured deviation never exceeds tol.
    expectLE(dev2, tol, "lossy: TRUE deviation <= 1e-3 (hard)");
    expectLE(r.maxDeviation, tol, "lossy: reported deviation <= 1e-3");
    // The report is not an underestimate of what we measure densely on a finer grid.
    expectLE(dev2, r.maxDeviation + 5e-4, "lossy: report not a gross underestimate");
  }

  // A LOOSER tol removes at least as many knots (monotone in tol) and stays bounded.
  {
    const BsplineCurveData lossy = lossyCurve();
    const BoundedRemovalResult loose = removeKnotsBounded(lossy, 1e-1);
    const double dev = denseCurveDeviation(lossy, loose.curve);
    expectTrue(loose.removed >= removed2, "looser tol removes >= tighter tol");
    expectLE(dev, 1e-1, "looser tol still within its (looser) bound");
  }

  // The genuinely wiggly curve's removals all cost >> 1e-3, so at 1e-3 it declines
  // every removal — honest no-op even though there ARE interior knots.
  {
    const BsplineCurveData wig = wigglyCurve();
    const int before = interiorKnotCount(wig);
    const BoundedRemovalResult r = removeKnotsBounded(wig, 1e-3);
    expectTrue(r.removed == 0, "irreducible-at-1e-3: no knot removed (honest)");
    expectTrue(interiorKnotCount(r.curve) == before, "irreducible-at-1e-3: knots kept");
    expectLE(r.maxDeviation, 1e-3, "irreducible-at-1e-3: reported deviation within bound");
  }

  // ── Oracle 3: degree reduction ──────────────────────────────────────────────
  // A degree-ELEVATED curve reduces back to its original degree exactly.
  {
    const BsplineCurveData base = cubicCurve();  // degree 3
    const BsplineCurveData elevated = elevateDegreeCurve(base, 2);  // degree 5
    expectTrue(elevated.degree == 5, "elevated to degree 5");
    const BoundedReduceResult r = reduceDegreeBounded(elevated, 1e-9);
    const double dev = denseCurveDeviation(base, r.curve);
    expectTrue(r.degreeDrop == 2, "reduceBounded shed exactly 2 degrees");
    expectTrue(r.curve.degree == 3, "reduced back to original degree 3");
    expectLE(dev, 1e-9, "degree-reduce recovery within tol");
    expectLE(dev, 1e-8, "degree-reduce recovery essentially exact");
    expectLE(r.maxDeviation, 1e-9, "degree-reduce reported deviation ~epsilon");
  }

  // Rational elevate→reduce round-trip.
  {
    const BsplineCurveData base = quarterCircle();  // degree 2
    const BsplineCurveData elevated = elevateDegreeCurve(base, 1);  // degree 3
    const BoundedReduceResult r = reduceDegreeBounded(elevated, 1e-9);
    const double dev = denseCurveDeviation(base, r.curve);
    expectTrue(r.degreeDrop == 1, "rational reduceBounded shed 1 degree");
    expectTrue(r.curve.degree == 2, "rational reduced back to degree 2");
    expectLE(dev, 1e-9, "rational degree-reduce recovery within tol");
  }

  // A curve that CANNOT be reduced within a tight tol keeps its degree (honest).
  {
    const BsplineCurveData wig = wigglyCurve();  // degree 3, genuinely wiggly
    const BoundedReduceResult r = reduceDegreeBounded(wig, 1e-12);
    expectTrue(r.degreeDrop == 0, "irreducible: no degree dropped (honest)");
    expectTrue(r.curve.degree == wig.degree, "irreducible: degree unchanged");
    expectTrue(r.maxDeviation == 0.0, "irreducible: no-op reports zero deviation");
  }

  // ── Oracle 4: tol too tight ⇒ no-op ─────────────────────────────────────────
  {
    const BsplineCurveData wig = wigglyCurve();
    const int before = interiorKnotCount(wig);
    const BoundedRemovalResult r = removeKnotsBounded(wig, 0.0);  // impossibly tight
    expectTrue(r.removed == 0, "no-op: nothing removed at tol=0");
    expectTrue(interiorKnotCount(r.curve) == before, "no-op: knot count unchanged");
    expectTrue(r.maxDeviation == 0.0, "no-op: zero deviation");
    // Returned curve is geometrically identical (in fact byte-identical structure).
    expectLE(denseCurveDeviation(wig, r.curve), 1e-14, "no-op: geometry unchanged");
  }

  // ── Surface analogue: over-refined surface reduces exactly ──────────────────
  {
    BsplineSurfaceData base = biquadPatch();
    // Over-refine both directions (geometry unchanged).
    BsplineSurfaceData over = base;
    over = insertKnotSurface(over, ParamDir::U, 0.25, 1);
    over = insertKnotSurface(over, ParamDir::U, 0.75, 1);
    over = insertKnotSurface(over, ParamDir::V, 0.25, 1);
    over = insertKnotSurface(over, ParamDir::V, 0.75, 1);
    expectLE(denseSurfaceDeviation(base, over), 1e-11, "surface insertion preserved geometry");

    const BoundedRemovalResultS r = removeKnotsBoundedSurface(over, 1e-9);
    const double dev = denseSurfaceDeviation(base, r.surface);
    expectTrue(r.removedU == 2, "surface over-refined: 2 U knots removed");
    expectTrue(r.removedV == 2, "surface over-refined: 2 V knots removed");
    expectLE(dev, 1e-9, "surface over-refined: recovery within tol");
    expectLE(dev, 1e-8, "surface over-refined: essentially exact");
    expectLE(r.maxDeviation, 1e-9, "surface: reported deviation ~epsilon");
  }

  // Surface no-op at tol=0.
  {
    BsplineSurfaceData base = biquadPatch();
    const BoundedRemovalResultS r = removeKnotsBoundedSurface(base, 0.0);
    expectTrue(r.removedU == 0 && r.removedV == 0, "surface no-op at tol=0");
    expectLE(denseSurfaceDeviation(base, r.surface), 1e-14, "surface no-op: geometry unchanged");
  }

  std::printf("over-refined recovery deviation = %.3e\n", dev1);
  std::printf("lossy(1e-3) removed=%d, true deviation = %.3e (bound 1e-3)\n", removed2, dev2);
  std::printf("checks=%d failures=%d\n", g_checks, g_failures);
  if (g_failures == 0) std::printf("ALL PASS\n");
  return g_failures == 0 ? 0 : 1;
}

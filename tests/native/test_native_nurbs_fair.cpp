// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 6/7 — minimal-energy fairing /
// smoothing (src/native/math/bspline_fair.{h,cpp}). OCCT-FREE. The oracles are
// airtight and closed-form:
//
//   1. ALREADY-FAIR NO-OP — a curve / surface whose control net is affine (a
//      straight line / a flat plane grid) has ZERO bending energy; fairing leaves
//      the geometry within tol (deviation 0) and energy stays ~0 (nothing to remove).
//   2. NOISE REMOVED — a smooth analytic curve / surface with a small HIGH-FREQUENCY
//      perturbation added to its interior control points is faired: the bending
//      energy drops SUBSTANTIALLY, the result is CLOSER to the ORIGINAL smooth shape
//      than the noisy input was (fairing recovers the underlying smoothness), and the
//      endpoints / boundary are preserved to ~1e-12.
//   3. DEVIATION BOUND HONORED — the max deviation of the faired shape from the INPUT
//      never exceeds tol (measured on a dense sample, asserted <= tol exactly).
//   4. OVER-TIGHT DECLINE — a tol far below the machine-visible smoothing step cannot
//      reduce energy without breaching the bound → HONEST DECLINE (ok=false, energy
//      unchanged, shape returned unchanged).
//
// The routines are numsci-gated (they call numerics::lin_solve), so the whole gate
// is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). With the guard OFF this
// compiles to a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_fair.h"
#include "native/math/bspline_ops.h"

#include <algorithm>
#include <cmath>
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
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.15g want %.15g (|d|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectLT(double a, double b, const char* what) {
  ++g_checks;
  if (!(a < b)) {
    std::printf("FAIL %-46s %.6g < %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}
static Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
}

// Max ‖a(t) − b(t)‖ of two curves sharing knots/degree over the parametric domain.
static double curveMaxDiff(const BsplineCurveData& a, const BsplineCurveData& b, int n) {
  const double t0 = a.knots[a.degree];
  const double t1 = a.knots[a.poles.size()];
  double m = 0.0;
  for (int s = 0; s <= n; ++s) {
    const double t = t0 + (t1 - t0) * (static_cast<double>(s) / n);
    m = std::max(m, distance(evalCurve(a, t), evalCurve(b, t)));
  }
  return m;
}
static double surfaceMaxDiff(const BsplineSurfaceData& a, const BsplineSurfaceData& b, int n) {
  const double u0 = a.knotsU[a.degreeU], u1 = a.knotsU[a.nPolesU];
  const double v0 = a.knotsV[a.degreeV], v1 = a.knotsV[a.nPolesV];
  double m = 0.0;
  for (int i = 0; i <= n; ++i)
    for (int j = 0; j <= n; ++j) {
      const double u = u0 + (u1 - u0) * (static_cast<double>(i) / n);
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / n);
      m = std::max(m, distance(evalSurface(a, u, v), evalSurface(b, u, v)));
    }
  return m;
}

// ── Builders ─────────────────────────────────────────────────────────────────
// Clamped uniform flat knot vector for `nPoles` degree-`p` poles on [0,1].
static std::vector<double> clampedKnots(int nPoles, int p) {
  const int m = nPoles + p + 1;
  std::vector<double> U(m, 0.0);
  const int nInner = nPoles - p - 1;  // number of interior knots
  for (int i = 0; i <= p; ++i) U[i] = 0.0;
  for (int i = m - p - 1; i < m; ++i) U[i] = 1.0;
  for (int i = 1; i <= nInner; ++i)
    U[p + i] = static_cast<double>(i) / (nInner + 1);
  return U;
}

// A degree-3 straight line — an AFFINE control net (zero bending energy).
static BsplineCurveData straightLineCurve(int nPoles) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = clampedKnots(nPoles, 3);
  c.poles.resize(nPoles);
  for (int i = 0; i < nPoles; ++i) {
    const double t = static_cast<double>(i) / (nPoles - 1);
    c.poles[i] = {2.0 * t, 3.0 * t + 1.0, -t};  // points on a straight 3-D line
  }
  return c;
}

// A smooth degree-3 curve: control poles sampled from a gentle parabola-in-space.
static BsplineCurveData smoothCurve(int nPoles) {
  BsplineCurveData c;
  c.degree = 3;
  c.knots = clampedKnots(nPoles, 3);
  c.poles.resize(nPoles);
  for (int i = 0; i < nPoles; ++i) {
    const double t = static_cast<double>(i) / (nPoles - 1);
    c.poles[i] = {t, 0.5 * std::sin(1.2 * t), 0.3 * t * t};  // low-frequency, smooth
  }
  return c;
}

// A flat plane grid — an affine surface net (zero bending energy).
static BsplineSurfaceData flatSurface(int nu, int nv) {
  BsplineSurfaceData s;
  s.degreeU = s.degreeV = 3;
  s.nPolesU = nu; s.nPolesV = nv;
  s.knotsU = clampedKnots(nu, 3);
  s.knotsV = clampedKnots(nv, 3);
  s.poles.resize(static_cast<std::size_t>(nu) * nv);
  for (int i = 0; i < nu; ++i)
    for (int j = 0; j < nv; ++j)
      s.poles[static_cast<std::size_t>(i) * nv + j] = {
          static_cast<double>(i), static_cast<double>(j), 0.0};
  return s;
}

// A smooth bump surface (low-frequency), degree 3 × 3.
static BsplineSurfaceData smoothSurface(int nu, int nv) {
  BsplineSurfaceData s;
  s.degreeU = s.degreeV = 3;
  s.nPolesU = nu; s.nPolesV = nv;
  s.knotsU = clampedKnots(nu, 3);
  s.knotsV = clampedKnots(nv, 3);
  s.poles.resize(static_cast<std::size_t>(nu) * nv);
  for (int i = 0; i < nu; ++i)
    for (int j = 0; j < nv; ++j) {
      const double u = static_cast<double>(i) / (nu - 1);
      const double v = static_cast<double>(j) / (nv - 1);
      const double z = 0.4 * std::sin(1.1 * u) * std::cos(0.9 * v);  // smooth bump
      s.poles[static_cast<std::size_t>(i) * nv + j] = {
          static_cast<double>(i), static_cast<double>(j), z};
    }
  return s;
}

// Deterministic high-frequency perturbation added to INTERIOR poles of a curve.
static BsplineCurveData addCurveNoise(const BsplineCurveData& c, double amp) {
  BsplineCurveData n = c;
  const int np = static_cast<int>(c.poles.size());
  for (int i = 2; i < np - 2; ++i) {  // keep two poles at each end pristine
    const double s = (i % 2 == 0) ? 1.0 : -1.0;  // alternating ⇒ high frequency
    n.poles[i].z += s * amp;
    n.poles[i].y += s * amp * 0.7;
  }
  return n;
}

static BsplineSurfaceData addSurfaceNoise(const BsplineSurfaceData& s, double amp) {
  BsplineSurfaceData n = s;
  const int nu = s.nPolesU, nv = s.nPolesV;
  for (int i = 1; i < nu - 1; ++i)
    for (int j = 1; j < nv - 1; ++j) {
      const double sgn = ((i + j) % 2 == 0) ? 1.0 : -1.0;  // checkerboard ⇒ high freq
      n.poles[static_cast<std::size_t>(i) * nv + j].z += sgn * amp;
    }
  return n;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

static void testAlreadyFairCurveNoOp() {
  BsplineCurveData line = straightLineCurve(8);
  expectNear(curveBendingEnergy(line), 0.0, 1e-20, "line-energy-zero");
  const CurveFairResult r = fairCurve(line, 1e-3, /*keepEnds=*/true);
  expectTrue(r.ok, "line-fair-ok");                       // no-op success
  expectNear(r.energyAfter, 0.0, 1e-18, "line-energy-after-zero");
  expectLE(curveMaxDiff(r.curve, line, 200), 1e-12, "line-unmoved");
}

static void testAlreadyFairSurfaceNoOp() {
  BsplineSurfaceData flat = flatSurface(6, 6);
  expectNear(surfaceBendingEnergy(flat), 0.0, 1e-18, "flat-energy-zero");
  const SurfaceFairResult r = fairSurface(flat, 1e-3, /*keepBoundary=*/true);
  expectTrue(r.ok, "flat-fair-ok");
  expectNear(r.energyAfter, 0.0, 1e-16, "flat-energy-after-zero");
  expectLE(surfaceMaxDiff(r.surface, flat, 40), 1e-12, "flat-unmoved");
}

static void testCurveNoiseRemoved() {
  const BsplineCurveData smooth = smoothCurve(11);
  const double amp = 0.05;
  const BsplineCurveData noisy = addCurveNoise(smooth, amp);

  const double eSmooth = curveBendingEnergy(smooth);
  const double eNoisy = curveBendingEnergy(noisy);
  expectLT(eSmooth, eNoisy, "noise-raises-energy");

  // Deviation of the noisy input from the smooth original — the fairing must land
  // CLOSER to smooth than this, within a tol at the NOISE scale (a much larger tol
  // would let the minimal-energy solution flatten PAST the smooth original's own
  // intrinsic bending, moving away from it — physically correct, but not what this
  // recover-smoothness oracle asserts).
  const double noisyVsSmooth = curveMaxDiff(noisy, smooth, 400);
  const double tol = noisyVsSmooth;

  const CurveFairResult r = fairCurve(noisy, tol, /*keepEnds=*/true);
  expectTrue(r.ok, "curve-noise-fair-ok");

  // Energy dropped substantially (at least halfway back toward the smooth energy).
  expectLT(r.energyAfter, r.energyBefore, "curve-energy-drops");
  expectLE(r.energyAfter, 0.5 * (r.energyBefore + eSmooth), "curve-energy-halved");

  // The faired curve is CLOSER to the smooth original than the noisy input was.
  const double fairedVsSmooth = curveMaxDiff(r.curve, smooth, 400);
  expectLT(fairedVsSmooth, noisyVsSmooth, "curve-closer-to-smooth");

  // Endpoints (and, with keepEnds, the tangent poles) preserved to ~1e-12.
  const int np = static_cast<int>(noisy.poles.size());
  expectLE(distance(r.curve.poles.front(), noisy.poles.front()), 1e-12, "curve-end0-fixed");
  expectLE(distance(r.curve.poles.back(), noisy.poles.back()), 1e-12, "curve-end1-fixed");
  expectLE(distance(r.curve.poles[1], noisy.poles[1]), 1e-12, "curve-tan0-fixed");
  expectLE(distance(r.curve.poles[np - 2], noisy.poles[np - 2]), 1e-12, "curve-tan1-fixed");

  // Deviation bound honored exactly.
  expectLE(r.maxDeviation, tol, "curve-dev-within-tol");
}

static void testSurfaceNoiseRemoved() {
  const BsplineSurfaceData smooth = smoothSurface(7, 7);
  const double amp = 0.04;
  const BsplineSurfaceData noisy = addSurfaceNoise(smooth, amp);

  const double eSmooth = surfaceBendingEnergy(smooth);
  const double eNoisy = surfaceBendingEnergy(noisy);
  expectLT(eSmooth, eNoisy, "surf-noise-raises-energy");

  const double noisyVsSmooth = surfaceMaxDiff(noisy, smooth, 40);
  const double tol = noisyVsSmooth;  // noise-scale (see fairCurve oracle rationale)

  const SurfaceFairResult r = fairSurface(noisy, tol, /*keepBoundary=*/true);
  expectTrue(r.ok, "surf-noise-fair-ok");
  expectLT(r.energyAfter, r.energyBefore, "surf-energy-drops");
  expectLE(r.energyAfter, 0.5 * (r.energyBefore + eSmooth), "surf-energy-halved");

  const double fairedVsSmooth = surfaceMaxDiff(r.surface, smooth, 40);
  expectLT(fairedVsSmooth, noisyVsSmooth, "surf-closer-to-smooth");

  // Boundary control ring preserved to ~1e-12.
  const int nu = noisy.nPolesU, nv = noisy.nPolesV;
  double maxBnd = 0.0;
  for (int i = 0; i < nu; ++i)
    for (int j = 0; j < nv; ++j)
      if (i == 0 || j == 0 || i == nu - 1 || j == nv - 1) {
        const std::size_t k = static_cast<std::size_t>(i) * nv + j;
        maxBnd = std::max(maxBnd, distance(r.surface.poles[k], noisy.poles[k]));
      }
  expectLE(maxBnd, 1e-12, "surf-boundary-fixed");

  expectLE(r.maxDeviation, tol, "surf-dev-within-tol");
}

static void testDeviationBoundHonored() {
  // Across a spread of tolerances (some tight, some loose), the achieved deviation
  // must NEVER exceed the requested tol — the hard bound, asserted exactly.
  const BsplineCurveData noisy = addCurveNoise(smoothCurve(11), 0.05);
  for (double tol : {1e-4, 1e-3, 1e-2, 5e-2, 1e-1}) {
    const CurveFairResult r = fairCurve(noisy, tol, true);
    expectLE(r.maxDeviation, tol, "curve-dev<=tol-sweep");
    if (r.ok) expectLE(r.energyAfter, r.energyBefore + 1e-15, "curve-energy-nonincreasing");
  }
  const BsplineSurfaceData snoisy = addSurfaceNoise(smoothSurface(7, 7), 0.04);
  for (double tol : {1e-4, 1e-3, 1e-2, 5e-2}) {
    const SurfaceFairResult r = fairSurface(snoisy, tol, true);
    expectLE(r.maxDeviation, tol, "surf-dev<=tol-sweep");
    if (r.ok) expectLE(r.energyAfter, r.energyBefore + 1e-15, "surf-energy-nonincreasing");
  }
}

static void testOverTightDecline() {
  // A tol far below the smallest smoothing step cannot reduce energy without
  // breaching the bound → honest decline (unchanged, energyAfter == energyBefore).
  const BsplineCurveData noisy = addCurveNoise(smoothCurve(11), 0.05);
  const double eBefore = curveBendingEnergy(noisy);
  const CurveFairResult r = fairCurve(noisy, 1e-14, true);
  expectTrue(!r.ok, "curve-overtight-declines");
  expectNear(r.energyAfter, eBefore, 0.0, "curve-overtight-energy-unchanged");
  expectLE(curveMaxDiff(r.curve, noisy, 200), 0.0, "curve-overtight-unchanged");
  // maxDeviation reported as 0 (no move made).
  expectNear(r.maxDeviation, 0.0, 0.0, "curve-overtight-dev-zero");

  const BsplineSurfaceData snoisy = addSurfaceNoise(smoothSurface(7, 7), 0.04);
  const double eBeforeS = surfaceBendingEnergy(snoisy);
  const SurfaceFairResult rs = fairSurface(snoisy, 1e-14, true);
  expectTrue(!rs.ok, "surf-overtight-declines");
  expectNear(rs.energyAfter, eBeforeS, 0.0, "surf-overtight-energy-unchanged");
  expectLE(surfaceMaxDiff(rs.surface, snoisy, 40), 0.0, "surf-overtight-unchanged");
}

static void testGuards() {
  // Rational input is a documented residual — declined, unchanged.
  BsplineCurveData rat = smoothCurve(8);
  rat.weights.assign(rat.poles.size(), 1.0);
  const CurveFairResult rr = fairCurve(rat, 1.0, true);
  expectTrue(!rr.ok, "rational-curve-declines");

  // Too-small net → decline (degree 3 needs ≥ its own poles; a 2-pole net can't fair).
  BsplineCurveData tiny;
  tiny.degree = 1;
  tiny.poles = {{0, 0, 0}, {1, 0, 0}};
  tiny.knots = {0, 0, 1, 1};
  const CurveFairResult rt = fairCurve(tiny, 1.0, true);
  expectTrue(!rt.ok, "tiny-curve-declines");

  // Non-positive tol → decline.
  const CurveFairResult rn = fairCurve(addCurveNoise(smoothCurve(11), 0.05), 0.0, true);
  expectTrue(!rn.ok, "nonpos-tol-declines");
}

#endif  // CYBERCAD_HAS_NUMSCI

int main() {
#ifdef CYBERCAD_HAS_NUMSCI
  testAlreadyFairCurveNoOp();
  testAlreadyFairSurfaceNoOp();
  testCurveNoiseRemoved();
  testSurfaceNoiseRemoved();
  testDeviationBoundHonored();
  testOverTightDecline();
  testGuards();

  std::printf("bspline_fair: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
#else
  std::printf("bspline_fair: CYBERCAD_HAS_NUMSCI off — trivial pass\n");
  return 0;
#endif
}

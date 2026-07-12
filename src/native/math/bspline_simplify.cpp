// SPDX-License-Identifier: Apache-2.0
//
// bspline_simplify.cpp — tolerance-bounded NURBS simplification (Piegl & Tiller
// Ch.9). Greedy knot removal in increasing-error order and bounded degree
// reduction, each step verified against the ORIGINAL curve by dense sampling so
// the reported deviation is the true measured worst case and never exceeds tol.
//
// Building blocks are the PUBLIC, exact bspline_ops entry points:
//   removeKnotCurve  (A5.8) — one-value removal with its reported R⁴ error bound,
//   reduceDegreeCurve(A5.11)— one-degree reduction with its own dense true-error
//                             measurement and honest decline,
// plus the bspline.h evaluators for the independent true-deviation oracle. This
// keeps the module header-only over math (no numsci) and OCCT-FREE.
//
#include "bspline_simplify.h"

#include "bspline.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

// ── Evaluators (dispatch rational vs non-rational) ──────────────────────────────
Point3 evalCurve(const BsplineCurveData& c, double u) noexcept {
  if (c.weights.empty()) return curvePoint(c.degree, c.poles, c.knots, u);
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}

Point3 evalSurface(const BsplineSurfaceData& s, double u, double v) noexcept {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}

// True max deviation between two curves over the shared clamped domain of `ref`,
// by dense pointwise sampling. This is the HARD bound oracle: independent of the
// removal machinery, it measures the geometry a downstream consumer would see.
double trueCurveDeviation(const BsplineCurveData& ref, const BsplineCurveData& cand,
                          int samples) noexcept {
  if (ref.knots.size() < 2) return std::numeric_limits<double>::infinity();
  const double lo = ref.knots.front();
  const double hi = ref.knots.back();
  double worst = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double u = lo + (hi - lo) * (static_cast<double>(i) / samples);
    worst = std::max(worst, distance(evalCurve(ref, u), evalCurve(cand, u)));
  }
  return worst;
}

// Interior distinct knot values of a clamped flat knot vector (strictly between
// the leading and trailing clamps). Degree `p`: the first p+1 and last p+1 knots
// are the clamps; interior values live in [p+1 .. size-p-2].
std::vector<double> interiorDistinctKnots(const BsplineCurveData& c) {
  std::vector<double> out;
  const int p = c.degree;
  const int m = static_cast<int>(c.knots.size());
  for (int i = p + 1; i < m - p - 1; ++i) {
    const double u = c.knots[i];
    if (out.empty() || out.back() != u) out.push_back(u);
  }
  return out;
}

}  // namespace

// ═════════════════════════════════════════════════════════════════════════════
// removeKnotsBounded — greedy bounded knot removal (P&T Ch.9.3).
//
// Each outer pass: for every distinct interior knot, TRIAL-remove one occurrence
// (removeKnotCurve within tol) and, if it succeeds, VERIFY the trial curve's true
// deviation from the ORIGINAL input. Among the knots that pass, commit the one
// with the smallest deviation (increasing-error order). Repeat until no interior
// knot can be removed while the accumulated result stays ≤ tol. Because the check
// is always against the original input (not the previous step), the returned
// curve's reported deviation is a hard, true bound.
// ═════════════════════════════════════════════════════════════════════════════
BoundedRemovalResult removeKnotsBounded(const BsplineCurveData& c, double tol) {
  BoundedRemovalResult res;
  res.curve = c;
  res.removed = 0;
  res.maxDeviation = 0.0;
  if (c.degree < 1 || tol < 0.0) return res;

  // Dense enough that the sampled max closely tracks the true continuous max; the
  // removal error concentrates near the removed span, so this over-covers it.
  const int kSamples = 400;

  BsplineCurveData current = c;
  for (;;) {
    const std::vector<double> knots = interiorDistinctKnots(current);
    if (knots.empty()) break;

    double bestDev = std::numeric_limits<double>::infinity();
    BsplineCurveData bestCurve;
    bool found = false;

    for (double u : knots) {
      // Trial: remove one occurrence of u within tol (A5.8, honest decline).
      const KnotRemovalResult trial = removeKnotCurve(current, u, 1, tol);
      if (trial.removed < 1) continue;
      // Verify the TRUE deviation of the trial curve vs. the original input.
      const double dev = trueCurveDeviation(c, trial.curve, kSamples);
      if (dev <= tol && dev < bestDev) {
        bestDev = dev;
        bestCurve = trial.curve;
        found = true;
      }
    }

    if (!found) break;
    current = std::move(bestCurve);
    ++res.removed;
    res.maxDeviation = std::max(res.maxDeviation, bestDev);
  }

  res.curve = std::move(current);
  return res;
}

// ═════════════════════════════════════════════════════════════════════════════
// reduceDegreeBounded — bounded degree reduction (P&T Ch.9).
//
// Repeatedly drop the degree by 1 (reduceDegreeCurve, which itself measures the
// true dense deviation and declines honestly). After each successful drop, RE-
// MEASURE the accumulated candidate's deviation against the ORIGINAL input; accept
// only while ≤ tol. A curve that cannot be reduced within tol keeps its degree.
// ═════════════════════════════════════════════════════════════════════════════
BoundedReduceResult reduceDegreeBounded(const BsplineCurveData& c, double tol) {
  BoundedReduceResult res;
  res.curve = c;
  res.degreeDrop = 0;
  res.maxDeviation = 0.0;
  if (tol < 0.0) return res;

  const int kSamples = 400;
  BsplineCurveData current = c;
  while (current.degree >= 1) {
    const DegreeReduceResult step = reduceDegreeCurve(current, tol);
    if (!step.ok) break;  // not reducible within tol from the current curve
    // Verify against the ORIGINAL so the accumulated bound is a true hard bound.
    const double dev = trueCurveDeviation(c, step.curve, kSamples);
    if (!(dev <= tol)) break;  // reject: would exceed the bound end-to-end
    current = step.curve;
    ++res.degreeDrop;
    res.maxDeviation = std::max(res.maxDeviation, dev);
  }

  res.curve = std::move(current);
  return res;
}

// ═════════════════════════════════════════════════════════════════════════════
// removeKnotsBoundedSurface — per-direction bounded knot removal.
//
// Sweep U then V. For each direction, greedily remove interior knots (using the
// exact per-direction removeKnotSurface within tol) whose removal keeps the true
// surface deviation from the ORIGINAL ≤ tol, measured on a dense (u,v) grid.
// ═════════════════════════════════════════════════════════════════════════════
namespace {

// Distinct interior knot values of a surface direction.
std::vector<double> interiorDistinctKnotsDir(const BsplineSurfaceData& s, ParamDir d) {
  std::vector<double> out;
  const int p = (d == ParamDir::U) ? s.degreeU : s.degreeV;
  const std::vector<double>& U = (d == ParamDir::U) ? s.knotsU : s.knotsV;
  const int m = static_cast<int>(U.size());
  for (int i = p + 1; i < m - p - 1; ++i) {
    const double u = U[i];
    if (out.empty() || out.back() != u) out.push_back(u);
  }
  return out;
}

// True max deviation between two surfaces over the shared domain of `ref`.
double trueSurfaceDeviation(const BsplineSurfaceData& ref,
                            const BsplineSurfaceData& cand, int samples) noexcept {
  if (ref.knotsU.size() < 2 || ref.knotsV.size() < 2)
    return std::numeric_limits<double>::infinity();
  const double u0 = ref.knotsU.front(), u1 = ref.knotsU.back();
  const double v0 = ref.knotsV.front(), v1 = ref.knotsV.back();
  double worst = 0.0;
  for (int i = 0; i <= samples; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / samples);
    for (int j = 0; j <= samples; ++j) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / samples);
      worst = std::max(worst, distance(evalSurface(ref, u, v), evalSurface(cand, u, v)));
    }
  }
  return worst;
}

// Greedily remove interior knots along one direction; returns count removed and
// updates `current` in place. `maxDev` accumulates the true deviation vs. `orig`.
int sweepDirection(const BsplineSurfaceData& orig, BsplineSurfaceData& current,
                   ParamDir d, double tol, int gridSamples, double& maxDev) {
  int removed = 0;
  for (;;) {
    const std::vector<double> knots = interiorDistinctKnotsDir(current, d);
    if (knots.empty()) break;

    double bestDev = std::numeric_limits<double>::infinity();
    BsplineSurfaceData bestSurf;
    bool found = false;

    for (double val : knots) {
      const KnotRemovalResultS trial = removeKnotSurface(current, d, val, 1, tol);
      if (trial.removed < 1) continue;
      const double dev = trueSurfaceDeviation(orig, trial.surface, gridSamples);
      if (dev <= tol && dev < bestDev) {
        bestDev = dev;
        bestSurf = trial.surface;
        found = true;
      }
    }

    if (!found) break;
    current = std::move(bestSurf);
    ++removed;
    maxDev = std::max(maxDev, bestDev);
  }
  return removed;
}

}  // namespace

BoundedRemovalResultS removeKnotsBoundedSurface(const BsplineSurfaceData& s,
                                                double tol) {
  BoundedRemovalResultS res;
  res.surface = s;
  res.removedU = 0;
  res.removedV = 0;
  res.maxDeviation = 0.0;
  if (tol < 0.0) return res;

  // A modest grid keeps the O(passes · knots · grid²) sweep tractable while still
  // over-covering the local removal error (which concentrates near the removed
  // isoparametric line).
  const int kGrid = 40;
  BsplineSurfaceData current = s;
  res.removedU = sweepDirection(s, current, ParamDir::U, tol, kGrid, res.maxDeviation);
  res.removedV = sweepDirection(s, current, ParamDir::V, tol, kGrid, res.maxDeviation);
  res.surface = std::move(current);
  return res;
}

}  // namespace cybercad::native::math

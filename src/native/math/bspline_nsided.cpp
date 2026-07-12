// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided.cpp — NURBS roadmap Layer 6 (N-SIDED boundary-filled surface) impl.
//
// Fill a closed N-gon boundary (N ≠ 4) with N tensor-product B-spline Coons sub-patches
// by Catmull-Clark-style MIDPOINT subdivision: split each boundary edge at its midpoint,
// take the polygon centroid as the single interior hub, and build one Coons quad per
// original corner (bounded by two boundary half-edges and two straight spokes to the
// centroid). The UNION reproduces every input boundary curve pointwise. It COMPOSES the
// exact Layer-1 ops (splitCurve / reparamCurve / knot-reversal to halve and orient the
// boundary edges) and bspline_coons (the per-quad fill). No linear solve — but the TU is
// guarded by CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated
// Layer-6 surfacing family (and because it composes bspline_coons, which is itself
// numsci-gated). With the guard OFF the TU is inert and the functions are absent.
//
#include "native/math/bspline_nsided.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint / nurbsCurvePoint (corner / midpoint eval)
#include "native/math/bspline_coons.h"  // CoonsBoundary / coonsPatch (per-quad fill)
#include "native/math/bspline_ops.h"    // splitCurve / reparamCurve (halve the edges)

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

bool wellFormed(const BsplineCurveData& c) {
  return c.weights.empty() && c.degree >= 1 && !c.poles.empty() &&
         static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 2;
}

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

// Reverse a curve's parametrization: C_rev(t) = C(1 − t) on the same [0,1] domain.
// Poles are reversed; the flat knot vector is mirrored about its span so the clamped
// [0,1] domain is preserved. Geometry-exact (the curve traced backwards).
BsplineCurveData reverseCurve(const BsplineCurveData& c) {
  BsplineCurveData r;
  r.degree = c.degree;
  r.poles.assign(c.poles.rbegin(), c.poles.rend());
  const double a = c.knots.front();
  const double b = c.knots.back();
  r.knots.resize(c.knots.size());
  for (std::size_t i = 0; i < c.knots.size(); ++i)
    r.knots[c.knots.size() - 1 - i] = a + b - c.knots[i];  // mirror + reverse order
  return r;  // non-rational in, non-rational out (weights left empty)
}

// The first half of edge e (e on [0,0.5]), reparametrized onto [0,1]. Runs V→M.
BsplineCurveData firstHalf(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reparamCurve(s.left, 0.0, 1.0);
}

// The second half of edge e (e on [0.5,1]), reparametrized onto [0,1] then REVERSED so
// it runs from the NEXT corner back to the midpoint (M_prev → … no: V → M direction the
// quad needs). splitCurve(e,0.5).right runs M→V(next); reversed it runs V(next)→M.
BsplineCurveData secondHalfReversed(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reverseCurve(reparamCurve(s.right, 0.0, 1.0));
}

// A straight degree-1 edge between two points (2 poles), on [0,1].
BsplineCurveData lineEdge(const Point3& a, const Point3& b) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0.0, 0.0, 1.0, 1.0};
  c.poles = {a, b};
  return c;
}

// ═════════════════════════════════════════════════════════════════════════════
// RATIONAL path — the whole subdivision + Coons boolean-sum runs in HOMOGENEOUS
// R⁴ (wx, wy, wz, w), de-homogenizing only at the very end, so a rational boundary
// (e.g. an exact circular arc) is reproduced EXACTLY rather than approximated.
// ═════════════════════════════════════════════════════════════════════════════

// A homogeneous R⁴ control point (w·x, w·y, w·z, w).
struct Homog4 { double x = 0.0, y = 0.0, z = 0.0, w = 0.0; };

Homog4 operator+(const Homog4& a, const Homog4& b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z, a.w + b.w};
}
Homog4 operator-(const Homog4& a, const Homog4& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z, a.w - b.w};
}

// Weight of curve pole i (empty weights ⇒ non-rational ⇒ w = 1).
double weightAt(const BsplineCurveData& c, int i) {
  return c.weights.empty() ? 1.0 : c.weights[i];
}

// Well-formed for the RATIONAL fill: clamped flat knots, degree ≥ 1, ≥ 2 poles, and — if
// rational — exactly one strictly-positive weight per pole. A non-positive or mismatched
// weight is dishonest and rejected.
bool wellFormedRational(const BsplineCurveData& c) {
  const int n = nPolesOf(c);
  if (c.degree < 1 || c.poles.empty() || n < 2) return false;
  if (static_cast<int>(c.knots.size()) != n + c.degree + 1) return false;
  if (!c.weights.empty()) {
    if (static_cast<int>(c.weights.size()) != n) return false;
    for (double w : c.weights)
      if (!(w > 0.0)) return false;
  }
  return true;
}

// Rational point evaluation (uses the NURBS evaluator when weighted).
Point3 evalCurveR(const BsplineCurveData& c, double u) {
  if (c.weights.empty()) return curvePoint(c.degree, c.poles, c.knots, u);
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}

// Reverse a (possibly rational) curve: C_rev(t) = C(1 − t) on the same [0,1] domain. Poles
// AND weights are reversed; the flat knot vector is mirrored about its span. Rational-exact.
BsplineCurveData reverseCurveR(const BsplineCurveData& c) {
  BsplineCurveData r;
  r.degree = c.degree;
  r.poles.assign(c.poles.rbegin(), c.poles.rend());
  if (!c.weights.empty()) r.weights.assign(c.weights.rbegin(), c.weights.rend());
  const double a = c.knots.front();
  const double b = c.knots.back();
  r.knots.resize(c.knots.size());
  for (std::size_t i = 0; i < c.knots.size(); ++i)
    r.knots[c.knots.size() - 1 - i] = a + b - c.knots[i];
  return r;
}

// First half of edge e (e on [0,0.5]) reparametrized onto [0,1], V→M. splitCurve/reparam
// preserve the rational half-arc EXACTLY (the Layer-1 ops run in homogeneous space).
BsplineCurveData firstHalfR(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reparamCurve(s.left, 0.0, 1.0);
}

// Second half of edge e (e on [0.5,1]) reparametrized onto [0,1] then reversed, V(next)→M.
BsplineCurveData secondHalfReversedR(const BsplineCurveData& e) {
  const CurveSplit s = splitCurve(e, 0.5);
  return reverseCurveR(reparamCurve(s.right, 0.0, 1.0));
}

// A straight degree-1 edge a→b carrying prescribed ENDPOINT WEIGHTS. A rational-linear
// segment with any positive endpoint weights still traces the SAME straight geometry (the
// weights only reparametrize speed along it), so this lets the interior spokes carry the
// exact corner weights of the adjoining rational boundary half-edges. That makes the four
// homogeneous corners of the Coons quad CONSISTENT, which is the precondition for the
// homogeneous boolean sum to reproduce every boundary edge exactly (Piegl & Tiller §10.5,
// rational Coons: matched corner weights).
BsplineCurveData weightedLineEdge(const Point3& a, const Point3& b, double wa, double wb) {
  BsplineCurveData c;
  c.degree = 1;
  c.knots = {0.0, 0.0, 1.0, 1.0};
  c.poles = {a, b};
  c.weights = {wa, wb};
  return c;
}

// Endpoint weight of a (possibly non-rational) curve at its first / last pole.
double startWeight(const BsplineCurveData& c) { return c.weights.empty() ? 1.0 : c.weights.front(); }
double endWeight(const BsplineCurveData& c)   { return c.weights.empty() ? 1.0 : c.weights.back(); }

// ── Homogeneous curve compatibility (raise to common degree + knot vector, exact) ──
// The Layer-1 elevate/refine ops round-trip through homogeneous R⁴, so applied to a
// rational curve they preserve its homogeneous net exactly. kKnotEps mirrors bspline_coons.
constexpr double kKnotEps = 1e-9;

std::vector<double> distinctKnotUnion(const std::vector<double>& a,
                                      const std::vector<double>& b) {
  struct KM { double v; int m; };
  std::vector<KM> uni;
  auto absorb = [&](const std::vector<double>& kv) {
    std::size_t i = 0;
    while (i < kv.size()) {
      const double val = kv[i];
      int mult = 0;
      while (i < kv.size() && std::fabs(kv[i] - val) <= kKnotEps) { ++mult; ++i; }
      auto it = std::find_if(uni.begin(), uni.end(),
                             [&](const KM& u) { return std::fabs(u.v - val) <= kKnotEps; });
      if (it == uni.end()) uni.push_back({val, mult});
      else it->m = std::max(it->m, mult);
    }
  };
  absorb(a);
  absorb(b);
  std::sort(uni.begin(), uni.end(), [](const KM& x, const KM& y) { return x.v < y.v; });
  std::vector<double> flat;
  for (const KM& u : uni)
    for (int r = 0; r < u.m; ++r) flat.push_back(u.v);
  return flat;
}

std::vector<double> knotsToInsert(const std::vector<double>& have,
                                  const std::vector<double>& target) {
  std::vector<double> ins;
  std::size_t i = 0;
  while (i < target.size()) {
    const double val = target[i];
    int need = 0;
    while (i < target.size() && std::fabs(target[i] - val) <= kKnotEps) { ++need; ++i; }
    int has = 0;
    for (double kv : have)
      if (std::fabs(kv - val) <= kKnotEps) ++has;
    for (int r = has; r < need; ++r) ins.push_back(val);
  }
  return ins;
}

// Raise two (possibly rational) curves to one common degree + knot vector. Exact on both.
bool unifyCurvesR(BsplineCurveData& a, BsplineCurveData& b) {
  const int maxDeg = std::max(a.degree, b.degree);
  if (a.degree < maxDeg) a = elevateDegreeCurve(a, maxDeg - a.degree);
  if (b.degree < maxDeg) b = elevateDegreeCurve(b, maxDeg - b.degree);
  const std::vector<double> uni = distinctKnotUnion(a.knots, b.knots);
  const std::vector<double> insA = knotsToInsert(a.knots, uni);
  const std::vector<double> insB = knotsToInsert(b.knots, uni);
  if (!insA.empty()) a = refineKnotCurve(a, insA);
  if (!insB.empty()) b = refineKnotCurve(b, insB);
  if (a.degree != b.degree || nPolesOf(a) != nPolesOf(b)) return false;
  if (a.knots.size() != b.knots.size()) return false;
  for (std::size_t i = 0; i < a.knots.size(); ++i)
    if (std::fabs(a.knots[i] - b.knots[i]) > 1e-7) return false;
  return true;
}

// A rational surface carried as its HOMOGENEOUS net: hpoles[i] = (w·x, w·y, w·z, w). The
// three boolean-sum summands live here so the sum is exact in R⁴; project once at the end.
struct HSurface {
  int degreeU = 0, degreeV = 0;
  int nPolesU = 0, nPolesV = 0;
  std::vector<Homog4> hpoles;  // row-major, U outer
  std::vector<double> knotsU, knotsV;
};

// Pack a rational BsplineSurfaceData (poles + weights, empty ⇒ w=1) into its homogeneous net.
HSurface toH(const BsplineSurfaceData& s) {
  HSurface h;
  h.degreeU = s.degreeU; h.degreeV = s.degreeV;
  h.nPolesU = s.nPolesU; h.nPolesV = s.nPolesV;
  h.knotsU = s.knotsU;   h.knotsV = s.knotsV;
  h.hpoles.resize(s.poles.size());
  for (std::size_t i = 0; i < s.poles.size(); ++i) {
    const double w = s.weights.empty() ? 1.0 : s.weights[i];
    h.hpoles[i] = {w * s.poles[i].x, w * s.poles[i].y, w * s.poles[i].z, w};
  }
  return h;
}

// L_v(u,v) = (1−v)·c0(u) + v·c1(u): ruled between c0/c1, degree 1 in v. Rational — the two
// weight rows are carried into the homogeneous net. c0/c1 must already be compatible in u.
BsplineSurfaceData ruledInV_R(const BsplineCurveData& c0, const BsplineCurveData& c1) {
  const int N = nPolesOf(c0);
  BsplineSurfaceData s;
  s.degreeU = c0.degree;   s.degreeV = 1;
  s.nPolesU = N;           s.nPolesV = 2;
  s.knotsU = c0.knots;     s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles.assign(static_cast<std::size_t>(N) * 2, Point3{});
  s.weights.assign(static_cast<std::size_t>(N) * 2, 1.0);
  for (int i = 0; i < N; ++i) {
    s.poles[static_cast<std::size_t>(i) * 2 + 0] = c0.poles[i];
    s.poles[static_cast<std::size_t>(i) * 2 + 1] = c1.poles[i];
    s.weights[static_cast<std::size_t>(i) * 2 + 0] = weightAt(c0, i);
    s.weights[static_cast<std::size_t>(i) * 2 + 1] = weightAt(c1, i);
  }
  return s;
}

// L_u(u,v) = (1−u)·d0(v) + u·d1(v): ruled between d0/d1, degree 1 in u. Rational.
BsplineSurfaceData ruledInU_R(const BsplineCurveData& d0, const BsplineCurveData& d1) {
  const int M = nPolesOf(d0);
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = d0.degree;
  s.nPolesU = 2;                   s.nPolesV = M;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = d0.knots;
  s.poles.assign(static_cast<std::size_t>(2) * M, Point3{});
  s.weights.assign(static_cast<std::size_t>(2) * M, 1.0);
  for (int j = 0; j < M; ++j) {
    s.poles[static_cast<std::size_t>(0) * M + j] = d0.poles[j];
    s.poles[static_cast<std::size_t>(1) * M + j] = d1.poles[j];
    s.weights[static_cast<std::size_t>(0) * M + j] = weightAt(d0, j);
    s.weights[static_cast<std::size_t>(1) * M + j] = weightAt(d1, j);
  }
  return s;
}

// B(u,v) = the degree-(1,1) bilinear tensor product of the four homogeneous corners. The
// corner weights are the boundary curves' endpoint weights (shared by both adjoining edges).
BsplineSurfaceData bilinearCornersR(const Point3& p00, double w00, const Point3& p10, double w10,
                                    const Point3& p01, double w01, const Point3& p11, double w11) {
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = 1;
  s.nPolesU = 2;                   s.nPolesV = 2;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles   = {p00, p01, p10, p11};   // [0*2+0],[0*2+1],[1*2+0],[1*2+1]
  s.weights = {w00, w01, w10, w11};
  return s;
}

// ── Surface-direction compatibility on the RATIONAL summands (exact Layer-1) ──
const std::vector<double>& dirKnots(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.knotsU : s.knotsV;
}
int dirDegree(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.degreeU : s.degreeV;
}
void unifyDirectionR(BsplineSurfaceData& a, BsplineSurfaceData& b, ParamDir d) {
  const int maxDeg = std::max(dirDegree(a, d), dirDegree(b, d));
  if (dirDegree(a, d) < maxDeg) a = elevateDegreeSurface(a, d, maxDeg - dirDegree(a, d));
  if (dirDegree(b, d) < maxDeg) b = elevateDegreeSurface(b, d, maxDeg - dirDegree(b, d));
  const std::vector<double> uni = distinctKnotUnion(dirKnots(a, d), dirKnots(b, d));
  const std::vector<double> insA = knotsToInsert(dirKnots(a, d), uni);
  const std::vector<double> insB = knotsToInsert(dirKnots(b, d), uni);
  if (!insA.empty()) a = refineKnotSurface(a, d, insA);
  if (!insB.empty()) b = refineKnotSurface(b, d, insB);
}
bool sameBasis(const BsplineSurfaceData& a, const BsplineSurfaceData& b) {
  if (a.degreeU != b.degreeU || a.degreeV != b.degreeV) return false;
  if (a.nPolesU != b.nPolesU || a.nPolesV != b.nPolesV) return false;
  if (a.knotsU.size() != b.knotsU.size() || a.knotsV.size() != b.knotsV.size()) return false;
  for (std::size_t i = 0; i < a.knotsU.size(); ++i)
    if (std::fabs(a.knotsU[i] - b.knotsU[i]) > 1e-7) return false;
  for (std::size_t i = 0; i < a.knotsV.size(); ++i)
    if (std::fabs(a.knotsV[i] - b.knotsV[i]) > 1e-7) return false;
  return true;
}

// Homogeneous Coons boolean sum `L_u ⊕ L_v ⊖ B` for a (possibly rational) quad. Returns the
// projected (poles, weights) surface, or ok=false on a mismatched-corner / non-positive
// projected weight / basis-unification failure — honest, never a silently-wrong net.
bool coonsPatchRational(const BsplineCurveData& c0_in, const BsplineCurveData& c1_in,
                        const BsplineCurveData& d0_in, const BsplineCurveData& d1_in,
                        double tol, BsplineSurfaceData& out) {
  // Corner consistency (rational endpoints): c0(0)=d0(0), c0(1)=d1(0), c1(0)=d0(1), c1(1)=d1(1).
  const Point3 c00 = evalCurveR(c0_in, 0.0), c01 = evalCurveR(c0_in, 1.0);
  const Point3 c10 = evalCurveR(c1_in, 0.0), c11 = evalCurveR(c1_in, 1.0);
  const Point3 d00 = evalCurveR(d0_in, 0.0), d01 = evalCurveR(d0_in, 1.0);
  const Point3 d10 = evalCurveR(d1_in, 0.0), d11 = evalCurveR(d1_in, 1.0);
  const double cornErr = std::max({distance(c00, d00), distance(c01, d10),
                                   distance(c10, d01), distance(c11, d11)});
  if (cornErr > tol) return false;

  // Make each opposing pair compatible in its own direction (exact, rational-preserving).
  BsplineCurveData c0 = c0_in, c1 = c1_in, d0 = d0_in, d1 = d1_in;
  if (!unifyCurvesR(c0, c1)) return false;
  if (!unifyCurvesR(d0, d1)) return false;

  // Corner points + weights from the compatible curves (endpoints are interpolatory on a
  // clamped B-spline, so the endpoint pole/weight IS the corner in homogeneous space).
  const int nc = nPolesOf(c0), nd = nPolesOf(d0);
  const Point3 p00 = c0.poles.front(), p10 = c0.poles.back();
  const Point3 p01 = c1.poles.front(), p11 = c1.poles.back();
  const double w00 = weightAt(c0, 0), w10 = weightAt(c0, nc - 1);
  const double w01 = weightAt(c1, 0), w11 = weightAt(c1, nc - 1);
  (void)nd;

  BsplineSurfaceData Lv = ruledInV_R(c0, c1);
  BsplineSurfaceData Lu = ruledInU_R(d0, d1);
  BsplineSurfaceData B  = bilinearCornersR(p00, w00, p10, w10, p01, w01, p11, w11);

  for (int pass = 0; pass < 2; ++pass) {
    unifyDirectionR(Lv, Lu, ParamDir::U);  unifyDirectionR(Lv, Lu, ParamDir::V);
    unifyDirectionR(Lv, B,  ParamDir::U);  unifyDirectionR(Lv, B,  ParamDir::V);
    unifyDirectionR(Lu, B,  ParamDir::U);  unifyDirectionR(Lu, B,  ParamDir::V);
  }
  if (!sameBasis(Lv, Lu) || !sameBasis(Lv, B)) return false;

  // Boolean sum in HOMOGENEOUS R⁴, then project once.
  const HSurface hLv = toH(Lv), hLu = toH(Lu), hB = toH(B);
  const std::size_t n = hLv.hpoles.size();
  out.degreeU = Lv.degreeU; out.degreeV = Lv.degreeV;
  out.nPolesU = Lv.nPolesU; out.nPolesV = Lv.nPolesV;
  out.knotsU = Lv.knotsU;   out.knotsV = Lv.knotsV;
  out.poles.resize(n);
  out.weights.resize(n);
  bool anyNonUnitWeight = false;
  for (std::size_t i = 0; i < n; ++i) {
    const Homog4 h = hLv.hpoles[i] + hLu.hpoles[i] - hB.hpoles[i];
    if (!(h.w > 0.0)) return false;  // non-positive projected weight — documented guard
    out.poles[i] = {h.x / h.w, h.y / h.w, h.z / h.w};
    out.weights[i] = h.w;
    if (std::fabs(h.w - 1.0) > 1e-12) anyNonUnitWeight = true;
  }
  // Non-rational reduction: if every projected weight is 1, drop the weight vector so the
  // result is byte-identical in shape to the non-rational fillNSided patch.
  if (!anyNonUnitWeight) out.weights.clear();
  return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// N-sided boundary consistency.
// ─────────────────────────────────────────────────────────────────────────────

NSidedBoundaryCheck verifyNSidedBoundary(const NSidedBoundary& b, double tol) {
  NSidedBoundaryCheck r;
  r.n = static_cast<int>(b.edges.size());

  if (r.n < 3) { r.reason = "N-sided fill needs at least 3 boundary edges"; return r; }

  for (int i = 0; i < r.n; ++i) {
    if (!wellFormed(b.edges[i])) {
      r.reason = "edge " + std::to_string(i) + " is rational or malformed";
      return r;
    }
  }

  // Consecutive corners must meet: edges[i](1) == edges[(i+1)%N](0). This is the
  // closed-loop precondition (the N-gon actually closes on shared corners).
  double worst = 0.0;
  for (int i = 0; i < r.n; ++i) {
    const int j = (i + 1) % r.n;
    const Point3 end_i = evalCurve(b.edges[i], 1.0);
    const Point3 start_j = evalCurve(b.edges[j], 0.0);
    worst = std::max(worst, distance(end_i, start_j));
  }
  r.maxCornerError = worst;

  if (worst > tol) {
    r.reason = "boundary is not a closed loop (consecutive corners do not coincide)";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// N-sided fill (midpoint subdivision → N Coons sub-patches).
// ─────────────────────────────────────────────────────────────────────────────

NSidedFillResult fillNSided(const NSidedBoundary& b, double tol) {
  NSidedFillResult r;

  // Step 1 — verify the boundary (declines honestly on non-closed / rational / degenerate).
  const NSidedBoundaryCheck chk = verifyNSidedBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  const int N = chk.n;

  // Step 2 — corners V[i] = edges[i](0), edge midpoints M[i] = edges[i](0.5),
  // and the centroid C = mean(V[i]) (the single interior hub all sub-patches share).
  std::vector<Point3> V(N), M(N);
  Vec3 sum{0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    V[i] = evalCurve(b.edges[i], 0.0);
    M[i] = evalCurve(b.edges[i], 0.5);
    sum = sum + Vec3{V[i].x, V[i].y, V[i].z};
  }
  const Point3 C{sum.x / N, sum.y / N, sum.z / N};
  r.centroid = C;

  // Guard: a degenerate (zero-length) spoke to the centroid would collapse a sub-quad.
  for (int i = 0; i < N; ++i) {
    if (distance(M[i], C) <= tol) {
      r.reason = "degenerate N-gon: an edge midpoint coincides with the centroid";
      return r;
    }
  }

  // Step 3 — one Coons quad per corner V[i]. Coons convention corners:
  //   P00 = V[i], P10 = M[i], P01 = M[i-1], P11 = C.
  //   c0 (v=0, V[i]→M[i])   = first half of e[i]                 (boundary),
  //   d0 (u=0, V[i]→M[i-1]) = second half of e[i-1] reversed     (boundary),
  //   c1 (v=1, M[i-1]→C)    = straight spoke M[i-1] → C,
  //   d1 (u=1, M[i]→C)      = straight spoke M[i]   → C.
  r.patches.reserve(N);
  for (int i = 0; i < N; ++i) {
    const int prev = (i + N - 1) % N;

    CoonsBoundary quad;
    quad.c0 = firstHalf(b.edges[i]);                 // V[i] → M[i]     (on boundary)
    quad.d0 = secondHalfReversed(b.edges[prev]);     // V[i] → M[i-1]   (on boundary)
    quad.c1 = lineEdge(M[prev], C);                  // M[i-1] → C      (interior spoke)
    quad.d1 = lineEdge(M[i], C);                     // M[i]   → C      (interior spoke)

    const CoonsResult cr = coonsPatch(quad, tol);
    if (!cr.ok) {
      r.patches.clear();
      r.reason = "sub-quad " + std::to_string(i) + " Coons fill declined: " + cr.reason;
      return r;
    }
    r.patches.push_back(cr.surface);
  }

  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational N-sided boundary consistency + fill (homogeneous subdivision + Coons).
// ─────────────────────────────────────────────────────────────────────────────

NSidedBoundaryCheck verifyNSidedBoundaryRational(const NSidedBoundary& b, double tol) {
  NSidedBoundaryCheck r;
  r.n = static_cast<int>(b.edges.size());

  if (r.n < 3) { r.reason = "N-sided fill needs at least 3 boundary edges"; return r; }

  for (int i = 0; i < r.n; ++i) {
    if (!wellFormedRational(b.edges[i])) {
      r.reason = "edge " + std::to_string(i) +
                 " is malformed or carries a non-positive/mismatched weight";
      return r;
    }
  }

  // Consecutive corners must meet (measured with the RATIONAL evaluator).
  double worst = 0.0;
  for (int i = 0; i < r.n; ++i) {
    const int j = (i + 1) % r.n;
    worst = std::max(worst, distance(evalCurveR(b.edges[i], 1.0), evalCurveR(b.edges[j], 0.0)));
  }
  r.maxCornerError = worst;

  if (worst > tol) {
    r.reason = "boundary is not a closed loop (consecutive corners do not coincide)";
    return r;
  }
  r.ok = true;
  return r;
}

NSidedFillRationalResult fillNSidedRational(const NSidedBoundary& b, double tol) {
  NSidedFillRationalResult r;

  // Step 1 — verify (declines honestly on non-closed / malformed / non-positive-weight).
  const NSidedBoundaryCheck chk = verifyNSidedBoundaryRational(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  const int N = chk.n;

  // Step 2 — corners V[i] = edges[i](0), midpoints M[i] = edges[i](0.5) (rational eval),
  // centroid C = mean(V[i]). The midpoint is the split point of the RATIONAL half-arcs.
  std::vector<Point3> V(N), M(N);
  Vec3 sum{0.0, 0.0, 0.0};
  for (int i = 0; i < N; ++i) {
    V[i] = evalCurveR(b.edges[i], 0.0);
    M[i] = evalCurveR(b.edges[i], 0.5);
    sum = sum + Vec3{V[i].x, V[i].y, V[i].z};
  }
  const Point3 C{sum.x / N, sum.y / N, sum.z / N};
  r.centroid = C;

  for (int i = 0; i < N; ++i) {
    if (distance(M[i], C) <= tol) {
      r.reason = "degenerate N-gon: an edge midpoint coincides with the centroid";
      return r;
    }
  }

  // Step 3 — one homogeneous Coons quad per corner V[i]. The two boundary half-edges are
  // RATIONAL-exact (splitCurve preserves the arc); the two interior spokes are straight
  // (weight 1). The boolean sum runs in R⁴, de-homogenized only at the end.
  // The homogeneous boolean sum reproduces a boundary edge exactly only when the four
  // Coons corners are consistent in HOMOGENEOUS space — i.e. the two boundary/spoke curves
  // meeting at each corner carry the SAME weight there. A split rational arc has a non-unit
  // weight at its midpoint M[i] (e.g. cos45° for a quarter-arc), so the interior spokes must
  // inherit those midpoint weights. The centroid C is a free interior corner shared by both
  // spokes; give it weight 1 (any consistent positive value works). The corners V[i] are real
  // boundary corners where an arc has weight 1, so the two half-edges already agree there.
  std::vector<double> wM(N);  // weight of each edge at its midpoint (from the split half-edge)
  for (int i = 0; i < N; ++i) wM[i] = endWeight(firstHalfR(b.edges[i]));  // = weight at M[i]

  r.patches.reserve(N);
  for (int i = 0; i < N; ++i) {
    const int prev = (i + N - 1) % N;

    const BsplineCurveData c0 = firstHalfR(b.edges[i]);              // V[i] → M[i]   (rational)
    const BsplineCurveData d0 = secondHalfReversedR(b.edges[prev]);  // V[i] → M[i-1] (rational)
    // Interior spokes carry the matched corner weights (midpoint weight → 1 at the centroid).
    const BsplineCurveData c1 = weightedLineEdge(M[prev], C, wM[prev], 1.0);  // M[i-1] → C
    const BsplineCurveData d1 = weightedLineEdge(M[i], C, wM[i], 1.0);        // M[i]   → C

    BsplineSurfaceData patch;
    if (!coonsPatchRational(c0, c1, d0, d1, tol, patch)) {
      r.patches.clear();
      r.reason = "sub-quad " + std::to_string(i) + " homogeneous Coons fill declined";
      return r;
    }
    r.patches.push_back(std::move(patch));
  }

  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

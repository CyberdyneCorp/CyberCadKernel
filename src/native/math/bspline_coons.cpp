// SPDX-License-Identifier: Apache-2.0
//
// bspline_coons.cpp — NURBS roadmap Layer 6 (boundary-filled Coons patch) impl.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 (and Coons 1967 /
// Farin) — the bilinearly-blended Coons patch as a BOOLEAN SUM `Coons = L_u ⊕ L_v ⊖ B`.
// It COMPOSES Layer 1 (bspline_ops: elevateDegreeCurve/refineKnotCurve to make the
// boundary curves compatible, and elevateDegreeSurface/refineKnotSurface to bring the
// three summand surfaces to one common tensor-product basis EXACTLY, so their control
// nets can be added/subtracted). Unlike skin/gordon this construction uses NO linear
// solve — the ruled and bilinear summands are direct — but the TU is guarded by
// CYBERCAD_HAS_NUMSCI to sit uniformly with the rest of the numsci-gated Layer-6
// surfacing family. With the guard OFF the TU is inert and the functions are absent.
//
#include "native/math/bspline_coons.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint / nurbsCurvePoint (corner evaluation)
#include "native/math/bspline_ops.h"    // elevate/refine curve + surface (compatibility)

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

constexpr double kKnotEps = 1e-9;  // knot-value coincidence tolerance for the union.

int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

bool wellFormed(const BsplineCurveData& c) {
  return c.weights.empty() && c.degree >= 1 && !c.poles.empty() &&
         static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 2;
}

// Rational well-formedness: exactly one strictly-positive weight per pole, degree ≥ 1,
// clamped flat knot vector, ≥ 2 poles.
bool wellFormedRat(const BsplineCurveData& c) {
  if (c.weights.size() != c.poles.size()) return false;  // non-rational or mismatched
  if (c.poles.empty()) return false;
  for (double w : c.weights)
    if (!(w > 0.0)) return false;
  return c.degree >= 1 &&
         static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 2;
}

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
}

Point3 evalCurveRat(const BsplineCurveData& c, double u) {
  return nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u);
}

// ── Homogeneous R⁴ point (wx, wy, wz, w) — the lift of a rational control point. ──
struct Homog4 { double x, y, z, w; };
Homog4 liftH(const Point3& p, double w) { return {w * p.x, w * p.y, w * p.z, w}; }

// The homogeneous point (wx, wy, wz, w) of a rational curve at parameter u — the
// numerator/denominator of the NURBS quotient BEFORE the divide (the value the boolean
// sum must be consistent in). Ordinary B-spline basis on the lifted R⁴ net.
Homog4 homogCurvePoint(const BsplineCurveData& c, double u) {
  const int n = nPolesOf(c) - 1;
  const int span = findSpan(n, c.degree, u, c.knots);
  std::vector<double> Nb(c.degree + 1);
  basisFuns(span, u, c.degree, c.knots, Nb);
  Homog4 acc{0, 0, 0, 0};
  for (int j = 0; j <= c.degree; ++j) {
    const int idx = span - c.degree + j;
    const double w = c.weights[idx];
    const Point3& p = c.poles[idx];
    acc.x += Nb[j] * w * p.x;
    acc.y += Nb[j] * w * p.y;
    acc.z += Nb[j] * w * p.z;
    acc.w += Nb[j] * w;
  }
  return acc;
}

// ── Knot union of two curves in ONE direction (max multiplicity per distinct value) ──
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

// Knots to INSERT into `have` to reach `target` (both flat, same domain).
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

// Raise two curves to a common degree + common knot vector (exact Layer-1), so both
// share degree, knots and control-point count. Geometry-preserving on both.
bool unifyCurves(BsplineCurveData& a, BsplineCurveData& b) {
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

// ── Surface-direction helpers (bring two surfaces to a common basis in one dir) ───
const std::vector<double>& dirKnots(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.knotsU : s.knotsV;
}
int dirDegree(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.degreeU : s.degreeV;
}

void unifyDirection(BsplineSurfaceData& a, BsplineSurfaceData& b, ParamDir d) {
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

// ── The three boolean-sum summands ────────────────────────────────────────────────

// L_v(u,v) = (1−v)·c0(u) + v·c1(u): ruled between c0 and c1, degree 1 in v (2 poles),
// the c-shape carried in u. c0/c1 must already be compatible (same U-basis, N poles).
// Row-major U-outer: pole(i,j) = poles[i*2 + j], i over u, j over v; j=0→c0, j=1→c1.
BsplineSurfaceData ruledInV(const BsplineCurveData& c0, const BsplineCurveData& c1) {
  const int N = nPolesOf(c0);
  BsplineSurfaceData s;
  s.degreeU = c0.degree;   s.degreeV = 1;
  s.nPolesU = N;           s.nPolesV = 2;
  s.knotsU = c0.knots;     s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles.assign(static_cast<std::size_t>(N) * 2, Point3{});
  for (int i = 0; i < N; ++i) {
    s.poles[static_cast<std::size_t>(i) * 2 + 0] = c0.poles[i];
    s.poles[static_cast<std::size_t>(i) * 2 + 1] = c1.poles[i];
  }
  s.weights.clear();
  return s;
}

// L_u(u,v) = (1−u)·d0(v) + u·d1(v): ruled between d0 and d1, degree 1 in u (2 poles),
// the d-shape carried in v. d0/d1 must already be compatible (same V-basis, M poles).
// Row-major U-outer: pole(i,j) = poles[i*M + j], i over u (i=0→d0, i=1→d1), j over v.
BsplineSurfaceData ruledInU(const BsplineCurveData& d0, const BsplineCurveData& d1) {
  const int M = nPolesOf(d0);
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = d0.degree;
  s.nPolesU = 2;                   s.nPolesV = M;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = d0.knots;
  s.poles.assign(static_cast<std::size_t>(2) * M, Point3{});
  for (int j = 0; j < M; ++j) {
    s.poles[static_cast<std::size_t>(0) * M + j] = d0.poles[j];  // pole(0,j) = d0
    s.poles[static_cast<std::size_t>(1) * M + j] = d1.poles[j];  // pole(1,j) = d1
  }
  s.weights.clear();
  return s;
}

// B(u,v) = the degree-(1,1) bilinear tensor product of the four corner points.
// Row-major U-outer 2×2: pole(i,j) = poles[i*2 + j], i over u, j over v.
//   pole(0,0) = P00 = corner(u=0,v=0)   pole(0,1) = P01 = corner(u=0,v=1)
//   pole(1,0) = P10 = corner(u=1,v=0)   pole(1,1) = P11 = corner(u=1,v=1)
BsplineSurfaceData bilinearCorners(const Point3& p00, const Point3& p10,
                                   const Point3& p01, const Point3& p11) {
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = 1;
  s.nPolesU = 2;                   s.nPolesV = 2;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles = {p00, p01, p10, p11};  // [0*2+0], [0*2+1], [1*2+0], [1*2+1]
  s.weights.clear();
  return s;
}

// ── Rational summands: the SAME three boolean-sum surfaces built in HOMOGENEOUS R⁴. ──
// A rational surface is carried as (poles, weights); the homogeneous net is (w·P, w). The
// ruled surfaces linearly blend the boundary curves' HOMOGENEOUS nets (a rational ruled
// surface's control net is exactly the pair of homogeneous curve nets), so weight(i,j) is
// just the source curve's weight and pole(i,j) its Euclidean pole. The bilinear term is the
// homogeneous tensor of the four corner homogeneous points. Because the corner homogeneous
// points equal the curves' endpoint homogeneous points, `L_u − B` cancels on every edge in
// R⁴, so the rational boolean sum interpolates every rational boundary curve.

// L_v(u,v) rational: pole(i,0)=c0.pole[i] w c0.w[i]; pole(i,1)=c1.pole[i] w c1.w[i].
BsplineSurfaceData ruledInVRat(const BsplineCurveData& c0, const BsplineCurveData& c1) {
  const int N = nPolesOf(c0);
  BsplineSurfaceData s;
  s.degreeU = c0.degree;   s.degreeV = 1;
  s.nPolesU = N;           s.nPolesV = 2;
  s.knotsU = c0.knots;     s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles.assign(static_cast<std::size_t>(N) * 2, Point3{});
  s.weights.assign(static_cast<std::size_t>(N) * 2, 1.0);
  for (int i = 0; i < N; ++i) {
    s.poles[static_cast<std::size_t>(i) * 2 + 0] = c0.poles[i];
    s.weights[static_cast<std::size_t>(i) * 2 + 0] = c0.weights[i];
    s.poles[static_cast<std::size_t>(i) * 2 + 1] = c1.poles[i];
    s.weights[static_cast<std::size_t>(i) * 2 + 1] = c1.weights[i];
  }
  return s;
}

// L_u(u,v) rational: pole(0,j)=d0.pole[j] w d0.w[j]; pole(1,j)=d1.pole[j] w d1.w[j].
BsplineSurfaceData ruledInURat(const BsplineCurveData& d0, const BsplineCurveData& d1) {
  const int M = nPolesOf(d0);
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = d0.degree;
  s.nPolesU = 2;                   s.nPolesV = M;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = d0.knots;
  s.poles.assign(static_cast<std::size_t>(2) * M, Point3{});
  s.weights.assign(static_cast<std::size_t>(2) * M, 1.0);
  for (int j = 0; j < M; ++j) {
    s.poles[static_cast<std::size_t>(0) * M + j] = d0.poles[j];
    s.weights[static_cast<std::size_t>(0) * M + j] = d0.weights[j];
    s.poles[static_cast<std::size_t>(1) * M + j] = d1.poles[j];
    s.weights[static_cast<std::size_t>(1) * M + j] = d1.weights[j];
  }
  return s;
}

// B(u,v) rational: the degree-(1,1) tensor of the four corner (pole, weight) pairs.
BsplineSurfaceData bilinearCornersRat(const Point3& p00, double w00, const Point3& p10, double w10,
                                      const Point3& p01, double w01, const Point3& p11,
                                      double w11) {
  BsplineSurfaceData s;
  s.degreeU = 1;                   s.degreeV = 1;
  s.nPolesU = 2;                   s.nPolesV = 2;
  s.knotsU = {0.0, 0.0, 1.0, 1.0}; s.knotsV = {0.0, 0.0, 1.0, 1.0};
  s.poles = {p00, p01, p10, p11};  // [0*2+0], [0*2+1], [1*2+0], [1*2+1]
  s.weights = {w00, w01, w10, w11};
  return s;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Boundary-corner consistency.
// ─────────────────────────────────────────────────────────────────────────────

CoonsCornerCheck verifyCoonsBoundary(const CoonsBoundary& b, double tol) {
  CoonsCornerCheck r;

  if (!wellFormed(b.c0)) { r.reason = "c0 is rational or malformed"; return r; }
  if (!wellFormed(b.c1)) { r.reason = "c1 is rational or malformed"; return r; }
  if (!wellFormed(b.d0)) { r.reason = "d0 is rational or malformed"; return r; }
  if (!wellFormed(b.d1)) { r.reason = "d1 is rational or malformed"; return r; }

  // The four shared corners must coincide (endpoints on the clamped [0,1] domain).
  const Point3 c00 = evalCurve(b.c0, 0.0), c01 = evalCurve(b.c0, 1.0);  // c0 endpoints
  const Point3 c10 = evalCurve(b.c1, 0.0), c11 = evalCurve(b.c1, 1.0);  // c1 endpoints
  const Point3 d00 = evalCurve(b.d0, 0.0), d01 = evalCurve(b.d0, 1.0);  // d0 endpoints
  const Point3 d10 = evalCurve(b.d1, 0.0), d11 = evalCurve(b.d1, 1.0);  // d1 endpoints

  // Corner identities: c0(0)=d0(0), c0(1)=d1(0), c1(0)=d0(1), c1(1)=d1(1).
  const double e00 = distance(c00, d00);  // P00
  const double e10 = distance(c01, d10);  // P10
  const double e01 = distance(c10, d01);  // P01
  const double e11 = distance(c11, d11);  // P11
  r.maxCornerError = std::max(std::max(e00, e10), std::max(e01, e11));

  if (r.maxCornerError > tol) {
    r.reason = "boundary corners do not coincide within tolerance";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Coons patch (boolean sum L_u ⊕ L_v ⊖ B).
// ─────────────────────────────────────────────────────────────────────────────

CoonsResult coonsPatch(const CoonsBoundary& b, double tol) {
  CoonsResult r;

  // Step 1 — verify the boundary (declines honestly on mismatched/rational/degenerate).
  const CoonsCornerCheck chk = verifyCoonsBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  // Step 2 — make each opposing boundary pair compatible in its own direction (exact).
  // c0/c1 share the U basis (they run in u); d0/d1 share the V basis (they run in v).
  BsplineCurveData c0 = b.c0, c1 = b.c1, d0 = b.d0, d1 = b.d1;
  if (!unifyCurves(c0, c1)) { r.reason = "c0/c1 compatibility failed"; return r; }
  if (!unifyCurves(d0, d1)) { r.reason = "d0/d1 compatibility failed"; return r; }

  // The four corners (from the compatible curves — exact-preserving, same points).
  const Point3 p00 = evalCurve(c0, 0.0);  // c0(0) = d0(0)
  const Point3 p10 = evalCurve(c0, 1.0);  // c0(1) = d1(0)
  const Point3 p01 = evalCurve(c1, 0.0);  // c1(0) = d0(1)
  const Point3 p11 = evalCurve(c1, 1.0);  // c1(1) = d1(1)

  // Step 3 — the three boolean-sum summands.
  BsplineSurfaceData Lv = ruledInV(c0, c1);              // (1−v)c0 + v·c1
  BsplineSurfaceData Lu = ruledInU(d0, d1);              // (1−u)d0 + u·d1
  BsplineSurfaceData B  = bilinearCorners(p00, p10, p01, p11);

  // Step 4 — bring all three to ONE common basis in each direction (exact Layer-1),
  // then form the Coons net poles(Lv) + poles(Lu) − poles(B). Unify pairwise, twice, to
  // fold any degree/knot raised by a later pass back onto the others (idempotent once
  // shared). Mirrors the Gordon boolean-sum unification.
  for (int pass = 0; pass < 2; ++pass) {
    unifyDirection(Lv, Lu, ParamDir::U);  unifyDirection(Lv, Lu, ParamDir::V);
    unifyDirection(Lv, B,  ParamDir::U);  unifyDirection(Lv, B,  ParamDir::V);
    unifyDirection(Lu, B,  ParamDir::U);  unifyDirection(Lu, B,  ParamDir::V);
  }

  if (!sameBasis(Lv, Lu) || !sameBasis(Lv, B)) {
    r.reason = "summands failed to reach a common basis"; return r;
  }

  BsplineSurfaceData S = Lv;  // inherits the common basis/degrees/knots.
  const std::size_t n = Lv.poles.size();
  for (std::size_t i = 0; i < n; ++i) {
    S.poles[i] = {Lv.poles[i].x + Lu.poles[i].x - B.poles[i].x,
                  Lv.poles[i].y + Lu.poles[i].y - B.poles[i].y,
                  Lv.poles[i].z + Lu.poles[i].z - B.poles[i].z};
  }
  S.weights.clear();  // non-rational.

  r.surface = std::move(S);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational boundary-corner consistency (position AND weight must match).
// ─────────────────────────────────────────────────────────────────────────────

CoonsCornerCheck verifyRationalCoonsBoundary(const CoonsBoundary& b, double tol) {
  CoonsCornerCheck r;

  if (!wellFormedRat(b.c0)) { r.reason = "c0 is non-rational or malformed"; return r; }
  if (!wellFormedRat(b.c1)) { r.reason = "c1 is non-rational or malformed"; return r; }
  if (!wellFormedRat(b.d0)) { r.reason = "d0 is non-rational or malformed"; return r; }
  if (!wellFormedRat(b.d1)) { r.reason = "d1 is non-rational or malformed"; return r; }

  // The four shared corners must coincide in HOMOGENEOUS R⁴ (position AND weight): the two
  // curves meeting at a corner must carry the same effective weight there, or the rational
  // boolean sum's bilinear term cannot cancel the ruled terms on the boundary.
  auto homogCornerDist = [](const Homog4& a, const Homog4& c) {
    const double dx = a.x - c.x, dy = a.y - c.y, dz = a.z - c.z, dw = a.w - c.w;
    return std::sqrt(dx * dx + dy * dy + dz * dz + dw * dw);
  };
  const Homog4 c0_0 = homogCurvePoint(b.c0, 0.0), c0_1 = homogCurvePoint(b.c0, 1.0);
  const Homog4 c1_0 = homogCurvePoint(b.c1, 0.0), c1_1 = homogCurvePoint(b.c1, 1.0);
  const Homog4 d0_0 = homogCurvePoint(b.d0, 0.0), d0_1 = homogCurvePoint(b.d0, 1.0);
  const Homog4 d1_0 = homogCurvePoint(b.d1, 0.0), d1_1 = homogCurvePoint(b.d1, 1.0);

  // Corner identities: c0(0)=d0(0), c0(1)=d1(0), c1(0)=d0(1), c1(1)=d1(1).
  const double e00 = homogCornerDist(c0_0, d0_0);  // P00
  const double e10 = homogCornerDist(c0_1, d1_0);  // P10
  const double e01 = homogCornerDist(c1_0, d0_1);  // P01
  const double e11 = homogCornerDist(c1_1, d1_1);  // P11
  r.maxCornerError = std::max(std::max(e00, e10), std::max(e01, e11));

  if (r.maxCornerError > tol) {
    r.reason = "rational boundary corners do not coincide (position or weight) within tolerance";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational Coons patch (boolean sum L_u ⊕ L_v ⊖ B in HOMOGENEOUS R⁴).
// ─────────────────────────────────────────────────────────────────────────────

CoonsResult coonsPatchRational(const CoonsBoundary& b, double tol) {
  CoonsResult r;

  // Step 1 — verify the rational boundary (declines on non-rational/mismatched/degenerate).
  const CoonsCornerCheck chk = verifyRationalCoonsBoundary(b, tol);
  r.maxCornerError = chk.maxCornerError;
  if (!chk.ok) { r.reason = chk.reason; return r; }

  // Step 2 — make each opposing boundary pair compatible in its own direction (exact,
  // rational-aware: elevate/refine carry the weights through the homogeneous lift).
  BsplineCurveData c0 = b.c0, c1 = b.c1, d0 = b.d0, d1 = b.d1;
  if (!unifyCurves(c0, c1)) { r.reason = "c0/c1 compatibility failed"; return r; }
  if (!unifyCurves(d0, d1)) { r.reason = "d0/d1 compatibility failed"; return r; }
  if (c0.weights.size() != c0.poles.size() || c1.weights.size() != c1.poles.size() ||
      d0.weights.size() != d0.poles.size() || d1.weights.size() != d1.poles.size()) {
    r.reason = "boundary lost its weights under compatibility"; return r;
  }

  // The four corner homogeneous points (from the compatible curves — exact-preserving). Use
  // the along-curve poles/weights at the clamped endpoints (a clamped curve interpolates its
  // end control point with its end weight, so the corner homogeneous point is exact).
  const int Nc = nPolesOf(c0);
  const int Md = nPolesOf(d0);
  const Homog4 h00 = liftH(c0.poles[0],       c0.weights[0]);        // c0(0) = d0(0)
  const Homog4 h10 = liftH(c0.poles[Nc - 1],  c0.weights[Nc - 1]);   // c0(1) = d1(0)
  const Homog4 h01 = liftH(c1.poles[0],       c1.weights[0]);        // c1(0) = d0(1)
  const Homog4 h11 = liftH(c1.poles[Nc - 1],  c1.weights[Nc - 1]);   // c1(1) = d1(1)
  (void)Md;

  // Step 3 — the three homogeneous boolean-sum summands (built rational-aware).
  BsplineSurfaceData Lv = ruledInVRat(c0, c1);              // (1−v)c0 + v·c1
  BsplineSurfaceData Lu = ruledInURat(d0, d1);              // (1−u)d0 + u·d1
  BsplineSurfaceData B  = bilinearCornersRat(
      Point3{h00.x / h00.w, h00.y / h00.w, h00.z / h00.w}, h00.w,
      Point3{h10.x / h10.w, h10.y / h10.w, h10.z / h10.w}, h10.w,
      Point3{h01.x / h01.w, h01.y / h01.w, h01.z / h01.w}, h01.w,
      Point3{h11.x / h11.w, h11.y / h11.w, h11.z / h11.w}, h11.w);

  // Step 4 — bring all three to ONE common basis in each direction (exact rational-aware
  // Layer-1), then form the Coons net homog(Lv) + homog(Lu) − homog(B), projected back.
  for (int pass = 0; pass < 2; ++pass) {
    unifyDirection(Lv, Lu, ParamDir::U);  unifyDirection(Lv, Lu, ParamDir::V);
    unifyDirection(Lv, B,  ParamDir::U);  unifyDirection(Lv, B,  ParamDir::V);
    unifyDirection(Lu, B,  ParamDir::U);  unifyDirection(Lu, B,  ParamDir::V);
  }

  if (!sameBasis(Lv, Lu) || !sameBasis(Lv, B)) {
    r.reason = "summands failed to reach a common basis"; return r;
  }
  if (Lv.weights.size() != Lv.poles.size() || Lu.weights.size() != Lu.poles.size() ||
      B.weights.size() != B.poles.size()) {
    r.reason = "rational summand lost its weights under unification"; return r;
  }

  BsplineSurfaceData S = Lv;  // inherits the common basis/degrees/knots.
  const std::size_t n = Lv.poles.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Homog4 hv = liftH(Lv.poles[i], Lv.weights[i]);
    const Homog4 hu = liftH(Lu.poles[i], Lu.weights[i]);
    const Homog4 hb = liftH(B.poles[i],  B.weights[i]);
    const double sx = hv.x + hu.x - hb.x;
    const double sy = hv.y + hu.y - hb.y;
    const double sz = hv.z + hu.z - hb.z;
    const double sw = hv.w + hu.w - hb.w;
    if (!(sw > 0.0)) { r.reason = "rational boolean sum produced a non-positive weight"; return r; }
    S.poles[i] = {sx / sw, sy / sw, sz / sw};
    S.weights[i] = sw;
  }

  r.surface = std::move(S);
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

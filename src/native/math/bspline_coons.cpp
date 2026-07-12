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

#include "native/math/bspline.h"        // curvePoint (corner evaluation)
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

Point3 evalCurve(const BsplineCurveData& c, double u) {
  return curvePoint(c.degree, c.poles, c.knots, u);
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

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

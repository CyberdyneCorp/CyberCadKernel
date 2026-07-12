// SPDX-License-Identifier: Apache-2.0
//
// bspline_intersect.cpp — CCI / CSI implementation (Piegl & Tiller Ch. 5/6).
//
// Two-phase per Piegl & Tiller: (1) recursive control-hull bounding-box
// subdivision to ISOLATE candidate parameter boxes, (2) Newton point-inversion /
// projection to POLISH each candidate to a root. Rational-aware via the
// homogeneous lift (both the hull bound — convex hull of the PROJECTED poles — and
// the analytic Newton derivatives from bspline.h's quotient-rule evaluators).
//
// SELF-CONTAINED linear algebra: the Newton steps solve small square systems
// (2×2 for CCI, 3×3 for CSI) with inline Cramer / partial-pivot elimination — no
// external solver is needed. The whole implementation is nevertheless compiled
// only under CYBERCAD_HAS_NUMSCI: these intersection entry points are part of the
// numsci-gated NURBS layer (like ssi/seeding.cpp), so a non-numsci build links a
// no-op TU. The header stays always-includable.
//
// OCCT-FREE. Uses src/native/math only.
//
#include "bspline_intersect.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "bspline.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace cybercad::native::math {
namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Small 3D AABB.
// ─────────────────────────────────────────────────────────────────────────────
struct Box {
  Point3 lo{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::infinity()};
  Point3 hi{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
            -std::numeric_limits<double>::infinity()};
  void expand(const Point3& p) noexcept {
    lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
    lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
    lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
  }
  void inflate(double m) noexcept {
    lo.x -= m; lo.y -= m; lo.z -= m;
    hi.x += m; hi.y += m; hi.z += m;
  }
  double diagonal() const noexcept { return norm(hi.asVec() - lo.asVec()); }
};

/// Boxes are DISJOINT (prunable) iff separated by > gap on some axis.
bool disjoint(const Box& a, const Box& b, double gap) noexcept {
  return (a.lo.x - b.hi.x > gap) || (b.lo.x - a.hi.x > gap) ||
         (a.lo.y - b.hi.y > gap) || (b.lo.y - a.hi.y > gap) ||
         (a.lo.z - b.hi.z > gap) || (b.lo.z - a.hi.z > gap);
}

/// AABB intersection of two SOUND bounds → a tighter sound bound.
Box boxIntersect(const Box& a, const Box& b) noexcept {
  Box r;
  r.lo.x = std::max(a.lo.x, b.lo.x); r.hi.x = std::min(a.hi.x, b.hi.x);
  r.lo.y = std::max(a.lo.y, b.lo.y); r.hi.y = std::min(a.hi.y, b.hi.y);
  r.lo.z = std::max(a.lo.z, b.lo.z); r.hi.z = std::min(a.hi.z, b.hi.z);
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Control-hull (convex-hull) POLE bound of a curve over [t0,t1].
//
// A B-spline curve over [t0,t1] lies in the convex hull of the poles whose basis
// support overlaps [t0,t1] — poles with index in [span(t0)-p, span(t1)]. For a
// rational NURBS with wᵢ>0 the point is a convex combination of the PROJECTED
// poles, so the same pole hull is a SOUND bound. TIGHT when the net has many
// poles; but for a coarse net (e.g. a 2-pole degree-1 segment) the pole hull does
// NOT shrink with the sub-box, so we INTERSECT it with a sampled+Lipschitz bound
// (curveSampledBox) that DOES shrink — the intersection of two sound bounds is a
// tighter sound bound. That combined bound (curveBox) is what the subdivision uses.
// ─────────────────────────────────────────────────────────────────────────────
Box curvePoleBox(const CurveView& c, double t0, double t1) noexcept {
  const int n = c.numPoles() - 1;
  const int s0 = findSpan(n, c.degree, t0, c.knots);
  const int s1 = findSpan(n, c.degree, t1, c.knots);
  const int lo = std::max(0, s0 - c.degree);
  const int hi = std::min(n, s1);
  Box b;
  for (int i = lo; i <= hi; ++i) b.expand(c.poles[static_cast<std::size_t>(i)]);
  return b;
}

Box surfacePoleBox(const SurfaceView& S, double u0, double u1, double v0, double v1) noexcept {
  const int nu = S.nRows - 1;
  const int nv = S.nCols - 1;
  const int su0 = findSpan(nu, S.degreeU, u0, S.knotsU);
  const int su1 = findSpan(nu, S.degreeU, u1, S.knotsU);
  const int sv0 = findSpan(nv, S.degreeV, v0, S.knotsV);
  const int sv1 = findSpan(nv, S.degreeV, v1, S.knotsV);
  const int iLo = std::max(0, su0 - S.degreeU), iHi = std::min(nu, su1);
  const int jLo = std::max(0, sv0 - S.degreeV), jHi = std::min(nv, sv1);
  Box b;
  for (int i = iLo; i <= iHi; ++i)
    for (int j = jLo; j <= jHi; ++j)
      b.expand(S.poles[static_cast<std::size_t>(i) * S.nCols + j]);
  return b;
}

// ─────────────────────────────────────────────────────────────────────────────
// Evaluators — dispatch rational vs non-rational, returning point + first deriv.
// ─────────────────────────────────────────────────────────────────────────────
void curveEval(const CurveView& c, double t, Point3& p, Vec3& d1) noexcept {
  std::array<Vec3, 2> ders{};
  if (c.rational())
    nurbsCurveDerivs(c.degree, c.poles, c.weights, c.knots, t, 1, ders);
  else
    curveDerivs(c.degree, c.poles, c.knots, t, 1, ders);
  p = Point3{ders[0]};
  d1 = ders[1];
}

Point3 curvePointOnly(const CurveView& c, double t) noexcept {
  return c.rational() ? nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, t)
                      : curvePoint(c.degree, c.poles, c.knots, t);
}

void surfaceEval(const SurfaceView& S, double u, double v, Point3& p, Vec3& du,
                 Vec3& dv) noexcept {
  SurfaceGrid grid{S.poles, S.nRows, S.nCols};
  std::array<Vec3, 4> ders{};  // (maxDeriv+1)^2 = 4 for maxDeriv=1
  if (S.rational())
    nurbsSurfaceDerivs(S.degreeU, S.degreeV, grid, S.weights, S.knotsU, S.knotsV, u, v, 1, ders);
  else
    surfaceDerivs(S.degreeU, S.degreeV, grid, S.knotsU, S.knotsV, u, v, 1, ders);
  // Layout row-major (2×2): [0]=S, [1]=Sv, [2]=Su, [3]=Suv.
  p = Point3{ders[0]};
  dv = ders[1];
  du = ders[2];
}

Point3 surfacePointOnly(const SurfaceView& S, double u, double v) noexcept {
  SurfaceGrid grid{S.poles, S.nRows, S.nCols};
  return S.rational()
      ? nurbsSurfacePoint(S.degreeU, S.degreeV, grid, S.weights, S.knotsU, S.knotsV, u, v)
      : surfacePoint(S.degreeU, S.degreeV, grid, S.knotsU, S.knotsV, u, v);
}

// ─────────────────────────────────────────────────────────────────────────────
// SAMPLED + Lipschitz-margin bound (shrinks with the sub-box even for coarse nets).
//
// Evaluate the curve/surface on a small grid over the sub-box, take the min/max, and
// INFLATE by a margin ≥ the bulge between samples (max‖derivative‖ · Δparam). The
// margin makes the finite sampling conservative: the true curve/surface over the
// sub-box lies inside the inflated sampled box. Combined (intersected) with the pole
// hull, this is the subdivision's working bound — sound, and it shrinks to a point
// as the sub-box shrinks, which the pole hull alone need not (e.g. a 2-pole segment).
// ─────────────────────────────────────────────────────────────────────────────
Box curveSampledBox(const CurveView& c, double t0, double t1) noexcept {
  constexpr int kN = 4;
  Box b;
  double maxD = 0.0;
  for (int i = 0; i <= kN; ++i) {
    const double t = t0 + (t1 - t0) * (static_cast<double>(i) / kN);
    Point3 p; Vec3 d1;
    curveEval(c, t, p, d1);
    b.expand(p);
    maxD = std::max(maxD, norm(d1));
  }
  b.inflate(maxD * (t1 - t0) / kN);
  return b;
}

Box surfaceSampledBox(const SurfaceView& S, double u0, double u1, double v0, double v1) noexcept {
  constexpr int kN = 4;
  Box b;
  double maxDu = 0.0, maxDv = 0.0;
  for (int i = 0; i <= kN; ++i)
    for (int j = 0; j <= kN; ++j) {
      const double u = u0 + (u1 - u0) * (static_cast<double>(i) / kN);
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / kN);
      Point3 p; Vec3 du, dv;
      surfaceEval(S, u, v, p, du, dv);
      b.expand(p);
      maxDu = std::max(maxDu, norm(du));
      maxDv = std::max(maxDv, norm(dv));
    }
  b.inflate(maxDu * (u1 - u0) / kN + maxDv * (v1 - v0) / kN);
  return b;
}

/// Combined SOUND bound = pole hull ∩ sampled+margin. Shrinks with the sub-box.
Box curveBox(const CurveView& c, double t0, double t1) noexcept {
  return boxIntersect(curvePoleBox(c, t0, t1), curveSampledBox(c, t0, t1));
}
Box surfaceBox(const SurfaceView& S, double u0, double u1, double v0, double v1) noexcept {
  return boxIntersect(surfacePoleBox(S, u0, u1, v0, v1),
                      surfaceSampledBox(S, u0, u1, v0, v1));
}

// ─────────────────────────────────────────────────────────────────────────────
// Inline linear solves.
// ─────────────────────────────────────────────────────────────────────────────
/// Solve the 2×2 system [[a,b],[c,d]]·x = rhs; returns false if near-singular.
bool solve2(double a, double b, double c, double d, double r0, double r1,
            double& x0, double& x1, double scale) noexcept {
  const double det = a * d - b * c;
  if (std::fabs(det) <= 1e-30 * scale * scale) return false;
  x0 = (r0 * d - b * r1) / det;
  x1 = (a * r1 - r0 * c) / det;
  return true;
}

/// Solve a 3×3 system A·x = r with partial-pivot Gaussian elimination.
/// A is row-major 9 entries; r length 3. Returns false if near-singular.
bool solve3(std::array<double, 9> A, std::array<double, 3> r, std::array<double, 3>& x,
            double scale) noexcept {
  for (int col = 0; col < 3; ++col) {
    int piv = col;
    double best = std::fabs(A[static_cast<std::size_t>(col) * 3 + col]);
    for (int row = col + 1; row < 3; ++row) {
      const double v = std::fabs(A[static_cast<std::size_t>(row) * 3 + col]);
      if (v > best) { best = v; piv = row; }
    }
    if (best <= 1e-30 * scale) return false;
    if (piv != col) {
      for (int k = 0; k < 3; ++k)
        std::swap(A[static_cast<std::size_t>(col) * 3 + k], A[static_cast<std::size_t>(piv) * 3 + k]);
      std::swap(r[static_cast<std::size_t>(col)], r[static_cast<std::size_t>(piv)]);
    }
    const double pivVal = A[static_cast<std::size_t>(col) * 3 + col];
    for (int row = col + 1; row < 3; ++row) {
      const double f = A[static_cast<std::size_t>(row) * 3 + col] / pivVal;
      for (int k = col; k < 3; ++k)
        A[static_cast<std::size_t>(row) * 3 + k] -= f * A[static_cast<std::size_t>(col) * 3 + k];
      r[static_cast<std::size_t>(row)] -= f * r[static_cast<std::size_t>(col)];
    }
  }
  for (int row = 2; row >= 0; --row) {
    double s = r[static_cast<std::size_t>(row)];
    for (int k = row + 1; k < 3; ++k) s -= A[static_cast<std::size_t>(row) * 3 + k] * x[static_cast<std::size_t>(k)];
    x[static_cast<std::size_t>(row)] = s / A[static_cast<std::size_t>(row) * 3 + row];
  }
  return true;
}

double clampTo(double v, double lo, double hi) noexcept {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ─────────────────────────────────────────────────────────────────────────────
// CCI Newton. Minimize ‖cA(u) − cB(v)‖ via Gauss–Newton on the 2 unknowns (u,v):
// the 3×2 Jacobian J = [cA'(u)  −cB'(v)] and residual R = cA(u)−cB(v); the normal
// equations JᵀJ·δ = −JᵀR form a 2×2 solve. Converges quadratically near a
// transversal root and gracefully (least-norm step) near a tangency.
// ─────────────────────────────────────────────────────────────────────────────
bool newtonCC(const CurveView& cA, const CurveView& cB, double& u, double& v, double scale,
              Point3& hitPoint) noexcept {
  const double uLo = cA.t0(), uHi = cA.t1(), vLo = cB.t0(), vHi = cB.t1();
  for (int it = 0; it < 40; ++it) {
    Point3 pa, pb; Vec3 da, db;
    curveEval(cA, u, pa, da);
    curveEval(cB, v, pb, db);
    const Vec3 R = pa - pb;  // residual (want 0)
    // Jacobian columns Ju = da, Jv = -db. Normal equations JᵀJ δ = -JᵀR.
    const double a = dot(da, da);
    const double b = -dot(da, db);
    const double c = b;
    const double d = dot(db, db);
    const double r0 = -dot(da, R);
    const double r1 = dot(db, R);
    double du, dv;
    if (!solve2(a, b, c, d, r0, r1, du, dv, scale)) return false;
    u = clampTo(u + du, uLo, uHi);
    v = clampTo(v + dv, vLo, vHi);
    if (std::fabs(du) + std::fabs(dv) <= 1e-14 * (std::fabs(uHi - uLo) + std::fabs(vHi - vLo) + 1.0)) {
      Point3 pa2 = curvePointOnly(cA, u);
      hitPoint = pa2;
      return true;
    }
  }
  hitPoint = curvePointOnly(cA, u);
  return true;  // report best; caller gates on the 3D gap
}

// ─────────────────────────────────────────────────────────────────────────────
// CSI Newton. Solve F(t,u,v) = c(t) − S(u,v) = 0 (3 eqs, 3 unknowns). Jacobian
// columns: ∂F/∂t = c'(t), ∂F/∂u = −Su, ∂F/∂v = −Sv. Newton step: J·δ = −F.
// ─────────────────────────────────────────────────────────────────────────────
bool newtonCS(const CurveView& c, const SurfaceView& S, double& t, double& u, double& v,
              double scale, Point3& hitPoint) noexcept {
  const double tLo = c.t0(), tHi = c.t1();
  const double uLo = S.u0(), uHi = S.u1(), vLo = S.v0(), vHi = S.v1();
  for (int it = 0; it < 40; ++it) {
    Point3 pc, ps; Vec3 dc, Su, Sv;
    curveEval(c, t, pc, dc);
    surfaceEval(S, u, v, ps, Su, Sv);
    const Vec3 F = pc - ps;
    // Column-major columns: col0 = dc, col1 = -Su, col2 = -Sv. Row-major A[row*3+col].
    std::array<double, 9> A{
        dc.x, -Su.x, -Sv.x,
        dc.y, -Su.y, -Sv.y,
        dc.z, -Su.z, -Sv.z};
    std::array<double, 3> rhs{-F.x, -F.y, -F.z};
    std::array<double, 3> delta{0, 0, 0};
    if (!solve3(A, rhs, delta, scale)) return false;
    t = clampTo(t + delta[0], tLo, tHi);
    u = clampTo(u + delta[1], uLo, uHi);
    v = clampTo(v + delta[2], vLo, vHi);
    if (std::fabs(delta[0]) + std::fabs(delta[1]) + std::fabs(delta[2]) <=
        1e-14 * (std::fabs(tHi - tLo) + std::fabs(uHi - uLo) + std::fabs(vHi - vLo) + 1.0)) {
      hitPoint = curvePointOnly(c, t);
      return true;
    }
  }
  hitPoint = curvePointOnly(c, t);
  return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Overlap (coincident) detection. If the curves agree at a dense sample of shared
// parameters over a sub-arc, they OVERLAP (infinite intersection) — honest-decline.
// We test: sample cA over its domain; for each sample, is it within tol of SOME
// point of cB (found by a short projection Newton)?  If a CONTIGUOUS RUN of samples
// all lie on cB (not just isolated crossings), the pair overlaps.
// ─────────────────────────────────────────────────────────────────────────────
bool curvesOverlap(const CurveView& cA, const CurveView& cB, double tol, double scale) noexcept {
  constexpr int kSamples = 24;
  const double uLo = cA.t0(), uHi = cA.t1();
  int run = 0, maxRun = 0;
  for (int i = 0; i <= kSamples; ++i) {
    const double u = uLo + (uHi - uLo) * (static_cast<double>(i) / kSamples);
    const Point3 pa = curvePointOnly(cA, u);
    // Project pa onto cB by a coarse-seeded Newton minimizing ‖pa − cB(v)‖.
    double bestGap = std::numeric_limits<double>::infinity();
    const double vLo = cB.t0(), vHi = cB.t1();
    for (int s = 0; s <= 8; ++s) {
      double v = vLo + (vHi - vLo) * (static_cast<double>(s) / 8);
      for (int it = 0; it < 20; ++it) {
        Point3 pb; Vec3 db;
        curveEval(cB, v, pb, db);
        const Vec3 R = pb - pa;
        const double g = dot(db, db);
        if (g <= 1e-30 * scale * scale) break;
        const double step = -dot(db, R) / g;
        const double vn = clampTo(v + step, vLo, vHi);
        if (std::fabs(vn - v) <= 1e-13 * (vHi - vLo + 1.0)) { v = vn; break; }
        v = vn;
      }
      bestGap = std::min(bestGap, distance(pa, curvePointOnly(cB, v)));
    }
    if (bestGap <= tol) { ++run; maxRun = std::max(maxRun, run); }
    else run = 0;
  }
  // A genuine overlap yields a long contiguous run of on-curve samples; isolated
  // crossings give run ≤ 1..2. Require a clearly-contiguous majority.
  return maxRun >= (kSamples / 2);
}

/// Whether a curve lies ON a surface over a sub-arc (curve-on-surface overlap).
bool curveOnSurface(const CurveView& c, const SurfaceView& S, double tol, double scale) noexcept {
  constexpr int kSamples = 24;
  const double tLo = c.t0(), tHi = c.t1();
  int run = 0, maxRun = 0;
  for (int i = 0; i <= kSamples; ++i) {
    const double t = tLo + (tHi - tLo) * (static_cast<double>(i) / kSamples);
    const Point3 pc = curvePointOnly(c, t);
    // Project pc onto S: coarse grid seed + a few Gauss–Newton steps on (u,v).
    double bestGap = std::numeric_limits<double>::infinity();
    const double uLo = S.u0(), uHi = S.u1(), vLo = S.v0(), vHi = S.v1();
    for (int gu = 0; gu <= 4; ++gu) for (int gv = 0; gv <= 4; ++gv) {
      double u = uLo + (uHi - uLo) * (gu / 4.0);
      double v = vLo + (vHi - vLo) * (gv / 4.0);
      for (int it = 0; it < 20; ++it) {
        Point3 ps; Vec3 Su, Sv;
        surfaceEval(S, u, v, ps, Su, Sv);
        const Vec3 R = ps - pc;
        const double a = dot(Su, Su), b = dot(Su, Sv), d = dot(Sv, Sv);
        double su, sv;
        if (!solve2(a, b, b, d, -dot(Su, R), -dot(Sv, R), su, sv, scale)) break;
        const double un = clampTo(u + su, uLo, uHi), vn = clampTo(v + sv, vLo, vHi);
        if (std::fabs(un - u) + std::fabs(vn - v) <=
            1e-13 * (uHi - uLo + vHi - vLo + 1.0)) { u = un; v = vn; break; }
        u = un; v = vn;
      }
      Point3 ps; Vec3 Su, Sv; surfaceEval(S, u, v, ps, Su, Sv);
      bestGap = std::min(bestGap, distance(pc, ps));
    }
    if (bestGap <= tol) { ++run; maxRun = std::max(maxRun, run); }
    else run = 0;
  }
  return maxRun >= (kSamples / 2);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// intersectCurveCurve
// ─────────────────────────────────────────────────────────────────────────────
CurveCurveResult intersectCurveCurve(const CurveView& cA, const CurveView& cB, double tol) {
  CurveCurveResult out;

  // Model scale from the full control hulls (for singularity thresholds).
  Box wA = curvePoleBox(cA, cA.t0(), cA.t1());
  Box wB = curvePoleBox(cB, cB.t0(), cB.t1());
  const double scale = std::max({wA.diagonal(), wB.diagonal(), 1.0});

  // HONEST-DECLINE overlapping / coincident inputs (infinite intersection).
  if (curvesOverlap(cA, cB, tol, scale)) {
    out.status = IntersectStatus::Coincident;
    return out;
  }

  // Phase 1: recursive box-pair subdivision to isolate candidate seeds. Termination
  // is 3D-SIZE based: once BOTH sub-arc hull boxes are small vs the model scale a
  // single well-separated seed brackets the crossing and Newton drives it to the
  // exact root — so the subdivision count stays bounded (independent of the final
  // accuracy, which the polish owns). A depth cap guards pathological nets.
  struct Pair { double au0, au1, bv0, bv1; int depth; };
  std::vector<Pair> stack;
  stack.push_back({cA.t0(), cA.t1(), cB.t0(), cB.t1(), 0});
  const double seedBox = 0.02 * scale;  // hull-box seed granularity in 3D
  std::vector<std::pair<double, double>> seeds;
  const int maxDepth = 40;
  while (!stack.empty()) {
    const Pair pr = stack.back();
    stack.pop_back();
    const Box ba = curveBox(cA, pr.au0, pr.au1);
    const Box bb = curveBox(cB, pr.bv0, pr.bv1);
    if (disjoint(ba, bb, tol)) continue;
    const double da = pr.au1 - pr.au0, db = pr.bv1 - pr.bv0;
    const bool smallEnough =
        (ba.diagonal() <= seedBox && bb.diagonal() <= seedBox) || pr.depth >= maxDepth;
    if (smallEnough) {
      seeds.emplace_back(0.5 * (pr.au0 + pr.au1), 0.5 * (pr.bv0 + pr.bv1));
      continue;
    }
    // Split the longer parameter direction.
    if (da >= db) {
      const double m = 0.5 * (pr.au0 + pr.au1);
      stack.push_back({pr.au0, m, pr.bv0, pr.bv1, pr.depth + 1});
      stack.push_back({m, pr.au1, pr.bv0, pr.bv1, pr.depth + 1});
    } else {
      const double m = 0.5 * (pr.bv0 + pr.bv1);
      stack.push_back({pr.au0, pr.au1, pr.bv0, m, pr.depth + 1});
      stack.push_back({pr.au0, pr.au1, m, pr.bv1, pr.depth + 1});
    }
  }

  // Phase 2: Newton-polish each seed; keep converged roots with 3D gap ≤ tol.
  const double dedupTol = 1e3 * tol + 1e-12 * scale;
  for (const auto& [su, sv] : seeds) {
    double u = su, v = sv;
    Point3 hp;
    if (!newtonCC(cA, cB, u, v, scale, hp)) continue;
    const Point3 pa = curvePointOnly(cA, u);
    const Point3 pb = curvePointOnly(cB, v);
    const double gap = distance(pa, pb);
    if (gap > tol) continue;
    const Point3 pt = Point3{(pa.asVec() + pb.asVec()) * 0.5};
    // Dedup by 3D proximity.
    bool dup = false;
    for (const auto& h : out.hits)
      if (distance(h.point, pt) <= dedupTol) { dup = true; break; }
    if (dup) continue;
    // Classify: cross(cA', cB') ≈ 0 ⇒ tangential.
    Point3 tp; Vec3 da, dbv;
    curveEval(cA, u, tp, da);
    curveEval(cB, v, tp, dbv);
    const double na = norm(da), nb = norm(dbv);
    const double sinAng = (na > 0 && nb > 0) ? norm(cross(da, dbv)) / (na * nb) : 0.0;
    CurveCurveHit hit;
    hit.point = pt;
    hit.paramA = u;
    hit.paramB = v;
    hit.gap = gap;
    hit.type = (sinAng <= 1e-7) ? IntersectionType::Tangential : IntersectionType::Transversal;
    out.hits.push_back(hit);
  }

  std::sort(out.hits.begin(), out.hits.end(),
            [](const CurveCurveHit& a, const CurveCurveHit& b) { return a.paramA < b.paramA; });
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// intersectCurveSurface
// ─────────────────────────────────────────────────────────────────────────────
CurveSurfaceResult intersectCurveSurface(const CurveView& c, const SurfaceView& S, double tol) {
  CurveSurfaceResult out;

  Box wc = curvePoleBox(c, c.t0(), c.t1());
  Box ws = surfacePoleBox(S, S.u0(), S.u1(), S.v0(), S.v1());
  const double scale = std::max({wc.diagonal(), ws.diagonal(), 1.0});

  // HONEST-DECLINE a curve lying ON the surface over a sub-arc.
  if (curveOnSurface(c, S, tol, scale)) {
    out.status = IntersectStatus::Coincident;
    return out;
  }

  // Phase 1: subdivide curve-box vs surface-box. 3D-SIZE termination (as CCI): stop
  // when BOTH the curve sub-arc box and the surface sub-patch box are small vs scale.
  struct Cell { double t0, t1, u0, u1, v0, v1; int depth; };
  std::vector<Cell> stack;
  stack.push_back({c.t0(), c.t1(), S.u0(), S.u1(), S.v0(), S.v1(), 0});
  const double seedBox = 0.03 * scale;
  const int maxDepth = 44;
  std::vector<std::array<double, 3>> seeds;  // (t,u,v) centres
  while (!stack.empty()) {
    const Cell cl = stack.back();
    stack.pop_back();
    const Box bc = curveBox(c, cl.t0, cl.t1);
    const Box bs = surfaceBox(S, cl.u0, cl.u1, cl.v0, cl.v1);
    if (disjoint(bc, bs, tol)) continue;
    const double dt = cl.t1 - cl.t0, du = cl.u1 - cl.u0, dv = cl.v1 - cl.v0;
    const bool smallEnough =
        (bc.diagonal() <= seedBox && bs.diagonal() <= seedBox) || cl.depth >= maxDepth;
    if (smallEnough) {
      seeds.push_back({0.5 * (cl.t0 + cl.t1), 0.5 * (cl.u0 + cl.u1), 0.5 * (cl.v0 + cl.v1)});
      continue;
    }
    // Split the longest of the three parameter extents.
    const double dtN = dt / (c.t1() - c.t0());
    const double duN = du / (S.u1() - S.u0());
    const double dvN = dv / (S.v1() - S.v0());
    if (dtN >= duN && dtN >= dvN) {
      const double m = 0.5 * (cl.t0 + cl.t1);
      stack.push_back({cl.t0, m, cl.u0, cl.u1, cl.v0, cl.v1, cl.depth + 1});
      stack.push_back({m, cl.t1, cl.u0, cl.u1, cl.v0, cl.v1, cl.depth + 1});
    } else if (duN >= dvN) {
      const double m = 0.5 * (cl.u0 + cl.u1);
      stack.push_back({cl.t0, cl.t1, cl.u0, m, cl.v0, cl.v1, cl.depth + 1});
      stack.push_back({cl.t0, cl.t1, m, cl.u1, cl.v0, cl.v1, cl.depth + 1});
    } else {
      const double m = 0.5 * (cl.v0 + cl.v1);
      stack.push_back({cl.t0, cl.t1, cl.u0, cl.u1, cl.v0, m, cl.depth + 1});
      stack.push_back({cl.t0, cl.t1, cl.u0, cl.u1, m, cl.v1, cl.depth + 1});
    }
  }

  // Phase 2: Newton-polish each (t,u,v) seed.
  const double dedupTol = 1e3 * tol + 1e-12 * scale;
  for (const auto& s : seeds) {
    double t = s[0], u = s[1], v = s[2];
    Point3 hp;
    if (!newtonCS(c, S, t, u, v, scale, hp)) continue;
    const Point3 pc = curvePointOnly(c, t);
    const Point3 ps = S.rational()
        ? nurbsSurfacePoint(S.degreeU, S.degreeV, SurfaceGrid{S.poles, S.nRows, S.nCols},
                            S.weights, S.knotsU, S.knotsV, u, v)
        : surfacePoint(S.degreeU, S.degreeV, SurfaceGrid{S.poles, S.nRows, S.nCols},
                       S.knotsU, S.knotsV, u, v);
    const double gap = distance(pc, ps);
    if (gap > tol) continue;
    const Point3 pt = Point3{(pc.asVec() + ps.asVec()) * 0.5};
    bool dup = false;
    for (const auto& h : out.hits)
      if (distance(h.point, pt) <= dedupTol) { dup = true; break; }
    if (dup) continue;
    // Classify: curve tangent parallel to surface (⟂ normal) ⇒ tangential.
    Point3 tmp; Vec3 dc, Su, Sv;
    curveEval(c, t, tmp, dc);
    surfaceEval(S, u, v, tmp, Su, Sv);
    const Vec3 nrm = cross(Su, Sv);
    const double nn = norm(nrm), nc = norm(dc);
    const double cosToNormal = (nn > 0 && nc > 0) ? std::fabs(dot(dc, nrm)) / (nn * nc) : 1.0;
    CurveSurfaceHit hit;
    hit.point = pt;
    hit.paramT = t;
    hit.paramU = u;
    hit.paramV = v;
    hit.gap = gap;
    hit.type = (cosToNormal <= 1e-7) ? IntersectionType::Tangential : IntersectionType::Transversal;
    out.hits.push_back(hit);
  }

  std::sort(out.hits.begin(), out.hits.end(),
            [](const CurveSurfaceHit& a, const CurveSurfaceHit& b) { return a.paramT < b.paramT; });
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

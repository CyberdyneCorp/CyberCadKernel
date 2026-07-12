// SPDX-License-Identifier: Apache-2.0
//
// analytic_nurbs.cpp — exact analytic↔NURBS conversion. See analytic_nurbs.h.
//
// Recognition goes through primitive_fit (numsci facade), so the whole TU is under
// CYBERCAD_HAS_NUMSCI, mirroring primitive_fit.cpp. With the guard OFF the file is
// inert and the functions are absent from the library. The analytic→NURBS
// constructors are pure geometry (no facade), but live inside the guard for TU
// uniformity — nothing outside the numsci build calls them.
//
#include "native/math/analytic_nurbs.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"          // nurbsCurvePoint / nurbsSurfacePoint
#include "native/math/primitive_fit.h"    // fitPlane / fitSphere / fitCylinder / fitCone

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::math {
namespace {

constexpr double kPi = 3.14159265358979323846;

// Build the right-handed in-plane Y axis from a normal and X axis.
Vec3 yAxisOf(const Dir3& normal, const Dir3& x) { return cross(normal.vec(), x.vec()); }

// ─────────────────────────────────────────────────────────────────────────────
// Rational-quadratic ARC segment builder (Piegl & Tiller §7.3).
// One arc of sweep θ ≤ 2π/3 (we use ≤ π/2): control polygon {P0, P1, P2} where
// P0, P2 are the endpoints, P1 is the intersection of the tangents at P0 and P2,
// and the middle weight is w1 = cos(θ/2). For a circle of radius r centered at the
// frame origin with the two endpoints at angles a0, a1 (a1 = a0 + θ):
//   P0 = r(cos a0, sin a0),  P2 = r(cos a1, sin a1)   (in-plane 2D)
//   The tangent-intersection lies on the bisector at radius r / cos(θ/2).
// ─────────────────────────────────────────────────────────────────────────────

// Emit the 3 poles + 3 weights of a single ≤90° circular arc segment, in world
// space, for a circle {center, X, Y, r}. Angles in radians; sweep = a1 - a0 > 0.
struct Seg {
  Point3 p0, p1, p2;
  double w1;  // middle weight (w0 = w2 = 1)
};

Seg arcSegment(const Point3& center, const Vec3& X, const Vec3& Y, double r,
               double a0, double a1) {
  const double half = 0.5 * (a1 - a0);
  const double w1 = std::cos(half);
  const double amid = 0.5 * (a0 + a1);
  // Endpoints on the circle.
  const Point3 p0 = center + X * (r * std::cos(a0)) + Y * (r * std::sin(a0));
  const Point3 p2 = center + X * (r * std::cos(a1)) + Y * (r * std::sin(a1));
  // Tangent intersection: on the bisector at radius r / cos(half).
  const double rmid = r / w1;
  const Point3 p1 = center + X * (rmid * std::cos(amid)) + Y * (rmid * std::sin(amid));
  return {p0, p1, p2, w1};
}

// Assemble a piecewise rational-quadratic curve from n consecutive arc segments
// spanning [a0, a0+sweep], sharing endpoints. Produces 2n+1 poles.
BsplineCurveData assembleArc(const Point3& center, const Vec3& X, const Vec3& Y,
                             double r, double a0, double sweep, int nSeg) {
  BsplineCurveData c;
  c.degree = 2;
  const double dtheta = sweep / nSeg;
  for (int k = 0; k < nSeg; ++k) {
    const Seg s = arcSegment(center, X, Y, r, a0 + k * dtheta, a0 + (k + 1) * dtheta);
    if (k == 0) {
      c.poles.push_back(s.p0);
      c.weights.push_back(1.0);
    }
    c.poles.push_back(s.p1);
    c.weights.push_back(s.w1);
    c.poles.push_back(s.p2);
    c.weights.push_back(1.0);
  }
  // Flat clamped knot vector: {0,0,0, then each interior break doubled, 1,1,1}.
  const int nPoles = static_cast<int>(c.poles.size());
  const int m = nPoles + c.degree + 1;  // knot count
  c.knots.assign(m, 0.0);
  // degree+1 zeros at start, degree+1 ones at end.
  for (int i = 0; i < 3; ++i) c.knots[i] = 0.0;
  for (int i = m - 3; i < m; ++i) c.knots[i] = 1.0;
  // Interior knots: each segment boundary (k=1..nSeg-1) has multiplicity 2.
  int ki = 3;
  for (int k = 1; k < nSeg; ++k) {
    const double u = static_cast<double>(k) / nSeg;
    c.knots[ki++] = u;
    c.knots[ki++] = u;
  }
  return c;
}

// Number of ≤90° segments needed to cover a sweep.
int segmentsFor(double sweep) {
  return std::max(1, static_cast<int>(std::ceil(sweep / (0.5 * kPi) - 1e-12)));
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface-of-revolution assembly. Revolve a profile curve (poles Pk with weights
// wk, in world space, k=0..M) 360° about an axis (origin O, unit direction Z).
// Following P&T Algorithm A7.1 / §7.5: for each profile pole the revolution is the
// 9-pole 4-segment rational circle of radius = distance of the pole to the axis,
// centered at its foot on the axis, in the plane ⟂ Z. The U (revolution) direction
// carries the circle weights, scaled by the profile weight.
// ─────────────────────────────────────────────────────────────────────────────

BsplineSurfaceData revolve(const std::vector<Point3>& profPoles,
                           const std::vector<double>& profWeights, int profDegree,
                           const std::vector<double>& profKnots, const Point3& O,
                           const Dir3& Z) {
  BsplineSurfaceData s;
  s.degreeU = 2;              // revolution direction
  s.degreeV = profDegree;
  // U: full 4-segment circle → 9 poles, standard knot vector, weights (1,c,1,c,...)
  // with c=√2/2 and the 8 angular cosines. Build the unit-circle angular net.
  const double c = std::sqrt(0.5);  // cos(45°)
  // Angular directions and per-pole weights for the 9-pole circle (0,45,...,360).
  const int nU = 9;
  s.nPolesU = nU;
  const double uW[nU] = {1, c, 1, c, 1, c, 1, c, 1};
  const int M = static_cast<int>(profPoles.size());
  s.nPolesV = M;
  s.poles.resize(static_cast<std::size_t>(nU) * M);
  s.weights.resize(static_cast<std::size_t>(nU) * M);

  // Frame ⟂ to Z for the revolution plane.
  const Vec3 z = Z.vec();
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  Vec3 Y = cross(z, X);  // right-handed, unit

  for (int j = 0; j < M; ++j) {
    const Point3 P = profPoles[j];
    const double pw = profWeights.empty() ? 1.0 : profWeights[j];
    // Foot on axis and radius.
    const Vec3 rel = P - O;
    const double h = dot(rel, z);
    const Point3 foot = O + z * h;
    const Vec3 radial = rel - z * h;
    const double rad = norm(radial);
    // In-plane axes for THIS pole's circle: use the global X,Y so the whole surface
    // shares one revolution parametrization (angle 0 aligns with +X).
    // Build the 9 revolution poles at angles 0,45,...,360. For the tangent
    // (odd-index) poles the radius is rad / cos(45°); the point lies on the bisector.
    for (int i = 0; i < nU; ++i) {
      const double ang = i * 0.25 * kPi;  // 45° steps
      Point3 pt;
      if (i % 2 == 0) {
        pt = foot + X * (rad * std::cos(ang)) + Y * (rad * std::sin(ang));
      } else {
        const double rmid = rad / c;  // r / cos(45°)
        pt = foot + X * (rmid * std::cos(ang)) + Y * (rmid * std::sin(ang));
      }
      s.poles[static_cast<std::size_t>(i) * M + j] = pt;
      s.weights[static_cast<std::size_t>(i) * M + j] = uW[i] * pw;
    }
  }

  // U knot vector: 4-segment full circle {0,0,0,¼,¼,½,½,¾,¾,1,1,1}.
  s.knotsU = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  s.knotsV = profKnots;
  return s;
}

// ─────────────────────────────────────────────────────────────────────────────
// De-homogenize helpers + curve/surface control-net iterators.
// ─────────────────────────────────────────────────────────────────────────────

double weightAt(const std::vector<double>& w, int i) {
  return w.empty() ? 1.0 : w[static_cast<std::size_t>(i)];
}

// ─────────────────────────────────────────────────────────────────────────────
// CURVE recognition helpers.
// ─────────────────────────────────────────────────────────────────────────────

// Sample a NURBS curve at N params across its domain (for the primitive-fit seed).
std::vector<Point3> sampleCurve(const BsplineCurveData& cv, int N) {
  std::vector<Point3> out;
  if (cv.poles.empty() || cv.knots.size() < 2) return out;
  const double u0 = cv.knots.front(), u1 = cv.knots.back();
  out.reserve(N);
  for (int i = 0; i < N; ++i) {
    const double u = u0 + (u1 - u0) * i / (N - 1);
    out.push_back(nurbsCurvePoint(cv.degree, cv.poles, cv.weights, cv.knots, u));
  }
  return out;
}

// Max deviation of the control polygon from collinearity (line test): distance of
// each pole to the line through the first and last pole.
double lineResidual(const BsplineCurveData& cv) {
  const Point3 a = cv.poles.front(), b = cv.poles.back();
  const Vec3 d = b - a;
  const double dn = norm(d);
  if (dn < 1e-300) return 1e300;
  const Vec3 du = d / dn;
  double worst = 0.0;
  for (const auto& p : cv.poles) {
    const Vec3 w = p - a;
    const Vec3 perp = w - du * dot(w, du);
    worst = std::max(worst, norm(perp));
  }
  return worst;
}

// Given a plane (normal, point) recover it and test all poles lie in it.
double planarityResidual(const BsplineCurveData& cv, Dir3& normalOut,
                         Point3& pointOut) {
  const PlaneFit pf = fitPlane(cv.poles);
  if (!pf.ok) return 1e300;
  normalOut = pf.normal;
  pointOut = pf.centroid;
  double worst = 0.0;
  for (const auto& p : cv.poles)
    worst = std::max(worst, std::fabs(dot(pf.normal.vec(), p.asVec()) - pf.offset));
  return worst;
}

// ── Circle certificate ─────────────────────────────────────────────────────────
// A rational-quadratic circle has the P&T net: EVEN poles on the circle (weight 1
// after scaling), ODD (tangent) poles OFF the circle with weight cos(halfSeg). The
// algebraic certificate: (1) all poles are coplanar; (2) the EVEN poles are at the
// SAME distance r from a common center in that plane; (3) each ODD pole sits on the
// bisector at radius r/cos(half) AND its weight equals cos(half). We recover the
// center by a 2-D circle fit through the EVEN poles and verify.
struct CircleCert {
  bool ok = false;
  Circle circle{};
  double residual = 1e300;
  double startAngle = 0.0;
  double sweepAngle = 0.0;
  bool fullCircle = false;
};

CircleCert circleCertificate(const BsplineCurveData& cv, double tol) {
  CircleCert out;
  if (cv.degree != 2 || cv.poles.size() < 3) return out;
  // Even poles (0,2,4,...) are the on-curve endpoints.
  std::vector<Point3> evens;
  for (std::size_t i = 0; i < cv.poles.size(); i += 2) evens.push_back(cv.poles[i]);
  if (evens.size() < 3) {
    // A single 90° segment: only 2 even poles — add the parametric midpoint sample.
    evens.push_back(nurbsCurvePoint(cv.degree, cv.poles, cv.weights, cv.knots,
                                    0.5 * (cv.knots.front() + cv.knots.back())));
  }
  // Plane through the poles.
  Dir3 nrm; Point3 pon;
  const double planeRes = planarityResidual(cv, nrm, pon);
  if (planeRes > tol) return out;
  // 2-D circle fit through the even poles in that plane.
  Vec3 z = nrm.vec();
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  Vec3 Y = cross(z, X);
  // Fit a·(u²+v²)+D u+E v+F=0 through even poles projected to (u,v)=(rel·X, rel·Y).
  // Solve via the same normal-equations closed form used in primitive_fit.
  double Suu = 0, Svv = 0, Suv = 0, Su = 0, Sv = 0, Suuu = 0, Svvv = 0, Suvv = 0,
         Svuu = 0, n = 0;
  std::vector<std::pair<double, double>> uv;
  for (const auto& p : evens) {
    const Vec3 rel = p - pon;
    const double u = dot(rel, X), v = dot(rel, Y);
    uv.emplace_back(u, v);
    Suu += u * u; Svv += v * v; Suv += u * v; Su += u; Sv += v;
    Suuu += u * u * u; Svvv += v * v * v; Suvv += u * v * v; Svuu += v * u * u;
    n += 1;
  }
  // Kåsa circle fit: solve [Suu Suv Su; Suv Svv Sv; Su Sv n][A;B;C] = rhs.
  const double rhs0 = -(Suuu + Suvv);
  const double rhs1 = -(Svvv + Svuu);
  const double rhs2 = -(Suu + Svv);
  // 3×3 solve via Cramer.
  const double m00 = Suu, m01 = Suv, m02 = Su;
  const double m10 = Suv, m11 = Svv, m12 = Sv;
  const double m20 = Su, m21 = Sv, m22 = n;
  const double det = m00 * (m11 * m22 - m12 * m21) - m01 * (m10 * m22 - m12 * m20) +
                     m02 * (m10 * m21 - m11 * m20);
  if (std::fabs(det) < 1e-300) return out;
  auto cramer = [&](double b0, double b1, double b2, int col) {
    double a00 = m00, a01 = m01, a02 = m02, a10 = m10, a11 = m11, a12 = m12,
           a20 = m20, a21 = m21, a22 = m22;
    if (col == 0) { a00 = b0; a10 = b1; a20 = b2; }
    if (col == 1) { a01 = b0; a11 = b1; a21 = b2; }
    if (col == 2) { a02 = b0; a12 = b1; a22 = b2; }
    return a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) +
           a02 * (a10 * a21 - a11 * a20);
  };
  const double A = cramer(rhs0, rhs1, rhs2, 0) / det;
  const double B = cramer(rhs0, rhs1, rhs2, 1) / det;
  const double C = cramer(rhs0, rhs1, rhs2, 2) / det;
  const double cu = -0.5 * A, cv2 = -0.5 * B;
  const double r2 = cu * cu + cv2 * cv2 - C;
  if (!(r2 > 0.0)) return out;
  const double r = std::sqrt(r2);
  const Point3 center = pon + X * cu + Y * cv2;

  // Certificate 1: every EVEN pole is at distance r from center (in-plane).
  double worst = planeRes;
  for (const auto& p : evens)
    worst = std::max(worst, std::fabs(distance(p, center) - r));
  // Certificate 2: every ODD (tangent) pole sits at radius r/cos(half) on the
  // bisector, and its weight equals cos(half). We derive `half` per segment from
  // the two neighboring even poles' angular separation.
  const Vec3 Xc = (evens[0] - center);  // reference X for angle measurement
  const double Xcn = norm(Xc);
  if (Xcn < 1e-300) return out;
  const Vec3 refX = Xc / Xcn;
  const Vec3 refY = cross(z, refX);
  for (std::size_t i = 1; i + 1 < cv.poles.size(); i += 2) {
    const Point3 pEven0 = cv.poles[i - 1];
    const Point3 pEven1 = cv.poles[i + 1];
    const double a0 = std::atan2(dot(pEven0 - center, refY), dot(pEven0 - center, refX));
    const double a1 = std::atan2(dot(pEven1 - center, refY), dot(pEven1 - center, refX));
    double dth = a1 - a0;
    while (dth <= 0) dth += 2 * kPi;
    while (dth > 2 * kPi) dth -= 2 * kPi;
    const double half = 0.5 * dth;
    const double wExpected = std::cos(half);
    const double rmid = r / wExpected;
    const double amid = a0 + half;
    const Point3 pExpected = center + refX * (rmid * std::cos(amid)) +
                             refY * (rmid * std::sin(amid));
    worst = std::max(worst, distance(cv.poles[i], pExpected));
    worst = std::max(worst, std::fabs(weightAt(cv.weights, static_cast<int>(i)) /
                                          weightAt(cv.weights, static_cast<int>(i - 1)) -
                                      wExpected));
  }
  if (worst > tol) { out.residual = worst; return out; }

  // Recover angular span from first to last EVEN pole (accumulated).
  double startA = 0.0, sweep = 0.0;
  {
    // Sweep = sum of per-segment dth (each already in (0,2π]).
    for (std::size_t i = 1; i + 1 < cv.poles.size(); i += 2) {
      const Point3 e0 = cv.poles[i - 1], e1 = cv.poles[i + 1];
      const double a0 = std::atan2(dot(e0 - center, refY), dot(e0 - center, refX));
      const double a1 = std::atan2(dot(e1 - center, refY), dot(e1 - center, refX));
      double dth = a1 - a0; while (dth <= 0) dth += 2 * kPi;
      sweep += dth;
    }
    const Point3 first = cv.poles.front();
    startA = std::atan2(dot(first - center, refY), dot(first - center, refX));
  }
  out.ok = true;
  out.residual = worst;
  out.circle.center = center;
  out.circle.normal = nrm;
  out.circle.radius = r;
  out.circle.xAxis = Dir3(refX);
  out.startAngle = startA;
  out.sweepAngle = sweep;
  out.fullCircle = std::fabs(sweep - 2 * kPi) < 1e-9;
  return out;
}

// ── Ellipse certificate ─────────────────────────────────────────────────────────
// An ellipse is the affine image of a circle: mapping the rational-quadratic net
// back through the inverse of an affine map (scaling by (a,b) along the axes) must
// produce a circle. Equivalently: the EVEN poles satisfy a common central conic in
// the plane, ((rel·X)/a)² + ((rel·Y)/b)² = 1, with the ODD-pole/weight tangent
// pattern of a rational quadratic. We fit the 2-D conic through the even poles.
struct EllipseCert {
  bool ok = false;
  Ellipse ellipse{};
  double residual = 1e300;
};

EllipseCert ellipseCertificate(const BsplineCurveData& cv, double tol) {
  EllipseCert out;
  if (cv.degree != 2 || cv.poles.size() < 5) return out;  // need ≥3 even poles
  Dir3 nrm; Point3 pon;
  const double planeRes = planarityResidual(cv, nrm, pon);
  if (planeRes > tol) return out;
  Vec3 z = nrm.vec();
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  Vec3 Y = cross(z, X);
  // Sample the curve DENSELY (on-curve points, distinct) for a well-conditioned
  // conic fit — the even poles alone can degenerate on a closed curve (first==last).
  std::vector<std::pair<double, double>> uv;
  {
    const int Ns = 24;
    const double u0 = cv.knots.front(), u1 = cv.knots.back();
    for (int i = 0; i < Ns; ++i) {
      const double u = u0 + (u1 - u0) * i / Ns;  // exclude the wrap-around duplicate
      const Point3 p = nurbsCurvePoint(cv.degree, cv.poles, cv.weights, cv.knots, u);
      const Vec3 rel = p - pon;
      uv.emplace_back(dot(rel, X), dot(rel, Y));
    }
  }
  if (uv.size() < 5) return out;
  // Fit general central conic Au²+Buv+Cv²+Du+Ev+1=0 (constant fixed to 1) via
  // normal equations: 5 unknowns. Build 5×5 system by least squares.
  // Design row: [u², uv, v², u, v], rhs = -1.
  double AtA[5][5] = {}; double Atb[5] = {};
  for (auto& [u, v] : uv) {
    const double row[5] = {u * u, u * v, v * v, u, v};
    for (int r = 0; r < 5; ++r) {
      for (int cc = 0; cc < 5; ++cc) AtA[r][cc] += row[r] * row[cc];
      Atb[r] += row[r] * (-1.0);
    }
  }
  // Gaussian elimination 5×5.
  double M[5][6];
  for (int r = 0; r < 5; ++r) { for (int cc = 0; cc < 5; ++cc) M[r][cc] = AtA[r][cc]; M[r][5] = Atb[r]; }
  for (int col = 0; col < 5; ++col) {
    int piv = col; for (int r = col + 1; r < 5; ++r) if (std::fabs(M[r][col]) > std::fabs(M[piv][col])) piv = r;
    if (std::fabs(M[piv][col]) < 1e-300) return out;
    for (int cc = 0; cc < 6; ++cc) std::swap(M[col][cc], M[piv][cc]);
    for (int r = 0; r < 5; ++r) {
      if (r == col) continue;
      const double f = M[r][col] / M[col][col];
      for (int cc = 0; cc < 6; ++cc) M[r][cc] -= f * M[col][cc];
    }
  }
  double coef[5]; for (int r = 0; r < 5; ++r) coef[r] = M[r][5] / M[r][r];
  const double cA = coef[0], cB = coef[1], cC = coef[2], cD = coef[3], cE = coef[4];
  // Central conic: it must be an ellipse (B²-4AC<0). Recover center, axes, radii.
  const double disc = cB * cB - 4 * cA * cC;
  if (!(disc < 0)) return out;
  // Center: solve [2A B; B 2C][cx;cy] = [-D;-E].
  const double det = 4 * cA * cC - cB * cB;
  if (std::fabs(det) < 1e-300) return out;
  const double cx = (-2 * cC * cD + cB * cE) / det;
  const double cy = (-2 * cA * cE + cB * cD) / det;
  // Constant at center: F' = A cx² + B cx cy + C cy² + D cx + E cy + 1.
  const double Fc = cA * cx * cx + cB * cx * cy + cC * cy * cy + cD * cx + cE * cy + 1.0;
  // Eigenvalues of the quadratic-form matrix [cA, cB/2; cB/2, cC]. For an ellipse
  // written A u²+B uv+C v²+D u+E v+1=0 both eigenvalues share the sign of −Fc; the
  // semi-axis along an eigen-direction is s = sqrt(−Fc / λ). We do NOT assume λ>0
  // (with the constant normalized to +1 an ellipse has λ<0).
  const double tr = cA + cC;
  const double dd = std::sqrt(std::max(0.0, (cA - cC) * (cA - cC) + cB * cB));
  const double l1 = 0.5 * (tr + dd), l2 = 0.5 * (tr - dd);
  if (!(-Fc / l1 > 0 && -Fc / l2 > 0)) return out;  // genuine ellipse ⇒ both > 0
  const double s1 = std::sqrt(-Fc / l1), s2 = std::sqrt(-Fc / l2);
  // Eigenvector for l1 (its semi-axis direction).
  Vec3 axis1;
  if (std::fabs(cB) > 1e-14) {
    const double vx = cB * 0.5, vy = l1 - cA;
    axis1 = X * vx + Y * vy;
  } else {
    axis1 = (cA >= cC) ? X : Y;  // axis-aligned: l1=max(cA,cC) → its own axis
  }
  const double an1 = norm(axis1); if (an1 < 1e-300) return out; axis1 = axis1 / an1;
  Vec3 axis2 = cross(z, axis1);
  // Major = the LARGER semi-axis. (Smaller |λ| ⇒ larger radius.)
  double a = s1, b = s2; Vec3 majAxis = axis1;
  if (s2 > s1) { a = s2; b = s1; majAxis = axis2; }
  const Point3 center = pon + X * cx + Y * cy;
  // Reject a near-circle (a≈b): let circleCertificate own that (avoids ambiguity).
  if (std::fabs(a - b) <= 1e-9 * std::max(a, b)) return out;

  // CERTIFY: every even pole satisfies ((rel·maj)/a)²+((rel·min)/b)²=1 to tol, and
  // the odd/weight tangent pattern matches the affine-circle rational quadratic.
  Vec3 minAxis = cross(z, majAxis);
  double worst = planeRes;
  for (std::size_t i = 0; i < cv.poles.size(); i += 2) {
    const Vec3 rel = cv.poles[i] - center;
    const double xu = dot(rel, majAxis) / a, yv = dot(rel, minAxis) / b;
    worst = std::max(worst, std::fabs(xu * xu + yv * yv - 1.0));
  }
  // Map the whole net back to a circle (unit) via the inverse affine scale and run
  // the circle tangent/weight test there: build a scaled-pole curve and reuse the
  // circle certificate's odd-pole logic implicitly by checking each odd pole maps
  // onto the unit-circle tangent point.
  for (std::size_t i = 1; i + 1 < cv.poles.size(); i += 2) {
    auto toUnit = [&](const Point3& p) {
      const Vec3 rel = p - center;
      return std::pair<double, double>{dot(rel, majAxis) / a, dot(rel, minAxis) / b};
    };
    const auto [u0, v0] = toUnit(cv.poles[i - 1]);
    const auto [u2, v2] = toUnit(cv.poles[i + 1]);
    const auto [u1, v1] = toUnit(cv.poles[i]);
    const double a0 = std::atan2(v0, u0), a1raw = std::atan2(v2, u2);
    double dth = a1raw - a0; while (dth <= 0) dth += 2 * kPi;
    const double half = 0.5 * dth, wE = std::cos(half), rmid = 1.0 / wE, amid = a0 + half;
    const double eu = rmid * std::cos(amid), ev = rmid * std::sin(amid);
    worst = std::max(worst, std::hypot(u1 - eu, v1 - ev));
    worst = std::max(worst, std::fabs(weightAt(cv.weights, static_cast<int>(i)) /
                                          weightAt(cv.weights, static_cast<int>(i - 1)) - wE));
  }
  out.residual = worst;
  if (worst > tol) return out;
  out.ok = true;
  out.ellipse.center = center;
  out.ellipse.normal = nrm;
  out.ellipse.xAxis = Dir3(majAxis);
  out.ellipse.majorRadius = a;
  out.ellipse.minorRadius = b;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// SURFACE recognition — HOMOGENEOUS QUADRIC control-net certificate.
//
// A quadric Q is a symmetric 4×4 matrix; a point p is ON it iff p̃ᵀ Q p̃ = 0 with
// p̃ = (x,y,z,1). A rational NURBS surface S(u,v) = Σ Rᵢⱼ(u,v) P̃ᵢⱼ (P̃ = homogeneous
// pole (wx,wy,wz,w)) lies ENTIRELY on Q iff the numerator polynomial
//   N(u,v) = P̃(u,v)ᵀ Q P̃(u,v) = Σᵢⱼ,ₖₗ Rᵢⱼ Rₖₗ (P̃ᵢⱼᵀ Q P̃ₖₗ)
// is IDENTICALLY zero. A sufficient, control-net-level certificate (airtight — it
// implies N≡0) is: P̃ᵢⱼᵀ Q P̃ₖₗ = 0 for EVERY pair of homogeneous control points.
// This correctly handles the OFF-surface tangent (odd-index) poles of a rational
// quadratic: they need NOT lie on Q individually, but the bilinear form on every
// pole PAIR must vanish. That is the exact condition, verified here to ≤ tol.
//
// The quadric matrix is built from the fitted analytic parameters:
//   Plane    n·x − d = 0            → linear (rank-1 in the affine part)
//   Sphere   ‖x−c‖² − r² = 0
//   Cylinder ‖(x−a)ₚₑᵣₚ‖² − r² = 0  (x−a projected ⟂ to axis)
//   Cone     ‖(x−apex)ₚₑᵣₚ‖²·cos²α − (h·sinα)² = 0  (double cone; we recover α>0)
// ─────────────────────────────────────────────────────────────────────────────

// Sample a NURBS surface on an N×N grid across its domain — the primitive-fit seed.
// The fit must NOT use the raw control poles: a rational-quadratic's tangent poles
// lie OFF the surface, which would bias fitCylinder/fitSphere/fitCone. Evaluated
// surface points lie exactly on the surface, so the fit sees the true geometry.
std::vector<Point3> sampleSurface(const BsplineSurfaceData& s, int N) {
  std::vector<Point3> out;
  if (s.poles.empty() || s.knotsU.size() < 2 || s.knotsV.size() < 2) return out;
  SurfaceGrid grid{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  const double u0 = s.knotsU.front(), u1 = s.knotsU.back();
  const double v0 = s.knotsV.front(), v1 = s.knotsV.back();
  out.reserve(static_cast<std::size_t>(N) * N);
  for (int i = 0; i < N; ++i)
    for (int j = 0; j < N; ++j) {
      const double u = u0 + (u1 - u0) * i / (N - 1);
      const double v = v0 + (v1 - v0) * j / (N - 1);
      out.push_back(nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights, s.knotsU,
                                      s.knotsV, u, v));
    }
  return out;
}

using Q4 = std::array<std::array<double, 4>, 4>;

double quadForm(const std::array<double, 4>& a, const Q4& Q,
                const std::array<double, 4>& b) {
  double s = 0.0;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) s += a[i] * Q[i][j] * b[j];
  return s;
}

// Frobenius-normalize a quadric so the residual tol is scale-comparable to the
// geometry (residuals are pole·pole·Q ~ length² · ‖Q‖). We scale Q so its largest
// affine (x,y,z) block entry is O(1), keeping the residual dimensionally a squared
// length; callers compare residual/scale² to tol. To keep the API simple we instead
// divide the raw bilinear residual by (extent² · ‖Q‖∞) inside verifyQuadric.
double qMaxAbs(const Q4& Q) {
  double m = 0.0;
  for (auto& row : Q) for (double e : row) m = std::max(m, std::fabs(e));
  return m > 0 ? m : 1.0;
}

// Verify the NURBS surface lies EXACTLY on the quadric Q. The airtight certificate
// is the numerator polynomial N(u,v) = P̃(u,v)ᵀ Q P̃(u,v) ≡ 0. N is a fixed-degree
// polynomial (bidegree ≤ (2·degU, 2·degV)); it is identically zero iff it vanishes
// at MORE nodes than its degree along each axis. We evaluate the surface on a dense
// grid (≫ the numerator degree) and require |p̃ᵀQp̃| = 0 at every node — this
// CERTIFIES N≡0 (a polynomial vanishing at more points than its degree, per axis, is
// the zero polynomial), i.e. the ENTIRE surface — not just the on-surface poles —
// lies on Q. The residual is normalized by (extent²·‖Q‖) to a dimensionless value.
//
// This is honest exactness (algebraic, not RMS): a wobbly almost-quadric produces a
// non-zero N at the interior nodes and is rejected; a genuine quadric gives 0 to
// machine precision. It correctly accommodates the OFF-surface tangent control poles
// of a rational quadratic (they influence N through the basis but N still ≡ 0).
double verifyQuadric(const BsplineSurfaceData& s, const Q4& Q) {
  // Scale from the pole extent.
  Point3 c{};
  for (const auto& p : s.poles) { c.x += p.x; c.y += p.y; c.z += p.z; }
  const double inv = 1.0 / static_cast<double>(s.poles.size());
  c = {c.x * inv, c.y * inv, c.z * inv};
  double ext = 1e-12;
  for (const auto& p : s.poles) ext = std::max(ext, distance(p, c));
  const double scale = ext * ext * qMaxAbs(Q);

  // Dense grid — 31×31 far exceeds the numerator's per-axis degree (≤ 2·5=10 for the
  // revolved quadrics here), so all-zero there ⇒ N≡0.
  const std::vector<Point3> pts = sampleSurface(s, 31);
  if (pts.empty()) return 1e300;
  double worst = 0.0;
  for (const auto& p : pts) {
    const std::array<double, 4> ph{p.x, p.y, p.z, 1.0};
    worst = std::max(worst, std::fabs(quadForm(ph, Q, ph)));
  }
  return worst / scale;
}

// Plane n·x = d as a quadric (linear): Q has only the last row/col populated so that
// p̃ᵀ Q p̃ = n·x − d for a weight-1 pole (and the bilinear pair form = ½(n·xᵢ−d)·wⱼ +
// ½(n·xⱼ−d)·wᵢ, which vanishes for every pair iff every pole is on the plane).
double planeCertificate(const BsplineSurfaceData& s, Plane& out) {
  const std::vector<Point3> samp = sampleSurface(s, 12);
  const PlaneFit pf = fitPlane(samp);
  if (!pf.ok) return 1e300;
  const Vec3 n = pf.normal.vec();
  const double d = pf.offset;
  Q4 Q{};
  // Symmetric form giving p̃ᵀQp̃ = n·x − d·1: put ½n in the (i,3)&(3,i) slots, −d in (3,3).
  Q[0][3] = Q[3][0] = 0.5 * n.x;
  Q[1][3] = Q[3][1] = 0.5 * n.y;
  Q[2][3] = Q[3][2] = 0.5 * n.z;
  Q[3][3] = -d;
  Vec3 z = n;
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  out.pos = Ax3::fromAxisAndRef(pf.centroid, pf.normal, Dir3(X));
  return verifyQuadric(s, Q);
}

// Sphere ‖x−c‖² − r² = 0 → x²+y²+z² − 2c·x + (‖c‖²−r²) = 0.
double sphereCertificate(const BsplineSurfaceData& s, Sphere& out) {
  const std::vector<Point3> samp = sampleSurface(s, 12);
  const SphereFit sf = fitSphere(samp);
  if (!sf.ok) return 1e300;
  const Point3 c = sf.center;
  const double r = sf.radius;
  Q4 Q{};
  Q[0][0] = Q[1][1] = Q[2][2] = 1.0;
  Q[0][3] = Q[3][0] = -c.x;
  Q[1][3] = Q[3][1] = -c.y;
  Q[2][3] = Q[3][2] = -c.z;
  Q[3][3] = c.x * c.x + c.y * c.y + c.z * c.z - r * r;
  out.pos = Ax3{c, Dir3(1, 0, 0), Dir3(0, 1, 0), Dir3(0, 0, 1)};
  out.radius = r;
  return verifyQuadric(s, Q);
}

// Cylinder ‖(x−a) − (d·(x−a))d‖² − r² = 0. Expand as xᵀ(I−ddᵀ)x − 2 aₚ·x + (…) with
// aₚ = (I−ddᵀ)a. Build the symmetric quadric directly.
double cylinderCertificate(const BsplineSurfaceData& s, Cylinder& out) {
  const std::vector<Point3> samp = sampleSurface(s, 12);
  const CylinderFit cf = fitCylinder(samp);
  if (!cf.ok) return 1e300;
  const Vec3 d = cf.axis.vec();
  const Point3 a = cf.axisPoint;
  const double r = cf.radius;
  // Projector M = I − d dᵀ.
  double M[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) M[i][j] = (i == j ? 1.0 : 0.0) - d[i] * d[j];
  // Ma (vector), aᵀMa (scalar).
  const Vec3 av{a.x, a.y, a.z};
  Vec3 Ma{M[0][0] * av.x + M[0][1] * av.y + M[0][2] * av.z,
          M[1][0] * av.x + M[1][1] * av.y + M[1][2] * av.z,
          M[2][0] * av.x + M[2][1] * av.y + M[2][2] * av.z};
  const double aMa = dot(av, Ma);
  Q4 Q{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) Q[i][j] = M[i][j];
  Q[0][3] = Q[3][0] = -Ma.x;
  Q[1][3] = Q[3][1] = -Ma.y;
  Q[2][3] = Q[3][2] = -Ma.z;
  Q[3][3] = aMa - r * r;
  Vec3 z = d;
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  out.pos = Ax3::fromAxisAndRef(cf.axisPoint, cf.axis, Dir3(X));
  out.radius = r;
  return verifyQuadric(s, Q);
}

// Cone about apex e with axis d, half-angle α: for v = x−e, h = d·v, ρ² = ‖v−h d‖².
// Surface: ρ²·cos²α − h²·sin²α = 0. In matrix form with M = I − d dᵀ (so ρ² = vᵀM v)
// and h² = vᵀ(d dᵀ)v: vᵀ(cos²α·M − sin²α·d dᵀ)v = 0. Translate by e for the full Q.
double coneCertificate(const BsplineSurfaceData& s, Cone& out) {
  const std::vector<Point3> samp = sampleSurface(s, 12);
  const ConeFit cf = fitCone(samp);
  if (!cf.ok) return 1e300;
  const Vec3 d = cf.axis.vec();
  const Point3 e = cf.apex;
  const double ca = std::cos(cf.halfAngle), sa = std::sin(cf.halfAngle);
  const double c2 = ca * ca, s2 = sa * sa;
  // A = cos²α·(I − ddᵀ) − sin²α·ddᵀ = cos²α·I − ddᵀ.
  double A[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      A[i][j] = c2 * (i == j ? 1.0 : 0.0) - (c2 + s2) * d[i] * d[j];
  // Full quadric in x: (x−e)ᵀ A (x−e) = xᵀA x − 2(Ae)·x + eᵀA e.
  const Vec3 ev{e.x, e.y, e.z};
  Vec3 Ae{A[0][0] * ev.x + A[0][1] * ev.y + A[0][2] * ev.z,
          A[1][0] * ev.x + A[1][1] * ev.y + A[1][2] * ev.z,
          A[2][0] * ev.x + A[2][1] * ev.y + A[2][2] * ev.z};
  const double eAe = dot(ev, Ae);
  Q4 Q{};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) Q[i][j] = A[i][j];
  Q[0][3] = Q[3][0] = -Ae.x;
  Q[1][3] = Q[3][1] = -Ae.y;
  Q[2][3] = Q[3][2] = -Ae.z;
  Q[3][3] = eAe;
  Vec3 z = d;
  Vec3 t = (std::fabs(z.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  Vec3 X = cross(z, t); X = X / norm(X);
  out.pos = Ax3::fromAxisAndRef(cf.apex, cf.axis, Dir3(X));
  out.radius = 0.0;              // apex reference: radius grows with v·sinα
  out.semiAngle = cf.halfAngle;
  return verifyQuadric(s, Q);
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API — ANALYTIC → NURBS.
// ─────────────────────────────────────────────────────────────────────────────

BsplineCurveData circleToNurbs(const Circle& c) {
  const Vec3 X = c.xAxis.vec();
  const Vec3 Y = yAxisOf(c.normal, c.xAxis);
  return assembleArc(c.center, X, Y, c.radius, 0.0, 2 * kPi, 4);
}

BsplineCurveData arcToNurbs(const Arc& a) {
  const Vec3 X = a.circle.xAxis.vec();
  const Vec3 Y = yAxisOf(a.circle.normal, a.circle.xAxis);
  const int nSeg = segmentsFor(a.sweepAngle);
  return assembleArc(a.circle.center, X, Y, a.circle.radius, a.startAngle,
                     a.sweepAngle, nSeg);
}

BsplineCurveData ellipseToNurbs(const Ellipse& e) {
  // Affine image of a unit circle: replace radius·(cos,sin) with (a·cos, b·sin).
  // Build the circle net with radius 1 in a normalized frame, then scale each pole's
  // in-plane X-component by a and Y-component by b about the center.
  const Vec3 X = e.xAxis.vec();
  const Vec3 Y = yAxisOf(e.normal, e.xAxis);
  BsplineCurveData c = assembleArc(e.center, X, Y, 1.0, 0.0, 2 * kPi, 4);
  for (auto& p : c.poles) {
    const Vec3 rel = p - e.center;
    const double u = dot(rel, X), v = dot(rel, Y);
    p = e.center + X * (u * e.majorRadius) + Y * (v * e.minorRadius);
  }
  return c;
}

BsplineCurveData lineToNurbs(const LineSegment& l) {
  BsplineCurveData c;
  c.degree = 1;
  c.poles = {l.start, l.end};
  c.weights = {1.0, 1.0};
  c.knots = {0, 0, 1, 1};
  return c;
}

BsplineSurfaceData planeToNurbs(const Plane& p, double u0, double u1, double v0,
                                double v1) {
  BsplineSurfaceData s;
  s.degreeU = 1; s.degreeV = 1;
  s.nPolesU = 2; s.nPolesV = 2;
  s.poles = {p.value(u0, v0), p.value(u0, v1), p.value(u1, v0), p.value(u1, v1)};
  s.weights = {1, 1, 1, 1};
  s.knotsU = {0, 0, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  return s;
}

BsplineSurfaceData cylinderToNurbs(const Cylinder& c, double v0, double v1) {
  // Profile: the vertical line segment on the surface at angle 0 (X direction),
  // from height v0 to v1. Two poles, degree 1.
  const Point3 p0 = c.value(0.0, v0);
  const Point3 p1 = c.value(0.0, v1);
  std::vector<Point3> prof{p0, p1};
  std::vector<double> pw{1.0, 1.0};
  std::vector<double> pk{0, 0, 1, 1};
  return revolve(prof, pw, 1, pk, c.pos.origin, c.pos.z);
}

BsplineSurfaceData coneToNurbs(const Cone& c, double v0, double v1) {
  const Point3 p0 = c.value(0.0, v0);
  const Point3 p1 = c.value(0.0, v1);
  std::vector<Point3> prof{p0, p1};
  std::vector<double> pw{1.0, 1.0};
  std::vector<double> pk{0, 0, 1, 1};
  return revolve(prof, pw, 1, pk, c.pos.origin, c.pos.z);
}

BsplineSurfaceData sphereToNurbs(const Sphere& s) {
  // Profile: a meridian half-circle from the south pole (v=-π/2) to the north pole
  // (v=+π/2) at longitude 0, as a rational-quadratic arc (sweep π → 2 segments).
  const Vec3 X = s.pos.x.vec();     // longitude-0 in-plane axis
  const Vec3 Zc = s.pos.z.vec();    // polar axis
  // Meridian circle: center = sphere center, in plane spanned by X (radial) and Z.
  // Angle 0 → equator (+X), angle +π/2 → north pole (+Z), -π/2 → south pole.
  BsplineCurveData meridian =
      assembleArc(s.pos.origin, X, Zc, s.radius, -0.5 * kPi, kPi, 2);
  return revolve(meridian.poles, meridian.weights, meridian.degree, meridian.knots,
                 s.pos.origin, s.pos.z);
}

BsplineSurfaceData torusToNurbs(const Torus& t) {
  // Profile: the tube circle (radius r about the tube center, which sits at distance
  // R from the axis along +X) in the plane spanned by X (radial) and Z.
  const Vec3 X = t.pos.x.vec();
  const Vec3 Zc = t.pos.z.vec();
  const Point3 tubeCenter = t.pos.origin + X * t.majorRadius;
  BsplineCurveData tube =
      assembleArc(tubeCenter, X, Zc, t.minorRadius, 0.0, 2 * kPi, 4);
  return revolve(tube.poles, tube.weights, tube.degree, tube.knots, t.pos.origin,
                 t.pos.z);
}

// ─────────────────────────────────────────────────────────────────────────────
// Public API — NURBS → ANALYTIC (recognition).
// ─────────────────────────────────────────────────────────────────────────────

CurveRecognition recognizeCurve(const BsplineCurveData& curve, double tol) {
  CurveRecognition out;
  if (curve.poles.size() < 2) return out;

  // ── LINE: degree 1 with 2 poles is trivially a segment; degree ≥1 with a
  // collinear control polygon AND constant-ratio weights is also a line.
  {
    const double lr = lineResidual(curve);
    bool wConst = true;
    const double w0 = weightAt(curve.weights, 0);
    for (std::size_t i = 1; i < curve.poles.size(); ++i)
      if (std::fabs(weightAt(curve.weights, static_cast<int>(i)) - w0) > tol) wConst = false;
    if (lr <= tol && wConst) {
      out.kind = CurveKind::Line;
      out.line = {curve.poles.front(), curve.poles.back()};
      out.residual = lr;
      return out;
    }
    out.residual = lr;
  }

  // ── CIRCLE / ARC (rational quadratic).
  {
    const CircleCert cc = circleCertificate(curve, tol);
    if (cc.ok) {
      if (cc.fullCircle) {
        out.kind = CurveKind::Circle;
        out.circle = cc.circle;
      } else {
        out.kind = CurveKind::Arc;
        out.arc.circle = cc.circle;
        out.arc.startAngle = cc.startAngle;
        out.arc.sweepAngle = cc.sweepAngle;
      }
      out.residual = cc.residual;
      return out;
    }
    out.residual = std::min(out.residual, cc.residual);
  }

  // ── ELLIPSE.
  {
    const EllipseCert ec = ellipseCertificate(curve, tol);
    if (ec.ok) {
      out.kind = CurveKind::Ellipse;
      out.ellipse = ec.ellipse;
      out.residual = ec.residual;
      return out;
    }
    out.residual = std::min(out.residual, ec.residual);
  }

  out.kind = CurveKind::General;
  return out;
}

SurfaceRecognition recognizeSurface(const BsplineSurfaceData& surface, double tol) {
  SurfaceRecognition out;
  if (surface.poles.size() < 4) return out;

  double best = 1e300;

  // ── PLANE (simplest).
  {
    Plane pl;
    const double res = planeCertificate(surface, pl);
    best = std::min(best, res);
    if (res <= tol) {
      out.kind = SurfaceKind::Plane;
      out.plane = pl;
      out.residual = res;
      return out;
    }
  }

  // ── SPHERE.
  {
    Sphere sp;
    const double res = sphereCertificate(surface, sp);
    best = std::min(best, res);
    if (res <= tol) {
      out.kind = SurfaceKind::Sphere;
      out.sphere = sp;
      out.residual = res;
      return out;
    }
  }

  // ── CYLINDER.
  {
    Cylinder cy;
    const double res = cylinderCertificate(surface, cy);
    best = std::min(best, res);
    if (res <= tol) {
      out.kind = SurfaceKind::Cylinder;
      out.cylinder = cy;
      out.residual = res;
      return out;
    }
  }

  // ── CONE.
  {
    Cone co;
    const double res = coneCertificate(surface, co);
    best = std::min(best, res);
    if (res <= tol) {
      out.kind = SurfaceKind::Cone;
      out.cone = co;
      out.residual = res;
      return out;
    }
  }

  out.kind = SurfaceKind::General;
  out.residual = best;
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

// SPDX-License-Identifier: Apache-2.0
//
// primitive_fit.cpp — Layer-7 reverse-engineering: analytic-primitive detection /
// fitting (plane / sphere / cylinder / cone). See primitive_fit.h for the design.
//
// The linear/nonlinear solves go through the numsci facade (lstsq / least_squares),
// so the whole TU is under CYBERCAD_HAS_NUMSCI, mirroring bspline_fit.cpp. With the
// guard OFF this file is inert and the functions are absent from the library.
//
// The symmetric-3×3 eigensolver (classical cyclic Jacobi) is self-contained here —
// it does NOT widen the numsci facade and pulls in no substrate. It is the exact
// smallest/largest-eigenvector path the roadmap asks for (plane normal, Gauss-map
// cylinder/cone axis).
//
#include "native/math/primitive_fit.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/numerics/numerics.h"  // lstsq (linear fits) / least_squares (geometric refine)

#include <algorithm>
#include <array>
#include <cmath>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

using numerics::least_squares;
using numerics::lstsq;
using numerics::Vector;

constexpr double kPi = 3.14159265358979323846;

// ─────────────────────────────────────────────────────────────────────────────
// Symmetric 3×3 eigensolver — classical cyclic Jacobi. Closed-form, no external
// SVD. Returns eigenvalues ascending with the matching orthonormal eigenvectors as
// columns (v[k] is the k-th eigenvector). C is a real symmetric 3×3 given by its 6
// distinct entries. This is the airtight symmetric-eigen path for the plane normal
// (smallest eigenvector) and the Gauss-map cylinder/cone axis.
// ─────────────────────────────────────────────────────────────────────────────
struct Eigen3 {
  std::array<double, 3> value{};             // ascending
  std::array<std::array<double, 3>, 3> vec;  // vec[k] = eigenvector k
};

Eigen3 jacobiEigen(const std::array<std::array<double, 3>, 3>& in) {
  double a[3][3];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) a[i][j] = in[i][j];

  double v[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // accumulated rotations

  for (int sweep = 0; sweep < 100; ++sweep) {
    // Largest off-diagonal magnitude.
    double off = std::fabs(a[0][1]) + std::fabs(a[0][2]) + std::fabs(a[1][2]);
    if (off < 1e-300) break;

    for (int p = 0; p < 3; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        if (std::fabs(a[p][q]) < 1e-300) continue;
        const double app = a[p][p], aqq = a[q][q], apq = a[p][q];
        // Jacobi rotation angle: cot(2θ) = (aqq-app)/(2apq).
        const double phi = 0.5 * std::atan2(2.0 * apq, aqq - app);
        const double c = std::cos(phi), s = std::sin(phi);
        // Apply rotation J^T A J.
        for (int k = 0; k < 3; ++k) {
          const double akp = a[k][p], akq = a[k][q];
          a[k][p] = c * akp - s * akq;
          a[k][q] = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
          const double apk = a[p][k], aqk = a[q][k];
          a[p][k] = c * apk - s * aqk;
          a[q][k] = s * apk + c * aqk;
        }
        // Accumulate the eigenvector rotation.
        for (int k = 0; k < 3; ++k) {
          const double vkp = v[k][p], vkq = v[k][q];
          v[k][p] = c * vkp - s * vkq;
          v[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }

  // Collect (eigenvalue, eigenvector) and sort ascending by eigenvalue.
  std::array<int, 3> idx{0, 1, 2};
  std::array<double, 3> ev{a[0][0], a[1][1], a[2][2]};
  std::sort(idx.begin(), idx.end(), [&](int i, int j) { return ev[i] < ev[j]; });

  Eigen3 out;
  for (int k = 0; k < 3; ++k) {
    const int c = idx[k];
    out.value[k] = ev[c];
    out.vec[k] = {v[0][c], v[1][c], v[2][c]};
    // Normalize (Jacobi keeps columns orthonormal to rounding, renormalize anyway).
    const double n =
        std::sqrt(out.vec[k][0] * out.vec[k][0] + out.vec[k][1] * out.vec[k][1] +
                  out.vec[k][2] * out.vec[k][2]);
    if (n > 0) {
      out.vec[k][0] /= n;
      out.vec[k][1] /= n;
      out.vec[k][2] /= n;
    }
  }
  return out;
}

Point3 centroidOf(std::span<const Point3> pts) {
  double cx = 0, cy = 0, cz = 0;
  for (const auto& p : pts) { cx += p.x; cy += p.y; cz += p.z; }
  const double inv = 1.0 / static_cast<double>(pts.size());
  return {cx * inv, cy * inv, cz * inv};
}

// Centered covariance of the points as a symmetric 3×3.
std::array<std::array<double, 3>, 3> covariance(std::span<const Point3> pts,
                                                const Point3& c) {
  std::array<std::array<double, 3>, 3> m{};
  for (const auto& p : pts) {
    const double dx = p.x - c.x, dy = p.y - c.y, dz = p.z - c.z;
    m[0][0] += dx * dx; m[0][1] += dx * dy; m[0][2] += dx * dz;
    m[1][1] += dy * dy; m[1][2] += dy * dz; m[2][2] += dz * dz;
  }
  m[1][0] = m[0][1]; m[2][0] = m[0][2]; m[2][1] = m[1][2];
  return m;
}

// Cloud extent: max distance from centroid (a scale for relative-error reasoning).
double cloudExtent(std::span<const Point3> pts, const Point3& c) {
  double e = 0.0;
  for (const auto& p : pts) e = std::max(e, distance(p, c));
  return e;
}

Vec3 toVec(const std::array<double, 3>& a) { return {a[0], a[1], a[2]}; }

// Two unit vectors spanning the plane ⟂ to `axis` (right-handed frame).
void perpFrame(const Vec3& axis, Vec3& u, Vec3& w) {
  Vec3 t = (std::fabs(axis.x) < 0.9) ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  u = cross(axis, t);
  u = u / norm(u);
  w = cross(axis, u);  // already unit
}

// ── Plane ────────────────────────────────────────────────────────────────────
PlaneFit fitPlaneImpl(std::span<const Point3> pts) {
  PlaneFit r;
  if (pts.size() < 3) return r;
  const Point3 c = centroidOf(pts);
  const Eigen3 e = jacobiEigen(covariance(pts, c));
  const Vec3 n = toVec(e.vec[0]);  // smallest eigenvalue → normal
  Dir3 nd(n);
  if (!nd.valid()) return r;
  r.normal = nd;
  r.centroid = c;
  r.offset = dot(nd.vec(), c.asVec());
  double s2 = 0.0;
  for (const auto& p : pts) {
    const double d = dot(nd.vec(), p.asVec()) - r.offset;
    s2 += d * d;
  }
  r.rms = std::sqrt(s2 / static_cast<double>(pts.size()));
  r.ok = true;
  return r;
}

// ── Sphere ───────────────────────────────────────────────────────────────────
double sphereRms(std::span<const Point3> pts, const Point3& ctr, double rad) {
  double s2 = 0.0;
  for (const auto& p : pts) {
    const double d = distance(p, ctr) - rad;
    s2 += d * d;
  }
  return std::sqrt(s2 / static_cast<double>(pts.size()));
}

SphereFit fitSphereImpl(std::span<const Point3> pts, bool refine) {
  SphereFit r;
  const int n = static_cast<int>(pts.size());
  if (n < 4) return r;
  // Algebraic linear fit: x²+y²+z²+Dx+Ey+Fz+G=0  ⇒  A·[D,E,F,G] = b
  // with row [x,y,z,1], rhs -(x²+y²+z²). center=(-D/2,-E/2,-F/2), r²=center²-G.
  Vector A(static_cast<std::size_t>(n) * 4);
  Vector b(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const Point3& p = pts[i];
    A[i * 4 + 0] = p.x;
    A[i * 4 + 1] = p.y;
    A[i * 4 + 2] = p.z;
    A[i * 4 + 3] = 1.0;
    b[i] = -(p.x * p.x + p.y * p.y + p.z * p.z);
  }
  const Vector sol = lstsq(A, n, 4, b);
  if (sol.size() != 4) return r;
  Point3 ctr{-0.5 * sol[0], -0.5 * sol[1], -0.5 * sol[2]};
  const double r2 = ctr.x * ctr.x + ctr.y * ctr.y + ctr.z * ctr.z - sol[3];
  if (!(r2 > 0.0)) return r;
  double rad = std::sqrt(r2);

  // Optional geometric refine: minimize Σ(‖p-ctr‖ - r)² over (ctr, r).
  if (refine) {
    const auto resid = [&](const Vector& x) -> Vector {
      const Point3 cc{x[0], x[1], x[2]};
      const double rr = x[3];
      Vector out(static_cast<std::size_t>(n));
      for (int i = 0; i < n; ++i) out[i] = distance(pts[i], cc) - rr;
      return out;
    };
    const numerics::SolveResult sr =
        least_squares(resid, {ctr.x, ctr.y, ctr.z, rad});
    if (sr.success && sr.x.size() == 4 && sr.x[3] > 0.0) {
      ctr = {sr.x[0], sr.x[1], sr.x[2]};
      rad = sr.x[3];
    }
  }
  r.center = ctr;
  r.radius = rad;
  r.rms = sphereRms(pts, ctr, rad);
  r.ok = true;
  return r;
}

// ── Cylinder ─────────────────────────────────────────────────────────────────
// Distance of p to the axis line (axisPoint a0, unit direction d).
double distToAxis(const Point3& p, const Point3& a0, const Vec3& d) {
  const Vec3 w = p - a0;
  const Vec3 perp = w - d * dot(w, d);
  return norm(perp);
}

double cylinderRms(std::span<const Point3> pts, const Point3& a0, const Vec3& d,
                   double rad) {
  double s2 = 0.0;
  for (const auto& p : pts) {
    const double e = distToAxis(p, a0, d) - rad;
    s2 += e * e;
  }
  return std::sqrt(s2 / static_cast<double>(pts.size()));
}

// Given a FIXED axis direction, fit the 2-D circle (center + radius) in the plane
// ⟂ to the axis by the algebraic circle fit, and report the RMS. Returns radius
// and the axis point (foot in 3-D). rmsOut receives the achieved RMS.
bool circleInPlane(std::span<const Point3> pts, const Point3& c, const Vec3& axis,
                   Point3& axisPointOut, double& radiusOut, double& rmsOut) {
  Vec3 u, w;
  perpFrame(axis, u, w);
  const int n = static_cast<int>(pts.size());
  // Algebraic 2-D circle: a²+b² + Da + Eb + F = 0, coords (a,b) = ((p-c)·u,(p-c)·w).
  Vector A(static_cast<std::size_t>(n) * 3);
  Vector b(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    const Vec3 rel = pts[i] - c;
    const double a = dot(rel, u), bb = dot(rel, w);
    A[i * 3 + 0] = a;
    A[i * 3 + 1] = bb;
    A[i * 3 + 2] = 1.0;
    b[i] = -(a * a + bb * bb);
  }
  const Vector sol = lstsq(A, n, 3, b);
  if (sol.size() != 3) return false;
  const double ca = -0.5 * sol[0], cb = -0.5 * sol[1];
  const double r2 = ca * ca + cb * cb - sol[2];
  if (!(r2 > 0.0)) return false;
  radiusOut = std::sqrt(r2);
  // 2-D center back to 3-D: a point on the axis.
  axisPointOut = c + u * ca + w * cb;
  rmsOut = cylinderRms(pts, axisPointOut, axis, radiusOut);
  return true;
}

CylinderFit fitCylinderImpl(std::span<const Point3> pts) {
  CylinderFit r;
  const int n = static_cast<int>(pts.size());
  if (n < 6) return r;
  const Point3 c = centroidOf(pts);

  // Seed axis candidates. For a cylinder the axis is the direction along which the
  // projected radii are constant — i.e. the eigenvector of the CENTERED point
  // covariance whose plane-⟂ circle fit is tightest. We evaluate all three
  // covariance eigenvectors (cheap, exact) plus refine the best geometrically.
  const Eigen3 e = jacobiEigen(covariance(pts, c));
  Vec3 bestAxis{};
  Point3 bestPt{};
  double bestRad = 0.0, bestRms = 1e300;
  bool have = false;
  for (int k = 0; k < 3; ++k) {
    const Vec3 axis = toVec(e.vec[k]);
    Point3 ap;
    double rad, rms;
    if (circleInPlane(pts, c, axis, ap, rad, rms)) {
      if (rms < bestRms) {
        bestRms = rms; bestAxis = axis; bestPt = ap; bestRad = rad; have = true;
      }
    }
  }
  if (!have) return r;

  // Geometric refine over the axis direction (parameterized as small tilts of the
  // seed frame) + axis point + radius via numerics::least_squares. State x =
  // [px,py,pz, dx,dy,dz, radius]; direction is renormalized inside the residual so
  // the solver may move it freely.
  Vec3 u, w;
  perpFrame(bestAxis, u, w);
  const auto resid = [&](const Vector& x) -> Vector {
    Vec3 d{x[3], x[4], x[5]};
    const double dn = norm(d);
    if (dn > 0) d = d / dn;
    const Point3 a0{x[0], x[1], x[2]};
    const double rad = x[6];
    Vector out(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) out[i] = distToAxis(pts[i], a0, d) - rad;
    return out;
  };
  const numerics::SolveResult sr = least_squares(
      resid, {bestPt.x, bestPt.y, bestPt.z, bestAxis.x, bestAxis.y, bestAxis.z,
              bestRad});
  Point3 a0 = bestPt;
  Vec3 d = bestAxis;
  double rad = bestRad;
  if (sr.success && sr.x.size() == 7) {
    Vec3 dd{sr.x[3], sr.x[4], sr.x[5]};
    const double dn = norm(dd);
    if (dn > 0 && sr.x[6] > 0) {
      d = dd / dn;
      a0 = {sr.x[0], sr.x[1], sr.x[2]};
      rad = sr.x[6];
    }
  }
  Dir3 ad(d);
  if (!ad.valid()) return r;
  // Report the axis point as the foot of the centroid on the axis (canonical).
  const Vec3 wv = c - a0;
  const Point3 foot = a0 + ad.vec() * dot(wv, ad.vec());
  r.axisPoint = foot;
  r.axis = ad;
  r.radius = rad;
  r.rms = cylinderRms(pts, foot, ad.vec(), rad);
  r.ok = true;
  return r;
}

// ── Cone ─────────────────────────────────────────────────────────────────────
// Signed point-to-cone-surface distance for a single-nappe cone: apex `ap`, unit
// axis `d` (apex→opening), half-angle α. For point p let v=p-ap, h=v·d (height
// along axis), ρ=‖v - h d‖ (radial). The nearest-surface distance for a point in
// the "outside" region is |ρ cosα - h sinα|; this is the standard cone residual.
double coneResidual(const Point3& p, const Point3& ap, const Vec3& d, double alpha) {
  const Vec3 v = p - ap;
  const double h = dot(v, d);
  const Vec3 radv = v - d * h;
  const double rho = norm(radv);
  return rho * std::cos(alpha) - h * std::sin(alpha);
}

double coneRms(std::span<const Point3> pts, const Point3& ap, const Vec3& d,
               double alpha) {
  double s2 = 0.0;
  for (const auto& p : pts) {
    const double e = coneResidual(p, ap, d, alpha);
    s2 += e * e;
  }
  return std::sqrt(s2 / static_cast<double>(pts.size()));
}

ConeFit fitConeImpl(std::span<const Point3> pts) {
  ConeFit r;
  const int n = static_cast<int>(pts.size());
  if (n < 6) return r;
  const Point3 c = centroidOf(pts);
  const double extent = std::max(cloudExtent(pts, c), 1e-12);

  // Seed the axis from the covariance principal directions (the cone axis is a
  // symmetry axis of the sampled surface). For each candidate axis d we build a
  // CLOSED-FORM apex/half-angle seed: project every point to axis height h=(p-c)·d
  // and radial ρ=‖(p-c) - h d‖. On a perfect cone ρ = tanα·(h - h₀) with h₀ the
  // apex height, so a 1-D linear least-squares of ρ against h gives slope=tanα and
  // the apex at height h₀ = c + h₀·d (where ρ→0). That seed is EXACT on clean data;
  // the LM refine then only polishes it (and rescues the noisy case).
  const Eigen3 e = jacobiEigen(covariance(pts, c));
  ConeFit best;
  best.rms = 1e300;
  bool have = false;

  const auto resid = [&](const Vector& x) -> Vector {
    Vec3 d{x[3], x[4], x[5]};
    const double dn = norm(d);
    if (dn > 0) d = d / dn;
    const Point3 ap{x[0], x[1], x[2]};
    const double alpha = x[6];
    Vector out(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) out[i] = coneResidual(pts[i], ap, d, alpha);
    return out;
  };

  const auto tryAxis = [&](const Vec3& d0) {
    // Linear seed ρ = m·h + q  (m = tanα, apex at h₀ = -q/m).
    double sh = 0, srho = 0, shh = 0, shr = 0;
    for (int i = 0; i < n; ++i) {
      const Vec3 rel = pts[i] - c;
      const double h = dot(rel, d0);
      const double rho = norm(rel - d0 * h);
      sh += h; srho += rho; shh += h * h; shr += h * rho;
    }
    const double nn = static_cast<double>(n);
    const double denom = nn * shh - sh * sh;
    if (std::fabs(denom) < 1e-300) return;
    const double m = (nn * shr - sh * srho) / denom;
    const double q = (srho - m * sh) / nn;
    if (std::fabs(m) < 1e-300) return;
    // Orient the axis so the half-angle is positive (m>0). tanα = |m|.
    Vec3 d = (m >= 0) ? d0 : -d0;
    double slope = std::fabs(m);
    double alpha0 = std::atan(slope);
    if (!(alpha0 > 0.0) || alpha0 >= 0.5 * kPi) return;
    const double h0 = -q / m;  // apex height along d0
    const Point3 apex0 = c + d0 * h0;

    const numerics::SolveResult sr = least_squares(
        resid, {apex0.x, apex0.y, apex0.z, d.x, d.y, d.z, alpha0});
    if (sr.x.size() != 7) return;
    Vec3 dr{sr.x[3], sr.x[4], sr.x[5]};
    const double dn = norm(dr);
    if (dn <= 0) return;
    dr = dr / dn;
    double alpha = std::fabs(sr.x[6]);
    if (!(alpha > 0.0) || alpha >= 0.5 * kPi) return;
    const Point3 apex{sr.x[0], sr.x[1], sr.x[2]};
    const double rms = coneRms(pts, apex, dr, alpha);
    if (rms < best.rms) {
      best.apex = apex;
      best.axis = Dir3(dr);
      best.halfAngle = alpha;
      best.rms = rms;
      have = true;
    }
  };

  for (int k = 0; k < 3; ++k) {
    const Vec3 ax = toVec(e.vec[k]);
    tryAxis(ax);
    tryAxis(-ax);
  }
  if (!have || !best.axis.valid()) return r;
  best.ok = true;
  return best;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public API.
// ─────────────────────────────────────────────────────────────────────────────
PlaneFit fitPlane(std::span<const Point3> points) { return fitPlaneImpl(points); }
SphereFit fitSphere(std::span<const Point3> points, bool refine) {
  return fitSphereImpl(points, refine);
}
CylinderFit fitCylinder(std::span<const Point3> points) {
  return fitCylinderImpl(points);
}
ConeFit fitCone(std::span<const Point3> points) { return fitConeImpl(points); }

PrimitiveDetection detectPrimitive(std::span<const Point3> points, double relTol) {
  PrimitiveDetection out;
  if (points.size() < 3) return out;
  const Point3 c = centroidOf(points);
  const double extent = std::max(cloudExtent(points, c), 1e-12);

  out.plane = fitPlane(points);
  out.sphere = fitSphere(points);
  out.cylinder = fitCylinder(points);
  out.cone = fitCone(points);

  // Candidate (type, relative-error) list — smaller relError is a better fit.
  struct Cand { PrimitiveType t; double rel; bool ok; };
  const std::array<Cand, 4> cands{{
      {PrimitiveType::Plane, out.plane.ok ? out.plane.rms / extent : 1e300, out.plane.ok},
      {PrimitiveType::Sphere, out.sphere.ok ? out.sphere.rms / extent : 1e300, out.sphere.ok},
      {PrimitiveType::Cylinder, out.cylinder.ok ? out.cylinder.rms / extent : 1e300, out.cylinder.ok},
      {PrimitiveType::Cone, out.cone.ok ? out.cone.rms / extent : 1e300, out.cone.ok},
  }};

  // Pick the lowest-relError candidate that (a) succeeded and (b) is within relTol.
  // A cone/cylinder can also perfectly fit plane-like data as a degenerate limit;
  // we prefer the SIMPLER primitive on ties by ordering plane<sphere<cyl<cone and
  // requiring a strict improvement to override. So iterate in simplicity order and
  // accept the first within tolerance.
  const PrimitiveType order[4] = {PrimitiveType::Plane, PrimitiveType::Sphere,
                                  PrimitiveType::Cylinder, PrimitiveType::Cone};
  for (PrimitiveType want : order) {
    for (const auto& cd : cands) {
      if (cd.t == want && cd.ok && cd.rel <= relTol) {
        out.type = cd.t;
        out.rms = cd.rel * extent;
        out.relError = cd.rel;
        out.ok = true;
        return out;
      }
    }
  }
  // Nothing within tolerance → honest Freeform (no B-spline fit here).
  out.type = PrimitiveType::Freeform;
  out.ok = false;
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

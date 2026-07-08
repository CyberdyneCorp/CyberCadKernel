// SPDX-License-Identifier: Apache-2.0
//
// curvature.h — GS4 curvature: Gaussian K, mean H, principal k1/k2 at a surface
// (u,v) point, and edge curvature κ at a curve parameter t. OCCT-FREE.
//
// Clean-room differential geometry (do Carmo; Piegl & Tiller for the NURBS
// derivative evaluation, already native in math/bspline.h). Two arms per query:
//   * ANALYTIC closed form (plane / sphere / cylinder / cone / torus; line /
//     circle / ellipse) — exact, no derivative evaluation.
//   * FUNDAMENTAL FORMS for a freeform (B-spline / Bézier / NURBS) surface from
//     the native maxDeriv=2 derivatives; curvature κ = ‖C′×C″‖/‖C′‖³ for a
//     freeform curve.
//
// HONEST DECLINE (std::nullopt): a parametric singularity EG−F² ≤ ε·max(E,G)²
// (e.g. a sphere pole / cone apex chart), a cone within ε of its apex, or a
// stationary/cusp curve point ‖C′‖ ≤ ε — the service NEVER emits a blown-up
// number. Verified vs BRepLProp_SLProps / GeomLProp on the sim gate.
//
// Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_CURVATURE_H
#define CYBERCAD_NATIVE_ANALYSIS_CURVATURE_H

#include <array>
#include <cmath>
#include <optional>
#include <vector>

#include "native/analysis/native_analysis.h"
#include "native/math/bezier.h"
#include "native/math/bspline.h"

namespace cybercad::native::analysis {

// ─────────────────────────────────────────────────────────────────────────────
// Surface curvature.
// ─────────────────────────────────────────────────────────────────────────────

/// Curvature from the two fundamental forms given S_u, S_v, S_uu, S_uv, S_vv.
/// DECLINE when EG−F² is degenerate relative to max(E,G)² (a parametric
/// singularity — the chart normal is undefined there).
inline std::optional<SurfaceCurvature> curvatureFromForms(
    const Vec3& Su, const Vec3& Sv, const Vec3& Suu, const Vec3& Suv,
    const Vec3& Svv, double eps = 1e-12) {
  const double E = dot(Su, Su), F = dot(Su, Sv), G = dot(Sv, Sv);
  const double W = E * G - F * F;
  const double scale = std::max(E, G);
  if (!(W > eps * scale * scale)) return std::nullopt;  // parametric singularity

  const Vec3 nRaw = cross(Su, Sv);
  const double nLen = math::norm(nRaw);
  if (!(nLen > 0.0)) return std::nullopt;
  const Vec3 n = nRaw / nLen;

  const double L = dot(Suu, n), M = dot(Suv, n), N = dot(Svv, n);
  const double K = (L * N - M * M) / W;
  const double H = (E * N - 2.0 * F * M + G * L) / (2.0 * W);
  const double disc = std::sqrt(std::max(0.0, H * H - K));
  return SurfaceCurvature{K, H, H + disc, H - disc};
}

namespace detail {

/// Fill S_u, S_v, S_uu, S_uv, S_vv for a freeform (B-spline / Bézier / NURBS)
/// surface at (u,v). A Bézier patch is evaluated through the B-spline path with
/// a synthesised clamped knot vector so the second-derivative machinery is
/// shared. `out` = {Su, Sv, Suu, Suv, Svv}.
inline void freeformSurfaceDerivs2(const topo::FaceSurface& s, double u, double v,
                                   std::array<Vec3, 5>& out) {
  using K = topo::FaceSurface::Kind;
  const int degU = (s.kind == K::Bezier) ? s.nPolesU - 1 : s.degreeU;
  const int degV = (s.kind == K::Bezier) ? s.nPolesV - 1 : s.degreeV;

  // Clamped knot vector for a single Bézier span, else the stored B-spline knots.
  std::vector<double> kuBuf, kvBuf;
  auto clamped = [](int deg) {
    std::vector<double> kv(static_cast<std::size_t>(2 * (deg + 1)));
    for (int i = 0; i <= deg; ++i) { kv[i] = 0.0; kv[deg + 1 + i] = 1.0; }
    return kv;
  };
  std::span<const double> knotsU, knotsV;
  if (s.kind == K::Bezier) {
    kuBuf = clamped(degU); kvBuf = clamped(degV);
    knotsU = kuBuf; knotsV = kvBuf;
  } else {
    knotsU = s.knotsU; knotsV = s.knotsV;
  }

  math::SurfaceGrid grid{s.poles, s.nPolesU, s.nPolesV};
  std::array<Vec3, 9> d{};  // row-major 3×3, d[k*3+l] = ∂^(k+l)S/∂u^k∂v^l
  if (s.weights.empty())
    math::surfaceDerivs(degU, degV, grid, knotsU, knotsV, u, v, 2, d);
  else
    math::nurbsSurfaceDerivs(degU, degV, grid, s.weights, knotsU, knotsV, u, v, 2, d);

  out[0] = d[3];  // S_u  = ∂/∂u
  out[1] = d[1];  // S_v  = ∂/∂v
  out[2] = d[6];  // S_uu = ∂²/∂u²
  out[3] = d[4];  // S_uv = ∂²/∂u∂v
  out[4] = d[2];  // S_vv = ∂²/∂v²
}

}  // namespace detail

/// Surface curvature {K, H, k1, k2} at (u, v), or DECLINE.
/// Analytic surfaces use the exact closed form; freeform surfaces use the
/// fundamental forms of the native derivatives. `eps` guards the analytic apex
/// (cone) and the freeform parametric singularity.
inline std::optional<SurfaceCurvature> surfaceCurvature(const topo::FaceSurface& s,
                                                        double u, double v,
                                                        double eps = 1e-9) {
  using K = topo::FaceSurface::Kind;
  switch (s.kind) {
    case K::Plane:
      return SurfaceCurvature{0.0, 0.0, 0.0, 0.0};
    case K::Sphere: {
      const double c = 1.0 / s.radius;              // outward-normal magnitude
      return SurfaceCurvature{c * c, c, c, c};
    }
    case K::Cylinder: {
      const double c = 1.0 / s.radius;
      return SurfaceCurvature{0.0, c * 0.5, c, 0.0};
    }
    case K::Cone: {
      // Circle radius at axial param v is ρ = radius + v·sin α; principal
      // curvatures are {0, cos α / ρ}. DECLINE within ε of the apex (ρ → 0).
      const double rho = s.radius + v * std::sin(s.semiAngle);
      if (!(std::fabs(rho) > eps)) return std::nullopt;
      const double kc = std::cos(s.semiAngle) / rho;
      return SurfaceCurvature{0.0, kc * 0.5, kc, 0.0};
    }
    case K::Torus: {
      const double R = s.radius, r = s.minorRadius;
      const double denom = R + r * std::cos(v);      // outer-radius factor
      if (!(std::fabs(denom) > eps) || !(std::fabs(r) > eps)) return std::nullopt;
      const double kMinor = 1.0 / r;                 // tube direction
      const double kMajor = std::cos(v) / denom;     // revolution direction
      const double Kg = kMinor * kMajor;
      const double Hm = 0.5 * (kMinor + kMajor);
      const double hi = std::max(kMinor, kMajor), lo = std::min(kMinor, kMajor);
      return SurfaceCurvature{Kg, Hm, hi, lo};
    }
    case K::BSpline:
    case K::Bezier: {
      std::array<Vec3, 5> d{};
      detail::freeformSurfaceDerivs2(s, u, v, d);
      return curvatureFromForms(d[0], d[1], d[2], d[3], d[4], eps);
    }
  }
  return std::nullopt;
}

// ─────────────────────────────────────────────────────────────────────────────
// Edge curvature.
// ─────────────────────────────────────────────────────────────────────────────

/// κ = ‖C′×C″‖ / ‖C′‖³ from the first two derivatives. DECLINE at a
/// stationary / cusp point ‖C′‖ ≤ ε.
inline std::optional<double> curvatureFromDerivs(const Vec3& d1, const Vec3& d2,
                                                 double eps = 1e-12) {
  const double s = math::norm(d1);
  if (!(s > eps)) return std::nullopt;  // stationary / cusp point
  return math::norm(cross(d1, d2)) / (s * s * s);
}

/// Edge curvature κ at parameter t, or DECLINE. Line → 0, circle → 1/R, ellipse
/// and freeform via κ = ‖C′×C″‖/‖C′‖³.
inline std::optional<double> edgeCurvature(const topo::EdgeCurve& e, double t,
                                           double eps = 1e-12) {
  using K = topo::EdgeCurve::Kind;
  switch (e.kind) {
    case K::Line:
      return 0.0;
    case K::Circle:
      if (!(std::fabs(e.radius) > eps)) return std::nullopt;
      return 1.0 / e.radius;
    case K::Ellipse: {
      const double a = e.radius, b = e.minorRadius;
      const Vec3 X = e.frame.x.vec(), Y = e.frame.y.vec();
      const Vec3 d1 = X * (-a * std::sin(t)) + Y * (b * std::cos(t));
      const Vec3 d2 = X * (-a * std::cos(t)) + Y * (-b * std::sin(t));
      return curvatureFromDerivs(d1, d2, eps);
    }
    case K::Bezier: {
      std::array<Vec3, 3> d{};
      if (e.weights.empty()) {
        math::bezierDerivs(e.poles, t, 2, d);
      } else {
        // Rational Bézier via the clamped-knot NURBS derivative path.
        const int deg = static_cast<int>(e.poles.size()) - 1;
        std::vector<double> kv(static_cast<std::size_t>(2 * (deg + 1)));
        for (int i = 0; i <= deg; ++i) { kv[i] = 0.0; kv[deg + 1 + i] = 1.0; }
        math::nurbsCurveDerivs(deg, e.poles, e.weights, kv, t, 2, d);
      }
      return curvatureFromDerivs(d[1], d[2], eps);
    }
    case K::BSpline: {
      std::array<Vec3, 3> d{};
      if (e.weights.empty())
        math::curveDerivs(e.degree, e.poles, e.knots, t, 2, d);
      else
        math::nurbsCurveDerivs(e.degree, e.poles, e.weights, e.knots, t, 2, d);
      return curvatureFromDerivs(d[1], d[2], eps);
    }
  }
  return std::nullopt;
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_CURVATURE_H

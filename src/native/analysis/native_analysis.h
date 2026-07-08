// SPDX-License-Identifier: Apache-2.0
//
// native_analysis.h — public aggregate header for the native ANALYSIS services
// (MOAT M-GS, GS3 measurement + GS4 curvature).
//
// This module exposes two exact, OCCT-FREE analysis SERVICES on top of the
// landed native evaluators (src/native/math + src/native/numerics):
//   * GS3 MEASUREMENT — minimum distance between two B-rep entities (vertex /
//     edge / face) and the angle between two entities (line / plane), via
//     closed-form differential geometry for the analytic cells and a
//     seed-and-refine multi-start minimizer (numerics/closest_point.h) for the
//     NURBS cells. distance.h / angle.h.
//   * GS4 CURVATURE — Gaussian K, mean H and principal k1/k2 at a surface (u,v)
//     from the first/second fundamental forms of the native surface derivatives,
//     plus edge curvature κ from the curve derivatives. curvature.h.
//
// SELF-VERIFY / HONEST DECLINE: every service returns a std::optional; a
// std::nullopt is a first-class "measured decline" (parametric singularity,
// non-line/plane angle, non-certifiable freeform-trimmed minimizer) — the
// service NEVER emits a wrong number. OCCT is the ORACLE only (the sim gate); no
// OCCT header is included anywhere under src/native/**.
//
// Conventions mirror OCCT (gp_*/ElSLib/BSplSLib/BRepLProp) so results can be
// verified against the oracle on the simulator. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_H
#define CYBERCAD_NATIVE_ANALYSIS_H

#include <cmath>
#include <functional>
#include <optional>

#include "native/math/native_math.h"
#include "native/topology/shape.h"

namespace cybercad::native::analysis {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

using math::Point3;
using math::Vec3;

// ─────────────────────────────────────────────────────────────────────────────
// Result payloads. A std::optional<…> return of std::nullopt is an HONEST
// DECLINE (see file header) — distinct from any numeric result.
// ─────────────────────────────────────────────────────────────────────────────

/// Minimum distance between two entities and the witness (closest) point on each.
struct DistanceResult {
  double distance = 0.0;  ///< ‖p1 − p2‖ (the minimum gap)
  Point3 p1{};            ///< witness point on entity A
  Point3 p2{};            ///< witness point on entity B
};

/// Local differential curvature of a surface at a (u,v) point.
/// Sign convention: the chart normal n = normalize(S_u × S_v) (the sim-gate
/// facade flips H/k1/k2 for a Reversed face to match BRepLProp's face normal).
/// K is orientation-independent.
struct SurfaceCurvature {
  double K = 0.0;   ///< Gaussian curvature (LN−M²)/(EG−F²)
  double H = 0.0;   ///< mean curvature (EN−2FM+GL)/(2(EG−F²))
  double k1 = 0.0;  ///< principal curvature, k1 ≥ k2 = H ± √(H²−K)
  double k2 = 0.0;  ///< principal curvature
};

// ─────────────────────────────────────────────────────────────────────────────
// Entity — a resolved measurement operand: a vertex point, an edge (curve +
// parameter range) or a face (surface + a (u,v) search window). Analytic
// surfaces are periodic/unbounded, so a face always carries an explicit window.
// The pointers alias caller-owned geometry (the native topology leaf) and must
// outlive any analysis call that consumes the Entity.
// ─────────────────────────────────────────────────────────────────────────────
struct Entity {
  enum class Kind : std::uint8_t { Vertex, Edge, Face };
  Kind kind = Kind::Vertex;

  Point3 vertex{};                         ///< Kind::Vertex

  const topo::EdgeCurve* edge = nullptr;   ///< Kind::Edge geometry
  double t0 = 0.0, t1 = 0.0;               ///< Kind::Edge parameter range

  const topo::FaceSurface* face = nullptr; ///< Kind::Face geometry
  double u0 = 0.0, u1 = 0.0;               ///< Kind::Face (u,v) search window …
  double v0 = 0.0, v1 = 0.0;               ///< … (analytic surfaces require it)

  static Entity ofVertex(const Point3& p) {
    Entity e; e.kind = Kind::Vertex; e.vertex = p; return e;
  }
  static Entity ofEdge(const topo::EdgeCurve& c, double a, double b) {
    Entity e; e.kind = Kind::Edge; e.edge = &c; e.t0 = a; e.t1 = b; return e;
  }
  static Entity ofFace(const topo::FaceSurface& s, double ua, double ub,
                       double va, double vb) {
    Entity e; e.kind = Kind::Face; e.face = &s;
    e.u0 = ua; e.u1 = ub; e.v0 = va; e.v1 = vb; return e;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Kind predicates. An "analytic" curve/surface (line/circle/ellipse; plane/
// cylinder/cone/sphere/torus) has a robust closed-form or exact projection, so
// a minimizer half-step onto it is trustworthy; a freeform (BSpline/Bezier)
// entity is where the non-certifiable global-minimum risk lives.
// ─────────────────────────────────────────────────────────────────────────────
inline bool isAnalytic(topo::EdgeCurve::Kind k) noexcept {
  using K = topo::EdgeCurve::Kind;
  return k == K::Line || k == K::Circle || k == K::Ellipse;
}
inline bool isAnalytic(topo::FaceSurface::Kind k) noexcept {
  using K = topo::FaceSurface::Kind;
  return k != K::BSpline && k != K::Bezier;
}
inline bool isAnalytic(const Entity& e) noexcept {
  switch (e.kind) {
    case Entity::Kind::Vertex: return true;
    case Entity::Kind::Edge:   return isAnalytic(e.edge->kind);
    case Entity::Kind::Face:   return isAnalytic(e.face->kind);
  }
  return false;
}

// ─────────────────────────────────────────────────────────────────────────────
// Evaluators. A curve/surface is handed to the numeric minimizer as a plain
// callable over its parameter domain. These builders map every native geometry
// kind (analytic + freeform, rational + non-rational) onto that callable.
// ─────────────────────────────────────────────────────────────────────────────

/// A parametric curve evaluator + its search domain [t0, t1].
struct CurveEval {
  std::function<Point3(double)> fn;
  double t0 = 0.0, t1 = 1.0;
};

/// A parametric surface evaluator + its search window [u0,u1]×[v0,v1].
struct SurfEval {
  std::function<Point3(double, double)> fn;
  double u0 = 0.0, u1 = 1.0, v0 = 0.0, v1 = 1.0;
};

namespace detail {

/// Clamped B-spline domain [knots[deg], knots[end−deg]] for a flat knot vector.
inline std::pair<double, double> bsplineDomain(int degree,
                                               const std::vector<double>& knots) {
  const auto d = static_cast<std::size_t>(degree);
  return {knots[d], knots[knots.size() - 1 - d]};
}

}  // namespace detail

/// Build a curve evaluator for an edge over the parameter range [t0, t1].
/// For a freeform curve the range is overridden by the clamped knot domain
/// (Bézier: [0,1]) so the whole curve is searched.
inline CurveEval makeCurveEval(const topo::EdgeCurve& e, double t0, double t1) {
  using K = topo::EdgeCurve::Kind;
  const topo::EdgeCurve* ep = &e;
  const math::Ax3 f = e.frame;
  const double R = e.radius, b = e.minorRadius;
  switch (e.kind) {
    case K::Line:
      return {[f](double t) { return f.origin + f.x.vec() * t; }, t0, t1};
    case K::Circle:
      return {[f, R](double t) {
                return f.origin + f.x.vec() * (R * std::cos(t)) +
                       f.y.vec() * (R * std::sin(t));
              }, t0, t1};
    case K::Ellipse:
      return {[f, R, b](double t) {
                return f.origin + f.x.vec() * (R * std::cos(t)) +
                       f.y.vec() * (b * std::sin(t));
              }, t0, t1};
    case K::Bezier:
      if (ep->weights.empty())
        return {[ep](double t) { return math::bezierPoint(ep->poles, t); }, 0.0, 1.0};
      return {[ep](double t) {
                return math::rationalBezierPoint(ep->poles, ep->weights, t);
              }, 0.0, 1.0};
    case K::BSpline: {
      const auto [d0, d1] = detail::bsplineDomain(e.degree, e.knots);
      if (ep->weights.empty())
        return {[ep](double t) {
                  return math::curvePoint(ep->degree, ep->poles, ep->knots, t);
                }, d0, d1};
      return {[ep](double t) {
                return math::nurbsCurvePoint(ep->degree, ep->poles, ep->weights,
                                             ep->knots, t);
              }, d0, d1};
    }
  }
  return {[f](double t) { return f.origin + f.x.vec() * t; }, t0, t1};
}

/// Build a surface evaluator for a face. Analytic (periodic) surfaces use the
/// caller-supplied (u,v) window; a freeform surface overrides it with the
/// clamped knot / [0,1]² domain so the whole patch is searched.
inline SurfEval makeSurfEval(const topo::FaceSurface& s, double u0, double u1,
                             double v0, double v1) {
  using K = topo::FaceSurface::Kind;
  const topo::FaceSurface* sp = &s;
  const math::Ax3 f = s.frame;
  const double R = s.radius, sa = s.semiAngle, mr = s.minorRadius;
  switch (s.kind) {
    case K::Plane:
      return {[f](double u, double v) { return math::Plane{f}.value(u, v); },
              u0, u1, v0, v1};
    case K::Cylinder:
      return {[f, R](double u, double v) {
                return math::Cylinder{f, R}.value(u, v);
              }, u0, u1, v0, v1};
    case K::Cone:
      return {[f, R, sa](double u, double v) {
                return math::Cone{f, R, sa}.value(u, v);
              }, u0, u1, v0, v1};
    case K::Sphere:
      return {[f, R](double u, double v) {
                return math::Sphere{f, R}.value(u, v);
              }, u0, u1, v0, v1};
    case K::Torus:
      return {[f, R, mr](double u, double v) {
                return math::Torus{f, R, mr}.value(u, v);
              }, u0, u1, v0, v1};
    case K::Bezier: {
      const int nR = sp->nPolesU, nC = sp->nPolesV;
      if (sp->weights.empty())
        return {[sp, nR, nC](double u, double v) {
                  return math::bezierSurfacePoint(sp->poles, nR, nC, u, v);
                }, 0.0, 1.0, 0.0, 1.0};
      return {[sp, nR, nC](double u, double v) {
                return math::rationalBezierSurfacePoint(sp->poles, sp->weights,
                                                        nR, nC, u, v);
              }, 0.0, 1.0, 0.0, 1.0};
    }
    case K::BSpline: {
      const auto [ua, ub] = detail::bsplineDomain(s.degreeU, s.knotsU);
      const auto [va, vb] = detail::bsplineDomain(s.degreeV, s.knotsV);
      math::SurfaceGrid grid{sp->poles, sp->nPolesU, sp->nPolesV};
      if (sp->weights.empty())
        return {[sp, grid](double u, double v) {
                  return math::surfacePoint(sp->degreeU, sp->degreeV, grid,
                                            sp->knotsU, sp->knotsV, u, v);
                }, ua, ub, va, vb};
      return {[sp, grid](double u, double v) {
                return math::nurbsSurfacePoint(sp->degreeU, sp->degreeV, grid,
                                               sp->weights, sp->knotsU, sp->knotsV,
                                               u, v);
              }, ua, ub, va, vb};
    }
  }
  return {[f](double u, double v) { return math::Plane{f}.value(u, v); },
          u0, u1, v0, v1};
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_H

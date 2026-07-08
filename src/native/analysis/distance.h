// SPDX-License-Identifier: Apache-2.0
//
// distance.h — GS3 measurement: minimum distance between two B-rep entities
// (vertex / edge / face) with the witness (closest) point on each. OCCT-FREE.
//
// Two families of cell:
//   * CLOSED FORM (analytic·analytic) — point·{point,segment,line,plane,circle},
//     segment·segment (parallel + skew, feet clamped to their ranges). Exact,
//     certifiable; matches BRepExtrema_DistShapeShape's bounded distance.
//   * SEED-AND-REFINE (any freeform/curved cell) — project onto the other entity
//     with the multi-start minimizer (numerics/closest_point.h) and alternate the
//     two global projections to a mutually-closest witness pair. Point·entity is
//     the robust multi-start projection directly.
//
// SELF-VERIFY / HONEST DECLINE (std::nullopt): the alternating minimizer returns
// a result ONLY when it certifies convergence to a mutually-closest witness pair;
// a non-converging / non-certifiable freeform-trimmed pair DECLINES rather than
// emit a guessed minimum. Verified vs BRepExtrema_DistShapeShape on the sim gate.
//
// Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_DISTANCE_H
#define CYBERCAD_NATIVE_ANALYSIS_DISTANCE_H

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

#include "native/analysis/native_analysis.h"
#include "native/numerics/closest_point.h"

namespace cybercad::native::analysis {

// ─────────────────────────────────────────────────────────────────────────────
// Closed-form analytic cells (exact).
// ─────────────────────────────────────────────────────────────────────────────

/// Distance from a point to the INFINITE line through O with unit direction d.
inline DistanceResult distPointLine(const Point3& p, const Point3& o, const Vec3& d) {
  const Vec3 w = p - o;
  const double t = dot(w, d);
  const Point3 foot = o + d * t;
  return {math::distance(p, foot), p, foot};
}

/// Distance from a point to the SEGMENT [a, b] (foot clamped to the segment).
inline DistanceResult distPointSegment(const Point3& p, const Point3& a,
                                       const Point3& b) {
  const Vec3 ab = b - a;
  const double len2 = dot(ab, ab);
  double t = (len2 > 0.0) ? dot(p - a, ab) / len2 : 0.0;
  t = std::clamp(t, 0.0, 1.0);
  const Point3 foot = a + ab * t;
  return {math::distance(p, foot), p, foot};
}

/// Distance from a point to the plane through O with unit normal n.
inline DistanceResult distPointPlane(const Point3& p, const Point3& o, const Vec3& n) {
  const double s = dot(p - o, n);
  const Point3 foot = p - n * s;
  return {std::fabs(s), p, foot};
}

/// Distance from a point to the circle of radius R centred at C in the plane
/// spanned by (x, y) with axis z (all unit, right-handed).
inline DistanceResult distPointCircle(const Point3& p, const Point3& c,
                                      const Vec3& x, const Vec3& y, const Vec3& z,
                                      double R) {
  const Vec3 w = p - c;
  const double wz = dot(w, z);
  Vec3 radial = w - z * wz;          // component in the circle's plane
  const double rho = math::norm(radial);
  const Vec3 dir = (rho > math::kLinearTolerance)
                       ? radial / rho
                       : x;          // on the axis: any circle point is nearest
  const Point3 foot = c + dir * R;
  const double planar = (rho > math::kLinearTolerance) ? (rho - R) : -R;
  return {std::sqrt(planar * planar + wz * wz), p, foot};
}

/// Minimum distance between two SEGMENTS [p0,p1] and [q0,q1] with the witness
/// point on each (Ericson, Real-Time Collision Detection §5.1.9 — robust for
/// parallel and skew, feet clamped to the segments).
inline DistanceResult distSegSeg(const Point3& p0, const Point3& p1,
                                 const Point3& q0, const Point3& q1) {
  const Vec3 d1 = p1 - p0, d2 = q1 - q0, r = p0 - q0;
  const double a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r);
  double s, t;
  constexpr double kEps = 1e-15;
  if (a <= kEps && e <= kEps) {                 // both degenerate → point·point
    s = t = 0.0;
  } else if (a <= kEps) {                        // first degenerate
    s = 0.0; t = std::clamp(f / e, 0.0, 1.0);
  } else {
    const double c = dot(d1, r);
    if (e <= kEps) {                             // second degenerate
      t = 0.0; s = std::clamp(-c / a, 0.0, 1.0);
    } else {
      const double b = dot(d1, d2);
      const double denom = a * e - b * b;        // ≥ 0
      s = (denom > kEps) ? std::clamp((b * f - c * e) / denom, 0.0, 1.0) : 0.0;
      t = (b * s + f) / e;
      if (t < 0.0)      { t = 0.0; s = std::clamp(-c / a, 0.0, 1.0); }
      else if (t > 1.0) { t = 1.0; s = std::clamp((b - c) / a, 0.0, 1.0); }
    }
  }
  const Point3 c1 = p0 + d1 * s, c2 = q0 + d2 * t;
  return {math::distance(c1, c2), c1, c2};
}

// ─────────────────────────────────────────────────────────────────────────────
// Seed-and-refine machinery for curved / freeform cells.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// A geometry the alternating minimizer can sample and globally project onto.
struct Projectable {
  std::function<void(int, std::vector<Point3>&)> sample;              // grid seeds
  std::function<std::pair<Point3, double>(const Point3&)> project;    // global NN
};

inline Projectable projectableCurve(const CurveEval& c) {
  return {
      [c](int n, std::vector<Point3>& out) {
        out.clear();
        const int m = std::max(2, n);
        for (int i = 0; i < m; ++i)
          out.push_back(c.fn(c.t0 + (c.t1 - c.t0) * (double(i) / (m - 1))));
      },
      [c](const Point3& target) {
        auto r = numerics::project_point_to_curve(target, c.fn, c.t0, c.t1);
        return std::pair<Point3, double>{r.point, r.distance};
      }};
}

inline Projectable projectableSurf(const SurfEval& s) {
  return {
      [s](int n, std::vector<Point3>& out) {
        out.clear();
        const int m = std::max(2, n);
        for (int i = 0; i < m; ++i)
          for (int j = 0; j < m; ++j)
            out.push_back(s.fn(s.u0 + (s.u1 - s.u0) * (double(i) / (m - 1)),
                               s.v0 + (s.v1 - s.v0) * (double(j) / (m - 1))));
      },
      [s](const Point3& target) {
        auto r = numerics::project_point_to_surface(target, s.fn, s.u0, s.u1,
                                                    s.v0, s.v1);
        return std::pair<Point3, double>{r.point, r.distance};
      }};
}

/// Alternating-projection refinement from a single starting point `seedA` on A.
/// Each half-step is a GLOBAL multi-start projection, so the fixed point is a
/// mutually-closest (critical) witness pair. Reports the converged distance and
/// whether it converged.
struct RefineResult {
  double d = std::numeric_limits<double>::infinity();
  Point3 pA{}, pB{};
  bool converged = false;
};

inline RefineResult refinePair(const Projectable& A, const Projectable& B,
                               const Point3& seedA) {
  RefineResult r;
  Point3 pA = seedA, pB{};
  double prev = std::numeric_limits<double>::infinity();
  for (int it = 0; it < 60; ++it) {
    auto [b, dB] = B.project(pA); pB = b;
    auto [a, dA] = A.project(pB); pA = a;
    if (std::fabs(dA - prev) <= 1e-12 * (1.0 + dA)) { r.converged = true; break; }
    prev = dA;
  }
  r.pA = pA; r.pB = pB; r.d = math::distance(pA, pB);
  return r;
}

/// Certify a refined pair as a genuine mutually-closest witness: re-projecting
/// each witness onto the other entity must reproduce the same gap (within tol).
inline bool certify(const Projectable& A, const Projectable& B,
                    const RefineResult& r, double tol) {
  if (!r.converged) return false;
  const double dB = B.project(r.pA).second;
  const double dA = A.project(r.pB).second;
  return std::fabs(dB - r.d) <= tol && std::fabs(dA - r.d) <= tol;
}

/// Minimum distance between two curved/freeform entities via multi-seed
/// alternating projection + certification. DECLINE (std::nullopt) when no seed
/// yields a certified mutually-closest witness pair (a non-certifiable freeform-
/// trimmed minimizer).
inline std::optional<DistanceResult> entityEntity(const Projectable& A,
                                                  const Projectable& B) {
  constexpr int kSeeds = 8;   // per-side grid density
  constexpr int kTopK = 6;    // seeds refined
  std::vector<Point3> sa;
  A.sample(kSeeds, sa);

  // Rank A-seeds by their global gap to B; refine the best few.
  std::vector<std::pair<double, Point3>> ranked;
  ranked.reserve(sa.size());
  for (const Point3& pa : sa) ranked.push_back({B.project(pa).second, pa});
  std::sort(ranked.begin(), ranked.end(),
            [](const auto& x, const auto& y) { return x.first < y.first; });

  RefineResult best;
  double scale = 1.0;
  for (int i = 0; i < kTopK && i < static_cast<int>(ranked.size()); ++i) {
    const RefineResult r = refinePair(A, B, ranked[i].second);
    scale = std::max(scale, r.d);
    if (r.d < best.d) best = r;
  }

  const double certTol = 1e-7 * scale;
  if (!certify(A, B, best, certTol)) return std::nullopt;  // honest decline
  return DistanceResult{best.d, best.pA, best.pB};
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Dispatcher.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// Endpoints of a Line edge over its parameter range.
inline std::pair<Point3, Point3> lineSegment(const Entity& e) {
  const math::Ax3& f = e.edge->frame;
  return {f.origin + f.x.vec() * e.t0, f.origin + f.x.vec() * e.t1};
}

/// Point → edge distance (closed form for line/circle, else projection).
inline std::optional<DistanceResult> pointEdge(const Point3& p, const Entity& edge) {
  using K = topo::EdgeCurve::Kind;
  const topo::EdgeCurve& c = *edge.edge;
  if (c.kind == K::Line) {
    auto [a, b] = lineSegment(edge);
    return distPointSegment(p, a, b);
  }
  if (c.kind == K::Circle) {
    const math::Ax3& f = c.frame;
    return distPointCircle(p, f.origin, f.x.vec(), f.y.vec(), f.z.vec(), c.radius);
  }
  const CurveEval ce = makeCurveEval(c, edge.t0, edge.t1);
  auto r = numerics::project_point_to_curve(p, ce.fn, ce.t0, ce.t1);
  if (!r.success) return std::nullopt;
  return DistanceResult{r.distance, p, r.point};
}

/// Point → face distance (closed form for a plane, else projection).
inline std::optional<DistanceResult> pointFace(const Point3& p, const Entity& face) {
  const topo::FaceSurface& s = *face.face;
  if (s.kind == topo::FaceSurface::Kind::Plane)
    return distPointPlane(p, s.frame.origin, s.frame.z.vec());
  const SurfEval se = makeSurfEval(s, face.u0, face.u1, face.v0, face.v1);
  auto r = numerics::project_point_to_surface(p, se.fn, se.u0, se.u1, se.v0, se.v1);
  if (!r.success) return std::nullopt;
  return DistanceResult{r.distance, p, r.point};
}

/// Build the alternating-projection Projectable for an edge or face entity.
inline Projectable projectableOf(const Entity& e) {
  if (e.kind == Entity::Kind::Edge)
    return projectableCurve(makeCurveEval(*e.edge, e.t0, e.t1));
  return projectableSurf(makeSurfEval(*e.face, e.u0, e.u1, e.v0, e.v1));
}

}  // namespace detail

/// Minimum distance + witness points between two entities, or HONEST DECLINE.
/// `a`/`b` order is preserved: result.p1 is on `a`, result.p2 is on `b`.
inline std::optional<DistanceResult> minDistance(const Entity& a, const Entity& b) {
  using Kd = Entity::Kind;

  // Vertex on the left: point·{vertex,edge,face}.
  if (a.kind == Kd::Vertex) {
    if (b.kind == Kd::Vertex)
      return DistanceResult{math::distance(a.vertex, b.vertex), a.vertex, b.vertex};
    auto r = (b.kind == Kd::Edge) ? detail::pointEdge(a.vertex, b)
                                  : detail::pointFace(a.vertex, b);
    return r;  // p1 on a (the point), p2 on b — matches DistanceResult layout
  }
  // Vertex on the right: reuse the left-vertex path and swap witnesses back.
  if (b.kind == Kd::Vertex) {
    auto r = minDistance(b, a);
    if (!r) return std::nullopt;
    return DistanceResult{r->distance, r->p2, r->p1};
  }

  // Edge·edge with both straight → exact segment-segment.
  if (a.kind == Kd::Edge && b.kind == Kd::Edge &&
      a.edge->kind == topo::EdgeCurve::Kind::Line &&
      b.edge->kind == topo::EdgeCurve::Kind::Line) {
    auto [a0, a1] = detail::lineSegment(a);
    auto [b0, b1] = detail::lineSegment(b);
    return distSegSeg(a0, a1, b0, b1);
  }

  // Any other edge/face pair → certified seed-and-refine.
  return detail::entityEntity(detail::projectableOf(a), detail::projectableOf(b));
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_DISTANCE_H

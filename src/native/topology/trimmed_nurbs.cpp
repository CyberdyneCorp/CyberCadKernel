// SPDX-License-Identifier: Apache-2.0
//
// trimmed_nurbs.cpp — NURBS roadmap Layer 8 implementation (trimmed-NURBS B-rep
// data model + pcurve robustness). See trimmed_nurbs.h for the contract.
//
// The always-on part (data model, classify, pcurveFidelity) uses only src/native/
// math + src/native/topology. constructPcurve is compiled ONLY under
// CYBERCAD_HAS_NUMSCI (it depends on numerics::closest_point_on_surface + bspline_fit),
// exactly like src/native/math/bspline_fit.cpp. With the guard OFF that one function
// is simply absent from the library; everything else builds and tests without it.
//
#include "native/topology/trimmed_nurbs.h"

#include "native/topology/accessors.h"  // rangeOf / pcurveOf
#include "native/topology/explore.h"    // Explorer

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_fit.h"           // interpolateCurve / approximateCurve
#include "native/numerics/numerics.h"          // closest_point_on_surface
#endif

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>

namespace cybercad::native::topology {
namespace {

// ─────────────────────────────────────────────────────────────────────────────
// Local, topology-scoped evaluators. Kept self-contained (the tessellate module's
// SurfaceEvaluator / pcurveValue do the same thing, but topology must not depend on
// tessellate). Small Visitor switches over the payload kinds.
// ─────────────────────────────────────────────────────────────────────────────

math::Point3 placePoint(const Location& loc, const math::Point3& p) noexcept {
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}

math::SurfaceGrid gridOf(const FaceSurface& s) noexcept {
  return math::SurfaceGrid{std::span<const math::Point3>(s.poles.data(), s.poles.size()),
                           s.nPolesU, s.nPolesV};
}

// LOCAL (un-placed) S(u,v) for each FaceSurface kind.
math::Point3 surfaceLocal(const FaceSurface& s, double u, double v) noexcept {
  using K = FaceSurface::Kind;
  switch (s.kind) {
    case K::Plane:    return math::Plane{s.frame}.value(u, v);
    case K::Cylinder: return math::Cylinder{s.frame, s.radius}.value(u, v);
    case K::Cone:     return math::Cone{s.frame, s.radius, s.semiAngle}.value(u, v);
    case K::Sphere:   return math::Sphere{s.frame, s.radius}.value(u, v);
    case K::Torus:    return math::Torus{s.frame, s.radius, s.minorRadius}.value(u, v);
    case K::Bezier:
      if (s.poles.empty()) return s.frame.origin;
      return s.weights.empty()
                 ? math::bezierSurfacePoint({s.poles.data(), s.poles.size()}, s.nPolesU,
                                            s.nPolesV, u, v)
                 : math::rationalBezierSurfacePoint({s.poles.data(), s.poles.size()},
                                                    {s.weights.data(), s.weights.size()},
                                                    s.nPolesU, s.nPolesV, u, v);
    case K::BSpline:
    default:
      if (s.poles.empty()) return s.frame.origin;
      return s.weights.empty()
                 ? math::surfacePoint(s.degreeU, s.degreeV, gridOf(s),
                                      {s.knotsU.data(), s.knotsU.size()},
                                      {s.knotsV.data(), s.knotsV.size()}, u, v)
                 : math::nurbsSurfacePoint(s.degreeU, s.degreeV, gridOf(s),
                                           {s.weights.data(), s.weights.size()},
                                           {s.knotsU.data(), s.knotsU.size()},
                                           {s.knotsV.data(), s.knotsV.size()}, u, v);
  }
}

// LOCAL (un-placed) C(t) for each EdgeCurve kind.
math::Point3 edgeCurveLocal(const EdgeCurve& c, double t) noexcept {
  using K = EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
      return c.frame.origin + c.frame.x.vec() * t;
    case K::Circle:
      return c.frame.origin + c.frame.x.vec() * (c.radius * std::cos(t)) +
             c.frame.y.vec() * (c.radius * std::sin(t));
    case K::Ellipse:
      return c.frame.origin + c.frame.x.vec() * (c.radius * std::cos(t)) +
             c.frame.y.vec() * (c.minorRadius * std::sin(t));
    case K::Bezier:
      if (c.poles.empty()) return c.frame.origin;
      return c.weights.empty()
                 ? math::bezierPoint({c.poles.data(), c.poles.size()}, t)
                 : math::rationalBezierPoint({c.poles.data(), c.poles.size()},
                                             {c.weights.data(), c.weights.size()}, t);
    case K::BSpline:
    default:
      if (c.poles.empty()) return c.frame.origin;
      if (c.knots.empty())
        return math::bezierPoint({c.poles.data(), c.poles.size()}, t);
      return c.weights.empty()
                 ? math::curvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                    {c.knots.data(), c.knots.size()}, t)
                 : math::nurbsCurvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                         {c.weights.data(), c.weights.size()},
                                         {c.knots.data(), c.knots.size()}, t);
  }
}

// Evaluate a 2-D pcurve at true parameter `t` (analytic kinds) with `frac` the
// [0,1] position across the edge's range (free-form pole-poly fallback). Mirrors
// tessellate::pcurveValue exactly (the seam-weld contract), duplicated here to keep
// topology dependency-clean.
ParamPoint pcurveValue(const PCurve& c, double t, double frac) noexcept {
  using K = EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
      return {c.origin2d.x + c.dir2d.x * t, c.origin2d.y + c.dir2d.y * t};
    case K::Circle: {
      const double r = c.dir2d.x;
      return {c.origin2d.x + r * std::cos(t), c.origin2d.y + r * std::sin(t)};
    }
    case K::Ellipse: {
      const double a = c.dir2d.x, b = c.dir2d.y;
      return {c.origin2d.x + a * std::cos(t), c.origin2d.y + b * std::sin(t)};
    }
    case K::BSpline: {
      const int deg = c.degree;
      const std::size_t np = c.poles2d.size();
      const bool haveKnots =
          deg >= 1 && np >= static_cast<std::size_t>(deg + 1) && c.knots.size() == np + deg + 1;
      if (haveKnots) {
        const bool rational = c.weights.size() == np;
        const math::Point3 p =
            rational ? math::nurbsCurvePoint(deg, {c.poles2d.data(), np},
                                             {c.weights.data(), c.weights.size()},
                                             {c.knots.data(), c.knots.size()}, t)
                     : math::curvePoint(deg, {c.poles2d.data(), np},
                                        {c.knots.data(), c.knots.size()}, t);
        return {p.x, p.y};
      }
      [[fallthrough]];
    }
    case K::Bezier:
    default: {
      if (c.poles2d.empty()) return {c.origin2d.x, c.origin2d.y};
      if (c.poles2d.size() == 1) return {c.poles2d[0].x, c.poles2d[0].y};
      const double f = frac <= 0.0 ? 0.0 : (frac >= 1.0 ? 1.0 : frac);
      const double scaled = f * static_cast<double>(c.poles2d.size() - 1);
      const auto i = static_cast<std::size_t>(scaled);
      const std::size_t j = std::min(i + 1, c.poles2d.size() - 1);
      const double a = scaled - static_cast<double>(i);
      return {c.poles2d[i].x * (1 - a) + c.poles2d[j].x * a,
              c.poles2d[i].y * (1 - a) + c.poles2d[j].y * a};
    }
  }
}

// Evaluate one PcurveSegment at fraction s ∈ [0,1] along its (oriented) traversal.
ParamPoint segmentValue(const PcurveSegment& seg, double s) noexcept {
  const double a = s < 0.0 ? 0.0 : (s > 1.0 ? 1.0 : s);
  const double f = seg.reversed ? 1.0 - a : a;  // fraction along the stored poly
  const double t = seg.first + (seg.last - seg.first) * f;
  return pcurveValue(seg.curve, t, f);
}

// ─────────────────────────────────────────────────────────────────────────────
// Flatten a trim loop into a UV polyline (implicitly closed). Consecutive
// duplicate points (join vertices between segments) are dropped so the polyline is
// non-degenerate for the raycast.
// ─────────────────────────────────────────────────────────────────────────────
std::vector<ParamPoint> flattenLoop(const TrimLoop& loop, int segsPerSegment) {
  std::vector<ParamPoint> poly;
  if (loop.empty()) return poly;
  const int n = std::max(2, segsPerSegment);
  for (const PcurveSegment& seg : loop) {
    for (int i = 0; i <= n; ++i) {
      const ParamPoint p = segmentValue(seg, static_cast<double>(i) / n);
      if (!poly.empty()) {
        const ParamPoint& q = poly.back();
        if (std::fabs(p.u - q.u) < 1e-15 && std::fabs(p.v - q.v) < 1e-15) continue;
      }
      poly.push_back(p);
    }
  }
  // Drop a closing duplicate (last == first) so the implicit-close edge is unique.
  if (poly.size() >= 2) {
    const ParamPoint& a = poly.front();
    const ParamPoint& b = poly.back();
    if (std::fabs(a.u - b.u) < 1e-15 && std::fabs(a.v - b.v) < 1e-15) poly.pop_back();
  }
  return poly;
}

// Distance from point p to segment [a,b] in UV.
double distPointSeg(const ParamPoint& p, const ParamPoint& a, const ParamPoint& b) noexcept {
  const double dx = b.u - a.u, dy = b.v - a.v;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 0.0) t = ((p.u - a.u) * dx + (p.v - a.v) * dy) / len2;
  t = std::clamp(t, 0.0, 1.0);
  const double cu = a.u + t * dx, cv = a.v + t * dy;
  const double eu = p.u - cu, ev = p.v - cv;
  return std::sqrt(eu * eu + ev * ev);
}

// UV extent (diagonal of the bounding box) of a polyline — the loop's length scale.
double polyExtent(const std::vector<ParamPoint>& poly) noexcept {
  if (poly.empty()) return 0.0;
  double uMin = poly[0].u, uMax = poly[0].u, vMin = poly[0].v, vMax = poly[0].v;
  for (const ParamPoint& p : poly) {
    uMin = std::min(uMin, p.u); uMax = std::max(uMax, p.u);
    vMin = std::min(vMin, p.v); vMax = std::max(vMax, p.v);
  }
  const double du = uMax - uMin, dv = vMax - vMin;
  return std::sqrt(du * du + dv * dv);
}

// Robust even-odd ray-cast of a simple polygon.
//   Returns: 0 = outside, 1 = inside, 2 = on-boundary (within tol), 3 = degenerate
//            (fewer than 3 points — an honest decline upstream).
//
// The parity test is the Franklin PNPOLY half-open rule: an edge is counted iff the
// ray's height p.v lies in the HALF-OPEN v-interval of the edge, i.e.
// (a.v > p.v) != (b.v > p.v). This treats a shared vertex CONSISTENTLY (one incident
// edge's interval includes it, the other excludes it), so a ray grazing a polygon
// vertex is handled deterministically and correctly — no per-vertex veto is needed
// (an axis-aligned rectangle sampled uniformly puts many vertices at the query
// height; a veto there would spuriously decline). Genuinely degenerate loops (empty /
// open / self-touching) are declined UPSTREAM by loopWellFormed; true on-boundary
// grazes are caught FIRST by the on-edge band below.
int raycast(const std::vector<ParamPoint>& poly, const ParamPoint& p, double onEdgeTol) noexcept {
  const std::size_t n = poly.size();
  if (n < 3) return 3;  // degenerate/open loop → decline (honest)

  // On-boundary band first: a point within tol of ANY edge is on the boundary.
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    if (distPointSeg(p, poly[i], poly[j]) <= onEdgeTol) return 2;

  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const ParamPoint& a = poly[i];
    const ParamPoint& b = poly[j];
    const bool straddles = (a.v > p.v) != (b.v > p.v);
    if (straddles) {
      const double xCross = (b.u - a.u) * (p.v - a.v) / (b.v - a.v) + a.u;
      if (p.u < xCross) inside = !inside;
    }
  }
  return inside ? 1 : 0;
}

// Is a loop well-formed (closed, non-self-touching to first order)? A loop with
// fewer than 3 distinct points, or with a repeated NON-adjacent vertex (a pinch /
// self-touch), is declined. Cheap O(n²) check — trim loops are small.
bool loopWellFormed(const std::vector<ParamPoint>& poly) noexcept {
  const std::size_t n = poly.size();
  if (n < 3) return false;
  constexpr double kEps = 1e-12;
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t k = i + 2; k < n; ++k) {
      if (i == 0 && k == n - 1) continue;  // adjacent across the closing edge
      if (std::fabs(poly[i].u - poly[k].u) < kEps && std::fabs(poly[i].v - poly[k].v) < kEps)
        return false;  // a self-touch / pinch point
    }
  return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// makeTrimmedFace — build from an existing topology face Shape.
// ─────────────────────────────────────────────────────────────────────────────
std::optional<TrimmedNurbsFace> makeTrimmedFace(const Shape& face) {
  if (face.isNull() || face.type() != ShapeType::Face) return std::nullopt;
  const auto& geom = face.tshape()->geometry();
  if (!std::holds_alternative<FaceSurface>(geom)) return std::nullopt;

  TrimmedNurbsFace out;
  out.surface = std::get<FaceSurface>(geom);
  out.location = face.location();

  const auto& wires = face.tshape()->children();
  for (std::size_t w = 0; w < wires.size(); ++w) {
    TrimLoop loop;
    for (Explorer ex(wires[w], ShapeType::Edge); ex.more(); ex.next()) {
      const Shape& edge = ex.current();
      const PCurve* pc = pcurveOf(edge, face);
      if (!pc) {
        // Single-pcurve fallback (an edge whose pcurve was laid on a sibling node
        // sharing this surface): unambiguous when the edge carries exactly one.
        const auto& pcs = edge.tshape()->pcurves();
        if (pcs.size() == 1) pc = &pcs.front().curve;
      }
      if (!pc) continue;
      PcurveSegment seg;
      seg.curve = *pc;
      if (const auto rr = rangeOf(edge)) { seg.first = rr->first; seg.last = rr->last; }
      seg.reversed = edge.orientation() == Orientation::Reversed;
      loop.push_back(std::move(seg));
    }
    if (w == 0) {
      if (loop.empty()) return std::nullopt;  // no usable outer loop → decline
      out.outer = std::move(loop);
    } else if (!loop.empty()) {
      out.holes.push_back(std::move(loop));
    }
  }
  if (!out.hasOuter()) return std::nullopt;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// classify — robust point-in-trimmed-region.
// ─────────────────────────────────────────────────────────────────────────────
Containment classify(const TrimmedNurbsFace& face, const ParamPoint& p,
                     const ClassifyOptions& opts) {
  if (!face.hasOuter()) return Containment::Unknown;

  const std::vector<ParamPoint> outer = flattenLoop(face.outer, opts.flattenSegments);
  if (!loopWellFormed(outer)) return Containment::Unknown;

  const double extent = std::max(polyExtent(outer), 1e-300);
  const double tol = opts.onEdgeTol * std::max(extent, 1.0);

  const int outerHit = raycast(outer, p, tol);
  if (outerHit == 2) return Containment::OnBoundary;
  if (outerHit == 3) return Containment::Unknown;
  if (outerHit == 0) return Containment::Out;  // outside the outer loop

  // Inside the outer loop: a point inside ANY hole is Out; on a hole edge is
  // OnBoundary; a hole ambiguity is Unknown.
  for (const TrimLoop& hole : face.holes) {
    const std::vector<ParamPoint> hpoly = flattenLoop(hole, opts.flattenSegments);
    if (!loopWellFormed(hpoly)) return Containment::Unknown;
    const double hext = std::max(polyExtent(hpoly), 1e-300);
    const double htol = opts.onEdgeTol * std::max(hext, 1.0);
    const int h = raycast(hpoly, p, htol);
    if (h == 2) return Containment::OnBoundary;
    if (h == 3) return Containment::Unknown;
    if (h == 1) return Containment::Out;  // inside a hole → removed
  }
  return Containment::In;
}

// ─────────────────────────────────────────────────────────────────────────────
// pcurveFidelity — verify S(p(t)) == C(t) on a dense sample.
// ─────────────────────────────────────────────────────────────────────────────
FidelityReport pcurveFidelity(const FaceSurface& surface, const Location& surfLoc,
                              const EdgeCurve& edgeCurve, const Location& edgeLoc,
                              const PCurve& pcurve, double first, double last,
                              const FidelityOptions& opts) {
  FidelityReport rep;
  const int m = std::max(2, opts.samples);
  const double span = last - first;

  // Length scale of the 3-D edge over the sampled range (sum of chord lengths).
  double lengthScale = 0.0;
  math::Point3 prev = placePoint(edgeLoc, edgeCurveLocal(edgeCurve, first));
  for (int i = 1; i <= m; ++i) {
    const double t = first + span * (static_cast<double>(i) / m);
    const math::Point3 cur = placePoint(edgeLoc, edgeCurveLocal(edgeCurve, t));
    lengthScale += math::distance(prev, cur);
    prev = cur;
  }
  rep.tolerance = opts.absTol + opts.relTol * std::max(lengthScale, 1.0);

  double sum = 0.0, worst = -1.0, worstT = first;
  for (int i = 0; i <= m; ++i) {
    const double a = static_cast<double>(i) / m;
    const double t = first + span * a;
    // pcurve(t): true-parameter for analytic, frac for the poly fallback.
    const ParamPoint uv = pcurveValue(pcurve, t, a);
    const math::Point3 sOnPc = placePoint(surfLoc, surfaceLocal(surface, uv.u, uv.v));
    const math::Point3 cOfT = placePoint(edgeLoc, edgeCurveLocal(edgeCurve, t));
    const double d = math::distance(sOnPc, cOfT);
    sum += d;
    if (d > worst) { worst = d; worstT = t; }
  }
  rep.samples = m + 1;
  rep.maxDeviation = worst < 0.0 ? 0.0 : worst;
  rep.meanDeviation = sum / rep.samples;
  rep.atParam = worstT;
  rep.ok = rep.maxDeviation <= rep.tolerance;
  return rep;
}

#ifdef CYBERCAD_HAS_NUMSCI
// ─────────────────────────────────────────────────────────────────────────────
// constructPcurve (numsci-gated) — project sampled edge points to (u,v) and fit a
// 2-D B-spline pcurve, then round-trip-verify fidelity.
// ─────────────────────────────────────────────────────────────────────────────
PcurveConstruction constructPcurve(const FaceSurface& surface, const Location& surfLoc,
                                   const EdgeCurve& edgeCurve, const Location& edgeLoc,
                                   double first, double last,
                                   double u0, double u1, double v0, double v1,
                                   const ConstructOptions& opts) {
  namespace nn = cybercad::native::numerics;
  PcurveConstruction out;
  const int m = std::max(2, opts.samples);
  const double span = last - first;

  // A world-placed surface evaluator for the projector.
  nn::SurfaceEval surfEval = [&](double u, double v) {
    return placePoint(surfLoc, surfaceLocal(surface, u, v));
  };

  // Sample the 3-D edge, project each point to (u,v), collect the UV path.
  std::vector<math::Point3> uvPts;   // (u,v,0) for the 2-D fit
  uvPts.reserve(static_cast<std::size_t>(m + 1));
  double projMax = 0.0;
  for (int i = 0; i <= m; ++i) {
    const double t = first + span * (static_cast<double>(i) / m);
    const math::Point3 world = placePoint(edgeLoc, edgeCurveLocal(edgeCurve, t));
    const nn::SurfaceProjection pr =
        nn::closest_point_on_surface(surfEval, u0, u1, v0, v1, world,
                                     opts.surfSamplesU, opts.surfSamplesV);
    if (!pr.success) return out;  // projection failed → honest decline
    projMax = std::max(projMax, pr.distance);
    uvPts.push_back(math::Point3{pr.u, pr.v, 0.0});
  }
  out.projMaxDistance = projMax;

  // If the edge does not actually lie on S the projection residual is large — decline
  // rather than fit a pcurve to off-surface feet. The acceptance band is a small
  // fraction of the WORLD extent of the sampled edge feet (scale-relative, independent
  // of the round-trip fidelity tolerance): an edge lying on S projects to ~0, a genuinely
  // off-surface edge to a residual comparable to its distance from S (≫ this band).
  double worldExtent = 0.0;
  for (std::size_t i = 1; i < uvPts.size(); ++i) {
    const math::Point3 a = surfEval(uvPts[i - 1].x, uvPts[i - 1].y);
    const math::Point3 b = surfEval(uvPts[i].x, uvPts[i].y);
    worldExtent += math::distance(a, b);
  }
  const double projTol = 1e-2 * std::max(worldExtent, 1.0);
  if (projMax > projTol) return out;  // edge is not on S → honest decline

  // Fit a 2-D B-spline through the projected UV path (interpolation → passes through
  // every projected foot). Degree clamped so degree+1 ≤ #points.
  const int degree = std::clamp(opts.fitDegree, 1, static_cast<int>(uvPts.size()) - 1);
  const math::CurveFitResult fit =
      math::interpolateCurve({uvPts.data(), uvPts.size()}, degree,
                             math::ParamMethod::ChordLength);
  if (!fit.ok) return out;  // fit failed → honest decline

  // Assemble the PCurve payload from the fitted 2-D curve. The fit parametrizes on
  // [0,1]; reparametrize the knots onto the edge's [first,last] so pcurve(t) is
  // evaluated at the SAME parameter as C(t) (the fidelity contract).
  PCurve pc;
  pc.kind = EdgeCurve::Kind::BSpline;
  pc.degree = fit.curve.degree;
  pc.poles2d = fit.curve.poles;                 // (u,v,0) poles
  pc.weights = fit.curve.weights;               // empty ⇒ non-rational
  pc.knots = fit.curve.knots;
  for (double& k : pc.knots) k = first + (last - first) * k;  // [0,1] → [first,last]
  if (!pc.poles2d.empty()) {
    pc.origin2d = pc.poles2d.front();
    pc.dir2d = math::Vec3{1, 0, 0};
  }
  out.pcurve = pc;

  // Round-trip fidelity: S(pcurve(t)) must reproduce C(t) over [first,last].
  out.fidelity = pcurveFidelity(surface, surfLoc, edgeCurve, edgeLoc, pc, first, last,
                                opts.fidelity);
  out.ok = out.fidelity.ok;
  return out;
}
#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::topology

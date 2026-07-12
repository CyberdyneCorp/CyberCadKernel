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

// UV distance between two param points.
double dist2d(const ParamPoint& a, const ParamPoint& b) noexcept {
  const double du = a.u - b.u, dv = a.v - b.v;
  return std::sqrt(du * du + dv * dv);
}

// Flatten a loop for HEALING and record the raw gap at every segment JOIN.
//   * The polyline keeps every sampled vertex (only EXACT bit-identical consecutive dups
//     from oversampling a single segment are dropped) — so a small inter-segment gap of,
//     say, 1e-7 between one segment's end and the next segment's start stays VISIBLE (the
//     production flattenLoop's 1e-15 dedup would erase gaps below 1e-15 only, but the raw
//     form here also feeds the join-gap vector below).
//   * joinGaps[k] = ‖end of segment k − start of segment k+1‖ for k=0..S-2, and the last
//     entry = ‖end of the LAST segment − start of the FIRST segment‖ (the closing join).
//     This is what distinguishes a real GAP from an ordinary long shape edge.
// Append a sampled vertex, skipping an EXACT bit-identical consecutive dup (oversampling of
// one segment — not a gap; a real gap is a nonzero step that survives).
void appendUnique(std::vector<ParamPoint>& poly, const ParamPoint& p) {
  if (!poly.empty() && p.u == poly.back().u && p.v == poly.back().v) return;
  poly.push_back(p);
}

std::vector<ParamPoint> flattenLoopForHeal(const TrimLoop& loop, int segsPerSegment,
                                           std::vector<double>& joinGaps) {
  std::vector<ParamPoint> poly;
  joinGaps.clear();
  if (loop.empty()) return poly;
  const int n = std::max(2, segsPerSegment);
  const ParamPoint firstStart = segmentValue(loop.front(), 0.0);
  ParamPoint prevEnd = firstStart;
  for (std::size_t s = 0; s < loop.size(); ++s) {
    const PcurveSegment& seg = loop[s];
    if (s > 0) joinGaps.push_back(dist2d(prevEnd, segmentValue(seg, 0.0)));  // gap into seg
    for (int i = 0; i <= n; ++i)
      appendUnique(poly, segmentValue(seg, static_cast<double>(i) / n));
    prevEnd = segmentValue(seg, 1.0);
  }
  joinGaps.push_back(dist2d(prevEnd, firstStart));  // closing join
  return poly;
}

}  // namespace

// Public thin wrapper over the anonymous flattenLoop so other topology TUs (the
// parameter-space region boolean) share the exact, seam-consistent pcurve evaluator.
std::vector<ParamPoint> flattenTrimLoop(const TrimLoop& loop, int segsPerSegment) {
  return flattenLoop(loop, segsPerSegment);
}

// ─────────────────────────────────────────────────────────────────────────────
// healLoop — tolerant-topology healing on a flattened loop polyline.
//
// REGION-PRESERVATION PROOF (why a heal never flips a classification):
//   Every weld moves a vertex by at most gapTol/2 (two vertices ≤ gapTol apart are merged
//   to their midpoint). A ray-cast verdict for a point P changes only if the boundary
//   crosses P, i.e. only if some boundary point moves across P. No boundary vertex moves
//   farther than gapTol/2, so a P whose distance to the boundary exceeds gapTol/2 keeps
//   its verdict. Points within gapTol of the boundary are the OnBoundary band anyway
//   (classify uses onEdgeTol; a heal is only enabled when gapTol ≥ onEdgeTol scale). A
//   genuine large gap is NOT welded (declined) and a pinch is NOT split (declined) — so a
//   heal only ever nudges an already-near-valid loop; it never re-routes the boundary.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Pass 0 — join-gap triage. A gap at a segment JOIN that exceeds tol is a GENUINE large gap
// (the loop is not closeable within tolerance): the ONLY reliable way to tell a real gap
// from a long shape edge (a rectangle's long side is a huge step but is NOT a gap). Records
// largeGap/residualGap in `rep`; returns true iff a large gap forces a decline.
bool healTriageLargeGap(const std::vector<double>& joinGaps, double tol, HealReport& rep) {
  for (const double g : joinGaps)
    if (g > tol) {
      rep.largeGap = true;
      rep.residualGap = std::max(rep.residualGap, g);
    }
  return rep.largeGap;
}

// Pass 1 — weld small consecutive gaps to their midpoint (region-preserving: each vertex
// moves ≤ tol/2), including the loop's closing edge. Returns the welded polyline; updates
// gapsClosed / maxGapClosed / changed in `rep`.
std::vector<ParamPoint> healWeldGaps(const std::vector<ParamPoint>& poly, double tol,
                                     HealReport& rep) {
  std::vector<ParamPoint> out;
  out.reserve(poly.size());
  for (const ParamPoint& p : poly) {
    const double d = out.empty() ? -1.0 : dist2d(out.back(), p);
    if (d > 0.0 && d <= tol) {  // a small gap → weld previous vertex to the midpoint
      out.back() = ParamPoint{0.5 * (out.back().u + p.u), 0.5 * (out.back().v + p.v)};
      rep.gapsClosed += 1;
      rep.maxGapClosed = std::max(rep.maxGapClosed, d);
      rep.changed = true;
      continue;
    }
    out.push_back(p);
  }
  // Closing edge: weld first/last if within tol (the loop's implicit closure).
  if (out.size() >= 2) {
    const double d = dist2d(out.front(), out.back());
    if (d == 0.0) { out.pop_back(); rep.changed = true; }  // already coincident
    else if (d <= tol) {
      out.front() = ParamPoint{0.5 * (out.front().u + out.back().u),
                               0.5 * (out.front().v + out.back().v)};
      out.pop_back();
      rep.gapsClosed += 1;
      rep.maxGapClosed = std::max(rep.maxGapClosed, d);
      rep.changed = true;
    }
  }
  return out;
}

// Pass 2 — pinch detection: a repeated NON-adjacent vertex (within tol) means the loop
// self-touches. Returns true iff a pinch is found.
bool healHasPinch(const std::vector<ParamPoint>& out, double tol) {
  const std::size_t n = out.size();
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t k = i + 2; k < n; ++k) {
      if (i == 0 && k == n - 1) continue;  // adjacent across the closing edge
      if (dist2d(out[i], out[k]) <= tol) return true;
    }
  return false;
}

}  // namespace

HealReport healLoop(std::vector<ParamPoint>& poly, const std::vector<double>& joinGaps,
                    const HealOptions& opts) {
  HealReport rep;
  if (poly.size() < 2) {  // nothing to weld; validity decided by the raycast/wellformed
    rep.healed = poly.size() >= 3;
    return rep;
  }

  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = opts.gapTol * std::max(extent, 1.0);
  rep.tolerance = tol;

  if (healTriageLargeGap(joinGaps, tol, rep)) return rep;  // genuine large gap → decline

  std::vector<ParamPoint> out = healWeldGaps(poly, tol, rep);  // weld small gaps + closure
  poly.swap(out);

  if (poly.size() < 3) return rep;                 // collapsed → degenerate, honest decline
  if (healHasPinch(poly, tol)) { rep.pinch = true; return rep; }  // self-touch → decline

  rep.healed = true;  // welded (if needed) and non-self-touching → valid to classify
  return rep;
}

// healTrimLoop — flatten (join-gap-aware) + heal, discarding the healed polyline.
HealReport healTrimLoop(const TrimLoop& loop, const HealOptions& opts, int flatten) {
  std::vector<double> joinGaps;
  std::vector<ParamPoint> poly = flattenLoopForHeal(loop, flatten, joinGaps);
  return healLoop(poly, joinGaps, opts);
}

// ─────────────────────────────────────────────────────────────────────────────
// splitAtPinch — resolve a CLEAN 2-way pinch into two region-preserving sub-loops.
//
// REGION-PRESERVATION PROOF (why the UNION of the two sub-loops classifies identically to
// the original pinched loop): a non-crossing loop that self-touches at a single vertex P is
// a figure-eight whose two lobes A and B are disjoint simple regions meeting only at P. The
// original even-odd ray-cast of the WHOLE loop counts, for a query point Q, the parity of
// boundary crossings; because A and B are traced by disjoint edge sets that share only P,
// the crossing count for Q equals (crossings of A) + (crossings of B). Q is inside exactly
// one lobe (the lobes are disjoint) or neither, so the original parity is odd ⇔ Q ∈ A XOR
// Q ∈ B ⇔ Q ∈ A OR Q ∈ B (mutual exclusivity). Therefore the ORIGINAL In-set = A-interior
// ∪ B-interior: classifying the union of the two split sub-loops reproduces the original
// verdict for EVERY interior/exterior point. Only a clean 2-way, non-crossing pinch has
// this property — a 3+-way or self-crossing pinch is declined (ambiguous), never forced.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Collect every distinct non-adjacent coincident vertex pair (i,k), i<k, within tol.
std::vector<std::pair<std::size_t, std::size_t>> pinchPairs(const std::vector<ParamPoint>& poly,
                                                            double tol) {
  std::vector<std::pair<std::size_t, std::size_t>> pairs;
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i)
    for (std::size_t k = i + 2; k < n; ++k) {
      if (i == 0 && k == n - 1) continue;  // adjacent across the closing edge
      if (dist2d(poly[i], poly[k]) <= tol) pairs.emplace_back(i, k);
    }
  return pairs;
}

// A sub-loop is valid to ray-cast iff it has ≥3 distinct points and does NOT itself pinch
// (no residual self-touch within tol) — so a lobe that is itself non-manifold is rejected.
bool subLoopValid(const std::vector<ParamPoint>& poly, double tol) {
  if (poly.size() < 3) return false;
  return !healHasPinch(poly, tol);
}

}  // namespace

SplitReport splitAtPinch(const std::vector<ParamPoint>& poly, const HealOptions& opts) {
  SplitReport rep;
  const std::size_t n = poly.size();
  if (n < 4) return rep;  // too small to hold two non-degenerate lobes

  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = opts.gapTol * std::max(extent, 1.0);
  rep.tolerance = tol;

  const auto pairs = pinchPairs(poly, tol);
  rep.pinchCount = static_cast<int>(pairs.size());
  if (pairs.empty()) return rep;  // no pinch → split=false, pinch=false (honest: not a pinch)
  rep.pinch = true;

  // A clean 2-way pinch is EXACTLY ONE coincident pair. More than one pair means a 3+-way
  // pinch or two separate self-touches — ambiguous, decline honestly.
  if (pairs.size() != 1) { rep.ambiguous = true; return rep; }

  const std::size_t i = pairs.front().first;
  const std::size_t k = pairs.front().second;

  // Snap both pinch vertices to their shared midpoint so the two sub-loops meet EXACTLY at
  // one point (region-preserving: each moves ≤ tol/2, like a weld).
  const ParamPoint pinchPt{0.5 * (poly[i].u + poly[k].u), 0.5 * (poly[i].v + poly[k].v)};

  // Lobe A = poly[i .. k-1] with the pinch vertex as its shared representative; closes k-1→i.
  std::vector<ParamPoint> loopA;
  loopA.reserve(k - i);
  loopA.push_back(pinchPt);
  for (std::size_t t = i + 1; t < k; ++t) loopA.push_back(poly[t]);

  // Lobe B = poly[k .. n-1] + poly[0 .. i-1]; closes i-1→k. Shared pinch vertex again.
  std::vector<ParamPoint> loopB;
  loopB.reserve(n - (k - i));
  loopB.push_back(pinchPt);
  for (std::size_t t = k + 1; t < n; ++t) loopB.push_back(poly[t]);
  for (std::size_t t = 0; t < i; ++t) loopB.push_back(poly[t]);

  // Each lobe must be a valid simple loop (≥3 distinct, no residual self-touch); otherwise
  // the pinch is not a clean figure-eight (e.g. a crossing) → ambiguous decline.
  if (!subLoopValid(loopA, tol) || !subLoopValid(loopB, tol)) {
    rep.ambiguous = true;
    return rep;
  }

  rep.loopA = std::move(loopA);
  rep.loopB = std::move(loopB);
  rep.split = true;
  return rep;
}

// splitTrimLoopAtPinch — flatten (join-gap-aware) + weld small gaps + split at a 2-way pinch.
SplitReport splitTrimLoopAtPinch(const TrimLoop& loop, const HealOptions& opts, int flatten) {
  std::vector<double> joinGaps;
  std::vector<ParamPoint> poly = flattenLoopForHeal(loop, flatten, joinGaps);
  SplitReport rep;
  if (poly.size() < 4) return rep;
  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = opts.gapTol * std::max(extent, 1.0);
  // A genuine large gap makes the loop uncloseable — cannot split honestly.
  HealReport hg;
  if (healTriageLargeGap(joinGaps, tol, hg)) { rep.tolerance = tol; return rep; }
  HealReport wr;
  std::vector<ParamPoint> welded = healWeldGaps(poly, tol, wr);  // close small gaps first
  return splitAtPinch(welded, opts);
}

// ─────────────────────────────────────────────────────────────────────────────
// splitAtPinches — GENERAL N-way / crossing pinch resolution by CCW-adjacency.
//
// A welded polyline V[0..n-1] traces one closed loop with edges V[i]→V[next(i)] (initially
// next(i)=(i+1) mod n). Where several vertices coincide (within tol) the loop self-touches:
// that location is a PINCH vertex through which N≥2 strands pass. We resolve ONE pinch cluster
// per pass by RE-ROUTING the successor links so that each INCOMING strand continues along its
// CCW-adjacent OUTGOING strand (the planar-subdivision rotational-order rule); tracing the
// re-routed links then yields simple sub-loops that meet only at pinch points. Crossing
// pinches (a figure-8-of-figure-8) are handled by iterating the single-cluster resolution over
// the emitted sub-loops to a FIXPOINT. Region- and area-preserving (see the header proof): the
// reconnection only re-partitions the SAME directed edges into cycles, so total signed area and
// even-odd parity are invariant. A pinch whose in/out strands do not alternate around P (a non-
// manifold touch no CCW pairing resolves) is DECLINED (ambiguous), never force-split.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Signed area (shoelace) of a closed polyline — used to prove area preservation.
double signedArea(const std::vector<ParamPoint>& poly) noexcept {
  const std::size_t n = poly.size();
  if (n < 3) return 0.0;
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += (poly[j].u * poly[i].v) - (poly[i].u * poly[j].v);
  return 0.5 * a;
}

// Find the FIRST pinch cluster: the earliest vertex whose location repeats (within tol) at ≥1
// other NON-adjacent vertex. Returns the sorted index list of the cluster (≥2), or empty if
// the loop has no pinch. Adjacency across the closing edge (0,n-1) is not a pinch.
std::vector<std::size_t> firstPinchCluster(const std::vector<ParamPoint>& poly, double tol) {
  const std::size_t n = poly.size();
  std::vector<bool> used(n, false);
  for (std::size_t i = 0; i < n; ++i) {
    if (used[i]) continue;
    std::vector<std::size_t> cluster{i};
    for (std::size_t k = i + 1; k < n; ++k)
      if (!used[k] && dist2d(poly[i], poly[k]) <= tol) cluster.push_back(k);
    if (cluster.size() >= 2) {
      // Exclude a pure closing-edge adjacency (only i=0 & k=n-1, nothing else): not a pinch.
      bool onlyClosing = cluster.size() == 2 && cluster[0] == 0 && cluster[1] == n - 1;
      if (!onlyClosing) return cluster;
    }
    for (std::size_t c : cluster) used[c] = true;
  }
  return {};
}

// Signed CCW turn (in (−π, π]) from travel direction `inDir` (arriving) to `outDir` (leaving).
double signedTurn(double inDir, double outDir) noexcept {
  double t = outDir - inDir;
  while (t <= -M_PI) t += 2.0 * M_PI;
  while (t > M_PI) t -= 2.0 * M_PI;
  return t;
}

// Re-route `next` at ONE pinch cluster by the LEFTMOST-TURN rule, then trace the resulting
// cycles. Returns the emitted sub-loops; sets `ok=false` if the pairing is not a bijection.
//
// MODEL. Each cluster member c is one VISIT to the pinch point P: the loop ARRIVES via the edge
// origPrv[c]→c (travel direction INTO P = P − origPrv[c]) and DEPARTS via c→origNxt[c] (travel
// direction OUT of P = origNxt[c] − P). We keep the ORIGINAL next/prev of the whole loop and
// only re-wire the departures AT the cluster: each INCOMING strand is paired with the OUTGOING
// strand that makes the LARGEST signed CCW turn (the "leftmost turn" / smallest right turn). This
// is the standard orientation-preserving face-tracing rule for a planar subdivision: it keeps the
// interior consistently on one side, so tracing the re-wired links yields SIMPLE sub-loops whose
// SIGNED areas sum to the original (region- and area-preserving). Concretely we set a join map
// depart[c] = origNxt[pairedMember]; every non-cluster vertex keeps origNxt. Tracing that mapping
// walks each sub-loop; every cluster visit contributes exactly ONE copy of P.
std::vector<std::vector<ParamPoint>> resolveCluster(const std::vector<ParamPoint>& poly,
                                                    const std::vector<std::size_t>& cluster,
                                                    double tol, bool& ok) {
  ok = true;
  const std::size_t n = poly.size();
  std::vector<std::size_t> origNxt(n), origPrv(n);
  for (std::size_t i = 0; i < n; ++i) { origNxt[i] = (i + 1) % n; origPrv[i] = (i + n - 1) % n; }

  // The shared pinch point (average of the cluster members → they meet EXACTLY at one point).
  ParamPoint P{0.0, 0.0};
  for (std::size_t c : cluster) { P.u += poly[c].u; P.v += poly[c].v; }
  P.u /= static_cast<double>(cluster.size());
  P.v /= static_cast<double>(cluster.size());

  auto isClusterMember = [&](std::size_t idx) {
    return std::find(cluster.begin(), cluster.end(), idx) != cluster.end();
  };

  // Incoming TRAVEL direction (into P) and outgoing TRAVEL direction (out of P) per member.
  struct HalfEdge { std::size_t member; double dir; };
  std::vector<HalfEdge> ins, outs;
  for (std::size_t c : cluster) {
    const ParamPoint& u = poly[origPrv[c]];
    const ParamPoint& w = poly[origNxt[c]];
    ins.push_back({c, std::atan2(P.v - u.v, P.u - u.u)});    // travel INTO P
    outs.push_back({c, std::atan2(w.v - P.v, w.u - P.u)});   // travel OUT of P
  }

  // Pair each incoming with the outgoing of LARGEST signed CCW turn (leftmost turn).
  // depart[c] = the vertex the loop continues to after ARRIVING at cluster member c.
  std::vector<std::size_t> depart(n);
  for (std::size_t i = 0; i < n; ++i) depart[i] = origNxt[i];  // default: unchanged
  std::vector<bool> outTaken(outs.size(), false);
  for (const HalfEdge& in : ins) {
    int best = -1;
    double bestTurn = -1e300;
    for (std::size_t j = 0; j < outs.size(); ++j) {
      if (outTaken[j]) continue;
      const double turn = signedTurn(in.dir, outs[j].dir);
      if (turn > bestTurn) { bestTurn = turn; best = static_cast<int>(j); }
    }
    if (best < 0) { ok = false; return {}; }  // no outgoing left → not a bijection (ambiguous)
    outTaken[best] = true;
    // Arriving at incoming member `in.member`, continue to the off-cluster neighbour of the
    // paired outgoing member (skipping the paired member's own P-visit — c already IS P).
    depart[in.member] = origNxt[outs[best].member];
  }

  // Trace cycles over `depart`, starting from every not-yet-visited vertex.
  std::vector<std::vector<ParamPoint>> loops;
  std::vector<bool> seen(n, false);
  for (std::size_t start = 0; start < n; ++start) {
    if (seen[start]) continue;
    std::vector<ParamPoint> loop;
    std::size_t cur = start;
    std::size_t guard = 0;
    while (!seen[cur] && guard <= n) {
      seen[cur] = true;
      // A cluster member is snapped to the shared pinch point P (region-preserving: the members
      // are within tol of P). Non-cluster vertices pass through unchanged.
      loop.push_back(isClusterMember(cur) ? P : poly[cur]);
      cur = depart[cur];
      ++guard;
    }
    // De-duplicate consecutive coincident points (a re-routed join may repeat P) and drop a
    // closing duplicate.
    std::vector<ParamPoint> compact;
    for (const ParamPoint& p : loop)
      if (compact.empty() || dist2d(compact.back(), p) > tol) compact.push_back(p);
    if (compact.size() >= 2 && dist2d(compact.front(), compact.back()) <= tol) compact.pop_back();
    if (!compact.empty()) loops.push_back(std::move(compact));
  }
  return loops;
}

}  // namespace

MultiSplitReport splitAtPinches(const std::vector<ParamPoint>& poly, const HealOptions& opts) {
  MultiSplitReport rep;
  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = opts.gapTol * std::max(extent, 1.0);
  rep.tolerance = tol;
  if (poly.size() < 3) return rep;  // degenerate → decline

  // No pinch at all → the loop is already simple; return it verbatim (ok, region unchanged).
  if (firstPinchCluster(poly, tol).empty()) {
    rep.ok = true;
    rep.loops.push_back(poly);
    return rep;
  }
  rep.pinch = true;

  const double areaBefore = signedArea(poly);  // SIGNED — the invariant preserved by re-routing

  // Fixpoint: repeatedly resolve the FIRST pinch cluster in each loop until none self-touches.
  // A crossing (figure-8-of-figure-8) needs several passes; each pass resolves one cluster per
  // still-pinched loop and the emitted sub-loops are re-fed until every loop is simple.
  std::vector<std::vector<ParamPoint>> work{poly};
  std::vector<std::vector<ParamPoint>> done;
  const std::size_t kMaxIter = 64;  // bounded; crossing pinches converge quickly
  std::size_t iter = 0;
  while (!work.empty() && iter < kMaxIter) {
    ++iter;
    std::vector<std::vector<ParamPoint>> nextWork;
    for (const std::vector<ParamPoint>& loop : work) {
      if (loop.size() < 3) { rep.ambiguous = true; return rep; }  // collapsed → honest decline
      const std::vector<std::size_t> cluster = firstPinchCluster(loop, tol);
      if (cluster.empty()) { done.push_back(loop); continue; }  // already simple
      rep.maxWays = std::max(rep.maxWays, static_cast<int>(cluster.size()));
      rep.pinchVertices += 1;
      bool ok = false;
      std::vector<std::vector<ParamPoint>> subs = resolveCluster(loop, cluster, tol, ok);
      if (!ok || subs.empty()) { rep.ambiguous = true; return rep; }  // non-alternating → decline
      // Non-progress guard: a re-routing that returns the SAME single loop (still self-touching)
      // means the pinch could not be un-crossed at this vertex → ambiguous, honest decline.
      if (subs.size() == 1 && subs.front().size() >= loop.size() &&
          !firstPinchCluster(subs.front(), tol).empty()) {
        rep.ambiguous = true;
        return rep;
      }
      for (std::vector<ParamPoint>& s : subs) nextWork.push_back(std::move(s));
    }
    work.swap(nextWork);
  }
  if (!work.empty()) { rep.ambiguous = true; return rep; }  // did not converge → honest decline
  rep.iterations = static_cast<int>(iter);

  // Every emitted sub-loop must be a valid SIMPLE loop (≥3 distinct, no residual self-touch).
  double areaAfter = 0.0;
  for (const std::vector<ParamPoint>& s : done) {
    if (!subLoopValid(s, tol)) { rep.ambiguous = true; return rep; }
    areaAfter += signedArea(s);  // SIGNED sum
  }
  // SIGNED-area preservation guard: re-routing only re-partitions the same directed edges into
  // cycles, so the sum of the sub-loops' SIGNED areas equals the original loop's signed area. A
  // mismatch means the re-routing produced wrong loops → honest decline (never a fabricated
  // region). |sub-loops| may individually have either winding (a lobe can be a hole).
  if (std::fabs(areaAfter - areaBefore) > std::max(1e-9, 1e-6 * std::max(std::fabs(areaBefore), 1.0))) {
    rep.ambiguous = true;
    return rep;
  }

  rep.loops = std::move(done);
  rep.ok = !rep.loops.empty();
  return rep;
}

// splitTrimLoopAtPinches — flatten (join-gap-aware) + weld small gaps + N-way pinch split.
MultiSplitReport splitTrimLoopAtPinches(const TrimLoop& loop, const HealOptions& opts, int flatten) {
  std::vector<double> joinGaps;
  std::vector<ParamPoint> poly = flattenLoopForHeal(loop, flatten, joinGaps);
  MultiSplitReport rep;
  if (poly.size() < 3) return rep;
  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = opts.gapTol * std::max(extent, 1.0);
  HealReport hg;
  if (healTriageLargeGap(joinGaps, tol, hg)) { rep.tolerance = tol; return rep; }
  HealReport wr;
  std::vector<ParamPoint> welded = healWeldGaps(poly, tol, wr);
  return splitAtPinches(welded, opts);
}

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
namespace {

// Prepare one loop's raycast polyline. With healing on, flatten (join-gap-aware), heal,
// and use the healed polyline iff the heal succeeded; a large gap / pinch / degeneracy
// makes `ok` false (→ the caller declines Unknown, honestly). With healing off, use the
// production flattenLoop + loopWellFormed exactly as before (no behaviour change).
std::vector<ParamPoint> preparedLoop(const TrimLoop& loop, const ClassifyOptions& opts,
                                     bool& ok) {
  if (!opts.heal) {
    std::vector<ParamPoint> poly = flattenLoop(loop, opts.flattenSegments);
    ok = loopWellFormed(poly);
    return poly;
  }
  std::vector<double> joinGaps;
  std::vector<ParamPoint> poly = flattenLoopForHeal(loop, opts.flattenSegments, joinGaps);
  const HealReport rep = healLoop(poly, joinGaps, HealOptions{opts.healGapTol});
  ok = rep.healed && loopWellFormed(poly);
  return poly;
}

// Ray-cast a prepared polyline and map the raw code to a Containment. `2`→OnBoundary,
// `3`→Unknown, `1`→In, `0`→Out (used for the outer-loop verdict, including split lobes).
Containment classifyAgainstLoop(const std::vector<ParamPoint>& poly, const ParamPoint& p,
                                double onEdgeTol) {
  const double extent = std::max(polyExtent(poly), 1e-300);
  const double tol = onEdgeTol * std::max(extent, 1.0);
  switch (raycast(poly, p, tol)) {
    case 2: return Containment::OnBoundary;
    case 3: return Containment::Unknown;
    case 1: return Containment::In;
    default: return Containment::Out;
  }
}

// Outer-loop verdict with the OPT-IN pinch split. If the outer loop heals cleanly, use it.
// Otherwise, when splitPinch is on, try to SPLIT a clean 2-way pinch and classify the UNION
// of the two sub-loops (In iff inside either lobe; OnBoundary if on either seam) — this is
// region-preserving (see splitAtPinch). A non-splittable pinch / large gap / degeneracy is
// an honest decline (Unknown).
Containment classifyOuter(const TrimLoop& loop, const ParamPoint& p, const ClassifyOptions& opts) {
  bool ok = false;
  const std::vector<ParamPoint> outer = preparedLoop(loop, opts, ok);
  if (ok) return classifyAgainstLoop(outer, p, opts.onEdgeTol);
  if (!opts.heal || !opts.splitPinch) return Containment::Unknown;  // no split path → decline

  const SplitReport sr = splitTrimLoopAtPinch(loop, HealOptions{opts.healGapTol},
                                              opts.flattenSegments);
  if (sr.split) {
    const Containment ca = classifyAgainstLoop(sr.loopA, p, opts.onEdgeTol);
    const Containment cb = classifyAgainstLoop(sr.loopB, p, opts.onEdgeTol);
    if (ca == Containment::OnBoundary || cb == Containment::OnBoundary)
      return Containment::OnBoundary;
    if (ca == Containment::Unknown || cb == Containment::Unknown) return Containment::Unknown;
    // Region = union of the two disjoint lobes: In iff inside either lobe.
    return (ca == Containment::In || cb == Containment::In) ? Containment::In : Containment::Out;
  }
  if (!opts.splitNWay) return Containment::Unknown;  // 2-way declined, N-way opt-out → decline

  // GENERAL N-way / crossing pinch: decompose into a set of simple sub-loops and classify by the
  // EVEN-ODD parity of the sub-loops (In iff inside an ODD number of them; OnBoundary if on any
  // seam). Because the sub-loops partition the SAME directed edges as the original loop, the
  // parity of crossings — hence the even-odd verdict — is IDENTICAL to the original self-touching
  // loop (region-preserving). For disjoint lobes this reduces to "In iff inside any lobe".
  const MultiSplitReport ms = splitTrimLoopAtPinches(loop, HealOptions{opts.healGapTol},
                                                     opts.flattenSegments);
  if (!ms.ok) return Containment::Unknown;  // unresolvable pinch → honest decline
  bool parityIn = false;
  for (const std::vector<ParamPoint>& sub : ms.loops) {
    const Containment c = classifyAgainstLoop(sub, p, opts.onEdgeTol);
    if (c == Containment::OnBoundary) return Containment::OnBoundary;
    if (c == Containment::Unknown) return Containment::Unknown;
    if (c == Containment::In) parityIn = !parityIn;  // even-odd XOR
  }
  return parityIn ? Containment::In : Containment::Out;
}

}  // namespace

Containment classify(const TrimmedNurbsFace& face, const ParamPoint& p,
                     const ClassifyOptions& opts) {
  if (!face.hasOuter()) return Containment::Unknown;

  const Containment outerVerdict = classifyOuter(face.outer, p, opts);
  if (outerVerdict == Containment::OnBoundary) return Containment::OnBoundary;
  if (outerVerdict == Containment::Unknown) return Containment::Unknown;
  if (outerVerdict == Containment::Out) return Containment::Out;  // outside the outer region

  // Inside the outer loop: a point inside ANY hole is Out; on a hole edge is
  // OnBoundary; a hole ambiguity is Unknown.
  for (const TrimLoop& hole : face.holes) {
    bool hok = false;
    const std::vector<ParamPoint> hpoly = preparedLoop(hole, opts, hok);
    if (!hok) return Containment::Unknown;
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
namespace {

// Is a 3-D edge RATIONAL? A NURBS/Bezier edge with a parallel weight vector is; the analytic
// Circle/Ellipse are trigonometric (weight ≡ 1 in their own trig parameter — NOT rational in t),
// so they are treated as non-rational for the PCURVE parametrization contract (pcurve(t)==C(t)
// at the same t requires the edge to be a genuine rational polynomial in t).
bool edgeIsRational(const EdgeCurve& c) noexcept {
  using K = EdgeCurve::Kind;
  return (c.kind == K::BSpline || c.kind == K::Bezier) && !c.weights.empty() &&
         c.weights.size() == c.poles.size();
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// constructPcurve (numsci-gated) — project sampled edge points to (u,v) and fit a 2-D pcurve,
// then round-trip-verify fidelity. When the 3-D edge is RATIONAL and opts.rational is set, the
// fit is a WEIGHTED (homogeneous) interpolation: each projected (u,v) foot inherits the edge's
// rational denominator w(t) at that parameter, so the constructed pcurve is a RATIONAL NURBS
// that reproduces a rational (e.g. circular) trim curve EXACTLY — no polygonal sag.
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

  // A rational edge (with opts.rational on) builds a RATIONAL pcurve via homogeneous weighting.
  const bool wantRational = opts.rational && edgeIsRational(edgeCurve);

  // Sample the 3-D edge, project each point to (u,v), collect the UV path (used for the on-S
  // residual test and the non-rational fit; the rational path additionally projects the control
  // net below).
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

  PCurve pc;
  pc.kind = EdgeCurve::Kind::BSpline;

  if (wantRational) {
    // EXACT rational pcurve. The fidelity contract requires pcurve(t) == C(t) at the SAME t, so
    // the pcurve must inherit the edge's EXACT parametrization (its degree, knots and weights) —
    // a fresh chord-length fit would reparametrize and only approximate. For a surface whose
    // parametrization is affine in (u,v) over the edge's region (a plane exactly; the height
    // direction of a cylinder), the (u,v) preimage of the rational edge is the SAME rational
    // curve with each control POLE mapped to its (u,v) foot and the weights/knots unchanged. We
    // therefore PROJECT the edge's control net to (u,v) and reuse the edge's weights + knots; the
    // round-trip fidelity below then CONFIRMS exactness (and honestly declines a surface where
    // this affine assumption does not hold, rather than faking a rational pcurve).
    const std::size_t np = edgeCurve.poles.size();
    pc.degree = edgeCurve.degree;
    pc.weights = edgeCurve.weights;
    pc.knots = edgeCurve.knots;
    pc.poles2d.reserve(np);
    double poleProjMax = 0.0;
    for (std::size_t i = 0; i < np; ++i) {
      const math::Point3 worldPole = placePoint(edgeLoc, edgeCurve.poles[i]);
      const nn::SurfaceProjection pr =
          nn::closest_point_on_surface(surfEval, u0, u1, v0, v1, worldPole,
                                       opts.surfSamplesU, opts.surfSamplesV);
      if (!pr.success) return out;  // a control pole could not be projected → decline
      poleProjMax = std::max(poleProjMax, pr.distance);
      pc.poles2d.push_back(math::Point3{pr.u, pr.v, 0.0});
    }
    (void)poleProjMax;  // control poles of a rational curve need NOT lie on S; fidelity is the gate
    if (!pc.poles2d.empty()) { pc.origin2d = pc.poles2d.front(); pc.dir2d = math::Vec3{1, 0, 0}; }
    out.pcurve = pc;
    out.rational = !pc.weights.empty();
    out.fidelity = pcurveFidelity(surface, surfLoc, edgeCurve, edgeLoc, pc, first, last,
                                  opts.fidelity);
    if (out.fidelity.ok) { out.ok = true; return out; }
    // The affine assumption did not hold (curved-in-(u,v) surface): fall through to the general
    // non-rational fit, which reports its honest (possibly larger) fidelity deviation.
    pc = PCurve{};
    pc.kind = EdgeCurve::Kind::BSpline;
    out.rational = false;
  }

  // Non-rational (or rational-fallback) 2-D interpolation through the projected UV path
  // (passes through every projected foot). Degree clamped so degree+1 ≤ #points.
  //
  // PARAMETER-ALIGNED FIT (the boolean-grade contract). The edge C(t) is sampled at UNIFORM
  // t over [first,last] (t_k = first + span·k/m), so its k-th foot uvPts[k] sits at parameter
  // fraction k/m. We fit the pcurve with those SAME uniform parameters (k/m) — NOT a fresh
  // chord-length reparam of the projected feet — so pcurve's parameter fraction matches C's:
  // pcurve(first+span·a) lands on the foot of C(first+span·a) at EVERY a, not just at the
  // sampled knots. (A chord-length reparam would place its interior parameters by the projected
  // UV arc length, which drifts from the edge's uniform-t parameter between samples → the 0.026
  // round-trip mismatch the readiness doc measured.) The knots are then remapped [0,1]→[first,
  // last] so pcurve(t) is evaluated at the SAME t as C(t). Exact for a polynomial edge; a
  // rational edge fitted here keeps its honest (non-zero) deviation — never a widened tolerance.
  const int degree = std::clamp(opts.fitDegree, 1, static_cast<int>(uvPts.size()) - 1);
  std::vector<double> fitParams(uvPts.size());
  for (std::size_t i = 0; i < uvPts.size(); ++i)
    fitParams[i] = static_cast<double>(i) / static_cast<double>(uvPts.size() - 1);
  const math::CurveFitResult fit = math::interpolateCurveWithParams(
      {uvPts.data(), uvPts.size()}, {fitParams.data(), fitParams.size()}, degree);
  if (!fit.ok) return out;  // fit failed → honest decline

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
  out.rational = !pc.weights.empty();

  // Round-trip fidelity: S(pcurve(t)) must reproduce C(t) over [first,last].
  out.fidelity = pcurveFidelity(surface, surfLoc, edgeCurve, edgeLoc, pc, first, last,
                                opts.fidelity);
  out.ok = out.fidelity.ok;
  return out;
}
#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::topology

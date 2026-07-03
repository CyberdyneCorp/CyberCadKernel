// SPDX-License-Identifier: Apache-2.0
//
// edge_mesher.h — STAGE 1 of the two-stage watertight solid mesher: the SHARED
// per-edge 1D discretization (the piece OCCT's BRepMesh builds first, before any
// face is meshed).
//
// ── WHY THIS EXISTS (the curved shared-edge gap) ─────────────────────────────
// Two faces share a topological EDGE node (topology sharing — a cube's two faces
// share one edge node; a cylinder's cap and side share one circular edge node).
// If each face samples that shared edge INDEPENDENTLY on its own UV grid, the two
// samplings only coincide when the edge is straight (grid nodes line up). For a
// CURVED shared edge (cylinder cap↔side circle, fillet-blend seams) the cap's
// sampling and the side's sampling land on DIFFERENT 3D points, so the two face
// meshes do not share boundary vertices and the welded solid is left OPEN along
// that seam (the historical "cylinder boundaryFrac≈0.119, 2-manifold-bounded-open"
// limitation).
//
// The fix, exactly as BRepMesh does it: discretize each UNIQUE edge ONCE into a
// shared list of parameter FRACTIONS f∈[0,1] across the edge's [first,last]
// range, chosen from a deflection bound on the edge's 3D curvature. BOTH adjacent
// faces then place their boundary vertices at those SAME fractions, mapped through
// each face's own pcurve to (u,v) and evaluated on that face's surface. Because a
// pcurve satisfies  S_face(pcurve(f)) = C_edge(f)  (the pcurve is the edge curve
// expressed in the face's parameter plane), the two faces produce the SAME 3D
// point at each shared fraction — so their boundary vertices coincide and the
// spatial weld fuses them into a watertight seam.
//
// This header owns:
//   * EdgeDiscretization — the shared fraction list for one edge (+ its 3D
//     polyline, cached for the deflection sizing / diagnostics).
//   * EdgeCache          — memoises EdgeDiscretization by edge TShape identity so
//     an edge shared by two faces is discretized ONCE and both faces read the
//     identical fraction list.
//
// The 3D curve is evaluated OCCT-free via src/native/math (analytic line/circle/
// ellipse; free-form via bspline/bezier). Deflection sizing uses the same sagitta
// law as the face mesher: for a curve of curvature magnitude ‖C″‖ over a param
// step Δ, chord error d ≈ ⅛‖C″‖Δ² ⇒ Δ ≤ √(8·deflection/‖C″‖).
//
// Cognitive complexity: curve eval is a Visitor-style switch (systems band, ~18,
// flagged); the discretizer and cache are short. NO OCCT.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_EDGE_MESHER_H
#define CYBERCAD_NATIVE_TESSELLATE_EDGE_MESHER_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace cybercad::native::tessellate {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

// ─────────────────────────────────────────────────────────────────────────────
// EdgeDiscretization — the shared 1D sampling of ONE topological edge.
//
// `fracs` are monotone increasing values in [0,1] (0 and 1 always present),
// the position within the edge's [first,last] range. A face maps a fraction f to
// the true curve parameter  t = first + f·(last−first)  and feeds it to that
// face's pcurve to obtain (u,v). `points` is the 3D polyline C_edge(t(f)) — the
// same for every face (used for deflection sizing and diagnostics).
// ─────────────────────────────────────────────────────────────────────────────
struct EdgeDiscretization {
  std::vector<double> fracs;         ///< shared fractions in [0,1], monotone, endpoints incl.
  std::vector<math::Point3> points;  ///< C_edge at each fraction (world-placed)

  std::size_t segmentCount() const noexcept {
    return fracs.empty() ? 0 : fracs.size() - 1;
  }
};

namespace detail {

// ── World-placed 3D evaluation of an edge curve at true parameter t ───────────
// Analytic kinds are exact; free-form kinds use the math bspline/bezier routines.
// The edge's Location is applied so the point is world-placed (as the face's
// surface evaluator also world-places), keeping the two consistent.
inline math::Point3 placePoint(const topo::Location& loc, const math::Point3& p) noexcept {
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}

// Evaluate C(t) in LOCAL coordinates for each EdgeCurve kind. Systems-band
// Visitor switch (flagged ~18): one arm per curve representation.
inline math::Point3 edgeCurveLocal(const topo::EdgeCurve& c, double t) noexcept {
  using K = topo::EdgeCurve::Kind;
  switch (c.kind) {
    case K::Line:
      // frame.origin + t·frame.x (direction stored in X, as the bridge lays it).
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
      if (c.knots.empty())  // no knots ⇒ treat poles as a Bézier control poly
        return math::bezierPoint({c.poles.data(), c.poles.size()}, t);
      return c.weights.empty()
                 ? math::curvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                    {c.knots.data(), c.knots.size()}, t)
                 : math::nurbsCurvePoint(c.degree, {c.poles.data(), c.poles.size()},
                                         {c.weights.data(), c.weights.size()},
                                         {c.knots.data(), c.knots.size()}, t);
  }
}

// Curvature magnitude ‖C″(t)‖ (local, curvature is rigid-motion invariant), via a
// central second difference of the local point. Used only to size the number of
// segments from the deflection bound.
inline double edgeCurvature(const topo::EdgeCurve& c, double t, double first,
                            double last) noexcept {
  const double span = std::fabs(last - first);
  const double h = std::max(span * 1e-4, 1e-7);
  const math::Vec3 a = edgeCurveLocal(c, t - h).asVec();
  const math::Vec3 m = edgeCurveLocal(c, t).asVec();
  const math::Vec3 b = edgeCurveLocal(c, t + h).asVec();
  return math::norm((a - 2.0 * m + b) / (h * h));
}

// Number of segments to hold the chord (sagitta) error of the edge's 3D curve
// under `deflection`: Δ ≤ √(8·defl/‖C″‖); n = ceil(span/Δ), clamped to [min,max].
// A line (‖C″‖≈0) collapses to `minSegs`; a circle/blend subdivides.
inline int edgeSegments(const topo::EdgeCurve& c, double first, double last, double deflection,
                        int minSegs, int maxSegs) noexcept {
  const double span = std::fabs(last - first);
  if (span <= 0.0) return minSegs;
  // Probe curvature at a few points across the range and take the worst.
  double worst = 0.0;
  for (int i = 0; i <= 4; ++i) {
    const double t = first + (last - first) * (i * 0.25);
    worst = std::max(worst, edgeCurvature(c, t, first, last));
  }
  if (worst <= 1e-12) return minSegs;  // straight in 3D
  const double step = std::sqrt(8.0 * deflection / worst);
  if (!(step > 0.0)) return maxSegs;
  const int n = static_cast<int>(std::ceil(span / step));
  return std::clamp(n, minSegs, maxSegs);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// EdgeCache — build-once-per-edge shared discretization store.
//
// discretize(edge, deflection) returns the SAME EdgeDiscretization for every
// Shape referencing the same edge TShape node (regardless of the wire orientation
// it is traversed in — fractions are stored in the edge's own forward [first,last]
// direction; the face flattener reverses them when the wire uses the edge
// Reversed). This is what makes two adjacent faces agree on the seam samples.
// ─────────────────────────────────────────────────────────────────────────────
class EdgeCache {
 public:
  EdgeCache(double deflection, int minSegs, int maxSegs) noexcept
      : deflection_(deflection), minSegs_(std::max(1, minSegs)), maxSegs_(std::max(1, maxSegs)) {}

  // Raise the MINIMUM segment count for the edge with the given ENDPOINTS, ahead of
  // discretize(). Used by the solid mesher's pre-pass: a STRAIGHT edge bounding a
  // TWISTED (ruled/free-form saddle) face must be subdivided to match the face's
  // chord-deflection need, even though the edge's own 3D curvature is zero.
  // Otherwise the twisted face and its flat neighbour (a planar cap) place a
  // different number of points on the (geometrically) shared edge and the seam
  // opens. The requirement is keyed by the endpoint pair (order-independent, quantized
  // to the weld grid), so it applies to the twisted face's edge AND the coincident
  // cap edge even when — as this builder does — the two faces hold SEPARATE edge
  // nodes with the same endpoints (vertex-share + spatial weld, not edge-share).
  // Both then discretize to the SAME uniform straight-line samples, which coincide
  // in 3D and weld watertight. Must be called before discretize().
  void requireMinSegs(const topo::Shape& edge, int segs) {
    if (edge.isNull() || segs <= 1) return;
    EndpointKey k;
    if (!endpointKey(edge, k)) return;
    int& cur = segsByEndpoints_[k];
    cur = std::max(cur, std::min(segs, maxSegs_));
  }

  // Shared discretization for `edge` (keyed by TShape identity). Curved edges get
  // enough segments to meet the deflection bound; straight edges get minSegs (or
  // the requireMinSegs override, when an adjacent face demanded more).
  const EdgeDiscretization& discretize(const topo::Shape& edge) {
    const topo::TShape* key = edge.tshape().get();
    if (auto it = cache_.find(key); it != cache_.end()) return it->second;
    EdgeDiscretization d = build(edge);
    return cache_.emplace(key, std::move(d)).first->second;
  }

 private:
  // Order-independent endpoint key, quantized so two coincident edges (built as
  // separate nodes) hash to the same slot. The quantum is a small fraction of the
  // weld tolerance basis so distinct vertices never collide.
  struct EndpointKey {
    long long ax, ay, az, bx, by, bz;
    bool operator==(const EndpointKey& o) const noexcept {
      return ax == o.ax && ay == o.ay && az == o.az && bx == o.bx && by == o.by && bz == o.bz;
    }
  };
  struct EndpointKeyHash {
    std::size_t operator()(const EndpointKey& k) const noexcept {
      std::size_t h = 1469598103934665603ull;
      for (long long v : {k.ax, k.ay, k.az, k.bx, k.by, k.bz}) {
        h ^= static_cast<std::size_t>(v);
        h *= 1099511628211ull;
      }
      return h;
    }
  };
  static long long quant(double v) noexcept {
    const double q = 1e6;  // 1e-6 spatial quantum (well below any real feature)
    return static_cast<long long>(v >= 0 ? v * q + 0.5 : v * q - 0.5);
  }
  bool endpointKey(const topo::Shape& edge, EndpointKey& out) const {
    const auto cr = topo::curveOf(edge);
    if (!cr || !cr->curve) return false;
    const math::Point3 a =
        detail::placePoint(cr->location, detail::edgeCurveLocal(*cr->curve, cr->first));
    const math::Point3 b =
        detail::placePoint(cr->location, detail::edgeCurveLocal(*cr->curve, cr->last));
    EndpointKey ka{quant(a.x), quant(a.y), quant(a.z), 0, 0, 0};
    EndpointKey kb{quant(b.x), quant(b.y), quant(b.z), 0, 0, 0};
    // Order-independent: put the lexicographically smaller endpoint first.
    const bool aFirst = std::tie(ka.ax, ka.ay, ka.az) <= std::tie(kb.ax, kb.ay, kb.az);
    out = aFirst ? EndpointKey{ka.ax, ka.ay, ka.az, kb.ax, kb.ay, kb.az}
                 : EndpointKey{kb.ax, kb.ay, kb.az, ka.ax, ka.ay, ka.az};
    return true;
  }

  EdgeDiscretization build(const topo::Shape& edge) const {
    EdgeDiscretization d;
    const auto cr = topo::curveOf(edge);
    const double first = cr ? cr->first : 0.0;
    const double last = cr ? cr->last : 1.0;
    int localMin = minSegs_;
    EndpointKey k;
    if (endpointKey(edge, k))
      if (auto it = segsByEndpoints_.find(k); it != segsByEndpoints_.end())
        localMin = std::max(localMin, it->second);
    const int n =
        (cr && cr->curve)
            ? detail::edgeSegments(*cr->curve, first, last, deflection_, localMin, maxSegs_)
            : localMin;
    d.fracs.reserve(static_cast<std::size_t>(n) + 1);
    d.points.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i) {
      const double f = static_cast<double>(i) / n;
      d.fracs.push_back(f);
      if (cr && cr->curve) {
        const double t = first + (last - first) * f;
        d.points.push_back(detail::placePoint(cr->location, detail::edgeCurveLocal(*cr->curve, t)));
      }
    }
    return d;
  }

  std::unordered_map<const topo::TShape*, EdgeDiscretization> cache_;
  std::unordered_map<EndpointKey, int, EndpointKeyHash> segsByEndpoints_;
  double deflection_;
  int minSegs_;
  int maxSegs_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_EDGE_MESHER_H

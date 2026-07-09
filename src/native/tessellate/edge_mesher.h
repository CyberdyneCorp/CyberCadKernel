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
#include <optional>
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

// ── Canonical (build-order-independent) endpoints of an edge, world-placed ─────
// Two topologically DISTINCT edge nodes that occupy the same segment (a per-turn
// thread ridge shared by two ruled bands, built as SEPARATE nodes with opposite
// vertex order) have Line frames that differ in origin/direction, so evaluating a
// shared fraction through each node's own frame — or through each adjacent face's
// own surface — lands on 3D points that agree only to ~1 ULP. When such a point
// falls on a weld-grid cell boundary its two copies round to opposite cells and
// the spatial weld fails to fuse them, leaving a per-turn seam sliver OPEN at some
// deflections (the thread robustlyWatertight residual). Canonicalising the two
// endpoints (lexicographically ordered) makes BOTH coincident edges hand back the
// SAME ordered pair, so a straight-edge sample interpolated between them is
// BIT-IDENTICAL across the two edges → the two faces place the same boundary point
// → the weld fuses them. Curved edges are unaffected (they already share the edge
// node, hence identical points). This is the "one shared 1D discretization pinned
// on both faces" contract, completed at the 3D-point level.
struct CanonicalEndpoints {
  math::Point3 a;         ///< lexicographically-smaller world endpoint
  math::Point3 b;         ///< the other world endpoint
  bool aIsFirst = false;  ///< internal: whether `a` is the edge's `first`-param end
  bool valid = false;
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

// Worst curvature magnitude probed across the edge's range (5-point sample) — the
// basis both for segment sizing and for the "straight in 3D" test. Factored so the
// canonical curved-edge sharing (EdgeCache) can reuse the SAME straight/curved
// decision `edgeSegments` uses, keeping the two in lock-step.
inline double worstEdgeCurvature(const topo::EdgeCurve& c, double first, double last) noexcept {
  double worst = 0.0;
  for (int i = 0; i <= 4; ++i) {
    const double t = first + (last - first) * (i * 0.25);
    worst = std::max(worst, edgeCurvature(c, t, first, last));
  }
  return worst;
}

// True iff the edge is straight in 3D (‖C″‖ ≈ 0 across its range) — the same
// threshold edgeSegments uses to collapse a line to minSegs. A genuinely-curved
// edge (circle, blend, boolean seam) returns false.
inline bool edgeStraight3d(const topo::EdgeCurve& c, double first, double last) noexcept {
  if (std::fabs(last - first) <= 0.0) return true;
  return worstEdgeCurvature(c, first, last) <= 1e-12;
}

// True iff `edge` is a CLOSED-SEAM STRAIGHT CHORD: a degree-1 curve with exactly two
// poles whose 3-D geometry is straight (‖C″‖≈0). This is precisely the shape
// `smooth_trim_split`/`curved_wall_cut` lay as ONE segment of a closed interior seam
// (buildSeamEdge / edgeFromPiece: a 2-pole degree-1 chord between two adjacent traced
// seam nodes). Such a chord's 3-D geometry is the STRAIGHT segment, but its pcurve
// rides the (possibly CURVED) surface of the face it bounds — so a curved sub-face
// (the bowl annulus/disk) that evaluates its seam boundary through S_face(pcurve) lands
// on the bulging surface, NOT on the shared chord the flat cap places its boundary on,
// and the closed seam does not weld watertight. The face mesher uses this predicate to
// PIN such a boundary vertex to the edge's canonical 3-D chord point (C_edge, shared by
// both sub-faces) instead of the per-face surface evaluation — the closed-loop analogue
// of the shared-curved-edge single-sampling fix. A Line-kind edge, a genuinely curved
// edge (Circle/blend/multi-pole spline), or a chord with ≠2 poles returns false, so the
// straight-endpoint and curved-edge paths are untouched and every existing mesh is
// byte-identical.
inline bool isSeamChord(const topo::EdgeCurve& c, double /*first*/, double /*last*/) noexcept {
  if (c.kind != topo::EdgeCurve::Kind::BSpline && c.kind != topo::EdgeCurve::Kind::Bezier)
    return false;
  if (c.poles.size() != 2) return false;  // exactly a 2-node segment of the seam
  if (c.degree != 1) return false;         // faithful straight-chord representation
  // A 2-pole degree-1 curve is straight BY CONSTRUCTION (linear between its two poles):
  // no numeric curvature test — a central-difference ‖C″‖ of an exact line is ~1e-8
  // fp-noise (amplified by 1/h²), which would spuriously reject a genuine seam chord.
  return true;
}

// True iff `edge` is a GENUINELY-CURVED SHARED RIM edge: a degree≥2 FREE-FORM (Bézier /
// B-spline) arc whose 3-D geometry is actually curved (‖C″‖ ≫ 0) — analytic Circle/Ellipse
// seams are EXCLUDED by kind (see below). This is precisely the shape a curved-wall freeform
// operand lays as the OUTER rim shared between the FREEFORM wall and an ADJACENT ANALYTIC
// face (the bowl↔flat-lid rim — a per-segment degree-2 Bézier arc). Such a rim's 3-D curve
// dips off the analytic neighbour's plane, so the neighbour's
// pcurve (which stays IN the plane) does NOT reproduce C_edge once the rim is subdivided —
// the rim opens. The face mesher uses this predicate (alongside isSeamChord, mutually
// exclusive) to PIN the diverging face's rim samples to the edge's canonical 3-D
// discretization (d.points == C_edge, the ONE list both faces share) — the CURVED-edge
// analogue of the straight seam-chord pin.
//
// Returns false for a straight Line edge, a 2-pole degree-1 seam chord (isSeamChord owns it),
// any degree-1 polyline, and any edge straight-in-3-D (‖C″‖≈0). The pin itself is additionally
// divergence-gated (only samples ≫ kSnapEps off C_edge are pinned), so a CURVED shared edge
// that already satisfies S_face(pcurve)=C_edge on BOTH faces — every analytic primitive's
// cap↔side circle — records no pin and stays byte-identical; this predicate only makes such
// an edge ELIGIBLE, the divergence test decides.
inline bool isCurvedSharedRim(const topo::EdgeCurve& c, double first, double last) noexcept {
  using K = topo::EdgeCurve::Kind;
  // A curved-wall freeform operand's rim is a FREE-FORM (Bézier/B-spline) arc of degree ≥ 2
  // (the bowl edge over a straight UV chord is exactly degree-2). It is NEVER an analytic
  // Circle/Ellipse: those are the shared seams of ANALYTIC primitives (a cylinder cap↔side,
  // a sphere/cone latitude, a torus rim, a revolve's latitude circles), whose two incident
  // faces are BOTH analytic and BOTH reproduce C_edge through their own pcurves — the weld
  // contract already holds, so those must NOT be eligible for the rim pin. Restricting to
  // degree ≥ 2 free-form arcs excludes every analytic seam by KIND (the strong topology
  // guard), leaving only the freeform-boolean rim eligible; the divergence gate then fires
  // the pin solely on the analytic neighbour whose planar pcurve fails to track the arc.
  if (c.kind != K::Bezier && c.kind != K::BSpline) return false;  // analytic circle/ellipse/line excluded
  if (c.degree <= 1) return false;                                 // degree-1 polyline / seam chord
  if (c.poles.size() < 3) return false;                            // needs ≥3 poles to curve
  return !edgeStraight3d(c, first, last);                          // confirmed genuinely curved in 3-D
}

// Number of segments to hold the chord (sagitta) error of the edge's 3D curve
// under `deflection`: Δ ≤ √(8·defl/‖C″‖); n = ceil(span/Δ), clamped to [min,max].
// A line (‖C″‖≈0) collapses to `minSegs`; a circle/blend subdivides.
inline int edgeSegments(const topo::EdgeCurve& c, double first, double last, double deflection,
                        int minSegs, int maxSegs) noexcept {
  const double span = std::fabs(last - first);
  if (span <= 0.0) return minSegs;
  // Probe curvature at a few points across the range and take the worst.
  const double worst = worstEdgeCurvature(c, first, last);
  if (worst <= 1e-12) return minSegs;  // straight in 3D
  const double step = std::sqrt(8.0 * deflection / worst);
  if (!(step > 0.0)) return maxSegs;
  const int n = static_cast<int>(std::ceil(span / step));
  return std::clamp(n, minSegs, maxSegs);
}

// Order two world endpoints lexicographically (x, then y, then z). Deterministic
// and independent of which node/orientation the endpoints came from, so two
// coincident edges canonicalise to the SAME pair.
inline bool endpointLess(const math::Point3& p, const math::Point3& q) noexcept {
  if (p.x != q.x) return p.x < q.x;
  if (p.y != q.y) return p.y < q.y;
  return p.z < q.z;
}

// World-placed canonical endpoints of a STRAIGHT edge (Line curve). Returns
// `valid == false` for a non-line edge (curved seams already share their edge
// node and thus identical points; only the thread's per-band straight seams need
// canonicalising). The face mesher interpolates seam samples between `a` and `b`
// in this fixed order (see recordEdgeAnchors), so two coincident edges produce
// bit-identical points.
//
// The endpoints are read from the edge's BOUNDING VERTICES (which two coincident
// edges SHARE as the same TShape node → bit-identical points), NOT reconstructed
// from the Line frame: `origin + dir·len` differs by ~1 ULP between two edges
// built with opposite vertex order (different origin/direction), which would
// defeat the canonicalisation. The first stored vertex is the `first`-parameter
// end (ShapeBuilder::makeEdge stores v0 Forward at `first`, v1 Reversed at
// `last`); we take the first two vertices in stored order.
inline CanonicalEndpoints canonicalLineEndpoints(const topo::Shape& edge) noexcept {
  CanonicalEndpoints ce;
  const auto cr = topo::curveOf(edge);
  if (!cr || !cr->curve || cr->curve->kind != topo::EdgeCurve::Kind::Line) return ce;
  std::optional<math::Point3> pFirst, pLast;
  for (topo::Explorer ex(edge, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const std::optional<math::Point3> p = topo::pointOf(ex.current());
    if (!p) continue;
    if (!pFirst) pFirst = p;
    else if (!pLast) { pLast = p; break; }
  }
  if (!pFirst || !pLast) return ce;  // degenerate edge → leave unshared
  ce.aIsFirst = !endpointLess(*pLast, *pFirst);  // a = min endpoint; ties keep first
  ce.a = ce.aIsFirst ? *pFirst : *pLast;
  ce.b = ce.aIsFirst ? *pLast : *pFirst;
  ce.valid = true;
  return ce;
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

  // ── FREEFORM-BACKED curved-rim registry (curved-rim weld pin gate) ────────────
  // Mark the CURVED shared rim edge with the given endpoints as FREEFORM-BACKED: a
  // FREE-FORM (Bézier / B-spline) face has been found whose surface evaluation of this
  // edge's boundary REPRODUCES the edge's canonical 3-D discretization d.points (the weld
  // contract S_freeform(pcurve) = C_edge holds on the free-form side). This is the exact
  // condition under which pinning the DIVERGING flat (Plane) neighbour's rim samples to
  // d.points is SAFE — the free-form face genuinely lies on d.points, so pinning the lid to
  // it fuses the two boundaries onto the ONE shared curve (the bowl↔lid rim). Without this
  // guarantee (a synthesized boolean cut-cap curved seam whose free-form neighbour does NOT
  // reproduce d.points — the first-freeform / split-plane / chain-seam family) d.points is a
  // WRONG pin target and pinning the Plane would open the seam; that edge is NEVER marked, so
  // the Plane pin never fires there. Keyed by the order-independent, quantized endpoint pair
  // PLUS a quantized 3-D midpoint, so two DIFFERENT arcs between the same endpoints never
  // collide. Populated by the SolidMesher pre-pass before any face is meshed.
  void markFreeformBackedRim(const topo::Shape& edge) {
    EndpointKey k;
    math::Point3 mid;
    if (!rimKey(edge, k, mid)) return;
    freeformBackedRims_[k].push_back(mid);
  }
  bool isFreeformBackedRim(const topo::Shape& edge) const {
    EndpointKey k;
    math::Point3 mid;
    if (!rimKey(edge, k, mid)) return false;
    const auto it = freeformBackedRims_.find(k);
    if (it == freeformBackedRims_.end()) return false;
    for (const math::Point3& m : it->second)
      if (math::distance(mid, m) <= kCurveMidTol) return true;  // same arc → backed
    return false;
  }

  // Shared discretization for `edge` (keyed by TShape identity). Curved edges get
  // enough segments to meet the deflection bound; straight edges get minSegs (or
  // the requireMinSegs override, when an adjacent face demanded more).
  //
  // Canonical curved-edge SINGLE-SAMPLING (MOAT M0 weld robustness). A genuinely
  // CURVED edge reached here via a SEPARATE topological node (not the TShape fast
  // path) — e.g. a freeform boolean seam that the cap and the trimmed freeform
  // sub-face each carry as their OWN edge node over the SAME 3-D curve — adopts ONE
  // canonical EdgeDiscretization keyed by its (order-independent, quantized)
  // endpoint pair PLUS a quantized 3-D midpoint discriminator. Both incident faces
  // then read BIT-IDENTICAL sample points, so the per-face boundary anchors coincide
  // and the seam welds watertight at ANY deflection (not only where two independent
  // samplings coincidentally align). The TShape fast path and the straight-edge
  // endpoint sharing are untouched: an edge shared through ONE node (every primitive)
  // and every straight edge keep their exact behaviour, so existing meshes are
  // byte-identical (the midpoint discriminator also prevents two DIFFERENT arcs
  // between the same endpoints from ever collapsing to one key).
  const EdgeDiscretization& discretize(const topo::Shape& edge) {
    const topo::TShape* key = edge.tshape().get();
    if (auto it = cache_.find(key); it != cache_.end()) return it->second;
    EdgeDiscretization d = build(edge);
    EndpointKey ek;
    math::Point3 mid;
    if (curvedSeparateNode(edge, d, ek, mid)) {
      std::vector<CanonicalCurve>& bucket = curvedByEndpoints_[ek];
      for (const CanonicalCurve& cc : bucket)
        if (math::distance(mid, cc.mid) <= kCurveMidTol)      // same arc → adopt canonical
          return cache_.emplace(key, cc.disc).first->second;
      bucket.push_back(CanonicalCurve{mid, d});               // first builder wins → canonical
    }
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

  // ── Canonical curved-edge record: the shared discretization + the 3-D midpoint
  // that disambiguates two DIFFERENT arcs sharing the same endpoints ────────────
  // Bucketed by the (order-independent, quantized) endpoint key, then matched by the
  // midpoint at parameter 0.5 using a real-distance tolerance — NOT a quantized
  // midpoint equality. A quantized midpoint has the same cell-boundary fragility the
  // weld does: a midpoint that lands on a quantum boundary (e.g. 0.0453125 at a 1e-6
  // quantum) would split the two representations of the SAME arc into different keys
  // and defeat the sharing. The distance test is boundary-free: two representations
  // of one arc agree to ~1 ULP (≪ tol) and merge; two genuinely different arcs
  // between the same endpoints differ macroscopically at the midpoint (≫ tol) and
  // never merge. The midpoint is taken at the exact mid-parameter (not a sampled
  // index) so it is independent of which direction/node the edge is traversed in.
  struct CanonicalCurve {
    math::Point3 mid;
    EdgeDiscretization disc;
  };
  static constexpr double kCurveMidTol = 1e-6;  ///< same-arc midpoint agreement (≫ ULP, ≪ any real arc gap)

  // Fill `ekOut`/`midOut` and return true iff `edge` is a GENUINELY-CURVED edge that
  // reached discretize() via a separate node (the TShape fast path already missed).
  // A Line curve, or any curve straight in 3-D (‖C″‖≈0 — a collinear degree-1
  // polyline), returns false and keeps the unchanged straight/endpoint-shared path.
  bool curvedSeparateNode(const topo::Shape& edge, const EdgeDiscretization& d,
                          EndpointKey& ekOut, math::Point3& midOut) const {
    if (d.points.size() < 2) return false;
    const auto cr = topo::curveOf(edge);
    if (!cr || !cr->curve) return false;
    if (cr->curve->kind == topo::EdgeCurve::Kind::Line) return false;
    if (detail::edgeStraight3d(*cr->curve, cr->first, cr->last)) return false;
    if (!endpointKey(edge, ekOut)) return false;
    midOut = detail::placePoint(cr->location,
                                detail::edgeCurveLocal(*cr->curve, 0.5 * (cr->first + cr->last)));
    return true;
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

  // Endpoint key + 3-D midpoint for the freeform-backed-rim registry (order-independent,
  // arc-disambiguated). True only for a genuinely-curved edge (matches the pin's eligibility).
  bool rimKey(const topo::Shape& edge, EndpointKey& ekOut, math::Point3& midOut) const {
    const auto cr = topo::curveOf(edge);
    if (!cr || !cr->curve) return false;
    if (detail::edgeStraight3d(*cr->curve, cr->first, cr->last)) return false;
    if (!endpointKey(edge, ekOut)) return false;
    midOut = detail::placePoint(cr->location,
                                detail::edgeCurveLocal(*cr->curve, 0.5 * (cr->first + cr->last)));
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
  std::unordered_map<EndpointKey, std::vector<CanonicalCurve>, EndpointKeyHash> curvedByEndpoints_;
  std::unordered_map<EndpointKey, std::vector<math::Point3>, EndpointKeyHash> freeformBackedRims_;
  double deflection_;
  int minSegs_;
  int maxSegs_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_EDGE_MESHER_H

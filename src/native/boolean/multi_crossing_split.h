// SPDX-License-Identifier: Apache-2.0
//
// multi_crossing_split.h — Layer-3 Stage-3 (L3-b): MULTI-CROSSING / RE-ENTRANT /
// HOLE-CROSSING face split. The general planar-arrangement generalisation of the two
// byte-frozen split slices (`face_split.h` convex-1-chord, `smooth_trim_split.h`
// closed-interior-seam) into N ≥ 2 sub-regions.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// `face_split.h::splitFace` partitions a CONVEX outer loop cut by ONE clean chord
// (crossings == 2) into TWO sub-faces. `smooth_trim_split.h::splitFaceSmoothTrim`
// partitions a face along ONE CLOSED interior seam (crossings == 0) into disk +
// annulus. Everything BETWEEN and BEYOND them declines today: a face cut by MULTIPLE
// chords (multi-crossing → > 2 sub-regions), a RE-ENTRANT cut (a chord that crosses
// the boundary more than twice, or two chords that cross each other inside), and a
// seam that CROSSES an existing trim HOLE. This verb resolves that family.
//
// `multiCrossingSplit(parent, seams)` takes a trimmed face — its outer loop + hole
// loops as UV polygons — and ONE OR MORE seam curves (open chords or closed loops as
// UV polylines) and returns the N ≥ 2 sub-regions the seams cut the face into, each a
// simple oriented loop, self-verified to TILE the parent (Σ area == parent area).
//
// ── METHOD (planar arrangement + face-trace, same family as trim_boolean's arc-walk) ─
//   1. Collect boundary arcs (outer + holes) and seam arcs as UV polyline segments.
//      Clip every seam to the parent region (drop the part outside the outer loop or
//      inside a hole — a re-entrant seam re-enters through its crossings).
//   2. Find ALL crossings: seam×outer, seam×hole, seam×seam (the H1 curve-curve
//      intersector's polyline specialisation — the SAME segCross closed-form the
//      Wave-I trim_boolean uses). Split every arc at its crossings.
//   3. Build the planar arrangement as a doubly-connected edge list: every undirected
//      segment becomes TWO directed half-edges; at each vertex sort the outgoing
//      half-edges by angle; walk faces by "next = tightest clockwise turn". This is
//      the standard planar-subdivision face traversal — the same orientation-coherent
//      arc-walk family as trim_boolean.cpp (outer CCW, holes CW).
//   4. Keep the BOUNDED faces that lie inside the parent region (inside the outer loop,
//      outside every hole); discard the unbounded face and any face outside the parent.
//   5. SELF-VERIFY (host-checkable, no OCCT): Σ area(sub-regions) == area(parent) within
//      a scale-relative tolerance (≤ 1e-10), and each sub-region is a SIMPLE loop. ANY
//      failure → an honest MULTI-decline (no partition), never a leaky / wrong tiling.
//
// ── HONESTY CONTRACT (hard invariant, mirrors the rest of the kernel) ───────────────
// A DEGENERATE seam — tangent to the boundary (a touch with no transversal crossing),
// coincident with a boundary/hole edge, or a seam that does not actually subdivide the
// face — is AMBIGUOUS and is HONEST-DECLINED (a first-class outcome carrying WHY), NEVER
// resolved into a fabricated tiling. No tolerance is EVER widened to force a crossing.
//
// OCCT-FREE (0 OCCT includes). Header-only. Reuses ONLY native/topology (ParamPoint,
// TrimRegion, flattenTrimLoop). clang++ -std=c++20, fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_MULTI_CROSSING_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_MULTI_CROSSING_SPLIT_H

#include "native/topology/trim_boolean.h"
#include "native/topology/trimmed_nurbs.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
using topo::ParamPoint;
using topo::TrimLoop;
using topo::TrimRegion;

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes. `Ok` iff a verified N-region tiling is returned; every other value
// is an HONEST decline (a first-class outcome) carrying WHY the seams are beyond a
// clean tiling. The caller falls through (to face_split / smooth_trim_split / OCCT).
// ─────────────────────────────────────────────────────────────────────────────
enum class MultiSplitDecline : std::uint8_t {
  Ok,
  NoParent,          ///< the parent has no usable outer loop (< 3 UV points)
  NoSeam,            ///< no seam supplied / every seam degenerate
  NoSubdivision,     ///< the seams do not subdivide the face (single region ⇒ no cut)
  Degenerate,        ///< a tangent / coincident / self-touching seam — ambiguous, declined
  DegenerateSubRegion,  ///< a sub-region has near-zero area or is not a simple loop
  TilingGap          ///< Σ area(sub-regions) != area(parent) beyond tolerance
};

inline const char* multiSplitDeclineName(MultiSplitDecline d) noexcept {
  switch (d) {
    case MultiSplitDecline::Ok: return "Ok";
    case MultiSplitDecline::NoParent: return "NoParent";
    case MultiSplitDecline::NoSeam: return "NoSeam";
    case MultiSplitDecline::NoSubdivision: return "NoSubdivision";
    case MultiSplitDecline::Degenerate: return "Degenerate";
    case MultiSplitDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case MultiSplitDecline::TilingGap: return "TilingGap";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// One sub-region of the tiling: a closed UV polyline (outer boundary of the piece)
// plus any hole loops it inherits from the parent, its signed area (CCW > 0), and its
// hole loops (parent holes wholly contained in this piece). The airtight oracles read
// `outer` + `signedArea` directly (exact vertices + shoelace area).
// ─────────────────────────────────────────────────────────────────────────────
struct SubRegion {
  std::vector<ParamPoint> outer;            ///< CCW outer boundary of the sub-region
  std::vector<std::vector<ParamPoint>> holes;  ///< parent holes fully inside this piece (CW)
  double signedArea = 0.0;                  ///< net area (outer − holes), > 0
};

struct MultiSplitResult {
  std::optional<std::vector<SubRegion>> regions;  ///< nullopt on decline
  MultiSplitDecline decline = MultiSplitDecline::Ok;
  double parentArea = 0.0;   ///< parent net UV area (scale reference)
  double tiledArea = 0.0;    ///< Σ sub-region net area (blocker witness)
  double tilingGap = 0.0;    ///< |parent − tiled| (blocker witness)
  int subRegions = 0;        ///< number of sub-regions produced
  bool ok() const noexcept { return regions.has_value() && decline == MultiSplitDecline::Ok; }
};

struct MultiSplitOptions {
  int flattenSegments = 256;    ///< polyline samples per pcurve segment (curved seams/boundary)
  double areaFloorFrac = 1e-9;  ///< min |sub-region area| as a fraction of the parent
  double tilingTolFrac = 1e-10; ///< Σ-area identity relative tolerance (NEVER weakened to pass)
};

// ─────────────────────────────────────────────────────────────────────────────
// The parent face + its cutting seams, expressed directly as UV polylines — the most
// airtight form for the closed-form oracles (exact vertices). A convenience overload
// below accepts a topology `TrimRegion` + `TrimLoop` seams and flattens them.
// ─────────────────────────────────────────────────────────────────────────────
struct MultiSplitInput {
  std::vector<ParamPoint> outer;               ///< parent outer loop (CCW or CW; auto-oriented)
  std::vector<std::vector<ParamPoint>> holes;  ///< parent hole loops
  std::vector<std::vector<ParamPoint>> seams;  ///< cutting seams (open chords or closed loops)
};

namespace mcsdetail {

// ── UV primitives (mirror trim_boolean.cpp's, kept local so this stays header-only) ──
inline double signedArea(const std::vector<ParamPoint>& p) noexcept {
  const std::size_t n = p.size();
  if (n < 3) return 0.0;
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += p[j].u * p[i].v - p[i].u * p[j].v;
  return 0.5 * a;
}

inline double dist(const ParamPoint& a, const ParamPoint& b) noexcept {
  const double du = a.u - b.u, dv = a.v - b.v;
  return std::sqrt(du * du + dv * dv);
}

// Even-odd ray-cast (half-open PNPOLY — the SAME rule trimmed_nurbs::raycast /
// trim_boolean use). True ⇔ p strictly inside the closed polygon `poly`.
inline bool pointInPolygon(const std::vector<ParamPoint>& poly, const ParamPoint& p) noexcept {
  const std::size_t n = poly.size();
  if (n < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const ParamPoint& a = poly[i];
    const ParamPoint& b = poly[j];
    if ((a.v > p.v) != (b.v > p.v)) {
      const double x = (b.u - a.u) * (p.v - a.v) / (b.v - a.v) + a.u;
      if (p.u < x) inside = !inside;
    }
  }
  return inside;
}

// A point strictly inside the parent region (inside outer, outside every hole).
inline bool inParent(const std::vector<ParamPoint>& outer,
                     const std::vector<std::vector<ParamPoint>>& holes,
                     const ParamPoint& p) noexcept {
  if (!pointInPolygon(outer, p)) return false;
  for (const auto& h : holes)
    if (pointInPolygon(h, p)) return false;
  return true;
}

inline double bboxDiag(const std::vector<ParamPoint>& outer) noexcept {
  if (outer.empty()) return 1.0;
  double uMin = outer[0].u, uMax = uMin, vMin = outer[0].v, vMax = vMin;
  for (const ParamPoint& q : outer) {
    uMin = std::min(uMin, q.u); uMax = std::max(uMax, q.u);
    vMin = std::min(vMin, q.v); vMax = std::max(vMax, q.v);
  }
  const double du = uMax - uMin, dv = vMax - vMin;
  const double e = std::sqrt(du * du + dv * dv);
  return e > 0.0 ? e : 1.0;
}

// Segment–segment transversal crossing (closed form). Returns the interior crossing
// point and along-segment fractions sA (on a) / sB (on b). Sets `coincident` on a real
// collinear overlap (the honest-decline seam). Byte-equivalent to trim_boolean's
// segCross, kept local so this header pulls in no .cpp internals.
struct Xing {
  ParamPoint pt{};
  double sA = 0.0;
  double sB = 0.0;
};

inline std::optional<Xing> segCross(const ParamPoint& a1, const ParamPoint& a2,
                                    const ParamPoint& b1, const ParamPoint& b2, double tol,
                                    bool& coincident) noexcept {
  const double rX = a2.u - a1.u, rY = a2.v - a1.v;
  const double sX = b2.u - b1.u, sY = b2.v - b1.v;
  const double denom = rX * sY - rY * sX;
  const double qpX = b1.u - a1.u, qpY = b1.v - a1.v;
  if (std::fabs(denom) <= tol * tol) {
    const double cross_qp_r = qpX * rY - qpY * rX;
    const double rr = rX * rX + rY * rY;
    if (std::fabs(cross_qp_r) <= tol && rr > 0.0) {
      const double t0 = (qpX * rX + qpY * rY) / rr;
      const double t1 = t0 + (sX * rX + sY * rY) / rr;
      const double lo = std::min(t0, t1), hi = std::max(t0, t1);
      if (hi > tol && lo < 1.0 - tol) coincident = true;
    }
    return std::nullopt;
  }
  const double t = (qpX * sY - qpY * sX) / denom;
  const double u = (qpX * rY - qpY * rX) / denom;
  if (t < -tol || t > 1.0 + tol || u < -tol || u > 1.0 + tol) return std::nullopt;
  Xing c;
  c.sA = std::clamp(t, 0.0, 1.0);
  c.sB = std::clamp(u, 0.0, 1.0);
  c.pt = {a1.u + c.sA * rX, a1.v + c.sA * rY};
  return c;
}

// ── Planar arrangement (doubly-connected edge list, welded vertices). ──
// One directed half-edge: from vertex `from` to vertex `to`; `twin` is the opposite
// half-edge (same undirected segment); `next` is the next half-edge in the face walk.
struct HalfEdge {
  int from = -1;
  int to = -1;
  int twin = -1;
  int next = -1;
  bool visited = false;
};

// The arrangement: welded vertices + directed half-edges (2 per undirected segment).
struct Arrangement {
  std::vector<ParamPoint> verts;
  std::vector<HalfEdge> he;

  // Weld a vertex (return its index), snapping to an existing vertex within `weldTol`.
  int addVertex(const ParamPoint& p, double weldTol) {
    for (std::size_t i = 0; i < verts.size(); ++i)
      if (dist(verts[i], p) <= weldTol) return static_cast<int>(i);
    verts.push_back(p);
    return static_cast<int>(verts.size() - 1);
  }

  // Add an undirected segment between welded vertices a and b as two twinned half-edges.
  void addSegment(int a, int b) {
    if (a == b) return;
    const int e0 = static_cast<int>(he.size());
    he.push_back(HalfEdge{a, b, e0 + 1, -1, false});
    he.push_back(HalfEdge{b, a, e0, -1, false});
  }
};

// A closed polyline `poly` (implicitly closed) → welded arrangement segments.
inline void addClosedPolyline(Arrangement& arr, const std::vector<ParamPoint>& poly,
                              double weldTol) {
  const std::size_t n = poly.size();
  if (n < 2) return;
  int first = arr.addVertex(poly[0], weldTol);
  int prev = first;
  for (std::size_t i = 1; i < n; ++i) {
    int cur = arr.addVertex(poly[i], weldTol);
    arr.addSegment(prev, cur);
    prev = cur;
  }
  arr.addSegment(prev, first);  // close
}

// An OPEN polyline (chord) `poly` split at its crossing points → welded segments.
inline void addOpenPolyline(Arrangement& arr, const std::vector<ParamPoint>& poly,
                            double weldTol) {
  const std::size_t n = poly.size();
  if (n < 2) return;
  int prev = arr.addVertex(poly[0], weldTol);
  for (std::size_t i = 1; i < n; ++i) {
    int cur = arr.addVertex(poly[i], weldTol);
    arr.addSegment(prev, cur);
    prev = cur;
  }
}

// Angle of half-edge e (direction from→to), for the rotational vertex order.
inline double heAngle(const Arrangement& arr, int e) noexcept {
  const HalfEdge& h = arr.he[static_cast<std::size_t>(e)];
  const ParamPoint& a = arr.verts[static_cast<std::size_t>(h.from)];
  const ParamPoint& b = arr.verts[static_cast<std::size_t>(h.to)];
  return std::atan2(b.v - a.v, b.u - a.u);
}

// Wire up `next` for every half-edge: from a half-edge arriving at vertex v, the next
// half-edge in a minimal (clockwise) face walk is the outgoing half-edge whose
// direction is the FIRST one clockwise from the reverse of the arriving edge. Standard
// planar-subdivision face traversal (rotational system).
inline void linkNext(Arrangement& arr) {
  const int nV = static_cast<int>(arr.verts.size());
  // Outgoing half-edges per vertex, sorted CCW by angle.
  std::vector<std::vector<int>> out(static_cast<std::size_t>(nV));
  for (int e = 0; e < static_cast<int>(arr.he.size()); ++e)
    out[static_cast<std::size_t>(arr.he[static_cast<std::size_t>(e)].from)].push_back(e);
  for (auto& lst : out)
    std::sort(lst.begin(), lst.end(),
              [&](int x, int y) { return heAngle(arr, x) < heAngle(arr, y); });
  for (int e = 0; e < static_cast<int>(arr.he.size()); ++e) {
    const HalfEdge& h = arr.he[static_cast<std::size_t>(e)];
    const int v = h.to;
    const int twin = h.twin;  // reverse edge (leaves v back toward `from`)
    const auto& lst = out[static_cast<std::size_t>(v)];
    // Find twin's position in v's CCW order; the next face half-edge is the one
    // immediately BEFORE it (i.e. the tightest clockwise turn).
    int idx = 0;
    for (std::size_t i = 0; i < lst.size(); ++i)
      if (lst[i] == twin) { idx = static_cast<int>(i); break; }
    const int m = static_cast<int>(lst.size());
    const int prevIdx = (idx - 1 + m) % m;
    arr.he[static_cast<std::size_t>(e)].next = lst[static_cast<std::size_t>(prevIdx)];
  }
}

// Trace one face starting at half-edge `start` → its closed vertex polyline.
inline std::vector<ParamPoint> traceFace(Arrangement& arr, int start, int guardMax) {
  std::vector<ParamPoint> poly;
  int e = start, guard = 0;
  do {
    arr.he[static_cast<std::size_t>(e)].visited = true;
    poly.push_back(arr.verts[static_cast<std::size_t>(arr.he[static_cast<std::size_t>(e)].from)]);
    e = arr.he[static_cast<std::size_t>(e)].next;
  } while (e != start && e >= 0 && ++guard < guardMax);
  return poly;
}

// A point strictly INSIDE a simple CCW polygon (robust for non-convex cells, where the
// centroid can fall outside). Walk an edge midpoint a hair toward the interior (to the
// LEFT of the directed edge, since the polygon is CCW), scaled by the local edge length;
// verify the probe is genuinely inside, else fall back along successive edges. This is
// the standard "representative interior point" of an arrangement cell.
inline ParamPoint interiorPoint(const std::vector<ParamPoint>& poly) noexcept {
  const int n = static_cast<int>(poly.size());
  if (n < 3) return poly.empty() ? ParamPoint{} : poly[0];
  for (int i = 0; i < n; ++i) {
    const ParamPoint& a = poly[static_cast<std::size_t>(i)];
    const ParamPoint& b = poly[static_cast<std::size_t>((i + 1) % n)];
    const double mx = 0.5 * (a.u + b.u), my = 0.5 * (a.v + b.v);
    const double dx = b.u - a.u, dy = b.v - a.v;
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len <= 0.0) continue;
    // Inward normal of a CCW polygon = rotate the edge direction +90° (−dy, dx).
    const double nx = -dy / len, ny = dx / len;
    for (double step = len * 1e-3; step <= len; step *= 4.0) {
      const ParamPoint p{mx + nx * step, my + ny * step};
      if (pointInPolygon(poly, p)) return p;
    }
  }
  // Fallback: the centroid (only reached for a degenerate cell already floored out).
  ParamPoint c{0.0, 0.0};
  for (const ParamPoint& q : poly) { c.u += q.u; c.v += q.v; }
  c.u /= static_cast<double>(n);
  c.v /= static_cast<double>(n);
  return c;
}

// True ⇔ the closed polygon `poly` is a SIMPLE loop (no non-adjacent edge crossing).
inline bool simpleLoop(const std::vector<ParamPoint>& poly) noexcept {
  const int n = static_cast<int>(poly.size());
  if (n < 3) return false;
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const int i1 = (i + 1) % n, j1 = (j + 1) % n;
      if (i == j1 || j == i1) continue;  // adjacent edges legitimately share a vertex
      bool coin = false;
      if (segCross(poly[static_cast<std::size_t>(i)], poly[static_cast<std::size_t>(i1)],
                   poly[static_cast<std::size_t>(j)], poly[static_cast<std::size_t>(j1)], 0.0,
                   coin))
        return false;
    }
  return true;
}

// Split every arrangement-input polyline at all pairwise crossings among a segment
// soup, returning the crossing points to insert. We build the arrangement in TWO
// passes: pass 1 collects the raw polylines; here we compute the split points per
// polyline and re-emit each polyline with its crossings inserted in order.
struct RawPoly {
  std::vector<ParamPoint> pts;
  bool closed = false;
};

// Insert into every polyline the crossing points it shares with every other polyline
// (and with itself for a self-crossing seam), so the arrangement segments meet only at
// shared welded vertices. Sets `coincident` on a real collinear overlap.
inline std::vector<RawPoly> splitAtCrossings(const std::vector<RawPoly>& in, double tol,
                                             bool& coincident) {
  // Expand each polyline to its directed segment list with a back-reference.
  struct Seg { int poly; int idx; ParamPoint a, b; };
  std::vector<Seg> segs;
  for (int pi = 0; pi < static_cast<int>(in.size()); ++pi) {
    const RawPoly& rp = in[static_cast<std::size_t>(pi)];
    const std::size_t n = rp.pts.size();
    const std::size_t lim = rp.closed ? n : (n >= 1 ? n - 1 : 0);
    for (std::size_t i = 0; i < lim; ++i)
      segs.push_back(Seg{pi, static_cast<int>(i), rp.pts[i], rp.pts[(i + 1) % n]});
  }
  // Per (poly,segment) list of split fractions.
  std::vector<std::vector<std::vector<double>>> frac(in.size());
  for (std::size_t pi = 0; pi < in.size(); ++pi) {
    const RawPoly& rp = in[pi];
    const std::size_t n = rp.pts.size();
    const std::size_t lim = rp.closed ? n : (n >= 1 ? n - 1 : 0);
    frac[pi].assign(lim, {});
  }
  for (std::size_t x = 0; x < segs.size(); ++x)
    for (std::size_t y = x + 1; y < segs.size(); ++y) {
      const Seg& A = segs[x];
      const Seg& B = segs[y];
      // Adjacent segments of the SAME polyline share an endpoint — skip (not a crossing).
      if (A.poly == B.poly) {
        const RawPoly& rp = in[static_cast<std::size_t>(A.poly)];
        const int n = static_cast<int>(rp.pts.size());
        const int d = std::abs(A.idx - B.idx);
        if (d <= 1 || (rp.closed && d == n - 1)) continue;
      }
      bool coin = false;
      auto c = segCross(A.a, A.b, B.a, B.b, tol, coin);
      if (coin) coincident = true;
      if (!c) continue;
      frac[static_cast<std::size_t>(A.poly)][static_cast<std::size_t>(A.idx)].push_back(c->sA);
      frac[static_cast<std::size_t>(B.poly)][static_cast<std::size_t>(B.idx)].push_back(c->sB);
    }
  // Re-emit each polyline with crossing points inserted in parametric order per segment.
  std::vector<RawPoly> out(in.size());
  for (std::size_t pi = 0; pi < in.size(); ++pi) {
    const RawPoly& rp = in[pi];
    out[pi].closed = rp.closed;
    const std::size_t n = rp.pts.size();
    const std::size_t lim = rp.closed ? n : (n >= 1 ? n - 1 : 0);
    for (std::size_t i = 0; i < lim; ++i) {
      out[pi].pts.push_back(rp.pts[i]);
      auto& fs = frac[pi][i];
      std::sort(fs.begin(), fs.end());
      const ParamPoint& a = rp.pts[i];
      const ParamPoint& b = rp.pts[(i + 1) % n];
      for (double f : fs) {
        if (f <= 1e-12 || f >= 1.0 - 1e-12) continue;  // endpoint hit ⇒ already a vertex
        out[pi].pts.push_back(ParamPoint{a.u + f * (b.u - a.u), a.v + f * (b.v - a.v)});
      }
    }
    if (!rp.closed) out[pi].pts.push_back(rp.pts[n - 1]);
  }
  return out;
}

}  // namespace mcsdetail

// ─────────────────────────────────────────────────────────────────────────────
// multiCrossingSplit — the L3-b entry point. Splits the parent trimmed face along ALL
// seams into N ≥ 2 sub-regions. Returns a verified tiling, or an honest decline with
// the measured blocker. The parent + seams are supplied as UV polylines (exact for the
// closed-form oracles). Boundary orientation is auto-normalised (outer CCW, holes CW).
// ─────────────────────────────────────────────────────────────────────────────
inline MultiSplitResult multiCrossingSplit(const MultiSplitInput& in,
                                           const MultiSplitOptions& opts = {}) {
  using namespace mcsdetail;
  MultiSplitResult r;

  // ── (1) validate + normalise parent orientation (outer CCW, holes CW) ───────
  if (in.outer.size() < 3) {
    r.decline = MultiSplitDecline::NoParent;
    return r;
  }
  std::vector<ParamPoint> outer = in.outer;
  if (signedArea(outer) < 0.0) std::reverse(outer.begin(), outer.end());
  std::vector<std::vector<ParamPoint>> holes;
  for (const auto& h : in.holes) {
    if (h.size() < 3) continue;
    std::vector<ParamPoint> hp = h;
    if (signedArea(hp) > 0.0) std::reverse(hp.begin(), hp.end());  // holes CW
    holes.push_back(std::move(hp));
  }
  const double parentArea = std::fabs(signedArea(outer)) - [&] {
    double s = 0.0;
    for (const auto& h : holes) s += std::fabs(signedArea(h));
    return s;
  }();
  r.parentArea = parentArea;
  const double diag = bboxDiag(outer);
  const double weldTol = diag * 1e-9;
  const double crossTol = diag * 1e-10;

  // Gather non-degenerate seams (≥ 2 distinct points).
  std::vector<std::vector<ParamPoint>> seams;
  for (const auto& s : in.seams) {
    std::vector<ParamPoint> sp;
    for (const ParamPoint& q : s)
      if (sp.empty() || dist(sp.back(), q) > weldTol) sp.push_back(q);
    if (sp.size() >= 2) seams.push_back(std::move(sp));
  }
  if (seams.empty()) {
    r.decline = MultiSplitDecline::NoSeam;
    return r;
  }

  // ── (2) build the raw polyline soup: boundary (closed) + seams (closed if the
  // endpoints coincide, else open chords), split at ALL pairwise crossings ─────
  std::vector<RawPoly> raw;
  raw.push_back(RawPoly{outer, true});
  for (const auto& h : holes) raw.push_back(RawPoly{h, true});
  for (auto& s : seams) {
    const bool closed = dist(s.front(), s.back()) <= weldTol;
    if (closed && s.size() >= 3) {
      s.pop_back();  // drop the closing duplicate
      raw.push_back(RawPoly{s, true});
    } else {
      raw.push_back(RawPoly{s, false});
    }
  }
  bool coincident = false;
  std::vector<RawPoly> split = splitAtCrossings(raw, crossTol, coincident);
  if (coincident) {  // coincident boundary/seam edge ⇒ ambiguous, honest-decline
    r.decline = MultiSplitDecline::Degenerate;
    return r;
  }

  // ── (3) weld the split polylines into the planar arrangement ────────────────
  Arrangement arr;
  std::size_t idx = 0;
  addClosedPolyline(arr, split[idx++].pts, weldTol);              // outer
  for (std::size_t k = 0; k < holes.size(); ++k)                  // holes
    addClosedPolyline(arr, split[idx++].pts, weldTol);
  for (; idx < split.size(); ++idx) {                             // seams
    if (split[idx].closed) addClosedPolyline(arr, split[idx].pts, weldTol);
    else addOpenPolyline(arr, split[idx].pts, weldTol);
  }
  if (arr.he.empty()) {
    r.decline = MultiSplitDecline::NoSeam;
    return r;
  }

  // ── (4) trace every bounded arrangement face; keep the parent-interior ones ─
  linkNext(arr);
  const int guardMax = static_cast<int>(arr.he.size()) * 2 + 16;
  std::vector<std::vector<ParamPoint>> faces;
  for (int e = 0; e < static_cast<int>(arr.he.size()); ++e) {
    if (arr.he[static_cast<std::size_t>(e)].visited) continue;
    std::vector<ParamPoint> poly = traceFace(arr, e, guardMax);
    if (poly.size() < 3) continue;
    // The unbounded (outer) face traces CW (negative area); bounded faces trace CCW.
    if (signedArea(poly) <= 0.0) continue;
    faces.push_back(std::move(poly));
  }

  // Keep only faces whose interior lies inside the parent region (inside outer, outside
  // every hole). A face representative interior point = centroid nudged to a true
  // interior sample via the polygon centroid (robust: arrangement faces are convex-ish
  // cells, centroid lands inside).
  std::vector<SubRegion> regions;
  for (auto& poly : faces) {
    // A robust representative interior point of the traced cell (centroid is unreliable
    // for a non-convex arrangement cell — it can fall outside / in a notch).
    const ParamPoint c = interiorPoint(poly);
    if (!inParent(outer, holes, c)) continue;  // a cell of a hole, or outside the outer
    if (!simpleLoop(poly)) {
      r.decline = MultiSplitDecline::DegenerateSubRegion;
      return r;
    }
    const double a = signedArea(poly);
    const double floor = std::max(std::fabs(parentArea), 1.0) * opts.areaFloorFrac;
    if (a < floor) {
      r.decline = MultiSplitDecline::DegenerateSubRegion;
      return r;
    }
    SubRegion sr;
    sr.outer = std::move(poly);
    sr.signedArea = a;
    regions.push_back(std::move(sr));
  }

  if (regions.size() < 2) {
    r.decline = MultiSplitDecline::NoSubdivision;
    return r;
  }

  // ── (5) attribute ISOLATED parent holes (no seam crosses them) to the sub-region
  // that contains them, and subtract their area from that piece's net area. A hole
  // CROSSED by a seam is already part of the arrangement — the traced cells go AROUND
  // it, so its area is already excluded and must NOT be subtracted again (double count).
  auto seamCrossesHole = [&](const std::vector<ParamPoint>& h) {
    for (const auto& s : seams) {
      const std::size_t ns = s.size();
      const std::size_t nh = h.size();
      for (std::size_t i = 0; i + 1 < ns; ++i)
        for (std::size_t j = 0; j < nh; ++j) {
          bool coin = false;
          if (segCross(s[i], s[i + 1], h[j], h[(j + 1) % nh], crossTol, coin)) return true;
        }
    }
    return false;
  };
  for (const auto& h : holes) {
    if (seamCrossesHole(h)) continue;  // crossed ⇒ already resolved by the walk
    // An isolated hole belongs to whichever sub-region's outer loop contains it.
    ParamPoint hp = h.empty() ? ParamPoint{} : h[0];
    for (auto& sr : regions)
      if (pointInPolygon(sr.outer, hp)) {
        sr.signedArea -= std::fabs(signedArea(h));
        sr.holes.push_back(h);
        break;
      }
  }

  // ── (6) SELF-VERIFY: Σ area(sub-regions) == area(parent) (host-checkable) ────
  double tiled = 0.0;
  for (const auto& sr : regions) tiled += sr.signedArea;
  r.tiledArea = tiled;
  r.tilingGap = std::fabs(parentArea - tiled);
  r.subRegions = static_cast<int>(regions.size());
  if (r.tilingGap > std::max(std::fabs(parentArea), 1.0) * opts.tilingTolFrac) {
    r.decline = MultiSplitDecline::TilingGap;
    return r;
  }

  r.regions = std::move(regions);
  r.decline = MultiSplitDecline::Ok;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Convenience overload: split a topology `TrimRegion` (pcurve outer + hole loops) along
// `TrimLoop` seams (closed loops) or raw UV-polyline seams. The pcurve loops are
// flattened via the SAME `flattenTrimLoop` the trim_boolean arc-walk uses (rational
// circle/ellipse pcurves flatten with no sag), so a STEP-read trimmed face composes
// with NO re-derivation.
// ─────────────────────────────────────────────────────────────────────────────
inline MultiSplitResult multiCrossingSplit(const TrimRegion& parent,
                                           const std::vector<TrimLoop>& seamLoops,
                                           const std::vector<std::vector<ParamPoint>>& seamPolys,
                                           const MultiSplitOptions& opts = {}) {
  MultiSplitInput in;
  in.outer = topo::flattenTrimLoop(parent.outer, opts.flattenSegments);
  for (const TrimLoop& h : parent.holes)
    in.holes.push_back(topo::flattenTrimLoop(h, opts.flattenSegments));
  for (const TrimLoop& s : seamLoops) {
    std::vector<ParamPoint> sp = topo::flattenTrimLoop(s, opts.flattenSegments);
    if (!sp.empty()) sp.push_back(sp.front());  // re-close (flattenTrimLoop drops the dup)
    in.seams.push_back(std::move(sp));
  }
  for (const auto& s : seamPolys) in.seams.push_back(s);
  return multiCrossingSplit(in, opts);
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_MULTI_CROSSING_SPLIT_H

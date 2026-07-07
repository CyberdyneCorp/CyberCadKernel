// SPDX-License-Identifier: Apache-2.0
//
// face_split.h — MOAT M2b / B2 (first slice): partition ONE trimmed freeform face
// along an M1-traced seam into TWO genuinely-trimmed sub-faces that TILE it.
//
// ── ROLE ──────────────────────────────────────────────────────────────────────
// The general freeform boolean (M2) needs, between the S3 tracer (M1) and the M0
// welder, a purely-2D combinatorial step: take one trimmed freeform face's outer
// EDGE_LOOP in its own (u,v) plane and the seam polyline (u1,v1) and cut the trimmed
// domain into two closed sub-loops (in / out of the cutter). This header is EXACTLY
// that step for the SIMPLEST reachable case — a CONVEX outer loop cut by ONE seam
// chord that CLEANLY crosses it (enters through one boundary edge, exits through
// another, no tangency, no re-entry). Everything outside that envelope DECLINES
// (returns a NULL split with a measured blocker); a leaky/overlapping split is NEVER
// emitted (self-verify → OCCT fall-through discipline).
//
// ── CONSUMES (never rewrites) ──────────────────────────────────────────────────
//   * M1 seam — ssi::WLine (marching.h). points[i].{u1,v1} is the seam directly in
//     THIS face's (u,v) domain (surface A = this face); read, not re-traced.
//   * The face's outer wire flattening + pcurve evaluation from tessellate/trim.h
//     (pcurveForFace / pcurveValue) — the same UV boundary the M0 mesher builds.
//   * The 2D predicates orient2d / segmentsCross / signedArea from uv_triangulate.h.
//   * SurfaceEvaluator (surface_eval.h) to place the seam / crossing endpoints on the
//     surface for the rebuilt sub-face's 3D edges.
// The two sub-faces it emits are ordinary genuinely-trimmed freeform faces over the
// SAME FaceSurface node, so the M0 FaceMesher meshes them with NO tessellator change.
//
// ── SELF-VERIFY GATE (host-checkable, no OCCT; design D5) ──────────────────────
// Before returning a split the header proves, in the UV plane: exactly one clean
// entry + one clean exit crossing; every interior seam node strictly inside the outer
// loop; each sub-loop a simple polygon with |area| above a scale-relative floor; the
// seam the EXACT shared boundary of both sub-loops (identical UV node sequence,
// opposite order); and area(L1)+area(L2) == area(parent) within a scale-relative
// tolerance. ANY failure → SplitDecline (no sub-faces), with the measured blocker.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FACE_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_FACE_SPLIT_H

#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/tessellate/uv_triangulate.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace ssi = cybercad::native::ssi;
namespace math = cybercad::native::math;

using tess::UV;
using tess::UVPolygon;

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes. `Ok` iff a verified two-face split is returned; every other value
// is an HONEST decline (a first-class outcome) carrying WHY the seam is beyond this
// first slice. The caller falls through to OCCT on any decline.
// ─────────────────────────────────────────────────────────────────────────────
enum class SplitDecline {
  Ok,
  NoOuterLoop,          ///< face has no usable outer wire / < 3 UV boundary points
  EmptySeam,            ///< the WLine seam has < 2 nodes
  CrossingsNot2,        ///< the seam does not cross the outer loop exactly twice
  InteriorNodeOutside,  ///< a seam node between the crossings is not strictly inside (re-entry/tangency)
  DegenerateSubRegion,  ///< a sub-loop has near-zero area or self-intersects
  SeamNotShared,        ///< the seam is not the bit-identical shared boundary of both sub-loops
  TilingGap,            ///< area(L1)+area(L2) != area(parent) beyond tolerance
  RebuildMismatch       ///< a rebuilt sub-face's flattened UV loop does not match its combinatorial loop
};

inline const char* declineName(SplitDecline d) noexcept {
  switch (d) {
    case SplitDecline::Ok: return "Ok";
    case SplitDecline::NoOuterLoop: return "NoOuterLoop";
    case SplitDecline::EmptySeam: return "EmptySeam";
    case SplitDecline::CrossingsNot2: return "CrossingsNot2";
    case SplitDecline::InteriorNodeOutside: return "InteriorNodeOutside";
    case SplitDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case SplitDecline::SeamNotShared: return "SeamNotShared";
    case SplitDecline::TilingGap: return "TilingGap";
    case SplitDecline::RebuildMismatch: return "RebuildMismatch";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// A verified split. `loopIn` / `loopOut` are the two closed UV sub-loops; `seam` is
// the clipped seam chord [E, interior…, X] they SHARE (loopIn walks it X→E, loopOut
// walks it E→X). `faceIn` / `faceOut` are the rebuilt genuinely-trimmed sub-faces
// over the parent's FaceSurface node. The areas are the UV signed-area magnitudes.
// ─────────────────────────────────────────────────────────────────────────────
struct FaceSplit {
  topo::Shape faceIn;   ///< sub-face on the cutter-inside side (centroid u below the seam side)
  topo::Shape faceOut;  ///< sub-face on the other side
  UVPolygon loopIn;
  UVPolygon loopOut;
  UVPolygon seam;       ///< the shared clipped seam chord (E … X)
  double parentArea = 0.0;
  double areaIn = 0.0;
  double areaOut = 0.0;
};

struct SplitResult {
  std::optional<FaceSplit> split;         ///< nullopt on decline
  SplitDecline decline = SplitDecline::Ok;
  int crossings = -1;                     ///< measured boundary-crossing count (blocker witness)
  double tilingGap = 0.0;                 ///< |parent − (in+out)| UV area (blocker witness)
  double measuredArea = 0.0;              ///< parent UV area (scale reference)
  bool ok() const noexcept { return split.has_value(); }
};

struct SplitOptions {
  int segsPerEdge = 1;          ///< outer-loop flatten density per parent edge (1 = straight edges exactly)
  double areaFloorFrac = 1e-6;  ///< min |sub-loop area| as a fraction of the parent (degeneracy floor)
  double tilingTolFrac = 1e-9;  ///< area-sum identity relative tolerance (scale-relative; never weakened to pass)
  double rebuildTolFrac = 1e-6; ///< rebuilt-face UV vs combinatorial loop match tolerance (scale-relative)
};

namespace detail {

using tess::detail::orient2d;
using tess::detail::segmentsCross;

inline double shoelace(const UVPolygon& p) noexcept {
  double a = 0.0;
  const std::size_t n = p.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += p[j].u * p[i].v - p[i].u * p[j].v;
  return 0.5 * a;
}

// One boundary segment of the flattened outer loop, tagged with the parent edge it
// came from and that edge's true curve parameters at the segment ends.
struct BndSeg {
  UV a, b;         ///< segment endpoints (UV)
  int edgeIdx;     ///< index of the parent edge in the outer wire
  double ta, tb;   ///< parent-edge curve parameters at a and b (wire-traversal order)
};

// A crossing of the seam with the outer boundary.
struct Crossing {
  UV p;            ///< the crossing point (UV)
  int seamSeg;     ///< seam polyline segment index (node seamSeg → seamSeg+1)
  int bndSeg;      ///< boundary segment index in `segs`
  double tEdge;    ///< parent-edge curve parameter at the crossing
  int edgeIdx;     ///< parent edge the crossing lands on
};

// Proper (interior) crossing of open segments p0p1 (the seam) and q0q1 (a boundary
// edge). On success returns the along-seam fraction `s` and the crossing point.
inline bool seamCross(const UV& p0, const UV& p1, const UV& q0, const UV& q1, double& s,
                      UV& out) noexcept {
  if (!segmentsCross(p0, p1, q0, q1)) return false;
  const double rx = p1.u - p0.u, ry = p1.v - p0.v;
  const double sx = q1.u - q0.u, sy = q1.v - q0.v;
  const double denom = rx * sy - ry * sx;
  if (std::fabs(denom) < 1e-300) return false;  // parallel (segmentsCross already excluded)
  s = ((q0.u - p0.u) * sy - (q0.v - p0.v) * sx) / denom;
  out = UV{p0.u + s * rx, p0.v + s * ry};
  return true;
}

// Flatten the face's outer wire into boundary segments (each tagged with its parent
// edge + edge params). Mirrors tess::flattenWire but keeps per-edge provenance so a
// crossing can be mapped back to (edge, param) for the sub-face rebuild.
inline bool flattenOuter(const topo::Shape& face, int segsPerEdge, std::vector<BndSeg>& segs,
                         std::vector<topo::Shape>& edges) {
  if (face.isNull() || face.type() != topo::ShapeType::Face) return false;
  const auto& wires = face.tshape()->children();
  if (wires.empty()) return false;
  const topo::Shape& wire = wires[0];
  if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return false;

  int edgeIdx = 0;
  UV prev{};
  bool havePrev = false;
  for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
    const topo::Shape& edge = ex.current();
    const topo::PCurve* pc = tess::pcurveForFace(edge, face);
    if (!pc) continue;
    const auto rr = topo::rangeOf(edge);
    const double first = rr ? rr->first : 0.0;
    const double last = rr ? rr->last : 1.0;
    const bool rev = edge.orientation() == topo::Orientation::Reversed;
    for (int i = 0; i < segsPerEdge; ++i) {
      const double fa = static_cast<double>(i) / segsPerEdge;
      const double fb = static_cast<double>(i + 1) / segsPerEdge;
      const double ta = rev ? last + (first - last) * fa : first + (last - first) * fa;
      const double tb = rev ? last + (first - last) * fb : first + (last - first) * fb;
      const UV a = havePrev ? prev : tess::pcurveValue(*pc, ta, rev ? 1.0 - fa : fa);
      const UV b = tess::pcurveValue(*pc, tb, rev ? 1.0 - fb : fb);
      segs.push_back(BndSeg{a, b, edgeIdx, ta, tb});
      prev = b;
      havePrev = true;
    }
    edges.push_back(edge);
    ++edgeIdx;
  }
  return segs.size() >= 3;
}

// Clamped uniform knot vector for a degree-`deg` B-spline with `nPoles` poles.
inline std::vector<double> clampedKnots(int nPoles, int deg) {
  std::vector<double> k;
  k.reserve(static_cast<std::size_t>(nPoles + deg + 1));
  for (int i = 0; i <= deg; ++i) k.push_back(0.0);
  const int nInner = nPoles - deg - 1;
  for (int i = 1; i <= nInner; ++i) k.push_back(static_cast<double>(i) / (nInner + 1));
  for (int i = 0; i <= deg; ++i) k.push_back(1.0);
  return k;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// Build the shared seam edge from the clipped seam chord. 3D curve = degree-1
// B-spline (polyline) through the on-surface seam points; pcurve = degree-1 B-spline
// through the seam UV nodes (SAME knots). ONE pcurve is attached so both sub-faces
// resolve it via pcurveForFace's single-pcurve fallback. Built once, shared by both
// sub-wires with opposite orientation → the seam is their bit-exact common boundary.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

inline topo::Shape buildSeamEdge(const UVPolygon& seamUV, const std::vector<math::Point3>& seam3d,
                                 const std::shared_ptr<const topo::TShape>& faceNode) {
  const int n = static_cast<int>(seamUV.size());
  const int deg = 1;
  const std::vector<double> knots = clampedKnots(n, deg);

  topo::EdgeCurve c3d{};
  c3d.kind = topo::EdgeCurve::Kind::BSpline;
  c3d.degree = deg;
  c3d.poles = seam3d;
  c3d.knots = knots;

  auto v0 = topo::ShapeBuilder::makeVertex(seam3d.front());
  auto v1 = topo::ShapeBuilder::makeVertex(seam3d.back());
  topo::Shape edge = topo::ShapeBuilder::makeEdge(c3d, 0.0, 1.0, v0, v1);

  topo::PCurve pc{};
  pc.kind = topo::EdgeCurve::Kind::BSpline;
  pc.degree = deg;
  pc.poles2d.reserve(seamUV.size());
  for (const UV& q : seamUV) pc.poles2d.push_back(math::Point3{q.u, q.v, 0.0});
  pc.knots = knots;
  return topo::ShapeBuilder::addPCurve(edge, faceNode, pc);
}

// Build one boundary-arc sub-edge = the parent edge restricted to [tLo,tHi]. Reuses
// the parent's 3D curve and its single pcurve verbatim (D4 "carry parent edges");
// orientation is chosen so the wire traverses tFrom → tTo.
inline topo::Shape restrictEdge(const topo::Shape& parentEdge, const topo::Shape& face,
                                double tFrom, double tTo) {
  const auto cr = topo::curveOf(parentEdge);
  const topo::PCurve* pc = tess::pcurveForFace(parentEdge, face);
  const double tLo = std::min(tFrom, tTo), tHi = std::max(tFrom, tTo);
  topo::EdgeCurve curve = cr ? *cr->curve : topo::EdgeCurve{};
  topo::Shape e = topo::ShapeBuilder::makeEdge(curve, tLo, tHi, topo::Shape{}, topo::Shape{});
  if (pc) e = topo::ShapeBuilder::addPCurve(e, face.tshape(), *pc);
  return tFrom <= tTo ? e : e.reversedShape();
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// splitFace — the B2 first-slice entry point. Returns a verified two-face split, or
// an honest decline with the measured blocker. `seam.points[i].{u1,v1}` is read as
// the seam pcurve on `face`'s own (u,v) domain (surface A = this face).
// ─────────────────────────────────────────────────────────────────────────────
inline SplitResult splitFace(const topo::Shape& face, const ssi::WLine& seam,
                             const SplitOptions& opts = {}) {
  using namespace detail;
  SplitResult r;

  // ── (1) flatten the outer loop; extract the seam polyline ──────────────────
  std::vector<BndSeg> segs;
  std::vector<topo::Shape> edges;
  if (!flattenOuter(face, opts.segsPerEdge, segs, edges)) {
    r.decline = SplitDecline::NoOuterLoop;
    return r;
  }
  if (seam.points.size() < 2) {
    r.decline = SplitDecline::EmptySeam;
    return r;
  }
  const int nB = static_cast<int>(segs.size());
  UVPolygon outer;
  outer.reserve(segs.size());
  for (const BndSeg& s : segs) outer.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outer));
  r.measuredArea = parentArea;

  UVPolygon seamUV;
  seamUV.reserve(seam.points.size());
  for (const ssi::WLinePoint& p : seam.points) seamUV.push_back(UV{p.u1, p.v1});
  const int nS = static_cast<int>(seamUV.size());

  // ── (2) seam × boundary crossings — require exactly one entry + one exit ────
  std::vector<Crossing> xs;
  for (int k = 0; k + 1 < nS; ++k)
    for (int i = 0; i < nB; ++i) {
      double s = 0.0;
      UV cp{};
      if (seamCross(seamUV[k], seamUV[k + 1], segs[i].a, segs[i].b, s, cp)) {
        double f = 0.0;
        {  // along-boundary fraction (for the edge param)
          const double dx = segs[i].b.u - segs[i].a.u, dy = segs[i].b.v - segs[i].a.v;
          f = (std::fabs(dx) >= std::fabs(dy)) ? (cp.u - segs[i].a.u) / dx
                                               : (cp.v - segs[i].a.v) / dy;
        }
        const double tEdge = segs[i].ta + f * (segs[i].tb - segs[i].ta);
        xs.push_back(Crossing{cp, k, i, tEdge, segs[i].edgeIdx});
      }
    }
  r.crossings = static_cast<int>(xs.size());
  if (xs.size() != 2) {
    r.decline = SplitDecline::CrossingsNot2;
    return r;
  }
  // Order the two crossings by seam index: the first is the ENTRY (outside→inside),
  // the second the EXIT (inside→outside).
  if (xs[0].seamSeg > xs[1].seamSeg) std::swap(xs[0], xs[1]);
  const Crossing E = xs[0], X = xs[1];

  // ── (3) every interior seam node strictly inside the outer loop ────────────
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k)
    if (!tess::pointInPolygon(outer, seamUV[k])) {
      r.decline = SplitDecline::InteriorNodeOutside;
      return r;
    }

  // ── (4) clipped seam chord E … X (E, interior nodes, X) ─────────────────────
  UVPolygon seamChord;
  seamChord.push_back(E.p);
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) seamChord.push_back(seamUV[k]);
  seamChord.push_back(X.p);

  // ── (5) two boundary arcs + two sub-loops ──────────────────────────────────
  // Arc1: E forward around the ring to X. Arc2: X forward around the ring to E.
  auto boundaryArc = [&](int fromSeg, int toSeg) {
    UVPolygon arc;
    for (int i = (fromSeg + 1) % nB; i != (toSeg + 1) % nB; i = (i + 1) % nB) arc.push_back(segs[i].a);
    return arc;
  };
  const UVPolygon arc1 = boundaryArc(E.bndSeg, X.bndSeg);  // vertices strictly between E and X
  const UVPolygon arc2 = boundaryArc(X.bndSeg, E.bndSeg);

  UVPolygon loop1;  // E, arc1 (…→X), X, seam interior reversed (…→E)
  loop1.push_back(E.p);
  for (const UV& q : arc1) loop1.push_back(q);
  loop1.push_back(X.p);
  for (int k = X.seamSeg; k >= E.seamSeg + 1; --k) loop1.push_back(seamUV[k]);

  UVPolygon loop2;  // X, arc2 (…→E), E, seam interior forward (…→X)
  loop2.push_back(X.p);
  for (const UV& q : arc2) loop2.push_back(q);
  loop2.push_back(E.p);
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) loop2.push_back(seamUV[k]);

  // ── (6) SELF-VERIFY GATE (design D5) ───────────────────────────────────────
  const double a1 = std::fabs(shoelace(loop1));
  const double a2 = std::fabs(shoelace(loop2));
  const double floor = std::max(parentArea, 1.0) * opts.areaFloorFrac;
  if (a1 < floor || a2 < floor) {
    r.decline = SplitDecline::DegenerateSubRegion;
    return r;
  }
  auto simple = [](const UVPolygon& p) {
    const int n = static_cast<int>(p.size());
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j) {
        if (i == j) continue;
        const int i1 = (i + 1) % n, j1 = (j + 1) % n;
        if (i == j1 || j == i1) continue;  // adjacent edges share a vertex
        if (segmentsCross(p[i], p[i1], p[j], p[j1])) return false;
      }
    return true;
  };
  if (!simple(loop1) || !simple(loop2)) {
    r.decline = SplitDecline::DegenerateSubRegion;
    return r;
  }
  // Exact shared seam: the seam run in loop1 (starting at X.p, just past arc1) walks
  // the seamChord in REVERSE (X…E); the seam run in loop2 (starting at E.p, just past
  // arc2) walks it FORWARD (E…X). Both runs are copied from the one seamChord array,
  // so this is a bit-identity assertion (not a tolerance test); the runs wrap the loop
  // cyclically, so they are indexed modulo the loop length.
  const int m = static_cast<int>(seamChord.size());
  const int n1 = static_cast<int>(loop1.size());
  const int n2 = static_cast<int>(loop2.size());
  const int start1 = 1 + static_cast<int>(arc1.size());  // index of X.p in loop1
  const int start2 = 1 + static_cast<int>(arc2.size());  // index of E.p in loop2
  bool shared = true;
  for (int k = 0; k < m && shared; ++k) {
    const UV& s1 = loop1[(start1 + k) % n1];      // reverse seam run: X … E
    const UV& s2 = loop2[(start2 + k) % n2];      // forward seam run: E … X
    const UV& refRev = seamChord[m - 1 - k];
    const UV& refFwd = seamChord[k];
    if (s1.u != refRev.u || s1.v != refRev.v || s2.u != refFwd.u || s2.v != refFwd.v)
      shared = false;
  }
  if (!shared) {
    r.decline = SplitDecline::SeamNotShared;
    return r;
  }
  const double gap = std::fabs(parentArea - (a1 + a2));
  r.tilingGap = gap;
  if (gap > std::max(parentArea, 1.0) * opts.tilingTolFrac) {
    r.decline = SplitDecline::TilingGap;
    return r;
  }

  // ── (7) place the seam chord on the surface (3D) for the rebuilt sub-edges ──
  const auto sr = topo::surfaceOf(face);
  if (!sr) {
    r.decline = SplitDecline::NoOuterLoop;
    return r;
  }
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(seamChord.size());
  seam3d.push_back(eval.value(E.p.u, E.p.v));
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k)
    seam3d.push_back(eval.value(seamUV[k].u, seamUV[k].v));
  seam3d.push_back(eval.value(X.p.u, X.p.v));

  // ── (8) rebuild the two sub-faces over the SAME FaceSurface node ────────────
  // The seam edge is built ONCE and added to both wires with opposite orientation.
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();

  // Boundary-arc edges for the arc that traverses the parent edges from crossing
  // `from` to crossing `to` (forward around the loop), splicing the seam edge to
  // close the wire. `seamEdge` is added with `seamRev` orientation.
  auto buildWire = [&](const Crossing& from, const Crossing& to, const topo::Shape& seamEdge,
                       bool seamRev) -> topo::Shape {
    std::vector<topo::Shape> wireEdges;
    // Walk boundary segments from `from` (partial) forward to `to` (partial),
    // grouping consecutive segments that belong to the same parent edge into one
    // restricted sub-edge spanning [param at arc-entry, param at arc-exit].
    int i = from.bndSeg;
    double groupStart = from.tEdge;         // arc-entry param on the first edge
    int groupEdge = from.edgeIdx;
    while (true) {
      const bool isLast = (i == to.bndSeg);
      const double segEnd = isLast ? to.tEdge : segs[i].tb;
      const int next = (i + 1) % nB;
      const bool edgeChanges = isLast || segs[next].edgeIdx != groupEdge;
      if (edgeChanges) {
        wireEdges.push_back(restrictEdge(edges[groupEdge], face, groupStart, segEnd));
        if (isLast) break;
        groupEdge = segs[next].edgeIdx;
        groupStart = segs[next].ta;
      }
      i = next;
    }
    wireEdges.push_back(seamRev ? seamEdge.reversedShape() : seamEdge);
    return topo::ShapeBuilder::makeFace(surface, topo::ShapeBuilder::makeWire(std::move(wireEdges)),
                                        {}, orient);
  };

  const topo::Shape seamEdge = buildSeamEdge(seamChord, seam3d, face.tshape());
  // Sub-face 1 boundary arc goes E→X (seam splices X→E: reversed). Sub-face 2 arc
  // goes X→E (seam splices E→X: forward). Opposite seam orientation ⇒ shared edge.
  topo::Shape face1 = buildWire(E, X, seamEdge, /*seamRev=*/true);
  topo::Shape face2 = buildWire(X, E, seamEdge, /*seamRev=*/false);

  // ── (9) rebuild self-verify — the flattened sub-face UV loop must match the
  // combinatorial sub-loop (area) within a scale-relative tolerance, else DECLINE
  // (never emit topology that does not reproduce the proven partition). ──────────
  const tess::UVRegion reg1 = tess::buildRegion(face1, std::max(opts.segsPerEdge, 8));
  const tess::UVRegion reg2 = tess::buildRegion(face2, std::max(opts.segsPerEdge, 8));
  const double rtol = std::max(parentArea, 1.0) * opts.rebuildTolFrac;
  if (!reg1.hasOuter() || !reg2.hasOuter() ||
      std::fabs(std::fabs(shoelace(reg1.outer)) - a1) > rtol ||
      std::fabs(std::fabs(shoelace(reg2.outer)) - a2) > rtol) {
    r.decline = SplitDecline::RebuildMismatch;
    return r;
  }

  // Label in/out by sub-loop centroid u relative to the seam (lower-u side = "in").
  auto centroidU = [](const UVPolygon& p) {
    double s = 0.0;
    for (const UV& q : p) s += q.u;
    return s / static_cast<double>(p.size());
  };
  FaceSplit fs;
  if (centroidU(loop1) <= centroidU(loop2)) {
    fs.faceIn = face1; fs.loopIn = loop1; fs.areaIn = a1;
    fs.faceOut = face2; fs.loopOut = loop2; fs.areaOut = a2;
  } else {
    fs.faceIn = face2; fs.loopIn = loop2; fs.areaIn = a2;
    fs.faceOut = face1; fs.loopOut = loop1; fs.areaOut = a1;
  }
  fs.seam = seamChord;
  fs.parentArea = parentArea;
  r.split = std::move(fs);
  r.decline = SplitDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FACE_SPLIT_H

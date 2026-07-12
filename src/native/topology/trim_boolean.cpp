// SPDX-License-Identifier: Apache-2.0
//
// trim_boolean.cpp — parameter-space trimmed-region boolean (Union / Intersect /
// Difference) on 2-D trim loops sharing a surface (u,v) domain. See trim_boolean.h.
//
// The algorithm is a region-based arc-walk (Greiner–Hormann) on the FLATTENED loop
// polylines (flattenTrimLoop reuses the trimmed_nurbs pcurve evaluator, so rational
// circle/ellipse pcurves flatten with no sag):
//   1. Flatten every loop of both regions to a UV polygon.
//   2. Compute the pairwise A-edge / B-edge crossings. Build each polygon's ring by,
//      for every original edge, emitting its start vertex then that edge's crossings
//      sorted by parametric position — so crossings are shared, mated A↔B vertices.
//   3. Mark each crossing ENTRY / EXIT of the other region (per op) from the even-odd
//      containment of the arc leaving it.
//   4. Trace result loops: walk a polygon ring, hop to the mate at each crossing per the
//      entry/exit flags, and close loops; orient outer CCW / holes CW by signed area.
// With NO transversal crossings the result is decided by containment alone (disjoint /
// nested). A coincident boundary edge, or a touch with no genuine in/out alternation
// (tangential-only), is honest-declined (Degenerate) — never a fabricated region.
//
// OCCT-FREE. src/native/math + src/native/topology only. clang++ -std=c++20, fp64.
//
#include "native/topology/trim_boolean.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <tuple>
#include <vector>

namespace cybercad::native::topology {
namespace {

// ── UV primitives ────────────────────────────────────────────────────────────
double dist(const ParamPoint& a, const ParamPoint& b) noexcept {
  const double du = a.u - b.u, dv = a.v - b.v;
  return std::sqrt(du * du + dv * dv);
}

double signedAreaOf(const std::vector<ParamPoint>& p) noexcept {
  const std::size_t n = p.size();
  if (n < 3) return 0.0;
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += (p[j].u * p[i].v) - (p[i].u * p[j].v);
  return 0.5 * a;
}

// Drop collinear interior vertices from a closed polygon. A straight Line pcurve flattens
// to endpoints PLUS regularly-spaced interior samples; those interior samples are exactly
// collinear, and — being regular — can land ON an A/B crossing point, creating the classic
// GH "crossing through a vertex" degeneracy. Removing them leaves a straight edge as a
// single segment (so a square is 4 vertices) while a genuinely curved (circle/ellipse)
// pcurve keeps all its samples (its consecutive triples are never collinear). `relEps` is
// scaled by the polygon extent so it is scale-relative.
std::vector<ParamPoint> dropCollinear(const std::vector<ParamPoint>& p, double relEps) {
  const std::size_t n = p.size();
  if (n < 3) return p;
  double uMin = p[0].u, uMax = p[0].u, vMin = p[0].v, vMax = p[0].v;
  for (const auto& q : p) {
    uMin = std::min(uMin, q.u); uMax = std::max(uMax, q.u);
    vMin = std::min(vMin, q.v); vMax = std::max(vMax, q.v);
  }
  const double diag = std::sqrt((uMax - uMin) * (uMax - uMin) + (vMax - vMin) * (vMax - vMin));
  const double areaEps = relEps * (diag > 0.0 ? diag : 1.0) * (diag > 0.0 ? diag : 1.0);
  std::vector<ParamPoint> out;
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const ParamPoint& prev = p[(i + n - 1) % n];
    const ParamPoint& cur = p[i];
    const ParamPoint& next = p[(i + 1) % n];
    const double cross =
        (cur.u - prev.u) * (next.v - prev.v) - (cur.v - prev.v) * (next.u - prev.u);
    if (std::fabs(cross) > areaEps) out.push_back(cur);  // keep a genuine corner
  }
  return out.size() >= 3 ? out : p;
}

// Even-odd ray-cast (half-open PNPOLY — the SAME rule trimmed_nurbs::raycast uses).
bool pointInPolygon(const std::vector<ParamPoint>& poly, const ParamPoint& p) noexcept {
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

// A region = outer polygon minus hole polygons (membership as in trimmed_nurbs::classify).
struct Region {
  std::vector<std::vector<ParamPoint>> loops;  // loops[0] = outer, rest = holes
  bool contains(const ParamPoint& p) const noexcept {
    if (loops.empty() || !pointInPolygon(loops[0], p)) return false;
    for (std::size_t i = 1; i < loops.size(); ++i)
      if (pointInPolygon(loops[i], p)) return false;
    return true;
  }
};

// A point genuinely INSIDE the region (in the outer loop, outside every hole). Probes a
// coarse grid over the outer loop's bounding box; returns the first in-region sample. Robust
// for a holed region where the outer centroid can fall in a hole. Falls back to the outer
// centroid if no grid sample lands (a degenerate sliver).
ParamPoint regionInterior(const Region& r) noexcept {
  const auto& o = r.loops.empty() ? std::vector<ParamPoint>{} : r.loops[0];
  double uMin = 0, uMax = 0, vMin = 0, vMax = 0, cu = 0, cv = 0;
  for (std::size_t i = 0; i < o.size(); ++i) {
    if (i == 0) { uMin = uMax = o[0].u; vMin = vMax = o[0].v; }
    uMin = std::min(uMin, o[i].u); uMax = std::max(uMax, o[i].u);
    vMin = std::min(vMin, o[i].v); vMax = std::max(vMax, o[i].v);
    cu += o[i].u; cv += o[i].v;
  }
  if (!o.empty()) { cu /= static_cast<double>(o.size()); cv /= static_cast<double>(o.size()); }
  constexpr int kG = 13;  // odd, so the centre column/row sample the middle
  for (int iy = 1; iy < kG; ++iy)
    for (int ix = 1; ix < kG; ++ix) {
      const ParamPoint p{uMin + (uMax - uMin) * ix / kG, vMin + (vMax - vMin) * iy / kG};
      if (r.contains(p)) return p;
    }
  return {cu, cv};
}

double extentOf(const Region& a, const Region& b) noexcept {
  bool any = false;
  double uMin = 0, uMax = 0, vMin = 0, vMax = 0;
  auto acc = [&](const Region& r) {
    for (const auto& loop : r.loops)
      for (const ParamPoint& q : loop) {
        if (!any) { uMin = uMax = q.u; vMin = vMax = q.v; any = true; continue; }
        uMin = std::min(uMin, q.u); uMax = std::max(uMax, q.u);
        vMin = std::min(vMin, q.v); vMax = std::max(vMax, q.v);
      }
  };
  acc(a); acc(b);
  if (!any) return 1.0;
  const double du = uMax - uMin, dv = vMax - vMin;
  const double e = std::sqrt(du * du + dv * dv);
  return e > 0.0 ? e : 1.0;
}

// Segment–segment transversal crossing (closed form). Sets `coincident` on a real
// collinear overlap so the caller can honest-decline.
struct Xing {
  ParamPoint pt{};
  double sA = 0.0;
  double sB = 0.0;
};

std::optional<Xing> segCross(const ParamPoint& a1, const ParamPoint& a2, const ParamPoint& b1,
                             const ParamPoint& b2, double tol, bool& coincident) noexcept {
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
      if (hi > tol && lo < 1.0 - tol) coincident = true;  // real overlap span
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

// ── Greiner–Hormann node ring ────────────────────────────────────────────────
struct Node {
  ParamPoint p{};
  int next = -1;
  int prev = -1;
  int neighbor = -1;  // mate node in the OTHER polygon (crossings only)
  bool intersect = false;
  bool entry = false;
  bool visited = false;
};

// A raw crossing on an (A-edge, B-edge) pair, before ring assembly.
struct RawCross {
  int aLoop = 0, aEdge = 0;  // edge index (start vertex) on the A loop
  int bLoop = 0, bEdge = 0;
  double aAlpha = 0.0, bAlpha = 0.0;
  ParamPoint pt{};
};

// Build one polygon's rings into `nodes`, returning each loop's ring-head index and, for
// each raw crossing on this side, the index of its inserted node (so mates can be linked).
struct RingBuild {
  std::vector<int> heads;               // ring head node per loop
  std::vector<int> crossNode;           // crossNode[k] = node index of raw crossing k
};

// The crossings lying on (loop li, edge e) of `side`, sorted by their parametric position along
// that edge, as (rawCrossingIndex) values.
std::vector<int> crossingsOnEdge(const std::vector<RawCross>& crosses, bool side, std::size_t li,
                                 std::size_t e) {
  std::vector<std::pair<double, int>> onEdge;
  for (std::size_t k = 0; k < crosses.size(); ++k) {
    const RawCross& c = crosses[k];
    const int cl = side ? c.bLoop : c.aLoop;
    const int ce = side ? c.bEdge : c.aEdge;
    if (cl == static_cast<int>(li) && ce == static_cast<int>(e))
      onEdge.emplace_back(side ? c.bAlpha : c.aAlpha, static_cast<int>(k));
  }
  std::sort(onEdge.begin(), onEdge.end());
  std::vector<int> ids;
  ids.reserve(onEdge.size());
  for (const auto& [alpha, k] : onEdge) ids.push_back(k);
  return ids;
}

RingBuild buildRings(std::vector<Node>& nodes,
                     const std::vector<std::vector<ParamPoint>>& loops,
                     const std::vector<RawCross>& crosses, bool side /*false=A,true=B*/) {
  RingBuild rb;
  rb.crossNode.assign(crosses.size(), -1);
  for (std::size_t li = 0; li < loops.size(); ++li) {
    const auto& L = loops[li];
    const int head = static_cast<int>(nodes.size());
    for (std::size_t e = 0; e < L.size(); ++e) {
      nodes.push_back(Node{L[e]});  // the edge's start vertex
      for (int k : crossingsOnEdge(crosses, side, li, e)) {  // then its crossings, in order
        Node cv;
        cv.p = crosses[k].pt;
        cv.intersect = true;
        rb.crossNode[k] = static_cast<int>(nodes.size());
        nodes.push_back(cv);
      }
    }
    const int tail = static_cast<int>(nodes.size());
    for (int i = head; i < tail; ++i) {  // wire the ring [head, tail)
      nodes[i].next = (i + 1 < tail) ? i + 1 : head;
      nodes[i].prev = (i > head) ? i - 1 : tail - 1;
    }
    rb.heads.push_back(head);
  }
  return rb;
}

// A point a hair PAST node n along its ring edge (its arc's representative interior sample).
ParamPoint arcMidpointAfter(const std::vector<Node>& nodes, int n) noexcept {
  const Node& a = nodes[n];
  const Node& b = nodes[a.next];
  return {0.5 * (a.p.u + b.p.u), 0.5 * (a.p.v + b.p.v)};
}

// ── Phase 1: flatten a TrimRegion to polygons (collinear-simplified). ──
bool flattenRegion(const TrimRegion& r, int segs, Region& reg) {
  if (r.outer.empty()) return false;
  std::vector<ParamPoint> o = dropCollinear(flattenTrimLoop(r.outer, segs), 1e-12);
  if (o.size() < 3) return false;
  reg.loops.push_back(std::move(o));
  for (const TrimLoop& h : r.holes) {
    std::vector<ParamPoint> hp = dropCollinear(flattenTrimLoop(h, segs), 1e-12);
    if (hp.size() < 3) return false;
    reg.loops.push_back(std::move(hp));
  }
  return true;
}

// ── Phase 2: all A/B edge crossings via a uniform-grid broad-phase. ──
// B edges are bucketed into the grid cells their bbox spans; each A edge segment-tests only the
// B edges in the cells it spans — near-linear vs the O(nA·nB) pairwise scan (so a finely-
// flattened circle stays fast). Sets `coincident` on any real collinear-overlap boundary.
std::vector<RawCross> findCrossings(const Region& A, const Region& B, double tol,
                                    bool& coincident) {
  std::vector<RawCross> crosses;
  double uMin = A.loops[0][0].u, uMax = uMin, vMin = A.loops[0][0].v, vMax = vMin;
  std::size_t nBedges = 0;
  for (const Region* r : {&A, &B})
    for (const auto& L : r->loops)
      for (const auto& q : L) {
        uMin = std::min(uMin, q.u); uMax = std::max(uMax, q.u);
        vMin = std::min(vMin, q.v); vMax = std::max(vMax, q.v);
      }
  for (const auto& L : B.loops) nBedges += L.size();

  const int gN = std::max(1, static_cast<int>(std::sqrt(static_cast<double>(nBedges) / 2.0)));
  const double gu = (uMax - uMin) > 0 ? (uMax - uMin) / gN : 1.0;
  const double gv = (vMax - vMin) > 0 ? (vMax - vMin) / gN : 1.0;
  auto cellX = [&](double u) { return std::clamp(static_cast<int>((u - uMin) / gu), 0, gN - 1); };
  auto cellY = [&](double v) { return std::clamp(static_cast<int>((v - vMin) / gv), 0, gN - 1); };
  std::vector<std::vector<std::pair<int, int>>> grid(static_cast<std::size_t>(gN) * gN);
  for (std::size_t bi = 0; bi < B.loops.size(); ++bi) {
    const auto& BL = B.loops[bi];
    for (std::size_t be = 0; be < BL.size(); ++be) {
      const ParamPoint& b1 = BL[be];
      const ParamPoint& b2 = BL[(be + 1) % BL.size()];
      const int x0 = cellX(std::min(b1.u, b2.u)), x1 = cellX(std::max(b1.u, b2.u));
      const int y0 = cellY(std::min(b1.v, b2.v)), y1 = cellY(std::max(b1.v, b2.v));
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
          grid[static_cast<std::size_t>(y) * gN + x].emplace_back(static_cast<int>(bi),
                                                                  static_cast<int>(be));
    }
  }
  for (std::size_t ai = 0; ai < A.loops.size(); ++ai) {
    const auto& AL = A.loops[ai];
    for (std::size_t ae = 0; ae < AL.size(); ++ae) {
      const ParamPoint& a1 = AL[ae];
      const ParamPoint& a2 = AL[(ae + 1) % AL.size()];
      const int x0 = cellX(std::min(a1.u, a2.u)), x1 = cellX(std::max(a1.u, a2.u));
      const int y0 = cellY(std::min(a1.v, a2.v)), y1 = cellY(std::max(a1.v, a2.v));
      for (int y = y0; y <= y1; ++y)
        for (int x = x0; x <= x1; ++x)
          for (const auto& [bi, be] : grid[static_cast<std::size_t>(y) * gN + x]) {
            const auto& BL = B.loops[static_cast<std::size_t>(bi)];
            bool coin = false;
            auto c = segCross(a1, a2, BL[be], BL[(be + 1) % BL.size()], tol, coin);
            if (coin) coincident = true;
            if (!c) continue;
            crosses.push_back({static_cast<int>(ai), static_cast<int>(ae), bi, be, c->sA, c->sB,
                               c->pt});
          }
    }
  }
  // A B edge spanning several cells can be tested twice ⇒ dedupe identical (aEdge,bEdge).
  auto key = [](const RawCross& x) { return std::tie(x.aLoop, x.aEdge, x.bLoop, x.bEdge); };
  std::sort(crosses.begin(), crosses.end(),
            [&](const RawCross& x, const RawCross& y) { return key(x) < key(y); });
  crosses.erase(std::unique(crosses.begin(), crosses.end(),
                            [&](const RawCross& x, const RawCross& y) { return key(x) == key(y); }),
                crosses.end());
  return crosses;
}

// ── No-crossings resolution: the boundaries do not cross, so each region is wholly inside or
//    outside the other; decide by genuine interior points (robust for holed regions). ──
TrimBoolResult resolveNoCrossings(const Region& A, const Region& B, TrimBoolOp op) {
  TrimBoolResult out;
  auto emit = [&](const Region& r) {
    for (const auto& loop : r.loops) {
      ResultLoop rl;
      rl.signedArea = signedAreaOf(loop);
      rl.outer = rl.signedArea > 0.0;
      rl.poly = loop;
      out.loops.push_back(std::move(rl));
      out.area += rl.signedArea;
    }
  };
  const bool aInB = B.contains(regionInterior(A));
  const bool bInA = A.contains(regionInterior(B));
  if (op == TrimBoolOp::Union) {
    if (aInB) emit(B); else if (bInA) emit(A); else { emit(A); emit(B); }
    out.status = out.loops.empty() ? TrimBoolStatus::Empty : TrimBoolStatus::Ok;
  } else if (op == TrimBoolOp::Intersect) {
    if (aInB) emit(A); else if (bInA) emit(B);
    out.status = out.loops.empty() ? TrimBoolStatus::Empty : TrimBoolStatus::Ok;
  } else {  // Difference A∖B
    if (aInB) {
      out.status = TrimBoolStatus::Empty;
    } else if (bInA) {  // B ⊂ A ⇒ A with B as a hole
      emit(A);
      ResultLoop hole;
      hole.poly = B.loops[0];
      if (signedAreaOf(hole.poly) > 0.0) std::reverse(hole.poly.begin(), hole.poly.end());
      hole.signedArea = signedAreaOf(hole.poly);
      hole.outer = false;
      out.area += hole.signedArea;
      out.loops.push_back(std::move(hole));
      out.status = TrimBoolStatus::Ok;
    } else {
      emit(A);
      out.status = TrimBoolStatus::Ok;
    }
  }
  return out;
}

// ── Tangential-only touch: crossings exist but no genuine transversal alternation (the arc
//    leaving each crossing has the SAME containment as the arc entering it). Ambiguous. ──
bool anyTransversal(const std::vector<Node>& nodes, const RingBuild& rbld, const Region& other) {
  for (int head : rbld.heads) {
    int cur = head;
    do {
      if (nodes[cur].intersect) {
        const bool leaveIn = other.contains(arcMidpointAfter(nodes, cur));
        const int pv = nodes[cur].prev;
        const ParamPoint enterMid = {0.5 * (nodes[pv].p.u + nodes[cur].p.u),
                                     0.5 * (nodes[pv].p.v + nodes[cur].p.v)};
        if (leaveIn != other.contains(enterMid)) return true;
      }
      cur = nodes[cur].next;
    } while (cur != head);
  }
  return false;
}

// ── Assemble traced polylines into oriented ResultLoops. Orientation is set by NESTING depth:
//    a loop inside an odd number of others is a HOLE (CW); even depth is an OUTER (CCW). ──
void assembleLoops(std::vector<std::vector<ParamPoint>> polys, double tol, TrimBoolResult& out) {
  std::vector<std::vector<ParamPoint>> cleaned;
  for (auto& poly : polys) {
    std::vector<ParamPoint> clean;
    for (const auto& q : poly)
      if (clean.empty() || dist(clean.back(), q) > tol) clean.push_back(q);
    if (clean.size() >= 2 && dist(clean.front(), clean.back()) <= tol) clean.pop_back();
    clean = dropCollinear(clean, 1e-12);
    if (clean.size() >= 3) cleaned.push_back(std::move(clean));
  }
  if (cleaned.empty()) { out.status = TrimBoolStatus::Empty; return; }

  for (std::size_t i = 0; i < cleaned.size(); ++i) {
    const std::size_t n = cleaned[i].size();
    const ParamPoint probe{(cleaned[i][0].u + cleaned[i][1 % n].u + cleaned[i][n - 1].u) / 3.0,
                           (cleaned[i][0].v + cleaned[i][1 % n].v + cleaned[i][n - 1].v) / 3.0};
    int depth = 0;
    for (std::size_t j = 0; j < cleaned.size(); ++j)
      if (j != i && pointInPolygon(cleaned[j], probe)) ++depth;
    const bool isOuter = (depth % 2 == 0);
    double a = signedAreaOf(cleaned[i]);
    if ((isOuter && a < 0.0) || (!isOuter && a > 0.0)) {  // re-orient to role
      std::reverse(cleaned[i].begin(), cleaned[i].end());
      a = -a;
    }
    ResultLoop rl;
    rl.poly = std::move(cleaned[i]);
    rl.signedArea = a;
    rl.outer = isOuter;
    out.area += a;
    out.loops.push_back(std::move(rl));
  }
  out.status = out.loops.empty() ? TrimBoolStatus::Empty : TrimBoolStatus::Ok;
}

// Mark each crossing's kept leave-direction (`entry`: true ⇒ forward, false ⇒ backward) per the
// op's arc-selection rule, then TRACE the selected arcs into closed polylines (canonical
// Greiner–Hormann walk: leave along the kept direction to the next crossing, hop to its mate,
// repeat). Returns the traced (un-oriented) polylines.
std::vector<std::vector<ParamPoint>> selectAndTrace(std::vector<Node>& nodes, const RingBuild& ra,
                                                    const RingBuild& rb,
                                                    const std::vector<RawCross>& crosses,
                                                    const Region& A, const Region& B,
                                                    TrimBoolOp op) {
  // Keep-rule (against the OTHER region): Intersect keeps INSIDE arcs on both; Union keeps
  // OUTSIDE on both; Difference A∖B keeps A OUTSIDE-B and B INSIDE-A.
  const bool keepA_inside = (op == TrimBoolOp::Intersect);
  const bool keepB_inside = (op != TrimBoolOp::Union);
  int firstBNode = static_cast<int>(nodes.size());
  for (int h : rb.heads) firstBNode = std::min(firstBNode, h);
  auto onB = [&](int idx) { return idx >= firstBNode; };

  for (const RingBuild* rbld : {&ra, &rb})
    for (int head : rbld->heads) {
      int cur = head;
      do {
        if (nodes[cur].intersect) {
          const Region& other = onB(cur) ? A : B;
          const bool fwdInside = other.contains(arcMidpointAfter(nodes, cur));
          const bool wantInside = onB(cur) ? keepB_inside : keepA_inside;
          nodes[cur].entry = (fwdInside == wantInside);  // leave forward iff forward arc kept
        }
        cur = nodes[cur].next;
      } while (cur != head);
    }

  for (auto& nd : nodes) nd.visited = false;
  std::vector<std::vector<ParamPoint>> polys;
  const int guardMax = static_cast<int>(nodes.size()) * 4 + 16;
  for (std::size_t k = 0; k < crosses.size(); ++k) {
    const int start = ra.crossNode[k];
    if (start < 0 || nodes[start].visited) continue;
    std::vector<ParamPoint> loop;
    int cur = start, guard = 0;
    do {
      nodes[cur].visited = true;
      const bool forward = nodes[cur].entry;
      do {
        loop.push_back(nodes[cur].p);
        cur = forward ? nodes[cur].next : nodes[cur].prev;
      } while (!nodes[cur].intersect && ++guard < guardMax);
      nodes[cur].visited = true;
      if (nodes[cur].neighbor < 0) break;
      cur = nodes[cur].neighbor;  // hop to the shared crossing on the other polygon
    } while (cur != start && ++guard < guardMax);
    if (loop.size() >= 3) polys.push_back(std::move(loop));
  }
  return polys;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// trimRegionBoolean — orchestrates the phases (see the file header).
// ─────────────────────────────────────────────────────────────────────────────
TrimBoolResult trimRegionBoolean(const TrimRegion& regionA, const TrimRegion& regionB,
                                 TrimBoolOp op, const TrimBoolOptions& opts) {
  TrimBoolResult out;

  Region A, B;
  if (!flattenRegion(regionA, opts.flattenSegments, A) ||
      !flattenRegion(regionB, opts.flattenSegments, B)) {
    out.status = TrimBoolStatus::Invalid;
    return out;
  }
  const double tol = opts.tol * extentOf(A, B);

  bool coincident = false;
  std::vector<RawCross> crosses = findCrossings(A, B, tol, coincident);
  if (coincident) {  // coincident boundary edge ⇒ ambiguous, honest-decline
    out.status = TrimBoolStatus::Degenerate;
    return out;
  }
  if (crosses.empty()) return resolveNoCrossings(A, B, op);

  // Build both polygons' rings with mated crossing vertices.
  std::vector<Node> nodes;
  RingBuild ra = buildRings(nodes, A.loops, crosses, false);
  RingBuild rb = buildRings(nodes, B.loops, crosses, true);
  for (std::size_t k = 0; k < crosses.size(); ++k) {
    const int an = ra.crossNode[k], bn = rb.crossNode[k];
    if (an < 0 || bn < 0) { out.status = TrimBoolStatus::Degenerate; return out; }
    nodes[an].neighbor = bn;
    nodes[bn].neighbor = an;
  }

  if (!anyTransversal(nodes, ra, B) && !anyTransversal(nodes, rb, A)) {
    out.status = TrimBoolStatus::Degenerate;  // tangential-only touch ⇒ honest-decline
    return out;
  }

  std::vector<std::vector<ParamPoint>> polys = selectAndTrace(nodes, ra, rb, crosses, A, B, op);
  if (polys.empty()) {
    out.status = TrimBoolStatus::Empty;
    return out;
  }
  assembleLoops(std::move(polys), tol, out);
  return out;
}

}  // namespace cybercad::native::topology

// SPDX-License-Identifier: Apache-2.0
//
// cap_hole.h — bounded, opt-in synthesis of ONE cap face for a single simple
// planar hole (MOAT stage M5 tail).
//
// The landed slices DECLINE a shell that sews cleanly (every corner paired within
// `tolerance`) but is simply MISSING one face: the sew leaves a ring of boundary
// edges (each side referenced by exactly one face) and `heal.cpp` reports
// `Unhealed{OpenShell}`. OCCT closes this by synthesizing the absent face
// (`BRepBuilderAPI_MakeFace` on the free-boundary wire + `ShapeFix_Solid`). This
// pass earns the same win natively under an EXPLICIT, BOUNDED, OPT-IN flag — never
// by weakening the weld tolerance and never by fabricating a closure.
//
// ── THE BOUND (why this is not a fabricated closure) ─────────────────────────────
// A cap face is synthesized ONLY when EVERY layer below accepts the hole; otherwise
// the heal declines honestly with the input unchanged (`Unhealed{OpenShell}`):
//
//   1. OPT-IN FLAG. `capPlanarHoles == false` ⇒ this pass never runs (heal.cpp guards
//      it) ⇒ the landed slices are byte-identical. A missing face still declines.
//
//   2. SINGLE SIMPLE BOUNDARY CYCLE. The surviving boundary edges (the `EdgePool`
//      used-once tally, reconstructed here from the sewn faces' shared vertex nodes)
//      MUST form EXACTLY ONE closed cycle in which every boundary vertex has exactly
//      two incident boundary edges. A branching boundary, or TWO OR MORE disjoint
//      loops (two or more missing faces), is declined — this slice caps exactly one.
//
//   3. PLANARITY WITHIN TOLERANCE. Every corner of the boundary loop MUST lie within
//      `tolerance` of the loop's best-fit plane (Newell normal + centroid). A
//      non-planar / curved hole is declined (freeform re-approximation is OCCT's moat).
//
//   4. SIMPLE POLYGON. The boundary loop, projected onto its best-fit plane, MUST be
//      non-self-intersecting (no two non-adjacent edges cross). A self-intersecting
//      boundary is declined.
//
//   5. MANDATORY SELF-VERIFY (in heal.cpp, UNCHANGED). The cap is built from the hole's
//      EXISTING shared vertex nodes, so the two faces meeting on each capped side place
//      identical boundary points and the result welds. After capping + re-sew the
//      candidate must STILL tessellate watertight with positive enclosed volume; a
//      capped candidate that does not is discarded (`Unhealed{SelfVerifyFailed}`).
//      Self-verify — not this pass's bookkeeping — is the authoritative closure check.
//
// ── MULTI-HOLE SUPERSET (opt-in `capMultiplePlanarHoles`) ────────────────────────
// `capAllPlanarHoles` (below) generalizes this to a shell missing TWO OR MORE faces:
// it reuses the IDENTICAL bestFitPlane / maxPlaneDeviation / isSimplePolygon layers,
// only tracing ALL disjoint boundary cycles (`traceAllLoops`) instead of one. It is a
// strict superset — ALL-OR-NOTHING: every hole must be a disjoint simple coplanar
// non-self-intersecting cycle or the WHOLE set is declined (no partial closure). The
// landed single-hole `capPlanarHole` / `traceSingleLoop` are left BYTE-IDENTICAL and
// still run when `capMultiplePlanarHoles == false`.
//
// ── ASYMPTOTIC-TAIL CAVEAT ───────────────────────────────────────────────────────
// Even with the multi-hole superset, this closes only DISJOINT SIMPLE PLANAR holes.
// A non-planar / curved hole, a self-intersecting boundary, a branching boundary,
// pcurve reconstruction, and self-intersecting-wire repair remain OCCT `ShapeFix`'s
// moat and are declined honestly (never claimed here). No new `UnhealedReason` is
// introduced: `OpenShell` already means exactly "boundary edges survive after sewing".
//
// OCCT-FREE. Uses src/native/{math,topology} + the FaceLoop / SewResult sew types
// only. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_CAP_HOLE_H
#define CYBERCAD_NATIVE_HEAL_CAP_HOLE_H

#include "native/heal/face_soup.h"
#include "native/heal/tolerant_sew.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

/// Outcome of the cap pass. When `declined == true` no cap was synthesized (the hole
/// is outside the bound) and `cap` is empty; heal.cpp then keeps the honest
/// `Unhealed{OpenShell}` verdict. When `declined == false`, `cap` carries ONE cap
/// FaceLoop (loop corners + Newell normal) that heal.cpp appends to the working soup
/// before the UNCHANGED re-sew + self-verify.
struct CapResult {
  std::optional<FaceLoop> cap;  ///< the synthesized cap face-loop (empty ⇒ declined)
  bool declined = true;         ///< true ⇒ no cap (out of bound / not a simple hole)
  double planarityDev = 0.0;    ///< max coplanarity deviation of the capped loop (≤ tol)
};

/// Outcome of the multi-hole cap pass (opt-in `capMultiplePlanarHoles`). ALL-OR-NOTHING:
/// when `declined == true` no cap was synthesized (some hole is outside the bound —
/// branching, non-planar, or self-intersecting) and `caps` is empty; heal.cpp then
/// keeps the honest `Unhealed{OpenShell}` verdict with the input unchanged (never a
/// partial closure). When `declined == false`, `caps` carries ONE cap FaceLoop per
/// disjoint simple planar hole (each on its EXISTING shared nodes) that heal.cpp
/// appends to the working soup before the UNCHANGED re-sew + self-verify.
struct MultiCapResult {
  std::vector<FaceLoop> caps;  ///< one cap FaceLoop per hole (empty ⇒ declined)
  bool declined = true;        ///< true ⇒ no caps (any hole out of bound / not simple)
  double planarityDev = 0.0;   ///< max coplanarity deviation across all capped loops (≤ tol)
};

namespace detail {

// A boundary vertex is identified by its shared topology node pointer (unique after
// unification, so two faces meeting on a side reference the same VId).
using VId = const topo::TShape*;

// The boundary-edge graph: adjacency among boundary vertices + their world positions.
struct BoundaryGraph {
  std::unordered_map<VId, std::vector<VId>> adj;  ///< boundary-vertex → boundary neighbours
  std::unordered_map<VId, math::Point3> pos;      ///< world position per boundary vertex
};

// Unordered vertex-node-pair key (canonicalized min/max pointer) + its hash.
struct EKey {
  VId a, b;
  bool operator==(const EKey& o) const noexcept { return a == o.a && b == o.b; }
};
struct EKeyHash {
  std::size_t operator()(const EKey& k) const noexcept {
    return std::hash<VId>{}(k.a) * 1099511628211ull ^ std::hash<VId>{}(k.b);
  }
};
inline EKey ekey(VId a, VId b) noexcept { return a <= b ? EKey{a, b} : EKey{b, a}; }

// Reconstruct the boundary graph from the sewn faces: count how many faces reference
// each undirected shared-vertex pair; a pair used by exactly ONE face is a boundary
// edge (the same used-once tally EdgePool exposes). Boundary vertices + their world
// positions are collected from those boundary edges only.
inline BoundaryGraph boundaryGraph(const SewResult& sr) {
  std::unordered_map<EKey, int, EKeyHash> uses;
  std::unordered_map<VId, math::Point3> allPos;
  for (const SewnFace& sf : sr.faces) {
    const std::size_t n = sf.verts.size();
    if (n < 3) continue;
    for (std::size_t i = 0; i < n; ++i) {
      const VId a = sf.verts[i].tshape().get();
      const VId b = sf.verts[(i + 1) % n].tshape().get();
      if (const auto pa = topo::pointOf(sf.verts[i])) allPos[a] = *pa;
      if (const auto pb = topo::pointOf(sf.verts[(i + 1) % n])) allPos[b] = *pb;
      ++uses[ekey(a, b)];
    }
  }
  BoundaryGraph g;
  for (const auto& [k, count] : uses) {
    if (count != 1) continue;  // shared side (used ≥ 2×) — interior, not a hole edge
    g.adj[k.a].push_back(k.b);
    g.adj[k.b].push_back(k.a);
    g.pos[k.a] = allPos[k.a];
    g.pos[k.b] = allPos[k.b];
  }
  return g;
}

// Trace the boundary into ONE ordered cycle of vertex nodes. Returns the loop (each
// vertex once, not repeated at the end) ONLY when every boundary vertex has exactly
// two incident boundary edges AND they form a single cycle covering ALL boundary
// vertices; returns empty for a branching boundary or ≥ 2 disjoint loops.
inline std::vector<VId> traceSingleLoop(const BoundaryGraph& g) {
  if (g.adj.size() < 3) return {};  // a hole needs ≥ 3 boundary vertices
  for (const auto& [v, nbrs] : g.adj)
    if (nbrs.size() != 2) return {};  // branching / dangling boundary → decline

  const VId start = g.adj.begin()->first;
  std::vector<VId> loop;
  VId prev = nullptr, cur = start;
  do {
    loop.push_back(cur);
    const std::vector<VId>& nbrs = g.adj.at(cur);
    const VId next = (nbrs[0] != prev) ? nbrs[0] : nbrs[1];
    prev = cur;
    cur = next;
  } while (cur != start && loop.size() <= g.adj.size());

  if (cur != start) return {};                 // did not close cleanly
  if (loop.size() != g.adj.size()) return {};  // a second disjoint loop exists
  return loop;
}

// Trace the boundary into ONE OR MORE disjoint ordered cycles of vertex nodes — the
// multi-hole generalization of traceSingleLoop (used only by the opt-in
// `capMultiplePlanarHoles` pass; the landed single-hole path is untouched). Returns
// the cycles ONLY when EVERY boundary vertex has exactly two incident boundary edges
// AND every connected component closes into a simple cycle; returns empty (⇒ decline
// the WHOLE set) for a branching / dangling boundary (any degree != 2) or a component
// that does not close. Each returned loop lists its vertices once (not repeated at the
// end); the loops are vertex-disjoint and together cover ALL boundary vertices.
inline std::vector<std::vector<VId>> traceAllLoops(const BoundaryGraph& g) {
  if (g.adj.size() < 3) return {};  // a hole needs ≥ 3 boundary vertices
  for (const auto& [v, nbrs] : g.adj)
    if (nbrs.size() != 2) return {};  // branching / dangling boundary → decline whole

  std::unordered_map<VId, bool> seen;
  seen.reserve(g.adj.size());
  std::vector<std::vector<VId>> loops;
  for (const auto& kv : g.adj) {
    const VId startV = kv.first;
    if (seen[startV]) continue;  // already consumed by an earlier component's walk
    std::vector<VId> loop;
    VId prev = nullptr, cur = startV;
    do {
      seen[cur] = true;
      loop.push_back(cur);
      const std::vector<VId>& nbrs = g.adj.at(cur);
      const VId next = (nbrs[0] != prev) ? nbrs[0] : nbrs[1];
      prev = cur;
      cur = next;
    } while (cur != startV && loop.size() <= g.adj.size());
    if (cur != startV) return {};  // this component did not close → decline whole
    loops.push_back(std::move(loop));
  }
  return loops;
}

// The loop's best-fit plane, as (centroid, Newell unit normal).
inline std::pair<math::Point3, math::Dir3> bestFitPlane(const std::vector<math::Point3>& loop) {
  math::Vec3 c{0, 0, 0};
  for (const math::Point3& p : loop) c += p.asVec();
  const math::Point3 centroid{c.x / loop.size(), c.y / loop.size(), c.z / loop.size()};
  return {centroid, newellNormal(loop)};
}

// Largest signed-distance magnitude of any corner from the best-fit plane.
inline double maxPlaneDeviation(const std::vector<math::Point3>& loop,
                                const math::Point3& centroid, const math::Dir3& n) {
  double worst = 0.0;
  for (const math::Point3& p : loop)
    worst = std::max(worst, std::fabs(math::dot(p - centroid, n.vec())));
  return worst;
}

// z-component of (b−a)×(c−a) — the 2D orientation test in the plane's (u,v) frame.
inline double cross2(const math::Point3& a, const math::Point3& b, const math::Point3& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

// True iff segments p1p2 and p3p4 properly cross (interiors intersect).
inline bool segmentsCross(const math::Point3& p1, const math::Point3& p2,
                          const math::Point3& p3, const math::Point3& p4) {
  const double d1 = cross2(p3, p4, p1);
  const double d2 = cross2(p3, p4, p2);
  const double d3 = cross2(p1, p2, p3);
  const double d4 = cross2(p1, p2, p4);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0));
}

// Simple-polygon test: project the loop onto its best-fit plane's (u,v) frame and
// decline if any two NON-ADJACENT edges properly cross (a self-intersecting boundary).
inline bool isSimplePolygon(const std::vector<math::Point3>& loop, const math::Point3& centroid,
                            const math::Dir3& n) {
  const math::Vec3 ref = std::fabs(n.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(centroid, n, math::Dir3{ref});
  std::vector<math::Point3> uv;
  uv.reserve(loop.size());
  for (const math::Point3& p : loop) {
    const math::Vec3 d = p - frame.origin;
    uv.push_back(math::Point3{math::dot(d, frame.x.vec()), math::dot(d, frame.y.vec()), 0.0});
  }
  const std::size_t m = uv.size();
  for (std::size_t i = 0; i < m; ++i)
    for (std::size_t j = i + 1; j < m; ++j) {
      if (j == i || (i + 1) % m == j || (j + 1) % m == i) continue;  // adjacent edges
      if (segmentsCross(uv[i], uv[(i + 1) % m], uv[j], uv[(j + 1) % m])) return false;
    }
  return true;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// capPlanarHole — attempt to synthesize ONE cap FaceLoop for a single simple planar
// hole in the sewn shell. Returns a declined CapResult (no cap) unless all four
// geometric layers pass; heal.cpp appends any emitted cap to the working soup and
// re-runs the UNCHANGED sew + self-verify. See the file header for the bound.
// ─────────────────────────────────────────────────────────────────────────────
inline CapResult capPlanarHole(const SewResult& sr, double tol) {
  CapResult out;  // declined by default; cap empty

  // Layer 2: exactly one simple boundary cycle.
  const detail::BoundaryGraph g = detail::boundaryGraph(sr);
  const std::vector<detail::VId> loopIds = detail::traceSingleLoop(g);
  if (loopIds.empty()) return out;

  std::vector<math::Point3> loop;
  loop.reserve(loopIds.size());
  for (const detail::VId v : loopIds) loop.push_back(g.pos.at(v));

  // Layer 3: coplanar within tolerance.
  const auto [centroid, normal] = detail::bestFitPlane(loop);
  const double dev = detail::maxPlaneDeviation(loop, centroid, normal);
  if (dev > tol) return out;

  // Layer 4: simple (non-self-intersecting) polygon in that plane.
  if (!detail::isSimplePolygon(loop, centroid, normal)) return out;

  // All layers pass — emit ONE cap FaceLoop (corners + winding-coherent Newell
  // normal). heal.cpp appends it and re-sews; orientation-fix + self-verify decide
  // the final outcome.
  FaceLoop cap;
  cap.corners = std::move(loop);
  cap.normal = normal;
  cap.valid = true;
  out.cap = std::move(cap);
  out.declined = false;
  out.planarityDev = dev;
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// capAllPlanarHoles — the opt-in `capMultiplePlanarHoles` generalization: attempt to
// synthesize ONE cap FaceLoop per disjoint simple planar hole in the sewn shell. This
// is a strict superset of `capPlanarHole`: it reuses the IDENTICAL bestFitPlane /
// maxPlaneDeviation / isSimplePolygon layers per loop, only trading `traceSingleLoop`
// for `traceAllLoops`. The decision is ALL-OR-NOTHING — if the boundary branches, or
// ANY loop is non-planar or self-intersecting, the WHOLE set is declined (empty caps)
// and heal.cpp keeps `Unhealed{OpenShell}` with the input unchanged (never a partial
// closure). heal.cpp appends every emitted cap to the working soup and re-runs the
// UNCHANGED sew + self-verify, which remains the authoritative closure check.
// ─────────────────────────────────────────────────────────────────────────────
inline MultiCapResult capAllPlanarHoles(const SewResult& sr, double tol) {
  MultiCapResult out;  // declined by default; caps empty

  // Layer 2 (multi): the boundary must resolve into disjoint simple cycles.
  const detail::BoundaryGraph g = detail::boundaryGraph(sr);
  const std::vector<std::vector<detail::VId>> loopIds = detail::traceAllLoops(g);
  if (loopIds.empty()) return out;  // branching / non-closing / < 3 verts ⇒ decline whole

  std::vector<FaceLoop> caps;
  caps.reserve(loopIds.size());
  double worstDev = 0.0;
  for (const std::vector<detail::VId>& ids : loopIds) {
    std::vector<math::Point3> loop;
    loop.reserve(ids.size());
    for (const detail::VId v : ids) loop.push_back(g.pos.at(v));

    // Layers 3 + 4 per loop (UNCHANGED helpers). ANY failure declines the WHOLE set.
    const auto [centroid, normal] = detail::bestFitPlane(loop);
    const double dev = detail::maxPlaneDeviation(loop, centroid, normal);
    if (dev > tol) return out;                                        // non-planar hole
    if (!detail::isSimplePolygon(loop, centroid, normal)) return out;  // self-intersecting

    FaceLoop cap;
    cap.corners = std::move(loop);
    cap.normal = normal;
    cap.valid = true;
    caps.push_back(std::move(cap));
    worstDev = std::max(worstDev, dev);
  }

  // Every loop passed all layers — emit one cap per hole (heal.cpp re-sews + self-verifies).
  out.caps = std::move(caps);
  out.declined = false;
  out.planarityDev = worstDev;
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_CAP_HOLE_H

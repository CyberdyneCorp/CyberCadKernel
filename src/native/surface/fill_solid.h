// SPDX-License-Identifier: Apache-2.0
//
// fill_solid.h — complete an OPEN shell (a solid missing one face) into a WATERTIGHT
// solid by filling its single free-boundary loop with the bounded N-sided patch
// (ngon_fill.h), welded to the shell's existing faces on shared boundary points.
//
// This generalizes heal/cap_hole.h (which caps ONE simple PLANAR hole with a planar
// face) to a SMOOTH (non-planar-boundary) Coons/Gregory tessellated patch. The weld +
// self-verify substrate is REUSED unchanged:
//   * boolean::extractPolygons(shell)  — the shell's existing planar faces as Polygons,
//   * the patch mesh triangles         — one Polygon per triangle (trivially planar),
//   * boolean::assembleSolid(polys)    — welds the whole bag on shared vertices (weld
//                                        tol 1e-7; the patch boundary samples are
//                                        bit-exact so they land in the same weld cell),
//   * heal::self_verify semantics      — watertight across the deflection ladder +
//                                        positive volume, plus isConsistentlyOriented.
// The builder never trusts its own bookkeeping — the self-verify is the authoritative
// closure check, and a candidate that fails is DISCARDED (honest decline).
//
// The PLANAR-boundary case reduces to a planar fan (the patch grid IS planar), so a box
// missing one face is restored EXACTLY (volume restored). The heal cap_hole path is
// left BYTE-IDENTICAL; this is an additive sibling in a new module.
//
// OCCT-FREE. Uses src/native/{math,topology,tessellate,boolean} + ngon_fill.h. clang++
// -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_SURFACE_FILL_SOLID_H
#define CYBERCAD_NATIVE_SURFACE_FILL_SOLID_H

#include "native/boolean/assemble.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/surface/ngon_fill.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace cybercad::native::surface {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace bln = cybercad::native::boolean;

namespace detail {

// Spatial-hash key for welding coincident boundary points (cells of side 1e-7, the
// same weld resolution assembleSolid uses).
inline long long cellCoord(double x) {
  return static_cast<long long>(std::llround(x / 1e-7));
}
struct PKey {
  long long x, y, z;
  bool operator==(const PKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
};
struct PKeyHash {
  std::size_t operator()(const PKey& k) const noexcept {
    std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
    h ^= static_cast<std::size_t>(k.y) * 19349663u;
    h ^= static_cast<std::size_t>(k.z) * 83492791u;
    return h;
  }
};
inline PKey pkey(const math::Point3& p) { return PKey{cellCoord(p.x), cellCoord(p.y), cellCoord(p.z)}; }

// Undirected boundary-edge tally from the shell's polygons: an edge used by exactly
// one polygon is a free-boundary (hole) edge. Mirrors cap_hole's boundaryGraph but on
// world points (position-welded) so it works on the boolean Polygon soup.
struct BoundaryLoop {
  std::vector<math::Point3> corners;  ///< ordered loop of the hole boundary (not closed)
  bool ok = false;
};

struct EK {
  PKey a, b;
  bool operator==(const EK& o) const noexcept { return a == o.a && b == o.b; }
};
struct EKHash {
  std::size_t operator()(const EK& e) const noexcept {
    return PKeyHash{}(e.a) * 1099511628211ull ^ PKeyHash{}(e.b);
  }
};
inline EK ek(const PKey& a, const PKey& b) {
  // canonical order by tuple compare
  const bool aFirst = std::tie(a.x, a.y, a.z) <= std::tie(b.x, b.y, b.z);
  return aFirst ? EK{a, b} : EK{b, a};
}

// Trace the single free-boundary loop of the shell's polygons. Returns ok=false for a
// branching boundary, ≥2 disjoint loops, or a non-closing walk.
inline BoundaryLoop traceHoleLoop(const std::vector<bln::Polygon>& polys) {
  BoundaryLoop out;
  std::unordered_map<EK, int, EKHash> uses;
  std::unordered_map<PKey, math::Point3, PKeyHash> pos;
  for (const bln::Polygon& poly : polys) {
    const std::size_t n = poly.vertices.size();
    if (n < 3) continue;
    for (std::size_t i = 0; i < n; ++i) {
      const PKey a = pkey(poly.vertices[i]);
      const PKey b = pkey(poly.vertices[(i + 1) % n]);
      pos[a] = poly.vertices[i];
      pos[b] = poly.vertices[(i + 1) % n];
      ++uses[ek(a, b)];
    }
  }
  std::unordered_map<PKey, std::vector<PKey>, PKeyHash> adj;
  for (const auto& [k, c] : uses) {
    if (c != 1) continue;  // shared edge (used ≥2) → interior
    adj[k.a].push_back(k.b);
    adj[k.b].push_back(k.a);
  }
  if (adj.size() < 3) return out;
  for (const auto& [v, nb] : adj)
    if (nb.size() != 2) return out;  // branching / dangling
  const PKey start = adj.begin()->first;
  std::vector<PKey> loop;
  PKey prev{0x7fffffff, 0, 0};
  bool hasPrev = false;
  PKey cur = start;
  do {
    loop.push_back(cur);
    const std::vector<PKey>& nb = adj.at(cur);
    const PKey next = (!hasPrev || !(nb[0] == prev)) ? nb[0] : nb[1];
    prev = cur;
    hasPrev = true;
    cur = next;
  } while (!(cur == start) && loop.size() <= adj.size());
  if (!(cur == start)) return out;
  if (loop.size() != adj.size()) return out;  // second disjoint loop
  out.corners.reserve(loop.size());
  for (const PKey& k : loop) out.corners.push_back(pos.at(k));
  out.ok = true;
  return out;
}

// A welded triangle soup: a shared vertex list + oriented index triples.
struct TriSoup {
  std::vector<math::Point3> verts;
  std::vector<std::array<std::uint32_t, 3>> tris;
};

// Weld a bag of Polygons into a shared-vertex triangle soup (fan-triangulate each
// polygon; position-weld corners to 1e-7 cells). Degenerate slivers are skipped.
inline TriSoup weldSoup(const std::vector<bln::Polygon>& polys) {
  TriSoup s;
  std::unordered_map<PKey, std::uint32_t, PKeyHash> idx;
  auto vid = [&](const math::Point3& p) -> std::uint32_t {
    const PKey k = pkey(p);
    if (auto it = idx.find(k); it != idx.end()) return it->second;
    const auto id = static_cast<std::uint32_t>(s.verts.size());
    s.verts.push_back(p);
    idx.emplace(k, id);
    return id;
  };
  for (const bln::Polygon& poly : polys) {
    const std::size_t n = poly.vertices.size();
    for (std::size_t i = 1; i + 1 < n; ++i) {
      const math::Point3& a = poly.vertices[0];
      const math::Point3& b = poly.vertices[i];
      const math::Point3& c = poly.vertices[i + 1];
      if (math::norm(math::cross(b - a, c - a)) < 1e-14) continue;
      s.tris.push_back({vid(a), vid(b), vid(c)});
    }
  }
  return s;
}

// Flood-fill orient the triangle soup: BFS across shared undirected edges, flipping a
// neighbour whose winding traverses the shared edge in the SAME direction (a coherent
// 2-manifold then has every interior edge used once forward, once reversed). Then a
// single global-sign fix flips the WHOLE soup so the signed enclosed volume is
// positive (outward). This is the standard manifold-orientation flood-fill (the same
// idea heal/orient.h uses on faces), applied here on the welded fill soup.
inline void orientSoup(TriSoup& s) {
  std::unordered_map<EK, std::vector<int>, EKHash> em;
  auto vk = [&](std::uint32_t v) { return PKey{static_cast<long long>(v), 0, 0}; };
  auto edge = [&](std::uint32_t a, std::uint32_t b) { return ek(vk(a), vk(b)); };
  for (int i = 0; i < static_cast<int>(s.tris.size()); ++i) {
    const auto& t = s.tris[i];
    em[edge(t[0], t[1])].push_back(i);
    em[edge(t[1], t[2])].push_back(i);
    em[edge(t[2], t[0])].push_back(i);
  }
  std::vector<char> seen(s.tris.size(), 0);
  for (int start = 0; start < static_cast<int>(s.tris.size()); ++start) {
    if (seen[start]) continue;
    std::vector<int> stack{start};
    seen[start] = 1;
    while (!stack.empty()) {
      const int i = stack.back();
      stack.pop_back();
      const std::array<std::uint32_t, 3> t = s.tris[i];
      auto walk = [&](std::uint32_t a, std::uint32_t b) {
        for (const int j : em[edge(a, b)]) {
          if (j == i || seen[j]) continue;
          std::array<std::uint32_t, 3>& u = s.tris[j];
          const bool sameDir = (u[0] == a && u[1] == b) || (u[1] == a && u[2] == b) ||
                               (u[2] == a && u[0] == b);
          if (sameDir) std::swap(u[1], u[2]);
          seen[j] = 1;
          stack.push_back(j);
        }
      };
      walk(t[0], t[1]);
      walk(t[1], t[2]);
      walk(t[2], t[0]);
    }
  }
  double sv = 0.0;
  for (const auto& t : s.tris)
    sv += math::dot(s.verts[t[0]].asVec(), math::cross(s.verts[t[1]].asVec(), s.verts[t[2]].asVec()));
  if (sv < 0.0)
    for (auto& t : s.tris) std::swap(t[1], t[2]);
}

// Build one planar Polygon per oriented triangle of the soup (for assembleSolid). The
// winding is already coherent + outward, so assembleSolid re-welds it preserving the
// consistent orientation.
inline std::vector<bln::Polygon> soupPolygons(const TriSoup& s) {
  std::vector<bln::Polygon> out;
  out.reserve(s.tris.size());
  for (const auto& t : s.tris) {
    const math::Point3& a = s.verts[t[0]];
    const math::Point3& b = s.verts[t[1]];
    const math::Point3& c = s.verts[t[2]];
    const math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::norm(nrm) < 1e-14) continue;
    out.emplace_back(std::vector<math::Point3>{a, b, c},
                     bln::Plane::fromPointNormal(a, math::Dir3{nrm}.vec()));
  }
  return out;
}

// Build the analytic boundary from an ordered loop of world corners: each side is a
// straight segment between consecutive corners (the open-shell hole is bounded by the
// shell's straight edges; arc boundaries are supplied through the cc_fill_ngon path).
inline Boundary segmentBoundary(const std::vector<math::Point3>& corners) {
  Boundary b;
  const std::size_t n = corners.size();
  b.sides.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    BoundarySide s;
    s.start = corners[i];
    s.end = corners[(i + 1) % n];
    s.arc = false;
    b.sides.push_back(s);
  }
  return b;
}

// Insert the patch's boundary samples into every shell polygon edge that lies on the
// hole boundary, so the shell side and the patch share IDENTICAL sub-edges (no
// T-junctions — the weld closes watertight AND consistently oriented). `sideSamples`
// are the patch's ordered per-side samples (endpoints bit-exact); each side runs
// corner→corner. A shell polygon edge (a→b) coincides with a side iff {a,b} equals a
// side's {start,end} endpoints (position-welded); the samples are inserted in the
// edge's own traversal direction. Non-boundary edges are copied unchanged.
inline std::vector<bln::Polygon> stitchBoundary(const std::vector<bln::Polygon>& polys,
                                                const std::vector<std::vector<math::Point3>>& sideSamples) {
  // Index each side by its unordered endpoint key → the ordered interior samples.
  std::unordered_map<EK, const std::vector<math::Point3>*, EKHash> byEnds;
  for (const std::vector<math::Point3>& s : sideSamples) {
    if (s.size() < 2) continue;
    byEnds[ek(pkey(s.front()), pkey(s.back()))] = &s;
  }
  std::vector<bln::Polygon> out;
  out.reserve(polys.size());
  for (const bln::Polygon& poly : polys) {
    const std::size_t n = poly.vertices.size();
    std::vector<math::Point3> nv;
    nv.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      const math::Point3& a = poly.vertices[i];
      const math::Point3& b = poly.vertices[(i + 1) % n];
      nv.push_back(a);
      auto it = byEnds.find(ek(pkey(a), pkey(b)));
      if (it == byEnds.end()) continue;  // not a hole-boundary edge
      const std::vector<math::Point3>& s = *it->second;
      // Emit the INTERIOR samples in the edge's a→b direction (skip both endpoints).
      const bool forward = pkey(s.front()) == pkey(a);
      if (forward) {
        for (std::size_t k = 1; k + 1 < s.size(); ++k) nv.push_back(s[k]);
      } else {
        for (std::size_t k = s.size() - 1; k-- > 1;) nv.push_back(s[k]);
      }
    }
    out.emplace_back(std::move(nv), poly.plane);
  }
  return out;
}

}  // namespace detail

/// Outcome of the solid-fill.
struct FillSolidResult {
  topo::Shape solid;                  ///< welded watertight solid (null ⇒ declined)
  NGonDecline decline = NGonDecline::Ok;
  bool watertight = false;
  double volume = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// fillHoleSolid — complete an open shell by filling its single free-boundary loop.
// `openShell` is a Shell/Solid whose faces leave exactly one hole (its edges used
// once). Builds the bounded patch over the hole's straight-edge boundary, welds it to
// the shell's polygons via assembleSolid, and SELF-VERIFIES. Returns a null solid +
// measured decline for a branching/multi boundary, a >6 / <3-sided hole, a non-planar
// boundary whose patch will not weld, or a self-verify failure. See the header bound.
// ─────────────────────────────────────────────────────────────────────────────
inline FillSolidResult fillHoleSolid(const topo::Shape& openShell, const NGonOptions& opts) {
  FillSolidResult out;
  if (openShell.isNull()) { out.decline = NGonDecline::DegenerateBoundary; return out; }

  // The shell must be all-planar to extract Polygons (its existing faces are planar;
  // the hole boundary they leave is what we fill). A curved shell → OCCT.
  const std::vector<bln::Polygon> shellPolys = bln::extractPolygons(openShell);
  if (shellPolys.size() < 3) { out.decline = NGonDecline::DegenerateBoundary; return out; }

  const detail::BoundaryLoop loop = detail::traceHoleLoop(shellPolys);
  if (!loop.ok) { out.decline = NGonDecline::DegenerateBoundary; return out; }
  const int N = static_cast<int>(loop.corners.size());
  if (N < 3 || N > 6) { out.decline = NGonDecline::TooManySides; return out; }

  // Build + evaluate the patch over the hole's straight-edge boundary.
  const Boundary boundary = detail::segmentBoundary(loop.corners);
  NGonDecline why = NGonDecline::Ok;
  const NGonPatch patch = fillNGon(boundary, opts, &why);
  if (!patch.valid) { out.decline = why; return out; }

  // Stitch the patch's boundary samples into the shell's hole-boundary edges so the
  // shell and patch share identical sub-edges (no T-junctions), then weld the whole
  // (stitched shell + patch triangles) into ONE shared-vertex triangle soup and
  // FLOOD-FILL ORIENT it (coherent 2-manifold + positive-volume global sign). Building
  // assembleSolid from the pre-oriented triangles yields a watertight AND consistently
  // oriented solid even for a subdivided (smooth) patch — the B-rep re-mesh preserves
  // the coherent winding.
  std::vector<bln::Polygon> stitched = detail::stitchBoundary(shellPolys, patch.sideSamples);
  for (const tess::Triangle& t : patch.mesh.triangles) {
    const math::Point3& a = patch.mesh.vertices[t.a];
    const math::Point3& b = patch.mesh.vertices[t.b];
    const math::Point3& c = patch.mesh.vertices[t.c];
    const math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::norm(nrm) < 1e-14) continue;
    stitched.emplace_back(std::vector<math::Point3>{a, b, c},
                          bln::Plane::fromPointNormal(a, math::Dir3{nrm}.vec()));
  }
  detail::TriSoup soup = detail::weldSoup(stitched);
  detail::orientSoup(soup);
  const topo::Shape solid = bln::assembleSolid(detail::soupPolygons(soup));
  if (solid.isNull()) { out.decline = NGonDecline::SelfIntersecting; return out; }

  // Self-verify (heal semantics): watertight across the deflection ladder + positive
  // volume + consistently oriented. A failure is discarded (honest decline).
  bool allClosed = true;
  double vol = 0.0;
  bool oriented = true;
  for (const double d : {0.05, 0.02, 0.01}) {
    tess::MeshParams p;
    p.deflection = d;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(solid);
    if (!tess::isWatertight(m)) { allClosed = false; break; }
    if (!tess::isConsistentlyOriented(m)) oriented = false;
    vol = std::fabs(tess::enclosedVolume(m));
  }
  if (!allClosed || !oriented || !(vol > 0.0)) {
    out.decline = NGonDecline::SelfIntersecting;
    return out;
  }

  out.solid = solid;
  out.watertight = true;
  out.volume = vol;
  out.decline = NGonDecline::Ok;
  return out;
}

}  // namespace cybercad::native::surface

#endif  // CYBERCAD_NATIVE_SURFACE_FILL_SOLID_H

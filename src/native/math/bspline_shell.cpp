// SPDX-License-Identifier: Apache-2.0
//
// bspline_shell.cpp — NURBS roadmap Layer 5 (MULTI-FACE SOLID THICKEN / SHELL).
//
// Clean-room. COMPOSES the Layer-5 offset (bspline_offset.h: offsetSurface — the
// normal-locus fit AND its 2nd-fundamental-form fold guard) and the evaluators
// (bspline.h: nurbsSurfacePoint / surfacePoint / surfaceNormal) into ONE CLOSED,
// watertight triangle shell (tessellate::Mesh) spanning a small B-rep of
// edge-adjacent NURBS faces. Because the offset fit solves linear systems through
// the numsci facade, the WHOLE file is under CYBERCAD_HAS_NUMSCI (mirroring
// bspline_thicken.cpp / bspline_offset.cpp). With the guard OFF this TU is inert
// and the Layer-5 multi-face shell function is absent from the library.
//
// The shell is assembled by SPATIAL WELD:
//   * each face's ORIGINAL cap Sᶠ and OFFSET cap Oᶠ = Sᶠ + d·N are sampled on a
//     (nu × nv) grid, and their triangles are emitted with OPPOSITE cap windings;
//   * a tolerance-bucketed weld deduplicates coincident samples — so two faces
//     sharing a model edge (sampled to the same points on both) collapse to ONE
//     boundary vertex chain, the interior seam is used by two caps (no wall), and
//     a corner where 3+ faces meet welds to a single vertex;
//   * a ruled SIDE WALL is built ONLY on OUTER boundary grid-edges (an S-cap edge
//     used once after the weld), joining the welded S boundary to the welded O
//     boundary.
// Every seam edge ends up used by exactly two triangles → watertight by
// construction. The module VERIFIES closure (isWatertight → χ = 2, zero boundary
// edges; isConsistentlyOriented) and declines a non-closed shell rather than
// returning it.
//
#include "native/math/bspline_shell.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"          // nurbsSurfacePoint / surfacePoint / surfaceNormal
#include "native/math/bspline_offset.h"   // offsetSurface (Layer-5 offset + fold guard)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <span>
#include <unordered_map>
#include <vector>

namespace cybercad::native::math {
namespace {

namespace tess = cybercad::native::tessellate;

int nPolesUof(const BsplineSurfaceData& s) {
  return static_cast<int>(s.knotsU.size()) - s.degreeU - 1;
}
int nPolesVof(const BsplineSurfaceData& s) {
  return static_cast<int>(s.knotsV.size()) - s.degreeV - 1;
}

bool wellFormed(const BsplineSurfaceData& s) {
  if (s.degreeU < 1 || s.degreeV < 1) return false;
  if (s.nPolesU < s.degreeU + 1 || s.nPolesV < s.degreeV + 1) return false;
  if (static_cast<int>(s.poles.size()) != s.nPolesU * s.nPolesV) return false;
  if (nPolesUof(s) != s.nPolesU || nPolesVof(s) != s.nPolesV) return false;
  if (!s.weights.empty() && static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV)
    return false;
  return true;
}

double knotLo(const std::vector<double>& k, int degree) {
  return k[static_cast<std::size_t>(degree)];
}
double knotHi(const std::vector<double>& k, int degree) {
  return k[k.size() - 1 - static_cast<std::size_t>(degree)];
}

Point3 evalS(const BsplineSurfaceData& s, const SurfaceGrid& grid, double u, double v) {
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v);
}
Dir3 evalN(const BsplineSurfaceData& s, const SurfaceGrid& grid, double u, double v) {
  return surfaceNormal(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v);
}

ShellStatus fromOffset(OffsetStatus s) {
  switch (s) {
    case OffsetStatus::DegenerateInput:  return ShellStatus::DegenerateInput;
    case OffsetStatus::DegenerateNormal: return ShellStatus::DegenerateNormal;
    case OffsetStatus::SelfIntersection: return ShellStatus::SelfIntersection;
    case OffsetStatus::FitFailed:        return ShellStatus::OffsetFailed;
    case OffsetStatus::ToleranceNotMet:  return ShellStatus::OffsetFailed;
    case OffsetStatus::Ok:               return ShellStatus::Ok;
  }
  return ShellStatus::OffsetFailed;
}

// ── Spatial vertex weld ─────────────────────────────────────────────────────────
// A tolerance-bucketed position → welded-index map. Two samples within `tol` (in
// each axis, on a snapped grid) collapse to the same vertex. This is what makes a
// shared model edge (sampled to coincident points on both faces) a SINGLE vertex
// chain, so the interior seam is used by two caps (no wall) and corners close.
struct VertexWeld {
  explicit VertexWeld(double tol) : inv_(1.0 / std::max(tol, 1e-15)) {}

  // Snap a coordinate to an integer bucket. To avoid a point landing on a bucket
  // boundary (two nearby samples snapping to adjacent buckets), we probe the 3×3×3
  // neighbourhood of the query bucket for an existing welded vertex within tol.
  std::int64_t bucket(double x) const { return static_cast<std::int64_t>(std::llround(x * inv_)); }

  // Return the welded index for `p`, creating a new mesh vertex if none is within
  // `tol`. Appends to `mesh` on creation.
  std::uint32_t weld(tess::Mesh& mesh, const Point3& p, double tol) {
    const std::int64_t bx = bucket(p.x), by = bucket(p.y), bz = bucket(p.z);
    const double t2 = tol * tol;
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          const Key k{bx + dx, by + dy, bz + dz};
          auto it = map_.find(k);
          if (it == map_.end()) continue;
          for (std::uint32_t idx : it->second) {
            const Vec3 diff = mesh.vertices[idx] - p;
            if (normSquared(diff) <= t2) return idx;
          }
        }
    const std::uint32_t idx = mesh.addVertex(p);
    map_[Key{bx, by, bz}].push_back(idx);
    return idx;
  }

 private:
  struct Key {
    std::int64_t x, y, z;
    bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
      std::size_t h = static_cast<std::size_t>(k.x) * 0x9E3779B97F4A7C15ull;
      h ^= static_cast<std::size_t>(k.y) + 0x9E3779B9 + (h << 6) + (h >> 2);
      h ^= static_cast<std::size_t>(k.z) + 0x9E3779B9 + (h << 6) + (h >> 2);
      return h;
    }
  };
  double inv_;
  std::unordered_map<Key, std::vector<std::uint32_t>, KeyHash> map_;
};

// Euler characteristic V − E + F of a triangle mesh.
int eulerChar(const tess::Mesh& m) {
  const auto counts = tess::edgeUseCounts(m);
  const int V = static_cast<int>(m.vertexCount());
  const int E = static_cast<int>(counts.size());
  const int F = static_cast<int>(m.triangleCount());
  return V - E + F;
}

// Coherent orientation by BFS across shared edges (identical contract to the
// single-patch thicken): flip any neighbour that traverses a shared edge the SAME
// way. Returns false on a non-manifold edge (≥ 3 triangles), in which case the
// caller declines.
bool orientCoherently(tess::Mesh& m) {
  const std::size_t nT = m.triangles.size();
  if (nT == 0) return false;

  struct EdgePair { int t0 = -1, t1 = -1; };
  std::unordered_map<tess::UndirectedEdge, EdgePair, tess::UndirectedEdgeHash> inc;
  inc.reserve(nT * 3);
  auto addEdge = [&](std::uint32_t x, std::uint32_t y, int t) -> bool {
    const tess::UndirectedEdge e{std::min(x, y), std::max(x, y)};
    EdgePair& p = inc[e];
    if (p.t0 < 0) p.t0 = t;
    else if (p.t1 < 0) p.t1 = t;
    else return false;  // ≥ 3 triangles on an edge — non-manifold
    return true;
  };
  for (std::size_t t = 0; t < nT; ++t) {
    const tess::Triangle& tri = m.triangles[t];
    if (!addEdge(tri.a, tri.b, static_cast<int>(t))) return false;
    if (!addEdge(tri.b, tri.c, static_cast<int>(t))) return false;
    if (!addEdge(tri.c, tri.a, static_cast<int>(t))) return false;
  }

  auto hasDirected = [&](int t, std::uint32_t x, std::uint32_t y) {
    const tess::Triangle& tr = m.triangles[static_cast<std::size_t>(t)];
    return (tr.a == x && tr.b == y) || (tr.b == x && tr.c == y) ||
           (tr.c == x && tr.a == y);
  };

  std::vector<char> visited(nT, 0);
  std::vector<int> stack;
  for (std::size_t seed = 0; seed < nT; ++seed) {
    if (visited[seed]) continue;
    visited[seed] = 1;
    stack.push_back(static_cast<int>(seed));
    while (!stack.empty()) {
      const int t = stack.back();
      stack.pop_back();
      const std::uint32_t va = m.triangles[static_cast<std::size_t>(t)].a;
      const std::uint32_t vb = m.triangles[static_cast<std::size_t>(t)].b;
      const std::uint32_t vc = m.triangles[static_cast<std::size_t>(t)].c;
      const std::uint32_t tv[3][2] = {{va, vb}, {vb, vc}, {vc, va}};
      for (auto& ed : tv) {
        const tess::UndirectedEdge key{std::min(ed[0], ed[1]), std::max(ed[0], ed[1])};
        const EdgePair& p = inc[key];
        const int nb = (p.t0 == t) ? p.t1 : p.t0;
        if (nb < 0 || visited[static_cast<std::size_t>(nb)]) continue;
        if (hasDirected(nb, ed[0], ed[1]))
          std::swap(m.triangles[static_cast<std::size_t>(nb)].b,
                    m.triangles[static_cast<std::size_t>(nb)].c);
        visited[static_cast<std::size_t>(nb)] = 1;
        stack.push_back(nb);
      }
    }
  }
  return true;
}

// ── Per-face sampled caps ────────────────────────────────────────────────────────
// A face's S-cap and O-cap welded vertex indices on the (nu × nv) grid, row-major
// (i over U, j over V). sIdx[i*nv+j] / oIdx[i*nv+j] are welded mesh indices.
struct FaceCaps {
  int nu = 0, nv = 0;
  std::vector<std::uint32_t> sIdx;  // S-cap welded indices
  std::vector<std::uint32_t> oIdx;  // O-cap welded indices
  std::vector<Point3> sPos;         // S-cap positions (for mitre apex solve)
  std::vector<Vec3> nrm;            // unit surface normals at each node (mitre apex solve)
};

inline std::size_t gflat(int i, int j, int nv) {
  return static_cast<std::size_t>(i) * static_cast<std::size_t>(nv) +
         static_cast<std::size_t>(j);
}

// The boundary node chain of a patch grid along one parametric edge, as a list of
// (i,j) grid coordinates in the edge's natural direction. Used to compare two
// faces' shared edges (do they sample coincident?) and to enumerate wall segments.
std::vector<std::pair<int, int>> edgeNodes(PatchEdge e, int nu, int nv) {
  std::vector<std::pair<int, int>> out;
  switch (e) {
    case PatchEdge::U0: for (int j = 0; j < nv; ++j) out.push_back({0, j}); break;       // u=lo
    case PatchEdge::U1: for (int j = 0; j < nv; ++j) out.push_back({nu - 1, j}); break;  // u=hi
    case PatchEdge::V0: for (int i = 0; i < nu; ++i) out.push_back({i, 0}); break;       // v=lo
    case PatchEdge::V1: for (int i = 0; i < nu; ++i) out.push_back({i, nv - 1}); break;  // v=hi
  }
  return out;
}

// Sample ONE face's S-cap and O-cap on the shared (nu × nv) grid into `mesh` (via the
// shared weld), filling `fc` (welded S/O indices + S positions + normals) and adding the
// face's mid-surface (S) area into `midArea2`. Returns Ok, or a degeneracy status (a
// collapsed parameter domain / a near-null normal) — the caller declines on non-Ok.
ShellStatus sampleOneFaceCaps(const BsplineSurfaceData& s, int nu, int nv, double d,
                              double wtol, tess::Mesh& mesh, VertexWeld& weld,
                              FaceCaps& fc, double& midArea2) {
  const SurfaceGrid grid{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  const double u0 = knotLo(s.knotsU, s.degreeU), u1 = knotHi(s.knotsU, s.degreeU);
  const double v0 = knotLo(s.knotsV, s.degreeV), v1 = knotHi(s.knotsV, s.degreeV);
  if (!(u1 > u0) || !(v1 > v0)) return ShellStatus::DegenerateInput;

  fc.nu = nu;
  fc.nv = nv;
  const std::size_t n = static_cast<std::size_t>(nu) * nv;
  fc.sIdx.resize(n);
  fc.oIdx.resize(n);
  fc.sPos.resize(n);
  fc.nrm.resize(n);
  for (int i = 0; i < nu; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / (nu - 1));
    for (int j = 0; j < nv; ++j) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / (nv - 1));
      const Point3 p = evalS(s, grid, u, v);
      const Dir3 nrm = evalN(s, grid, u, v);
      if (!nrm.valid()) return ShellStatus::DegenerateNormal;
      const std::size_t g = gflat(i, j, nv);
      fc.sPos[g] = p;
      fc.nrm[g] = nrm.vec();
      fc.sIdx[g] = weld.weld(mesh, p, wtol);
      fc.oIdx[g] = weld.weld(mesh, p + nrm.vec() * d, wtol);
    }
  }
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j) {
      const Point3& a = fc.sPos[gflat(i, j, nv)];
      const Point3& b = fc.sPos[gflat(i + 1, j, nv)];
      const Point3& c = fc.sPos[gflat(i + 1, j + 1, nv)];
      const Point3& e = fc.sPos[gflat(i, j + 1, nv)];
      midArea2 += norm(cross(b - a, c - a)) + norm(cross(c - a, e - a));
    }
  return ShellStatus::Ok;
}

// One DIHEDRAL shared seam whose two offset caps diverge and must be joined by a MITRE:
// the two matched O-cap index chains (oA/oB), the shared S-vertex chain, and the mitre-apex
// chain (where the two offset planes meet). `apexValid` ⇔ every apex is a finite corner.
struct MitreSeam {
  std::vector<std::uint32_t> oA, oB;  // matched (aligned) O-cap index chains to bridge
  std::vector<std::uint32_t> s;       // the shared S-vertex chain (welded model edge)
  std::vector<std::uint32_t> apex;    // the mitre-apex vertex chain (offset planes' meet)
  bool apexValid = false;             // true ⇔ every apex has a finite extend-to-meet corner
};

// Mitre apex for two unit face normals nA,nB and thickness d at a shared S-point: the point
// X = S + α nA + β nB satisfying (X−S)·nA = (X−S)·nB = d (both offset planes). By symmetry
// α = β = d/(1+nA·nB); the offset planes meet there (the proper mitre corner, not a
// corner-chopping chamfer). Near-anti-parallel faces (1+c ≈ 0) meet at infinity → no apex.
bool mitreApex(const Point3& sp, const Vec3& nA, const Vec3& nB, double d, Point3& out) {
  const double denom = 1.0 + dot(nA, nB);
  if (std::fabs(denom) < 1e-9) return false;  // anti-parallel — no finite apex
  out = sp + (nA + nB) * (d / denom);
  return true;
}

// A ruled band between two aligned index chains p[] and q[] (each unit quad split into two
// triangles), skipping degenerate segments. The reusable mitre-panel / offset-bridge builder.
void ruledBand(tess::Mesh& mesh, const std::vector<std::uint32_t>& p,
               const std::vector<std::uint32_t>& q) {
  for (std::size_t k = 0; k + 1 < p.size(); ++k) {
    const std::uint32_t a0 = p[k], a1 = p[k + 1], b0 = q[k], b1 = q[k + 1];
    if (a0 == b0 && a1 == b1) continue;  // this segment already coincides
    if (a1 != b1 && a0 != a1) mesh.addTriangle(a0, a1, b1);
    if (b1 != b0 && a0 != b0) mesh.addTriangle(a0, b1, b0);
  }
}

// Emit the mitre panels for every dihedral seam and return the count of bridged segments.
// A seam with a finite apex extends both offset planes to their true meeting corner (no
// volume-chopping chamfer); a degenerate seam falls back to a direct chamfer bridge. Each
// OUTER-corner endpoint (a seam end reached by both side-wall rails, per `hasRail`) is
// sealed with corner triangles to the adjacent walls.
std::size_t emitMitreSeams(
    tess::Mesh& mesh, const std::vector<MitreSeam>& mitres,
    const std::unordered_map<tess::UndirectedEdge, int, tess::UndirectedEdgeHash>& wallRail) {
  auto hasRail = [&](std::uint32_t x, std::uint32_t y) {
    return wallRail.count(tess::UndirectedEdge{std::min(x, y), std::max(x, y)}) > 0;
  };
  std::size_t mitreEdges = 0;
  for (const MitreSeam& seam : mitres) {
    ruledBand(mesh, seam.oA, seam.apexValid ? seam.apex : seam.oB);  // oA → apex (or oB chamfer)
    if (seam.apexValid) ruledBand(mesh, seam.apex, seam.oB);         // apex → oB
    for (std::size_t k = 0; k + 1 < seam.oA.size(); ++k) ++mitreEdges;
    // Seal each OUTER-corner endpoint (both side-wall rails present) to the adjacent walls.
    const std::size_t last = seam.s.size() - 1;
    for (std::size_t end : {std::size_t{0}, last}) {
      const std::uint32_t sc = seam.s[end], oa = seam.oA[end], ob = seam.oB[end];
      if (!(hasRail(sc, oa) && hasRail(sc, ob))) continue;  // interior junction — skip
      if (seam.apexValid) {
        const std::uint32_t ap = seam.apex[end];
        if (oa != ap && sc != oa) mesh.addTriangle(sc, oa, ap);
        if (ob != ap && sc != ob) mesh.addTriangle(sc, ap, ob);
      } else if (oa != ob) {
        mesh.addTriangle(sc, oa, ob);
      }
    }
  }
  return mitreEdges;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Multi-face solid thicken / shell (Layer 5): offset each face, weld shared edges,
// wall only the outer boundary → one closed shell.
// ─────────────────────────────────────────────────────────────────────────────

ShellResult thickenPatches(const std::vector<BsplineSurfaceData>& faces,
                           const std::vector<SharedEdge>& adjacency, double d,
                           double tol, int gridU, int gridV, double weldTol) {
  ShellResult r;

  if (faces.empty()) {
    r.status = ShellStatus::DegenerateInput;
    return r;
  }
  for (const BsplineSurfaceData& f : faces)
    if (!wellFormed(f)) {
      r.status = ShellStatus::DegenerateInput;
      return r;
    }
  if (std::fabs(d) <= kLinearTolerance) {
    r.status = ShellStatus::ZeroThickness;
    return r;
  }
  const int nu = std::max(2, gridU);
  const int nv = std::max(2, gridV);
  const double wtol = std::max(weldTol, 1e-12);

  for (const SharedEdge& a : adjacency)
    if (a.faceA >= faces.size() || a.faceB >= faces.size() || a.faceA == a.faceB) {
      r.status = ShellStatus::DegenerateInput;
      return r;
    }

  // ── Offset each face (carries the fold + degenerate guards) ────────────────────
  r.minCurvatureRadius = 0.0;
  for (const BsplineSurfaceData& f : faces) {
    const OffsetResult off = offsetSurface(f, d, tol);
    r.maxOffsetError = std::max(r.maxOffsetError, off.maxError);
    if (off.minCurvatureRadius > 0.0)
      r.minCurvatureRadius = (r.minCurvatureRadius == 0.0)
                                 ? off.minCurvatureRadius
                                 : std::min(r.minCurvatureRadius, off.minCurvatureRadius);
    if (!off.ok) {
      r.status = fromOffset(off.status);
      return r;  // decline: degenerate normal / self-intersection / fit failure
    }
  }

  // ── Sample every face's S-cap and O-cap on a shared (nu × nv) grid, welding
  //    coincident vertices into ONE global vertex list ─────────────────────────────
  tess::Mesh mesh;
  VertexWeld weld(wtol);
  std::vector<FaceCaps> caps(faces.size());
  double midArea2 = 0.0;  // 2× total S-cap area accumulator

  for (std::size_t fi = 0; fi < faces.size(); ++fi) {
    const ShellStatus st =
        sampleOneFaceCaps(faces[fi], nu, nv, d, wtol, mesh, weld, caps[fi], midArea2);
    if (st != ShellStatus::Ok) {
      r.status = st;
      return r;
    }
  }
  r.surfaceAreaMid = 0.5 * midArea2;

  // ── Verify each adjacency record and classify its offset side ──────────────────
  // The shared MODEL edge is the ORIGINAL (S) edge: the two faces' S-samples along it
  // MUST weld to the same vertex chain (else the B-rep is inconsistent — decline). The
  // OFFSET (O) edge either welds too (coplanar/tangent seam → offset caps meet directly)
  // or diverges (dihedral corner → a MITRE strip must bridge the offset gap). We record
  // the matched O-index chains of dihedral seams here and emit mitre strips after the
  // caps are laid down. A record whose S-edges do NOT weld is an AdjacencyMismatch.
  std::vector<MitreSeam> mitres;
  for (const SharedEdge& a : adjacency) {
    const auto ea = edgeNodes(a.edgeA, nu, nv);
    auto eb = edgeNodes(a.edgeB, nu, nv);
    if (a.reversed) std::reverse(eb.begin(), eb.end());
    if (ea.size() != eb.size()) {
      r.status = ShellStatus::AdjacencyMismatch;
      return r;
    }
    bool offsetWelds = true;
    MitreSeam seam;
    seam.oA.reserve(ea.size());
    seam.oB.reserve(ea.size());
    seam.s.reserve(ea.size());
    seam.apex.reserve(ea.size());
    bool apexOk = true;
    for (std::size_t k = 0; k < ea.size(); ++k) {
      const std::size_t ga = gflat(ea[k].first, ea[k].second, nv);
      const std::size_t gb = gflat(eb[k].first, eb[k].second, nv);
      const std::uint32_t sa = caps[a.faceA].sIdx[ga];
      const std::uint32_t sb = caps[a.faceB].sIdx[gb];
      const std::uint32_t oa = caps[a.faceA].oIdx[ga];
      const std::uint32_t ob = caps[a.faceB].oIdx[gb];
      // The weld should have collapsed the coincident MODEL-edge samples to one index;
      // if two supposedly-shared S-samples did not weld, the B-rep is inconsistent.
      if (sa != sb) {
        r.status = ShellStatus::AdjacencyMismatch;
        return r;
      }
      if (oa != ob) offsetWelds = false;  // dihedral: offset caps diverge here
      seam.oA.push_back(oa);
      seam.oB.push_back(ob);
      seam.s.push_back(sa);
      // Mitre apex where the two offset planes meet (proper extend-to-meet corner).
      Point3 apexP;
      if (mitreApex(caps[a.faceA].sPos[ga], caps[a.faceA].nrm[ga], caps[a.faceB].nrm[gb],
                    d, apexP))
        seam.apex.push_back(weld.weld(mesh, apexP, wtol));
      else
        apexOk = false;
    }
    ++r.interiorSharedEdges;
    if (!offsetWelds) {
      seam.apexValid = apexOk;
      mitres.push_back(std::move(seam));  // needs a mitre bridge
    }
  }

  // ── Emit cap triangles (welded indices) ────────────────────────────────────────
  // S-caps one winding, O-caps the opposite. After the weld a shared model edge is a
  // single vertex chain, so its S-cap edge is used by BOTH adjacent faces' caps (used
  // twice = interior, no wall). Outer S-cap edges are used once → walled below.
  auto emitCapQuad = [&](const std::vector<std::uint32_t>& idx, int i, int j, bool flip) {
    const std::uint32_t a = idx[gflat(i, j, nv)];
    const std::uint32_t b = idx[gflat(i + 1, j, nv)];
    const std::uint32_t c = idx[gflat(i + 1, j + 1, nv)];
    const std::uint32_t e = idx[gflat(i, j + 1, nv)];
    if (!flip) {
      mesh.addTriangle(a, b, c);
      mesh.addTriangle(a, c, e);
    } else {
      mesh.addTriangle(a, c, b);
      mesh.addTriangle(a, e, c);
    }
  };
  for (const FaceCaps& fc : caps) {
    for (int i = 0; i < nu - 1; ++i)
      for (int j = 0; j < nv - 1; ++j) {
        emitCapQuad(fc.sIdx, i, j, /*flip=*/false);
        emitCapQuad(fc.oIdx, i, j, /*flip=*/true);
      }
  }

  // ── Side walls on OUTER boundary grid-edges only ────────────────────────────────
  // An S-cap boundary GRID-EDGE (a unit segment along a patch boundary) is OUTER iff,
  // after welding, the undirected S-edge it spans is used by exactly ONE cap triangle.
  // (An interior shared edge is used by two caps.) For each outer S grid-edge we raise
  // a ruled wall to the matching O grid-edge, reusing the welded S/O boundary vertices.
  // The count of cap uses is taken over the S-cap triangles emitted so far.
  const std::size_t nCapTris = mesh.triangles.size();
  std::unordered_map<tess::UndirectedEdge, int, tess::UndirectedEdgeHash> capEdgeUse;
  capEdgeUse.reserve(nCapTris * 3);
  auto bumpCap = [&](std::uint32_t x, std::uint32_t y) {
    ++capEdgeUse[tess::UndirectedEdge{std::min(x, y), std::max(x, y)}];
  };
  for (std::size_t t = 0; t < nCapTris; ++t) {
    const tess::Triangle& tr = mesh.triangles[t];
    bumpCap(tr.a, tr.b);
    bumpCap(tr.b, tr.c);
    bumpCap(tr.c, tr.a);
  }
  auto capUses = [&](std::uint32_t x, std::uint32_t y) -> int {
    auto it = capEdgeUse.find(tess::UndirectedEdge{std::min(x, y), std::max(x, y)});
    return it == capEdgeUse.end() ? 0 : it->second;
  };

  // Track the S→O "rail" edges the walls create (a boundary S-vertex to its own O-vertex).
  // A mitre-seam endpoint that is an OUTER corner has exactly such a rail on each side; the
  // corner triangle (shared-S, oA_end, oB_end) then closes the three-panel corner.
  std::unordered_map<tess::UndirectedEdge, int, tess::UndirectedEdgeHash> wallRail;
  auto wallQuad = [&](std::uint32_t s0, std::uint32_t s1, std::uint32_t o0, std::uint32_t o1) {
    mesh.addTriangle(s0, s1, o1);
    mesh.addTriangle(s0, o1, o0);
    ++wallRail[tess::UndirectedEdge{std::min(s0, o0), std::max(s0, o0)}];
    ++wallRail[tess::UndirectedEdge{std::min(s1, o1), std::max(s1, o1)}];
  };
  // Walk each face's four boundary edges; for every unit grid-segment whose S-edge is
  // used by exactly one cap triangle (outer), raise a wall. A segment on an interior
  // shared edge is used twice by caps and is skipped (no double-wall). Each outer
  // segment is emitted by exactly the one face that owns it (the other face doesn't
  // have that boundary segment as its own boundary), so no wall is duplicated.
  const PatchEdge kEdges[4] = {PatchEdge::U0, PatchEdge::U1, PatchEdge::V0, PatchEdge::V1};
  for (const FaceCaps& fc : caps) {
    for (PatchEdge pe : kEdges) {
      const auto nodes = edgeNodes(pe, nu, nv);
      for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
        const std::size_t g0 = gflat(nodes[k].first, nodes[k].second, nv);
        const std::size_t g1 = gflat(nodes[k + 1].first, nodes[k + 1].second, nv);
        const std::uint32_t s0 = fc.sIdx[g0], s1 = fc.sIdx[g1];
        const std::uint32_t o0 = fc.oIdx[g0], o1 = fc.oIdx[g1];
        if (s0 == s1) continue;  // degenerate (collapsed) boundary segment — no wall
        if (capUses(s0, s1) != 1) continue;  // interior shared edge → NO wall
        wallQuad(s0, s1, o0, o1);
        ++r.wallEdges;
      }
    }
  }

  // ── Mitre panels on DIHEDRAL shared edges (offset caps diverge) ─────────────────
  // At a coplanar/tangent seam the two offset caps already welded (interior, closed). At a
  // DIHEDRAL corner they separate; emitMitreSeams extends both offset planes to their true
  // meeting corner (apex) — or chamfers if no finite apex — and seals each outer corner to
  // the adjacent side walls. Returns the number of bridged segments.
  r.mitreEdges = emitMitreSeams(mesh, mitres, wallRail);

  // ── Orientation: coherent winding, then outward (volume sign > 0) ────────────────
  if (!orientCoherently(mesh)) {
    r.status = ShellStatus::NonManifold;  // ≥ 3 caps on a seam — never returned as valid
    return r;
  }
  double vol = tess::enclosedVolume(mesh);
  if (vol < 0.0) {
    for (tess::Triangle& t : mesh.triangles) std::swap(t.b, t.c);
    vol = -vol;
  }

  // ── Verify closure — decline anything not watertight / not coherently oriented ───
  r.watertight = tess::isWatertight(mesh);
  r.consistentlyOriented = tess::isConsistentlyOriented(mesh);
  r.boundaryEdges = tess::boundaryEdgeCount(mesh);
  r.eulerCharacteristic = eulerChar(mesh);
  if (!r.watertight || !r.consistentlyOriented || r.boundaryEdges != 0 ||
      r.eulerCharacteristic != 2) {
    r.status = ShellStatus::NotClosed;  // never return a non-closed / double-walled solid
    return r;
  }

  r.solid = std::move(mesh);
  r.enclosedVolume = vol;
  r.gridU = nu;
  r.gridV = nv;
  r.ok = true;
  r.status = ShellStatus::Ok;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

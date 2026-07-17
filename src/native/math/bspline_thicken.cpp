// SPDX-License-Identifier: Apache-2.0
//
// bspline_thicken.cpp — NURBS roadmap Layer 5 (SOLID THICKEN / SHELL) implementation.
//
// Clean-room. It COMPOSES the Layer-5 offset (bspline_offset.h: offsetSurface — the
// normal-locus fit AND its 2nd-fundamental-form self-intersection guard) and the
// evaluators (bspline.h: nurbsSurfacePoint / surfaceNormal) into a CLOSED, watertight
// triangle shell (tessellate::Mesh). Because the offset fit solves linear systems
// through the numsci facade, the WHOLE file is under CYBERCAD_HAS_NUMSCI (mirroring
// bspline_offset.cpp / bspline_fit.cpp). With the guard OFF this TU is inert and the
// Layer-5 thicken function is absent from the library.
//
// The shell is built from THREE kinds of panel that share EXACT boundary vertices, so
// it is watertight by construction:
//   * the ORIGINAL cap S(u,v) sampled on a (nu × nv) grid,
//   * the OFFSET cap  O(u,v) = S + d·N sampled on the SAME grid (the true offset locus,
//     which the fitted offsetSurface approximates — sampling the locus directly keeps
//     the cap vertices exactly |d| from S along N and exactly matched to the walls),
//   * four RULED SIDE WALLS, each joining a boundary edge of the S cap to the
//     corresponding boundary edge of the O cap, reusing the shared boundary vertices.
// Every seam edge is then used by exactly two triangles. The module VERIFIES closure
// (isWatertight → χ = 2, zero boundary edges; isConsistentlyOriented) and declines a
// non-closed shell rather than returning it.
//
#include "native/math/bspline_thicken.h"

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

// Basic structural validity for a tensor B-spline surface carrier (mirrors the offset
// layer's own well-formedness check).
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

// Map an offset-layer decline onto a thicken decline (same honest reasons).
ThickenStatus fromOffset(OffsetStatus s) {
  switch (s) {
    case OffsetStatus::DegenerateInput:  return ThickenStatus::DegenerateInput;
    case OffsetStatus::DegenerateNormal: return ThickenStatus::DegenerateNormal;
    case OffsetStatus::SelfIntersection: return ThickenStatus::SelfIntersection;
    case OffsetStatus::FitFailed:        return ThickenStatus::OffsetFailed;
    case OffsetStatus::ToleranceNotMet:  return ThickenStatus::OffsetFailed;
    case OffsetStatus::Ok:               return ThickenStatus::Ok;
  }
  return ThickenStatus::OffsetFailed;
}

// (i,j) → flat index into an (nu × nv) grid (i over U rows, j over V columns).
inline std::uint32_t gidx(int i, int j, int nv) {
  return static_cast<std::uint32_t>(i) * static_cast<std::uint32_t>(nv) +
         static_cast<std::uint32_t>(j);
}

// Emit the two CCW triangles of the grid quad whose lower-left corner is (i,j).
// `base` offsets into the mesh's vertex array for this panel's block; `flip` reverses
// the winding (used so the two caps + walls all present OUTWARD faces).
void emitQuad(tess::Mesh& m, std::uint32_t base, int i, int j, int nv, bool flip) {
  const std::uint32_t a = base + gidx(i,     j,     nv);
  const std::uint32_t b = base + gidx(i + 1, j,     nv);
  const std::uint32_t c = base + gidx(i + 1, j + 1, nv);
  const std::uint32_t dd = base + gidx(i,    j + 1, nv);
  if (!flip) {
    m.addTriangle(a, b, c);
    m.addTriangle(a, c, dd);
  } else {
    m.addTriangle(a, c, b);
    m.addTriangle(a, dd, c);
  }
}

// Euler characteristic V − E + F of a triangle mesh (E = #distinct undirected edges).
int eulerChar(const tess::Mesh& m) {
  const auto counts = tess::edgeUseCounts(m);
  const int V = static_cast<int>(m.vertexCount());
  const int E = static_cast<int>(counts.size());
  const int F = static_cast<int>(m.triangleCount());
  return V - E + F;
}

// Make a CLOSED 2-manifold triangle mesh COHERENTLY oriented: BFS across shared edges,
// flipping any neighbour that traverses the shared edge the SAME way as the current
// triangle (a coherent pair traverses it in OPPOSITE directions). Assumes each
// undirected edge is shared by exactly two triangles (watertight); returns false if a
// non-manifold edge (≥ 3 triangles) is met, in which case coherence is undefined and the
// caller declines. Idempotent when the mesh is already coherent. This removes any
// dependence on hand-picked panel windings — the seam-sharing alone guarantees closure,
// and this pass guarantees the single consistent orientation enclosedVolume() needs.
bool orientCoherently(tess::Mesh& m) {
  const std::size_t nT = m.triangles.size();
  if (nT == 0) return false;

  // undirected edge → up to two incident triangle indices.
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

  // Does triangle `t` traverse the directed half-edge (x→y)?
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
        // `t` traverses ed[0]→ed[1]; a COHERENT neighbour traverses ed[1]→ed[0].
        // If the neighbour also goes ed[0]→ed[1] it is inconsistent → flip it.
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

// A rectangular sub-domain of the parameter space to thicken over.
struct Domain { double u0, u1, v0, v1; };

// ── Shared shell assembly over ONE parameter rectangle ───────────────────────────
// Samples S and its offset locus O = S + d·N on a (nu × nv) grid over `dom`, assembles the
// closed six-panel (2 caps + 4 walls) shell, orients it coherently + outward, verifies
// closure, and writes the geometry fields into `r`. `off` is the ALREADY-COMPUTED offset
// result over the same region (its maxError/minCurvatureRadius are reported). This is the
// common core of thickenSurface (full domain) and thickenTrimmed (fold-free sub-rectangle);
// factoring it keeps both paths bit-for-bit identical when the domain is the same.
void assembleShell(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                   const Domain& dom, int nu, int nv, ThickenResult& r) {
  const double u0 = dom.u0, u1 = dom.u1, v0 = dom.v0, v1 = dom.v1;

  std::vector<Point3> sPts(static_cast<std::size_t>(nu) * nv);
  std::vector<Point3> oPts(static_cast<std::size_t>(nu) * nv);
  double midArea2 = 0.0;  // 2× the S-panel area accumulator (for surfaceAreaMid report)
  for (int i = 0; i < nu; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / (nu - 1));
    for (int j = 0; j < nv; ++j) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / (nv - 1));
      const Point3 p = evalS(surface, grid, u, v);
      const Dir3 n = evalN(surface, grid, u, v);
      if (!n.valid()) {  // belt-and-braces: offset guard already covers this
        r.status = ThickenStatus::DegenerateNormal;
        return;
      }
      sPts[static_cast<std::size_t>(i) * nv + j] = p;
      oPts[static_cast<std::size_t>(i) * nv + j] = p + n.vec() * d;
    }
  }

  // ── Assemble the closed shell ────────────────────────────────────────────────────
  // Two grid blocks (S cap then O cap) share NOTHING internally, but the walls reuse
  // both blocks' boundary vertices, so the seam edges close. Vertex layout:
  //   S block: indices [0, nu*nv)         O block: indices [nu*nv, 2*nu*nv)
  tess::Mesh mesh;
  const std::uint32_t sBase = 0;
  for (const Point3& p : sPts) mesh.addVertex(p);
  const std::uint32_t oBase = static_cast<std::uint32_t>(mesh.vertexCount());
  for (const Point3& p : oPts) mesh.addVertex(p);

  // Provisional winding: give the two caps OPPOSITE windings (one is the "inside" of the
  // slab, the other the "outside"); the walls wrap around. Global orientation is fixed
  // after the fact by the signed-volume sign, so the provisional choice only needs to be
  // internally consistent (watertight), not correct-side — which the seam-sharing gives.
  // S cap:
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j)
      emitQuad(mesh, sBase, i, j, nv, /*flip=*/false);
  // O cap (opposite winding so its outward face points the other way):
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j)
      emitQuad(mesh, oBase, i, j, nv, /*flip=*/true);

  // Side walls: for each of the four boundary edges of the grid, a ruled strip between
  // the S-cap boundary vertices and the O-cap boundary vertices. Each strip is a quad
  // band; winding chosen so wall triangles use each seam edge exactly once (matched by
  // the cap on the other side of that edge). Correct global side is fixed by volume sign.
  auto wallQuad = [&](std::uint32_t s0, std::uint32_t s1, std::uint32_t o0,
                      std::uint32_t o1) {
    // Quad (s0 → s1 → o1 → o0) split into two triangles.
    mesh.addTriangle(s0, s1, o1);
    mesh.addTriangle(s0, o1, o0);
  };
  // Edge j = 0 (v = v0), varying i:
  for (int i = 0; i < nu - 1; ++i)
    wallQuad(sBase + gidx(i, 0, nv), sBase + gidx(i + 1, 0, nv),
             oBase + gidx(i, 0, nv), oBase + gidx(i + 1, 0, nv));
  // Edge j = nv-1 (v = v1), varying i:
  for (int i = 0; i < nu - 1; ++i)
    wallQuad(sBase + gidx(i + 1, nv - 1, nv), sBase + gidx(i, nv - 1, nv),
             oBase + gidx(i + 1, nv - 1, nv), oBase + gidx(i, nv - 1, nv));
  // Edge i = 0 (u = u0), varying j:
  for (int j = 0; j < nv - 1; ++j)
    wallQuad(sBase + gidx(0, j + 1, nv), sBase + gidx(0, j, nv),
             oBase + gidx(0, j + 1, nv), oBase + gidx(0, j, nv));
  // Edge i = nu-1 (u = u1), varying j:
  for (int j = 0; j < nv - 1; ++j)
    wallQuad(sBase + gidx(nu - 1, j, nv), sBase + gidx(nu - 1, j + 1, nv),
             oBase + gidx(nu - 1, j, nv), oBase + gidx(nu - 1, j + 1, nv));

  // ── Orientation: coherent winding, then outward (volume sign > 0) ────────────────
  // The seam-sharing makes the shell watertight regardless of per-panel winding, but the
  // panels may not yet agree on a single orientation. Propagate coherence by BFS across
  // shared edges; a non-manifold shell (≥ 3 triangles on an edge) cannot be coherently
  // oriented → decline. Then a single signed-volume sign fixes global inside/out: if
  // negative, reverse every triangle (preserves watertight + coherence, flips to outward).
  if (!orientCoherently(mesh)) {
    r.status = ThickenStatus::NotClosed;  // non-manifold seam — never returned as valid
    return;
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
    r.status = ThickenStatus::NotClosed;  // never return a non-closed solid as valid
    return;
  }

  // S-panel (mid-surface) area for the thin-slab volume oracle.
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j) {
      const Point3& a = sPts[static_cast<std::size_t>(i) * nv + j];
      const Point3& b = sPts[static_cast<std::size_t>(i + 1) * nv + j];
      const Point3& c = sPts[static_cast<std::size_t>(i + 1) * nv + (j + 1)];
      const Point3& e = sPts[static_cast<std::size_t>(i) * nv + (j + 1)];
      midArea2 += norm(cross(b - a, c - a));
      midArea2 += norm(cross(c - a, e - a));
    }
  r.surfaceAreaMid = 0.5 * midArea2;

  r.solid = std::move(mesh);
  r.enclosedVolume = vol;
  r.gridU = nu;
  r.gridV = nv;
  r.ok = true;
  r.status = ThickenStatus::Ok;
}

// ── Warped shell assembly over a fold-locus COLUMN-BAND ──────────────────────────
// The fold-locus analogue of assembleShell: instead of a rectangular (u,v) domain, S and its
// offset O = S + d·N are sampled over the column-band whose per-u fold-free v-interval is the
// polyline (bandU[k], bandVLo[k], bandVHi[k]). The (i,j) sample sits at
//   u = lerp(bandU across s=i/(nu-1)),   v = lerp(vLo,vHi at that u) blended by t=j/(nv-1).
// The two side walls along j=0 / j=nv-1 then trace the fold boundary. The six-panel shell is
// assembled, oriented, and verified EXACTLY as assembleShell (shared boundary vertices ⇒
// watertight by construction). This recovers the triangular corner a rectangle band drops.
void assembleShellBand(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                       const std::vector<double>& bandU, const std::vector<double>& bandVLo,
                       const std::vector<double>& bandVHi, int nu, int nv, ThickenResult& r) {
  const int nc = static_cast<int>(bandU.size());
  if (nc < 2) { r.status = ThickenStatus::DegenerateInput; return; }
  const double u0 = bandU.front(), u1 = bandU.back();
  if (!(u1 > u0)) { r.status = ThickenStatus::DegenerateInput; return; }

  // Piecewise-linear envelope lookup: at parameter u return the (vLo,vHi) interval.
  auto envelope = [&](double u, double& lo, double& hi) {
    if (u <= bandU.front()) { lo = bandVLo.front(); hi = bandVHi.front(); return; }
    if (u >= bandU.back())  { lo = bandVLo.back();  hi = bandVHi.back();  return; }
    int seg = 0;
    while (seg + 1 < nc && bandU[seg + 1] < u) ++seg;
    const double a = (u - bandU[seg]) / (bandU[seg + 1] - bandU[seg]);
    lo = bandVLo[seg] + a * (bandVLo[seg + 1] - bandVLo[seg]);
    hi = bandVHi[seg] + a * (bandVHi[seg + 1] - bandVHi[seg]);
  };

  std::vector<Point3> sPts(static_cast<std::size_t>(nu) * nv);
  std::vector<Point3> oPts(static_cast<std::size_t>(nu) * nv);
  for (int i = 0; i < nu; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / (nu - 1));
    double lo, hi;
    envelope(u, lo, hi);
    for (int j = 0; j < nv; ++j) {
      const double v = lo + (hi - lo) * (static_cast<double>(j) / (nv - 1));
      const Point3 p = evalS(surface, grid, u, v);
      const Dir3 n = evalN(surface, grid, u, v);
      if (!n.valid()) { r.status = ThickenStatus::DegenerateNormal; return; }
      sPts[static_cast<std::size_t>(i) * nv + j] = p;
      oPts[static_cast<std::size_t>(i) * nv + j] = p + n.vec() * d;
    }
  }

  // ── Assemble the closed shell (identical topology to assembleShell) ──
  tess::Mesh mesh;
  const std::uint32_t sBase = 0;
  for (const Point3& p : sPts) mesh.addVertex(p);
  const std::uint32_t oBase = static_cast<std::uint32_t>(mesh.vertexCount());
  for (const Point3& p : oPts) mesh.addVertex(p);

  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j) emitQuad(mesh, sBase, i, j, nv, /*flip=*/false);
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j) emitQuad(mesh, oBase, i, j, nv, /*flip=*/true);

  auto wallQuad = [&](std::uint32_t s0, std::uint32_t s1, std::uint32_t o0, std::uint32_t o1) {
    mesh.addTriangle(s0, s1, o1);
    mesh.addTriangle(s0, o1, o0);
  };
  for (int i = 0; i < nu - 1; ++i)
    wallQuad(sBase + gidx(i, 0, nv), sBase + gidx(i + 1, 0, nv),
             oBase + gidx(i, 0, nv), oBase + gidx(i + 1, 0, nv));
  for (int i = 0; i < nu - 1; ++i)
    wallQuad(sBase + gidx(i + 1, nv - 1, nv), sBase + gidx(i, nv - 1, nv),
             oBase + gidx(i + 1, nv - 1, nv), oBase + gidx(i, nv - 1, nv));
  for (int j = 0; j < nv - 1; ++j)
    wallQuad(sBase + gidx(0, j + 1, nv), sBase + gidx(0, j, nv),
             oBase + gidx(0, j + 1, nv), oBase + gidx(0, j, nv));
  for (int j = 0; j < nv - 1; ++j)
    wallQuad(sBase + gidx(nu - 1, j, nv), sBase + gidx(nu - 1, j + 1, nv),
             oBase + gidx(nu - 1, j, nv), oBase + gidx(nu - 1, j + 1, nv));

  if (!orientCoherently(mesh)) { r.status = ThickenStatus::NotClosed; return; }
  double vol = tess::enclosedVolume(mesh);
  if (vol < 0.0) {
    for (tess::Triangle& t : mesh.triangles) std::swap(t.b, t.c);
    vol = -vol;
  }

  r.watertight = tess::isWatertight(mesh);
  r.consistentlyOriented = tess::isConsistentlyOriented(mesh);
  r.boundaryEdges = tess::boundaryEdgeCount(mesh);
  r.eulerCharacteristic = eulerChar(mesh);
  if (!r.watertight || !r.consistentlyOriented || r.boundaryEdges != 0 ||
      r.eulerCharacteristic != 2) {
    r.status = ThickenStatus::NotClosed;
    return;
  }

  double midArea2 = 0.0;
  for (int i = 0; i < nu - 1; ++i)
    for (int j = 0; j < nv - 1; ++j) {
      const Point3& a = sPts[static_cast<std::size_t>(i) * nv + j];
      const Point3& b = sPts[static_cast<std::size_t>(i + 1) * nv + j];
      const Point3& c = sPts[static_cast<std::size_t>(i + 1) * nv + (j + 1)];
      const Point3& e = sPts[static_cast<std::size_t>(i) * nv + (j + 1)];
      midArea2 += norm(cross(b - a, c - a));
      midArea2 += norm(cross(c - a, e - a));
    }
  r.surfaceAreaMid = 0.5 * midArea2;

  r.solid = std::move(mesh);
  r.enclosedVolume = vol;
  r.gridU = nu;
  r.gridV = nv;
  r.ok = true;
  r.status = ThickenStatus::Ok;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Solid thicken / shell (Layer 5): offset panel + ruled walls → closed shell.
// ─────────────────────────────────────────────────────────────────────────────

ThickenResult thickenSurface(const BsplineSurfaceData& surface, double d, double tol,
                             int gridU, int gridV) {
  ThickenResult r;

  if (!wellFormed(surface)) {
    r.status = ThickenStatus::DegenerateInput;
    return r;
  }
  if (std::fabs(d) <= kLinearTolerance) {
    r.status = ThickenStatus::ZeroThickness;  // no solid to build from a zero offset
    return r;
  }
  const int nu = std::max(2, gridU);
  const int nv = std::max(2, gridV);

  // ── Offset panel via the Layer-5 offset (carries the fold + degenerate guards) ──
  // A thicken past a principal radius of curvature FOLDS its offset panel — the offset
  // layer detects this from the 2nd fundamental form and declines; we propagate that
  // decline honestly (never build a folded solid). We fit at `tol` for the error report;
  // the geometric offset cap itself is sampled from the true locus S + d·N below so the
  // caps and walls share EXACT vertices (watertight by construction).
  const OffsetResult off = offsetSurface(surface, d, tol);
  r.offsetError = off.maxError;
  r.minCurvatureRadius = off.minCurvatureRadius;
  if (!off.ok) {
    r.status = fromOffset(off.status);
    return r;  // decline: degenerate normal / self-intersection / fit failure
  }

  const SurfaceGrid grid{std::span<const Point3>(surface.poles), surface.nPolesU,
                         surface.nPolesV};
  Domain dom;
  dom.u0 = knotLo(surface.knotsU, surface.degreeU);
  dom.u1 = knotHi(surface.knotsU, surface.degreeU);
  dom.v0 = knotLo(surface.knotsV, surface.degreeV);
  dom.v1 = knotHi(surface.knotsV, surface.degreeV);
  if (!(dom.u1 > dom.u0) || !(dom.v1 > dom.v0)) {
    r.status = ThickenStatus::DegenerateInput;
    return r;
  }

  assembleShell(surface, grid, d, dom, nu, nv, r);
  // Full-domain thicken: kept rectangle IS the full domain, never trimmed.
  r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// SELF-INTERSECTION-TRIMMED thicken (additive; thickenSurface byte-unchanged above).
// ─────────────────────────────────────────────────────────────────────────────

ThickenResult thickenTrimmed(const BsplineSurfaceData& surface, double d, double tol,
                             int gridU, int gridV) {
  ThickenResult r;

  if (!wellFormed(surface)) {
    r.status = ThickenStatus::DegenerateInput;
    return r;
  }
  if (std::fabs(d) <= kLinearTolerance) {
    r.status = ThickenStatus::ZeroThickness;
    return r;
  }
  const int nu = std::max(2, gridU);
  const int nv = std::max(2, gridV);

  // ── Fold analysis + fold-free rectangle via the offset layer's trim machinery ──
  // offsetSurfaceTrimmed reuses the offset Jacobian-regularity (1 + d·κᵢ) fold detection:
  //   * fold-free everywhere  → trimmed == false, kept rectangle == full domain,
  //   * partial fold          → trimmed == true, kept == maximal fold-free rectangle,
  //   * degenerate normal     → DegenerateNormal decline,
  //   * whole domain folds     → SelfIntersection decline (no meaningful fold-free region).
  // We build the closed shell over the KEPT rectangle only, so the interpenetrating portion
  // (where the offset would cross back through S) is cut at the self-intersection locus.
  const OffsetResult off = offsetSurfaceTrimmed(surface, d, tol);
  r.offsetError = off.maxError;
  r.minCurvatureRadius = off.minCurvatureRadius;
  if (!off.ok) {
    // Degenerate normal / fully-degenerate fold (whole face interpenetrates) → honest-decline.
    r.status = fromOffset(off.status);
    return r;
  }

  const SurfaceGrid grid{std::span<const Point3>(surface.poles), surface.nPolesU,
                         surface.nPolesV};
  const double fu0 = knotLo(surface.knotsU, surface.degreeU);
  const double fu1 = knotHi(surface.knotsU, surface.degreeU);
  const double fv0 = knotLo(surface.knotsV, surface.degreeV);
  const double fv1 = knotHi(surface.knotsV, surface.degreeV);
  if (!(fu1 > fu0) || !(fv1 > fv0)) {
    r.status = ThickenStatus::DegenerateInput;
    return r;
  }

  if (!off.trimmed) {
    // ── No interpenetration: full-domain thicken. To honour the passthrough oracle
    //    (BYTE-IDENTICAL to thickenSurface when nothing folds) delegate to thickenSurface
    //    verbatim — same offset call, same domain, same assembly, same output. ──
    return thickenSurface(surface, d, tol, gridU, gridV);
  }

  // ── Interpenetration trimmed: build the closed shell over the kept fold-free rectangle ──
  Domain kept{off.keptU0, off.keptU1, off.keptV0, off.keptV1};
  // Clamp defensively to the input domain (the offset layer already returns an interior
  // rectangle; this only guards against a degenerate report).
  kept.u0 = std::max(kept.u0, fu0); kept.u1 = std::min(kept.u1, fu1);
  kept.v0 = std::max(kept.v0, fv0); kept.v1 = std::min(kept.v1, fv1);
  if (!(kept.u1 > kept.u0) || !(kept.v1 > kept.v0)) {
    r.status = ThickenStatus::SelfIntersection;  // no valid region — honest-decline
    return r;
  }

  assembleShell(surface, grid, d, kept, nu, nv, r);
  if (!r.ok) return r;  // assembly-side decline (non-manifold / not-closed) propagates
  r.trimmed = true;
  r.keptU0 = kept.u0; r.keptU1 = kept.u1; r.keptV0 = kept.v0; r.keptV1 = kept.v1;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// MULTI-REGION self-intersection-trimmed thicken (additive; thickenTrimmed byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

std::vector<ThickenResult> thickenMultiTrimmed(const BsplineSurfaceData& surface, double d,
                                               double tol, int gridU, int gridV) {
  std::vector<ThickenResult> out;

  if (!wellFormed(surface)) return out;                  // malformed → honest empty
  if (std::fabs(d) <= kLinearTolerance) return out;      // zero thickness → nothing to build
  const int nu = std::max(2, gridU);
  const int nv = std::max(2, gridV);

  // Reuse the offset layer's MULTI-region fold decomposition: one OffsetResult per meaningful
  // fold-free rectangle (empty ⇔ fully-folding / degenerate → honest-decline). This is the
  // SAME machinery thickenTrimmed uses (offsetSurfaceTrimmed), just returning every region
  // instead of only the largest. We build a closed shell over EACH kept rectangle.
  const std::vector<OffsetResult> regions = offsetSurfaceMultiTrimmed(surface, d, tol);
  if (regions.empty()) return out;  // no valid fold-free region — never emit a folded solid

  const SurfaceGrid grid{std::span<const Point3>(surface.poles), surface.nPolesU,
                         surface.nPolesV};
  const double fu0 = knotLo(surface.knotsU, surface.degreeU);
  const double fu1 = knotHi(surface.knotsU, surface.degreeU);
  const double fv0 = knotLo(surface.knotsV, surface.degreeV);
  const double fv1 = knotHi(surface.knotsV, surface.degreeV);
  if (!(fu1 > fu0) || !(fv1 > fv0)) return out;

  for (const OffsetResult& reg : regions) {
    ThickenResult r;
    r.offsetError = reg.maxError;
    r.minCurvatureRadius = reg.minCurvatureRadius;

    if (!reg.trimmed) {
      // No interpenetration anywhere: a single full-domain solid, byte-identical to
      // thickenSurface (there is exactly one region in this case).
      ThickenResult full = thickenSurface(surface, d, tol, gridU, gridV);
      if (full.ok) out.push_back(std::move(full));
      continue;
    }

    Domain kept{reg.keptU0, reg.keptU1, reg.keptV0, reg.keptV1};
    kept.u0 = std::max(kept.u0, fu0); kept.u1 = std::min(kept.u1, fu1);
    kept.v0 = std::max(kept.v0, fv0); kept.v1 = std::min(kept.v1, fv1);
    if (!(kept.u1 > kept.u0) || !(kept.v1 > kept.v0)) continue;  // degenerate region — skip

    assembleShell(surface, grid, d, kept, nu, nv, r);
    if (!r.ok) continue;  // a region that fails closure is dropped, never returned open
    r.trimmed = true;
    r.keptU0 = kept.u0; r.keptU1 = kept.u1; r.keptV0 = kept.v0; r.keptV1 = kept.v1;
    out.push_back(std::move(r));
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-LOCUS-following thicken (additive; every routine above stays byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

std::vector<ThickenResult> thickenFoldTrim(const BsplineSurfaceData& surface, double d,
                                           double tol, int gridU, int gridV) {
  std::vector<ThickenResult> out;

  if (!wellFormed(surface)) return out;                  // malformed → honest empty
  if (std::fabs(d) <= kLinearTolerance) return out;      // zero thickness → nothing to build
  const int nu = std::max(2, gridU);
  const int nv = std::max(2, gridV);

  // The offset layer's fold-LOCUS decomposition: one region per fold-free component, each with
  // its column-band polyline (foldU / foldVLo / foldVHi) following the diagonal / curved fold.
  // Empty ⇔ fully-folding / degenerate → honest-decline (never emit a self-intersecting solid).
  const std::vector<OffsetResult> regions = offsetSurfaceFoldTrim(surface, d, tol);
  if (regions.empty()) return out;

  const SurfaceGrid grid{std::span<const Point3>(surface.poles), surface.nPolesU,
                         surface.nPolesV};

  for (const OffsetResult& reg : regions) {
    ThickenResult r;
    r.offsetError = reg.maxError;
    r.minCurvatureRadius = reg.minCurvatureRadius;

    if (!reg.foldTrimmed) {
      // No fold anywhere: a single full-domain solid, byte-identical to thickenSurface.
      ThickenResult full = thickenSurface(surface, d, tol, gridU, gridV);
      if (full.ok) out.push_back(std::move(full));
      continue;
    }

    if (reg.foldU.size() < 2) continue;  // degenerate band — skip
    assembleShellBand(surface, grid, d, reg.foldU, reg.foldVLo, reg.foldVHi, nu, nv, r);
    if (!r.ok) continue;  // a region that fails closure is dropped, never returned open
    r.trimmed = true;
    r.keptU0 = reg.keptU0; r.keptU1 = reg.keptU1; r.keptV0 = reg.keptV0; r.keptV1 = reg.keptV1;
    out.push_back(std::move(r));
  }
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

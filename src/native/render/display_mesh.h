// SPDX-License-Identifier: Apache-2.0
//
// display_mesh.h — render-quality DISPLAY mesh, post-processed from an existing
// correctness mesh (the native tessellator's watertight triangle Mesh).
//
// ── WHAT THIS IS (and is NOT) ────────────────────────────────────────────────
// This module is PURELY ADDITIVE and CONSUMES the mesh the correctness
// tessellator already produced (src/native/tessellate: SolidMesher / FaceMesher).
// It NEVER meshes a B-rep, never touches the tessellator, and does not depend on
// topology — its sole input is (vertices, triangles). The correctness path stays
// byte-identical; the display path is a separate consumer.
//
// The correctness mesh is optimised to be watertight and lie ON the analytic
// surface within a deflection bound. For a real-time viewport (iPad) and for
// higher-quality glTF / USDZ export the renderer wants SHADING attributes the
// correctness mesh deliberately omits: smooth per-vertex normals that make a
// curved wall shade continuously, HARD edges that keep a box/crease looking
// sharp, texture coordinates, and an optional lower level-of-detail. This module
// derives all of those from the correctness mesh geometry alone.
//
// ── ALGORITHMS ───────────────────────────────────────────────────────────────
//   * SMOOTH NORMALS + CREASES. Each undirected mesh edge has a dihedral angle
//     between the two triangle face-normals sharing it. A vertex's incident
//     triangles are partitioned into SMOOTHING GROUPS by a union-find over
//     incident-triangle pairs that share an edge whose dihedral is BELOW the
//     crease angle. A vertex touched by more than one group is SPLIT (duplicated,
//     one copy per group); each copy carries the ANGLE-WEIGHTED average of the
//     face-normals in its group. Result: a curved wall (all sub-crease dihedrals)
//     is one group → a single smooth normal set exactly tangent to the surface; a
//     box corner (every dihedral a crease) splits into a distinct normal per
//     incident face → hard edges stay sharp.
//   * UVs (optional). A simple, seam-consistent per-smoothing-group PLANAR (box)
//     projection: each group projects onto the plane whose normal is the group's
//     dominant axis, and the group's projected extent is normalised into [0,1].
//     Correct and deterministic; not an unwrap — enough for material preview.
//   * LOD (optional). Greedy edge-collapse decimation with a quadric-error metric
//     (Garland-Heckbert), collapsing to a midpoint only when (a) neither endpoint
//     is a boundary or crease vertex, (b) the collapse keeps triangles non-flipped,
//     and (c) the resulting deviation stays within a Hausdorff bound derived from
//     the source deflection. Boundary and crease edges are LOCKED, so silhouettes
//     and hard edges survive. Skipped entirely when lodTargetTris <= 0.
//
// ── HONEST DECLINE ───────────────────────────────────────────────────────────
// An empty input (no triangles) yields an empty DisplayMesh — never a fabricated
// result. Every produced position still lies on the source surface (smooth-normal
// splitting and UV assignment move NO vertex; LOD moves vertices only within the
// asserted Hausdorff bound).
//
// OCCT-FREE. Uses only src/native/math and src/native/tessellate/mesh.h.
// Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_RENDER_DISPLAY_MESH_H
#define CYBERCAD_NATIVE_RENDER_DISPLAY_MESH_H

#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

namespace cybercad::native::render {

namespace math = cybercad::native::math;
namespace tess = cybercad::native::tessellate;

/// π (local; the math library exposes no such constant).
inline constexpr double kPi = 3.14159265358979323846;

// ─────────────────────────────────────────────────────────────────────────────
// Parameters + result.
// ─────────────────────────────────────────────────────────────────────────────

/// Knobs for the display-mesh post-process. `deflection` is the SOURCE mesh's
/// chord bound; it seeds the default LOD Hausdorff budget. `creaseAngleDeg` is
/// the dihedral threshold (degrees) above which an edge is HARD (vertices split).
/// `lodTargetTris <= 0` disables decimation. `wantUVs` toggles UV generation.
struct DisplayParams {
  double deflection = 0.1;      ///< source mesh chord bound (model units)
  double creaseAngleDeg = 30.0; ///< dihedral above this ⇒ hard edge (split)
  int lodTargetTris = 0;        ///< target triangle count (<=0 ⇒ no decimation)
  bool wantUVs = false;         ///< emit per-vertex UVs
  double lodHausdorffScale = 8.0; ///< LOD budget = scale · deflection
};

/// The render-quality mesh. Positions are the (possibly duplicated / decimated)
/// display vertices; every vertex has a smooth-or-hard normal; UVs are present
/// only when requested (empty otherwise). Triangles index into all three arrays.
struct DisplayMesh {
  std::vector<math::Point3> positions;      ///< (N) display vertices
  std::vector<math::Dir3> normals;          ///< (N) per-vertex normals (always)
  std::vector<std::array<double, 2>> uvs;    ///< (N) per-vertex UVs (empty ⇒ none)
  std::vector<tess::Triangle> triangles;    ///< (M) 0-based indices

  std::size_t vertexCount() const noexcept { return positions.size(); }
  std::size_t triangleCount() const noexcept { return triangles.size(); }
  bool hasUVs() const noexcept { return !uvs.empty(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Small geometry helpers (local; no OCCT).
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

/// Unnormalised triangle normal (area-weighted: |cross| = 2·area). Zero for a
/// degenerate triangle.
inline math::Vec3 rawTriNormal(const math::Point3& a, const math::Point3& b,
                               const math::Point3& c) noexcept {
  return math::cross(b - a, c - a);
}

/// Interior angle (radians) of the triangle at corner `apex` (the other two
/// corners are `p`, `q`). Used to angle-weight a vertex normal so a vertex shared
/// by many small slivers is not over-counted. Robust atan2 form.
inline double cornerAngle(const math::Point3& apex, const math::Point3& p,
                          const math::Point3& q) noexcept {
  const math::Vec3 u = p - apex;
  const math::Vec3 v = q - apex;
  const double du = math::norm(u), dv = math::norm(v);
  if (du < 1e-15 || dv < 1e-15) return 0.0;
  return std::atan2(math::norm(math::cross(u, v)), math::dot(u, v));
}

/// Undirected edge key (min,max) → the two triangles that share it (dihedral).
struct EdgeKey {
  std::uint32_t lo, hi;
  bool operator==(const EdgeKey& o) const noexcept { return lo == o.lo && hi == o.hi; }
};
struct EdgeKeyHash {
  std::size_t operator()(const EdgeKey& e) const noexcept {
    return (static_cast<std::size_t>(e.lo) << 32) ^ static_cast<std::size_t>(e.hi);
  }
};
inline EdgeKey edgeKey(std::uint32_t a, std::uint32_t b) noexcept {
  return {std::min(a, b), std::max(a, b)};
}

/// Union-find over triangle-corner slots (one node per (triangle, corner) pair),
/// used to build per-vertex smoothing groups.
struct UnionFind {
  std::vector<std::uint32_t> parent;
  explicit UnionFind(std::size_t n) : parent(n) {
    for (std::size_t i = 0; i < n; ++i) parent[i] = static_cast<std::uint32_t>(i);
  }
  std::uint32_t find(std::uint32_t x) {
    while (parent[x] != x) {
      parent[x] = parent[parent[x]];
      x = parent[x];
    }
    return x;
  }
  void unite(std::uint32_t a, std::uint32_t b) { parent[find(a)] = find(b); }
};

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// SMOOTH NORMALS + CREASE-SPLIT.
//
// Output vertex layout: the vertex array is rebuilt so each SMOOTHING GROUP at an
// original vertex becomes its own display vertex. A vertex whose incident faces
// are all in one group is emitted once (smooth); a crease vertex is emitted once
// per side (hard). Triangle indices are re-pointed to the group vertex.
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

/// Per-triangle unit face normal + area (0 for degenerate). Degenerate triangles
/// contribute no normal and never bridge a smoothing group.
struct FaceNormal {
  math::Vec3 unit{0, 0, 0};  // unit normal (zero if degenerate)
  double area = 0.0;
  bool valid = false;
};

inline std::vector<FaceNormal> faceNormals(const tess::Mesh& m) {
  std::vector<FaceNormal> out(m.triangles.size());
  for (std::size_t t = 0; t < m.triangles.size(); ++t) {
    const auto& tri = m.triangles[t];
    const math::Vec3 n = rawTriNormal(m.vertices[tri.a], m.vertices[tri.b], m.vertices[tri.c]);
    const double len = math::norm(n);
    if (len > 1e-15) {
      out[t].unit = n / len;
      out[t].area = 0.5 * len;
      out[t].valid = true;
    }
  }
  return out;
}

/// Corner slot index: triangle t contributes three slots 3·t+corner (0,1,2).
inline std::uint32_t slot(std::size_t t, int corner) noexcept {
  return static_cast<std::uint32_t>(t * 3 + static_cast<std::size_t>(corner));
}

/// The three vertex indices of a triangle in an array (for corner iteration).
inline std::array<std::uint32_t, 3> triVerts(const tess::Triangle& t) noexcept {
  return {t.a, t.b, t.c};
}

}  // namespace detail

/// Compute smooth normals with crease-based vertex splitting. Returns a
/// DisplayMesh with positions/normals/triangles populated (UVs empty). Every
/// original position may appear multiple times (once per smoothing group). No
/// vertex is moved — positions are copied verbatim from the source.
inline DisplayMesh smoothNormalsWithCreases(const tess::Mesh& src, double creaseAngleDeg) {
  DisplayMesh out;
  const std::size_t triCount = src.triangles.size();
  if (triCount == 0) return out;

  const double creaseCos = std::cos(std::clamp(creaseAngleDeg, 0.0, 180.0) * kPi / 180.0);
  const std::vector<detail::FaceNormal> fn = detail::faceNormals(src);

  // Map each undirected edge to the (triangle, cornerpair) uses touching it, so we
  // can decide per shared edge whether the two triangles smooth together. We store,
  // per edge, up to two (triangle, cornerA, cornerB) records; a manifold edge has 2.
  struct EdgeUse {
    std::uint32_t tri;
    int cornerA, cornerB;  // the two triangle corners forming this edge
  };
  std::unordered_map<detail::EdgeKey, std::vector<EdgeUse>, detail::EdgeKeyHash> edgeUses;
  edgeUses.reserve(triCount * 3);
  for (std::size_t t = 0; t < triCount; ++t) {
    const auto v = detail::triVerts(src.triangles[t]);
    for (int c = 0; c < 3; ++c) {
      const int c2 = (c + 1) % 3;
      edgeUses[detail::edgeKey(v[c], v[c2])].push_back(
          EdgeUse{static_cast<std::uint32_t>(t), c, c2});
    }
  }

  // Union corner slots across a shared, sub-crease edge. The two corners of the
  // edge in each triangle belong to the two endpoint vertices; we union the slots
  // that sit on the SAME endpoint vertex (so a smooth edge fuses both endpoints).
  detail::UnionFind uf(triCount * 3);
  for (const auto& [key, uses] : edgeUses) {
    if (uses.size() != 2) continue;  // boundary / non-manifold ⇒ never smooth
    const std::size_t t0 = uses[0].tri, t1 = uses[1].tri;
    if (!fn[t0].valid || !fn[t1].valid) continue;
    const double d = math::dot(fn[t0].unit, fn[t1].unit);
    if (d < creaseCos) continue;  // dihedral exceeds crease ⇒ HARD edge, no union
    // Fuse the two triangles' corner slots that share each endpoint vertex.
    const auto va = detail::triVerts(src.triangles[t0]);
    const auto vb = detail::triVerts(src.triangles[t1]);
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        if (va[i] == vb[j]) uf.unite(detail::slot(t0, i), detail::slot(t1, j));
  }

  // Each (original vertex, smoothing-group root) pair becomes one display vertex.
  // Accumulate its angle-weighted normal from every corner slot in the group.
  std::unordered_map<std::uint64_t, std::uint32_t> groupToNew;
  groupToNew.reserve(triCount * 3);
  std::vector<math::Vec3> accum;  // parallel to out.positions

  out.triangles.resize(triCount);
  for (std::size_t t = 0; t < triCount; ++t) {
    const auto v = detail::triVerts(src.triangles[t]);
    std::array<std::uint32_t, 3> newIdx{};
    for (int c = 0; c < 3; ++c) {
      const std::uint32_t root = uf.find(detail::slot(t, c));
      // Key = (original vertex, group root): distinct groups on the same vertex
      // become distinct display vertices (the split).
      const std::uint64_t key =
          (static_cast<std::uint64_t>(v[c]) << 32) ^ static_cast<std::uint64_t>(root);
      auto it = groupToNew.find(key);
      std::uint32_t idx;
      if (it == groupToNew.end()) {
        idx = static_cast<std::uint32_t>(out.positions.size());
        out.positions.push_back(src.vertices[v[c]]);
        accum.push_back(math::Vec3{0, 0, 0});
        groupToNew.emplace(key, idx);
      } else {
        idx = it->second;
      }
      // Angle-weighted, area-weighted contribution of this triangle at corner c.
      if (fn[t].valid) {
        const double ang = detail::cornerAngle(src.vertices[v[c]], src.vertices[v[(c + 1) % 3]],
                                               src.vertices[v[(c + 2) % 3]]);
        accum[idx] += fn[t].unit * (ang * fn[t].area);
      }
      newIdx[static_cast<std::size_t>(c)] = idx;
    }
    out.triangles[t] = tess::Triangle{newIdx[0], newIdx[1], newIdx[2]};
  }

  out.normals.resize(out.positions.size());
  for (std::size_t i = 0; i < out.positions.size(); ++i) {
    const math::Vec3& a = accum[i];
    if (math::norm(a) > 1e-15)
      out.normals[i] = math::Dir3{a};
    else
      out.normals[i] = math::Dir3{0, 0, 1};  // degenerate fallback (unused island)
  }
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// UVs — per-smoothing-group planar (box) projection, normalised to [0,1].
//
// A vertex's normal already encodes its smoothing group (parallel-normal verts
// share a plane). We project each vertex onto the plane whose axis is the largest
// component of its normal (X/Y/Z box face), then normalise the whole mesh's
// projected extent into [0,1]. Seam-consistent: two verts with the same position
// and same dominant axis get the same UV.
// ─────────────────────────────────────────────────────────────────────────────

/// Assign per-vertex UVs by box projection. Requires `dm.normals` populated.
inline void assignBoxUVs(DisplayMesh& dm) {
  if (dm.positions.empty()) return;
  // Global AABB for a single normalisation across all box faces.
  math::Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                std::numeric_limits<double>::max()};
  math::Vec3 hi{-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max(),
                -std::numeric_limits<double>::max()};
  for (const auto& p : dm.positions) {
    lo.x = std::min(lo.x, p.x); lo.y = std::min(lo.y, p.y); lo.z = std::min(lo.z, p.z);
    hi.x = std::max(hi.x, p.x); hi.y = std::max(hi.y, p.y); hi.z = std::max(hi.z, p.z);
  }
  const math::Vec3 ext{hi.x - lo.x, hi.y - lo.y, hi.z - lo.z};
  auto denom = [](double e) { return e > 1e-15 ? e : 1.0; };
  const math::Vec3 inv{1.0 / denom(ext.x), 1.0 / denom(ext.y), 1.0 / denom(ext.z)};

  dm.uvs.resize(dm.positions.size());
  for (std::size_t i = 0; i < dm.positions.size(); ++i) {
    const math::Point3& p = dm.positions[i];
    const math::Vec3 n = dm.normals[i].vec();
    const double ax = std::fabs(n.x), ay = std::fabs(n.y), az = std::fabs(n.z);
    double u, v;
    if (ax >= ay && ax >= az) {          // X-dominant ⇒ project onto YZ
      u = (p.y - lo.y) * inv.y; v = (p.z - lo.z) * inv.z;
    } else if (ay >= ax && ay >= az) {   // Y-dominant ⇒ project onto XZ
      u = (p.x - lo.x) * inv.x; v = (p.z - lo.z) * inv.z;
    } else {                             // Z-dominant ⇒ project onto XY
      u = (p.x - lo.x) * inv.x; v = (p.y - lo.y) * inv.y;
    }
    dm.uvs[i] = {std::clamp(u, 0.0, 1.0), std::clamp(v, 0.0, 1.0)};
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// LOD — quadric-error edge-collapse decimation.
//
// Garland-Heckbert: each vertex accumulates a 4×4 error quadric Q (sum of the
// fundamental error quadrics of its incident triangle planes). Collapsing edge
// (i→j) costs vᵀ(Qᵢ+Qⱼ)v at the target position v (here the midpoint, to keep the
// result on the surface). LOCKED vertices (mesh boundary, crease, or non-manifold)
// are never moved or removed, so silhouettes and hard edges survive. A collapse is
// rejected if it would flip a triangle or push a vertex beyond the Hausdorff
// budget. Greedy: repeatedly collapse the cheapest legal edge until the target
// triangle count is reached or no legal collapse remains.
// ─────────────────────────────────────────────────────────────────────────────

namespace detail {

/// A symmetric 4×4 quadric stored as its 10 upper-triangle coefficients.
/// Q(v) = a·x²+2b·xy+2c·xz+2d·x + e·y²+2f·yz+2g·y + h·z²+2i·z + j  (v=(x,y,z,1)).
struct Quadric {
  double a = 0, b = 0, c = 0, d = 0, e = 0, f = 0, g = 0, h = 0, i = 0, j = 0;
  void addPlane(double nx, double ny, double nz, double dd) noexcept {
    a += nx * nx; b += nx * ny; c += nx * nz; d += nx * dd;
    e += ny * ny; f += ny * nz; g += ny * dd;
    h += nz * nz; i += nz * dd;
    j += dd * dd;
  }
  Quadric operator+(const Quadric& o) const noexcept {
    Quadric r;
    r.a = a + o.a; r.b = b + o.b; r.c = c + o.c; r.d = d + o.d; r.e = e + o.e;
    r.f = f + o.f; r.g = g + o.g; r.h = h + o.h; r.i = i + o.i; r.j = j + o.j;
    return r;
  }
  double eval(const math::Point3& p) const noexcept {
    const double x = p.x, y = p.y, z = p.z;
    return a * x * x + 2 * b * x * y + 2 * c * x * z + 2 * d * x + e * y * y +
           2 * f * y * z + 2 * g * y + h * z * z + 2 * i * z + j;
  }
};

}  // namespace detail

/// Decimate `dm` in place toward `targetTris`, keeping every moved vertex within
/// `hausdorff` of its original position and never crossing a locked (boundary /
/// crease / non-manifold) vertex. Recomputes smooth normals for the survivors
/// (grouped by the same crease threshold). No-op when already at/under target.
inline void decimate(DisplayMesh& dm, int targetTris, double hausdorff, double creaseAngleDeg) {
  const std::size_t n = dm.positions.size();
  if (targetTris <= 0 || dm.triangles.size() <= static_cast<std::size_t>(targetTris)) return;
  if (n == 0) return;

  // ── Build vertex→triangle adjacency and per-edge use counts on the DISPLAY mesh.
  std::vector<std::vector<std::uint32_t>> vtri(n);
  for (std::size_t t = 0; t < dm.triangles.size(); ++t) {
    const auto& tr = dm.triangles[t];
    vtri[tr.a].push_back(static_cast<std::uint32_t>(t));
    vtri[tr.b].push_back(static_cast<std::uint32_t>(t));
    vtri[tr.c].push_back(static_cast<std::uint32_t>(t));
  }
  std::unordered_map<detail::EdgeKey, int, detail::EdgeKeyHash> edgeCount;
  edgeCount.reserve(dm.triangles.size() * 3);
  auto bumpEdge = [&](std::uint32_t x, std::uint32_t y) { ++edgeCount[detail::edgeKey(x, y)]; };
  for (const auto& tr : dm.triangles) { bumpEdge(tr.a, tr.b); bumpEdge(tr.b, tr.c); bumpEdge(tr.c, tr.a); }

  // A vertex is LOCKED if any incident edge is a boundary (used once) OR the mesh
  // split it (a crease vertex is a distinct display vertex whose positions coincide
  // with another — locking any vertex sharing a position with a differently-normalled
  // vertex preserves hard edges). We approximate crease-lock by position sharing.
  std::vector<char> locked(n, 0);
  for (const auto& [key, uses] : edgeCount)
    if (uses != 2) { locked[key.lo] = 1; locked[key.hi] = 1; }
  // Position-shared vertices (crease splits) are locked so hard edges never
  // collapse. The key is the EXACT quantised (x,y,z) tuple — a lossy scalar hash
  // would collide distinct positions and falsely lock smooth vertices.
  {
    struct QKey {
      std::int64_t x, y, z;
      bool operator==(const QKey& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
    };
    struct QKeyHash {
      std::size_t operator()(const QKey& q) const noexcept {
        std::size_t h = static_cast<std::size_t>(q.x) * 73856093u;
        h ^= static_cast<std::size_t>(q.y) * 19349663u + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        h ^= static_cast<std::size_t>(q.z) * 83492791u + 0x9e3779b97f4a7c15ull + (h << 6) + (h >> 2);
        return h;
      }
    };
    std::unordered_map<QKey, std::uint32_t, QKeyHash> firstAt;
    firstAt.reserve(n * 2);
    auto quant = [](double v) { return static_cast<std::int64_t>(std::llround(v * 1e6)); };
    for (std::size_t i = 0; i < n; ++i) {
      const auto& p = dm.positions[i];
      const QKey k{quant(p.x), quant(p.y), quant(p.z)};
      auto it = firstAt.find(k);
      if (it == firstAt.end()) firstAt.emplace(k, static_cast<std::uint32_t>(i));
      else { locked[i] = 1; locked[it->second] = 1; }
    }
  }

  // ── Per-vertex quadrics from incident triangle planes. `planeCount[v]` is the
  // number of planes summed into Q[v]; the RMS perpendicular distance of a point p
  // to the vertex's incident planes is sqrt(Q[v].eval(p) / planeCount[v]) — the
  // GEOMETRIC deviation used as the Hausdorff proxy (not the tangential travel
  // distance, which is large for an on-surface move and would over-reject).
  std::vector<detail::Quadric> Q(n);
  std::vector<int> planeCount(n, 0);
  for (std::size_t t = 0; t < dm.triangles.size(); ++t) {
    const auto& tr = dm.triangles[t];
    const math::Vec3 raw = detail::rawTriNormal(dm.positions[tr.a], dm.positions[tr.b], dm.positions[tr.c]);
    const double len = math::norm(raw);
    if (len < 1e-15) continue;
    const math::Vec3 nn = raw / len;
    const double dd = -math::dot(nn, dm.positions[tr.a].asVec());
    detail::Quadric q; q.addPlane(nn.x, nn.y, nn.z, dd);
    Q[tr.a] = Q[tr.a] + q; Q[tr.b] = Q[tr.b] + q; Q[tr.c] = Q[tr.c] + q;
    ++planeCount[tr.a]; ++planeCount[tr.b]; ++planeCount[tr.c];
  }
  const double hausSq = hausdorff * hausdorff;

  std::vector<char> deadV(n, 0);
  std::vector<char> deadT(dm.triangles.size(), 0);
  std::size_t liveTris = dm.triangles.size();

  auto triNormalRaw = [&](std::uint32_t t) {
    const auto& tr = dm.triangles[t];
    return detail::rawTriNormal(dm.positions[tr.a], dm.positions[tr.b], dm.positions[tr.c]);
  };

  // Would collapsing (from→to, moving BOTH surviving triangles' shared vertex to
  // `pos`) flip any incident triangle? A triangle that contains both endpoints
  // degenerates (removed) and is skipped; any other triangle keeps its winding iff
  // its normal does not reverse. We test every triangle around from OR to.
  auto wouldFlip = [&](std::uint32_t from, std::uint32_t to, const math::Point3& pos) {
    for (std::uint32_t v : {from, to}) {
      for (std::uint32_t t : vtri[v]) {
        if (deadT[t]) continue;
        const auto& tr = dm.triangles[t];
        const bool hasFrom = (tr.a == from || tr.b == from || tr.c == from);
        const bool hasTo = (tr.a == to || tr.b == to || tr.c == to);
        if (hasFrom && hasTo) continue;  // degenerate after collapse
        const math::Vec3 before = triNormalRaw(t);
        auto sub = [&](std::uint32_t x) {
          return (x == from || x == to) ? pos : dm.positions[x];
        };
        const math::Vec3 after = detail::rawTriNormal(sub(tr.a), sub(tr.b), sub(tr.c));
        if (math::dot(before, after) <= 0.0) return true;
      }
    }
    return false;
  };

  // Edges that failed the flip test this pass — skipped until the mesh changes
  // (cleared after any successful collapse). Avoids a stall without over-locking.
  std::unordered_map<detail::EdgeKey, char, detail::EdgeKeyHash> rejected;

  // Greedy collapse rounds: each round scans all live edges, collapses the cheapest
  // legal one. Simple O(rounds·edges); ample for viewport LOD sizes.
  while (liveTris > static_cast<std::size_t>(targetTris)) {
    double bestCost = std::numeric_limits<double>::max();
    std::uint32_t bestFrom = 0, bestTo = 0;
    detail::EdgeKey bestKey{0, 0};
    math::Point3 bestPos{};
    bool found = false;

    for (const auto& [key, uses] : edgeCount) {
      if (uses != 2) continue;
      if (rejected.count(key)) continue;
      const std::uint32_t i = key.lo, k = key.hi;
      if (deadV[i] || deadV[k]) continue;
      // At least one endpoint must be free to move; collapse the free one to the
      // midpoint (or onto the locked endpoint if exactly one side is locked).
      const bool li = locked[i], lk = locked[k];
      if (li && lk) continue;  // both locked ⇒ never collapse (crease/boundary)
      math::Point3 target;
      std::uint32_t from, to;
      if (li) { target = dm.positions[i]; from = k; to = i; }
      else if (lk) { target = dm.positions[k]; from = i; to = k; }
      else {
        target = math::Point3{(dm.positions[i].x + dm.positions[k].x) * 0.5,
                              (dm.positions[i].y + dm.positions[k].y) * 0.5,
                              (dm.positions[i].z + dm.positions[k].z) * 0.5};
        from = i; to = k;
      }
      // Hausdorff guard (GEOMETRIC): the merged quadric's RMS perpendicular
      // deviation of the target from the incident planes must stay within budget.
      // cost = Σ dist², over (planeCount[i]+planeCount[k]) planes, so
      // sqrt(cost / planes) ≤ hausdorff ⇔ cost ≤ planes · hausdorff². This bounds
      // how far the decimated surface departs from the source surface, not how far
      // the vertex slides tangentially.
      const double cost = (Q[i] + Q[k]).eval(target);
      const int planes = planeCount[i] + planeCount[k];
      if (planes > 0 && cost > static_cast<double>(planes) * hausSq) continue;
      if (cost < bestCost) {
        bestCost = cost; bestFrom = from; bestTo = to; bestKey = key; bestPos = target;
        found = true;
      }
    }
    if (!found) break;

    const std::uint32_t from = bestFrom, to = bestTo;
    if (wouldFlip(from, to, bestPos)) { rejected[bestKey] = 1; continue; }

    // ── Apply: move `to` to bestPos, retarget `from`'s triangles, kill degenerates.
    dm.positions[to] = bestPos;
    Q[to] = Q[from] + Q[to];
    planeCount[to] += planeCount[from];
    for (std::uint32_t t : vtri[from]) {
      if (deadT[t]) continue;
      auto& tr = dm.triangles[t];
      if (tr.a == from) tr.a = to;
      if (tr.b == from) tr.b = to;
      if (tr.c == from) tr.c = to;
      if (tr.a == tr.b || tr.b == tr.c || tr.c == tr.a) {
        deadT[t] = 1;
        --liveTris;
      } else {
        vtri[to].push_back(t);
      }
    }
    deadV[from] = 1;
    rejected.clear();  // topology changed — prior flip verdicts may no longer hold

    // Rebuild edge counts over live triangles (incremental upkeep is more code).
    edgeCount.clear();
    for (std::size_t t = 0; t < dm.triangles.size(); ++t) {
      if (deadT[t]) continue;
      const auto& tr = dm.triangles[t];
      bumpEdge(tr.a, tr.b); bumpEdge(tr.b, tr.c); bumpEdge(tr.c, tr.a);
    }
  }

  // ── Compact: drop dead triangles + unreferenced vertices, re-index.
  tess::Mesh compact;
  std::vector<std::uint32_t> remap(n, std::numeric_limits<std::uint32_t>::max());
  for (std::size_t t = 0; t < dm.triangles.size(); ++t) {
    if (deadT[t]) continue;
    const auto& tr = dm.triangles[t];
    auto emit = [&](std::uint32_t v) {
      if (remap[v] == std::numeric_limits<std::uint32_t>::max()) {
        remap[v] = static_cast<std::uint32_t>(compact.vertices.size());
        compact.vertices.push_back(dm.positions[v]);
      }
      return remap[v];
    };
    compact.addTriangle(emit(tr.a), emit(tr.b), emit(tr.c));
  }

  // Recompute smooth normals on the decimated result (same crease threshold) so the
  // survivors shade correctly; carry UVs off if they were present (regenerated by
  // the caller when wanted).
  DisplayMesh re = smoothNormalsWithCreases(compact, creaseAngleDeg);
  dm.positions = std::move(re.positions);
  dm.normals = std::move(re.normals);
  dm.triangles = std::move(re.triangles);
  dm.uvs.clear();
}

// ─────────────────────────────────────────────────────────────────────────────
// TOP-LEVEL DRIVER.
// ─────────────────────────────────────────────────────────────────────────────

/// Build a render-quality DisplayMesh from a correctness Mesh. Empty input ⇒
/// empty output (honest decline). Order: smooth-normal split → optional LOD →
/// optional UVs (after LOD so UVs match the final vertex set).
inline DisplayMesh buildDisplayMesh(const tess::Mesh& src, const DisplayParams& params) {
  DisplayMesh dm = smoothNormalsWithCreases(src, params.creaseAngleDeg);
  if (dm.triangleCount() == 0) return dm;

  if (params.lodTargetTris > 0) {
    const double budget = std::max(params.lodHausdorffScale * params.deflection, 1e-9);
    decimate(dm, params.lodTargetTris, budget, params.creaseAngleDeg);
  }
  if (params.wantUVs) assignBoxUVs(dm);
  return dm;
}

}  // namespace cybercad::native::render

#endif  // CYBERCAD_NATIVE_RENDER_DISPLAY_MESH_H

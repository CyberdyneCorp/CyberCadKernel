// SPDX-License-Identifier: Apache-2.0
//
// mesh.h — the triangle Mesh produced by the native tessellator, plus the
// tolerance-based property checks the verification model relies on.
//
// Tessellation is an APPROXIMATION of a B-rep (Phase 4, capability #3
// `native-tessellation`, see openspec/NATIVE-REWRITE.md). We therefore never
// compare a mesh triangle-for-triangle against OCCT; instead the verification
// checks TOLERANCE-BASED PROPERTIES of the mesh, and this header supplies the
// primitives those checks are built on:
//
//   * surfaceArea()     — Σ ½|(b−a)×(c−a)| over triangles. Converges to the true
//                         B-rep face area as the deflection bound → 0.
//   * enclosedVolume()  — signed-tetra sum ⅙ Σ aᵢ·(bᵢ×cᵢ) (divergence theorem).
//                         For a watertight, outward-oriented closed mesh this is
//                         the enclosed volume; converges to the solid's volume.
//   * isWatertight()    — every undirected edge is shared by exactly two
//                         triangles (closed 2-manifold, no boundary/no T-junction).
//   * isTwoManifold()   — every undirected edge is used by ≤ 2 triangles (a mesh
//                         may be an open manifold: boundary edges used once).
//
// Vertices are fp64 (the source of truth). Triangles index into the vertex
// array (CCW as seen from outside ⇒ outward normal for a closed, consistently
// oriented mesh). Per-vertex normals are optional (empty ⇒ absent).
//
// OCCT-FREE. Uses only src/native/math. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_MESH_H
#define CYBERCAD_NATIVE_TESSELLATE_MESH_H

#include "native/math/native_math.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace cybercad::native::tessellate {

namespace math = cybercad::native::math;

/// One triangle as three 0-based indices into Mesh::vertices, wound CCW when
/// viewed from the outside of the surface (front face ⇒ outward normal).
struct Triangle {
  std::uint32_t a = 0;
  std::uint32_t b = 0;
  std::uint32_t c = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// Mesh — fp64 vertices (N×3), integer triangles (M×3), optional per-vertex
// normals. The container is deliberately dumb (plain arrays); all geometric
// meaning lives in the free-function property checks below so a mesh assembled
// by any mesher (face, solid) shares one verification vocabulary.
// ─────────────────────────────────────────────────────────────────────────────
struct Mesh {
  std::vector<math::Point3> vertices;  ///< (N,3) positions, fp64
  std::vector<Triangle> triangles;     ///< (M,3) 0-based indices
  std::vector<math::Dir3> normals;     ///< optional (N) per-vertex; empty ⇒ none

  std::size_t vertexCount() const noexcept { return vertices.size(); }
  std::size_t triangleCount() const noexcept { return triangles.size(); }
  bool hasNormals() const noexcept { return !normals.empty(); }

  /// Append a vertex, return its 0-based index.
  std::uint32_t addVertex(const math::Point3& p) {
    vertices.push_back(p);
    return static_cast<std::uint32_t>(vertices.size() - 1);
  }

  /// Append a triangle (indices must already be valid).
  void addTriangle(std::uint32_t a, std::uint32_t b, std::uint32_t c) {
    triangles.push_back(Triangle{a, b, c});
  }

  /// Concatenate `other` into this mesh, offsetting its triangle indices. Used
  /// by the solid mesher to accumulate per-face meshes before welding. Normals
  /// are carried through only if BOTH meshes have them (otherwise dropped, since
  /// a partial normal array is meaningless).
  void append(const Mesh& other) {
    const auto base = static_cast<std::uint32_t>(vertices.size());
    const bool keepNormals = hasNormals() && other.hasNormals();
    vertices.insert(vertices.end(), other.vertices.begin(), other.vertices.end());
    if (keepNormals)
      normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    else
      normals.clear();
    triangles.reserve(triangles.size() + other.triangles.size());
    for (const Triangle& t : other.triangles)
      triangles.push_back(Triangle{t.a + base, t.b + base, t.c + base});
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Geometric properties (tolerance-based verification primitives).
// ─────────────────────────────────────────────────────────────────────────────

/// Total surface area = Σ ½‖(b−a)×(c−a)‖. Independent of winding (uses the
/// cross-product magnitude). Converges to the true face/solid area as deflection
/// → 0 (a triangulation always UNDER-estimates a convex-ish smooth patch, but
/// the gap is O(deflection); the verification asserts a relative bound).
inline double surfaceArea(const Mesh& m) noexcept {
  double area = 0.0;
  for (const Triangle& t : m.triangles) {
    const math::Vec3 ab = m.vertices[t.b] - m.vertices[t.a];
    const math::Vec3 ac = m.vertices[t.c] - m.vertices[t.a];
    area += 0.5 * math::norm(math::cross(ab, ac));
  }
  return area;
}

/// Signed enclosed volume via the divergence theorem: V = ⅙ Σ aᵢ·(bᵢ×cᵢ), the
/// sum of signed tetra volumes from the origin to each triangle. For a closed,
/// outward-CCW-wound (watertight) mesh this is the enclosed volume (sign +). A
/// non-closed mesh gives a meaningless value — always pair with isWatertight().
/// Origin-independence holds exactly for a closed surface (the open part of the
/// sum telescopes to zero).
inline double enclosedVolume(const Mesh& m) noexcept {
  double vol6 = 0.0;  // 6·V accumulator
  for (const Triangle& t : m.triangles) {
    const math::Vec3 a = m.vertices[t.a].asVec();
    const math::Vec3 b = m.vertices[t.b].asVec();
    const math::Vec3 c = m.vertices[t.c].asVec();
    vol6 += math::dot(a, math::cross(b, c));
  }
  return vol6 / 6.0;
}

// ── Shared-edge accounting (basis of the manifold/watertight checks) ──────────

/// An undirected edge key: the two endpoint vertex indices, min first, so
/// triangle (a,b,c) and its neighbour sharing that edge hash to the same key
/// regardless of traversal direction.
struct UndirectedEdge {
  std::uint32_t lo;
  std::uint32_t hi;
  bool operator==(const UndirectedEdge& o) const noexcept { return lo == o.lo && hi == o.hi; }
};
struct UndirectedEdgeHash {
  std::size_t operator()(const UndirectedEdge& e) const noexcept {
    // Cantor-style pairing keeps the hash well-spread for small indices.
    return (static_cast<std::size_t>(e.lo) << 32) ^ static_cast<std::size_t>(e.hi);
  }
};

/// Count how many triangles use each undirected edge. The result is the sole
/// input to both manifold checks (an edge used exactly twice = interior; once =
/// boundary; three+ = non-manifold).
inline std::unordered_map<UndirectedEdge, int, UndirectedEdgeHash> edgeUseCounts(const Mesh& m) {
  std::unordered_map<UndirectedEdge, int, UndirectedEdgeHash> counts;
  counts.reserve(m.triangles.size() * 3);
  auto bump = [&](std::uint32_t x, std::uint32_t y) {
    const UndirectedEdge e{std::min(x, y), std::max(x, y)};
    ++counts[e];
  };
  for (const Triangle& t : m.triangles) {
    bump(t.a, t.b);
    bump(t.b, t.c);
    bump(t.c, t.a);
  }
  return counts;
}

/// Watertight ⇔ closed 2-manifold: every undirected edge is shared by EXACTLY
/// two triangles. This rejects both boundary edges (used once — a hole/open
/// shell) and non-manifold edges (used 3+ times). A degenerate mesh (no
/// triangles) is not watertight.
inline bool isWatertight(const Mesh& m) {
  if (m.triangles.empty()) return false;
  for (const auto& [edge, uses] : edgeUseCounts(m))
    if (uses != 2) return false;
  return true;
}

/// 2-manifold (possibly with boundary): no undirected edge is used by more than
/// two triangles. Weaker than watertight — an open surface patch (a single face
/// mesh) satisfies this but not isWatertight().
inline bool isTwoManifold(const Mesh& m) {
  for (const auto& [edge, uses] : edgeUseCounts(m))
    if (uses > 2) return false;
  return true;
}

/// Number of undirected edges used by exactly one triangle (the open boundary).
/// Zero ⇔ closed. Useful for diagnostics in the parity harness.
inline std::size_t boundaryEdgeCount(const Mesh& m) {
  std::size_t n = 0;
  for (const auto& [edge, uses] : edgeUseCounts(m))
    if (uses == 1) ++n;
  return n;
}

// ── Directed-edge (orientation-coherence) accounting ──────────────────────────
// isWatertight() is UNDIRECTED: it only asks that each edge be used twice, so a
// shell whose two halves wind the SAME way across a shared seam (both caps CW, or
// both CCW) still passes — yet it is NOT a coherently-oriented solid boundary and
// its signed enclosedVolume() is wrong. The DIRECTED test closes that gap: in a
// consistently-oriented closed 2-manifold every directed half-edge (a→b) occurs
// exactly once and is matched by exactly one reverse (b→a) from the adjacent
// triangle. A same-direction duplicate (a→b seen from two triangles) is the
// signature of an inconsistent shell.

/// A directed edge key (ordered endpoints) for orientation-coherence counting.
struct DirectedEdge {
  std::uint32_t from;
  std::uint32_t to;
  bool operator==(const DirectedEdge& o) const noexcept { return from == o.from && to == o.to; }
};
struct DirectedEdgeHash {
  std::size_t operator()(const DirectedEdge& e) const noexcept {
    return (static_cast<std::size_t>(e.from) << 32) ^ static_cast<std::size_t>(e.to);
  }
};

/// Number of DIRECTED half-edges that occur more than once in the same direction
/// (each extra use counted). Zero ⇔ no two triangles traverse the same edge the
/// same way — the orientation-coherence signature of a consistently-wound shell.
inline std::size_t sameDirectionEdgeCount(const Mesh& m) {
  std::unordered_map<DirectedEdge, int, DirectedEdgeHash> dir;
  dir.reserve(m.triangles.size() * 3);
  auto bump = [&](std::uint32_t x, std::uint32_t y) { ++dir[DirectedEdge{x, y}]; };
  for (const Triangle& t : m.triangles) {
    bump(t.a, t.b);
    bump(t.b, t.c);
    bump(t.c, t.a);
  }
  std::size_t dup = 0;
  for (const auto& [edge, uses] : dir)
    if (uses > 1) dup += static_cast<std::size_t>(uses - 1);
  return dup;
}

/// A closed mesh is CONSISTENTLY ORIENTED iff it is watertight AND no directed
/// half-edge is traversed the same way twice (every interior edge is used once
/// forward and once reversed). This is the invariant that makes enclosedVolume()
/// meaningful — a watertight-but-orientation-inconsistent shell fails it.
inline bool isConsistentlyOriented(const Mesh& m) {
  return isWatertight(m) && sameDirectionEdgeCount(m) == 0;
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_MESH_H

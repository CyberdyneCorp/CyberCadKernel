// SPDX-License-Identifier: Apache-2.0
//
// solid_mesher.h — mesh every Face of a Solid and stitch them into one mesh.
//
// Meshing a closed solid must yield a WATERTIGHT mesh: the per-face meshes have
// to agree exactly along shared edges so neighbouring faces' boundary vertices
// coincide and the shared-edge count is exactly two everywhere — INCLUDING curved
// shared edges (cylinder cap↔side circle, fillet blend seams). Two cooperating
// mechanisms make that hold; the mesher uses both:
//
//   1. SHARED PER-EDGE 1D DISCRETIZATION (the two-stage core). An edge is shared
//      by two faces (topology shares the edge node). STAGE 1 (edge_mesher.h) builds
//      ONE deflection-based fraction list per unique edge, cached by the edge's
//      TShape node. STAGE 2 (face_mesher.h) pins BOTH adjacent faces' boundary
//      vertices to those SAME fractions, mapped through each face's pcurve; because
//      a pcurve satisfies S_face(pcurve(f)) = C_edge(f), the two faces place the
//      SAME 3D point at each fraction. This is the exact analogue of OCCT BRepMesh
//      building a shared 1D discretization per edge before meshing the faces, and
//      it is what makes CURVED shared edges agree (the case independent per-face
//      grids left open).
//
//   2. SPATIAL WELDING (stitch). After accumulating all face meshes, coincident
//      vertices (within a weld tolerance) are merged via a spatial hash grid, so
//      the (now identical) shared boundary points collapse to one index. Welding is
//      what turns the shared-edge agreement into a topologically closed mesh (every
//      interior edge used by exactly two triangles).
//
// For a closed solid the result passes isWatertight(); enclosedVolume() then
// approximates the solid's volume and converges as deflection → 0.
//
// The edge discretization is the piece that guarantees agreement BEFORE welding;
// welding alone (proximity) is a safety net whose tolerance must not exceed the
// smallest feature, so we derive it from the deflection (a fraction of it).
//
// Cognitive complexity: mesh() delegates edge pre-sampling, per-face meshing and
// welding to helpers (driver ≤ ~10). The weld grid loop is flagged systems-band
// (~14).
//
// OCCT-FREE. Uses src/native/math + src/native/topology. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_SOLID_MESHER_H
#define CYBERCAD_NATIVE_TESSELLATE_SOLID_MESHER_H

#include "native/tessellate/face_mesher.h"
#include "native/tessellate/mesh.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace cybercad::native::tessellate {

// ─────────────────────────────────────────────────────────────────────────────
// Spatial weld — merge vertices closer than `tol` into a single index.
//
// A hash grid on cells of side `tol` groups near-coincident points; a merged
// vertex is the first occupant of its cell (a representative). Triangles are
// re-indexed onto the merged set and degenerate triangles (two indices equal
// after the merge) are dropped. Per-vertex normals, if present, are carried from
// the representative.
// ─────────────────────────────────────────────────────────────────────────────
class VertexWelder {
 public:
  explicit VertexWelder(double tol) noexcept : tol_(tol > 0 ? tol : 1e-9), inv_(1.0 / tol_) {}

  Mesh weld(const Mesh& in) const {
    Mesh out;
    const bool normals = in.hasNormals();
    if (normals) out.normals.reserve(in.vertices.size());

    std::unordered_map<Cell, std::uint32_t, CellHash> cellToNew;
    cellToNew.reserve(in.vertices.size() * 2);
    std::vector<std::uint32_t> remap(in.vertices.size());

    for (std::size_t i = 0; i < in.vertices.size(); ++i)
      remap[i] = mapVertex(in, i, normals, out, cellToNew);

    reindexTriangles(in, remap, out);
    return out;
  }

 private:
  struct Cell {
    long long x, y, z;
    bool operator==(const Cell& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
      // Mix the three lattice coords (large primes) — cheap and well-spread.
      std::size_t h = static_cast<std::size_t>(c.x) * 73856093u;
      h ^= static_cast<std::size_t>(c.y) * 19349663u;
      h ^= static_cast<std::size_t>(c.z) * 83492791u;
      return h;
    }
  };

  Cell cellOf(const math::Point3& p) const noexcept {
    // Round to the nearest cell so points on either side of a boundary that are
    // within tol land together (nearest, not floor, halves the boundary-split risk).
    return Cell{llround(p.x * inv_), llround(p.y * inv_), llround(p.z * inv_)};
  }
  static long long llround(double v) noexcept {
    return static_cast<long long>(v >= 0 ? v + 0.5 : v - 0.5);
  }

  // Return the merged index of input vertex i, inserting it as a representative
  // if its cell is empty.
  std::uint32_t mapVertex(const Mesh& in, std::size_t i, bool normals, Mesh& out,
                          std::unordered_map<Cell, std::uint32_t, CellHash>& cellToNew) const {
    const Cell c = cellOf(in.vertices[i]);
    if (auto it = cellToNew.find(c); it != cellToNew.end()) return it->second;
    const auto id = static_cast<std::uint32_t>(out.vertices.size());
    out.vertices.push_back(in.vertices[i]);
    if (normals) out.normals.push_back(in.normals[i]);
    cellToNew.emplace(c, id);
    return id;
  }

  static void reindexTriangles(const Mesh& in, const std::vector<std::uint32_t>& remap, Mesh& out) {
    out.triangles.reserve(in.triangles.size());
    for (const Triangle& t : in.triangles) {
      const std::uint32_t a = remap[t.a], b = remap[t.b], c = remap[t.c];
      if (a == b || b == c || a == c) continue;  // collapsed → drop
      out.triangles.push_back(Triangle{a, b, c});
    }
  }

  double tol_;
  double inv_;
};

// ─────────────────────────────────────────────────────────────────────────────
// SolidMesher — tessellate every face of a shape (Solid/Shell/Compound) and
// stitch. Works on any shape the Explorer can walk for faces; a closed solid
// yields a watertight mesh.
// ─────────────────────────────────────────────────────────────────────────────
class SolidMesher {
 public:
  explicit SolidMesher(MeshParams params = {}) noexcept : p_(params) {}

  Mesh mesh(const topo::Shape& shape) const {
    Mesh accumulated = meshAllFaces(shape);
    // Weld tolerance: a fraction of the deflection, so it merges the coincident
    // boundary vertices two faces put on a shared edge without collapsing genuine
    // nearby features. (Shared-edge agreement makes those vertices land within
    // fp round-off; a generous 0.5·deflection floor guards curved-edge sampling.)
    const double weldTol = std::max(p_.deflection * 0.5, 1e-7);
    return VertexWelder{weldTol}.weld(accumulated);
  }

 private:
  // Mesh every face and append into one (pre-weld) mesh, sharing ONE EdgeCache
  // across all faces. STAGE 1 (the cache) discretizes each unique topological edge
  // ONCE into a shared fraction list; STAGE 2 (each FaceMesher::mesh) pins that
  // face's boundary to those fractions. Two faces sharing an edge therefore place
  // boundary vertices at IDENTICAL 3D points (each maps the same fraction through
  // its own pcurve: S_face(pcurve(f)) = C_edge(f)), so welding fuses them and the
  // seam is watertight — including CURVED shared edges (cylinder cap↔side circle,
  // fillet blend seams), the gap the old independent-grid path left open.
  Mesh meshAllFaces(const topo::Shape& shape) const {
    FaceMesher fm(p_);
    EdgeCache cache(p_.deflection, p_.edgeMinSegs, p_.edgeMaxSegs);
    Mesh all;
    for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next())
      all.append(fm.mesh(ex.current(), cache));
    return all;
  }

  MeshParams p_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_SOLID_MESHER_H

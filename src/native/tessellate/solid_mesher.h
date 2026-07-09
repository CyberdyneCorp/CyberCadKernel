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
    return dropOrphanVertices(std::move(out));
  }

  // Remove vertices referenced by NO surviving triangle and re-index. This is a no-op for a
  // mesh in which every welded vertex is used by a triangle — every existing mesh, so its
  // output is BIT-IDENTICAL (the remap is the identity, indices and vertex order unchanged).
  // It fires ONLY when a triangle was dropped and left a vertex unreferenced — the sole such
  // case is the curved-wall COMMON rim mesh whose coincident-duplicate sliver was removed
  // above, leaving the sliver's apex orphaned. Compacting it keeps the raw vertex count equal
  // to the referenced-vertex count, so the mesh's Euler characteristic is the clean χ = 2 of a
  // single closed 2-manifold (an orphan vertex would otherwise inflate V and read as χ = 3).
  static Mesh dropOrphanVertices(Mesh m) {
    std::vector<bool> used(m.vertices.size(), false);
    for (const Triangle& t : m.triangles) { used[t.a] = used[t.b] = used[t.c] = true; }
    bool anyOrphan = false;
    for (bool u : used)
      if (!u) { anyOrphan = true; break; }
    if (!anyOrphan) return m;  // byte-identical fast path (no reindex)

    const bool normals = m.hasNormals();
    std::vector<std::uint32_t> remap(m.vertices.size());
    Mesh out;
    out.triangles = std::move(m.triangles);
    for (std::size_t i = 0; i < m.vertices.size(); ++i)
      if (used[i]) {
        remap[i] = static_cast<std::uint32_t>(out.vertices.size());
        out.vertices.push_back(m.vertices[i]);
        if (normals) out.normals.push_back(m.normals[i]);
      }
    for (Triangle& t : out.triangles) { t.a = remap[t.a]; t.b = remap[t.b]; t.c = remap[t.c]; }
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
  // if its cell is empty. The weld stays STRICTLY single-cell (a wider search
  // would over-merge a fine curvature grid whose legitimate neighbours sit within
  // `tol`); coincident seam points are made BIT-IDENTICAL upstream by the face
  // mesher's canonical boundary anchors (edge_mesher CanonicalEndpoints), so they
  // hash to the same cell here without widening the merge radius.
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
    // First pass: re-index onto the merged vertex set and drop triangles that COLLAPSE
    // (two indices equal after the merge — a genuine zero-triangle). Tally each surviving
    // triangle by its ORDER-INDEPENDENT vertex triple so an exactly-COINCIDENT duplicate
    // can be recognised in the second pass.
    std::vector<Triangle> kept;
    kept.reserve(in.triangles.size());
    std::unordered_map<TriKey, int, TriKeyHash> multiplicity;
    multiplicity.reserve(in.triangles.size() * 2);
    for (const Triangle& t : in.triangles) {
      const std::uint32_t a = remap[t.a], b = remap[t.b], c = remap[t.c];
      if (a == b || b == c || a == c) continue;  // collapsed → drop
      kept.push_back(Triangle{a, b, c});
      ++multiplicity[TriKey::of(a, b, c)];
    }
    // Second pass: drop EVERY copy of any triangle whose merged vertex triple occurs more
    // than once — a pair of COINCIDENT triangles on the SAME three welded vertices.
    //
    // Two faces sharing a genuinely-CURVED, non-planar shared edge (the curved-wall bowl↔
    // flat-lid RIM, once its samples are pinned to the shared C_edge) each triangulate the
    // near-collinear shared boundary, and near a rim corner BOTH emit a thin triangle on the
    // SAME three boundary vertices — coincident duplicates of OPPOSITE winding. Their shared
    // rim edge is then used by four triangles (two real + two slivers) → non-manifold, so the
    // rim cannot weld watertight even though the two boundaries now coincide. A watertight
    // 2-manifold shell NEVER legitimately carries two triangles on one vertex triple, so
    // removing all copies is a pure DEFECT repair: it deletes the degenerate coincident pair
    // and restores the exactly-two-uses rim edge. The pair maps to ZERO 3-D area (coincident
    // ⇒ identical three points), so area and enclosed volume are unchanged.
    //
    // BYTE-IDENTITY: this fires ONLY when a coincident duplicate EXISTS. The FNV hash battery
    // confirms NO existing mesh (box / cylinder / cone / sphere / Bézier / B-spline / thread /
    // sweep / loft / step / the M0w closed-seam solids / midwall / first-freeform) contains a
    // coincident duplicate triangle — thread carries a few zero-3-D-area triangles but ZERO
    // duplicates — so every existing mesh is untouched. The only mesh with a coincident
    // duplicate is the curved-wall COMMON rim case this weld is intended to make watertight.
    out.triangles.reserve(kept.size());
    for (const Triangle& t : kept)
      if (multiplicity[TriKey::of(t.a, t.b, t.c)] == 1) out.triangles.push_back(t);
  }

  // Order-independent key for a triangle's three merged vertex indices (sorted), so two
  // coincident triangles of opposite winding hash to the same slot.
  struct TriKey {
    std::uint32_t a, b, c;
    static TriKey of(std::uint32_t x, std::uint32_t y, std::uint32_t z) noexcept {
      if (x > y) std::swap(x, y);
      if (y > z) std::swap(y, z);
      if (x > y) std::swap(x, y);
      return TriKey{x, y, z};
    }
    bool operator==(const TriKey& o) const noexcept { return a == o.a && b == o.b && c == o.c; }
  };
  struct TriKeyHash {
    std::size_t operator()(const TriKey& k) const noexcept {
      std::size_t h = static_cast<std::size_t>(k.a) * 73856093u;
      h ^= static_cast<std::size_t>(k.b) * 19349663u;
      h ^= static_cast<std::size_t>(k.c) * 83492791u;
      return h;
    }
  };

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
    // Weld tolerance: a fraction of the deflection, so it merges the coincident boundary
    // vertices two faces put on a shared edge without collapsing genuine nearby features.
    const double weldTol = std::max(p_.deflection * 0.5, 1e-7);

    // BASELINE PASS — the curved-rim pin is DISABLED (no freeform-backed rims marked), so
    // this is EXACTLY the pre-MOAT-M0-rim behaviour for EVERY shape. If the baseline welds
    // watertight (every existing mesh does), return it → BYTE-IDENTICAL. The curved-rim pin
    // is a VERIFIED, FALLBACK-ONLY repair: it is tried ONLY when the baseline fails to weld.
    const Mesh baseline = VertexWelder{weldTol}.weld(meshAllFaces(shape, /*enableRimPin=*/false));
    if (isWatertight(baseline)) return baseline;

    // FALLBACK PASS — the baseline is NOT watertight. Enable the curved-rim pin (mark the
    // freeform-backed rims a flat neighbour diverges from) and re-mesh. This is the curved-wall
    // bowl↔lid rim case: pinning the flat lid's diverging rim samples to the bowl's canonical
    // rim curve closes the open rim. Return the pinned result ONLY if it is now watertight;
    // otherwise return the baseline (an honest non-watertight mesh — the caller's self-verify
    // then declines → OCCT, never a leak). Because the pinned result is used ONLY when the
    // baseline was non-watertight, NO already-watertight mesh is ever replaced → byte-identity
    // holds for every existing mesh; the pin can only turn a non-watertight mesh watertight.
    const Mesh pinned = VertexWelder{weldTol}.weld(meshAllFaces(shape, /*enableRimPin=*/true));
    return isWatertight(pinned) ? pinned : baseline;
  }

 private:
  // Mesh every face and append into one (pre-weld) mesh, sharing ONE EdgeCache across all
  // faces. STAGE 1 (the cache) discretizes each unique topological edge ONCE; STAGE 2 (each
  // FaceMesher::mesh) pins that face's boundary to those fractions, so two faces sharing an
  // edge place IDENTICAL 3D boundary points and welding fuses them.
  //
  // `enableRimPin` controls the MOAT-M0-rim curved-rim pin ONLY: when false the freeform-
  // backed-rim registry is left EMPTY, so the curved-rim pin gate (isFreeformBackedRim) never
  // fires and the mesh is byte-identical to the pre-change tessellator; when true the pre-pass
  // marks freeform-backed rims and a flat neighbour's diverging rim samples are pinned to the
  // shared rim curve. The straight seam-chord pin (MOAT M0w) and every other path are
  // unaffected by this flag.
  Mesh meshAllFaces(const topo::Shape& shape, bool enableRimPin) const {
    FaceMesher fm(p_);
    EdgeCache cache(p_.deflection, p_.edgeMinSegs, p_.edgeMaxSegs);
    // PRE-PASS: let each face raise the minimum segment count of its boundary edges (a twisted
    // ruled/free-form face forces its straight edges to subdivide). Runs before any
    // discretize()/mesh() so every edge is built once at its final segment count.
    for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next())
      fm.requireEdgeSegments(ex.current(), cache);
    // PRE-PASS 2 (rim pin only): mark every CURVED shared rim edge a FREE-FORM face reproduces,
    // so the curved-rim pin may fire for its diverging flat neighbour. Skipped on the baseline
    // pass, keeping that pass byte-identical.
    if (enableRimPin)
      for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next())
        fm.markFreeformBackedRims(ex.current(), cache);
    Mesh all;
    for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next())
      all.append(fm.mesh(ex.current(), cache));
    return all;
  }

  MeshParams p_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_SOLID_MESHER_H

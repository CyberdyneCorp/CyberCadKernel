// SPDX-License-Identifier: Apache-2.0
//
// assemble.h — turn the bag of surviving planar Polygons a boolean produces back
// into a native B-rep Solid (Phase 4 #5 `native-booleans`).
//
// After the BSP-CSG core (bsp.h) has clipped/merged the two solids' polygon soups
// for the requested op, the result is a list of oriented planar Polygons whose
// union is the boundary of the answer. To hand that to the rest of the kernel
// (tessellate / mass / query) it must become a topology::Solid:
//
//   1. WELD vertices — coincident polygon corners (within a weld tolerance) collapse
//      to ONE shared topology Vertex node, so adjacent faces share vertices and the
//      tessellator welds them watertight (mirrors the tessellation weld idea, reused
//      here at B-rep level).
//   2. build one planar FACE per Polygon — a Plane surface (frame Z = the polygon's
//      outward normal) bounded by a wire of Line edges, each edge carrying its Line
//      pcurve on that plane so the existing FaceMesher can flatten + ear-clip it.
//   3. wrap the faces in a Shell → Solid.
//
// The face-build reuses the SAME planar-face construction the extrude builder uses
// (a Plane frame + per-edge Line pcurves) so the boolean's output tessellates by the
// identical path a natively-built prism does — which is exactly what the self-verify
// guard (watertight + volume) then checks.
//
// CLEAN-ROOM. Uses src/native/math + src/native/topology + polygon.h. No OCCT.
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_ASSEMBLE_H
#define CYBERCAD_NATIVE_BOOLEAN_ASSEMBLE_H

#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/tessellate/trim.h"            // UV
#include "native/tessellate/uv_triangulate.h"  // triangulatePolygon
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

namespace tess = cybercad::native::tessellate;

// Weld tolerance for coincident boolean vertices. The BSP split interpolates
// crossing points exactly (same t on both sides of an edge), so shared corners
// land within fp round-off; a modest tolerance fuses them without collapsing a
// genuine small feature of an axis-aligned box.
inline constexpr double kWeldTol = 1e-7;

// ─────────────────────────────────────────────────────────────────────────────
// VertexPool — hash coincident points to a single shared topology Vertex node so
// faces of the assembled solid share vertices (a spatial hash on cells of side
// kWeldTol, nearest-cell rounding, mirroring the tessellator's VertexWelder).
// ─────────────────────────────────────────────────────────────────────────────
class VertexPool {
 public:
  topo::Shape vertexFor(const math::Point3& p) {
    const Cell c = cellOf(p);
    if (auto it = cells_.find(c); it != cells_.end()) return verts_[it->second];
    const auto id = verts_.size();
    verts_.push_back(topo::ShapeBuilder::makeVertex(p));
    cells_.emplace(c, id);
    return verts_.back();
  }

 private:
  struct Cell {
    long long x, y, z;
    bool operator==(const Cell& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
      std::size_t h = static_cast<std::size_t>(c.x) * 73856093u;
      h ^= static_cast<std::size_t>(c.y) * 19349663u;
      h ^= static_cast<std::size_t>(c.z) * 83492791u;
      return h;
    }
  };
  static long long lr(double v) noexcept {
    return static_cast<long long>(v >= 0 ? v + 0.5 : v - 0.5);
  }
  Cell cellOf(const math::Point3& p) const noexcept {
    const double inv = 1.0 / kWeldTol;
    return Cell{lr(p.x * inv), lr(p.y * inv), lr(p.z * inv)};
  }

  std::unordered_map<Cell, std::size_t, CellHash> cells_;
  std::vector<topo::Shape> verts_;
};

namespace detail {

// A straight edge with a Line pcurve on the given planar frame, between two SHARED
// vertices (from the pool). Mirrors construct.h planarEdge but takes pre-welded
// vertices so the face graph shares corners.
inline topo::Shape planarEdgeShared(const topo::Shape& v0, const topo::Shape& v1,
                                    const math::Ax3& frame) {
  const auto p0 = topo::pointOf(v0);
  const auto p1 = topo::pointOf(v1);
  const math::Vec3 d = *p1 - *p0;
  const double len = std::max(math::norm(d), kWeldTol);

  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Line;
  c.frame.origin = *p0;
  c.frame.x = math::norm(d) > kWeldTol ? math::Dir3{d} : math::Dir3{1, 0, 0};
  c.frame.z = frame.z;
  const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);

  auto toUV = [&](const math::Point3& p) -> math::Point3 {
    const math::Vec3 dd = p - frame.origin;
    return math::Point3{math::dot(dd, frame.x.vec()), math::dot(dd, frame.y.vec()), 0.0};
  };
  const math::Point3 uv0 = toUV(*p0), uv1 = toUV(*p1);
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Line;
  pc.origin2d = uv0;
  pc.dir2d = (uv1 - uv0) / len;
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// One triangular planar Face from three shared vertices on `frame` (Z = outward
// normal). The three corners are shared pool vertices so neighbouring triangles
// weld; each side is a Line edge with its Line pcurve so the FaceMesher meshes it
// as a single triangle (no interior points → exact).
inline topo::Shape triangleFace(const topo::Shape& a, const topo::Shape& b, const topo::Shape& c,
                                const math::Ax3& frame) {
  std::vector<topo::Shape> edges;
  edges.push_back(planarEdgeShared(a, b, frame));
  edges.push_back(planarEdgeShared(b, c, frame));
  edges.push_back(planarEdgeShared(c, a, frame));
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));

  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, wire, {}, topo::Orientation::Forward);
}

// Triangulate a Polygon into a set of triangular planar Faces (shared pool
// vertices) and append them to `faces`.
//
// Why triangulate at the B-rep level: a boolean tiles each merged plane with
// fragments that can be CONCAVE (an L-shaped union cap) and can meet at
// T-junctions. Emitting one N-gon face per fragment leaves the mesher to ear-clip
// each fragment independently, so two coplanar fragments' triangulations need not
// agree along their shared straight span → hairline cracks (not watertight even
// though the volume is exact). Triangulating HERE, after T-junction repair inserts
// every crossing point as a real corner, makes every face a triangle whose three
// straight edges are shared-vertex Line edges: adjacent triangles (coplanar or
// across a solid edge) share those vertices, so the welded mesh closes. The
// polygon is projected to its plane's 2D frame, ear-clipped (uv_triangulate,
// hole-free), and each 2D triangle mapped back to the shared 3D vertices.
inline void triangulatePolygonToFaces(const Polygon& poly, VertexPool& pool,
                                      std::vector<topo::Shape>& faces) {
  const math::Dir3 nz{poly.plane.normal};
  if (!nz.valid() || poly.size() < 3) return;
  const math::Vec3 ref = std::fabs(nz.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(poly.vertices.front(), nz, math::Dir3{ref});

  // Project to 2D on the plane frame and collect shared vertices in loop order.
  std::vector<tess::UV> uv;
  std::vector<topo::Shape> verts;
  uv.reserve(poly.size());
  verts.reserve(poly.size());
  for (const math::Point3& p : poly.vertices) {
    const math::Vec3 d = p - frame.origin;
    uv.push_back(tess::UV{math::dot(d, frame.x.vec()), math::dot(d, frame.y.vec())});
    verts.push_back(pool.vertexFor(p));
  }

  std::vector<int> loop(poly.size());
  for (int i = 0; i < static_cast<int>(poly.size()); ++i) loop[i] = i;
  const std::vector<tess::UVTri> tris =
      tess::triangulatePolygon(uv, std::vector<std::vector<int>>{loop});

  for (const tess::UVTri& t : tris) {
    const topo::Shape& va = verts[static_cast<std::size_t>(t.a)];
    const topo::Shape& vb = verts[static_cast<std::size_t>(t.b)];
    const topo::Shape& vc = verts[static_cast<std::size_t>(t.c)];
    // Drop a degenerate triangle (two welded-coincident corners).
    if (va.isSameGeometry(vb) || vb.isSameGeometry(vc) || va.isSameGeometry(vc)) continue;
    faces.push_back(triangleFace(va, vb, vc, frame));
  }
}

// ── T-junction repair ─────────────────────────────────────────────────────────
// BSP-CSG tiles a merged plane with fragments from different split histories, so a
// corner of one fragment can land in the MIDDLE of a neighbour's edge (a
// "T-junction"): the neighbour's edge has no vertex there, so the two faces do not
// share that point and the tessellator leaves a hairline crack (mesh not
// watertight even though the geometry is closed and the volume exact). This is the
// classic BSP-CSG watertightness gap.
//
// Repair: gather every distinct polygon corner (welded to a tolerance), then for
// each polygon edge insert, in order along the edge, any gathered point that lies
// strictly ON that edge (collinear + between the endpoints, off both ends). After
// this every T-junction point is a real vertex of BOTH incident faces, so the welded
// mesh closes.

inline bool onSegmentInterior(const math::Point3& a, const math::Point3& b,
                              const math::Point3& p, double tol) {
  const math::Vec3 ab = b - a;
  const double len2 = math::normSquared(ab);
  if (len2 < tol * tol) return false;
  const math::Vec3 ap = p - a;
  const double t = math::dot(ap, ab) / len2;
  if (t <= tol || t >= 1.0 - tol) return false;  // at or beyond an endpoint
  // Perpendicular distance from the segment.
  const math::Point3 proj{a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t};
  return math::distance(proj, p) <= tol;
}

// Insert collinear-interior points from `all` into each edge of `poly`, in order.
inline Polygon repairPolygon(const Polygon& poly, const std::vector<math::Point3>& all,
                             double tol) {
  std::vector<math::Point3> out;
  const std::size_t n = poly.size();
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3& a = poly.vertices[i];
    const math::Point3& b = poly.vertices[(i + 1) % n];
    out.push_back(a);
    // Collect points on (a,b), sorted by parameter along the edge.
    const math::Vec3 ab = b - a;
    const double len2 = math::normSquared(ab);
    std::vector<std::pair<double, math::Point3>> mids;
    for (const math::Point3& p : all) {
      if (onSegmentInterior(a, b, p, tol)) {
        const double t = math::dot(p - a, ab) / len2;
        mids.emplace_back(t, p);
      }
    }
    std::sort(mids.begin(), mids.end(),
              [](const auto& x, const auto& y) { return x.first < y.first; });
    for (const auto& [t, p] : mids) out.push_back(p);
  }
  return Polygon{std::move(out), poly.plane};
}

// The distinct corner points of every polygon (welded to `tol` via a grid) — the
// candidate T-junction insertion set.
inline std::vector<math::Point3> distinctCorners(const std::vector<Polygon>& polys, double tol) {
  std::vector<math::Point3> pts;
  auto near = [&](const math::Point3& p) {
    for (const math::Point3& q : pts)
      if (math::distance(p, q) <= tol) return true;
    return false;
  };
  for (const Polygon& poly : polys)
    for (const math::Point3& v : poly.vertices)
      if (!near(v)) pts.push_back(v);
  return pts;
}

}  // namespace detail

/// Repair T-junctions across a coplanar-tiled polygon soup (see repairPolygon).
inline std::vector<Polygon> repairTJunctions(const std::vector<Polygon>& polys) {
  const std::vector<math::Point3> corners = detail::distinctCorners(polys, kWeldTol);
  std::vector<Polygon> out;
  out.reserve(polys.size());
  for (const Polygon& p : polys) out.push_back(detail::repairPolygon(p, corners, kWeldTol));
  return out;
}

/// Assemble a bag of oriented planar Polygons into a native Solid (welded vertices,
/// one planar face per polygon, one shell). Runs a T-junction repair first so the
/// coplanar-tiled boolean output closes watertight. Returns a NULL Shape if fewer
/// than four valid faces survive (no closed solid possible).
inline topo::Shape assembleSolid(const std::vector<Polygon>& polysIn) {
  const std::vector<Polygon> polys = repairTJunctions(polysIn);
  VertexPool pool;
  std::vector<topo::Shape> faces;
  faces.reserve(polys.size() * 2);
  for (const Polygon& p : polys) detail::triangulatePolygonToFaces(p, pool, faces);
  if (faces.size() < 4) return {};  // a solid needs ≥ 4 faces (tetrahedron)
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_ASSEMBLE_H

// SPDX-License-Identifier: Apache-2.0
//
// blend_geom.h — shared geometry helpers for the native blend slice (Phase 4 #6
// `native-blends`, openspec/NATIVE-REWRITE.md).
//
// The blend ops (chamfer / fillet / offset-face / shell) all operate on a
// PLANAR-FACED solid by editing its bag of oriented planar polygons (the same
// Polygon representation the native boolean flattens a solid into) and
// re-assembling a watertight Solid with the boolean's assembleSolid (weld +
// T-junction repair + triangulate). Working at the polygon-soup level means:
//
//   * the input topology is read through the SAME extractPolygons the boolean uses
//     (world-space loops, outward-front normal), and
//   * the output is re-welded and re-triangulated by assembleSolid, so it meshes
//     watertight by the identical path a native prism / boolean result does — which
//     is exactly what the engine's mandatory self-verify (watertight + sane volume)
//     then checks. A blend that cannot be expressed as a clean planar-polygon edit
//     returns a NULL Shape and the engine falls through to OCCT (never faked).
//
// This header owns the pieces every op shares:
//   * PlanarModel        — the solid flattened to oriented planar Polygons + an
//                          adjacency index (edge key → the ≤2 faces on it) so an op
//                          can find the two faces meeting at a picked edge.
//   * edge/face id lookup — resolve the 1-based ABI ids (mapShapes order) to the
//                          world-space geometry of the picked edge / face.
//   * half-space clip     — clip every polygon of the model to one side of a plane
//                          (used by chamfer/fillet setback and by shell/offset).
//
// CLEAN-ROOM. Uses src/native/math + src/native/topology + src/native/boolean
// (Polygon/Plane/assembleSolid) + src/native/tessellate (via boolean/assemble). No
// OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_GEOM_H
#define CYBERCAD_NATIVE_BLEND_GEOM_H

#include "native/boolean/assemble.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace nb = cybercad::native::boolean;

// On-plane / coincidence tolerance for blend edits. Matches the boolean's
// kPlaneEps band so setback planes and re-extracted faces classify consistently.
inline constexpr double kBlendEps = 1e-7;

// ─────────────────────────────────────────────────────────────────────────────
// A directed segment of a polygon boundary in world coordinates. Two faces that
// share a solid edge each contribute one such segment with OPPOSITE direction; we
// key an undirected edge by its rounded endpoints so the two faces on a picked
// edge can be found.
// ─────────────────────────────────────────────────────────────────────────────
struct EdgeKey {
  std::array<long long, 6> k{};  // (lo.xyz, hi.xyz) quantized
  bool operator==(const EdgeKey& o) const noexcept { return k == o.k; }
};

inline long long quant(double v) noexcept {
  const double s = 1.0 / kBlendEps;
  return static_cast<long long>(v >= 0 ? v * s + 0.5 : v * s - 0.5);
}

inline EdgeKey edgeKeyOf(const math::Point3& a, const math::Point3& b) noexcept {
  const std::array<long long, 3> qa{quant(a.x), quant(a.y), quant(a.z)};
  const std::array<long long, 3> qb{quant(b.x), quant(b.y), quant(b.z)};
  // Order endpoints so the two directed uses hash the same.
  const bool aFirst = qa <= qb;
  const std::array<long long, 3>& lo = aFirst ? qa : qb;
  const std::array<long long, 3>& hi = aFirst ? qb : qa;
  return EdgeKey{{lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]}};
}

// ─────────────────────────────────────────────────────────────────────────────
// PlanarModel — a planar-faced solid flattened to oriented Polygons (world, front
// = outward) plus an edge→face adjacency so an op can locate the two faces on a
// picked crease. Only built for an all-planar solid; isValid() is false otherwise
// (→ the op returns NULL → OCCT fallthrough).
// ─────────────────────────────────────────────────────────────────────────────
class PlanarModel {
 public:
  explicit PlanarModel(const topo::Shape& solid) {
    if (solid.isNull() || !nb::isAllPlanar(solid)) return;
    polys_ = nb::extractPolygons(solid);
    valid_ = polys_.size() >= 4;
  }

  bool isValid() const noexcept { return valid_; }
  const std::vector<nb::Polygon>& polygons() const noexcept { return polys_; }
  std::vector<nb::Polygon>& polygons() noexcept { return polys_; }

 private:
  std::vector<nb::Polygon> polys_;
  bool valid_ = false;
};

// ─────────────────────────────────────────────────────────────────────────────
// Edge / face id resolution (ABI 1-based ids ↔ world geometry).
//
// The facade passes edge ids from mapShapes(solid, Edge) and face ids from
// mapShapes(solid, Face) — the SAME deterministic explorer order the engine's
// subshape_ids reports. We resolve an edge id to its two world endpoints and a
// face id to its outward plane, so a blend op can act on the picked feature.
// ─────────────────────────────────────────────────────────────────────────────

// World endpoints of the picked edge (1-based id). Returns nullopt if the id is
// out of range or the edge is not a straight line with two vertices.
struct EdgeEnds {
  math::Point3 a;
  math::Point3 b;
};
inline std::optional<EdgeEnds> edgeEnds(const topo::Shape& solid, int edgeId) {
  const topo::ShapeMap map = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeId < 1 || static_cast<std::size_t>(edgeId) > map.size()) return std::nullopt;
  const topo::Shape& edge = map.shape(edgeId);
  const auto& verts = edge.tshape()->children();
  if (verts.size() < 2) return std::nullopt;
  // Fold the edge's world location onto each vertex, then read its point.
  const topo::Shape v0 = verts.front().located(edge.location());
  const topo::Shape v1 = verts.back().located(edge.location());
  const auto pa = topo::pointOf(v0);
  const auto pb = topo::pointOf(v1);
  if (!pa || !pb) return std::nullopt;
  return EdgeEnds{*pa, *pb};
}

// Outward plane of the picked face (1-based id) in world coordinates. Returns
// nullopt if the id is out of range or the face is not planar.
inline std::optional<nb::Plane> facePlane(const topo::Shape& solid, int faceId) {
  const topo::ShapeMap map = topo::mapShapes(solid, topo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > map.size()) return std::nullopt;
  const topo::Shape& face = map.shape(faceId);
  const auto surf = topo::surfaceOf(face);
  if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Plane) return std::nullopt;

  math::Vec3 n = surf->surface->frame.z.vec();
  if (!surf->location.isIdentity()) n = surf->location.transform().applyToVector(n);
  if (face.orientation() == topo::Orientation::Reversed) n = -n;
  const math::Dir3 nd{n};
  if (!nd.valid()) return std::nullopt;

  // A point on the face: its outer wire's first vertex (world).
  const auto& wires = face.tshape()->children();
  if (wires.empty()) return std::nullopt;
  const topo::Shape outer = wires.front().located(face.location());
  topo::Explorer ex(outer, topo::ShapeType::Vertex);
  if (!ex.more()) return std::nullopt;
  const auto p = topo::pointOf(ex.current());
  if (!p) return std::nullopt;
  return nb::Plane::fromPointNormal(*p, nd.vec());
}

// ─────────────────────────────────────────────────────────────────────────────
// Polygon ↔ plane geometry.
// ─────────────────────────────────────────────────────────────────────────────

// Signed distance of a point from a boolean::Plane.
inline double signedDist(const nb::Plane& pl, const math::Point3& p) noexcept {
  return math::dot(pl.normal, p.asVec()) - pl.w;
}

// Clip a single convex-or-simple polygon to the HALF-SPACE signedDist <= 0 (the
// back side of `cut`, i.e. keep material behind the cut plane). Sutherland–Hodgman
// against one plane. Returns the clipped loop (empty if fully removed). The kept
// polygon inherits the source plane (a face being cut back stays on its own plane).
inline std::vector<math::Point3> clipBelow(const std::vector<math::Point3>& loop,
                                           const nb::Plane& cut, double eps = kBlendEps) {
  std::vector<math::Point3> out;
  const std::size_t n = loop.size();
  if (n < 3) return out;
  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3& a = loop[i];
    const math::Point3& b = loop[(i + 1) % n];
    const double da = signedDist(cut, a);
    const double db = signedDist(cut, b);
    const bool aIn = da <= eps;
    const bool bIn = db <= eps;
    if (aIn) out.push_back(a);
    if (aIn != bIn) {
      const double t = da / (da - db);  // crossing param
      out.push_back(math::Point3{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t,
                                 a.z + (b.z - a.z) * t});
    }
  }
  return out;
}

// Find the ≤2 polygon indices in a soup that carry the undirected edge (a,b).
// Fills found[0..n) and returns n. Used by chamfer/fillet to locate the two faces
// meeting at a picked crease within the (progressively edited) polygon soup.
inline int facesOnEdgeInSoup(const std::vector<nb::Polygon>& polys, const math::Point3& a,
                             const math::Point3& b, std::size_t found[2]) {
  const EdgeKey key = edgeKeyOf(a, b);
  int n = 0;
  for (std::size_t fi = 0; fi < polys.size() && n < 2; ++fi) {
    const auto& vs = polys[fi].vertices;
    for (std::size_t j = 0; j < vs.size(); ++j)
      if (edgeKeyOf(vs[j], vs[(j + 1) % vs.size()]) == key) { found[n++] = fi; break; }
  }
  return n;
}

// A unit direction along the crease (b − a), or nullopt if degenerate.
inline std::optional<math::Dir3> creaseDir(const math::Point3& a, const math::Point3& b) {
  const math::Dir3 d{b - a};
  if (!d.valid()) return std::nullopt;
  return d;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_GEOM_H

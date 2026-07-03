// SPDX-License-Identifier: Apache-2.0
//
// polygon.h — the planar-polygon primitive the native boolean operates on, plus
// extraction of these polygons from a native B-rep Solid.
//
// The native planar-polyhedron boolean (Phase 4 #5 `native-booleans`, see
// openspec/NATIVE-REWRITE.md) does NOT reason over the B-rep face graph directly.
// It first FLATTENS every solid into a bag of oriented convex-or-simple planar
// polygons (world coordinates, one supporting plane each, normal pointing OUT of
// the material), runs a BSP-based constructive-solid-geometry algorithm on those
// bags (bsp.h), and re-assembles the surviving polygons into a fresh native Solid
// (assemble.h). This header owns:
//
//   * Plane     — a supporting plane (unit normal + signed offset w = n·p).
//   * Polygon   — an ordered CCW-as-seen-from-outside vertex loop + its Plane.
//   * extractPolygons(solid) — walk the native topology and emit one Polygon per
//                              planar face (fan-independent: the loop is taken
//                              straight from the face's outer wire vertices, with
//                              the face's EFFECTIVE orientation folded into the
//                              stored normal so "front" always means outward).
//   * isAllPlanar(solid)     — the guard: the native boolean only accepts solids
//                              whose every face is a Plane. Any curved face (a
//                              Cylinder/Cone/Sphere/free-form) makes the engine
//                              fall through to OCCT.
//
// CLEAN-ROOM. Uses only src/native/math + src/native/topology. No OCCT. The plane/
// polygon split algebra (splitPolygon in bsp.h) is the classic BSP-CSG formulation
// (Naylor–Amanatides–Thibault, "Merging BSP Trees Yields Polyhedral Set
// Operations", SIGGRAPH 1990); nothing is copied from OCCT's BOPAlgo.
//
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_POLYGON_H
#define CYBERCAD_NATIVE_BOOLEAN_POLYGON_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::boolean {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

// Coplanarity / on-plane tolerance for the split classification. Deliberately
// looser than the raw fp tolerance so that vertices a box construction places on a
// shared plane (accumulated fp round-off through the frame math) all classify as
// COPLANAR rather than fragmenting the polygon into slivers. Axis-aligned integer-
// coordinate boxes stay exact well within this band.
inline constexpr double kPlaneEps = 1e-7;

// ─────────────────────────────────────────────────────────────────────────────
// Plane — supporting plane of a polygon: points x with dot(normal,x) == w.
// The normal is unit-length and points OUT of the solid's material (the polygon's
// front side). `w` is the signed distance of the plane from the origin along n.
// ─────────────────────────────────────────────────────────────────────────────
struct Plane {
  math::Vec3 normal{0, 0, 1};
  double w = 0.0;

  Plane() = default;
  Plane(const math::Vec3& n, double offset) noexcept : normal(n), w(offset) {}

  /// Plane through `p` with unit normal `n`.
  static Plane fromPointNormal(const math::Point3& p, const math::Vec3& n) noexcept {
    return Plane{n, math::dot(n, p.asVec())};
  }

  /// Signed distance of a point from the plane (>0 in front / normal side).
  double signedDistance(const math::Point3& p) const noexcept {
    return math::dot(normal, p.asVec()) - w;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Polygon — a planar face fragment: an ordered vertex loop (CCW as seen from the
// front, i.e. from the +normal side) plus its supporting plane. Fragments produced
// by the BSP split inherit the parent's plane exactly, so coplanar coincidence is
// decided by plane identity, not re-derivation.
// ─────────────────────────────────────────────────────────────────────────────
struct Polygon {
  std::vector<math::Point3> vertices;
  Plane plane;

  Polygon() = default;
  Polygon(std::vector<math::Point3> v, const Plane& pl) noexcept
      : vertices(std::move(v)), plane(pl) {}

  std::size_t size() const noexcept { return vertices.size(); }

  /// Flip the polygon: reverse the loop and negate the plane so the front side
  /// (material-outward) swaps. Used by cut(a−b), which turns B's kept fragments
  /// inside-out.
  void flip() {
    std::reverse(vertices.begin(), vertices.end());
    plane.normal = -plane.normal;
    plane.w = -plane.w;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Face → Polygon extraction.
//
// A native planar face is a Plane surface bounded by an outer wire (child 0). The
// boolean needs the face as a simple world-space loop with an outward normal, so
// we:
//   1. walk the outer wire's vertices in traversal order (ordered_loop below),
//   2. take the face's stored plane normal, and
//   3. flip that normal when the face's EFFECTIVE orientation is Reversed, so the
//      stored polygon normal is always the material-outward one (front = outside).
//
// Fan independence: build_prism gives each face its OWN edges/vertices, so a face's
// loop is self-contained — we do NOT need pcurves or edge sharing here. We collect
// the ordered vertices by taking, for each edge in the wire, the vertex at the
// edge's start in wire-traversal order.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

// Ordered world-space vertices of a wire, in traversal order. For each edge (in
// stored order) we append the world point of its start vertex; consecutive edges
// share an endpoint so the loop is the polygon boundary. Reversed edges contribute
// their end vertex as the "start" of traversal.
inline std::vector<math::Point3> orderedLoop(const topo::Shape& wire) {
  std::vector<math::Point3> pts;
  if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return pts;
  for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
    const topo::Shape& edge = ex.current();
    const auto& verts = edge.tshape()->children();
    if (verts.empty()) continue;
    // The traversal-start vertex of the edge within the wire: for a Forward edge
    // it is the Forward-stored vertex; for a Reversed edge the wire walks the edge
    // backward, so its traversal start is the Reversed-stored (end) vertex. We use
    // the edge's own orientation (already composed with the wire by the Explorer).
    const bool rev = edge.orientation() == topo::Orientation::Reversed;
    // Locate the vertex whose stored role matches the traversal start.
    const topo::Shape* start = nullptr;
    for (const topo::Shape& v : verts) {
      const bool isEnd = v.orientation() == topo::Orientation::Reversed;
      if (isEnd == rev) { start = &v; break; }
    }
    if (!start) start = &verts.front();
    // The vertex handle stored on the edge carries only the edge's location; the
    // world point must also fold the wire/face location the Explorer accumulated
    // onto the edge. Re-place the vertex under the edge's (already world) location.
    const topo::Shape placed = start->located(edge.location());
    if (auto p = topo::pointOf(placed)) pts.push_back(*p);
  }
  return pts;
}

}  // namespace detail

/// Emit one Polygon per planar face of `solid` (world coordinates, outward normal).
/// Faces with < 3 boundary vertices or a non-Plane surface are skipped (the caller
/// gates on isAllPlanar first, so a skipped face means a degenerate loop).
inline std::vector<Polygon> extractPolygons(const topo::Shape& solid) {
  std::vector<Polygon> out;
  for (topo::Explorer ex(solid, topo::ShapeType::Face); ex.more(); ex.next()) {
    const topo::Shape& face = ex.current();
    const auto surf = topo::surfaceOf(face);
    if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Plane) continue;

    const auto& wires = face.tshape()->children();
    if (wires.empty()) continue;
    // Outer wire (child 0), re-placed under the face's world location.
    const topo::Shape outer = wires.front().located(face.location());
    std::vector<math::Point3> loop = detail::orderedLoop(outer);
    if (loop.size() < 3) continue;

    // Stored plane normal (world). The face surface frame's Z is the plane normal
    // in local coords; apply the face location's linear part to world it.
    math::Vec3 n = surf->surface->frame.z.vec();
    if (!surf->location.isIdentity())
      n = surf->location.transform().applyToVector(n);
    // Fold the face's effective orientation: Reversed flips the material side.
    if (face.orientation() == topo::Orientation::Reversed) n = -n;
    const math::Dir3 nd{n};
    if (!nd.valid()) continue;

    // Ensure the loop winds CCW as seen from +normal (front). If the loop's own
    // area normal opposes the outward normal, reverse the loop so front = outside.
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i) {
      const math::Point3& a = loop[i];
      const math::Point3& b = loop[(i + 1) % loop.size()];
      area += math::cross(a.asVec(), b.asVec());
    }
    if (math::dot(area, nd.vec()) < 0.0)
      std::reverse(loop.begin(), loop.end());

    // Plane through an actual loop vertex for an exact offset.
    const Plane plane = Plane::fromPointNormal(loop.front(), nd.vec());
    out.emplace_back(std::move(loop), plane);
  }
  return out;
}

/// True iff every face of `solid` is a planar face (the native boolean's domain).
/// A solid with any Cylinder/Cone/Sphere/free-form face returns false → the engine
/// falls through to OCCT.
inline bool isAllPlanar(const topo::Shape& solid) {
  bool sawFace = false;
  for (topo::Explorer ex(solid, topo::ShapeType::Face); ex.more(); ex.next()) {
    sawFace = true;
    const auto surf = topo::surfaceOf(ex.current());
    if (!surf) return false;
    if (surf->surface->kind != topo::FaceSurface::Kind::Plane) return false;
  }
  return sawFace;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_POLYGON_H

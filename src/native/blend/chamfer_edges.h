// SPDX-License-Identifier: Apache-2.0
//
// chamfer_edges.h — native planar chamfer on a convex edge between two planar
// faces (Phase 4 #6 `native-blends`, openspec/NATIVE-REWRITE.md).
//
// A chamfer cuts the sharp corner off a convex edge: face 1 and face 2 are each set
// back by `distance` (measured in their own plane, perpendicular to the edge) and a
// new PLANAR chamfer face bridges the two setback lines. Geometrically this is
// exactly slicing the solid with the single plane through both setback lines and
// discarding the corner wedge — a boolean cut with a PLANAR cutter, as
// NATIVE-REWRITE.md notes.
//
// IMPLEMENTATION (planar-polygon clip). The solid is flattened to oriented planar
// polygons (blend_geom.h). For a picked convex edge we compute the chamfer plane
// (through the two setback lines) with its normal pointing OUT of the material, and:
//   1. clip EVERY polygon to the material side of that plane (Sutherland–Hodgman) —
//      this sets back the two adjacent faces and leaves every other face intact;
//   2. the clip opens a planar hole bounded by the setback lines; assembleSolid's
//      T-junction repair + welded triangulation closes it, so the new planar chamfer
//      face is created implicitly by the boundary the clip exposes. To make that
//      face explicit and guarantee closure we add the chamfer cross-section polygon
//      directly (the ordered ring of clip-crossing points on that plane).
// Multiple edges are chamfered by applying the clip successively (each on the result
// of the previous). The engine self-verify (watertight + volume reduced) accepts the
// result or falls through to OCCT.
//
// ASYMMETRIC (two-distance) chamfer. `chamfer_edges_asym` sets face 1 back by `d1`
// and face 2 by `d2` (each measured in its own plane, ⊥ the crease) — the standard
// CAD "chamfer with two distances / distance+angle" cut. The bridging plane still
// passes through the two edge-parallel setback lines; because the setbacks differ,
// the plane is OBLIQUE (its normal tilts toward the smaller-setback face). The
// symmetric `chamfer_edges(…, distance)` is exactly the `d1==d2` case, and mirrors
// the curved analog `curved_chamfer_edge_asym` (curved_chamfer.h). Removed volume
// for a box corner = ½·d1·d2·L (a right-triangle prism of legs d1,d2 over length L).
//
// SCOPE (honest). Native only for a CONVEX edge shared by exactly TWO PLANAR faces,
// with a chamfer plane that does not run past either face's extent. A concave edge,
// a curved adjacent face, an edge shared by ≠2 faces, or a setback larger than a
// face → NULL for that edge → the whole op returns NULL → OCCT (BRepFilletAPI
// MakeChamfer) fallthrough. Likely EXACT vs OCCT for a box corner. The
// larger-than-a-face setback is rejected UP FRONT by an on-face guard (each
// setback point must lie within its finite face) so an oversized distance
// HONEST-DECLINES rather than clipping past the far edge and welding a
// silently-wrong (though watertight, volume-reduced) body.
//
// CLEAN-ROOM. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CHAMFER_EDGES_H
#define CYBERCAD_NATIVE_BLEND_CHAMFER_EDGES_H

#include "native/blend/blend_geom.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// In-plane inward direction of face `f` (outward normal nf) perpendicular to the
// crease direction `t`: the unit vector lying in the face, orthogonal to the edge,
// pointing AWAY from the edge into the face interior. inward = normalize(t × nf)
// oriented so it points toward the face's centroid.
inline math::Vec3 faceInward(const math::Vec3& t, const math::Vec3& nf, const math::Point3& edgeMid,
                             const math::Point3& faceCentroid) {
  math::Vec3 d = math::cross(t, nf);
  const double len = math::norm(d);
  if (len < kBlendEps) return {0, 0, 0};
  d = d / len;
  if (math::dot(d, faceCentroid - edgeMid) < 0.0) d = -d;
  return d;
}

inline math::Point3 centroidOf(const std::vector<math::Point3>& loop) {
  if (loop.empty()) return {};
  math::Vec3 c{0, 0, 0};
  for (const math::Point3& p : loop) c += p.asVec();
  const double inv = 1.0 / static_cast<double>(loop.size());
  return math::Point3{c.x * inv, c.y * inv, c.z * inv};
}

// On-face guard: is point `p` inside the (convex) planar face `f`, allowing a small
// world-unit boundary `slack`? Self-contained convex winding test — the signed cross
// of each boundary edge against the face normal must share one sign for every edge (p
// on the interior side of all edges). A single opposite sign (beyond slack), OR a
// non-convex face, reports "outside" so the caller conservatively DECLINES (→ OCCT),
// never welding a chamfer whose setback line has left the face. This is what makes an
// oversized `distance` honest-decline instead of clipping past the far face edge.
inline bool chamferPointOnFace(const nb::Polygon& f, const math::Point3& p, double slack) {
  const std::size_t m = f.vertices.size();
  if (m < 3) return false;
  const math::Vec3 nf = f.plane.normal;
  int sign = 0;
  for (std::size_t i = 0; i < m; ++i) {
    const math::Point3& a = f.vertices[i];
    const math::Point3& b = f.vertices[(i + 1) % m];
    const math::Vec3 inwardN = math::cross(nf, b - a);  // in-plane, ⊥ the edge
    const double len = math::norm(inwardN);
    if (len < kBlendEps) continue;  // degenerate edge — skip, don't false-reject
    const double d = math::dot(inwardN, p - a) / len;  // signed world distance to edge
    if (d > slack) {
      if (sign < 0) return false;
      sign = 1;
    } else if (d < -slack) {
      if (sign > 0) return false;
      sign = -1;
    }
  }
  return true;
}

// The chamfer plane for a convex edge (endpoints ea,eb) between faces f1,f2. Its
// setback points are p1 = edgeMid + d1·inward1, p2 = edgeMid + d2·inward2 (face 1 set
// back by d1, face 2 by d2); the plane passes through the edge-parallel setback lines,
// i.e. through p1, p2 and the crease direction t. Normal = t × (p2 − p1), oriented
// OUTWARD (away from the material, toward the removed corner). Returns nullopt if the
// config is degenerate, the edge is CONCAVE (the two inward dirs make the corner
// reflex), or EITHER setback point overruns its finite face (the on-face guard, so an
// oversized distance honest-declines rather than welding a silently-wrong body).
inline std::optional<nb::Plane> chamferPlane(const math::Point3& ea, const math::Point3& eb,
                                             const nb::Polygon& f1, const nb::Polygon& f2,
                                             double d1, double d2) {
  const auto tOpt = creaseDir(ea, eb);
  if (!tOpt) return std::nullopt;
  const math::Vec3 t = tOpt->vec();
  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};

  const math::Vec3 in1 = faceInward(t, f1.plane.normal, mid, centroidOf(f1.vertices));
  const math::Vec3 in2 = faceInward(t, f2.plane.normal, mid, centroidOf(f2.vertices));
  if (math::isNull(in1) || math::isNull(in2)) return std::nullopt;

  // Convexity guard: the outward corner direction is n1 + n2; it must point away
  // from the material. For a convex edge each inward dir points opposite the other
  // face's outward normal component. Reject a reflex (concave) corner.
  const math::Vec3 cornerOut = f1.plane.normal + f2.plane.normal;
  if (math::dot(cornerOut, in1) > -kBlendEps || math::dot(cornerOut, in2) > -kBlendEps)
    return std::nullopt;  // not a clean convex corner

  const math::Point3 p1 = mid + in1 * d1;
  const math::Point3 p2 = mid + in2 * d2;

  // On-face guard: each setback line (through pk parallel to t) must lie within its
  // finite face — check its midpoint pk. A distance larger than the face's ⊥ extent
  // pushes pk past the far edge; declining here (→ OCCT) is honest, versus letting the
  // clip run past the face and welding a wrong body the self-verify might still accept.
  const double slack = 1e-9;
  if (!chamferPointOnFace(f1, p1, slack) || !chamferPointOnFace(f2, p2, slack))
    return std::nullopt;

  math::Vec3 n = math::cross(t, p2 - p1);
  const math::Dir3 nd{n};
  if (!nd.valid()) return std::nullopt;
  // Orient outward: away from the material (same side as the removed corner). The
  // corner tip (the edge itself) must be on the +normal side of the chamfer plane.
  nb::Plane pl = nb::Plane::fromPointNormal(p1, nd.vec());
  if (signedDist(pl, mid) < 0.0) { pl.normal = -pl.normal; pl.w = -pl.w; }
  return pl;
}

// Apply one chamfer plane to a polygon soup: clip every polygon to the material
// (below, signedDist ≤ 0) side and collect the exposed cross-section ring, then add
// the chamfer face. Returns the new soup, or empty on degeneracy.
inline std::vector<nb::Polygon> applyCut(const std::vector<nb::Polygon>& in, const nb::Plane& cut) {
  std::vector<nb::Polygon> out;
  out.reserve(in.size() + 1);
  std::vector<math::Point3> ring;  // crossing points → the chamfer cross-section
  for (const nb::Polygon& poly : in) {
    std::vector<math::Point3> clipped = clipBelow(poly.vertices, cut, kBlendEps);
    // Record newly-created boundary points (those lying ON the cut plane).
    for (const math::Point3& v : clipped)
      if (std::fabs(signedDist(cut, v)) < 1e-6) ring.push_back(v);
    if (clipped.size() >= 3) out.emplace_back(std::move(clipped), poly.plane);
  }
  if (ring.size() < 3) return {};

  // Order the ring points around the chamfer plane and add ONE chamfer face
  // (outward normal = the cut plane's normal). assembleSolid re-triangulates it.
  const math::Dir3 nz{cut.normal};
  const math::Vec3 refv = std::fabs(nz.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(ring.front(), nz, math::Dir3{refv});
  const math::Point3 c = centroidOf(ring);
  std::sort(ring.begin(), ring.end(), [&](const math::Point3& p, const math::Point3& q) {
    const double ap = std::atan2(math::dot(p - c, frame.y.vec()), math::dot(p - c, frame.x.vec()));
    const double aq = std::atan2(math::dot(q - c, frame.y.vec()), math::dot(q - c, frame.x.vec()));
    return ap < aq;
  });
  // Dedup near-coincident ring points.
  std::vector<math::Point3> face;
  for (const math::Point3& p : ring)
    if (face.empty() || math::distance(face.back(), p) > 1e-6) face.push_back(p);
  if (!face.empty() && math::distance(face.front(), face.back()) < 1e-6) face.pop_back();
  if (face.size() < 3) return {};
  out.emplace_back(face, nb::Plane::fromPointNormal(face.front(), cut.normal));
  return out;
}

}  // namespace detail

// Asymmetric chamfer: set the FIRST face on each picked edge back by `d1` and the
// SECOND by `d2` (each in its own plane, ⊥ the crease). The "first" face is the one
// facesOnEdgeInSoup returns first for that edge in the current soup; for a symmetric
// chamfer (d1==d2) the order is immaterial. Returns the chamfered solid, or a NULL
// Shape on any out-of-domain edge (non-convex / non-planar / ≠2 faces / oversized
// setback / degenerate input) → OCCT fallthrough.
inline topo::Shape chamfer_edges_asym(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                      double d1, double d2) {
  if (edgeIds == nullptr || edgeCount <= 0 || !(d1 > kBlendEps) || !(d2 > kBlendEps)) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  std::vector<nb::Polygon> polys = model.polygons();
  for (int i = 0; i < edgeCount; ++i) {
    // Resolve the edge on the ORIGINAL solid (ids are stable to the input body).
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) return {};

    // Find the two faces on this edge in the CURRENT soup (adjacency by edge key).
    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, ends->a, ends->b, faces) != 2) return {};

    const auto plane = detail::chamferPlane(ends->a, ends->b, polys[faces[0]], polys[faces[1]],
                                            d1, d2);
    if (!plane) return {};
    std::vector<nb::Polygon> next = detail::applyCut(polys, *plane);
    if (next.empty()) return {};
    polys = std::move(next);
  }
  return nb::assembleSolid(polys);
}

// Chamfer the convex planar-dihedral edges `edgeIds` (1-based, mapShapes order) of
// `solid` by `distance` (SYMMETRIC — both faces set back equally). Returns the
// chamfered solid, or a NULL Shape if any picked edge is not a convex edge between two
// planar faces / the solid is not planar / input is degenerate (→ OCCT fallthrough).
inline topo::Shape chamfer_edges(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                 double distance) {
  return chamfer_edges_asym(solid, edgeIds, edgeCount, distance, distance);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CHAMFER_EDGES_H

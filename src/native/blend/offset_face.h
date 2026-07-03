// SPDX-License-Identifier: Apache-2.0
//
// offset_face.h — native offset of ONE planar face along its outward normal
// (Phase 4 #6 `native-blends`). Grows (distance > 0) or shrinks (distance < 0) the
// solid by sliding the picked planar face along its own normal by `distance` and
// dragging the adjacent side faces with it.
//
// APPROACH (planar-polygon edit). The solid is flattened to its oriented planar
// polygons (blend_geom.h PlanarModel). The picked face is translated by
// distance·n̂; every OTHER face that shares a vertex with the picked face has that
// shared corner translated by the SAME distance·n̂. For a prism-like solid (the
// tractable case) the picked face is a cap whose ring vertices are shared with the
// side quads, so moving the cap slides the side faces to follow — exactly a slab
// grow/shrink. The re-assembled solid is welded + triangulated by the boolean's
// assembleSolid, so it meshes watertight; the engine self-verify then checks the
// volume grew (distance>0) / shrank (distance<0) as expected, else falls to OCCT.
//
// SCOPE. Native only when the picked face is PLANAR and the whole solid is planar-
// faced (a curved adjacent face cannot follow a planar translation cleanly). A
// curved solid / non-planar picked face → NULL → OCCT (BRepOffsetAPI) fallthrough.
// A shrink that would collapse the solid (|distance| ≥ the opposite extent) is left
// to the self-verify to reject.
//
// CLEAN-ROOM. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_OFFSET_FACE_H
#define CYBERCAD_NATIVE_BLEND_OFFSET_FACE_H

#include "native/blend/blend_geom.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// Which polygon carries the picked face plane and matches its supporting plane the
// closest (a solid can have several coplanar faces; the picked one is the polygon
// whose every vertex lies ON the plane and whose outward normal agrees). Returns
// npos if none matches.
inline std::size_t pickFacePolygon(const std::vector<nb::Polygon>& polys, const nb::Plane& plane) {
  std::size_t best = static_cast<std::size_t>(-1);
  for (std::size_t i = 0; i < polys.size(); ++i) {
    const nb::Polygon& p = polys[i];
    if (math::dot(p.plane.normal, plane.normal) < 0.999) continue;  // normal must agree
    bool allOn = true;
    for (const math::Point3& v : p.vertices)
      if (std::fabs(signedDist(plane, v)) > 1e-5) { allOn = false; break; }
    if (allOn) return i;
  }
  return best;
}

}  // namespace detail

// Offset the planar face `faceId` (1-based, mapShapes order) of `solid` outward
// along its normal by `distance`. Returns the grown/shrunk solid, or a NULL Shape
// for a non-planar solid / face or degenerate input (→ OCCT fallthrough).
inline topo::Shape offset_face(const topo::Shape& solid, int faceId, double distance) {
  if (std::fabs(distance) < kBlendEps) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  const auto planeOpt = facePlane(solid, faceId);
  if (!planeOpt) return {};
  const nb::Plane plane = *planeOpt;

  const std::size_t pick = detail::pickFacePolygon(model.polygons(), plane);
  if (pick == static_cast<std::size_t>(-1)) return {};

  // The set of picked-face corner positions; every coincident corner (on ANY face)
  // slides by distance·n̂. Using position identity (quantized) shares the moved ring
  // between the cap and its side faces so the slab stays closed.
  const math::Vec3 shift = plane.normal * distance;
  const std::vector<math::Point3>& pickVerts = model.polygons()[pick].vertices;

  auto isPicked = [&](const math::Point3& p) {
    for (const math::Point3& q : pickVerts)
      if (math::distance(p, q) < 1e-6) return true;
    return false;
  };

  std::vector<nb::Polygon> out;
  out.reserve(model.polygons().size());
  for (const nb::Polygon& poly : model.polygons()) {
    std::vector<math::Point3> loop;
    loop.reserve(poly.vertices.size());
    for (const math::Point3& v : poly.vertices)
      loop.push_back(isPicked(v) ? v + shift : v);
    // The plane offset shifts with the moved vertices only for the picked face
    // (its whole loop translated); side faces keep their own plane (recomputed by
    // assembleSolid's triangulation from the moved corners). Recompute the plane
    // from the (possibly moved) first vertex + original normal so it stays exact.
    nb::Plane pl = poly.plane;
    pl.w = math::dot(pl.normal, loop.front().asVec());
    out.emplace_back(std::move(loop), pl);
  }
  return nb::assembleSolid(out);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_OFFSET_FACE_H

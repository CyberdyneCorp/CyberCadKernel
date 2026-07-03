// SPDX-License-Identifier: Apache-2.0
//
// shell.h — native uniform-thickness hollow (Phase 4 #6 `native-blends`,
// openspec/NATIVE-REWRITE.md). Removes the selected faces and hollows the solid to a
// uniform wall of `thickness`, leaving the removed faces open.
//
// ── APPROACH (offset + boolean, as NATIVE-REWRITE.md prescribes) ──────────────────
// Expressed as: result = outer − cavity, where the CAVITY is the solid inset inward
// by `thickness` on every wall that STAYS, and flush with the outer face on every
// REMOVED wall (so cutting the cavity out opens those faces). For a convex planar
// solid (a box / convex prism — the tractable case) the cavity is the intersection
// of half-spaces:
//   * a kept face plane, moved inward by `thickness` (w −= thickness along +normal),
//   * a removed face plane, kept at its original position (the opening).
// We build the cavity by clipping the outer polygon soup successively against each
// inward half-space (blend_geom.h clipBelow + a cross-section cap per clip, reusing
// the chamfer applyCut tiling), then run the native planar BSP-CSG CUT
// (boolean_solid) of outer − cavity. The result is welded + verified watertight by
// the same self-verify the engine runs for any boolean, so a wall that would be
// thicker than the solid (cavity empty) or a non-convex case that the half-space
// intersection misrepresents is caught and falls through to OCCT.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native for a CONVEX planar solid (box / convex prism) with `thickness` smaller than
// half the smallest wall-to-wall extent, removing one or more of its planar faces. A
// curved solid, a non-convex solid (the half-space cavity would over/under-cut a
// concave pocket), a thickness too large, or a removed face that is not planar → NULL
// → OCCT (BRepOffsetAPI_MakeThickSolid) fallthrough.
//
// CLEAN-ROOM. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_SHELL_H
#define CYBERCAD_NATIVE_BLEND_SHELL_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"  // detail::applyCut
#include "native/boolean/native_boolean.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// True if the picked face plane (world) matches polygon `p` (coplanar, same outward
// normal, all vertices on the plane) — the same test offset_face uses.
inline bool polygonMatchesPlane(const nb::Polygon& p, const nb::Plane& plane) {
  if (math::dot(p.plane.normal, plane.normal) < 0.999) return false;
  for (const math::Point3& v : p.vertices)
    if (std::fabs(signedDist(plane, v)) > 1e-5) return false;
  return true;
}

}  // namespace detail

// Hollow `solid` to a uniform wall `thickness`, opening the faces `faceIds` (1-based,
// mapShapes order). Returns the shelled solid, or a NULL Shape for a non-planar /
// non-convex solid, a thickness too large, or degenerate input (→ OCCT fallthrough).
inline topo::Shape shell(const topo::Shape& solid, const int* faceIds, int faceCount,
                         double thickness) {
  if (!(thickness > kBlendEps)) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  // Resolve the removed face planes.
  std::vector<nb::Plane> removed;
  if (faceIds != nullptr) {
    for (int i = 0; i < faceCount; ++i) {
      const auto pl = facePlane(solid, faceIds[i]);
      if (!pl) return {};  // non-planar picked face → OCCT
      removed.push_back(*pl);
    }
  }
  auto isRemoved = [&](const nb::Polygon& p) {
    for (const nb::Plane& rp : removed)
      if (detail::polygonMatchesPlane(p, rp)) return true;
    return false;
  };

  // Build the cavity by clipping the outer soup against each KEPT face's inward
  // half-space (plane moved inward by `thickness`). The removed faces are NOT clipped,
  // so the cavity reaches flush to them → cutting it out opens those faces.
  std::vector<nb::Polygon> cavity = model.polygons();
  for (const nb::Polygon& face : model.polygons()) {
    if (isRemoved(face)) continue;
    // Inward plane: same outward normal, offset moved inward by `thickness`.
    nb::Plane inward = face.plane;
    inward.w -= thickness;                     // material side is signedDist ≤ 0
    inward.normal = face.plane.normal;         // keep outward normal
    std::vector<nb::Polygon> next = detail::applyCut(cavity, inward);
    if (next.empty()) return {};               // thickness too large → cavity collapsed
    cavity = std::move(next);
  }

  const topo::Shape cavitySolid = nb::assembleSolid(cavity);
  if (cavitySolid.isNull()) return {};

  // result = outer − cavity (native planar BSP-CSG cut). The engine self-verify will
  // confirm the wall volume (outer − cavity) is watertight and positive.
  return nb::boolean_solid(solid, cavitySolid, nb::Op::Cut);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_SHELL_H

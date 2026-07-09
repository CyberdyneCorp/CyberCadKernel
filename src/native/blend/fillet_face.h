// SPDX-License-Identifier: Apache-2.0
//
// fillet_face.h — native `cc_fillet_face`: round EVERY edge bounding a picked
// planar face at constant radius (MOAT M3, `moat-m3af-analytic-fillet`).
//
// ── WHAT THIS IS ──────────────────────────────────────────────────────────────
// The OCCT adapter this mirrors (OcctEngine::fillet_face) rounds every edge of the
// picked face with BRepFilletAPI_MakeFillet::Add(r, edge). For a native ALL-PLANAR
// solid that is exactly the landed multi-edge rolling-ball fillet
// (blend/fillet_edges.h) applied to the CONVEX planar-dihedral edges bounding the
// face: each such edge is replaced by a tangent cylinder of radius r, G1 to both
// adjacent planes, welded watertight through the SAME assembleSolid path a native
// prism / boolean uses (open-seam weld, already robust).
//
// ── ALGORITHM (assembly of the landed dihedral fillet — NO new geometry) ─────────
//   1. Resolve faceId → its outward plane (facePlane; guards planar).
//   2. Build the global 1-based edge map (mapShapes(Edge)) and, for each edge of the
//      picked face, its id.
//   3. Keep an edge only if it is a CONVEX planar dihedral for which a radius-r
//      rolling-ball tangent cylinder fits both adjacent faces — probed with the
//      SAME detail::filletArc the dihedral fillet uses (so the selection can never
//      admit a concave / curved / non-two-face / oversized edge).
//   4. Call blend::fillet_edges(solid, keptIds, r) — the byte-frozen tangent-
//      cylinder blend — and return its result.
// The engine then runs its mandatory SHRINK self-verify (0 < Vr < Vo — a face
// fillet REMOVES material) and DISCARDS a bad result → OCCT.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────
// Native only when the solid is all-planar, the picked face is planar, and it has at
// least one convex planar-dihedral bounding edge that fits radius r. A non-planar
// face, a non-all-planar solid, no convex bounding edge, or an interfering/oversized
// radius → NULL → OCCT (BRepFilletAPI_MakeFillet) fallthrough. Deflection-bounded vs
// OCCT.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean + blend/fillet_edges.
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_FACE_H
#define CYBERCAD_NATIVE_BLEND_FILLET_FACE_H

#include "native/blend/blend_geom.h"
#include "native/blend/fillet_edges.h"

#include <vector>

namespace cybercad::native::blend {

// Why fillet_face declined (diagnostic; the engine maps NULL → honest error → OCCT).
enum class FilletFaceDecline {
  Ok = 0,
  BadInput,        // null/degenerate radius or solid
  NonPlanarSolid,  // the solid carries a curved face
  NonPlanarFace,   // the picked face is not a plane
  NoConvexEdges,   // no bounding edge is a convex planar dihedral fitting radius r
  WeldGatesM2,     // edges found but the multi-edge weld failed (corner-sphere patch
                   // between adjacent edge-fillets gates on M2) → OCCT this wave
};

namespace detail {

// The 1-based ids (mapShapes(Edge) order) of the edges bounding face `faceId` that
// are CONVEX planar dihedrals fitting a radius-`r` rolling ball. An edge is kept
// only when detail::filletArc succeeds on the two faces meeting at it in the current
// polygon soup — the identical guard the dihedral fillet uses, so a concave / curved
// / ≠2-face / oversized edge is silently skipped (never faked).
inline std::vector<int> convexBoundingEdges(const topo::Shape& solid, int faceId,
                                            const std::vector<nb::Polygon>& polys, double r) {
  std::vector<int> ids;
  const topo::ShapeMap faceMap = topo::mapShapes(solid, topo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > faceMap.size()) return ids;
  const topo::ShapeMap edgeMap = topo::mapShapes(solid, topo::ShapeType::Edge);

  const topo::Shape& face = faceMap.shape(faceId);
  for (topo::Explorer ex(face, topo::ShapeType::Edge); ex.more(); ex.next()) {
    const int eid = edgeMap.findIndex(ex.current());
    if (eid < 1) continue;
    const auto ends = edgeEnds(solid, eid);
    if (!ends) continue;
    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, ends->a, ends->b, faces) != 2) continue;
    const auto arc = filletArc(ends->a, ends->b, polys[faces[0]], polys[faces[1]], r);
    if (!arc) continue;  // concave / near-flat / oversized → skip (OCCT owns it)
    // Dedup (an explorer can surface the same edge twice through shared wires).
    bool seen = false;
    for (int have : ids)
      if (have == eid) { seen = true; break; }
    if (!seen) ids.push_back(eid);
  }
  return ids;
}

}  // namespace detail

// Round every convex planar-dihedral edge bounding the picked planar face `faceId`
// (1-based, mapShapes(Face) order) of `solid` at constant `radius`. Returns the
// filleted solid, or a NULL Shape (with *why set) when the input is out of the
// analytic domain → OCCT fallthrough. `deflection` bounds the facet chord error.
inline topo::Shape fillet_face(const topo::Shape& solid, int faceId, double radius,
                               double deflection = 0.01,
                               FilletFaceDecline* why = nullptr) {
  auto fail = [&](FilletFaceDecline d) {
    if (why) *why = d;
    return topo::Shape{};
  };
  if (why) *why = FilletFaceDecline::Ok;
  if (solid.isNull() || !(radius > kBlendEps)) return fail(FilletFaceDecline::BadInput);

  PlanarModel model(solid);
  if (!model.isValid()) return fail(FilletFaceDecline::NonPlanarSolid);

  // Guard the picked face is planar (facePlane is nullopt for a curved face).
  if (!facePlane(solid, faceId)) return fail(FilletFaceDecline::NonPlanarFace);

  const std::vector<int> edgeIds =
      detail::convexBoundingEdges(solid, faceId, model.polygons(), radius);
  if (edgeIds.empty()) return fail(FilletFaceDecline::NoConvexEdges);

  // Reuse the byte-frozen multi-edge tangent-cylinder blend (open-seam weld). A NULL
  // from it (interference / self-verify) propagates as the same honest decline.
  topo::Shape result =
      fillet_edges(solid, edgeIds.data(), static_cast<int>(edgeIds.size()), radius, deflection);
  if (result.isNull()) return fail(FilletFaceDecline::WeldGatesM2);
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_FACE_H

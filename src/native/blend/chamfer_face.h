// SPDX-License-Identifier: Apache-2.0
//
// chamfer_face.h — MOAT M2 FULL-FACE chamfer weld: chamfer EVERY convex
// planar-dihedral edge bounding a picked planar face at constant setback, in ONE
// watertight solid (the planar sibling of `fillet_face`).
//
// ── ROLE (the additive full-face verb the corner weld unblocks) ───────────────────
// `fillet_face.h` rounds every convex edge bounding a picked face; its chamfer twin
// has no landed native path because the byte-frozen SEQUENTIAL `chamfer_edges` DECLINES
// a corner-sharing edge loop (the first edge's set-back removes the shared corner, so
// the next edge is lost from the soup — `facesOnEdgeInSoup == 0` → NULL → OCCT). A
// planar face's outer edge loop is EXACTLY such a corner-sharing set: consecutive
// bounding edges meet at the face's corners.
//
// The landed M2 convex-corner weld `chamfer_corner` (corner_chamfer_weld.h) resolves
// EVERY chamfer plane UP FRONT on the original soup and applies all clips together, so
// a corner-sharing edge loop welds watertight. Crucially, at EVERY corner of ONE face's
// edge loop exactly TWO picked edges meet (the two bounding edges incident to that
// corner) — never a TRIPLE (the third edge through that vertex runs off the OTHER,
// unpicked faces). So a full-face chamfer is a pure set of 2-edge DIHEDRAL corners,
// which `chamfer_corner` welds AND which matches OCCT `BRepFilletAPI_MakeChamfer` to
// fp64 (the landed corner weld's own gate: a 2-edge dihedral corner is a union of two
// setback half-space prisms, reproduced exactly by OCCT). No triple corner ever arises
// from a single face's loop, so the oracle-gap decline `chamfer_corner` carries for a
// triple corner is unreachable here.
//
// ── ALGORITHM (assembly of the landed corner weld — NO new geometry) ──────────────
//   1. Guard: `solid` all-planar (PlanarModel) and the picked face planar (facePlane).
//   2. Build the global 1-based edge map and, for each edge of the picked face, keep it
//      only if it is a CONVEX planar dihedral for which a radius-`distance` rolling-ball
//      set-back fits — probed with the SAME `detail::filletArc` guard `fillet_face` uses
//      (a shared, conservative convexity/fit predicate that admits only a clean convex
//      2-planar-face edge). A concave / curved-neighbour / ≠2-face / oversized edge is
//      silently skipped — the OCCT-owned residue.
//   3. Call `chamfer_corner(solid, keptIds, distance)` — the byte-frozen corner weld —
//      and return its result. It runs the mandatory watertight + SHRINK self-verify and
//      DECLINES (NULL → OCCT) otherwise.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native only when the solid is all-planar, the picked face is planar, and it has at
// least one convex planar-dihedral bounding edge that fits `distance`. A non-planar
// face, a non-all-planar solid, no convex bounding edge, an oversized setback, or a
// self-verify miss → NULL → OCCT (`BRepFilletAPI_MakeChamfer` adding every edge of the
// face). Chamfer geometry is EXACT (planar), so the native result matches OCCT to
// machine ε on the full-face loop. No tolerance is widened.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `blend_geom.h` (`PlanarModel`/`facePlane`/`edgeEnds`/`facesOnEdgeInSoup`),
// `fillet_edges.h` (`detail::filletArc` fit guard), `corner_chamfer_weld.h`
// (`chamfer_corner`). Additive sibling — touches NONE of them, nor `chamfer_edges`'s own
// sequential entry point, nor `fillet_face`.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BLEND_CHAMFER_FACE_H
#define CYBERCAD_NATIVE_BLEND_CHAMFER_FACE_H

#include "native/blend/blend_geom.h"
#include "native/blend/corner_chamfer_weld.h"
#include "native/blend/fillet_edges.h"  // detail::filletArc (shared convex-fit guard)

#include <cstddef>
#include <vector>

namespace cybercad::native::blend {

// Why chamfer_face declined (diagnostic; NULL → OCCT). `Ok` iff a verified watertight
// result solid is returned.
enum class ChamferFaceDecline {
  Ok = 0,
  BadInput,        ///< null/degenerate distance or solid
  NonPlanarSolid,  ///< the solid carries a curved face (not the planar domain)
  NonPlanarFace,   ///< the picked face is not a plane
  NoConvexEdges,   ///< no bounding edge is a convex planar dihedral fitting `distance`
  WeldFailed,      ///< edges found but the corner weld failed its watertight/shrink verify
};

inline const char* chamferFaceDeclineName(ChamferFaceDecline d) noexcept {
  switch (d) {
    case ChamferFaceDecline::Ok: return "Ok";
    case ChamferFaceDecline::BadInput: return "BadInput";
    case ChamferFaceDecline::NonPlanarSolid: return "NonPlanarSolid";
    case ChamferFaceDecline::NonPlanarFace: return "NonPlanarFace";
    case ChamferFaceDecline::NoConvexEdges: return "NoConvexEdges";
    case ChamferFaceDecline::WeldFailed: return "WeldFailed";
  }
  return "?";
}

namespace detail {

// The 1-based ids (mapShapes(Edge) order) of the edges bounding face `faceId` that are
// CONVEX planar dihedrals for which a setback-`d` chamfer fits. An edge is kept only
// when `detail::filletArc` succeeds on the two faces meeting at it in the current
// polygon soup — the identical guard `fillet_face` uses — so a concave / curved / ≠2-face
// / oversized edge is silently skipped (never faked). (`filletArc`'s convexity + fit test
// is radius-independent for the accept/reject decision at these box/prism scales; it is
// the SAME predicate the corner weld's own `chamferPlane` accepts, so the kept set never
// admits an edge the weld would reject.)
inline std::vector<int> convexFaceEdges(const topo::Shape& solid, int faceId,
                                        const std::vector<nb::Polygon>& polys, double d) {
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
    const auto arc = filletArc(ends->a, ends->b, polys[faces[0]], polys[faces[1]], d);
    if (!arc) continue;  // concave / near-flat / oversized → skip (OCCT owns it)
    bool seen = false;
    for (int have : ids)
      if (have == eid) { seen = true; break; }
    if (!seen) ids.push_back(eid);
  }
  return ids;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// chamfer_face — chamfer every convex planar-dihedral edge bounding the picked planar
// face `faceId` (1-based, mapShapes(Face) order) of `solid` at constant `distance`, in
// ONE watertight solid. Returns the chamfered solid, or a NULL Shape (with *why set) →
// OCCT (`BRepFilletAPI_MakeChamfer` over every edge of the face) fall-through. NEVER a
// leaky / partial / wrong solid; no tolerance widened.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape chamfer_face(const topo::Shape& solid, int faceId, double distance,
                                ChamferFaceDecline* why = nullptr) {
  auto fail = [&](ChamferFaceDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (why) *why = ChamferFaceDecline::Ok;
  if (solid.isNull() || !(distance > kBlendEps)) return fail(ChamferFaceDecline::BadInput);

  PlanarModel model(solid);
  if (!model.isValid()) return fail(ChamferFaceDecline::NonPlanarSolid);

  // Guard the picked face is planar (facePlane is nullopt for a curved face).
  if (!facePlane(solid, faceId)) return fail(ChamferFaceDecline::NonPlanarFace);

  const std::vector<int> edgeIds =
      detail::convexFaceEdges(solid, faceId, model.polygons(), distance);
  if (edgeIds.empty()) return fail(ChamferFaceDecline::NoConvexEdges);

  // Reuse the byte-frozen convex-CORNER chamfer weld: a single face's edge loop is a set
  // of 2-edge DIHEDRAL corners (never a triple), so this welds watertight AND matches the
  // OCCT oracle exactly. A NULL from it (self-verify miss) is the same honest decline.
  const topo::Shape result =
      chamfer_corner(solid, edgeIds.data(), static_cast<int>(edgeIds.size()), distance);
  if (result.isNull()) return fail(ChamferFaceDecline::WeldFailed);
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CHAMFER_FACE_H

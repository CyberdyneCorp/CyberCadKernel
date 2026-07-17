// SPDX-License-Identifier: Apache-2.0
//
// corner_chamfer_weld.h — MOAT M2 CONVEX-CORNER chamfer weld: chamfer a set of
// mutually-ADJACENT convex planar-dihedral edges that share a common corner (2 or
// 3 edges meeting at a box/prism vertex) in ONE watertight solid.
//
// ── ROLE (the planar sibling of the M2 spherical corner weld) ─────────────────────
// The byte-frozen `chamfer_edges.h` chamfers edges SEQUENTIALLY: each edge's chamfer
// plane clips the whole polygon soup, then the NEXT edge is looked up in the ALREADY
// CLIPPED soup. That works for NON-adjacent edges (an opposite pair), but for two/three
// edges that share a CORNER the first clip removes the shared corner vertex, so the
// next edge is no longer present in the soup (`facesOnEdgeInSoup == 0`) and the whole
// op DECLINES to NULL → OCCT. (Measured: a 10³ box, edges {x,y} or {x,y,z} at one
// corner → `chamfer_edges` returns NULL; only single / opposite-pair edges land.)
//
// This ADDITIVE sibling closes that corner: it resolves EVERY picked edge and its
// chamfer plane UP FRONT against the ORIGINAL (un-clipped) soup — where the shared
// corner still exists — then applies ALL the clips. The mutual intersection of the
// chamfer planes near the corner is closed automatically by the boolean's exposed-ring
// face synthesis (`detail::applyCut`) + `assembleSolid`'s T-junction repair, which
// synthesises the CORNER FACET (the small planar polygon where the chamfer planes meet)
// with no extra geometry. This is the flat-facet analogue of the spherical corner patch
// the fillet corner needs — and being ALL-PLANAR it welds watertight through the SAME
// `assembleSolid` path with NO tessellator change.
//
// ── ALGORITHM (assembly of the byte-frozen chamfer verbs — NO new geometry) ───────
//   1. Flatten `solid` to the oriented planar polygon soup (`PlanarModel`).
//   2. For EACH picked edge id: resolve its world endpoints (`edgeEnds`, on the
//      original body) and the two faces on it IN THE ORIGINAL SOUP, then its outward
//      chamfer plane (`detail::chamferPlane`). Any non-convex / curved-neighbour /
//      ≠2-face / oversized edge → measured decline. All planes gathered before any clip.
//   3. Apply EACH chamfer plane's `detail::applyCut` in turn (each rebuilds its chamfer
//      face from the freshly-exposed ring — including where later planes cross earlier
//      chamfer faces, which is what forms the corner facet).
//   4. `assembleSolid` welds + triangulates + T-junction-repairs the soup.
//   5. MANDATORY self-verify (the way the engine does): the result meshes WATERTIGHT
//      and its volume is 0 < V < V(original) (a convex chamfer only REMOVES material).
//      ANY failure → NULL Shape → OCCT (`BRepFilletAPI_MakeChamfer`) fall-through.
//      NEVER a leaky / partial / wrong solid; no tolerance widened.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native only when the solid is all-planar and every picked edge is a CONVEX planar
// dihedral whose chamfer plane fits both its faces. The picked edges may share corners
// (that is the point). A concave edge, a curved neighbour, an edge on ≠2 faces, a
// setback larger than a face, or a self-verify miss → NULL → OCCT. Chamfer geometry is
// EXACT (planar), so the native result matches OCCT to machine ε.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `blend_geom.h` (`PlanarModel`/`edgeEnds`/`facesOnEdgeInSoup`), `chamfer_edges.h`
// (`detail::chamferPlane`/`detail::applyCut`), `boolean/assemble.h` (`assembleSolid`),
// the tessellate evaluators. Additive sibling — touches NONE of them, nor
// `chamfer_edges`'s own sequential `chamfer_edges` entry point.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BLEND_CORNER_CHAMFER_WELD_H
#define CYBERCAD_NATIVE_BLEND_CORNER_CHAMFER_WELD_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include <cstddef>
#include <vector>

namespace cybercad::native::blend {

namespace tess = cybercad::native::tessellate;

// Why a corner chamfer declined (diagnostic; the engine maps NULL → honest error →
// OCCT). `Ok` iff a verified watertight result solid is returned.
enum class CornerChamferDecline {
  Ok = 0,
  BadInput,        ///< null solid / no edges / non-positive distance
  NonPlanarSolid,  ///< the solid carries a curved face (not the planar domain)
  EdgeNotFound,    ///< a picked edge id is out of range / not a straight 2-vertex edge
  NotConvexEdge,   ///< an edge is not a convex dihedral between two planar faces
  CutFailed,       ///< a chamfer plane's clip exposed no usable cross-section ring
  AssembleFailed,  ///< fewer than four faces survived (no closed solid possible)
  NotWatertight,   ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent,  ///< self-verify: volume non-positive / not below the original
  TripleCornerOracleGap  ///< ≥3 picked edges share ONE vertex: OCCT's MakeChamfer breaks
                         ///< the triple corner into chamfer-chamfer facets that trim MORE
                         ///< than a plain 3-half-space clip (measured OCCT 985.667 vs the
                         ///< half-space 985.75 on a 10³ box, d=1). We cannot MATCH the OCCT
                         ///< oracle here, so DECLINE → OCCT (never a wrong solid).
};

inline const char* cornerChamferDeclineName(CornerChamferDecline d) noexcept {
  switch (d) {
    case CornerChamferDecline::Ok: return "Ok";
    case CornerChamferDecline::BadInput: return "BadInput";
    case CornerChamferDecline::NonPlanarSolid: return "NonPlanarSolid";
    case CornerChamferDecline::EdgeNotFound: return "EdgeNotFound";
    case CornerChamferDecline::NotConvexEdge: return "NotConvexEdge";
    case CornerChamferDecline::CutFailed: return "CutFailed";
    case CornerChamferDecline::AssembleFailed: return "AssembleFailed";
    case CornerChamferDecline::NotWatertight: return "NotWatertight";
    case CornerChamferDecline::VolumeInconsistent: return "VolumeInconsistent";
    case CornerChamferDecline::TripleCornerOracleGap: return "TripleCornerOracleGap";
  }
  return "?";
}

namespace detail {

// Reject a picked edge set in which ≥3 edges meet at ONE common vertex. At such a
// TRIPLE corner OCCT's BRepFilletAPI_MakeChamfer breaks the corner into chamfer-
// chamfer facets that trim MORE material than a plain intersection of the per-edge
// setback half-spaces (measured: OCCT 985.667 vs the half-space 985.75 on a 10³ box,
// d=1). This weld builds the half-space corner, which is watertight and EXACT vs OCCT
// through a DIHEDRAL corner (≤2 picked edges per vertex — verified 990.333 match), but
// does NOT reproduce the OCCT triple corner. So a triple corner honestly DECLINES to
// OCCT rather than emit a solid the oracle disagrees with. Endpoints are matched by the
// blend weld-grid vertex key (order-independent) over the resolved edge ends.
inline bool hasTripleSharedVertex(const topo::Shape& solid, const int* edgeIds, int edgeCount) {
  std::vector<math::Point3> pts;   // one entry per edge endpoint occurrence
  for (int i = 0; i < edgeCount; ++i) {
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) continue;
    pts.push_back(ends->a);
    pts.push_back(ends->b);
  }
  for (std::size_t i = 0; i < pts.size(); ++i) {
    int count = 0;
    for (std::size_t j = 0; j < pts.size(); ++j)
      if (math::distance(pts[i], pts[j]) <= kBlendEps) ++count;
    if (count >= 3) return true;  // ≥3 endpoint occurrences at one vertex ⇒ ≥3 edges meet
  }
  return false;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// chamfer_corner — chamfer a set of mutually-adjacent convex planar-dihedral edges
// sharing a corner (or any convex edge set) of `solid` by `distance`, in ONE
// watertight solid. `edgeIds` are 1-based mapShapes(Edge) ids on the input body.
// Returns the chamfered solid, or a NULL Shape (with *why set) → OCCT fall-through.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape chamfer_corner(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                  double distance, CornerChamferDecline* why = nullptr) {
  auto fail = [&](CornerChamferDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (why) *why = CornerChamferDecline::Ok;
  if (solid.isNull() || edgeIds == nullptr || edgeCount <= 0 || !(distance > kBlendEps))
    return fail(CornerChamferDecline::BadInput);

  PlanarModel model(solid);
  if (!model.isValid()) return fail(CornerChamferDecline::NonPlanarSolid);

  // A TRIPLE corner (≥3 picked edges at one vertex) does not match the OCCT oracle
  // (see hasTripleSharedVertex) — honest decline rather than a solid OCCT disagrees with.
  if (detail::hasTripleSharedVertex(solid, edgeIds, edgeCount))
    return fail(CornerChamferDecline::TripleCornerOracleGap);

  // (2) Resolve EVERY chamfer plane UP FRONT on the ORIGINAL soup — where the shared
  // corner vertices still exist, so an edge adjacent to an already-chamfered edge is
  // not lost (the sequential chamfer_edges' corner blocker).
  const std::vector<nb::Polygon>& original = model.polygons();
  std::vector<nb::Plane> planes;
  planes.reserve(static_cast<std::size_t>(edgeCount));
  for (int i = 0; i < edgeCount; ++i) {
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) return fail(CornerChamferDecline::EdgeNotFound);
    std::size_t faces[2];
    if (facesOnEdgeInSoup(original, ends->a, ends->b, faces) != 2)
      return fail(CornerChamferDecline::NotConvexEdge);
    const auto plane =
        detail::chamferPlane(ends->a, ends->b, original[faces[0]], original[faces[1]], distance,
                             distance);
    if (!plane) return fail(CornerChamferDecline::NotConvexEdge);
    planes.push_back(*plane);
  }

  // (3) Apply every chamfer plane in turn (later planes crossing earlier chamfer faces
  // form the corner facet from the exposed ring — no explicit corner geometry).
  std::vector<nb::Polygon> polys = original;
  for (const nb::Plane& pl : planes) {
    std::vector<nb::Polygon> next = detail::applyCut(polys, pl);
    if (next.empty()) return fail(CornerChamferDecline::CutFailed);
    polys = std::move(next);
  }

  // (4) weld + triangulate + T-junction repair.
  const topo::Shape result = nb::assembleSolid(polys);
  if (result.isNull()) return fail(CornerChamferDecline::AssembleFailed);

  // (5) mandatory self-verify: watertight AND 0 < V < V(original).
  tess::MeshParams mp;
  mp.deflection = 0.01;
  const tess::Mesh mResult = tess::SolidMesher(mp).mesh(result);
  if (!tess::isWatertight(mResult)) return fail(CornerChamferDecline::NotWatertight);
  const double v = std::fabs(tess::enclosedVolume(mResult));
  const tess::Mesh mOrig = tess::SolidMesher(mp).mesh(solid);
  const double v0 = std::fabs(tess::enclosedVolume(mOrig));
  const double tol = 1e-9 * std::max(v0, 1.0);
  if (!(v > tol) || std::isnan(v) || !(v < v0 - tol))
    return fail(CornerChamferDecline::VolumeInconsistent);

  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CORNER_CHAMFER_WELD_H

// SPDX-License-Identifier: Apache-2.0
//
// full_round.h — native full-round fillet: replace a narrow middle face with a
// full tangent round between its two neighbour walls, consuming the middle face
// (MOAT M3, `moat-m3af-analytic-fillet`).
//
// ── ANALYTIC PRISMATIC CASE (what lands native) ──────────────────────────────────
// The tractable family is a PRISMATIC rib: a planar middle strip M of width w capped
// between two PARALLEL planar walls L and R (nL·nR ≈ −1). A ball of radius r = w/2
// rolled along the strip is tangent to both walls; its crest lands on the middle
// plane, so as the ball rolls it exactly consumes M and leaves a half-cylinder cap
// G1-tangent to both walls at the two seam edges (M↔L, M↔R). This is the r = w/2
// SPECIAL CASE of the landed rolling-ball tangent-cylinder blend (blend/fillet_edges.h):
// rounding BOTH seam edges at radius w/2 makes the two arcs meet tangentially on the
// strip mid-plane and the middle face vanishes. So the native full round is simply
//     blend::fillet_edges(solid, {eL, eR}, r = w/2)
// welded through the SAME watertight assembleSolid path (open-seam weld, robust).
//
// ── ALGORITHM (assembly of the landed fillet — NO new geometry) ──────────────────
//   full_round_fillet_faces(L, M, R): seam edges = the longest shared edge of M∩L
//       and M∩R; w = perpendicular gap; require L ∥ R and prismatic (straight, equal-
//       length seams) → fillet both at w/2.
//   full_round_fillet(M): the two LONGEST edges of M are the seams; their across-
//       neighbours are L, R (edge→face ancestry) → as above.
// The engine then runs its mandatory SHRINK self-verify (0 < Vr < Vo — a convex full
// round REMOVES material) AND checks the middle face is gone; a bad result is
// DISCARDED → OCCT.
//
// ── HONEST DECLINE (freeform / dihedral / closed-seam → OCCT) ─────────────────────
//   * a DIHEDRAL (non-parallel) middle needs the nL×nR valley-solve + a two-seam
//     ARC-MERGE across the consumed face — the closed-seam weld that gates on M2;
//   * a CURVED wall has no constant-radius rolling ball;
//   * a CLOSED-SEAM / annulus full round (a circular boss top) needs the M2 closed-
//     seam weld;
//   * a non-planar middle / non-all-planar solid is out of the planar domain.
// Each returns NULL with a measured reason → OCCT BRepFilletAPI full-round oracle. A
// native void is NEVER handed to OCCT.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean + blend/fillet_edges.
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FULL_ROUND_H
#define CYBERCAD_NATIVE_BLEND_FULL_ROUND_H

#include "native/blend/blend_geom.h"
#include "native/blend/fillet_edges.h"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

// Why a full round declined (diagnostic; the engine maps NULL → honest error → OCCT).
enum class FullRoundDecline {
  Ok = 0,
  BadInput,       // null solid / bad ids
  NonPlanarSolid, // the solid carries a curved face
  NoSeams,        // could not identify two seam edges / neighbour walls
  NotParallel,    // the two walls are not parallel (dihedral — M2 valley-solve/merge)
  ClosedSeam,     // a closed-seam / annulus full round (M2 closed-seam weld)
};

namespace detail {

// A resolved prismatic full round: the two seam-edge ids (mapShapes(Edge) order) and
// the rolling-ball radius r = w/2 (w = the parallel-wall gap).
struct FullRoundPlan {
  int seamL = 0;
  int seamR = 0;
  double radius = 0.0;
};

// Squared length of an edge (by id).
inline double edgeLen2(const topo::Shape& solid, int edgeId) {
  const auto e = edgeEnds(solid, edgeId);
  if (!e) return 0.0;
  const math::Vec3 d = e->b - e->a;
  return math::dot(d, d);
}

// Does face `faceId` carry the undirected edge (a,b) on its boundary? Matched by
// vertex GEOMETRY (endpoint keys), NOT by isSame — a box stores a distinct edge node
// per incident face, so isSame ancestry cannot join them, but the endpoints coincide.
inline bool faceHasEdge(const topo::Shape& solid, const topo::ShapeMap& faceMap, int faceId,
                        const math::Point3& a, const math::Point3& b) {
  if (faceId < 1 || static_cast<std::size_t>(faceId) > faceMap.size()) return false;
  const EdgeKey want = edgeKeyOf(a, b);
  const topo::ShapeMap edgeMap = topo::mapShapes(solid, topo::ShapeType::Edge);
  for (topo::Explorer ex(faceMap.shape(faceId), topo::ShapeType::Edge); ex.more(); ex.next()) {
    const int eid = edgeMap.findIndex(ex.current());
    if (eid < 1) continue;
    const auto e = edgeEnds(solid, eid);
    if (e && edgeKeyOf(e->a, e->b) == want) return true;
  }
  return false;
}

// Outward plane of the planar face carrying the seam edge (a,b) OTHER than the middle
// face — found by endpoint geometry over the face map. nullopt if no other planar
// face carries it (curved neighbour / open edge).
inline std::optional<nb::Plane> neighbourWallPlane(const topo::Shape& solid,
                                                   const topo::ShapeMap& faceMap, int middleFaceId,
                                                   const math::Point3& a, const math::Point3& b) {
  for (std::size_t fid = 1; fid <= faceMap.size(); ++fid) {
    if (static_cast<int>(fid) == middleFaceId) continue;
    if (!faceHasEdge(solid, faceMap, static_cast<int>(fid), a, b)) continue;
    return facePlane(solid, static_cast<int>(fid));  // nullopt if that neighbour is curved
  }
  return std::nullopt;
}

// Build the prismatic full-round plan from the middle face + its two seam edges and
// the two neighbour walls. Requires the walls parallel (|nL·nR| ≈ 1) and the two
// seams straight + (near-)equal length (a prismatic strip). w = perpendicular gap
// between the seams; r = w/2. Returns nullopt with *why set otherwise.
inline std::optional<FullRoundPlan> planPrismatic(const topo::Shape& solid,
                                                  const topo::ShapeMap& faceMap, int middleFaceId,
                                                  int seamL, int seamR, FullRoundDecline* why) {
  auto fail = [&](FullRoundDecline d) -> std::optional<FullRoundPlan> {
    if (why) *why = d;
    return std::nullopt;
  };
  const auto eL = edgeEnds(solid, seamL);
  const auto eR = edgeEnds(solid, seamR);
  if (!eL || !eR) return fail(FullRoundDecline::NoSeams);

  const auto plL = neighbourWallPlane(solid, faceMap, middleFaceId, eL->a, eL->b);
  const auto plR = neighbourWallPlane(solid, faceMap, middleFaceId, eR->a, eR->b);
  if (!plL || !plR) return fail(FullRoundDecline::NoSeams);  // curved / missing neighbour

  // Walls must be PARALLEL (|nL·nR| ≈ 1). A dihedral needs the M2 valley-solve/merge.
  const double dp = math::dot(plL->normal, plR->normal);
  if (std::abs(std::abs(dp) - 1.0) > 1e-4) return fail(FullRoundDecline::NotParallel);

  // Prismatic: the two seams are straight (edgeEnds already guarantees a 2-vertex
  // line) and near-equal length; a closed/annulus seam would not resolve as a single
  // straight edge here → ClosedSeam.
  const double l2L = edgeLen2(solid, seamL);
  const double l2R = edgeLen2(solid, seamR);
  if (!(l2L > kBlendEps) || !(l2R > kBlendEps)) return fail(FullRoundDecline::ClosedSeam);
  const double lenRatio = std::sqrt(std::min(l2L, l2R) / std::max(l2L, l2R));
  if (lenRatio < 0.999) return fail(FullRoundDecline::ClosedSeam);  // not a straight prism strip

  // Width w = perpendicular gap between the parallel walls = |(pR − pL)·nL|, measured
  // between the two seam midpoints along the wall normal.
  const math::Point3 mL{0.5 * (eL->a.x + eL->b.x), 0.5 * (eL->a.y + eL->b.y),
                        0.5 * (eL->a.z + eL->b.z)};
  const math::Point3 mR{0.5 * (eR->a.x + eR->b.x), 0.5 * (eR->a.y + eR->b.y),
                        0.5 * (eR->a.z + eR->b.z)};
  const double w = std::abs(math::dot(mR - mL, plL->normal));
  if (!(w > 2.0 * kBlendEps)) return fail(FullRoundDecline::NoSeams);

  return FullRoundPlan{seamL, seamR, 0.5 * w};
}

}  // namespace detail

// Explicit left/middle/right → seam edges are the longest shared edges M∩L, M∩R.
// Builds the r = w/2 prismatic cap via fillet_edges on the two seams for PARALLEL
// walls. Returns the capped solid, or NULL (with *why set) → OCCT fallthrough.
inline topo::Shape full_round_fillet_faces(const topo::Shape& solid, int leftFaceId,
                                           int middleFaceId, int rightFaceId,
                                           double deflection = 0.01,
                                           FullRoundDecline* why = nullptr) {
  auto fail = [&](FullRoundDecline d) {
    if (why) *why = d;
    return topo::Shape{};
  };
  if (why) *why = FullRoundDecline::Ok;
  if (solid.isNull()) return fail(FullRoundDecline::BadInput);

  PlanarModel model(solid);
  if (!model.isValid()) return fail(FullRoundDecline::NonPlanarSolid);
  if (!facePlane(solid, middleFaceId) || !facePlane(solid, leftFaceId) ||
      !facePlane(solid, rightFaceId))
    return fail(FullRoundDecline::NonPlanarSolid);

  const topo::ShapeMap faceMap = topo::mapShapes(solid, topo::ShapeType::Face);
  const topo::ShapeMap edgeMap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (middleFaceId < 1 || static_cast<std::size_t>(middleFaceId) > faceMap.size() ||
      leftFaceId < 1 || static_cast<std::size_t>(leftFaceId) > faceMap.size() ||
      rightFaceId < 1 || static_cast<std::size_t>(rightFaceId) > faceMap.size())
    return fail(FullRoundDecline::BadInput);

  // Longest edge of the middle face also carried (by endpoint geometry) by face fb.
  auto longestShared = [&](int fb) -> int {
    int best = 0;
    double bestLen2 = 0.0;
    for (topo::Explorer ex(faceMap.shape(middleFaceId), topo::ShapeType::Edge); ex.more();
         ex.next()) {
      const int eid = edgeMap.findIndex(ex.current());
      if (eid < 1) continue;
      const auto e = edgeEnds(solid, eid);
      if (!e) continue;
      if (!detail::faceHasEdge(solid, faceMap, fb, e->a, e->b)) continue;
      const double l2 = detail::edgeLen2(solid, eid);
      if (l2 > bestLen2) { bestLen2 = l2; best = eid; }
    }
    return best;
  };
  const int seamL = longestShared(leftFaceId);
  const int seamR = longestShared(rightFaceId);
  if (seamL < 1 || seamR < 1 || seamL == seamR) return fail(FullRoundDecline::NoSeams);

  const auto plan = detail::planPrismatic(solid, faceMap, middleFaceId, seamL, seamR, why);
  if (!plan) return topo::Shape{};

  const int seams[2] = {plan->seamL, plan->seamR};
  topo::Shape result = fillet_edges(solid, seams, 2, plan->radius, deflection);
  if (result.isNull()) return fail(FullRoundDecline::ClosedSeam);
  return result;
}

// Single middle face → the two LONGEST edges are the seams; their across-neighbours
// are the walls. Builds the r = w/2 prismatic cap. Returns NULL (with *why) → OCCT.
inline topo::Shape full_round_fillet(const topo::Shape& solid, int middleFaceId,
                                     double deflection = 0.01, FullRoundDecline* why = nullptr) {
  auto fail = [&](FullRoundDecline d) {
    if (why) *why = d;
    return topo::Shape{};
  };
  if (why) *why = FullRoundDecline::Ok;
  if (solid.isNull()) return fail(FullRoundDecline::BadInput);

  PlanarModel model(solid);
  if (!model.isValid()) return fail(FullRoundDecline::NonPlanarSolid);
  if (!facePlane(solid, middleFaceId)) return fail(FullRoundDecline::NonPlanarSolid);

  const topo::ShapeMap faceMap = topo::mapShapes(solid, topo::ShapeType::Face);
  const topo::ShapeMap edgeMap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (middleFaceId < 1 || static_cast<std::size_t>(middleFaceId) > faceMap.size())
    return fail(FullRoundDecline::BadInput);

  // The middle face's edges, sorted by length (longest first).
  std::vector<std::pair<double, int>> edges;
  for (topo::Explorer ex(faceMap.shape(middleFaceId), topo::ShapeType::Edge); ex.more();
       ex.next()) {
    const int eid = edgeMap.findIndex(ex.current());
    if (eid < 1) continue;
    bool dup = false;
    for (const auto& e : edges)
      if (e.second == eid) { dup = true; break; }
    if (!dup) edges.emplace_back(detail::edgeLen2(solid, eid), eid);
  }
  if (edges.size() < 2) return fail(FullRoundDecline::NoSeams);
  std::sort(edges.begin(), edges.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
  const int seamL = edges[0].second;
  const int seamR = edges[1].second;

  const auto plan = detail::planPrismatic(solid, faceMap, middleFaceId, seamL, seamR, why);
  if (!plan) return topo::Shape{};

  const int seams[2] = {plan->seamL, plan->seamR};
  topo::Shape result = fillet_edges(solid, seams, 2, plan->radius, deflection);
  if (result.isNull()) return fail(FullRoundDecline::ClosedSeam);
  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FULL_ROUND_H

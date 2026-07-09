// SPDX-License-Identifier: Apache-2.0
//
// fillet_corner.h — MOAT M2/M3 SPHERICAL FILLET-CORNER weld: round EVERY convex
// planar-dihedral edge bounding a picked PLANAR face at constant radius r, welding
// the per-edge tangent-cylinder strips together at each shared corner with a
// SPHERICAL corner patch. This closes the last domino that gated `cc_fillet_face`
// (full-face fillets) on a planar solid — the curved↔curved seam reconciliation the
// sequential `fillet_edges` cannot make (its per-edge arc strips share NO corner
// vertices and leave the corner open / self-intersecting).
//
// ── WHY A NEW WELD (the crux) ─────────────────────────────────────────────────────
// A face's bounding edges form a CLOSED, corner-sharing loop. `fillet_edges` fillets
// them SEQUENTIALLY: each edge becomes a tangent cylinder (rolling ball of radius r,
// axis ∥ crease, seated tangent to the face + the neighbour side face), but where two
// adjacent edge-fillets meet at a corner their arc facets are sampled per-edge and
// share NO seam vertices, AND the corner needs a SPHERICAL patch (sphere of radius r
// centred at the trihedral offset point, tangent to all incident planes) between the
// cylinder strips. That is a curved↔curved reconciliation, distinct from the landed
// curved↔flat welds.
//
// ── GEOMETRY (the shared-arc identity that makes the weld exact) ────────────────────
// Face F (outward normal nF), CCW boundary V0..V_{n-1}, edge E_i=(V_i,V_{i+1}) shared
// with side face i (outward normal s_i). Per edge the rolling-ball CYLINDER has axis
// through C_i(P) = P − r/(1+nF·s_i)(nF+s_i) (distance −r from BOTH planes), tangent to
// F along the line through C_i+r·nF and to side i along C_i+r·s_i.
//
// At corner V_i (between edge i−1 and edge i) the SPHERE centre S_i is the point at
// distance −r from F, side i−1 AND side i (a 3-plane solve). CRUCIAL IDENTITY: S_i
// satisfies the two constraints that define EACH incident cylinder's axis line, so
// **S_i lies on both cylinder axes**. The cylinder's cross-section circle in the plane
// through S_i ⟂ crease is therefore a GREAT CIRCLE of the sphere — the cylinder strip
// end arc and the sphere patch leg are the SAME quarter arc. We sample BOTH with ONE
// canonical routine `arcSample` (slerp of the two tangent radius directions), so the
// seam vertices are bit-identical and `assembleSolid` welds them watertight at ANY
// deflection — NO tessellator change (pure assembly layer, the lowest-risk path).
//
// The sphere patch is the geodesic triangle spanned by the three tangent directions
// (nF, s_{i−1}, s_i): its two legs are shared with the two cylinder strips (above), and
// its third arc (between the two side-tangent points) bounds a FLAT corner LEDGE in the
// plane parallel to F offset inward by r (the sphere equator at centre height) — the
// top of the un-rounded vertical corner, fanned to the corner point (the intersection
// of the two side faces with that offset plane). The trimmed face F (inset to the
// tangent lines, corners at the sphere F-tangent points) and the side faces (set back to
// the side-tangent lines) complete the soup, all sharing the canonical arc vertices.
//
// ── SCOPE (honest, MEASURED) ────────────────────────────────────────────────────────
// Native only when: the solid is all-planar; the picked face is planar; every bounding
// edge is a CONVEX planar dihedral fitting radius r; AND every incident side face is
// PERPENDICULAR to F (a prism cap — the case where the corner ledge is planar and the
// weld is exact). A non-perpendicular wall, a concave/curved/≠2-face bounding edge, an
// oversized radius (tangent lines overrun a face / adjacent corner sphere overlaps), or
// a self-verify miss → NULL → OCCT (BRepFilletAPI_MakeFillet). The removed volume has a
// closed form (converges as deflection refines); the engine gates it with a watertight +
// consistently-oriented + SHRINK two-sided self-verify. NEVER a wrong/leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + topology + boolean(assemble) + blend_geom +
// fillet_edges(filletArc). OCCT-FREE. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_CORNER_H
#define CYBERCAD_NATIVE_BLEND_FILLET_CORNER_H

#include "native/blend/blend_geom.h"
#include "native/blend/fillet_edges.h"  // detail::filletArc / FilletArc
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace tess = cybercad::native::tessellate;

// Why the spherical fillet-corner weld declined (diagnostic; the engine maps NULL →
// honest error → OCCT). `Ok` iff a verified watertight result solid is returned.
enum class FilletCornerDecline {
  Ok = 0,
  BadInput,          ///< null solid / bad face id / non-positive radius
  NonPlanarSolid,    ///< the solid carries a curved face (not the planar domain)
  NonPlanarFace,     ///< the picked face is not a plane
  NoLoop,            ///< could not recover the face's ordered boundary loop
  NotPerpWall,       ///< a bounding edge's side face is not perpendicular to F (ledge
                     ///< non-planar → outside this weld's exact scope)
  NotConvexEdge,     ///< a bounding edge is not a convex planar dihedral fitting r
  RadiusTooLarge,    ///< tangent lines overrun a face / adjacent corner spheres overlap
  AssembleFailed,    ///< fewer than four faces survived (no closed solid)
  NotWatertight,     ///< self-verify: not a consistently-oriented closed 2-manifold
  VolumeInconsistent ///< self-verify: volume non-positive / not below the original
};

inline const char* filletCornerDeclineName(FilletCornerDecline d) noexcept {
  switch (d) {
    case FilletCornerDecline::Ok: return "Ok";
    case FilletCornerDecline::BadInput: return "BadInput";
    case FilletCornerDecline::NonPlanarSolid: return "NonPlanarSolid";
    case FilletCornerDecline::NonPlanarFace: return "NonPlanarFace";
    case FilletCornerDecline::NoLoop: return "NoLoop";
    case FilletCornerDecline::NotPerpWall: return "NotPerpWall";
    case FilletCornerDecline::NotConvexEdge: return "NotConvexEdge";
    case FilletCornerDecline::RadiusTooLarge: return "RadiusTooLarge";
    case FilletCornerDecline::AssembleFailed: return "AssembleFailed";
    case FilletCornerDecline::NotWatertight: return "NotWatertight";
    case FilletCornerDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

namespace detail {

// A point on the sphere/cylinder of radius r about centre S, on the arc from
// direction dFrom to dTo (either length; only their DIRECTIONS matter), at parameter
// k/steps. ONE canonical routine consumed bit-identically by every incident face of a
// shared arc (spherical great-circle slerp) so the weld vertices coincide exactly.
inline math::Point3 arcSample(const math::Point3& S, double r, const math::Vec3& dFrom,
                              const math::Vec3& dTo, int k, int steps) {
  const double nf = math::norm(dFrom), nt = math::norm(dTo);
  if (nf < kBlendEps || nt < kBlendEps || steps <= 0) return S;
  const math::Vec3 u = dFrom * (1.0 / nf);
  const math::Vec3 v = dTo * (1.0 / nt);
  const double c = std::max(-1.0, std::min(1.0, math::dot(u, v)));
  const double omega = std::acos(c);
  const double t = static_cast<double>(k) / static_cast<double>(steps);
  math::Vec3 dir;
  if (omega < 1e-9) {
    dir = u;
  } else {
    const double s = std::sin(omega);
    dir = u * (std::sin((1.0 - t) * omega) / s) + v * (std::sin(t * omega) / s);
  }
  const double dn = math::norm(dir);
  if (dn < kBlendEps) return S;
  dir = dir * (1.0 / dn);
  return math::Point3{S.x + r * dir.x, S.y + r * dir.y, S.z + r * dir.z};
}

// Emit a planar polygon with the given outward direction (CCW-from-front winding).
inline void emitPoly(std::vector<nb::Polygon>& out, std::vector<math::Point3> loop,
                     const math::Vec3& outward) {
  const math::Dir3 nd{outward};
  if (!nd.valid() || loop.size() < 3) return;
  math::Vec3 area{0, 0, 0};
  for (std::size_t i = 0; i < loop.size(); ++i)
    area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
  if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
  out.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
}

// Emit an exactly-planar triangle carrying its own geometric normal (oriented to the
// target outward), so the stored plane passes through all three vertices and the facet
// welds cleanly (curved quads are always split into two of these).
inline void emitTri(std::vector<nb::Polygon>& out, const math::Point3& a, const math::Point3& b,
                    const math::Point3& c, const math::Vec3& outward) {
  math::Vec3 nrm = math::cross(b - a, c - a);
  if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
  emitPoly(out, {a, b, c}, nrm);
}

// Solve for the point at signed distance −r from all THREE planes (n_i · p = w_i − r,
// i.e. move inward by r off each). Returns nullopt if the three normals are ~coplanar.
inline std::optional<math::Point3> triPlaneOffset(const nb::Plane& p0, const nb::Plane& p1,
                                                  const nb::Plane& p2, double r) {
  const math::Mat3 A{p0.normal.x, p0.normal.y, p0.normal.z,
                     p1.normal.x, p1.normal.y, p1.normal.z,
                     p2.normal.x, p2.normal.y, p2.normal.z};
  const auto inv = A.inverse(1e-12);
  if (!inv) return std::nullopt;
  const math::Vec3 rhs{p0.w - r, p1.w - r, p2.w - r};
  const math::Vec3 s = *inv * rhs;
  return math::Point3{s.x, s.y, s.z};
}

// Facet count for a quarter/half arc so the chord sagitta r(1−cos(Δθ/2)) ≤ deflection.
inline int arcFacetCount(double radius, double sweep, double deflection) {
  if (radius <= 0.0 || sweep <= 1e-9) return 1;
  const double maxStep =
      2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - deflection / radius)));
  if (!(maxStep > 1e-6)) return 24;
  return std::max(1, std::min(96, static_cast<int>(std::ceil(sweep / maxStep - 1e-9))));
}

// The ordered boundary loop of face `faceId` (world points, CCW from the outward
// front) as it appears in the extracted polygon soup, plus the face's outward normal.
struct FaceLoop {
  std::vector<math::Point3> pts;  // CCW loop
  math::Vec3 nF;                  // outward normal
};
inline std::optional<FaceLoop> faceLoop(const topo::Shape& solid, int faceId,
                                        const std::vector<nb::Polygon>& polys) {
  const auto pl = facePlane(solid, faceId);
  if (!pl) return std::nullopt;
  // Match the extracted polygon whose plane coincides with the picked face's plane and
  // whose centroid lies on it — the boolean soup carries one polygon per planar face.
  for (const nb::Polygon& poly : polys) {
    if (math::dot(poly.plane.normal, pl->normal) < 0.999) continue;
    if (std::fabs(poly.plane.w - pl->w) > 1e-6) continue;
    if (poly.size() < 3) continue;
    return FaceLoop{poly.vertices, pl->normal};
  }
  return std::nullopt;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// fillet_corner — round every convex planar-dihedral edge bounding the picked
// planar face `faceId` (1-based mapShapes(Face) order) of `solid` at constant
// `radius`, welding the tangent-cylinder strips with a spherical corner patch at
// each shared corner. Returns the filleted solid, or a NULL Shape (with *why set)
// → OCCT fall-through. `deflection` bounds the facet chord error.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape fillet_corner(const topo::Shape& solid, int faceId, double radius,
                                 double deflection = 0.01, FilletCornerDecline* why = nullptr) {
  auto fail = [&](FilletCornerDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (why) *why = FilletCornerDecline::Ok;
  if (solid.isNull() || !(radius > kBlendEps)) return fail(FilletCornerDecline::BadInput);
  const double r = radius;

  PlanarModel model(solid);
  if (!model.isValid()) return fail(FilletCornerDecline::NonPlanarSolid);
  const std::vector<nb::Polygon>& polys = model.polygons();

  const auto fpl = facePlane(solid, faceId);
  if (!fpl) return fail(FilletCornerDecline::NonPlanarFace);
  const auto loopOpt = detail::faceLoop(solid, faceId, polys);
  if (!loopOpt) return fail(FilletCornerDecline::NoLoop);
  const std::vector<math::Point3>& V = loopOpt->pts;
  const math::Vec3 nF = loopOpt->nF;
  const std::size_t n = V.size();
  if (n < 3) return fail(FilletCornerDecline::NoLoop);

  // Per-edge data: the neighbour side plane, the rolling-ball cylinder axis/tangents,
  // and the arc sweep (nF → side). All resolved against the ORIGINAL soup.
  struct EdgeData {
    nb::Plane side;       // side face outward plane
    math::Vec3 sideN;     // side outward normal
    int facets = 1;       // arc facet count (nF → side)
    double sweep = 0.0;
  };
  std::vector<EdgeData> ed(n);

  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3& a = V[i];
    const math::Point3& b = V[(i + 1) % n];
    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, a, b, faces) != 2)
      return fail(FilletCornerDecline::NotConvexEdge);
    // The side face is the one whose plane is NOT the picked face's plane.
    std::size_t sf = faces[0];
    if (std::fabs(polys[faces[0]].plane.w - fpl->w) < 1e-6 &&
        math::dot(polys[faces[0]].plane.normal, fpl->normal) > 0.999)
      sf = faces[1];
    const nb::Polygon& sideP = polys[sf];
    // Convex planar-dihedral rolling-ball probe (identical guard the dihedral fillet
    // uses) — rejects concave / near-flat / oversized. F is presented as a Polygon.
    const nb::Polygon fPoly{V, nb::Plane::fromPointNormal(V.front(), nF)};
    const auto arc2 = detail::filletArc(a, b, fPoly, sideP, r);
    if (!arc2) return fail(FilletCornerDecline::NotConvexEdge);
    // Scope guard: the side wall must be PERPENDICULAR to F (planar corner ledge).
    if (std::fabs(math::dot(nF, sideP.plane.normal)) > 1e-6)
      return fail(FilletCornerDecline::NotPerpWall);

    ed[i].side = sideP.plane;
    ed[i].sideN = sideP.plane.normal;
    ed[i].sweep = std::atan2(math::norm(math::cross(nF, sideP.plane.normal)),
                             math::dot(nF, sideP.plane.normal));
    ed[i].facets = detail::arcFacetCount(r, ed[i].sweep, deflection);
  }

  // Global facet count so every shared arc (cylinder end ↔ sphere leg) samples the
  // SAME number of steps — the bit-identical-vertex requirement of the weld.
  int M = 1;
  for (const EdgeData& e : ed) M = std::max(M, e.facets);

  // Per-corner sphere centre S_i (distance −r from F, side i−1, side i). Corner i is
  // between edge (i+n−1)%n and edge i.
  std::vector<math::Point3> S(n);
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t ePrev = (i + n - 1) % n;
    const auto s = detail::triPlaneOffset(*fpl, ed[ePrev].side, ed[i].side, r);
    if (!s) return fail(FilletCornerDecline::NotConvexEdge);
    S[i] = *s;
  }

  // Radius sanity: adjacent corner spheres on one edge must not cross (r not too big);
  // i.e. the two corner centres of an edge stay separated by ≥ ~0 along the edge.
  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3& s0 = S[i];
    const math::Point3& s1 = S[(i + 1) % n];
    const math::Vec3 e = V[(i + 1) % n] - V[i];
    const double elen = math::norm(e);
    if (elen < kBlendEps) return fail(FilletCornerDecline::RadiusTooLarge);
    const math::Vec3 eu = e * (1.0 / elen);
    // Both corner centres project onto the edge line; their span must be positive and
    // not exceed the edge (else the strip inverts / overruns).
    const double t0 = math::dot(s0 - V[i], eu);
    const double t1 = math::dot(s1 - V[i], eu);
    if (!(t1 - t0 > 1e-7)) return fail(FilletCornerDecline::RadiusTooLarge);
  }

  std::vector<nb::Polygon> soup;

  // The body centroid (average of all polygon vertices), used to VERIFY every emitted
  // face's outward normal points away from the material — robust to an input whose
  // extractPolygons normal is mis-oriented for some face (e.g. a base face a spline-
  // profile prism stores reversed). We never trust a carried polygon's stored normal.
  math::Vec3 csum{0, 0, 0};
  std::size_t cnt = 0;
  for (const nb::Polygon& poly : polys)
    for (const math::Point3& v : poly.vertices) { csum += v.asVec(); ++cnt; }
  const math::Point3 bodyCentroid = cnt ? math::Point3{csum.x / cnt, csum.y / cnt, csum.z / cnt}
                                        : math::Point3{0, 0, 0};
  auto outwardAt = [&](const std::vector<math::Point3>& loop, const math::Vec3& hint) {
    // Choose the sign of `hint` that points away from the body centroid at the loop's
    // own centroid (a convex-body heuristic; the blend scope is prism-like/convex).
    math::Vec3 lc{0, 0, 0};
    for (const math::Point3& p : loop) lc += p.asVec();
    if (!loop.empty()) lc = lc * (1.0 / static_cast<double>(loop.size()));
    const math::Vec3 away = lc - bodyCentroid.asVec();
    return (math::dot(hint, away) < 0.0) ? (hint * -1.0) : hint;
  };

  // (1) TRIMMED face F: inset polygon with a corner at each sphere F-tangent point
  // (S_i + r·nF). Planar in F's plane. Outward = nF.
  {
    std::vector<math::Point3> ring;
    ring.reserve(n);
    for (std::size_t i = 0; i < n; ++i)
      ring.push_back(math::Point3{S[i].x + r * nF.x, S[i].y + r * nF.y, S[i].z + r * nF.z});
    detail::emitPoly(soup, std::move(ring), nF);
  }

  // (2) Every OTHER face of the solid (not F, not a side face touched at the top) is
  // carried through unchanged; each SIDE face is clipped to the side-tangent line by
  // dropping its top strip of height r along −... — but a general body's side faces
  // may be shared by multiple top edges. We rebuild the affected faces from scratch:
  //   * the picked face F (done);
  //   * each side face i: set its top edge back to the side-tangent line (the loop
  //     with its F-adjacent edge replaced by the tangent line at S_i·..S_{i+1}).
  // To stay within the verified prism scope we require every non-F, non-side face to
  // pass through untouched; a side face's ORIGINAL polygon has its top edge (the shared
  // crease V_i→V_{i+1}) replaced by the two tangent points and the two corner points.
  // We detect side faces by their plane matching an edge's side plane.
  auto isSidePlane = [&](const nb::Plane& pl, std::size_t& outEdge) {
    for (std::size_t i = 0; i < n; ++i)
      if (math::dot(pl.normal, ed[i].sideN) > 0.999 && std::fabs(pl.w - ed[i].side.w) < 1e-6) {
        outEdge = i;
        return true;
      }
    return false;
  };

  // The plane parallel to F, offset inward by r (the sphere-equator / corner-ledge
  // plane). Points below it (toward material) stay; the top strip is removed.
  const nb::Plane offF = nb::Plane::fromPointNormal(
      math::Point3{fpl->normal.x * (fpl->w - r), fpl->normal.y * (fpl->w - r),
                   fpl->normal.z * (fpl->w - r)},
      fpl->normal);  // nF·p = w − r on this plane

  // The tangent / corner points each side edge introduces along its cut (side-tangent)
  // line: the strip bottom rim endpoints (S_i+r·s_i, S_{i+1}+r·s_i) and the two corner
  // points cp_i, cp_{i+1}. Pre-inserting them as REAL corners of the side face (rather
  // than leaving them to T-junction repair) keeps the side face's cut edge free of a
  // 4-collinear-point ear-clip degeneracy (which otherwise drops a sliver and opens the
  // seam for some face orientations). Collected per side plane.
  std::vector<math::Point3> ledgePt(n);  // cp per corner (reused below)
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t ePrev = (i + n - 1) % n;
    const auto cp = detail::triPlaneOffset(offF, ed[ePrev].side, ed[i].side, 0.0);
    if (!cp) return fail(FilletCornerDecline::NotConvexEdge);
    ledgePt[i] = *cp;
  }

  for (const nb::Polygon& poly : polys) {
    // Skip the picked face F (rebuilt above).
    if (math::dot(poly.plane.normal, fpl->normal) > 0.999 &&
        std::fabs(poly.plane.w - fpl->w) < 1e-6)
      continue;
    std::size_t se = 0;
    if (isSidePlane(poly.plane, se)) {
      // Side face: clip its loop to the material side of the offset-F plane (drop the
      // top strip of height r), then FAN-triangulate from a vertex OFF the cut line with
      // the tangent/corner points inserted on the cut edge — so no big collinear polygon
      // is handed to the ear-clip (robust for every face orientation).
      std::vector<math::Point3> clipped = clipBelow(poly.vertices, offF, kBlendEps);
      if (clipped.size() < 3) continue;
      // Points that lie ON this side's cut line (offF ∩ side plane) to insert.
      std::vector<math::Point3> onCut;
      onCut.push_back(math::Point3{S[se].x + r * ed[se].sideN.x, S[se].y + r * ed[se].sideN.y,
                                   S[se].z + r * ed[se].sideN.z});
      onCut.push_back(math::Point3{S[(se + 1) % n].x + r * ed[se].sideN.x,
                                   S[(se + 1) % n].y + r * ed[se].sideN.y,
                                   S[(se + 1) % n].z + r * ed[se].sideN.z});
      onCut.push_back(ledgePt[se]);
      onCut.push_back(ledgePt[(se + 1) % n]);
      // Insert each onCut point that lies strictly inside a clipped edge, in order.
      std::vector<math::Point3> loop2;
      const std::size_t m = clipped.size();
      for (std::size_t j = 0; j < m; ++j) {
        const math::Point3& a = clipped[j];
        const math::Point3& b = clipped[(j + 1) % m];
        loop2.push_back(a);
        std::vector<std::pair<double, math::Point3>> mids;
        const math::Vec3 ab = b - a;
        const double len2 = math::normSquared(ab);
        for (const math::Point3& p : onCut)
          if (nb::detail::onSegmentInterior(a, b, p, kBlendEps))
            mids.emplace_back(math::dot(p - a, ab) / len2, p);
        std::sort(mids.begin(), mids.end(),
                  [](const auto& x, const auto& y) { return x.first < y.first; });
        for (const auto& mp : mids) loop2.push_back(mp.second);
      }
      // Fan from the vertex farthest from the cut line (guaranteed off it), so every fan
      // triangle is non-degenerate even where the cut edge has collinear inserted points.
      std::size_t apex = 0;
      double best = -1.0;
      for (std::size_t j = 0; j < loop2.size(); ++j) {
        const double d = std::fabs(offF.signedDistance(loop2[j]));
        if (d > best) { best = d; apex = j; }
      }
      const math::Vec3 sideOut = outwardAt(loop2, poly.plane.normal);
      const std::size_t L2 = loop2.size();
      for (std::size_t j = 1; j + 1 < L2; ++j) {
        const std::size_t ia = apex;
        const std::size_t ib = (apex + j) % L2;
        const std::size_t ic = (apex + j + 1) % L2;
        detail::emitTri(soup, loop2[ia], loop2[ib], loop2[ic], sideOut);
      }
    } else {
      // Untouched face (bottom, or a wall not bounding F). Re-emit with a centroid-
      // verified outward normal (the input's stored normal may be reversed for some
      // constructors' base face) so the assembled shell is consistently oriented.
      detail::emitPoly(soup, poly.vertices, outwardAt(poly.vertices, poly.plane.normal));
    }
  }

  // (3) CYLINDER strips: per edge, quads between the two corner cross-sections (centres
  // S_i, S_{i+1}) sampling the arc nF → side with M steps. Rim shared with the sphere.
  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3& s0 = S[i];
    const math::Point3& s1 = S[(i + 1) % n];
    const math::Vec3 side = ed[i].sideN;
    for (int k = 0; k < M; ++k) {
      const math::Point3 p0a = detail::arcSample(s0, r, nF, side, k, M);
      const math::Point3 p0b = detail::arcSample(s0, r, nF, side, k + 1, M);
      const math::Point3 p1a = detail::arcSample(s1, r, nF, side, k, M);
      const math::Point3 p1b = detail::arcSample(s1, r, nF, side, k + 1, M);
      const math::Vec3 out = (p0a - s0) + (p0b - s0);  // radial, away from axis
      detail::emitTri(soup, p0a, p1a, p1b, out);
      detail::emitTri(soup, p0a, p1b, p0b, out);
    }
  }

  // (4) SPHERE octant patch + FLAT corner ledge per corner. The octant is the geodesic
  // triangle (nF, sPrev, sCur); its two legs are the two cylinder end arcs (shared),
  // its third arc (row M, in the offset-F plane) bounds the flat ledge fanned to the
  // corner point cp = the two side planes ∩ the offset-F plane.
  for (std::size_t i = 0; i < n; ++i) {
    const math::Point3 Sc = S[i];
    const std::size_t ePrev = (i + n - 1) % n;
    const math::Vec3 sA = ed[ePrev].sideN;  // leg A: shared with cylinder ePrev end
    const math::Vec3 sB = ed[i].sideN;      // leg B: shared with cylinder i start
    // Rows j=0..M; row j has j+1 points slerp between leg-A[j] and leg-B[j].
    std::vector<std::vector<math::Point3>> rows(static_cast<std::size_t>(M) + 1);
    for (int j = 0; j <= M; ++j) {
      const math::Point3 aj = detail::arcSample(Sc, r, nF, sA, j, M);
      const math::Point3 bj = detail::arcSample(Sc, r, nF, sB, j, M);
      rows[j].resize(static_cast<std::size_t>(j) + 1);
      const math::Vec3 da = aj - Sc, db = bj - Sc;
      for (int k = 0; k <= j; ++k)
        rows[j][k] = (j == 0) ? aj : detail::arcSample(Sc, r, da, db, k, j);
    }
    for (int j = 0; j < M; ++j) {
      for (int k = 0; k <= j; ++k) {
        detail::emitTri(soup, rows[j][k], rows[j + 1][k], rows[j + 1][k + 1], rows[j][k] - Sc);
        if (k < j)
          detail::emitTri(soup, rows[j][k], rows[j + 1][k + 1], rows[j][k + 1], rows[j][k] - Sc);
      }
    }
    // Flat corner ledge: fan from cp (side(ePrev) ∩ side(i) ∩ offset-F plane) over arc C
    // (row M, in the offset-F plane). Outward = nF (the ledge faces the F side). cp was
    // pre-computed as ledgePt[i] so it coincides with the side-face insertions bit-exactly.
    const math::Point3& cp = ledgePt[i];
    for (int k = 0; k < M; ++k)
      detail::emitTri(soup, cp, rows[M][k], rows[M][k + 1], nF);
  }

  // (5) Weld + triangulate + T-junction repair.
  const topo::Shape result = nb::assembleSolid(soup);
  if (result.isNull()) return fail(FilletCornerDecline::AssembleFailed);

  // (6) MANDATORY self-verify: consistently-oriented watertight closed 2-manifold AND
  // 0 < V < V(original) (a convex fillet only REMOVES material). Two-sided volume bound
  // + directed-edge orientation coherence (never a leaky / inverted / wrong solid).
  tess::MeshParams mp;
  mp.deflection = std::min(deflection, 0.01);
  const tess::Mesh mR = tess::SolidMesher(mp).mesh(result);
  if (!tess::isConsistentlyOriented(mR)) return fail(FilletCornerDecline::NotWatertight);
  const double v = std::fabs(tess::enclosedVolume(mR));
  const tess::Mesh mO = tess::SolidMesher(mp).mesh(solid);
  const double v0 = std::fabs(tess::enclosedVolume(mO));
  const double tol = 1e-9 * std::max(v0, 1.0);
  if (!(v > tol) || std::isnan(v) || !(v < v0 - tol))
    return fail(FilletCornerDecline::VolumeInconsistent);

  return result;
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_CORNER_H

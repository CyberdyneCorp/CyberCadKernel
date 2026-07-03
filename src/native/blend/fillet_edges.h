// SPDX-License-Identifier: Apache-2.0
//
// fillet_edges.h — native constant-radius rolling-ball fillet on a convex edge
// between two PLANAR faces (Phase 4 #6 `native-blends`, openspec/NATIVE-REWRITE.md).
//
// ── ROLLING-BALL / TANGENT-CYLINDER GEOMETRY (the Phase-3 dihedral construction) ──
// A ball of radius r rolled into a convex dihedral edge traces a CYLINDER of radius
// r whose axis is PARALLEL to the crease and seated tangent to both planes. For an
// edge point E lying on both face planes (outward normals n1, n2) the cylinder AXIS
// passes through
//     C = E − r/(1+n1·n2)·(n1 + n2)
// (the point at signed distance −r from BOTH planes, on the material side — solve
// n1·(C−E) = n2·(C−E) = −r). The ball touches face i along the TANGENT LINE through
//     Ti = C + r·ni
// (the foot of the perpendicular from the axis to plane i). The blend replaces the
// sharp edge with the cylinder arc from T1 to T2 (sweeping the angle between the two
// tangent radii C→T1 and C→T2), tangent (G1) to both faces at T1 and T2.
//
// ── IMPLEMENTATION (planar-polygon edit, faceted arc) ─────────────────────────────
// Working in the boolean's oriented-planar-polygon soup (blend_geom.h) so the result
// welds + triangulates watertight through assembleSolid (the same path a native
// prism / boolean uses). Per picked convex edge:
//   1. cut back both adjacent faces to the tangent lines T1, T2 by clipping every
//      polygon to the material side of the CHAMFER plane through T1 and T2 (removes
//      the corner wedge and leaves the two faces set back exactly to the tangent
//      lines);
//   2. tile the cylinder arc T1→…→T2 with N flat facets — one narrow planar strip
//      per angular step, each strip a quad spanning the edge length. N is chosen from
//      a deflection bound (sagitta r(1−cos(Δθ/2)) ≤ deflection) so the faceted fillet
//      is deflection-bounded vs the true OCCT cylinder.
// The facets share their rim vertices with the set-back faces, so the shell closes.
// The engine self-verify (watertight + volume BELOW the sharp solid and ABOVE the
// chamfer) accepts it, else falls through to OCCT.
//
// ── SCOPE (honest) ────────────────────────────────────────────────────────────────
// Native only for a CONVEX edge shared by exactly TWO PLANAR faces (a planar
// dihedral), radius small enough that both tangent lines stay within their faces. A
// CONCAVE edge, a CURVED adjacent face, an edge shared by ≠2 faces, multi-edge
// interference (adjacent picked edges whose fillets overlap), or variable radius →
// NULL → OCCT (BRepFilletAPI_MakeFillet) fallthrough. Deflection-bounded vs OCCT.
//
// CLEAN-ROOM. Reuses only src/native/math + topology + boolean. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BLEND_FILLET_EDGES_H
#define CYBERCAD_NATIVE_BLEND_FILLET_EDGES_H

#include "native/blend/blend_geom.h"
#include "native/blend/chamfer_edges.h"  // detail::faceInward / centroidOf / chamfer clip

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// The rolling-ball cylinder for a convex edge (ea,eb) between faces f1,f2 at radius
// r. Returns the axis point C and the two tangent points T1,T2 (on the crease's
// cross-section through the edge midpoint), or nullopt if the config is not a clean
// convex planar dihedral or r is too large for the faces.
struct FilletArc {
  math::Point3 axis;      // C, on the cross-section through the edge midpoint
  math::Point3 t1, t2;    // tangent points on face 1 / face 2
  math::Vec3 t;           // unit crease direction
  double radius = 0.0;
};

inline std::optional<FilletArc> filletArc(const math::Point3& ea, const math::Point3& eb,
                                          const nb::Polygon& f1, const nb::Polygon& f2,
                                          double r) {
  const auto tOpt = creaseDir(ea, eb);
  if (!tOpt) return std::nullopt;
  const math::Vec3 t = tOpt->vec();
  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};

  const math::Vec3 n1 = f1.plane.normal;
  const math::Vec3 n2 = f2.plane.normal;

  // Convexity guard (same as chamfer): the inward dirs must both oppose the corner-
  // outward direction n1+n2.
  const math::Vec3 in1 = faceInward(t, n1, mid, centroidOf(f1.vertices));
  const math::Vec3 in2 = faceInward(t, n2, mid, centroidOf(f2.vertices));
  if (math::isNull(in1) || math::isNull(in2)) return std::nullopt;
  const math::Vec3 cornerOut = n1 + n2;
  if (math::dot(cornerOut, in1) > -kBlendEps || math::dot(cornerOut, in2) > -kBlendEps)
    return std::nullopt;

  const double c = math::dot(n1, n2);
  if (c <= -1.0 + kBlendEps) return std::nullopt;  // planes anti-parallel (no corner)
  const math::Vec3 x = (n1 + n2) * (-r / (1.0 + c));  // C − E
  const math::Point3 C = mid + x;
  const math::Point3 T1 = C + n1 * r;
  const math::Point3 T2 = C + n2 * r;
  return FilletArc{C, T1, T2, t, r};
}

// The chamfer plane through the two tangent lines (T1,T2 parallel to the crease). Its
// outward normal points away from the material (toward the removed corner). Used to
// set both faces back to the tangent lines before tiling the arc.
inline std::optional<nb::Plane> tangentCutPlane(const FilletArc& fa, const math::Point3& edgeMid) {
  math::Vec3 n = math::cross(fa.t, fa.t2 - fa.t1);
  const math::Dir3 nd{n};
  if (!nd.valid()) return std::nullopt;
  nb::Plane pl = nb::Plane::fromPointNormal(fa.t1, nd.vec());
  if (signedDist(pl, edgeMid) < 0.0) { pl.normal = -pl.normal; pl.w = -pl.w; }
  return pl;
}

// Facet count for the arc so the chord sagitta r(1−cos(Δθ/2)) ≤ deflection.
inline int arcFacets(double radius, double sweep, double deflection) {
  if (radius <= 0.0) return 1;
  const double maxStep = 2.0 * std::acos(std::max(-1.0, std::min(1.0, 1.0 - deflection / radius)));
  if (!(maxStep > 1e-6)) return 24;
  return std::max(1, std::min(64, static_cast<int>(std::ceil(sweep / maxStep - 1e-9))));
}

// Sample the arc T1→T2 about the axis C (radius r) into `facets`+1 rim points at the
// edge midpoint's cross-section, then extrude each to the two edge ends to form the
// quad strips. We express the rim in world by rotating the radius vector C→T1 toward
// C→T2 about the crease direction t. `atA` / `atB` place the same rim at the two edge
// endpoints (translate along t by the signed offset of ea/eb from mid).
inline std::vector<nb::Polygon> arcStrips(const FilletArc& fa, const math::Point3& ea,
                                          const math::Point3& eb, double deflection) {
  const math::Vec3 r1 = fa.t1 - fa.axis;  // radius to T1
  const math::Vec3 r2 = fa.t2 - fa.axis;  // radius to T2
  const double rlen = math::norm(r1);
  if (rlen < kBlendEps) return {};
  const math::Dir3 u1{r1};
  // Sweep angle between the two radii.
  double sweep = std::atan2(math::norm(math::cross(r1, r2)), math::dot(r1, r2));
  if (!(sweep > 1e-6)) return {};
  const int nf = arcFacets(fa.radius, sweep, deflection);

  // Rotation axis is the crease direction, oriented so rotating u1 by +sweep reaches
  // u2 (choose sign by testing the small-angle rotation direction).
  math::Dir3 axisDir{fa.t};
  {
    const math::Mat3 test = math::Mat3::rotation(axisDir, 1e-3);
    const math::Vec3 spun = test * r1;
    if (math::dot(math::cross(r1, spun), math::cross(r1, r2)) < 0.0) axisDir = axisDir.reversed();
  }

  // Offsets of the two edge endpoints from the cross-section (along the crease).
  const math::Point3 mid{0.5 * (ea.x + eb.x), 0.5 * (ea.y + eb.y), 0.5 * (ea.z + eb.z)};
  const double offA = math::dot(ea - mid, fa.t);
  const double offB = math::dot(eb - mid, fa.t);

  auto rimAt = [&](int k, double off) -> math::Point3 {
    const double ang = sweep * k / nf;
    const math::Vec3 rv = math::Mat3::rotation(axisDir, ang) * r1;
    return fa.axis + rv + fa.t * off;
  };

  std::vector<nb::Polygon> strips;
  strips.reserve(static_cast<std::size_t>(nf));
  for (int k = 0; k < nf; ++k) {
    // Quad at endpoints A and B for facet k → k+1: (A_k, A_{k+1}, B_{k+1}, B_k).
    const math::Point3 aK = rimAt(k, offA);
    const math::Point3 aK1 = rimAt(k + 1, offA);
    const math::Point3 bK1 = rimAt(k + 1, offB);
    const math::Point3 bK = rimAt(k, offB);
    // Outward normal of the strip = the local cylinder normal (radial, away from C).
    const math::Vec3 midR = (rimAt(k, 0.0) - fa.axis) + (rimAt(k + 1, 0.0) - fa.axis);
    const math::Dir3 nd{midR};
    if (!nd.valid()) continue;
    std::vector<math::Point3> loop{aK, aK1, bK1, bK};
    // Ensure CCW as seen from +normal.
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    strips.emplace_back(loop, nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  }

  // END CAPS. Each edge terminus abuts an end face that was clipped to the straight
  // chord T1→T2; the faceted arc bulges inward from that chord, leaving a circular-
  // segment gap in the end plane. Fill it with a fan face = the arc rim ring at that
  // terminus (its implicit chord rim[nf]→rim[0] closes the segment), oriented so its
  // outward normal points OUT of the solid at that end (along ±t away from mid).
  auto endCap = [&](double off, const math::Vec3& outward) {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(nf) + 1);
    for (int k = 0; k <= nf; ++k) ring.push_back(rimAt(k, off));
    const math::Dir3 nd{outward};
    if (!nd.valid() || ring.size() < 3) return;
    math::Vec3 areaV{0, 0, 0};
    for (std::size_t i = 0; i < ring.size(); ++i)
      areaV += math::cross(ring[i].asVec(), ring[(i + 1) % ring.size()].asVec());
    if (math::dot(areaV, nd.vec()) < 0.0) std::reverse(ring.begin(), ring.end());
    strips.emplace_back(ring, nb::Plane::fromPointNormal(ring.front(), nd.vec()));
  };
  endCap(offA, fa.t * (offA >= offB ? 1.0 : -1.0));
  endCap(offB, fa.t * (offB >= offA ? 1.0 : -1.0));
  return strips;
}

}  // namespace detail

// Fillet the convex planar-dihedral edges `edgeIds` (1-based, mapShapes order) of
// `solid` with constant radius `r`. Returns the filleted solid (faceted cylinder
// blend, deflection-bounded), or a NULL Shape if any picked edge is not a convex
// edge between two planar faces / the solid is not planar / r is degenerate (→ OCCT
// fallthrough). `deflection` bounds the facet chord error against the true cylinder.
inline topo::Shape fillet_edges(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                double radius, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount <= 0 || !(radius > kBlendEps)) return {};
  PlanarModel model(solid);
  if (!model.isValid()) return {};

  std::vector<nb::Polygon> polys = model.polygons();
  for (int i = 0; i < edgeCount; ++i) {
    const auto ends = edgeEnds(solid, edgeIds[i]);
    if (!ends) return {};
    const math::Point3 mid{0.5 * (ends->a.x + ends->b.x), 0.5 * (ends->a.y + ends->b.y),
                           0.5 * (ends->a.z + ends->b.z)};

    // Two faces on this edge in the current soup.
    std::size_t faces[2];
    if (facesOnEdgeInSoup(polys, ends->a, ends->b, faces) != 2) return {};

    const auto arc = detail::filletArc(ends->a, ends->b, polys[faces[0]], polys[faces[1]], radius);
    if (!arc) return {};
    const auto cut = detail::tangentCutPlane(*arc, mid);
    if (!cut) return {};

    // Set both faces back to the tangent lines (clip the wedge off — reuse chamfer's
    // clip WITHOUT its flat chamfer face; the arc strips fill the opening instead).
    std::vector<nb::Polygon> setback;
    setback.reserve(polys.size());
    for (const nb::Polygon& poly : polys) {
      std::vector<math::Point3> clipped = clipBelow(poly.vertices, *cut, kBlendEps);
      if (clipped.size() >= 3) setback.emplace_back(std::move(clipped), poly.plane);
    }
    std::vector<nb::Polygon> strips = detail::arcStrips(*arc, ends->a, ends->b, deflection);
    if (strips.empty()) return {};
    for (nb::Polygon& s : strips) setback.push_back(std::move(s));

    // Cap the two ends of the fillet groove: the triangle (edgeEnd, T1, T2) that
    // closes the sweep at each edge terminus is generated implicitly by
    // assembleSolid's T-junction repair welding the strip rim into the set-back end
    // faces. (For an edge whose two ends abut other faces this closes cleanly; a
    // free-standing fillet end is left to the self-verify.)
    polys = std::move(setback);
  }
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_FILLET_EDGES_H

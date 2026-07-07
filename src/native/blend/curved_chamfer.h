// SPDX-License-Identifier: Apache-2.0
//
// curved_chamfer.h — native constant-distance CHAMFER on a CIRCULAR crease (Phase 4
// #6 `native-blends`, curved-chamfer slice). The chamfer counterpart to the convex
// circular fillet (curved_fillet.h): where the fillet rolls a ball into the rim and
// lays a G1-tangent TORUS arc, the chamfer cuts a FLAT bevel — a CONE FRUSTUM band —
// across the rim where a CYLINDER lateral face meets a coaxial PLANAR cap.
//
// ── CONE-FRUSTUM (STRAIGHT BEVEL) GEOMETRY ─────────────────────────────────────────
// On the CONVEX circular rim where a cylinder wall (radius Rc, axis A) meets a coaxial
// planar cap at axial height H, a SYMMETRIC chamfer of distance d sets EACH face back
// by d (measured in that face, perpendicular to the rim):
//   * cylinder seam = CIRCLE radius Rc at axial H − s·d   (the wall, set back AXIALLY;
//                     s = axial sign from the far end toward the cap);
//   * cap seam      = CIRCLE radius Rc − d at axial H      (the cap, set back RADIALLY).
// The band between the two seams is a CONE FRUSTUM — a ruled straight bevel
//   radius(τ) = Rc − d·τ,  axial(τ) = (H − s·d) + s·d·τ,  τ ∈ [0,1]
// NOT a torus arc. In the (radial, axial) meridian its profile is the straight segment
// (Rc, H−s·d) → (Rc−d, H); the removed sharp corner is the right triangle with legs
// d×d at the rim.
//
// ── C0, NOT G1 (the load-bearing correctness inversion vs the fillet) ──────────────
// A chamfer is a FLAT bevel, so it is C0 (position-continuous) but NOT tangent at
// either seam. The frustum outward normal is the exact BISECTOR of the two face
// normals: for a right-angle rim it makes cos = 1/√2 (≈0.7071) with BOTH the cylinder
// radial normal (at τ=0) and the cap axial normal (at τ=1) — the 45° symmetric bevel.
// This is the discriminator vs the fillet, whose seam normals give cos = 1 (tangent).
// The builder must NEVER assert G1 here — that would be geometrically WRONG for a
// chamfer. The engine's watertight + shrink self-verify accepts the result; else NULL.
//
// ── EXACT REMOVED VOLUME (Pappus) ──────────────────────────────────────────────────
// The removed corner ring is the right triangle (legs d×d, area d²/2, centroid radial
// Rc − d/3) revolved about the axis: V_removed = π·d²·(Rc − d/3). Volume REDUCES. A
// chamfer of setback d removes MORE than a fillet of radius d (d²/2 > d²(1−π/4)).
//
// ── IMPLEMENTATION (planar-facet weld, tessellator-pristine) ───────────────────────
// Reuses the fillet's rim recognition + emit helpers. The capped cylinder is rebuilt
// as one deflection-bounded planar-triangle soup sharing N angular samples:
//   1. far cap disk radius Rc at farH (outward −capNormal);
//   2. wall farH → hSeam=H−s·d, radius Rc, N quads;
//   3. FRUSTUM band ring(Rc,hSeam) → ring(Rc−d,H), N quads — ONE meridian step (the
//      only new geometry; simpler than the torus' M-step arc), per-triangle geometric
//      normal so every facet is exactly planar;
//   4. trimmed cap disk radius Rc−d at H (outward +capNormal).
// All share the SAME N samples so every seam welds with coincident vertices via
// assembleSolid.
//
// ── SCOPE (honest) ─────────────────────────────────────────────────────────────────
// Native only for a SYMMETRIC-distance chamfer on a CONVEX circular rim between exactly
// ONE cylinder lateral face and ONE coaxial planar cap, with Rc − d > 0 and the wall
// covering H − s·d. Everything else returns NULL → OCCT BRepFilletAPI_MakeChamfer:
//   * asymmetric (two-distance) chamfers; non-circular creases; cone↔plane / sphere /
//     spline / tilted caps; concave rims; curved↔curved (cyl↔cyl) rims; Rc ≤ d;
//     multiple picked edges.
//
// CLEAN-ROOM. Reuses only src/native/blend/curved_fillet.h detail helpers (which use
// src/native/math + topology + boolean). No OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_CHAMFER_H
#define CYBERCAD_NATIVE_BLEND_CURVED_CHAMFER_H

#include "native/blend/blend_geom.h"
#include "native/blend/curved_fillet.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// Assemble the CHAMFERED capped cylinder as a planar-facet soup, for the ASYMMETRIC
// two-distance chamfer (d1 = axial WALL setback, d2 = radial CAP setback). The band is
// an OBLIQUE cone frustum between the two setback circles (Rc, H − s·d1) and (Rc − d2, H):
//   radius(τ) = Rc − d2·τ,  axial(τ) = (H − s·d1) + s·d1·τ,  τ ∈ [0,1].
// The bevel outward normal is radial·d1 + axial·(s·d2) — C0 (position-continuous) at the
// two DIFFERENT bevel angles (cos = d1/√(d1²+d2²) at the wall, d2/√(d1²+d2²) at the cap),
// NEVER G1. The SYMMETRIC chamfer is the exact d1 = d2 case (buildChamferedCylinder wraps
// this with d1 = d2 = d, reproducing the byte-identical soup). Returns the polygons
// (empty on any degeneracy → NULL → OCCT). CCW-from-front winding per facet; assembleSolid
// welds.
inline std::vector<nb::Polygon> buildChamferedCylinderAsym(const RimGeom& g, double d1, double d2,
                                                           double defl) {
  const double Rc = g.radius;
  const double Rin = Rc - d2;                // trimmed-cap radius (cap set back radially d2)
  if (!(Rin > 1e-9)) return {};              // Rc − d2 > 0 (else the bevel crosses the axis)
  const math::Ax3& ax = g.axis;

  // Orient the axis so +z·s points toward the cap; s = sign of (capH − farH) along az.
  const double s = (g.capH >= g.farH) ? 1.0 : -1.0;
  const double hCap = g.capH;
  const double hFar = g.farH;
  const double hSeam = hCap - s * d1;        // cylinder seam (wall set back axially by d1)
  // The far end must be beyond the wall→frustum seam.
  if (s * (hSeam - hFar) <= 1e-9) return {};

  // The cap's OUTWARD direction is toward the +s end (away from the far cap), derived
  // from geometry — NOT from capNormal, whose stored sign is not reliably outward.
  const math::Vec3 capOut = ax.z.vec() * s;

  const int N = sagittaSteps(Rc, kTwoPi, defl, 8, 256);  // angular (both seams share N)

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 3 + 4);

  // Planar polygon (constant-height rings are exactly planar).
  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    const nb::Plane pl = nb::Plane::fromPointNormal(loop.front(), nd.vec());
    polys.emplace_back(std::move(loop), pl);
  };
  // Each curved/beveled quad splits into two exactly-planar triangles carrying their
  // own geometric normal (oriented to the target outward), so every facet welds cleanly.
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };

  // 1. Cylinder wall: hFar → hSeam, radius Rc, N quads (each split into two triangles).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N;
    const double um = 0.5 * (u0 + u1);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    emitQuad(ringPoint(ax, Rc, u0, hFar), ringPoint(ax, Rc, u1, hFar),
             ringPoint(ax, Rc, u1, hSeam), ringPoint(ax, Rc, u0, hSeam), outN);
  }

  // 2. Frustum bevel band: ring(Rc, hSeam) → ring(Rin=Rc−d2, hCap), N quads. ONE
  //    meridian step — the OBLIQUE straight bevel. Outward = radial·d1 + s·axial·d2
  //    (the bevel normal, C0 at the two different angles; d1=d2 → the 45° bisector).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N;
    const double um = 0.5 * (u0 + u1);
    const math::Vec3 radial = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    const math::Vec3 outN = radial * d1 + ax.z.vec() * (s * d2);  // oblique bevel normal
    emitQuad(ringPoint(ax, Rc, u0, hSeam), ringPoint(ax, Rc, u1, hSeam),
             ringPoint(ax, Rin, u1, hCap), ringPoint(ax, Rin, u0, hCap), outN);
  }

  // 3. Trimmed cap: disk radius Rin (=Rc−d2) at hCap, outward = capOut (toward +s end).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rin, kTwoPi * i / N, hCap));
    emit(std::move(ring), capOut);
  }

  // 4. Far cap: full disk radius Rc at hFar, outward = −capOut (opposite end).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rc, kTwoPi * i / N, hFar));
    emit(std::move(ring), capOut * -1.0);
  }

  return polys;
}

// SYMMETRIC chamfer (setback d on BOTH faces) — the exact d1 = d2 special case of the
// oblique-frustum builder. Kept byte-identical to the historical symmetric soup: with
// d1 = d2 the wall seam (Rc, H−s·d), cap seam (Rc−d, H) and the bevel normal direction
// radial·d + s·axial·d (== the 45° bisector radial + s·axial up to the positive scale d,
// which does not change the per-triangle geometric normal) all coincide with the former
// builder. Kept so curved_chamfer_edge and the 9/9 control stay untouched.
inline std::vector<nb::Polygon> buildChamferedCylinder(const RimGeom& g, double d, double defl) {
  return buildChamferedCylinderAsym(g, d, d, defl);
}

}  // namespace detail

// Chamfer a single CONVEX CIRCULAR crease (cylinder lateral face ↔ coaxial planar cap)
// of `solid` with a SYMMETRIC setback distance `d` — a CONE-FRUSTUM straight bevel (C0,
// NOT G1) between the two setback circles. Returns the chamfered solid (faceted frustum
// bevel, deflection-bounded, watertight) or a NULL Shape (→ OCCT) when the edge is not a
// convex circular cylinder↔cap rim, when Rc ≤ d, or on any degeneracy. Same convex
// cyl↔cap classification as curved_fillet_edge (only the transition band differs), so a
// chamfer also REMOVES material. Everything else — asymmetric distances, non-circular /
// concave rims, cyl↔cyl canals — returns NULL. Multiple picked edges (edgeCount ≠ 1) →
// NULL (this slice handles a single circular rim).
inline topo::Shape curved_chamfer_edge(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                       double d, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(d > kBlendEps)) return {};
  // The picked edge must be a Circle shared by a Cylinder + a Plane face.
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  detail::RimFaces rf;
  if (!detail::facesOnRim(solid, edgeIds[0], rf)) return {};
  const auto cyl = detail::cylinderInfo(solid, rf.cyl);
  if (!cyl) return {};
  const auto cap = facePlane(solid, rf.cap);
  if (!cap) return {};
  const auto g = detail::rimGeom(solid, edgeIds[0], *cyl, *cap);
  if (!g) return {};

  std::vector<nb::Polygon> polys = detail::buildChamferedCylinder(*g, d, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

// Chamfer a single CONVEX CIRCULAR crease (cylinder lateral face ↔ coaxial planar cap)
// of `solid` with ASYMMETRIC two-distance setbacks: `d1` = the AXIAL setback on the
// cylinder WALL, `d2` = the RADIAL setback on the cap — an OBLIQUE cone-frustum bevel
// (C0 at two DIFFERENT angles, NEVER G1) between the two setback circles (Rc, H−s·d1)
// and (Rc−d2, H). The symmetric `curved_chamfer_edge` is the exact d1 = d2 case. Returns
// the chamfered solid (faceted oblique frustum, deflection-bounded, watertight) or a NULL
// Shape (→ OCCT BRepFilletAPI_MakeChamfer::Add(d1,d2,edge,face)) when the edge is not a
// convex circular cylinder↔cap rim, when Rc ≤ d2, when the wall does not cover H−s·d1, or
// on any degeneracy. Removes material (Pappus volume π·d1·d2·(Rc − d2/3)). Everything
// else — non-circular / concave rims, cyl↔cyl canals, tilted caps — returns NULL.
// Multiple picked edges (edgeCount ≠ 1) → NULL (this slice handles a single circular rim).
inline topo::Shape curved_chamfer_edge_asym(const topo::Shape& solid, const int* edgeIds,
                                            int edgeCount, double d1, double d2,
                                            double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(d1 > kBlendEps) || !(d2 > kBlendEps)) return {};
  // The picked edge must be a Circle shared by a Cylinder + a Plane face.
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  detail::RimFaces rf;
  if (!detail::facesOnRim(solid, edgeIds[0], rf)) return {};
  const auto cyl = detail::cylinderInfo(solid, rf.cyl);
  if (!cyl) return {};
  const auto cap = facePlane(solid, rf.cap);
  if (!cap) return {};
  const auto g = detail::rimGeom(solid, edgeIds[0], *cyl, *cap);
  if (!g) return {};

  std::vector<nb::Polygon> polys = detail::buildChamferedCylinderAsym(*g, d1, d2, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_CHAMFER_H

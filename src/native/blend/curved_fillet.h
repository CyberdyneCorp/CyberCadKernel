// SPDX-License-Identifier: Apache-2.0
//
// curved_fillet.h — native constant-radius rolling-ball fillet on a CIRCULAR crease
// (Phase 4 #6 `native-blends`, first CURVED-blend slice). Extends the planar
// tangent-cylinder fillet (fillet_edges.h) to the first curved crease: the rim where
// a CYLINDER lateral face meets a coaxial PLANAR cap.
//
// ── ROLLING-BALL / CANAL-TORUS GEOMETRY ───────────────────────────────────────────
// A ball of radius r rolled into the convex circular crease between a cylinder wall
// (radius Rc, axis A) and a planar cap (perpendicular to A, at axial height H, capping
// the +axis end) stays tangent to BOTH: its centre sits at radial distance Rc−r from
// the axis and at axial distance r inside the cap (below H). As the contact point
// runs around the rim the ball centre traces a CIRCLE of radius Rc−r at height H−r —
// the tube-centre circle of a TORUS coaxial with the cylinder:
//     major radius R = Rc − r   (axis → tube-centre, in the plane ⟂ A)
//     minor radius r            (the ball)
//     tube-centre circle at axial height H − r.
// The blend surface is the quarter-tube of that torus swept by the MINOR angle
// v ∈ [0, π/2]:
//   * v = 0   → point at radius Rc, height H−r, normal RADIAL → the tangent circle
//               on the CYLINDER (torus∩cylinder, an analytic SSI-S1 coaxial seam).
//   * v = π/2 → point at radius Rc−r, height H, normal AXIAL (+A) → the tangent
//               circle on the CAP (torus∩plane, radius Rc−r).
// So the fillet is G1-tangent to the cylinder at v=0 (both normals radial) and to the
// cap at v=π/2 (both normals along the axis). Requires a RING torus R ≥ r, i.e.
// Rc ≥ 2r (else the tube would self-intersect the axis — deferred to OCCT).
//
// ── IMPLEMENTATION (planar-facet weld, tessellator-pristine) ───────────────────────
// The whole native blend pipeline welds + meshes PLANAR polygons (blend_geom.h +
// boolean assembleSolid). We therefore rebuild the filleted solid as one deflection-
// bounded planar-facet soup that shares vertices exactly along every seam so it welds
// watertight through the SAME path a native prism/boolean uses:
//   1. the FAR cap (the untouched end) as an N-gon disk (radius Rc);
//   2. the cylinder wall from the far end up to the tangent circle (H−r) as N quads;
//   3. the TORUS quarter-tube v∈[0,π/2] × u∈[0,2π] as N·M quads (two sagitta bounds:
//      N from the major radius Rc, M from the minor radius r);
//   4. the TRIMMED cap as an N-gon disk (radius Rc−r) at H.
// All four share the SAME N angular samples, so the wall→torus seam (at H−r, radius
// Rc) and the torus→cap seam (at H, radius Rc−r) weld with coincident vertices. The
// engine self-verify (watertight + 0 < Vr < Vo) then accepts it, else → OCCT.
//
// ── SCOPE (honest) ─────────────────────────────────────────────────────────────────
// Native only for a CONVEX circular rim between exactly ONE cylinder lateral face and
// ONE coaxial planar cap, radius r with Rc ≥ 2r and r < |cap height − far end| (the
// tangent circle stays on the wall). Everything else returns NULL → OCCT:
//   * concave rims, variable radius, cyl↔cyl / cyl↔cone canal fillets;
//   * non-circular curved creases, tilted (non-perpendicular) caps, freeform;
//   * a solid whose picked edge is not a Circle shared by a Cylinder + a Plane face;
//   * multiple picked edges (this slice handles a single circular rim).
//
// CLEAN-ROOM. Reuses only src/native/math (Torus/Cylinder frames) + topology +
// boolean (Polygon/assembleSolid). No OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_FILLET_H
#define CYBERCAD_NATIVE_BLEND_CURVED_FILLET_H

#include "native/blend/blend_geom.h"
#include "native/math/elementary.h"
#include "native/math/torus.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

inline constexpr double kCurveEps = 1e-9;
inline constexpr double kTwoPi = 6.283185307179586476925286766559;

// World-space frame + radius of a Cylinder face (folds the face location).
struct CylInfo {
  math::Ax3 frame;   // origin on axis, z = axis direction
  double radius = 0.0;
};
inline std::optional<CylInfo> cylinderInfo(const topo::Shape& solid, int faceId) {
  const topo::ShapeMap map = topo::mapShapes(solid, topo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > map.size()) return std::nullopt;
  const auto surf = topo::surfaceOf(map.shape(faceId));
  if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Cylinder) return std::nullopt;
  math::Ax3 f = surf->surface->frame;
  if (!surf->location.isIdentity()) {
    const math::Transform& t = surf->location.transform();
    f.origin = t.applyToPoint(f.origin);
    f.x = math::Dir3{t.applyToVector(f.x.vec())};
    f.y = math::Dir3{t.applyToVector(f.y.vec())};
    f.z = math::Dir3{t.applyToVector(f.z.vec())};
  }
  return CylInfo{f, surf->surface->radius};
}

// World centre + radius of a Circle edge (folds the edge location).
struct RimCircle {
  math::Point3 centre;
  math::Vec3 axis;   // circle plane normal (edge frame Z)
  double radius = 0.0;
};
inline std::optional<RimCircle> circleOf(const topo::Shape& solid, int edgeId) {
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeId < 1 || static_cast<std::size_t>(edgeId) > emap.size()) return std::nullopt;
  const auto c = topo::curveOf(emap.shape(edgeId));
  if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) return std::nullopt;
  math::Ax3 f = c->curve->frame;
  if (!c->location.isIdentity()) {
    const math::Transform& t = c->location.transform();
    f.origin = t.applyToPoint(f.origin);
    f.z = math::Dir3{t.applyToVector(f.z.vec())};
  }
  return RimCircle{f.origin, f.z.vec(), c->curve->radius};
}

// Find the cylinder lateral face and the planar cap face at the picked circular rim,
// matched by GEOMETRY (each face owns its own edge/vertex nodes, so node identity
// cannot pair them). The cylinder is the sole Cylinder face coaxial with the rim at
// the rim radius; the cap is the sole Plane face passing through the rim circle whose
// normal is along the rim axis. Returns false if the model is not exactly that config.
struct RimFaces {
  int cyl = 0;    // cylinder lateral face id
  int cap = 0;    // planar cap face id
};

// Is face `fi` the cylinder wall carrying the rim? Coaxial with the rim axis, at the
// rim radius, with the rim centre on its axis.
inline bool isRimCylinder(const topo::Shape& solid, int fi, const RimCircle& rim,
                          const math::Dir3& axisDir) {
  const auto info = cylinderInfo(solid, fi);
  if (!info) return false;
  if (std::fabs(std::fabs(math::dot(info->frame.z.vec(), axisDir.vec())) - 1.0) > 1e-6)
    return false;
  if (std::fabs(info->radius - rim.radius) > 1e-6) return false;
  const math::Vec3 d = rim.centre - info->frame.origin;
  const math::Vec3 perp = d - info->frame.z.vec() * math::dot(d, info->frame.z.vec());
  return math::norm(perp) <= 1e-6;  // rim centre lies on the cylinder axis
}

// Is face `fi` the planar cap carrying the rim? Normal along the rim axis, plane
// through the rim centre.
inline bool isRimCap(const topo::Shape& solid, int fi, const RimCircle& rim,
                     const math::Dir3& axisDir) {
  const auto pl = facePlane(solid, fi);
  if (!pl) return false;
  if (std::fabs(std::fabs(math::dot(pl->normal, axisDir.vec())) - 1.0) > 1e-6) return false;
  return std::fabs(signedDist(*pl, rim.centre)) <= 1e-6;
}

inline bool facesOnRim(const topo::Shape& solid, int edgeId, RimFaces& out) {
  const auto rim = circleOf(solid, edgeId);
  if (!rim) return false;
  const math::Dir3 axisDir{rim->axis};
  if (!axisDir.valid()) return false;

  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  int cyl = 0, cap = 0, cylN = 0, capN = 0;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) continue;
    const int id = static_cast<int>(fi);
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder &&
        isRimCylinder(solid, id, *rim, axisDir)) { cyl = id; ++cylN; }
    else if (surf->surface->kind == topo::FaceSurface::Kind::Plane &&
             isRimCap(solid, id, *rim, axisDir)) { cap = id; ++capN; }
  }
  if (cylN != 1 || capN != 1) return false;
  out.cyl = cyl;
  out.cap = cap;
  return true;
}

// The circular crease resolved in the cylinder frame: the axial height of the cap
// rim and of the FAR end of the cylinder wall (the other rim), plus the axial
// direction sign toward the cap. Returns nullopt for a tilted/degenerate config.
struct RimGeom {
  math::Ax3 axis;      // cylinder frame (z = axis)
  double radius = 0.0; // Rc
  double capH = 0.0;   // axial coord of the filleted cap plane
  double farH = 0.0;   // axial coord of the opposite (far) cap plane
  math::Vec3 capNormal;  // outward normal of the filleted cap (≈ ±axis)
};
inline std::optional<RimGeom> rimGeom(const topo::Shape& solid, int edgeId, const CylInfo& cyl,
                                      const nb::Plane& capPlane) {
  const auto ends = edgeEnds(solid, edgeId);
  // A full circle edge may report identical endpoints; fall back to sampling the rim
  // via the cap plane ∩ cylinder axial coordinate.
  const math::Vec3 az = cyl.frame.z.vec();
  // Axial coord of the cap plane: project a plane point onto the axis. capPlane.w is
  // dot(n, p); with n ≈ ±az the plane is at axial height along az. Solve for the
  // axial coord h s.t. origin + h·az lies on the plane: dot(n, origin+h·az) = w.
  const double nDotAz = math::dot(capPlane.normal, az);
  if (std::fabs(nDotAz) < 0.999) return std::nullopt;  // cap not perpendicular → defer
  const double capH = (capPlane.w - math::dot(capPlane.normal, cyl.frame.origin.asVec())) / nDotAz;

  // Far end: the cap's outward normal points away from the material, so the material
  // (and the far end) is on the −capNormal side. Find the axial extent of the wall by
  // reading both circle rims of the cylinder face's owning solid.
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  double hmin = capH, hmax = capH;
  bool any = false;
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    math::Point3 o = c->curve->frame.origin;
    if (!c->location.isIdentity()) o = c->location.transform().applyToPoint(o);
    const double h = math::dot(o - cyl.frame.origin, az);
    if (!any) { hmin = hmax = h; any = true; }
    else { hmin = std::min(hmin, h); hmax = std::max(hmax, h); }
  }
  if (!any) return std::nullopt;
  const double farH = (std::fabs(hmax - capH) > std::fabs(hmin - capH)) ? hmax : hmin;
  (void)ends;
  RimGeom g;
  g.axis = cyl.frame;
  g.radius = cyl.radius;
  g.capH = capH;
  g.farH = farH;
  g.capNormal = capPlane.normal;
  return g;
}

// Facet counts for the two curvatures from a sagitta bound r(1−cos(Δ/2)) ≤ defl.
inline int sagittaSteps(double radius, double span, double defl, int lo, int hi) {
  if (radius <= kCurveEps || span <= kCurveEps) return lo;
  const double ratio = 1.0 - std::clamp(defl / radius, 0.0, 1.0);
  const double dmax = 2.0 * std::acos(std::clamp(ratio, -1.0, 1.0));
  const int n = dmax > 1e-9 ? static_cast<int>(std::ceil(span / dmax)) : hi;
  return std::clamp(n, lo, hi);
}

// A point on the cylinder-frame at radius `rad`, angle u (about the axis), axial h.
inline math::Point3 ringPoint(const math::Ax3& ax, double rad, double u, double h) {
  return math::frameCombine(ax, rad * std::cos(u), rad * std::sin(u), h);
}

// Assemble the filleted capped cylinder as a planar-facet soup. Returns the polygons
// (empty on any degeneracy). CCW-from-front winding per facet; assembleSolid welds.
inline std::vector<nb::Polygon> buildFilletedCylinder(const RimGeom& g, double r, double defl) {
  const double Rc = g.radius;
  const double R = Rc - r;                 // torus major radius
  if (!(R >= r - 1e-12)) return {};        // ring torus guard Rc ≥ 2r
  const math::Ax3& ax = g.axis;

  // Orient the axis so +z points toward the cap; s = sign of (capH − farH) along az.
  const double s = (g.capH >= g.farH) ? 1.0 : -1.0;
  const double hCap = g.capH;
  const double hFar = g.farH;
  const double hSeam = hCap - s * r;        // tangent circle on the wall (axial)
  const double zTube = hCap - s * r;        // tube-centre axial height (== hSeam)
  // The far end must be beyond the wall→torus seam.
  if (s * (hSeam - hFar) <= 1e-9) return {};

  const int N = sagittaSteps(Rc, kTwoPi, defl, 8, 256);           // angular
  const int M = sagittaSteps(r, kTwoPi / 4.0, defl, 3, 64);       // torus minor

  // Torus frame: coaxial, tube-centre circle at axial zTube. v is measured so that
  // v=0 → outer equator (radius R+r=Rc, height zTube) and v grows toward the cap.
  // Build minor-angle samples v_j = s·(π/2)·(j/M) so sin(v)·(z axis)·? — we compute
  // points directly to keep the sign explicit:
  //   radius(v) = R + r·cos(v),  axialOffset(v) = s·r·sin(v)   (toward the cap)
  auto torusPoint = [&](double u, double vAbs) -> math::Point3 {
    const double rad = R + r * std::cos(vAbs);
    const double h = zTube + s * r * std::sin(vAbs);
    return ringPoint(ax, rad, u, h);
  };
  auto torusNormal = [&](double u, double vAbs) -> math::Vec3 {
    // Outward = radial·cos(v) + axial·sin(v) (toward +cap side).
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * std::cos(vAbs) + ax.z.vec() * (s * std::sin(vAbs));
  };

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (M + 2) + 4);

  // Emit a planar polygon (caps: constant-height rings are exactly planar).
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
  // Emit a TRIANGLE (always exactly planar) with a target outward normal. The four
  // corners of a wall/torus quad are NOT coplanar, so a quad Polygon would carry a
  // derived plane its 4th vertex misses — the mesher then triangulates neighbours
  // inconsistently and leaks. Splitting each curved quad into two triangles keeps
  // every facet planar and welds watertight.
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    // Use the triangle's EXACT geometric normal (oriented to agree with the target
    // outward direction) so the stored Plane passes through all three vertices — a
    // triangle is always planar, so this plane is exact and the facet welds cleanly.
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };
  auto emitQuad = [&](const math::Point3& p00, const math::Point3& p10, const math::Point3& p11,
                      const math::Point3& p01, const math::Vec3& outward) {
    emitTri(p00, p10, p11, outward);
    emitTri(p00, p11, p01, outward);
  };

  // 1. Cylinder wall: hFar → hSeam, N quads (each split into two triangles).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N;
    const math::Point3 a0 = ringPoint(ax, Rc, u0, hFar);
    const math::Point3 a1 = ringPoint(ax, Rc, u1, hFar);
    const math::Point3 b1 = ringPoint(ax, Rc, u1, hSeam);
    const math::Point3 b0 = ringPoint(ax, Rc, u0, hSeam);
    const double um = 0.5 * (u0 + u1);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    emitQuad(a0, a1, b1, b0, outN);
  }

  // 2. Torus quarter-tube: v ∈ [0, π/2] × u full turn, N·M quads (two triangles each).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N;
    const double um = 0.5 * (u0 + u1);
    for (int j = 0; j < M; ++j) {
      const double v0 = (kTwoPi / 4.0) * j / M, v1 = (kTwoPi / 4.0) * (j + 1) / M;
      const double vm = 0.5 * (v0 + v1);
      emitQuad(torusPoint(u0, v0), torusPoint(u1, v0), torusPoint(u1, v1), torusPoint(u0, v1),
               torusNormal(um, vm));
    }
  }

  // 3. Trimmed cap: disk radius R (=Rc−r) at hCap, outward = capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, R, kTwoPi * i / N, hCap));
    emit(std::move(ring), g.capNormal);
  }

  // 4. Far cap: full disk radius Rc at hFar, outward = −capNormal (opposite end).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rc, kTwoPi * i / N, hFar));
    emit(std::move(ring), g.capNormal * -1.0);
  }

  return polys;
}

}  // namespace detail

// Fillet a single CIRCULAR crease (cylinder lateral face ↔ coaxial planar cap) of
// `solid` with constant radius `r`. Returns the filleted solid (faceted torus blend,
// deflection-bounded, watertight) or a NULL Shape (→ OCCT) when the edge is not a
// convex circular cylinder↔cap rim, when Rc < 2r, or on any degeneracy. Multiple
// edges (edgeCount ≠ 1) → NULL (this slice handles one rim).
inline topo::Shape curved_fillet_edge(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                      double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
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

  std::vector<nb::Polygon> polys = detail::buildFilletedCylinder(*g, r, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_FILLET_H

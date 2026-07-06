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

// ── CONCAVE circular-rim fillet (the material-side torus canal) ─────────────────────
// A CONCAVE rim is where a cylinder (a BOSS, radius Rc) meets a LARGER coaxial planar
// SHOULDER in a concave dihedral: the base rim of a boss standing on a bigger coaxial
// disc/plate (a stepped shaft / turned part). The rolling ball of radius r seats on the
// MATERIAL side of the corner, so THREE signs flip vs the convex builder:
//   * ball-centre locus radius  R_t = Rc + r  (convex Rc − r);
//   * tube-centre axial         zTube = capH + s·r  (convex capH − s·r);
//   * the swept quadrant fills the corner (radius(v) = R_t − r·cos v) and the fillet
//     ADDS material → the enclosed volume GROWS (convex shrinks).
// The canal is still a coaxial torus (major R_t = Rc + r, minor r); because R_t > r
// always, NO ring-torus guard is needed — only the seam-inside-face guard remains.
// Seams (closed form, on the torus by construction): torus∩cylinder = circle radius Rc
// at capH + s·r (the v=0 INNER-equator ring); torus∩plane = circle radius Rc + r in the
// shoulder plane z = capH (the v=π/2 ring). G1-tangent at both (radial at v=0, axial at
// v=π/2). The whole stepped body is rebuilt as a planar-facet soup that shares the SAME
// N angular samples across every seam, welded watertight through assembleSolid.

// Axial heights (along ax.z, measured from ax.origin) of coaxial Circle edges whose
// radius ≈ targetR. Returns the count and fills [hmin,hmax]. Used to read a coaxial
// cylinder face's two rim heights (its axial extent) without trusting node identity.
inline int coaxialCircleHeights(const topo::Shape& solid, const math::Ax3& ax, double targetR,
                                double& hmin, double& hmax) {
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  const math::Vec3 az = ax.z.vec();
  int n = 0;
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    math::Point3 o = c->curve->frame.origin;
    math::Vec3 cz = c->curve->frame.z.vec();
    if (!c->location.isIdentity()) {
      const math::Transform& t = c->location.transform();
      o = t.applyToPoint(o);
      cz = t.applyToVector(cz);
    }
    if (std::fabs(std::fabs(math::dot(math::Dir3{cz}.vec(), az)) - 1.0) > 1e-6) continue;
    const math::Vec3 d = o - ax.origin;
    if (math::norm(d - az * math::dot(d, az)) > 1e-6) continue;  // centre off the axis
    if (std::fabs(c->curve->radius - targetR) > 1e-6) continue;
    const double h = math::dot(o - ax.origin, az);
    if (n == 0) { hmin = hmax = h; }
    else { hmin = std::min(hmin, h); hmax = std::max(hmax, h); }
    ++n;
  }
  return n;
}

// The largest coaxial Cylinder face radius strictly greater than Rc (the PLATE around
// the boss), or nullopt if none — i.e. this is the convex capped-cylinder case, which
// the concave builder declines (the convex builder / OCCT owns it).
inline std::optional<double> plateCylinderRadius(const topo::Shape& solid, const math::Ax3& ax,
                                                 double Rc) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  double best = -1.0;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto info = cylinderInfo(solid, static_cast<int>(fi));
    if (!info) continue;
    if (std::fabs(std::fabs(math::dot(info->frame.z.vec(), ax.z.vec())) - 1.0) > 1e-6) continue;
    const math::Vec3 d = info->frame.origin - ax.origin;
    if (math::norm(d - ax.z.vec() * math::dot(d, ax.z.vec())) > 1e-6) continue;  // not coaxial
    if (info->radius > Rc + 1e-6) best = std::max(best, info->radius);
  }
  if (best < 0.0) return std::nullopt;
  return best;
}

// Every face is either a coaxial Cylinder (radius Rc or Rp) or an axis-normal Plane —
// the strict stepped-shaft topology the analytic rebuild reproduces exactly. A body
// with any other face (a rectangular slab's side walls, a tilted plane, a freeform
// face) → false → NULL → OCCT, so the rebuild can never emit a solid of a DIFFERENT
// shape than the input (which the volume self-verify might otherwise wrongly accept).
inline bool isSteppedShaftAboutAxis(const topo::Shape& solid, const math::Ax3& ax, double Rc,
                                    double Rp) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  const math::Vec3 az = ax.z.vec();
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return false;
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder) {
      const auto info = cylinderInfo(solid, static_cast<int>(fi));
      if (!info) return false;
      if (std::fabs(std::fabs(math::dot(info->frame.z.vec(), az)) - 1.0) > 1e-6) return false;
      const math::Vec3 d = info->frame.origin - ax.origin;
      if (math::norm(d - az * math::dot(d, az)) > 1e-6) return false;
      if (std::fabs(info->radius - Rc) > 1e-6 && std::fabs(info->radius - Rp) > 1e-6) return false;
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, static_cast<int>(fi));
      if (!pl) return false;
      if (std::fabs(std::fabs(math::dot(pl->normal, az)) - 1.0) > 1e-6) return false;  // not ⟂ axis
    } else {
      return false;  // cone / sphere / freeform → defer
    }
  }
  return true;
}

// The concave stepped-shaft crease resolved in the boss-cylinder frame.
struct ConcaveGeom {
  math::Ax3 axis;        // boss cylinder frame (z = axis)
  double Rc = 0.0;       // boss radius
  double Rp = 0.0;       // plate (shoulder outer) radius
  double capH = 0.0;     // shoulder plane axial coord (the concave rim height)
  double bossTop = 0.0;  // boss cylinder's FAR rim axial coord
  double plateBottom = 0.0;  // plate cylinder's FAR rim axial coord
  double s = 1.0;        // axial sign from the shoulder toward the boss
};
inline std::optional<ConcaveGeom> concaveGeom(const topo::Shape& solid, const CylInfo& bossCyl,
                                              const RimCircle& rim) {
  ConcaveGeom g;
  g.axis = bossCyl.frame;
  g.Rc = bossCyl.radius;
  const math::Vec3 az = g.axis.z.vec();
  g.capH = math::dot(rim.centre - g.axis.origin, az);

  const auto rp = plateCylinderRadius(solid, g.axis, g.Rc);
  if (!rp) return std::nullopt;  // no larger coaxial cylinder → convex cap case, not concave
  g.Rp = *rp;

  // The body must be the strict stepped-shaft (so the analytic rebuild == the input).
  if (!isSteppedShaftAboutAxis(solid, g.axis, g.Rc, g.Rp)) return std::nullopt;

  double bmin = 0.0, bmax = 0.0;
  if (coaxialCircleHeights(solid, g.axis, g.Rc, bmin, bmax) < 2) return std::nullopt;
  g.bossTop = (std::fabs(bmax - g.capH) > std::fabs(bmin - g.capH)) ? bmax : bmin;

  double pmin = 0.0, pmax = 0.0;
  if (coaxialCircleHeights(solid, g.axis, g.Rp, pmin, pmax) < 2) return std::nullopt;
  // The plate's TOP rim must sit on the shoulder (share capH); otherwise the picked rim
  // is not the boss/shoulder crease (e.g. a blind-hole bottom, whose plate cylinder rims
  // are elsewhere) → defer to OCCT.
  if (std::fabs(pmin - g.capH) > 1e-6 && std::fabs(pmax - g.capH) > 1e-6) return std::nullopt;
  g.plateBottom = (std::fabs(pmax - g.capH) > std::fabs(pmin - g.capH)) ? pmax : pmin;

  g.s = (g.bossTop >= g.capH) ? 1.0 : -1.0;
  if (g.s * (g.plateBottom - g.capH) >= -1e-9) return std::nullopt;  // plate must be below shoulder
  return g;
}

// Assemble the concave-filleted stepped shaft as a planar-facet soup (CCW-from-front
// per facet; assembleSolid welds). Empty on any degeneracy → NULL → OCCT.
inline std::vector<nb::Polygon> buildConcaveFillet(const ConcaveGeom& g, double r, double defl) {
  const double Rc = g.Rc, Rp = g.Rp, s = g.s;
  const double capH = g.capH, bossTop = g.bossTop, plateBottom = g.plateBottom;
  const math::Ax3& ax = g.axis;
  const double R_t = Rc + r;             // torus major radius (material-side offset)
  const double hSeamWall = capH + s * r; // v=0 tangent circle on the boss wall
  const double zTube = hSeamWall;        // tube-centre axial height
  // Seam-inside-face guards (no ring-torus guard: R_t = Rc + r > r always).
  if (!(Rp >= R_t - 1e-9)) return {};                 // shoulder annulus reaches Rc+r
  if (!(s * (bossTop - hSeamWall) > 1e-9)) return {};  // boss wall covers the v=0 seam
  if (!(s * (capH - plateBottom) > 1e-9)) return {};   // plate lies below the shoulder

  const int N = sagittaSteps(Rp, kTwoPi, defl, 8, 256);      // angular
  const int M = sagittaSteps(r, kTwoPi / 4.0, defl, 3, 64);  // torus minor

  // Concave quarter-tube: radius(v) = R_t − r·cos v (Rc at v=0 → Rc+r at v=π/2),
  // axial(v) = zTube − s·r·sin v (capH+s·r at v=0 → capH at v=π/2). Outward (air-side)
  // normal = radial·cos v + axis·(s·sin v): radial at v=0, +s·axial at v=π/2.
  auto torusPoint = [&](double u, double v) -> math::Point3 {
    return ringPoint(ax, R_t - r * std::cos(v), u, zTube - s * r * std::sin(v));
  };
  auto torusNormal = [&](double u, double v) -> math::Vec3 {
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * std::cos(v) + ax.z.vec() * (s * std::sin(v));
  };

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (M + 3) + 4);

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
  // Each curved quad is split into two exactly-planar triangles carrying their own
  // geometric normal (oriented to the target outward), so every facet welds cleanly.
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

  const math::Vec3 upN = ax.z.vec() * s;         // shoulder / boss-top outward (toward boss)
  const math::Vec3 downN = ax.z.vec() * (-s);    // plate-bottom outward (away from boss)

  // 1. Plate bottom disk (radius Rp at plateBottom).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rp, kTwoPi * i / N, plateBottom));
    emit(std::move(ring), downN);
  }
  // 2. Plate outer wall: plateBottom → capH, N quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    emitQuad(ringPoint(ax, Rp, u0, plateBottom), ringPoint(ax, Rp, u1, plateBottom),
             ringPoint(ax, Rp, u1, capH), ringPoint(ax, Rp, u0, capH),
             ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um));
  }
  // 3. Shoulder annulus: inner radius R_t (=Rc+r, the v=π/2 seam) → outer Rp at capH.
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N;
    emitQuad(ringPoint(ax, R_t, u0, capH), ringPoint(ax, R_t, u1, capH),
             ringPoint(ax, Rp, u1, capH), ringPoint(ax, Rp, u0, capH), upN);
  }
  // 4. Concave torus quarter-tube: v ∈ [0, π/2] × u full turn, N·M quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    for (int j = 0; j < M; ++j) {
      const double v0 = (kTwoPi / 4.0) * j / M, v1 = (kTwoPi / 4.0) * (j + 1) / M;
      const double vm = 0.5 * (v0 + v1);
      emitQuad(torusPoint(u0, v0), torusPoint(u1, v0), torusPoint(u1, v1), torusPoint(u0, v1),
               torusNormal(um, vm));
    }
  }
  // 5. Boss wall: hSeamWall (v=0 seam) → bossTop, N quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    emitQuad(ringPoint(ax, Rc, u0, hSeamWall), ringPoint(ax, Rc, u1, hSeamWall),
             ringPoint(ax, Rc, u1, bossTop), ringPoint(ax, Rc, u0, bossTop),
             ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um));
  }
  // 6. Boss top cap (radius Rc at bossTop).
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rc, kTwoPi * i / N, bossTop));
    emit(std::move(ring), upN);
  }
  return polys;
}

// ── VARIABLE-RADIUS convex circular-rim fillet (swept variable-r torus canal) ───────
// Generalizes buildFilletedCylinder by promoting the constant ball radius r to a LINEAR
// law r(θ) = r1 + (r2−r1)·θ/(2π), θ∈[0,2π), around the SAME convex cylinder↔coaxial-cap
// rim. The rolling-ball centre is now a SWEPT curve (radial Rc−r(θ), axial capH−s·r(θ)),
// and the two trim seams are NON-circular:
//   * cylinder seam (v=0):  radius Rc, axial capH−s·r(θ)  — a helix;
//   * cap seam (v=π/2):     radius Rc−r(θ), axial capH    — an Archimedean spiral.
// The blend is a per-station upright meridian arc
//   radius(θ,v)=(Rc−r(θ))+r(θ)cos v,  axial(θ,v)=(capH−s·r(θ))+s·r(θ)sin v,  v∈[0,π/2],
// tiled into planar triangles. G1 holds at BOTH seams for ANY differentiable law: at v=0
// ∂radius/∂v=0 (normal radial == cylinder) and the helical seam tangent has zero radial
// component; at v=π/2 ∂axial/∂v=0 (normal axial == cap) and the spiral seam lies in the
// cap plane — so the blend normal equals the neighbour normal independent of r'(θ).
//
// The r-varies-with-θ law makes the two profiles at the azimuth-0 seam DIFFER (r1 vs r2),
// so the swept body is NOT a closed solid of revolution: a planar SEAM WALL in the
// azimuth-0 half-plane bridges the r1 and r2 meridians (the material step the larger
// fillet removes). To weld watertight with NO T-junction, the wall + cap are split at the
// DEEPEST station (rMax=max(r1,r2)): a closed lower wall (hFar→hLow=capH−s·rMax) + a
// helical band (hLow→capH−s·r_i) that tapers to zero at the rMax station, and a closed
// inner cap disk (radius Rin=Rc−rMax) + a spiral band (Rin→Rc−r_i). The seam-wall lens
// then shares its wall/cap struts exactly with the band edges of the min-r station.
// r1==r2 collapses every band + the seam wall to zero → byte-identical to the constant
// torus. Empty on any degeneracy → NULL → OCCT. Requires Rc ≥ 2·rMax (ring torus).
inline std::vector<nb::Polygon> buildVariableFilletedCylinder(const RimGeom& g, double r1,
                                                              double r2, double defl) {
  const double Rc = g.radius;
  const double rMax = std::max(r1, r2);
  const double Rin = Rc - rMax;               // inner (deepest) trimmed-cap radius
  if (!(Rc - 2.0 * rMax >= -1e-12)) return {};  // ring-torus guard at the deepest station
  if (!(Rin > 1e-9)) return {};
  const math::Ax3& ax = g.axis;

  const double s = (g.capH >= g.farH) ? 1.0 : -1.0;
  const double hCap = g.capH;
  const double hFar = g.farH;
  const double hLow = hCap - s * rMax;        // deepest wall→canal seam (rMax station)
  if (s * (hLow - hFar) <= 1e-9) return {};    // the far end must lie beyond the deepest seam

  // Angular stations: the major-radius sagitta bound PLUS a radius-gradient term so the
  // per-station seam step |r2−r1|/N stays ≤ defl (the swept canal never oversteps).
  const int Ngrad = static_cast<int>(std::ceil(std::fabs(r2 - r1) / std::max(defl, kCurveEps)));
  const int N = std::clamp(std::max(sagittaSteps(Rc, kTwoPi, defl, 8, 256), Ngrad), 8, 512);
  const int M = sagittaSteps(rMax, kTwoPi / 4.0, defl, 3, 64);   // canal meridian

  auto rAt = [&](int i) { return r1 + (r2 - r1) * (static_cast<double>(i) / N); };
  auto uAt = [&](int i) { return kTwoPi * i / N; };
  auto seamH = [&](int i) { return hCap - s * rAt(i); };   // helix (v=0) axial at station i
  auto capR = [&](int i) { return Rc - rAt(i); };          // spiral (v=π/2) radius at station i

  // Blend (canal) point at station i, meridian angle vAbs∈[0,π/2], and its outward normal.
  auto canalPoint = [&](int i, double vAbs) -> math::Point3 {
    const double ri = rAt(i);
    return ringPoint(ax, (Rc - ri) + ri * std::cos(vAbs), uAt(i), (hCap - s * ri) + s * ri * std::sin(vAbs));
  };
  auto canalNormal = [&](double u, double vAbs) -> math::Vec3 {
    const math::Vec3 radial = ax.x.vec() * std::cos(u) + ax.y.vec() * std::sin(u);
    return radial * std::cos(vAbs) + ax.z.vec() * (s * std::sin(vAbs));
  };

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * (M + 3) + 8);

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
  // Each curved quad → two exactly-planar triangles carrying their own geometric normal
  // (oriented to the target outward), so every facet welds cleanly. Degenerate (zero-area)
  // triangles — e.g. the bands at the rMax station — self-skip via nd.valid()==false.
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

  // 1. Far cap: full disk radius Rc at hFar, outward = −capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rc, kTwoPi * i / N, hFar));
    emit(std::move(ring), g.capNormal * -1.0);
  }
  // 2. Lower cylinder wall: hFar → hLow (closed ring), N quads.
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    emitQuad(ringPoint(ax, Rc, u0, hFar), ringPoint(ax, Rc, u1, hFar), ringPoint(ax, Rc, u1, hLow),
             ringPoint(ax, Rc, u0, hLow), ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um));
  }
  // 3. Helical band: hLow → seamH(i) (the v=0 canal seam), N quads (zero at the rMax station).
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    emitQuad(ringPoint(ax, Rc, u0, hLow), ringPoint(ax, Rc, u1, hLow),
             ringPoint(ax, Rc, u1, seamH(i + 1)), ringPoint(ax, Rc, u0, seamH(i)),
             ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um));
  }
  // 4. Variable canal quarter-tube: v ∈ [0, π/2] × the N stations, N·M quads.
  for (int i = 0; i < N; ++i) {
    const double um = 0.5 * (uAt(i) + uAt(i + 1));
    for (int j = 0; j < M; ++j) {
      const double v0 = (kTwoPi / 4.0) * j / M, v1 = (kTwoPi / 4.0) * (j + 1) / M;
      const double vm = 0.5 * (v0 + v1);
      emitQuad(canalPoint(i, v0), canalPoint(i + 1, v0), canalPoint(i + 1, v1), canalPoint(i, v1),
               canalNormal(um, vm));
    }
  }
  // 5. Spiral band: Rin → capR(i) at hCap, N quads (zero at the rMax station). Outward capNormal.
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1);
    emitQuad(ringPoint(ax, Rin, u0, hCap), ringPoint(ax, Rin, u1, hCap),
             ringPoint(ax, capR(i + 1), u1, hCap), ringPoint(ax, capR(i), u0, hCap), g.capNormal);
  }
  // 6. Inner trimmed cap: full disk radius Rin at hCap, outward = +capNormal.
  {
    std::vector<math::Point3> ring;
    ring.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) ring.push_back(ringPoint(ax, Rin, kTwoPi * i / N, hCap));
    emit(std::move(ring), g.capNormal);
  }
  // 7. Seam wall (load-bearing): the planar lens in the azimuth-0 half-plane bridging the
  //    station-0 (r1) and station-N (r2) meridians — the material step the larger fillet
  //    removes. Its wall strut (Q1_0→Q2_0) coincides with the min-r helical-band edge and
  //    its cap strut (Q1_M→Q2_M) with the min-r spiral-band edge, so it welds with no
  //    T-junction. Outward points toward the LARGER-r (more-removed / void) side.
  {
    const math::Vec3 seamOut = ax.y.vec() * (r1 > r2 ? 1.0 : -1.0);
    for (int j = 0; j < M; ++j) {
      const double v0 = (kTwoPi / 4.0) * j / M, v1 = (kTwoPi / 4.0) * (j + 1) / M;
      emitQuad(canalPoint(0, v0), canalPoint(0, v1), canalPoint(N, v1), canalPoint(N, v0), seamOut);
    }
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

// Fillet a single CONCAVE CIRCULAR crease (a boss cylinder ↔ a LARGER coaxial planar
// shoulder — a stepped shaft base rim) of `solid` with constant radius `r`. The rolling
// ball seats on the MATERIAL side (offset Rc + r), so the coaxial torus canal ADDS
// material and the enclosed volume GROWS (the engine gates it with wantGrow=true).
// Returns the filleted solid (faceted torus blend, deflection-bounded, watertight) or a
// NULL Shape (→ OCCT) when the edge is not a concave boss/shoulder rim, when a seam would
// leave its face, or on any degeneracy. Mutually exclusive with curved_fillet_edge (that
// one needs NO larger coaxial cylinder; this one requires exactly one). Multiple edges
// (edgeCount ≠ 1) → NULL.
inline topo::Shape concave_fillet_edge(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                       double r, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r > kBlendEps)) return {};
  const topo::ShapeMap emap = topo::mapShapes(solid, topo::ShapeType::Edge);
  if (edgeIds[0] < 1 || static_cast<std::size_t>(edgeIds[0]) > emap.size()) return {};
  const auto ce = topo::curveOf(emap.shape(edgeIds[0]));
  if (!ce || ce->curve->kind != topo::EdgeCurve::Kind::Circle) return {};

  const auto rim = detail::circleOf(solid, edgeIds[0]);
  if (!rim) return {};
  const math::Dir3 axisDir{rim->axis};
  if (!axisDir.valid()) return {};

  // Find the BOSS cylinder: any Cylinder face coaxial with the rim at the rim radius.
  // (A body-of-revolution splits the wall into angular sectors, so we do NOT require a
  // single face here — the stepped-shaft topology is validated wholesale by concaveGeom
  // / isSteppedShaftAboutAxis.)
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  std::optional<detail::CylInfo> bossCyl;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    if (detail::isRimCylinder(solid, static_cast<int>(fi), *rim, axisDir)) {
      bossCyl = detail::cylinderInfo(solid, static_cast<int>(fi));
      break;
    }
  }
  if (!bossCyl) return {};

  const auto g = detail::concaveGeom(solid, *bossCyl, *rim);
  if (!g) return {};  // not a concave stepped-shaft base rim (→ convex builder / OCCT)

  std::vector<nb::Polygon> polys = detail::buildConcaveFillet(*g, r, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

// Fillet a single CONVEX CIRCULAR crease (cylinder lateral face ↔ coaxial planar cap) of
// `solid` with a VARIABLE radius that varies LINEARLY around the rim,
// r(θ) = r1 + (r2−r1)·θ/(2π), θ∈[0,2π). The rolling-ball centre is a swept curve and the
// blend is a variable-radius torus canal, G1-tangent to both faces at the two NON-circular
// (helix / spiral) seams. Same convex cyl↔cap classification as curved_fillet_edge (only
// the radius law differs), so a variable fillet also REMOVES material. Returns the filleted
// solid (deflection-bounded planar-facet soup, watertight) or a NULL Shape (→ OCCT) when the
// edge is not a convex circular cylinder↔cap rim, when Rc < 2·max(r1,r2), or on any
// degeneracy. r1==r2 reduces exactly to the constant torus. Everything else — non-circular /
// non-linear laws, concave-variable rims, cyl↔cyl canals — returns NULL. Multiple picked
// edges (edgeCount ≠ 1) → NULL.
inline topo::Shape variable_fillet_edge(const topo::Shape& solid, const int* edgeIds, int edgeCount,
                                        double r1, double r2, double deflection = 0.01) {
  if (edgeIds == nullptr || edgeCount != 1 || !(r1 > kBlendEps) || !(r2 > kBlendEps)) return {};
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

  std::vector<nb::Polygon> polys = detail::buildVariableFilletedCylinder(*g, r1, r2, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_FILLET_H

// SPDX-License-Identifier: Apache-2.0
//
// curved_offset.h — MOAT M3 CURVED offset-face: offset the CYLINDRICAL LATERAL face of a
// capped cylinder RADIALLY (grow/shrink the tube radius Rc → Rc+d), the curved analogue of
// the planar `offset_face` (offset_face.h). The offset of a cylinder surface is a coaxial
// cylinder, so the operation is exactly analytic: the wall moves to radius Rc+d and the two
// coaxial planar caps grow/shrink to match, keeping the axial extent.
//
// ── WHY A CURVED ARM ────────────────────────────────────────────────────────────────
// The planar `offset_face` needs an ALL-PLANAR solid (PlanarModel), so a capped cylinder —
// which carries a Cylinder lateral face — declines to OCCT even when the picked face is the
// cylinder wall (the exact "offset a curved face" the app hits, readiness M3 @10 sites). A
// cylinder wall's outward normal is radial, so offsetting it by d is a pure radius change;
// the caps (perpendicular discs) simply widen/narrow to the new radius. No adjacent face
// tilts — the operation is a clean coaxial re-radius.
//
// ── REBUILD (planar-facet weld, tessellator-pristine) ───────────────────────────────
// The capped cylinder is rebuilt at the new radius Rc' = Rc + d as one deflection-bounded
// planar-facet soup sharing N angular samples across every seam (a wall band of N quads +
// two N-gon disc caps), welded watertight through the existing `nb::assembleSolid` — NO
// tessellator change. The engine self-verify (watertight + the correctly-signed volume
// change: grow d>0 → Vr>Vo, shrink d<0 → 0<Vr<Vo) then accepts it, else → OCCT.
//
// ── SCOPE (honest) ──────────────────────────────────────────────────────────────────
// Native only when the picked face is a CYLINDER lateral face of a PURE capped cylinder
// (every face a coaxial cylinder of the SAME axis/radius, or an axis-normal disc plane at
// exactly two heights), and Rc + d > 0 (a shrink cannot invert the tube). A stepped shaft /
// cone / sphere / multi-cylinder / tilted cap / a picked PLANAR face (owned by the planar
// arm) / a non-cylinder solid → NULL → OCCT (BRepOffsetAPI). The removed/added volume has a
// closed form (πΔ(Rc'²−Rc²)·H); the engine gates it with the two-sided volume + orientation
// self-verify. NEVER a wrong/leaky solid.
//
// CLEAN-ROOM. Reuses src/native/math + topology + boolean(assembleSolid) + blend_geom +
// curved_fillet(cylinderInfo/ringPoint/sagittaSteps). OCCT-FREE. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H
#define CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H

#include "native/blend/blend_geom.h"
#include "native/blend/curved_fillet.h"  // detail::cylinderInfo / ringPoint / sagittaSteps

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::blend {

namespace detail {

// The pure capped cylinder resolved about a picked Cylinder lateral face: the frame, the
// radius, and the two cap axial coords. Recognised WHOLESALE (a full-turn cylinder is one
// face here, but validating the whole body guarantees the analytic rebuild == the input, so
// the volume self-verify cannot wrongly accept a differently-shaped rebuild).
struct CappedCylGeom {
  math::Ax3 axis;    // cylinder frame (z = axis)
  double radius = 0.0;
  double hLo = 0.0;  // lower cap axial coord (along z from frame origin)
  double hHi = 0.0;  // upper cap axial coord
};

inline std::optional<CappedCylGeom> cappedCylGeom(const topo::Shape& solid, int faceId) {
  const auto info = cylinderInfo(solid, faceId);
  if (!info) return std::nullopt;  // picked face is not a Cylinder → planar arm / OCCT
  const math::Ax3 ax = info->frame;
  const math::Vec3 az = ax.z.vec();
  const double Rc = info->radius;
  if (!(Rc > kBlendEps)) return std::nullopt;

  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  std::vector<double> capH;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Cylinder) {
      // Every cylinder face must be THIS coaxial cylinder at radius Rc (a full wall may be
      // one face; a segmented revolve splits it into sectors that still share the frame).
      const auto ci = cylinderInfo(solid, static_cast<int>(fi));
      if (!ci) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(ci->frame.z.vec(), az)) - 1.0) > 1e-6) return std::nullopt;
      const math::Vec3 d = ci->frame.origin - ax.origin;
      if (math::norm(d - az * math::dot(d, az)) > 1e-6) return std::nullopt;  // not coaxial
      if (std::fabs(ci->radius - Rc) > 1e-6) return std::nullopt;             // a different R
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      const auto pl = facePlane(solid, static_cast<int>(fi));
      if (!pl) return std::nullopt;
      if (std::fabs(std::fabs(math::dot(pl->normal, az)) - 1.0) > 1e-6) return std::nullopt;  // ⟂ axis
      const double h = (pl->w - math::dot(pl->normal, ax.origin.asVec())) / math::dot(pl->normal, az);
      bool found = false;
      for (double e : capH)
        if (std::fabs(e - h) < 1e-6) { found = true; break; }
      if (!found) capH.push_back(h);
    } else {
      return std::nullopt;  // cone / sphere / freeform → not a pure capped cylinder
    }
  }
  if (capH.size() != 2) return std::nullopt;  // exactly two disc caps
  CappedCylGeom g;
  g.axis = ax;
  g.radius = Rc;
  g.hLo = std::min(capH[0], capH[1]);
  g.hHi = std::max(capH[0], capH[1]);
  if (!(g.hHi - g.hLo > kBlendEps)) return std::nullopt;
  return g;
}

// Rebuild the capped cylinder at radius `newR` as a planar-facet soup (empty on degeneracy).
// Wall band (N quads, each split into two planar triangles) + two disc caps; all share the
// SAME N angular samples so the wall→cap seams weld. Outward normals: wall radial, caps ±z.
inline std::vector<nb::Polygon> buildCappedCyl(const CappedCylGeom& g, double newR, double defl) {
  if (!(newR > kBlendEps)) return {};
  const math::Ax3& ax = g.axis;
  const double hLo = g.hLo, hHi = g.hHi;
  const int N = sagittaSteps(newR, kTwoPi, defl, 8, 256);

  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 3 + 4);

  auto emit = [&](std::vector<math::Point3> loop, const math::Vec3& outward) {
    const math::Dir3 nd{outward};
    if (!nd.valid() || loop.size() < 3) return;
    math::Vec3 area{0, 0, 0};
    for (std::size_t i = 0; i < loop.size(); ++i)
      area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
    if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
    polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(loop.front(), nd.vec()));
  };
  auto emitTri = [&](const math::Point3& a, const math::Point3& b, const math::Point3& c,
                     const math::Vec3& outward) {
    math::Vec3 nrm = math::cross(b - a, c - a);
    if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
    emit({a, b, c}, nrm);
  };

  // Wall band: hLo → hHi, N quads (two triangles each, kept exactly planar).
  for (int i = 0; i < N; ++i) {
    const double u0 = kTwoPi * i / N, u1 = kTwoPi * (i + 1) / N, um = 0.5 * (u0 + u1);
    const math::Point3 a0 = ringPoint(ax, newR, u0, hLo);
    const math::Point3 a1 = ringPoint(ax, newR, u1, hLo);
    const math::Point3 b1 = ringPoint(ax, newR, u1, hHi);
    const math::Point3 b0 = ringPoint(ax, newR, u0, hHi);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    emitTri(a0, a1, b1, outN);
    emitTri(a0, b1, b0, outN);
  }
  // Lower cap disk (outward = −z) and upper cap disk (outward = +z).
  {
    std::vector<math::Point3> lo, hi;
    lo.reserve(static_cast<std::size_t>(N));
    hi.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) {
      lo.push_back(ringPoint(ax, newR, kTwoPi * i / N, hLo));
      hi.push_back(ringPoint(ax, newR, kTwoPi * i / N, hHi));
    }
    emit(std::move(lo), ax.z.vec() * -1.0);
    emit(std::move(hi), ax.z.vec());
  }
  return polys;
}

}  // namespace detail

// Offset the CYLINDRICAL LATERAL face `faceId` (1-based, mapShapes order) of a capped
// cylinder RADIALLY by `distance` (grow d>0 → radius Rc+d, shrink d<0 → Rc+d, must stay
// > 0). Returns the re-radiused capped cylinder (deflection-bounded planar-facet soup,
// watertight) or a NULL Shape (→ OCCT) when the picked face is not a cylinder wall of a
// pure capped cylinder, when Rc + distance ≤ 0, or on any degeneracy. A picked PLANAR face
// is left to the planar `offset_face` arm; a cone / sphere / stepped / multi-cylinder body
// → NULL.
inline topo::Shape curved_offset_face(const topo::Shape& solid, int faceId, double distance,
                                      double deflection = 0.01) {
  if (std::fabs(distance) < kBlendEps) return {};
  const auto g = detail::cappedCylGeom(solid, faceId);
  if (!g) return {};
  const double newR = g->radius + distance;
  if (!(newR > kBlendEps)) return {};  // a shrink cannot invert the tube
  std::vector<nb::Polygon> polys = detail::buildCappedCyl(*g, newR, deflection);
  if (polys.size() < 4) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::blend

#endif  // CYBERCAD_NATIVE_BLEND_CURVED_OFFSET_H

// SPDX-License-Identifier: Apache-2.0
//
// project.h — MOAT M-DM DM4: the native `project_point_on_face` (the app's project
// tool — drop a 3-D point onto a face's underlying analytic surface, returning the
// foot-of-perpendicular and the min distance). This is the CLOSED-FORM normal
// projection for the three analytic surface kinds the app's project / measure path
// needs, mirroring OCCT `GeomAPI_ProjectPointOnSurf` on the face's untrimmed surface:
//
//   * PLANE     — foot = P − ((P−o)·n̂)n̂ ; distance = |(P−o)·n̂|.
//   * CYLINDER  — split P−o into axial + radial; push the radial component onto the
//                 radius: foot = o + axial + (R/ρ)·radial ; distance = |ρ − R|.
//   * SPHERE    — foot = c + (R/ρ)(P−c) ; distance = |ρ − R|.
//
// ── HONEST DECLINE (never a fabricated foot) ─────────────────────────────────────
// A CONE / TORUS / freeform (BSpline/Bezier) face is out of this analytic slice and
// returns nullopt (→ engine reports the decline, falls to OCCT). The AMBIGUOUS pose
// where the foot is a whole circle / undefined — a point ON a cylinder axis or AT a
// sphere centre — also declines (no single closed-form foot). The engine NEVER hands
// a native void to OCCT and NEVER emits an unverified foot.
//
// Projection is onto the face's INFINITE analytic surface (matching
// GeomAPI_ProjectPointOnSurf on the untrimmed Geom_Surface), not the trimmed patch.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20. Additive sibling —
// touches no landed DM1/DM2 header.
//
#ifndef CYBERCAD_NATIVE_DIRECTMODEL_PROJECT_H
#define CYBERCAD_NATIVE_DIRECTMODEL_PROJECT_H

#include "native/math/elementary.h"
#include "native/math/native_math.h"
#include "native/topology/accessors.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstddef>
#include <optional>

namespace cybercad::native::directmodel {

namespace pmath = cybercad::native::math;
namespace ptopo = cybercad::native::topology;

// Measured reason a native projection declined (→ engine reports it, falls to OCCT).
enum class ProjectDecline {
  Ok = 0,
  ForeignBody,       // the face id is out of range / the shape carries no B-rep face
  NonAnalyticFace,   // cone / torus / freeform surface — out of the analytic slice
  DegenerateSurface, // a zero-radius / malformed analytic surface
  Ambiguous,         // point on the cylinder axis / at the sphere centre (no single foot)
};

// The closed-form projection of a point onto an analytic surface.
struct ProjectionResult {
  pmath::Point3 foot;      // the foot-of-perpendicular on the surface
  double distance = 0.0;   // the (unsigned) minimum distance point → surface
};

namespace projdetail {

inline constexpr double kTinyRadial = 1e-9;  // "point on the axis / at the centre" band

// World-place a local Ax3 by a topology Location (identity fast-path).
inline pmath::Ax3 placeFrame(const pmath::Ax3& f, const ptopo::Location& loc) {
  if (loc.isIdentity()) return f;
  const pmath::Transform& t = loc.transform();
  return pmath::Ax3{t.applyToPoint(f.origin), t.applyToDir(f.x), t.applyToDir(f.y),
                    t.applyToDir(f.z)};
}

}  // namespace projdetail

// ─────────────────────────────────────────────────────────────────────────────
// Project `p` onto the analytic surface of face `faceId` (1-based, mapShapes Face
// order — the same ids the facade/OCCT path uses). Returns the closed-form foot +
// distance, or nullopt (with a measured `ProjectDecline`) for an honest decline.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<ProjectionResult> projectPointOnFace(const ptopo::Shape& solid, int faceId,
                                                          const pmath::Point3& p,
                                                          ProjectDecline* why = nullptr) {
  using K = ptopo::FaceSurface::Kind;
  auto fail = [&](ProjectDecline d) -> std::optional<ProjectionResult> {
    if (why) *why = d;
    return std::nullopt;
  };

  const ptopo::ShapeMap map = ptopo::mapShapes(solid, ptopo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > map.size())
    return fail(ProjectDecline::ForeignBody);
  const ptopo::Shape& face = map.shape(faceId);
  const auto surf = ptopo::surfaceOf(face);
  if (!surf) return fail(ProjectDecline::ForeignBody);

  const pmath::Ax3 fr = projdetail::placeFrame(surf->surface->frame, surf->location);
  const double R = surf->surface->radius;
  const pmath::Vec3 v = p - fr.origin;
  ProjectionResult out;

  switch (surf->surface->kind) {
    case K::Plane: {
      const pmath::Vec3 n = fr.z.vec();
      const double s = pmath::dot(v, n);
      out.foot = p - n * s;
      out.distance = std::fabs(s);
      break;
    }
    case K::Cylinder: {
      if (!(R > projdetail::kTinyRadial)) return fail(ProjectDecline::DegenerateSurface);
      const pmath::Vec3 axis = fr.z.vec();
      const double h = pmath::dot(v, axis);            // axial component
      const pmath::Vec3 radial = v - axis * h;         // radial component
      const double rho = pmath::norm(radial);
      if (!(rho > projdetail::kTinyRadial))            // P on the axis — foot is a circle
        return fail(ProjectDecline::Ambiguous);
      out.foot = fr.origin + axis * h + radial * (R / rho);
      out.distance = std::fabs(rho - R);
      break;
    }
    case K::Sphere: {
      if (!(R > projdetail::kTinyRadial)) return fail(ProjectDecline::DegenerateSurface);
      const double rho = pmath::norm(v);
      if (!(rho > projdetail::kTinyRadial))            // P at the centre — foot is a sphere
        return fail(ProjectDecline::Ambiguous);
      out.foot = fr.origin + v * (R / rho);
      out.distance = std::fabs(rho - R);
      break;
    }
    default:  // Cone / Torus / BSpline / Bezier — out of the analytic slice.
      return fail(ProjectDecline::NonAnalyticFace);
  }

  if (why) *why = ProjectDecline::Ok;
  return out;
}

}  // namespace cybercad::native::directmodel

#endif  // CYBERCAD_NATIVE_DIRECTMODEL_PROJECT_H

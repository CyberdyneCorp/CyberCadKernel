// SPDX-License-Identifier: Apache-2.0
//
// angle.h — GS3 angle between two B-rep entities. OCCT-FREE.
//
// Only the three well-defined cells have a single angle:
//   * line·line  θ = acos(|d_a·d_b|) ∈ [0, π/2]   (unsigned; parallel → 0)
//   * plane·plane θ = acos(clamp(n_a·n_b,−1,1)) ∈ [0, π]  (oriented normals)
//   * line·plane θ = asin(|d·n|) ∈ [0, π/2]       (angle to the plane; 0 = in-plane)
//
// HONEST DECLINE (std::nullopt): any entity that is not a Line edge / Plane face
// (a general curve or curved surface has no single angle), or a degenerate
// direction ‖d‖ ≈ 0 — the service NEVER fabricates an angle.
//
// Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_ANALYSIS_ANGLE_H
#define CYBERCAD_NATIVE_ANALYSIS_ANGLE_H

#include <algorithm>
#include <cmath>
#include <optional>

#include "native/analysis/native_analysis.h"

namespace cybercad::native::analysis {

namespace detail {

/// Unit direction of a Line edge (its frame X), or std::nullopt if not a line /
/// degenerate.
inline std::optional<Vec3> lineDir(const Entity& e) {
  if (e.kind != Entity::Kind::Edge ||
      e.edge->kind != topo::EdgeCurve::Kind::Line)
    return std::nullopt;
  const math::Dir3& d = e.edge->frame.x;
  if (!d.valid()) return std::nullopt;
  return d.vec();
}

/// Unit normal of a Plane face (its frame Z), or std::nullopt if not a plane /
/// degenerate.
inline std::optional<Vec3> planeNormal(const Entity& e) {
  if (e.kind != Entity::Kind::Face ||
      e.face->kind != topo::FaceSurface::Kind::Plane)
    return std::nullopt;
  const math::Dir3& n = e.face->frame.z;
  if (!n.valid()) return std::nullopt;
  return n.vec();
}

}  // namespace detail

/// Angle (radians) between two entities, or DECLINE for any non-line/plane pair.
inline std::optional<double> angle(const Entity& a, const Entity& b) {
  const auto da = detail::lineDir(a), db = detail::lineDir(b);
  const auto na = detail::planeNormal(a), nb = detail::planeNormal(b);

  if (da && db)  // line · line → acute angle between directions
    return std::acos(std::min(1.0, std::fabs(dot(*da, *db))));

  if (na && nb)  // plane · plane → angle between oriented normals
    return std::acos(std::clamp(dot(*na, *nb), -1.0, 1.0));

  if (da && nb)  // line · plane
    return std::asin(std::min(1.0, std::fabs(dot(*da, *nb))));
  if (na && db)  // plane · line
    return std::asin(std::min(1.0, std::fabs(dot(*db, *na))));

  return std::nullopt;  // non-line/plane entity or degenerate direction
}

}  // namespace cybercad::native::analysis

#endif  // CYBERCAD_NATIVE_ANALYSIS_ANGLE_H

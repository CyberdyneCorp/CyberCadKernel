// SPDX-License-Identifier: Apache-2.0
//
// tolerance.h — shared tolerances + small geometric predicates for the SSI S1
// handlers. Values mirror the scale OCCT's IntAna_QuadQuadGeo uses to decide
// parallel/perpendicular/coincident (angular ~1e-8..1e-6, linear ~Precision).
//
// Header-only, OCCT-FREE.
//
#ifndef CYBERCAD_NATIVE_SSI_TOLERANCE_H
#define CYBERCAD_NATIVE_SSI_TOLERANCE_H

#include "native/math/vec.h"

#include <cmath>

namespace cybercad::native::ssi {

namespace math = cybercad::native::math;

/// Linear tolerance for "point on plane / same locus" tests (model units).
inline constexpr double kLinearEps = 1e-9;

/// Angular tolerance (radians) for parallel / perpendicular axis classification.
/// A |sin θ| or |cos θ| below this is treated as exactly aligned.
inline constexpr double kAngularEps = 1e-7;

/// True iff two unit directions are parallel (same or opposite) within kAngularEps.
inline bool parallelDirs(const math::Vec3& a, const math::Vec3& b) noexcept {
  return math::norm(math::cross(a, b)) <= kAngularEps;
}

/// True iff two unit directions are perpendicular within kAngularEps.
inline bool perpendicularDirs(const math::Vec3& a, const math::Vec3& b) noexcept {
  return std::fabs(math::dot(a, b)) <= kAngularEps;
}

/// Signed distance from point `p` to the line through `o` with unit direction `d`,
/// as the length of the rejection (always ≥ 0).
inline double distancePointLine(const math::Point3& p, const math::Point3& o,
                                const math::Vec3& d) noexcept {
  const math::Vec3 w = p - o;
  return math::norm(math::cross(w, d));
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_TOLERANCE_H

// SPDX-License-Identifier: Apache-2.0
//
// same_surface.h — closed-form "are these two elementary surfaces the SAME LOCUS?"
// predicates (SSI Stage S4-a, the analytic coincidence family).
//
// Each predicate decides `FullSurfaceSame` from the surface FRAMES + SIZES alone, in
// closed form, using the shared linear/angular tolerances. They GENERALISE the partial
// coincidence detection already living in quadric_pairs.h / plane_conics.h (same-sphere,
// coaxial-equal cylinder, same plane) into one consistent family across ALL elementary
// kinds:
//
//   plane/plane      — same normal (± sign) AND same signed offset.
//   sphere/sphere    — same centre AND equal radius.
//   cylinder/cylinder— collinear axes (parallel dir + zero axis-to-axis distance) AND
//                      equal radius.
//   cone/cone        — same apex, collinear axis, equal half-angle. (A right circular
//                      cone's locus is double-napped and axis-flip symmetric, so the axis
//                      may be parallel OR antiparallel.)
//   torus/torus      — same centre, collinear axis, equal major AND minor radius. (The
//                      torus locus is axis-flip symmetric, so antiparallel axis is fine.)
//
// A NEAR-MISS (shifted / rotated / resized beyond tolerance) is NOT the same locus and
// each predicate returns false — never a false coincidence. These are pure geometry;
// the caller (dispatch.h `classify_degeneracy`) wraps a true verdict as
// `CoincidentRegion::fullSurfaceSame()`.
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE. clang++ -std=c++20. Uses src/native/math only.
//
#ifndef CYBERCAD_NATIVE_SSI_SAME_SURFACE_H
#define CYBERCAD_NATIVE_SSI_SAME_SURFACE_H

#include "native/math/elementary.h"
#include "native/math/torus.h"
#include "native/ssi/tolerance.h"

#include <cmath>

namespace cybercad::native::ssi {

/// Two points are the same within the linear tolerance.
inline bool samePoint(const Point3& a, const Point3& b) noexcept {
  return math::distance(a, b) <= kLinearEps;
}

/// Two radii/lengths are equal within the linear tolerance.
inline bool sameLength(double a, double b) noexcept {
  return std::fabs(a - b) <= kLinearEps;
}

/// Two axis LINES are collinear: parallel directions (same or opposite) AND the second
/// origin lies on the first's axis line (zero perpendicular distance). Used for cylinder,
/// cone (via apex) and torus, whose loci are axis-flip symmetric.
inline bool collinearAxes(const Point3& o1, const Dir3& d1,
                          const Point3& o2, const Dir3& d2) noexcept {
  return parallelDirs(d1.vec(), d2.vec()) &&
         distancePointLine(o2, o1, d1.vec()) <= kLinearEps;
}

// ─────────────────────────────────────────────────────────────────────────────
// FullSurfaceSame predicates — one per elementary family.
// ─────────────────────────────────────────────────────────────────────────────

/// Same plane: normals parallel (± sign) AND the second origin lies on the first plane
/// (signed offset along the normal is zero).
inline bool samePlane(const math::Plane& a, const math::Plane& b) noexcept {
  if (!parallelDirs(a.pos.z.vec(), b.pos.z.vec())) return false;
  const double offset = math::dot(b.pos.origin - a.pos.origin, a.pos.z.vec());
  return std::fabs(offset) <= kLinearEps;
}

/// Same sphere: same centre AND equal radius. (Folds the same-sphere case that
/// intersectSphereSphere reports as Coincident.)
inline bool sameSphere(const math::Sphere& a, const math::Sphere& b) noexcept {
  return samePoint(a.pos.origin, b.pos.origin) && sameLength(a.radius, b.radius);
}

/// Same cylinder: collinear axes AND equal radius. (Folds the coaxial-equal case that
/// intersectCylinderCylinder reports as Coincident.)
inline bool sameCylinder(const math::Cylinder& a, const math::Cylinder& b) noexcept {
  return collinearAxes(a.pos.origin, a.pos.z, b.pos.origin, b.pos.z) &&
         sameLength(a.radius, b.radius);
}

/// Apex of a right circular cone: the point on the axis where the surface radius
/// (radius + v·sinα) is zero, i.e. v = −radius/sinα, at axial offset v·cosα.
inline Point3 coneApex(const math::Cone& c) noexcept {
  const double sa = std::sin(c.semiAngle);
  if (std::fabs(sa) <= kAngularEps) return c.pos.origin;  // degenerate (≈ cylinder)
  const double vApex = -c.radius / sa;
  return c.pos.origin + c.pos.z.vec() * (vApex * std::cos(c.semiAngle));
}

/// Same cone: equal half-angle, collinear axis, AND coincident apex. The locus is
/// double-napped and axis-flip symmetric, so `collinearAxes` (parallel OR antiparallel)
/// through the shared apex is the right test.
inline bool sameCone(const math::Cone& a, const math::Cone& b) noexcept {
  if (std::fabs(a.semiAngle - b.semiAngle) > kAngularEps) return false;
  const Point3 apexA = coneApex(a);
  const Point3 apexB = coneApex(b);
  if (!samePoint(apexA, apexB)) return false;
  return collinearAxes(apexA, a.pos.z, apexB, b.pos.z);
}

/// Same torus: same centre, collinear axis, equal major AND minor radius. The locus is
/// axis-flip symmetric, so antiparallel axes still describe the same torus.
inline bool sameTorus(const math::Torus& a, const math::Torus& b) noexcept {
  return samePoint(a.pos.origin, b.pos.origin) &&
         collinearAxes(a.pos.origin, a.pos.z, b.pos.origin, b.pos.z) &&
         sameLength(a.majorRadius, b.majorRadius) &&
         sameLength(a.minorRadius, b.minorRadius);
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_SAME_SURFACE_H

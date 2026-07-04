// SPDX-License-Identifier: Apache-2.0
//
// plane_torus.h — analytic plane ∩ torus intersector (SSI Stage S1), for the two
// closed-form orientations. The general OBLIQUE plane cuts a torus in a planar
// QUARTIC (a "spiric section" of Perseus) with no rational conic decomposition —
// per SSI-ROADMAP S1 that oblique case is honestly DEFERRED (NotAnalytic → S2/S3
// marching / OCCT). We handle exactly the orientations that decompose into native
// circles:
//
//   (A) plane PERPENDICULAR to the torus axis (n ∥ axis), at axial height t from the
//       torus centre. The tube's sin v = t/r fixes v (if |t| ≤ r), giving up to two
//       cos v values ⇒ up to two concentric CIRCLES of radius R ± r·cos v centred on
//       the axis in the cutting plane. |t| = r → one circle (radius R). |t| > r →
//       Empty.
//
//   (B) plane CONTAINING the axis (n ⟂ axis and the axis lies in the plane): the
//       section is the two generating tube CIRCLES, radius r, centred at ±R along the
//       in-plane radial direction from the torus centre (the classic "two circles"
//       meridian section).
//
// Torus (matching torus.h):
//   S(u,v) = O + (R + r·cos v)(cos u·X + sin u·Y) + r·sin v·Z
//   axis = Z, R = majorRadius, r = minorRadius.
//
// CLEAN-ROOM. IntAna_QuadQuadGeo(Pln,Torus) consulted only as an ORACLE for which
// orientations yield circles vs the deferred quartic.
//
// Header-only, OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_PLANE_TORUS_H
#define CYBERCAD_NATIVE_SSI_PLANE_TORUS_H

#include "native/math/torus.h"
#include "native/ssi/curve.h"
#include "native/ssi/tolerance.h"

#include <cmath>
#include <vector>

namespace cybercad::native::ssi {

inline IntersectionResult intersectPlaneTorus(const math::Plane& pl, const math::Torus& to) {
  const Dir3 n = pl.pos.z;
  const Vec3 axis = to.pos.z.vec();
  const double R = to.majorRadius, r = to.minorRadius;
  const Point3 O = to.pos.origin;

  const bool nParallelAxis = parallelDirs(n.vec(), axis);
  const bool nPerpAxis = perpendicularDirs(n.vec(), axis);

  // ── (A) plane ⟂ axis → concentric circle(s). ────────────────────────────────
  if (nParallelAxis) {
    // Signed axial height of the plane relative to the torus centre.
    const double t = math::dot(pl.pos.origin - O, axis);
    if (std::fabs(t) > r + kLinearEps) return IntersectionResult::empty();
    const double sinv = std::max(-1.0, std::min(1.0, t / r));
    const double cosvMag = std::sqrt(std::max(0.0, 1.0 - sinv * sinv));
    // Circle plane is the cutting plane (normal = axis) at axial height t.
    const Point3 centre = O + axis * t;
    const Dir3 xref = orthogonalTo(Dir3{axis});
    std::vector<IntersectionCurve> circles;
    if (cosvMag <= kLinearEps) {  // tube top/bottom: single circle radius R
      circles.push_back(makeCircle(centre, R, Dir3{axis}, xref));
    } else {
      // cos v = ±cosvMag → radii R ± r·cosvMag (outer & inner ring circles).
      const double rOuter = R + r * cosvMag;
      const double rInner = R - r * cosvMag;
      circles.push_back(makeCircle(centre, rOuter, Dir3{axis}, xref));
      if (rInner > kLinearEps)
        circles.push_back(makeCircle(centre, rInner, Dir3{axis}, xref));
    }
    return IntersectionResult::ok(std::move(circles));
  }

  // ── (B) plane CONTAINS the axis → two meridian tube circles. ────────────────
  // Requires n ⟂ axis AND the axis line lies in the plane (the torus centre is on the
  // plane, since the axis passes through O).
  if (nPerpAxis) {
    const double centreGap = math::dot(O - pl.pos.origin, n.vec());
    if (std::fabs(centreGap) > kLinearEps) {
      // n ⟂ axis but the plane is offset from the centre — a genuine spiric quartic.
      return IntersectionResult::notAnalytic();
    }
    // In-plane radial direction = the plane direction ⟂ the axis (the meridian line).
    // The plane contains `axis`; its other in-plane direction is m = n × axis.
    const Dir3 m{math::cross(n.vec(), axis)};
    // Two tube circles: centres at O ± R·m, radius r, lying in this plane (normal n),
    // with the tube's own frame (v measured from the outer equator, cos v along m).
    std::vector<IntersectionCurve> circles;
    circles.push_back(makeCircle(O + m.vec() * R, r, n, m));
    circles.push_back(makeCircle(O - m.vec() * R, r, n, m.reversed()));
    return IntersectionResult::ok(std::move(circles));
  }

  // ── Oblique plane → planar quartic spiric section: deferred per S1 scope. ────
  return IntersectionResult::notAnalytic();
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_PLANE_TORUS_H

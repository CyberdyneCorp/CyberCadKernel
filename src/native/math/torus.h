// SPDX-License-Identifier: Apache-2.0
//
// torus.h — analytic torus surface (surface of revolution of a circle whose
// centre is OFF the revolution axis).
//
// Clean-room, parametrization matching OCCT ElSLib
// (/Users/leonardoaraujo/work/OCCT/src/FoundationClasses/TKMath/ElSLib, the
// TorusValue / TorusD1 / TorusNormal arms) so (u,v) parameters agree with the
// oracle:
//
//   S(u,v) = O + (R + r·cos v)·(cos u·X + sin u·Y) + r·sin v·Z
//
//   * u ∈ [0, 2π)  — the MAJOR (revolution) angle about the frame Z axis.
//   * v ∈ [0, 2π)  — the MINOR (tube) angle around the circular cross-section.
//   * R (majorRadius) — distance from the axis (frame origin, along Z) to the
//     centre of the tube circle, measured in the frame's X–Y plane.
//   * r (minorRadius) — radius of the tube circle itself.
//
// The placement Ax3 = {origin O, orthonormal X, Y, Z=axis}, right-handed
// (X × Y = Z). At v = 0 the point is on the OUTER equator (radius R + r); at
// v = π it is on the INNER equator (radius R − r). The outward normal points
// away from the tube's centreline circle (radially out of the tube), matching
// OCCT's outward orientation for a right-handed frame.
//
// This is exactly the surface a native REVOLVE of an off-axis circular arc sweeps
// (see src/native/construct/residuals.h build_revolution_torus): the generatrix
// is the tube circle (radius r about the arc centre, which sits at distance R from
// the axis) and revolving it about the axis produces the torus. The analytic form
// here provides value / dU / dV / normal for verification and future direct meshing;
// the construction builder emits the same surface as an EXACT rational-quadratic
// B-spline patch (which the current tessellator meshes without a new surface kind).
//
// Header-only, OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_TORUS_H
#define CYBERCAD_NATIVE_MATH_TORUS_H

#include "elementary.h"  // Ax3, frameCombine
#include "vec.h"

#include <cmath>

namespace cybercad::native::math {

// ─── Torus ─────────────────────────────────────────────────────────────────---
// A torus of major radius R and minor radius r placed by the frame `pos`.
//
// Degeneracy note: r > 0 is required for a well-formed tube; R may be ≥ r
// (ring torus, the only case a revolve-of-an-off-axis-arc produces — the arc must
// clear the axis) or 0 < R < r (self-intersecting "spindle" torus) which the
// construction builder rejects up front. This struct evaluates the parametrization
// for whatever R, r it is given; validity is the caller's responsibility.
struct Torus {
  Ax3 pos;
  double majorRadius = 1.0;  ///< R: axis → tube-centre distance (in the X–Y plane)
  double minorRadius = 0.5;  ///< r: tube cross-section radius

  /// S(u,v).
  Point3 value(double u, double v) const noexcept {
    const double rr = majorRadius + minorRadius * std::cos(v);
    return frameCombine(pos, rr * std::cos(u), rr * std::sin(u), minorRadius * std::sin(v));
  }

  /// ∂S/∂u — tangent along the MAJOR circle (revolution direction).
  Vec3 dU(double u, double v) const noexcept {
    const double rr = majorRadius + minorRadius * std::cos(v);
    return pos.x.vec() * (-rr * std::sin(u)) + pos.y.vec() * (rr * std::cos(u));
  }

  /// ∂S/∂v — tangent along the MINOR (tube) circle.
  Vec3 dV(double u, double v) const noexcept {
    const double s = minorRadius * std::sin(v);
    return pos.x.vec() * (-s * std::cos(u)) + pos.y.vec() * (-s * std::sin(u)) +
           pos.z.vec() * (minorRadius * std::cos(v));
  }

  /// Outward unit normal = (S − C(u))/r, where C(u) is the tube-centre point on the
  /// MAJOR circle at angle u. Equivalent to normalize(∂S/∂v × ∂S/∂u) for R+r·cos v>0
  /// (a ring torus); computed directly for robustness at the tube's top/bottom.
  Dir3 normal(double u, double v) const noexcept {
    return Dir3{pos.x.vec() * (std::cos(v) * std::cos(u)) +
                pos.y.vec() * (std::cos(v) * std::sin(u)) + pos.z.vec() * std::sin(v)};
  }
};

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_TORUS_H

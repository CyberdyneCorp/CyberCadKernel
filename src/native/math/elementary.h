// SPDX-License-Identifier: Apache-2.0
//
// elementary.h — analytic elementary surfaces: plane, cylinder, cone, sphere.
//
// Clean-room, parametrizations matching OCCT ElSLib
// (/Users/leonardoaraujo/work/OCCT/src/FoundationClasses/TKMath/ElSLib) so
// (u,v) parameters agree with the oracle:
//   Plane:    S(u,v) = O + u·X + v·Y
//   Cylinder: S(u,v) = O + R(cos u·X + sin u·Y) + v·Z
//   Cone:     S(u,v) = O + (R + v·sin α)(cos u·X + sin u·Y) + v·cos α·Z
//   Sphere:   S(u,v) = O + R·cos v(cos u·X + sin u·Y) + R·sin v·Z
// where the placement Ax3 = {origin O, orthonormal X, Y, Z=axis}, right-handed
// (X × Y = Z). Normals point outward (away from the axis/center) for a
// right-handed frame, matching OCCT's outward orientation.
//
// Header-only, constexpr-friendly (except the trig, which is runtime).
// OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_ELEMENTARY_H
#define CYBERCAD_NATIVE_MATH_ELEMENTARY_H

#include "vec.h"

#include <cmath>

namespace cybercad::native::math {

/// Right-handed local frame: origin + orthonormal axes (X, Y, Z=axis).
/// Mirrors gp_Ax3. Construction does not force orthonormality — the caller is
/// expected to supply an orthonormal frame (as OCCT geometry does).
struct Ax3 {
  Point3 origin{};
  Dir3 x{1, 0, 0};
  Dir3 y{0, 1, 0};
  Dir3 z{0, 0, 1};  // the "Direction" / main axis

  /// Build a right-handed frame from an origin, a main axis and a reference X.
  /// Y is derived as z × x. Any component of xRef along z is removed first.
  static Ax3 fromAxisAndRef(const Point3& o, const Dir3& axis, const Dir3& xRef) noexcept {
    const Vec3 zx = axis.vec();
    Vec3 xp = xRef.vec() - zx * dot(xRef.vec(), zx);  // Gram-Schmidt
    const Dir3 xn{xp};
    const Dir3 yn{cross(zx, xn.vec())};
    return Ax3{o, xn, yn, axis};
  }
};

// Helper: O + a·X + b·Y + c·Z.
inline Point3 frameCombine(const Ax3& f, double a, double b, double c) noexcept {
  return f.origin + f.x.vec() * a + f.y.vec() * b + f.z.vec() * c;
}

// ─── Plane ───────────────────────────────────────────────────────────────────
struct Plane {
  Ax3 pos;

  Point3 value(double u, double v) const noexcept { return frameCombine(pos, u, v, 0.0); }
  /// Outward normal is the plane's Z axis (constant).
  Dir3 normal(double /*u*/, double /*v*/) const noexcept { return pos.z; }
  Vec3 dU(double, double) const noexcept { return pos.x.vec(); }
  Vec3 dV(double, double) const noexcept { return pos.y.vec(); }
};

// ─── Cylinder ─────────────────────────────────────────────────────────────────
struct Cylinder {
  Ax3 pos;
  double radius = 1.0;

  Point3 value(double u, double v) const noexcept {
    return frameCombine(pos, radius * std::cos(u), radius * std::sin(u), v);
  }
  Vec3 dU(double u, double /*v*/) const noexcept {
    return pos.x.vec() * (-radius * std::sin(u)) + pos.y.vec() * (radius * std::cos(u));
  }
  Vec3 dV(double, double) const noexcept { return pos.z.vec(); }
  /// Outward radial normal.
  Dir3 normal(double u, double /*v*/) const noexcept {
    return Dir3{pos.x.vec() * std::cos(u) + pos.y.vec() * std::sin(u)};
  }
};

// ─── Cone ─────────────────────────────────────────────────────────────────────
// `radius` is the reference radius at v=0; `semiAngle` α is the half-angle.
struct Cone {
  Ax3 pos;
  double radius = 1.0;
  double semiAngle = 0.0;

  Point3 value(double u, double v) const noexcept {
    const double r = radius + v * std::sin(semiAngle);
    return frameCombine(pos, r * std::cos(u), r * std::sin(u), v * std::cos(semiAngle));
  }
  Vec3 dU(double u, double v) const noexcept {
    const double r = radius + v * std::sin(semiAngle);
    return pos.x.vec() * (-r * std::sin(u)) + pos.y.vec() * (r * std::cos(u));
  }
  Vec3 dV(double u, double /*v*/) const noexcept {
    const double sa = std::sin(semiAngle), ca = std::cos(semiAngle);
    return pos.x.vec() * (sa * std::cos(u)) + pos.y.vec() * (sa * std::sin(u)) + pos.z.vec() * ca;
  }
  Dir3 normal(double u, double v) const noexcept { return Dir3{cross(dU(u, v), dV(u, v))}; }
};

// ─── Sphere ───────────────────────────────────────────────────────────────────
// u ∈ [0,2π) longitude, v ∈ [-π/2, π/2] latitude.
struct Sphere {
  Ax3 pos;
  double radius = 1.0;

  Point3 value(double u, double v) const noexcept {
    const double rc = radius * std::cos(v);
    return frameCombine(pos, rc * std::cos(u), rc * std::sin(u), radius * std::sin(v));
  }
  Vec3 dU(double u, double v) const noexcept {
    const double rc = radius * std::cos(v);
    return pos.x.vec() * (-rc * std::sin(u)) + pos.y.vec() * (rc * std::cos(u));
  }
  Vec3 dV(double u, double v) const noexcept {
    const double rs = radius * std::sin(v);
    return pos.x.vec() * (-rs * std::cos(u)) + pos.y.vec() * (-rs * std::sin(u)) +
           pos.z.vec() * (radius * std::cos(v));
  }
  /// Outward radial normal = (S − center)/R.
  Dir3 normal(double u, double v) const noexcept {
    return Dir3{value(u, v) - pos.origin};
  }
};

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_ELEMENTARY_H

// SPDX-License-Identifier: Apache-2.0
//
// quadric_pairs.h — analytic quadric∩quadric intersectors that reduce to circles /
// lines in CLOSED FORM (SSI Stage S1). Every returned curve provably lies on both
// surfaces.
//
// SCOPE (honest — only the closed-form sub-families):
//   sphere ∩ sphere            → Circle / Point / Empty / Coincident  (ALWAYS analytic)
//   coaxial sphere ∩ cylinder  → 0/1/2 Circles (latitude circles where R_s²−z²=R_c²)
//   coaxial sphere ∩ cone      → 0/1/2 Circles (where the cone radius meets the sphere)
//   coaxial cylinder ∩ cylinder→ Coincident / Empty (same axis) — no isolated curve
//   parallel cylinder ∩ cylinder→ 0/1/2 ruling Lines (axes parallel, offset by δ)
//   coaxial cylinder ∩ cone    → 0/1/2 Circles (where the cone radius equals R_cyl)
//
// NOT here (return NotAnalytic → S2/S3 marching / OCCT): skew cylinder∩cylinder
// (a genuine quartic space curve), general cone∩cone, any non-coaxial sphere/cone
// combination that is not one of the above.
//
// CLEAN-ROOM. IntAna_QuadQuadGeo consulted as an ORACLE for the circle-count logic
// only.
//
// Header-only, OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_QUADRIC_PAIRS_H
#define CYBERCAD_NATIVE_SSI_QUADRIC_PAIRS_H

#include "native/math/elementary.h"
#include "native/ssi/curve.h"
#include "native/ssi/tolerance.h"

#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::ssi {

// ─────────────────────────────────────────────────────────────────────────────
// sphere ∩ sphere → a Circle (always closed-form).
//
// Centres C1, C2, radii R1, R2, d = |C2 − C1|, axis u = (C2 − C1)/d. The
// intersection circle lies in the plane ⟂ u at signed distance
//   x = (d² + R1² − R2²) / (2d)   from C1 along u,
// with radius ρ = √(R1² − x²). d = 0 & R1 = R2 → same sphere (Coincident); other
// d = 0 → Empty (concentric, different radii). ρ² < 0 → Empty; ρ = 0 → tangent Point.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectSphereSphere(const math::Sphere& s1, const math::Sphere& s2) {
  const Vec3 c = s2.pos.origin - s1.pos.origin;
  const double d = math::norm(c);
  const double R1 = s1.radius, R2 = s2.radius;

  if (d <= kLinearEps) {
    return (std::fabs(R1 - R2) <= kLinearEps) ? IntersectionResult::coincident()
                                              : IntersectionResult::empty();
  }
  if (d > R1 + R2 + kLinearEps) return IntersectionResult::empty();       // too far
  if (d < std::fabs(R1 - R2) - kLinearEps) return IntersectionResult::empty();  // nested
  const Dir3 u{c};
  const double x = (d * d + R1 * R1 - R2 * R2) / (2.0 * d);
  const double rho2 = R1 * R1 - x * x;
  const Point3 centre = s1.pos.origin + u.vec() * x;
  if (rho2 <= kLinearEps * kLinearEps) return IntersectionResult::ok(makePoint(centre));
  return IntersectionResult::ok(makeCircle(centre, std::sqrt(rho2), u, orthogonalTo(u)));
}

// ─────────────────────────────────────────────────────────────────────────────
// coaxial sphere ∩ cylinder → latitude Circle(s).
//
// Coaxial: the cylinder axis passes through the sphere centre. Put z along the axis
// with origin at the sphere centre. Sphere: x²+y²+z² = R_s²  ⇒ ring radius² = R_s²−z².
// Cylinder: ring radius = R_c. So z² = R_s² − R_c². If R_c > R_s → Empty; R_c = R_s →
// one tangent circle at z=0 (the equator); else two circles at z = ±√(R_s²−R_c²).
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectSphereCylinderCoaxial(const math::Sphere& sp,
                                                         const math::Cylinder& cy) {
  const Dir3 axis = cy.pos.z;
  // Coaxiality: sphere centre lies on the cylinder axis line.
  if (distancePointLine(sp.pos.origin, cy.pos.origin, axis.vec()) > kLinearEps)
    return IntersectionResult::notAnalytic();

  const double Rs = sp.radius, Rc = cy.radius;
  const double disc = Rs * Rs - Rc * Rc;
  if (disc < -kLinearEps) return IntersectionResult::empty();  // cylinder wider than sphere
  const Dir3 xref = orthogonalTo(axis);

  if (std::fabs(disc) <= kLinearEps) {  // tangent equator: one circle radius Rc at centre
    return IntersectionResult::ok(makeCircle(sp.pos.origin, Rc, axis, xref));
  }
  const double z = std::sqrt(disc);
  std::vector<IntersectionCurve> circles;
  circles.push_back(makeCircle(sp.pos.origin + axis.vec() * z, Rc, axis, xref));
  circles.push_back(makeCircle(sp.pos.origin - axis.vec() * z, Rc, axis, xref));
  return IntersectionResult::ok(std::move(circles));
}

// ─────────────────────────────────────────────────────────────────────────────
// coaxial cylinder ∩ cylinder → same-axis Coincident/Empty, OR parallel-axis rulings.
//
// COAXIAL (axes collinear): equal radii → Coincident (identical lateral surface);
// unequal → Empty (nested tubes never meet). No isolated curve either way.
//
// PARALLEL (axes parallel, offset by δ ⟂ the axis): two circular cylinders of radii
// R1, R2 whose axes are δ apart intersect in straight generators parallel to the
// axis. In the cross-section plane this is two circles (centres δ apart) → 0/1/2
// points, each lifting to a ruling Line. Points: with the two axis feet A1, A2 and
// m = (A2 − A1)/δ (unit, ⟂ axis), x = (δ² + R1² − R2²)/(2δ) from A1, h² = R1² − x².
// h < 0 → Empty; h = 0 → 1 tangent ruling; else 2 rulings at A1 + x·m ± h·(axis×m).
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectCylinderCylinder(const math::Cylinder& c1,
                                                    const math::Cylinder& c2) {
  const Vec3 a1 = c1.pos.z.vec();
  const Vec3 a2 = c2.pos.z.vec();
  if (!parallelDirs(a1, a2)) return IntersectionResult::notAnalytic();  // skew → quartic → S2/S3

  // Offset vector between the two axis lines, projected ⟂ the (shared) axis a1.
  const Vec3 w = c2.pos.origin - c1.pos.origin;
  const Vec3 perp = w - a1 * math::dot(w, a1);  // component ⟂ the axis
  const double delta = math::norm(perp);

  if (delta <= kLinearEps) {  // COAXIAL
    return (std::fabs(c1.radius - c2.radius) <= kLinearEps) ? IntersectionResult::coincident()
                                                            : IntersectionResult::empty();
  }

  // PARALLEL, offset δ. Work in the cross-section: foot of c1 axis is c1.origin's
  // projection; use A1 = c1.origin (any axis point), A2 = A1 + perp.
  const Dir3 m{perp};                          // unit offset direction ⟂ axis
  const double R1 = c1.radius, R2 = c2.radius;
  const double x = (delta * delta + R1 * R1 - R2 * R2) / (2.0 * delta);
  const double h2 = R1 * R1 - x * x;
  if (h2 < -kLinearEps) return IntersectionResult::empty();
  const Point3 base = c1.pos.origin + m.vec() * x;
  const Dir3 tang{math::cross(a1, m.vec())};   // in cross-section, ⟂ m

  if (h2 <= kLinearEps) {  // tangent → single ruling
    return IntersectionResult::ok(makeLine(base, Dir3{a1}));
  }
  const double h = std::sqrt(h2);
  std::vector<IntersectionCurve> lines;
  lines.push_back(makeLine(base + tang.vec() * h, Dir3{a1}));
  lines.push_back(makeLine(base - tang.vec() * h, Dir3{a1}));
  return IntersectionResult::ok(std::move(lines));
}

// ─────────────────────────────────────────────────────────────────────────────
// coaxial cylinder ∩ cone → Circle(s).
//
// Coaxial: the cone axis is collinear with the cylinder axis. Along the axis (z from
// the cone apex V, measured so radius = |z|·tanα... but our Cone stores radius R0 at
// v=0 and grows by sinα per unit v), the cone's cross-section radius at axial height
// is r(z). It equals the cylinder radius R_c at up to two heights → up to two
// circles of radius R_c. We solve r = R_c on the cone directly using its
// parametrization: at parameter v the cone radius is (R0 + v·sinα). Setting it to
// R_c gives v = (R_c − R0)/sinα (one solution per nappe sign of the radius). Because
// a right circular cone's radius is |R0 + v sinα|, both +R_c and the mirrored nappe
// can appear; we emit the physically-valid circle(s) with radius R_c.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectCylinderConeCoaxial(const math::Cylinder& cy,
                                                       const math::Cone& co) {
  const Dir3 axis = co.pos.z;
  if (!parallelDirs(cy.pos.z.vec(), axis.vec())) return IntersectionResult::notAnalytic();
  // Coaxial: cylinder axis line passes through the cone origin.
  if (distancePointLine(co.pos.origin, cy.pos.origin, axis.vec()) > kLinearEps)
    return IntersectionResult::notAnalytic();

  const double sa = std::sin(co.semiAngle), ca = std::cos(co.semiAngle);
  if (std::fabs(sa) <= kAngularEps) return IntersectionResult::notAnalytic();  // ~cylinder
  const double Rc = cy.radius, R0 = co.radius;
  const Dir3 xref = orthogonalTo(axis);

  // A right circular cone is DOUBLE-NAPPED: the native Cone parametrizes the surface
  // radius as (R0 + v·sinα), which passes through 0 at the apex and goes NEGATIVE on
  // the opposite nappe — a negative signed radius is a real point on the mirror nappe
  // at spatial radius |R0 + v·sinα|. The coaxial cylinder (radius Rc > 0) therefore
  // meets the cone wherever the signed cone radius equals +Rc OR −Rc, i.e. on BOTH
  // nappes → up to two circles, both of spatial radius Rc. (Skipping the −Rc target,
  // as an earlier version did, dropped the opposite-nappe circle that OCCT reports.)
  std::vector<IntersectionCurve> circles;
  for (double target : {Rc, -Rc}) {
    const double v = (target - R0) / sa;         // signed cone radius == target here
    const double effR = std::fabs(target);       // spatial radius of the ring (= Rc)
    const Point3 centre = co.pos.origin + axis.vec() * (v * ca);
    circles.push_back(makeCircle(centre, effR < kLinearEps ? 0.0 : effR, axis, xref));
  }
  if (circles.empty()) return IntersectionResult::empty();
  return IntersectionResult::ok(std::move(circles));
}

// ─────────────────────────────────────────────────────────────────────────────
// coaxial sphere ∩ cone → Circle(s).
//
// Coaxial: cone axis passes through the sphere centre. Parametrize the cone by v:
// axial height from the cone origin is h(v) = v·cosα (along the axis), cross-section
// radius r(v) = R0 + v·sinα. A point on the cone at parameter v lies on the sphere iff
//   |axialOffsetFromSphereCentre|² + r(v)² = R_s².
// Let the cone origin sit at signed axial coord o0 from the sphere centre along the
// axis; then axial coord of the ring is z(v) = o0 + v·cosα, and we solve
//   z(v)² + r(v)² = R_s²  →  a quadratic in v. Real roots (with r ≥ 0) give circles
// of radius r(v) at z(v). This is closed-form (quadratic), no external solver needed.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersectSphereConeCoaxial(const math::Sphere& sp,
                                                     const math::Cone& co) {
  const Dir3 axis = co.pos.z;
  if (distancePointLine(sp.pos.origin, co.pos.origin, axis.vec()) > kLinearEps)
    return IntersectionResult::notAnalytic();

  const double sa = std::sin(co.semiAngle), ca = std::cos(co.semiAngle);
  const double R0 = co.radius, Rs = sp.radius;
  // o0 = signed axial coord of the cone origin relative to the sphere centre.
  const double o0 = math::dot(co.pos.origin - sp.pos.origin, axis.vec());

  // z(v) = o0 + v ca ; r(v) = R0 + v sa. Solve z² + r² = Rs²:
  //   (ca² + sa²) v² + 2(o0 ca + R0 sa) v + (o0² + R0² − Rs²) = 0, ca²+sa²=1.
  const double A = 1.0;
  const double B = 2.0 * (o0 * ca + R0 * sa);
  const double C = o0 * o0 + R0 * R0 - Rs * Rs;
  const double disc = B * B - 4.0 * A * C;
  if (disc < -kLinearEps) return IntersectionResult::empty();
  const Dir3 xref = orthogonalTo(axis);
  const double sq = std::sqrt(std::max(0.0, disc));

  // The cone is DOUBLE-NAPPED (signed radius R0 + v·sinα crosses 0 at the apex and is
  // negative on the mirror nappe — a real ring at spatial radius |R0 + v·sinα|). Each
  // quadratic root in v is therefore a genuine intersection circle regardless of the
  // sign of its radius; a coaxial sphere spanning the apex meets both nappes → two
  // circles. (An earlier version skipped the negative-radius root, dropping the
  // opposite-nappe circle that OCCT reports.)
  auto circleAt = [&](double v) -> std::optional<IntersectionCurve> {
    const double r = R0 + v * sa;               // signed cone radius at this height
    const double z = o0 + v * ca;
    const Point3 centre = sp.pos.origin + axis.vec() * z;
    return makeCircle(centre, std::fabs(r) < kLinearEps ? 0.0 : std::fabs(r), axis, xref);
  };

  std::vector<IntersectionCurve> circles;
  if (std::fabs(disc) <= kLinearEps) {
    if (auto c = circleAt(-B / (2.0 * A))) circles.push_back(*c);
  } else {
    if (auto c = circleAt((-B + sq) / (2.0 * A))) circles.push_back(*c);
    if (auto c = circleAt((-B - sq) / (2.0 * A))) circles.push_back(*c);
  }
  if (circles.empty()) return IntersectionResult::empty();
  return IntersectionResult::ok(std::move(circles));
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_QUADRIC_PAIRS_H

// SPDX-License-Identifier: Apache-2.0
//
// tangent_analytic.h — closed-form ANALYTIC tangent-contact classifiers (SSI Stage
// S4-b). Given a pair of elementary surfaces in a tangent configuration, decide — in
// CLOSED FORM from the surface frames + sizes — whether the contact is an isolated
// TangentPoint or a TangentCurve, and EMIT the point / conic. Analytic tangency is
// always DECIDABLE: a right-quadric tangent config is either a single touch point or a
// tangent curve, never a near-tangent transversal or an ambiguous jet — so these
// classifiers never return `NearTangentTransversal` / `Undecided` (those arise only on
// the seeded differential-geometry path, where curvature noise can blur the verdict).
//
// The decidable analytic tangent families (mirroring the S1 intersect handlers, which
// already reduce these to a `CurveKind::Point` or a single tangent conic):
//
//   sphere ∩ sphere       d = R₁+R₂ (external) or d = |R₁−R₂| (internal, d>0)
//                         → TangentPoint on the centre line.
//   plane  ∩ sphere       |signed dist centre→plane| = R
//                         → TangentPoint (the foot of the centre on the plane).
//   coaxial sphere∩cyl    R_c = R_s (cylinder touches the sphere's equator)
//                         → TangentCurve (the equator Circle).
//   plane ∩ cylinder      axis ∥ plane AND axis-to-plane distance = R
//                         → TangentCurve (the single tangent ruling Line).
//   coaxial sphere∩cone   the sphere touches the cone on one latitude (double root)
//                         → TangentCurve (a Circle).
//   coaxial cylinder∩cone the cone radius equals R_cyl at a grazing height (double root)
//                         → TangentCurve (a Circle).
//
// METHOD. We do NOT re-derive the conics here — we call the existing S1 handler for the
// pair and READ its result: a single `CurveKind::Point` branch IS a tangent point, and a
// single tangent conic (produced only in a tangent configuration, detected by the
// closed-form tangency predicate for that pair) IS a tangent curve. This keeps one source
// of truth for the geometry (the S1 handlers) and adds only the tangency PREDICATE that
// distinguishes "this single curve is a tangency" from "this single curve is a transversal
// section". A pair that is not in a tangent configuration → TransversalOnly.
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_TANGENT_ANALYTIC_H
#define CYBERCAD_NATIVE_SSI_TANGENT_ANALYTIC_H

#include "native/math/elementary.h"
#include "native/ssi/curve.h"
#include "native/ssi/quadric_pairs.h"
#include "native/ssi/plane_conics.h"
#include "native/ssi/tangent_contact.h"
#include "native/ssi/tolerance.h"

#include <cmath>

namespace cybercad::native::ssi {

// ── sphere ∩ sphere ─────────────────────────────────────────────────────────────
// Tangent iff d = R₁+R₂ (external touch) or d = |R₁−R₂| with d > 0 (internal touch).
// Both reduce intersectSphereSphere to a single CurveKind::Point at the touch point.
inline TangentContact tangentSphereSphere(const math::Sphere& s1, const math::Sphere& s2) {
  const double d = math::distance(s1.pos.origin, s2.pos.origin);
  const double sum = s1.radius + s2.radius;
  const double diff = std::fabs(s1.radius - s2.radius);
  const bool external = std::fabs(d - sum) <= kLinearEps;
  const bool internal = d > kLinearEps && std::fabs(d - diff) <= kLinearEps;
  if (!external && !internal) return TangentContact::transversal();

  const IntersectionResult r = intersectSphereSphere(s1, s2);
  if (r.ok_() && r.curves.size() == 1 && r.curves[0].kind == CurveKind::Point)
    return TangentContact::tangentPoint(r.curves[0].point, 0.0);
  return TangentContact::transversal();  // predicate said tangent but handler disagreed → safe
}

// ── plane ∩ sphere ──────────────────────────────────────────────────────────────
// Tangent iff |signed distance centre→plane| = R. intersectPlaneSphere returns a
// single CurveKind::Point there (the foot of the centre on the plane).
inline TangentContact tangentPlaneSphere(const math::Plane& pl, const math::Sphere& sp) {
  const double h = math::dot(sp.pos.origin - pl.pos.origin, pl.pos.z.vec());
  if (std::fabs(std::fabs(h) - sp.radius) > kLinearEps) return TangentContact::transversal();
  const IntersectionResult r = intersectPlaneSphere(pl, sp);
  if (r.ok_() && r.curves.size() == 1 && r.curves[0].kind == CurveKind::Point)
    return TangentContact::tangentPoint(r.curves[0].point, 0.0);
  return TangentContact::transversal();
}

// ── coaxial sphere ∩ cylinder ────────────────────────────────────────────────────
// Coaxial (axis through the sphere centre) AND R_c = R_s: the cylinder touches the
// sphere along its equator (a single tangent Circle). intersectSphereCylinderCoaxial
// returns exactly one Circle in that case (disc = R_s²−R_c² = 0).
inline TangentContact tangentSphereCylinder(const math::Sphere& sp, const math::Cylinder& cy) {
  if (distancePointLine(sp.pos.origin, cy.pos.origin, cy.pos.z.vec()) > kLinearEps)
    return TangentContact::transversal();  // not coaxial → not this analytic tangent family
  if (std::fabs(sp.radius - cy.radius) > kLinearEps) return TangentContact::transversal();
  const IntersectionResult r = intersectSphereCylinderCoaxial(sp, cy);
  if (r.ok_() && r.curves.size() == 1) return TangentContact::tangentCurve(r.curves[0], 0.0);
  return TangentContact::transversal();
}

// ── plane ∩ cylinder ─────────────────────────────────────────────────────────────
// Axis ∥ plane (n ⟂ axis) AND the axis-to-plane distance = R: the plane grazes the
// cylinder along a SINGLE ruling line (tangent). intersectPlaneCylinder returns one
// Line there (disc = R²−D² = 0 in the parallel branch).
inline TangentContact tangentPlaneCylinder(const math::Plane& pl, const math::Cylinder& cy) {
  const Vec3 a = cy.pos.z.vec();
  const Vec3 n = pl.pos.z.vec();
  if (std::fabs(math::dot(n, a)) > kAngularEps) return TangentContact::transversal();  // not ∥
  const double D = math::dot(cy.pos.origin - pl.pos.origin, n);  // axis-to-plane signed dist
  if (std::fabs(std::fabs(D) - cy.radius) > kLinearEps) return TangentContact::transversal();
  const IntersectionResult r = intersectPlaneCylinder(pl, cy);
  if (r.ok_() && r.curves.size() == 1 && r.curves[0].kind == CurveKind::Line)
    return TangentContact::tangentCurve(r.curves[0], 0.0);
  return TangentContact::transversal();
}

// ── coaxial sphere ∩ cone ────────────────────────────────────────────────────────
// Coaxial and the sphere grazes the cone on one latitude (the sphere∩cone quadratic in
// v has a DOUBLE root). intersectSphereConeCoaxial then returns exactly one Circle — a
// tangent latitude circle. Two distinct circles ⇒ transversal (two proper sections).
inline TangentContact tangentSphereCone(const math::Sphere& sp, const math::Cone& co) {
  if (distancePointLine(sp.pos.origin, co.pos.origin, co.pos.z.vec()) > kLinearEps)
    return TangentContact::transversal();
  const double sa = std::sin(co.semiAngle), ca = std::cos(co.semiAngle);
  const double R0 = co.radius, Rs = sp.radius;
  const double o0 = math::dot(co.pos.origin - sp.pos.origin, co.pos.z.vec());
  const double B = 2.0 * (o0 * ca + R0 * sa);
  const double C = o0 * o0 + R0 * R0 - Rs * Rs;
  const double disc = B * B - 4.0 * C;
  if (std::fabs(disc) > kLinearEps) return TangentContact::transversal();  // 0 or 2 sections
  const IntersectionResult r = intersectSphereConeCoaxial(sp, co);
  if (r.ok_() && r.curves.size() == 1) return TangentContact::tangentCurve(r.curves[0], 0.0);
  return TangentContact::transversal();
}

// ── coaxial cylinder ∩ cone ──────────────────────────────────────────────────────
// Coaxial and the cone's signed radius meets +R_c and −R_c at the SAME axial height
// (grazing): the two roots v = (±R_c − R₀)/sinα coincide only when R_c = 0, so a
// genuine coaxial cyl∩cone with R_c > 0 always has two distinct circles → transversal.
// The tangent sub-case (a single grazing circle) does not arise for R_c > 0; we report
// TransversalOnly (honest — no analytic tangent curve here).
inline TangentContact tangentCylinderCone(const math::Cylinder& /*cy*/, const math::Cone& /*co*/) {
  return TangentContact::transversal();
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_TANGENT_ANALYTIC_H

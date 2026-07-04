// SPDX-License-Identifier: Apache-2.0
//
// dispatch.h — the SSI Stage-S1 surface-pair CLASSIFIER + router.
//
// A `Surface` is a tagged union over the native elementary surfaces (Plane,
// Cylinder, Cone, Sphere, Torus). `intersect_surfaces(A, B)` classifies the
// (surface-type, relative-orientation) pair and routes to the matching closed-form
// handler, or returns NotAnalytic for anything outside the S1 family (skew
// cylinder∩cylinder, general cone∩cone, torus∩curved, NURBS/freeform, oblique
// plane∩torus, …) — never faking a curve.
//
// ORDER-INDEPENDENT: intersect_surfaces(A,B) == intersect_surfaces(B,A). The handlers
// have a fixed argument order (e.g. plane first); the dispatcher canonicalises the
// pair before calling, so callers may pass operands in any order.
//
// Header-only, OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_DISPATCH_H
#define CYBERCAD_NATIVE_SSI_DISPATCH_H

#include "native/math/elementary.h"
#include "native/math/torus.h"
#include "native/ssi/coincidence.h"
#include "native/ssi/curve.h"
#include "native/ssi/plane_conics.h"
#include "native/ssi/plane_torus.h"
#include "native/ssi/quadric_pairs.h"
#include "native/ssi/same_surface.h"
#include "native/ssi/tangent_analytic.h"
#include "native/ssi/tangent_contact.h"

#include <variant>

namespace cybercad::native::ssi {

/// The kinds of surface S1 can classify. Freeform (Bézier/B-spline/NURBS) surfaces
/// are deliberately NOT in this enum: a query involving one is NotAnalytic by
/// construction (the caller passes them through a different, non-S1 path).
enum class SurfaceKind { Plane, Cylinder, Cone, Sphere, Torus };

/// A tagged elementary surface. The variant holds the concrete native-math surface
/// object; `kind()` reports its tag for the dispatcher.
struct Surface {
  std::variant<math::Plane, math::Cylinder, math::Cone, math::Sphere, math::Torus> geom;

  SurfaceKind kind() const noexcept {
    return static_cast<SurfaceKind>(geom.index());
  }

  static Surface of(const math::Plane& p) { return {p}; }
  static Surface of(const math::Cylinder& c) { return {c}; }
  static Surface of(const math::Cone& c) { return {c}; }
  static Surface of(const math::Sphere& s) { return {s}; }
  static Surface of(const math::Torus& t) { return {t}; }

  const math::Plane& plane() const { return std::get<math::Plane>(geom); }
  const math::Cylinder& cylinder() const { return std::get<math::Cylinder>(geom); }
  const math::Cone& cone() const { return std::get<math::Cone>(geom); }
  const math::Sphere& sphere() const { return std::get<math::Sphere>(geom); }
  const math::Torus& torus() const { return std::get<math::Torus>(geom); }
};

namespace detail {

// Ordering rank so we can canonicalise a pair to a fixed (lo,hi) handler order,
// independent of the caller's argument order. Plane < Sphere < Cylinder < Cone <
// Torus — chosen so each supported handler's first argument is the lower rank
// (plane first everywhere; sphere before cylinder/cone; cylinder before cone).
inline int rank(SurfaceKind k) noexcept {
  switch (k) {
    case SurfaceKind::Plane:    return 0;
    case SurfaceKind::Sphere:   return 1;
    case SurfaceKind::Cylinder: return 2;
    case SurfaceKind::Cone:     return 3;
    case SurfaceKind::Torus:    return 4;
  }
  return 5;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// intersect_surfaces — the S1 dispatch. Canonicalises the pair by `rank`, then
// routes to the closed-form handler. Any pair not enumerated below is NotAnalytic.
// ─────────────────────────────────────────────────────────────────────────────
inline IntersectionResult intersect_surfaces(const Surface& A, const Surface& B) {
  // Canonicalise: `lo` has the not-greater rank, `hi` the other.
  const Surface& lo = detail::rank(A.kind()) <= detail::rank(B.kind()) ? A : B;
  const Surface& hi = detail::rank(A.kind()) <= detail::rank(B.kind()) ? B : A;
  const SurfaceKind lk = lo.kind(), hk = hi.kind();

  using K = SurfaceKind;

  // ── plane ∩ * ───────────────────────────────────────────────────────────────
  if (lk == K::Plane) {
    switch (hk) {
      case K::Plane:    return intersectPlanePlane(lo.plane(), hi.plane());
      case K::Sphere:   return intersectPlaneSphere(lo.plane(), hi.sphere());
      case K::Cylinder: return intersectPlaneCylinder(lo.plane(), hi.cylinder());
      case K::Cone:     return intersectPlaneCone(lo.plane(), hi.cone());
      case K::Torus:    return intersectPlaneTorus(lo.plane(), hi.torus());
    }
  }

  // ── sphere ∩ * ────────────────────────────────────────────────────────────────
  if (lk == K::Sphere) {
    switch (hk) {
      case K::Sphere:   return intersectSphereSphere(lo.sphere(), hi.sphere());
      case K::Cylinder: return intersectSphereCylinderCoaxial(lo.sphere(), hi.cylinder());
      case K::Cone:     return intersectSphereConeCoaxial(lo.sphere(), hi.cone());
      default: break;  // sphere ∩ torus → NotAnalytic
    }
  }

  // ── cylinder ∩ * ──────────────────────────────────────────────────────────────
  if (lk == K::Cylinder) {
    switch (hk) {
      case K::Cylinder: return intersectCylinderCylinder(lo.cylinder(), hi.cylinder());
      case K::Cone:     return intersectCylinderConeCoaxial(lo.cylinder(), hi.cone());
      default: break;  // cylinder ∩ torus → NotAnalytic
    }
  }

  // cone ∩ cone, cone ∩ torus, torus ∩ torus, and any freeform pair: not S1.
  return IntersectionResult::notAnalytic();
}

// ─────────────────────────────────────────────────────────────────────────────
// classify_degeneracy — the SSI Stage-S4-a ANALYTIC coincidence classifier (sibling
// to intersect_surfaces; order-independent).
//
// Decides, in CLOSED FORM from the surface frames + sizes, whether A and B are the SAME
// LOCUS (`CoincidentRegion::fullSurfaceSame()`), returning `none()` otherwise. Only
// SAME-KIND elementary pairs can be a full-surface coincidence — a plane is never the
// same locus as a cone, a sphere never the same locus as a cylinder, etc. — so a
// mixed-kind pair is `None` by construction. This GENERALISES the partial `Coincident`
// detection already in intersect_surfaces (same-sphere, coaxial-equal cyl, same plane)
// into the complete elementary family and backs it with the typed region; the shipped
// `IntersectionStatus::Coincident` results are UNCHANGED (this is an additive sibling).
//
// The seeded OverlapSubRegion / Undecided outcomes are produced on the S2 path
// (seeding.cpp, under CYBERCAD_HAS_NUMSCI); this analytic classifier only ever returns
// `FullSurfaceSame` or `None` — an exact, closed-form decision, never `Undecided`.
// ─────────────────────────────────────────────────────────────────────────────
inline CoincidentRegion classify_degeneracy(const Surface& A, const Surface& B) {
  if (A.kind() != B.kind()) return CoincidentRegion::none();

  bool same = false;
  switch (A.kind()) {
    case SurfaceKind::Plane:    same = samePlane(A.plane(), B.plane()); break;
    case SurfaceKind::Sphere:   same = sameSphere(A.sphere(), B.sphere()); break;
    case SurfaceKind::Cylinder: same = sameCylinder(A.cylinder(), B.cylinder()); break;
    case SurfaceKind::Cone:     same = sameCone(A.cone(), B.cone()); break;
    case SurfaceKind::Torus:    same = sameTorus(A.torus(), B.torus()); break;
  }
  return same ? CoincidentRegion::fullSurfaceSame() : CoincidentRegion::none();
}

// ─────────────────────────────────────────────────────────────────────────────
// classify_tangency — the SSI Stage-S4-b ANALYTIC tangent-contact classifier (sibling
// to intersect_surfaces / classify_degeneracy; order-independent).
//
// Decides, in CLOSED FORM, whether A and B meet in a TANGENT configuration and, if so,
// whether the contact is an isolated `TangentPoint` (e.g. spheres at d = R₁+R₂) or a
// `TangentCurve` (e.g. a coaxial cylinder tangent to a sphere's equator, a plane tangent
// to a cylinder along a ruling). A pair not in a tangent configuration → `TransversalOnly`.
//
// Analytic tangency is exact and decidable: a right-quadric tangent config is a single
// touch point or a tangent conic, never a `NearTangentTransversal` or an ambiguous jet —
// so this classifier NEVER returns `NearTangentTransversal` / `Undecided` (those arise
// only on the seeded differential-geometry path). It NEVER marches through a tangency and
// NEVER fabricates a curve across a degeneracy — it only READS the closed-form S1 result.
//
// This complements `classify_degeneracy`: coincidence (same locus) is a 2D shared region;
// a tangency is a 0D/1D degenerate contact. A `FullSurfaceSame` pair is NOT a tangency
// (the surfaces are identical, not touching) — classify_tangency reports TransversalOnly
// for a same-locus pair; the caller distinguishes coincidence via classify_degeneracy.
// ─────────────────────────────────────────────────────────────────────────────
inline TangentContact classify_tangency(const Surface& A, const Surface& B) {
  const Surface& lo = detail::rank(A.kind()) <= detail::rank(B.kind()) ? A : B;
  const Surface& hi = detail::rank(A.kind()) <= detail::rank(B.kind()) ? B : A;
  const SurfaceKind lk = lo.kind(), hk = hi.kind();
  using K = SurfaceKind;

  if (lk == K::Plane) {
    switch (hk) {
      case K::Sphere:   return tangentPlaneSphere(lo.plane(), hi.sphere());
      case K::Cylinder: return tangentPlaneCylinder(lo.plane(), hi.cylinder());
      default: break;   // plane∩plane, plane∩cone, plane∩torus: no S4-b analytic tangent family
    }
  } else if (lk == K::Sphere) {
    switch (hk) {
      case K::Sphere:   return tangentSphereSphere(lo.sphere(), hi.sphere());
      case K::Cylinder: return tangentSphereCylinder(lo.sphere(), hi.cylinder());
      case K::Cone:     return tangentSphereCone(lo.sphere(), hi.cone());
      default: break;
    }
  } else if (lk == K::Cylinder && hk == K::Cone) {
    return tangentCylinderCone(lo.cylinder(), hi.cone());
  }
  return TangentContact::transversal();
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_DISPATCH_H

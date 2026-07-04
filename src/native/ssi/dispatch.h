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
#include "native/ssi/curve.h"
#include "native/ssi/plane_conics.h"
#include "native/ssi/plane_torus.h"
#include "native/ssi/quadric_pairs.h"

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

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_DISPATCH_H

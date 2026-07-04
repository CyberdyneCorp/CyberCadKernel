// SPDX-License-Identifier: Apache-2.0
//
// tangent_contact.h — the SSI Stage-S4-b TYPED tangent-contact result.
//
// Where two native surfaces meet with their normals PARALLEL (‖n_A × n_B‖ ≈ 0), the
// intersection is DEGENERATE: it is not a clean transversal crossing. S2/S3 previously
// lumped every such situation into a single blunt `deferredTangent` counter — an honest
// "seen but not safely handled" gap, but structureless. S4-b replaces that with a TYPED
// classification of WHAT the degeneracy is, so downstream (booleans / the S4-c marcher /
// the OCCT fallback) can act on the SHAPE of the contact instead of a bare flag.
//
// A `TangentContact` is exactly one of:
//   * TransversalOnly        — the surfaces are NOT tangent here; the normals cross at a
//                              well-conditioned angle. (A classifier called on a clearly
//                              transversal solution reports this — there is no degeneracy.)
//   * TangentPoint           — an ISOLATED 0-dimensional contact: the surfaces touch at a
//                              single point and separate everywhere around it (e.g. two
//                              spheres at centre distance d = R₁+R₂, or a plane tangent to
//                              a sphere). `point` carries the touch point. The relative
//                              second fundamental form there is SIGN-DEFINITE.
//   * TangentCurve           — the surfaces are tangent ALONG A WHOLE CURVE (a 1-dimensional
//                              contact locus): e.g. a cylinder tangent to a plane along a
//                              ruling line, or a sphere tangent to a coaxial cylinder along
//                              its equator circle. `curve` carries that contact curve (a
//                              native conic) when determinable; `point` a point on it. The
//                              relative second fundamental form is RANK-1 (one zero
//                              principal direction — the tangent-curve direction).
//   * NearTangentTransversal — the surfaces GRAZE but still CROSS (the contact is a
//                              transversal branch whose crossing angle is tiny). The
//                              relative second fundamental form is INDEFINITE. This is the
//                              hard case S4-c owns (marching THROUGH the near-tangency);
//                              S4-b classifies it and HANDS IT ON — it never traces it.
//                              `crossingSine` = ‖n_A × n_B‖ witnesses how shallow the
//                              crossing is.
//   * Undecided              — the contact is degenerate but the local jet is within the
//                              model-scale curvature-noise band, so the point/curve/cross
//                              discrimination is not robust. The honest "→ OCCT" outcome:
//                              the native layer does NOT guess; the ENGINE owns the OCCT
//                              fallback + self-verify. A correct `Undecided` is first-class.
//
// SCOPE (S4-b — DETECTION + CLASSIFICATION only). This header carries the RESULT; the
// closed-form analytic classifier lives in dispatch.h (`classify_tangency`) and the seeded
// differential-geometry classifier lives in seeding.cpp (under CYBERCAD_HAS_NUMSCI). This
// layer NEVER marches through a tangency and NEVER fabricates a curve across a degeneracy —
// `NearTangentTransversal` is a classified hand-off to S4-c, not a traced curve.
//
// Built on the EXISTING seams: a `TangentPoint`'s point is exactly a `CurveKind::Point`
// degenerate branch; a `TangentCurve`'s curve is an `IntersectionCurve` (Line/Circle/…).
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE. Uses src/native/math + curve.h only.
// clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_TANGENT_CONTACT_H
#define CYBERCAD_NATIVE_SSI_TANGENT_CONTACT_H

#include "native/ssi/curve.h"  // IntersectionCurve / CurveKind::Point / Point3

#include <optional>

namespace cybercad::native::ssi {

/// The five mutually-exclusive tangent-contact outcomes (see the file header).
enum class TangentContactType {
  TransversalOnly,          ///< not tangent — normals cross at a well-conditioned angle
  TangentPoint,             ///< isolated 0-dim contact (definite relative 2nd form) → `point`
  TangentCurve,             ///< tangent along a curve (rank-1 relative 2nd form) → `curve`
  NearTangentTransversal,   ///< grazes but crosses (indefinite) → S4-c gap, handed on
  Undecided,                ///< degenerate but within curvature-noise band → OCCT
};

/// The typed contact descriptor. `point` is filled for `TangentPoint` (the touch point)
/// and, when known, for `TangentCurve` (a representative point on the contact curve).
/// `curve` is filled for `TangentCurve` when the contact locus is determinable as a native
/// conic. `crossingSine` = ‖n_A × n_B‖ at the contact (0 for exact tangency; small and
/// non-zero for `NearTangentTransversal`; above the tangent threshold for `TransversalOnly`).
/// Pure data — OCCT-free, substrate-free.
struct TangentContact {
  TangentContactType type = TangentContactType::TransversalOnly;
  Point3 point{};                          ///< TangentPoint: touch point; TangentCurve: a point on it
  std::optional<IntersectionCurve> curve{}; ///< TangentCurve: the contact conic, when determinable
  double crossingSine = 0.0;               ///< ‖n_A × n_B‖ at the contact (transversality witness)

  static TangentContact transversal(double sine = 1.0) {
    TangentContact t;
    t.type = TangentContactType::TransversalOnly;
    t.crossingSine = sine;
    return t;
  }
  static TangentContact tangentPoint(const Point3& p, double sine = 0.0) {
    TangentContact t;
    t.type = TangentContactType::TangentPoint;
    t.point = p;
    t.crossingSine = sine;
    return t;
  }
  static TangentContact tangentCurve(const IntersectionCurve& c, double sine = 0.0) {
    TangentContact t;
    t.type = TangentContactType::TangentCurve;
    t.curve = c;
    t.point = c.value(c.naturalRange().first);  // a representative point on the contact curve
    t.crossingSine = sine;
    return t;
  }
  /// A tangent curve whose conic we could not pin down, but whose tangency is confirmed
  /// (rank-1 relative second form on the seeded path). Carries a witness point only.
  static TangentContact tangentCurveAt(const Point3& p, double sine = 0.0) {
    TangentContact t;
    t.type = TangentContactType::TangentCurve;
    t.point = p;
    t.crossingSine = sine;
    return t;
  }
  static TangentContact nearTangentTransversal(const Point3& p, double sine) {
    TangentContact t;
    t.type = TangentContactType::NearTangentTransversal;
    t.point = p;
    t.crossingSine = sine;
    return t;
  }
  static TangentContact undecided(const Point3& p, double sine) {
    TangentContact t;
    t.type = TangentContactType::Undecided;
    t.point = p;
    t.crossingSine = sine;
    return t;
  }

  /// True for the two decided TANGENT kinds (isolated point / tangent curve). A
  /// `NearTangentTransversal` is NOT tangent (it crosses); `Undecided` is not decided.
  bool isTangent() const noexcept {
    return type == TangentContactType::TangentPoint ||
           type == TangentContactType::TangentCurve;
  }
  /// True for the honest "hand-on to S4-c / OCCT" outcomes (not resolved by S4-b).
  bool isDeferred() const noexcept {
    return type == TangentContactType::NearTangentTransversal ||
           type == TangentContactType::Undecided;
  }
};

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_TANGENT_CONTACT_H

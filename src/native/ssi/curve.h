// SPDX-License-Identifier: Apache-2.0
//
// curve.h — the native SSI result curve types (Stage S1, analytic SSI).
//
// An IntersectionCurve is one branch of a surface–surface intersection expressed
// in CLOSED FORM as a native conic: a Line, Circle, Ellipse, Parabola or
// Hyperbola placed by a right-handed frame (math::Ax3). Every analytic S1 handler
// returns curves of these kinds only — the exact family OCCT's analytic quadric
// intersector IntAna_QuadQuadGeo yields (its IntAna_ResultType is
// Point/Line/Circle/Ellipse/Parabola/Hyperbola/Empty/Same/NoGeometricSolution).
// We cross-check curve KIND and geometry against that oracle on the simulator; the
// native code here is OCCT-free.
//
// PARAMETRIZATION (matches gp_Lin / gp_Circ / gp_Elips / gp_Parab / gp_Hypr so the
// oracle's Value(t) agrees):
//   Line       P(t) = O + t·X                          t ∈ ℝ
//   Circle     P(t) = O + R(cos t·X + sin t·Y)         t ∈ [0,2π)
//   Ellipse    P(t) = O + a·cos t·X + b·sin t·Y        t ∈ [0,2π),  a=major, b=minor
//   Parabola   P(t) = O + (t²/(4f))·X + t·Y            focal length f, apex O, axis X
//   Hyperbola  P(t) = O + a·cosh t·X + b·sinh t·Y      t ∈ ℝ
// The frame's Z is the plane normal of the (planar) conic; X is the conic's own
// major/reference axis, Y = Z × X, right-handed. For a Line, X is the direction and
// Y/Z complete a frame (unused by Value).
//
// The CORRECTNESS INVARIANT every handler must satisfy: for all t on the returned
// curve, P(t) lies on BOTH input surfaces within tolerance. This header ships the
// evaluator (`value`) that the host verification gate samples to check exactly that.
//
// Header-only, OCCT-FREE. clang++ -std=c++20. Uses src/native/math only.
//
#ifndef CYBERCAD_NATIVE_SSI_CURVE_H
#define CYBERCAD_NATIVE_SSI_CURVE_H

#include "native/math/elementary.h"  // Ax3, frameCombine
#include "native/math/vec.h"

#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::ssi {

namespace math = cybercad::native::math;

using math::Ax3;
using math::Dir3;
using math::Point3;
using math::Vec3;

inline constexpr double kSsiPi = 3.14159265358979323846;

/// A unit vector orthogonal to `d` (any one — used to complete a frame when only the
/// axis is meaningful, e.g. a Line's carrier plane is arbitrary). Picks the world
/// axis least aligned with `d` to avoid a near-null cross product.
inline Dir3 orthogonalTo(const Dir3& d) noexcept {
  const double ax = std::fabs(d.x()), ay = std::fabs(d.y()), az = std::fabs(d.z());
  Vec3 pick = (ax <= ay && ax <= az) ? Vec3{1, 0, 0} : (ay <= az) ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
  return Dir3{math::cross(d.vec(), pick)};
}

/// The closed-form kind of one analytic intersection branch. Mirrors the geometric
/// arms of OCCT's IntAna_ResultType (a Point is degenerate tangency; Line/Circle/
/// Ellipse/Parabola/Hyperbola are the proper conic curves).
enum class CurveKind {
  Point,      ///< a single tangency point (degenerate branch)
  Line,       ///< straight line: P(t) = O + t·X
  Circle,     ///< circle radius R in the frame X–Y plane
  Ellipse,    ///< ellipse semi-axes (a=X, b=Y)
  Parabola,   ///< parabola focal length f, apex at O, opening along X
  Hyperbola,  ///< one branch: P(t) = O + a·cosh t·X + b·sinh t·Y
};

/// One analytic intersection curve, placed by `frame` and sized by the fields the
/// kind uses. `frame.origin` is the conic centre (Circle/Ellipse/Hyperbola) or the
/// line/parabola reference point; `frame.x` the reference/major axis; `frame.z` the
/// carrier-plane normal (for the planar conics). Native only — no OCCT.
struct IntersectionCurve {
  CurveKind kind = CurveKind::Line;
  Ax3 frame{};

  // Sizes — meaning depends on `kind` (documented in the parametrization block above).
  double radius = 0.0;  ///< Circle: R
  double a = 0.0;       ///< Ellipse/Hyperbola: semi-major (along X)
  double b = 0.0;       ///< Ellipse/Hyperbola: semi-minor (along Y)
  double focal = 0.0;   ///< Parabola: focal length f

  /// Hyperbola branch selector: +1 = the +X branch (cosh t ≥ 1 → x ≥ +a), −1 = the
  /// mirror −X branch (x ≤ −a). A plane cutting a double-napped cone below the
  /// half-angle meets BOTH nappes, so such an intersection is TWO Hyperbola curves
  /// sharing a frame — one per branch. Ignored by every non-Hyperbola kind (which are
  /// each a single connected curve). Default +1.
  double branch = 1.0;

  /// A single point for CurveKind::Point (also frame.origin, kept explicit).
  Point3 point{};

  /// Evaluate the curve at parameter `t` (see the parametrization block).
  Point3 value(double t) const noexcept {
    switch (kind) {
      case CurveKind::Point:
        return point;
      case CurveKind::Line:
        return frame.origin + frame.x.vec() * t;
      case CurveKind::Circle:
        return frame.origin + frame.x.vec() * (radius * std::cos(t)) +
               frame.y.vec() * (radius * std::sin(t));
      case CurveKind::Ellipse:
        return frame.origin + frame.x.vec() * (a * std::cos(t)) +
               frame.y.vec() * (b * std::sin(t));
      case CurveKind::Parabola:
        return frame.origin + frame.x.vec() * (t * t / (4.0 * focal)) + frame.y.vec() * t;
      case CurveKind::Hyperbola:
        // branch = ±1 selects the +X (x ≥ +a) or mirror −X (x ≤ −a) branch.
        return frame.origin + frame.x.vec() * (branch * a * std::cosh(t)) +
               frame.y.vec() * (b * std::sinh(t));
    }
    return frame.origin;
  }

  /// A natural parameter sampling span for verification / display. Closed conics
  /// (Circle/Ellipse) sweep [0,2π); the open conics (Line/Parabola/Hyperbola) use a
  /// symmetric bounded span the caller can widen.
  std::pair<double, double> naturalRange() const noexcept {
    switch (kind) {
      case CurveKind::Circle:
      case CurveKind::Ellipse:
        return {0.0, 2.0 * kSsiPi};
      case CurveKind::Point:
        return {0.0, 0.0};
      default:  // Line / Parabola / Hyperbola: unbounded → a symmetric window
        return {-10.0, 10.0};
    }
  }
};

/// Status of an intersection query. Mirrors the oracle's Empty / Same /
/// NoGeometricSolution outcomes plus the S1 "not a closed-form pair" verdict.
enum class IntersectionStatus {
  Ok,             ///< 1..N analytic curves produced (see `curves`)
  NoIntersection, ///< surfaces provably do not meet (IntAna_Empty)
  Coincident,     ///< surfaces are the same locus (IntAna_Same) — no isolated curve
  NotAnalytic,    ///< outside the S1 closed-form family → defer to S2/S3 marching / OCCT
};

/// The multi-branch result of intersect_surfaces: 0..N analytic curves plus a status.
/// A well-formed S1 result is either Ok with ≥1 curve, or one of the non-Ok statuses
/// with an empty curve list. NotAnalytic is the honest "not in scope" signal — the
/// caller must fall back; S1 NEVER fakes a curve for an unsupported pair.
struct IntersectionResult {
  IntersectionStatus status = IntersectionStatus::NotAnalytic;
  std::vector<IntersectionCurve> curves{};

  static IntersectionResult ok(std::vector<IntersectionCurve> cs) {
    return {IntersectionStatus::Ok, std::move(cs)};
  }
  static IntersectionResult ok(IntersectionCurve c) {
    return {IntersectionStatus::Ok, {std::move(c)}};
  }
  static IntersectionResult empty() { return {IntersectionStatus::NoIntersection, {}}; }
  static IntersectionResult coincident() { return {IntersectionStatus::Coincident, {}}; }
  static IntersectionResult notAnalytic() { return {IntersectionStatus::NotAnalytic, {}}; }

  bool ok_() const noexcept { return status == IntersectionStatus::Ok; }
};

// ── small conic-frame constructors ────────────────────────────────────────────

/// A Line curve through `o` with unit direction `d`.
inline IntersectionCurve makeLine(const Point3& o, const Dir3& d) {
  IntersectionCurve c;
  c.kind = CurveKind::Line;
  // A line's carrier plane is arbitrary; we only need frame.x == direction and any
  // orthonormal Y,Z to complete a right-handed frame (Value uses only frame.x).
  const Dir3 x = d;
  const Dir3 z = orthogonalTo(x);
  const Dir3 y{math::cross(z.vec(), x.vec())};
  c.frame = Ax3{o, x, y, z};
  return c;
}

/// A Circle of radius `r` centred at `o`, lying in the plane with normal `n`; `xref`
/// gives the start (t=0) direction (projected into the plane).
inline IntersectionCurve makeCircle(const Point3& o, double r, const Dir3& n, const Dir3& xref) {
  IntersectionCurve c;
  c.kind = CurveKind::Circle;
  c.frame = Ax3::fromAxisAndRef(o, n, xref);
  c.radius = r;
  return c;
}

/// An Ellipse centred at `o` in plane normal `n`, semi-major `a` along `xmajor`,
/// semi-minor `b` along n×xmajor.
inline IntersectionCurve makeEllipse(const Point3& o, double a, double b, const Dir3& n,
                                     const Dir3& xmajor) {
  IntersectionCurve c;
  c.kind = CurveKind::Ellipse;
  c.frame = Ax3::fromAxisAndRef(o, n, xmajor);
  c.a = a;
  c.b = b;
  return c;
}

/// A single degenerate tangency point.
inline IntersectionCurve makePoint(const Point3& p) {
  IntersectionCurve c;
  c.kind = CurveKind::Point;
  c.point = p;
  c.frame.origin = p;
  return c;
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_CURVE_H

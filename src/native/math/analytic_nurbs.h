// SPDX-License-Identifier: Apache-2.0
//
// analytic_nurbs.h — NURBS roadmap Layer 1: EXACT bidirectional conversion between
// analytic primitives and rational NURBS (Piegl & Tiller, "The NURBS Book" Ch.7).
//
// TWO directions, both EXACT (not approximate):
//
//   ANALYTIC → NURBS (construction).  Every conic and quadric has an EXACT rational
//   (weighted) B-spline form. This module builds it directly:
//     circleToNurbs / arcToNurbs   — the standard rational-quadratic representation.
//       An arc of sweep ≤ 90° is a SINGLE rational Bézier segment: control polygon
//       {P0, P1, P2} where P1 is the intersection of the end tangents and the middle
//       weight is w1 = cos(halfSweep) (§7.3, Eq. 7.30–7.32). A full circle is 4
//       quarter-circle segments joined into one piecewise rational-quadratic curve
//       (9 poles, knot vector {0,0,0,¼,¼,½,½,¾,¾,1,1,1}, alternating weights
//       1, √2/2, 1, √2/2, …). This is EXACT: the rational quadratic traces the true
//       circle, it does not sample it.
//     ellipseToNurbs               — the circle net scaled by (a,b) in the plane
//       (an affine image of a circle is an exact rational ellipse; same weights).
//     lineToNurbs                  — a degree-1 two-pole segment (trivially exact).
//     planeToNurbs                 — a degree-(1,1) 2×2 bilinear patch over a finite
//       (u,v) window (an unbounded plane has no NURBS form; a window is exact).
//     cylinderToNurbs / coneToNurbs / sphereToNurbs / torusToNurbs — surfaces of
//       revolution: the profile (a line, or a circle/arc) revolved about the axis by
//       the same 4-segment rational-quadratic circle in the U (revolution) direction
//       (§7.5, revolved-surface construction). Exact quadric/torus surfaces.
//
//   NURBS → ANALYTIC (recognition).  The HARD direction: given a rational NURBS, is it
//   EXACTLY one of these analytic forms, and if so what are its parameters?
//     recognizeCurve(curve)     → Line / Circle / Arc / Ellipse / General
//     recognizeSurface(surface) → Plane / Cylinder / Cone / Sphere / General
//   Strategy: the H3 primitive-fit machinery (primitive_fit.h — fitPlane / fitSphere /
//   fitCylinder / fitCone, and a curve-side circle/line fit here) runs on sampled
//   points as a CANDIDATE GENERATOR. Then — and this is the honesty bar — we VERIFY
//   EXACTNESS ALGEBRAICALLY: every CONTROL POINT of the NURBS (de-homogenized) must
//   satisfy the candidate primitive's implicit equation to ≤ 1e-12, AND the weights
//   must match the exact rational pattern. A fit with a small RMS but control points
//   that do NOT satisfy the equation is rejected as "General" — we never force a
//   primitive. (A rational curve lies in the convex hull of its poles, so if all poles
//   satisfy a quadric's implicit equation AND the curve is a genuine conic, the curve
//   lies on that quadric; the control-net test is the airtight algebraic certificate.)
//
// The whole module goes through primitive_fit (which calls the numsci facade), so it
// is compiled only under CYBERCAD_HAS_NUMSCI, exactly like primitive_fit / bspline_fit.
// The analytic→NURBS constructors are pure geometry and would build without the guard,
// but recognition needs the fitter, so the TU is guarded as a unit; declarations stay
// visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic. Homogeneous lift throughout.
//
#ifndef CYBERCAD_NATIVE_MATH_ANALYTIC_NURBS_H
#define CYBERCAD_NATIVE_MATH_ANALYTIC_NURBS_H

#include "bspline_ops.h"   // BsplineCurveData / BsplineSurfaceData
#include "elementary.h"    // Ax3, Plane, Cylinder, Cone, Sphere
#include "torus.h"         // Torus
#include "vec.h"

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Analytic curve descriptors (for the constructors + recognition results).
// ─────────────────────────────────────────────────────────────────────────────

/// A circle: center, unit normal of its plane, an in-plane unit X axis, radius.
/// Points are C(t)=center + r(cos t·X + sin t·Y), Y = normal × X.
struct Circle {
  Point3 center{};
  Dir3 normal{0, 0, 1};
  Dir3 xAxis{1, 0, 0};
  double radius = 1.0;
};

/// A circular arc: a circle plus a start angle and sweep (radians, sweep in (0,2π]).
struct Arc {
  Circle circle{};
  double startAngle = 0.0;  ///< radians, measured from xAxis
  double sweepAngle = 0.0;  ///< radians, > 0 (CCW about normal)
};

/// An ellipse in its plane: center, unit normal, major (X) axis, and the two
/// semi-axes. Points are E(t)=center + a·cos t·X + b·sin t·Y, Y = normal × X.
struct Ellipse {
  Point3 center{};
  Dir3 normal{0, 0, 1};
  Dir3 xAxis{1, 0, 0};   ///< direction of the semi-major axis a
  double majorRadius = 1.0;  ///< a
  double minorRadius = 1.0;  ///< b
};

/// A line SEGMENT: two endpoints (the NURBS form is finite).
struct LineSegment {
  Point3 start{};
  Point3 end{};
};

// ─────────────────────────────────────────────────────────────────────────────
// Analytic → NURBS (EXACT rational construction).
// ─────────────────────────────────────────────────────────────────────────────

/// Full circle as a piecewise rational-quadratic B-spline (4 quarter segments,
/// 9 poles). Exact: the curve IS the circle.
BsplineCurveData circleToNurbs(const Circle& c);

/// A circular arc (sweep in (0, 2π]) as a piecewise rational-quadratic B-spline.
/// Split into ≤ 90° segments; each is one rational Bézier with weight cos(half).
BsplineCurveData arcToNurbs(const Arc& a);

/// A full ellipse as a rational-quadratic B-spline (affine image of the circle net).
BsplineCurveData ellipseToNurbs(const Ellipse& e);

/// A line segment as a degree-1 two-pole B-spline (weights 1, exact).
BsplineCurveData lineToNurbs(const LineSegment& l);

/// A finite window [u0,u1]×[v0,v1] of a plane as a bilinear (degree 1,1) 2×2 patch.
BsplineSurfaceData planeToNurbs(const Plane& p, double u0, double u1, double v0,
                                double v1);

/// A finite-height cylinder [v0,v1] as an exact rational surface of revolution
/// (rational-quadratic in U, linear in V).
BsplineSurfaceData cylinderToNurbs(const Cylinder& c, double v0, double v1);

/// A finite-height cone [v0,v1] as an exact rational surface of revolution.
BsplineSurfaceData coneToNurbs(const Cone& c, double v0, double v1);

/// A full sphere as an exact rational surface of revolution (a meridian half-circle
/// revolved 360°). Rational-quadratic in both U and V.
BsplineSurfaceData sphereToNurbs(const Sphere& s);

/// A full torus as an exact rational surface of revolution (the tube circle revolved
/// 360°). Rational-quadratic in both U and V.
BsplineSurfaceData torusToNurbs(const Torus& t);

// ─────────────────────────────────────────────────────────────────────────────
// NURBS → analytic (recognition). Recover parameters EXACTLY or report General.
// ─────────────────────────────────────────────────────────────────────────────

enum class CurveKind { General, Line, Circle, Arc, Ellipse };
enum class SurfaceKind { General, Plane, Cylinder, Cone, Sphere };

/// Recognition of a rational NURBS CURVE. Only the field matching `kind` is
/// meaningful. `residual` is the max control-point algebraic residual actually
/// verified (0 within tol ⇒ exact; a General result reports the best residual seen).
struct CurveRecognition {
  CurveKind kind = CurveKind::General;
  LineSegment line{};
  Circle circle{};
  Arc arc{};
  Ellipse ellipse{};
  double residual = 0.0;
};

/// Recognition of a rational NURBS SURFACE. Only the field matching `kind` is
/// meaningful. `residual` is the max control-point algebraic residual verified.
struct SurfaceRecognition {
  SurfaceKind kind = SurfaceKind::General;
  Plane plane{};
  Cylinder cylinder{};
  Cone cone{};
  Sphere sphere{};
  double residual = 0.0;
};

/// Recognize whether `curve` is EXACTLY a line / circle / arc / ellipse. The fit is
/// a candidate generator; acceptance requires the control net to satisfy the
/// primitive's implicit equation to ≤ `tol` (default 1e-12). Otherwise → General.
CurveRecognition recognizeCurve(const BsplineCurveData& curve, double tol = 1e-12);

/// Recognize whether `surface` is EXACTLY a plane / cylinder / cone / sphere, with
/// the same control-net algebraic-exactness certificate. Otherwise → General.
SurfaceRecognition recognizeSurface(const BsplineSurfaceData& surface,
                                    double tol = 1e-12);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_ANALYTIC_NURBS_H

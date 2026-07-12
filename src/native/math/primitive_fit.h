// SPDX-License-Identifier: Apache-2.0
//
// primitive_fit.h — NURBS roadmap Layer 7 (reverse-engineering): analytic-primitive
// DETECTION and exact FITTING (points → plane / sphere / cylinder / cone).
//
// Layer 7 (bspline_fit.{h,cpp}) fits a free-form B-spline to sampled points. Scan-
// to-CAD, however, must FIRST ask whether the samples actually lie on an ANALYTIC
// primitive — a scanned cylinder should become an EXACT rational cylinder, not a
// wobbly B-spline. This module answers that question:
//
//   fitPlane     — total-least-squares plane. The best-fit normal is the smallest-
//                  eigenvector of the centered covariance C = Σ(pᵢ-p̄)(pᵢ-p̄)ᵀ. We
//                  diagonalize the symmetric 3×3 C with a classical cyclic Jacobi
//                  sweep (closed-form 2×2 rotations, no external SVD, no facade
//                  widening) — the airtight symmetric-eigen path this roadmap asks
//                  for. Reports normal, signed offset d (n·x = d), and RMS.
//   fitSphere    — algebraic fit of x²+y²+z²+Dx+Ey+Fz+G=0 (a LINEAR system in
//                  D,E,F,G solved through numerics::lstsq), then an optional
//                  geometric (least-squares distance) refine. Reports center,
//                  radius, RMS.
//   fitCylinder  — axis direction from the Gauss map: surface normals of a cylinder
//                  are all ⟂ to the axis, so the axis is the SMALLEST-eigenvector of
//                  the normal covariance (same Jacobi path). With the axis fixed the
//                  points project to a circle whose algebraic 2-D fit gives the axis
//                  point + radius. Reports axis point, direction, radius, RMS.
//   fitCone      — apex + axis + half-angle. The apex/axis seed from the cylinder-
//                  style normal analysis (cone normals lie on a cone about the axis),
//                  then a geometric refine over (apex, axis, half-angle) minimizes the
//                  point-to-cone distance through numerics::least_squares.
//   detectPrimitive — try all four, return the best type whose RMS is within a
//                  RELATIVE tolerance; otherwise report Freeform (fall back to the
//                  B-spline territory of bspline_fit — this module does NOT fit a
//                  free-form here, it only declines honestly).
//
// HONESTY — no tolerance is ever widened to force a fit. Each result carries the
// TRUE achieved RMS. On noisy input the parameters land within the noise band and
// the RMS is reported as-is (never clamped to claim exactness). detectPrimitive
// returns Freeform rather than a spurious primitive when nothing fits.
//
// RATIONAL LIFT — plane/sphere/cylinder/cone are all exact rational NURBS. This
// header reports the ANALYTIC parameters (center/axis/radius/half-angle); handing
// those to the existing rational constructors (bspline_ops / torus / elementary)
// yields the exact rational surface. That construction is out of scope here.
//
// GUARD — the fitting routines call the numsci facade (numerics::lstsq /
// least_squares), so the whole module is compiled only under CYBERCAD_HAS_NUMSCI,
// exactly like bspline_fit. Declarations stay visible for documentation; with the
// guard OFF the implementation TU is inert and the functions are absent. The 3×3
// Jacobi eigensolver is self-contained (no substrate), living in the .cpp.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_PRIMITIVE_FIT_H
#define CYBERCAD_NATIVE_MATH_PRIMITIVE_FIT_H

#include "vec.h"

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Result types. Every fit carries the ACHIEVED RMS (root-mean-square of the signed
// point-to-primitive distances) and an `ok` flag — false when the input is too
// small / degenerate to define the primitive (honest decline, never a crash).
// ─────────────────────────────────────────────────────────────────────────────

/// Best-fit plane  n·x = offset,  with |n| = 1.
struct PlaneFit {
  Dir3 normal{};        ///< unit plane normal
  double offset = 0.0;  ///< signed distance of the plane from the origin (n·x = offset)
  Point3 centroid{};    ///< centroid of the input (a point guaranteed on the plane)
  double rms = 0.0;     ///< RMS point-to-plane distance
  bool ok = false;
};

/// Best-fit sphere.
struct SphereFit {
  Point3 center{};
  double radius = 0.0;
  double rms = 0.0;     ///< RMS |‖p-center‖ - radius|
  bool ok = false;
};

/// Best-fit (infinite) cylinder: an axis line (point + unit direction) and radius.
struct CylinderFit {
  Point3 axisPoint{};   ///< a point on the axis (foot of the centroid)
  Dir3 axis{};          ///< unit axis direction
  double radius = 0.0;
  double rms = 0.0;     ///< RMS |dist(p, axis) - radius|
  bool ok = false;
};

/// Best-fit (infinite, single-nappe) cone: apex, unit axis (apex → opening), and
/// half-angle in radians (angle between axis and the cone surface, 0 < α < π/2).
struct ConeFit {
  Point3 apex{};
  Dir3 axis{};
  double halfAngle = 0.0;  ///< radians
  double rms = 0.0;        ///< RMS point-to-cone-surface distance
  bool ok = false;
};

/// Which analytic primitive detectPrimitive settled on.
enum class PrimitiveType { Freeform, Plane, Sphere, Cylinder, Cone };

/// Outcome of detectPrimitive — the winning type plus whichever fit(s) were
/// computed. Only the field matching `type` is meaningful; the rest are defaulted.
/// `relError` is the winning RMS divided by the point-cloud extent (scale-free),
/// so callers can reason about fit quality independent of model size.
struct PrimitiveDetection {
  PrimitiveType type = PrimitiveType::Freeform;
  PlaneFit plane{};
  SphereFit sphere{};
  CylinderFit cylinder{};
  ConeFit cone{};
  double rms = 0.0;       ///< achieved RMS of the winning fit (0 for Freeform)
  double relError = 0.0;  ///< rms / cloud-extent for the winning fit
  bool ok = false;        ///< true iff a primitive (non-Freeform) was accepted
};

// ─────────────────────────────────────────────────────────────────────────────
// Fitting entry points. Each needs enough points to define the primitive
// (plane ≥ 3, sphere ≥ 4, cylinder ≥ 6, cone ≥ 6); fewer → ok=false.
// ─────────────────────────────────────────────────────────────────────────────

/// Total-least-squares plane through the point cloud (smallest-eigenvector of the
/// centered covariance via a self-contained symmetric-3×3 Jacobi eigensolver).
PlaneFit fitPlane(std::span<const Point3> points);

/// Algebraic sphere fit (linearized x²+y²+z²+Dx+Ey+Fz+G=0 via numerics::lstsq),
/// with an optional geometric distance refine when `refine` is true.
SphereFit fitSphere(std::span<const Point3> points, bool refine = true);

/// Cylinder fit: Gauss-map axis (smallest-eigenvector of the discrete normal
/// covariance) + algebraic 2-D circle in the plane ⟂ to the axis.
CylinderFit fitCylinder(std::span<const Point3> points);

/// Cone fit: apex + axis + half-angle, seeded from the normal-cone analysis and
/// refined geometrically through numerics::least_squares.
ConeFit fitCone(std::span<const Point3> points);

/// Try all primitives and return the best one whose RELATIVE RMS is ≤ `relTol`
/// (default 1e-6 of the cloud extent). If none qualifies, return Freeform — this
/// function never fits a free-form B-spline; it only decides / declines.
PrimitiveDetection detectPrimitive(std::span<const Point3> points,
                                   double relTol = 1e-6);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_PRIMITIVE_FIT_H

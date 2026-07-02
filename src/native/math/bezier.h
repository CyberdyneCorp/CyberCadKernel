// SPDX-License-Identifier: Apache-2.0
//
// bezier.h — Bézier curve & surface evaluation (de Casteljau).
//
// Clean-room from *The NURBS Book* / classic CAGD:
//   de Casteljau       — Algorithm A1.5 (curve point by repeated linear interp)
//   curve derivatives   — hodograph: C'(t) = n·Σ (P_{i+1}−P_i)·B_{i,n-1}(t)
//   surface point       — tensor-product de Casteljau (A1.7)
// Rational Bézier reuses the homogeneous-then-project scheme (weighted control
// points), the degree-elevated special case of NURBS with a single Bernstein
// span, so results agree with the NURBS path.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BEZIER_H
#define CYBERCAD_NATIVE_MATH_BEZIER_H

#include "vec.h"

#include <span>

namespace cybercad::native::math {

/// Bézier curve point at t∈[0,1] via de Casteljau (A1.5). poles = n+1 points.
Point3 bezierPoint(std::span<const Point3> poles, double t) noexcept;

/// Rational Bézier curve point (weights parallel poles).
Point3 rationalBezierPoint(std::span<const Point3> poles,
                           std::span<const double> weights, double t) noexcept;

/// Bézier curve derivatives 0..maxDeriv. out[k] = k-th derivative. Length
/// maxDeriv+1. Derivatives above the degree are zero.
void bezierDerivs(std::span<const Point3> poles, double t, int maxDeriv,
                  std::span<Vec3> out);

/// Bézier surface point. Poles row-major: pole(i,j)=poles[i*nCols+j], i over U
/// (degree nRows-1), j over V (degree nCols-1). Tensor-product de Casteljau.
Point3 bezierSurfacePoint(std::span<const Point3> poles, int nRows, int nCols,
                          double u, double v) noexcept;

/// Rational Bézier surface point (weights parallel the pole grid).
Point3 rationalBezierSurfacePoint(std::span<const Point3> poles,
                                  std::span<const double> weights,
                                  int nRows, int nCols, double u, double v) noexcept;

/// Bézier surface first derivatives ∂/∂u, ∂/∂v and the unit normal.
struct BezierSurfaceD1 {
  Point3 point;
  Vec3 du;
  Vec3 dv;
  Dir3 normal;
};
BezierSurfaceD1 bezierSurfaceD1(std::span<const Point3> poles, int nRows, int nCols,
                                double u, double v) noexcept;

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BEZIER_H

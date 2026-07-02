// SPDX-License-Identifier: Apache-2.0
//
// bspline.h — B-spline and rational NURBS evaluation (curves & surfaces).
//
// Clean-room implementation from *The NURBS Book* (Piegl & Tiller, 2nd ed.):
//   FindSpan          — Algorithm A2.1
//   BasisFuns         — Algorithm A2.2
//   DersBasisFuns     — Algorithm A2.3 (basis-function derivatives)
//   CurvePoint        — Algorithm A3.1
//   CurveDerivsAlg1   — Algorithm A3.2
//   SurfacePoint      — Algorithm A3.5
//   SurfaceDerivsAlg1 — Algorithm A3.6
//   de Boor           — non-rational curve point via knot insertion (Boehm)
// Rational (NURBS) results follow §4 / §4.4: evaluate in homogeneous space
// (weighted control points wᵢ·Pᵢ, wᵢ) then project, and apply the quotient
// rule (A4.2 / A4.4) for derivatives.
//
// Knot convention: FLAT knot vector U (knots repeated by multiplicity), the
// same internal representation OCCT's BSplCLib uses. This lets us follow the
// book verbatim and compare numerically against BSplCLib / BSplSLib.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_H

#include "vec.h"

#include <cstddef>
#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Basis functions (low-level, reusable by curve & surface code).
// ─────────────────────────────────────────────────────────────────────────────

/// FindSpan (A2.1): index i of the knot span [U[i], U[i+1]) containing u,
/// with p ≤ i ≤ n. `n` is the index of the last control point (n = #poles − 1);
/// U is the flat knot vector of length n + p + 2. Clamps u to the valid domain.
int findSpan(int n, int degree, double u, std::span<const double> knots) noexcept;

/// BasisFuns (A2.2): non-zero basis functions N[0..p] at span `span`.
/// Output span must have length degree+1.
void basisFuns(int span, double u, int degree, std::span<const double> knots,
               std::span<double> out) noexcept;

/// DersBasisFuns (A2.3): basis functions and their derivatives up to order
/// `maxDeriv`. `out` is row-major (maxDeriv+1)×(degree+1): out[k*(p+1)+j] is the
/// k-th derivative of the j-th non-zero basis function.
void dersBasisFuns(int span, double u, int degree, int maxDeriv,
                   std::span<const double> knots, std::span<double> out);

// ─────────────────────────────────────────────────────────────────────────────
// Curves.
// ─────────────────────────────────────────────────────────────────────────────

/// Non-rational B-spline curve point (A3.1).
/// poles: n+1 control points; knots: flat, length n+p+2.
Point3 curvePoint(int degree, std::span<const Point3> poles,
                  std::span<const double> knots, double u) noexcept;

/// Non-rational B-spline curve point via de Boor's algorithm (Boehm knot
/// insertion). Equivalent to curvePoint; kept as an independent, corner-cutting
/// evaluator (numerically stable, no explicit basis array).
Point3 curvePointDeBoor(int degree, std::span<const Point3> poles,
                        std::span<const double> knots, double u) noexcept;

/// Non-rational curve derivatives 0..maxDeriv (A3.2). out[k] = k-th derivative.
/// out must have length maxDeriv+1.
void curveDerivs(int degree, std::span<const Point3> poles,
                 std::span<const double> knots, double u, int maxDeriv,
                 std::span<Vec3> out);

/// Rational NURBS curve point. `weights` parallels `poles` (wᵢ > 0).
Point3 nurbsCurvePoint(int degree, std::span<const Point3> poles,
                       std::span<const double> weights,
                       std::span<const double> knots, double u) noexcept;

/// Rational NURBS curve derivatives 0..maxDeriv via the quotient rule (A4.2).
/// out[0] is the point; out[k] the k-th derivative. Length maxDeriv+1.
void nurbsCurveDerivs(int degree, std::span<const Point3> poles,
                      std::span<const double> weights,
                      std::span<const double> knots, double u, int maxDeriv,
                      std::span<Vec3> out);

// ─────────────────────────────────────────────────────────────────────────────
// Surfaces. Poles are stored row-major: pole(i,j) = poles[i*nCols + j],
// i over the U direction (nRows = nU poles), j over V (nCols = nV poles).
// ─────────────────────────────────────────────────────────────────────────────

struct SurfaceGrid {
  std::span<const Point3> poles;    // nRows*nCols, row-major (U outer, V inner)
  int nRows = 0;                    // #poles in U
  int nCols = 0;                    // #poles in V
  Point3 pole(int i, int j) const noexcept { return poles[static_cast<std::size_t>(i) * nCols + j]; }
};

/// Non-rational surface point (A3.5).
Point3 surfacePoint(int degreeU, int degreeV, const SurfaceGrid& grid,
                    std::span<const double> knotsU, std::span<const double> knotsV,
                    double u, double v) noexcept;

/// Non-rational surface derivatives up to total order `maxDeriv` (A3.6).
/// out is row-major (maxDeriv+1)×(maxDeriv+1): out[k*(maxDeriv+1)+l] = ∂^(k+l)S/∂u^k∂v^l.
/// Entries with k+l > maxDeriv are set to zero.
void surfaceDerivs(int degreeU, int degreeV, const SurfaceGrid& grid,
                   std::span<const double> knotsU, std::span<const double> knotsV,
                   double u, double v, int maxDeriv, std::span<Vec3> out);

/// Rational NURBS surface point. `weights` parallels the pole grid.
Point3 nurbsSurfacePoint(int degreeU, int degreeV, const SurfaceGrid& grid,
                         std::span<const double> weights,
                         std::span<const double> knotsU, std::span<const double> knotsV,
                         double u, double v) noexcept;

/// Rational NURBS surface derivatives up to total order `maxDeriv` (quotient
/// rule A4.4). Layout identical to surfaceDerivs.
void nurbsSurfaceDerivs(int degreeU, int degreeV, const SurfaceGrid& grid,
                        std::span<const double> weights,
                        std::span<const double> knotsU, std::span<const double> knotsV,
                        double u, double v, int maxDeriv, std::span<Vec3> out);

/// Unit surface normal = normalize(∂S/∂u × ∂S/∂v). Works for both B-spline and
/// rational surfaces (pass weights, or empty span for non-rational).
Dir3 surfaceNormal(int degreeU, int degreeV, const SurfaceGrid& grid,
                   std::span<const double> weights,
                   std::span<const double> knotsU, std::span<const double> knotsV,
                   double u, double v) noexcept;

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_H

// SPDX-License-Identifier: Apache-2.0
//
// bspline_fit.h — NURBS roadmap Layer 7: fitting / approximation (points → NURBS).
//
// Given a sampled sequence (curve) or grid (surface) of points, construct a
// B-spline that either INTERPOLATES them (passes through every point exactly) or
// APPROXIMATES them (least-squares, fewer control points, endpoints pinned). This
// is the scan-to-CAD / data-reduction direction (docs/NURBS-SCOPE.md §2/§4 Layer 7).
//
// It sits above the evaluators in bspline.{h,cpp} and reuses the Layer-1 data
// types BsplineCurveData / BsplineSurfaceData from bspline_ops.h (flat knots,
// row-major U-outer surface poles) as its fit OUTPUT — so a fitted curve/surface
// drops straight into the rest of the NURBS stack. The linear systems are solved
// through the numsci facade (numerics::lin_solve for the square interpolation
// collocation, numerics::lstsq for the least-squares normal-equation-free solve).
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 9:
//   chordLengthParams / centripetalParams — Eq 9.4 / 9.5 parameter assignment
//   averagingKnots                        — Eq 9.8 knot averaging for interpolation
//   approxKnots                           — Eq 9.68/9.69 knot placement for approximation
//   interpolateCurve                      — A9.1 global curve interpolation
//   approximateCurve                      — A9.4/9.6 least-squares curve approximation
//   interpolateSurface / approximateSurface — tensor-product (A9.4-class): fit each
//                                          row then each column via the curve routines
//
// SCOPE — NON-RATIONAL fitting only (all weights = 1). Rational / weighted fitting
// (fitting the weights, e.g. Ma–Kruth) is a documented residual for a later slice;
// this module never fakes it. See docs/NURBS-SCOPE.md Layer-7 row.
//
// GUARD — the solve-bearing routines are compiled only when CYBERCAD_HAS_NUMSCI is
// defined (the numsci facade is the sole linear-algebra dependency), exactly like
// src/native/ssi/marching.cpp. The declarations remain visible for documentation;
// with the guard OFF the implementation TU is inert and the functions are absent.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_FIT_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_FIT_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)
#include "vec.h"

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Parametrization — assign a parameter uₖ ∈ [0,1] to each input point Qₖ.
// ─────────────────────────────────────────────────────────────────────────────

/// Parameter-assignment method for a point sequence (*The NURBS Book* §9.2.1).
enum class ParamMethod {
  Uniform,      ///< uₖ = k/(n) (Eq 9.3) — simplest, ignores geometry.
  ChordLength,  ///< uₖ ∝ chord length ‖Qₖ − Qₖ₋₁‖ (Eq 9.4/9.5) — the common default.
  Centripetal,  ///< uₖ ∝ √‖Qₖ − Qₖ₋₁‖ (Eq 9.6, Lee) — tamer near sharp turns.
};

/// Assign parameters uₖ ∈ [0,1] to `points` by the chosen method. The result is
/// monotone non-decreasing with u₀ = 0 and u_{last} = 1. Coincident/duplicate
/// points contribute zero chord length (honestly handled, never divide-by-zero):
/// a run of identical points shares the same parameter. Returns empty if fewer
/// than two points, or if every point is coincident (total chord length is zero —
/// there is no length to normalize against; the caller must guard).
std::vector<double> assignParams(std::span<const Point3> points, ParamMethod method);

/// Convenience wrappers.
inline std::vector<double> chordLengthParams(std::span<const Point3> pts) {
  return assignParams(pts, ParamMethod::ChordLength);
}
inline std::vector<double> centripetalParams(std::span<const Point3> pts) {
  return assignParams(pts, ParamMethod::Centripetal);
}

// ─────────────────────────────────────────────────────────────────────────────
// Knot-vector generation.
// ─────────────────────────────────────────────────────────────────────────────

/// Averaging knots for global INTERPOLATION of n+1 points of degree p (Eq 9.8):
/// a clamped knot vector whose interior knots are running averages of p
/// consecutive parameters. Guarantees the collocation matrix is totally positive
/// and banded (well-conditioned). Length = (params.size()) + degree + 1.
std::vector<double> averagingKnots(std::span<const double> params, int degree);

/// Knots for least-squares APPROXIMATION with `nCtrl` control points of degree p
/// over n+1 data parameters (Eq 9.68/9.69): a clamped vector that spreads the
/// interior knots across the data so every span sees data. Length = nCtrl+degree+1.
std::vector<double> approxKnots(std::span<const double> params, int degree, int nCtrl);

// ─────────────────────────────────────────────────────────────────────────────
// Curve fitting.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a curve fit (interpolation or approximation).
struct CurveFitResult {
  bool ok = false;         ///< true ⇔ the fit succeeded (solvable, non-degenerate input)
  BsplineCurveData curve;  ///< the fitted non-rational B-spline (empty weights)
  double maxError = 0.0;   ///< max ‖C(uₖ) − Qₖ‖ over the data (≈0 for interpolation)
  double rmsError = 0.0;   ///< root-mean-square of ‖C(uₖ) − Qₖ‖ over the data
};

/// A9.1 — global curve INTERPOLATION: build a degree-`degree` non-rational
/// B-spline through EVERY input point. Uses `method` for the parameters and
/// averaging knots (Eq 9.8); assembles the (n+1)×(n+1) basis collocation matrix
/// and solves it once per coordinate with numerics::lin_solve. The result passes
/// through Qₖ at parameter uₖ to solver precision. Requires points.size() ≥
/// degree+1; degenerate/all-coincident inputs return ok=false (honest guard).
CurveFitResult interpolateCurve(std::span<const Point3> points, int degree,
                                ParamMethod method = ParamMethod::ChordLength);

/// A9.4/9.6 — least-squares curve APPROXIMATION: fit a degree-`degree` B-spline
/// with exactly `nCtrl` control points (nCtrl < points.size()) minimizing the
/// summed squared distance to the data, with the FIRST and LAST control points
/// PINNED to the first/last data points (endpoint interpolation). The free
/// interior control points solve the least-squares system via numerics::lstsq.
/// Reports the achieved max / RMS error (no widened tolerance — the true error).
/// Requires degree+1 ≤ nCtrl < points.size(); otherwise ok=false.
CurveFitResult approximateCurve(std::span<const Point3> points, int nCtrl, int degree,
                                ParamMethod method = ParamMethod::ChordLength);

// ─────────────────────────────────────────────────────────────────────────────
// Surface fitting (tensor-product; each row then each column via the curve core).
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a surface fit.
struct SurfaceFitResult {
  bool ok = false;
  BsplineSurfaceData surface;  ///< the fitted non-rational tensor B-spline
  double maxError = 0.0;       ///< max ‖S(uᵢ,vⱼ) − Q(i,j)‖ over the grid
  double rmsError = 0.0;       ///< RMS of the grid deviations
};

/// A grid of (nU × nV) points, row-major with U outer: point(i,j) = pts[i*nV + j],
/// i over U (0..nU-1), j over V (0..nV-1) — the same layout as BsplineSurfaceData
/// poles, so a fitted surface's poles line up with the input indexing.
struct PointGrid {
  std::span<const Point3> pts;
  int nU = 0;
  int nV = 0;
  Point3 at(int i, int j) const { return pts[static_cast<std::size_t>(i) * nV + j]; }
};

/// Tensor-product surface INTERPOLATION (A9.4-class): interpolate every grid point.
/// First interpolate each of the nU rows as a degree-`degreeV` curve in V (giving
/// an intermediate nU × nV net of V-control points sharing one V knot vector), then
/// interpolate each of the nV resulting columns as a degree-`degreeU` curve in U.
/// The result passes through Q(i,j) at (uᵢ,vⱼ). Requires nU ≥ degreeU+1 and
/// nV ≥ degreeV+1; degenerate inputs return ok=false.
SurfaceFitResult interpolateSurface(const PointGrid& grid, int degreeU, int degreeV,
                                    ParamMethod method = ParamMethod::ChordLength);

/// Tensor-product surface APPROXIMATION (A9.4-class): least-squares fit a
/// nCtrlU × nCtrlV control net (each dimension fewer than the data) of degree
/// (degreeU,degreeV). Approximates each row in V to nCtrlV control points, then
/// approximates each resulting column in U to nCtrlU control points, endpoints
/// pinned in both directions. Reports the achieved max / RMS grid error. Requires
/// degreeU+1 ≤ nCtrlU ≤ nU and degreeV+1 ≤ nCtrlV ≤ nV (equality ⇒ interpolation
/// in that direction); otherwise ok=false.
SurfaceFitResult approximateSurface(const PointGrid& grid, int nCtrlU, int nCtrlV,
                                    int degreeU, int degreeV,
                                    ParamMethod method = ParamMethod::ChordLength);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_FIT_H

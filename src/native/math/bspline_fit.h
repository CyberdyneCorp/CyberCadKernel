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
// SCOPE — NON-RATIONAL fitting, plus RATIONAL interpolation with PRESCRIBED weights
// (the tractable exact case). Given data points Qₖ AND a weight wₖ per point, the
// rational routines lift each datum to the homogeneous point Qʷₖ = (wₖ·xₖ, wₖ·yₖ,
// wₖ·zₖ, wₖ) ∈ R⁴, run the SAME non-rational collocation solve on the 4-D net (the
// 4th coordinate solves for the control weights Wᵢ, the first three for wᵢ·Pᵢ), and
// project back Pᵢ = (Xᵢ/Wᵢ, …), weightᵢ = Wᵢ. Because Cʷ(uₖ) = Qʷₖ, the projected
// rational curve passes through the EUCLIDEAN datum Qₖ = (wₖQₖ)/wₖ exactly. This is
// the same rational-lift convention as bspline_ops.h (Layer 1). Full weight
// ESTIMATION from unweighted points (Ma–Kruth) — recovering the wₖ themselves — is
// the HARDER residual and is NOT attempted here; this module never fakes it. See
// docs/NURBS-SCOPE.md Layer-7 row.
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

/// RATIONAL global curve INTERPOLATION with PRESCRIBED weights. Given the same
/// data points as interpolateCurve PLUS a positive weight wₖ per point, build a
/// degree-`degree` RATIONAL NURBS (poles + weights) that passes through every Qₖ at
/// parameter uₖ to solver precision. Each datum is lifted to the homogeneous point
/// Qʷₖ = (wₖ·xₖ, wₖ·yₖ, wₖ·zₖ, wₖ); the SAME averaging-knot collocation matrix as
/// interpolateCurve is solved for all FOUR homogeneous coordinates (the 4th yields
/// the control weights Wᵢ), then the control net is projected back Pᵢ = (Xᵢ/Wᵢ,
/// Yᵢ/Wᵢ, Zᵢ/Wᵢ) with weightᵢ = Wᵢ. Because Cʷ(uₖ) = Qʷₖ, the projected rational
/// curve interpolates the EUCLIDEAN Qₖ exactly. The returned curve.weights has one
/// weight per pole (non-empty ⇒ rational). Requires weights.size()==points.size(),
/// points.size() ≥ degree+1, every input weight strictly POSITIVE, and every
/// SOLVED control weight Wᵢ strictly positive (a projected non-positive weight is a
/// documented guard — never divide by ≤ 0); otherwise ok=false. NOTE — the weights
/// are PRESCRIBED inputs; this does NOT estimate weights from unweighted data
/// (Ma–Kruth) — that harder inverse problem is an explicit residual.
CurveFitResult interpolateRationalCurve(std::span<const Point3> points,
                                        std::span<const double> weights, int degree,
                                        ParamMethod method = ParamMethod::ChordLength);

// ─────────────────────────────────────────────────────────────────────────────
// Rational weight ESTIMATION (Ma–Kruth) — recover BOTH control points AND weights
// from UNWEIGHTED data, so a fitted rational curve can exactly represent conics
// (circle / ellipse) that a polynomial fit cannot. This is the harder inverse that
// interpolateRationalCurve does NOT do (there the weights are prescribed).
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a rational weight-estimating fit.
///
/// The estimator lifts the rational interpolation condition C(uₖ)=Qₖ to its
/// bilinear form Σᵢ Nᵢ(uₖ)·wᵢ·(Pᵢ − Qₖ) = 0. Writing the WEIGHTED control points
/// Hᵢ = wᵢ·Pᵢ this is LINEAR and HOMOGENEOUS in the unknown vector z = (H₀..Hₙ,
/// w₀..wₙ): one 3-vector equation per datum, stacked into M·z = 0. The nontrivial
/// null-space direction (smallest-singular-vector, gauge = overall scale) is the
/// least-squares rational fit; recovered by shifted inverse iteration on MᵀM
/// through numerics::lin_solve (no external SVD). After solving, Pᵢ = Hᵢ/wᵢ and the
/// weights are normalized to the gauge w₀ = 1. See Ma & Kruth, "NURBS curve and
/// surface fitting for reverse engineering" (Computer-Aided Design 1998) and Piegl
/// & Tiller §9 — the classic weight-recovery/eigenproblem formulation.
struct RationalFitResult {
  bool ok = false;               ///< true ⇔ a VALID positive-weight rational fit was recovered
  BsplineCurveData curve;        ///< fitted rational curve (poles + positive weights)
  double maxError = 0.0;         ///< max ‖C(uₖ) − Qₖ‖ over the data
  double rmsError = 0.0;         ///< RMS of ‖C(uₖ) − Qₖ‖ over the data
  double weightSpread = 0.0;     ///< max wᵢ − min wᵢ after w₀=1 gauge (≈0 ⇒ non-rational)
  bool rationalityDetected = false;  ///< true ⇔ weightSpread exceeds the flatness threshold
  const char* diagnostic = "";   ///< on decline: WHY (rank-deficient / sign-flip / degenerate)
};

/// Estimate BOTH the `nCtrl` control points AND their weights of a degree-`degree`
/// rational B-spline that fits the data — the weights are UNKNOWN and recovered
/// (Ma–Kruth), not prescribed. Uses `method` for the parameters and approximation
/// knots (Eq 9.68/9.69), exactly like approximateCurve. The estimation is a
/// HOMOGENEOUS null-space solve, so it must be OVER-determined: pass MORE data than
/// unknowns — a conic (circle/ellipse) arc is a rational quadratic with nCtrl=3,
/// so sample it at ≫3 points and fit with nCtrl=3, degree=2. Requires
/// degree+1 ≤ nCtrl and 3·points.size() > 4·nCtrl (enough data to pin the weights).
///
/// The bilinear fit condition Σᵢ Nᵢ(uₖ)·wᵢ·(Pᵢ − Qₖ) = 0 is linear in z = (Hᵢ, wᵢ)
/// with Hᵢ = wᵢ·Pᵢ (see RationalFitResult): one 3-vector row per datum, stacked into
/// M·z = 0. The nontrivial null-space direction (smallest-singular-vector of M,
/// equivalently the smallest-eigenvalue eigenvector of MᵀM) is the least-squares
/// rational fit, recovered by shifted inverse iteration on MᵀM through
/// numerics::lin_solve (no external SVD). Then Pᵢ = Hᵢ/wᵢ.
///
/// GAUGE — weights are defined only up to a common scale; the result is normalized
/// so w₀ = 1 (the null vector is also sign-normalized so the shared sign is +).
/// GUARDS (HONEST-DECLINE, never a faked rational): declines with a `diagnostic`
/// when the data is degenerate/all-coincident, when the system is not over-
/// determined, when the null space is not cleanly one-dimensional (the two smallest
/// eigenvalues are comparable — rank-deficient, no stable weights), or when the
/// recovered weights do not all share one sign (a sign-flipping weight makes the
/// rational denominator vanish inside the domain — an INVALID NURBS). On success
/// every weight is strictly positive after the gauge fix.
///
/// Detects the POLYNOMIAL (non-rational) case: if the recovered weights are all
/// ≈ equal (weightSpread ≤ `flatTol`, default 1e-6) the data needs no rationality;
/// rationalityDetected is false (the fit is still returned, with ≈unit weights).
RationalFitResult fitRationalCurveEstimateWeights(
    std::span<const Point3> points, int nCtrl, int degree,
    ParamMethod method = ParamMethod::ChordLength, double flatTol = 1e-6);

/// Weight-estimating fit with EXPLICIT parameters and knots — the airtight form for
/// exact conic recovery. Same homogeneous Ma–Kruth null-space solve as the
/// method-based overload, but the caller supplies the parameter uₖ for every datum
/// and the flat knot vector directly. This matters because a rational shape's
/// projective parameter is NOT its chord length: an exact circular/elliptical arc is
/// recovered to MACHINE precision only when the data carries its own NURBS parameter
/// (feed the arc's evaluation parameters). Requires params.size()==points.size(),
/// knots.size()==nCtrl+degree+1, degree+1 ≤ nCtrl ≤ points.size(), and
/// 3·points.size() > 4·nCtrl. Same gauge (w₀=1), positivity, rank and non-rationality
/// handling as the other overload.
RationalFitResult fitRationalCurveEstimateWeightsWithParams(
    std::span<const Point3> points, std::span<const double> params,
    std::span<const double> knots, int nCtrl, int degree, double flatTol = 1e-6);

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
// CONSTRAINED least-squares curve fitting (*The NURBS Book* §9.4.1 — end
// derivatives; the equality-constrained normal equations / KKT solve).
//
// Fit a degree-`degree`, `nCtrl`-control-point B-spline that MINIMIZES the summed
// squared distance to the data WHILE EXACTLY satisfying a set of boundary
// constraints at the START (u=0) and/or END (u=1) of the curve:
//   * fixed endpoint POSITION   C (u_end) = P   (interpolate a prescribed point)
//   * fixed end TANGENT         C'(u_end) = T   (match a prescribed 1st derivative)
//   * fixed end CURVATURE       C"(u_end) = K   (match a prescribed 2nd derivative)
// This is the stitching primitive: constraining an end position + tangent lets a
// fitted patch meet existing geometry with G1 continuity (position + curvature → G2).
//
// Each requested constraint is one LINEAR equation in the control points (its row is
// the basis functions Nᵢ, or their derivatives N'ᵢ / N"ᵢ, evaluated at the end
// parameter — SAME row for all three coordinates, only the right-hand side differs
// per axis). Stacking them into C·x = d and the least-squares design into A·x ≈ b,
// the fit is the KKT system  [AᵀA Cᵀ; C 0][x; λ] = [Aᵀb; d]  solved once per axis
// through numerics::lin_solve. The constraints hold to solver precision; the free
// remaining degrees of freedom absorb the interior data in the least-squares sense.
// ─────────────────────────────────────────────────────────────────────────────

/// Which end of the curve a constraint applies to.
enum class CurveEnd {
  Start,  ///< u = 0 (clamped: the first control point / start derivatives)
  End,    ///< u = 1 (clamped: the last control point / end derivatives)
};

/// One boundary constraint: the `order`-th derivative of the curve at `end` must
/// equal `value`. order 0 = position (interpolate the point), order 1 = tangent
/// (first derivative), order 2 = curvature vector (second derivative). Each is an
/// EXACT equality the fit satisfies to solver precision.
struct CurveEndConstraint {
  CurveEnd end = CurveEnd::Start;
  int order = 0;    ///< 0 = position, 1 = 1st derivative, 2 = 2nd derivative
  Vec3 value{};     ///< the prescribed value (a point for order 0; a derivative vector otherwise)
};

/// A9.4-class equality-CONSTRAINED least-squares curve fit. Fits a degree-`degree`,
/// `nCtrl`-control-point B-spline minimizing the summed squared distance to `points`
/// (chord-length / centripetal / uniform parameters, approximation knots Eq 9.68/9.69
/// — identical to approximateCurve) SUBJECT TO every constraint in `constraints`
/// holding EXACTLY. Formulated as the KKT system [AᵀA Cᵀ; C 0][x;λ]=[Aᵀb;d] and solved
/// through numerics::lin_solve, once per coordinate (the constraint block is shared,
/// only the RHS d differs by axis). Reports the achieved max / RMS error over the data
/// (the TRUE error of the constrained fit — never a widened tolerance).
///
/// REDUCTION — with an EMPTY `constraints` list this reproduces the unconstrained
/// approximateCurve result to solver precision (the KKT system degenerates to the
/// plain normal equations AᵀA x = Aᵀb). NOTE — unlike approximateCurve, endpoints are
/// NOT auto-pinned here: pin them explicitly with order-0 Start/End constraints when
/// you want endpoint interpolation. This keeps the semantics of `constraints` total.
///
/// GUARDS (HONEST-DECLINE, never a faked fit, never a widened tolerance):
///   * degree < 1, nCtrl < degree+1, or nCtrl > points.size() → ok=false
///   * OVER-CONSTRAINED: more independent constraints than control points
///     (constraints.size() ≥ nCtrl) leaves no least-squares freedom → ok=false
///   * an order > 2 constraint, or a degree too low to carry the requested
///     derivative (order > degree) → ok=false
///   * a rank-deficient / inconsistent constraint set makes the KKT matrix singular;
///     numerics::lin_solve returns empty and the fit declines → ok=false
CurveFitResult fitCurveConstrained(std::span<const Point3> points,
                                   std::span<const CurveEndConstraint> constraints, int degree,
                                   int nCtrl, ParamMethod method = ParamMethod::ChordLength);

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

/// A grid of prescribed weights parallel to a PointGrid: weight(i,j) = w[i*nV + j],
/// same row-major U-outer layout as the data points and the surface pole net.
struct WeightGrid {
  std::span<const double> w;
  int nU = 0;
  int nV = 0;
  double at(int i, int j) const { return w[static_cast<std::size_t>(i) * nV + j]; }
};

/// RATIONAL tensor-product surface INTERPOLATION with PRESCRIBED weights — the
/// surface analogue of interpolateRationalCurve. Each grid datum Q(i,j) is lifted
/// to the homogeneous point Qʷ(i,j) = (w·x, w·y, w·z, w) using its prescribed weight
/// w(i,j), the tensor-product interpolation runs on the 4-D homogeneous net (the
/// weight coordinate is interpolated exactly like x/y/z), and the resulting control
/// net is projected back to (pole, weight). The rational surface passes through
/// every EUCLIDEAN Q(i,j) at (uᵢ,vⱼ) to solver precision, and surface.weights holds
/// one weight per pole (non-empty ⇒ rational). Requires wg dimensions == grid
/// dimensions, nU ≥ degreeU+1, nV ≥ degreeV+1, every input weight strictly POSITIVE,
/// and every solved control weight strictly positive (documented guard); otherwise
/// ok=false. Weights are PRESCRIBED — this does NOT estimate weights (residual).
SurfaceFitResult interpolateRationalSurface(const PointGrid& grid, const WeightGrid& wg,
                                            int degreeU, int degreeV,
                                            ParamMethod method = ParamMethod::ChordLength);

// ─────────────────────────────────────────────────────────────────────────────
// Rational SURFACE weight ESTIMATION (Ma–Kruth, tensor-product) — recover BOTH the
// control-point net AND the weight net from an UNWEIGHTED grid, so a fitted rational
// surface can exactly represent a quadric patch (quarter-cylinder, sphere octant)
// that a polynomial fit cannot. The surface analogue of
// fitRationalCurveEstimateWeights — the harder inverse that
// interpolateRationalSurface does NOT do (there the weights are prescribed).
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a rational SURFACE weight-estimating fit.
///
/// Lifts the tensor-product rational-fit condition S(uₖ,vₗ)=Q(k,l) to its bilinear
/// form Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ·(Pᵢⱼ − Q(k,l)) = 0. Writing the WEIGHTED control net
/// Hᵢⱼ = wᵢⱼ·Pᵢⱼ this is LINEAR and HOMOGENEOUS in the unknown vector
/// z = (H₀₀..H_{mn}, w₀₀..w_{mn}): one 3-vector equation per grid datum, stacked into
/// M·z = 0. The nontrivial null-space direction (smallest-singular-vector, gauge =
/// overall scale) is the least-squares rational fit; recovered by the SAME shifted
/// inverse iteration on MᵀM through numerics::lin_solve (no external SVD) as the curve
/// case. After solving, Pᵢⱼ = Hᵢⱼ/wᵢⱼ and the weights are normalized to w₀₀ = 1.
/// See Ma & Kruth (Computer-Aided Design 1998) and Piegl & Tiller §9.
struct RationalSurfaceFitResult {
  bool ok = false;                   ///< true ⇔ a VALID positive-weight rational surface was recovered
  BsplineSurfaceData surface;        ///< fitted rational surface (poles + positive weights)
  double maxError = 0.0;             ///< max ‖S(uᵢ,vⱼ) − Q(i,j)‖ over the grid
  double rmsError = 0.0;             ///< RMS of the grid deviations
  double weightSpread = 0.0;         ///< max wᵢⱼ − min wᵢⱼ after w₀₀=1 gauge (≈0 ⇒ non-rational)
  bool rationalityDetected = false;  ///< true ⇔ weightSpread exceeds the flatness threshold
  const char* diagnostic = "";       ///< on decline: WHY (rank-deficient / sign-flip / degenerate)
};

/// Estimate BOTH the (nCtrlU × nCtrlV) control net AND its weights of a degree
/// (degreeU,degreeV) rational tensor-product B-spline surface that fits the grid —
/// the weights are UNKNOWN and recovered (Ma–Kruth), not prescribed. Uses `method`
/// for the U/V parameters (averaged across the grid, Euclidean geometry) and
/// approximation knots (Eq 9.68/9.69). The estimation is a HOMOGENEOUS null-space
/// solve, so it must be OVER-determined: pass MORE grid data than unknowns. A
/// quarter-cylinder / sphere-octant patch is a rational quadratic in the circular
/// direction — sample it densely and fit with the small control net that spans it.
/// Requires degreeU+1 ≤ nCtrlU ≤ nU, degreeV+1 ≤ nCtrlV ≤ nV, and
/// 3·nU·nV > 4·nCtrlU·nCtrlV (enough grid data to pin the weights).
///
/// GAUGE — weights are defined only up to a common scale; the result is normalized so
/// w₀₀ = 1 (the null vector is also sign-normalized so the shared sign is +).
/// GUARDS (HONEST-DECLINE, never a faked rational): declines with a `diagnostic` when
/// the grid is degenerate, when the system is not over-determined, when the null space
/// is not cleanly one-dimensional (the two smallest eigenvalues are comparable —
/// rank-deficient, no stable weights), or when the recovered weights do not all share
/// one sign (a sign-flipping / near-zero weight makes the rational denominator vanish
/// inside the domain — an INVALID NURBS). On success every weight is strictly positive.
///
/// Detects the POLYNOMIAL (non-rational) case: if the recovered weights are all
/// ≈ equal (weightSpread ≤ `flatTol`, default 1e-6) the data needs no rationality;
/// rationalityDetected is false (the fit is still returned, with ≈unit weights).
RationalSurfaceFitResult fitRationalSurfaceEstimateWeights(
    const PointGrid& grid, int nCtrlU, int nCtrlV, int degreeU, int degreeV,
    ParamMethod method = ParamMethod::ChordLength, double flatTol = 1e-6);

/// Surface weight-estimating fit with EXPLICIT U/V parameters and knots — the airtight
/// form for exact quadric recovery. Same homogeneous Ma–Kruth tensor null-space solve
/// as the method-based overload, but the caller supplies the parameter uᵢ for every
/// grid ROW, vⱼ for every grid COLUMN, and both flat knot vectors directly. This
/// matters because a rational shape's projective parameter is NOT its chord length: an
/// exact quarter-cylinder / sphere octant is recovered to MACHINE precision only when
/// the data carries its own NURBS parameters. Requires uParams.size()==grid.nU,
/// vParams.size()==grid.nV, knotsU.size()==nCtrlU+degreeU+1,
/// knotsV.size()==nCtrlV+degreeV+1, the dimension bounds above, and
/// 3·nU·nV > 4·nCtrlU·nCtrlV. Same gauge (w₀₀=1), positivity, rank and non-rationality
/// handling as the other overload.
RationalSurfaceFitResult fitRationalSurfaceEstimateWeightsWithParams(
    const PointGrid& grid, std::span<const double> uParams, std::span<const double> vParams,
    std::span<const double> knotsU, std::span<const double> knotsV, int nCtrlU, int nCtrlV,
    int degreeU, int degreeV, double flatTol = 1e-6);

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

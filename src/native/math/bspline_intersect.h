// SPDX-License-Identifier: Apache-2.0
//
// bspline_intersect.h — exact NURBS curve↔curve (CCI) and curve↔surface (CSI)
// intersection for the native math kernel (Wave-H, track H1).
//
// These are the FOUNDATIONAL intersection primitives the trimming / pcurve /
// edge-computation layers (L2/L3) build on. The kernel already ships surface–
// surface intersection (src/native/ssi); this module adds the two missing
// first-class families:
//
//   * intersectCurveCurve(cA, cB)   — every point where two NURBS curves meet.
//   * intersectCurveSurface(c, S)   — every point where a NURBS curve pierces a
//                                     NURBS surface.
//
// METHOD (Piegl & Tiller, *The NURBS Book*, Ch. 5 subdivision / Ch. 6 point
// inversion & projection):
//
//   1. ISOLATE candidates by RECURSIVE BOUNDING-BOX subdivision. Each curve /
//      surface is bounded over a parameter sub-box by the CONVEX HULL of the
//      control points whose basis support overlaps that sub-box (the convex-hull
//      property of the B-spline basis; for a rational NURBS with wᵢ > 0 the
//      point is still a convex combination of the PROJECTED poles, so the hull of
//      the projected poles is a SOUND bound). Disjoint boxes are pruned; a box
//      pair small enough in parameter (or 3D) space yields ONE candidate seed.
//
//   2. POLISH each seed with NEWTON on the square system:
//        CCI: F(u,v) = cA(u) − cB(v) = 0            projected onto (cA', cB')
//        CSI: F(t,u,v) = c(t) − S(u,v) = 0          (3 equations, 3 unknowns)
//      using the analytic first derivatives from bspline.h (rational-aware,
//      quotient rule). Converged roots inside the domain are kept.
//
//   3. CLASSIFY each root TRANSVERSAL vs TANGENTIAL (CCI: cross(cA',cB') ≈ 0;
//      CSI: cA' ⟂ surface normal, i.e. dot(c', n) ≈ 0), DEDUPE coincident roots
//      by 3D proximity, and HONEST-DECLINE overlapping / coincident inputs (an
//      infinite intersection set is not a finite point list — status Coincident).
//
// HONESTY CONTRACT (hard invariant): this module NEVER widens a tolerance to
// manufacture a hit, and NEVER returns a finite point list for a coincident /
// overlapping pair. A pair it cannot resolve robustly is reported Coincident
// (overlap) — the caller falls back; it is never faked. This mirrors the SSI
// module's NotAnalytic / Coincident honesty seams.
//
// Rational curves/surfaces are supplied by passing a non-empty `weights` span
// (parallel to the poles); an empty span means non-rational (all wᵢ = 1). The
// homogeneous lift used for both the hull bound and the Newton derivatives keeps
// the code exact for rational circles / cylinders (the airtight oracles).
//
// OCCT-FREE. Uses src/native/math only (bspline.h + vec.h). clang++ -std=c++20,
// fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_INTERSECT_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_INTERSECT_H

#include "vec.h"

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Input geometry descriptors. These are non-owning VIEWS over caller-owned pole /
// weight / knot storage (std::span), matching the bspline.h evaluator signatures.
// A rational curve/surface passes a non-empty `weights` (wᵢ > 0, parallel to the
// poles); an empty `weights` means non-rational.
// ─────────────────────────────────────────────────────────────────────────────

/// A NURBS (or B-spline) curve view: degree, poles, optional weights, flat knots.
struct CurveView {
  int degree = 1;
  std::span<const Point3> poles{};      ///< n+1 control points
  std::span<const double> weights{};    ///< empty ⇒ non-rational; else parallels poles
  std::span<const double> knots{};      ///< flat, length n+p+2

  bool rational() const noexcept { return !weights.empty(); }
  int numPoles() const noexcept { return static_cast<int>(poles.size()); }
  /// Clamped parameter domain [knots[degree], knots[numPoles]].
  double t0() const noexcept { return knots[static_cast<std::size_t>(degree)]; }
  double t1() const noexcept { return knots[static_cast<std::size_t>(numPoles())]; }
};

/// A NURBS (or B-spline) surface view: bidegree, pole grid, optional weights, knots.
struct SurfaceView {
  int degreeU = 1;
  int degreeV = 1;
  std::span<const Point3> poles{};      ///< nRows*nCols row-major (U outer, V inner)
  std::span<const double> weights{};    ///< empty ⇒ non-rational; else parallels poles
  int nRows = 0;                        ///< #poles in U
  int nCols = 0;                        ///< #poles in V
  std::span<const double> knotsU{};     ///< flat U knots, length nRows+degreeU+1
  std::span<const double> knotsV{};     ///< flat V knots, length nCols+degreeV+1

  bool rational() const noexcept { return !weights.empty(); }
  double u0() const noexcept { return knotsU[static_cast<std::size_t>(degreeU)]; }
  double u1() const noexcept { return knotsU[static_cast<std::size_t>(nRows)]; }
  double v0() const noexcept { return knotsV[static_cast<std::size_t>(degreeV)]; }
  double v1() const noexcept { return knotsV[static_cast<std::size_t>(nCols)]; }
};

// ─────────────────────────────────────────────────────────────────────────────
// Results.
// ─────────────────────────────────────────────────────────────────────────────

/// The geometric character of an intersection point.
enum class IntersectionType {
  Transversal,  ///< the curves/surface cross cleanly (non-parallel tangents)
  Tangential,   ///< tangent contact (parallel tangents / curve ∥ surface at the hit)
};

/// One curve↔curve intersection point: the 3D point and both curve parameters.
struct CurveCurveHit {
  Point3 point{};
  double paramA = 0.0;   ///< parameter on cA
  double paramB = 0.0;   ///< parameter on cB
  double gap = 0.0;      ///< residual ‖cA(paramA) − cB(paramB)‖ at the root
  IntersectionType type = IntersectionType::Transversal;
};

/// One curve↔surface intersection point: the 3D point, curve param, surface (u,v).
struct CurveSurfaceHit {
  Point3 point{};
  double paramT = 0.0;   ///< parameter on the curve
  double paramU = 0.0;   ///< surface u
  double paramV = 0.0;   ///< surface v
  double gap = 0.0;      ///< residual ‖c(t) − S(u,v)‖ at the root
  IntersectionType type = IntersectionType::Transversal;
};

/// Query status. Mirrors the SSI honesty seam: Ok (0..N isolated points),
/// Coincident (the inputs OVERLAP / are the same locus over a sub-arc — an
/// infinite intersection set, honest-declined, NEVER faked as points).
enum class IntersectStatus {
  Ok,          ///< 0..N isolated intersection points (see the hit list)
  Coincident,  ///< curves/curve-on-surface overlap ⇒ infinite intersection → declined
};

struct CurveCurveResult {
  IntersectStatus status = IntersectStatus::Ok;
  std::vector<CurveCurveHit> hits{};
};

struct CurveSurfaceResult {
  IntersectStatus status = IntersectStatus::Ok;
  std::vector<CurveSurfaceHit> hits{};
};

// ─────────────────────────────────────────────────────────────────────────────
// Entry points.
// ─────────────────────────────────────────────────────────────────────────────

/// All intersection points of two NURBS curves. Handles planar and 3D
/// (near-miss) inputs: only genuine crossings with 3D gap ≤ `tol` are returned.
/// Tangent contacts are reported with IntersectionType::Tangential. Overlapping /
/// coincident curves are honest-declined (status Coincident, empty hits).
CurveCurveResult intersectCurveCurve(const CurveView& cA, const CurveView& cB,
                                     double tol = 1e-9);

/// All points where a NURBS curve pierces a NURBS surface. Curve tangent to the
/// surface at a hit ⇒ IntersectionType::Tangential. A curve lying ON the surface
/// over a sub-arc is honest-declined (status Coincident, empty hits).
CurveSurfaceResult intersectCurveSurface(const CurveView& c, const SurfaceView& S,
                                         double tol = 1e-9);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_INTERSECT_H

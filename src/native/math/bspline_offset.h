// SPDX-License-Identifier: Apache-2.0
//
// bspline_offset.h — NURBS roadmap Layer 5: surface OFFSET (base of thicken/shell).
//
// Given a tensor-product NURBS surface S(u,v), construct an OFFSET surface at
// signed distance d: the true offset locus is the point set
//
//     O(u,v) = S(u,v) + d · N(u,v)        N = unit surface normal (bspline.h)
//
// The EXACT offset of a NURBS surface is NOT a NURBS in general (the normal field
// carries a square root, so O is not piecewise-rational). It is therefore
// APPROXIMATED (docs/NURBS-SCOPE.md §4 Layer 5, *The NURBS Book* Piegl & Tiller
// §10.? offset approximation): sample the true offset locus O on a (u,v) grid, FIT
// a non-rational B-spline surface through the samples with the Layer-7 fitter
// (bspline_fit.h → tensor-product interpolation, `numerics::lin_solve`), and REFINE
// the grid until the fitted surface's maximum deviation from the true locus O is
// within a caller tolerance. The achieved error is REPORTED, never widened — a
// fitted offset that cannot reach tolerance within the refinement budget returns
// its best result flagged with the true error, so the caller sees the honest gap.
//
// This module sits ABOVE and COMPOSES three landed layers, modifying none of them:
//   * bspline.h        — surfacePoint / nurbsSurfacePoint + surfaceNormal (the unit
//                        normal N we offset along) and surfaceDerivs (∂²S for the
//                        curvature-radius self-intersection guard).
//   * bspline_ops.h    — the Layer-1 BsplineSurfaceData carrier (flat knots,
//                        row-major U-outer poles); both input S and output fit use it.
//   * bspline_fit.h    — Layer-7 interpolateSurface: fit a NURBS surface through the
//                        sampled offset grid (the constructor for the offset surface).
//
// HONEST DEGENERACY GUARDS (never return a folded / self-intersecting offset):
//   * SELF-INTERSECTION — an offset by |d| that meets or exceeds a principal RADIUS
//     OF CURVATURE on the concave side folds the surface onto itself. This is
//     DETECTED from the surface's second fundamental form (principal curvatures κ₁,κ₂
//     via surfaceDerivs) and the offset is DECLINED with reason "offset exceeds
//     curvature radius" — it is NOT silently returned folded.
//   * DEGENERATE NORMAL — a patch with a near-zero normal (‖∂S/∂u × ∂S/∂v‖ ≈ 0, e.g.
//     a collapsed/singular grid point) has no defined offset direction and DECLINES.
//
// SCOPE — NON-RATIONAL offset (the fitted result has empty weights). The input S may
// be rational (weights are honoured through nurbsSurfacePoint / surfaceNormal), but
// the offset APPROXIMATION is fitted non-rationally. SOLID thicken / shell / hollow
// (offset both faces + stitch side walls into a closed solid) and robust
// self-intersecting-offset TRIMMING (recover a valid offset by trimming the folded
// region rather than declining) are documented residuals for later slices — this
// module never fakes them. See docs/NURBS-SCOPE.md Layer-5 row.
//
// GUARD — the fit-bearing routine is compiled only when CYBERCAD_HAS_NUMSCI is
// defined (the numsci facade is the sole linear-algebra dependency, via the Layer-7
// interpolation), exactly like bspline_fit.cpp / bspline_skin.cpp. With the guard OFF
// the implementation TU is inert and the function is absent; the declaration remains
// visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_OFFSET_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_OFFSET_H

#include "bspline_ops.h"  // BsplineSurfaceData (Layer-1 data type)

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Offset result.
// ─────────────────────────────────────────────────────────────────────────────

/// Why an offset request was declined (`ok == false`).
enum class OffsetStatus {
  Ok = 0,                 ///< a valid offset surface was produced.
  DegenerateInput,        ///< malformed / empty / degree < 1 input surface.
  DegenerateNormal,       ///< the surface has a near-zero normal somewhere on the grid.
  SelfIntersection,       ///< |d| meets/exceeds a principal radius of curvature (would fold).
  FitFailed,              ///< the Layer-7 surface fit did not solve (singular collocation).
  ToleranceNotMet,        ///< best fit exceeds `tol` after the refinement budget (surface is
                          ///< still returned; `maxError` is the honest achieved deviation).
};

/// Result of an offset-surface construction.
struct OffsetResult {
  bool ok = false;             ///< true ⇔ a valid offset within tolerance was produced.
  OffsetStatus status = OffsetStatus::DegenerateInput;
  BsplineSurfaceData surface;  ///< the fitted non-rational offset surface (empty on decline).
  double maxError = 0.0;       ///< max ‖fitted(uᵢ,vⱼ) − O(uᵢ,vⱼ)‖ over a dense CHECK grid —
                               ///< the true, un-widened deviation of the fit from the offset locus.
  int gridU = 0, gridV = 0;    ///< the sample-grid resolution the accepted fit used.
  double minCurvatureRadius = 0.0;  ///< smallest principal radius of curvature seen on the
                                    ///< OFFSETTING side (the self-intersection bound); 0 if
                                    ///< the surface is (locally) flat there.
};

// ─────────────────────────────────────────────────────────────────────────────
// Offset surface.
// ─────────────────────────────────────────────────────────────────────────────

/// Construct the OFFSET of `surface` at signed distance `d` as a fitted non-rational
/// B-spline surface (NURBS-SCOPE Layer 5). Positive `d` offsets along +N (the surface
/// normal ∂S/∂u × ∂S/∂v, normalized), negative along −N.
///
/// Algorithm:
///   1. Guard the input (well-formed, degree ≥ 1). Sweep a coarse (u,v) grid over S's
///      parameter domain and, at every node, evaluate S, the unit normal N, and the
///      principal curvatures. If any node's normal is degenerate → decline
///      (DegenerateNormal). If |d| ≥ the smallest principal radius of curvature on the
///      side `d` bends toward → decline (SelfIntersection): the offset would fold.
///   2. Sample the true offset locus O(u,v) = S + d·N on the grid and INTERPOLATE a
///      degree-(min(3,·)) tensor B-spline through it with the Layer-7 fitter.
///   3. Measure the fit's deviation from O on a denser CHECK grid (independent of the
///      fit nodes), then REFINE (double the sample density, capped by `maxGrid`) until
///      the deviation ≤ `tol` or the budget is spent. Return the best fit with its true
///      achieved `maxError` (flagged ToleranceNotMet if the budget was spent short).
///
/// The offset-distance GUARANTEE the host gate checks: every point of the returned
/// surface lies at distance ≈ |d| from S along S's normal, within the reported error.
///
/// `tol` is the target max deviation of the fit from the true offset locus (default
/// 1e-4 of the surface's parametric scale). `startGrid`/`maxGrid` bound the sample
/// resolution per direction. Declines (ok=false, empty surface) rather than ever
/// returning a folded or degenerate offset.
OffsetResult offsetSurface(const BsplineSurfaceData& surface, double d,
                           double tol = 1e-4, int startGrid = 9, int maxGrid = 33);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_OFFSET_H

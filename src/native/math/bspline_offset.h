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
// SCOPE — the base `offsetSurface` fits NON-RATIONALLY (empty weights). Two additive
// entry points extend it:
//   * offsetSurfaceRational — for a RATIONAL input, samples the offset locus in
//     homogeneous form (per-node effective weight) and fits a RATIONAL approximant,
//     so an exact conic (cylinder → coaxial cylinder r±d, sphere → sphere R±d) offsets
//     to the exact offset conic (deviation ≤ ~1e-6). Falls back to the non-rational fit
//     for non-rational input / when it does not improve. Weights are never fabricated.
//   * offsetSurfaceTrimmed — when the offset self-intersects (folds) over PART of the
//     domain, TRIMS to the maximal fold-free parameter rectangle and returns a valid
//     offset there (plus the kept region), declining only when no meaningful fold-free
//     region remains. A self-intersecting surface is NEVER returned as valid.
// SOLID thicken / shell / hollow (offset both faces + stitch side walls into a closed
// solid) lives in bspline_thicken / bspline_shell. Non-rectangular (curved) fold-trim
// boundaries and weight ESTIMATION remain documented residuals — never faked here.
// See docs/NURBS-SCOPE.md Layer-5 row.
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

  // ── Additive fields (default-valued; non-trim/non-rational callers are unaffected) ──
  bool trimmed = false;        ///< true ⇔ the domain was TRIMMED to a fold-free sub-region
                               ///< (offsetSurfaceTrimmed only). false ⇒ full-domain offset.
  double keptU0 = 0.0, keptU1 = 0.0;  ///< the kept parameter rectangle in the INPUT surface's
  double keptV0 = 0.0, keptV1 = 0.0;  ///< (u,v) coordinates; equals the full domain when
                                      ///< `trimmed == false`. The complement is the region
                                      ///< trimmed away because the offset folded there.
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

// ─────────────────────────────────────────────────────────────────────────────
// RATIONAL offset (additive; existing offsetSurface stays byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// Construct the OFFSET of `surface` at signed distance `d` as a fitted RATIONAL B-spline
/// surface when `surface` is rational (has weights). Samples the SAME true offset locus
/// O(u,v) = S + d·N as offsetSurface (S, N evaluated rational-aware in homogeneous form),
/// but assigns each sample the input surface's EFFECTIVE rational weight w(u,v) at that node,
/// lifts each sample to the homogeneous point (w·x, w·y, w·z, w), and fits a rational
/// approximant through the samples with the Layer-7 rational interpolation
/// (interpolateRationalSurface). The grid is refined identically to offsetSurface and the
/// TRUE achieved deviation from the offset locus is reported (never widened).
///
/// For the analytic conics whose offset IS an exact rational surface — a cylinder offsets to
/// a coaxial cylinder radius r±d, a sphere to a concentric sphere R±d — the reproduced weight
/// profile makes the fitted offset the EXACT offset conic (deviation ≤ ~1e-6). For a
/// NON-RATIONAL input, or if the rational fit does not improve on the non-rational one, this
/// falls back to offsetSurface's result (never worse). Weights are NEVER fabricated: the fit
/// uses only the input's prescribed weight pattern.
///
/// Same guards / decline semantics as offsetSurface (degenerate normal, self-intersection).
OffsetResult offsetSurfaceRational(const BsplineSurfaceData& surface, double d,
                                   double tol = 1e-4, int startGrid = 9, int maxGrid = 33);

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-TRIMMED offset (additive; recover a valid offset by trimming, not declining).
// ─────────────────────────────────────────────────────────────────────────────

/// Construct the OFFSET of `surface` at signed distance `d`, but when the offset would
/// SELF-INTERSECT (fold) over PART of the parameter domain, TRIM the domain to the maximal
/// fold-free axis-aligned parameter rectangle and return a valid offset over just that region
/// (rather than declining the whole request, as offsetSurface does).
///
/// The fold-free region is found from the offset map's principal Jacobian factors (1 + d·κᵢ)
/// (κ from the surface's first/second fundamental forms) on a dense analysis grid: a node is
/// fold-free iff every (1 + d·κ) > 0. The maximal all-fold-free axis-aligned rectangle of that
/// grid is located and shrunk inward by ½ cell so its interior is provably fold-free; the
/// offset core (sample → fit → refine) is then run over that kept rectangle.
///
/// The result reports `trimmed` and the kept rectangle keptU0..keptV1 (in the input surface's
/// parameter coordinates); the complement is the trimmed-away folded region. When the WHOLE
/// domain is fold-free the full offset is returned with `trimmed == false`. When no fold-free
/// region of meaningful area remains, DECLINES (SelfIntersection, ok=false, empty surface) —
/// a self-intersecting (folded) surface is NEVER returned as valid.
OffsetResult offsetSurfaceTrimmed(const BsplineSurfaceData& surface, double d,
                                  double tol = 1e-4, int startGrid = 9, int maxGrid = 33);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_OFFSET_H

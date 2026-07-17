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

#include <vector>

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

  // ── Fold-LOCUS trim (offsetSurfaceFoldTrim only; default-valued for every other path) ──
  bool foldTrimmed = false;    ///< true ⇔ the kept region is bounded by the actual (diagonal /
                               ///< curved) FOLD LOCUS, not the axis-aligned rectangle staircase.
                               ///< When true the kept region is the COLUMN-BAND swept by the
                               ///< per-u v-interval [foldVLo[k], foldVHi[k]] below (a curved-
                               ///< boundary region that follows the fold), keptU0..keptV1 is its
                               ///< bounding box, and the fitted surface is parametrized over the
                               ///< warped band (fold-free everywhere, at distance |d| from S).
  std::vector<double> foldU;   ///< the sample u-stations of the kept column-band (ascending).
  std::vector<double> foldVLo; ///< per-station lower v of the fold-free interval (‖foldU‖).
  std::vector<double> foldVHi; ///< per-station upper v of the fold-free interval (‖foldU‖).
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

// ─────────────────────────────────────────────────────────────────────────────
// MULTI-REGION fold-trimmed offset (additive; offsetSurfaceTrimmed byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// Construct the OFFSET of `surface` at signed distance `d`, recovering a valid offset over
/// EVERY meaningful fold-free parameter rectangle — not just the single maximal one
/// offsetSurfaceTrimmed keeps.
///
/// A fold that runs as a BAND across the domain (a ridge/valley crossing in u or v) splits
/// the fold-free parameter space into TWO (or more) disjoint axis-aligned rectangles on
/// either side of the band. offsetSurfaceTrimmed keeps only the single largest rectangle and
/// silently discards the other fold-free region(s) — up to ~half the valid material. This
/// entry point instead performs a GREEDY maximal-rectangle COVER of the fold-free node map:
/// repeatedly extract the largest all-fold-free rectangle, mask it out, and repeat, until no
/// remaining rectangle covers at least `kMinKeptFraction` of the domain. Each kept rectangle
/// is offset independently (the SAME sample→fit→refine core, each individually fold-free and
/// within tolerance) and returned as its own OffsetResult with `trimmed == true` and its kept
/// rectangle reported.
///
/// Behaviour contract:
///   * FOLD-FREE everywhere — returns a SINGLE result identical to offsetSurface (`trimmed ==
///     false`, full-domain kept rectangle).
///   * SINGLE fold-free region — returns a single result identical to offsetSurfaceTrimmed.
///   * BAND / MULTIPLE fold-free regions — returns ONE result per meaningful rectangle, in
///     descending area order; their union is the recovered fold-free material.
///   * FULLY FOLDING / degenerate — returns an EMPTY vector (honest-decline; a folded surface
///     is NEVER returned as valid). A degenerate-normal input likewise returns empty.
///
/// The rectangles are pairwise non-overlapping and each is provably fold-free (½-cell inset,
/// same as offsetSurfaceTrimmed). Never widens tolerance; never emits a folded region.
std::vector<OffsetResult> offsetSurfaceMultiTrimmed(const BsplineSurfaceData& surface, double d,
                                                    double tol = 1e-4, int startGrid = 9,
                                                    int maxGrid = 33);

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-LOCUS-following trim (additive; every routine above stays byte-unchanged).
// ─────────────────────────────────────────────────────────────────────────────

/// Construct the OFFSET of `surface` at signed distance `d`, trimming to the actual FOLD
/// LOCUS rather than to an axis-aligned rectangle (or staircase of rectangles).
///
/// The multi-region trim above covers the fold-free parameter map with AXIS-ALIGNED
/// rectangles. When the fold runs DIAGONALLY in (u,v) (a ridge along a slanted line) or along
/// a CURVED locus, each fold-free side is a TRIANGULAR / curved-boundary region, and an
/// axis-aligned rectangle inscribed in a triangle recovers only a fraction of its area — the
/// documented "staircase" residual. This entry point instead traces the fold boundary as a
/// per-u v-interval and returns a valid offset over the COLUMN-BAND that FOLLOWS the fold:
///
///   1. Build the fold-free node map (the same (1 + d·κᵢ) > 0 test as the rectangle trim).
///   2. Split the fold-free map into connected COMPONENTS (a diagonal band leaves two).
///   3. For each component, at every sample u-column record the maximal contiguous fold-free
///      v-interval [vLo(u), vHi(u)] (½-cell inset so the interval interior is provably
///      fold-free). The interval endpoints trace the fold locus as u varies.
///   4. Fit the offset over the WARPED band (s,t) ↦ (u(s), vLo(u)+t·(vHi(u)−vLo(u))): sample
///      the true offset locus O = S + d·N on that curved-boundary region and interpolate a
///      NURBS surface through the samples (same sample→fit→refine core as offsetSurface, over
///      a warped rather than rectangular domain). The fitted patch is fold-free everywhere and
///      lies at distance |d| from S.
///
/// Behaviour contract:
///   * FOLD-FREE everywhere — returns a SINGLE result identical to offsetSurface
///     (`foldTrimmed == false`, `trimmed == false`, full-domain kept box).
///   * DIAGONAL / CURVED fold — returns ONE result per fold-free component, each with
///     `foldTrimmed == true`, the column-band polyline (foldU / foldVLo / foldVHi) reported,
///     and keptU0..keptV1 the band's bounding box. Their union follows the fold locus and
///     recovers strictly MORE area than the rectangle staircase (offsetSurfaceMultiTrimmed).
///   * FULLY FOLDING / degenerate — returns an EMPTY vector (honest-decline; a folded surface
///     is NEVER returned as valid). A degenerate-normal input likewise returns empty.
///
/// Every returned patch is provably fold-free (½-cell inset on the traced interval) and within
/// tolerance. Never widens tolerance; never emits a folded region.
std::vector<OffsetResult> offsetSurfaceFoldTrim(const BsplineSurfaceData& surface, double d,
                                                double tol = 1e-4, int startGrid = 9,
                                                int maxGrid = 33);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_OFFSET_H

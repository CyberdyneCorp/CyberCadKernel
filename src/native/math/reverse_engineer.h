// SPDX-License-Identifier: Apache-2.0
//
// reverse_engineer.h — NURBS roadmap Layer 7 reverse-engineering PIPELINE: given a
// RAW point cloud sampled from a COMPOSITE CAD part, SEGMENT it into regions each
// best-explained by an analytic primitive OR a freeform patch, and FIT each region.
//
// The other Layer-7 modules answer LOCAL questions — primitive_fit.h asks "do THESE
// points lie on a plane/sphere/cylinder/cone?", bspline_fit.h asks "fit a B-spline to
// THIS grid". Neither SEGMENTS: a scan of a rounded box is 6 planes + 12 fillet
// cylinder-quarters + 8 sphere-corner octants in ONE cloud, and the reverse-engineer
// step is deciding WHICH points belong to WHICH face before any single-surface fit is
// even meaningful. This module is that missing piece, and it COMPOSES the existing
// fitters — it never re-implements fitting:
//
//   segmentAndFit(points, normals?, tol) → a set of TYPED regions. Each region is a
//   partition of the input point indices, classified to
//     Plane / Sphere / Cylinder / Cone   (an analytic primitive fit within `tol`), or
//     Freeform                           (a fitted tensor B-spline patch, RMS ≤ tol), or
//     Declined                           (HONEST-DECLINE: fits no primitive within tol
//                                         AND is too sparse/degenerate for a stable
//                                         freeform fit — reported unassigned, never a
//                                         forced wrong primitive).
//
// ALGORITHM (region growing, textbook RANSAC-free deterministic variant):
//   1. Build a symmetric k-nearest-neighbour adjacency over the cloud (brute-force
//      fp64 distances — OCCT-free, deterministic; no spatial index dependency).
//   2. SEED from the lowest unassigned index. Grow a candidate region by a
//      breadth-first frontier: a neighbour joins iff, with the CURRENT best-fit
//      primitive (re-fit as the region grows via primitive_fit::detectPrimitive on the
//      inlier set), its residual to that primitive is ≤ `tol`. Growth stops at the
//      residual-`tol` frontier (no more consistent neighbours).
//   3. CLASSIFY + FIT the grown region: detectPrimitive (if a primitive fits within
//      tol) ELSE attempt a freeform tensor-B-spline patch (bspline_fit::
//      approximateSurface after projecting the region onto its total-least-squares
//      plane and recovering a rectangular parameter grid). Report type + params /
//      surface + achieved RMS. A region that is neither a primitive nor a stable
//      freeform patch is Declined (honest).
//   4. REPEAT on the unassigned remainder until every point is assigned or declined.
//
// HONESTY (hard invariant): NO tolerance is ever widened to force a segmentation. A
// region that genuinely fits nothing within `tol` is reported Declined with its
// inliers, not shoe-horned into the nearest primitive. Every region carries its TRUE
// achieved RMS (never clamped to claim exactness on noisy data). detectPrimitive's own
// simplicity-ordered discrimination (plane < sphere < cylinder < cone) is reused, so a
// sphere region is not mis-typed as a plane.
//
// GUARD — the fitters go through the numsci facade (primitive_fit → lstsq /
// least_squares, approximateSurface → lstsq), so the whole module is compiled only
// under CYBERCAD_HAS_NUMSCI, exactly like primitive_fit.h / bspline_fit.h. Declarations
// stay visible for documentation; with the guard OFF the implementation TU is inert and
// the functions are absent.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_REVERSE_ENGINEER_H
#define CYBERCAD_NATIVE_MATH_REVERSE_ENGINEER_H

#include "bspline_fit.h"     // SurfaceFitResult / approximateSurface (freeform patch)
#include "bspline_ops.h"     // BsplineSurfaceData
#include "primitive_fit.h"   // PrimitiveType / *Fit / detectPrimitive
#include "vec.h"

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// A segmented region: the point indices it owns, its classified type, and — for the
// matching type — the fitted parameters or surface. Every region carries its achieved
// RMS (the TRUE point-to-fit residual), never widened.
// ─────────────────────────────────────────────────────────────────────────────

/// The classified kind of a segmented region.
///   Plane / Sphere / Cylinder / Cone — an analytic primitive fit within tol.
///   Freeform                          — a fitted tensor-B-spline patch (RMS ≤ tol).
///   Declined                          — HONEST-DECLINE: no primitive within tol AND no
///                                       stable freeform fit; inliers reported, never
///                                       forced to a wrong primitive.
enum class RegionKind { Declined, Plane, Sphere, Cylinder, Cone, Freeform };

/// One segmented region of the input cloud.
struct SegmentRegion {
  RegionKind kind = RegionKind::Declined;
  std::vector<int> inliers;   ///< indices into the input `points` owned by this region

  // Exactly ONE of the following is meaningful, selected by `kind`:
  PlaneFit plane{};           ///< kind == Plane
  SphereFit sphere{};         ///< kind == Sphere
  CylinderFit cylinder{};     ///< kind == Cylinder
  ConeFit cone{};             ///< kind == Cone
  BsplineSurfaceData surface{};  ///< kind == Freeform (the fitted patch)

  double rms = 0.0;           ///< achieved RMS of the region's fit (0 for Declined)
};

/// Result of segmentAndFit — the partition of the cloud into typed regions. Every
/// input index appears in exactly one region's `inliers` (a Declined region collects
/// the honestly-unassignable points). `regions` is deterministic (seed order = ascending
/// index), so the output is reproducible.
///
/// The ROBUST path (segmentAndFitRobust) additionally populates `outliers` /
/// `outlierCount` (points consistent with no region within the robust tolerance —
/// isolated, NEVER force-assigned) and `noiseSigma` (the estimated scan noise level used
/// to scale the residual band). The noise-free path (segmentAndFit) leaves these at their
/// defaults (empty / 0), so its result is byte-compatible.
struct SegmentationResult {
  std::vector<SegmentRegion> regions;
  int assignedCount = 0;   ///< points in a NON-Declined region
  int declinedCount = 0;   ///< points in a Declined region (honestly unassigned)

  // ── Robust-path-only fields (defaulted; unused by the noise-free segmentAndFit) ──
  std::vector<int> outliers;   ///< indices consistent with NO region (gross outliers), isolated
  int outlierCount = 0;        ///< == outliers.size(); points that fit no region robustly
  double noiseSigma = 0.0;     ///< estimated scan noise σ (0 when not estimated / noise-free)
};

// ─────────────────────────────────────────────────────────────────────────────
// Parameters controlling the region-growing (sensible closed-form defaults; the
// tolerance is the ONLY quality knob that matters for the oracles).
// ─────────────────────────────────────────────────────────────────────────────

/// Tunables for segmentAndFit. Defaults are chosen so the airtight oracles (a KNOWN
/// composite, a pure primitive, a bicubic bump) segment correctly without any per-case
/// widening. `tol` is an ABSOLUTE residual bound in model units.
struct SegmentParams {
  double tol = 1e-6;    ///< absolute residual frontier for region membership AND fit acceptance
  int kNeighbors = 12;  ///< k for the k-nearest-neighbour adjacency (region-growth graph)
  int minRegion = 6;    ///< smallest region that may be classified (else Declined) — matches
                        ///< the largest primitive minimum (cylinder/cone need ≥ 6 points)
  int freeformCtrl = 4; ///< control points per direction for a freeform patch fit (degree 3)
};

// ─────────────────────────────────────────────────────────────────────────────
// Robust (noisy / outlier-laden scan) tunables. The noise-free segmentAndFit grows on
// EXACT residuals; a real scan has Gaussian noise σ and gross outliers, so region
// membership on an exact `tol` frontier either over-segments (noise splits a face) or
// swallows outliers. segmentAndFitRobust addresses that with:
//   • a noise-RELATIVE residual band  band = max(tol, bandSigma·σ)  where σ is ESTIMATED
//     from the data (robust MAD of local residuals), so the frontier scales with noise;
//   • an outlier-REJECTING primitive fit inside region growing (RANSAC-style consensus
//     over deterministically-seeded minimal subsets + a Huber/IRLS refine on the
//     consensus inliers), so a few gross outliers never corrupt the plane/sphere/cyl/cone;
//   • an explicit OUTLIER set — points consistent with NO region within `band` are
//     isolated, never force-assigned to the nearest region.
// The reported RMS is always the TRUE robust inlier RMS (≈ σ on clean-inlier data); it is
// NEVER clamped to claim exactness on noisy input.
// ─────────────────────────────────────────────────────────────────────────────

/// Tunables for segmentAndFitRobust. `base` carries the shared knobs (kNeighbors,
/// minRegion, freeformCtrl and the FLOOR tolerance `base.tol`); the robust knobs below
/// govern noise handling. Defaults segment the airtight noisy oracles (a plane/sphere/cyl
/// cloud + Gaussian σ + 5% gross outliers) correctly without per-case widening.
struct RobustSegmentParams {
  SegmentParams base{};    ///< shared region-growing knobs; base.tol is the ABSOLUTE floor band
  double sigma = 0.0;      ///< known noise σ; when ≤0 it is ESTIMATED from the data (MAD)
  double bandSigma = 3.0;  ///< region membership band = max(base.tol, bandSigma·σ) (3σ inliers)
  double outlierSigma = 4.0;  ///< a point beyond outlierSigma·σ of EVERY region → OUTLIER set
  int ransacIters = 64;    ///< RANSAC consensus trials per candidate fit (deterministic RNG)
  int huberIters = 6;      ///< IRLS/Huber refine passes after consensus (Huber k = bandSigma·σ)
  double minInlierFrac = 0.6;  ///< a robust primitive must explain ≥ this fraction of its region
  int robustMinRegion = 12;    ///< floor on a robust region's size: a higher-DOF primitive
                               ///< (cone/cylinder, 6-7 DOF) fits ANY 6 points EXACTLY, so a
                               ///< handful of gross outliers would otherwise fabricate a
                               ///< near-zero-RMS spurious primitive. Requiring well more
                               ///< points than the DOF makes an exact fit of random outliers
                               ///< statistically implausible → they stay OUTLIERS, not a region.
};

// ─────────────────────────────────────────────────────────────────────────────
// Entry point.
// ─────────────────────────────────────────────────────────────────────────────

/// Segment `points` into typed regions and fit each. `normals` is OPTIONAL (pass an
/// empty span when unavailable) — when present it seeds a better cylinder/cone axis via
/// the underlying fitters, but the pipeline never REQUIRES normals. The segmentation is
/// deterministic and every input index lands in exactly one region.
///
/// COMPOSES primitive_fit::detectPrimitive (primitive classification, simplicity-
/// ordered so a sphere is not mis-typed as a plane) and bspline_fit::approximateSurface
/// (the freeform patch), never re-implementing a fit. HONEST-DECLINES any region that
/// fits no primitive within `params.tol` and is too sparse/degenerate for a stable
/// freeform patch — such a region is returned with kind == Declined and its inliers,
/// never forced to a wrong primitive.
SegmentationResult segmentAndFit(std::span<const Point3> points,
                                 std::span<const Vec3> normals,
                                 const SegmentParams& params = {});

/// Convenience overload without normals.
inline SegmentationResult segmentAndFit(std::span<const Point3> points,
                                        const SegmentParams& params = {}) {
  return segmentAndFit(points, std::span<const Vec3>{}, params);
}

/// ROBUST segmentation for NOISY / OUTLIER-LADEN scans. Same partition semantics as
/// segmentAndFit — every region carries its type + params + TRUE achieved RMS — but:
///   • each candidate primitive is fitted with an OUTLIER-REJECTING estimator (RANSAC
///     consensus over deterministically-seeded minimal subsets, then a Huber/IRLS refine
///     on the consensus inliers), so a few gross outliers never corrupt the fit;
///   • region membership uses a NOISE-RELATIVE band (base.tol floored, scaled by the
///     estimated σ), so noise does not over/under-segment a face;
///   • points consistent with NO region within the robust band become an explicit OUTLIER
///     set (`result.outliers`), isolated rather than force-assigned;
///   • the reported RMS is the honest robust inlier RMS (≈ σ), NEVER claimed exact.
/// On a CLEAN cloud (no injected noise, σ estimated ≈ 0) the band collapses to base.tol
/// and this REPRODUCES segmentAndFit (same region count + types, params ≤ 1e-6). When no
/// stable primitive survives the noise, the region HONEST-DECLINES to Freeform or Declined
/// — never a fabricated primitive. Deterministic (seeded RNG by index; no global state).
SegmentationResult segmentAndFitRobust(std::span<const Point3> points,
                                       std::span<const Vec3> normals,
                                       const RobustSegmentParams& params = {});

/// Convenience overload without normals.
inline SegmentationResult segmentAndFitRobust(std::span<const Point3> points,
                                              const RobustSegmentParams& params = {}) {
  return segmentAndFitRobust(points, std::span<const Vec3>{}, params);
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_REVERSE_ENGINEER_H

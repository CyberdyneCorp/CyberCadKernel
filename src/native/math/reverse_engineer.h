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
struct SegmentationResult {
  std::vector<SegmentRegion> regions;
  int assignedCount = 0;   ///< points in a NON-Declined region
  int declinedCount = 0;   ///< points in a Declined region (honestly unassigned)
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

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_REVERSE_ENGINEER_H

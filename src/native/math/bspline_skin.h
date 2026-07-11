// SPDX-License-Identifier: Apache-2.0
//
// bspline_skin.h — NURBS roadmap Layer 6: skinning / lofting (section curves →
// tensor-product surface).
//
// Given K B-spline SECTION curves, construct a single tensor-product B-spline
// SURFACE that CONTAINS every section as an iso-parametric curve — a skinned (a.k.a.
// lofted) surface. This is the "cross-sections → smooth skin" direction of freeform
// surfacing (docs/NURBS-SCOPE.md §2/§4 Layer 6): the sections define the shape of
// one parameter direction (U), and the surface interpolates smoothly across them in
// the other (V).
//
// It sits above the evaluators in bspline.{h,cpp} and COMPOSES two lower layers:
//   * Layer 1 (bspline_ops.h) — elevateDegreeCurve / refineKnotCurve make the input
//     sections COMPATIBLE (same degree, same knot vector, same control-point count)
//     while preserving each section's geometry EXACTLY.
//   * Layer 7 (bspline_fit.h) — the curve-interpolation machinery (collocation +
//     numerics::lin_solve) interpolates a B-spline across the sections in V.
// The output is a Layer-1 BsplineSurfaceData (flat knots, row-major U-outer poles),
// so a skinned surface drops straight into the rest of the NURBS stack.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 10:
//   §10.3 skinning (Algorithm A10.3) — make sections compatible, assign section
//   parameters v_k (chord-length across the section control polygons), then for each
//   control-point index i interpolate a degree-q B-spline in V through the K control
//   points {P_i^k}. The interpolated curves' control points form the surface net.
//
// SCOPE — NON-RATIONAL sections only (all weights = 1). Rational (weighted) skinning
// (interpolating the weights too), and general Gordon/network surfacing and exact
// swept surfaces, are documented residuals for later slices — this module never fakes
// them. See docs/NURBS-SCOPE.md Layer-6 row.
//
// GUARD — the solve-bearing routine is compiled only when CYBERCAD_HAS_NUMSCI is
// defined (the numsci facade is the sole linear-algebra dependency, via the Layer-7
// interpolation), exactly like src/native/math/bspline_fit.cpp. With the guard OFF
// the implementation TU is inert and the function is absent; the declaration remains
// visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_SKIN_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_SKIN_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types + ops)

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Section compatibility.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of making a set of sections compatible.
struct SectionCompatibility {
  bool ok = false;                        ///< true ⇔ compatibilization succeeded
  int degree = 0;                         ///< the common (max) degree of every section
  std::vector<double> knots;              ///< the common flat knot vector shared by all
  std::vector<BsplineCurveData> sections; ///< the compatible sections (same degree/knots/N)
};

/// Make `sections` COMPATIBLE (*The NURBS Book* §10.3): raise every section to the
/// common maximum degree with `elevateDegreeCurve`, then merge every section onto the
/// UNION of all sections' knot vectors with `refineKnotCurve`. After this every
/// returned section has the SAME degree, the SAME flat knot vector, and therefore the
/// SAME control-point count `N = knots.size() − degree − 1`. Both ops are exact
/// (Layer-1), so each compatible section still represents its ORIGINAL curve exactly.
///
/// Requires the sections to share a common parameter domain `[a,b]` (the standard
/// pre-condition: reparametrize first if they do not). NON-RATIONAL sections only —
/// a rational section (non-empty weights) makes the whole call decline (`ok=false`),
/// an honest guard, never a silently-wrong result. Fewer than one section, an empty
/// section, or a degenerate (degree < 1) section also declines.
SectionCompatibility makeSectionsCompatible(std::span<const BsplineCurveData> sections);

// ─────────────────────────────────────────────────────────────────────────────
// Skinning / lofting.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a skinning operation.
struct SkinResult {
  bool ok = false;             ///< true ⇔ the skin succeeded
  BsplineSurfaceData surface;  ///< the skinned non-rational tensor-product surface
  std::vector<double> vParams; ///< the section parameters v_k the surface interpolates at
};

/// A10.3 — SKIN (loft) a tensor-product B-spline surface through `sections`. The
/// U direction carries the (compatibilized) section shape (degree u = the common
/// section degree, U-knots = the common section knots); the V direction interpolates
/// smoothly ACROSS the sections with degree `degreeV`.
///
/// Steps:
///   1. `makeSectionsCompatible(sections)` → every section shares degree p, knots, and
///      control-point count N (each section still equals its original exactly).
///   2. Assign section parameters `v_k ∈ [0,1]` by chord length across the sections'
///      control polygons (averaged over the N control-point indices), and build a
///      common averaging V-knot vector of degree `degreeV`.
///   3. For each control-point index `i ∈ [0,N)`, interpolate a degree-`degreeV`
///      B-spline curve in V through the K points `{P_i^k}` at parameters `v_k` on the
///      common V-knots (a single collocation matrix, one solve per coordinate via the
///      Layer-7 `numerics::lin_solve` path). The K→(control points of that V-curve)
///      form column `i` of the surface net.
///   4. The interpolated V-curves' control points assemble the (nV × N) surface net
///      (V outer via the interpolation, U inner via the section index) — re-laid to the
///      kernel's row-major U-outer BsplineSurfaceData convention.
///
/// GUARANTEE (the core oracle): the surface's iso-curve at `v = v_k` is EXACTLY the
/// compatibilized section `k` (the surface CONTAINS every input section). `degreeV` is
/// clamped to `K−1` (a straight/quadratic/... loft when few sections). Requires ≥ 2
/// sections. Non-rational only. Declines (`ok=false`) on rational input, fewer than two
/// sections, or an all-coincident set of sections (no V length to normalize) — honest
/// guards, never a crash.
SkinResult skinSurface(std::span<const BsplineCurveData> sections, int degreeV = 3);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_SKIN_H

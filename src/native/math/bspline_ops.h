// SPDX-License-Identifier: Apache-2.0
//
// bspline_ops.h — exact-NURBS geometry kernel: knot insertion / refinement /
// removal, degree elevation / reduction, splitting, Bézier decomposition and
// reparametrization for B-spline and rational NURBS curves and tensor-product
// surfaces.
//
// This is NURBS roadmap Layer 1 (docs/NURBS-SCOPE.md §2). It is the construction
// layer that sits above the evaluators in bspline.{h,cpp}: every operation here
// transforms one NURBS representation into an equivalent one while PRESERVING the
// represented geometry. Knot removal and degree reduction are the only non-exact
// ops — they carry a *reported* error bound and decline honestly when the input
// is not reducible within tolerance.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 5:
//   insertKnotCurve         — A5.1 (Boehm knot insertion, r-fold)
//   refineKnotCurve         — A5.4 (whole-vector Oslo-class refinement)
//   removeKnotCurve         — A5.8 (tolerance-bounded knot removal)
//   elevateDegreeCurve      — A5.9 (degree elevation by t)
//   reduceDegreeCurve       — A5.11 (degree reduction by 1)
//   decomposeCurveToBezier  — A5.6 (Bézier decomposition)
//   splitCurve              — full-multiplicity insertion + partition
//   reparamCurve            — affine knot remap
// Surfaces reuse the curve core per row (V-dir) / column (U-dir) so the curve,
// surface and rational paths never diverge.
//
// Conventions match the rest of the kernel and OCCT exactly: FLAT knot vectors
// with expanded multiplicities (length = nPoles + degree + 1); surface poles are
// row-major, U outer (pole(i,j) = poles[i*nPolesV + j]); weights are stored
// separately, one per pole, and an EMPTY weight vector means non-rational.
//
// Rational handling is uniform: a pole P=(x,y,z) with weight w is lifted to the
// homogeneous point Pʷ=(w·x, w·y, w·z, w) in R⁴, the non-rational algorithm runs
// on the R⁴ net (knots/degree unchanged by the lift), and the result is projected
// back P'=(x'/w', y'/w', z'/w'). Non-positive projected weights are a documented
// guard (never divide by ≤ 0).
//
// OCCT-FREE, NumPP/SciPP-FREE, always-on. fp64, deterministic. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_OPS_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_OPS_H

#include "vec.h"

#include <span>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Data types (mirror the free-form fields of EdgeCurve / FaceSurface in shape.h).
// ─────────────────────────────────────────────────────────────────────────────

/// A B-spline / NURBS curve: flat knot vector of length poles.size()+degree+1.
struct BsplineCurveData {
  int degree = 0;
  std::vector<Point3> poles;
  std::vector<double> weights;  ///< empty ⇒ non-rational
  std::vector<double> knots;    ///< flat, length = poles.size() + degree + 1
};

/// A tensor-product B-spline / NURBS surface. Poles are row-major, U outer:
/// pole(i,j) = poles[i*nPolesV + j], i over U (0..nPolesU-1), j over V.
struct BsplineSurfaceData {
  int degreeU = 0, degreeV = 0;
  int nPolesU = 0, nPolesV = 0;
  std::vector<Point3> poles;    ///< row-major, U outer
  std::vector<double> weights;  ///< empty ⇒ non-rational
  std::vector<double> knotsU, knotsV;
};

/// Surface direction selector.
enum class ParamDir { U, V };

// ─────────────────────────────────────────────────────────────────────────────
// Result structs for the non-exact / reporting ops.
// ─────────────────────────────────────────────────────────────────────────────

struct KnotRemovalResult {
  int removed = 0;         ///< how many of `num` requested removals succeeded
  double maxError = 0.0;   ///< max deviation introduced (0 for an exact removal)
  BsplineCurveData curve;
};

struct DegreeReduceResult {
  bool ok = false;         ///< true ⇔ reducible within tol
  double maxError = 0.0;   ///< achieved bound (the true deviation)
  BsplineCurveData curve;
};

struct CurveSplit {
  BsplineCurveData left, right;
};

struct KnotRemovalResultS {
  int removed = 0;
  double maxError = 0.0;
  BsplineSurfaceData surface;
};

struct DegreeReduceResultS {
  bool ok = false;
  double maxError = 0.0;
  BsplineSurfaceData surface;
};

struct SurfaceSplit {
  BsplineSurfaceData low, high;
};

// ─────────────────────────────────────────────────────────────────────────────
// Curve operations.
// ─────────────────────────────────────────────────────────────────────────────

/// A5.1 — insert knot `u` into the curve `r` times (Boehm). Exact: the curve is
/// unchanged. `r` is clamped to degree − existingMultiplicity(u).
BsplineCurveData insertKnotCurve(const BsplineCurveData& c, double u, int r = 1);

/// A5.4 — refine by inserting an entire sorted new-knot vector at once
/// (Oslo-class). Equivalent to the corresponding sequence of single insertions.
BsplineCurveData refineKnotCurve(const BsplineCurveData& c,
                                 std::span<const double> newKnots);

/// A5.8 — remove knot value `u` up to `num` times within `tol`. Reports how many
/// removals succeeded and the maximum deviation. A removal that would exceed `tol`
/// is not performed (honest decline).
KnotRemovalResult removeKnotCurve(const BsplineCurveData& c, double u, int num,
                                  double tol);

/// A5.9 — raise the degree by `t`. Exact: the curve is unchanged; the result's
/// degree is degree+t and interior-knot multiplicities are raised by t.
BsplineCurveData elevateDegreeCurve(const BsplineCurveData& c, int t);

/// A5.11 — reduce the degree by 1. Exact when the geometry is genuinely reducible
/// (ok=true, recovered curve), otherwise ok=false with the true error bound; never
/// returns a lower-degree curve presented as an exact match when it is not.
DegreeReduceResult reduceDegreeCurve(const BsplineCurveData& c, double tol);

/// Split at parameter `u` (insert to multiplicity = degree, then partition). The
/// two pieces reconstruct `c` on their sub-domains and share the point C(u).
CurveSplit splitCurve(const BsplineCurveData& c, double u);

/// A5.6 — decompose into Bézier segments (full-multiplicity knot insertion). Each
/// segment re-evaluates to `c` on its span; count == #distinct interior spans.
std::vector<BsplineCurveData> decomposeCurveToBezier(const BsplineCurveData& c);

/// Affine reparametrization of the knot domain to [a,b]; poles/weights unchanged.
BsplineCurveData reparamCurve(const BsplineCurveData& c, double a, double b);

// ─────────────────────────────────────────────────────────────────────────────
// Surface operations (tensor-product; the curve core is applied per row/column).
// ─────────────────────────────────────────────────────────────────────────────

BsplineSurfaceData  insertKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                      double val, int r = 1);
BsplineSurfaceData  refineKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                      std::span<const double> newKnots);
KnotRemovalResultS  removeKnotSurface(const BsplineSurfaceData& s, ParamDir d,
                                      double val, int num, double tol);
BsplineSurfaceData  elevateDegreeSurface(const BsplineSurfaceData& s, ParamDir d,
                                         int t);
DegreeReduceResultS reduceDegreeSurface(const BsplineSurfaceData& s, ParamDir d,
                                        double tol);
SurfaceSplit        splitSurface(const BsplineSurfaceData& s, ParamDir d,
                                 double val);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_OPS_H

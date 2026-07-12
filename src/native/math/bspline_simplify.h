// SPDX-License-Identifier: Apache-2.0
//
// bspline_simplify.h — tolerance-BOUNDED NURBS simplification: remove as many
// interior knots (and reduce degree) as possible while the represented geometry
// stays within a user deviation tolerance. This is the operation that turns an
// over-refined / heavy NURBS into a light one, the CAD complement to the EXACT
// (geometry-preserving) knot-removal / degree-reduction in bspline_ops.{h,cpp}.
//
// This is NURBS roadmap Layer 1 (docs/NURBS-SCOPE.md §2), additive to bspline_ops.
// Where bspline_ops::removeKnotCurve / reduceDegreeCurve remove ONE value / drop
// ONE degree exactly (decline the moment geometry would change), the routines here
// SWEEP the whole curve, greedily discarding knots in increasing-error order and
// reducing degree, accepting each step only while the TRUE deviation stays ≤ tol.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.):
//   removeKnotsBounded        — Chapter 9.3 (bounded knot removal), driving the
//                               A5.8 removal core + the A5.8/A9.x removal-error
//                               bound in R⁴ and a dense sampled deviation check.
//   reduceDegreeBounded       — Chapter 9 bounded reduction, driving A5.11.
//   removeKnotsBoundedSurface — per-direction sweep (row/column reuse).
//
// The deviation bound is HARD: no accepted result ever exceeds `tol`. Every step
// is verified by densely sampling both the candidate and the ORIGINAL curve and
// taking the max pointwise distance — the reported `maxDeviation` is that measured
// worst case, not merely the local removal estimate. A tolerance too tight to
// remove anything returns the input unchanged (honest no-op).
//
// Rational handling is uniform with bspline_ops: the removal core runs on the
// homogeneous R⁴ net (weighted poles wᵢ·Pᵢ, wᵢ), and the verification samples the
// projected rational curve, so a rational input is bounded in true (projected)
// geometry, not merely in homogeneous space.
//
// OCCT-FREE, NumPP/SciPP-FREE, always-on. fp64, deterministic. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_SIMPLIFY_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_SIMPLIFY_H

#include "bspline_ops.h"

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Result structs. `maxDeviation` is the MEASURED true deviation (dense pointwise
// sample of candidate vs. original), always ≤ tol for the returned curve.
// ─────────────────────────────────────────────────────────────────────────────

struct BoundedRemovalResult {
  int removed = 0;             ///< total interior knots removed (with multiplicity)
  double maxDeviation = 0.0;   ///< true max deviation of result from the input
  BsplineCurveData curve;      ///< the simplified curve (== input when removed==0)
};

struct BoundedReduceResult {
  int degreeDrop = 0;          ///< how many degrees were shed (0 ⇒ unchanged)
  double maxDeviation = 0.0;   ///< true max deviation of result from the input
  BsplineCurveData curve;      ///< reduced curve (== input when degreeDrop==0)
};

struct BoundedRemovalResultS {
  int removedU = 0, removedV = 0;
  double maxDeviation = 0.0;
  BsplineSurfaceData surface;
};

// ─────────────────────────────────────────────────────────────────────────────
// Curve simplification.
// ─────────────────────────────────────────────────────────────────────────────

/// Greedy bounded knot removal (P&T Ch.9.3). Repeatedly removes the interior knot
/// whose removal introduces the least error, accepting the removal only while the
/// TRUE max deviation of the resulting curve from the original stays ≤ `tol`.
///
/// * Over-refined input (a curve knot-INSERTED many times) with a tight `tol`
///   recovers its minimal knot vector exactly (deviation ≈ machine epsilon).
/// * A genuinely wiggly curve with a loose `tol` sheds knots while the reported
///   deviation never exceeds `tol` (measured densely).
/// * `tol` too tight to remove anything ⇒ the input is returned unchanged.
BoundedRemovalResult removeKnotsBounded(const BsplineCurveData& c, double tol);

/// Bounded degree reduction (P&T Ch.9). Repeatedly drops the degree by 1 (A5.11)
/// while the TRUE deviation of the reduced curve from the original stays ≤ `tol`.
///
/// * A degree-ELEVATED curve reduces back to its original degree exactly.
/// * A curve that cannot be reduced within `tol` keeps its degree (honest).
BoundedReduceResult reduceDegreeBounded(const BsplineCurveData& c, double tol);

// ─────────────────────────────────────────────────────────────────────────────
// Surface simplification (per-direction sweep; the curve core runs on each line).
// ─────────────────────────────────────────────────────────────────────────────

/// Per-direction bounded knot removal: sweep U then V, removing interior knots
/// while the true surface deviation from the input stays ≤ `tol` (measured on a
/// dense (u,v) grid). Returns per-direction removal counts and the true deviation.
BoundedRemovalResultS removeKnotsBoundedSurface(const BsplineSurfaceData& s,
                                                double tol);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_SIMPLIFY_H

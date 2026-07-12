// SPDX-License-Identifier: Apache-2.0
//
// bspline_join.h — NURBS roadmap Layer 6: surface G1/G2 CONTINUITY JOIN across a shared
// edge. Given two ALREADY-BUILT adjacent tensor-product NURBS patches A and B that share a
// boundary edge (a common boundary curve, C0), reposition the near-boundary control rows so
// the two patches meet TANGENT-PLANE-continuously (G1) or CURVATURE-continuously (G2) across
// that edge — with MINIMAL control-point movement, and leaving the shared boundary curve
// itself unchanged.
//
// This is NOT a surface builder (skin / sweep / Coons / N-sided all CONSTRUCT a surface). It
// is a POST-hoc continuity operator: two independent patches already exist, they touch along
// an edge, and the caller wants them to blend smoothly there without rebuilding either. The
// only degrees of freedom used are the SECOND control row of B (and, symmetrically, of A for
// the minimal split) one step in from the shared edge; the boundary row is frozen so C0 and
// the boundary curve are preserved exactly.
//
// EDGE SHARING & ORIENTATION ─────────────────────────────────────────────────────────────
// A tensor-product surface has four boundaries: U0 (u=umin), U1 (u=umax), V0 (v=vmin), V1
// (v=vmax). The `EdgeSpec` names, for each of A and B, WHICH boundary is the shared edge and
// whether the two boundaries run in the SAME or REVERSED along-edge parameter direction. The
// join needs, along the shared edge, COMPATIBLE degree and knots in the along-edge direction
// (so the two patches share the same boundary control-point count and basis). `makeCompatible
// AlongEdge` establishes that by knot-merging + degree-matching (reusing Layer-1 bspline_ops).
//
// The join views each surface as a stack of control ROWS parallel to the shared edge:
//   row 0  — the boundary itself (shared, frozen),
//   row 1  — one step in (the CROSS-boundary first-derivative control → drives G1),
//   row 2  — two steps in (the CROSS-boundary second-derivative control → drives G2).
//
// G1 CONDITION (Piegl & Tiller ch.10; cross-boundary tangent continuity) ────────────────────
// Two patches are G1 across the shared edge iff at every along-edge station the two cross-
// boundary tangent vectors are COPLANAR with the common boundary tangent AND point to
// opposite sides. The minimal-movement enforcement used here makes the cross-boundary tangent
// RIBBON shared: B's row-1 offset from the boundary is the NEGATIVE, scaled by a single global
// proportionality s>0, of A's row-1 offset:   (B1[i]-P0[i]) = -s·(A1[i]-P0[i]).
// Then ∂S/∂n is antiparallel-collinear across the edge at every station ⇒ the two tangent
// planes coincide ⇒ the UNIT NORMAL is continuous (G1). The scalar s is chosen by closed-form
// least squares to MINIMIZE the total row-1 movement (minimal-norm adjustment). The boundary
// row P0 is never touched, so the boundary curve is preserved to machine precision.
//
// G2 CONDITION ──────────────────────────────────────────────────────────────────────────────
// G2 additionally requires the cross-boundary SECOND derivative to be continuous (matching
// normal curvature). Row 2 is repositioned so the second-order cross ribbon is the reflection
// of A's:  (B2[i]-P0[i]) = s²·(A2[i]-2·A1[i]+P0[i]) - 2·s·(1-s)·(A1[i]-P0[i]) ... (the exact
// clamped-basis reflection derived in the .cpp). This matches ∂²S/∂n² across the edge while
// keeping G1 (row 1 unchanged from joinG1) and C0 (row 0 frozen). Minimal movement holds by
// the same s that minimized the G1 row-1 move (row 2 then follows in closed form).
//
// ORACLES (airtight, closed-form) ──────────────────────────────────────────────────────────
//   1. COPLANAR NO-OP — two co-planar patches already meeting G-infinity → the join is a
//      no-op (maxMovement == 0, residual already below tol; nothing to move).
//   2. G1 ENFORCED — two patches meeting only C0 (a crease) → after joinG1 the UNIT NORMAL is
//      continuous across the shared edge at sampled stations (≤ 1e-7 rad) and the boundary
//      curve is unchanged (≤ 1e-12).
//   3. G2 ENFORCED — after joinG2 the NORMAL CURVATURE is continuous across the edge
//      (relative ≤ 1e-5) AND G1 still holds; boundary unchanged.
//   4. ANALYTIC (CYLINDER) — two halves of a cylinder split along a knot line already meet G2
//      → join is a no-op (movement 0). A demand that needs movement beyond the caller cap is
//      an HONEST DECLINE (would distort the surface), never a silently-widened tolerance.
//
// HONEST DECLINES — incompatible-along-edge degree/knots that cannot be reconciled, a non-
// coincident (not actually C0) shared edge, a rational-mismatch, a degenerate boundary, or a
// required movement beyond the caller `maxMovementCap` all return ok=false with a reason —
// never a wrong surface, never a crash, never a widened tolerance.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic. Additive: no existing API changes.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_JOIN_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_JOIN_H

#include "bspline_ops.h"  // BsplineSurfaceData / Layer-1 knot-merge + degree-elevate

#include <string>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Edge specification.
// ─────────────────────────────────────────────────────────────────────────────

/// Which boundary of a surface is the shared edge.
///   U0 = u-min row (i=0),      U1 = u-max row (i=nPolesU-1)  — along-edge param is V,
///   V0 = v-min column (j=0),   V1 = v-max column (j=nPolesV-1) — along-edge param is U.
enum class SurfaceEdge { U0, U1, V0, V1 };

/// Names the shared boundary on each patch and whether their along-edge parameters run the
/// same way. If `reversed` is true, B's along-edge index runs opposite to A's (B[i] pairs
/// with A[n-1-i]) — the usual case when two patches are laid edge-to-edge.
struct EdgeSpec {
  SurfaceEdge edgeA = SurfaceEdge::V1;  ///< A's boundary that is shared
  SurfaceEdge edgeB = SurfaceEdge::V0;  ///< B's boundary that is shared
  bool reversed = false;                ///< B's along-edge param runs opposite to A's
};

// ─────────────────────────────────────────────────────────────────────────────
// Results.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of `makeCompatibleAlongEdge`: A and B refined to a COMMON along-edge degree + knot
/// vector (geometry of each surface unchanged — knot-merge + degree-elevate are exact).
struct EdgeCompatResult {
  bool ok = false;
  BsplineSurfaceData A, B;
  int sharedDegree = 0;            ///< common along-edge degree after matching
  int sharedPoles = 0;            ///< common along-edge pole count after merge
  std::string reason;             ///< decline reason when !ok
};

/// Result of a G1 / G2 join. On success A and B carry the repositioned near-boundary rows.
struct JoinResult {
  bool ok = false;
  BsplineSurfaceData A, B;          ///< the joined patches (boundary row frozen)
  double continuityResidual = 0.0;  ///< achieved residual: G1 → max unit-normal mismatch (rad);
                                    ///< G2 → max relative normal-curvature mismatch. 0 ⇒ perfect.
  double maxMovement = 0.0;         ///< largest control-point displacement applied
  double boundaryDev = 0.0;         ///< shared-boundary displacement (must stay ~0)
  bool noop = false;                ///< true ⇔ already continuous (nothing moved)
  std::string reason;               ///< decline reason when !ok
};

// ─────────────────────────────────────────────────────────────────────────────
// API.
// ─────────────────────────────────────────────────────────────────────────────

/// Knot-merge + degree-match A and B along their shared edge so they share the same along-edge
/// degree and knot vector (a precondition for the row-wise join). Exact — the represented
/// geometry of neither surface changes (reuses Layer-1 elevateDegreeSurface / refineKnotSurface).
/// Declines if the two boundaries are not actually coincident within `tol`, if a patch is
/// malformed, or if the rationality differs. `tol` bounds the along-edge coincidence check.
EdgeCompatResult makeCompatibleAlongEdge(const BsplineSurfaceData& A,
                                         const BsplineSurfaceData& B,
                                         const EdgeSpec& edge, double tol = 1e-9);

/// Enforce G1 across the shared edge with MINIMAL row-1 movement: reposition B's second control
/// row (and, when `adjustBoth`, split the move symmetrically onto A's second row) so the cross-
/// boundary tangent ribbons are collinear ⇒ unit-normal continuous. The boundary row is frozen
/// (C0 + boundary curve preserved). If the required max movement exceeds `maxMovementCap`, this
/// HONEST-DECLINES (would distort the surface). Already-G1 input ⇒ no-op (movement 0). `tol` is
/// the residual acceptance / coincidence tolerance.
JoinResult joinG1(const BsplineSurfaceData& A, const BsplineSurfaceData& B, const EdgeSpec& edge,
                  double maxMovementCap = 1e30, bool adjustBoth = false, double tol = 1e-9);

/// Enforce G2 across the shared edge: first joinG1 (row 1), then reposition B's THIRD control
/// row so the cross-boundary second derivative matches (normal curvature continuous), still G1,
/// still C0. HONEST-DECLINES if the required movement exceeds `maxMovementCap` or if the patch
/// lacks a third row (along-edge degree < 2 cannot carry G2). Already-G2 ⇒ no-op. `tol` as above.
JoinResult joinG2(const BsplineSurfaceData& A, const BsplineSurfaceData& B, const EdgeSpec& edge,
                  double maxMovementCap = 1e30, bool adjustBoth = false, double tol = 1e-9);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_JOIN_H

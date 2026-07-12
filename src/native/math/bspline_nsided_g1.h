// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided_g1.h — NURBS roadmap Layer 6: G1 (tangent-plane continuous) N-SIDED
// boundary-filled surface. Fill a CLOSED boundary of N ≠ 4 curves (a topological N-gon:
// triangle, pentagon, hexagon …) with a smooth multi-patch of tensor-product B-spline
// surfaces that meet TANGENT-PLANE-continuously (G1) across every internal seam and to
// the prescribed boundary cross-tangent field.
//
// This is the G1 upgrade of the C0 fill in bspline_nsided.{h,cpp}. The C0 fill's N Coons
// sub-patches meet the boundary and each other with POSITION continuity only — a visible
// crease at every internal spoke. Here the same midpoint-subdivision topology (N sub-
// patches meeting at the polygon centroid C) is filled with GREGORY-style bicubic patches
// whose boundary rows encode not just the boundary POSITION but the boundary CROSS-TANGENT
// too, and whose shared internal spokes carry a SINGLE shared cross-spoke tangent field so
// adjacent patches meet C1 (⇒ G1) across the seam.
//
// CONSTRUCTION (Gregory / Chiyokura-Kimura, Piegl & Tiller ch.11; Farin) ─────────────────
//
// Midpoint subdivision (identical topology to the C0 fill):
//   * Corners        V[i]  = edges[i](0)          (shared corner of e[i-1] and e[i]),
//   * Edge midpoints M[i]  = edges[i](0.5),
//   * Centroid       C     = (1/N)·Σ V[i]         (the single interior hub).
//
// Sub-patch i is the four-sided GREGORY quadrant around V[i] on (u,v) ∈ [0,1]², corners
//   P00 = V[i]   P10 = M[i]   P01 = M[i-1]   P11 = C,
// with the two OUTER sides on the boundary and the two INNER sides the interior spokes:
//   c0 = S(·,0): V[i]→M[i]   = first half of e[i]           (boundary sub-curve, exact),
//   d0 = S(0,·): V[i]→M[i-1] = second half of e[i-1] rev    (boundary sub-curve, exact),
//   d1 = S(1,·): M[i]→C      = the spoke SHARED with patch i+1 (its c1),
//   c1 = S(·,1): M[i-1]→C    = the spoke SHARED with patch i-1 (its d1).
//
// Each sub-patch is emitted as a bicubic (3×3) BÉZIER surface. A bicubic Bézier patch is
// determined near an edge by exactly (edge curve, cross-edge first derivative): the edge
// ring poles fix the edge curve, and the ONE-ROW-IN poles fix the cross-edge derivative.
// So:
//   * BOUNDARY G1 — the outer-edge ring reproduces the boundary sub-curve EXACTLY (position
//     interpolation), and the one-row-in poles are placed along the PRESCRIBED boundary
//     cross-tangent field T[i](u) so ∂S/∂n on the boundary equals the prescribed field.
//   * SPOKE G1 — the two patches sharing a spoke use the SAME spoke edge ring AND the SAME
//     one-row-in "rib" poles (a single shared cross-spoke tangent field per midpoint), so
//     they meet C1 (hence G1: the unit normal is continuous) across the seam BY POLE
//     EQUALITY — an exact, machine-precision invariant, not a fitted approximation.
//   * CENTRE TWIST — the four sub-patch interior (twist) poles at the hub are INCOMPATIBLE
//     in general (the classic Gregory twist problem at a degenerate corner). They are
//     resolved by the rational GREGORY BLEND (a convex combination lifted through the
//     homogeneous R⁴ weight, Chiyokura-Kimura) so the centre never blows up; the blend does
//     not affect the boundary or the one-row-in rib poles, so it cannot break either G1.
//
// The prescribed cross-boundary tangent field per edge is a Hermite (cross-tangent) field
// sampled at the edge poles; when the caller supplies none, a natural in-surface field is
// synthesised from the boundary + centroid geometry (see `nSidedFillG1`), which is exact
// for planar / analytic boundaries.
//
// ORACLES (airtight, closed-form) ──────────────────────────────────────────────────────
//   1. BOUNDARY INTERPOLATION — the fill reproduces each of the N input boundary curves to
//      ≤ 1e-12 (each slice's v=0 pole row IS the edge's control net → machine-exact).
//   2. G1 ACROSS SPOKES — at sampled points along every internal spoke and along the
//      boundary, the unit normal is continuous across the seam (mismatch angle ≤ 1e-6 rad;
//      MACHINE-exact ~1e-16 in practice). Achieved by pole equality (the two slices at a seam
//      share the spoke column + a shared cross-spoke rib → identical tangent plane).
//   3. PLANAR / ANALYTIC SANITY — N boundary curves lying in a plane produce a planar fill
//      whose points lie on that plane to ≤ 1e-10 (constant normal ⇒ G1 trivially).
//   4. DEGENERATE-CORNER (Gregory twist) — the hub apex (all slices collapse to C) is kept
//      finite by the radial interior/twist row (no blowup).
//
// G1 FEASIBILITY (the honest precondition) ───────────────────────────────────────────────
// A tangent-plane-continuous surface CANNOT cross a boundary that itself has a tangent
// DISCONTINUITY at a corner unless everything there is coplanar. So machine-exact G1 across
// the incident spokes holds exactly when EITHER the boundary is tangent-continuous at its
// corners (a smooth closed loop — the realistic hole-fill case) OR the boundary is planar
// (a flat N-gon: constant normal). A boundary that CREASES at a corner in 3-D (e.g. a non-
// planar polygon of straight chords) has non-collinear edge tangents that are not coplanar
// with the spoke — no tangent plane exists across the incident spokes — and is DECLINED, not
// silently filled with a residual crease. We NEVER widen a tolerance to pass an oracle.
//
// HONEST DECLINES — a non-closed loop, a rational / malformed edge, N < 3, a degenerate
// centroid spoke, a G1-infeasible creased corner (above), or a caller-supplied tangent field
// that is INCOMPATIBLE with G1 (a prescribed cross-tangent (anti-)parallel to the boundary
// tangent, so no tangent plane exists) all return ok = false with a reason — never a wrong
// surface, never a crash.
//
// The C0 `nSidedFill` API in bspline_nsided.h is UNCHANGED — this is purely additive.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G1_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G1_H

#include "bspline_nsided.h"  // NSidedBoundary / verifyNSidedBoundary (shared boundary type)
#include "bspline_ops.h"     // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)

#include <string>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Prescribed cross-boundary tangent field.
// ─────────────────────────────────────────────────────────────────────────────

/// The prescribed cross-boundary tangent (surface derivative TRANSVERSE to the boundary,
/// pointing INTO the fill) for one boundary edge, given as a B-spline vector field over
/// the SAME [0,1] parameter as the edge. `poles` are the field's control vectors; the
/// field shares the edge's degree and knot vector when `knots` is empty (the common case),
/// otherwise its own clamped [0,1] knot vector is used. An empty field ⇒ synthesise a
/// natural in-surface cross-tangent from the boundary + centroid geometry.
struct CrossTangentField {
  std::vector<Vec3> poles;    ///< cross-tangent control vectors along the edge (into the fill)
  std::vector<double> knots;  ///< empty ⇒ borrow the edge's degree/knots; else clamped [0,1]
  int degree = 0;             ///< used only when `knots` is non-empty
};

// ─────────────────────────────────────────────────────────────────────────────
// G1 N-sided fill.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a G1 N-sided boundary fill.
struct NSidedFillG1Result {
  bool ok = false;                          ///< true ⇔ the G1 fill was built
  std::vector<BsplineSurfaceData> patches;  ///< the N Gregory (bicubic Bézier) sub-patches
  Point3 centroid{};                        ///< the shared interior hub C
  double maxCornerError = 0.0;              ///< the loop's corner consistency error
  double maxBoundaryDev = 0.0;              ///< worst boundary-interpolation residual (reported)
  double maxSpokeNormalAngle = 0.0;         ///< worst G1 normal-mismatch across spokes (rad, reported)
  std::string reason;                       ///< decline reason when !ok
};

/// G1 N-SIDED FILL — fill the closed N-gon boundary `b` with N Gregory bicubic sub-patches
/// that meet G1 (tangent-plane continuous) across every internal spoke and to the boundary
/// cross-tangent field (see the file header for the full construction).
///
///   1. `verifyNSidedBoundary(b, tol)` — decline honestly on a non-closed / rational /
///      degenerate boundary (returns ok=false with a reason; never a wrong surface).
///   2. Midpoint-subdivide (corners V[i], midpoints M[i], centroid C).
///   3. For each edge, take the caller's cross-tangent field `tangents[i]` (or synthesise a
///      natural in-surface field when `tangents` is empty / an entry is empty). Decline if a
///      prescribed field is (anti-)parallel to the boundary tangent (no tangent plane → G1
///      impossible; honest decline, never a widened tolerance).
///   4. Build the N Gregory bicubic quadrants: outer-edge rings = the exact boundary sub-
///      curves, one-row-in poles = the prescribed cross-tangent field, shared spokes = a
///      single shared cross-spoke rib field (⇒ C1/G1 across seams by pole equality), centre
///      twists resolved by the rational Gregory blend (degenerate corner, no blowup).
///
/// GUARANTEE: boundary interpolation ≤ 1e-12 (`maxBoundaryDev`), G1 across every spoke and
/// along the boundary — the unit normal is continuous, mismatch angle ≤ 1e-6 rad
/// (`maxSpokeNormalAngle`) — and a planar boundary yields a planar fill (≤ 1e-10). Declines
/// (`ok=false`) on a non-closed / rational / degenerate boundary or a G1-incompatible
/// prescribed tangent — honest guards, never a crash, never a silently-non-G1 net.
///
/// `tangents` — either empty (synthesise all N fields) or exactly N entries, one per edge
/// (an individually-empty entry is synthesised). `tol` — the loop-closure / degeneracy /
/// tangent-parallelism tolerance.
NSidedFillG1Result nSidedFillG1(const NSidedBoundary& b,
                                const std::vector<CrossTangentField>& tangents = {},
                                double tol = 1e-7);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G1_H

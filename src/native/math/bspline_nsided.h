// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided.h — NURBS roadmap Layer 6: N-SIDED boundary-filled surface. Fill a
// CLOSED boundary of N ≠ 4 curves (a topological N-gon: triangle, pentagon, hexagon …)
// with a smooth multi-patch of tensor-product B-spline surfaces.
//
// A single bilinearly-blended Coons patch (bspline_coons.h) fills exactly FOUR
// boundaries — a topological quad. For N ≠ 4 there is no single natural tensor-product
// quad, so we SUBDIVIDE the N-gon into N quads by the classic Catmull-Clark-style
// MIDPOINT subdivision and fill each quad with a Coons patch:
//
//   * Corners           V[i]  = the shared corner between boundary edge e[i-1] and e[i]
//                               (e[i] runs from V[i] to V[i+1] on its [0,1] domain).
//   * Edge midpoints    M[i]  = e[i](0.5)          — the split point of boundary edge i.
//   * Centroid          C     = (1/N) · Σ V[i]     — the single interior hub all sub-
//                               patches share.
//
// Sub-patch i (the quad CENTRED on corner V[i]) is bounded by, in the Coons convention
// on [0,1]² (corners P00 = V[i], P10 = M[i], P01 = M[i-1], P11 = C):
//
//   c0 (v=0, u: V[i]→M[i])   = the FIRST HALF of boundary edge e[i]   (e[i] on [0,0.5]),
//   d0 (u=0, v: V[i]→M[i-1]) = the SECOND HALF of edge e[i-1] REVERSED (e[i-1] on [0.5,1]
//                              read from M[i-1]… no: from V[i] back to M[i-1]),
//   c1 (v=1, u: M[i-1]→C)    = the straight interior spoke M[i-1] → C,
//   d1 (u=1, v: M[i]→C)      = the straight interior spoke M[i]   → C.
//
// Each boundary edge e[k] is thus covered by exactly TWO sub-patch outer edges — its
// first half by sub-patch k (as c0) and its second half by sub-patch k+1 (as d0) — so
// the UNION of the N Coons sub-patches reproduces every one of the N input boundary
// curves POINTWISE (each Coons patch interpolates its own boundary exactly, and the two
// halves reconstruct the full edge). Adjacent sub-patches meet C0 along the shared
// interior spoke M[i] → C (sub-patch i's d1 == sub-patch i+1's c1, the SAME straight
// segment) and all meet at the shared centroid C.
//
// This is the OCCT-free 80/20 N-sided fill: bounded, position-exact on the boundary,
// planar-exact for a flat N-gon. It COMPOSES the exact Layer-1 ops (splitCurve /
// reparamCurve to halve the boundary edges; the Coons builder's own elevate/refine) and
// bspline_coons (the per-quad fill). It sits with the rest of the numsci-gated Layer-6
// surfacing family and is CYBERCAD_HAS_NUMSCI-gated for family uniformity (the
// construction itself uses only exact Layer-1 ops — no linear solve).
//
// SCOPE — NON-RATIONAL boundary curves only (all weights = 1), N ≥ 3 boundaries forming
// a closed loop with consecutive shared corners. Rational (weighted) N-sided fill, and
// the GREGORY / energy-minimizing plate blends that achieve tangent (G1) / curvature
// (G2) continuity ACROSS the interior spokes (this construction is C0 there, not G1),
// and mildly-curved-boundary interior fairing, are documented residuals — this module
// never fakes them. See docs/NURBS-SCOPE.md Layer-6 row.
//
// DECLINES HONESTLY (ok = false, with a reason, never a wrong surface, never a crash)
// on a non-closed loop (consecutive corners do not meet), a rational or malformed edge,
// a self-degenerate boundary (N < 3, a zero-length centroid spoke), or any sub-quad the
// Coons builder itself declines.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)

#include <string>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// N-sided boundary.
// ─────────────────────────────────────────────────────────────────────────────

/// The N boundary curves of a topological N-gon, in CCW (or CW) loop order. Edge
/// `edges[i]` runs on [0,1] from corner V[i] to corner V[i+1] (indices mod N), so the
/// loop closes when `edges[i](1) == edges[(i+1)%N](0)` for every i (consecutive shared
/// corners). All edges are non-rational and parametrized on [0,1] (reparametrize first
/// if they are not). N ≥ 3.
struct NSidedBoundary {
  std::vector<BsplineCurveData> edges;  ///< the N boundary curves in loop order
};

/// Result of an N-sided boundary consistency check.
struct NSidedBoundaryCheck {
  bool ok = false;              ///< true ⇔ the N edges form a consistent closed loop
  int n = 0;                    ///< number of edges
  double maxCornerError = 0.0;  ///< max ‖consecutive-corner mismatch‖ (0 ⇔ exact loop)
  std::string reason;           ///< human-readable decline reason when !ok
};

/// Verify that the N edges form a CLOSED loop: every edge is non-rational and well-formed
/// (clamped flat knot vector, degree ≥ 1, ≥ 2 poles), N ≥ 3, and consecutive corners meet
/// `edges[i](1) == edges[(i+1)%N](0)` to within `tol`. Reports the worst corner mismatch.
/// On any violation returns `ok = false` with a reason — never a silently-broken loop.
NSidedBoundaryCheck verifyNSidedBoundary(const NSidedBoundary& b, double tol = 1e-7);

// ─────────────────────────────────────────────────────────────────────────────
// N-sided fill (midpoint subdivision → N Coons sub-patches).
// ─────────────────────────────────────────────────────────────────────────────

/// Result of an N-sided boundary fill.
struct NSidedFillResult {
  bool ok = false;                          ///< true ⇔ the N-sided fill was built
  std::vector<BsplineSurfaceData> patches;  ///< the N non-rational Coons sub-patches
  Point3 centroid{};                        ///< the shared interior hub C
  double maxCornerError = 0.0;              ///< the loop's corner consistency error
  std::string reason;                       ///< decline reason when !ok
};

/// N-SIDED FILL — fill the closed N-gon boundary `b` with N tensor-product B-spline Coons
/// sub-patches by midpoint subdivision (see the file header):
///
///   1. `verifyNSidedBoundary(b, tol)` — decline honestly on a non-closed / rational /
///      degenerate boundary (returns ok=false with a reason; never a wrong surface).
///   2. Compute the N corners V[i] (= edges[i](0)), the N edge midpoints M[i] (= split of
///      edges[i] at 0.5), and the centroid C = mean(V[i]).
///   3. For each i build the quad centred on V[i] with outer edges = the two boundary
///      half-edges (first half of e[i], second half of e[i-1] reversed) and interior edges
///      = the straight spokes to C, then fill it with `coonsPatch`.
///
/// GUARANTEE (the core oracle): the UNION of the N sub-patches CONTAINS every input
/// boundary curve pointwise — each edge e[k] is reproduced by sub-patch k's first-half
/// outer edge and sub-patch k+1's second-half outer edge, and each Coons patch
/// interpolates its own boundary exactly. Adjacent patches meet C0 along the shared
/// interior spoke M[i]→C (same straight segment) and all share the centroid C. A PLANAR
/// N-gon boundary yields N coplanar (flat) patches on the boundary's plane exactly. For
/// N = 4 the same subdivision applies (4 sub-patches whose union reproduces the same
/// four boundary curves as the single Coons patch does). Non-rational only. Declines
/// (`ok=false`) on a non-closed / rational / degenerate boundary or a sub-quad the Coons
/// builder rejects — honest guards, never a crash, never a silently-wrong net.
NSidedFillResult fillNSided(const NSidedBoundary& b, double tol = 1e-7);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_H

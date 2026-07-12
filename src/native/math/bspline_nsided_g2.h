// SPDX-License-Identifier: Apache-2.0
//
// bspline_nsided_g2.h — NURBS roadmap Layer 6: G2 (curvature-continuous) N-SIDED
// boundary-filled surface. Fill a CLOSED boundary of N ≠ 4 curves (a topological N-gon:
// triangle, pentagon, hexagon …) with a smooth multi-patch of tensor-product B-spline
// surfaces that meet CURVATURE-CONTINUOUSLY (G2) across every internal seam and to a
// prescribed boundary curvature field.
//
// This is the G2 upgrade of the G1 fill in bspline_nsided_g1.{h,cpp}. The G1 fill's N
// Gregory bicubic pie-slices meet TANGENT-PLANE-continuously (G1) across every internal
// spoke — the unit normal is continuous, but the second fundamental form (the normal
// curvature) generally jumps at the seam: a curvature crease, visible in a reflection /
// zebra analysis even when the surface looks smooth. Here the same pie-slice topology (N
// sub-patches meeting at the polygon centroid C) is filled with GREGORY QUINTIC patches
// whose across-seam rows encode not just POSITION and the CROSS-TANGENT (∂S/∂u) but the
// CROSS-2ND-DERIVATIVE (∂²S/∂u²) too, so both the unit normal AND the normal curvature
// are continuous across every spoke.
//
// CONSTRUCTION (G2 Gregory / Chiyokura-Kimura; Piegl & Tiller ch.11; Farin) ───────────────
//
// Identical midpoint / pie-slice topology to the G1 fill:
//   * Corners   V[i] = edges[i](0),     * Centroid C = (1/N)·Σ V[i]  (the single hub).
// Slice i covers the WHOLE boundary edge e[i] on the v=0 iso and shrinks to C at v=1; its
// u=0 / u=1 sides are the spokes V[i]→C / V[i+1]→C, each SHARED with a neighbour slice.
//
//   * v-DIRECTION is now QUINTIC (degree 5, 6 poles per u-station). A clamped quintic Bézier
//     endpoint is fixed by (position, 1st derivative, 2nd derivative) at each end — exactly
//     the three data a G2 Hermite carries. At v=0 the six-pole column encodes the boundary
//     POSITION (Q0 = e[i] pole → boundary interpolation machine-exact), the boundary CROSS-
//     TANGENT T (Q1 = Q0 + T/5 → ∂S/∂v(u,0) = T, boundary G1), and the boundary CROSS-2ND-
//     DERIVATIVE K (Q2 = 2Q1 − Q0 + K/20 → ∂²S/∂v²(u,0) = K, boundary G2). At v=1 the column
//     reaches the hub C with radially-tapered 1st/2nd derivatives (the degenerate hub corner).
//
//   * u-DIRECTION G2 across a seam — the two slices sharing spoke k build the SAME three
//     across-seam columns (position spoke column, 1st-inward rib r1, 2nd-inward rib r2) via
//     the IDENTICAL degree-5 clamped u-basis (columns {0,1,2} at u=0 mirror columns
//     {nu-1,nu-2,nu-3} at u=1). Because both incident slices inject byte-for-byte identical
//     position + r1 + r2 into their across-seam columns over a symmetric clamped basis, ALL
//     u-endpoint derivatives — 0th (C0), 1st (∂S/∂u → G1 unit-normal continuity) AND 2nd
//     (∂²S/∂u² → G2 normal-curvature continuity) — match across the seam by POLE EQUALITY:
//     an exact, machine-precision invariant, not a fitted approximation.
//
//   * CENTRE TWIST — the slice interior (twist) rows at the hub are incompatible in general
//     (the classic Gregory twist at a degenerate corner). They are kept finite by radial
//     tapering of the 1st/2nd-derivative rows toward C (the Chiyokura-Kimura reconciliation,
//     folded into the interior rows), which touches neither the boundary rows nor the across-
//     seam ribs, so it can break neither G1 nor G2.
//
// The prescribed boundary cross-tangent (∂S/∂v at v=0) and cross-curvature (∂²S/∂v² at v=0)
// fields are optional; when the caller supplies none, a NATURAL minimal-energy field is
// synthesised from the boundary + centroid geometry (radial toward C, curvature tapered),
// which is analytic-exact for planar / spherical / cylindrical boundaries.
//
// ORACLES (airtight, closed-form) ──────────────────────────────────────────────────────
//   1. BOUNDARY INTERPOLATION — the fill reproduces each of the N input boundary curves to
//      ≤ 1e-12 (each slice's v=0 pole row IS the edge's control net → machine-exact).
//   2. G2 ACROSS SPOKES — at sampled points along every internal spoke BOTH the unit normal
//      (≤ 1e-6 rad) AND the normal curvature (relative ≤ 1e-5) are continuous across the
//      seam. This is the key new invariant beyond G1 (achieved by pole equality of the three
//      across-seam columns → identical ∂S/∂u AND ∂²S/∂u²).
//   3. ANALYTIC (SPHERE / CYLINDER) SANITY — boundary curves lying on a sphere / cylinder
//      patch, with the true analytic cross-tangent + cross-curvature prescribed, produce a
//      fill whose POINTS (≤ 1e-9) AND curvatures (≤ 1e-5) match that analytic surface.
//   4. DEGENERATE-CORNER (Gregory twist) — the hub apex (all slices collapse to C) is kept
//      finite by the radially-tapered interior 1st/2nd rows (no blowup).
//
// G2 FEASIBILITY (the honest precondition) ────────────────────────────────────────────────
// A curvature-continuous surface across the incident spokes requires (a) G1 feasibility (a
// tangent-continuous or planar boundary corner — see bspline_nsided_g1.h) AND (b) a curvature
// field that is CONSISTENT at each corner (the two incident edges' prescribed cross-curvatures
// agree there, or are reconcilable). A boundary that CREASES in 3-D, or a prescribed curvature
// field that is INCOMPATIBLE at a corner (irreconcilable second-order data), is DECLINED — we
// NEVER widen a tolerance to pretend a curvature crease is G2.
//
// HONEST DECLINES — a non-closed loop, a rational / malformed edge, N < 3, a degenerate
// centroid spoke, a G1-infeasible creased corner, a caller cross-tangent (anti-)parallel to
// the boundary tangent (no tangent plane → G1 impossible ⇒ G2 impossible), or a caller
// curvature field whose corner values are incompatible, all return ok = false with a reason —
// never a wrong surface, never a crash.
//
// The C0 `nSidedFill` and G1 `nSidedFillG1` APIs are UNCHANGED — this is purely additive.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G2_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G2_H

#include "bspline_nsided.h"     // NSidedBoundary / verifyNSidedBoundary (shared boundary type)
#include "bspline_nsided_g1.h"  // CrossTangentField (shared prescribed cross-tangent type)
#include "bspline_ops.h"        // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)

#include <string>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Prescribed cross-boundary curvature field.
// ─────────────────────────────────────────────────────────────────────────────

/// The prescribed cross-boundary curvature (surface SECOND derivative ∂²S/∂v² TRANSVERSE to
/// the boundary at v=0) for one boundary edge, given as a B-spline vector field over the SAME
/// [0,1] parameter as the (elevated) edge. `poles` are the field's control vectors, one per
/// u-pole of the elevated edge. An empty field ⇒ synthesise a natural minimal-energy cross-
/// curvature from the boundary + centroid geometry. Reuses CrossTangentField's shape (a per-
/// u-pole vector field); the vector is a 2nd-derivative here, not a 1st.
using CrossCurvatureField = CrossTangentField;

// ─────────────────────────────────────────────────────────────────────────────
// G2 N-sided fill.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a G2 N-sided boundary fill.
struct NSidedFillG2Result {
  bool ok = false;                          ///< true ⇔ the G2 fill was built
  std::vector<BsplineSurfaceData> patches;  ///< the N Gregory (quintic-in-v) sub-patches
  Point3 centroid{};                        ///< the shared interior hub C
  double maxCornerError = 0.0;              ///< the loop's corner consistency error
  double maxBoundaryDev = 0.0;              ///< worst boundary-interpolation residual (reported)
  double maxSpokeNormalAngle = 0.0;         ///< worst G1 normal-mismatch across spokes (rad, reported)
  double maxSpokeCurvatureRel = 0.0;        ///< worst relative normal-curvature mismatch across spokes
  std::string reason;                       ///< decline reason when !ok
};

/// G2 N-SIDED FILL — fill the closed N-gon boundary `b` with N Gregory quintic-in-v sub-patches
/// that meet G2 (curvature continuous) across every internal spoke and to the boundary cross-
/// tangent + cross-curvature fields (see the file header for the full construction).
///
///   1. `verifyNSidedBoundary(b, tol)` — decline honestly on a non-closed / rational /
///      degenerate boundary (returns ok=false with a reason; never a wrong surface).
///   2. Midpoint-subdivide (corners V[i], centroid C); pre-elevate every edge to degree ≥ 5.
///   3. For each edge take the caller's cross-tangent field `tangents[i]` and cross-curvature
///      field `curvatures[i]` (or synthesise natural minimal-energy fields when empty). Decline
///      if a prescribed cross-tangent is (anti-)parallel to the boundary tangent (G1 → G2
///      impossible) or a prescribed curvature is corner-incompatible.
///   4. Build the N Gregory quintic pie-slices: v=0 rows = exact boundary + prescribed cross-
///      tangent + cross-curvature, shared spokes = shared position + 1st-rib + 2nd-rib across-
///      seam columns (⇒ C0/G1/G2 across seams by pole equality), centre twists kept finite by
///      radially-tapered interior 1st/2nd rows (degenerate corner, no blowup).
///
/// GUARANTEE: boundary interpolation ≤ 1e-12, G2 across every spoke — the unit normal
/// (≤ 1e-6 rad) AND the normal curvature (relative ≤ 1e-5) are continuous — and a sphere /
/// cylinder boundary (with the analytic fields prescribed) yields a fill matching that surface
/// (points ≤ 1e-9, curvature ≤ 1e-5). Declines (`ok=false`) on a non-closed / rational /
/// degenerate boundary, a G1-incompatible prescribed tangent, or a corner-incompatible
/// curvature field — honest guards, never a crash, never a silently-non-G2 net.
///
/// `tangents` / `curvatures` — either empty (synthesise all N fields) or exactly N entries,
/// one per edge (an individually-empty entry is synthesised). `tol` — the loop-closure /
/// degeneracy / tangent-parallelism / curvature-consistency tolerance.
NSidedFillG2Result nSidedFillG2(const NSidedBoundary& b,
                                const std::vector<CrossTangentField>& tangents = {},
                                const std::vector<CrossCurvatureField>& curvatures = {},
                                double tol = 1e-7);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_NSIDED_G2_H

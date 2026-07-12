// SPDX-License-Identifier: Apache-2.0
//
// bspline_fair.h — NURBS roadmap Layer 6/7: minimal-energy fairing / smoothing.
//
// FAIRING is the operation that smooths a noisy / wiggly B-spline by minimizing
// its BENDING ENERGY while staying within a tolerance of the original shape. It is
// the clean-up direction for reverse-engineered surfaces (docs/NURBS-SCOPE.md): a
// scan-fitted curve or surface carries high-frequency ripple in its interior
// control points; fairing removes that ripple without moving the geometry more than
// the caller allows.
//
// It sits above the evaluators in bspline.{h,cpp} and reuses the Layer-1 data
// types BsplineCurveData / BsplineSurfaceData from bspline_ops.h (flat knots,
// row-major U-outer surface poles) as both INPUT and OUTPUT — so a faired curve /
// surface drops straight into the rest of the NURBS stack.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 9 (fairing /
// smoothing). The bending energy is discretized on the control net:
//
//   CURVE   E(P)  = Σ_i |P_{i-1} − 2·P_i + P_{i+1}|²                 (≈ ∫|C''|²)
//   SURFACE E(P)  = Σ_{i,j} |S_uu|² + 2|S_uv|² + |S_vv|²             (thin-plate)
//                 with S_uu, S_uv, S_vv the second control-net differences.
//
// Both are quadratic forms P⁰ᵀ K P in the poles (one K per coordinate axis, shared
// across x/y/z). Fairing is the QUADRATIC PROGRAM
//
//   minimize  E(P)   subject to   deviation(P, P⁰) ≤ tol,   ends/boundary fixed.
//
// It is solved as a penalized least squares — minimize ‖P − P⁰‖² + λ·E(P), whose
// stationary point is the linear system  (I + λ·K) P = P⁰  (per coordinate,
// interior poles only). λ is swept upward and the LARGEST λ whose result still
// honors the HARD deviation bound wins: that maximizes energy reduction while the
// max deviation stays ≤ tol exactly. The linear systems go through the numsci
// facade (numerics::lin_solve). No facade widening.
//
// SCOPE — NON-RATIONAL fairing. For a rational input the homogeneous lift applies
// (fair the weighted net in R⁴), documented as a residual; the current routines
// decline rational input honestly rather than fair the Euclidean poles (which is
// not energy-correct for w ≠ 1).
//
// HARD RULES — the deviation bound is NEVER widened to "succeed": the returned
// curve / surface is guaranteed within `tol` of the input, and if no positive
// smoothing is possible without breaching `tol` the routine HONESTLY DECLINES
// (ok=false, energyAfter == energyBefore, the input returned unchanged).
//
// The routines call numerics::lin_solve, so the WHOLE module is under
// CYBERCAD_HAS_NUMSCI (mirroring bspline_fit). With the guard OFF this TU is inert
// and the Layer-6/7 fairing functions are simply absent from the library.
//
// OCCT-FREE, NumPP/SciPP referenced only via the facade. fp64, deterministic.
// clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_FAIR_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_FAIR_H

#include "native/math/bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Result structs — report energy before/after and the achieved max deviation.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of fairing a curve.
struct CurveFairResult {
  bool ok = false;             ///< true ⇔ a within-tol smoothing was applied
  double energyBefore = 0.0;   ///< discrete bending energy of the input
  double energyAfter = 0.0;    ///< discrete bending energy of the result (≤ before)
  double maxDeviation = 0.0;   ///< max ‖faired(t) − input(t)‖ over a dense sample (≤ tol)
  BsplineCurveData curve;      ///< the faired curve (== input when ok == false)
};

/// Result of fairing a surface.
struct SurfaceFairResult {
  bool ok = false;
  double energyBefore = 0.0;
  double energyAfter = 0.0;
  double maxDeviation = 0.0;   ///< max ‖faired(u,v) − input(u,v)‖ over a dense grid (≤ tol)
  BsplineSurfaceData surface;  ///< the faired surface (== input when ok == false)
};

// ─────────────────────────────────────────────────────────────────────────────
// Fairing.
// ─────────────────────────────────────────────────────────────────────────────

/// Fair a B-spline curve: minimize the discrete bending energy Σ|ΔΔP|² over the
/// interior control points, subject to the faired curve staying within `tol` of
/// the input everywhere. `keepEnds` fixes the two end poles at each end
/// (endpoint + first-derivative pole) so position AND end tangent are preserved
/// exactly; when false only the endpoint poles are held (positions preserved).
///
/// Reports energyBefore / energyAfter and the achieved maxDeviation (≤ tol).
/// HONEST DECLINE (ok=false, curve == input, energyAfter == energyBefore) when
/// `tol` is too tight to reduce energy, the input is rational, or the input is too
/// small / malformed to fair.
CurveFairResult fairCurve(const BsplineCurveData& curve, double tol,
                          bool keepEnds = true);

/// Fair a tensor-product B-spline surface: minimize the discrete thin-plate energy
/// Σ|S_uu|² + 2|S_uv|² + |S_vv|² over the interior control points, subject to the
/// faired surface staying within `tol` of the input everywhere. `keepBoundary`
/// fixes the entire boundary control-point ring (all four edges) when true; when
/// false only the four corner poles are held.
///
/// Reports energyBefore / energyAfter and the achieved maxDeviation (≤ tol).
/// HONEST DECLINE (ok=false, surface == input, energyAfter == energyBefore) when
/// `tol` is too tight to reduce energy, the input is rational, or the input grid is
/// too small / malformed to fair.
SurfaceFairResult fairSurface(const BsplineSurfaceData& surface, double tol,
                              bool keepBoundary = true);

// ─────────────────────────────────────────────────────────────────────────────
// Energy inspectors (exposed for testing / reporting — no facade needed).
// ─────────────────────────────────────────────────────────────────────────────

/// Discrete bending energy Σ_i |P_{i-1} − 2·P_i + P_{i+1}|² of a curve's control
/// net (0 for ≤ 2 poles, and 0 for a straight/affine net). Substrate-free.
double curveBendingEnergy(const BsplineCurveData& curve) noexcept;

/// Discrete thin-plate energy Σ|S_uu|² + 2|S_uv|² + |S_vv|² of a surface's control
/// net (second differences along U, along V, and the mixed U-V difference).
/// Substrate-free.
double surfaceBendingEnergy(const BsplineSurfaceData& surface) noexcept;

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_FAIR_H

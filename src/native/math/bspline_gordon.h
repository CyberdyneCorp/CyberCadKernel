// SPDX-License-Identifier: Apache-2.0
//
// bspline_gordon.h — NURBS roadmap Layer 6: GORDON / NETWORK surface (a surface
// through a NETWORK of curves in BOTH parameter directions).
//
// Given a NETWORK of `K` u-direction curves `{C_k(u)}` and `L` v-direction curves
// `{D_l(v)}` that INTERSECT at a `K × L` grid of points `Q_{k,l}`, construct a single
// tensor-product B-spline SURFACE that INTERPOLATES every network curve — the surface
// contains every `C_k` as the iso-curve `S(·, v_k)` and every `D_l` as the iso-curve
// `S(u_l, ·)`. This is the "curve network → smooth skin" direction of freeform
// surfacing (docs/NURBS-SCOPE.md §2/§4 Layer 6): skinning interpolates ONE family of
// parallel sections; a Gordon surface interpolates TWO transversal families at once.
//
// It sits above the evaluators in bspline.{h,cpp} and COMPOSES lower layers:
//   * Layer 1 (bspline_ops.h) — degree elevation / knot refinement make the three
//     tensor-product summand surfaces COMPATIBLE (raise to a common degree, merge to
//     the union knot vector) EXACTLY, so their control nets can be added/subtracted.
//   * Layer 6 (bspline_skin.h) — `skinSurface` skins each direction's family of curves;
//     the compatibility machinery `makeSectionsCompatible` normalizes each family.
//   * Layer 7 (bspline_fit.h) — `interpolateSurface` builds the tensor-product
//     interpolant `T` of the `K × L` intersection grid (collocation + `lin_solve`).
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 — the Gordon
// surface as a BOOLEAN SUM (Coons/Gordon combination):
//
//     G = S_u ⊕ S_v ⊖ T
//
//   where
//     S_u = the lofted (skinned) surface INTERPOLATING the K u-curves `C_k` across v,
//     S_v = the lofted (skinned) surface INTERPOLATING the L v-curves `D_l` across u,
//     T   = the tensor-product surface INTERPOLATING the K×L intersection grid `Q_{k,l}`.
//
// The three summands are brought to a COMMON tensor-product basis (common degrees
// (p,q), common knot vectors in u and v) by the exact Layer-1 ops, after which the
// Gordon net is the pointwise `poles(S_u) + poles(S_v) − poles(T)`. Because the
// boolean sum is the classic Coons/Gordon cancellation, the result interpolates every
// input network curve: on `v = v_k`, `S_u` reduces to `C_k`, and `S_v − T` cancels
// (both reduce to the same interpolant of `Q_{k,·}`), so `G(·, v_k) = C_k`; and
// symmetrically `G(u_l, ·) = D_l`.
//
// NETWORK CONSISTENCY (the honest precondition): the boolean sum only interpolates the
// network when the curves actually form a consistent grid — the u-curve `C_k` evaluated
// at the v-curve station `u_l` must equal the v-curve `D_l` evaluated at `v_k`, both
// equal to the grid point `Q_{k,l}`, to within tolerance. `verifyNetwork` checks this;
// `gordonSurface` DECLINES (`ok = false`, with a reason) on an inconsistent or
// degenerate network — it never emits a surface that silently misses its own curves.
//
// SCOPE — NON-RATIONAL network only (all weights = 1). Rational (weighted) Gordon
// surfaces, IRREGULAR / N-sided networks (non-grid topologies, trimmed boundaries),
// and the exact BRepFill/GeomFill residual are documented residuals for later slices —
// this module never fakes them. See docs/NURBS-SCOPE.md Layer-6 row.
//
// GUARD — the construction composes `skinSurface` / `interpolateSurface`, which solve
// linear systems through the numsci facade, so the whole implementation TU is under
// CYBERCAD_HAS_NUMSCI (exactly like bspline_skin.cpp / bspline_fit.cpp). With the guard
// OFF the TU is inert and the functions are absent; the declarations remain visible for
// documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_GORDON_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_GORDON_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)

#include <span>
#include <string>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Curve network.
// ─────────────────────────────────────────────────────────────────────────────

/// A NETWORK of intersecting curves for a Gordon surface.
///
/// `uCurves[k]` is the k-th u-direction curve `C_k(u)`, associated with the
/// v-parameter `vParams[k]` (the v at which the finished surface's iso-curve equals
/// `C_k`). `vCurves[l]` is the l-th v-direction curve `D_l(v)`, associated with the
/// u-parameter `uParams[l]`. The two families intersect at the grid `Q_{k,l}`, where
/// `C_k(uParams[l]) == D_l(vParams[k])` for every (k,l). All curves are non-rational
/// and share, per family, a common parameter domain (reparametrize first if not).
///
/// Sizes: `uCurves.size() == vParams.size() == K`, `vCurves.size() == uParams.size() == L`.
struct CurveNetwork {
  std::vector<BsplineCurveData> uCurves;  ///< K curves in u, C_k(u)
  std::vector<BsplineCurveData> vCurves;  ///< L curves in v, D_l(v)
  std::vector<double> vParams;            ///< v_k for each u-curve  (size K)
  std::vector<double> uParams;            ///< u_l for each v-curve  (size L)
};

/// Result of a network-consistency check.
struct NetworkCheck {
  bool ok = false;             ///< true ⇔ the network is a consistent, non-degenerate grid
  double maxGridError = 0.0;   ///< max ‖C_k(u_l) − D_l(v_k)‖ over the grid (0 ⇔ exact)
  int K = 0, L = 0;            ///< grid dimensions actually seen
  std::string reason;          ///< human-readable decline reason when !ok
  std::vector<Point3> grid;    ///< the K×L intersection points (row-major, K outer): grid[k*L+l]
};

/// Verify that the `network` forms a CONSISTENT grid: both families are non-rational,
/// well-formed, and their intersections agree — `C_k(uParams[l])` equals
/// `D_l(vParams[k])` (both equal the grid point `Q_{k,l}`) to within `tol` for every
/// (k,l). Also requires the v-params to be strictly increasing over the u-curves and
/// the u-params strictly increasing over the v-curves (a proper monotone grid). On
/// success `grid` holds the K×L intersection points (averaged from the two families).
/// On failure `ok=false` with a `reason` and `maxGridError` (the worst mismatch found).
NetworkCheck verifyNetwork(const CurveNetwork& network, double tol = 1e-7);

// ─────────────────────────────────────────────────────────────────────────────
// Gordon / network surface.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a Gordon-surface construction.
struct GordonResult {
  bool ok = false;             ///< true ⇔ the Gordon surface was built
  BsplineSurfaceData surface;  ///< the non-rational tensor-product Gordon surface
  double maxGridError = 0.0;   ///< the network's grid consistency error (from verifyNetwork)
  std::string reason;          ///< decline reason when !ok
};

/// GORDON surface (§10.5) — build the tensor-product B-spline surface through the whole
/// curve NETWORK as the BOOLEAN SUM `G = S_u ⊕ S_v ⊖ T`:
///
///   1. `verifyNetwork(network, tol)` — decline honestly on an inconsistent/degenerate
///      network (returns ok=false with a reason; never a wrong surface).
///   2. `S_u = skinSurface(uCurves)` re-parametrized so its across-v interpolation lands
///      the u-curves at exactly `network.vParams` (the finished surface contains `C_k`
///      at `v = v_k`). `S_v = skinSurface(vCurves)` likewise across u, at `uParams`.
///   3. `T = interpolateSurface(grid)` — the tensor-product interpolant of the K×L
///      intersection grid at parameters `(uParams, vParams)`.
///   4. Raise the three surfaces to a COMMON degree `(p,q)` and merge them onto a
///      COMMON pair of knot vectors with the exact Layer-1 surface ops, so all three
///      share one basis, then form the Gordon net pointwise:
///          poles(G) = poles(S_u) + poles(S_v) − poles(T).
///
/// GUARANTEE (the core oracle): the Gordon surface CONTAINS every network curve —
/// `G(·, v_k)` equals `C_k` and `G(u_l, ·)` equals `D_l` pointwise, and `G(u_l, v_k)`
/// equals the grid point `Q_{k,l}`. Non-rational only. Requires `K ≥ 2` and `L ≥ 2`.
/// Declines (`ok=false`) on an inconsistent/degenerate network, a rational curve, or a
/// singular interpolation — honest guards, never a crash and never a silently-wrong net.
///
/// `uInterpDegree` / `vInterpDegree` choose the interpolation degree ACROSS each family
/// (clamped to `[1, K−1]` / `[1, L−1]`); the along-curve degrees come from the curves.
GordonResult gordonSurface(const CurveNetwork& network, double tol = 1e-7,
                           int uInterpDegree = 3, int vInterpDegree = 3);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_GORDON_H

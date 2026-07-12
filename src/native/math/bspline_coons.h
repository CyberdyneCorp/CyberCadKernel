// SPDX-License-Identifier: Apache-2.0
//
// bspline_coons.h — NURBS roadmap Layer 6: boundary-filled surface (bilinearly-
// blended COONS patch). Fill a CLOSED boundary of four curves with a smooth
// tensor-product B-spline surface.
//
// Given four boundary B-spline curves forming a topological quad on the unit square
// [0,1]² — `c0(u)` (the edge v = 0), `c1(u)` (the edge v = 1), `d0(v)` (the edge
// u = 0) and `d1(v)` (the edge u = 1) — that share corners:
//
//     c0(0) = d0(0) = P00      c0(1) = d1(0) = P10
//     c1(0) = d0(1) = P01      c1(1) = d1(1) = P11
//
// construct a single tensor-product B-spline SURFACE that INTERPOLATES all four
// boundaries: the surface's four edge iso-curves reproduce the four input curves
// exactly. This is the core surfacing op for FILLING HOLES, CAPPING openings and
// LOFTING a closed boundary (docs/NURBS-SCOPE.md §2/§4 Layer 6). Skinning fills ONE
// family of parallel sections; a Gordon surface fills a REGULAR grid network; a Coons
// patch fills a single CLOSED four-sided boundary loop.
//
// It sits above the evaluators in bspline.{h,cpp} and COMPOSES lower layers:
//   * Layer 1 (bspline_ops.h) — degree elevation / knot refinement make the boundary
//     curves and the three tensor-product summand surfaces COMPATIBLE (raise to a
//     common degree, merge to the union knot vector) EXACTLY, so their control nets
//     can be added / subtracted with no geometry drift.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 (and the classic
// Coons construction, Coons 1967 / Farin) — the BILINEARLY-BLENDED Coons patch as a
// BOOLEAN SUM:
//
//     Coons = L_u ⊕ L_v ⊖ B
//
//   where
//     L_v(u,v) = (1−v)·c0(u) + v·c1(u)   — the RULED surface between c0 and c1
//                                          (linear blend in v; U carries the c-shape),
//     L_u(u,v) = (1−u)·d0(v) + u·d1(v)   — the RULED surface between d0 and d1
//                                          (linear blend in u; V carries the d-shape),
//     B(u,v)   = [(1−u) u]·[[P00 P01];[P10 P11]]·[(1−v) v]ᵀ
//                                          — the BILINEAR tensor product of the four
//                                          corner points (degree 1 in both u and v).
//
// The three summand surfaces are brought to a COMMON tensor-product basis (common
// degrees, common knot vectors in u and v) by the exact Layer-1 ops, after which the
// Coons net is the pointwise `poles(L_v) + poles(L_u) − poles(B)`. The bilinear
// term B is exactly the part L_u and L_v have IN COMMON along the boundary, so the
// boolean sum interpolates every boundary curve: on v = 0, `L_v` reduces to `c0`,
// while `L_u` and `B` both reduce to the SAME straight line between the corners
// P00 and P10 (so `L_u − B` cancels), giving `Coons(u,0) = c0(u)`; and symmetrically
// for the other three edges. Corner interpolation is exact by construction.
//
// COONS IS EXACT for surfaces that ARE bilinearly-blended: feed the four boundary
// iso-curves of a RULED / bilinear surface back through the Coons builder and it
// recovers the original surface pointwise. For a general tensor-product surface the
// Coons patch reproduces the boundary exactly but the interior is the bilinear blend
// (not the original interior) — that is the definition of a Coons patch, not an error.
//
// CORNER CONSISTENCY (the honest precondition): the four boundaries only form a valid
// quad when their shared corners actually coincide — `c0(0) == d0(0)`, `c0(1) == d1(0)`,
// `c1(0) == d0(1)`, `c1(1) == d1(1)` — to within tolerance. `coonsPatch` DECLINES
// (`ok = false`, with a reason) on a mismatched-corner / incompatible / degenerate
// boundary — it never emits a surface that silently misses its own boundary.
//
// SCOPE — NON-RATIONAL boundary curves only (all weights = 1) and exactly FOUR
// boundaries (a topological quad). Rational (weighted) Coons patches, N-SIDED fill
// (5+ boundaries, degenerate-corner / triangular patches), and the Gregory /
// energy-minimizing PLATE blends that achieve tangent (G1) / curvature (G2)
// continuity to the surrounding surfaces are documented residuals for later slices —
// this module never fakes them. See docs/NURBS-SCOPE.md Layer-6 row.
//
// GUARD — the construction is built entirely on the exact Layer-1 ops (no linear
// solve), but the module is compiled under CYBERCAD_HAS_NUMSCI to sit uniformly with
// the rest of the numsci-gated Layer-6 surfacing family (bspline_skin / bspline_sweep
// / bspline_gordon). With the guard OFF the implementation TU is inert and the
// functions are absent; the declarations remain visible for documentation.
//
// OCCT-FREE. clang++ -std=c++20. fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_MATH_BSPLINE_COONS_H
#define CYBERCAD_NATIVE_MATH_BSPLINE_COONS_H

#include "bspline_ops.h"  // BsplineCurveData / BsplineSurfaceData (Layer-1 data types)

#include <string>
#include <vector>

namespace cybercad::native::math {

// ─────────────────────────────────────────────────────────────────────────────
// Four-sided boundary.
// ─────────────────────────────────────────────────────────────────────────────

/// The four boundary curves of a topological quad on the unit square [0,1]²:
///   c0(u) — the edge at v = 0        c1(u) — the edge at v = 1
///   d0(v) — the edge at u = 0        d1(v) — the edge at u = 1
/// with the shared corners c0(0)=d0(0), c0(1)=d1(0), c1(0)=d0(1), c1(1)=d1(1).
/// All four curves are non-rational and parametrized on [0,1] (reparametrize first
/// if they are not). c0/c1 run in u; d0/d1 run in v.
struct CoonsBoundary {
  BsplineCurveData c0;  ///< edge v = 0, a curve in u
  BsplineCurveData c1;  ///< edge v = 1, a curve in u
  BsplineCurveData d0;  ///< edge u = 0, a curve in v
  BsplineCurveData d1;  ///< edge u = 1, a curve in v
};

/// Result of a boundary-corner consistency check.
struct CoonsCornerCheck {
  bool ok = false;             ///< true ⇔ the four boundaries form a consistent quad
  double maxCornerError = 0.0; ///< max ‖corner mismatch‖ over the four corners (0 ⇔ exact)
  std::string reason;          ///< human-readable decline reason when !ok
};

/// Verify that the four boundaries of `b` share their corners: `c0(0)==d0(0)`,
/// `c0(1)==d1(0)`, `c1(0)==d0(1)`, `c1(1)==d1(1)` to within `tol`, and that every
/// boundary is non-rational and well-formed (clamped flat knot vector, degree ≥ 1,
/// ≥ 2 poles). Reports the worst corner mismatch. On any violation returns
/// `ok = false` with a reason, never a silently-wrong quad and never a crash.
CoonsCornerCheck verifyCoonsBoundary(const CoonsBoundary& b, double tol = 1e-7);

// ─────────────────────────────────────────────────────────────────────────────
// Coons patch.
// ─────────────────────────────────────────────────────────────────────────────

/// Result of a Coons-patch construction.
struct CoonsResult {
  bool ok = false;             ///< true ⇔ the Coons surface was built
  BsplineSurfaceData surface;  ///< the non-rational tensor-product Coons surface
  double maxCornerError = 0.0; ///< the boundary's corner consistency error (from the check)
  std::string reason;          ///< decline reason when !ok
};

/// COONS patch — fill the four-sided boundary `b` with the non-rational tensor-product
/// B-spline surface that INTERPOLATES all four boundary curves, as the BOOLEAN SUM
/// `Coons = L_u ⊕ L_v ⊖ B`:
///
///   1. `verifyCoonsBoundary(b, tol)` — decline honestly on a mismatched-corner /
///      rational / degenerate boundary (returns ok=false with a reason; never a wrong
///      surface).
///   2. `L_v` = the ruled surface between c0 and c1 (linear blend in v, degree 1 in v,
///      the c-shape carried in u — c0/c1 first made compatible in u by the Layer-1
///      exact ops). `L_u` = the ruled surface between d0 and d1 (linear blend in u,
///      degree 1 in u, the d-shape carried in v — d0/d1 made compatible in v).
///      `B` = the degree-(1,1) bilinear tensor product of the four corner points.
///   3. Raise the three summands to a COMMON degree and merge them onto COMMON knot
///      vectors in each direction with the exact Layer-1 surface ops, then form the
///      Coons control net pointwise `poles(Coons) = poles(L_v) + poles(L_u) − poles(B)`.
///
/// GUARANTEE (the core oracle): the Coons surface CONTAINS every boundary curve — its
/// edge iso-curve `S(·, 0)` equals `c0`, `S(·, 1)` equals `c1`, `S(0, ·)` equals `d0`
/// and `S(1, ·)` equals `d1` pointwise; the four corners are interpolated exactly.
/// Coons is EXACT for bilinearly-blended surfaces (feed a ruled/bilinear surface's own
/// four boundary iso-curves back in → the original is recovered). Non-rational only.
/// Requires exactly four well-formed boundaries with matching corners. Declines
/// (`ok=false`) on a mismatched-corner / rational / degenerate boundary — honest
/// guards, never a crash and never a silently-wrong net.
CoonsResult coonsPatch(const CoonsBoundary& b, double tol = 1e-7);

}  // namespace cybercad::native::math

#endif  // CYBERCAD_NATIVE_MATH_BSPLINE_COONS_H

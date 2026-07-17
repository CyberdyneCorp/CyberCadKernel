// SPDX-License-Identifier: Apache-2.0
//
// chart_singularity.h — SSI Stage S4-e CHART-SINGULARITY DETECTION + MAPPING.
//
// A CHART SINGULARITY is where ONE surface's own (u,v) PARAMETRIZATION degenerates while
// its 3D point + normal stay perfectly finite: a SPHERE parametric pole (v = ±π/2, where
// ‖dU‖ = R·cos v → 0) or a CONE apex (signed radius R₀ + v·sin α = 0, where the tangential
// ‖dU‖ → 0). The intersection curve can be perfectly TRANSVERSAL through such a point — the
// pair transversality sine ‖n_A × n_B‖ need NOT collapse — yet the S3 marcher breaks there:
// its predictor `advanceParams` solves the single-surface 2×2 normal equations
// [dU dV]ᵀ[dU dV]·(Δu,Δv) = [dU dV]ᵀ(h·t); when the `dU` row vanishes that 2×2 is rank-1, so
// the (u,v) update is ill-conditioned even though the 3D residual and the normal are fine.
// The pole/apex also sits on a NON-PERIODIC v edge, so the marcher reports a spurious
// BoundaryExit (sphere) or step-collapse-crawls the node budget (cone).
//
// WITNESS — the single-surface JACOBIAN RANK-DROP (DISTINCT from S4-c / S4-d). On each
// surface we watch ‖dU‖ against ‖dV‖·scale: a collapse (‖dU‖ → 0 while the normal stays
// finite) flags a pole/apex approach on THAT surface. This is computed from ONE surface's
// own Jacobian — NOT the pair sine ‖n_A × n_B‖ (the S4-c near-tangent witness, which need
// not collapse at a pole) and NOT a locus-tangent flip (the S4-d branch witness). The two
// seams see different quantities and gate different code paths, so a pole crossing (healthy
// sine, no flip) is caught ONLY by this chart witness, and a pair graze / locus branch (no
// ‖dU‖ collapse) is never mistaken for a chart singularity.
//
// THIS HEADER IS THE DETECTOR + MAP-BACK MATH ONLY (design S4-e-1). The CROSSING / reproject
// (S4-e-2/3/4) stays in marching.cpp, which owns the corrector and the WLine assembly — just
// as branch_point.h leaves the arm routing to marching.cpp. Kept out of marching.cpp so the
// single-surface conditioning + the pole-longitude-continuity / apex-single-point mapping are
// isolated and separately readable.
//
// HONESTY. This header only DETECTS + MAPS; it fabricates nothing. The crossing in
// marching.cpp emits a node only if it verifies on BOTH surfaces ≤ onSurfTol (see the guard
// there). A wrong pole-longitude pick simply fails that verification and the march defers.
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE (uses src/native/math only — the finite differences
// need no native-numerics). Meaningful under CYBERCAD_HAS_NUMSCI like the marcher, but does
// not itself require it. clang++ -std=c++20, fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_SSI_CHART_SINGULARITY_H
#define CYBERCAD_NATIVE_SSI_CHART_SINGULARITY_H

#include "native/math/vec.h"
#include "native/ssi/patch_bounds.h"  // SurfaceAdapter / ParamBox

#include <cmath>

namespace cybercad::native::ssi {

namespace chartsing {

using math::Dir3;
using math::Point3;
using math::Vec3;

/// π and 2π for the pole-longitude map.
inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kTwoPi = 6.28318530717958647692;

// ─────────────────────────────────────────────────────────────────────────────
// ChartCond — a single surface's chart conditioning at one (u,v): the finite-difference
// tangent magnitudes ‖dU‖, ‖dV‖ and whether the U chart has COLLAPSED (a pole/apex).
// `collapsed` is the S4-e witness (see chartConditionAt).
// ─────────────────────────────────────────────────────────────────────────────
struct ChartCond {
  double dU = 0.0;         ///< ‖∂S/∂u‖ (finite-difference, same scheme as advanceParams)
  double dV = 0.0;         ///< ‖∂S/∂v‖
  bool normalFinite = true;///< the surface normal is finite here (a pole/apex, not a NaN)
  bool collapsed = false;  ///< the U chart collapsed: ‖dU‖ ≪ ‖dV‖·frac AND ‖dU‖ ≪ scale·frac
  // TRANSPOSE-SYMMETRIC witness (S4-e transposed slice). A surface can carry its degenerate
  // chart in EITHER parametric direction: the sphere-of-revolution family collapses ‖dU‖ at a
  // v-edge (the `collapsed` flag above), but a TRANSPOSED authoring — the profile on U, the
  // revolution on V — collapses ‖dV‖ at a u-edge instead. `collapsedV` is the exact mirror of
  // `collapsed` with the roles of U and V swapped: ‖dV‖ ≪ ‖dU‖·frac AND ‖dV‖ ≪ scale·frac. The
  // two flags are mutually exclusive by construction (each fires only when ITS direction is the
  // smaller). A node with `collapsedV` is crossed by the SAME point-based corrector; only the
  // far-side chart re-seed transposes (a u-edge pole, a far-v inversion) — see marching.cpp.
  bool collapsedV = false; ///< the V chart collapsed: ‖dV‖ ≪ ‖dU‖·frac AND ‖dV‖ ≪ scale·frac
};

// Central finite-difference ‖dU‖ / ‖dV‖ at (u,v), the SAME scheme advanceParams uses (a
// domain-scaled step), then the collapse test. `collapseFrac` is MarchOptions.chartCollapseFrac
// (sentinel-resolved by the caller). The chart U direction has COLLAPSED iff
//   ‖dU‖ < collapseFrac · ‖dV‖   AND   ‖dU‖ < collapseFrac · scale
// while the normal stays finite — i.e. the single-surface Jacobian dropped rank in U with a
// well-defined point + normal (a REMOVABLE pole/apex singularity, not a genuine boundary — a
// finite cap keeps ‖dU‖ = O(‖dV‖) there, so `collapsed` is false and the caller exits it as a
// normal BoundaryExit). Two-sided differences so a v exactly on the ±π/2 pole edge still reads
// a sound ‖dU‖ from the interior side.
inline ChartCond chartConditionAt(const SurfaceAdapter& S, double u, double v, double scale,
                                  double collapseFrac) {
  const ParamBox& dom = S.domain;
  const double hu = std::max(dom.du() * 1e-6, 1e-9);
  const double hv = std::max(dom.dv() * 1e-6, 1e-9);
  // Two-sided where the domain allows, one-sided into the interior at an edge (so a pole on
  // the v edge still measures ‖dV‖ from inside and ‖dU‖ along the collapsing circle).
  const double up = std::min(u + hu, dom.u1), um = std::max(u - hu, dom.u0);
  const double vp = std::min(v + hv, dom.v1), vm = std::max(v - hv, dom.v0);
  const Vec3 dU = (S.point(up, v) - S.point(um, v)) / std::max(up - um, 1e-300);
  const Vec3 dV = (S.point(u, vp) - S.point(u, vm)) / std::max(vp - vm, 1e-300);
  ChartCond c;
  c.dU = math::norm(dU);
  c.dV = math::norm(dV);
  const Vec3 n = S.normal(u, v).vec();
  c.normalFinite = std::isfinite(n.x) && std::isfinite(n.y) && std::isfinite(n.z);
  c.collapsed = c.normalFinite && c.dU < collapseFrac * c.dV && c.dU < collapseFrac * scale;
  // Transpose-symmetric V collapse (the mirror of `collapsed` with U/V swapped). Only fires when
  // V is the SMALLER (dV < dU) so the two flags never fire together — a genuine V-direction chart
  // pole (profile on U, revolution on V) that the U-only witness would miss.
  c.collapsedV = c.normalFinite && c.dV < collapseFrac * c.dU && c.dV < collapseFrac * scale;
  return c;
}

// ── degenerate-v-edge (collapsed-row) pole detector ────────────────────────────────
//
// The per-node `chartConditionAt` collapse test is a POINTWISE ‖dU‖/‖dV‖ ratio. On a CIRCULAR
// collapsed-row pole (a sphere / a surface of revolution) ‖dU‖ = R·… → 0 uniformly along the
// whole u circle, so the ratio dips below `collapseFrac` on every meridian at the same latitude
// and the pointwise witness fires reliably. On a NON-CIRCULAR (elliptical / lumpy) collapsed-row
// pole ‖dU‖ still → 0 (the whole row maps to ONE point — a genuine chart pole), but at DIFFERENT
// RATES per meridian: it collapses fast along the ellipse's major axis and slowly along the minor
// axis, so the pointwise ratio only crosses `collapseFrac` in a razor-thin band the marcher's
// finite step overshoots — it reaches the non-periodic v edge and spuriously BoundaryExits before
// the pointwise witness ever fires. This detector recognises the collapse as a SURFACE PROPERTY of
// the v EDGE (not a pointwise ratio at the current node): the v-domain edge nearest `v` is
// DEGENERATE iff sampling the WHOLE u row at that edge collapses to essentially one 3D point
// (max spread ≪ collapseFrac·scale). That is exactly a chart pole (every u → one point) and is
// FALSE at a genuine finite boundary (a cylinder v-cap, where the edge row is a full circle of
// radius R, spread = 2R ≫ 0). Point-only (never the degenerate dU); a wrong verdict cannot
// fabricate geometry — the crossing that consumes it still emits only nodes verified on both
// surfaces ≤ onSurfTol, else it defers. Returns the degenerate edge's v (v0 or v1) in `edgeV`.
inline bool degenerateVEdge(const SurfaceAdapter& S, double v, double scale, double collapseFrac,
                            double& edgeV) {
  const ParamBox& dom = S.domain;
  // Only meaningful for a non-periodic v (a pole edge is an open latitude limit).
  if (S.vPeriod > 0.0) return false;
  edgeV = (std::fabs(v - dom.v0) <= std::fabs(v - dom.v1)) ? dom.v0 : dom.v1;
  // Sample the u row at the edge; a genuine pole collapses it to one point.
  const int nu = 12;
  const Point3 p0 = S.point(dom.u0, edgeV);
  double spread = 0.0;
  for (int i = 1; i <= nu; ++i) {
    const double u = dom.u0 + (dom.u1 - dom.u0) * (static_cast<double>(i) / nu);
    spread = std::max(spread, math::norm(S.point(u, edgeV) - p0));
  }
  // Collapsed row ⇔ spread ≪ scale (use collapseFrac·scale, the same order the pointwise
  // witness uses for ‖dU‖). A finite v-cap has spread = O(scale) and is rejected.
  return spread < collapseFrac * scale;
}

// ── transposed degenerate-u-edge (collapsed-COLUMN) pole detector ──────────────────
//
// The exact mirror of `degenerateVEdge` with the roles of U and V swapped: recognises a chart
// pole whose degenerate direction is V (the whole v COLUMN at a u-edge maps to one 3D point —
// ‖dV‖ → 0). A transposed surface of revolution (profile on U, revolution on V) collapses here
// rather than at a v-edge, so the collapsed-row detector would never fire. Only meaningful for a
// NON-periodic u (a genuine open u-limit); FALSE at a finite u-cap (that column is a full curve,
// spread = O(scale)). Point-only; a wrong verdict cannot fabricate geometry (the crossing that
// consumes it still emits only nodes verified on both surfaces ≤ onSurfTol, else defers). Returns
// the degenerate edge's u (u0 or u1) in `edgeU`.
inline bool degenerateUEdge(const SurfaceAdapter& S, double u, double scale, double collapseFrac,
                            double& edgeU) {
  const ParamBox& dom = S.domain;
  if (S.uPeriod > 0.0) return false;
  edgeU = (std::fabs(u - dom.u0) <= std::fabs(u - dom.u1)) ? dom.u0 : dom.u1;
  const int nv = 12;
  const Point3 p0 = S.point(edgeU, dom.v0);
  double spread = 0.0;
  for (int i = 1; i <= nv; ++i) {
    const double v = dom.v0 + (dom.v1 - dom.v0) * (static_cast<double>(i) / nv);
    spread = std::max(spread, math::norm(S.point(edgeU, v) - p0));
  }
  return spread < collapseFrac * scale;
}

// ── transposed freeform pole far-side LATITUDE inversion (the V mirror of freeformChartInvert) ──
//
// The V-direction analog of `freeformChartInvert`: at a transposed freeform pole the u is fixed
// just inside the collapsed u-edge and the FAR v (the outgoing "latitude" on the far column) is
// recovered by a point-only nearest-point search over the v domain at that fixed uFix. Same
// well-posedness argument (a fixed u off the pole edge keeps the tiny parallel ring distinguishing
// near vs far column by v; never touches the degenerate dV). Yields only a SEED; the corrector
// finishes the landing and verifies on both surfaces.
inline double freeformChartInvertV(const SurfaceAdapter& S, const Point3& target, double uFix) {
  const ParamBox& d = S.domain;
  auto dist2 = [&](double v) {
    const Vec3 e = S.point(uFix, v) - target;
    return math::dot(e, e);
  };
  const int nv = 64;
  double bv = d.v0, best = dist2(bv);
  for (int i = 1; i <= nv; ++i) {
    const double v = d.v0 + (d.v1 - d.v0) * (static_cast<double>(i) / nv);
    const double f = dist2(v);
    if (f < best) { best = f; bv = v; }
  }
  double hv = (d.v1 - d.v0) / nv;
  for (int it = 0; it < 40; ++it) {
    bool improved = false;
    for (const double sv : {+hv, -hv}) {
      const double v = bv + sv;
      if (v < d.v0 || v > d.v1) continue;
      const double f = dist2(v);
      if (f < best) { best = f; bv = v; improved = true; }
    }
    if (!improved) hv *= 0.5;
  }
  return bv;
}

// ── pole-longitude continuity map ──────────────────────────────────────────────
//
// At a sphere pole the whole u circle collapses to one point, so the longitude u is a FREE
// coordinate there. A great arc through the pole CONTINUES on the OPPOSITE meridian: the
// incoming u and the outgoing u differ by half a turn. Pin the far-side longitude as
// u_out = u_in + π (mod the U period). The caller VERIFIES the resulting far node on both
// surfaces ≤ onSurfTol; a wrong pick does not verify and the march defers (no fabrication).
inline double poleContinuationU(double uIn, double uPeriod) {
  const double period = uPeriod > 0.0 ? uPeriod : kTwoPi;
  double u = uIn + 0.5 * period;
  // wrap into [0, period)
  u = std::fmod(u, period);
  if (u < 0.0) u += period;
  return u;
}

// ── freeform pole far-side longitude inversion (uPeriod == 0) ─────────────────────
//
// The analytic sphere pole has a CLOSED-FORM far meridian (poleContinuationU returns u_in+π at
// the SAME latitude v — the far arc leaves the pole on the opposite meridian and runs back down
// from the same pole edge), but that closed form only exists because the surface is 2π-periodic
// in u. A FREEFORM parametric pole — a collapsed B-spline/NURBS control ROW where ‖dU‖ → 0 while
// the 3D point + normal stay finite (the spline analog of the sphere pole / cone tip) — carries
// uPeriod == 0, so there is NO analytic u+π to apply. We recover the SAME map numerically:
// KEEP the latitude v = vFix (just inside the pole edge, exactly as the analytic reflect does)
// and solve only for the far LONGITUDE — the u at that fixed latitude whose 3D point is nearest
// the marcher's CONTINUED tangent target `anchor + t★·h`. A 1-D search over the u domain (coarse
// scan + shrinking refine) at the FIXED near-pole latitude.
//
// Holding v OFF the pole edge is what keeps this well posed: a full 2-D nearest-point search
// would collapse onto the degenerate pole TIP (every u there maps to one point), whereas at a
// fixed v just inside the edge the tiny parallel ring still distinguishes the near vs far
// meridian by u, so the far longitude is recovered cleanly. POINT-ONLY: touches only S.point,
// never the degenerate dU (the reason it is well posed at the pole, like the point-based
// corrector). It yields only a SEED — the caller's fixed-plane corrector does the exact landing
// and VERIFIES on BOTH surfaces ≤ onSurfTol, so a wrong pick simply fails verification and the
// march DEFERS (no fabrication). The non-periodic analog of poleContinuationU; the periodic
// (analytic) path never calls it and stays bit-identical.
inline double freeformChartInvert(const SurfaceAdapter& S, const Point3& target, double vFix) {
  const ParamBox& d = S.domain;
  auto dist2 = [&](double u) {
    const Vec3 e = S.point(u, vFix) - target;
    return math::dot(e, e);
  };
  const int nu = 64;
  double bu = d.u0, best = dist2(bu);
  for (int i = 1; i <= nu; ++i) {
    const double u = d.u0 + (d.u1 - d.u0) * (static_cast<double>(i) / nu);
    const double f = dist2(u);
    if (f < best) { best = f; bu = u; }
  }
  // Shrinking local refine from the best scan node (the corrector finishes the landing).
  double hu = (d.u1 - d.u0) / nu;
  for (int it = 0; it < 40; ++it) {
    bool improved = false;
    for (const double su : {+hu, -hu}) {
      const double u = bu + su;
      if (u < d.u0 || u > d.u1) continue;
      const double f = dist2(u);
      if (f < best) { best = f; bu = u; improved = true; }
    }
    if (!improved) hu *= 0.5;
  }
  return bu;
}

}  // namespace chartsing

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_CHART_SINGULARITY_H

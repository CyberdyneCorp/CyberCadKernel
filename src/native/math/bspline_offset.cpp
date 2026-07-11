// SPDX-License-Identifier: Apache-2.0
//
// bspline_offset.cpp — NURBS roadmap Layer 5 (surface offset) implementation.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), offset-approximation
// (Ch. 10 § offset). It COMPOSES the evaluators (bspline.h: nurbsSurfacePoint /
// surfaceNormal / surfaceDerivs) and the Layer-7 tensor-product surface fitter
// (bspline_fit.h: interpolateSurface, a collocation solve through the numsci facade).
// Because the fit solves linear systems through numerics::lin_solve, the WHOLE file is
// under CYBERCAD_HAS_NUMSCI (mirroring bspline_fit.cpp / bspline_skin.cpp). With the
// guard OFF this TU is inert and the Layer-5 function is absent from the library.
//
// The exact offset O(u,v) = S + d·N is NOT a NURBS (the unit normal carries a square
// root), so it is sampled on an adaptive grid and a non-rational B-spline is FITTED
// through the samples; the grid is refined until the fit's deviation from the true
// locus O is within tolerance. Two honest guards run BEFORE any fit: a degenerate
// (near-zero) surface normal has no offset direction, and an offset whose |d| reaches
// a principal radius of curvature on the concave side would FOLD — both decline rather
// than return a bad offset.
//
#include "native/math/bspline_offset.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"      // nurbsSurfacePoint / surfaceNormal / surfaceDerivs
#include "native/math/bspline_fit.h"  // interpolateSurface (Layer-7 fit through the grid)
#include "native/numerics/numerics.h" // closest_point_on_surface (offset-distance metric)

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

// A near-zero normal / tangent is a degenerate patch point (no defined offset dir).
constexpr double kNormalEps = 1e-9;
// A first-fundamental-form determinant EG−F² below this is a degenerate parametrization.
constexpr double kMetricEps = 1e-14;

int nPolesUof(const BsplineSurfaceData& s) {
  return static_cast<int>(s.knotsU.size()) - s.degreeU - 1;
}
int nPolesVof(const BsplineSurfaceData& s) {
  return static_cast<int>(s.knotsV.size()) - s.degreeV - 1;
}

// Basic structural validity for a tensor B-spline surface carrier.
bool wellFormed(const BsplineSurfaceData& s) {
  if (s.degreeU < 1 || s.degreeV < 1) return false;
  if (s.nPolesU < s.degreeU + 1 || s.nPolesV < s.degreeV + 1) return false;
  if (static_cast<int>(s.poles.size()) != s.nPolesU * s.nPolesV) return false;
  if (nPolesUof(s) != s.nPolesU || nPolesVof(s) != s.nPolesV) return false;
  if (!s.weights.empty() && static_cast<int>(s.weights.size()) != s.nPolesU * s.nPolesV)
    return false;
  return true;
}

// Parameter domain endpoints of a clamped flat knot vector.
double knotLo(const std::vector<double>& k, int degree) { return k[static_cast<std::size_t>(degree)]; }
double knotHi(const std::vector<double>& k, int degree) {
  return k[k.size() - 1 - static_cast<std::size_t>(degree)];
}

// The evaluated point S(u,v) (rational-aware).
Point3 evalS(const BsplineSurfaceData& s, const SurfaceGrid& grid, double u, double v) {
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v);
}

// The unit surface normal N(u,v) (rational-aware). `valid` reports non-degeneracy.
Dir3 evalN(const BsplineSurfaceData& s, const SurfaceGrid& grid, double u, double v) {
  return surfaceNormal(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v);
}

// ── Principal curvatures at (u,v) (non-rational metric via surfaceDerivs) ─────────
// Returns {κ₁, κ₂} (the two principal curvatures) and sets `ok=false` on a degenerate
// tangent plane / normal. Sign convention: κ > 0 where the surface bends TOWARD +N.
// Uses the first (E,F,G) and second (L,M,Nⁿ) fundamental forms; principal curvatures
// are the eigenvalues of the shape operator, κ = H ± √(H²−K) with mean H and Gaussian K.
struct Curvatures { double k1 = 0.0, k2 = 0.0; bool ok = false; };

Curvatures principalCurvatures(const BsplineSurfaceData& s, const SurfaceGrid& grid,
                               double u, double v) {
  Curvatures out;
  constexpr int md = 2;
  std::array<Vec3, (md + 1) * (md + 1)> d{};
  // Non-rational second-order metric is the standard, robust curvature estimator here;
  // the input may be rational but the offset approximation is non-rational, so the
  // curvature guard uses the control-polygon (non-rational) derivatives of the net.
  if (s.weights.empty())
    surfaceDerivs(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, u, v, md, d);
  else
    nurbsSurfaceDerivs(s.degreeU, s.degreeV, grid, s.weights, s.knotsU, s.knotsV, u, v, md, d);

  const Vec3 Su  = d[1 * (md + 1) + 0];
  const Vec3 Sv  = d[0 * (md + 1) + 1];
  const Vec3 Suu = d[2 * (md + 1) + 0];
  const Vec3 Suv = d[1 * (md + 1) + 1];
  const Vec3 Svv = d[0 * (md + 1) + 2];

  const Vec3 nRaw = cross(Su, Sv);
  const double nLen = norm(nRaw);
  if (nLen <= kNormalEps) return out;  // degenerate normal — no curvature defined
  const Vec3 n = nRaw / nLen;

  const double E = dot(Su, Su), F = dot(Su, Sv), G = dot(Sv, Sv);
  const double det1 = E * G - F * F;
  if (det1 <= kMetricEps) return out;  // degenerate parametrization

  const double L = dot(Suu, n), M = dot(Suv, n), Nn = dot(Svv, n);
  const double K = (L * Nn - M * M) / det1;                    // Gaussian curvature
  const double H = (E * Nn - 2.0 * F * M + G * L) / (2.0 * det1);  // mean curvature
  double disc = H * H - K;
  if (disc < 0.0) disc = 0.0;  // fp guard: umbilic point has disc == 0
  const double root = std::sqrt(disc);
  out.k1 = H + root;
  out.k2 = H - root;
  out.ok = true;
  return out;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Offset surface (Layer 5): normal-locus sampling + Layer-7 fit + adaptive refine.
// ─────────────────────────────────────────────────────────────────────────────

OffsetResult offsetSurface(const BsplineSurfaceData& surface, double d, double tol,
                           int startGrid, int maxGrid) {
  OffsetResult r;

  if (!wellFormed(surface)) {
    r.status = OffsetStatus::DegenerateInput;
    return r;
  }
  if (startGrid < 3) startGrid = 3;
  if (maxGrid < startGrid) maxGrid = startGrid;

  const SurfaceGrid grid{std::span<const Point3>(surface.poles), surface.nPolesU,
                         surface.nPolesV};
  const double u0 = knotLo(surface.knotsU, surface.degreeU);
  const double u1 = knotHi(surface.knotsU, surface.degreeU);
  const double v0 = knotLo(surface.knotsV, surface.degreeV);
  const double v1 = knotHi(surface.knotsV, surface.degreeV);
  if (!(u1 > u0) || !(v1 > v0)) {
    r.status = OffsetStatus::DegenerateInput;
    return r;
  }

  // ── Guards on a dense analysis grid (independent of, and finer than, the coarse
  // fit start grid): degenerate normals and self-intersection (fold) detection. ──
  const int gN = std::max(11, startGrid);  // analysis density for the guards
  double worstFactor = std::numeric_limits<double>::infinity();  // min (1 + d·κ)
  double minRadius = std::numeric_limits<double>::infinity();    // on the offsetting side
  for (int i = 0; i < gN; ++i) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(i) / (gN - 1));
    for (int j = 0; j < gN; ++j) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(j) / (gN - 1));
      const Dir3 nrm = evalN(surface, grid, u, v);
      if (!nrm.valid()) {
        r.status = OffsetStatus::DegenerateNormal;
        return r;
      }
      const Curvatures c = principalCurvatures(surface, grid, u, v);
      if (!c.ok) {
        r.status = OffsetStatus::DegenerateNormal;
        return r;
      }
      // The offset map S ↦ S + d·N has Jacobian eigenvalues (1 + d·κᵢ) in the principal
      // directions. It stays regular (no fold) iff every (1 + d·κᵢ) > 0. The offset FOLDS
      // at the first κ where (1 + d·κ) ≤ 0, i.e. d·κ ≤ −1 — which requires d·κ < 0: the
      // offset bends TOWARD the centre of curvature of that principal direction and the
      // fold occurs once |d| reaches the radius 1/|κ|. Track the smallest regularity
      // factor and, for reporting, the smallest curvature radius on that concave side.
      for (double k : {c.k1, c.k2}) {
        const double factor = 1.0 + d * k;
        worstFactor = std::min(worstFactor, factor);
        if (d * k < 0.0 && std::fabs(k) > kNormalEps)
          minRadius = std::min(minRadius, 1.0 / std::fabs(k));
      }
    }
  }
  r.minCurvatureRadius = std::isfinite(minRadius) ? minRadius : 0.0;
  if (worstFactor <= 0.0) {
    // |d| meets/exceeds a principal radius of curvature on the concave side: the offset
    // would self-intersect (fold). Decline honestly — never return a folded offset.
    r.status = OffsetStatus::SelfIntersection;
    return r;
  }

  // ── Sample the true offset locus O = S + d·N on a grid, fit, refine ──────────────
  // Degrees for the fit: cubic where the surface supports it, else the surface's own.
  const int degU = std::min(3, surface.degreeU);
  const int degV = std::min(3, surface.degreeV);

  BsplineSurfaceData best;
  double bestErr = std::numeric_limits<double>::infinity();
  int bestGU = 0, bestGV = 0;
  bool anyFit = false;

  for (int g = startGrid; ; g = std::min(maxGrid, (g - 1) * 2 + 1)) {
    // Build the (g × g) offset-locus sample grid over the parameter domain.
    std::vector<Point3> samples(static_cast<std::size_t>(g) * g);
    for (int i = 0; i < g; ++i) {
      const double u = u0 + (u1 - u0) * (static_cast<double>(i) / (g - 1));
      for (int j = 0; j < g; ++j) {
        const double v = v0 + (v1 - v0) * (static_cast<double>(j) / (g - 1));
        const Point3 p = evalS(surface, grid, u, v);
        const Dir3 nrm = evalN(surface, grid, u, v);
        samples[static_cast<std::size_t>(i) * g + j] = p + nrm.vec() * d;
      }
    }

    // Fit a non-rational tensor B-spline THROUGH the offset samples (Layer-7).
    const PointGrid pg{std::span<const Point3>(samples), g, g};
    const SurfaceFitResult fit = interpolateSurface(pg, degU, degV);
    if (fit.ok) {
      // Measure the fit's TRUE offset deviation GEOMETRICALLY, independent of how the
      // fitter re-parametrized (its knots are chord-length, not aligned to S's grid): on
      // a dense grid over the FITTED surface's own domain, project each fitted point onto
      // S and require the nearest distance to be |d|. The offset error is
      // max | dist(fittedPoint, S) − |d| | — this is exactly the offset-locus property and
      // needs no param correspondence. Sampled denser than the fit nodes.
      const SurfaceGrid fitGrid{std::span<const Point3>(fit.surface.poles),
                                fit.surface.nPolesU, fit.surface.nPolesV};
      const double fu0 = knotLo(fit.surface.knotsU, fit.surface.degreeU);
      const double fu1 = knotHi(fit.surface.knotsU, fit.surface.degreeU);
      const double fv0 = knotLo(fit.surface.knotsV, fit.surface.degreeV);
      const double fv1 = knotHi(fit.surface.knotsV, fit.surface.degreeV);
      const numerics::SurfaceEval Seval = [&](double uu, double vv) {
        return evalS(surface, grid, uu, vv);
      };
      // A FIXED, modest check grid (not tied to g) keeps the per-level cost bounded; its
      // points fall BETWEEN the fit nodes (offset at ½-cell) so they probe the interpolant
      // where it is free to stray from the locus, not at the reproduced-exactly samples.
      constexpr int kCheck = 7;
      const double absd = std::fabs(d);
      double maxErr = 0.0;
      for (int i = 0; i < kCheck; ++i) {
        const double tu = (i + 0.5) / kCheck;  // cell-centred: strictly interior
        const double fu = fu0 + (fu1 - fu0) * tu;
        for (int j = 0; j < kCheck; ++j) {
          const double tv = (j + 0.5) / kCheck;
          const double fv = fv0 + (fv1 - fv0) * tv;
          const Point3 got =
              surfacePoint(fit.surface.degreeU, fit.surface.degreeV, fitGrid,
                           fit.surface.knotsU, fit.surface.knotsV, fu, fv);
          // Nearest distance from the fitted point to S; the offset property demands it
          // equals |d|. Parametrization-independent, so the fitter's chord-length knots
          // are irrelevant to the measured deviation.
          const numerics::SurfaceProjection pr =
              numerics::closest_point_on_surface(Seval, u0, u1, v0, v1, got, 20, 20);
          if (!pr.success) { maxErr = std::numeric_limits<double>::infinity(); break; }
          maxErr = std::max(maxErr, std::fabs(pr.distance - absd));
        }
        if (!std::isfinite(maxErr)) break;
      }
      if (maxErr < bestErr) {
        bestErr = maxErr;
        best = fit.surface;
        bestGU = g;
        bestGV = g;
        anyFit = true;
      }
      if (maxErr <= tol) break;  // converged within tolerance
    }
    if (g >= maxGrid) break;     // refinement budget spent
  }

  if (!anyFit) {
    r.status = OffsetStatus::FitFailed;
    return r;
  }

  r.surface = std::move(best);
  r.maxError = bestErr;
  r.gridU = bestGU;
  r.gridV = bestGV;
  if (bestErr <= tol) {
    r.ok = true;
    r.status = OffsetStatus::Ok;
  } else {
    // Best fit did not reach tolerance within the budget. The surface IS returned so the
    // caller can inspect it, but flagged with the HONEST achieved error — never widened.
    r.ok = false;
    r.status = OffsetStatus::ToleranceNotMet;
  }
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

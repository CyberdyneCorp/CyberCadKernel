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
#include <functional>
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

// Effective rational weight field w(u,v) = Σ Nᵢ(u)Nⱼ(v)·wᵢⱼ (the homogeneous denominator of
// nurbsSurfacePoint). For a rational input this is the weight the offset SAMPLE inherits when
// we fit a rational approximant: reproducing the input's arc/circle weight profile makes the
// offset of an exact conic the exact offset conic. Returns 1.0 for a non-rational surface.
double evalWeight(const BsplineSurfaceData& s, double u, double v) {
  if (s.weights.empty()) return 1.0;
  const int nU = s.nPolesU - 1;
  const int nV = s.nPolesV - 1;
  const int spanU = findSpan(nU, s.degreeU, u, s.knotsU);
  const int spanV = findSpan(nV, s.degreeV, v, s.knotsV);
  std::array<double, 64> Nu{};
  std::array<double, 64> Nv{};
  basisFuns(spanU, u, s.degreeU, s.knotsU, {Nu.data(), static_cast<std::size_t>(s.degreeU + 1)});
  basisFuns(spanV, v, s.degreeV, s.knotsV, {Nv.data(), static_cast<std::size_t>(s.degreeV + 1)});
  double w = 0.0;
  for (int i = 0; i <= s.degreeU; ++i) {
    const int ii = spanU - s.degreeU + i;
    for (int j = 0; j <= s.degreeV; ++j) {
      const int jj = spanV - s.degreeV + j;
      w += Nu[i] * Nv[j] * s.weights[static_cast<std::size_t>(ii) * s.nPolesV + jj];
    }
  }
  return w;
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

// A rectangular sub-domain of the parameter space to offset over.
struct Domain { double u0, u1, v0, v1; };

// ── Fold analysis on a dense grid ────────────────────────────────────────────────
// Evaluates the offset-map regularity factor min_κ (1 + d·κ) at every node of a gN×gN grid
// over [u0,u1]×[v0,v1]. `regular[i*gN+j]` is true iff that node is fold-free (all factors
// > kFoldEps). Reports the global worst factor and the min curvature radius on the folding
// side. `status` is set to DegenerateNormal on a null normal / degenerate metric.
constexpr double kFoldEps = 1e-9;  // (1 + d·κ) at/below this is a fold (numerically).

struct FoldMap {
  int gN = 0;
  std::vector<char> regular;   // gN*gN, 1 = fold-free node
  double worstFactor = std::numeric_limits<double>::infinity();
  double minRadius = std::numeric_limits<double>::infinity();
  OffsetStatus status = OffsetStatus::Ok;
  bool degenerate = false;
};

FoldMap analyzeFold(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                    const Domain& dom, int gN) {
  FoldMap fm;
  fm.gN = gN;
  fm.regular.assign(static_cast<std::size_t>(gN) * gN, 0);
  for (int i = 0; i < gN; ++i) {
    const double u = dom.u0 + (dom.u1 - dom.u0) * (static_cast<double>(i) / (gN - 1));
    for (int j = 0; j < gN; ++j) {
      const double v = dom.v0 + (dom.v1 - dom.v0) * (static_cast<double>(j) / (gN - 1));
      const Dir3 nrm = evalN(surface, grid, u, v);
      const Curvatures c = principalCurvatures(surface, grid, u, v);
      if (!nrm.valid() || !c.ok) {
        fm.degenerate = true;
        fm.status = OffsetStatus::DegenerateNormal;
        return fm;
      }
      // Jacobian factors (1 + d·κᵢ): regular (no fold) iff every one is > 0. The fold occurs
      // where the offset bends TOWARD a centre of curvature (d·κ < 0) once |d| reaches 1/|κ|.
      double nodeFactor = std::numeric_limits<double>::infinity();
      for (double k : {c.k1, c.k2}) {
        const double factor = 1.0 + d * k;
        nodeFactor = std::min(nodeFactor, factor);
        fm.worstFactor = std::min(fm.worstFactor, factor);
        if (d * k < 0.0 && std::fabs(k) > kNormalEps)
          fm.minRadius = std::min(fm.minRadius, 1.0 / std::fabs(k));
      }
      fm.regular[static_cast<std::size_t>(i) * gN + j] = (nodeFactor > kFoldEps) ? 1 : 0;
    }
  }
  return fm;
}

// ── Maximal all-true axis-aligned rectangle of a boolean grid ────────────────────
// Deterministic largest-rectangle-in-histogram scan over the regularity map: returns the
// inclusive node index bounds [i0,i1]×[j0,j1] of the rectangle with the greatest node area
// whose every node is regular. Empty (i1<i0) if no regular node exists.
struct IndexRect { int i0 = 0, i1 = -1, j0 = 0, j1 = -1; long area = 0; };

IndexRect maximalRegularRect(const std::vector<char>& regular, int gN) {
  IndexRect best;
  std::vector<int> heights(gN, 0);  // consecutive regular nodes UP to row i, per column j
  for (int i = 0; i < gN; ++i) {
    for (int j = 0; j < gN; ++j)
      heights[j] = regular[static_cast<std::size_t>(i) * gN + j] ? heights[j] + 1 : 0;
    // Largest rectangle in this histogram row (monotone stack), tracking node index bounds.
    std::vector<int> stack;  // indices of increasing bar heights
    for (int j = 0; j <= gN; ++j) {
      const int h = (j < gN) ? heights[j] : 0;
      while (!stack.empty() && heights[stack.back()] >= h) {
        const int top = stack.back();
        stack.pop_back();
        const int height = heights[top];
        const int left = stack.empty() ? 0 : stack.back() + 1;
        const int width = j - left;
        const long area = static_cast<long>(height) * width;
        if (height > 0 && area > best.area) {
          best.area = area;
          best.j0 = left;
          best.j1 = j - 1;
          best.i0 = i - height + 1;
          best.i1 = i;
        }
      }
      stack.push_back(j);
    }
  }
  return best;
}

// A parametric warp from the unit square [0,1]² to (u,v) space. The rectangular domain uses
// the identity affine map; the fold-locus trim uses a per-column v-band warp so the sampled
// region FOLLOWS the fold. `s`,`t` ∈ [0,1]. Returns the (u,v) the sample is taken at.
using Warp = std::function<void(double s, double t, double& u, double& v)>;

// Sample the true offset locus O = S + d·N on a g×g grid over the WARPED domain `warp`. When
// `rational`, also fills `outW` with each node's effective input weight (else leaves it empty).
void buildOffsetSamplesWarp(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                            const Warp& warp, int g, bool rational, std::vector<Point3>& out,
                            std::vector<double>& outW) {
  out.resize(static_cast<std::size_t>(g) * g);
  outW.clear();
  if (rational) outW.assign(static_cast<std::size_t>(g) * g, 1.0);
  for (int i = 0; i < g; ++i) {
    const double s = static_cast<double>(i) / (g - 1);
    for (int j = 0; j < g; ++j) {
      const double t = static_cast<double>(j) / (g - 1);
      double u, v;
      warp(s, t, u, v);
      const Dir3 nrm = evalN(surface, grid, u, v);
      const std::size_t idx = static_cast<std::size_t>(i) * g + j;
      out[idx] = evalS(surface, grid, u, v) + nrm.vec() * d;
      if (rational) outW[idx] = evalWeight(surface, u, v);
    }
  }
}

// Sample the true offset locus O = S + d·N on a g×g grid over `dom`. When `rational`, also
// fills `outW` with each node's effective input weight (else leaves it empty).
void buildOffsetSamples(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                        const Domain& dom, int g, bool rational, std::vector<Point3>& out,
                        std::vector<double>& outW) {
  out.resize(static_cast<std::size_t>(g) * g);
  outW.clear();
  if (rational) outW.assign(static_cast<std::size_t>(g) * g, 1.0);
  for (int i = 0; i < g; ++i) {
    const double u = dom.u0 + (dom.u1 - dom.u0) * (static_cast<double>(i) / (g - 1));
    for (int j = 0; j < g; ++j) {
      const double v = dom.v0 + (dom.v1 - dom.v0) * (static_cast<double>(j) / (g - 1));
      const Dir3 nrm = evalN(surface, grid, u, v);
      const std::size_t idx = static_cast<std::size_t>(i) * g + j;
      out[idx] = evalS(surface, grid, u, v) + nrm.vec() * d;
      if (rational) outW[idx] = evalWeight(surface, u, v);
    }
  }
}

// Measure the fitted offset's TRUE geometric deviation from the offset locus over `dom`: on a
// kCheck×kCheck cell-centred grid, project each fitted point onto S and require the nearest
// distance to equal |d|. Parametrization-independent (the fitter re-parametrizes). Returns
// max|dist − |d|| over check points with an INTERIOR foot; +inf if a projection fails.
double measureOffsetDeviation(const numerics::SurfaceEval& Seval, const BsplineSurfaceData& fitS,
                              const Domain& dom, double absd, int kCheck) {
  const SurfaceGrid fitGrid{std::span<const Point3>(fitS.poles), fitS.nPolesU, fitS.nPolesV};
  const double fu0 = knotLo(fitS.knotsU, fitS.degreeU), fu1 = knotHi(fitS.knotsU, fitS.degreeU);
  const double fv0 = knotLo(fitS.knotsV, fitS.degreeV), fv1 = knotHi(fitS.knotsV, fitS.degreeV);
  const bool fitRational = !fitS.weights.empty();
  const double mu = 1e-6 * (dom.u1 - dom.u0), mv = 1e-6 * (dom.v1 - dom.v0);
  double maxErr = 0.0;
  for (int i = 0; i < kCheck; ++i) {
    const double fu = fu0 + (fu1 - fu0) * ((i + 0.5) / kCheck);  // cell-centred, interior
    for (int j = 0; j < kCheck; ++j) {
      const double fv = fv0 + (fv1 - fv0) * ((j + 0.5) / kCheck);
      const Point3 got =
          fitRational ? nurbsSurfacePoint(fitS.degreeU, fitS.degreeV, fitGrid, fitS.weights,
                                          fitS.knotsU, fitS.knotsV, fu, fv)
                      : surfacePoint(fitS.degreeU, fitS.degreeV, fitGrid, fitS.knotsU,
                                     fitS.knotsV, fu, fv);
      const numerics::SurfaceProjection pr =
          numerics::closest_point_on_surface(Seval, dom.u0, dom.u1, dom.v0, dom.v1, got, 20, 20);
      if (!pr.success) return std::numeric_limits<double>::infinity();
      // Skip a foot on the (bounded) patch BOUNDARY: there the nearest point is a boundary
      // point, not the radial offset foot, so dist ≠ |d| is a bounded-patch artifact (notably
      // for INWARD offsets), NOT a fit error. Interior feet give the honest deviation.
      const bool interiorFoot = pr.u > dom.u0 + mu && pr.u < dom.u1 - mu &&
                                pr.v > dom.v0 + mv && pr.v < dom.v1 - mv;
      if (interiorFoot) maxErr = std::max(maxErr, std::fabs(pr.distance - absd));
    }
  }
  return maxErr;
}

// ── Sample → fit → refine the offset locus over one rectangular domain ────────────
// Shared core of all three public entry points. Fits NON-RATIONAL by default; when
// `rational` and the input has weights, samples inherit the input's effective weight and the
// fit is rational (interpolateRationalSurface). Writes surface/maxError/grid* into `r`; sets
// r.ok / r.status (Ok / ToleranceNotMet / FitFailed). Does NOT run the fold guard.
void offsetFitRefine(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                     double tol, int startGrid, int maxGrid, const Domain& dom, bool rational,
                     OffsetResult& r) {
  const int degU = std::min(3, surface.degreeU);
  const int degV = std::min(3, surface.degreeV);
  const bool doRational = rational && !surface.weights.empty();
  // The convergence CHECK density. The non-rational path keeps the historical 7×7 (its fit is
  // low-degree-polynomial and 7×7 resolves its error). The rational path uses a DENSER check
  // so it does not stop early at the coarse grid's ~1e-7 projection floor while the true conic
  // deviation is still ~1e-5 — it must keep refining to reach a tight analytic bound. This is
  // honest (more measurement, the TRUE worst error reported), never a widened tolerance.
  const int kCheck = doRational ? 15 : 7;
  const double absd = std::fabs(d);

  BsplineSurfaceData best;
  double bestErr = std::numeric_limits<double>::infinity();
  int bestGU = 0, bestGV = 0;
  bool anyFit = false;

  const numerics::SurfaceEval Seval = [&](double uu, double vv) {
    return evalS(surface, grid, uu, vv);
  };

  std::vector<Point3> samples;
  std::vector<double> sampW;
  for (int g = startGrid; ; g = std::min(maxGrid, (g - 1) * 2 + 1)) {
    buildOffsetSamples(surface, grid, d, dom, g, doRational, samples, sampW);
    const PointGrid pg{std::span<const Point3>(samples), g, g};
    SurfaceFitResult fit;
    if (doRational) {
      const WeightGrid wg{std::span<const double>(sampW), g, g};
      fit = interpolateRationalSurface(pg, wg, degU, degV);
    } else {
      fit = interpolateSurface(pg, degU, degV);
    }
    if (fit.ok) {
      const double maxErr = measureOffsetDeviation(Seval, fit.surface, dom, absd, kCheck);
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
    r.ok = false;
    return;
  }
  r.surface = std::move(best);
  r.maxError = bestErr;
  r.gridU = bestGU;
  r.gridV = bestGV;
  if (bestErr <= tol) {
    r.ok = true;
    r.status = OffsetStatus::Ok;
  } else {
    r.ok = false;
    r.status = OffsetStatus::ToleranceNotMet;  // honest achieved error, never widened
  }
}

// ── Sample → fit → refine the offset locus over a WARPED domain ───────────────────
// The fold-locus analogue of offsetFitRefine: samples the offset locus over the warped band
// `warp` (an [0,1]² → (u,v) map that follows the fold), fits, and refines. Deviation is
// measured by projecting the fitted surface onto S over the band's bounding box `bbox`
// (on-locus points project to distance |d|; the interior-foot filter drops boundary artefacts).
// Fits NON-RATIONAL (the fold-trim path is non-rational, like offsetSurfaceTrimmed).
void offsetFitRefineWarp(const BsplineSurfaceData& surface, const SurfaceGrid& grid, double d,
                         double tol, int startGrid, int maxGrid, const Warp& warp,
                         const Domain& bbox, OffsetResult& r) {
  const int degU = std::min(3, surface.degreeU);
  const int degV = std::min(3, surface.degreeV);
  const int kCheck = 7;
  const double absd = std::fabs(d);

  BsplineSurfaceData best;
  double bestErr = std::numeric_limits<double>::infinity();
  int bestGU = 0, bestGV = 0;
  bool anyFit = false;

  const numerics::SurfaceEval Seval = [&](double uu, double vv) {
    return evalS(surface, grid, uu, vv);
  };

  std::vector<Point3> samples;
  std::vector<double> sampW;
  for (int g = startGrid; ; g = std::min(maxGrid, (g - 1) * 2 + 1)) {
    buildOffsetSamplesWarp(surface, grid, d, warp, g, /*rational=*/false, samples, sampW);
    const PointGrid pg{std::span<const Point3>(samples), g, g};
    const SurfaceFitResult fit = interpolateSurface(pg, degU, degV);
    if (fit.ok) {
      const double maxErr = measureOffsetDeviation(Seval, fit.surface, bbox, absd, kCheck);
      if (maxErr < bestErr) {
        bestErr = maxErr;
        best = fit.surface;
        bestGU = g;
        bestGV = g;
        anyFit = true;
      }
      if (maxErr <= tol) break;
    }
    if (g >= maxGrid) break;
  }

  if (!anyFit) {
    r.status = OffsetStatus::FitFailed;
    r.ok = false;
    return;
  }
  r.surface = std::move(best);
  r.maxError = bestErr;
  r.gridU = bestGU;
  r.gridV = bestGV;
  if (bestErr <= tol) {
    r.ok = true;
    r.status = OffsetStatus::Ok;
  } else {
    r.ok = false;
    r.status = OffsetStatus::ToleranceNotMet;
  }
}

// Common preamble: validate + clamp grids + compute the full parameter domain. Returns false
// (with r.status set) on a malformed / degenerate-domain surface.
bool prepare(const BsplineSurfaceData& surface, int& startGrid, int& maxGrid, SurfaceGrid& grid,
             Domain& dom, OffsetResult& r) {
  if (!wellFormed(surface)) {
    r.status = OffsetStatus::DegenerateInput;
    return false;
  }
  if (startGrid < 3) startGrid = 3;
  if (maxGrid < startGrid) maxGrid = startGrid;
  grid = SurfaceGrid{std::span<const Point3>(surface.poles), surface.nPolesU, surface.nPolesV};
  dom.u0 = knotLo(surface.knotsU, surface.degreeU);
  dom.u1 = knotHi(surface.knotsU, surface.degreeU);
  dom.v0 = knotLo(surface.knotsV, surface.degreeV);
  dom.v1 = knotHi(surface.knotsV, surface.degreeV);
  if (!(dom.u1 > dom.u0) || !(dom.v1 > dom.v0)) {
    r.status = OffsetStatus::DegenerateInput;
    return false;
  }
  return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Offset surface (Layer 5): normal-locus sampling + Layer-7 fit + adaptive refine.
// ─────────────────────────────────────────────────────────────────────────────

OffsetResult offsetSurface(const BsplineSurfaceData& surface, double d, double tol,
                           int startGrid, int maxGrid) {
  OffsetResult r;
  SurfaceGrid grid;
  Domain dom;
  if (!prepare(surface, startGrid, maxGrid, grid, dom, r)) return r;

  // Fold + degenerate-normal guard on a dense analysis grid over the full domain.
  const int gN = std::max(11, startGrid);
  const FoldMap fm = analyzeFold(surface, grid, d, dom, gN);
  if (fm.degenerate) { r.status = fm.status; return r; }
  r.minCurvatureRadius = std::isfinite(fm.minRadius) ? fm.minRadius : 0.0;
  if (fm.worstFactor <= 0.0) {
    // |d| meets/exceeds a principal radius of curvature on the concave side: would fold.
    r.status = OffsetStatus::SelfIntersection;  // decline honestly — never return folded.
    return r;
  }

  offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, dom, /*rational=*/false, r);
  r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// RATIONAL offset — homogeneous-form locus fit preserving the input weight pattern.
// ─────────────────────────────────────────────────────────────────────────────

OffsetResult offsetSurfaceRational(const BsplineSurfaceData& surface, double d, double tol,
                                   int startGrid, int maxGrid) {
  // Non-rational input has no weight pattern to preserve → identical to offsetSurface.
  if (surface.weights.empty())
    return offsetSurface(surface, d, tol, startGrid, maxGrid);

  OffsetResult r;
  SurfaceGrid grid;
  Domain dom;
  int sg = startGrid, mg = maxGrid;
  if (!prepare(surface, sg, mg, grid, dom, r)) return r;

  const int gN = std::max(11, sg);
  const FoldMap fm = analyzeFold(surface, grid, d, dom, gN);
  if (fm.degenerate) { r.status = fm.status; return r; }
  r.minCurvatureRadius = std::isfinite(fm.minRadius) ? fm.minRadius : 0.0;
  if (fm.worstFactor <= 0.0) {
    r.status = OffsetStatus::SelfIntersection;
    return r;
  }

  // A rational approximant through the offset-locus samples converges more slowly per node
  // than the underlying exact conic warrants (interpolating N points on an arc with the arc's
  // sampled weights is only ~O(h³) exact), so the rational path is allowed a HIGHER sample
  // ceiling to reach a tight (≤1e-6) analytic bound. This is honest refinement — more samples,
  // the TRUE measured error reported — never a widened tolerance.
  const int ratMaxGrid = std::max(mg, 65);
  offsetFitRefine(surface, grid, d, tol, sg, ratMaxGrid, dom, /*rational=*/true, r);
  r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;

  // Guard against a rational fit that FAILED to solve (singular homogeneous collocation): fall
  // back to the non-rational offset so the caller still gets a valid surface. We do NOT switch
  // on the projection-measured error alone (its ~1e-7 grid floor cannot resolve which fit is
  // truly closer); the rational fit is kept whenever it produced a surface.
  if (r.status == OffsetStatus::FitFailed) {
    const OffsetResult nr = offsetSurface(surface, d, tol, startGrid, maxGrid);
    if (nr.status != OffsetStatus::FitFailed) {
      const double kU0 = dom.u0, kU1 = dom.u1, kV0 = dom.v0, kV1 = dom.v1;
      r = nr;
      r.keptU0 = kU0; r.keptU1 = kU1; r.keptV0 = kV0; r.keptV1 = kV1;
    }
  }
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-TRIMMED offset — recover a valid offset over the maximal fold-free rectangle.
// ─────────────────────────────────────────────────────────────────────────────

// If the kept fold-free rectangle covers less than this fraction of the domain area, no
// meaningful offset remains → honest-decline rather than return a sliver.
namespace {
constexpr double kMinKeptFraction = 0.05;

// Map a fold-free NODE-index rectangle to a parameter rectangle, shrunk INWARD by ½ cell on
// each side so its interior is bounded by regular nodes (no fold on the trimmed edge).
// Returns false (leaving `kept` untouched) if the rectangle collapses after the ½-cell inset
// (a one-node-thick strip cannot form a rectangle). Shared by the single- and multi-region
// trim paths so both inset the SAME way.
bool rectToDomain(const IndexRect& rect, const Domain& dom, int gN, Domain& kept) {
  const double du = (dom.u1 - dom.u0) / (gN - 1);
  const double dv = (dom.v1 - dom.v0) / (gN - 1);
  Domain k;
  k.u0 = dom.u0 + (rect.i0 + 0.5) * du;
  k.u1 = dom.u0 + (rect.i1 - 0.5) * du;
  k.v0 = dom.v0 + (rect.j0 + 0.5) * dv;
  k.v1 = dom.v0 + (rect.j1 - 0.5) * dv;
  if (!(k.u1 > k.u0) || !(k.v1 > k.v0)) return false;
  kept = k;
  return true;
}
}  // namespace

OffsetResult offsetSurfaceTrimmed(const BsplineSurfaceData& surface, double d, double tol,
                                  int startGrid, int maxGrid) {
  OffsetResult r;
  SurfaceGrid grid;
  Domain dom;
  if (!prepare(surface, startGrid, maxGrid, grid, dom, r)) return r;

  const int gN = std::max(11, startGrid);
  const FoldMap fm = analyzeFold(surface, grid, d, dom, gN);
  if (fm.degenerate) { r.status = fm.status; return r; }
  r.minCurvatureRadius = std::isfinite(fm.minRadius) ? fm.minRadius : 0.0;

  // Whole domain fold-free → full offset, untrimmed.
  if (fm.worstFactor > 0.0) {
    offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, dom, /*rational=*/false, r);
    r.trimmed = false;
    r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;
    return r;
  }

  // The offset folds somewhere. Find the maximal all-fold-free node rectangle.
  const IndexRect rect = maximalRegularRect(fm.regular, gN);
  const long total = static_cast<long>(gN - 1) * (gN - 1);  // full-domain cell area
  const long keptCells = (rect.i1 > rect.i0 && rect.j1 > rect.j0)
                             ? static_cast<long>(rect.i1 - rect.i0) * (rect.j1 - rect.j0)
                             : 0;
  if (keptCells <= 0 ||
      static_cast<double>(keptCells) < kMinKeptFraction * static_cast<double>(total)) {
    // No fold-free region of meaningful area — decline honestly (never emit folded geometry).
    r.status = OffsetStatus::SelfIntersection;
    return r;
  }

  // Map the node-index rectangle to a parameter rectangle, shrunk INWARD by ½ cell on each
  // side so its interior is bounded by regular nodes (no fold on the trimmed edge).
  Domain kept;
  if (!rectToDomain(rect, dom, gN, kept)) {
    // A one-node-thick strip cannot form a rectangle after the ½-cell inset; decline honestly.
    r.status = OffsetStatus::SelfIntersection;
    return r;
  }

  offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, kept, /*rational=*/false, r);
  r.trimmed = true;
  r.keptU0 = kept.u0; r.keptU1 = kept.u1; r.keptV0 = kept.v0; r.keptV1 = kept.v1;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// MULTI-REGION fold-trimmed offset — greedy maximal-rectangle COVER of the fold-free map.
// ─────────────────────────────────────────────────────────────────────────────

std::vector<OffsetResult> offsetSurfaceMultiTrimmed(const BsplineSurfaceData& surface, double d,
                                                    double tol, int startGrid, int maxGrid) {
  std::vector<OffsetResult> out;

  OffsetResult probe;
  SurfaceGrid grid;
  Domain dom;
  if (!prepare(surface, startGrid, maxGrid, grid, dom, probe)) return out;  // degenerate → empty

  const int gN = std::max(11, startGrid);
  const FoldMap fm = analyzeFold(surface, grid, d, dom, gN);
  if (fm.degenerate) return out;  // degenerate normal → honest empty (no valid region)
  const double minR = std::isfinite(fm.minRadius) ? fm.minRadius : 0.0;

  // Whole domain fold-free → a single full-domain offset (identical to offsetSurface).
  if (fm.worstFactor > 0.0) {
    OffsetResult r;
    r.minCurvatureRadius = minR;
    offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, dom, /*rational=*/false, r);
    r.trimmed = false;
    r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;
    if (r.status != OffsetStatus::FitFailed) out.push_back(std::move(r));
    return out;
  }

  // The offset folds somewhere. GREEDILY cover the fold-free node map with maximal rectangles:
  // extract the largest all-fold-free rectangle, mask its nodes out, and repeat, until no
  // remaining rectangle covers at least kMinKeptFraction of the domain. Each rectangle is a
  // provably fold-free (½-cell inset) sub-region offset independently. This recovers BOTH
  // sides of a fold BAND, not just the single largest side offsetSurfaceTrimmed keeps.
  std::vector<char> regular = fm.regular;  // mutable working copy (mask extracted rects out)
  const long total = static_cast<long>(gN - 1) * (gN - 1);  // full-domain cell area

  for (;;) {
    const IndexRect rect = maximalRegularRect(regular, gN);
    const long keptCells = (rect.i1 > rect.i0 && rect.j1 > rect.j0)
                               ? static_cast<long>(rect.i1 - rect.i0) * (rect.j1 - rect.j0)
                               : 0;
    if (keptCells <= 0 ||
        static_cast<double>(keptCells) < kMinKeptFraction * static_cast<double>(total))
      break;  // no further meaningful fold-free rectangle

    // Mask this rectangle's nodes out of the working map so the next iteration finds a
    // DISJOINT rectangle (the extracted region is never re-covered).
    for (int i = rect.i0; i <= rect.i1; ++i)
      for (int j = rect.j0; j <= rect.j1; ++j)
        regular[static_cast<std::size_t>(i) * gN + j] = 0;

    Domain kept;
    if (!rectToDomain(rect, dom, gN, kept)) continue;  // collapsed after inset — skip, keep going

    OffsetResult r;
    r.minCurvatureRadius = minR;
    offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, kept, /*rational=*/false, r);
    if (r.status == OffsetStatus::FitFailed) continue;  // a region that will not fit is dropped
    r.trimmed = true;
    r.keptU0 = kept.u0; r.keptU1 = kept.u1; r.keptV0 = kept.v0; r.keptV1 = kept.v1;
    out.push_back(std::move(r));
  }
  // Empty ⇔ no meaningful fold-free region (honest-decline, same bar as offsetSurfaceTrimmed).
  return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// FOLD-LOCUS-following trim — column-band region bounded by the actual fold locus.
// ─────────────────────────────────────────────────────────────────────────────

namespace {

// 4-connected component labels of the fold-free node map. Every regular node gets a component
// id ≥ 0; folded nodes get -1. `nComp` is the number of components. A diagonal / curved fold
// BAND separates the fold-free map into ≥ 2 components (one per side); a corner-only fold
// leaves one. Using 4-connectivity (not 8) makes the band a genuine SEPARATOR — two regions
// touching only at a diagonal corner across the band stay distinct.
std::vector<int> labelComponents(const std::vector<char>& regular, int gN, int& nComp) {
  std::vector<int> label(static_cast<std::size_t>(gN) * gN, -1);
  nComp = 0;
  std::vector<int> stack;
  for (int si = 0; si < gN; ++si)
    for (int sj = 0; sj < gN; ++sj) {
      const std::size_t s0 = static_cast<std::size_t>(si) * gN + sj;
      if (!regular[s0] || label[s0] != -1) continue;
      const int id = nComp++;
      label[s0] = id;
      stack.clear();
      stack.push_back(si * gN + sj);
      while (!stack.empty()) {
        const int cur = stack.back();
        stack.pop_back();
        const int ci = cur / gN, cj = cur % gN;
        const int di[4] = {-1, 1, 0, 0};
        const int dj[4] = {0, 0, -1, 1};
        for (int k = 0; k < 4; ++k) {
          const int ni = ci + di[k], nj = cj + dj[k];
          if (ni < 0 || ni >= gN || nj < 0 || nj >= gN) continue;
          const std::size_t ns = static_cast<std::size_t>(ni) * gN + nj;
          if (regular[ns] && label[ns] == -1) {
            label[ns] = id;
            stack.push_back(ni * gN + nj);
          }
        }
      }
    }
  return label;
}

// A per-u-column contiguous fold-free v-interval for one component. `iLo..iHi` is the inclusive
// u-node range the component spans; `jLo[k]`,`jHi[k]` (k over the range) are the component's
// contiguous v-node interval on u-column iLo+k. A component whose column intervals are all
// well-formed maps to a column-BAND warp that follows the fold locus.
struct ColumnBand {
  int iLo = 0, iHi = -1;
  std::vector<int> jLo, jHi;  // size iHi-iLo+1
  long nodeArea = 0;          // Σ (jHi-jLo) per column × 1 (proxy for kept area)
};

// Decompose component `id` into SIMPLE column-bands (one contiguous fold-free v-run per
// column) by a SCANLINE sweep with run tracking. A component whose fold boundary is a
// straight/diagonal graph over u has one run per column and yields exactly ONE band (the
// behaviour of the original single-band trace). A GENUINELY CURVED fold locus — a CLOSED
// fold loop (e.g. a fold disk around a dome crest) or a C/arc-shaped fold band — leaves a
// component whose columns cross the fold and carry TWO+ runs; the sweep then SPLITS the
// component at the column where a run forks (and MERGES bands back when runs rejoin),
// partitioning the component into multiple pairwise-disjoint simple bands whose per-column
// intervals each trace their side of the curved fold boundary. Every run of every column
// belongs to exactly one band, so the union of the bands is the whole component (minus the
// one-column seams at split/merge stations, which the ½-cell u-inset drops anyway).
std::vector<ColumnBand> extractColumnBands(const std::vector<int>& label, int gN, int id) {
  std::vector<ColumnBand> closed;
  struct Run { int lo = 0, hi = 0; };
  struct Active {
    ColumnBand band;
    Run last;  // the band's run on the previous column (adjacency test for the next column)
  };
  std::vector<Active> act;

  for (int i = 0; i < gN; ++i) {
    // The runs of component `id` on column i (maximal contiguous j-intervals).
    std::vector<Run> runs;
    for (int j = 0; j < gN; ++j) {
      if (label[static_cast<std::size_t>(i) * gN + j] != id) continue;
      if (!runs.empty() && runs.back().hi == j - 1) runs.back().hi = j;
      else runs.push_back({j, j});
    }

    // Overlap counts between active bands and this column's runs (4-connectivity: a band
    // continues into a run iff their j-intervals share at least one node).
    const int nA = static_cast<int>(act.size());
    const int nR = static_cast<int>(runs.size());
    std::vector<int> aHits(nA, 0), rHits(nR, 0), rBand(nR, -1);
    for (int a = 0; a < nA; ++a)
      for (int r = 0; r < nR; ++r)
        if (act[a].last.lo <= runs[r].hi && runs[r].lo <= act[a].last.hi) {
          ++aHits[a];
          ++rHits[r];
          rBand[r] = a;
        }

    // A band EXTENDS by a run iff the match is one-to-one. A band overlapping 0 runs ends;
    // a band overlapping 2+ runs SPLITS (close it, each run starts fresh); 2+ bands
    // overlapping one run MERGE (close them, the run starts fresh).
    std::vector<Active> next;
    std::vector<char> extended(nA, 0);
    for (int r = 0; r < nR; ++r) {
      if (rHits[r] == 1 && aHits[rBand[r]] == 1) {
        Active a = std::move(act[rBand[r]]);
        extended[rBand[r]] = 1;
        a.band.iHi = i;
        a.band.jLo.push_back(runs[r].lo);
        a.band.jHi.push_back(runs[r].hi);
        a.band.nodeArea += (runs[r].hi - runs[r].lo);
        a.last = runs[r];
        next.push_back(std::move(a));
      } else {
        Active a;
        a.band.iLo = i;
        a.band.iHi = i;
        a.band.jLo.push_back(runs[r].lo);
        a.band.jHi.push_back(runs[r].hi);
        a.band.nodeArea = runs[r].hi - runs[r].lo;
        a.last = runs[r];
        next.push_back(std::move(a));
      }
    }
    for (int a = 0; a < nA; ++a)
      if (!extended[a]) closed.push_back(std::move(act[a].band));
    act = std::move(next);
  }
  for (Active& a : act) closed.push_back(std::move(a.band));
  return closed;
}

}  // namespace

std::vector<OffsetResult> offsetSurfaceFoldTrim(const BsplineSurfaceData& surface, double d,
                                                double tol, int startGrid, int maxGrid) {
  std::vector<OffsetResult> out;

  OffsetResult probe;
  SurfaceGrid grid;
  Domain dom;
  if (!prepare(surface, startGrid, maxGrid, grid, dom, probe)) return out;  // degenerate → empty

  // A DENSE fold map: the fold-locus trace needs finer resolution than the rectangle scan so the
  // traced per-column intervals hug the diagonal / curved fold. This is honest measurement (a
  // finer analysis of the SAME (1 + d·κ) map), never a widened tolerance.
  const int gN = std::max(41, startGrid);
  const FoldMap fm = analyzeFold(surface, grid, d, dom, gN);
  if (fm.degenerate) return out;  // degenerate normal → honest empty
  const double minR = std::isfinite(fm.minRadius) ? fm.minRadius : 0.0;

  // Whole domain fold-free → a single full-domain offset (identical to offsetSurface).
  if (fm.worstFactor > 0.0) {
    OffsetResult r;
    r.minCurvatureRadius = minR;
    offsetFitRefine(surface, grid, d, tol, startGrid, maxGrid, dom, /*rational=*/false, r);
    r.trimmed = false;
    r.foldTrimmed = false;
    r.keptU0 = dom.u0; r.keptU1 = dom.u1; r.keptV0 = dom.v0; r.keptV1 = dom.v1;
    if (r.status != OffsetStatus::FitFailed) out.push_back(std::move(r));
    return out;
  }

  // The offset folds somewhere. Label the fold-free node map into connected components (a
  // diagonal band leaves ≥ 2) and decompose each into SIMPLE COLUMN-BANDS that FOLLOW the
  // fold: per u-column one contiguous fold-free v-interval, inset ½ cell so the interior is
  // provably fold-free. A straight/diagonal fold yields one band per component; a CURVED /
  // CLOSED fold locus (columns with 2+ fold-free runs) is split by the scanline decomposition
  // into several bands around it. The warped fit over each band recovers the curved-boundary
  // material a rectangle (or a single per-u interval) drops.
  int nComp = 0;
  const std::vector<int> label = labelComponents(fm.regular, gN, nComp);
  const double du = (dom.u1 - dom.u0) / (gN - 1);
  const double dv = (dom.v1 - dom.v0) / (gN - 1);
  const long total = static_cast<long>(gN - 1) * (gN - 1);

  // Collect every component's simple column-bands (a curved / closed fold locus decomposes a
  // component into SEVERAL per-u single-interval bands), then emit in descending area order.
  struct Cand { int id; ColumnBand band; };
  std::vector<Cand> cands;
  for (int id = 0; id < nComp; ++id) {
    for (ColumnBand& band : extractColumnBands(label, gN, id)) {
      if (band.iHi < band.iLo) continue;
      // Meaningful-area gate (same bar as the rectangle trim): the band must cover ≥ kMinKeptFraction.
      if (static_cast<double>(band.nodeArea) < kMinKeptFraction * static_cast<double>(total)) continue;
      // The band must be ≥ 2 columns wide AND have some column with a ≥ 2-node interval, else the
      // ½-cell inset collapses it (a one-node sliver is not a fittable region).
      if (band.iHi - band.iLo < 2) continue;
      cands.push_back({id, std::move(band)});
    }
  }
  std::sort(cands.begin(), cands.end(),
            [](const Cand& a, const Cand& b) { return a.band.nodeArea > b.band.nodeArea; });

  for (const Cand& c : cands) {
    const ColumnBand& band = c.band;
    const int nCol = band.iHi - band.iLo + 1;

    // Per-column parameter (u, vLo, vHi). The fold-side edge is inset from the fold boundary by
    // kFoldEdgeCells cells (not just ½): the offset locus curvature is EXTREME approaching the
    // fold, so a strip hugging the fold pushes the fit error up. Insetting a cell keeps the
    // retained region genuinely fold-free with margin. An edge on the DOMAIN boundary (v=0 or
    // v=1) is a true patch boundary, not a high-curvature fold, so it is inset only ½ cell.
    constexpr double kFoldEdgeCells = 2.0;
    std::vector<double> uS(nCol), vLo(nCol), vHi(nCol);
    Domain bbox{dom.u1, dom.u0, dom.v1, dom.v0};  // accumulate the band bounding box
    bool anyCol = false;
    for (int k = 0; k < nCol; ++k) {
      const int i = band.iLo + k;
      uS[k] = dom.u0 + i * du;
      const double loMargin = (band.jLo[k] <= 0) ? 0.5 : kFoldEdgeCells;
      const double hiMargin = (band.jHi[k] >= gN - 1) ? 0.5 : kFoldEdgeCells;
      double lo = dom.v0 + (band.jLo[k] + loMargin) * dv;
      double hi = dom.v0 + (band.jHi[k] - hiMargin) * dv;
      if (!(hi > lo)) { lo = hi = dom.v0 + 0.5 * (band.jLo[k] + band.jHi[k]) * dv; }
      else anyCol = true;
      vLo[k] = lo; vHi[k] = hi;
      bbox.u0 = std::min(bbox.u0, uS[k]); bbox.u1 = std::max(bbox.u1, uS[k]);
      bbox.v0 = std::min(bbox.v0, lo);    bbox.v1 = std::max(bbox.v1, hi);
    }
    if (!anyCol) continue;  // every column collapsed after the inset — skip

    // Trim DEGENERATE TIP columns: a triangular band tapers to a point at the far corner, where
    // the interval width → 0. A zero-width column makes a degenerate cap quad (and adjacent caps
    // can pinch/self-touch), so drop leading/trailing columns whose width is below a small
    // fraction of the band's widest column — keeping a proper trapezoid, not a spike-to-point.
    {
      double wMax = 0.0;
      for (int k = 0; k < nCol; ++k) wMax = std::max(wMax, vHi[k] - vLo[k]);
      const double wMin = 0.05 * wMax;  // tip columns thinner than this are degenerate
      int a = 0, b = nCol - 1;
      while (a <= b && (vHi[a] - vLo[a]) < wMin) ++a;
      while (b >= a && (vHi[b] - vLo[b]) < wMin) --b;
      if (b - a + 1 < 3) continue;  // too few well-formed columns to fit a band — skip
      if (a > 0 || b < nCol - 1) {
        uS.assign(uS.begin() + a, uS.begin() + b + 1);
        vLo.assign(vLo.begin() + a, vLo.begin() + b + 1);
        vHi.assign(vHi.begin() + a, vHi.begin() + b + 1);
      }
    }

    // Inset the u-extent by ½ cell on each end too (the extreme columns border the fold).
    const double uInset = 0.5 * du;
    const double bandU0 = uS.front() + uInset, bandU1 = uS.back() - uInset;
    if (!(bandU1 > bandU0)) continue;

    // Recompute the band bounding box over the (possibly tip-trimmed) columns.
    bbox = Domain{bandU0, bandU1, dom.v1, dom.v0};
    for (std::size_t k = 0; k < vLo.size(); ++k) {
      bbox.v0 = std::min(bbox.v0, vLo[k]);
      bbox.v1 = std::max(bbox.v1, vHi[k]);
    }

    // The warp: s ∈ [0,1] → u across the band (inset), interpolate the (vLo,vHi) envelope at u by
    // piecewise-linear lookup on the per-column stations, t ∈ [0,1] → v across [vLo(u),vHi(u)].
    const std::vector<double> uSc = uS, vLoc = vLo, vHic = vHi;  // capture by value
    const int nc = static_cast<int>(uSc.size());
    Warp warp = [uSc, vLoc, vHic, nc, bandU0, bandU1](double s, double t, double& u, double& v) {
      u = bandU0 + (bandU1 - bandU0) * s;
      // locate u in the station array (ascending, uniform) → linear interp of the envelope.
      double lo, hi;
      if (u <= uSc.front()) { lo = vLoc.front(); hi = vHic.front(); }
      else if (u >= uSc.back()) { lo = vLoc.back(); hi = vHic.back(); }
      else {
        int seg = 0;
        while (seg + 1 < nc && uSc[seg + 1] < u) ++seg;
        const double a = (u - uSc[seg]) / (uSc[seg + 1] - uSc[seg]);
        lo = vLoc[seg] + a * (vLoc[seg + 1] - vLoc[seg]);
        hi = vHic[seg] + a * (vHic[seg + 1] - vHic[seg]);
      }
      v = lo + (hi - lo) * t;
    };

    OffsetResult r;
    r.minCurvatureRadius = minR;
    offsetFitRefineWarp(surface, grid, d, tol, startGrid, maxGrid, warp, bbox, r);
    if (r.status == OffsetStatus::FitFailed) continue;  // a band that will not fit is dropped
    r.trimmed = true;
    r.foldTrimmed = true;
    r.keptU0 = bbox.u0; r.keptU1 = bbox.u1; r.keptV0 = bbox.v0; r.keptV1 = bbox.v1;
    r.foldU = uS; r.foldVLo = vLo; r.foldVHi = vHi;
    out.push_back(std::move(r));
  }
  // Empty ⇔ no meaningful fold-free component (honest-decline; never emit a folded region).
  return out;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

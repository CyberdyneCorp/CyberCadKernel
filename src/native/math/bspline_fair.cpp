// SPDX-License-Identifier: Apache-2.0
//
// bspline_fair.cpp — NURBS roadmap Layer 6/7 (minimal-energy fairing) impl.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 9. The
// energy functionals are substrate-free (curveBendingEnergy / surfaceBendingEnergy
// need no facade); the fairing routines solve the penalized-least-squares system
// (I + λ·K) P = P⁰ through the numsci facade (numerics::lin_solve), so the WHOLE
// file is under CYBERCAD_HAS_NUMSCI (mirroring bspline_fit.cpp). With the guard OFF
// this TU is inert and the Layer-6/7 fairing functions are simply absent.
//
#include "native/math/bspline_fair.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint / surfacePoint (deviation oracle)
#include "native/numerics/numerics.h"   // lin_solve

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::math {
namespace {

using numerics::lin_solve;

// ── Energy: second control-net differences ────────────────────────────────────
// A curve's discrete bending energy is Σ over interior indices i of the squared
// length of the second difference Δ²P_i = P_{i-1} − 2·P_i + P_{i+1}. This is the
// standard discrete surrogate for ∫|C''|²: a straight (affine) net has Δ²P ≡ 0.
Vec3 secondDiff(const std::vector<Point3>& P, int i) noexcept {
  return (P[i - 1] - Point3{}) - 2.0 * (P[i] - Point3{}) + (P[i + 1] - Point3{});
}

double curveEnergy(const std::vector<Point3>& P) noexcept {
  double e = 0.0;
  const int n = static_cast<int>(P.size());
  for (int i = 1; i + 1 < n; ++i) e += normSquared(secondDiff(P, i));
  return e;
}

// Characteristic squared scale of a control net (max pairwise-adjacent squared
// extent). Used to decide when an energy is negligible (already-fair no-op) rather
// than genuinely reducible — a straight net carries only fp round-off energy
// (~1e-30·scale), which is not worth (and cannot be) faired.
double netScaleSquared(const std::vector<Point3>& P) noexcept {
  double s = 0.0;
  for (std::size_t i = 1; i < P.size(); ++i) s = std::max(s, normSquared(P[i] - P[i - 1]));
  return s;
}

// ── Curve deviation oracle: max ‖faired(u) − input(u)‖ over a dense sample ─────
// The HARD bound is the geometric deviation of the curve, not of the control net,
// so it is measured on the evaluated curve at many parameters spanning the domain.
double curveDeviation(const BsplineCurveData& faired, const BsplineCurveData& input) {
  const std::vector<double>& U = input.knots;
  const int p = input.degree;
  const int last = static_cast<int>(input.poles.size());  // = nPoles
  // Domain is [U[p], U[nPoles]] for a clamped flat knot vector.
  const double t0 = U[p];
  const double t1 = U[static_cast<std::size_t>(last)];
  const int samples = 20 * last + 40;
  double maxDev = 0.0;
  for (int s = 0; s <= samples; ++s) {
    const double t = t0 + (t1 - t0) * (static_cast<double>(s) / samples);
    const Point3 a = curvePoint(p, faired.poles, U, t);
    const Point3 b = curvePoint(p, input.poles, U, t);
    maxDev = std::max(maxDev, distance(a, b));
  }
  return maxDev;
}

// Solve the penalized-least-squares system (I + λ·K) x = b for the FREE (interior)
// poles only, one coordinate axis at a time. `free` lists the pole indices that may
// move; `L` is the second-difference incidence built once. The Hessian block over
// the free set is K = LᵀL restricted to free×free; the RHS folds the fixed poles'
// contribution to the identity term (the fixed poles sit at their original value).
//
// We assemble the (m×m) system A = I + λ·(Lᵀ L)|free and RHS = P⁰|free for each of
// x/y/z, then lin_solve. Only the identity anchors the free poles to their original
// positions; the λ·K term pulls them toward the low-energy (smooth) net. The fixed
// poles never appear as unknowns but DO enter K's rows for free poles adjacent to
// them, so their (constant) contribution is moved to the RHS.
struct FairSystem {
  int m = 0;                         // number of free poles
  std::vector<int> freeIdx;          // free pole indices (into the full net)
  std::vector<int> slot;             // full-net index → free-slot (−1 if fixed)
};

// Assemble and solve one λ for a 1-D chain of poles (curve). Returns the full
// updated pole list (fixed poles unchanged). `K` is the dense free×free Gram of the
// second-difference operator; `rhsConst[axis][s]` folds fixed-neighbour terms.
std::vector<Point3> solveCurveLambda(const std::vector<Point3>& P0,
                                     const FairSystem& fs,
                                     const std::vector<double>& K,  // m×m, row-major
                                     const std::vector<double>& rhsX,
                                     const std::vector<double>& rhsY,
                                     const std::vector<double>& rhsZ,
                                     double lambda) {
  const int m = fs.m;
  std::vector<double> A(static_cast<std::size_t>(m) * m);
  for (int i = 0; i < m; ++i)
    for (int j = 0; j < m; ++j) {
      const double kij = lambda * K[static_cast<std::size_t>(i) * m + j];
      A[static_cast<std::size_t>(i) * m + j] = (i == j ? 1.0 : 0.0) + kij;
    }
  const std::vector<double> cx = lin_solve(A, m, rhsX);
  const std::vector<double> cy = lin_solve(A, m, rhsY);
  const std::vector<double> cz = lin_solve(A, m, rhsZ);
  if (static_cast<int>(cx.size()) != m) return {};  // singular — signal failure

  std::vector<Point3> P = P0;
  for (int s = 0; s < m; ++s) P[fs.freeIdx[s]] = {cx[s], cy[s], cz[s]};
  return P;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public energy inspectors (substrate-free — usable without the facade too, but
// this TU only compiles under CYBERCAD_HAS_NUMSCI for family uniformity).
// ─────────────────────────────────────────────────────────────────────────────

double curveBendingEnergy(const BsplineCurveData& curve) noexcept {
  return curveEnergy(curve.poles);
}

double surfaceBendingEnergy(const BsplineSurfaceData& surface) noexcept {
  const int nu = surface.nPolesU, nv = surface.nPolesV;
  const auto& P = surface.poles;
  auto at = [&](int i, int j) -> Vec3 {
    return P[static_cast<std::size_t>(i) * nv + j] - Point3{};
  };
  double e = 0.0;
  // |S_uu|²: second difference along U (interior rows), every column.
  for (int i = 1; i + 1 < nu; ++i)
    for (int j = 0; j < nv; ++j)
      e += normSquared(at(i - 1, j) - 2.0 * at(i, j) + at(i + 1, j));
  // |S_vv|²: second difference along V (interior columns), every row.
  for (int i = 0; i < nu; ++i)
    for (int j = 1; j + 1 < nv; ++j)
      e += normSquared(at(i, j - 1) - 2.0 * at(i, j) + at(i, j + 1));
  // 2·|S_uv|²: mixed difference on interior 2×2 stencils.
  for (int i = 1; i < nu; ++i)
    for (int j = 1; j < nv; ++j) {
      const Vec3 duv = at(i, j) - at(i - 1, j) - at(i, j - 1) + at(i - 1, j - 1);
      e += 2.0 * normSquared(duv);
    }
  return e;
}

// ─────────────────────────────────────────────────────────────────────────────
// Curve fairing.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Build the free-set + the second-difference Gram matrix K = LᵀL restricted to the
// free poles, plus the constant RHS contributions from fixed neighbours (which for
// the *penalized* objective are folded into λ·K·P_fixed on the RHS). We keep the
// full second-difference operator L (rows = interior indices, cols = all poles) and
// project onto the free columns.
//
// Compute λ-independent pieces (K and the free P⁰), plus the fixed-pole coupling
// matrix Cfix (m×#fixed of the Gram) so the RHS at each λ is P⁰|free − λ·Cfix·P⁰|fixed.
struct CurveGram {
  FairSystem fs;
  std::vector<double> K;                // m×m free-free Gram
  std::vector<double> Cfix;             // m×f free-fixed Gram (row-major, f columns)
  std::vector<int> fixedIdx;            // fixed pole indices
};

CurveGram buildCurveGram(const BsplineCurveData& c, bool keepEnds) {
  const int n = static_cast<int>(c.poles.size());
  CurveGram g;
  // Fixed set: endpoints always; with keepEnds also the second pole from each end
  // (which pins the end tangent for a clamped B-spline).
  std::vector<char> fixed(n, 0);
  fixed[0] = fixed[n - 1] = 1;
  if (keepEnds && n >= 4) { fixed[1] = 1; fixed[n - 2] = 1; }

  g.fs.slot.assign(n, -1);
  for (int i = 0; i < n; ++i) {
    if (fixed[i]) g.fixedIdx.push_back(i);
    else { g.fs.slot[i] = g.fs.m; g.fs.freeIdx.push_back(i); ++g.fs.m; }
  }
  const int m = g.fs.m;
  const int f = static_cast<int>(g.fixedIdx.size());
  g.K.assign(static_cast<std::size_t>(m) * m, 0.0);
  g.Cfix.assign(static_cast<std::size_t>(m) * f, 0.0);

  // fixed pole index → column in Cfix
  std::vector<int> fixCol(n, -1);
  for (int t = 0; t < f; ++t) fixCol[g.fixedIdx[t]] = t;

  // Each interior index i contributes a second-difference row with stencil weights
  // (+1, −2, +1) at columns (i−1, i, i+1). Accumulate its outer product into the
  // free-free (K) and free-fixed (Cfix) blocks.
  for (int i = 1; i + 1 < n; ++i) {
    const int col[3] = {i - 1, i, i + 1};
    const double w[3] = {1.0, -2.0, 1.0};
    for (int a = 0; a < 3; ++a) {
      const int ca = col[a];
      const int sa = g.fs.slot[ca];
      if (sa < 0) continue;  // fixed row-source: contributes only to Cfix (below)
      for (int b = 0; b < 3; ++b) {
        const int cb = col[b];
        const double wab = w[a] * w[b];
        const int sb = g.fs.slot[cb];
        if (sb >= 0) g.K[static_cast<std::size_t>(sa) * m + sb] += wab;
        else g.Cfix[static_cast<std::size_t>(sa) * f + fixCol[cb]] += wab;
      }
    }
  }
  return g;
}

}  // namespace

CurveFairResult fairCurve(const BsplineCurveData& curve, double tol, bool keepEnds) {
  CurveFairResult r;
  r.curve = curve;
  r.energyBefore = curveEnergy(curve.poles);
  r.energyAfter = r.energyBefore;
  r.maxDeviation = 0.0;

  const int n = static_cast<int>(curve.poles.size());
  // Malformed / too small / rational → honest decline (unchanged input).
  if (curve.degree < 1 || n < 3) return r;
  if (!curve.weights.empty()) return r;  // rational: homogeneous fairing is a residual
  if (static_cast<int>(curve.knots.size()) != n + curve.degree + 1) return r;
  if (!(tol > 0.0)) return r;
  // Already fair: an affine net has energy 0 (up to fp round-off, ~1e-28·scale).
  // Treat a negligible relative energy as a no-op success — nothing to remove.
  const double scale2 = netScaleSquared(curve.poles);
  if (r.energyBefore <= 1e-18 * std::max(scale2, 1.0)) {
    r.ok = true;
    return r;
  }

  CurveGram g = buildCurveGram(curve, keepEnds);
  const int m = g.fs.m;
  const int f = static_cast<int>(g.fixedIdx.size());
  if (m == 0) return r;  // nothing free to move → decline

  // Free P⁰ and the fixed-pole coordinate vectors (for the RHS coupling term).
  std::vector<double> p0x(m), p0y(m), p0z(m);
  for (int s = 0; s < m; ++s) {
    const Point3& P = curve.poles[g.fs.freeIdx[s]];
    p0x[s] = P.x; p0y[s] = P.y; p0z[s] = P.z;
  }
  std::vector<double> fx(f), fy(f), fz(f);
  for (int t = 0; t < f; ++t) {
    const Point3& P = curve.poles[g.fixedIdx[t]];
    fx[t] = P.x; fy[t] = P.y; fz[t] = P.z;
  }

  // Sweep λ geometrically; keep the LARGEST λ whose faired curve honors tol AND
  // strictly reduces energy. RHS(λ) = P⁰|free − λ·Cfix·P⁰|fixed.
  auto rhsFor = [&](const std::vector<double>& p0f, const std::vector<double>& pf,
                    double lambda) {
    std::vector<double> b = p0f;
    for (int s = 0; s < m; ++s) {
      double acc = 0.0;
      for (int t = 0; t < f; ++t) acc += g.Cfix[static_cast<std::size_t>(s) * f + t] * pf[t];
      b[s] -= lambda * acc;
    }
    return b;
  };

  BsplineCurveData best = curve;
  double bestEnergy = r.energyBefore;
  double bestDev = 0.0;
  bool improved = false;

  for (double lambda = 1e-3; lambda <= 1e9; lambda *= 3.0) {
    const std::vector<double> bx = rhsFor(p0x, fx, lambda);
    const std::vector<double> by = rhsFor(p0y, fy, lambda);
    const std::vector<double> bz = rhsFor(p0z, fz, lambda);
    const std::vector<Point3> P = solveCurveLambda(curve.poles, g.fs, g.K, bx, by, bz, lambda);
    if (P.empty()) break;  // singular system

    BsplineCurveData cand = curve;
    cand.poles = P;
    const double dev = curveDeviation(cand, curve);
    if (dev > tol) break;  // this λ (and all larger) exceed the HARD bound — stop
    const double e = curveEnergy(P);
    if (e < bestEnergy - 1e-15) {
      best = cand;
      bestEnergy = e;
      bestDev = dev;
      improved = true;
    }
  }

  if (!improved) return r;  // cannot reduce energy within tol → honest decline

  r.curve = best;
  r.energyAfter = bestEnergy;
  r.maxDeviation = bestDev;
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface fairing.
//
// The thin-plate energy is a sum of squared second differences over the 2-D
// control net. Fairing minimizes ‖P − P⁰‖² + λ·E(P); its stationary point is the
// linear system (I + λ·K) P = P⁰ over the FREE (non-boundary) poles, where K is the
// Gram of ALL second-difference stencils (U-uu, V-vv, and the mixed U-V). We
// flatten the free poles into one unknown vector, assemble the sparse-but-dense
// Gram, and solve per coordinate through lin_solve, sweeping λ exactly as the curve.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

struct SurfGram {
  FairSystem fs;                // free flattened over the (nu*nv) net
  std::vector<double> K;        // m×m free-free Gram
  std::vector<double> Cfix;     // m×f free-fixed Gram
  std::vector<int> fixedIdx;    // fixed flat indices
  int nu = 0, nv = 0;
};

// Add a stencil (list of (flatIndex, weight)) as an outer product into K/Cfix.
void addStencil(SurfGram& g, int m, int f, const std::vector<int>& fixCol,
                const int* idx, const double* w, int k) {
  for (int a = 0; a < k; ++a) {
    const int sa = g.fs.slot[idx[a]];
    if (sa < 0) continue;
    for (int b = 0; b < k; ++b) {
      const double wab = w[a] * w[b];
      const int sb = g.fs.slot[idx[b]];
      if (sb >= 0) g.K[static_cast<std::size_t>(sa) * m + sb] += wab;
      else g.Cfix[static_cast<std::size_t>(sa) * f + fixCol[idx[b]]] += wab;
    }
  }
}

SurfGram buildSurfGram(const BsplineSurfaceData& s, bool keepBoundary) {
  const int nu = s.nPolesU, nv = s.nPolesV;
  const int N = nu * nv;
  SurfGram g;
  g.nu = nu; g.nv = nv;
  auto flat = [&](int i, int j) { return i * nv + j; };

  std::vector<char> fixed(N, 0);
  if (keepBoundary) {
    for (int i = 0; i < nu; ++i) { fixed[flat(i, 0)] = 1; fixed[flat(i, nv - 1)] = 1; }
    for (int j = 0; j < nv; ++j) { fixed[flat(0, j)] = 1; fixed[flat(nu - 1, j)] = 1; }
  } else {  // only the four corners
    fixed[flat(0, 0)] = fixed[flat(0, nv - 1)] = 1;
    fixed[flat(nu - 1, 0)] = fixed[flat(nu - 1, nv - 1)] = 1;
  }

  g.fs.slot.assign(N, -1);
  for (int idx = 0; idx < N; ++idx) {
    if (fixed[idx]) g.fixedIdx.push_back(idx);
    else { g.fs.slot[idx] = g.fs.m; g.fs.freeIdx.push_back(idx); ++g.fs.m; }
  }
  const int m = g.fs.m;
  const int f = static_cast<int>(g.fixedIdx.size());
  g.K.assign(static_cast<std::size_t>(m) * m, 0.0);
  g.Cfix.assign(static_cast<std::size_t>(m) * f, 0.0);
  std::vector<int> fixCol(N, -1);
  for (int t = 0; t < f; ++t) fixCol[g.fixedIdx[t]] = t;

  // |S_uu|²: (+1,−2,+1) along U.
  for (int i = 1; i + 1 < nu; ++i)
    for (int j = 0; j < nv; ++j) {
      const int idx[3] = {flat(i - 1, j), flat(i, j), flat(i + 1, j)};
      const double w[3] = {1.0, -2.0, 1.0};
      addStencil(g, m, f, fixCol, idx, w, 3);
    }
  // |S_vv|²: (+1,−2,+1) along V.
  for (int i = 0; i < nu; ++i)
    for (int j = 1; j + 1 < nv; ++j) {
      const int idx[3] = {flat(i, j - 1), flat(i, j), flat(i, j + 1)};
      const double w[3] = {1.0, -2.0, 1.0};
      addStencil(g, m, f, fixCol, idx, w, 3);
    }
  // 2·|S_uv|²: mixed (+1,−1,−1,+1) on the 2×2 stencil, weighted by √2 so the outer
  // product contributes 2× to the energy (w·w = 2 per matched term).
  for (int i = 1; i < nu; ++i)
    for (int j = 1; j < nv; ++j) {
      const int idx[4] = {flat(i, j), flat(i - 1, j), flat(i, j - 1), flat(i - 1, j - 1)};
      const double s2 = std::sqrt(2.0);
      const double w[4] = {s2, -s2, -s2, s2};
      addStencil(g, m, f, fixCol, idx, w, 4);
    }
  return g;
}

// Max deviation of the faired surface from the input over a dense (u,v) grid.
double surfaceDeviation(const BsplineSurfaceData& faired, const BsplineSurfaceData& input) {
  const int pu = input.degreeU, pv = input.degreeV;
  const int nu = input.nPolesU, nv = input.nPolesV;
  const double u0 = input.knotsU[pu], u1 = input.knotsU[static_cast<std::size_t>(nu)];
  const double v0 = input.knotsV[pv], v1 = input.knotsV[static_cast<std::size_t>(nv)];
  SurfaceGrid gf{faired.poles, nu, nv};
  SurfaceGrid gi{input.poles, nu, nv};
  const int su = 6 * nu + 12, sv = 6 * nv + 12;
  double maxDev = 0.0;
  for (int a = 0; a <= su; ++a) {
    const double u = u0 + (u1 - u0) * (static_cast<double>(a) / su);
    for (int b = 0; b <= sv; ++b) {
      const double v = v0 + (v1 - v0) * (static_cast<double>(b) / sv);
      const Point3 pf = surfacePoint(pu, pv, gf, input.knotsU, input.knotsV, u, v);
      const Point3 pi = surfacePoint(pu, pv, gi, input.knotsU, input.knotsV, u, v);
      maxDev = std::max(maxDev, distance(pf, pi));
    }
  }
  return maxDev;
}

}  // namespace

SurfaceFairResult fairSurface(const BsplineSurfaceData& surface, double tol,
                              bool keepBoundary) {
  SurfaceFairResult r;
  r.surface = surface;
  r.energyBefore = surfaceBendingEnergy(surface);
  r.energyAfter = r.energyBefore;
  r.maxDeviation = 0.0;

  const int nu = surface.nPolesU, nv = surface.nPolesV;
  const int N = nu * nv;
  if (surface.degreeU < 1 || surface.degreeV < 1 || nu < 3 || nv < 3) return r;
  if (static_cast<int>(surface.poles.size()) != N) return r;
  if (!surface.weights.empty()) return r;  // rational: residual
  if (static_cast<int>(surface.knotsU.size()) != nu + surface.degreeU + 1) return r;
  if (static_cast<int>(surface.knotsV.size()) != nv + surface.degreeV + 1) return r;
  if (!(tol > 0.0)) return r;
  const double scale2 = netScaleSquared(surface.poles);
  if (r.energyBefore <= 1e-18 * std::max(scale2, 1.0)) { r.ok = true; return r; }  // already fair

  SurfGram g = buildSurfGram(surface, keepBoundary);
  const int m = g.fs.m;
  const int f = static_cast<int>(g.fixedIdx.size());
  if (m == 0) return r;

  std::vector<double> p0x(m), p0y(m), p0z(m);
  for (int s = 0; s < m; ++s) {
    const Point3& P = surface.poles[g.fs.freeIdx[s]];
    p0x[s] = P.x; p0y[s] = P.y; p0z[s] = P.z;
  }
  std::vector<double> fx(f), fy(f), fz(f);
  for (int t = 0; t < f; ++t) {
    const Point3& P = surface.poles[g.fixedIdx[t]];
    fx[t] = P.x; fy[t] = P.y; fz[t] = P.z;
  }

  auto rhsFor = [&](const std::vector<double>& p0f, const std::vector<double>& pf,
                    double lambda) {
    std::vector<double> b = p0f;
    for (int s = 0; s < m; ++s) {
      double acc = 0.0;
      for (int t = 0; t < f; ++t) acc += g.Cfix[static_cast<std::size_t>(s) * f + t] * pf[t];
      b[s] -= lambda * acc;
    }
    return b;
  };

  auto solveLambda = [&](double lambda) -> std::vector<Point3> {
    std::vector<double> A(static_cast<std::size_t>(m) * m);
    for (int i = 0; i < m; ++i)
      for (int j = 0; j < m; ++j)
        A[static_cast<std::size_t>(i) * m + j] =
            (i == j ? 1.0 : 0.0) + lambda * g.K[static_cast<std::size_t>(i) * m + j];
    const std::vector<double> cx = lin_solve(A, m, rhsFor(p0x, fx, lambda));
    const std::vector<double> cy = lin_solve(A, m, rhsFor(p0y, fy, lambda));
    const std::vector<double> cz = lin_solve(A, m, rhsFor(p0z, fz, lambda));
    if (static_cast<int>(cx.size()) != m) return {};
    std::vector<Point3> P = surface.poles;
    for (int s = 0; s < m; ++s) P[g.fs.freeIdx[s]] = {cx[s], cy[s], cz[s]};
    return P;
  };

  BsplineSurfaceData best = surface;
  double bestEnergy = r.energyBefore;
  double bestDev = 0.0;
  bool improved = false;

  for (double lambda = 1e-3; lambda <= 1e9; lambda *= 3.0) {
    const std::vector<Point3> P = solveLambda(lambda);
    if (P.empty()) break;
    BsplineSurfaceData cand = surface;
    cand.poles = P;
    const double dev = surfaceDeviation(cand, surface);
    if (dev > tol) break;
    const double e = surfaceBendingEnergy(cand);
    if (e < bestEnergy - 1e-15) {
      best = cand; bestEnergy = e; bestDev = dev; improved = true;
    }
  }

  if (!improved) return r;
  r.surface = best;
  r.energyAfter = bestEnergy;
  r.maxDeviation = bestDev;
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

// SPDX-License-Identifier: Apache-2.0
//
// bspline_fit.cpp — NURBS roadmap Layer 7 (fitting / approximation) implementation.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), Chapter 9. The
// parameter-assignment and knot-generation helpers are substrate-free; the fitting
// routines solve linear systems through the numsci facade, so the WHOLE file is
// under CYBERCAD_HAS_NUMSCI (mirroring src/native/ssi/marching.cpp). With the guard
// OFF this TU is inert and the Layer-7 functions are simply absent from the library.
//
#include "native/math/bspline_fit.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint / findSpan / basisFuns / surfacePoint
#include "native/numerics/numerics.h"   // lin_solve (square) / lstsq (least-squares)

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

namespace cybercad::native::math {
namespace {

using numerics::lin_solve;
using numerics::lstsq;

// One clamped knot at each end, degree+1 multiplicity, is the convention shared
// with the evaluators / Layer-1 ops (flat knots, length nPoles+degree+1).

// ── Parameter assignment (Eq 9.3/9.4/9.6) ────────────────────────────────────
// chord-length exponent 1.0, centripetal exponent 0.5, uniform ignores geometry.
std::vector<double> paramsImpl(std::span<const Point3> pts, ParamMethod method) {
  const int n = static_cast<int>(pts.size());
  if (n < 2) return {};

  if (method == ParamMethod::Uniform) {
    std::vector<double> u(n);
    for (int k = 0; k < n; ++k) u[k] = static_cast<double>(k) / (n - 1);
    return u;
  }

  const double exponent = (method == ParamMethod::Centripetal) ? 0.5 : 1.0;

  // Per-segment (possibly exponentiated) chord lengths and their total.
  std::vector<double> seg(n - 1);
  double total = 0.0;
  for (int k = 1; k < n; ++k) {
    double d = distance(pts[k], pts[k - 1]);
    if (exponent != 1.0) d = std::pow(d, exponent);
    seg[k - 1] = d;
    total += d;
  }
  if (!(total > 0.0)) return {};  // all points coincident — no length to normalize

  std::vector<double> u(n);
  u[0] = 0.0;
  double acc = 0.0;
  for (int k = 1; k < n; ++k) {
    acc += seg[k - 1];
    u[k] = acc / total;
  }
  u[n - 1] = 1.0;  // pin the end exactly (guard fp drift)
  return u;
}

// ── Averaging knots for interpolation (Eq 9.8) ────────────────────────────────
std::vector<double> avgKnots(std::span<const double> u, int p) {
  const int n = static_cast<int>(u.size()) - 1;   // last data index
  const int m = n + p + 1;                         // last knot index
  std::vector<double> U(m + 1, 0.0);
  for (int i = m - p; i <= m; ++i) U[i] = 1.0;     // clamped tail
  for (int j = 1; j <= n - p; ++j) {               // interior averages
    double s = 0.0;
    for (int i = j; i <= j + p - 1; ++i) s += u[i];
    U[j + p] = s / p;
  }
  return U;
}

// ── Knots for least-squares approximation (Eq 9.68/9.69) ──────────────────────
// nCtrl control points (indices 0..h with h = nCtrl-1), n+1 data points.
std::vector<double> approxKnotsImpl(std::span<const double> u, int p, int nCtrl) {
  const int n = static_cast<int>(u.size()) - 1;
  const int h = nCtrl - 1;                          // last control index
  const int m = nCtrl + p;                          // last knot index
  std::vector<double> U(m + 1, 0.0);
  for (int i = m - p; i <= m; ++i) U[i] = 1.0;      // clamped tail
  const double d = static_cast<double>(n + 1) / (h - p + 1);
  for (int j = 1; j <= h - p; ++j) {               // Eq 9.69 spread
    const int i = static_cast<int>(j * d);
    const double alpha = j * d - i;
    U[p + j] = (1.0 - alpha) * u[i - 1] + alpha * u[i];
  }
  return U;
}

// Evaluate the max / RMS deviation of a fitted curve from its data at (uₖ,Qₖ).
void curveErrors(const BsplineCurveData& c, std::span<const Point3> pts,
                 std::span<const double> u, double& maxErr, double& rmsErr) {
  maxErr = 0.0;
  double sumSq = 0.0;
  const int n = static_cast<int>(pts.size());
  for (int k = 0; k < n; ++k) {
    const Point3 p = curvePoint(c.degree, c.poles, c.knots, u[k]);
    const double e = distance(p, pts[k]);
    maxErr = std::max(maxErr, e);
    sumSq += e * e;
  }
  rmsErr = (n > 0) ? std::sqrt(sumSq / n) : 0.0;
}

// Solve the interpolation collocation system for one axis (x/y/z chosen by `pick`).
// A is the (n+1)×(n+1) row-major basis matrix; returns ctrl-point coords or empty.
std::vector<double> solveAxis(const std::vector<double>& A, int dim,
                              std::span<const Point3> pts,
                              double (*pick)(const Point3&)) {
  std::vector<double> b(dim);
  for (int i = 0; i < dim; ++i) b[i] = pick(pts[i]);
  return lin_solve(A, dim, b);
}

double px(const Point3& p) { return p.x; }
double py(const Point3& p) { return p.y; }
double pz(const Point3& p) { return p.z; }

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Public parametrization / knot generation.
// ─────────────────────────────────────────────────────────────────────────────

std::vector<double> assignParams(std::span<const Point3> points, ParamMethod method) {
  return paramsImpl(points, method);
}

std::vector<double> averagingKnots(std::span<const double> params, int degree) {
  if (static_cast<int>(params.size()) < degree + 1) return {};
  return avgKnots(params, degree);
}

std::vector<double> approxKnots(std::span<const double> params, int degree, int nCtrl) {
  const int nData = static_cast<int>(params.size());
  if (nCtrl < degree + 1 || nCtrl > nData) return {};
  return approxKnotsImpl(params, degree, nCtrl);
}

// ─────────────────────────────────────────────────────────────────────────────
// Curve interpolation (A9.1).
// ─────────────────────────────────────────────────────────────────────────────

CurveFitResult interpolateCurve(std::span<const Point3> points, int degree,
                                ParamMethod method) {
  CurveFitResult r;
  const int n = static_cast<int>(points.size());
  if (degree < 1 || n < degree + 1) return r;  // need ≥ p+1 points

  const std::vector<double> u = paramsImpl(points, method);
  if (static_cast<int>(u.size()) != n) return r;  // degenerate (all-coincident) input

  const std::vector<double> U = avgKnots(u, degree);

  // Build the (n)×(n) collocation matrix A(k,i) = N_{i,p}(u_k), row-major.
  const int lastPole = n - 1;
  std::vector<double> A(static_cast<std::size_t>(n) * n, 0.0);
  std::vector<double> N(degree + 1);
  for (int k = 0; k < n; ++k) {
    const int span = findSpan(lastPole, degree, u[k], U);
    basisFuns(span, u[k], degree, U, N);
    for (int j = 0; j <= degree; ++j)
      A[static_cast<std::size_t>(k) * n + (span - degree + j)] = N[j];
  }

  // Solve the same matrix for each coordinate (three RHS).
  const std::vector<double> cx = solveAxis(A, n, points, px);
  const std::vector<double> cy = solveAxis(A, n, points, py);
  const std::vector<double> cz = solveAxis(A, n, points, pz);
  if (static_cast<int>(cx.size()) != n || static_cast<int>(cy.size()) != n ||
      static_cast<int>(cz.size()) != n)
    return r;  // singular collocation (should not happen for valid params)

  r.curve.degree = degree;
  r.curve.knots = U;
  r.curve.poles.resize(n);
  for (int i = 0; i < n; ++i) r.curve.poles[i] = {cx[i], cy[i], cz[i]};

  curveErrors(r.curve, points, u, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational curve interpolation with prescribed weights.
//
// Lift each datum Qₖ (weight wₖ) to the homogeneous point Qʷₖ = (wₖQₖ, wₖ) ∈ R⁴,
// solve the SAME averaging-knot collocation matrix as interpolateCurve for all four
// homogeneous coordinates (x,y,z, and the weight coord), then project each solved
// control point Pʷᵢ=(Xᵢ,Yᵢ,Zᵢ,Wᵢ) back to Pᵢ=(Xᵢ/Wᵢ,Yᵢ/Wᵢ,Zᵢ/Wᵢ), weightᵢ=Wᵢ.
// Since Cʷ(uₖ)=Qʷₖ, the projected rational curve passes through Qₖ exactly.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Max / RMS deviation of a RATIONAL fitted curve from its data at (uₖ,Qₖ).
void rationalCurveErrors(const BsplineCurveData& c, std::span<const Point3> pts,
                         std::span<const double> u, double& maxErr, double& rmsErr) {
  maxErr = 0.0;
  double sumSq = 0.0;
  const int n = static_cast<int>(pts.size());
  for (int k = 0; k < n; ++k) {
    const Point3 p = nurbsCurvePoint(c.degree, c.poles, c.weights, c.knots, u[k]);
    const double e = distance(p, pts[k]);
    maxErr = std::max(maxErr, e);
    sumSq += e * e;
  }
  rmsErr = (n > 0) ? std::sqrt(sumSq / n) : 0.0;
}

}  // namespace

CurveFitResult interpolateRationalCurve(std::span<const Point3> points,
                                        std::span<const double> weights, int degree,
                                        ParamMethod method) {
  CurveFitResult r;
  const int n = static_cast<int>(points.size());
  if (degree < 1 || n < degree + 1) return r;                 // need ≥ p+1 points
  if (static_cast<int>(weights.size()) != n) return r;        // one weight per point
  for (int k = 0; k < n; ++k)
    if (!(weights[k] > 0.0)) return r;                        // non-positive weight guard

  const std::vector<double> u = paramsImpl(points, method);
  if (static_cast<int>(u.size()) != n) return r;              // degenerate (all-coincident)

  const std::vector<double> U = avgKnots(u, degree);

  // Same (n)×(n) collocation matrix A(k,i) = N_{i,p}(u_k) as the non-rational case.
  const int lastPole = n - 1;
  std::vector<double> A(static_cast<std::size_t>(n) * n, 0.0);
  std::vector<double> N(degree + 1);
  for (int k = 0; k < n; ++k) {
    const int span = findSpan(lastPole, degree, u[k], U);
    basisFuns(span, u[k], degree, U, N);
    for (int j = 0; j <= degree; ++j)
      A[static_cast<std::size_t>(k) * n + (span - degree + j)] = N[j];
  }

  // Homogeneous RHS: Qʷₖ = (wₖ·xₖ, wₖ·yₖ, wₖ·zₖ, wₖ). Four solves, one matrix.
  std::vector<double> bx(n), by(n), bz(n), bw(n);
  for (int k = 0; k < n; ++k) {
    const double w = weights[k];
    bx[k] = w * points[k].x;
    by[k] = w * points[k].y;
    bz[k] = w * points[k].z;
    bw[k] = w;
  }
  const std::vector<double> cx = lin_solve(A, n, bx);
  const std::vector<double> cy = lin_solve(A, n, by);
  const std::vector<double> cz = lin_solve(A, n, bz);
  const std::vector<double> cw = lin_solve(A, n, bw);
  if (static_cast<int>(cx.size()) != n || static_cast<int>(cy.size()) != n ||
      static_cast<int>(cz.size()) != n || static_cast<int>(cw.size()) != n)
    return r;  // singular collocation (should not happen for valid params)

  // Project the homogeneous net back; a solved non-positive weight is a hard guard.
  r.curve.degree = degree;
  r.curve.knots = U;
  r.curve.poles.resize(n);
  r.curve.weights.resize(n);
  for (int i = 0; i < n; ++i) {
    if (!(cw[i] > 0.0)) return CurveFitResult{};  // projected weight ≤ 0 — decline
    r.curve.poles[i] = {cx[i] / cw[i], cy[i] / cw[i], cz[i] / cw[i]};
    r.curve.weights[i] = cw[i];
  }

  rationalCurveErrors(r.curve, points, u, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational weight ESTIMATION (Ma–Kruth) — recover control points AND weights from
// UNWEIGHTED data. The rational fit C(uₖ)=Qₖ is BILINEAR: writing the weighted
// control points Hᵢ = wᵢ·Pᵢ, the condition Σᵢ Nᵢ(uₖ)·wᵢ·(Pᵢ − Qₖ)=0 becomes
//   Σᵢ Nᵢ(uₖ)·Hᵢ − Qₖ·(Σᵢ Nᵢ(uₖ)·wᵢ) = 0
// which is LINEAR and HOMOGENEOUS in the unknown z = (H₀..Hₘ, w₀..wₘ). Each datum
// contributes 3 scalar rows (x/y/z), each coupling the H-block of that axis with the
// shared w-block. Over-determined (3·nData > 4·nCtrl) ⇒ the fit is the 1-D null
// space of M, found as the smallest eigenvector of MᵀM by shifted inverse iteration
// through lin_solve (no external SVD). Then Pᵢ=Hᵢ/wᵢ, gauge-normalized to w₀=1.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Symmetric-matrix vector product y = A·x, A row-major size×size.
std::vector<double> matVec(const std::vector<double>& A, int size,
                           const std::vector<double>& x) {
  std::vector<double> y(size, 0.0);
  for (int i = 0; i < size; ++i) {
    double s = 0.0;
    const double* row = &A[static_cast<std::size_t>(i) * size];
    for (int j = 0; j < size; ++j) s += row[j] * x[j];
    y[i] = s;
  }
  return y;
}

double dot(const std::vector<double>& a, const std::vector<double>& b) {
  double s = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) s += a[i] * b[i];
  return s;
}

// Smallest-eigenvalue eigenvector of the symmetric PSD matrix A (size×size) by
// shifted inverse iteration: repeatedly solve (A + shift·I)·y = z, normalize.
// `deflate` (optional, unit-norm) is projected out each step so a second call finds
// the SECOND-smallest eigenvector (for the null-space-gap rank test). Returns the
// eigenvector and, in `outEig`, its Rayleigh quotient zᵀAz (the eigenvalue).
std::vector<double> smallestEigvec(const std::vector<double>& A, int size, double shift,
                                   const std::vector<double>* deflate, double& outEig) {
  std::vector<double> shifted = A;
  for (int i = 0; i < size; ++i) shifted[static_cast<std::size_t>(i) * size + i] += shift;

  // Deterministic non-degenerate start.
  std::vector<double> z(size);
  for (int i = 0; i < size; ++i) z[i] = 1.0 + 0.1 * std::sin(0.3 * i + 1.0);
  auto normalize = [](std::vector<double>& v) {
    double nrm = std::sqrt(dot(v, v));
    if (nrm > 0.0) for (double& e : v) e /= nrm;
    return nrm;
  };
  auto orthoDeflate = [&](std::vector<double>& v) {
    if (deflate) {
      const double c = dot(v, *deflate);
      for (int i = 0; i < size; ++i) v[i] -= c * (*deflate)[i];
    }
  };
  orthoDeflate(z);
  normalize(z);

  for (int it = 0; it < 200; ++it) {
    std::vector<double> y = lin_solve(shifted, size, z);
    if (static_cast<int>(y.size()) != size) { outEig = -1.0; return {}; }
    orthoDeflate(y);
    if (normalize(y) == 0.0) { outEig = -1.0; return {}; }
    const double change = std::fabs(std::fabs(dot(y, z)) - 1.0);
    z.swap(y);
    if (change < 1e-14) break;
  }
  outEig = dot(z, matVec(A, size, z));  // Rayleigh quotient (unshifted eigenvalue)
  return z;
}

// Assemble MᵀM (size 4·nCtrl) for the homogeneous weight-estimation system over the
// given data/params/knots. Unknown layout: [Hx | Hy | Hz | w], each block nCtrl long.
std::vector<double> buildWeightEstNormalMatrix(std::span<const Point3> points,
                                               std::span<const double> u,
                                               std::span<const double> U, int nCtrl,
                                               int degree) {
  const int nUnknown = 4 * nCtrl;
  const int W0 = 3 * nCtrl;  // weight-block offset
  const int lastPole = nCtrl - 1;
  std::vector<double> AtA(static_cast<std::size_t>(nUnknown) * nUnknown, 0.0);
  std::vector<double> N(degree + 1);
  std::vector<std::pair<int, double>> terms;
  terms.reserve(2 * (degree + 1));
  const int nData = static_cast<int>(points.size());
  for (int k = 0; k < nData; ++k) {
    const int span = findSpan(lastPole, degree, u[k], U);
    basisFuns(span, u[k], degree, U, N);
    const double qd[3] = {points[k].x, points[k].y, points[k].z};
    for (int axis = 0; axis < 3; ++axis) {
      // Row for axis d: +Nᵢ at H_d[i], −Qₖ_d·Nᵢ at w[i]. MᵀM += row ⊗ row.
      terms.clear();
      for (int j = 0; j <= degree; ++j) {
        const int i = span - degree + j;
        terms.emplace_back(axis * nCtrl + i, N[j]);
        terms.emplace_back(W0 + i, -qd[axis] * N[j]);
      }
      for (const auto& [ci, cv] : terms)
        for (const auto& [cj, cw] : terms)
          AtA[static_cast<std::size_t>(ci) * nUnknown + cj] += cv * cw;
    }
  }
  return AtA;
}

}  // namespace

RationalFitResult fitRationalCurveEstimateWeightsWithParams(std::span<const Point3> points,
                                                           std::span<const double> params,
                                                           std::span<const double> knots,
                                                           int nCtrl, int degree,
                                                           double flatTol) {
  RationalFitResult r;
  const int nData = static_cast<int>(points.size());
  if (degree < 1 || nCtrl < degree + 1 || nData < nCtrl) {
    r.diagnostic = "invalid dimensions (need degree+1 <= nCtrl <= nData)";
    return r;
  }
  if (static_cast<int>(params.size()) != nData) {
    r.diagnostic = "params.size() must equal points.size()";
    return r;
  }
  if (static_cast<int>(knots.size()) != nCtrl + degree + 1) {
    r.diagnostic = "knots.size() must equal nCtrl+degree+1";
    return r;
  }
  // Over-determination: 3 rows/datum, 4 unknowns/control point. Need a clean 1-D
  // null space, so demand strictly more rows than unknowns.
  const int nUnknown = 4 * nCtrl;   // Hx,Hy,Hz,w per control point
  if (3 * nData <= nUnknown) {
    r.diagnostic = "under-determined: need 3*nData > 4*nCtrl to pin the weights";
    return r;
  }

  const std::span<const double> u = params;
  const std::span<const double> U = knots;
  const int W0 = 3 * nCtrl;  // offset of the weight block in the unknown vector

  // Assemble MᵀM for the homogeneous bilinear system (see comment block above).
  const std::vector<double> AtA = buildWeightEstNormalMatrix(points, u, U, nCtrl, degree);

  // Scale-invariant shift for inverse iteration (trace-relative).
  double trace = 0.0;
  for (int i = 0; i < nUnknown; ++i) trace += AtA[static_cast<std::size_t>(i) * nUnknown + i];
  const double shift = (trace > 0.0 ? trace / nUnknown : 1.0) * 1e-12;

  double eig0 = 0.0, eig1 = 0.0;
  std::vector<double> z = smallestEigvec(AtA, nUnknown, shift, nullptr, eig0);
  if (static_cast<int>(z.size()) != nUnknown) {
    r.diagnostic = "eigen solve failed (singular shifted system)";
    return r;
  }
  const std::vector<double> z1 = smallestEigvec(AtA, nUnknown, shift, &z, eig1);
  if (static_cast<int>(z1.size()) == nUnknown) {
    // Rank test: the smallest eigenvalue must be well SEPARATED from the next — a
    // clean 1-D null space means eig0 ≪ eig1 (large relative GAP). The absolute eig0
    // is NOT required to be machine-zero: the chord-length parametrization differs
    // from the datum's projective parameter, so an exact conic still leaves a small
    // fit residual (eig0 ~ 1e-6·trace). A comparable eig1 (small gap) means the null
    // space is not one-dimensional (rank-deficient) — weights not unique, decline.
    const double gap = eig1 / std::max(eig0, 1e-300);
    if (!(gap > 50.0)) {
      r.diagnostic = "rank-deficient: null space not one-dimensional (weights not unique)";
      return r;
    }
  }

  // Extract weights; enforce a single shared sign (flip the whole vector if needed).
  std::vector<double> w(nCtrl);
  double sumW = 0.0;
  for (int i = 0; i < nCtrl; ++i) { w[i] = z[W0 + i]; sumW += w[i]; }
  if (sumW < 0.0) for (double& e : z) e = -e;   // gauge the shared sign to +
  for (int i = 0; i < nCtrl; ++i) w[i] = z[W0 + i];

  // Sign-flip guard: a valid rational needs all weights strictly the SAME sign.
  double wmin = w[0], wmax = w[0];
  for (int i = 0; i < nCtrl; ++i) { wmin = std::min(wmin, w[i]); wmax = std::max(wmax, w[i]); }
  const double wref = std::fabs(w[0]) > 0.0 ? std::fabs(w[0]) : 1.0;
  if (!(wmin > 1e-9 * wref)) {
    r.diagnostic = "sign-flipping / near-zero weight recovered (invalid rational)";
    return r;
  }

  // De-homogenize: Pᵢ = Hᵢ/wᵢ, then normalize weights to the w₀ = 1 gauge.
  const double g = w[0];
  r.curve.degree = degree;
  r.curve.knots.assign(U.begin(), U.end());
  r.curve.poles.resize(nCtrl);
  r.curve.weights.resize(nCtrl);
  for (int i = 0; i < nCtrl; ++i) {
    const double wi = w[i];
    r.curve.poles[i] = {z[i] / wi, z[nCtrl + i] / wi, z[2 * nCtrl + i] / wi};
    r.curve.weights[i] = wi / g;  // gauge w₀ = 1
  }

  // Weight spread after the gauge fix — the non-rationality detector.
  double gmin = r.curve.weights[0], gmax = r.curve.weights[0];
  for (double wi : r.curve.weights) { gmin = std::min(gmin, wi); gmax = std::max(gmax, wi); }
  r.weightSpread = gmax - gmin;
  r.rationalityDetected = (r.weightSpread > flatTol);

  rationalCurveErrors(r.curve, points, u, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

RationalFitResult fitRationalCurveEstimateWeights(std::span<const Point3> points, int nCtrl,
                                                  int degree, ParamMethod method,
                                                  double flatTol) {
  RationalFitResult r;
  const int nData = static_cast<int>(points.size());
  if (degree < 1 || nCtrl < degree + 1 || nData < nCtrl) {
    r.diagnostic = "invalid dimensions (need degree+1 <= nCtrl <= nData)";
    return r;
  }
  const std::vector<double> u = paramsImpl(points, method);
  if (static_cast<int>(u.size()) != nData) {
    r.diagnostic = "degenerate parametrization (all-coincident points)";
    return r;
  }
  const std::vector<double> U = approxKnotsImpl(u, degree, nCtrl);
  return fitRationalCurveEstimateWeightsWithParams(points, u, U, nCtrl, degree, flatTol);
}

// ─────────────────────────────────────────────────────────────────────────────
// Curve least-squares approximation (A9.4/9.6), endpoints pinned.
// ─────────────────────────────────────────────────────────────────────────────

CurveFitResult approximateCurve(std::span<const Point3> points, int nCtrl, int degree,
                                ParamMethod method) {
  CurveFitResult r;
  const int n = static_cast<int>(points.size());  // n data points, indices 0..n-1
  if (degree < 1 || nCtrl < degree + 1 || nCtrl >= n) return r;

  const std::vector<double> u = paramsImpl(points, method);
  if (static_cast<int>(u.size()) != n) return r;

  const std::vector<double> U = approxKnotsImpl(u, degree, nCtrl);
  const int lastPole = nCtrl - 1;

  // Control points 0 and nCtrl-1 are PINNED to the first / last data point. The
  // free unknowns are P_1..P_{nCtrl-2}: nFree = nCtrl - 2. The interior data points
  // Q_1..Q_{n-2} drive the least-squares (Eq 9.63): for data point k,
  //   R_k = Q_k − N_{0,p}(u_k)·Q_0 − N_{h,p}(u_k)·Q_{n-1},
  // and the normal-equation-free system minimizes Σ‖Σ_i N_{i,p}(u_k)·P_i − Q_k‖².
  // We assemble the overdetermined (nData-2)×(nFree) design matrix and hand the
  // three RHS columns to lstsq.
  const int nFree = nCtrl - 2;
  const int nRows = n - 2;  // interior data points k = 1..n-2

  // Precompute basis rows for every data point once.
  std::vector<double> Nrow(degree + 1);

  auto basisAt = [&](double uk, int& outSpan, std::vector<double>& out) {
    outSpan = findSpan(lastPole, degree, uk, U);
    out.assign(degree + 1, 0.0);
    basisFuns(outSpan, uk, degree, U, out);
  };

  const Point3 Q0 = points[0];
  const Point3 Qn = points[n - 1];

  if (nFree == 0) {
    // Degenerate LS with only the two pinned ends (nCtrl == 2, a straight segment
    // of degree 1): the fit is just the endpoint line; no free system to solve.
    r.curve.degree = degree;
    r.curve.knots = U;
    r.curve.poles = {Q0, Qn};
    curveErrors(r.curve, points, u, r.maxError, r.rmsError);
    r.ok = true;
    return r;
  }

  std::vector<double> A(static_cast<std::size_t>(nRows) * nFree, 0.0);
  std::vector<double> bx(nRows), by(nRows), bz(nRows);

  for (int row = 0; row < nRows; ++row) {
    const int k = row + 1;  // data index 1..n-2
    int span = 0;
    basisAt(u[k], span, Nrow);
    // First / last basis values pin into the RHS.
    double n0 = 0.0, nh = 0.0;
    for (int j = 0; j <= degree; ++j) {
      const int idx = span - degree + j;
      if (idx == 0) n0 = Nrow[j];
      else if (idx == nCtrl - 1) nh = Nrow[j];
      else {
        const int free = idx - 1;  // free unknown column (P_1..P_{nCtrl-2})
        if (free >= 0 && free < nFree)
          A[static_cast<std::size_t>(row) * nFree + free] = Nrow[j];
      }
    }
    const Point3 Rk = points[k] - (Q0.asVec() * n0) - (Qn.asVec() * nh);
    bx[row] = Rk.x; by[row] = Rk.y; bz[row] = Rk.z;
  }

  const std::vector<double> cx = lstsq(A, nRows, nFree, bx);
  const std::vector<double> cy = lstsq(A, nRows, nFree, by);
  const std::vector<double> cz = lstsq(A, nRows, nFree, bz);
  if (static_cast<int>(cx.size()) != nFree || static_cast<int>(cy.size()) != nFree ||
      static_cast<int>(cz.size()) != nFree)
    return r;

  r.curve.degree = degree;
  r.curve.knots = U;
  r.curve.poles.resize(nCtrl);
  r.curve.poles[0] = Q0;
  r.curve.poles[nCtrl - 1] = Qn;
  for (int i = 0; i < nFree; ++i) r.curve.poles[i + 1] = {cx[i], cy[i], cz[i]};

  curveErrors(r.curve, points, u, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface fitting — tensor product: fit each row (V), then each column (U).
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Compute the max / RMS grid deviation of a fitted surface from the data.
void surfaceErrors(const BsplineSurfaceData& s, const PointGrid& g,
                   std::span<const double> uParams, std::span<const double> vParams,
                   double& maxErr, double& rmsErr) {
  SurfaceGrid grid{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  maxErr = 0.0;
  double sumSq = 0.0;
  int count = 0;
  for (int i = 0; i < g.nU; ++i)
    for (int j = 0; j < g.nV; ++j) {
      const Point3 p =
          surfacePoint(s.degreeU, s.degreeV, grid, s.knotsU, s.knotsV, uParams[i], vParams[j]);
      const double e = distance(p, g.at(i, j));
      maxErr = std::max(maxErr, e);
      sumSq += e * e;
      ++count;
    }
  rmsErr = (count > 0) ? std::sqrt(sumSq / count) : 0.0;
}

// Column-averaged parameters across the grid (§9.2.5): average the per-row (or
// per-column) parameter assignments so the whole surface shares one U and one V
// parametrization. `dirU==true` averages the U-direction parameters (down columns).
std::vector<double> gridParams(const PointGrid& g, ParamMethod method, bool dirU) {
  const int nMain = dirU ? g.nU : g.nV;   // length of the returned parameter vector
  const int nCross = dirU ? g.nV : g.nU;  // how many lines we average over
  std::vector<double> acc(nMain, 0.0);
  int used = 0;
  std::vector<Point3> line(nMain);
  for (int c = 0; c < nCross; ++c) {
    for (int mIdx = 0; mIdx < nMain; ++mIdx)
      line[mIdx] = dirU ? g.at(mIdx, c) : g.at(c, mIdx);
    std::vector<double> p = paramsImpl(line, method);
    if (static_cast<int>(p.size()) != nMain) continue;  // that line was degenerate
    for (int mIdx = 0; mIdx < nMain; ++mIdx) acc[mIdx] += p[mIdx];
    ++used;
  }
  if (used == 0) return {};
  for (double& v : acc) v /= used;
  acc.front() = 0.0;
  acc.back() = 1.0;
  return acc;
}

// Fit one line of points to a curve — interpolation (nCtrl==npts) or approximation.
// Returns the poles (length nCtrl) or empty on failure. Uses the SUPPLIED shared
// parameters/knots so every row/column of the surface stays compatible.
std::vector<Point3> fitLine(std::span<const Point3> line, int degree, int nCtrl,
                            std::span<const double> params, std::span<const double> U) {
  const int npts = static_cast<int>(line.size());
  const int lastPole = nCtrl - 1;

  if (nCtrl == npts) {
    // Interpolation: square collocation solve on the shared knots/params.
    std::vector<double> A(static_cast<std::size_t>(npts) * npts, 0.0);
    std::vector<double> N(degree + 1);
    for (int k = 0; k < npts; ++k) {
      const int span = findSpan(lastPole, degree, params[k], U);
      basisFuns(span, params[k], degree, U, N);
      for (int j = 0; j <= degree; ++j)
        A[static_cast<std::size_t>(k) * npts + (span - degree + j)] = N[j];
    }
    std::vector<double> bx(npts), by(npts), bz(npts);
    for (int k = 0; k < npts; ++k) { bx[k] = line[k].x; by[k] = line[k].y; bz[k] = line[k].z; }
    std::vector<double> cx = lin_solve(A, npts, bx);
    std::vector<double> cy = lin_solve(A, npts, by);
    std::vector<double> cz = lin_solve(A, npts, bz);
    if (static_cast<int>(cx.size()) != npts) return {};
    std::vector<Point3> out(npts);
    for (int i = 0; i < npts; ++i) out[i] = {cx[i], cy[i], cz[i]};
    return out;
  }

  // Approximation: endpoints pinned, interior via lstsq (same scheme as the
  // stand-alone approximateCurve, using the shared knots/params).
  const int nFree = nCtrl - 2;
  const int nRows = npts - 2;
  const Point3 Q0 = line[0];
  const Point3 Qn = line[npts - 1];
  if (nFree <= 0) {
    return {Q0, Qn};  // nCtrl==2 straight segment
  }
  std::vector<double> A(static_cast<std::size_t>(nRows) * nFree, 0.0);
  std::vector<double> bx(nRows), by(nRows), bz(nRows);
  std::vector<double> N(degree + 1);
  for (int row = 0; row < nRows; ++row) {
    const int k = row + 1;
    const int span = findSpan(lastPole, degree, params[k], U);
    N.assign(degree + 1, 0.0);
    basisFuns(span, params[k], degree, U, N);
    double n0 = 0.0, nh = 0.0;
    for (int j = 0; j <= degree; ++j) {
      const int idx = span - degree + j;
      if (idx == 0) n0 = N[j];
      else if (idx == nCtrl - 1) nh = N[j];
      else {
        const int free = idx - 1;
        if (free >= 0 && free < nFree)
          A[static_cast<std::size_t>(row) * nFree + free] = N[j];
      }
    }
    const Point3 Rk = line[k] - (Q0.asVec() * n0) - (Qn.asVec() * nh);
    bx[row] = Rk.x; by[row] = Rk.y; bz[row] = Rk.z;
  }
  std::vector<double> cx = lstsq(A, nRows, nFree, bx);
  std::vector<double> cy = lstsq(A, nRows, nFree, by);
  std::vector<double> cz = lstsq(A, nRows, nFree, bz);
  if (static_cast<int>(cx.size()) != nFree) return {};
  std::vector<Point3> out(nCtrl);
  out[0] = Q0;
  out[nCtrl - 1] = Qn;
  for (int i = 0; i < nFree; ++i) out[i + 1] = {cx[i], cy[i], cz[i]};
  return out;
}

// Shared driver for surface interpolation / approximation. nCtrlU/nCtrlV equal to
// nU/nV ⇒ interpolation in that direction; smaller ⇒ least-squares approximation.
SurfaceFitResult fitSurface(const PointGrid& g, int nCtrlU, int nCtrlV, int degreeU,
                            int degreeV, ParamMethod method) {
  SurfaceFitResult r;
  if (degreeU < 1 || degreeV < 1) return r;
  if (nCtrlU < degreeU + 1 || nCtrlU > g.nU) return r;
  if (nCtrlV < degreeV + 1 || nCtrlV > g.nV) return r;
  if (g.nU < degreeU + 1 || g.nV < degreeV + 1) return r;

  // Shared U/V parameter assignments (averaged across the grid).
  const std::vector<double> uP = gridParams(g, method, /*dirU=*/true);
  const std::vector<double> vP = gridParams(g, method, /*dirU=*/false);
  if (static_cast<int>(uP.size()) != g.nU || static_cast<int>(vP.size()) != g.nV) return r;

  const std::vector<double> knotsV =
      (nCtrlV == g.nV) ? avgKnots(vP, degreeV) : approxKnotsImpl(vP, degreeV, nCtrlV);
  const std::vector<double> knotsU =
      (nCtrlU == g.nU) ? avgKnots(uP, degreeU) : approxKnotsImpl(uP, degreeU, nCtrlU);

  // Step 1 — fit each of the nU rows in V → an (nU × nCtrlV) intermediate net.
  std::vector<Point3> mid(static_cast<std::size_t>(g.nU) * nCtrlV);
  std::vector<Point3> rowBuf(g.nV);
  for (int i = 0; i < g.nU; ++i) {
    for (int j = 0; j < g.nV; ++j) rowBuf[j] = g.at(i, j);
    std::vector<Point3> ctrl = fitLine(rowBuf, degreeV, nCtrlV, vP, knotsV);
    if (static_cast<int>(ctrl.size()) != nCtrlV) return r;
    for (int j = 0; j < nCtrlV; ++j)
      mid[static_cast<std::size_t>(i) * nCtrlV + j] = ctrl[j];
  }

  // Step 2 — fit each of the nCtrlV columns in U → the final (nCtrlU × nCtrlV) net.
  std::vector<Point3> poles(static_cast<std::size_t>(nCtrlU) * nCtrlV);
  std::vector<Point3> colBuf(g.nU);
  for (int j = 0; j < nCtrlV; ++j) {
    for (int i = 0; i < g.nU; ++i) colBuf[i] = mid[static_cast<std::size_t>(i) * nCtrlV + j];
    std::vector<Point3> ctrl = fitLine(colBuf, degreeU, nCtrlU, uP, knotsU);
    if (static_cast<int>(ctrl.size()) != nCtrlU) return r;
    for (int i = 0; i < nCtrlU; ++i)
      poles[static_cast<std::size_t>(i) * nCtrlV + j] = ctrl[i];
  }

  r.surface.degreeU = degreeU;
  r.surface.degreeV = degreeV;
  r.surface.nPolesU = nCtrlU;
  r.surface.nPolesV = nCtrlV;
  r.surface.knotsU = knotsU;
  r.surface.knotsV = knotsV;
  r.surface.poles = std::move(poles);

  surfaceErrors(r.surface, g, uP, vP, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

}  // namespace

SurfaceFitResult interpolateSurface(const PointGrid& grid, int degreeU, int degreeV,
                                    ParamMethod method) {
  return fitSurface(grid, grid.nU, grid.nV, degreeU, degreeV, method);
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational tensor-product surface interpolation with prescribed weights.
//
// Lift every grid datum Q(i,j) (weight w(i,j)) to Qʷ(i,j) = (wQ, w) ∈ R⁴ and run
// the SAME two-step tensor interpolation as fitSurface, but on FOUR homogeneous
// coordinates simultaneously (fit each row in V, then each column in U — square
// collocation via lin_solve, shared averaged params/knots). Project the resulting
// homogeneous control net back to (pole, weight). Cʷ interpolates Qʷ, so the
// projected rational surface passes through every Euclidean Q(i,j) exactly.
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// A homogeneous R⁴ node — the four scalars carried through the tensor solve.
struct Homog4 { double x, y, z, w; };

// Interpolate one line of R⁴ nodes to a rational-lift control net (square collocation
// on the shared knots/params). Returns npts control points or empty on a singular
// solve. Solves ALL FOUR coordinates against the same matrix.
std::vector<Homog4> fitLineH(std::span<const Homog4> line, int degree,
                             std::span<const double> params, std::span<const double> U) {
  const int npts = static_cast<int>(line.size());
  const int lastPole = npts - 1;
  std::vector<double> A(static_cast<std::size_t>(npts) * npts, 0.0);
  std::vector<double> N(degree + 1);
  for (int k = 0; k < npts; ++k) {
    const int span = findSpan(lastPole, degree, params[k], U);
    basisFuns(span, params[k], degree, U, N);
    for (int j = 0; j <= degree; ++j)
      A[static_cast<std::size_t>(k) * npts + (span - degree + j)] = N[j];
  }
  std::vector<double> bx(npts), by(npts), bz(npts), bw(npts);
  for (int k = 0; k < npts; ++k) {
    bx[k] = line[k].x; by[k] = line[k].y; bz[k] = line[k].z; bw[k] = line[k].w;
  }
  std::vector<double> cx = lin_solve(A, npts, bx);
  std::vector<double> cy = lin_solve(A, npts, by);
  std::vector<double> cz = lin_solve(A, npts, bz);
  std::vector<double> cw = lin_solve(A, npts, bw);
  if (static_cast<int>(cx.size()) != npts || static_cast<int>(cw.size()) != npts) return {};
  std::vector<Homog4> out(npts);
  for (int i = 0; i < npts; ++i) out[i] = {cx[i], cy[i], cz[i], cw[i]};
  return out;
}

// Max / RMS deviation of a RATIONAL fitted surface from the data over the grid.
void rationalSurfaceErrors(const BsplineSurfaceData& s, const PointGrid& g,
                           std::span<const double> uParams, std::span<const double> vParams,
                           double& maxErr, double& rmsErr) {
  SurfaceGrid grid{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  maxErr = 0.0;
  double sumSq = 0.0;
  int count = 0;
  for (int i = 0; i < g.nU; ++i)
    for (int j = 0; j < g.nV; ++j) {
      const Point3 p = nurbsSurfacePoint(s.degreeU, s.degreeV, grid, s.weights,
                                         s.knotsU, s.knotsV, uParams[i], vParams[j]);
      const double e = distance(p, g.at(i, j));
      maxErr = std::max(maxErr, e);
      sumSq += e * e;
      ++count;
    }
  rmsErr = (count > 0) ? std::sqrt(sumSq / count) : 0.0;
}

}  // namespace

SurfaceFitResult interpolateRationalSurface(const PointGrid& grid, const WeightGrid& wg,
                                            int degreeU, int degreeV, ParamMethod method) {
  SurfaceFitResult r;
  if (degreeU < 1 || degreeV < 1) return r;
  if (wg.nU != grid.nU || wg.nV != grid.nV) return r;         // weight grid must match
  if (grid.nU < degreeU + 1 || grid.nV < degreeV + 1) return r;
  const int nU = grid.nU, nV = grid.nV;
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j)
      if (!(wg.at(i, j) > 0.0)) return r;                     // non-positive weight guard

  // Shared U/V parameter assignments (averaged across the grid — Euclidean geometry).
  const std::vector<double> uP = gridParams(grid, method, /*dirU=*/true);
  const std::vector<double> vP = gridParams(grid, method, /*dirU=*/false);
  if (static_cast<int>(uP.size()) != nU || static_cast<int>(vP.size()) != nV) return r;

  const std::vector<double> knotsV = avgKnots(vP, degreeV);
  const std::vector<double> knotsU = avgKnots(uP, degreeU);

  // Step 0 — lift the grid to homogeneous R⁴.
  std::vector<Homog4> H(static_cast<std::size_t>(nU) * nV);
  for (int i = 0; i < nU; ++i)
    for (int j = 0; j < nV; ++j) {
      const Point3 q = grid.at(i, j);
      const double w = wg.at(i, j);
      H[static_cast<std::size_t>(i) * nV + j] = {w * q.x, w * q.y, w * q.z, w};
    }

  // Step 1 — interpolate each of the nU rows in V → an (nU × nV) intermediate net.
  std::vector<Homog4> mid(static_cast<std::size_t>(nU) * nV);
  std::vector<Homog4> rowBuf(nV);
  for (int i = 0; i < nU; ++i) {
    for (int j = 0; j < nV; ++j) rowBuf[j] = H[static_cast<std::size_t>(i) * nV + j];
    std::vector<Homog4> ctrl = fitLineH(rowBuf, degreeV, vP, knotsV);
    if (static_cast<int>(ctrl.size()) != nV) return r;
    for (int j = 0; j < nV; ++j) mid[static_cast<std::size_t>(i) * nV + j] = ctrl[j];
  }

  // Step 2 — interpolate each of the nV columns in U → the final (nU × nV) net.
  std::vector<Homog4> net(static_cast<std::size_t>(nU) * nV);
  std::vector<Homog4> colBuf(nU);
  for (int j = 0; j < nV; ++j) {
    for (int i = 0; i < nU; ++i) colBuf[i] = mid[static_cast<std::size_t>(i) * nV + j];
    std::vector<Homog4> ctrl = fitLineH(colBuf, degreeU, uP, knotsU);
    if (static_cast<int>(ctrl.size()) != nU) return r;
    for (int i = 0; i < nU; ++i) net[static_cast<std::size_t>(i) * nV + j] = ctrl[i];
  }

  // Project the homogeneous net back; a non-positive control weight is a hard guard.
  std::vector<Point3> poles(static_cast<std::size_t>(nU) * nV);
  std::vector<double> weights(static_cast<std::size_t>(nU) * nV);
  for (std::size_t idx = 0; idx < net.size(); ++idx) {
    const Homog4& h = net[idx];
    if (!(h.w > 0.0)) return SurfaceFitResult{};  // projected weight ≤ 0 — decline
    poles[idx] = {h.x / h.w, h.y / h.w, h.z / h.w};
    weights[idx] = h.w;
  }

  r.surface.degreeU = degreeU;
  r.surface.degreeV = degreeV;
  r.surface.nPolesU = nU;
  r.surface.nPolesV = nV;
  r.surface.knotsU = knotsU;
  r.surface.knotsV = knotsV;
  r.surface.poles = std::move(poles);
  r.surface.weights = std::move(weights);

  rationalSurfaceErrors(r.surface, grid, uP, vP, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational SURFACE weight ESTIMATION (Ma–Kruth, tensor-product) — recover the
// control net AND the weight net from an UNWEIGHTED grid. The tensor rational fit
// S(uₖ,vₗ)=Q(k,l) is BILINEAR: writing the weighted control net Hᵢⱼ = wᵢⱼ·Pᵢⱼ, the
// condition Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ·(Pᵢⱼ − Q(k,l)) = 0 becomes
//   Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·Hᵢⱼ − Q(k,l)·(Σᵢⱼ Nᵢ(uₖ)Nⱼ(vₗ)·wᵢⱼ) = 0
// which is LINEAR and HOMOGENEOUS in z = (H₀₀..H_{mn}, w₀₀..w_{mn}). Each grid datum
// contributes 3 scalar rows (x/y/z), each coupling the H-block of that axis with the
// shared w-block through the (degU+1)·(degV+1) basis products active at (uₖ,vₗ). Over-
// determined (3·nU·nV > 4·nCtrlU·nCtrlV) ⇒ the fit is the 1-D null space of M, found as
// the smallest eigenvector of MᵀM by the SAME shifted inverse iteration (smallestEigvec)
// as the curve case (no external SVD). Then Pᵢⱼ=Hᵢⱼ/wᵢⱼ, gauge-normalized to w₀₀=1.
// The unknown layout is [Hx | Hy | Hz | w], each block nCtrl = nCtrlU·nCtrlV long,
// control point (i,j) at flat index i·nCtrlV + j (row-major U-outer, matching poles).
// ─────────────────────────────────────────────────────────────────────────────
namespace {

// Assemble MᵀM (size 4·nCtrl, nCtrl = nCtrlU·nCtrlV) for the tensor weight-estimation
// system over the given grid/params/knots. Unknown layout: [Hx | Hy | Hz | w].
std::vector<double> buildSurfaceWeightEstNormalMatrix(const PointGrid& g,
                                                      std::span<const double> uP,
                                                      std::span<const double> vP,
                                                      std::span<const double> U,
                                                      std::span<const double> V, int nCtrlU,
                                                      int nCtrlV, int degU, int degV) {
  const int nCtrl = nCtrlU * nCtrlV;
  const int nUnknown = 4 * nCtrl;
  const int W0 = 3 * nCtrl;  // weight-block offset
  const int lastU = nCtrlU - 1, lastV = nCtrlV - 1;
  std::vector<double> AtA(static_cast<std::size_t>(nUnknown) * nUnknown, 0.0);
  std::vector<double> Nu(degU + 1), Nv(degV + 1);
  std::vector<std::pair<int, double>> terms;
  terms.reserve(2 * (degU + 1) * (degV + 1));
  for (int k = 0; k < g.nU; ++k) {
    const int spanU = findSpan(lastU, degU, uP[k], U);
    basisFuns(spanU, uP[k], degU, U, Nu);
    for (int l = 0; l < g.nV; ++l) {
      const int spanV = findSpan(lastV, degV, vP[l], V);
      basisFuns(spanV, vP[l], degV, V, Nv);
      const Point3 q = g.at(k, l);
      const double qd[3] = {q.x, q.y, q.z};
      for (int axis = 0; axis < 3; ++axis) {
        // Row for axis d: +Nᵢⱼ at H_d[i,j], −Q(k,l)_d·Nᵢⱼ at w[i,j]. MᵀM += row ⊗ row.
        terms.clear();
        for (int a = 0; a <= degU; ++a) {
          const int ii = spanU - degU + a;
          for (int b = 0; b <= degV; ++b) {
            const int jj = spanV - degV + b;
            const int ctrl = ii * nCtrlV + jj;          // flat control index (U outer)
            const double Nij = Nu[a] * Nv[b];
            terms.emplace_back(axis * nCtrl + ctrl, Nij);
            terms.emplace_back(W0 + ctrl, -qd[axis] * Nij);
          }
        }
        for (const auto& [ci, cv] : terms)
          for (const auto& [cj, cw] : terms)
            AtA[static_cast<std::size_t>(ci) * nUnknown + cj] += cv * cw;
      }
    }
  }
  return AtA;
}

}  // namespace

RationalSurfaceFitResult fitRationalSurfaceEstimateWeightsWithParams(
    const PointGrid& grid, std::span<const double> uParams, std::span<const double> vParams,
    std::span<const double> knotsU, std::span<const double> knotsV, int nCtrlU, int nCtrlV,
    int degreeU, int degreeV, double flatTol) {
  RationalSurfaceFitResult r;
  const int nU = grid.nU, nV = grid.nV;
  if (degreeU < 1 || degreeV < 1 || nCtrlU < degreeU + 1 || nCtrlV < degreeV + 1 ||
      nU < nCtrlU || nV < nCtrlV) {
    r.diagnostic = "invalid dimensions (need degree+1 <= nCtrl <= nGrid in each direction)";
    return r;
  }
  if (static_cast<int>(uParams.size()) != nU || static_cast<int>(vParams.size()) != nV) {
    r.diagnostic = "uParams/vParams size must equal grid.nU/grid.nV";
    return r;
  }
  if (static_cast<int>(knotsU.size()) != nCtrlU + degreeU + 1 ||
      static_cast<int>(knotsV.size()) != nCtrlV + degreeV + 1) {
    r.diagnostic = "knotsU/knotsV size must equal nCtrl+degree+1";
    return r;
  }
  // Over-determination: 3 rows/datum, 4 unknowns/control point. Need a clean 1-D
  // null space, so demand strictly more rows than unknowns.
  const int nCtrl = nCtrlU * nCtrlV;
  const int nUnknown = 4 * nCtrl;  // Hx,Hy,Hz,w per control point
  if (3 * nU * nV <= nUnknown) {
    r.diagnostic = "under-determined: need 3*nU*nV > 4*nCtrlU*nCtrlV to pin the weights";
    return r;
  }
  const int W0 = 3 * nCtrl;  // offset of the weight block in the unknown vector

  // Assemble MᵀM for the homogeneous bilinear tensor system (see comment block above).
  const std::vector<double> AtA = buildSurfaceWeightEstNormalMatrix(
      grid, uParams, vParams, knotsU, knotsV, nCtrlU, nCtrlV, degreeU, degreeV);

  // Scale-invariant shift for inverse iteration (trace-relative) — same as the curve.
  double trace = 0.0;
  for (int i = 0; i < nUnknown; ++i) trace += AtA[static_cast<std::size_t>(i) * nUnknown + i];
  const double shift = (trace > 0.0 ? trace / nUnknown : 1.0) * 1e-12;

  double eig0 = 0.0, eig1 = 0.0;
  std::vector<double> z = smallestEigvec(AtA, nUnknown, shift, nullptr, eig0);
  if (static_cast<int>(z.size()) != nUnknown) {
    r.diagnostic = "eigen solve failed (singular shifted system)";
    return r;
  }
  const std::vector<double> z1 = smallestEigvec(AtA, nUnknown, shift, &z, eig1);
  if (static_cast<int>(z1.size()) == nUnknown) {
    // Rank test: a clean 1-D null space means eig0 ≪ eig1 (large relative GAP). A
    // comparable eig1 (small gap) means the null space is not one-dimensional
    // (rank-deficient) — weights not unique, decline. Same criterion as the curve.
    const double gap = eig1 / std::max(eig0, 1e-300);
    if (!(gap > 50.0)) {
      r.diagnostic = "rank-deficient: null space not one-dimensional (weights not unique)";
      return r;
    }
  }

  // Extract weights; enforce a single shared sign (flip the whole vector if needed).
  std::vector<double> w(nCtrl);
  double sumW = 0.0;
  for (int i = 0; i < nCtrl; ++i) { w[i] = z[W0 + i]; sumW += w[i]; }
  if (sumW < 0.0) for (double& e : z) e = -e;   // gauge the shared sign to +
  for (int i = 0; i < nCtrl; ++i) w[i] = z[W0 + i];

  // Sign-flip guard: a valid rational needs all weights strictly the SAME sign.
  double wmin = w[0], wmax = w[0];
  for (int i = 0; i < nCtrl; ++i) { wmin = std::min(wmin, w[i]); wmax = std::max(wmax, w[i]); }
  const double wref = std::fabs(w[0]) > 0.0 ? std::fabs(w[0]) : 1.0;
  if (!(wmin > 1e-9 * wref)) {
    r.diagnostic = "sign-flipping / near-zero weight recovered (invalid rational)";
    return r;
  }

  // De-homogenize: Pᵢⱼ = Hᵢⱼ/wᵢⱼ, then normalize weights to the w₀₀ = 1 gauge.
  const double g = w[0];
  r.surface.degreeU = degreeU;
  r.surface.degreeV = degreeV;
  r.surface.nPolesU = nCtrlU;
  r.surface.nPolesV = nCtrlV;
  r.surface.knotsU.assign(knotsU.begin(), knotsU.end());
  r.surface.knotsV.assign(knotsV.begin(), knotsV.end());
  r.surface.poles.resize(nCtrl);
  r.surface.weights.resize(nCtrl);
  for (int i = 0; i < nCtrl; ++i) {
    const double wi = w[i];
    r.surface.poles[i] = {z[i] / wi, z[nCtrl + i] / wi, z[2 * nCtrl + i] / wi};
    r.surface.weights[i] = wi / g;  // gauge w₀₀ = 1
  }

  // Weight spread after the gauge fix — the non-rationality detector.
  double gmin = r.surface.weights[0], gmax = r.surface.weights[0];
  for (double wi : r.surface.weights) { gmin = std::min(gmin, wi); gmax = std::max(gmax, wi); }
  r.weightSpread = gmax - gmin;
  r.rationalityDetected = (r.weightSpread > flatTol);

  rationalSurfaceErrors(r.surface, grid, uParams, vParams, r.maxError, r.rmsError);
  r.ok = true;
  return r;
}

RationalSurfaceFitResult fitRationalSurfaceEstimateWeights(const PointGrid& grid, int nCtrlU,
                                                           int nCtrlV, int degreeU,
                                                           int degreeV, ParamMethod method,
                                                           double flatTol) {
  RationalSurfaceFitResult r;
  const int nU = grid.nU, nV = grid.nV;
  if (degreeU < 1 || degreeV < 1 || nCtrlU < degreeU + 1 || nCtrlV < degreeV + 1 ||
      nU < nCtrlU || nV < nCtrlV) {
    r.diagnostic = "invalid dimensions (need degree+1 <= nCtrl <= nGrid in each direction)";
    return r;
  }
  const std::vector<double> uP = gridParams(grid, method, /*dirU=*/true);
  const std::vector<double> vP = gridParams(grid, method, /*dirU=*/false);
  if (static_cast<int>(uP.size()) != nU || static_cast<int>(vP.size()) != nV) {
    r.diagnostic = "degenerate parametrization (all-coincident grid line)";
    return r;
  }
  const std::vector<double> U = approxKnotsImpl(uP, degreeU, nCtrlU);
  const std::vector<double> V = approxKnotsImpl(vP, degreeV, nCtrlV);
  return fitRationalSurfaceEstimateWeightsWithParams(grid, uP, vP, U, V, nCtrlU, nCtrlV,
                                                     degreeU, degreeV, flatTol);
}

SurfaceFitResult approximateSurface(const PointGrid& grid, int nCtrlU, int nCtrlV,
                                    int degreeU, int degreeV, ParamMethod method) {
  return fitSurface(grid, nCtrlU, nCtrlV, degreeU, degreeV, method);
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

// SPDX-License-Identifier: Apache-2.0
//
// bspline_gordon.cpp — NURBS roadmap Layer 6 (GORDON / NETWORK surface) implementation.
//
// Clean-room from *The NURBS Book* (Piegl & Tiller, 2nd ed.), §10.5 — the Gordon
// surface as a BOOLEAN SUM `G = S_u ⊕ S_v ⊖ T`. It COMPOSES Layer 1 (bspline_ops:
// elevateDegreeSurface / refineKnotSurface, to bring the three summands to one common
// tensor-product basis EXACTLY) and the Layer-6/7 interpolation machinery (a family of
// collocation solves through the numsci facade, to interpolate each curve family across
// its transversal direction and the K×L grid tensor-product). The interpolations solve
// linear systems through numerics::lin_solve, so — like bspline_skin.cpp / bspline_fit.cpp
// — the WHOLE file is under CYBERCAD_HAS_NUMSCI. With the guard OFF the TU is inert and
// the Layer-6 Gordon functions are simply absent from the library.
//
#include "native/math/bspline_gordon.h"

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"        // curvePoint / findSpan / basisFuns (collocation)
#include "native/math/bspline_ops.h"    // elevate/refine surface (compatibility) + curve ops
#include "native/math/bspline_skin.h"   // makeSectionsCompatible (per-family normalization)
#include "native/numerics/numerics.h"   // lin_solve (square interpolation solve)

#include <algorithm>
#include <cmath>
#include <span>
#include <string>
#include <vector>

namespace cybercad::native::math {
namespace {

using numerics::lin_solve;

constexpr double kKnotEps = 1e-9;  // knot-value coincidence tolerance for the union.

int nPolesOf(const BsplineCurveData& c) {
  return static_cast<int>(c.knots.size()) - c.degree - 1;
}

// ── Averaging knots for interpolation across a family at prescribed params (Eq 9.8) ──
// Clamped degree-q knot vector whose interior knots are running averages of q
// consecutive parameters. n = last param index (M−1), length = M + q + 1.
std::vector<double> avgKnots(const std::vector<double>& t, int q) {
  const int n = static_cast<int>(t.size()) - 1;  // last index (M−1)
  const int m = n + q + 1;
  std::vector<double> U(m + 1, 0.0);
  for (int i = m - q; i <= m; ++i) U[i] = 1.0;      // clamped tail
  for (int j = 1; j <= n - q; ++j) {                // interior averages
    double s = 0.0;
    for (int i = j; i <= j + q - 1; ++i) s += t[i];
    U[j + q] = s / q;
  }
  return U;
}

// ── Interpolate ONE family of compatible curves ACROSS the transversal direction at
// PRESCRIBED parameters t_k (unlike skinSurface, which picks chord-length params). The
// curves share degree p, along-curve knots and N control points (they were made
// compatible). Returns the tensor-product surface whose iso-curve at t_k equals curve k:
//   * the ALONG direction carries the curve shape (degree p, common curve knots, N poles),
//   * the ACROSS direction interpolates the M curves (degree q, averaging knots, M poles).
// `alongIsU` chooses which surface direction the curve shape maps to. Returns false on a
// singular collocation. The net is laid out row-major, U-outer.
bool interpFamilyAcross(const std::vector<BsplineCurveData>& compat,
                        const std::vector<double>& t, int q, bool alongIsU,
                        BsplineSurfaceData& out) {
  const int M = static_cast<int>(compat.size());  // #curves in the family
  const int p = compat.front().degree;
  const std::vector<double>& alongKnots = compat.front().knots;
  const int N = nPolesOf(compat.front());         // #poles per curve (along direction)
  const std::vector<double> acrossKnots = avgKnots(t, q);

  // Collocation matrix A(k,j) = N_{j,q}(t_k) for the across interpolation, K×K = M×M.
  std::vector<double> A(static_cast<std::size_t>(M) * M, 0.0);
  std::vector<double> Nb(q + 1);
  for (int k = 0; k < M; ++k) {
    const int span = findSpan(M - 1, q, t[k], acrossKnots);
    basisFuns(span, t[k], q, acrossKnots, Nb);
    for (int j = 0; j <= q; ++j)
      A[static_cast<std::size_t>(k) * M + (span - q + j)] = Nb[j];
  }

  // For each along-index i solve for the M across-control points through {P_i^k}.
  // acrossNet[i*M + j] = j-th across-control of along-index i.
  std::vector<Point3> acrossNet(static_cast<std::size_t>(N) * M);
  std::vector<double> bx(M), by(M), bz(M);
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < M; ++k) {
      const Point3 pnt = compat[k].poles[i];
      bx[k] = pnt.x; by[k] = pnt.y; bz[k] = pnt.z;
    }
    const std::vector<double> cx = lin_solve(A, M, bx);
    const std::vector<double> cy = lin_solve(A, M, by);
    const std::vector<double> cz = lin_solve(A, M, bz);
    if (static_cast<int>(cx.size()) != M || static_cast<int>(cy.size()) != M ||
        static_cast<int>(cz.size()) != M)
      return false;  // singular across-collocation
    for (int j = 0; j < M; ++j)
      acrossNet[static_cast<std::size_t>(i) * M + j] = {cx[j], cy[j], cz[j]};
  }

  // Assemble as a surface. If the curve shape is U: nPolesU=N (along), nPolesV=M (across),
  // and the row-major U-outer net is exactly acrossNet (pole(i,j)=acrossNet[i*M+j]). If
  // the curve shape is V: nPolesU=M (across), nPolesV=N (along) — transpose the net.
  if (alongIsU) {
    out.degreeU = p;         out.degreeV = q;
    out.nPolesU = N;         out.nPolesV = M;
    out.knotsU = alongKnots; out.knotsV = acrossKnots;
    out.poles = std::move(acrossNet);
  } else {
    out.degreeU = q;         out.degreeV = p;
    out.nPolesU = M;         out.nPolesV = N;
    out.knotsU = acrossKnots; out.knotsV = alongKnots;
    out.poles.assign(static_cast<std::size_t>(M) * N, Point3{});
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < M; ++j)
        out.poles[static_cast<std::size_t>(j) * N + i] = acrossNet[static_cast<std::size_t>(i) * M + j];
  }
  out.weights.clear();
  return true;
}

// ── Tensor-product interpolation of the K×L grid at (uParams, vParams) ────────────
// Interpolate the grid `Q` (K outer over v-station, L inner over u-station: Q[k*L+l])
// as a degree (qU,qV) tensor surface T with T(u_l, v_k) = Q_{k,l}. First interpolate each
// v-station row across u (L points at uParams → degree qU), then interpolate the resulting
// control columns across v (K rows at vParams → degree qV). Row-major U-outer output.
bool interpGrid(const std::vector<Point3>& Q, int K, int L,
                const std::vector<double>& uParams, const std::vector<double>& vParams,
                int qU, int qV, BsplineSurfaceData& out) {
  const std::vector<double> knotsU = avgKnots(uParams, qU);
  const std::vector<double> knotsV = avgKnots(vParams, qV);

  // Stage 1 — for each v-station k, interpolate the L points across u → L u-controls.
  // Au is L×L, shared by every station.
  std::vector<double> Au(static_cast<std::size_t>(L) * L, 0.0);
  std::vector<double> Nb(qU + 1);
  for (int l = 0; l < L; ++l) {
    const int span = findSpan(L - 1, qU, uParams[l], knotsU);
    basisFuns(span, uParams[l], qU, knotsU, Nb);
    for (int j = 0; j <= qU; ++j)
      Au[static_cast<std::size_t>(l) * L + (span - qU + j)] = Nb[j];
  }
  std::vector<Point3> mid(static_cast<std::size_t>(K) * L);  // mid[k*L + a] u-control a of row k
  std::vector<double> bx(L), by(L), bz(L);
  for (int k = 0; k < K; ++k) {
    for (int l = 0; l < L; ++l) {
      const Point3 pnt = Q[static_cast<std::size_t>(k) * L + l];
      bx[l] = pnt.x; by[l] = pnt.y; bz[l] = pnt.z;
    }
    const std::vector<double> cx = lin_solve(Au, L, bx);
    const std::vector<double> cy = lin_solve(Au, L, by);
    const std::vector<double> cz = lin_solve(Au, L, bz);
    if (static_cast<int>(cx.size()) != L || static_cast<int>(cy.size()) != L ||
        static_cast<int>(cz.size()) != L)
      return false;
    for (int a = 0; a < L; ++a)
      mid[static_cast<std::size_t>(k) * L + a] = {cx[a], cy[a], cz[a]};
  }

  // Stage 2 — for each u-control column a, interpolate the K rows across v → K v-controls.
  // Av is K×K, shared by every column.
  std::vector<double> Av(static_cast<std::size_t>(K) * K, 0.0);
  std::vector<double> Nv(qV + 1);
  for (int k = 0; k < K; ++k) {
    const int span = findSpan(K - 1, qV, vParams[k], knotsV);
    basisFuns(span, vParams[k], qV, knotsV, Nv);
    for (int j = 0; j <= qV; ++j)
      Av[static_cast<std::size_t>(k) * K + (span - qV + j)] = Nv[j];
  }
  // Output net: nPolesU = L (u-controls), nPolesV = K (v-controls), row-major U-outer.
  out.degreeU = qU;    out.degreeV = qV;
  out.nPolesU = L;     out.nPolesV = K;
  out.knotsU = knotsU; out.knotsV = knotsV;
  out.poles.assign(static_cast<std::size_t>(L) * K, Point3{});
  std::vector<double> gx(K), gy(K), gz(K);
  for (int a = 0; a < L; ++a) {
    for (int k = 0; k < K; ++k) {
      const Point3 pnt = mid[static_cast<std::size_t>(k) * L + a];
      gx[k] = pnt.x; gy[k] = pnt.y; gz[k] = pnt.z;
    }
    const std::vector<double> cx = lin_solve(Av, K, gx);
    const std::vector<double> cy = lin_solve(Av, K, gy);
    const std::vector<double> cz = lin_solve(Av, K, gz);
    if (static_cast<int>(cx.size()) != K || static_cast<int>(cy.size()) != K ||
        static_cast<int>(cz.size()) != K)
      return false;
    for (int k = 0; k < K; ++k)
      out.poles[static_cast<std::size_t>(a) * K + k] = {cx[k], cy[k], cz[k]};  // pole(a,k)
  }
  out.weights.clear();
  return true;
}

// ═════════════════════════════════════════════════════════════════════════════
// RATIONAL boolean-sum machinery — every interpolation runs in HOMOGENEOUS R⁴.
// A rational curve/surface is carried as its homogeneous net (w·P, w); the four
// coordinates (wx, wy, wz, w) are interpolated by the SAME collocation matrix as the
// non-rational path, then projected back to (pole, weight). Non-positive projected
// weights clear the caller's `ok` (never divide by ≤ 0, never a faked rational net).
// ═════════════════════════════════════════════════════════════════════════════

// A homogeneous point (wx, wy, wz, w) — the R⁴ lift of a rational control point.
struct Homog4 { double x, y, z, w; };

Homog4 liftH(const Point3& p, double w) { return {w * p.x, w * p.y, w * p.z, w}; }

// Evaluate the HOMOGENEOUS point (wx, wy, wz, w) of a rational curve at parameter u — the
// numerator/denominator of the NURBS quotient BEFORE the divide. Runs the ordinary B-spline
// basis on the lifted R⁴ net (the same span/basis the rational evaluator uses). This is the
// value the boolean-sum grid must be consistent in (not merely the projected Euclidean point).
Homog4 homogCurvePoint(const BsplineCurveData& c, double u) {
  const int n = nPolesOf(c) - 1;
  const int span = findSpan(n, c.degree, u, c.knots);
  std::vector<double> Nb(c.degree + 1);
  basisFuns(span, u, c.degree, c.knots, Nb);
  Homog4 acc{0, 0, 0, 0};
  for (int j = 0; j <= c.degree; ++j) {
    const int idx = span - c.degree + j;
    const double w = c.weights[idx];
    const Point3& p = c.poles[idx];
    acc.x += Nb[j] * w * p.x;
    acc.y += Nb[j] * w * p.y;
    acc.z += Nb[j] * w * p.z;
    acc.w += Nb[j] * w;
  }
  return acc;
}

// Interpolate ONE rational family of compatible curves ACROSS at prescribed params t_k,
// in homogeneous R⁴. `compat` are rational (poles + one weight per pole), sharing degree p,
// along-curve knots, and N control points. Produces the rational surface net (poles+weights)
// whose iso-curve at t_k equals rational curve k. `alongIsU` chooses the surface direction.
bool interpFamilyAcrossRational(const std::vector<BsplineCurveData>& compat,
                                const std::vector<double>& t, int q, bool alongIsU,
                                BsplineSurfaceData& out) {
  const int M = static_cast<int>(compat.size());
  const int p = compat.front().degree;
  const std::vector<double>& alongKnots = compat.front().knots;
  const int N = nPolesOf(compat.front());
  const std::vector<double> acrossKnots = avgKnots(t, q);

  // Collocation matrix A(k,j) = N_{j,q}(t_k), M×M, shared by every along-index / coord.
  std::vector<double> A(static_cast<std::size_t>(M) * M, 0.0);
  std::vector<double> Nb(q + 1);
  for (int k = 0; k < M; ++k) {
    const int span = findSpan(M - 1, q, t[k], acrossKnots);
    basisFuns(span, t[k], q, acrossKnots, Nb);
    for (int j = 0; j <= q; ++j)
      A[static_cast<std::size_t>(k) * M + (span - q + j)] = Nb[j];
  }

  std::vector<Point3> netPoles(static_cast<std::size_t>(N) * M);
  std::vector<double> netW(static_cast<std::size_t>(N) * M);
  std::vector<double> bx(M), by(M), bz(M), bw(M);
  for (int i = 0; i < N; ++i) {
    for (int k = 0; k < M; ++k) {
      const Homog4 h = liftH(compat[k].poles[i], compat[k].weights[i]);
      bx[k] = h.x; by[k] = h.y; bz[k] = h.z; bw[k] = h.w;
    }
    const std::vector<double> cx = lin_solve(A, M, bx);
    const std::vector<double> cy = lin_solve(A, M, by);
    const std::vector<double> cz = lin_solve(A, M, bz);
    const std::vector<double> cw = lin_solve(A, M, bw);
    if (static_cast<int>(cx.size()) != M || static_cast<int>(cy.size()) != M ||
        static_cast<int>(cz.size()) != M || static_cast<int>(cw.size()) != M)
      return false;  // singular across-collocation
    for (int j = 0; j < M; ++j) {
      if (!(cw[j] > 0.0)) return false;  // projected weight ≤ 0 — documented guard
      netPoles[static_cast<std::size_t>(i) * M + j] = {cx[j] / cw[j], cy[j] / cw[j], cz[j] / cw[j]};
      netW[static_cast<std::size_t>(i) * M + j] = cw[j];
    }
  }

  if (alongIsU) {
    out.degreeU = p;         out.degreeV = q;
    out.nPolesU = N;         out.nPolesV = M;
    out.knotsU = alongKnots; out.knotsV = acrossKnots;
    out.poles = std::move(netPoles);
    out.weights = std::move(netW);
  } else {
    out.degreeU = q;         out.degreeV = p;
    out.nPolesU = M;         out.nPolesV = N;
    out.knotsU = acrossKnots; out.knotsV = alongKnots;
    out.poles.assign(static_cast<std::size_t>(M) * N, Point3{});
    out.weights.assign(static_cast<std::size_t>(M) * N, 1.0);
    for (int i = 0; i < N; ++i)
      for (int j = 0; j < M; ++j) {
        const std::size_t src = static_cast<std::size_t>(i) * M + j;
        const std::size_t dst = static_cast<std::size_t>(j) * N + i;
        out.poles[dst] = netPoles[src];
        out.weights[dst] = netW[src];
      }
  }
  return true;
}

// Tensor-product interpolation of the K×L HOMOGENEOUS grid at (uParams, vParams). `Qh` is the
// homogeneous grid (K outer, L inner: Qh[k*L+l]) — the SAME homogeneous intersection values the
// two rational skins land at, so the boolean sum cancels. Interpolate each v-row across u then
// the resulting columns across v, all four coords, project back. Row-major U-outer output.
bool interpGridRational(const std::vector<Homog4>& Qh, int K, int L,
                        const std::vector<double>& uParams, const std::vector<double>& vParams,
                        int qU, int qV, BsplineSurfaceData& out) {
  const std::vector<double> knotsU = avgKnots(uParams, qU);
  const std::vector<double> knotsV = avgKnots(vParams, qV);

  // Stage 1 — each v-station row: interpolate the L homogeneous points across u.
  std::vector<double> Au(static_cast<std::size_t>(L) * L, 0.0);
  std::vector<double> Nb(qU + 1);
  for (int l = 0; l < L; ++l) {
    const int span = findSpan(L - 1, qU, uParams[l], knotsU);
    basisFuns(span, uParams[l], qU, knotsU, Nb);
    for (int j = 0; j <= qU; ++j)
      Au[static_cast<std::size_t>(l) * L + (span - qU + j)] = Nb[j];
  }
  std::vector<Homog4> mid(static_cast<std::size_t>(K) * L);  // homogeneous u-controls per row
  std::vector<double> bx(L), by(L), bz(L), bw(L);
  for (int k = 0; k < K; ++k) {
    for (int l = 0; l < L; ++l) {
      const Homog4 h = Qh[static_cast<std::size_t>(k) * L + l];
      bx[l] = h.x; by[l] = h.y; bz[l] = h.z; bw[l] = h.w;
    }
    const std::vector<double> cx = lin_solve(Au, L, bx);
    const std::vector<double> cy = lin_solve(Au, L, by);
    const std::vector<double> cz = lin_solve(Au, L, bz);
    const std::vector<double> cw = lin_solve(Au, L, bw);
    if (static_cast<int>(cx.size()) != L || static_cast<int>(cy.size()) != L ||
        static_cast<int>(cz.size()) != L || static_cast<int>(cw.size()) != L)
      return false;
    for (int a = 0; a < L; ++a)
      mid[static_cast<std::size_t>(k) * L + a] = {cx[a], cy[a], cz[a], cw[a]};
  }

  // Stage 2 — each u-control column: interpolate the K homogeneous rows across v, project back.
  std::vector<double> Av(static_cast<std::size_t>(K) * K, 0.0);
  std::vector<double> Nv(qV + 1);
  for (int k = 0; k < K; ++k) {
    const int span = findSpan(K - 1, qV, vParams[k], knotsV);
    basisFuns(span, vParams[k], qV, knotsV, Nv);
    for (int j = 0; j <= qV; ++j)
      Av[static_cast<std::size_t>(k) * K + (span - qV + j)] = Nv[j];
  }
  out.degreeU = qU;    out.degreeV = qV;
  out.nPolesU = L;     out.nPolesV = K;
  out.knotsU = knotsU; out.knotsV = knotsV;
  out.poles.assign(static_cast<std::size_t>(L) * K, Point3{});
  out.weights.assign(static_cast<std::size_t>(L) * K, 1.0);
  std::vector<double> gx(K), gy(K), gz(K), gw(K);
  for (int a = 0; a < L; ++a) {
    for (int k = 0; k < K; ++k) {
      const Homog4 h = mid[static_cast<std::size_t>(k) * L + a];
      gx[k] = h.x; gy[k] = h.y; gz[k] = h.z; gw[k] = h.w;
    }
    const std::vector<double> cx = lin_solve(Av, K, gx);
    const std::vector<double> cy = lin_solve(Av, K, gy);
    const std::vector<double> cz = lin_solve(Av, K, gz);
    const std::vector<double> cw = lin_solve(Av, K, gw);
    if (static_cast<int>(cx.size()) != K || static_cast<int>(cy.size()) != K ||
        static_cast<int>(cz.size()) != K || static_cast<int>(cw.size()) != K)
      return false;
    for (int k = 0; k < K; ++k) {
      if (!(cw[k] > 0.0)) return false;  // projected weight ≤ 0 — documented guard
      const std::size_t idx = static_cast<std::size_t>(a) * K + k;
      out.poles[idx] = {cx[k] / cw[k], cy[k] / cw[k], cz[k] / cw[k]};
      out.weights[idx] = cw[k];
    }
  }
  return true;
}

// ── Bring two surfaces to a common basis in ONE direction (exact Layer-1) ─────────
// Raise both to the common max degree, then merge both onto the union knot vector in
// direction d. Both ops are exact (geometry-preserving), so neither surface's geometry
// drifts. After this the two surfaces share degree and knots in direction d.
std::vector<double> distinctKnotUnion(const std::vector<double>& a,
                                      const std::vector<double>& b) {
  // Max multiplicity per distinct value across both vectors.
  struct KM { double v; int m; };
  std::vector<KM> uni;
  auto absorb = [&](const std::vector<double>& kv) {
    std::size_t i = 0;
    while (i < kv.size()) {
      const double val = kv[i];
      int mult = 0;
      while (i < kv.size() && std::fabs(kv[i] - val) <= kKnotEps) { ++mult; ++i; }
      auto it = std::find_if(uni.begin(), uni.end(),
                             [&](const KM& u) { return std::fabs(u.v - val) <= kKnotEps; });
      if (it == uni.end()) uni.push_back({val, mult});
      else it->m = std::max(it->m, mult);
    }
  };
  absorb(a);
  absorb(b);
  std::sort(uni.begin(), uni.end(), [](const KM& x, const KM& y) { return x.v < y.v; });
  std::vector<double> flat;
  for (const KM& u : uni)
    for (int r = 0; r < u.m; ++r) flat.push_back(u.v);
  return flat;
}

// Knots to INSERT into `have` to reach `target` (both flat, same domain).
std::vector<double> knotsToInsert(const std::vector<double>& have,
                                  const std::vector<double>& target) {
  std::vector<double> ins;
  // Distinct values of target with multiplicities.
  std::size_t i = 0;
  while (i < target.size()) {
    const double val = target[i];
    int need = 0;
    while (i < target.size() && std::fabs(target[i] - val) <= kKnotEps) { ++need; ++i; }
    int has = 0;
    for (double kv : have)
      if (std::fabs(kv - val) <= kKnotEps) ++has;
    for (int r = has; r < need; ++r) ins.push_back(val);
  }
  return ins;
}

const std::vector<double>& dirKnots(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.knotsU : s.knotsV;
}
int dirDegree(const BsplineSurfaceData& s, ParamDir d) {
  return d == ParamDir::U ? s.degreeU : s.degreeV;
}

// Raise both surfaces to a common degree + common knots in direction d. Exact.
void unifyDirection(BsplineSurfaceData& a, BsplineSurfaceData& b, ParamDir d) {
  const int maxDeg = std::max(dirDegree(a, d), dirDegree(b, d));
  if (dirDegree(a, d) < maxDeg) a = elevateDegreeSurface(a, d, maxDeg - dirDegree(a, d));
  if (dirDegree(b, d) < maxDeg) b = elevateDegreeSurface(b, d, maxDeg - dirDegree(b, d));
  const std::vector<double> uni = distinctKnotUnion(dirKnots(a, d), dirKnots(b, d));
  const std::vector<double> insA = knotsToInsert(dirKnots(a, d), uni);
  const std::vector<double> insB = knotsToInsert(dirKnots(b, d), uni);
  if (!insA.empty()) a = refineKnotSurface(a, d, insA);
  if (!insB.empty()) b = refineKnotSurface(b, d, insB);
}

// True when two surfaces already share degree + knots in BOTH directions (same net shape).
bool sameBasis(const BsplineSurfaceData& a, const BsplineSurfaceData& b) {
  if (a.degreeU != b.degreeU || a.degreeV != b.degreeV) return false;
  if (a.nPolesU != b.nPolesU || a.nPolesV != b.nPolesV) return false;
  if (a.knotsU.size() != b.knotsU.size() || a.knotsV.size() != b.knotsV.size()) return false;
  for (std::size_t i = 0; i < a.knotsU.size(); ++i)
    if (std::fabs(a.knotsU[i] - b.knotsU[i]) > 1e-7) return false;
  for (std::size_t i = 0; i < a.knotsV.size(); ++i)
    if (std::fabs(a.knotsV[i] - b.knotsV[i]) > 1e-7) return false;
  return true;
}

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────
// Network consistency.
// ─────────────────────────────────────────────────────────────────────────────

NetworkCheck verifyNetwork(const CurveNetwork& network, double tol) {
  NetworkCheck r;
  const int K = static_cast<int>(network.uCurves.size());
  const int L = static_cast<int>(network.vCurves.size());
  r.K = K;
  r.L = L;

  if (K < 2 || L < 2) { r.reason = "need at least 2 curves in each direction"; return r; }
  if (static_cast<int>(network.vParams.size()) != K) { r.reason = "vParams size != K"; return r; }
  if (static_cast<int>(network.uParams.size()) != L) { r.reason = "uParams size != L"; return r; }

  // Non-rational + well-formed guard for both families.
  auto wellFormed = [](const BsplineCurveData& c) {
    return c.weights.empty() && c.degree >= 1 && !c.poles.empty() &&
           static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 1;
  };
  for (const BsplineCurveData& c : network.uCurves)
    if (!wellFormed(c)) { r.reason = "a u-curve is rational or malformed"; return r; }
  for (const BsplineCurveData& c : network.vCurves)
    if (!wellFormed(c)) { r.reason = "a v-curve is rational or malformed"; return r; }

  // Strictly-increasing station params (a proper monotone grid).
  for (int k = 1; k < K; ++k)
    if (!(network.vParams[k] > network.vParams[k - 1] + 1e-12)) {
      r.reason = "vParams not strictly increasing"; return r;
    }
  for (int l = 1; l < L; ++l)
    if (!(network.uParams[l] > network.uParams[l - 1] + 1e-12)) {
      r.reason = "uParams not strictly increasing"; return r;
    }

  // Grid consistency: C_k(u_l) must equal D_l(v_k) (= Q_{k,l}). Average the two for the grid.
  r.grid.assign(static_cast<std::size_t>(K) * L, Point3{});
  double worst = 0.0;
  for (int k = 0; k < K; ++k) {
    const BsplineCurveData& Ck = network.uCurves[k];
    for (int l = 0; l < L; ++l) {
      const BsplineCurveData& Dl = network.vCurves[l];
      const Point3 pC = curvePoint(Ck.degree, Ck.poles, Ck.knots, network.uParams[l]);
      const Point3 pD = curvePoint(Dl.degree, Dl.poles, Dl.knots, network.vParams[k]);
      worst = std::max(worst, distance(pC, pD));
      r.grid[static_cast<std::size_t>(k) * L + l] = {0.5 * (pC.x + pD.x),
                                                     0.5 * (pC.y + pD.y),
                                                     0.5 * (pC.z + pD.z)};
    }
  }
  r.maxGridError = worst;
  if (worst > tol) {
    r.reason = "inconsistent network: curve intersections exceed tolerance";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational network consistency (grid checked with the RATIONAL evaluator).
// ─────────────────────────────────────────────────────────────────────────────

NetworkCheck verifyRationalNetwork(const CurveNetwork& network, double tol) {
  NetworkCheck r;
  const int K = static_cast<int>(network.uCurves.size());
  const int L = static_cast<int>(network.vCurves.size());
  r.K = K;
  r.L = L;

  if (K < 2 || L < 2) { r.reason = "need at least 2 curves in each direction"; return r; }
  if (static_cast<int>(network.vParams.size()) != K) { r.reason = "vParams size != K"; return r; }
  if (static_cast<int>(network.uParams.size()) != L) { r.reason = "uParams size != L"; return r; }

  // Rational + well-formed guard for both families (one strictly-positive weight per pole).
  auto wellFormedRat = [](const BsplineCurveData& c) {
    if (c.weights.size() != c.poles.size()) return false;  // non-rational or mismatched
    if (c.poles.empty()) return false;
    for (double w : c.weights)
      if (!(w > 0.0)) return false;
    return c.degree >= 1 &&
           static_cast<int>(c.knots.size()) == nPolesOf(c) + c.degree + 1 && nPolesOf(c) >= 1;
  };
  for (const BsplineCurveData& c : network.uCurves)
    if (!wellFormedRat(c)) { r.reason = "a u-curve is non-rational or malformed"; return r; }
  for (const BsplineCurveData& c : network.vCurves)
    if (!wellFormedRat(c)) { r.reason = "a v-curve is non-rational or malformed"; return r; }

  // Strictly-increasing station params (a proper monotone grid).
  for (int k = 1; k < K; ++k)
    if (!(network.vParams[k] > network.vParams[k - 1] + 1e-12)) {
      r.reason = "vParams not strictly increasing"; return r;
    }
  for (int l = 1; l < L; ++l)
    if (!(network.uParams[l] > network.uParams[l - 1] + 1e-12)) {
      r.reason = "uParams not strictly increasing"; return r;
    }

  // Grid consistency via the RATIONAL evaluator: C_k(u_l) must equal D_l(v_k) (= Q_{k,l}).
  r.grid.assign(static_cast<std::size_t>(K) * L, Point3{});
  double worst = 0.0;
  for (int k = 0; k < K; ++k) {
    const BsplineCurveData& Ck = network.uCurves[k];
    for (int l = 0; l < L; ++l) {
      const BsplineCurveData& Dl = network.vCurves[l];
      const Point3 pC = nurbsCurvePoint(Ck.degree, Ck.poles, Ck.weights, Ck.knots,
                                        network.uParams[l]);
      const Point3 pD = nurbsCurvePoint(Dl.degree, Dl.poles, Dl.weights, Dl.knots,
                                        network.vParams[k]);
      worst = std::max(worst, distance(pC, pD));
      r.grid[static_cast<std::size_t>(k) * L + l] = {0.5 * (pC.x + pD.x),
                                                     0.5 * (pC.y + pD.y),
                                                     0.5 * (pC.z + pD.z)};
    }
  }
  r.maxGridError = worst;
  if (worst > tol) {
    r.reason = "inconsistent rational network: curve intersections exceed tolerance";
    return r;
  }
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Gordon / network surface (boolean sum).
// ─────────────────────────────────────────────────────────────────────────────

GordonResult gordonSurface(const CurveNetwork& network, double tol,
                           int uInterpDegree, int vInterpDegree) {
  GordonResult r;

  // Step 1 — verify the network (declines honestly on inconsistent/degenerate input).
  const NetworkCheck chk = verifyNetwork(network, tol);
  r.maxGridError = chk.maxGridError;
  if (!chk.ok) { r.reason = chk.reason; return r; }
  const int K = chk.K, L = chk.L;

  // Step 2 — make each family compatible (shared degree/knots/N per family, exact).
  const SectionCompatibility compU = makeSectionsCompatible(network.uCurves);
  const SectionCompatibility compV = makeSectionsCompatible(network.vCurves);
  if (!compU.ok || !compV.ok) { r.reason = "family compatibility failed"; return r; }

  // Across-family interpolation degrees, clamped to the family sizes.
  int qV = vInterpDegree; if (qV > K - 1) qV = K - 1; if (qV < 1) qV = 1;  // across v (K curves)
  int qU = uInterpDegree; if (qU > L - 1) qU = L - 1; if (qU < 1) qU = 1;  // across u (L curves)

  // Step 3 — the three boolean-sum summands, all interpolated at the PRESCRIBED params.
  //   S_u: skin the K u-curves across v at vParams (u-shape carried in U).
  //   S_v: skin the L v-curves across u at uParams (v-shape carried in V).
  //   T  : tensor-product interpolant of the K×L grid at (uParams, vParams).
  BsplineSurfaceData Su, Sv, T;
  if (!interpFamilyAcross(compU.sections, network.vParams, qV, /*alongIsU=*/true, Su)) {
    r.reason = "singular u-family interpolation"; return r;
  }
  if (!interpFamilyAcross(compV.sections, network.uParams, qU, /*alongIsU=*/false, Sv)) {
    r.reason = "singular v-family interpolation"; return r;
  }
  if (!interpGrid(chk.grid, K, L, network.uParams, network.vParams, qU, qV, T)) {
    r.reason = "singular grid interpolation"; return r;
  }

  // Step 4 — bring all three to ONE common basis in each direction (exact Layer-1), then
  // form the Gordon net poles(Su) + poles(Sv) − poles(T). Unify pairwise: (Su,Sv), then
  // fold T onto that shared basis, then Su/Sv onto T's (idempotent once shared).
  unifyDirection(Su, Sv, ParamDir::U);
  unifyDirection(Su, Sv, ParamDir::V);
  unifyDirection(Su, T, ParamDir::U);
  unifyDirection(Su, T, ParamDir::V);
  unifyDirection(Sv, T, ParamDir::U);
  unifyDirection(Sv, T, ParamDir::V);
  // Su may now trail Sv/T in a direction where the later unify raised them; re-fold.
  unifyDirection(Su, Sv, ParamDir::U);
  unifyDirection(Su, Sv, ParamDir::V);
  unifyDirection(Su, T, ParamDir::U);
  unifyDirection(Su, T, ParamDir::V);

  if (!sameBasis(Su, Sv) || !sameBasis(Su, T)) {
    r.reason = "summands failed to reach a common basis"; return r;
  }

  BsplineSurfaceData G = Su;  // inherits the common basis/degrees/knots.
  const std::size_t n = Su.poles.size();
  for (std::size_t i = 0; i < n; ++i) {
    G.poles[i] = {Su.poles[i].x + Sv.poles[i].x - T.poles[i].x,
                  Su.poles[i].y + Sv.poles[i].y - T.poles[i].y,
                  Su.poles[i].z + Sv.poles[i].z - T.poles[i].z};
  }
  G.weights.clear();  // non-rational.

  r.surface = std::move(G);
  r.ok = true;
  return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// Rational Gordon / network surface (boolean sum in HOMOGENEOUS R⁴).
// ─────────────────────────────────────────────────────────────────────────────

GordonResult gordonRationalSurface(const CurveNetwork& network, double tol,
                                   int uInterpDegree, int vInterpDegree) {
  GordonResult r;

  // Step 1 — verify the rational network (grid checked with the rational evaluator).
  const NetworkCheck chk = verifyRationalNetwork(network, tol);
  r.maxGridError = chk.maxGridError;
  if (!chk.ok) { r.reason = chk.reason; return r; }
  const int K = chk.K, L = chk.L;

  // Step 2 — make each family compatible in HOMOGENEOUS space (rational-aware elevate/refine).
  const SectionCompatibility compU = makeRationalSectionsCompatible(network.uCurves);
  const SectionCompatibility compV = makeRationalSectionsCompatible(network.vCurves);
  if (!compU.ok || !compV.ok) { r.reason = "rational family compatibility failed"; return r; }

  int qV = vInterpDegree; if (qV > K - 1) qV = K - 1; if (qV < 1) qV = 1;  // across v (K curves)
  int qU = uInterpDegree; if (qU > L - 1) qU = L - 1; if (qU < 1) qU = 1;  // across u (L curves)

  // Homogeneous grid Qh_{k,l} = C_k^w(u_l): the value BOTH rational skins land at (must agree
  // homogeneously with D_l^w(v_k) for the boolean sum to cancel). Honest precondition: decline
  // if the two families disagree in HOMOGENEOUS space at the grid (a Euclidean-consistent but
  // weight-inconsistent network cannot be reproduced by the rational boolean sum).
  std::vector<Homog4> Qh(static_cast<std::size_t>(K) * L);
  double worstH = 0.0;
  for (int k = 0; k < K; ++k)
    for (int l = 0; l < L; ++l) {
      const Homog4 hC = homogCurvePoint(compU.sections[k], network.uParams[l]);
      const Homog4 hD = homogCurvePoint(compV.sections[l], network.vParams[k]);
      const double d = std::sqrt((hC.x - hD.x) * (hC.x - hD.x) + (hC.y - hD.y) * (hC.y - hD.y) +
                                 (hC.z - hD.z) * (hC.z - hD.z) + (hC.w - hD.w) * (hC.w - hD.w));
      worstH = std::max(worstH, d);
      Qh[static_cast<std::size_t>(k) * L + l] = hC;
    }
  if (worstH > tol) {
    r.reason = "rational network inconsistent in homogeneous (weight) space at the grid";
    return r;
  }

  // Step 3 — the three homogeneous boolean-sum summands.
  BsplineSurfaceData Su, Sv, T;
  if (!interpFamilyAcrossRational(compU.sections, network.vParams, qV, /*alongIsU=*/true, Su)) {
    r.reason = "singular rational u-family interpolation"; return r;
  }
  if (!interpFamilyAcrossRational(compV.sections, network.uParams, qU, /*alongIsU=*/false, Sv)) {
    r.reason = "singular rational v-family interpolation"; return r;
  }
  if (!interpGridRational(Qh, K, L, network.uParams, network.vParams, qU, qV, T)) {
    r.reason = "singular rational grid interpolation"; return r;
  }

  // Step 4 — common basis via the exact RATIONAL-AWARE Layer-1 ops (weights ride through).
  unifyDirection(Su, Sv, ParamDir::U);
  unifyDirection(Su, Sv, ParamDir::V);
  unifyDirection(Su, T, ParamDir::U);
  unifyDirection(Su, T, ParamDir::V);
  unifyDirection(Sv, T, ParamDir::U);
  unifyDirection(Sv, T, ParamDir::V);
  unifyDirection(Su, Sv, ParamDir::U);
  unifyDirection(Su, Sv, ParamDir::V);
  unifyDirection(Su, T, ParamDir::U);
  unifyDirection(Su, T, ParamDir::V);

  if (!sameBasis(Su, Sv) || !sameBasis(Su, T)) {
    r.reason = "rational summands failed to reach a common basis"; return r;
  }
  if (Su.weights.size() != Su.poles.size() || Sv.weights.size() != Sv.poles.size() ||
      T.weights.size() != T.poles.size()) {
    r.reason = "rational summand lost its weights under unification"; return r;
  }

  // Form the Gordon net in HOMOGENEOUS space: homog(G) = homog(Su) + homog(Sv) − homog(T),
  // then project back to (pole, weight). A projected non-positive weight is a documented guard.
  BsplineSurfaceData G = Su;  // inherits the common basis/degrees/knots.
  const std::size_t n = Su.poles.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Homog4 hu = liftH(Su.poles[i], Su.weights[i]);
    const Homog4 hv = liftH(Sv.poles[i], Sv.weights[i]);
    const Homog4 ht = liftH(T.poles[i], T.weights[i]);
    const double gx = hu.x + hv.x - ht.x;
    const double gy = hu.y + hv.y - ht.y;
    const double gz = hu.z + hv.z - ht.z;
    const double gw = hu.w + hv.w - ht.w;
    if (!(gw > 0.0)) { r.reason = "rational boolean sum produced a non-positive weight"; return r; }
    G.poles[i] = {gx / gw, gy / gw, gz / gw};
    G.weights[i] = gw;
  }

  r.surface = std::move(G);
  r.ok = true;
  return r;
}

}  // namespace cybercad::native::math

#endif  // CYBERCAD_HAS_NUMSCI

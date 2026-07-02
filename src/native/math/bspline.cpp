// SPDX-License-Identifier: Apache-2.0
//
// bspline.cpp — implementation of bspline.h.
// Algorithm references are cited per-function (The NURBS Book, 2nd ed.).
// OCCT-FREE; cross-checked numerically against BSplCLib / BSplSLib.
//
#include "bspline.h"

#include <algorithm>
#include <array>
#include <cmath>

namespace cybercad::native::math {
namespace {

// A weighted homogeneous point (w·P, w) used for rational evaluation.
struct Homog {
  double x = 0.0, y = 0.0, z = 0.0, w = 0.0;
  constexpr Homog operator+(const Homog& o) const noexcept { return {x + o.x, y + o.y, z + o.z, w + o.w}; }
  constexpr Homog operator*(double s) const noexcept { return {x * s, y * s, z * s, w * s}; }
  constexpr Homog& operator+=(const Homog& o) noexcept { x += o.x; y += o.y; z += o.z; w += o.w; return *this; }
};

constexpr Homog toHomog(const Point3& p, double w) noexcept { return {p.x * w, p.y * w, p.z * w, w}; }

// Binomial coefficients up to a small order, cached lazily. For the derivative
// orders used in CAD (typically ≤ 3) this is tiny.
double binomial(int n, int k) noexcept {
  if (k < 0 || k > n) return 0.0;
  double r = 1.0;
  for (int i = 0; i < k; ++i) r = r * (n - i) / (i + 1);
  return r;
}

}  // namespace

// ─── FindSpan (A2.1) ─────────────────────────────────────────────────────────
int findSpan(int n, int degree, double u, std::span<const double> knots) noexcept {
  // Special case: u at (or past) the right end maps to the last valid span.
  if (u >= knots[static_cast<std::size_t>(n) + 1]) return n;
  if (u <= knots[static_cast<std::size_t>(degree)]) return degree;
  // Binary search — the book's exact loop.
  int low = degree, high = n + 1;
  int mid = (low + high) / 2;
  while (u < knots[mid] || u >= knots[mid + 1]) {
    if (u < knots[mid]) high = mid; else low = mid;
    mid = (low + high) / 2;
  }
  return mid;
}

// ─── BasisFuns (A2.2) ────────────────────────────────────────────────────────
void basisFuns(int span, double u, int degree, std::span<const double> knots,
               std::span<double> out) noexcept {
  // out[0..degree]; uses O(p) scratch on the stack for typical degrees.
  std::array<double, 64> left{};
  std::array<double, 64> right{};
  out[0] = 1.0;
  for (int j = 1; j <= degree; ++j) {
    left[j] = u - knots[span + 1 - j];
    right[j] = knots[span + j] - u;
    double saved = 0.0;
    for (int r = 0; r < j; ++r) {
      const double temp = out[r] / (right[r + 1] + left[j - r]);
      out[r] = saved + right[r + 1] * temp;
      saved = left[j - r] * temp;
    }
    out[j] = saved;
  }
}

// ─── DersBasisFuns (A2.3) ────────────────────────────────────────────────────
// Cognitive complexity note: this is the irreducible A2.3 recurrence (nested
// loops over derivative order and basis index). Systems-band; kept 1:1 with the
// book so it can be checked against the reference. ~C.C. mid-20s.
void dersBasisFuns(int span, double u, int degree, int maxDeriv,
                   std::span<const double> knots, std::span<double> out) {
  const int p = degree;
  const int stride = p + 1;
  std::vector<double> ndu((p + 1) * (p + 1));
  std::vector<double> a(2 * (p + 1));
  std::array<double, 64> left{};
  std::array<double, 64> right{};
  auto NDU = [&](int r, int c) -> double& { return ndu[r * (p + 1) + c]; };

  NDU(0, 0) = 1.0;
  for (int j = 1; j <= p; ++j) {
    left[j] = u - knots[span + 1 - j];
    right[j] = knots[span + j] - u;
    double saved = 0.0;
    for (int r = 0; r < j; ++r) {
      NDU(j, r) = right[r + 1] + left[j - r];          // lower triangle: knot diffs
      const double temp = NDU(r, j - 1) / NDU(j, r);
      NDU(r, j) = saved + right[r + 1] * temp;         // upper triangle: basis funcs
      saved = left[j - r] * temp;
    }
    NDU(j, j) = saved;
  }

  // Zeroth derivatives = the basis functions themselves.
  for (int j = 0; j <= p; ++j) out[0 * stride + j] = NDU(j, p);

  // Derivatives via the A2.3 two-row (a[0], a[1]) coefficient recurrence.
  for (int r = 0; r <= p; ++r) {
    int s1 = 0, s2 = 1;
    a[0] = 1.0;
    for (int k = 1; k <= maxDeriv; ++k) {
      double d = 0.0;
      const int rk = r - k;
      const int pk = p - k;
      if (r >= k) {
        a[s2 * (p + 1) + 0] = a[s1 * (p + 1) + 0] / NDU(pk + 1, rk);
        d = a[s2 * (p + 1) + 0] * NDU(rk, pk);
      }
      const int j1 = (rk >= -1) ? 1 : -rk;
      const int j2 = (r - 1 <= pk) ? k - 1 : p - r;
      for (int j = j1; j <= j2; ++j) {
        a[s2 * (p + 1) + j] =
            (a[s1 * (p + 1) + j] - a[s1 * (p + 1) + (j - 1)]) / NDU(pk + 1, rk + j);
        d += a[s2 * (p + 1) + j] * NDU(rk + j, pk);
      }
      if (r <= pk) {
        a[s2 * (p + 1) + k] = -a[s1 * (p + 1) + (k - 1)] / NDU(pk + 1, r);
        d += a[s2 * (p + 1) + k] * NDU(r, pk);
      }
      out[k * stride + r] = d;
      std::swap(s1, s2);
    }
  }

  // Multiply by the factorial factors p!/(p-k)!.
  double factor = p;
  for (int k = 1; k <= maxDeriv; ++k) {
    for (int j = 0; j <= p; ++j) out[k * stride + j] *= factor;
    factor *= (p - k);
  }
}

// ─── CurvePoint (A3.1) ───────────────────────────────────────────────────────
Point3 curvePoint(int degree, std::span<const Point3> poles,
                  std::span<const double> knots, double u) noexcept {
  const int n = static_cast<int>(poles.size()) - 1;
  const int span = findSpan(n, degree, u, knots);
  std::array<double, 64> N{};
  basisFuns(span, u, degree, knots, {N.data(), static_cast<std::size_t>(degree + 1)});
  Vec3 c{};
  for (int i = 0; i <= degree; ++i) c += poles[span - degree + i].asVec() * N[i];
  return Point3{c.x, c.y, c.z};
}

// ─── CurvePoint via de Boor (Boehm knot insertion) ───────────────────────────
Point3 curvePointDeBoor(int degree, std::span<const Point3> poles,
                        std::span<const double> knots, double u) noexcept {
  const int n = static_cast<int>(poles.size()) - 1;
  const int span = findSpan(n, degree, u, knots);
  // Local copy of the p+1 affecting poles, then repeated affine interpolation.
  std::array<Vec3, 64> d{};
  for (int j = 0; j <= degree; ++j) d[j] = poles[span - degree + j].asVec();
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const int i = span - degree + j;
      const double denom = knots[i + degree - r + 1] - knots[i];
      const double alpha = denom > 0.0 ? (u - knots[i]) / denom : 0.0;
      d[j] = d[j - 1] * (1.0 - alpha) + d[j] * alpha;
    }
  }
  return Point3{d[degree].x, d[degree].y, d[degree].z};
}

// ─── CurveDerivsAlg1 (A3.2) ──────────────────────────────────────────────────
void curveDerivs(int degree, std::span<const Point3> poles,
                 std::span<const double> knots, double u, int maxDeriv,
                 std::span<Vec3> out) {
  const int n = static_cast<int>(poles.size()) - 1;
  const int du = std::min(maxDeriv, degree);
  for (int k = degree + 1; k <= maxDeriv; ++k) out[k] = Vec3{};  // zero above degree
  const int span = findSpan(n, degree, u, knots);
  const int stride = degree + 1;
  std::vector<double> ders((maxDeriv + 1) * stride);
  dersBasisFuns(span, u, degree, du, knots, ders);
  for (int k = 0; k <= du; ++k) {
    Vec3 ck{};
    for (int j = 0; j <= degree; ++j)
      ck += poles[span - degree + j].asVec() * ders[k * stride + j];
    out[k] = ck;
  }
}

// ─── NURBS curve point (§4.1) — evaluate in homogeneous space then project ───
Point3 nurbsCurvePoint(int degree, std::span<const Point3> poles,
                       std::span<const double> weights,
                       std::span<const double> knots, double u) noexcept {
  const int n = static_cast<int>(poles.size()) - 1;
  const int span = findSpan(n, degree, u, knots);
  std::array<double, 64> N{};
  basisFuns(span, u, degree, knots, {N.data(), static_cast<std::size_t>(degree + 1)});
  Homog cw{};
  for (int i = 0; i <= degree; ++i) {
    const int idx = span - degree + i;
    cw += toHomog(poles[idx], weights[idx]) * N[i];
  }
  const double invW = 1.0 / cw.w;
  return Point3{cw.x * invW, cw.y * invW, cw.z * invW};
}

// ─── NURBS curve derivatives (A4.2 quotient rule) ────────────────────────────
void nurbsCurveDerivs(int degree, std::span<const Point3> poles,
                      std::span<const double> weights,
                      std::span<const double> knots, double u, int maxDeriv,
                      std::span<Vec3> out) {
  // Derivatives of the homogeneous curve Cw = (A, w): Aders[k] = numerator, wders[k].
  const int n = static_cast<int>(poles.size()) - 1;
  const int du = std::min(maxDeriv, degree);
  const int span = findSpan(n, degree, u, knots);
  const int stride = degree + 1;
  std::vector<double> nd((du + 1) * stride);
  dersBasisFuns(span, u, degree, du, knots, nd);

  std::vector<Vec3> Aders(maxDeriv + 1, Vec3{});
  std::vector<double> wders(maxDeriv + 1, 0.0);
  for (int k = 0; k <= du; ++k) {
    Vec3 a{};
    double w = 0.0;
    for (int j = 0; j <= degree; ++j) {
      const int idx = span - degree + j;
      const double b = nd[k * stride + j];
      a += poles[idx].asVec() * (weights[idx] * b);
      w += weights[idx] * b;
    }
    Aders[k] = a;
    wders[k] = w;
  }

  // Ck = (Aders[k] − Σ_{i=1}^{k} C(k,i)·wders[i]·C[k-i]) / w0.  (A4.2)
  const double invW0 = 1.0 / wders[0];
  for (int k = 0; k <= maxDeriv; ++k) {
    Vec3 v = Aders[k];
    for (int i = 1; i <= k; ++i)
      v -= out[k - i] * (binomial(k, i) * wders[i]);
    out[k] = v * invW0;
  }
}

// ─── SurfacePoint (A3.5) ─────────────────────────────────────────────────────
Point3 surfacePoint(int degreeU, int degreeV, const SurfaceGrid& grid,
                    std::span<const double> knotsU, std::span<const double> knotsV,
                    double u, double v) noexcept {
  const int nU = grid.nRows - 1;
  const int nV = grid.nCols - 1;
  const int spanU = findSpan(nU, degreeU, u, knotsU);
  const int spanV = findSpan(nV, degreeV, v, knotsV);
  std::array<double, 64> Nu{};
  std::array<double, 64> Nv{};
  basisFuns(spanU, u, degreeU, knotsU, {Nu.data(), static_cast<std::size_t>(degreeU + 1)});
  basisFuns(spanV, v, degreeV, knotsV, {Nv.data(), static_cast<std::size_t>(degreeV + 1)});
  Vec3 s{};
  for (int i = 0; i <= degreeU; ++i) {
    Vec3 tmp{};
    const int ii = spanU - degreeU + i;
    for (int j = 0; j <= degreeV; ++j)
      tmp += grid.pole(ii, spanV - degreeV + j).asVec() * Nv[j];
    s += tmp * Nu[i];
  }
  return Point3{s.x, s.y, s.z};
}

// ─── SurfaceDerivsAlg1 (A3.6) ────────────────────────────────────────────────
// Systems-band: the A3.6 tensor-product derivative loop. Kept 1:1 with the book.
void surfaceDerivs(int degreeU, int degreeV, const SurfaceGrid& grid,
                   std::span<const double> knotsU, std::span<const double> knotsV,
                   double u, double v, int maxDeriv, std::span<Vec3> out) {
  const int nU = grid.nRows - 1;
  const int nV = grid.nCols - 1;
  const int du = std::min(maxDeriv, degreeU);
  const int dv = std::min(maxDeriv, degreeV);
  const int outStride = maxDeriv + 1;
  for (int k = 0; k <= maxDeriv; ++k)
    for (int l = 0; l <= maxDeriv; ++l) out[k * outStride + l] = Vec3{};

  const int spanU = findSpan(nU, degreeU, u, knotsU);
  const int spanV = findSpan(nV, degreeV, v, knotsV);
  const int su = degreeU + 1, sv = degreeV + 1;
  std::vector<double> Nu((du + 1) * su);
  std::vector<double> Nv((dv + 1) * sv);
  dersBasisFuns(spanU, u, degreeU, du, knotsU, Nu);
  dersBasisFuns(spanV, v, degreeV, dv, knotsV, Nv);

  std::vector<Vec3> temp(degreeV + 1);
  for (int k = 0; k <= du; ++k) {
    for (int s = 0; s <= degreeV; ++s) {
      temp[s] = Vec3{};
      for (int r = 0; r <= degreeU; ++r)
        temp[s] += grid.pole(spanU - degreeU + r, spanV - degreeV + s).asVec() * Nu[k * su + r];
    }
    const int dd = std::min(maxDeriv - k, dv);
    for (int l = 0; l <= dd; ++l) {
      Vec3 acc{};
      for (int s = 0; s <= degreeV; ++s) acc += temp[s] * Nv[l * sv + s];
      out[k * outStride + l] = acc;
    }
  }
}

// ─── NURBS surface point (§4.4) ──────────────────────────────────────────────
Point3 nurbsSurfacePoint(int degreeU, int degreeV, const SurfaceGrid& grid,
                         std::span<const double> weights,
                         std::span<const double> knotsU, std::span<const double> knotsV,
                         double u, double v) noexcept {
  const int nU = grid.nRows - 1;
  const int nV = grid.nCols - 1;
  const int spanU = findSpan(nU, degreeU, u, knotsU);
  const int spanV = findSpan(nV, degreeV, v, knotsV);
  std::array<double, 64> Nu{};
  std::array<double, 64> Nv{};
  basisFuns(spanU, u, degreeU, knotsU, {Nu.data(), static_cast<std::size_t>(degreeU + 1)});
  basisFuns(spanV, v, degreeV, knotsV, {Nv.data(), static_cast<std::size_t>(degreeV + 1)});
  Homog sw{};
  for (int i = 0; i <= degreeU; ++i) {
    Homog tmp{};
    const int ii = spanU - degreeU + i;
    for (int j = 0; j <= degreeV; ++j) {
      const int jj = spanV - degreeV + j;
      tmp += toHomog(grid.pole(ii, jj), weights[static_cast<std::size_t>(ii) * grid.nCols + jj]) * Nv[j];
    }
    sw += tmp * Nu[i];
  }
  const double invW = 1.0 / sw.w;
  return Point3{sw.x * invW, sw.y * invW, sw.z * invW};
}

// ─── NURBS surface derivatives (A4.4 quotient rule) ──────────────────────────
// Systems-band: the two nested binomial correction sums of A4.4 over the
// tensor grid. Irreducible; kept explicit and documented.
void nurbsSurfaceDerivs(int degreeU, int degreeV, const SurfaceGrid& grid,
                        std::span<const double> weights,
                        std::span<const double> knotsU, std::span<const double> knotsV,
                        double u, double v, int maxDeriv, std::span<Vec3> out) {
  const int outStride = maxDeriv + 1;
  const std::size_t count = static_cast<std::size_t>(outStride) * outStride;

  // 1) Homogeneous surface derivatives: Aders (weighted numerator) and wders.
  //    We reuse surfaceDerivs on the weighted poles and, separately, the scalar
  //    weight field. To avoid a second grid type, compute both here directly.
  const int nU = grid.nRows - 1;
  const int nV = grid.nCols - 1;
  const int du = std::min(maxDeriv, degreeU);
  const int dv = std::min(maxDeriv, degreeV);
  const int spanU = findSpan(nU, degreeU, u, knotsU);
  const int spanV = findSpan(nV, degreeV, v, knotsV);
  const int su = degreeU + 1, sv = degreeV + 1;
  std::vector<double> Nu((du + 1) * su);
  std::vector<double> Nv((dv + 1) * sv);
  dersBasisFuns(spanU, u, degreeU, du, knotsU, Nu);
  dersBasisFuns(spanV, v, degreeV, dv, knotsV, Nv);

  std::vector<Vec3> Aders(count, Vec3{});
  std::vector<double> wders(count, 0.0);
  std::vector<Vec3> tempA(degreeV + 1);
  std::vector<double> tempW(degreeV + 1);
  for (int k = 0; k <= du; ++k) {
    for (int s = 0; s <= degreeV; ++s) {
      tempA[s] = Vec3{}; tempW[s] = 0.0;
      for (int r = 0; r <= degreeU; ++r) {
        const int ii = spanU - degreeU + r;
        const int jj = spanV - degreeV + s;
        const double w = weights[static_cast<std::size_t>(ii) * grid.nCols + jj];
        const double b = Nu[k * su + r];
        tempA[s] += grid.pole(ii, jj).asVec() * (w * b);
        tempW[s] += w * b;
      }
    }
    const int dd = std::min(maxDeriv - k, dv);
    for (int l = 0; l <= dd; ++l) {
      Vec3 a{}; double w = 0.0;
      for (int s = 0; s <= degreeV; ++s) { a += tempA[s] * Nv[l * sv + s]; w += tempW[s] * Nv[l * sv + s]; }
      Aders[k * outStride + l] = a;
      wders[k * outStride + l] = w;
    }
  }

  // 2) Quotient rule A4.4:
  //    SKL = ( Aders[k][l]
  //            − Σ_{j=1}^{l} C(l,j) w[0][j] S[k][l-j]
  //            − Σ_{i=1}^{k} C(k,i) w[i][0] S[k-i][l]
  //            − Σ_{i=1}^{k} C(k,i) Σ_{j=1}^{l} C(l,j) w[i][j] S[k-i][l-j] ) / w[0][0]
  const double invW0 = 1.0 / wders[0];
  for (int k = 0; k <= maxDeriv; ++k) {
    for (int l = 0; l <= maxDeriv; ++l) {
      Vec3 v2 = Aders[k * outStride + l];
      for (int j = 1; j <= l; ++j)
        v2 -= out[k * outStride + (l - j)] * (binomial(l, j) * wders[0 * outStride + j]);
      for (int i = 1; i <= k; ++i) {
        v2 -= out[(k - i) * outStride + l] * (binomial(k, i) * wders[i * outStride + 0]);
        Vec3 inner{};
        for (int j = 1; j <= l; ++j)
          inner += out[(k - i) * outStride + (l - j)] * (binomial(l, j) * wders[i * outStride + j]);
        v2 -= inner * binomial(k, i);
      }
      out[k * outStride + l] = v2 * invW0;
    }
  }
}

// ─── Surface normal = normalize(Su × Sv) ─────────────────────────────────────
Dir3 surfaceNormal(int degreeU, int degreeV, const SurfaceGrid& grid,
                   std::span<const double> weights,
                   std::span<const double> knotsU, std::span<const double> knotsV,
                   double u, double v) noexcept {
  constexpr int md = 1;
  std::array<Vec3, 4> d{};  // 2x2 grid: [00,01,10,11]
  if (weights.empty())
    surfaceDerivs(degreeU, degreeV, grid, knotsU, knotsV, u, v, md, d);
  else
    nurbsSurfaceDerivs(degreeU, degreeV, grid, weights, knotsU, knotsV, u, v, md, d);
  const Vec3 su = d[1 * (md + 1) + 0];  // ∂/∂u at [1][0]
  const Vec3 sv = d[0 * (md + 1) + 1];  // ∂/∂v at [0][1]
  return Dir3{cross(su, sv)};
}

}  // namespace cybercad::native::math

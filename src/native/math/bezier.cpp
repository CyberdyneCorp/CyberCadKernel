// SPDX-License-Identifier: Apache-2.0
//
// bezier.cpp — implementation of bezier.h. de Casteljau + hodograph.
// OCCT-FREE.
//
#include "bezier.h"

#include <algorithm>
#include <vector>

namespace cybercad::native::math {
namespace {

struct Homog {
  double x = 0, y = 0, z = 0, w = 0;
  Homog operator*(double s) const noexcept { return {x * s, y * s, z * s, w * s}; }
};
Homog lerpH(const Homog& a, const Homog& b, double t) noexcept {
  const double s = 1.0 - t;
  return {a.x * s + b.x * t, a.y * s + b.y * t, a.z * s + b.z * t, a.w * s + b.w * t};
}

}  // namespace

// ─── de Casteljau curve point (A1.5) ─────────────────────────────────────────
Point3 bezierPoint(std::span<const Point3> poles, double t) noexcept {
  std::vector<Vec3> d(poles.size());
  for (std::size_t i = 0; i < poles.size(); ++i) d[i] = poles[i].asVec();
  for (std::size_t r = 1; r < poles.size(); ++r)
    for (std::size_t i = 0; i + r < poles.size(); ++i)
      d[i] = lerp(d[i], d[i + 1], t);
  return Point3{d[0].x, d[0].y, d[0].z};
}

Point3 rationalBezierPoint(std::span<const Point3> poles,
                           std::span<const double> weights, double t) noexcept {
  std::vector<Homog> d(poles.size());
  for (std::size_t i = 0; i < poles.size(); ++i) {
    const double w = weights[i];
    d[i] = {poles[i].x * w, poles[i].y * w, poles[i].z * w, w};
  }
  for (std::size_t r = 1; r < poles.size(); ++r)
    for (std::size_t i = 0; i + r < poles.size(); ++i)
      d[i] = lerpH(d[i], d[i + 1], t);
  const double invW = 1.0 / d[0].w;
  return Point3{d[0].x * invW, d[0].y * invW, d[0].z * invW};
}

// ─── Curve derivatives via repeated forward differences (hodograph) ──────────
// The k-th derivative of a degree-n Bézier is (n!/(n-k)!) times the Bézier of
// degree n-k over the k-th forward differences of the control points, evaluated
// at t. We build the difference poles then de Casteljau each order.
void bezierDerivs(std::span<const Point3> poles, double t, int maxDeriv,
                  std::span<Vec3> out) {
  const int n = static_cast<int>(poles.size()) - 1;  // degree
  // diff[k][i] = k-th forward difference; diff[0] = poles.
  std::vector<Vec3> cur(poles.size());
  for (std::size_t i = 0; i < poles.size(); ++i) cur[i] = poles[i].asVec();

  double factor = 1.0;  // n!/(n-k)!
  for (int k = 0; k <= maxDeriv; ++k) {
    if (k > n) { out[k] = Vec3{}; continue; }
    // de Casteljau on the current difference poles (there are n-k+1 of them).
    std::vector<Vec3> d(cur.begin(), cur.begin() + (n - k + 1));
    for (int r = 1; r <= n - k; ++r)
      for (int i = 0; i + r <= n - k; ++i)
        d[i] = lerp(d[i], d[i + 1], t);
    out[k] = d[0] * factor;
    // Advance to (k+1)-th forward differences for the next iteration.
    if (k < maxDeriv && k < n) {
      for (int i = 0; i + 1 <= n - k; ++i) cur[i] = cur[i + 1] - cur[i];
      factor *= (n - k);
    }
  }
}

// ─── Tensor-product surface point ────────────────────────────────────────────
Point3 bezierSurfacePoint(std::span<const Point3> poles, int nRows, int nCols,
                          double u, double v) noexcept {
  // Evaluate each U-row of control points at v, then de Casteljau in u.
  std::vector<Vec3> rowPts(nRows);
  std::vector<Vec3> scratch(nCols);
  for (int i = 0; i < nRows; ++i) {
    for (int j = 0; j < nCols; ++j) scratch[j] = poles[static_cast<std::size_t>(i) * nCols + j].asVec();
    for (int r = 1; r < nCols; ++r)
      for (int j = 0; j + r < nCols; ++j) scratch[j] = lerp(scratch[j], scratch[j + 1], v);
    rowPts[i] = scratch[0];
  }
  for (int r = 1; r < nRows; ++r)
    for (int i = 0; i + r < nRows; ++i) rowPts[i] = lerp(rowPts[i], rowPts[i + 1], u);
  return Point3{rowPts[0].x, rowPts[0].y, rowPts[0].z};
}

Point3 rationalBezierSurfacePoint(std::span<const Point3> poles,
                                  std::span<const double> weights,
                                  int nRows, int nCols, double u, double v) noexcept {
  std::vector<Homog> rowPts(nRows);
  std::vector<Homog> scratch(nCols);
  for (int i = 0; i < nRows; ++i) {
    for (int j = 0; j < nCols; ++j) {
      const std::size_t idx = static_cast<std::size_t>(i) * nCols + j;
      const double w = weights[idx];
      scratch[j] = {poles[idx].x * w, poles[idx].y * w, poles[idx].z * w, w};
    }
    for (int r = 1; r < nCols; ++r)
      for (int j = 0; j + r < nCols; ++j) scratch[j] = lerpH(scratch[j], scratch[j + 1], v);
    rowPts[i] = scratch[0];
  }
  for (int r = 1; r < nRows; ++r)
    for (int i = 0; i + r < nRows; ++i) rowPts[i] = lerpH(rowPts[i], rowPts[i + 1], u);
  const double invW = 1.0 / rowPts[0].w;
  return Point3{rowPts[0].x * invW, rowPts[0].y * invW, rowPts[0].z * invW};
}

// ─── Surface first derivatives + normal ──────────────────────────────────────
BezierSurfaceD1 bezierSurfaceD1(std::span<const Point3> poles, int nRows, int nCols,
                                double u, double v) noexcept {
  const int degU = nRows - 1;
  const int degV = nCols - 1;

  // ∂/∂u: difference along rows (degU·ΔP in u), then evaluate the reduced grid.
  // Build the u-derivative control net Q[i][j] = degU*(P[i+1][j]-P[i][j]).
  auto P = [&](int i, int j) { return poles[static_cast<std::size_t>(i) * nCols + j].asVec(); };

  // Point + ∂/∂v: evaluate v-hodograph on full u-net, and value.
  // We reuse a helper that de Casteljaus a length-`len` array in place.
  auto deCasteljau = [](std::vector<Vec3>& a, int len, double t) {
    for (int r = 1; r < len; ++r)
      for (int i = 0; i + r < len; ++i) a[i] = lerp(a[i], a[i + 1], t);
    return a[0];
  };

  // Value: eval each row at v, then rows at u.
  std::vector<Vec3> col(nRows);
  std::vector<Vec3> tmp(std::max(nRows, nCols));
  for (int i = 0; i < nRows; ++i) {
    for (int j = 0; j < nCols; ++j) tmp[j] = P(i, j);
    col[i] = deCasteljau(tmp, nCols, v);
  }
  std::vector<Vec3> colCopy = col;
  const Vec3 value = deCasteljau(colCopy, nRows, u);

  // ∂/∂u: eval each row at v (=col above), then take u-hodograph.
  std::vector<Vec3> duCtrl(nRows - 1);
  for (int i = 0; i < nRows - 1; ++i) duCtrl[i] = (col[i + 1] - col[i]) * static_cast<double>(degU);
  const Vec3 du = (nRows - 1 > 0) ? deCasteljau(duCtrl, nRows - 1, u) : Vec3{};

  // ∂/∂v: eval v-hodograph per row, then rows at u.
  std::vector<Vec3> dvRow(nRows);
  std::vector<Vec3> vh(std::max(nCols - 1, 1));
  for (int i = 0; i < nRows; ++i) {
    for (int j = 0; j < nCols - 1; ++j) vh[j] = (P(i, j + 1) - P(i, j)) * static_cast<double>(degV);
    dvRow[i] = (nCols - 1 > 0) ? deCasteljau(vh, nCols - 1, v) : Vec3{};
  }
  const Vec3 dv = deCasteljau(dvRow, nRows, u);

  return BezierSurfaceD1{Point3{value.x, value.y, value.z}, du, dv, Dir3{cross(du, dv)}};
}

}  // namespace cybercad::native::math

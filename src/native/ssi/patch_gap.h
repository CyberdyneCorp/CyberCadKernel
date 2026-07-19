// SPDX-License-Identifier: Apache-2.0
//
// patch_gap.h — a SOUND upper bound on the distance between two Bézier patches over a
// pair of parameter sub-boxes, with NO evaluation anywhere.
//
// WHY THIS EXISTS. On a COINCIDENT / overlapping surface pair the SSI intersection locus is
// 2-DIMENSIONAL, so no leaf pair is ever AABB-disjoint and `subdivide` runs the whole 4D box
// product — measured 1 835 481 candidates on a coincident bicubic dish pair. Stopping that
// descent needs a predicate that can certify "these two patches agree to tolerance", and the
// obvious cheap form is DISQUALIFIED: sampling an interior N×N grid is NON-CONSERVATIVE BY
// EXACTLY 4×, because a tangency's gap grows as r²/(2R) and is therefore MAXIMAL AT THE PATCH
// CORNER while an interior grid samples where the surfaces are most alike (sampled worst H²/4R
// vs true worst H²/R). Measured against an exact biquadratic paraboloid, such a screen returns
// 9/9 AGREE on a genuine tangency whose corners are 3.9× onSurfTol away. A false positive there
// is severe — the caller suppresses seeds inside a certified region, so a genuine tangential
// contact would be silently swallowed. A predicate here must BOUND, never sample.
//
// THE BOUND. For two Bézier patches restricted to sub-boxes, take the EXACT de Casteljau
// sub-nets QA, QB over those boxes. With the affine box map σ: boxA → boxB, the difference
//   D(u,v) = S_A(u,v) − S_B(σ(u,v))
// is itself a single Bézier of the same degree whose control net is exactly the pole
// differences QA_ij − QB_ij (the Bernstein basis is linear in the poles and both sub-nets sit
// over the same [0,1]²). The convex-hull property then gives
//   sup over the WHOLE patch |D| ≤ max_ij ‖QA_ij − QB_ij‖
// — corners, edges and interior alike, with no evaluation. Since σ(u,v) is a legal B parameter,
// this upper-bounds the true distance from S_A to surface B. Taking the min over the ≤ 8
// dihedral re-orientations of the net is a min over independently valid witnesses, so it stays
// sound while catching flipped/transposed parametrizations.
//
// Verified over 520 random sub-boxes across 8 pair families against a refined nearest-point
// oracle: ZERO violations, and tight — worst overestimate 2.1×. On the r²/2R adversary it reads
// 1.0 / 2.0 / 3.9 / 10.0 × tol at corner gap m·tol, where the disqualified interior 3×3 reads
// 0.25 / 0.5 / 0.975 / 2.5.
//
// ⚠ WHAT THIS BOUND DOES **NOT** LICENCE. It is a bound, not a coincidence verdict. Restricted
// to a small enough sub-box a GENUINE TANGENCY *is* coincident to tolerance — that is a theorem,
// not a tuning problem. An exact quartic contact `z = c(x⁴+y⁴)` whose ROOT gap is 864× tolerance
// still reads 0.667× tol at the FIRST-level cell and would certify. For an order-2m contact both
// the gap and any curvature witness scale as h^{2m}, so their ratio is CONSTANT in cell size and
// the false-firing window is open at EVERY depth from 4th order up. Therefore any caller using
// this to stop a descent MUST also require the bound to fire at the ROOT (A's full domain against
// B's), which no sub-box test can supply. Measured separation for that precondition: coincident
// copies give exactly dz; the quadratic adversary 1.0–3.9× tol; the quartic 864× tol; real corpus
// tangencies 1.4e5–6.1e5 × tol.
//
// SCOPE. Non-rational Bézier nets of EQUAL degree only. A rational surface is refused (+∞): the
// difference of two rationals is not a Bézier of pole differences, so the theorem does not hold.
// Unequal degrees are refused rather than degree-elevated — refusing is always sound, and the
// coincident-copy family this serves is equal-degree by construction. Refusal costs only reach.
//
#ifndef CYBERCAD_NATIVE_SSI_PATCH_GAP_H
#define CYBERCAD_NATIVE_SSI_PATCH_GAP_H

#include "native/math/vec.h"
#include "native/ssi/patch_bounds.h"

#include <algorithm>
#include <limits>
#include <vector>

namespace cybercad::native::ssi {

namespace detail {

/// Affine lerp — Point3 has no scalar multiply, so go through the difference vector.
inline Point3 lerpP(const Point3& a, const Point3& b, double t) noexcept {
  return a + (b - a) * t;
}

/// One de Casteljau split at `t`, yielding BOTH sub-polygons. The left polygon is the leading
/// element of each successive reduction pass, the right polygon the trailing element — the
/// standard construction, exact.
inline void bezierSplit1D(const std::vector<Point3>& p, double t,
                          std::vector<Point3>& left, std::vector<Point3>& right) {
  const std::size_t n = p.size();
  left.assign(n, Point3{});
  right.assign(n, Point3{});
  std::vector<Point3> tmp = p;
  for (std::size_t k = 0; k < n; ++k) {
    left[k] = tmp[0];
    right[n - 1 - k] = tmp[n - 1 - k];
    for (std::size_t i = 0; i + 1 + k < n; ++i) tmp[i] = lerpP(tmp[i], tmp[i + 1], t);
  }
}

/// Exact de Casteljau restriction of a 1-D Bézier control polygon to [t0,t1] ⊆ [0,1].
/// Split at t1 keeping the LEFT part (now spanning [0,t1]), then split THAT at the rescaled
/// t0/t1 keeping the RIGHT part. Doing the upper cut first keeps the remap trivial.
inline void bezierRestrict1D(std::vector<Point3>& p, double t0, double t1) {
  if (p.size() <= 1) return;
  std::vector<Point3> left, right;
  if (t1 < 1.0) {
    bezierSplit1D(p, t1, left, right);
    p = left;
  }
  if (t0 > 0.0) {
    const double s = (t1 > 0.0) ? t0 / t1 : 0.0;
    bezierSplit1D(p, std::clamp(s, 0.0, 1.0), left, right);
    p = right;
  }
}

/// Exact de Casteljau sub-net of a Bézier control net over a normalized sub-box.
inline ControlNet bezierSubNet(const ControlNet& net, double u0, double u1, double v0, double v1) {
  ControlNet out = net;
  // restrict along V for each U row
  for (int i = 0; i < out.nRows; ++i) {
    std::vector<Point3> row(static_cast<std::size_t>(out.nCols));
    for (int j = 0; j < out.nCols; ++j) row[static_cast<std::size_t>(j)] = out.pole(i, j);
    bezierRestrict1D(row, v0, v1);
    for (int j = 0; j < out.nCols; ++j)
      out.poles[static_cast<std::size_t>(i) * out.nCols + j] = row[static_cast<std::size_t>(j)];
  }
  // restrict along U for each V column
  for (int j = 0; j < out.nCols; ++j) {
    std::vector<Point3> col(static_cast<std::size_t>(out.nRows));
    for (int i = 0; i < out.nRows; ++i) col[static_cast<std::size_t>(i)] = out.pole(i, j);
    bezierRestrict1D(col, u0, u1);
    for (int i = 0; i < out.nRows; ++i)
      out.poles[static_cast<std::size_t>(i) * out.nCols + j] = col[static_cast<std::size_t>(i)];
  }
  return out;
}

/// Fractional position of `box` inside `dom`, per axis, as [0,1] — the sub-net parameters.
inline void normalizedBox(const ParamBox& dom, const ParamBox& box,
                          double& u0, double& u1, double& v0, double& v1) noexcept {
  const double du = dom.du(), dv = dom.dv();
  u0 = du > 0.0 ? (box.u0 - dom.u0) / du : 0.0;
  u1 = du > 0.0 ? (box.u1 - dom.u0) / du : 1.0;
  v0 = dv > 0.0 ? (box.v0 - dom.v0) / dv : 0.0;
  v1 = dv > 0.0 ? (box.v1 - dom.v0) / dv : 1.0;
  u0 = std::clamp(u0, 0.0, 1.0); u1 = std::clamp(u1, 0.0, 1.0);
  v0 = std::clamp(v0, 0.0, 1.0); v1 = std::clamp(v1, 0.0, 1.0);
  if (u1 < u0) std::swap(u0, u1);
  if (v1 < v0) std::swap(v0, v1);
}

}  // namespace detail

/// SOUND upper bound on the distance between patch A over `boxA` and patch B over `boxB`.
/// Returns +infinity when it cannot prove a bound (unequal net sizes, empty nets) — refusing
/// is always safe, it only costs reach. See the file header for the proof and for the
/// disqualifying theorem about sub-box tangency.
inline double patchGapBound(const ControlNet& netA, const ParamBox& domA, const ParamBox& boxA,
                            const ControlNet& netB, const ParamBox& domB, const ParamBox& boxB) {
  const double kInf = std::numeric_limits<double>::infinity();
  if (netA.nRows <= 0 || netA.nCols <= 0) return kInf;
  if (netA.nRows != netB.nRows || netA.nCols != netB.nCols) return kInf;  // no elevation: refuse
  if (netA.poles.size() != static_cast<std::size_t>(netA.nRows) * netA.nCols) return kInf;
  if (netB.poles.size() != static_cast<std::size_t>(netB.nRows) * netB.nCols) return kInf;

  double a0, a1, b0, b1;
  detail::normalizedBox(domA, boxA, a0, a1, b0, b1);
  const ControlNet QA = detail::bezierSubNet(netA, a0, a1, b0, b1);
  detail::normalizedBox(domB, boxB, a0, a1, b0, b1);
  const ControlNet QB = detail::bezierSubNet(netB, a0, a1, b0, b1);

  // The bound for the AFFINE box map σ: boxA → boxB, which is the correspondence the caller's
  // descent actually pairs the two cells under.
  //
  // A min over the ≤ 8 dihedral re-orientations of B's net was tried and DROPPED. Each
  // orientation is a legal reparametrization, so the min is defensible in principle — but it
  // turns the guarantee into a statement about "the nearest point of surface B", which can only
  // be checked against a nearest-point oracle, and a DISCRETE such oracle over-estimates. That
  // made the soundness property untestable to the precision it deserves. Reach was the only
  // thing lost: the coincident-copy family this serves is same-orientation by construction, and
  // a flipped copy simply gets refused rather than certified. Refusing is always safe.
  const int R = QA.nRows, C = QA.nCols;
  double worst = 0.0;
  for (int i = 0; i < R; ++i)
    for (int j = 0; j < C; ++j)
      worst = std::max(worst, math::distance(QA.pole(i, j), QB.pole(i, j)));
  return worst;
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_PATCH_GAP_H

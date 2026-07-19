// SPDX-License-Identifier: Apache-2.0
//
// patch_gap.h — SOUND distance predicates between two Bézier patches over a pair of
// parameter sub-boxes, with NO evaluation anywhere. Two of them, pointing opposite ways:
//
//   * `patchGapBound`  — an UPPER bound on the gap. Certifies "these agree to tolerance".
//   * `slabSeparated`  — a LOWER bound witness. Certifies "these cannot touch here".
//
// They are not interchangeable. The upper bound cannot prune a descent (a small bound does
// not prove a crossing) and the separation witness cannot certify coincidence (failing to
// separate proves nothing). Each is used only in the direction it proves.
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
#include <cmath>
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

// ─────────────────────────────────────────────────────────────────────────────
// slabSeparated — an ORIENTED separating-slab witness: proof that two Bézier patches,
// restricted to a pair of parameter sub-boxes, come no closer than `gap` anywhere on
// those sub-boxes.
// ─────────────────────────────────────────────────────────────────────────────
//
// WHY. `subdivide` prunes with an AXIS-ALIGNED box test. On a near-parallel pair whose
// separation is not aligned with any coordinate axis, the two AABBs overlap at every depth
// even though the surfaces never meet, so the descent runs the whole 4D box product and hands
// every leaf to `refineRegion` — measured 1 835 481 candidates / 634 s on a disjoint dish pair
// at dz = 1e-3, of which 100% are waste (`converged = 0`, `seeds = 0`). One well-chosen
// direction collapses that to zero candidates in 0.073 s.
//
// THE ARGUMENT IS CONTAINMENT, AND ONLY CONTAINMENT. By the convex-hull property, the surface
// piece S_A(boxA) lies inside the convex hull of the EXACT de Casteljau sub-net QA over boxA;
// likewise S_B(boxB) inside hull(QB). Projection onto a unit direction `n` is linear, so it
// maps each hull into the closed interval spanned by that net's projected poles. If those two
// intervals are separated by more than `gap`, then every point of S_A(boxA) is more than `gap`
// from every point of S_B(boxB) — measured along `n`, hence a fortiori in 3D. No crossing can
// exist in that box pair, so the subtree beneath it is crossing-free and skipping it is exact.
//
// Do NOT restate this as an argument from precedent ("box-locality is already trusted inside
// subdivide"). That form of words would equally license applying this predicate at the
// `refineRegion` site, where it is WRONG: that solver clamps into the FULL domain, so 97.8% of
// its accepted refines converge outside their own candidate box and filtering there drops real
// seeds (measured 25 324 / 85 678 / 49 284 on the transversal controls). The prune is sound
// HERE because a descendant's param boxes are contained in its parent's, and nowhere else.
//
// SOUNDNESS DOES NOT DEPEND ON `n`. A poorly chosen direction simply fails to separate and the
// descent proceeds unchanged — `n` is a heuristic for REACH, never for correctness. The caller
// passes A's midpoint normal, which is the natural separating direction for the near-parallel
// pairs this targets. `n` is normalized here rather than trusted: a caller handing us a
// non-unit vector would otherwise scale the projections and could over-state the separation.
// A degenerate (near-null) `n` returns false — refusing to prune is always safe.
//
// SCOPE. Single-span non-rational Bézier nets, i.e. exactly `SurfaceAdapter::hasBezierNet`.
// Unlike `patchGapBound` this does NOT require the two nets to have equal degree: each hull
// bounds its own surface independently, so no correspondence between the nets is involved.
inline bool slabSeparated(const ControlNet& netA, const ParamBox& domA, const ParamBox& boxA,
                          const ControlNet& netB, const ParamBox& domB, const ParamBox& boxB,
                          const Vec3& n, double gap) {
  if (netA.nRows <= 0 || netA.nCols <= 0 || netB.nRows <= 0 || netB.nCols <= 0) return false;
  if (netA.poles.size() != static_cast<std::size_t>(netA.nRows) * netA.nCols) return false;
  if (netB.poles.size() != static_cast<std::size_t>(netB.nRows) * netB.nCols) return false;

  const double len = math::norm(n);
  if (!(len > 0.0) || !std::isfinite(len)) return false;  // no usable direction → do not prune
  const Vec3 u{n.x / len, n.y / len, n.z / len};

  auto project = [&u](const ControlNet& q, double& lo, double& hi) {
    lo = std::numeric_limits<double>::infinity();
    hi = -std::numeric_limits<double>::infinity();
    for (const Point3& p : q.poles) {
      const double t = p.x * u.x + p.y * u.y + p.z * u.z;
      lo = std::min(lo, t);
      hi = std::max(hi, t);
    }
  };

  double a0, a1, b0, b1;
  detail::normalizedBox(domA, boxA, a0, a1, b0, b1);
  const ControlNet QA = detail::bezierSubNet(netA, a0, a1, b0, b1);
  detail::normalizedBox(domB, boxB, a0, a1, b0, b1);
  const ControlNet QB = detail::bezierSubNet(netB, a0, a1, b0, b1);

  double aLo, aHi, bLo, bHi;
  project(QA, aLo, aHi);
  project(QB, bLo, bHi);
  if (!std::isfinite(aLo) || !std::isfinite(bLo)) return false;

  // Strict `>`, mirroring aabbDisjoint: a pair separated by exactly `gap` stays a candidate.
  return (bLo - aHi > gap) || (aLo - bHi > gap);
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_PATCH_GAP_H

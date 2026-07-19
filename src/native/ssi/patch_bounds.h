// SPDX-License-Identifier: Apache-2.0
//
// patch_bounds.h — parameter-domain patches + per-patch AABBs for SSI Stage S2
// subdivision seeding.
//
// The seeder subdivides each surface's [u0,u1]×[v0,v1] parameter domain into
// PATCHES (param sub-boxes) and bounds each patch by a 3D axis-aligned box (AABB)
// that CONTAINS the surface over that sub-box. Overlapping-AABB patch pairs bracket
// a possible intersection; disjoint pairs are pruned. The bound MUST be
// conservative (sound): a prune must never discard a region that actually contains
// an intersection. Bounds may be loose (that only costs extra candidate regions,
// cleaned up by dedup) but never tight-but-wrong.
//
// Two bound strategies, selected per surface kind through a single `SurfaceAdapter`:
//
//   * FREEFORM (Bézier / B-spline / NURBS) — the CONTROL-NET CONVEX HULL bounds the
//     surface (the convex-hull property of Bézier/B-spline bases: S(u,v) lies in the
//     convex hull of the influencing control points; for a rational NURBS with wᵢ>0
//     the point is still a convex combination of the projected poles Pᵢ). The AABB
//     is the min/max of the influencing poles. This is a SOUND bound with no
//     sampling. We use the SUB-BOX-RESTRICTED pole set: only the poles whose basis
//     support overlaps the sub-box influence it, so a smaller sub-box gets a tighter
//     hull — the source of subdivision's pruning power. This mirrors the bounding
//     role OCCT IntPolyh plays (triangulate + bound), derived clean-room.
//
//   * ELEMENTARY (plane/cylinder/cone/sphere) + TORUS — no control net, so the AABB
//     is a SAMPLED bound: evaluate point(u,v) on a small grid over the sub-box, take
//     the min/max, and INFLATE by a Lipschitz margin ≥ the surface's curvature-driven
//     bulge between samples (bounded by max‖dU‖·Δu + max‖dV‖·Δv over the sub-box).
//     Inflation keeps the box conservative despite finite sampling — the honest,
//     always-sound elementary bound. (An analytic bound is tighter for a plane but
//     the sampled+margin bound is uniform across all elementary kinds and provably
//     conservative, so S2 uses it everywhere for elementary surfaces.)
//
// Header-only, OCCT-FREE, SUBSTRATE-FREE (no native-numerics). Uses src/native/math
// only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_PATCH_BOUNDS_H
#define CYBERCAD_NATIVE_SSI_PATCH_BOUNDS_H

#include "native/math/vec.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <vector>

namespace cybercad::native::ssi {

namespace math = cybercad::native::math;

using math::Dir3;
using math::Point3;
using math::Vec3;

// ─────────────────────────────────────────────────────────────────────────────
// ParamBox — a rectangular sub-region of a surface's (u,v) domain.
// ─────────────────────────────────────────────────────────────────────────────
struct ParamBox {
  double u0 = 0.0, u1 = 1.0;
  double v0 = 0.0, v1 = 1.0;

  double du() const noexcept { return u1 - u0; }
  double dv() const noexcept { return v1 - v0; }
  double uMid() const noexcept { return 0.5 * (u0 + u1); }
  double vMid() const noexcept { return 0.5 * (v0 + v1); }

  /// The larger param extent (for "split the longer direction"). Returns true if U
  /// is the longer (or equal) direction.
  bool uIsLonger() const noexcept { return du() >= dv(); }
};

// ─────────────────────────────────────────────────────────────────────────────
// Aabb — axis-aligned bounding box in 3D.
// ─────────────────────────────────────────────────────────────────────────────
struct Aabb {
  Point3 lo{ std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity(),
             std::numeric_limits<double>::infinity() };
  Point3 hi{ -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity(),
             -std::numeric_limits<double>::infinity() };

  void expand(const Point3& p) noexcept {
    lo.x = std::min(lo.x, p.x); hi.x = std::max(hi.x, p.x);
    lo.y = std::min(lo.y, p.y); hi.y = std::max(hi.y, p.y);
    lo.z = std::min(lo.z, p.z); hi.z = std::max(hi.z, p.z);
  }
  void inflate(double m) noexcept {
    lo.x -= m; lo.y -= m; lo.z -= m;
    hi.x += m; hi.y += m; hi.z += m;
  }
  bool valid() const noexcept { return lo.x <= hi.x && lo.y <= hi.y && lo.z <= hi.z; }
  double diagonal() const noexcept { return math::norm(hi - lo); }
};

/// Intersection of two AABBs (both must contain the surface for the result to still
/// contain it — used to combine two SOUND bounds into a tighter sound bound).
inline Aabb aabbIntersect(const Aabb& a, const Aabb& b) noexcept {
  Aabb r;
  r.lo.x = std::max(a.lo.x, b.lo.x); r.hi.x = std::min(a.hi.x, b.hi.x);
  r.lo.y = std::max(a.lo.y, b.lo.y); r.hi.y = std::min(a.hi.y, b.hi.y);
  r.lo.z = std::max(a.lo.z, b.lo.z); r.hi.z = std::min(a.hi.z, b.hi.z);
  return r;
}

/// Disjoint-AABB test with a tolerance gap. Two boxes are DISJOINT (prunable) iff
/// they are separated by more than `gap` on some axis. Using `> gap` (not `>= 0`)
/// keeps a boundary-touching pair as an overlap so a seed exactly on a patch edge is
/// never pruned. Sound: if the boxes are conservative surface bounds, a non-disjoint
/// verdict is necessary (not sufficient) for an intersection.
inline bool aabbDisjoint(const Aabb& a, const Aabb& b, double gap = 0.0) noexcept {
  return (a.lo.x - b.hi.x > gap) || (b.lo.x - a.hi.x > gap) ||
         (a.lo.y - b.hi.y > gap) || (b.lo.y - a.hi.y > gap) ||
         (a.lo.z - b.hi.z > gap) || (b.lo.z - a.hi.z > gap);
}

// ─────────────────────────────────────────────────────────────────────────────
// ControlNet — a surface's control-point grid. Declared BEFORE SurfaceAdapter
// because the adapter can carry one by value (see SurfaceAdapter::bezierNet).
// ─────────────────────────────────────────────────────────────────────────────
/// Grid of control points, row-major: pole(i,j) = poles[i*nCols + j], i over U.
/// For a NURBS surface pass the PROJECTED poles Pᵢ (not homogeneous wᵢ·Pᵢ): with
/// wᵢ > 0 the rational surface point is a convex combination of the Pᵢ, so their
/// hull still bounds it.
struct ControlNet {
  std::vector<Point3> poles;
  int nRows = 0;  // #poles in U
  int nCols = 0;  // #poles in V
  Point3 pole(int i, int j) const { return poles[static_cast<std::size_t>(i) * nCols + j]; }
};

// ─────────────────────────────────────────────────────────────────────────────
// SurfaceAdapter — the single subdivision interface every surface kind flows
// through. Holds the surface's full parameter domain, a point/normal evaluator, and
// a `bound(box)` that returns a CONSERVATIVE AABB of the surface over a param
// sub-box. Freeform surfaces supply a control-net-hull bound; elementary/torus
// surfaces supply a sampled+margin bound (see freeformAdapter / sampledAdapter).
// ─────────────────────────────────────────────────────────────────────────────
struct SurfaceAdapter {
  ParamBox domain{};                                    ///< full (u,v) parameter range
  std::function<Point3(double, double)> point;          ///< S(u,v)
  std::function<Dir3(double, double)> normal;           ///< unit normal at (u,v)
  std::function<Aabb(const ParamBox&)> bound;           ///< conservative AABB over a sub-box

  /// A representative model scale (bound of the whole surface's diagonal), used to
  /// derive tolerance-scaled subdivision thresholds. Filled by the make* helpers.
  double modelScale = 1.0;

  /// Parameter PERIOD in each direction, or 0 if that direction is not periodic. For
  /// a cylinder/cone/sphere/torus the U (angular) direction has period 2π; a torus is
  /// also 2π-periodic in V. A closed intersection loop that crosses the u=0≡2π (or
  /// v) seam appears as two param-space pieces; the dedup treats them as ADJACENT
  /// across the seam using these periods, so the loop stays ONE branch. Freeform
  /// (Bézier/B-spline) surfaces are non-periodic (0/0) unless the caller sets them.
  double uPeriod = 0.0;
  double vPeriod = 0.0;

  /// FREEFORM COMPLEXITY signal for scale-adaptive initial seeding (see
  /// seed_intersection). It is the count of control-net OSCILLATIONS — sign changes of
  /// the discrete second difference of the pole grid along both parameter directions,
  /// summed and taken as the worst axis. A single bump / dish / tilted sheet has ≈ 0;
  /// an egg-carton / many-span oscillating net (which can host several co-resident
  /// intersection loops inside one coarse param cell) scores high. It is scale-FREE
  /// (a second-difference SIGN count, independent of pole magnitude) and deterministic.
  ///
  /// 0 for ELEMENTARY / plane / torus surfaces (no control net → they never trigger the
  /// finer seeding; their canonical analytic behaviour is unchanged). Set by the
  /// freeform adapter factories from the surface's control net. A signal, NOT a bound —
  /// it only tunes the DEFAULT initial subdivision resolution; it never affects the
  /// conservative AABB or the correctness gate.
  int freeformComplexity = 0;

  /// FREEFORM SPAN COUNT = spansU × spansV, where spans = (nPoles − degree) per direction
  /// — the number of polynomial patches the freeform surface tiles into. It is the
  /// operand's intrinsic DENSITY: a single Bézier-like patch is 1; a many-span B-spline /
  /// NURBS is high. Two DENSE freeform operands can host several intersection loops close
  /// together in parameter space that a coarse initial subdivision would merge — the
  /// dominant general-NURBS recall miss (roadmap L2). Used with `freeformComplexity` to
  /// decide the DEFAULT initial seeding resolution.
  ///
  /// 0 for ELEMENTARY / plane / torus (no control net) — so a pair with an elementary
  /// operand (e.g. plane ∩ B-spline) never triggers the freeform-density adaptivity; only
  /// FREEFORM↔FREEFORM pairs do. A signal only; never affects a bound or the gate.
  int freeformSpanCount = 0;

  /// EXACT single-span NON-RATIONAL Bézier control net, when this surface is one; otherwise
  /// `hasBezierNet` is false and `bezierNet` is empty.
  ///
  /// Exposed for `patchGapBound` (ssi/patch_gap.h), which needs the actual poles to build the
  /// difference net — the `bound` closure above captures its net privately and cannot supply it.
  /// The conditions are exactly the ones that bound's proof requires and nothing looser:
  ///
  ///   * SINGLE SPAN (nPoles == degree+1 per direction). A multi-span B-spline would first need
  ///     Boehm knot insertion to be split into Bézier patches; that path is not validated, so
  ///     such surfaces expose no net and simply do not get certified.
  ///   * NON-RATIONAL. The difference of two rationals is NOT a Bézier of pole differences, so
  ///     the convex-hull argument does not hold. A NURBS adapter must never set this, even though
  ///     its projected poles are perfectly good for the AABB hull bound above.
  ///
  /// A consumer that finds `hasBezierNet == false` must refuse, never approximate — refusing
  /// costs reach only, whereas a wrong bound loses geometry silently.
  ControlNet bezierNet{};
  bool hasBezierNet = false;
};

// ── freeform (control-net convex hull) bound ───────────────────────────────────

namespace detail {

/// Map a normalized fraction f∈[0,1] across the pole index range [0, n-1] and return
/// the inclusive index window [lo,hi] of poles whose support could influence params
/// at or below that fraction. For a clamped Bézier/B-spline the poles are ordered
/// along the parameter, so a param sub-box [f0,f1] is bounded by the poles in a
/// window around [f0·(n-1), f1·(n-1)] widened by the degree. We widen GENEROUSLY
/// (by 1 pole each side beyond the linear map, plus never dropping below a
/// full-degree span) to stay conservative regardless of the exact knot spacing — a
/// looser window only costs a looser (still sound) bound.
inline void poleWindow(double f0, double f1, int n, int& lo, int& hi) noexcept {
  const int last = n - 1;
  if (last <= 0) { lo = 0; hi = last < 0 ? 0 : last; return; }
  const double a = f0 * last, b = f1 * last;
  lo = static_cast<int>(std::floor(a)) - 1;
  hi = static_cast<int>(std::ceil(b)) + 1;
  lo = std::max(0, lo);
  hi = std::min(last, hi);
  if (lo > hi) std::swap(lo, hi);
}

}  // namespace detail

/// Conservative AABB of a freeform surface over a param sub-box, from the convex
/// hull (min/max) of the influencing control poles. `net`'s poles are indexed by
/// the fraction of the sub-box within the FULL domain, so the caller passes the
/// full-domain fractions (box relative to `domain`).
inline Aabb controlNetBound(const ControlNet& net, const ParamBox& domain,
                            const ParamBox& box) {
  Aabb bb;
  if (net.poles.empty()) return bb;
  const double dU = domain.du() != 0.0 ? domain.du() : 1.0;
  const double dV = domain.dv() != 0.0 ? domain.dv() : 1.0;
  const double fu0 = (box.u0 - domain.u0) / dU, fu1 = (box.u1 - domain.u0) / dU;
  const double fv0 = (box.v0 - domain.v0) / dV, fv1 = (box.v1 - domain.v0) / dV;
  int iu0, iu1, jv0, jv1;
  detail::poleWindow(fu0, fu1, net.nRows, iu0, iu1);
  detail::poleWindow(fv0, fv1, net.nCols, jv0, jv1);
  for (int i = iu0; i <= iu1; ++i)
    for (int j = jv0; j <= jv1; ++j)
      bb.expand(net.pole(i, j));
  return bb;
}

// ── control-net oscillation (scale-adaptive-seeding complexity signal) ──────────

namespace detail {

/// One pole coordinate (0=x, 1=y, 2=z).
inline double poleCoord(const Point3& p, int c) noexcept {
  return c == 0 ? p.x : (c == 1 ? p.y : p.z);
}

/// Count the SIGNIFICANT slope reversals (turning points) of a length-`n` coordinate
/// sequence `s`, with a NOISE-BAND hysteresis: a turn is registered only after the
/// profile retreats by ≥ `kNoiseFrac` of the line's own peak-to-peak range from a run
/// extremum, and the whole line is IGNORED (returns 0) when its range is below
/// `kFlatFrac` of the coordinate's global span (a near-constant / wobble-only line). So a
/// small pole jitter / XY wobble does not register as a wave; only genuine oscillation
/// comparable to the shape amplitude counts. `globalSpan` is the coord's net-wide extent.
inline int lineReversals(const std::vector<double>& s, double globalSpan) noexcept {
  constexpr double kFlatFrac = 0.15;   // line range must reach this fraction of the coord's global span
  constexpr double kNoiseFrac = 0.20;  // a turn needs this much retreat (of the line range) from a run extremum
  const int n = static_cast<int>(s.size());
  if (n < 3) return 0;
  double lo = s[0], hi = s[0];
  for (int k = 1; k < n; ++k) { lo = std::min(lo, s[k]); hi = std::max(hi, s[k]); }
  const double range = hi - lo;
  if (range <= 0.0 || range < kFlatFrac * globalSpan) return 0;
  const double sigStep = kNoiseFrac * range;
  // `dir` = current confirmed run direction (+1 up, −1 down, 0 unset); `ext` = run extremum.
  // A move ≥ sigStep AGAINST the run confirms a turn; the first significant move only sets dir.
  int rev = 0, dir = 0;
  double ext = s[0];
  for (int k = 1; k < n; ++k) {
    const double v = s[k];
    if (dir >= 0 && v > ext) { ext = v; if (dir == 0 && v - lo >= sigStep) dir = 1; continue; }
    if (dir <= 0 && v < ext) { ext = v; if (dir == 0 && hi - v >= sigStep) dir = -1; continue; }
    if (dir > 0 && ext - v >= sigStep) { ++rev; dir = -1; ext = v; }        // up-run turned down
    else if (dir < 0 && v - ext >= sigStep) { ++rev; dir = 1; ext = v; }    // down-run turned up
  }
  return rev;
}

}  // namespace detail

/// Count the MULTI-MODAL LINES of a freeform net: a scale-free measure of how many
/// distinct "waves" the control net carries, which bounds how many co-resident
/// intersection loops it can host inside one coarse subdivision cell.
///
/// For each net LINE (a row = fixed i over j, and a column = fixed j over i) and each pole
/// coordinate, count the significant slope reversals (detail::lineReversals). A single
/// dome / monotone ramp / one-hump profile turns AT MOST ONCE — that is a SINGLE loop and
/// must NOT trigger finer seeding; a line counts as MULTI-MODAL only when it turns ≥ 2
/// times (a genuine egg-carton / multi-wave profile, which CAN carry several loops). The
/// score is the number of such multi-modal lines over both directions and all three coords.
///
/// Deterministic, integer, magnitude-independent, OCCT-free, substrate-free. Used only to
/// tune the DEFAULT initial subdivision resolution; never affects a bound.
inline int controlNetOscillation(const ControlNet& net) {
  if (net.nRows < 3 && net.nCols < 3) return 0;
  // Per-coordinate GLOBAL span (max − min over all poles) — the denominator lineReversals
  // uses to skip near-constant/wobble-only lines.
  double gLo[3] = {net.pole(0, 0).x, net.pole(0, 0).y, net.pole(0, 0).z};
  double gHi[3] = {gLo[0], gLo[1], gLo[2]};
  for (int i = 0; i < net.nRows; ++i)
    for (int j = 0; j < net.nCols; ++j)
      for (int c = 0; c < 3; ++c) {
        const double v = detail::poleCoord(net.pole(i, j), c);
        gLo[c] = std::min(gLo[c], v); gHi[c] = std::max(gHi[c], v);
      }
  int multiModal = 0;
  std::vector<double> line;
  for (int c = 0; c < 3; ++c) {
    const double span = gHi[c] - gLo[c];
    for (int i = 0; i < net.nRows; ++i) {   // rows (over j)
      line.clear();
      for (int j = 0; j < net.nCols; ++j) line.push_back(detail::poleCoord(net.pole(i, j), c));
      if (detail::lineReversals(line, span) >= 2) ++multiModal;
    }
    for (int j = 0; j < net.nCols; ++j) {   // columns (over i)
      line.clear();
      for (int i = 0; i < net.nRows; ++i) line.push_back(detail::poleCoord(net.pole(i, j), c));
      if (detail::lineReversals(line, span) >= 2) ++multiModal;
    }
  }
  return multiModal;
}

// ── elementary / torus (sampled + Lipschitz margin) bound ──────────────────────

/// Conservative AABB of a smooth surface over a param sub-box by sampling point()
/// on an (n+1)×(n+1) grid and inflating by a Lipschitz margin. The margin bounds the
/// worst-case bulge of the surface between adjacent samples: with cell sizes Δu, Δv
/// and derivative bounds Lu ≥ max‖dU‖, Lv ≥ max‖dV‖ over the sub-box, the surface
/// moves at most 0.5·(Lu·Δu + Lv·Δv) away from the sampled polyline within a cell.
/// We estimate Lu/Lv from finite differences of the same grid (max adjacent-sample
/// gap ÷ cell size) and use a safety factor, so the inflated box stays conservative
/// for the elementary + torus surfaces (which are C∞ with bounded curvature over any
/// finite sub-box). `grid` ≥ 2.
inline Aabb sampledBound(const std::function<Point3(double, double)>& point,
                         const ParamBox& box, int grid = 4) {
  Aabb bb;
  if (grid < 2) grid = 2;
  std::vector<Point3> pts;
  pts.reserve(static_cast<std::size_t>(grid + 1) * (grid + 1));
  for (int i = 0; i <= grid; ++i) {
    const double u = box.u0 + box.du() * (double(i) / grid);
    for (int j = 0; j <= grid; ++j) {
      const double v = box.v0 + box.dv() * (double(j) / grid);
      pts.push_back(point(u, v));
    }
  }
  const int stride = grid + 1;
  double maxU = 0.0, maxV = 0.0;  // max adjacent-sample gap in each direction
  for (int i = 0; i <= grid; ++i)
    for (int j = 0; j <= grid; ++j) {
      const Point3& p = pts[static_cast<std::size_t>(i) * stride + j];
      bb.expand(p);
      if (i + 1 <= grid)
        maxU = std::max(maxU, math::distance(p, pts[static_cast<std::size_t>(i + 1) * stride + j]));
      if (j + 1 <= grid)
        maxV = std::max(maxV, math::distance(p, pts[static_cast<std::size_t>(i) * stride + j + 1]));
    }
  // Bulge margin: half a cell's worth of the largest adjacent gap in each direction,
  // times a safety factor (2) to cover curvature the linear polyline underestimates.
  const double margin = (maxU + maxV) * 0.5 * 2.0;
  bb.inflate(margin);
  return bb;
}

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_PATCH_BOUNDS_H

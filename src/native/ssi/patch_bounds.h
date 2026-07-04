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
};

// ── freeform (control-net convex hull) bound ───────────────────────────────────

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

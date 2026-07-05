// SPDX-License-Identifier: Apache-2.0
//
// wrap_emboss.h — native FIRST SLICE of WRAP-EMBOSS (Phase 4 #7 `native-wrap-emboss`):
// emboss a RECTANGULAR pad onto a CYLINDER lateral face (boss=1), behind the existing
// OCCT-only `cc_wrap_emboss` ABI. Clean-room, OCCT-FREE, header-only.
//
// ── WHAT THIS BUILDS ──────────────────────────────────────────────────────────────
// A solid cylinder (radius R, axis A, axial extent [zLo,zHi]) with a raised rectangular
// pad wrapped onto its lateral face. The pad footprint is a rectangle drawn in the
// profile's own (px,py) space; it is WRAPPED onto the cylinder by the SAME map the OCCT
// oracle (occt_wrap_emboss.cpp) uses:
//     u = px / R              (arc-length px → angle about the axis)
//     v = py + vMid           (axial coordinate; vMid = axial middle of the wall)
// so a profile centred at py = 0 lands mid-face. The rectangle's two AXIAL edges
// (px = const) are straight axial segments; its two CIRCUMFERENTIAL edges (py = const)
// are circular arcs on the cylinder. The raised pad is the cap-and-side set:
//   * OUTER CAP — the cylinder at radius R+height over the footprint window
//                 [uMin,uMax] × [vMin,vMax] (a true co-axial cylindrical patch).
//   * 2 CIRCUMFERENTIAL walls — constant-v radial strips (planes ⟂ A at v=vMin,vMax)
//                 from radius R to R+height, following the arc in u.
//   * 2 AXIAL walls — constant-u radial strips (planes through A at u=uMin,uMax)
//                 from radius R to R+height, spanning v.
// The base cylinder wall is retiled over the FULL turn with the footprint window
// REMOVED (the pad's inner opening), so the pad's four walls close the material against
// the cylinder along the shared footprint boundary — exactly the fused body the OCCT
// oracle produces, with NO doubled/coincident faces.
//
// ── WHY A FACET SOUP, NOT AN SSI FUSE ──────────────────────────────────────────────
// The pad-wall∩cylinder seam IS an SSI/curved-boolean case, but the pad is NOT a single
// elementary curved solid (it carries planar walls + a cylindrical cap), so the S5-a
// `ssi_boolean_solid` (transversal single-elementary-pair) cannot drive this fuse
// robustly. Instead we mirror the proven curved-fillet slice: rebuild the WHOLE embossed
// solid as one deflection-bounded planar-facet soup that shares vertices exactly along
// every seam, and weld it watertight through the SAME boolean `assembleSolid` a native
// prism/boolean uses. Each curved quad (cylinder patch) is split into two TRIANGLES so
// every facet is exactly planar (a cylinder quad's 4 corners are not coplanar; a quad
// Polygon would carry a derived plane its 4th vertex misses and leak — the same lesson
// as curved_fillet.h). The engine self-verify (watertight + volume GROWS by
// ≈ wrappedFootprintArea × height) then accepts it, else → OCCT `cc_wrap_emboss`.
//
// ── SCOPE (honest) ─────────────────────────────────────────────────────────────────
// Native ONLY for: a native solid whose picked face is a Cylinder lateral face; boss=1
// (emboss / raise material); a CLOSED 4-corner RECTANGLE profile axis-aligned in (px,py)
// (two px=const axial edges + two py=const circumferential edges); positive height; a
// footprint that fits on the wall (arc span < 2π, axial span inside [zLo,zHi]). Anything
// else returns a NULL Shape → OCCT: deboss (boss=0), non-rectangular / >4-corner / dense
// profiles, a non-cylindrical or freeform base, a self-overlapping / >2π footprint, a
// footprint that runs off the axial ends. Nothing faked; the engine re-verifies.
//
// CLEAN-ROOM. Reuses src/native/math (Cylinder frame) + topology + boolean
// (Polygon/Plane/assembleSolid). No OCCT. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_FEATURE_WRAP_EMBOSS_H
#define CYBERCAD_NATIVE_FEATURE_WRAP_EMBOSS_H

#include "native/boolean/assemble.h"
#include "native/boolean/polygon.h"
#include "native/math/elementary.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::feature {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace nb = cybercad::native::boolean;

namespace detail {

inline constexpr double kWeTwoPi = 6.283185307179586476925286766559;
inline constexpr double kWeEps = 1e-9;

// World-space frame + radius + axial extent of a Cylinder lateral face (folds the face
// location). vLo/vHi are the axial coordinates of the wall's two ends along the axis.
struct CylWall {
  math::Ax3 frame;      // origin on axis, z = axis direction, x/y span the section
  double radius = 0.0;
  double vLo = 0.0;     // axial coord of one end
  double vHi = 0.0;     // axial coord of the other end
};

inline math::Ax3 worldFrame(const topo::FaceSurfaceResult& surf) {
  math::Ax3 f = surf.surface->frame;
  if (!surf.location.isIdentity()) {
    const math::Transform& t = surf.location.transform();
    f.origin = t.applyToPoint(f.origin);
    f.x = math::Dir3{t.applyToVector(f.x.vec())};
    f.y = math::Dir3{t.applyToVector(f.y.vec())};
    f.z = math::Dir3{t.applyToVector(f.z.vec())};
  }
  return f;
}

// Recover the picked face as a Cylinder wall + the solid's axial extent along its axis.
inline std::optional<CylWall> cylinderWall(const topo::Shape& solid, int faceId) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > fmap.size()) return std::nullopt;
  const auto surf = topo::surfaceOf(fmap.shape(faceId));
  if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Cylinder) return std::nullopt;
  CylWall w;
  w.frame = worldFrame(*surf);
  w.radius = surf->surface->radius;
  if (!(w.radius > kWeEps)) return std::nullopt;

  // Axial extent: project every solid vertex onto the axis; the wall spans [min,max].
  const math::Vec3 az = w.frame.z.vec();
  double lo = 0.0, hi = 0.0;
  bool any = false;
  for (topo::Explorer ex(solid, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const auto p = topo::pointOf(ex.current());
    if (!p) continue;
    const double a = math::dot(*p - w.frame.origin, az);
    if (!any) { lo = hi = a; any = true; }
    else { lo = std::min(lo, a); hi = std::max(hi, a); }
  }
  if (!any || !(hi - lo > kWeEps)) return std::nullopt;
  w.vLo = lo;
  w.vHi = hi;
  return w;
}

// A rectangular footprint recovered from the (px,py) profile: it must be a CLOSED
// 4-corner rectangle whose edges are axis-aligned in (px,py) — two px=const axial edges
// and two py=const circumferential edges. Returns the axis-aligned box in profile space.
struct FootRect {
  double pxMin = 0.0, pxMax = 0.0;  // arc-length bounds
  double pyMin = 0.0, pyMax = 0.0;  // axial bounds (before vMid offset)
};
inline std::optional<FootRect> rectFootprint(const double* profileXY, int count) {
  if (profileXY == nullptr || count != 4) return std::nullopt;
  double xs[4], ys[4];
  for (int i = 0; i < 4; ++i) { xs[i] = profileXY[i * 2]; ys[i] = profileXY[i * 2 + 1]; }
  FootRect r;
  r.pxMin = *std::min_element(xs, xs + 4);
  r.pxMax = *std::max_element(xs, xs + 4);
  r.pyMin = *std::min_element(ys, ys + 4);
  r.pyMax = *std::max_element(ys, ys + 4);
  if (!(r.pxMax - r.pxMin > kWeEps) || !(r.pyMax - r.pyMin > kWeEps)) return std::nullopt;
  // Every corner must sit at one of the two px extremes AND one of the two py extremes
  // (an axis-aligned rectangle); a slanted / non-rectangular loop → decline → OCCT.
  for (int i = 0; i < 4; ++i) {
    const bool onPx = std::fabs(xs[i] - r.pxMin) <= kWeEps || std::fabs(xs[i] - r.pxMax) <= kWeEps;
    const bool onPy = std::fabs(ys[i] - r.pyMin) <= kWeEps || std::fabs(ys[i] - r.pyMax) <= kWeEps;
    if (!onPx || !onPy) return std::nullopt;
  }
  return r;
}

// Facet count for a curved span from a sagitta bound r(1−cos(Δ/2)) ≤ defl.
inline int sagittaSteps(double radius, double span, double defl, int lo, int hi) {
  if (radius <= kWeEps || span <= kWeEps) return lo;
  const double ratio = 1.0 - std::clamp(defl / radius, 0.0, 1.0);
  const double dmax = 2.0 * std::acos(std::clamp(ratio, -1.0, 1.0));
  const int n = dmax > 1e-12 ? static_cast<int>(std::ceil(span / dmax)) : hi;
  return std::clamp(n, lo, hi);
}

// A point on the cylinder axis frame at radius `rad`, angle u, axial coord v.
inline math::Point3 ringPoint(const math::Ax3& ax, double rad, double u, double v) {
  return math::frameCombine(ax, rad * std::cos(u), rad * std::sin(u), v);
}

// Emit a TRIANGLE (always exactly planar) with a target outward normal, oriented so the
// stored Plane passes through all three vertices (curved quads split into two of these).
inline void emitTri(std::vector<nb::Polygon>& polys, const math::Point3& a,
                    const math::Point3& b, const math::Point3& c, const math::Vec3& outward) {
  math::Vec3 nrm = math::cross(b - a, c - a);
  if (math::dot(nrm, outward) < 0.0) nrm = nrm * -1.0;
  const math::Dir3 nd{nrm};
  if (!nd.valid()) return;
  polys.emplace_back(std::vector<math::Point3>{a, b, c},
                     nb::Plane::fromPointNormal(a, nd.vec()));
}

// Emit a planar CCW polygon with a target outward normal (constant-height rings / radial
// strips are exactly planar). Rewinds so the loop is CCW as seen from the +normal side.
inline void emitFlat(std::vector<nb::Polygon>& polys, std::vector<math::Point3> loop,
                     const math::Vec3& outward) {
  const math::Dir3 nd{outward};
  if (!nd.valid() || loop.size() < 3) return;
  math::Vec3 area{0, 0, 0};
  for (std::size_t i = 0; i < loop.size(); ++i)
    area += math::cross(loop[i].asVec(), loop[(i + 1) % loop.size()].asVec());
  if (math::dot(area, nd.vec()) < 0.0) std::reverse(loop.begin(), loop.end());
  const math::Point3 onPlane = loop.front();
  polys.emplace_back(std::move(loop), nb::Plane::fromPointNormal(onPlane, nd.vec()));
}

// Build the shared full-turn angular sample sequence, anchored at uMin, with uMax an
// EXACT interior breakpoint. Every part that spans the turn (wall rows, end-cap fans)
// reuses this exact sequence, and the pad's window parts reuse its [uMin,uMax] prefix,
// so every seam shares vertices and welds watertight. The window arc [uMin,uMax] and the
// remainder [uMax, uMin+2π] are each sagitta-tiled independently. Returns {samples,
// nUwin} where samples[0..nUwin] cover the window arc.
struct USamples {
  std::vector<double> u;   ///< u[0]=uMin … u[nUwin]=uMax … u[last]=uMin+2π
  int nUwin = 0;           ///< number of cells in the window arc
};
inline USamples uSamples(double rad, double uMin, double uMax, double defl) {
  USamples s;
  const int nWin = sagittaSteps(rad, uMax - uMin, defl, 1, 256);
  const int nRest = sagittaSteps(rad, (uMin + kWeTwoPi) - uMax, defl, 1, 256);
  s.nUwin = nWin;
  for (int i = 0; i <= nWin; ++i) s.u.push_back(uMin + (uMax - uMin) * i / nWin);
  for (int i = 1; i <= nRest; ++i)
    s.u.push_back(uMax + ((uMin + kWeTwoPi) - uMax) * i / nRest);
  // s.u.back() == uMin + 2π ≡ uMin (same point) — the fan/wall close onto s.u.front().
  return s;
}

// The base cylinder wall over the FULL turn with the footprint window removed, tiled on
// the shared u-sample sequence. Axial rows are [vLo,vMin],[vMin,vMax],[vHi]; the window
// arc cells (u index < nUwin) in the middle axial band are SKIPPED (the pad covers them).
inline void tileWallWithWindow(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double rad,
                               const USamples& us, double vLo, double vHi, double vMin,
                               double vMax) {
  const int nU = static_cast<int>(us.u.size()) - 1;
  const double vseg[4] = {vLo, vMin, vMax, vHi};
  for (int i = 0; i < nU; ++i) {
    const double ua = us.u[i], ub = us.u[i + 1];
    const double um = 0.5 * (ua + ub);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    for (int seg = 0; seg < 3; ++seg) {
      const double va = vseg[seg], vb = vseg[seg + 1];
      if (!(vb - va > kWeEps)) continue;
      if (i < us.nUwin && seg == 1) continue;  // window arc AND axial mid → pad covers it
      const math::Point3 p00 = ringPoint(ax, rad, ua, va);
      const math::Point3 p10 = ringPoint(ax, rad, ub, va);
      const math::Point3 p11 = ringPoint(ax, rad, ub, vb);
      const math::Point3 p01 = ringPoint(ax, rad, ua, vb);
      emitTri(polys, p00, p10, p11, outN);
      emitTri(polys, p00, p11, p01, outN);
    }
  }
}

// A flat end cap disk at axial coord `v`, triangulated as a fan from the axis centre to
// each wall-row edge on the SHARED u-sample sequence (so the cap-to-wall seam welds).
inline void emitEndCap(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double rad, double v,
                       const math::Vec3& outw, const USamples& us) {
  const math::Point3 c = math::frameCombine(ax, 0.0, 0.0, v);
  const int nU = static_cast<int>(us.u.size()) - 1;
  for (int i = 0; i < nU; ++i) {
    const math::Point3 a = ringPoint(ax, rad, us.u[i], v);
    const math::Point3 b = ringPoint(ax, rad, us.u[i + 1], v);
    emitTri(polys, c, a, b, outw);
  }
}

// Pad OUTER CAP: cylinder patch at radius Rout over the window u-cells × [vMin,vMax],
// outward radial (two triangles per cell). Shares the window u-samples with the base
// wall window and both axial walls, so it welds along all three seams.
inline void emitOuterCap(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double Rout,
                         const USamples& us, double vMin, double vMax) {
  for (int i = 0; i < us.nUwin; ++i) {
    const double ua = us.u[i], ub = us.u[i + 1], um = 0.5 * (ua + ub);
    const math::Vec3 outN = ax.x.vec() * std::cos(um) + ax.y.vec() * std::sin(um);
    emitTri(polys, ringPoint(ax, Rout, ua, vMin), ringPoint(ax, Rout, ub, vMin),
            ringPoint(ax, Rout, ub, vMax), outN);
    emitTri(polys, ringPoint(ax, Rout, ua, vMin), ringPoint(ax, Rout, ub, vMax),
            ringPoint(ax, Rout, ua, vMax), outN);
  }
}

// Pad AXIAL walls (u = uMin and u = uMax): radial strips from R to Rout spanning
// [vMin,vMax]. Each lies in the plane through the axis + the radial direction (planar,
// one quad = two triangles). Outward = ∓ tangential (away from the window interior).
inline void emitAxialWalls(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double R,
                           double Rout, double uMin, double uMax, double vMin, double vMax) {
  const double us2[2] = {uMin, uMax};
  for (int k = 0; k < 2; ++k) {
    const double u = us2[k];
    const math::Vec3 tang = ax.x.vec() * (-std::sin(u)) + ax.y.vec() * std::cos(u);
    const math::Vec3 outw = (k == 0) ? (tang * -1.0) : tang;
    emitTri(polys, ringPoint(ax, R, u, vMin), ringPoint(ax, R, u, vMax),
            ringPoint(ax, Rout, u, vMax), outw);
    emitTri(polys, ringPoint(ax, R, u, vMin), ringPoint(ax, Rout, u, vMax),
            ringPoint(ax, Rout, u, vMin), outw);
  }
}

// Pad CIRCUMFERENTIAL walls (v = vMin and v = vMax): constant-v radial strips from R to
// Rout, following the arc on the window u-cells (each lies in the plane ⟂ axis at v →
// planar). The R-side edge shares the window u-samples with the base wall. Outward = ∓ axis.
inline void emitCircWalls(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double R,
                          double Rout, const USamples& us, double vMin, double vMax) {
  const double vs2[2] = {vMin, vMax};
  for (int k = 0; k < 2; ++k) {
    const double v = vs2[k];
    const math::Vec3 outw = (k == 0) ? (ax.z.vec() * -1.0) : ax.z.vec();
    for (int i = 0; i < us.nUwin; ++i) {
      const double ua = us.u[i], ub = us.u[i + 1];
      emitTri(polys, ringPoint(ax, R, ua, v), ringPoint(ax, R, ub, v),
              ringPoint(ax, Rout, ub, v), outw);
      emitTri(polys, ringPoint(ax, R, ua, v), ringPoint(ax, Rout, ub, v),
              ringPoint(ax, Rout, ua, v), outw);
    }
  }
}

// Assemble the embossed cylinder as a planar-facet soup (empty on any degeneracy). A
// short linear composition; each part is one deflection-bounded tiling helper, all
// sharing the window u-samples so the whole soup welds watertight via assembleSolid.
inline std::vector<nb::Polygon> buildEmbossedCylinder(const CylWall& wall, const FootRect& foot,
                                                      double height, double vMid, double defl) {
  const math::Ax3& ax = wall.frame;
  const double R = wall.radius;
  const double Rout = R + height;
  if (!(height > kWeEps)) return {};

  // Wrap the footprint: u = px/R, v = py + vMid. Require the arc span < full turn and the
  // axial span strictly inside the wall (else a self-overlapping / off-end footprint).
  const double uMin = foot.pxMin / R, uMax = foot.pxMax / R;
  const double vMin = foot.pyMin + vMid, vMax = foot.pyMax + vMid;
  if (!(uMax - uMin > kWeEps) || uMax - uMin >= kWeTwoPi - kWeEps) return {};
  if (!(vMin > wall.vLo + kWeEps) || !(vMax < wall.vHi - kWeEps)) return {};

  // Shared u-sample sequence (window arc = the first nUwin cells). The axial (v)
  // direction is STRAIGHT on a cylinder, so a single v-segment across the window is
  // geometrically EXACT — no curvature to bound, keeping the facet count low.
  const USamples us = uSamples(R, uMin, uMax, defl);

  std::vector<nb::Polygon> polys;
  tileWallWithWindow(polys, ax, R, us, wall.vLo, wall.vHi, vMin, vMax);  // base wall − window
  emitOuterCap(polys, ax, Rout, us, vMin, vMax);                         // pad top
  emitAxialWalls(polys, ax, R, Rout, uMin, uMax, vMin, vMax);            // pad ends (u=const)
  emitCircWalls(polys, ax, R, Rout, us, vMin, vMax);                     // pad sides (v=const)
  emitEndCap(polys, ax, R, wall.vLo, ax.z.vec() * -1.0, us);             // cylinder end caps
  emitEndCap(polys, ax, R, wall.vHi, ax.z.vec(), us);
  return polys;
}

}  // namespace detail

// Emboss a RECTANGULAR pad onto the CYLINDER lateral face `faceId` of `solid`, raising it
// by `height` (boss). Returns the embossed solid (deflection-bounded facet soup, welded
// watertight) or a NULL Shape (→ OCCT) when the input is out of the native slice: not a
// cylinder face, boss=0, a non-rectangular / non-4-corner profile, non-positive height,
// or a footprint that does not fit on the wall. `vMid` defaults to the wall's axial
// middle (matching the OCCT oracle's V-mid centring).
inline topo::Shape wrap_emboss(const topo::Shape& solid, int faceId, const double* profileXY,
                               int count, double height, int boss, double deflection = 0.01) {
  if (boss != 1) return {};  // deboss / cut is OCCT-only in this slice
  if (!(height > detail::kWeEps)) return {};
  const auto wall = detail::cylinderWall(solid, faceId);
  if (!wall) return {};
  const auto foot = detail::rectFootprint(profileXY, count);
  if (!foot) return {};
  const double vMid = 0.5 * (wall->vLo + wall->vHi);
  std::vector<nb::Polygon> polys =
      detail::buildEmbossedCylinder(*wall, *foot, height, vMid, deflection);
  if (polys.size() < 6) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::feature

#endif  // CYBERCAD_NATIVE_FEATURE_WRAP_EMBOSS_H

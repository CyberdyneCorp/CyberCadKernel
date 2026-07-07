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
namespace tess = cybercad::native::tessellate;

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
                           double Rout, double uMin, double uMax, double vMin, double vMax,
                           double sign = 1.0) {
  const double us2[2] = {uMin, uMax};
  for (int k = 0; k < 2; ++k) {
    const double u = us2[k];
    const math::Vec3 tang = ax.x.vec() * (-std::sin(u)) + ax.y.vec() * std::cos(u);
    const math::Vec3 outw = ((k == 0) ? (tang * -1.0) : tang) * sign;
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
                          double Rout, const USamples& us, double vMin, double vMax,
                          double sign = 1.0) {
  const double vs2[2] = {vMin, vMax};
  for (int k = 0; k < 2; ++k) {
    const double v = vs2[k];
    const math::Vec3 outw = ((k == 0) ? (ax.z.vec() * -1.0) : ax.z.vec()) * sign;
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

// ── T1 DEBOSS (recessed rectangular pocket) ─────────────────────────────────────────
// The exact MIRROR of buildEmbossedCylinder with an INWARD target: the pad's outer cap
// becomes a POCKET FLOOR at radius R−depth, the axial/circumferential walls span
// [R−depth, R] with their outward normals FLIPPED to point INTO the pocket void (sign
// −1), and the base wall window + cylinder end caps are byte-identical to the emboss
// build. The material between R−depth and R over the footprint is REMOVED, so the
// volume SHRINKS by ≈ footprint area × depth. Empty on any degeneracy (depth ≥ R,
// off-wall / ≥2π footprint). Guard 0 < depth < R.
inline std::vector<nb::Polygon> buildDebossedCylinder(const CylWall& wall, const FootRect& foot,
                                                      double depth, double vMid, double defl) {
  const math::Ax3& ax = wall.frame;
  const double R = wall.radius;
  const double rFloor = R - depth;
  if (!(depth > kWeEps) || !(rFloor > kWeEps)) return {};

  const double uMin = foot.pxMin / R, uMax = foot.pxMax / R;
  const double vMin = foot.pyMin + vMid, vMax = foot.pyMax + vMid;
  if (!(uMax - uMin > kWeEps) || uMax - uMin >= kWeTwoPi - kWeEps) return {};
  if (!(vMin > wall.vLo + kWeEps) || !(vMax < wall.vHi - kWeEps)) return {};

  const USamples us = uSamples(R, uMin, uMax, defl);
  std::vector<nb::Polygon> polys;
  tileWallWithWindow(polys, ax, R, us, wall.vLo, wall.vHi, vMin, vMax);   // base wall − window
  emitOuterCap(polys, ax, rFloor, us, vMin, vMax);                       // pocket floor (R−depth)
  emitAxialWalls(polys, ax, R, rFloor, uMin, uMax, vMin, vMax, -1.0);    // pocket ends (u=const)
  emitCircWalls(polys, ax, R, rFloor, us, vMin, vMax, -1.0);             // pocket sides (v=const)
  emitEndCap(polys, ax, R, wall.vLo, ax.z.vec() * -1.0, us);             // cylinder end caps
  emitEndCap(polys, ax, R, wall.vHi, ax.z.vec(), us);
  return polys;
}

// ── T2 NON-RECTANGULAR POLYGON footprint ────────────────────────────────────────────
// An N-vertex simple closed polygon (px,py), CCW, non-self-intersecting, with a signed
// (shoelace) area. Wraps vertex-wise onto the cylinder like the rectangle. Emboss (boss)
// raises it to R+height; deboss (else) recesses it to R−height.
struct FootPoly {
  std::vector<double> px, py;                    // ordered CCW loop (profile space)
  double pxMin = 0.0, pxMax = 0.0, pyMin = 0.0, pyMax = 0.0;
};

inline double orient2dXY(double ax, double ay, double bx, double by, double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

// Do the OPEN segments p0p1 and q0q1 properly cross (strict interiors)? Shared endpoints
// do NOT count. Used to reject a self-intersecting profile.
inline bool properCrossXY(double p0x, double p0y, double p1x, double p1y, double q0x, double q0y,
                          double q1x, double q1y) {
  const double d1 = orient2dXY(q0x, q0y, q1x, q1y, p0x, p0y);
  const double d2 = orient2dXY(q0x, q0y, q1x, q1y, p1x, p1y);
  const double d3 = orient2dXY(p0x, p0y, p1x, p1y, q0x, q0y);
  const double d4 = orient2dXY(p0x, p0y, p1x, p1y, q1x, q1y);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && std::fabs(d1) > 0.0 &&
         std::fabs(d2) > 0.0 && std::fabs(d3) > 0.0 && std::fabs(d4) > 0.0;
}

// A CLOSED simple N-vertex profile (N ≥ 3, non-degenerate area, no self-intersection),
// re-wound CCW. Any degeneracy → nullopt (→ decline → OCCT).
inline std::optional<FootPoly> polyFootprint(const double* profileXY, int count) {
  if (profileXY == nullptr || count < 3) return std::nullopt;
  FootPoly f;
  f.px.reserve(count);
  f.py.reserve(count);
  for (int i = 0; i < count; ++i) {
    f.px.push_back(profileXY[i * 2]);
    f.py.push_back(profileXY[i * 2 + 1]);
  }
  f.pxMin = *std::min_element(f.px.begin(), f.px.end());
  f.pxMax = *std::max_element(f.px.begin(), f.px.end());
  f.pyMin = *std::min_element(f.py.begin(), f.py.end());
  f.pyMax = *std::max_element(f.py.begin(), f.py.end());
  if (!(f.pxMax - f.pxMin > kWeEps) || !(f.pyMax - f.pyMin > kWeEps)) return std::nullopt;

  double a2 = 0.0;  // twice the signed (shoelace) area
  for (int i = 0; i < count; ++i) {
    const int j = (i + 1) % count;
    a2 += f.px[i] * f.py[j] - f.px[j] * f.py[i];
  }
  if (!(std::fabs(a2) > kWeEps)) return std::nullopt;   // degenerate / collinear loop
  if (a2 < 0.0) {                                        // re-wind CCW
    std::reverse(f.px.begin(), f.px.end());
    std::reverse(f.py.begin(), f.py.end());
  }
  // Reject a self-intersecting loop (any non-adjacent edge pair properly crosses).
  for (int i = 0; i < count; ++i) {
    const int i1 = (i + 1) % count;
    for (int k = i + 1; k < count; ++k) {
      const int k1 = (k + 1) % count;
      if (i == k || i == k1 || i1 == k || i1 == k1) continue;  // adjacent / shared vertex
      if (properCrossXY(f.px[i], f.py[i], f.px[i1], f.py[i1], f.px[k], f.py[k], f.px[k1],
                        f.py[k1]))
        return std::nullopt;
    }
  }
  return f;
}

// Densify a closed (u,v) polygon: each edge is split by angular sagitta so a straight
// 3D chord across the arc stays within `defl`. Axial edges (Δu≈0) stay single-segment.
inline std::vector<tess::UV> densifyPolyUV(const FootPoly& poly, double R, double vMid,
                                           double defl) {
  std::vector<tess::UV> dp;
  const int n = static_cast<int>(poly.px.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double u0 = poly.px[i] / R, u1 = poly.px[j] / R;
    const double v0 = poly.py[i] + vMid, v1 = poly.py[j] + vMid;
    const int steps = sagittaSteps(R, std::fabs(u1 - u0), defl, 1, 256);
    for (int k = 0; k < steps; ++k) {
      const double t = static_cast<double>(k) / steps;
      dp.push_back(tess::UV{u0 + (u1 - u0) * t, v0 + (v1 - v0) * t});
    }
  }
  return dp;
}

// The base cylinder wall over the INFLATED bbox window MINUS the polygon (radius R). The
// annular ring between the rectangular window (whose boundary is the SHARED u-sample grid
// that tileWallWithWindow removed) and the polygon hole is ear-clipped with-hole in (u,v)
// and mapped to the cylinder — so it welds to the surrounding full-turn wall (shared bbox
// boundary vertices) and to the pad side walls (shared polygon boundary vertices).
inline void emitBboxMinusPolygonWall(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double R,
                                     const USamples& us, const std::vector<tess::UV>& dpoly,
                                     double vMin, double vMax) {
  const int nw = us.nUwin;
  std::vector<tess::UV> pts;
  std::vector<int> outer, hole;
  pts.reserve(static_cast<std::size_t>(2 * nw + 2) + dpoly.size());
  for (int i = 0; i <= nw; ++i) {                       // bottom edge (v=vMin), uMin→uMax
    outer.push_back(static_cast<int>(pts.size()));
    pts.push_back(tess::UV{us.u[i], vMin});
  }
  for (int i = nw; i >= 0; --i) {                       // top edge (v=vMax), uMax→uMin
    outer.push_back(static_cast<int>(pts.size()));
    pts.push_back(tess::UV{us.u[i], vMax});
  }
  for (const tess::UV& d : dpoly) {                     // polygon hole
    hole.push_back(static_cast<int>(pts.size()));
    pts.push_back(d);
  }
  const std::vector<tess::UVTri> tris = tess::triangulatePolygon(pts, {outer, hole});
  for (const tess::UVTri& t : tris) {
    const tess::UV& A = pts[static_cast<std::size_t>(t.a)];
    const tess::UV& B = pts[static_cast<std::size_t>(t.b)];
    const tess::UV& C = pts[static_cast<std::size_t>(t.c)];
    const double uc = (A.u + B.u + C.u) / 3.0;
    const math::Vec3 outN = ax.x.vec() * std::cos(uc) + ax.y.vec() * std::sin(uc);
    emitTri(polys, ringPoint(ax, R, A.u, A.v), ringPoint(ax, R, B.u, B.v),
            ringPoint(ax, R, C.u, C.v), outN);
  }
}

// The pad cap (emboss outer cap / deboss pocket floor): the polygon ear-clipped at radius
// `rTarget`, outward = +radial (points into the void on both sides — away from the axis
// for an outer cap, into the pocket for a floor).
inline void emitPolygonCap(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double rTarget,
                           const std::vector<tess::UV>& dpoly) {
  const int m = static_cast<int>(dpoly.size());
  std::vector<int> loop(m);
  for (int i = 0; i < m; ++i) loop[i] = i;
  const std::vector<tess::UVTri> tris = tess::triangulatePolygon(dpoly, {loop});
  for (const tess::UVTri& t : tris) {
    const tess::UV& A = dpoly[static_cast<std::size_t>(t.a)];
    const tess::UV& B = dpoly[static_cast<std::size_t>(t.b)];
    const tess::UV& C = dpoly[static_cast<std::size_t>(t.c)];
    const double uc = (A.u + B.u + C.u) / 3.0;
    const math::Vec3 outN = ax.x.vec() * std::cos(uc) + ax.y.vec() * std::sin(uc);
    emitTri(polys, ringPoint(ax, rTarget, A.u, A.v), ringPoint(ax, rTarget, B.u, B.v),
            ringPoint(ax, rTarget, C.u, C.v), outN);
  }
}

// One ruled side wall per densified polygon edge, from radius R to `rTarget`. The outward
// hint is the edge's exterior normal in (u,v) — (dv,−du) for a CCW loop — mapped to 3D
// (u→tangential, v→axis); the sign flips it INTO the pocket for a deboss.
inline void emitPolygonSideWalls(std::vector<nb::Polygon>& polys, const math::Ax3& ax, double R,
                                 double rTarget, const std::vector<tess::UV>& dpoly, int boss) {
  const int m = static_cast<int>(dpoly.size());
  const double sign = (boss == 1) ? 1.0 : -1.0;
  for (int i = 0; i < m; ++i) {
    const tess::UV& a = dpoly[static_cast<std::size_t>(i)];
    const tess::UV& b = dpoly[static_cast<std::size_t>((i + 1) % m)];
    const double du = b.u - a.u, dv = b.v - a.v;
    const double um = 0.5 * (a.u + b.u);
    const math::Vec3 tang = ax.x.vec() * (-std::sin(um)) + ax.y.vec() * std::cos(um);
    const math::Vec3 hint = (tang * dv + ax.z.vec() * (-du)) * sign;
    const math::Point3 Ra = ringPoint(ax, R, a.u, a.v), Rb = ringPoint(ax, R, b.u, b.v);
    const math::Point3 Ta = ringPoint(ax, rTarget, a.u, a.v);
    const math::Point3 Tb = ringPoint(ax, rTarget, b.u, b.v);
    emitTri(polys, Ra, Rb, Tb, hint);
    emitTri(polys, Ra, Tb, Ta, hint);
  }
}

// Assemble a polygon-footprint emboss (boss) / deboss on a cylinder as a planar-facet
// soup. Reuses the rectangle machinery for everything OUTSIDE the polygon's (inflated)
// bounding box, then fills the bbox ring around the polygon with a with-hole ear-clip and
// closes the polygon with a cap + per-edge side walls. Empty on any degeneracy (depth ≥ R,
// off-wall / ≥2π inflated bbox, too-few points). Non-convex loops are accepted only if the
// engine self-verify (watertight + signed volume) passes; otherwise → OCCT.
inline std::vector<nb::Polygon> buildPolyCylinder(const CylWall& wall, const FootPoly& poly,
                                                  double height, int boss, double vMid,
                                                  double defl) {
  const math::Ax3& ax = wall.frame;
  const double R = wall.radius;
  if (!(height > kWeEps)) return {};
  const double rTarget = (boss == 1) ? (R + height) : (R - height);
  if (!(rTarget > kWeEps)) return {};  // deboss depth ≥ R

  // Inflate the polygon bbox so the polygon is a STRICT interior hole (its extreme
  // vertices otherwise touch the bbox edges, pinching the fill ring to zero width).
  const double spanX = poly.pxMax - poly.pxMin, spanY = poly.pyMax - poly.pyMin;
  const double mx = std::max(0.5, 0.1 * spanX), my = std::max(0.5, 0.1 * spanY);
  const double uMin = (poly.pxMin - mx) / R, uMax = (poly.pxMax + mx) / R;
  const double vMin = (poly.pyMin - my) + vMid, vMax = (poly.pyMax + my) + vMid;
  if (!(uMax - uMin > kWeEps) || uMax - uMin >= kWeTwoPi - kWeEps) return {};
  if (!(vMin > wall.vLo + kWeEps) || !(vMax < wall.vHi - kWeEps)) return {};

  const USamples us = uSamples(R, uMin, uMax, defl);
  const std::vector<tess::UV> dpoly = densifyPolyUV(poly, R, vMid, defl);
  if (dpoly.size() < 3) return {};

  std::vector<nb::Polygon> polys;
  tileWallWithWindow(polys, ax, R, us, wall.vLo, wall.vHi, vMin, vMax);  // full turn − bbox
  emitEndCap(polys, ax, R, wall.vLo, ax.z.vec() * -1.0, us);             // cylinder end caps
  emitEndCap(polys, ax, R, wall.vHi, ax.z.vec(), us);
  emitBboxMinusPolygonWall(polys, ax, R, us, dpoly, vMin, vMax);         // bbox ring − polygon
  emitPolygonCap(polys, ax, rTarget, dpoly);                            // pad cap / floor
  emitPolygonSideWalls(polys, ax, R, rTarget, dpoly, boss);             // pad walls
  return polys;
}

}  // namespace detail

// Wrap a footprint onto the CYLINDER lateral face `faceId` of `solid` and emboss (boss=1,
// RAISE a pad to R+height) or deboss (boss=0, RECESS a pocket to R−height). Three native
// tracks share the same deflection-bounded facet-soup / weld path:
//   * raised RECTANGLE (control) — the original byte-identical build.
//   * T1 DEBOSS RECTANGLE — the mirror inward pocket (volume shrinks).
//   * T2 NON-RECTANGULAR polygon — an N-vertex simple closed loop, embossed or debossed.
// Returns the built solid (welded watertight) or a NULL Shape (→ OCCT) when the input is
// out of the native slice: a non-cylindrical / freeform base (T3 declines — no native
// cone/sphere path), a self-intersecting / degenerate profile, a footprint that runs off
// the wall or spans ≥ a full turn, a deboss depth ≥ the radius, or non-positive height.
// `vMid` is the wall's axial middle (matching the OCCT oracle's V-mid centring). The
// ENGINE re-verifies watertight + signed volume and falls to OCCT on any failure.
inline topo::Shape wrap_emboss(const topo::Shape& solid, int faceId, const double* profileXY,
                               int count, double height, int boss, double deflection = 0.01) {
  if (boss != 0 && boss != 1) return {};   // only emboss / deboss
  if (!(height > detail::kWeEps)) return {};
  const auto wall = detail::cylinderWall(solid, faceId);  // T3: non-cylinder → decline
  if (!wall) return {};
  const double vMid = 0.5 * (wall->vLo + wall->vHi);

  std::vector<nb::Polygon> polys;
  const auto rect = detail::rectFootprint(profileXY, count);
  if (rect && boss == 1) {
    // CONTROL — raised rectangular pad (byte-identical to the first native slice).
    polys = detail::buildEmbossedCylinder(*wall, *rect, height, vMid, deflection);
  } else if (rect && boss == 0) {
    // T1 — recessed rectangular pocket.
    polys = detail::buildDebossedCylinder(*wall, *rect, height, vMid, deflection);
  } else {
    // T2 — non-rectangular polygon footprint (emboss or deboss).
    const auto poly = detail::polyFootprint(profileXY, count);
    if (!poly) return {};
    polys = detail::buildPolyCylinder(*wall, *poly, height, boss, vMid, deflection);
  }
  if (polys.size() < 6) return {};
  return nb::assembleSolid(polys);
}

}  // namespace cybercad::native::feature

#endif  // CYBERCAD_NATIVE_FEATURE_WRAP_EMBOSS_H

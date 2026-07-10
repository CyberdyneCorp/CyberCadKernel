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

// ── F5 FREEFORM (curved) BASE: sphere-cap pole boss ─────────────────────────────────
// The cylinder base above is DEVELOPABLE (arc-length preserves area, so the embossed
// volume is exactly footprint-area × height). A sphere base is NON-developable — no
// arc-length map both tiles the wall and keeps the raised-region volume equal to
// area × height. The tractable analytic-curved slice is therefore an AXISYMMETRIC pole
// boss: raise a CIRCULAR pole-cap patch of the sphere-cap dome RADIALLY from R to R+height
// over the polar-angle window φ ∈ [0, φ0]. The raised region is a SPHERICAL-SHELL SECTOR
// with the EXACT closed-form volume
//     ΔV = 2π(1 − cos φ0) · ((R+height)³ − R³) / 3        (solid angle × radial shell),
// so the result is watertight AND its volume delta is analytic — the same two-gate rigor
// as the cylinder arm, on a genuinely curved (freeform) base. The pole-cap half-angle φ0
// is derived from the profile's arc-length half-extent ρ (φ0 = ρ / R) taken as the profile
// bounding-box in-radius; the footprint is the pole DISC (an axisymmetric circular pattern
// wrapped onto the pole), NOT the wrapped profile outline — that keeps the volume EXACT.
//
// SCOPE (honest): native ONLY for a raised (boss=1) pole cap of a PURE sphere-cap dome
// (recognised wholesale: every face a coaxial sphere of the same centre/R, or EXACTLY ONE
// axis-normal cap that cuts the ball), with 0 < φ0 < the dome's own polar opening (the boss
// disc must sit strictly inside the dome, away from the rim). A deboss pole cap, a
// non-sphere/non-cone freeform base, an off-centre / multi-radius sphere, a spherical zone
// (two caps), or a φ0 that reaches the rim → NULL → OCCT (which itself declines a
// non-cylindrical wrap_emboss, so the sim gate compares native volume to an OCCT concentric-
// sphere reference measured by BRepGProp — see native_wrap_emboss_parity.mm).

// The pure sphere-cap dome resolved about a picked Sphere wall: centre, pole direction
// (toward the closed pole, away from the cap), outer radius R, and the cap-plane axial coord
// capA along +pole from the centre (dome material = axial ≥ capA). Recognised WHOLESALE.
struct SphereDome {
  math::Point3 centre;
  math::Dir3 pole;    // unit axis from the centre toward the closed pole
  double R = 0.0;
  double capA = 0.0;  // cap-plane axial coord along +pole from centre
};

inline std::optional<SphereDome> sphereDome(const topo::Shape& solid, int faceId) {
  const topo::ShapeMap fmap = topo::mapShapes(solid, topo::ShapeType::Face);
  if (faceId < 1 || static_cast<std::size_t>(faceId) > fmap.size()) return std::nullopt;
  const auto pick = topo::surfaceOf(fmap.shape(faceId));
  if (!pick || pick->surface->kind != topo::FaceSurface::Kind::Sphere) return std::nullopt;
  math::Ax3 pf = pick->surface->frame;
  if (!pick->location.isIdentity())
    pf.origin = pick->location.transform().applyToPoint(pf.origin);
  const math::Point3 centre = pf.origin;
  const double R = pick->surface->radius;
  if (!(R > kWeEps)) return std::nullopt;

  // Every face must be THIS ball (same centre/R) or an axis-normal planar cap. A full
  // revolve fragments both wall and cap into sectors, so match by GEOMETRY and collect the
  // one distinct cap plane (normal + signed offset w = n·p).
  bool haveCap = false;
  math::Vec3 capN{0, 0, 0};
  double capW = 0.0;
  int distinctCaps = 0;
  for (std::size_t fi = 1; fi <= fmap.size(); ++fi) {
    const auto surf = topo::surfaceOf(fmap.shape(static_cast<int>(fi)));
    if (!surf) return std::nullopt;
    if (surf->surface->kind == topo::FaceSurface::Kind::Sphere) {
      math::Point3 c = surf->surface->frame.origin;
      if (!surf->location.isIdentity()) c = surf->location.transform().applyToPoint(c);
      if (math::norm(c - centre) > 1e-6) return std::nullopt;                 // off-centre
      if (std::fabs(surf->surface->radius - R) > 1e-6) return std::nullopt;   // different R
    } else if (surf->surface->kind == topo::FaceSurface::Kind::Plane) {
      math::Ax3 fr = surf->surface->frame;
      if (!surf->location.isIdentity()) {
        const math::Transform& t = surf->location.transform();
        fr.origin = t.applyToPoint(fr.origin);
        fr.z = math::Dir3{t.applyToVector(fr.z.vec())};
      }
      const math::Vec3 n = fr.z.vec();
      const double w = math::dot(n, fr.origin.asVec());
      if (!haveCap) {
        capN = n;
        capW = w;
        haveCap = true;
        ++distinctCaps;
      } else {
        const bool same = std::fabs(math::dot(n, capN) - 1.0) < 1e-6 && std::fabs(w - capW) < 1e-6;
        const bool flipped = std::fabs(math::dot(n, capN) + 1.0) < 1e-6 &&
                             std::fabs(w + capW) < 1e-6;  // same plane, opposite winding
        if (!same && !flipped) ++distinctCaps;
      }
    } else {
      return std::nullopt;  // cylinder / cone / freeform → not a pure sphere-cap dome
    }
  }
  if (distinctCaps != 1 || !haveCap) return std::nullopt;  // exactly ONE cap (a zone declines)

  // The cap plane has normal capN (which a revolve may wind either way — NOT reliably
  // outward), so resolve the pole direction GEOMETRICALLY: the dome closes at an APEX one
  // radius from the centre, on ONE side of the cap plane. Scan the solid vertices for the
  // extremes of the signed axial coord along capN; the pole is the side whose extreme reaches
  // ≈ R from the centre (the closed apex), and the material fills from the cap toward it.
  const math::Vec3 nAxis = capN;  // unit (from a normalised plane frame)
  double aMin = 0.0, aMax = 0.0;
  bool any = false;
  for (topo::Explorer ex(solid, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const auto p = topo::pointOf(ex.current());
    if (!p) continue;
    const double a = math::dot(*p - centre, nAxis);
    if (!any) { aMin = aMax = a; any = true; }
    else { aMin = std::min(aMin, a); aMax = std::max(aMax, a); }
  }
  if (!any) return std::nullopt;
  // The apex sits at ±R from the centre along nAxis; the cap plane cuts the ball at the OTHER
  // extreme. Pick +pole as the direction toward whichever extreme is nearer +R (the apex).
  const bool poleAlongPlusN = std::fabs(aMax - R) <= std::fabs(aMin + R);
  const math::Dir3 pole{poleAlongPlusN ? nAxis : nAxis * -1.0};
  if (!pole.valid()) return std::nullopt;
  // Cap-plane axial coord along +pole from the centre. The cap plane offset along nAxis is
  // (capW − dot(capN,centre)) relative to the centre; flip its sign when +pole = −nAxis.
  const double capAlongN = capW - math::dot(capN, centre.asVec());
  const double aCap = poleAlongPlusN ? capAlongN : -capAlongN;
  if (!(std::fabs(aCap) < R - 1e-9)) return std::nullopt;  // cap must actually cut the ball
  // Sanity: the apex extreme along +pole must reach ≈ R (a genuine closed dome, not a slab).
  const double apexAlong = poleAlongPlusN ? aMax : -aMin;
  if (std::fabs(apexAlong - R) > 1e-3 * R + 1e-6) return std::nullopt;

  SphereDome g;
  g.centre = centre;
  g.pole = pole;
  g.R = R;
  g.capA = aCap;
  return g;
}

// The profile's arc-length in-radius ρ: half the SMALLER bbox extent (a circle inscribed in
// the footprint bbox). φ0 = ρ / R. Using the in-radius keeps the raised pole DISC strictly
// inside a footprint the caller drew, and gives the exact axisymmetric closed form.
inline double profileInRadius(const double* profileXY, int count) {
  double xmin = profileXY[0], xmax = profileXY[0], ymin = profileXY[1], ymax = profileXY[1];
  for (int i = 1; i < count; ++i) {
    xmin = std::min(xmin, profileXY[i * 2]);
    xmax = std::max(xmax, profileXY[i * 2]);
    ymin = std::min(ymin, profileXY[i * 2 + 1]);
    ymax = std::max(ymax, profileXY[i * 2 + 1]);
  }
  return 0.5 * std::min(xmax - xmin, ymax - ymin);
}

// A point on a concentric sphere of radius `rad` at longitude u and polar angle φ from the
// pole (φ=0 at the pole, increasing toward the cap): axial coord along pole = rad·cosφ, off-
// axis radius = rad·sinφ. (ex,ey) span the plane ⟂ pole.
inline math::Point3 poleSpherePt(const SphereDome& g, const math::Vec3& ex, const math::Vec3& ey,
                                 double rad, double u, double phi) {
  const double a = rad * std::cos(phi);
  const double rho = rad * std::sin(phi);
  const math::Vec3 p = g.centre.asVec() + g.pole.vec() * a + (ex * std::cos(u) + ey * std::sin(u)) * rho;
  return math::Point3{p.x, p.y, p.z};
}

// Build the sphere-cap dome with a RAISED circular pole cap (φ ∈ [0,φ0] lifted R→R+height)
// as a planar-facet soup, welded on shared u-samples across the boss-rim seam. Parts:
//   * base dome wall from the CAP latitude up to the boss-rim latitude φ0 at radius R;
//   * boss OUTER cap: the φ∈[0,φ0] pole cap at radius R+height (curved patch);
//   * boss RIM wall: the annular frustum from R to R+height along the φ0 circle;
//   * the flat disc cap at capA closing the dome bottom.
// Empty on degeneracy. The φ0-rim ring on the base wall shares vertices with the rim wall.
inline std::vector<nb::Polygon> buildSpherePoleBoss(const SphereDome& g, double phi0, double height,
                                                    double defl) {
  const double R = g.R, Rout = R + height;
  if (!(height > kWeEps) || !(phi0 > kWeEps)) return {};
  // The dome's own polar opening (cap latitude): φ from the pole to the cap plane is
  // φcap = acos(capA / R). The boss must sit strictly inside → φ0 < φcap.
  const double phiCap = std::acos(std::clamp(g.capA / R, -1.0, 1.0));
  if (!(phi0 < phiCap - 1e-6)) return {};

  const math::Vec3 pole = g.pole.vec();
  math::Vec3 seed = std::fabs(pole.x) < 0.9 ? math::Vec3{1, 0, 0} : math::Vec3{0, 1, 0};
  math::Vec3 exV = seed - pole * math::dot(seed, pole);
  const double exn = math::norm(exV);
  if (!(exn > kWeEps)) return {};
  exV = exV * (1.0 / exn);
  const math::Vec3 eyV = math::cross(pole, exV);

  const int N = sagittaSteps(R, kWeTwoPi, defl, 8, 256);  // longitude samples (shared seam)
  std::vector<nb::Polygon> polys;
  polys.reserve(static_cast<std::size_t>(N) * 8 + 4);

  auto pt = [&](double rad, double u, double phi) { return poleSpherePt(g, exV, eyV, rad, u, phi); };
  auto uAt = [&](int i) { return kWeTwoPi * i / N; };

  // Base dome wall: φ from φ0 down to φcap at radius R (latitude bands). Outward = radial.
  const int Mbase = sagittaSteps(R, std::fabs(phiCap - phi0), defl, 2, 128);
  for (int j = 0; j < Mbase; ++j) {
    const double p0 = phi0 + (phiCap - phi0) * j / Mbase;
    const double p1 = phi0 + (phiCap - phi0) * (j + 1) / Mbase;
    for (int i = 0; i < N; ++i) {
      const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
      const math::Vec3 outN =
          (pole * std::cos(0.5 * (p0 + p1)) + (exV * std::cos(um) + eyV * std::sin(um)) * std::sin(0.5 * (p0 + p1)));
      emitTri(polys, pt(R, u0, p0), pt(R, u1, p0), pt(R, u1, p1), outN);
      emitTri(polys, pt(R, u0, p0), pt(R, u1, p1), pt(R, u0, p1), outN);
    }
  }
  // Boss OUTER cap: φ ∈ [0,φ0] at radius Rout. Top band closes to the pole as triangles.
  const int Mboss = sagittaSteps(Rout, phi0, defl, 2, 128);
  for (int j = 0; j < Mboss; ++j) {
    const double p0 = phi0 * j / Mboss, p1 = phi0 * (j + 1) / Mboss;
    for (int i = 0; i < N; ++i) {
      const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
      const double pm = 0.5 * (p0 + p1);
      const math::Vec3 outN =
          (pole * std::cos(pm) + (exV * std::cos(um) + eyV * std::sin(um)) * std::sin(pm));
      emitTri(polys, pt(Rout, u0, p0), pt(Rout, u1, p0), pt(Rout, u1, p1), outN);
      emitTri(polys, pt(Rout, u0, p0), pt(Rout, u1, p1), pt(Rout, u0, p1), outN);
    }
  }
  // Boss RIM wall: annular frustum at longitude, from radius R to Rout along the φ0 circle.
  // Outward hint points radially away from the pole axis at φ0 (the ring's exterior side).
  for (int i = 0; i < N; ++i) {
    const double u0 = uAt(i), u1 = uAt(i + 1), um = 0.5 * (u0 + u1);
    const math::Vec3 outw = (exV * std::cos(um) + eyV * std::sin(um)) * std::sin(phi0) +
                            pole * std::cos(phi0);
    emitTri(polys, pt(R, u0, phi0), pt(R, u1, phi0), pt(Rout, u1, phi0), outw);
    emitTri(polys, pt(R, u0, phi0), pt(Rout, u1, phi0), pt(Rout, u0, phi0), outw);
  }
  // Flat disc cap at the cap plane (φcap circle at radius R). Outward = −pole.
  {
    std::vector<math::Point3> rim;
    rim.reserve(static_cast<std::size_t>(N));
    for (int i = 0; i < N; ++i) rim.push_back(pt(R, uAt(i), phiCap));
    emitFlat(polys, std::move(rim), pole * -1.0);
  }
  return polys;
}

}  // namespace detail

// The exact closed-form embossed-volume DELTA for the sphere-cap pole boss: a spherical-
// shell sector, ΔV = 2π(1 − cos φ0)·((R+h)³ − R³)/3. Exposed for the engine self-verify and
// host/sim gates. Returns 0 when the picked face is not a recognised sphere-cap dome wall or
// the boss is out of scope (so the caller uses the cylinder area×height gate instead).
inline double spherePoleBossVolumeDelta(const topo::Shape& solid, int faceId,
                                        const double* profileXY, int count, double height) {
  if (profileXY == nullptr || count < 3 || !(height > detail::kWeEps)) return 0.0;
  const auto g = detail::sphereDome(solid, faceId);
  if (!g) return 0.0;
  const double rho = detail::profileInRadius(profileXY, count);
  if (!(rho > detail::kWeEps)) return 0.0;
  const double phi0 = rho / g->R;
  const double phiCap = std::acos(std::clamp(g->capA / g->R, -1.0, 1.0));
  if (!(phi0 > detail::kWeEps) || !(phi0 < phiCap - 1e-6)) return 0.0;
  const double Rout = g->R + height;
  return detail::kWeTwoPi * (1.0 - std::cos(phi0)) * (Rout * Rout * Rout - g->R * g->R * g->R) / 3.0;
}

// Wrap a footprint onto the CYLINDER lateral face `faceId` of `solid` and emboss (boss=1,
// RAISE a pad to R+height) or deboss (boss=0, RECESS a pocket to R−height). Three native
// tracks share the same deflection-bounded facet-soup / weld path:
//   * raised RECTANGLE (control) — the original byte-identical build.
//   * T1 DEBOSS RECTANGLE — the mirror inward pocket (volume shrinks).
//   * T2 NON-RECTANGULAR polygon — an N-vertex simple closed loop, embossed or debossed.
// A fourth arm handles a RAISED pole cap on a sphere-cap dome (F5 curved base, exact
// shell-sector volume). Returns the built solid (welded watertight) or a NULL Shape (→ OCCT)
// when the input is out of the native slice: a non-cylindrical, non-sphere-cap freeform base
// (T3 declines — no native cone / general-spline path), a self-intersecting / degenerate
// profile, a footprint that runs off
// the wall or spans ≥ a full turn, a deboss depth ≥ the radius, or non-positive height.
// `vMid` is the wall's axial middle (matching the OCCT oracle's V-mid centring). The
// ENGINE re-verifies watertight + signed volume and falls to OCCT on any failure.
inline topo::Shape wrap_emboss(const topo::Shape& solid, int faceId, const double* profileXY,
                               int count, double height, int boss, double deflection = 0.01) {
  if (boss != 0 && boss != 1) return {};   // only emboss / deboss
  if (!(height > detail::kWeEps)) return {};

  // F5 FREEFORM (curved) base: a RAISED pole cap on a sphere-cap dome. The tractable
  // analytic-curved slice — exact spherical-shell-sector volume. Only boss=1 (a pole
  // deboss / non-sphere freeform base is declined below → OCCT).
  if (boss == 1) {
    if (const auto g = detail::sphereDome(solid, faceId)) {
      const double rho = detail::profileInRadius(profileXY, count);
      if (!(rho > detail::kWeEps)) return {};
      const double phi0 = rho / g->R;
      std::vector<nb::Polygon> spolys = detail::buildSpherePoleBoss(*g, phi0, height, deflection);
      if (spolys.size() < 6) return {};  // out of scope (φ0 reaches the rim / degenerate)
      return nb::assembleSolid(spolys);
    }
  }

  const auto wall = detail::cylinderWall(solid, faceId);  // T3: other freeform → decline
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

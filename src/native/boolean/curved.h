// SPDX-License-Identifier: Apache-2.0
//
// curved.h — the NARROW, ANALYTIC curved-boolean slice: axis-aligned box <-> a
// cylinder whose axis is parallel to a box axis (Phase 4 #5 `native-booleans`,
// the deferred residual #2 in openspec/NATIVE-REWRITE.md).
//
// ── WHY A SEPARATE PATH FROM THE PLANAR BSP-CSG ───────────────────────────────
// The planar boolean (bsp.h/polygon.h/assemble.h) flattens every face to a planar
// polygon and does set algebra on the polygon soup. A cylinder's lateral face is
// CURVED, so it cannot be flattened without either (a) faceting it into a polygonal
// prism — which loses the analytic volume and would FAIL the mandatory self-verify
// (boxVol ± πr²h), or (b) a general surface-surface intersection engine, which is
// research-grade and explicitly OUT OF SCOPE. Instead, for the ONE tractable family
// where the plane-cylinder intersection is ANALYTIC, we recognise the configuration
// and build the RESULT B-rep directly out of the SAME watertight primitives the
// native construct library already ships (true Plane caps, true Circle rim edges,
// true Cylinder walls — the exact face/edge kinds a native cylinder-with-holes prism
// uses), so the result tessellates watertight by the identical mesher path and
// carries the EXACT analytic volume. Nothing is faceted; nothing is approximated.
//
// ── THE ANALYTIC FAMILY (honest, narrow) ──────────────────────────────────────
// Operands: one AXIS-ALIGNED BOX and one CYLINDER whose axis ∥ a box axis (X/Y/Z).
// The plane-cylinder intersection is analytic here:
//   * a box face PERPENDICULAR to the cylinder axis cuts the cylinder in a CIRCLE;
//   * a box face PARALLEL to the axis would cut it in ruling LINE segments — but to
//     keep the result a clean, watertight, analytically-measurable solid we require
//     the cylinder to sit RADIALLY INSIDE the box cross-section (the round feature
//     does not breach a side wall). The lateral surface therefore meets ONLY the two
//     perpendicular caps, in two circles — the round-hole / round-boss / disc family.
// Axially, the cylinder either spans THROUGH the box (a through feature) or its caps
// fall inside the box (a blind feature is deferred — see the guards below).
//
// The three ops then have closed-form B-reps:
//   cut  (box − cyl) = the box with a round THROUGH HOLE  → build_prism_with_holes
//                      (an OUTER rectangle + a CIRCULAR hole = Cylinder wall + two
//                       annular Plane caps). Volume = boxVol − πr²·h.
//   fuse (box ∪ cyl) = the box with a round BOSS standing proud on the cap face(s):
//                      box shell (its cap face trimmed by the boss circle where the
//                      cylinder emerges) + the protruding cylinder segment capped by
//                      a disc. Volume = boxVol + πr²·(protruding length).
//   common(box ∩ cyl)= the cylinder SEGMENT clipped to the box's axial extent — a
//                      shorter cylinder (disc caps + wall). Volume = πr²·overlapLen.
//
// ── SELF-VERIFY IS STILL THE ENGINE'S JOB ─────────────────────────────────────
// As with the planar path, boolean_solid returns the assembled Solid (or NULL when
// the configuration is not in this analytic family) and the engine runs the
// MANDATORY watertight + analytic-volume guard, DISCARDING anything that does not
// match and falling through to OCCT. Sphere, cone, non-axis-aligned cylinders,
// cylinder-cylinder, NURBS, near-tangent, radially-breaching, and blind features all
// return NULL here → OCCT (labelled, verified, never faked).
//
// CLEAN-ROOM. Uses src/native/math + topology + construct/profile.h. No OCCT. The
// OCCT BRepAlgoAPI/IntTools was consulted only as an oracle for the analytic curve
// forms (a plane ⟂ a cylinder axis → a circle; ∥ → rulings) — nothing is copied.
//
// clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_CURVED_H
#define CYBERCAD_NATIVE_BOOLEAN_CURVED_H

#include "native/boolean/polygon.h"            // Plane / Polygon / isAllPlanar
#include "native/construct/native_construct.h" // build_prism_with_holes, CircleHole
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean::curved {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace ncst = cybercad::native::construct;

// Tolerance for the analytic recognition. Looser than raw fp so a box/cylinder built
// through the frame math still classifies as axis-aligned; tight enough that a
// genuinely oblique cylinder or a radially-breaching feature is rejected.
inline constexpr double kAxisEps = 1e-6;
inline constexpr double kPi = 3.14159265358979323846;

// ─────────────────────────────────────────────────────────────────────────────
// AABox — an axis-aligned box, world coordinates.
// ─────────────────────────────────────────────────────────────────────────────
struct AABox {
  math::Point3 lo{};
  math::Point3 hi{};

  double size(int axis) const noexcept { return hi[axis] - lo[axis]; }
  double volume() const noexcept { return size(0) * size(1) * size(2); }
};

// ─────────────────────────────────────────────────────────────────────────────
// AxisCylinder — a finite cylinder whose axis is one of the world axes X/Y/Z.
//   axis   : 0=X, 1=Y, 2=Z (the direction the cylinder extends along).
//   c0, c1 : the two coordinates PERPENDICULAR to the axis of the centre line
//            (stored in ascending-axis order: for axis Z these are (cx, cy)).
//   radius : cylinder radius.
//   lo, hi : axial extent along `axis` (lo < hi).
// ─────────────────────────────────────────────────────────────────────────────
struct AxisCylinder {
  int axis = 2;
  double c0 = 0.0;  // first perpendicular-axis centre coord
  double c1 = 0.0;  // second perpendicular-axis centre coord
  double radius = 0.0;
  double lo = 0.0;
  double hi = 0.0;

  double length() const noexcept { return hi - lo; }
  double volume() const noexcept { return kPi * radius * radius * length(); }

  // The two world axes perpendicular to `axis`, in ascending order.
  std::array<int, 2> perpAxes() const noexcept {
    switch (axis) {
      case 0: return {1, 2};  // X axis → perp Y,Z
      case 1: return {0, 2};  // Y axis → perp X,Z
      default: return {0, 1}; // Z axis → perp X,Y
    }
  }
};

namespace detail {

// World-space AABB of a solid's B-rep vertices. Exact for a box / an axis-aligned
// cylinder (whose extremes are the cap-circle points, all present as topological
// vertices only at the angular stations — so for the cylinder we bound via the
// analytic cap circles instead; see recogniseCylinder). Here it is used only for
// the BOX, whose 8 corners are all real vertices.
inline std::optional<AABox> vertexAABB(const topo::Shape& s) {
  bool any = false;
  math::Point3 lo{}, hi{};
  for (topo::Explorer ex(s, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const auto p = topo::pointOf(ex.current());
    if (!p) continue;
    if (!any) {
      lo = hi = *p;
      any = true;
    } else {
      lo = math::Point3{std::min(lo.x, p->x), std::min(lo.y, p->y), std::min(lo.z, p->z)};
      hi = math::Point3{std::max(hi.x, p->x), std::max(hi.y, p->y), std::max(hi.z, p->z)};
    }
  }
  if (!any) return std::nullopt;
  return AABox{lo, hi};
}

// True iff a unit direction is aligned (within tolerance) with world axis `k`
// (its |k|-component is ±1 and the other two components are ~0).
inline bool alignedWithAxis(const math::Vec3& n, int k) noexcept {
  const double comp[3] = {std::fabs(n.x), std::fabs(n.y), std::fabs(n.z)};
  if (std::fabs(comp[k] - 1.0) > kAxisEps) return false;
  for (int i = 0; i < 3; ++i)
    if (i != k && comp[i] > kAxisEps) return false;
  return true;
}

// World-space normal of a planar face: the surface frame Z folded through the
// face's surface location and the face's own location.
inline math::Vec3 worldFaceNormal(const topo::Shape& face,
                                  const topo::FaceSurfaceResult& surf) noexcept {
  math::Vec3 n = surf.surface->frame.z.vec();
  if (!surf.location.isIdentity()) n = surf.location.transform().applyToVector(n);
  if (!face.location().isIdentity()) n = face.location().transform().applyToVector(n);
  return n;
}

// True iff every face is planar with a world-axis-aligned normal and the solid has
// exactly 6 such faces with non-degenerate extents (a genuine axis-aligned box in
// this narrow slice).
inline bool isAxisAlignedBox(const topo::Shape& s, const AABox& bb) {
  int planarFaces = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = topo::surfaceOf(ex.current());
    if (!surf || surf->surface->kind != topo::FaceSurface::Kind::Plane) return false;
    const math::Vec3 n = worldFaceNormal(ex.current(), *surf);
    if (!(alignedWithAxis(n, 0) || alignedWithAxis(n, 1) || alignedWithAxis(n, 2))) return false;
    ++planarFaces;
  }
  if (planarFaces != 6) return false;  // a genuine 6-faced box in this narrow slice
  // Non-degenerate extents.
  for (int k = 0; k < 3; ++k)
    if (!(bb.size(k) > kAxisEps)) return false;
  return true;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// recogniseBox — a native solid that is an axis-aligned box → its AABox.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<AABox> recogniseBox(const topo::Shape& s) {
  if (s.isNull() || !isAllPlanar(s)) return std::nullopt;
  const auto bb = detail::vertexAABB(s);
  if (!bb) return std::nullopt;
  if (!detail::isAxisAlignedBox(s, *bb)) return std::nullopt;
  return bb;
}

namespace detail {

// The analytic axis / origin / radius of ONE cylindrical face, in world coords, or
// nullopt if the face's axis is not world-aligned or the radius is degenerate.
struct CylFace {
  int axis;
  math::Point3 origin;
  double radius;
};
inline std::optional<CylFace> analyseCylFace(const topo::Shape& face,
                                             const topo::FaceSurfaceResult& surf) {
  math::Vec3 z = surf.surface->frame.z.vec();
  if (!surf.location.isIdentity()) z = surf.location.transform().applyToVector(z);
  if (!face.location().isIdentity()) z = face.location().transform().applyToVector(z);
  int a = -1;
  for (int k = 0; k < 3; ++k)
    if (alignedWithAxis(z, k)) a = k;
  if (a < 0) return std::nullopt;  // oblique cylinder → OCCT

  math::Point3 o = surf.surface->frame.origin;
  if (!surf.location.isIdentity()) o = surf.location.transform().applyToPoint(o);
  if (!face.location().isIdentity()) o = face.location().transform().applyToPoint(o);
  if (!(surf.surface->radius > kAxisEps)) return std::nullopt;
  return CylFace{a, o, surf.surface->radius};
}

// Two cylindrical faces share ONE analytic cylinder (same world-aligned axis, radius,
// and perpendicular centre) — the revolve tiles a cylinder into several patches that
// must all agree, else the solid is not a single cylinder in our narrow family.
inline bool sameCylinder(const CylFace& a, const CylFace& b) {
  if (a.axis != b.axis || std::fabs(a.radius - b.radius) > kAxisEps) return false;
  const auto perp = AxisCylinder{a.axis, 0, 0, 0, 0, 0}.perpAxes();
  return std::fabs(a.origin[perp[0]] - b.origin[perp[0]]) <= kAxisEps &&
         std::fabs(a.origin[perp[1]] - b.origin[perp[1]]) <= kAxisEps;
}

// Fold every face into a single shared CylFace, requiring every face to be either a
// Cylinder patch (all sharing one cylinder) or a Plane cap ⟂ that axis. Returns
// nullopt for any Cone/Sphere/free-form face, an oblique cylinder, a mismatched
// cylinder patch, or a slanted cap. A two-phase merge folds the axis in the first
// cylinder face seen, then checks caps against it.
inline std::optional<CylFace> foldCylinderFaces(const topo::Shape& s) {
  std::optional<CylFace> merged;
  std::vector<topo::Shape> planeFaces;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = topo::surfaceOf(ex.current());
    if (!surf) return std::nullopt;
    const auto kind = surf->surface->kind;
    if (kind == topo::FaceSurface::Kind::Cylinder) {
      const auto cf = analyseCylFace(ex.current(), *surf);
      if (!cf) return std::nullopt;
      if (!merged) merged = cf;
      else if (!sameCylinder(*merged, *cf)) return std::nullopt;
    } else if (kind == topo::FaceSurface::Kind::Plane) {
      planeFaces.push_back(ex.current());
    } else {
      return std::nullopt;  // Cone / Sphere / free-form → OCCT
    }
  }
  if (!merged) return std::nullopt;
  for (const topo::Shape& face : planeFaces) {
    const auto surf = topo::surfaceOf(face);
    if (!detail::alignedWithAxis(detail::worldFaceNormal(face, *surf), merged->axis))
      return std::nullopt;  // slanted cap → OCCT
  }
  return merged;
}

// Axial extent [lo,hi] of a solid's vertices projected onto world `axis`, or nullopt
// if there are no vertices or the span is degenerate.
inline std::optional<std::pair<double, double>> axialExtent(const topo::Shape& s, int axis) {
  bool have = false;
  double lo = 0.0, hi = 0.0;
  for (topo::Explorer ex(s, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    const auto p = topo::pointOf(ex.current());
    if (!p) continue;
    const double t = (*p)[axis];
    lo = have ? std::min(lo, t) : t;
    hi = have ? std::max(hi, t) : t;
    have = true;
  }
  if (!have || !(hi - lo > kAxisEps)) return std::nullopt;
  return std::make_pair(lo, hi);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// recogniseCylinder — a native solid that is a finite cylinder with its axis
// parallel to a world axis → its AxisCylinder. The native cylinder (produced by
// build_revolution) has exactly one Cylinder lateral face + two planar disc caps
// (the revolve tiles the lateral face into ≥3 Cylinder patches sharing the SAME
// analytic Cylinder surface, and each cap into ≥3 planar patches). We accept a solid
// whose faces are ONLY Cylinder (all sharing one axis+radius) and Plane (⟂ that
// axis), and derive the axial extent from the cylinder vertices.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<AxisCylinder> recogniseCylinder(const topo::Shape& s) {
  if (s.isNull()) return std::nullopt;
  const auto cf = detail::foldCylinderFaces(s);
  if (!cf) return std::nullopt;
  const auto extent = detail::axialExtent(s, cf->axis);
  if (!extent) return std::nullopt;

  AxisCylinder cyl;
  cyl.axis = cf->axis;
  const auto perp = cyl.perpAxes();
  cyl.c0 = cf->origin[perp[0]];
  cyl.c1 = cf->origin[perp[1]];
  cyl.radius = cf->radius;
  cyl.lo = extent->first;
  cyl.hi = extent->second;
  return cyl;
}

// ─────────────────────────────────────────────────────────────────────────────
// WORLD-FRAME axis-aware primitive builders.
//
// These emit geometry DIRECTLY in world coordinates for a chosen world axis (0=X,
// 1=Y, 2=Z), with identity Locations — never via a Location applied to a canonical
// Z-frame build. (The native tessellator folds a solid-level Location into a planar
// face's vertices but does NOT re-place an analytic Cylinder surface's frame under
// that Location, so a Z-canonical holed prism placed with a rotation/translation
// Location meshes with an inconsistent wall — the hole's cylindrical wall detaches
// from its planar annulus caps. Building world geometry directly sidesteps that
// entirely: every face frame is already correct, so the mesher meshes each patch as
// it does for a natively built Y-axis cylinder — proven watertight.)
//
// axisWorldPoint maps a perpendicular-plane pair (u,v) at axial coord t onto world
// coordinates: u → the first perpendicular axis, v → the second, t → the axis.
// ─────────────────────────────────────────────────────────────────────────────
inline math::Point3 axisWorldPoint(int axis, double u, double v, double t) noexcept {
  math::Point3 p{0, 0, 0};
  const std::array<int, 2> perp = AxisCylinder{axis, 0, 0, 0, 0, 0}.perpAxes();
  double c[3];
  c[perp[0]] = u;
  c[perp[1]] = v;
  c[axis] = t;
  p.x = c[0];
  p.y = c[1];
  p.z = c[2];
  return p;
}

// A RIGHT-HANDED world-aligned frame for `axis`: fz = +axis (the axial direction),
// fx = the first perpendicular world axis, and fy = fz × fx so that fx × fy = fz
// EXACTLY (a proper, non-mirrored frame). This matters: the analytic Cylinder
// normal and the signed enclosed-volume both assume a right-handed placement — a
// left-handed frame (which a naive fx=perp0, fy=perp1 gives for axis Y, since
// X × Z = −Y) flips normals and corrupts the volume. fy therefore equals +perp1 for
// axis X/Z and −perp1 (i.e. −X) for axis Y; either way the frame is right-handed.
inline void axisFrameDirs(int axis, math::Dir3& fx, math::Dir3& fy, math::Dir3& fz) noexcept {
  const std::array<int, 2> perp = AxisCylinder{axis, 0, 0, 0, 0, 0}.perpAxes();
  auto e = [](int k) {
    return math::Dir3{k == 0 ? 1.0 : 0.0, k == 1 ? 1.0 : 0.0, k == 2 ? 1.0 : 0.0};
  };
  fz = e(axis);
  fx = e(perp[0]);
  fy = math::Dir3{math::cross(fz.vec(), fx.vec())};  // right-handed: fx × fy = fz
}

namespace detail {

// A full circle rim edge of radius r about the axis centre line, at axial coord t.
// The circle lies in the plane ⟂ `axis` through t; its frame X/Y are the two perp
// world axes so its parametrization matches the cylinder wall's u = θ.
inline topo::Shape rimEdge(int axis, double c0, double c1, double r, double t) {
  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);
  const math::Point3 centre = axisWorldPoint(axis, c0, c1, t);
  const math::Point3 start = axisWorldPoint(axis, c0 + r, c1, t);
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = math::Ax3{centre, fx, fy, fz};
  c.radius = r;
  const topo::Shape v = topo::ShapeBuilder::makeVertex(start);
  return topo::ShapeBuilder::makeEdge(c, 0.0, 2.0 * kPi, v, v);
}

// The Cylinder wall face over the axial span [t0,t1] (one full-period patch). Its two
// boundary rim circles (v = 0 at t0, v = span at t1) are the shared seams the caps
// weld to. `outward` = true keeps the natural outward radial normal (a solid's outer
// wall / a boss); false reverses it (a hole wall, facing into the material).
inline topo::Shape cylinderWall(int axis, double c0, double c1, double r, double t0, double t1,
                                bool outward) {
  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);
  const double span = t1 - t0;
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Cylinder;
  s.frame = math::Ax3{axisWorldPoint(axis, c0, c1, t0), fx, fy, fz};
  s.radius = r;

  auto rimPc = [&](double vconst) {
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = math::Point3{0.0, vconst, 0.0};
    pc.dir2d = math::Vec3{1.0, 0.0, 0.0};  // u = θ
    return pc;
  };
  const topo::Shape rb = rimEdge(axis, c0, c1, r, t0);
  const topo::Shape rt = rimEdge(axis, c0, c1, r, t1);
  const topo::Shape rbp = topo::ShapeBuilder::addPCurve(rb, rb.tshape(), rimPc(0.0));
  const topo::Shape rtp = topo::ShapeBuilder::addPCurve(rt, rt.tshape(), rimPc(span));
  const topo::Shape wire =
      topo::ShapeBuilder::makeWire({rbp, rtp.oriented(topo::Orientation::Reversed)});
  return topo::ShapeBuilder::makeFace(s, wire, {},
                                      outward ? topo::Orientation::Forward
                                              : topo::Orientation::Reversed);
}

// A planar cap ⟂ `axis` at axial coord t. The plane frame's Z is ±axis (outward),
// X/Y the two perp world axes. Given an outer wire and optional hole wires, builds
// the trimmed face. `outwardPositive` selects the +axis (true) or −axis (false)
// outward normal.
inline topo::Shape planarCapFace(int axis, double t, bool outwardPositive,
                                 const topo::Shape& outerWire, std::vector<topo::Shape> holeWires) {
  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);
  const math::Dir3 nz = outwardPositive ? fz : fz.reversed();
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  // Frame Z is the outward normal; X kept, Y = Z × X to stay right-handed.
  s.frame = math::Ax3{axisWorldPoint(axis, 0, 0, t), fx,
                      math::Dir3{math::cross(nz.vec(), fx.vec())}, nz};
  return topo::ShapeBuilder::makeFace(s, outerWire, std::move(holeWires),
                                      topo::Orientation::Forward);
}

// Order the 4 box-corner vertices so the loop winds CCW as seen from the cap's
// OUTWARD normal (frame.z), so the trimmer reads the rectangle as the material-side
// outer boundary. Winding is decided from the loop's own area normal vs frame.z —
// axis-independent (no hard-coded per-axis reversal, which was only right for a
// right-handed (u,v)→(perp0,perp1) plane; for axis Y that plane is left-handed).
inline std::array<topo::Shape, 4> ccwRectOrder(const std::array<topo::Shape, 4>& ring,
                                               const math::Ax3& frame) {
  math::Vec3 area{0, 0, 0};
  for (int i = 0; i < 4; ++i) {
    const math::Point3 a = *topo::pointOf(ring[i]);
    const math::Point3 b = *topo::pointOf(ring[(i + 1) % 4]);
    area += math::cross(a.asVec(), b.asVec());
  }
  if (math::dot(area, frame.z.vec()) >= 0.0) return ring;  // already CCW from outward
  return {ring[0], ring[3], ring[2], ring[1]};             // reverse
}

// One box side-wall quad (ringLo[i]→ringLo[j]→ringHi[j]→ringHi[i]) with a robust
// outward normal and consistent winding. The outward normal points from the box
// CENTRE toward the wall midpoint (axis- and handedness-independent — the earlier
// cross(edge, axis) rule silently produced INWARD normals for the axis-Y box, whose
// (u,v)→(X,Z) plane is left-handed, so half the walls wound backward and the signed
// volume partially cancelled). The quad is reordered to wind CCW as seen from that
// normal so the meshed triangles all face out.
inline topo::Shape boxSideWall(const topo::Shape& lo0, const topo::Shape& lo1,
                               const topo::Shape& hi1, const topo::Shape& hi0,
                               const math::Point3& boxCentre) {
  std::array<topo::Shape, 4> q{lo0, lo1, hi1, hi0};
  const math::Point3 mid{
      0.25 * (topo::pointOf(lo0)->x + topo::pointOf(lo1)->x + topo::pointOf(hi1)->x +
              topo::pointOf(hi0)->x),
      0.25 * (topo::pointOf(lo0)->y + topo::pointOf(lo1)->y + topo::pointOf(hi1)->y +
              topo::pointOf(hi0)->y),
      0.25 * (topo::pointOf(lo0)->z + topo::pointOf(lo1)->z + topo::pointOf(hi1)->z +
              topo::pointOf(hi0)->z)};
  const math::Dir3 outward{mid - boxCentre};
  // Reorder so the loop winds CCW as seen from `outward`.
  math::Vec3 area{0, 0, 0};
  for (int i = 0; i < 4; ++i)
    area += math::cross(topo::pointOf(q[i])->asVec(), topo::pointOf(q[(i + 1) % 4])->asVec());
  if (math::dot(area, outward.vec()) < 0.0) q = {q[0], q[3], q[2], q[1]};
  return ncst::detail::planarFace({q[0], q[1], q[2], q[3]}, outward, topo::Orientation::Forward);
}

// A straight world edge between two shared vertices with a Line pcurve on the cap
// plane frame (matching construct.h planarEdge but for the world-axis cap frame).
inline topo::Shape capEdge(const topo::Shape& v0, const topo::Shape& v1, const math::Ax3& frame) {
  const auto p0 = topo::pointOf(v0);
  const auto p1 = topo::pointOf(v1);
  const math::Vec3 d = *p1 - *p0;
  const double len = std::max(math::norm(d), kAxisEps);
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Line;
  c.frame.origin = *p0;
  c.frame.x = math::norm(d) > kAxisEps ? math::Dir3{d} : math::Dir3{1, 0, 0};
  c.frame.z = frame.z;
  const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);
  auto toUV = [&](const math::Point3& p) {
    const math::Vec3 dd = p - frame.origin;
    return math::Point3{math::dot(dd, frame.x.vec()), math::dot(dd, frame.y.vec()), 0.0};
  };
  const math::Point3 uv0 = toUV(*p0), uv1 = toUV(*p1);
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Line;
  pc.origin2d = uv0;
  pc.dir2d = (uv1 - uv0) / len;
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// A circle wire (one closed Circle edge) on a cap plane at axial t, for the hole /
// disc boundary — with a Circle pcurve so the trimmer flattens it.
inline topo::Shape circleWire(int axis, double c0, double c1, double r, double t,
                              const math::Ax3& frame) {
  const topo::Shape edge = rimEdge(axis, c0, c1, r, t);
  // pcurve on the cap frame: centre in UV + radius (dir2d.x carries r).
  const math::Point3 centre = axisWorldPoint(axis, c0, c1, t);
  const math::Vec3 dc = centre - frame.origin;
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Circle;
  pc.origin2d =
      math::Point3{math::dot(dc, frame.x.vec()), math::dot(dc, frame.y.vec()), 0.0};
  pc.dir2d = math::Vec3{r, 0.0, 0.0};
  return topo::ShapeBuilder::makeWire({topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc)});
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// buildCommonSegment — box ∩ cylinder = the cylinder clipped to the box's axial
// extent: a finite cylinder over [lo,hi] (one Cylinder wall + two disc Plane caps),
// built directly in world coordinates. Volume = πr²·(hi−lo).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape buildCommonSegment(const AABox& box, const AxisCylinder& cyl) {
  const int axis = cyl.axis;
  const double lo = std::max(cyl.lo, box.lo[axis]);
  const double hi = std::min(cyl.hi, box.hi[axis]);
  if (!(hi - lo > kAxisEps)) return {};
  const double c0 = cyl.c0, c1 = cyl.c1, r = cyl.radius;

  std::vector<topo::Shape> faces;
  faces.reserve(3);
  // Lateral wall, outward radial normal.
  faces.push_back(detail::cylinderWall(axis, c0, c1, r, lo, hi, /*outward=*/true));
  // Two disc caps: lo cap outward −axis, hi cap outward +axis. Each is a full circle.
  {
    math::Dir3 fx, fy, fz;
    axisFrameDirs(axis, fx, fy, fz);
    const math::Ax3 loFrame{axisWorldPoint(axis, 0, 0, lo), fx,
                            math::Dir3{math::cross(fz.reversed().vec(), fx.vec())}, fz.reversed()};
    const math::Ax3 hiFrame{axisWorldPoint(axis, 0, 0, hi), fx,
                            math::Dir3{math::cross(fz.vec(), fx.vec())}, fz};
    faces.push_back(detail::planarCapFace(axis, lo, /*outwardPositive=*/false,
                                          detail::circleWire(axis, c0, c1, r, lo, loFrame), {}));
    faces.push_back(detail::planarCapFace(axis, hi, /*outwardPositive=*/true,
                                          detail::circleWire(axis, c0, c1, r, hi, hiFrame), {}));
  }
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// buildCutHole — box − cylinder = the box with a round THROUGH hole, built directly
// in world coordinates: four box side walls, two annular caps (rectangle − circle),
// and one INWARD-facing cylinder wall spanning the box axial depth. The hole wall's
// two rim circles are shared with the two annular caps → watertight. Volume =
// boxVol − πr²·boxAxialDepth.
//
// systems-band (~15 — a per-region face assembler); isolated + documented.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape buildCutHole(const AABox& box, const AxisCylinder& cyl) {
  const int axis = cyl.axis;
  const auto perp = cyl.perpAxes();
  const double lo = box.lo[axis], hi = box.hi[axis];
  const double u0 = box.lo[perp[0]], u1 = box.hi[perp[0]];
  const double v0 = box.lo[perp[1]], v1 = box.hi[perp[1]];
  const double c0 = cyl.c0, c1 = cyl.c1, r = cyl.radius;

  // Shared box-corner vertex rings at lo and hi axial coords (CCW in (u,v)).
  const double uu[4] = {u0, u1, u1, u0};
  const double vv[4] = {v0, v0, v1, v1};
  std::array<topo::Shape, 4> ringLo, ringHi;
  for (int i = 0; i < 4; ++i) {
    ringLo[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], lo));
    ringHi[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], hi));
  }

  std::vector<topo::Shape> faces;
  faces.reserve(7);

  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);

  // Two annular caps (rectangle outer wire CCW-as-seen-outward + circular hole).
  auto capFrameAt = [&](double t, bool outPos) {
    const math::Dir3 nz = outPos ? fz : fz.reversed();
    return math::Ax3{axisWorldPoint(axis, 0, 0, t), fx,
                     math::Dir3{math::cross(nz.vec(), fx.vec())}, nz};
  };
  auto rectWire = [&](const std::array<topo::Shape, 4>& ring, const math::Ax3& frame) {
    const std::array<topo::Shape, 4> o = detail::ccwRectOrder(ring, frame);
    std::vector<topo::Shape> edges;
    for (int i = 0; i < 4; ++i) edges.push_back(detail::capEdge(o[i], o[(i + 1) % 4], frame));
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };
  {
    const math::Ax3 loFrame = capFrameAt(lo, /*outPos=*/false);
    const math::Ax3 hiFrame = capFrameAt(hi, /*outPos=*/true);
    faces.push_back(detail::planarCapFace(
        axis, lo, false, rectWire(ringLo, loFrame),
        {detail::circleWire(axis, c0, c1, r, lo, loFrame)}));
    faces.push_back(detail::planarCapFace(
        axis, hi, true, rectWire(ringHi, hiFrame),
        {detail::circleWire(axis, c0, c1, r, hi, hiFrame)}));
  }

  // Four outer box side walls (plane quads) sharing the ring vertices.
  const math::Point3 boxCentre{0.5 * (box.lo.x + box.hi.x), 0.5 * (box.lo.y + box.hi.y),
                               0.5 * (box.lo.z + box.hi.z)};
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    faces.push_back(detail::boxSideWall(ringLo[i], ringLo[j], ringHi[j], ringHi[i], boxCentre));
  }

  // Inward-facing cylinder hole wall spanning [lo,hi].
  faces.push_back(detail::cylinderWall(axis, c0, c1, r, lo, hi, /*outward=*/false));

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// buildBlindHole — box − cylinder = the box with a round BLIND pocket (a flat-
// bottomed hole that does NOT break through the far face), built directly in world
// coordinates. The cylinder enters through the box's hi[axis] cap and its far cap
// (cyl.lo) sits STRICTLY inside the box (box.lo[axis] < cyl.lo < box.hi[axis]); the
// near cap is at or beyond the box's hi face (a through-the-opening pocket). Faces:
//   * lo box cap (plain rectangle, outward −axis)
//   * hi box cap (rectangle with the pocket circle as a hole = annulus, outward +axis)
//   * four box side walls
//   * pocket cylinder wall (bottom → hi[axis]), INWARD-facing (into the material)
//   * pocket flat-bottom DISC at the bottom, normal +axis (facing up into the cavity)
// The opening circle at hi[axis] is shared by the annulus hole and the pocket wall
// top rim; the bottom circle is shared by the wall and the disc → watertight.
// Volume = boxVol − πr²·depth, where depth = hi[axis] − bottom. The flat disk bottom
// is the analytic signature that distinguishes a blind pocket from a through hole.
//
// systems-band (~16 — a per-region assembler mirroring buildCutHole); isolated + documented.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape buildBlindHole(const AABox& box, const AxisCylinder& cyl) {
  const int axis = cyl.axis;
  const auto perp = cyl.perpAxes();
  const double lo = box.lo[axis], hi = box.hi[axis];
  const double bottom = cyl.lo;  // interior far cap of the cylinder → pocket bottom
  if (!(bottom - lo > kAxisEps)) return {};   // bottom not strictly inside → not blind
  if (!(hi - bottom > kAxisEps)) return {};   // degenerate depth
  const double u0 = box.lo[perp[0]], u1 = box.hi[perp[0]];
  const double v0 = box.lo[perp[1]], v1 = box.hi[perp[1]];
  const double c0 = cyl.c0, c1 = cyl.c1, r = cyl.radius;

  const double uu[4] = {u0, u1, u1, u0};
  const double vv[4] = {v0, v0, v1, v1};
  std::array<topo::Shape, 4> ringLo, ringHi;
  for (int i = 0; i < 4; ++i) {
    ringLo[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], lo));
    ringHi[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], hi));
  }

  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);
  auto capFrameAt = [&](double t, bool outPos) {
    const math::Dir3 nz = outPos ? fz : fz.reversed();
    return math::Ax3{axisWorldPoint(axis, 0, 0, t), fx,
                     math::Dir3{math::cross(nz.vec(), fx.vec())}, nz};
  };
  auto rectWire = [&](const std::array<topo::Shape, 4>& ring, const math::Ax3& frame) {
    const std::array<topo::Shape, 4> o = detail::ccwRectOrder(ring, frame);
    std::vector<topo::Shape> edges;
    for (int i = 0; i < 4; ++i) edges.push_back(detail::capEdge(o[i], o[(i + 1) % 4], frame));
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };

  std::vector<topo::Shape> faces;
  faces.reserve(8);

  // lo box cap: plain rectangle, outward −axis.
  {
    const math::Ax3 loFrame = capFrameAt(lo, false);
    faces.push_back(detail::planarCapFace(axis, lo, false, rectWire(ringLo, loFrame), {}));
  }
  // hi box cap: rectangle with the pocket circle as a hole (annulus), outward +axis.
  {
    const math::Ax3 hiFrame = capFrameAt(hi, true);
    faces.push_back(detail::planarCapFace(axis, hi, true, rectWire(ringHi, hiFrame),
                                          {detail::circleWire(axis, c0, c1, r, hi, hiFrame)}));
  }
  // Four box side walls (robust outward normal + winding).
  const math::Point3 boxCentre{0.5 * (box.lo.x + box.hi.x), 0.5 * (box.lo.y + box.hi.y),
                               0.5 * (box.lo.z + box.hi.z)};
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    faces.push_back(detail::boxSideWall(ringLo[i], ringLo[j], ringHi[j], ringHi[i], boxCentre));
  }
  // Pocket cylinder wall (bottom → hi), INWARD-facing (the cavity wall).
  faces.push_back(detail::cylinderWall(axis, c0, c1, r, bottom, hi, /*outward=*/false));
  // Flat-bottom disc at `bottom`: a full circle whose outward normal points +axis
  // (up into the cavity — the material sits below it).
  {
    const math::Ax3 botFrame = capFrameAt(bottom, /*outPos=*/true);
    faces.push_back(detail::planarCapFace(axis, bottom, /*outwardPositive=*/true,
                                          detail::circleWire(axis, c0, c1, r, bottom, botFrame), {}));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// buildFuseBoss — box ∪ cylinder = a round BOSS standing proud of the box's hi[axis]
// cap, built directly in world coordinates. Narrow topology: the boss base is inside/
// flush with the box (box.lo[axis] ≤ cyl.lo) and it protrudes past the hi[axis] face
// (cyl.hi > box.hi[axis]). Faces:
//   * lo box cap (plain rectangle, outward −axis)
//   * hi box cap (rectangle with the boss circle as a hole = annulus, outward +axis)
//   * four box side walls
//   * boss cylinder wall (hi[axis] → cyl.hi, outward radial)
//   * boss top disc (full circle at cyl.hi, outward +axis)
// The interface circle at hi[axis] is shared by the annulus hole and the boss wall
// base → watertight. Volume = boxVol + πr²·(cyl.hi − box.hi[axis]).
//
// systems-band (~16 — a per-region assembler); isolated + documented.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape buildFuseBoss(const AABox& box, const AxisCylinder& cyl) {
  const int axis = cyl.axis;
  const auto perp = cyl.perpAxes();
  const double lo = box.lo[axis], hi = box.hi[axis];
  const double top = cyl.hi;  // boss top axial coord
  if (!(top - hi > kAxisEps)) return {};             // must protrude past the hi face
  if (cyl.lo < lo - kAxisEps) return {};             // base breaches the lo face → defer
  const double u0 = box.lo[perp[0]], u1 = box.hi[perp[0]];
  const double v0 = box.lo[perp[1]], v1 = box.hi[perp[1]];
  const double c0 = cyl.c0, c1 = cyl.c1, r = cyl.radius;

  const double uu[4] = {u0, u1, u1, u0};
  const double vv[4] = {v0, v0, v1, v1};
  std::array<topo::Shape, 4> ringLo, ringHi;
  for (int i = 0; i < 4; ++i) {
    ringLo[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], lo));
    ringHi[i] = topo::ShapeBuilder::makeVertex(axisWorldPoint(axis, uu[i], vv[i], hi));
  }

  math::Dir3 fx, fy, fz;
  axisFrameDirs(axis, fx, fy, fz);
  auto capFrameAt = [&](double t, bool outPos) {
    const math::Dir3 nz = outPos ? fz : fz.reversed();
    return math::Ax3{axisWorldPoint(axis, 0, 0, t), fx,
                     math::Dir3{math::cross(nz.vec(), fx.vec())}, nz};
  };
  auto rectWire = [&](const std::array<topo::Shape, 4>& ring, const math::Ax3& frame) {
    const std::array<topo::Shape, 4> o = detail::ccwRectOrder(ring, frame);
    std::vector<topo::Shape> edges;
    for (int i = 0; i < 4; ++i) edges.push_back(detail::capEdge(o[i], o[(i + 1) % 4], frame));
    return topo::ShapeBuilder::makeWire(std::move(edges));
  };

  std::vector<topo::Shape> faces;
  faces.reserve(8);

  // lo box cap: plain rectangle, outward −axis.
  {
    const math::Ax3 loFrame = capFrameAt(lo, false);
    faces.push_back(detail::planarCapFace(axis, lo, false, rectWire(ringLo, loFrame), {}));
  }
  // hi box cap: rectangle with the boss circle as a hole, outward +axis.
  {
    const math::Ax3 hiFrame = capFrameAt(hi, true);
    faces.push_back(detail::planarCapFace(axis, hi, true, rectWire(ringHi, hiFrame),
                                          {detail::circleWire(axis, c0, c1, r, hi, hiFrame)}));
  }
  // Four box side walls (robust outward normal + winding).
  const math::Point3 boxCentre{0.5 * (box.lo.x + box.hi.x), 0.5 * (box.lo.y + box.hi.y),
                               0.5 * (box.lo.z + box.hi.z)};
  for (int i = 0; i < 4; ++i) {
    const int j = (i + 1) % 4;
    faces.push_back(detail::boxSideWall(ringLo[i], ringLo[j], ringHi[j], ringHi[i], boxCentre));
  }
  // Boss cylinder wall (hi → top), outward.
  faces.push_back(detail::cylinderWall(axis, c0, c1, r, hi, top, /*outward=*/true));
  // Boss top disc, outward +axis.
  {
    const math::Ax3 topFrame = capFrameAt(top, true);
    faces.push_back(detail::planarCapFace(axis, top, true,
                                          detail::circleWire(axis, c0, c1, r, top, topFrame), {}));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// Geometric preconditions for the analytic family.
// ─────────────────────────────────────────────────────────────────────────────

/// The cylinder sits RADIALLY INSIDE the box cross-section (the round feature does
/// not breach a side wall): its circle, centred at (c0,c1) with radius r, is fully
/// within the box's two perpendicular extents. This keeps the lateral surface off
/// the four parallel box faces, so the only plane-cylinder intersections are the two
/// analytic cap CIRCLES — the honest domain of this slice.
inline bool radiallyInside(const AABox& box, const AxisCylinder& cyl) {
  const auto perp = cyl.perpAxes();
  const double c[2] = {cyl.c0, cyl.c1};
  for (int i = 0; i < 2; ++i) {
    const int a = perp[i];
    if (c[i] - cyl.radius < box.lo[a] - kAxisEps) return false;
    if (c[i] + cyl.radius > box.hi[a] + kAxisEps) return false;
  }
  return true;
}

/// The cylinder spans THROUGH the box along its axis (both caps at or beyond the
/// box's axial faces): a true through feature.
inline bool spansThrough(const AABox& box, const AxisCylinder& cyl) {
  const int a = cyl.axis;
  return cyl.lo <= box.lo[a] + kAxisEps && cyl.hi >= box.hi[a] - kAxisEps;
}

/// The cylinder makes a BLIND pocket entering the box's hi[axis] face: the near cap
/// is at or beyond the hi face, the far cap (cyl.lo) sits STRICTLY inside the box, and
/// the near end does NOT also break the lo face (that would be a through hole). The
/// pocket therefore has a flat disk bottom at cyl.lo. Only this one orientation
/// (entering through hi) is native; a pocket entering the lo face, or a fully-interior
/// cylinder (both caps inside → an internal void), is deferred to OCCT.
inline bool blindThroughHi(const AABox& box, const AxisCylinder& cyl) {
  const int a = cyl.axis;
  return cyl.hi >= box.hi[a] - kAxisEps &&               // opens through the hi face
         cyl.lo > box.lo[a] + kAxisEps &&                // far cap strictly inside
         cyl.lo < box.hi[a] - kAxisEps;                  // …and below the hi face
}

// ─────────────────────────────────────────────────────────────────────────────
// tryBoxCylinder — the ANALYTIC dispatcher. Recognise (box, cylinder) in either
// operand order, check the narrow-family preconditions for the requested op, and
// build the closed-form result. Returns a NULL Shape for anything outside the
// family (→ the caller falls back to the planar path / OCCT). The engine's
// self-verify still guards the result.
//
//   op 0 fuse   — box ∪ cyl = a boss protruding one box cap (buildFuseBoss).
//   op 1 cut    — a − b: NATIVE only when a=box, b=cyl. A THROUGH feature → a round
//                 through hole (buildCutHole); a cylinder entering the hi cap with its
//                 far cap strictly inside → a flat-bottomed BLIND pocket
//                 (buildBlindHole). (cyl − box, or a fully-interior cylinder, is a
//                 different / non-manifold solid; deferred to OCCT.)
//   op 2 common — box ∩ cyl = the cylinder segment clipped to the box (buildCommon).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape tryBoxCylinder(const topo::Shape& a, const topo::Shape& b, int op) {
  const std::optional<AABox> aBox = recogniseBox(a);
  const std::optional<AABox> bBox = recogniseBox(b);
  const std::optional<AxisCylinder> aCyl = aBox ? std::nullopt : recogniseCylinder(a);
  const std::optional<AxisCylinder> bCyl = bBox ? std::nullopt : recogniseCylinder(b);

  // Exactly one box and one axis-aligned cylinder.
  const bool aIsBox = aBox.has_value();
  const bool bIsBox = bBox.has_value();
  const bool aIsCyl = aCyl.has_value();
  const bool bIsCyl = bCyl.has_value();
  if (!((aIsBox && bIsCyl) || (aIsCyl && bIsBox))) return {};

  const AABox box = aIsBox ? *aBox : *bBox;
  const AxisCylinder cyl = aIsCyl ? *aCyl : *bCyl;

  if (!radiallyInside(box, cyl)) return {};  // side-wall breach → OCCT

  switch (op) {
    case 0:  // fuse (commutative)
      return buildFuseBoss(box, cyl);
    case 1:  // cut a − b: round hole only when a is the box (cyl − box is deferred).
      if (!aIsBox) return {};              // cyl − box: deferred
      if (spansThrough(box, cyl)) return buildCutHole(box, cyl);   // through hole
      if (blindThroughHi(box, cyl)) return buildBlindHole(box, cyl);  // blind pocket
      return {};                           // fully-interior / lo-entering: deferred
    case 2:  // common (commutative)
      return buildCommonSegment(box, cyl);
    default:
      return {};
  }
}

}  // namespace cybercad::native::boolean::curved

#endif  // CYBERCAD_NATIVE_BOOLEAN_CURVED_H

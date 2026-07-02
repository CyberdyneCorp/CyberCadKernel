// SPDX-License-Identifier: Apache-2.0
//
// construct.h — native swept-solid construction (Phase 4, capability #4
// `native-construction`). Clean-room, OCCT-FREE builders that assemble a native
// B-rep (src/native/topology) with native-math geometry (src/native/math) for the
// two solid operations the native engine implements today:
//
//   * build_prism(profileXY, count, depth)  — extrude a CLOSED planar polygon
//     profile (x,y pairs on z=0) by `depth` along +Z into a prism SOLID: one
//     bottom planar face (z=0), one top planar face (z=depth), and ONE planar
//     quad side face per profile edge. Full native topology + geometry.
//
//   * build_revolution(segments, axis, angle) — revolve a LINE-SEGMENT profile
//     about an axis in the profile plane (z=0). Each segment sweeps into a
//     surface of revolution: an axis-parallel segment → cylinder, a slanted
//     segment → cone, a segment perpendicular to the axis (constant axial coord)
//     → planar annulus/disk. A full-turn (angle ≥ 2π) closes into a solid; a
//     partial turn additionally caps the two open ends with planar profile faces.
//
// SCOPE (honest — see openspec/NATIVE-REWRITE.md). The native path handles the
// two cases above. EXPLICITLY DEFERRED to OCCT (the engine falls through, it does
// NOT fake them): loft, sweep, twisted/guided sweep, threads, holed / typed-
// profile extrude variants, and revolve of ARC / SPLINE profiles. The builders
// below signal "not handled natively" by returning a NULL Shape so the engine can
// fall through cleanly.
//
// REFERENCE ORACLE ONLY: OCCT BRepPrimAPI_MakePrism / BRepPrimAPI_MakeRevol were
// consulted to confirm face decomposition and orientation conventions; nothing is
// copied. Surface/curve parametrizations match src/native/math (ElSLib-aligned) so
// the native tessellator and a future OCCT parity gate agree.
//
// This header is the AGREED API between the construction work and the engine glue
// (src/engine/native): the engine calls these free functions and wraps the
// returned topology::Shape into an EngineShape. Keep the signatures stable.
// Consumers should include the aggregate header `native_construct.h` (which pulls
// in this file), mirroring native_math.h / native_topology.h / native_tessellate.h.
//
// TESSELLATOR CAP-FILL (RESOLVED): the native tessellator's `isFullRectangle`
// fast-path used to treat a planar face whose every boundary vertex lies on its UV
// bounding box as an untrimmed rectangle and fill the whole box. A CONVEX polygon
// cap (triangle, hexagon, …) has every vertex on the bbox and tripped this, so it
// meshed as the bbox quad rather than the polygon — such a prism did not tessellate
// watertight. FIXED in trim.h: isFullRectangle now ALSO requires the boundary loop
// to visit ALL FOUR bbox corners (a triangle reaches at most three), so only a
// genuine full parametric rectangle takes the structured-grid fast path; any
// convex/concave polygon cap is ear-clipped and welds WATERTIGHT. Native extrude of
// ANY simple polygon profile now meshes watertight with the exact profile-area ×
// depth volume.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_H
#define CYBERCAD_NATIVE_CONSTRUCT_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::construct {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

// ─────────────────────────────────────────────────────────────────────────────
// Profile input (mirrors the cc_* line-segment profile the facade forwards).
//
// A LineSeg is a straight segment (x0,y0)→(x1,y1) in the profile plane (z=0). The
// revolve builder consumes a contiguous run of these; the extrude builder takes a
// raw x,y point loop (the closed polygon vertices in order).
// ─────────────────────────────────────────────────────────────────────────────
struct LineSeg {
  double x0 = 0.0, y0 = 0.0;
  double x1 = 0.0, y1 = 0.0;
};

// Revolution axis: a point (ax,ay) on z=0 and a direction (adx,ady) in the plane.
struct RevolveAxis {
  double ax = 0.0, ay = 0.0;
  double adx = 0.0, ady = 1.0;
};

// Tolerances local to construction (looser than the raw fp tolerance so a
// hand-authored profile with mm-scale features is judged degenerate sensibly).
inline constexpr double kProfileTol = 1e-7;    // coincident-point / zero-length
inline constexpr double kMinDepth = 1e-6;      // minimum prism height / sweep angle
inline constexpr double kFullTurn = 2.0 * 3.14159265358979323846;

// ─────────────────────────────────────────────────────────────────────────────
// Internal geometry helpers (planar builders).
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

inline math::Point3 xy(double x, double y, double z = 0.0) noexcept {
  return math::Point3{x, y, z};
}

// The straight-edge length (0 clamps to a tiny positive so the [first,last] range
// is non-degenerate).
inline double edgeLen(const math::Point3& p0, const math::Point3& p1) noexcept {
  return std::max(math::distance(p0, p1), kProfileTol);
}

// A straight edge between two vertices, with a Line EdgeCurve parametrized by arc
// length (first = 0, last = |p1 − p0|). v0 stored Forward, v1 Reversed. The native
// tessellator only reads the edge's endpoints + range for a line, so the frame's X
// carries the 3D direction.
inline topo::Shape lineEdge(const topo::Shape& v0, const topo::Shape& v1) {
  const auto p0 = topo::pointOf(v0);
  const auto p1 = topo::pointOf(v1);
  const math::Vec3 d = *p1 - *p0;
  const double len = edgeLen(*p0, *p1);
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Line;
  c.frame.origin = *p0;
  c.frame.x = math::norm(d) > kProfileTol ? math::Dir3{d} : math::Dir3{1, 0, 0};
  c.frame.z = math::Dir3{0, 0, 1};
  return topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);
}

// Build a straight edge AND attach its Line pcurve on the given planar frame, so
// the tessellator can flatten the face's wire to UV and trim/mesh it. The pcurve
// is a Line origin2d = uv(p0), dir2d = (uv(p1)−uv(p0))/len, so at edge parameter
// t∈[0,len] the pcurve traces uv(p0)→uv(p1) (matching pcurveValue's origin2d +
// dir2d·t convention). Because each face builds its OWN edges, an edge carries
// exactly one pcurve and pcurveForFace resolves it by the single-pcurve fallback.
inline topo::Shape planarEdge(const topo::Shape& v0, const topo::Shape& v1, const math::Ax3& frame) {
  const topo::Shape edge = lineEdge(v0, v1);
  const auto p0 = topo::pointOf(v0);
  const auto p1 = topo::pointOf(v1);
  const double len = edgeLen(*p0, *p1);
  auto toUV = [&](const math::Point3& p) -> math::Point3 {
    const math::Vec3 d = p - frame.origin;
    return math::Point3{math::dot(d, frame.x.vec()), math::dot(d, frame.y.vec()), 0.0};
  };
  const math::Point3 uv0 = toUV(*p0), uv1 = toUV(*p1);
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Line;
  pc.origin2d = uv0;
  pc.dir2d = (uv1 - uv0) / len;  // per-parameter step so uv0 + dir2d·len = uv1
  // The pcurve's face-node key is irrelevant here (single pcurve → fallback picks
  // it); use the edge's own node so the reference is non-null and valid.
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// A planar face from an ordered, closed loop of vertices (outer wire only). The
// plane's frame is (O = loop[0], Z = normal, X = a chosen in-plane axis, Y = Z×X);
// every boundary edge carries its Line pcurve on that frame. Orientation `orient`
// flips the material side.
inline topo::Shape planarFace(const std::vector<topo::Shape>& loop, const math::Dir3& normal,
                              topo::Orientation orient) {
  const auto o = topo::pointOf(loop[0]);
  // Pick an in-plane X reference not parallel to the normal.
  const math::Vec3 ref = std::fabs(normal.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  const math::Ax3 frame = math::Ax3::fromAxisAndRef(*o, normal, math::Dir3{ref});

  std::vector<topo::Shape> edges;
  edges.reserve(loop.size());
  const std::size_t n = loop.size();
  for (std::size_t i = 0; i < n; ++i)
    edges.push_back(planarEdge(loop[i], loop[(i + 1) % n], frame));
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));

  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, wire, {}, orient);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_prism — extrude a closed polygon profile by depth along +Z.
//
// Faces (all planar):
//   * bottom  (z = 0)      — normal −Z (outward for a +Z prism), so the loop is
//                            taken in reverse to point the material downward.
//   * top     (z = depth)  — normal +Z.
//   * side[i] (quad)       — profile edge i extruded: (b_i, b_{i+1}, t_{i+1}, t_i)
//                            with an outward normal = edge_dir × Zup for a CCW
//                            profile.
//
// The bottom/top rings SHARE vertices with the side quads (same TShape nodes), so
// the mesher welds them and the solid is watertight. Returns a NULL Shape if the
// profile is degenerate (< 3 points, zero-area, or depth ≤ 0) so the engine falls
// through to OCCT.
//
// Cognitive complexity: a linear assembler (~9), flagged.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism(const double* profileXY, int pointCount, double depth) {
  if (profileXY == nullptr || pointCount < 3 || !(depth > kMinDepth)) return {};

  // Deduplicate a repeated closing vertex (profile[n-1] == profile[0]).
  int n = pointCount;
  if (std::fabs(profileXY[0] - profileXY[(n - 1) * 2]) < kProfileTol &&
      std::fabs(profileXY[1] - profileXY[(n - 1) * 2 + 1]) < kProfileTol) {
    --n;
  }
  if (n < 3) return {};

  // Signed area (shoelace) → reject a zero-area profile and detect winding so the
  // side-quad outward normal is chosen consistently.
  double area2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    area2 += profileXY[i * 2] * profileXY[j * 2 + 1] - profileXY[j * 2] * profileXY[i * 2 + 1];
  }
  if (std::fabs(area2) < kProfileTol) return {};
  const bool ccw = area2 > 0.0;

  // Shared bottom/top vertex rings.
  std::vector<topo::Shape> bottom(static_cast<std::size_t>(n)), top(static_cast<std::size_t>(n));
  for (int i = 0; i < n; ++i) {
    bottom[i] = topo::ShapeBuilder::makeVertex(detail::xy(profileXY[i * 2], profileXY[i * 2 + 1], 0.0));
    top[i] = topo::ShapeBuilder::makeVertex(detail::xy(profileXY[i * 2], profileXY[i * 2 + 1], depth));
  }

  std::vector<topo::Shape> faces;
  faces.reserve(static_cast<std::size_t>(n) + 2);

  // Bottom face: outward normal −Z. For a CCW loop the bottom's material-outward
  // face uses the loop reversed (so its own CCW winding faces −Z).
  {
    std::vector<topo::Shape> loop = bottom;
    if (ccw) std::reverse(loop.begin(), loop.end());
    faces.push_back(detail::planarFace(loop, math::Dir3{0, 0, -1}, topo::Orientation::Forward));
  }
  // Top face: outward normal +Z, loop in profile order for CCW.
  {
    std::vector<topo::Shape> loop = top;
    if (!ccw) std::reverse(loop.begin(), loop.end());
    faces.push_back(detail::planarFace(loop, math::Dir3{0, 0, 1}, topo::Orientation::Forward));
  }
  // One quad side face per edge.
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    // Outward side normal for a CCW profile is edgeDir × Zup; flip for CW.
    const math::Vec3 e = *topo::pointOf(bottom[j]) - *topo::pointOf(bottom[i]);
    math::Vec3 nrm = math::cross(e, math::Vec3{0, 0, 1});
    if (!ccw) nrm = -nrm;
    // Quad wound so its CCW winding matches the outward normal: b_i → b_j → t_j → t_i
    // for CCW (right-hand rule with +Z up gives edgeDir × down = outward). Choose
    // the vertex order that keeps the loop CCW as seen from outside.
    std::vector<topo::Shape> loop =
        ccw ? std::vector<topo::Shape>{bottom[i], bottom[j], top[j], top[i]}
            : std::vector<topo::Shape>{bottom[j], bottom[i], top[i], top[j]};
    faces.push_back(detail::planarFace(loop, math::Dir3{nrm}, topo::Orientation::Forward));
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// build_revolution — revolve a LINE-SEGMENT profile about an in-plane axis.
//
// For each segment, the two endpoints are described by (r, h): r = signed
// distance from the axis (perpendicular), h = coordinate ALONG the axis. Revolving
// the segment yields:
//   * both r equal (axis-parallel segment)   → CYLINDER of that radius over [h0,h1]
//   * r differ, h differ (slanted segment)   → CONE (frustum)
//   * h equal (perpendicular segment)         → PLANAR annulus/disk at that h
//   * a segment ON the axis (both r ≈ 0)      → degenerate, skipped
//
// A full turn (angle ≥ 2π − tol) produces a closed body of the side faces (plus
// planar caps where a profile end sits on the axis form the closure implicitly).
// A partial turn additionally adds the two planar "start"/"end" profile faces that
// cap the swept opening.
//
// SCOPE: only kind-0 (line) segments are handled natively; ANY arc/spline segment
// makes the builder return NULL so the engine falls through to OCCT. Full native
// topology + analytic surfaces (Plane / Cylinder / Cone) are produced.
//
// Cognitive complexity: the per-segment face dispatch is a documented systems-band
// helper (~14); the driver stays ≤ ~10.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

// Axis frame: origin on the axis, Z = axis direction, X = the in-plane radial
// reference (perpendicular to the axis, in z=0), Y = Z × X (the +90° sweep dir).
struct AxisFrame {
  math::Point3 origin;
  math::Dir3 z;   // along the axis
  math::Dir3 x;   // radial reference (θ = 0)
  math::Dir3 y;   // Z × X

  // (r, h) of a profile point: h along the axis, r perpendicular distance.
  void decompose(const math::Point3& p, double& r, double& h) const noexcept {
    const math::Vec3 d = p - origin;
    h = math::dot(d, z.vec());
    const math::Vec3 radial = d - z.vec() * h;
    r = math::norm(radial);
  }
};

// Point on the surface of revolution at radius r, axial coord h, angle θ.
inline math::Point3 revolved(const AxisFrame& f, double r, double h, double theta) noexcept {
  return f.origin + f.z.vec() * h + f.x.vec() * (r * std::cos(theta)) +
         f.y.vec() * (r * std::sin(theta));
}


// A revolution profile point: perpendicular radius r from the axis and axial
// coord h along it. `onAxis` marks a point on the axis (a cone tip / disk center),
// whose angular copies all collapse to one shared apex vertex.
struct RevPoint {
  double r = 0.0;
  double h = 0.0;
  bool onAxis = false;
};

// v (the non-angular surface parameter) of a revolved point at radius r, axial h:
//   Cylinder: v = h ;  Cone: v = slant = h/cosα ;  Plane annulus: v = r (radius).
inline double surfV(topo::FaceSurface::Kind kind, double r, double h, double semiAngle) noexcept {
  using K = topo::FaceSurface::Kind;
  if (kind == K::Cylinder) return h;
  if (kind == K::Cone) {
    const double ca = std::cos(semiAngle);
    return std::fabs(ca) > kProfileTol ? h / ca : h;
  }
  return r;  // Plane annulus: radial distance is the "v".
}

// A straight (Line) 3D edge between two revolved vertices carrying a Line pcurve on
// the surface. On a cylinder/cone this is an iso-θ line (the profile edge): the
// pcurve runs (u=θ const, v: v0→v1). On a plane annulus it is a radial line at a
// fixed angle θ: the pcurve runs (u=r·cosθ, v=r·sinθ) from (v0) to (v1) — still a
// straight UV line. So a Line pcurve origin2d=uv0, dir2d=(uv1−uv0)/len is exact.
inline topo::Shape profileEdge(const topo::Shape& v0, const topo::Shape& v1,
                               const math::Point3& uv0, const math::Point3& uv1) {
  const topo::Shape edge = lineEdge(v0, v1);
  const double len = edgeLen(*topo::pointOf(v0), *topo::pointOf(v1));
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Line;
  pc.origin2d = uv0;
  pc.dir2d = (uv1 - uv0) / len;
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// A circumferential ARC edge at fixed profile point (r,h), sweeping θ: t0→t1. Its
// 3D curve is a Circle (center on the axis at height h, radius r) so the shared
// EdgeCache subdivides it by curvature — the piece that makes the rim WELD across
// the two faces meeting there (both represent the rim as the SAME Circle arc, so
// discretize() yields identical fractions → identical 3D points). The pcurve is a
// Line (u=θ, v const) on a cylinder/cone, or a Circle (radius v) on a plane annulus.
inline topo::Shape arcEdge(const AxisFrame& f, const topo::Shape& v0, const topo::Shape& v1,
                           double r, double h, double t0, double t1,
                           topo::FaceSurface::Kind kind, double vparam) {
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = math::Ax3{f.origin + f.z.vec() * h, f.x, f.y, f.z};
  c.radius = r;
  const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, t0, t1, v0, v1);

  topo::PCurve pc;
  if (kind == topo::FaceSurface::Kind::Plane) {
    // (u,v) = (r·cosθ, r·sinθ): a circle of radius `vparam`(=r) about the UV origin.
    pc.kind = topo::EdgeCurve::Kind::Circle;
    pc.origin2d = math::Point3{0, 0, 0};
    pc.dir2d = math::Vec3{vparam, 0, 0};  // dir2d.x carries the pcurve radius
  } else {
    // (u=θ, v=const): pcurveValue Line = origin2d + dir2d·t, t = arc angle θ.
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = math::Point3{0.0, vparam, 0.0};
    pc.dir2d = math::Vec3{1.0, 0.0, 0.0};  // u = 1·θ
  }
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// The surface's natural normal at the segment midpoint (world), from the analytic
// parametrization in elementary.h. Cylinder/cone point away from the axis; plane
// points along +axis. Used to decide the face Orientation so its EFFECTIVE normal
// (flipped when Reversed) points OUT of the material.
inline math::Vec3 naturalNormalAt(const topo::FaceSurface& surf, const RevPoint& p,
                                  const RevPoint& q, double theta) {
  using K = topo::FaceSurface::Kind;
  const double vmid = 0.5 * (surfV(surf.kind, p.r, p.h, surf.semiAngle) +
                             surfV(surf.kind, q.r, q.h, surf.semiAngle));
  switch (surf.kind) {
    case K::Cylinder: return math::Cylinder{surf.frame, surf.radius}.normal(theta, vmid).vec();
    case K::Cone: return math::Cone{surf.frame, surf.radius, surf.semiAngle}.normal(theta, vmid).vec();
    case K::Plane:
    default: return surf.frame.z.vec();
  }
}

// Orientation of a swept face so its effective normal points OUT of the material.
// The material-outward direction in the (r,h) half-plane is the profile edge
// direction rotated −90° for a CCW profile (windingSign > 0), i.e. (dh, −dr); this
// maps to 3D at angle θ. Reverse the face when the surface's natural normal opposes
// it. This gives a consistent outward shell — including the INNER wall of a tube
// (radius decreasing along the profile), whose outward normal points toward the axis.
inline topo::Orientation faceOrientation(const topo::FaceSurface& surf, const AxisFrame& f,
                                         const RevPoint& p, const RevPoint& q, double theta,
                                         double windingSign) {
  const double dr = q.r - p.r, dh = q.h - p.h;
  const double nr = windingSign * dh, nh = windingSign * (-dr);  // (r,h) material-outward
  const math::Vec3 outward =
      f.x.vec() * (nr * std::cos(theta)) + f.y.vec() * (nr * std::sin(theta)) + f.z.vec() * nh;
  const math::Vec3 nat = naturalNormalAt(surf, p, q, theta);
  return math::dot(nat, outward) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
}

// One revolution sub-patch (segment p→q over angular span [t0,t1]). Corners:
//   a0=(p,t0) a1=(q,t0) b1=(q,t1) b0=(p,t1). Boundary: profileEdge(a0→a1),
//   arcEdge(a1→b1 at q), profileEdge(b1→b0), arcEdge(b0→a0 at p). A profile end on
//   the axis collapses its two angular copies to one apex (its arc edge is omitted).
// `orient` sets the material-outward side (see faceOrientation).
inline topo::Shape sweptPatch(const topo::FaceSurface& surf, const AxisFrame& f, const RevPoint& p,
                              const RevPoint& q, double t0, double t1, topo::Orientation orient) {
  const double vp = surfV(surf.kind, p.r, p.h, surf.semiAngle);
  const double vq = surfV(surf.kind, q.r, q.h, surf.semiAngle);
  auto uv = [&](double v, double theta) -> math::Point3 {
    return surf.kind == topo::FaceSurface::Kind::Plane
               ? math::Point3{v * std::cos(theta), v * std::sin(theta), 0.0}
               : math::Point3{theta, v, 0.0};
  };

  const topo::Shape a0 = topo::ShapeBuilder::makeVertex(revolved(f, p.r, p.h, t0));
  const topo::Shape a1 = topo::ShapeBuilder::makeVertex(revolved(f, q.r, q.h, t0));
  const topo::Shape b0 = p.onAxis ? a0 : topo::ShapeBuilder::makeVertex(revolved(f, p.r, p.h, t1));
  const topo::Shape b1 = q.onAxis ? a1 : topo::ShapeBuilder::makeVertex(revolved(f, q.r, q.h, t1));

  std::vector<topo::Shape> edges;
  edges.push_back(profileEdge(a0, a1, uv(vp, t0), uv(vq, t0)));
  if (!q.onAxis) edges.push_back(arcEdge(f, a1, b1, q.r, q.h, t0, t1, surf.kind, vq));
  edges.push_back(profileEdge(b1, b0, uv(vq, t1), uv(vp, t1)));
  if (!p.onAxis) edges.push_back(arcEdge(f, b0, a0, p.r, p.h, t1, t0, surf.kind, vp));
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  return topo::ShapeBuilder::makeFace(surf, wire, {}, orient);
}

// Choose the analytic surface for the segment (RevPoint p → q). Cylinder when the
// radius is constant, plane when the axial coord is constant, else a cone frustum.
inline topo::FaceSurface segmentSurface(const AxisFrame& f, const RevPoint& p, const RevPoint& q) {
  topo::FaceSurface surf;
  if (std::fabs(q.h - p.h) < kProfileTol) {  // perpendicular → planar annulus/disk
    surf.kind = topo::FaceSurface::Kind::Plane;
    surf.frame = math::Ax3{f.origin + f.z.vec() * p.h, f.x, f.y, f.z};
  } else if (std::fabs(q.r - p.r) < kProfileTol) {  // axis-parallel → cylinder
    surf.kind = topo::FaceSurface::Kind::Cylinder;
    surf.frame = math::Ax3{f.origin, f.x, f.y, f.z};
    surf.radius = p.r;
  } else {  // slanted → cone
    const double dr = q.r - p.r, dh = q.h - p.h;
    surf.kind = topo::FaceSurface::Kind::Cone;
    surf.frame = math::Ax3{f.origin, f.x, f.y, f.z};
    surf.radius = p.r - (dr / dh) * p.h;  // reference radius at h = 0
    surf.semiAngle = std::atan2(dr, dh);
  }
  return surf;
}

}  // namespace detail

inline topo::Shape build_revolution(const std::vector<LineSeg>& segments, const RevolveAxis& axisIn,
                                    double angle) {
  if (segments.empty() || !(angle > kMinDepth)) return {};
  const math::Dir3 adir{axisIn.adx, axisIn.ady, 0.0};
  if (!adir.valid()) return {};

  detail::AxisFrame f;
  f.origin = detail::xy(axisIn.ax, axisIn.ay, 0.0);
  f.z = adir;
  // Radial reference X = in-plane perpendicular to the axis (rotate axis by −90°
  // in z=0), so a profile that lies on the +radial side maps to θ = 0.
  f.x = math::Dir3{math::Vec3{axisIn.ady, -axisIn.adx, 0.0}};
  f.y = math::Dir3{math::cross(f.z.vec(), f.x.vec())};

  const bool full = angle >= kFullTurn - 1e-9;
  const double sweep = full ? kFullTurn : angle;

  // Profile winding in the (r,h) half-plane (shoelace). The sign selects the
  // material-outward side of every swept face (see faceOrientation): a CCW profile
  // (sign > 0) puts material to the left of the directed boundary.
  double area2 = 0.0;
  for (const LineSeg& s : segments) {
    double r0, h0, r1, h1;
    f.decompose(detail::xy(s.x0, s.y0), r0, h0);
    f.decompose(detail::xy(s.x1, s.y1), r1, h1);
    area2 += r0 * h1 - r1 * h0;
  }
  const double windingSign = area2 >= 0.0 ? 1.0 : -1.0;

  // Angular stations: split the sweep into spans STRICTLY LESS THAN π (max 2π/3 =
  // 120°) so each swept patch is a simple non-periodic quad. A planar annulus
  // patch maps to a cartesian "pie-slice ring"; a span of exactly π makes its two
  // radial boundary rays antiparallel (a degenerate bowtie), and a full period
  // collapses its UV to a line — 120° spans avoid both. Adjacent patches share a
  // θ-edge and weld watertight (a full turn → 3 patches per segment).
  constexpr double kMaxSpan = kFullTurn / 3.0;  // 120°
  const int nSpan = std::max(1, static_cast<int>(std::ceil(sweep / kMaxSpan - 1e-9)));
  std::vector<double> stations(static_cast<std::size_t>(nSpan) + 1);
  for (int k = 0; k <= nSpan; ++k) stations[k] = sweep * k / nSpan;

  std::vector<topo::Shape> faces;
  faces.reserve(segments.size() * static_cast<std::size_t>(nSpan) + 2);

  // One or more swept sub-faces per profile segment.
  for (const LineSeg& s : segments) {
    detail::RevPoint p, q;
    f.decompose(detail::xy(s.x0, s.y0), p.r, p.h);
    f.decompose(detail::xy(s.x1, s.y1), q.r, q.h);
    p.onAxis = p.r < kProfileTol;
    q.onAxis = q.r < kProfileTol;
    if (p.onAxis && q.onAxis) continue;  // segment on the axis: no swept face

    const topo::FaceSurface surf = detail::segmentSurface(f, p, q);
    for (int k = 0; k < nSpan; ++k) {
      const double tMid = 0.5 * (stations[k] + stations[k + 1]);
      const topo::Orientation o = detail::faceOrientation(surf, f, p, q, tMid, windingSign);
      faces.push_back(detail::sweptPatch(surf, f, p, q, stations[k], stations[k + 1], o));
    }
  }

  if (faces.empty()) return {};

  // Partial turn: cap the two open ends with the planar profile face — the profile
  // loop swept to a single angular station (θ = 0 and θ = sweep). The cap lies in
  // the half-plane containing the axis at angle θ; its plane normal is perpendicular
  // to that half-plane, nθ = −sinθ·X + cosθ·Y. The θ=0 cap's material is on the +θ
  // side so its outward normal is −n₀ = −Y; the θ=sweep cap's outward is +n_sweep.
  // Coincident consecutive profile points (e.g. an on-axis pair) are de-duplicated
  // so the cap loop is a simple polygon.
  if (!full) {
    auto capAt = [&](double theta, const math::Vec3& outward) -> topo::Shape {
      std::vector<topo::Shape> verts;
      auto push = [&](double r, double h) {
        const math::Point3 p = detail::revolved(f, r, h, theta);
        if (!verts.empty() && math::distance(*topo::pointOf(verts.back()), p) < kProfileTol) return;
        verts.push_back(topo::ShapeBuilder::makeVertex(p));
      };
      for (const LineSeg& s : segments) {
        double r, h;
        f.decompose(detail::xy(s.x0, s.y0), r, h);
        push(r, h);
      }
      double r, h;
      f.decompose(detail::xy(segments.back().x1, segments.back().y1), r, h);
      push(r, h);
      // Drop a closing duplicate (loop end == start).
      if (verts.size() > 1 &&
          math::distance(*topo::pointOf(verts.front()), *topo::pointOf(verts.back())) < kProfileTol)
        verts.pop_back();
      if (verts.size() < 3) return {};
      return detail::planarFace(verts, math::Dir3{outward}, topo::Orientation::Forward);
    };
    const math::Vec3 n0 = f.y.vec();  // nθ at θ=0
    const math::Vec3 nS = f.x.vec() * (-std::sin(sweep)) + f.y.vec() * std::cos(sweep);
    const topo::Shape capStart = capAt(0.0, -n0);      // θ=0 cap: material on +θ side
    const topo::Shape capEnd = capAt(sweep, nS);       // θ=sweep cap: material on −θ side
    if (!capStart.isNull()) faces.push_back(capStart);
    if (!capEnd.isNull()) faces.push_back(capEnd);
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// Convenience overload: revolve a raw closed polygon (x,y pairs) as line segments.
// Used by the cc_solid_revolve facade path (a point loop, not typed segments).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_revolution(const double* profileXY, int pointCount,
                                     const RevolveAxis& axis, double angle) {
  if (profileXY == nullptr || pointCount < 2) return {};
  std::vector<LineSeg> segs;
  segs.reserve(static_cast<std::size_t>(pointCount));
  for (int i = 0; i < pointCount; ++i) {
    const int j = (i + 1) % pointCount;
    // Skip the implicit closing segment if the loop is already closed.
    if (i == pointCount - 1 && std::fabs(profileXY[0] - profileXY[i * 2]) < kProfileTol &&
        std::fabs(profileXY[1] - profileXY[i * 2 + 1]) < kProfileTol) {
      break;
    }
    segs.push_back(LineSeg{profileXY[i * 2], profileXY[i * 2 + 1], profileXY[j * 2],
                           profileXY[j * 2 + 1]});
  }
  return build_revolution(segs, axis, angle);
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_H

// SPDX-License-Identifier: Apache-2.0
//
// reference.h — native REFERENCE / DATUM geometry + topology-reference reads
//               (MOAT M-REF). OCCT-FREE, header-only.
//
// Datum and reference-geometry QUERIES computed directly from the native B-rep
// topology (src/native/topology). Every result is derived by exact fp64 vector
// math from the shared-node Shape graph; nothing is fabricated. Conventions
// mirror the OCCT oracle (gp_Ax1 / gp_Pln / gp_Lin / BRepTools::OuterWire /
// TopExp ancestry) so a native result can be verified against OCCT on the sim.
//
// Provided (each on a WORLD-PLACED sub-shape — the caller resolves the id and
// applies the shape's Location):
//   * faceAxis(face)          cylinder/cone axis         → [ox,oy,oz, dx,dy,dz]
//   * refPlaneFromFace(face)  planar-face datum plane     → [ox,oy,oz, nx,ny,nz]
//   * refAxisFromEdge(edge)   straight-edge axis          → [ox,oy,oz, dx,dy,dz]
//   * refAxisFromFace(face)   cyl/cone axis (== faceAxis) → [ox,oy,oz, dx,dy,dz]
//   * tangentChain(root, seeds)   connected tangent-continuous edge-id walk
//   * outerRimChain(root, seeds)  planar-cap outer-wire edge ids of the seeds
//   * offsetFaceBoundary(face, d) in-plane offset of a planar polygon boundary
//
// HONEST DECLINE (std::nullopt / documented empty): a non-planar face where a
// plane is required, a non-line edge where an axis line is required, a
// cylinder/cone axis asked of a face that has none, a freeform edge in a tangent
// walk (no closed-form tangent → deferred to the oracle), and an offset that is
// non-polygonal / would arc-round a convex corner / self-intersects. The service
// NEVER emits a wrong datum; the facade falls through to OCCT on a decline.
//
// Matches OCCT semantics on the parity-gated cells:
//   - faceAxis / refAxisFromFace : cylinder + cone ONLY (a plane/sphere/torus has
//     no single axis in cc_face_axis) — same guard as occt_query.cpp.
//   - refAxisFromEdge : LINE ONLY (a circular edge yields no gp_Lin in
//     occt_reference_geometry.cpp) — a circular edge is an honest decline.
//   - tangentChain : |t1·t2| ≥ cos(15°) = 0.966 at a shared vertex (occt_query.cpp).
//   - outerRimChain : planar cap faces whose plane contains ALL seed vertices
//     within 1.0 model unit; adds each cap's OUTER wire (occt_query.cpp).
//
// clang++ -std=c++20. Consumes src/native/{math,topology} read-only.
//
#ifndef CYBERCAD_NATIVE_REFERENCE_REFERENCE_H
#define CYBERCAD_NATIVE_REFERENCE_REFERENCE_H

#include "native/topology/accessors.h"
#include "native/topology/explore.h"
#include "native/topology/shape.h"

#include <array>
#include <cmath>
#include <optional>
#include <set>
#include <vector>

namespace cybercad::native::reference {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;

using math::Ax3;
using math::Dir3;
using math::Point3;
using math::Vec3;

/// A datum result: an origin point + a unit direction, serialised as the 6-double
/// POD the cc_* facade copies into out6 ([ox,oy,oz, dx,dy,dz]).
using Six = std::array<double, 6>;

// ─────────────────────────────────────────────────────────────────────────────
// World placement. Sub-shapes carry a Location (cumulated by the Explorer); the
// leaf geometry is stored LOCAL. These helpers bake the Location into a frame /
// point so results are world-placed exactly as BRep_Tool bakes TopLoc_Location.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

inline Point3 worldPoint(const topo::Location& loc, const Point3& p) noexcept {
  return loc.isIdentity() ? p : loc.transform().applyToPoint(p);
}
inline Dir3 worldDir(const topo::Location& loc, const Dir3& d) noexcept {
  return loc.isIdentity() ? d : loc.transform().applyToDir(d);
}
inline Ax3 worldFrame(const topo::Location& loc, const Ax3& f) noexcept {
  if (loc.isIdentity()) return f;
  return Ax3{worldPoint(loc, f.origin), worldDir(loc, f.x), worldDir(loc, f.y),
             worldDir(loc, f.z)};
}

inline Six six(const Point3& o, const Dir3& d) noexcept {
  return {o.x, o.y, o.z, d.x(), d.y(), d.z()};
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// faceAxis / refAxisFromFace — the axis of a cylindrical or conical face.
// Cylinder/cone ONLY (matches cc_face_axis in occt_query.cpp); a plane, sphere,
// torus or freeform face has no single axis → decline.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<Six> faceAxis(const topo::Shape& face) {
  const auto s = topo::surfaceOf(face);
  if (!s) return std::nullopt;
  using K = topo::FaceSurface::Kind;
  if (s->surface->kind != K::Cylinder && s->surface->kind != K::Cone)
    return std::nullopt;
  const Ax3 f = detail::worldFrame(s->location, s->surface->frame);
  if (!f.z.valid()) return std::nullopt;
  return detail::six(f.origin, f.z);
}

/// Identical to faceAxis (cc_ref_axis_from_face reuses the cc_face_axis extraction
/// bit-for-bit in occt_reference_geometry.cpp).
inline std::optional<Six> refAxisFromFace(const topo::Shape& face) {
  return faceAxis(face);
}

// ─────────────────────────────────────────────────────────────────────────────
// refPlaneFromFace — datum plane from a planar face. Outward normal = the plane's
// Z axis, reversed for a Reversed-oriented face (same outward convention as
// offset_face/replace_face). Origin = the centroid of the outer-wire vertices — a
// point that provably LIES on the (planar) face, the datum-plane analogue of
// OCCT evaluating the surface at the face's UV midpoint. (A datum plane is
// infinite; parity checks the normal exactly and that the two origins are
// coplanar, since any on-plane point is an equally valid datum origin.)
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// World-space centroid of the vertices of a shape's OUTER wire (child 0). Falls
/// back to the frame origin when the face carries no bounded wire.
inline Point3 outerWireCentroid(const topo::Shape& face, const Ax3& frame) {
  const auto& kids = face.tshape()->children();
  if (kids.empty()) return frame.origin;
  // The outer wire is child 0; cumulate the face's own Location onto it.
  const topo::Shape outer =
      kids.front().located(face.location()).oriented(face.orientation());
  Vec3 acc{0, 0, 0};
  int n = 0;
  for (topo::Explorer ex(outer, topo::ShapeType::Vertex); ex.more(); ex.next()) {
    if (const auto p = topo::pointOf(ex.current())) {
      acc += p->asVec();
      ++n;
    }
  }
  if (n == 0) return frame.origin;
  return Point3{acc / static_cast<double>(n)};
}

}  // namespace detail

inline std::optional<Six> refPlaneFromFace(const topo::Shape& face) {
  const auto s = topo::surfaceOf(face);
  if (!s) return std::nullopt;
  if (s->surface->kind != topo::FaceSurface::Kind::Plane) return std::nullopt;
  const Ax3 f = detail::worldFrame(s->location, s->surface->frame);
  if (!f.z.valid()) return std::nullopt;
  Dir3 n = f.z;
  if (face.orientation() == topo::Orientation::Reversed) n = n.reversed();
  return detail::six(detail::outerWireCentroid(face, f), n);
}

// ─────────────────────────────────────────────────────────────────────────────
// refAxisFromEdge — axis (line) of a straight edge. LINE ONLY (a circular edge
// yields no gp_Lin in occt_reference_geometry.cpp → honest decline).
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<Six> refAxisFromEdge(const topo::Shape& edge) {
  const auto c = topo::curveOf(edge);
  if (!c) return std::nullopt;
  if (c->curve->kind != topo::EdgeCurve::Kind::Line) return std::nullopt;
  const Ax3 f = detail::worldFrame(c->location, c->curve->frame);
  if (!f.x.valid()) return std::nullopt;  // Line direction is frame X
  return detail::six(f.origin, f.x);
}

// ─────────────────────────────────────────────────────────────────────────────
// tangentChain — grow a seed edge set to the connected set of tangent-continuous
// edges (edges meeting C1 — |cos(tangent angle)| ≥ cos(15°) — at a shared
// vertex). Mirrors occt_query.cpp::tangentChain so a fillet on one arc/line edge
// rounds the whole smooth contour. Line/Circle/Ellipse tangents are closed-form;
// a freeform (BSpline/Bezier) edge in the walk is an HONEST DECLINE (nullopt) so
// the oracle handles it rather than the native walk under-growing the chain.
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

inline constexpr double kTangentTol = 0.966;  // |cos| of ~15°

/// Closed-form unit tangent of an analytic edge at a world vertex point, or
/// nullopt for a freeform edge (declined) / a degenerate direction (skipped).
inline std::optional<Dir3> analyticTangentAt(const topo::EdgeCurve& c,
                                              const topo::Location& loc,
                                              const Point3& vWorld,
                                              bool& freeform) {
  using K = topo::EdgeCurve::Kind;
  const Ax3 f = worldFrame(loc, c.frame);
  if (c.kind == K::Line) {
    if (!f.x.valid()) return std::nullopt;
    return f.x;  // constant along a line
  }
  if (c.kind == K::Circle || c.kind == K::Ellipse) {
    const double a = c.radius;
    const double b = (c.kind == K::Circle) ? c.radius : c.minorRadius;
    if (a <= 0.0 || b <= 0.0) return std::nullopt;
    const Vec3 r = vWorld - f.origin;                       // world offset
    const double cosT = dot(r, f.x.vec()) / a;              // = cos(param)
    const double sinT = dot(r, f.y.vec()) / b;              // = sin(param)
    const double t = std::atan2(sinT, cosT);
    const Vec3 tan = f.x.vec() * (-a * std::sin(t)) + f.y.vec() * (b * std::cos(t));
    const Dir3 d{tan};
    return d.valid() ? std::optional<Dir3>{d} : std::nullopt;
  }
  freeform = true;  // BSpline / Bezier — cannot certify a closed-form tangent
  return std::nullopt;
}

/// Unit tangent of an edge Shape at world vertex point `v`. Sets `freeform` when
/// the edge is a BSpline/Bezier (no closed-form tangent → the caller declines).
inline std::optional<Dir3> edgeTangentAt(const topo::Shape& edge, const Point3& v,
                                         bool& freeform) {
  const auto ec = topo::curveOf(edge);
  if (!ec) return std::nullopt;
  return analyticTangentAt(*ec->curve, ec->location, v, freeform);
}

}  // namespace detail

/// Returns the grown chain as ascending 1-based edge ids (matching OCCT's
/// std::set order), or nullopt on an HONEST DECLINE (a freeform edge is incident
/// to the walk — the oracle owns that case).
inline std::optional<std::vector<int>> tangentChain(const topo::Shape& root,
                                                     const std::vector<int>& seeds) {
  const topo::ShapeMap emap = topo::mapShapes(root, topo::ShapeType::Edge);
  const topo::AncestryMap vtoe =
      topo::mapShapesAndAncestors(root, topo::ShapeType::Vertex, topo::ShapeType::Edge);
  const int nEdges = static_cast<int>(emap.size());

  std::set<int> inChain;
  std::vector<int> queue;
  for (int s : seeds)
    if (s >= 1 && s <= nEdges && inChain.insert(s).second) queue.push_back(s);

  bool freeform = false;
  while (!queue.empty()) {
    const topo::Shape edge = emap.shape(queue.back());
    queue.pop_back();
    // Walk each bounding vertex; grow across every incident edge that is C1 here.
    for (topo::Explorer vex(edge, topo::ShapeType::Vertex); vex.more(); vex.next()) {
      const auto vp = topo::pointOf(vex.current());
      if (!vp) continue;
      const auto t1 = detail::edgeTangentAt(edge, *vp, freeform);
      if (freeform) return std::nullopt;
      if (!t1) continue;
      for (const topo::Shape& ae : vtoe.parentsOf(vex.current())) {
        const int aid = emap.findIndex(ae);
        if (aid < 1 || inChain.count(aid)) continue;
        const auto t2 = detail::edgeTangentAt(ae, *vp, freeform);
        if (freeform) return std::nullopt;
        if (!t2) continue;
        if (std::fabs(dot(*t1, *t2)) >= detail::kTangentTol) {
          inChain.insert(aid);
          queue.push_back(aid);
        }
      }
    }
  }
  return std::vector<int>(inChain.begin(), inChain.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// outerRimChain — the outer-boundary edge ids of the planar CAP face(s) the seed
// edges bound. Mirrors occt_query.cpp::outer_rim_chain: a face qualifies as a cap
// only if its plane contains ALL seed vertices (within 1.0 model unit), which
// picks the cap and rejects the perpendicular side walls that share one seed
// edge. Each qualifying cap contributes its OUTER wire (child 0) edges, so tapping
// one rim edge rounds the whole rim. Empty result = no cap found (not an error).
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// World plane (origin + unit normal) of a planar face, or nullopt if non-planar.
inline std::optional<std::pair<Point3, Dir3>> facePlane(const topo::Shape& face) {
  const auto s = topo::surfaceOf(face);
  if (!s || s->surface->kind != topo::FaceSurface::Kind::Plane) return std::nullopt;
  const Ax3 f = worldFrame(s->location, s->surface->frame);
  if (!f.z.valid()) return std::nullopt;
  return std::make_pair(f.origin, f.z);
}

}  // namespace detail

inline std::vector<int> outerRimChain(const topo::Shape& root,
                                      const std::vector<int>& seeds) {
  const topo::ShapeMap emap = topo::mapShapes(root, topo::ShapeType::Edge);
  const topo::AncestryMap etof =
      topo::mapShapesAndAncestors(root, topo::ShapeType::Edge, topo::ShapeType::Face);
  const int nEdges = static_cast<int>(emap.size());
  constexpr double kPlaneTol = 1.0;  // separates in-plane (0) from body thickness

  // Collect every seed edge's vertex points (the set the cap plane must contain).
  std::vector<Point3> seedPts;
  for (int sid : seeds) {
    if (sid < 1 || sid > nEdges) continue;
    for (topo::Explorer vex(emap.shape(sid), topo::ShapeType::Vertex); vex.more(); vex.next())
      if (const auto p = topo::pointOf(vex.current())) seedPts.push_back(*p);
  }

  std::set<int> ids;
  for (int sid : seeds) {
    if (sid < 1 || sid > nEdges) continue;
    const topo::Shape se = emap.shape(sid);
    for (const topo::Shape& face : etof.parentsOf(se)) {
      const auto plane = detail::facePlane(face);
      if (!plane) continue;  // only planar caps
      bool allCoplanar = true;
      for (const Point3& p : seedPts)
        if (std::fabs(dot(p - plane->first, plane->second.vec())) > kPlaneTol) {
          allCoplanar = false;
          break;
        }
      if (!allCoplanar) continue;  // a side wall holds only some seeds → not a cap
      const auto& kids = face.tshape()->children();
      if (kids.empty()) continue;
      const topo::Shape outer =
          kids.front().located(face.location()).oriented(face.orientation());
      for (topo::Explorer eex(outer, topo::ShapeType::Edge); eex.more(); eex.next()) {
        const int eid = emap.findIndex(eex.current());
        if (eid >= 1) ids.insert(eid);
      }
    }
  }
  return std::vector<int>(ids.begin(), ids.end());
}

// ─────────────────────────────────────────────────────────────────────────────
// offsetFaceBoundary — offset a planar face's OUTER boundary loop by `distance`
// in the face plane. NATIVE SCOPE: a polygonal boundary (every outer-wire edge is
// a straight line) offset with SHARP (miter) joins, returned as a closed xyz
// polyline (the corner points, loop-ordered). This is the exact, closed-form case
// and the one where the result provably COINCIDES with OCCT's
// BRepOffsetAPI_MakeOffset (whose GeomAbs_Arc joins add NO arc at a corner that
// the offset sharpens — i.e. an inward offset of a convex loop).
//
// HONEST DECLINE (nullopt) — the oracle keeps these:
//   * non-planar face / no bounded outer wire;
//   * any non-line outer-wire edge (an arc boundary; OCCT discretises arcs and
//     arc-joins, which the native miter does not reproduce);
//   * an offset that would arc-round a CONVEX corner in OCCT (a loop-growing
//     offset of a convex polygon) — native does not silently emit sharp corners
//     where the oracle emits rounded ones;
//   * a self-intersecting / collapsing offset (a corner or edge flips).
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {

/// Ordered world vertices of a wire made of line edges, or nullopt if any edge is
/// non-linear. Walks edges in stored order, appending each edge's start vertex.
inline std::optional<std::vector<Point3>> polygonVertices(const topo::Shape& wire) {
  std::vector<Point3> verts;
  for (topo::Explorer eex(wire, topo::ShapeType::Edge); eex.more(); eex.next()) {
    const topo::Shape edge = eex.current();
    const auto c = topo::curveOf(edge);
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Line) return std::nullopt;
    std::vector<Point3> vp;
    for (topo::Explorer vex(edge, topo::ShapeType::Vertex); vex.more(); vex.next())
      if (const auto p = topo::pointOf(vex.current())) vp.push_back(*p);
    if (vp.empty()) return std::nullopt;
    verts.push_back(vp.front());  // start vertex of each edge → the loop corners
  }
  return verts;
}

}  // namespace detail

inline std::optional<std::vector<double>> offsetFaceBoundary(const topo::Shape& face,
                                                             double distance) {
  const auto s = topo::surfaceOf(face);
  if (!s || s->surface->kind != topo::FaceSurface::Kind::Plane) return std::nullopt;
  const Ax3 f = detail::worldFrame(s->location, s->surface->frame);
  if (!f.z.valid()) return std::nullopt;
  const auto& kids = face.tshape()->children();
  if (kids.empty()) return std::nullopt;
  const topo::Shape outer =
      kids.front().located(face.location()).oriented(face.orientation());

  const auto poly = detail::polygonVertices(outer);
  if (!poly || poly->size() < 3) return std::nullopt;
  const std::vector<Point3>& P = *poly;
  const int n = static_cast<int>(P.size());

  // Project the corners to 2D plane coords (u along X, v along Y).
  std::vector<double> u(n), v(n);
  for (int i = 0; i < n; ++i) {
    const Vec3 r = P[i] - f.origin;
    u[i] = dot(r, f.x.vec());
    v[i] = dot(r, f.y.vec());
  }
  // Signed area (shoelace) → loop winding sign; also its magnitude for the
  // grow/shrink test.
  double area2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    area2 += u[i] * v[j] - u[j] * v[i];
  }
  if (std::fabs(area2) < 1e-12) return std::nullopt;  // degenerate loop
  const double sign = (area2 > 0.0) ? 1.0 : -1.0;

  // Convexity: every turn has the same sign as the winding.
  auto cross2 = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
  bool convex = true;
  for (int i = 0; i < n; ++i) {
    const int p = (i + n - 1) % n, q = (i + 1) % n;
    const double c = cross2(u[i] - u[p], v[i] - v[p], u[q] - u[i], v[q] - v[i]);
    if (c * sign < 0.0) { convex = false; break; }
  }

  // Offset each edge inward by `distance` along its inward normal, then intersect
  // adjacent offset lines to get the mitered corner. Inward normal of edge (i→i+1)
  // for a loop of winding `sign` is (-dy, dx)*sign normalised, negated by distance
  // sign so a positive distance shrinks (inward) the loop.
  std::vector<std::array<double, 4>> lines(n);  // (px,py, dx,dy) offset edge lines
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    double ex = u[j] - u[i], ey = v[j] - v[i];
    const double len = std::sqrt(ex * ex + ey * ey);
    if (len < 1e-12) return std::nullopt;  // zero-length edge
    ex /= len; ey /= len;
    const double nx = -ey * sign, ny = ex * sign;  // inward normal (toward interior)
    lines[i] = {u[i] + nx * distance, v[i] + ny * distance, ex, ey};
  }
  // Corner i = intersection of offset lines (i-1) and (i).
  std::vector<double> ou(n), ov(n);
  for (int i = 0; i < n; ++i) {
    const int p = (i + n - 1) % n;
    const double p1x = lines[p][0], p1y = lines[p][1], d1x = lines[p][2], d1y = lines[p][3];
    const double p2x = lines[i][0], p2y = lines[i][1], d2x = lines[i][2], d2y = lines[i][3];
    const double denom = d1x * d2y - d1y * d2x;
    if (std::fabs(denom) < 1e-12) return std::nullopt;  // parallel (collinear corner)
    const double t = ((p2x - p1x) * d2y - (p2y - p1y) * d2x) / denom;
    ou[i] = p1x + d1x * t;
    ov[i] = p1y + d1y * t;
  }
  // Grow/shrink + self-intersection guard: the offset must not FLIP winding, and a
  // convex loop must be SHRINKING (a growing convex offset is arc-rounded by OCCT
  // → decline). Compare offset signed area to the original.
  double oarea2 = 0.0;
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    oarea2 += ou[i] * ov[j] - ou[j] * ov[i];
  }
  if (oarea2 * area2 <= 0.0) return std::nullopt;  // collapsed / flipped
  if (convex && std::fabs(oarea2) > std::fabs(area2))
    return std::nullopt;  // growing convex offset → OCCT rounds corners; decline

  // Lift back to world (xyz triplets, loop-ordered, closed implicitly).
  std::vector<double> out;
  out.reserve(static_cast<std::size_t>(n) * 3);
  for (int i = 0; i < n; ++i) {
    const Point3 w = f.origin + f.x.vec() * ou[i] + f.y.vec() * ov[i];
    out.push_back(w.x);
    out.push_back(w.y);
    out.push_back(w.z);
  }
  return out;
}

}  // namespace cybercad::native::reference

#endif  // CYBERCAD_NATIVE_REFERENCE_REFERENCE_H

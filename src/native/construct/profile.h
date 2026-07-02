// SPDX-License-Identifier: Apache-2.0
//
// profile.h — Tier-A native construction (Phase 4 #4b): typed profiles, holed
// extrude, and typed-profile revolve. Clean-room, OCCT-FREE builders that extend
// construct.h with:
//
//   * build_prism_with_holes(outer, holes[], depth) — extrude a closed OUTER loop
//     with one or more inner HOLE loops (circular OR polygon) into a prism SOLID.
//     Both caps carry the hole wires (a trimmed planar face with inner boundaries),
//     and every loop — outer AND each hole — grows its own ring of side faces so
//     the tube walls are closed. Circular holes keep a TRUE circle edge (one closed
//     Circle edge per cap ring + one Cylinder side face), not a sampled polygon.
//
//   * build_prism_profile(segs, holes[], depth) — extrude a TYPED profile whose
//     outer boundary is a sequence of ProfileSegment (kind 0 line / 1 arc / 2 full
//     circle) plus circular/polygon holes. Line edges → Line, arc edges → Circle
//     arc, a full-circle segment → one closed Circle edge; the swept side faces are
//     Plane (line) / Cylinder (arc or circle). kind 3 (spline) is NOT handled here
//     — the builder returns a NULL Shape so the engine falls through to OCCT (honest
//     deferral, never faked).
//
//   * build_revolution_profile(segs, axis, angle) — revolve a TYPED profile about an
//     in-plane axis. Per segment: a line classifies (as in build_revolution) to
//     Plane / Cylinder / Cone; an ARC whose circle centre lies ON the axis sweeps a
//     Sphere band. An arc whose centre is OFF the axis would sweep a TORUS (no native
//     Torus surface yet) and a spline segment a general surface of revolution — both
//     are deferred (NULL Shape → OCCT). Full 2π closes the shell; a partial angle
//     adds two planar meridian caps.
//
// The native tessellator (src/native/tessellate) already meshes a trimmed planar
// face with inner (hole) wires: children beyond index 0 become UV hole loops that
// are bridged + ear-clipped (trim.h / uv_triangulate.h), and a closed Circle edge is
// discretized by curvature into a shared fraction list (edge_mesher.h) so both the
// cap ring and the cylindrical wall weld watertight along the true circle. So these
// builders only assemble topology + analytic geometry; no mesher change is needed.
//
// REFERENCE ORACLE ONLY: OCCT BRepBuilderAPI_MakeFace(outer, holes) and
// BRepPrimAPI_MakePrism / _MakeRevol were consulted to confirm the face
// decomposition (outer wire + hole wires on the caps, one side ring per loop) and
// the outward-orientation convention; nothing is copied.
//
// OCCT-FREE. Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_CONSTRUCT_PROFILE_H
#define CYBERCAD_NATIVE_CONSTRUCT_PROFILE_H

#include "native/construct/construct.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::construct {

// ─────────────────────────────────────────────────────────────────────────────
// Typed profile input (mirrors CCProfileSeg / cyber::ProfileSeg 1:1 so the engine
// glue can memcpy-map). construct/ stays OCCT-FREE; this is a plain POD.
//
//   kind 0 line   : (x0,y0) → (x1,y1)
//   kind 1 arc    : circle (cx,cy,r), swept from angle a0 → a1, endpoints
//                   (x0,y0)/(x1,y1) (used only to seed shared vertices)
//   kind 2 circle : full circle (cx,cy,r) — a whole closed loop by itself
//   kind 3 spline : DEFERRED here (engine falls through to OCCT)
// ─────────────────────────────────────────────────────────────────────────────
struct ProfileSegment {
  int kind = 0;
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
  double cx = 0.0, cy = 0.0, r = 0.0;
  double a0 = 0.0, a1 = 0.0;
  int ptOffset = 0, ptCount = 0;  // kind 3 only (unused natively)
};

// A circular hole: centre (cx,cy) on z=0 and radius r.
struct CircleHole {
  double cx = 0.0, cy = 0.0, r = 0.0;
};

namespace detail {

// The Z-up plane frame at height z, X = +X world, Y = +Y world. Cap faces and the
// hole wires all live on this frame so their pcurves are the plain (x,y) of the
// point (u = x, v = y).
inline math::Ax3 capFrame(double z) noexcept {
  return math::Ax3{math::Point3{0, 0, z}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0},
                   math::Dir3{0, 0, 1}};
}

// ── A closed BOUNDARY LOOP in the z=0 plane, resolved to shared vertices ──────
// A loop is either a POLYGON (ordered vertices, straight edges between them) or a
// full CIRCLE (centre + radius, one closed Circle edge). Both are extruded: the
// polygon into one Plane quad per edge, the circle into one Cylinder wall.
//
// `pts` are the polygon vertices in the z=0 plane (empty for a circle). `ccw` is
// the loop winding (outer loops are forced CCW, holes CW, by the caller) so the
// swept side-face normals point out of the SOLID material.
struct Loop {
  bool isCircle = false;
  double cx = 0.0, cy = 0.0, r = 0.0;  // circle only
  std::vector<math::Point3> pts;       // polygon only (z=0)
  bool ccw = true;
};

// Signed area (shoelace) of a polygon loop; used to normalise winding.
inline double signedArea(const std::vector<math::Point3>& p) noexcept {
  double a2 = 0.0;
  const std::size_t n = p.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    a2 += p[i].x * p[j].y - p[j].x * p[i].y;
  }
  return 0.5 * a2;
}

// Drop a repeated closing vertex (last == first) so a loop is a clean cycle.
inline void dropClosingDuplicate(std::vector<math::Point3>& p) {
  if (p.size() > 1 && math::distance(p.front(), p.back()) < kProfileTol) p.pop_back();
}

// A vertex ring: one native Vertex per loop corner, at height z. For a circle the
// ring is empty (the closed edge needs no shared corner vertex).
inline std::vector<topo::Shape> vertexRing(const Loop& loop, double z) {
  std::vector<topo::Shape> ring;
  ring.reserve(loop.pts.size());
  for (const math::Point3& p : loop.pts)
    ring.push_back(topo::ShapeBuilder::makeVertex(math::Point3{p.x, p.y, z}));
  return ring;
}

// ── Cap wire for a loop, on the given plane frame ─────────────────────────────
// Polygon → a wire of Line edges (each carrying its Line pcurve = the segment in
// (x,y)). Circle → a wire with ONE closed Circle edge (param 0→2π) carrying a
// Circle pcurve (centre (cx,cy), radius r) so the trimmer flattens it as a true
// circle. Vertices are shared with `ring` (polygon) so caps weld to side walls.
inline topo::Shape capWire(const Loop& loop, const std::vector<topo::Shape>& ring,
                           const math::Ax3& frame) {
  if (loop.isCircle) {
    // One closed circle edge: start == end vertex at angle 0 on z = frame origin.
    const double z = frame.origin.z;
    const topo::Shape v =
        topo::ShapeBuilder::makeVertex(math::Point3{loop.cx + loop.r, loop.cy, z});
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Circle;
    c.frame = math::Ax3{math::Point3{loop.cx, loop.cy, z}, math::Dir3{1, 0, 0},
                        math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};
    c.radius = loop.r;
    const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, 0.0, kFullTurn, v, v);
    topo::PCurve pc;  // (u,v) = centre + R(cos t, sin t)
    pc.kind = topo::EdgeCurve::Kind::Circle;
    pc.origin2d = math::Point3{loop.cx, loop.cy, 0.0};
    pc.dir2d = math::Vec3{loop.r, 0.0, 0.0};  // dir2d.x carries the radius
    return topo::ShapeBuilder::makeWire({topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc)});
  }
  std::vector<topo::Shape> edges;
  const std::size_t n = ring.size();
  edges.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    edges.push_back(detail::planarEdge(ring[i], ring[(i + 1) % n], frame));
  return topo::ShapeBuilder::makeWire(std::move(edges));
}

// ── Side faces sweeping one loop between z=0 and z=depth ──────────────────────
// Polygon → one Plane quad per edge (outward normal = edgeDir × Zup for a CCW
// loop, flipped for CW so a hole's wall faces INTO the material as a solid wall).
// Circle → one Cylinder wall face (a full parametric rectangle: u=θ∈[0,2π],
// v=z∈[0,depth]) with the two closed circle rim edges pinned as its boundary.
inline void appendSideFaces(std::vector<topo::Shape>& faces, const Loop& loop,
                            const std::vector<topo::Shape>& bottom,
                            const std::vector<topo::Shape>& top, double depth) {
  if (loop.isCircle) {
    // Cylinder wall. Rim edges are true circles at z=0 and z=depth.
    auto rim = [&](double z, double t0, double t1) -> topo::Shape {
      const topo::Shape v0 =
          topo::ShapeBuilder::makeVertex(math::Point3{loop.cx + loop.r, loop.cy, z});
      topo::EdgeCurve c;
      c.kind = topo::EdgeCurve::Kind::Circle;
      c.frame = math::Ax3{math::Point3{loop.cx, loop.cy, z}, math::Dir3{1, 0, 0},
                          math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};
      c.radius = loop.r;
      return topo::ShapeBuilder::makeEdge(c, t0, t1, v0, v0);
    };
    topo::FaceSurface s;
    s.kind = topo::FaceSurface::Kind::Cylinder;
    s.frame = math::Ax3{math::Point3{loop.cx, loop.cy, 0.0}, math::Dir3{1, 0, 0},
                        math::Dir3{0, 1, 0}, math::Dir3{0, 0, 1}};
    s.radius = loop.r;

    // Cylinder pcurves: bottom/top rim = Line v=const (u=θ); the two seam profile
    // edges (u=0 and u=2π) close the parametric rectangle at v: 0→depth.
    auto rimPc = [&](double vconst) {
      topo::PCurve pc;
      pc.kind = topo::EdgeCurve::Kind::Line;
      pc.origin2d = math::Point3{0.0, vconst, 0.0};
      pc.dir2d = math::Vec3{1.0, 0.0, 0.0};  // u = θ
      return pc;
    };
    const topo::Shape rb = rim(0.0, 0.0, kFullTurn);
    const topo::Shape rt = rim(depth, 0.0, kFullTurn);
    const topo::Shape rbp = topo::ShapeBuilder::addPCurve(rb, rb.tshape(), rimPc(0.0));
    const topo::Shape rtp = topo::ShapeBuilder::addPCurve(rt, rt.tshape(), rimPc(depth));
    // Outer wire of the wall: bottom rim (forward) + top rim (reversed). A closed
    // periodic cylinder needs only the two rim loops; the tessellator's structured
    // grid over the natural [0,2π]×[0,depth] box meshes it (no distinct box corners
    // ⇒ admitted by the border test alone). Reversed for a hole so the wall faces in.
    const topo::Orientation orient =
        loop.ccw ? topo::Orientation::Forward : topo::Orientation::Reversed;
    faces.push_back(topo::ShapeBuilder::makeFace(
        s, topo::ShapeBuilder::makeWire({rbp, rtp.oriented(topo::Orientation::Reversed)}), {},
        orient));
    return;
  }
  // Uniform rule for BOTH outer and hole loops, given the caller stores the outer
  // CCW and each hole CW: the wall normal at directed edge i→j is edgeDir × Zup
  // ("to the right of travel"). For the CCW outer that points AWAY from the loop
  // interior (outward from the solid); for a CW hole it points INTO the hole void
  // (also outward from the solid). So no per-loop negation is needed — the stored
  // winding already encodes the material side. Quad wound b_i→b_j→t_j→t_i so its
  // own CCW winding matches that normal.
  const std::size_t n = bottom.size();
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    const math::Vec3 e = *topo::pointOf(bottom[j]) - *topo::pointOf(bottom[i]);
    const math::Vec3 nrm = math::cross(e, math::Vec3{0, 0, 1});
    std::vector<topo::Shape> quad{bottom[i], bottom[j], top[j], top[i]};
    faces.push_back(detail::planarFace(quad, math::Dir3{nrm}, topo::Orientation::Forward));
  }
}

// ── A trimmed planar cap face (outer loop + hole loops) at height z ───────────
// `normal` is the cap's outward normal (−Z bottom, +Z top). The outer wire is
// wound so its own CCW winding faces `normal`; hole wires are added as inner
// boundaries. Returns the face + the built vertex rings (so the side walls reuse
// the SAME vertices and weld watertight). `outerRing`/`holeRings` are filled.
inline topo::Shape capFace(const Loop& outer, const std::vector<Loop>& holes,
                           const std::vector<topo::Shape>& outerRing,
                           const std::vector<std::vector<topo::Shape>>& holeRings, double z,
                           const math::Dir3& normal) {
  const math::Ax3 frame = capFrame(z);
  // Outer wire: reverse the vertex order for the −Z cap so the wire winds CCW as
  // seen from the outward side (matching planarFace's convention).
  const bool flip = normal.z() < 0.0;
  Loop outerFixed = outer;
  std::vector<topo::Shape> outerV = outerRing;
  if (flip && !outer.isCircle) std::reverse(outerV.begin(), outerV.end());
  const topo::Shape outerWire = capWire(outerFixed, outerV, frame);

  std::vector<topo::Shape> holeWires;
  holeWires.reserve(holes.size());
  for (std::size_t k = 0; k < holes.size(); ++k) {
    // Hole wire on the cap: keep it wound OPPOSITE the outer so the trimmer reads
    // it as an inner boundary. For the −Z cap the whole face flips, so holes flip
    // too. (The tessellator's point-in-polygon test is winding-agnostic, but we
    // keep orientation faithful for future consumers.)
    std::vector<topo::Shape> hv = holeRings[k];
    if (flip && !holes[k].isCircle) std::reverse(hv.begin(), hv.end());
    holeWires.push_back(capWire(holes[k], hv, frame));
  }

  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, outerWire, std::move(holeWires),
                                      topo::Orientation::Forward);
}

// ── Assemble a holed prism from resolved loops ────────────────────────────────
// Shared bottom/top vertex rings per loop; a bottom cap, a top cap (both trimmed
// by the hole loops), and one side ring per loop (outer + each hole). Returns a
// NULL Shape on degenerate input.
inline topo::Shape assembleHoledPrism(Loop outer, std::vector<Loop> holes, double depth) {
  if (!(depth > kMinDepth)) return {};
  // Normalise winding: outer CCW, holes CW (so hole walls face inward as solid).
  if (!outer.isCircle) {
    dropClosingDuplicate(outer.pts);
    if (outer.pts.size() < 3) return {};
    const double a = signedArea(outer.pts);
    if (std::fabs(a) < kProfileTol) return {};
    outer.ccw = a > 0.0;
    if (!outer.ccw) {  // force CCW storage
      std::reverse(outer.pts.begin(), outer.pts.end());
      outer.ccw = true;
    }
  } else {
    if (!(outer.r > kProfileTol)) return {};
    outer.ccw = true;
  }
  for (Loop& h : holes) {
    if (!h.isCircle) {
      dropClosingDuplicate(h.pts);
      if (h.pts.size() < 3) return {};
      const double a = signedArea(h.pts);
      if (std::fabs(a) < kProfileTol) return {};
      // Force CW storage (a > 0 is CCW → reverse).
      if (a > 0.0) std::reverse(h.pts.begin(), h.pts.end());
      h.ccw = false;
    } else {
      if (!(h.r > kProfileTol)) return {};
      h.ccw = false;
    }
  }

  // Vertex rings (bottom/top) for the outer and each hole.
  const std::vector<topo::Shape> outerBot = vertexRing(outer, 0.0);
  const std::vector<topo::Shape> outerTop = vertexRing(outer, depth);
  std::vector<std::vector<topo::Shape>> holeBot(holes.size()), holeTop(holes.size());
  for (std::size_t k = 0; k < holes.size(); ++k) {
    holeBot[k] = vertexRing(holes[k], 0.0);
    holeTop[k] = vertexRing(holes[k], depth);
  }

  std::vector<topo::Shape> faces;
  faces.reserve(holes.size() * 2 + outer.pts.size() + 4);

  faces.push_back(capFace(outer, holes, outerBot, holeBot, 0.0, math::Dir3{0, 0, -1}));
  faces.push_back(capFace(outer, holes, outerTop, holeTop, depth, math::Dir3{0, 0, 1}));

  appendSideFaces(faces, outer, outerBot, outerTop, depth);
  for (std::size_t k = 0; k < holes.size(); ++k)
    appendSideFaces(faces, holes[k], holeBot[k], holeTop[k], depth);

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ── Resolve a typed profile to a single polygon/arc outer Loop ────────────────
// A profile made only of kind-0/1/2 segments resolves to ONE boundary loop:
//   * a lone kind-2 full circle → a circle Loop.
//   * a sequence of line (kind 0) and arc (kind 1) segments → a POLYGON loop whose
//     vertices are the segment endpoints (arcs tessellated into the cap polygon).
//     This POLYGON fallback is used ONLY for the cap silhouette when the caller does
//     NOT take the true-typed path; the true-typed path (assembleTypedProfileSolid)
//     keeps arc side walls as real Cylinders. Kept for the winding/degeneracy checks.
// Returns nullopt if the profile contains a kind-3 spline (deferred) or is empty.
inline int arcSamples(double r, double a0, double a1, double deflection) noexcept {
  const double span = std::fabs(a1 - a0);
  if (r <= kProfileTol || span <= kProfileTol) return 1;
  // sagitta d = r(1 − cos(Δ/2)) ≤ deflection ⇒ Δ ≤ 2·acos(1 − d/r).
  const double ratio = 1.0 - std::clamp(deflection / r, 0.0, 1.0);
  const double dmax = 2.0 * std::acos(std::clamp(ratio, -1.0, 1.0));
  const int n = dmax > 1e-9 ? static_cast<int>(std::ceil(span / dmax)) : 64;
  return std::clamp(n, 1, 512);
}

// ── True typed OUTER edge (kind 0 line or kind 1 arc), resolved to 3D endpoints ──
// An arc carries its circle centre/radius and angle range so the extruded side wall
// is a TRUE Cylinder patch and the cap boundary edge is a TRUE Circle arc (not a
// chord polyline). The endpoints (p0→p1) are the profile-plane points.
struct TypedEdge {
  bool isArc = false;
  math::Point3 p0{}, p1{};   // z=0 endpoints
  double cx = 0, cy = 0, r = 0, a0 = 0, a1 = 0;  // arc only
};

// Resolve a typed outer profile into an ordered TypedEdge loop. Returns nullopt for
// a kind-3 spline / stray kind-2 mid-loop / empty input (engine falls through). A
// lone kind-2 full circle is handled separately (Loop isCircle path) — not here.
inline std::optional<std::vector<TypedEdge>> resolveTypedOuter(
    const std::vector<ProfileSegment>& segs) {
  if (segs.empty()) return std::nullopt;
  std::vector<TypedEdge> out;
  out.reserve(segs.size());
  for (const ProfileSegment& s : segs) {
    if (s.kind == 3 || s.kind == 2) return std::nullopt;  // spline / stray circle: defer
    TypedEdge e;
    if (s.kind == 1) {
      e.isArc = true;
      e.cx = s.cx; e.cy = s.cy; e.r = s.r; e.a0 = s.a0; e.a1 = s.a1;
      e.p0 = math::Point3{s.cx + s.r * std::cos(s.a0), s.cy + s.r * std::sin(s.a0), 0.0};
      e.p1 = math::Point3{s.cx + s.r * std::cos(s.a1), s.cy + s.r * std::sin(s.a1), 0.0};
    } else {
      e.p0 = math::Point3{s.x0, s.y0, 0.0};
      e.p1 = math::Point3{s.x1, s.y1, 0.0};
    }
    out.push_back(e);
  }
  return out;
}

// A signed-area estimate of a typed loop (arcs sampled coarsely) — for winding.
inline double typedLoopArea(const std::vector<TypedEdge>& edges) {
  std::vector<math::Point3> poly;
  for (const TypedEdge& e : edges) {
    if (e.isArc) {
      const int n = 8;
      for (int i = 0; i < n; ++i) {
        const double a = e.a0 + (e.a1 - e.a0) * (static_cast<double>(i) / n);
        poly.push_back(math::Point3{e.cx + e.r * std::cos(a), e.cy + e.r * std::sin(a), 0.0});
      }
    } else {
      poly.push_back(e.p0);
    }
  }
  return signedArea(poly);
}

// One cap-boundary edge for a TypedEdge, on the cap plane frame at height z, using
// the SHARED endpoint vertices `v0`/`v1`. Line → Line edge + Line pcurve; arc →
// Circle edge over [a0,a1] + Circle pcurve (centre (cx,cy), radius r).
inline topo::Shape typedCapEdge(const TypedEdge& e, const topo::Shape& v0, const topo::Shape& v1,
                                const math::Ax3& frame) {
  if (!e.isArc) return detail::planarEdge(v0, v1, frame);
  const double z = frame.origin.z;
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = math::Ax3{math::Point3{e.cx, e.cy, z}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0},
                      math::Dir3{0, 0, 1}};
  c.radius = e.r;
  const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, e.a0, e.a1, v0, v1);
  topo::PCurve pc;
  pc.kind = topo::EdgeCurve::Kind::Circle;
  pc.origin2d = math::Point3{e.cx, e.cy, 0.0};
  pc.dir2d = math::Vec3{e.r, 0.0, 0.0};  // dir2d.x carries the pcurve radius
  return topo::ShapeBuilder::addPCurve(edge, edge.tshape(), pc);
}

// A Cylinder side-wall patch for one ARC outer edge over angular span [t0,t1] and
// z ∈ [0,depth]. Boundary: bottom arc (v=0), right vertical seam (u=t1), top arc
// (v=depth, reversed), left vertical seam (u=t0). A full parametric rectangle over
// [t0,t1]×[0,depth] ⇒ the tessellator meshes it as a structured grid and welds to
// the caps via the shared bottom/top arc edges. `outwardCcw` matches the loop
// winding (outer CCW → Forward; a hole would reverse, but arcs are outer-only here).
inline topo::Shape arcWallPatch(const TypedEdge& e, double t0, double t1, double depth,
                                bool outwardCcw) {
  auto arc = [&](double z, double s0, double s1) -> topo::Shape {
    const topo::Shape va =
        topo::ShapeBuilder::makeVertex(math::Point3{e.cx + e.r * std::cos(s0), e.cy + e.r * std::sin(s0), z});
    const topo::Shape vb =
        topo::ShapeBuilder::makeVertex(math::Point3{e.cx + e.r * std::cos(s1), e.cy + e.r * std::sin(s1), z});
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Circle;
    c.frame = math::Ax3{math::Point3{e.cx, e.cy, z}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0},
                        math::Dir3{0, 0, 1}};
    c.radius = e.r;
    return topo::ShapeBuilder::makeEdge(c, s0, s1, va, vb);
  };
  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Cylinder;
  s.frame = math::Ax3{math::Point3{e.cx, e.cy, 0.0}, math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0},
                      math::Dir3{0, 0, 1}};
  s.radius = e.r;
  auto arcPc = [&](double vconst) {
    topo::PCurve pc;
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = math::Point3{0.0, vconst, 0.0};
    pc.dir2d = math::Vec3{1.0, 0.0, 0.0};  // u = θ
    return pc;
  };
  const topo::Shape ab = arc(0.0, t0, t1);
  const topo::Shape at = arc(depth, t0, t1);
  const topo::Shape abp = topo::ShapeBuilder::addPCurve(ab, ab.tshape(), arcPc(0.0));
  const topo::Shape atp = topo::ShapeBuilder::addPCurve(at, at.tshape(), arcPc(depth));
  const topo::Orientation orient =
      outwardCcw ? topo::Orientation::Forward : topo::Orientation::Reversed;
  return topo::ShapeBuilder::makeFace(
      s, topo::ShapeBuilder::makeWire({abp, atp.oriented(topo::Orientation::Reversed)}), {}, orient);
}

// Build the cap wire for a typed outer loop on `frame` at height z, sharing the
// junction vertices in `ring` (ring[i] = start vertex of edge i, in loop order).
inline topo::Shape typedCapWire(const std::vector<TypedEdge>& edges,
                                const std::vector<topo::Shape>& ring, const math::Ax3& frame) {
  std::vector<topo::Shape> wireEdges;
  const std::size_t n = edges.size();
  wireEdges.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    wireEdges.push_back(typedCapEdge(edges[i], ring[i], ring[(i + 1) % n], frame));
  return topo::ShapeBuilder::makeWire(std::move(wireEdges));
}

// ── Assemble a typed-outer-profile prism (line/arc edges) with holes ──────────
// The outer boundary is `edges` (already CCW). Junction vertices are shared between
// the two caps and the side walls so the solid welds watertight. Line edges give a
// Plane quad; arc edges give one-or-more Cylinder patches (split into <π spans).
// Holes (circular / polygon) reuse the Loop machinery. Returns NULL on degeneracy.
//
// Cognitive complexity: a linear assembler (~20, systems band) — winding fix, the
// two shared vertex rings, hole normalisation, the two cap faces (isolated in the
// capFaceTyped lambda), and the per-edge side-wall dispatch (line→Plane / arc→
// Cylinder patches). Nesting stays ≤ 2; flagged, within the ≤25-35 geometry band.
inline topo::Shape assembleTypedProfileSolid(std::vector<TypedEdge> edges, std::vector<Loop> holes,
                                             double depth) {
  if (!(depth > kMinDepth) || edges.size() < 2) return {};
  // Force CCW outer winding.
  if (typedLoopArea(edges) < 0.0) {
    std::reverse(edges.begin(), edges.end());
    for (TypedEdge& e : edges) {
      std::swap(e.p0, e.p1);
      if (e.isArc) std::swap(e.a0, e.a1);
    }
  }
  const std::size_t n = edges.size();
  // Junction vertex rings (one vertex per edge start), bottom + top.
  std::vector<topo::Shape> bot(n), top(n);
  for (std::size_t i = 0; i < n; ++i) {
    bot[i] = topo::ShapeBuilder::makeVertex(math::Point3{edges[i].p0.x, edges[i].p0.y, 0.0});
    top[i] = topo::ShapeBuilder::makeVertex(math::Point3{edges[i].p0.x, edges[i].p0.y, depth});
  }
  // Normalise holes (CW storage) and build their rings.
  for (Loop& h : holes) {
    if (!h.isCircle) {
      dropClosingDuplicate(h.pts);
      if (h.pts.size() < 3) return {};
      if (signedArea(h.pts) > 0.0) std::reverse(h.pts.begin(), h.pts.end());
      h.ccw = false;
    } else {
      if (!(h.r > kProfileTol)) return {};
      h.ccw = false;
    }
  }
  std::vector<std::vector<topo::Shape>> holeBot(holes.size()), holeTop(holes.size());
  for (std::size_t k = 0; k < holes.size(); ++k) {
    holeBot[k] = vertexRing(holes[k], 0.0);
    holeTop[k] = vertexRing(holes[k], depth);
  }

  std::vector<topo::Shape> faces;

  // Cap faces (bottom −Z, top +Z), trimmed by the hole wires. The −Z cap reverses
  // the outer edge order/winding so its own CCW faces −Z.
  auto capFaceTyped = [&](const std::vector<topo::Shape>& ring,
                          const std::vector<std::vector<topo::Shape>>& hRings, double z,
                          bool bottom) -> topo::Shape {
    const math::Ax3 frame = capFrame(z);
    std::vector<TypedEdge> capEdges = edges;
    std::vector<topo::Shape> capRing = ring;
    if (bottom) {  // reverse to face −Z
      std::reverse(capEdges.begin(), capEdges.end());
      for (TypedEdge& e : capEdges) { std::swap(e.p0, e.p1); if (e.isArc) std::swap(e.a0, e.a1); }
      // ring[i] is edge i's start vertex; after reversing edges, the start of new
      // edge i is the OLD end of old edge (n-1-i) = old start of edge (n-i) mod n.
      std::vector<topo::Shape> r(n);
      for (std::size_t i = 0; i < n; ++i) r[i] = ring[(n - i) % n];
      capRing.swap(r);
    }
    const topo::Shape outerWire = typedCapWire(capEdges, capRing, frame);
    std::vector<topo::Shape> holeWires;
    for (std::size_t k = 0; k < holes.size(); ++k) {
      std::vector<topo::Shape> hv = hRings[k];
      if (bottom && !holes[k].isCircle) std::reverse(hv.begin(), hv.end());
      holeWires.push_back(capWire(holes[k], hv, frame));
    }
    topo::FaceSurface s;
    s.kind = topo::FaceSurface::Kind::Plane;
    s.frame = frame;
    return topo::ShapeBuilder::makeFace(s, outerWire, std::move(holeWires),
                                        topo::Orientation::Forward);
  };
  faces.push_back(capFaceTyped(bot, holeBot, 0.0, /*bottom=*/true));
  faces.push_back(capFaceTyped(top, holeTop, depth, /*bottom=*/false));

  // Outer side walls: Plane quad per line edge; Cylinder patch(es) per arc edge.
  //
  // Split threshold = π (180°), NOT the revolve's 120°. A revolve tiles a full
  // PERIODIC surface of revolution, so it must split into strictly-<π non-periodic
  // patches. An arc EXTRUDE wall is different: it is always a BOUNDED, non-periodic
  // cylindrical patch (four real boundary edges, ear-clipped by the tessellator) —
  // periodicity never arises. So a single OCCT-equivalent arc (≤ 180°, e.g. a
  // semicircle) stays ONE Cylinder patch, matching OCCT's single cylindrical face
  // (face-count parity). Spans in (π, 2π) split into 2 near-equal patches (each
  // still < 2π, non-periodic); a full circle is the kind-2 path, never reached here.
  constexpr double kMaxSpan = kFullTurn / 2.0;  // 180°
  for (std::size_t i = 0; i < n; ++i) {
    const std::size_t j = (i + 1) % n;
    const TypedEdge& e = edges[i];
    if (!e.isArc) {
      const math::Vec3 edge = *topo::pointOf(bot[j]) - *topo::pointOf(bot[i]);
      const math::Vec3 nrm = math::cross(edge, math::Vec3{0, 0, 1});
      std::vector<topo::Shape> quad{bot[i], bot[j], top[j], top[i]};
      faces.push_back(detail::planarFace(quad, math::Dir3{nrm}, topo::Orientation::Forward));
    } else {
      const double span = std::fabs(e.a1 - e.a0);
      const int nSpan = std::max(1, static_cast<int>(std::ceil(span / kMaxSpan - 1e-9)));
      for (int k = 0; k < nSpan; ++k) {
        const double s0 = e.a0 + (e.a1 - e.a0) * (static_cast<double>(k) / nSpan);
        const double s1 = e.a0 + (e.a1 - e.a0) * (static_cast<double>(k + 1) / nSpan);
        faces.push_back(arcWallPatch(e, s0, s1, depth, /*outwardCcw=*/true));
      }
    }
  }
  // Hole side rings (same uniform rule as assembleHoledPrism: stored CW → outward).
  for (std::size_t k = 0; k < holes.size(); ++k)
    appendSideFaces(faces, holes[k], holeBot[k], holeTop[k], depth);

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// Typed-profile REVOLVE helpers. Line segments reuse construct.h's segmentSurface
// / sweptPatch / faceOrientation (Plane / Cylinder / Cone). An ARC segment whose
// circle centre lies ON the axis sweeps a SPHERE band; an arc centred OFF the axis
// (a Torus) and a spline are DEFERRED (signalled by returning std::nullopt from the
// per-segment classifier so the whole builder falls through to OCCT).
// ─────────────────────────────────────────────────────────────────────────────

// The (r,h) of a 2D profile point in the axis frame (r = perpendicular radius,
// h = coord along the axis). Same decomposition AxisFrame::decompose does in 3D,
// specialised to a z=0 profile point.
inline void rh(const AxisFrame& f, double x, double y, double& r, double& h) noexcept {
  f.decompose(xy(x, y, 0.0), r, h);
}

// A sphere band swept by an arc segment. The arc's circle centre is on the axis at
// axial coord `hc`; `R` is the arc (== sphere) radius; the arc runs latitude
// v0→v1 (v = asin((h−hc)/R), the sphere's v-parameter). One angular span [t0,t1].
// Corners a0=(v0,t0) a1=(v1,t0) b1=(v1,t1) b0=(v0,t1); boundary = meridian(a0→a1),
// arc rim at v1 (a1→b1), meridian(b1→b0), arc rim at v0 (b0→a0). A pole (v=±π/2)
// collapses its angular copies to one apex.
inline topo::Shape sphereBandPatch(const AxisFrame& f, double hc, double R, double v0, double v1,
                                   double t0, double t1, topo::Orientation orient) {
  topo::FaceSurface surf;
  surf.kind = topo::FaceSurface::Kind::Sphere;
  surf.frame = math::Ax3{f.origin + f.z.vec() * hc, f.x, f.y, f.z};
  surf.radius = R;
  const math::Sphere sph{surf.frame, R};

  constexpr double kHalfPi = kFullTurn / 4.0;  // π/2 (a sphere latitude pole)
  auto pole = [&](double v) { return std::fabs(std::fabs(v) - kHalfPi) < kProfileTol; };
  const bool a0Pole = pole(v0), a1Pole = pole(v1);

  const topo::Shape a0 = topo::ShapeBuilder::makeVertex(sph.value(t0, v0));
  const topo::Shape a1 = topo::ShapeBuilder::makeVertex(sph.value(t0, v1));
  const topo::Shape b0 = a0Pole ? a0 : topo::ShapeBuilder::makeVertex(sph.value(t1, v0));
  const topo::Shape b1 = a1Pole ? a1 : topo::ShapeBuilder::makeVertex(sph.value(t1, v1));

  auto meridian = [&](const topo::Shape& p, const topo::Shape& q, double theta, double vp,
                      double vq) {
    return profileEdge(p, q, math::Point3{theta, vp, 0.0}, math::Point3{theta, vq, 0.0});
  };
  // A latitude rim (constant v) as a Circle edge of radius R·cos v about the axis at
  // height hc + R·sin v, with a Line pcurve (u=θ, v const).
  auto rim = [&](const topo::Shape& p, const topo::Shape& q, double v, double s0, double s1) {
    const double rr = R * std::cos(v);
    const double hh = hc + R * std::sin(v);
    return arcEdge(f, p, q, rr, hh, s0, s1, topo::FaceSurface::Kind::Cylinder, v);
  };

  std::vector<topo::Shape> edges;
  edges.push_back(meridian(a0, a1, t0, v0, v1));
  if (!a1Pole) edges.push_back(rim(a1, b1, v1, t0, t1));
  edges.push_back(meridian(b1, b0, t1, v1, v0));
  if (!a0Pole) edges.push_back(rim(b0, a0, v0, t1, t0));
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));
  return topo::ShapeBuilder::makeFace(surf, wire, {}, orient);
}

// Latitude v-parameter of a profile point (r,h) on a sphere centred at axial hc.
inline double sphereLatitude(double hc, double R, double h) noexcept {
  return std::asin(std::clamp((h - hc) / R, -1.0, 1.0));
}

// Sphere-band face orientation: outward normal is radial (away from centre). The
// material-outward side follows the profile winding, as for the line segments.
inline topo::Orientation sphereOrientation(const AxisFrame& f, double hc, double R, double vmid,
                                           double theta, double dr, double dh,
                                           double windingSign) {
  const math::Sphere sph{math::Ax3{f.origin + f.z.vec() * hc, f.x, f.y, f.z}, R};
  const math::Vec3 nat = sph.normal(theta, vmid).vec();
  const double nr = windingSign * dh, nh = windingSign * (-dr);
  const math::Vec3 outward =
      f.x.vec() * (nr * std::cos(theta)) + f.y.vec() * (nr * std::sin(theta)) + f.z.vec() * nh;
  return math::dot(nat, outward) < 0.0 ? topo::Orientation::Reversed : topo::Orientation::Forward;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// build_prism_with_holes — extrude an OUTER polygon with circular/polygon HOLES.
//
// `outerXY` is the outer closed polygon (x,y pairs on z=0). `circleHoles` are
// through-holes kept as TRUE circle edges; `polyHoles` are closed polygon holes
// (each an x,y vertex loop). Returns a NULL Shape on degenerate input so the engine
// falls through to OCCT.
//
// Cognitive complexity: a linear assembler over loops (driver ≤ ~8); the per-loop
// cap/side helpers are isolated in detail:: (each ≤ ~12), flagged.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism_with_holes(const double* outerXY, int outerCount,
                                          const std::vector<CircleHole>& circleHoles,
                                          const std::vector<std::vector<math::Point3>>& polyHoles,
                                          double depth) {
  if (outerXY == nullptr || outerCount < 3) return {};
  detail::Loop outer;
  outer.pts.reserve(static_cast<std::size_t>(outerCount));
  for (int i = 0; i < outerCount; ++i)
    outer.pts.push_back(math::Point3{outerXY[i * 2], outerXY[i * 2 + 1], 0.0});

  std::vector<detail::Loop> holes;
  holes.reserve(circleHoles.size() + polyHoles.size());
  for (const CircleHole& h : circleHoles) {
    detail::Loop l;
    l.isCircle = true;
    l.cx = h.cx;
    l.cy = h.cy;
    l.r = h.r;
    holes.push_back(std::move(l));
  }
  for (const std::vector<math::Point3>& poly : polyHoles) {
    detail::Loop l;
    l.isCircle = false;
    l.pts = poly;
    holes.push_back(std::move(l));
  }
  return detail::assembleHoledPrism(std::move(outer), std::move(holes), depth);
}

// ─────────────────────────────────────────────────────────────────────────────
// build_prism_profile — extrude a TYPED outer profile (line/arc/full-circle) with
// circular + polygon holes. Each kind-1 ARC edge becomes a TRUE Circle cap edge +
// a Cylinder side wall; a lone kind-2 full circle becomes a solid cylinder. kind-3
// spline (or a stray mid-loop kind-2) → NULL (engine falls through to OCCT).
// `deflection` is accepted for API stability (the cap outline now follows the true
// Circle edge via the shared edge discretization, so it is unused for sizing here).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_prism_profile(const std::vector<ProfileSegment>& segs,
                                       const std::vector<CircleHole>& circleHoles,
                                       const std::vector<std::vector<math::Point3>>& polyHoles,
                                       double depth, double deflection = 0.05) {
  (void)deflection;
  // Collect the holes once (shared by both the full-circle and typed paths).
  std::vector<detail::Loop> holes;
  holes.reserve(circleHoles.size() + polyHoles.size());
  for (const CircleHole& h : circleHoles)
    holes.push_back(detail::Loop{true, h.cx, h.cy, h.r, {}, false});
  for (const std::vector<math::Point3>& poly : polyHoles)
    holes.push_back(detail::Loop{false, 0, 0, 0, poly, false});

  // A lone kind-2 full circle → a solid cylinder (a closed Circle cap edge + one
  // Cylinder wall), via the Loop machinery.
  if (segs.size() == 1 && segs[0].kind == 2) {
    detail::Loop outer{true, segs[0].cx, segs[0].cy, segs[0].r, {}, false};
    return detail::assembleHoledPrism(std::move(outer), std::move(holes), depth);
  }
  // Otherwise a typed line/arc outer loop → true Cylinder walls for arc edges.
  std::optional<std::vector<detail::TypedEdge>> edges = detail::resolveTypedOuter(segs);
  if (!edges) return {};  // spline / stray circle / empty → OCCT fallthrough
  return detail::assembleTypedProfileSolid(std::move(*edges), std::move(holes), depth);
}

// ─────────────────────────────────────────────────────────────────────────────
// build_revolution_profile — revolve a TYPED profile about an in-plane axis.
//
// Line (kind 0) segments classify to Plane / Cylinder / Cone (as build_revolution).
// An ARC (kind 1) segment whose circle centre lies ON the axis sweeps a SPHERE
// band. DEFERRED (returns NULL so the engine falls through to OCCT, never faked):
//   * an arc whose circle centre is OFF the axis (a TORUS — no native Torus surface);
//   * any kind-2 stray full circle or kind-3 spline segment.
// A full 2π closes the shell; a partial angle adds two planar meridian caps.
//
// Cognitive complexity: the driver dispatches per segment (≤ ~14, flagged); the
// per-surface patch assembly is isolated in detail:: (sweptPatch / sphereBandPatch).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape build_revolution_profile(const std::vector<ProfileSegment>& segs,
                                            const RevolveAxis& axisIn, double angle) {
  if (segs.empty() || !(angle > kMinDepth)) return {};
  const math::Dir3 adir{axisIn.adx, axisIn.ady, 0.0};
  if (!adir.valid()) return {};

  // Reject deferred segment kinds up front (spline / stray full circle).
  for (const ProfileSegment& s : segs)
    if (s.kind == 2 || s.kind == 3) return {};

  detail::AxisFrame f;
  f.origin = detail::xy(axisIn.ax, axisIn.ay, 0.0);
  f.z = adir;
  f.x = math::Dir3{math::Vec3{axisIn.ady, -axisIn.adx, 0.0}};
  f.y = math::Dir3{math::cross(f.z.vec(), f.x.vec())};

  const bool full = angle >= kFullTurn - 1e-9;
  const double sweep = full ? kFullTurn : angle;

  // Profile winding in the (r,h) half-plane (endpoints of every segment).
  double area2 = 0.0;
  for (const ProfileSegment& s : segs) {
    double r0, h0, r1, h1;
    detail::rh(f, s.x0, s.y0, r0, h0);
    detail::rh(f, s.x1, s.y1, r1, h1);
    area2 += r0 * h1 - r1 * h0;
  }
  const double windingSign = area2 >= 0.0 ? 1.0 : -1.0;

  constexpr double kMaxSpan = kFullTurn / 3.0;  // 120° spans (see build_revolution)
  const int nSpan = std::max(1, static_cast<int>(std::ceil(sweep / kMaxSpan - 1e-9)));
  std::vector<double> stations(static_cast<std::size_t>(nSpan) + 1);
  for (int k = 0; k <= nSpan; ++k) stations[k] = sweep * k / nSpan;

  std::vector<topo::Shape> faces;

  for (const ProfileSegment& s : segs) {
    if (s.kind == 0) {  // ── line segment → Plane / Cylinder / Cone ──
      detail::RevPoint p, q;
      detail::rh(f, s.x0, s.y0, p.r, p.h);
      detail::rh(f, s.x1, s.y1, q.r, q.h);
      p.onAxis = p.r < kProfileTol;
      q.onAxis = q.r < kProfileTol;
      if (p.onAxis && q.onAxis) continue;
      const topo::FaceSurface surf = detail::segmentSurface(f, p, q);
      for (int k = 0; k < nSpan; ++k) {
        const double tMid = 0.5 * (stations[k] + stations[k + 1]);
        const topo::Orientation o = detail::faceOrientation(surf, f, p, q, tMid, windingSign);
        faces.push_back(detail::sweptPatch(surf, f, p, q, stations[k], stations[k + 1], o));
      }
    } else {  // ── arc segment (kind 1) → Sphere band (centre on axis) or defer ──
      double rc, hc;  // arc centre in (r,h)
      detail::rh(f, s.cx, s.cy, rc, hc);
      if (rc > kProfileTol) return {};  // centre off the axis → Torus: defer to OCCT
      const double R = s.r;
      if (!(R > kProfileTol)) return {};
      double r0, h0, r1, h1;
      detail::rh(f, s.x0, s.y0, r0, h0);
      detail::rh(f, s.x1, s.y1, r1, h1);
      const double v0 = detail::sphereLatitude(hc, R, h0);
      const double v1 = detail::sphereLatitude(hc, R, h1);
      const double dr = r1 - r0, dh = h1 - h0;
      const double vmid = 0.5 * (v0 + v1);
      for (int k = 0; k < nSpan; ++k) {
        const double tMid = 0.5 * (stations[k] + stations[k + 1]);
        const topo::Orientation o =
            detail::sphereOrientation(f, hc, R, vmid, tMid, dr, dh, windingSign);
        faces.push_back(
            detail::sphereBandPatch(f, hc, R, v0, v1, stations[k], stations[k + 1], o));
      }
    }
  }

  if (faces.empty()) return {};

  // Partial turn: cap the two open ends with the planar meridian profile face, the
  // profile loop swept to θ = 0 and θ = sweep. Arc segments contribute their
  // tessellated endpoints (the caps are planar polygons, sized by the endpoints).
  if (!full) {
    auto capAt = [&](double theta, const math::Vec3& outward) -> topo::Shape {
      std::vector<topo::Shape> verts;
      auto push = [&](double r, double h) {
        const math::Point3 pt = detail::revolved(f, r, h, theta);
        if (!verts.empty() && math::distance(*topo::pointOf(verts.back()), pt) < kProfileTol) return;
        verts.push_back(topo::ShapeBuilder::makeVertex(pt));
      };
      for (const ProfileSegment& s : segs) {
        double r, h;
        detail::rh(f, s.x0, s.y0, r, h);
        push(r, h);
      }
      double r, h;
      detail::rh(f, segs.back().x1, segs.back().y1, r, h);
      push(r, h);
      if (verts.size() > 1 &&
          math::distance(*topo::pointOf(verts.front()), *topo::pointOf(verts.back())) < kProfileTol)
        verts.pop_back();
      if (verts.size() < 3) return {};
      return detail::planarFace(verts, math::Dir3{outward}, topo::Orientation::Forward);
    };
    const math::Vec3 n0 = f.y.vec();
    const math::Vec3 nS = f.x.vec() * (-std::sin(sweep)) + f.y.vec() * std::cos(sweep);
    const topo::Shape capStart = capAt(0.0, -n0);
    const topo::Shape capEnd = capAt(sweep, nS);
    if (!capStart.isNull()) faces.push_back(capStart);
    if (!capEnd.isNull()) faces.push_back(capEnd);
  }

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace cybercad::native::construct

#endif  // CYBERCAD_NATIVE_CONSTRUCT_PROFILE_H

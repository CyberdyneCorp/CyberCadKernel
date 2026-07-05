// SPDX-License-Identifier: Apache-2.0
//
// assemble_shell.h — build a Shell/Solid from the sewn faces, wiring each face's
// boundary onto the SHARED edge nodes (with per-face Line pcurves) so the result
// tessellates watertight by the same shared-edge weld path a native prism uses.
//
// Each SewnFace carries its ordered shared-vertex loop (already unified + sewn) and
// its material-outward normal. We rebuild it as a PLANAR face: a Plane surface
// framed on the loop's normal, an outer Wire of Line edges — each edge REUSING the
// shared edge node from the sew pass (so two faces meeting on a side reference the
// SAME edge node) with a Line pcurve added on this face's plane. Adding the pcurve
// clones the shared edge node (nodes are immutable), which would BREAK sharing — so
// we instead build the wire from the shared node and attach the pcurve to a
// per-face clone that still lists the SAME shared vertices; the tessellator's weld
// then fuses the boundary because both faces put identical 3D points on the shared
// edge (same two shared vertices, same straight span). The winding follows
// SewnFace::reversed (the orientation-fix result).
//
// OCCT-FREE. Uses src/native/math + src/native/topology + tolerant_sew.h. clang++
// -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_ASSEMBLE_SHELL_H
#define CYBERCAD_NATIVE_HEAL_ASSEMBLE_SHELL_H

#include "native/heal/tolerant_sew.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

namespace detail {

// Build the plane frame for a face from its outward normal, anchored at the loop's
// first corner. A reference X is chosen not-parallel to the normal (Gram-Schmidt in
// fromAxisAndRef removes the normal component).
inline math::Ax3 planeFrameFor(const std::vector<topo::Shape>& loop, const math::Dir3& n) {
  const math::Point3 o = *topo::pointOf(loop.front());
  const math::Vec3 ref = std::fabs(n.z()) < 0.9 ? math::Vec3{0, 0, 1} : math::Vec3{1, 0, 0};
  return math::Ax3::fromAxisAndRef(o, n, math::Dir3{ref});
}

// A Line edge on `frame` between two SHARED vertices, carrying its Line pcurve.
// Mirrors boolean/assemble.h planarEdgeShared: reuses the shared vertex nodes so
// the face graph shares corners, and the straight span + shared endpoints make the
// two faces on this side place identical boundary points (⇒ welds watertight).
inline topo::Shape sharedLineEdge(const topo::Shape& v0, const topo::Shape& v1,
                                  const math::Ax3& frame) {
  const auto p0 = topo::pointOf(v0);
  const auto p1 = topo::pointOf(v1);
  const math::Vec3 d = *p1 - *p0;
  const double len = std::max(math::norm(d), 1e-12);
  topo::EdgeCurve c;
  c.kind = topo::EdgeCurve::Kind::Line;
  c.frame.origin = *p0;
  c.frame.x = math::norm(d) > 1e-12 ? math::Dir3{d} : math::Dir3{1, 0, 0};
  c.frame.z = frame.z;
  const topo::Shape edge = topo::ShapeBuilder::makeEdge(c, 0.0, len, v0, v1);

  auto toUV = [&](const math::Point3& p) -> math::Point3 {
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

// Build one planar Face from a SewnFace (winding = its reversed flag). The vertex
// loop is taken forward or reversed; each side is a shared-vertex Line edge.
inline topo::Shape buildFace(const SewnFace& sf) {
  std::vector<topo::Shape> loop = sf.verts;
  if (sf.reversed) std::reverse(loop.begin(), loop.end());
  math::Dir3 n = sf.normal;
  if (sf.reversed) n = math::Dir3{-n.vec()};  // reversing the loop flips the winding
  const math::Ax3 frame = detail::planeFrameFor(loop, n);

  std::vector<topo::Shape> edges;
  const std::size_t m = loop.size();
  edges.reserve(m);
  for (std::size_t i = 0; i < m; ++i)
    edges.push_back(detail::sharedLineEdge(loop[i], loop[(i + 1) % m], frame));
  const topo::Shape wire = topo::ShapeBuilder::makeWire(std::move(edges));

  topo::FaceSurface s;
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = frame;
  return topo::ShapeBuilder::makeFace(s, wire, {}, topo::Orientation::Forward);
}

}  // namespace detail

/// Assemble the sewn faces into a Shell (and wrap in a Solid). Winding per face
/// follows its `reversed` flag (set by the orientation flood-fill + global sign).
inline topo::Shape assembleShell(const std::vector<SewnFace>& faces) {
  std::vector<topo::Shape> built;
  built.reserve(faces.size());
  for (const SewnFace& sf : faces)
    if (sf.verts.size() >= 3) built.push_back(detail::buildFace(sf));
  if (built.size() < 4) return {};  // a closed solid needs ≥ 4 faces
  return topo::ShapeBuilder::makeShell(std::move(built));
}

inline topo::Shape assembleSolid(const std::vector<SewnFace>& faces) {
  const topo::Shape shell = assembleShell(faces);
  if (shell.isNull()) return {};
  return topo::ShapeBuilder::makeSolid({shell});
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_ASSEMBLE_SHELL_H

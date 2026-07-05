// SPDX-License-Identifier: Apache-2.0
//
// face_soup.h — extract each Face of an input shape as an ORDERED loop of world
// corner points plus its surface outward normal (the working set the healer
// rebuilds onto shared nodes).
//
// The input B-rep is IMMUTABLE (topology nodes cannot be mutated in place), so the
// healer never edits it — it reads each face's outer-wire corner sequence and
// surface, then the sew/rebuild steps construct a NEW graph with shared vertex/edge
// nodes. This header does only the READ: it walks the face's outer wire in edge
// order, emitting the ordered distinct corner points, and captures the face's
// plane frame (the sew step is exact for planar/analytic faces — the first slice's
// in-scope domain; a non-planar face still contributes its corner loop but its
// surface is rebuilt as the best-fit plane, which is why freeform re-approximation
// is out of scope and defers to OCCT).
//
// OCCT-FREE. Uses src/native/math + src/native/topology. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_FACE_SOUP_H
#define CYBERCAD_NATIVE_HEAL_FACE_SOUP_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

/// One face of the soup as an ordered corner loop. The loop is the outer wire's
/// vertices in traversal order (start vertex of each edge, following the edge's
/// cumulative orientation), so a Reversed face yields a reverse-wound loop — which
/// is exactly the signal the orientation flood-fill uses to detect a flipped face.
///
/// `normal` is the loop's OWN Newell normal (derived from the corner winding), NOT
/// the stored surface normal, so winding and normal are coherent BY CONSTRUCTION:
/// the plane frame is built from this normal and flipping the loop flips the
/// normal. Global outward-vs-inward is then a single enclosed-volume sign check on
/// the assembled shell (orient.h / heal.cpp), independent of any per-face surface
/// orientation the (possibly wrong) input carried.
struct FaceLoop {
  std::vector<math::Point3> corners;  ///< ordered (loop, not closed)
  math::Dir3 normal{0, 0, 1};         ///< Newell normal of the loop winding
  bool valid = false;                 ///< false ⇒ could not extract (skip)
};

/// Newell's method: the area-weighted normal of a (possibly non-planar) corner
/// loop, whose direction follows the winding (right-hand rule). Robust for any
/// polygon; degenerate loops (< 3 corners or zero area) yield a default +Z.
inline math::Dir3 newellNormal(const std::vector<math::Point3>& loop) {
  if (loop.size() < 3) return math::Dir3{0, 0, 1};
  math::Vec3 n{0, 0, 0};
  const std::size_t m = loop.size();
  for (std::size_t i = 0; i < m; ++i) {
    const math::Point3& a = loop[i];
    const math::Point3& b = loop[(i + 1) % m];
    n.x += (a.y - b.y) * (a.z + b.z);
    n.y += (a.z - b.z) * (a.x + b.x);
    n.z += (a.x - b.x) * (a.y + b.y);
  }
  return math::isNull(n) ? math::Dir3{0, 0, 1} : math::Dir3{n};
}

namespace detail {

// The stored start vertex point of an edge (child[0], the Forward-stored start),
// world-placed. This is the wire's LOCAL forward traversal start; the face's
// cumulative orientation is applied once at the loop level (see below), which is
// more robust than reversing per-edge — reversing a wire's traversal requires
// reversing the edge ORDER too, not just each edge's endpoints.
inline bool edgeStart(const topo::Shape& edge, math::Point3& start) {
  for (topo::Explorer ex(edge, topo::ShapeType::Vertex); ex.more(); ex.next())
    if (auto p = topo::pointOf(ex.current())) {
      start = *p;
      return true;
    }
  return false;
}

}  // namespace detail

/// Extract the ordered outer-wire corner loop of `face` (world-placed). The loop is
/// read in the wire's stored edge order, then the WHOLE loop is reversed once when
/// the face's cumulative orientation is Reversed — so a flipped face yields a
/// reverse-wound loop (its Newell normal flips), which is exactly the signal
/// orientation-fix repairs. The input's stored surface orientation is deliberately
/// NOT trusted beyond this loop reversal. Returns valid=false for a face with no
/// surface or no boundary. `tol` is unused here (degeneracy is measured in
/// degenerate.h); kept for signature symmetry with the soup pipeline.
inline FaceLoop extractFaceLoop(const topo::Shape& face, [[maybe_unused]] double tol) {
  FaceLoop out;
  const auto srf = topo::surfaceOf(face);
  if (!srf) return out;

  // The outer wire is the first Wire child; capture its cumulative orientation.
  topo::Shape outerWire;
  for (const topo::Shape& child : face.tshape()->children()) {
    const topo::Shape placed = child.located(face.location())
                                   .oriented(topo::composed(face.orientation(), child.orientation()));
    if (placed.type() == topo::ShapeType::Wire) {
      outerWire = placed;
      break;
    }
  }
  if (outerWire.isNull()) return out;

  // Walk edges in STORED order, taking each edge's stored start vertex — the wire's
  // local forward loop. No coincidence collapsing here (degenerate.h counts + drops
  // zero-length sides in one place, so nothing is silently discarded at read).
  std::vector<math::Point3> corners;
  for (const topo::Shape& e : outerWire.tshape()->children()) {
    const topo::Shape edge = e.located(outerWire.location());
    math::Point3 s;
    if (edge.type() == topo::ShapeType::Edge && detail::edgeStart(edge, s)) corners.push_back(s);
  }
  if (corners.empty()) return out;  // no boundary at all → not a face we can read

  // Apply the face's cumulative orientation once, at the loop level.
  if (outerWire.orientation() == topo::Orientation::Reversed)
    std::reverse(corners.begin(), corners.end());

  out.normal = newellNormal(corners);  // winding-coherent normal (see FaceLoop doc)
  out.corners = std::move(corners);
  out.valid = true;
  return out;
}

/// Extract every face of `shape` as a FaceLoop (a face with a surface + at least
/// one boundary corner; degenerate loops are kept so degenerate.h can count them).
inline std::vector<FaceLoop> extractSoup(const topo::Shape& shape, double tol) {
  std::vector<FaceLoop> loops;
  for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next()) {
    FaceLoop fl = extractFaceLoop(ex.current(), tol);
    if (fl.valid) loops.push_back(std::move(fl));
  }
  return loops;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_FACE_SOUP_H

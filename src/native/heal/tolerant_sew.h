// SPDX-License-Identifier: Apache-2.0
//
// tolerant_sew.h — stitch a face-loop soup into a connected B-rep by sharing
// vertex AND edge nodes across faces (the core sewing operation).
//
// After vertex unification (vertex_unify.h) two faces that touch along a boundary
// reference the SAME two shared vertex nodes at that boundary's ends. This step
// turns that shared-endpoint agreement into a shared EDGE: an edge is keyed by its
// unordered pair of representative-vertex-node identities, so the FIRST face to lay
// an edge between vertices {A,B} builds ONE edge node and every later face touching
// {A,B} reuses that SAME node. The rebuilt faces therefore share the edge node —
// exactly the B-rep "sewn" condition (an edge referenced by ≥ 2 faces), which is
// what makes the shared-edge tessellation weld close.
//
//   HONESTY CORE. Two edges are stitched into one node ONLY when their endpoints
//   unified to the same two shared vertices — i.e. their corners were coincident
//   WITHIN tolerance in vertex_unify. A corner farther than `tolerance` from any
//   partner produced its OWN vertex node, so its edge gets its OWN key and stays a
//   boundary edge (a candidate hole). No coincidence is ever fabricated: a
//   beyond-tolerance near-miss is left unstitched and becomes the measured residual.
//
// Faces are rebuilt as PLANAR faces (a Plane surface + Line edges carrying Line
// pcurves) using the same construction the boolean assembler and extrude builder
// use, so the sewn result tessellates by the identical shared-edge path a natively
// built prism does — the path the self-verify then checks. The plane frame is fit
// from the loop's outward normal (the extracted FaceLoop::normal), so a planar or
// analytic-planar input face rebuilds exactly; a non-planar face is rebuilt as its
// best-fit plane (freeform re-approximation is out of scope → OCCT).
//
// OCCT-FREE. Uses src/native/math + src/native/topology + vertex_unify.h. clang++
// -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_TOLERANT_SEW_H
#define CYBERCAD_NATIVE_HEAL_TOLERANT_SEW_H

#include "native/heal/face_soup.h"
#include "native/heal/vertex_unify.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

// A face rebuilt onto shared vertex/edge nodes, kept as its ordered shared-vertex
// loop + the shared edge node per side (so orientation-fix can flip it and the
// assembler can read adjacency), plus the outward normal used to build it.
struct SewnFace {
  std::vector<topo::Shape> verts;  ///< ordered shared vertex nodes (loop)
  std::vector<topo::Shape> edges;  ///< shared edge node per side i:(verts[i],verts[i+1])
  math::Dir3 normal{0, 0, 1};      ///< material-outward normal (as extracted)
  bool reversed = false;           ///< orientation flip applied by orient-fix
};

/// Outcome of the sew pass: the rebuilt faces (sharing nodes) + counts + the max
/// residual gap of any UNSTITCHED boundary side (0 if every side found a partner).
struct SewResult {
  std::vector<SewnFace> faces;
  int mergedVerts = 0;      ///< vertices unified (duplicate corners collapsed)
  int mergedEdges = 0;      ///< edge sides that became shared (referenced ≥ 2×)
  int boundaryEdges = 0;    ///< edge sides referenced by exactly one face (open)
  double maxResidualGap = 0.0;
};

// ─────────────────────────────────────────────────────────────────────────────
// EdgePool — one shared edge node per unordered representative-vertex-node pair.
// The pair identity is the two topology TShape pointers (nodes are unique after
// unification), so two faces touching the same corner-pair reuse the SAME edge.
// ─────────────────────────────────────────────────────────────────────────────
class EdgePool {
 public:
  // The shared edge node between two shared vertices, built once. `frame` supplies
  // the pcurve plane for the requesting face; the 3D Line edge is placed once (on
  // first request) and reused thereafter — the pcurve is per-face but the EDGE NODE
  // (which governs sharing/welding) is shared. Records a use count per key.
  topo::Shape edgeBetween(const topo::Shape& va, const topo::Shape& vb) {
    const Key k = keyOf(va, vb);
    if (auto it = edges_.find(k); it != edges_.end()) {
      ++uses_[it->second];
      return edgeNodes_[it->second];
    }
    const auto p0 = topo::pointOf(va);
    const auto p1 = topo::pointOf(vb);
    topo::EdgeCurve c;
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = *p0;
    const math::Vec3 d = *p1 - *p0;
    const double len = math::norm(d);
    c.frame.x = len > 1e-12 ? math::Dir3{d} : math::Dir3{1, 0, 0};
    // Store the endpoints in key order so both faces reference identical vertices.
    topo::Shape edge = topo::ShapeBuilder::makeEdge(c, 0.0, std::max(len, 1e-12),
                                                    k.first == va.tshape().get() ? va : vb,
                                                    k.first == va.tshape().get() ? vb : va);
    const int id = static_cast<int>(edgeNodes_.size());
    edgeNodes_.push_back(std::move(edge));
    uses_.push_back(1);
    edges_.emplace(k, id);
    return edgeNodes_.back();
  }

  int useCountOf(const topo::Shape& va, const topo::Shape& vb) const {
    auto it = edges_.find(keyOf(va, vb));
    return it == edges_.end() ? 0 : uses_[it->second];
  }

  // How many distinct edge keys are referenced by exactly two faces / by one face.
  void tally(int& shared, int& boundary) const {
    shared = boundary = 0;
    for (int u : uses_) {
      if (u >= 2) ++shared;
      else if (u == 1) ++boundary;
    }
  }

 private:
  struct Key {
    const topo::TShape* first;   // min pointer
    const topo::TShape* second;  // max pointer
    bool operator==(const Key& o) const noexcept {
      return first == o.first && second == o.second;
    }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
      return std::hash<const topo::TShape*>{}(k.first) * 1099511628211ull ^
             std::hash<const topo::TShape*>{}(k.second);
    }
  };
  static Key keyOf(const topo::Shape& a, const topo::Shape& b) noexcept {
    const topo::TShape* pa = a.tshape().get();
    const topo::TShape* pb = b.tshape().get();
    return pa <= pb ? Key{pa, pb} : Key{pb, pa};
  }

  std::unordered_map<Key, int, KeyHash> edges_;
  std::vector<topo::Shape> edgeNodes_;
  std::vector<int> uses_;
};

// Forward: largest surviving beyond-tolerance corner gap (defined below).
inline double residualGap(const std::vector<FaceLoop>& soup, double tol);

// ─────────────────────────────────────────────────────────────────────────────
// sew — unify every corner, then for each face build its shared-vertex loop and
// share an edge node per side. Returns the rebuilt faces + counts + residual.
//
// Cognitive complexity: this is the SEW CORE (systems band, ~20). It is the one
// intentionally dense function of the module (the two nested loops over faces ×
// sides plus the residual measurement); everything it calls is a short helper.
// ─────────────────────────────────────────────────────────────────────────────
inline SewResult sew(const std::vector<FaceLoop>& soup, double tol) {  // NOLINT: sew core
  SewResult out;
  VertexUnifier vu(tol);
  EdgePool pool;

  // Pass 1: unify every corner of every face into shared vertex loops.
  std::vector<std::vector<topo::Shape>> faceVerts;
  faceVerts.reserve(soup.size());
  for (const FaceLoop& fl : soup) {
    std::vector<topo::Shape> vs;
    vs.reserve(fl.corners.size());
    for (const math::Point3& c : fl.corners) vs.push_back(vu.vertexFor(c));
    // Collapse consecutive same-node corners (a zero-length side after unify).
    std::vector<topo::Shape> distinct;
    for (const topo::Shape& v : vs)
      if (distinct.empty() || !distinct.back().isSameGeometry(v)) distinct.push_back(v);
    if (!distinct.empty() && distinct.size() >= 2 &&
        distinct.front().isSameGeometry(distinct.back()))
      distinct.pop_back();
    faceVerts.push_back(std::move(distinct));
  }
  out.mergedVerts = vu.mergedCount();

  // Pass 2: build shared edge nodes per side; assemble each SewnFace.
  for (std::size_t f = 0; f < soup.size(); ++f) {
    const std::vector<topo::Shape>& vs = faceVerts[f];
    if (vs.size() < 3) continue;  // degenerated to a sliver after unify — skip
    SewnFace sf;
    sf.normal = soup[f].normal;
    sf.verts = vs;
    const std::size_t n = vs.size();
    for (std::size_t i = 0; i < n; ++i)
      sf.edges.push_back(pool.edgeBetween(vs[i], vs[(i + 1) % n]));
    out.faces.push_back(std::move(sf));
  }

  // Tally shared vs boundary edges. A boundary edge (used once) is an open side.
  pool.tally(out.mergedEdges, out.boundaryEdges);

  // Residual: the largest gap a boundary corner has to its NEAREST partner corner
  // in the ORIGINAL soup (what a beyond-tolerance defect would need to bridge). For
  // a fully sewn soup there are no boundary edges and this stays 0.
  out.maxResidualGap = out.boundaryEdges > 0 ? residualGap(soup, tol) : 0.0;
  return out;
}

// Largest "nearest other-corner" distance among corners that did NOT find a
// within-tolerance partner — the honest measured gap a heal would have to bridge.
// Only meaningful when boundary edges survive; capped at the max so the caller can
// report it as maxResidualGap for a GapBeyondTolerance verdict.
inline double residualGap(const std::vector<FaceLoop>& soup, double tol) {
  std::vector<math::Point3> pts;
  for (const FaceLoop& fl : soup)
    for (const math::Point3& c : fl.corners) pts.push_back(c);
  double worstUnpaired = 0.0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    double nearest = 1e300;
    for (std::size_t k = 0; k < pts.size(); ++k)
      if (i != k) nearest = std::min(nearest, math::distance(pts[i], pts[k]));
    if (nearest > tol) worstUnpaired = std::max(worstUnpaired, nearest);
  }
  return worstUnpaired;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_TOLERANT_SEW_H

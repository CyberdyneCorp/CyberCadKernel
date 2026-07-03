// SPDX-License-Identifier: Apache-2.0
//
// bsp.h — a Binary Space Partition tree over planar polygons, and the four tree
// operations that implement constructive-solid-geometry set algebra on planar
// polyhedra. This is the algorithmic core of the native boolean (Phase 4 #5).
//
// ── ALGORITHM (clean-room) ────────────────────────────────────────────────────
// Naylor, Amanatides & Thibault, "Merging BSP Trees Yields Polyhedral Set
// Operations" (SIGGRAPH 1990), in the compact form popularised for polygon soups:
//
//   * splitPolygon(plane, poly) classifies a polygon against a plane into four
//     buckets — COPLANAR-FRONT, COPLANAR-BACK, FRONT, BACK — splitting a spanning
//     polygon exactly along the plane (linear interpolation of the crossing edges).
//   * A Node partitions space by the plane of its first polygon; polygons in front
//     go to the `front` subtree, behind to `back`, coplanar ones are kept on the
//     node. build() inserts a batch of polygons, splitting as needed.
//   * clipPolygons(node, polys) drops the parts of `polys` that lie INSIDE the
//     solid the tree represents (behind every leaf), returning the parts OUTSIDE.
//   * invert() flips the solid (front↔back everywhere) — set complement.
//   * clipTo(a, b) removes from `a` all polygon parts inside `b`.
//
// The three booleans compose these (union shown; see boolean.h for cut/common):
//     union(A,B):  A.clipTo(B); B.clipTo(A);
//                  B.invert(); B.clipTo(A); B.invert();   // drop B's coplanar-
//                                                          // coincident overlap
//                  A.build(B.allPolygons());              // A ∪ (B \ A)
//
// Coplanar faces are the classic BOPAlgo hazard; the coplanar-front/back split and
// the invert-clip-invert step handle coincident faces of two boxes sharing a wall
// without leaving a doubled or missing face — verified on axis-aligned box cases.
//
// ── COMPLEXITY (systems band, flagged) ───────────────────────────────────────
// splitPolygon and build/clip recursion are irreducible geometry. splitPolygon is
// ~18 (a per-vertex classify + span split); build/clip are simple recursions ≤ ~10.
// The Node is heap-linked (unique_ptr children) so a deep partition does not blow
// the value semantics; trees here are shallow (a box is ≤ 6 planes).
//
// CLEAN-ROOM. Uses src/native/math + polygon.h. No OCCT. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_BSP_H
#define CYBERCAD_NATIVE_BOOLEAN_BSP_H

#include "native/boolean/polygon.h"
#include "native/math/native_math.h"

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

namespace cybercad::native::boolean {

// Classification of a point / polygon against a splitting plane.
enum : std::uint8_t { kCoplanar = 0, kFront = 1, kBack = 2, kSpanning = 3 };

// ─────────────────────────────────────────────────────────────────────────────
// splitPolygon — classify `poly` against `plane` and route its pieces into the
// four output bags. If the polygon spans the plane it is cut into a front piece
// and a back piece (vertices on the crossing edges interpolated onto the plane).
//
// COPLANAR polygons go to coplanarFront/coplanarBack by whether their own normal
// agrees with the splitting plane's normal — this is what lets two coincident
// faces be told apart (same wall, opposite materials).
//
// systems-band (~18): a per-vertex side classify, then a single loop that emits
// the front/back sub-loops. Isolated + documented per the complexity policy.
// ─────────────────────────────────────────────────────────────────────────────
inline void splitPolygon(const Plane& plane, const Polygon& poly,
                         std::vector<Polygon>& coplanarFront,
                         std::vector<Polygon>& coplanarBack, std::vector<Polygon>& front,
                         std::vector<Polygon>& back) {
  const std::size_t n = poly.size();
  // Per-vertex side and the aggregate polygon type.
  std::vector<std::uint8_t> side(n);
  std::uint8_t polyType = 0;
  for (std::size_t i = 0; i < n; ++i) {
    const double d = plane.signedDistance(poly.vertices[i]);
    const std::uint8_t s = d < -kPlaneEps ? kBack : d > kPlaneEps ? kFront : kCoplanar;
    side[i] = s;
    polyType |= s;
  }

  switch (polyType) {
    case kCoplanar: {
      // The polygon lies in the plane: sort by normal agreement.
      (math::dot(plane.normal, poly.plane.normal) > 0.0 ? coplanarFront : coplanarBack)
          .push_back(poly);
      break;
    }
    case kFront:
      front.push_back(poly);
      break;
    case kBack:
      back.push_back(poly);
      break;
    default: {  // kSpanning — cut the loop along the plane.
      std::vector<math::Point3> f, b;
      f.reserve(n + 1);
      b.reserve(n + 1);
      for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        const std::uint8_t si = side[i], sj = side[j];
        const math::Point3& vi = poly.vertices[i];
        const math::Point3& vj = poly.vertices[j];
        if (si != kBack) f.push_back(vi);
        if (si != kFront) b.push_back(vi);
        if ((si | sj) == kSpanning) {  // edge crosses the plane
          const double di = plane.signedDistance(vi);
          const double dj = plane.signedDistance(vj);
          const double t = di / (di - dj);
          const math::Point3 v{vi.x + (vj.x - vi.x) * t, vi.y + (vj.y - vi.y) * t,
                               vi.z + (vj.z - vi.z) * t};
          f.push_back(v);
          b.push_back(v);
        }
      }
      if (f.size() >= 3) front.emplace_back(std::move(f), poly.plane);
      if (b.size() >= 3) back.emplace_back(std::move(b), poly.plane);
      break;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Node — a BSP node. `plane` (once set) partitions space; `polygons` are the
// polygons coplanar with the node's plane; front/back are the sub-solids.
// ─────────────────────────────────────────────────────────────────────────────
class Node {
 public:
  Node() = default;
  explicit Node(std::vector<Polygon> polys) { build(std::move(polys)); }

  /// Insert polygons, splitting them by this node's plane (the first insert also
  /// chooses the plane). Recurses into front/back subtrees.
  void build(std::vector<Polygon> polys) {
    if (polys.empty()) return;
    if (!hasPlane_) {
      plane_ = polys.front().plane;
      hasPlane_ = true;
    }
    std::vector<Polygon> frontP, backP;
    for (const Polygon& p : polys)
      splitPolygon(plane_, p, polygons_, polygons_, frontP, backP);
    if (!frontP.empty()) {
      if (!front_) front_ = std::make_unique<Node>();
      front_->build(std::move(frontP));
    }
    if (!backP.empty()) {
      if (!back_) back_ = std::make_unique<Node>();
      back_->build(std::move(backP));
    }
  }

  /// Return the parts of `polys` that lie OUTSIDE the solid this tree represents.
  /// Parts behind every leaf (fully inside) are dropped.
  std::vector<Polygon> clipPolygons(const std::vector<Polygon>& polys) const {
    if (!hasPlane_) return polys;  // an empty node clips nothing
    std::vector<Polygon> frontP, backP;
    for (const Polygon& p : polys)
      // Coplanar polygons are routed with the FRONT/back they agree with, so a
      // coincident face is kept or dropped consistently with a spanning one.
      splitPolygon(plane_, p, frontP, backP, frontP, backP);

    std::vector<Polygon> result = front_ ? front_->clipPolygons(frontP) : std::move(frontP);
    if (back_) {
      std::vector<Polygon> clippedBack = back_->clipPolygons(backP);
      result.insert(result.end(), clippedBack.begin(), clippedBack.end());
    }
    // No back subtree ⇒ backP is INSIDE the solid ⇒ dropped (not appended).
    return result;
  }

  /// Remove from this tree every polygon part that lies inside `other`.
  void clipTo(const Node& other) {
    polygons_ = other.clipPolygons(polygons_);
    if (front_) front_->clipTo(other);
    if (back_) back_->clipTo(other);
  }

  /// Set complement: flip every polygon and swap front/back everywhere.
  void invert() {
    for (Polygon& p : polygons_) p.flip();
    plane_.normal = -plane_.normal;
    plane_.w = -plane_.w;
    if (front_) front_->invert();
    if (back_) back_->invert();
    std::swap(front_, back_);
  }

  /// All polygons in the tree (this node + subtrees), flattened.
  std::vector<Polygon> allPolygons() const {
    std::vector<Polygon> out = polygons_;
    if (front_) {
      std::vector<Polygon> f = front_->allPolygons();
      out.insert(out.end(), f.begin(), f.end());
    }
    if (back_) {
      std::vector<Polygon> b = back_->allPolygons();
      out.insert(out.end(), b.begin(), b.end());
    }
    return out;
  }

 private:
  Plane plane_{};
  bool hasPlane_ = false;
  std::vector<Polygon> polygons_;
  std::unique_ptr<Node> front_;
  std::unique_ptr<Node> back_;
};

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_BSP_H

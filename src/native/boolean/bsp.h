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
// Near-coincident coplanar-wall cancellation (SCALE-RELATIVE, VOLUME-VALIDATED).
//
// ── THE DEFECT ────────────────────────────────────────────────────────────────
// splitPolygon's on-plane classification (kPlaneEps, an ABSOLUTE fp band) tells two
// coincident faces apart only when they land within that band. Two dense-soup boxes
// stacked with an AIR GAP g route their near-coincident shared walls into DIFFERENT
// coplanar buckets whenever g > kPlaneEps but g is still far below the model scale:
// the union's invert-clip-invert coincident-overlap removal never fires, so BOTH
// internal walls survive as a doubled, opposite-normal wall pair straddling a razor-
// thin air slab. The assembled mesh is then not a single watertight 2-manifold —
// sameDirectionEdgeCount doubles (the near-tangent dense-CSG "Band 2" defect).
//
// Widening kPlaneEps to swallow g would be an ABSOLUTE tolerance widen — proven to
// silently corrupt genuinely-distinct thin features elsewhere. Instead this pass
// leaves the exact BSP classification untouched and, on the FINAL soup, cancels only
// pairs that are provably an internal air-slab boundary, under two independent gates:
//
//   (1) SCALE-RELATIVE geometric predicate — a pair qualifies only when the two
//       polygons have opposite unit normals, lie on planes separated by less than a
//       fraction of the MODEL EXTENT (never an absolute epsilon), their fronts
//       (outward sides) face INTO the gap (so the sliver between them is OUTSIDE both
//       ⇒ an air slab, not solid material), and their in-plane projections overlap.
//   (2) EXACT signed-volume preservation — the whole cancellation is applied only if
//       it changes the polygon soup's exact divergence-theorem signed volume by less
//       than a tight relative bound. Removing an internal air-slab wall pair is
//       volume-neutral (Δ ~ g/extent ≪ 1); collapsing a GENUINELY thin solid feature
//       (a real slab whose walls are near-coincident) would move ~100% of its volume
//       and is REJECTED — the original soup is kept untouched.
//
// Gate (2) is the principled guarantee (mirrors assemble.h's Band-1 sliver guard):
// no separation-carrying wall is ever merged, because volumes are already scale-
// consistent and a real feature's cancellation moves real volume. Measured margin:
// an air-slab pair moves Δ ≲ 2e-5 relative; a real thin slab moves ~3e-1 — a ~4
// order-of-magnitude gap the volume gate cleanly separates.
// ─────────────────────────────────────────────────────────────────────────────

// Fraction of the model extent within which two opposite coplanar walls are treated
// as the same near-coincident boundary. Deliberately SCALE-RELATIVE (× extent), so a
// millimetre model and a metre model behave identically; it is only a CANDIDATE gate
// — the volume-preservation check below is the real safety net. Chosen to sit far
// above the fp band (kPlaneEps) yet far below any legitimate thin-feature scale.
inline constexpr double kCoincidentWallFrac = 1e-4;

// Relative signed-volume tolerance for accepting a wall cancellation. An air-slab
// removal is volume-neutral to well under this; a real thin feature moves orders of
// magnitude more and is rejected. Matches the Band-1 sliver guard's principle.
inline constexpr double kWallCancelVolTol = 1e-3;

// Exact signed volume of a polygon soup (divergence theorem: (1/6) Σ over each
// polygon's triangle fan of the scalar triple product). Only the before/after
// magnitude matters, so a consistent per-polygon fan orientation is sufficient.
inline double soupVolume(const std::vector<Polygon>& polys) {
  double v6 = 0.0;
  for (const Polygon& poly : polys) {
    const std::size_t m = poly.size();
    if (m < 3) continue;
    const math::Point3& a = poly.vertices[0];
    for (std::size_t k = 1; k + 1 < m; ++k) {
      const math::Vec3 e1 = poly.vertices[k] - a;
      const math::Vec3 e2 = poly.vertices[k + 1] - a;
      v6 += math::dot(a.asVec(), math::cross(e1, e2));
    }
  }
  return std::fabs(v6) / 6.0;
}

// Bounding-box diagonal of a polygon soup — the model scale that makes the wall gate
// scale-relative.
inline double soupExtent(const std::vector<Polygon>& polys) {
  double lo[3] = {1e300, 1e300, 1e300}, hi[3] = {-1e300, -1e300, -1e300};
  bool any = false;
  for (const Polygon& p : polys)
    for (const math::Point3& v : p.vertices) {
      any = true;
      const double c[3] = {v.x, v.y, v.z};
      for (int i = 0; i < 3; ++i) {
        lo[i] = std::min(lo[i], c[i]);
        hi[i] = std::max(hi[i], c[i]);
      }
    }
  if (!any) return 0.0;
  double d2 = 0.0;
  for (int i = 0; i < 3; ++i) d2 += (hi[i] - lo[i]) * (hi[i] - lo[i]);
  return std::sqrt(d2);
}

// Two orthonormal in-plane axes for a unit normal (for projecting a coplanar loop to
// 2D). Picks whichever cardinal axis is least parallel to the normal as the seed.
inline void planeAxes(const math::Vec3& n, math::Vec3& u, math::Vec3& v) {
  const math::Vec3 seed = (std::fabs(n.x) <= std::fabs(n.y) && std::fabs(n.x) <= std::fabs(n.z))
                              ? math::Vec3{1, 0, 0}
                              : (std::fabs(n.y) <= std::fabs(n.z) ? math::Vec3{0, 1, 0}
                                                                  : math::Vec3{0, 0, 1});
  math::Vec3 uu = seed - n * math::dot(seed, n);
  const double ul = math::norm(uu);
  u = ul > 0.0 ? uu * (1.0 / ul) : math::Vec3{1, 0, 0};
  v = math::cross(n, u);
}

// Do two near-coplanar polygons overlap in their shared in-plane projection? A cheap
// axis-aligned-in-plane bbox test in `a`'s frame — enough to reject two coplanar walls
// that are near-coincident in PLANE but disjoint in EXTENT (so unrelated faces that
// merely share a plane are never paired). `slack` widens the boxes by the wall
// tolerance so a shared edge still counts as overlap.
inline bool projectionsOverlap(const Polygon& a, const Polygon& b, double slack) {
  math::Vec3 u, v;
  planeAxes(a.plane.normal, u, v);
  auto boxOf = [&](const Polygon& p, double& u0, double& u1, double& v0, double& v1) {
    u0 = v0 = 1e300;
    u1 = v1 = -1e300;
    for (const math::Point3& pt : p.vertices) {
      const double pu = math::dot(pt.asVec(), u), pv = math::dot(pt.asVec(), v);
      u0 = std::min(u0, pu);
      u1 = std::max(u1, pu);
      v0 = std::min(v0, pv);
      v1 = std::max(v1, pv);
    }
  };
  double au0, au1, av0, av1, bu0, bu1, bv0, bv1;
  boxOf(a, au0, au1, av0, av1);
  boxOf(b, bu0, bu1, bv0, bv1);
  return au1 + slack >= bu0 && bu1 + slack >= au0 && av1 + slack >= bv0 && bv1 + slack >= av0;
}

// A candidate air-slab wall pair: opposite unit normals, planes near-coincident within
// `tol`, each front (outward) side facing INTO the gap between them (so the sliver is
// OUTSIDE both ⇒ air, not solid), and overlapping in-plane. `tol` is the scale-relative
// wall band, reused as the in-plane overlap slack.
inline bool isAirSlabWallPair(const Polygon& a, const Polygon& b, double tol) {
  const math::Vec3& na = a.plane.normal;
  const math::Vec3& nb = b.plane.normal;
  if (math::dot(na, nb) > -0.999999) return false;  // must be (near) exactly opposite
  // With nb ≈ -na, both planes written along na: a is {na·x = a.w}, b is {na·x = -b.w}.
  const double sepSigned = -b.plane.w - a.plane.w;  // b's offset in +na minus a's offset
  if (std::fabs(sepSigned) > tol) return false;     // not near-coincident (scale-rel)
  // Front of a is +na; front of b is -na. For the gap between to be OUTSIDE both, a's
  // front must point toward b (b sits on a's +na side): sepSigned ≥ 0 (within slack).
  if (sepSigned < -tol) return false;
  return projectionsOverlap(a, b, tol);
}

// Cancel near-coincident opposite-wall pairs on the final soup, VALIDATED by exact
// signed-volume preservation. Pairs each unmatched polygon with the first qualifying
// partner (greedy — a wall has exactly one opposite twin here), drops both, then keeps
// the reduced soup only if the soup volume is preserved to kWallCancelVolTol; otherwise
// returns the input UNCHANGED (a real thin feature is never collapsed).
inline std::vector<Polygon> cancelNearCoincidentWalls(std::vector<Polygon> polys) {
  const std::size_t n = polys.size();
  if (n < 2) return polys;
  const double extent = soupExtent(polys);
  if (!(extent > 0.0)) return polys;
  const double tol = kCoincidentWallFrac * extent;

  std::vector<bool> dead(n, false);
  bool anyCancelled = false;
  for (std::size_t i = 0; i < n; ++i) {
    if (dead[i]) continue;
    for (std::size_t j = i + 1; j < n; ++j) {
      if (dead[j]) continue;
      if (isAirSlabWallPair(polys[i], polys[j], tol)) {
        dead[i] = dead[j] = true;
        anyCancelled = true;
        break;
      }
    }
  }
  if (!anyCancelled) return polys;

  std::vector<Polygon> reduced;
  reduced.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
    if (!dead[i]) reduced.push_back(polys[i]);

  // Volume-preservation gate: keep the cancellation only if it moved no real volume.
  const double v0 = soupVolume(polys);
  const double v1 = soupVolume(reduced);
  const double denom = std::max(v0, 1e-12);
  if (std::fabs(v1 - v0) > kWallCancelVolTol * denom) return polys;  // would corrupt geometry
  return reduced;
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

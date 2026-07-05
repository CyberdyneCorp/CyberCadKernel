// SPDX-License-Identifier: Apache-2.0
//
// vertex_unify.h — merge near-coincident B-rep vertices onto ONE shared node.
//
// This is the `boolean/assemble.h` VertexPool spatial hash, GENERALIZED from the
// axis-aligned boolean-corner case to arbitrary imported B-rep boundary vertices.
// A face soup carries an INDEPENDENT copy of every shared corner (each face built
// its own vertex node); those copies sit within the exporter's write tolerance but
// are topologically distinct, so nothing is shared and the shell cannot close.
// Unifying them onto one representative Vertex node per spatial cell is the
// PREREQUISITE for edge merging: two edges can be "the same" only if their
// endpoints already collapsed to the same two shared vertices.
//
// The hash quantizes each point to a cell of side `tolerance` (nearest-cell
// rounding, so points on either side of a cell boundary that are within tolerance
// still land together). A cell's first occupant is the representative; every later
// coincident point maps to it. Two points MORE than `tolerance` apart are never
// merged (they fall in different cells — the honesty guarantee: no fabricated
// coincidence).
//
// Because the hash is strictly single-cell, a pair separated by slightly less than
// `tolerance` can straddle a boundary and miss; a snap-to-representative probe of
// the 26 neighbouring cells closes that gap (still bounded by `tolerance`), so a
// sub-tolerance jitter merges regardless of where it lands relative to the grid.
//
// OCCT-FREE. Uses src/native/math + src/native/topology. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_VERTEX_UNIFY_H
#define CYBERCAD_NATIVE_HEAL_VERTEX_UNIFY_H

#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;
namespace math = cybercad::native::math;

// ─────────────────────────────────────────────────────────────────────────────
// VertexUnifier — map a world-space point to a shared representative Vertex node
// within `tolerance`. Reports how many merges happened (input count − distinct
// representatives) via mergedCount().
// ─────────────────────────────────────────────────────────────────────────────
class VertexUnifier {
 public:
  explicit VertexUnifier(double tolerance) noexcept
      : tol_(tolerance > 0 ? tolerance : 1e-12), inv_(1.0 / (tolerance > 0 ? tolerance : 1e-12)) {}

  /// Return the shared Vertex node for `p`: an existing representative within
  /// `tolerance`, or a new one if none. Counts a merge whenever an incoming point
  /// reuses an existing representative.
  topo::Shape vertexFor(const math::Point3& p) {
    ++nRequested_;
    if (const int rep = findRepresentative(p); rep >= 0) {
      ++nMerged_;
      return verts_[static_cast<std::size_t>(rep)];
    }
    const int id = static_cast<int>(verts_.size());
    verts_.push_back(topo::ShapeBuilder::makeVertex(p, tol_));
    points_.push_back(p);
    cells_.emplace(cellOf(p), id);
    return verts_.back();
  }

  /// Number of input points that collapsed onto an existing representative
  /// (== mergedVerts: how many duplicate corners were unified).
  int mergedCount() const noexcept { return nMerged_; }
  /// Distinct shared vertices produced.
  int distinctCount() const noexcept { return static_cast<int>(verts_.size()); }
  int requestedCount() const noexcept { return nRequested_; }

 private:
  struct Cell {
    long long x, y, z;
    bool operator==(const Cell& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct CellHash {
    std::size_t operator()(const Cell& c) const noexcept {
      std::size_t h = static_cast<std::size_t>(c.x) * 73856093u;
      h ^= static_cast<std::size_t>(c.y) * 19349663u;
      h ^= static_cast<std::size_t>(c.z) * 83492791u;
      return h;
    }
  };
  static long long lr(double v) noexcept {
    return static_cast<long long>(v >= 0 ? v + 0.5 : v - 0.5);
  }
  Cell cellOf(const math::Point3& p) const noexcept {
    return Cell{lr(p.x * inv_), lr(p.y * inv_), lr(p.z * inv_)};
  }

  // Search this point's cell and its 26 neighbours for an existing representative
  // within `tolerance` (Euclidean). Returns its id or -1. The neighbour scan makes
  // the merge robust to a jitter that straddles a cell boundary without widening
  // the merge radius beyond `tolerance` (the exact-distance test gates it).
  int findRepresentative(const math::Point3& p) const {
    const Cell base = cellOf(p);
    int best = -1;
    double bestD2 = tol_ * tol_;
    for (long long dx = -1; dx <= 1; ++dx)
      for (long long dy = -1; dy <= 1; ++dy)
        for (long long dz = -1; dz <= 1; ++dz) {
          const Cell c{base.x + dx, base.y + dy, base.z + dz};
          auto it = cells_.find(c);
          if (it == cells_.end()) continue;
          const double d2 = math::normSquared(points_[static_cast<std::size_t>(it->second)] - p);
          if (d2 <= bestD2) {
            bestD2 = d2;
            best = it->second;
          }
        }
    return best;
  }

  double tol_;
  double inv_;
  std::unordered_map<Cell, int, CellHash> cells_;  // one representative per occupied cell
  std::vector<topo::Shape> verts_;
  std::vector<math::Point3> points_;
  int nRequested_ = 0;
  int nMerged_ = 0;
};

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_VERTEX_UNIFY_H

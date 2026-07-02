// SPDX-License-Identifier: Apache-2.0
//
// explore.h — deterministic traversal of the topology graph.
//
// Provides, mirroring TopExp / TopTools conventions:
//   * Explorer     — visits every sub-shape of a requested ShapeType in a
//                    stable depth-first order, optionally skipping sub-shapes
//                    contained in an "avoid" type. Cumulates orientation and
//                    location down the graph so each Current() is world-placed.
//                    Matches TopExp_Explorer.
//   * ShapeMap     — an insertion-ordered, deduplicated set of Shapes with
//                    STABLE 1-BASED ids, deduped by isSame (node+location,
//                    orientation-independent). Matches
//                    TopTools_IndexedMapOfShape + TopExp::MapShapes ordering.
//   * AncestryMap  — sub-shape → list of parent shapes (e.g. edge → faces).
//                    Matches TopTools_IndexedDataMapOfShapeListOfShape built by
//                    TopExp::MapShapesAndAncestors.
//
// Ordering guarantee (must match the oracle): MapShapes / Explorer visit in the
// order produced by a pre-order DFS that, at each node, emits the node itself
// (if it matches) then recurses into children left-to-right. This is exactly
// TopExp_Explorer's traversal, so sampled ids line up with OCCT on the sim.
//
// OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TOPOLOGY_EXPLORE_H
#define CYBERCAD_NATIVE_TOPOLOGY_EXPLORE_H

#include "native/topology/shape.h"

#include <cstddef>
#include <functional>
#include <unordered_map>
#include <vector>

namespace cybercad::native::topology {

// ─────────────────────────────────────────────────────────────────────────────
// Explorer — depth-first visitor of sub-shapes of one ShapeType.
//
// Usage mirrors TopExp_Explorer:
//   for (Explorer ex(shape, ShapeType::Face); ex.more(); ex.next())
//     process(ex.current());
//
// Deduplication: a given (node+location) is visited once even if it is shared
// by several parents (deduped by isSame), matching the explorer's behaviour of
// not re-emitting a shared sub-shape.
// ─────────────────────────────────────────────────────────────────────────────
class Explorer {
 public:
  /// Explore `root` for shapes of `toFind`, skipping any that lie inside a
  /// sub-shape of `toAvoid`. If `toAvoid` is equal-or-less complex than
  /// `toFind`, it has no effect (matches TopExp semantics). Pass toAvoid == the
  /// same as toFind to disable avoidance (the default).
  Explorer(const Shape& root, ShapeType toFind, ShapeType toAvoid) {
    collect(root, toFind, toAvoid, /*insideAvoid=*/false);
    index_ = 0;
  }
  Explorer(const Shape& root, ShapeType toFind)
      : Explorer(root, toFind, toFind) {}

  bool more() const noexcept { return index_ < found_.size(); }
  void next() noexcept { ++index_; }
  const Shape& current() const noexcept { return found_[index_]; }

  /// All matches at once (already in traversal order, deduped).
  const std::vector<Shape>& all() const noexcept { return found_; }

 private:
  // Depth-first pre-order collection with orientation+location cumulation.
  void collect(const Shape& s, ShapeType toFind, ShapeType toAvoid, bool insideAvoid) {
    if (s.isNull()) return;
    const ShapeType t = s.type();

    // Emit this node if it is the sought type and not shadowed by an avoid.
    if (t == toFind && !insideAvoid) {
      emit(s);
      // A found node is a leaf of the search — do not descend further for the
      // same type (its own sub-shapes of that type cannot exist by ordering).
      return;
    }

    // Determine whether descending crosses into an avoided region. `toAvoid`
    // only bites when it is strictly more complex than `toFind`.
    const bool nowInside =
        insideAvoid || (t == toAvoid && isMoreComplex(toAvoid, toFind));

    for (const Shape& child : s.tshape()->children()) {
      // Cumulate placement (world = parent ∘ child) and orientation.
      const Shape placed = child
                               .located(s.location())
                               .oriented(composed(s.orientation(), child.orientation()));
      collect(placed, toFind, toAvoid, nowInside);
    }
  }

  void emit(const Shape& s) {
    for (const Shape& e : found_)
      if (e.isSame(s)) return;  // dedup shared sub-shapes
    found_.push_back(s);
  }

  std::vector<Shape> found_;
  std::size_t index_ = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// ShapeMap — insertion-ordered, deduplicated (by isSame) set with 1-based ids.
// Mirrors TopTools_IndexedMapOfShape; MapShapes(root, type) fills it in explorer
// order.
// ─────────────────────────────────────────────────────────────────────────────
class ShapeMap {
 public:
  /// Add a shape; returns its 1-based id. If an isSame shape is already present
  /// its existing id is returned (no duplicate insert).
  int add(const Shape& s) {
    if (int existing = findIndex(s)) return existing;
    entries_.push_back(s);
    const int id = static_cast<int>(entries_.size());
    keyToIndex_.emplace(Key{s.tshape().get(), s.location()}, id);
    return id;
  }

  /// 1-based id of a shape (by isSame), or 0 if absent.
  int findIndex(const Shape& s) const {
    auto range = keyToIndex_.equal_range(Key{s.tshape().get(), s.location()});
    for (auto it = range.first; it != range.second; ++it)
      if (entries_[static_cast<std::size_t>(it->second) - 1].isSame(s)) return it->second;
    return 0;
  }

  bool contains(const Shape& s) const { return findIndex(s) != 0; }

  /// 1-based lookup. id must be in [1, size()].
  const Shape& shape(int id) const noexcept { return entries_[static_cast<std::size_t>(id) - 1]; }

  std::size_t size() const noexcept { return entries_.size(); }
  bool empty() const noexcept { return entries_.empty(); }
  const std::vector<Shape>& shapes() const noexcept { return entries_; }

 private:
  // Hash key: node pointer only (location collisions resolved by linear probe in
  // the small bucket). Keeps the hash cheap while isSame stays authoritative.
  struct Key {
    const TShape* node;
    Location loc;
    bool operator==(const Key& o) const noexcept { return node == o.node && loc == o.loc; }
  };
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
      return std::hash<const TShape*>{}(k.node);
    }
  };

  std::vector<Shape> entries_;
  std::unordered_multimap<Key, int, KeyHash> keyToIndex_;
};

/// Fill `out` with every sub-shape of `type` in `root`, in explorer order, with
/// stable 1-based ids. Mirrors TopExp::MapShapes.
inline void mapShapes(const Shape& root, ShapeType type, ShapeMap& out) {
  for (Explorer ex(root, type); ex.more(); ex.next()) out.add(ex.current());
}

/// Convenience: build and return a ShapeMap of `type` in `root`.
inline ShapeMap mapShapes(const Shape& root, ShapeType type) {
  ShapeMap m;
  mapShapes(root, type, m);
  return m;
}

// ─────────────────────────────────────────────────────────────────────────────
// AncestryMap — sub-shape → its parents. Built like
// TopExp::MapShapesAndAncestors: for each ancestor of type `ancestorType`, every
// sub-shape of type `subType` it contains gets that ancestor appended to its
// parent list. Sub-shapes are keyed with stable 1-based ids (by isSame).
//
// Example: ancestry(solid, Edge, Face) → each edge maps to the faces using it.
// ─────────────────────────────────────────────────────────────────────────────
class AncestryMap {
 public:
  AncestryMap(const Shape& root, ShapeType subType, ShapeType ancestorType) {
    build(root, subType, ancestorType);
  }

  /// Parents of the sub-shape, or an empty list if unknown/none. Lookup by
  /// isSame.
  const std::vector<Shape>& parentsOf(const Shape& sub) const {
    static const std::vector<Shape> kEmpty;
    const int id = subMap_.findIndex(sub);
    if (id == 0) return kEmpty;
    return parents_[static_cast<std::size_t>(id) - 1];
  }

  /// The underlying id-map of sub-shapes (1-based). Parents(i) aligns with it.
  const ShapeMap& subShapes() const noexcept { return subMap_; }
  const std::vector<Shape>& parentsByIndex(int id) const noexcept {
    return parents_[static_cast<std::size_t>(id) - 1];
  }

 private:
  void build(const Shape& root, ShapeType subType, ShapeType ancestorType) {
    // For each ancestor, record it against each of its sub-shapes.
    for (Explorer anc(root, ancestorType); anc.more(); anc.next()) {
      for (Explorer sub(anc.current(), subType); sub.more(); sub.next()) {
        const int id = ensure(sub.current());
        appendUnique(parents_[static_cast<std::size_t>(id) - 1], anc.current());
      }
    }
    // Also register sub-shapes not under any ancestor (parents stay empty),
    // matching TopExp's second pass so ids are complete.
    for (Explorer sub(root, subType, ancestorType); sub.more(); sub.next())
      ensure(sub.current());
  }

  int ensure(const Shape& s) {
    const int before = static_cast<int>(subMap_.size());
    const int id = subMap_.add(s);
    if (id > before) parents_.emplace_back();  // new entry → new (empty) list
    return id;
  }

  static void appendUnique(std::vector<Shape>& list, const Shape& s) {
    for (const Shape& e : list)
      if (e.isSame(s)) return;
    list.push_back(s);
  }

  ShapeMap subMap_;
  std::vector<std::vector<Shape>> parents_;
};

/// Convenience free function mirroring TopExp::MapShapesAndAncestors.
inline AncestryMap mapShapesAndAncestors(const Shape& root, ShapeType subType,
                                         ShapeType ancestorType) {
  return AncestryMap{root, subType, ancestorType};
}

}  // namespace cybercad::native::topology

#endif  // CYBERCAD_NATIVE_TOPOLOGY_EXPLORE_H

// SPDX-License-Identifier: Apache-2.0
//
// orient.h — make every face's winding consistent (all outward) by flood-fill
// across shared edges, then a global enclosed-volume sign tie-break.
//
// A closed 2-manifold is consistently oriented iff every shared edge is traversed
// in OPPOSITE directions by its two incident faces (the standard manifold
// consistency rule: an interior edge appears once as A→B and once as B→A). The
// sewn faces already share vertex/edge nodes, so adjacency is exact: two faces are
// neighbours across a side iff they list the same unordered shared-vertex pair.
//
// FLOOD-FILL. Start from any face (its winding is the local reference), mark it
// visited, and propagate: for each neighbour sharing a directed side, if the
// neighbour traverses that side in the SAME direction (a winding conflict), FLIP
// the neighbour (reverse its vertex loop) so the shared side becomes opposite;
// then recurse. Every face is visited once; nFlipped counts the reversals.
//
// GLOBAL SIGN. Flood-fill only makes the shell CONSISTENT, not necessarily
// OUTWARD — the seed's winding could be uniformly inward. The orchestrator computes
// the enclosed-volume sign of the assembled shell and, if negative, flips the whole
// shell (reverse every loop). That is the unambiguous outward tie-break the design
// calls for; this header exposes reverseAll() for it.
//
// OCCT-FREE. Uses src/native/topology + tolerant_sew.h. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_ORIENT_H
#define CYBERCAD_NATIVE_HEAL_ORIENT_H

#include "native/heal/tolerant_sew.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cstdint>
#include <unordered_map>
#include <utility>
#include <vector>

namespace cybercad::native::heal {

namespace topo = cybercad::native::topology;

namespace detail {

// A directed side (ordered shared-vertex-node pair) of a face, tagged with which
// face and side index it came from. Two faces are neighbours across a side iff they
// share the same UNORDERED node pair; consistent iff they list it in OPPOSITE order.
struct SideRef {
  const topo::TShape* a;  // start node
  const topo::TShape* b;  // end node
  int face;
};

inline std::pair<const topo::TShape*, const topo::TShape*> unordered(const SideRef& s) {
  return s.a <= s.b ? std::make_pair(s.a, s.b) : std::make_pair(s.b, s.a);
}

struct PairHash {
  std::size_t operator()(const std::pair<const topo::TShape*, const topo::TShape*>& k) const noexcept {
    return std::hash<const topo::TShape*>{}(k.first) * 1099511628211ull ^
           std::hash<const topo::TShape*>{}(k.second);
  }
};

// Directed sides of face f from its (possibly reversed) vertex loop.
inline std::vector<SideRef> sidesOf(const SewnFace& sf, int f) {
  std::vector<SideRef> sides;
  const std::size_t n = sf.verts.size();
  for (std::size_t i = 0; i < n; ++i) {
    const topo::TShape* a = sf.verts[i].tshape().get();
    const topo::TShape* b = sf.verts[(i + 1) % n].tshape().get();
    if (sf.reversed) std::swap(a, b);  // the flip reverses traversal direction
    sides.push_back(SideRef{a, b, f});
  }
  return sides;
}

}  // namespace detail

/// Reverse a SewnFace's winding (toggles the traversal direction of every side).
inline void reverseFace(SewnFace& sf) { sf.reversed = !sf.reversed; }

/// Reverse every face's winding (the global outward tie-break).
inline void reverseAll(std::vector<SewnFace>& faces) {
  for (SewnFace& sf : faces) reverseFace(sf);
}

/// Flood-fill face orientation to make every shared side opposite-traversed.
/// Returns nFlipped (faces whose winding was reversed). Operates in place on
/// `faces[*].reversed`. Disconnected components are each seeded independently.
inline int makeConsistent(std::vector<SewnFace>& faces) {
  using detail::PairHash;
  using detail::SideRef;
  using detail::sidesOf;
  using detail::unordered;

  // Adjacency: unordered node-pair → the (face, directed a→b) that owns it.
  std::unordered_multimap<std::pair<const topo::TShape*, const topo::TShape*>, SideRef, PairHash> byPair;
  const int nf = static_cast<int>(faces.size());
  for (int f = 0; f < nf; ++f)
    for (const SideRef& s : sidesOf(faces[f], f)) byPair.emplace(unordered(s), s);

  std::vector<char> visited(static_cast<std::size_t>(nf), 0);
  int nFlipped = 0;

  for (int seed = 0; seed < nf; ++seed) {
    if (visited[static_cast<std::size_t>(seed)]) continue;
    std::vector<int> stack{seed};
    visited[static_cast<std::size_t>(seed)] = 1;
    while (!stack.empty()) {
      const int f = stack.back();
      stack.pop_back();
      for (const SideRef& s : sidesOf(faces[f], f)) {
        auto range = byPair.equal_range(unordered(s));
        for (auto it = range.first; it != range.second; ++it) {
          const SideRef& nb = it->second;
          if (nb.face == f) continue;
          if (visited[static_cast<std::size_t>(nb.face)]) continue;
          // Recompute the neighbour's CURRENT directed side (its flip state may
          // change as we visit), then check against THIS face's side s (a→b).
          bool sameDir = false;
          for (const SideRef& ns : sidesOf(faces[nb.face], nb.face))
            if (unordered(ns) == unordered(s)) { sameDir = (ns.a == s.a && ns.b == s.b); break; }
          if (sameDir) {            // conflict: both traverse a→b ⇒ flip neighbour
            reverseFace(faces[static_cast<std::size_t>(nb.face)]);
            ++nFlipped;
          }
          visited[static_cast<std::size_t>(nb.face)] = 1;
          stack.push_back(nb.face);
        }
      }
    }
  }
  return nFlipped;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_ORIENT_H

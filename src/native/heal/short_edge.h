// SPDX-License-Identifier: Apache-2.0
//
// short_edge.h — bounded, opt-in collapse of a spurious SHORT (sub-feature) edge
// that a boundary vertex-split inserted into an otherwise-straight wire run
// (MOAT stage M5 tail — small-edge / short-edge merge).
//
// The landed slices handle a ZERO-LENGTH side (two corners within `tolerance` ⇒
// dropZeroLengthSides removes the duplicate, degenerate.h) and a beyond-tolerance
// SEAM near-miss between two DIFFERENT faces (gap_bridge.h). Neither touches the
// distinct defect this pass addresses: a redundant vertex on a single face's
// straight boundary run that a STEP exporter / mesh→B-rep conversion split into a
// tiny NON-zero edge whose length sits ABOVE `tolerance` but below a caller-supplied
// merge length. Because that micro-edge carries an interior vertex the neighbouring
// face does NOT have (its matching span is one straight edge), the sew cannot share
// the run and the shell is left OpenShell — exactly the case OCCT closes with
// `ShapeFix_Wire::FixSmall` / sewing at a larger tolerance. This pass earns the same
// win natively under an EXPLICIT, BOUNDED, OPT-IN merge length — never by widening
// the primary weld tolerance and never by moving a corner that carries real feature.
//
// ── THE BOUND (why this is not a fabricated repair) ──────────────────────────────
// A short edge B→C (with wire neighbours A before B and D after C) is collapsed ONLY
// when EVERY layer below accepts it; otherwise the loop is left UNCHANGED:
//
//   1. OPT-IN MERGE LENGTH. `shortEdgeMergeLen == 0` ⇒ this pass never runs (heal.cpp
//      guards it) ⇒ the landed slices are byte-identical. `> 0` is a SEPARATE bound on
//      the absolute edge length that may be collapsed ABOVE `tolerance`; the primary
//      weld `tolerance` is untouched.
//
//   2. LOCAL-FEATURE-SIZE CAP. The effective bound is `min(shortEdgeMergeLen,
//      kLocalFeatureFraction · min(|A−B|, |C−D|))` with `kLocalFeatureFraction = 0.25`.
//      A short edge comparable to either neighbour edge can therefore NEVER be
//      collapsed — only an edge under a quarter of the smaller neighbour, i.e. a
//      genuine sub-feature sliver — regardless of how large the caller sets the merge
//      length. This is the geometric guarantee that the merge cannot erase real
//      topology.
//
//   3. COLLINEARITY WITHIN TOLERANCE. Both B and C MUST lie within `tolerance` of the
//      straight line A→D (the run the two collinear neighbour edges define). A short
//      edge that turns a real corner (B or C off the A-D line) is NOT redundant and is
//      left in place — collapsing it would change the face's boundary. Only a redundant
//      collinear split is removed, and removing it restores the EXACT straight span
//      A→D the neighbour face already carries, so vertex_unify then shares A and D and
//      the shell closes.
//
//   4. LOOP STAYS A POLYGON. A face is only rewritten if collapsing leaves ≥ 3 corners
//      (a triangle is the floor); a loop that would drop below 3 is left unchanged
//      (degenerate.h handles a genuine collapse-to-sliver separately).
//
//   5. MANDATORY SELF-VERIFY (in heal.cpp, UNCHANGED). After collapsing + re-sew the
//      candidate must STILL tessellate watertight with positive enclosed volume; a
//      candidate that does not is discarded (`Unhealed{SelfVerifyFailed}`). Self-verify
//      — not this pass's bookkeeping — is the authoritative closure check.
//
// ── ASYMPTOTIC-TAIL CAVEAT ───────────────────────────────────────────────────────
// This removes only a COLLINEAR redundant sub-feature edge. A short edge that turns a
// real (non-collinear) corner, a short edge whose removal would need the neighbour
// face re-projected, pcurve reconstruction, and self-intersecting-wire repair remain
// OCCT `ShapeFix`'s moat and are declined honestly (never claimed here). No new
// `UnhealedReason` is introduced.
//
// OCCT-FREE. Uses src/native/math + the FaceLoop soup type only. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_SHORT_EDGE_H
#define CYBERCAD_NATIVE_HEAL_SHORT_EDGE_H

#include "native/heal/face_soup.h"
#include "native/math/native_math.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace cybercad::native::heal {

namespace math = cybercad::native::math;

/// A quarter of the smaller NEIGHBOUR edge is the hard local-feature cap: a short edge
/// longer than this is never collapsed, whatever the merge length (see header, layer 2).
inline constexpr double kShortEdgeFeatureFraction = 0.25;

/// Outcome of the short-edge collapse pass: the (possibly) rewritten soup + what it
/// did. When `applied == false` the soup is a copy of the input and heal.cpp keeps its
/// original sew result unchanged.
struct ShortEdgeResult {
  std::vector<FaceLoop> soup;  ///< corners after collinear short-edge removal (normals refreshed)
  bool applied = false;        ///< true ⇒ at least one short edge was collapsed
  int nCollapsed = 0;          ///< redundant collinear short edges removed
  double maxCollapsed = 0.0;   ///< longest short edge collapsed (≤ the effective bound)
};

namespace detail {

// Perpendicular distance of point `p` from the (infinite) line through a and d.
// A degenerate a==d line falls back to |p−a| (treated as the deviation).
inline double distanceToLine(const math::Point3& p, const math::Point3& a, const math::Point3& d) {
  const math::Vec3 ad = d - a;
  const double len = math::norm(ad);
  if (len <= 1e-15) return math::distance(p, a);
  const math::Vec3 ap = p - a;
  return math::norm(math::cross(ap, ad)) / len;
}

// Collapse redundant collinear short edges from ONE loop. A side B→C (with wire
// neighbours A = prev(B) and D = next(C)) is a redundant collinear short edge when:
//   * |B−C| is in the band (tol, min(mergeLen, ¼·min(|A−B|, |C−D|))], AND
//   * both B and C lie within `tol` of the straight line A→D.
// It is removed by dropping BOTH B and C from the loop, restoring the straight span
// A→D (the span the neighbour face already carries). Marks + drops in one left-to-right
// sweep; each removal advances past D so a corner is never consumed by two collapses.
inline std::vector<math::Point3> collapseLoop(const std::vector<math::Point3>& loop, double tol,
                                              double mergeLen, int& nCollapsed, double& maxCollapsed) {
  const std::size_t n = loop.size();
  if (n < 5) return loop;  // need A,B,C,D distinct + ≥ 3 survivors ⇒ ≥ 5 corners

  std::vector<char> drop(n, 0);
  std::size_t i = 0;
  int survivors = static_cast<int>(n);
  while (i < n) {
    const std::size_t b = i, c = (i + 1) % n, a = (i + n - 1) % n, d = (c + 1) % n;
    // Skip if any of the four corners is already marked (keeps removals disjoint).
    if (drop[a] || drop[b] || drop[c] || drop[d]) { ++i; continue; }
    const double eBC = math::distance(loop[b], loop[c]);
    const double eAB = math::distance(loop[a], loop[b]);
    const double eCD = math::distance(loop[c], loop[d]);
    const double bound = std::min(mergeLen, kShortEdgeFeatureFraction * std::min(eAB, eCD));
    if (eBC > tol && eBC <= bound && survivors - 2 >= 3 &&
        distanceToLine(loop[b], loop[a], loop[d]) <= tol &&
        distanceToLine(loop[c], loop[a], loop[d]) <= tol) {
      drop[b] = drop[c] = 1;   // remove the redundant micro-edge ⇒ span becomes A→D
      ++nCollapsed;
      survivors -= 2;
      maxCollapsed = std::max(maxCollapsed, eBC);
      i += 2;                  // advance past C so D starts the next window
      continue;
    }
    ++i;
  }
  std::vector<math::Point3> out;
  out.reserve(static_cast<std::size_t>(survivors));
  for (std::size_t k = 0; k < n; ++k)
    if (!drop[k]) out.push_back(loop[k]);
  return out;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// collapseShortEdges — remove every redundant collinear short edge from every face
// loop when it lies in the bounded band `(tol, min(mergeLen, ¼·neighbour)]`, then hand
// the rewritten soup back for a re-sew. See the file header for the five safety layers.
// ─────────────────────────────────────────────────────────────────────────────
inline ShortEdgeResult collapseShortEdges(const std::vector<FaceLoop>& soup, double tol,
                                          double mergeLen) {
  ShortEdgeResult out;
  out.soup = soup;  // start from an unchanged copy; only collapsed faces are rewritten
  if (mergeLen <= 0.0) return out;

  for (std::size_t f = 0; f < out.soup.size(); ++f) {
    const int before = out.nCollapsed;
    std::vector<math::Point3> collapsed =
        detail::collapseLoop(out.soup[f].corners, tol, mergeLen, out.nCollapsed, out.maxCollapsed);
    if (out.nCollapsed != before) {
      out.soup[f].corners = std::move(collapsed);
      out.soup[f].normal = newellNormal(out.soup[f].corners);  // winding-coherent refresh
    }
  }
  out.applied = out.nCollapsed > 0;
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_SHORT_EDGE_H

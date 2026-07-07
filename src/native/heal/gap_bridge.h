// SPDX-License-Identifier: Apache-2.0
//
// gap_bridge.h — bounded, opt-in beyond-tolerance gap bridging (MOAT stage M5).
//
// The landed slice DECLINES a soup whose primary weld at `tolerance` leaves a
// boundary edge with a gap > `tolerance` (reason GapBeyondTolerance): a seam
// near-miss where two faces that should share an edge had their corners written a
// hair farther apart than the caller's weld tolerance. OCCT `BRepBuilderAPI_Sewing`
// closes these by sewing at a LARGER tolerance. This pass earns the same win
// natively under an EXPLICIT, BOUNDED, OPT-IN budget — never by widening the primary
// weld.
//
// ── THE BOUND (why this is not a fabricated closure) ─────────────────────────────
// A corner is only bridged when EVERY layer below accepts it; otherwise the heal
// declines honestly with the measured residual and the input unchanged:
//
//   1. OPT-IN BUDGET. `budget == 0` ⇒ this pass never runs (heal.cpp guards it) ⇒
//      the landed slice is byte-identical. `budget > 0` caps the absolute gap that
//      may be closed ABOVE `tolerance`; the primary weld `tolerance` is untouched.
//
//   2. LOCAL-FEATURE-SIZE CAP. For each corner the effective bound is
//      `min(budget, kLocalFeatureFraction · shortestIncidentEdgeLen)` with
//      `kLocalFeatureFraction = 0.25`. A gap comparable to a real edge can therefore
//      NEVER be bridged — only seams narrower than a quarter of the smallest local
//      feature — regardless of how large the caller sets `budget`. This is the
//      geometric guarantee that bridging cannot collapse distinct topology.
//
//   3. CROSS-FACE, UNPAIRED-ONLY, DETERMINISTIC MATCHING. Only corners the primary
//      weld left UNPAIRED (no within-`tolerance` partner on another face) are
//      eligible. A stray corner snaps to the position of its nearest corner on a
//      DIFFERENT face (never folding one face onto itself) when that corner already
//      belongs to established (within-tol paired) geometry; a symmetric seam whose
//      two sides are mutually-nearest and both unpaired snaps both to their midpoint.
//      Anything ambiguous is left unbridged (honest decline). No corner is moved
//      twice; the result is order-independent.
//
//   4. MANDATORY SELF-VERIFY (in heal.cpp, unchanged). After bridging + re-sew the
//      candidate must still tessellate watertight with positive enclosed volume; a
//      bridged candidate that does not is discarded (Unhealed{SelfVerifyFailed}). A
//      bridge that would create a non-manifold edge fails that watertight gate
//      (every mesh edge must be used by exactly two triangles), so self-verify — not
//      this pass's bookkeeping — is the authoritative manifold check.
//
// ── ASYMPTOTIC-TAIL CAVEAT ───────────────────────────────────────────────────────
// This closes only the bounded near-miss band `(tolerance, min(budget,
// ¼·localFeature)]`. Arbitrary beyond-tolerance repair — real holes needing a
// synthesized face, geometry-scale separation, non-seam defects — remains OCCT
// `ShapeFix`'s moat and is declined honestly (never claimed here).
//
// OCCT-FREE. Uses src/native/{math,topology} + the FaceLoop soup type only. clang++
// -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_GAP_BRIDGE_H
#define CYBERCAD_NATIVE_HEAL_GAP_BRIDGE_H

#include "native/heal/face_soup.h"
#include "native/math/native_math.h"

#include <algorithm>
#include <cstddef>
#include <limits>
#include <vector>

namespace cybercad::native::heal {

namespace math = cybercad::native::math;

/// A quarter of the shortest incident edge is the hard local-feature cap: a gap
/// wider than this is never bridged, whatever the budget (see file header, layer 2).
inline constexpr double kLocalFeatureFraction = 0.25;

/// Outcome of the bridging pass: the (possibly) rewritten soup + what it did. When
/// `applied == false` the soup is a copy of the input and heal.cpp keeps its
/// original sew result unchanged.
struct BridgeResult {
  std::vector<FaceLoop> soup;  ///< corners after bounded snapping (Newell normals refreshed)
  bool applied = false;        ///< true ⇒ at least one corner was bridged
  int nBridged = 0;            ///< boundary corners moved onto a partner
  double maxBridged = 0.0;     ///< largest gap closed (≤ the effective bound)
};

namespace detail {

// A flat, indexable view of every soup corner tagged with its owning face + the
// shortest edge incident to it (the local-feature length used for the cap).
struct CornerRef {
  math::Point3 p;        ///< world position
  int face;              ///< owning FaceLoop index
  int slot;              ///< corner index within that face's loop
  double shortestEdge;   ///< length of the shorter of its two incident face edges
};

// Build the flat corner list. The shortest incident edge of corner c in a face is
// the shorter of the sides c→c-1 and c→c+1 (the loop is closed modulo n).
inline std::vector<CornerRef> flattenCorners(const std::vector<FaceLoop>& soup) {
  std::vector<CornerRef> out;
  for (int f = 0; f < static_cast<int>(soup.size()); ++f) {
    const std::vector<math::Point3>& cs = soup[static_cast<std::size_t>(f)].corners;
    const std::size_t n = cs.size();
    if (n < 2) continue;
    for (std::size_t i = 0; i < n; ++i) {
      const math::Point3& prev = cs[(i + n - 1) % n];
      const math::Point3& next = cs[(i + 1) % n];
      const double e = std::min(math::distance(cs[i], prev), math::distance(cs[i], next));
      out.push_back(CornerRef{cs[i], f, static_cast<int>(i), e});
    }
  }
  return out;
}

// The effective bridge bound for one corner: min(budget, ¼·shortestIncidentEdge).
inline double effectiveBound(const CornerRef& c, double budget) {
  return std::min(budget, kLocalFeatureFraction * c.shortestEdge);
}

// Index of the nearest corner on a DIFFERENT face than `i` (deterministic: lowest
// index breaks a tie). Returns -1 if none exists.
inline int nearestCrossFace(const std::vector<CornerRef>& cs, int i) {
  int best = -1;
  double bestD2 = std::numeric_limits<double>::max();
  const CornerRef& a = cs[static_cast<std::size_t>(i)];
  for (int j = 0; j < static_cast<int>(cs.size()); ++j) {
    const CornerRef& b = cs[static_cast<std::size_t>(j)];
    if (b.face == a.face) continue;
    const double d2 = math::normSquared(b.p - a.p);
    if (d2 < bestD2) { bestD2 = d2; best = j; }
  }
  return best;
}

// A corner is PAIRED (handled by the primary weld already) iff it has a corner on a
// different face within `tol`.
inline bool isPaired(const std::vector<CornerRef>& cs, int i, double tol) {
  const CornerRef& a = cs[static_cast<std::size_t>(i)];
  for (int j = 0; j < static_cast<int>(cs.size()); ++j) {
    const CornerRef& b = cs[static_cast<std::size_t>(j)];
    if (b.face == a.face) continue;
    if (math::distance(a.p, b.p) <= tol) return true;
  }
  return false;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// bridgeGaps — snap each UNPAIRED boundary corner onto its nearest cross-face
// partner when the gap lies in the bounded band `(tol, effectiveBound]`, then hand
// the rewritten soup back for a re-sew. See the file header for the four safety
// layers. Cognitive complexity: the one dense function (systems band); each geometric
// sub-step is a named helper in `detail`.
// ─────────────────────────────────────────────────────────────────────────────
inline BridgeResult bridgeGaps(const std::vector<FaceLoop>& soup, double tol, double budget) {
  using detail::CornerRef;
  BridgeResult out;
  out.soup = soup;  // start from an unchanged copy; only bridged corners are moved
  if (budget <= 0.0) return out;

  const std::vector<CornerRef> cs = detail::flattenCorners(soup);
  const int n = static_cast<int>(cs.size());

  // Decide a target position per corner without moving anything yet (so every
  // decision reads the ORIGINAL geometry — order-independent, no corner moved twice).
  std::vector<math::Point3> newPos(static_cast<std::size_t>(n));
  std::vector<char> moved(static_cast<std::size_t>(n), 0);
  for (int i = 0; i < n; ++i) newPos[static_cast<std::size_t>(i)] = cs[static_cast<std::size_t>(i)].p;

  for (int i = 0; i < n; ++i) {
    if (detail::isPaired(cs, i, tol)) continue;  // primary weld already closed this corner
    const int j = detail::nearestCrossFace(cs, i);
    if (j < 0) continue;
    const CornerRef& a = cs[static_cast<std::size_t>(i)];
    const CornerRef& b = cs[static_cast<std::size_t>(j)];
    const double d = math::distance(a.p, b.p);
    // In-band AND within BOTH corners' feature caps (symmetric — never collapse
    // across the smaller local feature of either side).
    if (!(d > tol && d <= detail::effectiveBound(a, budget) &&
          d <= detail::effectiveBound(b, budget)))
      continue;

    if (detail::isPaired(cs, j, tol)) {
      // Stray corner snaps onto established (within-tol paired) geometry.
      newPos[static_cast<std::size_t>(i)] = b.p;
    } else if (detail::nearestCrossFace(cs, j) == i) {
      // Symmetric seam: both sides unpaired and mutually nearest → both to midpoint.
      newPos[static_cast<std::size_t>(i)] = a.p + (b.p - a.p) * 0.5;
    } else {
      continue;  // ambiguous (unpaired, non-mutual) → leave unbridged, decline honestly
    }
    moved[static_cast<std::size_t>(i)] = 1;
    out.nBridged++;
    out.maxBridged = std::max(out.maxBridged, d);
  }

  if (out.nBridged == 0) return out;  // nothing in-band → applied stays false

  // Write the snapped positions back into the soup and refresh each touched face's
  // winding-coherent Newell normal.
  std::vector<char> faceTouched(soup.size(), 0);
  for (int i = 0; i < n; ++i) {
    if (!moved[static_cast<std::size_t>(i)]) continue;
    const CornerRef& c = cs[static_cast<std::size_t>(i)];
    out.soup[static_cast<std::size_t>(c.face)].corners[static_cast<std::size_t>(c.slot)] =
        newPos[static_cast<std::size_t>(i)];
    faceTouched[static_cast<std::size_t>(c.face)] = 1;
  }
  for (std::size_t f = 0; f < out.soup.size(); ++f)
    if (faceTouched[f]) out.soup[f].normal = newellNormal(out.soup[f].corners);

  out.applied = true;
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_GAP_BRIDGE_H

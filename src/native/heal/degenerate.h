// SPDX-License-Identifier: Apache-2.0
//
// degenerate.h — detect and drop degenerate elements from the face-loop soup
// BEFORE sewing: zero-length edges and sliver / zero-area faces.
//
// Two degeneracies break a rebuild if carried through:
//   * ZERO-LENGTH EDGE — a wire side whose two corners are within `tolerance`
//     (often a duplicated corner). It contributes no boundary and, kept, produces a
//     null edge. Detected as consecutive corners closer than `tolerance`; the
//     duplicate corner is dropped from the loop (the side vanishes).
//   * SLIVER / ZERO-AREA FACE — a face whose loop encloses less than `tolerance²`
//     of area (all corners near-collinear, or the loop collapses to < 3 distinct
//     corners after removing zero-length sides). It carries no surface region and,
//     kept, injects a degenerate triangle. The whole face is removed from the soup.
//
// Both are counted into nDroppedDegenerate. This runs on the extracted FaceLoop set
// (world corner loops) so it is independent of the input graph and feeds a clean
// soup into vertex_unify / sew.
//
// OCCT-FREE. Uses src/native/math + face_soup.h. clang++ -std=c++20. Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_DEGENERATE_H
#define CYBERCAD_NATIVE_HEAL_DEGENERATE_H

#include "native/heal/face_soup.h"
#include "native/math/native_math.h"

#include <cmath>
#include <vector>

namespace cybercad::native::heal {

namespace math = cybercad::native::math;

/// The polygon area of a corner loop measured in the plane with normal `n`
/// (Newell / projected shoelace). Independent of winding sign (absolute value).
inline double loopArea(const std::vector<math::Point3>& loop, const math::Dir3& n) {
  if (loop.size() < 3) return 0.0;
  math::Vec3 acc{0, 0, 0};  // Σ (pᵢ × pᵢ₊₁)
  const std::size_t m = loop.size();
  for (std::size_t i = 0; i < m; ++i) {
    const math::Vec3 a = loop[i].asVec();
    const math::Vec3 b = loop[(i + 1) % m].asVec();
    acc += math::cross(a, b);
  }
  return 0.5 * std::fabs(math::dot(acc, n.vec()));
}

/// The longest side length of a corner loop (its widest extent along an edge).
inline double loopLongestSide(const std::vector<math::Point3>& loop) {
  double longest = 0.0;
  const std::size_t m = loop.size();
  for (std::size_t i = 0; i < m; ++i)
    longest = std::max(longest, math::distance(loop[i], loop[(i + 1) % m]));
  return longest;
}

/// A face is a SLIVER when its minimum height — the smallest perpendicular
/// distance from a vertex to the loop, i.e. 2·area / longest-side — is below
/// `tolerance`. This catches a near-collinear "needle" (large extent, negligible
/// width) that raw area vs tolerance² misses: such a face has real length but is
/// thinner than the sew tolerance, so it carries no meaningful surface region and a
/// rebuild would inject a degenerate triangle. A loop with < 3 corners is trivially
/// a sliver.
inline bool isSliver(const std::vector<math::Point3>& loop, const math::Dir3& n, double tol) {
  if (loop.size() < 3) return true;
  const double longest = loopLongestSide(loop);
  if (longest <= tol) return true;  // the whole loop fits inside one tolerance ball
  const double minHeight = 2.0 * loopArea(loop, n) / longest;
  return minHeight < tol;
}

/// Remove consecutive corners within `tolerance` of each other (zero-length sides)
/// and the closing duplicate. Increments `nDropped` per removed side.
inline std::vector<math::Point3> dropZeroLengthSides(const std::vector<math::Point3>& loop,
                                                     double tol, int& nDropped) {
  std::vector<math::Point3> out;
  for (const math::Point3& p : loop) {
    if (!out.empty() && math::distance(out.back(), p) <= tol) {
      ++nDropped;  // coincident with the previous corner ⇒ zero-length side
      continue;
    }
    out.push_back(p);
  }
  while (out.size() >= 2 && math::distance(out.front(), out.back()) <= tol) {
    out.pop_back();
    ++nDropped;
  }
  return out;
}

/// Clean the soup: drop zero-length sides from every loop, then drop whole faces
/// that became sliver/zero-area (area < tolerance² or < 3 distinct corners).
/// Returns the surviving loops; `nDropped` totals removed sides + removed faces.
inline std::vector<FaceLoop> removeDegenerate(const std::vector<FaceLoop>& soup, double tol,
                                              int& nDropped) {
  nDropped = 0;
  std::vector<FaceLoop> out;
  out.reserve(soup.size());
  for (const FaceLoop& fl : soup) {
    FaceLoop cleaned;
    cleaned.corners = dropZeroLengthSides(fl.corners, tol, nDropped);
    // Recompute the winding-coherent normal from the cleaned loop.
    cleaned.normal = newellNormal(cleaned.corners);
    if (isSliver(cleaned.corners, cleaned.normal, tol)) {
      ++nDropped;  // the whole face is a sliver / collapsed ⇒ remove it
      continue;
    }
    cleaned.valid = true;
    out.push_back(std::move(cleaned));
  }
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_DEGENERATE_H

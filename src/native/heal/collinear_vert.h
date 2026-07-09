// SPDX-License-Identifier: Apache-2.0
//
// collinear_vert.h — bounded, opt-in removal of a single REDUNDANT COLLINEAR vertex
// on an otherwise-straight boundary run (MOAT stage M5 tail — collinear/redundant
// vertex unification, the classic STEP-import "T-vertex" / seam-split artifact).
//
// ── THE DEFECT (distinct from every landed pass) ─────────────────────────────────
// A STEP exporter / mesh→B-rep conversion frequently drops one-or-MORE EXTRA vertices onto
// a face's straight boundary run A→C, so that face carries A→B1→…→C (two-or-more edges)
// while the NEIGHBOUR face carries the same span as ONE straight edge A→C. Each Bₖ lies on
// the line A→C (they are redundant — they turn no corner). The removal below iterates to a
// FIXPOINT, so a RUN of two-or-more consecutive collinear vertices is collapsed fully, not
// just the first. For a single B this is the classic STEP "T-vertex" / seam-split; B lies EXACTLY on the
// line A→C (it is redundant — it turns no corner), and BOTH incident edges |A−B| and
// |B−C| may be FULL-LENGTH real edges. Because the neighbour does not carry B, the
// tolerant sew cannot share the run: boundary edges survive and the shell is returned
// `Unhealed` honestly, with a measured residual. OCCT closes exactly this with
// `ShapeUpgrade_UnifySameDomain` / `ShapeFix_Wire` (it drops the collinear vertex).
//
// This is NOT any landed pass:
//   * `degenerate.h::dropZeroLengthSides` removes only a ≤`tolerance` (near-ZERO-length)
//     side — a duplicated corner. Here |A−B| and |B−C| are full-length real edges.
//   * `gap_bridge.h` snaps CROSS-FACE unpaired corners onto a partner (a seam near-miss
//     BETWEEN two faces). Here the redundant vertex is WITHIN a single face's wire.
//   * `short_edge.h` removes a redundant collinear SUB-FEATURE MICRO-EDGE B→C (a tiny
//     span bounded at ¼·neighbour, removing TWO corners B and C, needing ≥ 5 loop
//     corners). Here a SINGLE vertex B sits between two full-length edges; there is no
//     micro-edge and no ¼·neighbour length bound applies — only collinearity governs.
//     `collapseLoop` cannot touch this case (its length band rejects a full-length B→C).
//
// ── THE BOUND (why this is not a fabricated repair) ──────────────────────────────
// A vertex B (wire neighbours A = prev(B), C = next(B)) is removed ONLY when EVERY
// layer below accepts it; otherwise the loop is left UNCHANGED:
//
//   1. OPT-IN. `removeCollinearVerts == false` ⇒ this pass never runs (heal.cpp guards
//      it) ⇒ the landed slices are BYTE-IDENTICAL. It introduces NO length parameter:
//      the sole criterion is exact collinearity, so it never removes a real corner
//      whatever the edge lengths.
//
//   2. COLLINEARITY WITHIN TOLERANCE. B is removed ONLY when its perpendicular distance
//      to the straight line A→C is ≤ `tolerance` AND B projects strictly BETWEEN A and C
//      (0 < t < 1 along A→C, so a reflex/backtracking "spur" that folds back is NEVER
//      treated as redundant). A vertex that turns a real corner (B off the A-C line by
//      more than `tolerance`) is left in place — removing it would change the face's
//      boundary by more than the weld tolerance. Removing a within-tolerance-collinear B
//      restores the EXACT straight span A→C the neighbour face already carries, so
//      vertex_unify then shares A and C and the shell closes. This is the closed-form
//      arbiter: the boundary moves by at most `tolerance`, the same bound the sew honours.
//
//   3. LOOP STAYS A POLYGON. A face is only rewritten if it keeps ≥ 3 corners after the
//      removal (a triangle is the floor); a loop that would drop below 3 is left
//      unchanged. Within ONE pass removals are kept disjoint (a removed B's neighbours A,C
//      are not themselves removed in the same sweep) so one corner is never consumed twice;
//      the pass is then iterated to a FIXPOINT so a RUN of two-or-more consecutive collinear
//      vertices (A→B1→B2→C, the disjoint skip leaves B2 standing after B1 goes) collapses
//      fully — each pass reads the previous pass's survivors, so B2's neighbours become A,C
//      and it is removed on the next pass. The fixpoint removes only corners collinear
//      within tol among the current survivors, so it never removes a real corner.
//
//   4. MANDATORY SELF-VERIFY (in heal.cpp, UNCHANGED). After removal + re-sew the
//      candidate must STILL tessellate watertight with positive enclosed volume; a
//      candidate that does not is discarded (`Unhealed{SelfVerifyFailed}`). Self-verify
//      — not this pass's bookkeeping — is the authoritative closure check.
//
// ── ASYMPTOTIC-TAIL CAVEAT ───────────────────────────────────────────────────────
// This removes only a vertex that is COLLINEAR within tolerance on its own wire. A
// vertex that turns a real (non-collinear) corner, a vertex whose removal would need the
// neighbour face re-projected, pcurve reconstruction, and self-intersecting-wire repair
// remain OCCT `ShapeFix`'s moat and are declined honestly (never claimed here). No new
// `UnhealedReason` is introduced.
//
// OCCT-FREE. Uses src/native/math + the FaceLoop soup type only. clang++ -std=c++20.
// Header-only.
//
#ifndef CYBERCAD_NATIVE_HEAL_COLLINEAR_VERT_H
#define CYBERCAD_NATIVE_HEAL_COLLINEAR_VERT_H

#include "native/heal/face_soup.h"
#include "native/math/native_math.h"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace cybercad::native::heal {

namespace math = cybercad::native::math;

/// Outcome of the collinear-vertex removal pass: the (possibly) rewritten soup + what
/// it did. When `applied == false` the soup is a copy of the input and heal.cpp keeps
/// its original sew result unchanged.
struct CollinearVertResult {
  std::vector<FaceLoop> soup;  ///< corners after redundant-collinear-vertex removal (normals refreshed)
  bool applied = false;        ///< true ⇒ at least one collinear vertex was removed
  int nRemoved = 0;            ///< redundant collinear vertices removed
  double maxDeviation = 0.0;   ///< largest perpendicular deviation of a removed vertex (≤ tolerance)
};

namespace detail {

// Perpendicular distance of `p` from the segment line through a and c, AND the
// parametric projection t of p onto A→C. A degenerate a==c line returns |p−a| with
// t = 0. `t` lets the caller require B to lie strictly BETWEEN A and C (0 < t < 1) so a
// backtracking spur (t ≤ 0 or t ≥ 1) is never mistaken for a redundant interior vertex.
inline double perpAndParam(const math::Point3& p, const math::Point3& a, const math::Point3& c,
                           double& t) {
  const math::Vec3 ac = c - a;
  const double len2 = math::normSquared(ac);
  if (len2 <= 1e-30) {
    t = 0.0;
    return math::distance(p, a);
  }
  const math::Vec3 ap = p - a;
  t = math::dot(ap, ac) / len2;
  return math::norm(math::cross(ap, ac)) / std::sqrt(len2);
}

// One left-to-right removal sweep over `loop`, appending survivors to `out`. A corner B
// (wire neighbours A = prev(B), C = next(B) among the CURRENT survivors) is removed when:
//   * its perpendicular distance to the line A→C is ≤ tol, AND
//   * it projects strictly between A and C (0 < t < 1), AND
//   * the loop keeps ≥ 3 corners after the removal.
// A removal advances past C so a single pass never consumes a corner twice (removals stay
// disjoint WITHIN one pass — A and C survive their neighbour's removal). Returns the number
// removed THIS pass so the caller can iterate to a fixpoint; the neighbours A,C are read
// from `loop` (the survivors of the previous pass), which is why an ADJACENT second
// collinear vertex left standing by the disjoint skip is picked up on the next pass.
inline int removeLoopVertsPass(const std::vector<math::Point3>& loop, double tol,
                               double& maxDeviation, std::vector<math::Point3>& out) {
  const std::size_t n = loop.size();
  std::vector<char> drop(n, 0);
  int survivors = static_cast<int>(n);
  int removed = 0;
  std::size_t i = 0;
  while (i < n) {
    const std::size_t b = i, a = (i + n - 1) % n, c = (i + 1) % n;
    if (drop[a] || drop[b] || drop[c]) { ++i; continue; }
    double t = 0.0;
    const double dev = perpAndParam(loop[b], loop[a], loop[c], t);
    if (dev <= tol && t > 0.0 && t < 1.0 && survivors - 1 >= 3) {
      drop[b] = 1;  // B is redundant ⇒ the span becomes the straight edge A→C
      ++removed;
      --survivors;
      maxDeviation = std::max(maxDeviation, dev);
      i += 2;       // skip C so it starts the next window (keeps removals disjoint)
      continue;
    }
    ++i;
  }
  out.clear();
  out.reserve(static_cast<std::size_t>(survivors));
  for (std::size_t k = 0; k < n; ++k)
    if (!drop[k]) out.push_back(loop[k]);
  return removed;
}

// Remove ALL redundant collinear vertices from ONE loop, iterating removeLoopVertsPass to a
// FIXPOINT. A single disjoint pass leaves an ADJACENT second collinear vertex standing (the
// skip past C means a run A→B1→B2→C keeps B2), so a STEP exporter that split a straight span
// into three-or-more sub-edges (A→B1→B2→C, both Bₖ within tol of line A→C) would only lose
// one vertex per pass and still block the sew. Re-running the pass on each pass's survivors
// collapses the whole run: once B1 is gone the next pass reads B2's neighbours as A,C and
// removes it too, until a pass removes nothing. Each pass only ever removes a corner that is
// collinear-within-tol among the CURRENT survivors, so the fixpoint never removes a real
// corner and terminates in ≤ n passes (survivors strictly decrease while any removal occurs).
inline std::vector<math::Point3> removeLoopVerts(const std::vector<math::Point3>& loop, double tol,
                                                 int& nRemoved, double& maxDeviation) {
  if (loop.size() < 4) return loop;  // need ≥ 3 survivors after removing ≥ 1 ⇒ ≥ 4 corners

  std::vector<math::Point3> cur = loop;
  std::vector<math::Point3> next;
  for (;;) {
    const int removed = removeLoopVertsPass(cur, tol, maxDeviation, next);
    if (removed == 0) break;
    nRemoved += removed;
    cur.swap(next);
    if (cur.size() < 4) break;  // a triangle is the floor; no interior run can remain
  }
  return cur;
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// removeCollinearVertices — drop every redundant collinear vertex (perpendicular
// deviation ≤ tol, projecting strictly between its neighbours) from every face loop,
// then hand the rewritten soup back for a re-sew. See the file header for the four
// safety layers. Introduces NO length parameter: exact collinearity is the sole
// geometric criterion, so a real corner is never removed whatever the edge lengths.
// ─────────────────────────────────────────────────────────────────────────────
inline CollinearVertResult removeCollinearVertices(const std::vector<FaceLoop>& soup, double tol) {
  CollinearVertResult out;
  out.soup = soup;  // start from an unchanged copy; only rewritten faces change

  for (std::size_t f = 0; f < out.soup.size(); ++f) {
    const int before = out.nRemoved;
    std::vector<math::Point3> pruned =
        detail::removeLoopVerts(out.soup[f].corners, tol, out.nRemoved, out.maxDeviation);
    if (out.nRemoved != before) {
      out.soup[f].corners = std::move(pruned);
      out.soup[f].normal = newellNormal(out.soup[f].corners);  // winding-coherent refresh
    }
  }
  out.applied = out.nRemoved > 0;
  return out;
}

}  // namespace cybercad::native::heal

#endif  // CYBERCAD_NATIVE_HEAL_COLLINEAR_VERT_H

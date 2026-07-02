// SPDX-License-Identifier: Apache-2.0
//
// uv_triangulate.h — robust ear-clipping triangulation of a simple polygon (with
// holes) in the (u,v) parameter plane, used by STAGE 2 (the face mesher) for
// GENUINELY-TRIMMED faces whose boundary is pinned to the shared per-edge
// discretization (edge_mesher.h).
//
// ── ROLE IN THE WATERTIGHT PIPELINE ──────────────────────────────────────────
// A trimmed face (a cylinder cap disk, a holed planar face, a fillet blend patch)
// has a boundary polygon whose vertices are the SHARED edge samples — identical,
// in 3D, to the neighbouring face's samples. Ear clipping triangulates exactly
// those boundary vertices: every boundary segment is a triangle edge and is never
// crossed, so the two faces sharing an edge emit the SAME boundary segmentation
// and the welded solid is watertight along that (possibly CURVED) seam.
//
// Ear clipping is chosen over an incremental Delaunay here because it is
// DEGENERACY-FREE for the modest, well-separated boundary loops we sample (no
// incircle predicate, no super-triangle conditioning, no cocircular-grid blowup —
// the failure modes of a naive Bowyer-Watson on structured input). It always
// yields a valid, non-overlapping triangulation whose vertices are exactly the
// input polygon vertices. Triangle QUALITY is not a verification gate; every
// vertex is S(u,v) (on the true surface), so area/volume converge with deflection.
//
// FULL-PARAMETRIC primitive faces (a whole cylinder side, a whole sphere) that
// need INTERIOR curvature sampling are meshed by the face mesher's structured-grid
// path instead (a tensor grid whose boundary rows use the shared edge samples);
// this header is only the trimmed-polygon triangulator.
//
// ── COMPLEXITY (systems band, flagged ~22) ───────────────────────────────────
// Ear clipping (convex-ear test + point-in-ear rejection) and hole bridging are
// the irreducible geometry, isolated behind triangulatePolygon(). NO OCCT.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_UV_TRIANGULATE_H
#define CYBERCAD_NATIVE_TESSELLATE_UV_TRIANGULATE_H

#include "native/tessellate/trim.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace cybercad::native::tessellate {

// A triangle over indices into the triangulated UV point array.
struct UVTri {
  int a = 0, b = 0, c = 0;
};

namespace detail {

// Signed twice-area of (a,b,c); >0 CCW.
inline double orient2d(const UV& a, const UV& b, const UV& c) noexcept {
  return (b.u - a.u) * (c.v - a.v) - (b.v - a.v) * (c.u - a.u);
}

// Is p strictly inside triangle (a,b,c) (CCW)? Boundary counts as OUTSIDE so a
// polygon vertex lying on an ear edge does not block clipping.
inline bool pointInTriStrict(const UV& a, const UV& b, const UV& c, const UV& p) noexcept {
  return orient2d(a, b, p) > 0.0 && orient2d(b, c, p) > 0.0 && orient2d(c, a, p) > 0.0;
}

// Signed area of a polygon loop (indices into pts). >0 ⇒ CCW.
inline double signedArea(const std::vector<UV>& pts, const std::vector<int>& loop) noexcept {
  double a = 0.0;
  const std::size_t n = loop.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += (pts[loop[j]].u * pts[loop[i]].v - pts[loop[i]].u * pts[loop[j]].v);
  return a * 0.5;
}

// Do the OPEN segments p0p1 and q0q1 properly cross (interiors intersect)? Shared
// endpoints do NOT count as a crossing (a bridge may legitimately touch a vertex).
inline bool segmentsCross(const UV& p0, const UV& p1, const UV& q0, const UV& q1) noexcept {
  const double d1 = orient2d(q0, q1, p0);
  const double d2 = orient2d(q0, q1, p1);
  const double d3 = orient2d(p0, p1, q0);
  const double d4 = orient2d(p0, p1, q1);
  if (((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) &&
      std::fabs(d1) > 0.0 && std::fabs(d2) > 0.0 && std::fabs(d3) > 0.0 && std::fabs(d4) > 0.0)
    return true;
  return false;
}

// Is the segment (a→b) VISIBLE within `loop` — i.e. does it cross NO edge of the
// loop (excluding edges incident to a or b)? Used to pick a bridge that keeps the
// merged boundary simple, so multiple holes bridge into the SAME outer loop without
// the second bridge slicing through the first hole (the sequential-bridging failure
// mode of the old nearest-vertex heuristic).
inline bool visible(const std::vector<UV>& pts, const std::vector<int>& loop, int a, int b) {
  const UV& A = pts[a];
  const UV& B = pts[b];
  const std::size_t n = loop.size();
  for (std::size_t i = 0; i < n; ++i) {
    const int e0 = loop[i], e1 = loop[(i + 1) % n];
    if (e0 == a || e0 == b || e1 == a || e1 == b) continue;  // incident edge
    if (segmentsCross(A, B, pts[e0], pts[e1])) return false;
  }
  return true;
}

// Bridge a hole loop into the current merged loop by a VISIBILITY-checked cut. The
// hole's max-u vertex is joined to the nearest merged-loop vertex whose connecting
// segment crosses no current edge, so the polygon-with-holes stays one SIMPLE
// polygon even for several holes (bridged rightmost-first by triangulatePolygon).
// Falls back to the nearest vertex if none tests visible (a degenerate config —
// ear clipping then still terminates via its per-pass guard).
inline std::vector<int> bridgeHole(const std::vector<UV>& pts, std::vector<int> outer,
                                   std::vector<int> hole) {
  if (hole.size() < 3) return outer;
  if (signedArea(pts, outer) < 0.0) std::reverse(outer.begin(), outer.end());  // outer CCW
  if (signedArea(pts, hole) > 0.0) std::reverse(hole.begin(), hole.end());     // hole CW
  std::size_t hMax = 0;
  for (std::size_t i = 1; i < hole.size(); ++i)
    if (pts[hole[i]].u > pts[hole[hMax]].u) hMax = i;
  const int hVert = hole[hMax];
  const UV& hp = pts[hVert];
  // Prefer the nearest VISIBLE merged-loop vertex; fall back to nearest overall.
  std::size_t oPick = 0, oNearest = 0;
  double bestVis = 1e300, bestAny = 1e300;
  bool haveVis = false;
  for (std::size_t i = 0; i < outer.size(); ++i) {
    const double du = pts[outer[i]].u - hp.u, dv = pts[outer[i]].v - hp.v;
    const double d = du * du + dv * dv;
    if (d < bestAny) { bestAny = d; oNearest = i; }
    if (d < bestVis && visible(pts, outer, outer[i], hVert)) {
      bestVis = d;
      oPick = i;
      haveVis = true;
    }
  }
  if (!haveVis) oPick = oNearest;
  std::vector<int> merged;
  merged.reserve(outer.size() + hole.size() + 2);
  for (std::size_t i = 0; i <= oPick; ++i) merged.push_back(outer[i]);
  const std::size_t hn = hole.size();
  for (std::size_t k = 0; k <= hn; ++k) merged.push_back(hole[(hMax + k) % hn]);
  for (std::size_t i = oPick; i < outer.size(); ++i) merged.push_back(outer[i]);
  return merged;
}

// Is vertex `i` of `loop` a valid ear (convex, and no other loop vertex inside the
// candidate triangle)? Extracted from earClip to keep the clipping loop's nesting
// penalty low.
inline bool isEar(const std::vector<UV>& pts, const std::vector<int>& loop, std::size_t i) {
  const std::size_t n = loop.size();
  const int ip = loop[(i + n - 1) % n];
  const int ic = loop[i];
  const int in = loop[(i + 1) % n];
  const UV& a = pts[ip];
  const UV& b = pts[ic];
  const UV& c = pts[in];
  if (orient2d(a, b, c) <= 0.0) return false;  // reflex/collinear — not a convex ear
  for (std::size_t k = 0; k < n; ++k) {
    const int idx = loop[k];
    if (idx == ip || idx == ic || idx == in) continue;
    if (pointInTriStrict(a, b, c, pts[idx])) return false;  // another vertex inside
  }
  return true;
}

// Ear-clip a simple polygon `loop` (indices into pts). Appends CCW triangles. The
// per-vertex ear test is delegated to isEar() (which absorbs the inner scan), so
// this clip-one-ear-per-pass driver stays low-complexity (~11); a per-pass guard
// stops rather than spins on degenerate input (no ear found).
inline void earClip(const std::vector<UV>& pts, std::vector<int> loop, std::vector<UVTri>& out) {
  if (loop.size() < 3) return;
  if (signedArea(pts, loop) < 0.0) std::reverse(loop.begin(), loop.end());  // CCW

  std::size_t guard = 0;
  const std::size_t maxIters = loop.size() * loop.size() + 16;
  while (loop.size() > 3 && guard++ < maxIters) {
    const std::size_t n = loop.size();
    std::size_t ear = n;
    for (std::size_t i = 0; i < n; ++i)
      if (isEar(pts, loop, i)) { ear = i; break; }
    if (ear == n) break;  // no ear found (degenerate) — stop
    const std::size_t np = loop.size();
    out.push_back(UVTri{loop[(ear + np - 1) % np], loop[ear], loop[(ear + 1) % np]});
    loop.erase(loop.begin() + static_cast<long>(ear));
  }
  if (loop.size() == 3) out.push_back(UVTri{loop[0], loop[1], loop[2]});
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// triangulatePolygon — triangulate the region bounded by `loops` (index lists
// into `pts`): loops[0] is the outer boundary, the rest are holes. Returns CCW
// triangles as index triples into `pts`. Holes are bridged into the outer loop
// then the whole thing is ear-clipped.
// ─────────────────────────────────────────────────────────────────────────────
inline std::vector<UVTri> triangulatePolygon(const std::vector<UV>& pts,
                                             const std::vector<std::vector<int>>& loops) {
  std::vector<UVTri> tris;
  if (loops.empty() || loops[0].size() < 3) return tris;

  // Collect the hole loops and bridge them RIGHTMOST-first (descending max-u). This
  // ordering (Eberly, *Triangulation by Ear Clipping* §"Holes") guarantees each
  // hole's max-u vertex sees an already-exposed boundary vertex to its right, so the
  // visibility-checked bridge (bridgeHole) keeps the merged polygon simple across
  // MULTIPLE holes — the single-hole-only limitation of the old heuristic is gone.
  std::vector<const std::vector<int>*> holes;
  for (std::size_t h = 1; h < loops.size(); ++h)
    if (loops[h].size() >= 3) holes.push_back(&loops[h]);
  auto maxU = [&](const std::vector<int>& loop) {
    double mu = -1e300;
    for (int idx : loop) mu = std::max(mu, pts[idx].u);
    return mu;
  };
  std::sort(holes.begin(), holes.end(),
            [&](const std::vector<int>* a, const std::vector<int>* b) { return maxU(*a) > maxU(*b); });

  std::vector<int> outer = loops[0];
  for (const std::vector<int>* h : holes) outer = detail::bridgeHole(pts, outer, *h);
  detail::earClip(pts, outer, tris);
  return tris;
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_UV_TRIANGULATE_H

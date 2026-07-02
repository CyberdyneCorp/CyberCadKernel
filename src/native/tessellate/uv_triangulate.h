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

// Bridge a hole loop into the outer loop by connecting the hole's max-u vertex to
// the nearest outer vertex, so the polygon-with-holes becomes one simple polygon.
// Robust enough for the convex/near-convex holes we mesh (square/round hole in a
// planar face). The hole is spliced traversed opposite to the outer so the merged
// loop stays a single simple boundary.
inline std::vector<int> bridgeHole(const std::vector<UV>& pts, std::vector<int> outer,
                                   std::vector<int> hole) {
  if (hole.size() < 3) return outer;
  if (signedArea(pts, outer) < 0.0) std::reverse(outer.begin(), outer.end());  // outer CCW
  if (signedArea(pts, hole) > 0.0) std::reverse(hole.begin(), hole.end());     // hole CW
  std::size_t hMax = 0;
  for (std::size_t i = 1; i < hole.size(); ++i)
    if (pts[hole[i]].u > pts[hole[hMax]].u) hMax = i;
  const UV& hp = pts[hole[hMax]];
  std::size_t oClosest = 0;
  double best = 1e300;
  for (std::size_t i = 0; i < outer.size(); ++i) {
    const double du = pts[outer[i]].u - hp.u, dv = pts[outer[i]].v - hp.v;
    const double d = du * du + dv * dv;
    if (d < best) { best = d; oClosest = i; }
  }
  std::vector<int> merged;
  merged.reserve(outer.size() + hole.size() + 2);
  for (std::size_t i = 0; i <= oClosest; ++i) merged.push_back(outer[i]);
  const std::size_t hn = hole.size();
  for (std::size_t k = 0; k <= hn; ++k) merged.push_back(hole[(hMax + k) % hn]);
  for (std::size_t i = oClosest; i < outer.size(); ++i) merged.push_back(outer[i]);
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
  std::vector<int> outer = loops[0];
  for (std::size_t h = 1; h < loops.size(); ++h)
    if (loops[h].size() >= 3) outer = detail::bridgeHole(pts, outer, loops[h]);
  detail::earClip(pts, outer, tris);
  return tris;
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_UV_TRIANGULATE_H

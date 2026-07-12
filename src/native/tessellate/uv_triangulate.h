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
// path instead (a tensor grid whose boundary rows use the shared edge samples).
//
// A genuinely-trimmed CURVED FREE-FORM patch (a foreign B-spline / Bézier face with
// an EDGE_LOOP boundary) is the one case that needs BOTH: a boundary-conforming
// triangulation AND interior curvature samples. triangulateConstrained() (below)
// serves it — it folds interior Steiner points into the boundary ear-clip by
// point-location 1→3 subdivision, leaving triangulatePolygon (the planar path)
// byte-identical for its existing callers.
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
#include <unordered_map>
#include <unordered_set>
#include <utility>
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
// `minArea` is the smallest signed twice-area accepted as convex. Pass 0 for the
// strict convex-ear test; pass a small NEGATIVE-exclusive floor of 0 to also admit
// COLLINEAR ears (orient2d == 0) — needed when a straight polygon edge carries many
// collinear boundary samples (a planar loft cap whose straight edges were subdivided
// to match a twisted neighbour): with the strict test no collinear vertex is ever a
// convex ear, so the clip abandons that collinear run and leaves its boundary
// segments open. Clipping a collinear vertex emits a thin (zero-area) triangle whose
// two boundary edges (a-b, b-c) still match the neighbour and whose third edge (a-c)
// is interior to the cap, so the seam stays watertight and the volume is unaffected.
inline bool isEar(const std::vector<UV>& pts, const std::vector<int>& loop, std::size_t i,
                  double minArea) {
  const std::size_t n = loop.size();
  const int ip = loop[(i + n - 1) % n];
  const int ic = loop[i];
  const int in = loop[(i + 1) % n];
  const UV& a = pts[ip];
  const UV& b = pts[ic];
  const UV& c = pts[in];
  if (orient2d(a, b, c) < minArea) return false;  // reflex (or, when minArea>−ε, collinear)
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
// Index of the first clippable ear in `loop`, or loop.size() if none. Two tiers:
// a STRICT convex ear (minArea 0) is preferred; if none exists — a run of COLLINEAR
// boundary samples on a straight, subdivided cap edge — a COLLINEAR ear is accepted
// (minArea just below 0, still rejecting genuinely reflex vertices) so the clip
// keeps making progress instead of abandoning the run and opening the seam.
inline std::size_t findEar(const std::vector<UV>& pts, const std::vector<int>& loop) {
  const std::size_t n = loop.size();
  for (std::size_t i = 0; i < n; ++i)
    if (isEar(pts, loop, i, /*minArea=*/0.0)) return i;
  for (std::size_t i = 0; i < n; ++i)
    if (isEar(pts, loop, i, /*minArea=*/-1e-12)) return i;
  return n;
}

inline void earClip(const std::vector<UV>& pts, std::vector<int> loop, std::vector<UVTri>& out) {
  if (loop.size() < 3) return;
  if (signedArea(pts, loop) < 0.0) std::reverse(loop.begin(), loop.end());  // CCW

  std::size_t guard = 0;
  const std::size_t maxIters = loop.size() * loop.size() + 16;
  while (loop.size() > 3 && guard++ < maxIters) {
    const std::size_t n = loop.size();
    const std::size_t ear = findEar(pts, loop);
    if (ear == n) break;  // truly degenerate — stop
    out.push_back(UVTri{loop[(ear + n - 1) % n], loop[ear], loop[(ear + 1) % n]});
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

// ═════════════════════════════════════════════════════════════════════════════
// ConstrainedDelaunay — incremental constrained-Delaunay triangulator over the UV
// plane, used by the face mesher's trimmed-FREE-FORM path (MOAT M0). A genuinely
// trimmed CURVED patch (a foreign B-spline / Bézier face) needs its INTERIOR sampled
// on the surface, not only its boundary; the pure ear-clip (triangulatePolygon) adds
// no interior points, so a boundary-only mesh leaves the curved interior unresolved
// (chord deflection unbounded — the foreign-patch decline).
//
// WHY DELAUNAY (not ear-clip + Steiner splits). Ear-clip produces long interior
// diagonals (fan chords) that span the region. A 1→3 Steiner split can add interior
// points but NEVER subdivides an existing edge, so the chord deflection ALONG such a
// fan chord persists no matter how many interior points are added — the mesh cannot
// meet the deflection bound and its enclosed volume never converges. A Delaunay
// triangulation has no such spanning chords: an off-surface fan diagonal is
// non-Delaunay once interior points exist near it and is FLIPPED away, so every
// triangle is local and the surface is interpolated between nearby on-surface
// samples. Volume/area then converge with the deflection bound (verified on the
// analytic paraboloid-cap fixture: rel-volume 3.4%→1.1% as deflection halves).
//
// CONSTRUCTION. The base is triangulatePolygon(pts, loops) (unchanged — every
// boundary segment is an edge, so the seam still welds watertight). Every loop edge
// is marked CONSTRAINED and is never flipped, so the boundary segmentation is
// preserved exactly. All interior diagonals are then Lawson-legalized to Delaunay,
// interior Steiner points are inserted (point-location + 1→3 + legalize), and the
// caller refines to the deflection bound (refine()). Interior points are appended to
// the shared `pts`; the caller evaluates every entry as S(u,v).
//
// ROBUSTNESS. A flip fires only on a STRICT incircle violation (scale-relative eps),
// so cocircular points on a regular grid (the incircle-degeneracy the header warns
// of) never oscillate. A flip is also guarded by a convex-quad test, so it is valid
// for a non-convex simple outer polygon, not just the convex slice. A Steiner point
// that lands on an existing edge/vertex (not strictly inside any triangle) is skipped
// rather than producing a sliver. Complexity is systems-band (~30), isolated here.
// ═════════════════════════════════════════════════════════════════════════════
namespace detail {

// In-circle test: >0 ⇔ d is strictly inside the circumcircle of CCW triangle
// (a,b,c). The classic 3×3 determinant on lifted points (translated to d for
// conditioning). Sign is what matters; magnitude scales with the coordinates.
inline double inCircle(const UV& a, const UV& b, const UV& c, const UV& d) noexcept {
  const double ax = a.u - d.u, ay = a.v - d.v;
  const double bx = b.u - d.u, by = b.v - d.v;
  const double cx = c.u - d.u, cy = c.v - d.v;
  return (ax * ax + ay * ay) * (bx * cy - cx * by) -
         (bx * bx + by * by) * (ax * cy - cx * ay) +
         (cx * cx + cy * cy) * (ax * by - bx * ay);
}

// A triangle carrying its three vertex indices (CCW) and the index of the neighbour
// triangle across each edge v[i]→v[(i+1)%3] (−1 if none). This adjacency is what
// makes Lawson legalization O(1) per flip.
struct DTri {
  int v[3];
  int n[3];
};

// Undirected edge key (min,max) → hashable, for the constrained-edge set and the
// neighbour-link build.
inline long long edgeKey(int a, int b) noexcept {
  const int lo = a < b ? a : b, hi = a < b ? b : a;
  return (static_cast<long long>(lo) << 32) | static_cast<unsigned>(hi);
}

class ConstrainedDelaunay {
 public:
  // Build the CDT of the region bounded by `loops` (loops[0] outer; holes[1..]).
  // `pts` is grown in place with any inserted interior points.
  ConstrainedDelaunay(std::vector<UV>& pts, const std::vector<std::vector<int>>& loops)
      : pts_(pts), loops_(loops) {
    for (const std::vector<int>& lp : loops)
      for (std::size_t i = 0; i < lp.size(); ++i)
        constrained_.insert(edgeKey(lp[i], lp[(i + 1) % lp.size()]));
    double span = 0.0;
    for (const std::vector<int>& lp : loops)
      for (int i : lp) span = std::max(span, std::fabs(pts_[i].u) + std::fabs(pts_[i].v));
    eps_ = std::max(span, 1.0) * 1e-11;
    build();
  }

  // Insert one interior Steiner point; skipped if it is not strictly inside the mesh.
  void insert(const UV& p) {
    const int ti = locate(p);
    if (ti < 0) return;
    const int a = tris_[ti].v[0], b = tris_[ti].v[1], c = tris_[ti].v[2];
    const int nab = tris_[ti].n[0], nbc = tris_[ti].n[1], nca = tris_[ti].n[2];
    const int pi = static_cast<int>(pts_.size());
    pts_.push_back(p);
    const int t0 = ti, t1 = static_cast<int>(tris_.size()), t2 = t1 + 1;
    tris_[t0] = DTri{{a, b, pi}, {nab, t1, t2}};
    tris_.push_back(DTri{{b, c, pi}, {nbc, t2, t0}});
    tris_.push_back(DTri{{c, a, pi}, {nca, t0, t1}});
    relink(nab, a, b, t0);
    relink(nbc, b, c, t1);
    relink(nca, c, a, t2);
    legalize(t0, 0);
    legalize(t1, 0);
    legalize(t2, 0);
  }

  // Refine: while a triangle t satisfies needsSplit(t), insert its centroid. Bounded
  // by `maxPasses` and a total-point cap so a pathological curvature cannot explode.
  template <class Pred>
  void refine(Pred needsSplit, int maxPasses, std::size_t maxPts) {
    for (int pass = 0; pass < maxPasses; ++pass) {
      const std::size_t before = pts_.size();
      const std::size_t nt = tris_.size();
      for (std::size_t i = 0; i < nt && pts_.size() < maxPts; ++i) {
        const DTri& t = tris_[i];
        if (needsSplit(t.v[0], t.v[1], t.v[2]))
          insert(UV{(pts_[t.v[0]].u + pts_[t.v[1]].u + pts_[t.v[2]].u) / 3.0,
                    (pts_[t.v[0]].v + pts_[t.v[1]].v + pts_[t.v[2]].v) / 3.0});
      }
      if (pts_.size() == before) break;  // converged (nothing split this pass)
    }
  }

  // Export CCW triangles of the TRIMMED region (between the outer loop and the
  // holes). A holed region is trimmed by a TOPOLOGICAL FLOOD FILL bounded by the
  // constrained loop edges, NOT by a per-triangle centroid-in-hole geometric test.
  //
  // WHY (M0-WELD): the seam-as-hole weld of two curved annuli sharing an inner-hole
  // seam requires the two faces to place the SAME triangle strip on the shared hole
  // boundary. The centroid-in-hole test decides KEEP/DROP per triangle by where its
  // centroid falls relative to the hole polygon — a GEOMETRIC test that, for a thin
  // near-boundary triangle whose centroid sits ~on the hole loop, flips parity when
  // the two annuli bulge opposite ways off the shared flat seam chord (the two faces'
  // interior CDTs disagree by one edge, and the residual GROWS with refinement — the
  // measured per-face-CDT parity gap). The flood fill removes that fragility: the
  // hole loop is a set of CONSTRAINED edges (never flipped), a hard topological wall.
  // A triangle is kept iff it is reachable, across NON-constrained edges only, from a
  // triangle incident to the OUTER boundary loop — so a hole-interior triangle (walled
  // off behind the constrained hole loop) is dropped and a ring triangle is kept, with
  // NO dependence on the centroid's side of the (bulging) hole polygon. Because the
  // decision depends only on the constrained-edge topology — the SAME shared seam loop
  // on both annuli — the two faces cull IDENTICALLY and the shared hole strip welds.
  //
  // A hole-free region (loops_.size()==1) keeps every triangle, unchanged. For a holed
  // region whose base triangulation has no flip artifacts the flood reaches exactly the
  // ring triangles the centroid test kept, so existing holed meshes are unchanged.
  std::vector<UVTri> triangles() const {
    if (loops_.size() <= 1) {
      std::vector<UVTri> out;
      out.reserve(tris_.size());
      for (const DTri& t : tris_) out.push_back(UVTri{t.v[0], t.v[1], t.v[2]});
      return out;
    }
    std::vector<char> keep = floodFillKeep();
    dropConstrainedFolds(keep);
    std::vector<UVTri> out;
    out.reserve(tris_.size());
    for (std::size_t i = 0; i < tris_.size(); ++i)
      if (keep[i]) out.push_back(UVTri{tris_[i].v[0], tris_[i].v[1], tris_[i].v[2]});
    return out;
  }

 private:
  // Topological trim: mark each triangle KEEP iff it is reachable, crossing only
  // NON-constrained edges, from a triangle incident to the outer boundary loop
  // (loops_[0]). Constrained edges (every loop edge) are hard walls, so triangles
  // inside a hole loop are unreachable and dropped. Seeds are triangles that carry an
  // outer-loop directed edge; the annulus ring is connected across its (unconstrained)
  // interior diagonals, so one BFS covers it. If no seed is found (degenerate), keep
  // all triangles — the pre-flood behaviour — so a pathological input never empties.
  std::vector<char> floodFillKeep() const {
    const int nt = static_cast<int>(tris_.size());
    std::vector<char> keep(nt, 0);
    std::vector<int> stack;
    // Seed from triangles incident to an OUTER-loop constrained edge.
    std::unordered_set<long long> outerEdges;
    if (!loops_.empty())
      for (std::size_t i = 0; i < loops_[0].size(); ++i)
        outerEdges.insert(edgeKey(loops_[0][i], loops_[0][(i + 1) % loops_[0].size()]));
    for (int i = 0; i < nt; ++i) {
      const DTri& t = tris_[i];
      bool onOuter = false;
      for (int e = 0; e < 3 && !onOuter; ++e)
        onOuter = outerEdges.count(edgeKey(t.v[e], t.v[(e + 1) % 3])) != 0;
      if (onOuter) { keep[i] = 1; stack.push_back(i); }
    }
    if (stack.empty()) return std::vector<char>(nt, 1);  // degenerate — keep all
    while (!stack.empty()) {
      const int ti = stack.back();
      stack.pop_back();
      const DTri& t = tris_[ti];
      for (int e = 0; e < 3; ++e) {
        const int nb = t.n[e];
        if (nb < 0 || keep[nb]) continue;
        if (constrained_.count(edgeKey(t.v[e], t.v[(e + 1) % 3]))) continue;  // wall
        keep[nb] = 1;
        stack.push_back(nb);
      }
    }
    return keep;
  }

  // Enforce the boundary invariant: every CONSTRAINED loop edge (outer or hole) must
  // border EXACTLY ONE kept triangle. `triangulatePolygon` bridges each hole into the
  // outer loop by a zero-width cut that DOUBLES a vertex and emits a collinear (≈ zero-
  // UV-area) sliver whose edge lies on a hole loop; that sliver shares its hole-loop edge
  // with the real ring triangle beside it, so the hole edge is used by TWO kept triangles
  // in ONE face — and once the two annuli weld across the shared hole seam that edge is
  // used by FOUR triangles (non-manifold). This pass finds every constrained edge used by
  // >1 kept triangle and drops the DEGENERATE one (the collinear sliver, whose 3-D area is
  // zero so area/volume are unchanged), restoring one-triangle-per-boundary-edge so the
  // shared hole seam welds 2-manifold. It NEVER drops a positive-area ring triangle (a
  // clean boundary edge already has exactly one kept neighbour), so a normal holed mesh is
  // unaffected. The two annuli make the SAME decision (both drop their own bridge sliver),
  // so the shared seam stays consistent.
  void dropConstrainedFolds(std::vector<char>& keep) const {
    // Map each constrained edge → the kept triangles bordering it.
    std::unordered_map<long long, std::vector<int>> onEdge;
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i) {
      if (!keep[i]) continue;
      const DTri& t = tris_[i];
      for (int e = 0; e < 3; ++e) {
        const long long k = edgeKey(t.v[e], t.v[(e + 1) % 3]);
        if (constrained_.count(k)) onEdge[k].push_back(i);
      }
    }
    for (auto& [k, tri] : onEdge) {
      if (tri.size() < 2) continue;  // clean boundary edge — one kept neighbour
      // Drop the degenerate (smallest |area|) triangles until one remains.
      std::sort(tri.begin(), tri.end(), [&](int a, int b) {
        return triArea(a) < triArea(b);
      });
      for (std::size_t j = 0; j + 1 < tri.size(); ++j) keep[tri[j]] = 0;
    }
  }

  double triArea(int i) const {
    const DTri& t = tris_[i];
    return std::fabs(orient2d(pts_[t.v[0]], pts_[t.v[1]], pts_[t.v[2]]));
  }

  void build() {
    const std::vector<UVTri> base = triangulatePolygon(pts_, loops_);
    tris_.reserve(base.size());
    for (const UVTri& t : base) tris_.push_back(DTri{{t.a, t.b, t.c}, {-1, -1, -1}});
    // Neighbour links via a directed-edge map: triangle j across edge (a,b) is the
    // one carrying the opposite directed edge (b,a).
    std::unordered_map<long long, std::pair<int, int>> dirEdge;  // (a<<32|b) → (tri,edge)
    dirEdge.reserve(tris_.size() * 3);
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i)
      for (int e = 0; e < 3; ++e)
        dirEdge[dkey(tris_[i].v[e], tris_[i].v[(e + 1) % 3])] = {i, e};
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i)
      for (int e = 0; e < 3; ++e) {
        const auto it = dirEdge.find(dkey(tris_[i].v[(e + 1) % 3], tris_[i].v[e]));
        if (it != dirEdge.end()) tris_[i].n[e] = it->second.first;
      }
    // Legalize every interior edge of the ear-clip base to reach the CDT.
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i)
      for (int e = 0; e < 3; ++e) legalize(i, e);
  }

  static long long dkey(int a, int b) noexcept {
    return (static_cast<long long>(a) << 32) | static_cast<unsigned>(b);
  }

  // Point-location: the triangle strictly containing p (all three orient2d ≥ 0 for a
  // CCW triangle, with an eps margin so a point on an edge is rejected). Linear scan
  // — the interior grids are modest and this keeps the code degeneracy-free.
  int locate(const UV& p) const {
    for (int i = 0; i < static_cast<int>(tris_.size()); ++i) {
      const DTri& t = tris_[i];
      if (orient2d(pts_[t.v[0]], pts_[t.v[1]], p) > eps_ &&
          orient2d(pts_[t.v[1]], pts_[t.v[2]], p) > eps_ &&
          orient2d(pts_[t.v[2]], pts_[t.v[0]], p) > eps_)
        return i;
    }
    return -1;
  }

  int edgeOf(int ti, int a, int b) const {
    for (int e = 0; e < 3; ++e)
      if (tris_[ti].v[e] == a && tris_[ti].v[(e + 1) % 3] == b) return e;
    return -1;
  }

  // Repoint neighbour `nb`'s link on undirected edge (a,b) to triangle `to`.
  void relink(int nb, int a, int b, int to) {
    if (nb < 0) return;
    int e = edgeOf(nb, b, a);
    if (e < 0) e = edgeOf(nb, a, b);
    if (e >= 0) tris_[nb].n[e] = to;
  }

  // Lawson legalize edge e=(a,b) of triangle ti: if it is unconstrained and the
  // neighbour's apex is strictly inside ti's circumcircle AND the flip is a valid
  // convex-quad flip, flip a-b → apex-opp and recurse on the two exposed edges.
  void legalize(int ti, int e) {
    if (ti < 0) return;
    const int a = tris_[ti].v[e], b = tris_[ti].v[(e + 1) % 3], apex = tris_[ti].v[(e + 2) % 3];
    if (constrained_.count(edgeKey(a, b))) return;
    const int nb = tris_[ti].n[e];
    if (nb < 0) return;
    const int e2 = edgeOf(nb, b, a);
    if (e2 < 0) return;
    const int opp = tris_[nb].v[(e2 + 2) % 3];
    if (inCircle(pts_[a], pts_[b], pts_[apex], pts_[opp]) <= eps_) return;  // Delaunay: keep
    // Convex-quad guard: the flip diagonal apex→opp must cross the current edge a→b,
    // i.e. a and b lie on STRICTLY OPPOSITE sides of line apex-opp. (Orientation-
    // agnostic — a non-convex quad near a concave boundary would give same-side and
    // must NOT be flipped, else the new diagonal leaves the region.)
    const double sa = orient2d(pts_[apex], pts_[opp], pts_[a]);
    const double sb = orient2d(pts_[apex], pts_[opp], pts_[b]);
    if ((sa > 0.0) == (sb > 0.0) || sa == 0.0 || sb == 0.0) return;
    // Neighbour links around the two triangles (before the flip).
    const int nA = tris_[ti].n[(e + 1) % 3];   // across (b,apex)
    const int nB = tris_[ti].n[(e + 2) % 3];   // across (apex,a)
    const int nC = tris_[nb].n[(e2 + 1) % 3];  // across (a,opp)
    const int nD = tris_[nb].n[(e2 + 2) % 3];  // across (opp,b)
    tris_[ti] = DTri{{apex, a, opp}, {nB, nC, nb}};
    tris_[nb] = DTri{{apex, opp, b}, {ti, nD, nA}};
    relink(nB, apex, a, ti);
    relink(nC, a, opp, ti);
    relink(nD, opp, b, nb);
    relink(nA, b, apex, nb);
    legalize(ti, 1);  // new edge (a,opp)
    legalize(nb, 1);  // new edge (opp,b)
  }

  std::vector<UV>& pts_;
  const std::vector<std::vector<int>>& loops_;
  std::vector<DTri> tris_;
  std::unordered_set<long long> constrained_;
  double eps_ = 1e-11;

  // Even-odd point-in-polygon over an index loop (for the hole drop test).
  bool pipImpl(const std::vector<int>& loop, const UV& p) const {
    const std::size_t n = loop.size();
    if (n < 3) return false;
    bool in = false;
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
      const UV& A = pts_[loop[i]];
      const UV& B = pts_[loop[j]];
      if ((A.v > p.v) != (B.v > p.v)) {
        const double x = (B.u - A.u) * (p.v - A.v) / (B.v - A.v) + A.u;
        if (p.u < x) in = !in;
      }
    }
    return in;
  }
};

}  // namespace detail

// triangulateConstrained — CDT of the region bounded by `loops` with `interior`
// Steiner points folded in (see detail::ConstrainedDelaunay). `pts` is grown with the
// interior points that were actually inserted. This is the interior-sampling
// counterpart of triangulatePolygon; the planar path keeps calling triangulatePolygon
// unchanged. Refinement to a deflection bound is driven by the caller via the class's
// refine() (used directly by the face mesher, which owns the surface evaluator).
inline std::vector<UVTri> triangulateConstrained(std::vector<UV>& pts,
                                                 const std::vector<std::vector<int>>& loops,
                                                 const std::vector<UV>& interior) {
  if (loops.empty() || loops[0].size() < 3) return {};
  detail::ConstrainedDelaunay cdt(pts, loops);
  for (const UV& g : interior) cdt.insert(g);
  return cdt.triangles();
}

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_UV_TRIANGULATE_H

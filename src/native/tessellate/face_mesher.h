// SPDX-License-Identifier: Apache-2.0
//
// face_mesher.h — STAGE 2 of the two-stage watertight solid mesher: triangulate
// one trimmed Face whose BOUNDARY is pinned to the SHARED per-edge discretization
// (edge_mesher.h) and whose INTERIOR is sampled on the surface parameter grid.
//
// ── TWO-STAGE DESIGN (why this changed) ──────────────────────────────────────
// The historical face mesher meshed each face on an INDEPENDENT (u,v) grid and
// relied on spatial welding to stitch neighbours. That welds only STRAIGHT shared
// edges (grid nodes coincide); a CURVED shared edge (cylinder cap↔side circle,
// fillet-blend seams) was sampled differently by each face, so the solid was left
// OPEN along the seam (the "2-manifold-bounded-open" limitation). The fix mirrors
// OCCT BRepMesh: discretize each unique EDGE once (STAGE 1, edge_mesher.h) and
// have BOTH adjacent faces place their boundary vertices at those SAME fractions.
// Because a pcurve satisfies S_face(pcurve(f)) = C_edge(f), the two faces produce
// the SAME 3D boundary points at each shared fraction — so the welded solid is
// watertight even across a CURVED shared edge.
//
// ── TWO MESHING PATHS (chosen per face) ──────────────────────────────────────
//   * STRUCTURED-GRID path — for a face whose outer boundary is the full
//     parametric RECTANGLE (an untrimmed primitive: a whole cylinder side, a whole
//     sphere). A tensor (u,v) grid is built whose grid LINES include every u and v
//     coordinate appearing on the boundary (i.e. the shared edge samples), then
//     refined between them by the deflection metric. The four boundary rows/columns
//     are therefore exactly the shared edge samples ⇒ neighbours weld watertight;
//     the interior lines carry the curvature sampling. Quad → two triangles.
//   * EAR-CLIP path — for a genuinely trimmed face (a cap disk, a holed planar
//     face, a fillet blend patch). The boundary polygon (shared edge samples, plus
//     bridged holes) is ear-clipped: every boundary segment is a triangle edge, so
//     neighbours weld; a planar patch needs no interior points (flat), and a curved
//     trimmed patch is refined by inserting the boundary loop densely enough via
//     STAGE 1. (uv_triangulate.h)
// A face with NO usable pcurves falls back to the surface's natural UV box as a
// structured grid — unchanged behaviour for such faces.
//
// ── DEFLECTION METRIC (the correctness knob, unchanged) ──────────────────────
// Interior grid divisions come from the sagitta law Δ ≤ √(8·deflection/‖S″‖) with
// ‖Sᵤᵤ‖,‖Sᵥᵥ‖ probed on a 3×3 UV lattice (surface_eval curvatureMagnitude). A
// planar patch collapses to minDiv; a tight blend subdivides until the chord error
// is under the bound. Every produced vertex is S(u,v) (on the true surface).
//
// Cognitive complexity: mesh() delegates to buildBoundary / structuredGrid /
// earClipMesh helpers (driver ≤ ~12); the triangulation itself is isolated in
// uv_triangulate.h (systems band, flagged there). OCCT-FREE. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_TESSELLATE_FACE_MESHER_H
#define CYBERCAD_NATIVE_TESSELLATE_FACE_MESHER_H

#include "native/tessellate/edge_mesher.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/tessellate/uv_triangulate.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace cybercad::native::tessellate {

/// Tessellation parameters. `deflection` is the max allowed chord distance from a
/// triangle to the true surface (model units). Divisions are clamped so a flat
/// face stays cheap and a tight blend cannot explode.
struct MeshParams {
  double deflection = 0.1;  ///< linear deflection bound (model units)
  int minDiv = 1;           ///< min grid divisions per direction
  int maxDiv = 256;         ///< max grid divisions per direction (safety clamp)
  int wireSegsPerEdge = 24; ///< legacy pcurve samples per edge (untrimmed fallback)
  int trimMinDiv = 16;      ///< min interior grid divisions per direction (trimmed)
  int edgeMinSegs = 1;      ///< min segments per shared edge discretization
  int edgeMaxSegs = 256;    ///< max segments per shared edge discretization
};

namespace detail {

/// Choose divisions in one direction from the deflection bound and the worst
/// second-derivative magnitude seen along that direction. Δ = √(8·defl/‖S″‖);
/// n = ceil(span/Δ), clamped to [minDiv, maxDiv].
inline int divisionsFor(double span, double d2, const MeshParams& p) noexcept {
  if (span <= 0.0) return p.minDiv;
  if (d2 <= 1e-12) return p.minDiv;  // ~flat in this direction
  const double step = std::sqrt(8.0 * p.deflection / d2);
  if (!(step > 0.0)) return p.maxDiv;
  const int n = static_cast<int>(std::ceil(span / step));
  return std::clamp(n, p.minDiv, p.maxDiv);
}

/// Worst {‖Sᵤᵤ‖, ‖Sᵥᵥ‖} over a 3×3 UV lattice — a conservative step basis.
inline std::array<double, 2> worstCurvature(const SurfaceEvaluator& eval, double uMin, double uMax,
                                            double vMin, double vMax) noexcept {
  double d2u = 0.0, d2v = 0.0;
  for (int i = 0; i <= 2; ++i)
    for (int j = 0; j <= 2; ++j) {
      const double u = uMin + (uMax - uMin) * (i * 0.5);
      const double v = vMin + (vMax - vMin) * (j * 0.5);
      const auto c = eval.curvatureMagnitude(u, v);
      d2u = std::max(d2u, c[0]);
      d2v = std::max(d2v, c[1]);
    }
  return {d2u, d2v};
}

// Merge `extra` sorted-unique coordinates into `axis` (both sorted); dedup within
// `tol`. Used to fold the boundary sample coordinates into the curvature grid so a
// structured grid's boundary rows land exactly on the shared edge samples.
inline void mergeAxis(std::vector<double>& axis, const std::vector<double>& extra, double tol) {
  for (double x : extra) axis.push_back(x);
  std::sort(axis.begin(), axis.end());
  std::vector<double> out;
  for (double x : axis)
    if (out.empty() || x - out.back() > tol) out.push_back(x);
  axis.swap(out);
}

}  // namespace detail

// ─────────────────────────────────────────────────────────────────────────────
// FaceMesher — mesh one face. Stateless apart from the parameters. mesh() with an
// EdgeCache uses the shared per-edge discretization (STAGE 2, watertight);
// mesh(face) without one builds a throwaway cache (single-face host callers).
// ─────────────────────────────────────────────────────────────────────────────
class FaceMesher {
 public:
  explicit FaceMesher(MeshParams params = {}) noexcept : p_(params) {}

  /// Tessellate `face` using a throwaway edge cache (single-face use).
  Mesh mesh(const topo::Shape& face) const {
    EdgeCache cache(p_.deflection, p_.edgeMinSegs, p_.edgeMaxSegs);
    return mesh(face, cache);
  }

  /// Tessellate `face` with a shared `cache`. Empty mesh if the face has no surface.
  Mesh mesh(const topo::Shape& face, EdgeCache& cache) const {
    const auto sr = topo::surfaceOf(face);
    if (!sr) return {};
    SurfaceEvaluator eval(*sr->surface, sr->location);

    const std::vector<UVPolygon> loops = buildBoundaryLoops(face, cache);
    const UVRegion region = regionFromLoops(loops);
    const UVBox box = domainBox(eval, region);
    const bool flip = face.orientation() == topo::Orientation::Reversed;

    // No usable boundary (no pcurves) ⇒ structured grid over the natural bounds.
    if (!region.hasOuter()) return structuredGrid(eval, region, box, flip, /*hasBoundary=*/false);

    // A full-parametric-rectangle outer loop (no holes) ⇒ structured grid whose
    // boundary rows use the shared edge samples. Otherwise ear-clip the polygon.
    //
    // For a PLANAR face, require the loop to hit all four box corners: a convex
    // polygon cap (triangle / hexagon extrude cap) has every vertex on the bbox
    // border but fills only part of the box, so it must NOT take the full-rectangle
    // fast path (it would mesh as the whole bbox and break volume / watertightness)
    // — the corner test routes it to ear-clip. Curved full-parametric faces
    // (cylinder / sphere / cone), whose degenerate/seam boundaries lack distinct box
    // corners, are admitted by the border test alone (requireCorners = false).
    const bool planar = sr->surface->kind == topo::FaceSurface::Kind::Plane;
    if (region.holes.empty() && region.isFullRectangle(1e-4, /*requireCorners=*/planar))
      return structuredGrid(eval, region, box, flip, /*hasBoundary=*/true);

    return earClipMesh(eval, loops, flip);
  }

 private:
  // ── STAGE-2 boundary: each wire flattened at the shared edge fractions ───────
  std::vector<UVPolygon> buildBoundaryLoops(const topo::Shape& face, EdgeCache& cache) const {
    std::vector<UVPolygon> loops;
    if (face.isNull() || face.type() != topo::ShapeType::Face) return loops;
    for (const topo::Shape& wire : face.tshape()->children())
      loops.push_back(flattenWireShared(wire, face, cache));
    return loops;
  }

  // Flatten one wire to a UV polygon using the shared per-edge fraction list.
  UVPolygon flattenWireShared(const topo::Shape& wire, const topo::Shape& face,
                              EdgeCache& cache) const {
    UVPolygon poly;
    if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return poly;
    for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
      const topo::Shape& edge = ex.current();
      const topo::PCurve* pc = pcurveForFace(edge, face);
      if (!pc) continue;
      const EdgeDiscretization& d = cache.discretize(edge);
      appendEdgeSamplesAtFracs(poly, edge, *pc, d.fracs);
    }
    return poly;
  }

  static UVRegion regionFromLoops(const std::vector<UVPolygon>& loops) {
    UVRegion region;
    for (std::size_t i = 0; i < loops.size(); ++i) {
      if (i == 0)
        region.outer = loops[i];
      else if (loops[i].size() >= 3)
        region.holes.push_back(loops[i]);
    }
    for (const UV& p : region.outer) region.box.expand(p);
    return region;
  }

  static UVBox domainBox(const SurfaceEvaluator& eval, const UVRegion& region) {
    if (region.hasOuter() && region.box.valid) return region.box;
    const UVBounds b = eval.bounds();
    return UVBox{b.uMin, b.uMax, b.vMin, b.vMax, true};
  }

  // ── STRUCTURED-GRID path (full-parametric-rectangle faces) ───────────────────
  // Build a tensor (u,v) grid whose lines include every boundary u/v coordinate
  // (the shared edge samples, when hasBoundary) folded into a curvature-refined
  // grid, then emit two triangles per quad. Boundary rows/columns are exactly the
  // shared edge samples ⇒ neighbours weld watertight.
  Mesh structuredGrid(const SurfaceEvaluator& eval, const UVRegion& region, const UVBox& box,
                      bool flip, bool hasBoundary) const {
    const std::vector<double> us = axisSamples(eval, box, region, /*uDir=*/true, hasBoundary);
    const std::vector<double> vs = axisSamples(eval, box, region, /*uDir=*/false, hasBoundary);
    const int cols = static_cast<int>(vs.size());

    Mesh m;
    m.vertices.reserve(us.size() * vs.size());
    m.normals.reserve(us.size() * vs.size());
    for (double u : us)
      for (double v : vs) {
        const SurfaceSample s = eval.d1(u, v);
        m.vertices.push_back(s.point);
        m.normals.push_back(flip ? s.normal.reversed() : s.normal);
      }
    for (int i = 0; i + 1 < static_cast<int>(us.size()); ++i)
      for (int j = 0; j + 1 < cols; ++j) {
        const auto v00 = static_cast<std::uint32_t>(i * cols + j);
        const auto v01 = static_cast<std::uint32_t>(i * cols + (j + 1));
        const auto v10 = static_cast<std::uint32_t>((i + 1) * cols + j);
        const auto v11 = static_cast<std::uint32_t>((i + 1) * cols + (j + 1));
        if (flip) { m.addTriangle(v00, v11, v10); m.addTriangle(v00, v01, v11); }
        else      { m.addTriangle(v00, v10, v11); m.addTriangle(v00, v11, v01); }
      }
    return m;
  }

  // Sorted-unique sample coordinates for one axis: a curvature-driven uniform grid
  // over [lo,hi], with the boundary coordinates on that axis merged in (so the grid
  // lines land on the shared edge samples). uDir selects u vs v.
  std::vector<double> axisSamples(const SurfaceEvaluator& eval, const UVBox& box,
                                  const UVRegion& region, bool uDir, bool hasBoundary) const {
    const double lo = uDir ? box.uMin : box.vMin;
    const double hi = uDir ? box.uMax : box.vMax;
    const auto d2 = detail::worstCurvature(eval, box.uMin, box.uMax, box.vMin, box.vMax);
    const int n = detail::divisionsFor(hi - lo, uDir ? d2[0] : d2[1], p_);
    std::vector<double> axis;
    axis.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i)
      axis.push_back(lo + (hi - lo) * (static_cast<double>(i) / n));
    if (hasBoundary) {
      std::vector<double> bnd = boundaryCoords(region, uDir);
      const double tol = std::max(hi - lo, 1.0) * 1e-7;
      detail::mergeAxis(axis, bnd, tol);
    }
    return axis;
  }

  // Distinct coordinates that the outer boundary places on the given axis (the
  // shared edge samples along the two edges parallel to that axis). For a
  // rectangle these are the u-samples of the v=const edges (uDir) or the v-samples
  // of the u=const edges.
  static std::vector<double> boundaryCoords(const UVRegion& region, bool uDir) {
    std::vector<double> c;
    c.reserve(region.outer.size());
    for (const UV& p : region.outer) c.push_back(uDir ? p.u : p.v);
    std::sort(c.begin(), c.end());
    return c;
  }

  // ── EAR-CLIP path (genuinely trimmed faces) ──────────────────────────────────
  Mesh earClipMesh(const SurfaceEvaluator& eval, const std::vector<UVPolygon>& loops,
                   bool flip) const {
    std::vector<UV> pts;
    const std::vector<std::vector<int>> loopIdx = appendBoundaryLoops(loops, pts);
    if (loopIdx.empty() || loopIdx[0].size() < 3) return {};
    const std::vector<UVTri> tris = triangulatePolygon(pts, loopIdx);

    Mesh m = evaluatePoints(eval, pts, flip);
    m.triangles.reserve(tris.size());
    for (const UVTri& t : tris) {
      const auto ia = static_cast<std::uint32_t>(t.a);
      const auto ib = static_cast<std::uint32_t>(t.b);
      const auto ic = static_cast<std::uint32_t>(t.c);
      if (flip) m.addTriangle(ia, ic, ib); else m.addTriangle(ia, ib, ic);
    }
    return m;
  }

  // Append each loop's UNIQUE vertices to `pts`, returning the index range per
  // loop. The flattened wire is an open polyline whose ends may coincide (closed
  // loop); the duplicated closing vertex is dropped so each loop is a clean cycle.
  static std::vector<std::vector<int>> appendBoundaryLoops(const std::vector<UVPolygon>& loops,
                                                           std::vector<UV>& pts) {
    std::vector<std::vector<int>> out;
    for (const UVPolygon& loop : loops) {
      std::size_t count = loop.size();
      if (count >= 2 && nearlyEqual(loop.front(), loop.back())) --count;
      if (count < 3) { out.emplace_back(); continue; }  // degenerate loop (seam)
      std::vector<int> idx;
      idx.reserve(count);
      for (std::size_t i = 0; i < count; ++i) {
        idx.push_back(static_cast<int>(pts.size()));
        pts.push_back(loop[i]);
      }
      out.push_back(std::move(idx));
    }
    return out;
  }

  static bool nearlyEqual(const UV& a, const UV& b) noexcept {
    return std::fabs(a.u - b.u) < 1e-9 && std::fabs(a.v - b.v) < 1e-9;
  }

  // Evaluate every UV point on the surface → 3D vertex + normal (flipped normal
  // for a Reversed face so the solid's outward normal is consistent).
  static Mesh evaluatePoints(const SurfaceEvaluator& eval, const std::vector<UV>& pts, bool flip) {
    Mesh m;
    m.vertices.reserve(pts.size());
    m.normals.reserve(pts.size());
    for (const UV& q : pts) {
      const SurfaceSample s = eval.d1(q.u, q.v);
      m.vertices.push_back(s.point);
      m.normals.push_back(flip ? s.normal.reversed() : s.normal);
    }
    return m;
  }

  MeshParams p_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_FACE_MESHER_H

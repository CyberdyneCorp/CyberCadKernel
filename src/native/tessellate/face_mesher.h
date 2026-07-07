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
#include <unordered_map>
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

/// Worst effective per-direction second-derivative over a 3×3 UV lattice — a
/// conservative step basis. When `foldTwist` is set, each direction folds in the
/// MIXED term ‖Sᵤᵥ‖ scaled by the OPPOSITE parameter span: a twisted/ruled saddle
/// patch has Sᵤᵤ = Sᵥᵥ = 0 but ‖Sᵤᵥ‖ ≠ 0, and its chord sagitta off the flat quad
/// grows like ‖Sᵤᵥ‖·Δu·Δv, so treating ‖Sᵤᵥ‖·span_other as an effective directional
/// curvature makes divisionsFor subdivide the twist (otherwise it meshes as one flat
/// quad and the enclosed volume of a twisted loft is wrong). Only free-form
/// (Bezier/BSpline) faces set foldTwist: analytic faces (sphere/cone) are already
/// sized correctly by their Sᵤᵤ/Sᵥᵥ, and folding their (coupled) Sᵤᵥ in would
/// needlessly over-subdivide them.
inline std::array<double, 2> worstCurvature(const SurfaceEvaluator& eval, double uMin, double uMax,
                                            double vMin, double vMax,
                                            bool foldTwist = false) noexcept {
  const double uSpan = std::max(uMax - uMin, 0.0);
  const double vSpan = std::max(vMax - vMin, 0.0);
  double d2u = 0.0, d2v = 0.0;
  for (int i = 0; i <= 2; ++i)
    for (int j = 0; j <= 2; ++j) {
      const double u = uMin + (uMax - uMin) * (i * 0.5);
      const double v = vMin + (vMax - vMin) * (j * 0.5);
      const auto c = eval.curvatureMagnitude(u, v);
      d2u = std::max(d2u, foldTwist ? std::max(c[0], c[2] * vSpan) : c[0]);
      d2v = std::max(d2v, foldTwist ? std::max(c[1], c[2] * uSpan) : c[1]);
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

// ── Canonical boundary anchors (build-order-independent seam points) ──────────
// A spatial index of the CANONICAL 3D points a face places on its straight
// boundary seams (edge_mesher canonicalLinePoint). Two faces sharing a straight
// seam evaluate that seam through their own surfaces and land on 3D points that
// agree only to ~1 ULP; when such a point falls on a weld-grid cell boundary the
// two copies round to opposite cells and the weld leaves the seam OPEN (the
// per-turn thread residual). The mesher therefore SNAPS every vertex that lands
// on a seam to the canonical point: since both faces canonicalise to the SAME
// ordered endpoints, they snap to BIT-IDENTICAL points and the conservative
// single-cell weld fuses them — no widening of the merge radius (which would
// over-collapse a fine curvature grid).
//
// Lookup is by the surface-evaluated 3D vertex, quantised to a grid FAR finer
// than any real feature (kQuantum) yet coarser than the ~1-ULP seam jitter, so a
// vertex and its canonical anchor share a key. The 3×3×3 neighbour probe absorbs
// a point that straddles a quantum boundary; the epsilon check guards against
// snapping an unrelated nearby vertex.
struct BoundaryAnchors {
  static constexpr double kQuantum = 1e-7;   ///< index cell size (≫ ULP, ≪ feature)
  static constexpr double kSnapEps = 1e-6;   ///< max dist a vertex may be snapped

  struct Key {
    long long x, y, z;
    bool operator==(const Key& o) const noexcept { return x == o.x && y == o.y && z == o.z; }
  };
  struct Hash {
    std::size_t operator()(const Key& k) const noexcept {
      std::size_t h = static_cast<std::size_t>(k.x) * 73856093u;
      h ^= static_cast<std::size_t>(k.y) * 19349663u;
      h ^= static_cast<std::size_t>(k.z) * 83492791u;
      return h;
    }
  };
  std::unordered_map<Key, math::Point3, Hash> pts;

  static long long q(double v) noexcept {
    const double s = 1.0 / kQuantum;
    return static_cast<long long>(v >= 0 ? v * s + 0.5 : v * s - 0.5);
  }
  Key keyOf(const math::Point3& p) const noexcept { return Key{q(p.x), q(p.y), q(p.z)}; }

  // Record a canonical seam point (idempotent — the same point from another edge
  // simply re-inserts the identical value).
  void add(const math::Point3& p) { pts.emplace(keyOf(p), p); }

  // The canonical point coincident with `p` (within kSnapEps), or nullptr.
  const math::Point3* find(const math::Point3& p) const {
    if (pts.empty()) return nullptr;
    const Key c = keyOf(p);
    for (int dx = -1; dx <= 1; ++dx)
      for (int dy = -1; dy <= 1; ++dy)
        for (int dz = -1; dz <= 1; ++dz) {
          const auto it = pts.find(Key{c.x + dx, c.y + dy, c.z + dz});
          if (it != pts.end() && math::distance(p, it->second) <= kSnapEps) return &it->second;
        }
    return nullptr;
  }
  bool empty() const noexcept { return pts.empty(); }
};

}  // namespace detail

using detail::BoundaryAnchors;

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

    // Canonical boundary anchors: for every straight boundary edge, the
    // build-order-independent 3D point at each shared fraction, keyed by the
    // sample's (u,v). Any grid/ear-clip vertex that lands on the boundary is
    // snapped to its anchor so two faces sharing a straight seam place
    // BIT-IDENTICAL boundary points (see edge_mesher CanonicalEndpoints) and the
    // spatial weld cannot split them across a cell boundary.
    BoundaryAnchors anchors;
    const std::vector<UVPolygon> loops = buildBoundaryLoops(face, cache, anchors);
    const UVRegion region = regionFromLoops(loops);
    const UVBox box = domainBox(eval, region);
    const bool flip = face.orientation() == topo::Orientation::Reversed;

    // Free-form (Bezier/BSpline) faces may be TWISTED (ruled loft side patches):
    // their straight boundary edges are subdivided by the solid-mesher pre-pass and
    // the structured grid must be driven by those boundary samples (see axisSamples).
    // Analytic faces (Plane/Cylinder/Cone/Sphere) keep the historical curvature grid.
    const topo::FaceSurface::Kind k = sr->surface->kind;
    const bool freeForm =
        k == topo::FaceSurface::Kind::Bezier || k == topo::FaceSurface::Kind::BSpline;

    // No usable boundary (no pcurves) ⇒ structured grid over the natural bounds.
    if (!region.hasOuter())
      return structuredGrid(eval, region, box, flip, /*hasBoundary=*/false, freeForm, anchors);

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
    const bool planar = k == topo::FaceSurface::Kind::Plane;
    if (region.holes.empty() && region.isFullRectangle(1e-4, /*requireCorners=*/planar))
      return structuredGrid(eval, region, box, flip, /*hasBoundary=*/true, freeForm, anchors);

    // ── ADDITIVE trimmed-FREE-FORM arm (MOAT M0) ──────────────────────────────
    // A genuinely-trimmed CURVED free-form (Bézier/B-spline) face — a foreign patch
    // bounded by a real EDGE_LOOP, NOT the full parametric rectangle — reaches here.
    // The pure ear-clip path (below) samples only its BOUNDARY, so its curved
    // interior is never sampled and the chord deflection is unbounded (a foreign
    // rational-B-spline STEP patch declines for exactly this reason). trimmedFreeform
    // Mesh() reuses the SAME boundary (flattenWireShared + anchors) as the ear-clip,
    // then folds interior curvature samples into it (triangulateConstrained) and
    // refines to the deflection bound. This arm is reachable ONLY by this case:
    // our own bare-periodic B-spline faces are the full parametric rectangle
    // (structured-grid path above); analytic primitives and planar trims never set
    // freeForm here — so every existing mesh is byte-identical (see face_mesher tests).
    if (freeForm)
      return trimmedFreeformMesh(eval, loops, region, flip, anchors);

    return earClipMesh(eval, loops, flip, anchors);
  }

  // ── Edge pre-sizing (twisted-face support) ───────────────────────────────────
  // Report, into `cache`, the MINIMUM segment count each boundary edge of this
  // face needs so the face's chord deflection is met. For a TWISTED free-form face
  // (a ruled loft side patch: Sᵤᵤ = Sᵥᵥ = 0 but ‖Sᵤᵥ‖ ≠ 0) the deflection is
  // driven by the twist, so its STRAIGHT boundary edges must be subdivided — the
  // edge's own 3D curvature (zero) would leave it at one segment and open the seam
  // against the subdivided interior. Planar/analytic-only faces demand nothing
  // extra here (their straight edges stay at the edge's own sizing). Each boundary
  // edge is assigned the division count of the parameter axis it runs along.
  void requireEdgeSegments(const topo::Shape& face, EdgeCache& cache) const {
    const auto sr = topo::surfaceOf(face);
    if (!sr) return;
    // Only free-form (ruled/skinned) faces carry twist the edge sizing misses.
    const topo::FaceSurface::Kind k = sr->surface->kind;
    if (k != topo::FaceSurface::Kind::Bezier && k != topo::FaceSurface::Kind::BSpline) return;

    SurfaceEvaluator eval(*sr->surface, sr->location);
    const UVBounds b = eval.bounds();
    const auto d2 = detail::worstCurvature(eval, b.uMin, b.uMax, b.vMin, b.vMax, /*foldTwist=*/true);
    const int nu = detail::divisionsFor(b.uMax - b.uMin, d2[0], p_);
    const int nv = detail::divisionsFor(b.vMax - b.vMin, d2[1], p_);
    if (nu <= 1 && nv <= 1) return;

    for (const topo::Shape& wire : face.tshape()->children())
      for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
        const topo::Shape& edge = ex.current();
        const topo::PCurve* pc = pcurveForFace(edge, face);
        if (!pc) continue;
        // The pcurve direction tells which axis the edge runs along: |du| vs |dv|.
        const bool alongU = std::fabs(pc->dir2d.x) >= std::fabs(pc->dir2d.y);
        cache.requireMinSegs(edge, alongU ? nu : nv);
      }
  }

 private:
  // ── STAGE-2 boundary: each wire flattened at the shared edge fractions ───────
  std::vector<UVPolygon> buildBoundaryLoops(const topo::Shape& face, EdgeCache& cache,
                                            BoundaryAnchors& anchors) const {
    std::vector<UVPolygon> loops;
    if (face.isNull() || face.type() != topo::ShapeType::Face) return loops;
    for (const topo::Shape& wire : face.tshape()->children())
      loops.push_back(flattenWireShared(wire, face, cache, anchors));
    return loops;
  }

  // Flatten one wire to a UV polygon using the shared per-edge fraction list, and
  // record a canonical 3D anchor per straight-edge sample so the seam welds
  // exactly (see BoundaryAnchors).
  UVPolygon flattenWireShared(const topo::Shape& wire, const topo::Shape& face, EdgeCache& cache,
                              BoundaryAnchors& anchors) const {
    UVPolygon poly;
    if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return poly;
    for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
      const topo::Shape& edge = ex.current();
      const topo::PCurve* pc = pcurveForFace(edge, face);
      if (!pc) continue;
      const EdgeDiscretization& d = cache.discretize(edge);
      appendEdgeSamplesAtFracs(poly, edge, *pc, d.fracs);
      recordEdgeAnchors(anchors, edge, d);
    }
    return poly;
  }

  // Record the canonical world point at every shared sample of a boundary edge, so a
  // seam-lying vertex of EITHER adjacent face snaps to the same point and the spatial
  // weld cannot split the two across a cell boundary.
  //
  //   * STRAIGHT edge — the samples are generated at the canonical INDICES i/n in the
  //     fixed a→b endpoint order (NOT the edge's wire-traversal fractions). Two
  //     coincident straight edges built with opposite vertex order sample at
  //     COMPLEMENTARY fractions (i/n vs (n−i)/n); mapping one as `1−f` reintroduces a
  //     1-ULP difference. Generating both from the SAME i/n sequence over the SAME
  //     canonical endpoints makes the anchor SETS bit-identical.
  //   * CURVED edge (Circle/BSpline/…) — the shared EdgeDiscretization already holds the
  //     canonical 3D sample POINTS (`d.points`), computed once from the curve itself
  //     (deterministic, build-order-independent as a SET). When two faces share a curved
  //     seam through the SAME edge node these already agree; but when a builder gives the
  //     two faces SEPARATE edge nodes carrying the SAME curve (e.g. a spline extrude cap
  //     ↔ its B-spline side wall, evaluated through DIFFERENT surface types), the two
  //     faces' surface evaluations of the boundary differ by ~1 ULP and the seam splits.
  //     Recording d.points as anchors makes both faces snap their curved-seam vertices to
  //     the identical shared points ⇒ watertight (the curved-edge analogue of the
  //     straight-edge canonical anchor).
  static void recordEdgeAnchors(BoundaryAnchors& anchors, const topo::Shape& edge,
                                const EdgeDiscretization& d) {
    const CanonicalEndpoints ce = detail::canonicalLineEndpoints(edge);
    if (ce.valid && d.fracs.size() >= 2) {
      const int n = static_cast<int>(d.fracs.size()) - 1;  // segment count (endpoint-shared)
      const math::Vec3 dir = ce.b - ce.a;
      for (int i = 0; i <= n; ++i) {
        const double g = static_cast<double>(i) / static_cast<double>(n);
        anchors.add(math::Point3{ce.a.x + dir.x * g, ce.a.y + dir.y * g, ce.a.z + dir.z * g});
      }
      return;
    }
    // Curved edge: the shared discretization's 3D points are the canonical seam anchors.
    for (const math::Point3& p : d.points) anchors.add(p);
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
                      bool flip, bool hasBoundary, bool boundaryDriven,
                      const BoundaryAnchors& anchors) const {
    const std::vector<double> us =
        axisSamples(eval, box, region, /*uDir=*/true, hasBoundary, boundaryDriven);
    const std::vector<double> vs =
        axisSamples(eval, box, region, /*uDir=*/false, hasBoundary, boundaryDriven);
    const int cols = static_cast<int>(vs.size());

    Mesh m;
    m.vertices.reserve(us.size() * vs.size());
    m.normals.reserve(us.size() * vs.size());
    for (double u : us)
      for (double v : vs) {
        const SurfaceSample s = eval.d1(u, v);
        // Snap a seam-lying vertex to its canonical point (exact weld).
        const math::Point3* anchor = anchors.find(s.point);
        m.vertices.push_back(anchor ? *anchor : s.point);
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
  //
  // When the boundary edges bounding this axis are ALREADY subdivided (they place
  // more than two distinct coordinates on this axis — the twisted-loft case, where
  // the solid-mesher pre-pass forced the straight side edges to subdivide), the
  // boundary samples ARE the correct grid for this axis: every interior grid line
  // on this axis lies on a shared boundary edge, so it must match the neighbour's
  // discretization EXACTLY. Adding independent curvature lines would put rows on
  // the shared edge that the neighbour (a planar cap) lacks and open the seam.
  // We therefore use the boundary samples verbatim and skip the curvature grid.
  // A straight, unsubdivided axis (≤ 2 boundary coords — a cylinder/sphere's axis
  // direction) keeps the historical curvature grid unioned with the boundary.
  //
  // This boundary-driven shortcut is applied ONLY to free-form (Bezier/BSpline)
  // faces (`boundaryDriven`). Analytic faces (Cylinder/Cone/Sphere) keep the exact
  // historical union path — their curvature grid carries the circumferential
  // sampling and their straight seam edges are NOT subdivided by any pre-pass, so
  // switching them to boundary-only would drop the interior rows and open the seam.
  std::vector<double> axisSamples(const SurfaceEvaluator& eval, const UVBox& box,
                                  const UVRegion& region, bool uDir, bool hasBoundary,
                                  bool boundaryDriven) const {
    const double lo = uDir ? box.uMin : box.vMin;
    const double hi = uDir ? box.uMax : box.vMax;
    const double tol = std::max(hi - lo, 1.0) * 1e-7;

    if (hasBoundary && boundaryDriven) {
      std::vector<double> bnd = boundaryCoords(region, uDir);
      dedupSorted(bnd, tol);
      if (bnd.size() > 2) return bnd;  // boundary already subdivides this axis
    }

    const auto d2 =
        detail::worstCurvature(eval, box.uMin, box.uMax, box.vMin, box.vMax, boundaryDriven);
    const int n = detail::divisionsFor(hi - lo, uDir ? d2[0] : d2[1], p_);
    std::vector<double> axis;
    axis.reserve(static_cast<std::size_t>(n) + 1);
    for (int i = 0; i <= n; ++i)
      axis.push_back(lo + (hi - lo) * (static_cast<double>(i) / n));
    if (hasBoundary) detail::mergeAxis(axis, boundaryCoords(region, uDir), tol);
    return axis;
  }

  // In-place sort + dedup within `tol` (endpoints kept).
  static void dedupSorted(std::vector<double>& v, double tol) {
    std::sort(v.begin(), v.end());
    std::vector<double> out;
    for (double x : v)
      if (out.empty() || x - out.back() > tol) out.push_back(x);
    v.swap(out);
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

  // ── TRIMMED-FREE-FORM path (MOAT M0: curved patch, real EDGE_LOOP boundary) ───
  // Boundary from the shared per-edge discretization (identical, in 3D, to the
  // neighbour's — so the seam welds exactly like the ear-clip cap). The interior is
  // sampled by a CONSTRAINED-DELAUNAY triangulation (uv_triangulate ConstrainedDelaunay)
  // seeded with the curvature-driven grid and refined until every triangle's chord
  // deviation is within the deflection bound. Delaunay (not ear-clip + Steiner) is
  // essential: it flips away the boundary-spanning fan chords that would otherwise
  // hold an unbounded deflection and stop the volume from converging. Every emitted
  // vertex is S(u,v) on the true surface; boundary vertices snap to canonical anchors.
  Mesh trimmedFreeformMesh(const SurfaceEvaluator& eval, const std::vector<UVPolygon>& loops,
                           const UVRegion& region, bool flip,
                           const BoundaryAnchors& anchors) const {
    std::vector<UV> pts;
    const std::vector<std::vector<int>> loopIdx = appendBoundaryLoops(loops, pts);
    if (loopIdx.empty() || loopIdx[0].size() < 3) return {};

    detail::ConstrainedDelaunay cdt(pts, loopIdx);
    for (const UV& g : interiorGridPoints(eval, region)) cdt.insert(g);
    const auto maxPts = static_cast<std::size_t>(p_.maxDiv) * static_cast<std::size_t>(p_.maxDiv);
    cdt.refine(
        [&](int a, int b, int c) { return triangleDeflection(eval, pts, a, b, c) > p_.deflection; },
        /*maxPasses=*/20, maxPts);
    const std::vector<UVTri> tris = cdt.triangles();

    Mesh m = evaluatePoints(eval, pts, flip, anchors);
    m.triangles.reserve(tris.size());
    for (const UVTri& t : tris) {
      const auto ia = static_cast<std::uint32_t>(t.a);
      const auto ib = static_cast<std::uint32_t>(t.b);
      const auto ic = static_cast<std::uint32_t>(t.c);
      if (flip) m.addTriangle(ia, ic, ib); else m.addTriangle(ia, ib, ic);
    }
    return m;
  }

  // Curvature-driven interior UV sample grid, kept strictly inside the trimmed
  // region. Divisions come from the SAME sagitta law and worstCurvature(foldTwist)
  // used by the structured-grid path, so a foreign patch is sampled at the same
  // interior density its curvature demands. Boundary rows/cols (i,j at 0 or n) are
  // omitted — the boundary is already the shared-edge polygon.
  std::vector<UV> interiorGridPoints(const SurfaceEvaluator& eval, const UVRegion& region) const {
    std::vector<UV> out;
    const UVBox& b = region.box;
    const double du = b.uMax - b.uMin, dv = b.vMax - b.vMin;
    if (du <= 0.0 || dv <= 0.0) return out;
    const auto d2 = detail::worstCurvature(eval, b.uMin, b.uMax, b.vMin, b.vMax, /*foldTwist=*/true);
    const int nu = detail::divisionsFor(du, d2[0], p_);
    const int nv = detail::divisionsFor(dv, d2[1], p_);
    for (int i = 1; i < nu; ++i)
      for (int j = 1; j < nv; ++j) {
        const UV g{b.uMin + du * (static_cast<double>(i) / nu),
                   b.vMin + dv * (static_cast<double>(j) / nv)};
        if (region.inside(g)) out.push_back(g);
      }
    return out;
  }

  // Chord deviation of a UV triangle (vertex indices a,b,c into `pts`): the max
  // distance from the true surface to the triangle's 3D plane, probed at the centroid
  // and the three edge midpoints (where a smooth patch bulges farthest from the flat
  // triangle). A UV triangle that maps to a degenerate (zero-area) 3D triangle
  // contributes no deviation. Drives the CDT refinement's split predicate.
  static double triangleDeflection(const SurfaceEvaluator& eval, const std::vector<UV>& pts, int a,
                                   int b, int c) {
    const math::Point3 A = eval.value(pts[a].u, pts[a].v);
    const math::Point3 B = eval.value(pts[b].u, pts[b].v);
    const math::Point3 C = eval.value(pts[c].u, pts[c].v);
    math::Vec3 n = math::cross(B - A, C - A);
    const double L = math::norm(n);
    if (L < 1e-30) return 0.0;
    n = n / L;
    const UV probes[4] = {{(pts[a].u + pts[b].u + pts[c].u) / 3.0, (pts[a].v + pts[b].v + pts[c].v) / 3.0},
                          {(pts[a].u + pts[b].u) / 2.0, (pts[a].v + pts[b].v) / 2.0},
                          {(pts[b].u + pts[c].u) / 2.0, (pts[b].v + pts[c].v) / 2.0},
                          {(pts[c].u + pts[a].u) / 2.0, (pts[c].v + pts[a].v) / 2.0}};
    double dmax = 0.0;
    for (const UV& q : probes) {
      const math::Point3 P = eval.value(q.u, q.v);
      dmax = std::max(dmax, std::fabs(math::dot(P - A, n)));
    }
    return dmax;
  }

  // ── EAR-CLIP path (genuinely trimmed faces) ──────────────────────────────────
  Mesh earClipMesh(const SurfaceEvaluator& eval, const std::vector<UVPolygon>& loops, bool flip,
                   const BoundaryAnchors& anchors) const {
    std::vector<UV> pts;
    const std::vector<std::vector<int>> loopIdx = appendBoundaryLoops(loops, pts);
    if (loopIdx.empty() || loopIdx[0].size() < 3) return {};
    const std::vector<UVTri> tris = triangulatePolygon(pts, loopIdx);

    Mesh m = evaluatePoints(eval, pts, flip, anchors);
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
  // for a Reversed face so the solid's outward normal is consistent). A point that
  // lands on a straight boundary seam is snapped to its canonical anchor so the
  // seam welds exactly across the two adjacent faces.
  static Mesh evaluatePoints(const SurfaceEvaluator& eval, const std::vector<UV>& pts, bool flip,
                             const BoundaryAnchors& anchors) {
    Mesh m;
    m.vertices.reserve(pts.size());
    m.normals.reserve(pts.size());
    for (const UV& q : pts) {
      const SurfaceSample s = eval.d1(q.u, q.v);
      const math::Point3* anchor = anchors.find(s.point);
      m.vertices.push_back(anchor ? *anchor : s.point);
      m.normals.push_back(flip ? s.normal.reversed() : s.normal);
    }
    return m;
  }

  MeshParams p_;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_FACE_MESHER_H

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
#include "native/tessellate/seam_strip.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/tessellate/uv_triangulate.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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

// ── Closed-seam chord pins (topology-aware exact boundary placement) ──────────
// A shared CLOSED seam between two sub-faces (a curved annulus/disk ∪ its flat cap,
// produced by smooth_trim_split / curved_wall_cut) is carried as a loop of 2-pole
// degree-1 STRAIGHT chords (detail::isSeamChord). The seam's canonical 3-D geometry
// is the straight chord C_edge — the SAME for both sub-faces. But a CURVED sub-face
// evaluates its seam boundary through S_face(pcurve), which BULGES off the chord (the
// pcurve rides the bowl surface), so the two sub-faces' seam samples diverge and the
// closed boundary does not weld watertight once a chord is subdivided (n>1 samples).
//
// SeamPins fixes this the topology-aware way: for a seam-chord boundary edge on a
// curved sub-face, the face mesher records, keyed by the sample's EXACT (u,v), the
// edge's CANONICAL 3-D point (C_edge at that fraction). evaluatePoints then places the
// boundary vertex at that canonical point instead of S_face(u,v) — so BOTH sub-faces
// (the bulging annulus AND the flat cap) put the seam vertex at the identical shared
// chord point and the closed seam welds watertight at ANY deflection.
//
// The key is the boundary sample's (u,v), quantized to a grid far finer than any real
// UV feature — the flattened boundary polygon uses the SAME (u,v) as the recorded pin
// (both come from the shared pcurve at the shared fraction), so the match is exact.
// This is a per-face pin (unlike BoundaryAnchors, which is a 3-D spatial index): each
// sub-face pins ITS OWN boundary to the shared chord, and because the chord is the same
// canonical curve, the two pins agree bit-for-bit. Fires ONLY for isSeamChord edges on
// Bezier/BSpline faces — the planar cap's chord already equals S_plane(pcurve) so it is
// a no-op there, and no other edge/surface populates it, keeping existing meshes
// byte-identical.
struct SeamPins {
  static constexpr double kQuantum = 1e-9;  ///< UV index cell (≫ ULP, ≪ any real UV feature)
  struct Key {
    long long u, v;
    bool operator==(const Key& o) const noexcept { return u == o.u && v == o.v; }
  };
  struct Hash {
    std::size_t operator()(const Key& k) const noexcept {
      std::size_t h = static_cast<std::size_t>(k.u) * 73856093u;
      h ^= static_cast<std::size_t>(k.v) * 19349663u;
      return h;
    }
  };
  std::unordered_map<Key, math::Point3, Hash> pts;

  static long long q(double x) noexcept {
    const double s = 1.0 / kQuantum;
    return static_cast<long long>(x >= 0 ? x * s + 0.5 : x * s - 0.5);
  }
  Key keyOf(const UV& p) const noexcept { return Key{q(p.u), q(p.v)}; }
  void add(const UV& uv, const math::Point3& p) { pts.emplace(keyOf(uv), p); }
  const math::Point3* find(const UV& uv) const {
    if (pts.empty()) return nullptr;
    const auto it = pts.find(keyOf(uv));
    return it != pts.end() ? &it->second : nullptr;
  }
  bool empty() const noexcept { return pts.empty(); }
};

}  // namespace detail

using detail::BoundaryAnchors;
using detail::SeamPins;

// ─────────────────────────────────────────────────────────────────────────────
// FaceMesher — mesh one face. Stateless apart from the parameters. mesh() with an
// EdgeCache uses the shared per-edge discretization (STAGE 2, watertight);
// mesh(face) without one builds a throwaway cache (single-face host callers).
// ─────────────────────────────────────────────────────────────────────────────
class FaceMesher {
 public:
  explicit FaceMesher(MeshParams params = {}) noexcept : p_(params) {}

  /// Register the shared seam-strip registry (MESH-STRIP-IMPL). When non-null and a face
  /// carries a registered shared seam, the near-seam strip is meshed from the SHARED collar
  /// (so two faces sharing the seam emit identical near-seam triangles). Null (the default,
  /// and every single-face / baseline path) ⇒ byte-identical to the pre-strip mesher.
  void setSeamStrips(const SeamStripRegistry* r) noexcept { seamStrips_ = r; }

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
    SeamPins seamPins;
    const std::vector<UVPolygon> loops = buildBoundaryLoops(face, cache, eval, anchors, seamPins);
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
      return structuredGrid(eval, region, box, flip, /*hasBoundary=*/false, freeForm, anchors,
                            seamPins);

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
      return structuredGrid(eval, region, box, flip, /*hasBoundary=*/true, freeForm, anchors,
                            seamPins);

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
      return trimmedFreeformMesh(eval, loops, region, flip, anchors, seamPins);

    return earClipMesh(eval, loops, flip, anchors, seamPins);
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

  // ── Freeform-backed curved-rim marking (curved-rim weld pin gate) ────────────
  // If THIS face is FREE-FORM (Bézier / B-spline) and its surface evaluation of a
  // genuinely-curved boundary edge REPRODUCES the edge's canonical discretization
  // d.points (the weld contract S_freeform(pcurve) = C_edge holds to ≤ kSnapEps at every
  // shared sample), mark that edge FREEFORM-BACKED in the cache. This is the precise, GLOBAL
  // (cross-face) guarantee that makes pinning a DIVERGING flat neighbour's rim to d.points
  // safe: a free-form face genuinely lies on d.points, so the pin fuses the flat lid onto the
  // ONE shared curve (the bowl↔lid rim). A synthesized boolean cut-cap curved seam whose
  // free-form neighbour does NOT reproduce d.points is NEVER marked, so the Plane pin cannot
  // fire on it (the first-freeform / split-plane / chain-seam family stays byte-identical).
  // Must run (for every face) before any face is meshed. Analytic faces mark nothing.
  void markFreeformBackedRims(const topo::Shape& face, EdgeCache& cache) const {
    const auto sr = topo::surfaceOf(face);
    if (!sr || !sr->surface) return;
    const topo::FaceSurface::Kind k = sr->surface->kind;
    if (k != topo::FaceSurface::Kind::Bezier && k != topo::FaceSurface::Kind::BSpline) return;
    SurfaceEvaluator eval(*sr->surface, sr->location);
    for (const topo::Shape& wire : face.tshape()->children())
      for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
        const topo::Shape& edge = ex.current();
        const auto cr = topo::curveOf(edge);
        if (!cr || !cr->curve) continue;
        if (!detail::isCurvedSharedRim(*cr->curve, cr->first, cr->last)) continue;
        const topo::PCurve* pc = pcurveForFace(edge, face);
        if (!pc) continue;
        const EdgeDiscretization& d = cache.discretize(edge);
        if (d.fracs.size() != d.points.size() || d.points.size() < 2) continue;
        const auto rr = topo::rangeOf(edge);
        const double first = rr ? rr->first : 0.0;
        const double last = rr ? rr->last : 1.0;
        bool reproduces = true;
        for (std::size_t i = 0; i < d.fracs.size() && reproduces; ++i) {
          const double f = d.fracs[i];
          const UV uv = pcurveValue(*pc, first + (last - first) * f, f);
          if (math::distance(eval.value(uv.u, uv.v), d.points[i]) > BoundaryAnchors::kSnapEps)
            reproduces = false;
        }
        if (reproduces) cache.markFreeformBackedRim(edge);
      }
  }

 private:
  // ── STAGE-2 boundary: each wire flattened at the shared edge fractions ───────
  std::vector<UVPolygon> buildBoundaryLoops(const topo::Shape& face, EdgeCache& cache,
                                            const SurfaceEvaluator& eval, BoundaryAnchors& anchors,
                                            SeamPins& seamPins) const {
    std::vector<UVPolygon> loops;
    if (face.isNull() || face.type() != topo::ShapeType::Face) return loops;
    for (const topo::Shape& wire : face.tshape()->children())
      loops.push_back(flattenWireShared(wire, face, cache, eval, anchors, seamPins));
    return loops;
  }

  // Flatten one wire to a UV polygon using the shared per-edge fraction list, and
  // record a canonical 3D anchor per straight-edge sample so the seam welds
  // exactly (see BoundaryAnchors). Also record a per-sample SeamPin whenever this
  // face's surface evaluation of a SHARED CURVED/SEAM edge boundary DIVERGES from the
  // edge's canonical 3-D discretization (see recordSharedEdgePins / SeamPins) — the
  // topology-aware pin that welds a closed seam (annulus↔cap) AND a fragile curved rim
  // (bowl↔lid) watertight at any deflection.
  UVPolygon flattenWireShared(const topo::Shape& wire, const topo::Shape& face, EdgeCache& cache,
                              const SurfaceEvaluator& eval, BoundaryAnchors& anchors,
                              SeamPins& seamPins) const {
    UVPolygon poly;
    if (wire.isNull() || wire.type() != topo::ShapeType::Wire) return poly;
    const auto fsr = topo::surfaceOf(face);
    const topo::FaceSurface::Kind faceKind =
        fsr && fsr->surface ? fsr->surface->kind : topo::FaceSurface::Kind::Plane;
    for (topo::Explorer ex(wire, topo::ShapeType::Edge); ex.more(); ex.next()) {
      const topo::Shape& edge = ex.current();
      const topo::PCurve* pc = pcurveForFace(edge, face);
      if (!pc) continue;
      const EdgeDiscretization& d = cache.discretize(edge);
      appendEdgeSamplesAtFracs(poly, edge, *pc, d.fracs);
      recordEdgeAnchors(anchors, edge, d);
      const bool freeformBackedRim = cache.isFreeformBackedRim(edge);
      recordSeamChordPins(seamPins, edge, *pc, d, eval, faceKind, freeformBackedRim);
    }
    return poly;
  }

  // Record UV → canonical-3-D pins for a SHARED CURVED or SEAM-CHORD boundary edge whose
  // per-face surface evaluation DIVERGES from the edge's canonical discretization d.points.
  //
  // The watertight-weld contract is S_face(pcurve(f)) = C_edge(f) = d.points[f], so BOTH
  // faces sharing an edge place the SAME 3-D boundary point at each shared fraction. That
  // contract HOLDS to fp round-off for every analytic primitive (cylinder cap↔side,
  // sphere, cone) — so this pin equals the surface point and the mesh is BYTE-IDENTICAL.
  // It is VIOLATED in two topology-specific cases the curved-wall boolean produces:
  //   * a closed-seam 2-pole degree-1 STRAIGHT CHORD carried on a CURVED bowl sub-face —
  //     the pcurve rides the bowl, so S_bowl(pcurve) BULGES off the shared flat chord;
  //   * a curved rim edge whose neighbour (the flat lid) carries a pcurve that does not
  //     reproduce C_edge at the subdivided fractions — S_lid(pcurve) ≠ d.points.
  // In both, the diverging face's subdivided seam/rim samples land far (≫ kSnapEps) from
  // the shared canonical points, so the two faces' boundaries do not coincide and the
  // seam/rim opens once subdivided. Pinning the DIVERGING samples to d.points (the ONE
  // canonical discretization the shared EdgeCache hands both faces) makes the two
  // boundaries bit-identical and the weld watertight at ANY deflection.
  //
  // GUARDED by TOPOLOGY so existing meshes are byte-identical: a pin is recorded ONLY for
  // a CLOSED-SEAM STRAIGHT CHORD (detail::isSeamChord — a 2-pole degree-1 curve, the exact
  // shape smooth_trim_split / curved_wall_cut lay as ONE segment of a closed interior
  // seam) carried on a CURVED (Bezier/BSpline) sub-face whose surface evaluation genuinely
  // DIVERGES from the shared chord. No analytic primitive, no genuinely-curved shared edge
  // (a cylinder cap↔side circle, a blend rim), and no straight Line edge is a seam chord,
  // so none is ever pinned — every existing mesh is byte-identical. Only the annulus/disk
  // sub-face of a curved-wall split, whose seam chord's pcurve rides the bowl surface and
  // bulges off the shared flat chord, is pinned to d.points (== C_edge, the ONE canonical
  // discretization both sub-faces share), so the closed seam welds watertight at ANY
  // deflection. The divergence test is what keeps the FLAT cap side (S_plane(pcurve) ==
  // chord) a no-op even though its seam edges are also chords — only the bulging curved
  // side records a pin; the flat side already places the boundary on the chord.
  static void recordSeamChordPins(SeamPins& seamPins, const topo::Shape& edge,
                                  const topo::PCurve& pc, const EdgeDiscretization& d,
                                  const SurfaceEvaluator& eval, topo::FaceSurface::Kind faceKind,
                                  bool freeformBackedRim) {
    if (d.fracs.size() != d.points.size() || d.points.size() < 2) return;
    const auto cr = topo::curveOf(edge);
    if (!cr || !cr->curve) return;
    // TOPOLOGY GUARD — two mutually-exclusive shared-edge shapes are pinned, both of which
    // the curved-wall boolean produces and which VIOLATE the S_face(pcurve) = C_edge weld
    // contract for one incident face. Everything else returns early and is NEVER pinned.
    //
    //   (A) CLOSED-SEAM STRAIGHT CHORD (detail::isSeamChord — a 2-pole degree-1 curve): the
    //       exact shape smooth_trim_split / curved_wall_cut lay as ONE segment of a closed
    //       INTERIOR seam. Its 3-D geometry is straight, but on a CURVED sub-face the pcurve
    //       rides the bowl surface and bulges off the shared flat chord (MOAT M0w). Pinned on
    //       ANY face whose surface evaluation diverges (the curved sub-face).
    //   (B) SHARED CURVED RIM (detail::isCurvedSharedRim — a genuinely-curved degree≥2
    //       free-form arc, NOT a straight seam chord) — pinned ONLY on a PLANE face AND ONLY
    //       when the edge is FREEFORM-BACKED (`freeformBackedRim`, the SolidMesher pre-pass
    //       proved a FREE-FORM face reproduces this edge's d.points to ≤ kSnapEps). This is
    //       the exact shape the curved-wall operand lays as the OUTER rim shared by a FREEFORM
    //       bowl wall and an ADJACENT FLAT analytic LID: both subdivide the rim to the SAME
    //       shared fraction list, but the flat lid's PLANAR pcurve stays IN the plane and does
    //       NOT reproduce the 3-D rim arc (which dips off the plane), so S_lid(pcurve) ≠ C_edge
    //       and the subdivided rim opens (MOAT M0-rim). Pinning the diverging lid samples to
    //       d.points fuses them onto the ONE curve the free-form bowl already lies on. The
    //       FREEFORM-BACKED gate is the correctness GUARANTEE (not a heuristic): the pin fires
    //       only where a free-form face is KNOWN to lie on d.points, so pinning the flat
    //       neighbour to it is always right. A synthesized boolean cut-cap curved seam whose
    //       free-form neighbour does NOT reproduce d.points is never marked, so it is never
    //       pinned (the first-freeform / split-plane / chain-seam / two-operand / multi-seam
    //       family stays byte-identical). The curved-rim pin also SKIPS the two ENDPOINT
    //       samples (f = 0, f = 1): those are the rim's shared CORNER vertices where it meets
    //       straight side edges and must stay bit-identical to those edges' endpoints.
    //
    // In BOTH cases the DIVERGING face's pinned samples land far (≫ kSnapEps) from the shared
    // canonical C_edge points d.points, so the two faces' boundaries diverge and the seam/rim
    // opens once subdivided. Pinning the diverging samples to d.points (the ONE canonical
    // EdgeCache discretization BOTH faces read) makes the two boundaries bit-identical and the
    // weld watertight at ANY deflection.
    //
    // WHY EVERY EXISTING MESH STAYS BYTE-IDENTICAL. The recorded pin is DIVERGENCE-GATED
    // (‖S_face(pcurve) − C_edge‖ > kSnapEps): it fires ONLY where the weld contract is
    // genuinely violated. For the curved rim it is ALSO KIND-gated (free-form arc only, never an
    // analytic Circle/Ellipse), FACE-gated (Plane face only), FREEFORM-BACKED-gated (a free-form
    // face provably lies on d.points), and skips the corner endpoints. Every analytic
    // primitive's shared curved edge (a cylinder cap↔side circle, a sphere/cone seam, a torus
    // rim) is a Circle/Ellipse — excluded by kind; a curved edge shared through ONE node
    // reproduces C_edge on both faces — never diverges; a free-form boolean seam that no
    // free-form face backs is never marked — never pinned. The ONLY shapes that match a
    // predicate AND satisfy every gate AND diverge are the curved-wall boolean's closed seam and
    // its bowl↔flat-lid rim — precisely the meshes intended to change (non-watertight →
    // watertight); the FNV hash battery confirms every other mesh is byte-identical.
    const bool seamChord = detail::isSeamChord(*cr->curve, cr->first, cr->last);
    const bool curvedRim = !seamChord && freeformBackedRim &&
                           faceKind == topo::FaceSurface::Kind::Plane &&
                           detail::isCurvedSharedRim(*cr->curve, cr->first, cr->last);
    if (!seamChord && !curvedRim) return;
    const auto rr = topo::rangeOf(edge);
    const double first = rr ? rr->first : 0.0;
    const double last = rr ? rr->last : 1.0;
    // Pin ONLY the samples that genuinely diverge from the shared canonical curve. For the
    // curved rim, skip the two ENDPOINT samples (shared corner vertices) so a rim that meets
    // straight side edges keeps those junctions bit-identical; the closed seam chord has no
    // such corners (its endpoints coincide with the next chord's) so it pins the full range.
    for (std::size_t i = 0; i < d.fracs.size(); ++i) {
      const double f = d.fracs[i];
      if (curvedRim && (i == 0 || i + 1 == d.fracs.size())) continue;  // keep rim corner vertices
      const UV uv = pcurveValue(pc, first + (last - first) * f, f);
      if (math::distance(eval.value(uv.u, uv.v), d.points[i]) > BoundaryAnchors::kSnapEps)
        seamPins.add(uv, d.points[i]);
    }
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
        // The two ENDPOINT anchors are the EXACT canonical endpoints (the shared
        // bounding vertices), NOT the interpolated ce.a + dir·(i/n). At i=n that
        // interpolation is ce.a + (ce.b − ce.a), which rounds ~1 ULP off ce.b (e.g.
        // −0.5 + 0.585 ≠ 0.085 exactly). A shared CORNER where a curved seam edge
        // terminates against straight edges is anchored by BOTH the curved edge (its
        // d.points endpoint = the vertex) and these straight edges; if a straight
        // edge contributes the rounded endpoint it competes for the same weld/hash
        // slot (first-add-wins) with the exact vertex, so which value the corner
        // takes depends on wire/edge order — the two incident faces can then place
        // copies ~1 ULP apart, and when the corner lands on a weld-cell boundary the
        // copies split and the seam opens. Pinning i=0→ce.a and i=n→ce.b makes every
        // incident straight edge place the BIT-IDENTICAL vertex, matching the curved
        // seam's endpoint, so the corner welds at any deflection. Interior samples
        // keep the canonical ce.a + dir·(i/n) so two opposite-order straight edges
        // still produce bit-identical anchor SETS.
        if (i == 0) { anchors.add(ce.a); continue; }
        if (i == n) { anchors.add(ce.b); continue; }
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
                      const BoundaryAnchors& anchors, const SeamPins& seamPins) const {
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
        // A closed-seam boundary sample pins to the shared chord; else snap a seam-lying
        // vertex to its canonical spatial anchor (exact weld). (A full-parametric face
        // never carries seam pins — its boundary is the parametric rectangle — so this
        // lookup is a no-op there; kept for uniform boundary placement.)
        if (const math::Point3* pin = seamPins.find(UV{u, v})) {
          m.vertices.push_back(*pin);
          m.normals.push_back(flip ? s.normal.reversed() : s.normal);
          continue;
        }
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
                           const UVRegion& region, bool flip, const BoundaryAnchors& anchors,
                           const SeamPins& seamPins) const {
    // ── SHARED SEAM-STRIP path (MESH-STRIP-IMPL) ──────────────────────────────
    // If this face carries a registered shared seam loop, REPLACE that seam loop (in the
    // CDT input) with its concentric COLLAR loop (one ring inward on the material side, at
    // the SHARED 3-D collar points), then splice the fixed seam↔collar strip triangles. The
    // collar is computed ONLY from the shared seam geometry, so both faces sharing the seam
    // emit the IDENTICAL near-seam strip and the weld pairs 1:1. The CDT fills only the
    // collar-outward remainder (interior Steiner points are suppressed inside the band).
    std::vector<SeamCollar> collars;
    std::vector<UVPolygon> cdtLoops = loops;
    SeamPins collarPins;  // collar UV → shared collar 3-D (so the CDT collar ring matches the strip)
    if (seamStrips_ && !seamStrips_->empty())
      collars = substituteCollarLoops(eval, region, cdtLoops, seamPins, collarPins);

    std::vector<UV> pts;
    const std::vector<std::vector<int>> loopIdx = appendBoundaryLoops(cdtLoops, pts);
    if (loopIdx.empty() || loopIdx[0].size() < 3) return {};

    detail::ConstrainedDelaunay cdt(pts, loopIdx);
    for (const UV& g : interiorGridPoints(eval, region))
      if (collars.empty() || !inAnyCollarBand(eval, g)) cdt.insert(g);
    const auto maxPts = static_cast<std::size_t>(p_.maxDiv) * static_cast<std::size_t>(p_.maxDiv);
    cdt.refine(
        [&](int a, int b, int c) {
          if (!collars.empty() && triangleInCollarBand(eval, pts, a, b, c)) return false;
          return triangleDeflection(eval, pts, a, b, c) > p_.deflection;
        },
        /*maxPasses=*/20, maxPts);
    const std::vector<UVTri> tris = cdt.triangles();

    Mesh m = collarPins.empty() ? evaluatePoints(eval, pts, flip, anchors, seamPins)
                                : evaluatePointsWithCollar(eval, pts, flip, anchors, seamPins,
                                                           collarPins);
    m.triangles.reserve(tris.size() + collars.size() * 8);
    for (const UVTri& t : tris) {
      const auto ia = static_cast<std::uint32_t>(t.a);
      const auto ib = static_cast<std::uint32_t>(t.b);
      const auto ic = static_cast<std::uint32_t>(t.c);
      if (flip) m.addTriangle(ia, ic, ib); else m.addTriangle(ia, ib, ic);
    }
    // Splice the fixed seam↔collar strip (its 3-D vertices are the SHARED seam + collar
    // points, so the two faces' strips are bit-identical and weld 2-manifold).
    for (const SeamCollar& sc : collars) appendCollarStrip(m, sc, flip);
    return m;
  }

  // One registered seam loop's per-vertex strip data on THIS face: the shared seam 3-D
  // point, the shared collar 3-D point, and the collar's UV (an inward march from the seam
  // toward the material). The collar UV replaced the seam loop in the CDT input.
  struct SeamCollar {
    std::vector<math::Point3> seam3d;    ///< shared seam vertices (bit-identical across faces)
    std::vector<math::Point3> collar3d;  ///< shared collar vertices (bit-identical across faces)
    std::vector<UV> collarUV;            ///< collar UV on this face (for CDT + band test)
    const detail::SeamStrip* strip = nullptr;
  };

  // For every loop that is a REGISTERED shared seam, replace it in `cdtLoops` with its
  // collar loop (UV) and return the collar data for splicing. A loop is a registered seam
  // iff every vertex resolves to a registered seam vertex (via the seam pin / surface) AND
  // a shared collar exists. Non-seam loops are left untouched (byte-identical).
  std::vector<SeamCollar> substituteCollarLoops(const SurfaceEvaluator& eval,
                                                const UVRegion& region,
                                                std::vector<UVPolygon>& cdtLoops,
                                                const SeamPins& seamPins,
                                                SeamPins& collarPins) const {
    std::vector<SeamCollar> out;
#ifdef CYBERCAD_SEAMSTRIP_DEBUG
    std::fprintf(stderr, "[collar] seamPins.empty=%d\n", (int)seamPins.empty());
#endif
    for (std::size_t li = 0; li < cdtLoops.size(); ++li) {
      UVPolygon& loop = cdtLoops[li];
      std::size_t count = loop.size();
      if (count >= 2 && nearlyEqual(loop.front(), loop.back())) --count;
      if (count < 3) continue;
      SeamCollar sc;
      sc.seam3d.reserve(count);
      sc.collar3d.reserve(count);
      sc.collarUV.reserve(count);
      // PASS 1 — resolve every seam vertex to its shared 3-D seam point and march inward
      // (toward the material) to this FACE'S OWN collar UV at the band edge: the surface
      // point whose radius off the seam axis is δ (collarInset) away from the seam radius.
      // The seam ring is shared (bit-identical across the two faces); the collar is per-face
      // on the face's OWN surface. This is the WEDGE-CARRYING collar (MESH-COLLAR-WEDGE):
      // the two faces share ONLY the seam ring, so their near-seam bands DIVERGE onto their
      // respective surfaces and the collar wedge volume is carried through the weld instead
      // of annihilating (the old shared PLANAR collar made both bands bit-identical, so the
      // weld's coincident-duplicate-triangle drop annihilated both and lost the wedge).
      std::vector<math::Point3> seamPts(count);
      std::vector<UV> collarUVs(count);
      bool isSeam = true;
      for (std::size_t i = 0; i < count && isSeam; ++i) {
        const UV& uv = loop[i];
        const math::Point3* pin = seamPins.find(uv);
        const math::Point3 seamP = pin ? *pin : eval.value(uv.u, uv.v);
        if (seamStrips_->stripIndexFor(seamP) < 0) { isSeam = false; break; }
        seamPts[i] = seamP;
        collarUVs[i] = marchToBandEdge(loop, i, count, region, eval, seamP,
                                       seamStrips_->seamRadiusFor(seamP),
                                       seamStrips_->collarInsetFor(seamP));
      }
      if (!isSeam) continue;
      // PASS 2 — take this face's own-surface collar for every KEPT vertex. The registry's
      // weld-resolution decimation (MESH-WELD-TOL, seam_strip.h) flags the subset of seam
      // vertices spaced ≥ 2·weldTol, so no two SEAM-ring vertices merge at the spatial weld
      // (an over-dense seam ring otherwise collapses runs of ring vertices into 4×-used
      // edges at fine deflection). Both faces resolve a seam sample to the SAME registry
      // entry, so they keep the identical SEAM subset → the shared seam ring pairs 1:1. An
      // undecimated ring keeps every vertex. The collar sits δ off the seam (≫ weldTol), so
      // no collar vertex merges into its seam vertex.
      for (std::size_t i = 0; i < count; ++i) {
        if (!seamStrips_->keptSeamVertex(seamPts[i])) continue;
        sc.seam3d.push_back(seamPts[i]);
        sc.collarUV.push_back(collarUVs[i]);
        sc.collar3d.push_back(eval.value(collarUVs[i].u, collarUVs[i].v));
      }
#ifdef CYBERCAD_SEAMSTRIP_DEBUG
      std::fprintf(stderr, "[collar] loop %zu count=%zu isSeam=%d collar3d=%zu\n", li,
                   count, (int)isSeam, sc.collar3d.size());
#endif
      if (!isSeam || sc.collar3d.size() < 3) continue;
      // Replace the seam loop with this face's collar loop in the CDT input (the CDT then
      // fills only collar-outward; the seam↔collar band is the spliced strip). Pin each
      // collar UV to its own-surface 3-D collar point so the CDT collar-boundary vertex
      // coincides EXACTLY with the spliced strip's collar vertex (this face → its own strip
      // welds internally; the two faces weld to each other only at the shared seam ring).
      for (std::size_t i = 0; i < sc.collarUV.size(); ++i)
        collarPins.add(sc.collarUV[i], sc.collar3d[i]);
      loop.assign(sc.collarUV.begin(), sc.collarUV.end());
      out.push_back(std::move(sc));
    }
    return out;
  }

  // March UV vertex `i` inward (toward the region interior) far enough that the surface
  // point sits at the seam-strip BAND EDGE — its radius off the seam axis is `inset` (δ)
  // away from the seam radius `seamR`, ON THIS FACE'S OWN SURFACE. Grows the inward step
  // geometrically until the surface radius crosses the band edge, then bisects; falls back
  // to the largest in-region step if the region is thinner than δ (a graceful narrow-band
  // collar, still per-face on the surface, never off it). Direction is the loop-edge inward
  // normal (rotate the tangent 90°), sign fixed by which side lands inside the region.
  UV marchToBandEdge(const UVPolygon& loop, std::size_t i, std::size_t count,
                     const UVRegion& region, const SurfaceEvaluator& eval,
                     const math::Point3& seamP, double seamR, double inset) const {
    const UV& prev = loop[(i + count - 1) % count];
    const UV& next = loop[(i + 1) % count];
    const double tx = next.u - prev.u, ty = next.v - prev.v;  // tangent
    double nx = -ty, ny = tx;                                 // rotate 90° → inward normal
    const double nl = std::sqrt(nx * nx + ny * ny);
    if (nl < 1e-30 || inset <= 0.0) return loop[i];
    nx /= nl; ny /= nl;
    const double edge = std::sqrt(tx * tx + ty * ty);
    // Fix the inward sign: the direction whose small step lands inside the region. `h0` is
    // that known-inside step, the walk's guaranteed non-degenerate anchor (so a region
    // narrower than the first grown step never collapses the collar back onto the seam).
    const double h0 = 0.25 * edge;
    {
      const UV probe{loop[i].u + nx * h0, loop[i].v + ny * h0};
      if (!region.inside(probe)) { nx = -nx; ny = -ny; }
    }
    const UV base = loop[i];
    auto radiusAt = [&](double h) {
      const UV c{base.u + nx * h, base.v + ny * h};
      return seamStrips_->radiusInStrip(eval.value(c.u, c.v), seamP);
    };
    // Grow the step until the surface radius is ≥ inset off the seam radius (the band edge),
    // seeding from the known-inside h0 so bestIn is always a genuine inward step.
    double lo = 0.0, hi = -1.0;
    double h = h0, bestIn = h0;
    for (int it = 0; it < 32 && h > 0.0; ++it) {
      const UV c{base.u + nx * h, base.v + ny * h};
      if (!region.inside(c)) break;  // ran out of region — stop growing
      bestIn = h;
      if (std::fabs(radiusAt(h) - seamR) >= inset) { hi = h; break; }
      lo = h;
      h *= 1.7;
    }
    if (hi < lo) return UV{base.u + nx * bestIn, base.v + ny * bestIn};  // never reached δ
    // Bisect [lo, hi] for the step whose surface radius is exactly the band edge.
    for (int it = 0; it < 32; ++it) {
      const double mid = 0.5 * (lo + hi);
      if (std::fabs(radiusAt(mid) - seamR) >= inset) hi = mid; else lo = mid;
    }
    return UV{base.u + nx * hi, base.v + ny * hi};
  }

  // Is UV grid point `g` inside ANY collar band (in the shared 3-D seam frame)? Suppressed
  // from CDT interior insertion so neither face triangulates the shared strip independently.
  bool inAnyCollarBand(const SurfaceEvaluator& eval, const UV& g) const {
    return seamStrips_->inCollarBand(eval.value(g.u, g.v));
  }

  // Does a CDT triangle fall inside a collar band (all 3 vertices in the band)? Then its
  // refinement is suppressed (the band is the fixed shared strip, not per-face refined).
  bool triangleInCollarBand(const SurfaceEvaluator& eval, const std::vector<UV>& pts, int a, int b,
                            int c) const {
    return seamStrips_->inCollarBand(eval.value(pts[a].u, pts[a].v)) &&
           seamStrips_->inCollarBand(eval.value(pts[b].u, pts[b].v)) &&
           seamStrips_->inCollarBand(eval.value(pts[c].u, pts[c].v));
  }

  // Splice the fixed seam↔collar strip: two triangles per seam segment, over the SHARED
  // seam + collar 3-D vertices. Both faces emit these SAME triangles (mirror-wound by
  // `flip`), so the shared seam edge is used exactly twice → 2-manifold weld.
  static void appendCollarStrip(Mesh& m, const SeamCollar& sc, bool flip) {
    const std::size_t n = sc.seam3d.size();
    if (n < 3 || sc.collar3d.size() != n) return;
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    const bool normals = m.hasNormals();
    // Append seam ring (0..n-1) then collar ring (n..2n-1). A flat placeholder normal is
    // fine — these near-seam strip triangles are thin; the weld carries the neighbour normal.
    const math::Dir3 nrm{0.0, 0.0, flip ? -1.0 : 1.0};
    for (std::size_t i = 0; i < n; ++i) {
      m.vertices.push_back(sc.seam3d[i]);
      if (normals) m.normals.push_back(nrm);
    }
    for (std::size_t i = 0; i < n; ++i) {
      m.vertices.push_back(sc.collar3d[i]);
      if (normals) m.normals.push_back(nrm);
    }
    for (std::size_t i = 0; i < n; ++i) {
      const auto s0 = base + static_cast<std::uint32_t>(i);
      const auto s1 = base + static_cast<std::uint32_t>((i + 1) % n);
      const auto c0 = base + static_cast<std::uint32_t>(n + i);
      const auto c1 = base + static_cast<std::uint32_t>(n + (i + 1) % n);
      // Quad (s0, s1, c1, c0) → two triangles. Winding: seam-ring outward-facing; `flip`
      // reverses it so both faces' strips wind oppositely and the shared seam edge pairs.
      if (flip) {
        m.addTriangle(s0, c1, s1);
        m.addTriangle(s0, c0, c1);
      } else {
        m.addTriangle(s0, s1, c1);
        m.addTriangle(s0, c1, c0);
      }
    }
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
                   const BoundaryAnchors& anchors, const SeamPins& seamPins) const {
    std::vector<UV> pts;
    const std::vector<std::vector<int>> loopIdx = appendBoundaryLoops(loops, pts);
    if (loopIdx.empty() || loopIdx[0].size() < 3) return {};
    const std::vector<UVTri> tris = triangulatePolygon(pts, loopIdx);

    Mesh m = evaluatePoints(eval, pts, flip, anchors, seamPins);
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
  // for a Reversed face so the solid's outward normal is consistent). A boundary UV
  // that carries a closed-seam pin is placed EXACTLY on the shared chord point (so the
  // curved sub-face's seam matches the flat cap's, see SeamPins); otherwise a point
  // that lands on a straight boundary seam is snapped to its canonical spatial anchor.
  static Mesh evaluatePoints(const SurfaceEvaluator& eval, const std::vector<UV>& pts, bool flip,
                             const BoundaryAnchors& anchors, const SeamPins& seamPins) {
    Mesh m;
    m.vertices.reserve(pts.size());
    m.normals.reserve(pts.size());
    for (const UV& q : pts) {
      const SurfaceSample s = eval.d1(q.u, q.v);
      // A closed-seam boundary sample is pinned to the shared chord by its (u,v)
      // (bit-identical between the two sub-faces); the surface normal is still the
      // sub-face's own (only the position welds).
      if (const math::Point3* pin = seamPins.find(q)) {
        m.vertices.push_back(*pin);
        m.normals.push_back(flip ? s.normal.reversed() : s.normal);
        continue;
      }
      const math::Point3* anchor = anchors.find(s.point);
      m.vertices.push_back(anchor ? *anchor : s.point);
      m.normals.push_back(flip ? s.normal.reversed() : s.normal);
    }
    return m;
  }

  // As evaluatePoints, but a UV carrying a COLLAR pin is placed at the shared collar 3-D
  // point (so the CDT's collar-boundary vertex coincides with the spliced strip's collar
  // vertex). Collar pins take precedence over seam/anchor snapping (a collar UV is interior,
  // never a seam-boundary sample). Used ONLY on the seam-strip path (collarPins non-empty),
  // so every existing mesh keeps the plain evaluatePoints and stays byte-identical.
  static Mesh evaluatePointsWithCollar(const SurfaceEvaluator& eval, const std::vector<UV>& pts,
                                       bool flip, const BoundaryAnchors& anchors,
                                       const SeamPins& seamPins, const SeamPins& collarPins) {
    Mesh m;
    m.vertices.reserve(pts.size());
    m.normals.reserve(pts.size());
    for (const UV& q : pts) {
      const SurfaceSample s = eval.d1(q.u, q.v);
      if (const math::Point3* collar = collarPins.find(q)) {
        m.vertices.push_back(*collar);
        m.normals.push_back(flip ? s.normal.reversed() : s.normal);
        continue;
      }
      if (const math::Point3* pin = seamPins.find(q)) {
        m.vertices.push_back(*pin);
        m.normals.push_back(flip ? s.normal.reversed() : s.normal);
        continue;
      }
      const math::Point3* anchor = anchors.find(s.point);
      m.vertices.push_back(anchor ? *anchor : s.point);
      m.normals.push_back(flip ? s.normal.reversed() : s.normal);
    }
    return m;
  }

  MeshParams p_;
  const SeamStripRegistry* seamStrips_ = nullptr;
};

}  // namespace cybercad::native::tessellate

#endif  // CYBERCAD_NATIVE_TESSELLATE_FACE_MESHER_H

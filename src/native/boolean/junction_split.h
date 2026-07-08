// SPDX-License-Identifier: Apache-2.0
//
// junction_split.h — MOAT M2-multiseam: the JUNCTION-AWARE face split, the one new
// verb the seam-GRAPH weld needs beyond the byte-frozen single-seam B2 `splitFace`.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// The landed seam-graph builder (`seam_graph.h`) assembles the two-arc, one-junction
// wall boundary: `arc0` (the `x = 0` cut) and `arc1` (the `y = 0` cut) meet at the
// analytic junction vertex `J`, joined into one bent boundary→J→boundary `jointSeam`.
// Feeding that bent seam to the byte-frozen B2 `splitFace` REACHES a geometrically-exact
// partition (`crossings == 2`, `tilingGap ≈ 0`) but DECLINES `RebuildMismatch`: B2 builds
// the whole seam as ONE degree-1 B-spline edge, and its fixed-density rebuild self-verify
// (`buildRegion` at 8 samples/edge) SHORTCUTS the sharp interior kink at `J` — the 9
// uniform samples never land exactly on `J`, so the reflattened corner sub-loop loses
// ~1e-5·parentArea and the strict rebuild tolerance (rightly) rejects it. Weakening that
// tolerance is FORBIDDEN.
//
// This verb resolves the kink by CONSTRUCTION: it introduces `J` as an EXACT shared
// valence-3 vertex, building the seam as TWO edges — `arc0`-half (E→J) and `arc1`-half
// (J→X) — that meet at `J`. Because the two arcs are ORTHOGONAL iso-parametric curves
// (`arc0` is u≈const → a vertical line in UV, `arc1` is v≈const → a horizontal line),
// the ONLY bend in the UV seam is the right-angle at `J`; making `J` an edge endpoint
// makes `buildRegion` sample it EXACTLY, so each straight-in-UV half reflattens to its
// combinatorial loop to machine precision and the rebuild self-verify PASSES — with the
// SAME strict tolerance B2 uses, never weakened.
//
// The wall partitions into the CORNER sub-face (the removed quadrant `A ∩ {x≥0, y≥0}`,
// its UV projection `Q ∩ {u≥½, v≥½}`) and the L-shaped SURVIVOR sub-face, sharing the
// bent seam (with `J` as the exact shared interior vertex) as their common boundary.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// Every `detail::` primitive of the byte-frozen B2 `face_split.h` (`flattenOuter`,
// `seamCross`, `buildSeamEdge`, `restrictEdge`, `shoelace`, `Crossing`, `BndSeg`,
// `orient2d`, `segmentsCross`), the same `tess::buildRegion` reflatten, and the same
// `SplitOptions` tolerances. Additive sibling — touches NONE of them, nor `splitFace`,
// nor the landed single-seam path. The junction-aware wire construction (two seam edges
// at `J`) is the only genuinely-new step.
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate is a geometry test. If the bent seam does not cross the boundary
// exactly twice, or `J` is not a node of the clipped chord, or a sub-loop is degenerate
// / non-simple, or the seam is not the bit-identical shared boundary, or the tiling gap
// or the rebuild residual exceeds the STRICT (unweakened) tolerance, the verb returns a
// MEASURED decline (nullopt) — a first-class outcome the caller logs before OCCT. No
// tolerance is weakened; no partial/leaky split is ever emitted.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_JUNCTION_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_JUNCTION_SPLIT_H

#include "native/boolean/face_split.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes. `Ok` iff a verified junction-aware two-face split is returned;
// every other value is an HONEST decline carrying WHY the bent seam is unusable.
// Mirrors `SplitDecline` plus the one junction-specific blocker.
// ─────────────────────────────────────────────────────────────────────────────
enum class JunctionDecline {
  Ok,
  NoOuterLoop,          ///< face has no usable outer wire / < 3 UV boundary points
  EmptySeam,            ///< the bent seam has < 3 nodes (needs boundary→J→boundary)
  CrossingsNot2,        ///< the bent seam does not cross the outer loop exactly twice
  InteriorNodeOutside,  ///< a seam node between the crossings is not strictly inside
  JunctionNotOnSeam,    ///< `J` is not an exact interior node of the clipped seam chord
  DegenerateSubRegion,  ///< a sub-loop has near-zero area or self-intersects
  SeamNotShared,        ///< the seam is not the bit-identical shared boundary of both sub-loops
  TilingGap,            ///< area(corner)+area(survivor) != area(parent) beyond tolerance
  RebuildMismatch       ///< a rebuilt sub-face's flattened UV loop does not match its loop
};

inline const char* junctionDeclineName(JunctionDecline d) noexcept {
  switch (d) {
    case JunctionDecline::Ok: return "Ok";
    case JunctionDecline::NoOuterLoop: return "NoOuterLoop";
    case JunctionDecline::EmptySeam: return "EmptySeam";
    case JunctionDecline::CrossingsNot2: return "CrossingsNot2";
    case JunctionDecline::InteriorNodeOutside: return "InteriorNodeOutside";
    case JunctionDecline::JunctionNotOnSeam: return "JunctionNotOnSeam";
    case JunctionDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case JunctionDecline::SeamNotShared: return "SeamNotShared";
    case JunctionDecline::TilingGap: return "TilingGap";
    case JunctionDecline::RebuildMismatch: return "RebuildMismatch";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// A verified junction-aware split. `loopCorner` is the removed quadrant
// `A ∩ {x≥0, y≥0}` (UV projection `Q ∩ {u≥½, v≥½}`); `loopSurvivor` is the L-shape.
// `seam` is the clipped bent chord `E … J … X` they SHARE (with `J` at `jIdx`); the
// two sub-faces walk it in opposite directions, so it is their bit-exact common
// boundary and `J` is the exact shared valence-3 vertex.
// ─────────────────────────────────────────────────────────────────────────────
struct JunctionFaceSplit {
  topo::Shape faceCorner;    ///< the removed-quadrant sub-face (discarded for FUSE, kept for COMMON)
  topo::Shape faceSurvivor;  ///< the L-shaped survivor sub-face (kept for FUSE)
  UVPolygon loopCorner;
  UVPolygon loopSurvivor;
  UVPolygon seam;            ///< the shared clipped bent chord (E … J … X)
  int jIdx = -1;             ///< index of `J` within `seam`
  double parentArea = 0.0;
  double areaCorner = 0.0;
  double areaSurvivor = 0.0;
  double rebuildResidual = 0.0;  ///< max |reflattened area − combinatorial area| (below rtol)
};

struct JunctionSplitResult {
  std::optional<JunctionFaceSplit> split;   ///< nullopt on decline
  JunctionDecline decline = JunctionDecline::Ok;
  int crossings = -1;
  double tilingGap = 0.0;
  double measuredArea = 0.0;
  bool ok() const noexcept { return split.has_value(); }
};

namespace jsdetail {

using detail::BndSeg;
using detail::Crossing;
using detail::buildSeamEdge;
using detail::flattenOuter;
using detail::restrictEdge;
using detail::seamCross;
using detail::shoelace;

// Walk boundary segments `from`→`to` (forward around the ring), grouping consecutive
// segments of the same parent edge into one restricted sub-edge — the byte-frozen B2
// arc-walk (`face_split.h` step 8), reused verbatim. `seamEdges` (in order, each with
// its orientation already applied) are spliced after the boundary arc to close the wire.
inline topo::Shape buildJunctionWire(const Crossing& from, const Crossing& to,
                                     const std::vector<BndSeg>& segs, int nB,
                                     const std::vector<topo::Shape>& edges, const topo::Shape& face,
                                     const topo::FaceSurface& surface, topo::Orientation orient,
                                     const std::vector<topo::Shape>& seamEdges) {
  std::vector<topo::Shape> wireEdges;
  int i = from.bndSeg;
  double groupStart = from.tEdge;
  int groupEdge = from.edgeIdx;
  while (true) {
    const bool isLast = (i == to.bndSeg);
    const double segEnd = isLast ? to.tEdge : segs[i].tb;
    const int next = (i + 1) % nB;
    const bool edgeChanges = isLast || segs[next].edgeIdx != groupEdge;
    if (edgeChanges) {
      wireEdges.push_back(restrictEdge(edges[groupEdge], face, groupStart, segEnd));
      if (isLast) break;
      groupEdge = segs[next].edgeIdx;
      groupStart = segs[next].ta;
    }
    i = next;
  }
  for (const topo::Shape& se : seamEdges) wireEdges.push_back(se);
  return topo::ShapeBuilder::makeFace(surface, topo::ShapeBuilder::makeWire(std::move(wireEdges)), {},
                                      orient);
}

}  // namespace jsdetail

// ─────────────────────────────────────────────────────────────────────────────
// splitFaceJunction — the junction-aware B2 companion. Partitions `face` along the
// bent `jointSeam` (boundary→`J`→boundary, `J` at `junctionUV` / `junction3d`) into
// the corner + survivor sub-faces, with `J` an EXACT shared valence-3 vertex. Returns
// a verified split, or an honest `JunctionDecline` with the measured blocker. `J` must
// be a bit-identical interior node of the seam (as produced by `buildSeamGraph`).
// ─────────────────────────────────────────────────────────────────────────────
inline JunctionSplitResult splitFaceJunction(const topo::Shape& face, const ssi::WLine& jointSeam,
                                             const UV& junctionUV, const math::Point3& junction3d,
                                             const SplitOptions& opts = {}) {
  using namespace detail;
  using namespace jsdetail;
  JunctionSplitResult r;

  // ── (1) flatten the outer loop; extract the bent seam polyline ─────────────
  std::vector<BndSeg> segs;
  std::vector<topo::Shape> edges;
  if (!flattenOuter(face, opts.segsPerEdge, segs, edges)) {
    r.decline = JunctionDecline::NoOuterLoop;
    return r;
  }
  if (jointSeam.points.size() < 3) {
    r.decline = JunctionDecline::EmptySeam;
    return r;
  }
  const int nB = static_cast<int>(segs.size());
  UVPolygon outer;
  outer.reserve(segs.size());
  for (const BndSeg& s : segs) outer.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outer));
  r.measuredArea = parentArea;

  UVPolygon seamUV;
  seamUV.reserve(jointSeam.points.size());
  for (const ssi::WLinePoint& p : jointSeam.points) seamUV.push_back(UV{p.u1, p.v1});
  const int nS = static_cast<int>(seamUV.size());

  // ── (2) seam × boundary crossings — exactly one entry + one exit ───────────
  std::vector<Crossing> xs;
  for (int k = 0; k + 1 < nS; ++k)
    for (int i = 0; i < nB; ++i) {
      double s = 0.0;
      UV cp{};
      if (seamCross(seamUV[k], seamUV[k + 1], segs[i].a, segs[i].b, s, cp)) {
        const double dx = segs[i].b.u - segs[i].a.u, dy = segs[i].b.v - segs[i].a.v;
        const double f = (std::fabs(dx) >= std::fabs(dy)) ? (cp.u - segs[i].a.u) / dx
                                                          : (cp.v - segs[i].a.v) / dy;
        const double tEdge = segs[i].ta + f * (segs[i].tb - segs[i].ta);
        xs.push_back(Crossing{cp, k, i, tEdge, segs[i].edgeIdx});
      }
    }
  r.crossings = static_cast<int>(xs.size());
  if (xs.size() != 2) {
    r.decline = JunctionDecline::CrossingsNot2;
    return r;
  }
  if (xs[0].seamSeg > xs[1].seamSeg) std::swap(xs[0], xs[1]);
  const Crossing E = xs[0], X = xs[1];

  // ── (3) every interior seam node strictly inside the outer loop ────────────
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k)
    if (!tess::pointInPolygon(outer, seamUV[k])) {
      r.decline = JunctionDecline::InteriorNodeOutside;
      return r;
    }

  // ── (4) clipped bent chord E … J … X, and locate J EXACTLY within it ───────
  UVPolygon seamChord;
  seamChord.push_back(E.p);
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) seamChord.push_back(seamUV[k]);
  seamChord.push_back(X.p);
  int jIdx = -1;
  for (int k = 1; k + 1 < static_cast<int>(seamChord.size()); ++k)
    if (seamChord[k].u == junctionUV.u && seamChord[k].v == junctionUV.v) { jIdx = k; break; }
  if (jIdx < 0) {
    r.decline = JunctionDecline::JunctionNotOnSeam;
    return r;
  }

  // ── (5) two boundary arcs + two sub-loops (byte-frozen B2 construction) ────
  auto boundaryArc = [&](int fromSeg, int toSeg) {
    UVPolygon arc;
    for (int i = (fromSeg + 1) % nB; i != (toSeg + 1) % nB; i = (i + 1) % nB) arc.push_back(segs[i].a);
    return arc;
  };
  const UVPolygon arc1 = boundaryArc(E.bndSeg, X.bndSeg);
  const UVPolygon arc2 = boundaryArc(X.bndSeg, E.bndSeg);

  UVPolygon loop1;  // E, arc1(…→X), X, seam interior reversed(…→E)
  loop1.push_back(E.p);
  for (const UV& q : arc1) loop1.push_back(q);
  loop1.push_back(X.p);
  for (int k = X.seamSeg; k >= E.seamSeg + 1; --k) loop1.push_back(seamUV[k]);

  UVPolygon loop2;  // X, arc2(…→E), E, seam interior forward(…→X)
  loop2.push_back(X.p);
  for (const UV& q : arc2) loop2.push_back(q);
  loop2.push_back(E.p);
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) loop2.push_back(seamUV[k]);

  // ── (6) SELF-VERIFY GATE (same strict tolerances as B2) ────────────────────
  const double a1 = std::fabs(shoelace(loop1));
  const double a2 = std::fabs(shoelace(loop2));
  const double floor = std::max(parentArea, 1.0) * opts.areaFloorFrac;
  if (a1 < floor || a2 < floor) {
    r.decline = JunctionDecline::DegenerateSubRegion;
    return r;
  }
  auto simple = [](const UVPolygon& p) {
    const int n = static_cast<int>(p.size());
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j) {
        const int i1 = (i + 1) % n, j1 = (j + 1) % n;
        if (i == j1 || j == i1) continue;
        if (segmentsCross(p[i], p[i1], p[j], p[j1])) return false;
      }
    return true;
  };
  if (!simple(loop1) || !simple(loop2)) {
    r.decline = JunctionDecline::DegenerateSubRegion;
    return r;
  }
  const int m = static_cast<int>(seamChord.size());
  const int n1 = static_cast<int>(loop1.size());
  const int n2 = static_cast<int>(loop2.size());
  const int start1 = 1 + static_cast<int>(arc1.size());
  const int start2 = 1 + static_cast<int>(arc2.size());
  bool shared = true;
  for (int k = 0; k < m && shared; ++k) {
    const UV& s1 = loop1[(start1 + k) % n1];
    const UV& s2 = loop2[(start2 + k) % n2];
    const UV& refRev = seamChord[m - 1 - k];
    const UV& refFwd = seamChord[k];
    if (s1.u != refRev.u || s1.v != refRev.v || s2.u != refFwd.u || s2.v != refFwd.v) shared = false;
  }
  if (!shared) {
    r.decline = JunctionDecline::SeamNotShared;
    return r;
  }
  const double gap = std::fabs(parentArea - (a1 + a2));
  r.tilingGap = gap;
  if (gap > std::max(parentArea, 1.0) * opts.tilingTolFrac) {
    r.decline = JunctionDecline::TilingGap;
    return r;
  }

  // ── (7) place the seam chord on the surface (3D); J uses the ANALYTIC point ─
  const auto sr = topo::surfaceOf(face);
  if (!sr) {
    r.decline = JunctionDecline::NoOuterLoop;
    return r;
  }
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(seamChord.size());
  seam3d.push_back(eval.value(E.p.u, E.p.v));
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) seam3d.push_back(eval.value(seamUV[k].u, seamUV[k].v));
  seam3d.push_back(eval.value(X.p.u, X.p.v));
  seam3d[jIdx] = junction3d;  // the exact analytic junction (graph-verified on both planes)

  // ── (8) build TWO seam edges split at J → the exact shared valence-3 vertex ─
  // arc0-half = chord[0 .. jIdx] (E→J); arc1-half = chord[jIdx .. end] (J→X). Each is
  // straight in UV (arc0 u-const, arc1 v-const), so its degree-1 pcurve reflattens
  // EXACTLY; making J an edge endpoint stops buildRegion from shortcutting the kink.
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();
  UVPolygon chordA(seamChord.begin(), seamChord.begin() + (jIdx + 1));
  UVPolygon chordB(seamChord.begin() + jIdx, seamChord.end());
  std::vector<math::Point3> seam3dA(seam3d.begin(), seam3d.begin() + (jIdx + 1));
  std::vector<math::Point3> seam3dB(seam3d.begin() + jIdx, seam3d.end());
  const topo::Shape seamEdgeA = buildSeamEdge(chordA, seam3dA, face.tshape());  // E→J
  const topo::Shape seamEdgeB = buildSeamEdge(chordB, seam3dB, face.tshape());  // J→X

  // face1: boundary E→X, then seam X→J→E (seamEdgeB reversed, seamEdgeA reversed).
  const topo::Shape face1 = buildJunctionWire(E, X, segs, nB, edges, face, surface, orient,
                                              {seamEdgeB.reversedShape(), seamEdgeA.reversedShape()});
  // face2: boundary X→E, then seam E→J→X (seamEdgeA forward, seamEdgeB forward). Opposite
  // seam orientation on BOTH sub-edges ⇒ shared bit-exact boundary + shared vertex J.
  const topo::Shape face2 =
      buildJunctionWire(X, E, segs, nB, edges, face, surface, orient, {seamEdgeA, seamEdgeB});

  // ── (9) rebuild self-verify — SAME strict rtol B2 uses, NOW satisfied because J
  // is an exact vertex (each straight-in-UV half reflattens to its combinatorial loop). ─
  const tess::UVRegion reg1 = tess::buildRegion(face1, std::max(opts.segsPerEdge, 8));
  const tess::UVRegion reg2 = tess::buildRegion(face2, std::max(opts.segsPerEdge, 8));
  const double rtol = std::max(parentArea, 1.0) * opts.rebuildTolFrac;
  if (!reg1.hasOuter() || !reg2.hasOuter()) {
    r.decline = JunctionDecline::RebuildMismatch;
    return r;
  }
  const double e1 = std::fabs(std::fabs(shoelace(reg1.outer)) - a1);
  const double e2 = std::fabs(std::fabs(shoelace(reg2.outer)) - a2);
  if (e1 > rtol || e2 > rtol) {
    r.decline = JunctionDecline::RebuildMismatch;
    return r;
  }

  // ── (10) label corner vs survivor: the removed quadrant sits at upper-right in UV
  // (u≥½, v≥½), so its centroid maximises u+v. Grounded against the closed-form UV
  // corner area by the host gate. ────────────────────────────────────────────────
  auto centroidUV = [](const UVPolygon& p) {
    double u = 0.0, v = 0.0;
    for (const UV& q : p) { u += q.u; v += q.v; }
    const double n = static_cast<double>(p.size());
    return UV{u / n, v / n};
  };
  const UV c1 = centroidUV(loop1), c2 = centroidUV(loop2);
  JunctionFaceSplit fs;
  if (c1.u + c1.v >= c2.u + c2.v) {
    fs.faceCorner = face1; fs.loopCorner = loop1; fs.areaCorner = a1;
    fs.faceSurvivor = face2; fs.loopSurvivor = loop2; fs.areaSurvivor = a2;
  } else {
    fs.faceCorner = face2; fs.loopCorner = loop2; fs.areaCorner = a2;
    fs.faceSurvivor = face1; fs.loopSurvivor = loop1; fs.areaSurvivor = a1;
  }
  fs.seam = seamChord;
  fs.jIdx = jIdx;
  fs.parentArea = parentArea;
  fs.rebuildResidual = std::max(e1, e2);
  r.split = std::move(fs);
  r.decline = JunctionDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_JUNCTION_SPLIT_H

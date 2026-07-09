// SPDX-License-Identifier: Apache-2.0
//
// strip_split.h — MOAT M2 blocker #4 (≥3-seam weld): the TWO-junction WALL SPLIT, the
// additive generalisation of the landed one-junction `splitFaceJunction`
// (`junction_split.h`) to the three-arc, two-junction seam CHAIN.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// `splitFaceJunction` partitions the wall along a bent boundary→`J`→boundary seam (ONE
// interior valence-3 vertex) into the removed CORNER quadrant + the L-shaped survivor.
// The chain seam-graph builder (`seam_graph_chain.h`) produces a bent
// boundary→`J1`→[middle]→`J2`→boundary `chainSeam` with TWO interior junctions. Feeding
// that to the byte-frozen B2 `splitFace` reaches an exact partition (`crossings == 2`)
// but DECLINES `RebuildMismatch`: B2 builds the whole seam as ONE degree-1 edge and its
// fixed-density reflatten SHORTCUTS BOTH sharp interior kinks (`J1`, `J2`).
//
// This verb resolves BOTH kinks by CONSTRUCTION, exactly as the one-junction verb does:
// it introduces `J1` and `J2` as EXACT shared valence-3 vertices, building the seam as
// THREE edges — `end0`-half (E→J1), the `middle` (J1→J2), `end1`-half (J2→X) — that meet
// at `J1`, `J2`. On the edge-straddling pose the three arcs are iso-parametric (the two
// ends iso-`u`, the middle iso-`v`), so EACH sub-edge is straight in UV; making `J1`/`J2`
// edge endpoints makes `buildRegion` sample them EXACTLY, so each straight-in-UV part
// reflattens to its combinatorial loop to machine precision and the rebuild self-verify
// PASSES with the SAME strict tolerance B2 uses — never weakened.
//
// The wall partitions into the removed STRIP sub-face (the removed region
// `A ∩ {x0 ≤ x ≤ x1, y ≥ y0}`, its UV projection `Q ∩ {u0 ≤ u ≤ u1, v ≥ v0}`) and the
// SURVIVOR sub-face, sharing the bent chain seam (with `J1`, `J2` as exact shared
// interior vertices) as their common boundary.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// Every `detail::` primitive of the byte-frozen B2 `face_split.h` (`flattenOuter`,
// `seamCross`, `buildSeamEdge`, `restrictEdge`, `shoelace`, `Crossing`, `BndSeg`,
// `segmentsCross`), `jsdetail::buildJunctionWire` (the byte-frozen one-junction arc-walk,
// reused verbatim), the same `tess::buildRegion` reflatten, and the same `SplitOptions`
// tolerances. Additive sibling — touches NONE of them, nor `splitFace`, nor
// `splitFaceJunction`, nor the landed single/one-junction paths. The only new step is the
// two-junction wire construction (three seam edges at `J1`, `J2`).
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate is a geometry test. If the bent seam does not cross the boundary
// exactly twice, or `J1`/`J2` are not exact interior nodes of the clipped chord, or a
// sub-loop is degenerate / non-simple, or the seam is not the bit-identical shared
// boundary, or the tiling gap or the rebuild residual exceeds the STRICT (unweakened)
// tolerance, the verb returns a MEASURED `StripSplitDecline` (nullopt). No tolerance is
// weakened; no partial/leaky split is ever emitted.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_STRIP_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_STRIP_SPLIT_H

#include "native/boolean/face_split.h"
#include "native/boolean/junction_split.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes. `Ok` iff a verified two-junction two-face split is returned; every
// other value is an HONEST decline carrying WHY the bent chain seam is unusable.
// Mirrors `JunctionDecline` plus the two-junction-specific node blockers.
// ─────────────────────────────────────────────────────────────────────────────
enum class StripSplitDecline {
  Ok,
  NoOuterLoop,          ///< face has no usable outer wire / < 3 UV boundary points
  EmptySeam,            ///< the bent chain seam has < 4 nodes (needs boundary→J1→J2→boundary)
  CrossingsNot2,        ///< the bent seam does not cross the outer loop exactly twice
  InteriorNodeOutside,  ///< a seam node between the crossings is not strictly inside
  JunctionNotOnSeam,    ///< `J1` or `J2` is not an exact interior node of the clipped chord
  DegenerateSubRegion,  ///< a sub-loop has near-zero area or self-intersects
  SeamNotShared,        ///< the seam is not the bit-identical shared boundary of both sub-loops
  TilingGap,            ///< area(strip)+area(survivor) != area(parent) beyond tolerance
  RebuildMismatch       ///< a rebuilt sub-face's flattened UV loop does not match its loop
};

inline const char* stripSplitDeclineName(StripSplitDecline d) noexcept {
  switch (d) {
    case StripSplitDecline::Ok: return "Ok";
    case StripSplitDecline::NoOuterLoop: return "NoOuterLoop";
    case StripSplitDecline::EmptySeam: return "EmptySeam";
    case StripSplitDecline::CrossingsNot2: return "CrossingsNot2";
    case StripSplitDecline::InteriorNodeOutside: return "InteriorNodeOutside";
    case StripSplitDecline::JunctionNotOnSeam: return "JunctionNotOnSeam";
    case StripSplitDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case StripSplitDecline::SeamNotShared: return "SeamNotShared";
    case StripSplitDecline::TilingGap: return "TilingGap";
    case StripSplitDecline::RebuildMismatch: return "RebuildMismatch";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// A verified two-junction split. `loopStrip` is the removed strip
// `A ∩ {u0 ≤ u ≤ u1, v ≥ v0}`; `loopSurvivor` is the remainder. `seam` is the clipped
// bent chord `E … J1 … J2 … X` they SHARE (with `J1` at `j1Idx`, `J2` at `j2Idx`); the
// two sub-faces walk it in opposite directions, so it is their bit-exact common boundary
// and `J1`, `J2` are the exact shared valence-3 vertices.
// ─────────────────────────────────────────────────────────────────────────────
struct StripFaceSplit {
  topo::Shape faceStrip;     ///< the removed-strip sub-face (discarded for FUSE, kept for COMMON)
  topo::Shape faceSurvivor;  ///< the survivor sub-face (kept for FUSE / CUT)
  UVPolygon loopStrip;
  UVPolygon loopSurvivor;
  UVPolygon seam;            ///< the shared clipped bent chord (E … J1 … J2 … X)
  int j1Idx = -1;            ///< index of `J1` within `seam`
  int j2Idx = -1;            ///< index of `J2` within `seam`
  double parentArea = 0.0;
  double areaStrip = 0.0;
  double areaSurvivor = 0.0;
  double rebuildResidual = 0.0;  ///< max |reflattened area − combinatorial area| (below rtol)
};

struct StripSplitResult {
  std::optional<StripFaceSplit> split;   ///< nullopt on decline
  StripSplitDecline decline = StripSplitDecline::Ok;
  int crossings = -1;
  double tilingGap = 0.0;
  double measuredArea = 0.0;
  bool ok() const noexcept { return split.has_value(); }
};

namespace ssdetail {

using detail::BndSeg;
using detail::Crossing;
using detail::buildSeamEdge;
using detail::flattenOuter;
using detail::restrictEdge;
using detail::shoelace;
using detail::seamCross;
using detail::segmentsCross;

/// Build ONE sub-face wire that walks the boundary from crossing `from` to crossing `to`
/// FORWARD around the ring, then splices the ordered `seamEdges` to close it. Generalises
/// `jsdetail::buildJunctionWire` to the edge-straddling pose where the strip pokes through
/// ONE boundary edge, so `from.bndSeg == to.bndSeg` can denote EITHER the short within-edge
/// sub-segment (`fullWrap == false`) OR the full-ring wrap (`fullWrap == true`) — the whole
/// ring visited once, the shared edge split at both crossings. Reuses the byte-frozen
/// `restrictEdge` grouping verbatim; the only new logic is the full-wrap termination.
inline topo::Shape buildStripWire(const Crossing& from, const Crossing& to,
                                  const std::vector<BndSeg>& segs, int nB,
                                  const std::vector<topo::Shape>& edges, const topo::Shape& face,
                                  const topo::FaceSurface& surface, topo::Orientation orient,
                                  bool fullWrap, const std::vector<topo::Shape>& seamEdges) {
  // Number of segment-steps to walk forward from `from.bndSeg` to `to.bndSeg`: the ring
  // distance, but a full wrap of a same-segment pair walks all nB segments (not zero).
  int steps = (to.bndSeg - from.bndSeg + nB) % nB;
  if (fullWrap && steps == 0) steps = nB;
  std::vector<topo::Shape> wireEdges;
  int i = from.bndSeg;
  double groupStart = from.tEdge;
  int groupEdge = from.edgeIdx;
  for (int walked = 0;; ++walked) {
    const bool isLast = (walked == steps);
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

}  // namespace ssdetail

// ─────────────────────────────────────────────────────────────────────────────
// splitFaceStrip — the two-junction B2 companion. Partitions `face` along the bent
// `chainSeam` (boundary→`J1`→`J2`→boundary, `J1` at `j1UV`/`j1_3d`, `J2` at `j2UV`/
// `j2_3d`) into the strip + survivor sub-faces, with `J1`, `J2` EXACT shared valence-3
// vertices. Returns a verified split, or an honest `StripSplitDecline`. `J1`, `J2` must
// be bit-identical interior nodes of the seam (as produced by `buildChainSeamGraph`).
// ─────────────────────────────────────────────────────────────────────────────
inline StripSplitResult splitFaceStrip(const topo::Shape& face, const ssi::WLine& chainSeam,
                                       const UV& j1UV, const math::Point3& j1_3d,
                                       const UV& j2UV, const math::Point3& j2_3d,
                                       const SplitOptions& opts = {}) {
  using namespace detail;
  using namespace ssdetail;
  StripSplitResult r;

  // ── (1) flatten the outer loop; extract the bent chain seam polyline ───────
  std::vector<BndSeg> segs;
  std::vector<topo::Shape> edges;
  if (!flattenOuter(face, opts.segsPerEdge, segs, edges)) {
    r.decline = StripSplitDecline::NoOuterLoop;
    return r;
  }
  if (chainSeam.points.size() < 4) {
    r.decline = StripSplitDecline::EmptySeam;
    return r;
  }
  const int nB = static_cast<int>(segs.size());
  UVPolygon outer;
  outer.reserve(segs.size());
  for (const BndSeg& s : segs) outer.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outer));
  r.measuredArea = parentArea;

  UVPolygon seamUV;
  seamUV.reserve(chainSeam.points.size());
  for (const ssi::WLinePoint& p : chainSeam.points) seamUV.push_back(UV{p.u1, p.v1});
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
    r.decline = StripSplitDecline::CrossingsNot2;
    return r;
  }
  if (xs[0].seamSeg > xs[1].seamSeg) std::swap(xs[0], xs[1]);
  const Crossing E = xs[0], X = xs[1];

  // ── (3) every interior seam node strictly inside the outer loop ────────────
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k)
    if (!tess::pointInPolygon(outer, seamUV[k])) {
      r.decline = StripSplitDecline::InteriorNodeOutside;
      return r;
    }

  // ── (4) clipped bent chord E … J1 … J2 … X, locate J1 and J2 EXACTLY ───────
  UVPolygon seamChord;
  seamChord.push_back(E.p);
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) seamChord.push_back(seamUV[k]);
  seamChord.push_back(X.p);
  int j1Idx = -1, j2Idx = -1;
  for (int k = 1; k + 1 < static_cast<int>(seamChord.size()); ++k) {
    if (j1Idx < 0 && seamChord[k].u == j1UV.u && seamChord[k].v == j1UV.v) j1Idx = k;
    else if (seamChord[k].u == j2UV.u && seamChord[k].v == j2UV.v) j2Idx = k;
  }
  if (j1Idx < 0 || j2Idx < 0 || j1Idx >= j2Idx) {
    r.decline = StripSplitDecline::JunctionNotOnSeam;
    return r;
  }

  // ── (5) two boundary arcs + two sub-loops (byte-frozen B2 construction) ────
  // The strip on the edge-straddling pose pokes THROUGH ONE boundary edge, so BOTH
  // crossings can share a segment (`E.bndSeg == X.bndSeg`). In that case the forward
  // arc E→X inside that segment is EMPTY (no vertex between them) and the complement
  // arc X→E wraps the WHOLE ring (all nB vertices). Disambiguate by the on-edge param:
  // walking whole vertices alone (the one-junction builder) cannot tell the two apart.
  const bool sameSeg = (E.bndSeg == X.bndSeg);
  auto boundaryArc = [&](int fromSeg, int toSeg, bool forwardWithinSeg) {
    UVPolygon arc;
    if (sameSeg) {
      // forwardWithinSeg: E→X within the segment (empty) vs the full wrap X→E (all vertices).
      if (forwardWithinSeg) return arc;                        // empty short arc
      for (int i = (fromSeg + 1) % nB; ; i = (i + 1) % nB) {    // full ring wrap
        arc.push_back(segs[i].a);
        if (i == fromSeg) break;
      }
      return arc;
    }
    for (int i = (fromSeg + 1) % nB; i != (toSeg + 1) % nB; i = (i + 1) % nB) arc.push_back(segs[i].a);
    return arc;
  };
  // arc1 = boundary E→X (forward within-seg when same); arc2 = boundary X→E (full wrap).
  const bool eBeforeX = !sameSeg || (E.tEdge <= X.tEdge);
  const UVPolygon arc1 = boundaryArc(E.bndSeg, X.bndSeg, eBeforeX);
  const UVPolygon arc2 = boundaryArc(X.bndSeg, E.bndSeg, !eBeforeX);

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
    r.decline = StripSplitDecline::DegenerateSubRegion;
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
    r.decline = StripSplitDecline::DegenerateSubRegion;
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
    r.decline = StripSplitDecline::SeamNotShared;
    return r;
  }
  const double gap = std::fabs(parentArea - (a1 + a2));
  r.tilingGap = gap;
  if (gap > std::max(parentArea, 1.0) * opts.tilingTolFrac) {
    r.decline = StripSplitDecline::TilingGap;
    return r;
  }

  // ── (7) place the seam chord on the surface (3D); J1/J2 use the ANALYTIC pts ─
  const auto sr = topo::surfaceOf(face);
  if (!sr) {
    r.decline = StripSplitDecline::NoOuterLoop;
    return r;
  }
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(seamChord.size());
  seam3d.push_back(eval.value(E.p.u, E.p.v));
  for (int k = E.seamSeg + 1; k <= X.seamSeg; ++k) seam3d.push_back(eval.value(seamUV[k].u, seamUV[k].v));
  seam3d.push_back(eval.value(X.p.u, X.p.v));
  seam3d[j1Idx] = j1_3d;  // the exact analytic junctions (graph-verified on both planes)
  seam3d[j2Idx] = j2_3d;

  // ── (8) build THREE seam edges split at J1, J2 → exact shared valence-3 verts ─
  // end0-half = chord[0 .. j1Idx] (E→J1); middle = chord[j1Idx .. j2Idx] (J1→J2);
  // end1-half = chord[j2Idx .. end] (J2→X). Each is straight in UV (ends u-const,
  // middle v-const), so its degree-1 pcurve reflattens EXACTLY; making J1/J2 edge
  // endpoints stops buildRegion from shortcutting either kink.
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();
  UVPolygon chordA(seamChord.begin(), seamChord.begin() + (j1Idx + 1));
  UVPolygon chordM(seamChord.begin() + j1Idx, seamChord.begin() + (j2Idx + 1));
  UVPolygon chordB(seamChord.begin() + j2Idx, seamChord.end());
  std::vector<math::Point3> seam3dA(seam3d.begin(), seam3d.begin() + (j1Idx + 1));
  std::vector<math::Point3> seam3dM(seam3d.begin() + j1Idx, seam3d.begin() + (j2Idx + 1));
  std::vector<math::Point3> seam3dB(seam3d.begin() + j2Idx, seam3d.end());
  const topo::Shape seamEdgeA = buildSeamEdge(chordA, seam3dA, face.tshape());  // E→J1
  const topo::Shape seamEdgeM = buildSeamEdge(chordM, seam3dM, face.tshape());  // J1→J2
  const topo::Shape seamEdgeB = buildSeamEdge(chordB, seam3dB, face.tshape());  // J2→X

  // On the same-segment pose exactly ONE of the two boundary arcs is the full-ring wrap:
  // the one whose within-segment direction is NOT the short E→X sub-segment.
  const bool face1Wrap = sameSeg && !eBeforeX;  // E→X is the wrap iff X is "before" E
  const bool face2Wrap = sameSeg && eBeforeX;   // X→E is the wrap iff E is "before" X
  // face1: boundary E→X (short arc when same-seg), then seam X→J2→J1→E (all reversed).
  const topo::Shape face1 =
      buildStripWire(E, X, segs, nB, edges, face, surface, orient, face1Wrap,
                     {seamEdgeB.reversedShape(), seamEdgeM.reversedShape(), seamEdgeA.reversedShape()});
  // face2: boundary X→E (full-ring wrap when same-seg), then seam E→J1→J2→X (all forward).
  // Opposite seam orientation on all sub-edges ⇒ shared bit-exact boundary + shared J1, J2.
  const topo::Shape face2 =
      buildStripWire(X, E, segs, nB, edges, face, surface, orient, face2Wrap,
                     {seamEdgeA, seamEdgeM, seamEdgeB});

  // ── (9) rebuild self-verify — SAME strict rtol B2 uses, NOW satisfied because
  // J1/J2 are exact vertices (each straight-in-UV part reflattens to its loop). ─
  const tess::UVRegion reg1 = tess::buildRegion(face1, std::max(opts.segsPerEdge, 8));
  const tess::UVRegion reg2 = tess::buildRegion(face2, std::max(opts.segsPerEdge, 8));
  const double rtol = std::max(parentArea, 1.0) * opts.rebuildTolFrac;
  if (!reg1.hasOuter() || !reg2.hasOuter()) {
    r.decline = StripSplitDecline::RebuildMismatch;
    return r;
  }
  const double e1 = std::fabs(std::fabs(shoelace(reg1.outer)) - a1);
  const double e2 = std::fabs(std::fabs(shoelace(reg2.outer)) - a2);
  if (e1 > rtol || e2 > rtol) {
    r.decline = StripSplitDecline::RebuildMismatch;
    return r;
  }

  // ── (10) label strip vs survivor GEOMETRICALLY: the removed strip is the sub-loop
  // whose centroid lies inside `{u0 ≤ u ≤ u1, v ≥ v0}` (u0,u1 = the two junctions'
  // iso-u; v0 = their shared iso-v). Grounded against the closed-form UV strip area by
  // the host gate. ─────────────────────────────────────────────────────────────────
  auto centroidUV = [](const UVPolygon& p) {
    double u = 0.0, v = 0.0;
    for (const UV& q : p) { u += q.u; v += q.v; }
    const double n = static_cast<double>(p.size());
    return UV{u / n, v / n};
  };
  const double uLo = std::min(j1UV.u, j2UV.u), uHi = std::max(j1UV.u, j2UV.u);
  const double vMid = 0.5 * (j1UV.v + j2UV.v);
  auto inStrip = [&](const UV& c) { return c.u >= uLo && c.u <= uHi && c.v >= vMid; };
  const UV c1 = centroidUV(loop1), c2 = centroidUV(loop2);
  StripFaceSplit fs;
  const bool oneIsStrip = inStrip(c1) && !inStrip(c2);
  if (oneIsStrip) {
    fs.faceStrip = face1; fs.loopStrip = loop1; fs.areaStrip = a1;
    fs.faceSurvivor = face2; fs.loopSurvivor = loop2; fs.areaSurvivor = a2;
  } else {
    fs.faceStrip = face2; fs.loopStrip = loop2; fs.areaStrip = a2;
    fs.faceSurvivor = face1; fs.loopSurvivor = loop1; fs.areaSurvivor = a1;
  }
  fs.seam = seamChord;
  fs.j1Idx = j1Idx;
  fs.j2Idx = j2Idx;
  fs.parentArea = parentArea;
  fs.rebuildResidual = std::max(e1, e2);
  r.split = std::move(fs);
  r.decline = StripSplitDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_STRIP_SPLIT_H

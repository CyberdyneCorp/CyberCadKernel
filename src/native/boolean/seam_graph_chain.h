// SPDX-License-Identifier: Apache-2.0
//
// seam_graph_chain.h — MOAT M2 blocker #4 (≥3-seam seam graph): the CHAIN seam-graph
// builder, the additive generalisation of the landed two-arc, one-junction
// `buildSeamGraph` (`seam_graph.h`) to THREE adjacent cutting faces meeting at TWO
// interior junctions.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// `seam_graph.h` assembles the inter-solid wall boundary for the CORNER-box pose: two
// adjacent `B` faces slice `A`'s Bézier wall in two arcs meeting at ONE junction `J`.
// The next roadmap blocker is the ≥3-seam graph. This header assembles the reachable
// first step: an EDGE-straddling box `B` whose THREE consecutive faces (`P0`, `Pm`,
// `P1`) each slice `A`'s wall, so the inter-solid boundary is a CHAIN of three arcs
//
//     boundary → J1 → J2 → boundary
//
// meeting at TWO interior junctions. `P0` (`x = x0`) and `P1` (`x = x1`) are the two
// PARALLEL end faces (each an iso-`u` arc `arc0`, `arc1`); `Pm` (`y = y0`) is the
// ORTHOGONAL middle face (an iso-`v` arc `arcM` bounded by BOTH junctions). The removed
// region is the STRIP `A ∩ {x0 ≤ x ≤ x1, y ≥ y0}`.
//
// Both junctions are computed ANALYTICALLY (not sampled), exactly as the landed
// one-junction builder does: `arc0`/`arc1` are iso-`u` (constant `u = u(x0)` / `u(x1)`),
// `arcM` is iso-`v` (constant `v = v(y0)`), so
//
//     J1 = wall(u(arc0), v(arcM)),   J2 = wall(u(arc1), v(arcM)),
//
// and the builder VERIFIES each `Jk` lies on BOTH of its adjacent cutting planes
// (residual < weldTol) AND inside the trimmed wall — the grounding that the chain
// closes. The three arcs are clipped at their junctions and joined into ONE
// `chainSeam` (boundary→J1→J2→boundary) whose interior vertices are `J1`, `J2` — the
// shared boundary of the removed strip sub-face and the survivor sub-face.
//
// ── SCOPE + HONESTY ─────────────────────────────────────────────────────────────
// This is the GRAPH BUILDER + its standalone closed-graph oracle (arcs iso-parametric,
// both junctions on-plane and in-domain, the three arcs chain end-to-end within tol).
// It is the direct analogue of the landed `buildSeamGraph`, which lands its graph in
// isolation and records the WELD as the next blocker. The 2-junction WALL SPLIT + strip
// weld is the sharpened next blocker (a generalisation of `junction_split.h` /
// `multi_face_weld.h` to two interior valence-3 vertices), tracked after this enabler.
// Every predicate is a geometry test; anything outside the reachable 3-face/2-junction
// pose is a MEASURED `ChainSeamDecline` (nullopt), never a fudge, never a partial graph.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// B1 `recogniseFreeformSolid`, M1 `hscdetail::traceWallSeam`, every `isdetail::`
// primitive of the landed `inter_solid_seam.h` (`planeStraddlesWall`, `aabbInsidePlane`,
// `tracePlaneOf`, `wallWorldPoles`), every `sgdetail::` primitive of the landed
// `seam_graph.h` (`arcIsoParam`, `wallOuterUV`, `interiorSubArc`, `signedDist`), and
// `extractPolygons`/`isAllPlanar`. Additive sibling — touches NONE of them, nor
// `buildSeamGraph`, nor the byte-frozen B2 `splitFace`.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_CHAIN_H
#define CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_CHAIN_H

#include "native/boolean/inter_solid_seam.h"
#include "native/boolean/seam_graph.h"
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace ssi = cybercad::native::ssi;

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a closed
/// three-arc, two-junction seam graph is returned.
enum class ChainSeamDecline {
  Ok,
  NotPlanarB,             ///< `B` is not an all-planar solid
  NotThreeCuttingFaces,   ///< the number of `B` faces slicing `A`'s wall is not exactly three
  NotContained,           ///< a non-cutting `B` face does not contain `A`
  NotChainTopology,       ///< the three cutting faces are not two-parallel-ends + one-middle
  SeamUnusable,           ///< M1 arc missing / < 2 nodes for a cutting face
  ArcNotIsoParam,         ///< a traced arc is not iso-parametric on the wall
  JunctionUnusable,       ///< a junction is outside the trimmed wall UV
  JunctionOffPlane,       ///< a junction is not on BOTH its adjacent cutting planes
  JunctionNotJoined       ///< the clipped arcs do not close at a junction within tol
};

inline const char* chainSeamDeclineName(ChainSeamDecline d) noexcept {
  switch (d) {
    case ChainSeamDecline::Ok: return "Ok";
    case ChainSeamDecline::NotPlanarB: return "NotPlanarB";
    case ChainSeamDecline::NotThreeCuttingFaces: return "NotThreeCuttingFaces";
    case ChainSeamDecline::NotContained: return "NotContained";
    case ChainSeamDecline::NotChainTopology: return "NotChainTopology";
    case ChainSeamDecline::SeamUnusable: return "SeamUnusable";
    case ChainSeamDecline::ArcNotIsoParam: return "ArcNotIsoParam";
    case ChainSeamDecline::JunctionUnusable: return "JunctionUnusable";
    case ChainSeamDecline::JunctionOffPlane: return "JunctionOffPlane";
    case ChainSeamDecline::JunctionNotJoined: return "JunctionNotJoined";
  }
  return "?";
}

/// The assembled three-arc, two-junction seam graph. `arcs[0]`, `arcs[1]` are the two
/// PARALLEL end faces (iso-`u`); `arcs[2]` is the ORTHOGONAL middle face (iso-`v`).
struct ChainSeamGraph {
  std::vector<Polygon> bPolys;                 ///< every planar face of `B` (outward normals)
  std::array<std::size_t, 3> cutIdx{0, 0, 0};  ///< bPolys indices: {end0, end1, middle}
  std::array<SeamArc, 3> arcs;                 ///< {end0 (iso-u), end1 (iso-u), middle (iso-v)}
  std::array<tess::UV, 2> junctionUV{};        ///< J1, J2 in the wall's own (u,v) domain
  std::array<math::Point3, 2> junction3d{};    ///< J1, J2 in world coordinates
  double junctionPlaneResidual = 0.0;          ///< max |signedDist(P, J)| over both junctions
  double junctionJoinGap = 0.0;                ///< max clipped-arc closure residual over J1, J2
  ssi::WLine chainSeam;                         ///< end0(→J1)+J1+middle(J1→J2)+J2+end1(J2→), bent
};

namespace sgcdetail {

using hscdetail::signedDist;

/// (1) The cutting-face SET: EXACTLY three faces straddle the wall; the rest contain
/// `A`. Writes the three indices (order arbitrary here; the chain order is resolved in
/// (2b) once the arcs are traced + iso-classified).
inline ChainSeamDecline findThreeCuttingFaces(ChainSeamGraph& g,
                                              const std::vector<math::Point3>& poles,
                                              const Aabb& bbox, double band) {
  std::vector<std::size_t> cut;
  for (std::size_t i = 0; i < g.bPolys.size(); ++i)
    if (isdetail::planeStraddlesWall(g.bPolys[i], poles, band)) cut.push_back(i);
  if (cut.size() != 3) return ChainSeamDecline::NotThreeCuttingFaces;
  for (std::size_t i = 0; i < g.bPolys.size(); ++i)
    if (i != cut[0] && i != cut[1] && i != cut[2] &&
        !isdetail::aabbInsidePlane(bbox, g.bPolys[i], band))
      return ChainSeamDecline::NotContained;
  g.cutIdx = {cut[0], cut[1], cut[2]};
  return ChainSeamDecline::Ok;
}

/// (2a) Trace each cutting face's arc and classify its iso-parametric structure.
inline ChainSeamDecline traceArcs(ChainSeamGraph& g, const FreeformOperand& A,
                                  const topo::FaceSurface& fs, double isoTol) {
  for (int k = 0; k < 3; ++k) {
    SeamArc& sa = g.arcs[k];
    sa.cutIdx = g.cutIdx[k];
    sa.tracePlane = isdetail::tracePlaneOf(g.bPolys[g.cutIdx[k]]);
    sa.arc = hscdetail::traceWallSeam(A, fs, sa.tracePlane);
    if (sa.arc.points.size() < 2) return ChainSeamDecline::SeamUnusable;
    if (!sgdetail::arcIsoParam(sa.arc, isoTol, sa.uConst, sa.isoVal))
      return ChainSeamDecline::ArcNotIsoParam;
  }
  return ChainSeamDecline::Ok;
}

/// (2b) Resolve the chain order: the MIDDLE arc is the one whose iso-orientation is
/// unique among the three (the two ENDS are parallel — same `uConst` — and the middle
/// is orthogonal). Reorders `g.arcs`/`g.cutIdx` to {end0, end1, middle}. Returns the
/// NotChainTopology decline if the three arcs are not two-parallel + one-orthogonal, or
/// the two ends are not on opposite sides of the middle's varying span.
inline ChainSeamDecline orderChain(ChainSeamGraph& g) {
  int uCount = 0;
  for (int k = 0; k < 3; ++k) uCount += g.arcs[k].uConst ? 1 : 0;
  // Two ends parallel (share uConst) + one orthogonal middle ⇒ uCount is 1 or 2.
  const bool endUConst = (uCount == 2);  // ends are iso-u when two arcs are iso-u
  int mid = -1;
  std::array<int, 2> ends{-1, -1};
  int ne = 0;
  for (int k = 0; k < 3; ++k) {
    if (g.arcs[k].uConst == endUConst) { if (ne < 2) ends[ne++] = k; }
    else mid = k;
  }
  if (mid < 0 || ne != 2) return ChainSeamDecline::NotChainTopology;
  // The two ends must have DISTINCT iso values (two parallel planes, not coincident).
  if (std::fabs(g.arcs[ends[0]].isoVal - g.arcs[ends[1]].isoVal) < 1e-6)
    return ChainSeamDecline::NotChainTopology;
  const std::array<SeamArc, 3> src = g.arcs;
  const std::array<std::size_t, 3> srcIdx = g.cutIdx;
  // Canonical order: end0 = smaller iso, end1 = larger iso, middle last.
  int e0 = ends[0], e1 = ends[1];
  if (src[e0].isoVal > src[e1].isoVal) std::swap(e0, e1);
  g.arcs = {src[e0], src[e1], src[mid]};
  g.cutIdx = {srcIdx[e0], srcIdx[e1], srcIdx[mid]};
  return ChainSeamDecline::Ok;
}

/// (3) The two analytic junctions: `Jk = wall(u(end_k), v(middle))`, verified inside
/// the trimmed wall AND on BOTH adjacent cutting planes.
inline ChainSeamDecline computeJunctions(ChainSeamGraph& g, const OperandFace& wall,
                                         const topo::FaceSurface& fs, const topo::Location& loc,
                                         double weldTol) {
  const bool endUConst = g.arcs[0].uConst;         // ends iso-u ⇒ end fixes u, middle fixes v
  const SeamArc& mid = g.arcs[2];
  const UVPolygon wallUV = sgdetail::wallOuterUV(wall.face);
  if (wallUV.size() < 3) return ChainSeamDecline::JunctionUnusable;
  tess::SurfaceEvaluator ev(fs, loc);
  double maxResid = 0.0;
  for (int j = 0; j < 2; ++j) {
    const SeamArc& end = g.arcs[j];
    // end fixes one coordinate (its isoVal), middle fixes the other.
    g.junctionUV[j] = endUConst ? tess::UV{end.isoVal, mid.isoVal}
                                : tess::UV{mid.isoVal, end.isoVal};
    if (!tess::pointInPolygon(wallUV, g.junctionUV[j])) return ChainSeamDecline::JunctionUnusable;
    g.junction3d[j] = ev.value(g.junctionUV[j].u, g.junctionUV[j].v);
    const double re = std::fabs(signedDist(end.tracePlane, g.junction3d[j]));
    const double rm = std::fabs(signedDist(mid.tracePlane, g.junction3d[j]));
    maxResid = std::max({maxResid, re, rm});
  }
  g.junctionPlaneResidual = maxResid;
  return maxResid > weldTol ? ChainSeamDecline::JunctionOffPlane : ChainSeamDecline::Ok;
}

/// A junction WLinePoint (params on the wall + shared 3-D position).
inline ssi::WLinePoint junctionPoint(const tess::UV& uv, const math::Point3& p3) {
  ssi::WLinePoint jp{};
  jp.u1 = uv.u; jp.v1 = uv.v; jp.point = p3;
  return jp;
}

/// (4) Clip each arc at its junction(s) and join into the bent boundary→J1→J2→boundary
/// `chainSeam`. `end0` runs boundary→J1; `middle` runs J1→J2; `end1` runs J2→boundary.
inline ChainSeamDecline joinChain(ChainSeamGraph& g, double band, double diag) {
  const SeamArc& end0 = g.arcs[0];
  const SeamArc& end1 = g.arcs[1];
  const SeamArc& mid = g.arcs[2];
  const bool endUConst = end0.uConst;

  // end0-half: on the middle's interior side, J1-ward last.
  std::vector<ssi::WLinePoint> h0 = sgdetail::interiorSubArc(
      end0.arc, mid.tracePlane, band, end0.uConst,
      endUConst ? g.junctionUV[0].v : g.junctionUV[0].u);
  // end1-half: on the middle's interior side, J2-ward last (then reversed to J2-first).
  std::vector<ssi::WLinePoint> h1 = sgdetail::interiorSubArc(
      end1.arc, mid.tracePlane, band, end1.uConst,
      endUConst ? g.junctionUV[1].v : g.junctionUV[1].u);
  // middle-half: BETWEEN the two ends' planes (interior of both end half-spaces).
  std::vector<ssi::WLinePoint> hm;
  for (const ssi::WLinePoint& p : mid.arc.points)
    if (signedDist(end0.tracePlane, p.point) >= -band &&
        signedDist(end1.tracePlane, p.point) >= -band)
      hm.push_back(p);
  if (h0.size() < 2 || h1.size() < 2 || hm.size() < 2) return ChainSeamDecline::JunctionNotJoined;

  // Order the middle J1→J2 by its varying coordinate matching J1 first.
  auto varyMid = [&](const ssi::WLinePoint& p) { return mid.uConst ? p.v1 : p.u1; };
  const double j1vary = endUConst ? g.junctionUV[0].u : g.junctionUV[0].v;  // middle varies the end-fixed coord
  if (std::fabs(varyMid(hm.front()) - j1vary) > std::fabs(varyMid(hm.back()) - j1vary))
    std::reverse(hm.begin(), hm.end());
  std::reverse(h1.begin(), h1.end());  // interiorSubArc puts J-ward last; want J2-ward first

  // Closure residuals at each junction.
  const double g1a = math::distance(h0.back().point, g.junction3d[0]);
  const double g1b = math::distance(hm.front().point, g.junction3d[0]);
  const double g2a = math::distance(hm.back().point, g.junction3d[1]);
  const double g2b = math::distance(h1.front().point, g.junction3d[1]);
  g.junctionJoinGap = std::max({g1a, g1b, g2a, g2b});
  if (g.junctionJoinGap > 0.05 * diag) return ChainSeamDecline::JunctionNotJoined;

  const ssi::WLinePoint j1 = junctionPoint(g.junctionUV[0], g.junction3d[0]);
  const ssi::WLinePoint j2 = junctionPoint(g.junctionUV[1], g.junction3d[1]);
  for (const ssi::WLinePoint& p : h0) g.chainSeam.points.push_back(p);
  g.chainSeam.points.push_back(j1);
  for (const ssi::WLinePoint& p : hm) g.chainSeam.points.push_back(p);
  g.chainSeam.points.push_back(j2);
  for (const ssi::WLinePoint& p : h1) g.chainSeam.points.push_back(p);
  g.chainSeam.status = ssi::TraceStatus::BoundaryExit;
  return ChainSeamDecline::Ok;
}

}  // namespace sgcdetail

// ─────────────────────────────────────────────────────────────────────────────
// buildChainSeamGraph — assemble the three-arc, two-junction seam graph for the
// edge-straddling box pose, or return a measured `ChainSeamDecline`. `A` is the
// recognised freeform operand; `B` the finite all-planar solid. NEVER emits a partial
// graph. Additive sibling to `buildSeamGraph` (the two-arc, one-junction builder).
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<ChainSeamGraph> buildChainSeamGraph(const FreeformOperand& A,
                                                         const topo::Shape& B,
                                                         ChainSeamDecline* why = nullptr) {
  auto fail = [&](ChainSeamDecline d) -> std::optional<ChainSeamGraph> {
    if (why) *why = d;
    return std::nullopt;
  };
  if (!isAllPlanar(B)) return fail(ChainSeamDecline::NotPlanarB);
  if (A.freeform.size() != 1) return fail(ChainSeamDecline::NotThreeCuttingFaces);

  ChainSeamGraph g;
  g.bPolys = extractPolygons(B);
  if (g.bPolys.empty()) return fail(ChainSeamDecline::NotPlanarB);

  const double diag = std::max(A.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);
  const double isoTol = 1e-6;

  const OperandFace& wall = A.faces[A.freeform.front()];
  const std::vector<math::Point3> poles = isdetail::wallWorldPoles(wall);
  if (poles.empty()) return fail(ChainSeamDecline::SeamUnusable);
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return fail(ChainSeamDecline::SeamUnusable);

  ChainSeamDecline d = sgcdetail::findThreeCuttingFaces(g, poles, A.bbox, band);
  if (d != ChainSeamDecline::Ok) return fail(d);
  d = sgcdetail::traceArcs(g, A, *srf->surface, isoTol);
  if (d != ChainSeamDecline::Ok) return fail(d);
  d = sgcdetail::orderChain(g);
  if (d != ChainSeamDecline::Ok) return fail(d);
  d = sgcdetail::computeJunctions(g, wall, *srf->surface, srf->location, weldTol);
  if (d != ChainSeamDecline::Ok) return fail(d);
  d = sgcdetail::joinChain(g, band, diag);
  if (d != ChainSeamDecline::Ok) return fail(d);

  if (why) *why = ChainSeamDecline::Ok;
  return g;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_CHAIN_H

// SPDX-License-Identifier: Apache-2.0
//
// seam_graph.h — MOAT M2-multiseam: the SEAM-GRAPH builder, the one new verb the
// FIRST *multi-seam* two-operand freeform boolean needs beyond the landed
// single-curved-cut `inter_solid_seam.h`.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// `inter_solid_seam.h` assembles the inter-solid seam for the pose where EXACTLY ONE
// of `B`'s planar faces slices `A`'s freeform wall (one closed seam loop). The next
// blocker is the SEAM GRAPH: a finite cutter `B` positioned so TWO adjacent faces
// slice `A`'s wall, so the inter-solid boundary on the wall is TWO curved arcs that
// MEET at a junction vertex `J`. This header assembles that two-arc, one-junction
// graph for the SIMPLEST reachable pose (design §1):
//
//   * `A` = the bowl-lidded convex-quad prism (the landed B1 operand: one Bézier wall
//     + planar walls + planar bottom).
//   * `B` = a FINITE axis-aligned box straddling ONE corner of `A`, so EXACTLY TWO of
//     its planar faces (`P0`, `P1`, sharing a box vertical edge) each slice `A`'s wall
//     in one transversal arc, and the FOUR other faces contain `A`.
//
// The graph is: `arc0 = P0 ∩ wall` and `arc1 = P1 ∩ wall` (each traced by the LANDED
// `hscdetail::traceWallSeam`, byte-unchanged), meeting at the junction vertex
// `J = (P0 ∩ P1 line) ∩ wall`. `J` is computed ANALYTICALLY (not sampled): the two
// arcs are each iso-parametric on `A`'s Bézier wall (`arc0` at constant `u`, `arc1` at
// constant `v` for the axis-aligned box), so `J`'s wall parameters are `(u(arc0),
// v(arc1))`, and the builder VERIFIES `J` lies on BOTH cutting planes (residual below
// `weldTol`) and inside the trimmed wall — the grounding that the graph closes.
//
// The two arcs are clipped at `J` and joined into ONE bent boundary→J→boundary seam
// (`jointSeam`) whose interior vertex is `J`, the shared boundary of the corner
// sub-face and the L-shaped survivor sub-face.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// B1 `recogniseFreeformSolid`, M1 `traceWallSeam`, and every `isdetail::` /
// `hscdetail::` primitive of the landed `inter_solid_seam.h` / `half_space_cut.h`
// (`wallWorldPoles`, `planeStraddlesWall`, `aabbInsidePlane`, `tracePlaneOf`,
// `signedDist`, `Piece`). Additive sibling — touches none of them, nor B2 `splitFace`,
// nor the landed single-seam `buildInterSolidSeam` / `freeformBooleanTwoOperand` path.
// The `findCuttingFace` `nCut == 1` gate is GENERALISED here (a cutting-face SET of
// exactly two), NOT edited in place.
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate below is a geometry test, never a fudge. If `B` does not present
// EXACTLY two adjacent curved cuts, or `A`'s material is not contained by the other
// faces, or `J` does not lie on both planes inside the wall, or the two arcs do not
// meet at `J`, the builder returns a MEASURED `SeamGraphDecline` (nullopt) — a
// first-class outcome the caller logs before the OCCT fall-through. No tolerance is
// weakened.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_H
#define CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_H

#include "native/boolean/half_space_cut.h"
#include "native/boolean/inter_solid_seam.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/tessellate/uv_triangulate.h"
#include "native/topology/native_topology.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace ssi  = cybercad::native::ssi;

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a closed
/// two-arc, one-junction seam graph is returned.
enum class SeamGraphDecline {
  Ok,
  NotPlanarB,           ///< `B` is not an all-planar solid (the box operand domain)
  NoOverlap,            ///< `A`.bbox ∩ `B`.bbox is empty — no boolean interaction
  NotTwoCuttingFaces,   ///< the number of `B` faces slicing `A`'s wall is not exactly two
  NotContained,         ///< a non-cutting `B` face does not contain `A` (pose guard)
  SeamUnusable,         ///< an arc trace is missing / < 2 nodes / wrong status
  ArcNotIsoParam,       ///< an arc is not iso-parametric on the wall (beyond this pose)
  JunctionUnusable,     ///< `J`'s wall parameters are not inside the trimmed wall
  JunctionOffPlane,     ///< `J` does not lie on both cutting planes within weldTol
  JunctionNotJoined     ///< the two clipped arcs do not meet at `J` within weldTol
};

inline const char* seamGraphDeclineName(SeamGraphDecline d) noexcept {
  switch (d) {
    case SeamGraphDecline::Ok: return "Ok";
    case SeamGraphDecline::NotPlanarB: return "NotPlanarB";
    case SeamGraphDecline::NoOverlap: return "NoOverlap";
    case SeamGraphDecline::NotTwoCuttingFaces: return "NotTwoCuttingFaces";
    case SeamGraphDecline::NotContained: return "NotContained";
    case SeamGraphDecline::SeamUnusable: return "SeamUnusable";
    case SeamGraphDecline::ArcNotIsoParam: return "ArcNotIsoParam";
    case SeamGraphDecline::JunctionUnusable: return "JunctionUnusable";
    case SeamGraphDecline::JunctionOffPlane: return "JunctionOffPlane";
    case SeamGraphDecline::JunctionNotJoined: return "JunctionNotJoined";
  }
  return "?";
}

/// One traced inter-solid arc on the wall, tagged with its iso-parametric structure.
struct SeamArc {
  std::size_t cutIdx = 0;        ///< index into `bPolys` of this arc's cutting face
  math::Plane tracePlane;        ///< `tracePlaneOf(cutting face)`, z = INTO `B`
  ssi::WLine arc;                ///< the LANDED `traceWallSeam` WLine (byte-unchanged)
  bool uConst = false;           ///< true: the arc holds `u` ≈ constant (spans `v`)
  double isoVal = 0.0;           ///< the constant wall parameter (`u` if `uConst`, else `v`)
};

/// The assembled two-arc, one-junction seam graph.
struct SeamGraph {
  std::vector<Polygon> bPolys;              ///< every planar face of `B` (outward normals)
  std::array<std::size_t, 2> cutIdx{0, 0};  ///< indices into `bPolys` of the two cutting faces
  std::array<SeamArc, 2> arcs;              ///< the two traced inter-solid arcs
  tess::UV junctionUV{0.0, 0.0};            ///< `J` in the wall's own (u,v) domain
  math::Point3 junction3d{0.0, 0.0, 0.0};   ///< `J` in world coordinates
  double junctionPlaneResidual = 0.0;       ///< max |signedDist(Pk, J)| — the on-plane grounding
  double junctionJoinGap = 0.0;             ///< the two clipped arcs' closure residual at `J`
  ssi::WLine jointSeam;                     ///< arc0(boundary→J) + J + arc1(J→boundary), bent
};

namespace sgdetail {

using hscdetail::signedDist;

/// Per-arc iso-parametric structure: which wall parameter is (near) constant, and its
/// value. Returns false (→ ArcNotIsoParam) if NEITHER parameter is constant to `tol`.
inline bool arcIsoParam(const ssi::WLine& w, double tol, bool& uConst, double& isoVal) {
  double uMin = 1e30, uMax = -1e30, vMin = 1e30, vMax = -1e30, uSum = 0, vSum = 0;
  for (const ssi::WLinePoint& p : w.points) {
    uMin = std::min(uMin, p.u1); uMax = std::max(uMax, p.u1); uSum += p.u1;
    vMin = std::min(vMin, p.v1); vMax = std::max(vMax, p.v1); vSum += p.v1;
  }
  const double n = static_cast<double>(w.points.size());
  const double uSpan = uMax - uMin, vSpan = vMax - vMin;
  if (uSpan <= tol && uSpan <= vSpan) { uConst = true;  isoVal = uSum / n; return true; }
  if (vSpan <= tol) { uConst = false; isoVal = vSum / n; return true; }
  return false;
}

/// The wall's outer-loop UV polygon (for the in-domain test on `J`).
inline UVPolygon wallOuterUV(const topo::Shape& wallFace) {
  const tess::UVRegion reg = tess::buildRegion(wallFace, 1);
  UVPolygon out;
  if (reg.hasOuter()) out = reg.outer;
  return out;
}

/// The sub-arc of `w` on the interior side of plane `other` (signedDist ≥ −band),
/// ordered so the LAST node is the one nearest the junction parameter `nearIso` in the
/// arc's varying coordinate. `uConst` selects the varying coordinate (v when uConst).
inline std::vector<ssi::WLinePoint> interiorSubArc(const ssi::WLine& w, const math::Plane& other,
                                                   double band, bool uConst, double nearIso) {
  std::vector<ssi::WLinePoint> sel;
  for (const ssi::WLinePoint& p : w.points)
    if (signedDist(other, p.point) >= -band) sel.push_back(p);
  // Order so the varying coordinate DECREASES toward `nearIso` (the junction end last):
  // pick the direction that puts the node farthest from `nearIso` first.
  auto vary = [&](const ssi::WLinePoint& p) { return uConst ? p.v1 : p.u1; };
  if (sel.size() >= 2) {
    const double d0 = std::fabs(vary(sel.front()) - nearIso);
    const double dn = std::fabs(vary(sel.back()) - nearIso);
    if (dn > d0) std::reverse(sel.begin(), sel.end());  // farthest-from-J first, J-ward last
  }
  return sel;
}

/// (1) The cutting-face SET: EXACTLY two faces straddle the wall; the rest contain `A`.
inline SeamGraphDecline findTwoCuttingFaces(SeamGraph& g, const std::vector<math::Point3>& poles,
                                            const Aabb& bbox, double band) {
  std::vector<std::size_t> cut;
  for (std::size_t i = 0; i < g.bPolys.size(); ++i)
    if (isdetail::planeStraddlesWall(g.bPolys[i], poles, band)) cut.push_back(i);
  if (cut.size() != 2) return SeamGraphDecline::NotTwoCuttingFaces;
  for (std::size_t i = 0; i < g.bPolys.size(); ++i)
    if (i != cut[0] && i != cut[1] && !isdetail::aabbInsidePlane(bbox, g.bPolys[i], band))
      return SeamGraphDecline::NotContained;
  g.cutIdx = {cut[0], cut[1]};
  return SeamGraphDecline::Ok;
}

/// (2) Trace both arcs (byte-unchanged M1) + read their iso-parametric structure. The
/// pose needs the two arcs ORTHOGONAL in UV (one u-const, one v-const).
inline SeamGraphDecline traceArcs(SeamGraph& g, const FreeformOperand& A,
                                  const topo::FaceSurface& fs, double isoTol) {
  for (int k = 0; k < 2; ++k) {
    SeamArc& sa = g.arcs[k];
    sa.cutIdx = g.cutIdx[k];
    sa.tracePlane = isdetail::tracePlaneOf(g.bPolys[g.cutIdx[k]]);
    sa.arc = hscdetail::traceWallSeam(A, fs, sa.tracePlane);
    const bool ok = sa.arc.points.size() >= 2 &&
                    (sa.arc.status == ssi::TraceStatus::BoundaryExit ||
                     sa.arc.status == ssi::TraceStatus::Closed);
    if (!ok) return SeamGraphDecline::SeamUnusable;
    if (!arcIsoParam(sa.arc, isoTol, sa.uConst, sa.isoVal)) return SeamGraphDecline::ArcNotIsoParam;
  }
  if (g.arcs[0].uConst == g.arcs[1].uConst) return SeamGraphDecline::ArcNotIsoParam;
  return SeamGraphDecline::Ok;
}

/// (3) The analytic junction `J = (u(uConst arc), v(vConst arc))`, verified inside the
/// trimmed wall AND on BOTH cutting planes (the graph-closes grounding).
inline SeamGraphDecline computeJunction(SeamGraph& g, const OperandFace& wall,
                                        const topo::FaceSurface& fs, const topo::Location& loc,
                                        double weldTol) {
  const int uc = g.arcs[0].uConst ? 0 : 1, vc = 1 - uc;
  g.junctionUV = tess::UV{g.arcs[uc].isoVal, g.arcs[vc].isoVal};
  const UVPolygon wallUV = wallOuterUV(wall.face);
  if (wallUV.size() < 3 || !tess::pointInPolygon(wallUV, g.junctionUV))
    return SeamGraphDecline::JunctionUnusable;
  tess::SurfaceEvaluator ev(fs, loc);
  g.junction3d = ev.value(g.junctionUV.u, g.junctionUV.v);
  const double r0 = std::fabs(signedDist(g.arcs[0].tracePlane, g.junction3d));
  const double r1 = std::fabs(signedDist(g.arcs[1].tracePlane, g.junction3d));
  g.junctionPlaneResidual = std::max(r0, r1);
  return g.junctionPlaneResidual > weldTol ? SeamGraphDecline::JunctionOffPlane : SeamGraphDecline::Ok;
}

/// (4) Clip each arc at `J` on the OTHER plane's interior side and join into the bent
/// boundary→J→boundary seam whose interior vertex is `J`.
inline SeamGraphDecline joinArcs(SeamGraph& g, double band, double diag) {
  ssi::WLinePoint jp{};
  jp.u1 = g.junctionUV.u; jp.v1 = g.junctionUV.v; jp.point = g.junction3d;
  std::vector<ssi::WLinePoint> half0 = interiorSubArc(
      g.arcs[0].arc, g.arcs[1].tracePlane, band, g.arcs[0].uConst,
      g.arcs[0].uConst ? g.junctionUV.v : g.junctionUV.u);
  std::vector<ssi::WLinePoint> half1 = interiorSubArc(
      g.arcs[1].arc, g.arcs[0].tracePlane, band, g.arcs[1].uConst,
      g.arcs[1].uConst ? g.junctionUV.v : g.junctionUV.u);
  if (half0.size() < 2 || half1.size() < 2) return SeamGraphDecline::JunctionNotJoined;
  std::reverse(half1.begin(), half1.end());  // interiorSubArc puts J-ward last; want J-ward first
  const double g0 = math::distance(half0.back().point, g.junction3d);
  const double g1 = math::distance(half1.front().point, g.junction3d);
  g.junctionJoinGap = std::max(g0, g1);
  if (g.junctionJoinGap > 0.05 * diag) return SeamGraphDecline::JunctionNotJoined;

  for (const ssi::WLinePoint& p : half0) g.jointSeam.points.push_back(p);
  g.jointSeam.points.push_back(jp);  // J as the exact shared interior vertex
  for (const ssi::WLinePoint& p : half1) g.jointSeam.points.push_back(p);
  g.jointSeam.status = ssi::TraceStatus::BoundaryExit;
  return SeamGraphDecline::Ok;
}

}  // namespace sgdetail

// ─────────────────────────────────────────────────────────────────────────────
// buildSeamGraph — assemble the two-arc, one-junction seam graph for the corner-box
// pose, or return a measured `SeamGraphDecline`. `A` is the recognised freeform
// operand; `B` the finite all-planar solid. NEVER emits a partial graph.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<SeamGraph> buildSeamGraph(const FreeformOperand& A, const topo::Shape& B,
                                               SeamGraphDecline* why = nullptr) {
  auto fail = [&](SeamGraphDecline d) -> std::optional<SeamGraph> {
    if (why) *why = d;
    return std::nullopt;
  };
  if (!isAllPlanar(B)) return fail(SeamGraphDecline::NotPlanarB);
  if (A.freeform.size() != 1) return fail(SeamGraphDecline::NotTwoCuttingFaces);

  SeamGraph g;
  g.bPolys = extractPolygons(B);
  if (g.bPolys.empty()) return fail(SeamGraphDecline::NotPlanarB);

  const double diag = std::max(A.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);
  const double isoTol = 1e-6;  // UV-domain iso-parametric span tolerance (scale-free)

  const OperandFace& wall = A.faces[A.freeform.front()];
  const std::vector<math::Point3> poles = isdetail::wallWorldPoles(wall);
  if (poles.empty()) return fail(SeamGraphDecline::SeamUnusable);
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return fail(SeamGraphDecline::SeamUnusable);

  // Compose the four graph-building steps; each writes into `g` or returns its blocker.
  SeamGraphDecline d = sgdetail::findTwoCuttingFaces(g, poles, A.bbox, band);
  if (d != SeamGraphDecline::Ok) return fail(d);
  d = sgdetail::traceArcs(g, A, *srf->surface, isoTol);
  if (d != SeamGraphDecline::Ok) return fail(d);
  d = sgdetail::computeJunction(g, wall, *srf->surface, srf->location, weldTol);
  if (d != SeamGraphDecline::Ok) return fail(d);
  d = sgdetail::joinArcs(g, band, diag);
  if (d != SeamGraphDecline::Ok) return fail(d);

  if (why) *why = SeamGraphDecline::Ok;
  return g;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SEAM_GRAPH_H

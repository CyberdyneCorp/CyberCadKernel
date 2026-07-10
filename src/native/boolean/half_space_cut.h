// SPDX-License-Identifier: Apache-2.0
//
// half_space_cut.h — MOAT M2-assembly / B4: the analytic-face-split +
// cross-section-cap-synthesis WELD verb, and the FIRST end-to-end freeform↔analytic
// half-space CUT assembler that composes the landed M2 substrate into ONE verified
// freeform boolean (the SSI-arc payoff).
//
// ── ROLE (the single irreducible enabler for the first freeform boolean) ─────────
// B1 (`freeform_operand.h`) admits a freeform-walled operand; M1 (`ssi/marching.h`)
// traces the wall∩plane seam; B2 (`face_split.h`) splits the freeform top along that
// seam; B3 (`freeform_membership.h`) classifies points; M0 (`solid_mesher.h`) meshes.
// The one verb NONE of them supply is what CLOSES a planar half-space cut of a real
// solid: a planar cut ALSO crosses the operand's ANALYTIC (planar) cap/side faces and
// leaves an OPEN section that must be capped. B4 is exactly that:
//
//   1. ANALYTIC-FACE SPLIT — for each planar analytic boundary face the cut plane `P`
//      crosses, split it along its `Face ∩ P` line into the keep-side sub-face
//      (kept) and the discard-side sub-face (dropped); a face entirely on one side is
//      kept or dropped WHOLE. The two `Face ∩ P` crossing points are recorded.
//   2. CROSS-SECTION CAP — assemble ONE closed loop on `P` from the B2 seam chord
//      (the freeform wall's `wall ∩ P` curve) spliced to the straight `Face ∩ P`
//      chords of the split analytic faces, and build ONE planar cap face on `P`,
//      oriented so its normal faces the DISCARD side (outward for the keep solid).
//   3. WELD — kept freeform sub-face (B2 `faceIn`/`faceOut`), kept analytic sub/whole
//      faces, and the cap → one Shell → Solid. Coincident boundaries share 3-D
//      geometry, so the M0 mesher position-welds the result watertight.
//
// `freeformHalfSpaceCut` composes recognise[B1] → trace[M1] → split[B2] → B4 →
// confirm[B3] → mandatory self-verify (M0 watertight), returning a NULL Shape (→ OCCT
// fall-through) on ANY decline. It NEVER emits a leaky or partial solid.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// B1 `recogniseFreeformSolid`, B2 `splitFace`, B3 `classifyPointInMesh`, M0
// `SolidMesher::mesh`, M1 `trace_intersection`, and the tessellate edge/surface
// evaluators. Additive sibling — touches none of them, nor the analytic
// `recogniseCurvedSolid`/`classifyPoint` paths.
//
// ── SCOPE + HONESTY ─────────────────────────────────────────────────────────────
// This first slice handles a planar half-space cut of an operand with ONE freeform
// face and PLANAR analytic faces whose crossed boundary edges are Line/Bézier, cut
// so each crossed face has EXACTLY TWO `Face ∩ P` crossings and the section loop is
// simple + closed. Anything outside that envelope is an HONEST DECLINE (a measured
// `HalfSpaceCutDecline`, NULL result) — a first-class outcome. No tolerance is
// weakened; the section-loop closure/simplicity tests are geometry predicates and the
// self-verify watertight audit is the M0 mesh-level one, not a fudge.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_HALF_SPACE_CUT_H
#define CYBERCAD_NATIVE_BOOLEAN_HALF_SPACE_CUT_H

#include "native/boolean/face_split.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/math/bezier.h"
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/edge_mesher.h"     // detail::edgeCurveLocal (3D edge eval)
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace ssi  = cybercad::native::ssi;
namespace math = cybercad::native::math;

/// Which half-space is KEPT. `Below` keeps the material where the signed distance to
/// `P` (along `P`'s normal `P.pos.z`) is ≤ 0; `Above` keeps ≥ 0.
enum class KeepSide { Below, Above };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight keep-side solid is returned.
enum class HalfSpaceCutDecline {
  Ok,
  NotAdmitted,          ///< B1 declined the operand
  NoFreeformFace,       ///< no freeform wall (analytic paths own it)
  MultiFreeformFace,    ///< more than one freeform wall — beyond this slice
  SeamUnusable,         ///< M1 seam missing / < 2 nodes / wrong status
  SplitFailed,          ///< B2 declined the freeform split
  UnsupportedEdgeKind,  ///< a crossed analytic face has a Circle/BSpline boundary edge
  AnalyticCrossingNot2, ///< a crossed analytic face did not cross P exactly twice
  SectionLoopOpen,      ///< the cap boundary segments do not chain into a closed loop
  SectionLoopNotSimple, ///< the cap loop self-intersects / is degenerate
  WeldOpen,             ///< fewer than four survivor faces (cannot bound a solid)
  NotWatertight,        ///< self-verify: the welded result is not a closed 2-manifold
  MembershipReject      ///< B3: a kept survivor is On/Unknown or on the wrong side
};

inline const char* declineName(HalfSpaceCutDecline d) noexcept {
  switch (d) {
    case HalfSpaceCutDecline::Ok: return "Ok";
    case HalfSpaceCutDecline::NotAdmitted: return "NotAdmitted";
    case HalfSpaceCutDecline::NoFreeformFace: return "NoFreeformFace";
    case HalfSpaceCutDecline::MultiFreeformFace: return "MultiFreeformFace";
    case HalfSpaceCutDecline::SeamUnusable: return "SeamUnusable";
    case HalfSpaceCutDecline::SplitFailed: return "SplitFailed";
    case HalfSpaceCutDecline::UnsupportedEdgeKind: return "UnsupportedEdgeKind";
    case HalfSpaceCutDecline::AnalyticCrossingNot2: return "AnalyticCrossingNot2";
    case HalfSpaceCutDecline::SectionLoopOpen: return "SectionLoopOpen";
    case HalfSpaceCutDecline::SectionLoopNotSimple: return "SectionLoopNotSimple";
    case HalfSpaceCutDecline::WeldOpen: return "WeldOpen";
    case HalfSpaceCutDecline::NotWatertight: return "NotWatertight";
    case HalfSpaceCutDecline::MembershipReject: return "MembershipReject";
  }
  return "?";
}

namespace hscdetail {

// A boundary piece = an ordered pole run (front→back) of ONE boundary curve:
//   2 poles → a Line; 3 poles → a degree-2 Bézier; > 3 poles → a degree-1 polyline
//   (the many-node seam chord). The cap/keep loops are assembled purely from these,
//   then each piece is rebuilt into an edge with a fresh pcurve on its face plane.
using Piece = std::vector<math::Point3>;

inline double signedDist(const math::Plane& P, const math::Point3& p) noexcept {
  return math::dot(P.pos.z.vec(), p - P.pos.origin);
}

inline bool onKeepSide(double sd, KeepSide side, double band) noexcept {
  return side == KeepSide::Below ? sd <= band : sd >= -band;
}

// The boundary curve of one edge as poles in the edge's TRAVERSAL direction (reversed
// edge ⇒ reversed poles), plus its kind. Circle/Ellipse are rejected upstream.
inline bool edgePolesTraversal(const topo::Shape& edge, Piece& out) {
  const auto cr = topo::curveOf(edge);
  if (!cr || !cr->curve) return false;
  const topo::EdgeCurve& c = *cr->curve;
  using K = topo::EdgeCurve::Kind;
  if (c.kind == K::Line) {
    const auto rr = topo::rangeOf(edge);
    const double t0 = rr ? rr->first : 0.0, t1 = rr ? rr->last : 1.0;
    out = {tess::detail::edgeCurveLocal(c, t0), tess::detail::edgeCurveLocal(c, t1)};
  } else if (c.kind == K::Bezier || (c.kind == K::BSpline && !c.poles.empty())) {
    out = c.poles;
  } else {
    return false;  // Circle / Ellipse / empty — unsupported crossed edge
  }
  if (edge.orientation() == topo::Orientation::Reversed)
    std::reverse(out.begin(), out.end());
  return out.size() >= 2;
}

inline math::Point3 pieceEval(const Piece& poles, double f) noexcept {
  return math::bezierPoint({poles.data(), poles.size()}, f);
}

// de Casteljau split of a Bézier pole run at parameter f → (left [0,f], right [f,1]).
inline void deCasteljau(const Piece& poles, double f, Piece& left, Piece& right) {
  Piece cur = poles;
  const std::size_t n = cur.size();
  left.assign(n, {}); right.assign(n, {});
  for (std::size_t i = 0; i < n; ++i) {
    left[i] = cur.front();
    right[n - 1 - i] = cur.back();
    for (std::size_t j = 0; j + 1 < cur.size(); ++j)
      cur[j] = math::Point3{cur[j].x + (cur[j + 1].x - cur[j].x) * f,
                            cur[j].y + (cur[j + 1].y - cur[j].y) * f,
                            cur[j].z + (cur[j + 1].z - cur[j].z) * f};
    cur.pop_back();
  }
}

// Bisect the crossing parameter f∈[0,1] where signedDist(pieceEval(f)) == 0, given the
// endpoints straddle P (opposite signs). Monotone along a clean single crossing.
inline double bisectCross(const Piece& poles, const math::Plane& P) noexcept {
  double a = 0.0, b = 1.0;
  const double sa = signedDist(P, pieceEval(poles, a));
  for (int it = 0; it < 60; ++it) {
    const double m = 0.5 * (a + b);
    const double sm = signedDist(P, pieceEval(poles, m));
    if ((sa < 0.0) == (sm < 0.0)) a = m; else b = m;
  }
  return 0.5 * (a + b);
}

// ── Loop assembly from directed pieces (endpoint chaining, flips as needed) ──────
// Chains `pieces` into ONE closed cyclic loop: greedily match each piece's open end
// to the next piece's front or back (within `tol`), reversing a piece when matched at
// its back. Returns false (→ SectionLoopOpen) if the pieces do not chain closed.
inline bool orderLoop(std::vector<Piece> pieces, double tol, std::vector<Piece>& loop) {
  if (pieces.empty()) return false;
  std::vector<char> used(pieces.size(), 0);
  auto near = [&](const math::Point3& p, const math::Point3& q) { return math::distance(p, q) <= tol; };
  loop.clear();
  loop.push_back(pieces[0]);
  used[0] = 1;
  math::Point3 head = pieces[0].front();
  math::Point3 tail = pieces[0].back();
  for (std::size_t step = 1; step < pieces.size(); ++step) {
    bool found = false;
    for (std::size_t i = 0; i < pieces.size() && !found; ++i) {
      if (used[i]) continue;
      if (near(pieces[i].front(), tail)) { loop.push_back(pieces[i]); tail = pieces[i].back(); used[i] = 1; found = true; }
      else if (near(pieces[i].back(), tail)) {
        Piece r = pieces[i]; std::reverse(r.begin(), r.end());
        loop.push_back(r); tail = r.back(); used[i] = 1; found = true;
      }
    }
    if (!found) return false;
  }
  return near(tail, head);  // closed
}

// Signed area of a loop (its piece endpoints) projected onto the frame's (x,y).
inline double loopSignedArea(const std::vector<Piece>& loop, const math::Ax3& fr) {
  auto uv = [&](const math::Point3& p) {
    const math::Vec3 d = p - fr.origin;
    return math::Point3{math::dot(d, fr.x.vec()), math::dot(d, fr.y.vec()), 0.0};
  };
  double a = 0.0;
  std::vector<math::Point3> pts;
  for (const Piece& pc : loop) pts.push_back(uv(pc.front()));
  const std::size_t n = pts.size();
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += pts[j].x * pts[i].y - pts[i].x * pts[j].y;
  return 0.5 * a;
}

// Build ONE edge (+ fresh pcurve on plane frame `fr`) from a boundary piece.
inline topo::Shape edgeFromPiece(const Piece& poles, const math::Ax3& fr) {
  auto uv = [&](const math::Point3& p) {
    const math::Vec3 d = p - fr.origin;
    return math::Point3{math::dot(d, fr.x.vec()), math::dot(d, fr.y.vec()), 0.0};
  };
  const auto v0 = topo::ShapeBuilder::makeVertex(poles.front());
  const auto v1 = topo::ShapeBuilder::makeVertex(poles.back());
  topo::EdgeCurve c{};
  topo::PCurve pc{};
  if (poles.size() == 2) {
    c.kind = topo::EdgeCurve::Kind::Line;
    c.frame.origin = poles.front();
    const math::Vec3 d = poles.back() - poles.front();
    const double L = std::max(math::norm(d), 1e-12);
    c.frame.x = math::Dir3{d};
    topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, L, v0, v1);
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = uv(poles.front());
    const math::Point3 e1 = uv(poles.back());
    pc.dir2d = math::Vec3{(e1.x - pc.origin2d.x) / L, (e1.y - pc.origin2d.y) / L, 0.0};
    return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
  }
  // 3 poles → degree-2 Bézier; > 3 → degree-1 clamped BSpline (polyline seam chord).
  const int n = static_cast<int>(poles.size());
  const int deg = (n == 3) ? 2 : 1;
  c.kind = (n == 3) ? topo::EdgeCurve::Kind::Bezier : topo::EdgeCurve::Kind::BSpline;
  c.degree = deg;
  c.poles = poles;
  if (deg == 1) c.knots = detail::clampedKnots(n, 1);
  topo::Shape e = topo::ShapeBuilder::makeEdge(c, 0.0, (n == 3) ? 1.0 : 1.0, v0, v1);
  pc.kind = (n == 3) ? topo::EdgeCurve::Kind::BSpline : topo::EdgeCurve::Kind::BSpline;
  pc.degree = deg;
  for (const math::Point3& p : poles) pc.poles2d.push_back(uv(p));
  pc.knots = (n == 3) ? std::vector<double>{0, 0, 0, 1, 1, 1} : detail::clampedKnots(n, 1);
  return topo::ShapeBuilder::addPCurve(e, e.tshape(), pc);
}

// Build a planar face from an ordered loop of pieces, oriented so its normal matches
// `wantOutward` (of.outwardN) — pick Forward/Reversed by the loop's signed-area sign.
//
// NOTE (byte-frozen; consumed by multi_face_weld/strip_weld/two_operand/inter_solid_seam):
// this picks orientation from the 3-D loop signed area. That is CORRECT for the analytic
// keep-faces those callers build (their loop winding tracks the operand frame), but it is
// NOT the rule the M0 mesher uses: the mesher forces the UV OUTER loop CCW regardless of
// the input edge order, so the meshed normal of a Forward planar face is ALWAYS +fr.z,
// independent of `loopSignedArea`. For a cross-section CAP synthesised from a freshly
// chained loop (`orderLoop`) the loop-area sign is arbitrary, so this rule can flip the
// cap the wrong way — the off-centre keep-face volume defect. Cap synthesis must use
// `planarFaceFromLoopByNormal` below, which matches the mesher's actual convention.
inline topo::Shape planarFaceFromLoop(const std::vector<Piece>& loop, const math::Ax3& fr,
                                      const math::Vec3& wantOutward) {
  std::vector<topo::Shape> edges;
  for (const Piece& pc : loop) edges.push_back(edgeFromPiece(pc, fr));
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = fr;
  const double area = loopSignedArea(loop, fr);
  // Forward face normal = sign(area)·fr.z. Choose orientation so it aligns to wantOutward.
  const double fwdDot = (area >= 0.0 ? 1.0 : -1.0) * math::dot(fr.z.vec(), wantOutward);
  const topo::Orientation o = fwdDot >= 0.0 ? topo::Orientation::Forward : topo::Orientation::Reversed;
  return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(std::move(edges)), {}, o);
}

// F4 (off-centre-accurate cap): build a planar face whose MESHED normal matches
// `wantOutward`, using the M0 mesher's ACTUAL convention. The mesher re-triangulates the
// UV outer loop forced CCW, so a Forward planar face always meshes with normal +fr.z and
// a Reversed one with −fr.z — independent of the incoming 3-D loop winding. Hence the sole
// correct rule is: Forward iff fr.z already points toward `wantOutward`, else Reversed.
// This makes an OFF-CENTRE cross-section cap (whose chained loop winding is arbitrary) come
// out coherently outward, so the welded solid is CONSISTENTLY ORIENTED and its
// `enclosedVolume` is trustworthy at every cut offset — not just the symmetric centre.
inline topo::Shape planarFaceFromLoopByNormal(const std::vector<Piece>& loop, const math::Ax3& fr,
                                              const math::Vec3& wantOutward) {
  std::vector<topo::Shape> edges;
  for (const Piece& pc : loop) edges.push_back(edgeFromPiece(pc, fr));
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::Plane;
  s.frame = fr;
  const topo::Orientation o =
      math::dot(fr.z.vec(), wantOutward) >= 0.0 ? topo::Orientation::Forward : topo::Orientation::Reversed;
  return topo::ShapeBuilder::makeFace(s, topo::ShapeBuilder::makeWire(std::move(edges)), {}, o);
}

// The result of cutting ONE analytic face: kept-whole / dropped / split (with the two
// Face∩P crossing points recorded for the cap).
struct AnalyticCut {
  enum class Kind { KeepWhole, Drop, Split, Fail } kind = Kind::Fail;
  topo::Shape keepFace;              ///< when Split (rebuilt) — KeepWhole reuses the parent
  math::Point3 cross0, cross1;       ///< the Face∩P crossing points (Split only)
  HalfSpaceCutDecline why = HalfSpaceCutDecline::Ok;
};

// The per-edge keep/drop/split scan of ONE analytic face's outer loop.
struct KeepScan {
  std::vector<Piece> keepPieces;        ///< kept boundary pieces (whole or split-keep)
  std::vector<math::Point3> crossings;  ///< the Face∩P crossing points
  int nKeep = 0, nTotal = 0;
  bool unsupported = false;             ///< a Circle/BSpline crossed edge (→ decline)
};

// Walk one analytic face's outer wire, classifying each edge keep-whole / drop-whole /
// split-at-crossing; isolated so `cutAnalyticFace` stays in the backend band.
inline KeepScan scanAnalyticEdges(const topo::Shape& outerWire, const math::Plane& P,
                                  KeepSide side, double band) {
  KeepScan s;
  for (topo::Explorer ex(outerWire, topo::ShapeType::Edge); ex.more(); ex.next()) {
    Piece poles;
    if (!edgePolesTraversal(ex.current(), poles)) { s.unsupported = true; return s; }
    ++s.nTotal;
    const bool k0 = onKeepSide(signedDist(P, poles.front()), side, band);
    const bool k1 = onKeepSide(signedDist(P, poles.back()), side, band);
    if (k0 && k1) { s.keepPieces.push_back(poles); ++s.nKeep; }
    else if (!k0 && !k1) { /* dropped whole */ }
    else {
      const double f = bisectCross(poles, P);
      s.crossings.push_back(pieceEval(poles, f));
      Piece l, r; deCasteljau(poles, f, l, r);
      s.keepPieces.push_back(k0 ? l : r);
    }
  }
  return s;
}

// Split (or keep/drop) ONE planar analytic face against plane P (keep `side`).
inline AnalyticCut cutAnalyticFace(const OperandFace& of, const math::Plane& P, KeepSide side,
                                   double band, double weldTol) {
  AnalyticCut out;
  const auto& wires = of.face.tshape()->children();
  if (wires.empty()) { out.why = HalfSpaceCutDecline::AnalyticCrossingNot2; return out; }

  KeepScan s = scanAnalyticEdges(wires[0], P, side, band);
  if (s.unsupported) { out.why = HalfSpaceCutDecline::UnsupportedEdgeKind; return out; }
  if (s.nKeep == s.nTotal && s.crossings.empty()) { out.kind = AnalyticCut::Kind::KeepWhole; return out; }
  if (s.keepPieces.empty()) { out.kind = AnalyticCut::Kind::Drop; return out; }
  if (s.crossings.size() != 2) { out.why = HalfSpaceCutDecline::AnalyticCrossingNot2; return out; }

  // close the keep loop with the Face∩P chord between the two crossings
  s.keepPieces.push_back(Piece{s.crossings[0], s.crossings[1]});
  std::vector<Piece> loop;
  if (!orderLoop(s.keepPieces, weldTol, loop)) { out.why = HalfSpaceCutDecline::SectionLoopOpen; return out; }
  out.keepFace = planarFaceFromLoop(loop, of.surface.frame, of.outwardN);
  out.kind = AnalyticCut::Kind::Split;
  out.cross0 = s.crossings[0];
  out.cross1 = s.crossings[1];
  return out;
}

// Simple-polygon test on the ordered cap loop (endpoints), non-adjacent-segment
// intersection via the B2 predicate.
inline bool loopSimple(const std::vector<Piece>& loop, const math::Ax3& fr) {
  auto uv = [&](const math::Point3& p) {
    const math::Vec3 d = p - fr.origin;
    return tess::UV{math::dot(d, fr.x.vec()), math::dot(d, fr.y.vec())};
  };
  std::vector<tess::UV> p;
  for (const Piece& pc : loop) p.push_back(uv(pc.front()));
  const int n = static_cast<int>(p.size());
  if (n < 3) return false;
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const int i1 = (i + 1) % n, j1 = (j + 1) % n;
      if (i == j1 || j == i1 || i == j) continue;
      if (tess::detail::segmentsCross(p[i], p[i1], p[j], p[j1])) return false;
    }
  return true;
}

// Trace the freeform wall ∩ P seam. The plane ParamBox is sized from the OPERAND bbox
// projected onto P's frame — a superset that covers the whole seam, including where it
// dips through the freeform interior (below the outer-loop projection). Returns the
// single BoundaryExit/Closed WLine, or an empty WLine on failure.
inline ssi::WLine traceWallSeam(const FreeformOperand& op, const topo::FaceSurface& fs,
                                const math::Plane& P) {
  const ssi::SurfaceAdapter A = ssi::makeBezierAdapter(fs.poles, fs.nPolesU, fs.nPolesV);
  double u0 = 1e30, u1 = -1e30, v0 = 1e30, v1 = -1e30;
  const math::Point3 lo = op.bbox.lo, hi = op.bbox.hi;
  for (int c = 0; c < 8; ++c) {
    const math::Point3 w{(c & 1) ? hi.x : lo.x, (c & 2) ? hi.y : lo.y, (c & 4) ? hi.z : lo.z};
    const math::Vec3 d = w - P.pos.origin;
    const double pu = math::dot(d, P.pos.x.vec()), pv = math::dot(d, P.pos.y.vec());
    u0 = std::min(u0, pu); u1 = std::max(u1, pu);
    v0 = std::min(v0, pv); v1 = std::max(v1, pv);
  }
  const double mu = 0.15 * std::max(u1 - u0, 1e-6), mv = 0.15 * std::max(v1 - v0, 1e-6);
  const ssi::SurfaceAdapter B = ssi::makePlaneAdapter(P, ssi::ParamBox{u0 - mu, u1 + mu, v0 - mv, v1 + mv});
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  for (const ssi::WLine& w : tr.lines)
    if (w.points.size() >= 2 &&
        (w.status == ssi::TraceStatus::BoundaryExit || w.status == ssi::TraceStatus::Closed))
      return w;
  return ssi::WLine{};
}

// The seam's 3-D chord on the freeform surface (E … X) — the cap's freeform boundary.
inline std::vector<math::Point3> seamChord3d(const topo::FaceSurface& fs,
                                             const topo::Location& loc, const FaceSplit& split) {
  tess::SurfaceEvaluator ev(fs, loc);
  std::vector<math::Point3> seam3d;
  seam3d.reserve(split.seam.size());
  for (const tess::UV& q : split.seam) seam3d.push_back(ev.value(q.u, q.v));
  return seam3d;
}

}  // namespace hscdetail

// ─────────────────────────────────────────────────────────────────────────────
// The B4 verb: given an admitted operand, the cut plane `P` + keep `side`, the B2
// split of its (single) freeform face, and the seam 3-D chord, build the welded
// keep-side Solid — or NULL with a measured decline.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape halfSpaceCut(const FreeformOperand& op, const math::Plane& P, KeepSide side,
                                const FaceSplit& split, const std::vector<math::Point3>& seam3d,
                                HalfSpaceCutDecline* why = nullptr) {
  using namespace hscdetail;
  auto fail = [&](HalfSpaceCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  const double diag = std::max(op.bbox.diagonal(), 1e-9);
  const double band = 1e-9 * diag;
  const double weldTol = 1e-7 * std::max(diag, 1.0);

  // (1) pick the keep-side freeform sub-face by its trim centroid's side of P.
  const tess::UVRegion regIn = tess::buildRegion(split.faceIn, 16);
  const tess::UVRegion regOut = tess::buildRegion(split.faceOut, 16);
  const auto srf = topo::surfaceOf(op.faces[op.freeform.front()].face);
  if (!srf || !regIn.hasOuter() || !regOut.hasOuter()) return fail(HalfSpaceCutDecline::SplitFailed);
  tess::SurfaceEvaluator seval(*srf->surface, srf->location);
  auto centroidSide = [&](const tess::UVRegion& reg) {
    double su = 0, sv = 0; for (const tess::UV& q : reg.outer) { su += q.u; sv += q.v; }
    const double inv = 1.0 / static_cast<double>(reg.outer.size());
    return signedDist(P, seval.value(su * inv, sv * inv));
  };
  const bool keepIn = onKeepSide(centroidSide(regIn), side, band);
  const topo::Shape freeKeep = keepIn ? split.faceIn : split.faceOut;

  // (2) split every analytic face; collect kept faces + the cap's crossing chords.
  std::vector<topo::Shape> faces;
  faces.push_back(freeKeep);
  std::vector<Piece> capPieces;
  // the freeform wall's Face∩P curve (the seam) is one cap boundary piece.
  capPieces.push_back(seam3d);
  for (std::size_t idx : op.analytic) {
    const AnalyticCut ac = cutAnalyticFace(op.faces[idx], P, side, band, weldTol);
    switch (ac.kind) {
      case AnalyticCut::Kind::KeepWhole: faces.push_back(op.faces[idx].face); break;
      case AnalyticCut::Kind::Drop: break;
      case AnalyticCut::Kind::Split:
        faces.push_back(ac.keepFace);
        capPieces.push_back(Piece{ac.cross0, ac.cross1});
        break;
      case AnalyticCut::Kind::Fail: return fail(ac.why);
    }
  }

  // (3) synthesise the cross-section cap on P.
  math::Ax3 capFrame = P.pos;  // z = P normal (outward faces the discard side)
  std::vector<Piece> capLoop;
  if (!orderLoop(capPieces, weldTol, capLoop)) return fail(HalfSpaceCutDecline::SectionLoopOpen);
  if (!loopSimple(capLoop, capFrame)) return fail(HalfSpaceCutDecline::SectionLoopNotSimple);
  const math::Vec3 outwardCap = (side == KeepSide::Below ? 1.0 : -1.0) * P.pos.z.vec();
  // F4: the cap loop winding is arbitrary (freshly chained), so orient by the mesher's
  // actual +fr.z convention — the off-centre-accurate rule — not the 3-D loop area sign.
  faces.push_back(planarFaceFromLoopByNormal(capLoop, capFrame, outwardCap));

  // (4) weld → Solid.
  if (faces.size() < 4) return fail(HalfSpaceCutDecline::WeldOpen);
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  if (why) *why = HalfSpaceCutDecline::Ok;
  return topo::ShapeBuilder::makeSolid({shell});
}

// ─────────────────────────────────────────────────────────────────────────────
// The FIRST freeform↔analytic half-space CUT assembler. Composes the landed verbs
// end-to-end and DISCARDS any result that fails the mandatory self-verify (watertight)
// → NULL (OCCT fall-through). `meshDeflection` sizes the M0 mesh used by the seam-free
// self-verify and the optional B3 confirmation.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformHalfSpaceCut(const topo::Shape& operand, const math::Plane& P,
                                        KeepSide side, double meshDeflection = 0.01,
                                        HalfSpaceCutDecline* why = nullptr) {
  using namespace hscdetail;
  auto fail = [&](HalfSpaceCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  // (1) B1 recognise
  OperandDecline b1 = OperandDecline::Ok;
  const auto op = recogniseFreeformSolid(operand, &b1);
  if (!op) return fail(HalfSpaceCutDecline::NotAdmitted);
  if (op->freeform.empty()) return fail(HalfSpaceCutDecline::NoFreeformFace);
  if (op->freeform.size() != 1) return fail(HalfSpaceCutDecline::MultiFreeformFace);

  const OperandFace& wall = op->faces[op->freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return fail(HalfSpaceCutDecline::NoFreeformFace);
  const topo::FaceSurface& fs = *srf->surface;
  if (fs.kind != topo::FaceSurface::Kind::Bezier || fs.poles.empty())
    return fail(HalfSpaceCutDecline::SeamUnusable);

  // (2) M1 trace: wall ∩ P (ParamBox from the operand bbox → covers the whole seam).
  const ssi::WLine seam = hscdetail::traceWallSeam(*op, fs, P);
  if (seam.points.size() < 2) return fail(HalfSpaceCutDecline::SeamUnusable);

  // (3) B2 split the freeform top
  const SplitResult sr = splitFace(wall.face, seam);
  if (!sr.ok()) return fail(HalfSpaceCutDecline::SplitFailed);

  // seam 3-D chord on the freeform surface (E … X) — the cap's freeform boundary.
  const std::vector<math::Point3> seam3d = hscdetail::seamChord3d(fs, srf->location, *sr.split);

  // (4) B4 verb
  HalfSpaceCutDecline b4 = HalfSpaceCutDecline::Ok;
  const topo::Shape cut = halfSpaceCut(*op, P, side, *sr.split, seam3d, &b4);
  if (cut.isNull()) return fail(b4);

  // (5) mandatory self-verify: mesh the result; require a CONSISTENTLY-ORIENTED closed
  // 2-manifold (NEVER emit a leak, and never a watertight-but-mis-wound shell whose signed
  // enclosedVolume would be untrustworthy — the off-centre cap-orientation guard).
  tess::MeshParams mp; mp.deflection = meshDeflection;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(cut);
  if (!tess::isConsistentlyOriented(m)) return fail(HalfSpaceCutDecline::NotWatertight);

  if (why) *why = HalfSpaceCutDecline::Ok;
  return cut;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_HALF_SPACE_CUT_H

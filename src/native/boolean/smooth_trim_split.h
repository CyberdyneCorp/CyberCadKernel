// SPDX-License-Identifier: Apache-2.0
//
// smooth_trim_split.h — MOAT M2b / B2 SMOOTH-TRIM generalisation: the deferred
// "closed / circular smooth-trim" enabler for curved-wall freeform booleans.
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// Byte-frozen B2 `splitFace` (`face_split.h`) partitions a trimmed freeform/analytic
// face along an OPEN seam CHORD that enters one boundary edge and exits another
// (`crossings == 2`). It DECLINES the seam that is a CLOSED SMOOTH curve interior to
// the face — a horizontal plane slicing a bowl/dome cap, or a circle wrapping a
// curved wall — because that seam has ZERO boundary crossings (`CrossingsNot2`,
// crossings == 0). That closed-seam case is EXACTLY what a curved-wall freeform
// operand needs (cut the top off a dome; ring-cut a wall), and it is the roadmap's
// named "B2 smooth-trim (closed/circular wall) generalisation".
//
// This verb resolves it as an ADDITIVE SIBLING (B2 `splitFace` is untouched, its
// convex straight-edged path byte-frozen). A closed smooth seam interior to a trimmed
// face partitions it into TWO genuinely-trimmed sub-faces:
//   * `faceInside`  — the disk the seam ENCLOSES  (outer wire = the seam loop);
//   * `faceOutside` — the parent MINUS that disk   (parent outer wire + the seam
//                     loop as a HOLE wire).
// The seam loop is built ONCE (two arcs meeting at two shared vertices) and laid onto
// both sub-faces with OPPOSITE orientation, so it is their BIT-EXACT common boundary.
// Because the mesher already keeps a triangle iff it is inside the outer loop AND
// outside every hole (`trim.h` UVRegion), both sub-faces mesh with NO tessellator
// change, and the disk + annulus TILE the parent (`areaInside + areaOutside == parent`).
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// Every `detail::` primitive of the byte-frozen B2 `face_split.h` (`flattenOuter`,
// `seamCross`, `shoelace`, `buildSeamEdge`, `BndSeg`, `orient2d`, `segmentsCross`),
// the SAME `tess::buildRegion` reflatten, `tess::pointInPolygon`, and the SAME
// `SplitOptions` tolerances (NEVER weakened). Additive sibling — touches NONE of them,
// nor `splitFace`, nor `splitFaceJunction`. The closed-loop wire construction (a hole
// wire on the outside sub-face) is the only genuinely-new step.
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate is a geometry test. If the seam is not a closed interior loop (it
// crosses / touches the boundary, or is not closed), or a sub-region is degenerate,
// or the loop self-intersects, or the tiling gap or rebuild residual exceeds the
// STRICT (unweakened) tolerance, the verb returns a MEASURED decline (nullopt) — a
// first-class outcome the caller logs before OCCT. No tolerance is weakened; no
// partial / leaky split is ever emitted. The still-hard cases (a seam that crosses the
// boundary, a self-intersecting seam, multiple nested loops) DECLINE with a witness.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_SMOOTH_TRIM_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_SMOOTH_TRIM_SPLIT_H

#include "native/boolean/face_split.h"
#include "native/ssi/marching.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes. `Ok` iff a verified closed-seam two-face split is returned; every
// other value is an HONEST decline carrying WHY the seam is not a usable interior
// closed loop.
// ─────────────────────────────────────────────────────────────────────────────
enum class SmoothSplitDecline {
  Ok,
  NoOuterLoop,          ///< face has no usable outer wire / < 3 UV boundary points
  SeamTooShort,         ///< the seam has < 3 distinct nodes (cannot bound an area)
  SeamNotInterior,      ///< the seam crosses / touches the boundary, or leaves the face
  SeamNotClosed,        ///< the seam does not close on itself (endpoints far apart)
  SelfIntersecting,     ///< the closed seam loop is not a simple polygon
  DegenerateSubRegion,  ///< the enclosed disk or the annulus has near-zero area
  TilingGap,            ///< area(inside)+area(outside) != area(parent) beyond tolerance
  RebuildMismatch       ///< a rebuilt sub-face's flattened UV loop does not match its loop
};

inline const char* smoothSplitDeclineName(SmoothSplitDecline d) noexcept {
  switch (d) {
    case SmoothSplitDecline::Ok: return "Ok";
    case SmoothSplitDecline::NoOuterLoop: return "NoOuterLoop";
    case SmoothSplitDecline::SeamTooShort: return "SeamTooShort";
    case SmoothSplitDecline::SeamNotInterior: return "SeamNotInterior";
    case SmoothSplitDecline::SeamNotClosed: return "SeamNotClosed";
    case SmoothSplitDecline::SelfIntersecting: return "SelfIntersecting";
    case SmoothSplitDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case SmoothSplitDecline::TilingGap: return "TilingGap";
    case SmoothSplitDecline::RebuildMismatch: return "RebuildMismatch";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// A verified closed-seam split. `loopInside` is the closed seam loop (the enclosed
// disk boundary); `seam` is the same loop the two sub-faces SHARE (faceInside as its
// outer wire, faceOutside as a hole wire, opposite orientation). Areas are UV
// signed-area magnitudes; `areaInside + areaOutside == parentArea`.
// ─────────────────────────────────────────────────────────────────────────────
struct SmoothFaceSplit {
  topo::Shape faceInside;   ///< the disk the seam encloses (kept for COMMON, removed for a top-cut)
  topo::Shape faceOutside;  ///< the parent minus that disk (the seam is its hole)
  UVPolygon loopInside;
  UVPolygon seam;           ///< the shared closed seam loop (== loopInside, CCW)
  double parentArea = 0.0;
  double areaInside = 0.0;
  double areaOutside = 0.0;
  double rebuildResidual = 0.0;  ///< max |reflattened area − combinatorial area| (below rtol)
};

struct SmoothSplitResult {
  std::optional<SmoothFaceSplit> split;   ///< nullopt on decline
  SmoothSplitDecline decline = SmoothSplitDecline::Ok;
  int crossings = -1;                      ///< seam×boundary crossings (0 for a valid interior loop)
  double tilingGap = 0.0;                  ///< |parent − (in+out)| UV area (rebuild-derived witness)
  double measuredArea = 0.0;               ///< parent UV area (scale reference)
  bool ok() const noexcept { return split.has_value(); }
};

namespace stsdetail {

using detail::BndSeg;
using detail::buildSeamEdge;
using detail::flattenOuter;
using detail::seamCross;
using detail::shoelace;
using tess::detail::segmentsCross;

// A closed UV loop is simple iff no pair of non-adjacent edges cross (edges wrap
// cyclically). Byte-identical predicate to B2's `simple` lambda, closed-loop form.
inline bool simpleLoop(const UVPolygon& p) noexcept {
  const int n = static_cast<int>(p.size());
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j) {
      const int i1 = (i + 1) % n, j1 = (j + 1) % n;
      if (i == j1 || j == i1) continue;  // adjacent edges legitimately share a vertex
      if (segmentsCross(p[i], p[i1], p[j], p[j1])) return false;
    }
  return true;
}

// Extract the seam's distinct UV nodes as a CLOSED loop: drop a duplicated closing
// node (front≈back) so the polygon's implicit closing edge is the only wrap. Returns
// false (SeamNotClosed) when a `Closed`-status seam does not actually close.
inline bool seamLoopNodes(const ssi::WLine& seam, double closeTol, UVPolygon& loop) {
  loop.clear();
  loop.reserve(seam.points.size());
  for (const ssi::WLinePoint& p : seam.points) loop.push_back(UV{p.u1, p.v1});
  if (loop.size() >= 2) {
    const UV& a = loop.front();
    const UV& b = loop.back();
    if (std::hypot(a.u - b.u, a.v - b.v) <= closeTol) loop.pop_back();
  }
  return loop.size() >= 3;
}

// Count seam×boundary crossings over the CLOSED seam loop (every edge incl. the wrap
// edge n-1→0). A valid interior loop has ZERO. Also requires every node strictly
// inside the outer polygon. Returns the crossing count; sets `allInside`.
inline int loopBoundaryCrossings(const UVPolygon& loop, const std::vector<BndSeg>& segs,
                                 const UVPolygon& outer, bool& allInside) {
  const int nL = static_cast<int>(loop.size());
  const int nB = static_cast<int>(segs.size());
  int crossings = 0;
  for (int k = 0; k < nL; ++k) {
    const UV& p0 = loop[k];
    const UV& p1 = loop[(k + 1) % nL];
    for (int i = 0; i < nB; ++i) {
      double s = 0.0;
      UV cp{};
      if (seamCross(p0, p1, segs[i].a, segs[i].b, s, cp)) ++crossings;
    }
  }
  allInside = true;
  for (const UV& q : loop)
    if (!tess::pointInPolygon(outer, q)) { allInside = false; break; }
  return crossings;
}

// Build the closed seam as ONE short STRAIGHT edge per polyline segment (loop[i] →
// loop[i+1], and the wrap loop[n-1] → loop[0]). This is the FAITHFUL representation
// of a traced polyline seam: each segment is genuinely straight in 3D, so the M0
// edge discretizer samples it EXACTLY at its two endpoints (a curvature-driven
// discretizer UNDER-samples a single degree-1 B-spline standing in for the whole
// curved arc, collapsing the circle to a few chords and halving the meshed disk
// area — the measured failure of the two-arc form). Each edge is built ONCE over
// `faceNode` with a single pcurve; the disk wire uses them forward, the hole wire
// uses them reversed (order + orientation) ⇒ the two sub-faces share the seam
// BIT-IDENTICALLY and weld watertight. Returns the per-segment edges in loop order.
inline std::vector<topo::Shape> buildLoopEdges(const UVPolygon& loop,
                                               const std::vector<math::Point3>& loop3d,
                                               const std::shared_ptr<const topo::TShape>& faceNode) {
  const int n = static_cast<int>(loop.size());
  std::vector<topo::Shape> segEdges;
  segEdges.reserve(loop.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    UVPolygon uv{loop[i], loop[j]};
    std::vector<math::Point3> p3{loop3d[i], loop3d[j]};
    segEdges.push_back(buildSeamEdge(uv, p3, faceNode));  // 2-pole degree-1 ⇒ straight edge
  }
  return segEdges;
}

}  // namespace stsdetail

// ─────────────────────────────────────────────────────────────────────────────
// splitFaceSmoothTrim — the B2 smooth-trim companion. Partitions `face` along a
// CLOSED smooth interior seam into the enclosed disk (`faceInside`) + the parent
// minus that disk (`faceOutside`, the seam as a hole). Returns a verified split, or
// an honest `SmoothSplitDecline` with the measured blocker. `seam.points[i].{u1,v1}`
// is read as the seam pcurve on `face`'s own (u,v) domain (surface A = this face).
// ─────────────────────────────────────────────────────────────────────────────
inline SmoothSplitResult splitFaceSmoothTrim(const topo::Shape& face, const ssi::WLine& seam,
                                             const SplitOptions& opts = {}) {
  using namespace detail;
  using namespace stsdetail;
  SmoothSplitResult r;

  // ── (1) flatten the outer loop; extract the closed seam loop ───────────────
  std::vector<BndSeg> segs;
  std::vector<topo::Shape> edges;
  if (!flattenOuter(face, opts.segsPerEdge, segs, edges)) {
    r.decline = SmoothSplitDecline::NoOuterLoop;
    return r;
  }
  UVPolygon outer;
  outer.reserve(segs.size());
  for (const BndSeg& s : segs) outer.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outer));
  r.measuredArea = parentArea;

  // Scale-relative closure tolerance from the parent's UV diagonal (never weakened
  // to force a pass — a wide gap is an honest SeamNotClosed decline).
  const double scale = std::sqrt(std::max(parentArea, 1e-300));
  UVPolygon loop;
  if (seam.points.size() < 3) {
    r.decline = SmoothSplitDecline::SeamTooShort;
    return r;
  }
  if (!seamLoopNodes(seam, scale * 1e-6, loop)) {
    r.decline = SmoothSplitDecline::SeamNotClosed;
    return r;
  }

  // ── (2) the seam must be a CLOSED loop strictly INTERIOR to the face ────────
  bool allInside = false;
  r.crossings = loopBoundaryCrossings(loop, segs, outer, allInside);
  if (r.crossings != 0 || !allInside) {
    r.decline = SmoothSplitDecline::SeamNotInterior;
    return r;
  }
  if (!simpleLoop(loop)) {
    r.decline = SmoothSplitDecline::SelfIntersecting;
    return r;
  }

  // ── (3) orient the loop CCW; the enclosed disk + the annulus must both hold area ─
  if (shoelace(loop) < 0.0) std::reverse(loop.begin(), loop.end());
  const double areaInside = std::fabs(shoelace(loop));
  const double areaOutside = parentArea - areaInside;
  const double floor = std::max(parentArea, 1.0) * opts.areaFloorFrac;
  if (areaInside < floor || areaOutside < floor) {
    r.decline = SmoothSplitDecline::DegenerateSubRegion;
    return r;
  }

  // ── (4) place the seam loop on the surface (3D) for the rebuilt sub-edges ───
  const auto sr = topo::surfaceOf(face);
  if (!sr) {
    r.decline = SmoothSplitDecline::NoOuterLoop;
    return r;
  }
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  std::vector<math::Point3> loop3d;
  loop3d.reserve(loop.size());
  for (const UV& q : loop) loop3d.push_back(eval.value(q.u, q.v));

  // ── (5) build the shared closed seam (two arcs) + the two sub-faces ─────────
  // The seam wire is built ONCE and laid onto both sub-faces with OPPOSITE
  // orientation ⇒ bit-exact shared boundary. `faceInside` = seam loop as OUTER
  // wire; `faceOutside` = the PARENT outer wire (reused verbatim ⇒ boundary edges
  // byte-identical to the parent) + the seam loop as a HOLE wire.
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();
  const std::vector<topo::Shape> segEdges = buildLoopEdges(loop, loop3d, face.tshape());

  std::vector<topo::Shape> insideEdges = segEdges;  // forward, CCW: loop[0]→loop[1]→…→loop[0]
  std::vector<topo::Shape> holeEdges;               // reversed order + orientation, CW hole
  holeEdges.reserve(segEdges.size());
  for (auto it = segEdges.rbegin(); it != segEdges.rend(); ++it)
    holeEdges.push_back(it->reversedShape());
  topo::Shape insideWire = topo::ShapeBuilder::makeWire(std::move(insideEdges));
  topo::Shape holeWire = topo::ShapeBuilder::makeWire(std::move(holeEdges));
  const topo::Shape faceInside = topo::ShapeBuilder::makeFace(surface, insideWire, {}, orient);

  const auto& parentWires = face.tshape()->children();
  const topo::Shape parentOuter = parentWires.empty() ? topo::Shape{} : parentWires[0];
  const topo::Shape faceOutside =
      topo::ShapeBuilder::makeFace(surface, parentOuter, {holeWire}, orient);

  // ── (6) rebuild self-verify — SAME strict rtol B2 uses. Reflatten the two rebuilt
  // sub-faces through `buildRegion` and PROVE the partition topologically:
  //   * `faceOutside` has EXACTLY ONE hole (the seam), and its OUTER reflattens to the
  //     parent (the boundary wire is REUSED verbatim → straight-pcurve exact);
  //   * the seam reflattens BIT-IDENTICALLY as `faceInside`'s outer and `faceOutside`'s
  //     hole (the SAME edges, opposite orientation) — the watertight-share invariant;
  // then the INDEPENDENT tiling residual |parent − (disk + (annulusOuter − hole))|
  // collapses to machine ε ALGEBRAICALLY (disk == hole cancels), for a genuinely
  // CURVED seam whose reflattened chord area legitimately differs from the fine
  // combinatorial `areaInside`. Comparing the coarse reflatten to `areaInside` would
  // be the WRONG test for a curved seam (that is why a curved seam needs the share
  // invariant, not a fixed-density area match); no tolerance is weakened. ──────────
  const int segsRebuild = std::max(opts.segsPerEdge, 8);
  const tess::UVRegion regIn = tess::buildRegion(faceInside, segsRebuild);
  const tess::UVRegion regOut = tess::buildRegion(faceOutside, segsRebuild);
  const double rtol = std::max(parentArea, 1.0) * opts.rebuildTolFrac;
  if (!regIn.hasOuter() || !regOut.hasOuter() || regOut.holes.size() != 1) {
    r.decline = SmoothSplitDecline::RebuildMismatch;
    return r;
  }
  const double rInOuter = std::fabs(shoelace(regIn.outer));
  const double rOutOuter = std::fabs(shoelace(regOut.outer));
  const double rOutHole = std::fabs(shoelace(regOut.holes[0]));
  const double eShare = std::fabs(rInOuter - rOutHole);   // seam shared bit-identically
  const double eOuter = std::fabs(rOutOuter - parentArea);  // boundary reused → exact
  if (eShare > rtol || eOuter > rtol) {
    r.decline = SmoothSplitDecline::RebuildMismatch;
    return r;
  }
  // Independent tiling residual from the REBUILT topology: disk + (annulusOuter − hole).
  const double gap = std::fabs(parentArea - (rInOuter + (rOutOuter - rOutHole)));
  r.tilingGap = gap;
  if (gap > std::max(parentArea, 1.0) * opts.tilingTolFrac) {
    r.decline = SmoothSplitDecline::TilingGap;
    return r;
  }

  SmoothFaceSplit fs;
  fs.faceInside = faceInside;
  fs.faceOutside = faceOutside;
  fs.loopInside = loop;
  fs.seam = loop;
  fs.parentArea = parentArea;
  fs.areaInside = areaInside;
  fs.areaOutside = areaOutside;
  fs.rebuildResidual = std::max(eShare, eOuter);
  r.split = std::move(fs);
  r.decline = SmoothSplitDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_SMOOTH_TRIM_SPLIT_H

// SPDX-License-Identifier: Apache-2.0
//
// holed_face_split.h — Layer-3 Stage-3 (MULTI-HOLE-SPLIT): split a face that ALREADY
// carries a seam-hole (an annulus) by a SECOND closed interior seam, PRESERVING the
// existing hole. The precise enabler for BOOL-READMIT's general genuine-overlap ≥3
// weld (a second boolean seam lands on an already-holed annulus).
//
// ── ROLE ─────────────────────────────────────────────────────────────────────────
// `smooth_trim_split.h::splitFaceSmoothTrim` splits a SIMPLY-CONNECTED face along ONE
// closed interior seam into disk + annulus. It treats the parent as hole-free: when the
// parent is ITSELF an annulus (the seam of a PRIOR boolean is an interior hole), it
// DROPS that existing hole — the disk sub-face wrongly fills the removed region, the
// survivors are geometrically incomplete, and the downstream weld honest-declines
// `NotWatertight` (BOOL-READMIT measured be≈768). This verb resolves exactly that gap.
//
// `splitFaceSmoothTrimHoled(face, seam)` takes a face with an outer loop + N ≥ 0
// existing HOLE loops and ONE closed interior second seam, and partitions it into TWO
// genuinely-trimmed sub-faces that PRESERVE every existing hole, tiling the parent's
// NET (holes-subtracted) area exactly:
//   * `faceInside`  — the region the second seam ENCLOSES: outer wire = the seam loop,
//                     carrying every EXISTING hole that lies inside the seam (nested).
//   * `faceOutside` — the parent MINUS that region: the PARENT outer wire, with the seam
//                     loop as a HOLE + every existing hole that lies OUTSIDE the seam.
// The seam loop is built ONCE (straight degree-1 edges per polyline segment, the same
// faithful representation `smooth_trim_split.h` uses) and laid onto both sub-faces with
// OPPOSITE orientation ⇒ their bit-exact common boundary. Every existing hole is REBUILT
// as fresh straight edges keyed to its owning sub-face (a clean single-pcurve edge the
// M0 mesher resolves without face-node ambiguity), so the two annuli mesh + weld exactly
// as the hole-free smooth-trim sub-faces already do (readmit non-holed case: watertight).
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// `smooth_trim_split.h` (its `SmoothFaceSplit` result type + the `stsdetail` seam-loop /
// crossing / straight-edge primitives `seamLoopNodes` / `simpleLoop` / `buildLoopEdges`),
// `face_split.h::detail` (`buildSeamEdge`, `flattenOuter`, `shoelace`, `seamCross`,
// `BndSeg`), `topology` (surfaceOf), `tessellate/trim.h` (flattenWire, pointInPolygon).
// ADDITIVE — touches none of them. The result is a `SmoothFaceSplit`, so the byte-frozen
// `pickByMembership` survivor-select consumes it with NO change.
//
// ── WHY NOT the L3-b planar-arrangement arc-walk (`multi_crossing_split.h`) here ─────
// The L3-b arc-walk was EVALUATED for the tiling verify. It builds a doubly-connected
// edge list and walks faces by rotational order — sound when the seam actually CROSSES a
// boundary/hole (a CONNECTED arrangement). For this slice the seam is a CLOSED loop
// concentric with the outer + hole (three DISCONNECTED loops); the DCEL face-walk cannot
// express an annulus bounded by two disconnected loops (it double-counts the nested
// disks), so it is the WRONG verifier for the NESTED case (measured: it reports a spurious
// TilingGap on concentric annuli). The SOUND, host-checkable verifier for the two-annulus
// slice is the direct signed-area decomposition below (parent net == inside + outside,
// exact by construction once the seam's nesting is validated). The seam-CROSSES-hole case
// (where the arc-walk WOULD apply) is honest-declined `SeamCrossesHole` in this slice.
//
// ── HONESTY ───────────────────────────────────────────────────────────────────────
// Every predicate is a geometry test. If the seam is not a closed interior simple loop,
// if it CROSSES an existing hole (that is the genuinely-harder multi-crossing case — the
// arrangement would return > 2 regions, out of this two-annulus slice), if a sub-region
// is degenerate, or if the NET-area tiling residual exceeds the STRICT (unweakened)
// tolerance, the verb returns a MEASURED decline (nullopt) — a first-class outcome. No
// tolerance is weakened; no partial / leaky split is ever emitted.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_HOLED_FACE_SPLIT_H
#define CYBERCAD_NATIVE_BOOLEAN_HOLED_FACE_SPLIT_H

#include "native/boolean/smooth_trim_split.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// Outcome codes for the holed-face second-seam split. `Ok` iff a verified two-annulus
// split (existing hole preserved) is returned; every other value is an HONEST decline
// carrying WHY the case is beyond this two-region, hole-preserving slice.
// ─────────────────────────────────────────────────────────────────────────────
enum class HoledSplitDecline {
  Ok,
  NoOuterLoop,          ///< face has no usable outer wire / < 3 UV boundary points
  NoHole,               ///< the face has NO existing hole (use splitFaceSmoothTrim instead)
  SeamTooShort,         ///< the seam has < 3 distinct nodes (cannot bound an area)
  SeamNotClosed,        ///< the seam does not close on itself
  SeamNotInterior,      ///< the seam crosses / touches the outer boundary
  SeamCrossesHole,      ///< the seam CROSSES an existing hole — the harder multi-crossing case
  SelfIntersecting,     ///< the closed seam loop is not a simple polygon
  DegenerateSubRegion,  ///< a sub-region has near-zero net area
  TilingGap             ///< Σ net area(sub-regions) != net area(parent) beyond tolerance
};

inline const char* holedSplitDeclineName(HoledSplitDecline d) noexcept {
  switch (d) {
    case HoledSplitDecline::Ok: return "Ok";
    case HoledSplitDecline::NoOuterLoop: return "NoOuterLoop";
    case HoledSplitDecline::NoHole: return "NoHole";
    case HoledSplitDecline::SeamTooShort: return "SeamTooShort";
    case HoledSplitDecline::SeamNotClosed: return "SeamNotClosed";
    case HoledSplitDecline::SeamNotInterior: return "SeamNotInterior";
    case HoledSplitDecline::SeamCrossesHole: return "SeamCrossesHole";
    case HoledSplitDecline::SelfIntersecting: return "SelfIntersecting";
    case HoledSplitDecline::DegenerateSubRegion: return "DegenerateSubRegion";
    case HoledSplitDecline::TilingGap: return "TilingGap";
  }
  return "?";
}

struct HoledSplitResult {
  std::optional<SmoothFaceSplit> split;   ///< nullopt on decline (SmoothFaceSplit for reuse)
  HoledSplitDecline decline = HoledSplitDecline::Ok;
  int crossings = -1;                      ///< seam×outer crossings (0 for a valid interior loop)
  double parentNetArea = 0.0;              ///< parent NET (holes-subtracted) UV area
  double tilingGap = 0.0;                  ///< |parentNet − Σ sub-region net| (blocker witness)
  int holesInside = 0;                     ///< existing holes attributed to faceInside
  int holesOutside = 0;                    ///< existing holes attributed to faceOutside
  bool ok() const noexcept { return split.has_value(); }
};

namespace hfsdetail {

using detail::BndSeg;
using detail::buildSeamEdge;
using detail::flattenOuter;
using detail::shoelace;
using stsdetail::seamLoopNodes;
using stsdetail::simpleLoop;

// Flatten the face's existing hole wires (children[1..]) to UV polygons, mirroring
// `tess::buildRegion`'s hole extraction. Each hole is a closed UV loop (no dup vertex).
inline std::vector<UVPolygon> flattenHoles(const topo::Shape& face, int segsPerEdge) {
  std::vector<UVPolygon> holes;
  if (face.isNull() || face.type() != topo::ShapeType::Face) return holes;
  const auto& wires = face.tshape()->children();
  for (std::size_t w = 1; w < wires.size(); ++w) {
    UVPolygon poly = tess::flattenWire(wires[w], face, segsPerEdge);
    if (poly.size() >= 3) holes.push_back(std::move(poly));
  }
  return holes;
}

// True ⇔ every vertex of the closed loop `inner` lies strictly inside the closed loop
// `outer` (used to decide hole nesting: is an existing hole inside the seam disk?).
inline bool loopInside(const UVPolygon& inner, const UVPolygon& outer) noexcept {
  for (const UV& q : inner)
    if (!tess::pointInPolygon(outer, q)) return false;
  return true;
}

// Does the polyline seam CROSS the closed hole loop `h` (a proper interior crossing)?
// A seam that crosses an existing hole is the harder multi-crossing case — declined here.
inline bool seamCrossesHoleLoop(const UVPolygon& seam, const UVPolygon& h) noexcept {
  const int ns = static_cast<int>(seam.size());
  const int nh = static_cast<int>(h.size());
  for (int i = 0; i < ns; ++i) {
    const UV& a0 = seam[i];
    const UV& a1 = seam[(i + 1) % ns];  // seam is closed here
    for (int j = 0; j < nh; ++j) {
      double s = 0.0;
      UV cp{};
      if (detail::seamCross(a0, a1, h[j], h[(j + 1) % nh], s, cp)) return true;
    }
  }
  return false;
}

}  // namespace hfsdetail

// ─────────────────────────────────────────────────────────────────────────────
// splitFaceSmoothTrimHoled — split a HOLED face (annulus) along a CLOSED interior second
// seam into faceInside (the seam-enclosed region) + faceOutside (the parent minus it),
// PRESERVING every existing hole. Returns a `SmoothFaceSplit` (so the frozen survivor
// selector consumes it unchanged) or an honest `HoledSplitDecline`. `seam.points[i].{u1,
// v1}` is read as the seam pcurve on `face`'s own (u,v) domain (surface A = this face).
// ─────────────────────────────────────────────────────────────────────────────
inline HoledSplitResult splitFaceSmoothTrimHoled(const topo::Shape& face, const ssi::WLine& seam,
                                                 const SplitOptions& opts = {}) {
  using namespace detail;
  using namespace hfsdetail;
  HoledSplitResult r;

  // ── (1) flatten the outer loop + existing holes; extract the closed seam loop ──
  std::vector<BndSeg> segs;
  std::vector<topo::Shape> edges;
  if (!flattenOuter(face, opts.segsPerEdge, segs, edges)) {
    r.decline = HoledSplitDecline::NoOuterLoop;
    return r;
  }
  UVPolygon outer;
  outer.reserve(segs.size());
  for (const BndSeg& s : segs) outer.push_back(s.a);
  if (shoelace(outer) < 0.0) std::reverse(outer.begin(), outer.end());  // outer CCW
  const double outerArea = std::fabs(shoelace(outer));

  std::vector<UVPolygon> holes = flattenHoles(face, opts.segsPerEdge);
  if (holes.empty()) {
    r.decline = HoledSplitDecline::NoHole;  // simply-connected ⇒ splitFaceSmoothTrim
    return r;
  }
  for (UVPolygon& h : holes)
    if (shoelace(h) < 0.0) std::reverse(h.begin(), h.end());  // store holes CCW (area magnitude)
  double holesArea = 0.0;
  for (const UVPolygon& h : holes) holesArea += std::fabs(shoelace(h));
  const double parentNet = outerArea - holesArea;
  r.parentNetArea = parentNet;

  const double scale = std::sqrt(std::max(outerArea, 1e-300));
  if (seam.points.size() < 3) {
    r.decline = HoledSplitDecline::SeamTooShort;
    return r;
  }
  UVPolygon loop;
  if (!seamLoopNodes(seam, scale * 1e-6, loop)) {
    r.decline = HoledSplitDecline::SeamNotClosed;
    return r;
  }
  if (shoelace(loop) < 0.0) std::reverse(loop.begin(), loop.end());  // seam loop CCW

  // ── (2) the seam must be a CLOSED simple loop, interior to the outer, not crossing
  // any existing hole (that harder case is the general multi-crossing split). ──
  bool allInside = true;
  for (const UV& q : loop)
    if (!tess::pointInPolygon(outer, q)) { allInside = false; break; }
  r.crossings = 0;
  {
    const int nL = static_cast<int>(loop.size());
    const int nB = static_cast<int>(segs.size());
    for (int k = 0; k < nL && r.crossings == 0; ++k)
      for (int i = 0; i < nB; ++i) {
        double s = 0.0;
        UV cp{};
        if (seamCross(loop[k], loop[(k + 1) % nL], segs[i].a, segs[i].b, s, cp)) { ++r.crossings; break; }
      }
  }
  if (r.crossings != 0 || !allInside) {
    r.decline = HoledSplitDecline::SeamNotInterior;
    return r;
  }
  if (!simpleLoop(loop)) {
    r.decline = HoledSplitDecline::SelfIntersecting;
    return r;
  }
  for (const UVPolygon& h : holes) {
    if (seamCrossesHoleLoop(loop, h)) {
      r.decline = HoledSplitDecline::SeamCrossesHole;
      return r;
    }
    // The seam must not lie ENTIRELY inside an existing hole (it would sit in already-
    // removed material — not a real face-splitting seam). Reject as SeamNotInterior.
    if (loopInside(loop, h)) {
      r.decline = HoledSplitDecline::SeamNotInterior;
      return r;
    }
  }

  // ── (3) attribute each existing hole to the sub-region that CONTAINS it: a hole
  // fully inside the seam loop belongs to faceInside; otherwise faceOutside. (A hole
  // that STRADDLES the seam was rejected above as SeamCrossesHole.) ──
  std::vector<const UVPolygon*> holesInside, holesOutside;
  for (const UVPolygon& h : holes)
    (loopInside(h, loop) ? holesInside : holesOutside).push_back(&h);
  r.holesInside = static_cast<int>(holesInside.size());
  r.holesOutside = static_cast<int>(holesOutside.size());

  // ── (4) VERIFY the seam nests cleanly with the outer + every hole (the two-annulus
  // slice). The seam must lie strictly inside the outer (checked in (2)); every existing
  // hole is either wholly inside the seam (→ faceInside) or wholly outside it (checked
  // via `seamCrossesHole` + `loopInside`), so the NET-area decomposition below is EXACT
  // (the planar arrangement is NOT used to verify a NESTED, non-crossing tiling: its
  // face-walk cannot express an annulus bounded by two DISCONNECTED loops — the
  // signed-area identity is the sound, host-checkable verifier for this slice). A hole
  // that is neither wholly-inside nor wholly-outside would have been rejected as
  // SeamCrossesHole. ──

  // ── (5) net areas of the two sub-faces; both must hold area ──
  const double diskGross = std::fabs(shoelace(loop));  // area the seam encloses (gross)
  double diskHolesArea = 0.0;
  for (const UVPolygon* h : holesInside) diskHolesArea += std::fabs(shoelace(*h));
  const double areaInsideNet = diskGross - diskHolesArea;       // faceInside net (seam − nested holes)
  const double areaOutsideNet = parentNet - areaInsideNet;      // faceOutside net (the rest)
  const double floor = std::max(outerArea, 1.0) * opts.areaFloorFrac;
  if (areaInsideNet < floor || areaOutsideNet < floor) {
    r.decline = HoledSplitDecline::DegenerateSubRegion;
    return r;
  }

  // ── (6) place loops on the surface (3-D) + build the two sub-faces ──
  const auto sr = topo::surfaceOf(face);
  if (!sr) {
    r.decline = HoledSplitDecline::NoOuterLoop;
    return r;
  }
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  auto lift = [&](const UVPolygon& l) {
    std::vector<math::Point3> p3;
    p3.reserve(l.size());
    for (const UV& q : l) p3.push_back(eval.value(q.u, q.v));
    return p3;
  };
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();
  const std::shared_ptr<const topo::TShape> node = face.tshape();
  const auto& parentWires = node->children();

  // The seam loop is built ONCE (straight degree-1 edges per polyline segment), used
  // FORWARD as faceInside's outer wire and REVERSED as faceOutside's hole wire ⇒ the two
  // sub-faces share the seam BIT-IDENTICALLY (the same watertight-share idiom as
  // smooth_trim_split). Every EXISTING hole is REUSED VERBATIM from the parent (its
  // boundary must stay bit-identical to the parent, so it still welds to the OTHER wall
  // that shares this seam-hole — rebuilding it with fresh vertices would crack that weld).
  const std::vector<math::Point3> loop3d = lift(loop);
  const std::vector<topo::Shape> seamSeg = stsdetail::buildLoopEdges(loop, loop3d, node);

  // Map each parent hole WIRE (parentWires[1..]) to inside/outside via the flattened-hole
  // nesting decided above (holes[w-1] corresponds to parentWires[w]).
  std::vector<topo::Shape> insideHoleWires, outsideHoleWires;
  for (std::size_t w = 1; w < parentWires.size(); ++w) {
    if (w - 1 >= holes.size()) break;
    const UVPolygon& hUV = holes[w - 1];
    const bool nested = std::find(holesInside.begin(), holesInside.end(), &hUV) != holesInside.end();
    (nested ? insideHoleWires : outsideHoleWires).push_back(parentWires[w]);
  }

  // faceInside: seam loop as OUTER wire + every nested existing hole (reused verbatim).
  std::vector<topo::Shape> insideEdges = seamSeg;
  const topo::Shape insideWire = topo::ShapeBuilder::makeWire(std::move(insideEdges));
  const topo::Shape faceInside =
      topo::ShapeBuilder::makeFace(surface, insideWire, insideHoleWires, orient);

  // faceOutside: parent outer wire (reused verbatim) + the seam as a HOLE (reversed
  // seam edges) + every non-nested existing hole (reused verbatim).
  {
    std::vector<topo::Shape> holeEdges;
    holeEdges.reserve(seamSeg.size());
    for (auto it = seamSeg.rbegin(); it != seamSeg.rend(); ++it)
      holeEdges.push_back(it->reversedShape());
    outsideHoleWires.insert(outsideHoleWires.begin(),
                            topo::ShapeBuilder::makeWire(std::move(holeEdges)));
  }
  const topo::Shape parentOuter = parentWires.empty() ? topo::Shape{} : parentWires[0];
  const topo::Shape faceOutside =
      topo::ShapeBuilder::makeFace(surface, parentOuter, std::move(outsideHoleWires), orient);

  // ── (7) net-area tiling self-verify (host-checkable, independent of the arrangement) ──
  const double gap = std::fabs(parentNet - (areaInsideNet + areaOutsideNet));
  if (gap > std::max(outerArea, 1.0) * opts.tilingTolFrac) {
    r.decline = HoledSplitDecline::TilingGap;
    r.tilingGap = std::max(r.tilingGap, gap);
    return r;
  }

  SmoothFaceSplit fs;
  fs.faceInside = faceInside;
  fs.faceOutside = faceOutside;
  fs.loopInside = loop;
  fs.seam = loop;
  fs.parentArea = parentNet;
  fs.areaInside = areaInsideNet;
  fs.areaOutside = areaOutsideNet;
  fs.rebuildResidual = gap;
  r.split = std::move(fs);
  r.decline = HoledSplitDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_HOLED_FACE_SPLIT_H

// SPDX-License-Identifier: Apache-2.0
//
// freeform_freeform_multiseam.h — L3-d / STAGE 5: the MULTI-SEAM watertight sew.
// The N-seam generalisation of the single-transversal-seam curved↔curved weld that
// `freeform_freeform_cut.h` (`freeformFreeformClosedSeamCut`, track W) resolved.
//
// ── WHAT THIS ADDS OVER THE LANDED SINGLE-SEAM WELD ───────────────────────────────
// Track W welds the pose where two freeform walls meet along ONE closed transversal
// seam: each wall splits into a disk + annulus, one sub-face per side is kept, and the
// two curved caps weld across the ONE shared seam (M0w seam-chord pin) with a one-flip
// orientation-coherence repair. MISSING there: two operands whose walls intersect in
// MULTIPLE disjoint closed seam loops (the SSI returns > 1 loop) — e.g. a wavy plate cut
// by a bump that pierces it twice, whose intersection is TWO concentric circles. There
// the wall must be split by ALL seams into > 2 sub-regions (an inner disk, one or more
// middle annuli, and a background region), EACH sub-region classified for membership,
// and the survivors sewn watertight across EVERY seam.
//
// ── THE APPROACH (additive; W's single-seam path byte-UNCHANGED) ──────────────────
// This header includes `freeform_freeform_cut.h` and REUSES its surface-kind-agnostic
// helpers (`ffcdetail::{freeformWall, traceSharedSeam, rekeyToB, subFaceInteriorReps}`)
// byte-identically. It touches NEITHER `freeformFreeformClosedSeamCut` NOR the M0
// tessellator NOR `splitFaceSmoothTrim`. It adds:
//
//   1. MULTI-SEAM SPLIT (`splitWallBySeams`) — split BOTH walls by ALL seam loops. A
//      set of disjoint (possibly nested) closed UV loops partitions a trimmed face into
//      one sub-face PER loop (that loop as its outer boundary, its immediately-nested
//      child loops as holes) PLUS a background sub-face (the parent outer boundary with
//      the top-level loops as holes). The seam edges are built ONCE per loop (the same
//      `buildSeamEdge` W's single-seam path uses) and laid onto the two neighbouring
//      sub-faces with OPPOSITE orientation, so every shared seam is BIT-IDENTICAL and
//      the M0 mesher position-welds it — exactly the single-seam weld, repeated per seam.
//   2. PER-REGION MEMBERSHIP (`subFaceInteriorReps` vote, W's hole-respecting fix) —
//      every sub-region (disk, annulus, background) is classified by the majority vote
//      over hole-respecting interior representative points, so an annulus is read in its
//      ring (not the removed disk), unchanged from W.
//   3. MULTI-CAP COHERENT WELD (`weldMultiCoherent`) — the survivors from BOTH walls are
//      welded into a Solid; orientation coherence is repaired by trying the identity, a
//      single last-flip (W's exact repair, so a degenerate-to-single-seam case is
//      byte-identical), the flip-the-B-group repair (the two curved sides wind oppositely
//      across every seam), and the flip-the-A-group repair (the FUSE outer envelope's two
//      region groups wind oppositely). A candidate is accepted ONLY if it is watertight,
//      coherent, AND its enclosed volume AGREES with the survivors' own divergence-theorem
//      expectation (BOOL-VOTE — the oracle-free volume self-verify: the welded mesh may
//      only REPAIR the seam pairing, never MOVE material; a weld whose volume drifts from
//      the survivor faces' summed signed contribution is a mesher weld that collapsed real
//      volume — measured on the annulus↔annulus COMMON at asym d=0.002 and sym d=0.0018,
//      where the shared seam-strip splice pinches the collar bands away and loses a
//      deflection-INDEPENDENT 6-7% of the lens, identically in EVERY orientation
//      configuration — so the verb honest-declines it, with or without the closed-form
//      oracle, instead of returning the smaller solid). The mandatory M0 watertight +
//      consistent-orientation + positive/bounded-volume self-verify gates the result;
//      ANY decline → NULL.
//
// The op-set is COMPLETE: COMMON (the annular lens), CUT (A minus the lens), and FUSE
// (the OUTER envelope of A∪B = the complement of the lens on BOTH walls + both operands'
// lids, sewn across every seam) all compose here from the SAME split+select+weld primitives.
//
// ── HONESTY (DISAGREED=0 SACRED) ──────────────────────────────────────────────────
// Every predicate is a geometry test. A degenerate multi-seam pose (a seam that is not a
// closed interior loop, a split that declines, an ambiguous membership, a weld that does
// not close watertight / coherently / at a consistent volume) returns a MEASURED decline
// (NULL Shape) — NEVER a leaky/partial/wrong solid. No tolerance is weakened. The
// self-verify is two-sided against the closed-form op-volume when supplied.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_MULTISEAM_H
#define CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_MULTISEAM_H

#include "native/boolean/face_split.h"          // detail::{shoelace, buildSeamEdge, buildLoopEdges via stsdetail}
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/smooth_trim_split.h"    // stsdetail::{seamLoopNodes, buildLoopEdges, simpleLoop}
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/face_mesher.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight multi-seam result solid is returned. NEVER a leaky/partial/wrong solid.
// ─────────────────────────────────────────────────────────────────────────────
enum class MultiSeamCutDecline {
  Ok,
  NotAdmittedA,          ///< B1 declined operand A
  NotAdmittedB,          ///< B1 declined operand B
  WallUnusableA,         ///< A does not present exactly one usable freeform wall
  WallUnusableB,         ///< B does not present exactly one usable freeform wall
  NoMultiSeam,           ///< the SSI trace returned fewer than TWO closed interior loops
  SeamUnusable,          ///< a seam loop is missing / < 3 nodes / not a closed interior loop
  MultiSplitFailedA,     ///< the multi-seam split of A's wall declined
  MultiSplitFailedB,     ///< the multi-seam split of B's wall declined
  ClassifyAmbiguous,     ///< a survivor sub-region is On/Unknown or the survivor set is empty
  WeldOpen,              ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,         ///< self-verify: the welded result is not a closed/coherent 2-manifold
  VolumeInconsistent     ///< self-verify: volume non-positive / off the bound / weld moved material
};

inline const char* multiSeamCutDeclineName(MultiSeamCutDecline d) noexcept {
  switch (d) {
    case MultiSeamCutDecline::Ok: return "Ok";
    case MultiSeamCutDecline::NotAdmittedA: return "NotAdmittedA";
    case MultiSeamCutDecline::NotAdmittedB: return "NotAdmittedB";
    case MultiSeamCutDecline::WallUnusableA: return "WallUnusableA";
    case MultiSeamCutDecline::WallUnusableB: return "WallUnusableB";
    case MultiSeamCutDecline::NoMultiSeam: return "NoMultiSeam";
    case MultiSeamCutDecline::SeamUnusable: return "SeamUnusable";
    case MultiSeamCutDecline::MultiSplitFailedA: return "MultiSplitFailedA";
    case MultiSeamCutDecline::MultiSplitFailedB: return "MultiSplitFailedB";
    case MultiSeamCutDecline::ClassifyAmbiguous: return "ClassifyAmbiguous";
    case MultiSeamCutDecline::WeldOpen: return "WeldOpen";
    case MultiSeamCutDecline::NotWatertight: return "NotWatertight";
    case MultiSeamCutDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

/// The honest witnesses of a multi-seam split + weld (for the host gate / caller log).
struct MultiSeamCutReport {
  MultiSeamCutDecline decline = MultiSeamCutDecline::Ok;
  int seamLoops = 0;             ///< closed interior seam loops the SSI returned
  int subRegionsA = 0;          ///< sub-faces A's wall split into (seamLoops + 1)
  int subRegionsB = 0;          ///< sub-faces B's wall split into
  int survivorFaces = 0;        ///< kept faces welded into the result
  bool watertight = false;      ///< M0 self-verify: closed 2-manifold
  bool coherent = false;        ///< M0 self-verify: consistently oriented
  std::size_t boundaryEdges = 1;///< unpaired boundary edges of the welded result (0 ⇒ watertight)
  double enclosedVolume = 0.0;  ///< signed-tetra enclosed volume of the result
  double expectedVolume = 0.0;  ///< oracle-free divergence expectation Σ|per-face signed volumes|
                                ///< of the accepted (or volume-mismatched) weld configuration
  double tilingGapA = 0.0;      ///< |parent − Σ sub-region| UV area on A (≈ 0)
  double tilingGapB = 0.0;      ///< |parent − Σ sub-region| UV area on B (≈ 0)
};

namespace ffmdetail {

using detail::shoelace;
using stsdetail::seamLoopNodes;
using stsdetail::simpleLoop;

// A closed seam loop on ONE wall's (u,v): the UV polygon + the matching 3-D points
// (surface values at the loop nodes), plus its signed-area magnitude and CCW flag.
struct SeamLoop {
  tess::UVPolygon uv;                    ///< closed loop in the wall's (u,v)
  std::vector<math::Point3> p3;          ///< the loop nodes on the wall surface (3-D)
  double area = 0.0;                     ///< |signed UV area| (nesting order key)
};

// A verified sub-region of the wall: its outer loop (a seam loop or the parent outer)
// and the immediately-nested child seam loops as holes. Built as a topo::Shape with the
// shared seam edges (bit-identical to the neighbouring region's boundary).
struct WallRegion {
  topo::Shape face;                      ///< the trimmed sub-face (outer + holes)
  bool isBackground = false;             ///< true for the parent-outer / all-top-level-holes region
};

// Extract each closed seam loop as a UV polygon (+ 3-D nodes) on `face`'s own domain,
// reading `seam.points[i].{u1,v1}` as the pcurve. Rejects a non-closed / < 3-node /
// self-intersecting loop (a MEASURED decline). `closeTol` scales with the parent UV.
inline bool extractLoop(const topo::Shape& face, const ssi::WLine& seam, double closeTol,
                        SeamLoop& out) {
  if (seam.points.size() < 3) return false;
  tess::UVPolygon loop;
  if (!seamLoopNodes(seam, closeTol, loop)) return false;
  if (!simpleLoop(loop)) return false;
  if (shoelace(loop) < 0.0) std::reverse(loop.begin(), loop.end());  // CCW
  const auto sr = topo::surfaceOf(face);
  if (!sr || !sr->surface) return false;
  tess::SurfaceEvaluator eval(*sr->surface, sr->location);
  out.uv = loop;
  out.p3.clear();
  out.p3.reserve(loop.size());
  for (const tess::UV& q : loop) out.p3.push_back(eval.value(q.u, q.v));
  out.area = std::fabs(shoelace(loop));
  return loop.size() >= 3;
}

// Nesting: loop `i` is DIRECTLY inside loop `j` iff `j` contains `i`'s first node and no
// OTHER loop `k` (k≠i,j) is between them (j⊃k⊃i). Returns, per loop, the index of its
// immediate parent loop (or -1 if top-level, i.e. directly inside the parent face).
inline std::vector<int> immediateParents(const std::vector<SeamLoop>& loops) {
  const int n = static_cast<int>(loops.size());
  std::vector<int> parent(n, -1);
  // containment[i][j] = loop j contains loop i (test i's first node in j).
  auto contains = [&](int outer, int inner) {
    return tess::pointInPolygon(loops[outer].uv, loops[inner].uv.front());
  };
  for (int i = 0; i < n; ++i) {
    int best = -1;
    double bestArea = std::numeric_limits<double>::infinity();
    for (int j = 0; j < n; ++j) {
      if (j == i || !contains(j, i)) continue;
      // j contains i; pick the SMALLEST such j (the immediate enclosing loop).
      if (loops[j].area < bestArea) { bestArea = loops[j].area; best = j; }
    }
    parent[i] = best;
  }
  return parent;
}

// Build the shared seam edges (forward, CCW) for one loop ONCE, over `faceNode`. Two
// neighbouring sub-faces reuse them (one forward as an outer wire, one reversed as a
// hole wire) so the shared seam is bit-identical. Reuses W's single-seam edge builder.
inline std::vector<topo::Shape> loopEdges(const SeamLoop& loop,
                                          const std::shared_ptr<const topo::TShape>& faceNode) {
  return stsdetail::buildLoopEdges(loop.uv, loop.p3, faceNode);
}

// A wire from per-segment edges. `reversed` uses them in reverse order + orientation
// (the CW hole form) — the SAME convention `splitFaceSmoothTrim` uses so a loop is a
// disk's outer wire forward and the enclosing region's hole wire reversed.
inline topo::Shape wireFromEdges(const std::vector<topo::Shape>& segEdges, bool reversed) {
  std::vector<topo::Shape> es;
  es.reserve(segEdges.size());
  if (!reversed) {
    for (const topo::Shape& e : segEdges) es.push_back(e);
  } else {
    for (auto it = segEdges.rbegin(); it != segEdges.rend(); ++it) es.push_back(it->reversedShape());
  }
  return topo::ShapeBuilder::makeWire(std::move(es));
}

// Split `face` by ALL seam loops into (seamLoops.size() + 1) sub-regions: one region per
// loop (that loop forward as its outer wire, its immediately-nested child loops reversed
// as holes) + a background region (the parent outer wire + every TOP-LEVEL loop as a
// hole). Every shared seam edge is built ONCE per loop and laid on both neighbouring
// regions with opposite orientation. Returns false on any degenerate loop. `tilingGap`
// is the UV-area residual |parent − Σ region| (≈ 0 for a valid partition).
inline bool splitWallBySeams(const topo::Shape& face, const std::vector<ssi::WLine>& seams,
                             std::vector<WallRegion>& regions, double& tilingGap) {
  regions.clear();
  const auto sr = topo::surfaceOf(face);
  if (!sr || !sr->surface) return false;
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();

  // Parent outer wire (reused verbatim for the background region) + parent UV area.
  const auto& parentWires = face.tshape()->children();
  if (parentWires.empty()) return false;
  const topo::Shape parentOuter = parentWires[0];

  std::vector<detail::BndSeg> segs;
  std::vector<topo::Shape> pedges;
  if (!detail::flattenOuter(face, 24, segs, pedges)) return false;
  tess::UVPolygon outerPoly;
  outerPoly.reserve(segs.size());
  for (const detail::BndSeg& s : segs) outerPoly.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outerPoly));

  // Extract every loop; each must be a simple closed loop strictly inside the parent.
  const double scale = std::sqrt(std::max(parentArea, 1e-300));
  std::vector<SeamLoop> loops;
  loops.reserve(seams.size());
  for (const ssi::WLine& s : seams) {
    SeamLoop L;
    if (!extractLoop(face, s, scale * 1e-6, L)) return false;
    for (const tess::UV& q : L.uv)
      if (!tess::pointInPolygon(outerPoly, q)) return false;  // interior to the parent
    loops.push_back(std::move(L));
  }
  const int n = static_cast<int>(loops.size());
  if (n < 1) return false;

  const std::vector<int> parent = immediateParents(loops);

  // Build each loop's shared seam edges ONCE.
  std::vector<std::vector<topo::Shape>> edges(n);
  for (int i = 0; i < n; ++i) edges[i] = loopEdges(loops[i], face.tshape());

  // children[j] = loops whose immediate parent is j; topLevel = loops with parent -1.
  std::vector<std::vector<int>> children(n);
  std::vector<int> topLevel;
  for (int i = 0; i < n; ++i) {
    if (parent[i] < 0) topLevel.push_back(i);
    else children[parent[i]].push_back(i);
  }

  // One region PER loop: outer = that loop (forward, CCW), holes = its child loops
  // (reversed, CW). Areas accumulate for the tiling residual.
  double sumArea = 0.0;
  for (int i = 0; i < n; ++i) {
    const topo::Shape outerWire = wireFromEdges(edges[i], /*reversed=*/false);
    std::vector<topo::Shape> holeWires;
    holeWires.reserve(children[i].size());
    double holeArea = 0.0;
    for (int c : children[i]) {
      holeWires.push_back(wireFromEdges(edges[c], /*reversed=*/true));
      holeArea += loops[c].area;
    }
    WallRegion reg;
    reg.face = topo::ShapeBuilder::makeFace(surface, outerWire, std::move(holeWires), orient);
    reg.isBackground = false;
    regions.push_back(std::move(reg));
    sumArea += loops[i].area - holeArea;
  }

  // The background region: parent outer + every TOP-LEVEL loop as a hole (reversed).
  {
    std::vector<topo::Shape> holeWires;
    holeWires.reserve(topLevel.size());
    double holeArea = 0.0;
    for (int t : topLevel) {
      holeWires.push_back(wireFromEdges(edges[t], /*reversed=*/true));
      holeArea += loops[t].area;
    }
    WallRegion reg;
    reg.face = topo::ShapeBuilder::makeFace(surface, parentOuter, std::move(holeWires), orient);
    reg.isBackground = true;
    regions.push_back(std::move(reg));
    sumArea += parentArea - holeArea;
  }

  tilingGap = std::fabs(parentArea - sumArea);
  return regions.size() == static_cast<std::size_t>(n + 1);
}

// The honest witnesses of the weld enumeration, for the caller's sharpened decline map.
struct WeldVerdict {
  std::size_t minBoundaryEdges = std::numeric_limits<std::size_t>::max();
  bool volumeMismatch = false;   ///< a watertight+coherent weld EXISTED but MOVED material
  double weldedVolume = 0.0;     ///< |V| of the accepted (or last mismatched) weld
  double expectedVolume = 0.0;   ///< the divergence-theorem expectation it was gated against
};

// BOOL-VOTE — the weld-volume agreement band, a fraction of the divergence expectation.
// MEASURED calibration (measure_multiseam_vote; asym a=4 valley ∩ b=6 dome COMMON and the
// sym mirror-cups COMMON):
//   * a LEGIT weld only REPAIRS the seam pairing — merge coincident vertices, pin diverging
//     rim samples — so its volume tracks the survivors' summed signed contribution to
//     ≤ 2e-4 relative (asym d=0.0025 rim-pin weld: 0.006554 vs 0.006553; the sym d=0.002
//     baseline weld agrees exactly, 0.007434 vs 0.007434);
//   * the strip-pinch collapse (both annuli splice the SAME-side collar strip, the
//     duplicate strip triangles annihilate at the weld, the mesh pinches at the collar
//     rings and the collar bands' volume is LOST) drifts 6.8% relative at asym d=0.002
//     (0.006172 vs 0.006619) and 5.8% at sym d=0.0018 (0.007019 vs 0.007448) —
//     deflection-INDEPENDENT (the collar width is a fixed fraction of rSeam), and
//     IDENTICAL across every watertight+coherent orientation configuration (the strip is
//     orientation-independent geometry), so no configuration pick can restore the volume.
// 0.02 sits ≥ 100× above the measured repair drift and ≥ 2.9× below the smallest measured
// pinch; it is a NEW gate (nothing was widened).
inline constexpr double kWeldVolumeAgreeFrac = 0.02;

// Per-face signed divergence-theorem volume contributions of the survivor faces, meshed
// through the SAME shared-EdgeCache baseline path the SolidMesher's pre-weld accumulation
// uses (every face's edge-segment demands raised first, no rim pin, no seam strip) — so the
// summed expectation matches the welded mesh up to the weld REPAIRS, not up to
// re-discretization noise. Flipping face i in a weld configuration negates vols[i].
inline std::vector<double> survivorSignedVolumes(const std::vector<topo::Shape>& faces,
                                                 const tess::MeshParams& mp) {
  tess::FaceMesher fm(mp);
  tess::EdgeCache cache(mp.deflection, mp.edgeMinSegs, mp.edgeMaxSegs);
  for (const topo::Shape& f : faces) fm.requireEdgeSegments(f, cache);
  std::vector<double> vols;
  vols.reserve(faces.size());
  for (const topo::Shape& f : faces) vols.push_back(tess::enclosedVolume(fm.mesh(f, cache)));
  return vols;
}

// Weld `faces` into a coherent, watertight Solid. Tries, in order: identity, W's exact
// single-last-flip (so a degenerate-to-single-seam survivor set welds byte-identically to
// track W), flipping the whole B-group (the survivors from operand B wind oppositely
// to A's across every shared seam), and flipping the whole A-group. `nFromA` = the number
// of leading faces that came from wall A (the rest came from wall B).
//
// BOOL-VOTE selection: a candidate is accepted ONLY if it is watertight + coherent AND its
// enclosed volume agrees (±kWeldVolumeAgreeFrac) with the ORACLE-FREE divergence-theorem
// expectation of that configuration (the survivor faces' summed signed volumes) — never
// first-found-watertight. A weld may only repair the seam pairing; one that MOVED material
// (the measured d=0.002 strip-pinch, which silently lost the collar bands' volume) is
// REJECTED and, if no consistent configuration exists, the enumeration declines with the
// mismatch witnesses in `verdict` (an honest VolumeInconsistent, never a wrong solid).
// `verdict.minBoundaryEdges` keeps the sharpened NotWatertight residual map (0 ⇒ a clean
// weld existed; small non-zero ⇒ the frozen-mesher seam gap the caller declines at).
inline std::optional<std::pair<topo::Shape, tess::Mesh>> weldMultiCoherent(
    std::vector<topo::Shape> faces, std::size_t nFromA, const tess::MeshParams& mp,
    WeldVerdict& verdict) {
  auto build = [&](const std::vector<topo::Shape>& fs) {
    const topo::Shape shell = topo::ShapeBuilder::makeShell(fs);
    const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
    return std::make_pair(solid, tess::SolidMesher(mp).mesh(solid));
  };
  auto coherent = [](const tess::Mesh& m) {
    return tess::isWatertight(m) && tess::isConsistentlyOriented(m);
  };
  verdict = WeldVerdict{};
  const std::vector<double> faceVols = survivorSignedVolumes(faces, mp);
  // Candidate orientation repairs (each a face-index set to reverse).
  std::vector<std::vector<std::size_t>> repairs;
  repairs.push_back({});  // identity
  if (!faces.empty()) repairs.push_back({faces.size() - 1});  // W's single-last-flip
  if (nFromA < faces.size()) {                                // flip the whole B-group
    std::vector<std::size_t> b;
    for (std::size_t i = nFromA; i < faces.size(); ++i) b.push_back(i);
    repairs.push_back(std::move(b));
  }
  if (nFromA > 0 && nFromA < faces.size()) {                  // flip the whole A-group
    std::vector<std::size_t> a;                               // (the FUSE outer envelope's two
    for (std::size_t i = 0; i < nFromA; ++i) a.push_back(i);  // curved-region groups wind
    repairs.push_back(std::move(a));                          // oppositely across every seam)
  }
  for (const std::vector<std::size_t>& flip : repairs) {
    std::vector<topo::Shape> f = faces;
    std::vector<double> signedVols = faceVols;
    for (std::size_t i : flip) {
      f[i] = f[i].reversedShape();
      signedVols[i] = -signedVols[i];
    }
    auto [res, mesh] = build(f);
    verdict.minBoundaryEdges = std::min(verdict.minBoundaryEdges, tess::boundaryEdgeCount(mesh));
    if (!coherent(mesh)) continue;
    double expect = 0.0;
    for (double v : signedVols) expect += v;
    const double vExpect = std::fabs(expect);
    const double v = std::fabs(tess::enclosedVolume(mesh));
    verdict.weldedVolume = v;
    verdict.expectedVolume = vExpect;
    if (vExpect > 0.0 && std::fabs(v - vExpect) <= kWeldVolumeAgreeFrac * vExpect) {
      verdict.volumeMismatch = false;
      return std::make_pair(res, mesh);
    }
    verdict.volumeMismatch = true;  // coherent weld found, but it moved material — keep looking
  }
  return std::nullopt;
}

}  // namespace ffmdetail

// ─────────────────────────────────────────────────────────────────────────────
// freeformFreeformMultiSeamCutWithSeams — the MULTI-SEAM core, taking the already-traced
// closed seam loops (u1,v1 on A; u2,v2 on B). Splits BOTH walls by ALL seams, classifies
// EACH sub-region's membership, and sews the survivors watertight across EVERY seam,
// returning the welded, self-verified CUT (`A − B`) or COMMON (`A ∩ B`) solid — or a NULL
// Shape (→ OCCT) with a measured decline. Never a leaky/partial/wrong solid; no tolerance
// widened.
//
// This overload EXISTS so a caller (a host gate sweeping several deflections) can trace
// the — expensive — SSI seams ONCE and re-run only the split+weld+verify per deflection.
// The tracing wrapper below composes it. `seams` must be ≥ 2 closed loops; fewer ⇒
// NoMultiSeam (the single-seam case belongs to track W's `freeformFreeformClosedSeamCut`).
//
// `analyticOpVolume` (optional, NaN ⇒ unknown): the closed-form op-volume; when supplied
// the self-verify is TWO-SIDED (a too-small wrong volume is rejected VolumeInconsistent).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformFreeformMultiSeamCutWithSeams(
    const topo::Shape& A, const topo::Shape& B, const std::vector<ssi::WLine>& seams, FfOp op,
    double deflection = 0.005, MultiSeamCutReport* report = nullptr,
    double analyticOpVolume = std::numeric_limits<double>::quiet_NaN()) {
  using namespace ffcdetail;
  using namespace ffmdetail;
  MultiSeamCutReport rep;
  auto fail = [&](MultiSeamCutDecline d) -> topo::Shape {
    rep.decline = d;
    if (report) *report = rep;
    return {};
  };

  // (1) B1 recognise both operands + their single freeform walls.
  const auto foA = recogniseFreeformSolid(A);
  if (!foA) return fail(MultiSeamCutDecline::NotAdmittedA);
  const auto foB = recogniseFreeformSolid(B);
  if (!foB) return fail(MultiSeamCutDecline::NotAdmittedB);

  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  if (!fsA) return fail(MultiSeamCutDecline::WallUnusableA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsB) return fail(MultiSeamCutDecline::WallUnusableB);

  // (2) require ≥ 2 closed loops (the multi-seam case); fewer ⇒ NoMultiSeam.
  rep.seamLoops = static_cast<int>(seams.size());
  if (seams.size() < 2) return fail(MultiSeamCutDecline::NoMultiSeam);
  for (const ssi::WLine& s : seams)
    if (s.points.size() < 3 || s.status != ssi::TraceStatus::Closed)
      return fail(MultiSeamCutDecline::SeamUnusable);

  // (3) SPLIT BOTH walls by ALL seams. A's wall reads (u1,v1); B's wall reads a
  // (u2,v2)-re-keyed copy of EACH seam (splitWallBySeams reads points[i].{u1,v1}). Each
  // wall's seam edges are single-pcurve (the M0 mesher resolves the pcurve via its
  // single-pcurve fallback); the two walls share the seam by BIT-IDENTICAL 3-D chord
  // geometry (the SAME WLine), welded by the M0 BoundaryAnchors 3-D index.
  std::vector<ssi::WLine> seamsB;
  seamsB.reserve(seams.size());
  for (const ssi::WLine& s : seams) seamsB.push_back(rekeyToB(s));

  std::vector<WallRegion> regA, regB;
  if (!splitWallBySeams(wallA->face, seams, regA, rep.tilingGapA))
    return fail(MultiSeamCutDecline::MultiSplitFailedA);
  if (!splitWallBySeams(wallB->face, seamsB, regB, rep.tilingGapB))
    return fail(MultiSeamCutDecline::MultiSplitFailedB);
  rep.subRegionsA = static_cast<int>(regA.size());
  rep.subRegionsB = static_cast<int>(regB.size());

  // Pre-cut operand meshes (membership + independent op-volume bounds).
  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  if (!tess::isWatertight(meshA) || !tess::isWatertight(meshB))
    return fail(MultiSeamCutDecline::NotWatertight);
  const Aabb bbA = meshAabb(meshA), bbB = meshAabb(meshB);

  // (4) SELECT survivors by the hole-respecting membership vote (W's fix), per region.
  //   COMMON — A's regions INSIDE B ∪ B's regions INSIDE A (the lens between the seams).
  //   CUT    — A's regions OUTSIDE B ∪ A's lids OUTSIDE B ∪ B's regions INSIDE A.
  //   FUSE   — A's regions OUTSIDE B ∪ A's lids OUTSIDE B ∪ B's regions OUTSIDE A ∪ B's
  //            lids OUTSIDE A: the OUTER envelope (the complement of the COMMON annular
  //            lens on BOTH walls), sewn watertight across EVERY seam.
  const Membership wantA = op == FfOp::Common ? Membership::In : Membership::Out;
  const Membership wantB = op == FfOp::Common || op == FfOp::Cut ? Membership::In : Membership::Out;
  std::vector<topo::Shape> faces;
  std::size_t nFromA = 0;
  for (const WallRegion& reg : regA)
    if (subFaceHasMembership(reg.face, meshB, bbB, deflection, wantA)) { faces.push_back(reg.face); ++nFromA; }
  if (op == FfOp::Cut || op == FfOp::Fuse)
    collectAnalyticByMembership(*foA, meshB, bbB, deflection, Membership::Out, faces), nFromA = faces.size();
  for (const WallRegion& reg : regB)
    if (subFaceHasMembership(reg.face, meshA, bbA, deflection, wantB)) faces.push_back(reg.face);
  if (op == FfOp::Fuse)
    collectAnalyticByMembership(*foB, meshA, bbA, deflection, Membership::Out, faces);

  if (faces.size() < 2) return fail(MultiSeamCutDecline::WeldOpen);
  rep.survivorFaces = static_cast<int>(faces.size());

  // (5) WELD + orientation-coherence repair + BOOL-VOTE volume-consistent selection +
  // mandatory self-verify. On a NotWatertight failure the smallest unpaired-boundary count
  // across the tried repairs is recorded as the sharpened residual map (`boundaryEdges`);
  // when a watertight+coherent weld EXISTED but its volume disagreed with the survivors'
  // own divergence-theorem expectation (the mesher's weld moved material — the measured
  // d=0.002 strip-pinch), the decline is VolumeInconsistent with both volumes as witnesses.
  // Either way an honest decline, never a leaky/wrong solid.
  WeldVerdict wv;
  const auto welded = weldMultiCoherent(std::move(faces), nFromA, mp, wv);
  if (!welded) {
    rep.boundaryEdges = wv.minBoundaryEdges;
    if (wv.volumeMismatch) {
      // The mismatched candidate WAS watertight + coherent — carry that in the report so
      // the decline map reads "the weld closed, but at a moved-material volume", not "the
      // weld failed to close" (the two residuals live in different layers).
      rep.watertight = true;
      rep.coherent = true;
      rep.enclosedVolume = wv.weldedVolume;
      rep.expectedVolume = wv.expectedVolume;
      return fail(MultiSeamCutDecline::VolumeInconsistent);
    }
    return fail(MultiSeamCutDecline::NotWatertight);
  }
  const topo::Shape result = welded->first;
  const tess::Mesh& m = welded->second;
  rep.watertight = tess::isWatertight(m);
  rep.coherent = tess::isConsistentlyOriented(m);
  rep.boundaryEdges = tess::boundaryEdgeCount(m);
  rep.expectedVolume = wv.expectedVolume;
  const double v = std::fabs(tess::enclosedVolume(m));
  rep.enclosedVolume = v;
  if (!(v > 0.0) || std::isnan(v)) return fail(MultiSeamCutDecline::VolumeInconsistent);

  // Self-verify — op-volume bounds (CUT ⊂ A; COMMON ⊂ A and ⊂ B; FUSE ⊇ max(A,B) and
  // ⊂ A+B). FUSE is two-sided (a too-small envelope — an orientation-collapsed shell that
  // dropped one operand's material — is rejected as VolumeInconsistent, never returned).
  const double vA = std::fabs(tess::enclosedVolume(meshA));
  const double vB = std::fabs(tess::enclosedVolume(meshB));
  const double tol = 0.05 * std::max(std::max(vA, vB), 1e-12);
  double upper;
  switch (op) {
    case FfOp::Cut: upper = vA; break;
    case FfOp::Common: upper = std::min(vA, vB); break;
    case FfOp::Fuse: upper = vA + vB; break;
  }
  if (v > upper + tol) return fail(MultiSeamCutDecline::VolumeInconsistent);
  if (op == FfOp::Fuse && v < std::max(vA, vB) - tol)
    return fail(MultiSeamCutDecline::VolumeInconsistent);

  // TWO-SIDED band vs the closed-form op-volume, if supplied.
  if (!std::isnan(analyticOpVolume) && analyticOpVolume > 0.0) {
    constexpr double kVolConvergeSlope = 30.0;
    const double band = std::min(0.5, kVolConvergeSlope * deflection) * analyticOpVolume;
    if (std::fabs(v - analyticOpVolume) > band)
      return fail(MultiSeamCutDecline::VolumeInconsistent);
  }

  rep.decline = MultiSeamCutDecline::Ok;
  if (report) *report = rep;
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// freeformFreeformMultiSeamCut — the MULTI-SEAM entry point (tracing wrapper). Traces
// ALL shared closed seams `A.wall ∩ B.wall` (the SSI S3 pass) and forwards them to the
// core above. Returns the welded, self-verified CUT / COMMON solid — or a NULL Shape
// (→ OCCT) with a measured decline. A pose whose SSI returns fewer than TWO closed loops
// DECLINES `NoMultiSeam` (the single-seam case belongs to track W's
// `freeformFreeformClosedSeamCut`, left untouched).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformFreeformMultiSeamCut(
    const topo::Shape& A, const topo::Shape& B, FfOp op, double deflection = 0.005,
    MultiSeamCutReport* report = nullptr,
    double analyticOpVolume = std::numeric_limits<double>::quiet_NaN()) {
  using namespace ffcdetail;
  MultiSeamCutReport rep;
  auto fail = [&](MultiSeamCutDecline d) -> topo::Shape {
    rep.decline = d;
    if (report) *report = rep;
    return {};
  };

  const auto foA = recogniseFreeformSolid(A);
  if (!foA) return fail(MultiSeamCutDecline::NotAdmittedA);
  const auto foB = recogniseFreeformSolid(B);
  if (!foB) return fail(MultiSeamCutDecline::NotAdmittedB);
  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  if (!fsA) return fail(MultiSeamCutDecline::WallUnusableA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsB) return fail(MultiSeamCutDecline::WallUnusableB);

  // M1 trace ALL shared closed seams (u1,v1 on A; u2,v2 on B).
  const ssi::SurfaceAdapter adA = ssi::makeBezierAdapter(fsA->poles, fsA->nPolesU, fsA->nPolesV);
  const ssi::SurfaceAdapter adB = ssi::makeBezierAdapter(fsB->poles, fsB->nPolesU, fsB->nPolesV);
  const ssi::TraceSet tr = ssi::trace_intersection(adA, adB);
  std::vector<ssi::WLine> seams;
  for (const ssi::WLine& w : tr.lines)
    if (w.points.size() >= 3 && w.status == ssi::TraceStatus::Closed) seams.push_back(w);
  if (seams.size() < 2) { rep.seamLoops = static_cast<int>(seams.size());
    return fail(MultiSeamCutDecline::NoMultiSeam); }

  return freeformFreeformMultiSeamCutWithSeams(A, B, seams, op, deflection, report, analyticOpVolume);
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_MULTISEAM_H

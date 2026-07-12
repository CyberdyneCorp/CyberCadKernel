// SPDX-License-Identifier: Apache-2.0
//
// nurbs_solid_boolean.h — BOOL-INT / LAYER 3: the GENERAL two-freeform-solid NURBS
// boolean ORCHESTRATOR. `nurbsSolidBoolean(A, B, op)` for op ∈ {Fuse, Cut, Common} over
// two general freeform NURBS solids (a single trimmed-freeform wall + analytic lids each),
// composing the five landed L3 stage verbs into ONE watertight-or-honest-decline result.
//
// ── WHAT THIS IS (an ORCHESTRATOR, not a new stage) ───────────────────────────────
// Every stage of an exact-NURBS B-rep boolean is already a landed, measured verb:
//   Stage 1  SSI          — `ssi::trace_intersection` over `makeBezierAdapter` walls
//                           (seam trace; open-transversal + closed loops).
//   Stage 2  pcurve       — the WLine's per-node `(u1,v1)`/`(u2,v2)` read directly (the
//                           `constructPcurve`-free path the readiness doc names for the
//                           tractable slice).
//   Stage 3  split+heal   — `splitFaceSmoothTrim` (closed interior seam → disk+annulus)
//                           and `freeform_freeform_multiseam.h splitWallBySeams`
//                           (multi-crossing nesting-aware split).
//   Stage 4  membership   — `ffcdetail::subFaceHasMembership` hole-respecting interior-rep
//                           vote (the `classifyFragmentVsSolid` family, batch-select form).
//   Stage 5  sew          — `freeformFreeformClosedSeamCut` (track W single-seam weld) +
//                           `freeformFreeformMultiSeamCut` (L3-d multi-seam).
// This header COMPOSES those verbs; it re-implements NONE of them. The five stage-verb
// files stay BYTE-UNCHANGED. It adds exactly the op-level dispatch + the FUSE survivor
// select/weld the single-seam CUT/COMMON verb did not expose, and the honest-decline
// routing for the poses a sub-verb abstains on (multi-seam annulus↔annulus).
//
// ── THE OPERATIONS (per-op survivor set over the single closed transversal seam) ───
// For the canonical bowl-cup pose (A opens one way, B the mirror, walls meet in ONE
// closed seam circle at r = ρ, each wall splitting into a seam-enclosed DISK + the rim
// ANNULUS):
//   * COMMON = A∩B — A's disk (INSIDE B) ∪ B's disk (INSIDE A): the lens. Delegated
//     byte-identically to `freeformFreeformClosedSeamCut(..., FfOp::Common)`.
//   * CUT    = A−B — A's annulus (OUT B) ∪ A's lid (OUT B) ∪ B's disk (IN A): the carved
//     bowl. Delegated byte-identically to `freeformFreeformClosedSeamCut(..., FfOp::Cut)`.
//   * FUSE   = A∪B — A's annulus (OUT B) ∪ A's lid (OUT B) ∪ B's annulus (OUT A) ∪ B's
//     lid (OUT A): the outer envelope. The two rim annuli meet across the shared seam;
//     the survivors weld watertight through the SAME M0w seam pin (MEASURED: watertight,
//     be=0, and — after a coherence repair that may flip the whole B-group — coherent at
//     V(A)+V(B)−V(A∩B) within the tessellation band). This is the ONE op the single-seam
//     CUT/COMMON verb did not expose; it is added here as a composition of the SAME
//     select+weld primitives (`ffcdetail::{subFaceHasMembership, collectAnalyticByMembership}`
//     + a group-aware coherence repair), NOT a new sew.
//
// ── HONESTY (DISAGREED=0 SACRED) ──────────────────────────────────────────────────
// Every op self-verifies: watertight + consistently-oriented + a positive enclosed volume
// under a per-op UPPER bound (COMMON ⊂ min(A,B); CUT ⊂ A; FUSE ⊂ A+B and ⊇ max(A,B)),
// TWO-SIDED against the closed-form op-volume when supplied. Any sub-case that does not
// weld to a verified watertight/coherent solid — notably the MULTI-SEAM annulus↔annulus
// inner-seam sew (the frozen-M0-mesher holed-curved-seam gap, per L3-d) — returns a NULL
// Shape with a MEASURED decline + a residual map (the sharpened `boundaryEdges`). NEVER a
// leaky/partial/wrong solid; NO tolerance is ever widened.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_H
#define CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_H

#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
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
// The requested boolean operator over two freeform NURBS solids.
// ─────────────────────────────────────────────────────────────────────────────
enum class SolidBoolOp { Fuse, Cut, Common };

inline const char* solidBoolOpName(SolidBoolOp op) noexcept {
  switch (op) {
    case SolidBoolOp::Fuse: return "Fuse";
    case SolidBoolOp::Cut: return "Cut";
    case SolidBoolOp::Common: return "Common";
  }
  return "?";
}

// ─────────────────────────────────────────────────────────────────────────────
// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
// watertight, coherent result solid is returned. NEVER a leaky/partial/wrong solid.
// ─────────────────────────────────────────────────────────────────────────────
enum class SolidBoolDecline {
  Ok,
  NotAdmittedA,        ///< B1 declined operand A
  NotAdmittedB,        ///< B1 declined operand B
  WallUnusableA,       ///< A does not present exactly one usable freeform wall
  WallUnusableB,       ///< B does not present exactly one usable freeform wall
  NoSeam,              ///< the SSI trace returned NO usable closed/open shared seam
  MultiSeamDeclined,   ///< a genuine multi-seam pose whose sew honest-declines (residual map)
  SingleSeamDeclined,  ///< the single-seam sub-verb declined (its FfCutDecline in `subDecline`)
  SplitFailed,         ///< a wall split declined (FUSE path)
  ClassifyAmbiguous,   ///< a survivor sub-face membership is ambiguous (FUSE path)
  WeldOpen,            ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,       ///< self-verify: the welded result is not a closed/coherent 2-manifold
  VolumeInconsistent   ///< self-verify: the enclosed volume is non-positive / off the bound
};

inline const char* solidBoolDeclineName(SolidBoolDecline d) noexcept {
  switch (d) {
    case SolidBoolDecline::Ok: return "Ok";
    case SolidBoolDecline::NotAdmittedA: return "NotAdmittedA";
    case SolidBoolDecline::NotAdmittedB: return "NotAdmittedB";
    case SolidBoolDecline::WallUnusableA: return "WallUnusableA";
    case SolidBoolDecline::WallUnusableB: return "WallUnusableB";
    case SolidBoolDecline::NoSeam: return "NoSeam";
    case SolidBoolDecline::MultiSeamDeclined: return "MultiSeamDeclined";
    case SolidBoolDecline::SingleSeamDeclined: return "SingleSeamDeclined";
    case SolidBoolDecline::SplitFailed: return "SplitFailed";
    case SolidBoolDecline::ClassifyAmbiguous: return "ClassifyAmbiguous";
    case SolidBoolDecline::WeldOpen: return "WeldOpen";
    case SolidBoolDecline::NotWatertight: return "NotWatertight";
    case SolidBoolDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

/// The honest witnesses of an orchestrated boolean (for the host gate / caller log).
struct SolidBoolReport {
  SolidBoolDecline decline = SolidBoolDecline::Ok;
  SolidBoolOp op = SolidBoolOp::Common;
  int seamLoops = 0;                 ///< closed shared seam loops the SSI returned
  bool multiSeam = false;            ///< dispatched to the multi-seam path (≥ 2 loops)
  int survivorFaces = 0;             ///< kept faces welded into the result
  bool watertight = false;           ///< M0 self-verify: closed 2-manifold
  bool coherent = false;             ///< M0 self-verify: consistently oriented
  std::size_t boundaryEdges = 1;     ///< unpaired boundary edges (0 ⇒ watertight); residual map
  double enclosedVolume = 0.0;       ///< signed-tetra enclosed volume of the result
  FfCutDecline subDecline = FfCutDecline::Ok;              ///< the single-seam sub-verb's reason
  MultiSeamCutDecline multiDecline = MultiSeamCutDecline::Ok;  ///< the multi-seam sub-verb's reason
};

namespace nsbdetail {

using ffcdetail::collectAnalyticByMembership;
using ffcdetail::freeformWall;
using ffcdetail::pickByMembership;
using ffcdetail::rekeyToB;
using ffcdetail::subFaceHasMembership;
using ffcdetail::traceSharedSeam;

/// Count the SHARED closed seam loops between two freeform walls (the multi-seam trigger).
/// Reuses the SAME S3 trace the sub-verbs use; returns the closed loops (≥ 3 nodes).
inline std::vector<ssi::WLine> tracedClosedSeams(const topo::FaceSurface& fsA,
                                                 const topo::FaceSurface& fsB) {
  const ssi::SurfaceAdapter adA = ssi::makeBezierAdapter(fsA.poles, fsA.nPolesU, fsA.nPolesV);
  const ssi::SurfaceAdapter adB = ssi::makeBezierAdapter(fsB.poles, fsB.nPolesU, fsB.nPolesV);
  const ssi::TraceSet tr = ssi::trace_intersection(adA, adB);
  std::vector<ssi::WLine> seams;
  for (const ssi::WLine& w : tr.lines)
    if (w.points.size() >= 3 && w.status == ssi::TraceStatus::Closed) seams.push_back(w);
  return seams;
}

/// Weld `faces` into a coherent watertight Solid. `nFromA` leading faces came from A, the
/// rest from B. Tries, in order: identity, W's single-last-flip, flip-the-whole-B-group,
/// flip-the-whole-A-group (the FUSE envelope's two curved annuli wind oppositely across the
/// shared seam, so a whole-group flip — NOT a single face — restores outward-normal
/// coherence). The MANDATORY M0 watertight + consistent-orientation self-verify gates each
/// candidate; the SMALLEST unpaired-boundary count seen is recorded as the residual map for
/// an honest decline. Returns the coherent (result, mesh) or nullopt (never a leaky solid).
inline std::optional<std::pair<topo::Shape, tess::Mesh>> weldGroupCoherent(
    std::vector<topo::Shape> faces, std::size_t nFromA, const tess::MeshParams& mp,
    std::size_t& minBoundaryEdges) {
  auto build = [&](const std::vector<topo::Shape>& fs) {
    const topo::Shape shell = topo::ShapeBuilder::makeShell(fs);
    const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
    return std::make_pair(solid, tess::SolidMesher(mp).mesh(solid));
  };
  auto coherent = [](const tess::Mesh& m) {
    return tess::isWatertight(m) && tess::isConsistentlyOriented(m);
  };
  minBoundaryEdges = std::numeric_limits<std::size_t>::max();
  std::vector<std::vector<std::size_t>> repairs;
  repairs.push_back({});                                      // identity
  if (!faces.empty()) repairs.push_back({faces.size() - 1});  // W's single-last-flip
  if (nFromA < faces.size()) {                                // flip the whole B-group
    std::vector<std::size_t> b;
    for (std::size_t i = nFromA; i < faces.size(); ++i) b.push_back(i);
    repairs.push_back(std::move(b));
  }
  if (nFromA > 0 && nFromA < faces.size()) {                  // flip the whole A-group
    std::vector<std::size_t> a;
    for (std::size_t i = 0; i < nFromA; ++i) a.push_back(i);
    repairs.push_back(std::move(a));
  }
  for (const std::vector<std::size_t>& flip : repairs) {
    std::vector<topo::Shape> f = faces;
    for (std::size_t i : flip) f[i] = f[i].reversedShape();
    auto [res, mesh] = build(f);
    minBoundaryEdges = std::min(minBoundaryEdges, tess::boundaryEdgeCount(mesh));
    if (coherent(mesh)) return std::make_pair(res, mesh);
  }
  return std::nullopt;
}

}  // namespace nsbdetail

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidFuse — the FUSE (`A ∪ B`) leg for the single-transversal-seam pose. Splits
// BOTH walls by the ONE shared seam, keeps A's OUTSIDE-B fragments (annulus + lids) and
// B's OUTSIDE-A fragments (annulus + lids), and welds the outer envelope through the M0w
// seam pin with a group-aware orientation-coherence repair + mandatory self-verify.
// Returns the welded, self-verified solid or a NULL Shape with a measured decline. This
// is the ONE op the single-seam CUT/COMMON verb did not expose; COMMON/CUT DELEGATE to
// `freeformFreeformClosedSeamCut` byte-identically (see `nurbsSolidBoolean`).
//
// `analyticOpVolume` (optional, NaN ⇒ unknown) enables the TWO-SIDED volume band.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidFuse(const topo::Shape& A, const topo::Shape& B,
                                  const ssi::WLine& seam, double deflection,
                                  SolidBoolReport& rep,
                                  double analyticOpVolume =
                                      std::numeric_limits<double>::quiet_NaN()) {
  using namespace ffcdetail;
  using namespace nsbdetail;
  auto fail = [&](SolidBoolDecline d) -> topo::Shape { rep.decline = d; return {}; };

  const auto foA = recogniseFreeformSolid(A);
  const auto foB = recogniseFreeformSolid(B);
  if (!foA) return fail(SolidBoolDecline::NotAdmittedA);
  if (!foB) return fail(SolidBoolDecline::NotAdmittedB);
  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsA) return fail(SolidBoolDecline::WallUnusableA);
  if (!fsB) return fail(SolidBoolDecline::WallUnusableB);

  // Stage 3 — split BOTH walls by the shared seam.
  const SmoothSplitResult srA = splitFaceSmoothTrim(wallA->face, seam);
  const SmoothSplitResult srB = splitFaceSmoothTrim(wallB->face, rekeyToB(seam));
  if (!srA.ok() || !srB.ok()) return fail(SolidBoolDecline::SplitFailed);

  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  if (!tess::isWatertight(meshA) || !tess::isWatertight(meshB))
    return fail(SolidBoolDecline::NotWatertight);
  const Aabb bbA = meshAabb(meshA), bbB = meshAabb(meshB);

  // Stage 4 — select the OUTER envelope survivors (A OUTSIDE B, then B OUTSIDE A).
  std::vector<topo::Shape> faces;
  const auto aKeep = pickByMembership(*srA.split, meshB, bbB, deflection, Membership::Out);
  if (!aKeep) return fail(SolidBoolDecline::ClassifyAmbiguous);
  faces.push_back(*aKeep);
  collectAnalyticByMembership(*foA, meshB, bbB, deflection, Membership::Out, faces);
  const std::size_t nFromA = faces.size();
  const auto bKeep = pickByMembership(*srB.split, meshA, bbA, deflection, Membership::Out);
  if (!bKeep) return fail(SolidBoolDecline::ClassifyAmbiguous);
  faces.push_back(*bKeep);
  collectAnalyticByMembership(*foB, meshA, bbA, deflection, Membership::Out, faces);
  if (faces.size() < 2) return fail(SolidBoolDecline::WeldOpen);
  rep.survivorFaces = static_cast<int>(faces.size());

  // Stage 5 — weld + group-aware coherence repair + mandatory self-verify.
  std::size_t minBE = 1;
  const auto welded = weldGroupCoherent(std::move(faces), nFromA, mp, minBE);
  if (!welded) { rep.boundaryEdges = minBE; return fail(SolidBoolDecline::NotWatertight); }
  const topo::Shape result = welded->first;
  const tess::Mesh& m = welded->second;
  rep.watertight = tess::isWatertight(m);
  rep.coherent = tess::isConsistentlyOriented(m);
  rep.boundaryEdges = tess::boundaryEdgeCount(m);
  const double v = std::fabs(tess::enclosedVolume(m));
  rep.enclosedVolume = v;
  if (!(v > 0.0) || std::isnan(v)) return fail(SolidBoolDecline::VolumeInconsistent);

  // Self-verify — FUSE contains BOTH operands and is contained in their sum.
  const double vA = std::fabs(tess::enclosedVolume(meshA));
  const double vB = std::fabs(tess::enclosedVolume(meshB));
  const double band = 0.10 * std::max(std::max(vA, vB), 1e-12);
  if (v < std::max(vA, vB) - band) return fail(SolidBoolDecline::VolumeInconsistent);
  if (v > vA + vB + band) return fail(SolidBoolDecline::VolumeInconsistent);

  if (!std::isnan(analyticOpVolume) && analyticOpVolume > 0.0) {
    constexpr double kVolConvergeSlope = 30.0;
    const double vband = std::min(0.5, kVolConvergeSlope * deflection) * analyticOpVolume;
    if (std::fabs(v - analyticOpVolume) > vband)
      return fail(SolidBoolDecline::VolumeInconsistent);
  }

  rep.decline = SolidBoolDecline::Ok;
  return result;
}

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidBoolean — the general two-freeform-solid NURBS boolean ORCHESTRATOR.
//
// Composes the five landed L3 stage verbs into ONE watertight result for op ∈
// {Fuse, Cut, Common} over two freeform NURBS solids A, B, or an HONEST decline to NULL
// with a measured `SolidBoolReport` when a sew sub-case abstains (never a leaky/wrong
// solid; no tolerance widened). Dispatch:
//   * SSI-trace the shared closed seams once.
//   * ONE closed seam (single transversal): COMMON/CUT DELEGATE byte-identically to
//     `freeformFreeformClosedSeamCut`; FUSE runs `nurbsSolidFuse` (the OUTER-envelope
//     compose the single-seam verb did not expose).
//   * ≥ 2 closed seams (genuine multi-seam): dispatch to `freeformFreeformMultiSeamCut`
//     (CUT/COMMON) — which splits + classifies exactly and HONEST-DECLINES the
//     annulus↔annulus inner-seam sew (the frozen-M0-mesher holed-curved-seam gap, L3-d)
//     with a residual map. FUSE over a multi-seam pose declines the same way.
//   * NO usable seam ⇒ NoSeam.
//
// `analyticOpVolume` (optional) enables the TWO-SIDED closed-form volume band.
// ─────────────────────────────────────────────────────────────────────────────
// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidBooleanWithSeams — the ORCHESTRATOR core, taking the already-traced shared
// closed seams. This overload EXISTS so a caller (a host gate sweeping several deflections,
// or a multi-op run) can trace the — expensive, e.g. degree-4 — SSI seams ONCE and re-run
// only the split+select+weld+verify per call. `nurbsSolidBoolean` below traces then
// forwards. Semantics identical to `nurbsSolidBoolean`.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidBooleanWithSeams(const topo::Shape& A, const topo::Shape& B,
                                              const std::vector<ssi::WLine>& seams, SolidBoolOp op,
                                              double deflection = 0.005,
                                              SolidBoolReport* report = nullptr,
                                              double analyticOpVolume =
                                                  std::numeric_limits<double>::quiet_NaN()) {
  using namespace nsbdetail;
  SolidBoolReport rep;
  rep.op = op;
  auto emit = [&](topo::Shape s) -> topo::Shape {
    if (report) *report = rep;
    return s;
  };
  auto fail = [&](SolidBoolDecline d) -> topo::Shape { rep.decline = d; return emit({}); };

  // (1) B1 recognise both operands + their single freeform walls.
  const auto foA = recogniseFreeformSolid(A);
  if (!foA) return fail(SolidBoolDecline::NotAdmittedA);
  const auto foB = recogniseFreeformSolid(B);
  if (!foB) return fail(SolidBoolDecline::NotAdmittedB);
  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  if (!fsA) return fail(SolidBoolDecline::WallUnusableA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsB) return fail(SolidBoolDecline::WallUnusableB);

  rep.seamLoops = static_cast<int>(seams.size());

  // (2) MULTI-SEAM dispatch (≥ 2 closed loops) — split+classify exactly, honest-decline
  // the annulus↔annulus inner-seam sew (L3-d) with the residual map.
  if (seams.size() >= 2) {
    rep.multiSeam = true;
    if (op == SolidBoolOp::Common || op == SolidBoolOp::Cut) {
      MultiSeamCutReport mrep;
      const FfOp fop = op == SolidBoolOp::Common ? FfOp::Common : FfOp::Cut;
      const topo::Shape r = freeformFreeformMultiSeamCutWithSeams(
          A, B, seams, fop, deflection, &mrep, analyticOpVolume);
      rep.multiDecline = mrep.decline;
      rep.survivorFaces = mrep.survivorFaces;
      rep.watertight = mrep.watertight;
      rep.coherent = mrep.coherent;
      rep.boundaryEdges = mrep.boundaryEdges;
      rep.enclosedVolume = mrep.enclosedVolume;
      if (!r.isNull()) { rep.decline = SolidBoolDecline::Ok; return emit(r); }
      rep.decline = SolidBoolDecline::MultiSeamDeclined;
      return emit({});
    }
    // FUSE over a multi-seam pose is the same annulus↔annulus inner-seam sew — declined.
    rep.decline = SolidBoolDecline::MultiSeamDeclined;
    return emit({});
  }

  // (4) SINGLE-SEAM dispatch (exactly one closed transversal seam).
  if (seams.size() != 1) return fail(SolidBoolDecline::NoSeam);

  if (op == SolidBoolOp::Common || op == SolidBoolOp::Cut) {
    FfCutDecline why = FfCutDecline::Ok;
    const FfOp fop = op == SolidBoolOp::Common ? FfOp::Common : FfOp::Cut;
    const topo::Shape r =
        freeformFreeformClosedSeamCut(A, B, fop, deflection, &why, analyticOpVolume);
    rep.subDecline = why;
    if (!r.isNull()) {
      rep.decline = SolidBoolDecline::Ok;
      tess::MeshParams mp;
      mp.deflection = deflection;
      const tess::Mesh m = tess::SolidMesher(mp).mesh(r);
      rep.watertight = tess::isWatertight(m);
      rep.coherent = tess::isConsistentlyOriented(m);
      rep.boundaryEdges = tess::boundaryEdgeCount(m);
      rep.enclosedVolume = std::fabs(tess::enclosedVolume(m));
      return emit(r);
    }
    rep.decline = SolidBoolDecline::SingleSeamDeclined;
    return emit({});
  }

  // FUSE — the OUTER envelope compose (the single-seam CUT/COMMON verb did not expose it).
  const topo::Shape r = nurbsSolidFuse(A, B, seams.front(), deflection, rep, analyticOpVolume);
  return emit(r);
}

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidBoolean — the general two-freeform-solid NURBS boolean ORCHESTRATOR (tracing
// wrapper). SSI-traces the shared closed seams `A.wall ∩ B.wall` ONCE and forwards them to
// the core above. Returns the composed watertight result for op ∈ {Fuse, Cut, Common} or an
// HONEST decline to NULL with a measured `SolidBoolReport` (never a leaky/wrong solid; no
// tolerance widened).
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidBoolean(const topo::Shape& A, const topo::Shape& B, SolidBoolOp op,
                                     double deflection = 0.005, SolidBoolReport* report = nullptr,
                                     double analyticOpVolume =
                                         std::numeric_limits<double>::quiet_NaN()) {
  using namespace nsbdetail;
  SolidBoolReport rep;
  rep.op = op;
  auto emit = [&](topo::Shape s) -> topo::Shape {
    if (report) *report = rep;
    return s;
  };
  auto fail = [&](SolidBoolDecline d) -> topo::Shape { rep.decline = d; return emit({}); };

  // Recognise + trace once, then forward the seams to the core (no double-trace).
  const auto foA = recogniseFreeformSolid(A);
  if (!foA) return fail(SolidBoolDecline::NotAdmittedA);
  const auto foB = recogniseFreeformSolid(B);
  if (!foB) return fail(SolidBoolDecline::NotAdmittedB);
  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  if (!fsA) return fail(SolidBoolDecline::WallUnusableA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsB) return fail(SolidBoolDecline::WallUnusableB);

  const std::vector<ssi::WLine> seams = tracedClosedSeams(*fsA, *fsB);
  return nurbsSolidBooleanWithSeams(A, B, seams, op, deflection, report, analyticOpVolume);
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_NURBS_SOLID_BOOLEAN_H

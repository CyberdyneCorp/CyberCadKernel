// SPDX-License-Identifier: Apache-2.0
//
// boolean_readmit.h — BOOL-READMIT / LAYER 3: make the binary boolean's WELDED OUTPUT
// re-admissible as a boolean INPUT, so `nurbsSolidUnionN` / `nurbsSolidCutN` /
// `nurbsSolidIntersectN` weld for ≥3 operands instead of honest-declining at the
// re-admission boundary (nurbs_solid_boolean_nary.h fold index ≥2).
//
// ── THE MEASURED BLOCKER (from BOOL-MULTI-OP) ─────────────────────────────────────
// The binary `nurbsSolidBoolean(A, B, op)` welds two SINGLE-freeform-wall bowl-cups into
// ONE watertight solid, but that solid is NOT a single-freeform-wall bowl-cup — it is a
// MULTI-WALL result whose freeform walls are SEAM-SPLIT ANNULI (the shared seam is an
// interior HOLE loop). The byte-frozen B1 gate `recogniseFreeformSolid` deliberately
// declines it on TWO counts (both MEASURED on the canonical pose):
//   (1) `HoledFreeformFace` — an admitted freeform wall must carry EXACTLY one outer trim
//       loop and NO hole; a boolean-output annulus carries the seam as an interior hole.
//   (2) exactly-ONE-freeform-wall — `ffcdetail::freeformWall` requires `freeform.size()==1`;
//       a Fuse envelope has TWO curved annuli (A's + B's), a Cut has A's annulus + B's
//       curved cap. So even a hole-healed operand still presents ≥2 freeform walls.
// Neither is a DEFECT: an annulus is a perfectly valid trimmed-NURBS face (the seam is a
// legitimate trim loop), and a multi-wall solid is a perfectly valid solid — they are just
// OUTSIDE the frozen gate's single-simply-trimmed-wall admission window.
//
// ── WHAT THIS IS (an ADDITIVE re-admitting ORCHESTRATOR, not a new stage/gate) ─────
// This header does NOT edit `recogniseFreeformSolid` / `freeformWall` (byte-frozen,
// DISAGREED-sacred) and re-implements NONE of the five stage verbs. It adds:
//   * `recogniseFreeformSolidReadmit` — a strictly MORE-PERMISSIVE sibling of the B1 gate
//     that ADMITS a holed freeform wall (annulus) and ANY NUMBER of freeform walls,
//     reusing `fodetail::{classifyFaceRole, faceOutwardNormal, watertightByEdgeIncidence,
//     foldAabb}` byte-identically (only the per-face `HoledFreeformFace` reject and the
//     `NoFreeformFace`/single-wall constraint are relaxed). It yields the SAME
//     `FreeformOperand` struct the stage verbs consume.
//   * `nurbsSolidBooleanReadmit` — the general two-operand boolean over a possibly
//     MULTI-freeform-wall accumulator `acc` and a pristine single-wall `operand`. It picks
//     the ONE `acc` wall that shares a usable closed seam with `operand`'s wall (MEASURED:
//     for the canonical fold exactly one does), splits that pair by the seam
//     (`splitFaceSmoothTrim`), selects survivors by hole-respecting membership
//     (`pickByMembership` / `subFaceHasMembership` / `collectAnalyticByMembership`),
//     CARRIES THROUGH `acc`'s non-participating freeform walls whole by their membership,
//     then welds + a group-aware orientation-coherence repair + the SAME mandatory
//     watertight/coherent/volume-band self-verify. Every geometric primitive is reused
//     byte-identically; the only new code is the MULTI-WALL orchestration.
//
// ── HONESTY (DISAGREED=0 SACRED) ──────────────────────────────────────────────────
// The re-admit path self-verifies exactly as the binary boolean does (watertight +
// consistently-oriented + a per-op volume band, TWO-SIDED vs the closed form when
// supplied). Any sub-case that does not weld to a verified watertight/coherent solid —
// no seam-sharing wall, an ambiguous membership, a non-coherent weld, an off-band volume
// — returns a NULL Shape with a MEASURED decline. NEVER a leaky/partial/wrong solid; NO
// tolerance is EVER widened. When `acc` is itself a pristine single-wall operand (the
// FIRST fold step), the re-admit path is bit-identical to the binary boolean (it defers
// to `nurbsSolidBoolean` verbatim), so 2-operand folds stay UNREGRESSED.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20. NO cc_* ABI change.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_BOOLEAN_READMIT_H
#define CYBERCAD_NATIVE_BOOLEAN_BOOLEAN_READMIT_H

#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/nurbs_solid_boolean.h"
#include "native/boolean/smooth_trim_split.h"
#include "native/ssi/marching.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace cybercad::native::boolean {

// ─────────────────────────────────────────────────────────────────────────────
// The re-admit descriptor: a FreeformOperand admitted through the MORE-PERMISSIVE
// gate (holed walls + any number of freeform walls). It is the SAME struct the stage
// verbs read; `freeform` may now hold ≥ 1 index.
// ─────────────────────────────────────────────────────────────────────────────

namespace readmit {

/// Admit ONE boundary face into an `OperandFace`, TOLERATING a holed freeform wall (the
/// seam trim loop of a boolean-output annulus is a legitimate loop, not a defect). Reuses
/// `fodetail::{classifyFaceRole, faceOutwardNormal}` byte-identically; only the
/// `HoledFreeformFace`/`BareFreeformFace` rejects are dropped for the Freeform role. A
/// missing surface / unsupported kind still declines (those ARE genuine non-admissions).
inline std::optional<OperandFace> admitFaceReadmit(const topo::Shape& f, OperandDecline& why) {
  const auto surf = topo::surfaceOf(f);
  if (!surf || !surf->surface) { why = OperandDecline::FaceSurfaceMissing; return std::nullopt; }
  const auto role = fodetail::classifyFaceRole(surf->surface->kind);
  if (!role) { why = OperandDecline::UnsupportedSurfaceKind; return std::nullopt; }
  OperandFace of;
  of.face = f;
  of.surface = *surf->surface;
  of.location = surf->location;
  of.role = *role;
  of.outwardN = fodetail::faceOutwardNormal(f, of.surface, of.location);
  return of;
}

/// Watertight witness for a re-admitted solid. The frozen gate uses a TOPOLOGY-graph
/// edge-incidence audit (`fodetail::watertightByEdgeIncidence`) that requires coincident
/// faces to SHARE vertex identities — a boolean-welded output shares the seam GEOMETRICALLY
/// but its newly-built survivor faces carry independent seam vertices, so that stricter
/// audit fails even though the boolean already SELF-VERIFIED the result watertight at the
/// MESH level. We therefore accept EITHER the topology audit (a pristine solid) OR a
/// mesh-level `isWatertight` at a fine deflection (the boolean-output case). This is NOT a
/// weakened tolerance: mesh-level watertightness is the SAME exactly-two-incidences
/// predicate, and it is precisely the property the binary boolean's own self-verify proved
/// before this solid was ever produced. A genuinely open/leaky solid fails BOTH.
inline bool watertightForReadmit(const topo::Shape& s) {
  if (fodetail::watertightByEdgeIncidence(s)) return true;
  tess::MeshParams mp;
  mp.deflection = 0.0025;  // a fine mesh; watertightness is deflection-independent for a
                          // closed solid (finer only splits triangles, never opens edges).
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  return tess::isWatertight(m);
}

}  // namespace readmit

// ─────────────────────────────────────────────────────────────────────────────
// recogniseFreeformSolidReadmit — the MORE-PERMISSIVE B1 sibling. Admits a solid with
// one shell, a watertight boundary (topology audit OR mesh-level watertight — see
// `watertightForReadmit`), at least one freeform wall (of ANY trim status incl. holed
// annulus) and analytic caps. `nullopt` with a measured `OperandDecline` otherwise. Does
// NOT edit `recogniseFreeformSolid`.
// ─────────────────────────────────────────────────────────────────────────────
inline std::optional<FreeformOperand> recogniseFreeformSolidReadmit(
    const topo::Shape& s, OperandDecline* why = nullptr) {
  auto fail = [&](OperandDecline d) -> std::optional<FreeformOperand> {
    if (why) *why = d;
    return std::nullopt;
  };
  if (s.isNull() || s.type() != topo::ShapeType::Solid) return fail(OperandDecline::NotSolid);

  int shells = 0;
  for (topo::Explorer sh(s, topo::ShapeType::Shell); sh.more(); sh.next()) ++shells;
  if (shells != 1) return fail(OperandDecline::MultiShell);

  FreeformOperand op;
  op.solid = s;
  for (topo::Explorer fx(s, topo::ShapeType::Face); fx.more(); fx.next()) {
    OperandDecline faceWhy = OperandDecline::Ok;
    auto of = readmit::admitFaceReadmit(fx.current(), faceWhy);
    if (!of) return fail(faceWhy);
    const bool isFreeform = of->role == FaceRole::Freeform;
    const std::size_t idx = op.faces.size();
    op.faces.push_back(std::move(*of));
    (isFreeform ? op.freeform : op.analytic).push_back(idx);
  }
  if (op.freeform.empty()) return fail(OperandDecline::NoFreeformFace);
  if (!readmit::watertightForReadmit(s)) return fail(OperandDecline::NotWatertight);

  op.watertight = true;
  op.bbox = fodetail::foldAabb(s);
  if (why) *why = OperandDecline::Ok;
  return op;
}

namespace rdmdetail {

using ffcdetail::collectAnalyticByMembership;
using ffcdetail::pickByMembership;
using ffcdetail::rekeyToB;
using ffcdetail::subFaceHasMembership;

/// The Bézier `FaceSurface` behind an OperandFace freeform wall (the split/trace input).
inline const topo::FaceSurface* bezierWallSurface(const OperandFace& wall) {
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) return nullptr;
  if (srf->surface->kind != topo::FaceSurface::Kind::Bezier || srf->surface->poles.empty())
    return nullptr;
  return &*srf->surface;
}

/// Trace `acc`'s freeform walls against `operand`'s single wall; return the index (into
/// `acc.freeform`) of the ONE wall that yields a usable CLOSED seam, plus that seam keyed
/// on the acc wall. `nullopt` if NONE or MORE THAN ONE acc wall shares a closed seam (the
/// re-admit path handles only the single-participating-wall pose; anything else is an
/// honest decline, never a wrong pick).
struct SeamPick {
  std::size_t accWallIdx = 0;  ///< index into acc.freeform
  ssi::WLine seam;             ///< closed seam, (u1,v1) on the acc wall
};
inline std::optional<SeamPick> pickSeamSharingWall(const FreeformOperand& acc,
                                                   const topo::FaceSurface& opWall) {
  const ssi::SurfaceAdapter adOp =
      ssi::makeBezierAdapter(opWall.poles, opWall.nPolesU, opWall.nPolesV);
  std::optional<SeamPick> found;
  int nSharing = 0;
  for (std::size_t k = 0; k < acc.freeform.size(); ++k) {
    const topo::FaceSurface* fs = bezierWallSurface(acc.faces[acc.freeform[k]]);
    if (!fs) continue;
    const ssi::SurfaceAdapter adW = ssi::makeBezierAdapter(fs->poles, fs->nPolesU, fs->nPolesV);
    const ssi::TraceSet tr = ssi::trace_intersection(adW, adOp);
    for (const ssi::WLine& w : tr.lines) {
      if (w.points.size() < 3 || w.status != ssi::TraceStatus::Closed) continue;
      ++nSharing;
      SeamPick p;
      p.accWallIdx = k;
      p.seam = w;
      found = p;
      break;
    }
  }
  if (nSharing != 1) return std::nullopt;  // exactly-one-participating-wall pose only
  return found;
}

/// Membership `want` of a whole (unsplit) freeform wall in `other`'s mesh — the
/// hole-respecting interior-rep vote, reused byte-identically. Used to CARRY THROUGH
/// acc's NON-participating freeform walls (they do not cross `other`, so they are wholly
/// In or wholly Out).
inline bool wallHasMembership(const OperandFace& wall, const tess::Mesh& other,
                              const Aabb& otherBox, double deflection, Membership want) {
  return subFaceHasMembership(wall.face, other, otherBox, deflection, want);
}

/// Group-aware weld + orientation-coherence repair over an arbitrary survivor face set.
/// Reuses `nsbdetail::weldGroupCoherent`'s repair strategy (identity / last-flip / whole-
/// B-group flip / whole-A-group flip), taking `nFromAcc` = the count of leading acc faces
/// as the group boundary. Returns the coherent (result, mesh) + the residual boundary-edge
/// witness, or nullopt (never a leaky solid).
inline std::optional<std::pair<topo::Shape, tess::Mesh>> weldReadmit(
    std::vector<topo::Shape> faces, std::size_t nFromAcc, const tess::MeshParams& mp,
    std::size_t& minBoundaryEdges) {
  return nsbdetail::weldGroupCoherent(std::move(faces), nFromAcc, mp, minBoundaryEdges);
}

/// Does ANY hole-respecting interior representative point of `inner`'s boundary faces vote
/// STRICTLY `side` (In / Out) w.r.t. `outer`'s mesh? Reuses `subFaceInteriorReps` +
/// `classifyPointInMesh` byte-identically. `On`/`Unknown` samples abstain. This is the
/// primitive under the REDUNDANT-operand short-circuit's set-containment witnesses, and —
/// crucially — it is COINCIDENCE-TOLERANT: a wall of `inner` that lies exactly ON `outer`'s
/// boundary (the idempotent-fold pose: the re-applied operand's wall coincides with an
/// existing wall) votes `On`, contributing NO strict-side vote, so it does not spuriously
/// read as an escape/overlap.
inline bool anyRepOnSide(const FreeformOperand& inner, const tess::Mesh& outer,
                         const Aabb& outerBox, double deflection, Membership side) {
  for (const OperandFace& f : inner.faces)
    for (const math::Point3& p : ffcdetail::subFaceInteriorReps(f.face)) {
      const Membership m = classifyPointInMesh(outer, outerBox, deflection, p);
      if (m == side) return true;
    }
  return false;
}

/// `inner ⊆ outer` witness: NO interior rep of any `inner` boundary face escapes `outer`
/// (none votes strictly Out) AND at least one rep is usable (In or On). Sound because a
/// closed `inner` whose entire boundary is In-or-On a closed `outer` is contained; a face
/// that dips outside would have an Out rep. Coincidence-tolerant (a shared wall reads On).
inline bool innerSubsetOuter(const FreeformOperand& inner, const tess::Mesh& outer,
                             const Aabb& outerBox, double deflection) {
  if (anyRepOnSide(inner, outer, outerBox, deflection, Membership::Out)) return false;
  // require at least one usable (In or On) rep so an empty/degenerate sampling that produced
  // no votes at all is not a false subset.
  return anyRepOnSide(inner, outer, outerBox, deflection, Membership::In) ||
         anyRepOnSide(inner, outer, outerBox, deflection, Membership::On);
}

/// `inner ∩ outer` interiors are DISJOINT witness: NO interior rep of any `inner` boundary
/// face is strictly In `outer` (inner does not dip into outer's material). Paired with the
/// symmetric call it certifies mutual interior-disjointness (a redundant CUT tool).
inline bool innerDisjointOuter(const FreeformOperand& inner, const tess::Mesh& outer,
                               const Aabb& outerBox, double deflection) {
  return !anyRepOnSide(inner, outer, outerBox, deflection, Membership::In);
}

/// REDUNDANT-OPERAND SHORT-CIRCUIT (exact, membership-only, no weld). A fold often re-applies
/// an operand that adds/removes nothing; the sound, DISAGREED-safe answer is `acc` UNCHANGED
/// (no geometry synthesised, so no leaky/wrong solid is even possible):
///   * UNION  — `operand ⊆ acc`  ⇒ A ∪ B = A.
///   * CUT    — `operand ∩ acc = ∅` (interiors disjoint) ⇒ A − B = A.
///   * COMMON — `acc ⊆ operand`  ⇒ A ∩ B = A.
/// Returns `acc.solid` when a redundant witness holds AND (when supplied) the closed-form
/// volume band accepts it; `nullopt` otherwise (fall through to the weld path). The witnesses
/// are COINCIDENCE-TOLERANT and a genuinely-overlapping operand always fails them, so it is
/// never mis-taken as redundant.
inline std::optional<topo::Shape> redundantOperandResult(SolidBoolOp op, const FreeformOperand& acc,
                                                         const FreeformOperand& opnd,
                                                         const tess::Mesh& meshAcc,
                                                         const tess::Mesh& meshOp, const Aabb& bbAcc,
                                                         const Aabb& bbOp, double deflection,
                                                         double analyticOpVolume) {
  bool redundant = false;
  if (op == SolidBoolOp::Fuse) redundant = innerSubsetOuter(opnd, meshAcc, bbAcc, deflection);
  else if (op == SolidBoolOp::Common) redundant = innerSubsetOuter(acc, meshOp, bbOp, deflection);
  else redundant = innerDisjointOuter(opnd, meshAcc, bbAcc, deflection) &&
                   innerDisjointOuter(acc, meshOp, bbOp, deflection);
  if (!redundant) return std::nullopt;
  const double vAcc = std::fabs(tess::enclosedVolume(meshAcc));
  if (!nsbdetail::analyticVolumeBandOk(vAcc, analyticOpVolume, deflection)) return std::nullopt;
  return acc.solid;
}

}  // namespace rdmdetail

// ─────────────────────────────────────────────────────────────────────────────
// The survivor-selection membership targets per op, for the ACC side (the split
// participating wall + the carried-through whole walls + analytic lids) and the OPERAND
// side (the split participating wall). This mirrors the binary boolean's per-op survivor
// sets exactly, generalised to a multi-wall acc:
//   * FUSE  (A∪B): keep everything OUTSIDE the other operand on BOTH sides.
//   * CUT   (A−B): keep acc OUTSIDE B; keep the operand wall's INSIDE-A cap (the new
//                  ceiling). Non-participating acc walls OUTSIDE B are carried; operand's
//                  lids are NOT kept (the tool contributes only its in-A cap).
//   * COMMON(A∩B): keep acc INSIDE B and operand INSIDE A (the lens).
// ─────────────────────────────────────────────────────────────────────────────
namespace rdmdetail {

/// Assemble the survivor face set for the re-admit boolean. Returns false (ambiguous /
/// open) on any unresolved membership — never a wrong pick. `nFromAcc` (out) marks the
/// weld's acc/operand group boundary. Isolated to keep the driver a flat guard pass.
inline bool selectReadmitSurvivors(SolidBoolOp op, const FreeformOperand& acc,
                                   const FreeformOperand& opnd, std::size_t accWallIdx,
                                   const SmoothFaceSplit& splitAcc, const SmoothFaceSplit& splitOp,
                                   const tess::Mesh& meshAcc, const tess::Mesh& meshOp,
                                   const Aabb& bbAcc, const Aabb& bbOp, double deflection,
                                   std::vector<topo::Shape>& faces, std::size_t& nFromAcc) {
  const Membership accWant = op == SolidBoolOp::Common ? Membership::In : Membership::Out;
  const Membership opWant =
      op == SolidBoolOp::Common ? Membership::In : op == SolidBoolOp::Cut ? Membership::In
                                                                          : Membership::Out;
  // (a) the split participating acc wall.
  const auto accKeep = pickByMembership(splitAcc, meshOp, bbOp, deflection, accWant);
  if (!accKeep) return false;
  faces.push_back(*accKeep);
  // (b) acc's NON-participating freeform walls, carried whole by their membership.
  for (std::size_t k = 0; k < acc.freeform.size(); ++k) {
    if (k == accWallIdx) continue;
    const OperandFace& wall = acc.faces[acc.freeform[k]];
    if (wallHasMembership(wall, meshOp, bbOp, deflection, accWant)) faces.push_back(wall.face);
  }
  // (c) acc's analytic lids, by membership.
  collectAnalyticByMembership(acc, meshOp, bbOp, deflection, accWant, faces);
  nFromAcc = faces.size();
  // (d) the split participating operand wall.
  const auto opKeep = pickByMembership(splitOp, meshAcc, bbAcc, deflection, opWant);
  if (!opKeep) return false;
  faces.push_back(*opKeep);
  // (e) for FUSE, the operand also contributes its OUTSIDE-acc lids + non-participating
  // walls (the pristine operand has one wall, so only its lids). CUT/COMMON take only the
  // operand's participating cap (the tool contributes its in-acc surface only).
  if (op == SolidBoolOp::Fuse) collectAnalyticByMembership(opnd, meshAcc, bbAcc, deflection,
                                                           Membership::Out, faces);
  return true;
}

}  // namespace rdmdetail

// ─────────────────────────────────────────────────────────────────────────────
// nurbsSolidBooleanReadmit — the RE-ADMITTING two-operand boolean. `acc` may be a binary
// boolean's MULTI-freeform-wall output (a holed-annulus-walled solid); `operand` is a
// pristine single-freeform-wall solid (the next fold entry). Returns the welded watertight
// result or an HONEST decline to NULL with a measured `SolidBoolReport`. When `acc` is
// itself a pristine single-wall operand, DEFERS bit-identically to `nurbsSolidBoolean` (so
// the FIRST fold step is unregressed).
//
// `analyticOpVolume` (optional) gates the FINAL weld's TWO-SIDED volume band.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape nurbsSolidBooleanReadmit(const topo::Shape& acc, const topo::Shape& operand,
                                            SolidBoolOp op, double deflection = 0.005,
                                            SolidBoolReport* report = nullptr,
                                            double analyticOpVolume =
                                                std::numeric_limits<double>::quiet_NaN()) {
  using namespace rdmdetail;
  SolidBoolReport rep;
  rep.op = op;
  auto emit = [&](topo::Shape s) -> topo::Shape {
    if (report) *report = rep;
    return s;
  };
  auto fail = [&](SolidBoolDecline d) -> topo::Shape { rep.decline = d; return emit({}); };

  // (0) If `acc` is a PRISTINE single-freeform-wall operand (the first fold step), defer
  // bit-identically to the frozen binary boolean — 2-operand folds stay UNREGRESSED.
  if (recogniseFreeformSolid(acc)) return nurbsSolidBoolean(acc, operand, op, deflection, report,
                                                            analyticOpVolume);

  // (1) Re-admit `acc` (holed / multi-wall) + recognise the pristine `operand`.
  const auto foAcc = recogniseFreeformSolidReadmit(acc);
  if (!foAcc) return fail(SolidBoolDecline::NotAdmittedA);
  const auto foOp = recogniseFreeformSolid(operand);
  if (!foOp) return fail(SolidBoolDecline::NotAdmittedB);
  if (foOp->freeform.size() != 1) return fail(SolidBoolDecline::WallUnusableB);
  const OperandFace& opWallFace = foOp->faces[foOp->freeform.front()];
  const topo::FaceSurface* opWall = bezierWallSurface(opWallFace);
  if (!opWall) return fail(SolidBoolDecline::WallUnusableB);

  // Pre-op meshes (membership + volume bounds) — needed by both the redundant-operand
  // short-circuit and the weld path.
  tess::MeshParams mp;
  mp.deflection = deflection;
  const tess::Mesh meshAcc = tess::SolidMesher(mp).mesh(foAcc->solid);
  const tess::Mesh meshOp = tess::SolidMesher(mp).mesh(foOp->solid);
  if (!tess::isWatertight(meshAcc) || !tess::isWatertight(meshOp))
    return fail(SolidBoolDecline::NotWatertight);
  const Aabb bbAcc = meshAabb(meshAcc), bbOp = meshAabb(meshOp);

  // (1a) REDUNDANT-OPERAND SHORT-CIRCUIT — a re-applied operand that adds/removes nothing
  // resolves to `acc` UNCHANGED, exactly, membership-only (see `redundantOperandResult`).
  // A genuinely-overlapping operand fails the witness and falls through to the weld path.
  if (const auto sc = redundantOperandResult(op, *foAcc, *foOp, meshAcc, meshOp, bbAcc, bbOp,
                                             deflection, analyticOpVolume)) {
    rep.decline = SolidBoolDecline::Ok;
    nsbdetail::recordMeshWitnesses(rep, *sc, deflection);
    return emit(*sc);
  }

  // (2) Find the ONE acc wall that shares a usable closed seam with the operand wall.
  const auto pick = pickSeamSharingWall(*foAcc, *opWall);
  if (!pick) return fail(SolidBoolDecline::NoSeam);
  const OperandFace& accWallFace = foAcc->faces[foAcc->freeform[pick->accWallIdx]];

  // (3) Split BOTH participating walls by the shared seam. NOTE (MEASURED re-admission
  // boundary): when the participating acc wall is an ALREADY-HOLED annulus (the seam of a
  // PRIOR boolean is an interior hole), splitting it by this SECOND seam is a MULTI-HOLE /
  // multi-crossing face split — `splitFaceSmoothTrim` treats the face as simply-connected
  // and does NOT preserve the existing hole, so the OUT/IN sub-faces are geometrically
  // incomplete and the downstream weld honest-declines `NotWatertight` (never leaky). This
  // is the readiness doc's UNLANDED §4 multi-crossing split; the general genuine-overlap
  // ≥3-operand weld is gated on it. The REDUNDANT-operand short-circuit above handles the
  // reachable idempotent folds without ever reaching this split.
  const SmoothSplitResult srAcc = splitFaceSmoothTrim(accWallFace.face, pick->seam);
  const SmoothSplitResult srOp = splitFaceSmoothTrim(opWallFace.face, rekeyToB(pick->seam));
  if (!srAcc.ok() || !srOp.ok()) return fail(SolidBoolDecline::SplitFailed);

  // (4) Select survivors (multi-wall aware) — acc split + carried whole walls + lids, then
  // the operand split cap (+ operand lids for FUSE).
  std::vector<topo::Shape> faces;
  std::size_t nFromAcc = 0;
  if (!selectReadmitSurvivors(op, *foAcc, *foOp, pick->accWallIdx, *srAcc.split, *srOp.split,
                              meshAcc, meshOp, bbAcc, bbOp, deflection, faces, nFromAcc))
    return fail(SolidBoolDecline::ClassifyAmbiguous);
  if (faces.size() < 2) return fail(SolidBoolDecline::WeldOpen);
  rep.survivorFaces = static_cast<int>(faces.size());

  // (5) Weld + group-aware coherence repair + mandatory self-verify.
  std::size_t minBE = 1;
  const auto welded = weldReadmit(std::move(faces), nFromAcc, mp, minBE);
  if (!welded) { rep.boundaryEdges = minBE; return fail(SolidBoolDecline::NotWatertight); }
  const tess::Mesh& m = welded->second;
  rep.watertight = tess::isWatertight(m);
  rep.coherent = tess::isConsistentlyOriented(m);
  rep.boundaryEdges = tess::boundaryEdgeCount(m);
  const double v = std::fabs(tess::enclosedVolume(m));
  rep.enclosedVolume = v;
  if (!(v > 0.0) || std::isnan(v)) return fail(SolidBoolDecline::VolumeInconsistent);
  if (!nsbdetail::analyticVolumeBandOk(v, analyticOpVolume, deflection))
    return fail(SolidBoolDecline::VolumeInconsistent);

  rep.decline = SolidBoolDecline::Ok;
  return welded->first;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_BOOLEAN_READMIT_H

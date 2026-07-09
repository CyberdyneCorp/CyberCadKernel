// SPDX-License-Identifier: Apache-2.0
//
// freeform_freeform_cut.h — MOAT M2 freeform↔freeform CLOSED-SEAM CUT / COMMON: the
// FIRST boolean whose SHARED seam has a CURVED sub-face on BOTH sides.
//
// ── ROLE (the genuine curved↔curved weld the M0w seam pin unblocks) ───────────────
// Every landed M2 freeform boolean welds a freeform operand against an ANALYTIC cutter,
// so exactly ONE side of the shared seam is curved: `half_space_cut.h` cuts a freeform
// wall by an infinite PLANE; `curved_wall_cut.h` cuts a freeform wall by a horizontal
// PLANE along a CLOSED circular seam (the OTHER side is the flat cap); `two_operand.h`
// welds a freeform operand against an all-PLANAR box. This verb welds the genuine
// freeform↔freeform case: the CUT (`A − B`) and COMMON (`A ∩ B`) of TWO coaxial curved
// bowl-cup operands whose two curved walls meet in ONE CLOSED curved seam, so BOTH sides
// of the seam are CURVED sub-faces. This is exactly the fragility the just-landed M0w
// topology-aware closed-inner-seam tessellator weld resolves (a shared CLOSED
// straight-chord seam between two CURVED sub-faces welds watertight at any deflection),
// only previously exercised curved↔flat.
//
// ── THE REACHABLE POSE ────────────────────────────────────────────────────────────
//   * `A` = an UP bowl-cup: bowl z_A = a·r² (opens up) + a flat TOP lid at z = a·R².
//   * `B` = a DOWN dome-cup: dome z_B = H − a·r² (opens down) + a flat BOTTOM lid at
//     z = H − a·R², placed so its lid sits BELOW `A`'s apex and its apex BELOW `A`'s lid
//     (so `B` slices `A` ONLY through the two curved walls).
// `A.wall ∩ B.wall` is ONE CLOSED CIRCLE at r = ρ = √(H/2a), z = H/2 — the shared
// closed curved seam. The verb:
//
//   1. RECOGNISE — B1 admits both operands (each ONE freeform wall + ONE analytic lid).
//   2. TRACE — M1 `ssi::trace_intersection` over the two `makeBezierAdapter` surfaces →
//      the CLOSED seam WLine, carrying `(u1,v1)` on `A`'s wall and `(u2,v2)` on `B`'s.
//   3. SPLIT BOTH WALLS — byte-frozen B2 smooth-trim `splitFaceSmoothTrim`: `A`'s wall by
//      the `(u1,v1)` pcurve; `B`'s wall by a `(u2,v2)`-re-keyed copy of the SAME seam.
//      Each → a disk (the seam-enclosed cap) + an annulus.
//   4. SELECT SURVIVORS — by B3 membership of each sub-face's centroid in the OTHER
//      operand's mesh:
//        * CUT   — `A`'s sub-face(s) OUTSIDE `B` (the annulus + `A`'s lid) ∪ `B`'s wall
//          sub-face INSIDE `A` (the disk — the new curved ceiling of the carved lens).
//        * COMMON— `A`'s wall sub-face INSIDE `B` (`A`'s disk cap) ∪ `B`'s wall sub-face
//          INSIDE `A` (`B`'s disk cap) — the lens, a curved-cap-to-curved-cap solid.
//   5. WELD + SELF-VERIFY — M0 `SolidMesher`; require WATERTIGHT and a consistent
//      op-volume bound. ANY decline → NULL Shape (→ OCCT `BRepAlgoAPI_{Cut,Common}`).
//      NEVER a leaky/partial/wrong solid; NO tolerance widened.
//
// ── CONSUMES (byte-identical, never rewritten) ────────────────────────────────────
// B1 `recogniseFreeformSolid`, M1 `ssi::trace_intersection` + `makeBezierAdapter`, B2
// SMOOTH-TRIM `splitFaceSmoothTrim`, B3 `classifyPointInMesh` / `meshAabb`, M0
// `SolidMesher`/`isWatertight`/`enclosedVolume`, the tessellate evaluators. Additive
// sibling — touches NONE of them, nor `curvedWallHalfSpaceCut`, nor `two_operand.h`.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_CUT_H
#define CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_CUT_H

#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/smooth_trim_split.h"
#include "native/math/native_math.h"
#include "native/ssi/marching.h"
#include "native/ssi/seeding.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace ssi  = cybercad::native::ssi;

/// The requested freeform↔freeform operator.
enum class FfOp { Cut, Common };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight result solid is returned.
enum class FfCutDecline {
  Ok,
  NotAdmittedA,        ///< B1 declined operand `A`
  NotAdmittedB,        ///< B1 declined operand `B`
  MultiFreeformFace,   ///< an operand does not have EXACTLY one freeform wall
  WallSurfaceUnusable, ///< a freeform wall is not a Bézier with poles
  SeamUnusable,        ///< M1 seam missing / < 3 nodes / not a closed loop
  SmoothSplitFailedA,  ///< B2 smooth-trim declined `A`'s wall split
  SmoothSplitFailedB,  ///< B2 smooth-trim declined `B`'s wall split
  ClassifyAmbiguous,   ///< B3: a survivor sub-face is On/Unknown or on the wrong side
  WeldOpen,            ///< fewer than two survivor faces (cannot bound a solid)
  NotWatertight,       ///< self-verify: the welded result is not a closed 2-manifold
  VolumeInconsistent   ///< self-verify: the enclosed volume is non-positive / off the bound
};

inline const char* ffCutDeclineName(FfCutDecline d) noexcept {
  switch (d) {
    case FfCutDecline::Ok: return "Ok";
    case FfCutDecline::NotAdmittedA: return "NotAdmittedA";
    case FfCutDecline::NotAdmittedB: return "NotAdmittedB";
    case FfCutDecline::MultiFreeformFace: return "MultiFreeformFace";
    case FfCutDecline::WallSurfaceUnusable: return "WallSurfaceUnusable";
    case FfCutDecline::SeamUnusable: return "SeamUnusable";
    case FfCutDecline::SmoothSplitFailedA: return "SmoothSplitFailedA";
    case FfCutDecline::SmoothSplitFailedB: return "SmoothSplitFailedB";
    case FfCutDecline::ClassifyAmbiguous: return "ClassifyAmbiguous";
    case FfCutDecline::WeldOpen: return "WeldOpen";
    case FfCutDecline::NotWatertight: return "NotWatertight";
    case FfCutDecline::VolumeInconsistent: return "VolumeInconsistent";
  }
  return "?";
}

namespace ffcdetail {

/// The freeform wall's `FaceSurface` for a recognised single-freeform-wall operand, or
/// nullptr (with `why`) if the operand does not present exactly one usable Bézier wall.
inline const topo::FaceSurface* freeformWall(const FreeformOperand& op, const OperandFace** face,
                                             FfCutDecline& why) {
  if (op.freeform.size() != 1) { why = FfCutDecline::MultiFreeformFace; return nullptr; }
  const OperandFace& wall = op.faces[op.freeform.front()];
  const auto srf = topo::surfaceOf(wall.face);
  if (!srf || !srf->surface) { why = FfCutDecline::WallSurfaceUnusable; return nullptr; }
  const topo::FaceSurface& fs = *srf->surface;
  if (fs.kind != topo::FaceSurface::Kind::Bezier || fs.poles.empty()) {
    why = FfCutDecline::WallSurfaceUnusable;
    return nullptr;
  }
  *face = &wall;
  return &fs;
}

/// Trace the shared seam `A.wall ∩ B.wall` as a CLOSED WLine, keyed with `(u1,v1)` on A
/// and `(u2,v2)` on B. Returns the single closed branch, or an empty WLine.
inline ssi::WLine traceSharedSeam(const topo::FaceSurface& fsA, const topo::FaceSurface& fsB) {
  const ssi::SurfaceAdapter A = ssi::makeBezierAdapter(fsA.poles, fsA.nPolesU, fsA.nPolesV);
  const ssi::SurfaceAdapter B = ssi::makeBezierAdapter(fsB.poles, fsB.nPolesU, fsB.nPolesV);
  const ssi::TraceSet tr = ssi::trace_intersection(A, B);
  const ssi::WLine* best = nullptr;
  for (const ssi::WLine& w : tr.lines) {
    if (w.points.size() < 3) continue;
    if (w.status == ssi::TraceStatus::Closed) return w;
    if (!best || w.points.size() > best->points.size()) best = &w;
  }
  return best ? *best : ssi::WLine{};
}

/// Re-key a seam WLine so its `(u1,v1)` carry `B`'s params — `splitFaceSmoothTrim` reads
/// `points[i].{u1,v1}` as the pcurve on the face it is splitting, so `B`'s wall must be
/// split by a seam whose primary params are `B`'s `(u2,v2)`.
inline ssi::WLine rekeyToB(const ssi::WLine& seam) {
  ssi::WLine out = seam;
  for (ssi::WLinePoint& p : out.points) {
    p.u1 = p.u2;
    p.v1 = p.v2;
  }
  return out;
}

/// The 3-D centroid of a (trimmed) sub-face: the surface value at its flattened
/// outer-loop UV centroid (an in-domain (u,v) for the disk/annulus sub-faces the
/// smooth-trim split lays). Mirrors `todetail::faceCentroid3d`.
inline std::optional<math::Point3> subFaceCentroid3d(const topo::Shape& face) {
  const auto srf = topo::surfaceOf(face);
  if (!srf || !srf->surface) return std::nullopt;
  const tess::UVRegion reg = tess::buildRegion(face, 16);
  if (!reg.hasOuter()) return std::nullopt;
  double su = 0, sv = 0;
  for (const tess::UV& q : reg.outer) { su += q.u; sv += q.v; }
  const double inv = 1.0 / static_cast<double>(reg.outer.size());
  const tess::SurfaceEvaluator ev(*srf->surface, srf->location);
  return ev.value(su * inv, sv * inv);
}

/// Pick the sub-face of a smooth-trim split whose centroid has the requested membership
/// in `other`'s mesh (`In` or `Out`). Returns nullopt (→ ClassifyAmbiguous) if NEITHER
/// sub-face confirms the wanted side, or a centroid is unusable / On / Unknown.
inline std::optional<topo::Shape> pickByMembership(const SmoothFaceSplit& split,
                                                   const tess::Mesh& other, const Aabb& otherBox,
                                                   double deflection, Membership want) {
  const topo::Shape* cands[2] = {&split.faceInside, &split.faceOutside};
  for (const topo::Shape* c : cands) {
    const auto ctr = subFaceCentroid3d(*c);
    if (!ctr) continue;
    if (classifyPointInMesh(other, otherBox, deflection, *ctr) == want) return *c;
  }
  return std::nullopt;
}

/// Every analytic (lid) face of `op` whose centroid has the requested membership in
/// `other`'s mesh. For CUT the lids OUTSIDE the other operand are kept whole.
inline void collectAnalyticByMembership(const FreeformOperand& op, const tess::Mesh& other,
                                        const Aabb& otherBox, double deflection, Membership want,
                                        std::vector<topo::Shape>& out) {
  for (std::size_t idx : op.analytic) {
    const auto ctr = subFaceCentroid3d(op.faces[idx].face);
    if (!ctr) continue;
    if (classifyPointInMesh(other, otherBox, deflection, *ctr) == want) out.push_back(op.faces[idx].face);
  }
}

}  // namespace ffcdetail

// ─────────────────────────────────────────────────────────────────────────────
// freeformFreeformClosedSeamCut — the entry point. `A`,`B` are freeform bowl-cup
// operands whose curved walls meet in ONE CLOSED curved seam. Returns the welded,
// self-verified CUT (`A − B`) or COMMON (`A ∩ B`) solid, or a NULL Shape (→ OCCT) with
// a measured decline. Never emits a leaky/partial/wrong solid; no tolerance widened.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformFreeformClosedSeamCut(const topo::Shape& A, const topo::Shape& B,
                                                 FfOp op, double deflection = 0.005,
                                                 FfCutDecline* why = nullptr) {
  using namespace ffcdetail;
  auto fail = [&](FfCutDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  // (1) B1 recognise both operands.
  const auto foA = recogniseFreeformSolid(A);
  if (!foA) return fail(FfCutDecline::NotAdmittedA);
  const auto foB = recogniseFreeformSolid(B);
  if (!foB) return fail(FfCutDecline::NotAdmittedB);

  const OperandFace* wallA = nullptr;
  const OperandFace* wallB = nullptr;
  FfCutDecline wA = FfCutDecline::Ok, wB = FfCutDecline::Ok;
  const topo::FaceSurface* fsA = freeformWall(*foA, &wallA, wA);
  if (!fsA) return fail(wA);
  const topo::FaceSurface* fsB = freeformWall(*foB, &wallB, wB);
  if (!fsB) return fail(wB);

  // (2) M1 trace the shared CLOSED curved seam (u1,v1 on A; u2,v2 on B).
  const ssi::WLine seam = traceSharedSeam(*fsA, *fsB);
  if (seam.points.size() < 3 || seam.status != ssi::TraceStatus::Closed)
    return fail(FfCutDecline::SeamUnusable);

  // (3) B2 smooth-trim split BOTH walls by the SAME 3-D seam.
  const SmoothSplitResult srA = splitFaceSmoothTrim(wallA->face, seam);
  if (!srA.ok()) return fail(FfCutDecline::SmoothSplitFailedA);
  const SmoothSplitResult srB = splitFaceSmoothTrim(wallB->face, rekeyToB(seam));
  if (!srB.ok()) return fail(FfCutDecline::SmoothSplitFailedB);

  // Pre-cut operand meshes (B3 membership + independent op-volume bounds).
  tess::MeshParams mp; mp.deflection = deflection;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  if (!tess::isWatertight(meshA) || !tess::isWatertight(meshB))
    return fail(FfCutDecline::NotWatertight);
  const Aabb bbA = meshAabb(meshA), bbB = meshAabb(meshB);

  // (4) select survivors by B3 membership.
  std::vector<topo::Shape> faces;
  if (op == FfOp::Cut) {
    // A − B: A's wall sub-face OUTSIDE B (the annulus) + A's lids OUTSIDE B + B's wall
    // sub-face INSIDE A (the new curved ceiling of the carved lens).
    const auto aKeep = pickByMembership(*srA.split, meshB, bbB, deflection, Membership::Out);
    const auto bKeep = pickByMembership(*srB.split, meshA, bbA, deflection, Membership::In);
    if (!aKeep || !bKeep) return fail(FfCutDecline::ClassifyAmbiguous);
    faces.push_back(*aKeep);
    faces.push_back(*bKeep);
    collectAnalyticByMembership(*foA, meshB, bbB, deflection, Membership::Out, faces);
  } else {
    // A ∩ B: A's wall sub-face INSIDE B (A's disk cap) + B's wall sub-face INSIDE A
    // (B's disk cap) — the lens (curved cap to curved cap across the shared seam).
    const auto aKeep = pickByMembership(*srA.split, meshB, bbB, deflection, Membership::In);
    const auto bKeep = pickByMembership(*srB.split, meshA, bbA, deflection, Membership::In);
    if (!aKeep || !bKeep) return fail(FfCutDecline::ClassifyAmbiguous);
    faces.push_back(*aKeep);
    faces.push_back(*bKeep);
  }
  if (faces.size() < 2) return fail(FfCutDecline::WeldOpen);

  // (5) weld → Solid + mandatory self-verify (watertight + a consistent op-volume).
  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape result = topo::ShapeBuilder::makeSolid({shell});
  const tess::Mesh m = tess::SolidMesher(mp).mesh(result);
  if (!tess::isWatertight(m)) return fail(FfCutDecline::NotWatertight);
  const double v = std::fabs(tess::enclosedVolume(m));
  if (!(v > 0.0) || std::isnan(v)) return fail(FfCutDecline::VolumeInconsistent);

  // op-volume bound: CUT ⊂ A (0 < V ≤ V(A)); COMMON ⊂ A and ⊂ B (0 < V ≤ min(V(A),V(B))).
  const double vA = std::fabs(tess::enclosedVolume(meshA));
  const double vB = std::fabs(tess::enclosedVolume(meshB));
  const double tol = 0.05 * std::max(std::max(vA, vB), 1e-12);
  const double upper = op == FfOp::Cut ? vA : std::min(vA, vB);
  if (v > upper + tol) return fail(FfCutDecline::VolumeInconsistent);

  if (why) *why = FfCutDecline::Ok;
  return result;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_FREEFORM_FREEFORM_CUT_H

// SPDX-License-Identifier: Apache-2.0
//
// two_operand.h — MOAT M2-FUSE: the FIRST end-to-end *two-operand* freeform boolean.
// Composes the landed verbs into a self-verified FUSE / CUT / COMMON of a bowl-lidded
// freeform prism `A` against a FINITE all-planar operand `B` (an axis-aligned box),
// for the single-curved-cut pose (design §1) — or returns a NULL `Shape` (→ OCCT).
//
// ── WHAT THIS ADDS OVER THE LANDED SINGLE-OPERAND CUT ─────────────────────────────
// `half_space_cut.h` cuts `A` by an INFINITE planar half-space (one seam, one cap).
// Here the second operand `B` is FINITE, so:
//
//   * FUSE (`A ∪ B`) — the GENUINE new two-operand weld. Survivors are
//     `A`-outside-`B` (the landed `A`-keep faces, WITHOUT the cap — the cap region is
//     now interior to the union) ∪ `B`-outside-`A` (`B`'s non-cutting faces WHOLE, plus
//     `B`'s cutting face `Pcut` as a rectangle-minus-`D` ANNULUS whose curved hole IS
//     the shared inter-solid seam from `inter_solid_seam.h`). Welded, meshed, and
//     admitted ONLY if watertight and at the closed-form union volume.
//
//   * CUT (`A − B`) / COMMON (`A ∩ B`) — for THIS pose (`B` contains `A`'s
//     interior-side material, `B`'s only cut of `A` is the single curved `Pcut`) the
//     two-operand result reduces EXACTLY to the landed single-operand half-space cut of
//     `A` by `Pcut`'s plane: `A − B = A ∩ (outside Pcut)` and `A ∩ B = A ∩ (inside
//     Pcut)`. That reduction is a THEOREM of the pose (the containment guard in
//     `buildInterSolidSeam` establishes its precondition), not a shortcut, so CUT/COMMON
//     delegate to the LANDED, already-self-verified `freeformHalfSpaceCut`. This is not
//     dead code: both are exercised against their closed-form values by the host gate.
//
// ── SELF-VERIFY → OCCT FALLBACK (load-bearing) ────────────────────────────────────
// The welded FUSE is admitted ONLY if the M0 mesh is WATERTIGHT (every edge shared by
// exactly two triangles) AND its enclosed volume is a consistent union volume
// (`max(V(A),V(B)) ≤ V ≤ V(A)+V(B)`). Membership of the clean `A`/`B` fragments is
// CONFIRMED (never guessed) via B3 `classifyPointInMesh`; an `On`/`Unknown`/wrong-side
// verdict DECLINES. Any failure returns a NULL `Shape` → the engine falls through to
// OCCT `BRepAlgoAPI_Fuse`. No leaky/overlapping/wrong-volume solid is EVER emitted; no
// tolerance is weakened. (The strong mesh-free closed-form union-volume oracle lives in
// the host gate, which has the fixture's exact bowl-over-box integrand.)
//
// ── CONSUMES (byte-identical) ─────────────────────────────────────────────────────
// B1 `recogniseFreeformSolid`, `buildInterSolidSeam`, B3 `classifyPointInMesh`, M0
// `SolidMesher`/`isWatertight`, `assemble.h` VertexPool + `triangulatePolygonToFaces`,
// the landed `freeformHalfSpaceCut`, and every `hscdetail::` primitive. Additive
// sibling; edits none of them.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_BOOLEAN_TWO_OPERAND_H
#define CYBERCAD_NATIVE_BOOLEAN_TWO_OPERAND_H

#include "native/boolean/assemble.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/boolean/half_space_cut.h"
#include "native/boolean/inter_solid_seam.h"
#include "native/boolean/polygon.h"
#include "native/math/native_math.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/tessellate/surface_eval.h"
#include "native/tessellate/trim.h"
#include "native/topology/native_topology.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace cybercad::native::boolean {

namespace topo = cybercad::native::topology;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

/// The requested two-operand operator.
enum class TwoOperandOp { Fuse, Cut, Common };

/// The measured blocker (logged before the OCCT fall-through). `Ok` iff a verified
/// watertight result solid is returned.
enum class TwoOperandDecline {
  Ok,
  NotAdmittedA,        ///< B1 declined operand `A`
  SeamDeclined,        ///< the inter-solid seam builder declined (see SeamDecline)
  WeldOpen,            ///< fewer than four survivor faces (cannot bound a solid)
  ClassifyAmbiguous,   ///< B3: a survivor fragment is On/Unknown or on the wrong side
  NotWatertight,       ///< self-verify: the welded FUSE is not a closed 2-manifold
  VolumeInconsistent,  ///< self-verify: the FUSE volume is not a valid union volume
  CutCommonDeclined    ///< the delegated landed half-space cut declined
};

inline const char* twoOperandDeclineName(TwoOperandDecline d) noexcept {
  switch (d) {
    case TwoOperandDecline::Ok: return "Ok";
    case TwoOperandDecline::NotAdmittedA: return "NotAdmittedA";
    case TwoOperandDecline::SeamDeclined: return "SeamDeclined";
    case TwoOperandDecline::WeldOpen: return "WeldOpen";
    case TwoOperandDecline::ClassifyAmbiguous: return "ClassifyAmbiguous";
    case TwoOperandDecline::NotWatertight: return "NotWatertight";
    case TwoOperandDecline::VolumeInconsistent: return "VolumeInconsistent";
    case TwoOperandDecline::CutCommonDeclined: return "CutCommonDeclined";
  }
  return "?";
}

namespace todetail {

using hscdetail::Piece;

/// The 3-D centroid of a face: the surface value at its flattened outer-loop centroid.
/// Valid for both the curved keep sub-face and the planar analytic keeps (an in-domain
/// (u,v) for a convex trim), mirroring `fodetail::faceOutwardNormal`.
inline std::optional<math::Point3> faceCentroid3d(const topo::Shape& face) {
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

/// Build `B`'s `Pcut` face as a rectangle-minus-`D` ANNULUS: the `Pcut` outer loop
/// (outer wire) minus the shared inter-solid seam (hole wire), on the `Pcut` plane,
/// oriented outward (away from `B`'s interior). The mesher normalises outer/hole
/// winding, so only the shared 3-D geometry (bit-exact via `edgeFromPiece`) matters.
inline topo::Shape buildAnnulus(const InterSolidSeam& seam) {
  const math::Ax3 frame = seam.tracePlane.pos;  // z = INTO B; Reversed ⇒ outward normal
  const Polygon& pcut = seam.bPolys[seam.pcutIdx];

  std::vector<topo::Shape> outerEdges;
  const std::size_t n = pcut.vertices.size();
  for (std::size_t k = 0; k < n; ++k)
    outerEdges.push_back(
        hscdetail::edgeFromPiece(Piece{pcut.vertices[k], pcut.vertices[(k + 1) % n]}, frame));
  const topo::Shape outerWire = topo::ShapeBuilder::makeWire(std::move(outerEdges));

  std::vector<topo::Shape> innerEdges;
  for (const Piece& pc : seam.capLoop) innerEdges.push_back(hscdetail::edgeFromPiece(pc, frame));
  const topo::Shape innerWire = topo::ShapeBuilder::makeWire(std::move(innerEdges));

  topo::FaceSurface plane{};
  plane.kind = topo::FaceSurface::Kind::Plane;
  plane.frame = frame;
  return topo::ShapeBuilder::makeFace(plane, outerWire, {innerWire}, topo::Orientation::Reversed);
}

/// Assemble the FUSE survivor faces: `A`-outer keeps ∪ `B`'s non-cutting faces (whole)
/// ∪ the `Pcut` annulus. Coincident boundaries share 3-D geometry (bit-exact seam +
/// shared box corners), so the M0 mesher position-welds the result watertight.
inline std::vector<topo::Shape> assembleFuseFaces(const InterSolidSeam& seam) {
  std::vector<topo::Shape> faces = seam.aKeepFaces;
  VertexPool pool;
  for (std::size_t i = 0; i < seam.bPolys.size(); ++i)
    if (i != seam.pcutIdx) detail::triangulatePolygonToFaces(seam.bPolys[i], pool, faces);
  faces.push_back(buildAnnulus(seam));
  return faces;
}

/// Confirm (never guess) the clean survivor fragments via B3: every `A`-keep fragment
/// is OUTSIDE `B`, and every whole `B` face is OUTSIDE `A`. On/Unknown/wrong-side → decline.
inline bool confirmMembership(const FreeformOperand& A, const InterSolidSeam& seam,
                              const tess::Mesh& meshA, const tess::Mesh& meshB, double deflection) {
  const Aabb bbA = meshAabb(meshA), bbB = meshAabb(meshB);
  for (const topo::Shape& f : seam.aKeepFaces) {
    const auto c = faceCentroid3d(f);
    if (!c || classifyPointInMesh(meshB, bbB, deflection, *c) != Membership::Out) return false;
  }
  for (std::size_t i = 0; i < seam.bPolys.size(); ++i) {
    if (i == seam.pcutIdx) continue;
    const std::vector<math::Point3>& v = seam.bPolys[i].vertices;
    math::Point3 c{0, 0, 0};
    for (const math::Point3& p : v) c = c + p.asVec() / static_cast<double>(v.size());
    if (classifyPointInMesh(meshA, bbA, deflection, c) != Membership::Out) return false;
  }
  return true;
}

/// Reconstruct `B`'s solid from its extracted polygons (for the pre-cut `B` mesh).
/// `B` is passed to the entry point as a Shape; the seam only retained its polygons,
/// so rebuild an all-planar solid from them (outward-oriented triangle faces).
inline topo::Shape bSolidFromPolys(const std::vector<Polygon>& polys) {
  VertexPool pool;
  std::vector<topo::Shape> faces;
  for (const Polygon& p : polys) detail::triangulatePolygonToFaces(p, pool, faces);
  if (faces.size() < 4) return {};
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

}  // namespace todetail

/// `B`'s solid rebuilt from the seam's retained polygons (helper for the pre-cut mesh).
inline topo::Shape seamBSolid(const InterSolidSeam& seam) {
  return todetail::bSolidFromPolys(seam.bPolys);
}

// ─────────────────────────────────────────────────────────────────────────────
// The two-operand FUSE assembler (the genuine new weld). Consumes the recognised
// operand `A` + its inter-solid seam against `B`; returns the welded, self-verified
// union solid or a NULL `Shape` with a measured decline.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape fuseTwoOperand(const FreeformOperand& A, const InterSolidSeam& seam,
                                  double deflection, TwoOperandDecline* why = nullptr) {
  auto fail = [&](TwoOperandDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  std::vector<topo::Shape> faces = todetail::assembleFuseFaces(seam);
  if (faces.size() < 4) return fail(TwoOperandDecline::WeldOpen);

  // Pre-cut operand meshes (B3 membership + independent union-volume bounds).
  tess::MeshParams mp; mp.deflection = deflection;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(A.solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(seamBSolid(seam));
  if (!tess::isWatertight(meshA) || !tess::isWatertight(meshB))
    return fail(TwoOperandDecline::NotWatertight);
  if (!todetail::confirmMembership(A, seam, meshA, meshB, deflection))
    return fail(TwoOperandDecline::ClassifyAmbiguous);

  const topo::Shape shell = topo::ShapeBuilder::makeShell(std::move(faces));
  const topo::Shape fused = topo::ShapeBuilder::makeSolid({shell});

  // Mandatory self-verify: watertight + a consistent union volume.
  const tess::Mesh m = tess::SolidMesher(mp).mesh(fused);
  if (!tess::isWatertight(m)) return fail(TwoOperandDecline::NotWatertight);
  const double vA = std::fabs(tess::enclosedVolume(meshA));
  const double vB = std::fabs(tess::enclosedVolume(meshB));
  const double v = std::fabs(tess::enclosedVolume(m));
  const double tol = 0.03 * std::max(vA + vB, 1e-12);
  if (v < std::max(vA, vB) - tol || v > vA + vB + tol)
    return fail(TwoOperandDecline::VolumeInconsistent);

  if (why) *why = TwoOperandDecline::Ok;
  return fused;
}

// ─────────────────────────────────────────────────────────────────────────────
// freeformBooleanTwoOperand — the entry point. `A` is a freeform-walled solid, `B`
// a finite all-planar solid (a box). Returns the self-verified FUSE / CUT / COMMON
// solid, or a NULL `Shape` (→ OCCT `BRepAlgoAPI_{Fuse,Cut,Common}`) with a measured
// decline. Never emits a leaky/overlapping/wrong-volume solid.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape freeformBooleanTwoOperand(const topo::Shape& A, const topo::Shape& B,
                                             TwoOperandOp op, double deflection = 0.01,
                                             TwoOperandDecline* why = nullptr) {
  auto fail = [&](TwoOperandDecline d) -> topo::Shape { if (why) *why = d; return {}; };

  OperandDecline b1 = OperandDecline::Ok;
  const auto opA = recogniseFreeformSolid(A, &b1);
  if (!opA) return fail(TwoOperandDecline::NotAdmittedA);

  SeamDecline sd = SeamDecline::Ok;
  const auto seam = buildInterSolidSeam(*opA, B, &sd);
  if (!seam) return fail(TwoOperandDecline::SeamDeclined);

  if (op == TwoOperandOp::Fuse) return fuseTwoOperand(*opA, *seam, deflection, why);

  // CUT / COMMON reduce to the landed single-operand half-space cut for this pose
  // (theorem of the containment guard): CUT keeps A OUTSIDE Pcut, COMMON keeps A INSIDE.
  const KeepSide side = op == TwoOperandOp::Cut ? seam->keepSide
                                                : (seam->keepSide == KeepSide::Below
                                                       ? KeepSide::Above
                                                       : KeepSide::Below);
  HalfSpaceCutDecline hd = HalfSpaceCutDecline::Ok;
  const topo::Shape r = freeformHalfSpaceCut(A, seam->tracePlane, side, deflection, &hd);
  if (r.isNull()) return fail(TwoOperandDecline::CutCommonDeclined);
  if (why) *why = TwoOperandDecline::Ok;
  return r;
}

}  // namespace cybercad::native::boolean

#endif  // CYBERCAD_NATIVE_BOOLEAN_TWO_OPERAND_H

// SPDX-License-Identifier: Apache-2.0
//
// draft_faces.h — MOAT feature: the native DRAFT ANGLE (the molding / manufacturing
// taper the app's push/pull tools cannot express — tilt one or more PLANAR side faces
// of a prismatic solid about a planar NEUTRAL PLANE by a draft angle θ, so the walls
// taper for mold release along a PULL DIRECTION).
//
// The verb re-derives NO booleans of its own: it DERIVES, per drafted face, the tilted
// TARGET PLANE and TRIMS the solid to it via the landed, already-two-gate-verified DM1
// half-space cut `nb::splitByPlane` (each cut self-verifies watertight). A draft only
// REMOVES stock, so every drafted face is a pure inward trim — no grow. A multi-face
// draft is a SEQUENCE of such cuts, the planes all derived UP FRONT from the original
// solid (so a later cut never re-reads a fragmented mesh), then one composite re-audit.
//
// ── THE GEOMETRY (one drafted planar face F) ─────────────────────────────────────
// F has outward unit normal n̂_F and a point o_F on it. The neutral plane N has origin
// o_N and unit normal = the PULL direction p̂. The face's trace on N is the line
//   L = F ∩ N,   direction t̂ = normalize(n̂_F × p̂).
// A draft PIVOTS F about L: the trace stays put (it lies on the neutral plane, which does
// not move) and the face tilts by θ about t̂. So:
//   * the rotation axis is t̂ (the trace direction),
//   * the new outward normal is n̂' = Rot(t̂, φ)·n̂_F, and
//   * the target plane passes through the foot of the face centroid on N (a point of L).
// The tilt sense φ is chosen so a POSITIVE angle draws material IN on the +p̂ side
// (standard "draft angle" convention: the wall leans toward the pull axis as it recedes
// from the neutral plane, easing mold release). `splitByPlane` then re-trims the walls
// to the tilted plane, keeping the material (bulk) side; the composite is re-audited
// watertight / χ=2 / consistently oriented / strictly SMALLER than the original.
//
// ── HONEST DECLINE (never a wrong / self-intersecting solid) ──────────────────────
// Anything outside the tractable prismatic-planar slice DECLINES (measured reason) and
// the engine falls to OCCT — never a widened tolerance, never a fabricated solid:
//   * NonPrismaticOrForeign — the solid is not all-planar (a curved neighbour / mesh body)
//     or a picked face is not an identifiable planar polygon.
//   * NonPlanarNeutral      — the pull direction is degenerate (the neutral plane normal
//     is ~0), so no well-defined neutral plane.
//   * FaceParallelToPull    — a drafted face is (near-)perpendicular to the pull axis (its
//     normal ∥ p̂): it has no trace line on N to pivot about, so a draft is undefined.
//   * DegenerateAngle       — |θ| below the numeric floor (a no-op) or ≥ 90° (a flip).
//   * ResolveFailed         — a per-face trim, or the composite self-verify, failed
//     (self-intersecting draft, topology change, non-shrink, or a leaky result).
// The engine reports the decline and NEVER hands a native void to OCCT.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// `splitByPlane` (boolean/split_plane.h), `rfdetail::readPickedFace` + the mesh-audit
// primitives `eulerChar` (directmodel/replace_face.h), `math::Mat3::rotation`
// (math/transform.h), the tessellate mesh audit. Additive sibling — touches none of the
// landed DM1/DM2 headers.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_FEATURE_DRAFT_FACES_H
#define CYBERCAD_NATIVE_FEATURE_DRAFT_FACES_H

#include "native/directmodel/replace_face.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <vector>

namespace cybercad::native::feature {

namespace math = cybercad::native::math;
namespace topo = cybercad::native::topology;
namespace dm   = cybercad::native::directmodel;
namespace nb   = cybercad::native::boolean;
namespace bl   = cybercad::native::blend;
namespace tess = cybercad::native::tessellate;

// Measured reason a native draft declined (→ engine reports it, falls to OCCT).
enum class DraftFacesDecline {
  Ok = 0,
  NonPrismaticOrForeign,  // solid not all-planar / picked face not a planar polygon
  NonPlanarNeutral,       // pull direction (neutral-plane normal) ~ 0
  FaceParallelToPull,     // a drafted face normal ∥ pull axis → no trace line to pivot
  DegenerateAngle,        // |θ| below the no-op floor, or ≥ 90° (a flip)
  ResolveFailed,          // a per-face DM2 re-solve could not be verified (→ OCCT)
};

namespace dfdetail {
// A draft below this (in RADIANS) is a no-op. Prismatic meshes are exact so this is
// tight — never widened to force a marginal case through.
inline constexpr double kAngleFloorRad = 1e-9;
// A face whose normal is within this of ∥ to the pull axis has no usable trace line.
inline constexpr double kPullParallelTol = 1e-6;  // |n̂_F·p̂| ≥ 1 − tol ⇒ parallel
}  // namespace dfdetail

namespace dfdetail {

// The DRAFTED TARGET PLANE for one picked planar face. Reads the face `faceId` off
// `solid` (must be an identifiable planar polygon of an all-planar solid), pivots its
// plane about the trace line L = F ∩ N (fixed on the neutral plane), and returns the
// tilted plane as (pointOnL, unit outward normal n̂'). `dr` receives the measured
// decline on failure; the returned optional is empty then.
struct DraftedPlane {
  math::Point3 pointOnTrace;  // a point of the pivot line L (lies on both F and N)
  math::Vec3 normal;          // unit outward normal of the drafted face
};
inline std::optional<DraftedPlane> draftedTargetPlane(const topo::Shape& solid, int faceId,
                                                      double angleRad, const math::Point3& nOrigin,
                                                      const math::Dir3& pHat, DraftFacesDecline* dr) {
  auto fail = [&](DraftFacesDecline d) -> std::optional<DraftedPlane> {
    if (dr) *dr = d;
    return std::nullopt;
  };
  const auto pfOpt = dm::rfdetail::readPickedFace(solid, faceId);
  if (!pfOpt) return fail(DraftFacesDecline::NonPrismaticOrForeign);
  const dm::rfdetail::PickedFace& pf = *pfOpt;
  const math::Vec3 nF = pf.plane.normal;  // unit outward face normal

  // The trace direction t̂ = n̂_F × p̂. If n̂_F ∥ p̂ the cross vanishes → no trace line
  // (the face is a cap perpendicular to the pull axis and cannot be drafted).
  const math::Dir3 tHat{math::cross(nF, pHat.vec())};
  if (!tHat.valid() ||
      std::fabs(math::dot(nF, pHat.vec())) >= 1.0 - dfdetail::kPullParallelTol)
    return fail(DraftFacesDecline::FaceParallelToPull);

  // Tilt sense: a POSITIVE draft leans the wall INWARD (toward the pull axis) as the
  // face recedes from the neutral plane along +p̂ — the standard mold-release taper and
  // OCCT BRepOffsetAPI_DraftAngle's convention. The centroid's signed height above the
  // neutral plane selects which way the pivot tips the outward normal so the +p̂ half
  // draws in for +θ (and the −p̂ half draws in for −θ), and its foot on N gives a point
  // of the pivot line L (L ⊂ N and L ⊂ F, so the tilted plane through it contains L).
  const double hC = math::dot(pf.centroid - nOrigin, pHat.vec());
  const double phi = (hC >= 0.0 ? 1.0 : -1.0) * angleRad;
  const math::Vec3 nNew = math::Mat3::rotation(tHat, phi) * nF;
  if (!math::Dir3{nNew}.valid()) return fail(DraftFacesDecline::DegenerateAngle);

  const math::Point3 footN = pf.centroid - pHat.vec() * hC;  // on N and on L
  return DraftedPlane{footN, nNew};
}

}  // namespace dfdetail

// ─────────────────────────────────────────────────────────────────────────────
// Draft `faceCount` planar side faces of `solid` by `angleRad` (RADIANS, positive =
// taper inward for mold release) about the neutral plane (origin `nOrigin`, unit normal
// = pull direction `pull`). Each drafted face pivots on its trace with the neutral
// plane; the walls are re-trimmed to the tilted planes so the solid stays watertight.
// Returns the one verified watertight solid, or a NULL Shape (with a measured
// `DraftFacesDecline`) for an honest decline.
//
// ── HOW (composition of the landed DM1 half-space cut) ────────────────────────────
// Every drafted face tapers INWARD, so drafting it is a pure TRIM — no grow. We
// compute EACH drafted target plane up front from the ORIGINAL solid's face geometry
// (so a face's plane is read before any earlier cut fragments its neighbours' meshes),
// then apply them as a SEQUENCE of `nb::splitByPlane` half-space cuts, keeping the
// material (bulk) side each time. Computing the planes from the untouched original
// avoids the id-remap / coplanar-fragmentation hazard of re-reading a boolean's
// triangulated output between steps. Each cut self-verifies watertight on its own; the
// composite is then re-audited (watertight closed 2-manifold, single lump χ=2,
// consistently oriented, positive volume strictly SMALLER than the original — a draft
// only removes material). A cut that cannot be verified, or a composite that fails the
// audit, DECLINES (never a leaky / self-intersecting / grown solid) → OCCT.
// ─────────────────────────────────────────────────────────────────────────────
inline topo::Shape draftFaces(const topo::Shape& solid, const int* faceIds, int faceCount,
                              double angleRad, const math::Point3& nOrigin,
                              const math::Vec3& pull, DraftFacesDecline* why = nullptr,
                              double defl = 0.005) {
  auto fail = [&](DraftFacesDecline d) -> topo::Shape {
    if (why) *why = d;
    return {};
  };
  if (faceIds == nullptr || faceCount < 1) return fail(DraftFacesDecline::NonPrismaticOrForeign);

  // Pull direction defines the neutral plane's normal; must be non-degenerate.
  const math::Dir3 pHat{pull};
  if (!pHat.valid()) return fail(DraftFacesDecline::NonPlanarNeutral);

  // |θ| in [floor, 90°). Below the floor is a no-op; ≥ 90° flips the face.
  const double a = std::fabs(angleRad);
  if (a < dfdetail::kAngleFloorRad || a >= (0.5 * M_PI - 1e-9))
    return fail(DraftFacesDecline::DegenerateAngle);

  // Derive every drafted target plane from the ORIGINAL solid (ids stable, no fragments).
  std::vector<dfdetail::DraftedPlane> planes;
  planes.reserve(static_cast<std::size_t>(faceCount));
  for (int i = 0; i < faceCount; ++i) {
    DraftFacesDecline dr = DraftFacesDecline::Ok;
    const auto dp = dfdetail::draftedTargetPlane(solid, faceIds[i], angleRad, nOrigin, pHat, &dr);
    if (!dp) return fail(dr);
    planes.push_back(*dp);
  }

  // The original enclosed volume (planar meshes exact) — the composite must SHRINK.
  tess::MeshParams mp0; mp0.deflection = defl;
  const tess::Mesh origMesh = tess::SolidMesher(mp0).mesh(solid);
  if (!tess::isWatertight(origMesh)) return fail(DraftFacesDecline::NonPrismaticOrForeign);
  const double v0 = std::fabs(tess::enclosedVolume(origMesh));
  if (!(v0 > 0.0)) return fail(DraftFacesDecline::NonPrismaticOrForeign);

  // Apply each tilted trim; keep the material (−n̂') half (the bulk), the same side a
  // parallel pull keeps. `splitByPlane` self-verifies watertight per cut.
  topo::Shape work = solid;
  for (const dfdetail::DraftedPlane& dp : planes) {
    nb::HalfSpaceCutDecline sd = nb::HalfSpaceCutDecline::Ok;
    topo::Shape r = nb::splitByPlane(work, dp.pointOnTrace, dp.normal, /*keepPositive=*/false,
                                     defl, &sd);
    if (r.isNull()) return fail(DraftFacesDecline::ResolveFailed);
    work = r;
  }

  // Composite self-verify: watertight closed 2-manifold, single lump (χ=2), consistently
  // oriented, positive volume STRICTLY smaller than the original (a draft removes stock).
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(work);
  if (!tess::isWatertight(m)) return fail(DraftFacesDecline::ResolveFailed);
  if (!tess::isConsistentlyOriented(m)) return fail(DraftFacesDecline::ResolveFailed);
  if (dm::rfdetail::eulerChar(m) != 2) return fail(DraftFacesDecline::ResolveFailed);
  const double vr = std::fabs(tess::enclosedVolume(m));
  const double volTol = std::max(1e-6 * v0, 1e-9);
  if (!(vr > 0.0) || !(vr < v0 - volTol)) return fail(DraftFacesDecline::ResolveFailed);

  if (why) *why = DraftFacesDecline::Ok;
  return work;
}

}  // namespace cybercad::native::feature

#endif  // CYBERCAD_NATIVE_FEATURE_DRAFT_FACES_H

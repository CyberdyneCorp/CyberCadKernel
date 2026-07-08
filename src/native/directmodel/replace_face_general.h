// SPDX-License-Identifier: Apache-2.0
//
// replace_face_general.h — MOAT M-DM DM3: the native GENERAL `replace_face`
// (the app's `cc_replace_face(body, faceId, offset, tiltDeg)` — retarget a planar
// face by OFFSETTING it along its own outward normal and TILTING it about the face's
// parametric X-axis, trimming the solid to the new plane). This is the more general
// sibling of DM2's `replace_face_to_plane` (an explicit target plane).
//
// The verb re-derives NO geometry: it DERIVES the target plane from the picked face's
// own plane and re-solves via the landed, already-gated DM2 `replaceFaceToPlane`
// (grow-then-trim = 1 Fuse + 1 Cut, each individually watertight self-verified):
//
//   * PURE OFFSET (tiltDeg ≈ 0)  — target plane = (o + n̂_F·offset, n̂_F). A pure
//     parallel push/pull along the face normal → the DM2 parallel grow / trim branch,
//     ΔV = A_F·offset exactly. This is the NATIVE scope.
//
// ── HONEST DECLINE (never a wrong / foreign-convention solid) ────────────────────
// A NON-ZERO tilt rotates the normal about the face's OCCT surface-PARAMETRIZATION
// X-axis (`gp_Pln::XAxis`) — a foreign convention we do NOT reproduce for a native
// B-rep body — so a tilted retarget DECLINES to OCCT (`TiltNotReproduced`). A
// non-planar picked face / non-all-planar solid / mesh-only body declines
// (`NonPlanarOrForeign`), and a degenerate no-op offset or a re-solve the DM2 verb
// cannot verify declines (`ResolveFailed`). The engine reports the honest decline and
// falls to OCCT; it NEVER hands a native void to OCCT and NEVER emits an unverified
// solid.
//
// ── CONSUMES (byte-identical, never rewritten) ──────────────────────────────────
// `replaceFaceToPlane` + `rfdetail::readPickedFace` (directmodel/replace_face.h).
// Additive sibling — touches none of the landed DM1/DM2 headers.
//
// OCCT-FREE (0 OCCT includes). Header-only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_GENERAL_H
#define CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_GENERAL_H

#include "native/directmodel/replace_face.h"
#include "native/math/native_math.h"
#include "native/topology/native_topology.h"

#include <cmath>
#include <optional>

namespace cybercad::native::directmodel {

// Measured reason a general (offset+tilt) retarget declined (→ engine reports it,
// falls to OCCT). Distinct from DM2's ReplaceFaceDecline so the DM2 enum stays frozen.
enum class ReplaceFaceGeneralDecline {
  Ok = 0,
  NonPlanarOrForeign,  // solid not all-planar / picked face not an identifiable planar polygon
  TiltNotReproduced,   // non-zero tilt about OCCT's face-parametrization X-axis → OCCT
  ResolveFailed,       // degenerate no-op offset, or the DM2 re-solve could not be verified
};

namespace rfgdetail {
// A tilt below this (in DEGREES) is treated as a pure normal offset. Planar meshes
// are exact, so this is tight — never widened to force a tilted case through.
inline constexpr double kTiltEpsDeg = 1e-9;
}  // namespace rfgdetail

// ─────────────────────────────────────────────────────────────────────────────
// Retarget the planar face `faceId` (1-based, mapShapes Face order) by `offset`
// along its outward normal and `tiltDeg` about the face X-axis. Returns the one
// verified watertight solid, or a NULL Shape (with a measured
// `ReplaceFaceGeneralDecline`) for an honest decline.
// ─────────────────────────────────────────────────────────────────────────────
inline topology::Shape replaceFaceOffsetTilt(const topology::Shape& solid, int faceId,
                                             double offset, double tiltDeg,
                                             ReplaceFaceGeneralDecline* why = nullptr,
                                             double defl = 0.005) {
  namespace tmath = cybercad::native::math;
  auto fail = [&](ReplaceFaceGeneralDecline d) -> topology::Shape {
    if (why) *why = d;
    return {};
  };

  const auto pfOpt = rfdetail::readPickedFace(solid, faceId);
  if (!pfOpt) return fail(ReplaceFaceGeneralDecline::NonPlanarOrForeign);

  // A non-zero tilt needs OCCT's face-parametrization X-axis — a foreign convention.
  if (std::fabs(tiltDeg) > rfgdetail::kTiltEpsDeg)
    return fail(ReplaceFaceGeneralDecline::TiltNotReproduced);

  // PURE OFFSET: derive the target plane (o + n̂_F·offset, n̂_F) and re-solve via DM2.
  const tmath::Vec3 nF = pfOpt->plane.normal;                 // unit outward normal
  const tmath::Point3 tp = pfOpt->centroid + nF * offset;     // a point on the target plane
  ReplaceFaceDecline sub = ReplaceFaceDecline::Ok;
  topology::Shape r = replaceFaceToPlane(solid, faceId, tp, nF, &sub, defl);
  if (r.isNull()) return fail(ReplaceFaceGeneralDecline::ResolveFailed);

  if (why) *why = ReplaceFaceGeneralDecline::Ok;
  return r;
}

}  // namespace cybercad::native::directmodel

#endif  // CYBERCAD_NATIVE_DIRECTMODEL_REPLACE_FACE_GENERAL_H

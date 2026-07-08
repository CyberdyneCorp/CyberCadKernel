// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M-DM DM3 — the native GENERAL `replace_face`
// (src/native/directmodel/replace_face_general.h), OCCT-FREE. The verb models the
// app's `cc_replace_face(body, faceId, offset, tiltDeg)`: derive the target plane
// from the picked planar face's own plane (offset along the outward normal, tilt
// about the face X-axis) and re-solve via the landed DM2 replaceFaceToPlane. For the
// NATIVE (pure-offset) slice δ is affine and constant over the face, so the swept
// volume is the CLOSED-FORM oracle `V' = V₀ + A_F·offset`. We assert, with no OCCT:
//   * PURE PUSH / PULL along an axis-aligned normal — V' fp-exact;
//   * PURE OFFSET of a NON-AXIS-ALIGNED face (the box rotated about Z) — V' fp-exact,
//     the DM3-over-DM2 breadth (an off-axis planar retarget);
//   * the watertight closed 2-manifold self-verify (inherited from DM2);
// and the HONEST-DECLINE envelope returns a NULL Shape (→ engine reports a decline):
//   * a NON-ZERO tilt (OCCT face-parametrization X-axis is a foreign convention);
//   * a curved neighbour (cylinder side — the solid is not all-planar);
//   * a degenerate no-op offset.
// Requires CYBERCAD_HAS_NUMSCI (replaceFaceToPlane's re-solve probes the real seam).
//
#include "native/directmodel/replace_face_general.h"

#include "harness.h"

#include <cmath>
#include <optional>

namespace dm    = cybercad::native::directmodel;
namespace tess  = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace tcst  = cybercad::native::construct;
namespace ttopo = cybercad::native::topology;

namespace {

double meshVolume(const ttopo::Shape& s, double defl, bool& watertight) {
  if (s.isNull()) { watertight = false; return 0.0; }
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

ttopo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[8] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  ttopo::Shape s = tcst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, z0})));
  return s;
}

ttopo::Shape cylinderZ(double r, double zlo, double zhi) {
  tcst::ProfileSegment seg; seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = r;
  ttopo::Shape s = tcst::build_prism_profile({seg}, {}, {}, zhi - zlo);
  if (!s.isNull() && zlo != 0.0)
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, zlo})));
  return s;
}

std::optional<int> faceByNormal(const ttopo::Shape& s, const tmath::Vec3& dir) {
  const ttopo::ShapeMap map = ttopo::mapShapes(s, ttopo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = cybercad::native::blend::facePlane(s, static_cast<int>(i));
    if (pl && tmath::dot(pl->normal, tmath::Vec3{dir.x, dir.y, dir.z}) > 0.999)
      return static_cast<int>(i);
  }
  return std::nullopt;
}

}  // namespace

// ── PURE PUSH (grow): box +x face, offset +3 ⇒ V = 1000 + 100·3 = 1300 (fp-exact) ──
CC_TEST(general_pure_push_closed_form_exact) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);  // V₀=1000, A(+x)=100
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());

  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(box, *fid, /*offset=*/3.0, /*tilt=*/0.0, &why);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 1300.0) <= 1e-3);
}

// ── PURE PULL (trim): box +x face, offset −3 ⇒ V = 1000 − 100·3 = 700 (fp-exact) ──
CC_TEST(general_pure_pull_closed_form_exact) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());

  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(box, *fid, /*offset=*/-3.0, /*tilt=*/0.0, &why);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 700.0) <= 1e-3);
}

// ── DM3 BREADTH: PURE OFFSET of a NON-AXIS-ALIGNED face. Rotate the box 30° about Z;
// its former +x face now has normal (cos30,sin30,0). Offset +2 along THAT normal —
// A_F is unchanged (100), so ΔV = +200 ⇒ V = 1200 (invariant under the rigid move). ──
CC_TEST(general_offaxis_face_offset_closed_form) {
  const double defl = 0.005;
  ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  box = box.located(ttopo::Location(tmath::Transform::rotationOf(
      tmath::Point3{0, 0, 0}, tmath::Dir3{0, 0, 1}, 30.0 * 3.14159265358979323846 / 180.0)));
  const tmath::Vec3 nOff{std::cos(30.0 * 3.14159265358979323846 / 180.0),
                         std::sin(30.0 * 3.14159265358979323846 / 180.0), 0};
  const auto fid = faceByNormal(box, nOff);
  CC_CHECK(fid.has_value());

  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(box, *fid, /*offset=*/2.0, /*tilt=*/0.0, &why);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 1200.0) <= 1e-2);
}

// ── HONEST DECLINE: a non-zero tilt (OCCT face-parametrization X-axis, foreign). ──
CC_TEST(general_nonzero_tilt_declines) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{0, 0, 1});
  CC_CHECK(fid.has_value());
  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(box, *fid, /*offset=*/0.0, /*tilt=*/12.0, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::TiltNotReproduced);
}

// ── HONEST DECLINE: a curved neighbour (cylinder cap) — not all-planar → NULL. ──
CC_TEST(general_curved_neighbour_declines) {
  const ttopo::Shape cyl = cylinderZ(5, 0, 20);
  CC_CHECK(!cyl.isNull());
  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(cyl, 1, /*offset=*/2.0, /*tilt=*/0.0, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::NonPlanarOrForeign);
}

// ── HONEST DECLINE: a degenerate no-op offset (|d̄| ≈ 0) → NULL via the DM2 re-solve. ──
CC_TEST(general_noop_offset_declines) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  dm::ReplaceFaceGeneralDecline why = dm::ReplaceFaceGeneralDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceOffsetTilt(box, *fid, /*offset=*/0.0, /*tilt=*/0.0, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceGeneralDecline::ResolveFailed);
}

CC_RUN_ALL()

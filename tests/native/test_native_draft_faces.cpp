// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT feature DRAFT ANGLE (src/native/feature/draft_faces.h),
// OCCT-FREE. The verb tapers one or more PLANAR side faces of a prismatic solid about
// a planar NEUTRAL plane by a draft angle θ along a PULL direction, re-solving the
// neighbours via the landed DM2 replaceFaceToPlane. For a box drafted about its base
// plane the removed material is a CLOSED-FORM WEDGE, so we assert the exact taper
// volume with no OCCT:
//   * SINGLE side face drafted +θ about the base ⇒ V = V₀ − ½·H·(H·tanθ)·D (a wedge);
//     watertight closed 2-manifold; single lump (χ=2); consistently oriented.
//   * ALL FOUR side faces drafted ⇒ a frustum-like taper; V matches the closed form.
//   * an OFF-AXIS (rotated) box side face drafted — the wedge is rigid-motion invariant.
// and the HONEST-DECLINE envelope returns a NULL Shape (→ engine reports a decline):
//   * a CURVED base (cylinder — not all-planar);
//   * a NON-PLANAR neutral (degenerate pull direction);
//   * a face PERPENDICULAR to the pull axis (no trace line to pivot about — a cap);
//   * a DEGENERATE (near-0 / ≥90°) angle.
// Requires CYBERCAD_HAS_NUMSCI (the DM2 tilted re-solve probes the real seam).
//
#include "native/feature/draft_faces.h"

#include "harness.h"

#include <cmath>
#include <optional>

namespace df    = cybercad::native::feature;
namespace tess  = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace tcst  = cybercad::native::construct;
namespace ttopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;

struct MeshAudit {
  double volume = 0.0;
  bool watertight = false;
  bool oriented = false;
  long euler = 0;
};

MeshAudit auditMesh(const ttopo::Shape& s, double defl) {
  MeshAudit a;
  if (s.isNull()) return a;
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  a.watertight = tess::isWatertight(m);
  a.oriented = tess::isConsistentlyOriented(m);
  a.volume = std::fabs(tess::enclosedVolume(m));
  a.euler = df::dm::rfdetail::eulerChar(m);
  return a;
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
    if (pl && tmath::dot(pl->normal, dir) > 0.999) return static_cast<int>(i);
  }
  return std::nullopt;
}

}  // namespace

// ── SINGLE FACE: box[0,10]³ drafted +8° about z=0 base, pull +z, on the +x face.
// Removed wedge = ½·H·(H·tanθ)·D = ½·10·(10·tan8°)·10 = 500·tan8° ⇒ V = 1000 − 500·tan8°. ──
CC_TEST(draft_single_side_wedge_closed_form) {
  const double defl = 0.005;
  const double th = 8.0 * kPi / 180.0;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());

  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {*fid};
  const ttopo::Shape r =
      df::draftFaces(box, ids, 1, th, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  const MeshAudit a = auditMesh(r, defl);
  const double expected = 1000.0 - 500.0 * std::tan(th);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::Ok);
  CC_CHECK(a.watertight);
  CC_CHECK(a.oriented);
  CC_CHECK(a.euler == 2);
  CC_CHECK(std::fabs(a.volume - expected) <= 1e-2);
}

// ── FOUR FACES: draft all 4 side faces +5° about z=0 ⇒ a truncated pyramid (frustum).
// Each face recedes; the taper closed form is the frustum volume. Bottom 10×10 at z=0,
// top (10−2·10·tanθ)² at z=10 (each side moves in by H·tanθ). Frustum volume =
// (H/3)(A_bot + A_top + √(A_bot·A_top)). ──
CC_TEST(draft_four_sides_frustum_closed_form) {
  const double defl = 0.005;
  const double th = 5.0 * kPi / 180.0;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  int ids[4] = {0, 0, 0, 0};
  const auto fx = faceByNormal(box, tmath::Vec3{1, 0, 0});
  const auto fnx = faceByNormal(box, tmath::Vec3{-1, 0, 0});
  const auto fy = faceByNormal(box, tmath::Vec3{0, 1, 0});
  const auto fny = faceByNormal(box, tmath::Vec3{0, -1, 0});
  CC_CHECK(fx && fnx && fy && fny);
  ids[0] = *fx; ids[1] = *fnx; ids[2] = *fy; ids[3] = *fny;

  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const ttopo::Shape r =
      df::draftFaces(box, ids, 4, th, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  const MeshAudit a = auditMesh(r, defl);
  const double side = 10.0 - 2.0 * 10.0 * std::tan(th);  // each side recedes H·tanθ
  const double Abot = 10.0 * 10.0, Atop = side * side;
  const double expected = (10.0 / 3.0) * (Abot + Atop + std::sqrt(Abot * Atop));
  CC_CHECK(!r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::Ok);
  CC_CHECK(a.watertight);
  CC_CHECK(a.oriented);
  CC_CHECK(a.euler == 2);
  CC_CHECK(std::fabs(a.volume - expected) <= 5e-2);
}

// ── OFF-AXIS: rotate the box 25° about Z; draft its former +x face 8° about z=0.
// The wedge is invariant under the rigid rotation ⇒ same V = 1000 − 500·tan8°. ──
CC_TEST(draft_offaxis_side_wedge_invariant) {
  const double defl = 0.005;
  const double th = 8.0 * kPi / 180.0;
  ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  box = box.located(ttopo::Location(
      tmath::Transform::rotationOf(tmath::Point3{0, 0, 0}, tmath::Dir3{0, 0, 1}, 25.0 * kPi / 180.0)));
  const tmath::Vec3 nOff{std::cos(25.0 * kPi / 180.0), std::sin(25.0 * kPi / 180.0), 0};
  const auto fid = faceByNormal(box, nOff);
  CC_CHECK(fid.has_value());

  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {*fid};
  const ttopo::Shape r =
      df::draftFaces(box, ids, 1, th, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  const MeshAudit a = auditMesh(r, defl);
  const double expected = 1000.0 - 500.0 * std::tan(th);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::Ok);
  CC_CHECK(a.watertight);
  CC_CHECK(a.oriented);
  CC_CHECK(std::fabs(a.volume - expected) <= 2e-2);
}

// ── HONEST DECLINE: curved base (cylinder — not all-planar) → NULL. ──
CC_TEST(draft_curved_base_declines) {
  const ttopo::Shape cyl = cylinderZ(5, 0, 20);
  CC_CHECK(!cyl.isNull());
  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {1};
  const ttopo::Shape r =
      df::draftFaces(cyl, ids, 1, 5.0 * kPi / 180.0, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::NonPrismaticOrForeign);
}

// ── HONEST DECLINE: degenerate pull direction (no neutral plane) → NULL. ──
CC_TEST(draft_nonplanar_neutral_declines) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {*fid};
  const ttopo::Shape r =
      df::draftFaces(box, ids, 1, 5.0 * kPi / 180.0, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 0}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::NonPlanarNeutral);
}

// ── HONEST DECLINE: a cap face (+z top) is PERPENDICULAR to the pull axis — no trace
// line to pivot about → FaceParallelToPull. ──
CC_TEST(draft_cap_face_parallel_to_pull_declines) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{0, 0, 1});  // top cap, normal ∥ pull
  CC_CHECK(fid.has_value());
  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {*fid};
  const ttopo::Shape r =
      df::draftFaces(box, ids, 1, 5.0 * kPi / 180.0, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::FaceParallelToPull);
}

// ── HONEST DECLINE: a degenerate (near-0) draft angle is a no-op → NULL. ──
CC_TEST(draft_degenerate_angle_declines) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  df::DraftFacesDecline why = df::DraftFacesDecline::Ok;
  const int ids[1] = {*fid};
  const ttopo::Shape r =
      df::draftFaces(box, ids, 1, 1e-12, tmath::Point3{0, 0, 0}, tmath::Vec3{0, 0, 1}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == df::DraftFacesDecline::DegenerateAngle);
}

CC_RUN_ALL()

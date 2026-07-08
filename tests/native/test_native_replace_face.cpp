// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M-DM DM2 — the native `replace_face_to_plane`
// (src/native/directmodel/replace_face.h), OCCT-FREE. The verb moves ONE planar
// face of a convex planar polyhedron onto a target plane and re-solves the
// neighbours. δ is affine over the planar face, so the swept-wedge volume is the
// CLOSED-FORM oracle `V' = V₀ + A_F·d̄`. On the reachable fixtures we assert, with no
// OCCT:
//   * the result is a watertight closed 2-manifold (never a leak);
//   * single lump (Euler χ = 2) and the distinct-plane (face) count is preserved;
//   * the moved face lies on the target plane;
//   * V' matches the closed-form `V₀ + A_F·d̄` — parallel push (grow) / pull (trim)
//     fp-exact, and a tilted re-solve within the mesh band.
// The HONEST-DECLINE envelope returns a NULL Shape (→ engine reports an honest
// decline): a curved neighbour (cylinder cap), a non-planar picked "face"
// (a cylinder side), a degenerate normal, a coincident (no-op) target, and a
// topology-changing plane that would clip a neighbour away.
// Requires CYBERCAD_HAS_NUMSCI (splitByPlane's freeform probe traces the real seam).
//
#include "native/directmodel/replace_face.h"

#include "harness.h"

#include <cmath>
#include <optional>

namespace dm   = cybercad::native::directmodel;
namespace bo   = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace tcst = cybercad::native::construct;
namespace ttopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;

double meshVolume(const ttopo::Shape& s, double defl, bool& watertight) {
  if (s.isNull()) { watertight = false; return 0.0; }
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// Axis-aligned box [x0,x0+sx]×[y0,y0+sy]×[z0,z0+sz] through the native construct path.
ttopo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[8] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  ttopo::Shape s = tcst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, z0})));
  return s;
}

// A Z-axis cylinder (radius r, z ∈ [zlo,zhi]) through the native construct path.
ttopo::Shape cylinderZ(double r, double zlo, double zhi) {
  tcst::ProfileSegment seg; seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = r;
  ttopo::Shape s = tcst::build_prism_profile({seg}, {}, {}, zhi - zlo);
  if (!s.isNull() && zlo != 0.0)
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, zlo})));
  return s;
}

// Find the 1-based (mapShapes Face order) id of the face whose outward normal ≈ dir.
std::optional<int> faceByNormal(const ttopo::Shape& s, const tmath::Vec3& dir) {
  const ttopo::ShapeMap map = ttopo::mapShapes(s, ttopo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = cybercad::native::blend::facePlane(s, static_cast<int>(i));
    if (pl && tmath::dot(pl->normal, dir) > 0.999) return static_cast<int>(i);
  }
  return std::nullopt;
}

}  // namespace

// ── PARALLEL PULL (trim): box x=10 face → x=7, ΔV = −A·3 (fp-exact) ────────────────
CC_TEST(box_parallel_pull_closed_form_exact) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);  // V₀ = 1000, A(x-face) = 100
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());

  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(box, *fid, tmath::Point3{7, 0, 0}, tmath::Vec3{1, 0, 0}, &why, defl);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 700.0) <= 1e-3);              // V₀ + A·(−3) = 700, fp-exact
}

// ── PARALLEL PUSH (grow): box x=10 face → x=13, ΔV = +A·3 (fp-exact) ───────────────
CC_TEST(box_parallel_push_closed_form_exact) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());

  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(box, *fid, tmath::Point3{13, 0, 0}, tmath::Vec3{1, 0, 0}, &why, defl);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 1300.0) <= 1e-3);             // V₀ + A·(+3) = 1300, fp-exact
}

// ── TILTED PURE-TRIM: box top z=10 → plane through (5,5,8) tilted about x through the
// centroid; d̄ = δ(centroid) = −2, ΔV = A·d̄ = −200 (single tilted cut). ─────────────
CC_TEST(box_tilted_pure_trim_closed_form) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);  // top face normal +Z, A = 100
  const auto fid = faceByNormal(box, tmath::Vec3{0, 0, 1});
  CC_CHECK(fid.has_value());

  // Tilt about the x-axis through the centroid (5,5): normal in the y–z plane, both
  // corners at y=0 raised and y=10 lowered symmetrically, so d̄ (at y=5) = −2.
  const double th = 0.15;  // radians
  const tmath::Vec3 n{0, std::sin(th), std::cos(th)};
  const tmath::Point3 tp{5, 5, 8};  // δ(centroid) = ((tp−c)·n)/(n·ẑ), c=(5,5,10)
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceToPlane(box, *fid, tp, n, &why, defl);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::Ok);
  CC_CHECK(wt);
  // Closed form: d̄ = ((5-5,5-5,8-10)·n)/cos(th) = (−2·cos th)/cos th = −2 ⇒ ΔV = −200.
  CC_CHECK(std::fabs(v - 800.0) <= 5e-2);
}

// ── TILTED GROW-then-TRIM (mixed in/out): box top z=10 → plane through (5,5,10)
// tilted about x; centroid stays (d̄=0) but one edge grows outward, the other trims.
// The re-solve grows a slab then trims: ΔV = A·d̄ = 0 (volume preserved). ────────────
CC_TEST(box_tilted_mixed_grow_trim_volume_preserved) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{0, 0, 1});
  CC_CHECK(fid.has_value());

  const double th = 0.15;
  const tmath::Vec3 n{0, std::sin(th), std::cos(th)};
  const tmath::Point3 tp{5, 5, 10};  // through the original centroid ⇒ d̄ = 0
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r = dm::replaceFaceToPlane(box, *fid, tp, n, &why, defl);
  bool wt = false;
  const double v = meshVolume(r, defl, wt);
  CC_CHECK(!r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - 1000.0) <= 5e-2);             // d̄ = 0 ⇒ volume preserved
}

// ── HONEST DECLINE: a curved neighbour (cylinder). Moving the top cap cannot re-solve
// the curved side → the solid is not all-planar → NULL. ──────────────────────────
CC_TEST(cylinder_cap_declines_null) {
  const double defl = 0.01;
  const ttopo::Shape cyl = cylinderZ(5, 0, 20);
  CC_CHECK(!cyl.isNull());
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(cyl, 1, tmath::Point3{0, 0, 15}, tmath::Vec3{0, 0, 1}, &why, defl);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::NonPlanarOrForeign);
}

// ── HONEST DECLINE: a degenerate target normal → NULL (never a wrong plane). ───────
CC_TEST(degenerate_normal_declines_null) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(box, *fid, tmath::Point3{7, 0, 0}, tmath::Vec3{0, 0, 0}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::DegenerateNormal);
}

// ── HONEST DECLINE: a coincident target (no-op, |d̄| ≈ 0) → NULL (degenerate). ─────
CC_TEST(coincident_target_declines_null) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(box, *fid, tmath::Point3{10, 0, 0}, tmath::Vec3{1, 0, 0}, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == dm::ReplaceFaceDecline::DegenerateTarget);
}

// ── HONEST DECLINE: a topology-changing plane that clips a neighbour fully away.
// Pulling the x=10 face all the way to x=−1 (past the opposite x=0 face) leaves no
// bulk on the keep side → the DM1 cut / self-verify rejects it → NULL. ────────────
CC_TEST(topology_change_clips_neighbour_declines_null) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  const auto fid = faceByNormal(box, tmath::Vec3{1, 0, 0});
  CC_CHECK(fid.has_value());
  dm::ReplaceFaceDecline why = dm::ReplaceFaceDecline::Ok;
  const ttopo::Shape r =
      dm::replaceFaceToPlane(box, *fid, tmath::Point3{-1, 0, 0}, tmath::Vec3{1, 0, 0}, &why);
  CC_CHECK(r.isNull());  // no valid single-lump re-solve — honest decline, never emitted
}

CC_RUN_ALL()

// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2-assembly / B4 — the FIRST end-to-end freeform↔analytic
// half-space CUT, OCCT-FREE. On the ONE reachable fixture (a bowl-lidded convex-quad
// prism, cut by the plane x=0, keep x≤0) we assert:
//   * B1 recogniseFreeformSolid ADMITS the operand (1 freeform + 5 analytic faces);
//   * M0 meshes the operand watertight with volume = the closed-form ∫∫_Q(H0+a(x²+y²));
//   * the closed-form polynomial oracle matches a hand value (independent of the mesher);
//   * freeformHalfSpaceCut composes B1→M1→B2→B4→self-verify into a WATERTIGHT solid
//     whose enclosed volume = the closed-form CUT value ∫∫_{Q∩{x≤0}} within the
//     deflection band, and whose every mesh edge is shared by exactly two triangles;
//   * the honest-decline envelope: a plane that misses the operand, and a wrong-shaped
//     operand, each return a NULL Shape (→ OCCT), never a leaky solid.
// Requires CYBERCAD_HAS_NUMSCI (the composition traces the real M1 seam).
//
#include "native/boolean/freeform_operand.h"
#include "native/boolean/half_space_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "native/first_freeform_boolean_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;

namespace {
double meshVolume(const cybercad::native::topology::Shape& s, double defl, bool& watertight) {
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}
}  // namespace

// ── The closed-form polynomial oracle, independently checked ──────────────────
CC_TEST(closed_form_volume_oracle_is_exact) {
  // Full operand volume: over the whole quad. Cut volume: over the x≤0 clip.
  // The cut is a strict sub-region, so 0 < cutVolume < fullVolume.
  const double vFull = ffx::fullVolume();
  const double vCut = ffx::cutVolume();
  CC_CHECK(vFull > 0.0);
  CC_CHECK(vCut > 0.0);
  CC_CHECK(vCut < vFull);
  // Unit check: ∫∫_T (H0 + a(x²+y²)) over the unit right triangle (0,0),(1,0),(0,1):
  // = H0·½ + a·(1/12 + 1/12) = H0/2 + a/6.
  const double tri = ffx::polyVolume({{0, 0}, {1, 0}, {0, 1}});
  CC_CHECK(std::fabs(tri - (ffx::kH0 * 0.5 + ffx::kA / 6.0)) <= 1e-12);
}

// ── B1 admits the operand; M0 meshes it watertight at the closed-form volume ──
CC_TEST(operand_admitted_and_meshes_at_closed_form_volume) {
  const auto solid = ffx::buildOperand();

  bo::OperandDecline why = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(solid, &why);
  CC_CHECK(op.has_value());
  CC_CHECK(why == bo::OperandDecline::Ok);
  if (!op) return;
  CC_CHECK_EQ(op->freeform.size(), std::size_t{1});
  CC_CHECK_EQ(op->analytic.size(), std::size_t{5});
  CC_CHECK(op->watertight);

  bool wt = false;
  const double v = meshVolume(solid, 0.01, wt);
  CC_CHECK(wt);
  CC_CHECK(std::fabs(v - ffx::fullVolume()) <= 0.02 * ffx::fullVolume());  // deflection band
}

// ── The FIRST freeform↔analytic CUT: watertight + closed-form CUT volume ──────
CC_TEST(first_freeform_cut_is_watertight_at_closed_form_volume) {
  const auto solid = ffx::buildOperand();
  const fmath::Plane P = ffx::cutPlane();

  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const auto cut = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Below, 0.01, &why);
  CC_CHECK(why == bo::HalfSpaceCutDecline::Ok);
  CC_CHECK(!cut.isNull());
  if (cut.isNull()) return;

  bool wt = false;
  const double v = meshVolume(cut, 0.01, wt);
  CC_CHECK(wt);                                              // watertight — no leak
  CC_CHECK(std::fabs(v - ffx::cutVolume()) <= 0.02 * ffx::cutVolume());  // closed-form CUT volume
}

// ── Honest-decline envelope: never emit a leaky/wrong solid ───────────────────
CC_TEST(cut_plane_missing_operand_declines_null) {
  const auto solid = ffx::buildOperand();
  // Plane x = 100 (normal +x): the whole operand is on the keep (x≤0) side → the wall
  // never crosses it → the M1 seam is empty / B2 declines → NULL (no cap possible).
  fmath::Ax3 fr;
  fr.origin = fmath::Point3{100, 0, 0};
  fr.x = fmath::Dir3{fmath::Vec3{0, 1, 0}}; fr.y = fmath::Dir3{fmath::Vec3{0, 0, 1}};
  fr.z = fmath::Dir3{fmath::Vec3{1, 0, 0}};
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const auto cut = bo::freeformHalfSpaceCut(solid, fmath::Plane{fr}, bo::KeepSide::Below, 0.01, &why);
  CC_CHECK(cut.isNull());
  CC_CHECK(why != bo::HalfSpaceCutDecline::Ok);
}

CC_TEST(non_freeform_operand_declines_null) {
  // A shape that is not an admissible freeform operand (a bare vertex) → B1 declines.
  const auto v = cybercad::native::topology::ShapeBuilder::makeVertex(fmath::Point3{0, 0, 0});
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const auto cut = bo::freeformHalfSpaceCut(v, ffx::cutPlane(), bo::KeepSide::Below, 0.01, &why);
  CC_CHECK(cut.isNull());
  CC_CHECK(why == bo::HalfSpaceCutDecline::NotAdmitted);
}

CC_RUN_ALL()

// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2-FUSE — the FIRST two-operand freeform boolean, OCCT-FREE.
// On the single-curved-cut pose (A = bowl-lidded convex-quad prism; B = finite box whose
// only cut of A is the plane x=0) we assert, against the mesh-free closed-form oracle:
//   * buildInterSolidSeam ADMITS the pose (unique curved cut, containment) and its
//     inter-solid seam loop CLOSES (design §2, §6.3 — provable even if a weld declines);
//   * freeformBooleanTwoOperand(FUSE) composes recognise[B1] → inter-solid seam → split
//     both → B3-confirm → weld → self-verify into a WATERTIGHT solid whose enclosed
//     volume = the closed-form union V(B)+V(A∩{x≤0}) within the deflection band, with
//     every mesh edge shared by exactly two triangles;
//   * CUT (A−B) and COMMON (A∩B) reduce to the landed half-space cut and match their
//     closed-form values;
//   * the honest-decline envelope: a box that never cuts A's wall, and a non-freeform
//     operand, each return a NULL Shape (→ OCCT), never a leaky/wrong solid.
// Requires CYBERCAD_HAS_NUMSCI (the composition traces the real M1 seam).
//
#include "native/boolean/freeform_operand.h"
#include "native/boolean/inter_solid_seam.h"
#include "native/boolean/two_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "native/first_freeform_boolean_fixture.h"
#include "native/two_operand_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace tox = two_operand_fixture;

namespace {
double meshVolume(const cybercad::native::topology::Shape& s, double defl, bool& watertight) {
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}
}  // namespace

// ── The inter-solid seam-set builder: admits the pose and closes the loop ──────
CC_TEST(inter_solid_seam_closes_for_single_curved_cut) {
  const auto A = ffx::buildOperand();
  const auto B = tox::buildBoxB();

  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  if (!op) return;

  bo::SeamDecline sd = bo::SeamDecline::Ok;
  const auto seam = bo::buildInterSolidSeam(*op, B, &sd);
  CC_CHECK(sd == bo::SeamDecline::Ok);
  CC_CHECK(seam.has_value());
  if (!seam) return;
  CC_CHECK(seam->bPolys.size() == std::size_t{6});   // six box quad faces
  CC_CHECK(seam->capLoop.size() >= std::size_t{3});  // the closed D-outline
  CC_CHECK(!seam->aKeepFaces.empty());
}

// ── FUSE: watertight at the closed-form UNION volume ──────────────────────────
CC_TEST(fuse_is_watertight_at_closed_form_union_volume) {
  const auto A = ffx::buildOperand();
  const auto B = tox::buildBoxB();

  bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
  const auto fused = bo::freeformBooleanTwoOperand(A, B, bo::TwoOperandOp::Fuse, 0.01, &why);
  CC_CHECK(why == bo::TwoOperandDecline::Ok);
  CC_CHECK(!fused.isNull());
  if (fused.isNull()) return;

  bool wt = false;
  const double v = meshVolume(fused, 0.01, wt);
  CC_CHECK(wt);  // watertight — no leak
  const double vExp = tox::unionVolume();
  CC_CHECK(std::fabs(v - vExp) <= 0.02 * vExp);  // closed-form union volume
}

// ── CUT (A−B) and COMMON (A∩B) reduce to the landed half-space cut ────────────
CC_TEST(cut_and_common_match_closed_form) {
  const auto A = ffx::buildOperand();
  const auto B = tox::buildBoxB();

  bo::TwoOperandDecline whyC = bo::TwoOperandDecline::Ok;
  const auto cut = bo::freeformBooleanTwoOperand(A, B, bo::TwoOperandOp::Cut, 0.01, &whyC);
  CC_CHECK(whyC == bo::TwoOperandDecline::Ok);
  CC_CHECK(!cut.isNull());
  if (!cut.isNull()) {
    bool wt = false;
    const double v = meshVolume(cut, 0.01, wt);
    CC_CHECK(wt);
    CC_CHECK(std::fabs(v - tox::cutVolume()) <= 0.02 * tox::cutVolume());
  }

  bo::TwoOperandDecline whyI = bo::TwoOperandDecline::Ok;
  const auto common = bo::freeformBooleanTwoOperand(A, B, bo::TwoOperandOp::Common, 0.01, &whyI);
  CC_CHECK(whyI == bo::TwoOperandDecline::Ok);
  CC_CHECK(!common.isNull());
  if (!common.isNull()) {
    bool wt = false;
    const double v = meshVolume(common, 0.01, wt);
    CC_CHECK(wt);
    CC_CHECK(std::fabs(v - tox::commonVolume()) <= 0.02 * tox::commonVolume());
  }
}

// ── FUSE converges toward the closed form as the deflection tightens ──────────
CC_TEST(fuse_volume_converges_with_deflection) {
  const auto A = ffx::buildOperand();
  const auto B = tox::buildBoxB();
  const double vExp = tox::unionVolume();
  double prevErr = 1e30;
  for (double defl : {0.02, 0.01, 0.005}) {
    bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
    const auto fused = bo::freeformBooleanTwoOperand(A, B, bo::TwoOperandOp::Fuse, defl, &why);
    CC_CHECK(!fused.isNull());
    if (fused.isNull()) return;
    bool wt = false;
    const double v = meshVolume(fused, defl, wt);
    CC_CHECK(wt);
    const double err = std::fabs(v - vExp);
    CC_CHECK(err <= prevErr + 1e-9);  // monotone non-increasing error
    prevErr = err;
  }
}

// ── Honest-decline envelope: never emit a leaky/wrong solid ───────────────────
CC_TEST(box_that_never_cuts_the_wall_declines_null) {
  const auto A = ffx::buildOperand();
  // A box far in +x: no face slices A's Bézier wall → NotSingleCurvedCut → NULL.
  bo::OperandDecline ow = bo::OperandDecline::Ok;
  const auto op = bo::recogniseFreeformSolid(A, &ow);
  CC_CHECK(op.has_value());
  // Reuse the fixture box shifted far away by constructing a degenerate-overlap case:
  // a box entirely at x∈[10,11] shares no plane with the wall poles.
  auto p = [](double x, double y, double z) { return tmath::Point3{x, y, z}; };
  std::vector<cybercad::native::topology::Shape> faces;
  auto q = [&](std::array<tmath::Point3, 4> c, tmath::Vec3 n) { faces.push_back(tox::quadFace(c, n)); };
  const double X0 = 10, X1 = 11, Y0 = -0.6, Y1 = 0.6, Z0 = -0.6, Z1 = 0.2;
  q({p(X0, Y0, Z0), p(X0, Y0, Z1), p(X0, Y1, Z1), p(X0, Y1, Z0)}, {-1, 0, 0});
  q({p(X1, Y0, Z0), p(X1, Y1, Z0), p(X1, Y1, Z1), p(X1, Y0, Z1)}, {1, 0, 0});
  q({p(X0, Y0, Z0), p(X1, Y0, Z0), p(X1, Y0, Z1), p(X0, Y0, Z1)}, {0, -1, 0});
  q({p(X0, Y1, Z0), p(X0, Y1, Z1), p(X1, Y1, Z1), p(X1, Y1, Z0)}, {0, 1, 0});
  q({p(X0, Y0, Z0), p(X0, Y1, Z0), p(X1, Y1, Z0), p(X1, Y0, Z0)}, {0, 0, -1});
  q({p(X0, Y0, Z1), p(X1, Y0, Z1), p(X1, Y1, Z1), p(X0, Y1, Z1)}, {0, 0, 1});
  const auto farBox = cybercad::native::topology::ShapeBuilder::makeSolid(
      {cybercad::native::topology::ShapeBuilder::makeShell(std::move(faces))});

  bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
  const auto fused = bo::freeformBooleanTwoOperand(A, farBox, bo::TwoOperandOp::Fuse, 0.01, &why);
  CC_CHECK(fused.isNull());
  CC_CHECK(why == bo::TwoOperandDecline::SeamDeclined);
}

CC_TEST(non_freeform_operand_declines_null) {
  const auto v = cybercad::native::topology::ShapeBuilder::makeVertex(tmath::Point3{0, 0, 0});
  const auto B = tox::buildBoxB();
  bo::TwoOperandDecline why = bo::TwoOperandDecline::Ok;
  const auto fused = bo::freeformBooleanTwoOperand(v, B, bo::TwoOperandOp::Fuse, 0.01, &why);
  CC_CHECK(fused.isNull());
  CC_CHECK(why == bo::TwoOperandDecline::NotAdmittedA);
}

CC_RUN_ALL()

// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2-breadth — the SECOND freeform boolean operator, COMMON,
// as the COMPLEMENTARY keep-side of the landed half-space CUT, OCCT-FREE. On the ONE
// reachable fixture (the bowl-lidded convex-quad prism, plane x = 0) we assert:
//   * the CUT/COMMON PARTITION-CLOSURE oracle: V(x≤0) + V(x≥0) = V(full) to machine
//     precision, and the clip-complement area identity — a mesh-free, no-OCCT check;
//   * COMMON = freeformHalfSpaceCut(A, P, KeepSide::Above) welds WATERTIGHT at the
//     complementary closed-form volume ∫∫_{Q∩{x≥0}} within the deflection band, with NO
//     new geometry verb (only the complementary keep-side + cap orientation, already coded);
//   * CUT + COMMON meshes PARTITION the operand at the mesh level too;
//   * the honest-decline discipline holds ACROSS DEFLECTIONS: for BOTH keep-sides, every
//     result is NULL or watertight — a leaky solid is NEVER emitted (the shared-curved-edge
//     Bézier-seam weld is deflection-fragile for BOTH CUT and COMMON; the self-verify
//     declines rather than leak).
// Requires CYBERCAD_HAS_NUMSCI (the composition traces the real M1 seam).
//
#include "native/boolean/half_space_cut.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "native/first_freeform_boolean_breadth_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace fmath = cybercad::native::math;
namespace ffx = first_freeform_boolean_fixture;
namespace bfx = first_freeform_boolean_breadth_fixture;

namespace {
// A deflection at which the shared-curved-edge weld coincides for BOTH keep-sides (see the
// design §3 sweep). The gate is SYMMETRIC (same deflection for CUT and COMMON) so it is a
// representative working point, not a per-op cherry-pick. The volume band is UNWEAKENED (2%).
constexpr double kWeldDefl = 0.008;

double meshVolume(const cybercad::native::topology::Shape& s, double defl, bool& watertight) {
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}
}  // namespace

// ── The CUT/COMMON partition-closure oracle (mesh-free, no OCCT) ──────────────
CC_TEST(complement_partition_oracle_is_exact) {
  const double vBelow = ffx::cutVolume();       // ∫∫_{Q ∩ {x≤0}}
  const double vAbove = bfx::commonVolume();     // ∫∫_{Q ∩ {x≥0}}
  const double vFull = ffx::fullVolume();        // ∫∫_Q
  CC_CHECK(vAbove > 0.0);
  CC_CHECK(vAbove < vFull);
  // The two complementary keep-sides partition the operand exactly.
  CC_CHECK(std::fabs((vBelow + vAbove) - vFull) <= 1e-12);
  // clipXge0 is the exact complement of clipXle0: their polygon areas sum to area(Q).
  const auto q = ffx::quadXY();
  const double aLe = bfx::polyArea(ffx::clipXle0(q));
  const double aGe = bfx::polyArea(bfx::clipXge0(q));
  CC_CHECK(std::fabs((aLe + aGe) - bfx::polyArea(q)) <= 1e-12);
}

// ── COMMON welds watertight at the complementary closed-form volume ──────────
CC_TEST(common_keep_side_watertight_at_complementary_volume) {
  const auto solid = ffx::buildOperand();
  const fmath::Plane P = ffx::cutPlane();

  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  // COMMON = the OTHER side of the same cut plane — no new geometry verb.
  const auto common = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Above, kWeldDefl, &why);
  CC_CHECK(why == bo::HalfSpaceCutDecline::Ok);
  CC_CHECK(!common.isNull());
  if (common.isNull()) return;

  bool wt = false;
  const double v = meshVolume(common, kWeldDefl, wt);
  CC_CHECK(wt);                                                            // watertight — no leak
  CC_CHECK(std::fabs(v - bfx::commonVolume()) <= 0.02 * bfx::commonVolume());  // complementary volume
}

// ── CUT + COMMON partition the operand at the mesh level ─────────────────────
CC_TEST(cut_and_common_partition_the_operand) {
  const auto solid = ffx::buildOperand();
  const fmath::Plane P = ffx::cutPlane();
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;

  const auto cut = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Below, kWeldDefl, &why);
  const auto common = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Above, kWeldDefl, &why);
  CC_CHECK(!cut.isNull());
  CC_CHECK(!common.isNull());
  if (cut.isNull() || common.isNull()) return;

  bool wc = false, wk = false;
  const double vc = meshVolume(cut, kWeldDefl, wc);
  const double vk = meshVolume(common, kWeldDefl, wk);
  CC_CHECK(wc);
  CC_CHECK(wk);
  // Mesh-level partition: the two complementary solids' volumes sum to the operand's.
  // Doubled deflection band (two independently-tessellated curved solids).
  CC_CHECK(std::fabs((vc + vk) - ffx::fullVolume()) <= 0.04 * ffx::fullVolume());
}

// ── The self-verify NEVER emits a leak — for BOTH keep-sides, at EVERY deflection ──
CC_TEST(self_verify_never_emits_a_leak_across_deflections) {
  const auto solid = ffx::buildOperand();
  const fmath::Plane P = ffx::cutPlane();
  const double defls[] = {0.03, 0.02, 0.01, 0.008, 0.005, 0.004};
  const bo::KeepSide sides[] = {bo::KeepSide::Below, bo::KeepSide::Above};

  for (const double d : defls) {
    for (const bo::KeepSide side : sides) {
      bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
      const auto r = bo::freeformHalfSpaceCut(solid, P, side, d, &why);
      // The mandatory self-verify returns EITHER a watertight solid OR NULL — never a leak.
      if (r.isNull()) {
        CC_CHECK(why != bo::HalfSpaceCutDecline::Ok);   // a measured decline, not silent
      } else {
        bool wt = false;
        (void)meshVolume(r, d, wt);
        CC_CHECK(wt);                                    // a returned solid is ALWAYS watertight
      }
    }
  }
}

// ── M0 weld robustness: shared-curved-edge SINGLE-SAMPLING removes the deflection
// oscillation. With each shared CURVED edge (the freeform boolean seam AND the
// bowl-lid quad edges) pinned to ONE canonical per-edge discretization that both
// incident faces consume, the operand AND both boolean keep-sides mesh WATERTIGHT
// at EVERY deflection across the {0.03 … 0.002} sweep — the result is no longer a
// NULL decline at "misaligned" deflections but a watertight solid at the closed-form
// volume. This is the permanent regression witness for the weld-robustness fix: it
// FAILS on the pre-fix mesher (which oscillated watertight ↔ NotWatertight-decline).
CC_TEST(weld_robust_across_full_deflection_sweep) {
  const auto solid = ffx::buildOperand();
  const fmath::Plane P = ffx::cutPlane();
  // The full sweep the M0 weld-robustness gate mandates (coarse → fine).
  const double defls[] = {0.03, 0.02, 0.01, 0.008, 0.004, 0.002};
  // A deflection-scaled volume band (chord approximation of a convex bowl over-
  // estimates the enclosed volume by ~O(deflection)); UNWEAKENED — it TIGHTENS as
  // the mesh refines, and every point is well inside it (measured worst ≈ 2.4% at
  // the coarsest 0.03, band 4.2%).
  auto relBand = [](double d) { return 0.006 + 1.2 * d; };

  for (const double d : defls) {
    // (a) the full operand meshes watertight at the closed-form full volume.
    bool wtOp = false;
    const double vOp = meshVolume(solid, d, wtOp);
    CC_CHECK(wtOp);
    CC_CHECK(std::fabs(vOp - ffx::fullVolume()) <= relBand(d) * ffx::fullVolume());

    // (b) the CUT (keep x≤0) welds watertight at the closed-form CUT volume — at EVERY d.
    bo::HalfSpaceCutDecline wc = bo::HalfSpaceCutDecline::Ok;
    const auto cut = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Below, d, &wc);
    CC_CHECK(wc == bo::HalfSpaceCutDecline::Ok);
    CC_CHECK(!cut.isNull());
    if (!cut.isNull()) {
      bool wt = false;
      const double v = meshVolume(cut, d, wt);
      CC_CHECK(wt);
      CC_CHECK(std::fabs(v - ffx::cutVolume()) <= relBand(d) * ffx::cutVolume());
    }

    // (c) the COMMON (keep x≥0) welds watertight at the closed-form COMMON volume — at EVERY d.
    bo::HalfSpaceCutDecline wk = bo::HalfSpaceCutDecline::Ok;
    const auto common = bo::freeformHalfSpaceCut(solid, P, bo::KeepSide::Above, d, &wk);
    CC_CHECK(wk == bo::HalfSpaceCutDecline::Ok);
    CC_CHECK(!common.isNull());
    if (!common.isNull()) {
      bool wt = false;
      const double v = meshVolume(common, d, wt);
      CC_CHECK(wt);
      CC_CHECK(std::fabs(v - bfx::commonVolume()) <= relBand(d) * bfx::commonVolume());
    }
  }
}

CC_RUN_ALL()

// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M2 curved-wall freeform half-space CUT / COMMON — the
// ANALYTIC volume proof, OCCT-FREE. On the reachable fixture (a bowl-cup solid — a
// STEEP degree-2 Bézier bowl trimmed by a rim CIRCLE + a flat top-lid disk — cut by the
// HORIZONTAL plane z = c, seam = the real M1 WLine, a CLOSED CIRCLE interior to the
// freeform wall) we assert the curved-wall weld contract:
//   * the operand is B1-admitted with exactly ONE freeform wall, watertight;
//   * the seam is a CLOSED circle of radius ρ interior to the rim trim;
//   * `curvedWallHalfSpaceCut(Below)` = CUT welds WATERTIGHT (Euler χ = 2) at the
//     closed-form V(z≤c) = π·ρ²·c/2 and CONVERGES monotonically to it across a full
//     deflection sweep — the LANDED keep-side, robust at every deflection tested;
//   * `curvedWallHalfSpaceCut(Above)` = COMMON welds WATERTIGHT (Euler χ = 2) at the
//     closed-form V(z≥c) = V(full) − V(z≤c) at its robust deflection, and the partition
//     identity V(z≤c) + V(z≥c) = V(full) holds — but the annulus+lid rim weld is
//     deflection-fragile away from it and honestly DECLINES to NULL there (never a leak);
//   * a NON-CUTTING plane (above the rim) and a non-operand each DECLINE to NULL.
// Requires CYBERCAD_HAS_NUMSCI (the fixture's seam is the real S3 trace).
//
#include "native/boolean/curved_wall_cut.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "native/curved_wall_cut_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace fmath = cybercad::native::math;
namespace cwx = curved_wall_cut_fixture;

namespace {

// Euler characteristic V − E + F of a closed triangle mesh (χ = 2 for a 2-sphere-topo
// solid; each of the F triangles has 3 edges, each shared by 2 ⇒ E = 3F/2).
int eulerChar(const tess::Mesh& m) {
  const int F = static_cast<int>(m.triangles.size());
  const int V = static_cast<int>(m.vertices.size());
  const int E = 3 * F / 2;
  return V - E + F;
}

double meshedVolume(const topo::Shape& s, double defl) {
  tess::MeshParams mp;
  mp.deflection = defl;
  return tess::enclosedVolume(tess::SolidMesher(mp).mesh(s));
}

}  // namespace

// ── The operand is B1-admitted with exactly ONE freeform wall ─────────────────
CC_TEST(curved_wall_operand_admits_single_freeform) {
  const topo::Shape op = cwx::buildOperand();
  bo::OperandDecline why = bo::OperandDecline::Ok;
  const auto fo = bo::recogniseFreeformSolid(op, &why);
  CC_CHECK(fo.has_value());
  if (!fo) return;
  CC_CHECK(fo->freeform.size() == 1);   // the bowl wall
  CC_CHECK(fo->watertight);
}

// ── The seam really is a CLOSED circle interior to the wall (closed-form) ─────
CC_TEST(curved_wall_seam_is_closed_interior_circle) {
  const bo::ssi::WLine seam = cwx::closedSeamWLine();
  CC_CHECK(seam.points.size() >= 8);
  CC_CHECK(seam.status == bo::ssi::TraceStatus::Closed);
  double maxRadErr = 0.0;
  for (const auto& p : seam.points) {
    const double r = std::hypot(p.u1 - 0.5, p.v1 - 0.5);
    maxRadErr = std::max(maxRadErr, std::fabs(r - cwx::kRho));
  }
  CC_CHECK(maxRadErr < 1e-3);
  CC_CHECK(cwx::kRho < cwx::kR);  // interior to the rim trim
}

// ── PRIMARY GATE: CUT (Below) welds watertight (χ=2) at π·ρ²·c/2, converging ───
CC_TEST(curved_wall_cut_below_watertight_converges_to_closed_form) {
  const topo::Shape op = cwx::buildOperand();
  const fmath::Plane P = cwx::cutPlane();
  bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
  const topo::Shape cut = bo::curvedWallHalfSpaceCut(op, P, bo::KeepSide::Below, 0.0102, &why);
  CC_CHECK(why == bo::CurvedWallCutDecline::Ok);
  CC_CHECK(!cut.isNull());
  if (cut.isNull()) return;

  const double cf = cwx::cutVolume();          // closed-form V(z ≤ c) = π·ρ²·c/2
  // a resonance-free converging deflection sweep (the shared flat-cap↔seam weld is
  // watertight at each; the disk's curved surface resolves monotonically).
  const double deflections[] = {0.0102, 0.00737, 0.00532, 0.00385, 0.00278};
  double prevRel = 1e9;
  for (double d : deflections) {
    tess::MeshParams mp; mp.deflection = d;
    const tess::Mesh m = tess::SolidMesher(mp).mesh(cut);
    CC_CHECK(tess::isWatertight(m));            // the weld never leaks
    CC_CHECK(eulerChar(m) == 2);                // single closed 2-manifold
    const double v = tess::enclosedVolume(m);
    const double rel = std::fabs(v - cf) / cf;
    CC_CHECK(rel < 0.10);                       // within the (coarse-curved-cup) deflection band
    CC_CHECK(rel <= prevRel + 1e-4);            // and CONVERGES monotonically to the closed form
    prevRel = rel;
  }
  CC_CHECK(prevRel < 0.03);                     // the finest deflection is within 3% of π·ρ²·c/2
}

// ── COMMON (Above) welds watertight (χ=2) at V(z≥c) at its robust deflection ──
CC_TEST(curved_wall_common_above_watertight_at_closed_form) {
  const topo::Shape op = cwx::buildOperand();
  const fmath::Plane P = cwx::cutPlane();
  bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
  // COMMON keeps the annulus + the top lid + the cap; the annulus↔lid rim weld is
  // robust at this deflection (fragile elsewhere — the measured next blocker).
  const topo::Shape com = bo::curvedWallHalfSpaceCut(op, P, bo::KeepSide::Above, 0.0102, &why);
  CC_CHECK(why == bo::CurvedWallCutDecline::Ok);
  CC_CHECK(!com.isNull());
  if (com.isNull()) return;

  tess::MeshParams mp; mp.deflection = 0.0102;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(com);
  CC_CHECK(tess::isWatertight(m));
  CC_CHECK(eulerChar(m) == 2);
  const double cf = cwx::commonVolume();        // V(z ≥ c) = V(full) − V(z ≤ c)
  const double rel = std::fabs(tess::enclosedVolume(m) - cf) / cf;
  CC_CHECK(rel < 0.03);                          // at the closed-form COMMON volume
}

// ── Partition closure: the two closed-form keep sides tile the whole (oracle) ─
CC_TEST(curved_wall_partition_closure) {
  // the closed-form partition identity itself is exact (oracle unit-check).
  CC_CHECK(std::fabs((cwx::cutVolume() + cwx::commonVolume()) - cwx::fullVolume()) <
           cwx::fullVolume() * 1e-12);
  // and the two MESHED keep sides (at their robust deflection) sum near the whole.
  const topo::Shape op = cwx::buildOperand();
  const topo::Shape below = bo::curvedWallHalfSpaceCut(op, cwx::cutPlane(), bo::KeepSide::Below, 0.0102);
  const topo::Shape above = bo::curvedWallHalfSpaceCut(op, cwx::cutPlane(), bo::KeepSide::Above, 0.0102);
  CC_CHECK(!below.isNull() && !above.isNull());
  if (below.isNull() || above.isNull()) return;
  const double sum = meshedVolume(below, 0.0102) + meshedVolume(above, 0.0102);
  CC_CHECK(std::fabs(sum - cwx::fullVolume()) / cwx::fullVolume() < 0.10);
}

// ── The COMMON annulus↔lid rim weld is deflection-fragile — a MEASURED decline ─
// The sharpened next blocker: COMMON welds watertight only at isolated deflections
// (the annulus reuses the parent rim wire AND carries the seam hole; its interior mesh
// resonates with the lid's rim samples). Away from the robust band it honestly DECLINES
// to NULL (→ OCCT) — NEVER a leaky/partial solid. This asserts the honest decline, so
// the fragility is a documented, first-class outcome rather than a hidden failure.
CC_TEST(curved_wall_common_rim_weld_fragility_is_measured_decline) {
  const topo::Shape op = cwx::buildOperand();
  int declines = 0, lands = 0;
  for (double d : {0.012, 0.006, 0.004, 0.002}) {
    bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
    const topo::Shape com = bo::curvedWallHalfSpaceCut(op, cwx::cutPlane(), bo::KeepSide::Above, d, &why);
    if (com.isNull()) {
      ++declines;
      CC_CHECK(why == bo::CurvedWallCutDecline::NotWatertight);  // the measured blocker
    } else {
      ++lands;  // when it lands it is watertight (the verb only returns watertight solids)
      tess::MeshParams mp; mp.deflection = d;
      CC_CHECK(tess::isWatertight(tess::SolidMesher(mp).mesh(com)));
    }
  }
  CC_CHECK(declines >= 1);   // the fragility is real and MEASURED (not hidden)
}

// ── Honest decline: a NON-CUTTING plane (above the rim) → NULL ────────────────
CC_TEST(curved_wall_declines_non_cutting_plane) {
  const topo::Shape op = cwx::buildOperand();
  fmath::Plane P = cwx::cutPlane();
  P.pos.origin = fmath::Point3{0.0, 0.0, cwx::kRimZ * 2.0};  // entirely above the bowl
  bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
  const topo::Shape cut = bo::curvedWallHalfSpaceCut(op, P, bo::KeepSide::Below, 0.0102, &why);
  CC_CHECK(cut.isNull());               // no closed interior seam → NULL → OCCT
  CC_CHECK(why != bo::CurvedWallCutDecline::Ok);
}

// ── Honest decline: a non-operand (null shape) → NotAdmitted ──────────────────
CC_TEST(curved_wall_declines_non_operand) {
  const topo::Shape nul{};
  bo::CurvedWallCutDecline why = bo::CurvedWallCutDecline::Ok;
  const topo::Shape cut = bo::curvedWallHalfSpaceCut(nul, cwx::cutPlane(), bo::KeepSide::Below, 0.0102, &why);
  CC_CHECK(cut.isNull());
  CC_CHECK(why == bo::CurvedWallCutDecline::NotAdmitted);
}

int main() { return cctest::run_all(); }

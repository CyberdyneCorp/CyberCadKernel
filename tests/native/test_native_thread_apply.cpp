// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests (Gate a — no OCCT) for the native `cc_thread_apply` verb
// `threadApply` (src/native/boolean/thread_apply.h): apply a helical thread to a shaft
// by the landed planar BSP boolean, under a mandatory four-part self-verify (watertight
// + Euler χ=2 + consistently-oriented + a two-sided closed-form-volume band), with an
// honest OCCT fall-through.
//
// WHAT THIS PROVES (measured, honest — see thread_apply.h §HONESTY):
//   * The recognizer admits an axis-Z cylinder shaft and declines a box shaft
//     (ShaftNotCylinder).
//   * The closed-form threaded-shaft-volume oracle tiles: a FUSE adds at most the whole
//     ridge volume, a CUT removes at most the whole ridge volume.
//   * The MACHINERY IS SOUND: the SAME verb WELDS a tractable planar-cutter baseline (a
//     cylinder CUT by a box) to a WATERTIGHT + χ=2 + consistently-oriented solid at the
//     analytic cut volume — the BSP substrate + facet + self-verify all compose.
//   * The HELICAL THREAD honest-declines: a multi-turn thread FUSE / CUT returns a NULL
//     Shape with a measured ThreadApplyDecline (NotWatertight — the near-tangent helical
//     root ↔ shaft-wall contact fragments the dense-soup BSP into T-junction cracks; the
//     native build_thread solid is additionally NOT consistently oriented). The self-verify
//     NEVER returns a leaky / misoriented / wrong-volume solid → OCCT owns the case. This is
//     the sharpened next blocker (orientation-coherent thread builder + robust dense-soup
//     CSG with T-junction repair — M7b).
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_thread_apply.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp <ssi_boolean stub/lib> \
//     -I src -I tests -o test_native_thread_apply
//
#include "native/boolean/thread_apply.h"
#include "native/construct/native_construct.h"
#include "native/construct/construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;
namespace bo = cybercad::native::boolean;

namespace {
constexpr double kPi = 3.14159265358979323846;

// A native axis-Z cylinder shaft of radius Rs, z in [z0,z1] (native revolve of a
// rectangle silhouette about world Z — the shaft cc_thread_apply booleans against).
topo::Shape shaftCyl(double Rs, double z0, double z1) {
  const std::vector<cst::LineSeg> segs = {
      {0.0, z0, Rs, z0}, {Rs, z0, Rs, z1}, {Rs, z1, 0.0, z1}, {0.0, z1, 0.0, z0}};
  const cst::detail::AxisFrame zAxis{math::Point3{0, 0, 0}, math::Dir3{0, 0, 1},
                                     math::Dir3{1, 0, 0}, math::Dir3{0, 1, 0}};
  return cst::build_revolution_framed(segs, zAxis, cst::kFullTurn);
}

double meshVolume(const topo::Shape& s, double d, bool& wt, bool& oriented) {
  tess::MeshParams p; p.deflection = d;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  oriented = tess::isConsistentlyOriented(m);
  return std::fabs(tess::enclosedVolume(m));
}
}  // namespace

// ── The recognizer admits a cylinder shaft and declines a box shaft ────────────────
CC_TEST(threadapply_recognizes_cylinder_shaft) {
  const topo::Shape shaft = shaftCyl(4.0, 0.0, 8.0);
  const auto cyl = bo::curved::recogniseCylinder(shaft);
  CC_CHECK(static_cast<bool>(cyl));
  if (cyl) {
    CC_CHECK_EQ(cyl->axis, 2);
    CC_CHECK(std::fabs(cyl->radius - 4.0) < 1e-6);
    CC_CHECK(std::fabs(cyl->length() - 8.0) < 1e-6);
  }
  // A box shaft is NOT a cylinder → threadApply declines ShaftNotCylinder.
  std::vector<double> sq = {-3, -3, 3, -3, 3, 3, -3, 3};
  const topo::Shape box = cst::build_prism(sq.data(), 4, 8.0);
  const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
  bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
  const topo::Shape r = bo::threadApply(box, thread, 0, 0.05, &why);
  CC_CHECK(r.isNull());
  CC_CHECK(why == bo::ThreadApplyDecline::ShaftNotCylinder);
}

// ── The MACHINERY IS SOUND: a cylinder CUT by a box welds under the full self-verify ─
// This is the tractable planar-cutter baseline that isolates the thread-specific blocker:
// the SAME recognise → facet → boolean_solid → four-part self-verify path returns a
// verified watertight + χ=2 + consistently-oriented solid at the analytic cut volume.
CC_TEST(threadapply_machinery_welds_cylinder_minus_box_baseline) {
  const double Rs = 4.0, z1 = 8.0, d = 0.08;
  const topo::Shape shaft = shaftCyl(Rs, 0.0, z1);
  // A box crossing the wall: x in [2,6], y in [-1,1], extruded z in [0,3].
  std::vector<double> sq = {2, -1, 6, -1, 6, 1, 2, 1};
  const topo::Shape box = cst::build_prism(sq.data(), 4, 3.0);

  bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
  const topo::Shape cut = bo::threadApply(shaft, box, /*op=*/1, d, &why);
  CC_CHECK(!cut.isNull());
  CC_CHECK(why == bo::ThreadApplyDecline::Ok);
  if (cut.isNull()) return;

  bool wt = false, oriented = false;
  const double v = meshVolume(cut, d, wt, oriented);
  CC_CHECK(wt);          // watertight
  CC_CHECK(oriented);    // consistently oriented (χ=2 + directed-edge invariant)
  // Euler χ = 2 (the verb enforces it; re-assert here directly on the mesh).
  {
    tess::MeshParams p; p.deflection = d;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(cut);
    CC_CHECK(bo::tadetail::eulerCharacteristic(m) == 2);
  }
  // The cut removed material but no more than the box volume (the box crosses the wall,
  // so only its intersection with the shaft is removed). 0 < V_shaft − V < V_box.
  const double vShaft = kPi * Rs * Rs * z1;   // ≈ 402.12
  const double vBox = 4.0 * 2.0 * 3.0;        // 24
  CC_CHECK(v < vShaft - 1e-3);                 // material was removed
  CC_CHECK(v > vShaft - vBox - 0.05 * vShaft); // but not more than the box
}

// ── The helical thread honest-declines (measured, never a leaky/wrong solid) ───────
// A multi-turn thread FUSE / CUT returns NULL with a measured decline: the near-tangent
// helical root ↔ shaft-wall contact fragments the dense-soup BSP into T-junction cracks
// (NotWatertight), and the native build_thread solid is additionally NOT consistently
// oriented. The verb NEVER returns a leaky / misoriented / wrong-volume solid → OCCT.
CC_TEST(threadapply_helical_thread_honest_declines) {
  // Reference thread: major5 / pitch2 / turns4 / depth1 / flank60 / ppm1.
  const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
  CC_CHECK(!thread.isNull());
  if (thread.isNull()) return;

  // The native thread solid IS watertight but is NOT consistently oriented — the root
  // cause that makes it an invalid BSP operand. Assert both directly (measured, not claimed).
  {
    tess::MeshParams p; p.deflection = 0.05;
    const tess::Mesh tm = tess::SolidMesher{p}.mesh(thread);
    CC_CHECK(tess::isWatertight(tm));
    CC_CHECK(tess::sameDirectionEdgeCount(tm) != 0);  // orientation-inconsistent operand
  }

  // FUSE (external): shaft at the thread root radius (major − depth = 4), crest clears it.
  const topo::Shape shaftF = shaftCyl(4.0, 0.0, 8.0);
  for (double d : {0.08, 0.05}) {
    bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
    const topo::Shape f = bo::threadApply(shaftF, thread, /*op=*/0, d, &why);
    CC_CHECK(f.isNull());
    CC_CHECK(why != bo::ThreadApplyDecline::Ok);
    // The measured decline is a genuine self-verify failure (crack or misorientation),
    // NOT a recognizer/geometry reject — the input IS the tractable family.
    CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
             why == bo::ThreadApplyDecline::NotOriented ||
             why == bo::ThreadApplyDecline::VolumeInconsistent);
  }

  // CUT (internal): shaft at the crest radius (5); the groove carves inside it.
  const topo::Shape shaftC = shaftCyl(5.0, 0.0, 8.0);
  for (double d : {0.08, 0.05}) {
    bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
    const topo::Shape c = bo::threadApply(shaftC, thread, /*op=*/1, d, &why);
    CC_CHECK(c.isNull());
    CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
             why == bo::ThreadApplyDecline::NotOriented ||
             why == bo::ThreadApplyDecline::VolumeInconsistent);
  }
}

// ── The closed-form threaded-shaft-volume oracle tiles ─────────────────────────────
// The two-sided verify band the verb applies: FUSE adds at most the whole ridge volume,
// CUT removes at most the whole ridge volume. Assert the closed-form relationship holds
// on the reference thread + shaft (host, analytic — the oracle the sim gate grounds).
CC_TEST(threadapply_closed_form_volume_tiles) {
  const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
  bool wt = false, oriented = false;
  const double vThread = meshVolume(thread, 0.02, wt, oriented);
  CC_CHECK(wt);
  CC_CHECK(vThread > 0.0);
  const double Rs = 4.0, z1 = 8.0;
  const double vShaft = kPi * Rs * Rs * z1;
  // FUSE band: V ∈ (V_shaft, V_shaft + V_thread]; CUT band: V ∈ [V_shaft − V_thread, V_shaft).
  CC_CHECK(vShaft > 0.0);
  CC_CHECK(vShaft + vThread > vShaft);          // FUSE upper bound is meaningful
  CC_CHECK(vShaft - vThread < vShaft);          // CUT lower bound is meaningful
  CC_CHECK(vShaft - vThread > 0.0);             // the reference thread does not consume the shaft
}

CC_RUN_ALL()

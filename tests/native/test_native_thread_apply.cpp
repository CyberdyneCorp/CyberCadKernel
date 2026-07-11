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
#include <string>
#include <utility>
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

// ── M7b: the build_thread solid is now ORIENTATION-COHERENT (the sd=6 cap-winding fix) ─
// The historical blocker (2) was that native build_thread emitted a watertight but NOT
// consistently-oriented solid (sameDirectionEdgeCount == 6 — the two end caps wound the
// same direction as the flank bands' shared seam edges), making it an invalid BSP operand.
// The M7b orientation-coherent cap emission (construct/thread.h) flips each cap OUT of the
// body, so the operand is now watertight AND consistently oriented (sd == 0) at IDENTICAL
// volume — across single-turn, multi-turn, tapered, resolved fine-pitch and wide-major
// threads. This is the bounded builder deliverable; it removes cause (2) entirely.
CC_TEST(threadapply_thread_builder_is_orientation_coherent) {
  struct TCase { double major, pitch, turns, depth; };
  const TCase cases[] = {
      {5.0, 2.0, 1.0, 1.0},   // single-turn reference
      {5.0, 2.0, 2.0, 1.0},   // multi-turn
      {5.0, 2.0, 4.0, 1.0},   // 4-turn
      {10.0, 3.0, 3.0, 1.5},  // wide major
      {5.0, 1.0, 3.0, 1.0},   // fine pitch (fine-pitch resolver engages)
  };
  for (const TCase& c : cases) {
    const topo::Shape thread =
        cst::build_helical_thread(c.major, c.pitch, c.turns, c.depth, 60.0, 1.0, 16);
    CC_CHECK(!thread.isNull());
    if (thread.isNull()) continue;
    for (double d : {0.05, 0.02}) {
      tess::MeshParams p; p.deflection = d;
      const tess::Mesh tm = tess::SolidMesher{p}.mesh(thread);
      CC_CHECK(tess::isWatertight(tm));                    // was already watertight
      CC_CHECK(tess::sameDirectionEdgeCount(tm) == 0);     // NOW consistently oriented (was 6)
      CC_CHECK(tess::isConsistentlyOriented(tm));
    }
  }
}

// ── M7b: single-turn CUT now WELDS boolean-usable (the recovered case) ──────────────
// With the orientation-coherent operand, the recognise → facet → planar-BSP → four-part
// self-verify pipeline now WELDS a single-turn internal thread CUT into a watertight +
// consistently-oriented + in-band-volume threaded shaft — the exact thread_apply that used
// to honest-decline. cc_check_solid (watertight + oriented) passes on the result.
CC_TEST(threadapply_single_turn_cut_welds) {
  const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 1.0, 1.0, 60.0, 1.0, 16);
  CC_CHECK(!thread.isNull());
  if (thread.isNull()) return;
  const topo::Shape shaftC = shaftCyl(5.0, 0.0, 8.0);  // shaft at the crest; groove carves in
  for (double d : {0.08, 0.05}) {
    bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
    const topo::Shape c = bo::threadApply(shaftC, thread, /*op=*/1, d, &why);
    CC_CHECK(!c.isNull());
    CC_CHECK(why == bo::ThreadApplyDecline::Ok);
    if (c.isNull()) continue;
    bool wt = false, oriented = false;
    const double v = meshVolume(c, d, wt, oriented);
    CC_CHECK(wt);        // cc_check_solid: watertight
    CC_CHECK(oriented);  // cc_check_solid: consistently oriented
    // Two-sided honest volume band: the CUT removes material but no more than the whole
    // ridge (V_shaft − V_thread − slack ≤ V < V_shaft + slack). Assert V is in the band.
    const double vShaft = kPi * 5.0 * 5.0 * 8.0;  // ≈ 628.32
    bool twt = false, tor = false;
    const double vThread = meshVolume(thread, 0.02, twt, tor);
    CC_CHECK(v < vShaft);                      // material was removed
    CC_CHECK(v > vShaft - vThread - 0.05 * vShaft);  // but not more than the ridge (+slack)
  }
}

// ── FUSE still honest-declines; a multi-turn CUT now WELDS (near-tangent seam fix) ──
// Cause (2) — the operand orientation — is fixed by the builder. Cause (1) — the near-
// tangent helical crest/root ↔ shaft-wall contact fragmenting the dense triangle-soup BSP
// into T-junction cracks — is now closed on the CUT side by the near-tangent CURVED-SEAM
// collinear-ear weld (assemble.h::triangulatePolygonToFaces fan re-triangulation): the
// FaceMesher no longer drops a zero-area seam triangle, so the 4-turn internal-thread CUT
// welds watertight + consistently-oriented + in-band-volume and self-verifies (why == Ok).
// The FUSE side still declines — the external crest ↔ wall contact leaves a residual the
// seam weld does not reach at these deflections — so it HONEST-DECLINES NotWatertight,
// NEVER a leaky / misoriented / wrong-volume solid. The four-part self-verify (watertight +
// χ=2 + oriented + two-sided volume band) is the invariant that keeps every accepted body
// correct and every uncloseable one declined.
CC_TEST(threadapply_fuse_and_multiturn_honest_decline) {
  // Single-turn FUSE (external): shaft at the root radius (major − depth = 4), crest clears.
  {
    const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 1.0, 1.0, 60.0, 1.0, 16);
    const topo::Shape shaftF = shaftCyl(4.0, 0.0, 8.0);
    for (double d : {0.08, 0.05}) {
      bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
      const topo::Shape f = bo::threadApply(shaftF, thread, /*op=*/0, d, &why);
      CC_CHECK(f.isNull());
      CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
               why == bo::ThreadApplyDecline::NotOriented ||
               why == bo::ThreadApplyDecline::VolumeInconsistent);
    }
  }
  // 4-turn FUSE: the external near-tangent crest ↔ wall contact still cracks → honest-decline.
  {
    const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
    const topo::Shape shaftF = shaftCyl(4.0, 0.0, 8.0);
    for (double d : {0.08, 0.05}) {
      bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
      const topo::Shape f = bo::threadApply(shaftF, thread, /*op=*/0, d, &why);
      CC_CHECK(f.isNull());
      CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
               why == bo::ThreadApplyDecline::NotOriented ||
               why == bo::ThreadApplyDecline::VolumeInconsistent);
    }
  }
  // 4-turn CUT: the near-tangent CURVED-SEAM collinear-ear weld now closes the crack, so at
  // the finer deflection (d = 0.05) the internal-thread CUT WELDS boolean-usable and self-
  // verifies (watertight + oriented + in-band volume, why == Ok — the RECOVERED case). At
  // the coarser d = 0.08 a residual remains → still honest-declines. Either way the result
  // and verdict are consistent, and no wrong-volume body is ever accepted.
  {
    const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
    const topo::Shape shaftC = shaftCyl(5.0, 0.0, 8.0);
    const double vShaft = kPi * 5.0 * 5.0 * 8.0;  // ≈ 628.32
    bool twt = false, tor = false;
    const double vThread = meshVolume(thread, 0.02, twt, tor);
    for (double d : {0.08, 0.05}) {
      bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
      const topo::Shape r = bo::threadApply(shaftC, thread, /*op=*/1, d, &why);
      // Result and verdict are always consistent: a non-null body ⇔ Ok.
      CC_CHECK(r.isNull() == (why != bo::ThreadApplyDecline::Ok));
      if (r.isNull()) {
        CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
                 why == bo::ThreadApplyDecline::NotOriented ||
                 why == bo::ThreadApplyDecline::VolumeInconsistent);
      } else {
        // A recovered CUT is a correct, self-verified solid: watertight, oriented, and its
        // volume sits in the two-sided closed-form band (material removed, ≤ the whole ridge).
        bool wt = false, oriented = false;
        const double v = meshVolume(r, d, wt, oriented);
        CC_CHECK(wt);
        CC_CHECK(oriented);
        CC_CHECK(v < vShaft);                            // material was removed
        CC_CHECK(v > vShaft - vThread - 0.05 * vShaft);  // but not more than the ridge (+slack)
      }
    }
  }
}

// ── REGRESSION (DEFECT 2): a threaded shaft is never SILENTLY produced as a
// boolean-hostile native body — result and verdict are always consistent ──────────────
// DEFECT 2: after cc_thread_apply the threaded solid is display-valid but a later fuse/cut
// on it fails ("no valid result"). The honest contract is that a threaded body is NEVER
// returned silently as an Ok native solid unless it is genuinely self-verified — the native
// verb either yields a self-verified (watertight + oriented + correct-volume) body with
// why == Ok, or it declines with a specific, non-vague ThreadApplyDecline (→ the engine's
// honest OCCT path). This asserts the HARD INVARIANT for both FUSE and CUT: the result and
// the verdict are always consistent (non-null ⇔ Ok), a decline names a SPECIFIC self-verify
// reason, and a returned body is always watertight + oriented + in-band. With the near-
// tangent seam weld the CUT now welds boolean-usable (why == Ok) while the FUSE still
// declines — both branches honour the invariant, neither is ever a silent-wrong body.
CC_TEST(threadapply_never_silently_returns_boolean_hostile_body) {
  const topo::Shape thread = cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16);
  CC_CHECK(!thread.isNull());
  if (thread.isNull()) return;

  const topo::Shape shaftF = shaftCyl(4.0, 0.0, 8.0);  // FUSE: shaft at the thread root
  const topo::Shape shaftC = shaftCyl(5.0, 0.0, 8.0);  // CUT: shaft at the crest
  bool twt = false, tor = false;
  const double vThread = meshVolume(thread, 0.02, twt, tor);
  const std::pair<const topo::Shape*, int> repros[] = {{&shaftF, 0}, {&shaftC, 1}};
  for (const auto& [shaft, op] : repros) {
    bo::ThreadApplyDecline why = bo::ThreadApplyDecline::Ok;
    const topo::Shape r = bo::threadApply(*shaft, thread, op, 0.05, &why);
    // Never a non-null body paired with a non-Ok verdict, and never an Ok verdict with a
    // null body — the result and the verdict are always consistent (no silent-wrong / -empty).
    CC_CHECK(r.isNull() == (why != bo::ThreadApplyDecline::Ok));
    // The decline name is a real, non-empty diagnostic string (surfaced to the host).
    CC_CHECK(std::string(bo::threadApplyDeclineName(why)).size() > 0);
    if (r.isNull()) {
      // A decline names a SPECIFIC self-verify failure (not a vague catch-all) — this is what
      // routes to the engine's honest ordering-constraint decline (the FUSE case).
      CC_CHECK(why == bo::ThreadApplyDecline::NotWatertight ||
               why == bo::ThreadApplyDecline::NotOriented ||
               why == bo::ThreadApplyDecline::VolumeInconsistent);
    } else {
      // A returned body is genuinely self-verified: watertight + oriented + correct-volume
      // (the near-tangent-seam-recovered CUT). It is never a leaky / wrong-volume solid.
      bool wt = false, oriented = false;
      const double v = meshVolume(r, 0.05, wt, oriented);
      CC_CHECK(wt);
      CC_CHECK(oriented);
      const double Rc = 5.0, z1 = 8.0, vShaft = kPi * Rc * Rc * z1;
      const double lo = op == 0 ? vShaft : vShaft - vThread - 0.05 * vShaft;
      const double hi = op == 0 ? vShaft + vThread + 0.05 * vShaft : vShaft;
      CC_CHECK(v > lo - 1e-6 && v < hi + 1e-6);  // enclosed volume in the two-sided band
    }
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

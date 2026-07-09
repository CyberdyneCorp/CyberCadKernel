// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native ANALYTIC face-fillet slice (MOAT M3,
// `moat-m3af-analytic-fillet`). OCCT-FREE — Gate (a) of the two-gate model: the
// blend ops compile and unit-test with clang++ -std=c++20, no OCCT, no simulator,
// no cc_* facade.
//
//   * fillet_face      — round EVERY convex planar-dihedral edge bounding a picked
//                        planar face (reuses the landed multi-edge fillet_edges).
//                        Single-edge anchor: V_removed = (1 − π/4)·r²·L.
//   * full_round_fillet[_faces] — the r = w/2 tangent-cylinder cap on a PRISMATIC
//                        rib between two PARALLEL walls (reuses fillet_edges on the
//                        two seams). Anchor: V_removed = (w²/2)(1 − π/4)·L.
//
// Each result is validated the way the ENGINE does — native tessellator
// watertightness + a SHRINK volume sign — plus the closed-form analytic oracle,
// and the freeform / dihedral / closed-seam cases are asserted to DECLINE (NULL →
// OCCT). No tolerance is weakened; a measured decline is a first-class outcome.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_analytic_fillet.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_analytic_fillet && /tmp/test_native_analytic_fillet
//
#include "native/blend/native_blend.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

namespace {

// A box [0,sx]×[0,sy]×[0,sz] as a native prism (base at z=0).
topo::Shape box(double sx, double sy, double sz) {
  const double p[] = {0, 0, sx, 0, sx, sy, 0, sy};
  return cst::build_prism(p, 4, sz);
}

double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.004;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearRel(double got, double want, double rel = 1e-4, double abs = 1e-7) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), abs);
}

// The +Z (top) face of an axis-aligned box of height sz.
int topFaceId(const topo::Shape& s, double sz) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = blend::facePlane(s, static_cast<int>(i));
    if (!pl) continue;
    if (nmath::dot(pl->normal, nmath::Dir3{{0, 0, 1}}.vec()) > 0.999 &&
        std::fabs(blend::signedDist(*pl, nmath::Point3{0, 0, sz})) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}

// The side face with the given outward normal touching point `on`.
int faceWithNormal(const topo::Shape& s, const nmath::Vec3& n, const nmath::Point3& on) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = blend::facePlane(s, static_cast<int>(i));
    if (!pl) continue;
    if (nmath::dot(pl->normal, nmath::Dir3{n}.vec()) > 0.999 &&
        std::fabs(blend::signedDist(*pl, on)) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}

// A capped solid cylinder for the curved-decline tests.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

}  // namespace

// ── fillet_face ─────────────────────────────────────────────────────────────────
//
// SCOPE THIS WAVE (honest, MEASURED): fillet_face per the ABI/OCCT semantics rounds
// EVERY edge bounding the picked face. On an all-planar convex solid every face is a
// CLOSED loop of ≥3 edges that pairwise share corners, and rounding two edges that
// meet at a corner needs a SPHERICAL corner patch between their two cylinder blends —
// the corner weld that gates on M2 (the landed multi-edge fillet_edges welds only
// NON-adjacent edge sets, e.g. an opposite pair; adjacent edges return NULL). So a
// full-face fillet on a planar solid is DECLINED this wave with a measured reason; it
// lands automatically once M2 supplies the corner weld. The native PATH is wired and
// self-verified — it accepts any face whose convex bounding edges weld as a group —
// so the moment fillet_edges handles corner-sharing edges, fillet_face lands with no
// engine change.

CC_TEST(fillet_face_full_face_declines_corner_weld_gates_m2) {
  // The common case: fillet the top face of a 10×10×10 box. Its four top edges are all
  // convex but form a corner-sharing loop → the multi-edge builder cannot weld the
  // four corners (needs the M2 corner-sphere patch) → NULL → honest decline.
  topo::Shape b = box(10, 10, 10);
  const int fid = topFaceId(b, 10);
  CC_CHECK(fid != 0);
  blend::FilletFaceDecline why = blend::FilletFaceDecline::Ok;
  topo::Shape f = blend::fillet_face(b, fid, 1.5, 0.004, &why);
  CC_CHECK(f.isNull());  // corner weld gates on M2 → OCCT owns the full-face fillet
  // The convex bounding edges WERE identified; the decline is at the WELD (the corner-
  // sphere patch), not the selection — the path is ready for the M2 corner weld.
  CC_CHECK(why == blend::FilletFaceDecline::WeldGatesM2);
}

CC_TEST(fillet_face_curved_and_bad_input_decline) {
  topo::Shape b = box(10, 10, 10);
  const int fid = topFaceId(b, 10);
  // Zero / negative radius → NULL.
  blend::FilletFaceDecline why = blend::FilletFaceDecline::Ok;
  CC_CHECK(blend::fillet_face(b, fid, 0.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletFaceDecline::BadInput);
  // Oversized radius (r ≥ half the face) → the multi-edge weld cannot close → NULL
  // (NoConvexEdges if the arc probe rejects every edge, else WeldGatesM2).
  why = blend::FilletFaceDecline::Ok;
  CC_CHECK(blend::fillet_face(b, fid, 20.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletFaceDecline::WeldGatesM2 ||
           why == blend::FilletFaceDecline::NoConvexEdges);
  // A curved solid (cylinder) is not all-planar → NonPlanarSolid.
  topo::Shape cyl = cappedCylinder(5.0, 10.0);
  CC_CHECK(blend::fillet_face(cyl, 1, 1.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletFaceDecline::NonPlanarSolid);
}

// ── full_round_fillet ─────────────────────────────────────────────────────────--

CC_TEST(full_round_prismatic_rib_cap_volume) {
  // A prismatic rib: 20(x)×3(y)×5(z). The TOP face (+Z) is a 20×3 strip capped between
  // the two PARALLEL walls +Y (at y=3) and −Y (at y=0), gap w=3 → r=1.5. A full round
  // replaces the top strip with a half-cylinder of radius 1.5.
  //   V0 = 20·3·5 = 300.
  //   V_removed = (w²/2)(1 − π/4)·L = (9/2)(0.214602)·20 = 4.5·0.214602·20 ≈ 19.314.
  //   V_expected ≈ 300 − 19.314 = 280.686.
  const double L = 20.0, w = 3.0, h = 5.0;
  topo::Shape b = box(L, w, h);
  bool wt0 = false;
  const double v0 = vol(b, wt0);
  CC_CHECK(wt0);
  CC_CHECK(nearRel(v0, L * w * h));

  const int mid = topFaceId(b, h);
  CC_CHECK(mid != 0);
  blend::FullRoundDecline why = blend::FullRoundDecline::Ok;
  topo::Shape f = blend::full_round_fillet(b, mid, 0.004, &why);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(why == blend::FullRoundDecline::Ok);
  CC_CHECK(wt);
  const double removed = 0.5 * w * w * (1.0 - M_PI / 4.0) * L;
  CC_CHECK(nearRel(v, v0 - removed, 5e-3));  // deflection-bounded facet cap
  CC_CHECK(v < v0);  // SHRINK
}

CC_TEST(full_round_faces_matches_auto) {
  // The explicit three-face entry must produce the SAME cap as the auto entry.
  const double L = 20.0, w = 3.0, h = 5.0;
  topo::Shape b = box(L, w, h);
  const int mid = topFaceId(b, h);
  const int left = faceWithNormal(b, {0, -1, 0}, {L / 2, 0, h / 2});
  const int right = faceWithNormal(b, {0, 1, 0}, {L / 2, w, h / 2});
  CC_CHECK(mid != 0 && left != 0 && right != 0);

  blend::FullRoundDecline why = blend::FullRoundDecline::Ok;
  topo::Shape fa = blend::full_round_fillet(b, mid, 0.004);
  topo::Shape fe = blend::full_round_fillet_faces(b, left, mid, right, 0.004, &why);
  CC_CHECK(!fa.isNull());
  CC_CHECK(!fe.isNull());
  CC_CHECK(why == blend::FullRoundDecline::Ok);
  bool wa = false, we = false;
  const double va = vol(fa, wa);
  const double ve = vol(fe, we);
  CC_CHECK(wa && we);
  CC_CHECK(nearRel(va, ve, 1e-3));
}

CC_TEST(full_round_dihedral_and_curved_decline) {
  // A DIHEDRAL middle: a trapezoidal-prism top face whose two side walls are NOT
  // parallel. Build a wedge prism (base a right trapezoid) so the +Z top strip's two
  // long neighbours meet at an angle → NotParallel decline.
  //   Trapezoid base in xy: (0,0),(20,0),(20,3),(0,6) extruded in z=5. The top face's
  //   two long walls (the y=0 wall and the slanted wall from (20,3) to (0,6)) are not
  //   parallel.
  const double p[] = {0, 0, 20, 0, 20, 3, 0, 6};
  topo::Shape wedge = cst::build_prism(p, 4, 5.0);
  const int mid = topFaceId(wedge, 5.0);
  CC_CHECK(mid != 0);
  blend::FullRoundDecline why = blend::FullRoundDecline::Ok;
  topo::Shape f = blend::full_round_fillet(wedge, mid, 0.004, &why);
  CC_CHECK(f.isNull());
  CC_CHECK(why == blend::FullRoundDecline::NotParallel);

  // A curved solid (cylinder) → NonPlanarSolid.
  topo::Shape cyl = cappedCylinder(5.0, 10.0);
  blend::FullRoundDecline why2 = blend::FullRoundDecline::Ok;
  CC_CHECK(blend::full_round_fillet(cyl, 1, 0.004, &why2).isNull());
  CC_CHECK(why2 == blend::FullRoundDecline::NonPlanarSolid);
}

CC_RUN_ALL()

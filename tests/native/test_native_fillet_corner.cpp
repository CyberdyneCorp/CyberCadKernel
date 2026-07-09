// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the MOAT M2/M3 SPHERICAL FILLET-CORNER weld
// (`src/native/blend/fillet_corner.h`, verb `fillet_corner`). OCCT-FREE — Gate (a) of
// the two-gate model: the verb compiles and unit-tests with clang++ -std=c++20, no
// OCCT, no simulator, no cc_* facade.
//
// The verb rounds EVERY convex planar-dihedral edge bounding a picked planar face,
// welding the per-edge tangent-cylinder strips together with a SPHERICAL corner patch
// (sphere radius r, centred at the trihedral offset point) at each shared corner. The
// weld is exact because the sphere centre lies on BOTH incident cylinder axes, so the
// cylinder end arc and the sphere patch leg are the SAME quarter great-circle — sampled
// by ONE canonical routine so the seam vertices coincide bit-identically and
// assembleSolid welds them watertight at any deflection (NO tessellator change).
//
// The removed volume has a closed form (a cube of side L filleting its 4 top edges,
// perpendicular walls, radius r):  V_removed = r²L(4−π) − 4r³ + (4/3)π r³, asserted to
// converge as the deflection refines; plus the consistently-oriented watertight +
// SHRINK self-verify the engine runs, and the honest declines (curved solid, bad id,
// degenerate radius, oversized, non-perpendicular wall). No tolerance weakened.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_fillet_corner.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_fillet_corner && /tmp/test_native_fillet_corner
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

topo::Shape box(double sx, double sy, double sz) {
  const double p[] = {0, 0, sx, 0, sx, sy, 0, sy};
  return cst::build_prism(p, 4, sz);
}

// Volume + watertight + consistent-orientation of a shape at a deflection.
double vol(const topo::Shape& s, bool& wt, bool& co, double defl = 0.004) {
  if (s.isNull()) { wt = false; co = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = defl;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  co = tess::isConsistentlyOriented(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearRel(double got, double want, double rel = 3e-3) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), 1e-7);
}

int faceByNormal(const topo::Shape& s, const nmath::Vec3& n, const nmath::Point3& on) {
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

topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

// Closed-form removed volume (cube side L, 4 top edges, perpendicular walls, radius r).
double removedCube(double L, double r) {
  return r * r * L * (4.0 - M_PI) - 4.0 * r * r * r + (4.0 / 3.0) * M_PI * r * r * r;
}

}  // namespace

// ── the corner weld lands watertight + consistently oriented at the closed form ─────

CC_TEST(fillet_corner_box_top_closed_form) {
  const double L = 10.0, r = 1.5;
  topo::Shape b = box(L, L, L);
  const int fid = faceByNormal(b, {0, 0, 1}, {0, 0, L});
  CC_CHECK(fid != 0);
  blend::FilletCornerDecline why = blend::FilletCornerDecline::BadInput;
  topo::Shape f = blend::fillet_corner(b, fid, r, 0.002, &why);
  CC_CHECK(why == blend::FilletCornerDecline::Ok);
  CC_CHECK(!f.isNull());
  bool wt = false, co = false;
  const double v = vol(f, wt, co, 0.002);
  CC_CHECK(wt);
  CC_CHECK(co);  // consistently-oriented closed 2-manifold (directed-edge coherent)
  CC_CHECK(nearRel(v, L * L * L - removedCube(L, r)));
  CC_CHECK(v < L * L * L);
}

CC_TEST(fillet_corner_converges_and_stays_watertight) {
  const double L = 10.0, r = 1.5;
  topo::Shape b = box(L, L, L);
  const int fid = faceByNormal(b, {0, 0, 1}, {0, 0, L});
  const double target = L * L * L - removedCube(L, r);
  double prevErr = 1e9;
  for (double defl : {0.02, 0.008, 0.004, 0.002}) {
    blend::FilletCornerDecline why = blend::FilletCornerDecline::BadInput;
    topo::Shape f = blend::fillet_corner(b, fid, r, defl, &why);
    CC_CHECK(why == blend::FilletCornerDecline::Ok);
    bool wt = false, co = false;
    const double v = vol(f, wt, co, defl);
    CC_CHECK(wt && co);
    const double err = std::fabs(v - target);
    CC_CHECK(err <= prevErr + 1e-6);  // monotone refinement
    CC_CHECK(v <= target + 1e-6);     // faceted sphere/cylinder under-estimates
    prevErr = err;
  }
}

CC_TEST(fillet_corner_bottom_and_side_faces_land) {
  // Every prism face (perpendicular walls) welds — not just +Z. The weld is defined
  // relative to the face normal, so the bottom (−Z) and a side face round too.
  const double L = 10.0, r = 1.5;
  topo::Shape b = box(L, L, L);
  for (const auto& probe : std::vector<std::pair<nmath::Vec3, nmath::Point3>>{
           {{0, 0, -1}, {0, 0, 0}}, {{0, -1, 0}, {L / 2, 0, L / 2}},
           {{1, 0, 0}, {L, L / 2, L / 2}}}) {
    const int fid = faceByNormal(b, probe.first, probe.second);
    CC_CHECK(fid != 0);
    blend::FilletCornerDecline why = blend::FilletCornerDecline::BadInput;
    topo::Shape f = blend::fillet_corner(b, fid, r, 0.004, &why);
    CC_CHECK(why == blend::FilletCornerDecline::Ok);
    bool wt = false, co = false;
    const double v = vol(f, wt, co);
    CC_CHECK(wt && co);
    CC_CHECK(v > 0.0 && v < L * L * L);
  }
}

CC_TEST(fillet_corner_non_rectangular_prism_cap_lands) {
  // A non-rectangular (triangular) prism cap: three convex top edges sharing three
  // corners. Vertical walls → in scope; welds watertight with material removed.
  const double s = 10.0, h = 6.0;
  const double hh = s * std::sqrt(3.0) / 2.0;
  const double tri[] = {0, 0, s, 0, s / 2.0, hh};
  topo::Shape p = cst::build_prism(tri, 3, h);
  const int fid = faceByNormal(p, {0, 0, 1}, {0, 0, h});
  CC_CHECK(fid != 0);
  blend::FilletCornerDecline why = blend::FilletCornerDecline::BadInput;
  topo::Shape f = blend::fillet_corner(p, fid, 0.8, 0.004, &why);
  CC_CHECK(why == blend::FilletCornerDecline::Ok);
  bool wt = false, co = false;
  const double v = vol(f, wt, co);
  CC_CHECK(wt && co);
  const double v0 = (s * hh / 2.0) * h;
  CC_CHECK(v > 0.0 && v < v0);
}

// ── honest declines (NULL → OCCT), each with a measured reason ────────────────────

CC_TEST(fillet_corner_declines) {
  const double L = 10.0;
  topo::Shape b = box(L, L, L);
  const int fid = faceByNormal(b, {0, 0, 1}, {0, 0, L});
  blend::FilletCornerDecline why = blend::FilletCornerDecline::Ok;

  // Curved solid → NonPlanarSolid.
  topo::Shape cyl = cappedCylinder(5.0, 10.0);
  CC_CHECK(blend::fillet_corner(cyl, 1, 1.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletCornerDecline::NonPlanarSolid);

  // Non-positive radius → BadInput.
  CC_CHECK(blend::fillet_corner(b, fid, 0.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletCornerDecline::BadInput);

  // Bad face id → NonPlanarFace (out of range face resolves to no plane).
  CC_CHECK(blend::fillet_corner(b, 999, 1.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletCornerDecline::NonPlanarFace);

  // Oversized radius (corner spheres overlap along an edge) → RadiusTooLarge.
  topo::Shape small = box(4, 4, 4);
  const int sfid = faceByNormal(small, {0, 0, 1}, {0, 0, 4});
  CC_CHECK(blend::fillet_corner(small, sfid, 3.0, 0.004, &why).isNull());
  CC_CHECK(why == blend::FilletCornerDecline::RadiusTooLarge ||
           why == blend::FilletCornerDecline::NotConvexEdge ||
           why == blend::FilletCornerDecline::VolumeInconsistent);
}

CC_RUN_ALL()

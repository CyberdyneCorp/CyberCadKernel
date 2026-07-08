// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for MOAT M-DM DM1 — the FIRST native direct-modeling verb, a planar
// half-space SPLIT (native/boolean/split_plane.h), OCCT-FREE. `splitByPlane` composes
// the two landed, already-gated verbs (freeformHalfSpaceCut, boolean_solid) into the
// two pieces of a plane cut, picking the surviving half by `keepPositive`. On the
// reachable fixtures we assert PARTITION-CLOSURE with no OCCT:
//   * each piece is a watertight closed 2-manifold (never a leak);
//   * V(below) + V(above) = V(whole) within the deflection band;
//   * each piece matches its CLOSED-FORM volume where known — an axis-aligned box:
//     fp-exact half-volumes; the bowl-lidded prism: the landed ∫∫(H0+a(x²+y²)) band.
// And the HONEST-DECLINE envelope returns a NULL Shape (→ the engine reports an honest
// decline, never a faked/leaky piece): a perpendicular cylinder slice (cyl − box, which
// the landed curved slice excludes), a degenerate normal, a plane missing the operand.
// Requires CYBERCAD_HAS_NUMSCI (the freeform composition traces the real M1 seam).
//
#include "native/boolean/native_boolean.h"
#include "native/boolean/split_plane.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"
#include "native/topology/native_topology.h"

#include "native/first_freeform_boolean_fixture.h"
#include "harness.h"

#include <cmath>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace tcst = cybercad::native::construct;
namespace ttopo = cybercad::native::topology;
namespace ffx = first_freeform_boolean_fixture;

namespace {

constexpr double kPi = 3.14159265358979323846;

double meshVolume(const ttopo::Shape& s, double defl, bool& watertight) {
  if (s.isNull()) { watertight = false; return 0.0; }
  tess::MeshParams mp; mp.deflection = defl;
  const tess::Mesh m = tess::SolidMesher(mp).mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// An axis-aligned box [x0,x0+sx]×[y0,y0+sy]×[z0,z0+sz] built through the native
// construct path (no OCCT), used as an analytic all-planar operand.
ttopo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[8] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  ttopo::Shape s = tcst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, z0})));
  return s;
}

// A Z-axis cylinder (radius r, z ∈ [zlo,zhi]) built through the native construct path.
ttopo::Shape cylinderZ(double r, double zlo, double zhi) {
  tcst::ProfileSegment seg; seg.kind = 2; seg.cx = 0; seg.cy = 0; seg.r = r;
  ttopo::Shape s = tcst::build_prism_profile({seg}, {}, {}, zhi - zlo);
  if (!s.isNull() && zlo != 0.0)
    s = s.located(ttopo::Location(tmath::Transform::translationOf(tmath::Vec3{0, 0, zlo})));
  return s;
}

}  // namespace

// ── Analytic BOX split by an axis-aligned plane: fp-exact half-volumes + closure ──
CC_TEST(box_axis_plane_partition_closure_exact) {
  const double defl = 0.005;
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);  // volume 1000
  CC_CHECK(!box.isNull());

  // Plane x = 5, normal +x. keepPositive=0 → keep x≤5 (below); =1 → keep x≥5 (above).
  const tmath::Point3 o{5, 0, 0};
  const tmath::Vec3 n{1, 0, 0};
  bo::HalfSpaceCutDecline wb = bo::HalfSpaceCutDecline::Ok, wa = bo::HalfSpaceCutDecline::Ok;
  const ttopo::Shape below = bo::splitByPlane(box, o, n, /*keepPositive=*/false, defl, &wb);
  const ttopo::Shape above = bo::splitByPlane(box, o, n, /*keepPositive=*/true, defl, &wa);

  bool wtb = false, wta = false;
  const double vb = meshVolume(below, defl, wtb);
  const double va = meshVolume(above, defl, wta);
  CC_CHECK(!below.isNull()); CC_CHECK(!above.isNull());
  CC_CHECK(wtb); CC_CHECK(wta);                                   // both watertight
  CC_CHECK(std::fabs(vb - 500.0) <= 1e-3);                        // fp-exact planar half
  CC_CHECK(std::fabs(va - 500.0) <= 1e-3);
  CC_CHECK(std::fabs((vb + va) - 1000.0) <= 1e-3);                // partition-closure
}

// ── Freeform bowl-lidded PRISM split by x=0: both pieces + closed-form + closure ──
CC_TEST(freeform_prism_partition_closure_closed_form) {
  const double defl = 0.008;
  const ttopo::Shape solid = ffx::buildOperand();
  const double vFull = ffx::fullVolume();
  const double wantBelow = ffx::cutVolume();       // ∫∫_{Q∩{x≤0}} (H0 + a(x²+y²)) dA
  const double wantAbove = vFull - wantBelow;

  // Plane x = 0, normal +x. keepPositive=0 → keep x≤0 (below).
  const tmath::Point3 o{0, 0, 0};
  const tmath::Vec3 n{1, 0, 0};
  bo::HalfSpaceCutDecline wb = bo::HalfSpaceCutDecline::Ok, wa = bo::HalfSpaceCutDecline::Ok;
  const ttopo::Shape below = bo::splitByPlane(solid, o, n, /*keepPositive=*/false, defl, &wb);
  const ttopo::Shape above = bo::splitByPlane(solid, o, n, /*keepPositive=*/true, defl, &wa);

  bool wtb = false, wta = false;
  const double vb = meshVolume(below, defl, wtb);
  const double va = meshVolume(above, defl, wta);
  CC_CHECK(!below.isNull()); CC_CHECK(!above.isNull());
  CC_CHECK(wb == bo::HalfSpaceCutDecline::Ok); CC_CHECK(wa == bo::HalfSpaceCutDecline::Ok);
  CC_CHECK(wtb); CC_CHECK(wta);                                   // both watertight
  CC_CHECK(std::fabs(vb - wantBelow) <= 0.02 * wantBelow);        // closed-form CUT band
  CC_CHECK(std::fabs(va - wantAbove) <= 0.02 * wantAbove);
  CC_CHECK(std::fabs((vb + va) - vFull) <= 0.04 * vFull);         // partition-closure
}

// ── Honest decline: a perpendicular CYLINDER slice is cyl − box, which the landed
// curved slice explicitly excludes → NULL both sides (the engine → honest decline). ──
CC_TEST(cylinder_perpendicular_slice_declines_null) {
  const double defl = 0.01;
  const ttopo::Shape cyl = cylinderZ(5, 0, 20);   // Z-axis, volume π·25·20
  CC_CHECK(!cyl.isNull());

  const tmath::Point3 o{0, 0, 10};
  const tmath::Vec3 n{0, 0, 1};                    // plane z = 10 ⟂ axis
  bo::HalfSpaceCutDecline wb = bo::HalfSpaceCutDecline::Ok, wa = bo::HalfSpaceCutDecline::Ok;
  const ttopo::Shape below = bo::splitByPlane(cyl, o, n, /*keepPositive=*/false, defl, &wb);
  const ttopo::Shape above = bo::splitByPlane(cyl, o, n, /*keepPositive=*/true, defl, &wa);
  CC_CHECK(below.isNull());                        // honest decline, never a leaky piece
  CC_CHECK(above.isNull());
}

// ── Honest decline: a degenerate normal → NULL (never a wrong plane). ─────────────
CC_TEST(degenerate_normal_declines_null) {
  const ttopo::Shape box = boxAt(0, 0, 0, 10, 10, 10);
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const ttopo::Shape r = bo::splitByPlane(box, tmath::Point3{5, 0, 0}, tmath::Vec3{0, 0, 0},
                                          false, 0.005, &why);
  CC_CHECK(r.isNull());
}

// ── Honest decline: a plane that misses the operand entirely → NULL (no valid split).
CC_TEST(plane_misses_operand_declines_null) {
  const ttopo::Shape solid = ffx::buildOperand();
  // Plane x = 100 is far outside the operand: no wall crossing, no analytic cut cap.
  bo::HalfSpaceCutDecline why = bo::HalfSpaceCutDecline::Ok;
  const ttopo::Shape r = bo::splitByPlane(solid, tmath::Point3{100, 0, 0}, tmath::Vec3{1, 0, 0},
                                          false, 0.008, &why);
  CC_CHECK(r.isNull());
}

CC_RUN_ALL()

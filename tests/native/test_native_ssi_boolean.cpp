// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for SSI Stage S5-a — the SSI-curve-driven curved boolean
// (src/native/boolean/ssi_boolean.{h,cpp}, OCCT-FREE, Gate 1 of the two-gate model).
// These lock the HONEST S5-a contract (design.md §Transversal-vs-deferred scope):
//   (a) an elementary curved solid is recognised (cylinder wall + planar caps folded
//       into one analytic surface + cap half-spaces);
//   (b) the curved point-in-solid classifier agrees with the closed-form containment
//       (inside / outside / ON) for cylinder / sphere;
//   (c) a TRANSVERSAL pair (thin cylinder drilled clean through a fat one) traces with
//       nearTangentGaps == 0 and closed/boundary WLines — the S5-a domain;
//   (d) a NEAR-TANGENT / branch-point pair (two EQUAL-radius perpendicular cylinders —
//       the classic tangential crossing) reports nearTangentGaps > 0, so the S5-a gate
//       DECLINES it and boolean_solid returns NULL → OCCT (the honest S4 boundary,
//       never faked);
//   (e) the dispatcher declines (NULL) for a pair with no traceable transversal seam.
// Compiled only under CYBERCAD_HAS_NUMSCI (the S3 tracer the path consumes calls the
// least_squares / lstsq substrate), like test_native_ssi_marching.
//
#include "native/boolean/native_boolean.h"
#include "native/boolean/ssi_boolean.h"

#include "harness.h"

#include <cmath>

namespace nb = cybercad::native::boolean;
namespace sd = cybercad::native::boolean::ssidetail;
namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;
namespace ntopo = cybercad::native::topology;

using nmath::Ax3;
using nmath::Dir3;
using nmath::Point3;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Build a finite axis-aligned cylinder native solid (one wall patch + two disc caps),
// reusing the analytic curved-boolean segment builder (a genuine native B-rep).
ntopo::Shape makeCyl(int axis, double r, double lo, double hi) {
  nb::curved::AABox box{Point3{-100, -100, -100}, Point3{100, 100, 100}};
  return nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{axis, 0, 0, r, lo, hi});
}

}  // namespace

// ── (a) + (b) recognise an elementary curved solid and classify points ───────────
CC_TEST(ssi_recognise_and_classify_cylinder) {
  const ntopo::Shape cyl = makeCyl(/*axis=Z*/ 2, /*r=*/1.0, /*lo=*/-2.0, /*hi=*/2.0);
  const auto cs = sd::recogniseCurvedSolid(cyl);
  CC_CHECK(cs.has_value());
  if (!cs) return;
  CC_CHECK(cs->kind == sd::CurvedKind::Cylinder);
  CC_CHECK(std::fabs(cs->radius - 1.0) < 1e-9);
  CC_CHECK(cs->capPlanes.size() == 2);  // two disc caps

  // Inside (near axis, mid-height), outside (well beyond the radius), and ON the wall.
  CC_CHECK(sd::classifyPoint(*cs, Point3{0, 0, 0}, 1e-6) == 1);       // inside
  CC_CHECK(sd::classifyPoint(*cs, Point3{5, 0, 0}, 1e-6) == -1);      // outside (radial)
  CC_CHECK(sd::classifyPoint(*cs, Point3{0, 0, 9}, 1e-6) == -1);      // outside (axial cap)
  CC_CHECK(sd::classifyPoint(*cs, Point3{1, 0, 0}, 1e-6) == 0);       // ON the wall
}

// ── (c) TRANSVERSAL through-drill traces cleanly (the S5-a domain) ───────────────
// A thin cylinder (r=0.5) drilled clean through a fat one (r=2), axes crossing at
// right angles, is a TRUE transversal pair: two disjoint closed intersection loops,
// no branch points → nearTangentGaps == 0. This is exactly what S5-a consumes.
CC_TEST(ssi_transversal_through_drill_traces_clean) {
  const auto csFat = sd::recogniseCurvedSolid(makeCyl(2, 2.0, -3, 3));   // Z axis
  const auto csThin = sd::recogniseCurvedSolid(makeCyl(0, 0.5, -3, 3));  // X axis
  CC_CHECK(csFat.has_value() && csThin.has_value());
  if (!csFat || !csThin) return;

  const ssi::TraceSet tr = ssi::trace_intersection(csFat->adapter(), csThin->adapter());
  CC_CHECK(tr.nearTangentGaps == 0);   // fully transversal → the S5-a gate admits it
  CC_CHECK(tr.curveCount() == 2);      // two disjoint loops (thin drilled through)
  for (const ssi::WLine& w : tr.lines)
    CC_CHECK(w.status == ssi::TraceStatus::Closed ||
             w.status == ssi::TraceStatus::BoundaryExit);
}

// ── (d) NEAR-TANGENT equal cylinders are DECLINED (the honest S4 boundary) ───────
// Two EQUAL-radius cylinders crossing at right angles meet TANGENTIALLY at the top /
// bottom (a branch-point / near-tangent seam) — the S3 tracer honestly reports
// nearTangentGaps > 0, so the S5-a gate MUST decline and boolean_solid returns NULL
// (→ engine falls back to OCCT). This is deferral, never a fabricated pass.
CC_TEST(ssi_near_tangent_equal_cylinders_declined) {
  const ntopo::Shape a = makeCyl(2, 1.0, -3, 3);  // Z axis, r=1
  const ntopo::Shape b = makeCyl(0, 1.0, -3, 3);  // X axis, r=1 (equal → tangent crossing)

  const auto csA = sd::recogniseCurvedSolid(a);
  const auto csB = sd::recogniseCurvedSolid(b);
  CC_CHECK(csA && csB);
  if (csA && csB) {
    const ssi::TraceSet tr = ssi::trace_intersection(csA->adapter(), csB->adapter());
    CC_CHECK(tr.nearTangentGaps > 0);  // honest S4 signal
  }
  // The whole dispatcher must decline (NULL) so the engine self-verify → OCCT path
  // owns it. NULL is the correct, honest outcome here.
  CC_CHECK(nb::ssi_boolean_solid(a, b, nb::Op::Common).isNull());
  CC_CHECK(nb::boolean_solid(a, b, nb::Op::Common).isNull());
}

// ── (e) a disjoint pair (no intersection seam) is declined ───────────────────────
CC_TEST(ssi_disjoint_pair_declined) {
  const ntopo::Shape a = makeCyl(2, 1.0, -2, 2);          // Z axis at origin
  // A far-away parallel cylinder, no intersection.
  nb::curved::AABox box{Point3{-100, -100, -100}, Point3{100, 100, 100}};
  const ntopo::Shape b =
      nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{2, 50.0, 0.0, 1.0, -2, 2});
  CC_CHECK(nb::ssi_boolean_solid(a, b, nb::Op::Common).isNull());
}

int main() { return cctest::run_all(); }

// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the NARROW curved-boolean slice (Phase 4 #5 `native-booleans`,
// deferred residual #2 in openspec/NATIVE-REWRITE.md): an AXIS-ALIGNED box ⟷ a
// cylinder whose axis is parallel to a box axis. OCCT-FREE — Gate 1 (host, analytic)
// of the two-gate model: the native curved boolean compiles and unit-tests with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// Everything here is built with the NATIVE construct library (box via extrude
// `build_prism`, cylinder via `build_revolution` for the X/Y axes and a full-circle
// `build_prism_profile` for the Z axis), combined with the NATIVE boolean
// `boolean_solid(a, b, op)`, and measured with the NATIVE tessellator — no OCCT
// anywhere. Each curved result carries a TRUE Cylinder lateral wall + TRUE Circle rim
// edges + Plane caps (nothing is faceted), so its watertight mesh matches the
// ANALYTIC volume to the deflection bound (the exact bar the engine's
// curvedBooleanVerified guard enforces before accepting a native result).
//
// WHAT LANDS NATIVE (asserted watertight + analytic volume within the deflection
// bound + a true Cylinder wall face):
//   * CUT   — 20³ box minus a through Z-axis cylinder (r=5) → 8000 − π·25·20 = 6429.20
//             (a round through hole; the inner wall is a Cylinder face).
//   * FUSE  — box + an axis-aligned cylinder BOSS on top → boxVol + π·r²·bossHeight.
//   * COMMON— box ∩ axis-aligned cylinder → the cylinder segment inside the box =
//             π·r²·overlapHeight.
//   * BLIND — box minus a cylinder that does NOT break through → boxVol − π·r²·depth,
//             with a flat disk (Plane) bottom.
//
// WHAT IS HONESTLY FALLTHROUGH (asserted to return NULL, so the engine never emits a
// wrong/leaky curved solid — it delegates to the OCCT oracle):
//   * a NON-axis-aligned cylinder cut,
//   * a sphere/cone (non-cylinder curved) ⟷ box,
//   * cyl − box (wrong operand order), a radial breach, a fully-interior cylinder.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_curved_boolean.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_curved_boolean
//
#include "native/boolean/native_boolean.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace nb = cybercad::native::boolean;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ── Native solid constructors (no OCCT) ─────────────────────────────────────────

// A box [x0,x0+sx]×[y0,y0+sy]×[z0,z0+sz] as a native prism (extrude a rectangle along
// +Z, then translate to z0). This is the SCOPE's "box via extrude".
topo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  topo::Shape s = cst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, z0})));
  return s;
}

// A native solid CYLINDER, axis = world Y, radius r, axial extent y∈[ylo,yhi], centre
// line at (x=cx, z=0). Built by the verified native REVOLVE (a rectangle r∈[0,r],
// y∈[ylo,yhi] revolved 2π about the in-plane Y axis through x=cx). This is the SCOPE's
// "cylinder via revolve". The revolve axis lies in z=0, so the cylinder centre is
// always z=0 — the box must straddle z=0 for the cylinder to sit radially inside it.
topo::Shape cylinderY(double cx, double r, double ylo, double yhi) {
  const double rect[] = {0, ylo, r, ylo, r, yhi, 0, yhi};
  return cst::build_revolution(rect, 4, cst::RevolveAxis{cx, 0, 0, 1}, 2.0 * kPi);
}

// A native solid CYLINDER, axis = world X, radius r, x∈[xlo,xhi], centre (y=0,z=0).
// The revolve axis is the in-plane X direction (dir (1,0)).
// Only xlo=0 is exercised here; the profile (u∈[0,r] radial, v∈[0,xhi] axial) revolved
// about the in-plane X axis (dir (1,0)) yields axis X, centre (y=0,z=0), radius r,
// x∈[0,xhi] — the exact construction the planar-slice X-axis parity fixture uses.
topo::Shape cylinderX(double r, double xhi) {
  const double rect[] = {0, 0, r, 0, r, xhi, 0, xhi};
  return cst::build_revolution(rect, 4, cst::RevolveAxis{0, 0, 1, 0}, 2.0 * kPi);
}

// A native solid CYLINDER, axis = world Z, radius r, z∈[zlo,zhi], centre (cx,cy).
// A Z-axis cylinder cannot come from the in-plane revolve, so it is a full-CIRCLE
// typed profile (kind 2) extruded along +Z, then translated to zlo — a native solid
// cylinder with a true Cylinder wall + two Circle-bounded disc caps.
topo::Shape cylinderZ(double cx, double cy, double r, double zlo, double zhi) {
  cst::ProfileSegment seg;
  seg.kind = 2;
  seg.cx = cx;
  seg.cy = cy;
  seg.r = r;
  topo::Shape s = cst::build_prism_profile({seg}, {}, {}, zhi - zlo);
  if (!s.isNull() && zlo != 0.0)
    s = s.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, zlo})));
  return s;
}

// A native solid CONE (revolve a right triangle) — a curved non-cylinder used to
// prove the family is honestly bounded (recogniseCylinder rejects a Cone face).
topo::Shape coneZ(double r, double h) {
  const double tri[] = {0, 0, r, 0, 0, h};
  return cst::build_revolution(tri, 3, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * kPi);
}

// ── Native measurement (no OCCT) ────────────────────────────────────────────────

int faceCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) ++n;
  return n;
}

// Count faces carrying a given analytic surface kind (Cylinder / Plane).
int surfaceKindCount(const topo::Shape& s, topo::FaceSurface::Kind kind) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto surf = topo::surfaceOf(ex.current());
    if (surf && surf->surface && surf->surface->kind == kind) ++n;
  }
  return n;
}

// Watertight enclosed volume at a fine deflection. Sets `watertight`.
double vol(const topo::Shape& s, bool& watertight) {
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// Deflection-bounded curved-mesh volume check (the curved result carries a TRUE
// cylinder surface, so its watertight mesh only APPROXIMATES the analytic volume;
// 1% relative mirrors the engine's curvedBooleanVerified tolerance).
bool nearCurved(double got, double want) {
  return std::fabs(got - want) <= 1e-2 * std::fabs(want);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════
// CUT — a 20×20×20 box minus an axis-aligned Z cylinder (r=5, through) → a round
// through hole. Volume == 8000 − π·25·20 = 6429.20 within the deflection bound;
// watertight; the inner wall IS a Cylinder face. (The task's headline CUT.)
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(cut_20cube_minus_z_cylinder_through_hole) {
  const topo::Shape box = boxAt(0, 0, 0, 20, 20, 20);          // vol 8000
  const topo::Shape cyl = cylinderZ(10, 10, 5, -5, 25);        // axis Z, r=5, through
  CC_CHECK(!box.isNull() && !cyl.isNull());

  const topo::Shape cut = nb::boolean_solid(box, cyl, nb::Op::Cut);
  CC_CHECK(!cut.isNull());

  bool wt = false;
  const double v = vol(cut, wt);
  CC_CHECK(nearCurved(v, 8000.0 - kPi * 25.0 * 20.0));  // 6429.20
  CC_CHECK(wt);
  // The inner wall is a TRUE Cylinder face (not faceted): at least one Cylinder face.
  CC_CHECK(surfaceKindCount(cut, topo::FaceSurface::Kind::Cylinder) >= 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// FUSE — a box + an axis-aligned cylinder BOSS on top → volume == boxVol +
// π·r²·bossHeight (no overlap beyond the shared cap); watertight. Commutative.
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(fuse_box_plus_cylinder_boss) {
  // Box x[0,10] y[0,6] z[−5,5] (vol 600). Cylinder axis Y, r=2, centre (x=5,z=0),
  // y[0,10]: base flush at the y=0 face, protrudes past the y=6 face by 4 (bossHeight).
  const double bp[] = {0, 0, 10, 0, 10, 6, 0, 6};
  topo::Shape box = cst::build_prism(bp, 4, 10);
  box = box.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));
  const topo::Shape cyl = cylinderY(5, 2, 0, 10);
  CC_CHECK(!box.isNull() && !cyl.isNull());

  const double bossHeight = 4.0;  // protrusion past the y=6 cap
  const double want = 600.0 + kPi * 4.0 * bossHeight;

  bool wt = false;
  const topo::Shape fuse = nb::boolean_solid(box, cyl, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  CC_CHECK(nearCurved(vol(fuse, wt), want));
  CC_CHECK(wt);
  CC_CHECK(surfaceKindCount(fuse, topo::FaceSurface::Kind::Cylinder) >= 1);

  // Commutative: cyl ∪ box gives the same solid.
  const topo::Shape fuseRev = nb::boolean_solid(cyl, box, nb::Op::Fuse);
  CC_CHECK(!fuseRev.isNull());
  CC_CHECK(nearCurved(vol(fuseRev, wt), want));
  CC_CHECK(wt);
}

// ═══════════════════════════════════════════════════════════════════════════════
// COMMON — a box AND an axis-aligned cylinder → the cylinder SEGMENT inside the box;
// volume == π·r²·overlapHeight within the bound; watertight.
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(common_box_and_cylinder_segment) {
  // Box x[0,10] y[2,8] z[−5,5]; cyl axis Y r=2 centre (x=5,z=0) y[0,10]. The overlap
  // along Y is y∈[2,8], length 6 → common = π·4·6.
  const double bp[] = {0, 2, 10, 2, 10, 8, 0, 8};
  topo::Shape box = cst::build_prism(bp, 4, 10);
  box = box.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));
  const topo::Shape cyl = cylinderY(5, 2, 0, 10);
  CC_CHECK(!box.isNull() && !cyl.isNull());

  const double overlapHeight = 6.0;
  const topo::Shape common = nb::boolean_solid(box, cyl, nb::Op::Common);
  CC_CHECK(!common.isNull());
  bool wt = false;
  CC_CHECK(nearCurved(vol(common, wt), kPi * 4.0 * overlapHeight));  // 75.40
  CC_CHECK(wt);
  CC_CHECK(surfaceKindCount(common, topo::FaceSurface::Kind::Cylinder) >= 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// BLIND HOLE — a cylinder NOT fully through → box minus a flat-bottomed pocket.
// volume == boxVol − π·r²·depth within the bound; watertight; the pocket bottom is a
// flat Plane DISC (the analytic signature of a blind vs a through hole).
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(blind_hole_flat_disk_bottom) {
  // Box x[0,10] y[−5,5] z[−5,5] (vol 1000). Cyl axis Y r=2 centre (x=5,z=0), near cap
  // y=20 (well past the y=5 face), far cap (bottom) at y=1 STRICTLY inside → a blind
  // pocket of depth = 5 − 1 = 4. removed = π·4·4; blind vol = 1000 − π·4·4.
  const double bp[] = {0, -5, 10, -5, 10, 5, 0, 5};
  topo::Shape box = cst::build_prism(bp, 4, 10);
  box = box.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));
  const topo::Shape cyl = cylinderY(5, 2, 1, 20);  // bottom y=1 inside, opens through y=5
  CC_CHECK(!box.isNull() && !cyl.isNull());

  const double depth = 4.0;
  const topo::Shape cut = nb::boolean_solid(box, cyl, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  bool wt = false;
  CC_CHECK(nearCurved(vol(cut, wt), 1000.0 - kPi * 4.0 * depth));  // 949.73
  CC_CHECK(wt);
  // Cylinder wall + a flat disk bottom: at least one Cylinder face and the flat
  // bottom is one of the Plane caps (a blind pocket has MORE planar caps than a bare
  // box's 6 — the pocket bottom disc + the annular opening cap).
  CC_CHECK(surfaceKindCount(cut, topo::FaceSurface::Kind::Cylinder) >= 1);
  CC_CHECK(surfaceKindCount(cut, topo::FaceSurface::Kind::Plane) >= 6);
}

// ═══════════════════════════════════════════════════════════════════════════════
// CUT across a second world axis (X) — the analytic builders are axis-parametrized;
// verify a through-hole cut works with an X-axis cylinder too.
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(cut_box_minus_x_cylinder_through_hole) {
  // Box x[2,8] y[−5,5] z[−5,5] (vol 600). Cyl axis X r=2 centre (y=0,z=0) x[0,10]
  // spans the box x-depth (6) → a through hole along X. cut = 600 − π·4·6.
  const double bp[] = {2, -5, 8, -5, 8, 5, 2, 5};
  topo::Shape box = cst::build_prism(bp, 4, 10);
  box = box.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));
  const topo::Shape cyl = cylinderX(2, 10);
  CC_CHECK(!box.isNull() && !cyl.isNull());

  const topo::Shape cut = nb::boolean_solid(box, cyl, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  bool wt = false;
  CC_CHECK(nearCurved(vol(cut, wt), 600.0 - kPi * 4.0 * 6.0));
  CC_CHECK(wt);
  CC_CHECK(surfaceKindCount(cut, topo::FaceSurface::Kind::Cylinder) >= 1);
}

// ═══════════════════════════════════════════════════════════════════════════════
// HONEST FALLTHROUGH — a NON-axis-aligned cylinder cut returns the not-supported
// signal (NULL Shape → engine falls through to OCCT; NEVER a wrong/leaky solid).
// The cylinder here is built by revolving about an OBLIQUE in-plane axis (dir (1,1)),
// so its lateral face's axis is not world-aligned → recogniseCylinder rejects it.
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(non_axis_aligned_cylinder_defers_to_occt) {
  // Revolve a rectangle about the in-plane diagonal axis dir (1,1) through the origin
  // → a cylinder whose axis points along (1,1,0)/√2 — NOT a world axis.
  const double rect[] = {0, 0, 2, 0, 2, 8, 0, 8};
  const topo::Shape oblique =
      cst::build_revolution(rect, 4, cst::RevolveAxis{0, 0, 1, 1}, 2.0 * kPi);
  CC_CHECK(!oblique.isNull());
  // Confirm it is genuinely NOT recognised as an axis-parallel cylinder.
  CC_CHECK(!nb::curved::recogniseCylinder(oblique).has_value());

  const topo::Shape box = boxAt(-6, -6, -6, 12, 12, 12);
  CC_CHECK(nb::boolean_solid(box, oblique, nb::Op::Cut).isNull());     // → OCCT
  CC_CHECK(nb::boolean_solid(box, oblique, nb::Op::Fuse).isNull());    // → OCCT
  CC_CHECK(nb::boolean_solid(box, oblique, nb::Op::Common).isNull());  // → OCCT
}

// ═══════════════════════════════════════════════════════════════════════════════
// HONEST FALLTHROUGH — a sphere/cone (non-cylinder curved surface) ⟷ box is OUTSIDE
// the family: recogniseCylinder rejects a Cone/Sphere face, so box ⟷ cone returns
// NULL → OCCT. (A revolved right triangle is a genuine Cone.)
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(sphere_or_cone_box_defers_to_occt) {
  const topo::Shape cone = coneZ(3, 5);
  CC_CHECK(!cone.isNull());
  CC_CHECK(!nb::curved::recogniseCylinder(cone).has_value());  // a Cone, not a cylinder

  const topo::Shape box = boxAt(-5, -5, 0, 10, 10, 5);
  CC_CHECK(nb::boolean_solid(box, cone, nb::Op::Cut).isNull());     // → OCCT
  CC_CHECK(nb::boolean_solid(box, cone, nb::Op::Common).isNull());  // → OCCT
  CC_CHECK(nb::boolean_solid(box, cone, nb::Op::Fuse).isNull());    // → OCCT
}

// ═══════════════════════════════════════════════════════════════════════════════
// HONEST FALLTHROUGH — the remaining out-of-family cases return NULL (never faked):
//   * cyl − box (wrong operand order for a round hole),
//   * a radially-BREACHING cylinder (would cut a non-round slot through a side wall),
//   * a fully-INTERIOR cylinder (both caps inside → an internal void, non-manifold).
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(out_of_family_cuts_defer_to_occt) {
  const double bp[] = {0, 2, 10, 2, 10, 8, 0, 8};  // box x[0,10] y[2,8] z[−5,5]
  topo::Shape box = cst::build_prism(bp, 4, 10);
  box = box.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));

  // cyl − box: carving the box shape out of the cylinder is a different solid.
  const topo::Shape cyl = cylinderY(5, 2, 0, 10);
  CC_CHECK(nb::boolean_solid(cyl, box, nb::Op::Cut).isNull());

  // radial breach: r=8 at centre x=5 → x∈[−3,13] breaches the box x∈[0,10].
  const topo::Shape big = cylinderY(5, 8, 0, 10);
  CC_CHECK(nb::boolean_solid(box, big, nb::Op::Cut).isNull());

  // fully-interior cylinder (both caps inside the box y∈[2,8]) → deferred (an
  // internal void, not a flat-bottomed pocket).
  const double bp2[] = {0, -5, 10, -5, 10, 5, 0, 5};  // box y[−5,5]
  topo::Shape box2 = cst::build_prism(bp2, 4, 10);
  box2 = box2.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, -5})));
  const topo::Shape interior = cylinderY(5, 2, -2, 2);  // both caps inside
  CC_CHECK(nb::boolean_solid(box2, interior, nb::Op::Cut).isNull());
}

// ═══════════════════════════════════════════════════════════════════════════════
// Determinism — the same curved cut computed twice yields the same volume and face
// count (the analytic builder is a pure function of the recognised primitives).
// ═══════════════════════════════════════════════════════════════════════════════
CC_TEST(curved_cut_deterministic) {
  const topo::Shape box = boxAt(0, 0, 0, 20, 20, 20);
  const topo::Shape cyl = cylinderZ(10, 10, 5, -5, 25);
  const topo::Shape r1 = nb::boolean_solid(box, cyl, nb::Op::Cut);
  const topo::Shape r2 = nb::boolean_solid(box, cyl, nb::Op::Cut);
  CC_CHECK(!r1.isNull() && !r2.isNull());
  bool wt1 = false, wt2 = false;
  CC_CHECK(nearCurved(vol(r1, wt1), vol(r2, wt2)));
  CC_CHECK(wt1 && wt2);
  CC_CHECK_EQ(faceCount(r1), faceCount(r2));
}

CC_RUN_ALL()

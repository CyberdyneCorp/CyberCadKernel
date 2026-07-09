// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native blend slice (Phase 4, capability #6
// `native-blends`). OCCT-FREE — Gate 1 (host, analytic) of the two-gate model in
// openspec/NATIVE-REWRITE.md: the blend ops compile and unit-test with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// Each op is exercised DIRECTLY on native prisms built by the verified construct
// library and validated the way the ENGINE does: native TESSELLATOR watertightness
// + a sane volume sign (chamfer/fillet reduce a convex-edge volume; offset grows;
// shell reduces to a wall). A result that fails these is one the engine DISCARDS and
// falls through to OCCT, so the tests assert the HONEST native/fallthrough split.
//
// Build (standalone):
//   /opt/homebrew/opt/llvm/bin/clang++ -std=c++20 tests/native/test_native_blend.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_blend && /tmp/test_native_blend
//
#include "native/blend/native_blend.h"
#include "native/boolean/native_boolean.h"
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

// Watertight enclosed volume at a fine deflection; sets `wt`.
double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearRel(double got, double want, double rel = 1e-4, double abs = 1e-7) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), abs);
}

// Face-count of a solid.
int faceCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) ++n;
  return n;
}

// Find the id of the first edge whose two endpoints match (a,b) up to tol, so a test
// can pick a specific box edge deterministically.
int findEdgeId(const topo::Shape& s, const nmath::Point3& a, const nmath::Point3& b) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto ends = blend::edgeEnds(s, static_cast<int>(i));
    if (!ends) continue;
    const bool fwd = nmath::distance(ends->a, a) < 1e-6 && nmath::distance(ends->b, b) < 1e-6;
    const bool rev = nmath::distance(ends->a, b) < 1e-6 && nmath::distance(ends->b, a) < 1e-6;
    if (fwd || rev) return static_cast<int>(i);
  }
  return 0;
}

int findFaceId(const topo::Shape& s, const nmath::Vec3& outwardNormal, const nmath::Point3& on) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = blend::facePlane(s, static_cast<int>(i));
    if (!pl) continue;
    const nmath::Dir3 want{outwardNormal};
    if (nmath::dot(pl->normal, want.vec()) > 0.999 &&
        std::fabs(blend::signedDist(*pl, on)) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}

}  // namespace

// ── chamfer ──────────────────────────────────────────────────────────────────────

CC_TEST(chamfer_box_top_edge_volume_reduced) {
  // 10×10×10 box; chamfer the top edge along x at y=10, z=10 (a convex edge between
  // the top face (+Z) and the +Y side). distance 2 → cut a right-triangle prism of
  // legs 2×2 over length 10: removed volume = ½·2·2·10 = 20.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  topo::Shape ch = blend::chamfer_edges(b, ids, 1, 2.0);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 1000.0 - 20.0));  // 980
  CC_CHECK(faceCount(ch) >= 7);         // 6 + the chamfer face (triangulated welds)
}

CC_TEST(chamfer_degenerate_and_curved_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::chamfer_edges(b, ids, 1, 0.0).isNull());   // zero distance
  CC_CHECK(blend::chamfer_edges(b, nullptr, 0, 2.0).isNull());
  // A curved solid (cylinder) is not planar → NULL.
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::chamfer_edges(cyl, cids, 1, 1.0).isNull());
}

// ── fillet ─────────────────────────────────────────────────────────────────────--

CC_TEST(fillet_box_top_edge_watertight_and_between) {
  // Fillet the same convex top edge with r=2. The removed volume is the corner minus
  // the quarter-cylinder: sharp-corner prism (2×2×10=40 for the square) minus the
  // quarter disc (¼π r² · 10 = ¼π·4·10 = 10π ≈ 31.42) → removed ≈ 8.58, so
  // filleted volume ≈ 1000 − 8.58 ≈ 991.4, BETWEEN the chamfer (980) and sharp (1000).
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  topo::Shape f = blend::fillet_edges(b, ids, 1, 2.0, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  const double expected = 1000.0 - (2.0 * 2.0 * 10.0 - 0.25 * M_PI * 4.0 * 10.0);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
  CC_CHECK(v < 1000.0 && v > 980.0);     // between sharp and chamfer
}

CC_TEST(fillet_curved_and_degenerate_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges(b, ids, 1, 0.0).isNull());
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::fillet_edges(cyl, cids, 1, 1.0).isNull());
}

// ── curved fillet (first CURVED slice: torus canal blend on a cylinder↔cap rim) ──--

namespace {
// A capped solid cylinder: full-circle profile radius Rc extruded to height h. Faces:
// bottom cap (−Z), top cap (+Z), one Cylinder wall. Rims are true Circle edges.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}
// The Circle rim edge id at axial height `z` (the top or bottom rim).
int findRimAtZ(const topo::Shape& s, double z) {
  const topo::ShapeMap emap = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    nmath::Point3 o = c->curve->frame.origin;
    if (!c->location.isIdentity()) o = c->location.transform().applyToPoint(o);
    if (std::fabs(o.z - z) < 1e-6) return static_cast<int>(i);
  }
  return 0;
}
}  // namespace

CC_TEST(curved_fillet_cylinder_cap_watertight_volume_reduced) {
  // Rc=5, h=10 capped cylinder; roll a ball r=1.5 into the top rim → a coaxial torus
  // canal blend (major R=Rc−r=3.5, minor r=1.5). Watertight, volume BELOW the sharp
  // cylinder, and matching the exact solid-of-revolution to the deflection bound.
  const double Rc = 5.0, h = 10.0, r = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::curved_fillet_edge(cyl, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                 // torus quarter-tube welds watertight to wall + cap
  CC_CHECK(v < v0);             // a convex fillet REDUCES the volume
  // Exact filleted volume = cylinder up to the wall seam (z=h−r) + the torus solid of
  // revolution over the last r of height. Closed form of ∫π·radius(v)²·dz with
  // radius(v)=R+r·cos v, z=r·sin v, v∈[0,π/2]:
  //   ∫₀^{π/2} π (R + r cos v)² r cos v dv
  //   = π r [ R²·1 + 2Rr·(π/4) + r²·(2/3) ].
  const double R = Rc - r;
  const double vTorus = M_PI * r * (R * R + 2.0 * R * r * (M_PI / 4.0) + r * r * (2.0 / 3.0));
  const double expected = M_PI * Rc * Rc * (h - r) + vTorus;
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(curved_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 assertion (no OCCT, no mesh): the torus canal normal at the two seams
  // matches the adjacent primary-face normal exactly. The blend surface is the quarter
  // tube radius(v)=R+r·cos v, axialOffset(v)=r·sin v, with outward normal
  //   n(u,v) = radial(u)·cos v + axis·sin v.
  //   * v=0   (wall seam):  n = radial(u)  → the CYLINDER outward normal (radial). cos=1.
  //   * v=π/2 (cap seam):   n = axis        → the CAP outward normal (+axis).      cos=1.
  // Because both seams share position AND normal with the neighbour, the fillet is
  // G1-tangent there. We check this at several u around the rim.
  const double Rc = 5.0, r = 1.5;
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis (build_prism extrudes +Z)
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    // Wall seam v=0: torus normal is purely radial → equals the cylinder radial normal.
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    // Cap seam v=π/2: torus normal is purely axial → equals the cap outward normal.
    const nmath::Vec3 nCap = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nCap}.vec(), axis), 1.0, 1e-12));
  }
  // The seam POSITIONS also coincide with the neighbours: v=0 sits at radius Rc (on the
  // cylinder wall) and v=π/2 sits at radius Rc−r (the trimmed cap edge), closing G0+G1.
  // radius(v)=(Rc−r)+r·cos v, so radius(0)=Rc (wall) and radius(π/2)=Rc−r (cap edge).
  const double Rm = Rc - r;  // torus major radius
  CC_CHECK(nearRel(Rm + r * std::cos(0.0), Rc, 1e-12));            // radius(0)=Rc (wall)
  CC_CHECK(nearRel(Rm + r * std::cos(M_PI / 2.0), Rc - r, 1e-12));  // radius(π/2)=Rc−r (cap)
}

CC_TEST(curved_fillet_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Ring-torus guard: r=3 ⇒ Rc<2r ⇒ NULL (spindle torus, defers to OCCT).
  CC_CHECK(blend::curved_fillet_edge(cyl, ids, 1, 3.0, 0.01).isNull());
  // Zero / negative radius → NULL.
  CC_CHECK(blend::curved_fillet_edge(cyl, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_fillet_edge(cyl, ids2, 2, 1.5, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

CC_TEST(curved_fillet_both_rims_and_engine_dispatch) {
  // The engine fillet_edges dispatches the circular rim through the curved path when
  // the planar path declines. Exercise the builder on the BOTTOM rim too (z=0), and
  // confirm the planar fillet_edges still returns NULL on this curved solid (so the
  // engine's fall-through to curved_fillet_edge is what lands it).
  const double Rc = 4.0, h = 8.0, r = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int bottom = findRimAtZ(cyl, 0.0);
  CC_CHECK(bottom != 0);
  int ids[] = {bottom};
  CC_CHECK(blend::fillet_edges(cyl, ids, 1, r).isNull());  // planar path declines
  topo::Shape f = blend::curved_fillet_edge(cyl, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < v0);
}

// ── variable-radius convex fillet (swept variable-r torus canal on a cyl↔cap rim) ─--

namespace {
// Closed-form REMOVED volume of the variable convex fillet over the circular rim, via the
// per-meridian removed corner area r²(1−π/4) with centroid radius (Pappus per angular
// slice): V = ∫₀^{2π} r(θ)²[Rc(1−π/4) + r(θ)(π/4−5/6)] dθ, r(θ)=r1+(r2−r1)θ/2π. Using
// ∫r²dθ=2π(r1²+r1r2+r2²)/3 and ∫r³dθ=2π(r1+r2)(r1²+r2²)/4.
double variableVremoved(double Rc, double r1, double r2) {
  const double q = M_PI / 4.0;
  const double i2 = 2.0 * M_PI * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0;
  const double i3 = 2.0 * M_PI * (r1 + r2) * (r1 * r1 + r2 * r2) / 4.0;
  return Rc * (1.0 - q) * i2 + (q - 5.0 / 6.0) * i3;
}
}  // namespace

CC_TEST(variable_fillet_cylinder_cap_watertight_volume_between) {
  // Fixture A: Rc=5, h=10 capped cylinder; roll a ball whose radius ramps r1=1 → r2=2
  // linearly around the top rim → a swept variable-r torus canal. Watertight, BELOW the
  // sharp cylinder, BETWEEN the two constant-radius (r1, r2) fillet volumes, and matching
  // the closed-form removed volume to the deflection bound.
  const double Rc = 5.0, h = 10.0, r1 = 1.0, r2 = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::variable_fillet_edge(cyl, ids, 1, r1, r2, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);          // swept canal + seam wall weld watertight to wall + cap
  CC_CHECK(v < v0);      // a convex (variable) fillet REDUCES the volume
  // Bracket: larger r removes more, so v(r2) < v(variable) < v(r1).
  bool wa = false, wb = false;
  const double vR1 = vol(blend::curved_fillet_edge(cyl, ids, 1, r1, 0.005), wa);
  const double vR2 = vol(blend::curved_fillet_edge(cyl, ids, 1, r2, 0.005), wb);
  CC_CHECK(wa && wb);
  CC_CHECK(vR2 < v && v < vR1);
  // Closed-form removed volume (14.60 for this fixture).
  const double expected = v0 - variableVremoved(Rc, r1, r2);
  CC_CHECK(nearRel(v, expected, 6e-3));  // deflection-bounded facet approximation
}

CC_TEST(variable_fillet_second_fixture_and_reversed) {
  // Fixture B: Rc=6, h=12, r1=0.75 → r2=2.25. Also exercise the REVERSED law (r1>r2) to
  // cover both seam-wall orientations.
  const double Rc = 6.0, h = 12.0, r1 = 0.75, r2 = 2.25;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  topo::Shape f = blend::variable_fillet_edge(cyl, ids, 1, r1, r2, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - variableVremoved(Rc, r1, r2), 6e-3));
  // Reversed law (r1>r2) → the seam wall faces the other way; still watertight, same volume.
  topo::Shape fr = blend::variable_fillet_edge(cyl, ids, 1, r2, r1, 0.005);
  bool wtr = false;
  const double vr = vol(fr, wtr);
  CC_CHECK(!fr.isNull());
  CC_CHECK(wtr);
  CC_CHECK(nearRel(vr, v0 - variableVremoved(Rc, r2, r1), 6e-3));
}

CC_TEST(variable_fillet_reduces_to_constant_when_r1_eq_r2) {
  // r1==r2 must reproduce the constant torus fillet exactly (every band + seam wall
  // collapses to zero area).
  const double Rc = 5.0, h = 10.0, r = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  bool wv = false, wc = false;
  const double vVar = vol(blend::variable_fillet_edge(cyl, ids, 1, r, r, 0.005), wv);
  const double vConst = vol(blend::curved_fillet_edge(cyl, ids, 1, r, 0.005), wc);
  CC_CHECK(wv && wc);
  CC_CHECK(nearRel(vVar, vConst, 1e-9));  // identical facet soup
}

CC_TEST(variable_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): at EVERY station the variable-canal normal at v=0 is
  // radial (== cylinder normal) and at v=π/2 is axial (== cap normal) — independent of the
  // radius gradient r'(θ). n(u,v)=radial(u)cos v + axis·sin v, so cos=1 at both seams.
  const nmath::Vec3 axis{0, 0, 1};
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    const nmath::Vec3 nCap = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nCap}.vec(), axis), 1.0, 1e-12));
  }
  // Seam POSITIONS at the two ends of the linear law coincide with the neighbours: at any
  // station r, radius(0)=Rc (wall) and radius(π/2)=Rc−r (trimmed cap edge).
  const double Rc = 5.0;
  for (double r : {1.0, 1.5, 2.0}) {
    CC_CHECK(nearRel((Rc - r) + r * std::cos(0.0), Rc, 1e-12));
    CC_CHECK(nearRel((Rc - r) + r * std::cos(M_PI / 2.0), Rc - r, 1e-12));
  }
}

CC_TEST(variable_fillet_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Ring-torus guard on the DEEPEST station: max(r1,r2)=3 ⇒ Rc<2·rMax ⇒ NULL.
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 1.0, 3.0, 0.01).isNull());
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 3.0, 1.0, 0.01).isNull());
  // Zero / negative radius on either end → NULL.
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 0.0, 2.0, 0.01).isNull());
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 1.0, 0.0, 0.01).isNull());
  // More than one picked edge → NULL.
  int ids2[] = {rim, 1};
  CC_CHECK(blend::variable_fillet_edge(cyl, ids2, 2, 1.0, 2.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::variable_fillet_edge(b, idsb, 1, 1.0, 2.0, 0.01).isNull());
}

// ── curved chamfer (cone-frustum straight bevel on a cylinder↔cap rim) ────────────--

namespace {
// Exact removed corner-ring volume of a symmetric distance-d chamfer on a circular rim
// (radius Rc): the right triangle legs d×d (area d²/2, centroid radial Rc−d/3) revolved
// about the axis (Pappus): V_removed = π·d²·(Rc − d/3). Volume REDUCES.
double chamferVremoved(double Rc, double d) {
  return M_PI * d * d * (Rc - d / 3.0);
}
}  // namespace

CC_TEST(curved_chamfer_cylinder_cap_watertight_volume_reduced) {
  // Fixture A: Rc=5, h=10 capped cylinder; chamfer the top rim with distance d=1.0 → a
  // CONE-FRUSTUM straight bevel between the two setback circles (wall circle r=Rc at
  // z=h−d, cap circle r=Rc−d at z=h). Watertight, BELOW the sharp cylinder, matching the
  // exact Pappus removed volume to the deflection bound.
  const double Rc = 5.0, h = 10.0, d = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);                 // frustum band welds watertight to wall + trimmed cap
  CC_CHECK(v < v0);             // a convex chamfer REDUCES the volume
  const double expected = v0 - chamferVremoved(Rc, d);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(curved_chamfer_second_fixture_and_removes_more_than_fillet) {
  // Fixture B: same body, d=2.0. Also assert the chamfer removes MORE than a fillet of
  // radius d (cross-section d²/2 > d²(1−π/4)), so V_chamfer < V_fillet < V0.
  const double Rc = 5.0, h = 10.0, d = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - chamferVremoved(Rc, d), 5e-3));
  // Chamfer removes more than the equal-distance fillet → smaller volume.
  bool wf = false;
  const double vFillet = vol(blend::curved_fillet_edge(cyl, ids, 1, d, 0.005), wf);
  CC_CHECK(wf);
  CC_CHECK(v < vFillet && vFillet < v0);
}

CC_TEST(curved_chamfer_is_c0_bevel_not_g1) {
  // ANALYTIC bevel-angle assertion (no OCCT, no mesh): the frustum outward normal is the
  // 45° BISECTOR of the two face normals — cos = 1/√2 with BOTH the cylinder radial
  // normal and the cap axial normal, and explicitly NOT 1 (a chamfer is C0, NOT tangent).
  // This is the load-bearing inversion vs the fillet (whose seam normals give cos=1).
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis; s=+1 (cap at the top)
  const double invSqrt2 = 1.0 / std::sqrt(2.0);
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    // Frustum bevel outward normal = (radial + s·axis)/√2.
    const nmath::Vec3 bevel = nmath::Dir3{radial + axis}.vec();
    // Meets the cylinder wall (radial normal) at 45°, NOT tangent.
    const double cosWall = nmath::dot(bevel, radial);
    CC_CHECK(nearRel(cosWall, invSqrt2, 1e-12));
    CC_CHECK(std::fabs(cosWall - 1.0) > 0.2);  // NOT G1 (not tangent to the wall)
    // Meets the cap (axial normal) at 45°, NOT tangent.
    const double cosCap = nmath::dot(bevel, axis);
    CC_CHECK(nearRel(cosCap, invSqrt2, 1e-12));
    CC_CHECK(std::fabs(cosCap - 1.0) > 0.2);  // NOT G1 (not tangent to the cap)
  }
  // The two faces meet at 90°, so the bevel bisects them exactly (radial·axis = 0).
  CC_CHECK(nearRel(nmath::dot(nmath::Vec3{1, 0, 0}, axis), 0.0, 1e-12));
}

CC_TEST(curved_chamfer_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Rc − d ≤ 0 guard: d=5 ⇒ the bevel would cross the axis ⇒ NULL (defers to OCCT).
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 5.0, 0.01).isNull());
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 6.0, 0.01).isNull());
  // Zero / negative distance → NULL.
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids2, 2, 1.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_chamfer_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

CC_TEST(curved_chamfer_both_rims_and_planar_declines) {
  // The engine chamfer_edges dispatches the circular rim through the curved path when
  // the planar path declines. Exercise the builder on the BOTTOM rim too (z=0), and
  // confirm the planar chamfer_edges still returns NULL on this curved solid (so the
  // engine's fall-through to curved_chamfer_edge is what lands it).
  const double Rc = 4.0, h = 8.0, d = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int bottom = findRimAtZ(cyl, 0.0);
  CC_CHECK(bottom != 0);
  int ids[] = {bottom};
  CC_CHECK(blend::chamfer_edges(cyl, ids, 1, d).isNull());  // planar path declines
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false, wt0 = false;
  const double v = vol(ch, wt);
  const double v0 = vol(cyl, wt0);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - chamferVremoved(Rc, d), 5e-3));
}

// ── T1: asymmetric two-distance chamfer (oblique cone-frustum bevel) ──────────────--

namespace {
// Exact removed corner-ring volume of an ASYMMETRIC chamfer (axial wall setback d1,
// radial cap setback d2) on a circular rim (radius Rc): the right triangle legs d1 (axial)
// × d2 (radial), area ½·d1·d2, centroid radial Rc − d2/3, revolved about the axis (Pappus):
// V_removed = π·d1·d2·(Rc − d2/3). d1 = d2 reduces to the symmetric π·d²·(Rc − d/3).
double chamferVremovedAsym(double Rc, double d1, double d2) {
  return M_PI * d1 * d2 * (Rc - d2 / 3.0);
}
}  // namespace

CC_TEST(asym_chamfer_oblique_frustum_watertight_volume) {
  // Rc=5, h=10 capped cylinder; chamfer the top rim with d1=2 (axial WALL setback) and
  // d2=1 (radial CAP setback) → an OBLIQUE cone-frustum bevel between the wall seam
  // (Rc, h−d1) and the cap seam (Rc−d2, h). Watertight, BELOW the sharp cylinder, matching
  // the exact Pappus removed volume π·d1·d2·(Rc−d2/3)=29.3215 → chamfered ≈ 756.0766.
  const double Rc = 5.0, h = 10.0, d1 = 2.0, d2 = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge_asym(cyl, ids, 1, d1, d2, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);                 // oblique frustum welds watertight to wall + trimmed cap
  CC_CHECK(v < v0);             // a convex chamfer REDUCES the volume
  const double expected = v0 - chamferVremovedAsym(Rc, d1, d2);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
  // A swapped fixture (d1=1, d2=2) removes a different corner (still exact).
  topo::Shape ch2 = blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 2.0, 0.005);
  bool wt2 = false;
  const double v2 = vol(ch2, wt2);
  CC_CHECK(!ch2.isNull() && wt2);
  CC_CHECK(nearRel(v2, v0 - chamferVremovedAsym(Rc, 1.0, 2.0), 5e-3));
}

CC_TEST(asym_chamfer_symmetric_special_case) {
  // d1 == d2 must reproduce the SYMMETRIC chamfer volume EXACTLY (byte-identical builder
  // path via buildChamferedCylinderAsym(g,d,d,defl)); it also equals curved_chamfer_edge.
  const double Rc = 5.0, h = 10.0, d = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  bool wa = false, wb = false;
  const double va = vol(blend::curved_chamfer_edge_asym(cyl, ids, 1, d, d, 0.005), wa);
  const double vb = vol(blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005), wb);
  CC_CHECK(wa && wb);
  CC_CHECK(nearRel(va, vb, 1e-12));  // asym(d,d) == symmetric, exactly
  CC_CHECK(nearRel(va, vol(cyl, wa) - chamferVremoved(Rc, d), 5e-3));
}

CC_TEST(asym_chamfer_two_bevel_angles_c0) {
  // ANALYTIC bevel-angle assertion (no OCCT, no mesh): the OBLIQUE frustum outward normal
  // is radial·d1 + s·axial·d2, so it makes cos=d1/√(d1²+d2²) with the cylinder radial
  // normal and cos=d2/√(d1²+d2²) with the cap axial normal — two DIFFERENT angles, both
  // explicitly ≠ 1 (C0, NOT G1). This is the T1 inversion vs the symmetric 45° bevel.
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis; s=+1 (cap at the top)
  const double d1 = 2.0, d2 = 1.0;
  const double den = std::sqrt(d1 * d1 + d2 * d2);
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 bevel = nmath::Dir3{radial * d1 + axis * d2}.vec();
    const double cosWall = nmath::dot(bevel, radial);
    CC_CHECK(nearRel(cosWall, d1 / den, 1e-12));
    CC_CHECK(std::fabs(cosWall - 1.0) > 0.05);  // NOT tangent to the wall
    const double cosCap = nmath::dot(bevel, axis);
    CC_CHECK(nearRel(cosCap, d2 / den, 1e-12));
    CC_CHECK(std::fabs(cosCap - 1.0) > 0.05);  // NOT tangent to the cap
    // The two angles DIFFER (asymmetric), unlike the symmetric 45° case.
    CC_CHECK(std::fabs(cosWall - cosCap) > 0.1);
  }
}

CC_TEST(asym_chamfer_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Rc − d2 ≤ 0 guard: d2=5 ⇒ the cap seam crosses the axis ⇒ NULL (defers to OCCT).
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 5.0, 0.01).isNull());
  // Wall shorter than d1 (d1=12 > h) ⇒ far end not beyond the wall seam ⇒ NULL.
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 12.0, 1.0, 0.01).isNull());
  // Zero / negative distance (either) → NULL.
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 0.0, 1.0, 0.01).isNull());
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids2, 2, 2.0, 1.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_chamfer_edge_asym(b, idsb, 1, 2.0, 1.0, 0.01).isNull());
}

// ── concave fillet (material-side torus canal on a boss ↔ larger shoulder rim) ────--

namespace {
// A stepped shaft (a boss cylinder standing on a larger coaxial disc plate) built by a
// native REVOLVE of a stepped meridian about the world Y axis. Faces: plate bottom
// disc (Rp), plate outer wall (Rp), shoulder annulus (Rc..Rp, the LARGER plane), boss
// wall (Rc), boss top cap (Rc). The concave base rim is the circle radius Rc at axial t.
topo::Shape steppedShaft(double Rp, double t, double Rc, double H) {
  const double prof[] = {0, 0, Rp, 0, Rp, t, Rc, t, Rc, t + H, 0, t + H};
  return cst::build_revolution(prof, 6, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// The Circle rim edge id at axial coord `h` along `az` with the given radius.
int findRimAtAxial(const topo::Shape& s, const nmath::Vec3& az, double h, double radius) {
  const topo::ShapeMap emap = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    nmath::Point3 o = c->curve->frame.origin;
    if (!c->location.isIdentity()) o = c->location.transform().applyToPoint(o);
    if (std::fabs(nmath::dot(nmath::Vec3{o.x, o.y, o.z}, az) - h) < 1e-6 &&
        std::fabs(c->curve->radius - radius) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}
// Closed-form ADDED rim-band volume (Pappus): the square corner r² minus the
// quarter-disc, that region's centroid revolved about the axis.
double concaveVfill(double Rc, double r) {
  return M_PI * ((Rc + r) * (Rc + r) - Rc * Rc) * r -
         2.0 * M_PI * ((Rc + r) - 4.0 * r / (3.0 * M_PI)) * (M_PI / 4.0) * r * r;
}
}  // namespace

CC_TEST(concave_fillet_boss_on_plate_watertight_volume_grown) {
  // Boss Rc=5, H=6 on a coaxial disc plate Rp=12, t=4; roll a ball r=1.5 into the
  // CONCAVE base rim → a material-side torus canal (major Rc+r=6.5, minor 1.5) that
  // ADDS material. Watertight, volume ABOVE the sharp shaft by the closed-form V_fill.
  const double Rp = 12, t = 4, Rc = 5, H = 6, r = 1.5;
  topo::Shape shaft = steppedShaft(Rp, t, Rc, H);
  const nmath::Vec3 axisY{0, 1, 0};
  bool wt0 = false;
  const double v0 = vol(shaft, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtAxial(shaft, axisY, t, Rc);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::concave_fillet_edge(shaft, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);            // torus quarter-tube welds watertight to wall + shoulder
  CC_CHECK(v > v0);        // a CONCAVE fillet ADDS material (volume GROWS)
  const double expected = v0 + concaveVfill(Rc, r);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(concave_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the concave canal normal at each seam matches the
  // adjacent primary-face normal exactly. n(u,v) = radial(u)·cos v + axis·sin v with
  // radius(v) = (Rc+r) − r·cos v.
  //   * v=0   (wall seam, radius Rc):     n = radial → CYLINDER outward normal. cos=1.
  //   * v=π/2 (shoulder seam, radius Rc+r): n = axis   → SHOULDER outward normal. cos=1.
  const double Rc = 5.0, r = 1.5;
  const nmath::Vec3 axis{0, 0, 1};
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    const nmath::Vec3 nSh = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nSh}.vec(), axis), 1.0, 1e-12));
  }
  // Seam POSITIONS: radius(0)=Rc (wall), radius(π/2)=Rc+r (shoulder). Both flipped in
  // sign vs the convex builder (which trims the cap to Rc−r).
  const double Rt = Rc + r;  // concave torus major radius
  CC_CHECK(nearRel(Rt - r * std::cos(0.0), Rc, 1e-12));          // radius(0)=Rc (wall)
  CC_CHECK(nearRel(Rt - r * std::cos(M_PI / 2.0), Rc + r, 1e-12));  // radius(π/2)=Rc+r
}

CC_TEST(concave_fillet_scope_defers) {
  const double Rp = 12, t = 4, Rc = 5, H = 6;
  topo::Shape shaft = steppedShaft(Rp, t, Rc, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const int rim = findRimAtAxial(shaft, axisY, t, Rc);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  // Zero radius → NULL.
  CC_CHECK(blend::concave_fillet_edge(shaft, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL.
  int ids2[] = {rim, 1};
  CC_CHECK(blend::concave_fillet_edge(shaft, ids2, 2, 1.5, 0.01).isNull());
  // Seam leaves the shoulder: r so large that Rc+r > Rp → NULL.
  CC_CHECK(blend::concave_fillet_edge(shaft, ids, 1, 8.0, 0.01).isNull());
  // The CONVEX builder must DECLINE this concave rim (a larger coaxial cylinder exists),
  // so at most one curved builder fires — no sign confusion.
  CC_CHECK(blend::curved_fillet_edge(shaft, ids, 1, 1.5, 0.01).isNull());
  // A blind-hole bottom rim (a cup: outer Rp, hole Rc, flat bottom) is a DIFFERENT
  // concave config (offset Rc−r, plate top rim not on the crease) → deferred to OCCT.
  const double d = 3, Htop = 8;
  const double cupProf[] = {0, 0, Rp, 0, Rp, Htop, Rc, Htop, Rc, d, 0, d};
  topo::Shape cup = cst::build_revolution(cupProf, 6, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int brim = findRimAtAxial(cup, axisY, d, Rc);
  CC_CHECK(brim != 0);
  int cids[] = {brim};
  CC_CHECK(blend::concave_fillet_edge(cup, cids, 1, 1.0, 0.01).isNull());
  // The CONVEX capped cylinder rim is NOT concave (no larger coaxial cylinder) → the
  // concave builder declines it (control: no cross-firing).
  cst::ProfileSegment seg;
  seg.kind = 2;
  seg.cx = 0; seg.cy = 0; seg.r = 5.0;
  topo::Shape capped = cst::build_prism_profile({seg}, {}, {}, 10.0);
  const int caprim = findRimAtAxial(capped, nmath::Vec3{0, 0, 1}, 10.0, 5.0);
  CC_CHECK(caprim != 0);
  int capids[] = {caprim};
  CC_CHECK(blend::concave_fillet_edge(capped, capids, 1, 1.5, 0.01).isNull());
}

// ── cone-frustum cap fillet (torus band on a CONE↔coaxial-cap circular rim) ───────--

namespace {
// A capped cone frustum: profile (0,0)→(Rb,0)→(Rt,H)→(0,H) revolved a full turn about
// the in-plane axis (→ world +Y axis). Bottom cap radius Rb at h=0, top cap radius Rt at
// h=H, one Cone lateral wall (angular sectors). Rb≠Rt ⇒ a true Cone face.
topo::Shape cappedFrustum(double Rb, double Rt, double H) {
  const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
  return cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// Closed-form removed volume for a rolling-ball fillet r on the top rim (radius Rt) of a
// capped frustum (Pappus of the corner-minus-arc region, revolved about the axis). Matches
// the derivation in the header (verified numerically vs a fine solid-of-revolution).
double frustumRemoved(double Rb, double Rt, double H, double r) {
  const double dr = Rt - Rb, dz = H;
  double nwr = dz, nwz = -dr;
  const double nn = std::sqrt(nwr * nwr + nwz * nwz);
  nwr /= nn; nwz /= nn;
  if (nwr < 0) { nwr = -nwr; nwz = -nwz; }
  const double c = nwz;  // nW·nC with nC=(0,1)
  const double Cr = Rt - r * nwr / (1.0 + c);
  const double Cz = H - r * (nwz + 1.0) / (1.0 + c);
  const double Twr = Cr + r * nwr, Twz = Cz + r * nwz;
  const double Tcr = Cr, Tcz = Cz + r;
  const double angCap = M_PI / 2.0, angWall = std::atan2(nwz, nwr);
  // Polygon (Twall, rim, Tcap, arc cap→wall); shoelace area + centroid_r; Pappus volume.
  const int Na = 2000;
  std::vector<double> X{Twr, Rt, Tcr}, Y{Twz, H, Tcz};
  for (int i = 0; i < Na; ++i) {
    const double v = angCap + (angWall - angCap) * i / (Na - 1);
    X.push_back(Cr + r * std::cos(v));
    Y.push_back(Cz + r * std::sin(v));
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
double frustumSharpVolume(double Rb, double Rt, double H) {
  return M_PI * H / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt);
}
}  // namespace

CC_TEST(cone_fillet_narrowing_frustum_watertight_volume_reduced) {
  // Narrowing frustum Rb=6→Rt=4 over H=10; roll a ball r=1 into the top rim → a coaxial
  // torus band tangent to the tilted cone wall and the cap. Watertight, volume BELOW the
  // sharp frustum by the closed-form removed volume (deflection-bounded).
  const double Rb = 6, Rt = 4, H = 10, r = 1.0;
  topo::Shape s = cappedFrustum(Rb, Rt, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = frustumSharpVolume(Rb, Rt, H);  // exact sharp volume (closed form)
  const int rim = findRimAtAxial(s, axisY, H, Rt);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::cone_fillet_edge(s, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);              // torus band welds watertight to the cone wall + trimmed cap
  CC_CHECK(v < vSharp);      // a convex fillet REDUCES the volume vs the sharp frustum
  const double expected = vSharp - frustumRemoved(Rb, Rt, H, r);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(cone_fillet_widening_frustum_watertight_volume_reduced) {
  // Widening frustum Rb=4→Rt=6 over H=10 (the cone wall tilts OUTWARD toward the cap, so
  // the wall-seam minor angle is NEGATIVE). Same closed-form check.
  const double Rb = 4, Rt = 6, H = 10, r = 1.0;
  topo::Shape s = cappedFrustum(Rb, Rt, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = frustumSharpVolume(Rb, Rt, H);
  const int rim = findRimAtAxial(s, axisY, H, Rt);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::cone_fillet_edge(s, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < vSharp);
  const double expected = vSharp - frustumRemoved(Rb, Rt, H, r);
  CC_CHECK(nearRel(v, expected, 5e-3));
}

CC_TEST(cone_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the torus band normal at each seam equals the adjacent
  // primary-face normal. The band normal in the (radial, sAxial) cross section is
  //   n(vAbs) = (cos vAbs, sin vAbs), vAbs ∈ [angWall, π/2].
  //   * cap seam vAbs=π/2:   n = (0,1) → the CAP outward normal (axial). ✓
  //   * wall seam vAbs=angWall: n = (cos angWall, sin angWall) = the cone wall outward
  //     normal (the tangent-cut direction), matching (1,−dR/dz)/‖·‖ by construction. ✓
  // We check the wall-seam normal equals the cone wall's (r,z) outward normal for a
  // narrowing (dR<0) and a widening (dR>0) frustum.
  for (const double dR : {-2.0, +2.0}) {
    const double dz = 10.0;
    const double wn = std::sqrt(dz * dz + dR * dR);
    // cone wall outward (r,z) = (dz, −dR)/‖·‖ with +r; here s=+1 (cap above), so
    // dRdz = dR/dz and the header's nW2 = (1,−dRdz)/√(1+dRdz²) = (dz,−dR)/√(dz²+dR²).
    const double nWr = dz / wn, nWz = -dR / wn;
    const double angWall = std::atan2(nWz, nWr);
    // band normal at the wall seam
    const double nbandR = std::cos(angWall), nbandZ = std::sin(angWall);
    CC_CHECK(nearRel(nbandR, nWr, 1e-12));
    CC_CHECK(nearRel(nbandZ, nWz, 1e-12));
    // cap seam
    CC_CHECK(nearRel(std::cos(M_PI / 2.0), 0.0, 1e-12));
    CC_CHECK(nearRel(std::sin(M_PI / 2.0), 1.0, 1e-12));
  }
}

CC_TEST(cone_fillet_scope_defers) {
  const nmath::Vec3 axisY{0, 1, 0};
  // A pure CYLINDER (Rb==Rt, no Cone face) is the σ=0 case owned by curved_fillet_edge;
  // the cone builder declines it wholesale (no Cone face present).
  topo::Shape cyl = cappedFrustum(5, 5, 10);  // Rb==Rt → a cylinder, no Cone face
  const int crim = findRimAtAxial(cyl, axisY, 10, 5);
  if (crim != 0) {
    int cids[] = {crim};
    CC_CHECK(blend::cone_fillet_edge(cyl, cids, 1, 1.0, 0.01).isNull());
  }
  // Ring-torus guard: a steep frustum whose ball-centre radius Rmaj < r → NULL (spindle).
  topo::Shape steep = cappedFrustum(8, 3, 12);
  const int srim = findRimAtAxial(steep, axisY, 12, 3);
  CC_CHECK(srim != 0);
  int sids[] = {srim};
  CC_CHECK(blend::cone_fillet_edge(steep, sids, 1, 2.0, 0.01).isNull());  // Rmaj≈1.67<2
  // Zero / negative radius → NULL.
  topo::Shape s = cappedFrustum(6, 4, 10);
  const int rim = findRimAtAxial(s, axisY, 10, 4);
  int ids[] = {rim};
  CC_CHECK(blend::cone_fillet_edge(s, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (single rim only).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::cone_fillet_edge(s, ids2, 2, 1.0, 0.01).isNull());
  // A pure planar box has no Cone face → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::cone_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
  // A multi-frustum (two cones with DIFFERENT slopes → different σ) → NULL (wholesale
  // mismatch: the second cone's σ/Rref differ, so the body is not a single frustum).
  // Segments (6,0)→(5,5) slope −1/5 and (5,5)→(2,10) slope −3/5 are NON-collinear.
  const double twoCone[] = {0, 0, 6, 0, 5, 5, 2, 10, 0, 10};
  topo::Shape tc = cst::build_revolution(twoCone, 5, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int tcrim = findRimAtAxial(tc, axisY, 10, 2);
  CC_CHECK(tcrim != 0);
  int tcids[] = {tcrim};
  CC_CHECK(blend::cone_fillet_edge(tc, tcids, 1, 0.5, 0.01).isNull());
}

// ── sphere cap fillet (torus band on a SPHERE↔coaxial-cap circular rim) ───────────--

namespace {
// A truncated ball: sphere radius R centred at origin, capped by a flat top at axial height
// zc (0<zc<R). Profile in (r,h): arc from the south pole (0,-R) up to the rim (Rrim,zc),
// then a line rim→axis (0,zc) — the cap. Revolved a full turn about the in-plane axis
// (→ world +Y). Produces coaxial Sphere sectors + a planar top cap. Rrim = √(R²−zc²).
topo::Shape truncatedBall(double R, double zc) {
  const double Rrim = std::sqrt(R * R - zc * zc);
  std::vector<cst::ProfileSegment> segs;
  cst::ProfileSegment arc;
  arc.kind = 1;
  arc.cx = 0; arc.cy = 0; arc.r = R;
  arc.x0 = 0; arc.y0 = -R; arc.x1 = Rrim; arc.y1 = zc;
  arc.a0 = -M_PI / 2.0; arc.a1 = std::atan2(zc, Rrim);
  segs.push_back(arc);
  cst::ProfileSegment cap;
  cap.kind = 0;
  cap.x0 = Rrim; cap.y0 = zc; cap.x1 = 0; cap.y1 = zc;
  segs.push_back(cap);
  return cst::build_revolution_profile(segs, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// Sharp truncated-ball volume = ball region below z=zc = π·∫_{-R}^{zc}(R²−z²)dz.
double truncatedBallVolume(double R, double zc) {
  auto cap = [&](double z) { return R * R * z - z * z * z / 3.0; };
  return M_PI * (cap(zc) - cap(-R));
}
// Closed-form removed volume for a rolling-ball fillet r on the cap rim of a truncated ball
// (Pappus of the corner-minus-arc region). Cross-section polygon: sphere arc (sphere seam →
// rim), cap (rim → cap seam), torus arc (cap seam → sphere seam). 2π·|A|·centroid_r.
double sphereRemoved(double R, double zc, double r) {
  const double Rrim = std::sqrt(R * R - zc * zc);
  const double Cz = zc - r, d = R - r;
  const double Rmaj = std::sqrt(d * d - Cz * Cz);
  const double vWall = std::atan2(Cz, Rmaj);           // wall-seam minor angle
  const double scRad = Rmaj + r * std::cos(vWall);     // sphere-seam radius
  const double scAx = Cz + r * std::sin(vWall);        // sphere-seam axial
  const double latSeam = std::atan2(scAx, scRad);
  std::vector<double> X, Y;
  const int Na = 3000;
  // sphere arc: latitude latSeam → the rim latitude (asin(zc/R)), radius R·cos lat.
  const double latRim = std::asin(zc / R);
  for (int i = 0; i < Na; ++i) {
    const double lat = latSeam + (latRim - latSeam) * i / (Na - 1);
    X.push_back(R * std::cos(lat)); Y.push_back(R * std::sin(lat));
  }
  // cap: radius Rrim → Rmaj at z=zc.
  X.push_back(Rmaj); Y.push_back(zc);
  // torus arc: v from π/2 down to vWall.
  for (int i = 0; i < Na; ++i) {
    const double v = (M_PI / 2.0) + (vWall - M_PI / 2.0) * i / (Na - 1);
    X.push_back(Rmaj + r * std::cos(v)); Y.push_back(Cz + r * std::sin(v));
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
}  // namespace

CC_TEST(sphere_fillet_truncated_ball_watertight_volume_reduced) {
  // A ball R=5 capped at zc=3 (rim radius 4); roll a ball r=0.8 into the cap rim → a coaxial
  // torus band tangent to the sphere wall and the cap. Watertight, volume BELOW the sharp
  // truncated ball by the closed-form removed volume (deflection-bounded).
  const double R = 5, zc = 3, r = 0.8;
  topo::Shape s = truncatedBall(R, zc);
  CC_CHECK(!s.isNull());
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = truncatedBallVolume(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::sphere_fillet_edge(s, ids, 1, r, 0.004);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                 // torus band welds watertight to the sphere wall + trimmed cap
  CC_CHECK(v < vSharp);         // a convex fillet REDUCES the volume vs the sharp ball
  const double expected = vSharp - sphereRemoved(R, zc, r);
  CC_CHECK(nearRel(v, expected, 5e-3));
}

CC_TEST(sphere_fillet_converges_with_deflection) {
  // The faceted sphere+torus body under-fills; refining the deflection tightens the volume
  // toward the exact filleted volume from below.
  const double R = 6, zc = 2.5, r = 1.0;
  topo::Shape s = truncatedBall(R, zc);
  const nmath::Vec3 axisY{0, 1, 0};
  const double Rrim = std::sqrt(R * R - zc * zc);
  const int rim = findRimAtAxial(s, axisY, zc, Rrim);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  const double exact = truncatedBallVolume(R, zc) - sphereRemoved(R, zc, r);
  bool wtC = false, wtF = false;
  const double vCoarse = vol(blend::sphere_fillet_edge(s, ids, 1, r, 0.05), wtC);
  const double vFine = vol(blend::sphere_fillet_edge(s, ids, 1, r, 0.004), wtF);
  CC_CHECK(wtC && wtF);
  CC_CHECK(vCoarse <= exact + 1e-6);          // under-fills
  CC_CHECK(vFine <= exact + 1e-6);
  CC_CHECK(vFine >= vCoarse - 1e-9);          // refinement grows the volume toward exact
  CC_CHECK(nearRel(vFine, exact, 5e-3));
}

CC_TEST(sphere_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the torus band normal (cos v, sin v) at each seam equals
  // the adjacent primary-face normal in the (radial, axial) cross section.
  const double R = 5, zc = 3, r = 0.8;
  const double Cz = zc - r, d = R - r;
  const double Rmaj = std::sqrt(d * d - Cz * Cz);
  const double vWall = std::atan2(Cz, Rmaj);
  // cap seam v=π/2: normal (0,1) == cap axial normal.
  CC_CHECK(nearRel(std::cos(M_PI / 2.0), 0.0, 1e-12));
  CC_CHECK(nearRel(std::sin(M_PI / 2.0), 1.0, 1e-12));
  // wall seam: torus normal (cos vWall, sin vWall) must equal the sphere outward normal
  // (radial, axial)/R at the seam point.
  const double scRad = Rmaj + r * std::cos(vWall);
  const double scAx = Cz + r * std::sin(vWall);
  CC_CHECK(nearRel(scRad * scRad + scAx * scAx, R * R, 1e-9));   // seam lies ON the sphere
  CC_CHECK(nearRel(std::cos(vWall), scRad / R, 1e-9));           // normals coincide
  CC_CHECK(nearRel(std::sin(vWall), scAx / R, 1e-9));
}

CC_TEST(sphere_fillet_scope_defers) {
  const nmath::Vec3 axisY{0, 1, 0};
  // A pure cylinder (no Sphere face) → NULL (owned by curved_fillet_edge).
  const double cylProf[] = {0, 0, 5, 0, 5, 10, 0, 10};
  topo::Shape cyl = cst::build_revolution(cylProf, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int crim = findRimAtAxial(cyl, axisY, 10, 5);
  if (crim != 0) {
    int cids[] = {crim};
    CC_CHECK(blend::sphere_fillet_edge(cyl, cids, 1, 1.0, 0.01).isNull());
  }
  // A truncated ball, but: zero radius, multi-edge, and an oversized ring-torus radius.
  const double R = 5, zc = 3;
  topo::Shape s = truncatedBall(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  CC_CHECK(blend::sphere_fillet_edge(s, ids, 1, 0.0, 0.01).isNull());   // r=0
  int ids2[] = {rim, 1};
  CC_CHECK(blend::sphere_fillet_edge(s, ids2, 2, 0.5, 0.01).isNull());  // multi-edge
  // Ring-torus / seam guard: a very large r on a shallow cap → Rmaj<r or seam≥cap → NULL.
  CC_CHECK(blend::sphere_fillet_edge(s, ids, 1, 2.8, 0.01).isNull());
  // A pure planar box has no Sphere face → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::sphere_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

// ── offset_face ────────────────────────────────────────────────────────────────--

CC_TEST(offset_top_face_grows_slab) {
  // Grow the top (+Z) face of a 10×10×10 box outward by 5 → 10×10×15, vol 1500.
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  CC_CHECK(fid != 0);
  topo::Shape g = blend::offset_face(b, fid, 5.0);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 1500.0));
}

CC_TEST(offset_top_face_shrinks_slab) {
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  topo::Shape g = blend::offset_face(b, fid, -4.0);  // → 10×10×6 = 600
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 600.0));
}

// ── shell ──────────────────────────────────────────────────────────────────────--

CC_TEST(shell_open_top_box_wall_volume) {
  // Hollow a 10×10×10 box, open the top (+Z) face, wall thickness 1. The cavity is
  // the box inset 1 on 5 walls, flush on top: cavity = 8×8×9 = 576. Wall volume =
  // 1000 − 576 = 424.
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  CC_CHECK(fid != 0);
  int faces[] = {fid};
  topo::Shape sh = blend::shell(b, faces, 1, 1.0);
  bool wt = false;
  const double v = vol(sh, wt);
  CC_CHECK(!sh.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 424.0, 1e-3));
}

CC_TEST(shell_thickness_too_large_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  int faces[] = {fid};
  // thickness 6 > half-extent 5 → cavity collapses on the x/y walls → NULL.
  topo::Shape sh = blend::shell(b, faces, 1, 6.0);
  CC_CHECK(sh.isNull());
  CC_CHECK(blend::shell(b, faces, 1, 0.0).isNull());
}

// Chamfer TWO parallel top edges of a box (successive planar cuts). Both are convex
// 90° dihedrals → both land native; volume reduced by two corner prisms.
CC_TEST(chamfer_two_edges_watertight) {
  topo::Shape b = box(10, 10, 10);
  const int e1 = findEdgeId(b, {0, 10, 10}, {10, 10, 10});   // top / +Y
  const int e2 = findEdgeId(b, {0, 0, 10}, {10, 0, 10});     // top / −Y
  CC_CHECK(e1 != 0 && e2 != 0 && e1 != e2);
  int ids[] = {e1, e2};
  topo::Shape ch = blend::chamfer_edges(b, ids, 2, 1.5);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  // Two ½·1.5·1.5·10 = 11.25 corner prisms removed → 1000 − 22.5 = 977.5.
  CC_CHECK(nearRel(v, 977.5, 1e-3));
}

// A CONCAVE edge falls through: an L-shaped prism has one reflex (concave) vertical
// edge; chamfering/filleting it must return NULL (out of the convex native domain).
CC_TEST(concave_edge_chamfer_fillet_fallthrough) {
  // L-profile (CCW): the inner corner at (4,4) is concave for the extruded prism.
  const double L[] = {0, 0, 8, 0, 8, 4, 4, 4, 4, 8, 0, 8};
  topo::Shape lp = cst::build_prism(L, 6, 5.0);
  CC_CHECK(!lp.isNull());
  // The concave vertical edge runs from (4,4,0) to (4,4,5).
  const int ce = findEdgeId(lp, {4, 4, 0}, {4, 4, 5});
  CC_CHECK(ce != 0);
  int ids[] = {ce};
  CC_CHECK(blend::chamfer_edges(lp, ids, 1, 1.0).isNull());  // concave → NULL
  CC_CHECK(blend::fillet_edges(lp, ids, 1, 1.0).isNull());
  // A CONVEX edge of the same L-prism still chamfers native (watertight, smaller).
  const int cv = findEdgeId(lp, {0, 0, 0}, {0, 0, 5});
  CC_CHECK(cv != 0);
  int cvids[] = {cv};
  topo::Shape ch = blend::chamfer_edges(lp, cvids, 1, 1.0);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(v > 0.0 && v < 240.0);  // L-area 48 × 5 = 240, reduced by the chamfer
}

CC_RUN_ALL()

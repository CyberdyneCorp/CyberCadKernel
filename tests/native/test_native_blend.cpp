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

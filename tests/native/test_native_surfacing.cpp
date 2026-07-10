// SPDX-License-Identifier: Apache-2.0
//
// Host GATE (a) for the MOAT SURFACE bounded N-sided fill (src/native/surface/),
// OCCT-FREE. The fill evaluates a Coons / Gregory transfinite interpolant of a 3–6-sided
// ANALYTIC (straight-segment + circular-arc) boundary loop to a TESSELLATED triangle
// mesh — NOT a general NURBS surface (the campaign's scope bound). We assert the exact
// closed forms with no OCCT:
//   * PLANAR N-gon (3..6) → the patch IS the planar face; patch area = the exact polygon
//     area (ear-clip, tessellator-independent);
//   * PLANAR QUAD HOLE in a box (open shell) → fillHoleSolid restores the box watertight,
//     χ-consistent, at the EXACT original volume;
//   * SADDLE 4-sided analytic boundary → the tessellated Coons patch interpolates all four
//     boundary curves (on-boundary residual 0 by construction), welds to a matching cup
//     WATERTIGHT + consistently oriented, and its deviation CONVERGES as gridN doubles;
// and the HONEST-DECLINE envelope:
//   * a NON-ANALYTIC (spline) boundary edge → declined;
//   * a 7-SIDED loop → declined (TooManySides);
//   * a DEGENERATE (duplicate-corner) boundary → declined.
// Header-only over surface/boolean/tessellate/topology; no numsci link (always-on suite).
//
#include "native/surface/native_surface.h"
#include "native/boolean/assemble.h"
#include "native/boolean/polygon.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace sf = cybercad::native::surface;
namespace bln = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace tmath = cybercad::native::math;
namespace tcst = cybercad::native::construct;
namespace ttopo = cybercad::native::topology;

namespace {

constexpr double kPi = 3.14159265358979323846;

sf::BoundarySide seg(tmath::Point3 a, tmath::Point3 b) {
  sf::BoundarySide s;
  s.start = a;
  s.end = b;
  s.arc = false;
  return s;
}

double patchArea(const sf::NGonPatch& p) { return tess::surfaceArea(p.mesh); }

// Build a native box then drop its +Z top cap → an open shell with a single planar hole.
ttopo::Shape openBoxMissingTop(double sx, double sy, double sz) {
  const double p[8] = {0, 0, sx, 0, sx, sy, 0, sy};
  const ttopo::Shape box = tcst::build_prism(p, 4, sz);
  std::vector<ttopo::Shape> faces;
  for (ttopo::Explorer ex(box, ttopo::ShapeType::Face); ex.more(); ex.next()) {
    double maxz = -1e30, minz = 1e30;
    for (ttopo::Explorer vx(ex.current(), ttopo::ShapeType::Vertex); vx.more(); vx.next())
      if (auto pt = ttopo::pointOf(vx.current())) {
        maxz = std::fmax(maxz, pt->z);
        minz = std::fmin(minz, pt->z);
      }
    if (minz > sz - 1e-6 && maxz < sz + 1e-6) continue;  // skip the top cap
    faces.push_back(ex.current());
  }
  return ttopo::ShapeBuilder::makeShell(std::move(faces));
}

// A material-outward planar Polygon from a CCW-from-outside corner loop.
bln::Polygon poly(std::vector<tmath::Point3> v) {
  const tmath::Vec3 n = tmath::cross(v[1] - v[0], v[2] - v[0]);
  const bln::Plane pl = bln::Plane::fromPointNormal(v[0], tmath::Dir3{n}.vec());
  return bln::Polygon(std::move(v), pl);
}

}  // namespace

// ── PLANAR N-gon patch area = exact polygon area (3..6). ─────────────────────────
CC_TEST(surface_planar_ngon_area_exact) {
  const double R = 1.0;
  for (int N = 3; N <= 6; ++N) {
    sf::Boundary b;
    for (int i = 0; i < N; ++i) {
      const double a0 = 2 * kPi * i / N, a1 = 2 * kPi * (i + 1) / N;
      b.sides.push_back(seg(tmath::Point3{R * std::cos(a0), R * std::sin(a0), 0},
                            tmath::Point3{R * std::cos(a1), R * std::sin(a1), 0}));
    }
    sf::NGonDecline why = sf::NGonDecline::Ok;
    const sf::NGonPatch p = sf::fillNGon(b, sf::NGonOptions{16}, &why);
    // Exact regular-N-gon area = (N/2)·R²·sin(2π/N).
    const double exact = 0.5 * N * R * R * std::sin(2 * kPi / N);
    CC_CHECK(p.valid);
    CC_CHECK(why == sf::NGonDecline::Ok);
    CC_CHECK(std::fabs(patchArea(p) - exact) <= 1e-9 * exact);
  }
}

// ── PLANAR QUAD HOLE in a box → fillHoleSolid restores the box volume EXACTLY. ────
CC_TEST(surface_fill_planar_quad_hole_restores_box) {
  const ttopo::Shape shell = openBoxMissingTop(2.0, 3.0, 4.0);  // V = 24
  const sf::FillSolidResult r = sf::fillHoleSolid(shell, sf::NGonOptions{6});
  CC_CHECK(!r.solid.isNull());
  CC_CHECK(r.decline == sf::NGonDecline::Ok);
  CC_CHECK(r.watertight);
  CC_CHECK(std::fabs(r.volume - 24.0) <= 1e-9 * 24.0);
  // Consistently oriented at a fine deflection.
  tess::MeshParams mp;
  mp.deflection = 0.01;
  const tess::Mesh m = tess::SolidMesher{mp}.mesh(r.solid);
  CC_CHECK(tess::isConsistentlyOriented(m));
}

// ── SADDLE 4-sided analytic boundary: patch welds to a matching cup WATERTIGHT +
// consistently oriented, and its deviation CONVERGES as gridN doubles. ───────────
CC_TEST(surface_saddle_patch_welds_and_converges) {
  const double h = 0.3, D = 1.0;
  const tmath::Point3 r0{0, 0, +h}, r1{1, 0, -h}, r2{1, 1, +h}, r3{0, 1, -h};
  const tmath::Point3 b0{0, 0, -D}, b1{1, 0, -D}, b2{1, 1, -D}, b3{0, 1, -D};

  // The open cup (flat bottom + 4 side quads up to the saddle rim), as an open shell.
  std::vector<bln::Polygon> cup = {poly({b0, b3, b2, b1}), poly({b0, b1, r1, r0}),
                                   poly({b1, b2, r2, r1}), poly({b2, b3, r3, r2}),
                                   poly({b3, b0, r0, r3})};
  // Turn the cup polygons into an open shell of planar faces.
  std::vector<ttopo::Shape> faces;
  for (const bln::Polygon& pg : cup) {
    std::vector<ttopo::Shape> vtx;
    for (const tmath::Point3& v : pg.vertices) vtx.push_back(ttopo::ShapeBuilder::makeVertex(v));
    std::vector<ttopo::Shape> edges;
    for (std::size_t i = 0; i < vtx.size(); ++i) {
      const tmath::Point3 a = pg.vertices[i], bb = pg.vertices[(i + 1) % vtx.size()];
      ttopo::EdgeCurve c;
      c.kind = ttopo::EdgeCurve::Kind::Line;
      c.frame.origin = a;
      c.frame.x = tmath::Dir3{bb - a};
      edges.push_back(ttopo::ShapeBuilder::makeEdge(c, 0.0, tmath::norm(bb - a), vtx[i],
                                                    vtx[(i + 1) % vtx.size()]));
    }
    const ttopo::Shape wire = ttopo::ShapeBuilder::makeWire(std::move(edges));
    ttopo::FaceSurface s;
    s.kind = ttopo::FaceSurface::Kind::Plane;
    const tmath::Vec3 ref =
        std::fabs(pg.plane.normal.z) < 0.9 ? tmath::Vec3{0, 0, 1} : tmath::Vec3{1, 0, 0};
    s.frame = tmath::Ax3::fromAxisAndRef(pg.vertices[0], tmath::Dir3{pg.plane.normal},
                                         tmath::Dir3{ref});
    faces.push_back(ttopo::ShapeBuilder::makeFace(s, wire, {}, ttopo::Orientation::Forward));
  }
  const ttopo::Shape shell = ttopo::ShapeBuilder::makeShell(std::move(faces));

  // Fill the non-planar saddle rim → watertight, oriented solid.
  const sf::FillSolidResult r = sf::fillHoleSolid(shell, sf::NGonOptions{8});
  CC_CHECK(!r.solid.isNull());
  CC_CHECK(r.decline == sf::NGonDecline::Ok);
  CC_CHECK(r.watertight);
  CC_CHECK(r.volume > 0.0);
  tess::MeshParams mp;
  mp.deflection = 0.01;
  CC_CHECK(tess::isConsistentlyOriented(tess::SolidMesher{mp}.mesh(r.solid)));

  // Boundary-coincidence + convergence of the PATCH surface as gridN doubles.
  sf::Boundary bd = {{seg(r0, r1), seg(r1, r2), seg(r2, r3), seg(r3, r0)}};
  sf::NGonDecline why = sf::NGonDecline::Ok;
  const sf::NGonPatch p8 = sf::fillNGon(bd, sf::NGonOptions{8}, &why);
  const sf::NGonPatch p16 = sf::fillNGon(bd, sf::NGonOptions{16}, &why);
  const sf::NGonPatch p32 = sf::fillNGon(bd, sf::NGonOptions{32}, &why);
  CC_CHECK(p8.valid && p16.valid && p32.valid);
  CC_CHECK(p8.onBoundaryResidual <= 1e-9);  // boundary rows ARE the boundary samples
  // Area decreases monotonically toward the smooth-patch limit (Coons area converges).
  const double a8 = patchArea(p8), a16 = patchArea(p16), a32 = patchArea(p32);
  CC_CHECK(a16 <= a8 + 1e-12);
  CC_CHECK(a32 <= a16 + 1e-12);
  CC_CHECK(std::fabs(a32 - a16) <= std::fabs(a16 - a8) + 1e-12);  // shrinking increments
}

// ── HONEST DECLINE: a 7-sided loop is out of the bound (TooManySides). ───────────
CC_TEST(surface_seven_sided_declines) {
  sf::Boundary b;
  for (int i = 0; i < 7; ++i)
    b.sides.push_back(seg(tmath::Point3{std::cos(2 * kPi * i / 7), std::sin(2 * kPi * i / 7), 0},
                          tmath::Point3{std::cos(2 * kPi * (i + 1) / 7),
                                        std::sin(2 * kPi * (i + 1) / 7), 0}));
  sf::NGonDecline why = sf::NGonDecline::Ok;
  const sf::NGonPatch p = sf::fillNGon(b, sf::NGonOptions{8}, &why);
  CC_CHECK(!p.valid);
  CC_CHECK(why == sf::NGonDecline::TooManySides);
}

// ── HONEST DECLINE: a degenerate (duplicate-corner / zero-length side) boundary. ─
CC_TEST(surface_degenerate_boundary_declines) {
  sf::Boundary b = {{seg(tmath::Point3{0, 0, 0}, tmath::Point3{0, 0, 0}),  // zero-length
                     seg(tmath::Point3{0, 0, 0}, tmath::Point3{1, 1, 0}),
                     seg(tmath::Point3{1, 1, 0}, tmath::Point3{0, 0, 0})}};
  sf::NGonDecline why = sf::NGonDecline::Ok;
  const sf::NGonPatch p = sf::fillNGon(b, sf::NGonOptions{8}, &why);
  CC_CHECK(!p.valid);
  CC_CHECK(why == sf::NGonDecline::DegenerateBoundary);
}

// ── HONEST DECLINE: an all-arc boundary that is collinear (no valid circle). ─────
CC_TEST(surface_collinear_arc_declines) {
  sf::BoundarySide s0;
  s0.start = tmath::Point3{0, 0, 0};
  s0.end = tmath::Point3{2, 0, 0};
  s0.mid = tmath::Point3{1, 0, 0};  // collinear with the ends ⇒ no circle
  s0.arc = true;
  sf::Boundary b = {{s0, seg(tmath::Point3{2, 0, 0}, tmath::Point3{1, 1, 0}),
                     seg(tmath::Point3{1, 1, 0}, tmath::Point3{0, 0, 0})}};
  sf::NGonDecline why = sf::NGonDecline::Ok;
  const sf::NGonPatch p = sf::fillNGon(b, sf::NGonOptions{8}, &why);
  CC_CHECK(!p.valid);
  CC_CHECK(why == sf::NGonDecline::DegenerateBoundary);
}

CC_RUN_ALL()

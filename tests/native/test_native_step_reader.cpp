// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native STEP AP203 READER (the first import slice) —
// OCCT-FREE Gate 1 (host round-trip, the core correctness proof of the two-gate
// model in openspec/NATIVE-REWRITE.md): build a native solid → step_export_native
// to a STEP string → step_import_native back → tessellate → assert the imported
// solid is valid + WATERTIGHT with volume/topology matching the original EXACTLY
// (both ends native). This proves the reader inverts the writer.
//
// It also asserts the tokenizer handles the writer's real forms (typed reals,
// negatives, exponents, enums, $/*, nested lists, combined instances), and the
// HONEST decline path (out-of-scope surface / assembly / non-mm unit → NULL).
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_step_reader.cpp \
//     src/native/exchange/step_reader.cpp src/native/exchange/step_writer.cpp \
//     src/native/heal/heal.cpp src/native/math/bspline.cpp \
//     src/native/math/bezier.cpp -I src -I tests -o test_native_step_reader
//
#include "native/exchange/native_exchange.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/edge_mesher.h"  // detail::edgeCurveLocal — the rational edge evaluator
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cctype>
#include <cmath>
#include <functional>
#include <sstream>
#include <string>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace ex = cybercad::native::exchange;
namespace ntess = cybercad::native::tessellate;

namespace {

// ── Fixtures (mirror the writer test) ─────────────────────────────────────────

// A 10×10×10 box: planar / LINE / PLANE, 6 faces, 12 edges, 8 vertices.
topo::Shape box10() {
  const double p[] = {0, 0, 10, 0, 10, 10, 0, 10};
  return cst::build_prism(p, 4, 10.0);
}

// A cylinder r=5, h=20: revolve a rectangle about Y (3× 120° CYLINDRICAL_SURFACE
// sectors + plane caps + CIRCLE rims).
topo::Shape cylinder() {
  const double p[] = {0, 0, 5, 0, 5, 20, 0, 20};
  return cst::build_revolution(p, 4, cst::RevolveAxis{0.0, 0.0, 0.0, 1.0}, 6.28318530717958647692);
}

// A 20×12×4 plate with one r=3 through-hole: a full-turn CYLINDRICAL_SURFACE hole
// wall (periodic wall + SEAM) + FACE_BOUND inner loops.
topo::Shape holedPlate() {
  const double outer[] = {0, 0, 20, 0, 20, 12, 0, 12};
  const std::vector<cst::CircleHole> holes = {{10.0, 6.0, 3.0}};
  return cst::build_prism_with_holes(outer, 4, holes, {}, 4.0);
}

// A spline-profile extrude prism: a 10×6 rectangle whose TOP edge is a kind-3
// B-spline (control pts bulging up to y≈9), extruded depth 4. The spline side is ONE
// true B_SPLINE_SURFACE face (residuals.h build_prism_profile_spline_walls) and the
// two caps are trimmed PLANE faces whose boundary includes the spline rim (a
// B_SPLINE_CURVE edge). This is the T3 fixture (deferred import task 7.4): a native
// watertight B-spline-FACE solid the writer serialises, exercising the reader's
// B-spline-surface projectUV + the B-spline-edge-on-plane pcurve reconstruction.
topo::Shape splineWallPrism() {
  const double splineXY[] = {10, 6, 7, 8, 3, 8, 0, 6};
  cst::ProfileSegment bottom, right, top, left;
  bottom.kind = 0; bottom.x0 = 0;  bottom.y0 = 0; bottom.x1 = 10; bottom.y1 = 0;
  right.kind  = 0; right.x0  = 10; right.y0  = 0; right.x1  = 10; right.y1  = 6;
  top.kind = 3; top.ptOffset = 0; top.ptCount = 4;  // spline top edge
  left.kind   = 0; left.x0   = 0;  left.y0   = 6; left.x1   = 0;  left.y1   = 0;
  return cst::build_prism_profile_spline({bottom, right, top, left}, splineXY, 8, {}, {}, 4.0);
}

// Robust watertight-volume of a solid via the native tessellator (planar solids
// mesh exactly; curved solids converge as deflection → 0).
double volumeOf(const topo::Shape& s, double deflection = 0.005) {
  ntess::MeshParams p;
  p.deflection = deflection;
  const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
  if (!ntess::isWatertight(m)) return -1.0;
  return std::fabs(ntess::enclosedVolume(m));
}

bool watertight(const topo::Shape& s, double deflection = 0.005) {
  ntess::MeshParams p;
  p.deflection = deflection;
  return ntess::isWatertight(ntess::SolidMesher{p}.mesh(s));
}

// Count faces / edges / vertices of a shape (dedup explorer).
int countType(const topo::Shape& s, topo::ShapeType t) {
  int n = 0;
  for (topo::Explorer ex(s, t); ex.more(); ex.next()) ++n;
  return n;
}

// A round-trip: export the native solid, import it back natively.
topo::Shape roundTrip(const topo::Shape& solid, const std::string& name) {
  const std::string step = ex::writeStepString(solid, name);
  if (step.empty()) return {};
  return ex::readStepString(step);
}

// World AABB of a solid via its meshed vertices (the tessellator bakes any
// Location into world coordinates, so a placed solid's box reflects its placement).
struct Box { double lo[3]; double hi[3]; };
Box worldBox(const topo::Shape& s, double deflection = 0.005) {
  ntess::MeshParams p;
  p.deflection = deflection;
  const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
  Box b{{1e300, 1e300, 1e300}, {-1e300, -1e300, -1e300}};
  for (const auto& v : m.vertices) {
    const double c[3] = {v.x, v.y, v.z};
    for (int k = 0; k < 3; ++k) { b.lo[k] = std::min(b.lo[k], c[k]); b.hi[k] = std::max(b.hi[k], c[k]); }
  }
  return b;
}

// The DATA-section body of a native STEP string (records between "DATA;\n" and
// "ENDSEC;"), with every #N renumbered by `offset` so two bodies can coexist in one
// file with disjoint ids. (Same splice the flat multi-solid test uses.)
std::string renumberedDataBody(const std::string& s, long offset) {
  const std::size_t d = s.find("DATA;");
  const std::size_t e = s.find("ENDSEC;", d);
  const std::size_t start = s.find('\n', d) + 1;
  const std::string body = s.substr(start, e - start);
  std::string out;
  for (std::size_t i = 0; i < body.size(); ++i) {
    out += body[i];
    if (body[i] == '#') {
      std::size_t j = i + 1; std::string num;
      while (j < body.size() && std::isdigit(static_cast<unsigned char>(body[j]))) num += body[j++];
      if (!num.empty()) { out += std::to_string(std::stol(num) + offset); i = j - 1; }
    }
  }
  return out;
}

// The #id of the (single) MANIFOLD_SOLID_BREP declared in a DATA body — used to
// wire an assembly transform onto a specific spliced root.
long firstBrepId(const std::string& body) {
  const std::size_t k = body.find("MANIFOLD_SOLID_BREP");
  if (k == std::string::npos) return 0;
  const std::size_t h = body.rfind('#', k);
  std::size_t j = h + 1; std::string num;
  while (j < body.size() && std::isdigit(static_cast<unsigned char>(body[j]))) num += body[j++];
  return num.empty() ? 0 : std::stol(num);
}

}  // namespace

// ── Tokenizer: the writer's real / enum / list forms parse and re-map ──────────
CC_TEST(reader_parses_writer_output_and_returns_a_solid) {
  const std::string step = ex::writeStepString(box10(), "box");
  CC_CHECK(!step.empty());
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());
  CC_CHECK(s.type() == topo::ShapeType::Solid);
}

// ── Box round-trip: exact volume + topology, watertight ───────────────────────
CC_TEST(box_round_trip_exact_volume_and_topology) {
  const topo::Shape orig = box10();
  const topo::Shape back = roundTrip(orig, "box");
  CC_CHECK(!back.isNull());
  CC_CHECK(watertight(back));

  const double v0 = volumeOf(orig);
  const double v1 = volumeOf(back);
  CC_CHECK(v0 > 0.0 && v1 > 0.0);
  CC_CHECK(std::fabs(v0 - 1000.0) < 1e-6);       // 10^3
  CC_CHECK(std::fabs(v1 - v0) < 1e-6);           // EXACT (planar)

  // Topology preserved (same face/vertex/edge counts as the original solid). The
  // Explorer dedups by node+location; a box shares 8 vertices and 12 unique edges
  // (each traversed by two faces), matching the original exactly.
  CC_CHECK(countType(back, topo::ShapeType::Face) == countType(orig, topo::ShapeType::Face));
  CC_CHECK(countType(back, topo::ShapeType::Vertex) == countType(orig, topo::ShapeType::Vertex));
  CC_CHECK(countType(back, topo::ShapeType::Edge) == countType(orig, topo::ShapeType::Edge));
  CC_CHECK(countType(back, topo::ShapeType::Face) == 6);
  CC_CHECK(countType(back, topo::ShapeType::Vertex) == 8);
}

// ── Cylinder round-trip: watertight, volume matches within tessellation bound ──
CC_TEST(cylinder_round_trip_watertight_volume_matches) {
  const topo::Shape orig = cylinder();
  const topo::Shape back = roundTrip(orig, "cyl");
  CC_CHECK(!back.isNull());
  CC_CHECK(watertight(back));

  const double v0 = volumeOf(orig);
  const double v1 = volumeOf(back);
  CC_CHECK(v0 > 0.0 && v1 > 0.0);
  // Analytic volume π·r²·h = π·25·20 ≈ 1570.796; both ends mesh the SAME sectored
  // cylinder, so v1 must equal v0 to fp (identical tessellation), and both land
  // near the analytic value within the deflection bound.
  CC_CHECK(std::fabs(v1 - v0) / v0 < 1e-9);
  CC_CHECK(std::fabs(v0 - 1570.7963267948966) / 1570.7963267948966 < 5e-3);

  // Same face count (3 wall sectors + 3+3 cap sectors = 9 or writer's actual).
  CC_CHECK(countType(back, topo::ShapeType::Face) == countType(orig, topo::ShapeType::Face));
}

// ── Holed plate round-trip: periodic hole wall + seam + FACE_BOUND ─────────────
CC_TEST(holed_plate_round_trip_watertight_volume_matches) {
  const topo::Shape orig = holedPlate();
  const topo::Shape back = roundTrip(orig, "plate");
  CC_CHECK(!back.isNull());
  CC_CHECK(watertight(back));

  const double v0 = volumeOf(orig);
  const double v1 = volumeOf(back);
  CC_CHECK(v0 > 0.0 && v1 > 0.0);
  // Plate 20·12·4 minus a r=3 through-hole depth 4: 960 − π·9·4 ≈ 960 − 113.097.
  CC_CHECK(std::fabs(v1 - v0) / v0 < 1e-9);
  const double expected = 960.0 - 3.14159265358979323846 * 9.0 * 4.0;
  CC_CHECK(std::fabs(v0 - expected) / expected < 5e-3);
}

// ── DECLINE: out-of-scope surface (TOROIDAL_SURFACE) → NULL ────────────────────
CC_TEST(decline_unsupported_surface_returns_null) {
  // Take a valid box file and swap a PLANE for a TOROIDAL_SURFACE keyword so the
  // surface mapper declines (parser succeeds; mapper returns NULL → OCCT).
  std::string step = ex::writeStepString(box10(), "box");
  const std::size_t pos = step.find("PLANE('',");
  CC_CHECK(pos != std::string::npos);
  step.replace(pos, 5, "TORUS");  // TORUS(... ) is not a modelled surface keyword
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(s.isNull());
}

// ── DECLINE: two MANIFOLD_SOLID_BREP (assembly) → NULL ─────────────────────────
CC_TEST(decline_multiple_roots_returns_null) {
  std::string step = ex::writeStepString(box10(), "box");
  // Inject a second (dangling) MANIFOLD_SOLID_BREP record into the DATA section so
  // the single-root gate declines.
  const std::size_t dpos = step.find("DATA;");
  CC_CHECK(dpos != std::string::npos);
  const std::size_t insert = step.find('\n', dpos) + 1;
  step.insert(insert, "#99001 = MANIFOLD_SOLID_BREP('extra',#99002);\n");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(s.isNull());
}

// ── DECLINE: non-mm unit (metre, no MILLI prefix) → NULL ───────────────────────
CC_TEST(decline_non_mm_unit_returns_null) {
  std::string step = ex::writeStepString(box10(), "box");
  const std::size_t pos = step.find("SI_UNIT(.MILLI.,.METRE.)");
  CC_CHECK(pos != std::string::npos);
  step.replace(pos, std::string("SI_UNIT(.MILLI.,.METRE.)").size(), "SI_UNIT($,.METRE.)");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(s.isNull());
}

// ── DECLINE: malformed record (unterminated string) → NULL ─────────────────────
CC_TEST(decline_malformed_input_returns_null) {
  const std::string bad =
      "ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1 = CARTESIAN_POINT(',(0.,0.,0.));\nENDSEC;\n"
      "END-ISO-10303-21;\n";
  const topo::Shape s = ex::readStepString(bad);
  CC_CHECK(s.isNull());
}

// ── Empty / no-DATA input → NULL (never a fabricated solid) ────────────────────
CC_TEST(decline_empty_input_returns_null) {
  CC_CHECK(ex::readStepString("").isNull());
  CC_CHECK(ex::readStepString("ISO-10303-21;\nHEADER;\nENDSEC;\n").isNull());
}

// ── T3 — B-SPLINE-FACE solid round-trip: EXACT volume + watertight ─────────────
// The deferred import task 7.4. The spline-wall prism carries a genuine
// B_SPLINE_SURFACE side face + B_SPLINE_CURVE cap-rim edges. It must round-trip
// native-export → native-import to the SAME watertight solid (exact volume): the
// reader reconstructs the B-spline surface's (u,v) (projectBSplineUV) and the
// B-spline rim's planar pcurve so the cap↔wall seam closes.
CC_TEST(spline_wall_face_round_trip_exact_volume_and_watertight) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  // The fixture must genuinely carry a B-spline SURFACE face (not the polyline
  // fallback) for this to be the T3 test; the writer only emits B_SPLINE_SURFACE for
  // a true B-spline face.
  const std::string step = ex::writeStepString(orig, "spline");
  CC_CHECK(!step.empty());
  CC_CHECK(step.find("B_SPLINE_SURFACE") != std::string::npos);

  const topo::Shape back = ex::readStepString(step);
  CC_CHECK(!back.isNull());
  if (back.isNull()) return;
  CC_CHECK(watertight(back));

  const double v0 = volumeOf(orig);
  const double v1 = volumeOf(back);
  CC_CHECK(v0 > 0.0 && v1 > 0.0);
  CC_CHECK(std::fabs(v1 - v0) / v0 < 1e-9);  // both ends mesh the SAME solid
  CC_CHECK(countType(back, topo::ShapeType::Face) == countType(orig, topo::ShapeType::Face));
}

// ── M4 — foreign trimmed B_SPLINE_SURFACE face admission (curved UV boundary) ───
// The MOAT M4 first slice: a trimmed B_SPLINE_SURFACE_WITH_KNOTS face whose EDGE_LOOP is a
// genuinely CURVED-in-(u,v) boundary (a rim CIRCLE, not an isoparametric line) is admitted
// and meshed watertight by the landed M0 mesher — PROVIDED the reconstructed pcurve is
// faithful (S_face(pcurve(t)) = C_edge(t)); any unfaithful edge DECLINES → OCCT. The
// fixture is a bump-capped cylinder: cylinder side + flat bottom disk + a biquadratic
// B-spline dome cap trimmed by the top circle (the M0 host keystone). Enclosed volume has
// the closed form V = πR²h (cylinder) + πR²H/2 (cap dome above the rim).
namespace {

namespace m = cybercad::native::math;
constexpr double kBumpH = 1.0;  // dome height above the rim (mirrors the M0 host gate)

topo::Shape bvertex(double x, double y, double z) {
  return topo::ShapeBuilder::makeVertex(m::Point3{x, y, z});
}
topo::Shape blineEdge(const topo::Shape& a, const topo::Shape& b) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Line;
  return topo::ShapeBuilder::makeEdge(c, 0.0, 1.0, a, b);
}
topo::Shape bcircleEdge(double R, double z, const topo::Shape& v0, const topo::Shape& v1) {
  topo::EdgeCurve c{};
  c.kind = topo::EdgeCurve::Kind::Circle;
  c.frame = m::Ax3{m::Point3{0, 0, z}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  c.radius = R;
  return topo::ShapeBuilder::makeEdgeWithVertices(c, 0.0, 6.28318530717958647692, {v0, v1});
}
// A biquadratic B-spline (knotted, unit-weight) bump-cap surface. `perturb` lifts the
// centre pole off the true dome so the rim circle no longer lies on the surface (the
// deliberately-unfaithful fixture the guard must reject).
topo::FaceSurface bumpCapBSpline(double capZ, double rho, double perturb) {
  const double k = kBumpH / (rho * rho), c0 = kBumpH / 2 - 0.25 * k, c1 = kBumpH / 2 + 0.25 * k;
  const double xc[3] = {-0.5, 0.0, 0.5}, fz[3] = {c0, c1, c0};
  topo::FaceSurface s{};
  s.kind = topo::FaceSurface::Kind::BSpline;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.degreeU = 2;
  s.degreeV = 2;
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 0, 1, 1, 1};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      const double dz = (i == 1 && j == 1) ? perturb : 0.0;
      s.poles.push_back(m::Point3{xc[i], xc[j], capZ + fz[i] + fz[j] + dz});
      s.weights.push_back(1.0);
    }
  return s;
}
// Closed solid: cylinder side (z∈[0,h]) + flat bottom disk + B-spline dome cap sharing the
// top circle. `perturb` != 0 makes the cap unfaithful to its rim.
topo::Shape bumpCappedCylinder(double R, double h, double perturb) {
  const double kPi = 3.14159265358979323846, kTwoPi = 6.28318530717958647692;
  const m::Ax3 fr{m::Point3{0, 0, 0}, m::Dir3{1, 0, 0}, m::Dir3{0, 1, 0}, m::Dir3{0, 0, 1}};
  topo::FaceSurface sideS{};
  sideS.kind = topo::FaceSurface::Kind::Cylinder;
  sideS.frame = fr;
  sideS.radius = R;
  auto vb = bvertex(R, 0, 0), vt = bvertex(R, 0, h);
  topo::Shape botC = bcircleEdge(R, 0, vb, vb), topC = bcircleEdge(R, h, vt, vt);
  topo::Shape seam0 = blineEdge(vb, vt), seam1 = blineEdge(vb, vt);
  topo::Shape sf0 = topo::ShapeBuilder::makeFace(sideS, topo::Shape{});
  auto pcLine = [&](m::Point3 o, m::Vec3 d) {
    topo::PCurve pc{};
    pc.kind = topo::EdgeCurve::Kind::Line;
    pc.origin2d = o;
    pc.dir2d = d;
    return pc;
  };
  auto bS = topo::ShapeBuilder::addPCurve(botC, sf0.tshape(), pcLine({0, 0, 0}, {1, 0, 0}));
  auto tS = topo::ShapeBuilder::addPCurve(topC, sf0.tshape(), pcLine({0, h, 0}, {1, 0, 0}));
  auto s0 = topo::ShapeBuilder::addPCurve(seam0, sf0.tshape(), pcLine({0, 0, 0}, {0, h, 0}));
  auto s1 = topo::ShapeBuilder::addPCurve(seam1, sf0.tshape(), pcLine({kTwoPi, 0, 0}, {0, h, 0}));
  topo::Shape sideFace = topo::ShapeBuilder::makeFace(
      sideS, topo::ShapeBuilder::makeWire({bS, s1, tS.reversedShape(), s0.reversedShape()}));
  topo::FaceSurface disk{};
  disk.kind = topo::FaceSurface::Kind::Plane;
  disk.frame = fr;
  topo::Shape kf0 = topo::ShapeBuilder::makeFace(disk, topo::Shape{});
  topo::PCurve pcd{};
  pcd.kind = topo::EdgeCurve::Kind::Circle;
  pcd.origin2d = {0, 0, 0};
  pcd.dir2d = {R, 0, 0};
  auto dOn = topo::ShapeBuilder::addPCurve(botC, kf0.tshape(), pcd);
  topo::Shape botCap = topo::ShapeBuilder::makeFace(disk, topo::ShapeBuilder::makeWire({dOn}), {},
                                                    topo::Orientation::Reversed);
  topo::FaceSurface cap = bumpCapBSpline(h, R, perturb);
  topo::Shape cf0 = topo::ShapeBuilder::makeFace(cap, topo::Shape{});
  topo::PCurve pcc{};
  pcc.kind = topo::EdgeCurve::Kind::Circle;
  pcc.origin2d = {0.5, 0.5, 0};
  pcc.dir2d = {R, 0, 0};
  auto cOn = topo::ShapeBuilder::addPCurve(topC, cf0.tshape(), pcc);
  topo::Shape capFace = topo::ShapeBuilder::makeFace(cap, topo::ShapeBuilder::makeWire({cOn}), {},
                                                     topo::Orientation::Forward);
  (void)kPi;
  return topo::ShapeBuilder::makeSolid(
      {topo::ShapeBuilder::makeShell({sideFace, botCap, capFace})});
}

}  // namespace

// The faithful fixture round-trips: the writer emits the B_SPLINE_SURFACE cap; the reader
// reconstructs the curved (circle) UV pcurve, the guard ACCEPTS it, and the M0 mesher meshes
// the solid WATERTIGHT with the same volume as the source and within tol of the closed form.
CC_TEST(foreign_trimmed_bspline_curved_boundary_admitted_watertight) {
  const double R = 0.4, h = 0.5;
  const double kPi = 3.14159265358979323846;
  const double truth = kPi * R * R * h + kPi * R * R * kBumpH / 2.0;
  const topo::Shape orig = bumpCappedCylinder(R, h, 0.0);
  CC_CHECK(!orig.isNull());
  const std::string step = ex::writeStepString(orig, "bumpcap");
  CC_CHECK(step.find("B_SPLINE_SURFACE") != std::string::npos);  // genuine foreign patch
  const topo::Shape back = ex::readStepString(step);
  CC_CHECK(!back.isNull());  // admitted (NOT declined) — the guard accepted the curved pcurve
  if (back.isNull()) return;
  CC_CHECK(watertight(back));  // the M0 mesher welded the curved cap↔side seam
  const double vb = volumeOf(back), vo = volumeOf(orig);
  CC_CHECK(vb > 0.0 && vo > 0.0);
  CC_CHECK(std::fabs(vb - vo) / vo < 1e-9);       // reconstructs the SAME solid as the source
  CC_CHECK(std::fabs(vb - truth) / truth < 0.03); // and matches the closed form within defl
  CC_CHECK(countType(back, topo::ShapeType::Face) == countType(orig, topo::ShapeType::Face));
}

// The deliberately-unfaithful fixture (centre pole lifted off the dome, so the rim circle
// no longer lies on the cap surface) DECLINES → NULL: the faithful-reconstruction guard
// measures the beyond-tolerance gap and refuses to emit an off-surface trimmed patch. The
// never-fabricate-geometry contract, so the engine round-trips the file through OCCT.
CC_TEST(foreign_trimmed_bspline_unfaithful_edge_declines) {
  const topo::Shape bad = bumpCappedCylinder(0.4, 0.5, 0.05);
  const std::string step = ex::writeStepString(bad, "bumpcapbad");
  CC_CHECK(step.find("B_SPLINE_SURFACE") != std::string::npos);
  CC_CHECK(ex::readStepString(step).isNull());  // guard declines the unfaithful curved edge
}

// ── M4-RATIONAL — foreign RATIONAL B-spline surface via the combined record ─────
// OCCT STEPControl_Writer emits a rational NURBS surface NOT as the bare
// B_SPLINE_SURFACE_WITH_KNOTS keyword but as a COMBINED Part-21 instance whose fields are
// split across sub-records:
//   ( BOUNDED_SURFACE() B_SPLINE_SURFACE(degU,degV,((poles)),form,cU,cV,si)
//     B_SPLINE_SURFACE_WITH_KNOTS((uM),(vM),(uK),(vK),spec) GEOMETRIC_REPRESENTATION_ITEM()
//     RATIONAL_B_SPLINE_SURFACE(((weights))) REPRESENTATION_ITEM('') SURFACE() )
// The reader now parses that record, populates FaceSurface::weights, and admits the face
// through the SAME guard + M0 mesher as the non-rational path. These host gates prove it
// against independent oracles with NO OCCT linked.
namespace {

// Split `s` on top-level commas (commas inside () or '' are ignored).
std::vector<std::string> splitTopCommas(const std::string& s) {
  std::vector<std::string> out;
  int depth = 0;
  bool inStr = false;
  std::size_t start = 0;
  for (std::size_t i = 0; i < s.size(); ++i) {
    const char c = s[i];
    if (inStr) {
      if (c == '\'') inStr = false;
      continue;
    }
    if (c == '\'') inStr = true;
    else if (c == '(') ++depth;
    else if (c == ')') --depth;
    else if (c == ',' && depth == 0) {
      out.push_back(s.substr(start, i - start));
      start = i + 1;
    }
  }
  out.push_back(s.substr(start));
  return out;
}

// Rewrite the single non-rational B_SPLINE_SURFACE_WITH_KNOTS record OCCT/the native writer
// emits into the COMBINED RATIONAL_B_SPLINE_SURFACE form, injecting `weights` (a "((w..)..)"
// grid). Splits the 13 keyword args EXACTLY as OCCT splits them across sub-records: degrees +
// poles + form flags into B_SPLINE_SURFACE, the RLE knots into B_SPLINE_SURFACE_WITH_KNOTS,
// the weights into RATIONAL_B_SPLINE_SURFACE. When `withRational` is false the RATIONAL
// sub-record is omitted (a plain combined surface the reader must still DECLINE). Everything
// else in the file (cylinder side, disk, edges) is the writer's genuine output.
std::string rationalizeCap(const std::string& step, const std::string& weights,
                           bool withRational = true) {
  const std::string kw = "B_SPLINE_SURFACE_WITH_KNOTS(";
  const std::size_t kwPos = step.find(kw);
  if (kwPos == std::string::npos) return step;
  const std::size_t argStart = kwPos + kw.size();
  int depth = 1;
  bool inStr = false;
  std::size_t i = argStart;
  for (; i < step.size(); ++i) {
    const char c = step[i];
    if (inStr) {
      if (c == '\'') inStr = false;
      continue;
    }
    if (c == '\'') inStr = true;
    else if (c == '(') ++depth;
    else if (c == ')' && --depth == 0) break;
  }
  const std::vector<std::string> a = splitTopCommas(step.substr(argStart, i - argStart));
  if (a.size() < 13) return step;
  std::string combined = "( BOUNDED_SURFACE() B_SPLINE_SURFACE(" + a[1] + "," + a[2] + "," +
                         a[3] + "," + a[4] + "," + a[5] + "," + a[6] + "," + a[7] +
                         ") B_SPLINE_SURFACE_WITH_KNOTS(" + a[8] + "," + a[9] + "," + a[10] +
                         "," + a[11] + "," + a[12] + ")";
  if (withRational)
    combined += " GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" + weights + ")";
  combined += " REPRESENTATION_ITEM('') SURFACE() )";
  return step.substr(0, kwPos) + combined + step.substr(i + 1);
}

// Author the 9×5 tensor-product rational-quadratic grid of an EXACT sphere of radius R,
// centre origin, +Z axis — the SAME grid the reader's internal revolvedProfile builds for a
// (a=b=R) axis ellipse (u = standard rational full circle {1,1/√2,..}; v = pole-to-pole
// meridian half-circle promoted to two 90° rational-quadratic arcs {1,1/√2,1,1/√2,1}). Emits
// the 45 CARTESIAN_POINTs and the combined RATIONAL_B_SPLINE_SURFACE #5 record. Because the
// grid is byte-identical to the proven revolution path, it meshes watertight to V=4/3·πR³.
std::string rationalSphereRecords(double R) {
  const double s2 = 1.0 / std::sqrt(2.0);
  const double Cx[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
  const double Cy[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
  const double Cw[9] = {1, s2, 1, s2, 1, s2, 1, s2, 1};
  const double mr[5] = {0, R, R, R, 0}, mz[5] = {R, R, 0, -R, -R}, mw[5] = {1, s2, 1, s2, 1};
  auto num = [](double x) {
    std::ostringstream o;
    o.precision(15);
    o << x;
    std::string t = o.str();
    if (t.find('.') == std::string::npos && t.find('e') == std::string::npos) t += ".";
    return t;
  };
  std::ostringstream pts, grid, wgrid;
  int id = 200, pid[9][5];
  for (int u = 0; u < 9; ++u)
    for (int v = 0; v < 5; ++v) {
      pid[u][v] = id;
      pts << "#" << id << " = CARTESIAN_POINT('',(" << num(Cx[u] * mr[v]) << ","
          << num(Cy[u] * mr[v]) << "," << num(mz[v]) << "));\n";
      ++id;
    }
  grid << "(";
  wgrid << "(";
  for (int u = 0; u < 9; ++u) {
    grid << (u ? ",(" : "(");
    wgrid << (u ? ",(" : "(");
    for (int v = 0; v < 5; ++v) {
      grid << (v ? "," : "") << "#" << pid[u][v];
      wgrid << (v ? "," : "") << num(Cw[u] * mw[v]);
    }
    grid << ")";
    wgrid << ")";
  }
  grid << ")";
  wgrid << ")";
  const double hp = 1.5707963267948966, pi = 3.141592653589793, tp = 4.71238898038469,
               twp = 6.283185307179586;
  std::ostringstream rec;
  rec << "#5 = ( BOUNDED_SURFACE() B_SPLINE_SURFACE(2,2," << grid.str()
      << ",.UNSPECIFIED.,.T.,.F.,.F.) B_SPLINE_SURFACE_WITH_KNOTS((3,2,2,2,3),(3,2,3),(" << num(0.)
      << "," << num(hp) << "," << num(pi) << "," << num(tp) << "," << num(twp) << "),(" << num(0.)
      << "," << num(hp) << "," << num(pi)
      << "),.UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_SURFACE(" << wgrid.str()
      << ") REPRESENTATION_ITEM('') SURFACE() );\n";
  return pts.str() + rec.str();
}

}  // namespace

// The all-unit-weight rational record denotes the SAME surface as the non-rational keyword
// form: the combined-record cap admits, meshes watertight, and reproduces the SAME solid as
// the non-rational M4 import — proving the split-record parse (fillBsplineGrid shared factor)
// and the row-major weight read pair correctly with the poles, and the rational-aware guard
// (bsplineSurfaceValue → math::nurbsSurfacePoint) accepts the faithful curved rim.
CC_TEST(foreign_rational_bspline_surface_unit_weights_matches_nonrational) {
  const double R = 0.4, h = 0.5;
  const topo::Shape orig = bumpCappedCylinder(R, h, 0.0);
  const std::string nonRat = ex::writeStepString(orig, "bumpcap");
  const std::string rat = rationalizeCap(nonRat, "((1.,1.,1.),(1.,1.,1.),(1.,1.,1.))");
  CC_CHECK(rat.find("RATIONAL_B_SPLINE_SURFACE") != std::string::npos);  // now a combined record
  const topo::Shape rBack = ex::readStepString(rat);
  const topo::Shape nBack = ex::readStepString(nonRat);
  CC_CHECK(!rBack.isNull());  // admitted via the combined rational arm
  CC_CHECK(!nBack.isNull());
  if (rBack.isNull() || nBack.isNull()) return;
  CC_CHECK(watertight(rBack));
  const double vr = volumeOf(rBack), vn = volumeOf(nBack);
  CC_CHECK(vr > 0.0 && vn > 0.0);
  CC_CHECK(std::fabs(vr - vn) / vn < 1e-9);  // unit weights ⇒ identical geometry to non-rational
  CC_CHECK(countType(rBack, topo::ShapeType::Face) == countType(nBack, topo::ShapeType::Face));
}

// The rational-aware faithful guard REJECTS an off-surface boundary: the perturbed cap (rim no
// longer on the surface), delivered as a unit-weight combined rational record, still DECLINES →
// OCCT. Proves the guard runs on the rational path (weights populated) exactly as non-rational.
CC_TEST(foreign_rational_bspline_surface_unfaithful_edge_declines) {
  const std::string rat = rationalizeCap(ex::writeStepString(bumpCappedCylinder(0.4, 0.5, 0.05),
                                                             "bad"),
                                         "((1.,1.,1.),(1.,1.,1.),(1.,1.,1.))");
  CC_CHECK(rat.find("RATIONAL_B_SPLINE_SURFACE") != std::string::npos);
  CC_CHECK(ex::readStepString(rat).isNull());  // rational-aware guard rejects the off-surface rim
}

// A malformed weight grid is an HONEST DECLINE (NULL → OCCT), never a clamped weight: a ragged
// row, a wrong row/column cardinality, or a non-positive (zero/negative) weight each declines.
CC_TEST(foreign_rational_bspline_surface_malformed_weights_decline) {
  const std::string base = ex::writeStepString(bumpCappedCylinder(0.4, 0.5, 0.0), "c");
  CC_CHECK(base.find("B_SPLINE_SURFACE_WITH_KNOTS") != std::string::npos);
  // Ragged (middle row has 2, not 3) → decline.
  CC_CHECK(ex::readStepString(rationalizeCap(base, "((1.,1.,1.),(1.,1.),(1.,1.,1.))")).isNull());
  // Wrong row count (2 rows, poles are 3×3) → decline.
  CC_CHECK(ex::readStepString(rationalizeCap(base, "((1.,1.,1.),(1.,1.,1.))")).isNull());
  // Zero weight (not strictly positive) → decline.
  CC_CHECK(ex::readStepString(rationalizeCap(base, "((1.,1.,1.),(1.,0.,1.),(1.,1.,1.))")).isNull());
  // Negative weight → decline.
  CC_CHECK(
      ex::readStepString(rationalizeCap(base, "((1.,1.,1.),(1.,-1.,1.),(1.,1.,1.))")).isNull());
}

// A COMBINED surface record that carries NO RATIONAL_B_SPLINE_SURFACE sub-record keeps the
// honest OCCT decline (unchanged): the new arm is reachable ONLY by the rational sub-record, so
// every other combined surface still returns NULL — the zero-regression contract at the read.
CC_TEST(combined_bspline_surface_without_rational_sub_declines) {
  const std::string base = ex::writeStepString(bumpCappedCylinder(0.4, 0.5, 0.0), "c");
  const std::string comb = rationalizeCap(base, "", /*withRational=*/false);
  CC_CHECK(comb.find("( BOUNDED_SURFACE()") != std::string::npos);          // genuinely combined
  CC_CHECK(comb.find("RATIONAL_B_SPLINE_SURFACE") == std::string::npos);    // but not rational
  CC_CHECK(ex::readStepString(comb).isNull());  // other combined surfaces still decline → OCCT
}

// ── M4-tail-2 — foreign RATIONAL B-spline CURVE (edge / trim geometry) ──────────
// OCCT STEPControl_Writer emits a rational NURBS edge/trim curve NOT as the bare
// B_SPLINE_CURVE_WITH_KNOTS keyword but as a COMBINED Part-21 instance whose fields are split
// across sub-records (the one-dimension-down analogue of the M4-RATIONAL surface record):
//   ( BOUNDED_CURVE() B_SPLINE_CURVE(deg,(poles),form,closed,si)
//     B_SPLINE_CURVE_WITH_KNOTS((mults),(knots),spec) CURVE() GEOMETRIC_REPRESENTATION_ITEM()
//     RATIONAL_B_SPLINE_CURVE((weights)) REPRESENTATION_ITEM('') )
// The reader now parses that record, populates EdgeCurve::weights, and evaluates the edge
// RATIONALLY (evalEdge / edgeCurveLocal → math::nurbsCurvePoint) through the SAME faithful guard as
// the non-rational path. These host gates prove it with NO OCCT linked.
namespace {

namespace mm = cybercad::native::math;

// The exact rational evaluator the reader's admitted rational edge is meshed with
// (native_tessellate → edgeCurveLocal) — byte-identical to the reader's own guard-side evalEdge.
mm::Point3 edgeAt(const topo::EdgeCurve& c, double t) {
  return ntess::detail::edgeCurveLocal(c, t);
}

// A weight list "(w0,w1,…)" with `n` entries, `first` overriding entry 0 (the rest 1.0). A caller
// injects a unit grid (first=1), a perturbing grid (first≠1), or a non-positive entry (first≤0).
std::string weightList(int n, double first) {
  std::ostringstream q;
  q << "(";
  for (int i = 0; i < n; ++i) {
    const double w = (i == 0) ? first : 1.0;
    std::ostringstream w0;
    w0.precision(15);
    w0 << w;
    std::string t = w0.str();  // STEP reals need a decimal point (or exponent)
    if (t.find('.') == std::string::npos && t.find('e') == std::string::npos) t += ".";
    q << (i ? "," : "") << t;
  }
  q << ")";
  return q.str();
}

// Rewrite EVERY non-rational B_SPLINE_CURVE_WITH_KNOTS('',deg,(poles),form,closed,si,(mults),
// (knots),spec) record the writer emits into the COMBINED RATIONAL_B_SPLINE_CURVE form, splitting
// the 9 keyword args EXACTLY as OCCT splits them across sub-records (alphabetical complex-entity
// order): deg+poles+flags → B_SPLINE_CURVE, RLE knots → B_SPLINE_CURVE_WITH_KNOTS, the weights →
// RATIONAL_B_SPLINE_CURVE. `makeWeights(nPoles)` supplies the "(w..)" list (a caller injects unit /
// perturbed / malformed grids). `withRational=false` omits the RATIONAL sub-record (a plain
// combined curve the reader must still DECLINE). Everything else is the writer's genuine output.
std::string rationalizeCurves(const std::string& step,
                              const std::function<std::string(int)>& makeWeights,
                              bool withRational = true) {
  const std::string kw = "B_SPLINE_CURVE_WITH_KNOTS(";
  std::string out = step;
  std::size_t from = 0;
  while (true) {
    const std::size_t kwPos = out.find(kw, from);
    if (kwPos == std::string::npos) break;
    const std::size_t argStart = kwPos + kw.size();
    int depth = 1;
    bool inStr = false;
    std::size_t i = argStart;
    for (; i < out.size(); ++i) {
      const char c = out[i];
      if (inStr) {
        if (c == '\'') inStr = false;
        continue;
      }
      if (c == '\'') inStr = true;
      else if (c == '(') ++depth;
      else if (c == ')' && --depth == 0) break;
    }
    const std::vector<std::string> a = splitTopCommas(out.substr(argStart, i - argStart));
    if (a.size() < 9) { from = i + 1; continue; }
    const std::size_t lp = a[2].find('('), rp = a[2].rfind(')');
    const int nPoles = (lp == std::string::npos || rp == std::string::npos || rp <= lp + 1)
                           ? 0
                           : static_cast<int>(splitTopCommas(a[2].substr(lp + 1, rp - lp - 1)).size());
    std::string combined = "( BOUNDED_CURVE() B_SPLINE_CURVE(" + a[1] + "," + a[2] + "," + a[3] +
                           "," + a[4] + "," + a[5] + ") B_SPLINE_CURVE_WITH_KNOTS(" + a[6] + "," +
                           a[7] + "," + a[8] + ") CURVE()";
    if (withRational)
      combined += " GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(" + makeWeights(nPoles) +
                  ")";
    combined += " REPRESENTATION_ITEM('') )";
    out = out.substr(0, kwPos) + combined + out.substr(i + 1);
    from = kwPos + combined.size();
  }
  return out;
}

std::function<std::string(int)> unitW = [](int n) { return weightList(n, 1.0); };

}  // namespace

// GATE A (HOST ANALYTIC) — the rational evaluator the reader meshes an admitted rational edge with
// reproduces a CLOSED-FORM curve to machine epsilon, and the rational weights are LOAD-BEARING. The
// fixture is the textbook rational-quadratic 90° circular arc (degree 2, poles {(R,0),(R,R),(0,R)},
// weights {1,1/√2,1}, knots {0,0,0,1,1,1}): the ONLY exact circle representation a NURBS admits, and
// one a non-rational B-spline on the SAME poles canNOT reproduce. Evaluating it rationally must land
// on the exact circle r=R; evaluating the same poles NON-rationally must miss it by O(10%).
CC_TEST(rational_bspline_curve_arc_reproduces_exact_circle) {
  const double R = 2.0, s = 1.0 / std::sqrt(2.0);
  topo::EdgeCurve arc{};
  arc.kind = topo::EdgeCurve::Kind::BSpline;
  arc.degree = 2;
  arc.poles = {mm::Point3{R, 0, 0}, mm::Point3{R, R, 0}, mm::Point3{0, R, 0}};
  arc.knots = {0, 0, 0, 1, 1, 1};
  arc.weights = {1.0, s, 1.0};  // rational: exact quarter circle
  double maxErrRat = 0.0;
  for (int k = 0; k <= 32; ++k) {
    const mm::Point3 p = edgeAt(arc, k / 32.0);
    maxErrRat = std::max(maxErrRat, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - R));
  }
  CC_CHECK(maxErrRat < 1e-9);  // exact circle to machine epsilon

  // The SAME poles/knots WITHOUT weights (non-rational) trace a parabola-like curve that misses the
  // circle by O(10%) — so the rational weights are genuinely load-bearing, not coincidental.
  topo::EdgeCurve nonRat = arc;
  nonRat.weights.clear();
  double maxErrNon = 0.0;
  for (int k = 0; k <= 32; ++k) {
    const mm::Point3 p = edgeAt(nonRat, k / 32.0);
    maxErrNon = std::max(maxErrNon, std::fabs(std::sqrt(p.x * p.x + p.y * p.y) - R));
  }
  CC_CHECK(maxErrNon > 1e-2);
}

// GATE A2 (HOST) — the all-unit-weight combined rational-curve record denotes the SAME geometry as
// the non-rational keyword form: the spline-wall prism's B_SPLINE_CURVE rim edges, rewritten into
// the combined RATIONAL_B_SPLINE_CURVE form with unit weights, admit, mesh watertight, and
// reproduce the SAME solid (volume + face count) as the non-rational round-trip — proving the
// split-record parse (fillBsplineCurve shared factor), the weight read paired with the poles, and
// the rational-aware faithful guard (evalEdge → nurbsCurvePoint) accepting the on-surface rim.
CC_TEST(foreign_rational_bspline_curve_unit_weights_matches_nonrational) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  const std::string nonRat = ex::writeStepString(orig, "spline");
  CC_CHECK(nonRat.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos);
  const std::string rat = rationalizeCurves(nonRat, unitW);
  CC_CHECK(rat.find("RATIONAL_B_SPLINE_CURVE") != std::string::npos);  // now a combined record
  const topo::Shape rBack = ex::readStepString(rat);
  const topo::Shape nBack = ex::readStepString(nonRat);
  CC_CHECK(!rBack.isNull());  // admitted via the combined rational-curve arm
  CC_CHECK(!nBack.isNull());
  if (rBack.isNull() || nBack.isNull()) return;
  CC_CHECK(watertight(rBack));
  const double vr = volumeOf(rBack), vn = volumeOf(nBack);
  CC_CHECK(vr > 0.0 && vn > 0.0);
  CC_CHECK(std::fabs(vr - vn) / vn < 1e-9);  // unit weights ⇒ identical geometry to non-rational
  CC_CHECK(countType(rBack, topo::ShapeType::Face) == countType(nBack, topo::ShapeType::Face));
}

// GATE A3 (HOST) — a genuinely NON-UNIT curve weight pulls the rational rim OFF the B-spline wall's
// top isocurve, so the wall face's faithful-reconstruction guard REJECTS the edge → DECLINE. This
// is the precise regression test for the rational-aware evalEdge: if the reader ignored curve
// weights, the perturbed rim would evaluate on-surface and WRONGLY admit.
CC_TEST(foreign_rational_bspline_curve_nonunit_weight_off_surface_declines) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  const std::string nonRat = ex::writeStepString(orig, "spline");
  const std::string rat = rationalizeCurves(nonRat, [](int n) { return weightList(n, 3.0); });
  CC_CHECK(rat.find("RATIONAL_B_SPLINE_CURVE") != std::string::npos);
  CC_CHECK(ex::readStepString(rat).isNull());  // off-surface rational rim → guard declines → OCCT
}

// GATE A4 (HOST) — a malformed weight list is an HONEST DECLINE (NULL → OCCT), never a clamped
// weight: a wrong cardinality (fewer weights than poles), a zero, or a negative weight each decline.
CC_TEST(foreign_rational_bspline_curve_malformed_weights_decline) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  const std::string base = ex::writeStepString(orig, "spline");
  CC_CHECK(base.find("B_SPLINE_CURVE_WITH_KNOTS") != std::string::npos);
  // Too few weights (cardinality < poles) → decline.
  CC_CHECK(
      ex::readStepString(rationalizeCurves(base, [](int) { return std::string("(1.,1.)"); }))
          .isNull());
  // Zero weight (not strictly positive) → decline.
  CC_CHECK(
      ex::readStepString(rationalizeCurves(base, [](int n) { return weightList(n, 0.0); })).isNull());
  // Negative weight → decline.
  CC_CHECK(ex::readStepString(rationalizeCurves(base, [](int n) { return weightList(n, -1.0); }))
               .isNull());
}

// GATE A5 (HOST) — a COMBINED curve record carrying NO RATIONAL_B_SPLINE_CURVE sub-record keeps the
// honest OCCT decline (unchanged): the new arm is reachable ONLY via the rational sub-record, so
// every other combined curve still returns NULL — the zero-regression contract at the read.
CC_TEST(combined_bspline_curve_without_rational_sub_declines) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  const std::string base = ex::writeStepString(orig, "spline");
  const std::string comb = rationalizeCurves(base, unitW, /*withRational=*/false);
  CC_CHECK(comb.find("( BOUNDED_CURVE()") != std::string::npos);        // genuinely combined
  CC_CHECK(comb.find("RATIONAL_B_SPLINE_CURVE") == std::string::npos);  // but not rational
  CC_CHECK(ex::readStepString(comb).isNull());  // other combined curves still decline → OCCT
}

// ── T2 — multi-solid file → a Compound of watertight Solids ────────────────────
// Concatenate two independent native box files' DATA sections into ONE file with two
// MANIFOLD_SOLID_BREP roots (renumbered so #ids don't collide), no assembly transform
// entities. The reader must import BOTH as a Compound of two solids (not decline), each
// watertight with the right volume.
CC_TEST(multi_solid_flat_file_imports_as_compound) {
  // Two boxes of different sizes so the per-solid volumes are distinguishable.
  const double pa[] = {0, 0, 10, 0, 10, 10, 0, 10};
  const double pb[] = {0, 0, 4, 0, 4, 4, 0, 4};
  const topo::Shape a = cst::build_prism(pa, 4, 10.0);   // vol 1000
  const topo::Shape b = cst::build_prism(pb, 4, 4.0);    // vol 64
  const std::string sa = ex::writeStepString(a, "A");
  const std::string sb = ex::writeStepString(b, "B");

  // Splice: take file A whole, and inject B's DATA-section records (renumbered by a
  // large offset so the two solids' #ids are disjoint) just after A's "DATA;".
  auto dataBody = [](const std::string& s) {
    const std::size_t d = s.find("DATA;");
    const std::size_t e = s.find("ENDSEC;", d);
    const std::size_t start = s.find('\n', d) + 1;
    return s.substr(start, e - start);
  };
  std::string bBody = dataBody(sb);
  // Renumber every #N in B's body to #(N+100000) so it cannot collide with A.
  std::string renum;
  for (std::size_t i = 0; i < bBody.size(); ++i) {
    renum += bBody[i];
    if (bBody[i] == '#') {
      std::size_t j = i + 1; std::string num;
      while (j < bBody.size() && std::isdigit(static_cast<unsigned char>(bBody[j]))) num += bBody[j++];
      if (!num.empty()) { renum += std::to_string(std::stol(num) + 100000); i = j - 1; }
    }
  }
  std::string merged = sa;
  const std::size_t insert = merged.find('\n', merged.find("DATA;")) + 1;
  merged.insert(insert, renum);

  const topo::Shape shape = ex::readStepString(merged);
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    ++solids;
    if (!watertight(e.current())) allWatertight = false;
    volSum += volumeOf(e.current());
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 64.0)) < 1e-6);  // exact (planar)
}

// ── ASSEMBLY (Form A) — a two-box transform tree → a PLACED Compound ───────────
// Splice two native boxes into one file (disjoint #ids), then wire a real assembly
// transform onto box B: an ITEM_DEFINED_TRANSFORMATION with a FROM(identity)/TO
// AXIS2_PLACEMENT_3D pair (pure translation by (30,5,0)) reached through a
// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION → (REPRESENTATION_RELATIONSHIP +
// REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION) whose CHILD shape-representation
// lists box B's MANIFOLD_SOLID_BREP. Box A (the root, no CDSR) stays at the origin.
// The reader must import BOTH as a placed Compound: A at [0,10]³, B translated.
CC_TEST(assembly_two_box_placed_compound) {
  const double pa[] = {0, 0, 10, 0, 10, 10, 0, 10};  // 10-cube at origin (vol 1000)
  const double pb[] = {0, 0, 6, 0, 6, 6, 0, 6};       // 6-cube (vol 216)
  const std::string sa = ex::writeStepString(cst::build_prism(pa, 4, 10.0), "A");
  const std::string sb = ex::writeStepString(cst::build_prism(pb, 4, 6.0), "B");

  const std::string bodyA = renumberedDataBody(sa, 0);
  const std::string bodyB = renumberedDataBody(sb, 100000);
  const long brepB = firstBrepId(bodyB);  // box B's spliced MANIFOLD_SOLID_BREP id
  CC_CHECK(brepB != 0);

  // Assembly transform records in a high id range (no collision). from=identity
  // frame at origin, to=frame translated by (30,5,0) → T places box B there.
  std::string asm_;
  asm_ += "#900001 = CARTESIAN_POINT('',(0.,0.,0.));\n";
  asm_ += "#900002 = DIRECTION('',(0.,0.,1.));\n";
  asm_ += "#900003 = DIRECTION('',(1.,0.,0.));\n";
  asm_ += "#900004 = AXIS2_PLACEMENT_3D('',#900001,#900002,#900003);\n";  // FROM
  asm_ += "#900005 = CARTESIAN_POINT('',(30.,5.,0.));\n";
  asm_ += "#900006 = AXIS2_PLACEMENT_3D('',#900005,#900002,#900003);\n";  // TO
  asm_ += "#900007 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900006);\n";
  // A CHILD shape-representation whose item list contains box B's brep.
  asm_ += "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(brepB) + "),#900020);\n";
  asm_ += "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n";  // PARENT (assembly)
  asm_ += "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900009) "
          "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) "
          "SHAPE_REPRESENTATION_RELATIONSHIP() );\n";
  asm_ += "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n";
  asm_ += "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n";
  asm_ += "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900014,#900015,$);\n";
  asm_ += "#900020 = GEOMETRIC_REPRESENTATION_CONTEXT(3);\n";  // context stub (unused)

  std::string merged = sa;
  const std::size_t insert = merged.find('\n', merged.find("DATA;")) + 1;
  merged.insert(insert, bodyB + asm_);

  const topo::Shape shape = ex::readStepString(merged);
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  Box boxA{}, boxB_{};
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const topo::Shape s = e.current();
    if (!watertight(s)) allWatertight = false;
    volSum += volumeOf(s);
    const Box wb = worldBox(s);
    // Discriminate the two members by their size (A is 10-cube, B is 6-cube).
    if (wb.hi[0] - wb.lo[0] > 8.0) boxA = wb; else boxB_ = wb;
    ++solids;
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 216.0)) < 1e-6);  // exact (planar)

  // Box A at origin; box B translated to (30,5,0)..(36,11,6).
  CC_CHECK(std::fabs(boxA.lo[0] - 0.0) < 1e-6 && std::fabs(boxA.hi[0] - 10.0) < 1e-6);
  CC_CHECK(std::fabs(boxB_.lo[0] - 30.0) < 1e-6 && std::fabs(boxB_.hi[0] - 36.0) < 1e-6);
  CC_CHECK(std::fabs(boxB_.lo[1] - 5.0) < 1e-6 && std::fabs(boxB_.hi[1] - 11.0) < 1e-6);
  CC_CHECK(std::fabs(boxB_.lo[2] - 0.0) < 1e-6 && std::fabs(boxB_.hi[2] - 6.0) < 1e-6);
}

// ── ASSEMBLY — a ROTATED placement composes exactly (rigid, det≈+1) ────────────
// Same wiring, but the TO frame is rotated 90° about +Z (frame X = +Y, so
// Y = Z×X = −X) and translated to (5,5,0). A unit box [0,1]³ placed by this frame
// lands at x_world = 5 − y_local ∈ [4,5], y_world = 5 + x_local ∈ [5,6], z ∈ [0,1].
CC_TEST(assembly_rotated_placement_composes) {
  const double p[] = {0, 0, 1, 0, 1, 1, 0, 1};
  const std::string sa = ex::writeStepString(cst::build_prism(p, 4, 1.0), "A");
  const std::string sb = ex::writeStepString(cst::build_prism(p, 4, 1.0), "B");
  const std::string bodyB = renumberedDataBody(sb, 100000);
  const long brepB = firstBrepId(bodyB);

  std::string asm_;
  asm_ += "#900001 = CARTESIAN_POINT('',(0.,0.,0.));\n";
  asm_ += "#900002 = DIRECTION('',(0.,0.,1.));\n";
  asm_ += "#900003 = DIRECTION('',(1.,0.,0.));\n";
  asm_ += "#900004 = AXIS2_PLACEMENT_3D('',#900001,#900002,#900003);\n";  // FROM identity
  asm_ += "#900005 = CARTESIAN_POINT('',(5.,5.,0.));\n";
  asm_ += "#900014 = DIRECTION('',(0.,1.,0.));\n";  // TO X axis = +Y (90° about Z)
  asm_ += "#900006 = AXIS2_PLACEMENT_3D('',#900005,#900002,#900014);\n";  // TO rotated
  asm_ += "#900007 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900006);\n";
  asm_ += "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(brepB) + "),#900020);\n";
  asm_ += "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n";
  asm_ += "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900009) "
          "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) "
          "SHAPE_REPRESENTATION_RELATIONSHIP() );\n";
  asm_ += "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n";
  asm_ += "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n";
  asm_ += "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900016,#900017,$);\n";

  std::string merged = sa;
  const std::size_t insert = merged.find('\n', merged.find("DATA;")) + 1;
  merged.insert(insert, bodyB + asm_);

  const topo::Shape shape = ex::readStepString(merged);
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;

  bool found = false;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const Box wb = worldBox(e.current());
    // The ROTATED box lands at x∈[4,5], y∈[5,6]; the root stays at [0,1]³.
    if (wb.hi[1] > 4.0) {
      found = true;
      CC_CHECK(std::fabs(wb.lo[0] - 4.0) < 1e-6 && std::fabs(wb.hi[0] - 5.0) < 1e-6);
      CC_CHECK(std::fabs(wb.lo[1] - 5.0) < 1e-6 && std::fabs(wb.hi[1] - 6.0) < 1e-6);
      CC_CHECK(std::fabs(wb.lo[2] - 0.0) < 1e-6 && std::fabs(wb.hi[2] - 1.0) < 1e-6);
    }
  }
  CC_CHECK(found);
}

// ── NESTED (M4-tail) — a 2-level assembly composes W = T₁ ∘ T₂ (HOST ANALYTIC) ──
// A leaf (box B, a 6-cube) sits in a child shape-representation placed INTO a
// sub-assembly SR by T₂ (a 90° rotation about +Z then translate (10,0,0)); the
// sub-assembly SR is in turn placed INTO the root SR by T₁ (translate (0,40,0)). Box A
// (the 10-cube root) stays at the origin. The reader must compose the FULL chain so the
// leaf lands at W = T₁ ∘ T₂ (not T₂-only — the latent nested mis-placement) — the leaf's
// world box is [4,10]×[40,46]×[0,6], centroid (7,43,3). GATE (a): this is verified against
// an INDEPENDENT math::Transform composition (T₁.composedWith(T₂)) with NO OCCT.
namespace {

namespace nm = cybercad::native::math;

struct TwoBox { std::string sa; std::string bodyB; long brepB; };
TwoBox twoBox() {
  const double pa[] = {0, 0, 10, 0, 10, 10, 0, 10};  // 10-cube root (vol 1000)
  const double pb[] = {0, 0, 6, 0, 6, 6, 0, 6};       // 6-cube leaf  (vol 216)
  TwoBox t;
  t.sa = ex::writeStepString(cst::build_prism(pa, 4, 10.0), "A");
  const std::string sb = ex::writeStepString(cst::build_prism(pb, 4, 6.0), "B");
  t.bodyB = renumberedDataBody(sb, 100000);
  t.brepB = firstBrepId(t.bodyB);
  return t;
}
std::string mergeAsm(const TwoBox& t, const std::string& asm_) {
  std::string merged = t.sa;
  const std::size_t insert = merged.find('\n', merged.find("DATA;")) + 1;
  merged.insert(insert, t.bodyB + asm_);
  return merged;
}

// The three shape-representations + the two frame-pair transforms shared by the nested
// fixtures. `leaf` lists box B's brep; `sub` and `root` are empty assembly reps. T₂ =
// R90z + (10,0,0) via the frame pair (#900004 identity FROM, #900006 TO); T₁ = (0,40,0)
// via (#900004 FROM, #900106 TO). The leaf-into-`sub` and `sub`-into-`root` CDSRs are
// authored per-test so the decline variants (cycle / ambiguous / dangling / shear) can
// re-route them.
std::string nestedReps(long brepB) {
  return "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(brepB) + "),#900020);\n"
         "#900030 = SHAPE_REPRESENTATION('',(),#900020);\n"
         "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n"
         "#900020 = GEOMETRIC_REPRESENTATION_CONTEXT(3);\n"
         "#900001 = CARTESIAN_POINT('',(0.,0.,0.));\n"
         "#900002 = DIRECTION('',(0.,0.,1.));\n"
         "#900003 = DIRECTION('',(1.,0.,0.));\n"
         "#900004 = AXIS2_PLACEMENT_3D('',#900001,#900002,#900003);\n"   // identity FROM
         "#900005 = CARTESIAN_POINT('',(10.,0.,0.));\n"
         "#900014 = DIRECTION('',(0.,1.,0.));\n"                          // TO X = +Y (R90z)
         "#900006 = AXIS2_PLACEMENT_3D('',#900005,#900002,#900014);\n"   // T2 TO
         "#900007 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900006);\n"  // T2
         "#900105 = CARTESIAN_POINT('',(0.,40.,0.));\n"
         "#900106 = AXIS2_PLACEMENT_3D('',#900105,#900002,#900003);\n"   // T1 TO (no rot)
         "#900107 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900106);\n";  // T1
}
// The two placing CDSRs of the well-formed nested tree: leaf(#900008) → sub(#900030) by
// T2(#900007); sub(#900030) → root(#900009) by T1(#900107).
std::string nestedCdsrs() {
  return "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900030) "
         "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) "
         "SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
         "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n"
         "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n"
         "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900008,#900030,$);\n"
         "#900110 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900009) "
         "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900107) "
         "SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
         "#900111 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900110,#900112);\n"
         "#900112 = PRODUCT_DEFINITION_SHAPE('','',#900113);\n"
         "#900113 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900009,$);\n";
}

// The world box of the nested leaf (box B), discriminated from the 10-cube root by its
// y-placement (the leaf is translated to y ≥ 40; the root stays at y ∈ [0,10]).
Box nestedLeafBox(const topo::Shape& shape) {
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const Box b = worldBox(e.current());
    if (b.lo[1] > 20.0) return b;
  }
  return Box{{0, 0, 0}, {0, 0, 0}};
}

}  // namespace

CC_TEST(nested_two_level_assembly_composes_chain) {
  const TwoBox t = twoBox();
  const topo::Shape shape = ex::readStepString(mergeAsm(t, nestedReps(t.brepB) + nestedCdsrs()));
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    ++solids;
    if (!watertight(e.current())) allWatertight = false;
    volSum += volumeOf(e.current());
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 216.0)) < 1e-6);  // both members, correct volumes

  // INDEPENDENT composition (no OCCT, no reader graph walk): T₂ = R90z + (10,0,0),
  // T₁ = (0,40,0); W = T₁ ∘ T₂ places the local centroid (3,3,3) at (7,43,3).
  const nm::Transform T2{nm::Mat3{0, -1, 0, 1, 0, 0, 0, 0, 1}, nm::Vec3{10, 0, 0}};
  const nm::Transform T1{nm::Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}, nm::Vec3{0, 40, 0}};
  const nm::Point3 wc = T1.composedWith(T2).applyToPoint(nm::Point3{3, 3, 3});
  CC_CHECK(std::fabs(wc.x - 7.0) < 1e-9 && std::fabs(wc.y - 43.0) < 1e-9 &&
           std::fabs(wc.z - 3.0) < 1e-9);

  // The reader's placed leaf box must equal W·(local box): [4,10]×[40,46]×[0,6], centre
  // (7,43,3) = W·centroid_local. (T₂-only would give y∈[0,6]; T₁-only x∈[0,6] — both
  // rejected, so this pins the FULL chain composition and its order.)
  const Box b = nestedLeafBox(shape);
  CC_CHECK(std::fabs(b.lo[0] - 4.0) < 1e-6 && std::fabs(b.hi[0] - 10.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[1] - 40.0) < 1e-6 && std::fabs(b.hi[1] - 46.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[2] - 0.0) < 1e-6 && std::fabs(b.hi[2] - 6.0) < 1e-6);
  const double cx = 0.5 * (b.lo[0] + b.hi[0]), cy = 0.5 * (b.lo[1] + b.hi[1]),
               cz = 0.5 * (b.lo[2] + b.hi[2]);
  CC_CHECK(std::fabs(cx - wc.x) < 1e-6 && std::fabs(cy - wc.y) < 1e-6 &&
           std::fabs(cz - wc.z) < 1e-6);
}

// ── NESTED (DECLINE) — a CYCLE in the parent-edge forest → NULL → OCCT ──────────
// leaf(#900008) → sub(#900030) by T₂, and sub(#900030) → leaf(#900008) by T₁: the chain
// walk revisits #900008 (visited-set hit) and DECLINES rather than looping forever.
CC_TEST(nested_cyclic_graph_declines) {
  const TwoBox t = twoBox();
  const std::string cyc =
      "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900030) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n"
      "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n"
      "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900008,#900030,$);\n"
      "#900110 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900008) "  // back-edge → cycle
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900107) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900111 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900110,#900112);\n"
      "#900112 = PRODUCT_DEFINITION_SHAPE('','',#900113);\n"
      "#900113 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900008,$);\n";
  CC_CHECK(ex::readStepString(mergeAsm(t, nestedReps(t.brepB) + cyc)).isNull());
}

// ── NESTED (DECLINE) — an AMBIGUOUS child (two distinct parents) → NULL → OCCT ──
// leaf(#900008) is placed into BOTH sub(#900030) and an alternate parent(#900009) by two
// CDSRs: a shared sub-assembly instanced twice needs per-instance world transforms this
// slice does not model → parentEdges declines (childSr with two distinct parents).
CC_TEST(nested_ambiguous_two_parent_declines) {
  const TwoBox t = twoBox();
  const std::string amb =
      "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900030) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n"
      "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n"
      "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900008,#900030,$);\n"
      "#900110 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900009) "  // second parent
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900107) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900111 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900110,#900112);\n"
      "#900112 = PRODUCT_DEFINITION_SHAPE('','',#900113);\n"
      "#900113 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900008,#900009,$);\n";
  CC_CHECK(ex::readStepString(mergeAsm(t, nestedReps(t.brepB) + amb)).isNull());
}

// ── NESTED (DECLINE) — a DANGLING transform reference → NULL → OCCT ─────────────
// The sub-assembly level's transform_operator references a non-existent id (#900777): the
// chain walk cannot resolve that level's placement and DECLINES rather than dropping it.
CC_TEST(nested_dangling_transform_declines) {
  const TwoBox t = twoBox();
  const std::string dangling =
      "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900030) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n"
      "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n"
      "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900008,#900030,$);\n"
      "#900110 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900009) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900777) "  // dangling operator id
      "SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900111 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900110,#900112);\n"
      "#900112 = PRODUCT_DEFINITION_SHAPE('','',#900113);\n"
      "#900113 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900009,$);\n";
  CC_CHECK(ex::readStepString(mergeAsm(t, nestedReps(t.brepB) + dangling)).isNull());
}

// ── NESTED (DECLINE) — a NON-CONFORMAL (shear) ancestor level → NULL → OCCT ─────
// The OUTER level places the sub-assembly into the root by a CARTESIAN_TRANSFORMATION_-
// OPERATOR_3D with a NON-ORTHOGONAL triad (axis2 = (1,1,0)) — a shear, MᵀM ≠ k²·I. The
// per-level conformality gate in the chain walk rejects it (only rigid / uniform-scale /
// mirror are applied), so the whole file DECLINES → OCCT even though the inner T₂ is rigid.
CC_TEST(nested_non_conformal_ancestor_declines) {
  const TwoBox t = twoBox();
  const std::string shear =
      "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900030) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n"
      "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n"
      "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900008,#900030,$);\n"
      "#900201 = DIRECTION('',(1.,0.,0.));\n#900202 = DIRECTION('',(1.,1.,0.));\n"  // shear
      "#900203 = DIRECTION('',(0.,0.,1.));\n#900204 = CARTESIAN_POINT('',(0.,40.,0.));\n"
      "#900207 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#900201,#900202,#900204,1.,#900203);\n"
      "#900110 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900009) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900207) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#900111 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900110,#900112);\n"
      "#900112 = PRODUCT_DEFINITION_SHAPE('','',#900113);\n"
      "#900113 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900009,$);\n";
  CC_CHECK(ex::readStepString(mergeAsm(t, nestedReps(t.brepB) + shear)).isNull());
}

// ── DECLINE: a non-composable assembly structure (Form B) → NULL ───────────────
// A MAPPED_ITEM / REPRESENTATION_MAP placement (Form B) is out of this slice: the
// reader must decline the WHOLE file rather than place the sub-solid at a wrong
// (identity) location. This is the honest decline pin (was a lone NAUO before the
// Form-A parse landed; a lone NAUO now has no placement to apply and would still
// decline, but Form B is the meaningful uncomposable structure to pin).
CC_TEST(decline_form_b_mapped_item_returns_null) {
  std::string step = ex::writeStepString(box10(), "box");
  const std::size_t insert = step.find('\n', step.find("DATA;")) + 1;
  step.insert(insert,
              "#98001 = REPRESENTATION_MAP(#98002,#98003);\n"
              "#98004 = MAPPED_ITEM('',#98001,#98005);\n");
  CC_CHECK(ex::readStepString(step).isNull());
}

// ── DECLINE: an assembly with NO composable transform → NULL ───────────────────
// A lone NEXT_ASSEMBLY_USAGE_OCCURRENCE (a transform tree with no
// CONTEXT_DEPENDENT_SHAPE_REPRESENTATION / IDT) has no placement to apply:
// placedCount==0 → honest NULL, never a silent flat import at a fabricated
// identity location. (A genuinely scaled placement cannot be authored through an
// AXIS2_PLACEMENT_3D pair — the axes are normalized — so the isRigid det≈+1 /
// orthonormal guard in itemDefinedTransform is a defensive gate on any future
// non-AXIS transform source.)
CC_TEST(decline_assembly_without_transform_returns_null) {
  std::string step = ex::writeStepString(box10(), "box");
  const std::size_t insert = step.find('\n', step.find("DATA;")) + 1;
  step.insert(insert, "#98001 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',#98002,#98003,$);\n");
  CC_CHECK(ex::readStepString(step).isNull());
}

// ── AP214 / AP242 FILE_SCHEMA headers are accepted (schema-independent) ─────────
// The reader enters at DATA; and never gates on FILE_SCHEMA, so a native solid whose
// header is rewritten to AP214 (AUTOMOTIVE_DESIGN) or AP242 imports identically. This
// pins that acceptance is genuinely schema-independent (confirmed, not newly added).
CC_TEST(accepts_ap214_and_ap242_file_schema) {
  const std::string base = ex::writeStepString(box10(), "box");
  const double vRef = volumeOf(ex::readStepString(base));
  CC_CHECK(vRef > 0.0);
  auto rewriteSchema = [&](const char* schema) {
    const std::size_t s = base.find("FILE_SCHEMA");
    const std::size_t e = base.find(';', s);
    std::string out = base;
    out.replace(s, e - s, std::string("FILE_SCHEMA(('") + schema + "'))");
    return out;
  };
  const std::string ap214 = rewriteSchema("AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }");
  const std::string ap242 =
      rewriteSchema("AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF { 1 0 10303 442 1 1 4 }");
  const topo::Shape s214 = ex::readStepString(ap214);
  const topo::Shape s242 = ex::readStepString(ap242);
  CC_CHECK(!s214.isNull() && !s242.isNull());
  CC_CHECK(std::fabs(volumeOf(s214) - vRef) < 1e-9);
  CC_CHECK(std::fabs(volumeOf(s242) - vRef) < 1e-9);
}

namespace {

// Author a two-box assembly (A=10-cube root, B=6-cube component) where B's placement
// is carried by the `operatorRecords` — a CARTESIAN_TRANSFORMATION_OPERATOR_3D declared
// as #900007 (the REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION transform_operator).
// This is the STEP entity a foreign system emits for a SCALED / MIRRORED instance (an
// AXIS2_PLACEMENT_3D frame pair cannot carry a scale or reflection). The wiring is the
// same brep-reaching CDSR the rigid assembly test uses.
std::string ctoAssembly(const std::string& operatorRecords) {
  const double pa[] = {0, 0, 10, 0, 10, 10, 0, 10};
  const double pb[] = {0, 0, 6, 0, 6, 6, 0, 6};
  const std::string sa = ex::writeStepString(cst::build_prism(pa, 4, 10.0), "A");
  const std::string sb = ex::writeStepString(cst::build_prism(pb, 4, 6.0), "B");
  const std::string bodyB = renumberedDataBody(sb, 100000);
  const long brepB = firstBrepId(bodyB);
  std::string asm_;
  asm_ += "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(brepB) + "),#900020);\n";
  asm_ += "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n";
  asm_ += "#900010 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900009) "
          "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900007) "
          "SHAPE_REPRESENTATION_RELATIONSHIP() );\n";
  asm_ += "#900011 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900010,#900012);\n";
  asm_ += "#900012 = PRODUCT_DEFINITION_SHAPE('','',#900013);\n";
  asm_ += "#900013 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900014,#900015,$);\n";
  asm_ += "#900020 = GEOMETRIC_REPRESENTATION_CONTEXT(3);\n";
  asm_ += operatorRecords;  // must declare #900007
  std::string merged = sa;
  const std::size_t insert = merged.find('\n', merged.find("DATA;")) + 1;
  merged.insert(insert, bodyB + asm_);
  return merged;
}

// The world box of the placed component solid, discriminated from the root by its
// PLACEMENT (every fixture translates the component to x ≥ 30; the 10-cube root stays
// at x ∈ [0,10]). Returns identity-marker box if absent.
Box componentBox(const topo::Shape& shape) {
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const Box b = worldBox(e.current());
    if (b.lo[0] > 15.0) return b;  // the translated component (root is at x ∈ [0,10])
  }
  return Box{{0, 0, 0}, {0, 0, 0}};
}

}  // namespace

// ── T1 (SCALE) — a UNIFORM-SCALE component transform scales the solid by k³ ─────
// The component's placement is a CARTESIAN_TRANSFORMATION_OPERATOR_3D with scale=2 and
// an orthonormal axis triad, translated to (30,5,0). The native reader must APPLY the
// scale: the 6-cube (vol 216) imports at 2× → vol 216·2³ = 1728, bbox 12-cube at
// [30,5,0]..[42,17,12]; the 10-cube root stays vol 1000. Total 2728. Both solids stay
// watertight (a uniform scale is conformal).
CC_TEST(scaled_assembly_component_scales_by_k_cubed) {
  const std::string step = ctoAssembly(
      "#900001 = DIRECTION('',(1.,0.,0.));\n#900002 = DIRECTION('',(0.,1.,0.));\n"
      "#900003 = DIRECTION('',(0.,0.,1.));\n#900004 = CARTESIAN_POINT('',(30.,5.,0.));\n"
      "#900007 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#900001,#900002,#900004,2.,#900003);\n");
  const topo::Shape shape = ex::readStepString(step);
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    ++solids;
    if (!watertight(e.current())) allWatertight = false;
    volSum += volumeOf(e.current());
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 1728.0)) < 1e-6);  // k³ = 8, 216·8 = 1728

  const Box b = componentBox(shape);
  CC_CHECK(std::fabs(b.lo[0] - 30.0) < 1e-6 && std::fabs(b.hi[0] - 42.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[1] - 5.0) < 1e-6 && std::fabs(b.hi[1] - 17.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[2] - 0.0) < 1e-6 && std::fabs(b.hi[2] - 12.0) < 1e-6);
}

// ── T1 (MIRROR) — a REFLECTED component is watertight with the right (positive) volume
// The component's placement is a CARTESIAN_TRANSFORMATION_OPERATOR_3D whose axis triad
// is LEFT-handed (axis3 = (0,0,-1), det < 0 → a reflection about z), translated x+30.
// A reflection flips cross(∂u,∂v), so without orientation compensation the solid would
// mesh inside-out (negative enclosed volume, not watertight). The reader complements the
// mirrored component's faces, so the 6-cube imports WATERTIGHT with the correct POSITIVE
// volume 216 and a reflected bbox: local [0,6]³ with z→−z gives z∈[−6,0], x∈[30,36].
CC_TEST(mirrored_assembly_component_watertight_reflected) {
  const std::string step = ctoAssembly(
      "#900001 = DIRECTION('',(1.,0.,0.));\n#900002 = DIRECTION('',(0.,1.,0.));\n"
      "#900003 = DIRECTION('',(0.,0.,-1.));\n#900004 = CARTESIAN_POINT('',(30.,0.,0.));\n"
      "#900007 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#900001,#900002,#900004,1.,#900003);\n");
  const topo::Shape shape = ex::readStepString(step);
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    ++solids;
    if (!watertight(e.current())) allWatertight = false;  // mirror must stay watertight
    volSum += volumeOf(e.current());                       // |vol| — outward, positive
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 216.0)) < 1e-6);  // positive 216, not −216

  const Box b = componentBox(shape);
  CC_CHECK(std::fabs(b.lo[0] - 30.0) < 1e-6 && std::fabs(b.hi[0] - 36.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[1] - 0.0) < 1e-6 && std::fabs(b.hi[1] - 6.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[2] + 6.0) < 1e-6 && std::fabs(b.hi[2] - 0.0) < 1e-6);  // z∈[−6,0]
}

// ── T1 (DECLINE) — a NON-UNIFORM / SHEAR component transform → NULL → OCCT ──────
// The component's CARTESIAN_TRANSFORMATION_OPERATOR_3D uses a NON-ORTHOGONAL axis triad
// (axis2 = (1,1,0)), so its linear part is a shear — MᵀM ≠ k²·I. The classifier declines
// (non-conformal is out of the honest slice); the whole file returns NULL → OCCT. This
// is the honest boundary: only rigid / uniform-scale / mirror are applied.
CC_TEST(decline_non_uniform_shear_assembly_returns_null) {
  const std::string step = ctoAssembly(
      "#900001 = DIRECTION('',(1.,0.,0.));\n#900002 = DIRECTION('',(1.,1.,0.));\n"
      "#900003 = DIRECTION('',(0.,0.,1.));\n#900004 = CARTESIAN_POINT('',(30.,0.,0.));\n"
      "#900007 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',#900001,#900002,#900004,1.,#900003);\n");
  CC_CHECK(ex::readStepString(step).isNull());
}

// ── T2 (AP242 PMI) — geometry imports; PMI / GD&T / annotation entities are SKIPPED ──
// An AP242 file carries semantic PMI, geometric-tolerance, datum, dimensional and
// draughting/annotation entities plus extra plane-/solid-angle unit contexts. NONE of it
// is geometry we import. Two cases must import the SOLID and drop the PMI:
//   (a) INERT PMI — annotation/GD&T entities the brep never references (unreferenced,
//       so they were already skipped);
//   (b) PMI carried by a REPRESENTATION_RELATIONSHIP graph (a DRAUGHTING_MODEL related to
//       the shape rep via REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION) — this used to
//       trip the assembly trigger and DECLINE the whole file; now that relationship does
//       not reach a MANIFOLD_SOLID_BREP, it is SKIPPED and the solid still imports.
CC_TEST(ap242_pmi_skipped_imports_solid) {
  const std::string base = ex::writeStepString(box10(), "box");
  const double vRef = volumeOf(ex::readStepString(base));
  CC_CHECK(vRef > 0.0);
  auto ap242 = [&](const std::string& pmi) {
    std::string out = base;
    const std::size_t s = out.find("FILE_SCHEMA");
    const std::size_t e = out.find(';', s);
    out.replace(s, e - s,
                "FILE_SCHEMA(('AP242_MANAGED_MODEL_BASED_3D_ENGINEERING_MIM_LF "
                "{ 1 0 10303 442 1 1 4 }'))");
    const std::size_t insert = out.find('\n', out.find("DATA;")) + 1;
    out.insert(insert, pmi);
    return out;
  };

  // (a) inert PMI vocabulary (angle units, draughting, datum, tolerance, dimension).
  const std::string inert =
      "#70001 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n"
      "#70002 = ( NAMED_UNIT(*) SOLID_ANGLE_UNIT() SI_UNIT($,.STERADIAN.) );\n"
      "#70010 = DRAUGHTING_MODEL('PMI',(#70011),#70020);\n"
      "#70011 = ANNOTATION_PLANE('',(#70012),#70013,$);\n"
      "#70012 = ANNOTATION_FILL_AREA_OCCURRENCE('',(#70014),$,$,$);\n"
      "#70013 = AXIS2_PLACEMENT_3D('',#70030,#70031,#70032);\n"
      "#70020 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
      "GLOBAL_UNIT_ASSIGNED_CONTEXT((#70001)) REPRESENTATION_CONTEXT('','') );\n"
      "#70040 = DATUM('A',$,#70041,.FRONT.);\n"
      "#70041 = PRODUCT_DEFINITION_SHAPE('','',#70042);\n"
      "#70050 = DIMENSIONAL_SIZE(#70051,'diameter');\n"
      "#70060 = GEOMETRIC_TOLERANCE('',$,#70061,#70062);\n"
      "#70070 = FLATNESS_TOLERANCE('',$,#70071,#70072);\n"
      "#70080 = PLACED_DATUM_TARGET_FEATURE('',$,$,$,#70081);\n"
      "#70030 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#70031 = DIRECTION('',(0.,0.,1.));\n#70032 = DIRECTION('',(1.,0.,0.));\n";
  const topo::Shape sa = ex::readStepString(ap242(inert));
  CC_CHECK(!sa.isNull());
  if (!sa.isNull()) {
    CC_CHECK(sa.type() == topo::ShapeType::Solid);
    CC_CHECK(std::fabs(volumeOf(sa) - vRef) < 1e-9);  // identical to the plain box
  }

  // (b) PMI linked via a representation-relationship graph (no brep reached → skipped).
  const std::string repRel =
      "#70090 = SHAPE_REPRESENTATION('',(),#70020);\n"
      "#70091 = DRAUGHTING_MODEL('PMI',(),#70020);\n"
      "#70092 = ( REPRESENTATION_RELATIONSHIP('','',#70091,#70090) "
      "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#70093) "
      "SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
      "#70093 = ITEM_DEFINED_TRANSFORMATION('','',#70094,#70095);\n"
      "#70094 = AXIS2_PLACEMENT_3D('',#70096,#70097,#70098);\n"
      "#70095 = AXIS2_PLACEMENT_3D('',#70096,#70097,#70098);\n"
      "#70096 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#70097 = DIRECTION('',(0.,0.,1.));\n#70098 = DIRECTION('',(1.,0.,0.));\n"
      "#70020 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
      "GLOBAL_UNIT_ASSIGNED_CONTEXT((#70001)) REPRESENTATION_CONTEXT('','') );\n"
      "#70001 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n";
  const topo::Shape sb = ex::readStepString(ap242(repRel));
  CC_CHECK(!sb.isNull());
  if (!sb.isNull()) {
    CC_CHECK(sb.type() == topo::ShapeType::Solid);
    CC_CHECK(std::fabs(volumeOf(sb) - vRef) < 1e-9);
  }
}

namespace {

// Wrap the FIRST `#id = keyword(` basis-curve declaration in a TRIMMED_CURVE: rename the
// basis record to a fresh #id and insert `#origId = TRIMMED_CURVE('',#newId,<trims>)` at
// the old id, so every EDGE_CURVE that referenced the basis now reaches it THROUGH a
// TRIMMED_CURVE (the reader must unwrap it). Returns the rewritten file (unchanged if the
// keyword is absent).
std::string wrapBasisInTrimmedCurve(std::string step, const std::string& keyword,
                                    const std::string& trims, long idOffset = 700000) {
  const std::string needle = "= " + keyword + "(";
  const std::size_t k = step.find(needle);
  if (k == std::string::npos) return step;
  const std::size_t hash = step.rfind('#', k);
  std::size_t j = hash + 1; std::string num;
  while (j < step.size() && std::isdigit(static_cast<unsigned char>(step[j]))) num += step[j++];
  const long oldId = std::stol(num);
  const long newId = oldId + idOffset;
  step.replace(hash, j - hash, "#" + std::to_string(newId));  // move the basis to newId
  const std::size_t ins = step.find('\n', step.find("DATA;")) + 1;
  step.insert(ins, "#" + std::to_string(oldId) + " = TRIMMED_CURVE('',#" +
                       std::to_string(newId) + "," + trims + ");\n");
  return step;
}

// Rewrite every `<surfaceKeyword>('',...)` record into a SURFACE_OF_REVOLUTION of an
// injected generatrix about the +Y axis through the origin (the revolution axis of the
// cylinder()/cone() fixtures). `profileRecords` declare the generatrix; `profileRef` is
// its #id. The reader's revolvedLine arm must reduce the revolution back to the EXACT
// native analytic surface the fixture originally carried. Choosing a non-line / off-axis
// generatrix drives the honest-decline path.
std::string revolveSurfaces(std::string step, const std::string& surfaceKeyword,
                            const std::string& profileRecords, const std::string& profileRef) {
  const std::size_t ins = step.find('\n', step.find("DATA;")) + 1;
  step.insert(ins, profileRecords +
                       "#800005 = CARTESIAN_POINT('',(0.,0.,0.));\n"
                       "#800006 = DIRECTION('',(0.,1.,0.));\n"
                       "#800007 = AXIS1_PLACEMENT('',#800005,#800006);\n");
  const std::string what = surfaceKeyword + "(";
  for (std::size_t k = step.find(what); k != std::string::npos; k = step.find(what, k)) {
    const std::size_t close = step.find(')', k);
    step.replace(k, close - k, "SURFACE_OF_REVOLUTION('',#" + profileRef + ",#800007");
    k += 1;
  }
  return step;
}

// Like revolveSurfaces but rewrites ONLY the n-th `<surfaceKeyword>('',...)` record —
// used for the ⟂-line→PLANE case, where a cylinder has SIX identical PLANE cap patches
// at TWO heights: rewriting them all would move the top cap onto the bottom (self-verify
// would fail). Rewriting a SINGLE bottom-cap patch turns it into a revolution-plane that
// must reconstruct byte-identically to its native-PLANE neighbours (same y=0 plane) so
// the disk stays watertight — the additive, geometry-preserving proof of the reduction.
std::string revolveNthSurface(std::string step, const std::string& surfaceKeyword,
                              const std::string& profileRecords, const std::string& profileRef,
                              int nth) {
  const std::size_t ins = step.find('\n', step.find("DATA;")) + 1;
  step.insert(ins, profileRecords +
                       "#800005 = CARTESIAN_POINT('',(0.,0.,0.));\n"
                       "#800006 = DIRECTION('',(0.,1.,0.));\n"
                       "#800007 = AXIS1_PLACEMENT('',#800005,#800006);\n");
  const std::string what = surfaceKeyword + "(";
  std::size_t k = step.find(what);
  for (int i = 0; i < nth && k != std::string::npos; ++i) k = step.find(what, k + 1);
  if (k == std::string::npos) return step;
  const std::size_t close = step.find(')', k);
  step.replace(k, close - k, "SURFACE_OF_REVOLUTION('',#" + profileRef + ",#800007");
  return step;
}

// A native cone (base radius 5 at y=0, apex at (0,20,0)): revolve a right triangle about
// +Y — the writer emits CONICAL_SURFACE walls, exercising the reader's oblique (cone)
// revolvedLine reduction when re-authored as a SURFACE_OF_REVOLUTION.
topo::Shape cone20() {
  const double p[] = {0, 0, 5, 0, 0, 20};
  return cst::build_revolution(p, 3, cst::RevolveAxis{0.0, 0.0, 0.0, 1.0}, 6.28318530717958647692);
}

// A native sphere (radius 6, centred at the origin ON the +Y axis): revolve a full
// meridian great-semicircle (south pole (0,-6) → north pole (0,6)) about +Y. The writer
// emits SPHERICAL_SURFACE lune faces; re-authoring them as SURFACE_OF_REVOLUTION of the
// on-axis meridian CIRCLE exercises the reader's revolvedCircle→Sphere reduction.
topo::Shape sphere6() {
  std::vector<cst::ProfileSegment> segs(1);
  segs[0].kind = 1; segs[0].cx = 0; segs[0].cy = 0; segs[0].r = 6.0;
  segs[0].x0 = 0.0; segs[0].y0 = -6.0; segs[0].x1 = 0.0; segs[0].y1 = 6.0;
  segs[0].a0 = -1.57079632679489662; segs[0].a1 = 1.57079632679489662;
  return cst::build_revolution_profile(segs, cst::RevolveAxis{0.0, 0.0, 0.0, 1.0},
                                       6.28318530717958647692);
}

// Wrap the FIRST `<surfaceKeyword>('',...)` record in a RECTANGULAR_TRIMMED_SURFACE over
// the (u1,u2,v1,v2) box and redirect the ADVANCED_FACE that referenced that surface to
// the wrapper — the way a foreign AP203 writer presents an analytic face re-parametrised
// to a (u,v) box. The reader must UNWRAP the rect-trim back to the basis surface and keep
// the face's EDGE_LOOP as the authoritative trim. `basisOverrideRef`, when non-empty,
// makes the rect-trim reference THAT id instead of the found surface (used to point the
// wrapper at an UNSUPPORTED basis for the decline test).
std::string wrapSurfaceInRectTrim(std::string step, const std::string& surfaceKeyword,
                                  const std::string& box, const std::string& extraRecords = "",
                                  const std::string& basisOverrideRef = "") {
  const std::string what = surfaceKeyword + "('',";
  const std::size_t k = step.find(what);
  if (k == std::string::npos) return step;
  const std::size_t h = step.rfind('#', k);
  std::size_t j = h + 1;
  std::string surfId;
  while (j < step.size() && std::isdigit(static_cast<unsigned char>(step[j]))) surfId += step[j++];
  if (surfId.empty()) return step;
  const std::string basisRef = basisOverrideRef.empty() ? ("#" + surfId) : basisOverrideRef;
  const std::size_t ins = step.find('\n', step.find("DATA;")) + 1;
  step.insert(ins, extraRecords + "#700000 = RECTANGULAR_TRIMMED_SURFACE(''," + basisRef +
                       "," + box + ",.T.,.T.);\n");
  // Redirect the ADVANCED_FACE surface ref (the 3rd arg, following the bound list's
  // closing ')'): `ADVANCED_FACE('',(#loop),#surfId,.T.)` → `...,#700000,.T.)`.
  const std::string faceRef = "),#" + surfId + ",";
  const std::size_t f = step.find(faceRef);
  if (f != std::string::npos) step.replace(f, faceRef.size(), "),#700000,");
  return step;
}

// The reconstructed analytic surface of the FIRST face of `solid` whose FaceSurface has
// the given kind, or nullptr. Used for the cylinder-basis closed-form corner check.
const topo::FaceSurface* firstSurfaceOfKind(const topo::Shape& solid,
                                            topo::FaceSurface::Kind kind) {
  for (topo::Explorer e(solid, topo::ShapeType::Face); e.more(); e.next()) {
    const auto sr = topo::surfaceOf(e.current());
    if (sr && sr->surface && sr->surface->kind == kind) return sr->surface;
  }
  return nullptr;
}

}  // namespace

// ── T1 (TRIMMED_CURVE, LINE basis) — accepted + unwrapped, box stays watertight ──
// A foreign file may wrap an edge's 3D geometry in a TRIMMED_CURVE. The reader must
// unwrap it to the basis LINE and bound the edge by its vertices exactly (a straight
// segment between two points is unique — the trims are redundant). The box round-trips
// watertight with EXACT volume, proving the keyword is accepted (it declined before).
CC_TEST(trimmed_curve_line_basis_imports_watertight) {
  const std::string base = ex::writeStepString(box10(), "box");
  const std::string wrapped = wrapBasisInTrimmedCurve(
      base, "LINE", "(PARAMETER_VALUE(0.0)),(PARAMETER_VALUE(1.0)),.T.,.PARAMETER.");
  CC_CHECK(wrapped.find("TRIMMED_CURVE") != std::string::npos);
  const topo::Shape s = ex::readStepString(wrapped);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  CC_CHECK(std::fabs(volumeOf(s) - 1000.0) < 1e-6);
}

// ── T1 (TRIMMED_CURVE, CIRCLE basis) — a wrapped rim arc unwraps, cylinder watertight ─
// A CIRCLE rim wrapped in a TRIMMED_CURVE must unwrap to the basis circle; the edge's
// vertices fix the arc endpoints (CCW convention), so the cylinder stays watertight with
// the analytic volume π·r²·h.
CC_TEST(trimmed_curve_circle_basis_imports_watertight) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string wrapped = wrapBasisInTrimmedCurve(
      base, "CIRCLE", "(PARAMETER_VALUE(0.0)),(PARAMETER_VALUE(6.2831853)),.T.,.PARAMETER.");
  CC_CHECK(wrapped.find("TRIMMED_CURVE") != std::string::npos);
  const topo::Shape s = ex::readStepString(wrapped);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  const double expected = 3.14159265358979323846 * 25.0 * 20.0;  // π·5²·20
  CC_CHECK(std::fabs(volumeOf(s) - expected) / expected < 5e-3);
}

// ── T1 (TRIMMED_CURVE, B_SPLINE basis) — trim bounds select the covered knot span ─────
// For a B-spline basis the endpoint vertices cannot recover the covered knot sub-domain,
// so the reader honors the PARAMETER_VALUE trims (clamped to the clamped knot span). WIDE
// trims clamp to the FULL span → the whole spline curve → the spline-wall prism round-trips
// watertight with EXACT volume, exercising the trim-cache B-spline arm.
CC_TEST(trimmed_curve_bspline_basis_full_span_watertight) {
  const topo::Shape orig = splineWallPrism();
  CC_CHECK(!orig.isNull());
  if (orig.isNull()) return;
  const std::string base = ex::writeStepString(orig, "spline");
  const std::string wrapped = wrapBasisInTrimmedCurve(
      base, "B_SPLINE_CURVE_WITH_KNOTS",
      "(PARAMETER_VALUE(-1000000.)),(PARAMETER_VALUE(1000000.)),.T.,.PARAMETER.");
  CC_CHECK(wrapped.find("TRIMMED_CURVE") != std::string::npos);
  const topo::Shape s = ex::readStepString(wrapped);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  CC_CHECK(std::fabs(volumeOf(s) - volumeOf(orig)) / volumeOf(orig) < 1e-9);
}

// ── T2 (SURFACE_OF_REVOLUTION → cylinder) — line ∥ axis reduces to a native Cylinder ──
// A straight generatrix parallel to the axis revolves to an EXACT cylinder. Rewriting the
// cylinder's CYLINDRICAL_SURFACE walls as SURFACE_OF_REVOLUTION(line, axis1) must import to
// the same watertight solid with the analytic volume — proving the revolvedLine analytic
// reduction (the keyword declined before).
CC_TEST(surface_of_revolution_line_parallel_maps_to_cylinder) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string rev = revolveSurfaces(
      base, "CYLINDRICAL_SURFACE",
      "#800001 = CARTESIAN_POINT('',(5.,0.,0.));\n"
      "#800002 = DIRECTION('',(0.,1.,0.));\n"
      "#800003 = VECTOR('',#800002,1.);\n"
      "#800004 = LINE('',#800001,#800003);\n",
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(rev.find("CYLINDRICAL_SURFACE") == std::string::npos);
  const topo::Shape s = ex::readStepString(rev);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  const double expected = 3.14159265358979323846 * 25.0 * 20.0;
  CC_CHECK(std::fabs(volumeOf(s) - expected) / expected < 5e-3);
}

// ── T2 (SURFACE_OF_REVOLUTION → cone) — an OBLIQUE line reduces to a native Cone ───────
// A generatrix that MEETS the axis at an angle revolves to an EXACT cone. Rewriting the
// cone's CONICAL_SURFACE walls as SURFACE_OF_REVOLUTION(line, axis1) must import to the
// same watertight solid with the analytic cone volume π·r²·h/3. The reader reconstructs
// the cone with origin on the axis / Z=+axis / signed semiAngle (byte-identical to the
// direct CONICAL_SURFACE convention), and the meridian-at-apex pcurve fix keeps the
// apex-touching wall faces welded — proving the oblique reduction (it declined before).
CC_TEST(surface_of_revolution_oblique_line_maps_to_cone) {
  const std::string base = ex::writeStepString(cone20(), "cone");
  const std::string rev = revolveSurfaces(
      base, "CONICAL_SURFACE",
      "#800001 = CARTESIAN_POINT('',(5.,0.,0.));\n"
      "#800002 = DIRECTION('',(-5.,20.,0.));\n"   // oblique — toward the apex (0,20,0)
      "#800003 = VECTOR('',#800002,1.);\n"
      "#800004 = LINE('',#800001,#800003);\n",
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(rev.find("CONICAL_SURFACE") == std::string::npos);
  const topo::Shape s = ex::readStepString(rev);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  const double expected = 3.14159265358979323846 * 25.0 * 20.0 / 3.0;  // π·5²·20/3
  CC_CHECK(std::fabs(volumeOf(s) - expected) / expected < 5e-3);
}

// ── T2 (SURFACE_OF_REVOLUTION → plane) — a ⟂ line reduces to a native Plane ────────────
// A generatrix PERPENDICULAR to the axis revolves to a flat annulus/disk PLANE. A cylinder
// has SIX identical PLANE cap patches at two heights; rewriting a SINGLE bottom-cap patch
// (y=0) as SURFACE_OF_REVOLUTION(⟂line, axis1) must reconstruct the SAME y=0 plane as its
// native-PLANE neighbours so the disk stays watertight with the unchanged cylinder volume.
CC_TEST(surface_of_revolution_perpendicular_line_maps_to_plane) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string rev = revolveNthSurface(
      base, "PLANE",
      "#800001 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#800002 = DIRECTION('',(1.,0.,0.));\n"      // ⟂ the +Y axis → a y=0 plane
      "#800003 = VECTOR('',#800002,1.);\n"
      "#800004 = LINE('',#800001,#800003);\n",
      "800004", /*nth=*/0);
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  const topo::Shape s = ex::readStepString(rev);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(watertight(s));
  const double expected = 3.14159265358979323846 * 25.0 * 20.0;  // π·5²·20 (unchanged)
  CC_CHECK(std::fabs(volumeOf(s) - expected) / expected < 5e-3);
}

// ── T2 (SURFACE_OF_REVOLUTION → sphere) — an on-axis meridian circle reduces to a Sphere ─
// A CIRCLE centred ON the axis, in a plane CONTAINING the axis, revolves to a SPHERE of the
// same radius. Rewriting a native sphere's SPHERICAL_SURFACE lunes as SURFACE_OF_REVOLUTION
// of the meridian circle must import to the SAME solid the direct SPHERICAL_SURFACE keyword
// produces — same non-null solid, same Sphere face count. (This checks Sphere-face-count
// parity against the native writer's multi-LUNE full-sphere B-rep; the end-to-end WATERTIGHT
// full sphere comes from the OCCT VERTEX_LOOP form — see
// spherical_surface_vertex_loop_full_sphere_imports_watertight below and the sim parity gate.
// Here we prove the revolvedCircle→Sphere REDUCTION: parity with the analytic-keyword import.)
CC_TEST(surface_of_revolution_on_axis_circle_maps_to_sphere) {
  const std::string base = ex::writeStepString(sphere6(), "sph");
  const topo::Shape direct = ex::readStepString(base);
  CC_CHECK(!direct.isNull());
  const std::string rev = revolveSurfaces(
      base, "SPHERICAL_SURFACE",
      "#800020 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#800021 = DIRECTION('',(0.,0.,1.));\n"       // circle-plane normal ⟂ the +Y axis
      "#800022 = DIRECTION('',(1.,0.,0.));\n"
      "#800023 = AXIS2_PLACEMENT_3D('',#800020,#800021,#800022);\n"
      "#800004 = CIRCLE('',#800023,6.);\n",          // centre ON axis, plane CONTAINS axis
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(rev.find("SPHERICAL_SURFACE") == std::string::npos);
  const topo::Shape s = ex::readStepString(rev);
  CC_CHECK(!s.isNull());                            // reduced (declined before this slice)
  if (s.isNull()) return;
  auto sphereFaces = [](const topo::Shape& sh) {
    int n = 0;
    for (topo::Explorer ex(sh, topo::ShapeType::Face); ex.more(); ex.next()) {
      const auto sr = topo::surfaceOf(ex.current());
      if (sr && sr->surface && sr->surface->kind == topo::FaceSurface::Kind::Sphere) ++n;
    }
    return n;
  };
  CC_CHECK(sphereFaces(s) == sphereFaces(direct));  // parity with the direct SPHERICAL import
  CC_CHECK(sphereFaces(s) > 0);
}

namespace {

// Author an OCCT-style FULL sphere solid: ONE ADVANCED_FACE whose bound is a
// VERTEX_LOOP (a single degenerate pole vertex, NO edges) — exactly how OCCT 7.x
// emits a whole sphere (no longitude-seam edge, no pole edges, just a bare periodic
// SPHERICAL_SURFACE). `surfaceRecs` declares the face surface; `surfRef` is its #id.
// The unit/product boilerplate is copied verbatim from the native writer (true mm) so
// the reader's unit-context gate accepts it.
std::string vertexLoopSolid(const std::string& surfaceRecs, const std::string& surfRef) {
  std::string s;
  s += "ISO-10303-21;\nHEADER;\n";
  s += "FILE_DESCRIPTION(('vertex-loop sphere'),'2;1');\n";
  s += "FILE_NAME('vl.step','',(''),(''),'t','t','');\n";
  s += "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n";
  s += surfaceRecs;                                    // must declare surfRef (the FaceSurface)
  s += "#40 = CARTESIAN_POINT('',(0.,0.,-6.));\n";     // the collapsed south-pole point
  s += "#41 = VERTEX_POINT('',#40);\n";
  s += "#42 = VERTEX_LOOP('',#41);\n";                 // single-vertex bound (no edges)
  s += "#43 = FACE_BOUND('',#42,.T.);\n";
  s += "#44 = ADVANCED_FACE('',(#43)," + surfRef + ",.T.);\n";
  s += "#45 = CLOSED_SHELL('',(#44));\n";
  s += "#46 = MANIFOLD_SOLID_BREP('vl',#45);\n";
  s += "#115 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n";
  s += "#116 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n";
  s += "#117 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n";
  s += "#118 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#115,'','');\n";
  s += "#119 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
       "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#118)) "
       "GLOBAL_UNIT_ASSIGNED_CONTEXT((#115,#116,#117)) REPRESENTATION_CONTEXT('','') );\n";
  s += "#120 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#46),#119);\n";
  s += "ENDSEC;\nEND-ISO-10303-21;\n";
  return s;
}

int sphereFaceCount(const topo::Shape& sh) {
  int n = 0;
  for (topo::Explorer e(sh, topo::ShapeType::Face); e.more(); e.next()) {
    const auto sr = topo::surfaceOf(e.current());
    if (sr && sr->surface && sr->surface->kind == topo::FaceSurface::Kind::Sphere) ++n;
  }
  return n;
}

int torusFaceCount(const topo::Shape& sh) {
  int n = 0;
  for (topo::Explorer e(sh, topo::ShapeType::Face); e.more(); e.next()) {
    const auto sr = topo::surfaceOf(e.current());
    if (sr && sr->surface && sr->surface->kind == topo::FaceSurface::Kind::Torus) ++n;
  }
  return n;
}

int bsplineFaceCount(const topo::Shape& sh) {
  int n = 0;
  for (topo::Explorer e(sh, topo::ShapeType::Face); e.more(); e.next()) {
    const auto sr = topo::surfaceOf(e.current());
    if (sr && sr->surface && sr->surface->kind == topo::FaceSurface::Kind::BSpline) ++n;
  }
  return n;
}

// A GENUINELY rational (non-unit weight) B-spline surface, delivered as the combined
// RATIONAL_B_SPLINE_SURFACE record, reproduces the EXACT closed-form sphere: the reader parses
// the split record, populates the weights, admits the bare-periodic face, and the M0 mesher
// closes it watertight with V = 4/3·πR³. Proven against an independent closed form, NO OCCT.
// (rationalSphereRecords is defined in the M4-RATIONAL fixtures block above.)
CC_TEST(foreign_rational_bspline_sphere_combined_record_imports_watertight) {
  const double R = 2.0;
  const std::string step = vertexLoopSolid(rationalSphereRecords(R), "#5");
  CC_CHECK(step.find("RATIONAL_B_SPLINE_SURFACE") != std::string::npos);
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());  // admitted via the NEW combined rational arm (declined before)
  if (s.isNull()) return;
  CC_CHECK(s.type() == topo::ShapeType::Solid);
  CC_CHECK(bsplineFaceCount(s) == 1);  // one bare-periodic rational B-spline sphere face
  CC_CHECK(watertight(s));             // u-seam welded, both axis poles collapsed
  const double vA = 4.0 / 3.0 * 3.14159265358979323846 * R * R * R;
  const double v = volumeOf(s);
  CC_CHECK(v > 0.0);
  CC_CHECK(std::fabs(v - vA) / vA < 1e-2);  // converges to the true sphere within deflection
}

// Author an OCCT-style FULL torus solid: ONE ADVANCED_FACE on a TOROIDAL_SURFACE
// (major R, minor r) whose single bound is a FULLY-SEAMED EDGE_LOOP — the equator
// v-seam circle (radius R+r) and the tube u-seam circle (radius r), EACH referenced
// forward AND reversed, exactly how OCCT emits a whole torus. The reader must map this
// to a native Kind::Torus BARE periodic face and mesh it watertight over its natural
// (u,v)∈[0,2π]² bounds. `major`/`minor` are formatted into the surface record.
std::string toroidalSolid(double major, double minor) {
  auto num = [](double x) {
    std::ostringstream o;
    o.precision(12);
    o << x;
    std::string t = o.str();
    if (t.find('.') == std::string::npos && t.find('e') == std::string::npos) t += ".";
    return t;
  };
  const std::string R = num(major), r = num(minor), Rr = num(major + minor);
  std::string s;
  s += "ISO-10303-21;\nHEADER;\n";
  s += "FILE_DESCRIPTION(('full torus'),'2;1');\n";
  s += "FILE_NAME('t.step','',(''),(''),'t','t','');\n";
  s += "FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n";
  s += "#31 = TOROIDAL_SURFACE('',#32," + R + "," + r + ");\n";
  s += "#32 = AXIS2_PLACEMENT_3D('',#33,#34,#35);\n";
  s += "#33 = CARTESIAN_POINT('',(0.,0.,0.));\n";
  s += "#34 = DIRECTION('',(0.,0.,1.));\n";
  s += "#35 = DIRECTION('',(1.,0.,0.));\n";
  s += "#23 = CARTESIAN_POINT('',(" + Rr + ",0.,0.));\n";
  s += "#22 = VERTEX_POINT('',#23);\n";
  // Equator (v-seam) circle radius R+r about the axis.
  s += "#26 = AXIS2_PLACEMENT_3D('',#33,#34,#35);\n";
  s += "#25 = CIRCLE('',#26," + Rr + ");\n";
  s += "#21 = EDGE_CURVE('',#22,#22,#25,.T.);\n";
  // Tube (u-seam) circle radius r in a meridian plane at the +X tube centre.
  s += "#54 = CARTESIAN_POINT('',(" + R + ",0.,0.));\n";
  s += "#55 = DIRECTION('',(0.,-1.,0.));\n";
  s += "#56 = DIRECTION('',(1.,0.,0.));\n";
  s += "#53 = AXIS2_PLACEMENT_3D('',#54,#55,#56);\n";
  s += "#52 = CIRCLE('',#53," + r + ");\n";
  s += "#50 = EDGE_CURVE('',#22,#22,#52,.T.);\n";
  // Each seam circle used forward AND reversed → a fully-seamed loop (no real trim).
  s += "#20 = ORIENTED_EDGE('',*,*,#21,.F.);\n";
  s += "#49 = ORIENTED_EDGE('',*,*,#50,.F.);\n";
  s += "#71 = ORIENTED_EDGE('',*,*,#21,.T.);\n";
  s += "#72 = ORIENTED_EDGE('',*,*,#50,.T.);\n";
  s += "#19 = EDGE_LOOP('',(#20,#49,#71,#72));\n";
  s += "#18 = FACE_BOUND('',#19,.T.);\n";
  s += "#17 = ADVANCED_FACE('',(#18),#31,.T.);\n";
  s += "#45 = CLOSED_SHELL('',(#17));\n";
  s += "#46 = MANIFOLD_SOLID_BREP('t',#45);\n";
  s += "#115 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n";
  s += "#116 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n";
  s += "#117 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n";
  s += "#118 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#115,'','');\n";
  s += "#119 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) "
       "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#118)) "
       "GLOBAL_UNIT_ASSIGNED_CONTEXT((#115,#116,#117)) REPRESENTATION_CONTEXT('','') );\n";
  s += "#120 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#46),#119);\n";
  s += "ENDSEC;\nEND-ISO-10303-21;\n";
  return s;
}

}  // namespace

// ── T (SPHERE, VERTEX_LOOP) — a full OCCT sphere face imports natively watertight ──────
// OCCT writes a whole sphere as ONE SPHERICAL_SURFACE ADVANCED_FACE bounded by a
// VERTEX_LOOP (one degenerate pole vertex, no edges). The reader maps that bare
// periodic surface to a native Sphere face with a null outer wire; the tessellator
// meshes its natural (u∈[0,2π], v∈[-π/2,π/2]) rectangle, welding the seam + both poles
// → a watertight Sphere solid. Volume = 4/3·π·6³ within the deflection bound.
CC_TEST(spherical_surface_vertex_loop_full_sphere_imports_watertight) {
  const std::string step = vertexLoopSolid(
      "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#2 = DIRECTION('',(0.,0.,1.));\n"
      "#3 = DIRECTION('',(1.,0.,0.));\n"
      "#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n"
      "#5 = SPHERICAL_SURFACE('',#4,6.);\n",
      "#5");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());                                 // now imports natively (declined before)
  if (s.isNull()) return;
  CC_CHECK(s.type() == topo::ShapeType::Solid);
  CC_CHECK(sphereFaceCount(s) == 1);                     // ONE bare periodic Sphere face
  CC_CHECK(watertight(s));                               // seam + both poles weld closed

  const double v = volumeOf(s);
  const double vAnalytic = 4.0 / 3.0 * 3.14159265358979323846 * 6.0 * 6.0 * 6.0;  // 904.7787
  CC_CHECK(v > 0.0);
  CC_CHECK(std::fabs(v - vAnalytic) / vAnalytic < 1e-2);  // converges to the true sphere

  const Box b = worldBox(s);
  for (int k = 0; k < 3; ++k) {
    CC_CHECK(b.hi[k] <= 6.0 + 1e-6 && b.lo[k] >= -6.0 - 1e-6);  // inside the R=6 sphere
    CC_CHECK(b.hi[k] > 5.9 && b.lo[k] < -5.9);                  // reaches ±R
  }
}

// ── T (SPHERE, VERTEX_LOOP via SURFACE_OF_REVOLUTION) — same bare-surface path ─────────
// The on-axis meridian-circle SURFACE_OF_REVOLUTION form (the sim runRevolvedSphere
// fixture) reduces to the SAME native Sphere and, with a VERTEX_LOOP bound, takes the
// same bare-surface route → a watertight sphere identical to the SPHERICAL keyword form.
CC_TEST(revolution_on_axis_circle_vertex_loop_imports_watertight) {
  const std::string step = vertexLoopSolid(
      "#20 = CARTESIAN_POINT('',(0.,0.,0.));\n"          // circle centre ON the axis
      "#21 = DIRECTION('',(0.,1.,0.));\n"                // circle-plane normal ⟂ +Z axis
      "#22 = DIRECTION('',(0.,0.,1.));\n"
      "#23 = AXIS2_PLACEMENT_3D('',#20,#21,#22);\n"
      "#24 = CIRCLE('',#23,6.);\n"                       // meridian circle, plane CONTAINS axis
      "#25 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#26 = DIRECTION('',(0.,0.,1.));\n"
      "#27 = AXIS1_PLACEMENT('',#25,#26);\n"             // +Z revolution axis
      "#5 = SURFACE_OF_REVOLUTION('',#24,#27);\n",
      "#5");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(sphereFaceCount(s) == 1);
  CC_CHECK(watertight(s));
  const double v = volumeOf(s);
  const double vAnalytic = 4.0 / 3.0 * 3.14159265358979323846 * 6.0 * 6.0 * 6.0;
  CC_CHECK(std::fabs(v - vAnalytic) / vAnalytic < 1e-2);
}

// ── T (GENERAL REVOLUTION → rational B-spline) — an ELLIPSE generatrix imports watertight ──
// OCCT emits an ELLIPSE (equatorial b=1, polar a=1.6 along the +Z revolution axis) revolved
// 360° as ONE SURFACE_OF_REVOLUTION(ELLIPSE) ADVANCED_FACE bounded by a VERTEX_LOOP — the
// same bare-periodic structure as a full sphere. The reader revolves the ellipse meridian
// into the EXACT rational tensor-product B-spline (u = the standard rational-quadratic full
// circle; v = the ellipse promoted to two rational-quadratic 90° arcs) and stores it as a
// native Kind::BSpline face WITH weights; the tessellator meshes its natural (u∈[0,2π],
// v∈[0,π]) bounds, welding the u-seam and collapsing both axis poles → a watertight spheroid.
// V = 4/3·π·b²·a = 6.70206, within the deflection bound.
CC_TEST(revolution_ellipse_generatrix_vertex_loop_imports_watertight) {
  const std::string step = vertexLoopSolid(
      "#20 = CARTESIAN_POINT('',(0.,0.,0.));\n"           // ellipse centre ON the axis
      "#21 = DIRECTION('',(0.,1.,0.));\n"                 // ellipse-plane normal ⟂ +Z axis
      "#22 = DIRECTION('',(0.,0.,1.));\n"                 // major-axis direction = +Z (polar)
      "#23 = AXIS2_PLACEMENT_3D('',#20,#21,#22);\n"
      "#24 = ELLIPSE('',#23,1.6,1.);\n"                   // semiAxis1=1.6 (∥ axis), semiAxis2=1.0
      "#25 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#26 = DIRECTION('',(0.,0.,1.));\n"
      "#27 = AXIS1_PLACEMENT('',#25,#26);\n"             // +Z revolution axis
      "#5 = SURFACE_OF_REVOLUTION('',#24,#27);\n",
      "#5");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());                                  // now imports natively (declined before)
  if (s.isNull()) return;
  CC_CHECK(s.type() == topo::ShapeType::Solid);
  CC_CHECK(bsplineFaceCount(s) == 1);                     // ONE bare periodic rational B-spline face
  CC_CHECK(watertight(s));                                // u-seam + both axis poles weld closed
  const double a = 1.6, b = 1.0;
  const double vAnalytic = 4.0 / 3.0 * 3.14159265358979323846 * b * b * a;  // 6.70206
  const double v = volumeOf(s);
  CC_CHECK(v > 0.0);
  CC_CHECK(std::fabs(v - vAnalytic) / vAnalytic < 1e-2);  // converges to the true spheroid
}

// ── T (GENERAL REVOLUTION → rational B-spline) — a non-rational B-SPLINE generatrix ────────
// A non-rational B_SPLINE_CURVE meridian from the north pole (0,0,2) through a bulge to the
// south pole (0,0,-2), revolved 360° about +Z, is emitted by OCCT with a VERTEX_LOOP bound.
// The reader revolves the profile directly (its own degree/knots as v, weights 1); the u=0
// column reproduces the profile curve EXACTLY, so the revolved rational B-spline meshes
// watertight over its natural bounds. Proves the arm generalises beyond the ellipse.
CC_TEST(revolution_bspline_generatrix_vertex_loop_imports_watertight) {
  const std::string step = vertexLoopSolid(
      "#20 = CARTESIAN_POINT('',(0.,0.,2.));\n"           // north pole ON the axis
      "#21 = CARTESIAN_POINT('',(1.2,0.,1.));\n"
      "#22 = CARTESIAN_POINT('',(1.5,0.,0.));\n"
      "#23 = CARTESIAN_POINT('',(1.2,0.,-1.));\n"
      "#24 = CARTESIAN_POINT('',(0.,0.,-2.));\n"          // south pole ON the axis
      "#28 = B_SPLINE_CURVE_WITH_KNOTS('',4,(#20,#21,#22,#23,#24),"
      ".UNSPECIFIED.,.F.,.F.,(5,5),(0.,1.),.PIECEWISE_BEZIER_KNOTS.);\n"
      "#25 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#26 = DIRECTION('',(0.,0.,1.));\n"
      "#27 = AXIS1_PLACEMENT('',#25,#26);\n"             // +Z revolution axis
      "#5 = SURFACE_OF_REVOLUTION('',#28,#27);\n",
      "#5");
  const topo::Shape s = ex::readStepString(step);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK(bsplineFaceCount(s) == 1);
  CC_CHECK(watertight(s));
  CC_CHECK(volumeOf(s) > 0.0);
}

// ── T (HONEST-OUT) — an ELLIPSE whose plane does NOT contain the axis DECLINES ─────────────
// A faithful spheroid meridian requires the ellipse plane to CONTAIN the revolution axis and
// one semi-axis to be PARALLEL to it (so the profile touches the axis at two poles). An
// ellipse whose plane is ⟂ the axis revolves to a non-spheroidal surface with no faithful
// rational-revolution reduction → NULL → OCCT (never a forced/broken solid).
CC_TEST(revolution_off_axis_ellipse_declines) {
  const std::string step = vertexLoopSolid(
      "#20 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#21 = DIRECTION('',(0.,0.,1.));\n"                 // ellipse-plane normal ∥ +Z axis (plane ⟂ axis)
      "#22 = DIRECTION('',(1.,0.,0.));\n"
      "#23 = AXIS2_PLACEMENT_3D('',#20,#21,#22);\n"
      "#24 = ELLIPSE('',#23,1.6,1.);\n"
      "#25 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#26 = DIRECTION('',(0.,0.,1.));\n"
      "#27 = AXIS1_PLACEMENT('',#25,#26);\n"
      "#5 = SURFACE_OF_REVOLUTION('',#24,#27);\n",
      "#5");
  CC_CHECK(ex::readStepString(step).isNull());
}

// ── T (HONEST-OUT) — a VERTEX_LOOP bound on a NON-sphere surface DECLINES ──────────────
// The bare-surface route closes watertight ONLY for a full sphere. A VERTEX_LOOP bound
// on a CYLINDRICAL_SURFACE (an open, non-closable periodic wall) must keep the honest
// OCCT deferral → NULL, never a forced/broken solid.
CC_TEST(vertex_loop_bound_on_non_sphere_declines) {
  const std::string step = vertexLoopSolid(
      "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#2 = DIRECTION('',(0.,0.,1.));\n"
      "#3 = DIRECTION('',(1.,0.,0.));\n"
      "#4 = AXIS2_PLACEMENT_3D('',#1,#2,#3);\n"
      "#5 = CYLINDRICAL_SURFACE('',#4,6.);\n",
      "#5");
  CC_CHECK(ex::readStepString(step).isNull());
}

// ── T1 (TORUS) — a full OCCT torus (TOROIDAL_SURFACE) imports natively watertight ──────
// OCCT writes a whole torus as ONE TOROIDAL_SURFACE ADVANCED_FACE bounded by a fully-seamed
// EDGE_LOOP (the equator v-seam + the tube u-seam, each forward AND reversed). The reader
// maps that bare doubly-periodic surface to a native Kind::Torus face with a null outer wire;
// the tessellator meshes its natural (u,v)∈[0,2π]² rectangle, welding BOTH seams (no poles)
// → a watertight Torus solid. Volume = 2·π²·R·r² within the deflection bound.
CC_TEST(toroidal_surface_full_torus_imports_watertight) {
  const double R = 10.0, r = 3.0;
  const topo::Shape s = ex::readStepString(toroidalSolid(R, r));
  CC_CHECK(!s.isNull());                              // now imports natively (declined before)
  if (s.isNull()) return;
  CC_CHECK(s.type() == topo::ShapeType::Solid);
  CC_CHECK(torusFaceCount(s) == 1);                   // ONE bare periodic Torus face
  CC_CHECK(watertight(s));                            // both seams weld closed, no poles

  const double v = volumeOf(s);
  const double vAnalytic = 2.0 * 3.14159265358979323846 * 3.14159265358979323846 * R * r * r;
  CC_CHECK(v > 0.0);
  CC_CHECK(std::fabs(v - vAnalytic) / vAnalytic < 1e-2);  // converges to 2π²Rr²

  const Box b = worldBox(s);
  const double outer = R + r;
  for (int k = 0; k < 2; ++k) {                       // x,y span the full outer ring
    CC_CHECK(b.hi[k] <= outer + 1e-6 && b.lo[k] >= -outer - 1e-6);
    CC_CHECK(b.hi[k] > outer - 0.2 && b.lo[k] < -(outer - 0.2));
  }
  CC_CHECK(b.hi[2] <= r + 1e-6 && b.lo[2] >= -r - 1e-6);  // z within ±r (the tube)
}

// ── T1 (HONEST-OUT) — a PARTIAL torus (real trim edges) DECLINES → OCCT ─────────────────
// The bare-surface route closes watertight ONLY for a FULL torus (a fully-seamed loop).
// A TOROIDAL_SURFACE face that carries a REAL trim rim (a partial-angle torus segment) has
// no native trimmed-torus mesh path, so it must keep the honest OCCT deferral → NULL.
CC_TEST(toroidal_surface_partial_torus_declines) {
  // Reuse the full-torus authoring but replace the fully-seamed loop with a single real
  // rim edge (one closed circle, used ONCE) → not fully seamed → partial torus → decline.
  std::string step = toroidalSolid(10.0, 3.0);
  const std::string full = "#19 = EDGE_LOOP('',(#20,#49,#71,#72));\n";
  const std::string partial = "#19 = EDGE_LOOP('',(#71));\n";  // one forward rim only
  const auto pos = step.find(full);
  CC_CHECK(pos != std::string::npos);
  step.replace(pos, full.size(), partial);
  CC_CHECK(ex::readStepString(step).isNull());        // honest decline → OCCT
}

// ── T2 (DECLINE) — a non-line generatrix revolves to a torus/general surface → NULL ────
// SURFACE_OF_REVOLUTION of a CIRCLE generatrix is a torus (off-axis) / sphere (on-axis) /
// general revolved surface. The off-axis-circle case now CLASSIFIES as a native Kind::Torus,
// but the synthetic fixture below reuses a CYLINDER's boundary (real rim edges, not a
// fully-seamed torus loop) → the partial-torus guard DECLINES the whole file → NULL → OCCT.
CC_TEST(surface_of_revolution_circle_generatrix_declines) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string rev = revolveSurfaces(
      base, "CYLINDRICAL_SURFACE",
      "#800010 = CARTESIAN_POINT('',(10.,0.,0.));\n"
      "#800011 = DIRECTION('',(0.,1.,0.));\n"
      "#800012 = DIRECTION('',(1.,0.,0.));\n"
      "#800013 = AXIS2_PLACEMENT_3D('',#800010,#800011,#800012);\n"
      "#800004 = CIRCLE('',#800013,2.);\n",  // off-axis circle → a torus generatrix
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(ex::readStepString(rev).isNull());  // honest decline → OCCT
}

// ── T2 (DECLINE) — a SKEW oblique line (→ hyperboloid) is honestly declined ────────────
// An oblique generatrix whose support line does NOT meet the axis (a nonzero common
// perpendicular) revolves to a hyperboloid of one sheet — no native FaceSurface kind.
// lineMeetsAxis returns none → the reader DECLINES → NULL → OCCT. The generatrix here is
// oblique (not ∥, not ⟂) AND offset out of the axis plane (z-component) so it is skew.
CC_TEST(surface_of_revolution_skew_oblique_line_declines) {
  const std::string base = ex::writeStepString(cone20(), "cone");
  const std::string rev = revolveSurfaces(
      base, "CONICAL_SURFACE",
      "#800001 = CARTESIAN_POINT('',(5.,0.,3.));\n"   // offset off the axis plane (z=3)
      "#800002 = DIRECTION('',(-5.,20.,0.));\n"       // oblique but skew to the +Y axis
      "#800003 = VECTOR('',#800002,1.);\n"
      "#800004 = LINE('',#800001,#800003);\n",
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(ex::readStepString(rev).isNull());  // hyperboloid → honest decline → OCCT
}

// ── T2 (DECLINE) — an ELLIPSE whose plane is ⟂ the axis is honestly declined ────────────
// The general-revolution arm maps ONLY a faithful spheroid meridian: an axis-aligned ellipse
// (plane CONTAINS the axis, one semi-axis PARALLEL to it, centre on the axis). This ellipse's
// plane normal is PARALLEL to the +Y revolution axis (plane ⟂ axis) — revolving it sweeps a
// flat washer, not a spheroid — so ellipseMeridian rejects it → NULL → OCCT.
CC_TEST(surface_of_revolution_ellipse_generatrix_declines) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string rev = revolveSurfaces(
      base, "CYLINDRICAL_SURFACE",
      "#800010 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#800011 = DIRECTION('',(0.,1.,0.));\n"       // ellipse-plane normal ∥ the +Y axis (plane ⟂ axis)
      "#800012 = DIRECTION('',(1.,0.,0.));\n"
      "#800013 = AXIS2_PLACEMENT_3D('',#800010,#800011,#800012);\n"
      "#800004 = ELLIPSE('',#800013,5.,3.);\n",    // plane ⟂ axis → not a spheroid meridian
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(ex::readStepString(rev).isNull());  // plane ⟂ axis → honest decline → OCCT
}

// ── T2 (DECLINE) — an on-axis circle whose PLANE is ⟂ the axis is honestly declined ────
// A CIRCLE centred on the axis but lying in a plane PERPENDICULAR to the axis (its normal
// is PARALLEL to the axis) does not revolve to a sphere — revolving it is degenerate. The
// revolvedCircle plane-contains-axis guard rejects it → NULL → OCCT (never a forced sphere).
CC_TEST(surface_of_revolution_on_axis_circle_perp_plane_declines) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string rev = revolveSurfaces(
      base, "CYLINDRICAL_SURFACE",
      "#800010 = CARTESIAN_POINT('',(0.,0.,0.));\n"
      "#800011 = DIRECTION('',(0.,1.,0.));\n"        // circle-plane normal ∥ the +Y axis
      "#800012 = DIRECTION('',(1.,0.,0.));\n"
      "#800013 = AXIS2_PLACEMENT_3D('',#800010,#800011,#800012);\n"
      "#800004 = CIRCLE('',#800013,3.);\n",
      "800004");
  CC_CHECK(rev.find("SURFACE_OF_REVOLUTION") != std::string::npos);
  CC_CHECK(ex::readStepString(rev).isNull());  // plane ⟂ axis → degenerate → decline
}

// ═════════════════════════════════════════════════════════════════════════════
// M4-tail-4 — deep-nested N-level chain guard + general RECTANGULAR_TRIMMED_SURFACE
// ═════════════════════════════════════════════════════════════════════════════

// ── DEEP-NESTED (M4-tail-4) — a 3-level assembly composes W = T₁∘T₂∘T₃ (HOST ANALYTIC) ──
// The landed composeChain walks parent edges leaf→root in an UNBOUNDED loop, so depth is
// data, not a constant. This locks the "already composes N levels" finding with a genuine
// depth-3 tree: leaf(#900008) placed into sub2(#900040) by T₃=(1,0,0); sub2 into
// sub1(#900030) by T₂=(0,2,0); sub1 into root(#900009) by T₁=(0,0,40). GATE (a): the
// reader's placed leaf box is verified against an INDEPENDENT math::Transform product
// W = T₁∘T₂∘T₃ (computed in the test, NOT via composeChain) with NO OCCT. The leaf's
// local box [0,6]³ (centroid (3,3,3)) must land at [1,7]×[2,8]×[40,46], centre (4,5,43).
CC_TEST(nested_three_level_assembly_composes_chain_host_analytic) {
  const TwoBox t = twoBox();
  std::string a;
  a += "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(t.brepB) + "),#900020);\n";
  a += "#900040 = SHAPE_REPRESENTATION('',(),#900020);\n";
  a += "#900030 = SHAPE_REPRESENTATION('',(),#900020);\n";
  a += "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n";
  a += "#900020 = GEOMETRIC_REPRESENTATION_CONTEXT(3);\n";
  a += "#900001 = CARTESIAN_POINT('',(0.,0.,0.));\n";
  a += "#900002 = DIRECTION('',(0.,0.,1.));\n#900003 = DIRECTION('',(1.,0.,0.));\n";
  a += "#900004 = AXIS2_PLACEMENT_3D('',#900001,#900002,#900003);\n";      // identity FROM
  a += "#900101 = CARTESIAN_POINT('',(1.,0.,0.));\n";
  a += "#900102 = AXIS2_PLACEMENT_3D('',#900101,#900002,#900003);\n";      // T3 TO
  a += "#900103 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900102);\n";  // T3
  a += "#900201 = CARTESIAN_POINT('',(0.,2.,0.));\n";
  a += "#900202 = AXIS2_PLACEMENT_3D('',#900201,#900002,#900003);\n";      // T2 TO
  a += "#900203 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900202);\n";  // T2
  a += "#900301 = CARTESIAN_POINT('',(0.,0.,40.));\n";
  a += "#900302 = AXIS2_PLACEMENT_3D('',#900301,#900002,#900003);\n";      // T1 TO
  a += "#900303 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900302);\n";  // T1
  a += "#900410 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900040) "
       "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900103) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
       "#900411 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900410,#900412);\n"
       "#900412 = PRODUCT_DEFINITION_SHAPE('','',#900413);\n"
       "#900413 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('3','','',#900008,#900040,$);\n";
  a += "#900420 = ( REPRESENTATION_RELATIONSHIP('','',#900040,#900030) "
       "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900203) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
       "#900421 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900420,#900422);\n"
       "#900422 = PRODUCT_DEFINITION_SHAPE('','',#900423);\n"
       "#900423 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900040,#900030,$);\n";
  a += "#900430 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900009) "
       "REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900303) SHAPE_REPRESENTATION_RELATIONSHIP() );\n"
       "#900431 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900430,#900432);\n"
       "#900432 = PRODUCT_DEFINITION_SHAPE('','',#900433);\n"
       "#900433 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900009,$);\n";

  const topo::Shape shape = ex::readStepString(mergeAsm(t, a));
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  CC_CHECK(shape.type() == topo::ShapeType::Compound);

  int solids = 0;
  double volSum = 0.0;
  bool allWatertight = true;
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    ++solids;
    if (!watertight(e.current())) allWatertight = false;
    volSum += volumeOf(e.current());
  }
  CC_CHECK(solids == 2);
  CC_CHECK(allWatertight);
  CC_CHECK(std::fabs(volSum - (1000.0 + 216.0)) < 1e-6);

  // INDEPENDENT composition (no reader graph walk, no OCCT): W = T₁∘T₂∘T₃, all
  // translations, so W is a pure translate by (1,2,40); centroid (3,3,3) → (4,5,43).
  const nm::Transform T3{nm::Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}, nm::Vec3{1, 0, 0}};
  const nm::Transform T2{nm::Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}, nm::Vec3{0, 2, 0}};
  const nm::Transform T1{nm::Mat3{1, 0, 0, 0, 1, 0, 0, 0, 1}, nm::Vec3{0, 0, 40}};
  const nm::Point3 wc = T1.composedWith(T2).composedWith(T3).applyToPoint(nm::Point3{3, 3, 3});
  CC_CHECK(std::fabs(wc.x - 4.0) < 1e-9 && std::fabs(wc.y - 5.0) < 1e-9 &&
           std::fabs(wc.z - 43.0) < 1e-9);

  // Discriminate the leaf by its z-placement (z ≥ 40; root stays z ∈ [0,10]).
  Box b{{0, 0, 0}, {0, 0, 0}};
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const Box wb = worldBox(e.current());
    if (wb.lo[2] > 20.0) b = wb;
  }
  CC_CHECK(std::fabs(b.lo[0] - 1.0) < 1e-6 && std::fabs(b.hi[0] - 7.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[1] - 2.0) < 1e-6 && std::fabs(b.hi[1] - 8.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[2] - 40.0) < 1e-6 && std::fabs(b.hi[2] - 46.0) < 1e-6);
  const double cx = 0.5 * (b.lo[0] + b.hi[0]), cy = 0.5 * (b.lo[1] + b.hi[1]),
               cz = 0.5 * (b.lo[2] + b.hi[2]);
  CC_CHECK(std::fabs(cx - wc.x) < 1e-6 && std::fabs(cy - wc.y) < 1e-6 &&
           std::fabs(cz - wc.z) < 1e-6);
}

// ── DEEP-NESTED (M4-tail-4) — a 4-level chain composes (latent depth-cap guard) ──
// Same wiring extended one level: leaf→sub3→sub2→sub1→root, four translations summing to
// (1,2,4,8)→(1,2,44) with T levels (1,0,0),(0,2,0),(0,0,4),(0,0,40). If any constant
// capped the chain length the leaf would land short; the unbounded walk composes all four.
CC_TEST(nested_four_level_assembly_composes_chain) {
  const TwoBox t = twoBox();
  std::string a;
  a += "#900008 = SHAPE_REPRESENTATION('',(#" + std::to_string(t.brepB) + "),#900020);\n";
  a += "#900050 = SHAPE_REPRESENTATION('',(),#900020);\n";   // sub3
  a += "#900040 = SHAPE_REPRESENTATION('',(),#900020);\n";   // sub2
  a += "#900030 = SHAPE_REPRESENTATION('',(),#900020);\n";   // sub1
  a += "#900009 = SHAPE_REPRESENTATION('',(),#900020);\n";   // root
  a += "#900020 = GEOMETRIC_REPRESENTATION_CONTEXT(3);\n";
  a += "#900001 = CARTESIAN_POINT('',(0.,0.,0.));\n";
  a += "#900002 = DIRECTION('',(0.,0.,1.));\n#900003 = DIRECTION('',(1.,0.,0.));\n";
  a += "#900004 = AXIS2_PLACEMENT_3D('',#900001,#900002,#900003);\n";
  a += "#900101 = CARTESIAN_POINT('',(1.,0.,0.));\n#900102 = AXIS2_PLACEMENT_3D('',#900101,#900002,#900003);\n#900103 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900102);\n";
  a += "#900201 = CARTESIAN_POINT('',(0.,2.,0.));\n#900202 = AXIS2_PLACEMENT_3D('',#900201,#900002,#900003);\n#900203 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900202);\n";
  a += "#900301 = CARTESIAN_POINT('',(0.,0.,4.));\n#900302 = AXIS2_PLACEMENT_3D('',#900301,#900002,#900003);\n#900303 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900302);\n";
  a += "#900501 = CARTESIAN_POINT('',(0.,0.,40.));\n#900502 = AXIS2_PLACEMENT_3D('',#900501,#900002,#900003);\n#900503 = ITEM_DEFINED_TRANSFORMATION('','',#900004,#900502);\n";
  a += "#900410 = ( REPRESENTATION_RELATIONSHIP('','',#900008,#900050) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900103) SHAPE_REPRESENTATION_RELATIONSHIP() );\n#900411 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900410,#900412);\n#900412 = PRODUCT_DEFINITION_SHAPE('','',#900413);\n#900413 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('4','','',#900008,#900050,$);\n";
  a += "#900420 = ( REPRESENTATION_RELATIONSHIP('','',#900050,#900040) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900203) SHAPE_REPRESENTATION_RELATIONSHIP() );\n#900421 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900420,#900422);\n#900422 = PRODUCT_DEFINITION_SHAPE('','',#900423);\n#900423 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('3','','',#900050,#900040,$);\n";
  a += "#900430 = ( REPRESENTATION_RELATIONSHIP('','',#900040,#900030) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900303) SHAPE_REPRESENTATION_RELATIONSHIP() );\n#900431 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900430,#900432);\n#900432 = PRODUCT_DEFINITION_SHAPE('','',#900433);\n#900433 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','','',#900040,#900030,$);\n";
  a += "#900440 = ( REPRESENTATION_RELATIONSHIP('','',#900030,#900009) REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#900503) SHAPE_REPRESENTATION_RELATIONSHIP() );\n#900441 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#900440,#900442);\n#900442 = PRODUCT_DEFINITION_SHAPE('','',#900443);\n#900443 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','','',#900030,#900009,$);\n";

  const topo::Shape shape = ex::readStepString(mergeAsm(t, a));
  CC_CHECK(!shape.isNull());
  if (shape.isNull()) return;
  // Leaf translated by (1,2,44): local [0,6]³ → [1,7]×[2,8]×[44,50].
  Box b{{0, 0, 0}, {0, 0, 0}};
  for (topo::Explorer e(shape, topo::ShapeType::Solid); e.more(); e.next()) {
    const Box wb = worldBox(e.current());
    if (wb.lo[2] > 20.0) b = wb;
  }
  CC_CHECK(std::fabs(b.lo[0] - 1.0) < 1e-6 && std::fabs(b.hi[0] - 7.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[1] - 2.0) < 1e-6 && std::fabs(b.hi[1] - 8.0) < 1e-6);
  CC_CHECK(std::fabs(b.lo[2] - 44.0) < 1e-6 && std::fabs(b.hi[2] - 50.0) < 1e-6);
}

// ── RECTANGULAR_TRIMMED_SURFACE (PLANE basis) — unwraps to the SAME solid (HOST) ──
// One box face's PLANE is re-parametrised as a RECTANGULAR_TRIMMED_SURFACE over a (u,v)
// box (a foreign analytic-face form). The reader must unwrap to the basis PLANE and keep
// the face's EDGE_LOOP trim, importing to the IDENTICAL solid as the un-wrapped file:
// EXACT volume (planar), same watertight topology, same face/edge/vertex counts. GATE (a):
// independent host equivalence to the basis-referenced import — no OCCT.
CC_TEST(rect_trimmed_plane_unwraps_to_same_solid) {
  const std::string base = ex::writeStepString(box10(), "box");
  const topo::Shape basis = ex::readStepString(base);
  const std::string wrapped =
      wrapSurfaceInRectTrim(base, "PLANE", "0.,10.,0.,10.");
  CC_CHECK(wrapped.find("RECTANGULAR_TRIMMED_SURFACE") != std::string::npos);
  const topo::Shape back = ex::readStepString(wrapped);
  CC_CHECK(!basis.isNull() && !back.isNull());
  if (basis.isNull() || back.isNull()) return;
  CC_CHECK(watertight(back));
  const double v0 = volumeOf(basis), v1 = volumeOf(back);
  CC_CHECK(std::fabs(v0 - 1000.0) < 1e-6);
  CC_CHECK(std::fabs(v1 - v0) < 1e-6);  // EXACT: the rect box is discarded, loop is the trim
  CC_CHECK(countType(back, topo::ShapeType::Face) == countType(basis, topo::ShapeType::Face));
  CC_CHECK(countType(back, topo::ShapeType::Edge) == countType(basis, topo::ShapeType::Edge));
  CC_CHECK(countType(back, topo::ShapeType::Vertex) == countType(basis, topo::ShapeType::Vertex));
  CC_CHECK(countType(back, topo::ShapeType::Face) == 6);
}

// ── RECTANGULAR_TRIMMED_SURFACE (CYLINDER basis) — watertight + closed-form corner ──
// One cylinder-wall sector's CYLINDRICAL_SURFACE is wrapped in a rect-trim over the
// angular×axial (u,v) box. The reader unwraps to the basis CYLINDER; the solid imports
// watertight with volume = the CLOSED-FORM π·r²·h (analytic, no OCCT). GATE (a): the
// reconstructed cylinder surface's radius equals the analytic value 5, and the surface
// sampled at the box corners S(u,v)=O+r(cos u·X+sin u·Y)+v·Z lies EXACTLY radius r from
// the axis — the closed-form invariant of the unwrapped basis.
CC_TEST(rect_trimmed_cylinder_unwraps_watertight_closed_form) {
  const std::string base = ex::writeStepString(cylinder(), "cyl");
  const std::string wrapped =
      wrapSurfaceInRectTrim(base, "CYLINDRICAL_SURFACE", "0.,6.2831853071795862,0.,20.");
  CC_CHECK(wrapped.find("RECTANGULAR_TRIMMED_SURFACE") != std::string::npos);
  const topo::Shape back = ex::readStepString(wrapped);
  CC_CHECK(!back.isNull());
  if (back.isNull()) return;
  CC_CHECK(watertight(back));
  const double v1 = volumeOf(back);
  const double analytic = 3.14159265358979323846 * 25.0 * 20.0;  // π r² h, r=5 h=20
  CC_CHECK(std::fabs(v1 - analytic) / analytic < 5e-3);          // closed-form volume

  // The reconstructed basis cylinder: radius must equal the analytic 5, and evaluating
  // the analytic surface at the (u,v) box corners must land exactly r from the axis.
  const topo::FaceSurface* cyl = firstSurfaceOfKind(back, topo::FaceSurface::Kind::Cylinder);
  CC_CHECK(cyl != nullptr);
  if (!cyl) return;
  CC_CHECK(std::fabs(cyl->radius - 5.0) < 1e-9);
  const nm::Vec3 X = cyl->frame.x.vec(), Y = cyl->frame.y.vec(), Z = cyl->frame.z.vec();
  const nm::Point3 O = cyl->frame.origin;
  const double us[] = {0.0, 6.2831853071795862}, vs[] = {0.0, 20.0};
  for (double u : us)
    for (double v : vs) {
      const nm::Vec3 P = O.asVec() + (X * (cyl->radius * std::cos(u)) +
                                      Y * (cyl->radius * std::sin(u)) + Z * v);
      const nm::Vec3 rel = P - O.asVec();
      const double axial = nm::dot(rel, Z);
      const double perp = nm::norm(rel - Z * axial);  // distance from the axis
      CC_CHECK(std::fabs(perp - cyl->radius) < 1e-9);
    }
}

// ── RECTANGULAR_TRIMMED_SURFACE (DECLINE) — an UNSUPPORTED basis → NULL → OCCT ──
// The wrapper points at an OFFSET_SURFACE (a keyword surface() does not map). The unwrap
// recurses to the basis, inherits its decline, and returns NULL → OCCT. The rect-trim
// arm never fabricates a face for a basis the reader cannot reconstruct.
CC_TEST(rect_trimmed_unsupported_basis_declines) {
  const std::string base = ex::writeStepString(box10(), "box");
  const std::string wrapped = wrapSurfaceInRectTrim(
      base, "PLANE", "0.,10.,0.,10.",
      /*extraRecords=*/"#700001 = OFFSET_SURFACE('',#5,1.,.T.);\n",
      /*basisOverrideRef=*/"#700001");
  CC_CHECK(wrapped.find("RECTANGULAR_TRIMMED_SURFACE") != std::string::npos);
  CC_CHECK(ex::readStepString(wrapped).isNull());  // unsupported basis → decline
}

// ── RECTANGULAR_TRIMMED_SURFACE (DECLINE) — an INVERTED rect box → NULL → OCCT ──
// The (u,v) box is inverted (u2 < u1): a malformed / unrepresentable trim. validRectBox
// rejects it and the whole file DECLINES rather than importing a fabricated face.
CC_TEST(rect_trimmed_inverted_box_declines) {
  const std::string base = ex::writeStepString(box10(), "box");
  const std::string wrapped = wrapSurfaceInRectTrim(base, "PLANE", "10.,0.,0.,10.");  // u2<u1
  CC_CHECK(wrapped.find("RECTANGULAR_TRIMMED_SURFACE") != std::string::npos);
  CC_CHECK(ex::readStepString(wrapped).isNull());  // inverted box → decline
}

CC_RUN_ALL()

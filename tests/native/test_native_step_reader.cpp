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
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cctype>
#include <cmath>
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

CC_RUN_ALL()

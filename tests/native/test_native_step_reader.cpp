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

// ── DECLINE: a TRANSFORMED assembly (transform tree) → NULL ────────────────────
// A flat multi-solid file imports (above); a file that ALSO carries an assembly
// transform entity must still decline (we cannot place the sub-solids without
// modelling the transform tree). Inject a NEXT_ASSEMBLY_USAGE_OCCURRENCE record.
CC_TEST(decline_transformed_assembly_returns_null) {
  std::string step = ex::writeStepString(box10(), "box");
  const std::size_t insert = step.find('\n', step.find("DATA;")) + 1;
  step.insert(insert, "#98001 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('','','',#98002,#98003,$);\n");
  CC_CHECK(ex::readStepString(step).isNull());
}

CC_RUN_ALL()

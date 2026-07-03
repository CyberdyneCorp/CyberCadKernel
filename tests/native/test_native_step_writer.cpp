// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native STEP AP203 writer (Phase 4, capability #7
// `native-data-exchange`, EXPORT slice). OCCT-FREE — Gate 1 (host, analytic) of
// the two-gate model in openspec/NATIVE-REWRITE.md: the writer compiles and
// unit-tests with clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// It builds native solids with the verified construct library and serialises them
// with writeStepString, then asserts on the emitted ISO-10303-21 entity graph:
//   * the HEADER (FILE_DESCRIPTION / FILE_SCHEMA) + DATA/ENDSEC framing,
//   * the expected Part-42 entity kinds per solid (a box → only PLANE surfaces +
//     LINE curves; a cylinder revolve → CYLINDRICAL_SURFACE + CIRCLE),
//   * the wrapper (MANIFOLD_SOLID_BREP, CLOSED_SHELL, ADVANCED_BREP_SHAPE_
//     REPRESENTATION, SI_UNIT millimetre, PRODUCT),
//   * face/vertex COUNTS matching the topology (a box → 6 ADVANCED_FACE, 8
//     VERTEX_POINT after dedup), proving the shared-vertex dedup works,
//   * that every `#n = ENTITY(...);` line is well-formed and reals carry a '.'.
// The true correctness gate (re-read through OCCT STEPControl_Reader to the same
// solid) runs on the simulator; here we assert the file is structurally valid AP203.
//
// It also asserts the HONEST scope boundary: canSerialize is true for the native
// solids above and false for a null shape / a non-solid, so the engine chooses the
// OCCT fallback correctly.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_step_writer.cpp \
//     src/native/exchange/step_writer.cpp src/native/math/bspline.cpp \
//     src/native/math/bezier.cpp -I src -I tests -o test_native_step_writer
//
#include "native/exchange/native_exchange.h"
#include "native/construct/native_construct.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <string>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace ex = cybercad::native::exchange;

namespace {

// Count non-overlapping occurrences of `needle` in `hay`.
int countOf(const std::string& hay, const std::string& needle) {
  int n = 0;
  for (std::size_t pos = 0; (pos = hay.find(needle, pos)) != std::string::npos; pos += needle.size())
    ++n;
  return n;
}

// A 10×10×10 box as a native prism at the origin.
topo::Shape box10() {
  const double p[] = {0, 0, 10, 0, 10, 10, 0, 10};
  return cst::build_prism(p, 4, 10.0);
}

// A cylinder radius 5, height 20: revolve a rectangle profile (line segments) a
// full turn about Y. Uses the verified native revolve → cylinder + plane caps.
topo::Shape cylinder() {
  // profile in the x>=0 half-plane, axis = Y through origin: points
  // (0,0)->(5,0)->(5,20)->(0,20) closed.
  const double p[] = {0, 0, 5, 0, 5, 20, 0, 20};
  return cst::build_revolution(p, 4, cst::RevolveAxis{0.0, 0.0, 0.0, 1.0}, 2.0 * M_PI);
}

// A 20×12×4 plate with one r=3 circular through-hole at (10,6): the hole wall is a
// single FULL-TURN CYLINDRICAL_SURFACE face — the periodic-wall case that must carry
// a SEAM edge in the STEP file to re-read as a valid solid.
topo::Shape holedPlate() {
  const double outer[] = {0, 0, 20, 0, 20, 12, 0, 12};
  const std::vector<cst::CircleHole> holes = {{10.0, 6.0, 3.0}};
  return cst::build_prism_with_holes(outer, 4, holes, {}, 4.0);
}

// The number of TOP-LEVEL, comma-separated arguments of the FIRST `type(` instance
// in `s` (paren-depth aware, so nested lists/tuples count as one arg). Used to catch
// a malformed entity whose argument count does not match the ISO-10303-42 schema
// (the exact regression this guards: a stray extra '' turning EDGE_LOOP's 2 args into
// 3, or ADVANCED_FACE's 4 into 5, which OCCT's reader rejects as an invalid solid).
int firstEntityArgCount(const std::string& s, const std::string& type) {
  const std::size_t name = s.find(type + "(");
  if (name == std::string::npos) return -1;
  std::size_t i = name + type.size();  // at the '('
  int depth = 0;
  int args = 0;
  bool sawAny = false;
  for (; i < s.size(); ++i) {
    const char c = s[i];
    if (c == '(') {
      ++depth;
      if (depth == 1) { args = 1; }  // opened the arg list → at least one arg
    } else if (c == ')') {
      --depth;
      if (depth == 0) break;  // closed the outer arg list
    } else if (c == ',' && depth == 1) {
      ++args;
    }
    if (depth >= 1 && c != '(' && c != ' ') sawAny = true;
  }
  return sawAny ? args : 0;  // "TYPE()" → 0 args
}

}  // namespace

// ── canSerialize scope boundary ───────────────────────────────────────────────
CC_TEST(can_serialize_box_and_cylinder_reject_null_and_nonsolid) {
  CC_CHECK(ex::canSerialize(box10()));
  const topo::Shape cyl = cylinder();
  CC_CHECK(!cyl.isNull());
  CC_CHECK(ex::canSerialize(cyl));

  CC_CHECK(!ex::canSerialize(topo::Shape{}));  // null → false
  // A bare vertex is not a solid/shell → false.
  const topo::Shape v = topo::ShapeBuilder::makeVertex({1, 2, 3});
  CC_CHECK(!ex::canSerialize(v));
}

// ── HEADER + framing + wrapper for a box ──────────────────────────────────────
CC_TEST(box_step_has_valid_header_and_wrapper) {
  const std::string s = ex::writeStepString(box10(), "box");
  CC_CHECK(!s.empty());

  CC_CHECK(s.rfind("ISO-10303-21;", 0) == 0);        // starts with the magic
  CC_CHECK(countOf(s, "END-ISO-10303-21;") == 1);    // ends with the terminator
  CC_CHECK(countOf(s, "HEADER;") == 1);
  CC_CHECK(countOf(s, "DATA;") == 1);
  CC_CHECK(countOf(s, "ENDSEC;") == 2);              // one per section
  CC_CHECK(countOf(s, "FILE_DESCRIPTION(") == 1);
  CC_CHECK(countOf(s, "FILE_NAME(") == 1);
  CC_CHECK(countOf(s, "FILE_SCHEMA(") == 1);

  // Part-42 topology wrapper.
  CC_CHECK(countOf(s, "MANIFOLD_SOLID_BREP(") == 1);
  CC_CHECK(countOf(s, "CLOSED_SHELL(") == 1);
  CC_CHECK(countOf(s, "ADVANCED_BREP_SHAPE_REPRESENTATION(") == 1);

  // mm units + product boilerplate.
  CC_CHECK(countOf(s, "SI_UNIT(.MILLI.,.METRE.)") == 1);
  CC_CHECK(countOf(s, "PRODUCT(") == 1);
  CC_CHECK(countOf(s, "PRODUCT_DEFINITION(") == 1);
  CC_CHECK(countOf(s, "APPLICATION_CONTEXT(") == 1);
}

// ── Box geometry: 6 planar faces, 12 lines, 8 shared vertices ─────────────────
CC_TEST(box_emits_six_planes_and_eight_vertices) {
  const std::string s = ex::writeStepString(box10(), "box");

  // A prism from a 4-gon: 2 caps + 4 sides = 6 planar faces.
  CC_CHECK(countOf(s, "ADVANCED_FACE(") == 6);
  CC_CHECK(countOf(s, "PLANE(") == 6);
  // No curved surfaces in a box.
  CC_CHECK(countOf(s, "CYLINDRICAL_SURFACE(") == 0);
  CC_CHECK(countOf(s, "CIRCLE(") == 0);
  CC_CHECK(countOf(s, "B_SPLINE_SURFACE_WITH_KNOTS(") == 0);

  // 8 corner vertices after node+location dedup (the native builder shares the
  // cap vertices between adjacent side faces).
  CC_CHECK(countOf(s, "VERTEX_POINT(") == 8);

  // Each face is bounded by exactly one outer loop (no holes).
  CC_CHECK(countOf(s, "FACE_OUTER_BOUND(") == 6);
  CC_CHECK(countOf(s, "FACE_BOUND(") == 0);
  CC_CHECK(countOf(s, "EDGE_LOOP(") == 6);

  // Every line edge is a LINE + EDGE_CURVE; a box has 12 unique edges.
  CC_CHECK(countOf(s, "EDGE_CURVE(") == 12);
  CC_CHECK(countOf(s, "LINE(") == 12);
}

// ── Cylinder geometry: cylindrical wall + circle rims + plane caps ────────────
CC_TEST(cylinder_emits_cylindrical_surface_and_circles) {
  const std::string s = ex::writeStepString(cylinder(), "cyl");
  CC_CHECK(!s.empty());

  CC_CHECK(countOf(s, "CYLINDRICAL_SURFACE(") >= 1);
  CC_CHECK(countOf(s, "CIRCLE(") >= 2);   // top + bottom rim (at least)
  CC_CHECK(countOf(s, "PLANE(") >= 2);    // the two end caps
  CC_CHECK(countOf(s, "MANIFOLD_SOLID_BREP(") == 1);
  CC_CHECK(countOf(s, "CLOSED_SHELL(") == 1);
}

// ── Every DATA line is a well-formed `#n = ENTITY(...);` and reals carry a '.' ──
CC_TEST(every_data_line_is_well_formed) {
  const std::string s = ex::writeStepString(box10(), "box");
  // Extract the DATA section.
  const auto dpos = s.find("DATA;");
  const auto epos = s.find("ENDSEC;", dpos);
  CC_CHECK(dpos != std::string::npos && epos != std::string::npos);

  std::size_t pos = s.find('\n', dpos) + 1;
  int lines = 0;
  int lastId = 0;
  while (pos < epos) {
    const std::size_t eol = s.find('\n', pos);
    const std::string line = s.substr(pos, eol - pos);
    pos = eol + 1;
    if (line.empty()) continue;
    ++lines;
    // Form: "#<id> = NAME(...);"
    CC_CHECK(line[0] == '#');
    const std::size_t eq = line.find(" = ");
    CC_CHECK(eq != std::string::npos);
    const int id = std::stoi(line.substr(1, eq - 1));
    CC_CHECK(id == lastId + 1);  // ids are 1..N, contiguous, ascending
    lastId = id;
    CC_CHECK(line.back() == ';');
    CC_CHECK(line.find('(') != std::string::npos);
  }
  CC_CHECK(lines > 0);
}

// ── A CARTESIAN_POINT coordinate is a STEP REAL (contains a '.') ──────────────
CC_TEST(coordinates_are_step_reals) {
  const std::string s = ex::writeStepString(box10(), "box");
  // Find the first CARTESIAN_POINT and confirm its coordinate tuple has '.'s.
  const auto cp = s.find("CARTESIAN_POINT('',(");
  CC_CHECK(cp != std::string::npos);
  const auto open = s.find("((", cp) == std::string::npos ? s.find("(", cp + 16) : 0;
  (void)open;
  const auto lp = s.find("(", cp + std::string("CARTESIAN_POINT('',").size() - 1);
  const auto rp = s.find(")", lp);
  const std::string tuple = s.substr(lp, rp - lp);
  // The box has integer-valued corners (0 and 10); the writer must still print
  // them as reals "0." / "10." so a STEP reader parses REALs not INTEGERs.
  CC_CHECK(tuple.find('.') != std::string::npos);
}

// ── Regression: EDGE_LOOP and ADVANCED_FACE carry the SCHEMA argument count ─────
// A prior writer bug emitted an extra empty string ('','') into EDGE_LOOP and
// ADVANCED_FACE, giving EDGE_LOOP 3 args (schema: 2 — name + edge_list) and
// ADVANCED_FACE 5 args (schema: 4 — name + bounds + face_geometry + same_sense).
// OCCT's STEPControl_Reader rejects both ("Count of Parameters is not 2/4") and
// transfers an EMPTY solid (0 faces, 0 edges, volume 0), which the round-trip gate
// caught. This asserts the exact schema arg counts so the malformed form cannot
// return.
CC_TEST(edge_loop_and_advanced_face_have_schema_arg_counts) {
  const std::string s = ex::writeStepString(box10(), "box");
  // EDGE_LOOP(name, edge_list) → 2 args.
  CC_CHECK(firstEntityArgCount(s, "EDGE_LOOP") == 2);
  // ADVANCED_FACE(name, bounds, face_geometry, same_sense) → 4 args.
  CC_CHECK(firstEntityArgCount(s, "ADVANCED_FACE") == 4);
  // CLOSED_SHELL(name, cfs_faces) → 2 args (was already correct; pin it).
  CC_CHECK(firstEntityArgCount(s, "CLOSED_SHELL") == 2);
  // FACE_OUTER_BOUND(name, bound, orientation) → 3 args.
  CC_CHECK(firstEntityArgCount(s, "FACE_OUTER_BOUND") == 3);
}

// ── Regression: a full-turn cylindrical hole wall carries a SEAM edge ───────────
// The native hole wall is ONE full-turn CYLINDRICAL_SURFACE face whose loop is two
// closed full-circle rim edges. A periodic STEP surface trimmed to its full period
// is only a valid bounded face if the loop carries a seam edge (a curve used once at
// u=0 and once at u=period). Without the seam OCCT reads the wall back with ZERO
// area → an invalid, leaky solid (volume too large). The writer synthesises the
// seam: the wall's EDGE_LOOP must have 4 oriented edges (2 rims + the seam twice),
// and the seam is an extra LINE edge shared forward+reversed.
CC_TEST(cylindrical_hole_wall_emits_seam_edge) {
  const topo::Shape plate = holedPlate();
  CC_CHECK(!plate.isNull());
  CC_CHECK(ex::canSerialize(plate));
  const std::string s = ex::writeStepString(plate, "plate");
  CC_CHECK(!s.empty());

  // One full-turn cylinder wall.
  CC_CHECK(countOf(s, "CYLINDRICAL_SURFACE(") == 1);
  // Two circular rims on the caps (top + bottom of the hole).
  CC_CHECK(countOf(s, "CIRCLE(") == 2);

  // The seam edge is a single EDGE_CURVE referenced by two ORIENTED_EDGEs of
  // OPPOSITE sense (u=0 reversed, u=period forward). A box (no periodic face) emits
  // NO reversed oriented edge; the holed plate emits at least one — the seam. This
  // pins that the seam path fires ONLY for the periodic wall.
  CC_CHECK(countOf(ex::writeStepString(box10(), "box"), ",.F.)") == 0);
  CC_CHECK(countOf(s, ",.F.)") >= 1);  // the seam's reversed reference exists

  CC_CHECK(firstEntityArgCount(s, "EDGE_LOOP") == 2);      // still schema-correct
  CC_CHECK(firstEntityArgCount(s, "ADVANCED_FACE") == 4);  // still schema-correct
}

CC_RUN_ALL()

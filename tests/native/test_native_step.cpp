// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native STEP AP203 EXPORT slice (Phase 4, capability #7
// `native-data-exchange`). OCCT-FREE — Gate 1 (host, structural) of the two-gate
// model in openspec/NATIVE-REWRITE.md: the writer compiles + unit-tests with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// This file complements tests/native/test_native_step_writer.cpp (which asserts on
// the in-memory string): here we drive writeStepFile() to a TEMP PATH on disk,
// read the file back, and assert STRUCTURALLY on the ISO-10303-21 exchange file —
// there is no OCCT STEPControl_Reader on the host, so correctness is checked by
// FILE structure + REFERENCE INTEGRITY rather than a re-read:
//   * the file begins with `ISO-10303-21;` and ends with `END-ISO-10303-21;`,
//   * it has HEADER; ... ENDSEC; and DATA; ... ENDSEC; sections,
//   * exactly one MANIFOLD_SOLID_BREP and one CLOSED_SHELL,
//   * ADVANCED_FACE count == the solid's face count (box → 6 PLANE faces;
//     cylinder → CYLINDRICAL_SURFACE present + planar caps),
//   * EVERY entity reference (#N) that is USED is also DEFINED — no dangling refs
//     (we parse the `#id =` definitions and every `#id` USE and diff the sets),
//   * units are millimetre (SI_UNIT .MILLI. .METRE.).
//
// The true correctness gate (re-read through OCCT STEPControl_Reader to the SAME
// solid within volume/bbox/topology tolerance) runs on the simulator.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_step.cpp \
//     src/native/exchange/step_writer.cpp src/native/math/bspline.cpp \
//     src/native/math/bezier.cpp -I src -I tests -o test_native_step
//
#include "native/exchange/native_exchange.h"
#include "native/construct/native_construct.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cctype>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <unistd.h>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace ex = cybercad::native::exchange;

namespace {

// ── Fixtures ──────────────────────────────────────────────────────────────────

// A 10×10×10 box as a native prism at the origin → 6 planar faces.
topo::Shape box10() {
  const double p[] = {0, 0, 10, 0, 10, 10, 0, 10};
  return cst::build_prism(p, 4, 10.0);
}

// A cylinder radius 5, height 20: revolve a rectangle profile (line segments) a
// full turn about Z → CYLINDRICAL_SURFACE wall + PLANE caps + CIRCLE rims.
topo::Shape cylinder() {
  const double p[] = {0, 0, 5, 0, 5, 20, 0, 20};
  return cst::build_revolution(p, 4, cst::RevolveAxis{0.0, 0.0, 0.0, 1.0}, 2.0 * M_PI);
}

// ── Small text helpers ──────────────────────────────────────────────────────────

int countOf(const std::string& hay, const std::string& needle) {
  int n = 0;
  for (std::size_t pos = 0; (pos = hay.find(needle, pos)) != std::string::npos; pos += needle.size())
    ++n;
  return n;
}

// The number of ADVANCED_FACE topology faces in a solid (via the native explorer),
// so the test asserts the file's ADVANCED_FACE count against the SHAPE, not a
// hard-coded literal.
int faceCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer fx(s, topo::ShapeType::Face); fx.more(); fx.next()) ++n;
  return n;
}

// Write `solid` to a unique temp .step path and read the whole file back. Returns
// the file contents (empty string on any failure). The path is placed under the
// system temp dir with a per-call suffix so parallel CTest runs do not collide.
std::string exportToTempAndRead(const topo::Shape& solid, const std::string& tag) {
  std::string base = std::string(P_tmpdir);
  if (base.empty()) base = "/tmp";
  const std::string path =
      base + "/cybercad_native_step_" + tag + "_" + std::to_string(::getpid()) + ".step";

  if (!ex::writeStepFile(solid, path, tag)) return {};

  std::ifstream in(path, std::ios::binary);
  if (!in) return {};
  std::ostringstream ss;
  ss << in.rdbuf();
  in.close();
  std::remove(path.c_str());
  return ss.str();
}

// Parse every entity DEFINITION id (a line-leading `#N = ...`) and every entity
// USE id (any `#N` token) in the DATA section, and return them as two sets. A valid
// file has USES ⊆ DEFINITIONS (no dangling references).
struct RefSets {
  std::set<int> defined;
  std::set<int> used;
};

RefSets parseRefs(const std::string& s) {
  RefSets r;
  const auto dpos = s.find("DATA;");
  const auto epos = s.find("ENDSEC;", dpos);
  if (dpos == std::string::npos || epos == std::string::npos) return r;

  // Scan the DATA section line by line.
  std::size_t pos = s.find('\n', dpos);
  if (pos == std::string::npos) return r;
  ++pos;
  while (pos < epos) {
    const std::size_t eol = s.find('\n', pos);
    const std::string line = s.substr(pos, (eol == std::string::npos ? epos : eol) - pos);
    pos = (eol == std::string::npos) ? epos : eol + 1;
    if (line.empty()) continue;

    // Definition: the id immediately before " = " at line start.
    if (line[0] == '#') {
      const std::size_t eq = line.find(" = ");
      if (eq != std::string::npos) {
        try {
          r.defined.insert(std::stoi(line.substr(1, eq - 1)));
        } catch (...) {
        }
      }
    }
    // Uses: every `#N` token anywhere on the line. This naturally includes the
    // definition id too, but a self-consistent set makes USES ⊆ DEFINITIONS hold.
    for (std::size_t h = line.find('#'); h != std::string::npos; h = line.find('#', h + 1)) {
      std::size_t j = h + 1;
      int val = 0;
      bool any = false;
      while (j < line.size() && std::isdigit(static_cast<unsigned char>(line[j]))) {
        val = val * 10 + (line[j] - '0');
        ++j;
        any = true;
      }
      if (any) r.used.insert(val);
    }
  }
  return r;
}

}  // namespace

// ── Box: full structural + reference-integrity assertions on a written FILE ────
CC_TEST(box_written_file_is_structurally_valid_ap203) {
  const topo::Shape box = box10();
  const std::string s = exportToTempAndRead(box, "box");
  CC_CHECK(!s.empty());

  // Exchange-file framing.
  CC_CHECK(s.rfind("ISO-10303-21;", 0) == 0);        // begins with the magic
  CC_CHECK(countOf(s, "END-ISO-10303-21;") == 1);    // ends with the terminator
  // The terminator is the last non-whitespace content in the file.
  const std::string term = "END-ISO-10303-21;";
  const auto tpos = s.find(term);
  CC_CHECK(tpos != std::string::npos &&
           s.find_first_not_of(" \r\n\t", tpos + term.size()) == std::string::npos);

  // HEADER; ... ENDSEC; and DATA; ... ENDSEC; sections, in order.
  const auto hpos = s.find("HEADER;");
  const auto dpos = s.find("DATA;");
  CC_CHECK(hpos != std::string::npos);
  CC_CHECK(dpos != std::string::npos);
  CC_CHECK(hpos < dpos);                              // HEADER before DATA
  CC_CHECK(countOf(s, "ENDSEC;") == 2);               // one per section

  // Exactly one solid + one shell.
  CC_CHECK(countOf(s, "MANIFOLD_SOLID_BREP(") == 1);
  CC_CHECK(countOf(s, "CLOSED_SHELL(") == 1);

  // ADVANCED_FACE count == the solid's face count; a box → 6 PLANE faces.
  CC_CHECK(countOf(s, "ADVANCED_FACE(") == faceCount(box));
  CC_CHECK(countOf(s, "ADVANCED_FACE(") == 6);
  CC_CHECK(countOf(s, "PLANE(") == 6);
  CC_CHECK(countOf(s, "CYLINDRICAL_SURFACE(") == 0);
  CC_CHECK(countOf(s, "CIRCLE(") == 0);

  // Units are mm.
  CC_CHECK(countOf(s, "SI_UNIT(.MILLI.,.METRE.)") == 1);

  // Reference integrity: every USED #N is DEFINED (no dangling refs).
  const RefSets refs = parseRefs(s);
  CC_CHECK(!refs.defined.empty());
  CC_CHECK(!refs.used.empty());
  int dangling = 0;
  for (int u : refs.used)
    if (refs.defined.find(u) == refs.defined.end()) ++dangling;
  CC_CHECK(dangling == 0);
}

// ── Cylinder: curved surface + circle rims + planar caps, refs intact ──────────
CC_TEST(cylinder_written_file_has_cylindrical_surface_and_planar_caps) {
  const topo::Shape cyl = cylinder();
  const std::string s = exportToTempAndRead(cyl, "cyl");
  CC_CHECK(!s.empty());

  CC_CHECK(s.rfind("ISO-10303-21;", 0) == 0);
  CC_CHECK(countOf(s, "END-ISO-10303-21;") == 1);

  CC_CHECK(countOf(s, "MANIFOLD_SOLID_BREP(") == 1);
  CC_CHECK(countOf(s, "CLOSED_SHELL(") == 1);

  // ADVANCED_FACE count matches the topology.
  CC_CHECK(countOf(s, "ADVANCED_FACE(") == faceCount(cyl));

  // A cylinder → cylindrical wall + planar caps + circular rims.
  CC_CHECK(countOf(s, "CYLINDRICAL_SURFACE(") >= 1);
  CC_CHECK(countOf(s, "PLANE(") >= 2);     // the two end caps
  CC_CHECK(countOf(s, "CIRCLE(") >= 2);    // the two rims (at least)

  // Units are mm.
  CC_CHECK(countOf(s, "SI_UNIT(.MILLI.,.METRE.)") == 1);

  // Reference integrity: no dangling refs.
  const RefSets refs = parseRefs(s);
  CC_CHECK(!refs.defined.empty());
  int dangling = 0;
  for (int u : refs.used)
    if (refs.defined.find(u) == refs.defined.end()) ++dangling;
  CC_CHECK(dangling == 0);
}

// ── writeStepFile returns false / writes nothing for an unserialisable shape ───
CC_TEST(unserialisable_shape_writes_no_file) {
  const topo::Shape v = topo::ShapeBuilder::makeVertex({1, 2, 3});
  CC_CHECK(!ex::canSerialize(v));

  std::string base = std::string(P_tmpdir);
  if (base.empty()) base = "/tmp";
  const std::string path =
      base + "/cybercad_native_step_none_" + std::to_string(::getpid()) + ".step";
  std::remove(path.c_str());  // ensure clean slate

  CC_CHECK(!ex::writeStepFile(v, path, "none"));  // returns false
  std::ifstream in(path);
  CC_CHECK(!in.good());  // and wrote nothing
}

CC_RUN_ALL()

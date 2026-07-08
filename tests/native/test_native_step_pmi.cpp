// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native AP242 PMI recognise/classify/count scan
// (step_scan_pmi / step_scan_pmi_content) — OCCT-FREE Gate (a), HOST ANALYTIC.
//
// The scan is a SEPARATE read-only pass over the parsed Part-21 record table; it
// never invokes the geometry mapper. These tests prove, with NO OCCT linked and
// against a KNOWN census:
//   1. A hand-authored PMI block injected into a real importable box solid is
//      recognised/classified/counted EXACTLY (per-class counts, per-item class +
//      keyword, per-item attachment #id).
//   2. Attachment carriers (SHAPE_ASPECT / DATUM_FEATURE) and the solid itself are
//      NOT counted; a PMI-adjacent-but-unrecognised keyword is counted `Unknown`,
//      never faked into a real class.
//   3. A solid-only file reports no PMI (anyPmi=false, total=0).
//   4. BYTE-IDENTICAL GEOMETRY: importing the SAME file with the PMI block present
//      yields a solid identical (volume + face/edge/vertex counts) to importing it
//      without the PMI — the scan is additive and cannot perturb the geometry.
//   5. The file-path entry point (step_scan_pmi) matches the in-memory scan.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_step_pmi.cpp \
//     src/native/exchange/step_reader.cpp src/native/exchange/step_writer.cpp \
//     src/native/heal/heal.cpp src/native/math/bspline.cpp \
//     src/native/math/bezier.cpp -I src -I tests -o test_native_step_pmi
//
#include "native/exchange/native_exchange.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace ex = cybercad::native::exchange;
namespace ntess = cybercad::native::tessellate;

namespace {

// A 10x10x10 box: a real importable native solid (6 planar faces, 12 edges).
topo::Shape box10() {
  const double p[] = {0, 0, 10, 0, 10, 10, 0, 10};
  return cst::build_prism(p, 4, 10.0);
}

double volumeOf(const topo::Shape& s, double deflection = 0.005) {
  ntess::MeshParams p;
  p.deflection = deflection;
  const ntess::Mesh m = ntess::SolidMesher{p}.mesh(s);
  if (!ntess::isWatertight(m)) return -1.0;
  return std::fabs(ntess::enclosedVolume(m));
}

int countType(const topo::Shape& s, topo::ShapeType t) {
  int n = 0;
  for (topo::Explorer ex_(s, t); ex_.more(); ex_.next()) ++n;
  return n;
}

// A hand-authored AP242 PMI block with a KNOWN census. Ids are high (#900xxx) so
// they never collide with the writer's box entities. Carriers (#900001 SHAPE_ASPECT,
// #900002 DATUM_FEATURE) are attachment targets and MUST NOT be counted.
//
// KNOWN CENSUS:  dimensions=2  tolerances=2  datums=1  datumTargets=1  notes=1
//                annotationGeometry=1  unknown=1  total=9  anyPmi=true
// items (ascending #id): 900010,900011,900020,900021,900030,900031,900040,900050,900060
// attachedTo (feature #id, 0 = none): 900001,900001,900001,900001,0,900001,0,0,0
const char* kPmiBlock() {
  return "#900001 = SHAPE_ASPECT('feature','',#900000,.T.);\n"
         "#900002 = DATUM_FEATURE('df','',#900000,.T.);\n"
         // 2 dimensions
         "#900010 = DIMENSIONAL_SIZE(#900001,'diameter');\n"
         "#900011 = DIMENSIONAL_LOCATION('loc','',#900001,#900002);\n"
         // 2 geometric tolerances: a *_TOLERANCE subtype (suffix) + a combined
         // instance whose first sub is GEOMETRIC_TOLERANCE (one entity, counted once)
         "#900020 = FLATNESS_TOLERANCE('','',#900099,#900001);\n"
         "#900021 = ( GEOMETRIC_TOLERANCE('','',#900099,#900001) FLATNESS_TOLERANCE() );\n"
         // 1 datum (its #900070 ref is a product-definition-shape, not a feature -> attachedTo 0)
         "#900030 = DATUM('','',#900070,.T.,'A');\n"
         // 1 datum target (points at the datum feature #900001)
         "#900031 = DATUM_TARGET('P1','1',#900001,.T.,$);\n"
         // 1 textual note (its callout element is inside a list -> attachedTo 0)
         "#900040 = DRAUGHTING_CALLOUT((#900041));\n"
         // 1 annotation geometry (graphical PMI OCCT XDE does not surface -> host-only)
         "#900050 = ANNOTATION_PLANE('',(#900051),#900052);\n"
         // 1 PMI-adjacent but UNRECOGNISED keyword -> Unknown (never faked as a tolerance)
         "#900060 = TOLERANCE_ZONE('',(#900061),#900062,#900063);\n";
}

// Insert the PMI block just before the DATA section's ENDSEC; (leaves geometry lines
// untouched — the injected entities are never reached from the MANIFOLD_SOLID_BREP).
std::string injectPmi(const std::string& stepFile, const std::string& pmiLines) {
  const std::size_t data = stepFile.find("DATA;");
  const std::size_t end = stepFile.find("ENDSEC;", data);
  return stepFile.substr(0, end) + pmiLines + stepFile.substr(end);
}

// Find the item with the given #id in a summary (nullptr if absent).
const ex::PmiAnnotation* itemOf(const ex::PmiSummary& s, int id) {
  for (const auto& a : s.items)
    if (a.id == id) return &a;
  return nullptr;
}

}  // namespace

// ── (1)(2) Known census: exact per-class counts, per-item class/keyword/attachment ──
CC_TEST(pmi_scan_matches_known_census_exactly) {
  const std::string box = ex::writeStepString(box10(), "box");
  CC_CHECK(!box.empty());
  const std::string augmented = injectPmi(box, kPmiBlock());

  const ex::PmiSummary s = ex::step_scan_pmi_content(augmented);

  // Per-class counts (the box itself contributes 0 PMI).
  CC_CHECK_EQ(s.dimensions, static_cast<std::size_t>(2));
  CC_CHECK_EQ(s.tolerances, static_cast<std::size_t>(2));
  CC_CHECK_EQ(s.datums, static_cast<std::size_t>(1));
  CC_CHECK_EQ(s.datumTargets, static_cast<std::size_t>(1));
  CC_CHECK_EQ(s.notes, static_cast<std::size_t>(1));
  CC_CHECK_EQ(s.annotationGeometry, static_cast<std::size_t>(1));
  CC_CHECK_EQ(s.unknown, static_cast<std::size_t>(1));
  CC_CHECK_EQ(s.total, static_cast<std::size_t>(9));
  CC_CHECK(s.anyPmi);
  CC_CHECK_EQ(s.items.size(), static_cast<std::size_t>(9));

  // Carriers + the solid are NOT counted as annotations.
  CC_CHECK(itemOf(s, 900001) == nullptr);  // SHAPE_ASPECT (attachment target)
  CC_CHECK(itemOf(s, 900002) == nullptr);  // DATUM_FEATURE (attachment target)

  // Per-item class + keyword + attachment #id.
  const ex::PmiAnnotation* d0 = itemOf(s, 900010);
  CC_CHECK(d0 && d0->cls == ex::PmiClass::Dimension && d0->keyword == "DIMENSIONAL_SIZE");
  CC_CHECK(d0 && d0->attachedTo == 900001);

  const ex::PmiAnnotation* d1 = itemOf(s, 900011);
  CC_CHECK(d1 && d1->cls == ex::PmiClass::Dimension && d1->keyword == "DIMENSIONAL_LOCATION");
  CC_CHECK(d1 && d1->attachedTo == 900001);

  const ex::PmiAnnotation* t0 = itemOf(s, 900020);
  CC_CHECK(t0 && t0->cls == ex::PmiClass::GeometricTolerance &&
           t0->keyword == "FLATNESS_TOLERANCE");
  CC_CHECK(t0 && t0->attachedTo == 900001);

  const ex::PmiAnnotation* t1 = itemOf(s, 900021);  // combined instance, counted once
  CC_CHECK(t1 && t1->cls == ex::PmiClass::GeometricTolerance &&
           t1->keyword == "GEOMETRIC_TOLERANCE");
  CC_CHECK(t1 && t1->attachedTo == 900001);

  const ex::PmiAnnotation* dat = itemOf(s, 900030);
  CC_CHECK(dat && dat->cls == ex::PmiClass::Datum && dat->keyword == "DATUM");
  CC_CHECK(dat && dat->attachedTo == 0);  // references a product shape, not a feature

  const ex::PmiAnnotation* dt = itemOf(s, 900031);
  CC_CHECK(dt && dt->cls == ex::PmiClass::DatumTarget && dt->keyword == "DATUM_TARGET");
  CC_CHECK(dt && dt->attachedTo == 900001);

  const ex::PmiAnnotation* note = itemOf(s, 900040);
  CC_CHECK(note && note->cls == ex::PmiClass::Note && note->keyword == "DRAUGHTING_CALLOUT");

  const ex::PmiAnnotation* ag = itemOf(s, 900050);
  CC_CHECK(ag && ag->cls == ex::PmiClass::AnnotationGeometry &&
           ag->keyword == "ANNOTATION_PLANE");

  // The PMI-adjacent-but-unrecognised entity is Unknown — its keyword is preserved
  // and it is NEVER promoted into GeometricTolerance.
  const ex::PmiAnnotation* unk = itemOf(s, 900060);
  CC_CHECK(unk && unk->cls == ex::PmiClass::Unknown && unk->keyword == "TOLERANCE_ZONE");

  // items are in ascending #id order (deterministic).
  for (std::size_t i = 1; i < s.items.size(); ++i)
    CC_CHECK(s.items[i - 1].id < s.items[i].id);
}

// ── (3) A solid-only file reports no PMI ───────────────────────────────────────
CC_TEST(pmi_scan_solid_only_reports_none) {
  const std::string box = ex::writeStepString(box10(), "box");
  const ex::PmiSummary s = ex::step_scan_pmi_content(box);
  CC_CHECK(!s.anyPmi);
  CC_CHECK_EQ(s.total, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.dimensions, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.tolerances, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.datums, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.datumTargets, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.notes, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.annotationGeometry, static_cast<std::size_t>(0));
  CC_CHECK_EQ(s.unknown, static_cast<std::size_t>(0));
  CC_CHECK(s.items.empty());
}

// ── (4) BYTE-IDENTICAL GEOMETRY: the PMI block cannot perturb the imported solid ──
CC_TEST(pmi_block_leaves_geometry_import_byte_identical) {
  const std::string box = ex::writeStepString(box10(), "box");
  const std::string augmented = injectPmi(box, kPmiBlock());

  const topo::Shape plain = ex::readStepString(box);
  const topo::Shape withPmi = ex::readStepString(augmented);
  CC_CHECK(!plain.isNull());
  CC_CHECK(!withPmi.isNull());
  CC_CHECK(plain.type() == topo::ShapeType::Solid);
  CC_CHECK(withPmi.type() == topo::ShapeType::Solid);

  // Same watertight volume and identical topology counts — the PMI is dropped from
  // the geometry exactly as before, now merely EXPOSED separately by the scan.
  const double v0 = volumeOf(plain);
  const double v1 = volumeOf(withPmi);
  CC_CHECK(v0 > 0.0 && v1 > 0.0);
  CC_CHECK(std::fabs(v1 - v0) < 1e-9);
  CC_CHECK(std::fabs(v1 - 1000.0) < 1e-6);
  CC_CHECK_EQ(countType(withPmi, topo::ShapeType::Face),
              countType(plain, topo::ShapeType::Face));
  CC_CHECK_EQ(countType(withPmi, topo::ShapeType::Edge),
              countType(plain, topo::ShapeType::Edge));
  CC_CHECK_EQ(countType(withPmi, topo::ShapeType::Vertex),
              countType(plain, topo::ShapeType::Vertex));
  CC_CHECK_EQ(countType(withPmi, topo::ShapeType::Face), 6);
}

// ── (5) The file-path entry point matches the in-memory scan ───────────────────
CC_TEST(pmi_scan_file_path_matches_content) {
  const std::string box = ex::writeStepString(box10(), "box");
  const std::string augmented = injectPmi(box, kPmiBlock());

  // Write to a temp file and scan via the path entry point.
  char tmpl[] = "/tmp/cc_pmi_XXXXXX";
  const int fd = ::mkstemp(tmpl);
  CC_CHECK(fd >= 0);
  if (fd >= 0) ::close(fd);
  const std::string path = tmpl;
  {
    std::ofstream out(path, std::ios::binary);
    out << augmented;
  }
  const ex::PmiSummary fromFile = ex::step_scan_pmi(path);
  const ex::PmiSummary fromMem = ex::step_scan_pmi_content(augmented);
  std::remove(path.c_str());

  CC_CHECK_EQ(fromFile.total, fromMem.total);
  CC_CHECK_EQ(fromFile.dimensions, fromMem.dimensions);
  CC_CHECK_EQ(fromFile.tolerances, fromMem.tolerances);
  CC_CHECK_EQ(fromFile.datums, fromMem.datums);
  CC_CHECK(fromFile.anyPmi == fromMem.anyPmi);

  // A missing path yields an empty census (never a crash / fabricated data).
  const ex::PmiSummary missing = ex::step_scan_pmi("/nonexistent/path/to/none.step");
  CC_CHECK(!missing.anyPmi);
  CC_CHECK_EQ(missing.total, static_cast<std::size_t>(0));
}

CC_RUN_ALL()

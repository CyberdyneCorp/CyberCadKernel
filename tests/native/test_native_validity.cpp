// SPDX-License-Identifier: Apache-2.0
//
// test_native_validity.cpp — GATE A (HOST ANALYTIC, no OCCT) for MOAT M-GS GS6:
// the native solid-validity checker (src/native/analysis/validity.h) and its
// cc_check_solid facade wiring over the NativeEngine.
//
// Every fixture has a KNOWN validity state (the state OCCT's BRepCheck_Analyzer
// would report, verified here without linking OCCT — that native-vs-OCCT match is
// gate (b), the sim harness). A valid box PASSES every check; each deliberately-
// broken fixture FAILS via the SPECIFIC check it violates:
//   * open_shell      — 2 faces removed  → closed = 0
//   * flipped_face    — one triangle reversed (watertight preserved) → oriented = 0
//   * degenerate      — an edge collapsed → nondegenerate = 0
//   * self_intersect  — two interpenetrating boxes → noSelfIntersection = 0
//   * disjoint_ctrl   — two boxes 10 apart → noSelfIntersection = 1 (broad-phase +
//                       parallel-plane guard: no FALSE positive on distant coplanar
//                       faces, the naïve Möller degeneracy).
// The checker NEVER emits a false "valid": a broken fixture's valid() is false.
//
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

#include "cybercadkernel/cc_kernel.h"
#include "native/analysis/validity.h"
#include "native/sheetmetal/sheetmetal.h"          // GS6 bend-strip regression fixture
#include "native/tessellate/native_tessellate.h"   // SolidMesher (host, no OCCT)
#include "native/topology/native_topology.h"

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;
namespace sm = cybercad::native::sheetmetal;
namespace topo = cybercad::native::topology;

static int g_pass = 0, g_fail = 0;
static void check(const char* name, bool ok) {
  if (ok) { ++g_pass; std::printf("  PASS %-52s\n", name); }
  else    { ++g_fail; std::printf("  FAIL %-52s\n", name); }
}

// Axis-aligned box [0,a]×[0,b]×[0,c], outward-CCW-wound and watertight.
static tess::Mesh boxMesh(double a, double b, double c) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{0, 0, 0}, {a, 0, 0}, {a, b, 0}, {0, b, 0},
                           {0, 0, c}, {a, 0, c}, {a, b, c}, {0, b, c}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto quad = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  quad(0, 3, 2, 1); quad(4, 5, 6, 7); quad(0, 1, 5, 4);
  quad(2, 3, 7, 6); quad(1, 2, 6, 5); quad(3, 0, 4, 7);
  return m;
}

// Append `b` (shifted by d) into `a` as a second shell → a two-component mesh.
static tess::Mesh twoBoxes(const nm::Vec3& d) {
  tess::Mesh m = boxMesh(2, 3, 4);
  tess::Mesh o = boxMesh(2, 3, 4);
  const auto base = static_cast<std::uint32_t>(m.vertices.size());
  for (const auto& p : o.vertices) m.vertices.push_back({p.x + d.x, p.y + d.y, p.z + d.z});
  for (const auto& t : o.triangles) m.addTriangle(t.a + base, t.b + base, t.c + base);
  return m;
}

// A single-bend sheet-metal edge-flange solid, meshed to the M0 boundary
// triangulation the checker consumes — the tessellated-cylinder BEND STRIP whose
// false-positive self-intersection is GS6. Built OCCT-FREE via the native builder +
// SolidMesher. Returns an empty mesh if the fold declines (params out of the slice).
static tess::Mesh bendStripMesh(double L, double W, double t, double r, double h, double th) {
  const double rect[8] = {0, 0, L, 0, L, W, 0, W};
  sm::SheetMetalDecline why = sm::SheetMetalDecline::Ok;
  const topo::Shape base = sm::baseFlange(rect, 4, t, &why);
  int rim = -1;
  const topo::ShapeMap map = topo::mapShapes(base, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= map.size(); ++i)
    if (sm::efdetail::isFlangeableRim(base, static_cast<int>(i), sm::BasePlate{L, W, t})) {
      rim = static_cast<int>(i); break;
    }
  const topo::Shape folded = sm::edgeFlange(base, rim, h, r, th, &why);
  if (folded.isNull()) return {};
  tess::MeshParams mp;
  mp.deflection = 0.005;
  return tess::SolidMesher(mp).mesh(folded);
}

// Two COPLANAR, positively-OVERLAPPING triangles welded into a (non-manifold) mesh —
// the genuine coplanar-overlap case the checker must still refuse to certify. Guards
// the coplanar-DISJOINT resolver against over-correction (it must not silently pass a
// real overlap). Both triangles lie in z=0; the second is shifted so it overlaps the
// first's interior. Adjacent same-plane facets (a bare edge/vertex touch) are the
// benign case handled elsewhere; this one truly overlaps in area.
static tess::Mesh coplanarOverlapPair() {
  tess::Mesh m;
  // Triangle A and triangle B, coplanar in z=0, overlapping quadrant. No shared index.
  const nm::Point3 v[6] = {{0, 0, 0}, {4, 0, 0}, {0, 4, 0},   // A
                           {1, 1, 0}, {5, 1, 0}, {1, 5, 0}};  // B (shifted into A)
  for (const auto& p : v) m.vertices.push_back(p);
  m.addTriangle(0, 1, 2);
  m.addTriangle(3, 4, 5);
  return m;
}

// ── GS6 known-state fixture table ─────────────────────────────────────────────
static void test_fixtures() {
  std::printf("[GS6 validity — known-state fixtures]\n");

  // valid box → every check passes.
  {
    const an::ValidityReport r = an::checkSolidMesh(boxMesh(2, 3, 4));
    check("valid box: closed", r.closed);
    check("valid box: oriented", r.oriented);
    check("valid box: nondegenerate", r.nondegenerate);
    check("valid box: finite", r.finite);
    check("valid box: noSelfIntersection", r.noSelfIntersection);
    check("valid box: VALID", r.valid());
  }
  // open shell → closed = 0, and NOT valid.
  {
    tess::Mesh m = boxMesh(2, 3, 4);
    m.triangles.resize(m.triangles.size() - 2);  // drop one face
    const an::ValidityReport r = an::checkSolidMesh(m);
    check("open shell: closed == 0", !r.closed);
    check("open shell: NOT valid", !r.valid());
  }
  // flipped face → watertight preserved (closed=1) but oriented = 0.
  {
    tess::Mesh m = boxMesh(2, 3, 4);
    std::swap(m.triangles[0].b, m.triangles[0].c);  // reverse one triangle's winding
    const an::ValidityReport r = an::checkSolidMesh(m);
    check("flipped face: closed still 1", r.closed);
    check("flipped face: oriented == 0", !r.oriented);
    check("flipped face: NOT valid", !r.valid());
  }
  // degenerate → an edge collapsed to zero length → nondegenerate = 0.
  {
    tess::Mesh m = boxMesh(2, 3, 4);
    m.vertices[1] = m.vertices[0];  // collapse vertex 1 onto 0
    const an::ValidityReport r = an::checkSolidMesh(m);
    check("degenerate: nondegenerate == 0", !r.nondegenerate);
    check("degenerate: NOT valid", !r.valid());
  }
  // self-intersecting: two interpenetrating boxes → noSelfIntersection = 0.
  {
    const an::ValidityReport r = an::checkSolidMesh(twoBoxes({1.0, 0.5, 0.5}));
    check("self-intersect: noSelfIntersection == 0", !r.noSelfIntersection);
    check("self-intersect: NOT valid", !r.valid());
  }
  // disjoint control: two boxes 10 apart → NO false positive (broad-phase +
  // parallel guard). closed/oriented hold per component; noSelfIntersection = 1.
  {
    const an::ValidityReport r = an::checkSolidMesh(twoBoxes({10.0, 0.0, 0.0}));
    check("disjoint control: noSelfIntersection == 1 (no false +)", r.noSelfIntersection);
    check("disjoint control: certified", r.selfIntersectionCertified);
  }
  // non-finite coordinate → finite = 0, NOT valid.
  {
    tess::Mesh m = boxMesh(2, 3, 4);
    m.vertices[2] = {std::nan(""), 0.0, 0.0};
    const an::ValidityReport r = an::checkSolidMesh(m);
    check("non-finite: finite == 0", !r.finite);
    check("non-finite: NOT valid", !r.valid());
  }
}

// ── GS6 cc_check_solid FACADE over the NativeEngine ───────────────────────────
static void test_facade() {
  std::printf("[GS6 validity — cc_check_solid facade]\n");
  cc_set_engine(1);  // native engine
  const double profile[8] = {0, 0, 20, 0, 20, 10, 0, 10};
  const CCShapeId box = cc_solid_extrude(profile, 4, 30.0);
  check("native box built", box != 0);
  if (box != 0) {
    CCValidityReport v{};
    const int ok = cc_check_solid(box, &v);
    check("facade box: cc_check_solid → 1 (decided)", ok == 1 && v.decided == 1);
    check("facade box: valid == 1", v.valid == 1);
    check("facade box: closed_manifold == 1", v.closed_manifold == 1);
    check("facade box: consistent_orientation == 1", v.consistent_orientation == 1);
    check("facade box: no_degenerate == 1", v.no_degenerate == 1);
    check("facade box: finite == 1", v.finite == 1);
    check("facade box: no_self_intersection == 1", v.no_self_intersection == 1);
    check("facade box: first_failure == CC_VALID_OK", v.first_failure == CC_VALID_OK);
    cc_shape_release(box);
  }
  // Unknown body → return 0 + zeroed report (valid == 0), never a fabricated verdict.
  CCValidityReport bad{};
  const int badOk = cc_check_solid(999999, &bad);
  check("cc_check_solid(bad id): returns 0", badOk == 0);
  check("cc_check_solid(bad id): valid == 0", bad.valid == 0);
  cc_set_engine(0);
}

// ── GS6 self-intersection: tessellated-cylinder BEND STRIP false-positive fix ──
// The curved bend region of a sheet-metal edge flange is a faceted cylindrical strip
// welded to the flat base and wedge end-caps. Its facets meet along TANGENT SEAMS
// (T-junctions with no shared vertex INDEX): the coarse base-top face abuts fine bend
// strips, and the fan-triangulated end-caps abut those strips. The pre-fix checker
// (a) reported those tangent-seam touches as a Möller "crossing" and (b) declined on
// the coplanar-disjoint neighbours. The straddle gate + coplanar-disjoint resolver
// close both: a bend-strip solid — VALID (watertight, oriented, χ=2, OCCT-accepted) —
// must now certify. A genuinely self-intersecting solid, and a genuine coplanar
// OVERLAP, must STILL fail (over-correction guard).
static void test_gs6_bend_strip() {
  std::printf("[GS6 validity — tessellated-cylinder bend strip (false-positive fix)]\n");
  struct Case { double L, W, t, r, h, th; const char* tag; };
  const double kPi = 3.14159265358979323846;
  const Case cases[] = {
      {40, 20, 2, 3, 15, kPi / 2, "90deg r3 t2"},
      {40, 20, 2, 3, 15, kPi * 0.75, "135deg r3 t2"},
      {30, 15, 1, 1, 8, 1.5, "tight r1==t1"},
      {30, 15, 1, 0.5, 8, 2.0, "very tight r0.5"},
      {50, 25, 3, 5, 20, kPi / 3, "60deg r5 t3"},
  };
  for (const Case& c : cases) {
    const tess::Mesh m = bendStripMesh(c.L, c.W, c.t, c.r, c.h, c.th);
    char nm1[96], nm2[96], nm3[96];
    std::snprintf(nm1, sizeof nm1, "bend %-16s: built + watertight", c.tag);
    std::snprintf(nm2, sizeof nm2, "bend %-16s: no_self_intersection (no false +)", c.tag);
    std::snprintf(nm3, sizeof nm3, "bend %-16s: VALID (certified)", c.tag);
    check(nm1, !m.vertices.empty() && tess::isWatertight(m));
    const an::ValidityReport r = an::checkSolidMesh(m);
    check(nm2, r.noSelfIntersection);
    check(nm3, r.valid());
  }

  // Over-correction guard #1: a genuine interpenetrating solid still FAILS. Two
  // boxes overlapping in a volume cross transversally (facets straddle) → caught.
  {
    const an::ValidityReport r = an::checkSolidMesh(twoBoxes({1.0, 0.5, 0.5}));
    check("guard: genuine self-intersect still noSelfIntersection == 0", !r.noSelfIntersection);
    check("guard: genuine self-intersect still NOT valid", !r.valid());
  }
  // Over-correction guard #2: a genuine coplanar OVERLAP is still refused (not
  // certified) — the coplanar-disjoint resolver must not pass a real area overlap.
  {
    const an::ValidityReport r = an::checkSolidMesh(coplanarOverlapPair());
    check("guard: coplanar overlap NOT certified (declined)", !r.selfIntersectionCertified);
    check("guard: coplanar overlap NOT valid", !r.valid());
  }
}

int main() {
  std::printf("=== test_native_validity (MOAT M-GS GS6, host-analytic gate) ===\n");
  test_fixtures();
  test_gs6_bend_strip();
  test_facade();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}

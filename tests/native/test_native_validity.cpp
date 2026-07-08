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

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

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

int main() {
  std::printf("=== test_native_validity (MOAT M-GS GS6, host-analytic gate) ===\n");
  test_fixtures();
  test_facade();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}

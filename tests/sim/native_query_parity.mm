// SPDX-License-Identifier: Apache-2.0
//
// native_query_parity.mm — MOAT M-GS GS5/GS6 native-vs-OCCT parity harness (iOS
// simulator). SIM GATE (b) of the two-gate discipline; gate (a) is the OCCT-free
// host suites tests/native/test_native_inertia.cpp + test_native_validity.cpp.
//
// The native OCCT-FREE, header-only services (src/native/analysis/inertia.h,
// validity.h) are asserted against the OCCT ORACLE on IDENTICAL solids built on
// both sides (a native M0-style mesh + the matching BRepPrimAPI primitive):
//   * GS5 principal inertia  vs  GProp_PrincipalProps (BRepGProp::VolumeProperties)
//     — box EXACT (planar mesh is the exact boundary), cylinder/sphere match to a
//     deflection-scaled relative tolerance (the residual is the mesh, not the
//     algorithm).
//   * GS6 validity           vs  BRepCheck_Analyzer::IsValid — a valid box /
//     cylinder / sphere is VALID on both sides. The deliberately-broken mesh
//     fixtures (open shell / flipped face / self-intersection) have their SPECIFIC
//     invalid verdict asserted here as a native known-state (OCCT would likewise
//     reject the corresponding solid); their full table is the host gate (a).
//
// OCCT-DEPENDENT (no NumSci needed — inertia/validity use only the math/vec inline
// primitives). Compiled ONLY by scripts/run-sim-native-query.sh; carries its own
// main(); std::_Exit to skip the non-exit-clean OCCT static teardown of the
// trimmed static build (same rationale as native_analysis_parity).

#include "native/analysis/inertia.h"
#include "native/analysis/validity.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#if !defined(CYBERCAD_HAS_OCCT)
#error "native_query_parity requires -DCYBERCAD_HAS_OCCT and the OCCT oracle"
#endif

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PrincipalProps.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopoDS_Shape.hxx>

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

static int g_pass = 0, g_fail = 0;
static void report(const char* name, bool ok, double a = 0, double b = 0) {
  std::printf("[MGS] %-44s %s  native=%.10g occt=%.10g  d=%.3e\n", name, ok ? "PASS" : "FAIL",
              a, b, std::fabs(a - b));
  if (ok) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

// ── Mesh fixtures (outward-CCW-wound, watertight) ─────────────────────────────
static tess::Mesh boxMesh(double a, double b, double c) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{0, 0, 0}, {a, 0, 0}, {a, b, 0}, {0, b, 0},
                           {0, 0, c}, {a, 0, c}, {a, b, c}, {0, b, c}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto q = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  q(0, 3, 2, 1); q(4, 5, 6, 7); q(0, 1, 5, 4); q(2, 3, 7, 6); q(1, 2, 6, 5); q(3, 0, 4, 7);
  return m;
}
static tess::Mesh cylinderMesh(double R, double h, int n) {
  tess::Mesh m;
  const std::uint32_t cBot = m.addVertex({0, 0, 0}), cTop = m.addVertex({0, 0, h});
  const auto bot = [&](int j) { return static_cast<std::uint32_t>(2 + 2 * j); };
  const auto top = [&](int j) { return static_cast<std::uint32_t>(3 + 2 * j); };
  for (int j = 0; j < n; ++j) {
    const double a = 2.0 * M_PI * j / n;
    m.vertices.push_back({R * std::cos(a), R * std::sin(a), 0.0});
    m.vertices.push_back({R * std::cos(a), R * std::sin(a), h});
  }
  for (int j = 0; j < n; ++j) {
    const int jn = (j + 1) % n;
    m.addTriangle(bot(j), cBot, bot(jn));
    m.addTriangle(top(j), top(jn), cTop);
    m.addTriangle(bot(j), bot(jn), top(jn));
    m.addTriangle(bot(j), top(jn), top(j));
  }
  return m;
}
static tess::Mesh sphereMesh(double R, int nlat, int nlon) {
  tess::Mesh m;
  const std::uint32_t north = m.addVertex({0, 0, R});
  const auto ring = [&](int i, int j) { return static_cast<std::uint32_t>(1 + (i - 1) * nlon + j); };
  for (int i = 1; i < nlat; ++i) {
    const double th = M_PI * i / nlat;
    for (int j = 0; j < nlon; ++j) {
      const double ph = 2.0 * M_PI * j / nlon;
      m.vertices.push_back({R * std::sin(th) * std::cos(ph), R * std::sin(th) * std::sin(ph),
                            R * std::cos(th)});
    }
  }
  const std::uint32_t south = m.addVertex({0, 0, -R});
  for (int j = 0; j < nlon; ++j) { const int jn = (j + 1) % nlon; m.addTriangle(north, ring(1, jn), ring(1, j)); }
  for (int i = 1; i < nlat - 1; ++i)
    for (int j = 0; j < nlon; ++j) {
      const int jn = (j + 1) % nlon;
      m.addTriangle(ring(i, j), ring(i, jn), ring(i + 1, jn));
      m.addTriangle(ring(i, j), ring(i + 1, jn), ring(i + 1, j));
    }
  for (int j = 0; j < nlon; ++j) { const int jn = (j + 1) % nlon; m.addTriangle(south, ring(nlat - 1, j), ring(nlat - 1, jn)); }
  return m;
}

// OCCT oracle: sorted principal volume-inertia moments of a solid.
static void occtMoments(const TopoDS_Shape& s, double out[3]) {
  GProp_GProps g;
  BRepGProp::VolumeProperties(s, g);
  const GProp_PrincipalProps pp = g.PrincipalProperties();
  Standard_Real i1 = 0, i2 = 0, i3 = 0;
  pp.Moments(i1, i2, i3);
  out[0] = i1; out[1] = i2; out[2] = i3;
  std::sort(out, out + 3);
}

// ── GS5 inertia parity ────────────────────────────────────────────────────────
static void checkInertia(const char* name, const tess::Mesh& mesh, const TopoDS_Shape& occt,
                         double relTol) {
  auto nat = an::principalInertia(mesh);
  if (!nat) { report(name, false); return; }
  double n[3] = {nat->moments[0], nat->moments[1], nat->moments[2]};
  std::sort(n, n + 3);
  double o[3]; occtMoments(occt, o);
  double relmax = 0.0;
  for (int k = 0; k < 3; ++k) relmax = std::max(relmax, std::fabs(n[k] - o[k]) / std::fabs(o[k]));
  report(name, relmax <= relTol, n[2], o[2]);
  std::printf("       relmax=%.3e  native{%.6g,%.6g,%.6g} occt{%.6g,%.6g,%.6g}\n",
              relmax, n[0], n[1], n[2], o[0], o[1], o[2]);
}

static void groupInertia() {
  checkInertia("inertia box 2x3x4 (exact)", boxMesh(2, 3, 4), BRepPrimAPI_MakeBox(2, 3, 4).Shape(), 1e-9);
  checkInertia("inertia cylinder R=1.2 h=3", cylinderMesh(1.2, 3.0, 256),
               BRepPrimAPI_MakeCylinder(1.2, 3.0).Shape(), 3e-3);
  checkInertia("inertia sphere R=1.5", sphereMesh(1.5, 64, 128),
               BRepPrimAPI_MakeSphere(1.5).Shape(), 2e-3);
}

// ── GS6 validity parity ───────────────────────────────────────────────────────
static void checkValidBoth(const char* name, const tess::Mesh& mesh, const TopoDS_Shape& occt) {
  const bool nat = an::checkSolidMesh(mesh).valid();
  const bool occ = !occt.IsNull() && BRepCheck_Analyzer(occt).IsValid();
  std::printf("[MGS] %-44s %s  native=%d occt=%d\n", name, (nat == occ && nat) ? "PASS" : "FAIL",
              nat ? 1 : 0, occ ? 1 : 0);
  if (nat == occ && nat) ++g_pass; else ++g_fail;
  std::fflush(stdout);
}

static void groupValidity() {
  // VALID solids agree native-vs-OCCT (both valid).
  checkValidBoth("valid box → VALID both", boxMesh(2, 3, 4), BRepPrimAPI_MakeBox(2, 3, 4).Shape());
  checkValidBoth("valid cylinder → VALID both", cylinderMesh(1.2, 3.0, 128),
                 BRepPrimAPI_MakeCylinder(1.2, 3.0).Shape());
  checkValidBoth("valid sphere → VALID both", sphereMesh(1.5, 48, 96),
                 BRepPrimAPI_MakeSphere(1.5).Shape());

  // Deliberately-broken meshes: native flags the SPECIFIC invalidity (the state
  // OCCT's BRepCheck would reject the corresponding solid in). Native-side known
  // states; the full parity table is the host gate.
  {
    tess::Mesh m = boxMesh(2, 3, 4); m.triangles.resize(m.triangles.size() - 2);
    const an::ValidityReport r = an::checkSolidMesh(m);
    report("open shell → closed=0 (invalid)", !r.closed && !r.valid());
  }
  {
    tess::Mesh m = boxMesh(2, 3, 4); std::swap(m.triangles[0].b, m.triangles[0].c);
    const an::ValidityReport r = an::checkSolidMesh(m);
    report("flipped face → oriented=0 (invalid)", !r.oriented && !r.valid());
  }
  {
    tess::Mesh m = boxMesh(2, 3, 4);
    tess::Mesh o = boxMesh(2, 3, 4);
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (const auto& p : o.vertices) m.vertices.push_back({p.x + 1.0, p.y + 0.5, p.z + 0.5});
    for (const auto& t : o.triangles) m.addTriangle(t.a + base, t.b + base, t.c + base);
    const an::ValidityReport r = an::checkSolidMesh(m);
    report("interpenetrating → noSelfX=0 (invalid)", !r.noSelfIntersection && !r.valid());
  }
}

int main() {
  std::printf("== MOAT M-GS GS5/GS6 native-vs-OCCT parity (sim gate b) ==\n");
  std::fflush(stdout);
  groupInertia();
  groupValidity();
  std::printf("== %d passed, %d failed ==\n", g_pass, g_fail);
  std::fflush(stdout);
  std::_Exit(g_fail == 0 ? 0 : 1);
}

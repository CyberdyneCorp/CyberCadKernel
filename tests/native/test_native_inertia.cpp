// SPDX-License-Identifier: Apache-2.0
//
// test_native_inertia.cpp — GATE A (HOST ANALYTIC, no OCCT) for MOAT M-GS GS5:
// the native mass-inertia service (src/native/analysis/inertia.h) and its
// cc_principal_moments facade wiring over the NativeEngine.
//
// Every assertion is against a HAND-DERIVED CLOSED FORM: the box principal moments
// (m/12)(b²+c²) etc. (reproduced to MACHINE PRECISION — the planar mesh is the
// exact boundary, so this certifies the signed-tetra + eigen algorithm itself),
// the cylinder (½mR² axial, (m/12)(3R²+h²) transverse) and the sphere (2mR²/5),
// which converge to the closed form as the M0 deflection → 0. OCCT is not linked
// (that native-vs-OCCT numeric match is gate (b), the sim harness). HONEST DECLINE
// is first-class: a non-watertight (open) mesh returns std::nullopt, never a
// fabricated tensor.
//
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

#include "cybercadkernel/cc_kernel.h"
#include "native/analysis/inertia.h"

namespace an = cybercad::native::analysis;
namespace tess = cybercad::native::tessellate;
namespace nm = cybercad::native::math;

static int g_pass = 0, g_fail = 0;
static void check(const char* name, bool ok, double got = 0, double want = 0) {
  if (ok) { ++g_pass; std::printf("  PASS %-52s\n", name); }
  else    { ++g_fail; std::printf("  FAIL %-52s got=%.10g want=%.10g\n", name, got, want); }
}

// ── Mesh fixtures (outward-CCW-wound, watertight) ─────────────────────────────

// Axis-aligned box [0,a]×[0,b]×[0,c].
static tess::Mesh boxMesh(double a, double b, double c) {
  tess::Mesh m;
  const nm::Point3 v[8] = {{0, 0, 0}, {a, 0, 0}, {a, b, 0}, {0, b, 0},
                           {0, 0, c}, {a, 0, c}, {a, b, c}, {0, b, c}};
  for (const auto& p : v) m.vertices.push_back(p);
  auto quad = [&](int A, int B, int C, int D) { m.addTriangle(A, B, C); m.addTriangle(A, C, D); };
  quad(0, 3, 2, 1);  // z=0 (−z outward)
  quad(4, 5, 6, 7);  // z=c (+z)
  quad(0, 1, 5, 4);  // y=0 (−y)
  quad(2, 3, 7, 6);  // y=b (+y)
  quad(1, 2, 6, 5);  // x=a (+x)
  quad(3, 0, 4, 7);  // x=0 (−x)
  return m;
}

// Cylinder radius R, height h along +z, base at z=0 (lateral quads + capped fans).
static tess::Mesh cylinderMesh(double R, double h, int n) {
  tess::Mesh m;
  const std::uint32_t cBot = m.addVertex({0, 0, 0});
  const std::uint32_t cTop = m.addVertex({0, 0, h});
  const auto bot = [&](int j) { return static_cast<std::uint32_t>(2 + 2 * j); };
  const auto top = [&](int j) { return static_cast<std::uint32_t>(3 + 2 * j); };
  for (int j = 0; j < n; ++j) {
    const double a = 2.0 * M_PI * j / n;
    m.vertices.push_back({R * std::cos(a), R * std::sin(a), 0.0});
    m.vertices.push_back({R * std::cos(a), R * std::sin(a), h});
  }
  for (int j = 0; j < n; ++j) {
    const int jn = (j + 1) % n;
    m.addTriangle(bot(j), cBot, bot(jn));              // bottom cap (−z outward)
    m.addTriangle(top(j), top(jn), cTop);              // top cap (+z outward)
    m.addTriangle(bot(j), bot(jn), top(jn));           // lateral
    m.addTriangle(bot(j), top(jn), top(j));
  }
  return m;
}

// Sphere radius R centred at origin (single pole vertices + quad rings).
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

// ── GS5: box — exact principal moments + axes (machine precision) ─────────────
static void test_box_exact() {
  std::printf("[GS5 inertia — box (exact)]\n");
  const double a = 2.0, b = 3.0, c = 4.0, V = a * b * c;  // unit density ⇒ m = V
  auto r = an::principalInertia(boxMesh(a, b, c));
  check("box watertight → result", r.has_value());
  if (!r) return;
  check("box volume == abc", std::fabs(r->volume - V) < 1e-12, r->volume, V);
  check("box centroid == (a/2,b/2,c/2)",
        std::fabs(r->centroid.x - a / 2) < 1e-12 && std::fabs(r->centroid.y - b / 2) < 1e-12 &&
            std::fabs(r->centroid.z - c / 2) < 1e-12);

  double exp[3] = {V * (b * b + c * c) / 12.0, V * (a * a + c * c) / 12.0,
                   V * (a * a + b * b) / 12.0};
  std::sort(exp, exp + 3);
  double relmax = 0.0;
  for (int k = 0; k < 3; ++k) relmax = std::max(relmax, std::fabs(r->moments[k] - exp[k]) / exp[k]);
  check("box principal moments == (m/12)(·²+·²) to 1e-12", relmax < 1e-12, relmax, 0.0);

  // Smallest moment is about the LONG axis (z, c=4): its axis is ±z.
  check("box smallest-moment axis ∥ z", std::fabs(r->axes[0].z) > 1.0 - 1e-9,
        std::fabs(r->axes[0].z), 1.0);
  check("box largest-moment axis ∥ x", std::fabs(r->axes[2].x) > 1.0 - 1e-9,
        std::fabs(r->axes[2].x), 1.0);
}

// ── GS5: cylinder — axial ½mR², transverse (m/12)(3R²+h²) (converges) ─────────
static void test_cylinder_converges() {
  std::printf("[GS5 inertia — cylinder (converges)]\n");
  const double R = 1.2, h = 3.0;
  auto r = an::principalInertia(cylinderMesh(R, h, 256));
  check("cylinder watertight → result", r.has_value());
  if (!r) return;
  const double m = r->volume;  // measured mesh mass (deflection-consistent)
  const double axial = 0.5 * m * R * R;
  const double trans = m * (3.0 * R * R + h * h) / 12.0;
  // moments sorted ascending: two equal transverse then the (larger) axial? For
  // R=1.2,h=3: axial=0.72m, trans≈1.11m ⇒ axial is the SMALLEST. Assert both.
  double exp[3] = {axial, trans, trans};
  std::sort(exp, exp + 3);
  double relmax = 0.0;
  for (int k = 0; k < 3; ++k) relmax = std::max(relmax, std::fabs(r->moments[k] - exp[k]) / exp[k]);
  check("cylinder moments ≈ closed form (<3e-3)", relmax < 3e-3, relmax, 0.0);
  // The two transverse moments are equal (axisymmetry) to tight tolerance.
  check("cylinder transverse moments equal", std::fabs(r->moments[1] - r->moments[2]) <
        1e-6 * r->moments[2]);
}

// ── GS5: sphere — 2mR²/5, isotropic (converges) ───────────────────────────────
static void test_sphere_converges() {
  std::printf("[GS5 inertia — sphere (converges)]\n");
  const double R = 1.5;
  double prev = 1e30;
  for (int n : {16, 32, 64}) {
    auto r = an::principalInertia(sphereMesh(R, n, 2 * n));
    if (!r) { check("sphere result", false); return; }
    const double m = r->volume, exp = 0.4 * m * R * R;
    double rel = 0.0;
    for (int k = 0; k < 3; ++k) rel = std::max(rel, std::fabs(r->moments[k] - exp) / exp);
    std::printf("    n=%3d  rel=%.3e\n", n, rel);
    if (n == 64) check("sphere principal == 2mR²/5 (<1e-3 at n=64)", rel < 1e-3, rel, 0.0);
    check("sphere convergence monotone (rel shrinks)", rel < prev + 1e-12);
    prev = rel;
  }
}

// ── GS5: HONEST DECLINE — a non-watertight (open) mesh has no defined inertia ──
static void test_open_declines() {
  std::printf("[GS5 inertia — honest decline]\n");
  tess::Mesh open = boxMesh(2, 3, 4);
  open.triangles.resize(open.triangles.size() - 2);  // drop one face (2 tris) → open shell
  check("DECLINE open (non-watertight) mesh", !an::principalInertia(open).has_value());
  tess::Mesh empty;
  check("DECLINE empty mesh", !an::principalInertia(empty).has_value());
}

// ── GS5: cc_principal_moments FACADE over the NativeEngine ─────────────────────
static void test_facade_box() {
  std::printf("[GS5 inertia — cc_principal_moments facade]\n");
  cc_set_engine(1);  // native engine
  const double a = 20.0, b = 10.0, c = 30.0, V = a * b * c;
  const double profile[8] = {0, 0, a, 0, a, b, 0, b};
  const CCShapeId box = cc_solid_extrude(profile, 4, c);
  check("native box built", box != 0);
  if (box != 0) {
    double got[3] = {0, 0, 0};
    const int ok = cc_principal_moments(box, got);
    check("cc_principal_moments → 1", ok == 1);
    double exp[3] = {V * (b * b + c * c) / 12.0, V * (a * a + c * c) / 12.0,
                     V * (a * a + b * b) / 12.0};
    std::sort(got, got + 3);
    std::sort(exp, exp + 3);
    double relmax = 0.0;
    for (int k = 0; k < 3; ++k) relmax = std::max(relmax, std::fabs(got[k] - exp[k]) / exp[k]);
    check("facade box moments == cuboid formula (<1e-9)", ok == 1 && relmax < 1e-9, relmax, 0.0);
    cc_shape_release(box);
  }
  double buf[3] = {0};
  check("cc_principal_moments(bad id) == 0", cc_principal_moments(999999, buf) == 0);
  cc_set_engine(0);
}

int main() {
  std::printf("=== test_native_inertia (MOAT M-GS GS5, host-analytic gate) ===\n");
  test_box_exact();
  test_cylinder_converges();
  test_sphere_converges();
  test_open_declines();
  test_facade_box();
  std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}

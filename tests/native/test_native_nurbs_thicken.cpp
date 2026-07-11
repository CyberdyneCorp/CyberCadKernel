// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 5 — SOLID THICKEN / SHELL
// (src/native/math/bspline_thicken.{h,cpp}). OCCT-FREE. The oracles are airtight and
// closed-form:
//
//   1. WATERTIGHT — the thickened solid is a CLOSED 2-manifold: every undirected edge is
//      shared by exactly two triangles (isWatertight), it is consistently oriented
//      (isConsistentlyOriented), it has ZERO boundary edges, and its Euler characteristic
//      is χ = 2 (a closed genus-0 shell). This holds for a flat patch, a curved bump, and
//      a rational quarter-cylinder, at both signs of the thickness.
//   2. VOLUME — the enclosed volume ≈ (mid-surface area)·|d| for thin |d|, and for a flat
//      rectangular patch thickened by d it is the EXACT box volume (area·|d|) to ~1e-9.
//      A curved bump's volume converges to area·|d| as |d| → 0. A cylinder shell's volume
//      matches the exact annular-wedge closed form.
//   3. OFFSET SIDE — the offset panel of the solid matches offsetSurface at distance |d|:
//      every offset-cap vertex lies at distance |d| from S (projected via
//      closest_point_on_surface), and the reported offsetError equals offsetSurface's.
//   4. FOLD GUARD — thickening a tightly-curved dome past its minimum radius of curvature
//      is DECLINED (status SelfIntersection, ok=false, empty solid), NOT returned folded;
//      a degenerate near-null-normal patch declines; a safe small thicken of the same dome
//      succeeds (the guard is a genuine curvature test, not a blanket reject).
//
// The routine composes offsetSurface (fits through numerics::lin_solve) and the distance
// oracle uses numerics::closest_point_on_surface, so the whole gate is under
// CYBERCAD_HAS_NUMSCI (like test_native_nurbs_offset). With the guard OFF this compiles to
// a trivial pass so the always-built suite stays green.
//
#include <cstdio>

#ifdef CYBERCAD_HAS_NUMSCI

#include "native/math/bspline.h"
#include "native/math/bspline_offset.h"
#include "native/math/bspline_ops.h"
#include "native/math/bspline_thicken.h"
#include "native/numerics/numerics.h"
#include "native/tessellate/mesh.h"

#include <algorithm>
#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::math;
namespace num = cybercad::native::numerics;
namespace tess = cybercad::native::tessellate;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectNear(double a, double b, double tol, const char* what) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    std::printf("FAIL %-46s got %.15g want %.15g (|Δ|=%.3g tol %g)\n", what, a, b,
                std::fabs(a - b), tol);
    ++g_failures;
  }
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}

// ── Evaluators ─────────────────────────────────────────────────────────────────
static Point3 evalSurf(const BsplineSurfaceData& s, double u, double v) {
  SurfaceGrid g{std::span<const Point3>(s.poles), s.nPolesU, s.nPolesV};
  if (s.weights.empty())
    return surfacePoint(s.degreeU, s.degreeV, g, s.knotsU, s.knotsV, u, v);
  return nurbsSurfacePoint(s.degreeU, s.degreeV, g, s.weights, s.knotsU, s.knotsV, u, v);
}
static double domLo(const std::vector<double>& k, int p) { return k[p]; }
static double domHi(const std::vector<double>& k, int p) { return k[k.size() - 1 - p]; }

// ── Test surfaces (mirror the offset gate's fixtures) ────────────────────────────

// A FLAT rectangular patch in the z=0 plane: [0, Lx]×[0, Ly]. Thickened by d it is an
// exact box of volume Lx·Ly·|d|. Bilinear (degree 1) so the tessellation is exact.
static BsplineSurfaceData flatRect(double Lx, double Ly) {
  BsplineSurfaceData s;
  s.degreeU = 1;
  s.degreeV = 1;
  s.nPolesU = 2;
  s.nPolesV = 2;
  s.knotsU = {0, 0, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  s.poles = {{0, 0, 0}, {0, Ly, 0}, {Lx, 0, 0}, {Lx, Ly, 0}};  // pole(i,j)=[i*nv+j]
  return s;
}

// A non-rational bicubic BUMP patch: a gentle sin/cos height field over a 6×6 net
// (identical to the offset gate). Gently curved so a modest thicken is fold-free.
static BsplineSurfaceData bicubicBump() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 6;
  s.nPolesV = 6;
  s.knotsU = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      const double x = i * 0.4;
      const double y = j * 0.4;
      const double z = 0.35 * std::sin(0.9 * i) * std::cos(0.8 * j);
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A RATIONAL NURBS quarter-cylinder of radius r about the z-axis (90° in U, height h in
// V) — the offset gate's fixture. Thickening OUTWARD by d yields an annular wedge shell.
static BsplineSurfaceData nurbsQuarterCylinder(double r, double h) {
  BsplineSurfaceData s;
  s.degreeU = 2;
  s.degreeV = 1;
  s.nPolesU = 3;
  s.nPolesV = 2;
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 1, 1};
  const double c = std::cos(M_PI / 4.0);
  const Point3 a0{r, 0, 0}, a1{r, r, 0}, a2{0, r, 0};
  const Point3 b0{r, 0, h}, b1{r, r, h}, b2{0, r, h};
  s.poles = {a0, b0, a1, b1, a2, b2};
  s.weights = {1, 1, c, c, 1, 1};
  return s;
}

// A tightly-curved dome (offset gate's tightDome): min principal radius of curvature ≈ R,
// so a thicken by d > R must fold (self-intersect).
static BsplineSurfaceData tightDome(double R) {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 5;
  s.nPolesV = 5;
  s.knotsU = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};
  const double half = 0.5 * R;
  for (int i = 0; i < 5; ++i)
    for (int j = 0; j < 5; ++j) {
      const double x = (-half) + (2 * half) * (i / 4.0);
      const double y = (-half) + (2 * half) * (j / 4.0);
      const double z = R - (x * x + y * y) / (2.0 * R);
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A degenerate patch whose ∂S/∂v ≡ 0 (null normal) — the offset gate's fixture.
static BsplineSurfaceData degenerateNormalPatch() {
  BsplineSurfaceData s;
  s.degreeU = 2;
  s.degreeV = 2;
  s.nPolesU = 3;
  s.nPolesV = 3;
  s.knotsU = {0, 0, 0, 1, 1, 1};
  s.knotsV = {0, 0, 0, 1, 1, 1};
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      s.poles.push_back({i * 1.0, 0.0, 0.0});
  return s;
}

// Assert all closure invariants of a solid (used by several tests).
static void assertClosed(const ThickenResult& r, const char* tag) {
  char buf[128];
  std::snprintf(buf, sizeof buf, "%s: ok", tag);            expectTrue(r.ok, buf);
  if (!r.ok) return;
  std::snprintf(buf, sizeof buf, "%s: watertight", tag);    expectTrue(r.watertight, buf);
  std::snprintf(buf, sizeof buf, "%s: isWatertight(mesh)", tag);
  expectTrue(tess::isWatertight(r.solid), buf);
  std::snprintf(buf, sizeof buf, "%s: consistently oriented", tag);
  expectTrue(r.consistentlyOriented && tess::isConsistentlyOriented(r.solid), buf);
  std::snprintf(buf, sizeof buf, "%s: zero boundary edges", tag);
  expectTrue(r.boundaryEdges == 0 && tess::boundaryEdgeCount(r.solid) == 0, buf);
  std::snprintf(buf, sizeof buf, "%s: euler chi == 2", tag);
  expectTrue(r.eulerCharacteristic == 2, buf);
  std::snprintf(buf, sizeof buf, "%s: positive volume", tag);
  expectTrue(r.enclosedVolume > 0.0 && tess::enclosedVolume(r.solid) > 0.0, buf);
}

int main() {
  // ═══ 1. WATERTIGHT — flat, curved, rational, both signs ═════════════════════════
  {
    for (double d : {0.3, -0.3}) {
      const ThickenResult r = thickenSurface(flatRect(2.0, 1.5), d, 1e-6, 12, 10);
      assertClosed(r, "flat");
    }
    for (double d : {0.15, -0.2}) {
      const ThickenResult r = thickenSurface(bicubicBump(), d, 1e-4, 20, 20);
      assertClosed(r, "bump");
    }
    for (double d : {0.4, -0.4}) {
      const ThickenResult r = thickenSurface(nurbsQuarterCylinder(2.0, 3.0), d, 1e-4, 18, 8);
      assertClosed(r, "cyl");
    }
  }

  // ═══ 2a. VOLUME — flat rectangle thickened by d is an EXACT box ══════════════════
  {
    const double Lx = 2.0, Ly = 1.5;
    for (double d : {0.3, -0.45, 0.8}) {
      const ThickenResult r = thickenSurface(flatRect(Lx, Ly), d, 1e-6, 8, 6);
      expectTrue(r.ok, "flat box ok");
      if (!r.ok) continue;
      const double want = Lx * Ly * std::fabs(d);  // exact box volume
      expectNear(r.enclosedVolume, want, 1e-9, "flat thicken volume == Lx*Ly*|d| exactly");
      // Mid-surface area is exactly the rectangle area (bilinear patch, exact tessellation).
      expectNear(r.surfaceAreaMid, Lx * Ly, 1e-9, "flat mid-surface area == Lx*Ly");
    }
  }

  // ═══ 2b. VOLUME — curved bump ≈ area·|d| for thin d (converges as d → 0) ═════════
  {
    const BsplineSurfaceData S = bicubicBump();
    // For a thin slab V ≈ A·|d|; the relative error is O(|d|·curvature). Shrinking d
    // must shrink the relative discrepancy.
    double prevRel = 1e9;
    bool converges = true;
    for (double d : {0.10, 0.05, 0.02}) {
      const ThickenResult r = thickenSurface(S, d, 1e-4, 28, 28);
      expectTrue(r.ok, "bump thin ok");
      if (!r.ok) { converges = false; break; }
      const double approx = r.surfaceAreaMid * d;  // d > 0 here
      const double rel = std::fabs(r.enclosedVolume - approx) / approx;
      if (rel > prevRel + 1e-6) converges = false;
      prevRel = rel;
    }
    expectTrue(converges, "bump volume→area·|d| relative error shrinks as d→0");
    expectLE(prevRel, 5e-2, "bump thinnest slab volume within 5% of area·|d|");
  }

  // ═══ 2c. VOLUME — cylinder shell matches the annular-wedge closed form ═══════════
  {
    const double r = 2.0, h = 3.0;
    const double d = 0.5;  // OUTWARD: radii r..r+d
    const ThickenResult res = thickenSurface(nurbsQuarterCylinder(r, h), d, 1e-4, 40, 6);
    expectTrue(res.ok, "cyl shell ok");
    if (res.ok) {
      // A 90° wedge of an annulus between radius r and r+d, height h:
      //   V = (π/4)·((r+d)² − r²)·h  = (π/4)·(2rd + d²)·h
      const double want = (M_PI / 4.0) * ((r + d) * (r + d) - r * r) * h;
      // Tessellation of the 90° arc under-integrates the area slightly; a modest relative
      // bound at this resolution is the honest oracle.
      const double rel = std::fabs(res.enclosedVolume - want) / want;
      expectLE(rel, 2e-3, "cyl shell volume ≈ annular-wedge closed form");
    }
  }

  // ═══ 3. OFFSET SIDE — offset cap at distance |d| from S; matches offsetSurface ═══
  {
    const BsplineSurfaceData S = bicubicBump();
    const double d = 0.18;
    const ThickenResult r = thickenSurface(S, d, 1e-4, 22, 22);
    expectTrue(r.ok, "offset-side thicken ok");
    const OffsetResult off = offsetSurface(S, d, 1e-4);
    expectTrue(off.ok, "reference offsetSurface ok");
    if (r.ok && off.ok) {
      // The thicken reports the SAME achieved offset error as offsetSurface (it composes it).
      expectNear(r.offsetError, off.maxError, 1e-12, "thicken.offsetError == offsetSurface.maxError");

      // Every OFFSET-cap vertex of the solid lies at distance |d| from S. The offset cap is
      // the second vertex block: indices [nu*nv, 2*nu*nv). Project onto S and require |d|.
      const std::size_t nCap = static_cast<std::size_t>(r.gridU) * r.gridV;
      expectTrue(r.solid.vertices.size() == 2 * nCap, "solid has two cap vertex blocks");
      const double su0 = domLo(S.knotsU, S.degreeU), su1 = domHi(S.knotsU, S.degreeU);
      const double sv0 = domLo(S.knotsV, S.degreeV), sv1 = domHi(S.knotsV, S.degreeV);
      auto Seval = [&](double u, double v) { return evalSurf(S, u, v); };
      double worst = 0.0;
      for (std::size_t k = nCap; k < 2 * nCap; ++k) {
        const Point3 p = r.solid.vertices[k];
        const num::SurfaceProjection pr =
            num::closest_point_on_surface(Seval, su0, su1, sv0, sv1, p, 24, 24);
        expectTrue(pr.success, "offset-cap projection onto S succeeds");
        worst = std::max(worst, std::fabs(pr.distance - std::fabs(d)));
      }
      expectLE(worst, 1e-9, "every offset-cap vertex is exactly |d| from S along N");

      // The ORIGINAL-cap block sits ON S (distance 0).
      double worstS = 0.0;
      for (std::size_t k = 0; k < nCap; ++k) {
        const Point3 p = r.solid.vertices[k];
        const num::SurfaceProjection pr =
            num::closest_point_on_surface(Seval, su0, su1, sv0, sv1, p, 24, 24);
        worstS = std::max(worstS, pr.distance);
      }
      expectLE(worstS, 1e-9, "every original-cap vertex lies on S");
    }
  }

  // ═══ 4. FOLD GUARD — over-radius thicken declines; degenerate declines ═══════════
  {
    const double R = 0.5;
    const BsplineSurfaceData dome = tightDome(R);

    const ThickenResult big1 = thickenSurface(dome, 1.5 * R, 1e-3, 16, 16);
    const ThickenResult big2 = thickenSurface(dome, -1.5 * R, 1e-3, 16, 16);
    const bool foldedDeclined =
        (big1.status == ThickenStatus::SelfIntersection && !big1.ok) ||
        (big2.status == ThickenStatus::SelfIntersection && !big2.ok);
    expectTrue(foldedDeclined, "over-radius thicken DECLINED as fold (SelfIntersection)");
    // A folded solid must NEVER be handed back: a self-intersection decline is !ok with an
    // empty solid.
    if (big1.status == ThickenStatus::SelfIntersection)
      expectTrue(!big1.ok && big1.solid.triangles.empty(),
                 "over-radius (sign +) declined with no folded solid");
    if (big2.status == ThickenStatus::SelfIntersection)
      expectTrue(!big2.ok && big2.solid.triangles.empty(),
                 "over-radius (sign -) declined with no folded solid");

    // The concave-side decline reports the curvature radius it tripped on (≈ R).
    const ThickenResult* fold =
        (big1.status == ThickenStatus::SelfIntersection) ? &big1 : &big2;
    expectTrue(fold->status == ThickenStatus::SelfIntersection,
               "the concave-side over-radius thicken trips the fold guard");
    expectNear(fold->minCurvatureRadius, R, 0.2 * R,
               "reported min curvature radius ≈ dome radius R");

    // A SAFE small thicken of the same dome on the fold-free side SUCCEEDS and is closed.
    const double safeD = (fold == &big1) ? -0.1 * R : 0.1 * R;
    const ThickenResult safe = thickenSurface(dome, safeD, 1e-3, 16, 16);
    assertClosed(safe, "dome-safe");

    // Degenerate (null-normal) patch declines, never a crash.
    const ThickenResult deg = thickenSurface(degenerateNormalPatch(), 0.2, 1e-3);
    expectTrue(!deg.ok, "degenerate-normal patch declines");
    expectTrue(deg.status == ThickenStatus::DegenerateNormal ||
                   deg.status == ThickenStatus::DegenerateInput,
               "degenerate patch reports a degeneracy status");
    expectTrue(deg.solid.triangles.empty(), "degenerate decline returns no solid");

    // Zero thickness declines cleanly (no solid to build).
    const ThickenResult zero = thickenSurface(flatRect(1.0, 1.0), 0.0, 1e-6);
    expectTrue(!zero.ok && zero.status == ThickenStatus::ZeroThickness,
               "zero thickness declines (ZeroThickness)");
  }

  std::printf("nurbs_thicken: %d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

#else  // CYBERCAD_HAS_NUMSCI

int main() {
  std::printf("nurbs_thicken: numsci disabled — trivially passing.\n");
  return 0;
}

#endif  // CYBERCAD_HAS_NUMSCI

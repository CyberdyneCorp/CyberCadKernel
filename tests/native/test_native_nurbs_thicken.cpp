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
#include <cstdint>
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

// A patch that folds its offset over only PART of the domain: a tight-curvature bump
// occupying the u∈[0,~0.5] strip, flat elsewhere. An inward/large offset self-intersects on
// the tight strip but is fold-free on the flat remainder — so thickenSurface DECLINES the
// whole request while thickenTrimmed keeps the fold-free region.
static BsplineSurfaceData partialFoldPatch(double bumpHeight) {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 6;
  s.nPolesV = 6;
  s.knotsU = {0, 0, 0, 0, 0.3, 0.6, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.333333333333, 0.666666666667, 1, 1, 1, 1};
  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j) {
      const double x = i * 0.5;
      const double y = j * 0.5;
      // A sharp isolated bump on the i∈[1,3] columns → tight curvature there only.
      const double z = (i >= 1 && i <= 3) ? bumpHeight * std::exp(-(i - 2.0) * (i - 2.0)) : 0.0;
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A CENTRAL-RIDGE patch: a tall narrow Gaussian ridge running as a BAND across the middle in
// u, spanning the whole v. A large outward offset folds ONLY the central band, splitting the
// fold-free parameter space into TWO large rectangles (low-u and high-u). thickenTrimmed keeps
// only the single largest side; thickenMultiTrimmed must recover BOTH as disjoint closed
// solids.
static BsplineSurfaceData centralRidgePatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 7;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35, y = j * 0.35;
      const double du = i - 3.0;                       // ridge centred at the mid-u row
      const double z = 0.9 * std::exp(-1.4 * du * du);  // narrow ridge, invariant in v
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A DIAGONAL-RIDGE patch: a tall narrow Gaussian ridge running along the (i == j) DIAGONAL in
// index space. A large offset folds ONLY the diagonal band, leaving TWO TRIANGULAR fold-free
// regions (upper-left, lower-right). thickenMultiTrimmed inscribes an axis-aligned rectangle in
// each triangle (dropping the corner); thickenFoldTrim follows the diagonal fold and recovers a
// column-band that hugs the diagonal — a strictly larger closed solid on each side.
static BsplineSurfaceData diagonalRidgePatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 7;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35, y = j * 0.35;
      const double t = i - j;                          // ridge along the i == j diagonal
      const double z = 0.9 * std::exp(-1.4 * t * t);    // narrow ridge, diagonal in (u,v)
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A CENTRAL-DOME patch: a tall tight round Gaussian dome at the domain centre. An offset
// toward the crest's centre of curvature folds a CLOSED, GENUINELY CURVED (circular) disk
// around the crest — the fold-free space is ONE component that WRAPS AROUND the disk, whose
// u-columns crossing the disk carry TWO fold-free v-runs. A per-u single-interval band cannot
// represent it (the previous fold-locus trace dropped the whole component); the multi-band
// decomposition must thicken the simple bands around the disk into disjoint closed solids.
static BsplineSurfaceData centralDomePatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 7;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 7; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35, y = j * 0.35;
      const double du = i - 3.0, dv = j - 3.0;          // round dome at the net centre
      const double z = 0.9 * std::exp(-1.4 * (du * du + dv * dv));
      s.poles.push_back({x, y, z});
    }
  return s;
}

// A TWIN TALL-DOME patch: two Gaussian domes along u, each ELONGATED in v (wu > wv), on a wide
// 11×7 net. A thicken toward the crests' centres of curvature folds TWO tall closed loops; the
// wrap-around fold-free component's column bands SPLIT into an above/below ARM pair at each
// loop's left edge and MERGE back at its right edge. Each arm is a sound multi-column band of
// only ~2.9% domain area — below the per-band 5% bar that used to drop all four (split/merge
// seam-column residual). The component-level gate must thicken them into closed solids.
static BsplineSurfaceData twinTallDomePatch() {
  BsplineSurfaceData s;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 11;
  s.nPolesV = 7;
  s.knotsU = {0, 0, 0, 0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1, 1, 1, 1};  // 11+3+1=15
  s.knotsV = {0, 0, 0, 0, 0.25, 0.5, 0.75, 1, 1, 1, 1};
  for (int i = 0; i < 11; ++i)
    for (int j = 0; j < 7; ++j) {
      const double x = i * 0.35, y = j * 0.35;
      double z = 0.0;
      for (double c : {3.0, 7.0}) {  // two tall (v-elongated) domes centred on the mid-v row
        const double du = i - c, dv = j - 3.0;
        z += 0.9 * std::exp(-(2.0 * du * du + 0.5 * dv * dv));
      }
      s.poles.push_back({x, y, z});
    }
  return s;
}

// ── Robust triangle-pair PIERCING self-intersection oracle (edge-vs-triangle) ──────
// True iff any two NON-adjacent triangles of the solid properly cross (an edge of one pierces
// the interior of the other). Ignores coplanar skin contact (scale-relative det skip) so a
// clean welded shell reads self-intersection-FREE. The airtight check for the trim oracle.
static bool segPierce(const Point3& p, const Point3& q, const Point3& t0, const Point3& t1,
                      const Point3& t2, double ep) {
  const Vec3 e1 = t1 - t0, e2 = t2 - t0, dir = q - p, pv = cross(dir, e2);
  const double det = dot(e1, pv);
  const double sv = norm(dir) * norm(cross(e1, e2));
  if (std::fabs(det) < 1e-9 * sv || sv < 1e-30) return false;
  const double inv = 1.0 / det;
  const Vec3 tv = p - t0;
  const double bu = dot(tv, pv) * inv;
  if (bu <= ep || bu >= 1.0 - ep) return false;
  const Vec3 qv = cross(tv, e1);
  const double bv = dot(dir, qv) * inv;
  if (bv <= ep || bu + bv >= 1.0 - ep) return false;
  const double t = dot(e2, qv) * inv;
  return t > ep && t < 1.0 - ep;
}
static bool selfIntersects(const tess::Mesh& m, double ep) {
  for (std::size_t i = 0; i < m.triangles.size(); ++i)
    for (std::size_t j = i + 1; j < m.triangles.size(); ++j) {
      const tess::Triangle& ti = m.triangles[i];
      const tess::Triangle& tj = m.triangles[j];
      const std::uint32_t vs[3] = {ti.a, ti.b, ti.c}, ws[3] = {tj.a, tj.b, tj.c};
      bool sh = false;
      for (int x = 0; x < 3 && !sh; ++x)
        for (int y = 0; y < 3; ++y) if (vs[x] == ws[y]) { sh = true; break; }
      if (sh) continue;
      const Point3 A[3] = {m.vertices[ti.a], m.vertices[ti.b], m.vertices[ti.c]};
      const Point3 B[3] = {m.vertices[tj.a], m.vertices[tj.b], m.vertices[tj.c]};
      for (int k = 0; k < 3; ++k) {
        if (segPierce(A[k], A[(k + 1) % 3], B[0], B[1], B[2], ep)) return true;
        if (segPierce(B[k], B[(k + 1) % 3], A[0], A[1], A[2], ep)) return true;
      }
    }
  return false;
}

// Byte-identical mesh comparison (same vertex positions, same triangle indices, same order).
static bool meshBitEqual(const tess::Mesh& a, const tess::Mesh& b) {
  if (a.vertices.size() != b.vertices.size() || a.triangles.size() != b.triangles.size())
    return false;
  for (std::size_t i = 0; i < a.vertices.size(); ++i)
    if (a.vertices[i].x != b.vertices[i].x || a.vertices[i].y != b.vertices[i].y ||
        a.vertices[i].z != b.vertices[i].z)
      return false;
  for (std::size_t i = 0; i < a.triangles.size(); ++i)
    if (a.triangles[i].a != b.triangles[i].a || a.triangles[i].b != b.triangles[i].b ||
        a.triangles[i].c != b.triangles[i].c)
      return false;
  return true;
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

  // ═══ 5. SELF-INTERSECTION TRIM — thickenTrimmed (G4) ═════════════════════════════
  {
    // 5a. NO-INTERPENETRATION PASSTHROUGH — a gently curved / flat face thickened by a safe
    //     |d| is BYTE-IDENTICAL to thickenSurface (the trim path is a no-op).
    for (int c = 0; c < 2; ++c) {
      const BsplineSurfaceData S = c ? bicubicBump() : flatRect(2.0, 1.5);
      const double d = c ? 0.15 : 0.3;
      const int gu = c ? 20 : 12, gv = c ? 20 : 10;
      const ThickenResult base = thickenSurface(S, d, 1e-4, gu, gv);
      const ThickenResult trim = thickenTrimmed(S, d, 1e-4, gu, gv);
      expectTrue(base.ok && trim.ok, c ? "passthrough bump ok" : "passthrough flat ok");
      expectTrue(!trim.trimmed, "passthrough: nothing interpenetrated (trimmed == false)");
      expectTrue(meshBitEqual(base.solid, trim.solid),
                 "passthrough: thickenTrimmed byte-identical to thickenSurface");
      expectNear(trim.enclosedVolume, base.enclosedVolume, 1e-12,
                 "passthrough: identical enclosed volume");
    }

    // 5b. INTERPENETRATION TRIMMED — a face whose offset folds over PART of the domain:
    //     thickenSurface DECLINES (SelfIntersection); thickenTrimmed cuts the folded strip
    //     and returns a WATERTIGHT, self-intersection-FREE solid over the fold-free region.
    {
      const BsplineSurfaceData S = partialFoldPatch(2.0);
      const double d = 0.8;  // large outward offset: folds on the tight-curvature strip only
      const ThickenResult base = thickenSurface(S, d, 1e-3, 16, 16);
      expectTrue(!base.ok && base.status == ThickenStatus::SelfIntersection,
                 "partial-fold: thickenSurface declines (SelfIntersection)");
      const ThickenResult trim = thickenTrimmed(S, d, 1e-3, 16, 16);
      expectTrue(trim.ok, "partial-fold: thickenTrimmed produces a valid solid");
      if (trim.ok) {
        expectTrue(trim.trimmed, "partial-fold: reported trimmed == true");
        expectTrue(trim.watertight && tess::isWatertight(trim.solid),
                   "partial-fold trimmed solid is watertight");
        expectTrue(trim.consistentlyOriented && tess::isConsistentlyOriented(trim.solid),
                   "partial-fold trimmed solid consistently oriented");
        expectTrue(trim.eulerCharacteristic == 2, "partial-fold trimmed χ == 2");
        expectTrue(trim.boundaryEdges == 0, "partial-fold trimmed zero boundary edges");
        expectTrue(trim.enclosedVolume > 0.0, "partial-fold trimmed positive volume");
        // The airtight oracle: NO two non-adjacent triangles cross (self-intersection-free).
        expectTrue(!selfIntersects(trim.solid, 1e-6),
                   "partial-fold trimmed solid is self-intersection-free");
        // The kept region is a strict interior sub-rectangle (the folded strip was cut away).
        expectTrue(trim.keptU1 > trim.keptU0 && trim.keptV1 > trim.keptV0,
                   "partial-fold kept rectangle is non-degenerate");
        expectTrue(trim.keptU0 > 0.0 || trim.keptU1 < 1.0,
                   "partial-fold trimmed the fold strip in u (kept ⊊ full domain)");
      }
    }

    // 5c. FULLY DEGENERATE — a dome that folds over its WHOLE domain has no valid fold-free
    //     region: thickenTrimmed HONEST-DECLINES (empty solid, never a self-intersecting one).
    {
      const double R = 0.5;
      const ThickenResult trim = thickenTrimmed(tightDome(R), -1.5 * R, 1e-3, 16, 16);
      expectTrue(!trim.ok, "fully-degenerate dome: thickenTrimmed declines");
      expectTrue(trim.solid.triangles.empty(),
                 "fully-degenerate decline returns no (self-intersecting) solid");
    }
  }

  // ═══ 6. MULTI-REGION SELF-INTERSECTION TRIM — thickenMultiTrimmed ════════════════
  {
    // A central ridge whose offset folds only the middle band, splitting the fold-free space
    // into TWO rectangles. thickenTrimmed keeps ONE side; thickenMultiTrimmed recovers BOTH as
    // DISJOINT closed watertight solids — the honest result of thickening a fold-split face.
    const BsplineSurfaceData S = centralRidgePatch();
    const double d = 0.6;  // toward the ridge's centre of curvature → fold the central band

    // Baseline: thickenSurface declines (fold); thickenTrimmed keeps a single fold-free side.
    const ThickenResult base = thickenSurface(S, d, 1e-3, 16, 16);
    expectTrue(!base.ok && base.status == ThickenStatus::SelfIntersection,
               "central-ridge: thickenSurface declines (SelfIntersection)");
    const ThickenResult single = thickenTrimmed(S, d, 1e-3, 16, 16);
    expectTrue(single.ok && single.trimmed, "central-ridge: thickenTrimmed keeps one side");

    // Multi-trim: >= 2 disjoint closed solids, each fully verified.
    const std::vector<ThickenResult> multi = thickenMultiTrimmed(S, d, 1e-3, 16, 16);
    expectTrue(multi.size() >= 2,
               "central-ridge: thickenMultiTrimmed recovers >= 2 disjoint solids");

    bool sawLowU = false, sawHighU = false;
    double totalVol = 0.0;
    for (std::size_t k = 0; k < multi.size(); ++k) {
      const ThickenResult& r = multi[k];
      char tag[64];
      std::snprintf(tag, sizeof tag, "central-ridge multi[%zu]", k);
      assertClosed(r, tag);  // ok, watertight, χ==2, zero boundary, oriented, positive volume
      if (!r.ok) continue;
      expectTrue(r.trimmed, "central-ridge: each recovered solid reports trimmed == true");
      // Each recovered solid is self-intersection-FREE (the fold band was cut out).
      char sbuf[80];
      std::snprintf(sbuf, sizeof sbuf, "central-ridge multi[%zu]: self-intersection-free", k);
      expectTrue(!selfIntersects(r.solid, 1e-6), sbuf);
      totalVol += r.enclosedVolume;
      const double midU = 0.5 * (r.keptU0 + r.keptU1);
      if (midU < 0.5) sawLowU = true;
      if (midU > 0.5) sawHighU = true;
    }
    expectTrue(sawLowU && sawHighU,
               "central-ridge: recovered BOTH the low-u and high-u fold-free sides");
    // The union recovers strictly more volume than the single largest side (the whole point).
    expectTrue(single.ok && totalVol > single.enclosedVolume + 1e-6,
               "central-ridge: multi-trim recovers more volume than single-side trim");

    // PASSTHROUGH: a small fold-free thicken yields a SINGLE full-domain solid, byte-identical
    // to thickenSurface (the multi path must not perturb the fold-free case).
    const std::vector<ThickenResult> gentle = thickenMultiTrimmed(S, 0.03, 1e-4, 16, 16);
    expectTrue(gentle.size() == 1 && gentle[0].ok && !gentle[0].trimmed,
               "central-ridge: fold-free thicken returns one full-domain solid");
    if (gentle.size() == 1) {
      const ThickenResult full = thickenSurface(S, 0.03, 1e-4, 16, 16);
      expectTrue(full.ok && meshBitEqual(gentle[0].solid, full.solid),
                 "central-ridge: passthrough solid byte-identical to thickenSurface");
    }

    // FULLY-FOLDING dome: no fold-free region → EMPTY vector (never a self-intersecting solid).
    const std::vector<ThickenResult> allFold = thickenMultiTrimmed(tightDome(0.5), -1.5 * 0.5,
                                                                   1e-3, 16, 16);
    expectTrue(allFold.empty(),
               "central-ridge: fully-folding thicken returns empty (honest-decline)");
  }

  // ═══ 7. FOLD-LOCUS TRIM — follow a DIAGONAL fold, beat the rectangle staircase ═══
  {
    // A diagonal ridge whose offset folds the diagonal band, leaving TWO TRIANGULAR fold-free
    // regions. thickenMultiTrimmed inscribes an axis-aligned rectangle in each triangle (dropping
    // the corner); thickenFoldTrim follows the diagonal fold and thickens the column-band that
    // hugs it — a strictly larger closed watertight solid on each side.
    const BsplineSurfaceData S = diagonalRidgePatch();
    // d chosen so the recovered column-band solid is genuinely SELF-INTERSECTION-FREE: the
    // node-wise offset fold guard (1 + d·κ) > 0 is necessary but not sufficient for the DISCRETE
    // offset PANEL to be embedding-free — at a larger |d| the offset cap over the near-fold band
    // edge can buckle between samples (a documented residual). |d| = 0.4 folds the diagonal band
    // (so the plain thicken still declines) while the retained fold-free band thickens cleanly.
    const double d = 0.4;      // toward the ridge's centre of curvature → fold the diagonal band
    const double tol = 1e-2;   // the warped-band fit floor near the high-curvature fold (honest)

    // Baseline: thickenSurface declines (fold); the axis-aligned staircase recovers rectangles.
    const ThickenResult base = thickenSurface(S, d, tol, 16, 16);
    expectTrue(!base.ok && base.status == ThickenStatus::SelfIntersection,
               "diagonal-fold: thickenSurface declines (SelfIntersection)");
    const std::vector<ThickenResult> stair = thickenMultiTrimmed(S, d, tol, 16, 16);
    double stairVol = 0.0;
    for (const ThickenResult& r : stair)
      if (r.ok) stairVol += r.enclosedVolume;

    // Fold-locus trim: >= 2 closed watertight solids, each following the diagonal fold.
    const std::vector<ThickenResult> fold = thickenFoldTrim(S, d, tol, 16, 16);
    expectTrue(fold.size() >= 2, "diagonal-fold: thickenFoldTrim recovers >= 2 solids");

    bool sawUL = false, sawLR = false;
    double foldVol = 0.0;
    for (std::size_t k = 0; k < fold.size(); ++k) {
      const ThickenResult& r = fold[k];
      char tag[64];
      std::snprintf(tag, sizeof tag, "diagonal-fold fold[%zu]", k);
      assertClosed(r, tag);  // ok, watertight, χ==2, zero boundary, oriented, positive volume
      if (!r.ok) continue;
      expectTrue(r.trimmed, "diagonal-fold: each recovered solid reports trimmed == true");
      // Each recovered solid is self-intersection-FREE (the diagonal fold band was cut out).
      char sbuf[80];
      std::snprintf(sbuf, sizeof sbuf, "diagonal-fold fold[%zu]: self-intersection-free", k);
      expectTrue(!selfIntersects(r.solid, 1e-6), sbuf);
      foldVol += r.enclosedVolume;
      const double midV = 0.5 * (r.keptV0 + r.keptV1);
      if (midV > 0.5) sawUL = true;
      if (midV < 0.5) sawLR = true;
    }
    expectTrue(sawUL && sawLR,
               "diagonal-fold: recovered BOTH the upper-left and lower-right triangles");

    // THE HEADLINE: the fold-locus trim recovers strictly MORE volume than the rectangle
    // staircase — following the diagonal fold beats the inscribed axis-aligned rectangles.
    expectTrue(foldVol > stairVol + 1e-3,
               "diagonal-fold: fold-locus thicken beats the rectangle staircase on volume");

    // PASSTHROUGH: a small fold-free thicken yields a SINGLE full-domain solid. (tol matches the
    // section: the gentle offset's fit floor is above 1e-4, so the passthrough uses the same 1e-2.)
    const std::vector<ThickenResult> gentle = thickenFoldTrim(S, 0.03, tol, 16, 16);
    expectTrue(gentle.size() == 1 && gentle[0].ok && !gentle[0].trimmed,
               "diagonal-fold: fold-free thicken returns one full-domain solid");

    // FULLY-FOLDING dome: no fold-free region → EMPTY vector (never a self-intersecting solid).
    const std::vector<ThickenResult> allFold = thickenFoldTrim(tightDome(0.5), -1.5 * 0.5,
                                                               1e-3, 16, 16);
    expectTrue(allFold.empty(),
               "diagonal-fold: fully-folding thicken returns empty (honest-decline)");
  }

  // ═══ 8. CURVED-ENVELOPE fold locus — a CLOSED fold loop, multi-band solids ═══════
  {
    // A central round dome whose thicken by d = 0.4 folds a CLOSED, genuinely CURVED
    // (circular) disk around the crest. The fold-free space is one component wrapping
    // around the disk (u-columns crossing it carry TWO fold-free v-runs). MEASURED BEFORE
    // the multi-band decomposition landed: thickenFoldTrim returned EMPTY (0 solids, 0
    // volume) while the rectangle staircase recovered only ~0.24 of the parameter area —
    // the documented curved-envelope residual. The scanline multi-band split must thicken
    // >= 4 simple bands around the disk (left/right + below/above), each a closed
    // watertight self-intersection-free solid tracing the curved fold boundary.
    // |d| = 0.4 keeps the discrete near-fold panels embedding-free (the same documented
    // large-|d| buckling bound as the diagonal section).
    const BsplineSurfaceData S = centralDomePatch();
    const double d = 0.4;      // toward the crest's centre of curvature → fold the crest disk
    const double tol = 1e-2;   // the warped-band fit floor near the high-curvature fold (honest)

    // Baseline: thickenSurface declines (fold); the staircase recovers only rectangles.
    const ThickenResult base = thickenSurface(S, d, tol, 16, 16);
    expectTrue(!base.ok && base.status == ThickenStatus::SelfIntersection,
               "curved-fold: thickenSurface declines (SelfIntersection)");
    const std::vector<ThickenResult> stair = thickenMultiTrimmed(S, d, tol, 16, 16);
    double stairVol = 0.0;
    for (const ThickenResult& r : stair)
      if (r.ok) stairVol += r.enclosedVolume;

    // Fold-locus trim: >= 4 closed watertight solids around the fold disk.
    const std::vector<ThickenResult> fold = thickenFoldTrim(S, d, tol, 16, 16);
    expectTrue(fold.size() >= 4, "curved-fold: thickenFoldTrim recovers >= 4 solids");

    bool sawLeft = false, sawRight = false, sawBelow = false, sawAbove = false;
    double foldVol = 0.0;
    for (std::size_t k = 0; k < fold.size(); ++k) {
      const ThickenResult& r = fold[k];
      char tag[64];
      std::snprintf(tag, sizeof tag, "curved-fold fold[%zu]", k);
      assertClosed(r, tag);  // ok, watertight, χ==2, zero boundary, oriented, positive volume
      if (!r.ok) continue;
      expectTrue(r.trimmed, "curved-fold: each recovered solid reports trimmed == true");
      // Each recovered solid is self-intersection-FREE (the curved fold disk was cut out).
      char sbuf[80];
      std::snprintf(sbuf, sizeof sbuf, "curved-fold fold[%zu]: self-intersection-free", k);
      expectTrue(!selfIntersects(r.solid, 1e-6), sbuf);
      foldVol += r.enclosedVolume;
      const double midU = 0.5 * (r.keptU0 + r.keptU1);
      const double midV = 0.5 * (r.keptV0 + r.keptV1);
      if (midU < 0.35) sawLeft = true;
      else if (midU > 0.65) sawRight = true;
      else if (midV < 0.5) sawBelow = true;
      else sawAbove = true;
    }
    expectTrue(sawLeft && sawRight && sawBelow && sawAbove,
               "curved-fold: solids cover ALL FOUR sides around the fold disk");

    // THE HEADLINE: the multi-band fold trim recovers FAR more volume than the rectangle
    // staircase (measured ~3.1x on this fixture) — and infinitely more than the previous
    // single-band trace, which declined this shape entirely.
    expectTrue(foldVol > stairVol + 1e-3,
               "curved-fold: fold-locus thicken beats the rectangle staircase on volume");
    expectTrue(foldVol > 2.0 * stairVol,
               "curved-fold: fold-locus thicken recovers >= 2x the staircase volume");

    // PASSTHROUGH: a small fold-free thicken yields a SINGLE full-domain solid.
    const std::vector<ThickenResult> gentle = thickenFoldTrim(S, 0.03, tol, 16, 16);
    expectTrue(gentle.size() == 1 && gentle[0].ok && !gentle[0].trimmed,
               "curved-fold: fold-free thicken returns one full-domain solid");
  }

  // ═══ 9. SPLIT/MERGE SEAM COLUMNS — bifurcating fold-free band, arm solids recovered ═══
  {
    // Two tall closed fold loops along u (see twinTallDomePatch): the wrap-around component's
    // bands split/merge at every loop edge, and the four ARMS (above/below of each loop) are
    // sound multi-column bands each below the old per-band 5% area bar. MEASURED BEFORE the
    // component-level gate: all four arms were dropped, so thickenFoldTrim built only 3 solids
    // (left/middle/right). The component-level gate must thicken all four arms too — each a
    // CLOSED, watertight, self-intersection-free solid. |d| = 0.4 keeps the discrete near-fold
    // panels embedding-free; the |d| = 0.6 BUCKLING control below must decline instead.
    const BsplineSurfaceData S = twinTallDomePatch();
    const double d = 0.4;      // toward the crests' centres of curvature → fold both loops
    const double tol = 1e-2;   // the warped-band fit floor near the high-curvature fold (honest)

    // Baseline: thickenSurface declines (fold); the staircase keeps a single small slab.
    const ThickenResult base = thickenSurface(S, d, tol, 16, 16);
    expectTrue(!base.ok && base.status == ThickenStatus::SelfIntersection,
               "seam-col: thickenSurface declines (SelfIntersection)");
    const std::vector<ThickenResult> stair = thickenMultiTrimmed(S, d, tol, 16, 16);
    double stairVol = 0.0;
    for (const ThickenResult& r : stair)
      if (r.ok) stairVol += r.enclosedVolume;

    // Fold-locus trim: left + middle + right solids PLUS all four bifurcation arm solids.
    const std::vector<ThickenResult> fold = thickenFoldTrim(S, d, tol, 16, 16);
    expectTrue(fold.size() >= 7,
               "seam-col: thickenFoldTrim recovers >= 7 solids (L/M/R + both arms, both loops)");

    // Loop 1 is centred at u ≈ 0.3, loop 2 at u ≈ 0.7. An ARM solid's kept box lies within one
    // loop's u-extent, on one v-side of it.
    bool arm1Below = false, arm1Above = false, arm2Below = false, arm2Above = false;
    double foldVol = 0.0;
    for (std::size_t k = 0; k < fold.size(); ++k) {
      const ThickenResult& r = fold[k];
      char tag[64];
      std::snprintf(tag, sizeof tag, "seam-col fold[%zu]", k);
      assertClosed(r, tag);  // ok, watertight, χ==2, zero boundary, oriented, positive volume
      if (!r.ok) continue;
      expectTrue(r.trimmed, "seam-col: each recovered solid reports trimmed == true");
      char sbuf[80];
      std::snprintf(sbuf, sizeof sbuf, "seam-col fold[%zu]: self-intersection-free", k);
      expectTrue(!selfIntersects(r.solid, 1e-6), sbuf);
      foldVol += r.enclosedVolume;
      const double midU = 0.5 * (r.keptU0 + r.keptU1);
      const double midV = 0.5 * (r.keptV0 + r.keptV1);
      const bool vSided = (r.keptV1 < 0.45) || (r.keptV0 > 0.55);
      if (midU > 0.15 && midU < 0.45 && vSided) ((midV < 0.5) ? arm1Below : arm1Above) = true;
      if (midU > 0.55 && midU < 0.85 && vSided) ((midV < 0.5) ? arm2Below : arm2Above) = true;
    }
    expectTrue(arm1Below && arm1Above,
               "seam-col: BOTH arm solids of the first bifurcation are recovered");
    expectTrue(arm2Below && arm2Above,
               "seam-col: BOTH arm solids of the second bifurcation are recovered");

    // THE HEADLINE: the fold-locus thicken beats the rectangle staircase on recovered volume.
    expectTrue(foldVol > stairVol + 1e-3,
               "seam-col: fold-locus thicken beats the rectangle staircase on volume");

    // CONTROL 1 — large-|d| near-fold panel BUCKLING must DECLINE, never leak: at d = 0.6 the
    // same fixture's band shells are watertight and χ = 2 yet SELF-PIERCING between samples
    // (the node-wise (1 + d·κ) guard cannot see it; measured: every band buckles, including
    // the left/middle/right bands the per-band gate used to emit as self-intersecting
    // solids). The discrete embedding guard must skip every buckled band → EMPTY.
    const std::vector<ThickenResult> buckled = thickenFoldTrim(S, 0.6, tol, 16, 16);
    for (std::size_t k = 0; k < buckled.size(); ++k) {
      char sbuf[96];
      std::snprintf(sbuf, sizeof sbuf, "seam-col buckling control[%zu]: never self-intersecting", k);
      expectTrue(!selfIntersects(buckled[k].solid, 1e-6), sbuf);
    }
    expectTrue(buckled.empty(),
               "seam-col: large-|d| buckling declines to empty (embedding guard, no SI leak)");

    // CONTROL 2 — what must decline still declines: the fully-folding tight dome's fold-free
    // slivers are whole components below the meaningful-area bar, so thickenFoldTrim returns
    // EMPTY (never a sliver solid, never a self-intersecting one).
    const BsplineSurfaceData ctrl = tightDome(0.5);
    const std::vector<ThickenResult> allFold = thickenFoldTrim(ctrl, 1.5 * 0.5, 1e-3, 16, 16);
    expectTrue(allFold.empty(),
               "seam-col: fully-folding thicken still returns empty (component gate declines)");

    // PASSTHROUGH: a small fold-free thicken yields a SINGLE full-domain solid.
    const std::vector<ThickenResult> gentle = thickenFoldTrim(S, 0.03, tol, 16, 16);
    expectTrue(gentle.size() == 1 && gentle[0].ok && !gentle[0].trimmed,
               "seam-col: fold-free thicken returns one full-domain solid");
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

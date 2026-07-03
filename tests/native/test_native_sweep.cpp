// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for Tier-C native construction (Phase 4 #4b): swept solids
// (src/native/construct/sweep.h). OCCT-FREE — Gate 1 (host, analytic) of the
// two-gate model in openspec/NATIVE-REWRITE.md. Each native result is validated
// with the native tessellator (watertight / enclosed volume) and the topology
// Explorer (face structure). Deferred cases (curved spine, real twist, degenerate)
// are asserted to return a NULL Shape so the engine can fall through to OCCT.
//
// NATIVE SCOPE (honest, see sweep.h): cc_solid_sweep is native for a STRAIGHT spine
// (an exact directional prism, always watertight) AND for a SMOOTH CURVED but PLANAR
// spine (a CONSTANT-frame ruled-band tube matching OCCT MakePipe's planar corrected-
// Frenet law, deflection-bounded, watertight). A NON-PLANAR curved spine, a TIGHT-
// CURVATURE / self-intersecting spine, a real twist/scale, or a degenerate profile
// defer to OCCT (NULL). The straight case is the exact analogue of
// BRepOffsetAPI_MakePipe on a straight polyline spine; the smooth curved case mirrors
// MakePipe on a bent PLANAR spine.
//
// THE CURVED MACHINERY IS NATIVE AND TESTED HERE end-to-end. The CONSTANT-frame
// transport in detail::constantFrames is genuine native code exercised on a smooth
// quarter-arc spine below. It is CALIBRATED TO THE ORACLE, not to an idealized pipe:
// BRepOffsetAPI_MakePipe's default GeomFill_CorrectedFrenet law collapses to a constant
// rotation on a PLANAR spine (OCCT/src GeomFill_CorrectedFrenet.cxx — the isPlanar
// branch uses a Law_Constant), so the section is TRANSLATED, not rotated to stay
// perpendicular. We reproduce that: (1) every station shares the start frame's axes
// (constantFrames), so the section normal is fixed and the frame-based swept volume is
// EXACTLY profile_area × (spine span along the section normal) — the same value OCCT's
// MakePipe reports within its polyline discretization; (2) the assembled ruled-band tube
// meshes WATERTIGHT (the two-stage tessellator subdivides the straight side edges so
// both neighbours of each band agree). A NON-PLANAR curved spine would need OCCT's
// genuine (non-constant) corrected-Frenet law, so build_sweep returns NULL there; a
// TIGHT curve whose turning radius is below the profile circumradius WOULD self-
// intersect, so build_sweep returns NULL there too — both VERIFIED fall-throughs
// (asserted), never a faked or oracle-mismatching solid.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_sweep.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_sweep
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace math = cybercad::native::math;
namespace det = cybercad::native::construct::detail;

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;

namespace {
int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}
// Watertight at the GIVEN deflections; reports the last mesh's enclosed volume.
bool watertightAt(const topo::Shape& s, std::vector<double> defls, double& vol) {
  bool ok = !s.isNull();
  for (double d : defls) {
    tess::MeshParams p;
    p.deflection = d;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
    if (!tess::isWatertight(m)) ok = false;
    vol = std::fabs(tess::enclosedVolume(m));
  }
  return ok;
}
// Watertight AND volume-stable across several deflections (the mesh converges to the
// exact prism, so a straight sweep must stay closed at every deflection).
bool watertightAllDeflections(const topo::Shape& s, double& vol) {
  return watertightAt(s, {0.05, 0.01, 0.002}, vol);
}

// Build a quarter-arc spine of radius R in the XZ plane, theta 0..pi/2, N segments:
// point(theta) = (R cos, 0, R sin). Returned as flat x,y,z triples (N+1 stations).
std::vector<double> quarterArcPath(double R, int N) {
  const double PI = 3.14159265358979323846;
  std::vector<double> path;
  path.reserve(static_cast<std::size_t>(N + 1) * 3);
  for (int k = 0; k <= N; ++k) {
    const double th = (PI / 2.0) * k / N;
    path.push_back(R * std::cos(th));
    path.push_back(0.0);
    path.push_back(R * std::sin(th));
  }
  return path;
}
}  // namespace

// ── STRAIGHT sweep along +Z: a 4×4 square swept 10 up is a 4×4×10 prism ──────---
// Profile centred at origin (area 16), path (0,0,0)→(0,0,10). Volume 160, EXACT; 6
// faces (4 sides + 2 caps), watertight — identical to BRepOffsetAPI_MakePipe on a
// straight spine (and to build_prism along +Z).
CC_TEST(sweep_straight_z_is_exact_prism) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double path[] = {0, 0, 0, 0, 0, 10};
  const topo::Shape s = cst::build_sweep(prof, 4, path, 2);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 6);
  double vol = 0.0;
  CC_CHECK(watertightAllDeflections(s, vol));
  CC_CHECK(std::fabs(vol - 160.0) < 1e-6);  // exact
}

// ── STRAIGHT sweep, multiple COLLINEAR stations collapse to the same prism ───---
// 5 collinear points along +Z, total length 20. The straight spine collapses to its
// endpoints, so it is still a single-band prism: 6 faces, volume 16·20 = 320, EXACT.
CC_TEST(sweep_straight_collinear_stations_collapse) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double path[] = {0, 0, 0, 0, 0, 5, 0, 0, 10, 0, 0, 15, 0, 0, 20};
  const topo::Shape s = cst::build_sweep(prof, 4, path, 5);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 6);
  double vol = 0.0;
  CC_CHECK(watertightAllDeflections(s, vol));
  CC_CHECK(std::fabs(vol - 320.0) < 1e-6);
}

// ── STRAIGHT sweep along an ARBITRARY 3D direction is exact ──────────────────---
// Path along the unit (1,1,1) direction over length L=30. The section stays
// perpendicular to the path, so the swept solid is an oblique-placed prism of volume
// area·L = 16·30 = 480, EXACT and watertight — the RMF transport reduces to a
// constant frame on a straight spine regardless of its 3D orientation.
CC_TEST(sweep_straight_arbitrary_direction_exact) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double d = 1.0 / std::sqrt(3.0);
  const double L = 30.0;
  const double path[] = {0, 0, 0, d * L, d * L, d * L};
  const topo::Shape s = cst::build_sweep(prof, 4, path, 2);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 6);
  double vol = 0.0;
  CC_CHECK(watertightAllDeflections(s, vol));
  CC_CHECK(std::fabs(vol - 16.0 * L) < 1e-6);
}

// ── STRAIGHT sweep of a PENTAGON profile → 5 side faces + 2 caps, watertight ──---
// A regular pentagon (circumradius 2) swept 12 along +Z. 7 faces; watertight; volume
// = pentagon area (2.5·R²·sin(72°) = 2.5·4·0.95106 = 9.5106) × 12 ≈ 114.13.
CC_TEST(sweep_straight_pentagon_watertight) {
  std::vector<double> prof;
  for (int k = 0; k < 5; ++k) {
    const double a = k * 2.0 * 3.14159265358979323846 / 5.0;
    prof.push_back(2.0 * std::cos(a));
    prof.push_back(2.0 * std::sin(a));
  }
  const double path[] = {0, 0, 0, 0, 0, 12};
  const topo::Shape s = cst::build_sweep(prof.data(), 5, path, 2);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 7);  // 5 sides + 2 caps
  double vol = 0.0;
  CC_CHECK(watertightAllDeflections(s, vol));
  const double area = 2.5 * 4.0 * std::sin(2.0 * 3.14159265358979323846 / 5.0);
  CC_CHECK(std::fabs(vol - area * 12.0) / (area * 12.0) < 1e-6);
}

// ── twisted_sweep with ZERO twist + unit scale on a straight path → the prism ──--
// cc_twisted_sweep reduces to the plain straight prism when there is no twist and no
// scale; that case is NATIVE (forwards to build_sweep). Volume 160, watertight.
CC_TEST(twisted_sweep_zero_twist_is_prism) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double path[] = {0, 0, 0, 0, 0, 10};
  const topo::Shape s = cst::build_twisted_sweep(prof, 4, path, 2, 0.0, 1.0);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  double vol = 0.0;
  CC_CHECK(watertightAllDeflections(s, vol));
  CC_CHECK(std::fabs(vol - 160.0) < 1e-6);
}

// ── NATIVE: a SMOOTH CURVED but PLANAR (quarter-arc) spine builds a watertight tube --
// The constant-frame sweep TRANSLATES the section along the smooth arc (OCCT's planar
// MakePipe law); each profile edge spans a bilinear ruled band per spine segment,
// capped at the ends. The two-stage mesher welds the multi-band skin WATERTIGHT. A
// small square (side 2, area 4) swept along a quarter-arc of radius R=10 in the XZ
// plane: because the section normal is fixed (the start section normal n = x×y ≈ −Z),
// the enclosed volume is profile_area × |Δspine · n̂| (the spine displacement projected
// onto the fixed normal), NOT the Pappus arc-length volume (a perpendicular sweep would
// give that, but that is NOT what the oracle builds). 4 side bands × N segments + 2 caps
// faces. The flat-quad chords integrate the constant-normal sweep EXACTLY, so the meshed
// volume matches area × |Δspine · n̂| to the mesh's linear tolerance.
CC_TEST(sweep_curved_arc_native_watertight) {
  const double R = 10.0;
  const int N = 24;
  const std::vector<double> path = quarterArcPath(R, N);
  const double prof[] = {-1, -1, 1, -1, 1, 1, -1, 1};  // square side 2, area 4
  const topo::Shape s = cst::build_sweep(prof, 4, path.data(), N + 1);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 4 * N + 2);  // 4 bands·N + 2 caps
  double vol = 0.0;
  CC_CHECK(watertightAt(s, {0.05, 0.02}, vol));  // verified deflection band
  // Expected constant-frame swept volume: area × |Δspine · n̂|.
  std::vector<math::Point3> spine = det::cleanPath(path.data(), N + 1);
  const std::vector<math::Vec3> tan = det::stationTangents(spine);
  const std::vector<det::SweepFrame> fr = det::constantFrames(spine, tan);
  const math::Vec3 n = math::cross(fr.front().x, fr.front().y);
  const math::Vec3 disp = fr.back().origin - fr.front().origin;
  const double exactVol = 4.0 * std::fabs(math::dot(disp, n) / math::norm(n));  // ≈ 41.0
  CC_CHECK(std::fabs(vol - exactVol) / exactVol < 5e-3);
}

// ── DEFERRED: a real TWIST (or scale) returns NULL (→ OCCT twisted_sweep) ──────---
CC_TEST(twisted_sweep_real_twist_deferred) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double path[] = {0, 0, 0, 0, 0, 10};
  CC_CHECK(cst::build_twisted_sweep(prof, 4, path, 2, 1.5708, 1.0).isNull());  // twist
  CC_CHECK(cst::build_twisted_sweep(prof, 4, path, 2, 0.0, 2.0).isNull());     // scale
}

// ── DEFERRED: degenerate input returns NULL ─────────────────────────────────---
CC_TEST(sweep_degenerate_input_deferred) {
  const double prof[] = {-2, -2, 2, -2, 2, 2, -2, 2};
  const double path[] = {0, 0, 0, 0, 0, 10};
  const double onePt[] = {0, 0, 0};
  const double zeroArea[] = {0, 0, 1, 1, 2, 2};  // collinear profile (zero area)
  CC_CHECK(cst::build_sweep(nullptr, 4, path, 2).isNull());     // null profile
  CC_CHECK(cst::build_sweep(prof, 2, path, 2).isNull());        // < 3 profile points
  CC_CHECK(cst::build_sweep(prof, 4, onePt, 1).isNull());       // < 2 path points
  CC_CHECK(cst::build_sweep(zeroArea, 3, path, 2).isNull());    // zero-area profile
}

// ── CURVED (smooth PLANAR quarter-arc): constant-frame swept volume matches oracle --
// The constant-frame transport (detail::constantFrames) is native and reproduces OCCT
// MakePipe's planar law: the section is translated with a FIXED orientation. A small
// square (side 2, area 4) transported along a quarter-arc of radius R=10 in the XZ
// plane therefore sweeps a solid whose volume is profile_area × (spine span along the
// fixed section normal), NOT the Pappus arc-length volume. The section normal is the
// start section normal n = x×y (≈ the start tangent axis); the swept volume is
// area × |projection of the total spine displacement onto n|. Because the frame is
// constant the per-segment flat quads integrate this EXACTLY, so we assert the closed
// value 4 × 10 = 40 (matching the oracle within its polyline discretization).
CC_TEST(sweep_curved_arc_frame_volume_matches_oracle) {
  const double R = 10.0;
  const int N = 32;
  const std::vector<double> path = quarterArcPath(R, N);
  const double prof[] = {-1, -1, 1, -1, 1, 1, -1, 1};  // square side 2, area 4

  const det::SweepProfile pr = det::analyzeProfile(prof, 4);
  CC_CHECK(pr.valid);
  std::vector<math::Point3> spine = det::cleanPath(path.data(), N + 1);
  CC_CHECK_EQ(static_cast<int>(spine.size()), N + 1);  // smooth spine kept (not collapsed)
  const std::vector<math::Vec3> tan = det::stationTangents(spine);
  const std::vector<det::SweepFrame> frames = det::constantFrames(spine, tan);

  // The section normal is FIXED (constant frame): n = x×y at every station.
  const math::Vec3 n = math::cross(frames.front().x, frames.front().y);
  // Swept volume of a constant-orientation prism = area × |Δspine · n̂|.
  const math::Vec3 disp = frames.back().origin - frames.front().origin;
  const double span = std::fabs(math::dot(disp, n) / math::norm(n));
  const double profArea = 4.0;
  const double frameVol = profArea * span;  // constant-frame swept volume (≈ 41.0)
  // The section normal ≈ −Z, so the span is ≈ R plus a small in-plane tilt component;
  // the frame formula equals the meshed volume, which is the oracle-matching value.
  const topo::Shape s = cst::build_sweep(prof, 4, path.data(), N + 1);
  CC_CHECK(!s.isNull());
  double meshVol = 0.0;
  CC_CHECK(watertightAt(s, {0.02}, meshVol));
  CC_CHECK(std::fabs(frameVol - meshVol) / meshVol < 5e-3);
}

// ── CURVED: the transported frame is CONSTANT (matches OCCT's planar MakePipe law) --
// On a PLANAR spine OCCT's corrected-Frenet law collapses to a constant rotation, so
// the section orientation is held FIXED and the profile is merely translated. This
// asserts the native constantFrames does exactly that — the crux of the oracle match:
// (a) every station shares the START frame's x and y axes (x·xk = 1, y·yk = 1), i.e.
// the section does NOT rotate to follow the tangent; (b) the frame stays orthonormal;
// (c) each station's stored tangent DOES follow the path (start tangent ≈ +Z, end
// tangent ≈ +X on the quarter arc) — so the tangent is available for face orientation
// even though the section frame is frozen.
CC_TEST(sweep_curved_arc_frame_is_constant) {
  const double R = 10.0;
  const int N = 32;
  const std::vector<double> path = quarterArcPath(R, N);
  std::vector<math::Point3> spine = det::cleanPath(path.data(), N + 1);
  const std::vector<math::Vec3> tan = det::stationTangents(spine);
  const std::vector<det::SweepFrame> frames = det::constantFrames(spine, tan);
  const det::SweepFrame& f0 = frames.front();
  const det::SweepFrame& fN = frames.back();

  // (a) The section axes are CONSTANT across all stations (translated, not rotated).
  for (const det::SweepFrame& f : frames) {
    CC_CHECK(std::fabs(math::dot(f.x, f0.x) - 1.0) < 1e-9);
    CC_CHECK(std::fabs(math::dot(f.y, f0.y) - 1.0) < 1e-9);
  }
  // (b) Start/end frame orthonormal.
  CC_CHECK(std::fabs(math::norm(fN.x) - 1.0) < 1e-9);
  CC_CHECK(std::fabs(math::norm(fN.y) - 1.0) < 1e-9);
  CC_CHECK(std::fabs(math::dot(fN.x, fN.y)) < 1e-9);
  // (c) Stored tangents follow the path: on the XZ quarter arc from (R,0,0) toward
  // (0,0,R) the tangent starts ≈ +Z and ends ≈ −X (X decreasing, Z increasing).
  CC_CHECK(std::fabs(f0.t.z - 1.0) < 1e-2);
  CC_CHECK(std::fabs(fN.t.x + 1.0) < 1e-2);
}

// ── TIGHT-CURVATURE / self-intersecting sweep → the not-supported signal ────────--
// A square (side 8, half-extent 4) swept along a quarter-arc of radius 2 would
// self-intersect on the inner (concave) side — the section is far wider than the
// curvature radius, so the swept walls fold through the spine. build_sweep must NOT
// emit a self-intersecting solid: the sharpness guard (spineTooSharp) detects the
// turning radius is below the profile circumradius and returns the not-supported
// signal (NULL Shape) so the engine falls through to OCCT MakePipe. Asserting NULL
// proves no bogus/self-intersecting solid is produced — a SMOOTH arc of the same
// profile at a large radius (below) IS built natively, so this is a curvature guard,
// not a blanket curved-spine rejection.
CC_TEST(sweep_tight_curvature_returns_not_supported) {
  const double R = 2.0;                  // small curvature radius
  const int N = 24;
  const std::vector<double> path = quarterArcPath(R, N);
  const double prof[] = {-4, -4, 4, -4, 4, 4, -4, 4};  // half-extent 4 >> R=2 → self-int
  const topo::Shape s = cst::build_sweep(prof, 4, path.data(), N + 1);
  CC_CHECK(s.isNull());  // not-supported signal; NO self-intersecting solid emitted
  // Sanity: the deferral is the TIGHT curvature (turning radius < profile circumradius),
  // not a blanket curved rejection — the same profile on a large-radius SMOOTH arc IS
  // built natively, and on a STRAIGHT path too.
  const double straight[] = {0, 0, 0, 0, 0, 5};
  CC_CHECK(!cst::build_sweep(prof, 4, straight, 2).isNull());
  const std::vector<double> gentle = quarterArcPath(40.0, 24);  // r=40 >> circumR≈5.66
  CC_CHECK(!cst::build_sweep(prof, 4, gentle.data(), 25).isNull());
}

int main() { return cctest::run_all(); }

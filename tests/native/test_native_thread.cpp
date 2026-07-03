// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for Tier-D native construction (Phase 4 #4b): helical threads +
// tapered shank (src/native/construct/thread.h). OCCT-FREE — Gate 1 (host, analytic)
// of the two-gate model in openspec/NATIVE-REWRITE.md. Each native result is validated
// with the native tessellator (watertight / enclosed volume) and the topology Explorer
// (face structure). Deferred / degenerate cases are asserted to return a NULL Shape so
// the engine can fall through to OCCT.
//
// NATIVE SCOPE (honest, see thread.h §HONESTY):
//   * tapered_shank IS genuinely native — a shank silhouette revolved 360° about Z
//     (reusing the native revolve): full radius over fullHeight, tapering to a TRUE
//     point over taperHeight. Watertight at every deflection; volume = cone tip +
//     full-radius cylinder, matching BRepPrimAPI_MakeRevol.
//   * helical_thread / tapered_thread ATTEMPT the radial-V helical tiling (a V section
//     swept radially via the axis-aux-spine law, tiled into ruled bands + planar caps,
//     guarded against self-intersection). The builder returns a candidate solid with
//     the CORRECT volume and V geometry, but its per-turn ruled-band ↔ cap seams do NOT
//     weld ROBUSTLY watertight across deflections on the current tessellator, so the
//     ENGINE self-verify (robustlyWatertight) defers them to OCCT. These host tests
//     therefore assert (a) the guards reject degenerate + self-intersecting input with
//     a NULL Shape, and (b) the candidate solid, WHEN built, carries the right V-tiling
//     face structure and the correct enclosed volume — documenting the native attempt
//     without claiming a robustly-watertight thread it does not yet produce.
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_thread.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_thread
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace math = cybercad::native::math;

namespace {
constexpr double kPi = 3.14159265358979323846;

int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}

// Watertight at EVERY deflection in the ladder; reports the last mesh's enclosed
// volume. This is the same robustness bar the engine's self-verify applies.
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

// The enclosed volume at a single (tight) deflection — used to check the thread
// candidate's volume even when its seams do not weld watertight.
double volumeAt(const topo::Shape& s, double defl) {
  tess::MeshParams p;
  p.deflection = defl;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  return std::fabs(tess::enclosedVolume(m));
}

// Axis-aligned bounding box of a shape's mesh at a given deflection. Used to check
// gross geometry (shank apex/head, thread major-radius + Z-extent) directly from the
// native tessellation — no OCCT.
struct Bbox {
  double xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
  bool empty = true;
};

Bbox bboxAt(const topo::Shape& s, double defl) {
  tess::MeshParams p;
  p.deflection = defl;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  Bbox b;
  for (const math::Point3& v : m.vertices) {
    if (b.empty) {
      b.xmin = b.xmax = v.x;
      b.ymin = b.ymax = v.y;
      b.zmin = b.zmax = v.z;
      b.empty = false;
      continue;
    }
    b.xmin = std::min(b.xmin, v.x);
    b.ymin = std::min(b.ymin, v.y);
    b.zmin = std::min(b.zmin, v.z);
    b.xmax = std::max(b.xmax, v.x);
    b.ymax = std::max(b.ymax, v.y);
    b.zmax = std::max(b.zmax, v.z);
  }
  return b;
}
}  // namespace

// ── NATIVE: tapered_shank is a watertight revolved silhouette ────────────────────
// r=5, fullHeight=20, taperHeight=10, ppm=1 ⇒ zTaper=10, zTop=30. The silhouette is a
// cone from the axis point (0,0) out to (5,10), a cylinder (5,10)→(5,30), and the head
// disk (5,30)→(0,30). Faces: 3 non-degenerate segments × 3 (120° spans on a full turn)
// = 9. Volume = cone ⅓π·5²·10 + cylinder π·5²·20 = 261.80 + 1570.80 = 1832.6, watertight
// at every deflection (the tip is a TRUE on-axis apex the revolve collapses to one
// shared vertex, so no sliver breaks the weld). build_tapered_shank revolves about the
// WORLD Z axis (explicit Z frame, to match the OCCT BRepPrimAPI_MakeRevol oracle exactly),
// so the axial height maps to world +Z (tip z=0, head z=30) and the radius to X/Y.
CC_TEST(tapered_shank_is_watertight_revolve) {
  const topo::Shape s = cst::build_tapered_shank(5.0, 20.0, 10.0, 1.0);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 9);  // 3 segments × 3 spans
  double vol = 0.0;
  CC_CHECK(watertightAt(s, {0.05, 0.02, 0.01, 0.005}, vol));  // robustly closed
  const double cone = kPi * 5.0 * 5.0 * 10.0 / 3.0;
  const double cyl = kPi * 5.0 * 5.0 * 20.0;
  const double exact = cone + cyl;  // ≈ 1832.6
  CC_CHECK(std::fabs(vol - exact) / exact < 2e-2);  // deflection-bounded convergence

  // Gross geometry: wide at the top (head), a point at the bottom (tip). The shank is
  // revolved about the WORLD Z axis (build_tapered_shank uses an explicit Z frame to
  // match the OCCT BRepPrimAPI_MakeRevol oracle exactly), so the AXIAL height runs along
  // +Z (tip z=0 → head z=zTop=30) and the radial extent is in X/Y (≈ ±r=5). Wide at the
  // top / point at the bottom = the bbox spans z∈[0,30] with radial ±5.
  const Bbox b = bboxAt(s, 0.01);
  CC_CHECK(std::fabs(b.zmin - 0.0) < 1e-6);   // tip (axial 0)
  CC_CHECK(std::fabs(b.zmax - 30.0) < 1e-6);  // head (axial zTop)
  const double rMax = std::max({-b.xmin, b.xmax, -b.ymin, b.ymax});
  CC_CHECK(std::fabs(rMax - 5.0) / 5.0 < 2e-2);  // head radius ≈ r=5
  // The apex vertex is on the axis at the tip (z≈0) — the section at the very bottom
  // collapses to a point, so among near-tip vertices the radial extent is ~0.
  double tipR = 1e9;
  tess::MeshParams tp;
  tp.deflection = 0.01;
  for (const math::Point3& v : tess::SolidMesher{tp}.mesh(s).vertices) {
    if (v.z < 0.5) tipR = std::min(tipR, std::sqrt(v.x * v.x + v.y * v.y));
  }
  CC_CHECK(tipR < 0.2);  // a genuine on-axis apex at the tip
}

// ── NATIVE: tapered_shank scales with pointsPerMM (volume ∝ ppm³) ────────────────
// Doubling ppm scales every linear dimension ×2 ⇒ the volume ×8. Watertight at ppm=2.
CC_TEST(tapered_shank_scales_with_ppm) {
  const topo::Shape s1 = cst::build_tapered_shank(5.0, 20.0, 10.0, 1.0);
  const topo::Shape s2 = cst::build_tapered_shank(5.0, 20.0, 10.0, 2.0);
  CC_CHECK(!s1.isNull());
  CC_CHECK(!s2.isNull());
  if (s1.isNull() || s2.isNull()) return;
  double v1 = 0.0, v2 = 0.0;
  CC_CHECK(watertightAt(s1, {0.02}, v1));
  CC_CHECK(watertightAt(s2, {0.02}, v2));
  CC_CHECK(std::fabs(v2 / v1 - 8.0) / 8.0 < 3e-2);
}

// ── DEFERRED: degenerate shank parameters return NULL ────────────────────────────
CC_TEST(tapered_shank_degenerate_deferred) {
  CC_CHECK(cst::build_tapered_shank(0.0, 20.0, 10.0, 1.0).isNull());   // r ≤ 0
  CC_CHECK(cst::build_tapered_shank(5.0, 0.0, 10.0, 1.0).isNull());    // fullHeight ≤ 0
  CC_CHECK(cst::build_tapered_shank(5.0, 20.0, 0.0, 1.0).isNull());    // taperHeight ≤ 0
  CC_CHECK(cst::build_tapered_shank(5.0, 20.0, 10.0, 0.0).isNull());   // ppm ≤ 0
}

// ── ATTEMPT: the helical-thread candidate carries the right V-tiling + volume ────
// A cylindrical V thread (major=10, pitch=3, turns=2, depth=1.5, flank=60°, ppm=1,
// spt=12). The radial-V tiling builds 3 ruled bands per span × (turns·spt) spans + 2
// caps. The pitch-line radius is major − depth/2 = 9.25; the V apex reaches the major
// radius 10 and the root sits at 8.5. This asserts the native builder returns a
// candidate (non-null) with the expected face structure and a plausible, positive
// enclosed volume — the native radial-V machinery IS exercised. It does NOT assert
// robust watertightness: the per-turn seams do not weld robustly on the current mesher,
// which is why the ENGINE self-verify defers this op to OCCT (see thread.h §HONESTY).
CC_TEST(helical_thread_candidate_has_v_tiling) {
  const int turns = 2, spt = 12;
  const topo::Shape s = cst::build_helical_thread(10.0, 3.0, turns, 1.5, 60.0, 1.0, spt);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  const int stations = turns * spt;             // spans between station rings
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 3 * stations + 2);  // 3 bands/span + 2 caps
  const double vol = volumeAt(s, 0.02);
  CC_CHECK(vol > 0.0);  // a real, positively-enclosed candidate solid

  // Gross geometry of the candidate (checked from the native mesh, no OCCT). The thread
  // is swept about the Z axis. Radial extent: the V apex is projected outward by `depth`
  // from the pitch-line radius (major−depth/2 = 9.25), so the outermost radius is
  // pitchR+depth = major+depth/2 = 10.75 (the bbox radius ≈ that). Z-extent: the helix
  // rises pitch·turns = 6 and the V section overhangs one half-base beyond each end
  // (halfBase = min(pitch/2, depth·tan(30°)) = min(1.5, 0.866) = 0.866), so ≈ 6 + 2·0.866.
  const double pitchR = 10.0 - 1.5 / 2.0;   // major − depth/2 = 9.25
  const double apexR = pitchR + 1.5;        // 10.75
  const Bbox b = bboxAt(s, 0.02);
  const double rMax = std::max({-b.xmin, b.xmax, -b.ymin, b.ymax});
  CC_CHECK(std::fabs(rMax - apexR) / apexR < 5e-2);  // apex reaches major + depth/2
  CC_CHECK(rMax > 10.0);                              // and clears the requested major radius
  const double rise = 3.0 * turns;                   // pitch·turns = 6
  const double zExtent = b.zmax - b.zmin;
  CC_CHECK(zExtent > rise * 0.95);                   // spans at least the requested turns
  CC_CHECK(zExtent < rise + 3.0);                    // + at most one V base of overhang/end
}

// ── ATTEMPT: the tapered-thread candidate is built (tapering pitch-line radius) ──
// top=6, tip=4, pitch=2, turns=3, depth=1, flank=60°, spt=16. The pitch-line radius
// tapers 3.5 (tip) → 5.5 (top). Candidate non-null with the V-tiling face count.
CC_TEST(tapered_thread_candidate_is_built) {
  const int turns = 3, spt = 16;
  const topo::Shape s = cst::build_tapered_thread(6.0, 4.0, 2.0, turns, 1.0, 60.0, 1.0, spt);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  const int stations = turns * spt;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 3 * stations + 2);
  CC_CHECK(volumeAt(s, 0.02) > 0.0);
}

// ── GUARD: degenerate thread parameters return NULL (→ OCCT) ─────────────────────
CC_TEST(thread_degenerate_params_deferred) {
  CC_CHECK(cst::build_helical_thread(5.0, 2.0, 4.0, 1.0, 200.0, 1.0, 16).isNull());  // flank ≥ 180
  CC_CHECK(cst::build_helical_thread(5.0, 2.0, 0.0, 1.0, 60.0, 1.0, 16).isNull());   // turns ≤ 0
  CC_CHECK(cst::build_helical_thread(5.0, 0.0, 4.0, 1.0, 60.0, 1.0, 16).isNull());   // pitch ≤ 0
  CC_CHECK(cst::build_helical_thread(0.0, 2.0, 4.0, 1.0, 60.0, 1.0, 16).isNull());   // major ≤ 0
  CC_CHECK(cst::build_helical_thread(5.0, 2.0, 4.0, 0.0, 60.0, 1.0, 16).isNull());   // depth ≤ 0
}

// ── GUARD: pitch-line radius must clear the axis (deep V on a thin thread) ────────
// A depth so large the pitch-line radius (major − depth/2) goes non-positive must
// defer, never emit a self-intersecting solid. major=1, depth=4 ⇒ pitchR = 1−2 = −1.
CC_TEST(thread_pitchradius_below_axis_deferred) {
  CC_CHECK(cst::build_helical_thread(1.0, 2.0, 4.0, 4.0, 60.0, 1.0, 16).isNull());
}

// ── GUARD: a tapered thread whose TIP end dives below the axis defers ─────────────
// tip=1, depth=3 ⇒ tip pitchR = 1 − 1.5 = −0.5 (< axis clearance) → NULL.
CC_TEST(tapered_thread_tip_below_axis_deferred) {
  CC_CHECK(cst::build_tapered_thread(6.0, 1.0, 2.0, 3.0, 3.0, 60.0, 1.0, 16).isNull());
}

// ── GUARD: a deliberately self-intersecting FINE-PITCH thread yields the not-supported
// signal (no self-intersecting solid ships native) ──────────────────────────────
// A very fine pitch (0.2) with a DEEP V (depth 3) on a thin thread (major 2): the axial
// V half-base is capped at pitch/2 = 0.1, so adjacent turns are only 0.2 apart while the
// radial V sweeps 3 units — the turns fold through each other (a self-intersecting body).
// The cheap parameter guard alone cannot detect this fold, so the builder may return a
// non-null CANDIDATE; the honest not-supported signal is the ENGINE self-verify
// (robustlyWatertight — closed at EVERY deflection). This asserts that self-verify FAILS
// on the fold, so the op falls through to OCCT rather than emitting a self-intersecting
// solid. `watertightAt` runs the exact same deflection ladder native_engine.cpp uses.
CC_TEST(fine_pitch_self_intersecting_thread_not_supported) {
  const topo::Shape s = cst::build_helical_thread(2.0, 0.2, 4.0, 3.0, 60.0, 1.0, 16);
  double vol = 0.0;
  const bool robust = watertightAt(s, {0.05, 0.02, 0.01, 0.005}, vol);
  CC_CHECK(!robust);  // NOT robustly watertight ⇒ engine defers to OCCT (not faked)
}

int main() { return cctest::run_all(); }

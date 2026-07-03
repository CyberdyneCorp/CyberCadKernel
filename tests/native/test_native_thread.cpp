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
//   * helical_thread / tapered_thread build the radial-V helical tiling (a V section
//     swept radially via the axis-aux-spine law, tiled into ruled bands + planar caps,
//     guarded against self-intersection) and now mesh ROBUSTLY WATERTIGHT: the per-turn
//     ruled-band ↔ band and band ↔ cap straight seams weld exactly because the mesher
//     emits build-order-independent CANONICAL seam points and snaps both faces' seam
//     vertices to them (edge_mesher CanonicalEndpoints / face_mesher BoundaryAnchors), so
//     the two coincident points are BIT-IDENTICAL and the conservative single-cell weld
//     fuses them at EVERY deflection. These host tests therefore assert (a) the guards
//     reject degenerate + self-intersecting input, and (b) a well-formed thread — both a
//     cylindrical (reference: major5 / pitch2 / turns4 / depth1) and a tapered one — is a
//     HARD REQUIREMENT to be WATERTIGHT (boundaryEdges==0) at EVERY deflection in a ladder
//     {0.1, 0.05, 0.02, 0.01}, not just one, with the right V-tiling face structure, a
//     positive enclosed-volume sign and the correct turn count. This is the same
//     robustness bar native_engine.cpp robustlyWatertight enforces before keeping the op
//     native; there is NO "candidate only / self-verify defers" allowance for these
//     well-formed cases — a leak at any single deflection FAILS the test. It is the
//     regression test for the seam-weld fix. Only a FINE-PITCH / self-intersecting thread
//     (turns fold through each other) still fails the ladder and falls through to OCCT
//     (guard unchanged).
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
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
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

// HARD watertight requirement across a deflection LADDER: assert boundaryEdges==0
// (fully closed 2-manifold) at EVERY rung, individually, so a leak at any single
// deflection fails the test with the offending deflection reported — not just the
// aggregate `watertightAt` bool. This is the bar the engine's robustlyWatertight
// self-verify enforces before keeping a thread native; the well-formed thread ops
// MUST clear it (no "candidate only / self-verify defers" allowance).
void requireWatertightLadder(bool& cc_ok_, const topo::Shape& s,
                             const std::vector<double>& defls) {
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  for (double d : defls) {
    tess::MeshParams p;
    p.deflection = d;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
    const std::size_t be = tess::boundaryEdgeCount(m);
    // boundaryEdges==0 AND a closed 2-manifold (no edge used 3+ times): a
    // self-intersecting fold can zero the open-boundary count yet still be
    // non-manifold, so both must hold for a genuinely watertight solid.
    CC_CHECK(be == 0);
    CC_CHECK(tess::isWatertight(m));
    if (be != 0 || !tess::isWatertight(m))
      std::printf("  [thread] NOT watertight at deflection=%.4f (boundaryEdges=%zu)\n", d, be);
  }
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

// The deflection LADDER every well-formed thread must be watertight across. This is
// the multi-deflection bar the engine's robustlyWatertight self-verify applies (its
// own rungs are {0.05,0.02,0.01,0.005}); we add the coarser 0.1 rung so a seam that
// only opens at low resolution is caught too. boundaryEdges MUST be 0 at every rung.
const std::vector<double> kThreadDeflLadder = {0.1, 0.05, 0.02, 0.01};

// ── NATIVE (HARD REQUIREMENT): a well-formed helical thread is WATERTIGHT at every
// deflection, with the right V-tiling + volume sign + turn count ──────────────────
// A cylindrical V thread with the reference parameters (major=5, pitch=2, turns=4,
// depth=1, flank=60°, ppm=1, spt=12). The radial-V tiling builds 3 ruled bands per
// span × (turns·spt) spans + 2 caps. The pitch-line radius is major − depth/2 = 4.5;
// the V apex reaches the major radius 5 and the root sits at 4.
//
// This is the seam-weld REGRESSION and a HARD gate (no "candidate only / self-verify
// defers" allowance for a well-formed thread): the per-turn ruled-band ↔ band and
// band ↔ cap straight seams used to open at isolated deflections; they now weld via
// the mesher's canonical shared-edge points (edge_mesher CanonicalEndpoints /
// face_mesher BoundaryAnchors), so the thread meshes boundaryEdges==0 at EVERY rung of
// the ladder and the ENGINE keeps this op native.
CC_TEST(helical_thread_is_watertight_across_ladder) {
  const int turns = 4, spt = 12;
  const topo::Shape s = cst::build_helical_thread(5.0, 2.0, turns, 1.0, 60.0, 1.0, spt);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  const int stations = turns * spt;             // spans between station rings
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 3 * stations + 2);  // 3 bands/span + 2 caps

  // HARD watertight requirement: boundaryEdges==0 at EVERY deflection in the ladder.
  requireWatertightLadder(cc_ok_, s, kThreadDeflLadder);

  // Correct volume SIGN + turn count, plus gross geometry from the native mesh (no
  // OCCT). Radial extent: the V apex is projected outward by `depth` from the
  // pitch-line radius (major−depth/2 = 4.5), so the outermost radius is pitchR+depth =
  // major+depth/2 = 5.5. Z-extent: the helix rises pitch·turns = 8 and the V section
  // overhangs one half-base beyond each end (halfBase = min(pitch/2, depth·tan(30°)) =
  // min(1.0, 0.577) = 0.577), so the Z span ≈ 8 + 2·0.577.
  tess::MeshParams vp;
  vp.deflection = 0.01;
  const tess::Mesh vm = tess::SolidMesher{vp}.mesh(s);
  const double vol = tess::enclosedVolume(vm);
  CC_CHECK(vol > 0.0);  // a real, positively-enclosed solid (correct volume sign)

  const double pitchR = 5.0 - 1.0 / 2.0;   // major − depth/2 = 4.5
  const double apexR = pitchR + 1.0;        // 5.5
  const Bbox b = bboxAt(s, 0.02);
  const double rMax = std::max({-b.xmin, b.xmax, -b.ymin, b.ymax});
  CC_CHECK(std::fabs(rMax - apexR) / apexR < 5e-2);  // apex reaches major + depth/2
  CC_CHECK(rMax > 5.0);                               // and clears the requested major radius
  const double rise = 2.0 * turns;                    // pitch·turns = 8 (the turn count)
  const double zExtent = b.zmax - b.zmin;
  CC_CHECK(zExtent > rise * 0.95);                    // spans at least the requested turns
  CC_CHECK(zExtent < rise + 2.0);                     // + at most one V base of overhang/end
}

// ── NATIVE (HARD REQUIREMENT): a well-formed tapered thread is WATERTIGHT at every
// deflection (tapering pitch radius) ──────────────────────────────────────────────
// top=6, tip=4, pitch=2, turns=3, depth=1, flank=60°, spt=16. The pitch-line radius
// tapers 3.5 (tip) → 5.5 (top). Non-null with the V-tiling face count, boundaryEdges==0
// at EVERY rung of the ladder (seam-weld regression, tapered variant), positive volume.
CC_TEST(tapered_thread_is_watertight_across_ladder) {
  const int turns = 3, spt = 16;
  const topo::Shape s = cst::build_tapered_thread(6.0, 4.0, 2.0, turns, 1.0, 60.0, 1.0, spt);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  const int stations = turns * spt;
  CC_CHECK_EQ(countSub(s, topo::ShapeType::Face), 3 * stations + 2);

  // HARD watertight requirement across the ladder.
  requireWatertightLadder(cc_ok_, s, kThreadDeflLadder);

  tess::MeshParams vp;
  vp.deflection = 0.01;
  const tess::Mesh vm = tess::SolidMesher{vp}.mesh(s);
  CC_CHECK(tess::enclosedVolume(vm) > 0.0);  // correct volume sign
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

// ── NATIVE (RESOLVER, widened envelope): near-touching turns weld watertight ──────
// The FINE-PITCH RESOLVER (thread.h detail::resolveHalfBase) opens a small root flat
// where adjacent turns' V bases would otherwise MEET (2·halfBase ≈ pitch), so a thread
// whose turns nearly touch — but do NOT geometrically fold — now welds ROBUSTLY
// watertight instead of falling back. These configurations previously produced a
// non-manifold seam (the coincident root rings shared a root edge among four faces) and
// were rejected by the engine self-verify; they are now native.
//
//   * major5/pitch0.5/depth1 — a FINE pitch at the reference depth; the natural
//     60°-flank half-base (0.577) exceeds pitch/2 (0.25), so the resolver caps it and
//     opens a root flat. Shallow lead ⇒ genuinely valid, now native. depth/pitch = 2.
//   * major5/pitch1/depth2 — a DEEP V at moderate pitch (former defer). depth/pitch = 2.
// Both have a shallow lead, roots that clear the axis, and depth/pitch ≤ the deep-spike
// deferral threshold (kMaxDepthOverPitch), so the native pure-radial V agrees with the
// OCCT swept solid to within the parity bound and stays native.
CC_TEST(fine_pitch_resolver_welds_near_touching_turns) {
  for (const auto& [mj, pi, tu, de] : std::vector<std::array<double, 4>>{
           {5.0, 0.5, 4.0, 1.0}, {5.0, 1.0, 4.0, 2.0}}) {
    const topo::Shape s = cst::build_helical_thread(mj, pi, tu, de, 60.0, 1.0, 16);
    CC_CHECK(!s.isNull());  // resolver keeps it a native candidate (no defer)
    // HARD: watertight at EVERY rung of the same ladder the engine self-verify uses.
    requireWatertightLadder(cc_ok_, s, kThreadDeflLadder);
    tess::MeshParams vp;
    vp.deflection = 0.01;
    CC_CHECK(tess::enclosedVolume(tess::SolidMesher{vp}.mesh(s)) > 0.0);
  }
}

// ── GUARD (regression, native-vs-OCCT parity): a DEEP-SPIKE fine-pitch thread defers ──
// major2/pitch0.2/depth3 (depth/pitch = 15) builds a watertight native radial-V solid
// (it does NOT geometrically fold — the roots clear the axis and the lead is shallow),
// but its purely-radial V transport diverges from OCCT's MakePipeShell swept solid by
// ~11% in volume (native's exact radial Pappus volume vs OCCT's healed swept volume) —
// FAR outside the parity bound. The native self-verify (watertight only, no OCCT) cannot
// see that mismatch, so a depth/pitch guard (thread.h detail::threadUnsafe,
// depth > kMaxDepthOverPitch·pitch) DEFERS it to the OCCT oracle rather than ship a
// native solid that silently disagrees with the reference. Regression for the
// native_geomcompletion_parity [sweep] self-intersecting-thread fall-through case.
CC_TEST(deep_spike_fine_pitch_thread_deferred) {
  CC_CHECK(cst::build_helical_thread(2.0, 0.2, 4.0, 3.0, 60.0, 1.0, 16).isNull());
  CC_CHECK(cst::build_helical_thread(2.0, 0.2, 8.0, 3.0, 60.0, 4.0, 16).isNull());  // parity params
}

// ── GUARD (unchanged intent): a GENUINELY self-intersecting thread falls back ─────
// A root flat cannot fix a sweep whose radial-V flanks cross in 3D. That happens at a
// STEEP helix LEAD — a large pitch on a small pitch radius, lead = atan(pitch/(2π·pitchR))
// well past ~20°. The lead-ratio guard (thread.h detail::threadUnsafe,
// pitch/(2π·pitchR) > kMaxLeadRatio) DEFERS these to OCCT by returning a NULL Shape;
// this is Tier-4 surface-surface-intersection territory, never attempted natively.
//   * major1/pitch3/depth0.4 — pitchR 0.8, lead ~31° → fold → NULL.
//   * major2/pitch6/depth0.5 — pitchR 1.75, lead ~29° → fold → NULL.
//   * major0.6/pitch3/depth0.3 — pitchR 0.45, lead ~47° → fold → NULL.
CC_TEST(steep_lead_self_intersecting_thread_deferred) {
  CC_CHECK(cst::build_helical_thread(1.0, 3.0, 3.0, 0.4, 60.0, 1.0, 16).isNull());
  CC_CHECK(cst::build_helical_thread(2.0, 6.0, 3.0, 0.5, 60.0, 1.0, 16).isNull());
  CC_CHECK(cst::build_helical_thread(0.6, 3.0, 3.0, 0.3, 60.0, 1.0, 16).isNull());
  // The tapered analogue: a tip whose local pitch radius makes the lead steep defers.
  CC_CHECK(cst::build_tapered_thread(3.0, 1.0, 3.0, 3.0, 0.4, 60.0, 1.0, 16).isNull());
}

// ── NATIVE (HARD REQUIREMENT): a FINER-PITCH thread than the reference is watertight
// at MULTIPLE deflections ──────────────────────────────────────────────────────────
// The reference watertight case uses pitch 2 (helical_thread_is_watertight_across_ladder).
// This asserts a FINER pitch (1.0 — half the reference lead, more turns packed into the
// same axial rise) still meshes ROBUSTLY WATERTIGHT (boundaryEdges==0) at EVERY rung of
// the deflection ladder, with the right V-tiling face count and a positive volume. A
// finer pitch puts the per-turn ruled-band ↔ band seams closer together, so it is the
// stronger regression for the canonical seam-weld fix (edge_mesher CanonicalEndpoints /
// face_mesher BoundaryAnchors). major6/pitch1/turns4/depth0.8/flank60°/spt16 — the lead
// stays shallow (pitchR = 6 − 0.4 = 5.6, lead ≈ atan(1/(2π·5.6)) ≈ 1.6°), well clear of
// the self-intersection guard, so it is a legitimately NATIVE thread.
CC_TEST(finer_pitch_thread_is_watertight_across_ladder) {
  const int turns = 4, spt = 16;
  const topo::Shape s = cst::build_helical_thread(6.0, 1.0, turns, 0.8, 60.0, 1.0, spt);
  CC_CHECK(!s.isNull());
  if (s.isNull()) return;
  const int stations = turns * spt;
  // A fine pitch triggers the root-flat RESOLVER (thread.h detail::resolveHalfBase),
  // which opens an extra root band where adjacent turns' V bases would meet, so the
  // face count is ≥ the plain 3-band-per-span tiling (not the exact 3·stations+2 of a
  // coarse thread). We assert it is a genuinely tiled thread with the expected per-span
  // band structure lower bound.
  CC_CHECK(countSub(s, topo::ShapeType::Face) >= 3 * stations + 2);  // ≥ 3 bands/span + 2 caps

  // HARD watertight requirement across the whole deflection ladder (finer pitch = the
  // tighter seam-weld regression).
  requireWatertightLadder(cc_ok_, s, kThreadDeflLadder);

  tess::MeshParams vp;
  vp.deflection = 0.01;
  const tess::Mesh vm = tess::SolidMesher{vp}.mesh(s);
  CC_CHECK(tess::enclosedVolume(vm) > 0.0);  // correct volume sign

  // Gross geometry: axial rise = pitch·turns = 4; apex reaches major + depth/2 = 6.4.
  const double pitchR = 6.0 - 0.8 / 2.0;  // 5.6
  const double apexR = pitchR + 0.8;       // 6.4
  const Bbox b = bboxAt(s, 0.02);
  const double rMax = std::max({-b.xmin, b.xmax, -b.ymin, b.ymax});
  CC_CHECK(std::fabs(rMax - apexR) / apexR < 5e-2);
  const double rise = 1.0 * turns;  // pitch·turns = 4
  const double zExtent = b.zmax - b.zmin;
  CC_CHECK(zExtent > rise * 0.95);
  CC_CHECK(zExtent < rise + 2.0);
}

int main() { return cctest::run_all(); }

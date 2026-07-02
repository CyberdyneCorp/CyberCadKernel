// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native swept-solid CONSTRUCTION library (Phase 4,
// capability #4 `native-construction`). OCCT-FREE — this is Gate 1 (host,
// analytic) of the two-gate verification model in openspec/NATIVE-REWRITE.md:
// native code compiles and unit-tests with clang++ -std=c++20, no OCCT, no
// simulator, no cc_* facade.
//
// Unlike tests/test_native_engine.cpp (which drives the SAME ops through the
// cc_* facade + NativeEngine toggle), this suite exercises the native builders
// DIRECTLY (construct::build_prism / build_revolution → topology::Shape) and
// validates each result with the native TESSELLATOR (watertight / enclosed
// volume / surface area) and the native topology EXPLORER (unique sub-shape
// counts + analytic face-surface classification). Nothing here links OCCT.
//
// WHAT IS ASSERTED (honest — matches the delivered scope, see construct.h):
//   * build_prism of a 10×10 square, depth 10 → a box: native topology has 8
//     unique vertices and 6 faces; the native mesh is WATERTIGHT, satisfies the
//     mesh Euler relation V − E + F = 2 (a genus-0 closed surface), and has the
//     EXACT enclosed volume 1000 and surface area 600 (planar ⇒ mesh is exact).
//     NOTE ON EDGE COUNT: build_prism gives every face its OWN boundary edges
//     (each planar face builds independent edges carrying its own pcurve), so the
//     topological Explorer reports 24 edges, not the 12 of a fully edge-shared
//     B-rep. Vertices ARE shared (the bottom/top rings are reused by the side
//     quads), which is what welds the mesh watertight. We therefore assert the
//     Euler characteristic on the WELDED MESH (where 8 V, 18 E, 12 F ⇒ 2), which
//     is the genus-0 closed-solid invariant the box must satisfy, and assert the
//     shared-vertex / face counts on the topology.
//   * build_prism of an L-shaped (concave rectilinear) profile → correct prism
//     volume (profile area × depth) and WATERTIGHT. A triangular profile is also
//     built: its 5-face topology is asserted AND — now that the tessellator's convex
//     cap-fill is fixed (for a planar face isFullRectangle requires the loop to hit
//     all four box corners, so an inscribed triangle is ear-clipped) — it meshes
//     watertight with the exact profile-area × depth volume too.
//   * build_revolution of a rectangle profile about the Y axis, full 2π → a solid
//     cylinder: WATERTIGHT; volume ≈ πr²h within a fine-deflection relative bound;
//     every face classifies to a PLANAR cap or a CYLINDER side (no cone / free-
//     form). The full turn is split into 120° angular spans (see construct.h), so
//     there are three cylindrical side patches and pie-slice planar caps rather
//     than a single cylinder + two disks — we assert the surface KINDS present and
//     that side/cap patches both exist, not literal counts of 1 and 2.
//   * build_revolution partial angle (π) → a half cylinder capped by planar
//     profile faces: WATERTIGHT with half the full-turn volume.
//   * a DEFERRED case returns the not-supported signal — a NULL Shape — and never
//     fabricates a wrong solid. Arc/spline profiles are deferred at the engine's
//     typed solid_revolve_profile entry (forwarded wholesale to OCCT); the native
//     build_revolution accepts only line segments, and returns a null Shape for
//     inputs it cannot sweep (empty profile, zero angle, a profile lying on the
//     axis so no face of revolution exists).
//
// Build (standalone, no CMake):
//   clang++ -std=c++20 tests/native/test_native_construct.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp \
//     -I src -I tests -o test_native_construct
//
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace m = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// Count the UNIQUE sub-shapes of `type` in `shape` (Explorer dedups by isSame,
// i.e. shared topology nodes are counted once — the same rule OCCT's TopExp uses).
int countSub(const topo::Shape& shape, topo::ShapeType type) {
  int n = 0;
  for (topo::Explorer ex(shape, type); ex.more(); ex.next()) ++n;
  return n;
}

// Euler characteristic of a triangle MESH: V unique vertices, E unique undirected
// edges (from the shared-edge accounting the watertight check is built on), F
// triangles. A genus-0 closed surface (a topological sphere — box, cylinder, …)
// satisfies V − E + F = 2. Computed on the welded mesh, this is the closed-solid
// invariant, independent of how the B-rep chose to share/split its edges.
int meshEuler(const tess::Mesh& mesh) {
  const int V = static_cast<int>(mesh.vertexCount());
  const int E = static_cast<int>(tess::edgeUseCounts(mesh).size());
  const int F = static_cast<int>(mesh.triangleCount());
  return V - E + F;
}

// Tally the analytic surface kinds over a shape's faces.
struct SurfaceTally {
  int plane = 0, cylinder = 0, cone = 0, other = 0;
};
SurfaceTally classifyFaces(const topo::Shape& shape) {
  SurfaceTally t;
  for (topo::Explorer ex(shape, topo::ShapeType::Face); ex.more(); ex.next()) {
    const auto s = topo::surfaceOf(ex.current());
    if (!s) {
      ++t.other;
      continue;
    }
    switch (s->surface->kind) {
      case topo::FaceSurface::Kind::Plane: ++t.plane; break;
      case topo::FaceSurface::Kind::Cylinder: ++t.cylinder; break;
      case topo::FaceSurface::Kind::Cone: ++t.cone; break;
      default: ++t.other; break;
    }
  }
  return t;
}

}  // namespace

// ── build_prism: a box ─────────────────────────────────────────────────────────
// A 10×10 square extruded by depth 10 → a 10×10×10 box. The native topology has
// 8 unique vertices (shared rings) and 6 faces; the native mesh is watertight,
// closed (mesh Euler = 2), and has the EXACT volume 1000 and area 600.
CC_TEST(prism_box_topology_watertight_exact_volume_area) {
  const double square[] = {0, 0, 10, 0, 10, 10, 0, 10};  // CCW 10×10 on z=0
  const topo::Shape box = cst::build_prism(square, 4, 10.0);
  CC_CHECK(!box.isNull());
  if (box.isNull()) return;

  // Native topology: 8 shared vertices, 6 faces. (Edges are per-face here — see
  // the file header — so we assert vertices + faces, and the closed-solid Euler
  // relation on the welded mesh below.)
  CC_CHECK_EQ(countSub(box, topo::ShapeType::Vertex), 8);
  CC_CHECK_EQ(countSub(box, topo::ShapeType::Face), 6);
  CC_CHECK_EQ(countSub(box, topo::ShapeType::Solid), 1);
  CC_CHECK_EQ(countSub(box, topo::ShapeType::Shell), 1);
  // All six faces are planar.
  const SurfaceTally st = classifyFaces(box);
  CC_CHECK_EQ(st.plane, 6);
  CC_CHECK_EQ(st.cylinder + st.cone + st.other, 0);

  // Tessellate and validate the mesh.
  tess::MeshParams p;
  p.deflection = 0.5;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(box);
  CC_CHECK(tess::isWatertight(mesh));
  // Closed genus-0 solid ⇒ mesh Euler characteristic V − E + F = 2.
  CC_CHECK_EQ(meshEuler(mesh), 2);

  // Planar box ⇒ the mesh is EXACT (no curvature to approximate).
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - 1000.0) < 1e-6);
  CC_CHECK(std::fabs(tess::surfaceArea(mesh) - 600.0) < 1e-6);
}

// ── build_prism: an L-shaped (concave, rectilinear) profile ──────────────────---
// L profile: outer 4×4 minus the 2×2 top-right corner → area 12, extruded by
// depth 3 → volume 36. Rectilinear caps tessellate exactly, so this is
// watertight with the exact profile-area × depth volume. (8 faces: 2 caps + one
// quad per the 6 profile edges.)
CC_TEST(prism_L_profile_volume_watertight) {
  const double L[] = {0, 0, 4, 0, 4, 2, 2, 2, 2, 4, 0, 4};  // CCW L, area 12
  const topo::Shape prism = cst::build_prism(L, 6, 3.0);
  CC_CHECK(!prism.isNull());
  if (prism.isNull()) return;

  // 2 caps + one side quad per profile edge (6) = 8 faces.
  CC_CHECK_EQ(countSub(prism, topo::ShapeType::Face), 8);

  tess::MeshParams p;
  p.deflection = 0.2;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(prism);
  CC_CHECK(tess::isWatertight(mesh));
  // profile area (12) × depth (3) = 36, exact for a rectilinear profile.
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - 36.0) < 1e-6);
}

// A triangular profile → a triangular prism with the correct 5-face TOPOLOGY
// (2 caps + 3 side quads), meshed WATERTIGHT with the exact profile-area × depth
// volume. A triangle inscribes every profile vertex on its UV bounding-box border,
// which USED to trip the tessellator's `isFullRectangle` fast-path (it filled the
// whole bbox rectangle rather than the triangle). For a planar face that heuristic
// now also requires the loop to hit all four box corners (a triangle reaches at most
// three), so a convex-polygon cap is correctly ear-clipped and welds watertight
// (trim.h). A right triangle with legs 3 has area 4.5; ×depth 2 = volume 9.
CC_TEST(prism_triangle_watertight_exact_volume) {
  const double tri[] = {0, 0, 3, 0, 0, 3};  // right triangle, legs 3, area 4.5
  const topo::Shape prism = cst::build_prism(tri, 3, 2.0);
  CC_CHECK(!prism.isNull());
  if (prism.isNull()) return;

  CC_CHECK_EQ(countSub(prism, topo::ShapeType::Face), 5);   // 2 caps + 3 quads
  CC_CHECK_EQ(countSub(prism, topo::ShapeType::Vertex), 6);  // 3 bottom + 3 top
  const SurfaceTally st = classifyFaces(prism);
  CC_CHECK_EQ(st.plane, 5);

  tess::MeshParams p;
  p.deflection = 0.2;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(prism);
  CC_CHECK(tess::isWatertight(mesh));
  CC_CHECK_EQ(meshEuler(mesh), 2);  // closed genus-0 solid
  // profile area (4.5) × depth (2) = 9, exact for a planar cap.
  CC_CHECK(std::fabs(tess::enclosedVolume(mesh) - 9.0) < 1e-6);
}

// build_prism rejects degenerate profiles by returning a NULL Shape (the engine
// then falls through to OCCT — it NEVER fakes a shape). Covers < 3 points, a
// zero-area (collinear) profile, and depth ≤ 0.
CC_TEST(prism_degenerate_returns_null) {
  const double twoPts[] = {0, 0, 1, 0};
  CC_CHECK(cst::build_prism(twoPts, 2, 1.0).isNull());

  const double collinear[] = {0, 0, 1, 0, 2, 0};  // zero area
  CC_CHECK(cst::build_prism(collinear, 3, 1.0).isNull());

  const double square[] = {0, 0, 1, 0, 1, 1, 0, 1};
  CC_CHECK(cst::build_prism(square, 4, 0.0).isNull());   // depth 0
  CC_CHECK(cst::build_prism(square, 4, -1.0).isNull());  // depth < 0

  CC_CHECK(cst::build_prism(nullptr, 4, 1.0).isNull());  // null profile
}

// ── build_revolution: a full-turn solid cylinder ─────────────────────────────---
// Rectangle profile [x∈[0,2], y∈[0,5]] revolved a full 2π about the Y axis
// (through the origin) → a SOLID cylinder of radius 2, height 5. Volume πr²h =
// 20π ≈ 62.83. Every face is a planar cap or a cylinder side (no cone / free-form).
CC_TEST(revolve_full_turn_cylinder) {
  const double profile[] = {0, 0, 2, 0, 2, 5, 0, 5};  // CCW rectangle on z=0
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};    // origin, +Y
  const topo::Shape cyl = cst::build_revolution(profile, 4, yAxis, 2.0 * kPi);
  CC_CHECK(!cyl.isNull());
  if (cyl.isNull()) return;

  tess::MeshParams p;
  p.deflection = 0.02;  // fine deflection ⇒ tight volume convergence
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(cyl);
  CC_CHECK(tess::isWatertight(mesh));

  const double expected = kPi * 2.0 * 2.0 * 5.0;  // πr²h = 20π
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.02);  // within 2% at defl 0.02

  // Faces classify to planar caps + cylinder sides only (segment on the axis
  // sweeps no face; the two off-axis segments give a cylinder side and a planar
  // annulus/disk). No cone, no free-form.
  const SurfaceTally st = classifyFaces(cyl);
  CC_CHECK(st.cylinder >= 1);  // ≥ 1 cylindrical side patch (120° spans ⇒ 3)
  CC_CHECK(st.plane >= 2);     // ≥ 2 planar caps (pie-slice split ⇒ more)
  CC_CHECK_EQ(st.cone, 0);
  CC_CHECK_EQ(st.other, 0);
}

// ── build_revolution: a partial (π) half cylinder ────────────────────────────---
// The same rectangle revolved only π → a half cylinder. A partial turn adds two
// planar profile-face caps to close the opening, so it is watertight with HALF
// the full-turn volume.
CC_TEST(revolve_partial_turn_half_cylinder) {
  const double profile[] = {0, 0, 2, 0, 2, 5, 0, 5};
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  const topo::Shape half = cst::build_revolution(profile, 4, yAxis, kPi);
  CC_CHECK(!half.isNull());
  if (half.isNull()) return;

  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(half);
  CC_CHECK(tess::isWatertight(mesh));

  const double expected = 0.5 * (kPi * 2.0 * 2.0 * 5.0);  // ½·20π
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.02);

  // The two planar side caps that close the π opening ⇒ strictly more planar
  // faces than the full turn's disk caps at the same span count.
  const SurfaceTally st = classifyFaces(half);
  CC_CHECK(st.cone == 0 && st.other == 0);
  CC_CHECK(st.cylinder >= 1);
}

// ── build_revolution: a cone (slanted profile segment) ───────────────────────---
// Profile [(0,0),(2,0),(0,3)] revolved full 2π about the Y axis → a cone of base
// radius 2, height 3, volume ⅓πr²h = 4π. Confirms the slanted-segment → CONE
// surface classification and the analytic volume.
CC_TEST(revolve_cone_volume_and_classification) {
  const double profile[] = {0, 0, 2, 0, 0, 3};
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};
  const topo::Shape cone = cst::build_revolution(profile, 3, yAxis, 2.0 * kPi);
  CC_CHECK(!cone.isNull());
  if (cone.isNull()) return;

  tess::MeshParams p;
  p.deflection = 0.02;
  const tess::Mesh mesh = tess::SolidMesher{p}.mesh(cone);
  CC_CHECK(tess::isWatertight(mesh));

  const double expected = kPi * 2.0 * 2.0 * 3.0 / 3.0;  // ⅓πr²h = 4π
  const double vol = tess::enclosedVolume(mesh);
  CC_CHECK(std::fabs(vol - expected) / expected < 0.03);

  // The slanted segment sweeps a CONE; the base segment sweeps a planar disk.
  const SurfaceTally st = classifyFaces(cone);
  CC_CHECK(st.cone >= 1);
  CC_CHECK(st.plane >= 1);
  CC_CHECK_EQ(st.cylinder + st.other, 0);
}

// ── deferral: the native builder returns NULL, never a wrong solid ───────────---
// The native build_revolution accepts only LINE segments; arc/spline profiles are
// deferred at the engine's typed solid_revolve_profile entry (forwarded wholesale
// to OCCT). At the native level, the not-supported signal is a NULL Shape: for any
// input it cannot sweep into a valid solid, build_revolution returns {} so the
// engine falls through cleanly. It NEVER fabricates a wrong solid.
CC_TEST(revolve_deferred_and_degenerate_return_null) {
  const cst::RevolveAxis yAxis{0.0, 0.0, 0.0, 1.0};

  // Empty profile → null.
  CC_CHECK(cst::build_revolution(std::vector<cst::LineSeg>{}, yAxis, 2.0 * kPi).isNull());

  // Zero / non-positive angle → null (no sweep).
  const double profile[] = {1, 0, 2, 0, 2, 5, 1, 5};
  CC_CHECK(cst::build_revolution(profile, 4, yAxis, 0.0).isNull());
  CC_CHECK(cst::build_revolution(profile, 4, yAxis, -1.0).isNull());

  // A profile that lies ENTIRELY on the axis (a segment on the Y axis) sweeps no
  // face of revolution → null (never a degenerate/wrong solid).
  std::vector<cst::LineSeg> onAxis = {cst::LineSeg{0.0, 0.0, 0.0, 5.0}};
  CC_CHECK(cst::build_revolution(onAxis, yAxis, 2.0 * kPi).isNull());

  // An invalid axis direction (zero-length) → null.
  const cst::RevolveAxis badAxis{0.0, 0.0, 0.0, 0.0};
  CC_CHECK(cst::build_revolution(profile, 4, badAxis, 2.0 * kPi).isNull());
}

CC_RUN_ALL()

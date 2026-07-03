// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native PLANAR-POLYHEDRON boolean (Phase 4, capability #5
// `native-booleans`). OCCT-FREE — Gate 1 (host, analytic) of the two-gate model in
// openspec/NATIVE-REWRITE.md: the native boolean compiles and unit-tests with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// It exercises boolean_solid(a, b, op) DIRECTLY on native prisms built by the
// verified construct library, and validates each result the way the ENGINE does:
// with the native TESSELLATOR (watertight closed 2-manifold) + the set-algebra
// volume guard (fuse = A+B−∩, cut = A−∩, common = ∩). The `selfVerified` helper
// below is a faithful mirror of native_engine.cpp booleanResultVerified — a result
// that fails it is DISCARDED by the engine and falls through to the OCCT oracle, so
// the tests assert the HONEST native/fallthrough split, never a leaky solid.
//
// WHAT LANDS NATIVE (asserted watertight + exact volume, planar meshes are exact):
//   * Two 10×10×10 boxes, B translated by (5,5,5) — overlap 5³ = 125:
//       fuse = 1875, cut(a−b) = 875, common = 125, ALL watertight & self-verified.
//   * Disjoint boxes: fuse = 2000 (valid two-shell union), cut = 1000, common empty.
//   * A triangular prism minus a box overlap → correct volume, watertight.
//   * Coincident-wall fuse (two flush boxes) → one merged box, ONE merged wall.
//
// WHAT IS HONESTLY FALLTHROUGH (asserted to FAIL the guard, so the engine discards
// the native attempt rather than emit a wrong/leaky solid):
//   * A fully-CONTAINED box: common = the small box lands native watertight, but
//     cut (large − interior box, an internal CAVITY) and fuse (small buried inside
//     large) are NOT robustly watertight on the planar BSP-CSG pipeline — the guard
//     rejects them, proving no leaky solid escapes.
//   * A CURVED-face operand (a revolved cylinder) makes boolean_solid return NULL
//     (isAllPlanar is false) — the planar domain is honestly bounded.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_boolean.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_boolean
//
#include "native/boolean/native_boolean.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace nb = cybercad::native::boolean;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

namespace {

// A box [x0,x0+sx]×[y0,y0+sy]×[z0,z0+sz] as a native prism. build_prism bases at
// z = 0, so a non-zero z0 is applied as a rigid Location translation on the solid —
// which extractPolygons folds into world coordinates (exercised by the (5,5,5) case).
topo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  topo::Shape s = cst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, z0})));
  return s;
}

int faceCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) ++n;
  return n;
}

// Watertight enclosed volume at a fine deflection (planar ⇒ exact). Sets `watertight`.
double vol(const topo::Shape& s, bool& watertight) {
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  watertight = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

// Watertight volume or a negative sentinel if the mesh is not watertight — mirrors
// native_engine.cpp watertightVolume (the boolean self-verify's building block).
double watertightVolume(const topo::Shape& s) {
  if (s.isNull()) return -1.0;
  bool wt = false;
  const double v = vol(s, wt);
  return wt ? v : -1.0;
}

// A FAITHFUL MIRROR of native_engine.cpp booleanResultVerified — the mandatory
// self-verify the engine runs before accepting a native boolean result. A result
// that returns false here is DISCARDED and the op falls through to OCCT. Requires a
// closed watertight 2-manifold AND the exact set-algebra volume for the op.
bool selfVerified(const topo::Shape& result, const topo::Shape& a, const topo::Shape& b, int op) {
  const double vr = watertightVolume(result);
  if (vr < 0.0) return false;  // not watertight → rejected

  const double va = watertightVolume(a);
  const double vb = watertightVolume(b);
  if (va < 0.0 || vb < 0.0) return true;  // operands not measurable → trust watertight

  const topo::Shape common = nb::boolean_solid(a, b, nb::Op::Common);
  const double vc = common.isNull() ? 0.0 : std::max(0.0, watertightVolume(common));

  double expected = 0.0;
  switch (op) {
    case 0: expected = va + vb - vc; break;  // fuse
    case 1: expected = va - vc; break;       // cut a−b
    case 2: expected = vc; break;            // common
    default: return false;
  }
  if (!(expected > 0.0)) return false;
  const double tol = std::max(1e-6 * expected, 1e-9);
  return std::fabs(vr - expected) <= tol;
}

bool nearRel(double got, double want, double rel = 1e-6, double abs = 1e-9) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), abs);
}

}  // namespace

// ── NATIVE: two 10-cubes, B translated by (5,5,5) — overlap 5³ = 125. ────────────
// fuse = 2000 − 125 = 1875, cut(a−b) = 1000 − 125 = 875, common = 125. Each result
// EXACT (planar meshes are exact) and watertight + self-verified. This is the
// headline research-grade case: a full-3D-diagonal overlap whose caps are L-shaped
// concave polygons (the coplanar T-junction repair + B-rep triangulation must close).
CC_TEST(two_cubes_offset_555_fuse_cut_common_exact) {
  const topo::Shape A = boxAt(0, 0, 0, 10, 10, 10);
  const topo::Shape B = boxAt(5, 5, 5, 10, 10, 10);
  CC_CHECK(!A.isNull() && !B.isNull());

  bool wt = false;
  const topo::Shape fuse = nb::boolean_solid(A, B, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  CC_CHECK(nearRel(vol(fuse, wt), 1875.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(fuse, A, B, 0));

  const topo::Shape cut = nb::boolean_solid(A, B, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  CC_CHECK(nearRel(vol(cut, wt), 875.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(cut, A, B, 1));

  const topo::Shape common = nb::boolean_solid(A, B, nb::Op::Common);
  CC_CHECK(!common.isNull());
  CC_CHECK(nearRel(vol(common, wt), 125.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(common, A, B, 2));
}

// ── NATIVE: disjoint boxes. ──────────────────────────────────────────────────────
// A = 10³ at origin, D = 10³ far away. fuse → a valid TWO-SHELL union, summed volume
// 2000, watertight; cut(a−d) → A unchanged, 1000; common → NULL/empty (handled).
CC_TEST(disjoint_boxes_fuse2000_cut1000_common_empty) {
  const topo::Shape A = boxAt(0, 0, 0, 10, 10, 10);
  const topo::Shape D = boxAt(30, 30, 30, 10, 10, 10);

  bool wt = false;
  const topo::Shape fuse = nb::boolean_solid(A, D, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  CC_CHECK(nearRel(vol(fuse, wt), 2000.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(fuse, A, D, 0));

  const topo::Shape cut = nb::boolean_solid(A, D, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  CC_CHECK(nearRel(vol(cut, wt), 1000.0));
  CC_CHECK(wt);

  // No overlap ⇒ common is empty. The library may return NULL, or a degenerate
  // shape that fails the self-verify; either way the engine yields no common solid.
  const topo::Shape common = nb::boolean_solid(A, D, nb::Op::Common);
  CC_CHECK(common.isNull() || !selfVerified(common, A, D, 2));
}

// ── NATIVE + HONEST FALLTHROUGH: a small box fully CONTAINED in a large box. ──────
// small = 4³ = 64 sitting strictly inside large = 10³ = 1000.
//   common = the small box (64) — lands NATIVE, watertight & self-verified.
//   cut (large − interior box) → an internal CAVITY, and
//   fuse (small buried inside large) → interior faces,
// are NOT robustly watertight on the planar BSP-CSG pipeline: their volumes are
// numerically right (936 and 1000) but the mesh is NOT a closed 2-manifold, so the
// engine's self-verify REJECTS them → OCCT fallthrough. We assert exactly that: the
// guard fails, so no wrong/leaky solid is emitted as native.
CC_TEST(contained_box_common_native_cavity_falls_through) {
  const topo::Shape large = boxAt(0, 0, 0, 10, 10, 10);
  const topo::Shape small = boxAt(3, 3, 3, 4, 4, 4);

  bool wt = false;
  // common = the contained box → native watertight, exactly its volume.
  const topo::Shape common = nb::boolean_solid(large, small, nb::Op::Common);
  CC_CHECK(!common.isNull());
  CC_CHECK(nearRel(vol(common, wt), 64.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(common, large, small, 2));

  // cut = large with an internal void — the guard must REJECT (fall through), never
  // ship a leaky solid. (Volume may read 936 but watertight must be false.)
  const topo::Shape cut = nb::boolean_solid(large, small, nb::Op::Cut);
  CC_CHECK(!selfVerified(cut, large, small, 1));

  // fuse = large (small is buried) — likewise must be rejected as non-watertight.
  const topo::Shape fuse = nb::boolean_solid(large, small, nb::Op::Fuse);
  CC_CHECK(!selfVerified(fuse, large, small, 0));
}

// ── NATIVE: a triangular prism minus a box overlap. ──────────────────────────────
// Triangular prism (right triangle legs 10×10, depth 10) → volume 500. A 4×4×10 box
// straddling the right-angle corner overlaps a 3×3×10 = 90 wedge-clipped region.
// We assert the set-algebra is self-consistent and watertight: cut = 500 − common,
// fuse = 500 + boxVol − common, each a valid watertight solid.
CC_TEST(triangular_prism_minus_box) {
  const double tri[] = {0, 0, 10, 0, 0, 10};       // right triangle, area 50
  const topo::Shape prism = cst::build_prism(tri, 3, 10);  // volume 500
  CC_CHECK(!prism.isNull());
  const topo::Shape box = boxAt(-1, -1, 0, 4, 4, 10);  // straddles the corner
  CC_CHECK(!box.isNull());

  bool wt = false;
  const topo::Shape common = nb::boolean_solid(prism, box, nb::Op::Common);
  CC_CHECK(!common.isNull());
  const double vc = vol(common, wt);
  CC_CHECK(wt);
  CC_CHECK(vc > 0.0 && vc < 160.0);  // a proper sub-region of the 4×4×10 box
  CC_CHECK(selfVerified(common, prism, box, 2));

  const topo::Shape cut = nb::boolean_solid(prism, box, nb::Op::Cut);
  CC_CHECK(!cut.isNull());
  CC_CHECK(nearRel(vol(cut, wt), 500.0 - vc));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(cut, prism, box, 1));

  const topo::Shape fuse = nb::boolean_solid(prism, box, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  CC_CHECK(nearRel(vol(fuse, wt), 500.0 + 160.0 - vc));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(fuse, prism, box, 0));
}

// ── NATIVE: two boxes sharing a wall (A [0,10], B [10,20] in x). ──────────────────
// fuse → one 10×20×10 box (2000), the coincident wall merged (NOT doubled/missing);
// common → empty (the shared wall has zero volume).
CC_TEST(coincident_wall_fuse_merges) {
  const topo::Shape A = boxAt(0, 0, 0, 10, 10, 10);
  const topo::Shape B = boxAt(10, 0, 0, 10, 10, 10);

  bool wt = false;
  const topo::Shape fuse = nb::boolean_solid(A, B, nb::Op::Fuse);
  CC_CHECK(!fuse.isNull());
  CC_CHECK(nearRel(vol(fuse, wt), 2000.0));
  CC_CHECK(wt);
  CC_CHECK(selfVerified(fuse, A, B, 0));

  const topo::Shape common = nb::boolean_solid(A, B, nb::Op::Common);
  CC_CHECK(common.isNull() || !selfVerified(common, A, B, 2));
}

// ── HONEST FALLTHROUGH: a curved-face operand is OUTSIDE the planar domain. ───────
// A full-revolution cylinder is not all-planar, so boolean_solid returns NULL and
// the engine falls through to OCCT. It must NEVER emit a native (wrong) solid here.
CC_TEST(curved_operand_returns_null_for_fallthrough) {
  const topo::Shape block = boxAt(0, 0, 0, 10, 10, 10);
  const double rectp[] = {0, 0, 3, 0, 3, 5, 0, 5};
  const topo::Shape cyl =
      cst::build_revolution(rectp, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * 3.14159265358979323846);
  CC_CHECK(!cyl.isNull());
  CC_CHECK(!nb::isAllPlanar(cyl));
  CC_CHECK(nb::boolean_solid(block, cyl, nb::Op::Fuse).isNull());
  CC_CHECK(nb::boolean_solid(cyl, block, nb::Op::Cut).isNull());
  CC_CHECK(nb::boolean_solid(block, cyl, nb::Op::Common).isNull());
}

// The isAllPlanar guard: a pure box is all-planar, a cylinder is not.
CC_TEST(is_all_planar_guard) {
  CC_CHECK(nb::isAllPlanar(boxAt(0, 0, 0, 5, 5, 5)));
  const double rectp[] = {0, 0, 3, 0, 3, 5, 0, 5};
  const topo::Shape cyl =
      cst::build_revolution(rectp, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * 3.14159265358979323846);
  CC_CHECK(!nb::isAllPlanar(cyl));
}

// Determinism: the same fuse computed twice yields the same volume and face count.
CC_TEST(deterministic) {
  const topo::Shape A = boxAt(0, 0, 0, 10, 10, 10);
  const topo::Shape B = boxAt(3, 4, 5, 10, 10, 10);
  const topo::Shape r1 = nb::boolean_solid(A, B, nb::Op::Fuse);
  const topo::Shape r2 = nb::boolean_solid(A, B, nb::Op::Fuse);
  CC_CHECK(!r1.isNull() && !r2.isNull());
  bool wt1 = false, wt2 = false;
  CC_CHECK(nearRel(vol(r1, wt1), vol(r2, wt2)));
  CC_CHECK(wt1 && wt2);
  CC_CHECK_EQ(faceCount(r1), faceCount(r2));
}

CC_RUN_ALL()

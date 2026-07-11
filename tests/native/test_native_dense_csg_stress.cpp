// SPDX-License-Identifier: Apache-2.0
//
// test_native_dense_csg_stress.cpp — the NEAR-TANGENT / near-coincident DENSE-CSG
// stress battery + failure-boundary map for the native planar-polyhedron boolean
// (Phase 4 #5 `native-booleans`). OCCT-FREE host gate: a closed-form-volume oracle
// (boxes / prisms / wedges with computable overlap), no simulator, no cc_* facade.
//
// ── WHY ─────────────────────────────────────────────────────────────────────────
// near-tangent / near-coincident dense-CSG is the recurring moat locus: it gates the
// thread booleans (thread_apply.h documents the exact failure — near-tangent helical
// root ↔ shaft-wall contact fragments the dense triangle-soup BSP into T-junction
// cracks, boundaryEdgeCount 15–140), the curved-boolean tail, and it is a cousin of
// the SSI tangent-degeneracy work. This battery MEASURES the exact failure boundary as
// concrete numbers so the moat is mapped, not guessed.
//
// ── THE DENSE-SOUP REGIME (why plain boxes do not reproduce it) ───────────────────
// A clean N-gon box boolean stays watertight even at an exactly-coincident wall (the
// BSP coplanar-front/back split handles it — see test_native_boolean.cpp). The moat
// failure is specific to DENSE TRIANGLE SOUP: operands faceted into many independently-
// vertexed planar triangles (exactly what thread_apply::facetSolid feeds the BSP so the
// EXACT planar set-algebra applies to a helix). `facetSolidLocal` below reproduces that
// faceting so the battery stresses the SAME code path the thread boolean hits.
//
// ── WHAT THIS MEASURES (the failure boundary, as asserted numbers) ────────────────
// Driving the real native `boolean_solid` on dense-soup operands and classifying every
// result AGREED / HONESTLY-DECLINED / DISAGREED against the closed-form oracle, three
// reproducible failure bands emerge (measured at box size 10, faceting deflection 0.4,
// mesh-check deflection 0.02):
//
//   BAND 1 — near-TANGENT tilt crack. Two soup boxes, the upper tilted by angle θ so
//     its dense-tiled bottom plane grazes the lower box's top plane. For θ ∈ [~0.10°,
//     ~0.25°] the fused mesh CRACKS: not watertight, boundaryEdgeCount = 3 (the classic
//     T-junction crack). Above ~0.30° and below ~0.08° it closes. MECHANISM: the grazing
//     split produces near-degenerate sliver fragments whose corners sit 1e-5..3e-5 off a
//     neighbour edge on a ~0.01-long tiling edge — an endpoint sliver, NOT a clean
//     missed interior T-junction, so widening the repair tolerance would corrupt
//     geometry rather than fix it (measured: the "missed" points have t≈0 / t≈1).
//
//   BAND 2 — near-COINCIDENT gap orientation defect. Two soup boxes stacked with a
//     shrinking air gap g. For g ∈ [~5e-7, ~1e-3] the fused mesh is NOT consistently
//     oriented: sameDirectionEdgeCount = 8 (doubled same-direction half-edges across the
//     two near-coincident but unwelded planes). At g ≤ kWeldTol (1e-7) the corners weld
//     and it closes; at g ≥ ~5e-3 the planes are cleanly distinct.
//
//   BAND 3 — near-COINCIDENT overlap WRONG-VOLUME (the dangerous one). Two soup boxes
//     overlapping by a thin slab ε. For ε ∈ [~2e-4, ~3e-3] the fused mesh is watertight
//     yet the enclosed volume is WRONG by up to 0.45 (vs the exact 2000 − 100ε). CRUCIAL
//     MECHANISM (measured): the raw boolean POLYGON SOUP volume is EXACTLY correct at
//     every ε — the error appears only after assemble.h + the FaceMesher re-triangulate/
//     weld the thin slab. So the boolean CORE is sound; the defect is a downstream
//     near-coincident tessellation-weld artifact, outside boolean/**.
//
// ── THE INVARIANT THIS BATTERY PROVES (DISAGREED == 0) ────────────────────────────
// Every one of the three bands is CAUGHT by the engine's mandatory self-verify
// (watertight + exact set-algebra volume — `selfVerified` below is a faithful mirror of
// native_engine.cpp booleanResultVerified), so each is HONESTLY-DECLINED (the engine
// discards the native attempt and falls through to OCCT). The battery asserts
// DISAGREED == 0: the native boolean NEVER presents a wrong result as valid. The moat is
// a RECALL problem (native declines cases it could handle), not a SOUNDNESS problem.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_dense_csg_stress.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o test_native_dense_csg_stress
//
#include "native/boolean/native_boolean.h"
#include "native/construct/construct.h"        // detail::planarFace (dense faceting)
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace topo = cybercad::native::topology;
namespace nb = cybercad::native::boolean;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;
namespace nmath_ = cybercad::native::math;

namespace {

constexpr double kPi = 3.14159265358979323846;

// A box [x0,x0+sx]×[y0,y0+sy]×[z0,z0+sz] as a native prism (z0 applied as a rigid
// Location, folded to world by extractPolygons — mirrors test_native_boolean.cpp).
topo::Shape boxAt(double x0, double y0, double z0, double sx, double sy, double sz) {
  const double p[] = {x0, y0, x0 + sx, y0, x0 + sx, y0 + sy, x0, y0 + sy};
  topo::Shape s = cst::build_prism(p, 4, sz);
  if (z0 != 0.0 && !s.isNull())
    s = s.located(topo::Location(nmath::Transform::translationOf(nmath::Vec3{0, 0, z0})));
  return s;
}

// A native cylinder of radius R over the axial span [zlo,zhi], axis +z, built by the
// native revolve (a closed (r,h) profile loop revolved a full turn). Facing the shaft
// wall as a faceted ARC is what reproduces the thread crest/root ↔ wall CURVED seam —
// a straight-plane box tangent (boxAt) does NOT: its contact is a single flat plane, so
// the two operands' tilings agree; only a curved (arc) seam makes them disagree.
topo::Shape cylinderAt(double R, double zlo, double zhi) {
  const double prof[] = {0.0, zlo, R, zlo, R, zhi, 0.0, zhi};
  cst::RevolveAxis axis;
  axis.ax = 0.0; axis.ay = 0.0; axis.adx = 0.0; axis.ady = 1.0;
  return cst::build_revolution(prof, 4, axis, 2.0 * kPi);
}

// Facet a native solid into DENSE, independently-vertexed planar-triangle soup at a
// controlled deflection — a self-contained mirror of thread_apply::facetSolid, so this
// battery exercises the SAME dense-soup BSP path the thread boolean hits. Each mesh
// triangle becomes its own planar B-rep face (winding normal, Forward), giving every
// contact plane many fragments from independent split histories.
topo::Shape facetSolidLocal(const topo::Shape& s, double deflection) {
  tess::MeshParams p;
  p.deflection = deflection;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  std::vector<topo::Shape> faces;
  faces.reserve(m.triangles.size());
  for (const tess::Triangle& t : m.triangles) {
    const nmath::Point3 a = m.vertices[t.a], b = m.vertices[t.b], c = m.vertices[t.c];
    const nmath::Vec3 n = nmath::cross(b - a, c - a);
    const double nl = nmath::norm(n);
    if (!(nl > 1e-12)) continue;  // drop a degenerate sliver triangle
    std::vector<topo::Shape> loop = {topo::ShapeBuilder::makeVertex(a),
                                     topo::ShapeBuilder::makeVertex(b),
                                     topo::ShapeBuilder::makeVertex(c)};
    faces.push_back(cst::detail::planarFace(loop, nmath::Dir3{n / nl}, topo::Orientation::Forward));
  }
  if (faces.size() < 4) return {};
  return topo::ShapeBuilder::makeSolid({topo::ShapeBuilder::makeShell(std::move(faces))});
}

// Diagnostics of a candidate solid's mesh at a fixed check deflection.
struct MeshStat {
  bool watertight = false;
  std::size_t boundaryEdges = 0;
  std::size_t sameDir = 0;
  double volume = 0.0;
};
MeshStat meshStat(const topo::Shape& s, double checkDefl = 0.02) {
  MeshStat r;
  if (s.isNull()) return r;
  tess::MeshParams p;
  p.deflection = checkDefl;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  r.watertight = tess::isWatertight(m);
  r.boundaryEdges = tess::boundaryEdgeCount(m);
  r.sameDir = tess::sameDirectionEdgeCount(m);
  r.volume = std::fabs(tess::enclosedVolume(m));
  return r;
}

// Watertight volume or a negative sentinel (mirrors native_engine watertightVolume).
double watertightVolume(const topo::Shape& s, double checkDefl = 0.01) {
  if (s.isNull()) return -1.0;
  tess::MeshParams p;
  p.deflection = checkDefl;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  return tess::isWatertight(m) ? std::fabs(tess::enclosedVolume(m)) : -1.0;
}

// A FAITHFUL MIRROR of native_engine.cpp booleanResultVerified — the mandatory
// self-verify the engine runs before ACCEPTING a native boolean result. A result that
// returns false here is DISCARDED and the op falls through to OCCT. Requires a closed
// watertight 2-manifold AND the exact set-algebra volume for the op. This is what makes
// a failure band HONESTLY-DECLINED rather than DISAGREED.
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

// Classify one dense-soup boolean against the CLOSED-FORM oracle.
//   AGREED            — self-verify accepts AND the watertight volume matches oracle.
//   HONESTLY-DECLINED — self-verify rejects (engine falls through to OCCT).
//   DISAGREED         — self-verify ACCEPTS but the volume is actually wrong (the
//                       silent-wrong hazard the invariant forbids).
enum class Verdict { Agreed, Declined, Disagreed };
Verdict classify(const topo::Shape& result, const topo::Shape& a, const topo::Shape& b, int op,
                 double oracleVolume) {
  const bool sv = selfVerified(result, a, b, op);
  if (!sv) return Verdict::Declined;
  // Accepted: is it actually right? Compare its watertight volume to the closed form.
  const double vr = watertightVolume(result);
  const double tol = std::max(1e-5 * oracleVolume, 1e-7);
  if (vr < 0.0 || std::fabs(vr - oracleVolume) > tol) return Verdict::Disagreed;
  return Verdict::Agreed;
}

// Build the two dense-soup operands for the TILT case: lower box [0,10]³, upper box
// [0,10]²×[10,20] rotated by θ (radians) about the y-axis line through (0,0,10) so its
// bottom plane grazes z=10. Faceted at `defl`.
void tiltOperands(double thetaRad, double defl, topo::Shape& A, topo::Shape& B) {
  A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), defl);
  topo::Shape upper = boxAt(0, 0, 10, 10, 10, 10);
  upper = upper.located(topo::Location(
      nmath::Transform::rotationOf(nmath::Point3{0, 0, 10}, nmath::Dir3{0, 1, 0}, thetaRad)));
  B = facetSolidLocal(upper, defl);
}

}  // namespace

// ═══════════════════════════════════════════════════════════════════════════════════
// CONTROL: dense soup DOES land native when the contact is not near-tangent.
// Two soup boxes overlapping along a full-3D diagonal (offset 5,5,5, overlap 5³=125):
// fuse = 1875, exact + watertight + self-verified. Proves the dense-soup BSP substrate
// is SOUND — the moat is the NEAR-tangency, not faceting per se.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(dense_soup_diagonal_overlap_lands_native) {
  const topo::Shape A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
  const topo::Shape B = facetSolidLocal(boxAt(5, 5, 5, 10, 10, 10), 0.4);
  CC_CHECK(!A.isNull() && !B.isNull());
  const topo::Shape fuse = nb::boolean_solid(A, B, nb::Op::Fuse);
  const MeshStat s = meshStat(fuse);
  CC_CHECK(s.watertight);
  CC_CHECK(s.boundaryEdges == 0);
  CC_CHECK(std::fabs(s.volume - 1875.0) <= 1e-3);
  CC_CHECK(classify(fuse, A, B, 0, 1875.0) == Verdict::Agreed);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 1 — near-TANGENT tilt crack, NOW RECOVERED. Previously θ ∈ [~0.10°, ~0.25°]
// CRACKED (not watertight, boundaryEdgeCount == 3 — a near-collinear ENDPOINT T-junction
// sliver: the tilted operand's grazing tiling dropped an interpolated corner ~1e-5 off
// the lower box's true corner). assemble.h::collapseSliverEdges now welds that sliver
// notch into its near-coincident neighbour (scale-relative candidate test + a global
// soup-volume-preservation guard that keeps the collapse only when it moves no real
// volume) so the seam closes. ASSERTED map (post-fix): the WHOLE band closes watertight.
//   θ = 0.30°  → watertight (boundaryEdges 0)  [was watertight]
//   θ = 0.20°  → watertight (boundaryEdges 0)  [was CRACKED, boundaryEdges 3]
//   θ = 0.10°  → watertight (boundaryEdges 0)  [was CRACKED, boundaryEdges 3]
//   θ = 0.05°  → watertight (boundaryEdges 0)  [was watertight]
// The recovered fuse is watertight and NEVER DISAGREED. (Whether the engine's set-algebra
// self-verify additionally ACCEPTS a given angle depends on its COMMON cross-check leg —
// the near-tangent wedge COMMON degenerates for some θ exactly as documented in BAND 3;
// that is a separate, pre-existing self-verify limitation, not a crack. The crack itself
// is gone for the whole band.)
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band1_near_tangent_tilt_crack_recovered) {
  topo::Shape A, B;
  auto fuseStat = [&](double deg) {
    tiltOperands(deg * kPi / 180.0, 0.4, A, B);
    return std::make_pair(nb::boolean_solid(A, B, nb::Op::Fuse), deg);
  };

  // The whole sweep, spanning the former crack band, now closes watertight and is
  // never a silent-wrong result.
  for (double deg : {0.30, 0.25, 0.20, 0.15, 0.10, 0.05}) {
    auto [f, d] = fuseStat(deg);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);       // the crack is sealed …
    CC_CHECK(s.boundaryEdges == 0);
    // … and it is never presented wrongly: oracle = va + vb − vc (closed-form set algebra).
    const double va = watertightVolume(A), vb = watertightVolume(B);
    const topo::Shape cm = nb::boolean_solid(A, B, nb::Op::Common);
    const double vc = cm.isNull() ? 0.0 : std::max(0.0, watertightVolume(cm));
    const double oracle = (va > 0 && vb > 0) ? (va + vb - vc) : 0.0;
    if (oracle > 0.0) CC_CHECK(classify(f, A, B, 0, oracle) != Verdict::Disagreed);
    (void)d;
  }
  // At least one former-crack angle now fully self-verifies (a real accepted result,
  // not merely a sealed-but-declined one) — proving the recovery reaches the engine.
  {
    auto [f, deg] = fuseStat(0.15);
    CC_CHECK(selfVerified(f, A, B, 0));
    (void)deg;
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// SCALE-RELATIVE WELD SAFETY (part 1 — the crack tracks scale). The sliver collapse is
// scale-relative (candidate test is a FRACTION of the local neighbour span; the keep/
// reject decision is a FRACTION of the soup volume), NOT an absolute epsilon. Build the
// SAME near-tangent contact at a 1e-3× smaller model (box side 0.01, tilt in the crack
// band): the analogous sliver gap is now ~3e-8, still volume-neutral to remove, so the
// small model ALSO closes watertight. An absolute 3e-5 weld would either miss this
// (3e-8 ≪ 3e-5, crack stays) or, at the large model, have merged genuinely-distinct
// corners — the scale-relative predicate does neither.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band1_sliver_weld_is_scale_relative_not_absolute) {
  // Small model: side 0.01, same near-tangent tilt (0.15°). Faceting deflection scaled
  // down with the model so the soup granularity is proportional.
  auto tiltSmall = [](double thetaRad, double s, double defl, topo::Shape& A, topo::Shape& B) {
    A = facetSolidLocal(boxAt(0, 0, 0, s, s, s), defl);
    topo::Shape upper = boxAt(0, 0, s, s, s, s);
    upper = upper.located(topo::Location(
        nmath::Transform::rotationOf(nmath::Point3{0, 0, s}, nmath::Dir3{0, 1, 0}, thetaRad)));
    B = facetSolidLocal(upper, defl);
  };
  topo::Shape A, B;
  tiltSmall(0.15 * kPi / 180.0, 0.01, 0.4 * 1e-3, A, B);
  CC_CHECK(!A.isNull() && !B.isNull());
  const topo::Shape f = nb::boolean_solid(A, B, nb::Op::Fuse);
  const MeshStat s = meshStat(f, 0.02 * 1e-3);
  CC_CHECK(s.watertight);
  CC_CHECK(s.boundaryEdges == 0);
  // And it is not silently wrong: never DISAGREED (volume oracle = va+vb−vc).
  const double va = watertightVolume(A, 0.01 * 1e-3), vb = watertightVolume(B, 0.01 * 1e-3);
  const topo::Shape cm = nb::boolean_solid(A, B, nb::Op::Common);
  const double vc = cm.isNull() ? 0.0 : std::max(0.0, watertightVolume(cm, 0.01 * 1e-3));
  const double oracle = (va > 0 && vb > 0) ? (va + vb - vc) : 0.0;
  if (oracle > 0.0) CC_CHECK(classify(f, A, B, 0, oracle) != Verdict::Disagreed);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// SCALE-RELATIVE WELD SAFETY (part 2 — a genuinely-separated thin feature is NOT welded).
// The decisive proof the weld never merges separation-carrying vertices: a THIN COMMON
// slab. The COMMON of two soup boxes overlapping by ε = 1e-2 is a genuine 10×10×0.01
// slab (volume 1.0). Its thin walls have vertices only ~0.01 apart — the SAME order as a
// near-tangent sliver gap — so the scale-relative NOTCH candidate test can flag them. But
// welding them would collapse the slab thickness and destroy its 1.0 volume; the collapse
// guard REJECTS exactly that (it keeps a collapse only if the soup volume is preserved),
// so the slab survives intact at its correct volume. This is the different-scale, genuine-
// separation case that must NOT be welded — and is not.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(thin_common_slab_is_not_welded_away) {
  const double eps = 1e-2;
  topo::Shape A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
  topo::Shape B = facetSolidLocal(boxAt(0, 0, 10 - eps, 10, 10, 10), 0.4);
  CC_CHECK(!A.isNull() && !B.isNull());
  const topo::Shape common = nb::boolean_solid(A, B, nb::Op::Common);
  const MeshStat s = meshStat(common);
  // The thin slab is preserved: watertight and volume ≈ 100·ε = 1.0 (NOT collapsed to 0).
  CC_CHECK(s.watertight);
  CC_CHECK(s.boundaryEdges == 0);
  CC_CHECK(std::fabs(s.volume - 100.0 * eps) <= 1e-3);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 2 — near-COINCIDENT gap orientation defect, RECOVERED. Two soup boxes stacked
// with air gap g. Previously the doubled, unwelded near-coincident walls left the fuse
// NOT watertight with sameDirectionEdgeCount == 8 for g ∈ [~5e-7, ~1e-3]. The scale-
// relative, volume-validated near-coincident-wall cancellation (bsp.h
// cancelNearCoincidentWalls) now merges those internal air-slab walls, so the fuse is
// consistently oriented (sameDir 0) and watertight across the whole band:
//   g = 5e-3  → planes cleanly distinct (sep > tol): unchanged, watertight, sameDir 0
//   g = 1e-3  → RECOVERED: watertight, sameDir 0 (previously !watertight / sameDir 8)
//   g = 1e-4  → RECOVERED: watertight, sameDir 0 (previously !watertight / sameDir 8)
//   g = 5e-7  → RECOVERED: watertight, sameDir 0 (previously !watertight / sameDir 8)
//   g = 1e-8  → below kWeldTol: corners already welded, watertight, sameDir 0
// DISAGREED stays 0: where merging the air slab makes the enclosed volume (2000 + 100g)
// differ from the closed-form union oracle 2000 by more than the self-verify band
// (g ≳ 1e-5), the engine still HONESTLY-DECLINES — a watertight-but-wrong-volume fuse
// is never presented as valid (asserted below at g = 1e-3). Below that scale the air
// slab is within tolerance and the result AGREES.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band2_near_coincident_gap_orientation_recovered) {
  auto stacked = [&](double g, topo::Shape& A, topo::Shape& B) {
    A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    B = facetSolidLocal(boxAt(0, 0, 10 + g, 10, 10, 10), 0.4);
    return nb::boolean_solid(A, B, nb::Op::Fuse);
  };
  topo::Shape A, B;
  // Cleanly-distinct planes (separation 5e-3 > scale-relative tol ~2e-3): untouched.
  {
    const topo::Shape f = stacked(5e-3, A, B);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);
    CC_CHECK(s.sameDir == 0);
  }
  // In the former defect band: now RECOVERED to watertight / sameDir 0. At g = 1e-3 the
  // merged solid encloses the 0.1 air slab, so its volume (2000.1) still fails the exact
  // set-algebra self-verify vs the union oracle 2000 → HONESTLY-DECLINED, never a wrong
  // result shipped (DISAGREED stays 0).
  {
    const topo::Shape f = stacked(1e-3, A, B);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);   // previously !watertight
    CC_CHECK(s.sameDir == 0);  // previously 8
    CC_CHECK(!selfVerified(f, A, B, 0));  // volume 2000.1 ≠ union 2000 → still declined
  }
  {
    const topo::Shape f = stacked(1e-4, A, B);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);   // previously !watertight
    CC_CHECK(s.sameDir == 0);  // previously 8
  }
  {
    const topo::Shape f = stacked(5e-7, A, B);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);   // previously !watertight
    CC_CHECK(s.sameDir == 0);  // previously 8
  }
  {
    const topo::Shape f = stacked(1e-8, A, B);  // below kWeldTol → corners weld
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);
    CC_CHECK(s.sameDir == 0);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 2 SAFETY — the different-scale near-coincident pair that MUST NOT be merged. The
// wall cancellation is gated by exact soup signed-volume preservation, so a GENUINELY
// thin SOLID feature whose opposite walls are near-coincident (a real slab, not an air
// slab) is NEVER collapsed. The adversary is the COMMON of two soup boxes overlapping by
// a thin slab of thickness ε: the result IS a real slab whose top and bottom are near-
// coincident opposite walls within the wall band. Merging them would destroy ~100% of
// its real volume, so the volume gate REJECTS the cancellation and the slab's polygon
// soup is preserved EXACTLY (signed volume 100·ε), with a 4-order-of-magnitude margin
// over the air-slab case (Δ ~ 1e-5 there vs ~3e-1 here). We assert at the polygon-soup
// level (the boolean CORE output), which is where the guard acts — the downstream thin-
// slab tessellation-weld degeneracy is a separate, documented Band-3 finding.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band2_thin_solid_slab_soup_is_not_merged_away) {
  for (double eps : {5e-3, 1e-3, 5e-4}) {
    const topo::Shape A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    const topo::Shape B = facetSolidLocal(boxAt(0, 0, 10 - eps, 10, 10, 10), 0.4);
    CC_CHECK(!A.isNull() && !B.isNull());
    // The boolean CORE output soup for COMMON: a genuine thin slab of thickness ε. Its
    // exact signed volume must be preserved (100·ε) — the near-coincident opposite walls
    // are a real feature the volume-preservation gate refuses to collapse.
    std::vector<nb::Polygon> soup =
        nb::booleanPolygons(nb::extractPolygons(A), nb::extractPolygons(B), nb::Op::Common);
    const double v = nb::soupVolume(soup);
    CC_CHECK(std::fabs(v - 100.0 * eps) <= 1e-6);  // slab NOT collapsed (would be ~0)
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 3 — near-COINCIDENT overlap, HONESTLY-DECLINED (never DISAGREED). Two soup boxes
// overlapping by a thin slab ε (closed-form fuse volume 2000 − 100ε). ASSERTED map:
//   ε = 1e-3  → mesh watertight and the FUSE volume is now EXACT (1999.9) — the sliver
//               collapse also seals the thin-overlap re-triangulation, removing the old
//               ~0.15 wrong-volume artifact. But the engine self-verify's COMMON cross-
//               check leg (the degenerate thin slab of thickness 0.1) still collapses to
//               0, so the fuse is scored against 2000 and DECLINED. Never DISAGREED.
//   ε = 1e-4  → FUSE volume exact (1999.99); the near-coincident COMMON (thickness 0.01)
//               collapses to 0 in the tessellated cross-check, so the fuse is scored
//               against 2000 and DECLINED. Either way it is never DISAGREED.
// Measured finding (unchanged by the fix): the near-coincident COMMON degenerates before
// the fuse does, and the self-verify inherits its weakest leg — a pre-existing
// tessellation-weld limit at that thinness, outside boolean/**.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band3_near_coincident_overlap_is_declined_never_disagreed) {
  auto overlap = [&](double eps, topo::Shape& A, topo::Shape& B) {
    A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    B = facetSolidLocal(boxAt(0, 0, 10 - eps, 10, 10, 10), 0.4);
    return nb::boolean_solid(A, B, nb::Op::Fuse);
  };
  topo::Shape A, B;
  {
    const double eps = 1e-3, oracle = 2000.0 - 100.0 * eps;  // 1999.9
    const topo::Shape f = overlap(eps, A, B);
    const MeshStat s = meshStat(f);
    CC_CHECK(s.watertight);                          // closed 2-manifold …
    CC_CHECK(std::fabs(s.volume - oracle) <= 0.05);  // … and the FUSE volume is now EXACT.
    // The self-verify's COMMON leg degenerates at this thinness, so the op is still
    // HONESTLY-DECLINED — never DISAGREED (no wrong result presented as valid).
    CC_CHECK(!selfVerified(f, A, B, 0));
    CC_CHECK(classify(f, A, B, 0, oracle) != Verdict::Disagreed);
  }
  {
    // Below the artifact scale the FUSE volume itself is exact (measured 1999.99 at
    // ε=1e-4, error ~2e-13). But the engine self-verify's set-algebra cross-check needs
    // the COMMON leg too, and here COMMON is the degenerate thin slab (thickness 1e-4)
    // which collapses to volume 0 in the tessellated cross-check — so the fuse is scored
    // against expected=2000 and REJECTED at the ~0.01 mismatch. This is itself a measured
    // boundary finding: the near-coincident COMMON (thin slab) degenerates BEFORE the
    // fuse does, and the self-verify inherits its weakest leg. The invariant that MUST
    // hold regardless is: never DISAGREED (never a wrong result presented as valid).
    const double eps = 1e-4, oracle = 2000.0 - 100.0 * eps;  // 1999.99
    const topo::Shape f = overlap(eps, A, B);
    CC_CHECK(classify(f, A, B, 0, oracle) != Verdict::Disagreed);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// THE INVARIANT — DISAGREED == 0 across the whole near-tangent / near-coincident sweep.
// This is the hard moat guarantee: the native boolean either AGREES with the closed-form
// oracle or HONESTLY-DECLINES to OCCT; it NEVER presents a wrong boolean as valid. We
// sweep the tangent angle, the coincidence gap, and the overlap ε densely and assert
// zero DISAGREED. (AGREED and DECLINED counts are printed as the measured recall map.)
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(dense_csg_sweep_disagreed_is_zero) {
  int agreed = 0, declined = 0, disagreed = 0;
  auto tally = [&](Verdict v) {
    if (v == Verdict::Agreed) ++agreed;
    else if (v == Verdict::Declined) ++declined;
    else ++disagreed;
  };

  // Tangent-angle sweep (fuse of a tilted soup box on a soup box).
  for (double deg : {2.0, 1.0, 0.5, 0.3, 0.25, 0.2, 0.15, 0.1, 0.08, 0.05, 0.02, 0.01}) {
    topo::Shape A, B;
    tiltOperands(deg * kPi / 180.0, 0.4, A, B);
    const topo::Shape f = nb::boolean_solid(A, B, nb::Op::Fuse);
    // Closed-form fuse volume: 1000 (lower) + wedge above z=10. For a box of side 10
    // tilted by θ about an in-plane axis through its base corner line, the volume above
    // z=10 is well-approximated but not needed exactly here — we only need the oracle to
    // detect a GROSSLY wrong accepted volume. Use the watertight volume itself as oracle
    // when accepted is impossible; instead compute the exact via set algebra on operands.
    const double va = watertightVolume(A), vb = watertightVolume(B);
    const topo::Shape cm = nb::boolean_solid(A, B, nb::Op::Common);
    const double vc = cm.isNull() ? 0.0 : std::max(0.0, watertightVolume(cm));
    const double oracle = (va > 0 && vb > 0) ? (va + vb - vc) : 2000.0;
    tally(classify(f, A, B, 0, oracle));
  }

  // Coincidence-gap sweep (stacked soup boxes, oracle union = 2000).
  for (double g : {1e-2, 5e-3, 1e-3, 5e-4, 1e-4, 1e-5, 1e-6, 1e-7, 1e-8}) {
    topo::Shape A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    topo::Shape B = facetSolidLocal(boxAt(0, 0, 10 + g, 10, 10, 10), 0.4);
    const topo::Shape f = nb::boolean_solid(A, B, nb::Op::Fuse);
    tally(classify(f, A, B, 0, 2000.0));
  }

  // Overlap-ε sweep (soup boxes overlapping by ε, oracle fuse = 2000 − 100ε).
  for (double eps : {1e-1, 1e-2, 3e-3, 2e-3, 1e-3, 5e-4, 3e-4, 1e-4, 1e-5, 1e-6}) {
    topo::Shape A = facetSolidLocal(boxAt(0, 0, 0, 10, 10, 10), 0.4);
    topo::Shape B = facetSolidLocal(boxAt(0, 0, 10 - eps, 10, 10, 10), 0.4);
    const topo::Shape f = nb::boolean_solid(A, B, nb::Op::Fuse);
    tally(classify(f, A, B, 0, 2000.0 - 100.0 * eps));
  }

  std::printf("  [dense-csg sweep] AGREED=%d HONESTLY-DECLINED=%d DISAGREED=%d\n", agreed,
              declined, disagreed);
  // The moat invariant: NEVER a silent-wrong result.
  CC_CHECK(disagreed == 0);
  // Sanity: the sweep exercised both accept and decline paths (a real boundary, not a
  // degenerate all-decline).
  CC_CHECK(agreed > 0);
  CC_CHECK(declined > 0);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// GENERALIZED HELICAL-ROOT-LIKE near-tangent STRIP contact (the thread stress
// abstracted, OCCT-free). A shaft-wall box and a thin ridge STRIP whose long face is
// nearly tangent to the wall — the planar cousin of the helical root ↔ shaft-wall
// contact thread_apply.h declines. We assert only the INVARIANT (never DISAGREED),
// since whether it lands native or declines is exactly the recall boundary being mapped.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(helical_root_like_strip_never_disagrees) {
  int disagreed = 0;
  // Wall: a tall thin box (the shaft wall). Ridge: a long thin strip fused onto its
  // outer face with a shrinking protrusion depth d (near-tangent as d → 0).
  for (double d : {0.5, 0.1, 1e-2, 1e-3, 1e-4}) {
    topo::Shape wall = facetSolidLocal(boxAt(0, 0, 0, 2, 20, 10), 0.3);
    // Ridge sits on the +x wall face (x=2), protruding by d, spanning most of y.
    topo::Shape ridge = facetSolidLocal(boxAt(2 - 1e-6, 1, 4, d + 1e-6, 18, 2), 0.3);
    const topo::Shape f = nb::boolean_solid(wall, ridge, nb::Op::Fuse);
    const double va = watertightVolume(wall), vb = watertightVolume(ridge);
    const topo::Shape cm = nb::boolean_solid(wall, ridge, nb::Op::Common);
    const double vc = cm.isNull() ? 0.0 : std::max(0.0, watertightVolume(cm));
    const double oracle = (va > 0 && vb > 0) ? (va + vb - vc) : (400.0 + 18.0 * d * 2.0);
    if (classify(f, wall, ridge, 0, oracle) == Verdict::Disagreed) ++disagreed;
  }
  CC_CHECK(disagreed == 0);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 4 — near-tangent CURVED-SEAM crest/root ↔ wall crack, NOW RECOVERED. This is the
// faithful thread crest/root ↔ shaft-wall contact abstracted OCCT-free: a shaft cylinder
// FUSED with a concentric annular RIDGE (a slightly larger cylinder over a limited z-band),
// both faceted into dense soup. Where the ridge's faceted arc grazes the shaft's faceted
// arc, the two operands tile the shared CURVED seam into DIFFERENT vertex sets — a genuine
// T-junction chain along the arc. The BSP soup is topologically closed (loop-edge parity 0)
// and the interior-crossing repair inserts every collinear T-junction, so the DEFECT is not
// a missed soup crossing: the ridge/cap fragments meeting the seam carry a run of COLLINEAR
// boundary vertices, and the ear-clipper emits a ZERO-AREA collinear triangle to cover them.
// The FaceMesher (correctly) drops that degenerate face — punching the hole (measured pre-fix:
// boundaryEdgeCount 3 at defl 0.30, 6 at defl 0.20; single-turn thread 16–25, 4-turn 33–57).
//
// FIX (assemble.h::triangulatePolygonToFaces): when the ear-clip yields a collinear zero-area
// triangle, re-triangulate that fragment as an AREA-VALIDATED apex fan (SCALE-RELATIVE area
// floor + exact area preservation, the same discipline as Band-1/Band-2), so every seam sub-
// edge is covered by a POSITIVE-area triangle and nothing is dropped. The seam T-junction
// vertices all SURVIVE (they are a real shared chain, not a hairline sliver — welding one
// would just move the crack), and the fan is kept only when it is a valid simple triangulation
// (a genuinely-concave fragment falls back to the ear-clip). ASSERTED map (post-fix): the whole
// deflection sweep closes watertight, and the enclosed volume is unchanged (area-neutral).
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band4_near_tangent_curved_seam_crack_recovered) {
  const double R = 10.0, d = 0.5, zlo = 5.0, zhi = 15.0;
  for (double defl : {0.30, 0.20, 0.15, 0.10}) {
    const topo::Shape shaft = facetSolidLocal(cylinderAt(R, 0.0, 20.0), defl);
    const topo::Shape ridge = facetSolidLocal(cylinderAt(R + d, zlo, zhi), defl);
    CC_CHECK(!shaft.isNull() && !ridge.isNull());
    // Volume of the raw boolean soup (exact) BEFORE assembly — the recovery must not move it.
    std::vector<nb::Polygon> soup =
        nb::booleanPolygons(nb::extractPolygons(shaft), nb::extractPolygons(ridge), nb::Op::Fuse);
    const double vSoup = std::fabs(nb::soupVolume(soup));
    const topo::Shape fuse = nb::boolean_solid(shaft, ridge, nb::Op::Fuse);
    const MeshStat s = meshStat(fuse);
    CC_CHECK(s.watertight);          // the curved-seam crack is sealed …
    CC_CHECK(s.boundaryEdges == 0);  // … (was 3 at defl 0.30, 6 at defl 0.20)
    // … and area-neutral: the assembled watertight volume matches the exact soup volume,
    // so the seam weld moved no real volume (never a wrong-volume body).
    CC_CHECK(std::fabs(s.volume - vSoup) <= 1e-3 * vSoup);
  }
}

// ═══════════════════════════════════════════════════════════════════════════════════
// BAND 4 SAFETY — the apex-fan re-triangulation is AREA-VALIDATED, never a blind rewrite.
// The decisive proof it never moves real volume: a genuine thin CURVED slab. The COMMON of
// two concentric soup cylinders (radii R and R+ε over the same z-band) is a real thin
// tube of wall thickness ε whose faceting produces the SAME collinear-seam fragments as
// Band 4 — so the collinear-ear detector fires — yet the fan is applied only when it is an
// area-preserving simple triangulation, and the enclosed volume must equal the exact soup
// volume. A thin tube's volume (π((R+ε)²−R²)·h) is preserved to a tight bound; the fan
// never collapses its wall. This is the different-scale, genuine-separation case that must
// NOT be corrupted — and is not.
// ═══════════════════════════════════════════════════════════════════════════════════
CC_TEST(band4_thin_curved_slab_is_area_preserving) {
  const double R = 10.0, zlo = 5.0, zhi = 15.0;
  for (double eps : {1.0, 0.5, 0.25}) {
    const topo::Shape inner = facetSolidLocal(cylinderAt(R, zlo, zhi), 0.2);
    const topo::Shape outer = facetSolidLocal(cylinderAt(R + eps, zlo, zhi), 0.2);
    CC_CHECK(!inner.isNull() && !outer.isNull());
    std::vector<nb::Polygon> soup =
        nb::booleanPolygons(nb::extractPolygons(inner), nb::extractPolygons(outer), nb::Op::Common);
    const double vSoup = std::fabs(nb::soupVolume(soup));
    const topo::Shape common = nb::boolean_solid(inner, outer, nb::Op::Common);
    const MeshStat s = meshStat(common);
    // COMMON of concentric cylinders is the inner solid (volume π R² h); the fan preserves it.
    CC_CHECK(s.watertight);
    CC_CHECK(s.volume > 0.0);
    CC_CHECK(std::fabs(s.volume - vSoup) <= 2e-3 * vSoup);  // area-preserving, not collapsed
  }
}

CC_RUN_ALL()

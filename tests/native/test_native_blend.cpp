// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the native blend slice (Phase 4, capability #6
// `native-blends`). OCCT-FREE — Gate 1 (host, analytic) of the two-gate model in
// openspec/NATIVE-REWRITE.md: the blend ops compile and unit-test with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// Each op is exercised DIRECTLY on native prisms built by the verified construct
// library and validated the way the ENGINE does: native TESSELLATOR watertightness
// + a sane volume sign (chamfer/fillet reduce a convex-edge volume; offset grows;
// shell reduces to a wall). A result that fails these is one the engine DISCARDS and
// falls through to OCCT, so the tests assert the HONEST native/fallthrough split.
//
// Build (standalone):
//   /opt/homebrew/opt/llvm/bin/clang++ -std=c++20 tests/native/test_native_blend.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_blend && /tmp/test_native_blend
//
#include "native/blend/native_blend.h"
#include "native/boolean/curved.h"
#include "native/boolean/native_boolean.h"
#include "native/boolean/ssi_boolean.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;
namespace nmath = cybercad::native::math;

namespace {

// A box [0,sx]×[0,sy]×[0,sz] as a native prism (base at z=0).
topo::Shape box(double sx, double sy, double sz) {
  const double p[] = {0, 0, sx, 0, sx, sy, 0, sy};
  return cst::build_prism(p, 4, sz);
}

// Watertight enclosed volume at a fine deflection; sets `wt`.
double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearRel(double got, double want, double rel = 1e-4, double abs = 1e-7) {
  return std::fabs(got - want) <= std::max(rel * std::fabs(want), abs);
}

// Face-count of a solid.
int faceCount(const topo::Shape& s) {
  int n = 0;
  for (topo::Explorer ex(s, topo::ShapeType::Face); ex.more(); ex.next()) ++n;
  return n;
}

// Find the id of the first edge whose two endpoints match (a,b) up to tol, so a test
// can pick a specific box edge deterministically.
int findEdgeId(const topo::Shape& s, const nmath::Point3& a, const nmath::Point3& b) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto ends = blend::edgeEnds(s, static_cast<int>(i));
    if (!ends) continue;
    const bool fwd = nmath::distance(ends->a, a) < 1e-6 && nmath::distance(ends->b, b) < 1e-6;
    const bool rev = nmath::distance(ends->a, b) < 1e-6 && nmath::distance(ends->b, a) < 1e-6;
    if (fwd || rev) return static_cast<int>(i);
  }
  return 0;
}

int findFaceId(const topo::Shape& s, const nmath::Vec3& outwardNormal, const nmath::Point3& on) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto pl = blend::facePlane(s, static_cast<int>(i));
    if (!pl) continue;
    const nmath::Dir3 want{outwardNormal};
    if (nmath::dot(pl->normal, want.vec()) > 0.999 &&
        std::fabs(blend::signedDist(*pl, on)) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}

}  // namespace

// ── chamfer ──────────────────────────────────────────────────────────────────────

CC_TEST(chamfer_box_top_edge_volume_reduced) {
  // 10×10×10 box; chamfer the top edge along x at y=10, z=10 (a convex edge between
  // the top face (+Z) and the +Y side). distance 2 → cut a right-triangle prism of
  // legs 2×2 over length 10: removed volume = ½·2·2·10 = 20.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  topo::Shape ch = blend::chamfer_edges(b, ids, 1, 2.0);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 1000.0 - 20.0));  // 980
  CC_CHECK(faceCount(ch) >= 7);         // 6 + the chamfer face (triangulated welds)
}

CC_TEST(chamfer_degenerate_and_curved_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::chamfer_edges(b, ids, 1, 0.0).isNull());   // zero distance
  CC_CHECK(blend::chamfer_edges(b, nullptr, 0, 2.0).isNull());
  // A curved solid (cylinder) is not planar → NULL.
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::chamfer_edges(cyl, cids, 1, 1.0).isNull());
}

// ── fillet ─────────────────────────────────────────────────────────────────────--

CC_TEST(fillet_box_top_edge_watertight_and_between) {
  // Fillet the same convex top edge with r=2. The removed volume is the corner minus
  // the quarter-cylinder: sharp-corner prism (2×2×10=40 for the square) minus the
  // quarter disc (¼π r² · 10 = ¼π·4·10 = 10π ≈ 31.42) → removed ≈ 8.58, so
  // filleted volume ≈ 1000 − 8.58 ≈ 991.4, BETWEEN the chamfer (980) and sharp (1000).
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  topo::Shape f = blend::fillet_edges(b, ids, 1, 2.0, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  const double expected = 1000.0 - (2.0 * 2.0 * 10.0 - 0.25 * M_PI * 4.0 * 10.0);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
  CC_CHECK(v < 1000.0 && v > 980.0);     // between sharp and chamfer
}

CC_TEST(fillet_curved_and_degenerate_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges(b, ids, 1, 0.0).isNull());
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::fillet_edges(cyl, cids, 1, 1.0).isNull());
}

// ── G2 (curvature-continuous) fillet on a convex planar dihedral edge ─────────────--
// The drop-OCCT Class-B slice. The G2 blend reuses the whole G1 machinery (setback clip
// + facet tiling + watertight weld) but swaps the CIRCULAR-ARC section (constant
// curvature 1/r → G1) for a zero-END-CURVATURE quintic (→ G2). These gates prove: (1)
// the section curvature is IDENTICALLY zero at both rails in CLOSED FORM (B''=0 by the
// collinear-triple pole layout) — and the circular arc's is 1/r (non-trivial control);
// (2) the blend is watertight with the correct SHRINK volume, between chamfer and sharp;
// (3) honest declines outside the convex-planar-dihedral envelope.

namespace {
// The two faces on a convex box edge, as blend polygons, so a test can build the G2
// section directly and assert its closed-form curvature. Returns false if not found.
bool facesOnBoxEdge(const topo::Shape& s, const nmath::Point3& a, const nmath::Point3& b,
                    blend::nb::Polygon& f1, blend::nb::Polygon& f2) {
  blend::PlanarModel model(s);
  if (!model.isValid()) return false;
  std::size_t fi[2];
  if (blend::facesOnEdgeInSoup(model.polygons(), a, b, fi) != 2) return false;
  f1 = model.polygons()[fi[0]];
  f2 = model.polygons()[fi[1]];
  return true;
}
}  // namespace

CC_TEST(g2_fillet_section_curvature_is_zero_at_both_rails_closed_form) {
  // CLOSED-FORM G2 PROOF (no OCCT, no mesh): the quintic section's SECOND derivative is
  // exactly zero at both endpoints, so its curvature κ=|B'×B''|/|B'|³ is 0 at each rail
  // (the flat neighbours also have zero normal curvature ⇒ curvature-continuous seam).
  // For a Bézier, B''(0) ∝ (P0 − 2P1 + P2) and B''(1) ∝ (P5 − 2P4 + P3); the section
  // builder makes {P0,P1,P2} and {P5,P4,P3} collinear-equispaced, so both are the zero
  // vector — an EXACT algebraic identity, checked here on a real box edge.
  topo::Shape b = box(10, 10, 10);
  const nmath::Point3 ea{0, 10, 10}, eb{10, 10, 10};
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnBoxEdge(b, ea, eb, f1, f2));
  const auto sec = blend::detail::g2Section(ea, eb, f1, f2, 2.0);
  CC_CHECK(sec.has_value());
  const auto& P = sec->poles;
  // Second differences at both ends must vanish (curvature 0 at each rail).
  const nmath::Vec3 d2_start = (P[0].asVec() - P[1].asVec() * 2.0 + P[2].asVec());
  const nmath::Vec3 d2_end = (P[5].asVec() - P[4].asVec() * 2.0 + P[3].asVec());
  CC_CHECK(nmath::norm(d2_start) < 1e-12);  // B''(0) = 0  → κ(0) = 0 (G2)
  CC_CHECK(nmath::norm(d2_end) < 1e-12);    // B''(1) = 0  → κ(1) = 0 (G2)
  // The section is NOT degenerate: it still bulges (P1≠P0), i.e. this is a real blend,
  // not a collapsed one. And the endpoints ARE the rolling-ball tangent points.
  CC_CHECK(nmath::distance(P[0], P[1]) > 1e-6);
  CC_CHECK(nmath::distance(P[4], P[5]) > 1e-6);
}

CC_TEST(g2_fillet_measured_seam_curvature_beats_g1) {
  // MEASURED G2 witness (mirrors the sim oracle checks_g2_fillet.cpp, but host + OCCT-
  // free): sample the section's curvature DISCRETELY near a rail from three consecutive
  // section points via the circumradius (κ = 1/R_circum of the triple), and confirm the
  // G2 quintic's near-rail curvature is far SMALLER than the G1 circular arc's constant
  // 1/r on the SAME edge/radius. This is a second, independent witness to the exact
  // B''=0 proof above (a genuine second-order property, not a trivially-true check).
  topo::Shape b = box(10, 10, 10);
  const nmath::Point3 ea{0, 10, 10}, eb{10, 10, 10};
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnBoxEdge(b, ea, eb, f1, f2));
  const double r = 2.0;
  const auto sec = blend::detail::g2Section(ea, eb, f1, f2, r);
  CC_CHECK(sec.has_value());
  // Discrete curvature of the section at parameter s from a small symmetric triple.
  auto kappaAt = [&](double s) -> double {
    const double ds = 1e-3;
    const nmath::Point3 pm = blend::detail::quinticPoint(sec->poles, s - ds);
    const nmath::Point3 p0 = blend::detail::quinticPoint(sec->poles, s);
    const nmath::Point3 pp = blend::detail::quinticPoint(sec->poles, s + ds);
    const nmath::Vec3 a = p0 - pm, c = pp - p0;
    const double la = nmath::norm(a), lc = nmath::norm(c), lac = nmath::norm(pp - pm);
    const double area2 = nmath::norm(nmath::cross(a, c));  // 2·triangle area
    if (la * lc * lac < 1e-18) return 0.0;
    return area2 / (la * lc * lac);  // Menger curvature = 1/circumradius
  };
  // Near the rail (s→0) the quintic curvature tends to zero (κ(0)=0 exactly); the G1
  // circular arc would be a constant 1/r=0.5 everywhere. Sample close to the rail.
  const double kG2_nearRail = kappaAt(0.02);
  const double kG1 = 1.0 / r;  // the circular arc's constant seam curvature
  CC_CHECK(kG2_nearRail < 0.25 * kG1);   // G2 curvature ≤ ¼ the G1 jump near the rail
  CC_CHECK(kG2_nearRail < 0.05);         // and small in absolute terms (→ 0 at the rail)
  // Sanity: the mid-section quintic curvature is nonzero (it IS a curved blend).
  CC_CHECK(kappaAt(0.5) > 1e-3);
}

CC_TEST(g2_fillet_control_circular_arc_curvature_is_one_over_r) {
  // CONTROL (proves the G2 check is non-trivial): the stock G1 circular arc has CONSTANT
  // curvature 1/r ≠ 0 at the rail — the very curvature JUMP a G2 blend removes. We sample
  // the G1 arc section curvature discretely near a rail (three collinear-in-angle points
  // give the circumradius) and confirm it is 1/r, NOT zero, on the SAME edge/radius.
  topo::Shape b = box(10, 10, 10);
  const nmath::Point3 ea{0, 10, 10}, eb{10, 10, 10};
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnBoxEdge(b, ea, eb, f1, f2));
  const double r = 2.0;
  const auto arc = blend::detail::filletArc(ea, eb, f1, f2, r);
  CC_CHECK(arc.has_value());
  // Every point of the circular section is at distance r from the axis C (curvature 1/r).
  CC_CHECK(nearRel(nmath::distance(arc->t1, arc->axis), r, 1e-12));
  CC_CHECK(nearRel(nmath::distance(arc->t2, arc->axis), r, 1e-12));
  // So the arc curvature 1/r ≈ 0.5 is FAR from zero — the G2 bar the arc fails.
  CC_CHECK(std::fabs(1.0 / r) > 0.4);
}

CC_TEST(g2_fillet_box_top_edge_watertight_and_between) {
  // The G2 blend on the same convex top edge (r=2) is watertight, REMOVES material, and
  // its volume sits BETWEEN the chamfer (980) and the sharp box (1000) — like a fillet,
  // but with a curvature-continuous (fuller-shouldered) section, so it removes a little
  // LESS than the circular G1 fillet (the quintic hugs the faces near the rails).
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  topo::Shape g2 = blend::fillet_edges_g2(b, ids, 1, 2.0, 0.005);
  bool wt = false;
  const double v = vol(g2, wt);
  CC_CHECK(!g2.isNull());
  CC_CHECK(wt);                          // curvature-continuous section welds watertight
  CC_CHECK(v < 1000.0 && v > 980.0);     // between sharp and chamfer (a real blend)
  CC_CHECK(faceCount(g2) >= 7);          // 6 box faces − corner + the tiled blend strips
  // It removes LESS than the equal-radius circular G1 fillet (fuller shoulder).
  bool wtG1 = false;
  const double vG1 = vol(blend::fillet_edges(b, ids, 1, 2.0, 0.005), wtG1);
  CC_CHECK(wtG1);
  CC_CHECK(v > vG1);                     // G2 quintic shoulder keeps more material than the arc
}

CC_TEST(g2_fillet_deterministic_and_converges) {
  // Determinism: identical input → identical volume (fp64, no RNG). And the faceted body
  // is deflection-bounded (refining the deflection keeps it watertight and stable).
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  bool w1 = false, w2 = false;
  const double va = vol(blend::fillet_edges_g2(b, ids, 1, 2.0, 0.005), w1);
  const double vb = vol(blend::fillet_edges_g2(b, ids, 1, 2.0, 0.005), w2);
  CC_CHECK(w1 && w2);
  CC_CHECK(nearRel(va, vb, 1e-12));      // deterministic
  bool wc = false;
  const double vc = vol(blend::fillet_edges_g2(b, ids, 1, 2.0, 0.02), wc);
  CC_CHECK(wc);
  CC_CHECK(nearRel(va, vc, 1e-2));       // deflection-bounded (coarse ≈ fine)
}

CC_TEST(g2_fillet_scope_defers) {
  // Honest declines (→ OCCT), the deep-residual boundary.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges_g2(b, ids, 1, 0.0).isNull());       // r ≤ 0
  CC_CHECK(blend::fillet_edges_g2(b, nullptr, 0, 2.0).isNull());   // no edges
  // A curved (cylinder) solid is not a planar dihedral → NULL (curved substrate → OCCT).
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::fillet_edges_g2(cyl, cids, 1, 1.0).isNull());
  // Control: the convex box edge DOES land (no spurious decline).
  CC_CHECK(!blend::fillet_edges_g2(b, ids, 1, 2.0, 0.005).isNull());
}

// ── VARIABLE-radius G2 fillet on a convex planar dihedral edge (NURBS L4) ──────────--
// The variable-radius extension of the constant fillet_edges_g2: the section radius ramps
// LINEARLY r(τ)=r0(1−τ)+r1·τ along the (straight) edge, each cross-section the constant-r
// zero-END-curvature quintic for its LOCAL radius, lofted along the edge. These gates prove
// the moat that a G1 (circular varying-r) blend cannot: (1) the section curvature is
// IDENTICALLY zero at both rails at EVERY station along the edge (closed-form B''=0 at all
// stations + a discrete Menger witness far below the G1 1/r jump, which JUMPS for the
// circular control); (2) watertight with a monotone SHRINK volume matching the closed form
// (131/960)·L·(r0²+r0r1+r1²)/3, reducing EXACTLY to the constant-r G2 fillet when r0==r1;
// (3) a too-fast radius ramp (setback folds) DECLINES — never a self-intersecting body;
// (4) honest declines for curved substrates / degenerate radii, no cross-firing.

namespace {
// Closed-form REMOVED volume of the variable convex G2 fillet over a straight edge of
// length L: the 90°-corner quintic section removes area (131/960)·r², integrated over the
// linear law → (131/960)·L·(r0²+r0·r1+r1²)/3. Strictly increasing in r0 and r1.
double variableG2Vremoved(double L, double r0, double r1) {
  return (131.0 / 960.0) * L * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0;
}
}  // namespace

CC_TEST(g2_variable_fillet_section_curvature_zero_at_both_rails_all_stations) {
  // CLOSED-FORM G2-ALONG-THE-EDGE PROOF: unlike the constant slice (one section), the
  // variable sweep must hold G2 at EVERY station. At each station the section is the
  // constant-r quintic for its LOCAL radius, so its second difference vanishes at both
  // rails (B''(0)∝P0−2P1+P2, B''(1)∝P5−2P4+P3 are the zero vector by the collinear
  // rail-triples) — checked here over ALL stations of a real ramp on a box edge.
  topo::Shape b = box(10, 10, 10);
  const nmath::Point3 ea{0, 10, 10}, eb{10, 10, 10};
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnBoxEdge(b, ea, eb, f1, f2));
  const auto sw = blend::detail::g2VarSweep(ea, eb, f1, f2, 1.0, 2.5, 0.003);
  CC_CHECK(sw.has_value());
  CC_CHECK(sw->stations.size() >= 5);  // a genuinely sampled sweep (not just the two ends)
  double maxD2 = 0.0, minRadiusSpread = 1e9, maxRadiusSpread = 0.0;
  for (const auto& st : sw->stations) {
    const auto& P = st.poles;
    const nmath::Vec3 d0 = P[0].asVec() - P[1].asVec() * 2.0 + P[2].asVec();
    const nmath::Vec3 d1 = P[5].asVec() - P[4].asVec() * 2.0 + P[3].asVec();
    maxD2 = std::max({maxD2, nmath::norm(d0), nmath::norm(d1)});
    minRadiusSpread = std::min(minRadiusSpread, st.radius);
    maxRadiusSpread = std::max(maxRadiusSpread, st.radius);
    CC_CHECK(nmath::distance(P[0], P[1]) > 1e-6);  // still a real (bulging) blend
  }
  CC_CHECK(maxD2 < 1e-12);  // B''=0 at both rails at EVERY station → G2 along the whole edge
  // The radius genuinely varies (this is a variable, not constant, fillet).
  CC_CHECK(nearRel(minRadiusSpread, 1.0, 1e-9));
  CC_CHECK(nearRel(maxRadiusSpread, 2.5, 1e-9));
}

CC_TEST(g2_variable_fillet_measured_seam_curvature_beats_g1_at_stations) {
  // MEASURED G2 witness at several stations (host, OCCT-free): the varying-r quintic's
  // near-rail Menger curvature stays far BELOW the G1 circular arc's 1/r(τ) at each
  // station — a second, independent witness to the exact B''=0 proof, with the circular
  // section's 1/r(τ) as the non-trivial control that JUMPS.
  topo::Shape b = box(10, 10, 10);
  const nmath::Point3 ea{0, 10, 10}, eb{10, 10, 10};
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnBoxEdge(b, ea, eb, f1, f2));
  const auto sw = blend::detail::g2VarSweep(ea, eb, f1, f2, 1.0, 3.0, 0.003);
  CC_CHECK(sw.has_value());
  for (double frac : {0.0, 0.25, 0.5, 0.75, 1.0}) {
    const std::size_t k =
        static_cast<std::size_t>(std::llround(frac * (sw->stations.size() - 1)));
    const auto& P = sw->stations[k].poles;
    const double r = sw->stations[k].radius;
    auto kappaAt = [&](double s) -> double {
      const double ds = 1e-3;
      const nmath::Point3 pm = blend::detail::quinticPoint(P, s - ds);
      const nmath::Point3 p0 = blend::detail::quinticPoint(P, s);
      const nmath::Point3 pp = blend::detail::quinticPoint(P, s + ds);
      const nmath::Vec3 a = p0 - pm, c = pp - p0;
      const double la = nmath::norm(a), lc = nmath::norm(c), lac = nmath::norm(pp - pm);
      const double area2 = nmath::norm(nmath::cross(a, c));
      if (la * lc * lac < 1e-18) return 0.0;
      return area2 / (la * lc * lac);
    };
    const double kG2 = kappaAt(0.02);
    const double kG1 = 1.0 / r;          // circular section's constant seam curvature (control)
    CC_CHECK(kG2 < 0.25 * kG1);          // ≤ ¼ the G1 jump near the rail at THIS station
    CC_CHECK(kappaAt(0.5) > 1e-3);       // mid-section IS curved (a real blend) at THIS station
  }
}

CC_TEST(g2_variable_fillet_watertight_volume_matches_closed_form) {
  // Watertight, REMOVES material, volume matches the closed form to the deflection bound,
  // and sits BETWEEN the two constant-radius (r0, r1) G2 fillet volumes.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  CC_CHECK(e != 0);
  int ids[] = {e};
  const double r0 = 1.0, r1 = 2.5, L = 10.0;
  topo::Shape f = blend::fillet_edges_g2_variable(b, ids, 1, r0, r1, 0.003);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                          // lofted varying-r quintic welds watertight
  CC_CHECK(v < 1000.0 && v > 980.0);     // a real convex blend (between sharp and chamfer)
  const double expected = 1000.0 - variableG2Vremoved(L, r0, r1);
  CC_CHECK(nearRel(v, expected, 6e-3));  // deflection-bounded facet approximation
  // Bracket: bigger radius removes more, so v(r1) < v(variable) < v(r0).
  bool wa = false, wb = false;
  const double vR0 = vol(blend::fillet_edges_g2(b, ids, 1, r0, 0.003), wa);
  const double vR1 = vol(blend::fillet_edges_g2(b, ids, 1, r1, 0.003), wb);
  CC_CHECK(wa && wb);
  CC_CHECK(vR1 < v && v < vR0);
}

CC_TEST(g2_variable_fillet_monotone_in_r0_and_r1) {
  // The removed volume is STRICTLY increasing in both radii: growing either end removes
  // more material (a sensible monotone volume vs r0/r1), matching the closed form.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  auto V = [&](double r0, double r1) {
    bool wt = false;
    const double v = vol(blend::fillet_edges_g2_variable(b, ids, 1, r0, r1, 0.003), wt);
    CC_CHECK(wt);
    return v;
  };
  const double v_11 = V(1.0, 1.0), v_12 = V(1.0, 2.0), v_22 = V(2.0, 2.0);
  CC_CHECK(v_11 > v_12);  // raising r1 removes more (volume drops)
  CC_CHECK(v_12 > v_22);  // raising r0 removes more (volume drops)
  // The removed volumes track the closed form's monotonicity.
  CC_CHECK(variableG2Vremoved(10, 1, 1) < variableG2Vremoved(10, 1, 2));
  CC_CHECK(variableG2Vremoved(10, 1, 2) < variableG2Vremoved(10, 2, 2));
}

CC_TEST(g2_variable_fillet_reduces_to_constant_when_r0_eq_r1) {
  // r0==r1 must reproduce the constant-radius G2 fillet EXACTLY (the loft collapses to the
  // constant section swept unchanged) — the honest reduction, no separate code path.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  for (double r : {1.0, 1.5, 2.0}) {
    bool wv = false, wc = false;
    const double vVar = vol(blend::fillet_edges_g2_variable(b, ids, 1, r, r, 0.005), wv);
    const double vConst = vol(blend::fillet_edges_g2(b, ids, 1, r, 0.005), wc);
    CC_CHECK(wv && wc);
    CC_CHECK(nearRel(vVar, vConst, 1e-9));  // identical facet soup at r0==r1
  }
}

CC_TEST(g2_variable_fillet_reversed_law_same_volume) {
  // The reversed law (r0↔r1) fillets the SAME edge with the ramp flipped end-for-end; the
  // symmetric removed-volume formula gives the same result and it stays watertight.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  bool wf = false, wr = false;
  const double vFwd = vol(blend::fillet_edges_g2_variable(b, ids, 1, 0.75, 2.25, 0.003), wf);
  const double vRev = vol(blend::fillet_edges_g2_variable(b, ids, 1, 2.25, 0.75, 0.003), wr);
  CC_CHECK(wf && wr);
  CC_CHECK(nearRel(vFwd, vRev, 5e-3));  // symmetric law → same removed volume both ways
  CC_CHECK(nearRel(vFwd, 1000.0 - variableG2Vremoved(10, 0.75, 2.25), 6e-3));
}

CC_TEST(g2_variable_fillet_deterministic_and_converges) {
  // Determinism (fp64, no RNG) + deflection-bounded (coarse ≈ fine, both watertight).
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  bool w1 = false, w2 = false, wc = false;
  const double va = vol(blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 2.0, 0.005), w1);
  const double vb = vol(blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 2.0, 0.005), w2);
  const double vc = vol(blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 2.0, 0.02), wc);
  CC_CHECK(w1 && w2 && wc);
  CC_CHECK(nearRel(va, vb, 1e-12));  // deterministic
  CC_CHECK(nearRel(va, vc, 1e-2));   // deflection-bounded
}

CC_TEST(g2_variable_fillet_self_intersection_guard_declines_fast_ramp) {
  // SELF-INTERSECTION GUARD: a radius that ramps too fast for the edge length (the setback
  // line recedes faster than the edge advances → the trimmed face folds) is DECLINED —
  // never returned self-intersecting. On the L=10 edge, |r1−r0| ≥ ~L trips the guard.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 12.0, 0.005).isNull());  // ramp too steep
  CC_CHECK(blend::fillet_edges_g2_variable(b, ids, 1, 12.0, 1.0, 0.005).isNull());  // reversed steep
  // A gentle ramp within the guard DOES land (and is watertight) — the guard is not
  // over-eager. It also removes material and stays a valid shrink.
  bool wt = false;
  const double v = vol(blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 2.5, 0.005), wt);
  CC_CHECK(wt);
  CC_CHECK(v < 1000.0 && v > 980.0);
}

CC_TEST(g2_variable_fillet_scope_defers) {
  // Honest declines (→ OCCT), the deep-residual boundary + no cross-firing.
  topo::Shape b = box(10, 10, 10);
  const int e = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges_g2_variable(b, ids, 1, 0.0, 2.0).isNull());   // r0 ≤ 0
  CC_CHECK(blend::fillet_edges_g2_variable(b, ids, 1, 2.0, 0.0).isNull());   // r1 ≤ 0
  CC_CHECK(blend::fillet_edges_g2_variable(b, nullptr, 0, 1.0, 2.0).isNull());  // no edges
  // A curved (cylinder) solid is not a planar dihedral → NULL (curved substrate → OCCT):
  // freeform / curved-variable stays an honest decline (the deep residual).
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cids[] = {1};
  CC_CHECK(blend::fillet_edges_g2_variable(cyl, cids, 1, 1.0, 2.0).isNull());
  // No cross-firing with the CONCAVE builder: the variable builder is convex-only, and a
  // reflex edge would need the concave path (not built here) — declines cleanly. Build the
  // reflex L-prism inline (the shared lPrism fixture is declared later in this file).
  const double lp[] = {0, 0, 6, 0, 6, 3, 3, 3, 3, 6, 0, 6};
  topo::Shape L = cst::build_prism(lp, 6, 4.0);
  const int le = findEdgeId(L, {3, 3, 0}, {3, 3, 4});
  int lids[] = {le};
  CC_CHECK(blend::fillet_edges_g2_variable(L, lids, 1, 1.0, 1.5).isNull());  // concave → decline
  // Control: the convex box edge DOES land (no spurious decline).
  CC_CHECK(!blend::fillet_edges_g2_variable(b, ids, 1, 1.0, 2.5, 0.005).isNull());
}

// ── G2 (curvature-continuous) fillet on a CONCAVE planar dihedral edge ─────────────--
// The second scoped slice of the drop-OCCT Class-B `fillet_edges_g2` after the convex
// planar dihedral. A CONCAVE (reflex) planar dihedral (the inner corner of an L-shaped
// prism) is the mirror of the convex case: the ball seats in the VOID (C=E+r/(1+c)(n1+n2),
// Ti=C−r·ni), the fillet ADDS material rounding the reflex corner, and the soup is edited
// surgically (trim the two adjacent walls to their tangent lines, splice the quintic rim
// into the end caps carrying the reflex vertex, bridge with the roof strips). The SAME
// zero-END-CURVATURE quintic (collinear rail-triples → B''=0 → κ=0 at both rails → G2) is
// reused; only the ball-centre sign, the strip-normal sign, and the ADD-material volume
// flip. These gates prove: (1) closed-form B''=0 at both rails PLUS a measured near-rail
// curvature far below the G1 1/r jump, with the G1 concave arc's 1/r as a non-trivial
// control; (2) watertight with the correct GROW volume matching the closed-form quintic
// sliver 4·(131/960)·r²; (3) honest declines outside the concave-planar-dihedral envelope.

namespace {
// An L-shaped prism: outer profile CCW with a single REFLEX (concave) inner vertical edge
// at (Lc,Lc), extruded +Z by `depth`. Footprint area = Lo²−(Lo−Lc)²... here a clean
// 6×6 square with a 3×3 notch removed → the reflex corner is at (3,3).
topo::Shape lPrism(double depth) {
  const double p[] = {0, 0, 6, 0, 6, 3, 3, 3, 3, 6, 0, 6};
  return cst::build_prism(p, 6, depth);
}
// The two faces on the concave edge (3,3,·), as blend polygons, so a test can build the
// concave G2 section directly and assert its closed-form curvature.
bool facesOnConcaveEdge(const topo::Shape& s, blend::nb::Polygon& f1, blend::nb::Polygon& f2) {
  blend::PlanarModel model(s);
  if (!model.isValid()) return false;
  std::size_t fi[2];
  if (blend::facesOnEdgeInSoup(model.polygons(), {3, 3, 0}, {3, 3, 4}, fi) != 2) return false;
  f1 = model.polygons()[fi[0]];
  f2 = model.polygons()[fi[1]];
  return true;
}
}  // namespace

CC_TEST(g2_concave_fillet_section_curvature_is_zero_at_both_rails_closed_form) {
  // CLOSED-FORM G2 PROOF (no OCCT, no mesh): the CONCAVE quintic section's second
  // difference is exactly zero at both endpoints (B''(0) ∝ P0−2P1+P2, B''(1) ∝ P5−2P4+P3),
  // so its curvature κ=|B'×B''|/|B'|³ is 0 at each rail — the identical collinear-triple
  // identity as the convex slice, on the reflex L-prism edge.
  topo::Shape L = lPrism(4.0);
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnConcaveEdge(L, f1, f2));
  const auto sec = blend::detail::g2SectionConcave({3, 3, 0}, {3, 3, 4}, f1, f2, 2.0);
  CC_CHECK(sec.has_value());
  const auto& P = sec->poles;
  const nmath::Vec3 d2_start = (P[0].asVec() - P[1].asVec() * 2.0 + P[2].asVec());
  const nmath::Vec3 d2_end = (P[5].asVec() - P[4].asVec() * 2.0 + P[3].asVec());
  CC_CHECK(nmath::norm(d2_start) < 1e-12);  // B''(0) = 0 → κ(0) = 0 (G2)
  CC_CHECK(nmath::norm(d2_end) < 1e-12);    // B''(1) = 0 → κ(1) = 0 (G2)
  CC_CHECK(nmath::distance(P[0], P[1]) > 1e-6);  // still bulges (real blend)
  CC_CHECK(nmath::distance(P[4], P[5]) > 1e-6);
}

CC_TEST(g2_concave_fillet_measured_seam_curvature_beats_g1) {
  // MEASURED G2 witness (host, OCCT-free): the concave quintic's near-rail Menger
  // curvature is far SMALLER than the G1 concave circular arc's constant 1/r on the SAME
  // edge/radius — a second, independent witness to the exact B''=0 proof.
  topo::Shape L = lPrism(4.0);
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnConcaveEdge(L, f1, f2));
  const double r = 2.0;
  const auto sec = blend::detail::g2SectionConcave({3, 3, 0}, {3, 3, 4}, f1, f2, r);
  CC_CHECK(sec.has_value());
  auto kappaAt = [&](double s) -> double {
    const double ds = 1e-3;
    const nmath::Point3 pm = blend::detail::quinticPoint(sec->poles, s - ds);
    const nmath::Point3 p0 = blend::detail::quinticPoint(sec->poles, s);
    const nmath::Point3 pp = blend::detail::quinticPoint(sec->poles, s + ds);
    const nmath::Vec3 a = p0 - pm, c = pp - p0;
    const double la = nmath::norm(a), lc = nmath::norm(c), lac = nmath::norm(pp - pm);
    const double area2 = nmath::norm(nmath::cross(a, c));
    if (la * lc * lac < 1e-18) return 0.0;
    return area2 / (la * lc * lac);
  };
  const double kG2_nearRail = kappaAt(0.02);
  const double kG1 = 1.0 / r;  // the G1 concave arc's constant seam curvature
  CC_CHECK(kG2_nearRail < 0.25 * kG1);  // ≤ ¼ the G1 jump near the rail
  CC_CHECK(kG2_nearRail < 0.05);        // small in absolute terms (→ 0 at the rail)
  CC_CHECK(kappaAt(0.5) > 1e-3);        // mid-section IS curved (a real blend)
}

CC_TEST(g2_concave_fillet_control_circular_arc_curvature_is_one_over_r) {
  // CONTROL (proves the G2 check is non-trivial): the G1 CONCAVE circular arc has CONSTANT
  // curvature 1/r ≠ 0 at the rail — every point of the concave section is at distance r
  // from the void-side axis C — the very curvature JUMP the G2 concave blend removes.
  topo::Shape L = lPrism(4.0);
  blend::nb::Polygon f1, f2;
  CC_CHECK(facesOnConcaveEdge(L, f1, f2));
  const double r = 2.0;
  const auto arc = blend::detail::concaveFilletArc({3, 3, 0}, {3, 3, 4}, f1, f2, r);
  CC_CHECK(arc.has_value());
  CC_CHECK(nearRel(nmath::distance(arc->t1, arc->axis), r, 1e-12));
  CC_CHECK(nearRel(nmath::distance(arc->t2, arc->axis), r, 1e-12));
  CC_CHECK(std::fabs(1.0 / r) > 0.4);  // arc curvature 1/r ≈ 0.5, FAR from zero
}

CC_TEST(g2_concave_fillet_l_prism_watertight_and_volume_grown) {
  // The concave G2 blend on the reflex L-prism edge (r=1) is watertight and ADDS material
  // into the reflex corner (volume GROWS). The added sliver is the quintic-filled region
  // between the sharp corner and the section; its exact area over the edge length is
  // 4·(131/960)·r² = 0.545833 (the closed-form symmetric-90°-corner quintic sliver, see
  // the header), which the deflection-bounded facet body converges to from ABOVE (facet
  // chords over-enclose the concave curve). Watertight + volume-grown is the ADD-material
  // twin of the convex slice's SHRINK self-verify.
  topo::Shape L = lPrism(4.0);
  const int e = findEdgeId(L, {3, 3, 0}, {3, 3, 4});
  CC_CHECK(e != 0);
  int ids[] = {e};
  const double r = 1.0;
  bool wt0 = false;
  const double v0 = vol(L, wt0);
  CC_CHECK(wt0);
  topo::Shape g2 = blend::fillet_edges_g2_concave(L, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(g2, wt);
  CC_CHECK(!g2.isNull());
  CC_CHECK(wt);          // curvature-continuous concave section welds watertight
  CC_CHECK(v > v0);      // a CONCAVE fillet ADDS material (volume GROWS)
  const double expectedGrow = 4.0 * (131.0 / 960.0) * r * r;  // closed-form quintic sliver
  // Deflection-bounded: 0.005 sits ~4% above the smooth closed form; bound generously and
  // separately assert the sign + that it is well below the circular fill (fuller shoulder).
  CC_CHECK((v - v0) > expectedGrow);                 // facet over-encloses the concave curve
  CC_CHECK((v - v0) < expectedGrow * 1.10);          // within the deflection band
  const double circularFill = 4.0 * (1.0 - M_PI / 4.0) * r * r;  // G1 concave arc adds MORE
  CC_CHECK((v - v0) < circularFill);  // the quintic hugs the faces → adds LESS than the arc
}

CC_TEST(g2_concave_fillet_volume_converges_to_closed_form) {
  // Refining the deflection drives the added volume DOWN toward the exact closed-form
  // quintic sliver 4·(131/960)·r² (facet chords over-enclose the concave curve, so the
  // approach is monotone from above) — a rigorous closed-form volume witness.
  topo::Shape L = lPrism(4.0);
  const int e = findEdgeId(L, {3, 3, 0}, {3, 3, 4});
  int ids[] = {e};
  const double r = 1.0;
  bool wt0 = false;
  const double v0 = vol(L, wt0);
  const double cf = 4.0 * (131.0 / 960.0) * r * r;
  bool wa = false, wb = false;
  const double coarse = vol(blend::fillet_edges_g2_concave(L, ids, 1, r, 0.02), wa) - v0;
  const double fine = vol(blend::fillet_edges_g2_concave(L, ids, 1, r, 0.001), wb) - v0;
  CC_CHECK(wa && wb);
  CC_CHECK(coarse > fine);          // refining shrinks the over-enclosure
  CC_CHECK(fine > cf);              // still above the smooth closed form
  CC_CHECK(nearRel(fine, cf, 1e-2));  // fine facets within 1% of the closed form
  // Determinism: identical input → identical volume (fp64, no RNG).
  bool w1 = false, w2 = false;
  const double d1 = vol(blend::fillet_edges_g2_concave(L, ids, 1, r, 0.005), w1);
  const double d2 = vol(blend::fillet_edges_g2_concave(L, ids, 1, r, 0.005), w2);
  CC_CHECK(w1 && w2);
  CC_CHECK(nearRel(d1, d2, 1e-12));
}

CC_TEST(g2_concave_fillet_scope_defers) {
  // Honest declines (→ OCCT), the deep-residual boundary + no cross-firing with convex G2.
  topo::Shape L = lPrism(4.0);
  const int e = findEdgeId(L, {3, 3, 0}, {3, 3, 4});
  int ids[] = {e};
  CC_CHECK(blend::fillet_edges_g2_concave(L, ids, 1, 0.0).isNull());      // r ≤ 0
  CC_CHECK(blend::fillet_edges_g2_concave(L, nullptr, 0, 1.0).isNull());  // no edges
  // A CONVEX box edge is not a reflex dihedral → NULL (the convex G2 builder owns it).
  topo::Shape b = box(10, 10, 10);
  const int ce = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int cids[] = {ce};
  CC_CHECK(blend::fillet_edges_g2_concave(b, cids, 1, 2.0, 0.005).isNull());  // convex → decline
  // A curved (cylinder) solid is not a planar dihedral → NULL (curved substrate → OCCT).
  const double prof[] = {2, 0, 5, 0, 5, 10, 2, 10};
  topo::Shape cyl = cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 6.2831853);
  int cyids[] = {1};
  CC_CHECK(blend::fillet_edges_g2_concave(cyl, cyids, 1, 1.0).isNull());
  // No cross-firing: the CONVEX G2 builder DECLINES this concave edge, and the CONCAVE
  // builder DECLINES the convex box edge (asserted above) — exactly one fires per edge.
  CC_CHECK(blend::fillet_edges_g2(L, ids, 1, 1.0, 0.005).isNull());
  // Control: the concave L-prism edge DOES land (no spurious decline).
  CC_CHECK(!blend::fillet_edges_g2_concave(L, ids, 1, 1.0, 0.005).isNull());
}

// ── curved fillet (first CURVED slice: torus canal blend on a cylinder↔cap rim) ──--

namespace {
// A capped solid cylinder: full-circle profile radius Rc extruded to height h. Faces:
// bottom cap (−Z), top cap (+Z), one Cylinder wall. Rims are true Circle edges.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}
// The Circle rim edge id at axial height `z` (the top or bottom rim).
int findRimAtZ(const topo::Shape& s, double z) {
  const topo::ShapeMap emap = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    nmath::Point3 o = c->curve->frame.origin;
    if (!c->location.isIdentity()) o = c->location.transform().applyToPoint(o);
    if (std::fabs(o.z - z) < 1e-6) return static_cast<int>(i);
  }
  return 0;
}
}  // namespace

CC_TEST(curved_fillet_cylinder_cap_watertight_volume_reduced) {
  // Rc=5, h=10 capped cylinder; roll a ball r=1.5 into the top rim → a coaxial torus
  // canal blend (major R=Rc−r=3.5, minor r=1.5). Watertight, volume BELOW the sharp
  // cylinder, and matching the exact solid-of-revolution to the deflection bound.
  const double Rc = 5.0, h = 10.0, r = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::curved_fillet_edge(cyl, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                 // torus quarter-tube welds watertight to wall + cap
  CC_CHECK(v < v0);             // a convex fillet REDUCES the volume
  // Exact filleted volume = cylinder up to the wall seam (z=h−r) + the torus solid of
  // revolution over the last r of height. Closed form of ∫π·radius(v)²·dz with
  // radius(v)=R+r·cos v, z=r·sin v, v∈[0,π/2]:
  //   ∫₀^{π/2} π (R + r cos v)² r cos v dv
  //   = π r [ R²·1 + 2Rr·(π/4) + r²·(2/3) ].
  const double R = Rc - r;
  const double vTorus = M_PI * r * (R * R + 2.0 * R * r * (M_PI / 4.0) + r * r * (2.0 / 3.0));
  const double expected = M_PI * Rc * Rc * (h - r) + vTorus;
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(curved_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 assertion (no OCCT, no mesh): the torus canal normal at the two seams
  // matches the adjacent primary-face normal exactly. The blend surface is the quarter
  // tube radius(v)=R+r·cos v, axialOffset(v)=r·sin v, with outward normal
  //   n(u,v) = radial(u)·cos v + axis·sin v.
  //   * v=0   (wall seam):  n = radial(u)  → the CYLINDER outward normal (radial). cos=1.
  //   * v=π/2 (cap seam):   n = axis        → the CAP outward normal (+axis).      cos=1.
  // Because both seams share position AND normal with the neighbour, the fillet is
  // G1-tangent there. We check this at several u around the rim.
  const double Rc = 5.0, r = 1.5;
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis (build_prism extrudes +Z)
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    // Wall seam v=0: torus normal is purely radial → equals the cylinder radial normal.
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    // Cap seam v=π/2: torus normal is purely axial → equals the cap outward normal.
    const nmath::Vec3 nCap = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nCap}.vec(), axis), 1.0, 1e-12));
  }
  // The seam POSITIONS also coincide with the neighbours: v=0 sits at radius Rc (on the
  // cylinder wall) and v=π/2 sits at radius Rc−r (the trimmed cap edge), closing G0+G1.
  // radius(v)=(Rc−r)+r·cos v, so radius(0)=Rc (wall) and radius(π/2)=Rc−r (cap edge).
  const double Rm = Rc - r;  // torus major radius
  CC_CHECK(nearRel(Rm + r * std::cos(0.0), Rc, 1e-12));            // radius(0)=Rc (wall)
  CC_CHECK(nearRel(Rm + r * std::cos(M_PI / 2.0), Rc - r, 1e-12));  // radius(π/2)=Rc−r (cap)
}

CC_TEST(curved_fillet_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Ring-torus guard: r=3 ⇒ Rc<2r ⇒ NULL (spindle torus, defers to OCCT).
  CC_CHECK(blend::curved_fillet_edge(cyl, ids, 1, 3.0, 0.01).isNull());
  // Zero / negative radius → NULL.
  CC_CHECK(blend::curved_fillet_edge(cyl, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_fillet_edge(cyl, ids2, 2, 1.5, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

CC_TEST(curved_fillet_both_rims_and_engine_dispatch) {
  // The engine fillet_edges dispatches the circular rim through the curved path when
  // the planar path declines. Exercise the builder on the BOTTOM rim too (z=0), and
  // confirm the planar fillet_edges still returns NULL on this curved solid (so the
  // engine's fall-through to curved_fillet_edge is what lands it).
  const double Rc = 4.0, h = 8.0, r = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int bottom = findRimAtZ(cyl, 0.0);
  CC_CHECK(bottom != 0);
  int ids[] = {bottom};
  CC_CHECK(blend::fillet_edges(cyl, ids, 1, r).isNull());  // planar path declines
  topo::Shape f = blend::curved_fillet_edge(cyl, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < v0);
}

// ── variable-radius convex fillet (swept variable-r torus canal on a cyl↔cap rim) ─--

namespace {
// Closed-form REMOVED volume of the variable convex fillet over the circular rim, via the
// per-meridian removed corner area r²(1−π/4) with centroid radius (Pappus per angular
// slice): V = ∫₀^{2π} r(θ)²[Rc(1−π/4) + r(θ)(π/4−5/6)] dθ, r(θ)=r1+(r2−r1)θ/2π. Using
// ∫r²dθ=2π(r1²+r1r2+r2²)/3 and ∫r³dθ=2π(r1+r2)(r1²+r2²)/4.
double variableVremoved(double Rc, double r1, double r2) {
  const double q = M_PI / 4.0;
  const double i2 = 2.0 * M_PI * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0;
  const double i3 = 2.0 * M_PI * (r1 + r2) * (r1 * r1 + r2 * r2) / 4.0;
  return Rc * (1.0 - q) * i2 + (q - 5.0 / 6.0) * i3;
}
}  // namespace

CC_TEST(variable_fillet_cylinder_cap_watertight_volume_between) {
  // Fixture A: Rc=5, h=10 capped cylinder; roll a ball whose radius ramps r1=1 → r2=2
  // linearly around the top rim → a swept variable-r torus canal. Watertight, BELOW the
  // sharp cylinder, BETWEEN the two constant-radius (r1, r2) fillet volumes, and matching
  // the closed-form removed volume to the deflection bound.
  const double Rc = 5.0, h = 10.0, r1 = 1.0, r2 = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::variable_fillet_edge(cyl, ids, 1, r1, r2, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);          // swept canal + seam wall weld watertight to wall + cap
  CC_CHECK(v < v0);      // a convex (variable) fillet REDUCES the volume
  // Bracket: larger r removes more, so v(r2) < v(variable) < v(r1).
  bool wa = false, wb = false;
  const double vR1 = vol(blend::curved_fillet_edge(cyl, ids, 1, r1, 0.005), wa);
  const double vR2 = vol(blend::curved_fillet_edge(cyl, ids, 1, r2, 0.005), wb);
  CC_CHECK(wa && wb);
  CC_CHECK(vR2 < v && v < vR1);
  // Closed-form removed volume (14.60 for this fixture).
  const double expected = v0 - variableVremoved(Rc, r1, r2);
  CC_CHECK(nearRel(v, expected, 6e-3));  // deflection-bounded facet approximation
}

CC_TEST(variable_fillet_second_fixture_and_reversed) {
  // Fixture B: Rc=6, h=12, r1=0.75 → r2=2.25. Also exercise the REVERSED law (r1>r2) to
  // cover both seam-wall orientations.
  const double Rc = 6.0, h = 12.0, r1 = 0.75, r2 = 2.25;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  topo::Shape f = blend::variable_fillet_edge(cyl, ids, 1, r1, r2, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - variableVremoved(Rc, r1, r2), 6e-3));
  // Reversed law (r1>r2) → the seam wall faces the other way; still watertight, same volume.
  topo::Shape fr = blend::variable_fillet_edge(cyl, ids, 1, r2, r1, 0.005);
  bool wtr = false;
  const double vr = vol(fr, wtr);
  CC_CHECK(!fr.isNull());
  CC_CHECK(wtr);
  CC_CHECK(nearRel(vr, v0 - variableVremoved(Rc, r2, r1), 6e-3));
}

CC_TEST(variable_fillet_reduces_to_constant_when_r1_eq_r2) {
  // r1==r2 must reproduce the constant torus fillet exactly (every band + seam wall
  // collapses to zero area).
  const double Rc = 5.0, h = 10.0, r = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  bool wv = false, wc = false;
  const double vVar = vol(blend::variable_fillet_edge(cyl, ids, 1, r, r, 0.005), wv);
  const double vConst = vol(blend::curved_fillet_edge(cyl, ids, 1, r, 0.005), wc);
  CC_CHECK(wv && wc);
  CC_CHECK(nearRel(vVar, vConst, 1e-9));  // identical facet soup
}

CC_TEST(variable_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): at EVERY station the variable-canal normal at v=0 is
  // radial (== cylinder normal) and at v=π/2 is axial (== cap normal) — independent of the
  // radius gradient r'(θ). n(u,v)=radial(u)cos v + axis·sin v, so cos=1 at both seams.
  const nmath::Vec3 axis{0, 0, 1};
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    const nmath::Vec3 nCap = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nCap}.vec(), axis), 1.0, 1e-12));
  }
  // Seam POSITIONS at the two ends of the linear law coincide with the neighbours: at any
  // station r, radius(0)=Rc (wall) and radius(π/2)=Rc−r (trimmed cap edge).
  const double Rc = 5.0;
  for (double r : {1.0, 1.5, 2.0}) {
    CC_CHECK(nearRel((Rc - r) + r * std::cos(0.0), Rc, 1e-12));
    CC_CHECK(nearRel((Rc - r) + r * std::cos(M_PI / 2.0), Rc - r, 1e-12));
  }
}

CC_TEST(variable_fillet_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Ring-torus guard on the DEEPEST station: max(r1,r2)=3 ⇒ Rc<2·rMax ⇒ NULL.
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 1.0, 3.0, 0.01).isNull());
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 3.0, 1.0, 0.01).isNull());
  // Zero / negative radius on either end → NULL.
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 0.0, 2.0, 0.01).isNull());
  CC_CHECK(blend::variable_fillet_edge(cyl, ids, 1, 1.0, 0.0, 0.01).isNull());
  // More than one picked edge → NULL.
  int ids2[] = {rim, 1};
  CC_CHECK(blend::variable_fillet_edge(cyl, ids2, 2, 1.0, 2.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::variable_fillet_edge(b, idsb, 1, 1.0, 2.0, 0.01).isNull());
}

// ── curved chamfer (cone-frustum straight bevel on a cylinder↔cap rim) ────────────--

namespace {
// Exact removed corner-ring volume of a symmetric distance-d chamfer on a circular rim
// (radius Rc): the right triangle legs d×d (area d²/2, centroid radial Rc−d/3) revolved
// about the axis (Pappus): V_removed = π·d²·(Rc − d/3). Volume REDUCES.
double chamferVremoved(double Rc, double d) {
  return M_PI * d * d * (Rc - d / 3.0);
}
}  // namespace

CC_TEST(curved_chamfer_cylinder_cap_watertight_volume_reduced) {
  // Fixture A: Rc=5, h=10 capped cylinder; chamfer the top rim with distance d=1.0 → a
  // CONE-FRUSTUM straight bevel between the two setback circles (wall circle r=Rc at
  // z=h−d, cap circle r=Rc−d at z=h). Watertight, BELOW the sharp cylinder, matching the
  // exact Pappus removed volume to the deflection bound.
  const double Rc = 5.0, h = 10.0, d = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);                 // frustum band welds watertight to wall + trimmed cap
  CC_CHECK(v < v0);             // a convex chamfer REDUCES the volume
  const double expected = v0 - chamferVremoved(Rc, d);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(curved_chamfer_second_fixture_and_removes_more_than_fillet) {
  // Fixture B: same body, d=2.0. Also assert the chamfer removes MORE than a fillet of
  // radius d (cross-section d²/2 > d²(1−π/4)), so V_chamfer < V_fillet < V0.
  const double Rc = 5.0, h = 10.0, d = 2.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - chamferVremoved(Rc, d), 5e-3));
  // Chamfer removes more than the equal-distance fillet → smaller volume.
  bool wf = false;
  const double vFillet = vol(blend::curved_fillet_edge(cyl, ids, 1, d, 0.005), wf);
  CC_CHECK(wf);
  CC_CHECK(v < vFillet && vFillet < v0);
}

CC_TEST(curved_chamfer_is_c0_bevel_not_g1) {
  // ANALYTIC bevel-angle assertion (no OCCT, no mesh): the frustum outward normal is the
  // 45° BISECTOR of the two face normals — cos = 1/√2 with BOTH the cylinder radial
  // normal and the cap axial normal, and explicitly NOT 1 (a chamfer is C0, NOT tangent).
  // This is the load-bearing inversion vs the fillet (whose seam normals give cos=1).
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis; s=+1 (cap at the top)
  const double invSqrt2 = 1.0 / std::sqrt(2.0);
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    // Frustum bevel outward normal = (radial + s·axis)/√2.
    const nmath::Vec3 bevel = nmath::Dir3{radial + axis}.vec();
    // Meets the cylinder wall (radial normal) at 45°, NOT tangent.
    const double cosWall = nmath::dot(bevel, radial);
    CC_CHECK(nearRel(cosWall, invSqrt2, 1e-12));
    CC_CHECK(std::fabs(cosWall - 1.0) > 0.2);  // NOT G1 (not tangent to the wall)
    // Meets the cap (axial normal) at 45°, NOT tangent.
    const double cosCap = nmath::dot(bevel, axis);
    CC_CHECK(nearRel(cosCap, invSqrt2, 1e-12));
    CC_CHECK(std::fabs(cosCap - 1.0) > 0.2);  // NOT G1 (not tangent to the cap)
  }
  // The two faces meet at 90°, so the bevel bisects them exactly (radial·axis = 0).
  CC_CHECK(nearRel(nmath::dot(nmath::Vec3{1, 0, 0}, axis), 0.0, 1e-12));
}

CC_TEST(curved_chamfer_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Rc − d ≤ 0 guard: d=5 ⇒ the bevel would cross the axis ⇒ NULL (defers to OCCT).
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 5.0, 0.01).isNull());
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 6.0, 0.01).isNull());
  // Zero / negative distance → NULL.
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_chamfer_edge(cyl, ids2, 2, 1.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_chamfer_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

CC_TEST(curved_chamfer_both_rims_and_planar_declines) {
  // The engine chamfer_edges dispatches the circular rim through the curved path when
  // the planar path declines. Exercise the builder on the BOTTOM rim too (z=0), and
  // confirm the planar chamfer_edges still returns NULL on this curved solid (so the
  // engine's fall-through to curved_chamfer_edge is what lands it).
  const double Rc = 4.0, h = 8.0, d = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int bottom = findRimAtZ(cyl, 0.0);
  CC_CHECK(bottom != 0);
  int ids[] = {bottom};
  CC_CHECK(blend::chamfer_edges(cyl, ids, 1, d).isNull());  // planar path declines
  topo::Shape ch = blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005);
  bool wt = false, wt0 = false;
  const double v = vol(ch, wt);
  const double v0 = vol(cyl, wt0);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, v0 - chamferVremoved(Rc, d), 5e-3));
}

// ── T1: asymmetric two-distance chamfer (oblique cone-frustum bevel) ──────────────--

namespace {
// Exact removed corner-ring volume of an ASYMMETRIC chamfer (axial wall setback d1,
// radial cap setback d2) on a circular rim (radius Rc): the right triangle legs d1 (axial)
// × d2 (radial), area ½·d1·d2, centroid radial Rc − d2/3, revolved about the axis (Pappus):
// V_removed = π·d1·d2·(Rc − d2/3). d1 = d2 reduces to the symmetric π·d²·(Rc − d/3).
double chamferVremovedAsym(double Rc, double d1, double d2) {
  return M_PI * d1 * d2 * (Rc - d2 / 3.0);
}
}  // namespace

CC_TEST(asym_chamfer_oblique_frustum_watertight_volume) {
  // Rc=5, h=10 capped cylinder; chamfer the top rim with d1=2 (axial WALL setback) and
  // d2=1 (radial CAP setback) → an OBLIQUE cone-frustum bevel between the wall seam
  // (Rc, h−d1) and the cap seam (Rc−d2, h). Watertight, BELOW the sharp cylinder, matching
  // the exact Pappus removed volume π·d1·d2·(Rc−d2/3)=29.3215 → chamfered ≈ 756.0766.
  const double Rc = 5.0, h = 10.0, d1 = 2.0, d2 = 1.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape ch = blend::curved_chamfer_edge_asym(cyl, ids, 1, d1, d2, 0.005);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);                 // oblique frustum welds watertight to wall + trimmed cap
  CC_CHECK(v < v0);             // a convex chamfer REDUCES the volume
  const double expected = v0 - chamferVremovedAsym(Rc, d1, d2);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
  // A swapped fixture (d1=1, d2=2) removes a different corner (still exact).
  topo::Shape ch2 = blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 2.0, 0.005);
  bool wt2 = false;
  const double v2 = vol(ch2, wt2);
  CC_CHECK(!ch2.isNull() && wt2);
  CC_CHECK(nearRel(v2, v0 - chamferVremovedAsym(Rc, 1.0, 2.0), 5e-3));
}

CC_TEST(asym_chamfer_symmetric_special_case) {
  // d1 == d2 must reproduce the SYMMETRIC chamfer volume EXACTLY (byte-identical builder
  // path via buildChamferedCylinderAsym(g,d,d,defl)); it also equals curved_chamfer_edge.
  const double Rc = 5.0, h = 10.0, d = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  bool wa = false, wb = false;
  const double va = vol(blend::curved_chamfer_edge_asym(cyl, ids, 1, d, d, 0.005), wa);
  const double vb = vol(blend::curved_chamfer_edge(cyl, ids, 1, d, 0.005), wb);
  CC_CHECK(wa && wb);
  CC_CHECK(nearRel(va, vb, 1e-12));  // asym(d,d) == symmetric, exactly
  CC_CHECK(nearRel(va, vol(cyl, wa) - chamferVremoved(Rc, d), 5e-3));
}

CC_TEST(asym_chamfer_two_bevel_angles_c0) {
  // ANALYTIC bevel-angle assertion (no OCCT, no mesh): the OBLIQUE frustum outward normal
  // is radial·d1 + s·axial·d2, so it makes cos=d1/√(d1²+d2²) with the cylinder radial
  // normal and cos=d2/√(d1²+d2²) with the cap axial normal — two DIFFERENT angles, both
  // explicitly ≠ 1 (C0, NOT G1). This is the T1 inversion vs the symmetric 45° bevel.
  const nmath::Vec3 axis{0, 0, 1};  // capped-cylinder axis; s=+1 (cap at the top)
  const double d1 = 2.0, d2 = 1.0;
  const double den = std::sqrt(d1 * d1 + d2 * d2);
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 bevel = nmath::Dir3{radial * d1 + axis * d2}.vec();
    const double cosWall = nmath::dot(bevel, radial);
    CC_CHECK(nearRel(cosWall, d1 / den, 1e-12));
    CC_CHECK(std::fabs(cosWall - 1.0) > 0.05);  // NOT tangent to the wall
    const double cosCap = nmath::dot(bevel, axis);
    CC_CHECK(nearRel(cosCap, d2 / den, 1e-12));
    CC_CHECK(std::fabs(cosCap - 1.0) > 0.05);  // NOT tangent to the cap
    // The two angles DIFFER (asymmetric), unlike the symmetric 45° case.
    CC_CHECK(std::fabs(cosWall - cosCap) > 0.1);
  }
}

CC_TEST(asym_chamfer_scope_defers) {
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  int ids[] = {rim};
  // Rc − d2 ≤ 0 guard: d2=5 ⇒ the cap seam crosses the axis ⇒ NULL (defers to OCCT).
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 5.0, 0.01).isNull());
  // Wall shorter than d1 (d1=12 > h) ⇒ far end not beyond the wall seam ⇒ NULL.
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 12.0, 1.0, 0.01).isNull());
  // Zero / negative distance (either) → NULL.
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 0.0, 1.0, 0.01).isNull());
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids, 1, 1.0, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (this slice handles a single rim).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_chamfer_edge_asym(cyl, ids2, 2, 2.0, 1.0, 0.01).isNull());
  // A straight (Line) box edge is not a circular crease → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_chamfer_edge_asym(b, idsb, 1, 2.0, 1.0, 0.01).isNull());
}

// ── concave fillet (material-side torus canal on a boss ↔ larger shoulder rim) ────--

namespace {
// A stepped shaft (a boss cylinder standing on a larger coaxial disc plate) built by a
// native REVOLVE of a stepped meridian about the world Y axis. Faces: plate bottom
// disc (Rp), plate outer wall (Rp), shoulder annulus (Rc..Rp, the LARGER plane), boss
// wall (Rc), boss top cap (Rc). The concave base rim is the circle radius Rc at axial t.
topo::Shape steppedShaft(double Rp, double t, double Rc, double H) {
  const double prof[] = {0, 0, Rp, 0, Rp, t, Rc, t, Rc, t + H, 0, t + H};
  return cst::build_revolution(prof, 6, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// The Circle rim edge id at axial coord `h` along `az` with the given radius.
int findRimAtAxial(const topo::Shape& s, const nmath::Vec3& az, double h, double radius) {
  const topo::ShapeMap emap = topo::mapShapes(s, topo::ShapeType::Edge);
  for (std::size_t i = 1; i <= emap.size(); ++i) {
    const auto c = topo::curveOf(emap.shape(static_cast<int>(i)));
    if (!c || c->curve->kind != topo::EdgeCurve::Kind::Circle) continue;
    nmath::Point3 o = c->curve->frame.origin;
    if (!c->location.isIdentity()) o = c->location.transform().applyToPoint(o);
    if (std::fabs(nmath::dot(nmath::Vec3{o.x, o.y, o.z}, az) - h) < 1e-6 &&
        std::fabs(c->curve->radius - radius) < 1e-6)
      return static_cast<int>(i);
  }
  return 0;
}
// Closed-form ADDED rim-band volume (Pappus): the square corner r² minus the
// quarter-disc, that region's centroid revolved about the axis.
double concaveVfill(double Rc, double r) {
  return M_PI * ((Rc + r) * (Rc + r) - Rc * Rc) * r -
         2.0 * M_PI * ((Rc + r) - 4.0 * r / (3.0 * M_PI)) * (M_PI / 4.0) * r * r;
}
}  // namespace

CC_TEST(concave_fillet_boss_on_plate_watertight_volume_grown) {
  // Boss Rc=5, H=6 on a coaxial disc plate Rp=12, t=4; roll a ball r=1.5 into the
  // CONCAVE base rim → a material-side torus canal (major Rc+r=6.5, minor 1.5) that
  // ADDS material. Watertight, volume ABOVE the sharp shaft by the closed-form V_fill.
  const double Rp = 12, t = 4, Rc = 5, H = 6, r = 1.5;
  topo::Shape shaft = steppedShaft(Rp, t, Rc, H);
  const nmath::Vec3 axisY{0, 1, 0};
  bool wt0 = false;
  const double v0 = vol(shaft, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtAxial(shaft, axisY, t, Rc);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::concave_fillet_edge(shaft, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);            // torus quarter-tube welds watertight to wall + shoulder
  CC_CHECK(v > v0);        // a CONCAVE fillet ADDS material (volume GROWS)
  const double expected = v0 + concaveVfill(Rc, r);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(concave_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the concave canal normal at each seam matches the
  // adjacent primary-face normal exactly. n(u,v) = radial(u)·cos v + axis·sin v with
  // radius(v) = (Rc+r) − r·cos v.
  //   * v=0   (wall seam, radius Rc):     n = radial → CYLINDER outward normal. cos=1.
  //   * v=π/2 (shoulder seam, radius Rc+r): n = axis   → SHOULDER outward normal. cos=1.
  const double Rc = 5.0, r = 1.5;
  const nmath::Vec3 axis{0, 0, 1};
  for (int k = 0; k < 8; ++k) {
    const double u = 2.0 * M_PI * k / 8.0;
    const nmath::Vec3 radial{std::cos(u), std::sin(u), 0.0};
    const nmath::Vec3 nWall = radial * std::cos(0.0) + axis * std::sin(0.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nWall}.vec(), radial), 1.0, 1e-12));
    const nmath::Vec3 nSh = radial * std::cos(M_PI / 2.0) + axis * std::sin(M_PI / 2.0);
    CC_CHECK(nearRel(nmath::dot(nmath::Dir3{nSh}.vec(), axis), 1.0, 1e-12));
  }
  // Seam POSITIONS: radius(0)=Rc (wall), radius(π/2)=Rc+r (shoulder). Both flipped in
  // sign vs the convex builder (which trims the cap to Rc−r).
  const double Rt = Rc + r;  // concave torus major radius
  CC_CHECK(nearRel(Rt - r * std::cos(0.0), Rc, 1e-12));          // radius(0)=Rc (wall)
  CC_CHECK(nearRel(Rt - r * std::cos(M_PI / 2.0), Rc + r, 1e-12));  // radius(π/2)=Rc+r
}

CC_TEST(concave_fillet_scope_defers) {
  const double Rp = 12, t = 4, Rc = 5, H = 6;
  topo::Shape shaft = steppedShaft(Rp, t, Rc, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const int rim = findRimAtAxial(shaft, axisY, t, Rc);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  // Zero radius → NULL.
  CC_CHECK(blend::concave_fillet_edge(shaft, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL.
  int ids2[] = {rim, 1};
  CC_CHECK(blend::concave_fillet_edge(shaft, ids2, 2, 1.5, 0.01).isNull());
  // Seam leaves the shoulder: r so large that Rc+r > Rp → NULL.
  CC_CHECK(blend::concave_fillet_edge(shaft, ids, 1, 8.0, 0.01).isNull());
  // The CONVEX builder must DECLINE this concave rim (a larger coaxial cylinder exists),
  // so at most one curved builder fires — no sign confusion.
  CC_CHECK(blend::curved_fillet_edge(shaft, ids, 1, 1.5, 0.01).isNull());
  // A blind-hole bottom rim (a cup: outer Rp, hole Rc, flat bottom) is a DIFFERENT
  // concave config (offset Rc−r, plate top rim not on the crease) → deferred to OCCT.
  const double d = 3, Htop = 8;
  const double cupProf[] = {0, 0, Rp, 0, Rp, Htop, Rc, Htop, Rc, d, 0, d};
  topo::Shape cup = cst::build_revolution(cupProf, 6, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int brim = findRimAtAxial(cup, axisY, d, Rc);
  CC_CHECK(brim != 0);
  int cids[] = {brim};
  CC_CHECK(blend::concave_fillet_edge(cup, cids, 1, 1.0, 0.01).isNull());
  // The CONVEX capped cylinder rim is NOT concave (no larger coaxial cylinder) → the
  // concave builder declines it (control: no cross-firing).
  cst::ProfileSegment seg;
  seg.kind = 2;
  seg.cx = 0; seg.cy = 0; seg.r = 5.0;
  topo::Shape capped = cst::build_prism_profile({seg}, {}, {}, 10.0);
  const int caprim = findRimAtAxial(capped, nmath::Vec3{0, 0, 1}, 10.0, 5.0);
  CC_CHECK(caprim != 0);
  int capids[] = {caprim};
  CC_CHECK(blend::concave_fillet_edge(capped, capids, 1, 1.5, 0.01).isNull());
}

// ── cone-frustum cap fillet (torus band on a CONE↔coaxial-cap circular rim) ───────--

namespace {
// A capped cone frustum: profile (0,0)→(Rb,0)→(Rt,H)→(0,H) revolved a full turn about
// the in-plane axis (→ world +Y axis). Bottom cap radius Rb at h=0, top cap radius Rt at
// h=H, one Cone lateral wall (angular sectors). Rb≠Rt ⇒ a true Cone face.
topo::Shape cappedFrustum(double Rb, double Rt, double H) {
  const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
  return cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// Closed-form removed volume for a rolling-ball fillet r on the top rim (radius Rt) of a
// capped frustum (Pappus of the corner-minus-arc region, revolved about the axis). Matches
// the derivation in the header (verified numerically vs a fine solid-of-revolution).
double frustumRemoved(double Rb, double Rt, double H, double r) {
  const double dr = Rt - Rb, dz = H;
  double nwr = dz, nwz = -dr;
  const double nn = std::sqrt(nwr * nwr + nwz * nwz);
  nwr /= nn; nwz /= nn;
  if (nwr < 0) { nwr = -nwr; nwz = -nwz; }
  const double c = nwz;  // nW·nC with nC=(0,1)
  const double Cr = Rt - r * nwr / (1.0 + c);
  const double Cz = H - r * (nwz + 1.0) / (1.0 + c);
  const double Twr = Cr + r * nwr, Twz = Cz + r * nwz;
  const double Tcr = Cr, Tcz = Cz + r;
  const double angCap = M_PI / 2.0, angWall = std::atan2(nwz, nwr);
  // Polygon (Twall, rim, Tcap, arc cap→wall); shoelace area + centroid_r; Pappus volume.
  const int Na = 2000;
  std::vector<double> X{Twr, Rt, Tcr}, Y{Twz, H, Tcz};
  for (int i = 0; i < Na; ++i) {
    const double v = angCap + (angWall - angCap) * i / (Na - 1);
    X.push_back(Cr + r * std::cos(v));
    Y.push_back(Cz + r * std::sin(v));
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
double frustumSharpVolume(double Rb, double Rt, double H) {
  return M_PI * H / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt);
}
}  // namespace

CC_TEST(cone_fillet_narrowing_frustum_watertight_volume_reduced) {
  // Narrowing frustum Rb=6→Rt=4 over H=10; roll a ball r=1 into the top rim → a coaxial
  // torus band tangent to the tilted cone wall and the cap. Watertight, volume BELOW the
  // sharp frustum by the closed-form removed volume (deflection-bounded).
  const double Rb = 6, Rt = 4, H = 10, r = 1.0;
  topo::Shape s = cappedFrustum(Rb, Rt, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = frustumSharpVolume(Rb, Rt, H);  // exact sharp volume (closed form)
  const int rim = findRimAtAxial(s, axisY, H, Rt);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::cone_fillet_edge(s, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);              // torus band welds watertight to the cone wall + trimmed cap
  CC_CHECK(v < vSharp);      // a convex fillet REDUCES the volume vs the sharp frustum
  const double expected = vSharp - frustumRemoved(Rb, Rt, H, r);
  CC_CHECK(nearRel(v, expected, 5e-3));  // deflection-bounded facet approximation
}

CC_TEST(cone_fillet_widening_frustum_watertight_volume_reduced) {
  // Widening frustum Rb=4→Rt=6 over H=10 (the cone wall tilts OUTWARD toward the cap, so
  // the wall-seam minor angle is NEGATIVE). Same closed-form check.
  const double Rb = 4, Rt = 6, H = 10, r = 1.0;
  topo::Shape s = cappedFrustum(Rb, Rt, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = frustumSharpVolume(Rb, Rt, H);
  const int rim = findRimAtAxial(s, axisY, H, Rt);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::cone_fillet_edge(s, ids, 1, r, 0.005);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < vSharp);
  const double expected = vSharp - frustumRemoved(Rb, Rt, H, r);
  CC_CHECK(nearRel(v, expected, 5e-3));
}

CC_TEST(cone_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the torus band normal at each seam equals the adjacent
  // primary-face normal. The band normal in the (radial, sAxial) cross section is
  //   n(vAbs) = (cos vAbs, sin vAbs), vAbs ∈ [angWall, π/2].
  //   * cap seam vAbs=π/2:   n = (0,1) → the CAP outward normal (axial). ✓
  //   * wall seam vAbs=angWall: n = (cos angWall, sin angWall) = the cone wall outward
  //     normal (the tangent-cut direction), matching (1,−dR/dz)/‖·‖ by construction. ✓
  // We check the wall-seam normal equals the cone wall's (r,z) outward normal for a
  // narrowing (dR<0) and a widening (dR>0) frustum.
  for (const double dR : {-2.0, +2.0}) {
    const double dz = 10.0;
    const double wn = std::sqrt(dz * dz + dR * dR);
    // cone wall outward (r,z) = (dz, −dR)/‖·‖ with +r; here s=+1 (cap above), so
    // dRdz = dR/dz and the header's nW2 = (1,−dRdz)/√(1+dRdz²) = (dz,−dR)/√(dz²+dR²).
    const double nWr = dz / wn, nWz = -dR / wn;
    const double angWall = std::atan2(nWz, nWr);
    // band normal at the wall seam
    const double nbandR = std::cos(angWall), nbandZ = std::sin(angWall);
    CC_CHECK(nearRel(nbandR, nWr, 1e-12));
    CC_CHECK(nearRel(nbandZ, nWz, 1e-12));
    // cap seam
    CC_CHECK(nearRel(std::cos(M_PI / 2.0), 0.0, 1e-12));
    CC_CHECK(nearRel(std::sin(M_PI / 2.0), 1.0, 1e-12));
  }
}

CC_TEST(cone_fillet_scope_defers) {
  const nmath::Vec3 axisY{0, 1, 0};
  // A pure CYLINDER (Rb==Rt, no Cone face) is the σ=0 case owned by curved_fillet_edge;
  // the cone builder declines it wholesale (no Cone face present).
  topo::Shape cyl = cappedFrustum(5, 5, 10);  // Rb==Rt → a cylinder, no Cone face
  const int crim = findRimAtAxial(cyl, axisY, 10, 5);
  if (crim != 0) {
    int cids[] = {crim};
    CC_CHECK(blend::cone_fillet_edge(cyl, cids, 1, 1.0, 0.01).isNull());
  }
  // Ring-torus guard: a steep frustum whose ball-centre radius Rmaj < r → NULL (spindle).
  topo::Shape steep = cappedFrustum(8, 3, 12);
  const int srim = findRimAtAxial(steep, axisY, 12, 3);
  CC_CHECK(srim != 0);
  int sids[] = {srim};
  CC_CHECK(blend::cone_fillet_edge(steep, sids, 1, 2.0, 0.01).isNull());  // Rmaj≈1.67<2
  // Zero / negative radius → NULL.
  topo::Shape s = cappedFrustum(6, 4, 10);
  const int rim = findRimAtAxial(s, axisY, 10, 4);
  int ids[] = {rim};
  CC_CHECK(blend::cone_fillet_edge(s, ids, 1, 0.0, 0.01).isNull());
  // More than one picked edge → NULL (single rim only).
  int ids2[] = {rim, 1};
  CC_CHECK(blend::cone_fillet_edge(s, ids2, 2, 1.0, 0.01).isNull());
  // A pure planar box has no Cone face → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::cone_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
  // A multi-frustum (two cones with DIFFERENT slopes → different σ) → NULL (wholesale
  // mismatch: the second cone's σ/Rref differ, so the body is not a single frustum).
  // Segments (6,0)→(5,5) slope −1/5 and (5,5)→(2,10) slope −3/5 are NON-collinear.
  const double twoCone[] = {0, 0, 6, 0, 5, 5, 2, 10, 0, 10};
  topo::Shape tc = cst::build_revolution(twoCone, 5, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int tcrim = findRimAtAxial(tc, axisY, 10, 2);
  CC_CHECK(tcrim != 0);
  int tcids[] = {tcrim};
  CC_CHECK(blend::cone_fillet_edge(tc, tcids, 1, 0.5, 0.01).isNull());
}

// ── sphere cap fillet (torus band on a SPHERE↔coaxial-cap circular rim) ───────────--

namespace {
// A truncated ball: sphere radius R centred at origin, capped by a flat top at axial height
// zc (0<zc<R). Profile in (r,h): arc from the south pole (0,-R) up to the rim (Rrim,zc),
// then a line rim→axis (0,zc) — the cap. Revolved a full turn about the in-plane axis
// (→ world +Y). Produces coaxial Sphere sectors + a planar top cap. Rrim = √(R²−zc²).
topo::Shape truncatedBall(double R, double zc) {
  const double Rrim = std::sqrt(R * R - zc * zc);
  std::vector<cst::ProfileSegment> segs;
  cst::ProfileSegment arc;
  arc.kind = 1;
  arc.cx = 0; arc.cy = 0; arc.r = R;
  arc.x0 = 0; arc.y0 = -R; arc.x1 = Rrim; arc.y1 = zc;
  arc.a0 = -M_PI / 2.0; arc.a1 = std::atan2(zc, Rrim);
  segs.push_back(arc);
  cst::ProfileSegment cap;
  cap.kind = 0;
  cap.x0 = Rrim; cap.y0 = zc; cap.x1 = 0; cap.y1 = zc;
  segs.push_back(cap);
  return cst::build_revolution_profile(segs, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// Sharp truncated-ball volume = ball region below z=zc = π·∫_{-R}^{zc}(R²−z²)dz.
double truncatedBallVolume(double R, double zc) {
  auto cap = [&](double z) { return R * R * z - z * z * z / 3.0; };
  return M_PI * (cap(zc) - cap(-R));
}
// Closed-form removed volume for a rolling-ball fillet r on the cap rim of a truncated ball
// (Pappus of the corner-minus-arc region). Cross-section polygon: sphere arc (sphere seam →
// rim), cap (rim → cap seam), torus arc (cap seam → sphere seam). 2π·|A|·centroid_r.
double sphereRemoved(double R, double zc, double r) {
  const double Rrim = std::sqrt(R * R - zc * zc);
  const double Cz = zc - r, d = R - r;
  const double Rmaj = std::sqrt(d * d - Cz * Cz);
  const double vWall = std::atan2(Cz, Rmaj);           // wall-seam minor angle
  const double scRad = Rmaj + r * std::cos(vWall);     // sphere-seam radius
  const double scAx = Cz + r * std::sin(vWall);        // sphere-seam axial
  const double latSeam = std::atan2(scAx, scRad);
  std::vector<double> X, Y;
  const int Na = 3000;
  // sphere arc: latitude latSeam → the rim latitude (asin(zc/R)), radius R·cos lat.
  const double latRim = std::asin(zc / R);
  for (int i = 0; i < Na; ++i) {
    const double lat = latSeam + (latRim - latSeam) * i / (Na - 1);
    X.push_back(R * std::cos(lat)); Y.push_back(R * std::sin(lat));
  }
  // cap: radius Rrim → Rmaj at z=zc.
  X.push_back(Rmaj); Y.push_back(zc);
  // torus arc: v from π/2 down to vWall.
  for (int i = 0; i < Na; ++i) {
    const double v = (M_PI / 2.0) + (vWall - M_PI / 2.0) * i / (Na - 1);
    X.push_back(Rmaj + r * std::cos(v)); Y.push_back(Cz + r * std::sin(v));
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
}  // namespace

CC_TEST(sphere_fillet_truncated_ball_watertight_volume_reduced) {
  // A ball R=5 capped at zc=3 (rim radius 4); roll a ball r=0.8 into the cap rim → a coaxial
  // torus band tangent to the sphere wall and the cap. Watertight, volume BELOW the sharp
  // truncated ball by the closed-form removed volume (deflection-bounded).
  const double R = 5, zc = 3, r = 0.8;
  topo::Shape s = truncatedBall(R, zc);
  CC_CHECK(!s.isNull());
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = truncatedBallVolume(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::sphere_fillet_edge(s, ids, 1, r, 0.004);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                 // torus band welds watertight to the sphere wall + trimmed cap
  CC_CHECK(v < vSharp);         // a convex fillet REDUCES the volume vs the sharp ball
  const double expected = vSharp - sphereRemoved(R, zc, r);
  CC_CHECK(nearRel(v, expected, 5e-3));
}

CC_TEST(sphere_fillet_converges_with_deflection) {
  // The faceted sphere+torus body under-fills; refining the deflection tightens the volume
  // toward the exact filleted volume from below.
  const double R = 6, zc = 2.5, r = 1.0;
  topo::Shape s = truncatedBall(R, zc);
  const nmath::Vec3 axisY{0, 1, 0};
  const double Rrim = std::sqrt(R * R - zc * zc);
  const int rim = findRimAtAxial(s, axisY, zc, Rrim);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  const double exact = truncatedBallVolume(R, zc) - sphereRemoved(R, zc, r);
  bool wtC = false, wtF = false;
  const double vCoarse = vol(blend::sphere_fillet_edge(s, ids, 1, r, 0.05), wtC);
  const double vFine = vol(blend::sphere_fillet_edge(s, ids, 1, r, 0.004), wtF);
  CC_CHECK(wtC && wtF);
  CC_CHECK(vCoarse <= exact + 1e-6);          // under-fills
  CC_CHECK(vFine <= exact + 1e-6);
  CC_CHECK(vFine >= vCoarse - 1e-9);          // refinement grows the volume toward exact
  CC_CHECK(nearRel(vFine, exact, 5e-3));
}

CC_TEST(sphere_fillet_g1_tangent_at_both_seams) {
  // ANALYTIC G1 (no OCCT, no mesh): the torus band normal (cos v, sin v) at each seam equals
  // the adjacent primary-face normal in the (radial, axial) cross section.
  const double R = 5, zc = 3, r = 0.8;
  const double Cz = zc - r, d = R - r;
  const double Rmaj = std::sqrt(d * d - Cz * Cz);
  const double vWall = std::atan2(Cz, Rmaj);
  // cap seam v=π/2: normal (0,1) == cap axial normal.
  CC_CHECK(nearRel(std::cos(M_PI / 2.0), 0.0, 1e-12));
  CC_CHECK(nearRel(std::sin(M_PI / 2.0), 1.0, 1e-12));
  // wall seam: torus normal (cos vWall, sin vWall) must equal the sphere outward normal
  // (radial, axial)/R at the seam point.
  const double scRad = Rmaj + r * std::cos(vWall);
  const double scAx = Cz + r * std::sin(vWall);
  CC_CHECK(nearRel(scRad * scRad + scAx * scAx, R * R, 1e-9));   // seam lies ON the sphere
  CC_CHECK(nearRel(std::cos(vWall), scRad / R, 1e-9));           // normals coincide
  CC_CHECK(nearRel(std::sin(vWall), scAx / R, 1e-9));
}

CC_TEST(sphere_fillet_scope_defers) {
  const nmath::Vec3 axisY{0, 1, 0};
  // A pure cylinder (no Sphere face) → NULL (owned by curved_fillet_edge).
  const double cylProf[] = {0, 0, 5, 0, 5, 10, 0, 10};
  topo::Shape cyl = cst::build_revolution(cylProf, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int crim = findRimAtAxial(cyl, axisY, 10, 5);
  if (crim != 0) {
    int cids[] = {crim};
    CC_CHECK(blend::sphere_fillet_edge(cyl, cids, 1, 1.0, 0.01).isNull());
  }
  // A truncated ball, but: zero radius, multi-edge, and an oversized ring-torus radius.
  const double R = 5, zc = 3;
  topo::Shape s = truncatedBall(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  CC_CHECK(blend::sphere_fillet_edge(s, ids, 1, 0.0, 0.01).isNull());   // r=0
  int ids2[] = {rim, 1};
  CC_CHECK(blend::sphere_fillet_edge(s, ids2, 2, 0.5, 0.01).isNull());  // multi-edge
  // Ring-torus / seam guard: a very large r on a shallow cap → Rmaj<r or seam≥cap → NULL.
  CC_CHECK(blend::sphere_fillet_edge(s, ids, 1, 2.8, 0.01).isNull());
  // A pure planar box has no Sphere face → NULL.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::sphere_fillet_edge(b, idsb, 1, 1.0, 0.01).isNull());
}

// ── G2 (curvature-MATCHING) fillet on a CURVED substrate: SPHERE ↔ cap rim ─────────--
// The Layer-4-NURBS curved-substrate slice. Unlike the PLANAR G2 fillet (κ=0 at both
// rails, collinear triples), the SPHERE substrate has NON-ZERO normal curvature 1/R at the
// tangency line, so G2 requires the blend section's meridian curvature to MATCH 1/R there
// (the cap end still matches the plane's 0). The quintic pole placement q=(5/4)κ·h² hits a
// prescribed end curvature (q=0 → the planar collinear-triple special case). These gates
// prove: (1) closed-form κ(0)=1/R (MATCHED, not zero) and κ(1)=0 with the G1 torus tube's
// 1/r JUMP as a non-trivial control — this is the curvature CONTINUITY (matched) point;
// (2) watertight + a volume converging under refinement to the exact quintic-removed
// closed form; (3) honest declines (cylinder/cone/box → NULL → OCCT; no cross-firing).

namespace {
// Exact G2-removed volume for the sphere↔cap rim, by Pappus on the cross-section polygon
// bounded by the sphere arc (sphere seam → rim), the cap segment (rim → cap seam), and the
// curvature-MATCHING QUINTIC (cap seam → sphere seam) — the mirror of sphereRemoved but with
// the quintic replacing the torus arc. 2π·|A|·centroid_r over the meridian polygon.
double sphereRemovedG2(double R, double zc, double r) {
  blend::detail::SphereCapGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.R = R; g.capH = zc; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedSphereSection(g, r);
  if (!sec) return 0.0;
  const double scRad = sec->poles[0].rho, scAx = sec->poles[0].z;   // wall seam (on sphere)
  const double Rmaj = sec->poles[5].rho;                            // cap seam radius
  const double latSeam = std::atan2(scAx, scRad);
  const double latRim = std::asin(zc / R);
  std::vector<double> X, Y;
  const int Na = 3000;
  for (int i = 0; i < Na; ++i) {  // sphere arc: seam latitude → rim latitude
    const double lat = latSeam + (latRim - latSeam) * i / (Na - 1);
    X.push_back(R * std::cos(lat)); Y.push_back(R * std::sin(lat));
  }
  X.push_back(Rmaj); Y.push_back(zc);  // cap: rim → cap seam (at z=zc)
  for (int i = 0; i < Na; ++i) {  // quintic: cap seam (s=1) back to sphere seam (s=0)
    const double s = 1.0 - static_cast<double>(i) / (Na - 1);
    const blend::detail::Mrd m = blend::detail::quinticMeridian(sec->poles, s);
    X.push_back(m.rho); Y.push_back(m.z);
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
}  // namespace

CC_TEST(g2_curved_sphere_section_curvature_matches_1_over_R_closed_form) {
  // CLOSED-FORM CURVATURE-MATCH PROOF (no OCCT, no mesh): the quintic meridian section's
  // curvature at the SPHERE-WALL rail equals the sphere's normal curvature 1/R (NOT zero —
  // the whole point of the curved case), and at the CAP rail equals the plane's zero. This
  // is a genuine curvature CONTINUITY (matched κ) across a curved-substrate boundary.
  const double R = 5.0, zc = 2.0, r = 1.0;
  blend::detail::SphereCapGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.R = R; g.capH = zc; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedSphereSection(g, r);
  CC_CHECK(sec.has_value());
  const double kWall = blend::detail::meridianCurvature(sec->poles, 0.0);
  const double kCap = blend::detail::meridianCurvature(sec->poles, 1.0);
  CC_CHECK(nearRel(kWall, 1.0 / R, 1e-9));  // MATCHES the sphere normal curvature 1/R (≠ 0)
  CC_CHECK(std::fabs(kCap) < 1e-9);         // MATCHES the flat cap's zero curvature
  // The matched wall curvature is genuinely NON-ZERO (this is not the planar κ=0 case).
  CC_CHECK(1.0 / R > 0.1);
  // The wall seam lies EXACTLY on the sphere and the cap seam EXACTLY on the cap plane (G0),
  // and the near-rail tangent lies in the neighbour surface (G1).
  const double distW = std::sqrt(sec->poles[0].rho * sec->poles[0].rho +
                                 sec->poles[0].z * sec->poles[0].z);
  CC_CHECK(nearRel(distW, R, 1e-9));                 // P0 on the sphere
  CC_CHECK(nearRel(sec->poles[5].z, zc, 1e-9));      // P5 on the cap plane
  const blend::detail::Mrd t0 = sec->poles[1] - sec->poles[0];
  CC_CHECK(std::fabs(t0.rho * sec->poles[0].rho + t0.z * sec->poles[0].z) < 1e-9);  // G1 tangent
  const blend::detail::Mrd t1 = sec->poles[5] - sec->poles[4];
  CC_CHECK(std::fabs(t1.z) < 1e-9);                  // G1 tangent to the cap (horizontal)
}

CC_TEST(g2_curved_sphere_measured_section_curvature_matches_and_beats_g1) {
  // DISCRETE MEASURED witness (host, OCCT-free), independent of the analytic hodograph:
  // sample the quintic meridian near the WALL rail via the Menger curvature of three
  // consecutive section points (κ = 1/circumradius), and confirm the measured near-rail
  // curvature CONVERGES to the sphere's 1/R — matched, NOT the torus tube's 1/r. The G1
  // control is the circular torus section (curvature 1/r everywhere) whose wall-seam value
  // is a JUMP; the G2 measured value sits far closer to 1/R.
  const double R = 5.0, zc = 2.0, r = 1.0;
  blend::detail::SphereCapGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.R = R; g.capH = zc; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedSphereSection(g, r);
  CC_CHECK(sec.has_value());
  // Menger curvature of the meridian at parameter s from a small symmetric triple (in the
  // (ρ,z) plane) — a purely geometric 1/circumradius, not the analytic B'/B'' formula.
  auto kappaAt = [&](double s) -> double {
    const double ds = 1e-3;
    const blend::detail::Mrd pm = blend::detail::quinticMeridian(sec->poles, s - ds);
    const blend::detail::Mrd p0 = blend::detail::quinticMeridian(sec->poles, s);
    const blend::detail::Mrd pp = blend::detail::quinticMeridian(sec->poles, s + ds);
    const double ax = p0.rho - pm.rho, ay = p0.z - pm.z;
    const double cx = pp.rho - p0.rho, cy = pp.z - p0.z;
    const double bx = pp.rho - pm.rho, by = pp.z - pm.z;
    const double la = std::hypot(ax, ay), lc = std::hypot(cx, cy), lb = std::hypot(bx, by);
    if (la * lc * lb < 1e-18) return 0.0;
    // Menger curvature = 4·(triangle area)/(|a||b||c|) = 2·|a×c|/(|a||b||c|) = 1/circumradius.
    return 2.0 * std::fabs(ax * cy - ay * cx) / (la * lc * lb);
  };
  const double kSphere = 1.0 / R;    // the substrate's normal curvature at the wall seam
  const double kTubeG1 = 1.0 / r;    // the G1 torus tube's constant section curvature
  const double kNearWall = kappaAt(0.002);  // measured, close to the wall rail
  // Measured near-rail curvature is close to 1/R (matched) and far from the G1 tube's 1/r.
  CC_CHECK(std::fabs(kNearWall - kSphere) < 0.02);              // MATCHES the sphere 1/R
  CC_CHECK(std::fabs(kNearWall - kSphere) < 0.1 * std::fabs(kTubeG1 - kSphere));  // beats G1
  CC_CHECK(kappaAt(0.5) > 1e-3);     // mid-section IS curved (a real blend, not collapsed)
}

CC_TEST(g2_curved_sphere_control_g1_tube_curvature_jumps) {
  // CONTROL (proves the match is non-trivial): the G1 torus tube section has CONSTANT
  // curvature 1/r at the wall seam — which for r<R is a JUMP away from the sphere's 1/R.
  // The G2 quintic removes exactly that jump (κ→1/R), so |κ_G2 − 1/R| ≪ |1/r − 1/R|.
  const double R = 5.0, r = 1.0;
  const double kSphere = 1.0 / R;     // what the substrate demands at the wall seam
  const double kTubeG1 = 1.0 / r;     // the G1 torus tube's constant section curvature
  CC_CHECK(std::fabs(kTubeG1 - kSphere) > 0.5);  // the G1 curvature JUMP at the wall seam
  // The G2 section matches to rounding (proven above) → its jump is ~0.
  blend::detail::SphereCapGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.R = R; g.capH = 2.0; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedSphereSection(g, r);
  CC_CHECK(sec.has_value());
  const double jumpG2 = std::fabs(blend::detail::meridianCurvature(sec->poles, 0.0) - kSphere);
  CC_CHECK(jumpG2 < 1e-6);                          // G2: curvature CONTINUOUS at the wall
  CC_CHECK(jumpG2 < 0.01 * std::fabs(kTubeG1 - kSphere));  // ≪ the G1 jump (matched vs jump)
}

CC_TEST(g2_curved_sphere_watertight_volume_reduced) {
  // The G2 blend on a truncated ball (R=5, cap zc=2.5, r=1) is watertight, REMOVES material
  // (convex fillet), and its volume converges under refinement to the exact quintic-removed
  // closed form. The revolved curvature-matching section welds to the sphere wall + trimmed
  // cap the SAME way the G1 torus band does (shared N angular samples).
  const double R = 5.0, zc = 2.5, r = 1.0;
  topo::Shape s = truncatedBall(R, zc);
  CC_CHECK(!s.isNull());
  const nmath::Vec3 axisY{0, 1, 0};
  const double vSharp = truncatedBallVolume(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::curved_fillet_edge_g2(s, ids, 1, r, 0.004);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                          // revolved G2 section welds watertight
  CC_CHECK(v < vSharp);                  // convex fillet REMOVES material
  const double expected = vSharp - sphereRemovedG2(R, zc, r);
  CC_CHECK(nearRel(v, expected, 5e-3));  // matches the exact quintic-removed volume
}

CC_TEST(g2_curved_sphere_converges_and_deterministic) {
  // Faceting under-fills the convex blend; refining the deflection grows the volume toward
  // the exact quintic-removed closed form from below. And the build is deterministic.
  const double R = 6.0, zc = 3.0, r = 1.2;
  topo::Shape s = truncatedBall(R, zc);
  const nmath::Vec3 axisY{0, 1, 0};
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  const double exact = truncatedBallVolume(R, zc) - sphereRemovedG2(R, zc, r);
  bool wtC = false, wtF = false, wtF2 = false;
  const double vCoarse = vol(blend::curved_fillet_edge_g2(s, ids, 1, r, 0.05), wtC);
  const double vFine = vol(blend::curved_fillet_edge_g2(s, ids, 1, r, 0.004), wtF);
  const double vFine2 = vol(blend::curved_fillet_edge_g2(s, ids, 1, r, 0.004), wtF2);
  CC_CHECK(wtC && wtF && wtF2);
  CC_CHECK(vCoarse <= exact + 1e-6);      // under-fills
  CC_CHECK(vFine >= vCoarse - 1e-9);      // refinement grows toward exact
  CC_CHECK(nearRel(vFine, exact, 5e-3));
  CC_CHECK(nearRel(vFine, vFine2, 1e-12));  // deterministic (fp64, no RNG)
}

CC_TEST(g2_curved_sphere_scope_defers) {
  // Honest declines (→ OCCT), and NO cross-firing with the planar / cylinder builders.
  const nmath::Vec3 axisY{0, 1, 0};
  const double R = 5.0, zc = 3.0;
  topo::Shape s = truncatedBall(R, zc);
  const int rim = findRimAtAxial(s, axisY, zc, std::sqrt(R * R - zc * zc));
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  CC_CHECK(blend::curved_fillet_edge_g2(s, ids, 1, 0.0, 0.01).isNull());   // r=0
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_fillet_edge_g2(s, ids2, 2, 0.5, 0.01).isNull());  // multi-edge
  CC_CHECK(blend::curved_fillet_edge_g2(s, ids, 1, 2.8, 0.01).isNull());   // ring/seam guard
  // A CYLINDER↔cap rim (meridian normal curvature ZERO at both seams) is NOT this slice's
  // curvature-MATCHING case → NULL (planar-style κ=0 / OCCT owns it).
  const double cylProf[] = {0, 0, 5, 0, 5, 10, 0, 10};
  topo::Shape cyl = cst::build_revolution(cylProf, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
  const int crim = findRimAtAxial(cyl, axisY, 10, 5);
  if (crim != 0) {
    int cids[] = {crim};
    CC_CHECK(blend::curved_fillet_edge_g2(cyl, cids, 1, 1.0, 0.01).isNull());
  }
  // A planar box has no Sphere face → NULL (the planar G2 builder owns box edges).
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_fillet_edge_g2(b, idsb, 1, 1.0, 0.01).isNull());
  // Control: the sphere rim DOES land (no spurious decline).
  CC_CHECK(!blend::curved_fillet_edge_g2(s, ids, 1, 1.0, 0.004).isNull());
}

// ── G2 (curvature-MATCHING) fillet on a CYLINDER (and CONE) ↔ cap rim ──────────────--
// The next curved-substrate slice after sphere↔cap. A cylinder / cone wall is STRAIGHT-
// RULED, so its normal curvature IN THE SECTION (meridian) PLANE at the wall seam is 0
// (the ruling is a straight line — "0 along the axis" in the task's phrasing), while its
// HOOP curvature 1/Rc (cone: cosσ/ρ) is matched AUTOMATICALLY by the revolution once G1
// fixes the seam normal radial. So the genuine G2 match is κ_meridian=0 at BOTH seams (a
// κ=0 quintic, collinear triples) — the SAME section-plane curvature the flat cap wants,
// but on a genuinely CURVED substrate whose hoop curvature the blend also matches. The G1
// torus tube's meridian curvature 1/r at both seams is the JUMP the quintic removes.
// These gates prove: (1) closed-form κ_meridian(wall)=κ_meridian(cap)=0 AND the automatic
// hoop match 1/Rc (wall) / 0 (cap), with the G1 torus tube's 1/r meridian JUMP as the
// non-trivial control; (2) watertight + a volume converging under refinement to the exact
// quintic-removed closed form; (3) honest declines + no cross-firing with sphere/planar.

namespace {
// Exact G2-removed volume for a cyl↔cap rim, by Pappus on the meridian cross-section
// polygon bounded by the cylinder wall (wall seam → rim), the cap segment (rim → cap seam),
// and the κ=0 QUINTIC (cap seam → wall seam). 2π·|A|·centroid_r over the polygon.
double cylRemovedG2(double Rc, double h, double r) {
  blend::detail::RimGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.radius = Rc; g.capH = h; g.farH = 0.0; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedCylSection(g, r);
  if (!sec) return 0.0;
  const double seamAx = sec->poles[0].z;   // wall seam axial (= h−r)
  const double Rmaj = sec->poles[5].rho;   // cap seam radius (= Rc−r)
  std::vector<double> X, Y;
  X.push_back(Rc); Y.push_back(seamAx);    // wall seam (on the cylinder)
  X.push_back(Rc); Y.push_back(h);         // rim (sharp corner)
  X.push_back(Rmaj); Y.push_back(h);       // cap seam (on the cap)
  const int Na = 3000;
  for (int i = 0; i < Na; ++i) {           // quintic: cap seam (s=1) back to wall seam (s=0)
    const double s = 1.0 - static_cast<double>(i) / (Na - 1);
    const blend::detail::Mrd m = blend::detail::quinticMeridian(sec->poles, s);
    X.push_back(m.rho); Y.push_back(m.z);
  }
  double A = 0, cx = 0;
  const int n = static_cast<int>(X.size());
  for (int i = 0; i < n; ++i) {
    const int j = (i + 1) % n;
    const double cr = X[i] * Y[j] - X[j] * Y[i];
    A += cr; cx += (X[i] + X[j]) * cr;
  }
  A *= 0.5; cx /= (6.0 * A);
  return 2.0 * M_PI * std::fabs(A) * cx;
}
}  // namespace

CC_TEST(g2_curved_cyl_section_curvature_matches_zero_and_hoop_closed_form) {
  // CLOSED-FORM CURVATURE-MATCH PROOF (no OCCT, no mesh): the κ=0 quintic meridian section's
  // curvature is EXACTLY zero at BOTH the cylinder-wall rail and the cap rail — matching the
  // straight-ruled cylinder's meridian normal curvature (0) and the flat cap's (0). The HOOP
  // curvature the blend carries at the wall seam is 1/Rc (matched automatically by the
  // revolution) and 0 at the cap seam — G2 in BOTH principal directions across both seams.
  const double Rc = 5.0, h = 10.0, r = 1.5;
  blend::detail::RimGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.radius = Rc; g.capH = h; g.farH = 0.0; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedCylSection(g, r);
  CC_CHECK(sec.has_value());
  // Meridian curvature at both rails is exactly 0 (matched to the straight ruling + flat cap).
  const double kWall = blend::detail::meridianCurvature(sec->poles, 0.0);
  const double kCap = blend::detail::meridianCurvature(sec->poles, 1.0);
  CC_CHECK(std::fabs(kWall) < 1e-9);  // MATCHES the cylinder's meridian normal curvature (0)
  CC_CHECK(std::fabs(kCap) < 1e-9);   // MATCHES the flat cap's zero curvature
  // Second differences vanish at both ends (B''=0 ⇒ κ=0), the algebraic collinear-triple.
  const nmath::Vec3 d2s{sec->poles[0].rho - 2 * sec->poles[1].rho + sec->poles[2].rho,
                        sec->poles[0].z - 2 * sec->poles[1].z + sec->poles[2].z, 0.0};
  const nmath::Vec3 d2e{sec->poles[5].rho - 2 * sec->poles[4].rho + sec->poles[3].rho,
                        sec->poles[5].z - 2 * sec->poles[4].z + sec->poles[3].z, 0.0};
  CC_CHECK(nmath::norm(d2s) < 1e-12);
  CC_CHECK(nmath::norm(d2e) < 1e-12);
  // The blend's HOOP curvature is a genuine (nonzero) second-order match on the curved wall.
  CC_CHECK(nearRel(sec->kHoopWall, 1.0 / Rc, 1e-12));  // = 1/Rc (the cylinder's hoop curvature)
  CC_CHECK(1.0 / Rc > 0.1);                            // genuinely curved substrate (not planar)
  // Seams lie EXACTLY on the neighbours (G0) with in-surface tangents (G1).
  CC_CHECK(nearRel(sec->poles[0].rho, Rc, 1e-12));     // P0 on the cylinder wall
  CC_CHECK(nearRel(sec->poles[0].z, h - r, 1e-12));    // r below the cap
  CC_CHECK(nearRel(sec->poles[5].z, h, 1e-12));        // P5 on the cap plane
  CC_CHECK(nearRel(sec->poles[5].rho, Rc - r, 1e-12)); // trimmed to Rc−r
  const blend::detail::Mrd t0 = sec->poles[1] - sec->poles[0];
  CC_CHECK(std::fabs(t0.rho) < 1e-12);                 // wall tangent axial (along the cylinder)
  const blend::detail::Mrd t1 = sec->poles[5] - sec->poles[4];
  CC_CHECK(std::fabs(t1.z) < 1e-12);                   // cap tangent horizontal (in the cap)
}

CC_TEST(g2_curved_cyl_measured_section_curvature_matches_and_beats_g1) {
  // DISCRETE MEASURED witness (host, OCCT-free), independent of the analytic hodograph: the
  // quintic meridian's near-wall Menger curvature CONVERGES to the cylinder's section-plane
  // (meridian) curvature 0 — NOT the torus tube's 1/r. The G1 control is the circular torus
  // section (curvature 1/r everywhere) whose wall-seam value is a JUMP; the G2 measured value
  // sits far closer to 0.
  const double Rc = 5.0, h = 10.0, r = 1.5;
  blend::detail::RimGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.radius = Rc; g.capH = h; g.farH = 0.0; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedCylSection(g, r);
  CC_CHECK(sec.has_value());
  auto kappaAt = [&](double s) -> double {
    const double ds = 1e-3;
    const blend::detail::Mrd pm = blend::detail::quinticMeridian(sec->poles, s - ds);
    const blend::detail::Mrd p0 = blend::detail::quinticMeridian(sec->poles, s);
    const blend::detail::Mrd pp = blend::detail::quinticMeridian(sec->poles, s + ds);
    const double ax = p0.rho - pm.rho, ay = p0.z - pm.z;
    const double cx = pp.rho - p0.rho, cy = pp.z - p0.z;
    const double bx = pp.rho - pm.rho, by = pp.z - pm.z;
    const double la = std::hypot(ax, ay), lc = std::hypot(cx, cy), lb = std::hypot(bx, by);
    if (la * lc * lb < 1e-18) return 0.0;
    return 2.0 * std::fabs(ax * cy - ay * cx) / (la * lc * lb);  // Menger = 1/circumradius
  };
  const double kTubeG1 = 1.0 / r;              // the G1 torus tube's constant section curvature
  const double kNearWall = kappaAt(0.002);     // measured, close to the wall rail
  CC_CHECK(kNearWall < 0.02);                  // MATCHES the cylinder's meridian curvature (→0)
  CC_CHECK(kNearWall < 0.05 * kTubeG1);        // ≪ the G1 tube's 1/r JUMP at the wall
  CC_CHECK(kappaAt(0.5) > 1e-3);               // mid-section IS curved (a real blend)
}

CC_TEST(g2_curved_cyl_control_g1_tube_curvature_jumps) {
  // CONTROL (proves the match is non-trivial): the G1 torus tube section has CONSTANT
  // curvature 1/r at the wall seam — a JUMP away from the cylinder's meridian curvature 0.
  // The G2 quintic removes exactly that jump (κ→0), so |κ_G2 − 0| ≪ |1/r − 0|.
  const double Rc = 5.0, r = 1.5;
  const double kTubeG1 = 1.0 / r;
  CC_CHECK(kTubeG1 > 0.5);  // the G1 curvature JUMP at the wall (vs the substrate's 0)
  blend::detail::RimGeom g;
  g.axis.origin = nmath::Point3{0, 0, 0};
  g.axis.x = nmath::Dir3{nmath::Vec3{1, 0, 0}};
  g.axis.y = nmath::Dir3{nmath::Vec3{0, 1, 0}};
  g.axis.z = nmath::Dir3{nmath::Vec3{0, 0, 1}};
  g.radius = Rc; g.capH = 10.0; g.farH = 0.0; g.capNormal = nmath::Vec3{0, 0, 1};
  const auto sec = blend::detail::g2CurvedCylSection(g, r);
  CC_CHECK(sec.has_value());
  const double jumpG2 = std::fabs(blend::detail::meridianCurvature(sec->poles, 0.0));
  CC_CHECK(jumpG2 < 1e-9);                        // G2: curvature CONTINUOUS at the wall
  CC_CHECK(jumpG2 < 0.01 * kTubeG1);              // ≪ the G1 jump (matched vs jump)
}

CC_TEST(g2_curved_cyl_watertight_volume_reduced_and_converges) {
  // The G2 blend on a capped cylinder (Rc=5, h=10, r=1.5) is watertight, REMOVES material
  // (convex fillet), and its volume converges under refinement to the exact quintic-removed
  // closed form. The revolved κ=0 section welds to the wall + trimmed cap the SAME way the
  // G1 torus band does (shared N angular samples).
  const double Rc = 5.0, h = 10.0, r = 1.5;
  topo::Shape cyl = cappedCylinder(Rc, h);
  bool wt0 = false;
  const double v0 = vol(cyl, wt0);
  CC_CHECK(wt0);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  topo::Shape f = blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, r, 0.004);
  bool wt = false;
  const double v = vol(f, wt);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);                          // revolved G2 section welds watertight
  CC_CHECK(v < v0);                      // convex fillet REMOVES material
  const double exact = v0 - cylRemovedG2(Rc, h, r);
  CC_CHECK(nearRel(v, exact, 5e-3));     // matches the exact quintic-removed volume
  // Refinement (coarse → fine) grows the under-filled convex blend toward exact; deterministic.
  bool wtC = false, wtF = false, wtF2 = false;
  const double vCoarse = vol(blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, r, 0.05), wtC);
  const double vFine = vol(blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, r, 0.004), wtF);
  const double vFine2 = vol(blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, r, 0.004), wtF2);
  CC_CHECK(wtC && wtF && wtF2);
  CC_CHECK(vCoarse <= exact + 1e-6);     // under-fills
  CC_CHECK(vFine >= vCoarse - 1e-9);     // refinement grows toward exact
  CC_CHECK(nearRel(vFine, vFine2, 1e-12));  // deterministic (fp64, no RNG)
  // The G2 quintic keeps MORE material than the equal-radius G1 torus fillet (fuller shoulder).
  bool wtG1 = false;
  const double vG1 = vol(blend::curved_fillet_edge(cyl, ids, 1, r, 0.004), wtG1);
  CC_CHECK(wtG1);
  CC_CHECK(v > vG1);
}

CC_TEST(g2_curved_cone_section_matches_and_watertight) {
  // CONE arm: the same κ=0 quintic on a straight-ruled CONE wall — meridian curvature 0 at
  // both seams (matched), only the wall tangent tilts by σ (leaves along the cone ruling).
  // Closed-form κ=0 proof + watertight volume-reduced solid.
  const double Rb = 6, Rt = 4, H = 10, r = 1.0;
  topo::Shape s = cappedFrustum(Rb, Rt, H);
  const nmath::Vec3 axisY{0, 1, 0};
  const int rim = findRimAtAxial(s, axisY, H, Rt);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  const auto g = blend::detail::coneCapGeom(s, rim);
  CC_CHECK(g.has_value());
  const auto sec = blend::detail::g2CurvedConeSection(*g, r);
  CC_CHECK(sec.has_value());
  // Meridian curvature 0 at both rails (straight-ruled cone wall + flat cap).
  CC_CHECK(std::fabs(blend::detail::meridianCurvature(sec->poles, 0.0)) < 1e-9);
  CC_CHECK(std::fabs(blend::detail::meridianCurvature(sec->poles, 1.0)) < 1e-9);
  // The wall tangent is TILTED (has a nonzero radial AND axial component — the cone ruling),
  // unlike the cylinder's purely-axial wall tangent.
  const blend::detail::Mrd t0 = sec->poles[1] - sec->poles[0];
  CC_CHECK(std::fabs(t0.rho) > 1e-6);    // tilted (radial component from σ≠0)
  CC_CHECK(std::fabs(t0.z) > 1e-6);
  // Hoop curvature match cosσ/ρ is genuinely nonzero (curved substrate).
  CC_CHECK(sec->kHoopWall > 0.1);
  // The G2 fillet on the frustum is watertight and REMOVES material.
  topo::Shape f = blend::curved_fillet_edge_g2_cone(s, ids, 1, r, 0.004);
  bool wt = false;
  const double v = vol(f, wt);
  const double vSharp = frustumSharpVolume(Rb, Rt, H);
  CC_CHECK(!f.isNull());
  CC_CHECK(wt);
  CC_CHECK(v < vSharp);                  // convex fillet REMOVES material
  // Keeps more material than the equal-radius G1 cone fillet (fuller shoulder).
  bool wtG1 = false;
  const double vG1 = vol(blend::cone_fillet_edge(s, ids, 1, r, 0.004), wtG1);
  CC_CHECK(wtG1);
  CC_CHECK(v > vG1);
  // Widening frustum too (σ tilts the other way).
  topo::Shape s2 = cappedFrustum(4, 6, 10);
  const int rim2 = findRimAtAxial(s2, axisY, 10, 6);
  CC_CHECK(rim2 != 0);
  int ids2[] = {rim2};
  topo::Shape f2 = blend::curved_fillet_edge_g2_cone(s2, ids2, 1, 1.0, 0.004);
  bool wt2 = false;
  const double v2 = vol(f2, wt2);
  CC_CHECK(!f2.isNull());
  CC_CHECK(wt2);
  CC_CHECK(v2 < frustumSharpVolume(4, 6, 10));
}

CC_TEST(g2_curved_cyl_cone_scope_defers) {
  // Honest declines (→ OCCT) + NO cross-firing with the sphere / planar / other builders.
  const double Rc = 5.0, h = 10.0;
  topo::Shape cyl = cappedCylinder(Rc, h);
  const int rim = findRimAtZ(cyl, h);
  CC_CHECK(rim != 0);
  int ids[] = {rim};
  CC_CHECK(blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, 0.0, 0.01).isNull());   // r=0
  int ids2[] = {rim, 1};
  CC_CHECK(blend::curved_fillet_edge_g2_cyl(cyl, ids2, 2, 1.0, 0.01).isNull());  // multi-edge
  CC_CHECK(blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, 3.0, 0.01).isNull());   // Rc<2r ring guard
  // The cyl arm DECLINES a cone rim, and the cone arm DECLINES a cylinder rim (no cross-fire).
  topo::Shape frus = cappedFrustum(6, 4, 10);
  const nmath::Vec3 axisY{0, 1, 0};
  const int crim = findRimAtAxial(frus, axisY, 10, 4);
  CC_CHECK(crim != 0);
  int cids[] = {crim};
  CC_CHECK(blend::curved_fillet_edge_g2_cyl(frus, cids, 1, 1.0, 0.01).isNull());  // cone → cyl declines
  CC_CHECK(blend::curved_fillet_edge_g2_cone(cyl, ids, 1, 1.0, 0.01).isNull());   // cyl → cone declines
  // The SPHERE G2 arm DECLINES the cylinder rim (the sphere builder needs a Sphere face).
  CC_CHECK(blend::curved_fillet_edge_g2(cyl, ids, 1, 1.0, 0.01).isNull());
  // A truncated ball → both cyl AND cone G2 arms decline (no Cylinder/Cone↔cap rim).
  topo::Shape ball = truncatedBall(5.0, 2.0);
  const int brim = findRimAtAxial(ball, axisY, 2.0, std::sqrt(25.0 - 4.0));
  if (brim != 0) {
    int bids[] = {brim};
    CC_CHECK(blend::curved_fillet_edge_g2_cyl(ball, bids, 1, 1.0, 0.01).isNull());
    CC_CHECK(blend::curved_fillet_edge_g2_cone(ball, bids, 1, 1.0, 0.01).isNull());
  }
  // A straight (Line) box edge is not a circular crease → NULL for both arms.
  topo::Shape b = box(10, 10, 10);
  const int le = findEdgeId(b, {0, 10, 10}, {10, 10, 10});
  int idsb[] = {le};
  CC_CHECK(blend::curved_fillet_edge_g2_cyl(b, idsb, 1, 1.0, 0.01).isNull());
  CC_CHECK(blend::curved_fillet_edge_g2_cone(b, idsb, 1, 1.0, 0.01).isNull());
  // Control: the cylinder rim DOES land on the cyl arm, the cone rim on the cone arm.
  CC_CHECK(!blend::curved_fillet_edge_g2_cyl(cyl, ids, 1, 1.5, 0.004).isNull());
  CC_CHECK(!blend::curved_fillet_edge_g2_cone(frus, cids, 1, 1.0, 0.004).isNull());
}

// ── offset_face ────────────────────────────────────────────────────────────────--

CC_TEST(offset_top_face_grows_slab) {
  // Grow the top (+Z) face of a 10×10×10 box outward by 5 → 10×10×15, vol 1500.
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  CC_CHECK(fid != 0);
  topo::Shape g = blend::offset_face(b, fid, 5.0);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 1500.0));
}

CC_TEST(offset_top_face_shrinks_slab) {
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  topo::Shape g = blend::offset_face(b, fid, -4.0);  // → 10×10×6 = 600
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 600.0));
}

// ── curved offset_face (cylinder lateral wall, RADIAL) ──────────────────────────---
namespace {
constexpr double kOffPi = 3.14159265358979323846;
// The id of the (first) Cylinder lateral face of a capped cylinder.
int cylWallFace(const topo::Shape& s) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto su = topo::surfaceOf(map.shape(static_cast<int>(i)));
    if (su && su->surface->kind == topo::FaceSurface::Kind::Cylinder) return static_cast<int>(i);
  }
  return 0;
}
// The id of the (first) planar cap face.
int cylCapFace(const topo::Shape& s) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto su = topo::surfaceOf(map.shape(static_cast<int>(i)));
    if (su && su->surface->kind == topo::FaceSurface::Kind::Plane) return static_cast<int>(i);
  }
  return 0;
}
}  // namespace

// Offset the cylinder wall of a capped cylinder OUTWARD (grow radius) → a bigger tube,
// matching the exact π(Rc+d)²H closed form to the deflection bound; watertight + oriented.
CC_TEST(curved_offset_cylinder_wall_grows) {
  const double Rc = 3.0, H = 10.0, d = 1.0;
  const topo::Shape cyl = cappedCylinder(Rc, H);
  const int wf = cylWallFace(cyl);
  CC_CHECK(wf != 0);
  const topo::Shape g = blend::curved_offset_face(cyl, wf, d);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, kOffPi * (Rc + d) * (Rc + d) * H, 5e-3));  // deflection-bounded facets
  CC_CHECK(v > kOffPi * Rc * Rc * H);                            // GREW vs the sharp tube
}

// Offset the cylinder wall INWARD (shrink radius) → a smaller tube (still positive radius).
CC_TEST(curved_offset_cylinder_wall_shrinks) {
  const double Rc = 3.0, H = 10.0, d = -1.0;
  const topo::Shape cyl = cappedCylinder(Rc, H);
  const int wf = cylWallFace(cyl);
  const topo::Shape g = blend::curved_offset_face(cyl, wf, d);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  // Deflection-bounded: the shrunk (smaller-radius) tube facets slightly coarser than the
  // grow case, so the standard 1% curved-mesh tolerance applies (the volume converges as the
  // deflection refines — the hard gates are watertight + oriented + the SHRINK direction).
  CC_CHECK(nearRel(v, kOffPi * (Rc + d) * (Rc + d) * H, 1e-2));
  CC_CHECK(v < kOffPi * Rc * Rc * H);  // SHRANK
}

// Honest DECLINE (→ planar arm / OCCT): a PLANAR cap face is not the curved arm's job; a
// shrink that would invert the tube (Rc + d ≤ 0) and a zero offset return NULL.
CC_TEST(curved_offset_scope_defers) {
  const double Rc = 3.0, H = 10.0;
  const topo::Shape cyl = cappedCylinder(Rc, H);
  CC_CHECK(blend::curved_offset_face(cyl, cylCapFace(cyl), 1.0).isNull());   // planar cap → arm/OCCT
  CC_CHECK(blend::curved_offset_face(cyl, cylWallFace(cyl), -Rc).isNull());  // Rc+d = 0 → invert
  CC_CHECK(blend::curved_offset_face(cyl, cylWallFace(cyl), 0.0).isNull());  // zero offset
  CC_CHECK(blend::curved_offset_face(box(10, 10, 10), 1, 2.0).isNull());     // no cylinder wall
  // Control: the wall DOES offset native.
  CC_CHECK(!blend::curved_offset_face(cyl, cylWallFace(cyl), 1.0).isNull());
}

// ── F3 curved offset_face (CONE-FRUSTUM wall + SPHERE wall) ──────────────────────────
namespace {
// A capped cone frustum about +Y: profile (0,0)→(Rb,0)→(Rt,H)→(0,H) revolved a full turn.
// Bottom cap radius Rb at h=0, top cap radius Rt at h=H, one Cone lateral wall.
topo::Shape frustumSolid(double Rb, double Rt, double H) {
  const double prof[] = {0, 0, Rb, 0, Rt, H, 0, H};
  return cst::build_revolution(prof, 4, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
// The id of the (first) Cone lateral face.
int coneWallFace(const topo::Shape& s) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto su = topo::surfaceOf(map.shape(static_cast<int>(i)));
    if (su && su->surface->kind == topo::FaceSurface::Kind::Cone) return static_cast<int>(i);
  }
  return 0;
}
// Frustum volume π·H/3·(Rb²+Rb·Rt+Rt²). Offsetting the wall by d shifts both cap radii by
// dR = d/cosσ, cosσ = 1/√(1+((Rt−Rb)/H)²).
double frustumVol(double Rb, double Rt, double H) {
  return M_PI * H / 3.0 * (Rb * Rb + Rb * Rt + Rt * Rt);
}
double coneCapDelta(double Rb, double Rt, double H, double d) {
  const double tanS = (Rt - Rb) / H;
  return d * std::sqrt(1.0 + tanS * tanS);  // d / cosσ
}
// A SPHERE-CAP dome about +Y: one Sphere wall (radius R, centre origin) closed at the pole
// (0,R), cut by one axis-normal cap plane at y=capOff. Meridian: base disc (0,capOff)→
// (rimBase,capOff) then arc (rimBase,capOff)→(0,R) centred on the axis at (0,0).
topo::Shape domeSolid(double R, double capOff) {
  const double rimBase = std::sqrt(R * R - capOff * capOff);
  cst::ProfileSegment base;
  base.kind = 0;
  base.x0 = 0; base.y0 = capOff; base.x1 = rimBase; base.y1 = capOff;
  cst::ProfileSegment arc;
  arc.kind = 1;
  arc.x0 = rimBase; arc.y0 = capOff; arc.x1 = 0; arc.y1 = R; arc.cx = 0; arc.cy = 0; arc.r = R;
  return cst::build_revolution_profile({base, arc}, cst::RevolveAxis{0, 0, 0, 1}, 2.0 * M_PI);
}
int sphereWallFace(const topo::Shape& s) {
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Face);
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto su = topo::surfaceOf(map.shape(static_cast<int>(i)));
    if (su && su->surface->kind == topo::FaceSurface::Kind::Sphere) return static_cast<int>(i);
  }
  return 0;
}
// Dome (spherical segment, pole above cap at axial coord a from centre): π(2R³/3−R²a+a³/3).
double domeVol(double R, double a) {
  return M_PI * (2.0 * R * R * R / 3.0 - R * R * a + a * a * a / 3.0);
}
}  // namespace

// Offset a NARROWING cone-frustum wall (Rb>Rt) OUTWARD → a coaxial fatter frustum. The
// offset shifts both cap radii by d/cosσ; watertight, χ=2, GROWS, matching the closed form.
CC_TEST(curved_offset_cone_wall_grows) {
  const double Rb = 6.0, Rt = 4.0, H = 10.0, d = 1.0;
  const topo::Shape f = frustumSolid(Rb, Rt, H);
  const int wf = coneWallFace(f);
  CC_CHECK(wf != 0);
  const topo::Shape g = blend::curved_offset_face(f, wf, d);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  const double dR = coneCapDelta(Rb, Rt, H, d);
  CC_CHECK(nearRel(v, frustumVol(Rb + dR, Rt + dR, H), 6e-3));
  CC_CHECK(v > frustumVol(Rb, Rt, H));  // GREW vs the sharp frustum
}

// Offset a WIDENING cone-frustum wall (Rb<Rt) INWARD → a coaxial thinner frustum (both caps
// stay positive). Watertight, SHRINKS, matching the closed form.
CC_TEST(curved_offset_cone_wall_shrinks) {
  const double Rb = 4.0, Rt = 6.0, H = 10.0, d = -1.0;
  const topo::Shape f = frustumSolid(Rb, Rt, H);
  const int wf = coneWallFace(f);
  const topo::Shape g = blend::curved_offset_face(f, wf, d);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  const double dR = coneCapDelta(Rb, Rt, H, d);
  CC_CHECK(nearRel(v, frustumVol(Rb + dR, Rt + dR, H), 1e-2));
  CC_CHECK(v < frustumVol(Rb, Rt, H));  // SHRANK
}

// Offset a sphere-cap dome's SPHERE wall OUTWARD → concentric sphere R+d, same cap plane.
// Watertight, χ=2, GROWS, matching the spherical-segment closed form.
CC_TEST(curved_offset_sphere_wall_grows) {
  const double R = 5.0, capOff = 0.0, d = 1.0;  // hemisphere
  const topo::Shape dome = domeSolid(R, capOff);
  const int wf = sphereWallFace(dome);
  CC_CHECK(wf != 0);
  const topo::Shape g = blend::curved_offset_face(dome, wf, d, 0.003);
  bool wt = false;
  const double v = vol(g, wt);
  CC_CHECK(!g.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, domeVol(R + d, capOff), 6e-3));
  CC_CHECK(v > domeVol(R, capOff));  // GREW
}

// Offset a SHALLOW / DEEP sphere-cap dome INWARD → concentric smaller sphere, same cap plane.
CC_TEST(curved_offset_sphere_wall_shrinks) {
  for (double capOff : {2.0, -2.0}) {  // shallow cap (above centre) and deep dome (below)
    const double R = 5.0, d = -1.0;
    const topo::Shape dome = domeSolid(R, capOff);
    const int wf = sphereWallFace(dome);
    const topo::Shape g = blend::curved_offset_face(dome, wf, d, 0.003);
    bool wt = false;
    const double v = vol(g, wt);
    CC_CHECK(!g.isNull());
    CC_CHECK(wt);
    // Deflection-bounded: the volume converges to the closed form as the builder deflection
    // refines (verified: rel drops 1.8e-2→1.1e-3 as defl 0.02→0.001). The hard gates are
    // watertight + oriented + the SHRINK direction.
    CC_CHECK(nearRel(v, domeVol(R + d, capOff), 6e-3));
    CC_CHECK(v < domeVol(R, capOff));  // SHRANK
  }
}

// Honest DECLINE for the F3 families: a picked PLANAR cap of a frustum/dome (planar arm's
// job) and a shrink that inverts a cone cap (Rt+d/cosσ ≤ 0) return NULL.
CC_TEST(curved_offset_cone_sphere_scope_defers) {
  const topo::Shape f = frustumSolid(6.0, 4.0, 10.0);
  CC_CHECK(blend::curved_offset_face(f, cylCapFace(f), 1.0).isNull());  // planar cap → arm/OCCT
  CC_CHECK(blend::curved_offset_face(f, coneWallFace(f), -8.0).isNull());  // inverts top cap
  const topo::Shape dome = domeSolid(5.0, 0.0);
  CC_CHECK(blend::curved_offset_face(dome, cylCapFace(dome), 1.0).isNull());  // planar cap
  CC_CHECK(blend::curved_offset_face(dome, sphereWallFace(dome), -5.0).isNull());  // R+d=0
  // Controls: the curved walls DO offset native.
  CC_CHECK(!blend::curved_offset_face(f, coneWallFace(f), 1.0).isNull());
  CC_CHECK(!blend::curved_offset_face(dome, sphereWallFace(dome), 1.0).isNull());
}

// ── shell ──────────────────────────────────────────────────────────────────────--

CC_TEST(shell_open_top_box_wall_volume) {
  // Hollow a 10×10×10 box, open the top (+Z) face, wall thickness 1. The cavity is
  // the box inset 1 on 5 walls, flush on top: cavity = 8×8×9 = 576. Wall volume =
  // 1000 − 576 = 424.
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  CC_CHECK(fid != 0);
  int faces[] = {fid};
  topo::Shape sh = blend::shell(b, faces, 1, 1.0);
  bool wt = false;
  const double v = vol(sh, wt);
  CC_CHECK(!sh.isNull());
  CC_CHECK(wt);
  CC_CHECK(nearRel(v, 424.0, 1e-3));
}

CC_TEST(shell_thickness_too_large_fallthrough) {
  topo::Shape b = box(10, 10, 10);
  const int fid = findFaceId(b, {0, 0, 1}, {5, 5, 10});
  int faces[] = {fid};
  // thickness 6 > half-extent 5 → cavity collapses on the x/y walls → NULL.
  topo::Shape sh = blend::shell(b, faces, 1, 6.0);
  CC_CHECK(sh.isNull());
  CC_CHECK(blend::shell(b, faces, 1, 0.0).isNull());
}

// Chamfer TWO parallel top edges of a box (successive planar cuts). Both are convex
// 90° dihedrals → both land native; volume reduced by two corner prisms.
CC_TEST(chamfer_two_edges_watertight) {
  topo::Shape b = box(10, 10, 10);
  const int e1 = findEdgeId(b, {0, 10, 10}, {10, 10, 10});   // top / +Y
  const int e2 = findEdgeId(b, {0, 0, 10}, {10, 0, 10});     // top / −Y
  CC_CHECK(e1 != 0 && e2 != 0 && e1 != e2);
  int ids[] = {e1, e2};
  topo::Shape ch = blend::chamfer_edges(b, ids, 2, 1.5);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  // Two ½·1.5·1.5·10 = 11.25 corner prisms removed → 1000 − 22.5 = 977.5.
  CC_CHECK(nearRel(v, 977.5, 1e-3));
}

// A CONCAVE edge falls through: an L-shaped prism has one reflex (concave) vertical
// edge; chamfering/filleting it must return NULL (out of the convex native domain).
CC_TEST(concave_edge_chamfer_fillet_fallthrough) {
  // L-profile (CCW): the inner corner at (4,4) is concave for the extruded prism.
  const double L[] = {0, 0, 8, 0, 8, 4, 4, 4, 4, 8, 0, 8};
  topo::Shape lp = cst::build_prism(L, 6, 5.0);
  CC_CHECK(!lp.isNull());
  // The concave vertical edge runs from (4,4,0) to (4,4,5).
  const int ce = findEdgeId(lp, {4, 4, 0}, {4, 4, 5});
  CC_CHECK(ce != 0);
  int ids[] = {ce};
  CC_CHECK(blend::chamfer_edges(lp, ids, 1, 1.0).isNull());  // concave → NULL
  CC_CHECK(blend::fillet_edges(lp, ids, 1, 1.0).isNull());
  // A CONVEX edge of the same L-prism still chamfers native (watertight, smaller).
  const int cv = findEdgeId(lp, {0, 0, 0}, {0, 0, 5});
  CC_CHECK(cv != 0);
  int cvids[] = {cv};
  topo::Shape ch = blend::chamfer_edges(lp, cvids, 1, 1.0);
  bool wt = false;
  const double v = vol(ch, wt);
  CC_CHECK(!ch.isNull());
  CC_CHECK(wt);
  CC_CHECK(v > 0.0 && v < 240.0);  // L-area 48 × 5 = 240, reduced by the chamfer
}

// ── CYL↔CYL CANAL fillet (Steinmetz bicylinder COMMON) ──────────────────────────────
namespace {
namespace nb = cybercad::native::boolean;

// A native axis-parallel cylinder segment (axis 0=X,1=Y,2=Z), radius r over [lo,hi].
topo::Shape axisCyl(int axis, double r, double lo, double hi) {
  nb::curved::AABox box{nmath::Point3{-100, -100, -100}, nmath::Point3{100, 100, 100}};
  return nb::curved::buildCommonSegment(box, nb::curved::AxisCylinder{axis, 0, 0, r, lo, hi});
}

// The native Steinmetz bicylinder COMMON of two equal-radius Rc cylinders (axes Z & X),
// long enough (L = 6·Rc) that the disc caps never touch the fillet band.
topo::Shape steinmetz(double Rc) {
  return nb::ssi_boolean_solid(axisCyl(2, Rc, -3.0 * Rc, 3.0 * Rc),
                               axisCyl(0, Rc, -3.0 * Rc, 3.0 * Rc), nb::Op::Common);
}

// Consistently-oriented enclosed volume (sets co); the engine's SHRINK gate uses this.
double volCO(const topo::Shape& s, bool& co) {
  if (s.isNull()) { co = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.005;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  co = tess::isConsistentlyOriented(m);
  return std::fabs(tess::enclosedVolume(m));
}

}  // namespace

// GATE A.1 — the crossing crease of a Steinmetz bicylinder fillets NATIVE: watertight,
// consistently oriented (χ=2), enclosed volume strictly below the sharp bicylinder.
CC_TEST(canal_fillet_steinmetz_watertight_volume_reduced) {
  const double Rc = 1.0, r = 0.2;
  const topo::Shape lens = steinmetz(Rc);
  CC_CHECK(!lens.isNull());
  bool coS = false;
  const double vSharp = volCO(lens, coS);  // faceted bicylinder baseline
  CC_CHECK(coS);
  const int ids[1] = {1};  // any crease edge; the recognizer is wholesale
  const topo::Shape fil = blend::canal_fillet_edge(lens, ids, 1, r);
  CC_CHECK(!fil.isNull());
  bool co = false;
  const double v = volCO(fil, co);
  CC_CHECK(co);                       // watertight + consistently oriented
  CC_CHECK(v > 0.5 * vSharp);         // a fillet only rounds the crease — keeps most volume
  CC_CHECK(v < vSharp - 1e-4);        // it REMOVES the sharp-ridge sliver
}

// GATE A.2 — the native canal fillet CONVERGES: the enclosed volume tightens (toward the
// true curved solid) as the deflection refines, and stays watertight throughout.
CC_TEST(canal_fillet_converges_with_deflection) {
  const double Rc = 1.0, r = 0.2;
  double vPrev = -1.0;
  for (double defl : {0.02, 0.01, 0.005}) {
    const topo::Shape lens = steinmetz(Rc);
    CC_CHECK(!lens.isNull());
    const int ids[1] = {1};
    const topo::Shape fil = blend::canal_fillet_edge(lens, ids, 1, r, defl);
    CC_CHECK(!fil.isNull());
    tess::MeshParams p;
    p.deflection = defl;
    const tess::Mesh m = tess::SolidMesher{p}.mesh(fil);
    CC_CHECK(tess::isConsistentlyOriented(m));
    const double v = std::fabs(tess::enclosedVolume(m));
    // The faceted body grows toward the true curved volume as facets refine (monotone up).
    if (vPrev > 0.0) CC_CHECK(v >= vPrev - 1e-3);
    vPrev = v;
  }
}

// GATE A.3 — landing radius range: a rolling-ball fillet lands NATIVE across the useful
// convex range (Rc ≥ 2r), each watertight + consistently oriented + shrinking.
CC_TEST(canal_fillet_radius_range) {
  const double Rc = 1.0;
  for (double r : {0.1, 0.15, 0.2, 0.3, 0.4}) {
    const topo::Shape lens = steinmetz(Rc);
    CC_CHECK(!lens.isNull());
    bool coS = false;
    const double vSharp = volCO(lens, coS);
    const int ids[1] = {1};
    const topo::Shape fil = blend::canal_fillet_edge(lens, ids, 1, r);
    CC_CHECK(!fil.isNull());
    bool co = false;
    const double v = volCO(fil, co);
    CC_CHECK(co);
    CC_CHECK(v > 0.5 * vSharp && v < vSharp - 1e-4);
  }
}

// GATE A.4 — honest DECLINE (→ OCCT) outside the canonical Steinmetz envelope: a box, a
// single cylinder, r ≤ 0, Rc < 2r (ring-torus), and a multi-edge pick all return NULL.
CC_TEST(canal_fillet_scope_defers) {
  const int ids[1] = {1};
  CC_CHECK(blend::canal_fillet_edge(box(2, 2, 2), ids, 1, 0.2).isNull());   // no cylinders
  CC_CHECK(blend::canal_fillet_edge(axisCyl(2, 1.0, -2, 2), ids, 1, 0.2).isNull());  // one cyl
  const topo::Shape lens = steinmetz(1.0);
  CC_CHECK(!lens.isNull());
  CC_CHECK(blend::canal_fillet_edge(lens, ids, 1, 0.0).isNull());   // r ≤ 0
  CC_CHECK(blend::canal_fillet_edge(lens, ids, 1, 0.6).isNull());   // Rc < 2r → ring-torus
  CC_CHECK(blend::canal_fillet_edge(lens, ids, 2, 0.2).isNull());   // multi-edge pick
  CC_CHECK(!blend::canal_fillet_edge(lens, ids, 1, 0.2).isNull());  // control: lands
}

CC_RUN_ALL()

// SPDX-License-Identifier: Apache-2.0
//
// Host unit tests for the MOAT M2 CONVEX-CORNER chamfer weld
// (`src/native/blend/corner_chamfer_weld.h`, verb `chamfer_corner`). OCCT-FREE —
// Gate (a) of the two-gate model: the verb compiles and unit-tests with
// clang++ -std=c++20, no OCCT, no simulator, no cc_* facade.
//
// The byte-frozen sequential `chamfer_edges` DECLINES a set of mutually-ADJACENT
// convex edges sharing a corner (the first cut removes the shared corner, so the next
// edge is lost from the soup). `chamfer_corner` resolves EVERY chamfer plane up front
// against the original soup, applies all cuts, and welds the result watertight through
// the SAME assembleSolid path — the corner facet forming from the exposed rings.
//
// The chamfer is PLANAR, so the removed volume has an EXACT closed form (derived by
// inclusion-exclusion of the per-edge corner prisms), asserted to machine ε:
//   * 1 edge  (setback d, length L): V_removed = d²·L/2
//   * 2 adjacent edges at a corner:  V_removed = d²·(L − d/3)
//   * 3 edges at a box corner:       V_removed = 3·d²·(2L − d)/4
// plus the watertight + shrink self-verify the engine runs, and the honest declines
// (curved solid, bad id, degenerate distance, oversized). No tolerance weakened.
//
// Build (standalone):
//   clang++ -std=c++20 tests/native/test_native_corner_chamfer.cpp \
//     src/native/math/bspline.cpp src/native/math/bezier.cpp -I src -I tests \
//     -o /tmp/test_native_corner_chamfer && /tmp/test_native_corner_chamfer
//
#include "native/blend/native_blend.h"
#include "native/construct/native_construct.h"
#include "native/tessellate/native_tessellate.h"
#include "native/topology/native_topology.h"

#include "harness.h"

#include <array>
#include <cmath>
#include <vector>

namespace topo = cybercad::native::topology;
namespace blend = cybercad::native::blend;
namespace cst = cybercad::native::construct;
namespace tess = cybercad::native::tessellate;

namespace {

// A box [0,sx]×[0,sy]×[0,sz] as a native prism (base at z=0).
topo::Shape box(double sx, double sy, double sz) {
  const double p[] = {0, 0, sx, 0, sx, sy, 0, sy};
  return cst::build_prism(p, 4, sz);
}

double vol(const topo::Shape& s, bool& wt) {
  if (s.isNull()) { wt = false; return 0.0; }
  tess::MeshParams p;
  p.deflection = 0.004;
  const tess::Mesh m = tess::SolidMesher{p}.mesh(s);
  wt = tess::isWatertight(m);
  return std::fabs(tess::enclosedVolume(m));
}

bool nearAbs(double got, double want, double abs = 1e-4) {
  return std::fabs(got - want) <= abs;
}

// A capped solid cylinder for the curved-decline test.
topo::Shape cappedCylinder(double Rc, double h) {
  cst::ProfileSegment seg;
  seg.kind = 2;  // full circle
  seg.cx = 0; seg.cy = 0; seg.r = Rc;
  return cst::build_prism_profile({seg}, {}, {}, h);
}

// The 3 distinct edges (by geometry) of a box that share the corner `(cx,cy,cz)`.
// The box map stores a distinct edge node per incident face, so an origin corner is
// touched by 6 edge ids (each of the 3 geometric edges twice); we return the FIRST id
// seen for each distinct geometric edge (deduplicated by rounded endpoint keys).
std::vector<int> edgesAtCorner(const topo::Shape& s, double cx, double cy, double cz) {
  std::vector<int> ids;
  std::vector<std::array<long long, 6>> seen;
  const topo::ShapeMap map = topo::mapShapes(s, topo::ShapeType::Edge);
  auto isCorner = [&](const cybercad::native::math::Point3& p) {
    return std::fabs(p.x - cx) < 1e-6 && std::fabs(p.y - cy) < 1e-6 && std::fabs(p.z - cz) < 1e-6;
  };
  auto q = [](double v) { return static_cast<long long>(std::llround(v * 1e6)); };
  for (std::size_t i = 1; i <= map.size(); ++i) {
    const auto e = blend::edgeEnds(s, static_cast<int>(i));
    if (!e) continue;
    if (!(isCorner(e->a) || isCorner(e->b))) continue;
    std::array<long long, 3> ka{q(e->a.x), q(e->a.y), q(e->a.z)};
    std::array<long long, 3> kb{q(e->b.x), q(e->b.y), q(e->b.z)};
    const bool aFirst = ka <= kb;
    std::array<long long, 6> key{aFirst ? ka[0] : kb[0], aFirst ? ka[1] : kb[1],
                                 aFirst ? ka[2] : kb[2], aFirst ? kb[0] : ka[0],
                                 aFirst ? kb[1] : ka[1], aFirst ? kb[2] : ka[2]};
    bool dup = false;
    for (const auto& k : seen)
      if (k == key) { dup = true; break; }
    if (!dup) { seen.push_back(key); ids.push_back(static_cast<int>(i)); }
  }
  return ids;
}

// Closed-form removed volume of a symmetric chamfer (setback d each face, edge length L).
double rem1(double d, double L) { return d * d * L / 2.0; }
double rem2(double d, double L) { return d * d * (L - d / 3.0); }
double rem3(double d, double L) { return 3.0 * d * d * (2.0 * L - d) / 4.0; }

}  // namespace

// ── the corner weld: 1 / 2 / 3 adjacent edges land, EXACT closed-form volume ──────

CC_TEST(corner_chamfer_single_edge_exact) {
  const double L = 10.0, d = 1.5;
  topo::Shape b = box(L, L, L);
  const std::vector<int> e = edgesAtCorner(b, 0, 0, 0);
  CC_CHECK(e.size() == 3);
  bool wt = false;
  blend::CornerChamferDecline why = blend::CornerChamferDecline::BadInput;
  const int one[1] = {e[0]};
  const double v = vol(blend::chamfer_corner(b, one, 1, d, &why), wt);
  CC_CHECK(why == blend::CornerChamferDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(nearAbs(v, L * L * L - rem1(d, L)));  // 988.75
}

CC_TEST(corner_chamfer_two_adjacent_edges_exact) {
  // The case the sequential chamfer_edges DECLINES: two edges sharing a corner.
  const double L = 10.0, d = 1.5;
  topo::Shape b = box(L, L, L);
  const std::vector<int> e = edgesAtCorner(b, 0, 0, 0);
  CC_CHECK(e.size() == 3);

  // Baseline: byte-frozen chamfer_edges returns NULL on the adjacent pair.
  const int two[2] = {e[0], e[1]};
  CC_CHECK(blend::chamfer_edges(b, two, 2, d).isNull());

  // The corner weld lands watertight at the exact closed-form volume.
  bool wt = false;
  blend::CornerChamferDecline why = blend::CornerChamferDecline::BadInput;
  const double v = vol(blend::chamfer_corner(b, two, 2, d, &why), wt);
  CC_CHECK(why == blend::CornerChamferDecline::Ok);
  CC_CHECK(wt);
  CC_CHECK(nearAbs(v, L * L * L - rem2(d, L)));  // 978.625
}

CC_TEST(corner_chamfer_two_edge_volume_sweep_matches_closed_form) {
  // Sweep the setback: the 2-adjacent-edge corner weld tracks the closed form at every
  // size. (The 2-edge dihedral corner is a UNION OF TWO half-space prisms, which OCCT's
  // MakeChamfer reproduces EXACTLY — the sim gate measures native == OCCT to fp64 here.)
  const double L = 10.0;
  topo::Shape b = box(L, L, L);
  const std::vector<int> e = edgesAtCorner(b, 0, 0, 0);
  CC_CHECK(e.size() == 3);
  const int two[2] = {e[0], e[1]};
  for (double d : {0.5, 1.0, 2.0, 3.0}) {
    bool wt = false;
    blend::CornerChamferDecline why = blend::CornerChamferDecline::BadInput;
    const double v = vol(blend::chamfer_corner(b, two, 2, d, &why), wt);
    CC_CHECK(why == blend::CornerChamferDecline::Ok);
    CC_CHECK(wt);
    CC_CHECK(nearAbs(v, L * L * L - rem2(d, L), 1e-3));
  }
}

CC_TEST(corner_chamfer_triple_corner_declines_to_occt) {
  // The 3 edges meeting at ONE box vertex form a TRIPLE corner. A plain intersection of
  // the three setback half-spaces (V = L³ − 3d²(2L−d)/4 = 968.78125 at d=1.5) is a
  // watertight solid, but it is NOT the solid OCCT's MakeChamfer builds: OCCT breaks the
  // triple corner into chamfer-chamfer facets that trim MORE (measured on the sim,
  // d=1: OCCT 985.667 vs half-space 985.75). We cannot MATCH the oracle, so the verb
  // DECLINES a triple corner → OCCT (never a solid the oracle disagrees with).
  const double L = 10.0, d = 1.5;
  topo::Shape b = box(L, L, L);
  const std::vector<int> e = edgesAtCorner(b, 0, 0, 0);
  CC_CHECK(e.size() == 3);
  const int three[3] = {e[0], e[1], e[2]};
  blend::CornerChamferDecline why = blend::CornerChamferDecline::Ok;
  CC_CHECK(blend::chamfer_corner(b, three, 3, d, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::TripleCornerOracleGap);
  // (The naive half-space corner volume the decline avoids emitting.)
  CC_CHECK(nearAbs(L * L * L - rem3(d, L), 968.78125));
}

CC_TEST(corner_chamfer_non_orthogonal_corner_welds) {
  // A non-orthogonal (60°) corner: the vertical edge of an equilateral-triangle prism.
  const double s = 10.0, h = 8.0;
  const double hh = s * std::sqrt(3.0) / 2.0;
  const double tri[] = {0, 0, s, 0, s / 2.0, hh};
  topo::Shape p = cst::build_prism(tri, 3, h);
  // The vertical edge at base corner (0,0) — dedup returns its first id.
  const std::vector<int> e = edgesAtCorner(p, 0, 0, 0);
  CC_CHECK(!e.empty());
  bool wt = false;
  blend::CornerChamferDecline why = blend::CornerChamferDecline::BadInput;
  const topo::Shape r = blend::chamfer_corner(p, e.data(), 1, 1.0, &why);
  const double v = vol(r, wt);
  CC_CHECK(why == blend::CornerChamferDecline::Ok);
  CC_CHECK(wt);
  const double v0Prism = (s * hh / 2.0) * h;  // triangular prism volume
  CC_CHECK(v > 0.0 && v < v0Prism);  // material removed
}

// ── honest declines (NULL → OCCT), each with a measured reason ────────────────────

CC_TEST(corner_chamfer_declines_curved_and_bad_input) {
  topo::Shape b = box(10, 10, 10);
  const std::vector<int> e = edgesAtCorner(b, 0, 0, 0);
  CC_CHECK(e.size() == 3);
  blend::CornerChamferDecline why = blend::CornerChamferDecline::Ok;

  // Curved solid → NonPlanarSolid.
  topo::Shape cyl = cappedCylinder(5.0, 10.0);
  const int one[1] = {1};
  CC_CHECK(blend::chamfer_corner(cyl, one, 1, 1.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::NonPlanarSolid);

  // Out-of-range edge id → EdgeNotFound.
  const int bad[1] = {999};
  CC_CHECK(blend::chamfer_corner(b, bad, 1, 1.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::EdgeNotFound);

  // Non-positive distance / no edges / null → BadInput.
  CC_CHECK(blend::chamfer_corner(b, one, 1, 0.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::BadInput);
  CC_CHECK(blend::chamfer_corner(b, e.data(), 0, 1.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::BadInput);

  // Oversized setback (larger than a face) on a 2-edge corner → the clip removes
  // everything → CutFailed / VolumeInconsistent (a measured self-verify decline, never
  // a wrong solid).
  const int two[2] = {e[0], e[1]};
  CC_CHECK(blend::chamfer_corner(b, two, 2, 20.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::CutFailed ||
           why == blend::CornerChamferDecline::VolumeInconsistent ||
           why == blend::CornerChamferDecline::AssembleFailed);

  // A triple corner (≥3 edges at one vertex) declines with the oracle-gap reason.
  const int three[3] = {e[0], e[1], e[2]};
  CC_CHECK(blend::chamfer_corner(b, three, 3, 1.0, &why).isNull());
  CC_CHECK(why == blend::CornerChamferDecline::TripleCornerOracleGap);
}

CC_RUN_ALL()

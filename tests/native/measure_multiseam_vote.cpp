// SPDX-License-Identifier: Apache-2.0
// measure_multiseam_vote.cpp — BOOL-VOTE diagnostic probe for the fine-deflection VOLUME
// collapse of the multi-seam COMMON weld (MESH-COLLAR sharpened residual), swept on BOTH
// the asymmetric (a=4 valley ∩ b=6 dome) and the symmetric mirror-cups fixtures.
//
// MEASURED: where the mesher's seam-strip fallback fires (asym d=0.002, sym d=0.0018) the
// welded survivor mesh is watertight (be=0, nonmanif=0) but encloses ~6% LESS volume than
// the survivors' own divergence-theorem expectation — the two faces splice the SAME collar
// strip, the coincident duplicate triangles annihilate at the weld, and the collar bands'
// volume is lost, IDENTICALLY in every orientation configuration. Where the baseline weld
// suffices (asym d=0.0025 rim-pin, sym d=0.002) the welded volume tracks the expectation to
// ≤ 2e-4 relative. This probe separates the layers:
//
//   [FACES]   each survivor face meshed ALONE (single-face SolidMesher: no shared strip can
//             fire) — per-face signed volume contribution (divergence theorem) + be count
//   [RAW]     the appended per-face meshes, per orientation-repair config: signed volume
//             (the oracle-free assembly volume expectation) + baseline-weld histogram
//   [WELDED]  the full strip-pass SolidMesher mesh of the survivor solid, per config:
//             watertight/coherent/|V| — the configs weldMultiCoherent enumerates
//   [SIDES]   radial localization of the welded mesh's vertices near each seam (which
//             collar side each face's strip took)
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_multiseam_fixture.h"
#include "freeform_multiseam_asym_fixture.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace ffm = cybercad::native::boolean::ffmdetail;
namespace ffc = cybercad::native::boolean::ffcdetail;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ax = freeform_multiseam_asym_fixture;
namespace fx = freeform_freeform_multiseam_fixture;

static void histo(const tess::Mesh& m, int& open, int& nonmanif) {
  open = 0; nonmanif = 0;
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    if (uses == 1) ++open;
    else if (uses >= 3) ++nonmanif;
  }
}

// Localize defective edges radially (inner seam r1, outer seam r2, other) and probe the
// T-junction structure: an open edge with an open-boundary VERTEX lying on its interior
// (within tol of the segment, strictly between endpoints) is a T-junction defect.
static void defectCensus(const tess::Mesh& m, double tol, double r1, double r2) {
  int oIn = 0, oOut = 0, oOth = 0, nIn = 0, nOut = 0, nOth = 0;
  std::vector<std::pair<std::uint32_t, std::uint32_t>> openEdges;
  std::vector<char> openVert(m.vertices.size(), 0);
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    if (uses != 1 && uses < 3) continue;
    const auto& a = m.vertices[e.lo];
    const auto& b = m.vertices[e.hi];
    const double r = std::hypot(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
    const bool in = std::fabs(r - r1) < 0.03, out = std::fabs(r - r2) < 0.03;
    if (uses == 1) {
      (in ? oIn : out ? oOut : oOth)++;
      openEdges.push_back({e.lo, e.hi});
      openVert[e.lo] = openVert[e.hi] = 1;
    } else {
      (in ? nIn : out ? nOut : nOth)++;
    }
  }
  // T-junction test: for each open edge, an open vertex within tol of its interior.
  int tj = 0;
  for (const auto& [lo, hi] : openEdges) {
    const auto& a = m.vertices[lo];
    const auto& b = m.vertices[hi];
    const double ex = b.x - a.x, ey = b.y - a.y, ez = b.z - a.z;
    const double L2 = ex * ex + ey * ey + ez * ez;
    if (L2 <= 0) continue;
    bool isT = false;
    for (std::size_t v = 0; v < m.vertices.size() && !isT; ++v) {
      if (!openVert[v] || v == lo || v == hi) continue;
      const auto& p = m.vertices[v];
      const double t = ((p.x - a.x) * ex + (p.y - a.y) * ey + (p.z - a.z) * ez) / L2;
      if (t <= 0.05 || t >= 0.95) continue;
      const double dx = a.x + t * ex - p.x, dy = a.y + t * ey - p.y, dz = a.z + t * ez - p.z;
      if (dx * dx + dy * dy + dz * dz <= tol * tol) isT = true;
    }
    if (isT) ++tj;
  }
  std::printf("      [DEFECT] open in=%d out=%d oth=%d (T-junction-like=%d/%zu) | nonmanif in=%d out=%d oth=%d\n",
              oIn, oOut, oOth, tj, openEdges.size(), nIn, nOut, nOth);
}

// splitWallBySeams variant: identical to ffm::splitWallBySeams but the seam-edge 3-D
// polyline is the WLine's CANONICAL `point` (A.point≈B.point trace node) instead of the
// per-wall surface eval — so BOTH walls' seam chords are BIT-IDENTICAL.
static bool splitWallCanon(const topo::Shape& face, const std::vector<bo::ssi::WLine>& seams,
                           std::vector<ffm::WallRegion>& regions, double& tilingGap) {
  using namespace ffm;
  using bo::detail::shoelace;
  regions.clear();
  const auto sr = topo::surfaceOf(face);
  if (!sr || !sr->surface) return false;
  const topo::FaceSurface surface = *sr->surface;
  const topo::Orientation orient = face.orientation();
  const auto& parentWires = face.tshape()->children();
  if (parentWires.empty()) return false;
  const topo::Shape parentOuter = parentWires[0];

  std::vector<bo::detail::BndSeg> segs;
  std::vector<topo::Shape> pedges;
  if (!bo::detail::flattenOuter(face, 24, segs, pedges)) return false;
  tess::UVPolygon outerPoly;
  for (const auto& s : segs) outerPoly.push_back(s.a);
  const double parentArea = std::fabs(shoelace(outerPoly));
  const double scale = std::sqrt(std::max(parentArea, 1e-300));

  std::vector<SeamLoop> loops;
  for (const bo::ssi::WLine& s : seams) {
    SeamLoop L;
    tess::UVPolygon loop;
    if (!bo::stsdetail::seamLoopNodes(s, scale * 1e-6, loop)) return false;
    if (!bo::stsdetail::simpleLoop(loop)) return false;
    std::vector<cybercad::native::math::Point3> p3;
    for (std::size_t i = 0; i < loop.size(); ++i) p3.push_back(s.points[i].point);  // CANONICAL
    if (shoelace(loop) < 0.0) { std::reverse(loop.begin(), loop.end()); std::reverse(p3.begin(), p3.end()); }
    L.uv = loop; L.p3 = p3; L.area = std::fabs(shoelace(loop));
    for (const tess::UV& q : L.uv)
      if (!tess::pointInPolygon(outerPoly, q)) return false;
    loops.push_back(std::move(L));
  }
  const int n = static_cast<int>(loops.size());
  if (n < 1) return false;
  const std::vector<int> parent = immediateParents(loops);
  std::vector<std::vector<topo::Shape>> edges(n);
  for (int i = 0; i < n; ++i) edges[i] = loopEdges(loops[i], face.tshape());
  std::vector<std::vector<int>> children(n);
  std::vector<int> topLevel;
  for (int i = 0; i < n; ++i) {
    if (parent[i] < 0) topLevel.push_back(i);
    else children[parent[i]].push_back(i);
  }
  double sumArea = 0.0;
  for (int i = 0; i < n; ++i) {
    const topo::Shape outerWire = wireFromEdges(edges[i], false);
    std::vector<topo::Shape> holeWires;
    double holeArea = 0.0;
    for (int c : children[i]) { holeWires.push_back(wireFromEdges(edges[c], true)); holeArea += loops[c].area; }
    WallRegion reg;
    reg.face = topo::ShapeBuilder::makeFace(surface, outerWire, std::move(holeWires), orient);
    regions.push_back(std::move(reg));
    sumArea += loops[i].area - holeArea;
  }
  {
    std::vector<topo::Shape> holeWires;
    double holeArea = 0.0;
    for (int t : topLevel) { holeWires.push_back(wireFromEdges(edges[t], true)); holeArea += loops[t].area; }
    WallRegion reg;
    reg.face = topo::ShapeBuilder::makeFace(surface, parentOuter, std::move(holeWires), orient);
    reg.isBackground = true;
    regions.push_back(std::move(reg));
    sumArea += parentArea - holeArea;
  }
  tilingGap = std::fabs(parentArea - sumArea);
  return regions.size() == static_cast<std::size_t>(n + 1);
}

// Count welded-mesh vertices radially near each seam, split by side (inside/outside r_i),
// excluding the exact ring itself. Localizes which collar side each strip took.
static void sideCensus(const tess::Mesh& m, double r1, double r2) {
  const double d1 = 0.05 * r1, d2 = 0.05 * r2;  // collar insets (kCollarFrac = 0.05)
  int in1 = 0, out1 = 0, in2 = 0, out2 = 0;
  for (const auto& v : m.vertices) {
    const double r = std::hypot(v.x, v.y);
    auto near = [](double x, double t, double w) { return std::fabs(x - t) < 0.35 * w; };
    if (near(r, r1 - d1, d1)) ++in1;
    if (near(r, r1 + d1, d1)) ++out1;
    if (near(r, r2 - d2, d2)) ++in2;
    if (near(r, r2 + d2, d2)) ++out2;
  }
  std::printf("      [SIDES] collar-ring vertices: r1-δ=%d r1+δ=%d | r2-δ=%d r2+δ=%d\n",
              in1, out1, in2, out2);
}

static void sweepFixture(const char* name, const topo::Shape& A, const topo::Shape& B,
                         const std::vector<bo::ssi::WLine>& seams, double cf, double r1,
                         double r2, const std::vector<double>& deflections) {
  std::printf("=== BOOL-VOTE probe: %s COMMON, closed-form V=%.6f (r1=%.4f r2=%.4f) ===\n",
              name, cf, r1, r2);
  if (seams.size() != 2) { std::printf("NOT a 2-seam pose; abort.\n"); return; }

  using namespace ffc;
  using namespace ffm;
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  if (!freeformWall(*foA, &wallA, wA) || !freeformWall(*foB, &wallB, wB)) return;
  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(rekeyToB(s));

  for (double d : deflections) {
    std::printf("\n--- d=%.5f ---\n", d);
    std::vector<WallRegion> regA, regB;
    double gA = -1, gB = -1;
    if (!splitWallBySeams(wallA->face, seams, regA, gA) ||
        !splitWallBySeams(wallB->face, seamsB, regB, gB)) { std::printf("split failed\n"); continue; }

    tess::MeshParams mp; mp.deflection = d;
    const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
    const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
    const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);
    std::vector<topo::Shape> faces;
    std::size_t nFromA = 0;
    for (const auto& r : regA)
      if (subFaceHasMembership(r.face, meshB, bbB, d, bo::Membership::In)) { faces.push_back(r.face); ++nFromA; }
    for (const auto& r : regB)
      if (subFaceHasMembership(r.face, meshA, bbA, d, bo::Membership::In)) faces.push_back(r.face);
    std::printf("  survivors=%zu (nFromA=%zu)\n", faces.size(), nFromA);

    // [FACES] each survivor meshed ALONE (no shared strip possible).
    std::vector<tess::Mesh> faceMeshes;
    for (std::size_t i = 0; i < faces.size(); ++i) {
      const tess::Mesh fm = tess::SolidMesher(mp).mesh(faces[i]);
      int op = 0, nm = 0; histo(fm, op, nm);
      std::printf("  [FACE %zu] V=%zu T=%zu signedVol=%+.6f open=%d nonmanif=%d\n", i,
                  fm.vertices.size(), fm.triangles.size(), tess::enclosedVolume(fm), op, nm);
      faceMeshes.push_back(fm);
    }

    // [RAW] appended per-face meshes per orientation config: signed volume (divergence
    // estimate) + baseline-weld histogram. Config bitmask: bit i set => face i flipped
    // (winding reversed).
    const double weldTol = std::max(d * 0.5, 1e-7);
    for (unsigned cfg = 0; cfg < (1u << faces.size()); ++cfg) {
      tess::Mesh unionMesh;
      for (std::size_t i = 0; i < faceMeshes.size(); ++i) {
        tess::Mesh fm = faceMeshes[i];
        if (cfg & (1u << i))
          for (auto& t : fm.triangles) std::swap(t.b, t.c);
        unionMesh.append(fm);
      }
      const tess::Mesh basew = tess::VertexWelder{weldTol}.weld(unionMesh);
      int op = 0, nm = 0; histo(basew, op, nm);
      std::printf("  [RAW cfg=%u] signedVol=%+.6f | baseline-weld: open=%d nonmanif=%d wt=%d coh=%d vol=%+.6f\n",
                  cfg, tess::enclosedVolume(unionMesh), op, nm, (int)tess::isWatertight(basew),
                  (int)tess::isConsistentlyOriented(basew), tess::enclosedVolume(basew));
      if (cfg == 0) defectCensus(basew, weldTol, r1, r2);
    }

    // [WELDED] the verb's weld candidates (identity, last-flip, flip-B, flip-A) through the
    // full strip-pass SolidMesher.
    struct Cand { const char* nm; std::vector<std::size_t> flip; };
    std::vector<Cand> cands;
    cands.push_back({"identity", {}});
    cands.push_back({"last-flip", {faces.size() - 1}});
    { std::vector<std::size_t> b; for (std::size_t i = nFromA; i < faces.size(); ++i) b.push_back(i);
      cands.push_back({"flip-B", b}); }
    for (const Cand& c : cands) {
      std::vector<topo::Shape> f = faces;
      for (std::size_t i : c.flip) f[i] = f[i].reversedShape();
      const topo::Shape shell = topo::ShapeBuilder::makeShell(f);
      const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
      const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
      int op = 0, nm = 0; histo(m, op, nm);
      std::printf("  [WELDED %-9s] V=%zu T=%zu open=%d nonmanif=%d wt=%d coh=%d vol=%+.6f\n",
                  c.nm, m.vertices.size(), m.triangles.size(), op, nm,
                  (int)tess::isWatertight(m), (int)tess::isConsistentlyOriented(m),
                  tess::enclosedVolume(m));
      if (tess::isWatertight(m)) sideCensus(m, r1, r2);
    }

    // [CANON] the bit-identical-seam experiment: rebuild BOTH walls' regions with the seam
    // chords taken from the WLine's canonical 3-D `point` (identical across walls), re-select
    // the survivors, and weld through the full SolidMesher.
    {
      std::vector<WallRegion> cA, cB;
      double cgA = -1, cgB = -1;
      if (!splitWallCanon(wallA->face, seams, cA, cgA) ||
          !splitWallCanon(wallB->face, seamsB, cB, cgB)) {
        std::printf("  [CANON] split failed\n");
      } else {
        std::vector<topo::Shape> cfaces;
        std::size_t cnA = 0;
        for (const auto& r : cA)
          if (subFaceHasMembership(r.face, meshB, bbB, d, bo::Membership::In)) { cfaces.push_back(r.face); ++cnA; }
        for (const auto& r : cB)
          if (subFaceHasMembership(r.face, meshA, bbA, d, bo::Membership::In)) cfaces.push_back(r.face);
        std::printf("  [CANON] survivors=%zu gapA=%.2e gapB=%.2e\n", cfaces.size(), cgA, cgB);
        // Raw union of per-face meshes + baseline weld census.
        tess::Mesh unionMesh;
        for (const auto& f : cfaces) unionMesh.append(tess::SolidMesher(mp).mesh(f));
        const tess::Mesh basew = tess::VertexWelder{weldTol}.weld(unionMesh);
        int op = 0, nm = 0; histo(basew, op, nm);
        std::printf("  [CANON raw] signedVol=%+.6f | baseline-weld: open=%d nonmanif=%d wt=%d\n",
                    tess::enclosedVolume(unionMesh), op, nm, (int)tess::isWatertight(basew));
        defectCensus(basew, weldTol, r1, r2);
        for (const Cand& c : cands) {
          std::vector<topo::Shape> f = cfaces;
          for (std::size_t i : c.flip) f[i] = f[i].reversedShape();
          const topo::Shape shell = topo::ShapeBuilder::makeShell(f);
          const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
          const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
          int o2 = 0, n2 = 0; histo(m, o2, n2);
          std::printf("  [CANON weld %-9s] open=%d nonmanif=%d wt=%d coh=%d vol=%+.6f\n",
                      c.nm, o2, n2, (int)tess::isWatertight(m), (int)tess::isConsistentlyOriented(m),
                      tess::enclosedVolume(m));
          if (tess::isWatertight(m)) sideCensus(m, r1, r2);
        }
      }
    }
    std::fflush(stdout);
  }

  // [VERB] the DISAGREED=0 hazard check at the finest swept deflection: the verb WITH the
  // closed-form oracle and WITHOUT it (does it silently return a strip-pinched wrong-volume
  // solid, or honestly decline?).
  const double dFine = deflections.back();
  for (int withOracle = 1; withOracle >= 0; --withOracle) {
    bo::MultiSeamCutReport rep;
    const double cfArg = withOracle ? cf : std::numeric_limits<double>::quiet_NaN();
    const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common,
                                                                    dFine, &rep, cfArg);
    std::printf("[VERB d=%.5f COMMON %s] null=%d decline=%s wt=%d coh=%d vol=%.6f expect=%.6f (cf=%.6f)\n",
                dFine, withOracle ? "with-oracle" : "NO-oracle", (int)r.isNull(),
                bo::multiSeamCutDeclineName(rep.decline), (int)rep.watertight, (int)rep.coherent,
                rep.enclosedVolume, rep.expectedVolume, cf);
    std::fflush(stdout);
  }
}

int main() {
  sweepFixture("asym a=4 valley ∩ b=6 dome", ax::buildA(), ax::buildB(), ax::closedSeams(),
               ax::volCommon(), ax::seamR1(), ax::seamR2(), {0.0025, 0.002});
  std::printf("\n");
  sweepFixture("sym mirror-cups", fx::buildA(), fx::buildB(), fx::closedSeams(),
               fx::volCommon(), fx::seamR1(), fx::seamR2(), {0.002, 0.0018});
  return 0;
}

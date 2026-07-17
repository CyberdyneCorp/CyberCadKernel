// SPDX-License-Identifier: Apache-2.0
// measure_multiseam_fine.cpp — FINE-DEFLECTION diagnostic probe for the asymmetric
// multi-seam freeform↔freeform sew. The BOOL-MULTISEAM working band welds watertight to
// d≈0.0025; below it the sew declines. This harness pins WHERE the decline lives, per
// deflection, by separating the ASSEMBLY layer (boolean/, in-lane) from the MESHER weld
// (tessellate/, out-of-lane):
//
//   [ASSEMBLY] tiling gap of splitWallBySeams on each wall  (should be ~0 always)
//   [ASSEMBLY] survivor face count + per-region membership   (should be stable across d)
//   [MESHER]   raw survivor mesh histogram (open==1, nonmanif>=3), NO orientation repair
//              — the survivor set is a fixed, correct set of trimmed faces; if it fails to
//              weld watertight at fine d, the fault is the per-face CDT weld (frozen mesher)
//   [VERB]     freeformFreeformMultiSeamCutWithSeams decline + boundaryEdges
//   [MESHER]   per-radius localization of the residual boundary edges (inner r1 vs outer r2)
//
// A residual that (a) has tiling gap ~0 AND stable survivor set at fine d, but (b) shows
// open/nonmanif > 0 in the RAW survivor mesh, is an OUT-OF-LANE mesher-weld limiter, not an
// assembly limiter. A residual whose survivor set or tiling changes at fine d is in-lane.
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_multiseam_asym_fixture.h"

#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace ffm = cybercad::native::boolean::ffmdetail;
namespace ffc = cybercad::native::boolean::ffcdetail;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ax = freeform_multiseam_asym_fixture;

// Histogram + per-radius localization of unpaired/over-used edges. r is measured from the
// (0.5,0.5)-centred axis in world XY; the two seams sit at r1≈0.154 (inner) and r2≈0.365.
struct EdgeStats {
  int open = 0, nonmanif = 0;
  int openInner = 0, openOuter = 0, openOther = 0;
  int nmInner = 0, nmOuter = 0, nmOther = 0;
};

static EdgeStats histoLocalized(const tess::Mesh& m) {
  EdgeStats s;
  const double r1 = ax::seamR1(), r2 = ax::seamR2();
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    if (uses != 1 && uses < 3) continue;
    // edge midpoint radius (XY distance from the cup axis at world origin)
    const auto& a = m.vertices[e.lo];
    const auto& b = m.vertices[e.hi];
    const double mx = 0.5 * (a.x + b.x), my = 0.5 * (a.y + b.y);
    const double r = std::hypot(mx, my);
    const bool inner = std::fabs(r - r1) < 0.03;
    const bool outer = std::fabs(r - r2) < 0.03;
    if (uses == 1) {
      ++s.open;
      if (inner) ++s.openInner; else if (outer) ++s.openOuter; else ++s.openOther;
    } else {
      ++s.nonmanif;
      if (inner) ++s.nmInner; else if (outer) ++s.nmOuter; else ++s.nmOther;
    }
  }
  return s;
}

// Rebuild the exact survivor set the verb would weld (no orientation repair) and mesh it.
// Reports assembly witnesses (tiling gap, survivor count) and the raw mesh histogram.
static void probe(const topo::Shape& A, const topo::Shape& B,
                  const std::vector<bo::ssi::WLine>& seams, bo::FfOp op, double d) {
  using namespace ffc;
  using namespace ffm;
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  if (!foA || !foB) { std::printf("      recognise failed\n"); return; }
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  if (!freeformWall(*foA, &wallA, wA) || !freeformWall(*foB, &wallB, wB)) return;

  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(rekeyToB(s));
  std::vector<WallRegion> regA, regB;
  double gA = -1, gB = -1;
  const bool okA = splitWallBySeams(wallA->face, seams, regA, gA);
  const bool okB = splitWallBySeams(wallB->face, seamsB, regB, gB);
  std::printf("      [ASSEMBLY] splitA=%d gapA=%.3e  splitB=%d gapB=%.3e  (deflection-INDEPENDENT)\n",
              okA, gA, okB, gB);
  if (!okA || !okB) return;

  tess::MeshParams mp; mp.deflection = d;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);
  const bo::Membership wantA = op == bo::FfOp::Common ? bo::Membership::In : bo::Membership::Out;
  const bo::Membership wantB =
      op == bo::FfOp::Common || op == bo::FfOp::Cut ? bo::Membership::In : bo::Membership::Out;
  std::vector<topo::Shape> faces;
  std::size_t nFromA = 0;
  for (const auto& r : regA)
    if (subFaceHasMembership(r.face, meshB, bbB, d, wantA)) { faces.push_back(r.face); ++nFromA; }
  if (op == bo::FfOp::Cut || op == bo::FfOp::Fuse)
    collectAnalyticByMembership(*foA, meshB, bbB, d, bo::Membership::Out, faces), nFromA = faces.size();
  for (const auto& r : regB)
    if (subFaceHasMembership(r.face, meshA, bbA, d, wantB)) faces.push_back(r.face);
  if (op == bo::FfOp::Fuse)
    collectAnalyticByMembership(*foB, meshA, bbA, d, bo::Membership::Out, faces);

  const topo::Shape shell = topo::ShapeBuilder::makeShell(faces);
  const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  const EdgeStats s = histoLocalized(m);
  std::printf("      [MESHER raw] survivors=%zu tris=%zu open=%d(in=%d out=%d oth=%d) "
              "nonmanif=%d(in=%d out=%d oth=%d)\n",
              faces.size(), m.triangles.size(), s.open, s.openInner, s.openOuter, s.openOther,
              s.nonmanif, s.nmInner, s.nmOuter, s.nmOther);
}

int main() {
  std::printf("=== FINE-DEFLECTION multi-seam diagnostic (A valley a=%.0f, B dome b=%.0f) ===\n",
              ax::kAvalley, ax::kBdome);
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  std::printf("SSI trace: %zu closed loops (r1=%.4f inner, r2=%.4f outer)\n",
              seams.size(), ax::seamR1(), ax::seamR2());
  if (seams.size() != 2) { std::printf("NOT a 2-seam pose; abort.\n"); return 0; }

  struct { bo::FfOp op; const char* nm; double cf; } ops[] = {
      {bo::FfOp::Common, "COMMON", ax::volCommon()},
      {bo::FfOp::Cut, "CUT", ax::volCut()},
      {bo::FfOp::Fuse, "FUSE", ax::volA() + ax::volB() - ax::volCommon()},
  };
  // Sweep from the working band down into the fine-deflection residual region. The RAW
  // survivor histogram (full strip-pass SolidMesher, no orientation retry) IS the mesher's
  // weld verdict; the assembly witnesses (gap) are deflection-independent by construction.
  // We deliberately DO NOT call the 4-orientation verb weld here (it meshes the survivor
  // solid up to 4× per row — cost-prohibitive at fine deflection); the raw histogram's
  // open/nonmanif count is the decisive assembly-vs-mesher separator.
  const double defls[] = {0.0025, 0.002, 0.00125};
  for (const auto& o : ops) {
    std::printf("\n--- %s (closed-form %.6f) ---\n", o.nm, o.cf);
    for (double d : defls) {
      std::printf("  d=%.5f:\n", d);
      probe(A, B, seams, o.op, d);
      std::fflush(stdout);
    }
  }
  return 0;
}

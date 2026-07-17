// SPDX-License-Identifier: Apache-2.0
// measure_multiseam_asym.cpp — MEASUREMENT harness for the ASYMMETRIC (curvature-
// mismatched) multi-seam freeform↔freeform pose. Traces the two seams once, then runs
// freeformFreeformMultiSeamCutWithSeams for COMMON/CUT/FUSE at several deflections and
// reports: the decline verdict, watertight/coherent flags, boundaryEdges residual,
// meshed vs closed-form volume, and — via a direct survivor-mesh rebuild — the RAW
// edge-use histogram (open==1, nonmanifold>=3) so a decline is mapped to its mechanism.
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

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

static void histo(const tess::Mesh& m, int& open, int& nonmanif) {
  open = 0; nonmanif = 0;
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    (void)e;
    if (uses == 1) ++open;
    else if (uses >= 3) ++nonmanif;
  }
}

// Rebuild + weld the survivor faces exactly as the verb's core does, then mesh and report
// the RAW histogram (so we see the residual even when the verb declines).
static void rawSurvivorHisto(const topo::Shape& A, const topo::Shape& B,
                             const std::vector<bo::ssi::WLine>& seams, bo::FfOp op, double d) {
  using namespace ffc;
  using namespace ffm;
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  if (!foA || !foB) { std::printf("      raw: recognise failed\n"); return; }
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  if (!freeformWall(*foA, &wallA, wA) || !freeformWall(*foB, &wallB, wB)) return;

  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(rekeyToB(s));
  std::vector<WallRegion> regA, regB;
  double gA, gB;
  if (!splitWallBySeams(wallA->face, seams, regA, gA)) return;
  if (!splitWallBySeams(wallB->face, seamsB, regB, gB)) return;

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
  int open, nonmanif; histo(m, open, nonmanif);
  std::printf("      raw survivor mesh (no repair): survivors=%zu tris=%zu open=%d nonmanif=%d\n",
              faces.size(), m.triangles.size(), open, nonmanif);
}

int main() {
  std::printf("=== ASYMMETRIC multi-seam pose (A valley a=%.0f, B dome b=%.0f) ===\n",
              ax::kAvalley, ax::kBdome);
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  std::printf("SSI trace: %zu closed loops (expect 2)\n", seams.size());
  for (std::size_t k = 0; k < seams.size(); ++k) {
    double rsum = 0, maxSurf = 0;
    for (const auto& p : seams[k].points) {
      rsum += std::hypot(p.u1 - 0.5, p.v1 - 0.5);
      maxSurf = std::max(maxSurf, p.onSurfResidual);
    }
    std::printf("  loop %zu: %zu pts, meanR=%.5f, maxOnSurf=%.2e\n", k,
                seams[k].points.size(), rsum / seams[k].points.size(), maxSurf);
  }
  std::printf("closed-form: volA=%.6f volB=%.6f common=%.6f cut=%.6f fuse=%.6f\n",
              ax::volA(), ax::volB(), ax::volCommon(), ax::volCut(),
              ax::volA() + ax::volB() - ax::volCommon());
  if (seams.size() != 2) { std::printf("NOT a 2-seam pose; abort.\n"); return 0; }

  struct { bo::FfOp op; const char* nm; double cf; } ops[] = {
      {bo::FfOp::Common, "COMMON", ax::volCommon()},
      {bo::FfOp::Cut, "CUT", ax::volCut()},
      {bo::FfOp::Fuse, "FUSE", ax::volA() + ax::volB() - ax::volCommon()},
  };
  for (const auto& o : ops) {
    std::printf("\n--- %s (closed-form %.6f) ---\n", o.nm, o.cf);
    for (double d : {0.01, 0.006, 0.005, 0.0025}) {
      bo::MultiSeamCutReport rep;
      const topo::Shape r =
          bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, o.op, d, &rep, o.cf);
      const double relErr = rep.enclosedVolume > 0 ? std::fabs(rep.enclosedVolume - o.cf) / o.cf : -1;
      std::printf("  d=%.5f: decline=%-16s seams=%d subA=%d subB=%d surv=%d wt=%d coh=%d be=%zu vol=%.6f relErr=%.4f null=%d\n",
                  d, bo::multiSeamCutDeclineName(rep.decline), rep.seamLoops, rep.subRegionsA,
                  rep.subRegionsB, rep.survivorFaces, rep.watertight, rep.coherent,
                  rep.boundaryEdges, rep.enclosedVolume, relErr, r.isNull());
      rawSurvivorHisto(A, B, seams, o.op, d);
    }
  }
  return 0;
}

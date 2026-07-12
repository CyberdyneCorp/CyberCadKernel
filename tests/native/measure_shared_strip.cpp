// SPDX-License-Identifier: Apache-2.0
// measure_shared_strip.cpp — MESH-SHARED-STRIP measurement harness.
// Rebuilds the multi-seam survivor solid (mirroring the core's split+select), meshes
// it directly at fine deflections, and reports the RAW edge-use histogram (open ==1,
// nonmanifold >=3) plus the volume, so the fine-d seam residual is measured
// before/after the shared-strip change — even when the verb would decline.
#include "native/boolean/freeform_freeform_cut.h"
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/boolean/freeform_membership.h"
#include "native/boolean/freeform_operand.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_freeform_multiseam_fixture.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace ffm = cybercad::native::boolean::ffmdetail;
namespace ffc = cybercad::native::boolean::ffcdetail;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ffx = freeform_freeform_multiseam_fixture;

static void histo(const tess::Mesh& m, int& open, int& nonmanif) {
  open = 0; nonmanif = 0;
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    if (uses == 1) ++open;
    else if (uses >= 3) ++nonmanif;
  }
}

// Rebuild the survivor faces exactly as freeformFreeformMultiSeamCutWithSeams does, then
// weld them (best repair by boundaryEdges) and mesh — reporting the raw histogram.
static tess::Mesh buildSurvivorMesh(const topo::Shape& A, const topo::Shape& B,
                                    const std::vector<bo::ssi::WLine>& seams, bo::FfOp op,
                                    double d, int& survivors) {
  using namespace ffc;
  using namespace ffm;
  survivors = 0;
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  if (!foA || !foB) return {};
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  if (!freeformWall(*foA, &wallA, wA) || !freeformWall(*foB, &wallB, wB)) return {};

  std::vector<bo::ssi::WLine> seamsB;
  for (const bo::ssi::WLine& s : seams) seamsB.push_back(rekeyToB(s));
  std::vector<WallRegion> regA, regB;
  double gA = 0, gB = 0;
  if (!splitWallBySeams(wallA->face, seams, regA, gA)) return {};
  if (!splitWallBySeams(wallB->face, seamsB, regB, gB)) return {};

  tess::MeshParams mp; mp.deflection = d;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);

  const bo::Membership wantA = op == bo::FfOp::Common ? bo::Membership::In : bo::Membership::Out;
  const bo::Membership wantB =
      (op == bo::FfOp::Common || op == bo::FfOp::Cut) ? bo::Membership::In : bo::Membership::Out;
  std::vector<topo::Shape> faces;
  std::size_t nFromA = 0;
  for (const WallRegion& reg : regA)
    if (subFaceHasMembership(reg.face, meshB, bbB, d, wantA)) { faces.push_back(reg.face); ++nFromA; }
  if (op == bo::FfOp::Cut || op == bo::FfOp::Fuse)
    collectAnalyticByMembership(*foA, meshB, bbB, d, bo::Membership::Out, faces), nFromA = faces.size();
  for (const WallRegion& reg : regB)
    if (subFaceHasMembership(reg.face, meshA, bbA, d, wantB)) faces.push_back(reg.face);
  if (op == bo::FfOp::Fuse)
    collectAnalyticByMembership(*foB, meshA, bbA, d, bo::Membership::Out, faces);

  survivors = static_cast<int>(faces.size());
  if (faces.size() < 2) return {};

  // Try the same repairs weldMultiCoherent tries; keep the mesh with the fewest bad edges.
  std::vector<std::vector<std::size_t>> repairs;
  repairs.push_back({});
  repairs.push_back({faces.size() - 1});
  if (nFromA < faces.size()) { std::vector<std::size_t> b; for (std::size_t i = nFromA; i < faces.size(); ++i) b.push_back(i); repairs.push_back(std::move(b)); }
  if (nFromA > 0 && nFromA < faces.size()) { std::vector<std::size_t> a; for (std::size_t i = 0; i < nFromA; ++i) a.push_back(i); repairs.push_back(std::move(a)); }

  tess::Mesh best; int bestBad = std::numeric_limits<int>::max();
  for (const auto& flip : repairs) {
    std::vector<topo::Shape> f = faces;
    for (std::size_t i : flip) f[i] = f[i].reversedShape();
    const topo::Shape shell = topo::ShapeBuilder::makeShell(f);
    const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
    tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
    int o = 0, nm = 0; histo(m, o, nm);
    if (o + nm < bestBad) { bestBad = o + nm; best = m; }
  }
  return best;
}

int main() {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  std::printf("seams=%zu volA=%.6f volCommon=%.6f\n", seams.size(), ffx::volA(), ffx::volCommon());

  struct OpCase { bo::FfOp op; const char* name; double cf; };
  const std::vector<OpCase> ops = {
      {bo::FfOp::Common, "COMMON", ffx::volCommon()},
      {bo::FfOp::Cut, "CUT", ffx::volCut()},
      {bo::FfOp::Fuse, "FUSE", ffx::volA() + ffx::volB() - ffx::volCommon()},
  };
  const std::vector<double> defls = {0.01, 0.005, 0.0025, 0.002, 0.00125, 0.001};

  for (const OpCase& oc : ops) {
    std::printf("\n== %s (closed-form V=%.6f) ==\n", oc.name, oc.cf);
    for (double d : defls) {
      int surv = 0;
      const tess::Mesh m = buildSurvivorMesh(A, B, seams, oc.op, d, surv);
      int open = 0, nm = 0; histo(m, open, nm);
      const double v = std::fabs(tess::enclosedVolume(m));
      const double err = oc.cf > 0 && v > 0 ? std::fabs(v - oc.cf) / oc.cf : 0.0;
      const bool wt = tess::isWatertight(m);
      // OPEN (edges used 1×) and NONMANIF (edges used ≥3×) are the LOAD-BEARING signal:
      // wt ⇔ (OPEN==0 && NONMANIF==0). V/relErr are informational only — this harness picks
      // the fewest-bad-edges weld repair, which is NOT orientation-coherence-checked, so V may
      // read half/wrong for a non-watertight case; use the verb itself for the coherent volume.
      std::printf("  d=%-8.5g surv=%d OPEN=%-5d NONMANIF=%-4d wt=%d V=%.6f relErr=%.4f\n",
                  d, surv, open, nm, (int)wt, v, err);
      std::fflush(stdout);
    }
  }
  return 0;
}

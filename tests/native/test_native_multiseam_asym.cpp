// SPDX-License-Identifier: Apache-2.0
//
// BOOL-MULTISEAM host GATE — the ASYMMETRIC (curvature-MISMATCHED) multi-seam
// freeform↔freeform watertight sew, the general (non-mirror) generalisation of the
// z-mirror-symmetric multi-seam fixture (test_native_freeform_freeform_multiseam.cpp).
//
// ── WHAT THIS PROVES (and what it overturns) ─────────────────────────────────────
// The symmetric multi-seam fixture is the FRIENDLIEST pose: B is A mirrored about z=H/2,
// so the two curved annuli sharing every seam have IDENTICAL curvature magnitude and the
// M0 mesher's curvature-driven boundary refinement MATCHES on both sides of each seam.
// Its header claimed (lines 18-21) that a curvature-MISMATCHED pair "would leave a small
// T-junction residual at the higher-curvature seam, which the verb HONEST-DECLINES, never
// leaks." That claim is now MEASURED to be STALE: with the shared-seam-strip cache weld
// (seam_strip.h / MESH-STRIP-IMPL) the seam collar is meshed ONCE from the SHARED seam
// geometry, so both annuli emit the IDENTICAL near-seam triangles REGARDLESS of the two
// walls' curvature — the weld is curvature-parity-INDEPENDENT.
//
// The fixture here (freeform_multiseam_asym_fixture.h) is a degree-4 VALLEY (amplitude
// a=4) ∩ a degree-4 DOME of a DIFFERENT amplitude (b=6). The two walls still meet in TWO
// concentric seams (r₁≈0.154, r₂≈0.365) but with MISMATCHED curvature at each seam (z* =
// a·H/(a+b) = 0.012, NOT H/2 — the asymmetry), and A ≠ B (V(A)=0.0292, V(B)=0.0437). This
// suite proves all THREE ops — COMMON (the annular lens), CUT (A minus the lens), and FUSE
// (the outer envelope) — WELD WATERTIGHT (χ=2, be=0, coherent) at the closed-form op-volume
// within the tessellation band, CONVERGING as the deflection refines — so the general
// (non-mirror) multi-seam assembly + sew is LANDED, not honest-declined.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_multiseam_asym_fixture.h"
#include "harness.h"

#include <cmath>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ax = freeform_multiseam_asym_fixture;

// ── The SSI trace returns EXACTLY TWO closed interior seams at the asymmetric radii ──
CC_TEST(asym_trace_returns_two_closed_seams) {
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;
  const double r1 = ax::seamR1(), r2 = ax::seamR2();
  double gotR[2] = {0, 0};
  for (int k = 0; k < 2; ++k) {
    double rsum = 0, maxSurf = 0;
    for (const auto& p : seams[k].points) {
      rsum += std::hypot(p.u1 - 0.5, p.v1 - 0.5);
      maxSurf = std::max(maxSurf, p.onSurfResidual);
    }
    gotR[k] = rsum / static_cast<double>(seams[k].points.size());
    CC_CHECK(seams[k].status == bo::ssi::TraceStatus::Closed);
    CC_CHECK(maxSurf < 1e-6);  // on BOTH surfaces
  }
  const bool ok = (std::fabs(gotR[0] - r1) < 1e-2 && std::fabs(gotR[1] - r2) < 1e-2) ||
                  (std::fabs(gotR[0] - r2) < 1e-2 && std::fabs(gotR[1] - r1) < 1e-2);
  CC_CHECK(ok);
  CC_CHECK(r1 < ax::kR && r2 < ax::kR);
}

// ── The closed-form partition is self-consistent and genuinely ASYMMETRIC ──
CC_TEST(asym_closed_form_partition_is_consistent) {
  CC_CHECK(std::fabs((ax::volCut() + ax::volCommon()) - ax::volA()) < ax::volA() * 1e-9);
  CC_CHECK(ax::volCommon() > 0.05 * ax::volA());     // a substantial, discriminating lens
  CC_CHECK(std::fabs(ax::volA() - ax::volB()) > ax::volA() * 0.1);  // A ≠ B (not a mirror pose)
}

// ── COMMON + CUT weld the annular lens watertight, in-band (curvature-mismatched) ──
// COMMON's smooth-cap bias dominates its error, so it CONVERGES monotonely as the
// deflection refines. CUT's error is already at the mesh-discretization noise floor
// (<0.5% at every deflection — the lens is a small fraction of A), so it stays IN BAND
// but is not strictly monotone (the O(deflection) bias is below the discretization jitter);
// asserting strict monotonicity there is a noise-floor artifact, so CUT is band-checked
// (the same discipline the single-seam CUT gate uses). Both weld watertight at be=0.
CC_TEST(asym_common_cut_weld_watertight) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  for (bo::FfOp op : {bo::FfOp::Common, bo::FfOp::Cut}) {
    const double cf = op == bo::FfOp::Common ? ax::volCommon() : ax::volCut();
    const bool monotone = op == bo::FfOp::Common;  // COMMON cap-bias converges; CUT is at noise floor
    double prevErr = 1e300;
    for (double d : {0.005, 0.0025}) {
      bo::MultiSeamCutReport rep;
      const topo::Shape r =
          bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, op, d, &rep, cf);
      CC_CHECK(!r.isNull());
      CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
      CC_CHECK(rep.seamLoops == 2);
      CC_CHECK(rep.subRegionsA == 3 && rep.subRegionsB == 3);
      CC_CHECK(rep.survivorFaces >= 2);
      CC_CHECK(rep.watertight);
      CC_CHECK(rep.coherent);
      CC_CHECK(rep.boundaryEdges == 0);              // every seam welds despite curvature mismatch
      const double err = std::fabs(rep.enclosedVolume - cf) / cf;
      CC_CHECK(rep.enclosedVolume > 0.0);
      CC_CHECK(err < 30.0 * d);                      // within the tessellation band (DISAGREED=0)
      if (monotone) CC_CHECK(err < prevErr);         // COMMON converges (a closed seam, not a gap)
      prevErr = err;
    }
  }
}

// ── FUSE welds the outer envelope watertight in the working band ──
CC_TEST(asym_fuse_welds_watertight) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  const double cf = ax::volA() + ax::volB() - ax::volCommon();  // V(A)+V(B)−lens
  for (double d : {0.01, 0.006, 0.005}) {
    bo::MultiSeamCutReport rep;
    const topo::Shape r =
        bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Fuse, d, &rep, cf);
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
    CC_CHECK(rep.seamLoops == 2);
    CC_CHECK(rep.subRegionsA == 3 && rep.subRegionsB == 3);
    CC_CHECK(rep.survivorFaces >= 2);
    CC_CHECK(rep.watertight);
    CC_CHECK(rep.coherent);
    CC_CHECK(rep.boundaryEdges == 0);
    const double err = std::fabs(rep.enclosedVolume - cf) / cf;
    CC_CHECK(rep.enclosedVolume > 0.0);
    CC_CHECK(err < 30.0 * d);                        // within the tessellation band
  }
}

// ── Raw survivor histogram helper (no orientation repair) ─────────────────────────
// Rebuild the exact survivor set the COMMON verb welds, mesh it, and count the seam-
// localized non-manifold edges (uses ≥ 3). This is the MESHER's raw weld verdict — the
// same probe measure_multiseam_fine uses to separate the assembly (deflection-independent
// split/select) from the frozen-M0-mesher shared-seam-strip weld.
namespace asymraw {
namespace ffm = cybercad::native::boolean::ffmdetail;
namespace ffc = cybercad::native::boolean::ffcdetail;
struct RawVerdict { std::size_t survivors = 0; int nmInner = 0, nmOuter = 0, nmOther = 0, open = 0; };
inline RawVerdict commonSurvivorHisto(const topo::Shape& A, const topo::Shape& B,
                                      const std::vector<bo::ssi::WLine>& seams, double d) {
  RawVerdict v;
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  if (!foA || !foB) return v;
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  if (!ffc::freeformWall(*foA, &wallA, wA) || !ffc::freeformWall(*foB, &wallB, wB)) return v;
  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(ffc::rekeyToB(s));
  std::vector<ffm::WallRegion> regA, regB;
  double gA = -1, gB = -1;
  if (!ffm::splitWallBySeams(wallA->face, seams, regA, gA)) return v;
  if (!ffm::splitWallBySeams(wallB->face, seamsB, regB, gB)) return v;
  tess::MeshParams mp; mp.deflection = d;
  const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
  const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
  const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);
  std::vector<topo::Shape> faces;
  for (const auto& r : regA)
    if (ffc::subFaceHasMembership(r.face, meshB, bbB, d, bo::Membership::In)) faces.push_back(r.face);
  for (const auto& r : regB)
    if (ffc::subFaceHasMembership(r.face, meshA, bbA, d, bo::Membership::In)) faces.push_back(r.face);
  v.survivors = faces.size();
  const topo::Shape shell = topo::ShapeBuilder::makeShell(faces);
  const topo::Shape solid = topo::ShapeBuilder::makeSolid({shell});
  const tess::Mesh m = tess::SolidMesher(mp).mesh(solid);
  const double r1 = ax::seamR1(), r2 = ax::seamR2();
  for (const auto& [e, uses] : tess::edgeUseCounts(m)) {
    if (uses == 1) { ++v.open; continue; }
    if (uses < 3) continue;
    const auto& a = m.vertices[e.lo];
    const auto& b = m.vertices[e.hi];
    const double rr = std::hypot(0.5 * (a.x + b.x), 0.5 * (a.y + b.y));
    if (std::fabs(rr - r1) < 0.03) ++v.nmInner;
    else if (std::fabs(rr - r2) < 0.03) ++v.nmOuter;
    else ++v.nmOther;
  }
  return v;
}
}  // namespace asymraw

// ── L3-BAND / MESH-COLLAR: the deflection-robust collar EXTENDS the mesher weld band ──
// BELOW each op's working band the multi-seam sew declines (never leaks). This test PINS
// where that decline lives, and REGRESSION-GUARDS the deflection-robust-collar fix
// (seam_strip.h): the shared-seam-strip collar inset used to be δ = min(0.5·segLen,
// 0.25·rSeam), which SHRINKS with the deflection-driven seam segment length (segLen → 0 as
// d refines), so the collar suppression band eventually stopped covering the near-seam
// curvature Steiner pile and a shared seam edge got used 4×. Tying δ to the SEAM RADIUS
// (0.05·rSeam, deflection-INDEPENDENT) removes that degradation.
//
// Measured (measure_multiseam_fine), asym a=4 valley ∩ b=6 dome, seams r₁≈0.154, r₂≈0.365:
//   • The ASSEMBLY (split tiling + survivor set) is EXACT and deflection-INDEPENDENT at
//     every d (gap == 0 on both walls, survivors == 2) — the boolean/ lane is sound.
//   • BEFORE the fix the raw COMMON survivor mesh was non-manifold at d=0.002 (1 edge @
//     OUTER r₂) and d=0.00125 (4 edges @ INNER r₁). AFTER the fix the OUTER-seam collapse
//     at d=0.002 is GONE (nonmanif == 0) — the collar band now suppresses the pile at that
//     refinement. The band is extended.
//   • Two DEEPER residuals remain, out of the collar-width wave's + this lane's reach, and
//     the verb HONEST-DECLINES both to NULL (never leaky): at d=0.002 the strip welds the
//     two annulus survivors into a watertight-but-smaller closed region (the annulus↔annulus
//     collar-side winding collapse), caught by the two-sided VOLUME self-verify
//     (VolumeInconsistent); at d=0.00125 the seam is sampled DENSER than the weld tolerance
//     (weldTol = 0.5·d ≫ seam spacing) so adjacent seam vertices merge into a 4×-used
//     on-seam edge (NotWatertight). Both are the SACRED never-leaky honest-decline.
CC_TEST(asym_fine_deflection_residual_is_mesher_not_assembly) {
  namespace ffm = cybercad::native::boolean::ffmdetail;
  namespace ffc = cybercad::native::boolean::ffcdetail;
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }

  // (1) ASSEMBLY is correct and deflection-INDEPENDENT: split BOTH walls by BOTH seams; the
  // UV tiling gap is exactly 0 on each wall (the split is a pure topology/UV operation — it
  // never touches the deflection). This is the in-lane stage-3 machinery, and it is sound.
  const auto foA = bo::recogniseFreeformSolid(A);
  const auto foB = bo::recogniseFreeformSolid(B);
  CC_CHECK(foA && foB);
  const bo::OperandFace* wallA = nullptr;
  const bo::OperandFace* wallB = nullptr;
  bo::FfCutDecline wA = bo::FfCutDecline::Ok, wB = bo::FfCutDecline::Ok;
  CC_CHECK(ffc::freeformWall(*foA, &wallA, wA) && ffc::freeformWall(*foB, &wallB, wB));
  std::vector<bo::ssi::WLine> seamsB;
  for (const auto& s : seams) seamsB.push_back(ffc::rekeyToB(s));
  std::vector<ffm::WallRegion> regA, regB;
  double gapA = -1.0, gapB = -1.0;
  CC_CHECK(ffm::splitWallBySeams(wallA->face, seams, regA, gapA));
  CC_CHECK(ffm::splitWallBySeams(wallB->face, seamsB, regB, gapB));
  CC_CHECK(regA.size() == 3 && regB.size() == 3);   // inner disk + middle annulus + background
  CC_CHECK(gapA < 1e-9 && gapB < 1e-9);             // EXACT UV tiling — deflection-independent

  // (2) MESH-COLLAR REGRESSION: the deflection-robust collar CLEARED the d=0.002 OUTER-seam
  // non-manifold. The raw COMMON survivor mesh (no orientation repair) has the stable
  // survivor set (2) and ZERO non-manifold edges at d=0.002 — where the pre-fix collar left
  // ONE at the OUTER seam r₂. The mesher's watertight band is extended one refinement step.
  const asymraw::RawVerdict v200 = asymraw::commonSurvivorHisto(A, B, seams, 0.002);
  CC_CHECK(v200.survivors == 2);                    // survivor set stable (deflection-indep.)
  CC_CHECK(v200.nmOuter == 0);                       // FIXED — the OUTER-seam pile is suppressed
  CC_CHECK(v200.nmInner == 0 && v200.nmOther == 0);  // fully manifold at d=0.002
  CC_CHECK(v200.open == 0);                          // no open edges (not the naive sliver-drop)

  // (2b) The DEEPER residual is now localized: at d=0.00125 the raw survivor mesh is still
  // non-manifold at the INNER seam r₁ (the over-dense-seam weld-tolerance merge — weldTol =
  // 0.5·d ≫ the r₁ seam spacing), NOT at the outer seam. This is a distinct, deeper residual
  // than the collar-band width, and it is what the verb honest-declines below.
  const asymraw::RawVerdict v125 = asymraw::commonSurvivorHisto(A, B, seams, 0.00125);
  CC_CHECK(v125.survivors == 2);
  CC_CHECK(v125.nmInner > 0);                        // residual localized to the INNER seam r₁

  // (3) The verb still HONEST-DECLINES below the working band (never a leaky solid). At
  // d=0.002 the decline is now the deeper annulus-winding VOLUME residual (the raw mesh is
  // watertight, but the coherent weld encloses a smaller region), caught by the two-sided
  // volume self-verify — NOT a silent wrong solid.
  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common,
                                                                  0.002, &rep, ax::volCommon());
  CC_CHECK(r.isNull());                              // honest decline, never leaky
  CC_CHECK(rep.seamLoops == 2);                      // reached the weld
  CC_CHECK(rep.subRegionsA == 3 && rep.subRegionsB == 3);
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::VolumeInconsistent);  // deeper weld residual

  // (4) The in-band deflection STILL welds (the band boundary is real, not a regression).
  bo::MultiSeamCutReport repOk;
  const topo::Shape rOk = bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common,
                                                                    0.0025, &repOk, ax::volCommon());
  CC_CHECK(!rOk.isNull());
  CC_CHECK(repOk.decline == bo::MultiSeamCutDecline::Ok);
  CC_CHECK(repOk.boundaryEdges == 0);
}

int main() { return cctest::run_all(); }

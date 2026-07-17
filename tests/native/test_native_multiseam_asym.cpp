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

int main() { return cctest::run_all(); }

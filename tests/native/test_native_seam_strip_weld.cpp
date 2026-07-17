// SPDX-License-Identifier: Apache-2.0
//
// MESH-STRIP-IMPL regression — the SHARED-SEAM-STRIP CACHE WELD at FINE deflection.
//
// This suite pins the honest, measured behaviour of the fine-deflection multi-seam
// weld after the shared-seam-strip cache weld (seam_strip.h + face_mesher.h /
// solid_mesher.h integration) landed. It uses the SAME two-coaxial-mirror-cup fixture
// as test_native_freeform_freeform_multiseam.cpp (two degree-4 cups whose curved walls
// meet in TWO closed circular seams r₁≈0.131, r₂≈0.374), the pose whose annulus↔annulus
// survivor set is welded across BOTH shared curved seams.
//
// ── WHAT THE STRIP WELD IS (seam_strip.h) ────────────────────────────────────────
// Two curved annuli sharing a CLOSED seam carried as 2-pole straight chords each mesh
// alone as a clean 2-manifold, but their INDEPENDENT per-face constrained-Delaunay
// near-seam triangulations do not pair 1:1 at the spatial weld — so the shared seam
// edge is used 4× (a NON-MANIFOLD collapse whose residual GROWS with refinement). The
// SolidMesher runs the strip weld ONLY as a FALLBACK (after the baseline + rim-pin
// passes fail to weld): it meshes the seam-adjacent collar strip ONCE from the SHARED
// seam geometry so both faces emit the identical near-seam triangles, suppressing each
// face's independent interior Steiner insertion inside the collar band. The path fires
// ONLY for a genuinely-shared seam loop; every already-watertight mesh is byte-identical.
//
// ── VERIFIED BEHAVIOUR (this suite) ──────────────────────────────────────────────
//   1. FINE-DEFLECTION WATERTIGHT BAND. At d = 0.0045 — a FINE deflection BELOW the
//      0.005 floor the pre-strip suite tested — the FUSE outer envelope (whose survivor
//      set keeps each wall's rim-to-r₂ background annulus sharing the OUTER seam) welds
//      into ONE watertight, coherent, positive-volume closed 2-manifold (every edge used
//      exactly twice, be = 0), at the closed-form V(A)+V(B)−V(A∩B). This is the fine-
//      deflection multi-seam gate.
//   2. THE STRIP WELD IS NEVER-LEAKY. Below the working band the weld can still collapse
//      (the strip converts the 4×-used non-manifold into a small OPEN-edge residual — the
//      over-dense-seam weld-tolerance sliver mapped in SSI-ROADMAP MESH-STRIP-IMPL). The
//      mandatory self-verify then HONEST-DECLINES to a NULL Shape with a measured residual
//      — NEVER a silently non-watertight solid. Asserted at COMMON d = 0.0018.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/tessellate/mesh.h"

#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <cmath>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace topo = cybercad::native::topology;
namespace ffx = freeform_freeform_multiseam_fixture;

// ── (1) FINE-DEFLECTION multi-seam FUSE welds into ONE watertight solid ──
// d = 0.0045 is below the 0.005 floor the pre-strip freeform_freeform_multiseam suite
// exercised; the FUSE envelope shares the OUTER curved seam r₂ across two background
// annuli whose independent near-seam CDTs are the classic collapse trigger. It welds
// watertight, coherent, positive-volume, at the closed-form op-volume.
CC_TEST(seamstrip_fine_deflection_fuse_welds_watertight) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;
  const double cf = ffx::volA() + ffx::volB() - ffx::volCommon();  // V(A)+V(B)−lens
  const double d = 0.0045;                                         // FINE — below the old 0.005 floor
  bo::MultiSeamCutReport rep;
  const topo::Shape r =
      bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Fuse, d, &rep, cf);
  // Welds a coherent watertight solid — never NULL, never leaky, at the fine deflection.
  CC_CHECK(!r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
  CC_CHECK(rep.seamLoops == 2);
  CC_CHECK(rep.subRegionsA == 3 && rep.subRegionsB == 3);
  CC_CHECK(rep.survivorFaces >= 2);
  // The watertight invariant: closed 2-manifold (every edge used exactly twice → be = 0),
  // consistently oriented, positive enclosed volume at the closed-form op-volume.
  CC_CHECK(rep.watertight);
  CC_CHECK(rep.coherent);
  CC_CHECK(rep.boundaryEdges == 0);
  CC_CHECK(rep.enclosedVolume > 0.0);
  CC_CHECK(std::fabs(rep.enclosedVolume - cf) / cf < 30.0 * d);
}

// ── (2) Below the working band the weld HONEST-DECLINES — never a silent leak ──
// At COMMON d = 0.0018 the shared inner seam r₁ is sampled so densely that adjacent
// seam vertices merge under the deflection-derived weld tolerance, collapsing a strip
// sliver: the strip weld eliminates the 4×-used NON-MANIFOLD but leaves a tiny OPEN-edge
// residual (the over-dense-seam collapse mapped in SSI-ROADMAP MESH-STRIP-IMPL). The
// mandatory self-verify must then return a NULL Shape (→ OCCT fallback in the facade),
// NEVER a silently non-watertight solid. This is the hard "never emit a non-watertight
// mesh silently" invariant.
CC_TEST(seamstrip_subthreshold_common_declines_never_leaks) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  const double d = 0.0018;  // below the COMMON working band → the collapse residual
  bo::MultiSeamCutReport rep;
  const topo::Shape r =
      bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common, d, &rep, ffx::volCommon());
  // The verb declines to NULL with a measured residual — the honest, never-leaky outcome.
  CC_CHECK(r.isNull());
  CC_CHECK(!rep.watertight);
  // The machinery still reached the weld (both seams traced, both walls split), so the
  // decline is a measured watertight-verify failure, not an upstream give-up.
  CC_CHECK(rep.seamLoops == 2);
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::NotWatertight);
}

int main() { return cctest::run_all(); }

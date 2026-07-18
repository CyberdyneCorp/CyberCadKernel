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
//   2. THE STRIP WELD IS NEVER-LEAKY. Below the working band the weld self-verify
//      HONEST-DECLINES to a NULL Shape with a measured residual — NEVER a silently wrong
//      solid. Asserted at COMMON d = 0.0018. The DECLINE MECHANISM there has moved as the
//      mesher weld band extended: the over-dense-seam weld-tolerance sliver this case
//      originally pinned (NotWatertight — adjacent seam vertices merging under the
//      deflection-derived weld tolerance) is CLEARED by the weld-resolution seam-ring
//      decimation (MESH-WELD-TOL, seam_strip.h: each registered seam ring keeps only
//      samples ≥ 2·weldTol apart, so no two strip vertices can merge at the weld). The
//      strip now welds WATERTIGHT at d = 0.0018 (be = 0), and the decline is the deeper
//      annulus↔annulus collar-side WINDING collapse (`boolean/weldMultiCoherent` picks the
//      first watertight+coherent config, which encloses a smaller region), caught by the
//      two-sided VOLUME self-verify — VolumeInconsistent, still never-leaky.
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
// At COMMON d = 0.0018 the verb must return a NULL Shape with a measured residual (→ OCCT
// fallback in the facade), NEVER a silently wrong solid. The MECHANISM this case pins was
// UPDATED by MESH-WELD-TOL (seam_strip.h): the shared inner seam r₁ used to be sampled so
// densely that adjacent seam/collar ring vertices merged under the deflection-derived weld
// tolerance and the strip could not weld (NotWatertight). The weld-resolution seam-ring
// decimation cleared that — the strip now welds WATERTIGHT (be = 0, coherent) at d = 0.0018
// — and the remaining sub-band residual is the DEEPER annulus↔annulus collar-side winding
// collapse (`boolean/weldMultiCoherent` takes the first watertight+coherent config, which
// encloses a SMALLER region than the closed-form lens), caught by the two-sided VOLUME
// self-verify. Same honest-decline invariant, one layer deeper (measured: vol≈0.00702 vs
// closed-form 0.00770 at d=0.0018).
CC_TEST(seamstrip_subthreshold_common_declines_never_leaks) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  const double d = 0.0018;  // below the COMMON working band → the winding residual
  bo::MultiSeamCutReport rep;
  const topo::Shape r =
      bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common, d, &rep, ffx::volCommon());
  // The verb declines to NULL with a measured residual — the honest, never-leaky outcome.
  CC_CHECK(r.isNull());
  // MESH-WELD-TOL regression: the mesher weld itself now succeeds at this deflection —
  // the strip mesh is watertight and coherent (the weld-tolerance merge is CLEARED)...
  CC_CHECK(rep.watertight);
  CC_CHECK(rep.coherent);
  CC_CHECK(rep.boundaryEdges == 0);
  // The machinery still reached the weld (both seams traced, both walls split), so the
  // decline is a measured self-verify failure, not an upstream give-up.
  CC_CHECK(rep.seamLoops == 2);
  // ...and the decline is the deeper winding VOLUME residual (boolean/ lane), not the
  // cleared weld-tolerance NotWatertight.
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::VolumeInconsistent);
}

int main() { return cctest::run_all(); }

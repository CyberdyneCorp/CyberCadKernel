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
//   2. THE FINE-DEFLECTION COMMON WELDS AT THE CORRECT VOLUME. This case has tracked the
//      mesher weld band down as each layer sharpened. The over-dense-seam weld-tolerance
//      sliver it originally pinned (NotWatertight — adjacent seam vertices merging under the
//      deflection-derived weld tolerance) was CLEARED by the weld-resolution seam-ring
//      decimation (MESH-WELD-TOL, seam_strip.h: each registered seam ring keeps only samples
//      ≥ 2·weldTol apart, so no two strip vertices can merge at the weld) — the strip then
//      welded WATERTIGHT at d = 0.0018 but PINCHED: both annuli spliced the SAME shared
//      planar collar strip, the coincident duplicate triangles annihilated at the weld, and
//      the collar bands' volume was LOST (measured all coherent configs enclosed 0.007019 vs
//      the survivors' own divergence expectation 0.007448, −5.8%), so BOOL-VOTE HONEST-
//      DECLINED it VolumeInconsistent. MESH-COLLAR-WEDGE (seam_strip.h / face_mesher.h) now
//      removes the pinch: the per-face wedge-carrying collar places each face's near-seam
//      ring on its OWN surface at the band edge (r_seam ± δ) — the two faces share ONLY the
//      seam ring, so the collar wedge volume is CARRIED through the weld. Asserted at COMMON
//      d = 0.0018: the weld now closes watertight+coherent at enclosedVolume ≈ 0.007496,
//      MATCHING the expectation ≈ 0.007448 (inside the ±2% oracle-free gate) and the closed-
//      form lens within the tessellation band — with AND without the closed form. The never-
//      leaky property (a volume-moving weld → VolumeInconsistent) still guards below the new
//      floor; it is pinned in test_native_multiseam_vote.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/tessellate/mesh.h"

#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <cmath>
#include <limits>
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

// ── (2) The fine-deflection COMMON WELDS at the correct volume (collar-wedge fix) ──
// At COMMON d = 0.0018 the verb now returns a watertight, coherent solid at the CORRECT
// enclosed volume. The MECHANISM this case pins was UPDATED as the layers beneath it
// sharpened, and the residual is now CLEARED:
//   * MESH-WELD-TOL (seam_strip.h): the shared inner seam r₁ used to be sampled so densely
//     that adjacent seam/collar ring vertices merged under the deflection-derived weld
//     tolerance and the strip could not weld (NotWatertight). The weld-resolution seam-ring
//     decimation cleared that — the strip welded WATERTIGHT (be = 0, coherent) but PINCHED.
//   * BOOL-VOTE (boolean/weldMultiCoherent): the pinched weld enclosed 0.007019 vs the
//     survivors' own divergence expectation 0.007448 (−5.8%), so the oracle-free gate
//     HONEST-DECLINED it VolumeInconsistent — never a silent leak.
//   * MESH-COLLAR-WEDGE (seam_strip.h / face_mesher.h): the per-face wedge-carrying collar
//     places each face's near-seam ring on its OWN surface at the band edge (r_seam ± δ) —
//     the two faces share ONLY the seam ring, so the collar wedge volume is CARRIED through
//     the weld instead of annihilating. The pinch is GONE: measured the weld now encloses
//     ≈ 0.007496, MATCHING the expectation ≈ 0.007448 (inside the ±2% band) and the closed-
//     form lens within the tessellation band. The oracle-free gate ACCEPTS it — with AND
//     without the closed form. The never-leaky property (a volume-moving weld →
//     VolumeInconsistent) still guards below the new floor, pinned in multiseam_vote.
CC_TEST(seamstrip_subthreshold_common_welds_at_correct_volume) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }
  const double d = 0.0018;  // was below the COMMON band; the collar-wedge fix now welds it
  // The weld (and the oracle-free gate's acceptance) must NOT depend on the closed-form
  // oracle — assert both call forms.
  for (const double cf : {ffx::volCommon(), std::numeric_limits<double>::quiet_NaN()}) {
    bo::MultiSeamCutReport rep;
    const topo::Shape r =
        bo::freeformFreeformMultiSeamCutWithSeams(A, B, seams, bo::FfOp::Common, d, &rep, cf);
    // The verb welds a coherent watertight solid — no longer declines, never leaky.
    CC_CHECK(!r.isNull());
    CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
    CC_CHECK(rep.watertight);
    CC_CHECK(rep.coherent);
    CC_CHECK(rep.boundaryEdges == 0);
    // The machinery reached the weld (both seams traced, both walls split).
    CC_CHECK(rep.seamLoops == 2);
    // The oracle-free gate accepted it BECAUSE the enclosed volume AGREES with the
    // survivors' own divergence expectation (the pinch is gone — no material moved).
    CC_CHECK(rep.enclosedVolume > 0.0);
    CC_CHECK(rep.expectedVolume > 0.0);
    CC_CHECK(std::fabs(rep.enclosedVolume - rep.expectedVolume) <=
             bo::ffmdetail::kWeldVolumeAgreeFrac * rep.expectedVolume);
    // And the accepted volume is the true closed-form lens within the tessellation band.
    CC_CHECK(std::fabs(rep.enclosedVolume - ffx::volCommon()) < 30.0 * d * ffx::volCommon());
  }
}

int main() { return cctest::run_all(); }

// SPDX-License-Identifier: Apache-2.0
//
// BOOL-VOTE host GATE — the volume-consistent weld-configuration selection of the
// multi-seam freeform↔freeform sew (`weldMultiCoherent`, freeform_freeform_multiseam.h).
//
// ── THE MEASURED HAZARD THIS GUARDS (measure_multiseam_vote, asym fixture) ─────────
// At d=0.002 on the asymmetric annulus↔annulus COMMON (a=4 valley ∩ b=6 dome) the
// strip-pass WELDED survivor mesh is watertight (be=0, nonmanif=0) but its enclosed
// volume is 0.006172 — NOT the closed-form 0.006883 and NOT the survivors' own
// divergence-theorem expectation 0.006619. Mechanism (sharpening the MESH-COLLAR
// narrative): the mesher's shared seam-strip fallback fires; BOTH annuli's per-loop
// collar-side votes are CORRECT and therefore IDENTICAL (material on the SAME radial
// side at each seam), so the two faces splice the SAME strip triangles; the weld's
// coincident-duplicate-triangle drop annihilates BOTH copies and the mesh pinches shut
// at the collar rings — the collar bands' volume (a fixed fraction of the seam radii,
// deflection-INDEPENDENT) is silently LOST. Every watertight+coherent orientation
// configuration shares this one geometry (measured: all coherent configs enclose
// 0.006172; sym fixture at d=0.0018 likewise, all 0.007019 vs expectation 0.007448), so
// no orientation pick can restore the volume — but before BOOL-VOTE the verb, called
// WITHOUT the closed-form oracle, RETURNED this wrong-volume solid as Ok (the one-sided
// op bounds cannot see a too-small COMMON): a silent-wrong, the DISAGREED=0 violation.
//
// ── THE FIX UNDER TEST ──────────────────────────────────────────────────────────────
// `weldMultiCoherent` accepts a weld configuration ONLY if its enclosed volume agrees
// (±2%, measured calibration in the header) with the ORACLE-FREE divergence-theorem
// expectation of that configuration — the survivor faces' summed signed per-face volumes
// through the same shared-EdgeCache baseline meshing. A weld may only REPAIR the seam
// pairing (measured legit drift ≤ 4e-5 relative), never MOVE material (the pinch drifts
// 6.8%). No consistent configuration ⇒ VolumeInconsistent decline with both volumes as
// witnesses — an honest decline, never a wrong solid; no tolerance was widened.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/tessellate/mesh.h"
#include "native/tessellate/solid_mesher.h"

#include "freeform_multiseam_asym_fixture.h"
#include "harness.h"

#include <cmath>
#include <limits>
#include <vector>

namespace bo = cybercad::native::boolean;
namespace tess = cybercad::native::tessellate;
namespace topo = cybercad::native::topology;
namespace ax = freeform_multiseam_asym_fixture;

// ── THE regression: d=0.002 COMMON WITHOUT the closed-form oracle must DECLINE ──────
// Before BOOL-VOTE this returned a non-null solid with enclosedVolume ≈ 0.006172 (10.3%
// below the true lens) and decline == Ok — the silent-wrong. The oracle-free divergence
// self-verify must catch it: decline VolumeInconsistent, NULL shape, and the report must
// carry both witnesses (the pinched welded volume and the larger expectation it missed).
CC_TEST(vote_d002_common_without_oracle_declines_volume_inconsistent) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;

  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, 0.002, &rep,
      std::numeric_limits<double>::quiet_NaN());  // NO oracle — the verb is on its own
  CC_CHECK(r.isNull());                                                  // never a wrong solid
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::VolumeInconsistent);  // the honest reason
  // The witnesses: a watertight + coherent weld existed (the sharpened decline map reads
  // "the weld closed, but moved material" — not "the weld failed to close")...
  CC_CHECK(rep.watertight);
  CC_CHECK(rep.coherent);
  CC_CHECK(rep.boundaryEdges == 0);
  // ...but its volume fell SHORT of the survivors' own divergence expectation by far more
  // than a repair could account for.
  CC_CHECK(rep.enclosedVolume > 0.0);
  CC_CHECK(rep.expectedVolume > rep.enclosedVolume);
  CC_CHECK(std::fabs(rep.enclosedVolume - rep.expectedVolume) >
           bo::ffmdetail::kWeldVolumeAgreeFrac * rep.expectedVolume);
  // The expectation itself is honest: within the verb's own convergence band of the
  // closed-form lens (which the verb never saw) — the oracle-free gate is measuring the
  // right quantity, not a second wrong one.
  CC_CHECK(std::fabs(rep.expectedVolume - ax::volCommon()) < 0.1 * ax::volCommon());
}

// ── The gate must NOT over-fire: the in-band weld still welds, with agreeing volumes ──
// d=0.0025 COMMON welds via the rim-pin repair (no strip pinch); the welded volume and
// the divergence expectation agree to ≤4e-5 relative (measured) — far inside the 2% band.
// Checked WITHOUT the oracle too: the pure oracle-free path returns the solid.
CC_TEST(vote_inband_common_still_welds_and_volumes_agree) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;

  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, 0.0025, &rep,
      std::numeric_limits<double>::quiet_NaN());  // no oracle — oracle-free path welds
  CC_CHECK(!r.isNull());
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
  CC_CHECK(rep.watertight && rep.coherent);
  CC_CHECK(rep.boundaryEdges == 0);
  CC_CHECK(rep.expectedVolume > 0.0);
  CC_CHECK(std::fabs(rep.enclosedVolume - rep.expectedVolume) <=
           bo::ffmdetail::kWeldVolumeAgreeFrac * rep.expectedVolume);
  // And the result is the true lens within the tessellation band.
  CC_CHECK(std::fabs(rep.enclosedVolume - ax::volCommon()) < 0.1 * ax::volCommon());
}

// ── The divergence expectation is a genuine, self-standing witness ──────────────────
// The survivors' summed signed per-face volumes (the exact quantity the gate uses) must
// converge on the closed-form lens as the deflection refines — including at d=0.002,
// where the welded mesh does NOT. This pins the gate's reference to the correct side of
// the disagreement (the assembly is right; the strip-pinched weld is wrong).
CC_TEST(vote_divergence_expectation_tracks_closed_form) {
  namespace ffm = cybercad::native::boolean::ffmdetail;
  namespace ffc = cybercad::native::boolean::ffcdetail;
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;

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
  double gA = -1.0, gB = -1.0;
  CC_CHECK(ffm::splitWallBySeams(wallA->face, seams, regA, gA));
  CC_CHECK(ffm::splitWallBySeams(wallB->face, seamsB, regB, gB));

  for (double d : {0.0025, 0.002}) {
    tess::MeshParams mp;
    mp.deflection = d;
    const tess::Mesh meshA = tess::SolidMesher(mp).mesh(foA->solid);
    const tess::Mesh meshB = tess::SolidMesher(mp).mesh(foB->solid);
    const bo::Aabb bbA = bo::meshAabb(meshA), bbB = bo::meshAabb(meshB);
    std::vector<topo::Shape> faces;
    for (const auto& r : regA)
      if (ffc::subFaceHasMembership(r.face, meshB, bbB, d, bo::Membership::In))
        faces.push_back(r.face);
    std::size_t nFromA = faces.size();
    for (const auto& r : regB)
      if (ffc::subFaceHasMembership(r.face, meshA, bbA, d, bo::Membership::In))
        faces.push_back(r.face);
    CC_CHECK(faces.size() == 2 && nFromA == 1);  // the two annulus survivors

    const std::vector<double> vols = ffm::survivorSignedVolumes(faces, mp);
    CC_CHECK(vols.size() == 2);
    // The COMMON lens = |vA_contribution − vB_contribution| for the coherent pairing
    // (the two annuli wind oppositely as meshed, so ONE of the two pairings is the lens).
    const double lensSame = std::fabs(vols[0] + vols[1]);
    const double lensOpp = std::fabs(vols[0] - vols[1]);
    const double lens = std::max(lensSame, lensOpp);
    CC_CHECK(std::fabs(lens - ax::volCommon()) <
             std::min(0.5, 30.0 * d) * ax::volCommon());  // the verb's own convergence band
  }
}

int main() { return cctest::run_all(); }

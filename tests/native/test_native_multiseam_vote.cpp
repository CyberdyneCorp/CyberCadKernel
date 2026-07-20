// SPDX-License-Identifier: Apache-2.0
//
// BOOL-VOTE host GATE — the volume-consistent weld-configuration selection of the
// multi-seam freeform↔freeform sew (`weldMultiCoherent`, freeform_freeform_multiseam.h).
//
// ── THE MEASURED PROPERTY THIS GUARDS (measure_multiseam_vote, asym fixture) ────────
// The oracle-free divergence-volume gate `weldMultiCoherent` must ACCEPT a weld ONLY when
// its enclosed volume agrees (±2%) with the survivors' own divergence-theorem expectation —
// so it can never ship a material-moving weld, and must not over-fire on a correct one.
//
// History (why the gate exists): the pre-collar shared seam-strip fired as a fallback and
// pinched the annulus↔annulus COMMON shut — BOTH annuli spliced the SAME planar collar
// strip, the weld's coincident-duplicate-triangle drop annihilated both copies, and the
// collar bands' volume was LOST (asym d=0.002: welded 0.006172 vs expectation 0.006619,
// −6.8%; sym d=0.0018: 0.007019 vs 0.007448, −5.8%). Called WITHOUT the closed-form oracle
// the verb RETURNED that pinched solid as Ok (the one-sided op bound cannot see a too-small
// COMMON) — a silent-wrong. BOOL-VOTE closed that by making the gate DECLINE it oracle-free.
//
// ── WHAT CHANGED (MESH-COLLAR-WEDGE) ─────────────────────────────────────────────────
// The pinch itself is now FIXED in the mesher (seam_strip.h / face_mesher.h): the per-face
// wedge-carrying collar places each face's near-seam ring on its OWN surface at the band
// edge (r_seam ± δ) instead of a shared planar radial offset, so the two faces share only
// the seam ring and the collar wedge volume is CARRIED through the weld. The band therefore
// WELDS CORRECTLY down through d=0.00125: asym d=0.002 now encloses ≈ 0.006679 (matching the
// expectation 0.006619, inside the ±2% band), with AND without the oracle. The gate's role
// flips from "catch the pinch" to "prove the now-correct weld is genuinely volume-consistent
// oracle-free" — it still rejects any residual material-moving weld below the new floor.
//
// ── THE GATE UNDER TEST ──────────────────────────────────────────────────────────────
// `weldMultiCoherent` accepts a weld configuration ONLY if its enclosed volume agrees
// (±2%, measured calibration in the header) with the ORACLE-FREE divergence-theorem
// expectation of that configuration — the survivor faces' summed signed per-face volumes
// through the same shared-EdgeCache baseline meshing. A weld may only REPAIR the seam
// pairing (measured legit drift ≤ 2e-4 relative), never MOVE material. No consistent
// configuration ⇒ VolumeInconsistent decline with both volumes as witnesses — an honest
// decline, never a wrong solid; no tolerance was widened.
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

// ── THE regression: d=0.002 COMMON WITHOUT the closed-form oracle now WELDS CORRECTLY ──
// History of this case: the pre-collar strip pinched the annulus↔annulus COMMON shut and,
// called WITHOUT the closed-form oracle, the verb RETURNED that pinched solid as `Ok`
// (enclosedVolume ≈ 0.006172, 10.3% below the true lens) — a shipped silent-wrong. BOOL-VOTE
// made the verb DECLINE it VolumeInconsistent oracle-free. MESH-COLLAR-WEDGE then removed the
// pinch itself: the per-face wedge-carrying collar (seam_strip.h / face_mesher.h) places each
// face's near-seam ring on its OWN surface at the band edge, so the two faces share only the
// seam ring and the collar wedge volume is CARRIED through the weld instead of annihilating.
// Measured (measure_multiseam_vote, asym d=0.002): the weld now closes watertight+coherent at
// enclosedVolume ≈ 0.006679, MATCHING the survivors' own divergence expectation ≈ 0.006619
// (well inside the ±2% gate) and the closed-form lens within the tessellation band — with AND
// without the oracle. The DISAGREED=0 guard this pins: the ORACLE-FREE gate must ACCEPT the
// now-correct weld (not over-fire), while still rejecting any volume that moves material.
CC_TEST(vote_d002_common_without_oracle_welds_at_correct_volume) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;

  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, 0.002, &rep,
      std::numeric_limits<double>::quiet_NaN());  // NO oracle — the verb is on its own
  CC_CHECK(!r.isNull());                                     // welds — no longer declines
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::Ok);
  // The weld closes as a coherent watertight 2-manifold...
  CC_CHECK(rep.watertight);
  CC_CHECK(rep.coherent);
  CC_CHECK(rep.boundaryEdges == 0);
  // ...and the ORACLE-FREE gate ACCEPTED it because the enclosed volume AGREES with the
  // survivors' own divergence expectation (the pinch is gone — no material was moved).
  CC_CHECK(rep.enclosedVolume > 0.0);
  CC_CHECK(rep.expectedVolume > 0.0);
  CC_CHECK(std::fabs(rep.enclosedVolume - rep.expectedVolume) <=
           bo::ffmdetail::kWeldVolumeAgreeFrac * rep.expectedVolume);
  // And the accepted volume is the TRUE lens within the tessellation band — a
  // volume-moving weld (the old pinch) or a leaky solid would fail this loudly.
  CC_CHECK(std::fabs(rep.enclosedVolume - ax::volCommon()) <
           std::min(0.5, 30.0 * 0.002) * ax::volCommon());
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
// The survivors' summed signed per-face volumes (the exact quantity the gate compares the
// weld against) must converge on the closed-form lens as the deflection refines, at both
// d=0.0025 and d=0.002. This pins the gate's REFERENCE independently of the weld: it is the
// oracle-free stand-in for the closed form, so the gate accepts a weld exactly when the weld
// matches this (the fixed collar-wedge weld now does at d=0.002) and declines when it cannot.
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

// ── NEVER-LEAKY: a weld whose volume CANNOT match its expectation is DECLINED, not shipped ─
// MESH-COLLAR-WEDGE removed the collar pinch, so the multi-seam COMMON now WELDS correctly
// through d=0.00125 (measured, both oracle modes) — the natural oracle-free pinch decline no
// longer fires in the reachable band. This SYNTHETIC gross-error KEEPS the SACRED never-leaky
// invariant pinned WITHOUT a pathologically-slow fine-deflection case: at the in-band welding
// deflection d=0.0025 the mesher produces a genuinely CORRECT, watertight, coherent weld, but
// handed a grossly-wrong op-volume expectation (half the true lens — a volume the correct weld
// cannot possibly enclose) the mandatory self-verify must DECLINE `VolumeInconsistent` to a
// NULL Shape. The verb NEVER returns a solid whose volume disagrees with its gate; the report
// carries the watertight/coherent witnesses (the sew CLOSED — the VOLUME is what declined) and
// the enclosed volume it measured. If the volume self-verify were ever weakened, this fails.
CC_TEST(vote_gross_volume_mismatch_declines_never_leaks) {
  const topo::Shape A = ax::buildA();
  const topo::Shape B = ax::buildB();
  const std::vector<bo::ssi::WLine>& seams = ax::closedSeams();
  CC_CHECK(seams.size() == 2);
  if (seams.size() != 2) return;

  // Sanity — with the TRUE expectation the in-band weld is ACCEPTED (a real, correct weld).
  bo::MultiSeamCutReport ok;
  const topo::Shape rOk = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, 0.0025, &ok, ax::volCommon());
  CC_CHECK(!rOk.isNull());
  CC_CHECK(ok.decline == bo::MultiSeamCutDecline::Ok);
  CC_CHECK(ok.watertight && ok.coherent);

  // Gross error — an expectation the correct weld volume cannot match ⇒ honest decline, NULL.
  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, 0.0025, &rep, 0.5 * ax::volCommon());
  CC_CHECK(r.isNull());                                                 // never a wrong solid
  CC_CHECK(rep.decline == bo::MultiSeamCutDecline::VolumeInconsistent);
  CC_CHECK(rep.watertight);   // the weld itself CLOSED (coherent watertight 2-manifold)...
  CC_CHECK(rep.coherent);
  CC_CHECK(rep.enclosedVolume > 0.0);  // ...the VOLUME is what failed the self-verify.
}

int main() { return cctest::run_all(); }

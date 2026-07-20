// SPDX-License-Identifier: Apache-2.0
//
// MULTI-SEAM CALL-ORDER INDEPENDENCE — the multi-seam verb is a PURE function of its
// arguments, so a given deflection yields the SAME verdict whether it is the first verb
// call in the process or follows calls at other deflections.
//
// ── WHY THIS SUITE EXISTS ────────────────────────────────────────────────────────
// The strip weld landed a per-mesh `SeamStripRegistry` (seam_strip.h) that caches each
// shared seam's collar strip so both incident faces emit identical near-seam triangles.
// A cache is exactly the shape of construct that CAN leak state between calls, and if
// that registry outlived a call — or were keyed without the deflection — a strip built
// for a COARSE request could satisfy a later FINE one. The caller would then receive a
// solid coarser than the deflection it asked for, precisely where the honest-decline
// invariant is meant to refuse: a SILENT-ACCURACY defect, strictly worse than an
// ordering nuisance. That property was untested, so this suite pins it directly.
//
// The registry is in fact a FUNCTION-LOCAL of `SolidMesher::meshAllFaces` (solid_mesher.h),
// constructed per mesh call and destroyed with it, and the whole tessellate/boolean lane
// holds no mutable global, `thread_local`, or RNG state — so the property HOLDS today.
// This suite is a guard, not a bug repro: it fails the moment any such cache is hoisted
// to static/global storage or keyed without the deflection.
//
// ── WHAT DRIVES THE VERDICT ──────────────────────────────────────────────────────
// The COMMON accept band is `min(0.5, 30·d)·analyticOpVolume` (freeform_freeform_multiseam.h)
// — a pure function of the REQUESTED deflection, not of anything a previous call left
// behind. The welded volume at this pose is ≈0.00702 against the closed-form ≈0.00770
// (relative error ≈0.088, the documented annulus↔annulus winding residual). At d = 0.0025
// the band is 0.075 and the measured error 0.045, so the verb returns Ok; at d = 0.0018
// the band tightens to 0.054 while the error is 0.088, so it declines VolumeInconsistent.
// The tighter request is REFUSED because it was asked for more accuracy than the mesh
// delivers — the honest outcome, and one that must not soften just because a coarser call
// ran first.
//
// Requires CYBERCAD_HAS_NUMSCI (the seams are the real S3 trace between two Béziers).
//
#include "native/boolean/freeform_freeform_multiseam.h"
#include "native/tessellate/mesh.h"

#include "freeform_freeform_multiseam_fixture.h"
#include "harness.h"

#include <vector>

namespace bo = cybercad::native::boolean;
namespace topo = cybercad::native::topology;
namespace ffx = freeform_freeform_multiseam_fixture;

namespace {

// Every witness the verb reports, so "same verdict" is compared field-by-field rather
// than on the decline enum alone (a cache leak could preserve the verdict and still
// shift the mesh, hence the BITWISE volume comparison below).
struct Outcome {
  bool isNull = false;
  bo::MultiSeamCutDecline decline = bo::MultiSeamCutDecline::Ok;
  bool watertight = false;
  bool coherent = false;
  int boundaryEdges = -1;
  int seamLoops = -1;
  double volume = 0.0;
};

Outcome runCommon(const topo::Shape& A, const topo::Shape& B,
                  const std::vector<bo::ssi::WLine>& seams, double d) {
  bo::MultiSeamCutReport rep;
  const topo::Shape r = bo::freeformFreeformMultiSeamCutWithSeams(
      A, B, seams, bo::FfOp::Common, d, &rep, ffx::volCommon());
  return Outcome{r.isNull(),
                 rep.decline,
                 rep.watertight,
                 rep.coherent,
                 static_cast<int>(rep.boundaryEdges),
                 static_cast<int>(rep.seamLoops),
                 rep.enclosedVolume};
}

// Bitwise on the volume: the mesh is built by a deterministic pipeline from immutable
// topology, so a re-run at the same deflection must reproduce the SAME double. Any
// tolerance here would hide exactly the coarser-strip reuse this suite exists to catch.
bool sameOutcome(const Outcome& a, const Outcome& b) {
  return a.isNull == b.isNull && a.decline == b.decline && a.watertight == b.watertight &&
         a.coherent == b.coherent && a.boundaryEdges == b.boundaryEdges &&
         a.seamLoops == b.seamLoops && a.volume == b.volume;
}

}  // namespace

// This must be the ONLY test in this translation unit: `cold` below is required to be the
// first multi-seam verb call in the PROCESS, which is what makes it a cold-cache run.
CC_TEST(multiseam_common_verdict_is_call_order_independent) {
  const topo::Shape A = ffx::buildA();
  const topo::Shape B = ffx::buildB();
  const std::vector<bo::ssi::WLine> seams = ffx::closedSeams();
  if (seams.size() != 2) { CC_CHECK(false); return; }

  constexpr double kFine = 0.0018;   // below the COMMON working band → declines
  constexpr double kCoarse = 0.0025; // inside it → Ok

  const Outcome cold = runCommon(A, B, seams, kFine);   // process-first: cold cache
  const Outcome prior = runCommon(A, B, seams, kCoarse);
  const Outcome warm = runCommon(A, B, seams, kFine);   // same request, warm cache

  // THE PROPERTY: a prior call at a different deflection changes nothing.
  CC_CHECK(sameOutcome(cold, warm));

  // NON-VACUITY. The fields compared above must be able to DISAGREE, or `sameOutcome`
  // would pass on any input and prove nothing. The intervening coarse call is the
  // witness: it differs from the fine runs in verdict AND in welded volume, so the
  // comparison is discriminating and the equality above is a real constraint.
  CC_CHECK(prior.decline != cold.decline);
  CC_CHECK(prior.isNull != cold.isNull);
  CC_CHECK(prior.volume != cold.volume);

  // Pin the two verdicts themselves, so a future change that makes BOTH deflections
  // behave alike cannot satisfy the equality above by flattening the distinction.
  CC_CHECK(cold.isNull);
  CC_CHECK(cold.decline == bo::MultiSeamCutDecline::VolumeInconsistent);
  CC_CHECK(!prior.isNull);
  CC_CHECK(prior.decline == bo::MultiSeamCutDecline::Ok);

  // The fine run reaches the weld and welds cleanly — its decline is the measured volume
  // residual, not an upstream give-up. This keeps the equality anchored to a run that
  // exercised the strip path rather than one that bailed early.
  CC_CHECK(cold.seamLoops == 2);
  CC_CHECK(cold.watertight);
  CC_CHECK(cold.coherent);
  CC_CHECK(cold.boundaryEdges == 0);
}

int main() { return cctest::run_all(); }

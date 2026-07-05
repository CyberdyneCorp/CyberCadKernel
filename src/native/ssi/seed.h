// SPDX-License-Identifier: Apache-2.0
//
// seed.h — result types for SSI Stage S2 (subdivision seeding).
//
// A `Seed` is one point on ONE distinct TRANSVERSAL intersection branch of two
// native surfaces, refined to lie on BOTH surfaces within tolerance, and carrying
// its parameters on both surfaces (u1,v1,u2,v2). A `SeedSet` is the deduped set of
// such seeds (≈ one per branch) plus the honest diagnostics the S2 contract
// requires — how many candidate regions the subdivision produced, and how many were
// dropped as near-tangent / degenerate (the reported S4 gap).
//
// `SeedSet` is the INPUT CONTRACT for S3 marching: the tracer walks one WLine per
// `Seed`, starting from its (u1,v1,u2,v2). `deferredTangent > 0` is a first-class,
// honest outcome (branches seen but not safely seeded → S4), not an error — the
// same "deferral-is-data" stance S1's NotAnalytic takes.
//
// This header is pure data + OCCT-free + SUBSTRATE-FREE: it does not include
// native-numerics, so it compiles even without CYBERCAD_HAS_NUMSCI (the seeding
// ENTRY POINT that fills these structs is what is guarded). Uses src/native/math
// only. clang++ -std=c++20.
//
#ifndef CYBERCAD_NATIVE_SSI_SEED_H
#define CYBERCAD_NATIVE_SSI_SEED_H

#include "native/math/vec.h"
#include "native/ssi/coincidence.h"
#include "native/ssi/tangent_contact.h"

#include <vector>

namespace cybercad::native::ssi {

namespace math = cybercad::native::math;

/// One seed point on a single transversal intersection branch. The refine has
/// driven A.point(u1,v1) − B.point(u2,v2) → 0, so `point` (taken as A.point(u1,v1))
/// lies on BOTH surfaces within `onSurfResidual`. All four parameters are clamped
/// into their respective surface's parameter box.
struct Seed {
  double u1 = 0.0;  ///< parameter U on surface A (clamped to A's range)
  double v1 = 0.0;  ///< parameter V on surface A
  double u2 = 0.0;  ///< parameter U on surface B (clamped to B's range)
  double v2 = 0.0;  ///< parameter V on surface B

  math::Point3 point{};       ///< A.point(u1,v1) ≈ B.point(u2,v2) — the seed in 3D
  double onSurfResidual = 0.0;  ///< max ‖point − surface‖ over BOTH surfaces (≤ tol)

  /// Sine of the angle between the two surface normals at the seed
  /// (= ‖n₁ × n₂‖ for unit normals). A transversal crossing has this well above
  /// zero; a value near zero is a near-tangent branch (deferred, not seeded).
  /// Kept on the seed as a transversality witness for S3/diagnostics.
  double crossingSine = 0.0;

  /// Branch/cluster id assigned by the dedup pass (0..N-1). Seeds sharing an id
  /// landed on the same branch before dedup collapsed them to this representative.
  int branchId = 0;
};

/// The full S2 result: the deduped seeds plus honest diagnostics.
struct SeedSet {
  std::vector<Seed> seeds{};   ///< deduped: ≈ one seed per distinct transversal branch

  int candidateRegions = 0;    ///< surviving AABB-overlap patch pairs before refine (diagnostic)
  int refinedAccepted = 0;     ///< candidate regions that refined to an on-both-surfaces point
                               ///< (pre-dedup; ≥ seeds.size())
  int deferredTangent = 0;     ///< candidate regions dropped as near-tangent / degenerate → S4.
                               ///< `> 0` is the honest "seen but not safely seeded" S4 signal.
                               ///< COMPATIBILITY SUMMARY: still counts every dropped near-tangent
                               ///< cluster (typed or not), so pre-S4-b callers/tests are unchanged.

  /// S4-b: the TYPED tangent contacts classified for the near-tangent clusters the seeder
  /// dropped (‖n₁×n₂‖ < tangentSinTol at the refined solution). One entry per dropped
  /// near-tangent cluster, replacing the blunt `++deferredTangent` with WHAT the
  /// degeneracy is: `TangentPoint` (isolated touch), `TangentCurve` (tangent along a
  /// curve), `NearTangentTransversal` (grazes+crosses → S4-c gap, handed on, NOT traced),
  /// or `Undecided` (within the curvature-noise band → OCCT). `deferredTangent` above stays
  /// the count for backward compatibility; this vector carries the structure. Populated
  /// only on the S2 path. Empty on a purely transversal / coincident pair.
  std::vector<TangentContact> tangentContacts{};

  /// S4-a: coincident sub-regions the seeded detector delimited on the two surfaces
  /// (kind `OverlapSubRegion`), plus any `Undecided` verdicts where an overlap was
  /// suspected but could not be robustly delimited (→ OCCT). Seeds/marching inside a
  /// delimited `OverlapSubRegion` are SUPPRESSED (not emitted as spurious branches).
  /// Empty on a purely transversal pair. Populated only on the S2 path.
  std::vector<CoincidentRegion> coincidentRegions{};

  int branchCount() const noexcept { return static_cast<int>(seeds.size()); }
};

/// Branch-recall report for the sim native-vs-OCCT verification gate. `nativeBranches`
/// is the deduped native seed count; `trueBranches` is the OCCT (or analytically
/// known) transversal branch count. `recall` is the reported completeness figure —
/// NOT asserted to be 1.0 blindly (SSI-ROADMAP S2). `worstOnSurfResidual` is the
/// largest per-seed on-both-surfaces residual (should be ≤ tol for every emitted
/// seed). `deferredTangent` echoes the S4 gap count.
struct RecallReport {
  int nativeBranches = 0;
  int trueBranches = 0;
  int deferredTangent = 0;
  double worstOnSurfResidual = 0.0;

  /// S4-f: the honest completeness floor + residual. `criticFloorFrac` is the finest
  /// minPatchFrac the adaptive completeness critic re-seeded at (0 if the critic was off);
  /// a MEASURED recall of 1.0 on a fixture is scoped to THAT fixture at THAT floor — it is
  /// never a global completeness claim. `residualAcknowledged` is ALWAYS true: below any
  /// fixed resolution a smaller loop can still be missed, so a recall of 1.0 still carries a
  /// residual (completeness is asymptotic, never a proof).
  double criticFloorFrac = 0.0;
  bool residualAcknowledged = true;

  /// Branch recall = native branches carrying ≥1 seed ÷ true transversal branches.
  /// Defined as 1.0 when there are no true branches (nothing to miss).
  double recall() const noexcept {
    if (trueBranches <= 0) return 1.0;
    const double r = static_cast<double>(nativeBranches) / static_cast<double>(trueBranches);
    return r > 1.0 ? 1.0 : r;  // over-production (dedup slack) does not exceed 1.0
  }
};

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_SEED_H

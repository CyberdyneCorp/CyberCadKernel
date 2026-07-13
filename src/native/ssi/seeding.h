// SPDX-License-Identifier: Apache-2.0
//
// seeding.h — SSI Stage S2: subdivision seeding (public native API).
//
// S2 finds ≥1 SEED POINT on every distinct TRANSVERSAL intersection branch/loop of
// two native surfaces — the input contract for S3 marching (one WLine per seed). It
// targets exactly the pairs S1's analytic dispatch returns NotAnalytic for: freeform
// (Bézier / B-spline / NURBS) pairs and the non-closed-form quadric pairs (skew
// cylinder∩cylinder, general cone∩cone, non-coaxial cone∩cyl / sphere∩cyl /
// sphere∩cone, oblique plane∩torus, torus∩curved).
//
// METHOD (clean-room, SSI-ROADMAP S2; OCCT IntPolyh / IntPatch PrmPrm as ORACLE
// only, never copied):
//
//   1. SUBDIVIDE + PRUNE. Start from the two full param domains as one patch pair.
//      Recursively: if the two patches' conservative AABBs are DISJOINT, discard the
//      pair (the prune — where the work is saved); else if both patches are below a
//      size/depth threshold, emit the pair as a CANDIDATE REGION; else split the
//      larger patch on its longer param direction and recurse. Bounded by max depth
//      + min patch size (tolerance-scaled) so it always terminates.
//
//   2. REFINE (substrate least_squares). For each candidate region, seed
//      native-numerics `least_squares` at the region center and drive the residual
//      r(u1,v1,u2,v2) = A.point(u1,v1) − B.point(u2,v2) (m=3, n=4 — the extra DOF is
//      the along-curve freedom; LM's damping handles the rank-deficient-by-one
//      Jacobian) to a point on BOTH surfaces, CLAMPING the params to each range. A
//      converged refine with on-both-surfaces residual ≤ tol is a seed; a refine that
//      fails or that is NEAR-TANGENT (‖n₁ × n₂‖ ≈ 0 at the solution) is NOT seeded —
//      it increments `deferredTangent` (an honest S4 gap), never a fabricated seed.
//
//   3. DEDUP (spatial clustering). Many candidate regions on one branch refine to
//      nearby points; cluster by 3D proximity (tol-scaled radius) and keep one
//      representative per cluster → ≈ one seed per distinct branch/loop.
//
// HONEST SCOPE. S2 is TRANSVERSAL-only. Near-tangent / coincident / degenerate
// seeding ill-conditions the refine and is DEFERRED to S4 (reported in
// `deferredTangent`, never faked). Completeness is a MEASURED recall figure, not a
// blind 100% claim: too-shallow subdivision can miss a small loop (the acknowledged
// failure mode); a deeper `maxDepth` recovers smaller loops at more cost.
//
// SSI is INTERNAL — no cc_* entry point, no ABI change. Verified at the
// cybercad::native::ssi C++ boundary (host known-branch-count + sim OCCT recall).
//
// SUBSTRATE GUARD. Subdivision/pruning/dedup are OCCT-free and substrate-free, but a
// USEFUL seeder requires the refine, so `seed_intersection` is compiled only under
// CYBERCAD_HAS_NUMSCI (like native-numerics). The adapter factories and options are
// always visible (they need no substrate). clang++ -std=c++20, fp64, deterministic.
//
#ifndef CYBERCAD_NATIVE_SSI_SEEDING_H
#define CYBERCAD_NATIVE_SSI_SEEDING_H

#include "native/math/bezier.h"
#include "native/math/bspline.h"
#include "native/math/elementary.h"
#include "native/math/torus.h"
#include "native/ssi/patch_bounds.h"
#include "native/ssi/seed.h"

#include <span>
#include <vector>

namespace cybercad::native::ssi {

// ─────────────────────────────────────────────────────────────────────────────
// SeedOptions — the tolerance-scaled subdivision/refine/dedup knobs. Defaults are
// derived from the operands' scale at call time when left at their sentinel (≤ 0).
// `maxDepth` is the completeness knob: deeper subdivision recovers smaller loops at
// more cost (documented trade-off, not a correctness guarantee).
// ─────────────────────────────────────────────────────────────────────────────
struct SeedOptions {
  int maxDepth = 32;          ///< HARD recursion cap (termination safety), not the resolution knob
  double onSurfTol = -1.0;    ///< accept-seed residual (≤ 0 → scale·1e-7)
  double tangentSinTol = 1e-3;///< ‖n₁×n₂‖ below this at the solution ⇒ near-tangent → S4
  double dedupRadius = -1.0;  ///< reserved: explicit 3D cluster radius override (≤ 0 → topological dedup)
  double minPatchFrac = -1.0; ///< leaf param-fraction per direction — the RESOLUTION/recall knob (≤ 0 → 1/32)
  int initialGridU = 1;       ///< pre-split of the U domain before recursion (loop-catching)
  int initialGridV = 1;       ///< pre-split of the V domain before recursion

  // ── S4-f ADAPTIVE COMPLETENESS CRITIC (default OFF → the current fixed-resolution seed,
  // byte-identical). When on, trace_intersection runs the fixed-resolution seed+trace first,
  // then LOOP-UNTIL-DRY: re-subdivide finer (minPatchFrac *= criticRefineFactor per round) in
  // regions no traced curve covers, refine at the SAME onSurfTol (a candidate that does not
  // land on BOTH surfaces is DISCARDED — never a fabricated seed), dedup new seeds vs all kept
  // curves, trace the genuinely new ones. Bounded by a cost cap; reports the floor + residual.
  //
  // HONEST FRAMING: this RAISES the recall floor; it is NOT a completeness proof. Below any
  // fixed re-seed round a smaller loop can still be missed (TraceSet.completenessResidual). ──
  bool completenessCritic = false; ///< S4-f master switch (off → fixed-resolution seed, byte-identical)
  double criticRefineFactor = 0.5; ///< minPatchFrac *= this each round (finer resolution; must be < 1)
  int criticDryRounds = 2;         ///< K: stop after this many CONSECUTIVE rounds finding NO new branch
  int criticMaxRounds = 6;         ///< hard cap on re-seed rounds (cost bound)
  int criticMaxCandidates = 4096;  ///< hard cap on total re-seed candidate regions across all rounds (cost bound)

  // ── M1c SEEDING-RECALL BUMP (default OFF → the shipped whole-domain critic re-seed,
  // byte-identical). When on (and completenessCritic is also on) the critic re-seeds ONLY the
  // param cells NO traced curve covers — each uncovered A-cell is seeded as a RESTRICTED
  // sub-domain (A clamped to the cell) against B's full domain, and the recovered seeds are
  // accumulated. This is the residual/coverage-guided TARGETED re-seed: it reliably recovers the
  // SECOND loop of a twice-piercing off-axis quadric pose (which the coarse fixed grid merges into
  // one topological cluster → one representative seed) at practical grid densities, and is cheaper
  // than re-scanning the whole domain because it never re-subdivides the already-covered region.
  // It NEVER fabricates a seed (each cell's candidate must still land on BOTH surfaces) and NEVER
  // widens a tolerance; it only changes WHERE the finer re-seed looks. Bounded by criticMaxCells. ──
  bool criticTargetedReseed = false; ///< M1c: critic re-seeds only the uncovered A-cells (needs completenessCritic)
  int criticMaxCells = 64;           ///< M1c: hard cap on uncovered cells re-seeded per round (cost bound)

  // ── DISTINCT-BRANCH SPLIT (default ON) — recover merged co-resident loops ────────
  // The adjacency clustering (clusterRegions) unites candidate regions whose PARAM boxes
  // touch on both surfaces into one cluster. Two DISTINCT co-resident transversal loops
  // that a dense freeform pair hosts close together can have touching candidate boxes and
  // get merged into ONE cluster — the refine then kept only the single tightest seed and
  // DROPPED the second loop (the dominant measured NURBS-L2 multi-branch decline).
  //
  // With this on, the refine emits EVERY refined seed of a cluster that lands on a
  // SPATIALLY-DISTINCT 3D locus (3D seed points separated by more than
  // `splitDistinctFrac · modelScale`), each still driven onto BOTH surfaces at the SAME
  // `onSurfTol` and transversal (‖n₁×n₂‖ ≥ tangentSinTol) — never a widened tolerance, never
  // a fabricated seed. A cluster hosting a single physical loop still collapses to one seed
  // (all its refined points sit within the separation), so single-loop / canonical cases are
  // unchanged. Bounded by `splitMaxPerCluster` so refine noise cannot over-produce.
  //
  // CORRECTNESS: an over-split seed (two seeds that are actually the same loop, > separation
  // apart) is HARMLESS — it re-traces the same loop and the S3 marcher's per-branch
  // locus-dedup (`retraces`/`sameLocus`) collapses it. So the predicate is recall-only: it
  // can add a genuine second-loop seed or a harmless duplicate, never a wrong result.
  bool splitDistinctBranches = true;  ///< emit every spatially-distinct refined locus per cluster
  double splitDistinctFrac = 1.0 / 16.0;  ///< distinct-locus 3D separation as a fraction of modelScale
  int splitMaxPerCluster = 8;         ///< hard cap on distinct seeds emitted from one cluster (cost bound)

  // ── LOOP-STRUCTURE-AWARE CAP RETENTION — keep BOTH co-resident loci under the cap ─────
  // A single param-adjacency cluster on an ULTRA-DENSE freeform pair can host FAR MORE
  // refined transversal crossings than the per-cluster retention budget (measured:
  // 272 847 / 215 834 candidates on ONE cluster, seeds > 65 536). The previous FIFO
  // truncation appended refined seeds in candidate-iteration order until the cap, so one
  // dense locus's leaves FILLED the budget and the CO-RESIDENT locus's later leaves were
  // dropped BEFORE the distinct-locus split ever saw them → the split emitted one seed
  // for a twice-piercing pose (the residual co-resident miss the split-linkage lever
  // could not reach — the miss is UPSTREAM of the split).
  //
  // The fix is to make the cap retention LOOP-STRUCTURE-AWARE. When a cluster's refined
  // pile exceeds the budget, FIRST partition it into distinct co-resident SUB-LOCI (the
  // SAME spatial connected components the distinct-locus split uses — 3D single-linkage at
  // the split separation `sep`), THEN retain a per-sub-locus UNIFORM STRIDE so EACH locus
  // keeps FULL arc coverage (evenly-spaced leaves, no gaps) and BOTH survive to the split.
  // This is NOT plain thinning (rejected by E1: a flat stride over a merged pile leaves arc
  // gaps that spuriously OVER-SPLIT one loop) — the partition happens BEFORE the stride, so
  // a stride is applied WITHIN each connected locus, never across the gap between two loci.
  //
  // DISAGREED-SAFE / RECALL-ONLY. Every retained seed is still a real refined
  // on-both-surfaces transversal crossing (`refineRegion` gates point-on-both ≤ `onSurfTol`
  // and ‖n₁×n₂‖ ≥ `tangentSinTol`); retention only changes WHICH real seeds reach the split,
  // never widens a tolerance nor fabricates a locus. It only fires when the pile exceeds the
  // budget on the freeform↔freeform split path; smaller piles and the elementary/mixed path
  // are byte-identical (no partition, no stride). Uniform stride within a locus preserves the
  // dense chain that keeps ONE physical loop connected, so it does NOT introduce the arc gaps
  // that would over-split a single loop.
  int capRetentionBudget = 65536;    ///< target refined seeds retained per cluster after loop-aware decimation
};

// ─────────────────────────────────────────────────────────────────────────────
// Adapter factories — wrap a native-math surface as a SurfaceAdapter (evaluator +
// param domain + conservative AABB provider) so every surface kind flows through the
// one subdivision path. Elementary/torus use the sampled+margin bound; freeform uses
// the control-net convex hull. These are OCCT-free and substrate-free.
// ─────────────────────────────────────────────────────────────────────────────

/// Adapter for any elementary/torus surface exposing point(u,v) / normal(u,v) over
/// the param box [u0,u1]×[v0,v1]. Bound = sampled+Lipschitz-margin (always sound).
/// `uPeriod`/`vPeriod` give the angular period (2π for a cylinder/cone/sphere U and a
/// torus U/V; 0 for the non-periodic v of a cylinder/cone or the plane) so a loop
/// crossing the seam is deduped as one branch. See makeCylinderAdapter etc. below for
/// the per-kind period presets.
template <class Surf>
SurfaceAdapter makeElementaryAdapter(const Surf& s, const ParamBox& domain,
                                     double uPeriod = 0.0, double vPeriod = 0.0) {
  SurfaceAdapter a;
  a.domain = domain;
  a.uPeriod = uPeriod;
  a.vPeriod = vPeriod;
  a.point = [s](double u, double v) { return s.value(u, v); };
  a.normal = [s](double u, double v) { return s.normal(u, v); };
  a.bound = [s](const ParamBox& box) {
    return sampledBound([s](double u, double v) { return s.value(u, v); }, box);
  };
  const Aabb full = a.bound(domain);
  a.modelScale = full.valid() ? full.diagonal() : 1.0;
  return a;
}

/// 2π in radians — the angular period shared by the elementary surfaces' U direction.
inline constexpr double kTwoPi = 6.28318530717958647692;

/// Cylinder/cone: U angular (period 2π), V linear (non-periodic).
inline SurfaceAdapter makeCylinderAdapter(const math::Cylinder& c, const ParamBox& dom) {
  return makeElementaryAdapter(c, dom, kTwoPi, 0.0);
}
inline SurfaceAdapter makeConeAdapter(const math::Cone& c, const ParamBox& dom) {
  return makeElementaryAdapter(c, dom, kTwoPi, 0.0);
}
/// Sphere: U longitude (period 2π), V latitude in [-π/2, π/2] (non-periodic).
inline SurfaceAdapter makeSphereAdapter(const math::Sphere& s, const ParamBox& dom) {
  return makeElementaryAdapter(s, dom, kTwoPi, 0.0);
}
/// Torus: both U (major) and V (minor) angular, period 2π.
inline SurfaceAdapter makeTorusAdapter(const math::Torus& t, const ParamBox& dom) {
  return makeElementaryAdapter(t, dom, kTwoPi, kTwoPi);
}
/// Plane: both directions linear (non-periodic).
inline SurfaceAdapter makePlaneAdapter(const math::Plane& p, const ParamBox& dom) {
  return makeElementaryAdapter(p, dom, 0.0, 0.0);
}

/// Adapter for a non-rational Bézier surface (nRows×nCols poles, degrees nRows-1 /
/// nCols-1) over [0,1]². Bound = control-net convex hull. Point/normal via
/// native-math bezierSurfaceD1.
SurfaceAdapter makeBezierAdapter(std::vector<Point3> poles, int nRows, int nCols);

/// Adapter for a non-rational B-spline surface. `knotsU/V` are flat knot vectors.
/// Domain is the clamped knot range [knotsU[degU], knotsU[nU]] × similarly in V.
/// Bound = control-net convex hull. Point/normal via native-math surfacePoint /
/// surfaceNormal.
SurfaceAdapter makeBSplineAdapter(int degreeU, int degreeV,
                                  std::vector<Point3> poles, int nRows, int nCols,
                                  std::vector<double> knotsU, std::vector<double> knotsV);

/// Adapter for a rational NURBS surface (weights parallel the pole grid, wᵢ > 0).
/// Bound = control-net convex hull of the PROJECTED poles (sound for wᵢ > 0).
SurfaceAdapter makeNurbsAdapter(int degreeU, int degreeV,
                                std::vector<Point3> poles, std::vector<double> weights,
                                int nRows, int nCols,
                                std::vector<double> knotsU, std::vector<double> knotsV);

// ─────────────────────────────────────────────────────────────────────────────
// seed_intersection — the S2 entry point (requires the native-numerics refine).
//
// Returns the deduped SeedSet: ≈ one Seed per distinct transversal branch, each on
// both surfaces within tol with its (u1,v1,u2,v2), plus the honest diagnostics
// (candidateRegions / refinedAccepted / deferredTangent). Deterministic.
//
// GUARD: the DECLARATION is always visible (like native-numerics), but the
// DEFINITION (seeding.cpp) is compiled only under CYBERCAD_HAS_NUMSCI — the refine
// calls least_squares. A TU that links a kernel built without the substrate will not
// resolve this symbol; that is by design (a useful seeder needs the refine).
// ─────────────────────────────────────────────────────────────────────────────
SeedSet seed_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                          const SeedOptions& opts = {});

}  // namespace cybercad::native::ssi

#endif  // CYBERCAD_NATIVE_SSI_SEEDING_H

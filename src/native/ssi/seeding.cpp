// SPDX-License-Identifier: Apache-2.0
//
// seeding.cpp — SSI Stage S2 subdivision seeding implementation.
//
// Two parts:
//   * Adapter factories (freeform surfaces) — OCCT-free, substrate-free; always
//     compiled. They wrap native-math Bézier/B-spline/NURBS evaluators + a
//     control-net-convex-hull AABB provider.
//   * seed_intersection (subdivide → prune → refine → dedup) — the recall-bearing
//     seeder; the REFINE calls native-numerics least_squares, so this half is under
//     CYBERCAD_HAS_NUMSCI.
//
// See seeding.h for the method + honest-scope contract. Clean-room from
// SSI-ROADMAP S2; OCCT IntPolyh/IntPatch as oracle only.
//
#include "native/ssi/seeding.h"

#include "native/ssi/tangent_seeded.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef CYBERCAD_HAS_NUMSCI
#include "native/numerics/numerics.h"
#endif

namespace cybercad::native::ssi {

namespace {

// Clamp a knot vector's clamped domain [knots[deg], knots[nPoles]].
ParamBox knotDomain(int degU, int degV, int nRows, int nCols,
                    const std::vector<double>& kU, const std::vector<double>& kV) {
  ParamBox b;
  b.u0 = kU[static_cast<std::size_t>(degU)];
  b.u1 = kU[static_cast<std::size_t>(nRows)];
  b.v0 = kV[static_cast<std::size_t>(degV)];
  b.v1 = kV[static_cast<std::size_t>(nCols)];
  return b;
}

// Build a SurfaceAdapter around a freeform point/normal evaluator + control net.
// `degU`/`degV` are the surface's polynomial degrees, used only to compute the span
// count (density) signal for scale-adaptive seeding.
SurfaceAdapter freeformAdapter(ControlNet net, ParamBox domain, int degU, int degV,
                               std::function<Point3(double, double)> point,
                               std::function<Dir3(double, double)> normal) {
  SurfaceAdapter a;
  a.domain = domain;
  a.point = std::move(point);
  a.normal = std::move(normal);
  // Bound = the INTERSECTION of two independently sound bounds: the control-net
  // convex hull (tight where the net localizes — many-span B-splines) AND the
  // sampled+Lipschitz-margin bound (which always TIGHTENS with the sub-box, covering
  // the single-span Bézier / coarse-B-spline case where every pole influences every
  // point so the hull does not shrink). Intersecting two sound bounds stays sound and
  // gives a bound that always shrinks under subdivision — the property pruning needs.
  a.bound = [net, domain, pt = a.point](const ParamBox& box) {
    Aabb hull = controlNetBound(net, domain, box);
    Aabb sampled = sampledBound(pt, box);
    if (!hull.valid()) return sampled;
    return aabbIntersect(hull, sampled);
  };
  const Aabb full = a.bound(domain);
  a.modelScale = full.valid() ? full.diagonal() : 1.0;
  // Scale-adaptive-seeding signals: how WAVY this operand's control net is (0 for a
  // simple bump/dish; high for an oscillating egg-carton net) and how DENSE it is (span
  // count = polynomial-patch tiling). Both set on the adapter so seed_intersection can
  // pick a finer DEFAULT initial resolution only for genuinely freeform, wavy/dense pairs.
  a.freeformComplexity = controlNetOscillation(net);
  const int spansU = std::max(1, net.nRows - degU);
  const int spansV = std::max(1, net.nCols - degV);
  a.freeformSpanCount = spansU * spansV;
  return a;
}

}  // namespace

// ── freeform adapter factories ─────────────────────────────────────────────────

SurfaceAdapter makeBezierAdapter(std::vector<Point3> poles, int nRows, int nCols) {
  ControlNet net{poles, nRows, nCols};
  ParamBox domain{0.0, 1.0, 0.0, 1.0};
  auto polesCopy = poles;  // owned by the closures
  auto point = [polesCopy, nRows, nCols](double u, double v) {
    return math::bezierSurfacePoint(polesCopy, nRows, nCols, u, v);
  };
  auto normal = [polesCopy, nRows, nCols](double u, double v) {
    return math::bezierSurfaceD1(polesCopy, nRows, nCols, u, v).normal;
  };
  // A Bézier surface is a SINGLE polynomial patch: degree = nPoles − 1 per direction, so
  // its span count is 1 (freeformAdapter computes spans = max(1, nPoles − degree)).
  return freeformAdapter(std::move(net), domain, nRows - 1, nCols - 1,
                         std::move(point), std::move(normal));
}

SurfaceAdapter makeBSplineAdapter(int degreeU, int degreeV,
                                  std::vector<Point3> poles, int nRows, int nCols,
                                  std::vector<double> knotsU, std::vector<double> knotsV) {
  ControlNet net{poles, nRows, nCols};
  ParamBox domain = knotDomain(degreeU, degreeV, nRows, nCols, knotsU, knotsV);
  auto point = [=](double u, double v) {
    math::SurfaceGrid grid{poles, nRows, nCols};
    return math::surfacePoint(degreeU, degreeV, grid, knotsU, knotsV, u, v);
  };
  auto normal = [=](double u, double v) {
    math::SurfaceGrid grid{poles, nRows, nCols};
    return math::surfaceNormal(degreeU, degreeV, grid, {}, knotsU, knotsV, u, v);
  };
  return freeformAdapter(std::move(net), domain, degreeU, degreeV,
                         std::move(point), std::move(normal));
}

SurfaceAdapter makeNurbsAdapter(int degreeU, int degreeV,
                                std::vector<Point3> poles, std::vector<double> weights,
                                int nRows, int nCols,
                                std::vector<double> knotsU, std::vector<double> knotsV) {
  // The convex-hull bound uses the PROJECTED poles (already stored in `poles`); for
  // wᵢ > 0 the rational point is a convex combination of them, so their hull is sound.
  ControlNet net{poles, nRows, nCols};
  ParamBox domain = knotDomain(degreeU, degreeV, nRows, nCols, knotsU, knotsV);
  auto point = [=](double u, double v) {
    math::SurfaceGrid grid{poles, nRows, nCols};
    return math::nurbsSurfacePoint(degreeU, degreeV, grid, weights, knotsU, knotsV, u, v);
  };
  auto normal = [=](double u, double v) {
    math::SurfaceGrid grid{poles, nRows, nCols};
    // Rational normal (surfaceNormal applies the quotient rule internally for wᵢ>0).
    return math::surfaceNormal(degreeU, degreeV, grid, weights, knotsU, knotsV, u, v);
  };
  return freeformAdapter(std::move(net), domain, degreeU, degreeV,
                         std::move(point), std::move(normal));
}

#ifdef CYBERCAD_HAS_NUMSCI

namespace {

namespace nn = cybercad::native::numerics;

inline double clampd(double x, double lo, double hi) noexcept {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ── the recursive subdivision (prune / emit / split) ───────────────────────────
//
// Emits candidate regions (patch-pair centers) into `out`. Guard-clause structure:
// prune-disjoint → emit-if-small → split-larger-and-recurse. Cognitive complexity is
// in the SSI systems band (recursion + a 3-way branch); the AABB test and the split
// are isolated helpers so the recursion body stays a short guarded dispatch.
struct CandidateRegion {
  ParamBox a;  // sub-box on surface A
  ParamBox b;  // sub-box on surface B
};

// FEATURE-ADAPTIVE leaf refinement decision (additive, DISAGREED-safe). At a would-be uniform
// leaf, refine FURTHER (below `minPatchFrac`, toward the `adaptiveMinFrac` floor) on any cell
// where the two patch AABBs overlap at all — the caller has already pruned disjoint pairs, so in
// practice that is EVERY cell reaching here. Returns true iff there is room to refine (not both
// patches at the adaptive floor). Never emits a point — it only asks whether to split MORE; the
// emitted candidates still pass refineRegion.
//
// READ THE TEST BELOW LITERALLY. `ov.diagonal() <= overlapFrac * smaller` with the default
// `overlapFrac = 1.0` (seeding.h) is a TAUTOLOGY: the overlap box is contained in both patch
// boxes, so its diagonal never exceeds the smaller one. `overlapFrac` is therefore an optional
// SHALLOW-graze guard that is inert at its default, NOT a deep-overlap requirement — this
// docstring previously claimed the opposite (a `>=` "OVERLAP DEEPLY" test), contradicting both
// the code and the header.
//
// THE TAUTOLOGY IS LOAD-BEARING — do NOT "fix" the sign. Flipping to the documented `>=` was
// measured to be digit-for-digit identical to `adaptiveSubdivision = false` (30 421 candidates,
// 2 branches) and DROPS a real co-resident transversal locus; tightening `overlapFrac` to 0.5 or
// 0.25 likewise fails `test_native_ssi_seeding.cpp` `branchCount() == 3`. What this function
// actually delivers is a UNIFORM one-more-level refinement on every freeform↔freeform pair, and
// the third locus depends on it.
//
// The cost of that is real and is a known open defect: on a COINCIDENT pair every cell overlaps
// by construction, so this drives the WHOLE domain to `adaptiveMinFrac` rather than a feature
// cell (measured ladder 5 476 → 98 596 candidates as the floor goes 1/8 → 1/64). The fix is a
// sound descent stop, not a change to this predicate — see MOAT-ROADMAP.md M1 item A2.
inline bool featureWarrantsFinerLeaf(const Aabb& boxA, const Aabb& boxB,
                                     const ParamBox& ba, const ParamBox& bb,
                                     const SurfaceAdapter& A, const SurfaceAdapter& B,
                                     double adaptiveMinFrac, double overlapFrac) {
  // Room to refine: stop once BOTH patches are already at the adaptive floor.
  const bool aFloor = ba.du() <= adaptiveMinFrac * A.domain.du() &&
                      ba.dv() <= adaptiveMinFrac * A.domain.dv();
  const bool bFloor = bb.du() <= adaptiveMinFrac * B.domain.du() &&
                      bb.dv() <= adaptiveMinFrac * B.domain.dv();
  if (aFloor && bFloor) return false;
  // Overlap-box diagonal relative to the smaller patch AABB diagonal (shape-free, scale-free
  // deep-overlap witness). aabbIntersect is valid iff the boxes overlap (the caller already
  // ruled out disjoint), so its diagonal is the overlap extent.
  const Aabb ov = aabbIntersect(boxA, boxB);
  if (!ov.valid()) return false;
  const double smaller = std::min(boxA.diagonal(), boxB.diagonal());
  if (smaller <= 0.0) return false;
  // The cell brackets a genuine near-crossing (the caller ruled out disjoint); refine it one
  // level finer so a co-resident locus that shares the coarse leaf with the dominant locus gets
  // its OWN candidate → its own cluster → its own seed. `overlapFrac` is an optional SHALLOW
  // guard (skip a corner-graze whose overlap exceeds it — a no-op at the default 1.0 that keeps
  // every overlapping cell). The `adaptiveMinFrac` floor bounds the extra refinement cost.
  return ov.diagonal() <= overlapFrac * smaller;
}

// SIBLING-BOUND REUSE (`knownA` / `knownB`, exact). Each level splits exactly ONE operand, so the
// OTHER operand's param box is bit-identical in both children — and `bound` is a pure function of
// (surface, ParamBox) over by-value captures (see the adapter construction above), so recomputing
// it there returns the same Aabb by construction. Passing it down is therefore an exactness-
// preserving elimination of redundant work, not an approximation: the candidate list comes out
// element-for-element identical. Measured 2.00× fewer `bound` calls on every fixture tried
// (18 651 288 → 9 325 788 on the coincident repro), for a 1.9–2.1× faster descent.
//
// It is a CONSTANT-factor win only. It does not change how many nodes the recursion visits, so on
// a 2D coincident locus — where the AABB prune can never fire and the node count is the actual
// pathology — it returns ~14% of the wall clock, not the blow-up.
void subdivide(const SurfaceAdapter& A, const SurfaceAdapter& B,
               const ParamBox& ba, const ParamBox& bb, int depth,
               int maxDepth, double minFracU_A, double minFracV_A,
               double minFracU_B, double minFracV_B, double gap,
               std::vector<CandidateRegion>& out,
               bool adaptive, double adaptiveMinFrac, double adaptiveOverlapFrac,
               const Aabb* knownA = nullptr, const Aabb* knownB = nullptr) {
  const Aabb boxA = knownA ? *knownA : A.bound(ba);
  const Aabb boxB = knownB ? *knownB : B.bound(bb);
  if (!boxA.valid() || !boxB.valid()) return;
  if (aabbDisjoint(boxA, boxB, gap)) return;  // prune: no intersection in this region

  // Are both patches below the size/depth threshold? Then it is a candidate region.
  const bool aSmall = ba.du() <= minFracU_A * A.domain.du() &&
                      ba.dv() <= minFracV_A * A.domain.dv();
  const bool bSmall = bb.du() <= minFracU_B * B.domain.du() &&
                      bb.dv() <= minFracV_B * B.domain.dv();
  if (depth >= maxDepth ||
      (aSmall && bSmall &&
       !(adaptive &&
         featureWarrantsFinerLeaf(boxA, boxB, ba, bb, A, B, adaptiveMinFrac, adaptiveOverlapFrac)))) {
    out.push_back({ba, bb});
    return;
  }

  // Split the patch with the larger 3D AABB (bisecting the bigger bracket shrinks the
  // dominant uncertainty fastest and drives BOTH patches toward the size threshold —
  // so a leaf is small on both surfaces, which the topological dedup relies on). Ties
  // and degenerate bounds fall back to splitting A. No fixed A/B alternation: that
  // could leave one surface's patch large at a leaf and over-merge distinct branches.
  const bool splitA = boxA.diagonal() >= boxB.diagonal();
  auto halves = [](const ParamBox& p) -> std::pair<ParamBox, ParamBox> {
    ParamBox lo = p, hi = p;
    if (p.uIsLonger()) { lo.u1 = p.uMid(); hi.u0 = p.uMid(); }
    else               { lo.v1 = p.vMid(); hi.v0 = p.vMid(); }
    return {lo, hi};
  };
  if (splitA) {
    // B's box is unchanged in both children → hand `boxB` down instead of recomputing it twice.
    auto [a0, a1] = halves(ba);
    subdivide(A, B, a0, bb, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out,
              adaptive, adaptiveMinFrac, adaptiveOverlapFrac, nullptr, &boxB);
    subdivide(A, B, a1, bb, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out,
              adaptive, adaptiveMinFrac, adaptiveOverlapFrac, nullptr, &boxB);
  } else {
    // Symmetrically: A's box is unchanged in both children.
    auto [b0, b1] = halves(bb);
    subdivide(A, B, ba, b0, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out,
              adaptive, adaptiveMinFrac, adaptiveOverlapFrac, &boxA, nullptr);
    subdivide(A, B, ba, b1, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out,
              adaptive, adaptiveMinFrac, adaptiveOverlapFrac, &boxA, nullptr);
  }
}

// A near-tangent refined solution the seeder dropped: enough to run the S4-b
// differential-geometry classifier (params on both surfaces, the base point, both
// normals, and the measured crossing sine).
struct NearTangentSolution {
  double u1 = 0.0, v1 = 0.0, u2 = 0.0, v2 = 0.0;
  Point3 point{};
  Dir3 nA{}, nB{};
  double crossingSine = 0.0;
};

// ── refine one candidate region with least_squares ─────────────────────────────
//
// Returns true and fills `seed` on a converged, on-both-surfaces, transversal
// result; returns false (and sets `nearTangent`) when the region is near-tangent /
// degenerate (→ deferredTangent) or the refine simply did not converge. When
// `nearTangent` is set, `ntSol` carries the dropped solution for S4-b classification.
bool refineRegion(const SurfaceAdapter& A, const SurfaceAdapter& B,
                  const CandidateRegion& reg, const SeedOptions& opts,
                  double onSurfTol, Seed& seed, bool& nearTangent,
                  NearTangentSolution& ntSol) {
  nearTangent = false;
  // residual r(x) = A.point(u1,v1) − B.point(u2,v2), clamped into both domains.
  const ParamBox& da = A.domain;
  const ParamBox& db = B.domain;
  auto clampX = [&](const nn::Vector& x) {
    return std::array<double, 4>{
        clampd(x[0], da.u0, da.u1), clampd(x[1], da.v0, da.v1),
        clampd(x[2], db.u0, db.u1), clampd(x[3], db.v0, db.v1)};
  };
  nn::VecFn resid = [&](const nn::Vector& x) -> nn::Vector {
    const auto c = clampX(x);
    const Point3 pa = A.point(c[0], c[1]);
    const Point3 pb = B.point(c[2], c[3]);
    const Vec3 d = pa - pb;
    return {d.x, d.y, d.z};
  };
  const nn::Vector x0{reg.a.uMid(), reg.a.vMid(), reg.b.uMid(), reg.b.vMid()};
  const nn::SolveResult r = nn::least_squares(resid, x0);
  const auto c = clampX(r.x);

  const Point3 pa = A.point(c[0], c[1]);
  const Point3 pb = B.point(c[2], c[3]);
  const double gapDist = math::distance(pa, pb);
  if (!r.success && gapDist > onSurfTol) return false;  // did not converge to a crossing
  if (gapDist > onSurfTol) return false;                // clamped off the branch

  // Transversality: reject near-tangent (‖n₁ × n₂‖ ≈ 0) as an S4 gap, never faked.
  const Dir3 nA = A.normal(c[0], c[1]);
  const Dir3 nB = B.normal(c[2], c[3]);
  const double sinAngle = math::norm(math::cross(nA.vec(), nB.vec()));
  if (sinAngle < opts.tangentSinTol) {
    nearTangent = true;
    ntSol = {c[0], c[1], c[2], c[3], pa, nA, nB, sinAngle};  // hand to the S4-b classifier
    return false;
  }

  seed.u1 = c[0]; seed.v1 = c[1]; seed.u2 = c[2]; seed.v2 = c[3];
  seed.point = pa;
  seed.onSurfResidual = gapDist;   // A/B agree at pa≈pb; gap is the on-both residual
  seed.crossingSine = sinAngle;
  return true;
}

// ── S4-a: seeded coincident-patch detection ─────────────────────────────────────
//
// Two surfaces can COINCIDE OVER A REGION (not just cross): every point of a patch of A
// also lies on B, with the two normals aligned there. That is a 2D shared locus, NOT a
// transversal branch — seeding/marching it would emit spurious "branches" all over the
// overlap. S4-a detects it and returns a delimited `OverlapSubRegion` (or `Undecided`
// when the boundary cannot be pinned down), so the seeder can SUPPRESS the spurious work.
//
// The test is HONEST: a sample (uA,vA) on A AGREES with B iff, projecting A.point(uA,vA)
// onto B by least_squares, the on-both residual ≤ onSurfTol AND the normals align
// (‖nA×nB‖ ≤ tangentSinTol). A candidate region is a coincident patch only if a whole
// interior grid agrees; we then GROW the A-box to the agreement boundary by edge
// bisection and read the matching B-box off the projected corners. If the agreement runs
// to a domain edge (can't tell whether the overlap truly stops there or the surface is
// merely trimmed) or the boundary will not bisect cleanly, we return `Undecided` — never
// a fabricated rectangle.

// Project a 3D point onto surface B: least_squares over (u2,v2) minimising ‖p − B(u2,v2)‖,
// clamped to B's domain, seeded at (su,sv). Returns the clamped params + the residual +
// whether the surface normals at (uA,vA) on A and the projection on B ALIGN.
struct ProjResult {
  double u2 = 0.0, v2 = 0.0;
  double residual = 0.0;
  double normalSine = 1.0;  // ‖nA × nB‖ at the matched points
};

ProjResult projectOntoB(const SurfaceAdapter& B, const Point3& p, const Dir3& nA,
                        double su, double sv) {
  const ParamBox& db = B.domain;
  auto clampUV = [&](const nn::Vector& x) {
    return std::array<double, 2>{clampd(x[0], db.u0, db.u1), clampd(x[1], db.v0, db.v1)};
  };
  nn::VecFn resid = [&](const nn::Vector& x) -> nn::Vector {
    const auto c = clampUV(x);
    const Vec3 d = p - B.point(c[0], c[1]);
    return {d.x, d.y, d.z};
  };
  // Robust seed: a coarse grid scan of B for the nearest starting param, so the
  // least_squares refine converges to the TRUE nearest point regardless of the caller's
  // hint (which may be far off when sampling across a grown region). `su,sv` bias the
  // scan's tie-break toward the caller's expected match but never override a closer grid
  // point. Cost is a fixed small grid, run only inside the coincidence detector.
  double bu = clampd(su, db.u0, db.u1), bv = clampd(sv, db.v0, db.v1);
  double best = math::distance(p, B.point(bu, bv));
  constexpr int kScan = 8;
  for (int i = 0; i <= kScan; ++i)
    for (int j = 0; j <= kScan; ++j) {
      const double cu = db.u0 + db.du() * (double(i) / kScan);
      const double cv = db.v0 + db.dv() * (double(j) / kScan);
      const double d = math::distance(p, B.point(cu, cv));
      if (d < best) { best = d; bu = cu; bv = cv; }
    }
  const nn::SolveResult r = nn::least_squares(resid, nn::Vector{bu, bv});
  const auto c = clampUV(r.x);
  ProjResult out;
  out.u2 = c[0];
  out.v2 = c[1];
  out.residual = math::distance(p, B.point(c[0], c[1]));
  const Dir3 nB = B.normal(c[0], c[1]);
  out.normalSine = math::norm(math::cross(nA.vec(), nB.vec()));
  return out;
}

// Does surface A at (uA,vA) coincide with surface B (point on B + normals aligned)?
// `su,sv` seed the projection near the expected match. On agreement, writes the matched
// B params into `bu,bv` for boundary delimiting.
bool sampleAgrees(const SurfaceAdapter& A, const SurfaceAdapter& B, double uA, double vA,
                  double onSurfTol, double tangentSinTol, double su, double sv,
                  double& bu, double& bv) {
  const Point3 pa = A.point(uA, vA);
  const Dir3 nA = A.normal(uA, vA);
  const ProjResult pr = projectOntoB(B, pa, nA, su, sv);
  bu = pr.u2;
  bv = pr.v2;
  return pr.residual <= onSurfTol && pr.normalSine <= tangentSinTol;
}

// Grow one A-domain interval endpoint toward `limit` by bisection until the agreement
// boundary is located, holding the OTHER axis fixed at (fixU,fixV). `along` picks the
// axis (0 = u, 1 = v). Returns the boundary coordinate; sets `hitDomainEdge` if agreement
// persisted all the way to `limit` (ambiguous → Undecided).
struct GrowCtx {
  const SurfaceAdapter& A;
  const SurfaceAdapter& B;
  double onSurfTol, tangentSinTol;
  double projSeedU, projSeedV;  // B-projection scan seed (a param on B, near the match)
};

double growEdge(const GrowCtx& g, int along, double fixU, double fixV,
                double start, double limit, bool& hitDomainEdge) {
  hitDomainEdge = false;
  auto agreesAt = [&](double coord) {
    const double uA = (along == 0) ? coord : fixU;
    const double vA = (along == 0) ? fixV : coord;
    double bu, bv;
    return sampleAgrees(g.A, g.B, uA, vA, g.onSurfTol, g.tangentSinTol,
                        g.projSeedU, g.projSeedV, bu, bv);
  };
  // Agreement right at the domain limit ⇒ the overlap is not interior-delimited (runs
  // into the domain edge, ambiguous). Report it (→ Undecided).
  if (agreesAt(limit)) { hitDomainEdge = true; return limit; }
  double lo = start, hi = limit;  // lo agrees, hi disagrees
  for (int it = 0; it < 40; ++it) {
    const double mid = 0.5 * (lo + hi);
    if (agreesAt(mid)) lo = mid; else hi = mid;
  }
  return 0.5 * (lo + hi);  // the agreement/disagreement boundary estimate
}

// Grow all four edges of an A-box, holding each grow's opposite axis fixed at the box
// CENTRE. Returns the grown box; sets `hitDomainEdge` if any side ran to a domain edge.
ParamBox growBox(const GrowCtx& g, const SurfaceAdapter& A, const ParamBox& seed,
                 bool& hitDomainEdge) {
  const double cu = seed.uMid(), cv = seed.vMid();
  bool e0 = false, e1 = false, e2 = false, e3 = false;
  ParamBox a;
  a.u1 = growEdge(g, 0, cu, cv, seed.u1, A.domain.u1, e1);
  a.u0 = growEdge(g, 0, cu, cv, seed.u0, A.domain.u0, e0);
  a.v1 = growEdge(g, 1, cu, cv, seed.v1, A.domain.v1, e3);
  a.v0 = growEdge(g, 1, cu, cv, seed.v0, A.domain.v0, e2);
  hitDomainEdge = e0 || e1 || e2 || e3;
  return a;
}

// Attempt to detect a coincident overlap patch seeded at candidate region `reg`. Returns
// a CoincidentRegion of kind OverlapSubRegion (delimited), Undecided (suspected, not
// robustly delimitable), or None (no coincident patch here).
CoincidentRegion detectOverlap(const SurfaceAdapter& A, const SurfaceAdapter& B,
                               const CandidateRegion& reg, double onSurfTol,
                               double tangentSinTol) {
  // (1) whole-interior-grid agreement over reg.a. A 3×3 interior grid: if any sample
  //     disagrees, this is not a coincident patch (fall through to normal seeding).
  const double su = reg.b.uMid(), sv = reg.b.vMid();
  for (int i = 1; i <= 3; ++i)
    for (int j = 1; j <= 3; ++j) {
      const double uA = reg.a.u0 + reg.a.du() * (i / 4.0);
      const double vA = reg.a.v0 + reg.a.dv() * (j / 4.0);
      double bu, bv;
      if (!sampleAgrees(A, B, uA, vA, onSurfTol, tangentSinTol, su, sv, bu, bv))
        return CoincidentRegion::none();
    }

  // (2) grow the A-box to the agreement boundary in TWO passes: pass 1 from the small
  //     candidate box gives a rough box; pass 2 re-grows each edge holding the OTHER axis
  //     at the rough box's CENTRE, so the boundary estimate is decoupled from the
  //     arbitrary candidate location (a corner-anchored first pass otherwise under-grows).
  const GrowCtx g{A, B, onSurfTol, tangentSinTol, reg.b.uMid(), reg.b.vMid()};
  bool edge1 = false, edge2 = false;
  const ParamBox rough = growBox(g, A, reg.a, edge1);
  const ParamBox a = growBox(g, A, rough, edge2);

  // (3) honesty: if agreement ran to a domain edge on ANY side (either pass), the overlap
  //     boundary is not interior-delimited (could be the true overlap edge, or just the
  //     trimmed domain). We CANNOT robustly delimit it → Undecided (→ OCCT), not guessed.
  if (edge1 || edge2) return CoincidentRegion::undecided();
  if (a.du() <= 0.0 || a.dv() <= 0.0) return CoincidentRegion::undecided();

  // (4) delimit the matching B-box: project a grid over the grown A-box onto B and take
  //     the param bounds of the projections. We INSET the grid off the box edges by 10%:
  //     the grown edges sit exactly on the agree/disagree boundary (grow returns the
  //     bracket midpoint), where a sample can fall just outside the overlap. Sampling the
  //     solid interior confirms the region and reads clean B bounds. A disagreement here
  //     means the agreement is not solid across the grown box → Undecided (honest).
  ParamBox b;
  bool first = true;
  for (int i = 0; i <= 4; ++i)
    for (int j = 0; j <= 4; ++j) {
      const double fu = 0.1 + 0.8 * (i / 4.0);  // inset ∈ [0.1, 0.9]
      const double fv = 0.1 + 0.8 * (j / 4.0);
      const double uA = a.u0 + a.du() * fu;
      const double vA = a.v0 + a.dv() * fv;
      double bu, bv;
      if (!sampleAgrees(A, B, uA, vA, onSurfTol, tangentSinTol, su, sv, bu, bv))
        return CoincidentRegion::undecided();  // agreement not solid across grown box
      if (first) { b.u0 = b.u1 = bu; b.v0 = b.v1 = bv; first = false; }
      else {
        b.u0 = std::min(b.u0, bu); b.u1 = std::max(b.u1, bu);
        b.v0 = std::min(b.v0, bv); b.v1 = std::max(b.v1, bv);
      }
    }
  return CoincidentRegion::overlap(a, b);
}

// Is candidate region `reg` covered by an already-delimited overlap on surface A? A
// candidate is covered iff its A-box INTERSECTS a recorded OverlapSubRegion (not merely
// centre-inside) — so the boundary-straddling candidates around one overlap are all
// suppressed by the first detection, and the same shared locus is not re-detected (or
// spuriously re-classified `Undecided`) once per candidate.
bool insideOverlap(const CandidateRegion& reg, const std::vector<CoincidentRegion>& regions) {
  const ParamBox& q = reg.a;
  for (const auto& cr : regions) {
    if (cr.kind != CoincidenceKind::OverlapSubRegion) continue;
    const ParamBox& r = cr.regionA;
    const bool disjoint = q.u1 < r.u0 || q.u0 > r.u1 || q.v1 < r.v0 || q.v0 > r.v1;
    if (!disjoint) return true;
  }
  return false;
}

// ── S4-b: seeded tangent-contact classification (differential geometry) ─────────
//
// At a near-tangent refined solution (‖n_A × n_B‖ < tangentSinTol) the intersection is
// degenerate — S2 previously just did `++deferredTangent`. S4-b types WHAT it is by the
// RELATIVE SECOND FUNDAMENTAL FORM of the two surfaces in their shared tangent plane.
//
// Around the contact, each surface is a graph height_S(x,y) over the shared tangent plane
// (basis e1,e2 ⟂ the shared normal n), with height_S = ½·II_S(x,y) + O(³) (first order
// vanishes — the plane is tangent to BOTH). The GAP h(x,y) = height_A − height_B is then
// ½·(II_A − II_B) to leading order: a 2×2 symmetric quadratic form H whose eigenstructure
// classifies the contact:
//   * H sign-definite (both eigenvalues same sign, both |λ| above the noise floor)
//     → the surfaces separate on all sides → ISOLATED TangentPoint.
//   * H rank-1 (one eigenvalue ≈ 0, the other above the floor) → the gap is flat along one
//     tangent direction → the surfaces stay in contact ALONG A CURVE → TangentCurve.
//   * H indefinite (eigenvalues of OPPOSITE sign, both above the floor) → the gap changes
//     sign around the contact → the surfaces GRAZE AND CROSS → NearTangentTransversal
//     (the S4-c gap: handed on, NEVER traced through here).
//   * both eigenvalues within the curvature-noise band → the jet is not robust → Undecided
//     (→ OCCT). Honest: the native layer does not guess a definite/rank-1/indefinite call
//     it cannot support.
//
// h(x,y) is sampled by projecting probe points P + (x·e1 + y·e2) onto EACH surface (native
// closest_point_on_surface) and taking the signed height difference along n. The Hessian is
// read by central finite differences on a small stencil at a curvature-resolving step. This
// is OCCT-free (projection is native-numerics) and returns Undecided rather than a fabricated
// verdict when the samples are too noisy to resolve.

// A unit vector ⟂ `n` (completes a tangent-plane basis). Picks the world axis least aligned
// with n to avoid a near-null cross product.
Dir3 tangentBasis1(const Dir3& n) noexcept {
  const double ax = std::fabs(n.x()), ay = std::fabs(n.y()), az = std::fabs(n.z());
  const Vec3 pick = (ax <= ay && ax <= az) ? Vec3{1, 0, 0}
                                           : (ay <= az) ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
  return Dir3{math::cross(n.vec(), pick)};
}

// Signed height of surface S above the tangent plane {P; n} at the tangent-plane probe
// offset (x·e1 + y·e2): project the probe onto S and dot the displacement with n. `su,sv`
// seed the projection near the contact.
double surfaceHeight(const SurfaceAdapter& S, const Point3& P, const Dir3& n,
                     const Dir3& e1, const Dir3& e2, double x, double y,
                     double su, double sv) {
  const Point3 probe = P + e1.vec() * x + e2.vec() * y;
  const ParamBox& d = S.domain;
  const nn::SurfaceProjection pr = nn::closest_point_on_surface(
      [&](double u, double v) { return S.point(u, v); },
      d.u0, d.u1, d.v0, d.v1, probe, 12, 12);
  (void)su; (void)sv;
  return math::dot(pr.point - P, n.vec());
}

// Classify a near-tangent contact at the seed (params on A/B, base point, shared normal
// nA and the crossing sine already measured). Returns the typed TangentContact.
TangentContact classifyTangentContact(const SurfaceAdapter& A, const SurfaceAdapter& B,
                                      double uA, double vA, double uB, double vB,
                                      const Point3& P, const Dir3& nA, const Dir3& nB,
                                      double crossingSine, double scale) {
  // Shared tangent-plane basis from A's normal (nA and nB are ~parallel here).
  const Dir3 e1 = tangentBasis1(nA);
  const Dir3 e2{math::cross(nA.vec(), e1.vec())};

  // Finite-difference step: a curvature-resolving fraction of model scale. Too small and
  // projection noise dominates; too large and higher-order terms leak in. 1/64 of scale
  // matches the marcher's default step band.
  const double hStep = std::max(scale * (1.0 / 64.0), 1e-9);

  // Relative-height gap h(x,y) = height_A(x,y) − height_B(x,y), both measured along nA.
  auto gap = [&](double x, double y) {
    return surfaceHeight(A, P, nA, e1, e2, x, y, uA, vA) -
           surfaceHeight(B, P, nA, e1, e2, x, y, uB, vB);
  };

  // Central second differences → the symmetric Hessian H = [[hxx, hxy],[hxy, hyy]] of the
  // gap at the origin (the leading-order relative second fundamental form).
  const double h00 = gap(0.0, 0.0);
  const double hpx = gap(+hStep, 0.0), hmx = gap(-hStep, 0.0);
  const double hpy = gap(0.0, +hStep), hmy = gap(0.0, -hStep);
  const double hpp = gap(+hStep, +hStep), hmm = gap(-hStep, -hStep);
  const double hpm = gap(+hStep, -hStep), hmp = gap(-hStep, +hStep);
  const double inv = 1.0 / (hStep * hStep);
  const double hxx = (hpx - 2.0 * h00 + hmx) * inv;
  const double hyy = (hpy - 2.0 * h00 + hmy) * inv;
  const double hxy = (hpp - hpm - hmp + hmm) * (0.25 * inv);

  // Eigenvalues of the symmetric 2×2 form. Order them by MAGNITUDE: `big` is the dominant
  // principal curvature-difference, `small` the other — the classification looks at whether
  // `small` is negligible (rank-1) and, if both resolve, at their sign agreement.
  const double tr = hxx + hyy;
  const double det = hxx * hyy - hxy * hxy;
  const double rad = std::sqrt(std::max(0.0, 0.25 * tr * tr - det));
  const double lA = 0.5 * tr + rad;
  const double lB = 0.5 * tr - rad;
  const double big   = std::fabs(lA) >= std::fabs(lB) ? lA : lB;
  const double small = std::fabs(lA) >= std::fabs(lB) ? lB : lA;
  const double amax = std::fabs(big);

  // Consistent normal orientation: nA and nB may be parallel OR antiparallel. The height
  // gap's sign convention only flips the OVERALL sign of H (which we do not use — we look at
  // eigenvalue-sign AGREEMENT/OPPOSITION and negligibility), so no reorientation is needed.
  (void)nB;

  // Curvature-noise floor: an eigenvalue is a curvature difference (~1/length). If even the
  // DOMINANT eigenvalue is below a small fraction of 1/scale, the relative second form is
  // flat to our stencil's resolution — the point/curve/cross call is not robust → Undecided
  // (honest → OCCT), never a guessed verdict.
  const double absFloor = (1.0 / std::max(scale, 1e-12)) * 1e-4;
  if (amax <= absFloor)
    return TangentContact::undecided(P, crossingSine);

  // Rank test: the SMALLER-magnitude eigenvalue below 1% of the dominant one reads as a true
  // zero → the gap is flat along that principal direction → the surfaces stay in contact
  // ALONG A CURVE (rank-1 relative second form).
  const double relFloor = amax * 1e-2;
  if (std::fabs(small) <= relFloor)
    return TangentContact::tangentCurveAt(P, crossingSine);   // rank-1: tangent along a curve

  // Both eigenvalues resolved and non-zero: same sign → the gap keeps one sign all around →
  // the surfaces separate → ISOLATED point; opposite signs → the gap changes sign → the
  // surfaces graze AND cross → NearTangentTransversal (S4-c gap, handed on, NOT traced here).
  if ((big > 0.0) == (small > 0.0))
    return TangentContact::tangentPoint(P, crossingSine);     // sign-definite: isolated touch
  return TangentContact::nearTangentTransversal(P, crossingSine);  // indefinite: grazes+crosses
}

struct DisjointSet {
  std::vector<int> parent;
  explicit DisjointSet(int n) : parent(n) { for (int i = 0; i < n; ++i) parent[i] = i; }
  int find(int x) { while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; } return x; }
  void unite(int a, int b) { parent[find(a)] = find(b); }
};

// Do two 1D intervals [a0,a1], [b0,b1] touch/overlap within `eps`? If `period > 0`
// the axis is periodic (e.g. angular), so also test the wrapped copies of `b` at
// ±period — a box near the high end of the domain is adjacent to one near the low
// end across the seam. Non-periodic axes (period == 0) test only the direct gap.
inline bool intervalsAdjacent(double a0, double a1, double b0, double b1,
                              double eps, double period) noexcept {
  auto touch = [&](double lo, double hi) {
    return a0 - hi <= eps && lo - a1 <= eps;
  };
  if (touch(b0, b1)) return true;
  if (period > 0.0)
    return touch(b0 + period, b1 + period) || touch(b0 - period, b1 - period);
  return false;
}

// Two param boxes are adjacent iff both param axes are adjacent (periodic-aware).
inline bool paramBoxesAdjacent(const ParamBox& a, const ParamBox& b,
                               double epsU, double epsV,
                               double uPeriod, double vPeriod) noexcept {
  return intervalsAdjacent(a.u0, a.u1, b.u0, b.u1, epsU, uPeriod) &&
         intervalsAdjacent(a.v0, a.v1, b.v0, b.v1, epsV, vPeriod);
}

// ── cluster candidate regions → connected branches (TOPOLOGICAL, pre-refine) ─────
//
// A branch is a 1D locus: candidate regions along it tile CONTIGUOUSLY in parameter
// space, so we cluster regions that are ADJACENT (param boxes touch/overlap, periodic
// seam aware) on BOTH surfaces. Regions along one branch form one connected component;
// a distinct branch is a separate param-space component (its regions don't touch the
// first's). This is scale-free — no 3D radius to tune — and immune to the along-branch
// "sparse stretch" that breaks a metric 3D chain. Clustering BEFORE the refine also
// makes the expensive least_squares run once per branch (per cluster representative),
// not once per candidate region — the key performance decision. Deterministic
// (input-order-stable union-find). Returns, for each region, its 0..K-1 cluster id;
// `numClusters` is set to K.
struct DedupParams {
  double epsUA, epsVA, epsUB, epsVB;    // per-surface leaf-scaled param slack
  double uPerA, vPerA, uPerB, vPerB;    // per-surface param periods (0 = non-periodic)
};

// SPATIAL-HASHED 4D ADJACENCY (near-linear, not O(n²)) — the same connected components the
// all-pairs loop below computes, over the SAME predicate, at a cost that does not explode on a
// 2D shared locus.
//
// WHY. The adjacency relation is a conjunction of two 2D box-adjacency tests, i.e. an overlap of
// eps-expanded boxes in the 4D product (uA,vA,uB,vB). On a TRANSVERSAL pair the candidate count
// tracks a 1D curve and the all-pairs loop is affordable. On a COINCIDENT / overlapping pair the
// locus is 2D: no leaf pair anywhere is AABB-disjoint, so the candidate pile grows ~4.2× per
// halving and the quadratic loop becomes the entire wall clock — measured as a process spinning
// with resident memory FLAT for 17 minutes, i.e. allocation long finished. Handed two coincident
// freeform surfaces the seeder therefore did not decline, it HUNG (6/6 constructed pairs killed
// at 1200 s with no output, including a genuinely DISJOINT pair whose answer is the empty set).
//
// HOW IT STAYS EXACT. The grid is a CANDIDATE FILTER ONLY — every surviving pair is still decided
// by the unchanged `paramBoxesAdjacent` conjunction, so the grid can over-generate freely and only
// has to avoid MISSING a true pair. Cells are sized `extent + 2·eps` per axis, so an eps-expanded
// interval spans at most two cells per axis (≤ 16 cells in 4D): two boxes that satisfy the
// predicate share a point that both expanded intervals contain, hence share a cell. Periodic axes
// index modulo the cell count, and because the interval is expanded BEFORE indexing, a box at the
// low end wraps into the high-end cell — so seam-adjacent pairs co-occur exactly as the ±period
// tests in the predicate intend.
//
// LABELS ARE UNCHANGED. `rootLabel` assigns ids by ASCENDING first appearance, so a label depends
// only on the partition and the input order — never on the order unions happened in. Same
// partition ⇒ same labels ⇒ same cluster ids ⇒ same seeds. This mirrors `linkBySep` below, which
// already replaced an O(m²) linkage with a grid hash for the 3D seed split.
struct CellKey4 {
  std::int32_t c[4];
  bool operator==(const CellKey4& o) const noexcept {
    return c[0] == o.c[0] && c[1] == o.c[1] && c[2] == o.c[2] && c[3] == o.c[3];
  }
};
struct CellKey4Hash {
  std::size_t operator()(const CellKey4& k) const noexcept {
    // Deterministic 64-bit mix of the four cell indices (order-independent, stable across runs).
    std::uint64_t h = 1469598103934665603ull;
    for (int i = 0; i < 4; ++i) {
      h ^= static_cast<std::uint64_t>(static_cast<std::uint32_t>(k.c[i]));
      h *= 1099511628211ull;
    }
    return static_cast<std::size_t>(h);
  }
};

// Below this candidate count the all-pairs loop is cheaper than building the hash, and it is the
// path every transversal pose in the corpus takes — so the common case stays byte-identical with
// zero new allocation.
constexpr int kClusterHashMinN = 512;

std::vector<int> clusterRegions(const std::vector<CandidateRegion>& regs,
                                const DedupParams& p, int& numClusters) {
  const int n = static_cast<int>(regs.size());
  std::vector<int> label(n, -1);
  numClusters = 0;
  if (n == 0) return label;
  DisjointSet ds(n);

  const double eps[4]    = {p.epsUA, p.epsVA, p.epsUB, p.epsVB};
  const double period[4] = {p.uPerA, p.vPerA, p.uPerB, p.vPerB};
  // Per-axis interval of a candidate in the 4D product space.
  auto axisLo = [&](int r, int ax) noexcept {
    return ax == 0 ? regs[r].a.u0 : ax == 1 ? regs[r].a.v0 : ax == 2 ? regs[r].b.u0 : regs[r].b.v0;
  };
  auto axisHi = [&](int r, int ax) noexcept {
    return ax == 0 ? regs[r].a.u1 : ax == 1 ? regs[r].a.v1 : ax == 2 ? regs[r].b.u1 : regs[r].b.v1;
  };
  // The exact predicate — the ONLY thing that decides adjacency, on either path.
  auto adjacent = [&](int i, int j) noexcept {
    return paramBoxesAdjacent(regs[i].a, regs[j].a, p.epsUA, p.epsVA, p.uPerA, p.vPerA) &&
           paramBoxesAdjacent(regs[i].b, regs[j].b, p.epsUB, p.epsVB, p.uPerB, p.vPerB);
  };

  if (n < kClusterHashMinN) {
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j)
        if (adjacent(i, j)) ds.unite(i, j);
  } else {
    // Cell size per axis: widest candidate extent + 2·eps ⇒ an expanded interval spans ≤ 2 cells.
    double cell[4];
    for (int ax = 0; ax < 4; ++ax) {
      double maxExtent = 0.0;
      for (int r = 0; r < n; ++r) maxExtent = std::max(maxExtent, axisHi(r, ax) - axisLo(r, ax));
      cell[ax] = maxExtent + 2.0 * std::max(eps[ax], 0.0);
      if (!(cell[ax] > 0.0)) cell[ax] = 1.0;  // fully degenerate axis → one cell (still exact)
    }
    // Cell count on a periodic axis, for modular indexing. Below 2 cells the axis cannot
    // discriminate, so it collapses to a single cell — over-generating, never missing.
    std::int32_t cyc[4];
    for (int ax = 0; ax < 4; ++ax) {
      cyc[ax] = 0;
      if (period[ax] > 0.0) {
        const double k = std::floor(period[ax] / cell[ax]);
        cyc[ax] = (k >= 2.0 && k < 1e9) ? static_cast<std::int32_t>(k) : 1;
      }
    }
    auto wrap = [&](std::int32_t idx, int ax) noexcept -> std::int32_t {
      if (cyc[ax] <= 0) return idx;                  // non-periodic: index as-is
      std::int32_t m = idx % cyc[ax];
      return m < 0 ? m + cyc[ax] : m;                // periodic: fold into [0, cyc)
    };

    std::unordered_map<CellKey4, std::vector<int>, CellKey4Hash> grid;
    grid.reserve(static_cast<std::size_t>(n) * 2);
    std::int32_t lo[4], hi[4];
    for (int r = 0; r < n; ++r) {
      for (int ax = 0; ax < 4; ++ax) {
        const double e = std::max(eps[ax], 0.0);
        lo[ax] = static_cast<std::int32_t>(std::floor((axisLo(r, ax) - e) / cell[ax]));
        hi[ax] = static_cast<std::int32_t>(std::floor((axisHi(r, ax) + e) / cell[ax]));
        if (hi[ax] < lo[ax]) hi[ax] = lo[ax];
      }
      CellKey4 key{};
      for (std::int32_t i0 = lo[0]; i0 <= hi[0]; ++i0)
        for (std::int32_t i1 = lo[1]; i1 <= hi[1]; ++i1)
          for (std::int32_t i2 = lo[2]; i2 <= hi[2]; ++i2)
            for (std::int32_t i3 = lo[3]; i3 <= hi[3]; ++i3) {
              key.c[0] = wrap(i0, 0); key.c[1] = wrap(i1, 1);
              key.c[2] = wrap(i2, 2); key.c[3] = wrap(i3, 3);
              grid[key].push_back(r);
            }
    }
    // Every truly-adjacent pair shares at least one cell, so testing within cells is complete.
    // A pair may recur in several cells; `unite` is idempotent and the exact test is cheap.
    for (const auto& kv : grid) {
      const std::vector<int>& bucket = kv.second;
      const std::size_t m = bucket.size();
      for (std::size_t x = 0; x < m; ++x)
        for (std::size_t y = x + 1; y < m; ++y) {
          const int i = bucket[x], j = bucket[y];
          if (ds.find(i) != ds.find(j) && adjacent(i, j)) ds.unite(i, j);
        }
    }
  }
  std::vector<int> rootLabel(n, -1);
  for (int i = 0; i < n; ++i) {
    const int root = ds.find(i);
    if (rootLabel[root] < 0) rootLabel[root] = numClusters++;
    label[i] = rootLabel[root];
  }
  return label;
}

// Refine the representative seed(s) of each branch cluster and append the accepted seeds
// (and deferred-tangent count) to `out`. For each cluster, refine its candidate regions;
// with DISTINCT-BRANCH SPLIT (default on), emit one seed per SPATIALLY-DISTINCT 3D locus
// the cluster hosts (a merged two-loop cluster → a seed per loop) instead of only the
// single tightest — recovering co-resident loops the param-box adjacency clustering merged.
// A cluster that only ever ill-conditions (near-tangent at the solution) becomes a
// deferred-to-S4 gap — S4-b then TYPES that gap by the local differential geometry
// (TangentPoint / TangentCurve / NearTangentTransversal / Undecided) instead of the
// blunt counter (which is kept as a compatibility summary). A cluster where no region
// converges to a crossing is a refine miss (dropped, never faked). Running the expensive
// least_squares ≈ once per branch (not once per candidate region) is the key perf choice.
//
// Per-cluster accumulators for the refine pass. `xversal[cid]` holds every accepted
// transversal seed of cluster `cid` (bounded); a post-pass single-linkage groups them into
// 3D-connected LOCUS components and emits the tightest per component. When the split is off,
// only the single tightest transversal seed per cluster is retained (old behaviour).
struct ClusterAcc {
  std::vector<std::vector<Seed>> xversal;    // accepted transversal seeds per cluster
  std::vector<char> haveSeed, sawTangent;
  std::vector<NearTangentSolution> ntBest;   // flattest near-tangent solution per cluster
  explicit ClusterAcc(std::size_t k)
      : xversal(k), haveSeed(k, 0), sawTangent(k, 0), ntBest(k) {}
};

// Hard SAFETY ceiling on transversal seeds ACCUMULATED per cluster before decimation. This is a
// pure MEMORY guard (an intersection field this dense on one cluster is pathological) — the
// RECALL-relevant retention is now `opts.capRetentionBudget` applied LOOP-AWARE (retainLoopAware)
// AFTER the whole pile is accumulated, not a FIFO drop DURING accumulation. The original bug was a
// FLAT-FIFO cap so small (256, then 65 536) that one dense locus's leaves filled it in
// candidate-iteration order and DROPPED a co-resident locus's later leaves before the
// distinct-locus split ever saw them (measured: 272 847 / 215 834 candidates on ONE cluster, the
// second/third locus cap-starved → the split emitted one seed for a twice-piercing pose). The fix:
// (1) accumulate the pile up to this generous ceiling (measured worst case ≈ 273 k ≪ this, so a
// real field is never FIFO-truncated); (2) partition it into distinct co-resident SUB-LOCI FIRST
// and retain a per-sub-locus UNIFORM STRIDE (retainLoopAware) so EACH locus keeps full arc coverage
// and BOTH survive to the split; (3) the split is O(m) via a uniform 3D grid hash (linkBySep) so
// full density is affordable. FIFO drop at this ceiling remains only as the ultimate memory bound.
// DISAGREED-safe: every retained seed is a real refined on-both-surfaces transversal crossing;
// retention only changes WHICH real seeds reach the split, never fabricates a locus.
constexpr std::size_t kMaxSeedsPerCluster = 1u << 20;  // 1 048 576 — pure memory guard, not the recall knob

// Fold one refined region into its cluster's accumulator: append the transversal seed at FULL
// density (for the distinct-locus split), else keep the flattest (most degenerate) near-tangent
// solution for S4-b typing. With the split OFF, only the single tightest transversal seed is
// retained (byte-identical to the pre-split path).
void accumulateRegion(const SurfaceAdapter& A, const SurfaceAdapter& B,
                      const CandidateRegion& reg, const SeedOptions& opts, double onSurfTol,
                      bool doSplit, int cid, ClusterAcc& acc) {
  Seed s;
  bool nearTangent = false;
  NearTangentSolution ntSol;
  if (refineRegion(A, B, reg, opts, onSurfTol, s, nearTangent, ntSol)) {
    auto& xs = acc.xversal[cid];
    acc.haveSeed[cid] = 1;
    if (!doSplit) {
      // Split OFF (elementary-operand pair): keep ONLY the running tightest transversal seed
      // of the cluster — byte-identical to the pre-split behaviour (no cap truncation risk).
      if (xs.empty()) xs.push_back(s);
      else if (s.onSurfResidual < xs.front().onSurfResidual) xs.front() = s;
      return;
    }
    if (xs.size() < kMaxSeedsPerCluster) xs.push_back(s);
  } else if (nearTangent) {
    if (!acc.sawTangent[cid] || ntSol.crossingSine < acc.ntBest[cid].crossingSine)
      acc.ntBest[cid] = ntSol;
    acc.sawTangent[cid] = 1;
  }
}

// SPATIAL-HASHED 3D SINGLE-LINKAGE (O(m), not O(m²)) — the shared connected-components
// primitive for BOTH the distinct-locus split and the loop-aware cap retention. Two seeds
// are in the same component iff a chain connects them with every hop ≤ `sep`. Seeds are
// binned into a uniform 3D grid of cell size `sep`; each seed is only unite-tested against
// seeds in its own + 26 neighbouring cells — every pair within `sep` shares such a
// neighbourhood, so the linkage is EXACT while the cost is linear. Fills `ds` (deterministic,
// integer cell keys, stable order) — the caller derives component labels/reps from `ds.find`.
void linkBySep(const std::vector<Seed>& seeds, double sep, DisjointSet& ds) {
  const std::size_t m = seeds.size();
  const double sep2 = sep * sep;
  const double inv = 1.0 / (sep > 0.0 ? sep : 1.0);
  // Integer cell key per seed (floor(coord/sep)); bucket seed indices by key in a hash map.
  struct Key { long x, y, z; bool operator==(const Key& o) const { return x==o.x && y==o.y && z==o.z; } };
  struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept {
      // 64-bit mix of the three cell indices (deterministic, order-independent).
      auto h = static_cast<std::uint64_t>(k.x * 73856093L) ^
               static_cast<std::uint64_t>(k.y * 19349663L) ^
               static_cast<std::uint64_t>(k.z * 83492791L);
      return static_cast<std::size_t>(h);
    }
  };
  auto cellOf = [&](const Seed& s) -> Key {
    return Key{static_cast<long>(std::floor(s.point.x * inv)),
               static_cast<long>(std::floor(s.point.y * inv)),
               static_cast<long>(std::floor(s.point.z * inv))};
  };
  std::unordered_map<Key, std::vector<int>, KeyHash> grid;
  grid.reserve(m * 2);
  for (std::size_t i = 0; i < m; ++i) grid[cellOf(seeds[i])].push_back(static_cast<int>(i));
  // Unite each seed with neighbours in its own + 26 adjacent cells that are within `sep`.
  for (std::size_t i = 0; i < m; ++i) {
    const Key c = cellOf(seeds[i]);
    for (long dx = -1; dx <= 1; ++dx)
      for (long dy = -1; dy <= 1; ++dy)
        for (long dz = -1; dz <= 1; ++dz) {
          const auto it = grid.find(Key{c.x + dx, c.y + dy, c.z + dz});
          if (it == grid.end()) continue;
          for (const int j : it->second)
            if (static_cast<std::size_t>(j) > i &&
                math::normSquared(seeds[i].point - seeds[static_cast<std::size_t>(j)].point) <= sep2)
              ds.unite(static_cast<int>(i), j);
        }
  }
}

// DIAGNOSTIC ONLY (env-gated, no behaviour change): count distinct 3D single-linkage
// components in a seed pile at separation `sep`, and the max pairwise 3D extent. Used by the
// SEED-DIAG dump to reveal whether a capped/merged cluster hosts ONE physical locus or several
// co-resident loci the split should separate. Not on any hot path when diag is off.
int countComponentsAtSep(const std::vector<Seed>& seeds, double sep) {
  const std::size_t m = seeds.size();
  if (m == 0) return 0;
  DisjointSet ds(static_cast<int>(m));
  linkBySep(seeds, sep, ds);
  int n = 0;
  std::vector<char> seen(m, 0);
  for (std::size_t i = 0; i < m; ++i) {
    const int r = ds.find(static_cast<int>(i));
    if (!seen[static_cast<std::size_t>(r)]) { seen[static_cast<std::size_t>(r)] = 1; ++n; }
  }
  return n;
}

// LOOP-STRUCTURE-AWARE CAP RETENTION. When a cluster's refined pile exceeds `budget`, decimate
// it to ≤ `budget` seeds WITHOUT starving any co-resident locus. Plain (flat) thinning of the
// whole merged pile was REJECTED (E1): a uniform stride across two loci leaves ARC GAPS that
// spuriously over-split a SINGLE loop. The fix partitions FIRST, strides SECOND:
//   1. PARTITION the pile into distinct co-resident SUB-LOCI — the SAME 3D single-linkage
//      components (at the split separation `sep`) the distinct-locus split will use, so a
//      retained seed set maps EXACTLY onto the split's future components.
//   2. Give each sub-locus a share of the budget proportional to its size (so the dense locus
//      is thinned MORE and the sparse co-resident locus is thinned LESS — never starved), with
//      a floor of `kMinRetainedPerLocus` so a small locus always survives with enough leaves to
//      stay connected under the subsequent split.
//   3. Retain a UNIFORM STRIDE within each sub-locus (evenly-spaced leaves in append order — the
//      candidate-iteration order tiles the arc, so a stride keeps FULL arc coverage with no gap),
//      and force-keep each locus's TIGHTEST seed (the split's representative is unaffected).
// The stride is applied WITHIN a connected locus, never across the gap between two loci, so it
// preserves the dense chain that keeps one physical loop connected (no over-split) while
// guaranteeing BOTH co-resident loci reach the split. DISAGREED-safe / recall-only: every
// retained seed is a real refined on-both-surfaces transversal crossing; retention only changes
// WHICH real seeds reach the split, never a tolerance nor a fabricated locus. Deterministic.
constexpr std::size_t kMinRetainedPerLocus = 256;  ///< floor so a small co-resident locus is never starved

void retainLoopAware(std::vector<Seed>& seeds, double sep, std::size_t budget) {
  const std::size_t m = seeds.size();
  if (m <= budget) return;  // no decimation needed — byte-identical
  DisjointSet ds(static_cast<int>(m));
  linkBySep(seeds, sep, ds);
  // Group seed indices by connected component (append order preserved within each component,
  // so a per-component stride keeps evenly-spaced leaves along the arc).
  std::vector<int> compOf(m, -1);
  std::vector<std::vector<int>> comps;
  for (std::size_t i = 0; i < m; ++i) {
    const int root = ds.find(static_cast<int>(i));
    if (compOf[static_cast<std::size_t>(root)] < 0) {
      compOf[static_cast<std::size_t>(root)] = static_cast<int>(comps.size());
      comps.emplace_back();
    }
    comps[static_cast<std::size_t>(compOf[static_cast<std::size_t>(root)])].push_back(static_cast<int>(i));
  }
  // Per-locus budget: proportional to size (dense locus thinned more), floored so a small
  // co-resident locus keeps enough leaves to stay connected under the split. The floor total
  // can exceed `budget` when there are many components; that is acceptable — the guarantee is
  // "no locus starved", and the hard memory ceiling still bounds the pile (kMaxSeedsPerCluster).
  const std::size_t nComp = comps.size();
  std::vector<Seed> kept;
  kept.reserve(budget + nComp * kMinRetainedPerLocus);
  for (const auto& comp : comps) {
    const std::size_t cm = comp.size();
    // Proportional share of the budget for this locus, floored.
    std::size_t share = static_cast<std::size_t>(
        (static_cast<double>(cm) / static_cast<double>(m)) * static_cast<double>(budget));
    if (share < kMinRetainedPerLocus) share = kMinRetainedPerLocus;
    if (share >= cm) {  // keep the whole locus (already under its share)
      for (const int idx : comp) kept.push_back(seeds[static_cast<std::size_t>(idx)]);
      continue;
    }
    // Uniform stride over the locus's seeds (append order = arc order), plus its tightest seed.
    int tightest = comp[0];
    for (const int idx : comp)
      if (seeds[static_cast<std::size_t>(idx)].onSurfResidual <
          seeds[static_cast<std::size_t>(tightest)].onSurfResidual)
        tightest = idx;
    const double step = static_cast<double>(cm) / static_cast<double>(share);
    bool tightestKept = false;
    for (std::size_t k = 0; k < share; ++k) {
      const std::size_t pick = static_cast<std::size_t>(static_cast<double>(k) * step);
      const int idx = comp[std::min(pick, cm - 1)];
      if (idx == tightest) tightestKept = true;
      kept.push_back(seeds[static_cast<std::size_t>(idx)]);
    }
    if (!tightestKept) kept.push_back(seeds[static_cast<std::size_t>(tightest)]);
  }
  seeds.swap(kept);
}

// Split a cluster's accepted transversal seeds into SPATIALLY-DISTINCT 3D loci by
// single-linkage on the 3D seed points (see linkBySep): a single physical loop's refined
// points tile it densely and chain into ONE component (the cluster collapses to one seed);
// two co-resident loops separated by more than `sep` form TWO components → a seed per loop.
// Emits the TIGHTEST seed of each component into `outSeeds`, deterministic in seed-append
// order, capped at `maxOut`. An over-split (same loop, two components) only re-traces the loop
// — the S3 marcher's per-branch locus-dedup collapses it, so this is recall-only.
void splitDistinctLoci(const std::vector<Seed>& seeds, double sep, std::size_t maxOut,
                       std::vector<Seed>& outSeeds) {
  const std::size_t m = seeds.size();
  if (m == 0) return;
  DisjointSet ds(static_cast<int>(m));
  linkBySep(seeds, sep, ds);
  // Tightest seed per connected component, in first-appearance order of the component root.
  std::vector<int> compOf(m, -1);
  std::vector<int> order;          // component-id → index of its current tightest seed
  int nComp = 0;
  for (std::size_t i = 0; i < m; ++i) {
    const int root = ds.find(static_cast<int>(i));
    if (compOf[static_cast<std::size_t>(root)] < 0) {
      compOf[static_cast<std::size_t>(root)] = nComp++;
      order.push_back(static_cast<int>(i));
    } else {
      int& rep = order[static_cast<std::size_t>(compOf[static_cast<std::size_t>(root)])];
      if (seeds[i].onSurfResidual < seeds[static_cast<std::size_t>(rep)].onSurfResidual)
        rep = static_cast<int>(i);
    }
  }
  for (std::size_t c = 0; c < order.size() && outSeeds.size() < maxOut; ++c)
    outSeeds.push_back(seeds[static_cast<std::size_t>(order[c])]);
}

void refineClusters(const SurfaceAdapter& A, const SurfaceAdapter& B,
                    const std::vector<CandidateRegion>& regs,
                    const std::vector<int>& cluster, int numClusters,
                    const SeedOptions& opts, double onSurfTol, double scale,
                    const std::vector<char>& suppressed, SeedSet& out) {
  // FREEFORM↔FREEFORM GATE. The distinct-branch split fires ONLY when BOTH operands are
  // freeform (control-net-bearing → freeformSpanCount ≥ 1) — the general NURBS↔NURBS L2
  // domain where the param-box adjacency clustering merges close co-resident loops. Any pair
  // with an ELEMENTARY / plane / torus operand (span count 0) is left BYTE-IDENTICAL: each
  // cluster still collapses to its single tightest seed (the old behaviour, `accumulateRegion`
  // keeps only the running tightest), so every canonical / S4-f elementary-operand seed-count
  // contract is untouched. Same gate the scale-adaptive initial seeding uses, for the same reason.
  const bool bothFreeform = A.freeformSpanCount >= 1 && B.freeformSpanCount >= 1;
  const bool doSplit = opts.splitDistinctBranches && bothFreeform;

  // Distinct-locus separation: two refined seeds beyond this 3D distance are on distinct loci.
  // Scale-relative + deterministic; sized so one physical loop never splits (its refined
  // points are ≪ this apart) yet two co-resident loops (typically an operand-scale apart) do.
  const double sep = std::max(scale * opts.splitDistinctFrac, onSurfTol * 4.0);

  ClusterAcc acc(static_cast<std::size_t>(std::max(0, numClusters)));
  for (std::size_t i = 0; i < regs.size(); ++i) {
    const int cid = cluster[i];
    if (cid < 0) continue;
    if (!suppressed.empty() && suppressed[cid]) continue;  // S4-a coincident cluster
    accumulateRegion(A, B, regs[i], opts, onSurfTol, doSplit, cid, acc);
  }
  const std::size_t maxPer = doSplit
                                 ? static_cast<std::size_t>(std::max(1, opts.splitMaxPerCluster))
                                 : 1u;
  // DIAGNOSTIC (env-gated, OCCT-free): dump per-cluster accumulation so a declined
  // co-resident case reveals WHERE the second locus was lost — was a distinct-locus
  // component ever refined (split-merge problem), or was the cluster left with a single
  // spatial component (candidate never produced / never converged there)? Off by default;
  // no behaviour change.
  const bool diag = std::getenv("CYBERCAD_SSI_SEED_DIAG") != nullptr;

  // Loop-aware cap-retention budget: only decimates a pile larger than this, and only on the
  // freeform split path (elementary/mixed keep a single tightest seed, never a large pile).
  const std::size_t retainBudget =
      static_cast<std::size_t>(std::max(1, opts.capRetentionBudget));

  int branchId = 0;
  for (int cid = 0; cid < numClusters; ++cid) {
    if (acc.haveSeed[cid]) {
      std::vector<Seed> emit;
      // When the gate is off, accumulateRegion retained exactly ONE (tightest) seed per cluster,
      // so this splits into one component and emits it — byte-identical to the pre-split path.
      const double effSep = doSplit ? sep : std::numeric_limits<double>::infinity();
      auto& pile = acc.xversal[static_cast<std::size_t>(cid)];
      const std::size_t pileRaw = pile.size();
      // LOOP-AWARE CAP RETENTION (split path only): when the pile exceeds the budget, partition
      // it into distinct co-resident sub-loci and retain a per-sub-locus uniform stride FIRST, so
      // no locus is FIFO-starved before the split sees it. No-op when the pile is under budget.
      // DIAGNOSTIC (env-gated): probe the RAW pile's 3D component structure BEFORE retention at
      // several separation scales, so a merged/capped cluster reveals whether it hosts one
      // physical locus or co-resident loci the split at `sep` cannot see. No behaviour change.
      int compRaw = 0, compHalf = 0, compQtr = 0, compEps = 0;
      if (diag) {
        // Subsample large piles (uniform stride) so the O(m) component probe stays cheap on
        // ultra-dense clusters; a stride preserves distinct-locus separation for the count.
        std::vector<Seed> probe;
        const std::size_t kProbeCap = 20000;
        if (pile.size() > kProbeCap) {
          const double st = static_cast<double>(pile.size()) / static_cast<double>(kProbeCap);
          probe.reserve(kProbeCap);
          for (std::size_t k = 0; k < kProbeCap; ++k)
            probe.push_back(pile[std::min(pile.size() - 1, static_cast<std::size_t>(k * st))]);
        } else {
          probe = pile;
        }
        compRaw  = countComponentsAtSep(probe, sep);
        compHalf = countComponentsAtSep(probe, sep * 0.5);
        compQtr  = countComponentsAtSep(probe, sep * 0.25);
        compEps  = countComponentsAtSep(probe, std::max(onSurfTol * 8.0, sep * 0.05));
      }
      if (doSplit) retainLoopAware(pile, sep, retainBudget);
      splitDistinctLoci(pile, effSep, maxPer, emit);
      if (diag)
        std::fprintf(stderr,
            "[SEED-DIAG] cid=%d xversalSeeds=%zu retained=%zu emitted=%zu sep=%.4e doSplit=%d scale=%.4e "
            "| rawComps@sep=%d @sep/2=%d @sep/4=%d @~eps=%d\n",
            cid, pileRaw, pile.size(), emit.size(), sep, doSplit ? 1 : 0, scale,
            compRaw, compHalf, compQtr, compEps);
      for (Seed& s : emit) {
        ++out.refinedAccepted;
        s.branchId = branchId++;
        out.seeds.push_back(s);
      }
    } else if (acc.sawTangent[cid]) {
      if (diag)
        std::fprintf(stderr, "[SEED-DIAG] cid=%d NO-XVERSAL sawTangent=1 (deferred)\n", cid);
      // A near-tangent cluster with no transversal seed: TYPE the contact (S4-b) and keep the
      // compatibility counter. NearTangentTransversal is handed on to S4-c (never traced here);
      // Undecided → OCCT. deferredTangent stays the per-cluster count for pre-S4-b callers.
      ++out.deferredTangent;
      const NearTangentSolution& nt = acc.ntBest[cid];
      out.tangentContacts.push_back(classifyTangentContact(
          A, B, nt.u1, nt.v1, nt.u2, nt.v2, nt.point, nt.nA, nt.nB, nt.crossingSine, scale));
    } else if (diag) {
      std::fprintf(stderr, "[SEED-DIAG] cid=%d NO-SEED (no region converged, no tangent)\n", cid);
    }
  }
}

}  // namespace

// Public S4-b seeded classifier (declared in tangent_seeded.h) — delegates to the
// anonymous-namespace implementation so the S3 marcher can type its near-tangent stops
// with the SAME differential-geometry classifier the seeder uses.
TangentContact classify_tangent_contact_seeded(
    const SurfaceAdapter& A, const SurfaceAdapter& B,
    double u1, double v1, double u2, double v2,
    const Point3& P, const Dir3& nA, const Dir3& nB,
    double crossingSine, double scale) {
  return classifyTangentContact(A, B, u1, v1, u2, v2, P, nA, nB, crossingSine, scale);
}

SeedSet seed_intersection(const SurfaceAdapter& A, const SurfaceAdapter& B,
                          const SeedOptions& opts) {
  SeedSet result;
  const double scale = std::max(A.modelScale, B.modelScale);
  const double onSurfTol = opts.onSurfTol > 0 ? opts.onSurfTol : scale * 1e-7;
  // AABB-overlap prune gap: allow a small slack so a boundary-grazing crossing is not
  // pruned by finite bound looseness. Tied to scale, well below dedup radius.
  const double gap = scale * 1e-9;

  // Emit-as-leaf threshold: a patch is "small" once each param direction is below
  // `minFrac` of its domain. This — not maxDepth — sets the effective subdivision
  // RESOLUTION (the recall/cost knob): a smaller minFrac resolves finer loops and
  // separates closer branches, at more cost. maxDepth is only the hard recursion cap
  // that guarantees termination. Default 1/32 of each domain per direction.
  double minFrac = opts.minPatchFrac > 0 ? opts.minPatchFrac : (1.0 / 32.0);

  // Initial pre-split of A's domain into a coarse grid before recursion. This seeds
  // the recursion with several disjoint starting patch pairs so distinct loops that a
  // single root box would merge get independent subdivision (loop-catching recall).
  int gu = std::max(1, opts.initialGridU);
  int gv = std::max(1, opts.initialGridV);

  // ── SCALE-ADAPTIVE INITIAL SEEDING (default, no caller knob) ────────────────────
  //
  // A coarse fixed initial resolution MERGES co-resident / small transversal loops that a
  // pair of dense freeform operands host close together in parameter space — the dominant
  // SSI recall miss on general NURBS↔NURBS pairs (roadmap L2). The fix is a FINER initial
  // subdivision so the loops get independent clusters instead of being bridged into one;
  // this is why finer INITIAL seeding recovers loops a post-hoc completeness re-seed cannot
  // (the loops were already inside one covered cluster). But seeding finer EVERYWHERE would
  // waste work on the common simple poses and risk over-resolving a canonical case, so the
  // resolution ADAPTS to the pair's geometry:
  //
  //   * FREEFORM↔FREEFORM GATE. Adaptivity fires ONLY when BOTH operands are freeform
  //     (control-net-bearing → freeformSpanCount ≥ 1). An ELEMENTARY / plane / torus operand
  //     has span count 0, so any pair with one — plane∩sphere, plane∩B-spline (the S4-f
  //     completeness fixtures), sphere∩Bézier, all S1 analytic pairs — is left BYTE-IDENTICAL.
  //     This keeps the whole canonical SSI suite (elementary + mixed) and the S4-f BEFORE/
  //     AFTER seed-count contracts untouched; only the general-freeform L2 domain adapts.
  //   * DENSITY + WAVINESS STRENGTH. Among freeform↔freeform pairs, the refinement scales
  //     with the operands' density (span count = polynomial-patch tiling) and waviness
  //     (control-net oscillation) — the two ways a pair can hide close multiple loops. A
  //     single flat/low-span freeform pair (min span < 2 and no waviness) gets no change; a
  //     genuinely dense/wavy pair gets a finer grid + leaf, capped at the empirically
  //     calibrated sweet spot (initial grid ×2–3, leaf ½–¼) that lifts multi-loop recall
  //     without blowing up cost. Deterministic; maxDepth + the leaf floor bound termination.
  {
    const bool bothFreeform = A.freeformSpanCount >= 1 && B.freeformSpanCount >= 1;
    const int minSpan = std::min(A.freeformSpanCount, B.freeformSpanCount);
    const int osc = std::max(A.freeformComplexity, B.freeformComplexity);
    // A pair warrants finer seeding when BOTH operands are freeform. The two operands of a
    // general NURBS↔NURBS pair can host SEVERAL close co-resident transversal loops even when
    // each net is SMOOTH (a low-span, non-wavy freeform pair — e.g. two gently-bowed sheets or
    // two paraboloids that interpenetrate over a wide region cross in more than one loop). The
    // coarse fixed grid merges those loops into ONE cluster → one representative seed → the
    // second loop is missed (the dominant measured L2 recall gap; empirically the finer INITIAL
    // grid — not the post-hoc critic — is the only DISAGREED-safe lever that separates them,
    // because they were already inside one covered cluster). So the gate fires on ANY
    // freeform↔freeform pair; the STRENGTH still scales with density/waviness so smooth pairs
    // pay only the modest bump and canonical simple poses are not over-resolved:
    //   * ELEMENTARY / plane / torus operand ⇒ span count 0 ⇒ NOT bothFreeform ⇒ BYTE-IDENTICAL
    //     (every S1 analytic pair, plane∩sphere, plane∩B-spline S4-f fixtures, sphere∩Bézier).
    //   * base tier (any freeform↔freeform): grid ×2 / leaf ½ — the proven sweet spot.
    //   * dense/wavy tier (leaner operand ≥ 4 spans, or a clearly multi-modal net): grid ×3 /
    //     leaf ¼ — for pairs that can pack several loops per cell.
    // Both tiers are bounded by the same hard caps (grid ≤ 16, leaf ≥ 1/256); maxDepth + the
    // leaf floor still bound termination. Deterministic; no caller knob; OCCT-free.
    if (bothFreeform) {
      const bool dense = minSpan >= 4 || osc >= 6;
      const int refine  = dense ? 4 : 2;              // leaf divisor
      const int gridMul = dense ? 3 : 2;              // initial pre-split multiplier
      gu = std::min(gu * gridMul, 16);                // hard cost cap on the pre-split
      gv = std::min(gv * gridMul, 16);
      minFrac = std::max(minFrac / refine, 1.0 / 256.0);  // hard finest-leaf floor
    }
  }
  // FEATURE-ADAPTIVE INITIAL SUBDIVISION (see SeedOptions). Fires ONLY on freeform<->freeform
  // pairs (an elementary/plane/torus operand keeps span count 0 -> byte-identical uniform leaf,
  // so every S1 analytic / mixed / S4-f contract is untouched). It refines a would-be uniform
  // leaf FURTHER, toward `adaptiveMinFrac`, only where the two patch AABBs overlap deeply — a
  // near-crossing the uniform leaf would miss (the idx=43 placement miss). Additive: it only
  // adds candidate regions in feature cells; refineRegion still gates every seed.
  const bool bothFreeformAdapt = A.freeformSpanCount >= 1 && B.freeformSpanCount >= 1;
  const bool adaptive = opts.adaptiveSubdivision && bothFreeformAdapt;
  // Adaptive floor: never coarser than the uniform leaf, and no finer than a hard 1/512 cost
  // ceiling (deterministic; maxDepth still bounds recursion).
  const double adaptiveMinFrac =
      std::max(std::min(minFrac, opts.adaptiveMinFrac > 0 ? opts.adaptiveMinFrac : 1.0 / 256.0),
               1.0 / 512.0);
  const double adaptiveOverlapFrac = opts.adaptiveOverlapFrac > 0 ? opts.adaptiveOverlapFrac : 0.5;
  std::vector<CandidateRegion> candidates;
  for (int i = 0; i < gu; ++i) {
    for (int j = 0; j < gv; ++j) {
      ParamBox ba;
      ba.u0 = A.domain.u0 + A.domain.du() * (double(i) / gu);
      ba.u1 = A.domain.u0 + A.domain.du() * (double(i + 1) / gu);
      ba.v0 = A.domain.v0 + A.domain.dv() * (double(j) / gv);
      ba.v1 = A.domain.v0 + A.domain.dv() * (double(j + 1) / gv);
      subdivide(A, B, ba, B.domain, 0, opts.maxDepth,
                minFrac, minFrac, minFrac, minFrac, gap, candidates,
                adaptive, adaptiveMinFrac, adaptiveOverlapFrac);
    }
  }
  result.candidateRegions = static_cast<int>(candidates.size());

  // Cluster candidate regions into connected branches BEFORE refining. The param
  // epsilon is ~2 leaf-patch extents per direction — generous enough that
  // contiguously-tiled regions on one branch chain despite non-uniform leaf sizes and
  // param-track kinks, while distinct branches (separated by many pruned patch widths)
  // stay separate. Periodic seams (angular u/v) are bridged so a loop across the seam
  // is one branch.
  const double epsFrac = 2.0 * minFrac;
  const DedupParams dp{
      epsFrac * A.domain.du(), epsFrac * A.domain.dv(),
      epsFrac * B.domain.du(), epsFrac * B.domain.dv(),
      A.uPeriod, A.vPeriod, B.uPeriod, B.vPeriod};
  int numClusters = 0;
  const std::vector<int> cluster = clusterRegions(candidates, dp, numClusters);

  // S4-a: detect coincident OVERLAP patches PER CLUSTER, before refining. A cluster whose
  // representative candidate coincides with the other surface over a patch (point-on-both
  // + normals aligned across an interior grid) is a shared 2D locus, not a transversal
  // branch — we delimit it and SUPPRESS its seeds/march. Running the detector ONCE per
  // cluster (not per candidate) is the same "expensive work once per branch" decision the
  // refine makes. A cluster inside an already-delimited overlap is skipped; an overlap
  // that cannot be robustly delimited is recorded as Undecided (→ OCCT), never fabricated.
  const double tangentSinTol = opts.tangentSinTol;
  const auto nc = static_cast<std::size_t>(std::max(0, numClusters));
  std::vector<char> suppressed(nc, 0), clusterDecided(nc, 0);
  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const int cid = cluster[i];
    if (cid < 0 || clusterDecided[cid]) continue;
    // Skip candidates already covered by a delimited overlap (found via another cluster).
    if (insideOverlap(candidates[i], result.coincidentRegions)) {
      suppressed[cid] = 1;
      clusterDecided[cid] = 1;
      continue;
    }
    // Try this cluster's candidates until one is a coincident patch (interior grid agrees)
    // or is undecided; a transversal candidate fails step (1) on its first sample (cheap),
    // so we scan the cluster to find a well-interior representative for a real overlap.
    const CoincidentRegion cr = detectOverlap(A, B, candidates[i], onSurfTol, tangentSinTol);
    if (cr.kind == CoincidenceKind::OverlapSubRegion) {
      result.coincidentRegions.push_back(cr);
      suppressed[cid] = 1;
      clusterDecided[cid] = 1;                       // delimited → suppress this cluster
    } else if (cr.kind == CoincidenceKind::Undecided) {
      result.coincidentRegions.push_back(cr);        // suspected but not delimitable → OCCT
      clusterDecided[cid] = 1;                       // NOT suppressed: let normal seeding try
    }
    // else None: keep scanning the cluster's remaining candidates (do NOT mark decided).
  }

  refineClusters(A, B, candidates, cluster, numClusters, opts, onSurfTol, scale, suppressed, result);
  if (std::getenv("CYBERCAD_SSI_SEED_DIAG") != nullptr) {
    std::fprintf(stderr,
        "[SEED-DIAG] SUMMARY candidates=%d clusters=%d seeds=%d refinedAccepted=%d "
        "deferredTangent=%d gu=%d gv=%d minFrac=%.5f scale=%.4e\n",
        result.candidateRegions, numClusters, result.branchCount(), result.refinedAccepted,
        result.deferredTangent, gu, gv, minFrac, scale);
  }
  return result;
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::ssi

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

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
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
SurfaceAdapter freeformAdapter(ControlNet net, ParamBox domain,
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
  return freeformAdapter(std::move(net), domain, std::move(point), std::move(normal));
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
  return freeformAdapter(std::move(net), domain, std::move(point), std::move(normal));
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
  return freeformAdapter(std::move(net), domain, std::move(point), std::move(normal));
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

void subdivide(const SurfaceAdapter& A, const SurfaceAdapter& B,
               const ParamBox& ba, const ParamBox& bb, int depth,
               int maxDepth, double minFracU_A, double minFracV_A,
               double minFracU_B, double minFracV_B, double gap,
               std::vector<CandidateRegion>& out) {
  const Aabb boxA = A.bound(ba);
  const Aabb boxB = B.bound(bb);
  if (!boxA.valid() || !boxB.valid()) return;
  if (aabbDisjoint(boxA, boxB, gap)) return;  // prune: no intersection in this region

  // Are both patches below the size/depth threshold? Then it is a candidate region.
  const bool aSmall = ba.du() <= minFracU_A * A.domain.du() &&
                      ba.dv() <= minFracV_A * A.domain.dv();
  const bool bSmall = bb.du() <= minFracU_B * B.domain.du() &&
                      bb.dv() <= minFracV_B * B.domain.dv();
  if (depth >= maxDepth || (aSmall && bSmall)) {
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
    auto [a0, a1] = halves(ba);
    subdivide(A, B, a0, bb, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out);
    subdivide(A, B, a1, bb, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out);
  } else {
    auto [b0, b1] = halves(bb);
    subdivide(A, B, ba, b0, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out);
    subdivide(A, B, ba, b1, depth + 1, maxDepth, minFracU_A, minFracV_A, minFracU_B, minFracV_B, gap, out);
  }
}

// ── refine one candidate region with least_squares ─────────────────────────────
//
// Returns true and fills `seed` on a converged, on-both-surfaces, transversal
// result; returns false (and sets `nearTangent`) when the region is near-tangent /
// degenerate (→ deferredTangent) or the refine simply did not converge.
bool refineRegion(const SurfaceAdapter& A, const SurfaceAdapter& B,
                  const CandidateRegion& reg, const SeedOptions& opts,
                  double onSurfTol, Seed& seed, bool& nearTangent) {
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
  if (sinAngle < opts.tangentSinTol) { nearTangent = true; return false; }

  seed.u1 = c[0]; seed.v1 = c[1]; seed.u2 = c[2]; seed.v2 = c[3];
  seed.point = pa;
  seed.onSurfResidual = gapDist;   // A/B agree at pa≈pb; gap is the on-both residual
  seed.crossingSine = sinAngle;
  return true;
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

std::vector<int> clusterRegions(const std::vector<CandidateRegion>& regs,
                                const DedupParams& p, int& numClusters) {
  const int n = static_cast<int>(regs.size());
  std::vector<int> label(n, -1);
  numClusters = 0;
  if (n == 0) return label;
  DisjointSet ds(n);
  for (int i = 0; i < n; ++i)
    for (int j = i + 1; j < n; ++j)
      if (paramBoxesAdjacent(regs[i].a, regs[j].a, p.epsUA, p.epsVA, p.uPerA, p.vPerA) &&
          paramBoxesAdjacent(regs[i].b, regs[j].b, p.epsUB, p.epsVB, p.uPerB, p.vPerB))
        ds.unite(i, j);
  std::vector<int> rootLabel(n, -1);
  for (int i = 0; i < n; ++i) {
    const int root = ds.find(i);
    if (rootLabel[root] < 0) rootLabel[root] = numClusters++;
    label[i] = rootLabel[root];
  }
  return label;
}

// Refine ONE representative seed per branch cluster and append the accepted seeds
// (and deferred-tangent count) to `out`. For each cluster, try its candidate regions
// until one refines to an on-both-surfaces TRANSVERSAL point, keeping the tightest;
// a cluster that only ever ill-conditions (near-tangent at the solution) becomes a
// deferred-to-S4 gap; a cluster where no region converges to a crossing is a refine
// miss (dropped, never faked). Running the expensive least_squares ≈ once per branch
// (not once per candidate region) is the key performance decision.
void refineClusters(const SurfaceAdapter& A, const SurfaceAdapter& B,
                    const std::vector<CandidateRegion>& regs,
                    const std::vector<int>& cluster, int numClusters,
                    const SeedOptions& opts, double onSurfTol, SeedSet& out) {
  const auto k = static_cast<std::size_t>(std::max(0, numClusters));
  std::vector<Seed> best(k);
  std::vector<char> haveSeed(k, 0), sawTangent(k, 0);
  for (std::size_t i = 0; i < regs.size(); ++i) {
    const int cid = cluster[i];
    if (cid < 0) continue;
    Seed s;
    bool nearTangent = false;
    if (refineRegion(A, B, regs[i], opts, onSurfTol, s, nearTangent)) {
      if (!haveSeed[cid] || s.onSurfResidual < best[cid].onSurfResidual) {
        best[cid] = s;
        haveSeed[cid] = 1;
      }
    } else if (nearTangent) {
      sawTangent[cid] = 1;
    }
  }
  int branchId = 0;
  for (int cid = 0; cid < numClusters; ++cid) {
    if (haveSeed[cid]) {
      ++out.refinedAccepted;
      best[cid].branchId = branchId++;
      out.seeds.push_back(best[cid]);
    } else if (sawTangent[cid]) {
      ++out.deferredTangent;  // seen but not safely seeded → S4 (honest, not faked)
    }
  }
}

}  // namespace

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
  // that guarantees termination. Default 1/64 of each domain per direction.
  const double minFrac = opts.minPatchFrac > 0 ? opts.minPatchFrac : (1.0 / 32.0);

  // Initial pre-split of A's domain into a coarse grid before recursion. This seeds
  // the recursion with several disjoint starting patch pairs so distinct loops that a
  // single root box would merge get independent subdivision (loop-catching recall).
  const int gu = std::max(1, opts.initialGridU);
  const int gv = std::max(1, opts.initialGridV);
  std::vector<CandidateRegion> candidates;
  for (int i = 0; i < gu; ++i) {
    for (int j = 0; j < gv; ++j) {
      ParamBox ba;
      ba.u0 = A.domain.u0 + A.domain.du() * (double(i) / gu);
      ba.u1 = A.domain.u0 + A.domain.du() * (double(i + 1) / gu);
      ba.v0 = A.domain.v0 + A.domain.dv() * (double(j) / gv);
      ba.v1 = A.domain.v0 + A.domain.dv() * (double(j + 1) / gv);
      subdivide(A, B, ba, B.domain, 0, opts.maxDepth,
                minFrac, minFrac, minFrac, minFrac, gap, candidates);
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
  refineClusters(A, B, candidates, cluster, numClusters, opts, onSurfTol, result);
  return result;
}

#endif  // CYBERCAD_HAS_NUMSCI

}  // namespace cybercad::native::ssi

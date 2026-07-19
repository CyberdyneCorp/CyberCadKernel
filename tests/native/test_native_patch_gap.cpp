// test_native_patch_gap.cpp — the SOUNDNESS contract for patchGapBound.
//
// This bound exists so a descent stop can certify "these two patches agree to tolerance"
// WITHOUT sampling. The property under test is one-sided and absolute: it must never
// UNDER-estimate the true maximum distance over the patch. An under-estimate would let a
// caller suppress seeds across a genuine tangential contact, i.e. silently lose geometry.
//
// It also pins the two adversaries that killed the cheaper alternatives:
//   * the r²/2R corner construction, where an interior sample grid is non-conservative by 4×;
//   * the quartic contact, which shows why this bound must NEVER be used per-cell without a
//     root-level precondition (see patch_gap.h).
#include "native/ssi/patch_gap.h"
#include "native/ssi/seeding.h"

#include "harness.h"

#include <cmath>
#include <cstdio>
#include <limits>
#include <random>
#include <vector>

namespace ssi = cybercad::native::ssi;
namespace nmath = cybercad::native::math;
using cybercad::native::math::Point3;

namespace {

const double kPi = 3.14159265358979323846;

// A bicubic Bézier net over [-1,1]^2 whose height is f(x,y).
template <typename F>
ssi::ControlNet netFrom(F f, int n = 4) {
  ssi::ControlNet net;
  net.nRows = n;
  net.nCols = n;
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      const double x = -1.0 + 2.0 * i / (n - 1), y = -1.0 + 2.0 * j / (n - 1);
      net.poles.push_back({x, y, f(x, y)});
    }
  return net;
}

// Evaluate a Bézier patch from its net at normalized (s,t) by repeated de Casteljau —
// independent of the bound's own machinery, so it is a fair oracle.
Point3 evalNet(const ssi::ControlNet& net, double s, double t) {
  std::vector<Point3> col(static_cast<std::size_t>(net.nRows));
  for (int i = 0; i < net.nRows; ++i) {
    std::vector<Point3> row(static_cast<std::size_t>(net.nCols));
    for (int j = 0; j < net.nCols; ++j) row[static_cast<std::size_t>(j)] = net.pole(i, j);
    for (int k = 1; k < net.nCols; ++k)
      for (int j = 0; j + k < net.nCols; ++j)
        row[static_cast<std::size_t>(j)] = ssi::detail::lerpP(row[static_cast<std::size_t>(j)],
                                                              row[static_cast<std::size_t>(j) + 1], t);
    col[static_cast<std::size_t>(i)] = row[0];
  }
  for (int k = 1; k < net.nRows; ++k)
    for (int i = 0; i + k < net.nRows; ++i)
      col[static_cast<std::size_t>(i)] = ssi::detail::lerpP(col[static_cast<std::size_t>(i)],
                                                            col[static_cast<std::size_t>(i) + 1], s);
  return col[0];
}

// TRUE max distance between corresponding points of the two patches under the affine box map
// — exactly the correspondence patchGapBound bounds, so this is the right oracle. (A discrete
// nearest-point oracle was tried and rejected: it OVER-estimates by up to the sample spacing,
// which made it the unsound side of the comparison rather than the bound.)
double trueMaxGap(const ssi::ControlNet& A, const ssi::ParamBox& da, const ssi::ParamBox& ba,
                  const ssi::ControlNet& B, const ssi::ParamBox& db, const ssi::ParamBox& bb,
                  int n = 60) {
  double worst = 0.0;
  for (int i = 0; i <= n; ++i)
    for (int j = 0; j <= n; ++j) {
      const double fs = double(i) / n, ft = double(j) / n;
      const double ua = (ba.u0 + (ba.u1 - ba.u0) * fs - da.u0) / da.du();
      const double va = (ba.v0 + (ba.v1 - ba.v0) * ft - da.v0) / da.dv();
      const double ub = (bb.u0 + (bb.u1 - bb.u0) * fs - db.u0) / db.du();
      const double vb = (bb.v0 + (bb.v1 - bb.v0) * ft - db.v0) / db.dv();
      worst = std::max(worst, nmath::distance(evalNet(A, ua, va), evalNet(B, ub, vb)));
    }
  return worst;
}

const ssi::ParamBox kUnit{0.0, 1.0, 0.0, 1.0};

}  // namespace

// ── SOUNDNESS: the bound must never under-estimate, over many random sub-boxes ────────
CC_TEST(patch_gap_bound_is_never_below_the_true_gap) {
  std::mt19937 rng(20260719);
  std::uniform_real_distribution<double> u01(0.0, 1.0);

  // Eight pair families: identical, offset, tilted, bumped, saddle, and mixed combinations.
  const std::vector<ssi::ControlNet> nets = {
      netFrom([](double x, double y) { return 0.35 * (x * x + y * y); }),
      netFrom([](double x, double y) { return 0.35 * (x * x + y * y) + 0.01; }),
      netFrom([](double x, double y) { return 0.35 * (x * x - y * y); }),
      netFrom([](double x, double y) { return 0.2 * x + 0.1 * y; }),
      netFrom([](double, double) { return 0.0; }),
      netFrom([](double x, double y) { return 0.5 * std::sin(1.2 * x) * std::cos(0.9 * y); }),
  };

  int checked = 0, violations = 0;
  double worstRatio = 0.0;
  for (std::size_t a = 0; a < nets.size(); ++a)
    for (std::size_t b = a; b < nets.size(); ++b)
      for (int trial = 0; trial < 12; ++trial) {
        // random sub-box, same in both parameter spaces (the affine map is the identity here)
        double u0 = u01(rng), u1 = u01(rng), v0 = u01(rng), v1 = u01(rng);
        if (u1 < u0) std::swap(u0, u1);
        if (v1 < v0) std::swap(v0, v1);
        if (u1 - u0 < 1e-3 || v1 - v0 < 1e-3) continue;
        const ssi::ParamBox box{u0, u1, v0, v1};

        const double bound = ssi::patchGapBound(nets[a], kUnit, box, nets[b], kUnit, box);
        const double truth = trueMaxGap(nets[a], kUnit, box, nets[b], kUnit, box);
        ++checked;
        // The contract. A tiny absolute slack absorbs float noise when both are ~0.
        if (bound + 1e-12 < truth) ++violations;
        if (truth > 1e-9) worstRatio = std::max(worstRatio, bound / truth);
      }

  std::printf("[patch-gap] %d sub-boxes checked, %d violations, worst overestimate %.2fx\n",
              checked, violations, worstRatio);
  CC_CHECK(checked > 200);
  CC_CHECK(violations == 0);   // SOUND: never below the truth
  CC_CHECK(worstRatio < 8.0);  // and not uselessly loose
}

// ── ADVERSARY 1: the r²/2R corner construction an interior grid gets wrong by 4× ──────
// A tangency's gap is MAXIMAL AT THE CORNER, so an interior sample grid systematically
// under-reads it. The bound must see the corner gap; a 3×3 interior grid must not.
CC_TEST(patch_gap_sees_the_corner_gap_an_interior_grid_misses) {
  // Two paraboloids sharing a tangent plane at the origin, differing in curvature so the
  // gap grows as r^2 and peaks at the patch corner.
  const auto A = netFrom([](double x, double y) { return 0.0 * x * y; });          // flat
  const auto B = netFrom([](double x, double y) { return 0.25 * (x * x + y * y); });

  const ssi::ParamBox full{0.0, 1.0, 0.0, 1.0};
  const double bound = ssi::patchGapBound(A, kUnit, full, B, kUnit, full);
  const double truth = trueMaxGap(A, kUnit, full, B, kUnit, full);

  // What a 1/4..3/4 interior 3x3 grid would have read — the disqualified predicate.
  double sampled = 0.0;
  for (int i = 1; i <= 3; ++i)
    for (int j = 1; j <= 3; ++j) {
      const double s = i / 4.0, t = j / 4.0;
      sampled = std::max(sampled, nmath::distance(evalNet(A, s, t), evalNet(B, s, t)));
    }

  std::printf("[patch-gap] corner adversary: bound=%.4f truth=%.4f interior3x3=%.4f (ratio %.2fx)\n",
              bound, truth, sampled, truth / std::max(sampled, 1e-12));
  CC_CHECK(bound >= truth - 1e-12);      // sound
  CC_CHECK(sampled < truth);             // the interior grid genuinely under-reads
  CC_CHECK(truth / sampled >= 1.9);      // by 2.0x on THIS construction (see note)
  // NOTE ON THE FACTOR. The investigation that disqualified interior sampling measured 4x on an
  // exact biquadratic paraboloid, where the gap is purely r^2/(2R). This net is a bicubic whose
  // POLES sit at the paraboloid heights, so the surface it defines is not that paraboloid and
  // the corner-vs-interior ratio comes out at 2.0x. The direction and the disqualification are
  // the same; only the magnitude is construction-dependent, so 1.9x is asserted, not 4x.
}

// ── ADVERSARY 2: why this must never be used per-cell — the quartic contact ───────────
// An order-4 contact whose ROOT gap is far outside tolerance still looks coincident once
// restricted to a small cell, because the gap falls as h^4. This test does not assert a
// defect; it PINS the property that motivates the mandatory root-level precondition, so
// nobody later "optimises" that precondition away.
CC_TEST(patch_gap_shrinks_on_a_subcell_of_a_genuine_tangency) {
  const auto flat = netFrom([](double, double) { return 0.0; }, 5);
  const auto quartic = netFrom([](double x, double y) { return 0.5 * (x * x * x * x + y * y * y * y); }, 5);

  const ssi::ParamBox full{0.0, 1.0, 0.0, 1.0};
  const double atRoot = ssi::patchGapBound(flat, kUnit, full, quartic, kUnit, full);

  // A cell near the tangent point, where the two surfaces are nearly indistinguishable.
  const ssi::ParamBox cell{0.45, 0.55, 0.45, 0.55};
  const double atCell = ssi::patchGapBound(flat, kUnit, cell, quartic, kUnit, cell);

  std::printf("[patch-gap] quartic: root=%.6f  centre-cell=%.3e  (ratio %.0fx)\n",
              atRoot, atCell, atRoot / std::max(atCell, 1e-300));
  CC_CHECK(atRoot > 0.05);              // at the ROOT it is plainly not coincident
  CC_CHECK(atCell < atRoot * 0.25);     // yet a small cell reads far smaller — measured 6x here
  // The investigation's exact quartic contact shrank ~1300x root-to-cell (864x tol -> 0.667x
  // tol). This bicubic-net approximation only reaches 6x, because a Bezier through paraboloid
  // POLE heights is not a true order-4 contact. The PROPERTY is what matters and it holds: the
  // bound falls sharply on a sub-cell of a tangency, so a per-cell certificate without a
  // root-level precondition would certify a contact that is plainly not coincident at the root.
  // => a per-cell certificate WITHOUT the root precondition would swallow this contact.
}

// ── REFUSAL: anything the theorem does not cover must return +infinity, not a guess ───
CC_TEST(patch_gap_refuses_what_it_cannot_prove) {
  const auto a4 = netFrom([](double x, double y) { return 0.3 * (x * x + y * y); }, 4);
  const auto a5 = netFrom([](double x, double y) { return 0.3 * (x * x + y * y); }, 5);
  const ssi::ParamBox full{0.0, 1.0, 0.0, 1.0};

  // Unequal net sizes: no degree elevation is attempted, so it must refuse.
  CC_CHECK(std::isinf(ssi::patchGapBound(a4, kUnit, full, a5, kUnit, full)));
  // Empty net: refuse.
  ssi::ControlNet empty;
  CC_CHECK(std::isinf(ssi::patchGapBound(empty, kUnit, full, a4, kUnit, full)));
}

// ── IDENTITY: an exact copy must bound to (near) zero, and an offset copy to the offset ──
CC_TEST(patch_gap_is_tight_on_coincident_copies) {
  const auto A = netFrom([](double x, double y) { return 0.35 * (x * x + y * y); });
  const ssi::ParamBox full{0.0, 1.0, 0.0, 1.0};

  CC_CHECK(ssi::patchGapBound(A, kUnit, full, A, kUnit, full) < 1e-12);  // exact copy

  for (const double dz : {1e-9, 1e-6, 1e-3}) {
    ssi::ControlNet B = A;
    for (auto& p : B.poles) p.z += dz;
    const double bound = ssi::patchGapBound(A, kUnit, full, B, kUnit, full);
    // A pure translation is reproduced exactly by the pole differences.
    CC_CHECK(std::fabs(bound - dz) < 1e-12 + 1e-9 * dz);
  }
}

// ── THE ADAPTER HOOK: exposed only where the proof holds ──────────────────────────────
// patchGapBound needs real poles, which the adapter's `bound` closure captures privately.
// SurfaceAdapter::bezierNet exposes them, but ONLY under the conditions the convex-hull proof
// requires. Getting this gate wrong is how a rational or multi-span surface would silently get
// an unsound bound, so it is pinned here rather than left to the factories.
CC_TEST(adapter_exposes_a_bezier_net_only_where_the_proof_holds) {
  std::vector<Point3> poles;
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) poles.push_back({double(i), double(j), 0.1 * i * j});

  // (1) A Bezier surface IS a single-span non-rational patch — the net must be exposed,
  //     and it must be the actual poles, not a copy of something else.
  {
    const auto A = ssi::makeBezierAdapter(poles, 4, 4);
    CC_CHECK(A.hasBezierNet);
    CC_CHECK(A.bezierNet.nRows == 4 && A.bezierNet.nCols == 4);
    CC_CHECK(A.bezierNet.poles.size() == poles.size());
    double worst = 0.0;
    for (std::size_t k = 0; k < poles.size(); ++k)
      worst = std::max(worst, nmath::distance(A.bezierNet.poles[k], poles[k]));
    CC_CHECK(worst == 0.0);
    // ...and it is usable: a surface against itself bounds to zero.
    CC_CHECK(ssi::patchGapBound(A.bezierNet, A.domain, A.domain,
                                A.bezierNet, A.domain, A.domain) < 1e-12);
  }

  // (2) A single-span B-spline (nPoles == degree+1) is the same patch by another name.
  {
    const std::vector<double> kn = {0, 0, 0, 0, 1, 1, 1, 1};
    const auto B = ssi::makeBSplineAdapter(3, 3, poles, 4, 4, kn, kn);
    CC_CHECK(B.hasBezierNet);
  }

  // (3) A MULTI-SPAN B-spline must refuse — splitting it into Bezier patches needs Boehm
  //     knot insertion, which is not implemented, so no net may be handed out.
  {
    std::vector<Point3> p5;
    for (int i = 0; i < 5; ++i)
      for (int j = 0; j < 4; ++j) p5.push_back({double(i), double(j), 0.0});
    const std::vector<double> kuMulti = {0, 0, 0, 0, 0.5, 1, 1, 1, 1};  // 5 poles, deg 3 → 2 spans
    const std::vector<double> kv = {0, 0, 0, 0, 1, 1, 1, 1};
    const auto C = ssi::makeBSplineAdapter(3, 3, p5, 5, 4, kuMulti, kv);
    CC_CHECK(!C.hasBezierNet);
  }

  // (4) RATIONAL must refuse. The projected poles are a sound AABB hull, but the difference
  //     of two rationals is not a Bezier of pole differences — the proof does not apply.
  {
    const std::vector<double> kn = {0, 0, 0, 0, 1, 1, 1, 1};
    std::vector<double> w(poles.size(), 1.0);
    w[5] = 2.5;  // genuinely rational
    const auto D = ssi::makeNurbsAdapter(3, 3, poles, w, 4, 4, kn, kn);
    CC_CHECK(!D.hasBezierNet);
  }

  // (5) ELEMENTARY surfaces have no control net at all.
  {
    nmath::Plane pl{nmath::Ax3{{0, 0, 0}, nmath::Dir3{1, 0, 0}, nmath::Dir3{0, 1, 0}, nmath::Dir3{0, 0, 1}}};
    const auto E = ssi::makePlaneAdapter(pl, ssi::ParamBox{0, 1, 0, 1});
    CC_CHECK(!E.hasBezierNet);
  }
}

CC_RUN_ALL()

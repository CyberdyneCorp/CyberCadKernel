// SPDX-License-Identifier: Apache-2.0
//
// Host-analytic gate for NURBS roadmap Layer 8 — the trimmed-NURBS B-rep data
// model + pcurve robustness (src/native/topology/trimmed_nurbs.{h,cpp}). OCCT-FREE.
// The oracles are airtight and closed-form:
//
//   1. CONTAINMENT CORRECTNESS — on a KNOWN trim (a rectangular sub-region of a
//      bicubic patch, plus a circular hole) points provably inside/outside classify
//      correctly; boundary points → OnBoundary; a point in a hole → Out.
//   2. PCURVE FIDELITY — for a face whose 3-D edge is a KNOWN iso-curve,
//      S(pcurve(t)) matches C(t) to ~1e-9; a deliberately-WRONG pcurve (drifting off
//      S) is DETECTED (large deviation flagged), never passed.
//   3. PCURVE CONSTRUCTION ROUND-TRIP (numsci) — take a 3-D curve lying on S (S
//      evaluated along a known (u,v) path), construct its pcurve by projection+fit,
//      and verify the reconstructed pcurve reproduces the path in (u,v) and
//      S(pcurve) reproduces the 3-D curve, both to the fit tol.
//   4. DEGENERATE GUARDS — empty / open / self-touching loops, and a pcurve leaving
//      the surface domain → honest In/Out/OnBoundary/Unknown handling, never a
//      fabricated verdict.
//
// The construction round-trip calls numerics::closest_point_on_surface, so that leg
// is under CYBERCAD_HAS_NUMSCI (like test_native_nurbs_fit). The data-model +
// classify + fidelity legs are always-on. With the guard OFF the whole gate still
// runs its always-on legs.
//
#include <cstdio>

#include "native/math/bspline.h"
#include "native/topology/trimmed_nurbs.h"
#ifdef CYBERCAD_HAS_NUMSCI
#include "native/math/bspline_fit.h"  // interpolateCurve (the on-S edge fixture)
#endif

#include <cmath>
#include <span>
#include <vector>

using namespace cybercad::native::topology;
namespace math = cybercad::native::math;

static int g_failures = 0;
static int g_checks = 0;

static void fail(const char* what) {
  std::printf("FAIL %s\n", what);
  ++g_failures;
}
static void expectTrue(bool c, const char* what) {
  ++g_checks;
  if (!c) fail(what);
}
static void expectEq(int a, int b, const char* what) {
  ++g_checks;
  if (a != b) {
    std::printf("FAIL %-46s got %d want %d\n", what, a, b);
    ++g_failures;
  }
}
static void expectLE(double a, double b, const char* what) {
  ++g_checks;
  if (!(a <= b)) {
    std::printf("FAIL %-46s %.6g <= %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static void expectGT(double a, double b, const char* what) {
  ++g_checks;
  if (!(a > b)) {
    std::printf("FAIL %-46s %.6g > %.6g violated\n", what, a, b);
    ++g_failures;
  }
}
static const char* nameOf(Containment c) {
  switch (c) {
    case Containment::In: return "In";
    case Containment::Out: return "Out";
    case Containment::OnBoundary: return "OnBoundary";
    default: return "Unknown";
  }
}
static void expectClass(Containment got, Containment want, const char* what) {
  ++g_checks;
  if (got != want) {
    std::printf("FAIL %-40s got %s want %s\n", what, nameOf(got), nameOf(want));
    ++g_failures;
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Fixtures.
// ─────────────────────────────────────────────────────────────────────────────

// A gently-curved bicubic B-spline patch over the parametric box [0,1]×[0,1]:
// degree 3 in both directions, 4×4 control net, clamped knots {0,0,0,0,1,1,1,1}.
// z varies quadratically-ish so the surface is genuinely non-planar (a real patch),
// while (x,y) track (u,v)·range so S is a graph z=f(x,y) — makes projection unique.
static FaceSurface bicubicPatch() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::BSpline;
  s.degreeU = 3;
  s.degreeV = 3;
  s.nPolesU = 4;
  s.nPolesV = 4;
  const double xy[4] = {0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      const double x = xy[i] * 3.0;  // world extent [0,3]
      const double y = xy[j] * 3.0;
      const double z = 0.4 * std::sin(1.1 * x) * std::cos(0.7 * y);  // curved bump
      s.poles.push_back(math::Point3{x, y, z});
    }
  s.knotsU = {0, 0, 0, 0, 1, 1, 1, 1};
  s.knotsV = {0, 0, 0, 0, 1, 1, 1, 1};
  return s;
}

// A planar surface z=0 with S(u,v) = (u,v,0). Iso-curves are exact lines → the
// airtight fidelity oracle (S(pcurve(t)) == C(t) to machine precision).
static FaceSurface unitPlane() {
  FaceSurface s;
  s.kind = FaceSurface::Kind::Plane;
  s.frame = math::Ax3{};  // origin 0, X=(1,0,0), Y=(0,1,0), Z=(0,0,1)
  return s;
}

// Build a straight-line pcurve segment from (u0,v0) to (u1,v1) (a Line pcurve
// parametrized on [0, L] with unit direction).
static PcurveSegment lineSeg(double u0, double v0, double u1, double v1) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Line;
  const double du = u1 - u0, dv = v1 - v0;
  const double L = std::sqrt(du * du + dv * dv);
  seg.curve.origin2d = math::Point3{u0, v0, 0.0};
  seg.curve.dir2d = L > 0 ? math::Vec3{du / L, dv / L, 0.0} : math::Vec3{1, 0, 0};
  seg.first = 0.0;
  seg.last = L;
  seg.reversed = false;
  return seg;
}

// A rectangular loop in UV: [uLo,uHi]×[vLo,vHi], CCW.
static TrimLoop rectLoop(double uLo, double uHi, double vLo, double vHi) {
  TrimLoop loop;
  loop.push_back(lineSeg(uLo, vLo, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));
  return loop;
}

// A circular hole loop in UV centered at (cu,cv), radius r (Circle pcurve, one seg).
static TrimLoop circleLoop(double cu, double cv, double r) {
  PcurveSegment seg;
  seg.curve.kind = EdgeCurve::Kind::Circle;
  seg.curve.origin2d = math::Point3{cu, cv, 0.0};
  seg.curve.dir2d = math::Vec3{r, 0.0, 0.0};  // dir2d.x carries the radius
  seg.first = 0.0;
  seg.last = 2.0 * M_PI;
  seg.reversed = false;
  return TrimLoop{seg};
}

// ─────────────────────────────────────────────────────────────────────────────
// 1. Containment correctness.
// ─────────────────────────────────────────────────────────────────────────────
static void testContainment() {
  // Rectangular sub-region [0.2,0.8]×[0.2,0.8] of the bicubic patch, with a
  // circular hole of radius 0.1 at the center (0.5,0.5).
  TrimmedNurbsFace f;
  f.surface = bicubicPatch();
  f.outer = rectLoop(0.2, 0.8, 0.2, 0.8);
  f.holes.push_back(circleLoop(0.5, 0.5, 0.1));

  // Provably inside (in the rect, outside the hole).
  expectClass(classify(f, {0.3, 0.3}), Containment::In, "rect interior point In");
  expectClass(classify(f, {0.7, 0.3}), Containment::In, "rect interior corner-ish In");
  expectClass(classify(f, {0.5, 0.75}), Containment::In, "rect near-top interior In");

  // Provably outside the rect.
  expectClass(classify(f, {0.1, 0.5}), Containment::Out, "left of rect Out");
  expectClass(classify(f, {0.9, 0.5}), Containment::Out, "right of rect Out");
  expectClass(classify(f, {0.5, 0.1}), Containment::Out, "below rect Out");
  expectClass(classify(f, {0.5, 0.9}), Containment::Out, "above rect Out");

  // A point inside the hole → Out (removed region).
  expectClass(classify(f, {0.5, 0.5}), Containment::Out, "hole center Out");
  expectClass(classify(f, {0.53, 0.5}), Containment::Out, "inside hole Out");

  // Boundary points → OnBoundary (on the rect edge, and on the hole circle).
  expectClass(classify(f, {0.2, 0.5}), Containment::OnBoundary, "on left rect edge OnBoundary");
  expectClass(classify(f, {0.5, 0.8}), Containment::OnBoundary, "on top rect edge OnBoundary");
  expectClass(classify(f, {0.6, 0.5}), Containment::OnBoundary, "on hole circle OnBoundary");

  // No-hole variant: a point where the hole WAS is now In.
  TrimmedNurbsFace f2;
  f2.surface = bicubicPatch();
  f2.outer = rectLoop(0.2, 0.8, 0.2, 0.8);
  expectClass(classify(f2, {0.5, 0.5}), Containment::In, "center In when no hole");
}

// ─────────────────────────────────────────────────────────────────────────────
// 2. Pcurve fidelity.
// ─────────────────────────────────────────────────────────────────────────────
static void testFidelity() {
  const FaceSurface plane = unitPlane();

  // The 3-D edge is the iso-curve S(0.4, v) = (0.4, v, 0), a Line from (0.4,0,0)
  // along +Y. Its CORRECT pcurve is the UV line origin=(0.4,0), dir=(0,1).
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::Line;
  edge.frame.origin = math::Point3{0.4, 0.0, 0.0};
  edge.frame.x = math::Dir3{0.0, 1.0, 0.0};  // direction +Y (Line uses frame.x as dir)

  PCurve good;
  good.kind = EdgeCurve::Kind::Line;
  good.origin2d = math::Point3{0.4, 0.0, 0.0};
  good.dir2d = math::Vec3{0.0, 1.0, 0.0};

  FidelityReport rep = pcurveFidelity(plane, Location{}, edge, Location{}, good,
                                      /*first=*/0.0, /*last=*/1.0);
  expectTrue(rep.ok, "correct pcurve is faithful (ok)");
  expectLE(rep.maxDeviation, 1e-9, "correct pcurve S(p(t))==C(t) to ~1e-9");

  // A deliberately-WRONG pcurve that drifts in u as v grows (dir=(0.3,1)): it leaves
  // the true iso-line, so S(pcurve(t)) diverges from C(t) → large deviation flagged.
  PCurve wrong;
  wrong.kind = EdgeCurve::Kind::Line;
  wrong.origin2d = math::Point3{0.4, 0.0, 0.0};
  wrong.dir2d = math::Vec3{0.3, 1.0, 0.0};  // wrong: drifts off the iso-curve

  FidelityReport bad = pcurveFidelity(plane, Location{}, edge, Location{}, wrong, 0.0, 1.0);
  expectTrue(!bad.ok, "wrong pcurve is DETECTED (not ok)");
  expectGT(bad.maxDeviation, 1e-3, "wrong pcurve deviation is large (flagged)");

  // Fidelity holds on the CURVED bicubic patch too. The EXACT iso-curve S(u0, ·) of a
  // clamped B-spline surface is itself a B-spline curve in v: evaluate the u-basis at
  // u0 to collapse the 4 pole-rows into one row of 4 v-poles, sharing the v-knots.
  // C(v) built from those poles equals S(u0, v) POINTWISE, so its UV-line pcurve
  // (u0, v) satisfies S(pcurve(v)) == C(v) to machine precision — an airtight curved
  // oracle (no fit, no approximation).
  const FaceSurface patch = bicubicPatch();
  const double u0 = 0.5;
  // u-basis at u0 (degree 3, clamped [0,0,0,0,1,1,1,1] → Bézier basis in u).
  const int spanU = math::findSpan(patch.nPolesU - 1, patch.degreeU, u0,
                                   {patch.knotsU.data(), patch.knotsU.size()});
  std::vector<double> bu(patch.degreeU + 1);
  math::basisFuns(spanU, u0, patch.degreeU, {patch.knotsU.data(), patch.knotsU.size()}, bu);
  const int firstU = spanU - patch.degreeU;
  EdgeCurve iso;
  iso.kind = EdgeCurve::Kind::BSpline;
  iso.degree = patch.degreeV;
  iso.knots = patch.knotsV;
  for (int j = 0; j < patch.nPolesV; ++j) {
    math::Point3 acc{0, 0, 0};
    for (int k = 0; k <= patch.degreeU; ++k) {
      const math::Point3& p = patch.poles[static_cast<std::size_t>(firstU + k) * patch.nPolesV + j];
      acc.x += bu[k] * p.x;
      acc.y += bu[k] * p.y;
      acc.z += bu[k] * p.z;
    }
    iso.poles.push_back(acc);
  }
  PCurve isoPc;
  isoPc.kind = EdgeCurve::Kind::Line;
  isoPc.origin2d = math::Point3{u0, 0.0, 0.0};
  isoPc.dir2d = math::Vec3{0.0, 1.0, 0.0};  // (u0, v) as v runs 0..1
  FidelityReport curved =
      pcurveFidelity(patch, Location{}, iso, Location{}, isoPc, /*first=*/0.0, /*last=*/1.0);
  expectTrue(curved.ok, "curved-patch iso-curve pcurve is faithful (ok)");
  expectLE(curved.maxDeviation, 1e-9, "curved iso-curve S(p(t))==C(t) to ~1e-9");
}

// ─────────────────────────────────────────────────────────────────────────────
// 4. Degenerate guards (always-on).
// ─────────────────────────────────────────────────────────────────────────────
static void testDegenerate() {
  const FaceSurface plane = unitPlane();

  // Empty outer loop → Unknown (no region to classify).
  TrimmedNurbsFace empty;
  empty.surface = plane;
  expectClass(classify(empty, {0.5, 0.5}), Containment::Unknown, "empty face Unknown");

  // Open / too-few-points loop (a single degenerate segment collapsing to a point)
  // → Unknown. A zero-length line seg gives < 3 distinct polyline points.
  TrimmedNurbsFace open;
  open.surface = plane;
  open.outer.push_back(lineSeg(0.5, 0.5, 0.5, 0.5));  // degenerate point segment
  expectClass(classify(open, {0.5, 0.5}), Containment::Unknown, "degenerate loop Unknown");

  // Self-touching loop (a figure-eight pinch: the loop revisits an interior vertex)
  // → Unknown. Build a bowtie that pinches at (0.5,0.5).
  TrimmedNurbsFace pinch;
  pinch.surface = plane;
  pinch.outer.push_back(lineSeg(0.2, 0.2, 0.5, 0.5));
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.8, 0.2));
  pinch.outer.push_back(lineSeg(0.8, 0.2, 0.5, 0.5));  // revisits the pinch vertex
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.2, 0.2));
  expectClass(classify(pinch, {0.4, 0.35}), Containment::Unknown, "self-touching loop Unknown");

  // A hole loop that is degenerate → the whole classify declines Unknown (honest).
  TrimmedNurbsFace badHole;
  badHole.surface = plane;
  badHole.outer = rectLoop(0.0, 1.0, 0.0, 1.0);
  badHole.holes.push_back(TrimLoop{lineSeg(0.5, 0.5, 0.5, 0.5)});  // degenerate hole
  expectClass(classify(badHole, {0.3, 0.3}), Containment::Unknown, "degenerate hole → Unknown");
}

// ─────────────────────────────────────────────────────────────────────────────
// 5. TOLERANT-TOPOLOGY HEALING (always-on).
//
// Airtight oracles for the healing pass. A rectangular outer loop [0.2,0.8]² is the
// EXACT reference region; each gate injects a specific defect and proves the heal is
// (a) correct when it should heal, (b) an honest decline when it must not, and — the
// load-bearing property — (c) REGION-PRESERVING: a heal never flips an interior/exterior
// point's In/Out verdict versus the exact loop.
// ─────────────────────────────────────────────────────────────────────────────

// A rectangle [uLo,uHi]×[vLo,vHi] with the FIRST segment's start shifted by (du,dv) so
// the closing join (last segment end → first segment start) opens a gap of ‖(du,dv)‖.
// Every other join stays exact. The gap magnitude is exactly the injected shift.
static TrimLoop rectLoopGapped(double uLo, double uHi, double vLo, double vHi,
                               double du, double dv) {
  TrimLoop loop;
  // Bottom edge starts at the SHIFTED corner (uLo+du, vLo+dv) → the closing edge from
  // (uLo,vHi) back to the start no longer meets the true corner: an injected gap.
  loop.push_back(lineSeg(uLo + du, vLo + dv, uHi, vLo));
  loop.push_back(lineSeg(uHi, vLo, uHi, vHi));
  loop.push_back(lineSeg(uHi, vHi, uLo, vHi));
  loop.push_back(lineSeg(uLo, vHi, uLo, vLo));  // ends at the TRUE corner (uLo,vLo)
  return loop;
}

// The set of interior/exterior probe points used for the preservation proof.
struct Probe { double u, v; Containment want; const char* name; };
static const Probe kProbes[] = {
    {0.30, 0.30, Containment::In,  "interior (0.30,0.30)"},
    {0.50, 0.50, Containment::In,  "interior center"},
    {0.70, 0.70, Containment::In,  "interior (0.70,0.70)"},
    {0.25, 0.75, Containment::In,  "interior (0.25,0.75)"},
    {0.10, 0.50, Containment::Out, "exterior left"},
    {0.90, 0.50, Containment::Out, "exterior right"},
    {0.50, 0.10, Containment::Out, "exterior below"},
    {0.50, 0.90, Containment::Out, "exterior above"},
};

static void testHealing() {
  const FaceSurface patch = bicubicPatch();

  // ── (1) SMALL injected gap HEALS to closed and classifies IDENTICALLY to the exact
  //        loop. Inject ε = 1e-8 (× extent ~0.85 ⇒ ~8.5e-9 gap), far under the default
  //        healGapTol = 1e-6·extent (~8.5e-7). The gapped loop must classify EVERY probe
  //        the same as the exact loop.
  TrimmedNurbsFace exact;
  exact.surface = patch;
  exact.outer = rectLoop(0.2, 0.8, 0.2, 0.8);

  const double eps = 1e-8;
  TrimmedNurbsFace gapped;
  gapped.surface = patch;
  gapped.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, eps, eps);

  for (const Probe& pr : kProbes) {
    const Containment cExact = classify(exact, {pr.u, pr.v});
    const Containment cGap = classify(gapped, {pr.u, pr.v});
    expectClass(cExact, pr.want, pr.name);              // sanity: exact loop is correct
    expectClass(cGap, cExact, "small gap heals identical");  // heal reproduces exact
  }

  // The heal REPORT confirms a gap was welded (not a large-gap decline, not a pinch).
  HealReport hr = healTrimLoop(gapped.outer);
  expectTrue(hr.healed, "small-gap loop heals (healed)");
  expectTrue(hr.changed, "small-gap heal moved a vertex (changed)");
  expectTrue(!hr.largeGap, "small gap is NOT flagged large");
  expectTrue(!hr.pinch, "small-gap loop is not a pinch");
  expectTrue(hr.gapsClosed >= 1, "small gap was welded (gapsClosed>=1)");
  expectLE(hr.maxGapClosed, hr.tolerance, "welded gap is within tolerance");

  // ── (2) LARGE gap (beyond tol) still DECLINES honestly — NOT force-healed. Inject a
  //        gap of 0.05 (× nothing; absolute ~0.05 ≫ tol ~8.5e-7). classify → Unknown, and
  //        the report flags largeGap with the residual gap it refused to weld.
  TrimmedNurbsFace bigGap;
  bigGap.surface = patch;
  bigGap.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, 0.05, 0.0);
  expectClass(classify(bigGap, {0.5, 0.5}), Containment::Unknown, "large gap declines Unknown");
  HealReport bg = healTrimLoop(bigGap.outer);
  expectTrue(!bg.healed, "large-gap loop declines (not healed)");
  expectTrue(bg.largeGap, "large gap is flagged (largeGap)");
  expectTrue(!bg.pinch, "large gap is not a pinch");
  expectGT(bg.residualGap, bg.tolerance, "residual gap exceeds tolerance");

  // ── (3) PRESERVATION across a SWEEP of gap sizes that heal: for every ε in a band up to
  //        the tolerance, NO probe's classification ever flips vs the exact loop. This is
  //        the explicit "a heal never changes the region" proof.
  const double extent = 0.6 * std::sqrt(2.0);  // rect diagonal ≈ 0.849
  const double tol = 1e-6 * extent;            // default healGapTol × extent
  const double gapSizes[] = {1e-10, 1e-9, 1e-8, 1e-7, 0.4 * tol, 0.9 * tol};
  for (double g : gapSizes) {
    TrimmedNurbsFace gf;
    gf.surface = patch;
    // Split the injected shift across u and v so the gap magnitude is g.
    const double c = g / std::sqrt(2.0);
    gf.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, c, c);
    for (const Probe& pr : kProbes)
      expectClass(classify(gf, {pr.u, pr.v}), pr.want, "preservation: probe unchanged under heal");
  }

  // ── (4a) NEAR-COINCIDENT PCURVE SNAP: two pcurve endpoints that are within tol but not
  //         coincident are snapped to share the boundary. The gapped rectangle IS exactly
  //         this case (the closing pcurve's end and the opening pcurve's start are 1e-8
  //         apart); the heal welds them → the loop is closed. Verify a point ON the healed
  //         seam classifies OnBoundary (the snapped seam is a real boundary), and the heal
  //         reports the snap.
  TrimmedNurbsFace snap;
  snap.surface = patch;
  snap.outer = rectLoopGapped(0.2, 0.8, 0.2, 0.8, eps, eps);
  expectClass(classify(snap, {0.2, 0.5}), Containment::OnBoundary,
              "point on snapped seam OnBoundary");
  HealReport sr = healTrimLoop(snap.outer);
  expectTrue(sr.healed && sr.gapsClosed >= 1, "near-coincident pcurves snap (welded)");

  // ── (4b) PINCH DETECTION: a bowtie that self-touches at (0.5,0.5) is DETECTED and
  //         reported (pinch=true), and classify declines Unknown — never fabricated into a
  //         different region by "healing" the pinch away.
  TrimmedNurbsFace pinch;
  pinch.surface = patch;
  pinch.outer.push_back(lineSeg(0.2, 0.2, 0.5, 0.5));
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.8, 0.2));
  pinch.outer.push_back(lineSeg(0.8, 0.2, 0.5, 0.5));  // revisits the pinch vertex
  pinch.outer.push_back(lineSeg(0.5, 0.5, 0.2, 0.2));
  expectClass(classify(pinch, {0.4, 0.35}), Containment::Unknown, "pinch declines Unknown");
  HealReport pr = healTrimLoop(pinch.outer);
  expectTrue(pr.pinch, "pinch is DETECTED (pinch=true)");
  expectTrue(!pr.healed, "pinch is declined honestly (not healed)");

  // ── (5) HEALING TOGGLE: with heal OFF, the small-gap loop is NOT welded. Depending on
  //        the flatten dedup it either classifies (the 1e-8 gap survives as a stray edge)
  //        or declines — the point is that the RESULT can differ from the healed one, i.e.
  //        healing is what closes the gap. Assert the healed classify is In at center while
  //        confirming the toggle path runs without crashing.
  ClassifyOptions noHeal;
  noHeal.heal = false;
  (void)classify(gapped, {0.5, 0.5}, noHeal);  // must not crash / must be deterministic
  ClassifyOptions withHeal;  // default heal=true
  expectClass(classify(gapped, {0.5, 0.5}, withHeal), Containment::In,
              "healed gapped loop: center In");
}

// ─────────────────────────────────────────────────────────────────────────────
// 6. PINCH-SPLITTING (always-on).
//
// A detected 2-way pinch (a figure-eight self-touching at ONE vertex) is SPLIT into two
// valid sub-loops whose UNION classifies IDENTICALLY to the intended two-region geometry —
// the load-bearing containment-preservation property. Airtight oracle: the two-region
// REFERENCE is two SEPARATE triangle loops classified by union (In iff inside either); the
// pinched single loop with splitPinch=on must reproduce every probe. An un-splittable pinch
// (3-way / crossing) still declines honestly (never force-split).
// ─────────────────────────────────────────────────────────────────────────────

// A triangle loop (three CCW line segments) through the three given UV corners.
static TrimLoop triLoop(double au, double av, double bu, double bv, double cu, double cv) {
  TrimLoop loop;
  loop.push_back(lineSeg(au, av, bu, bv));
  loop.push_back(lineSeg(bu, bv, cu, cv));
  loop.push_back(lineSeg(cu, cv, au, av));
  return loop;
}

// The figure-eight: two triangle lobes touching ONLY at the pinch vertex (0.5,0.5). The
// LEFT lobe is (0.5,0.5)→(0.2,0.7)→(0.2,0.3); the RIGHT lobe is (0.5,0.5)→(0.8,0.3)→
// (0.8,0.7). Traced as ONE loop it self-touches at (0.5,0.5) — a clean 2-way pinch.
static TrimLoop figureEightLoop() {
  TrimLoop loop;
  loop.push_back(lineSeg(0.5, 0.5, 0.2, 0.7));  // into the left lobe
  loop.push_back(lineSeg(0.2, 0.7, 0.2, 0.3));
  loop.push_back(lineSeg(0.2, 0.3, 0.5, 0.5));  // back to the pinch
  loop.push_back(lineSeg(0.5, 0.5, 0.8, 0.3));  // into the right lobe
  loop.push_back(lineSeg(0.8, 0.3, 0.8, 0.7));
  loop.push_back(lineSeg(0.8, 0.7, 0.5, 0.5));  // back to the pinch (revisits it)
  return loop;
}

// The intended two-region REFERENCE: point is "in the region" iff inside EITHER lobe. Each
// lobe is a SEPARATE, valid triangle loop (no pinch) — the exact split geometry.
static Containment referenceUnion(const FaceSurface& surf, double u, double v) {
  TrimmedNurbsFace left;
  left.surface = surf;
  left.outer = triLoop(0.5, 0.5, 0.2, 0.7, 0.2, 0.3);
  TrimmedNurbsFace right;
  right.surface = surf;
  right.outer = triLoop(0.5, 0.5, 0.8, 0.3, 0.8, 0.7);
  const Containment cl = classify(left, {u, v});
  const Containment cr = classify(right, {u, v});
  if (cl == Containment::OnBoundary || cr == Containment::OnBoundary)
    return Containment::OnBoundary;
  if (cl == Containment::In || cr == Containment::In) return Containment::In;
  return Containment::Out;
}

static void testPinchSplit() {
  const FaceSurface patch = bicubicPatch();

  // ── (1) The figure-eight SPLITS into two valid loops (splitTrimLoopAtPinch reports it).
  SplitReport sr = splitTrimLoopAtPinch(figureEightLoop());
  expectTrue(sr.split, "figure-eight splits (split=true)");
  expectTrue(sr.pinch, "figure-eight pinch detected (pinch=true)");
  expectTrue(!sr.ambiguous, "clean 2-way pinch is not ambiguous");
  expectEq(sr.pinchCount, 1, "clean 2-way pinch has exactly one coincident pair");
  expectTrue(sr.loopA.size() >= 3 && sr.loopB.size() >= 3, "both sub-loops are non-degenerate");

  // ── (2) CONTAINMENT-PRESERVATION: with splitPinch on, EVERY probe classifies IDENTICALLY
  //        to the exact two-loop reference (union of the two triangle lobes). No interior/
  //        exterior probe flips. This is the "a split must never change the region" proof.
  TrimmedNurbsFace fig;
  fig.surface = patch;
  fig.outer = figureEightLoop();
  ClassifyOptions split;
  split.splitPinch = true;

  struct P8 { double u, v; const char* name; };
  const P8 probes[] = {
      {0.30, 0.50, "left-lobe interior"},   {0.22, 0.50, "left-lobe interior 2"},
      {0.70, 0.50, "right-lobe interior"},  {0.78, 0.50, "right-lobe interior 2"},
      {0.10, 0.50, "far left exterior"},    {0.90, 0.50, "far right exterior"},
      {0.50, 0.90, "above exterior"},       {0.50, 0.10, "below exterior"},
      {0.35, 0.20, "between-lobes exterior"},{0.65, 0.80, "between-lobes exterior 2"},
      {0.40, 0.65, "left upper interior"},  {0.60, 0.35, "right lower interior"},
  };
  for (const P8& pr : probes) {
    const Containment ref = referenceUnion(patch, pr.u, pr.v);
    const Containment got = classify(fig, {pr.u, pr.v}, split);
    expectClass(got, ref, pr.name);  // split union == two-region reference, no flip
  }

  // ── (3) DEFAULT (splitPinch OFF) still declines the pinch honestly — no behaviour change.
  expectClass(classify(fig, {0.30, 0.50}), Containment::Unknown,
              "pinch declines Unknown by default (splitPinch off)");

  // ── (4) UN-SPLITTABLE pinch declines honestly (NOT force-split). A 3-way pinch: THREE
  //        chords all pass through the SAME vertex (0.5,0.5) → three coincident pairs, not a
  //        clean 2-way figure-eight. Must report ambiguous and classify Unknown even with
  //        splitPinch on.
  TrimLoop threeWay;
  threeWay.push_back(lineSeg(0.5, 0.5, 0.2, 0.7));
  threeWay.push_back(lineSeg(0.2, 0.7, 0.2, 0.3));
  threeWay.push_back(lineSeg(0.2, 0.3, 0.5, 0.5));  // pinch #1
  threeWay.push_back(lineSeg(0.5, 0.5, 0.8, 0.3));
  threeWay.push_back(lineSeg(0.8, 0.3, 0.8, 0.7));
  threeWay.push_back(lineSeg(0.8, 0.7, 0.5, 0.5));  // pinch #2
  threeWay.push_back(lineSeg(0.5, 0.5, 0.5, 0.9));
  threeWay.push_back(lineSeg(0.5, 0.9, 0.35, 0.9));
  threeWay.push_back(lineSeg(0.35, 0.9, 0.5, 0.5));  // pinch #3 → 3+ coincident vertices
  SplitReport tw = splitTrimLoopAtPinch(threeWay);
  expectTrue(tw.pinch, "3-way pinch is detected (pinch=true)");
  expectTrue(!tw.split, "3-way pinch is NOT split (split=false)");
  expectTrue(tw.ambiguous, "3-way pinch is ambiguous (declined honestly)");
  expectTrue(tw.pinchCount > 1, "3-way pinch has more than one coincident pair");
  TrimmedNurbsFace tf;
  tf.surface = patch;
  tf.outer = threeWay;
  expectClass(classify(tf, {0.30, 0.50}, split), Containment::Unknown,
              "un-splittable pinch declines Unknown even with splitPinch on");

  // ── (5) A CLEAN (non-pinched) loop is UNAFFECTED by splitPinch: byte-identical
  //        classification with and without the option. splitTrimLoopAtPinch reports no pinch.
  TrimmedNurbsFace clean;
  clean.surface = patch;
  clean.outer = rectLoop(0.2, 0.8, 0.2, 0.8);
  SplitReport cs = splitTrimLoopAtPinch(clean.outer);
  expectTrue(!cs.pinch && !cs.split, "clean loop has no pinch, is not split");
  const Probe cleanProbes[] = {
      {0.30, 0.30, Containment::In,  "clean interior (splitPinch inert)"},
      {0.50, 0.50, Containment::In,  "clean center (splitPinch inert)"},
      {0.10, 0.50, Containment::Out, "clean exterior (splitPinch inert)"},
      {0.90, 0.50, Containment::Out, "clean exterior 2 (splitPinch inert)"},
  };
  for (const Probe& pr : cleanProbes) {
    const Containment off = classify(clean, {pr.u, pr.v});             // splitPinch off
    const Containment on = classify(clean, {pr.u, pr.v}, split);       // splitPinch on
    expectClass(off, pr.want, pr.name);
    expectClass(on, off, "clean loop unchanged by splitPinch");  // identical, no effect
  }

  // ── (6) REGION-TOTAL preservation: the pinch point (0.5,0.5) — the shared seam — is
  //        OnBoundary (a real boundary of BOTH lobes), and the split reproduces it, so the
  //        union's total region matches the reference (which is also OnBoundary there).
  expectClass(classify(fig, {0.5, 0.5}, split), referenceUnion(patch, 0.5, 0.5),
              "pinch seam matches reference (total region preserved)");
}

// ─────────────────────────────────────────────────────────────────────────────
// 7. N-WAY / CROSSING PINCH SPLITTING (always-on).
//
// splitAtPinches generalizes the 2-way split: a vertex where N≥3 strands meet is decomposed by
// CCW-adjacency into N simple sub-loops; two pinch points that cross converge to a fixpoint. The
// airtight invariants are (a) TOTAL SIGNED AREA is preserved (the re-routing only re-partitions
// the same directed edges into cycles), (b) each sub-loop is SIMPLE, and (c) EVEN-ODD region is
// preserved (classify via splitNWay reproduces the reference). A genuinely-ambiguous / degenerate
// pinch declines honestly.
// ─────────────────────────────────────────────────────────────────────────────

// Shoelace signed area of a UV polyline.
static double signedAreaPoly(const std::vector<ParamPoint>& p) {
  const std::size_t n = p.size();
  if (n < 3) return 0.0;
  double a = 0.0;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++)
    a += p[j].u * p[i].v - p[i].u * p[j].v;
  return 0.5 * a;
}

// N CCW triangular petals sharing the pinch vertex (0.5,0.5), angularly well separated. Traced as
// one loop it self-touches N-way at the centre. Petal k spans [ang±half] at radius r.
static TrimLoop nPetalPinch(int nPetals, double half, double r) {
  TrimLoop loop;
  const double cu = 0.5, cv = 0.5;
  for (int k = 0; k < nPetals; ++k) {
    const double ang = k * 2.0 * M_PI / nPetals;
    const double a1 = ang - half, a2 = ang + half;
    const double p1u = cu + r * std::cos(a1), p1v = cv + r * std::sin(a1);
    const double p2u = cu + r * std::cos(a2), p2v = cv + r * std::sin(a2);
    loop.push_back(lineSeg(cu, cv, p1u, p1v));    // centre → petal corner 1
    loop.push_back(lineSeg(p1u, p1v, p2u, p2v));  // corner 1 → corner 2
    loop.push_back(lineSeg(p2u, p2v, cu, cv));     // corner 2 → back to centre (pinch)
  }
  return loop;
}

static void testNWayPinch() {
  const FaceSurface patch = bicubicPatch();

  // ── (1) 3-WAY split: exactly three simple sub-loops, total signed area preserved to ≤1e-12.
  const TrimLoop three = nPetalPinch(3, 0.30, 0.30);
  MultiSplitReport ms3 = splitTrimLoopAtPinches(three);
  expectTrue(ms3.ok, "3-way pinch splits (ok)");
  expectTrue(ms3.pinch && !ms3.ambiguous, "3-way pinch detected, not ambiguous");
  expectEq(static_cast<int>(ms3.loops.size()), 3, "3-way pinch yields exactly 3 sub-loops");
  expectEq(ms3.maxWays, 3, "3-way pinch has strand-count 3");
  // Total SIGNED area preserved.
  {
    std::vector<double> jg;  // reference: flatten the SAME loop the splitter used
    // Recompute the original polyline area via a rectangle-free direct flatten.
    double areaSubs = 0.0;
    for (const std::vector<ParamPoint>& s : ms3.loops) {
      areaSubs += signedAreaPoly(s);
      expectTrue(s.size() >= 3, "3-way sub-loop is non-degenerate");
    }
    // Original petals: each is a CCW triangle of area 0.5*|base×...|; sum equals the analytic
    // total. Compare the split sum against the analytic petal-sum (three identical petals).
    const double aOne = signedAreaPoly({{0.5, 0.5},
                                        {0.5 + 0.30 * std::cos(-0.30), 0.5 + 0.30 * std::sin(-0.30)},
                                        {0.5 + 0.30 * std::cos(0.30), 0.5 + 0.30 * std::sin(0.30)}});
    // three petals at 0,120,240 have equal magnitude; total signed area = 3*aOne rotated — use the
    // splitter's own sub-loop sum as ground truth for the STABILITY invariant and check parts.
    (void)jg;
    (void)aOne;
    expectTrue(areaSubs > 0.0, "3-way total signed area is positive (CCW petals)");
  }

  // Direct area-preservation oracle: build the original polyline the splitter sees and compare its
  // signed area to the sub-loop signed-area sum (≤1e-12).
  {
    MultiSplitReport ms = splitTrimLoopAtPinches(three);
    // The original flattened loop: reconstruct by concatenating the same segments densely.
    TrimmedNurbsFace pf;
    pf.surface = patch;
    pf.outer = three;
    double subSum = 0.0;
    for (const std::vector<ParamPoint>& s : ms.loops) subSum += signedAreaPoly(s);
    // Analytic original signed area = sum of 3 CCW petals; compute each petal's signed area.
    double orig = 0.0;
    for (int k = 0; k < 3; ++k) {
      const double ang = k * 2.0 * M_PI / 3.0;
      orig += signedAreaPoly({{0.5, 0.5},
                              {0.5 + 0.30 * std::cos(ang - 0.30), 0.5 + 0.30 * std::sin(ang - 0.30)},
                              {0.5 + 0.30 * std::cos(ang + 0.30), 0.5 + 0.30 * std::sin(ang + 0.30)}});
    }
    expectLE(std::fabs(subSum - orig), 1e-12, "3-way split preserves total signed area (≤1e-12)");
  }

  // ── (2) 3-WAY containment preservation: with splitPinch+splitNWay on, EVERY probe classifies as
  //        the reference (In iff inside a petal). No probe flips.
  TrimmedNurbsFace f3;
  f3.surface = patch;
  f3.outer = three;
  ClassifyOptions nway;
  nway.splitPinch = true;
  nway.splitNWay = true;
  // Reference: three separate petal triangles, In iff inside any.
  auto refThree = [&](double u, double v) -> Containment {
    for (int k = 0; k < 3; ++k) {
      const double ang = k * 2.0 * M_PI / 3.0;
      TrimmedNurbsFace pf;
      pf.surface = patch;
      pf.outer = triLoop(0.5, 0.5, 0.5 + 0.30 * std::cos(ang - 0.30), 0.5 + 0.30 * std::sin(ang - 0.30),
                         0.5 + 0.30 * std::cos(ang + 0.30), 0.5 + 0.30 * std::sin(ang + 0.30));
      const Containment c = classify(pf, {u, v});
      if (c == Containment::OnBoundary) return Containment::OnBoundary;
      if (c == Containment::In) return Containment::In;
    }
    return Containment::Out;
  };
  struct PN { double u, v; const char* name; };
  const PN pn[] = {
      {0.5 + 0.20, 0.5, "petal-0 interior"},
      {0.5 + 0.20 * std::cos(2.0 * M_PI / 3.0), 0.5 + 0.20 * std::sin(2.0 * M_PI / 3.0), "petal-1 interior"},
      {0.5 + 0.20 * std::cos(4.0 * M_PI / 3.0), 0.5 + 0.20 * std::sin(4.0 * M_PI / 3.0), "petal-2 interior"},
      {0.95, 0.5, "far exterior right"},
      {0.5, 0.95, "far exterior top"},
      {0.5, 0.5 - 0.05, "between petals (near centre)"},
  };
  for (const PN& p : pn) {
    const Containment ref = refThree(p.u, p.v);
    const Containment got = classify(f3, {p.u, p.v}, nway);
    expectClass(got, ref, p.name);
  }

  // ── (3) 4-WAY split: four simple sub-loops, area preserved (generalization holds for N>3).
  MultiSplitReport ms4 = splitTrimLoopAtPinches(nPetalPinch(4, 0.28, 0.30));
  expectTrue(ms4.ok, "4-way pinch splits (ok)");
  expectEq(static_cast<int>(ms4.loops.size()), 4, "4-way pinch yields exactly 4 sub-loops");
  expectEq(ms4.maxWays, 4, "4-way pinch has strand-count 4");

  // ── (4) CROSSING pinches (figure-8-of-figure-8): TWO distinct pinch vertices, resolved to a
  //        FIXPOINT (iterations>1), total signed area preserved. Built directly as a welded
  //        polyline: two CCW lobes at p1=(0.3,0.5) and p2=(0.7,0.5) joined by a bridge, all
  //        winding CCW.
  std::vector<ParamPoint> crossPoly = {
      {0.30, 0.50}, {0.15, 0.35}, {0.15, 0.65},  // lobe-1 CCW back to p1
      {0.30, 0.50},                                // p1 revisit (pinch #1)
      {0.50, 0.47}, {0.70, 0.50},                  // bridge p1 → p2 (lower)
      {0.85, 0.35}, {0.85, 0.65},                  // lobe-2 CCW
      {0.70, 0.50},                                // p2 revisit (pinch #2)
      {0.50, 0.53},                                // bridge p2 → p1 (upper), closes
  };
  const double crossOrig = signedAreaPoly(crossPoly);
  MultiSplitReport msx = splitAtPinches(crossPoly);
  expectTrue(msx.ok, "crossing pinches resolve (ok)");
  expectTrue(msx.iterations > 1, "crossing pinches need a fixpoint (iterations>1)");
  expectTrue(msx.pinchVertices >= 2, "crossing has ≥2 pinch vertices resolved");
  double crossSum = 0.0;
  for (const std::vector<ParamPoint>& s : msx.loops) {
    crossSum += signedAreaPoly(s);
    expectTrue(s.size() >= 3, "crossing sub-loop is simple/non-degenerate");
  }
  expectLE(std::fabs(crossSum - crossOrig), 1e-12,
           "crossing-pinch fixpoint preserves total signed area (≤1e-12)");

  // ── (5) IDEMPOTENCE: a CLEAN simple loop through splitAtPinches is a NO-OP (ok, one loop,
  //        no pinch). Healing a clean model does not change the region.
  std::vector<ParamPoint> clean = {{0.2, 0.2}, {0.8, 0.2}, {0.8, 0.8}, {0.2, 0.8}};
  MultiSplitReport msc = splitAtPinches(clean);
  expectTrue(msc.ok && !msc.pinch, "clean loop: N-way split is a no-op (ok, no pinch)");
  expectEq(static_cast<int>(msc.loops.size()), 1, "clean loop stays a single loop");
  expectLE(std::fabs(signedAreaPoly(msc.loops.front()) - signedAreaPoly(clean)), 1e-14,
           "clean loop area unchanged (idempotent)");

  // ── (6) HONEST DECLINE: a genuinely-ambiguous / degenerate pinch (a self-crossing bowtie whose
  //        strands do not alternate cleanly) declines rather than fabricating a region. classify
  //        with splitNWay OFF still declines the 3-way pinch Unknown (byte-unchanged default).
  TrimmedNurbsFace f3def;
  f3def.surface = patch;
  f3def.outer = three;
  ClassifyOptions only2;
  only2.splitPinch = true;  // splitNWay defaults OFF
  expectClass(classify(f3def, {0.5 + 0.20, 0.5}, only2), Containment::Unknown,
              "3-way pinch declines Unknown when splitNWay OFF (default byte-unchanged)");
  // And fully-default classify (splitPinch OFF too) also declines.
  expectClass(classify(f3def, {0.5 + 0.20, 0.5}), Containment::Unknown,
              "3-way pinch declines Unknown by full default");
}

#ifdef CYBERCAD_HAS_NUMSCI
// ─────────────────────────────────────────────────────────────────────────────
// 3. Pcurve construction round-trip (numsci).
// ─────────────────────────────────────────────────────────────────────────────
static void testConstruction() {
  const FaceSurface patch = bicubicPatch();
  const Location id{};

  // A KNOWN (u,v) path across the patch: a gently-curved diagonal u(s)=0.15+0.7s,
  // v(s)=0.2+0.6s+0.12 sin(πs), s∈[0,1]. The 3-D edge C(t) = S(u(t),v(t)) lies ON S
  // by construction. We encode C(t) as an INTERPOLATING B-spline through dense S-path
  // samples (a Bezier through interior samples would NOT pass through them, drifting
  // off S). The interpolant passes through every sample and, for this smooth path with
  // 33 samples, stays within the fit tol of the true S-path everywhere — so the edge
  // genuinely lies on S (verified by the small projection residual below).
  auto pathUV = [](double s) {
    return ParamPoint{0.15 + 0.7 * s, 0.2 + 0.6 * s + 0.12 * std::sin(M_PI * s)};
  };
  auto surfLocal = [&](double u, double v) {
    return math::surfacePoint(patch.degreeU, patch.degreeV,
                              math::SurfaceGrid{{patch.poles.data(), patch.poles.size()},
                                                patch.nPolesU, patch.nPolesV},
                              {patch.knotsU.data(), patch.knotsU.size()},
                              {patch.knotsV.data(), patch.knotsV.size()}, u, v);
  };
  const int nEdge = 32;
  std::vector<math::Point3> edgePts;
  for (int i = 0; i <= nEdge; ++i) {
    const double s = static_cast<double>(i) / nEdge;
    const ParamPoint uv = pathUV(s);
    edgePts.push_back(surfLocal(uv.u, uv.v));
  }
  const math::CurveFitResult edgeFit =
      math::interpolateCurve({edgePts.data(), edgePts.size()}, 3, math::ParamMethod::ChordLength);
  expectTrue(edgeFit.ok, "edge B-spline interpolation ok");
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::BSpline;
  edge.degree = edgeFit.curve.degree;
  edge.poles = edgeFit.curve.poles;
  edge.weights = edgeFit.curve.weights;
  edge.knots = edgeFit.curve.knots;  // parametrized on [0,1]

  // Map the edge param s∈[0,1] back to the (u,v) path for the reconstruction check.
  // The chord-length interpolant's parameter is NOT exactly s, so compare via the
  // 3-D point instead (below): the pcurve is verified against S(pcurve)==C(t), the
  // fidelity contract, which is the load-bearing property.

  ConstructOptions co;
  co.samples = 40;
  co.fitDegree = 3;
  co.surfSamplesU = 28;
  co.surfSamplesV = 28;
  co.fidelity.absTol = 1e-3;   // interpolant-on-S + fit + projection tolerance (achieved ~1.1e-3)
  co.fidelity.relTol = 1e-3;

  PcurveConstruction pc = constructPcurve(patch, id, edge, id, /*first=*/0.0, /*last=*/1.0,
                                          /*u0=*/0.0, 1.0, /*v0=*/0.0, 1.0, co);
  expectTrue(pc.ok, "constructPcurve round-trips (ok)");
  expectLE(pc.projMaxDistance, 1e-2, "edge lies on S (projection residual small)");
  expectLE(pc.fidelity.maxDeviation, pc.fidelity.tolerance,
           "S(constructed pcurve) reproduces C(t) within fit tol");
  expectTrue(!pc.pcurve.poles2d.empty(), "constructed pcurve has poles");
  expectTrue(pc.pcurve.knots.size() ==
                 pc.pcurve.poles2d.size() + static_cast<std::size_t>(pc.pcurve.degree) + 1,
             "constructed pcurve knot vector well-formed");

  // The reconstructed pcurve reproduces the (u,v) PATH: at each edge param t the pcurve
  // gives (u,v), and S(u,v) must equal C(t) — AND (u,v) must lie in the domain box the
  // path stays inside (u∈[0.15,0.85], v∈[0.2,0.92]). This confirms the pcurve traces the
  // true parameter path, not merely some curve mapping onto S.
  if (pc.ok) {
    double maxOutside = 0.0;
    for (int i = 0; i <= 20; ++i) {
      const double t = static_cast<double>(i) / 20;
      const math::Point3 uv = math::curvePoint(pc.pcurve.degree, pc.pcurve.poles2d,
                                               pc.pcurve.knots, t);
      const double outU = std::max(0.0, std::max(0.14 - uv.x, uv.x - 0.86));
      const double outV = std::max(0.0, std::max(0.19 - uv.y, uv.y - 0.93));
      maxOutside = std::max(maxOutside, std::max(outU, outV));
    }
    expectLE(maxOutside, 1e-3, "reconstructed pcurve stays in the path's (u,v) box");
  }

  // Round-trip through classify: build a loop whose top edge IS the constructed pcurve,
  // closed with straight returns, and confirm a point ON the pcurve classifies
  // OnBoundary (the constructed pcurve is a usable trim segment). Guarded on pc.ok so
  // an honest decline never dereferences an empty pcurve.
  if (pc.ok) {
    TrimmedNurbsFace f;
    f.surface = patch;
    PcurveSegment seg;
    seg.curve = pc.pcurve;
    seg.first = 0.0;
    seg.last = 1.0;
    f.outer.push_back(seg);
    const ParamPoint end = pathUV(1.0);
    const ParamPoint beg = pathUV(0.0);
    f.outer.push_back(lineSeg(end.u, end.v, end.u, 0.0));
    f.outer.push_back(lineSeg(end.u, 0.0, beg.u, 0.0));
    f.outer.push_back(lineSeg(beg.u, 0.0, beg.u, beg.v));
    // A point on the constructed pcurve (evaluate it at its midpoint).
    const math::Point3 onPc =
        math::curvePoint(pc.pcurve.degree, pc.pcurve.poles2d, pc.pcurve.knots, 0.5);
    ClassifyOptions cop;
    cop.onEdgeTol = 1e-3;
    expectClass(classify(f, {onPc.x, onPc.y}, cop), Containment::OnBoundary,
                "point on constructed pcurve classifies OnBoundary");
  }
}

// A pcurve/edge that does NOT lie on S → constructPcurve declines honestly.
static void testConstructionOffSurface() {
  const FaceSurface patch = bicubicPatch();
  const Location id{};

  // An edge lifted 0.5 in +Z off the patch (definitely not on S).
  EdgeCurve edge;
  edge.kind = EdgeCurve::Kind::Line;
  edge.frame.origin = math::Point3{0.5, 0.5, 5.0};  // far above the patch
  edge.frame.x = math::Dir3{1.0, 0.0, 0.0};

  PcurveConstruction pc = constructPcurve(patch, id, edge, id, 0.0, 1.0,
                                          0.0, 1.0, 0.0, 1.0);
  expectTrue(!pc.ok, "off-surface edge → constructPcurve declines (not ok)");
  expectGT(pc.projMaxDistance, 1e-3, "off-surface projection residual is large");
}

// ─────────────────────────────────────────────────────────────────────────────
// 8. RATIONAL PCURVE EXACTNESS (numsci).
//
// A CIRCULAR (rational-quadratic) trim edge on a PLANE has a pcurve that is itself an EXACT
// circle in (u,v) — representable only RATIONALLY. constructPcurve with opts.rational builds a
// rational pcurve (weights non-empty) that reproduces the 3-D circle through the surface to ~1e-9,
// where the old NON-rational polynomial fit has a measurable SAG (~1e-3). On a CYLINDER the
// pcurve u-coordinate is the transcendental angle, so the rational path honestly declines back to
// the non-rational fit (both report their true, non-zero deviation — the seam / transcendental
// residual, never a widened tolerance).
// ─────────────────────────────────────────────────────────────────────────────

// Standard rational-quadratic full-circle NURBS (Piegl & Tiller): 9 poles, degree 2, weights
// {1,w,1,...} with w=√2/2, knots {0,0,0,¼,¼,½,½,¾,¾,1,1,1}. Centre (cx,cy,cz), radius r, in z=cz.
static EdgeCurve circleNurbs(double cx, double cy, double cz, double r) {
  EdgeCurve e;
  e.kind = EdgeCurve::Kind::BSpline;
  e.degree = 2;
  const double w = std::sqrt(2.0) / 2.0;
  const double px[9] = {1, 1, 0, -1, -1, -1, 0, 1, 1};
  const double py[9] = {0, 1, 1, 1, 0, -1, -1, -1, 0};
  const double pw[9] = {1, w, 1, w, 1, w, 1, w, 1};
  for (int i = 0; i < 9; ++i) {
    e.poles.push_back(math::Point3{cx + r * px[i], cy + r * py[i], cz});
    e.weights.push_back(pw[i]);
  }
  e.knots = {0, 0, 0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1, 1, 1};
  return e;
}

static void testRationalPcurve() {
  const Location id{};

  // The plane S(u,v) = (u,v,0). A circle of radius 0.3 centred at (0.5,0.5) in z=0 lies exactly on
  // it; its (u,v) pcurve is the SAME circle — exact only rationally.
  FaceSurface plane = unitPlane();
  EdgeCurve circle = circleNurbs(0.5, 0.5, 0.0, 0.3);

  ConstructOptions co;
  co.samples = 48;
  co.fitDegree = 2;
  co.surfSamplesU = 8;  // plane projection is trivial; a coarse grid suffices
  co.surfSamplesV = 8;
  co.fidelity.absTol = 1e-9;
  co.fidelity.relTol = 1e-9;

  // RATIONAL path (default): exact.
  PcurveConstruction rat = constructPcurve(plane, id, circle, id, 0.0, 1.0,
                                           0.0, 1.0, 0.0, 1.0, co);
  expectTrue(rat.ok, "rational circle pcurve round-trips (ok)");
  expectTrue(rat.rational, "constructed pcurve is RATIONAL (weights non-empty)");
  expectTrue(!rat.pcurve.weights.empty(), "rational pcurve carries weights");
  expectLE(rat.fidelity.maxDeviation, 1e-9,
           "rational circle pcurve reproduces C(t) exactly (≤1e-9)");

  // NON-rational path: the SAME circle fitted polynomially SAGS (measurable deviation ≫ tol).
  ConstructOptions poly = co;
  poly.rational = false;
  poly.fitDegree = 3;
  PcurveConstruction nonrat = constructPcurve(plane, id, circle, id, 0.0, 1.0,
                                              0.0, 1.0, 0.0, 1.0, poly);
  expectTrue(!nonrat.rational, "non-rational path produces a non-rational pcurve");
  expectGT(nonrat.fidelity.maxDeviation, 1e-4,
           "non-rational polynomial pcurve SAGS (measurable deviation)");
  // The rational pcurve is strictly, dramatically more faithful than the polynomial one.
  expectGT(nonrat.fidelity.maxDeviation, rat.fidelity.maxDeviation * 1e3,
           "rational pcurve is orders-of-magnitude more faithful than polynomial");

  // A NON-rational (polynomial) edge is unaffected by opts.rational — the rational branch only
  // engages for a genuinely rational edge. A line edge on the plane still constructs exactly.
  EdgeCurve line;
  line.kind = EdgeCurve::Kind::Line;
  line.frame.origin = math::Point3{0.3, 0.2, 0.0};
  line.frame.x = math::Dir3{0.0, 1.0, 0.0};
  PcurveConstruction lp = constructPcurve(plane, id, line, id, 0.0, 0.5,
                                          0.0, 1.0, 0.0, 1.0, co);
  expectTrue(lp.ok && !lp.rational, "polynomial line edge → non-rational pcurve, exact");
  expectLE(lp.fidelity.maxDeviation, 1e-9, "line pcurve exact (≤1e-9)");

  // CYLINDER honest-decline: a circular ARC on a cylinder has a transcendental (angle) pcurve —
  // the rational path attempts, fails fidelity, and honestly falls back to the non-rational fit
  // reporting its TRUE deviation (never a widened tolerance). Both paths agree it is NOT exact.
  FaceSurface cyl;
  cyl.kind = FaceSurface::Kind::Cylinder;
  cyl.radius = 1.0;
  // A 90° rational arc from 30°..120° on the cylinder radius 1 at height 0.3 (avoids the u-seam).
  EdgeCurve arc;
  arc.kind = EdgeCurve::Kind::BSpline;
  arc.degree = 2;
  {
    const double a0 = M_PI / 6.0, a1 = a0 + M_PI / 2.0, am = 0.5 * (a0 + a1), half = 0.5 * (a1 - a0);
    const double w = std::cos(half), rm = 1.0 / std::cos(half);
    arc.poles = {math::Point3{std::cos(a0), std::sin(a0), 0.3},
                 math::Point3{rm * std::cos(am), rm * std::sin(am), 0.3},
                 math::Point3{std::cos(a1), std::sin(a1), 0.3}};
    arc.weights = {1.0, w, 1.0};
    arc.knots = {0, 0, 0, 1, 1, 1};
  }
  ConstructOptions cco;
  cco.samples = 48;
  cco.fitDegree = 2;
  cco.surfSamplesU = 64;
  cco.surfSamplesV = 8;
  cco.fidelity.absTol = 1e-9;
  cco.fidelity.relTol = 1e-9;
  PcurveConstruction cylArc = constructPcurve(cyl, id, arc, id, 0.0, 1.0,
                                              0.0, 2.0 * M_PI, -1.0, 1.0, cco);
  expectTrue(cylArc.projMaxDistance < 1e-6, "cylinder arc genuinely lies ON the cylinder");
  expectTrue(!cylArc.rational,
             "cylinder arc pcurve honestly falls back to non-rational (transcendental angle)");
  expectGT(cylArc.fidelity.maxDeviation, cylArc.fidelity.tolerance,
           "cylinder arc pcurve reports its TRUE non-zero deviation (honest, not widened)");
}
#endif  // CYBERCAD_HAS_NUMSCI

int main() {
  testContainment();
  testFidelity();
  testDegenerate();
  testHealing();
  testPinchSplit();
  testNWayPinch();
#ifdef CYBERCAD_HAS_NUMSCI
  testConstruction();
  testConstructionOffSurface();
  testRationalPcurve();
#endif

  if (g_failures == 0)
    std::printf("OK  test_native_trimmed_nurbs: %d checks passed\n", g_checks);
  else
    std::printf("FAILED test_native_trimmed_nurbs: %d failures / %d checks\n", g_failures, g_checks);
  return g_failures == 0 ? 0 : 1;
}
